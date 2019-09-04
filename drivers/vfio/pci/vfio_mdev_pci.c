/*
 * Copyright © 2019 Intel Corporation.
 *     Author: Liu, Yi L <yi.l.liu@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Derived from original vfio_pci.c:
 * Copyright (C) 2012 Red Hat, Inc.  All rights reserved.
 *     Author: Alex Williamson <alex.williamson@redhat.com>
 *
 * Derived from original vfio:
 * Copyright 2010 Cisco Systems, Inc.  All rights reserved.
 * Author: Tom Lyon, pugs@cisco.com
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/device.h>
#include <linux/eventfd.h>
#include <linux/file.h>
#include <linux/interrupt.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/vfio.h>
#include <linux/vgaarb.h>
#include <linux/nospec.h>
#include <linux/mdev.h>

#include "vfio_pci_private.h"

#define DRIVER_VERSION  "0.1"
#define DRIVER_AUTHOR   "Liu, Yi L <yi.l.liu@intel.com>"
#define DRIVER_DESC     "VFIO Mdev PCI - Sample driver for PCI device as a mdev"

#define VFIO_MDEV_PCI_NAME  "vfio-mdev-pci"

static char ids[1024] __initdata;
module_param_string(ids, ids, sizeof(ids), 0);
MODULE_PARM_DESC(ids, "Initial PCI IDs to add to the vfio-mdev-pci driver, format is \"vendor:device[:subvendor[:subdevice[:class[:class_mask]]]]\" and multiple comma separated entries can be specified");

static bool nointxmask;
module_param_named(nointxmask, nointxmask, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(nointxmask,
		  "Disable support for PCI 2.3 style INTx masking.  If this resolves problems for specific devices, report lspci -vvvxxx to linux-pci@vger.kernel.org so the device can be fixed automatically via the broken_intx_masking flag.");

#ifdef CONFIG_VFIO_PCI_VGA
static bool disable_vga;
module_param(disable_vga, bool, S_IRUGO);
MODULE_PARM_DESC(disable_vga, "Disable VGA resource access through vfio-mdev-pci");
#endif

static bool disable_idle_d3;
module_param(disable_idle_d3, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(disable_idle_d3,
		 "Disable using the PCI D3 low power state for idle, unused devices");

static struct pci_driver vfio_mdev_pci_driver;

struct vfio_mdev_pci_device {
	struct vfio_pci_device vdev;
	struct mdev_parent_ops ops;
	struct attribute_group *groups[2];
	struct attribute_group attr;
	atomic_t avail;
};

static ssize_t
available_instances_show(struct kobject *kobj, struct device *dev, char *buf)
{
	struct vfio_mdev_pci_device *vmdev;

	vmdev = pci_get_drvdata(to_pci_dev(dev));

	return sprintf(buf, "%d\n", atomic_read(&vmdev->avail));
}

MDEV_TYPE_ATTR_RO(available_instances);

static ssize_t device_api_show(struct kobject *kobj, struct device *dev,
		char *buf)
{
	return sprintf(buf, "%s\n", VFIO_DEVICE_API_PCI_STRING);
}

MDEV_TYPE_ATTR_RO(device_api);

static struct attribute *vfio_mdev_pci_types_attrs[] = {
	&mdev_type_attr_device_api.attr,
	&mdev_type_attr_available_instances.attr,
	NULL,
};

struct vfio_mdev_pci {
	struct vfio_pci_device *vdev;
	struct mdev_device *mdev;
};

static int vfio_mdev_pci_create(struct kobject *kobj, struct mdev_device *mdev)
{
	struct device *pdev;
	struct vfio_device *device;
	struct vfio_mdev_pci_device *vmdev;
	struct vfio_mdev_pci *pmdev;
	int ret;

	pdev = mdev_parent_dev(mdev);
	device = vfio_device_get_from_dev(pdev);
	vmdev = vfio_device_data(device);

	if (atomic_dec_if_positive(&vmdev->avail) < 0) {
		ret = -ENOSPC;
		goto out;
	}

	pr_info("%s, available instance: %d\n",
			__func__, atomic_read(&vmdev->avail));
	pmdev = kzalloc(sizeof(struct vfio_mdev_pci), GFP_KERNEL);
	if (!pmdev) {
		ret = -ENOMEM;
		goto out;
	}

	pmdev->mdev = mdev;
	pmdev->vdev = &vmdev->vdev;
	mdev_set_drvdata(mdev, pmdev);
	ret = mdev_set_iommu_device(mdev_dev(mdev), pdev);
	if (ret) {
		pr_info("%s, failed to config iommu isolation for mdev: %s on pf: %s\n",
			__func__, dev_name(mdev_dev(mdev)), dev_name(pdev));
		kfree(pmdev);
		atomic_inc(&vmdev->avail);
	}

out:
	vfio_device_put(device);
	return ret;
}

static int vfio_mdev_pci_remove(struct mdev_device *mdev)
{
	struct vfio_mdev_pci *pmdev = mdev_get_drvdata(mdev);
	struct vfio_mdev_pci_device *vmdev;

	vmdev = container_of(pmdev->vdev, struct vfio_mdev_pci_device, vdev);

	kfree(pmdev);
	atomic_inc(&vmdev->avail);
	pr_info("%s, available instance: %d\n",
			__func__, atomic_read(&vmdev->avail));
	pr_info("%s, succeeded for mdev: %s\n", __func__,
		     dev_name(mdev_dev(mdev)));

	return 0;
}

static int vfio_mdev_pci_open(struct mdev_device *mdev)
{
	struct vfio_mdev_pci *pmdev = mdev_get_drvdata(mdev);
	struct vfio_pci_device *vdev = pmdev->vdev;
	int ret = 0;

	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	mutex_lock(&vdev->reflck->lock);

	if (!vdev->refcnt) {
		ret = vfio_pci_enable(vdev);
		if (ret)
			goto error;

		vfio_spapr_pci_eeh_open(vdev->pdev);
	}
	vdev->refcnt++;
error:
	mutex_unlock(&vdev->reflck->lock);
	if (!ret)
		pr_info("Succeeded to open mdev: %s on pf: %s\n",
		dev_name(mdev_dev(mdev)), dev_name(&pmdev->vdev->pdev->dev));
	else {
		pr_info("Failed to open mdev: %s on pf: %s\n",
		dev_name(mdev_dev(mdev)), dev_name(&pmdev->vdev->pdev->dev));
		module_put(THIS_MODULE);
	}
	return ret;
}

static void vfio_mdev_pci_release(struct mdev_device *mdev)
{
	struct vfio_mdev_pci *pmdev = mdev_get_drvdata(mdev);
	struct vfio_pci_device *vdev = pmdev->vdev;

	pr_info("Release mdev: %s on pf: %s\n",
		dev_name(mdev_dev(mdev)), dev_name(&pmdev->vdev->pdev->dev));

	mutex_lock(&vdev->reflck->lock);

	if (!(--vdev->refcnt)) {
		vfio_spapr_pci_eeh_release(vdev->pdev);
		vfio_pci_disable(vdev);
	}

	mutex_unlock(&vdev->reflck->lock);

	module_put(THIS_MODULE);
}

static long vfio_mdev_pci_ioctl(struct mdev_device *mdev, unsigned int cmd,
			     unsigned long arg)
{
	struct vfio_mdev_pci *pmdev = mdev_get_drvdata(mdev);

	return vfio_pci_ioctl(pmdev->vdev, cmd, arg);
}

static int vfio_mdev_pci_mmap(struct mdev_device *mdev,
				struct vm_area_struct *vma)
{
	struct vfio_mdev_pci *pmdev = mdev_get_drvdata(mdev);

	return vfio_pci_mmap(pmdev->vdev, vma);
}

static ssize_t vfio_mdev_pci_read(struct mdev_device *mdev, char __user *buf,
			size_t count, loff_t *ppos)
{
	struct vfio_mdev_pci *pmdev = mdev_get_drvdata(mdev);

	return vfio_pci_read(pmdev->vdev, buf, count, ppos);
}

static ssize_t vfio_mdev_pci_write(struct mdev_device *mdev,
				const char __user *buf,
				size_t count, loff_t *ppos)
{
	struct vfio_mdev_pci *pmdev = mdev_get_drvdata(mdev);

	return vfio_pci_write(pmdev->vdev, (char __user *)buf, count, ppos);
}

static int vfio_pci_dummy_open(void *device_data)
{
	struct vfio_mdev_pci_device *vmdev =
		(struct vfio_mdev_pci_device *) device_data;
	pr_warn("Device %s is not viable for vfio-pci passthru, please follow"
		" vfio-mdev passthru path as it has been wrapped as mdev!!!\n",
					dev_name(&vmdev->vdev.pdev->dev));
	return -ENODEV;
}

static void vfio_pci_dummy_release(void *device_data)
{
}

long vfio_pci_dummy_ioctl(void *device_data,
		   unsigned int cmd, unsigned long arg)
{
	return 0;
}

ssize_t vfio_pci_dummy_read(void *device_data, char __user *buf,
			     size_t count, loff_t *ppos)
{
	return 0;
}

ssize_t vfio_pci_dummy_write(void *device_data, const char __user *buf,
			      size_t count, loff_t *ppos)
{
	return 0;
}

int vfio_pci_dummy_mmap(void *device_data, struct vm_area_struct *vma)
{
	return 0;
}

void vfio_pci_dummy_request(void *device_data, unsigned int count)
{
}

static const struct vfio_device_ops vfio_pci_dummy_ops = {
	.name		= "vfio-pci",
	.open		= vfio_pci_dummy_open,
	.release	= vfio_pci_dummy_release,
	.ioctl		= vfio_pci_dummy_ioctl,
	.read		= vfio_pci_dummy_read,
	.write		= vfio_pci_dummy_write,
	.mmap		= vfio_pci_dummy_mmap,
	.request	= vfio_pci_dummy_request,
};

static int vfio_mdev_pci_driver_probe(struct pci_dev *pdev,
				       const struct pci_device_id *id)
{
	struct vfio_mdev_pci_device *vmdev;
	struct vfio_pci_device *vdev;
	const struct mdev_parent_ops *ops;
	struct iommu_group *group;
	int ret;

	if (pdev->hdr_type != PCI_HEADER_TYPE_NORMAL)
		return -EINVAL;

	/*
	 * Prevent binding to PFs with VFs enabled, this too easily allows
	 * userspace instance with VFs and PFs from the same device, which
	 * cannot work.  Disabling SR-IOV here would initiate removing the
	 * VFs, which would unbind the driver, which is prone to blocking
	 * if that VF is also in use by vfio-pci or vfio-mdev-pci. Just
	 * reject these PFs and let the user sort it out.
	 */
	if (pci_num_vf(pdev)) {
		pci_warn(pdev, "Cannot bind to PF with SR-IOV enabled\n");
		return -EBUSY;
	}

	group = vfio_iommu_group_get(&pdev->dev);
	if (!group)
		return -EINVAL;

	vmdev = kzalloc(sizeof(*vmdev), GFP_KERNEL);
	if (!vmdev)
		return -ENOMEM;

	vmdev->attr.name = kasprintf(GFP_KERNEL,
				     "%04x:%04x:%04x:%04x:%06x:%02x",
				     pdev->vendor, pdev->device,
				     pdev->subsystem_vendor,
				     pdev->subsystem_device, pdev->class,
				     pdev->revision);
	if (!vmdev->attr.name) {
		kfree(vmdev);
		return -ENOMEM;
	}

	atomic_set(&vmdev->avail, 1);

	vmdev->attr.attrs = vfio_mdev_pci_types_attrs;
	vmdev->groups[0] = &vmdev->attr;

	vmdev->ops.supported_type_groups = vmdev->groups;
	vmdev->ops.create = vfio_mdev_pci_create;
	vmdev->ops.remove = vfio_mdev_pci_remove;
	vmdev->ops.open	= vfio_mdev_pci_open;
	vmdev->ops.release = vfio_mdev_pci_release;
	vmdev->ops.read = vfio_mdev_pci_read;
	vmdev->ops.write = vfio_mdev_pci_write;
	vmdev->ops.mmap = vfio_mdev_pci_mmap;
	vmdev->ops.ioctl = vfio_mdev_pci_ioctl;
	ops = &vmdev->ops;

	vdev = &vmdev->vdev;
	vdev->pdev = pdev;
	vdev->irq_type = VFIO_PCI_NUM_IRQS;
	mutex_init(&vdev->igate);
	spin_lock_init(&vdev->irqlock);
	mutex_init(&vdev->ioeventfds_lock);
	INIT_LIST_HEAD(&vdev->ioeventfds_list);
	vdev->nointxmask = nointxmask;
#ifdef CONFIG_VFIO_PCI_VGA
	vdev->disable_vga = disable_vga;
#endif
	vdev->disable_idle_d3 = disable_idle_d3;

	ret = vfio_add_group_dev(&pdev->dev, &vfio_pci_dummy_ops, vmdev);
	if (ret) {
		vfio_iommu_group_put(group, &pdev->dev);
		kfree(vmdev);
		return ret;
	}

	ret = vfio_pci_reflck_attach(vdev);
	if (ret) {
		pci_set_drvdata(pdev, NULL);
		kfree(vdev);
		return ret;
	}

	if (vfio_pci_is_vga(pdev)) {
		vga_client_register(pdev, vdev, NULL, vfio_pci_set_vga_decode);
		vga_set_legacy_decoding(pdev,
					vfio_pci_set_vga_decode(vdev, false));
	}

	vfio_pci_probe_power_state(vdev);

	if (!vdev->disable_idle_d3) {
		/*
		 * pci-core sets the device power state to an unknown value at
		 * bootup and after being removed from a driver.  The only
		 * transition it allows from this unknown state is to D0, which
		 * typically happens when a driver calls pci_enable_device().
		 * We're not ready to enable the device yet, but we do want to
		 * be able to get to D3.  Therefore first do a D0 transition
		 * before going to D3.
		 */
		vfio_pci_set_power_state(vdev, PCI_D0);
		vfio_pci_set_power_state(vdev, PCI_D3hot);
	}

	ret = mdev_register_device(&pdev->dev, ops);
	if (ret)
		pr_err("Cannot register mdev for device %s\n",
			dev_name(&pdev->dev));
	else
		pr_info("Wrap device %s as a mdev\n", dev_name(&pdev->dev));

	return ret;
}

static void vfio_mdev_pci_driver_remove(struct pci_dev *pdev)
{
	struct vfio_mdev_pci_device *vmdev;
	struct vfio_pci_device *vdev;

	mdev_unregister_device(&pdev->dev);

	vmdev = vfio_del_group_dev(&pdev->dev);
	if (!vmdev)
		return;

	vdev = &vmdev->vdev;

	vfio_pci_reflck_put(vdev->reflck);

	kfree(vdev->region);
	mutex_destroy(&vdev->ioeventfds_lock);

	if (!disable_idle_d3)
		vfio_pci_set_power_state(vdev, PCI_D0);

	kfree(vdev->pm_save);

	if (vfio_pci_is_vga(pdev)) {
		vga_client_register(pdev, NULL, NULL, NULL);
		vga_set_legacy_decoding(pdev,
				VGA_RSRC_NORMAL_IO | VGA_RSRC_NORMAL_MEM |
				VGA_RSRC_LEGACY_IO | VGA_RSRC_LEGACY_MEM);
	}

	kfree(vmdev->attr.name);
	kfree(vmdev);
}

static struct pci_driver vfio_mdev_pci_driver = {
	.name		= VFIO_MDEV_PCI_NAME,
	.id_table	= NULL, /* only dynamic ids */
	.probe		= vfio_mdev_pci_driver_probe,
	.remove		= vfio_mdev_pci_driver_remove,
	.err_handler	= &vfio_err_handlers,
};

static void __exit vfio_mdev_pci_cleanup(void)
{
	pci_unregister_driver(&vfio_mdev_pci_driver);
	vfio_pci_uninit_perm_bits();
}

static int __init vfio_mdev_pci_init(void)
{
	int ret;

	/* Allocate shared config space permision data used by all devices */
	ret = vfio_pci_init_perm_bits();
	if (ret)
		return ret;

	/* Register and scan for devices */
	ret = pci_register_driver(&vfio_mdev_pci_driver);
	if (ret)
		goto out_driver;

	vfio_pci_fill_ids(ids, &vfio_mdev_pci_driver);

	return 0;
out_driver:
	vfio_pci_uninit_perm_bits();
	return ret;
}

module_init(vfio_mdev_pci_init);
module_exit(vfio_mdev_pci_cleanup);

MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
