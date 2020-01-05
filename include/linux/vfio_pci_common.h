/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * VFIO PCI API definition
 *
 * Derived from original vfio/pci/vfio_pci_private.h:
 * Copyright (C) 2012 Red Hat, Inc.  All rights reserved.
 *     Author: Alex Williamson <alex.williamson@redhat.com>
 *
 * Derived from original vfio:
 * Copyright 2010 Cisco Systems, Inc.  All rights reserved.
 * Author: Tom Lyon, pugs@cisco.com
 */

#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/irqbypass.h>
#include <linux/types.h>

#ifndef VFIO_PCI_COMMON_H
#define VFIO_PCI_COMMON_H

struct vfio_pci_irq_ctx {
	struct eventfd_ctx	*trigger;
	struct virqfd		*unmask;
	struct virqfd		*mask;
	char			*name;
	bool			masked;
	struct irq_bypass_producer	producer;
};

struct vfio_pci_device;
struct vfio_pci_region;

struct vfio_pci_regops {
	size_t	(*rw)(struct vfio_pci_device *vdev, char __user *buf,
		      size_t count, loff_t *ppos, bool iswrite);
	void	(*release)(struct vfio_pci_device *vdev,
			   struct vfio_pci_region *region);
	int	(*mmap)(struct vfio_pci_device *vdev,
			struct vfio_pci_region *region,
			struct vm_area_struct *vma);
	int	(*add_capability)(struct vfio_pci_device *vdev,
				  struct vfio_pci_region *region,
				  struct vfio_info_cap *caps);
};

struct vfio_pci_region {
	u32				type;
	u32				subtype;
	const struct vfio_pci_regops	*ops;
	void				*data;
	size_t				size;
	u32				flags;
};

struct vfio_pci_reflck {
	struct kref		kref;
	struct mutex		lock;
};

struct vfio_pci_device {
	struct pci_dev		*pdev;
	void __iomem		*barmap[PCI_STD_NUM_BARS];
	bool			bar_mmap_supported[PCI_STD_NUM_BARS];
	u8			*pci_config_map;
	u8			*vconfig;
	struct perm_bits	*msi_perm;
	spinlock_t		irqlock;
	struct mutex		igate;
	struct vfio_pci_irq_ctx	*ctx;
	int			num_ctx;
	int			irq_type;
	int			num_regions;
	struct vfio_pci_region	*region;
	u8			msi_qmax;
	u8			msix_bar;
	u16			msix_size;
	u32			msix_offset;
	u32			rbar[7];
	bool			pci_2_3;
	bool			virq_disabled;
	bool			reset_works;
	bool			extended_caps;
	bool			bardirty;
	bool			has_vga;
	bool			needs_reset;
	bool			nointx;
	bool			needs_pm_restore;
	struct pci_saved_state	*pci_saved_state;
	struct pci_saved_state	*pm_save;
	struct vfio_pci_reflck	*reflck;
	int			refcnt;
	int			ioeventfds_nr;
	struct eventfd_ctx	*err_trigger;
	struct eventfd_ctx	*req_trigger;
	struct list_head	dummy_resources_list;
	struct mutex		ioeventfds_lock;
	struct list_head	ioeventfds_list;
	bool			nointxmask;
#ifdef CONFIG_VFIO_PCI_VGA
	bool			disable_vga;
#endif
	bool			disable_idle_d3;
};

#define is_intx(vdev) (vdev->irq_type == VFIO_PCI_INTX_IRQ_INDEX)
#define is_msi(vdev) (vdev->irq_type == VFIO_PCI_MSI_IRQ_INDEX)
#define is_msix(vdev) (vdev->irq_type == VFIO_PCI_MSIX_IRQ_INDEX)
#define is_irq_none(vdev) (!(is_intx(vdev) || is_msi(vdev) || is_msix(vdev)))
#define irq_is(vdev, type) (vdev->irq_type == type)

extern const struct pci_error_handlers vfio_pci_err_handlers;

static inline bool vfio_pci_is_vga(struct pci_dev *pdev)
{
	return (pdev->class >> 8) == PCI_CLASS_DISPLAY_VGA;
}

static inline bool vfio_vga_disabled(struct vfio_pci_device *vdev)
{
#ifdef CONFIG_VFIO_PCI_VGA
	return vdev->disable_vga;
#else
	return true;
#endif
}

extern void vfio_pci_refresh_config(struct vfio_pci_device *vdev,
				bool nointxmask, bool disable_idle_d3);

extern int vfio_pci_register_dev_region(struct vfio_pci_device *vdev,
					unsigned int type, unsigned int subtype,
					const struct vfio_pci_regops *ops,
					size_t size, u32 flags, void *data);

extern int vfio_pci_set_power_state(struct vfio_pci_device *vdev,
				    pci_power_t state);
extern unsigned int vfio_pci_set_vga_decode(void *opaque, bool single_vga);
extern int vfio_pci_enable(struct vfio_pci_device *vdev);
extern void vfio_pci_disable(struct vfio_pci_device *vdev);
extern long vfio_pci_ioctl(void *device_data,
			unsigned int cmd, unsigned long arg);
extern ssize_t vfio_pci_read(void *device_data, char __user *buf,
			size_t count, loff_t *ppos);
extern ssize_t vfio_pci_write(void *device_data, const char __user *buf,
			size_t count, loff_t *ppos);
extern int vfio_pci_mmap(void *device_data, struct vm_area_struct *vma);
extern void vfio_pci_request(void *device_data, unsigned int count);
extern void vfio_pci_fill_ids(char *ids, struct pci_driver *driver);
extern int vfio_pci_reflck_attach(struct vfio_pci_device *vdev);
extern void vfio_pci_reflck_put(struct vfio_pci_reflck *reflck);
extern void vfio_pci_probe_power_state(struct vfio_pci_device *vdev);

#endif /* VFIO_PCI_COMMON_H */
