# SPDX-License-Identifier: GPL-2.0-only

vfio-pci-y := vfio_pci.o vfio_pci_common.o vfio_pci_intrs.o \
		vfio_pci_rdwr.o vfio_pci_config.o
vfio-pci-$(CONFIG_VFIO_PCI_IGD) += vfio_pci_igd.o
vfio-pci-$(CONFIG_VFIO_PCI_NVLINK2) += vfio_pci_nvlink2.o

vfio-mdev-pci-y := vfio_mdev_pci.o vfio_pci_common.o vfio_pci_intrs.o \
			vfio_pci_rdwr.o vfio_pci_config.o
vfio-mdev-pci-$(CONFIG_VFIO_PCI_IGD) += vfio_pci_igd.o
vfio-mdev-pci-$(CONFIG_VFIO_PCI_NVLINK2) += vfio_pci_nvlink2.o

obj-$(CONFIG_VFIO_PCI) += vfio-pci.o
obj-$(CONFIG_SAMPLE_VFIO_MDEV_PCI) += vfio-mdev-pci.o
