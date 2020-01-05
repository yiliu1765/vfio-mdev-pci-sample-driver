# SPDX-License-Identifier: GPL-2.0-only

vfio-pci-common-y := vfio_pci_common.o vfio_pci_intrs.o \
		vfio_pci_rdwr.o vfio_pci_config.o
vfio-pci-common-$(CONFIG_VFIO_PCI_IGD) += vfio_pci_igd.o
vfio-pci-common-$(CONFIG_VFIO_PCI_NVLINK2) += vfio_pci_nvlink2.o

vfio-pci-y := vfio_pci.o

obj-$(CONFIG_VFIO_PCI_COMMON) += vfio-pci-common.o
obj-$(CONFIG_VFIO_PCI) += vfio-pci.o
