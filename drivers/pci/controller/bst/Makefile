# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_PCIE_BST) += bst_pcie.o
bst_pcie-objs += pcie-bst.o pcie-bst-phy.o pcie-bst-plat.o
ifdef CONFIG_PCIE_BST_HOST
bst_pcie-objs += pcie-bst-host.o 
endif
ifdef CONFIG_PCIE_BST_EP
bst_pcie-objs += pcie-bst-ep.o 
endif
ifdef CONFIG_PCIE_BST_DIAGNOSTIC
bst_pcie-objs += pcie-bst-diagnostic.o 
endif

