# SPDX-License-Identifier: GPL-2.0-only
ccflags-y := \
	-I $(srctree)/$(src)/../include \
	-I $(srctree)/$(src) \
	-I $(srctree)/$(src)/../
# ccflags-y += -DDEBUG

bst-dptx-$(CONFIG_DRM_BST_DPTX) += dptx_drv.o dptx_core.o dptx_aux.o dptx_link.o \
		      avgen.o dptx_debugfs.o dptx_intr.o dptx_dbg.o dptx_utils.o \
			  api/api.o dptx_csr.o dptx_apg.o dptx_reg.o dptx_phy.o

obj-$(CONFIG_DRM_BST_DPTX) += bst-dptx.o
