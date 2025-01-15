# SPDX-License-Identifier: GPL-2.0-only
ccflags-y := \
	-I $(srctree)/$(src)/../include \
	-I $(srctree)/$(src)

bst-vout-$(CONFIG_DRM_BST_VOUT) = vout_conn.o
obj-$(CONFIG_DRM_BST_VOUT) = bst-vout.o
