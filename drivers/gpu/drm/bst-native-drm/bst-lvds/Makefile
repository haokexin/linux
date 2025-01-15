# SPDX-License-Identifier: GPL-2.0-only
ccflags-y := \
	-I $(srctree)/$(src)/../include \
	-I $(srctree)/$(src)

ccflags-y := -DUSING_ATF_CTRL

bst-lvds-$(CONFIG_DRM_BST_LVDS) = lvds_conn.o
obj-$(CONFIG_DRM_BST_LVDS) = bst-lvds.o
