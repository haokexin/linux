# SPDX-License-Identifier: GPL-2.0-only
ccflags-y := \
	-I $(srctree)/$(src)/../include \
	-I $(srctree)/$(src) \
	-I $(srctree)/$(src)/../
bst-mipi-dsi-$(CONFIG_DRM_BST_MIPI_DSI) += mipi-dsi-bst.o \
	mipi_dsi_hal.o mipi_dsi_debugfs.o mipi_dsi_dphy.o \

obj-$(CONFIG_DRM_BST_MIPI_DSI) += bst-mipi-dsi.o