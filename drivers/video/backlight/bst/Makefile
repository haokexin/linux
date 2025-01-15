# SPDX-License-Identifier: GPL-2.0

ccflags-y += -I$(srctree)/include/bst

obj-$(CONFIG_BACKLIGHT_BST)	+= bst-virt-backlight.o

bst-virt-backlight-y := \
	bst_virt_bl.o \
	virtual-backlight/virt_backlight.o \
	virtual-backlight/backlight-client/backlight_client.o \
	virtual-backlight/backlight-client/BacklightClient.o
