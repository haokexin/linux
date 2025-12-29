# SPDX-License-Identifier: GPL-2.0+
#
# Makefile for the Linux Kernel bst flexcan specific device drivers.
#
ccflags-$(CONFIG_VCAN_DEBUG) += -DVCAN_DEBUG
obj-$(CONFIG_BST_VCAN)               += bst_vcan.o
obj-$(CONFIG_BST_VCAN)               += switch0_can_gateway_client.o
obj-$(CONFIG_BST_VCAN)               += adas5_can_gateway_client.o