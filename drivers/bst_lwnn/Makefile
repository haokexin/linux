# SPDX-License-Identifier: GPL-2.0+
#
# Makefile for the Linux Bluetooth HCI device drivers.

# loadable module
# export symbol swiotlb_dma_ops in dma-mapping.c

EXTRA_CFLAGS +=-Wno-error=date-time
EXTRA_CFLAGS +=-Wno-date-time

_REPO_DIR_=$(dir $(abspath $(lastword $(MAKEFILE_LIST))))
_GIT_MSG_STR_=$(shell cd "$(_REPO_DIR_)" && git describe --always --dirty)
_GIT_DATE_STR_=$(shell cd "$(_REPO_DIR_)" && git log -1 --format=%cd --date=format:%Y-%m-%d\ %H:%M:%S)
EXTRA_CFLAGS += -D_GIT_MSG_='"$(_GIT_MSG_STR_)"'
EXTRA_CFLAGS += -D_GIT_DATE_='"$(_GIT_DATE_STR_)"'

ifneq ($(KERNELRELEASE),)
obj-$(CONFIG_BST_LWNN) += bst_lwnn_driver.o
bst_lwnn_driver-y = bst_lwnn_fw_manager.o bst_lwnn_mem_manager.o bst_lwnn_miscdev.o \
	bst_lwnn_dma_manager.o \
	bst_lwnn_msg_manager.o bst_lwnn_sysfile.o bst_lwnn.o \
	msgbx/lwnn_client.o msgbx/cvdsp0_client.o msgbx/cvdsp1_client.o \
	msgbx/cvdsp2_client.o msgbx/cvdsp3_client.o msgbx/cvdsp_safety_client.o

else
CROSS_COMPILETOOL=aarch64-linux-gnu-
MDIR=$$PWD
KDIR=$(MDIR)/../..
OUT_DIR=$(KDIR)/build

default:
	$(MAKE) CROSS_COMPILE=${CROSS_COMPILETOOL} -C $(KDIR) M=$(MDIR)v O=$(OUT_DIR)

endif
