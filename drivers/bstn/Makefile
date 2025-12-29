# SPDX-License-Identifier: GPL-2.0+
#
# Makefile for the Linux Bluetooth HCI device drivers.

# loadable module
# export symbol swiotlb_dma_ops in dma-mapping.c
#

EXTRA_CFLAGS +=-Wno-error=date-time
EXTRA_CFLAGS +=-Wno-date-time

_REPO_DIR_=$(dir $(abspath $(lastword $(MAKEFILE_LIST))))
_GIT_MSG_STR_=$(shell cd "$(_REPO_DIR_)" && git describe --always --dirty)
_GIT_DATE_STR_=$(shell cd "$(_REPO_DIR_)" && git log -1 --format=%cd --date=format:%Y-%m-%d\ %H:%M:%S)
EXTRA_CFLAGS += -D_GIT_MSG_='"$(_GIT_MSG_STR_)"'
EXTRA_CFLAGS += -D_GIT_DATE_='"$(_GIT_DATE_STR_)"'

ifneq ($(KERNELRELEASE),)
obj-$(CONFIG_BSTN) += bstn_driver.o
bstn_driver-y = bstn_sysfile.o bstn.o \
	bstn_dma_manager.o \
	bstn_fw_manager.o bstn_mem_manager.o \
	bstn_misc.o bstn_msg_manager.o \
	bstn_hwsem.o \
	msgbx/bstn_client.o msgbx/net_client.o msgbx/net_safety_client.o

else
CROSS_COMPILETOOL=aarch64-linux-gnu-
MDIR=$$PWD
KDIR=$(MDIR)/../..
OUT_DIR=$(KDIR)/build_out

default:
	$(MAKE) CROSS_COMPILE=${CROSS_COMPILETOOL} -C $(KDIR) M=$(MDIR)v O=$(OUT_DIR)

endif
