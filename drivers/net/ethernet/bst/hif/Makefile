# SPDX-License-Identifier: GPL-2.0+
#
# Copyright (C)2024Black Sesame Technologies. All Rights Reserved.
#
# Makefile for the vmac hif device drivers.

# loadable module

ccflags-y += -I$(srctree)/drivers/bst_ipc_msgbx/ipc_app_layer/posix/include -I$(srctree)/drivers/bst_ipc_msgbx/ipc_app_layer/posix \
			-I$(srctree)/drivers/bst_ipc_msgbx/include -I$(srctree)/drivers/bst_ipc_msgbx/ipc_trans_layer/include
ifneq ($(KERNELRELEASE),)
obj-$(CONFIG_BST_HIF_ETH) += bsthif.o
bsthif-objs:= fp_bd_api.o fp_library.o fp_net_driver.o fp_platform_driver.o
else
CROSS_COMPILETOOL=aarch64-linux-gnu-
MDIR=$$PWD
KDIR=$(MDIR)/../../../../..
OUT_DIR=$(KDIR)/build_out

default:
	$(MAKE) CROSS_COMPILE=${CROSS_COMPILETOOL} -C $(KDIR) M=$(MDIR)v O=$(OUT_DIR)

clean:
	rm -rf  *.ko *.o *.mod.c *.mod.o* .tmp_versions *.order *sysvers *Module.markers

endif
