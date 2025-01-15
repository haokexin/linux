# SPDX-License-Identifier: GPL-2.0
#
# Makefile for bst msgbx hw
#
ccflags-y += -I$(srctree)/drivers/bst_ipc_msgbx/include \
	-I$(srctree)/drivers/bst_ipc_msgbx/ipc_hw_layer/include -I$(srctree)/drivers/bst_ipc_msgbx/ipc_trans_layer/include
obj-y += src/
#obj-$(CONFIG_BST_IPC_MSGBX)	+= posix/ipc_trans_impl.o
obj-$(CONFIG_BST_IPC_MSGBX)	+= baremetal/ipc_trans_impl.o