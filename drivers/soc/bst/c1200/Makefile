# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the Linux BST SOC drivers.
#

ccflags-y +=  -I${srctree}/drivers/bst_ipc_msgbx/include \
	-I${srctree}/drivers/bst_ipc_msgbx/ipc_trans_layer/include \
	-I${srctree}/drivers/bst_ipc_msgbx/ipc_app_layer/include \
	-I${srctree}/drivers/bst_ipc_msgbx/ipc_app_layer/posix/include

obj-y += scmi_channel.o bst_samphore.o
obj-y		+= bst_sip_smc.o
