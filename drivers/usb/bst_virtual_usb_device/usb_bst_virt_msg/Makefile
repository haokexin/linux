# SPDX-License-Identifier: GPL-2.0+
ccflags-y := \
	-I $(srctree)/include/bst/ \
	-I $(srctree)/drivers/bst_ipc_msgbx/include \
	-I $(srctree)/drivers/bst_ipc_msgbx/ipc_app_layer/include \
	-I $(srctree)/drivers/bst_ipc_msgbx/ipc_trans_layer/include \
	-I $(srctree)/$(src)/src-gen

obj-$(CONFIG_USB_BST_VIRT_MSG) += usb_bst_virt_msg.o

usb_bst_virt_msg-y := \
	usb_bst_virt_msg_core.o src-gen/usb_client.o src-gen/UsbClient.o

