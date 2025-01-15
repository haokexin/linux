# SPDX-License-Identifier: GPL-2.0+
ccflags-y := \
	-I $(srctree)/include/bst/ \
	-I $(srctree)/drivers/bst_ipc_msgbx/include \
	-I $(srctree)/drivers/bst_ipc_msgbx/ipc_app_layer/include \
	-I $(srctree)/drivers/bst_ipc_msgbx/ipc_trans_layer/include \
	-I $(srctree)/$(src)/usb_bst_virt_msg \
	-I $(srctree)/$(src)/usb_bst_virt_msg/src-gen

obj-$(CONFIG_USB_BST_VIRT_DEVICE) += usb_virt_device.o

usb_virt_device-y := \
    usb_bst_virt_rx.o \
    usb_bst_virt_tx.o \
    usb_bst_pdu_kfifo.o \
	usb_bst_list.o \
	usb_bst_virt_class.o \
    usb_bst_virt_device.o

obj-y += usb_bst_virt_msg/