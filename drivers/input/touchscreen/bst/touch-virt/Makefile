# Makefile for the bst virt touchscreen drivers.
ccflags-y += -I$(srctree)/include/bst

obj-$(CONFIG_TOUCHSCREEN_VIRT_BST)      +=      ts_virt_bst.o
ts_virt_bst-y      +=      touch_virt_bst.o
ts_virt_bst-y      +=      touch_virt_bst_cdev.o
ts_virt_bst-y      +=      bst_touch_client.o
ts_virt_bst-y      +=      touchclient.o
