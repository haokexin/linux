# Makefile for the bst virt touchscreen drivers.
ccflags-y += -I$(srctree)/include/bst

obj-$(CONFIG_TOUCHSCREEN_VIRT_BST)      +=      touch_virt_bst_cdev.o
obj-$(CONFIG_TOUCHSCREEN_VIRT_BST)      +=      ts_virt_bst.o
ts_virt_bst-y      +=      touch_virt_bst.o
ts_virt_bst-y      +=      bst_touch_client.o
ts_virt_bst-y      +=      touchclient.o

# Configure whether to enable the debug function of statistics packet timestamp
# move to make menuconfig to enable
# CONFIG_TOUCHSCREEN_VIRT_BST_DEBUG_STATISTICS_DELAY_TIMESTAMPS ?= y

obj-$(CONFIG_TOUCHSCREEN_VIRT_BST_DEBUG_STATISTICS_DELAY_TIMESTAMPS)	+=	ts_virt_bst_statistic.o
ts_virt_bst_statistic-y      +=      touch_packet_statistic_cli.o
ts_virt_bst_statistic-y      +=      touch_packet_statistic_tool.o
