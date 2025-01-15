# SPDX-License-Identifier: GPL-2.0-only
ccflags-y := \
	-I $(srctree)/include/bst/ \
	-I $(srctree)/drivers/bst_ipc_msgbx/include \
	-I $(srctree)/drivers/bst_ipc_msgbx/ipc_app_layer/include \
	-I $(srctree)/drivers/bst_ipc_msgbx/ipc_trans_layer/include \
	-I $(srctree)/$(src)/coredev_hw_ipc/display-client/src-gen \
	-I $(srctree)/$(src)/firmware_cmdsets/

ccflags-y += -DBST_PLATFORM_C1200=0x1200 -DBST_C1200_MAX_1PIPE_LAYERS=4
ccflags-y += -D__DISPLAY_EVENTS_MGR__

bst-fw-msg-$(CONFIG_DRM_BST_FW_MSG) := \
	coredev_hw_ipc/display-client/src-gen/display_client.o \
	coredev_hw_ipc/display-client/src-gen/DisplayClient.o \
	coredev_hw_ipc/bst_fw_msg_core.o \
	firmware_cmdsets/bst_display_global_api.o \
	firmware_cmdsets/bst_display_dc_cmdset.o \
	firmware_cmdsets/bst_display_dp_cmdset.o \
	firmware_cmdsets/bst_display_lvds_cmdset.o \
	firmware_cmdsets/bst_display_mipi_cmdset.o \

obj-$(CONFIG_DRM_BST_FW_MSG) = bst-fw-msg.o