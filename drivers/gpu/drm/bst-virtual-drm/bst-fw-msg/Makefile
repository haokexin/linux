# SPDX-License-Identifier: GPL-2.0-only
ccflags-y := \
	-I $(srctree)/include/bst/ \
	-I $(srctree)/drivers/bst_ipc_msgbx/include \
	-I $(srctree)/drivers/bst_ipc_msgbx/ipc_app_layer/include \
	-I $(srctree)/drivers/bst_ipc_msgbx/ipc_trans_layer/include \
	-I $(srctree)/$(src)/bst-virt-drm-msgbx/display-client/src-gen \
	-I $(srctree)/$(src)/firmware_cmdsets/

ccflags-y += -DBST_PLATFORM_C1200=0x1200 -DBST_C1200_MAX_1PIPE_LAYERS=4
ccflags-y += -D__DISPLAY_EVENTS_MGR__
ccflags-y += -DDISPLAY_SUPPORT_SCALE

bst-fw-msg-$(CONFIG_DRM_BST_FW_MSGBX) := \
	bst-virt-drm-msgbx/display-client/src-gen/display_client.o \
	bst-virt-drm-msgbx/display-client/src-gen/DisplayClient.o \
	bst-virt-drm-msgbx/bst_fw_msg_core.o \
	firmware_cmdsets/bst_display_global_api.o \
	firmware_cmdsets/bst_display_conn_cmdset.o \
	firmware_cmdsets/bst_display_dc_cmdset.o \

obj-$(CONFIG_DRM_BST_FW_MSGBX) = bst-fw-msg.o

bst-fw-ipc-$(CONFIG_DRM_BST_FW_IPC) := \
	bst-virt-drm-ipc/bst_fw_ipc_core.o \
	firmware_cmdsets/bst_display_cmdset_api.o \
	firmware_cmdsets/bst_display_global_api.o \
	firmware_cmdsets/bst_display_conn_cmdset.o \
	firmware_cmdsets/bst_display_dc_cmdset.o \

obj-$(CONFIG_DRM_BST_FW_IPC) = bst-fw-ipc.o
