# SPDX-License-Identifier: GPL-2.0+
ccflags-y := \
	-I $(srctree)/include/bst/ \
	-I $(srctree)/$(src)/bst-fw-msg/firmware_cmdsets/ \
	-I $(srctree)/drivers/bst_ipc_msgbx/include \
	-I $(srctree)/drivers/bst_ipc_msgbx/ipc_app_layer/include \
	-I $(srctree)/drivers/bst_ipc_msgbx/ipc_trans_layer/include \
	-I $(srctree)/$(src)/bst-fw-msg/bst-virt-drm-msgbx/display-client/src-gen

ccflags-y += -DBST_PLATFORM_C1200=0x1200 -DBST_C1200_MAX_1PIPE_LAYERS=4
ccflags-y += -D__DISPLAY_EVENTS_MGR__
ccflags-y += -DDISPLAY_SUPPORT_SCALE

bst-virtual-drm-y := \
	bst_virt_dc/virt_dc_dev.o \
	bst_virt_dc/virt_dc_component.o \
	bst_virt_dp/virt_dp_component.o \
	bst_virt_dp/virt_dp_dev.o \
	bst_virt_lvds/virt_lvds_component.o \
	bst_virt_lvds/virt_lvds_dev.o \
	bst_virt_mipi/virt_mipi_component.o \
	bst_virt_mipi/virt_mipi_dev.o \
	bst_virt_shared_conn/virt_shared_conn_dev.o \
	bst_virt_pipeline.o \
	bst_virt_pipeline_state.o \
	bst_virt_format_color.o \
	bst_virt_drm_device.o \
	bst_virt_drm_drv.o \
	bst_virt_drm_framebuffer.o \
	bst_virt_drm_private_obj.o \
	bst_virt_drm_plane.o \
	bst_virt_drm_crtc.o \
	bst_virt_drm_kms.o \
	bst_virt_drm_connector.o \
	bst_virt_drm_wb_connector.o \
	bst_virt_drm_debugfs.o

obj-y += bst-fw-msg/
obj-$(CONFIG_DRM_BST_VIRTUAL_DISPLAY) += bst-virtual-drm.o
