# SPDX-License-Identifier: GPL-2.0

ccflags-y := \
	-I $(srctree)/$(src)/../include \
	-I $(srctree)/$(src)

bst-native-drm-y := \
	bst_drm_drv.o \
	bst_drm_dev.o \
	bst_drm_framebuffer.o \
	bst_drm_kms.o \
	bst_drm_crtc.o \
	bst_drm_plane.o \
	bst_drm_wb_connector.o \
	bst_drm_private_obj.o \
	bst_drm_resv_mem.o \
	bst_format_color.o \
	bst_pipeline.o \
	bst_pipeline_state.o \
	bst_md_csr.o \
	bst_disp_conn.o \
	bst_dpu_csr.o

#obj-$(CONFIG_DRM_BST_DC) += bst-dc/
obj-$(CONFIG_DRM_BST_VOUT) += bst-vout/
obj-$(CONFIG_DRM_BST_LVDS) += bst-lvds/
obj-$(CONFIG_DRM_BST_MIPI_DSI) += bst-mipi-dsi/
obj-$(CONFIG_DRM_BST_DPTX) += bst-dptx/
obj-$(CONFIG_DRM_BST_NATIVE_DISPLAY) += bst-native-drm.o
