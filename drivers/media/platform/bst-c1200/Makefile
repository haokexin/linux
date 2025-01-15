# SPDX-License-Identifier: GPL-2.0
#
# Makefile for BST's V4L2 core
#

USERINCLUDE    := \
		-I$(srctree)/arch/$(SRCARCH)/include/uapi \
		-I$(objtree)/arch/$(SRCARCH)/include/generated/uapi \
		-I$(srctree)/include/bst/uapi \
		-I$(srctree)/include/uapi \
		-I$(objtree)/include/generated/uapi \
                -include $(srctree)/include/linux/kconfig.h

LINUXINCLUDE    := \
		-I$(srctree)/arch/$(SRCARCH)/include \
		-I$(objtree)/arch/$(SRCARCH)/include/generated \
		$(if $(building_out_of_srctree),-I$(srctree)/include/bst) \
		$(if $(building_out_of_srctree),-I$(srctree)/include) \
		-I$(objtree)/include/bst \
		-I$(objtree)/include \
		$(USERINCLUDE)

c1200-isp-objs += \
	csi2_rx.o csi_cdphy.o csi_controller.o \
	isp_core.o isp_video.o isp_sysfile.o isp_fw_loader.o \
	isp_msgbx_server.o video_server.o

obj-$(CONFIG_VIDEO_BST_C1200_ISP) += c1200-isp.o

obj-$(CONFIG_BST_C1200_CSITX) += csi_tx.o csitx_video.o

subdir-ccflags-y += -DC1200_ISP -Wall -save-temps 
