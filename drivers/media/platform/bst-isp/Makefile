# SPDX-License-Identifier: GPL-2.0

USERINCLUDE	:= \
		-I$(srctree)/arch/$(SRCARCH)/include/uapi \
		-I$(objtree)/arch/$(SRCARCH)/include/generated/uapi \
		-I$(srctree)/include/bst/uapi \
		-I$(srctree)/include/uapi \
		-I$(objtree)/include/generated/uapi \
		-include $(srctree)/include/linux/compiler-version.h \
		-include $(srctree)/include/linux/kconfig.h

LINUXINCLUDE	:= \
		-I$(srctree)/arch/$(SRCARCH)/include \
		-I$(objtree)/arch/$(SRCARCH)/include/generated \
		$(if $(building_out_of_srctree),-I$(srctree)/include/bst) \
		$(if $(building_out_of_srctree),-I$(srctree)/include) \
		-I$(objtree)/include \
		$(USERINCLUDE)

csi-objs += csi_cdphy.o csi_controller.o csi_rx.o csi_safety.o csi_sysfs.o
isp-objs += isp_core.o isp_misc.o isp_msg.o isp_proto_base.o isp_sysfs.o isp_video.o

obj-$(CONFIG_VIDEO_BST_ISP) += csi.o isp.o
subdir-ccflags-y += -Wall
subdir-ccflags-y += -I$(srctree)/$(src)

ifneq ($(CONFIG_BST_IPC_MSGBX), )
    obj-$(CONFIG_VIDEO_BST_ISP) += msgbox/
endif

obj-$(CONFIG_VIDEO_BST_ISP) += c1200/
subdir-ccflags-y += -I$(srctree)/$(src)/c1200
