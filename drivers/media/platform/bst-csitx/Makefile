# SPDX-License-Identifier: GPL-2.0

USERINCLUDE	:= \
		-I$(srctree)/arch/$(SRCARCH)/include/uapi \
		-I$(objtree)/arch/$(SRCARCH)/include/generated/uapi \
		-I$(srctree)/include/bst/uapi \
		-I$(srctree)/include/uapi \
		-I$(objtree)/include/generated/uapi \
                -include $(srctree)/include/linux/kconfig.h

LINUXINCLUDE	:= \
		-I$(srctree)/arch/$(SRCARCH)/include \
		-I$(objtree)/arch/$(SRCARCH)/include/generated \
		$(if $(building_out_of_srctree),-I$(srctree)/include/bst) \
		$(if $(building_out_of_srctree),-I$(srctree)/include) \
		-I$(objtree)/include/bst \
		-I$(objtree)/include \
		$(USERINCLUDE)

obj-$(CONFIG_BST_C1200_CSITX) += csi_tx.o csitx_video.o

subdir-ccflags-y += -Wall
