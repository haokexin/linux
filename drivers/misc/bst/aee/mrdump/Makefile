#
# Copyright (C) 2016 MediaTek Inc.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See http://www.gnu.org/licenses/gpl-2.0.html for more details.
#
# Copyright (C) 2024 Black Sesame Technologies. Inc.
#

ccflags-y += -I$(srctree)/drivers/misc/bst/include
ccflags-y += -I$(srctree)/kernel/

ifneq ($(CONFIG_ARCH_BST), y)
ccflags-$(CONFIG_BST_AEE_IPANIC) += -DTEXT_OFFSET=$(TEXT_OFFSET)
endif
obj-$(CONFIG_BST_AEE_IPANIC)	+= mrdump.o

mrdump-objs-y := \
	mrdump_control.o \
	mrdump_hw.o \
	mrdump_full.o \
	mrdump_key_setup.o \
	mrdump_mini.o \
	mrdump_panic.o

mrdump-objs-$(CONFIG_ARM) += mrdump_arm.o
mrdump-objs-$(CONFIG_ARM_PSCI) += mrdump_panic_wdt.o

mrdump-objs-$(CONFIG_ARM64) += mrdump_arm64.o
mrdump-objs-$(CONFIG_BST_AEE_MRDUMP_WDT) += mrdump_panic_wdt.o

mrdump-objs := $(mrdump-objs-y)

KASAN_SANITIZE := n
