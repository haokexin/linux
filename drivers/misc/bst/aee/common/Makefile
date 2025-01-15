#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# Copyright (C) 2024 Black Sesame Technologies. Inc.
#

ccflags-y += -I$(srctree)/drivers/misc/bst/aee/mrdump/
ccflags-y += -I$(srctree)/drivers/misc/bst/include/
ccflags-y += -I$(srctree)/drivers/misc/bst/include/mt6765/include/

obj-y	+= aee.o
aee-y	:=dummy.o
aee-$(CONFIG_BST_AEE_FEATURE)	+= aee-common.o reboot-reason.o

KASAN_SANITIZE := n
