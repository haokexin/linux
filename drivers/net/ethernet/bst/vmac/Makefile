# SPDX-License-Identifier: GPL-2.0
ccflags-y += -I$(srctree)/drivers/bst_ipc_msgbx/ipc_app_layer/posix/include -I$(srctree)/drivers/bst_ipc_msgbx/ipc_app_layer/posix \
			 -I$(srctree)/drivers/bst_ipc_msgbx/include -I$(srctree)/drivers/bst_ipc_msgbx/ipc_trans_layer/include
obj-$(CONFIG_BST_VMAC_ETH) += bstvmac.o
bstvmac-objs:= bstvmac_main.o bstvmac_ethtool.o chain_mode.o \
	      bstvmac_dma.o bstvmac_core.o hwif.o bstvmac_descs.o \
	      $(bstvmac-y)

# Ordering matters. Generic driver must be last.

obj-$(CONFIG_BST_VMAC_PLATFORM)	+= bstvmac-platform.o
bstvmac-platform-objs:= bstvmac_of.o bstvmac_platform.o

KDIR:/lib/modules/'uname -r'/build

default:
	$(MAKE) -C $(KDIR) M='pwd' modules

clean:
	rm -rf .*.cmd *.o *.mod.c *.ko .tmp_versions *.order *sysvers *Module.markers

