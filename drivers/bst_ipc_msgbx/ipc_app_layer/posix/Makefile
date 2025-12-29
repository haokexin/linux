# SPDX-License-Identifier: GPL-2.0
#
# Makefile for bst msgbx hw
#
ccflags-y += -I$(srctree)/drivers/bst_ipc_msgbx/ipc_app_layer/posix/include \
	     -I$(srctree)/drivers/bst_ipc_msgbx/ipc_trans_layer/include
# obj-y += test_server.o server.o
# obj-y += test_client.o client.o
# obj-${CONFIG_MSGBOX_DEBUG_TEST} += client.o server.o
obj-y +=  db_client.o db_switch_client.o ivi_client.o ivi_switch_client.o MacSyncClient.o sw_macsync_client.o

#obj-$(CONFIG_MSGBOX_TEST_MODULE) := msgbx-test-a.o
#msgbx-test-a-objs := msgbox_test.o
#
#obj-$(CONFIG_MSGBOX_TEST_MODULE) += msgbx-test-b.o
#msgbx-test-b-objs := msgbox_test.o
