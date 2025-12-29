ifeq ($(CONFIG_BST_WATCHDOG_SLT),y)
        subdir-ccflags-y        += -DIS_SLT
endif

ccflags-y +=  -I${srctree}/drivers/bst_ipc_msgbx/include \
        -I${srctree}/drivers/bst_ipc_msgbx/ipc_trans_layer/include \
        -I${srctree}/drivers/bst_ipc_msgbx/ipc_app_layer/include \
        -I${srctree}/drivers/bst_ipc_msgbx/ipc_app_layer/posix/include \

obj-$(CONFIG_BST_WATCHDOG) += bst_wdt.o watchdog_client.o watchdogClient.o
