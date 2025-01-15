ccflags-y +=  -I${srctree}/drivers/bst_ipc_msgbx/include \
        -I${srctree}/drivers/bst_ipc_msgbx/ipc_trans_layer/include \
        -I${srctree}/drivers/bst_ipc_msgbx/ipc_app_layer/include \
        -I${srctree}/drivers/bst_ipc_msgbx/ipc_app_layer/posix/include \
			
#obj-y	+= virtual-bst.o st_public_client.o SocClient.o
#obj-y	+= virtual-bst.o

obj-$(CONFIG_VIRTUAL_QSPI)	+= virtual-qspi.o  st_public_client.o SocClient.o