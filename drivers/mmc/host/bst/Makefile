#obj-$(CONFIG_MMC_SDHCI_BST)	        += sdhci-of-bst.o

ccflags-y +=  -I${srctree}/drivers/bst_ipc_msgbx/include \
        -I${srctree}/drivers/bst_ipc_msgbx/ipc_trans_layer/include \
        -I${srctree}/drivers/bst_ipc_msgbx/ipc_app_layer/include \
        -I${srctree}/drivers/bst_ipc_msgbx/ipc_app_layer/posix/include \

obj-$(CONFIG_MMC_SDHCI_BST)	        += sdhci-of-bst-c1200.o SdemmcClient.o sdemmc_client.o
