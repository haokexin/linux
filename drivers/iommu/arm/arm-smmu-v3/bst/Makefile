# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_ARM_SMMU_V3) += arm_smmu_v3.o
arm_smmu_v3-objs-y += arm-smmu-v3.o
arm_smmu_v3-objs-$(CONFIG_ARM_SMMU_V3_SVA) += arm-smmu-v3-sva.o
arm_smmu_v3-objs := $(arm_smmu_v3-objs-y)

ccflags-${CONFIG_SMMU_MSGBX_R5MAP} := \
	-I $(srctree)/include/bst/ \
	-I $(srctree)/drivers/bst_ipc_msgbx/include \
	-I $(srctree)/drivers/bst_ipc_msgbx/ipc_app_layer/include \
	-I $(srctree)/drivers/bst_ipc_msgbx/ipc_trans_layer/include \

obj-${CONFIG_COREIP_SMMMU_MULTIOS_BST} += bst-smmu-iova.o
obj-${CONFIG_SMMU_MSGBX_R5MAP} += msgbox/smmu-client/src-gen/mapping_client.o msgbox/smmu-client/src-gen/smmu_client.o