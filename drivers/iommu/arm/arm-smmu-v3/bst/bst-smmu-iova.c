// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/reboot.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <asm-generic/signal.h>
#include "msgbox/smmu-client/src-gen/smmu_client.h"

static smmu_client_t *client;
static smmu_client_data_t data = {0,};

int smmu_msgbox_init(void)
{
	int ret = -1;
	ipc_inf_version_t version = {0};

	#ifdef CONFIG_BST_C1200_IVI
	#define MSG_PID 16
	#elif defined(CONFIG_BST_C1200_DB)
	#define MSG_PID 32
	#else
	#define MSG_PID 20
	#endif

	data.com_data.pid = MSG_PID;
	data.com_data.sid = 14;
	data.com_data.fid = 1;

	client = smmu_client_init(&data);
	if (!client)
		return -1;

	// get version
	version = client->mapping_client.version();
	pr_debug("%s, Interface version: major %d, minor %d.\n", __func__, version.major, version.minor);

	// start test_client
	ret = client->start();
	if (ret < 0) {
		pr_err("%s, start msgbx client failed!\n", __func__);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(smmu_msgbox_init);

static int iommu_by_proxy(unsigned long sid, unsigned long iova, phys_addr_t paddr, size_t size, size_t flag)
{
	int msgbox_ret = 0;
	unsigned int output = 0;
	mapping_ErrorEnum_t err = MAPPING_NO_ERROR;
	mapping_cfg_pte_t smmu_map_param;

	smmu_map_param.sid = sid;
	smmu_map_param.flag = flag; /* 0x1: map. 0x2：unmap */
	smmu_map_param.high_addr = paddr >> 32;
	smmu_map_param.low_addr = paddr & 0xffffffff;
	smmu_map_param.len = size;
	smmu_map_param.iova = iova;

	if (!client)
		return -1; /* client does not exist */

	msgbox_ret = client->mapping_client.smmu_r5_method_sync(&smmu_map_param, &output, &err, 1000, NULL);
	if (msgbox_ret != 0 || (err != MAPPING_NO_ERROR)) {
		pr_err("smmu_r5_method_sync fail, msgbox_ret: %d\n", msgbox_ret);
		return -2; /* msgbox transmission failed */
	}

	return 0;
}

int iommu_map_by_proxy(unsigned long sid, unsigned long iova, phys_addr_t paddr, size_t size)
{
	return iommu_by_proxy(sid, iova, paddr, size, 0x1);
}
EXPORT_SYMBOL_GPL(iommu_map_by_proxy);

int iommu_unmap_by_proxy(unsigned long sid, unsigned long iova, phys_addr_t paddr, size_t size)
{
	return iommu_by_proxy(sid, iova, paddr, size, 0x2);
}
EXPORT_SYMBOL_GPL(iommu_unmap_by_proxy);

void smmu_msgbox_exit(void)
{
	int ret = 0;

	if (!client)
		return;
	// stop to receive msg, must
	ret = client->stop();
	if (ret < 0) {
		pr_err("smmu_msgbox stop failed,ret = %d !", ret);
		return;
	}
	ret = smmu_client_destroy();
	if (ret)
		pr_err("smmu_msgbox destroy failed,ret = %d !", ret);
}
EXPORT_SYMBOL_GPL(smmu_msgbox_exit);
