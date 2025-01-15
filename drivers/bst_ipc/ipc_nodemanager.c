// SPDX-License-Identifier: (GPL-2.0 OR MIT)

/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#include "ipc_session.h"
#include "ipc_common.h"
#include "ipc_regs.h"
#include "ipc_mailbox_controller.h"
#include "ipc_nodemanager.h"

#define IPC_DRIVER_NAME "ipc_nodemanager"

/********************* local variables ***************************/
enum ipc_core_e ipc_channel[IPC_CORE_MAX] = {
	IPC_CORE_ARM1, IPC_CORE_ARM0, IPC_CORE_ARM3, IPC_CORE_ARM2,
	IPC_CORE_ARM0, IPC_CORE_ARM0, IPC_CORE_ARM0, IPC_CORE_ARM0,
	IPC_CORE_ARM1, IPC_CORE_ARM1, IPC_CORE_ARM1, IPC_CORE_ARM1,
	IPC_CORE_ARM2, IPC_CORE_ARM2, IPC_CORE_ARM2, IPC_CORE_ARM2,
	IPC_CORE_ARM3, IPC_CORE_ARM3, IPC_CORE_ARM3, IPC_CORE_ARM3,
	IPC_CORE_ARM0, IPC_CORE_ARM0
};

struct ipc_client_info *client_map[IPC_CORE_MAX] = { 0 };

/********************* function declaration ***************************/
int32_t ipc_node_parse(enum ipc_core_e core,
			  struct ipc_client_info **cl_info)
{
	IPC_LOG_INFO("%s, core = %d", __func__, core);
	// check driver is ready
	if (core >= ARRAY_SIZE(client_map)) {
		*cl_info = NULL;
		IPC_LOG_WARNING(" dst is invalid!");
		return -1;
	}
	if (client_map[core] == 0) {
		*cl_info = NULL;
		IPC_LOG_WARNING(" client is not available!");
		return -1;
	}

	*cl_info = (struct ipc_client_info *)client_map[core];
	IPC_LOG_INFO("get client info success");
	return 0;
}

int32_t ipc_node_destroy(enum ipc_core_e core_id)
{
	IPC_LOG_INFO("%s, core_id = %d", __func__, core_id);

	if (core_id >= ARRAY_SIZE(client_map)) {
		IPC_LOG_WARNING(" coreid %d is invalid, destroy fail", core_id);
		return -1;
	}
	client_map[core_id] = 0;

	return 0;
}

int32_t ipc_node_valid(enum ipc_core_e core_id)
{
	struct ipc_client_info *cl_info = NULL;
	uint32_t cnt = 100;

	IPC_LOG_INFO("core_id = %d", core_id);

	if (core_id >= ARRAY_SIZE(client_map)) {
		IPC_LOG_ERR(" source id is invalid!");
		return -1;
	}

	IPC_LOG_INFO("num_online_cpus = %d", num_online_cpus());
	if (core_id >= IPC_CORE_ARM0 && core_id <= IPC_CORE_ARM3) {
		if (num_online_cpus() <= core_id) {
			IPC_LOG_ERR("num_online_cpus() = %d, but core_id = %d",
				    num_online_cpus(), core_id);
			return -ENODEV;
		} else {
			return 0;
		}
	}

	if (client_map[core_id] == 0) {
		IPC_LOG_ERR("core is not ready!");
		return -CLIENT_STATUS_NOTREADY;
	}

	cl_info = (void *)client_map[core_id];
	while (cl_info->status != CLIENT_STATUS_READY && cnt--) {
		// schedule_timeout_interruptible(10);
		msleep(100);
	}

	if (cnt <= 0) {
		IPC_LOG_ERR("client_map[%d] is not OK, status is: %d", core_id,
			    cl_info->status);
		return cl_info->status;
	}

	IPC_LOG_INFO("driver is ready!");
	return cl_info->status;
}

int32_t ipc_get_node_of_coreid(enum ipc_core_e dest_core_id,
				    struct ipc_client_info **cl_info)
{
	struct ipc_client_info *client_info = NULL;
	struct ipc_mbox *ipc_mbox = NULL;
	int32_t ret = -1;

	IPC_LOG_INFO("%s, dst= %d", __func__, dest_core_id);
	if (dest_core_id < 0 || dest_core_id >= ARRAY_SIZE(client_map)) {
		IPC_LOG_ERR("core id %d is NOT available", dest_core_id);
		return -1;
	}

	// get client from map
	client_info = client_map[dest_core_id];
	if (client_info == NULL) {
		IPC_LOG_INFO("need new client for dst %d", dest_core_id);
		client_info = devm_kzalloc(&g_ipc_platform_dev->dev,
					   sizeof(*client_info), GFP_KERNEL);
		if (!client_info) {
			IPC_LOG_ERR("no enough memory!");
			return -ENOMEM;
		}

		ipc_mbox = platform_get_drvdata(g_ipc_platform_dev);
		if (IS_ERR_OR_NULL(ipc_mbox)) {
			ret = PTR_ERR_OR_ZERO(ipc_mbox);
			IPC_LOG_ERR(
				"platform_get_drvdata(g_ipc_platform_dev), ret %d",
				ret);
			return ret;
		}
		client_info->core_id = dest_core_id;
		client_info->sem_reg =
			ipc_mbox->sem_base + SEM_MST_ID_A_CPU +
			SEM_BANK0_OFFSET_BASE + SEM0_OFFSET_IN_BANK +
			SEM_IN_BANK_STEP * (dest_core_id - IPC_CORE_ARM0);

		client_info->tx_reg = IRQ_WRITE_ADDR(ipc_mbox->event_base,
						     ipc_channel[dest_core_id],
						     dest_core_id);
		IPC_LOG_INFO("client_info->tx_reg = 0x%llx",
			     __virt_to_phys(client_info->tx_reg));

		client_info->channel_status = CHANNEL_READY;
		init_completion(&client_info->tx_complete);
		client_info->status = CLIENT_STATUS_READY;
		client_map[dest_core_id] = client_info;
	}

	IPC_LOG_INFO("return dst %d client", dest_core_id);
	*cl_info = client_info;
	return 0;

}
