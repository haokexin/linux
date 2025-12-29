/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is also distributed under the terms of the BSD 3-Clause
 * License.
 *
 * Copyright (C) 2024 Black Sesame Technologies. Inc.
 */
#ifndef _IPC_HW_MISCDEV_H
#define _IPC_HW_MISCDEV_H

#include <linux/types.h>
#include "ipc_msgbox_controller.h"

struct handle_list {
	struct list_head list;
	char comm[TASK_COMM_LEN];
	uint8_t handle;
	uint8_t endid;
	uint8_t mapped_flag;
	pid_t pid;
	uint8_t close_flag;
	void* map_addr;
};

extern struct ipc_msgbox *g_ipc_msgbx;

extern ST_MSGBX_END_PARA msg_end_para[MAX_END_NUM];
extern msgbx_end_device_t *g_ipc_end_array[MAX_END_NUM];
extern libipc_hw_compat_ops_t ipc_hw_ops;
extern struct platform_device *g_ipc_msgbx_pdev;

#define for_each_ends(endid, total) \
	for (endid = g_start_pid; endid< total; endid++)

#define for_each_payloads(idx) \
	for (idx = 0; idx < 4; idx++)

#endif
