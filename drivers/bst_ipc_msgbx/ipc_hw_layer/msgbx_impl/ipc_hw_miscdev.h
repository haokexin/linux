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

extern struct ipc_msgbox *g_ipc_msgbx;
extern ST_MSGBX_END_PARA msg_end_para[MAX_END_NUM];
extern IPC_SHARE_BUFF *g_ipc_end_array[MAX_END_NUM];
extern IPC_SHARE_MSG_BUFF **g_ipc_end_ses_map[MAX_END_NUM];
extern struct platform_device *g_ipc_msgbx_pdev;
void per_msgbx_end_register(void *per_data);

#define for_each_ends(endid, total) \
	for (endid = g_start_pid; endid< total; endid++)

#define for_each_payloads(idx) \
	for (idx = 0; idx < 4; idx++)

#endif
