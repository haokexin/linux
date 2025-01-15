/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is also distributed under the terms of the BSD 3-Clause
 * License.
 *
 * Copyright (C) 2023 Black Sesame Technologies. Inc.
 */

#ifndef _IPC_TRANS_CFG_H
#define _IPC_TRANS_CFG_H

#include <bst/ipc_hw_layer.h>
#ifdef __cplusplus
extern "C" {
#endif

enum _flt_status_t {
	FLT_UNUSED = 0,
	FLT_SET = 1,
};
#define flt_status_t enum _flt_status_t

struct _ipc_flt_cfg_t {
	msgbx_flt_cfg_t cfg;
	msgbx_flt_info_t info;
	msgbx_flt_rule_cfg_t rule;
	flt_status_t status;
};
#define ipc_flt_cfg_t struct _ipc_flt_cfg_t

int8_t set_flt_rules(const uint8_t fid, void *addr);
int8_t flt_cfg_init(void *addr);

#ifdef __cplusplus
}
#endif
#endif
