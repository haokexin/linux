// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/* This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is also distributed under the terms of the BSD 3-Clause
 * License.
 *
 * Copyright (C) 2023 Black Sesame Technologies. Inc.
 */

/**
 * @file  ipc_trans_cfg.c
 * @brief This file is used for Msgbx filter configuration management.
 *        It provides functionalities to set filter configuration information to
 * device.
 *
 * @details Feature list:
 * 1. Filtering rule definition: Defines the rules for filtering messages.
 * 2. Filter configuration initialization: Initializes the filter configuration.
 * 3. Get filter rules: Generate the filter rules based on the hardware message header version.
 * 4. Set filter rules: Sets the filter rules based on the filter ID.
 *
 * @note This file is part of the Msgbx IPC transport layer. It is responsible for managing the configuration of the
 * Msgbx dispatching messages, including the definition and management of filtering rules.
 */

#include "../include/config.h"
#include "ipc_trans_cfg.h"
#include "ipc_trans_runtime.h"

#define DEFAULT_FLT_ID 0
#define CHANNEL_COUNT_IDX (CHANNEL_COUNT - 1)

int8_t flt_cfg_init(void *addr)
{
	msgbx_end_device_t *module = NULL;

	if (!addr)
		return -1;

	module = (msgbx_end_device_t *)addr;
	ipc_memset(&module->cfg_map, 0, sizeof(module->cfg_map));
	return 0;
}

static void get_flt_rules(const uint8_t fid, msgbx_flt_rule_cfg_t *rule)
{
#if (IPC_HW_LAYER_VERSION == 1)
	if (rule->cfg_type == IPC_FLT_RULE_USER) {
		rule->rule_user.cfg_loc = IPC_FLT_RULE_HEADER;
		rule->rule_user.msgh_combi_lh_comp = 1;
		rule->rule_user.msgh_flilter_invert = 0;

		rule->rule_user.rx_res_min = fid << 8;
		rule->rule_user.rx_res_max = rule->rule_user.rx_res_min | 0xF0;
		rule->rule_user.tx_reserved_filter_mask = 8;
	}
#endif
}

int8_t set_flt_rules(const uint8_t fid, void *addr)
{
	int32_t ret = -1;
	msgbx_end_device_t *module = NULL;
	uint8_t fid_idx = 0;

	if (!addr)
		return ret;

	if (fid >= CHANNEL_COUNT)
		return -2;

	// note: used filter do not need set filter rule again
	// note: default filter do not need set filter rule
	module = (msgbx_end_device_t *)addr;
	fid_idx = fid - 1;

	if (fid == DEFAULT_FLT_ID ||
	    module->cfg_map[fid_idx].status != FLT_UNUSED)
		return 1;

	// set rules
	module->cfg_map[fid_idx].cfg.flt_id = fid;
	module->cfg_map[fid_idx].rule.rule_user.cfg_loc = IPC_FLT_RULE_HEADER;
	module->cfg_map[fid_idx].rule.cfg_type = IPC_FLT_RULE_USER;

	get_flt_rules(fid, &module->cfg_map[fid_idx].rule);
#ifdef IPC_STATE_MGT_ENABLE
	module->cfg_map[fid_idx].cfg.mbx_flt_mgt_flag = MSG_FLT_MGT_CONFIG;
#else
	module->cfg_map[fid_idx].cfg.mbx_flt_mgt_flag = 0;
#endif
	IPC_LOG_DEBUG(
		"fid %u set flt min: %llu, max: %llu, mask: %llu, flag: %u\n", fid,
		module->cfg_map[fid_idx].rule.rule_user.rx_res_min,
		module->cfg_map[fid_idx].rule.rule_user.rx_res_max,
		module->cfg_map[fid_idx].rule.rule_user.tx_reserved_filter_mask,
		module->cfg_map[fid_idx].cfg.mbx_flt_mgt_flag);

	ret = ipc_hw_layer_flt_rule_set(module->ops.cpuid,
					module->cfg_map[fid_idx].cfg.flt_id,
					&module->cfg_map[fid_idx].rule);
	if (ret < 0) {
		IPC_LOG_WARNING("hw set flt rule fail ret: %d", ret);
		return ret;
	}

	module->cfg_map[fid_idx].status = FLT_SET;
	return 0;
}
