/* SPDX-License-Identifier: GPL-2.0 OR Apache 2.0
 *
 * Copyright (c) 2024 Black Sesame Technologies
 *
 * This program is also distributed under the terms of the Apache 2.0
 * License.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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

#include "ipc_trans_runtime.h"
#include "ipc_trans_cfg.h"

#ifndef FILTER_RULE_VERSION
#define FILTER_RULE_VERSION 0
#endif

#if (FILTER_RULE_VERSION == 1)
#define MSGBX_FID_MASK 0xF0000000
#define MSGBX_FID_BIT  28U
#endif
#if (FILTER_RULE_VERSION == 2)
static uint8_t pid_flt_rule_list[CHANNEL_COUNT][2] = {
	{0, 0}, {0, CPUMP2_1}, {ISPCV_0, ISPCV_4}, {NET_0, DMA_1},
	{SWITCH_0, SWITCH_5}, {SECURE_0, SAFETY_1}, {REALTIME_0, REALTIME_5},
	{MEDIA_0, MEDIA_0}
};
#endif

int8_t flt_cfg_init(void *addr)
{
#if (FILTER_RULE_VERSION != 0)
	msgbx_end_device_t *module = NULL;
	uint8_t fid_cnt;
	int32_t ret = -1;

	if (!addr)
		return -1;

	module = (msgbx_end_device_t *)addr;

	// note: Transfer the configuration of the hardware error interrupt flag to the implementation of the hardware layer
	// special hardware rule: disable default filter and enable other filter use fid filtering
	ret = ipc_hw_layer_flt_rule_clr(module->g_ipc_cpuid, 0);
	if (ret < 0) {
		IPC_LOG_WARNING("hw clr flt rule fail ret: %d", ret);
		return ret;
	}
	for(fid_cnt = 1; fid_cnt < CHANNEL_COUNT; ++fid_cnt)
		set_flt_rules(fid_cnt, 1, addr);
#endif
	return 0;
}

static int8_t get_flt_rules(const uint8_t fid, const uint8_t enable_flag, msgbx_flt_rule_cfg_t *rule, void *addr)
{
	if (!addr)
		return -1;

#if (FILTER_RULE_VERSION == 0)
#elif (FILTER_RULE_VERSION == 1)
	if (fid == 0) {
		rule->cfg_type = MSGBX_FLT_RULE_PID;
		rule->rule_pid.mbx_rx_pid_st = 0;
		rule->rule_pid.mbx_rx_pid_end = 0;
		rule->rule_pid.mbx_pid_flt_invert = 0;
	} else {
		rule->cfg_type = MSGBX_FLT_RULE_USER;
		rule->cfg_loc = MSGBX_FLT_RULE_HEADER;
		rule->rule_user.rx_res_mask = MSGBX_FID_MASK;
		rule->rule_user.rx_res_maskh = 0;
		if (fid == 1)
			rule->rule_user.rx_res_min = 0;
		else
			rule->rule_user.rx_res_min = fid << MSGBX_FID_BIT;
		rule->rule_user.rx_res_max = fid << MSGBX_FID_BIT;
		rule->rule_user.rx_res_maxh = 0;
		rule->rule_user.rx_res_minh = 0;
		rule->rule_user.msg_combi_lh_comp = 0;
		rule->rule_user.msg_flilter_invert = 0;
	}
#elif (FILTER_RULE_VERSION == 2)
	rule->cfg_type = MSGBX_FLT_RULE_PID;
	rule->rule_pid.mbx_rx_pid_st = pid_flt_rule_list[fid][0];
	rule->rule_pid.mbx_rx_pid_st = pid_flt_rule_list[fid][1];
	rule->rule_pid.mbx_len_flt_invert = 0;
#endif
	return 0;
}

int8_t set_flt_rules(const uint8_t fid, const uint8_t enable_flag, void *addr)
{
	int32_t ret = -1;
	msgbx_end_device_t *module = NULL;
	msgbx_flt_rule_cfg_t rule;

	if (!addr)
		return ret;

	if (fid >= CHANNEL_COUNT)
		return -2;

	module = (msgbx_end_device_t *)addr;
	ret = get_flt_rules(fid, enable_flag, &rule, addr);
	// set rules
	ret = ipc_hw_layer_flt_rule_set(module->g_ipc_cpuid, fid, &rule);
	if (ret < 0) {
		IPC_LOG_WARNING("hw set flt rule fail ret: %d", ret);
		return ret;
	}
	return 0;
}
