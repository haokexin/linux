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
#include <bst/ipc_hw_layer.h>
#include <bst/ipc_hw_impl.h>
#include "./msgbx_impl/ipc_hw_miscdev.h"

/* this is ipc hw_layer msgbx implementation. In this file, which provide
 * API for upper layer and compabitility to different hardware ipc mechanism.
 */
/********************* local variables ***************************/
static libipc_hw_compat_ops_t g_hw_ctl_ops;
static err_msg_ntf err_cb;
static recv_ntf recv_cb;

// ipc hw layer local function
int32_t ipc_hw_register_ops(const libipc_hw_compat_ops_t *ops)
{
	if (!ops)
		return -1;

	g_hw_ctl_ops = *ops;
	return 0;
}
EXPORT_SYMBOL(ipc_hw_register_ops);

// ipc hw layer api implementation
int32_t ipc_hw_layer_init(const uint8_t endid, const ipc_init_params_t *ipc_param, msgbx_hw_info_t *hw_info)
{
	// step1: register ops
	int32_t ret = -1;

	ret = ipc_hw_register_ops(&ipc_hw_ops);
	if (ret < 0)
		return -1;

	// step2: get info from hw
	ret = g_hw_ctl_ops.ipc_hw_get_info(endid, hw_info);
	if (ret < 0)
		return -2;

	// step3: init hardware
	ret = g_hw_ctl_ops.ipc_hw_init(endid, ipc_param);
	if (ret < 0)
		return -3;

	// step4: state mgt enable
	if(hw_info->mbx_end_id == CPU_7 || hw_info->mbx_end_id == CPUMP2_0) {
		ret = g_hw_ctl_ops.ipc_hw_fmu_mgt_enble(endid, MSG_END_MGT_CONFIG);
		if (ret < 0)
			return -4;
	}

	return 0;
}

int32_t ipc_hw_layer_deinit(const uint8_t cpuid)
{
	return g_hw_ctl_ops.ipc_hw_deinit(cpuid);
}

int32_t ipc_hw_layer_start(void)
{
	// step1:
	return 0;
}

// as for message receiving function, we use notify-getting strategy. It means that we would send a notification
// when we receive interrupt from msgbx default filter or dedicated config filter. IPC_trans_layer receive this
// notification, then call get_message actively from related filter fifo until it goes empty.
int32_t ipc_hw_layer_recv_ntf_register(void *addr, recv_ntf recv_func)
{
	recv_cb = recv_func;
	return 0;
}
EXPORT_SYMBOL(ipc_hw_layer_recv_ntf_register);

int32_t ipc_hw_layer_get_msg(const uint8_t endid, rw_msg_t *msg, const uint8_t fid)
{
	return g_hw_ctl_ops.ipc_hw_get_msg(endid, fid, msg);
}
EXPORT_SYMBOL(ipc_hw_layer_get_msg);

int32_t ipc_hw_layer_send_msg(const uint8_t cpuid, const rw_msg_t *msg)
{
	// remote end state check
	// self tx fifo state check
	if (msg->header.cid == msg->header.pid)
		if (recv_cb != NULL && g_ipc_end_array[cpuid] != NULL)
			return ((recv_ntf)recv_cb)(g_ipc_end_array[cpuid], 0, msg);
	return g_hw_ctl_ops.ipc_hw_send_msg(cpuid, msg);
}
EXPORT_SYMBOL(ipc_hw_layer_send_msg);

int32_t ipc_hw_recv_msg_notify(const uint8_t cpuid, const uint8_t fid, const rw_msg_t *msg)
{
#if defined(CONFIG_C1200_SLT) || defined(CONFIG_C1200_MASS)
	if (msg->header.typ ==  MSGBX_MSG_TYPE_HARDWARE && msg->header.cmd == 255) {
		ipc_trans_complete_test(fid, msg);
		return 0;
	}
	if (msg->header.typ ==  MSGBX_MSG_TYPE_USERDEFINED && msg->header.cmd == 255) {
		rw_msg_t sepc_msg = *msg;
		sepc_msg.payload[0] = fid << 4;	 // spec: high 32 bit store self irq number
		((recv_ntf)recv_cb)(g_ipc_end_array[cpuid], fid, &sepc_msg);
		return 0;
	}
#endif

	if (recv_cb != NULL && g_ipc_end_array[cpuid] != NULL)
		((recv_ntf)recv_cb)(g_ipc_end_array[cpuid], fid, msg);
	return 0;
}

int32_t ipc_hw_err_msg_notify(const uint8_t endid, const uint8_t fid, const msgbx_err_code_t code)
{
#ifdef IPC_STATE_MGT_ENABLE
	// get err msg and call function
	msgbx_err_msg_t err_msg;
	err_msg.fid = fid;
	err_msg.err_code = code;
	if (err_cb != NULL) {
		((err_msg_ntf)err_cb)(g_ipc_end_array[endid], &err_msg);
	}
#endif
	return 0;
}

// filtering rule processing
int32_t ipc_hw_layer_flt_init(const uint8_t cpuid, const msgbx_flt_cfg_t *cfg)
{
	int32_t ret = 0;
#ifdef IPC_FLT_MGT_ENABLE
	// filter state management enable
	ret = g_hw_ctl_ops.ipc_hw_flt_mgt_enble(cpuid, cfg->flt_id, cfg->mbx_flt_mgt_flag);
#endif
	return ret;
}
EXPORT_SYMBOL(ipc_hw_layer_flt_init);

int32_t ipc_hw_layer_flt_rule_set(const uint8_t cpuid, const uint8_t fid, msgbx_flt_rule_cfg_t *rule)
{
	// set rule to filter
	return g_hw_ctl_ops.ipc_hw_set_flt_cfg(cpuid, fid, rule);
}
EXPORT_SYMBOL(ipc_hw_layer_flt_rule_set);

int32_t ipc_hw_layer_flt_get_info(const uint8_t cpuid, const uint8_t fid, msgbx_flt_rule_cfg_t *info)
{
	return g_hw_ctl_ops.ipc_hw_get_flt_info(cpuid, fid, info);
}
EXPORT_SYMBOL(ipc_hw_layer_flt_get_info);


int32_t ipc_hw_layer_flt_rule_clr(const uint8_t cpuid, const uint8_t fid)
{
	return g_hw_ctl_ops.ipc_hw_clr_flt_cfg(cpuid, fid);
}
EXPORT_SYMBOL(ipc_hw_layer_flt_rule_clr);

int32_t ipc_hw_layer_err_msg_register(const uint8_t cpuid, err_msg_ntf err_func)
{
	err_cb = err_func;
	return 0;
}
EXPORT_SYMBOL(ipc_hw_layer_err_msg_register);

int32_t ipc_hw_layer_err_hdl(const uint8_t endid, uint8_t id, uint32_t hdl)
{
	return g_hw_ctl_ops.ipc_hw_err_hdl(endid, id, hdl);
}
EXPORT_SYMBOL(ipc_hw_layer_err_hdl);

int32_t ipc_hw_layer_get_time(const uint8_t cpuid, uint64_t *timestamp)
{
	if (g_hw_ctl_ops.ipc_hw_get_time)
		return g_hw_ctl_ops.ipc_hw_get_time(timestamp);
	else
		return 0;
}
EXPORT_SYMBOL(ipc_hw_layer_get_time);

int32_t ipc_hw_endmap_notify(const uint8_t endid, const sts_endmap_t *endmap)
{
	return 0;
}
EXPORT_SYMBOL(ipc_hw_endmap_notify);

int32_t ipc_hw_layer_endmap_ntf_register(void *addr, endmap_ntf endmap_func)
{
	return 0;
}
EXPORT_SYMBOL(ipc_hw_layer_endmap_ntf_register);

int32_t ipc_hw_layer_set_endmap(const uint8_t endid, const uint8_t idx, const uint8_t status)
{
	return 0;
}
EXPORT_SYMBOL(ipc_hw_layer_set_endmap);

int32_t ipc_hw_layer_get_endmap(const uint8_t endid, sts_endmap_t *map)
{
	return 0;
}
EXPORT_SYMBOL(ipc_hw_layer_get_endmap);

int32_t ipc_hw_layer_update_endmap(const uint8_t endid, const uint8_t updated_chipid)
{
	return 0;
}
EXPORT_SYMBOL(ipc_hw_layer_update_endmap);

int32_t ipc_hw_layer_get_hw_counter(const uint8_t endid, const uint8_t fid, msgbox_hw_counter_t *hw_cnt)
{
	return 0;
}
EXPORT_SYMBOL(ipc_hw_layer_get_hw_counter);

int32_t ipc_hw_layer_clr_hw_counter(const uint8_t endid, const uint8_t fid, const uint32_t clr_mask)
{
	return 0;
}
EXPORT_SYMBOL(ipc_hw_layer_clr_hw_counter);
