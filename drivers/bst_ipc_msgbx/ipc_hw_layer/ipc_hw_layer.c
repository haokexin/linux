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
#include <bst/ipc_hw_layer.h>
#include <bst/ipc_hw_impl.h>
#include "../ipc_trans_layer/src/ipc_trans_runtime.h"

/* this is ipc hw_layer msgbx implementation. In this file, which provide
 * API for upper layer and compabitility to different hardware ipc mechanism.
 */
/********************* local variables ***************************/
static libipc_hw_compat_ops_t g_hw_ctl_ops;
static err_msg_callback err_cb;
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
int32_t ipc_hw_layer_init(const uint8_t cpuid, const ipc_init_params_t *ipc_param, msgbx_hw_info_t *hw_info)
{
	// step1: register ops
	int32_t ret = -1;

	ret = ipc_hw_register_ops(&g_ipc_end_array[cpuid]->ops);
	if (ret < 0)
		return -1;

	// step2: get info from hw
	ret = g_hw_ctl_ops.ipc_hw_get_info(cpuid, hw_info);
	if (ret < 0)
		return -2;

	// step3: init hardware
	ret = g_hw_ctl_ops.ipc_hw_init(cpuid, ipc_param);
	if (ret < 0)
		return -3;

	// step4: state mgt enable
#ifdef IPC_STATE_MGT_ENABLE
	if (ipc_param->msgbx_end_mgt_flag != 0) {
		ret = g_hw_ctl_ops.ipc_hw_sts_mgt_enble(cpuid, ipc_param->msgbx_end_mgt_flag);
		if (ret < 0)
			return -4;
	}
#endif

	return 0;
}

int32_t ipc_hw_layer_deinit(const uint8_t cpuid)
{
#ifdef IPC_STATE_MGT_ENABLE
	int32_t ret = -1;

	ret = g_hw_ctl_ops.ipc_hw_sts_mgt_disable(cpuid);
	if (ret < 0)
		return ret;
#endif
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

int32_t ipc_hw_layer_get_msg(const uint8_t cpuid, rw_msg_t *msg, const uint8_t fid)
{
	return g_hw_ctl_ops.ipc_hw_get_msg(cpuid, msg, fid);
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
#ifdef CONFIG_C1200_SLT
	if (msg->header.typ ==  MSGBX_MSG_TYPE_USERDEFINED && msg->header.cmd == 0) {
		ipc_trans_complete_test(fid, msg);
		return 0;
	}
	if (msg->header.typ ==  MSGBX_MSG_TYPE_USERDEFINED && msg->header.cmd == 1) {
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

int32_t ipc_hw_err_msg_notify(const uint8_t cpuid, const uint8_t fid)
{
#ifdef IPC_STATE_MGT_ENABLE
	// get err msg and call function
	msgbx_err_msg_t err_msg = { 0 };

	g_hw_ctl_ops.ipc_hw_get_err_msg(cpuid, fid, &err_msg);
	if (err_cb != NULL)
		((err_msg_callback)err_cb)(g_ipc_end_array[cpuid], &err_msg);
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

int32_t ipc_hw_layer_err_msg_register(const uint8_t cpuid, err_msg_callback err_func)
{
	err_cb = err_func;
	return 0;
}
EXPORT_SYMBOL(ipc_hw_layer_err_msg_register);

int32_t ipc_hw_layer_err_hdl(const uint8_t cpuid, uint8_t type, uint8_t id, uint32_t hdl)
{
	return g_hw_ctl_ops.ipc_hw_err_hdl(cpuid, type, id, hdl);
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
