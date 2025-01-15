/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
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

#ifndef _IPC_HW_LAYER_H
#define _IPC_HW_LAYER_H

#include "ipc_hw_common.h"

// this is the header file for transferring layer

// access MsgBxEnd device steps
// 1. prepare ipc_hw environment, transferring config to hw layer.
// 2. add device (device type: msgbx, interrupt+shared buffer, semaphore)
// 3. open the device (reserve some time for hw layer finish hardware initiation)
// 4. in order to decoupling, we choose to callback notification and then user request data actively
// 5. write and read MsgBx TX or filter
// 6. close the device
// 7. clear the ipc_hw environment

// this is the abstraction compatible layer for MsgBxEnd processing.
// As for MsgBxEnd device, we should access the fifo, interrupt,

// use for transferring layer

/**
 * ipc_hw_layer_init: ipc hardware layer init.
 * @ipc_param: in parameter, define some hardware config to hw registers
 * @hw_info: out parameter, get hw info from msgbx registers
 * @return: 0 - succeeded, non-zero for failures.
 */
int32_t ipc_hw_layer_init(const uint8_t cpuid,
			  const ipc_init_params_t *ipc_param,
			  msgbx_hw_info_t *hw_info);
int32_t ipc_hw_layer_deinit(const uint8_t cpuid);
int32_t ipc_hw_layer_get_info(const uint8_t cpuid, msgbx_hw_info_t *hw_info);

int32_t ipc_hw_layer_start(void);
int32_t ipc_hw_layer_stop(void);

// filtering rule set
int32_t ipc_hw_layer_flt_init(const uint8_t cpuid, const msgbx_flt_cfg_t *cfg);
int32_t ipc_hw_layer_flt_rule_set(const uint8_t cpuid, const uint8_t fid,
				  msgbx_flt_rule_cfg_t *rule);
int32_t ipc_hw_layer_flt_get_info(const uint8_t cpuid, const uint8_t fid,
				  msgbx_flt_rule_cfg_t *info);
int32_t ipc_hw_layer_flt_rule_clr(const uint8_t cpuid, const uint8_t fid);

// receive message
typedef int32_t (*recv_ntf)(void *addr, const uint8_t fid, const rw_msg_t *msg);
int32_t ipc_hw_layer_recv_ntf_register(void *addr, recv_ntf recv_func);
int32_t ipc_hw_layer_get_msg(const uint8_t cpuid, rw_msg_t *msg,
			     const uint8_t fid);

// send message
int32_t ipc_hw_layer_send_msg(const uint8_t cpuid, const rw_msg_t *msg);

// state management processing
typedef int32_t (*err_msg_callback)(void *addr, msgbx_err_msg_t *err_msg);
int32_t ipc_hw_layer_err_msg_register(const uint8_t cpuid,
				      err_msg_callback err_func);
int32_t ipc_hw_layer_err_hdl(const uint8_t cpuid, uint8_t type, uint8_t id,
			     uint32_t hdl);
int32_t ipc_hw_layer_get_time(const uint8_t cpuid, uint64_t *timestamp);

#endif
