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

#ifndef _IPC_HW_LAYER_H
#define _IPC_HW_LAYER_H

#include "ipc_hw_common.h"

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

/**
 * ipc_hw_layer_init: ipc hardware layer init.
 * @ipc_param: in parameter, define some hardware config to hw registers
 * @hw_info: out parameter, get hw info from msgbx registers
 * @return: 0 - succeeded, non-zero for failures.
 */
int32_t ipc_hw_layer_init(const uint8_t endid, const ipc_init_params_t *ipc_param,
			msgbx_hw_info_t *hw_info);
int32_t ipc_hw_layer_deinit(const uint8_t endid);
int32_t ipc_hw_layer_get_info(const uint8_t endid, msgbx_hw_info_t *hw_info);

int32_t ipc_hw_layer_start(void);
int32_t ipc_hw_layer_stop(void);

// filtering rule set
int32_t ipc_hw_layer_flt_init(const uint8_t endid, const msgbx_flt_cfg_t *cfg);
int32_t ipc_hw_layer_flt_rule_set(const uint8_t endid, const uint8_t fid,
				  msgbx_flt_rule_cfg_t *rule);
int32_t ipc_hw_layer_flt_get_info(const uint8_t endid, const uint8_t fid,
				  msgbx_flt_rule_cfg_t *info);
int32_t ipc_hw_layer_flt_rule_clr(const uint8_t endid, const uint8_t fid);

// receive message
typedef int32_t (*recv_ntf)(void *addr, const uint8_t fid, const rw_msg_t *msg);
int32_t ipc_hw_layer_recv_ntf_register(void *addr, recv_ntf recv_func);
int32_t ipc_hw_layer_get_msg(const uint8_t endid, rw_msg_t *msg,
			    const uint8_t fid);

// send message
int32_t ipc_hw_layer_send_msg(const uint8_t endid, const rw_msg_t *msg);

// state management processing
typedef int32_t (*err_msg_ntf)(void *addr, msgbx_err_msg_t *err_msg);
int32_t ipc_hw_layer_err_msg_register(const uint8_t endid,
				    err_msg_ntf err_func);
int32_t ipc_hw_layer_err_hdl(const uint8_t endid, uint8_t id, uint32_t hdl);
int32_t ipc_hw_layer_get_time(const uint8_t endid, uint64_t *timestamp);

// status endmap processing
typedef int32_t (*endmap_ntf)(void *addr, const sts_endmap_t *endmap);
int32_t ipc_hw_layer_endmap_ntf_register(void *addr, endmap_ntf endmap_func);
int32_t ipc_hw_layer_set_endmap(const uint8_t endid, const uint8_t idx, const uint8_t status);
int32_t ipc_hw_layer_get_endmap(const uint8_t endid, sts_endmap_t *map);
int32_t ipc_hw_layer_update_endmap(const uint8_t endid, const uint8_t updated_chipid);

// msgbx hardware counter processing
int32_t ipc_hw_layer_get_hw_counter(const uint8_t endid, const uint8_t fid, msgbox_hw_counter_t *hw_cnt);
int32_t ipc_hw_layer_clr_hw_counter(const uint8_t endid, const uint8_t fid, const uint32_t clr_mask);
#endif
