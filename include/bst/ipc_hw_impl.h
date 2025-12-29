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
/**
 * @file  ipc_hw_impl.h
 * @brief this file is used as ipc hardware layer implementation api definition, you should implement your own ipc driver in follow options.
 * @note
 * @details feature list
 */
#ifndef _IPC_HW_IMPL_H
#define _IPC_HW_IMPL_H

#include <bst/ipc_hw_common.h>

// this is the header file for hardware implementation, the core developer should follow these definition
// to implement related hardware function.

/********************* extern global function *******************/
// NOTE: please keep these extern function declarations in your own code, they are requiered api for you
// ipc_hw_layer recv msg notify, hw_impl_layer would call this api when interrupts receive message
extern int32_t ipc_hw_recv_msg_notify(const uint8_t endid, const uint8_t fid, const rw_msg_t *msg);
// ipc_hw_layer error msg notify, hw_impl_layer would call this api when it receive error state
extern int32_t ipc_hw_err_msg_notify(const uint8_t endid, const uint8_t fid, const msgbx_err_code_t code);
// ipc_hw_layer hw endmap update notify, ipc_impl_layer would call this api when it receive endmap update irq
extern int32_t ipc_hw_endmap_notify(const uint8_t endid, const sts_endmap_t *endmap);

// api definition
struct _libipc_hw_compat_ops_t {
	//note: this label is used for multi-end, please refer to programmer guide for more details.
	int32_t (*ipc_hw_init)(const uint8_t endid, const ipc_init_params_t *ipc_param);
	int32_t (*ipc_hw_deinit)(const uint8_t endid);

	// msgbx spec init
	int32_t (*ipc_hw_get_info)(const uint8_t endid, msgbx_hw_info_t *hw_info);

	// msgbx filtering rule config
	int32_t (*ipc_hw_set_flt_cfg)(const uint8_t endid, const uint8_t fid,
				      const msgbx_flt_rule_cfg_t *rule);
	int32_t (*ipc_hw_clr_flt_cfg)(const uint8_t endid,
				    const uint8_t fid);
	int32_t (*ipc_hw_get_flt_info)(
		const uint8_t endid, const uint8_t fid,
		msgbx_flt_rule_cfg_t *info); //debug get rule setting

	// msgbx send / recv msg
	int32_t (*ipc_hw_send_msg)(const uint8_t endid, const rw_msg_t *msg);
	int32_t (*ipc_hw_get_msg)(const uint8_t endid, const uint8_t fid, rw_msg_t *msg);

	// msgbx spec req
	int32_t (*ipc_hw_get_time)(uint64_t *timestamp);

	// msgbx state management
	int32_t (*ipc_hw_fmu_mgt_enble)(const uint8_t endid,
					const uint32_t flag);

	// msgbx filter state management config
	int32_t (*ipc_hw_flt_mgt_enble)(const uint8_t endid,
					const uint8_t fid,
					const uint8_t flag);

	// msgbx fault handle
	int32_t (*ipc_hw_err_hdl)(const uint8_t endid, const uint8_t id,
				const uint32_t hdl);

	// msgbx set status endmap
	int32_t (*ipc_hw_set_endmap)(const uint8_t endid, const uint8_t idx, const uint8_t status);
	int32_t (*ipc_hw_get_endmap)(const uint8_t endid, sts_endmap_t *map);
	int32_t (*ipc_hw_update_endmap)(const uint8_t endid, const uint8_t updated_chipid);

	// msgbx get hw counter
	int32_t (*ipc_hw_get_hw_counter)(const uint8_t endid, const uint8_t fid, msgbox_hw_counter_t *hw_cnt);
	int32_t (*ipc_hw_clr_hw_counter)(const uint8_t endid, const uint8_t fid, const uint32_t flag);
};
#define libipc_hw_compat_ops_t struct _libipc_hw_compat_ops_t

#endif
