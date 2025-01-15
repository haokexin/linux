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
 * @file  ipc_trans_runtime.h
 * @brief this file is used as the api to access ipc tranferring layer.
 * this api may be used by ipc_app_layer directly, or related driver developers
 * may implement ipc_trans_impl.c to adapt to different OSs.
 * @note
 * @details feature list
 */
#ifndef _IPC_TRANS_RUNTIME_H
#define _IPC_TRANS_RUNTIME_H

#include <bst/ipc_hw_common.h>
#include <bst/ipc_hw_layer.h>
#include <bst/ipc_hw_impl.h>
#include <bst/bstipc_cfg.h>
#include <bst/ipc_trans_common.h>

#include "../include/config.h"

#include "ipc_trans_ses_mgt.h"
#include "ipc_trans_cfg.h"
#include "ipc_trans_sts_mgt.h"

#ifdef __cplusplus
extern "C" {
#endif

struct _debug_info_t {
	uint8_t role;
#ifdef DEBUG_MODE_ENABLE
	_Atomic uint8_t send_fail_cnt;
	uint8_t send_frame_cnt;
	uint8_t reserved;

	_Atomic uint32_t send_msg_cnt;
	_Atomic uint32_t send_rw_msg_cnt;

	_Atomic uint32_t recv_rw_msg_cnt;
	_Atomic uint32_t recv_msg_1_cnt;
	_Atomic uint32_t recv_msg_2_cnt;
#endif
#ifdef TIMESTAMP_DEBUG_ENABLE
	uint64_t send_start_time;
	uint64_t send_end_time;
	uint64_t collate_time;
	uint64_t get_msg_time;
#endif
};
#define debug_info_t struct _debug_info_t

struct _msgbx_end_device_t {
	// basic hw ops and info
	libipc_hw_compat_ops_t ops;
#if !defined(BAREMETAL_VERSION_TRUNCATE)
	msgbx_hw_info_t hw_info;
	int16_t method_register_map[CMD_MAX_COUNT];
#endif
#if !defined(USE_EXTERNAL_MSG_BUFFER)
	ipc_ses_t ses_map[CHANNEL_COUNT][SESSION_COUNT];
#else
	ipc_ses_t **ses_map;
#endif
	ipc_flt_cfg_t cfg_map[CHANNEL_COUNT - 1];
	uint8_t g_ipc_pid;
	uint8_t g_ipc_flt_cnt;
#ifndef REMOVE_STS_MGT
	int16_t g_update_ses_list[CHANNEL_COUNT * SESSION_COUNT];
	uint8_t g_req_endmap_cid;
	_Atomic uint64_t g_end_sts_map;
#endif
#ifdef ENABLE_REMOTE_LOG_PROCESS_FUNC
	uint8_t g_log_tok;
#endif
#ifdef DEBUG_MODE_ENABLE
	debug_info_t debug_info[CHANNEL_COUNT][SESSION_COUNT];
#endif
};
#define msgbx_end_device_t struct _msgbx_end_device_t

#define TRANS_LAYER_VERSION 2
#define IPC_HW_LAYER_VERSION 1

extern msgbx_end_device_t *g_ipc_end_array[NR_CPUS];
extern ipc_ses_t **g_ipc_end_ses_map[NR_CPUS];
#ifdef CONFIG_C1200_SLT
extern struct task_struct *loopback;
#endif

/********************* extern global function *******************/
extern int32_t ipc_trans_complete(const uint8_t cpuid, const uint8_t ses_id);
extern int32_t ipc_trans_complete_sts(const uint8_t cpuid);
#ifdef CONFIG_C1200_SLT
extern int32_t ipc_trans_complete_test(const uint8_t fid, const rw_msg_t *msg);
#endif

static inline int8_t session_dist(uint8_t info, uint8_t *sid, uint8_t *fid)
{
	*sid = info >> 4;
	*fid = info & 0x0f;
	return 0;
}

int32_t ipc_trans_layer_start(const uint8_t endid, uint8_t role);
int32_t ipc_trans_layer_stop(const uint8_t endid);

int32_t ipc_trans_init(const uint8_t role, err_msg_callback err_func,
		       void *dev_info);
int32_t ipc_trans_deinit(void *dev_info);
int32_t ipc_trans_reinit(void *dev_info);

int32_t ipc_trans_create_session(const uint8_t sid, const uint8_t fid,
				 const uint8_t cid, const uint8_t role,
				 uint8_t *ses_id, void *dev_info);
int32_t ipc_trans_close_session(const uint8_t ses_id, void *dev_info);

int32_t ipc_trans_read_msg(const uint8_t fid, const uint8_t mode,
			   void *dev_info);

int32_t ipc_trans_send_msg(const uint8_t ses_id, serdes_t *msg,
			   const uint8_t type, void *dev_info);
int32_t ipc_trans_get_msg(const uint8_t ses_id, const ipc_msg_type_t msg_typ,
			  serdes_t *msg, void *dev_info);

int32_t ipc_trans_register_method(const uint8_t session_id, const uint8_t cmd,
				  void *dev_info);
int32_t ipc_trans_unregister_method(const uint8_t session_id, void *dev_info);

int32_t ipc_trans_get_avail_info(const uint8_t session_id, uint8_t *type,
				 void *dev_info);

int32_t ipc_trans_err_hdl(const uint8_t type, const uint8_t id,
			  const uint32_t hdl, void *dev_info);

int32_t ipc_trans_get_debug_info(const uint8_t ses_id, debug_info_t *info,
				 void *dev_info);
int32_t ipc_trans_end_sts_broadcast(void *addr);

int32_t ipc_trans_transmit_log(const uint8_t cid, const char *log,
			       void *dev_info);
int32_t ipc_trans_query_remote_endmap(void *dev_info);

int32_t ipc_trans_send_rwmsg(const uint8_t ses_id, rw_msg_t *msg,
			     void *dev_info);
int32_t ipc_trans_get_rwmsg(const uint8_t ses_id, rw_msg_t *msg,
			    uint64_t *timestamp, void *dev_info);
int8_t session_isvalid(const uint8_t session_id, void *addr);
#ifdef __cplusplus
}
#endif
#endif
