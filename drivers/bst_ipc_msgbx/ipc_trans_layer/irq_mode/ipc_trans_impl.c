// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/* This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is also distributed under the terms of the BSD 3-Clause
 * License.
 *
 * Copyright (C) 2024 Black Sesame Technologies. Inc.
 */
/**
 * @file  ipc_trans_impl.c
 * @brief This file provides the implementation for the IPC transfer protocol
 * compatibility layer. Refer to ../include/ipc_trans_layer.h for the API
 * definitions to implement, and utilize the APIs available in
 * ./src/ipc_runtime.h as needed.
 * @details
 * feature list
 * 1. stub api implementation
 * 2. proxy api implementation
 * 3. state management process
 */
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <bst/config.h>
#include <bst/ipc_trans_layer.h>

#include "../src/ipc_trans_runtime.h"

/* performance optimization spec start */
// device info buffer management
static struct completion recv_comp[8][CHANNEL_COUNT][SESSION_COUNT];
#ifdef CONFIG_C1200_SLT
static struct completion loopback_test;
static rw_msg_t loopback_msg;
static uint8_t loopback_fid;
#endif
static bool session_exit[8][CHANNEL_COUNT][SESSION_COUNT];
#ifdef CONFIG_C1200_SLT
struct task_struct *loopback = NULL;
EXPORT_SYMBOL(loopback);
#endif
#define REMOTE_LOG_CPUID 0

static void completion_init(struct completion *comp)
{
	init_completion(comp);
}

void completion_all_init(void)
{
	int i, j, k;
	for (i = 0; i < 8; i++)
		for (j = 0; j < CHANNEL_COUNT; j++)
			for (k = 0; k < SESSION_COUNT; k++)
				completion_init(&recv_comp[i][j][k]);
#ifdef CONFIG_C1200_SLT
	completion_init(&loopback_test);
#endif
}
EXPORT_SYMBOL(completion_all_init);

/* Note: performance optimization spec function, if you do not need any
 * completion, you can return 0 directly.
 */
int32_t ipc_trans_complete(const uint8_t cpuid, const uint8_t ses_id)
{
	uint8_t fid, sid;

	session_dist(ses_id, &sid, &fid);
	complete(&recv_comp[cpuid][fid][sid]);
	return 0;
}
/* performance optimization spec end */

/* central end status monitor spec start */
/* Note: central end status monitor spec function, if you do not need any
 * completion, you can return 0 directly.
 */
int32_t ipc_trans_complete_sts(const uint8_t cpuid)
{
	return 0;
}

#ifdef CONFIG_C1200_SLT
/*
 * test for slt
 * fid: filter id
 * msg: the pointer to looback msg
 */
int32_t ipc_trans_complete_test(const uint8_t fid, const rw_msg_t *msg)
{

	IPC_LOG_DEBUG("recv from fid %d msg\n", fid);
	loopback_msg = *msg;
	loopback_fid = fid;
	complete(&loopback_test);
	return 0;
}

static int loopback_process_thread(void *arg)
{
	int32_t ret = -1;
	rw_msg_t reply;
	uint8_t cpuid = 0;

	ENDID_TO_CPUID(msgbx_get_start_pid());

	while (!kthread_should_stop()) {
		wait_for_completion(&loopback_test);
		IPC_LOG_DEBUG("recv loopback msg cmd = %d, len = %d\n",
				loopback_msg.header.cmd, loopback_msg.header.len);
		reply = loopback_msg;
		reply.header.pid = loopback_msg.header.cid;
		reply.header.cid = loopback_msg.header.pid;
		if (loopback_msg.header.cmd == 0) {
			reply.header.cmd = 1;
		}
		cpuid = get_cpuid_by_endid(loopback_msg.header.cid);
		ret = ipc_hw_layer_send_msg(cpuid, &reply);
		if(ret < 0)
			pr_err("ipc hw send msg ret = %d\n", ret);
	}

	return 0;
}
#endif

uint8_t get_cpuid_by_endid(uint8_t end_id)
{
	uint8_t cpuid = 0;
	u8 start_pid = msgbx_get_start_pid();

	cpuid = end_id - start_pid;

	if (cpuid >= NR_CPUS) {
		pr_info(KERN_ERR "\n cpuid:%u error end_id:%u end0_id:%u\n",
			   cpuid, end_id, start_pid);
		cpuid = CPUID_ERR;
	}
	return cpuid;
}
EXPORT_SYMBOL(get_cpuid_by_endid);

/* error message process, call ipc_trans_err_hdl to pass fault handle command.
 * handle command is TBD
 */
int err_func(void *addr, msgbx_err_msg_t *err_msg)
{
	pr_info("recv err msg, err_msg.type = %d, id = %d, msg = %d ",
		     err_msg->type, err_msg->id, err_msg->msg);
// #ifdef IPC_STATE_MGT_ENABLE
// 	ipc_trans_err_hdl(err_msg->type, err_msg->id, 0, addr);
// #endif
// 	pr_info("ipc_trans_err_hdl success!!\n");
	return 0;
}

/* remote log process, you can choose to save it in file or print it directly.
 */
#ifdef ENABLE_REMOTE_LOG_PROCESS_FUNC
static int remote_log_process_thread(void *arg)
{
	uint8_t type = 0;
	int32_t ret = -1;
	serdes_t log = { 0 };
	char msg[128];
	char *print_msg = msg;

	ENDID_TO_CPUID(msgbx_get_start_pid());

	while (1) {
		wait_for_completion(
			&recv_comp[REMOTE_LOG_CPUID][REMOTE_LOG_SES_FID][REMOTE_LOG_SES_SID]);
		ret = ipc_trans_get_avail_info(REMOTE_LOG_SES_ID, &type,
						   g_ipc_end_array[REMOTE_LOG_CPUID]);
		if (ret < 0)
			continue;
		ret = ipc_trans_get_msg(REMOTE_LOG_SES_ID, MSGBX_MSG_TYPE_METHOD,
					&log, g_ipc_end_array[REMOTE_LOG_CPUID]);
		if (ret < 0)
			continue;
		ipc_des_get_string(&log, print_msg, 128);
		pr_info("--------------pid %d print log: %s\n", log.header.pid,
			   print_msg);
	}
}
#endif
/**
 * @name start ipc driver
 * @attention only driver api
 * @details
 * 1. MsgBx hw layer init
 * 2. register receive message callback function
 * 3. set state management configuration, if necessary
 * NOTE:
 * 1. role in here is just a special demo MUST parameter.
 * 2. This API is invoked multiple times per session. To support multiple sessions for the msgbx driver, implement the
 * following function once during driver initialization. Subsequent calls to this API by the app should return 0
 * directly.
 */
int32_t ipc_trans_layer_start(const uint8_t endid, uint8_t role)
{
	int32_t ret = -1;
#ifdef ENABLE_REMOTE_LOG_PROCESS_FUNC
	struct task_struct *log_task = NULL;
#endif
	u8 cpuid = ENDID_TO_CPUID(endid);
	
#if defined(USE_EXTERNAL_MSG_BUFFER)
	g_ipc_end_array[cpuid]->ses_map = g_ipc_end_ses_map[cpuid];
#endif
	ret = ipc_trans_init(role, err_func, g_ipc_end_array[cpuid]);

	if (unlikely(ret))
		pr_info("ipc_trans_init error\n");

#ifdef ENABLE_REMOTE_LOG_PROCESS_FUNC
	/* Note: Implement the central end remote log clustering specification*/
	if (cpuid == REMOTE_LOG_CPUID) {
		log_task = kthread_run(remote_log_process_thread, NULL,
					"remote_log_thread");
		if (unlikely(!log_task))
			pr_info("pthread_create remote log process thread error\n");
	}

#endif
#ifdef CONFIG_C1200_SLT
	loopback = kthread_run(loopback_process_thread, NULL,
				"msgbox_slt_loopback");
	if (unlikely(!loopback))
		pr_err("cna't create msgbox_slt_loopback task\n");
#endif

	return ret;
}

/**
 * @name stop ipc driver
 * @attention only driver api
 * @details
 * NOTE:
 * 1. This API is invoked multiple times per session.To support multiple sessions for the msgbx driver,implement the
 * following function once during driver initialization. Subsequent calls to this API by the app should return 0
 * directly.
 */
int32_t ipc_trans_layer_stop(const uint8_t endid)
{
	// transfer endid to device info buffer address
	u8 cpuid = ENDID_TO_CPUID(endid);

	return ipc_trans_deinit(g_ipc_end_array[cpuid]);
}

/**
 * @name get info change
 * @attention both client and server api
 * @details
 * 1. query available message handle and type
 * 2. query any end status changed
 * NOTE: type is reserved, in this demo, we do not pass type information to
 * upper layer.
 * @return
 * ret < 0 means do not have available info changed, you should still wait or do other process. ret > 0 means end
 * status change ret = 0 means get receiving message
 */

/* Note: performance optimization spec implementation, please refer to
 * programmers' guide
 */
int32_t ipc_trans_layer_query_info(const uint8_t endid, const uint8_t handle, ...)
{
	uint8_t type = 0;
	uint8_t fid = 0, sid = 0;
	int32_t ret = -1;
	uint8_t is_from_user = false;
	va_list ap_ptr;
	u8 cpuid = ENDID_TO_CPUID(endid);

	va_start(ap_ptr, handle);
	is_from_user = va_arg(ap_ptr, int) == 1 ? true : false;
	va_end(ap_ptr);

	session_dist(handle, &sid, &fid);
	if (is_from_user) {
		wait_for_completion_timeout(&recv_comp[cpuid][fid][sid], msecs_to_jiffies(500));
		return ipc_trans_get_avail_info(handle, &type, g_ipc_end_array[cpuid]);
	}
	ret = ipc_trans_get_avail_info(handle, &type, g_ipc_end_array[cpuid]);
	while (ret < 0 && !session_exit[cpuid][fid][sid]) {
		wait_for_completion(&recv_comp[cpuid][fid][sid]);
		ret = ipc_trans_get_avail_info(handle, &type,
						   g_ipc_end_array[cpuid]);
	}
	return ret;
}
EXPORT_SYMBOL(ipc_trans_layer_query_info);

/**
 * @name session init
 * @attention server api only
 * @details
 * 1. fid, sid check
 * 2. register session
 * 3. return session id
 */
/* Note: Implement performance optimization specifications by adding completion_init and establishing relationships
 * between each session and its corresponding completion.
 */
int32_t ipc_trans_layer_stub_create_handle(const uint8_t endid,
					   const uint8_t fid, const uint8_t sid,
					   const uint8_t cid, uint8_t *handle)
{
	uint8_t ses_id = 0;
	int32_t ret = -1;
	u8 cpuid = ENDID_TO_CPUID(endid);

	if (endid != g_ipc_end_array[cpuid]->g_ipc_pid)
		return -ERR_PID_IS_INVALID;

	// transfer endid to device info buffer address
	ret = ipc_trans_create_session(sid, fid, cid, MSGBX_SES_ROLE_SERVER,
					   &ses_id, g_ipc_end_array[cpuid]);
	if (ret < 0)
		return ret;
	*handle = ses_id;

	session_exit[cpuid][fid][sid] = false;

	return 0;
}
EXPORT_SYMBOL(ipc_trans_layer_stub_create_handle);

/**
 * @name send a reply message
 * @attention server api only
 */
int32_t ipc_trans_layer_stub_send_reply_msg(const uint8_t endid,
						const uint8_t handle, serdes_t *msg)
{
	u8 cpuid = ENDID_TO_CPUID(endid);

	return ipc_trans_send_msg(handle, msg, MSGBX_MSG_TYPE_REPLY,
				  g_ipc_end_array[cpuid]);
}
EXPORT_SYMBOL(ipc_trans_layer_stub_send_reply_msg);

/**
 * @name send a signal message
 * @attention server api only
 */
int32_t ipc_trans_layer_stub_send_broadcast(const uint8_t endid,
						const uint8_t handle, serdes_t *msg)
{
	u8 cpuid = ENDID_TO_CPUID(endid);

	return ipc_trans_send_msg(handle, msg, MSGBX_MSG_TYPE_BROADCAST,
				  g_ipc_end_array[cpuid]);
}
EXPORT_SYMBOL(ipc_trans_layer_stub_send_broadcast);

int32_t ipc_trans_layer_register_method(const uint8_t endid,
					const uint8_t handle, const uint8_t cmd)
{
#if (SESSION_COUNT > 1)
	u8 cpuid = ENDID_TO_CPUID(endid);

	return ipc_trans_register_method(handle, cmd, g_ipc_end_array[cpuid]);
#endif
	return 0;
}
EXPORT_SYMBOL(ipc_trans_layer_register_method);

/**
 * @name server unregister method
 * @attention server api only
 * @details
 * 1. fid, sid check
 * 2. register cmd in method map
 */
int32_t ipc_trans_layer_unregister_method(const uint8_t endid,
					  const uint8_t handle)
{
#if (SESSION_COUNT > 1)
	u8 cpuid = ENDID_TO_CPUID(endid);

	return ipc_trans_unregister_method(handle, g_ipc_end_array[cpuid]);
#endif
	return 0;
}
EXPORT_SYMBOL(ipc_trans_layer_unregister_method);

/**
 * @name receive method message
 * @attention server api only
 * @details
 */
int32_t ipc_trans_layer_stub_get_method_msg(const uint8_t endid,
						const uint8_t handle, serdes_t *msg)
{
	u8 cpuid = ENDID_TO_CPUID(endid);

	return ipc_trans_get_msg(handle, MSGBX_MSG_TYPE_METHOD, msg,
				 g_ipc_end_array[cpuid]);
}
EXPORT_SYMBOL(ipc_trans_layer_stub_get_method_msg);

/**
 * @name client session init
 * @attention client api only
 * @details
 * 1. fid, sid check
 * 2. register session
 * 3. return session id
 */
/* Note: Implement performance optimization specifications by adding completion_init and establishing relationships
 * between each session and its corresponding completion.
 */
int32_t ipc_trans_layer_proxy_create_handle(const uint8_t endid,
						const uint8_t fid,
						const uint8_t sid,
						const uint8_t cid, uint8_t *handle)
{
	uint8_t ses_id = 0;
	int32_t ret = -1;
	u8 cpuid = ENDID_TO_CPUID(endid);

	if (endid != g_ipc_end_array[cpuid]->g_ipc_pid)
		return -ERR_PID_IS_INVALID;

	// transfer endid to device info buffer address
	ret = ipc_trans_create_session(sid, fid, cid, MSGBX_SES_ROLE_CLIENT,
					   &ses_id, g_ipc_end_array[cpuid]);
	if (ret < 0)
		return ret;
	*handle = ses_id;

	session_exit[cpuid][fid][sid] = false;

	return 0;
}
EXPORT_SYMBOL(ipc_trans_layer_proxy_create_handle);

/**
 * @name get a sinal message
 * @attention client api only
 * @details
 */
int32_t ipc_trans_layer_proxy_get_broadcast_msg(const uint8_t endid,
						const uint8_t handle,
						serdes_t *msg)
{
	u8 cpuid = ENDID_TO_CPUID(endid);

	return ipc_trans_get_msg(handle, MSGBX_MSG_TYPE_BROADCAST, msg,
				 g_ipc_end_array[cpuid]);
}
EXPORT_SYMBOL(ipc_trans_layer_proxy_get_broadcast_msg);

/**
 * @name send a method message
 * @attention client api only
 */
int32_t ipc_trans_layer_proxy_send_method(const uint8_t endid,
					  const uint8_t handle, serdes_t *msg)
{
	u8 cpuid = ENDID_TO_CPUID(endid);

	return ipc_trans_send_msg(handle, msg, MSGBX_MSG_TYPE_METHOD,
				  g_ipc_end_array[cpuid]);
}
EXPORT_SYMBOL(ipc_trans_layer_proxy_send_method);

/**
 * @name get a reply message
 * @attention client api only
 * @details
 */
int32_t ipc_trans_layer_proxy_get_reply_msg(const uint8_t endid,
						const uint8_t handle, serdes_t *msg)
{
	u8 cpuid = ENDID_TO_CPUID(endid);

	return ipc_trans_get_msg(handle, MSGBX_MSG_TYPE_REPLY, msg,
				 g_ipc_end_array[cpuid]);
}
EXPORT_SYMBOL(ipc_trans_layer_proxy_get_reply_msg);

/**
 * @name handle destroy
 * @attention both client and server api
 * @details
 */
/* Note: Demo code. Upon session termination, we output its associated
 * diagnostic information.
 */
int32_t ipc_trans_layer_destroy_handle(const uint8_t endid,
					   const uint8_t handle)
{
#ifdef DEBUG_MODE_ENABLE
	debug_info_t info = { 0 };
	u8 cpuid = ENDID_TO_CPUID(endid);

	ipc_trans_get_debug_info(handle, &info, g_ipc_end_array[cpuid]);
	pr_info("\nsession %u info role: %u, send_msg_cnt:%u, send_rw_msg_cnt:%u, "
		   "send_fail_cnt:%u, last_msg_frame_cnt:%u \n",
		   handle, info.role, info.send_msg_cnt, info.send_rw_msg_cnt,
		   info.send_fail_cnt, info.send_frame_cnt);
	pr_info("recv_msg_1_cnt:%u, recv_msg_2_cnt:%u, recv_rw_msg_cnt:%u \n", info.recv_msg_1_cnt,
		   info.recv_msg_2_cnt, info.recv_rw_msg_cnt);
	pr_info("send_start_time: %llu, send_end_time:%llu, collate_time:%llu, "
		   "get_msg_time:%llu \n",
		   info.send_start_time, info.send_end_time, info.collate_time,
		   info.get_msg_time);
#endif
	return ipc_trans_close_session(handle, g_ipc_end_array[cpuid]);
}
EXPORT_SYMBOL(ipc_trans_layer_destroy_handle);

/* Note: To implement the performance optimization specification, this function is invoked when an application needs to
 * stop a receiving thread, thereby releasing any blocking on the receiving thread.
 */
int32_t ipc_trans_layer_release_recv_wait(const uint8_t endid,
					  const uint8_t handle)
{
	uint8_t fid = 0, sid = 0;
	u8 cpuid = ENDID_TO_CPUID(endid);

	session_dist(handle, &sid, &fid);
	session_exit[cpuid][fid][sid] = true;
	complete(&recv_comp[cpuid][fid][sid]);

	return 0;
}
EXPORT_SYMBOL(ipc_trans_layer_release_recv_wait);

/**
 * @name session init
 * @attention fast path app only
 * @details
 * 1. fid, sid check
 * 2. register session
 * 3. return session id
 */
int32_t ipc_trans_layer_create_handle(const uint8_t endid, const uint8_t fid,
				      const uint8_t sid, const uint8_t cid,
				      uint8_t *handle)
{
	uint8_t ses_id = 0;
	int32_t ret = -1;
	u8 cpuid = ENDID_TO_CPUID(endid);

	if (endid != g_ipc_end_array[cpuid]->g_ipc_pid)
		return -ERR_PID_IS_INVALID;

	// transfer endid to device info buffer address
	ret = ipc_trans_create_session(sid, fid, 0, MSGBX_SES_ROLE_FASTPATH,
				       &ses_id, g_ipc_end_array[cpuid]);
	if (ret < 0)
		return ret;
	*handle = ses_id;

#if !defined(BAREMETAL_VERSION_TRUNCATE)
	completion_init(&recv_comp[cpuid][fid][sid]);
	session_exit[cpuid][fid][sid] = false;
#endif

	return 0;
}
EXPORT_SYMBOL(ipc_trans_layer_create_handle);

int32_t ipc_trans_layer_send_msg(const uint8_t endid, const uint8_t handle,
				 rw_msg_t *msg)
{
	u8 cpuid = ENDID_TO_CPUID(endid);

	return ipc_trans_send_rwmsg(handle, msg, g_ipc_end_array[cpuid]);
}
EXPORT_SYMBOL(ipc_trans_layer_send_msg);

/**
 * @name receive message
 * @attention fast path app only
 * @details
 */
int32_t ipc_trans_layer_get_msg(const uint8_t endid, const uint8_t handle,
				const int32_t timeout, rw_msg_t *msg,
				uint64_t *timestamp)
{
#if defined(IPC_RTE_KERNEL)
	uint8_t fid = 0, sid = 0;
	int32_t ret = -1;
	u8 cpuid = ENDID_TO_CPUID(endid);

	if (session_isvalid(handle, g_ipc_end_array[cpuid]) < 0)
		return -ERR_SES_IS_INVALID;

	session_dist(handle, &sid, &fid);
	if (timeout == -1)
		wait_for_completion(&recv_comp[cpuid][fid][sid]);
	else {
		ret = wait_for_completion_timeout(&recv_comp[cpuid][fid][sid],
						  msecs_to_jiffies(timeout));
		if (ret == 0)
			//timeout
			return -ERR_FASTPAH_GET_TIMEOUT;
	}

	ret = ipc_trans_get_rwmsg(handle, msg, timestamp, g_ipc_end_array[cpuid]);
	return ret;
#else
	return ipc_trans_get_rwmsg(handle, msg, timestamp, &dev_info);
#endif
}
EXPORT_SYMBOL(ipc_trans_layer_get_msg);
