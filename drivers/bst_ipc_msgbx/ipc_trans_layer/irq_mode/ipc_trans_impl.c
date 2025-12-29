// SPDX-License-Identifier: GPL-2.0 OR Apache 2.0
/*
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
#include <linux/device.h>
#include <linux/platform_device.h>
#include <bst/ipc_trans_layer.h>
#include "../src/ipc_trans_runtime.h"

/* performance optimization spec start */
// device info buffer management
static struct completion recv_comp[8][CHANNEL_COUNT * SESSION_COUNT];
static bool session_exit[8][CHANNEL_COUNT * SESSION_COUNT];
msgbx_end_device_t *g_ipc_end_array[NR_CPUS] = {NULL};
static struct mutex g_end_ses_create_mtx[NR_CPUS];
#if defined(CONFIG_C1200_SLT) || defined(CONFIG_C1200_MASS)
static struct completion loopback_test;
static rw_msg_t loopback_msg;
static uint8_t loopback_fid;
#endif
#if defined(CONFIG_C1200_SLT) || defined(CONFIG_C1200_MASS)
struct task_struct *loopback = NULL;
EXPORT_SYMBOL(loopback);
#endif

static void completion_init(struct completion *comp)
{
	init_completion(comp);
}

void completion_all_init(void)
{
	int i, j;
	for (i = 0; i < 8; i++)
		for (j = 0; j < CHANNEL_COUNT * SESSION_COUNT; j++)
				completion_init(&recv_comp[i][j]);
#if defined(CONFIG_C1200_SLT) || defined(CONFIG_C1200_MASS)
	completion_init(&loopback_test);
#endif
}
EXPORT_SYMBOL(completion_all_init);

/* Note: performance optimization spec function, if you do not need any
 * completion, you can return 0 directly.
 */
int32_t ipc_trans_complete(const uint8_t cpuid, const uint8_t ses_id)
{
	complete(&recv_comp[cpuid][ses_id]);
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

#if defined(CONFIG_C1200_SLT) || defined(CONFIG_C1200_MASS)
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
		if (loopback_msg.header.cmd == 255) {
			reply.header.typ = MSGBX_MSG_TYPE_USERDEFINED;
		}
		cpuid = get_cpuid_by_endid(loopback_msg.header.cid);
		ret = ipc_hw_layer_send_msg(cpuid, &reply);
		if(ret < 0)
			pr_err("ipc hw send msg ret = %d\n", ret);
	}

	return 0;
}
#endif

/* error message process, call ipc_trans_err_hdl to pass fault handle command.
 * handle command is TBD
 */
int err_func(void *addr, msgbx_err_msg_t *err_msg)
{
	pr_err("recv err msg, err_msg.fid = %d, err_code = %d\n",
		err_msg->fid, err_msg->err_code);
	return 0;
}

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
	u8 cpuid = ENDID_TO_CPUID(endid);

	// g_ipc_end_array[cpuid]->g_ipc_cpuid = cpuid;
	completion_all_init();
	ret = ipc_trans_init(role, err_func, g_ipc_end_array[cpuid]);
	if (unlikely(ret))
		pr_info("ipc_trans_init error\n");
	mutex_init(&g_end_ses_create_mtx[cpuid]);
#if defined(CONFIG_C1200_SLT) || defined(CONFIG_C1200_MASS)
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
int32_t ipc_trans_layer_query_info(const uint8_t endid, const uint8_t handle,
						const uint32_t polling_times, const uint8_t is_from_user)
{
	int32_t ret = -1;
	u8 cpuid = ENDID_TO_CPUID(endid);
	uint32_t times = 0;

	if (is_from_user) {
		ret = ipc_trans_get_avail_info(handle, g_ipc_end_array[cpuid]);
		while (ret < 0 && !session_exit[cpuid][handle]) {
			if (times < polling_times) {
				++times;
				schedule();
				ret = ipc_trans_get_avail_info(handle, g_ipc_end_array[cpuid]);
			} else {
				wait_for_completion_timeout(&recv_comp[cpuid][handle], msecs_to_jiffies(500));
				ret = ipc_trans_get_avail_info(handle, g_ipc_end_array[cpuid]);
				return ret;
			}
		}
		return ret;
	}
	ret = ipc_trans_get_avail_info(handle, g_ipc_end_array[cpuid]);
	while (ret < 0 && !session_exit[cpuid][handle]) {
		wait_for_completion(&recv_comp[cpuid][handle]);
		ret = ipc_trans_get_avail_info(handle, g_ipc_end_array[cpuid]);
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
	void* msg_queue_addr = NULL;
	u8 cpuid = ENDID_TO_CPUID(endid);

	if (endid != g_ipc_end_array[cpuid]->g_ipc_pid)
		return -ERR_PID_IS_INVALID;

	// transfer endid to device info buffer address
	mutex_lock(&g_end_ses_create_mtx[cpuid]);
	ret = ipc_trans_create_session(sid, fid, cid, 0, MSGBX_SES_ROLE_SERVER,
				       &ses_id, g_ipc_end_array[cpuid]);
	mutex_unlock(&g_end_ses_create_mtx[cpuid]);
	if (ret < 0)
		return ret;

	// alloc msg queue and align
	msg_queue_addr = kzalloc(sizeof(bst_msg_queue_t), GFP_KERNEL);
	if (!msg_queue_addr) {
		IPC_LOG_INFO("pid %u fid %u sid %u alloc msg_queue error", endid, fid, sid);
		return -ERR_CREATE_SES_ALLOC_FAIL;
	}
	ret = ipc_trans_msg_queue_alloc(ses_id, msg_queue_addr, g_ipc_end_array[cpuid]);
	if (ret < 0)
		return ret;
	*handle = ses_id;
	session_exit[cpuid][ses_id] = false;
	return ret;
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

	return ipc_trans_send_msg(handle, msg, MSGBX_MSG_TYPE_REPLY, g_ipc_end_array[cpuid]);
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

	return ipc_trans_send_msg(handle, msg, MSGBX_MSG_TYPE_BROADCAST, g_ipc_end_array[cpuid]);
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
					    const uint8_t cid, const uint8_t ccid, uint8_t *handle)
{
	uint8_t ses_id = 0;
	int32_t ret = -1;
	void* msg_queue_addr = NULL;
	u8 cpuid = ENDID_TO_CPUID(endid);

	if (endid != g_ipc_end_array[cpuid]->g_ipc_pid)
		return -ERR_PID_IS_INVALID;

	// transfer endid to device info buffer address
	mutex_lock(&g_end_ses_create_mtx[cpuid]);
	ret = ipc_trans_create_session(sid, fid, cid, ccid, MSGBX_SES_ROLE_CLIENT,
					   &ses_id, g_ipc_end_array[cpuid]);
	mutex_unlock(&g_end_ses_create_mtx[cpuid]);
	if (ret < 0)
		return ret;

	// alloc msg queue and align
	msg_queue_addr = kzalloc(sizeof(bst_msg_queue_t), GFP_KERNEL);
	if (!msg_queue_addr) {
		IPC_LOG_INFO("pid %u fid %u sid %u alloc msg_queue error", endid, fid, sid);
		return -ERR_CREATE_SES_ALLOC_FAIL;
	}
	ret = ipc_trans_msg_queue_alloc(ses_id, msg_queue_addr, g_ipc_end_array[cpuid]);
	if (ret < 0)
		return ret;
	*handle = ses_id;
	session_exit[cpuid][ses_id] = false;
	return ret;
}
EXPORT_SYMBOL(ipc_trans_layer_proxy_create_handle);

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
int32_t ipc_trans_layer_get_msg(const uint8_t endid,
						const uint8_t handle, serdes_t *msg)
{
	u8 cpuid = ENDID_TO_CPUID(endid);
	return ipc_trans_get_msg(handle, msg, g_ipc_end_array[cpuid]);
}
EXPORT_SYMBOL(ipc_trans_layer_get_msg);

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
	int32_t ret = -1;
	void* msg_queue_addr = NULL;
	u8 cpuid = ENDID_TO_CPUID(endid);
	debug_info_t info = { 0 };

	ret = ipc_trans_get_debug_info(handle, &info, g_ipc_end_array[cpuid]);
	if (ret < 0)
		return 0;

	// msgbx_clr_max_send_time(cpuid, handle);

	pr_debug("\nsession %u info cid: %u, role: %u, send_msg_cnt:%u, send_rw_msg_cnt:%u, "
		"send_fail_cnt:%u\n",handle, info.cid, info.role, info.send_msg_cnt, info.send_rw_msg_cnt,
		info.send_fail_cnt);
	pr_debug("recv_msg_1_cnt:%u, recv_msg_2_cnt:%u, recv_rw_msg_cnt:%u in_rw_msg_cnt:%u\n", info.recv_msg_1_cnt,
		info.recv_msg_2_cnt, info.recv_rw_msg_cnt, info.in_rw_msg_cnt);
	pr_debug("send_start_time: %llu, send_end_time:%llu\n", info.send_start_time, info.send_end_time);

	ret = ipc_trans_map_session(handle, &msg_queue_addr, g_ipc_end_array[cpuid]);
	if (ret < 0)
		return 0;

	ret = ipc_trans_close_session(handle, g_ipc_end_array[cpuid]);
	if (ret < 0)
		return 0;

	if (msg_queue_addr) {
		kfree(msg_queue_addr);
		g_ipc_end_array[cpuid]->ses_map[handle].msg_queue = NULL;
	}
	return ret;
}
EXPORT_SYMBOL(ipc_trans_layer_destroy_handle);

/* Note: To implement the performance optimization specification, this function is invoked when an application needs to
 * stop a receiving thread, thereby releasing any blocking on the receiving thread.
 */
int32_t ipc_trans_layer_release_recv_wait(const uint8_t endid,
					  const uint8_t handle)
{
	u8 cpuid = ENDID_TO_CPUID(endid);
	session_exit[cpuid][handle] = true;
	complete(&recv_comp[cpuid][handle]);

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
	void* msg_queue_addr = NULL;
	u8 cpuid = ENDID_TO_CPUID(endid);

	if (endid != g_ipc_end_array[cpuid]->g_ipc_pid)
		return -ERR_PID_IS_INVALID;

	// transfer endid to device info buffer address
	mutex_lock(&g_end_ses_create_mtx[cpuid]);
	ret = ipc_trans_create_session(sid, fid, 0, 0, MSGBX_SES_ROLE_FASTPATH,
				       &ses_id, g_ipc_end_array[cpuid]);
	mutex_unlock(&g_end_ses_create_mtx[cpuid]);
	if (ret < 0)
		return ret;

	// alloc msg queue and align
	msg_queue_addr = kzalloc(sizeof(bst_msg_queue_t), GFP_KERNEL);
	if (!msg_queue_addr) {
		IPC_LOG_INFO("pid %u fid %u sid %u alloc msg_queue error", endid, fid, sid);
		return -ERR_CREATE_SES_ALLOC_FAIL;
	}
	ret = ipc_trans_msg_queue_alloc(ses_id, msg_queue_addr, g_ipc_end_array[cpuid]);
	if (ret < 0)
		return ret;
	*handle = ses_id;

#if !defined(BAREMETAL_VERSION_TRUNCATE)
	completion_init(&recv_comp[cpuid][ses_id]);
	session_exit[cpuid][ses_id] = false;
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
int32_t ipc_trans_layer_get_rwmsg(const uint8_t endid, const uint8_t handle,
				const int32_t timeout, rw_msg_t *msg,
				uint64_t *timestamp)
{
#if defined(IPC_RTE_KERNEL)
	int32_t ret = -1;
	u8 cpuid = ENDID_TO_CPUID(endid);

	if (timeout == -1)
		wait_for_completion(&recv_comp[cpuid][handle]);
	else {
		ret = wait_for_completion_timeout(&recv_comp[cpuid][handle],
						  msecs_to_jiffies(timeout));
		if (ret == 0)
			//timeout
			return -ERR_FASTPAH_GET_TIMEOUT;
	}

	ret = ipc_trans_get_rwmsg(handle, msg, timestamp, g_ipc_end_array[cpuid]);
	return ret;
#else
	return ipc_trans_get_rwmsg(handle, msg, timestamp, g_ipc_end_array[cpuid]);
#endif
}
EXPORT_SYMBOL(ipc_trans_layer_get_rwmsg);

int32_t ipc_trans_layer_mmap_session(const uint8_t endid, const uint8_t handle, void** ses_addr)
{
	int32_t ret = 0;
	u8 cpuid = ENDID_TO_CPUID(endid);
	ret = ipc_trans_map_session(handle, ses_addr, g_ipc_end_array[cpuid]);
	return 0;
}
EXPORT_SYMBOL(ipc_trans_layer_mmap_session);

int32_t ipc_trans_layer_get_endmap(const uint8_t endid, sts_endmap_t *map)
{
	u8 cpuid = ENDID_TO_CPUID(endid);
	*map = g_ipc_end_array[cpuid]->g_end_sts_map;
	return 0;
}
EXPORT_SYMBOL(ipc_trans_layer_get_endmap);

int32_t ipc_trans_layer_get_debug_info(const uint8_t endid, const uint8_t handle, msgbox_debug_info_t *info)
{
	u8 cpuid = ENDID_TO_CPUID(endid);
	return ipc_trans_get_debug_info(handle, (debug_info_t *)info, g_ipc_end_array[cpuid]);
}
EXPORT_SYMBOL(ipc_trans_layer_get_debug_info);
