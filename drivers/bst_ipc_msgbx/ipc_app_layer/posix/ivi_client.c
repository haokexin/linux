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

/* This file is auto generated for message box v2.0.0.
 * All manual modifications will be LOST by next generation.
 * It is recommended NOT modify it.
 */

#include "ivi_client.h"

#define PID CPU_0
#define FID F1
#define SID 10U

static ivi_client_data_t *s_ins;

// receive messages
static int32_t receive_message(void)
{
	int32_t ret = 0;
	sts_endmap_t map = {0};
	uint64_t chip_map = 0;
	bool status = false;
	com_client_data_t *data = (com_client_data_t *)s_ins;
	ivi_switch_client_ext_t *ivi_switch_ext = &s_ins->ivi_switch_ext;

	if (!data)
		return -ERR_APP_PARAM;

	ret = ipc_trans_layer_query_info(data->pid, data->handle, 0, 0);

	// check if availability changed
	if (ret == QUERY_INFO_DST_STS_OFFLINE) {
		if (ivi_switch_ext->avail_changed_cb)
			ivi_switch_ext->avail_changed_cb(false, ivi_switch_ext->avail_ext);
	} else if (ret == QUERY_INFO_DST_STS_ONLINE) {
		if (ivi_switch_ext->avail_changed_cb)
			ivi_switch_ext->avail_changed_cb(true, ivi_switch_ext->avail_ext);
	} else if (ret == QUERY_INFO_DST_STS_CHANGED) {
		ret = ipc_trans_layer_get_endmap(data->pid, &map);
		if (ret == 0) {
			chip_map = ivi_switch_ext->ccid != 0 ? map.hi_map : map.lo_map;
			status = (chip_map & ivi_switch_ext->cid_mask) != 0 ? true : false;
			if (status != ivi_switch_ext->status) {
				ivi_switch_ext->status = status;
				if (ivi_switch_ext->avail_changed_cb)
					ivi_switch_ext->avail_changed_cb(status, ivi_switch_ext->avail_ext);
			}

		}
	}

	return ret;
}

// dispatch messages
static int32_t dispatch_message(void)
{
	int32_t ret = 0;
	des_buf_t *des = NULL;
	com_client_data_t *data = (com_client_data_t *)s_ins;
	packet_object_t *packet_obj = NULL;
	lflist_node_t *msg_node = NULL;
	msg_object_t *msg_obj = NULL;
	int8_t* des_data = NULL;
	uintptr_t k_base = 0;
	uintptr_t u_base = 0;

	if (!data || !data->queue)
		return -ERR_APP_PARAM;
	des = &data->des_buf;
	k_base = data->queue->k_addr;
	u_base = (uintptr_t)data->queue;

	packet_obj = KERNEL_2_USER(data->queue->completed_pack_list.tail, k_base, u_base);
	while (packet_obj) {
		msg_node = KERNEL_2_USER(packet_obj->packed_list.tail, k_base, u_base);
		des_data = des->data_buf;

		clear_des_buf(des);
		while (msg_node)
		{
			msg_obj = KERNEL_2_USER(msg_node->data, k_base, u_base);
			ipc_memcpy(des_data, msg_obj->msg.payload, IPC_PAYLOAD_SIZE);
			des_data += IPC_PAYLOAD_SIZE;
			msg_node = KERNEL_2_USER(msg_node->next, k_base, u_base);
		}
		des->header = msg_obj->msg.header;
		des->timestamp = msg_obj->timestamp;
		des->unavail_data_size = IPC_MAX_DATA_SIZE - (msg_obj->msg.header.idx
			* IPC_PAYLOAD_SIZE + msg_obj->msg.header.len * 8);

		// dispatch message
		ret = -1;
		if (des->header.pid == s_ins->ivi_switch_ext.cid) {
			if (des->header.typ == MSGBX_MSG_TYPE_BROADCAST)
				ret = s_ins->client.ivi_switch_client.dispatch_broadcast(des);
			else if (des->header.typ == MSGBX_MSG_TYPE_REPLY)
				ret = s_ins->client.ivi_switch_client.dispatch_reply(des);
		}
		if (ret < 0)
			IPC_LOG_ERR("Unexpected message from ID %u.\n", des->header.pid);

		packet_obj = KERNEL_2_USER(packet_obj->node.next, k_base, u_base);
	}

	return ret;
}
#ifndef IPC_RTE_BAREMETAL
// router function
#if defined IPC_RTE_KERNEL
static int router_func(void *arg)
#elif defined IPC_RTE_POSIX
static void *router_func(void *arg)
#elif defined IPC_RTE_RTOS
static void router_func(void *arg)
#else
#error "unknown rte"
#endif
{
	int32_t ret = 0;

#if defined IPC_RTE_POSIX || defined IPC_RTE_RTOS
	while (s_ins && s_ins->com_data.bRunning) {
#elif defined IPC_RTE_KERNEL
	while (unlikely(!kthread_should_stop())) {
#endif
		ret = receive_message();
		if (ret != 0)
			continue;

		ret = dispatch_message();
		if (ret < 0)
			continue;
	}
#if defined IPC_RTE_KERNEL
	return RESULT_SUCCESS;
#elif defined IPC_RTE_POSIX
	return arg;
#elif defined IPC_RTE_RTOS
	return;
#else
#error "unknown rte"
#endif
}

// start message router
static int32_t start(void)
{
#if defined IPC_RTE_POSIX
	int32_t ret = 0;
#elif defined IPC_RTE_RTOS
	static static_tcb_t tcb_buffer;
	static stack_type_t stack_buffer[0];
#endif
	com_client_data_t *data = (com_client_data_t *)s_ins;

	if (!data)
		return ERR_APP_PARAM;

	if (data->bRunning)
		return RESULT_SUCCESS;

	data->bRunning = true;
#if defined IPC_RTE_POSIX
	ret = pthread_create(&data->route_task, NULL, router_func, NULL);
	if (ret != 0) {
#elif defined IPC_RTE_RTOS
	data->route_task = TaskCreate((task_func_t)router_func, "ivi_client_thread", 0, NULL,
		0, (uint8_t *)stack_buffer, &tcb_buffer, true);
	if (data->route_task == 0) {
#elif defined IPC_RTE_KERNEL
	data->route_task = kthread_run(router_func, NULL, "ivi_client_thread");
	if (unlikely(!data->route_task)) {
#else
#error "unknown rte"
#endif
		data->bRunning = false;
		return -ERR_APP_START;
	}

	return RESULT_SUCCESS;
}

// stop message router.
static int32_t stop(void)
{
	int32_t ret = 0;
	com_client_data_t *data = (com_client_data_t *)s_ins;


	if (!data)
		return ERR_APP_PARAM;

	if (!data->bRunning)
		return RESULT_SUCCESS;

#if defined IPC_RTE_POSIX
	sleep(1);
	data->bRunning = false;
	ipc_trans_layer_release_recv_wait(data->pid, data->handle);
	ret = pthread_join(data->route_task, NULL);
	if (ret != 0)
		return -ERR_APP_STOP;
#elif defined IPC_RTE_RTOS
	Msleep(1000);
	data->bRunning = false;
	ipc_trans_layer_release_recv_wait(data->pid, data->handle);
	TaskDelete(data->route_task);
#elif defined IPC_RTE_KERNEL
	msleep(1000);
	if (likely(data->route_task)) {
		ipc_trans_layer_release_recv_wait(data->pid, data->handle);
		ret = kthread_stop(data->route_task);
		if (unlikely(ret))
			return -ERR_APP_STOP;
	}
	data->bRunning = false;
#else
#error "unknown rte"
#endif

	return RESULT_SUCCESS;
}

#endif
// ivi_client_init
ivi_client_t *ivi_client_init(ivi_client_data_t *ins)
{
	int32_t ret = 0;
	com_client_data_t *data = (com_client_data_t *)ins;

	if (!data)
		return NULL;
	if (s_ins && s_ins->com_data.initialized) {
		IPC_LOG_ERR("already initialized.\n");
		return &s_ins->client;
	}

	s_ins = ins;

	data->pid = data->pid == 0 ? PID : data->pid;
	data->fid = data->fid == 0 ? FID : data->fid;
	data->sid = data->sid == 0 ? SID : data->sid;

	// init clients
	ret = ivi_switch_client_init(data, &ins->client.ivi_switch_client, &ins->ivi_switch_ext);
	if (ret < 0)
		return NULL;

	// create client handle.
	ret = ipc_trans_layer_proxy_create_handle(data->pid, data->fid, data->sid, ins->ivi_switch_ext.cid, ins->ivi_switch_ext.ccid, &data->handle);
	if (ret < 0)
	{
		IPC_LOG_ERR("create handle fail %" PRId32 ".\n", ret);
		return NULL;
	}

	ret = ipc_trans_layer_mmap_session(data->pid, data->handle, (void**)&ins->com_data.queue);
	if (ret < 0)
	{
		IPC_LOG_ERR("mmap session addr fail %" PRId32 ".\n", ret);
		return NULL;
	}

#ifdef IPC_RTE_BAREMETAL
	ins->client.receive_message = receive_message;
	ins->client.dispatch_message = dispatch_message;
#else
	ins->client.start = start;
	ins->client.stop = stop;
#endif
	IPC_MUTEX_INIT(&data->send_mtx);
	init_registry_list(data->common_registry, IPC_TOKEN_NUM);
	data->initialized = true;
	return &ins->client;
}

// ivi_client_destroy
int32_t ivi_client_destroy(void)
{
	int32_t ret = 0;
	com_client_data_t *data = (com_client_data_t *)s_ins;

	// if is NULL, just return SUCCESS.
	if (!data)
		return RESULT_SUCCESS;

	ret = ipc_trans_layer_destroy_handle(data->pid, data->handle);
	if (ret < 0)
		return ret;

	ivi_switch_client_destroy();

	IPC_MUTEX_DESTROY(&data->send_mtx);
	destroy_registry_list(data->common_registry, IPC_TOKEN_NUM);
	ipc_memset(s_ins, 0, sizeof(ivi_client_data_t));
	s_ins = NULL;
	return ret;
}
