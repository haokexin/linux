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

/* This file is auto generated for message box v1.1.0.
 * All manual modifications will be LOST by next generation.
 * It is recommended NOT modify it.
 * Generator Version: francaidl 8957426 msgbx_ipc 1964fef
 */

#include "DisplayClient.h"

#define PID CPU_4
#define FID DEF
#define SID 2U

static DisplayClient_data_t *s_ins;

// receive messages
static int32_t receive_message(void)
{
	int32_t ret = 0;
	com_client_data_t *data = (com_client_data_t *)s_ins;

	if (!data)
		return -ERR_APP_PARAM;

	ret = ipc_trans_layer_query_info(data->pid, data->handle);

	// check if availability changed
	if (data->avail_changed_cb) {
		if (ret == QUERY_INFO_DST_STS_OFFLINE)
			data->avail_changed_cb(false, data->avail_ext);
		else if (ret == QUERY_INFO_DST_STS_ONLINE)
			data->avail_changed_cb(true, data->avail_ext);
	}

	return ret;
}

// dispatch messages
static int32_t dispatch_message(void)
{
	int32_t ret = 0;
	serdes_t *des = NULL;
	bool has_message = false;
	com_client_data_t *data = (com_client_data_t *)s_ins;

	if (!data)
		return -ERR_APP_PARAM;
	des = &data->deserializer;

	while (true) {
		has_message = false;
		if (ipc_trans_layer_proxy_get_broadcast_msg(data->pid, data->handle, des) >= 0) {
			has_message = true;
			if (des->header.pid == s_ins->display_ext.cid)
				ret = s_ins->client.display_client.dispatch_broadcast(des);
			else
				IPC_LOG_ERR("Unexpected broadcast message from ID %d.\n", des->header.pid);
		}
		if (ipc_trans_layer_proxy_get_reply_msg(data->pid, data->handle, des) >= 0) {
			has_message = true;
			if (des->header.pid == s_ins->display_ext.cid)
				ret = s_ins->client.display_client.dispatch_reply(des);
			else
				IPC_LOG_ERR("Unexpected reply message from ID %d.\n", des->header.pid);
		}
		if (!has_message)
			break;
	}
	return ret;
}
#ifndef IPC_RTE_BAREMETAL
static int router_func(void *arg)
{
	int32_t ret = 0;

#if defined IPC_RTE_POSIX
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

	return RESULT_SUCCESS;
}

// start message router
static int32_t start(void)
{
#if defined IPC_RTE_POSIX
	int32_t ret = 0;
#endif
	com_client_data_t *data = &s_ins->com_data;

	if (!data)
		return ERR_APP_PARAM;

	if (data->bRunning)
		return RESULT_SUCCESS;

	data->bRunning = true;
#if defined IPC_RTE_POSIX
	ret = thrd_create(&data->router_tid, router_func, NULL);
	if (ret != thrd_success) {
#elif defined IPC_RTE_KERNEL
	data->route_task = kthread_run(router_func, NULL, "DisplayClient_thread");
	if (unlikely(!data->route_task)) {
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
	com_client_data_t *data = &s_ins->com_data;

	if (!data)
		return ERR_APP_PARAM;

	if (!data->bRunning)
		return RESULT_SUCCESS;

	//sleep 1 seconds.
#if defined IPC_RTE_POSIX
	thrd_sleep(&(struct timespec){.tv_sec = 1}, NULL);
	data->bRunning = false;
	ipc_trans_layer_release_recv_wait(PID, data->handle);
	ret = thrd_join(data->router_tid, NULL);
	if (ret != thrd_success)
		return -ERR_APP_STOP;
#elif defined IPC_RTE_KERNEL
	msleep(1000);
	if (likely(data->route_task)) {
		ipc_trans_layer_release_recv_wait(PID, data->handle);
		ret = kthread_stop(data->route_task);
		if (unlikely(ret))
			return -ERR_APP_STOP;
	}
	data->bRunning = false;
#endif

	return RESULT_SUCCESS;
}
#endif
// DisplayClient_init
DisplayClient_t *DisplayClient_init(DisplayClient_data_t *ins)
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
	ret = display_client_init(data, &ins->client.display_client, &ins->display_ext);
	if (ret < 0)
		return NULL;

	// create client handle.
	ret = ipc_trans_layer_proxy_create_handle(data->pid, data->fid, data->sid, ins->display_ext.cid, &data->handle);
	if (ret < 0)
		return NULL;

#ifdef IPC_RTE_BAREMETAL
	ins->client.receive_message = receive_message;
	ins->client.dispatch_message = dispatch_message;
#else
	ins->client.start = start;
	ins->client.stop = stop;
#if defined IPC_RTE_POSIX
	(void)mtx_init(&data->send_mtx, mtx_plain);
#elif defined IPC_RTE_KERNEL
	mutex_init(&data->send_mtx);
#endif
#endif
	init_registry_list(data->method_registry, IPC_TOKEN_NUM);
	data->initialized = true;
	return &ins->client;
}

// DisplayClient_destroy
int32_t DisplayClient_destroy(void)
{
	int32_t ret = 0;
	com_client_data_t *data = (com_client_data_t *)s_ins;

	// if is NULL, just return SUCCESS.
	if (!data)
		return RESULT_SUCCESS;

	ret = ipc_trans_layer_destroy_handle(data->pid, data->handle);
	if (ret < 0)
		return ret;

	display_client_destroy();

#ifndef IPC_RTE_BAREMETAL
#if defined IPC_RTE_POSIX
	mtx_destroy(&data->send_mtx);
#elif defined IPC_RTE_KERNEL
	mutex_destroy(&data->send_mtx);
#endif
#endif
	destroy_registry_list(data->method_registry, IPC_TOKEN_NUM);
	ipc_memset(s_ins, 0, sizeof(DisplayClient_data_t));
	s_ins = NULL;
	return ret;
}
