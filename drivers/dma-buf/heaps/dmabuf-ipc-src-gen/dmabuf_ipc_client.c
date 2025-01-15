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

/* This file is auto generated for message box v1.2.0.
 * All manual modifications will be LOST by next generation.
 * It is recommended NOT modify it.
 * Generator Version: francaidl cb46a82 msgbx_ipc f2e1e48
 */

#include "dmabuf_ipc_client.h"

#ifdef CONFIG_BST_C1200_ADAS
#define PID CPU_4
#elif defined(CONFIG_BST_C1200_IVI)
#define PID CPU_0
#else
#define PID CPUMP2_0
#endif

#define FID DEF
#define SID 15U

static dmabuf_ipc_client_data_t *s_ins;

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
			if (des->header.pid == s_ins->r5mem_ext.cid)
				ret = s_ins->client.r5mem_client.dispatch_broadcast(des);
			else
				IPC_LOG_ERR("Unexpected broadcast message from ID %d.\n", des->header.pid);
		}
		if (ipc_trans_layer_proxy_get_reply_msg(data->pid, data->handle, des) >= 0) {
			has_message = true;
			if (des->header.pid == s_ins->r5mem_ext.cid)
				ret = s_ins->client.r5mem_client.dispatch_reply(des);
			else
				IPC_LOG_ERR("Unexpected reply message from ID %d.\n", des->header.pid);
		}
		if (!has_message)
			break;
	}
	return ret;
}
#ifndef IPC_RTE_BAREMETAL
#if defined IPC_RTE_KERNEL
static int router_func(void *arg)
#else
static void *router_func(void *arg)
#endif
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
#if defined IPC_RTE_KERNEL
	return RESULT_SUCCESS;
#else
	return arg;
#endif
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
	ret = pthread_create(&data->route_task, NULL, router_func, NULL);
	if (ret != 0) {
#elif defined IPC_RTE_KERNEL
	data->route_task = kthread_run(router_func, NULL, "dmabuf_ipc_client_thread");
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
	sleep(1);
	data->bRunning = false;
	ipc_trans_layer_release_recv_wait(data->pid, data->handle);
	ret = pthread_join(data->route_task, NULL);
	if (ret != 0)
		return -ERR_APP_STOP;
#elif defined IPC_RTE_KERNEL
	msleep(1000);
	if (likely(data->route_task)) {
		ipc_trans_layer_release_recv_wait(data->pid, data->handle);
		ret = kthread_stop(data->route_task);
		if (unlikely(ret))
			return -ERR_APP_STOP;
	}
	data->bRunning = false;
#endif

	return RESULT_SUCCESS;
}
#endif
// dmabuf_ipc_client_init
dmabuf_ipc_client_t *dmabuf_ipc_client_init(dmabuf_ipc_client_data_t *ins)
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
	ret = r5mem_client_init(data, &ins->client.r5mem_client, &ins->r5mem_ext);
	if (ret < 0)
		return NULL;

	// create client handle.
	ret = ipc_trans_layer_proxy_create_handle(data->pid, data->fid, data->sid, ins->r5mem_ext.cid, &data->handle);
	if (ret < 0)
	{
		IPC_LOG_ERR("create handle fail %d.\n", ret);
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
	init_registry_list(data->method_registry, IPC_TOKEN_NUM);
	data->initialized = true;
	return &ins->client;
}
EXPORT_SYMBOL(dmabuf_ipc_client_init);

// dmabuf_ipc_client_destroy
int32_t dmabuf_ipc_client_destroy(void)
{
	int32_t ret = 0;
	com_client_data_t *data = (com_client_data_t *)s_ins;

	// if is NULL, just return SUCCESS.
	if (!data)
		return RESULT_SUCCESS;

	ret = ipc_trans_layer_destroy_handle(data->pid, data->handle);
	if (ret < 0)
		return ret;

	r5mem_client_destroy();

	IPC_MUTEX_DESTROY(&data->send_mtx);
	destroy_registry_list(data->method_registry, IPC_TOKEN_NUM);
	ipc_memset(s_ins, 0, sizeof(dmabuf_ipc_client_data_t));
	s_ins = NULL;
	return ret;
}
EXPORT_SYMBOL(dmabuf_ipc_client_destroy);
