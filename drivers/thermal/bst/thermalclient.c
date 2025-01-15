// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
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

/* This file is auto generated for message box v1.0.0.
 * All manual modifications will be LOST by next generation.
 * It is recommended NOT modify it.
 * Generator Version: francaidl 3a7f767 msgbx_ipc 001bddd
 */

#include "thermalclient.h"

#define PID CPU_5
#define FID F1
#define SID 4U

static thermalclient_data_t *s_ins;

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
			if (des->header.pid == s_ins->st_public1_ext.cid)
				ret = s_ins->client.st_public1_client.dispatch_broadcast(des);
			else
				IPC_LOG_ERR("Unexpected broadcast message from ID %d.\n", des->header.pid);
		}
		if (ipc_trans_layer_proxy_get_reply_msg(data->pid, data->handle, des) >= 0) {
			has_message = true;
			if (des->header.pid == s_ins->st_public1_ext.cid)
				ret = s_ins->client.st_public1_client.dispatch_reply(des);
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
	int32_t ret = 0;
	com_client_data_t *data = &s_ins->com_data;

	if (!data)
		return ERR_APP_PARAM;

	if (data->bRunning)
		return RESULT_SUCCESS;

	ret = 0;
	data->bRunning = true;
#if defined IPC_RTE_POSIX
	ret = thrd_create(&data->router_tid, router_func, NULL);
	if (ret != thrd_success) {
#elif defined IPC_RTE_KERNEL
	data->route_task = kthread_run(router_func, NULL, "thermalclient_thread");
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
// thermalclient_init
thermalclient_t *thermalclient_init(thermalclient_data_t *ins)
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
	ret = st_public1_client_init(data, &ins->client.st_public1_client, &ins->st_public1_ext);
	if (ret < 0)
		return NULL;

	// create client handle.
	ret = ipc_trans_layer_proxy_create_handle(data->pid, data->fid, data->sid, ins->st_public1_ext.cid, &data->handle);
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

// thermalclient_destroy
int32_t thermalclient_destroy(void)
{
	int32_t ret = 0;
	com_client_data_t *data = (com_client_data_t *)s_ins;

	// if is NULL, just return SUCCESS.
	if (!data)
		return RESULT_SUCCESS;

	ret = ipc_trans_layer_destroy_handle(data->pid, data->handle);
	if (ret < 0)
		return ret;

	st_public1_client_destroy();

#ifndef IPC_RTE_BAREMETAL
#if defined IPC_RTE_POSIX
	mtx_destroy(&data->send_mtx);
#elif defined IPC_RTE_KERNEL
	mutex_destroy(&data->send_mtx);
#endif
#endif
	destroy_registry_list(data->method_registry, IPC_TOKEN_NUM);
	ipc_memset(s_ins, 0, sizeof(thermalclient_data_t));
	s_ins = NULL;
	return ret;
}
