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
 * Generator Version: francaidl 77a2400 msgbx_ipc eb42a92
 */

#include "sample_server.h"
#include <bst/ipc_trans_common.h>
#include <bst/ipc_trans_layer.h>

#define PID CPU_7
#define FID DEF
#define SID 0U

static sample_server_data_t *s_ins;

// receive messages
static int32_t receive_message(void)
{
	com_server_data_t *data = (com_server_data_t *)s_ins;

	if (!data)
		return -ERR_APP_PARAM;

	return ipc_trans_layer_query_info(data->pid, data->handle);
}

// dispatch messages
static int32_t dispatch_message(void)
{
	int32_t ret = 0;
	serdes_t *ser = NULL;
	serdes_t *des = NULL;
	com_server_data_t *data = (com_server_data_t *)s_ins;

	if (!data)
		return -ERR_APP_PARAM;
	ser = &data->serializer;
	des = &data->deserializer;

	while (ipc_trans_layer_stub_get_method_msg(data->pid, data->handle, des) >= 0) {
		bool need_reply = true;
		// initialize serializer.
		// it cannot fail, as ser won't be NULL.
		(void)ipc_ser_init(ser);
		// process message.
	   ret = s_ins->server.test_server.dispatch_request(des, &need_reply);
		if (ret < 0)
			IPC_LOG_ERR("test_server dispatch request failed %d.\n", ret);

		if (need_reply) {
			(void)ipc_ser_init(ser);
			ret = ipc_ser_put_32(ser, (uint32_t *)&ret);
			ser->header = des->header;
			ser->header.cid = des->header.pid;
			ser->header.pid = data->pid;
			ser->header.typ = IPC_MSG_TYPE_REPLY;
			if (ret >= 0)
				ret = ipc_ser_finish(ser);
			if (ret >= 0)
				ret = ipc_trans_layer_stub_send_reply_msg(data->pid, data->handle, ser);
			if (ret < 0)
				IPC_LOG_ERR("send reply fail %d.\n", ret);
		}
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
	com_server_data_t *data = &s_ins->com_data;

	if (!data)
		return ERR_APP_PARAM;

	if (data->bRunning)
		return RESULT_SUCCESS;

	data->bRunning = true;
#if defined IPC_RTE_POSIX
	ret = thrd_create(&data->router_tid, router_func, NULL);
	if (ret != thrd_success) {
#elif defined IPC_RTE_KERNEL
	data->route_task = kthread_run(router_func, NULL, "sample_server_thread");
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
	com_server_data_t *data = &s_ins->com_data;

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
// sample_server_init
sample_server_t *sample_server_init(sample_server_data_t *ins)
{
	int32_t ret = 0;
	com_server_data_t *data = (com_server_data_t *)ins;

	if (!data)
		return NULL;
	if (s_ins && s_ins->com_data.initialized) {
		IPC_LOG_ERR("already initialized.\n");
		return &s_ins->server;
	}

	s_ins = ins;

	data->pid = data->pid == 0 ? PID : data->pid;
	data->fid = data->fid == 0 ? FID : data->fid;
	data->sid = data->sid == 0 ? SID : data->sid;

	// create server handle.
	ret = ipc_trans_layer_stub_create_handle(data->pid, data->fid, data->sid,
				data->pid, &data->handle);
	if (ret < 0)
		return NULL;

	// init servers
	ret = test_server_init(data, &ins->server.test_server, &ins->test_ext);
	if (ret < 0) {
		(void)ipc_trans_layer_unregister_method(data->pid, data->handle);
		(void)ipc_trans_layer_destroy_handle(data->pid, data->handle);
		return NULL;
	}

#ifdef IPC_RTE_BAREMETAL
	ins->server.receive_message = receive_message;
	ins->server.dispatch_message = dispatch_message;
#else
	ins->server.start = start;
	ins->server.stop = stop;
#endif
	data->initialized = true;
	return &ins->server;
}

// sample_server_destroy
int32_t sample_server_destroy(void)
{
	int32_t ret = 0;
	com_server_data_t *data = (com_server_data_t *)s_ins;

	// if is NULL, just return SUCCESS.
	if (!data)
		return RESULT_SUCCESS;

	ret = ipc_trans_layer_unregister_method(data->pid, data->handle);
	if (ret < 0)
		return ret;
	ret = ipc_trans_layer_destroy_handle(data->pid, data->handle);
	if (ret < 0)
		return ret;

	test_server_destroy();

	ipc_memset(s_ins, 0, sizeof(sample_server_data_t));
	s_ins = NULL;
	return ret;
}
