// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include "isp_msgbx_server.h"
#include "ipc_trans_common.h"
#include "ipc_trans_layer.h"

#define PID CPU_4
#define FID DEF
#define SID 1U

static isp_msgbx_server_data_t *s_ins;

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

	while (ipc_trans_layer_stub_get_method_msg(data->pid, data->handle,
						   des) >= 0) {
		bool need_reply = true;
		// initialize serializer.
		// it cannot fail, as ser won't be NULL.
		(void)ipc_ser_init(ser);
		// process message.
		ret = s_ins->server.video_server.dispatch_request(des,
								  &need_reply);
		if (ret < 0)
			IPC_LOG_ERR(
				"video_server dispatch request failed %d.\n",
				ret);

		if (need_reply) {
			(void)ipc_ser_init(ser);
			ret = ipc_ser_put_32(ser, (uint32_t *)&ret);
			ser->header = des->header;
			ser->header.cid = des->header.pid;
			ser->header.pid = data->pid;
			ser->header.typ = MSGBX_MSG_TYPE_REPLY;
			if (ret >= 0)
				ret = ipc_ser_finish(ser);
			if (ret >= 0)
				ret = ipc_trans_layer_stub_send_reply_msg(
					data->pid, data->handle, ser);
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
	data->route_task =
		kthread_run(router_func, NULL, "isp_msgbx_server_thread");
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

	// sleep 1 seconds.
#if defined IPC_RTE_POSIX
	thrd_sleep(&(struct timespec) { .tv_sec = 1 }, NULL);
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

// isp_msgbx_server_init
isp_msgbx_server_t *isp_msgbx_server_init(isp_msgbx_server_data_t *ins)
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
	ret = ipc_trans_layer_stub_create_handle(
		data->pid, data->fid, data->sid, data->pid, &data->handle);
	if (ret < 0)
		return NULL;

	// init servers
	ret = video_server_init(data, &ins->server.video_server,
				&ins->video_ext);
	if (ret < 0) {
		(void)ipc_trans_layer_unregister_method(data->pid,
							data->handle);
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

// isp_msgbx_server_destroy
int32_t isp_msgbx_server_destroy(void)
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

	video_server_destroy();

	ipc_memset(s_ins, 0, sizeof(isp_msgbx_server_data_t));
	s_ins = NULL;
	return ret;
}
