// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include "video_server.h"
#include "ipc_trans_common.h"
#include "ipc_trans_layer.h"

// macro definitions
#define MAJOR 1U
#define MINOR 0U

#define MAX_METHOD_NUM	  10U
#define MAX_BROADCAST_NUM 10U

#define CMD_METHOD_ISP2ARM 1U

#define CMD_METHOD_SUB_ARM2ISP	 10U
#define CMD_METHOD_UNSUB_ARM2ISP 11U
#define CMD_BROADCAST_ARM2ISP	 1U

// local variables
static com_server_data_t *s_data;
static video_server_ext_t *s_ext;

// interface implementation
// get interface version
static ipc_inf_version_t get_ipc_inf_version(void)
{
	ipc_inf_version_t ret = { .major = MAJOR, .minor = MINOR };

	return ret;
}

// method
static int32_t register_isp2arm(video_isp2arm_t func)
{
	if (!s_ext)
		return -ERR_APP_PARAM;
	s_ext->isp2arm_ptr = func;
	return RESULT_SUCCESS;
}

static int32_t call_isp2arm(serdes_t *des)
{
	int32_t ret = 0;
	des_buf_t *buf = NULL;
	uint32_t len = 0;
	com_server_data_t *data = s_data;
	uint32_t msgAddr = 0;

	if (!des || !data || !s_ext || !s_ext->isp2arm_ptr)
		return -ERR_APP_PARAM;

	buf = &data->des_buf;
	clear_des_buf(buf);
	len = ipc_des_get_all(des, (uint8_t *)buf->data_buf);
	if (len <= 0)
		return -ERR_APP_SERDES;
	buf->unavail_data_size = IPC_MAX_DATA_SIZE - len;

	if (ret >= 0)
		ret = deserialize_32(buf, (uint32_t *)&msgAddr);

	if (ret < 0)
		return -ERR_APP_SERDES;

	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->recv_end_time;

	(*s_ext->isp2arm_ptr)(msgAddr, *(uint64_t *)&des->header, &data->info);

	return RESULT_SUCCESS;
}

static int32_t reply_isp2arm(const uint32_t result, const video_error_e_t err,
			     const uint64_t context)
{
	int32_t ret = 0;
#ifdef IPC_RTE_BAREMETAL
	serdes_t *ser = NULL;
#else
	serdes_t serdes = { 0 };
	serdes_t *ser = &serdes;
#endif
	com_server_data_t *data = s_data;

	if (!data)
		return -ERR_APP_PARAM;
#ifdef IPC_RTE_BAREMETAL
	ser = &data->serializer;
#endif
	ret = ipc_ser_init(ser);
	if (ret >= 0)
		ret = serialize_video_error_e(ser, &err);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&result);

	if (ret < 0) {
		int32_t _err = -1;
		(void)ipc_ser_init(ser);
		ret = ipc_ser_put_32(ser, (uint32_t *)&_err);
		IPC_LOG_ERR("serialization failed.\n");
	}

	if (ret >= 0) {
		ser->header = *(rw_msg_header_t *)&context;
		ser->header.cid = ser->header.pid;
		ser->header.pid = data->pid;
		ser->header.typ = MSGBX_MSG_TYPE_REPLY;
		ret = ipc_ser_finish(ser);
	}

	if (ret >= 0)
		ret = ipc_trans_layer_stub_send_reply_msg(data->pid,
							  data->handle, ser);

	if (ret < 0) {
		IPC_LOG_ERR("send reply fail %d.\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

// broadcast

static int32_t register_arm2isp_subcribed(broadcast_sub_t func)
{
	if (!s_ext)
		return -ERR_APP_PARAM;
	s_ext->arm2isp_sub_ptr = func;
	return RESULT_SUCCESS;
}

static int32_t register_arm2isp_unsubcribed(broadcast_sub_t func)
{
	if (!s_ext)
		return -ERR_APP_PARAM;
	s_ext->arm2isp_unsub_ptr = func;
	return RESULT_SUCCESS;
}

static int32_t arm2isp(uint32_t msgAddr)
{
	int32_t ret = 0;
	int32_t send_ret = 0;
	int32_t index = 0;
	rw_msg_header_t header = { 0 };
	broadcast_reg_entry_t *entry = NULL;
	broadcast_registry_t *reg = NULL;
#ifdef IPC_RTE_BAREMETAL
	serdes_t *ser = NULL;
#else
	serdes_t serdes = { 0 };
	serdes_t *ser = &serdes;
#endif
	com_server_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
#ifdef IPC_RTE_BAREMETAL
	ser = &data->serializer;
#endif
	header.pid = data->pid;
	header.cmd = CMD_BROADCAST_ARM2ISP;
	header.typ = MSGBX_MSG_TYPE_BROADCAST;
	ret = ipc_ser_init(ser);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&msgAddr);

	if (ret < 0)
		return -ERR_APP_SERDES;

	reg = &s_ext->arm2isp_registry;
	entry = reg->entries + reg->start;
	for (index = reg->start; index < reg->end; ++index, ++entry) {
		if (entry->pid != 0) {
			header.cid = entry->pid;
			header.fid = entry->fid;
			header.sid = entry->sid;
			header.tok = data->token;
			ipc_ser_set_header(ser, header);
			ipc_ser_finish(ser);
			send_ret = ipc_trans_layer_stub_send_broadcast(
				data->pid, data->handle, ser);
			if (send_ret < 0)
				IPC_LOG_ERR("send broadcast fail %d.\n",
					    send_ret);
			else
				++ret;
		}
	}
	increase_token(data);
	return ret;
}

// dispatch_request
static int32_t dispatch_request(serdes_t *des, bool *reply)
{
	int32_t ret = 0;

	if (!des)
		return -ERR_APP_PARAM;

	switch (des->header.cmd) {
	case CMD_METHOD_ISP2ARM:
		ret = call_isp2arm(des);
		if (ret >= 0)
			*reply = false;
		else
			*reply = true;
		return ret;
	case CMD_METHOD_SUB_ARM2ISP:
		if (s_ext->arm2isp_sub_ptr)
			ret = (*s_ext->arm2isp_sub_ptr)(
				(uint8_t)des->header.pid,
				(uint8_t)des->header.fid,
				(uint8_t)des->header.sid);
		if (ret >= 0)
			ret = add_registration(&s_ext->arm2isp_registry,
					       (uint8_t)des->header.pid,
					       (uint8_t)des->header.fid,
					       (uint8_t)des->header.sid);
		*reply = true;
		return ret;
	case CMD_METHOD_UNSUB_ARM2ISP:
		if (s_ext->arm2isp_unsub_ptr)
			ret = (*s_ext->arm2isp_unsub_ptr)(
				(uint8_t)des->header.pid,
				(uint8_t)des->header.fid,
				(uint8_t)des->header.sid);
		if (ret >= 0)
			ret = remove_registration(&s_ext->arm2isp_registry,
						  (uint8_t)des->header.pid,
						  (uint8_t)des->header.fid,
						  (uint8_t)des->header.sid);
		*reply = true;
		return ret;
	default:
		break;
	}
	ret = -ERR_APP_UNKNOWN_CMD;
	*reply = false;
	return ret;
}

// initialize server
int32_t video_server_init(com_server_data_t *data, video_server_t *server,
			  video_server_ext_t *ext)
{
	int32_t ret = 0;

	if (!data || !server || !ext)
		return -1;

	s_data = data;
	s_ext = ext;

	// register CMDs.
	ret = ipc_trans_layer_register_method(data->pid, data->handle,
					      CMD_METHOD_ISP2ARM);
	if (ret < 0) {
		IPC_LOG_ERR("Failed to register CMD_METHOD_ISP2ARM\n");
		return -1;
	}

	ret = ipc_trans_layer_register_method(data->pid, data->handle,
					      CMD_METHOD_SUB_ARM2ISP);
	if (ret < 0) {
		IPC_LOG_ERR("Failed to register CMD_METHOD_SUB_ARM2ISP\n");
		return -1;
	}
	ret = ipc_trans_layer_register_method(data->pid, data->handle,
					      CMD_METHOD_UNSUB_ARM2ISP);
	if (ret < 0) {
		IPC_LOG_ERR("Failed to register CMD_METHOD_UNSUB_ARM2ISP\n");
		return -1;
	}

	// set server
	server->version = get_ipc_inf_version;
	ext->isp2arm_ptr = NULL;
	server->register_isp2arm = register_isp2arm;
	server->reply_isp2arm = reply_isp2arm;

	server->arm2isp = arm2isp;
	server->register_arm2isp_subcribed = register_arm2isp_subcribed;
	server->register_arm2isp_unsubcribed = register_arm2isp_unsubcribed;

	server->dispatch_request = dispatch_request;

	return 0;
}

// destroy client
void video_server_destroy(void)
{
	s_data = NULL;
	s_ext = NULL;
}
