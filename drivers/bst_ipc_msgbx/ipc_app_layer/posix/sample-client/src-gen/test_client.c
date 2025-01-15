
/* © Copyright  Black Sesame Technologies (Chengdu) Co., Ltd. 2021. All rights reserved. 
 * This file contains proprietary information that is the sole intellectual property of 
 * Black Sesame Technologies (Chengdu) Co., Ltd. No part of this material or its 
 * documentation may be reproduced, distributed, transmitted, displayed or published in
 * any manner without the written permission of Black Sesame Technologies (Chengdu) Co.,
 * Ltd. Anyone who infringes on the copyright of Black Sesame Technologies (Chengdu) Co.,
 * Ltd. shall be held accountable by Black Sesame Technologies (Chengdu) Co., Ltd..
 */

#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <bst/ipc_serdes.h>
#include <bst/ipc_app_client_utils.h>
#include <bst/ipc_app_common.h>
#include <bst/ipc_trans_common.h>
#include <bst/ipc_trans_layer.h>

#include "test_client.h"

// macro definitions
#define PID CPU_7
#define CID CPU_7
#define FID DEF
#define SID 2U

#define CMD_METHOD_HELLO 1U
#define CMD_METHOD_COMPLEX_METHOD 2U

#define CMD_METHOD_SUB_HEARTBEAT 10U
#define CMD_METHOD_UNSUB_HEARTBEAT 11U
#define CMD_BROADCAST_HEARTBEAT 1U

// local variables
static ipc_inf_version_t s_version = {.major = 1, .minor = 0};
static uint32_t s_handle = 0U;
static test_client s_client = {0};
static volatile _Atomic uint8_t s_token = 0;
static int8_t s_recv_buffer[IPC_MAX_DATA_SIZE] = {0};
static callback_registration_t s_method_registry[IPC_TOKEN_NUM] = {0};
static struct mutex s_send_mtx = {0};
static callback_registration_t s_heartbeat_registry = {0};

// private data structure

typedef struct
{
    char ** message;
	test_ErrorEnum_t* err;
}hello_out_t;

typedef struct
{
    uint32_t* out1;
	char ** out2;
	byte_buffer* out3;
	test_MyArray_t* out4;
	test_MyStruct2_t* out5;
	test_MyUnion_t* out6;
	test_ErrorEnum_t* err;
}complex_method_out_t;

// interface implementation
// get interface version
static ipc_inf_version_t get_ipc_inf_version(void)
{
    return s_version;
}

// method

static void hello_sync_callback(const char * message, const test_ErrorEnum_t err, void *ext)
{
	hello_out_t *out = (hello_out_t*)ext;
	if (!out)
		return;
	if (!out->message)
		return;
	*out->message = kstrdup(message, GFP_KERNEL);
	*out->err = err;
}

static inline int32_t serialize_hello(serdes_t* ser, const char * name)
{
    int32_t ret = 0;
    ser->header.pid = PID;
    ser->header.cid = CID;
    ser->header.fid = FID;
    ser->header.sid = SID;
    ser->header.cmd = CMD_METHOD_HELLO;
    ser->header.typ = MSGBX_MSG_TYPE_METHOD;
    ret = ipc_ser_put_string(ser, name);
	if (ret < 0)
	    return -1;
	
    return 0;
}

static int32_t call_hello_sync(const char * name, char ** message, test_ErrorEnum_t* err, int64_t timeout_ms)
{
    int32_t ret = 0;
    serdes_t serdes = {0};

    // serialize
    (void)ipc_ser_init(&serdes);

    ret = serialize_hello(&serdes, name);
	if (ret != 0)
	{
		printf("Serialize hello fail.\n");
		return -1;
	}

	// take and set registry
	uint8_t tok = 0;
	callback_registration_t * reg = take_registry(s_method_registry, &s_token, &tok);
	if (!reg)
		return -2;
    hello_out_t out = {.message = message, .err = err};
	(void)add_registry(reg, (void *)hello_sync_callback, (void*)&out);
	serdes.header.tok = tok;

    (void)ipc_ser_finish(&serdes);

    // send message
    uint32_t i = 0;
    mutex_lock(&s_send_mtx);
    for (; i <= serdes.index; ++i)
    {
        ret = ipc_trans_layer_proxy_send_method(s_handle, serdes.msg_pool[i]);
        if (ret < 0)
            break;
    }
    mutex_unlock(&s_send_mtx);
    if (ret < 0)
    {
		clear_registry(reg);
        printf("send method hello fail %d.\n", ret);
        return ret;
    }

    //wait for reply
    if (timeout_ms <= 0)
        ret = wait_on_registry(reg);
    else
        ret = timedwait_on_registry(reg, timeout_ms);
    if (ret < 0)
    {
        clear_registry(reg);
        printf("%s wait timeout\n", __func__);
    }

    return ret;
}

static int32_t call_hello_async(const char * name, test_hello_callback_t cb, void *ext)
{
    int32_t ret = 0;
    serdes_t serdes = {0};

    // serialize
    (void)ipc_ser_init(&serdes);

    ret = serialize_hello(&serdes, name);
	if (ret != 0)
	{
		printf("Serialize hello fail.\n");
		return -1;
	}

	// take and set registry
	uint8_t tok = 0;
	callback_registration_t * reg = take_registry(s_method_registry, &s_token, &tok);
	if (!reg)
		return -2;
	(void)add_registry(reg, (void *)cb, ext);
	serdes.header.tok = tok;

    (void)ipc_ser_finish(&serdes);

    // send message
    uint32_t i = 0;
    mutex_lock(&s_send_mtx);
    for (; i <= serdes.index; ++i)
    {
        ret = ipc_trans_layer_proxy_send_method(s_handle, serdes.msg_pool[i]);
        if (ret < 0)
            break;
    }
    mutex_unlock(&s_send_mtx);
    if (ret < 0)
    {
		clear_registry(reg);
        printf("send method hello fail %d.\n", ret);
        return ret;
    }

    return 0;
}

static inline int32_t call_hello_callback(serdes_t *des)
{
    if (!des)
        return -1;
    
    int32_t ret = 0;
    uint32_t length = 0;

    test_ErrorEnum_t err = {0};
	int32_t err_val = 0;
	ret = ipc_des_get(des, (uint8_t *)&err_val, sizeof(int32_t));
	if (ret < 0)
	{
	    return -1;
	}
	else
	{
	    err = err_val;
	    length += sizeof(int32_t);
	    if (length >= IPC_MAX_DATA_SIZE)
	        return -1;
	}
	
	if (err != NO_ERROR)
	{
	    char* message = NULL;
		
	    callback_registration_t *reg = &s_method_registry[des->header.tok];
		if (reg->busy)
		{
	        test_hello_callback_t cb = (test_hello_callback_t)(reg->cb);
	        if (cb)
	            cb(message, err, reg->ext);
	        notify_callback_registry(reg);
	        clear_registry(reg);
	    }
	    else
		{
			printf("callback registry is invalid.\n");
		}
	    return 0;
	}
	
	char* message = NULL;
	message = &s_recv_buffer[length];
	ret = ipc_des_get_string(des, message, IPC_MAX_DATA_SIZE - length);
	if (ret < 0)
	{
	    return -1;
	}
	else
	{
	    length += ret;
	    if (length >= IPC_MAX_DATA_SIZE)
	        return -1;
	}
	
    callback_registration_t *reg = &s_method_registry[des->header.tok];
	if (reg->busy)
	{
        test_hello_callback_t cb = (test_hello_callback_t)(reg->cb);
        if (cb)
            cb(message, err, reg->ext);
        notify_callback_registry(reg);
        clear_registry(reg);
    }
	else
	{
		printf("callback registry is invalid.\n");
	}
    return 0;
}

static void complex_method_sync_callback(const uint32_t out1, const char * out2, const byte_buffer out3, const test_MyArray_t out4, const test_MyStruct2_t out5, const test_MyUnion_t out6, const test_ErrorEnum_t err, void *ext)
{
	complex_method_out_t *out = (complex_method_out_t*)ext;
	if (!out)
		return;
    *out->out1 = out1;
	*out->out2 = kstrdup(out2, GFP_KERNEL);
	out->out3->size = out3.size;
	out->out3->data = kzalloc(out3.size, GFP_KERNEL);
	memcpy(out->out3->data, out3.data, out3.size);
	out->out4->size = out4.size;
	out->out4->data = kzalloc(out4.size * sizeof(uint16_t), GFP_KERNEL);
	memcpy(out->out4->data, out4.data, out4.size * sizeof(uint16_t));
	out->out5->m1 = out5.m1;
	out->out5->m2 = out5.m2;
	out->out5->m3.size = out5.m3.size;
	out->out5->m3.data = kzalloc(out5.m3.size * sizeof(uint16_t), GFP_KERNEL);
	memcpy(out->out5->m3.data, out5.m3.data, out5.m3.size * sizeof(uint16_t));
	out->out5->m4 = kstrdup(out5.m4, GFP_KERNEL);
	out->out5->m5.size = out5.m5.size;
	out->out5->m5.data = kzalloc(out5.m5.size, GFP_KERNEL);
	memcpy(out->out5->m5.data, out5.m5.data, out5.m5.size);
	*out->out6 = out6;
	*out->err = err;
}

static inline int32_t serialize_complex_method(serdes_t* ser, const uint32_t in1, const char * in2, const byte_buffer in3, const test_MyArray_t in4, const test_MyStruct2_t in5, const test_MyUnion_t in6)
{
    int32_t ret = 0;
    ser->header.pid = PID;
    ser->header.cid = CID;
    ser->header.fid = FID;
    ser->header.sid = SID;
    ser->header.cmd = CMD_METHOD_COMPLEX_METHOD;
    ser->header.typ = MSGBX_MSG_TYPE_METHOD;
    ret = ipc_ser_put(ser, (uint8_t*)&in1, sizeof(uint32_t));
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put_string(ser, in2);
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, (uint8_t*)&in3.size, sizeof(uint32_t));
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, in3.data, in3.size);
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, (uint8_t*)&in4.size, sizeof(uint32_t));
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, (uint8_t*)in4.data, in4.size * sizeof(uint16_t));
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, (uint8_t*)&in5.m1, sizeof(uint8_t));
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, (uint8_t*)&in5.m2, sizeof(bool));
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, (uint8_t*)&in5.m3.size, sizeof(uint32_t));
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, (uint8_t*)in5.m3.data, in5.m3.size * sizeof(uint16_t));
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put_string(ser, in5.m4);
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, (uint8_t*)&in5.m5.size, sizeof(uint32_t));
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, in5.m5.data, in5.m5.size);
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, (uint8_t*)&in6, sizeof(test_MyUnion_t));
	if (ret < 0)
	    return -1;
	
    return 0;
}

static int32_t call_complex_method_sync(const uint32_t in1, const char * in2, const byte_buffer in3, const test_MyArray_t in4, const test_MyStruct2_t in5, const test_MyUnion_t in6, uint32_t* out1, char ** out2, byte_buffer* out3, test_MyArray_t* out4, test_MyStruct2_t* out5, test_MyUnion_t* out6, test_ErrorEnum_t* err, int64_t timeout_ms)
{
    int32_t ret = 0;
    serdes_t serdes = {0};

    // serialize
    (void)ipc_ser_init(&serdes);

    ret = serialize_complex_method(&serdes, in1, in2, in3, in4, in5, in6);
	if (ret != 0)
	{
		printf("Serialize complex_method fail.\n");
		return -1;
	}

	// take and set registry
	uint8_t tok = 0;
	callback_registration_t * reg = take_registry(s_method_registry, &s_token, &tok);
	if (!reg)
		return -2;
    complex_method_out_t out = {.out1 = out1, .out2 = out2, .out3 = out3, .out4 = out4, .out5 = out5, .out6 = out6, .err = err};
	(void)add_registry(reg, (void *)complex_method_sync_callback, (void*)&out);
	serdes.header.tok = tok;

    (void)ipc_ser_finish(&serdes);

    // send message
    uint32_t i = 0;
    mutex_lock(&s_send_mtx);
    for (; i <= serdes.index; ++i)
    {
        ret = ipc_trans_layer_proxy_send_method(s_handle, serdes.msg_pool[i]);
        if (ret < 0)
            break;
    }
    mutex_unlock(&s_send_mtx);
    if (ret < 0)
    {
		clear_registry(reg);
        printf("send method complex_method fail %d.\n", ret);
        return ret;
    }

    //wait for reply
    if (timeout_ms <= 0)
        ret = wait_on_registry(reg);
    else
        ret = timedwait_on_registry(reg, timeout_ms);
    if (ret < 0)
    {
        clear_registry(reg);
        printf("%s wait timeout\n", __func__);
    }

    return ret;
}

static int32_t call_complex_method_async(const uint32_t in1, const char * in2, const byte_buffer in3, const test_MyArray_t in4, const test_MyStruct2_t in5, const test_MyUnion_t in6, test_complex_method_callback_t cb, void *ext)
{
    int32_t ret = 0;
    serdes_t serdes = {0};

    // serialize
    (void)ipc_ser_init(&serdes);

    ret = serialize_complex_method(&serdes, in1, in2, in3, in4, in5, in6);
	if (ret != 0)
	{
		printf("Serialize complex_method fail.\n");
		return -1;
	}

	// take and set registry
	uint8_t tok = 0;
	callback_registration_t * reg = take_registry(s_method_registry, &s_token, &tok);
	if (!reg)
		return -2;
	(void)add_registry(reg, (void *)cb, ext);
	serdes.header.tok = tok;

    (void)ipc_ser_finish(&serdes);

    // send message
    uint32_t i = 0;
    mutex_lock(&s_send_mtx);
    for (; i <= serdes.index; ++i)
    {
        ret = ipc_trans_layer_proxy_send_method(s_handle, serdes.msg_pool[i]);
        if (ret < 0)
            break;
    }
    mutex_unlock(&s_send_mtx);
    if (ret < 0)
    {
		clear_registry(reg);
        printf("send method complex_method fail %d.\n", ret);
        return ret;
    }

    return 0;
}

static inline int32_t call_complex_method_callback(serdes_t *des)
{
    if (!des)
        return -1;
    
    int32_t ret = 0;
    uint32_t length = 0;

    test_ErrorEnum_t err = {0};
	int32_t err_val = 0;
	ret = ipc_des_get(des, (uint8_t *)&err_val, sizeof(int32_t));
	if (ret < 0)
	{
	    return -1;
	}
	else
	{
	    err = err_val;
	    length += sizeof(int32_t);
	    if (length >= IPC_MAX_DATA_SIZE)
	        return -1;
	}
	
	if (err != NO_ERROR)
	{
	    uint32_t out1 = 0;
		char* out2 = NULL;
		byte_buffer out3 = {0};
		test_MyArray_t out4 = {0};
		test_MyStruct_t out5 = {0};
		test_MyUnion_t out6 = {0};
		
	    callback_registration_t *reg = &s_method_registry[des->header.tok];
		if (reg->busy)
		{
	        test_complex_method_callback_t cb = (test_complex_method_callback_t)(reg->cb);
	        if (cb)
	            cb(out1, out2, out3, out4, out5, out6, err, reg->ext);
	        notify_callback_registry(reg);
	        clear_registry(reg);
	    }
	    else
		{
			printf("callback registry is invalid.\n");
		}
	    return 0;
	}
	
	uint32_t out1 = 0;
	ret = ipc_des_get(des, (uint8_t *)&out1, sizeof(uint32_t));
	if (ret < 0)
	{
	    return -1;
	}
	else
	{
	    length += sizeof(uint32_t);
	    if (length >= IPC_MAX_DATA_SIZE)
	        return -1;
	}
	char* out2 = NULL;
	out2 = &s_recv_buffer[length];
	ret = ipc_des_get_string(des, out2, IPC_MAX_DATA_SIZE - length);
	if (ret < 0)
	{
	    return -1;
	}
	else
	{
	    length += ret;
	    if (length >= IPC_MAX_DATA_SIZE)
	        return -1;
	}
	byte_buffer out3 = {0};
	ret = ipc_des_get(des, (uint8_t *)&out3.size, sizeof(uint32_t));
	if (ret < 0)
	{
	    return -1;
	}
	else
	{
	    length += sizeof(uint32_t);
	    if (length >= IPC_MAX_DATA_SIZE)
	        return -1;
	}
	out3.data = (uint8_t *)&s_recv_buffer[length];
	ret = ipc_des_get(des, out3.data, out3.size);
	if (ret < 0)
	{
	    return -1;
	}
	else
	{
	    length += out3.size;
	    if (length >= IPC_MAX_DATA_SIZE)
	        return -1;
	}
	test_MyArray_t out4 = {0};
	ret = ipc_des_get(des, (uint8_t *)&out4.size, sizeof(uint32_t));
	if (ret < 0)
	{
	    return -1;
	}
	else
	{
	    length += sizeof(uint32_t);
	    if (length >= IPC_MAX_DATA_SIZE)
	        return -1;
	}
	out4.data = (uint16_t *)&s_recv_buffer[length];
	ret = ipc_des_get(des, (uint8_t *)out4.data, out4.size * sizeof(uint16_t));
	if (ret < 0)
	{
	    return -1;
	}
	else
	{
	    length += out4.size * sizeof(uint16_t);
	    if (length >= IPC_MAX_DATA_SIZE)
	        return -1;
	}
	test_MyStruct_t out5 = {0};
	ret = ipc_des_get(des, (uint8_t *)&out5.m1, sizeof(uint8_t));
	if (ret < 0)
	{
	    return -1;
	}
	else
	{
	    length += sizeof(uint8_t);
	    if (length >= IPC_MAX_DATA_SIZE)
	        return -1;
	}
	ret = ipc_des_get(des, (uint8_t *)&out5.m2, sizeof(bool));
	if (ret < 0)
	{
	    return -1;
	}
	else
	{
	    length += sizeof(bool);
	    if (length >= IPC_MAX_DATA_SIZE)
	        return -1;
	}
	ret = ipc_des_get(des, (uint8_t *)&out5.m3.size, sizeof(uint32_t));
	if (ret < 0)
	{
	    return -1;
	}
	else
	{
	    length += sizeof(uint32_t);
	    if (length >= IPC_MAX_DATA_SIZE)
	        return -1;
	}
	out5.m3.data = (uint16_t *)&s_recv_buffer[length];
	ret = ipc_des_get(des, (uint8_t *)out5.m3.data, out5.m3.size * sizeof(uint16_t));
	if (ret < 0)
	{
	    return -1;
	}
	else
	{
	    length += out5.m3.size * sizeof(uint16_t);
	    if (length >= IPC_MAX_DATA_SIZE)
	        return -1;
	}
	out5.m4 = &s_recv_buffer[length];
	ret = ipc_des_get_string(des, out5.m4, IPC_MAX_DATA_SIZE - length);
	if (ret < 0)
	{
	    return -1;
	}
	else
	{
	    length += ret;
	    if (length >= IPC_MAX_DATA_SIZE)
	        return -1;
	}
	ret = ipc_des_get(des, (uint8_t *)&out5.m5.size, sizeof(uint32_t));
	if (ret < 0)
	{
	    return -1;
	}
	else
	{
	    length += sizeof(uint32_t);
	    if (length >= IPC_MAX_DATA_SIZE)
	        return -1;
	}
	out5.m5.data = (uint8_t *)&s_recv_buffer[length];
	ret = ipc_des_get(des, out5.m5.data, out5.m5.size);
	if (ret < 0)
	{
	    return -1;
	}
	else
	{
	    length += out5.m5.size;
	    if (length >= IPC_MAX_DATA_SIZE)
	        return -1;
	}
	test_MyUnion_t out6 = {0};
	ret = ipc_des_get(des, (uint8_t *)&out6, sizeof(test_MyUnion_t));
	if (ret < 0)
	{
	    return -1;
	}
	else
	{
	    length += sizeof(test_MyUnion_t);
	    if (length >= IPC_MAX_DATA_SIZE)
	        return -1;
	}
	
    callback_registration_t *reg = &s_method_registry[des->header.tok];
	if (reg->busy)
	{
        test_complex_method_callback_t cb = (test_complex_method_callback_t)(reg->cb);
        if (cb)
            cb(out1, out2, out3, out4, out5, out6, err, reg->ext);
        notify_callback_registry(reg);
        clear_registry(reg);
    }
	else
	{
		printf("callback registry is invalid.\n");
	}
    return 0;
}

// broadcast

// subscribe heartbeat
static int32_t subscribe_heartbeat(test_heartbeat_callback_t cb, void *ext, test_heartbeat_sub_callback_t cb2,
                                        void *ext2)
{
    int32_t ret = 0;
    serdes_t serdes = {0};

    // prepare message
    serdes.header.pid = PID;
    serdes.header.cid = CID;
    serdes.header.fid = FID;
    serdes.header.sid = SID;
    serdes.header.cmd = CMD_METHOD_SUB_HEARTBEAT;
    serdes.header.typ = MSGBX_MSG_TYPE_METHOD;
    serdes.header.len = 0;
    serdes.header.is_eof = 1;

	// take and set registry
	uint8_t tok = 0;
	callback_registration_t * reg = take_registry(s_method_registry, &s_token, &tok);
	if (!reg)
		return -2;
	(void)add_registry(reg, (void*)cb2, ext2);
	serdes.header.tok = tok;

	s_heartbeat_registry.busy = true;
	(void)add_registry(&s_heartbeat_registry, (void *)cb, ext);

    (void)ipc_ser_finish(&serdes);

    // send message
    uint32_t i = 0;
    mutex_lock(&s_send_mtx);
    for (; i <= serdes.index; ++i)
    {
        ret = ipc_trans_layer_proxy_send_method(s_handle, serdes.msg_pool[i]);
        if (ret < 0)
            break;
    }
    mutex_unlock(&s_send_mtx);
    if (ret < 0)
    {
		clear_registry(&s_heartbeat_registry);
		clear_registry(reg);
        printf("send method subscribe_heartbeat fail %d.\n", ret);
        return ret;
    }
    return 0;
}

// unsubscribe heartbeat
static int32_t unsubscribe_heartbeat(test_heartbeat_unsub_callback_t cb, void *ext)
{
    int32_t ret = 0;
    serdes_t serdes = {0};

    // prepare message
    serdes.header.pid = PID;
    serdes.header.cid = CID;
    serdes.header.fid = FID;
    serdes.header.sid = SID;
    serdes.header.cmd = CMD_METHOD_UNSUB_HEARTBEAT;
    serdes.header.typ = MSGBX_MSG_TYPE_METHOD;
    serdes.header.len = 0;
    serdes.header.is_eof = 1;

	// take and set registry
	uint8_t tok = 0;
	callback_registration_t * reg = take_registry(s_method_registry, &s_token, &tok);
	if (!reg)
		return -2;
	(void)add_registry(reg, (void*)cb, ext);
	serdes.header.tok = tok;

    (void)ipc_ser_finish(&serdes);

    // send message
    uint32_t i = 0;
    mutex_lock(&s_send_mtx);
    for (; i <= serdes.index; ++i)
    {
        ret = ipc_trans_layer_proxy_send_method(s_handle, serdes.msg_pool[i]);
        if (ret < 0)
            break;
    }
    mutex_unlock(&s_send_mtx);
    if (ret < 0)
    {
		clear_registry(reg);
        printf("send method unsubscribe_heartbeat fail %d.\n", ret);
        return ret;
    }
	clear_registry(&s_heartbeat_registry);
	return 0;
}

static inline int32_t call_heartbeat_callback(serdes_t *des)
{
    if (!des)
        return -1;
    
    int32_t ret = 0;
    uint32_t length = 0;

    uint8_t status = 0;
	ret = ipc_des_get(des, (uint8_t *)&status, sizeof(uint8_t));
	if (ret < 0)
	{
	    return -1;
	}
	else
	{
	    length += sizeof(uint8_t);
	    if (length >= IPC_MAX_DATA_SIZE)
	        return -1;
	}
	
    callback_registration_t *reg = &s_heartbeat_registry;
	if (reg->busy)
	{
        test_heartbeat_callback_t cb = (test_heartbeat_callback_t)(reg->cb);
        if (cb)
            cb(status, reg->ext);
	}
	else
	{
		printf("heartbeat callback registry is invalid.\n");
	}
    return 0;
}

// receive messages
static int32_t receive_message(void)
{
    return ipc_trans_layer_get_msg(s_handle);
}

// dispatch messages
static int32_t dispatch_message(void)
{
    int32_t ret = 0;
    static serdes_t broadcast_des = {0};
    static serdes_t reply_des = {0};

    while (ipc_trans_layer_proxy_get_broadcast_msg(s_handle, ipc_des_get_current_msg(&broadcast_des)) >= 0)
    {
        switch (broadcast_des.header.cmd)
        {
        case CMD_BROADCAST_HEARTBEAT:
		    ret = call_heartbeat_callback(&broadcast_des);
		    break;
		
        default:
            break;
        }
        (void)ipc_des_init(&broadcast_des);
    }
    while (ipc_trans_layer_proxy_get_reply_msg(s_handle, ipc_des_get_current_msg(&reply_des)) >= 0)
    {
        switch (reply_des.header.cmd)
        {
        case CMD_METHOD_HELLO: 
		    ret = call_hello_callback(&reply_des);
		    break;
		case CMD_METHOD_COMPLEX_METHOD: 
		    ret = call_complex_method_callback(&reply_des);
		    break;
		
        case CMD_METHOD_SUB_HEARTBEAT: 
		{
		    int32_t err = 0;
		    ret = ipc_des_get(&reply_des, (uint8_t *)&err, sizeof(err));
		    if (ret >= 0)
		    {
		        callback_registration_t *reg = &s_method_registry[reply_des.header.tok];
		        test_heartbeat_sub_callback_t cb = (test_heartbeat_sub_callback_t)(reg->cb);
		        if (cb)
		            cb(err, reg->ext);
		        clear_registry(reg);
		    }
		    break;
		}
		case CMD_METHOD_UNSUB_HEARTBEAT: 
		{
		    int32_t err = 0;
		    ret = ipc_des_get(&reply_des, (uint8_t *)&err, sizeof(err));
		    if (ret >= 0)
		    {
		        callback_registration_t *reg = &s_method_registry[reply_des.header.tok];
		        test_heartbeat_unsub_callback_t cb = (test_heartbeat_unsub_callback_t)(reg->cb);
		        if (cb)
		            cb(err, reg->ext);
		        clear_registry(reg);
		    }
		    break;
		}
        default:
            break;
        }
        (void)ipc_des_init(&reply_des);
        if (ret < 0)
            printf("deserialization failed.\n");
    }
    return ret;
}

static struct task_struct *route_task = NULL;
static volatile bool bRunning = false;
static int router_func(void* arg)
{
    int32_t ret = 0;
    while (unlikely(!kthread_should_stop()))
    {
        ret = receive_message();
        if (ret < 0)
        {
            yield();
            continue;
        }
        ret = dispatch_message();
        if (ret < 0)
        {
            yield();
            continue;
        }
        yield();
    }

    return 0;
}

// start message router
static int32_t start(void)
{
    if (bRunning)
        return 0;

    route_task = kthread_run(router_func, NULL, "test_client_thread");
    if (unlikely(!route_task))
    {
        return -1;
    }
	bRunning = true;
    return 0;
}

// stop message router.
static int32_t stop(void)
{
    if (!bRunning)
        return 0;
   
    //sleep 1 seconds.
    msleep(1000);
    if (likely(route_task)) {
    	int32_t ret = kthread_stop(route_task);
		if (unlikely(ret))
			return -1;
    }
    bRunning = false;
    return 0;
}

// init server
test_client *test_client_init(void)
{
	int32_t ret = 0;
    // start trans layer.
    // int32_t ret = ipc_trans_layer_start(1);
    // if (ret < 0)
    //     return NULL;
    // create client handle.
    ret = ipc_trans_layer_proxy_create_handle(FID, SID, &s_handle);
    if (ret < 0)
        return NULL;

    s_client.version = get_ipc_inf_version;
    s_client.hello_sync = call_hello_sync;
	s_client.hello_async = call_hello_async;
	s_client.complex_method_sync = call_complex_method_sync;
	s_client.complex_method_async = call_complex_method_async;
	
    s_client.heartbeat_sub = subscribe_heartbeat;
	s_client.heartbeat_unsub = unsubscribe_heartbeat;
    s_client.start = start;
    s_client.stop = stop;
    
    mutex_init(&s_send_mtx);
	init_registry(&s_heartbeat_registry);
	init_registry_list(&s_method_registry[0], IPC_TOKEN_NUM);
    return &s_client;
}

// destory client
int32_t test_client_destroy(void)
{
    int32_t ret = ipc_trans_layer_destroy_handle(s_handle);
    if (ret < 0)
        return ret;
    // ret = ipc_trans_layer_stop();
    // if (ret < 0)
    //     return ret;

    s_client.version = NULL;
    s_client.hello_sync = NULL;
	s_client.hello_async = NULL;
	s_client.complex_method_sync = NULL;
	s_client.complex_method_async = NULL;
	
    s_client.heartbeat_sub = NULL;
	s_client.heartbeat_unsub = NULL;
    s_client.start = NULL;
    s_client.stop = NULL;

	destroy_registry(&s_heartbeat_registry);
	destroy_registry_list(&s_method_registry[0], IPC_TOKEN_NUM);
    return ret;
}
