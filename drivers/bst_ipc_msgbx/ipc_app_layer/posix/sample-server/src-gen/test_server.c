
/* © Copyright  Black Sesame Technologies (Chengdu) Co., Ltd. 2021. All rights reserved. 
 * This file contains proprietary information that is the sole intellectual property of 
 * Black Sesame Technologies (Chengdu) Co., Ltd. No part of this material or its 
 * documentation may be reproduced, distributed, transmitted, displayed or published in
 * any manner without the written permission of Black Sesame Technologies (Chengdu) Co.,
 * Ltd. Anyone who infringes on the copyright of Black Sesame Technologies (Chengdu) Co.,
 * Ltd. shall be held accountable by Black Sesame Technologies (Chengdu) Co., Ltd..
 */


#include <bst/ipc_app_common.h>
#include <bst/ipc_app_svr_utils.h>
#include <bst/ipc_trans_common.h>
#include <bst/ipc_trans_layer.h>
#include <bst/ipc_serdes.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/slab.h>

#include "test_server.h"

// macro definitions
#define PID CPU_7
#define FID DEF
#define SID 1U
#define MAX_METHOD_NUM 10U
#define MAX_BROADCAST_NUM 10U

#define CMD_METHOD_HELLO 1U
#define CMD_METHOD_COMPLEX_METHOD 2U

#define CMD_METHOD_SUB_HEARTBEAT 10U
#define CMD_METHOD_UNSUB_HEARTBEAT 11U
#define CMD_BROADCAST_HEARTBEAT 1U

// local variables
static ipc_inf_version_t s_version = {.major = 1, .minor = 0};
static uint32_t s_handle = 0U;
static test_server s_server = {0};
static volatile uint8_t s_token = 0;
static int8_t s_recv_buffer[IPC_MAX_DATA_SIZE] = {0};

static test_hello_t s_hello_ptr = NULL;
static test_complex_method_t s_complex_method_ptr = NULL;

static test_broadcast_sub_t s_heartbeat_sub_ptr = NULL;
static test_broadcast_sub_t s_heartbeat_unsub_ptr = NULL;
static broadcast_registry s_heartbeat_registry = {0};


static inline void increase_token(void)
{
    if (++s_token >= IPC_TOKEN_NUM)
        s_token = 0;
}

// interface implementation
// get interface version
static ipc_inf_version_t get_ipc_inf_version(void)
{
    return s_version;
}

// method

static int32_t register_hello(test_hello_t func)
{
    s_hello_ptr = func;
    return 0;
}

static int32_t call_hello(serdes_t *des, serdes_t *ser)
{
    if (!des || !ser || !s_hello_ptr)
        return -1;

    int32_t ret = 0;
    uint32_t length = 0;

    char* name = NULL;
	name = &s_recv_buffer[length];
	ret = ipc_des_get_string(des, name, IPC_MAX_DATA_SIZE - length);
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
	
    test_ErrorEnum_t err = {0};
	char* message = NULL;
	
    (*s_hello_ptr)(name, &message, &err);

    int32_t err_val = err;
	ret = ipc_ser_put(ser, (uint8_t*)&err_val, sizeof(int32_t));
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put_string(ser, message);
	if (ret < 0)
	    return -1;
	
    return 0;
}

static int32_t register_complex_method(test_complex_method_t func)
{
    s_complex_method_ptr = func;
    return 0;
}

static int32_t call_complex_method(serdes_t *des, serdes_t *ser)
{
    if (!des || !ser || !s_complex_method_ptr)
        return -1;

    int32_t ret = 0;
    uint32_t length = 0;

    uint32_t in1 = 0;
	ret = ipc_des_get(des, (uint8_t *)&in1, sizeof(uint32_t));
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
	char* in2 = NULL;
	in2 = &s_recv_buffer[length];
	ret = ipc_des_get_string(des, in2, IPC_MAX_DATA_SIZE - length);
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
	byte_buffer in3 = {0};
	ret = ipc_des_get(des, (uint8_t *)&in3.size, sizeof(uint32_t));
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
	in3.data = (uint8_t *)&s_recv_buffer[length];
	ret = ipc_des_get(des, in3.data, in3.size);
	if (ret < 0)
	{
	    return -1;
	}
	else
	{
	    length += in3.size;
	    if (length >= IPC_MAX_DATA_SIZE)
	        return -1;
	}
	test_MyArray_t in4 = {0};
	ret = ipc_des_get(des, (uint8_t *)&in4.size, sizeof(uint32_t));
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
	in4.data = (uint16_t *)&s_recv_buffer[length];
	ret = ipc_des_get(des, (uint8_t *)in4.data, in4.size * sizeof(uint16_t));
	if (ret < 0)
	{
	    return -1;
	}
	else
	{
	    length += in4.size * sizeof(uint16_t);
	    if (length >= IPC_MAX_DATA_SIZE)
	        return -1;
	}
	test_MyStruct_t in5 = {0};
	ret = ipc_des_get(des, (uint8_t *)&in5.m1, sizeof(uint8_t));
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
	ret = ipc_des_get(des, (uint8_t *)&in5.m2, sizeof(bool));
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
	ret = ipc_des_get(des, (uint8_t *)&in5.m3.size, sizeof(uint32_t));
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
	in5.m3.data = (uint16_t *)&s_recv_buffer[length];
	ret = ipc_des_get(des, (uint8_t *)in5.m3.data, in5.m3.size * sizeof(uint16_t));
	if (ret < 0)
	{
	    return -1;
	}
	else
	{
	    length += in5.m3.size * sizeof(uint16_t);
	    if (length >= IPC_MAX_DATA_SIZE)
	        return -1;
	}
	in5.m4 = &s_recv_buffer[length];
	ret = ipc_des_get_string(des, in5.m4, IPC_MAX_DATA_SIZE - length);
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
	ret = ipc_des_get(des, (uint8_t *)&in5.m5.size, sizeof(uint32_t));
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
	in5.m5.data = (uint8_t *)&s_recv_buffer[length];
	ret = ipc_des_get(des, in5.m5.data, in5.m5.size);
	if (ret < 0)
	{
	    return -1;
	}
	else
	{
	    length += in5.m5.size;
	    if (length >= IPC_MAX_DATA_SIZE)
	        return -1;
	}
	test_MyUnion_t in6 = {0};
	ret = ipc_des_get(des, (uint8_t *)&in6, sizeof(test_MyUnion_t));
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
	
    test_ErrorEnum_t err = {0};
	uint32_t out1 = 0;
	char* out2 = NULL;
	byte_buffer out3 = {0};
	test_MyArray_t out4 = {0};
	test_MyStruct_t out5 = {0};
	test_MyUnion_t out6 = {0};
	
    (*s_complex_method_ptr)(in1, in2, in3, in4, in5, in6, &out1, &out2, &out3, &out4, &out5, &out6, &err);

    int32_t err_val = err;
	ret = ipc_ser_put(ser, (uint8_t*)&err_val, sizeof(int32_t));
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, (uint8_t*)&out1, sizeof(uint32_t));
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put_string(ser, out2);
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, (uint8_t*)&out3.size, sizeof(uint32_t));
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, out3.data, out3.size);
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, (uint8_t*)&out4.size, sizeof(uint32_t));
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, (uint8_t*)out4.data, out4.size * sizeof(uint16_t));
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, (uint8_t*)&out5.m1, sizeof(uint8_t));
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, (uint8_t*)&out5.m2, sizeof(bool));
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, (uint8_t*)&out5.m3.size, sizeof(uint32_t));
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, (uint8_t*)out5.m3.data, out5.m3.size * sizeof(uint16_t));
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put_string(ser, out5.m4);
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, (uint8_t*)&out5.m5.size, sizeof(uint32_t));
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, out5.m5.data, out5.m5.size);
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, (uint8_t*)&out6, sizeof(test_MyUnion_t));
	if (ret < 0)
	    return -1;
	
    return 0;
}


// broadcast

static int32_t register_heartbeat_subcribed(test_broadcast_sub_t func)
{
    s_heartbeat_sub_ptr = func;
    return 0;
}

static int32_t register_heartbeat_unsubcribed(test_broadcast_sub_t func)
{
    s_heartbeat_unsub_ptr = func;
    return 0;
}

static int32_t heartbeat(uint8_t status)
{
    int32_t ret = 0;
    int32_t index = 0;
    rw_msg_header header = {0};
    header.pid = PID;
    header.cmd = CMD_BROADCAST_HEARTBEAT;
    header.typ = MSGBX_MSG_TYPE_BROADCAST;

    serdes_t serdes = {0};
    (void)ipc_ser_init(&serdes);
    serdes_t *ser = &serdes;
    ret = ipc_ser_put(ser, (uint8_t*)&status, sizeof(uint8_t));
	if (ret < 0)
	    return -1;
	
    broadcast_reg_entry *entry = s_heartbeat_registry.entries;
    for (index = s_heartbeat_registry.start, entry += s_heartbeat_registry.start; index < s_heartbeat_registry.end;
         ++index, ++entry)
    {
        if (entry->pid != 0)
        {
            uint32_t i = 0;
            header.cid = entry->pid;
            header.fid = entry->fid;
            header.sid = entry->sid;
            header.tok = s_token;
            ipc_ser_set_header(&serdes, header);
            ipc_ser_finish(&serdes);
            for (; i <= serdes.index; ++i)
            {
		int32_t send_ret = ipc_trans_layer_stub_send_broadcast(s_handle, serdes.msg_pool[i]);
                if (send_ret < 0)
                {
                    printf("send broadcast fail %d.\n", send_ret);
                    break;
                }
            }
            if (i > serdes.index)
            {
                ++ret;
            }
        }
    }
    increase_token();
    return ret;
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
	uint32_t i;
    static serdes_t deserializer = {0};
    static serdes_t serializer = {0};

    while (ipc_trans_layer_stub_get_method_msg(s_handle, ipc_des_get_current_msg(&deserializer)) >= 0)
    {
        ret = ipc_des_validate_msg(&deserializer);
        if (ret < 0)
            continue;
        // init serializer.
        // it cannot fail, as &serializer won't be NULL.
        (void)ipc_ser_init(&serializer);
        // process message.
        switch (deserializer.header.cmd)
        {
        case CMD_METHOD_HELLO:
		    ret = call_hello(&deserializer, &serializer);
		    if (ret < 0)
		    {
		        printf("call_hello failed.\n");
		        (void)ipc_des_init(&deserializer);
		        (void)ipc_ser_init(&serializer);
		        ret = -1;
		        ret = ipc_ser_put(&serializer, (uint8_t *)&ret, sizeof(ret));
		    }
		    break;
		case CMD_METHOD_COMPLEX_METHOD:
		    ret = call_complex_method(&deserializer, &serializer);
		    if (ret < 0)
		    {
		        printf("call_complex_method failed.\n");
		        (void)ipc_des_init(&deserializer);
		        (void)ipc_ser_init(&serializer);
		        ret = -1;
		        ret = ipc_ser_put(&serializer, (uint8_t *)&ret, sizeof(ret));
		    }
		    break;
		
        case CMD_METHOD_SUB_HEARTBEAT:
		    if (s_heartbeat_sub_ptr)
		        (*s_heartbeat_sub_ptr)((uint8_t)deserializer.header.pid, (uint8_t)deserializer.header.fid, 
		                                (uint8_t)deserializer.header.sid);
		    ret = add_registration(&s_heartbeat_registry, (uint8_t)deserializer.header.pid,
		                            (uint8_t)deserializer.header.fid, (uint8_t)deserializer.header.sid);
		    ret = ipc_ser_put(&serializer, (uint8_t *)&ret, sizeof(ret));
		    break;
		case CMD_METHOD_UNSUB_HEARTBEAT:
		    if (s_heartbeat_unsub_ptr)
		        (*s_heartbeat_unsub_ptr)((uint8_t)deserializer.header.pid,
		                                (uint8_t)deserializer.header.fid, (uint8_t)deserializer.header.sid);
		    ret = remove_registration(&s_heartbeat_registry, (uint8_t)deserializer.header.pid,
		                                (uint8_t)deserializer.header.fid, (uint8_t)deserializer.header.sid);
		    ret = ipc_ser_put(&serializer, (uint8_t *)&ret, sizeof(ret));
		    break;
		
        default:
            ret = -1;
            ret = ipc_ser_put(&serializer, (uint8_t *)&ret, sizeof(ret));
            break;
        }
        serializer.header = deserializer.header;
        serializer.header.cid = deserializer.header.pid;
        serializer.header.pid = PID;
        serializer.header.typ = MSGBX_MSG_TYPE_REPLY;
        if (ret >= 0)
            ret = ipc_ser_finish(&serializer);

        if (ret >= 0)
        {
            for (i = 0; i <= serializer.index; ++i)
            {
                ret = ipc_trans_layer_stub_send_reply_msg(s_handle, serializer.msg_pool[i]);
                if (ret < 0)
                {
                    printk("send reply fail %d.\n", ret);
                    break;
                }
            }
        }

        deserializer.rcv_index = 0;
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

    route_task = kthread_run(router_func, NULL, "test_server_thread");
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
test_server *test_server_init()
{
	int32_t ret = 0;
    // start trans layer.
    // int32_t ret = ipc_trans_layer_start(0);
    // if (ret < 0)
    //     return NULL;

    // create server handle.
    ret = ipc_trans_layer_stub_create_handle(FID, SID, &s_handle);
    if (ret < 0)
	{
		//(void)ipc_trans_layer_stop(pid);
		return NULL;
	}
    
    // reset hello pointer.
    s_hello_ptr = NULL;

    // reset complex_method pointer.
    s_complex_method_ptr = NULL;

#if (IPC_TRANS_LAYER_SES_MODE == 2 && SESSION_COUNT > 1)
    // register CMDs.
    ret = ipc_trans_layer_register_method( s_handle, CMD_METHOD_HELLO);
    if (ret < 0)
	{
        (void)ipc_trans_layer_unregister_method( s_handle);
        (void)ipc_trans_layer_destroy_handle( s_handle);
		//(void)ipc_trans_layer_stop(pid);
		return NULL;
	}
    ret = ipc_trans_layer_register_method( s_handle, CMD_METHOD_COMPLEX_METHOD);
    if (ret < 0)
	{
        (void)ipc_trans_layer_unregister_method( s_handle);
        (void)ipc_trans_layer_destroy_handle( s_handle);
		//(void)ipc_trans_layer_stop(pid);
		return NULL;
	}
    ret = ipc_trans_layer_register_method( s_handle, CMD_METHOD_SUB_HEARTBEAT);
    if (ret < 0)
	{
        (void)ipc_trans_layer_unregister_method( s_handle);
        (void)ipc_trans_layer_destroy_handle( s_handle);
		//(void)ipc_trans_layer_stop(pid);
		return NULL;
	}

    ret = ipc_trans_layer_register_method( s_handle, CMD_METHOD_UNSUB_HEARTBEAT);
    if (ret < 0)
	{
        (void)ipc_trans_layer_unregister_method(s_handle);
        (void)ipc_trans_layer_destroy_handle(s_handle);
		//(void)ipc_trans_layer_stop(pid);
		return NULL;
	}
#endif

    
    s_server.version = get_ipc_inf_version;
    s_server.register_hello = register_hello;
	s_server.register_complex_method = register_complex_method;
	
    s_server.heartbeat = heartbeat;
    s_server.register_heartbeat_subcribed = register_heartbeat_subcribed;
    s_server.register_heartbeat_unsubcribed = register_heartbeat_unsubcribed;
    
    s_server.start = start;
    s_server.stop = stop;
    return &s_server;
}

// destory client
int32_t test_server_destroy()
{
	int32_t ret;
#if (IPC_TRANS_LAYER_SES_MODE == 2 && SESSION_COUNT > 1)
    ret = ipc_trans_layer_unregister_method(s_handle);
	if (ret < 0)
		return ret;
#endif
    ret = ipc_trans_layer_destroy_handle(s_handle);
    if (ret < 0)
        return ret;
    // ret = ipc_trans_layer_stop();
    // if (ret < 0)
    //     return ret;

    s_server.version = NULL;
    s_server.register_hello = NULL;
	s_server.register_complex_method = NULL;
	
    s_server.heartbeat = NULL;
    s_server.register_heartbeat_subcribed = NULL;
    s_server.register_heartbeat_unsubcribed = NULL;
    
    s_server.start = NULL;
    s_server.stop = NULL;
    
    // reset hello pointer.
    s_hello_ptr = NULL;

    // reset complex_method pointer.
    s_complex_method_ptr = NULL;

    return ret;
}
