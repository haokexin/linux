
/* © Copyright  Black Sesame Technologies (Chengdu) Co., Ltd. 2021. All rights reserved. 
 * This file contains proprietary information that is the sole intellectual property of 
 * Black Sesame Technologies (Chengdu) Co., Ltd. No part of this material or its 
 * documentation may be reproduced, distributed, transmitted, displayed or published in
 * any manner without the written permission of Black Sesame Technologies (Chengdu) Co.,
 * Ltd. Anyone who infringes on the copyright of Black Sesame Technologies (Chengdu) Co.,
 * Ltd. shall be held accountable by Black Sesame Technologies (Chengdu) Co., Ltd..
 */
#include <linux/dma-direct.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/hashtable.h>
#include <linux/highmem.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mman.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/sort.h>

#include "test_client.h"
#include "ipc_app_common.h"
//#include "ipc_app_serdes.h"
#include "ipc_app_client_utils.h"
#include "ipc_trans_common.h"
#include "ipc_trans_layer.h"

// macro definitions
#define PID CPU_0
#define CID MEDIA_0
#define FID DEF
#define SID 2U

#define CMD_METHOD_HELLO 1U
#define CMD_METHOD_HIFI_A78_MSG_SYNC 2U

#define CMD_METHOD_SUB_HEARTBEAT 10U
#define CMD_METHOD_UNSUB_HEARTBEAT 11U
#define CMD_BROADCAST_HEARTBEAT 1U

// local variables
static ipc_inf_version s_version = {.major = 1, .minor = 0};
static uint8_t s_handle = 0U;
static test_hifi_dsp_client s_client = {0};
static uint8_t s_token = 0;
static int8_t s_recv_buffer[IPC_MAX_DATA_SIZE] = {0};
static callback_registration_t s_method_registry[IPC_TOKEN_NUM] = {0};
static callback_registration_t s_heartbeat_registry = {0};
static struct mutex s_send_mtx = {0};

static inline void increase_token(void)
{
    if (++s_token >= IPC_TOKEN_NUM)
        s_token = 0;
}

// interface implementation
// get interface version
static ipc_inf_version get_ipc_inf_version(void)
{
    return s_version;
}

// method

static int32_t call_hello_async(const char * name, test_hello_callback_t cb, void *ext)
{
    int32_t ret = 0;
    serdes_t serdes = {0};
    serdes_t *ser = &serdes;

    // prepare message
    serdes.header.pid = PID;
    serdes.header.cid = CID;
    serdes.header.fid = FID;
    serdes.header.sid = SID;
    serdes.header.tok = s_token;
    serdes.header.cmd = CMD_METHOD_HELLO;
    serdes.header.typ = IPC_MSG_TYPE_METHOD;

    // serialize
    (void)ipc_ser_init(ser);
    ret = ipc_ser_put_string(ser, name);
	if (ret < 0)
	    return -1;
	
    (void)ipc_ser_finish(ser);
	
    // send message
    uint32_t i = 0;
 
    mutex_lock(&s_send_mtx);
    ret = ipc_trans_layer_proxy_send_method(PID, s_handle, ser);
    mutex_unlock(&s_send_mtx);
    if (ret < 0)
    {
        printf("send method hello fail %d.\n", ret);
        return ret;
    }

	add_registry(&s_method_registry[s_token], (void *)cb, (void *)ext);
    increase_token();
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
	    test_hello_callback_t cb = (test_hello_callback_t)(reg->cb);
	    if (cb)
	        cb(message, err, reg->ext);
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
    test_hello_callback_t cb = (test_hello_callback_t)(reg->cb);
    if (cb)
        cb(message, err, reg->ext);
  
    return 0;
}



static int32_t call_hifi_a78_msg_sync_async(const test_xrp_msg_t fs_msg, test_hifi_a78_msg_sync_callback_t cb, void *ext)
{
    int32_t ret = 0;
    serdes_t serdes = {0};
    serdes_t *ser = &serdes;

    // prepare message
    serdes.header.pid = PID;
    serdes.header.cid = CID;
    serdes.header.fid = FID;
    serdes.header.sid = SID;
    serdes.header.tok = s_token;
    serdes.header.cmd = CMD_METHOD_HIFI_A78_MSG_SYNC;
    serdes.header.typ = IPC_MSG_TYPE_METHOD;

    // serialize
    (void)ipc_ser_init(ser);
    ret = ipc_ser_put(ser, (uint8_t*)&fs_msg.opcode, sizeof(uint8_t));
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, (uint8_t*)&fs_msg.user_data.size, sizeof(uint32_t));
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, (uint8_t*)fs_msg.user_data.data, fs_msg.user_data.size * sizeof(uint8_t));
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, (uint8_t*)&fs_msg.i_name_space_id.size, sizeof(uint32_t));
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, (uint8_t*)fs_msg.i_name_space_id.data, fs_msg.i_name_space_id.size * sizeof(uint8_t));
	if (ret < 0)
	    return -1;
	
    (void)ipc_ser_finish(ser);

	
    // send message
    uint32_t i = 0;
    mutex_lock(&s_send_mtx);
    ret = ipc_trans_layer_proxy_send_method(PID, s_handle, ser);
    mutex_unlock(&s_send_mtx);

    if (ret < 0)
    {
        printf("send method hifi_a78_msg_sync fail %d.\n", ret);
        return ret;
    }

	add_registry(&s_method_registry[s_token], (void *)cb, (void *)ext);
    increase_token();
    return 0;

}

static inline int32_t call_hifi_a78_msg_sync_callback(serdes_t *des)
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
	    uint8_t response = 0;
		test_Array_Uint8_t resp_data = {0};
		
	    callback_registration_t *reg = &s_method_registry[des->header.tok];
	    test_hifi_a78_msg_sync_callback_t cb = (test_hifi_a78_msg_sync_callback_t)(reg->cb);
	    if (cb)
	        cb(response, resp_data, err, reg->ext);
	    return 0;
	}
	
	uint8_t response = 0;
	ret = ipc_des_get(des, (uint8_t *)&response, sizeof(uint8_t));
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
	test_Array_Uint8_t resp_data = {0};
	ret = ipc_des_get(des, (uint8_t *)&resp_data.size, sizeof(uint32_t));
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
	resp_data.data = (uint8_t *)&s_recv_buffer[length];
	ret = ipc_des_get(des, (uint8_t *)resp_data.data, resp_data.size * sizeof(uint8_t));
	if (ret < 0)
	{
	    return -1;
	}
	else
	{
	    length += resp_data.size * sizeof(uint8_t);
	    if (length >= IPC_MAX_DATA_SIZE)
	        return -1;
	}
	
    callback_registration_t *reg = &s_method_registry[des->header.tok];
    test_hifi_a78_msg_sync_callback_t cb = (test_hifi_a78_msg_sync_callback_t)(reg->cb);
    if (cb)
        cb(response, resp_data, err, reg->ext);


    return 0;
}

// broadcast

// subscribe heartbeat
static int32_t subscribe_heartbeat(test_heartbeat_callback_t cb, void *ext, test_heartbeat_sub_callback_t cb2,
                                        void *ext2)
{
    int32_t ret = 0;

    serdes_t serdes = {0};
    serdes_t *ser = &serdes;

    // prepare message
    serdes.header.pid = PID;
    serdes.header.cid = CID;
    serdes.header.fid = FID;
    serdes.header.sid = SID;
    serdes.header.tok = s_token;
    serdes.header.cmd = CMD_METHOD_SUB_HEARTBEAT;
    serdes.header.typ = IPC_MSG_TYPE_METHOD;
    serdes.header.len = 0;
    serdes.header.is_eof = 1;
    // serialize
    (void)ipc_ser_init(ser);

    (void)ipc_ser_finish(ser);

    mutex_lock(&s_send_mtx);
    ret = ipc_trans_layer_proxy_send_method(PID, s_handle, ser);
    mutex_unlock(&s_send_mtx);
    if (ret < 0)
    {
        printf("send method subscribe_heartbeat fail %d.\n", ret);
        return ret;
    }
    else
    {
		add_registry(&s_heartbeat_registry, (void *)cb, ext);
		add_registry(&s_method_registry[s_token], (void *)cb2, ext2);
        increase_token();
        return 0;
    }
}

// unsubscribe heartbeat
static int32_t unsubscribe_heartbeat(test_heartbeat_unsub_callback_t cb, void *ext)
{
    int32_t ret = 0;

    serdes_t serdes = {0};
    serdes_t *ser = &serdes;

    // prepare message
    serdes.header.pid = PID;
    serdes.header.cid = CID;
    serdes.header.fid = FID;
    serdes.header.sid = SID;
    serdes.header.tok = s_token;
    serdes.header.cmd = CMD_METHOD_UNSUB_HEARTBEAT;
    serdes.header.typ = IPC_MSG_TYPE_METHOD;
    serdes.header.len = 0;
    serdes.header.is_eof = 1;

    // serialize
    (void)ipc_ser_init(ser);

    (void)ipc_ser_finish(ser);

    mutex_lock(&s_send_mtx);
    ret = ipc_trans_layer_proxy_send_method(PID, s_handle, ser);
    mutex_unlock(&s_send_mtx);

    if (ret < 0)
    {
        printf("send method unsubscribe_heartbeat fail %d.\n", ret);
        return ret;
    }
    else
    {
		add_registry(&s_method_registry[s_token], (void *)cb, ext);
        increase_token();
        return 0;
    }

}

static inline int32_t call_heartbeat_callback(serdes_t *des)
{
    if (!des)
        return -1;
    
    int32_t ret = 0;
    uint32_t length = 0;

    test_hifi_a78_result_t fs_result = {0};
	ret = ipc_des_get(des, (uint8_t *)&fs_result.opcode, sizeof(uint8_t));
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
	ret = ipc_des_get(des, (uint8_t *)&fs_result.data_paddr, sizeof(uint32_t));
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
	ret = ipc_des_get(des, (uint8_t *)&fs_result.status, sizeof(uint8_t));
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
    test_heartbeat_callback_t cb = (test_heartbeat_callback_t)(reg->cb);
    if (cb)
        cb(fs_result, reg->ext);

    return 0;
}

// receive messages
static int32_t receive_message(void)
{
    return ipc_trans_layer_query_info(PID, s_handle);
}

// dispatch messages
static int32_t dispatch_message(void)
{
    int32_t ret = 0;
    static serdes_t broadcast_des = {0};
    static serdes_t reply_des = {0};

//    while (ipc_trans_layer_proxy_get_broadcast_msg(s_handle, ipc_des_get_current_msg(&broadcast_des)) >= 0)
    while (ipc_trans_layer_proxy_get_broadcast_msg(PID, s_handle, &broadcast_des) >= 0)
    {
//        ret = ipc_des_validate_msg(&broadcast_des);
 //       if (ret < 0)
 //           continue;
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
//    while (ipc_trans_layer_proxy_get_reply_msg(s_handle, ipc_des_get_current_msg(&reply_des)) >= 0)
    while (ipc_trans_layer_proxy_get_reply_msg(PID, s_handle, &reply_des) >= 0)
    {
 //       ret = ipc_des_validate_msg(&reply_des);
 //       if (ret < 0)
 //           continue;
        switch (reply_des.header.cmd)
        {
            case CMD_METHOD_HELLO: 
                ret = call_hello_callback(&reply_des);
                break;
            case CMD_METHOD_HIFI_A78_MSG_SYNC: 
                ret = call_hifi_a78_msg_sync_callback(&reply_des);
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

// init server
test_hifi_dsp_client *test_hifi_dsp_client_init(void)
{
    // start trans layer.
    #if 0
    int32_t ret = ipc_trans_layer_start(1);
    if (ret < 0)
        return NULL;
    #endif
    int32_t ret=0;
    // create client handle.
//    ret = ipc_trans_layer_proxy_create_handle(FID, SID, &s_handle);
    printf("111.\n");
    ret = ipc_trans_layer_proxy_create_handle(PID, FID, SID, CID, &s_handle);
    if (ret < 0)
        return NULL;
    printf("222.\n");
    s_client.version = get_ipc_inf_version;
    s_client.hello = call_hello_async;
	s_client.hifi_a78_msg_sync = call_hifi_a78_msg_sync_async;
	
    s_client.heartbeat_sub = subscribe_heartbeat;
	s_client.heartbeat_unsub = unsubscribe_heartbeat;
    s_client.receive_message = receive_message;
    s_client.dispatch_message = dispatch_message;
    s_client.def_xrp_msg  =kzalloc(sizeof(test_xrp_msg_t),GFP_KERNEL);
    pr_debug("%s: 333 \n", __func__);
    mutex_init(&s_send_mtx);
    pr_debug("%s: 444 \n", __func__);
	init_registry(&s_heartbeat_registry);
    pr_debug("%s: 555 \n", __func__);
	init_registry_list(s_method_registry, IPC_TOKEN_NUM);
    return &s_client;
}

// destory client
int32_t test_hifi_dsp_client_destroy(void)
{
    int32_t ret = ipc_trans_layer_destory_handle(PID, s_handle);
    if (ret < 0)
        return ret;
    ret = ipc_trans_layer_stop(PID);
    if (ret < 0)
        return ret;

    s_client.version = NULL;
    s_client.hello = NULL;
	s_client.hifi_a78_msg_sync = NULL;
	
    s_client.heartbeat_sub = NULL;
	s_client.heartbeat_unsub = NULL;
    s_client.receive_message = NULL;
    s_client.dispatch_message = NULL;
    kfree(s_client.def_xrp_msg);
    s_client.def_xrp_msg =NULL;

    destroy_registry(&s_heartbeat_registry);
	destroy_registry_list(s_method_registry, IPC_TOKEN_NUM);
    return ret;
}
