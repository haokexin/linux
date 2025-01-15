
/* © Copyright  Black Sesame Technologies (Chengdu) Co., Ltd. 2021. All rights reserved. 
 * This file contains proprietary information that is the sole intellectual property of 
 * Black Sesame Technologies (Chengdu) Co., Ltd. No part of this material or its 
 * documentation may be reproduced, distributed, transmitted, displayed or published in
 * any manner without the written permission of Black Sesame Technologies (Chengdu) Co.,
 * Ltd. Anyone who infringes on the copyright of Black Sesame Technologies (Chengdu) Co.,
 * Ltd. shall be held accountable by Black Sesame Technologies (Chengdu) Co., Ltd..
 */


#include "test_server.h"
#include "ipc_app_common.h"
#include "ipc_app_serdes.h"
#include "ipc_app_svr_utils.h"
#include "ipc_trans_common.h"
#include "ipc_trans_layer.h"

// macro definitions
#define PID MEDIA_0
#define FID DEF
#define SID 1U
#define MAX_METHOD_NUM 10U
#define MAX_BROADCAST_NUM 10U

#define CMD_METHOD_HELLO 1U
#define CMD_METHOD_HIFI_A78_MSG_SYNC 2U

#define CMD_METHOD_SUB_HEARTBEAT 10U
#define CMD_METHOD_UNSUB_HEARTBEAT 11U
#define CMD_BROADCAST_HEARTBEAT 1U

// local variables
static ipc_inf_version s_version = {.major = 1, .minor = 0};
static uint32_t s_handle = 0U;
static test_server s_server = {0};
static uint8_t s_token = 0;
static int8_t s_recv_buffer[IPC_MAX_DATA_SIZE] = {0};

static test_hello_t s_hello_ptr = NULL;
static test_hifi_a78_msg_sync_t s_hifi_a78_msg_sync_ptr = NULL;

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
static ipc_inf_version get_ipc_inf_version(void)
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

static int32_t register_hifi_a78_msg_sync(test_hifi_a78_msg_sync_t func)
{
    s_hifi_a78_msg_sync_ptr = func;
    return 0;
}

static int32_t call_hifi_a78_msg_sync(serdes_t *des, serdes_t *ser)
{
    if (!des || !ser || !s_hifi_a78_msg_sync_ptr)
        return -1;

    int32_t ret = 0;
    uint32_t length = 0;

    test_hifi_a78_msg_t fs_msg = {0};
	ret = ipc_des_get(des, (uint8_t *)&fs_msg.opcode, sizeof(uint8_t));
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
	ret = ipc_des_get(des, (uint8_t *)&fs_msg.user_data.size, sizeof(uint32_t));
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
	fs_msg.user_data.data = (uint8_t *)&s_recv_buffer[length];
	ret = ipc_des_get(des, (uint8_t *)fs_msg.user_data.data, fs_msg.user_data.size * sizeof(uint8_t));
	if (ret < 0)
	{
	    return -1;
	}
	else
	{
	    length += fs_msg.user_data.size * sizeof(uint8_t);
	    if (length >= IPC_MAX_DATA_SIZE)
	        return -1;
	}
	ret = ipc_des_get(des, (uint8_t *)&fs_msg.i_name_space_id.size, sizeof(uint32_t));
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
	fs_msg.i_name_space_id.data = (uint8_t *)&s_recv_buffer[length];
	ret = ipc_des_get(des, (uint8_t *)fs_msg.i_name_space_id.data, fs_msg.i_name_space_id.size * sizeof(uint8_t));
	if (ret < 0)
	{
	    return -1;
	}
	else
	{
	    length += fs_msg.i_name_space_id.size * sizeof(uint8_t);
	    if (length >= IPC_MAX_DATA_SIZE)
	        return -1;
	}
	
    test_ErrorEnum_t err = {0};
	uint8_t response = 0;
	test_Array_Uint8_t resp_data = {0};
	
    (*s_hifi_a78_msg_sync_ptr)(fs_msg, &response, &resp_data, &err);

    int32_t err_val = err;
	ret = ipc_ser_put(ser, (uint8_t*)&err_val, sizeof(int32_t));
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, (uint8_t*)&response, sizeof(uint8_t));
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, (uint8_t*)&resp_data.size, sizeof(uint32_t));
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, (uint8_t*)resp_data.data, resp_data.size * sizeof(uint8_t));
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

static int32_t heartbeat(test_xrp_result_t fs_result)
{
    int32_t ret = 0;
    int32_t index = 0;
    rw_msg_header header = {0};
    header.pid = PID;
    header.cmd = CMD_BROADCAST_HEARTBEAT;
    header.typ = IPC_MSG_TYPE_BROADCAST;

    serdes_t serdes = {0};
    (void)ipc_ser_init(&serdes);
    serdes_t *ser = &serdes;
    ret = ipc_ser_put(ser, (uint8_t*)&fs_result.opcode, sizeof(uint8_t));
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, (uint8_t*)&fs_result.data_paddr, sizeof(uint32_t));
	if (ret < 0)
	    return -1;
	ret = ipc_ser_put(ser, (uint8_t*)&fs_result.status, sizeof(uint8_t));
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
                ++ret;
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
		case CMD_METHOD_HIFI_A78_MSG_SYNC:
		    ret = call_hifi_a78_msg_sync(&deserializer, &serializer);
		    if (ret < 0)
		    {
		        printf("call_hifi_a78_msg_sync failed.\n");
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
        serializer.header.typ = IPC_MSG_TYPE_REPLY;
        if (ret >= 0)
            ret = ipc_ser_finish(&serializer);

        if (ret >= 0)
        {
            for (uint32_t i = 0; i <= serializer.index; ++i)
            {
                ret = ipc_trans_layer_stub_send_reply_msg(s_handle, serializer.msg_pool[i]);
                if (ret < 0)
                {
                    printf("send reply fail %d.\n", ret);
                    break;
                }
            }
        }

        deserializer.rcv_index = 0;
    }
    return ret;
}

// init server
test_server *test_server_init(void)
{
    // start trans layer.
    int32_t ret = ipc_trans_layer_start(0);
    if (ret < 0)
        return NULL;

    // create server handle.
    ret = ipc_trans_layer_stub_create_handle(FID, SID, &s_handle);
    if (ret < 0)
	{
		(void)ipc_trans_layer_stop();
		return NULL;
	}
    
    // reset hello pointer.
    s_hello_ptr = NULL;

    // reset hifi_a78_msg_sync pointer.
    s_hifi_a78_msg_sync_ptr = NULL;

    // register CMDs.
    ret = ipc_trans_layer_register_method(s_handle, CMD_METHOD_HELLO);
    if (ret < 0)
	{
		#if (IPC_TRANS_LAYER_SES_MODE == 2 && SESSION_COUNT > 1)
        (void)ipc_trans_layer_unregister_method(s_handle);
		#endif
        (void)ipc_trans_layer_destory_handle(s_handle);
		(void)ipc_trans_layer_stop();
		return NULL;
	}
    ret = ipc_trans_layer_register_method(s_handle, CMD_METHOD_HIFI_A78_MSG_SYNC);
    if (ret < 0)
	{
		#if (IPC_TRANS_LAYER_SES_MODE == 2 && SESSION_COUNT > 1)
        (void)ipc_trans_layer_unregister_method(s_handle);
		#endif
        (void)ipc_trans_layer_destory_handle(s_handle);
		(void)ipc_trans_layer_stop();
		return NULL;
	}
    ret = ipc_trans_layer_register_method(s_handle, CMD_METHOD_SUB_HEARTBEAT);
    if (ret < 0)
	{
		#if (IPC_TRANS_LAYER_SES_MODE == 2 && SESSION_COUNT > 1)
        (void)ipc_trans_layer_unregister_method(s_handle);
		#endif
        (void)ipc_trans_layer_destory_handle(s_handle);
		(void)ipc_trans_layer_stop();
		return NULL;
	}

    ret = ipc_trans_layer_register_method(s_handle, CMD_METHOD_UNSUB_HEARTBEAT);
    if (ret < 0)
	{
        (void)ipc_trans_layer_unregister_method(s_handle);
        (void)ipc_trans_layer_destory_handle(s_handle);
		(void)ipc_trans_layer_stop();
		return NULL;
	}

    
    s_server.version = get_ipc_inf_version;
    s_server.register_hello = register_hello;
	s_server.register_hifi_a78_msg_sync = register_hifi_a78_msg_sync;
	
    s_server.heartbeat = heartbeat;
    s_server.register_heartbeat_subcribed = register_heartbeat_subcribed;
    s_server.register_heartbeat_unsubcribed = register_heartbeat_unsubcribed;
    
    s_server.receive_message = receive_message;
    s_server.dispatch_message = dispatch_message;
    return &s_server;
}

// destory client
int32_t test_server_destroy(void)
{
    int32_t ret = ipc_trans_layer_unregister_method(s_handle);
	if (ret < 0)
		return ret;
    ret = ipc_trans_layer_destory_handle(s_handle);
    if (ret < 0)
        return ret;
    ret = ipc_trans_layer_stop();
    if (ret < 0)
        return ret;

    s_server.version = NULL;
    s_server.register_hello = NULL;
	s_server.register_hifi_a78_msg_sync = NULL;
	
    s_server.heartbeat = NULL;
    s_server.register_heartbeat_subcribed = NULL;
    s_server.register_heartbeat_unsubcribed = NULL;
    
    s_server.receive_message = NULL;
    s_server.dispatch_message = NULL;
    
    // reset hello pointer.
    s_hello_ptr = NULL;

    // reset hifi_a78_msg_sync pointer.
    s_hifi_a78_msg_sync_ptr = NULL;

    return ret;
}
