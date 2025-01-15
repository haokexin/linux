
/* © Copyright  Black Sesame Technologies (Chengdu) Co., Ltd. 2021. All rights reserved. 
 * This file contains proprietary information that is the sole intellectual property of 
 * Black Sesame Technologies (Chengdu) Co., Ltd. No part of this material or its 
 * documentation may be reproduced, distributed, transmitted, displayed or published in
 * any manner without the written permission of Black Sesame Technologies (Chengdu) Co.,
 * Ltd. Anyone who infringes on the copyright of Black Sesame Technologies (Chengdu) Co.,
 * Ltd. shall be held accountable by Black Sesame Technologies (Chengdu) Co., Ltd..
 */


#include "video_server.h"
#include <bst/ipc_app_common.h>
#include <bst/ipc_app_svr_utils.h>
#include <bst/ipc_trans_common.h>
#include <bst/ipc_trans_layer.h>
#include <bst/ipc_serdes.h>
#include <bst/bstipc_cfg.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/slab.h>

// macro definitions
#define PID CPU_0
#define CPUID PID
#define CID ISPCV_4
#define FID DEF
#define SID 1U
#define MAX_METHOD_NUM 10U
#define MAX_BROADCAST_NUM 10U

#define CMD_METHOD_ISP2ARM 1U

#define CMD_METHOD_SUB_ARM2ISP 10U
#define CMD_METHOD_UNSUB_ARM2ISP 11U
#define CMD_BROADCAST_ARM2ISP 1U

// local variables
static ipc_inf_version_t s_version = {.major = 1, .minor = 0};
static uint8_t s_handle = 0U;
static video_server s_server = {0};
static volatile uint8_t s_token = 0;
static int8_t s_recv_buffer[IPC_MAX_DATA_SIZE] = {0};

static video_isp2arm_t s_isp2arm_ptr = NULL;

static video_broadcast_sub_t s_arm2isp_sub_ptr = NULL;
static video_broadcast_sub_t s_arm2isp_unsub_ptr = NULL;
static broadcast_registry s_arm2isp_registry = {0};
static void *isp2arm_ext = NULL;
static void *arm2isp_sub_ext = NULL;
static void *arm2isp_unsub_ext = NULL;

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

static int32_t register_isp2arm(video_isp2arm_t func, void *ext)
{
    s_isp2arm_ptr = func;
    isp2arm_ext = ext;
    return 0;
}

static int32_t call_isp2arm(serdes_t *des, serdes_t *ser)
{
    if (!des || !ser || !s_isp2arm_ptr)
        return -1;

    int32_t ret = 0;
    uint32_t length = 0;

    uint32_t msgAddr = 0;
	ret = ipc_des_get(des, (uint8_t *)&msgAddr, sizeof(uint32_t));
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
	
    video_ErrorEnum_t err = {0};
	uint32_t result = 0;
	
    ext_info_t info = {0};
	info.uuid = ipc_msg_get_uuid(des->header);
    (*s_isp2arm_ptr)(msgAddr, &result, &err, &info, isp2arm_ext);

    if (ret >= 0)
	{
	    int32_t err_val = err;
	    ret = ipc_ser_put(ser, (uint8_t*)&err_val, sizeof(int32_t));
	}
	if (ret >= 0)
	{
	    ret = ipc_ser_put(ser, (uint8_t*)&result, sizeof(uint32_t));
	}
	
    if (ret >= 0)
        return 0;
    else
        return -1;
}


// broadcast

static int32_t register_arm2isp_subcribed(video_broadcast_sub_t func, void *ext)
{
    s_arm2isp_sub_ptr = func;
    arm2isp_sub_ext = ext;
    return 0;
}

static int32_t register_arm2isp_unsubcribed(video_broadcast_sub_t func, void *ext)
{
    s_arm2isp_unsub_ptr = func;
    arm2isp_unsub_ext = ext;
    return 0;
}

static int32_t arm2isp(uint32_t msgAddr)
{
    int32_t ret = 0;
    int32_t index = 0;
    serdes_t serdes = {0};
    serdes_t *ser = &serdes;
    rw_msg_header_t header = {0};
    header.pid = PID;
    header.cmd = CMD_BROADCAST_ARM2ISP;
    header.typ = IPC_MSG_TYPE_BROADCAST;

    ret = ipc_ser_init(ser);
    if (ret >= 0)
	{
	    ret = ipc_ser_put(ser, (uint8_t*)&msgAddr, sizeof(uint32_t));
	}
	
    if (ret < 0)
	    return -1;

    broadcast_reg_entry *entry = s_arm2isp_registry.entries;
    for (index = s_arm2isp_registry.start, entry += s_arm2isp_registry.start; index < s_arm2isp_registry.end;
         ++index, ++entry)
    {
        if (entry->pid != 0)
        {
            uint32_t i = 0;
            header.cid = entry->pid;
            header.fid = entry->fid;
            header.sid = entry->sid;
            header.tok = s_token;
            ipc_ser_set_header(ser, header);
            ipc_ser_finish(ser);
            int32_t send_ret = ipc_trans_layer_stub_send_broadcast(PID, s_handle, ser);
            if (send_ret < 0)
            {
                printf("send broadcast fail %d.\n", send_ret);
                break;
            }
            else
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
    return ipc_trans_layer_query_info(PID, s_handle);
}

// dispatch messages
static int32_t dispatch_message(void)
{
    int32_t ret = 0;
    static serdes_t deserializer = {0};
    static serdes_t serializer = {0};

    while (ipc_trans_layer_stub_get_method_msg(PID, s_handle, &deserializer) >= 0)
    {
        bool need_reply = true;
        ret = ipc_des_validate_msg(&deserializer);
        if (ret < 0)
            continue;
        // initialize serializer.
        // it cannot fail, as &serializer won't be NULL.
        (void)ipc_ser_init(&serializer);
        // process message.
        switch (deserializer.header.cmd)
        {
        case CMD_METHOD_ISP2ARM:
		    ret = call_isp2arm(&deserializer, &serializer);
		    if (ret < 0)
		    {
		        printf("call_isp2arm failed.\n");
		        (void)ipc_des_init(&deserializer);
		        (void)ipc_ser_init(&serializer);
		        ret = -1;
		        ret = ipc_ser_put(&serializer, (uint8_t *)&ret, sizeof(ret));
		    }
		    break;
		
        case CMD_METHOD_SUB_ARM2ISP:
		    if (s_arm2isp_sub_ptr)
		        (*s_arm2isp_sub_ptr)((uint8_t)deserializer.header.pid, (uint8_t)deserializer.header.fid, 
		                                (uint8_t)deserializer.header.sid, arm2isp_sub_ext);
		    ret = add_registration(&s_arm2isp_registry, (uint8_t)deserializer.header.pid,
		                            (uint8_t)deserializer.header.fid, (uint8_t)deserializer.header.sid);
		    ret = ipc_ser_put(&serializer, (uint8_t *)&ret, sizeof(ret));
		    break;
		case CMD_METHOD_UNSUB_ARM2ISP:
		    if (s_arm2isp_unsub_ptr)
		        (*s_arm2isp_unsub_ptr)((uint8_t)deserializer.header.pid,
		                                (uint8_t)deserializer.header.fid, (uint8_t)deserializer.header.sid, arm2isp_unsub_ext);
		    ret = remove_registration(&s_arm2isp_registry, (uint8_t)deserializer.header.pid,
		                                (uint8_t)deserializer.header.fid, (uint8_t)deserializer.header.sid);
		    ret = ipc_ser_put(&serializer, (uint8_t *)&ret, sizeof(ret));
		    break;
		
        default:
            ret = -1;
            ret = ipc_ser_put(&serializer, (uint8_t *)&ret, sizeof(ret));
            break;
        }

        if (need_reply)
		{
            serializer.header = deserializer.header;
            serializer.header.cid = deserializer.header.pid;
            serializer.header.pid = PID;
            serializer.header.typ = IPC_MSG_TYPE_REPLY;
            if (ret >= 0)
                ret = ipc_ser_finish(&serializer);
            if (ret >= 0)
                ret = ipc_trans_layer_stub_send_reply_msg(PID, s_handle, &serializer);
            if (ret < 0)
            {
                printf("send reply fail %d.\n", ret);
                break;
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

// initialize server
video_server *video_server_init(void)
{
    int32_t ret = 0;
    // start trans layer.
    // int32_t ret = ipc_trans_layer_start(PID, 0);
    // if (ret < 0)
    //     return NULL;

    // create server handle.
    ret = ipc_trans_layer_stub_create_handle(CPUID, FID, SID, CID, &s_handle);
    if (ret < 0)
	{
		//(void)ipc_trans_layer_stop(pid);
		return NULL;
	}
    
    // reset isp2arm pointer.
    s_isp2arm_ptr = NULL;

    // register CMDs.
    ret = ipc_trans_layer_register_method(PID, s_handle, CMD_METHOD_ISP2ARM);
    if (ret < 0)
	{
        (void)ipc_trans_layer_unregister_method(PID, s_handle);
        (void)ipc_trans_layer_destroy_handle(PID, s_handle);
		//(void)ipc_trans_layer_stop(pid);
		return NULL;
	}
    ret = ipc_trans_layer_register_method(PID, s_handle, CMD_METHOD_SUB_ARM2ISP);
    if (ret < 0)
	{
        (void)ipc_trans_layer_unregister_method(PID, s_handle);
        (void)ipc_trans_layer_destroy_handle(PID, s_handle);
		//(void)ipc_trans_layer_stop(pid);
		return NULL;
	}

    ret = ipc_trans_layer_register_method(PID, s_handle, CMD_METHOD_UNSUB_ARM2ISP);
    if (ret < 0)
	{
        (void)ipc_trans_layer_unregister_method(PID, s_handle);
        (void)ipc_trans_layer_destroy_handle(PID, s_handle);
		//(void)ipc_trans_layer_stop(pid);
		return NULL;
	}

    
    s_server.version = get_ipc_inf_version;
    s_server.register_isp2arm = register_isp2arm;
	
    s_server.arm2isp = arm2isp;
    s_server.register_arm2isp_subcribed = register_arm2isp_subcribed;
    s_server.register_arm2isp_unsubcribed = register_arm2isp_unsubcribed;
    
    s_server.start = start;
    s_server.stop = stop;
    return &s_server;
}

// destroy client
int32_t video_server_destroy(void)
{
    int32_t ret = ipc_trans_layer_unregister_method(PID, s_handle);
	if (ret < 0)
		return ret;
    ret = ipc_trans_layer_destroy_handle(PID, s_handle);
    if (ret < 0)
        return ret;
    // ret = ipc_trans_layer_stop(PID);
    // if (ret < 0)
    //     return ret;

    s_server.version = NULL;
    s_server.register_isp2arm = NULL;
	
    s_server.arm2isp = NULL;
    s_server.register_arm2isp_subcribed = NULL;
    s_server.register_arm2isp_unsubcribed = NULL;
    
    s_server.start = NULL;
    s_server.stop = NULL;
    
    // reset isp2arm pointer.
    s_isp2arm_ptr = NULL;

    return ret;
}
