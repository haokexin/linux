/**
 * @file  ipc_trans_impl.c
 * @brief this file is used as ipc transferring layer implementation. if you need to adapt to some
 * os has ipc(inter-process communication), you need fullfil this file.
 * for example, in Linux os, you may use syscall or readl/writel; while in QNX, you may use msgReceive()
 * to implement a central ipc server to process multiple thread using bstipc.
 * @details feature list
 * 1. stub api implementation
 * 2. proxy api implementation
 * 3. state management process
 */
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/slab.h>

#include "../src/ipc_trans_runtime.h"
//#include "config.h"
//#include "ipc_trans_layer.h"
#include "ipc_common.h"


static uint32_t g_pid = 0;

/**
 * @name receive message process
 * @details
 * 1. start a thread or task to wait recv_notification from hw layer
 * 2. when notification arrive, cancel suspending state
 * 3. call api to get messages from relative filter fifo
 *    mode refers to get single message or get messages until fifo is empty
 *    in trans_layer, message will be dispatched in different session message queue
 * 4. return result
 *
 * below is a demo
 */

int rcv_flg = 0;
uint8_t g_fid;
struct task_struct *tid;

// deprecated
int32_t recv_func(const uint8_t fid)
{
    return 0;
}

// deprecated
int ipc_trans_impl_recv_msg(void *arg)
{
	int ret = -1;
    while (1)
    {
        // wait for notification, you may use completion or other ways to wait
        // as for demo, we use a single value to indicate if there is any message we
        // receive
		ret = ipc_trans_read_msg(0, 0); // 1 means get single message
        if (ret == 0)
        {
            // add receive message process at here
            printk("recv msg \n"); 
        }
        else
        {
            msleep(100);
        }
    }
    return 0;
}

#ifdef IPC_STATE_MGT_ENABLE
int err_func(struct msgbx_err_msg *err_msg)
{
    IPC_LOG_ERR("recv err msg: ");
    IPC_LOG_ERR("err_msg.type = %d, id = %d, msg = %d \n", err_msg->type, err_msg->id, err_msg->msg);
    return 0;
}
#endif

/**
 * @name start ipc driver
 * @details
 * 1. MsgBx hw layer init
 * 2. register receive message callback function
 * 3. start a thread or task to wait recv_notification from hw layer
 * 4. set state management configuration, if necessary
 *
 */
int32_t ipc_trans_layer_start(uint8_t role)
{
    int32_t ret;
    
#ifdef IPC_STATE_MGT_ENABLE
    ret = ipc_trans_init(role, err_func);
#else
    ret = ipc_trans_init(role);
#endif

    if (ret != 0)
        IPC_LOG_INFO("ipc_trans_init error\n");
#if IPC_RECV_MODE == 2   
    // start a thread or task to wait receive notification
    tid = kthread_create(&ipc_trans_impl_recv_msg, NULL, "msgbox_rece");

	if (IS_ERR(tid)) {
        IPC_LOG_INFO("create ipc_trans_impl_recv_msg fail\n");
        return -1;
	}
#endif 

    return 0;
} 

int32_t ipc_trans_layer_get_msg(const uint32_t handle)
{
    int32_t ret;
    uint8_t type;
#ifdef IPC_STATE_MGT_ENABLE
    status_msg_process(0);
#endif
    ret = ipc_trans_get_avail_msg_typ(handle, &type);
    return ret;
}

/**
 * @name send a reply message
 * @attention server api only
 * @details
 * 1. alloc buffer
 * 2. memcpy message
 * 3. transfer message pointer to hw
 * 4. check result and free buffer
 */
int32_t ipc_trans_layer_stub_send_reply_msg(const uint32_t handle, rw_msg msg)
{
    int32_t ret;

    rw_msg *msg_ptr = kmalloc(sizeof(rw_msg), GFP_KERNEL);
    memcpy(msg_ptr, &msg, sizeof(rw_msg));

    ret = ipc_trans_send_msg(handle, msg_ptr, IPC_MSG_TYPE_REPLY);
    if (ret < 0)
    {
        kfree(msg_ptr);
        return ret;
    }
    kfree(msg_ptr);
    return 0;
}

/**
 * @name send a signal message
 * @attention server api only
 * @details
 * 1. alloc buffer
 * 2. memcpy message
 * 3. transfer message pointer to hw
 * 4. check result and free buffer
 */
int32_t ipc_trans_layer_stub_send_broadcast(const uint32_t handle, rw_msg msg)
{
    int32_t ret;

    rw_msg *msg_ptr = kmalloc(sizeof(rw_msg), GFP_KERNEL);
    memcpy(msg_ptr, &msg, sizeof(rw_msg));

    ret = ipc_trans_send_msg(handle, msg_ptr, IPC_MSG_TYPE_BROADCAST);
    if (ret < 0)
    {
        kfree(msg_ptr);
        return ret;
    }
    kfree(msg_ptr);
    return 0;
}

/**
 * @name send a method message
 * @attention client api only
 * @details
 * 1. alloc buffer
 * 2. memcpy message
 * 3. transfer message pointer to hw
 * 4. check result and free buffer
 */
int32_t ipc_trans_layer_proxy_send_method(const uint32_t handle, rw_msg msg)
{
    int32_t ret;

    rw_msg *msg_ptr = kmalloc(sizeof(rw_msg), GFP_KERNEL);
    memcpy(msg_ptr, &msg, sizeof(rw_msg));

    ret = ipc_trans_send_msg(handle, msg_ptr, IPC_MSG_TYPE_METHOD);
    if (ret < 0)
    {
        kfree(msg_ptr);
        return ret;
    }
    kfree(msg_ptr);
    return 0;
}

/**
 * @name session init
 * @attention server api only
 * @details
 * 1. fid, sid check
 * 2. register session
 * 3. return session id
 */
int32_t ipc_trans_layer_stub_create_handle(const uint8_t fid, const uint8_t sid, uint32_t *handle)
{
    uint32_t ses_id;
    int32_t ret;
    ret = ipc_trans_create_session(sid, fid, IPC_SES_ROLE_SERVER, &ses_id);
    IPC_LOG_INFO("ipc_trans_layer_stub_create_handle session id = %u\n", ses_id);
    if (ret < 0)
        return ret;
    *handle = ses_id;
    // todo: init diagnose information array
    return 0;
}

/**
 * @name server register method
 * @attention server api only
 * @details
 * 1. fid, sid check
 * 2. register session
 * 3. return session id
 */
#if (IPC_TRANS_LAYER_SES_MODE == 2 && SESSION_COUNT > 1)
int32_t ipc_trans_layer_register_method(const uint32_t handle, const uint8_t cmd)
{
    if (cmd > CMD_MAX_COUNT)
    {
        return -2;
    }

    int32_t ret;

    // todo: add lock
    ret = ipc_trans_register_method(handle, cmd);
    if (ret < 0)
    {
        return ret;
    }

    return 0;
}
#endif

/**
 * @name receive method message
 * @attention server api only
 * @details
 * 1. fid, sid check
 * 2. register session
 * 3. return session id
 */
int32_t ipc_trans_layer_stub_get_method_msg(const uint32_t handle, rw_msg *msg)
{
    int32_t ret;
    ret = ipc_trans_get_msg(handle, IPC_MSG_TYPE_METHOD, msg);
    if (ret < 0)
    {
        return ret;
    }

    return 0;
}

/**
 * @name receive sinal message
 * @attention client api only
 * @details
 * 1. fid, sid check
 * 2. register session
 * 3. return session id
 */
int32_t ipc_trans_layer_proxy_get_broadcast_msg(const uint32_t handle, rw_msg *msg)
{
    int32_t ret;
    ret = ipc_trans_get_msg(handle, IPC_MSG_TYPE_BROADCAST, msg);
    if (ret < 0)
    {
        return ret;
    }

    return 0;
}

/**
 * @name receive reply message
 * @attention client api only
 * @details
 * 1. fid, sid check
 * 2. register session
 * 3. return session id
 */
int32_t ipc_trans_layer_proxy_get_reply_msg(const uint32_t handle, rw_msg *msg)
{
    int32_t ret;
    ret = ipc_trans_get_msg(handle, IPC_MSG_TYPE_REPLY, msg);
    if (ret < 0)
    {
        return ret;
    }

    return 0;
}

/**
 * @name client session init
 * @attention client api only
 * @details
 * 1. fid, sid check
 * 2. register session
 * 3. return session id
 */
int32_t ipc_trans_layer_proxy_create_handle(const uint8_t fid, const uint8_t sid, uint32_t *handle)
{
    uint32_t ses_id;
    int32_t ret;
    ret = ipc_trans_create_session(sid, fid, IPC_SES_ROLE_CLIENT, &ses_id);
    IPC_LOG_INFO("ipc_trans_layer_proxy_create_handle session id = %u\n", ses_id);
    if (ret < 0)
        return ret;
    *handle = ses_id;
    // todo: init diagnose information array
    return 0;
}

/**
 * @name handle destroy
 * @attention both client and server api
 * @details
 */
int32_t ipc_trans_layer_destory_handle(const uint32_t handle)
{
    uint32_t ret;
    ret = ipc_trans_close_session(handle);
    return ret;
}

/**
 * @name stop ipc driver
 * @attention both client and server api
 * @details
 */
int32_t ipc_trans_layer_stop(void)
{
    return ipc_trans_deinit();
}