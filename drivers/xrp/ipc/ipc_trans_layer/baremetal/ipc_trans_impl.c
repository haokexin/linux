/**
 * @file  ipc_trans_impl.c
 * @brief this file is used as ipc transferring layer implementation. if you need to use call api directly,
 * such as running ipc driver in some baremetal environment, you need fullfil this file.
 * @details feature list
 * 1. stub api implementation
 * 2. proxy api implementation
 * 3. state management process
 */

#include "../src/ipc_trans_runtime.h"
#include "config.h"
#include "ipc_trans_layer.h"

/* this recv_func means that you have received a message interrupt. at here, we use ipc_trans_read_msg to
   get a message from related filter fifo and dispatch it in different sessions.
   IMPORTANT: this function calling is in interrupt handle, you should take interrupt process steps consider
   to avoid mask interrupt in long time.
*/
// deprecated
int32_t recv_func(const uint8_t fid)
{
    int32_t ret;
    ret = ipc_trans_read_msg(fid, 1); // 1 means get single message
    return ret;
}

/* error message process, call ipc_trans_err_hdl to pass fault handle command.
   handle command is TBD
*/
#ifdef IPC_STATE_MGT_ENABLE
int err_func(struct msgbx_err_msg *err_msg)
{
    printf("recv err msg: ");
    printf("err_msg.type = %d, id = %d, msg = %d \n", err_msg->type, err_msg->id, err_msg->msg);
    ipc_trans_err_hdl(err_msg->type, err_msg->id, 0);
    printf("ipc_trans_err_hdl success!!\n");
    return 0;
}
#endif

/**
 * @name start ipc driver
 * @attention both client and server api
 * @details
 * 1. MsgBx hw layer init
 * 2. register receive message callback function
 * 3. set state management configuration, if necessary
 * NOTE: role in here is just a special demo MUST parameter.
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
        printf("ipc_trans_init error\n");

    return ret;
}

/**
 * @name stop ipc driver
 * @attention both client and server api
 * @details
 */
int32_t ipc_trans_layer_stop()
{
    return ipc_trans_deinit();
}

/**
 * @name get available message
 * @attention both client and server api
 * @details
 * 1. enable trans_layer message process
 * 2. query available message handle and type
 * NOTE: type is reserved, in this demo, we do not pass type information to upper layer.
 * @return ret less than 0 means do not have available message, you should still wait or do other process.
 */
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
    printf("ipc_trans_layer_stub_create_handle session id = %u\n", ses_id);
    if (ret < 0)
        return ret;
    *handle = ses_id;
    // todo: init diagnose information array
    return 0;
}

/**
 * @name send a reply message
 * @attention server api only
 */
int32_t ipc_trans_layer_stub_send_reply_msg(const uint32_t handle, rw_msg msg)
{
    int32_t ret = ipc_trans_send_msg(handle, &msg, IPC_MSG_TYPE_REPLY);
    return ret;
}

/**
 * @name send a signal message
 * @attention server api only
 */
int32_t ipc_trans_layer_stub_send_broadcast(const uint32_t handle, rw_msg msg)
{
    int32_t ret = ipc_trans_send_msg(handle, &msg, IPC_MSG_TYPE_BROADCAST);
    return ret;
}

/**
 * @name server register method
 * @attention server api only
 * @details
 * 1. fid, sid check
 * 2. register cmd in method map
 */
#if (IPC_TRANS_LAYER_SES_MODE == 2 && SESSION_COUNT > 1)
int32_t ipc_trans_layer_register_method(const uint32_t handle, const uint8_t cmd)
{
    if (cmd > CMD_MAX_COUNT)
    {
        return -1;
    }

    // todo: add lock
    int32_t ret = ipc_trans_register_method(handle, cmd);
    return ret;
}
#endif

/**
 * @name receive method message
 * @attention server api only
 * @details
 */
int32_t ipc_trans_layer_stub_get_method_msg(const uint32_t handle, rw_msg *msg)
{
    int32_t ret = ipc_trans_get_msg(handle, IPC_MSG_TYPE_METHOD, msg);
    return ret;
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
    printf("ipc_trans_layer_proxy_create_handle session id = %u\n", ses_id);
    if (ret < 0)
        return ret;
    *handle = ses_id;
    // todo: init diagnose information array
    return 0;
}

/**
 * @name receive sinal message
 * @attention client api only
 * @details
 */
int32_t ipc_trans_layer_proxy_get_broadcast_msg(const uint32_t handle, rw_msg *msg)
{
    int32_t ret = ipc_trans_get_msg(handle, IPC_MSG_TYPE_BROADCAST, msg);
    return ret;
}

/**
 * @name send a method message
 * @attention client api only
 */
int32_t ipc_trans_layer_proxy_send_method(const uint32_t handle, rw_msg msg)
{
    int32_t ret = ipc_trans_send_msg(handle, &msg, IPC_MSG_TYPE_METHOD);
    return ret;
}

/**
 * @name receive reply message
 * @attention client api only
 * @details
 */
int32_t ipc_trans_layer_proxy_get_reply_msg(const uint32_t handle, rw_msg *msg)
{
    int32_t ret = ipc_trans_get_msg(handle, IPC_MSG_TYPE_REPLY, msg);
    return ret;
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