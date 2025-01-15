/**
 * @file  ipc_trans_routing.c
 * @brief file would use api from ipc_hw_layer
 * @details feature list
 * 1. receive message and put it into coreesponding session
 * 2. receive error message from hw_layer(if defined)
 * 3. parse request and transfer to hw_layer to send message (different protocol is reserved)
 */

#include "ipc_trans_layer.h"
#include "ipc_trans_routing.h"
#include "../../ipc_hw_layer/include/ipc_hw_layer.h"
#include "ipc_trans_runtime.h"
#include "ipc_trans_ses_mgt.h"
#include "ipc_trans_sts_mgt.h"

#if (IPC_TRANS_LAYER_SES_MODE == 2 && SESSION_COUNT > 1)
// index = cmd range
// content = SID << 8 | FID
static uint32_t method_register_map[CMD_MAX_COUNT] = {0};
#endif

/**
 * @name  recv_dispatch
 * @param in: fid, msg
 * @return dispatch result
 * @details dispatch receive message into corresponding session message queue
 */
int32_t recv_dispatch(const uint8_t fid, const rw_msg *msg)
{
    int32_t ret = 0;
    // according to dispatching rules, push messages into corresponding session message queue
    if (!msg)
        return -1;

    // message dispatching
    // printf("msg type = %d, sid = %d, fid = %d\n", msg->header.typ, msg->header.sid, msg->header.fid);
    switch (msg->header.typ)
    {
    case IPC_MSG_TYPE_REPLY: 
    case IPC_MSG_TYPE_BROADCAST: {
#if (IPC_TRANS_LAYER_SES_MODE == 1)
        session_msg_in(msg->header.sid, msg);
#else
        uint32_t session_id;
        session_comb(msg->header.sid, msg->header.fid, &session_id);
        printf("msg session id = %u, sid = %d, fid = %d\n", session_id, msg->header.sid, msg->header.fid);
        session_msg_in(session_id, msg);
#endif
    }
    break;
    case IPC_MSG_TYPE_METHOD: {
#if (IPC_TRANS_LAYER_SES_MODE == 1)
        ret = session_msg_in(msg->header.sid, msg);
#else
        // search method_register_map
        uint32_t session_id = method_register_map[msg->header.cmd];
        session_msg_in(session_id, msg);
#endif
    }
    break;
#ifdef IPC_STATE_MGT_ENABLE
    case IPC_MSG_TYPE_PROTOCOL: {
        if (msg->header.cid == 0)
        {
            ret = status_mgt_msg_in(*msg);
            if (ret < 0)
            {
                printf("status mgt msg in fail, ret = %d\n", ret);
            }
        }
    }
    break;
#endif
    default:
        break;
    }
    return ret;
}

/**
 * @name  ipc_trans_read_msg
 * @param sid, mode
 * @return get result
 * @details this function means trans_layer would get message from filter fifo and dispatch
 * message into session by defined mode.
 */
int32_t ipc_trans_read_msg(uint8_t fid, uint8_t mode)
{
    int8_t ret = -1;
    // check mode
    // 0 means get messages until filter becomes empty.
    // 1 means get single message from filter
    rw_msg recv_msg;
    if (mode == 0)
    {
        // means get all flt
        int8_t cnt = 0;
        for (cnt = 0; cnt < CHANNEL_COUNT; cnt++)
        {
            ret = ipc_hw_layer_get_msg(&recv_msg, cnt);
            if (ret == 0)
            {
                // printf("dispatch msg\n");
                recv_dispatch(cnt, &recv_msg);
            }
        }
    }
    else if (mode == 1)
    {
        ret = ipc_hw_layer_get_msg(&recv_msg, fid);
        if (ret < 0)
        {
            // printf("ipc_hw_layer_get_msg err is %d\n", ret);
            return -1;
        }
        recv_dispatch(fid, &recv_msg);
        return 0;
    }
    else
    {
    }
    return ret;
}


#if (IPC_TRANS_LAYER_SES_MODE == 2)
int32_t ipc_trans_register_method(const uint32_t session_id, const uint8_t cmd)
{
    if (session_isvalid(session_id) < 0)
        return -ERR_SES_IS_INVALID;

    if (cmd > CMD_MAX_COUNT)
        return -1;

    if (method_register_map[cmd] > 0)
        return -2;

    method_register_map[cmd] = session_id;
    return 0;
}
#endif


int32_t ipc_trans_routing_init()
{
    int8_t ret = 0;
#if (IPC_RECV_MODE == 1)
    ret = ipc_hw_layer_recv_ntf_register(recv_dispatch);
#endif   
    return ret;    
}