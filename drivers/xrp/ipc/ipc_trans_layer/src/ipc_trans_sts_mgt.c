/**
 * @file  ipc_trans_sts_mgt.c
 * @brief this file is used as ipc state management, and it is configurable.
 * @note you can use IPC_STATE_MGT_ENABLE flag in bstipc_cfg.h to enable or disable these functions.
 * @details feature list
 * 1. receive error message from ipc_trans_routing
 * 2. provide notification to ipc_trans_impl
 * 3. support default or user-defined process mechanism
 * 4. provide inquire api to other modules in ipc_trans_layer
 */
#include "../../ipc_hw_layer/include/ipc_hw_layer.h"
#include "ipc_trans_sts_mgt.h"
#include "../include/config.h"
#include "ipc_trans_buffer.h"
#include "ipc_trans_common.h"
#include "ipc_trans_runtime.h"
#include "ipc_trans_utl.h"

struct ipc_end_info
{
    enum ipc_msg_pid_e end_id;
    enum ipc_end_status_e status;
};

enum trans_layer_msg_cmd_e
{
    IPC_END_IS_ONLINE = 1,
    IPC_END_IS_OFFLINE,
};

const char *end_id_str[MSGBX_END_COUNT_MAX] = {
#define X_MACRO(a, b) #a,
    MACROS_TABLE
#undef X_MACRO
};

static struct ipc_end_info end_map[MSGBX_END_COUNT_MAX] = {0};
static struct buffer trans_msg_queue;

int32_t status_mgt_start(void)
{
    // step 1: init trans layer msg queue
    buffer_Malloc(&trans_msg_queue);

    // init end_map
    return 0;
}

int32_t ipc_end_register(const uint8_t pid)
{
    int32_t ret = 0;
    // pid valid checing

    // prepare a register broadcast to every msgbx node
    rw_msg notify_msg = {
        .header.cid = 0,
        .header.pid = pid,
        .header.len = 0,
        .header.fid = 0,
        .header.sid = 0,
        .header.typ = IPC_MSG_TYPE_PROTOCOL,
        .header.cmd = IPC_END_IS_ONLINE,
        .header.ver = TRANS_LAYER_VERSION,
    };

    ret = ipc_hw_layer_send_msg(&notify_msg);
    if (ret < 0)
        return -1;

    return 0;
}

int32_t ipc_end_unregister(const uint8_t pid)
{
    int32_t ret = 0;
    // pid valid checing

    // prepare a register broadcast to every msgbx node
    rw_msg notify_msg = {
        .header.cid = 0,
        .header.pid = pid,
        .header.len = 0,
        .header.fid = 0,
        .header.sid = 0,
        .header.typ = IPC_MSG_TYPE_PROTOCOL,
        .header.cmd = IPC_END_IS_OFFLINE,
        .header.ver = TRANS_LAYER_VERSION,
    };

    ret = ipc_hw_layer_send_msg(&notify_msg);
    if (ret < 0)
        return -1;

    return 0;
}

int32_t status_mgt_msg_in(rw_msg msg)
{
    uint32_t ret = 0;

    if (!buffer_IsFull(&(trans_msg_queue)))
    {
        ret = buffer_In(&(trans_msg_queue), &msg, sizeof(rw_msg));
        if (ret == 0)
        {
            return -1;
        }
    }
    else
    {
        return -2;
    }
    return ret;
}

void status_msg_process(uint8_t mode)
{
    // in this function, we will empty trans_msg_queue at one time to process all msg
    // so, this function running time maybe a litte bit long
    // mode value at here means single message one time or multi messages one time option

    rw_msg msg = {0};
    if (!buffer_IsEmpty(&(trans_msg_queue)))
    {
        int32_t ret = buffer_Out(&(trans_msg_queue), &msg, sizeof(rw_msg));
        // process logic
        if (msg.header.typ != IPC_MSG_TYPE_PROTOCOL || msg.header.cid != 0)
        {
            // printf("this msg format is not valid\n");
            return;
        }

        switch (msg.header.cmd)
        {
        case IPC_END_IS_ONLINE: {
            uint8_t pid_idx = 0;
            // printf("start process trans layer msg\n");
            if (end_is_valid(msg.header.pid) == 0)
            {
                printf("pid fail\n");
                return;
            }

            ret = query_end_id_idx(msg.header.pid);
            if (ret < 0)
            {
                printf("query end id idx fail\n");
                return;
            }
            end_map[ret].end_id = msg.header.pid;
            end_map[ret].status = IPC_END_STATUS_ONLINE;
            pid_idx = query_end_id_idx(msg.header.pid);
            printf("update %s status to online\n", end_id_str[pid_idx]);
        }
        break;

        case IPC_END_IS_OFFLINE: {
            if (end_is_valid(msg.header.pid) == 0)
            {
                return;
            }

            ret = query_end_id_idx(msg.header.pid);
            if (ret < 0)
            {
                printf("query end id idx fail\n");
                return;
            }
            end_map[ret].end_id = msg.header.pid;
            end_map[ret].status = IPC_END_STATUS_OFFLINE;
        }
        break;

        default:
            printf("trans msg cmd invalid\n");
            break;
        };
    }
    return;
}

int32_t ipc_end_sts_query(const uint8_t end_id)
{
    int32_t ret = 0;
    ret = query_end_id_idx(end_id);
    if (ret < 0)
    {
        printf("this end_id %d is invalid\n", end_id);
        return -1;
    }

    return end_map[ret].status;
}

int32_t ipc_trans_err_hdl(uint8_t type, uint8_t id, uint32_t hdl)
{
    return ipc_hw_layer_err_hdl(type, id, hdl);
}