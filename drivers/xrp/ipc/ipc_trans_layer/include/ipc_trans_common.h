#ifndef _IPC_TRANS_COMMON_H
#define _IPC_TRANS_COMMON_H

#include "../../include/bstipc_cfg.h"

// this header file define common structure of ipc transferring layer
enum ipc_msg_type_e
{
    IPC_MSG_TYPE_INVALID = 0,
    IPC_MSG_TYPE_METHOD,
    IPC_MSG_TYPE_REPLY,
    IPC_MSG_TYPE_BROADCAST,
    IPC_MSG_TYPE_PROTOCOL,
    IPC_MSG_TYPE_MAX
};

enum ipc_ses_role_e
{
    IPC_SES_ROLE_INVALID = 0,
    IPC_SES_ROLE_CLIENT = 1,
    IPC_SES_ROLE_SERVER = 2,
    IPC_SES_ROLE_MAX
};

#define MSGBX_END_COUNT_MAX 35
#define CMD_MAX_COUNT 255                  // ro
#define MSGBX_SID_MAX 16

#define TRANS_LAYER_VERSION 1

// 1 means interrupt; 2 means polling
#define TRANS_LAYER_RECV_USE_INT

#ifdef TRANS_LAYER_RECV_USE_INT 
#define IPC_RECV_MODE 1
#else
#define IPC_RECV_MODE 2
#endif

#endif