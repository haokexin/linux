#ifndef _IPC_TRANS_STS_MGT_H
#define _IPC_TRANS_STS_MGT_H

#include "bstipc_cfg.h"

// state management
enum ipc_end_status_e
{
    IPC_END_STATUS_OFFLINE = 1,
    IPC_END_STATUS_NOT_READY,
    IPC_END_STATUS_READY,
    IPC_END_STATUS_ONLINE,
    IPC_END_STATUS_MAX
};

int32_t status_mgt_start(void);

int32_t ipc_trans_layer_sts_mgt_set(void);
int32_t ipc_trans_layer_sts_mgt_clr(void);

int32_t ipc_trans_layer_flt_sts_mgt_set(void);
int32_t ipc_trans_layer_flt_sts_mgt_clr(void);

#ifdef IPC_STATE_MGT_ENABLE
typedef int (*err_msg_callback)(struct msgbx_err_msg *err_msg);
int32_t ipc_trans_layer_err_msg_register(err_msg_callback err_func);
#endif

int32_t ipc_end_register(const uint8_t pid);
int32_t ipc_end_unregister(const uint8_t pid);
int32_t ipc_end_sts_query(const uint8_t end_id);
int32_t status_mgt_msg_in(rw_msg msg);

#endif