#ifndef _IPC_TRANS_LAYER_H
#define _IPC_TRANS_LAYER_H

#include "bstipc_cfg.h"
#include "config.h"

// this header file is used by ipc_app_layer

// common api
int32_t ipc_trans_layer_destory_handle(const uint32_t handle);

// only for api calling, if you use server-mode, you don't need to call these two api to start the whole ipc driver
int32_t ipc_trans_layer_start(uint8_t role);
int32_t ipc_trans_layer_stop(void);

// server api only
int32_t ipc_trans_layer_stub_create_handle(const uint8_t fid, const uint8_t sid, uint32_t *handle);
int32_t ipc_trans_layer_stub_get_method_msg(const uint32_t handle, rw_msg *msg);
int32_t ipc_trans_layer_stub_send_reply_msg(const uint32_t handle, rw_msg msg);
int32_t ipc_trans_layer_stub_send_broadcast(const uint32_t handle, rw_msg msg);

#if (IPC_TRANS_LAYER_SES_MODE == 2 && SESSION_COUNT > 1)
int32_t ipc_trans_layer_register_method(const uint32_t handle, const uint8_t cmd);
int32_t ipc_trans_layer_unregister_method(void);
#endif

// client api only
int32_t ipc_trans_layer_proxy_create_handle(const uint8_t fid, const uint8_t sid, uint32_t *handle);
int32_t ipc_trans_layer_proxy_get_reply_msg(const uint32_t handle, rw_msg *msg);
int32_t ipc_trans_layer_proxy_get_broadcast_msg(const uint32_t handle, rw_msg *msg);
int32_t ipc_trans_layer_proxy_send_method(const uint32_t handle, rw_msg msg);

// special for polling-mode
int32_t ipc_trans_layer_get_msg(const uint32_t handle);

// error code
enum ipc_err_code_e {
    RESULT_SUCCESS = 0,
    ERR_TRANS_INIT_FAIL = 1,
    ERR_TRANS_INIT_RECV_REGISTER_FAIL,
    ERR_TRANS_INIT_START_FAIL,
    ERR_TRANS_END_REGISTER_FAIL,
    ERR_CREATE_SES_OUT_RANGE,
    ERR_CREATE_SES_ROLE_INVALID,
    ERR_CREATE_SES_FAIL,
    ERR_SET_FIL_RULE_FAIL,
    ERR_SES_IS_INVALID,
    ERR_PID_IS_INVALID,
    ERR_CID_IS_INVALID,
    ERR_TYP_IS_INVALID,
    ERR_SEND_MSG_FAIL,
    ERR_RECV_MSG_FAIL,
    ERR_SES_CLOSE_FAIL,
};
#endif