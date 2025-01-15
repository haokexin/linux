/**
 * @file  ipc_trans_runtime.h
 * @brief this file is used as the api to access ipc tranferring layer.
 * this api may be used by ipc_app_layer directly, or related driver developers may implement ipc_trans_impl.c
 * to adapt to different OSs.
 * @note
 * @details feature list
 * 1.
 */
#ifndef _IPC_TRANS_RUNTIME_H
#define _IPC_TRANS_RUNTIME_H

#include "../../ipc_hw_layer/include/config.h"
#include "../../ipc_hw_layer/include/ipc_hw_common.h"
#include "../../ipc_hw_layer/include/ipc_hw_layer.h"
#include "../include/config.h"
#include "../include/ipc_trans_common.h"
#include "bstipc_cfg.h"

#ifdef IPC_STATE_MGT_ENABLE
int32_t ipc_trans_init(uint8_t role, err_msg_callback err_func);
#else
int32_t ipc_trans_init(uint8_t role);
#endif

int32_t ipc_trans_deinit(void);

int32_t ipc_trans_create_session(uint8_t sid, uint8_t fid, uint8_t role, uint32_t *ses_id);
int32_t ipc_trans_close_session(uint32_t ses_id);

int32_t ipc_trans_read_msg(uint8_t fid, uint8_t mode);

int32_t ipc_trans_send_msg(uint32_t ses_id, rw_msg *msg, uint8_t type);
int32_t ipc_trans_get_msg(uint32_t ses_id, enum ipc_msg_type_e msg_typ, rw_msg *msg);

#if (IPC_TRANS_LAYER_SES_MODE == 2 && SESSION_COUNT > 1)
int32_t ipc_trans_register_method(const uint32_t session_id, const uint8_t cmd);
#endif

int32_t ipc_trans_get_avail_msg_typ(const uint32_t session_id, uint8_t *type);

// status management api
#ifdef IPC_STATE_MGT_ENABLE
void status_msg_process(uint8_t mode);
int32_t ipc_trans_err_hdl(uint8_t type, uint8_t id, uint32_t hdl);
#endif
#endif
