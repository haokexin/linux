/**
 * @file  ipc_hw_impl.h
 * @brief this file is used as ipc hardware layer implementation api definition, you should implement your own ipc driver in follow options.
 * @note
 * @details feature list
 */
#ifndef _IPC_HW_IMPL_H
#define _IPC_HW_IMPL_H

#include "ipc_hw_common.h"

// this is the header file for hardware implementation, the core developer should follow these definition
// to implement related hardware function.
// struct for value definition

// api definition
struct libipc_hw_compat_ops
{
    int32_t (*ipc_hw_init)(const struct ipc_init_params *ipc_param);
    int32_t (*ipc_hw_deinit)(void);

    // msgbx spec init
    int32_t (*ipc_hw_get_info)(struct msgbx_hw_info *hw_info);
    // int32_t (*ipc_hw_set_info)(const struct msgbx_hw_info *init_param);

    // msgbx filtering rule config
    int32_t (*ipc_hw_set_flt_cfg)(const uint8_t flt_id, const struct msgbx_flt_rule_cfg *rule);
    int32_t (*ipc_hw_clr_flt_cfg)(const uint8_t flt_id);
    int32_t (*ipc_hw_get_flt_info)(const uint8_t flt_id, struct msgbx_flt_rule_cfg *info); //debug get rule setting

    // msgbx send msg * recv msg
    int32_t (*ipc_hw_send_msg)(const rw_msg *msg);
    int32_t (*ipc_hw_get_msg)(rw_msg *msg, const int8_t fid);

#ifdef IPC_STATE_MGT_ENABLE
    // msgbx state management
    // flag define see state management enable flag bit
    int32_t (*ipc_hw_sts_mgt_enble)(const uint32_t flag);
    int32_t (*ipc_hw_sts_mgt_disable)(void);

    // msgbx filter state management config
    int32_t (*ipc_hw_flt_mgt_enble)(const uint8_t flt_id, const uint8_t flag);
    int32_t (*ipc_hw_flt_mgt_disable)(const uint8_t flt_id);

    // msgbx get error msg
    int32_t (*ipc_hw_get_err_msg)(const uint8_t flt_id, struct msgbx_err_msg *err_msg);

    // msgbx fault handle
    int32_t (*ipc_hw_err_hdl)(uint8_t type, uint8_t id, uint32_t hdl);
#endif
};

#endif