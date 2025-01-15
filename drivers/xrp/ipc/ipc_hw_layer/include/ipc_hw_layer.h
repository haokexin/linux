#ifndef _IPC_HW_LAYER_H
#define _IPC_HW_LAYER_H

#include "ipc_hw_common.h"

// this is the header file for transferring layer

// access MsgBxEnd device steps
// 1. prepare ipc_hw environment, transferring config to hw layer.
// 2. add device (device type: msgbx, interrupt+shared buffer, semaphore)
// 3. open the device (reserve some time for hw layer finish hardware initiation)
// 4. in order to decoupling, we choose to callback notification and then user request data actively
// 5. write and read MsgBx TX or MsgFilter
// 6. close the device
// 7. clear the ipc_hw environment

// this is the abstraction compatible layer for MsgBxEnd processing.
// As for MsgBxEnd device, we should access the fifo, interrupt,

// use for transferring layer

/**
 * ipc_hw_layer_init: ipc hardware layer init.
 * @ipc_param: in parameter, define some hardware config to hw registers
 * @hw_info: out parameter, get hw info from msgbx registers
 * @return: 0 - succeeded, non-zero for failures.
 */
int32_t ipc_hw_layer_init(const struct ipc_init_params *ipc_param, struct msgbx_hw_info *hw_info);
int32_t ipc_hw_layer_deinit(void);
int32_t ipc_hw_layer_get_info(struct msgbx_hw_info *hw_info);

int32_t ipc_hw_layer_start(void);
int32_t ipc_hw_layer_stop(void);

// filtering rule set
int32_t ipc_hw_layer_flt_init(const struct msgbx_flt_cfg *cfg);
int32_t ipc_hw_layer_flt_rule_set(const uint8_t fid, struct msgbx_flt_rule_cfg *rule);
int32_t ipc_hw_layer_flt_get_info(const uint8_t fid, struct msgbx_flt_rule_cfg *info);
int32_t ipc_hw_layer_flt_rule_clr(const uint8_t fid);

// receive message
typedef int32_t (*recv_ntf)(const uint8_t fid, const rw_msg *msg);
int32_t ipc_hw_layer_recv_ntf_register(recv_ntf recv_func);
int32_t ipc_hw_layer_get_msg(rw_msg *msg, const uint8_t fid);

// send message
int32_t ipc_hw_layer_send_msg(const rw_msg *msg);

// state management
int32_t ipc_hw_layer_sts_mgt_set(void);
int32_t ipc_hw_layer_sts_mgt_clr(void);

int32_t ipc_hw_layer_flt_sts_mgt_set(void);
int32_t ipc_hw_layer_flt_sts_mgt_clr(void);

// state management processing
#ifdef IPC_STATE_MGT_ENABLE
typedef int32_t (*err_msg_callback)(struct msgbx_err_msg *err_msg);
int32_t ipc_hw_layer_err_msg_register(err_msg_callback err_func);

int32_t ipc_hw_layer_err_hdl(uint8_t type, uint8_t id, uint32_t hdl);
#endif

#endif