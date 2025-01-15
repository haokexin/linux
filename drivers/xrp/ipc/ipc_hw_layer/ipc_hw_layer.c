/**
 * @file  ipc_hw_layer.c
 * @brief this file is used as the api implementation in hardware layer
 * this api may be used by ipc_trans_layer directly
 * @note
 * @details feature list
 */

#include "../include/ipc_hw_layer.h"
#include "./msgbx_impl/ipc_hw_impl.h"

/* this is ipc hw_layer msgbx implementation. In this file, which provide 
* API for upper layer and compabitility to different hardware ipc mechanism.
*/
/********************* local variables ***************************/
struct libipc_hw_compat_ops g_hw_ctl_ops;
#ifdef IPC_STATE_MGT_ENABLE
static err_msg_callback err_cb;
#endif
static recv_ntf recv_cb;

/********************* extern global variables *******************/
// NOTE: you should update your option pointer at here
extern struct libipc_hw_compat_ops ipc_hw_shm_ops;

// ipc hw layer local function
int32_t ipc_hw_register_ops(const struct libipc_hw_compat_ops *ops)
{
    if (ops == NULL) 
        return -1;

    g_hw_ctl_ops = *ops;
    return 0; 
}

// ipc hw layer api implementation
int32_t ipc_hw_layer_init(const struct ipc_init_params *ipc_param, struct msgbx_hw_info *hw_info)
{
    // step1: register ops
    int32_t ret = -1;
    ret = ipc_hw_register_ops(&ipc_hw_shm_ops);
    if (ret < 0)
        return -1;

    // step2: get info from hw
    ret = g_hw_ctl_ops.ipc_hw_get_info(hw_info);
    if (ret < 0)
        return -2;

    // step3: init hardware
    ret = g_hw_ctl_ops.ipc_hw_init(ipc_param);
    if (ret < 0)
        return -3;

    // step5: state mgt enable
#ifdef IPC_STATE_MGT_ENABLE
    if (ipc_param->msgbx_end_mgt_flag != 0)
    {
        ret = g_hw_ctl_ops.ipc_hw_sts_mgt_enble(ipc_param->msgbx_end_mgt_flag);
        if (ret < 0)
            return -4;
    }
#endif

    return 0;
}

int32_t ipc_hw_layer_deinit()
{
    return g_hw_ctl_ops.ipc_hw_deinit();
}

int32_t ipc_hw_layer_start()
{
    // step1:
    return 0;
}

// as for message receiving function, we use notify-getting strategy. It means that we would send a notification
// when we receive interrupt from msgbx default filter or dedicated config filter. IPC_trans_layer receive this
// notification, then call get_message actively from related filter fifo until it goes empty.
int32_t ipc_hw_layer_recv_ntf_register(recv_ntf recv_func)
{
    recv_cb = recv_func;
    return 0;
}

int32_t ipc_hw_layer_get_msg(rw_msg *msg, const uint8_t fid)
{
    return g_hw_ctl_ops.ipc_hw_get_msg(msg, fid);
}

int32_t ipc_hw_layer_send_msg(const rw_msg *msg)
{
    // remote end state check
    // self tx fifo state check
    return g_hw_ctl_ops.ipc_hw_send_msg(msg);
}

int32_t ipc_hw_recv_msg_notify(const uint8_t fid, const rw_msg *msg)
{
    if (recv_cb != NULL)
    {
        ((recv_ntf)recv_cb)(fid, msg);
    }
    return 0;
}

#ifdef IPC_STATE_MGT_ENABLE
int32_t ipc_hw_err_msg_notify(const uint8_t fid)
{
    // get err msg and call function
    struct msgbx_err_msg err_msg;
    g_hw_ctl_ops.ipc_hw_get_err_msg(fid, &err_msg);
    if (err_cb != NULL)
    {
        ((err_msg_callback)err_cb)(&err_msg);
    }

    return 0;
}
#endif

// filtering rule processing
int32_t ipc_hw_layer_flt_init(const struct msgbx_flt_cfg *cfg)
{
    int32_t ret = 0;
#ifdef IPC_FLT_MGT_ENABLE
    // filter state management enable
    ret = g_hw_ctl_ops.ipc_hw_flt_mgt_enble(cfg->flt_id, cfg->mbx_flt_mgt_flag);
#endif
    return ret;
}

int32_t ipc_hw_layer_flt_rule_set(const uint8_t fid, struct msgbx_flt_rule_cfg *rule)
{

    // set rule to filter
    return g_hw_ctl_ops.ipc_hw_set_flt_cfg(fid, rule);
}

int32_t ipc_hw_layer_flt_get_info(const uint8_t fid, struct msgbx_flt_rule_cfg *info)
{
    return g_hw_ctl_ops.ipc_hw_get_flt_info(fid, info);
}

#ifdef IPC_STATE_MGT_ENABLE
int32_t ipc_hw_layer_flt_rule_clr(const uint8_t fid)
{
    return g_hw_ctl_ops.ipc_hw_flt_mgt_disable(fid);
}

int32_t ipc_hw_layer_err_msg_register(err_msg_callback err_func)
{
    err_cb = err_func;
    return 0;
}

int32_t ipc_hw_layer_err_hdl(uint8_t type, uint8_t id, uint32_t hdl)
{
    return g_hw_ctl_ops.ipc_hw_err_hdl(type, id, hdl);
}
#endif
