/**
 * @file  ipc_trans_runtime.c
 * @brief this file is used as the api to access ipc tranferring layer.
 * this api may be used by ipc_app_layer directly, or related driver developers may implement ipc_trans_impl.c
 * to adapt to different OSs.
 * @note
 * @details feature list
 * 1.
 */

#include "ipc_trans_layer.h"

#include "ipc_trans_runtime.h"
#include "ipc_trans_routing.h"
#include "ipc_trans_cfg.h"
#include "ipc_trans_ses_mgt.h"
#include "ipc_trans_sts_mgt.h"
#include "ipc_trans_utl.h"
#include "config.h"

/********************* local variables ***************************/
static uint8_t g_ipc_pid = 0;
static uint8_t g_ipc_flt_cnt = 0;

/********************* extern global variables *******************/

/**
 * @name  ipc_trans_init
 * @param in :role
 * @param out:
 * @return result of initiating trans layer
 * @details
 * 1. MsgBx hw layer init
 * 2. set state management configuration, if necessary
 */
#ifdef IPC_STATE_MGT_ENABLE
int32_t ipc_trans_init(uint8_t role, err_msg_callback err_func)
#else
int32_t ipc_trans_init(uint8_t role)
#endif
{
    int32_t ret = 0;

    struct ipc_init_params init_params;
    init_params.mbx_device = IPC_HW_MSGBX;
#ifdef IPC_STATE_MGT_ENABLE
    init_params.msgbx_end_mgt_flag = MSG_DEF_FLT_MGT_CONFIG;
#endif

    // note: this is a special config for demo. It will tell hw which is the receiver. In practice, maybe we
    // need not to set this parameter
    if (role == 0) {
        init_params.mbx_type = IPC_ROLE_SERVER_ONLY;
    }
    else {
        init_params.mbx_type = IPC_ROLE_CLIENT_ONLY;
    }

    struct msgbx_hw_info hw_info;

    ret = ipc_hw_layer_init(&init_params, &hw_info);
    if (ret < 0)
        return -ERR_TRANS_INIT_FAIL;

    // update global pid
    g_ipc_pid = hw_info.mbx_end_id;
    printf("g_pid = %d\n", g_ipc_pid);

    g_ipc_flt_cnt = hw_info.mbx_flt_cnt;
    printf("g_ipc_flt_cnt = %d\n", g_ipc_flt_cnt);
    if (g_ipc_flt_cnt < CHANNEL_COUNT) {
        printf("hw filter count is less than sw define, please check your own definition\n");
        return -ERR_TRANS_INIT_FAIL;
    }

    ret = ipc_trans_routing_init();
    if (ret < 0)
        return -ERR_TRANS_INIT_FAIL;

    // session management init
    ret = session_mgt_init();
    if (ret < 0)
        return -ERR_TRANS_INIT_FAIL;

    // cfg init
    ret = flt_cfg_init();
    if (ret < 0)
        return -ERR_TRANS_INIT_FAIL;

    ret = ipc_hw_layer_start();
    if (ret < 0)
        return -ERR_TRANS_INIT_START_FAIL;

    // state management setting
#ifdef IPC_STATE_MGT_ENABLE
    ret = ipc_hw_layer_err_msg_register(err_func);
    if (ret != 0) 
        printf("ipc_hw_layer_err_msg_register error\n");

    // send broadcast to notify node available signal
    status_mgt_start();
#if 0
    ret = ipc_end_register(g_ipc_pid);
    if (ret < 0) {
        printf("ipc end register fail\n");
        return -ERR_TRANS_END_REGISTER_FAIL;
    }
#endif  

#endif 
    return RESULT_SUCCESS;
}

/**
 * @name  ipc_trans_deinit
 * @param in :
 * @param out:
 * @return result of deinit ipc driver
 * @details
 */
int32_t ipc_trans_deinit()
{
    return ipc_hw_layer_deinit();
}

/**
 * @name  ipc_trans_create_session
 * @param in :sid, fid, role, pid
 * @param out:ses_id
 * @return result of creating session
 * @details
 * 1. input param checking
 * 2. register session and allocate receive message queue buffer
 * 3. set MsgBx filter dispatching rules
 */
int32_t ipc_trans_create_session(uint8_t sid, uint8_t fid, uint8_t role, uint32_t *ses_id)
{
    int32_t ret = 0;
    // input param check
    if (sid > MSGBX_SID_MAX || fid > g_ipc_flt_cnt)
        return -ERR_CREATE_SES_OUT_RANGE;

    if (role > IPC_SES_ROLE_MAX)
        return -ERR_CREATE_SES_ROLE_INVALID;

    // create session buffer
    ses_base ses_info = {
        .sid = sid,
        .fid = fid,
        .role = role,
        .pid = g_ipc_pid,
    };
    ret = session_register(ses_info, ses_id);
    if (ret < 0) {
        printf("session register fail, ret = %d\n", ret);
        return -ERR_CREATE_SES_FAIL;
    }

    // check filter rule setting
    ret = set_flt_rules(fid);
    if (ret < 0) {
        printf("set_flt_rules fail, ret = %d\n", ret);
        return -ERR_SET_FIL_RULE_FAIL;
    }
    return RESULT_SUCCESS;
}

/**
 * @name  ipc_trans_send_msg
 * @param in: ses_id, msg, type
 * @return send result
 * @details
 * 1. input parameters checking
 * 2. send message
 */
int32_t ipc_trans_send_msg(uint32_t ses_id, rw_msg *msg, uint8_t type)
{
    int32_t ret = 0;
    // input param check
    if (session_isvalid(ses_id) < 0)
        return -ERR_SES_IS_INVALID;

    // msg content check
    if (msg->header.pid != g_ipc_pid)
        return -ERR_PID_IS_INVALID;

    if (end_is_valid(msg->header.cid) == 0)
        return -ERR_CID_IS_INVALID;
    
    if (type > IPC_MSG_TYPE_MAX)
        return -ERR_TYP_IS_INVALID;

    msg->header.ver = TRANS_LAYER_VERSION;
    msg->header.typ = type;

    // call hw_layer to send
    ret = ipc_hw_layer_send_msg(msg);
    if (ret < 0)
        return -ERR_SEND_MSG_FAIL;
    // return send result
    return RESULT_SUCCESS;
}

/**
 * @name  ipc_trans_get_msg
 * @param in: ses_id, msg_type
 * @param out: msg
 * @return receive message result
 * @details
 * 1. input parameters checking
 * 2. get message from relative session queue
 */
int32_t ipc_trans_get_msg(uint32_t ses_id, enum ipc_msg_type_e msg_typ, rw_msg *msg)
{
    int32_t ret = 0;
    if (session_isvalid(ses_id) < 0)
        return -ERR_SES_IS_INVALID;

    ret = session_msg_out(ses_id, msg_typ, msg);
    if (ret < 0)
        return -ERR_RECV_MSG_FAIL;
    // return recv result
    return RESULT_SUCCESS;
}

/**
 * @name  ipc_trans_close_session
 * @param in: ses_id
 * @param out:
 * @return receive message result
 * @details
 * 1. input parameters checking
 * 2. get message from relative session queue
 */
int32_t ipc_trans_close_session(uint32_t ses_id)
{
    int32_t ret = 0;
    if (session_isvalid(ses_id) < 0)
        return -ERR_SES_IS_INVALID;

    ret = session_destroy(ses_id);
    if (ret < 0)
        return -ERR_SES_CLOSE_FAIL;
    // return recv result
    return RESULT_SUCCESS;
}