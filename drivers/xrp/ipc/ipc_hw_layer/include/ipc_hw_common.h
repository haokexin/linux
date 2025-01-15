#ifndef _IPC_HW_COMMON_H
#define _IPC_HW_COMMON_H

#include "bstipc_cfg.h"
#include "config.h"

// msgbx end init  ------------------------------------------------------------------------
// safety management enable flag bit
#define MSGBX_ECC_RX_MULTIP_EN_BIT 0x08
#define MSGBX_ECC_RX_DETECT_EN_BIT 0x04
#define MSGBX_PARITY_HWDATA_EN_BIT 0x02
#define MSGBX_PARITY_HADDR_EN_BIT 0x01

enum ipc_hw_device_type_e
{
    IPC_HW_MSGBX = 0,
    IPC_HW_INT = 1,
    IPC_HW_SEM = 2,
    IPC_HW_MAX
};

enum ipc_type_e
{
    IPC_ROLE_CLIENT_ONLY = 1,
    IPC_ROLE_SERVER_ONLY = 2,
    IPC_ROLE_HYBRID = 3,
    IPC_ROLE_TYPE_MAX
};

/**
 * ipc_init_params: ipc hardware device static init param
 * @mbx_device:
 * @mbx_role:
 * @msgbx_end_mgt_flag:
 */
struct ipc_init_params
{
    // static device setting
    enum ipc_hw_device_type_e mbx_device;
    enum ipc_type_e mbx_type;

    // special device setting
    uint64_t spec_cfg;

    // optional device setting
    // state management setting
#ifdef IPC_STATE_MGT_ENABLE
    uint8_t msgbx_end_mgt_flag;
#endif
};

struct msgbx_hw_info
{
    uint8_t mbx_version : 4;      // RO, version checking
    uint8_t mbx_flt_cnt : 4;      // RO, flt checking
    uint8_t mbx_end_id : 8;       // RO, for msg header
    uint8_t mbx_txfifo_depth : 8; // RO
    uint8_t mbx_rxfifo_depth : 8; // RO
    uint8_t is_64_bit;            // RO, reserved
};

// msgbx filter management  ------------------------------------------------------------------------
// msgbx filter config
struct msgbx_flt_cfg
{
    uint8_t flt_id;

#ifdef IPC_FLT_MGT_ENABLE
    // msgbx filter general config, use bit offset refer to different config
    uint8_t mbx_flt_mgt_flag;
#endif
};

enum ipc_flt_rule_type_e
{
    IPC_FLT_RULE_PID = 1,
    IPC_FLT_RULE_LEN = 2,
    IPC_FLT_RULE_USER = 3,
    IPC_FLT_RULE_MAX,
};

struct msgbx_flt_rule_pid
{
    uint16_t mbx_rx_pid_end : 8;       // RW
    uint16_t mbx_rx_pid_st : 8;        // RW
    uint8_t mbx_pid_flt_invert : 1;    // RW
};

struct msgbx_flt_rule_len
{
    uint8_t mbx_len_flt_invert : 1;
    uint8_t mbx_rx_len_end : 4;
    uint8_t mbx_rx_len_st : 4;
};

enum ipc_flt_rule_location_e
{
    IPC_FLT_RULE_HEADER = 1,
    IPC_FLT_RULE_PAY1 = 2,
    IPC_FLT_RULE_PAY2 = 3,
    IPC_FLT_RULE_PAY3 = 4,
    IPC_FLT_RULE_PAY4 = 5,
    IPC_FLT_RULE_PAY_MAX,
};

struct msgbx_flt_rule_user
{
    enum ipc_flt_rule_location_e cfg_loc; 
    uint32_t msgh_combi_lh_comp;  // default is enable
    uint32_t msgh_flilter_invert; // default is disable
    uint64_t tx_reserved_filter_mask : 64; 
    uint64_t rx_res_min : 64;
    uint64_t rx_res_max : 64;
};

struct msgbx_flt_rule_cfg
{
    enum ipc_flt_rule_type_e cfg_type;  
    union 
    {
        struct msgbx_flt_rule_pid rule_pid;
        struct msgbx_flt_rule_len rule_len;
        struct msgbx_flt_rule_user rule_user;
    };
};

// msgbx state management ------------------------------------------------------------------------
// state management enable flag bit
#define MSGBX_TX_OVERFLOW_EN_BIT 0x04       // only available for msgbx default filter
#define MSGBX_RX_OVERFLOW_EN_BIT 0x02       
#define MSGBX_RX_UNDERFLOW_EN_BIT 0x01

enum ipc_err_msg_typ_e
{
    IPC_MSB_END_ERR_TYP = 1,
    IPC_MSB_FIL_ERR_TYP = 2,    // default filter err & config filter err
    IPC_ERR_TYP_MAX
};

struct msgbx_flt_info
{
    uint32_t mbx_rxfifo_end_addr : 10; // RW, this info is got from hw impl
    uint32_t mbx_rxfifo_st_addr : 10;  // RW
};

struct msgbx_flt_device
{
    struct msgbx_flt_cfg cfg;
    struct msgbx_flt_info info;
    struct msgbx_flt_rule_cfg rule;
};

// msgbx device
struct msgbx_hw_device
{
    enum ipc_hw_device_type_e hw_type;
    struct msgbx_hw_info hw_cfg;
    uint64_t spec_cfg;
};

// msgbx error msg
#ifdef IPC_STATE_MGT_ENABLE
struct msgbx_err_msg
{
    uint8_t type : 4; // msgbx. msgend, msgflt
    uint8_t id : 4;
    uint8_t msg : 8; // overflow, underflow, threshold
    uint64_t res : 16;
};
#endif

// maybe it has big-endian or little-endian issue

#endif