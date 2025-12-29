/* SPDX-License-Identifier: GPL-2.0 OR Apache 2.0
 *
 * Copyright (c) 2024 Black Sesame Technologies
 *
 * This program is also distributed under the terms of the Apache 2.0
 * License.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef _IPC_HW_COMMON_H
#define _IPC_HW_COMMON_H

#include <bst/bstipc_cfg.h>
#include <bst/ipc_serdes.h>

// msgbx end init  ------------------------------------------------------------------------
// safety management enable flag bit
#define MSGBX_ECC_RX_MULTIP_EN_BIT 0x08
#define MSGBX_ECC_RX_DETECT_EN_BIT 0x04
#define MSGBX_PARITY_HWDATA_EN_BIT 0x02
#define MSGBX_PARITY_HADDR_EN_BIT 0x01

enum _ipc_hw_device_type_t {
	IPC_HW_MSGBX_MODE = 0,
	IPC_HW_INT_MODE = 1,
	IPC_HW_SEM_MODE = 2,
	IPC_HW_MAX_MODE
};
#define ipc_hw_device_type_t enum _ipc_hw_device_type_t

/**
 * ipc_init_params: ipc hardware device static init param
 * @mbx_device:
 * @msgbx_end_mgt_flag:
 */
struct _ipc_init_params_t {
	ipc_hw_device_type_t mbx_device;
	uint8_t msgbx_end_mgt_flag;
	uint8_t rsv[3];
	uint64_t spec_cfg;
};
#define ipc_init_params_t struct _ipc_init_params_t

struct _msgbx_hw_info_t {
	uint8_t mbx_version;
	uint8_t mbx_flt_cnt;
	uint8_t mbx_end_id;
	uint8_t mbx_txfifo_depth;
	uint8_t mbx_rxfifo_depth;
	uint8_t is_64_bit;
	uint8_t chipid;
};
#define msgbx_hw_info_t struct _msgbx_hw_info_t

// msgbx filter management  ------------------------------------------------------------------------
// msgbx filter config
struct _msgbx_flt_cfg_t {
	uint8_t flt_id;
	uint8_t mbx_flt_mgt_flag;
	uint8_t flt_rxfifo_st;
	uint8_t flt_rxfifo_end;
};
#define msgbx_flt_cfg_t struct _msgbx_flt_cfg_t

enum _msgbx_flt_rule_typ_t {
	MSGBX_FLT_RULE_PID = 1,
	MSGBX_FLT_RULE_LEN = 2,
	MSGBX_FLT_RULE_USER = 3,
	MSGBX_FLT_RULE_CHIPID,
	MSGBX_FTT_RULE_NONSEC,
	MSGBX_FLT_RULE_MAX,
};
#define msgbx_flt_rule_typ_t enum _msgbx_flt_rule_typ_t

struct _msgbx_flt_rule_pid_t
{
	uint8_t mbx_rx_pid_end;
	uint8_t mbx_rx_pid_st;
	uint8_t mbx_pid_flt_invert;
};
#define msgbx_flt_rule_pid_t struct _msgbx_flt_rule_pid_t

struct _msgbx_flt_rule_chip_pid_t
{
	uint8_t mbx_rx_chip_pid_end;
	uint8_t mbx_rx_chip_pid_st;
	uint8_t mbx_chip_pid_flt_invert;
};
#define msgbx_flt_rule_chip_pid_t struct _msgbx_flt_rule_chip_pid_t

struct _msgbx_flt_rule_len_t {
	uint8_t mbx_rx_len_end : 4;
	uint8_t mbx_rx_len_st : 4;
	uint8_t mbx_len_flt_invert;
};
#define msgbx_flt_rule_len_t struct _msgbx_flt_rule_len_t

enum _msgbx_flt_rule_location_t {
	MSGBX_FLT_RULE_HEADER = 0,
	MSGBX_FLT_RULE_PAY1,
	MSGBX_FLT_RULE_PAY2,
	MSGBX_FLT_RULE_PAY3,
	MSGBX_FLT_RULE_PAY4,
	MSGBX_FLT_RULE_PAY_MAX,
};
#define msgbx_flt_rule_location_t enum _msgbx_flt_rule_location_t

struct _msgbx_flt_rule_user_t {
	uint8_t msg_combi_lh_comp;
	uint8_t msg_flilter_invert;
	uint8_t rsv[2];
	uint32_t rx_res_mask;
	uint32_t rx_res_maskh;
	uint32_t rx_res_min;
	uint32_t rx_res_minh;
	uint32_t rx_res_max;
	uint32_t rx_res_maxh;
};
#define msgbx_flt_rule_user_t struct _msgbx_flt_rule_user_t

struct _msgbx_flt_rule_cfg_t {
	uint8_t cfg_type;
	uint8_t cfg_loc;
	uint8_t filter_combi_mode;
	uint8_t rsv;
	union {
		msgbx_flt_rule_pid_t rule_pid;
		msgbx_flt_rule_len_t rule_len;
		msgbx_flt_rule_user_t rule_user;
		msgbx_flt_rule_chip_pid_t rule_chipid;
		uint8_t rule_nonsec_sel;
	};
};
#define msgbx_flt_rule_cfg_t struct _msgbx_flt_rule_cfg_t
// msgbx state management ------------------------------------------------------------------------
// state management enable flag bit
#define MSGBX_TX_MSGBX_ERR_EN_BIT 0x20
#define MSGBX_RX_OVERFLOW_EN_BIT 0x04
#define MSGBX_RX_UNDERFLOW_EN_BIT 0x02

enum _msgbx_err_code_t {
	MSGBX_ERR_RX_UNDERFLOW = 1,
	MSGBX_ERR_RX_OVERFLOW = 2,
	MSGBX_ERR_ECC_RX_MULTIP,
	MSGBX_ERR_ECC_RX_DETECT,
	MSGBX_ERR_PARITY_HWDATA,
	MSGBX_ERR_PARITY_HADDR,
	MSGBX_ERR_TX_MSG_ERR,
	MSGBX_ERR_TX_OVERFLOW,
	IPC_ERR_TYP_MAX
};
#define msgbx_err_code_t enum _msgbx_err_code_t

struct _msgbx_flt_info_t {
	uint8_t mbx_rxfifo_end_addr;
	uint8_t mbx_rxfifo_st_addr;
};
#define msgbx_flt_info_t struct _msgbx_flt_info_t

struct _msgbx_flt_device_t {
	msgbx_flt_cfg_t cfg;
	msgbx_flt_info_t info;
};
#define msgbx_flt_device_t struct _msgbx_flt_device_t

// msgbx device
struct _msgbx_hw_device_t {
	ipc_hw_device_type_t hw_type;
	msgbx_hw_info_t hw_cfg;
	uint64_t spec_cfg;
};
#define msgbx_hw_device_t struct _msgbx_hw_device_t

// msgbx error msg
struct _msgbx_err_msg_t {
	uint8_t fid;
	uint8_t err_code;
};
#define msgbx_err_msg_t struct _msgbx_err_msg_t

// hw rx/tx counter
struct _msgbox_hw_counter_t {
	uint64_t def_tx_cnt;
	uint64_t rx_cnt;
	uint64_t overflow_intr_cnt;
	uint64_t rx_thrs_intr_cnt;
};
#define msgbox_hw_counter_t struct _msgbox_hw_counter_t
// clear hw counter bit offset
#define MSGBX_DEF_OVERFLOW_CNT_CLR_BIT 0x8
#define MSGBX_DEF_THRS_CNT_CLR_BIT 0x4
#define MSGBX_DEF_RXMSG_CNT_CLR_BIT 0x2
#define MSGBX_DEF_TXMSG_CNT_CLR_BIT 0x1

#define MSGBX_FLT_OVERFLOW_CNT_CLR_BIT 0x4
#define MSGBX_FLT_THRS_CNT_CLR_BIT 0x2
#define MSGBX_FLT_RXMSG_CNT_CLR_BIT 0x1

//usage for miscdev of msgbox
struct handle_info_t {
	uint8_t endid;
	uint8_t fid;
	uint8_t sid;
	uint8_t cid;
	uint8_t role;
	uint8_t handle;
};

struct msg_t {
	uint8_t endid;
	uint8_t handle;
	serdes_t msg;
};

struct method_info_t {
	uint8_t endid;
	uint8_t handle;
	uint8_t cmd;
};

struct handle_t {
	uint8_t endid;
	uint8_t handle;
};

struct layer_info {
	uint8_t endid;
	uint8_t k_role;
};

struct hw_regs_cfg {
	uint32_t fid;
	uint32_t endid;
	uint32_t reg_type;
	uint32_t val_set;
	uint32_t val_get;
};

struct user_msg_t {
	uint8_t end_id;
	uint8_t handle;
	uint64_t timestamp;
	uint64_t timeout;
	rw_msg_t msg;
};

struct query_info_t{
	uint8_t end_id;
	uint8_t handle;
	uint32_t polling_times;
};

struct mmap_session_t {
	uint8_t end_id;
	uint8_t handle;
};

struct endmap_t {
	uint8_t end_id;
	sts_endmap_t endmap;
};

enum reg_type {
	REG_TYPE_RX_FF_ADDR, //End_Default_RxFIFO_ADDRR or End_filter1_RxFIFO_ADDRR
	REG_TYPE_RX_FF_THRD, //End_default_RxThrs_CFGR or End_filter_Thrs_CFGR
	REG_TYPE_RX_IRQ_EN, //End_EnR or End_filter_EnR
	REG_TYPE_RX_FLT_EN, //End_Default_MsgH_PIDF_CFGR or End_filter1_RxFIFO_ADDRR
	REG_TYPE_PID_FLT, //End_Default_MsgH_PIDF_CFGR or End_filter1_MsgH_PIDF_CFGR
	REG_TYPE_LEN_FLT, //End_filter1_MsgH_LenF_CFGR
	REG_TYPE_MSGH_MASKH, //End_filter1_MsgH_ResF_MaskHR
	REG_TYPE_MSGH_MINH, //End_filter1_MsgH_ResF_MinHR
	REG_TYPE_MSGH_MAXH, //End_filter1_MsgH_ResF_MaxHR
	REG_TYPE_MSGH_MASK, //End_filter1_MsgH_ResF_MaskR
	REG_TYPE_MSGH_MIN, //End_filter1_MsgH_ResF_MinR
	REG_TYPE_MSGH_MAX, //End_filter1_MsgH_ResF_MaxR
	REG_TYPE_FLT_CFGR, //End_filter1_RxFIFO_CFGR
	REG_TYPE_PAY_MASK, //End_filter1_MsgPx_MaskR or End_filter1_MsgPx_MaskHR
	REG_TYPE_PAY_MIN, //End_filter1_MsgPx_MinR or End_filter1_MsgPx_MinHR
	REG_TYPE_PAY_MAX, //End_filter1_MsgPx_MaxR or End_filter1_MsgPx_MaxHR
};

#define IPC_MSG_IO 'N'
#define IPC_MSG_IO_LAYER_START \
		_IOW(IPC_MSG_IO, 1, uint8_t)
#define IPC_MSG_IO_LAYER_STOP \
		_IOW(IPC_MSG_IO, 2, uint8_t)
#define IPC_MSG_IO_CREATE_HANDLE \
		_IOWR(IPC_MSG_IO, 3, struct handle_info_t)
#define IPC_MSG_IO_DESTORY_HANDLE \
		_IOW(IPC_MSG_IO, 4, struct handle_info_t)
#define IPC_MSG_IO_SEND_MSG \
		_IOW(IPC_MSG_IO, 5, struct msg_t)
#define IPC_MSG_IO_GET_MSG \
		_IOWR(IPC_MSG_IO, 6, struct msg_t)
#define IPC_MSG_IO_REGISTER_METHOD \
		_IOWR(IPC_MSG_IO, 7, struct method_info_t)
#define IPC_MSG_IO_UNREGISTER_METHOD \
		_IOW(IPC_MSG_IO, 8, struct method_info_t)
#define IPC_MSG_IO_QUERY_MSG \
		_IOR(IPC_MSG_IO, 9, struct query_info_t)
#define IPC_MSG_IO_RELEASE_RECV_WAIT \
		_IOW(IPC_MSG_IO, 10, struct handle_t)
#define IPC_MSG_IO_SET_REGS \
		_IOW(IPC_MSG_IO, 11, struct hw_regs_cfg)
#define IPC_MSG_IO_USER_SEND_MSG \
		_IOW(IPC_MSG_IO, 12, struct user_msg_t)
#define IPC_MSG_IO_USER_GET_MSG \
		_IOWR(IPC_MSG_IO, 13, struct user_msg_t)
#define IPC_MSG_IO_USER_GET_ENDMAP \
		_IOWR(IPC_MSG_IO, 14, struct endmap_t)

#endif
