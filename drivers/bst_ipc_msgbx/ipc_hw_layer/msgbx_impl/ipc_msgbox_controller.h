/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is also distributed under the terms of the BSD 3-Clause
 * License.
 *
 * Copyright (C) 2024 Black Sesame Technologies. Inc.
 */
#ifndef IPC_MSGBOX_CONTROLLER_H
#define IPC_MSGBOX_CONTROLLER_H

#include <linux/workqueue.h>
#include <bst/bstipc_cfg.h>
#include <bst/ipc_lflist_siso.h>
#include "../../ipc_trans_layer/include/ipc_trans_common.h"
#include <bst/ipc_hw_impl.h>

#define MSGBOX_MAX_FILTER_NUM (8)
#define CPU_MSGEND_PPI_NUM (4)
#define MSG_64_MAX_LEN (4)
#define MSGBX_WAIT_TIMEOUT_CNT 0xFF
#define IPC_DRIVER_NAME	 "ipc_msgbox"
#define MAX_END_NUM (NR_CPUS)

#ifdef CONFIG_MSGBOX_DEBUG_FS
extern int msgbx_dbg_sysfs_init(struct device *dev);
extern void msgbx_dbg_sysfs_exit(struct device *dev);
#endif

#ifdef CONFIG_MSGBOX_DEBUG
#define MSGBOX_DEBUG 1
#else
#define MSGBOX_DEBUG 0
#endif
#if (MSGBOX_DEBUG == 0)
#define PRINT_DEBUG IPC_LOG_DEBUG
#else
#define PRINT_DEBUG IPC_LOG_ERR
#endif

union un_reg_sem {
	struct {
		u32 res:2;
		u32 sem_id:4;
		u32 intr:1;
		u32 bank_id:2;
		u32 mst_id:4;
		u32 base_addr:19;
	} bit;
	u32 data;
};
#define UN_REG_SEM union un_reg_sem

struct ipc_msgbox {
	struct device *dev;
	void __iomem *fcsr_base;
	void __iomem *rxfifo_base;
	void __iomem *txfifo_base;
	void __iomem *ipc_sem_base;
	u32 irq_count;
	u32 msgend_count;
	u32 filter_num;
};

struct st_msgbx_end_para {
	// end private data
#if defined(MSGBX_HW_TYPE_C1200)
	u16 irq[MSGBOX_MAX_FILTER_NUM];
#elif defined(MSGBX_HW_TYPE_A2000)
	u16 irq[CPU_MSGEND_PPI_NUM];
#endif
	void *private_data;
	struct ipc_msgbox *ipc_msgbx;
};
#define ST_MSGBX_END_PARA struct st_msgbx_end_para

struct st_msg_message64 {
	union {
		struct {
			u64 pid:8;
			u64 cid:8;
			u64 len:4;
			u64 is_64_bit:1;
			u64 nonsec:1;
			u64 is_eof : 1;
			u64 resh : 1;
			u64 sid : 4;
			u64 fid : 4;
			u64 res:32;
		} bit;
		u64 data;
	} head;
	u64 payload[MSG_64_MAX_LEN];
};

#define ST_MSG_MESSAGE64 struct st_msg_message64

enum {
	RES_ID_FILTER_CSR = 0,
	RES_ID_RXFIFO,
	RES_ID_TXFIFI,
};

enum {
	PPI_DEFAULT = 0,
	PPI_NUM1,
	PPI_NUM2,
	PPI_NUM3,
};

enum {
	FILTER_DEFAULT = 0,
	FILTER_NUM1,
	FILTER_NUM2,
	FILTER_NUM3,
	FILTER_NUM4,
	FILTER_NUM5,
	FILTER_NUM6,
	FILTER_NUM7,
	FILTER_NUM_BUFF,
};

enum {
	CPU_ID0 = 0,
	CPU_ID1,
	CPU_ID2,
	CPU_ID3,
};

struct st_msg_inf {
	union {
		struct {
			u32 endid:8;
			u32 is_64_bit:1;
			u32 filter_num:4;
			u32 res:19;
		} bit;
		u32 reg1;
	} ablt_r;

	union {
		struct {
			u32 rx_fifo_depth:10;
			u32 tx_fifo_depth:10;
			u32 res:12;
		} bit;
		u32 reg2;
	} ablt_r2;
	u8 version;
};
#define ST_MSG_INF struct st_msg_inf

enum {
	FILTER1_THRS_INTR = (1 << 0),
	FILTER1_UNDERFLOW_INTR = (1 << 1),
	FILTER1_OVERFLOW_INTR = (1 << 2),
	FILTER1_BUFF_INTR,
};

#if defined(MSGBX_HW_TYPE_C1200)
enum {
	DEF_FILTER_RX_THRS_INTR = (1 << 0),
	DEF_FILTER_RX_UNDERFLOW_INTR = (1 << 1),
	DEF_FILTER_RX_OVERFLOW_INTR = (1 << 2),
	DEF_FILTER_TX_THRS_INTR = (1 << 3),
	DEF_FILTER_TX_OVERFLOW_INTR = (1 << 4),
	DEF_FILTER_MSGBX_END_POOL_STATUS_INTR = (1 << 5),
	DEF_FILTER_BUFF_INTR = (1 << 6),
};
#elif defined(MSGBX_HW_TYPE_A2000)
enum {
	DEF_FILTER_RX_THRS_INTR = (1 << 0),
	DEF_FILTER_RX_UNDERFLOW_INTR = (1 << 1),
	DEF_FILTER_RX_OVERFLOW_INTR = (1 << 2),
	DEF_FILTER_TX_THRS_INTR = (1 << 3),
	DEF_FILTER_TX_OVERFLOW_INTR = (1 << 4),
	DEF_FILTER_TX_ERR_INTR = (1 << 5),
	DEF_FILTER_END_ST_NTF_INTR = (1 << 6),
	DEF_FILTER_BUFF_INTR = (1 << 7),
};
#endif

enum {
	COMBI_MODE_AND = 0,
	COMBI_MODE_OR,
	COMBI_MODE_RES,
};

#define REGS_FLT_NUM(regs)	((regs & 0x1E00ul) >> 9)

#define FILTER1_RES_LOW_BIT_MASK (0xfff00000ull)
#define FILTER1_PAYLOAD_LOW_BIT_MASK (0xffffffffull)

#define MSGBOX_RXFIFO_OFFSET (0x10000u)

#if defined(MSGBX_HW_TYPE_C1200)
#define MSGBOX_TXFIFO_OFFSET (0x11000u)
#elif defined(MSGBX_HW_TYPE_A2000)
#define MSGBOX_TXFIFO_OFFSET (0x18000u)
#define MSGBOX_IRQ_REG	(0x25000ul)
#endif

#define FILTER_MAX 7
#define FILTER_CSR_SIZE (0x200)
/* default filter csr offset */
#define DEF_FIL_ABLT_R (0x0)
#define DEF_FIL_ABLT2_R (0x8)
#define DEF_RXFIFO_ADDR (0x10)
#define DEF_FLT_MSG_PIDF_CFGR (0x18)
#define DEF_TX_FIFO_THRES (0x20)
#define DEF_Tx_FIFO_Available (0x28)
#define DEF_DEFAULT_RXTHRS_CFGR (0x30)
#define DEF_FLT_RXFIFO_STATUS (0x38)

#if defined(MSGBX_HW_TYPE_C1200)
#define DEF_FLT_INTER_EN (0x60)
#define DEF_FLT_INTER_CLR_R (0x58)
#define DEF_FLT_INTER_R (0x68)
#define DEF_FLT_VERSION_R (0x78)
#elif defined(MSGBX_HW_TYPE_A2000)
#define DEF_FLT_INTER_R (0x58)
#define DEF_FLT_INTER_EN (0x60)
#define DEF_FLT_INTER_CLR_R (0x68)
#define DEF_FLT_VERSION_R (0x78)
#define DEF_FLT_END_MASK_R (0x80)
#endif

#define END_RXFIFO_ADDR_RX_FIFO_ST_ADDR_SHIFT_U32 (0)
#define END_RXFIFO_ADDR_RX_FIFO_END_ADDR_SHIFT_U32 (10)
#define END_RXFIFO_ADDR_FILTER_EN_SHIFT_U32 (31)
#define END_RXFIFO_ADDR_MASK (0x3fful)

#define END_PIDF_CFGR_RX_PID_ST_SHIFT_U32 (0)
#define END_PIDF_CFGR_RX_PID_END_SHIFT_U32 (8)
#define END_PIDF_CFGR_RX_FILTER_INVERT_SHIFT_U32 (30)
#define END_PIDF_CFGR_RX_FILTER_EN_SHIFT_U32 (31)
#define END_PIDF_CFGR_MASK (0xfful)

/* filterN csr offset */
#define FILTER1_RXFIFO_ADDRR (0x0)
#define FILTER1_MSG_PIDF_CFGR (0x8)
#define	End_filter1_RxFIFO_ADDRR	0x00
#define	End_filter1_MsgH_PIDF_CFGR	0x08
#define	End_filter1_MsgH_LenF_CFGR	0x10
#define	End_filter1_MsgH_ResF_MaskR	0x18
#define	End_filter1_MsgH_ResF_MaskHR	0x20
#define	End_filter1_MsgH_ResF_MinR	0x28
#define	End_filter1_MsgH_ResF_MinHR	0x30
#define	End_filter1_MsgH_ResF_MaxR	0x38
#define	End_filter1_MsgH_ResF_MaxHR	0x40
#define	End_filter1_MsgP1_MaskR	0x48
#define	End_filter1_MsgP1_MaskHR	0x50
#define	End_filter1_MsgP1_MinR	0x58
#define	End_filter1_MsgP1_MinHR	0x60
#define	End_filter1_MsgP1_MaxR	0x68
#define	End_filter1_MsgP1_MaxHR	0x70
#define	End_filter1_MsgP2_MaskR	0x78
#define	End_filter1_MsgP2_MaskHR	0x80
#define	End_filter1_MsgP2_MinR	0x88
#define	End_filter1_MsgP2_MinHR	0x90
#define	End_filter1_MsgP2_MaxR	0x98
#define	End_filter1_MsgP2_MaxHR	0xA0
#define	End_filter1_MsgP3_MaskR	0xA8
#define	End_filter1_MsgP3_MaskHR	0xB0
#define	End_filter1_MsgP3_MinR	0xB8
#define	End_filter1_MsgP3_MinHR	0xC0
#define	End_filter1_MsgP3_MaxR	0xD0
#define	End_filter1_MsgP3_MaxHR	0xD8
#define	End_filter1_MsgP4_MaskR	0xE0
#define	End_filter1_MsgP4_MaskHR	0xE8
#define	End_filter1_MsgP4_MinR	0xF0
#define	End_filter1_MsgP4_MinHR	0xF8
#define	End_filter1_MsgP4_MaxR	0x100
#define	End_filter1_MsgP4_MaxHR	0x108
#define	End_filter1_RxFIFO_CFGR	0x110
#define	End_filter_Thrs_CFGR	0x118
#define	End_filter_RxFIFO_StatusR	0x120

#define FILTER1_INTER_ST_R (0x128)
#define FILTER1_EN_INTER (0x130)
#define FILTER1_INTER_CLR_R (0x138)
#define FILTER_INTER_R_OFFSET (FILTER1_INTER_CLR_R - FILTER1_INTER_ST_R)

/* end fmu register */
#define END_FMU_SAFETY_INTR (0x8000)
#define END_FMU_SAFETY_INTR_EN (0x8008)

//default pid config
#define DEF_FILTER_PID_ST       0x0
#define DEF_FILTER_PID_END      0xFF
#define DEF_FILTER_PID_INVERT   0x0
/* receive one message triggle an interrupt */
#define DEF_FILTER_THRES (1)
/* rxfifo depth */
#define END_FILTER_RX_FIFO_DEPTH (8)
#define DEF_FILTER_ST_ADDR CONFIG_MSGBOX_DEF_FLT_ST_ADDR
#define DEF_FILTER_END_ADDR CONFIG_MSGBOX_DEF_FLT_END_ADDR

/* msg header bits */
#define MSGHEADER_IS_64BIT (20)
#define MSGHEADER_IS_NONSEC (21)

/* ppi select reg */
#define CORE0_FILTER_PPI_SEL_REG	0x00
#define CORE0_PPI0_STATUS_REG		0x04
#define CORE0_PPI1_STATUS_REG		0x08
#define CORE0_PPI2_STATUS_REG		0x0c
#define CORE0_PPI3_STATUS_REG		0x10

#define CORE1_FILTER_PPI_SEL_REG	0x20
#define CORE1_PPI0_STATUS_REG		0x24
#define CORE1_PPI1_STATUS_REG		0x28
#define CORE1_PPI2_STATUS_REG		0x2c
#define CORE1_PPI3_STATUS_REG		0x30

#define CORE2_FILTER_PPI_SEL_REG	0x40
#define CORE2_PPI0_STATUS_REG		0x44
#define CORE2_PPI1_STATUS_REG		0x48
#define CORE2_PPI2_STATUS_REG		0x4c
#define CORE2_PPI3_STATUS_REG		0x50

#define CORE3_FILTER_PPI_SEL_REG	0x60
#define CORE3_PPI0_STATUS_REG		0x64
#define CORE3_PPI1_STATUS_REG		0x68
#define CORE3_PPI2_STATUS_REG		0x6c
#define CORE3_PPI3_STATUS_REG		0x70

#define INVALID_FILTER_PPI_SEL_REG	0xff

/* parameter is error */
#define  ERR_PARA 1

/* define for register */
union flt1_len_cfgr {
	struct {
		u32 rx_len_st:4;
		u32 rx_len_end:4;
		u32 res:22;
		u32 rx_len_filter_invert:1;
		u32 rx_len_filter_en:1;
	} bit;
	u32 data;
};
#define FLT1_LEN_CFGR union flt1_len_cfgr

union flt1_rx_fifo_cfgr {
	struct {
		u32 msgh_flilter_en:1;
		u32 msgp1_filter_en:1;
		u32 msgp2_filter_en:1;
		u32 msgp3_filter_en:1;
		u32 msgp4_filter_en:1;	//BIT4
		u32 res:3;
		u32 msg_flilter_invert:1;	//BIT8
		u32 msgp1_filter_invert:1;
		u32 msgp2_filter_invert:1;	//BIT10
		u32 msgp3_filter_invert:1;
		u32 msgp4_filter_invert:1;	//BIT12
		u32 msg_combi_lh_comp:1;	//BIT13
		u32 msgp1_combi_lh_comp:1;
		u32 msgp2_combi_lh_comp:1;
		u32 msgp3_combi_lh_comp:1;
		u32 msgp4_combi_lh_comp:1;	//BIT17
		u32 res1:12;
		u32 filter_combi_mode:2;
	} bit;
	u32 data;
};
#define FLT1_RX_FIFO_CFGR union flt1_rx_fifo_cfgr

/* defined for rxfifo config register */
enum {
	RXFIFO_MSGH_FLILTER_EN = 0,
	RXFIFO_MSGP1_FILTER_EN,
	RXFIFO_MSGP2_FILTER_EN,
	RXFIFO_MSGP3_FILTER_EN,
	RXFIFO_MSGP4_FILTER_EN,
	RXFIFO_MSGH_FLILTER_INVERT = 8,
	RXFIFO_MSGP1_FILTER_INVERT,
	RXFIFO_MSGP2_FILTER_INVERT,
	RXFIFO_MSGP3_FILTER_INVERT,
	RXFIFO_MSGP4_FILTER_INVERT,
	RXFIFO_MSGH_COMBI_LH_COMP = 13,
	RXFIFO_MSGP1_COMBI_LH_COMP,
	RXFIFO_MSGP2_COMBI_LH_COMP,
	RXFIFO_MSGP3_COMBI_LH_COMP,
	RXFIFO_MSGP4_COMBI_LH_COMP,
	RXFIFO_FILTER_COMBI_MODE = 30,
};

enum {
	MSG_FILTER1_RULE_PAYLOAD1 = 0,
	MSG_FILTER1_RULE_PAYLOAD2,
	MSG_FILTER1_RULE_PAYLOAD3,
	MSG_FILTER1_RULE_PAYLOAD4,
};

#define ENMU_END_ID(node_id, core_id) (node_id << 4 | core_id)

#if defined(MSGBX_HW_TYPE_C1200)
enum {
	PID_CMN_MSG_END0 = 0x10,
	PID_CMN_MSG_END1,
	PID_CMN_MSG_END2,
	PID_CMN_MSG_END3,
	PID_CMN_MSG_END4,
	PID_CMN_MSG_END5,
	PID_CMN_MSG_END6,
	PID_CMN_MSG_END7,
	PID_DB_MSG_END0 = 0x20,
	PID_DB_MSG_END1,
	PID_ISPCV_MSG_END0 = 0x30,
	PID_ISPCV_MSG_END1,
	PID_ISPCV_MSG_END2,
	PID_ISPCV_MSG_END3,
	PID_RISCV_MSG_END0,
	PID_NET_MSG_END0 = 0x40,
	PID_DMA_MSG_END0 = 0x50,
	PID_DMA_MSG_END1,
	PID_R5_SW_MSG_END0 = 0x60,
	PID_R5_SW_MSG_END1,
	PID_R5_SW_MSG_END2,
	PID_R5_SW_MSG_END3,
	PID_R5_SW_MSG_END4,
	PID_R5_SW_MSG_END5,
	PID_R5_SECURE_END0 = 0x70,
	PID_R5_SECURE_END1,
	PID_R5_SAFETY_END0 = 0x80,
	PID_R5_SAFETY_END1,
	PID_R5_REALTIME_END0 = 0x90,
	PID_R5_REALTIME_END1,
	PID_R5_REALTIME_END2,
	PID_R5_REALTIME_END3,
	PID_R5_REALTIME_END4,
	PID_R5_REALTIME_END5,
	PID_MEDIA_MSG_END0 = 0xA0,
};
#define MAX_END_ID		(PID_MEDIA_MSG_END0 + 1)
#elif defined(MSGBX_HW_TYPE_A2000)
enum {
	ID_A78_MSG_NODE0_END0 = ENMU_END_ID(0, 0),
	ID_A78_MSG_NODE0_END1 = ENMU_END_ID(0, 1),
	ID_A78_MSG_NODE0_END2 = ENMU_END_ID(0, 2),
	ID_A78_MSG_NODE0_END3 = ENMU_END_ID(0, 3),
	ID_A78_MSG_NODE1_END0 = ENMU_END_ID(1, 0),
	ID_A78_MSG_NODE1_END1 = ENMU_END_ID(1, 1),
	ID_A78_MSG_NODE1_END2 = ENMU_END_ID(1, 2),
	ID_A78_MSG_NODE1_END3 = ENMU_END_ID(1, 3),
	ID_A78_MSG_NODE2_END0 = ENMU_END_ID(2, 0),
	ID_A78_MSG_NODE2_END1 = ENMU_END_ID(2, 1),
	ID_A78_MSG_NODE2_END2 = ENMU_END_ID(2, 2),
	ID_A78_MSG_NODE2_END3 = ENMU_END_ID(2, 3),
	ID_A78_MSG_NODE3_END0 = ENMU_END_ID(3, 0),
	ID_A78_MSG_NODE3_END1 = ENMU_END_ID(3, 1),
	ID_A78_MSG_NODE3_END2 = ENMU_END_ID(3, 2),
	ID_A78_MSG_NODE3_END3 = ENMU_END_ID(3, 3),
	ID_R52_DOWN_PADSYS_MSG_NODE4_END0 = ENMU_END_ID(4, 0),
	ID_R52_DOWN_PADSYS_MSG_NODE4_END1 = ENMU_END_ID(4, 1),
	ID_R52_DOWN_PADSYS_MSG_NODE4_END2 = ENMU_END_ID(4, 2),
	ID_R52_DOWN_PADSYS_MSG_NODE4_END3 = ENMU_END_ID(4, 3),
	ID_R52_DOWN_PADSYS_MSG_NODE4_END4 = ENMU_END_ID(4, 4),
	ID_R52_DOWN_PADSYS_MSG_NODE4_END5 = ENMU_END_ID(4, 5),
	ID_ISP_CV_MSG_NODE5_END0 = ENMU_END_ID(5, 0),
	ID_ISP_CV_MSG_NODE5_END1 = ENMU_END_ID(5, 1),
	ID_ISP_CV_MSG_NODE5_END2 = ENMU_END_ID(5, 2),
	ID_ISP_CV_MSG_NODE5_END3 = ENMU_END_ID(5, 3),
	ID_SEC_MSG_NODE6_END0 = ENMU_END_ID(6, 0),
	ID_SAFE_MSG_NODE7_END0 = ENMU_END_ID(7, 0),
	ID_NET_MSG_NODE8_END0 = ENMU_END_ID(8, 0),
	ID_NET_MSG_NODE8_END1 = ENMU_END_ID(8, 1),
	ID_R52_UP_PADSYS_MSG_NODE9_END0 = ENMU_END_ID(9, 0),
	ID_R52_UP_PADSYS_MSG_NODE9_END1 = ENMU_END_ID(9, 1),
};
#define MAX_END_ID		(ID_R52_UP_PADSYS_MSG_NODE9_END1 + 1)
#endif


/* define for smp call */
struct st_msg_hw_init {
	ipc_init_params_t *msgbx_param;
	int ret;
};
#define ST_MSG_HW_INIT struct st_msg_hw_init

struct st_msg_hw_defint {
	int ret;
};
#define ST_MSG_HW_DEINIT struct st_msg_hw_defint

struct st_msg_hw_set_flt_cfg {
	msgbx_flt_rule_cfg_t *rule;
	uint8_t flt_id;
	int ret;
};
#define ST_MSG_HW_SET_FLT_CFG struct st_msg_hw_set_flt_cfg

struct st_msg_hw_clr_flt_cfg {
	uint8_t flt_id;
	int ret;
};
#define ST_MSG_HW_CLR_FLT_CFG struct st_msg_hw_clr_flt_cfg

struct st_msg_hw_send_msg {
	rw_msg_t *msg;
	int ret;
	uint64_t send_time;
	struct completion done;
	struct work_struct wk;
};
#define ST_MSG_HW_SEND_MSG struct st_msg_hw_send_msg

struct st_msg_hw_recv_msg {
	rw_msg_t *msg;
	int ret;
	uint8_t flt_id;
};
#define ST_MSG_HW_RECV_MSG struct st_msg_hw_recv_msg

struct st_msg_hw_sts_mgt_enable {
	uint32_t flag;
	int ret;
};
#define ST_MSG_HW_STS_MGT_ENABLE struct st_msg_hw_sts_mgt_enable

struct st_msg_hw_flt_mgt_enable {
	uint8_t flt_id;
	uint8_t flag;
	int ret;
};
#define ST_MSG_HW_FLT_MGT_ENABLE struct st_msg_hw_flt_mgt_enable

struct st_msg_hw_get_err_msg {
	msgbx_err_msg_t *err_msg;
	int ret;
	uint8_t flt_id;
};
#define ST_MSG_HW_GET_ERR_MSG struct st_msg_hw_get_err_msg

struct st_msg_hw_err_hdl {
	uint8_t type;
	uint8_t id;
	uint32_t hdl;
	int ret;
};
#define ST_MSG_HW_ERR_HDL struct st_msg_hw_err_hdl

struct msgbox_statis {
	uint64_t rx_thrs_stats[NR_CPUS];
	uint64_t rx_overflow_stats[NR_CPUS];
	uint64_t rx_underflow_stats[NR_CPUS];
	uint64_t tx_overflow_stats[NR_CPUS];
	uint64_t tx_fifo_unavail_stats[NR_CPUS];
	uint64_t tx_stats[NR_CPUS][MAX_END_NUM];
	uint64_t rx_stats[NR_CPUS][MAX_END_NUM];
	uint64_t rx_err_stats[NR_CPUS][MAX_END_NUM];
	uint64_t tx_pidcid_stats[NR_CPUS][MAX_END_ID];
	uint64_t rx_pidcid_stats[NR_CPUS][MAX_END_ID];
};

// function define
#define CHECK_CPUID(_cpu_id) \
	do { \
		if (_cpu_id >= MAX_END_NUM) { \
			IPC_LOG_ERR("cpu%u parameter error!", _cpu_id); \
			ret = -ERR_PARA; \
		} \
	} while (0)

#define FUNC_SMP_CALL(_func, _para) \
	do { \
		if (cpu_id == get_cpu()) \
			_func(&_para[cpu_id]);\
		else \
			smp_call_function_single(cpu_id, _func, &_para[cpu_id], 1);\
		put_cpu(); \
	} while (0)

#define FUNC_WQ_CALL(_func, _check, _para, wq) \
	do { \
		ret = _check(_para, cpu_id);\
		if (ret) {\
			return ret; \
		} else { \
			INIT_WORK(&_para->wk, _func); \
			ret = queue_work_on(cpu_id, wq, &_para->wk) ? 0 : 1 ; \
			return ret; \
		} \
	} while (0)

#define MSGBX_WQ_LEN	20

/**********************extern functions ***************************/
extern struct msgbox_statis *msgbx_get_statis(void);
extern int ipc_msgbx_miscdev_init(void);
extern void ipc_msgbx_miscdev_exit(void);
extern void *msgbx_get_filter_base(u32 fid);

#endif
