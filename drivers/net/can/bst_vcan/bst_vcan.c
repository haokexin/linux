//SPDX-License-Identifier: GPL-2.0+
/*
* bst_flexcan.c - FLEXCAN CAN controller driver
* Copyright (c) 2005-2006 Varma Electronics Oy
* Copyright (c) 2009 Sascha Hauer, Pengutronix
* Copyright (c) 2010-2017 Pengutronix, Marc Kleine-Budde <kernel@pengutronix.de>
* Copyright (c) 2014 David Jander, Protonic Holland
* Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
*/
// Based on code originally by Andrey Volkov <avolkov@varma-el.com>

#include <linux/bitfield.h>
#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>
#include <linux/can/rx-offload.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/kthread.h>
#include <linux/io.h>
#include <linux/time.h>
#include <uapi/linux/sched/types.h>
#include <linux/syscalls.h>  
#include <linux/unistd.h>  
#include <linux/sched.h>  
#include "adas5_can_gateway_client.h"
#include "vcan_command.h"
#include "../../../bst_sqbuffer/sq_buffer.h"
#include "../../../bst_sqbuffer/mcu_id.h"
#include "../../../bst_sqbuffer/ipc_hw_sem.h"


#define DRV_NAME			"flexcan"
#define FLEXCAN_RXIMR_COUNT                       128
#define FLEXCAN_HR_TIME_STAMP_COUNT               128
#define FLEXCAN_ERFFEL_COUNT                      128

struct flexcan_priv* g_priv[16];
struct task_struct* g_p_vcan_send_thread;

static adas5_can_gateway_client_t* g_vcan_client;
static adas5_can_gateway_client_data_t g_vcan_client_data = { 0 };
static bool g_thread_init = false;
static bool g_any_can_opened = false;

char *g_can_names[16] = { "can0",  "can1" , "can2" , "can3" , "can4" , "can5" , "can6" , "can7" , "can8",
"can9",  "can10", "can11" , "can12" , "can13" , "can14" , "can15" };
switch0_can_gateway_UInt8Array128_t g_vcan_send_datas[16];
bool g_vcan_send_flag[16] = { false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false };
bool g_vcan_reg_flag[16] = { false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false };
int g_vcan_ipc_result[16] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }; //0:等待回复，1：发送成功，其他：发送失败
int g_vcan_dev_id[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 }; //0:等待回复，1：发送成功，其他：发送失败

#ifdef VCAN_DEBUG

int g_test_read[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int g_test_write[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int g_test_write_success[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
u64 g_test_delay[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
u64 g_max_delay[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
u64 g_min_delay[16] = { 100000000, 100000000, 100000000, 100000000, 100000000, 100000000, 100000000, 100000000, 100000000, 100000000, 100000000, 100000000, 100000000, 100000000, 100000000, 100000000 };
int g_max_num[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

u64 g_delay_info_value[16][1000];
u64 g_delay_info_get_time[16][1000];
u64 g_delay_info_sw_time[16][1000];
int g_delay_info_pos[16][1000];
int g_big_delay_num[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

u64 test_send_time_start[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
u64 test_send_time_end[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
u64 g_send_test_delay[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };


#endif // VCAN_DEBUG

#define ACTIVE_SEND_SLEEP_US 30 //us
#define INACTIVE_SEND_SLEEP_US 500 //us

#define IDLE_SLEEP_US 1000 //us

#define TIME_ACTIVE_TO_INACTIVE 3000000 //us

struct net_device* can_devs[16];
sq_buffer_proxy_t g_sq_buf;


char last_line_str[128];
char* last_line_str_p = last_line_str;
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

static const u8 dlc2len[] = { 0, 1, 2, 3, 4, 5, 6, 7,
				 8, 12, 16, 20, 24, 32, 48, 64 };

/* get data length from can_dlc with sanitized can_dlc */
static u8 can_dlc2len(u8 can_dlc)
{
	return dlc2len[can_dlc & 0x0F];
}


static const u8 len2dlc[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8,		/* 0 - 8 */
				 9, 9, 9, 9,			/* 9 - 12 */
				 10, 10, 10, 10,			/* 13 - 16 */
				 11, 11, 11, 11,			/* 17 - 20 */
				 12, 12, 12, 12,			/* 21 - 24 */
				 13, 13, 13, 13, 13, 13, 13, 13,	/* 25 - 32 */
				 14, 14, 14, 14, 14, 14, 14, 14,	/* 33 - 40 */
				 14, 14, 14, 14, 14, 14, 14, 14,	/* 41 - 48 */
				 15, 15, 15, 15, 15, 15, 15, 15,	/* 49 - 56 */
				 15, 15, 15, 15, 15, 15, 15, 15 };	/* 57 - 64 */

/* map the sanitized data length to an appropriate data length code */
static u8 can_len2dlc(u8 len)
{
	if (unlikely(len > 64))
		return 0xF;

	return len2dlc[len];
}

/* Structure of the message buffer */
struct flexcan_mb {
	u32 can_ctrl;
	u32 can_id;
	u32 data[];
};

/* Structure of the hardware registers */
struct flexcan_regs {
	u32 mcr;		/* 0x00 */
	u32 ctrl;		/* 0x04 - Not affected by Soft Reset */
	u32 timer;		/* 0x08 */
	u32 _reserved0;		/* 0x0c */
	u32 rxgmask;		/* 0x10 - Not affected by Soft Reset */
	u32 rx14mask;		/* 0x14 - Not affected by Soft Reset */
	u32 rx15mask;		/* 0x18 - Not affected by Soft Reset */
	u32 ecr;		/* 0x1c */
	u32 esr;		/* 0x20 */
	u32 imask2;		/* 0x24 */
	u32 imask1;		/* 0x28 */
	u32 iflag2;		/* 0x2c */
	u32 iflag1;		/* 0x30 */
	u32 ctrl2;		/* 0x34 */
	u32 esr2;		/* 0x38 */
	u32 _reserved1[2];	/* 0x3c - 0x40 */
	u32 crcr;		/* 0x44 */
	u32 rxfgmask;		/* 0x48 */
	u32 rxfir;		/* 0x4c - Not affected by Soft Reset */
	u32 cbt;		/* 0x50 - Not affected by Soft Reset */
	u32 _reserved2[5];		/* 0x54 - 0x64*/
	u32 imask4;		/* 0x68 */
	u32 imask3;		/* 0x6c */
	u32 iflag4;		/* 0x70 */
	u32 iflag3;		/* 0x74 */
	u32 _reserved3[2];/* 0x78 - 0x7c*/
	u8 mb[4][512];		/* 0x80 - Not affected by Soft Reset */
	/* FIFO-mode:
	 *			MB
	 * 0x080...0x08f	0	RX message buffer
	 * 0x090...0x0df	1-5	reserved
	 * 0x0e0...0x0ff	6-7	8 entry ID table
	 * 0x0e0...0x2df	6-7..37	8..128 entry ID table
	 *				size conf'ed via ctrl2::RFFN
	 */
	u32 rximr[FLEXCAN_RXIMR_COUNT];		/* 0x880 - 0xa7c */
	u32 _reserved4[12];	/* 0xa80 - 0xaac */
	u32 tx_smb[4];		/* 0xab0 */
	u32 rx_smb0[4];		/* 0xac0 */
	u32 rx_smb1[4];		/* 0xad0 */
	u32 mecr;		/* 0xae0 */
	u32 erriar;		/* 0xae4 */
	u32 erridpr;		/* 0xae8 */
	u32 errippr;		/* 0xaec */
	u32 rerrar;		/* 0xaf0 */
	u32 rerrdr;		/* 0xaf4 */
	u32 rerrsynr;		/* 0xaf8 */
	u32 errsr;		/* 0xafc */
	u32 _reserved5[60];	/* 0xb00 - 0xbec*/
	u32 eprs;		/* 0xbf0 */
	u32 encbt;		/* 0xbf4 */
	u32 edcbt;		/* 0xbf8 */
	u32 etdc;		/* 0xbfc */
	u32 fdctrl;		/* 0xc00 - Not affected by Soft Reset */
	u32 fdcbt;		/* 0xc04 - Not affected by Soft Reset */
	u32 fdcrc;		/* 0xc08 */
	u32 erfcr;		/* 0xc0c */
	u32 erfier;		/* 0xc10 */
	u32 erfsr;		/* 0xc14 */
	u32 _reserved6[2];	/* 0xc18 - 0xc1c */
	u32 rx_smb0_time_stamp;   /* 0xc20 */
	u32 rx_smb1_time_stamp;   /* 0xc24 */
	u32 _reserved7[2];   /* 0xc28 - 0xc2c */
	u32 hr_time_stamp[FLEXCAN_HR_TIME_STAMP_COUNT];/* 0xc30 - 0xe2c */
	u32 _reserved9[62];	/* 0xe30 - 0xf24*/
	u32 tx_smb_fd[18];	/* 0xf28 */
	u32 rx_smb0_fd[18];	/* 0xf70 */
	u32 rx_smb1_fd[18];	/* 0xfb8 */
	u8 emb[4][512];		/* 0x1000 - 0x17FF*/
	u32 _reserved10[512];/* 0x1800 - 0x1FFC*/
	u8 erfifo[2560]; /* 0x2000 */
	u32 _reserved11[384];   /* 0x2A00 */
	u32 erfilter[FLEXCAN_ERFFEL_COUNT];  /* 0x3000 */
};

struct flexcan_devtype_data {
	u32 quirks;		/* quirks needed for different IP cores */
};

struct flexcan_stop_mode {
	struct regmap *gpr;
	u8 req_gpr;
	u8 req_bit;
};

struct flexcan_ring_buffer {
	u32 head; /* head, dequeue direction */
	u32 tail; /* tail, enqueue direction */
	u32 size; /* total queue size */
	u32 *data; /* queue space */
	dma_addr_t data_phy_start;
	dma_addr_t data_phy;
};

struct flexcan_priv {
	struct can_priv can;
	struct can_rx_offload offload;
	struct device *dev;

	phys_addr_t phy_base;
	u8 tx_mb_idx;
	u8 mb_count;
	u8 mb_size;
	u8 clk_src;	/* clock source of CAN Protocol Engine */

	u32 rx_mask[4];
	u32 tx_mask[4];
	u32 rx_iflag[4];
	u32 tx_iflag[4];

	u32 reg_ctrl_default;
	u32 mailbox_tx_num;
	u32 net_queue_flag;
};

static int can_name_to_controller_id(const char* name)
{
	for (int i = 0; i != 16; i++)
	{
		if (strcmp(g_can_names[i], name) == 0)
		{
			return i;
		}
	}
	return 0;
};

static void vcan_method_callback(const switch0_can_gateway_ErrorEnum_t err, void* ext,const ext_info_t* info)
{
	int dev_id = (*(int*)ext);
	if (err == SWITCH0_CAN_GATEWAY_NO_ERROR)
	{
		g_vcan_ipc_result[dev_id] = 1;
#ifdef VCAN_DEBUG
		g_test_write_success[dev_id]++;
#endif
	}
	else {
		struct sk_buff* skb_err;
		struct can_frame* c_frame;
		g_vcan_ipc_result[dev_id] = 2;
		printk("error: vcan_method_callback got error from SW0\n");

		skb_err = alloc_can_skb(g_priv[dev_id]->offload.dev, &c_frame);
		if (skb_err)
		{
			c_frame = (struct can_frame*)skb_err->data;
			c_frame->can_id = CAN_ERR_TX_TIMEOUT;
			c_frame->len = 8;
			c_frame->len8_dlc = 8;
			__skb_queue_tail(&g_priv[dev_id]->offload.skb_irq_queue, skb_err);
			can_rx_offload_irq_finish(&g_priv[dev_id]->offload);
		}
	}
	if (can_devs[dev_id])
	{
		netif_wake_queue(can_devs[dev_id]);
	}
#ifdef VCAN_DEBUG
	if (test_send_time_start[dev_id])
	{
		struct timespec64 test_now_time;
		ktime_get_real_ts64(&test_now_time);
		g_send_test_delay[dev_id] += test_now_time.tv_sec * 1000000000 + test_now_time.tv_nsec - test_send_time_start[dev_id];
	}
#endif
};

static void on_vcan_dst_changed(bool flag, void* ext)
{
	printk("on_vcan_dst_changed success,%d. \n", (int)flag);
	*((bool*)ext) = flag;
}

static bool b_vcan_dst_avail = false;

/*send*/
static netdev_tx_t flexcan_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct canfd_frame* cfd = (struct canfd_frame*)skb->data;
	X2CAN* x2can = NULL;
	X2CAN_PARAMS* param = NULL;
	int dev_id = can_name_to_controller_id(dev->name);
	if (!g_vcan_ipc_result[dev_id])
	{
		return NETDEV_TX_BUSY;
	}
	if (g_vcan_client)
	{
		netif_stop_queue(dev);
		g_vcan_ipc_result[dev_id] = 0;
		x2can = (X2CAN*)&g_vcan_send_datas[dev_id];
		memset(&(g_vcan_send_datas[dev_id]), 0, sizeof(switch0_can_gateway_UInt8Array128_t));
		param = (X2CAN_PARAMS*)x2can->param;
	
		x2can->dlc = can_len2dlc(cfd->len);
		x2can->rtr = cfd->can_id & CAN_RTR_FLAG ? 1 : 0;
		x2can->ide = cfd->can_id & CAN_EFF_FLAG ? 1 : 0;
		x2can->esi = cfd->can_id & CAN_ERR_FLAG ? 1 : 0;
		x2can->edl = 0;
		if (skb->len == CANFD_MTU && cfd->len <= CANFD_MAX_DLEN)
		{
			x2can->edl = 1;
			x2can->brs = cfd->flags & CANFD_ESI ? 1 : 0;
		}
		x2can->id = ((cfd->can_id & CAN_SFF_MASK));
		param->des_can_bus_id = dev_id;
		for (int i = 0; i < (cfd->len); i += 1) 
		{
			x2can->data[i] = cfd->data[i];
		}
		g_vcan_send_flag[dev_id] = true;
	}
	kfree_skb(skb);
	return NETDEV_TX_OK;
}


static int do_vcan_send_thread(void* unused)
{
	int empty = 0;
	bool send_flag = false;
	bool send_active = false;
#ifdef VCAN_DEBUG
	struct timespec64 test_now_time;
#endif
	for (;;) 
	{
		send_flag = false;
		if (kthread_should_stop())
		{
#ifdef VCAN_DEBUG
			printk("do_vcan_send_thread exit.");
#endif
			break;
		}
		if (!g_any_can_opened)
		{
			usleep_range(IDLE_SLEEP_US, IDLE_SLEEP_US + 1);
		}
		for (int i = 0; i != 16; i++)
		{
			if (g_vcan_send_flag[i])
			{
				g_vcan_client->switch0_can_gateway_client.can_gateway_send_async((switch0_can_gateway_UInt8Array128_t*)&(g_vcan_send_datas[i]), vcan_method_callback, (void*)&(g_vcan_dev_id[i]), NULL);	
#ifdef VCAN_DEBUG
				g_test_write[i]++;
				ktime_get_real_ts64(&test_now_time);
				test_send_time_start[i] = test_now_time.tv_sec * 1000000000 + test_now_time.tv_nsec;
#endif // VCAN_DEBUG
				g_vcan_send_flag[i] = false;
				send_flag = true;
				send_active = true;
				g_any_can_opened = true;
				empty = 0;
			}
		}
		if(!send_flag)
		{
			empty++;
			if (empty > (TIME_ACTIVE_TO_INACTIVE / ACTIVE_SEND_SLEEP_US)) // 3s未收到数据
			{
				send_active = false;
			}
		}
		if (send_active)
			usleep_range(ACTIVE_SEND_SLEEP_US, ACTIVE_SEND_SLEEP_US + 1);
		else
			usleep_range(INACTIVE_SEND_SLEEP_US, INACTIVE_SEND_SLEEP_US + 1);
	}
	return 0;
}

struct sk_buff* g_read_sub;
struct canfd_frame* g_read_cfd_frame;
struct can_frame* g_read_c_frame;

CAN2X* g_read_can2x_data = NULL;
sqe_proxy_t* g_read_sq_buf = NULL;
CAN2X_PARAMS* g_read_param = NULL;
int g_read_can_id = 0;
u64 g_read_delay = 0;
struct timespec64 g_read_test_now_time;
bool g_b_set_priority = false;
bool g_b_msgbox_sub_ready = false;
void vcan_sub_cb(
	void* ext,
	const ext_info_t* info
)
{
	if(!g_b_set_priority)
	{
		struct sched_param sch_param;
		g_b_set_priority = true;
		// 设置调度策略为FIFO，并设置优先级
		sch_param.sched_priority = 99; // 优先级可以是1到99，数值越高，优先级越高
		if (sched_setscheduler(current, SCHED_FIFO, &sch_param) == -1) {
			printk("sched_setscheduler failed");
			return ;
		}
	}
	if (!g_b_msgbox_sub_ready)
	{
		g_b_msgbox_sub_ready = true;
	}
	while (true)
	{
		g_read_sq_buf = sq_buffer_consume_get(MCU_ADAS);
		if (g_read_sq_buf)
		{
			g_read_can2x_data = g_read_sq_buf->buffer;
			g_read_param = (CAN2X_PARAMS*)g_read_can2x_data->param;
			if (g_priv[g_read_param->src_can_bus_id])
			{
#ifdef VCAN_DEBUG
				g_any_can_opened = true;
				ktime_get_real_ts64(&g_read_test_now_time);
				g_read_delay = (g_read_test_now_time.tv_sec - (((u64)(g_read_param->gmac_timestamp_second_h)) << 32) - g_read_param->gmac_timestamp_second_l) * 1000000000 + (g_read_test_now_time.tv_nsec - g_read_param->gmac_timestamp_nanosecond);
				if (g_read_delay > g_max_delay[g_read_param->src_can_bus_id])
				{
					g_max_delay[g_read_param->src_can_bus_id] = g_read_delay;
					g_max_num[g_read_param->src_can_bus_id] = g_test_read[g_read_param->src_can_bus_id];
				}
				if (g_read_delay < g_min_delay[g_read_param->src_can_bus_id])
				{
					g_min_delay[g_read_param->src_can_bus_id] = g_read_delay;
				}
				if (g_read_delay > 100000)
				{
					if (g_big_delay_num[g_read_param->src_can_bus_id] < 1000)
					{
						g_delay_info_value[g_read_param->src_can_bus_id][g_big_delay_num[g_read_param->src_can_bus_id]] = g_read_delay;
						g_delay_info_get_time[g_read_param->src_can_bus_id][g_big_delay_num[g_read_param->src_can_bus_id]] = ((((u64)(g_read_param->gmac_timestamp_second_h)) << 32) + g_read_param->gmac_timestamp_second_l) * 1000000000 + g_read_param->gmac_timestamp_nanosecond;
						g_delay_info_sw_time[g_read_param->src_can_bus_id][g_big_delay_num[g_read_param->src_can_bus_id]] = g_read_test_now_time.tv_sec * 1000000000 + g_read_test_now_time.tv_nsec;
						g_delay_info_pos[g_read_param->src_can_bus_id][g_big_delay_num[g_read_param->src_can_bus_id]] = g_test_read[g_read_param->src_can_bus_id];
						g_big_delay_num[g_read_param->src_can_bus_id]++;
					}
				}
				g_test_delay[g_read_param->src_can_bus_id] += g_read_delay;
				g_test_read[g_read_param->src_can_bus_id]++;
#endif
				g_read_can_id = g_read_can2x_data->id;
				if (g_read_can2x_data->ide)
				{
					g_read_can_id = g_read_can_id | CAN_EFF_FLAG;
				}
				if (g_read_can2x_data->rtr)
				{
					g_read_can_id = g_read_can_id | CAN_RTR_FLAG;
				}
				if (g_read_can2x_data->esi)
				{
					g_read_can_id = g_read_can_id | CAN_ERR_FLAG;
				}

				if (g_read_can2x_data->edl)
				{
					g_read_sub = alloc_canfd_skb(g_priv[g_read_param->src_can_bus_id]->offload.dev, &g_read_cfd_frame);
					if (g_read_sub)
					{
						g_read_cfd_frame = (struct canfd_frame*)g_read_sub->data;
						g_read_cfd_frame->can_id = g_read_can_id;
						g_read_cfd_frame->len = can_dlc2len(g_read_can2x_data->dlc);
						g_read_cfd_frame->flags |= CANFD_FDF;
						if (g_read_can2x_data->brs)
						{
							g_read_cfd_frame->flags |= CANFD_ESI;
						}
						for (int i = 0; i != g_read_cfd_frame->len; i++)
						{
							g_read_cfd_frame->data[i] = g_read_can2x_data->data[i];
						}
					}
				}
				else
				{
					g_read_sub = alloc_can_skb(g_priv[g_read_param->src_can_bus_id]->offload.dev, &g_read_c_frame);
					if (g_read_sub)
					{
						g_read_c_frame = (struct can_frame*)g_read_sub->data;
						g_read_c_frame->can_id = g_read_can_id;
						if (g_read_can2x_data->dlc > 8)
						{
							g_read_can2x_data->dlc = 8;
						}
						g_read_c_frame->len = 8;
						g_read_c_frame->len8_dlc = g_read_can2x_data->dlc;
						for (int i = 0; i != g_read_can2x_data->dlc; i++)
						{
							g_read_c_frame->data[i] = g_read_can2x_data->data[i];
						}
					}
				}
				if (g_read_sub)
				{
					__skb_queue_tail(&g_priv[g_read_param->src_can_bus_id]->offload.skb_irq_queue, g_read_sub);
					can_rx_offload_irq_finish(&g_priv[g_read_param->src_can_bus_id]->offload);
				}
			}
			sq_buffer_consume_put(g_read_sq_buf);
		}
		else
		{
			break;
		}
	}
};

void vcan_sub_err_cb(int32_t err, void* ext,
	const ext_info_t* info)
{

}


static struct sk_buff* flexcan_mailbox_read(struct can_rx_offload* offload,
	unsigned int n, u32* timestamp,
	bool drop)
{
	return NULL;
}

//通用回调
static int flexcan_open(struct net_device *dev)
{
	struct flexcan_priv *priv = netdev_priv(dev);
	int dev_id = can_name_to_controller_id(dev->name);
	
	int err = 0;
	int timeout_count = 0;

	if (!g_thread_init)
	{
		g_thread_init = true;
		g_p_vcan_send_thread = kthread_create(do_vcan_send_thread, NULL, "do_vcan_send_thread");
		if (!IS_ERR(g_p_vcan_send_thread))
		{
			wake_up_process(g_p_vcan_send_thread);
		}
		else
		{
			printk("do_vcan_send_thread create failed\n");
			g_thread_init = false;
			return -ENOMEM;
		}
		g_vcan_client = adas5_can_gateway_client_init(&g_vcan_client_data);
		g_vcan_client->start();
		err = g_vcan_client->switch0_can_gateway_client.register_avail_changed(on_vcan_dst_changed, &b_vcan_dst_avail);
		if (err < 0)
		{
			printk("vcan service: register_avail_changed failed\n");
			return -EPERM;
		}
		printk("wait vcan service ready..\n");

		while (!b_vcan_dst_avail)
		{
			msleep(100);
			timeout_count++;
			if (timeout_count > 30)
			{
				printk("vcan service not ready. \n");
				g_thread_init = false;
				break;
			}
		}

		g_vcan_client->switch0_can_gateway_client.can_gateway_recv_sub(vcan_sub_cb, NULL, NULL, vcan_sub_err_cb, NULL);

		for (int i = 0; i != 1000; i++)
		{
			g_read_sq_buf = sq_buffer_consume_get(MCU_ADAS);
			if (!g_read_sq_buf || g_b_msgbox_sub_ready)
			{
				break;
			}
			else
			{
				sq_buffer_consume_put(g_read_sq_buf);
			}
		}
	}
	//默认支持canfd
	dev->mtu = CANFD_MTU;
	dev->tx_queue_len = 1000;

	err = open_candev(dev);
	if (err)
	{
		printk("open_candev %s failed.\n", dev->name);
		return err;
	}
	g_any_can_opened = true;
#ifdef VCAN_DEBUG
	printk("flexcan_open:%d %s, tx_queue_len:%d,addr_len:%d,hard_header_len:%d, neigh_priv_len:%d ,min_header_len:%d,num_rx_queues:%d,real_num_rx_queues:%d\n",
		dev->mtu, dev->name, dev->tx_queue_len, dev->addr_len, dev->hard_header_len, dev->neigh_priv_len, dev->min_header_len, dev->num_rx_queues, dev->real_num_rx_queues);
#endif
	priv->offload.mailbox_read = flexcan_mailbox_read;
	priv->mb_size = sizeof(struct flexcan_mb) + CANFD_MAX_DLEN;
	err = can_rx_offload_add_fifo(dev, &priv->offload, 20);
	can_rx_offload_enable(&priv->offload);
	netif_start_queue(dev);

	can_devs[dev_id] = dev;
	g_priv[dev_id] = priv;
	return 0;
}

static int flexcan_close(struct net_device *dev)
{
	struct flexcan_priv* priv = NULL;
	int dev_id = 0;
	
	priv = netdev_priv(dev);
	dev_id = can_name_to_controller_id(dev->name);
	
#ifdef VCAN_DEBUG
	printk("can%d close. read num:%d, write num:%d, write success num:%d, avg delay:%llu , max delay :%llu (%d) , min delay:%llu, send delay avg: %llu, delay bigger than 100us num: %d\n",
		dev_id, g_test_read[dev_id], g_test_write[dev_id], g_test_write_success[dev_id], g_test_delay[dev_id] / g_test_read[dev_id], g_max_delay[dev_id], g_max_num[dev_id], g_min_delay[dev_id],
		g_send_test_delay[dev_id]/ g_test_write_success[dev_id], g_big_delay_num[dev_id]);
	g_test_read[dev_id] = 0;
	g_test_write[dev_id] = 0;
	g_test_write_success[dev_id] = 0;
	g_max_delay[dev_id] = 0;
	g_test_delay[dev_id] = 0;
	g_max_num[dev_id] = 0;
	g_min_delay[dev_id] = 100000000;
	g_send_test_delay[dev_id] = 0;

	for (int i = 0; i != g_big_delay_num[dev_id]; i++)
	{
		if (g_delay_info_value[dev_id][i])
		{
			printk("%d %llu . %llu: %llu\n", g_delay_info_pos[dev_id][i], g_delay_info_value[dev_id][i], g_delay_info_sw_time[dev_id][i], g_delay_info_get_time[dev_id][i]);
		}
	}
	g_big_delay_num[dev_id] = 0;
#endif // VCAN_DEBUG

	g_priv[dev_id] = NULL;
	can_devs[dev_id] = NULL;
	g_vcan_ipc_result[dev_id] = 1;
	g_vcan_send_flag[dev_id] = false;
	can_rx_offload_disable(&priv->offload);
	can_rx_offload_del(&priv->offload);
	close_candev(dev);
	
	for (int i = 0; i != 16; i++)
	{
		if (g_priv[i])
		{
			return 0;
		}
	}
	g_any_can_opened = false;
	return 0;
}

static const struct net_device_ops flexcan_netdev_ops = {
	.ndo_open	= flexcan_open,
	.ndo_stop	= flexcan_close,
	.ndo_start_xmit	= flexcan_start_xmit,
	.ndo_change_mtu = can_change_mtu,
};
static const struct flexcan_devtype_data bst_c1200_devtype_data = {

};
static const struct of_device_id flexcan_of_match[] = {
	{ .compatible = "bst,bst-flexcan", .data = &bst_c1200_devtype_data, },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, flexcan_of_match);

static const struct platform_device_id flexcan_id_table[] = {
	{ .name = "flexcan", .driver_data = (kernel_ulong_t)&bst_c1200_devtype_data, },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(platform, flexcan_id_table);


static int register_can_dev(int id, struct platform_device* pdev)
{
	if (!g_vcan_reg_flag[id])
	{
		struct net_device* dev;
		struct flexcan_priv* priv;
		int err;
		dev = alloc_candev(sizeof(struct flexcan_priv), 1);
		dev->netdev_ops = &flexcan_netdev_ops;
		dev->flags |= IFF_ECHO;
		priv = netdev_priv(dev);
		priv->dev = &pdev->dev;
		strlcpy(dev->name, g_can_names[id], IFNAMSIZ);
		priv->can.ctrlmode_supported |= CAN_CTRLMODE_FD | CAN_CTRLMODE_FD_NON_ISO;
		err = register_candev(dev);
		if (err)
		{
			printk("register_candev can%d failed.\n", id);
			return -ENOMEM;
		}
		pm_runtime_put(priv->dev);
		g_vcan_reg_flag[id] = true;
	}
	return 0;
}


static int flexcan_probe(struct platform_device *pdev)
{
	for (int i = 0; i != 16; i++)
	{
		register_can_dev(i, pdev);
	}
	return 0;
}

static int flexcan_remove(struct platform_device *pdev)
{
	device_set_wakeup_enable(&pdev->dev, false);
	device_set_wakeup_capable(&pdev->dev, false);
	b_vcan_dst_avail = false;

	adas5_can_gateway_client_destroy();
	return 0;
}


static int __maybe_unused flexcan_suspend(struct device* device)
{
	printk("flexcan_suspend \n");
	return 0;
}

static int __maybe_unused flexcan_resume(struct device* device)
{
	printk("flexcan_resume \n");
	return 0;
}

static int __maybe_unused flexcan_runtime_suspend(struct device* device)
{
	printk("flexcan_runtime_suspend \n");
	return 0;
}

static int __maybe_unused flexcan_runtime_resume(struct device* device)
{
	printk("flexcan_runtime_resume \n");
	return 0;
}
static int __maybe_unused flexcan_noirq_suspend(struct device* device)
{
	printk("flexcan_noirq_suspend \n");
	return 0;
}

static int __maybe_unused flexcan_noirq_resume(struct device* device)
{
	printk("flexcan_noirq_resume \n");
	return 0;
}


static const struct dev_pm_ops flexcan_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(flexcan_suspend, flexcan_resume)
	SET_RUNTIME_PM_OPS(flexcan_runtime_suspend, flexcan_runtime_resume, NULL)
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(flexcan_noirq_suspend, flexcan_noirq_resume)
};

static struct platform_driver flexcan_driver = {
	.driver = {
		.name = DRV_NAME,
		.pm = &flexcan_pm_ops,
		.of_match_table = flexcan_of_match,
	},
	.probe = flexcan_probe,
	.remove = flexcan_remove,
	.id_table = flexcan_id_table,
};

module_platform_driver(flexcan_driver);

MODULE_DESCRIPTION("BST FLEXCAN driver");
MODULE_AUTHOR("BST Ltd.");
MODULE_LICENSE("GPL v2");
