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
#include <linux/debugfs.h>
#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/ptp_clock_kernel.h>
#include "adas5_can_gateway_client.h"
#include "vcan_command.h"
#include "../../../bst_sqbuffer/sq_buffer.h"

// #define DEBUG_SCH_TIMEOUT_PRINT

#define VCAN_NAPI_WEIGHT 64

#ifndef VCAN_DEBUG_CAN_ID
#define VCAN_DEBUG_CAN_ID 0x123
#endif

#define MAX_CAN_CHANNEL 15
#define MIN_CAN_CHANNEL 0
#define CAN_CHANNEL_NUM 16

#define VIRTUAL_PORT_ID 101
#define VIRTUAL_DEV_NUM 4
#define ADAS2CAN_BUFFER_SIZE (32 * 1024)
#define ADAS2CAN_SQBUFFER_HEADER_RESERVER_SIZE (16*1024)
#define ADAS_VCAN_BUFFER_SQ_BUFFER_COUNT 63
#define ADAS2CAN_TX_SQB_INDEX 0
#define ADAS2CAN_RX_SQB_INDEX 0
#define ADAS2CAN_TX_THREAD_PRI 98
#define ADAS2CAN_RX_THREAD_PRI 99
#define ADAS2CAN_RX_CPU_CORE 5 // same with msgbox core

#define DRV_NAME			"bstvcan"
/* 8 for RX fifo and 2 error handling */
#define FLEXCAN_NAPI_WEIGHT (8 + 2)

/* FLEXCAN module configuration register (CANMCR) bits */
#define FLEXCAN_MCR_MDIS BIT(31)
#define FLEXCAN_MCR_FRZ BIT(30)
#define FLEXCAN_MCR_RFEN BIT(29)
#define FLEXCAN_MCR_HALT BIT(28)
#define FLEXCAN_MCR_NOT_RDY BIT(27)
#define FLEXCAN_MCR_WAK_MSK BIT(26)
#define FLEXCAN_MCR_SOFTRST BIT(25)
#define FLEXCAN_MCR_FRZ_ACK BIT(24)
#define FLEXCAN_MCR_SUPV BIT(23)
#define FLEXCAN_MCR_SLF_WAK BIT(22)
#define FLEXCAN_MCR_WRN_EN BIT(21)
#define FLEXCAN_MCR_LPM_ACK BIT(20)
#define FLEXCAN_MCR_WAK_SRC BIT(19)
#define FLEXCAN_MCR_DOZE BIT(18)
#define FLEXCAN_MCR_SRX_DIS BIT(17)
#define FLEXCAN_MCR_IRMQ BIT(16)
#define FLEXCAN_MCR_DMA BIT(15)
#define FLEXCAN_MCR_LPRIO_EN BIT(13)
#define FLEXCAN_MCR_AEN BIT(12)
#define FLEXCAN_MCR_FDEN BIT(11)
/* MCR_MAXMB: maximum used MBs is MAXMB + 1 */
#define FLEXCAN_MCR_MAXMB(x) ((x)&0x7f)
#define FLEXCAN_MCR_IDAM_A (0x0 << 8)
#define FLEXCAN_MCR_IDAM_B (0x1 << 8)
#define FLEXCAN_MCR_IDAM_C (0x2 << 8)
#define FLEXCAN_MCR_IDAM_D (0x3 << 8)

/* FLEXCAN control register (CANCTRL) bits */
#define FLEXCAN_CTRL_PRESDIV(x) (((x)&0xff) << 24)
#define FLEXCAN_CTRL_RJW(x) (((x)&0x03) << 22)
#define FLEXCAN_CTRL_PSEG1(x) (((x)&0x07) << 19)
#define FLEXCAN_CTRL_PSEG2(x) (((x)&0x07) << 16)
#define FLEXCAN_CTRL_BOFF_MSK BIT(15)
#define FLEXCAN_CTRL_ERR_MSK BIT(14)
#define FLEXCAN_CTRL_CLK_SRC BIT(13)
#define FLEXCAN_CTRL_LPB BIT(12)
#define FLEXCAN_CTRL_TWRN_MSK BIT(11)
#define FLEXCAN_CTRL_RWRN_MSK BIT(10)
#define FLEXCAN_CTRL_SMP BIT(7)
#define FLEXCAN_CTRL_BOFF_REC BIT(6)
#define FLEXCAN_CTRL_TSYN BIT(5)
#define FLEXCAN_CTRL_LBUF BIT(4)
#define FLEXCAN_CTRL_LOM BIT(3)
#define FLEXCAN_CTRL_PROPSEG(x) ((x)&0x07)
#define FLEXCAN_CTRL_ERR_BUS (FLEXCAN_CTRL_ERR_MSK)
#define FLEXCAN_CTRL_ERR_STATE \
    (FLEXCAN_CTRL_TWRN_MSK | FLEXCAN_CTRL_RWRN_MSK | FLEXCAN_CTRL_BOFF_MSK)
#define FLEXCAN_CTRL_ERR_ALL (FLEXCAN_CTRL_ERR_BUS | FLEXCAN_CTRL_ERR_STATE)

/* FLEXCAN control register 2 (CTRL2) bits */
#define FLEXCAN_CTRL2_ECRWRE BIT(29)
#define FLEXCAN_CTRL2_WRMFRZ BIT(28)
#define FLEXCAN_CTRL2_RFFN(x) (((x)&0x0f) << 24)
#define FLEXCAN_CTRL2_TASD(x) (((x)&0x1f) << 19)
#define FLEXCAN_CTRL2_MRP BIT(18)
#define FLEXCAN_CTRL2_RRS BIT(17)
#define FLEXCAN_CTRL2_EACEN BIT(16)
#define FLEXCAN_CTRL2_ISOCANFDEN BIT(12)

/* FLEXCAN memory error control register (MECR) bits */
#define FLEXCAN_MECR_ECRWRDIS BIT(31)
#define FLEXCAN_MECR_HANCEI_MSK BIT(19)
#define FLEXCAN_MECR_FANCEI_MSK BIT(18)
#define FLEXCAN_MECR_CEI_MSK BIT(16)
#define FLEXCAN_MECR_HAERRIE BIT(15)
#define FLEXCAN_MECR_FAERRIE BIT(14)
#define FLEXCAN_MECR_EXTERRIE BIT(13)
#define FLEXCAN_MECR_RERRDIS BIT(9)
#define FLEXCAN_MECR_ECCDIS BIT(8)
#define FLEXCAN_MECR_NCEFAFRZ BIT(7)

/* FLEXCAN error and status register (ESR) bits */
#define FLEXCAN_ESR_TWRN_INT BIT(17)
#define FLEXCAN_ESR_RWRN_INT BIT(16)
#define FLEXCAN_ESR_BIT1_ERR BIT(15)
#define FLEXCAN_ESR_BIT0_ERR BIT(14)
#define FLEXCAN_ESR_ACK_ERR BIT(13)
#define FLEXCAN_ESR_CRC_ERR BIT(12)
#define FLEXCAN_ESR_FRM_ERR BIT(11)
#define FLEXCAN_ESR_STF_ERR BIT(10)
#define FLEXCAN_ESR_TX_WRN BIT(9)
#define FLEXCAN_ESR_RX_WRN BIT(8)
#define FLEXCAN_ESR_IDLE BIT(7)
#define FLEXCAN_ESR_TXRX BIT(6)
#define FLEXCAN_EST_FLT_CONF_SHIFT (4)
#define FLEXCAN_ESR_FLT_CONF_MASK (0x3 << FLEXCAN_EST_FLT_CONF_SHIFT)
#define FLEXCAN_ESR_FLT_CONF_ACTIVE (0x0 << FLEXCAN_EST_FLT_CONF_SHIFT)
#define FLEXCAN_ESR_FLT_CONF_PASSIVE (0x1 << FLEXCAN_EST_FLT_CONF_SHIFT)
#define FLEXCAN_ESR_BOFF_INT BIT(2)
#define FLEXCAN_ESR_ERR_INT BIT(1)
#define FLEXCAN_ESR_WAK_INT BIT(0)
#define FLEXCAN_ESR_ERR_BUS                                                  \
    (FLEXCAN_ESR_BIT1_ERR | FLEXCAN_ESR_BIT0_ERR | FLEXCAN_ESR_ACK_ERR | \
     FLEXCAN_ESR_CRC_ERR | FLEXCAN_ESR_FRM_ERR | FLEXCAN_ESR_STF_ERR)
#define FLEXCAN_ESR_ERR_STATE \
    (FLEXCAN_ESR_TWRN_INT | FLEXCAN_ESR_RWRN_INT | FLEXCAN_ESR_BOFF_INT)
#define FLEXCAN_ESR_ERR_ALL (FLEXCAN_ESR_ERR_BUS | FLEXCAN_ESR_ERR_STATE)
#define FLEXCAN_ESR_ALL_INT                                                   \
    (FLEXCAN_ESR_TWRN_INT | FLEXCAN_ESR_RWRN_INT | FLEXCAN_ESR_BOFF_INT | \
     FLEXCAN_ESR_ERR_INT)

/* FLEXCAN Bit Timing register (CBT) bits */
#define FLEXCAN_CBT_BTF BIT(31)
#define FLEXCAN_CBT_EPRESDIV_MASK GENMASK(30, 21)
#define FLEXCAN_CBT_ERJW_MASK GENMASK(20, 16)
#define FLEXCAN_CBT_EPROPSEG_MASK GENMASK(15, 10)
#define FLEXCAN_CBT_EPSEG1_MASK GENMASK(9, 5)
#define FLEXCAN_CBT_EPSEG2_MASK GENMASK(4, 0)

/* FLEXCAN FD control register (FDCTRL) bits */
#define FLEXCAN_FDCTRL_FDRATE BIT(31)
#define FLEXCAN_FDCTRL_MBDSR3 GENMASK(26, 25)
#define FLEXCAN_FDCTRL_MBDSR2 GENMASK(23, 22)
#define FLEXCAN_FDCTRL_MBDSR1 GENMASK(20, 19)
#define FLEXCAN_FDCTRL_MBDSR0 GENMASK(17, 16)
#define FLEXCAN_FDCTRL_MBDSR_8 0x0
#define FLEXCAN_FDCTRL_MBDSR_12 0x1
#define FLEXCAN_FDCTRL_MBDSR_32 0x2
#define FLEXCAN_FDCTRL_MBDSR_64 0x3
#define FLEXCAN_FDCTRL_TDCEN BIT(15)
#define FLEXCAN_FDCTRL_TDCFAIL BIT(14)
#define FLEXCAN_FDCTRL_TDCOFF GENMASK(12, 8)
#define FLEXCAN_FDCTRL_TDCVAL GENMASK(5, 0)

/* FLEXCAN FD Bit Timing register (FDCBT) bits */
#define FLEXCAN_FDCBT_FPRESDIV_MASK GENMASK(29, 20)
#define FLEXCAN_FDCBT_FRJW_MASK GENMASK(18, 16)
#define FLEXCAN_FDCBT_FPROPSEG_MASK GENMASK(14, 10)
#define FLEXCAN_FDCBT_FPSEG1_MASK GENMASK(7, 5)
#define FLEXCAN_FDCBT_FPSEG2_MASK GENMASK(2, 0)

/* FLEXCAN enhanced rx fifo control register (ERFCR) bits */
#define FLEXCAN_ERFCR_ERFEN BIT(31)
#define FLEXCAN_ERFCR_DMALW_MASK GENMASK(30, 26)
#define FLEXCAN_ERFCR_DMALW(x) (((x)&0x1f) << 26)
#define FLEXCAN_ERFCR_NEXIF_MASK GENMASK(22, 16)
#define FLEXCAN_ERFCR_NEXIF(x) (((x)&0x7f) << 16)
#define FLEXCAN_ERFCR_NFE_MASK GENMASK(13, 8)
#define FLEXCAN_ERFCR_NFE(x) (((x)&0x3f) << 8)
#define FLEXCAN_ERFCR_ERFWM_MASK GENMASK(4, 0)
#define FLEXCAN_ERFCR_ERFWM(x) ((x)&0x1f)

#define FLEXCAN_ERFSR_ERFUFW BIT(31)
#define FLEXCAN_ERFSR_ERFOVF BIT(30)
#define FLEXCAN_ERFSR_ERFWMI BIT(29)
#define FLEXCAN_ERFSR_ERFDA BIT(28)
#define FLEXCAN_ERFSR_ERFCLR BIT(27)

#define FLEXCAN_ERFIER_ERFUFWIE BIT(31)
#define FLEXCAN_ERFIER_ERFOVFIE BIT(30)
#define FLEXCAN_ERFIER_ERFWMIIE BIT(29)
#define FLEXCAN_ERFIER_ERFDAIE BIT(28)

/* FLEXCAN interrupt flag register (IFLAG) bits */
/* Errata ERR005829 step7: Reserve first valid MB */
#define FLEXCAN_TX_MB_RESERVED_RX_FIFO 8
#define FLEXCAN_TX_MB_RESERVED_RX_MAILBOX 0
// #define FLEXCAN_RX_MB_RX_MAILBOX_FIRST	(FLEXCAN_TX_MB_RESERVED_RX_MAILBOX + 1)
#define FLEXCAN_RX_MB_RX_MAILBOX_FIRST 0
#define FLEXCAN_IFLAG_MB(x) BIT_ULL(x)
#define FLEXCAN_IFLAG_RX_FIFO_OVERFLOW BIT(7)
#define FLEXCAN_IFLAG_RX_FIFO_WARN BIT(6)
#define FLEXCAN_IFLAG_RX_FIFO_AVAILABLE BIT(5)

/* FLEXCAN message buffers */
#define FLEXCAN_MB_CODE_MASK (0xf << 24)
#define FLEXCAN_MB_CODE_RX_BUSY_BIT (0x1 << 24)
#define FLEXCAN_MB_CODE_RX_INACTIVE (0x0 << 24)
#define FLEXCAN_MB_CODE_RX_EMPTY (0x4 << 24)
#define FLEXCAN_MB_CODE_RX_FULL (0x2 << 24)
#define FLEXCAN_MB_CODE_RX_OVERRUN (0x6 << 24)
#define FLEXCAN_MB_CODE_RX_RANSWER (0xa << 24)

#define FLEXCAN_MB_CODE_TX_INACTIVE (0x8 << 24)
#define FLEXCAN_MB_CODE_TX_ABORT (0x9 << 24)
#define FLEXCAN_MB_CODE_TX_DATA (0xc << 24)
#define FLEXCAN_MB_CODE_TX_TANSWER (0xe << 24)

#define FLEXCAN_MB_CNT_EDL BIT(31)
#define FLEXCAN_MB_CNT_BRS BIT(30)
#define FLEXCAN_MB_CNT_ESI BIT(29)
#define FLEXCAN_MB_CNT_SRR BIT(22)
#define FLEXCAN_MB_CNT_IDE BIT(21)
#define FLEXCAN_MB_CNT_RTR BIT(20)
#define FLEXCAN_MB_CNT_LENGTH(x) (((x)&0xf) << 16)
#define FLEXCAN_MB_CNT_TIMESTAMP(x) ((x)&0xffff)

#define FLEXCAN_TIMEOUT_US (250)



#define FLEXCAN_RXIMR_COUNT                       128
#define FLEXCAN_HR_TIME_STAMP_COUNT               128
#define FLEXCAN_ERFFEL_COUNT                      128

struct bstvcan_priv* g_priv[16];
struct task_struct* g_p_vcan_write_thread;
struct task_struct* g_p_vcan_init_thread;
struct completion g_vcan_write_event;

static adas5_can_gateway_client_t* g_vcan_client;
static adas5_can_gateway_client_data_t g_vcan_client_data = { 0 };
static bool g_thread_init = false;
static bool g_any_can_opened = false;

char* g_can_names[16] = { "can0",  "can1" , "can2" , "can3" , "can4" , "can5" , "can6" , "can7" , "can8",
"can9",  "can10", "can11" , "can12" , "can13" , "can14" , "can15" };
switch0_can_gateway_UInt8Array128_t g_vcan_write_datas[16];
bool g_vcan_write_flag[16] = { false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false };
bool g_vcan_reg_flag[16] = { false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false };
int g_vcan_tx_available[16] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }; //0: waiting for a reply, 1: successfully sent, others: failed to send
int g_vcan_last_skb_len[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; 
int g_vcan_dev_id[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 }; //0: waiting for a reply, 1: successfully sent, others: failed to send
int g_vcan_virtual_dev[VIRTUAL_DEV_NUM] = {4, 5, 6, 7};
static bool g_vport_open = false;
static atomic_t g_vcan_init = ATOMIC_INIT(0);;
static uint64_t g_sq_buffer_phyaddr = 0;
static struct dentry * g_debugfs_dir;
static uint64_t g_vcan_debug = 0;

int g_debug_read_num[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int g_debug_write_num[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int g_debug_write_success_num[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
u64 g_debug_total_read_delay[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
u64 g_debug_read_max_delay[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
u64 g_debug_read_min_delay[16] = { 100000000, 100000000, 100000000, 100000000, 100000000, 100000000, 100000000, 100000000, 100000000, 100000000, 100000000, 100000000, 100000000, 100000000, 100000000, 100000000 };
u32 g_debug_read_sch_max_delay[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

#define ACTIVE_SEND_SLEEP_US 50 //us
#define INACTIVE_SEND_SLEEP_US 500 //us

#define IDLE_SLEEP_US 500 //us

#define TIME_ACTIVE_TO_INACTIVE 1000000 //us

struct net_device* can_devs[16];

struct bstvcan_priv {
    struct can_priv can;
    struct can_rx_offload offload;
    struct device* dev;
};

static int vcan_init(void* unused);
extern int bst_get_ts_from_xgmac(struct timespec64 *ts, struct ptp_system_timestamp *sts);

static int can_name_to_controller_id(const char* name)
{
    for (int i = MIN_CAN_CHANNEL; i != MAX_CAN_CHANNEL + 1; i++)
    {
        if (strcmp(g_can_names[i], name) == 0)
        {
            return i;
        }
    }
    return 0;
};

static bool last_dst_avail = false;
static bool b_vcan_dst_avail = false;
static bool vcan_server_restart = false;

static void on_vcan_dst_changed(bool flag, void* ext)
{
    printk("on_vcan_dst_changed success,%d. \n", (int)flag);
    *((bool*)ext) = flag;

    if (flag) {
        if (last_dst_avail) {
            // If the value is true last time, the server restarts, resets the route, and reopen vport
            vcan_server_restart = true;
            g_p_vcan_init_thread = kthread_create(vcan_init, NULL, "vcan_init");
            if (!IS_ERR(g_p_vcan_init_thread))
            {
                wake_up_process(g_p_vcan_init_thread);
            }
        }
        else
            last_dst_avail = true;
    }
    else {
        if (last_dst_avail) {
            // If the value is true last time, the server is offline
            g_vport_open = false;
            atomic_set(&g_vcan_init, 0);
        }
    }
}

/*send*/
static netdev_tx_t bstvcan_start_xmit(struct sk_buff* skb, struct net_device* dev)
{
    struct canfd_frame* cfd = (struct canfd_frame*)skb->data;
    X2CAN* x2can = NULL;
    X2CAN_PARAMS* param = NULL;
    int dev_id = can_name_to_controller_id(dev->name);
    if (!g_vcan_tx_available[dev_id])
    {
        return NETDEV_TX_BUSY;
    }

    if (g_vcan_client)
    {
        netif_stop_queue(dev);
        g_vcan_tx_available[dev_id] = 0;
        g_vcan_last_skb_len[dev_id] = can_skb_get_data_len(skb);
        x2can = (X2CAN*)&g_vcan_write_datas[dev_id];
        memset(&(g_vcan_write_datas[dev_id]), 0, sizeof(switch0_can_gateway_UInt8Array128_t));
        param = (X2CAN_PARAMS*)x2can->param;

        x2can->dlc = can_fd_len2dlc(cfd->len);
        x2can->rtr = cfd->can_id & CAN_RTR_FLAG ? 1 : 0;
        x2can->ide = cfd->can_id & CAN_EFF_FLAG ? 1 : 0;
        x2can->esi = cfd->can_id & CAN_ERR_FLAG ? 1 : 0;
        x2can->edl = 0;
        if (skb->len == CANFD_MTU && cfd->len <= CANFD_MAX_DLEN)
        {
            x2can->edl = 1;
            x2can->brs = cfd->flags & CANFD_BRS ? 1 : 0;
        }
        if (cfd->can_id & CAN_EFF_FLAG)
        {
            x2can->id = cfd->can_id & CAN_EFF_MASK;
        }
        else
        {
            x2can->id = cfd->can_id & CAN_SFF_MASK;
        }

        param->src_vcan_port = VIRTUAL_PORT_ID;
        param->des_can_bus_id = dev_id;
        for (int i = 0; i < (cfd->len); i += 1)
        {
            x2can->data[i] = cfd->data[i];
        }
        //dcache_clean_poc((unsigned long)(&g_vcan_write_datas[dev_id]), (unsigned long)(&g_vcan_write_datas[dev_id])+sizeof(switch0_can_gateway_UInt8Array128_t));

        if (g_vcan_debug) {
            struct ptp_system_timestamp send_xgmac_ts_with_offset = {0};
            uint64_t send_timestamp = 0;
            int ret = 0;

            ret = bst_get_ts_from_xgmac(NULL, &send_xgmac_ts_with_offset);
            if (ret) {
                send_xgmac_ts_with_offset.pre_ts.tv_sec = TIME64_MIN;
                send_xgmac_ts_with_offset.pre_ts.tv_nsec = 0;
                printk("%s bst_get_ts_from_xgmac failed, ret = %d.\n", __func__, ret);
            }

            send_timestamp = send_xgmac_ts_with_offset.pre_ts.tv_sec * 1000000000 + send_xgmac_ts_with_offset.pre_ts.tv_nsec;
            param->timestamp_h = (send_timestamp >> 32) & 0xffffffff;
            param->timestamp_l = send_timestamp & 0xffffffff;
        }

        g_vcan_write_flag[dev_id] = true;
        g_debug_write_num[dev_id]++;
        complete(&g_vcan_write_event);
    } else {
        dev->stats.tx_dropped++;
    }
    kfree_skb(skb);
    return NETDEV_TX_OK;
}

static void shuffle_array(int *arr, int size)
{
    int i, j, temp;
    
    for (i = size - 1; i > 0; i--) 
    {
        j = get_random_u32() % (i + 1);
        temp = arr[i];
        arr[i] = arr[j];
        arr[j] = temp;
    }
}

static int do_vcan_write_thread(void* unused)
{
    sqe_proxy_t *sqe;
    struct net_device_stats* stats;
    int seq_indexs[VIRTUAL_DEV_NUM] = {0};
    struct sched_param sch_param;

    sch_param.sched_priority = ADAS2CAN_TX_THREAD_PRI;
    if (sched_setscheduler(current, SCHED_FIFO, &sch_param) == -1) {
        printk("%s sched_setscheduler failed\n", __func__);
    }

    memcpy(seq_indexs, g_vcan_virtual_dev, sizeof(int)*VIRTUAL_DEV_NUM);

    while (!kthread_should_stop())
    {
        wait_for_completion(&g_vcan_write_event);

        /* retry if any data in que. */
        while (!kthread_should_stop()) {
            bool data_left;

            /* inited? */
            if (!atomic_read(&g_vcan_init)) {
                usleep_range(IDLE_SLEEP_US, IDLE_SLEEP_US + 1);
                continue;
            }

            data_left = false;
            // send data through the can channel in a random sequence
            shuffle_array(seq_indexs, VIRTUAL_DEV_NUM);
            for (int j = 0; j < VIRTUAL_DEV_NUM; j++) {
                int i = seq_indexs[j];
                if (g_vcan_write_flag[i]) {
                    sqe = sq_buffer_produce_get();
                    if (sqe == NULL) {
                        data_left = true;
                        break;
                    }

                    g_vcan_write_flag[i] = false;
                    //dcache_inval_poc((unsigned long)(&g_vcan_write_datas[i]), (unsigned long)(&g_vcan_write_datas[i])+sizeof(switch0_can_gateway_UInt8Array128_t));
                    memcpy(sqe->buffer, &g_vcan_write_datas[i], sizeof(X2CAN));
                    sq_buffer_produce_put(sqe, BIT(ADAS2CAN_TX_SQB_INDEX));
                    stats = &can_devs[i]->stats;
                    stats->tx_bytes += g_vcan_last_skb_len[i];
                    stats->tx_packets++;
                    g_vcan_last_skb_len[i] = 0;
                    barrier();
                    g_vcan_tx_available[i] = 1;
                    if (can_devs[i]) {
                        netif_wake_queue(can_devs[i]);
                    }

                    //If the put is successful, it is considered that the transmission is successful. Whether it is sent or not is not concerned
                    if(g_vcan_debug) {
                        g_debug_write_success_num[i]++;
                    }
                }
            }

            if (!data_left)
                break;
            usleep_range(ACTIVE_SEND_SLEEP_US, ACTIVE_SEND_SLEEP_US + 1);
        }
    }

    if(g_vcan_debug)
        printk("vcan_write_thread exit.\n");
    return 0;
}

switch0_can_gateway_sw_status_enum_t g_eType[16] = {
    SWITCH0_CAN_GATEWAY_SW_STATUS_CANIP_UNKOWN, SWITCH0_CAN_GATEWAY_SW_STATUS_CANIP_UNKOWN, SWITCH0_CAN_GATEWAY_SW_STATUS_CANIP_UNKOWN, SWITCH0_CAN_GATEWAY_SW_STATUS_CANIP_UNKOWN, SWITCH0_CAN_GATEWAY_SW_STATUS_CANIP_UNKOWN, SWITCH0_CAN_GATEWAY_SW_STATUS_CANIP_UNKOWN, SWITCH0_CAN_GATEWAY_SW_STATUS_CANIP_UNKOWN, SWITCH0_CAN_GATEWAY_SW_STATUS_CANIP_UNKOWN, SWITCH0_CAN_GATEWAY_SW_STATUS_CANIP_UNKOWN, SWITCH0_CAN_GATEWAY_SW_STATUS_CANIP_UNKOWN, SWITCH0_CAN_GATEWAY_SW_STATUS_CANIP_UNKOWN, SWITCH0_CAN_GATEWAY_SW_STATUS_CANIP_UNKOWN, SWITCH0_CAN_GATEWAY_SW_STATUS_CANIP_UNKOWN, SWITCH0_CAN_GATEWAY_SW_STATUS_CANIP_UNKOWN, SWITCH0_CAN_GATEWAY_SW_STATUS_CANIP_UNKOWN, SWITCH0_CAN_GATEWAY_SW_STATUS_CANIP_UNKOWN
};


bool g_b_set_read_priority = false;
static void vport_ipc_data_notify(const switch0_can_gateway_vcan_port_msg_t evt, void *ext,
				const ext_info_t *info)
{
    struct sk_buff* skb;
    sqe_proxy_t *sqb;
    CAN2X *can2x_data;
    CAN2X_PARAMS *can2x_param;
    u32 can_id;
    u32 bus_bitmap = 0;
#ifdef DEBUG_SCH_TIMEOUT_PRINT
    static uint64_t end_cb_time;
#endif
    uint32_t sch_delay;

    if (!g_b_set_read_priority)
    {
        struct sched_param sch_param;
        cpumask_t mask;

        // Set the scheduling policy to FIFO and set the priority
        sch_param.sched_priority = ADAS2CAN_RX_THREAD_PRI;
        if (sched_setscheduler(current, SCHED_FIFO, &sch_param) == -1) {
            printk("%s sched_setscheduler failed\n", __func__);
        }
        g_b_set_read_priority = true;

        cpumask_clear(&mask);
        if (cpu_online(ADAS2CAN_RX_CPU_CORE)) {
            cpumask_set_cpu(ADAS2CAN_RX_CPU_CORE, &mask);
        } else {
            cpumask_set_cpu(ADAS2CAN_RX_CPU_CORE - 4, &mask);
        }

        if (set_cpus_allowed_ptr(current, &mask)) {
            printk("%s select cpu mask:%*pbl failed.\r\n", __func__, cpumask_pr_args(&mask));
        }
    }

    if (g_vcan_debug) {
#ifdef DEBUG_SCH_TIMEOUT_PRINT
        static uint64_t last_print_time = 0;
        static uint8_t first_enter = 0;
#endif
        uint64_t irq_time;
        uint64_t time;

        irq_time = info->timestamp / 1000;
        time = ktime_get_raw() / 1000;
        sch_delay = time - irq_time;
#ifdef DEBUG_SCH_TIMEOUT_PRINT
        if (first_enter && (sch_delay > 1000) && (time - last_print_time > 1000000)) {
            trace_printk("----bstvcan:get cost over 1ms");
            printk("-----get cost %u us, irq time %llu us, cb time %llu us, end cb time %llu us",
                sch_delay, irq_time, time, end_cb_time);
            last_print_time = time;
        }
        first_enter = 1;
#endif
    }

    while (true) {
        sqb = sq_buffer_consume_get(ADAS2CAN_RX_SQB_INDEX);
        if (!sqb) {
            break;
        }

        can2x_data = sqb->buffer;
        can2x_param = (CAN2X_PARAMS*)can2x_data->param;
        /* invalid bus id */
        if (can2x_param->src_can_bus_id > g_vcan_virtual_dev[VIRTUAL_DEV_NUM-1] || 
            can2x_param->src_can_bus_id < g_vcan_virtual_dev[0]) {
            //pr_warn("recv can data from bus: %d, out of range vcan port, drop!\n", can2x_param->src_can_bus_id);
            sq_buffer_consume_put(sqb);
            continue;
        }
        /* is port opened? */
        if (!g_priv[can2x_param->src_can_bus_id]) {
            sq_buffer_consume_put(sqb);
            continue;
        }

        /* build skb */
        can_id = can2x_data->id;
        if (can2x_data->ide) {
            can_id |= CAN_EFF_FLAG;
        }
        if (can2x_data->rtr) {
            can_id |= CAN_RTR_FLAG;
        }
        if (can2x_data->esi) {
            can_id |= CAN_ERR_FLAG;
        }
        if (can2x_data->edl) {
            struct canfd_frame *cfd;
            skb = alloc_canfd_skb(g_priv[can2x_param->src_can_bus_id]->offload.dev, &cfd);
            if (!skb) {
                break;
            }
            cfd->can_id = can_id;
            cfd->len = can_fd_dlc2len(can2x_data->dlc);
            cfd->flags |= CANFD_FDF;
            if (can2x_data->brs)
            {
                cfd->flags |= CANFD_BRS;
            }
            for (int i = 0; i != cfd->len; i++)
            {
                cfd->data[i] = can2x_data->data[i];
            }
        } else {
            struct can_frame *cf;
            skb = alloc_can_skb(g_priv[can2x_param->src_can_bus_id]->offload.dev, &cf);
            if (!skb) {
                break;
            }
            //cf = (struct can_frame*)skb->data;
            cf->can_id = can_id;
            if (can2x_data->dlc > 8)
            {
                can2x_data->dlc = 8;
            }
            cf->len = can2x_data->dlc;
            for (int i = 0; i != can2x_data->dlc; i++)
            {
                cf->data[i] = can2x_data->data[i];
            }
        }

        /* perf debug */
        if (g_vcan_debug && can_id == VCAN_DEBUG_CAN_ID) {
            struct ptp_system_timestamp read_xgmac_ts_with_offset = {0};
            u64 read_delay;
            int ret = 0;

            ret = bst_get_ts_from_xgmac(NULL, &read_xgmac_ts_with_offset);
            if (ret) {
                read_xgmac_ts_with_offset.pre_ts.tv_sec = TIME64_MAX;
                read_xgmac_ts_with_offset.pre_ts.tv_nsec = 999999999L;
                printk("%s bst_get_ts_from_xgmac failed, ret = %d.\n", __func__, ret);
            }

            read_delay = ((read_xgmac_ts_with_offset.pre_ts.tv_sec) - (((u64)(can2x_param->gmac_timestamp_second_h)) << 32) - can2x_param->gmac_timestamp_second_l) * 1000000000
                           + (read_xgmac_ts_with_offset.pre_ts.tv_nsec - can2x_param->gmac_timestamp_nanosecond);

            if (read_delay > g_debug_read_max_delay[can2x_param->src_can_bus_id]) {
                g_debug_read_max_delay[can2x_param->src_can_bus_id] = read_delay;
            }
            if (read_delay < g_debug_read_min_delay[can2x_param->src_can_bus_id]) {
                g_debug_read_min_delay[can2x_param->src_can_bus_id] = read_delay;
            }
            if (sch_delay > g_debug_read_sch_max_delay[can2x_param->src_can_bus_id]) {
                g_debug_read_sch_max_delay[can2x_param->src_can_bus_id] = sch_delay;
            }

            if (g_vcan_debug & BIT(1)) {
                static struct timespec64 last_read_test_time = {0};
                static u32 last_gmac_timestamp_second_h = 0;
                static u32 last_gmac_timestamp_second_l = 0;
                static u32 last_gmac_timestamp_nanosecond = 0;

                // over 1s, something must be wrong!
                if (read_delay > 1000000000) {
                    printk("error delay:%lld, now:%lld.%ld, can time:%lld.%d, last now:%lld.%ld, last can:H%d,L%d.%d\r\n",
                            read_delay,
                            read_xgmac_ts_with_offset.pre_ts.tv_sec, read_xgmac_ts_with_offset.pre_ts.tv_nsec,
                            (((u64)(can2x_param->gmac_timestamp_second_h)) << 32) + can2x_param->gmac_timestamp_second_l,
                            can2x_param->gmac_timestamp_nanosecond, last_read_test_time.tv_sec,
                            last_read_test_time.tv_nsec, last_gmac_timestamp_second_h,
                            last_gmac_timestamp_second_l, last_gmac_timestamp_nanosecond);
                }
                last_read_test_time = read_xgmac_ts_with_offset.pre_ts;
                last_gmac_timestamp_second_h = can2x_param->gmac_timestamp_second_h;
                last_gmac_timestamp_second_l = can2x_param->gmac_timestamp_second_l;
                last_gmac_timestamp_nanosecond = can2x_param->gmac_timestamp_nanosecond;
            }
            g_debug_total_read_delay[can2x_param->src_can_bus_id] += read_delay;
            g_debug_read_num[can2x_param->src_can_bus_id]++;
        }
        sq_buffer_consume_put(sqb);

        if (skb) {
            can_rx_offload_queue_tail(&g_priv[can2x_param->src_can_bus_id]->offload, skb);
            bus_bitmap |= 1 << can2x_param->src_can_bus_id;
        }
    }

    for (; bus_bitmap != 0; bus_bitmap &= bus_bitmap - 1) {
        unsigned int i = __builtin_ctz(bus_bitmap);
        can_rx_offload_irq_finish(&g_priv[i]->offload);
    }

#ifdef DEBUG_SCH_TIMEOUT_PRINT
	end_cb_time = ktime_get_raw() / 1000;
#endif
}


static void bstvcan_irq_bus_err(struct net_device *dev, u32 reg_esr)
{
    struct bstvcan_priv *priv = netdev_priv(dev);
    struct sk_buff *skb;
    struct can_frame *cf;
    bool rx_errors = false, tx_errors = false;
    skb = alloc_can_err_skb(dev, &cf);
    if (unlikely(!skb))
        return;

    cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;

    if (reg_esr & FLEXCAN_ESR_BIT1_ERR) {
        netdev_dbg(dev, "BIT1_ERR irq\n");
        cf->data[2] |= CAN_ERR_PROT_BIT1;
        tx_errors = true;
    }
    if (reg_esr & FLEXCAN_ESR_BIT0_ERR) {
        netdev_dbg(dev, "BIT0_ERR irq\n");
        cf->data[2] |= CAN_ERR_PROT_BIT0;
        tx_errors = true;
    }
    if (reg_esr & FLEXCAN_ESR_ACK_ERR) {
        netdev_dbg(dev, "ACK_ERR irq\n");
        cf->can_id |= CAN_ERR_ACK;
        cf->data[3] = CAN_ERR_PROT_LOC_ACK;
        tx_errors = true;
    }
    if (reg_esr & FLEXCAN_ESR_CRC_ERR) {
        netdev_dbg(dev, "CRC_ERR irq\n");
        cf->data[2] |= CAN_ERR_PROT_BIT;
        cf->data[3] = CAN_ERR_PROT_LOC_CRC_SEQ;
        rx_errors = true;
    }
    if (reg_esr & FLEXCAN_ESR_FRM_ERR) {
        netdev_dbg(dev, "FRM_ERR irq\n");
        cf->data[2] |= CAN_ERR_PROT_FORM;
        rx_errors = true;
    }
    if (reg_esr & FLEXCAN_ESR_STF_ERR) {
        netdev_dbg(dev, "STF_ERR irq\n");
        cf->data[2] |= CAN_ERR_PROT_STUFF;
        rx_errors = true;
    }

    priv->can.can_stats.bus_error++;
    if (rx_errors)
        dev->stats.rx_errors++;
    if (tx_errors)
        dev->stats.tx_errors++;

    if (skb_queue_len(&priv->offload.skb_queue) >
        priv->offload.skb_queue_len_max) {
        dev_kfree_skb_any(skb);
        dev->stats.rx_fifo_errors++;
    } else {
        __skb_queue_tail(&priv->offload.skb_irq_queue, skb);
        can_rx_offload_irq_finish(&priv->offload);
    }
}

static int __bstvcan_get_berr_counter(u32 ecr,
                      struct can_berr_counter *bec)
{
    bec->txerr = (ecr >> 0) & 0xff;
    bec->rxerr = (ecr >> 8) & 0xff;

    return 0;
}

static void bstvcan_irq_state(struct net_device *dev, u32 reg_esr, u32 ecr)
{
    struct bstvcan_priv *priv = netdev_priv(dev);

    struct sk_buff *skb;
    struct can_frame *cf;
    enum can_state new_state, rx_state, tx_state;
    int flt;
    struct can_berr_counter bec;

    flt = reg_esr & FLEXCAN_ESR_FLT_CONF_MASK;
    if (likely(flt == FLEXCAN_ESR_FLT_CONF_ACTIVE)) {
        tx_state = unlikely(reg_esr & FLEXCAN_ESR_TX_WRN) ?
                   CAN_STATE_ERROR_WARNING :
                   CAN_STATE_ERROR_ACTIVE;
        rx_state = unlikely(reg_esr & FLEXCAN_ESR_RX_WRN) ?
                   CAN_STATE_ERROR_WARNING :
                   CAN_STATE_ERROR_ACTIVE;
        new_state = max(tx_state, rx_state);
    } else {
        __bstvcan_get_berr_counter(ecr, &bec);
        new_state = flt == FLEXCAN_ESR_FLT_CONF_PASSIVE ?
                    CAN_STATE_ERROR_PASSIVE :
                    CAN_STATE_BUS_OFF;
        rx_state = bec.rxerr >= bec.txerr ? new_state : 0;
        tx_state = bec.rxerr <= bec.txerr ? new_state : 0;
    }

    /* state hasn't changed */
    if (likely(new_state == priv->can.state))
        return;

    skb = alloc_can_err_skb(dev, &cf);
    if (unlikely(!skb))
        return;

    can_change_state(dev, cf, tx_state, rx_state);

    if (unlikely(new_state == CAN_STATE_BUS_OFF))
        can_bus_off(dev);
    
    if (skb_queue_len(&priv->offload.skb_queue) >
        priv->offload.skb_queue_len_max) 
    {
        dev_kfree_skb_any(skb);
        dev->stats.rx_fifo_errors++;
    }
    else {
        __skb_queue_tail(&priv->offload.skb_irq_queue, skb);
        can_rx_offload_irq_finish(&priv->offload);
    }
}


void vcan_status_cb(const switch0_can_gateway_can_status_t *status_info,
            void *ext, const ext_info_t *info)
{
    unsigned int reg_esr, reg_ecr;
    int bus_id = -1;
    switch0_can_gateway_sw_status_enum_t sw_status_type;

    if (!status_info)
        return;

    bus_id = (int)status_info->can_bus_id;
    if (bus_id < MIN_CAN_CHANNEL ||
        bus_id > MAX_CAN_CHANNEL) {
        return;
    }

    sw_status_type =
            (switch0_can_gateway_sw_status_enum_t)
                status_info->sw_status;
    if (g_eType[bus_id] != sw_status_type ||
            sw_status_type == SWITCH0_CAN_GATEWAY_SW_STATUS_OK) {
            g_eType[bus_id] = sw_status_type;
            switch (sw_status_type) {
            case SWITCH0_CAN_GATEWAY_SW_STATUS_OK:
            if (g_vcan_debug)
            {
                if (can_devs[bus_id])
                {
                    printk("get status form can%d , status: ok, reg_esr1:%d, reg_ecr:%d \n",
                        (int)status_info->can_bus_id,
                        status_info->reg_esr1,
                        status_info->reg_ecr);
                }
            }
                break;
            case SWITCH0_CAN_GATEWAY_SW_STATUS_CANIP_UNINIT:
                if (can_devs[bus_id])
                {
                    printk("get status form can%d , status: uninit, reg_esr1:%d, reg_ecr:%d \n",
                        (int)status_info->can_bus_id,
                        status_info->reg_esr1,
                        status_info->reg_ecr);
                }
                break;
            case SWITCH0_CAN_GATEWAY_SW_STATUS_CANIP_DISABLED:
                if (can_devs[bus_id])
                {
                    printk("get status form can%d , status: disable, reg_esr1:%d, reg_ecr:%d \n",
                        (int)status_info->can_bus_id,
                        status_info->reg_esr1,
                        status_info->reg_ecr);
                }
                break;
            case SWITCH0_CAN_GATEWAY_SW_STATUS_CANIP_UNKOWN:
                if (can_devs[bus_id])
                {
                    printk("get status form can%d , status: unkown, reg_esr1:%d, reg_ecr:%d \n",
                        (int)status_info->can_bus_id,
                        status_info->reg_esr1,
                        status_info->reg_ecr);
                }
                break;
            }
        }
    if (!can_devs[bus_id]) {
        return;
    }
    reg_esr = status_info->reg_esr1;
    reg_ecr = status_info->reg_ecr;

    /* state change interrupt or broken error state quirk fix is enabled */
    if (reg_esr & FLEXCAN_ESR_ERR_STATE)
        bstvcan_irq_state(can_devs[bus_id], reg_esr, reg_ecr);

    /* bus error IRQ - handle if bus error reporting is activated */
    if (reg_esr & FLEXCAN_ESR_ERR_BUS)
        bstvcan_irq_bus_err(can_devs[bus_id], reg_esr);
     
}

void vcan_sub_err_cb(int32_t err, void* ext,
    const ext_info_t* info)
{
    printk("%s\n", __func__);
}

void vcan_status_unsub_cb(int32_t err, void *ext, const ext_info_t *info)
{
    printk("%s\n", __func__);
}

void vcan_notify_sqbuf_addr_cb(switch0_can_gateway_ErrorEnum_t err, void *ext,
                     const ext_info_t *info)
{
    printk("%s\n", __func__);
}

void vcan_config_cb(
    const switch0_can_gateway_can_config_t* config_info,
    const switch0_can_gateway_ErrorEnum_t err,
    void* ext,
    const ext_info_t* info
)
{
    struct bstvcan_priv* priv = ext; 
    struct can_bittiming *bt = &priv->can.bittiming;
    struct can_bittiming *dbt = &priv->can.data_bittiming;
    printk("can%d can_config_query: fd:%d, nominal_baudrate:%d, data_baudrate:%d.\n",
        (int)config_info->can_bus_id, (int)config_info->canfd_enable, config_info->nominal_baudrate,
        config_info->data_baudrate);
    
    priv->can.bittiming.bitrate = config_info->nominal_baudrate;
    priv->can.data_bittiming.bitrate = config_info->data_baudrate;
    bt->prop_seg = config_info->nominal_timing.propseg;
    bt->phase_seg1 = config_info->nominal_timing.phaseseg1;
    bt->phase_seg2 = config_info->nominal_timing.phaseseg2;
    bt->tq = 1 + bt->prop_seg + bt->phase_seg1 + bt->phase_seg2;
    bt->sample_point = (1 + bt->prop_seg + bt->phase_seg1) * 1000 / bt->tq;
    dbt->prop_seg = config_info->data_timing.propseg;
    dbt->phase_seg1 = config_info->data_timing.phaseseg1;
    dbt->phase_seg2 = config_info->data_timing.phaseseg2;
    dbt->tq = 1 + dbt->prop_seg + dbt->phase_seg1 + dbt->phase_seg2;
    dbt->sample_point = (1 + dbt->prop_seg + dbt->phase_seg1) * 1000 / dbt->tq;
    
}

static int vport_sqbuffer_init(uint64_t sq_buffer_virtual_addr, uint64_t phy_addr, uint32_t smmu_vaddr)
{
    int err = 0;
    SQ_Buffer *adas2can_sqbuffer = NULL;
    SQ_Buffer *can2adas_sqbuffer = NULL;
    uint64_t adas2can_data_phy_addr = phy_addr + ADAS2CAN_SQBUFFER_HEADER_RESERVER_SIZE;
    uint64_t can2adas_data_phy_addr = phy_addr + ADAS2CAN_BUFFER_SIZE + ADAS2CAN_SQBUFFER_HEADER_RESERVER_SIZE;
    uint32_t adas2can_data_smmu_vaddr = smmu_vaddr + ADAS2CAN_SQBUFFER_HEADER_RESERVER_SIZE;
    uint32_t can2adas_data_smmu_vaddr = smmu_vaddr + ADAS2CAN_BUFFER_SIZE + ADAS2CAN_SQBUFFER_HEADER_RESERVER_SIZE;
    uint32_t sq_buffer_addr[ADAS_VCAN_BUFFER_SQ_BUFFER_COUNT];

    printk("sqbuffer va: 0x%llx, pa: 0x%llx, smmu_vaddr: 0x%x\n", sq_buffer_virtual_addr, phy_addr, smmu_vaddr);

    adas2can_sqbuffer = (SQ_Buffer *)sq_buffer_virtual_addr;
    can2adas_sqbuffer = (SQ_Buffer *)(sq_buffer_virtual_addr + ADAS2CAN_BUFFER_SIZE);

    for(uint8_t i=0; i<ADAS_VCAN_BUFFER_SQ_BUFFER_COUNT; i++) {
        adas2can_sqbuffer->buffers_phy_addr64[i] = adas2can_data_phy_addr + (sizeof(X2CAN) * i);
        can2adas_sqbuffer->buffers_phy_addr64[i] = can2adas_data_phy_addr + (sizeof(CAN2X) * i);
    }

    for(uint8_t i=0; i<ADAS_VCAN_BUFFER_SQ_BUFFER_COUNT; i++) {
        sq_buffer_addr[i] = adas2can_data_smmu_vaddr + (sizeof(X2CAN) * i);
    }
    err = SQ_Buffer_Init(adas2can_sqbuffer, sq_buffer_addr, sizeof(X2CAN), ADAS_VCAN_BUFFER_SQ_BUFFER_COUNT);
    if (err)
        return err;
    
    for(uint8_t i=0; i<ADAS_VCAN_BUFFER_SQ_BUFFER_COUNT; i++) {
        sq_buffer_addr[i] = can2adas_data_smmu_vaddr + (sizeof(CAN2X) * i);
    }
    err = SQ_Buffer_Init(can2adas_sqbuffer, sq_buffer_addr, sizeof(CAN2X), ADAS_VCAN_BUFFER_SQ_BUFFER_COUNT);
    if (err)
        return err;
    return 0;
}

static int vport_open(void)
{
    int err = 0;
    uint64_t sq_buffer_addr = 0;
    uint64_t sq_buffer_virtual_addr = 0;
    uint32_t adas2can_smmu_va = 0;
    uint32_t can2adas_smmu_va = 0;
    VCan_PortOpenWithBufferReq open_req_content = {0};
    VCan_PortOpenWithBufferRsp *open_rsq_content = NULL;
    VCan_PortDdrSmmuAllocReq smmu_req_content = {0};
    VCan_PortDdrSmmuAllocRsp *smmu_rsp_content = NULL;
    switch0_can_gateway_vcan_port_msg_t vport_method_req = {0};
    switch0_can_gateway_vcan_port_msg_t vport_mtehod_rsp = {0};
    switch0_can_gateway_ErrorEnum_t vport_method_err = SWITCH0_CAN_GATEWAY_NO_ERROR;
    //init sqbuffer
    sq_buffer_addr = sq_buffer_get_base_paddr();
    sq_buffer_virtual_addr = sq_buffer_get_base_vaddr();

    g_sq_buffer_phyaddr = sq_buffer_addr;

    //SMMU IPC is sent to obtain the virtual address that sw0 can access
    //ADAS2CAN
    smmu_req_content.ddr_addr_H = sq_buffer_addr >> 32;
    smmu_req_content.ddr_addr_L = sq_buffer_addr;
    smmu_req_content.ddr_size = ADAS2CAN_BUFFER_SIZE * 2;
    vport_method_req.cmd_id = VCAN_CMD_PORT_DDR_SMMU_REQ;
    vport_method_req.msg_len = sizeof(smmu_req_content);
    vport_method_req.content.data = (uint32_t *)&smmu_req_content;
    vport_method_req.content.size = sizeof(smmu_req_content)/sizeof(uint32_t);
    err = g_vcan_client->switch0_can_gateway_client.vcan_port_method_sync(vport_method_req, &vport_mtehod_rsp, 
        &vport_method_err, 500, NULL);
    if (err) {
        printk("VCAN_CMD_PORT_DDR_SMMU_REQ ipc failed %d.\n",
            err);
        return -1;
    }
    smmu_rsp_content = (VCan_PortDdrSmmuAllocRsp *)vport_mtehod_rsp.content.data;
    if (smmu_rsp_content->err) {
        printk("VCAN_CMD_PORT_DDR_SMMU_REQ failed %d.\n",
            err);
        return -1;
    }
    adas2can_smmu_va = smmu_rsp_content->smm_va;
    printk("adas2can_smmu_va: 0x%x\n", adas2can_smmu_va);

    //CAN2ADAS
    can2adas_smmu_va = adas2can_smmu_va + ADAS2CAN_BUFFER_SIZE;
    printk("can2adas_smmu_va: 0x%x\n", can2adas_smmu_va);

    err = vport_sqbuffer_init(sq_buffer_virtual_addr, sq_buffer_addr, adas2can_smmu_va);
    if(err) {
        printk("vport_sqbuffer_init failed %d.\n",
            err);
        return -1;
    }

    // Call IPC to open the virtual port
    open_req_content.port_id = VIRTUAL_PORT_ID;
    open_req_content.rx_notify = 1;
    open_req_content.rx_method_type = VCAN_RX_METHOD_TYPE_CLIENT_SQB;
    open_req_content.rx_sqb_addr = can2adas_smmu_va;
    open_req_content.rx_sqb_index = ADAS2CAN_RX_SQB_INDEX;

    open_req_content.tx_method_type = VCAN_TX_METHOD_TYPE_SQB;
    open_req_content.tx_sqb_addr = adas2can_smmu_va;
    open_req_content.tx_sqb_index = ADAS2CAN_TX_SQB_INDEX;

    vport_method_req.cmd_id = VCAN_CMD_PORT_OPEN_EXT_REQ;
    vport_method_req.msg_len = sizeof(open_req_content);
    vport_method_req.content.data = (uint32_t *)&open_req_content;
    vport_method_req.content.size = sizeof(open_req_content)/sizeof(uint32_t);
    err = g_vcan_client->switch0_can_gateway_client.vcan_port_method_sync(vport_method_req, &vport_mtehod_rsp, 
        &vport_method_err, 500, NULL);
    if (err) {
        printk("VCAN_CMD_PORT_OPEN_EXT_REQ ipc failed %d.\n",
            err);
        return -1;
    }
    open_rsq_content = (VCan_PortOpenWithBufferRsp *)vport_mtehod_rsp.content.data;
    if (open_rsq_content->err) {
        printk("VCan_PortOpenWithBufferRsp failed %d.\n",
            err);
        return -1;
    }
    return err;
}

static int vport_close(void)
{
    int err = 0;
    VCan_PortCloseReq close_req_content = {0};
    VCan_PortCloseRsp *close_rsp_content = NULL;
    VCan_PortDdrSmmuFreeReq smmufree_req_content = {0};
    VCan_PortDdrSmmuFreeRsp *smmufree_rsp_content = NULL;
    switch0_can_gateway_vcan_port_msg_t vport_method_req = {0};
    switch0_can_gateway_vcan_port_msg_t vport_mtehod_rsp = {0};
    switch0_can_gateway_ErrorEnum_t vport_method_err = SWITCH0_CAN_GATEWAY_NO_ERROR;

    // Call IPC to close the virtual port
    close_req_content.port_id = VIRTUAL_PORT_ID;
    vport_method_req.cmd_id = VCAN_CMD_PORT_CLOSE_REQ;
    vport_method_req.msg_len = sizeof(close_req_content);
    vport_method_req.content.data = (uint32_t *)&close_req_content;
    vport_method_req.content.size = sizeof(close_req_content)/sizeof(uint32_t);
    err = g_vcan_client->switch0_can_gateway_client.vcan_port_method_sync(vport_method_req, &vport_mtehod_rsp, 
        &vport_method_err, 500, NULL);
    if (err) {
        printk("VCAN_CMD_PORT_CLOSE_REQ ipc failed %d.\n",
            err);
        return -1;
    }
    close_rsp_content = (VCan_PortCloseRsp *)vport_mtehod_rsp.content.data;
    if (close_rsp_content->err) {
        printk("VCAN_CMD_PORT_CLOSE_REQ failed %d.\n",
            err);
        return -1;
    }

    // Call IPC to cancel smmu mapping
    smmufree_req_content.ddr_addr_H = g_sq_buffer_phyaddr >> 32;
    smmufree_req_content.ddr_addr_L = g_sq_buffer_phyaddr;
    smmufree_req_content.ddr_size = ADAS2CAN_BUFFER_SIZE * 2;
    vport_method_req.cmd_id = VCAN_CMD_PORT_DDR_SMMU_FREE;
    vport_method_req.msg_len = sizeof(smmufree_req_content);
    vport_method_req.content.data = (uint32_t *)&smmufree_req_content;
    vport_method_req.content.size = sizeof(smmufree_req_content)/sizeof(uint32_t);
    err = g_vcan_client->switch0_can_gateway_client.vcan_port_method_sync(vport_method_req, &vport_mtehod_rsp, 
        &vport_method_err, 500, NULL);
    if (err) {
        printk("VCAN_CMD_PORT_DDR_SMMU_FREE ipc failed %d.\n",
            err);
        return -1;
    }
    smmufree_rsp_content = (VCan_PortDdrSmmuFreeRsp *)vport_mtehod_rsp.content.data;
    if (smmufree_rsp_content->err) {
        printk("VCAN_CMD_PORT_DDR_SMMU_FREE failed %d.\n",
            err);
        return -1;
    }

    return err;
}

static int vcan_init(void* unused)
{
    int err = 0;
    int timeout_count = 0;

    // sw0 restarts and sleeps 1 second until the server is ready
    if (vcan_server_restart) {
        msleep(1000);
    }

    g_vcan_client = adas5_can_gateway_client_init(&g_vcan_client_data);
    err = g_vcan_client->switch0_can_gateway_client.register_avail_changed(on_vcan_dst_changed, &b_vcan_dst_avail);
    if (err < 0)
    {
        printk("vcan service: register_avail_changed failed\n");
        return -EPERM;
    }
    g_vcan_client->start();
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

    if (!g_vport_open)
    {
        err = vport_open();
        if (err) {
            printk("vport_open failed %d.\n",
                err);
            return -1;
        }
        g_vport_open = true;
    }

    err = g_vcan_client->switch0_can_gateway_client
            .vcan_port_broadcast_sub(vport_ipc_data_notify, NULL,
                            NULL, NULL, NULL);
    if (err) {
        printk("vcan_port_broadcast_sub failed %d.\n",
                err);
        return -1;
    }
    err = g_vcan_client->switch0_can_gateway_client
                .can_status_notify_sub(vcan_status_cb, NULL, NULL,
                            vcan_status_unsub_cb,
                            NULL);
    if (err) {
        printk("can_status_notify_sub failed %d.\n", err);
        return -1;
    }
    atomic_set(&g_vcan_init, 1);

    return 0;
}

//Universal callback
static int bstvcan_open(struct net_device* dev)
{
    struct bstvcan_priv* priv = netdev_priv(dev);
    int dev_id = can_name_to_controller_id(dev->name);
    int err = 0;
    //int timeout_count = 0;
    priv->can.state = CAN_STATE_ERROR_ACTIVE;
    priv->can.bittiming.bitrate = CAN_BITRATE_UNKNOWN;
    priv->can.data_bittiming.bitrate = CAN_BITRATE_UNKNOWN;

    if (!g_thread_init)
    {
        g_thread_init = true;
        init_completion(&g_vcan_write_event);
        g_p_vcan_write_thread = kthread_create(do_vcan_write_thread, NULL, "vcan_write_thread");
        if (!IS_ERR(g_p_vcan_write_thread))
        {
            wake_up_process(g_p_vcan_write_thread);
        }
        else
        {
            printk("vcan_write_thread create failed\n");
            g_thread_init = false;
            return -ENOMEM;
        }
        
        err = vcan_init(NULL);
        if (err) {
            printk("vcan_init failed %d.\n", err);
            return -ENOMEM;
        }
    }
    
    err = g_vcan_client->switch0_can_gateway_client.can_config_query_async(
        dev_id, vcan_config_cb, priv, NULL);
    if (err) {
        printk("can_config_query_async failed %d.\n", err);
    } 
    //canfd is supported by default
    dev->mtu = CANFD_MTU;
    dev->tx_queue_len = 1000;

    err = open_candev(dev);
    if (err)
    {
        printk("open_candev %s failed.\n", dev->name);
        return err;
    }
    g_any_can_opened = true;

    err = can_rx_offload_add_manual(dev, &priv->offload, VCAN_NAPI_WEIGHT);
    can_rx_offload_enable(&priv->offload);
    netif_start_queue(dev);

    can_devs[dev_id] = dev;
    g_priv[dev_id] = priv;
    return 0;
}

static int bstvcan_close(struct net_device* dev)
{
    struct bstvcan_priv* priv = NULL;
    int dev_id = 0;

    priv = netdev_priv(dev);
    dev_id = can_name_to_controller_id(dev->name);
    priv->can.state = CAN_STATE_STOPPED;
    if (g_vcan_debug)
    {
        printk("can%d close. read num:%d, write num:%d, write success num:%d.\n read avg delay:%llu, max delay:%llu, min delay:%llu, sch max delay(us):%u.\n",
            dev_id, g_debug_read_num[dev_id], g_debug_write_num[dev_id], g_debug_write_success_num[dev_id],
            g_debug_total_read_delay[dev_id] / g_debug_read_num[dev_id], g_debug_read_max_delay[dev_id],
            g_debug_read_min_delay[dev_id], g_debug_read_sch_max_delay[dev_id]);
        g_debug_read_num[dev_id] = 0;
        g_debug_write_num[dev_id] = 0;

        g_debug_write_success_num[dev_id] = 0;

        g_debug_total_read_delay[dev_id] = 0;
        g_debug_read_max_delay[dev_id] = 0;
        g_debug_read_min_delay[dev_id] = 100000000;
        g_debug_read_sch_max_delay[dev_id] = 0;
    }

    g_priv[dev_id] = NULL;
    can_devs[dev_id] = NULL;
    g_vcan_tx_available[dev_id] = 1;
    g_vcan_write_flag[dev_id] = false;
    can_rx_offload_disable(&priv->offload);
    can_rx_offload_del(&priv->offload);
    close_candev(dev);

    for (int i = MIN_CAN_CHANNEL; i != MAX_CAN_CHANNEL + 1; i++)
    {
        if (g_priv[i])
        {
            return 0;
        }
    }
    g_any_can_opened = false;
    return 0;
}

static const struct net_device_ops bstvcan_netdev_ops = {
    .ndo_open = bstvcan_open,
    .ndo_stop = bstvcan_close,
    .ndo_start_xmit = bstvcan_start_xmit,
    .ndo_change_mtu = can_change_mtu,
};

static const struct of_device_id bstvcan_of_match[] = {
    {.compatible = "bst,bst-flexcan"},
    { /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, bstvcan_of_match);

static int bstvcan_set_bittiming(struct net_device *dev)
{
    return -EOPNOTSUPP; 
}
static int bstvcan_set_data_bittiming(struct net_device *dev)
{
    return -EOPNOTSUPP;
}

static int register_can_dev(int id, struct platform_device* pdev)
{
    if (!g_vcan_reg_flag[id])
    {
        struct net_device* dev;
        struct bstvcan_priv* priv;
        struct can_priv *can;
        int err;
        dev = alloc_candev(sizeof(struct bstvcan_priv), 1);
        if (!dev)
            return -ENOMEM;

        platform_set_drvdata(pdev, dev);

        dev->netdev_ops = &bstvcan_netdev_ops;
        priv = netdev_priv(dev);
        can = &priv->can; 
        can->bittiming.bitrate = 0; 
        can->data_bittiming.bitrate = 0;
        priv->dev = &pdev->dev;
        priv->can.do_set_bittiming = bstvcan_set_bittiming;
        priv->can.do_set_data_bittiming = bstvcan_set_data_bittiming;
        priv->can.bittiming_const = NULL;
        priv->can.data_bittiming_const = NULL;
        if (!priv->can.clock.freq)
            priv->can.clock.freq = 80000000;
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

static int bstvcan_probe(struct platform_device* pdev)
{
    for (int i = 0; i < VIRTUAL_DEV_NUM; i++)
    {
        register_can_dev(g_vcan_virtual_dev[i], pdev);
    }

    g_debugfs_dir = debugfs_create_dir("bst_vcan", NULL);
    debugfs_create_u64("debug", 0644, g_debugfs_dir, &g_vcan_debug);
    return 0;
}

static void bstvcan_shutdown(struct platform_device* pdev)
{
    device_set_wakeup_enable(&pdev->dev, false);
    device_set_wakeup_capable(&pdev->dev, false);
    b_vcan_dst_avail = false;
    if (g_vcan_client) {
        if (g_vport_open) {
            int ret = vport_close();
            if (ret)
                dev_err(&pdev->dev, "WARNING! Vport close failed!\n");
        }

        adas5_can_gateway_client_destroy();
    }
    debugfs_remove_recursive(g_debugfs_dir);
}


static int __maybe_unused bstvcan_suspend(struct device* device)
{
    struct net_device *dev = dev_get_drvdata(device);
    struct bstvcan_priv *priv = netdev_priv(dev);
    printk("bstvcan_suspend \n");
    priv->can.state = CAN_STATE_SLEEPING;
    return 0;
}

static int __maybe_unused bstvcan_resume(struct device* device)
{
    struct net_device *dev = dev_get_drvdata(device);
    struct bstvcan_priv *priv = netdev_priv(dev);
    printk("bstvcan_resume \n");
    priv->can.state = CAN_STATE_ERROR_ACTIVE;
    return 0;
}

static int __maybe_unused bstvcan_runtime_suspend(struct device* device)
{
    printk("bstvcan_runtime_suspend \n");
    return 0;
}

static int __maybe_unused bstvcan_runtime_resume(struct device* device)
{
    printk("bstvcan_runtime_resume \n");
    return 0;
}
static int __maybe_unused bstvcan_noirq_suspend(struct device* device)
{
    printk("bstvcan_noirq_suspend \n");
    return 0;
}

static int __maybe_unused bstvcan_noirq_resume(struct device* device)
{
    printk("bstvcan_noirq_resume \n");
    return 0;
}


static const struct dev_pm_ops bstvcan_pm_ops = {
    SET_SYSTEM_SLEEP_PM_OPS(bstvcan_suspend, bstvcan_resume)
    SET_RUNTIME_PM_OPS(bstvcan_runtime_suspend, bstvcan_runtime_resume, NULL)
    SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(bstvcan_noirq_suspend, bstvcan_noirq_resume)
};

static struct platform_driver bstvcan_driver = {
    .driver = {
        .name = DRV_NAME,
        .pm = &bstvcan_pm_ops,
        .of_match_table = bstvcan_of_match,
    },
    .probe = bstvcan_probe,
    .shutdown = bstvcan_shutdown
};

module_platform_driver(bstvcan_driver);

MODULE_DESCRIPTION("BST VCAN driver");
MODULE_AUTHOR("BST Ltd.");
MODULE_LICENSE("GPL v2");
