/*
 * fp_net_driver.h
 *
 * SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (C)2024Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/interrupt.h>
#include "fp_hif_reg.h"

#ifndef _fp_net_driver_
#define _fp_net_driver_ 

#define NUM_TOTAL_HIF            1
//#define NUM_HIF_CHANNELS         64
#if defined(CONFIG_BST_C1200_DB)
#define NUM_HIF_CHANNELS        9
#define HIF_CHAN_START          8
#elif defined(CONFIG_BST_C1200_IVI)
#define NUM_HIF_CHANNELS        17
#define HIF_CHAN_START          16
#endif
#define NUM_MAX_HIF_CHANNELS        64
#define HIF_CHANNELS_PER_IRQ        8
#define MAX_HIF_JUMBO_FRAME_SIZE    9600
#define HIF_MAX_IRQ     8
#define HIF_RESOURCE_NAME "bstvmachif"
#define MAX_SEND_FRAME_NUM			2000
#define HIF_HEADROOM				32

typedef enum {
    DB_MAC_ADDR_FLAG = 0x1,
    IVI_MAC_ADDR_FLAG = 0x2,
    DB_HIF_REINIT_FLAG = 0x101,
	IVI_HIF_REINIT_FLAG = 0x102,
} fp_msgbox_type_t;

struct hif_resources {
	void __iomem *addr;
	u8 mac[ETH_ALEN];
	int irq[HIF_MAX_IRQ];
};

struct plat_hif_data {
	u64 hif_channel_bit;
	u32 hif_max_mtu;
};

struct hif_chan {
    struct bd_tx_ring *txring;
    struct bd_rx_ring *rxring;
//    struct tasklet_struct tx_tasklet;
//    struct tasklet_struct rx_tasklet;
};

struct hif_base {
    struct hif_regs *regs;
//    channel_reg_t *ch_r[NUM_HIF_CHANNELS];
    struct channel_reg *ch_r[NUM_HIF_CHANNELS];
    struct hif_chan ch[NUM_HIF_CHANNELS];
};


struct hif_channel {
	struct napi_struct napi ____cacheline_aligned_in_smp;
	u32 index;
	volatile u32 hif_int_src;
	volatile u32 offset;
	int tx_clean_limit;
	unsigned int start;
	unsigned int end;
	unsigned int irq;
	DECLARE_BITMAP(_mask, HIF_CHANNELS_PER_IRQ);
	struct fp_private *fp;
};

enum hifvmac_state {
	HIFVMAC_DOWN,
	HIFVMAC_RESET_REQUESTED,
	HIFVMAC_RESETTING,
	HIFVMAC_SERVICE_SCHED,
	HIFVMAC_RUNING,
	HIFVMAC_TX_FIFO_CLEAR,
	HIFVMAC_TX_FIFO_CLEARED,
	HIFVMAC_RX_FIFO_CLEAR,
	HIFVMAC_RX_FIFO_CLEARED,
	HIFVMAC_NOE_EXEC_SW_RESET,
};

struct fp_private {
	unsigned char *mem_base; /* Memory mapped physical address */
	//    volatile struct hif_regs *mmio_addr; /* Memory mapped physical address */
	struct device  *device;     
	struct net_device *dev;
	struct plat_hif_data *plat;
	spinlock_t lock;                     /* Spin lock flag */
	spinlock_t tx_clean_lock;                     /* Spin lock flag */
	spinlock_t rx_lock;                     /* Spin lock flag */
	struct hif_base  hif[NUM_TOTAL_HIF];
	struct hif_channel channel[HIF_MAX_IRQ];
	DECLARE_BITMAP(_channel_mask, NUM_MAX_HIF_CHANNELS);
	unsigned int q_num;
	uint32_t txq_stopped;
	struct net_device_stats netstats;
	uint32_t alloc_rx_buff_failed;
	unsigned int   reset_bd;
	unsigned int safety_reset_with_failstop;
	unsigned int safety_reset_without_failstop;	
	unsigned char *ddr_va;
	unsigned int   ddr_pa;
	unsigned char *bmusec_buf_va;
	unsigned int   bmusec_buf_pa;
	unsigned char *llm_buf_va;
	unsigned int   llm_buf_pa;
#ifdef HASH_ARR_EN
	unsigned char *ddr_single_va;
	unsigned int   ddr_single_pa;
#endif /*End of HASH_ARR_EN*/
	struct work_struct service_task;
	struct workqueue_struct *wdt_wq;
	unsigned long state;
	bool tx_fifo_clear;
	bool rx_fifo_clear;
};

//#define MIN_FRAME_SIZE 46

//#define MAX_BUF_ALLOCSIZE 2048

#ifdef DEBUG
#define DPRINTK(lvl, fmt, args...) printk(lvl fmt, ##args)
#else
#define DPRINTK(lvl, fmt, args...)
#endif

#define RX_LOOP_THRESH 16
#define ENABLE_HIF_INTERRUPTS 1
#define NFP_IFS  1
extern struct fp_private *fpGlobal;
irqreturn_t fp_interrupt(int irq, void *dev);
void fp_en_hif_rx_engine(struct fp_private *fp);
void fp_disable_interrupts(struct fp_private *fp);
int fp_hw_sw_init(void);
int fp_net_release(struct net_device *dev);
extern const struct dev_pm_ops hif_pltfr_pm_ops;
extern const struct net_device_ops fp_netdev_ops;
extern struct ethtool_ops fp_ethtool_ops;
void fp_enable_interrupts(struct fp_private *fp);
struct net_device * fp_netdev_init(struct platform_device *pdev,
		struct plat_hif_data *plat_dat,
		struct hif_resources *res);
int hif_dvr_remove(struct device *dev);

#endif /* _fp_net_driver_*/
