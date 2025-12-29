// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021-2024 Black Sesame Technologies. All Rights Reserved.
 * Copyright (C) 2007-2011 STMicroelectronics Ltd
 */

#ifndef __BSTVMAC_H__
#define __BSTVMAC_H__

#include "common.h"

#define BSTVMAC_HEADROOM			32
#define BSTVMAC_TX_HDRLEN			16
#define BSTVMAC_CHANNELS_PER_IRQ	8
#define BSTVMAC_MAX_IRQ_NUM     	8
#define BSTVMAC_SEND_FRAME_NUM		2048
#define BSTVMAC_RESOURCE_NAME   "bstvmaceth"
#define DRV_MODULE_VERSION		"Aug_2024"

struct bstvmac_resources {
	void __iomem *addr;
	u8 mac[ETH_ALEN];
	int perch_irq[BSTVMAC_MAX_IRQ_NUM];
	int irq;
	u64 hif_channel_bit;
};

enum bstvmac_txbuf_type {
	BSTVMAC_TXBUF_T_SKB,
	BSTVMAC_TXBUF_T_XDP_TX,
	BSTVMAC_TXBUF_T_XDP_NDO,
	BSTVMAC_TXBUF_T_XSK_TX,
};

typedef enum {
    DB_MAC_ADDR_FLAG = 0x1,
    IVI_MAC_ADDR_FLAG = 0x2,
    DB_MAC_STOP_FLAG = 0x10,
    IVI_MAC_STOP_FLAG = 0x20,
    DB_HIF_REINIT_FLAG = 0x101,
	IVI_HIF_REINIT_FLAG = 0x102,
} vmac_msgbox_type_t;

#ifdef CONFIG_DEBUG_FS
#define DEBUGFS_RING_SIZE		2048
#define DEBUGFS_MAX_PARAMS 		5
#define DEBUGFS_MAX_BUF			60
#define DEBUGFS_DUMP_RING_NUM	50
#define DEBUGFS_PARAM(idx, max_val) \
    do { \
        if (priv->dbgfs_parm[idx] > (max_val)) \
            priv->dbgfs_parm[idx] = (max_val); \
    } while (0)

typedef enum {
    TX_SW_BIT,
    TX_BD_BIT,
    TX_WRBD_BIT,
    RX_BD_BIT,
	RX_WRBD_BIT,
}vmac_debugfs_bits_t;

typedef enum {
    TX_SW_DEBUGFS_FLAG = BIT(0),
    TX_BD_DEBUGFS_FLAG = BIT(1),
    TX_WRBD_DEBUGFS_FLAG = BIT(2),
    RX_SW_DEBUGFS_FLAG = BIT(4),
    RX_BD_DEBUGFS_FLAG = BIT(5),
	RX_WRBD_DEBUGFS_FLAG = BIT(6),
} vmac_debugfs_type_t;
#endif

struct bstvmac_tx_info {
	dma_addr_t buf;
	bool map_as_page;
	unsigned int len;
	bool last_segment;
	bool is_jumbo;
	enum bstvmac_txbuf_type buf_type;
};

/* Frequently used values are kept adjacent for cache effect */
struct bstvmac_tx_queue {
	u32 tx_count_frames;
	struct hrtimer txtimer;
	u32 queue_index;
	struct bstvmac_priv *priv_data;
	struct dma_bd_desc *dma_bd_tx ____cacheline_aligned_in_smp;
	struct dma_wrbd_desc *dma_wrbd_tx ____cacheline_aligned_in_smp;
	struct sk_buff **tx_skbuff;
	struct bstvmac_tx_info *tx_skbuff_dma;
	unsigned int cur_tx;
	unsigned int dirty_tx;
	dma_addr_t dma_bd_tx_phy;
	dma_addr_t dma_wrbd_tx_phy;
	u32 seq_num;
	u32 tx_tail_addr;
	u32 mss;
	u32 run_status;
};

struct bstvmac_rx_queue {
	u32 rx_count_frames;
    struct hrtimer rxtimer;
    struct hrtimer rxtimer2;
	u32 queue_index;
	struct bstvmac_priv *priv_data;
	struct dma_bd_desc *dma_bd_rx ____cacheline_aligned_in_smp;
	struct dma_wrbd_desc *dma_wrbd_rx ____cacheline_aligned_in_smp;
	spinlock_t que_lock ____cacheline_aligned_in_smp;
	struct sk_buff **rx_skbuff;
	dma_addr_t *rx_skbuff_dma;
	unsigned int cur_rx;
	unsigned int dirty_rx;
	dma_addr_t dma_bd_rx_phy;
	dma_addr_t dma_wrbd_rx_phy;
	u32 seq_num;
	u32 rx_tail_addr;
	u32 run_status;
};

struct bstvmac_channel {
	struct napi_struct rnapi ____cacheline_aligned_in_smp;
	struct bstvmac_priv *priv_data;
	spinlock_t lock;
	u32 index;
	int has_rx;
	int has_tx;
	struct work_struct tx_work;
	void __iomem *base_addr;
};

struct bstvmac_priv {
	/* Frequently used values are kept adjacent for cache effect */
	u32 tx_coal_frames[MAX_TX_QUEUES];
	u32 tx_coal_timer[MAX_TX_QUEUES];
	u32 rx_coal_frames[MAX_TX_QUEUES];

	unsigned int dma_buf_sz;
	void __iomem *ioaddr;
	struct net_device *dev;
	struct device *device;
	struct mac_device_info *hw;
	int (*hwif_quirks)(struct bstvmac_priv *priv);
	struct mutex lock;

	/* RX Queue */
	struct bstvmac_rx_queue rx_queue[MAX_RX_QUEUES];
	unsigned int dma_rx_size;

	/* TX Queue */
	struct bstvmac_tx_queue tx_queue[MAX_TX_QUEUES];
	unsigned int dma_tx_size;

	/* Generic channel for NAPI */
	struct bstvmac_channel rx_channel[IMGMAC_CH_MAX];
	struct bstvmac_channel tx_channel[IMGMAC_CH_MAX];

	unsigned int pause;

	struct bstvmac_extra_stats xstats ____cacheline_aligned_in_smp;
	struct plat_vmacenet_data *plat;
	int vmac_id;
	u32 msg_enable;
	int perch_irq[BSTVMAC_MAX_IRQ_NUM];
	unsigned int mode;
	unsigned int chain_mode;
	void __iomem *chanl_start_addr;
	void __iomem *hif_base_addr;

#ifdef CONFIG_DEBUG_FS
	int dbgfs_parm[DEBUGFS_MAX_PARAMS];
	struct dentry *dbgfs_dir;
	struct dentry *dbgfs_rings_status;
#endif

	unsigned long state;
	struct workqueue_struct *wq;
	struct workqueue_struct *rxmem;
	struct workqueue_struct *tx_wq;
	struct work_struct service_task;
	struct work_struct mem_mgmt_work;
	bool tx_fifo_clear;
	bool rx_fifo_clear;
};

enum bstvmac_state {
	BSTVMAC_DOWN,
	BSTVMAC_RUNNING,
	BSTVMAC_RESET_REQUESTED,
	BSTVMAC_RESETTING,
	BSTVMAC_SERVICE_SCHED,
	BSTVMAC_RXMEM_WORK_RUN,
	BSTVMAC_TX_FIFO_CLEAR,
	BSTVMAC_RX_FIFO_CLEAR,
	BSTVMAC_RX_FIFO_CLEAR_DONE,
};

int bstvmac_sendmsg_to_switch(struct net_device *ndev, unsigned char flag);
void bstvmac_set_ethtool_ops(struct net_device *netdev);
int bstvmac_resume(struct device *dev);
int bstvmac_suspend(struct device *dev);
int bstvmac_dvr_remove(struct device *dev);
int bstvmac_dvr_probe(struct platform_device *pdev,
		      struct plat_vmacenet_data *plat_dat,
		      struct bstvmac_resources *res);
#endif /* __BSTVMAC_H__ */
