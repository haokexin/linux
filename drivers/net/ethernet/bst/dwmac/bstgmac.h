// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 * Copyright (C) 2007-2011 STMicroelectronics Ltd
 */

#ifndef __BSTGMAC_H__
#define __BSTGMAC_H__

#define BSTGMAC_RESOURCE_NAME   "bstgmaceth"
#define DRV_MODULE_VERSION	"Jan_2016"

#include <linux/clk.h>
#include <linux/hrtimer.h>
#include <linux/if_vlan.h>
#include <linux/stmmac.h>
#include <linux/phylink.h>
#include <linux/pci.h>
#include "common.h"
#include <linux/ptp_clock_kernel.h>
#include <linux/reset.h>
#include <linux/kfifo.h>

#define BSTGMAC_RX_COE_NONE	0
#define BSTMAC_PTP_KFIFO_NUM	16

enum bstmac_mb_sub {
	BSTMAC_MB_SUB_INIT = 0,
	BSTMAC_MB_SUB_OK,
	BSTMAC_MB_SUB_DONE,
};

struct bstgmac_resources {
	void __iomem *addr;
#ifdef CONFIG_UIO
	struct resource *res;
	struct uio_info	*info;
#endif
	u8 mac[ETH_ALEN];
	int sfty_ce_irq;
	int sfty_uc_irq;
	int wol_irq;
	int perch_rx_irq[MTL_MAX_RX_QUEUES];
	int perch_tx_irq[MTL_MAX_TX_QUEUES];
	int lpi_irq;
	int irq;
	int wdata_ucerr_irq;
	int paddr_parity_irq;
};

enum bstgmac_txbuf_type {
	BSTGMAC_TXBUF_T_SKB,
	BSTGMAC_TXBUF_T_XDP_TX,
	BSTGMAC_TXBUF_T_XDP_NDO,
	BSTGMAC_TXBUF_T_XSK_TX,
};


struct bstgmac_tx_info {
	dma_addr_t buf;
	bool map_as_page;
	unsigned int len;
	bool last_segment;
	bool is_jumbo;
	enum bstgmac_txbuf_type buf_type;
};


#define BSTMAC_TBS_AVAIL	BIT(0)
#define BSTMAC_TBS_EN		BIT(1)

/* Frequently used values are kept adjacent for cache effect */
struct bstgmac_tx_queue {
	u32 tx_count_frames;
	int tbs;
	struct hrtimer txtimer;
	u32 queue_index;
	struct bstgmac_priv *priv_data;
	struct dma_extended_desc *dma_etx ____cacheline_aligned_in_smp;
	struct dma_edesc *dma_entx;
	struct dma_desc *dma_tx;
	struct sk_buff **tx_skbuff;
	struct bstgmac_tx_info *tx_skbuff_dma;
	unsigned int cur_tx;
	unsigned int dirty_tx;
	dma_addr_t dma_tx_phy;
	u32 tx_tail_addr;
	u32 mss;
	u32 run_status;
};

struct bstgmac_rx_queue {
	u32 rx_count_frames;
	u32 queue_index;
	struct bstgmac_priv *priv_data;
	struct dma_extended_desc *dma_erx;
	struct dma_desc *dma_rx ____cacheline_aligned_in_smp;
	spinlock_t que_lock ____cacheline_aligned_in_smp;
	struct sk_buff **rx_skbuff;
	dma_addr_t *rx_skbuff_dma;
	unsigned int cur_rx;
	unsigned int dirty_rx;
	u32 rx_zeroc_thresh;
	dma_addr_t dma_rx_phy;
	u32 rx_tail_addr;
	u32 run_status;
};

struct bstgmac_channel {
	struct napi_struct napi ____cacheline_aligned_in_smp;
	struct napi_struct rnapi ____cacheline_aligned_in_smp;
	struct napi_struct tnapi ____cacheline_aligned_in_smp;
	struct bstgmac_priv *priv_data;
	spinlock_t lock;
	u32 index;
	int has_rx;
	int has_tx;
	int int_mode;
	struct work_struct rx_work;
	struct work_struct tx_work;
};

struct bstgmac_tc_entry {
	bool in_use;
	bool in_hw;
	bool is_last;
	bool is_frag;
	void *frag_ptr;
	unsigned int table_pos;
	u32 handle;
	u32 prio;
	struct {
		u32 match_data;
		u32 match_en;
		u8 af:1;
		u8 rf:1;
		u8 im:1;
		u8 nc:1;
		u8 res1:4;
		u8 frame_offset;
		u8 pcv:1;
		u8 pcn:4;
		u8 res2:3;
		u8 giv:1;
		u8 gid:3;
		u8 res3:4;
		u16 dma_ch_no;
		u16 res4;
	} __packed val;
};

#define BSTGMAC_PPS_MAX		4
struct bstgmac_pps_cfg {
	bool available;
	struct timespec64 start;
	struct timespec64 period;
};

/*RSS*/
struct bstgmac_rss {
	int enable;
	u8 key[BSTXGMAC_RSS_HASH_KEY_SIZE];
	u32 table[BSTXGMAC_RSS_MAX_TABLE_SIZE];
};

#define BSTMAC_FLOW_ACTION_DROP		BIT(0)
#define BSTMAC_FLOW_ACTION_GATE		BIT(1)
struct bstgmac_flow_entry {
	unsigned long cookie;
	unsigned long action;
	u8 ip_proto;
	int in_use;
	int idx;
	int is_l4;
};

struct bstgmac_flex_pps {
	int idx;
	bool status;
	struct timespec64 start_time;
	struct timespec64 period;
};

struct bstgmac_aux_snap {
	int idx;
	bool status;
	struct timespec64 snap_time;
};

#define BSTGMAC_PKT_CAPTURE_OUI		0xB
struct bstgmac_pkt_info {
	u16 ether_type;
	u16 tci;
	u8 protocol;		/* protocol types in IP format */
};

struct bstgmac_pkt_capture {
	bool tx;
	bool rx;
	bool vlan;
	bool icmp;
	bool all;
	u16 ether_type;
	u16 tci;
};

struct bstptp_ctl {
	int ppssta;
	int ppsfix;
	int multip;
	int extintr;
	void __iomem *ptp0_reg;
	void __iomem *ptp1_reg;
	struct timespec64 utc;
	struct bstgmac_flex_pps flex_pps;
	struct bstgmac_aux_snap aux_snap;
	struct kfifo tx_ts_fifo;
	spinlock_t tx_ts_lock;
};

struct bstgmac_priv {
	/* Frequently used values are kept adjacent for cache effect */
	u32 tx_coal_frames[MTL_MAX_TX_QUEUES];
	u32 tx_coal_timer[MTL_MAX_TX_QUEUES];
	u32 rx_coal_frames[MTL_MAX_TX_QUEUES];

	int tx_coalesce;
	int hwts_tx_en;
	bool tx_path_in_lpi_mode;
	bool rx_path_in_lpi_mode;
	bool tso;
	int sph;
	u32 sarc_type;

	unsigned int dma_buf_sz;
	unsigned int rx_copybreak;
	u32 rx_riwt[MTL_MAX_TX_QUEUES];
	int hwts_rx_en;

	void __iomem *ioaddr;
	struct net_device *dev;
	struct device *device;
	struct mac_device_info *hw;
#ifdef CONFIG_UIO
	struct uio_info	*info;
#endif
	int (*hwif_quirks)(struct bstgmac_priv *priv);
	struct mutex lock;

	/* RX Queue */
	struct bstgmac_rx_queue rx_queue[MTL_MAX_RX_QUEUES];
	unsigned int dma_rx_size;

	/* TX Queue */
	struct bstgmac_tx_queue tx_queue[MTL_MAX_TX_QUEUES];
	unsigned int dma_tx_size;

	/* Generic channel for NAPI */
	struct bstgmac_channel rx_channel[STMMAC_CH_MAX];
	struct bstgmac_channel tx_channel[STMMAC_CH_MAX];

	bool oldlink;
	int speed;
	int oldduplex;
	unsigned int flow_ctrl;
	unsigned int pause;
	struct mii_bus *mii;
	int mii_irq[PHY_MAX_ADDR];

	struct phylink_config phylink_config;
	struct phylink *phylink;
	struct phy_device *phydev;

	struct bstgmac_extra_stats xstats ____cacheline_aligned_in_smp;
	struct bstgmac_safety_stats sstats;
	struct plat_stmmacenet_data *plat;
	struct dma_features dma_cap;
	struct bstgmac_counters mmc;
	int hw_cap_support;
	int synopsys_id;
	u32 msg_enable;
	int wolopts;
	int wol_irq;
	int clk_csr;
	struct timer_list eee_ctrl_timer;
	int lpi_irq;
	int sfty_ce_irq;
	int sfty_uc_irq;
	int perch_rx_irq[MTL_MAX_RX_QUEUES];
	int perch_tx_irq[MTL_MAX_TX_QUEUES];
	int wdata_ucerr_irq;
	int paddr_parity_irq;
	int eee_enabled;
	int eee_active;
	int tx_lpi_timer;
	int tx_lpi_enabled;
	int eee_tw_timer;
	unsigned int mode;
	unsigned int chain_mode;
	int extend_desc;
	struct ptp_clock *ptp_clock;
	struct ptp_clock_info ptp_clock_ops;
	unsigned int default_addend;
	u32 sub_second_inc;
	u32 systime_flags;
	u32 adv_ts;
	int use_riwt;
	int irq_wake;
	spinlock_t ptp_lock;
	spinlock_t ptp_flex_lock;
	void __iomem *mmcaddr;
	void __iomem *ptpaddr;
	unsigned long active_vlans[BITS_TO_LONGS(VLAN_N_VID)];

#ifdef CONFIG_DEBUG_FS
	struct dentry *dbgfs_dir;
	struct dentry *dbgfs_rings_status;
	struct dentry *dbgfs_dma_cap;
#endif

	unsigned long state;
	struct workqueue_struct *wq;
	struct workqueue_struct *rxmem;
	struct workqueue_struct *tx_wq;
	struct work_struct service_task;
	struct delayed_work mb_resub_task;
	struct work_struct mem_mgmt_work;

	/* Workqueue for handling FPE hand-shaking */
	unsigned long fpe_task_state;
	struct workqueue_struct *fpe_wq;
	struct work_struct fpe_task;
	char wq_name[IFNAMSIZ + 4];

	/* TC Handling */
	unsigned int tc_entries_max;
	unsigned int tc_off_max;
	struct bstgmac_tc_entry *tc_entries;
	unsigned int flow_entries_max;
	struct bstgmac_flow_entry *flow_entries;

	/* Pulse Per Second output */
	struct bstgmac_pps_cfg pps[BSTGMAC_PPS_MAX];
	int extend_op;

	/* Receive Side Scaling */
	struct bstgmac_rss rss;

	struct bstptp_ctl *ptpctl;
	struct bstgmac_pkt_capture pkt_capture;

	u32 fpe_tx_queue_mask;
	u32 fpe_min_frag_size;

	unsigned int ipc_state;
	bool bypass;
	int fpe_hs;
	int64_t time_offset;
	int tx_irq_num;
	int rx_irq_num;
};

enum bstgmac_state {
	BSTGMAC_DOWN,
	BSTGMAC_RESET_REQUESTED,
	BSTGMAC_RESETTING,
	BSTGMAC_SERVICE_SCHED,
	BSTGMAC_RXMEM_WORK_RUN,
};

struct bstgmac_mem_t {
	struct sk_buff *skb;
};

struct cmd_mac {
	unsigned char bst_num;
	unsigned char subtype;
	unsigned char mac_address[7][6];
};

int bstgmac_mdio_unregister(struct net_device *ndev);
int bstgmac_mdio_register(struct net_device *ndev);
int bstgmac_mdio_reset(struct mii_bus *mii);
void bstgmac_set_ethtool_ops(struct net_device *netdev);

void bstgmac_ptp_register(struct bstgmac_priv *priv);
void bstgmac_ptp_unregister(struct bstgmac_priv *priv);
int bstgmac_resume(struct device *dev);
int bstgmac_suspend(struct device *dev);
int bstgmac_dvr_remove(struct device *dev);
int bstgmac_dvr_probe(struct platform_device *pdev,
		      struct plat_stmmacenet_data *plat_dat,
		      struct bstgmac_resources *res);
void bstgmac_disable_eee_mode(struct bstgmac_priv *priv);
bool bstgmac_eee_init(struct bstgmac_priv *priv);
void bstgmac_fpe_handshake(struct bstgmac_priv *priv, bool enable);
int bstgmac_config_mv88e6352(struct net_device *ndev);
#ifdef CONFIG_BROADCOM_PHY
void bcm89881_extend_op(struct phy_device *phydev);
#else
static inline void bcm89881_extend_op(struct phy_device *phydev)
{
}
#endif
struct timespec64 bstgmac_calc_tas_basetime(ktime_t old_base_time,
					    ktime_t current_time,
					    u64 cycle_time);
void bstmac_selftest_run(struct net_device *dev,
			 struct ethtool_test *etest, u64 *buf);
void bstmac_selftest_get_strings(struct bstgmac_priv *priv, u8 *data);
int bstmac_selftest_get_count(struct bstgmac_priv *priv);
int bstmac_get_ts_flag_slt(void);
void bstmac_clr_ts_flag_slt(void);
void bstmac_set_ts_flag_slt(bool tx);
extern int scmi_read(u32 reg,u32 *val);
extern int scmi_write(u32 reg,u32 val);
extern int bstmac_test_hfilt(struct bstgmac_priv *priv);
extern int bstmac_test_pfilt(struct bstgmac_priv *priv);
extern int bstmac_test_mcfilt(struct bstgmac_priv *priv);
extern int bstmac_test_ucfilt(struct bstgmac_priv *priv);
extern int bstmac_test_l3filt_da(struct bstgmac_priv *priv);
extern int bstmac_test_l3filt_sa(struct bstgmac_priv *priv);
extern int bstmac_test_l4filt_da_tcp(struct bstgmac_priv *priv);
extern int bstmac_test_l4filt_sa_tcp(struct bstgmac_priv *priv);
extern int bstmac_test_l4filt_da_udp(struct bstgmac_priv *priv);
extern int bstmac_test_l4filt_sa_udp(struct bstgmac_priv *priv);
extern int bstmac_test_multichannel(struct bstgmac_priv *priv);
extern int bstmac_test_ipv6_l3filt_da(struct bstgmac_priv *priv);
extern int bstmac_test_ipv6_l3filt_sa(struct bstgmac_priv *priv);
extern int bstmac_test_flowctrl(struct bstgmac_priv *priv);
#endif /* __BSTGMAC_H__ */
