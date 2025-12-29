// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021-2024 Black Sesame Technologies. All Rights Reserved.
 * Copyright (C) 2018 Synopsys, Inc. and/or its affiliates.
 */

#ifndef __BSTVMAC_HWIF_H__
#define __BSTVMAC_HWIF_H__

#include <net/pkt_sched.h>
#include "bstvmac_platform.h"

#define bstmac_do_void_callback(__priv, __module, __cname,  __arg0, __args...) \
({ \
	int __result = -EINVAL; \
	if ((__priv)->hw->__module && (__priv)->hw->__module->__cname) { \
		(__priv)->hw->__module->__cname((__arg0), ##__args); \
		__result = 0; \
	} \
	__result; \
})
#define bstmac_do_callback(__priv, __module, __cname,  __arg0, __args...) \
({ \
	int __result = -EINVAL; \
	if ((__priv)->hw->__module && (__priv)->hw->__module->__cname) \
		__result = (__priv)->hw->__module->__cname((__arg0), ##__args); \
	__result; \
})

struct dma_bd_desc;
struct dma_wrbd_desc;
struct bstvmac_priv;
struct bstvmac_extra_stats;

/* Descriptors helpers */
struct bstvmac_desc_ops {
	/* Return the transmit status looking at the TDES1 */
	int (*tx_status)(void *data, struct bstvmac_extra_stats *x,
			 struct dma_bd_desc *p, struct dma_wrbd_desc *wp, void __iomem *ioaddr);
	/* Return the reception status looking at the RDES1 */
	int (*rx_status)(void __iomem *ioaddr, u32 chan, u32 index,
			 struct dma_wrbd_desc *wp);
	/* clear wrbd ctrl bits */
	void (*clear_status)(struct dma_wrbd_desc *wp);
	/* Set/get the owner of the descriptor */
	void (*set_tx_owner)(struct dma_bd_desc *p, bool en);
	/* Handle extra events on specific interrupts hw dependent */
	void (*set_rx_owner)(struct dma_bd_desc *p, bool en);
	/* Get the receive frame size */
	int (*get_rx_frame_len)(struct dma_wrbd_desc *wp);
	/* Clear interrupt on tx frame completion. When this bit is
	 * set an interrupt happens as soon as the frame is transmitted
	 */
	void (*set_tx_ic)(struct dma_bd_desc *p);
	/* Invoked by the xmit function to prepare the tx descriptor */
	void (*prepare_tx_desc)(struct dma_bd_desc *p, int len, bool ls_bd,
					 bool tx_own, bool ls, unsigned int seq_num);
	/* Clean the tx descriptor as soon as the tx irq is received */
	void (*release_tx_desc)(struct dma_bd_desc *p, struct dma_wrbd_desc *wp, int mode);
	/* DMA RX descriptor ring initialization */
	void (*init_bd_rx_desc)(struct dma_bd_desc *p, unsigned int seq_num, bool last,
					unsigned int buf_sz);
	void (*init_wrbd_desc)(struct dma_wrbd_desc *p, unsigned int seq_num);
	/* DMA TX descriptor ring initialization */
	void (*init_bd_tx_desc)(struct dma_bd_desc *p, int end);
	/* set descriptor skbuff address */
	void (*set_addr)(struct dma_bd_desc *p, dma_addr_t addr);
	void (*set_rx_dir)(struct dma_bd_desc *p);
	void (*set_rx_intr)(struct dma_bd_desc *p, bool on);
	void (*set_rx_bufsz)(struct dma_bd_desc *p, u32 size);
	void (*set_rx_seqnum)(struct dma_bd_desc *p, u32 seq_num);
	void (*set_rx_bd_last)(struct dma_bd_desc *p, bool last);
};

#define bstvmac_init_bd_rx_desc(__priv, __args...) \
	bstmac_do_void_callback(__priv, desc, init_bd_rx_desc, __args)
#define bstvmac_init_wrbd_desc(__priv, __args...) \
	bstmac_do_void_callback(__priv, desc, init_wrbd_desc, __args)
#define bstvmac_init_bd_tx_desc(__priv, __args...) \
	bstmac_do_void_callback(__priv, desc, init_bd_tx_desc, __args)
#define bstvmac_prepare_tx_desc(__priv, __args...) \
	bstmac_do_void_callback(__priv, desc, prepare_tx_desc, __args)
#define bstvmac_set_tx_owner(__priv, __args...) \
	bstmac_do_void_callback(__priv, desc, set_tx_owner, __args)
#define bstvmac_release_tx_desc(__priv, __args...) \
	bstmac_do_void_callback(__priv, desc, release_tx_desc, __args)
#define bstvmac_set_tx_ic(__priv, __args...) \
	bstmac_do_void_callback(__priv, desc, set_tx_ic, __args)
#define bstvmac_tx_status(__priv, __args...) \
	bstmac_do_callback(__priv, desc, tx_status, __args)
#define bstvmac_set_rx_owner(__priv, __args...) \
	bstmac_do_void_callback(__priv, desc, set_rx_owner, __args)
#define bstvmac_get_rx_frame_len(__priv, __args...) \
	bstmac_do_callback(__priv, desc, get_rx_frame_len, __args)
#define bstvmac_rx_status(__priv, __args...) \
	bstmac_do_callback(__priv, desc, rx_status, __args)
#define bstvmac_clear_status(__priv, __args...) \
	bstmac_do_void_callback(__priv, desc, clear_status, __args)
#define bstvmac_set_desc_addr(__priv, __args...) \
	bstmac_do_void_callback(__priv, desc, set_addr, __args)
#define bstvmac_set_bd_rx_dir(__priv, __args...) \
	bstmac_do_void_callback(__priv, desc, set_rx_dir, __args)
#define bstvmac_set_bd_rx_intr(__priv, __args...) \
	bstmac_do_void_callback(__priv, desc, set_rx_intr, __args)
#define bstvmac_set_bd_rx_bufsz(__priv, __args...) \
	bstmac_do_void_callback(__priv, desc, set_rx_bufsz, __args)
#define bstvmac_set_bd_rx_seqnum(__priv, __args...) \
	bstmac_do_void_callback(__priv, desc, set_rx_seqnum, __args)
#define bstvmac_set_bd_rx_last(__priv, __args...) \
	bstmac_do_void_callback(__priv, desc, set_rx_bd_last, __args)

/* Specific DMA helpers */
struct bstvmac_dma_ops {
	void (*init_rx_chan)(void __iomem *ioaddr, dma_addr_t dma_bd_rx_phy,
				 dma_addr_t dma_wrbd_rx_phy, u32 chan);
	void (*init_tx_chan)(void __iomem *ioaddr, dma_addr_t dma_bd_tx_phy,
				 dma_addr_t dma_wrbd_tx_phy, u32 chan);
	void (*enable_dma_irq)(void __iomem *ioaddr, u32 chan, bool rx, bool tx);
	void (*disable_dma_irq)(void __iomem *ioaddr, u32 chan, bool rx, bool tx);
	void (*start_tx)(void __iomem *ioaddr, u32 chan);
	void (*stop_tx)(void __iomem *ioaddr, u32 chan);
	void (*start_rx)(void __iomem *ioaddr, u32 chan);
	void (*stop_rx)(void __iomem *ioaddr, u32 chan);
	int (*dma_ri_interrupt)(void __iomem *ioaddr,
				struct bstvmac_extra_stats *x, u32 chan);
	int (*dma_ti_interrupt)(void __iomem *ioaddr,
				struct bstvmac_extra_stats *x, u32 chan);
	void (*set_rx_bfsize)(void __iomem *ioaddr, int bfsize, u32 chan);
	void (*set_tx_bfsize)(void __iomem *ioaddr, int bfsize, u32 chan);
	bool (*rx_fifo_clear_status)(void __iomem *ioaddr, u32 chan);
	bool (*tx_fifo_clear_status)(void __iomem *ioaddr, u32 chan);
	void (*recover_dma_irq)(void __iomem *ioaddr, u32 chan);
};

#define bstvmac_init_rx_chan(__priv, __args...) \
	bstmac_do_void_callback(__priv, dma, init_rx_chan, __args)
#define bstvmac_init_tx_chan(__priv, __args...) \
	bstmac_do_void_callback(__priv, dma, init_tx_chan, __args)
#define bstvmac_enable_dma_irq(__priv, __args...) \
	bstmac_do_void_callback(__priv, dma, enable_dma_irq, __args)
#define bstvmac_disable_dma_irq(__priv, __args...) \
	bstmac_do_void_callback(__priv, dma, disable_dma_irq, __args)
#define bstvmac_start_tx(__priv, __args...) \
	bstmac_do_void_callback(__priv, dma, start_tx, __args)
#define bstvmac_stop_tx(__priv, __args...) \
	bstmac_do_void_callback(__priv, dma, stop_tx, __args)
#define bstvmac_start_rx(__priv, __args...) \
	bstmac_do_void_callback(__priv, dma, start_rx, __args)
#define bstvmac_stop_rx(__priv, __args...) \
	bstmac_do_void_callback(__priv, dma, stop_rx, __args)
#define bstvmac_dma_ri_interrupt_status(__priv, __args...) \
	bstmac_do_callback(__priv, dma, dma_ri_interrupt, __args)
#define bstvmac_dma_ti_interrupt_status(__priv, __args...) \
	bstmac_do_callback(__priv, dma, dma_ti_interrupt, __args)
#define bstvmac_set_rx_dma_bfsize(__priv, __args...) \
	bstmac_do_void_callback(__priv, dma, set_rx_bfsize, __args)
#define bstvmac_set_tx_dma_bfsize(__priv, __args...) \
	bstmac_do_void_callback(__priv, dma, set_tx_bfsize, __args)
#define bstvmac_rx_fifo_clear_status(__priv, __args...) \
	bstmac_do_callback(__priv, dma, rx_fifo_clear_status, __args)
#define bstvmac_tx_fifo_clear_status(__priv, __args...) \
	bstmac_do_callback(__priv, dma, tx_fifo_clear_status, __args)
#define bstvmac_recover_dma_irq(__priv, __args...) \
	bstmac_do_void_callback(__priv, dma, recover_dma_irq, __args)

/* Helpers to program the MAC core */
struct bstvmac_ops {
	void (*timeout_en)(void __iomem *ioaddr, bool enable);
	void (*set_start_seqnum)(void __iomem *ioaddr, u32 value);
	void (*set_axi_write_done)(void __iomem *ioaddr, u32 value);
	void (*seqnum_check_en)(void __iomem *ioaddr, bool enable);
	/* Set/Get Unicast MAC addresses */
	int (*set_umac_addr)(struct net_device *ndev, int flag);
};

#define bstvmac_timeout_en(__priv, __args...) \
	bstmac_do_void_callback(__priv, mac, timeout_en, __args)
#define bstvmac_start_seqnum_set(__priv, __args...) \
	bstmac_do_void_callback(__priv, mac, set_start_seqnum, __args)
#define bstvmac_axi_write_done_set(__priv, __args...) \
	bstmac_do_void_callback(__priv, mac, set_axi_write_done, __args)
#define bstvmac_seqnum_check_en(__priv, __args...) \
	bstmac_do_void_callback(__priv, mac, seqnum_check_en, __args)
#define bstvmac_set_umac_addr(__priv, __args...) \
	bstmac_do_callback(__priv, mac, set_umac_addr, __args)

/* Helpers to manage the descriptors for chain and ring modes */
struct bstvmac_mode_ops {
	void (*init)(void *des, dma_addr_t phy_addr, unsigned int size, bool tx);
	void (*clean_desc0)(void *priv, struct dma_bd_desc *p);
};

#define bstvmac_mode_init(__priv, __args...) \
	bstmac_do_void_callback(__priv, mode, init, __args)
#define bstvmac_clean_desc0(__priv, __args...) \
	bstmac_do_void_callback(__priv, mode, clean_desc0, __args)

struct bstvmac_regs_off {
	u32 base_off;
	u32 chanl_start_off;
};

extern const struct bstvmac_ops dwvmac10_ops;
extern const struct bstvmac_dma_ops dwvmac10_dma_ops;
extern const struct bstvmac_desc_ops dwvmac10_desc_ops;

#if defined(CONFIG_BST_C1200_DB)
#define NUM_HIF_CHANNELS        9
#define HIF_CHAN_START          8
#define HIF_CHAN_MASK			BIT(8)
#elif defined(CONFIG_BST_C1200_IVI)
#define NUM_HIF_CHANNELS        17
#define HIF_CHAN_START          16
#define HIF_CHAN_MASK			BIT(0)
#endif

#define HIF_VERSION_OFFSET		0x0
#define HIF_BASE_ADDR			0x680000
#define HIF_CHANL_START_ADDR	(HIF_BASE_ADDR + (HIF_CHAN_START + 1) * 0x1000)

int bstvmac_hwif_init(struct bstvmac_priv *priv);

#endif /* __BSTGMAC_HWIF_H__ */
