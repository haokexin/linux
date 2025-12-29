// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021-2024 Black Sesame Technologies. All Rights Reserved.
 * Copyright (C) 2007-2009 STMicroelectronics Ltd
 */

#ifndef __COMMON_H__
#define __COMMON_H__

#include "hwif.h"

#define DEBUG
#define PAUSE_TIME				0xffff
#define JUMBO_LEN				9000

/* Imagination Technologies Core versions */
#define	BSTVMAC_CORE_10			0x10

/* These need to be power of two, and >= 4 */
#define DMA_TX_SIZE 			2048
#define DMA_RX_SIZE 			2048
#define BSTVMAC_GET_ENTRY(x, size)	((x + 1) & (size - 1))

/* Msgbox parameters */
#define BSTVMAC_MSGBOX_MAX_CNT	5

/* Tx coalesce parameters */
#define BSTVMAC_COAL_TX_TIMER	200
#define BSTVMAC_TX_FRAMES		16
#define BSTVMAC_RX_FRAMES		16

#define BSTVMAC_BUS_ID         	0
#define BSTVMAC_CORE_NUM        1
#define BSTVMAC_RXCHAN_NUM      1
#define BSTVMAC_RXMEM_THRE      (DMA_RX_SIZE)
#define BSTVMAC_RXMEM_MAX       (BSTVMAC_RXMEM_THRE * 2) /* must *2 */
#define BSTVMAC_RXMEM_MASK      (BSTVMAC_RXMEM_MAX - 1)

/* GMAC TX FIFO is 8K, Rx FIFO is 16K */
#define BUF_SIZE_16KiB 			16384
#define BUF_SIZE_9KiB			9212
#define BUF_SIZE_8KiB			8188
#define BUF_SIZE_4KiB			4096
#define BUF_SIZE_2KiB			2048

/* Extra statistic and debug information exposed by ethtool */
struct bstvmac_extra_stats {
	/* Tx/Rx IRQ Events */
	unsigned long tx_pkt_n ____cacheline_aligned;
	unsigned long rx_pkt_n;
	unsigned long rx_normal_irq_n;
	unsigned long rnapi_poll;
	unsigned long rx_memwork_poll;
	unsigned long txwork_poll;
	unsigned long tx_normal_irq_n;
	unsigned long tx_clean;
	unsigned long tx_set_ic_bit;
};

/* Rx IPC status */
enum rx_frame_status {
	good_frame = 0x0,
	rx_not_ls = 0x1,
	no_frame = 0x2,
	rx_behind = 0x4,
};

/* Tx status */
enum tx_frame_status {
	tx_done = 0x0,
	tx_not_ls = 0x1,
	tx_err = 0x2,
	tx_sw_own = 0x4,
	tx_not_done = 0x8,
};

enum dma_irq_status {
	tx_hard_error = 0x1,
	tx_hard_error_bump_tc = 0x2,
	handle_rx = 0x4,
	handle_tx = 0x8,
	rx_hard_error = 0x10,
};

struct mac_device_info {
	const struct bstvmac_ops *mac;
	const struct bstvmac_desc_ops *desc;
	const struct bstvmac_dma_ops *dma;
	const struct bstvmac_mode_ops *mode;
	const struct bstvmac_priv *priv;
};

struct tx_header
{
    u32 queue: 4; 
    u32 txport_map: 20;
    u32 ctrl: 8;

	u32 seq_num:16;
	u32 rx_ch_no:6;
	u32 ipsec_pbc:1;
	u32 ipsec_mcast_bcast:1;
	u32 ipsec_seq_num_valid:1;
	u32 ipsec_seq_num:2;	
    u32 rsvd1:5;	
	
    u32 ipsec_sap;
	
    u16 ipsec_iphy;
    u16 ipsec_ophy;
};

struct rx_header
{
    u16 punt_reason;
    u8 rxport_num;
    u8 ctrl;
    u32 rsvd;
    u32 rx_timestamp_nsec; 
    u32 rx_timestamp_sec; 
};

/* bd descriptor structure */
struct dma_bd_desc {
	__le32 des0;
	__le32 des1;
	__le32 des2;
	__le32 des3;
};

/* wrbd descriptor structure */
struct dma_wrbd_desc {
	__le32 des0;
	__le32 des1;
};

extern const struct bstvmac_mode_ops dwvmac10_chain_ops;
extern const struct bstvmac_desc_ops dwvmac10_desc_ops;

#endif /* __COMMON_H__ */
