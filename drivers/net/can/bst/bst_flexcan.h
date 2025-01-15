// SPDX-License-Identifier: GPL-2.0
 /* 
  * bst_flexcan.h - FLEXCAN CAN controller driver
  * Copyright (c) 2005-2006 Varma Electronics Oy
  * Copyright (c) 2009 Sascha Hauer, Pengutronix
  * Copyright (c) 2010-2017 Pengutronix, Marc Kleine-Budde <kernel@pengutronix.de>
  * Copyright (c) 2014 David Jander, Protonic Holland
  * Copyright (C) 2022 Amarula Solutions, Dario Binacchi <dario.binacchi@amarulasolutions.com>
  * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved. 
  */ 
// Based on code originally by Andrey Volkov <avolkov@varma-el.com>

#ifndef BST_FLEXCAN_H
#define BST_FLEXCAN_H

#include <linux/can/rx-offload.h>
#include "../../../dma/virt-dma.h"
// #include "../../../dma/bst/bst-axi-dmac.h"


#define SW_SYS_CTRL				0x217B1000
#define SW2SOC_INTR_SEL3_0		0x600

#define SW_CRM_CSR				0x217B0000
#define SW_RST					0x28

#define SC_PMM_REG			    0x30001000  

#define SWT_LSP_CRM_REG_CTRL 0x217a0000
#define CAN0_IP_CFG 0x18
#define CAN1_IP_CFG 0x1c
#define CAN2_IP_CFG 0x20
// extern unsigned long bst_sip_special_address_rw(u_int64_t x1, u_int64_t x2, u_int64_t x3);

#define DMA_FALGS_CANFD_DEV (1 << 31)
#define DMA_FALGS_CAN_DEV (1 << 30)

#define BST_FLEXCAN_FRAME_MAX_LEN 4
#define BST_FLEXCAN_FD_FRAME_MAX_LEN 20
#define BST_FLEXCAN_RX_DMA_FIFO_SIZE 8

/* FLEXCAN hardware feature flags
 *
 * Below is some version info we got:
 *    SOC   Version   IP-Version  Glitch- [TR]WRN_INT IRQ Err Memory err RTR rece-   FD Mode
 *                                Filter? connected?  Passive detection  ption in MB Supported?
 *   MX25  FlexCAN2  03.00.00.00     no        no        no       no        no           no
 *   MX28  FlexCAN2  03.00.04.00    yes       yes        no       no        no           no
 *   MX35  FlexCAN2  03.00.00.00     no        no        no       no        no           no
 *   MX53  FlexCAN2  03.00.00.00    yes        no        no       no        no           no
 *   MX6s  FlexCAN3  10.00.12.00    yes       yes        no       no       yes           no
 *   MX8QM FlexCAN3  03.00.23.00    yes       yes        no       no       yes          yes
 *   MX8MP FlexCAN3  03.00.17.01    yes       yes        no      yes       yes          yes
 *   VF610 FlexCAN3  ?               no       yes        no      yes       yes?          no
 * LS1021A FlexCAN2  03.00.04.00     no       yes        no       no       yes           no
 * LX2160A FlexCAN3  03.00.23.00     no       yes        no      yes       yes          yes
 *
 * Some SOCs do not have the RX_WARN & TX_WARN interrupt line connected.
 */

/* [TR]WRN_INT not connected */
#define FLEXCAN_QUIRK_BROKEN_WERR_STATE BIT(1)
 /* Disable RX FIFO Global mask */
#define FLEXCAN_QUIRK_DISABLE_RXFG BIT(2)
/* Enable EACEN and RRS bit in ctrl2 */
#define FLEXCAN_QUIRK_ENABLE_EACEN_RRS  BIT(3)
/* Disable non-correctable errors interrupt and freeze mode */
#define FLEXCAN_QUIRK_DISABLE_MECR BIT(4)
/* Use mailboxes (not FIFO) for RX path */
#define FLEXCAN_QUIRK_USE_RX_MAILBOX BIT(5)
/* No interrupt for error passive */
#define FLEXCAN_QUIRK_BROKEN_PERR_STATE BIT(6)
/* default to BE register access */
#define FLEXCAN_QUIRK_DEFAULT_BIG_ENDIAN BIT(7)
/* Setup stop mode to support wakeup */
#define FLEXCAN_QUIRK_SETUP_STOP_MODE BIT(8)
/* Support CAN-FD mode */
#define FLEXCAN_QUIRK_SUPPORT_FD BIT(9)
/* support memory detection and correction */
#define FLEXCAN_QUIRK_SUPPORT_ECC BIT(10)
/* Use Enhanced FIFO for RX path */
#define FLEXCAN_QUIRK_USE_RX_ENHANCED_FIFO BIT(11)

#define FLEXCAN_NUMBER_OF_MB 128
#define FLEXCAN_NUMBER_OF_MB_CANFD 56

/** FLEXCAN - Size of Registers Arrays */
#define FLEXCAN_RXIMR_COUNT                       128
#define FLEXCAN_HR_TIME_STAMP_COUNT               128
#define FLEXCAN_ERFFEL_COUNT                      128

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

	struct flexcan_regs __iomem *regs;
	struct flexcan_mb __iomem *tx_mb;
	struct flexcan_mb __iomem *tx_mb_reserved;
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

	struct clk *clk_ipg;
	struct clk *clk_per;
	const struct flexcan_devtype_data *devtype_data;
	struct regulator *reg_xceiver;
	struct flexcan_stop_mode stm;

	dma_cap_mask_t mask;
	struct dma_chan *chan;
	struct dma_slave_config cfg;
	struct scatterlist sg;
	struct dma_async_tx_descriptor *dma_tx;
	struct axi_dma_desc *desc;
	struct axi_dma_chan *ad_chan;
	struct flexcan_ring_buffer rb;
	u32 desc_head;

	struct hrtimer rx_timer;
	ktime_t tim;

	/* Read and Write APIs */
	u32 (*read)(void __iomem *addr);
	void (*write)(u32 val, void __iomem *addr);
};



#endif /* BST_FLEXCAN_H */