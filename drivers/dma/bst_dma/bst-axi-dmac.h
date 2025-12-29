// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *  Copyright (C) 2017-2018 Synopsys, Inc. (www.synopsys.com)
 */

#ifndef _AXI_DMA_PLATFORM_H
#define _AXI_DMA_PLATFORM_H

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dmaengine.h>
#include <linux/types.h>

#include "../virt-dma.h"

enum{
	BST_AXI_DMA_TYPE_COMMON,
	BST_AXI_DMA_TYPE_BST
};

#define DMAC_MAX_CHANNELS	32
#define DMAC_MAX_MASTERS	2
#define DMAC_MAX_BLK_SIZE	0x100000

/* sdma support sdma */
#define SW_SYS_CTRL          0x217B1000
#define SW2SOC_INTR_SEL3_0   0x600
#define SW2SOC_INTR_SEL7_4   0x604
#define SWDMA_TO_SOCINTER    0

#define BLOCK_TRF_INT_MODE    15
extern unsigned long bst_sip_special_address_rw(u_int64_t x1, u_int64_t x2, u_int64_t x3);
struct bst_axi_dma_hcfg {
	u32	nr_channels;
	u32	nr_masters;
	u32	chan_start;
	//u32	cmn_irq;
	u32	m_data_width;
	u32	block_size[DMAC_MAX_CHANNELS];
	u32	priority[DMAC_MAX_CHANNELS];
	u32	chan_irq[DMAC_MAX_CHANNELS];
	/* maximum supported axi burst length */
	u32	axi_rw_burst_len;
	/* Register map for DMAX_NUM_CHANNELS <= 8 */
	bool	reg_map_8_channels;
	bool	restrict_axi_burst_len;
};

struct axi_dma_chan {
	struct axi_dma_chip		*chip;
	void __iomem			*chan_regs;
	u8				id;
	u8				hw_chan;
	u8				hw_handshake_num;
	atomic_t			descs_allocated;

	struct dma_pool			*desc_pool;
	struct virt_dma_chan		vc;
 
  struct virt_dma_desc *vd_issueing;
  
	//struct axi_dma_desc		*desc;
	struct dma_slave_config		config;
	enum dma_transfer_direction	direction;
	bool				cyclic;
	/* these other elements are all protected by vc.lock */
	bool				is_paused;

	u64 block_num;

	unsigned long flags;
};

struct bst_axi_dma {
	struct dma_device	dma;
	struct bst_axi_dma_hcfg	*hdata;
	struct device_dma_parameters	dma_parms;
	struct reset_control	*reset;

	/* channels */
	struct axi_dma_chan	*chan;
};

struct axi_dma_chip {
	struct device		*dev;
	int			irq;
	void __iomem		*regs;
	void __iomem		*apb_regs;
	struct clk		*core_clk;
	struct clk		*cfgr_clk;
	struct bst_axi_dma	*bst;
};

/* LLI == Linked List Item */
struct __packed axi_dma_lli {
	__le64		sar;
	__le64		dar;
	__le32		block_ts_lo;
	__le32		block_ts_hi;
	__le64		llp;
	__le32		ctl_lo;
	__le32		ctl_hi;
	__le32		sstat;
	__le32		dstat;
	__le32		status_lo;
	__le32		status_hi;
	__le32		reserved_lo;
	__le32		reserved_hi;
};

struct axi_dma_hw_desc {
	struct axi_dma_lli	*lli;
	dma_addr_t		llp;
	u32			len;
};

struct axi_dma_desc {
	struct axi_dma_hw_desc	*hw_desc;

	struct virt_dma_desc		vd;
	struct axi_dma_chan		*chan;
	u32				completed_blocks;
	u32				length;
	u32				count;
	u32				period_len;
};

struct axi_dma_chan_config {
	u8 dst_multblk_type;
	u8 src_multblk_type;
	u8 dst_per;
	u8 src_per;
	u8 tt_fc;
	u8 prior;
	u8 hs_sel_dst;
	u8 hs_sel_src;
};

static inline struct device *dchan2dev(struct dma_chan *dchan)
{
	return &dchan->dev->device;
}

static inline struct device *chan2dev(struct axi_dma_chan *chan)
{
	return &chan->vc.chan.dev->device;
}

static inline struct axi_dma_desc *vd_to_axi_desc(struct virt_dma_desc *vd)
{
	return container_of(vd, struct axi_dma_desc, vd);
}

static inline struct axi_dma_chan *vc_to_axi_dma_chan(struct virt_dma_chan *vc)
{
	return container_of(vc, struct axi_dma_chan, vc);
}

static inline struct axi_dma_chan *dchan_to_axi_dma_chan(struct dma_chan *dchan)
{
	return vc_to_axi_dma_chan(to_virt_chan(dchan));
}


#define COMMON_REG_LEN		0x100
#define CHAN_REG_LEN		0x100

/* Common registers offset */
#define DMAC_ID			0x000 /* R DMAC ID */
#define DMAC_COMPVER		0x008 /* R DMAC Component Version */
#define DMAC_CFG		0x010 /* R/W DMAC Configuration */
#define DMAC_CHEN		0x018 /* R/W DMAC Channel Enable */
#define DMAC_CHEN_L		0x018 /* R/W DMAC Channel Enable 00-31 */
#define DMAC_CHEN_H		0x01C /* R/W DMAC Channel Enable 32-63 */
#define DMAC_CHSUSPREG		0x020 /* R/W DMAC Channel Suspend */
#define DMAC_CHABORTREG		0x028 /* R/W DMAC Channel Abort */
#define DMAC_INTSTATUS		0x030 /* R DMAC Interrupt Status */
#define DMAC_COMMON_INTCLEAR	0x038 /* W DMAC Interrupt Clear */
#define DMAC_COMMON_INTSTATUS_ENA 0x040 /* R DMAC Interrupt Status Enable */
#define DMAC_COMMON_INTSIGNAL_ENA 0x048 /* R/W DMAC Interrupt Signal Enable */
#define DMAC_COMMON_INTSTATUS	0x050 /* R DMAC Interrupt Status */
#define DMAC_RESET		0x058 /* R DMAC Reset Register1 */

/* DMA channel registers offset */
#define CH_SAR			0x000 /* R/W Chan Source Address */
#define CH_DAR			0x008 /* R/W Chan Destination Address */
#define CH_BLOCK_TS		0x010 /* R/W Chan Block Transfer Size */
#define CH_CTL			0x018 /* R/W Chan Control */
#define CH_CTL_L		0x018 /* R/W Chan Control 00-31 */
#define CH_CTL_H		0x01C /* R/W Chan Control 32-63 */
#define CH_CFG			0x020 /* R/W Chan Configuration */
#define CH_CFG_L		0x020 /* R/W Chan Configuration 00-31 */
#define CH_CFG_H		0x024 /* R/W Chan Configuration 32-63 */
#define CH_LLP			0x028 /* R/W Chan Linked List Pointer */
#define CH_STATUS		0x030 /* R Chan Status */
#define CH_SWHSSRC		0x038 /* R/W Chan SW Handshake Source */
#define CH_SWHSDST		0x040 /* R/W Chan SW Handshake Destination */
#define CH_BLK_TFR_RESUMEREQ	0x048 /* W Chan Block Transfer Resume Req */
#define CH_AXI_ID		0x050 /* R/W Chan AXI ID */
#define CH_AXI_QOS		0x058 /* R/W Chan AXI QOS */
#define CH_SSTAT		0x060 /* R Chan Source Status */
#define CH_DSTAT		0x068 /* R Chan Destination Status */
#define CH_SSTATAR		0x070 /* R/W Chan Source Status Fetch Addr */
#define CH_DSTATAR		0x078 /* R/W Chan Destination Status Fetch Addr */
#define CH_INTSTATUS_ENA	0x080 /* R/W Chan Interrupt Status Enable */
#define CH_INTSTATUS		0x088 /* R/W Chan Interrupt Status */
#define CH_INTSIGNAL_ENA	0x090 /* R/W Chan Interrupt Signal Enable */
#define CH_INTCLEAR		0x098 /* W Chan Interrupt Clear */

/* These Apb registers are used by Intel KeemBay SoC */
#define DMAC_APB_CFG		0x000 /* DMAC Apb Configuration Register */
#define DMAC_APB_STAT		0x004 /* DMAC Apb Status Register */
#define DMAC_APB_DEBUG_STAT_0	0x008 /* DMAC Apb Debug Status Register 0 */
#define DMAC_APB_DEBUG_STAT_1	0x00C /* DMAC Apb Debug Status Register 1 */
#define DMAC_APB_HW_HS_SEL_0	0x010 /* DMAC Apb HW HS register 0 */
#define DMAC_APB_HW_HS_SEL_1	0x014 /* DMAC Apb HW HS register 1 */
#define DMAC_APB_LPI		0x018 /* DMAC Apb Low Power Interface Reg */
#define DMAC_APB_BYTE_WR_CH_EN	0x01C /* DMAC Apb Byte Write Enable */
#define DMAC_APB_HALFWORD_WR_CH_EN	0x020 /* DMAC Halfword write enables */

#define UNUSED_CHANNEL		0x3F /* Set unused DMA channel to 0x3F */
#define DMA_APB_HS_SEL_BIT_SIZE	0x08 /* HW handshake bits per channel */
#define DMA_APB_HS_SEL_MASK	0xFF /* HW handshake select masks */
#define MAX_BLOCK_SIZE		0x400000 /* 1024 blocks * 4 bytes data width */
#define DMA_REG_MAP_CH_REF	0x08 /* Channel count to choose register map */

/* DMAC_CFG */
#define DMAC_EN_POS			0
#define DMAC_EN_MASK			BIT(DMAC_EN_POS)

#define INT_EN_POS			1
#define INT_EN_MASK			BIT(INT_EN_POS)

/* DMAC_CHEN */
#define DMAC_CHAN_EN_SHIFT		0
#define DMAC_CHAN_EN_WE_SHIFT		8

#define DMAC_CHAN_SUSP_SHIFT		16
#define DMAC_CHAN_SUSP_WE_SHIFT		24

/* DMAC_CHEN2 */
#define DMAC_CHAN_EN2_WE_SHIFT		16

/* DMAC_CHSUSP */
#define DMAC_CHAN_SUSP2_SHIFT		0
#define DMAC_CHAN_SUSP2_WE_SHIFT	16

/* CH_CTL_H */
#define CH_CTL_H_ARLEN_EN		BIT(6)
#define CH_CTL_H_ARLEN_POS		7
#define CH_CTL_H_AWLEN_EN		BIT(15)
#define CH_CTL_H_AWLEN_POS		16
#define CH_CTL_H_SRC_STAT_EN	BIT(24)
#define CH_CTL_H_DST_STAT_EN	BIT(25)
#define CH_CTL_H_IOC_BlkTf_EN	BIT(26)

#define INT_CMN_UDF_REG_MASK	BIT(8)
#define INT_CMN_WOH_ERR_MASK	BIT(3)
#define INT_CMN_R2WO_ERR_MASK	BIT(2)
#define INT_CMN_W2RO_ERR_MASK	BIT(1)
#define INT_CMN_DEC_ERR_MASK	BIT(0)

#define INT_CMN_MASK (INT_CMN_UDF_REG_MASK | INT_CMN_WOH_ERR_MASK  | INT_CMN_DEC_ERR_MASK)

enum {
	BSTAXIDMAC_ARWLEN_1		= 0,
	BSTAXIDMAC_ARWLEN_2		= 1,
	BSTAXIDMAC_ARWLEN_4		= 3,
	BSTAXIDMAC_ARWLEN_8		= 7,
	BSTAXIDMAC_ARWLEN_16		= 15,
	BSTAXIDMAC_ARWLEN_32		= 31,
	BSTAXIDMAC_ARWLEN_64		= 63,
	BSTAXIDMAC_ARWLEN_128		= 127,
	BSTAXIDMAC_ARWLEN_256		= 255,
	BSTAXIDMAC_ARWLEN_MIN		= BSTAXIDMAC_ARWLEN_1,
	BSTAXIDMAC_ARWLEN_MAX		= BSTAXIDMAC_ARWLEN_256
};

#define CH_CTL_H_LLI_LAST		BIT(30)
#define CH_CTL_H_LLI_VALID		BIT(31)
#define CH_CTL_H_BLK_TRF		BIT(26)
/* CH_CTL_L */
#define CH_CTL_L_LAST_WRITE_EN		BIT(30)

#define CH_CTL_L_DST_MSIZE_POS		18
#define CH_CTL_L_SRC_MSIZE_POS		14

enum {
	BSTAXIDMAC_BURST_TRANS_LEN_1	= 0,
	BSTAXIDMAC_BURST_TRANS_LEN_4,
	BSTAXIDMAC_BURST_TRANS_LEN_8,
	BSTAXIDMAC_BURST_TRANS_LEN_16,
	BSTAXIDMAC_BURST_TRANS_LEN_32,
	BSTAXIDMAC_BURST_TRANS_LEN_64,
	BSTAXIDMAC_BURST_TRANS_LEN_128,
	BSTAXIDMAC_BURST_TRANS_LEN_256,
	BSTAXIDMAC_BURST_TRANS_LEN_512,
	BSTAXIDMAC_BURST_TRANS_LEN_1024
};

#define CH_CTL_L_DST_WIDTH_POS		11
#define CH_CTL_L_SRC_WIDTH_POS		8

#define CH_CTL_L_DST_INC_POS		6
#define CH_CTL_L_SRC_INC_POS		4
enum {
	BSTAXIDMAC_CH_CTL_L_INC		= 0,
	BSTAXIDMAC_CH_CTL_L_NOINC
};

#define CH_CTL_L_DST_MAST		BIT(2)
#define CH_CTL_L_SRC_MAST		BIT(0)

/* CH_CFG_H */
#define CH_CFG_H_DST_OSR_LMT_POS	27
#define CH_CFG_H_SRC_OSR_LMT_POS	23
#define CH_CFG_H_PRIORITY_POS		17
#define CH_CFG_H_DST_PER_POS		12
#define CH_CFG_H_SRC_PER_POS		7
#define CH_CFG_H_HS_SEL_DST_POS		4
#define CH_CFG_H_HS_SEL_SRC_POS		3
enum {
	BSTAXIDMAC_HS_SEL_HW		= 0,
	BSTAXIDMAC_HS_SEL_SW
};

#define CH_CFG_H_TT_FC_POS		0
enum {
	BSTAXIDMAC_TT_FC_MEM_TO_MEM_DMAC	= 0,
	BSTAXIDMAC_TT_FC_MEM_TO_PER_DMAC,
	BSTAXIDMAC_TT_FC_PER_TO_MEM_DMAC,
	BSTAXIDMAC_TT_FC_PER_TO_PER_DMAC,
	BSTAXIDMAC_TT_FC_PER_TO_MEM_SRC,
	BSTAXIDMAC_TT_FC_PER_TO_PER_SRC,
	BSTAXIDMAC_TT_FC_MEM_TO_PER_DST,
	BSTAXIDMAC_TT_FC_PER_TO_PER_DST
};

/* CH_CFG_L */
#define CH_CFG_L_DST_MULTBLK_TYPE_POS	2
#define CH_CFG_L_SRC_MULTBLK_TYPE_POS	0
enum {
	BSTAXIDMAC_MBLK_TYPE_CONTIGUOUS	= 0,
	BSTAXIDMAC_MBLK_TYPE_RELOAD,
	BSTAXIDMAC_MBLK_TYPE_SHADOW_REG,
	BSTAXIDMAC_MBLK_TYPE_LL
};

/* CH_CFG2 */
#define CH_CFG2_L_SRC_PER_POS		4
#define CH_CFG2_L_DST_PER_POS		11

#define CH_CFG2_H_TT_FC_POS		0
#define CH_CFG2_H_HS_SEL_SRC_POS	3
#define CH_CFG2_H_HS_SEL_DST_POS	4
#define CH_CFG2_H_PRIORITY_POS		15

/**
 * BST AXI DMA channel interrupts
 *
 * @BSTAXIDMAC_IRQ_NONE: Bitmask of no one interrupt
 * @BSTAXIDMAC_IRQ_BLOCK_TRF: Block transfer complete
 * @BSTAXIDMAC_IRQ_DMA_TRF: Dma transfer complete
 * @BSTAXIDMAC_IRQ_SRC_TRAN: Source transaction complete
 * @BSTAXIDMAC_IRQ_DST_TRAN: Destination transaction complete
 * @BSTAXIDMAC_IRQ_SRC_DEC_ERR: Source decode error
 * @BSTAXIDMAC_IRQ_DST_DEC_ERR: Destination decode error
 * @BSTAXIDMAC_IRQ_SRC_SLV_ERR: Source slave error
 * @BSTAXIDMAC_IRQ_DST_SLV_ERR: Destination slave error
 * @BSTAXIDMAC_IRQ_LLI_RD_DEC_ERR: LLI read decode error
 * @BSTAXIDMAC_IRQ_LLI_WR_DEC_ERR: LLI write decode error
 * @BSTAXIDMAC_IRQ_LLI_RD_SLV_ERR: LLI read slave error
 * @BSTAXIDMAC_IRQ_LLI_WR_SLV_ERR: LLI write slave error
 * @BSTAXIDMAC_IRQ_INVALID_ERR: LLI invalid error or Shadow register error
 * @BSTAXIDMAC_IRQ_MULTIBLKTYPE_ERR: Slave Interface Multiblock type error
 * @BSTAXIDMAC_IRQ_DEC_ERR: Slave Interface decode error
 * @BSTAXIDMAC_IRQ_WR2RO_ERR: Slave Interface write to read only error
 * @BSTAXIDMAC_IRQ_RD2RWO_ERR: Slave Interface read to write only error
 * @BSTAXIDMAC_IRQ_WRONCHEN_ERR: Slave Interface write to channel error
 * @BSTAXIDMAC_IRQ_SHADOWREG_ERR: Slave Interface shadow reg error
 * @BSTAXIDMAC_IRQ_WRONHOLD_ERR: Slave Interface hold error
 * @BSTAXIDMAC_IRQ_LOCK_CLEARED: Lock Cleared Status
 * @BSTAXIDMAC_IRQ_SRC_SUSPENDED: Source Suspended Status
 * @BSTAXIDMAC_IRQ_SUSPENDED: Channel Suspended Status
 * @BSTAXIDMAC_IRQ_DISABLED: Channel Disabled Status
 * @BSTAXIDMAC_IRQ_ABORTED: Channel Aborted Status
 * @BSTAXIDMAC_IRQ_ALL_ERR: Bitmask of all error interrupts
 * @BSTAXIDMAC_IRQ_ALL: Bitmask of all interrupts
 */
enum {
	BSTAXIDMAC_IRQ_NONE		= 0,
	BSTAXIDMAC_IRQ_BLOCK_TRF		= BIT(0),
	BSTAXIDMAC_IRQ_DMA_TRF		= BIT(1),
	BSTAXIDMAC_IRQ_SRC_TRAN		= BIT(3),
	BSTAXIDMAC_IRQ_DST_TRAN		= BIT(4),
	BSTAXIDMAC_IRQ_SRC_DEC_ERR	= BIT(5),
	BSTAXIDMAC_IRQ_DST_DEC_ERR	= BIT(6),
	BSTAXIDMAC_IRQ_SRC_SLV_ERR	= BIT(7),
	BSTAXIDMAC_IRQ_DST_SLV_ERR	= BIT(8),
	BSTAXIDMAC_IRQ_LLI_RD_DEC_ERR	= BIT(9),
	BSTAXIDMAC_IRQ_LLI_WR_DEC_ERR	= BIT(10),
	BSTAXIDMAC_IRQ_LLI_RD_SLV_ERR	= BIT(11),
	BSTAXIDMAC_IRQ_LLI_WR_SLV_ERR	= BIT(12),
	BSTAXIDMAC_IRQ_INVALID_ERR	= BIT(13),
	BSTAXIDMAC_IRQ_MULTIBLKTYPE_ERR	= BIT(14),
	BSTAXIDMAC_IRQ_DEC_ERR		= BIT(16),
	BSTAXIDMAC_IRQ_WR2RO_ERR		= BIT(17),
	BSTAXIDMAC_IRQ_RD2RWO_ERR	= BIT(18),
	BSTAXIDMAC_IRQ_WRONCHEN_ERR	= BIT(19),
	BSTAXIDMAC_IRQ_SHADOWREG_ERR	= BIT(20),
	BSTAXIDMAC_IRQ_WRONHOLD_ERR	= BIT(21),
	BSTAXIDMAC_IRQ_LOCK_CLEARED	= BIT(27),
	BSTAXIDMAC_IRQ_SRC_SUSPENDED	= BIT(28),
	BSTAXIDMAC_IRQ_SUSPENDED		= BIT(29),
	BSTAXIDMAC_IRQ_DISABLED		= BIT(30),
	BSTAXIDMAC_IRQ_ABORTED		= BIT(31),
	BSTAXIDMAC_IRQ_ALL_ERR		= (GENMASK(21, 16) | GENMASK(14, 5)),
	BSTAXIDMAC_IRQ_ALL		= GENMASK(31, 0)
};

enum {
	BSTAXIDMAC_TRANS_WIDTH_8		= 0,
	BSTAXIDMAC_TRANS_WIDTH_16,
	BSTAXIDMAC_TRANS_WIDTH_32,
	BSTAXIDMAC_TRANS_WIDTH_64,
	BSTAXIDMAC_TRANS_WIDTH_128,
	BSTAXIDMAC_TRANS_WIDTH_256,
	BSTAXIDMAC_TRANS_WIDTH_512,
	BSTAXIDMAC_TRANS_WIDTH_MAX	= BSTAXIDMAC_TRANS_WIDTH_512
};

#define CH_CFG_L_DST_PERIPHERAL_POS 11
#define CH_CFG_L_SRC_PERIPHERAL_POS 4
/*
 * BST Soc gdma slave ID reference table.(slave_id)
 */
enum {
	LB_SOC_LSP_1_PCM_TDM1_DMA_RX = 0,
	LB_SOC_LSP_1_PCM_TDM1_DMA_TX = 1,
	LB_SOC_LSP_1_PCM_TDM0_DMA_RX = 2,
	LB_SOC_LSP_1_PCM_TDM0_DMA_TX = 3,

	LB_SOC_LSP_1_I2S_M1_DMA_RX_CHAN0 = 4,
	LB_SOC_LSP_1_I2S_M1_DMA_RX_CHAN1 = 5,
	LB_SOC_LSP_1_I2S_M1_DMA_RX_CHAN2 = 6,
	LB_SOC_LSP_1_I2S_M1_DMA_RX_CHAN3 = 7,

	LB_SOC_LSP_1_I2S_M1_DMA_TX_CHAN0 = 8,
	LB_SOC_LSP_1_I2S_M1_DMA_TX_CHAN1 = 9,
	LB_SOC_LSP_1_I2S_M1_DMA_TX_CHAN2 = 10,
	LB_SOC_LSP_1_I2S_M1_DMA_TX_CHAN3 = 11,

	LB_SOC_LSP_1_I2S_M0_DMA_RX_CHAN0 = 12,
	LB_SOC_LSP_1_I2S_M0_DMA_RX_CHAN1 = 13,
	LB_SOC_LSP_1_I2S_M0_DMA_RX_CHAN2 = 14,
	LB_SOC_LSP_1_I2S_M0_DMA_RX_CHAN3 = 15,

	LB_SOC_LSP_1_I2S_M0_DMA_TX_CHAN0 = 16,
	LB_SOC_LSP_1_I2S_M0_DMA_TX_CHAN1 = 17,
	LB_SOC_LSP_1_I2S_M0_DMA_TX_CHAN2 = 18,
	LB_SOC_LSP_1_I2S_M0_DMA_TX_CHAN3 = 19,

	LB_SOC_LSP_1_UART1_DMA_RX = 20,
	LB_SOC_LSP_1_UART1_DMA_TX = 21,
	LB_SOC_LSP_1_UART0_DMA_RX = 22,
	LB_SOC_LSP_1_UART0_DMA_TX = 23,

	LB_SOC_LSP_1_SSI_S_DMA_RX = 24,
	LB_SOC_LSP_1_SSI_S_DMA_TX = 25,
	LB_SOC_LSP_1_SSI_M_DMA_RX = 26,
	LB_SOC_LSP_1_SSI_M_DMA_TX = 27,

	LB_SOC_LSP_1_I2C3_DMA_RX = 28,
	LB_SOC_LSP_1_I2C2_DMA_RX = 29,
	LB_SOC_LSP_1_I2C1_DMA_TX = 30,
	LB_SOC_LSP_1_I2C0_DMA_TX = 31,

	LB_SOC_LSP_0_PCM_TDM1_DMA_RX = 32,
	LB_SOC_LSP_0_PCM_TDM1_DMA_TX = 33,
	LB_SOC_LSP_0_PCM_TDM0_DMA_RX = 34,
	LB_SOC_LSP_0_PCM_TDM0_DMA_TX = 35,

	LB_SOC_LSP_0_I2S_M1_DMA_RX_CHAN0 = 36,
	LB_SOC_LSP_0_I2S_M1_DMA_RX_CHAN1 = 37,
	LB_SOC_LSP_0_I2S_M1_DMA_RX_CHAN2 = 38,
	LB_SOC_LSP_0_I2S_M1_DMA_RX_CHAN3 = 39,

	LB_SOC_LSP_0_I2S_M1_DMA_TX_CHAN0 = 40,
	LB_SOC_LSP_0_I2S_M1_DMA_TX_CHAN1 = 41,
	LB_SOC_LSP_0_I2S_M1_DMA_TX_CHAN2 = 42,
	LB_SOC_LSP_0_I2S_M1_DMA_TX_CHAN3 = 43,

	LB_SOC_LSP_0_I2S_M0_DMA_RX_CHAN0 = 44,
	LB_SOC_LSP_0_I2S_M0_DMA_RX_CHAN1 = 45,
	LB_SOC_LSP_0_I2S_M0_DMA_RX_CHAN2 = 46,
	LB_SOC_LSP_0_I2S_M0_DMA_RX_CHAN3 = 47,

	LB_SOC_LSP_0_I2S_M0_DMA_TX_CHAN0 = 48,
	LB_SOC_LSP_0_I2S_M0_DMA_TX_CHAN1 = 49,
	LB_SOC_LSP_0_I2S_M0_DMA_TX_CHAN2 = 50,
	LB_SOC_LSP_0_I2S_M0_DMA_TX_CHAN3 = 51,

	LB_SOC_LSP_0_UART1_DMA_RX = 52,
	LB_SOC_LSP_0_UART1_DMA_TX = 53,
	LB_SOC_LSP_0_UART0_DMA_RX = 54,
	LB_SOC_LSP_0_UART0_DMA_TX = 55,

	LB_SOC_LSP_0_SSI_S_DMA_RX = 56,
	LB_SOC_LSP_0_SSI_S_DMA_TX = 57,
	LB_SOC_LSP_0_SSI_M_DMA_RX = 58,
	LB_SOC_LSP_0_SSI_M_DMA_TX = 59,

	LB_SOC_LSP_0_I2C3_DMA = 60,
	LB_SOC_LSP_0_I2C2_DMA = 61,
	LB_SOC_LSP_0_I2C1_DMA = 62,
	LB_SOC_LSP_0_I2C0_DMA = 63,

	CHAN_SLAVE_MAX_REQ,
};

#endif /* _AXI_DMA_PLATFORM_H */
