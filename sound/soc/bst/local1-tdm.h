// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (ST) 2012 Rajeev Kumar (rajeevkumar.linux@gmail.com)
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __DESIGNWARE_LOCAL_TDM_H
#define __DESIGNWARE_LOCAL_TDM_H

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/types.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm.h>
#include <sound/bst_tdm.h>

/* common register for all channel */
#define IER			0x000
#define IRER		0x004
#define ITER		0x008
#define CER			0x00C
#define CCR			0x010
#define RXFFR		0x014
#define TXFFR		0x018
#define DMACR		0x200

/* Interrupt status register fields */
#define ISR_TXFO	BIT(5)
#define ISR_TXFE	BIT(4)
#define ISR_RXFO	BIT(1)
#define ISR_RXDA	BIT(0)

/* I2STxRxRegisters for all channels */
#define LRBR_LTHR(x)	(0x40 * x + 0x020)
#define RRBR_RTHR(x)	(0x40 * x + 0x024)
#define RER(x)		(0x40 * x + 0x028)
#define TER(x)		(0x40 * x + 0x02C)
#define RCR(x)		(0x40 * x + 0x030)
#define TCR(x)		(0x40 * x + 0x034)
#define ISR(x)		(0x40 * x + 0x038)
#define IMR(x)		(0x40 * x + 0x03C)
#define ROR(x)		(0x40 * x + 0x040)
#define TOR(x)		(0x40 * x + 0x044)
#define RFCR(x)		(0x40 * x + 0x048)
#define TFCR(x)		(0x40 * x + 0x04C)
#define RFF(x)		(0x40 * x + 0x050)
#define TFF(x)		(0x40 * x + 0x054)

/* I2SCOMPRegisters */
#define TDM_COMP_PARAM_2	0x01F0
#define TDM_COMP_PARAM_1	0x01F4
#define I2S_COMP_VERSION	0x01F8
#define I2S_COMP_TYPE		0x01FC

/*
 * Component parameter register fields - define the I2S block's
 * configuration.
 */
#define	COMP1_TX_WORDSIZE_3(r)	(((r) & GENMASK(27, 25)) >> 25)
#define	COMP1_TX_WORDSIZE_2(r)	(((r) & GENMASK(24, 22)) >> 22)
#define	COMP1_TX_WORDSIZE_1(r)	(((r) & GENMASK(21, 19)) >> 19)
#define	COMP1_TX_WORDSIZE_0(r)	(((r) & GENMASK(18, 16)) >> 16)
#define	COMP1_TX_CHANNELS(r)	(((r) & GENMASK(10, 9)) >> 9)
#define	COMP1_RX_CHANNELS(r)	(((r) & GENMASK(8, 7)) >> 7)
#define	COMP1_RX_ENABLED(r)	(((r) & BIT(6)) >> 6)
#define	COMP1_TX_ENABLED(r)	(((r) & BIT(5)) >> 5)
#define	COMP1_MODE_EN(r)	(((r) & BIT(4)) >> 4)
#define	COMP1_FIFO_DEPTH_GLOBAL(r)	(((r) & GENMASK(3, 2)) >> 2)
#define	COMP1_APB_DATA_WIDTH(r)	(((r) & GENMASK(1, 0)) >> 0)

#define	COMP2_RX_WORDSIZE_3(r)	(((r) & GENMASK(12, 10)) >> 10)
#define	COMP2_RX_WORDSIZE_2(r)	(((r) & GENMASK(9, 7)) >> 7)
#define	COMP2_RX_WORDSIZE_1(r)	(((r) & GENMASK(5, 3)) >> 3)
#define	COMP2_RX_WORDSIZE_0(r)	(((r) & GENMASK(2, 0)) >> 0)

/* Number of entries in WORDSIZE and DATA_WIDTH parameter registers */
#define	COMP_MAX_WORDSIZE	(1 << 3)
#define	COMP_MAX_DATA_WIDTH	(1 << 2)

/* Number of slots to enable in Transmit Enable Register */
#define	TX_SLOTS_EN(r)	((~((~0U)<<(r)))<<(8))
#define	RX_SLOTS_EN(r)	((~((~0U)<<(r)))<<(8))

#define TDM_REG_TSLOT(r)		(0x4 * r + 0x224)
#define TDM_REG_RSLOT(r)		(0x4 * r + 0x224)

#define MAX_CHANNEL_NUM		16      //适配 pcm3168a
#define MIN_CHANNEL_NUM		1


union dw_tdm_snd_dma_data {
	struct tdm_dma_data pd;
	struct snd_dmaengine_dai_dma_data dt;
};


enum tdm_slot_num_t {
	TDM_SLOT_NUM_1 = 1,
	TDM_SLOT_NUM_2 = 2,
	TDM_SLOT_NUM_3 = 3,
	TDM_SLOT_NUM_4 = 4,
	TDM_SLOT_NUM_5 = 5,
	TDM_SLOT_NUM_6 = 6,
	TDM_SLOT_NUM_7 = 7,
	TDM_SLOT_NUM_8 = 8,
	TDM_SLOT_NUM_9 = 9,
	TDM_SLOT_NUM_10 = 10,
	TDM_SLOT_NUM_11 = 11,
	TDM_SLOT_NUM_12 = 12,
	TDM_SLOT_NUM_13 = 13,
	TDM_SLOT_NUM_14 = 14,
	TDM_SLOT_NUM_15 = 15,
	TDM_SLOT_NUM_16 = 16,
};



// 裸驱

typedef struct _reg_struct {
        u32 addr;
        u32 default_val;
        u32 bit_strobe;
        char *name;
} reg_struct; 


#define TDM0_REG_IER			(   0x0   )
#define TDM0_REG_IRER			(   0x4   )
#define TDM0_REG_ITER			(   0x8   )
#define TDM0_REG_CER			(   0xC   )
#define TDM0_REG_CCR			(   0x10  )
#define TDM0_REG_RXFFR			(   0x14  )
#define TDM0_REG_TXFFR			(   0x18  )
#define TDM0_REG_SR			    (   0x1C  )

#define TDM0_REG_LRBR_BASE		(   0x20  )
#define TDM0_REG_LTHR_BASE		(   0x20  )
#define TDM0_REG_RRBR_BASE		(   0x24  )
#define TDM0_REG_RTHR_BASE		(   0x24  )
#define TDM0_REG_RER_BASE		(   0x28  )
#define TDM0_REG_TER_BASE		(   0x2C  )
#define TDM0_REG_RCR_BASE		(   0x30  )
#define TDM0_REG_TCR_BASE		(   0x34  )
#define TDM0_REG_ISR_BASE		(   0x38  )
#define TDM0_REG_IMR_BASE		(   0x3C  )
#define TDM0_REG_ROR_BASE		(   0x40  )
#define TDM0_REG_TOR_BASE		(   0x44  )
#define TDM0_REG_RFCR_BASE		(   0x48  )
#define TDM0_REG_TFCR_BASE		(   0x4C  )
#define TDM0_REG_RFF_BASE		(   0x50  )
#define TDM0_REG_TFF_BASE		(   0x54  )
#define TDM0_REG_RXDMA			(   0x1C0 )
#define TDM0_REG_RRXDMA			(   0x1C4 )
#define TDM0_REG_TXDMA			(   0x1C8 )
#define TDM0_REG_RTXDMA			(   0x1CC )
#define TDM0_REG_COMP_PARAM_2		(   0x1F0 )
#define TDM0_REG_COMP_PARAM_1		(   0x1F4 )
#define TDM0_REG_COMP_VERSION		(   0x1F8 )
#define TDM0_REG_COMP_TYPE		(   0x1FC )
#define TDM0_REG_DMACR			(   0x200 )
#define TDM0_REG_RXDMA_CH_BASE		(   0x204 )
#define TDM0_REG_TXDMA_CH_BASE		(   0x214 )

#define TDM0_REG_TSLOTx_BASE		(  0x224 )
#define TDM0_REG_RSLOTx_BASE		(  0x224 )


#define TDM1_REG_TSLOTx_BASE		(   0x214 )

#define TDM_REG_N(base, n)		( (base) + 0x40*(n) )
#define TDM_REG_DMA_CH_N(base, n)	( (base) + 0x4*(n) )


struct dw_tdm_dev {
	void __iomem *tdm_base;
	resource_size_t phy_base;
	struct clk *clk;
	struct pinctrl *pin;
	int active;
	int lsp_crm;
	unsigned int capability;
	unsigned int quirks;
	unsigned int tdm_reg_comp1;
	unsigned int tdm_reg_comp2;
	struct device *dev;
	u32 ccr;
	u32 xfer_resolution;
	u32 fifo_th;
	u32 slot_cnt;

	/* data related to DMA transfers b/w i2s and DMAC */
	union dw_tdm_snd_dma_data play_dma_data;
	union dw_tdm_snd_dma_data capture_dma_data;
	struct tdm_clk_config_data config;
	int (*tdm_clk_cfg)(struct tdm_clk_config_data *config);

	/* data related to PIO transfers */
	bool use_pio;
	bool use_dma;
	struct snd_pcm_substream __rcu *tx_substream;
	struct snd_pcm_substream __rcu *rx_substream;
	unsigned int (*tx_fn)(struct dw_tdm_dev *dev,
			struct snd_pcm_runtime *runtime, unsigned int tx_ptr,
			bool *period_elapsed);
	unsigned int (*rx_fn)(struct dw_tdm_dev *dev,
			struct snd_pcm_runtime *runtime, unsigned int rx_ptr,
			bool *period_elapsed);
	unsigned int tx_ptr;
	unsigned int rx_ptr;
};

#if IS_ENABLED(CONFIG_SND_BST_PCM_TDM)
void dw_pcm_tdm_push_tx(struct dw_tdm_dev *dev);
void dw_pcm_tdm_pop_rx(struct dw_tdm_dev *dev);
int dw_tdm_pcm_register(struct platform_device *pdev);
#else
void dw_pcm_tdm_push_tx(struct dw_tdm_dev *dev) { }
void dw_pcm_tdm_pop_rx(struct dw_tdm_dev *dev) { }
int dw_tdm_pcm_register(struct platform_device *pdev)
{
	return -EINVAL;
}
#endif

#endif
