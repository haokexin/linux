// SPDX-License-Identifier: GPL-2.0+
/*
 * ALSA SoC TDM Audio Layer
 * TDM controller driver for BST TDM
 * sound/soc/bst/bst-tdm.c
 *
 * Copyright (C) 2010 ST Microelectronics
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <sound/bst_tdm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>
#include "local1-tdm.h"
#include <linux/debugfs.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/dma/bst-dma.h>
#include "../../../drivers/pinctrl/core.h"

#define TDM_WCLK (884736000UL)
//#define TDM_WCLK   (36864000UL)
#define CLK_DIV_MIN 1UL
#define CLK_DIV_MAX 0xFFFFUL

#define SOC_LSP0_CRM_REG_CTRL_BASE 0x20000000
#define SOC_LSP1_CRM_REG_CTRL_BASE 0x20020000
void __iomem *sc_lsp0_crm_ctrl_base;
void __iomem *sc_lsp1_crm_ctrl_base;
struct dentry *tdm_debugfs;
static int tdm_num = 1;
static int dma_num = 0;
static struct bst_dma_snd_peripheral_cfg tdm_dma_peripheral_cfg = {
	/**
	 * tell the DMA driver to select gdma block transfer mode.
	 **/
	.dma_transfer_mode = 1U,
};


static int send_safety_usrmsg(u32 fault_code, u32 strategy_code)
{
	return -EINVAL;
}

static inline void tdm_write_reg(void __iomem *io_base, int reg, u32 val)
{
	writel(val, io_base + reg);
}

static inline u32 tdm_read_reg(void __iomem *io_base, int reg)
{
	return readl(io_base + reg);
}

static inline void tdm_disable_channels(struct dw_tdm_dev *dev, u32 stream)
{

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) 
			tdm_write_reg(dev->tdm_base, TER(0), 0);
	else 
			tdm_write_reg(dev->tdm_base, RER(0), 0);

	if (dev->use_dma) {

		u32 dmacr_reg = tdm_read_reg(dev->tdm_base, DMACR);
		pr_info(" %s, dmacr_reg: 0x%x\n", __func__,dmacr_reg);
		if (stream == SNDRV_PCM_STREAM_PLAYBACK)
			tdm_write_reg(dev->tdm_base, DMACR, dmacr_reg & (~BIT(17)));
		else if (stream == SNDRV_PCM_STREAM_CAPTURE)
			tdm_write_reg(dev->tdm_base, DMACR, dmacr_reg & (~BIT(16)));
		
		dmacr_reg = tdm_read_reg(dev->tdm_base, DMACR);
		pr_info(" %s, dmacr_reg: 0x%x\n", __func__, dmacr_reg);
	}

}

#if 1
static inline void tdm_clear_irqs(struct dw_tdm_dev *dev, u32 stream)
{

	u32 ret;
	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		tdm_read_reg(dev->tdm_base, TOR(0));
	}
	else {
		ret = tdm_read_reg(dev->tdm_base, ROR(0));
	}

}
#endif

#if 0
static inline void tdm_clear_irqs(struct dw_tdm_dev *dev, u32 stream)
{

	u32 i = 0;
	u32 ret;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		for (i = 0; i < 4; i++)
			tdm_read_reg(dev->tdm_base, TOR(i));
	} else {
		for (i = 0; i < 4; i++) {
			ret = tdm_read_reg(dev->tdm_base, ROR(i));
			if (ret == 0x1) {
				ret = tdm_read_reg(dev->tdm_base, ROR(i));
				if (ret == 0x1) {
					pr_info("%s, i:%d , ror : 0x%x \n", __func__, i, ret);
				}
			}
		}
	}
	
}
#endif

static inline void tdm_disable_irqs(struct dw_tdm_dev *dev, u32 stream,
				    int chan_nr)
{
	u32 irq;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
			irq = tdm_read_reg(dev->tdm_base, IMR(0));
			tdm_write_reg(dev->tdm_base, IMR(0), irq | 0x30);
	} else {
			irq = tdm_read_reg(dev->tdm_base, IMR(0));
			tdm_write_reg(dev->tdm_base, IMR(0), irq | 0x03);
	}
}

static inline void tdm_enable_irqs(struct dw_tdm_dev *dev, u32 stream,
				   int chan_nr)
{
	u32 irq;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		irq = tdm_read_reg(dev->tdm_base, IMR(0));
		tdm_write_reg(dev->tdm_base, IMR(0), irq & ~0x30);
		
	} else {
		irq = tdm_read_reg(dev->tdm_base, IMR(0));
		tdm_write_reg(dev->tdm_base, IMR(0), irq & ~0x03);
	}
}


static irqreturn_t tdm_irq_handler(int irq, void *dev_id)
{
	struct dw_tdm_dev *dev = dev_id;
	bool irq_valid = false;
	u32 isr;
	u32 imr;

	isr = tdm_read_reg(dev->tdm_base, ISR(0));
	imr = tdm_read_reg(dev->tdm_base, IMR(0));

	tdm_clear_irqs(dev, SNDRV_PCM_STREAM_PLAYBACK);      //clear overrun irq
	tdm_clear_irqs(dev, SNDRV_PCM_STREAM_CAPTURE);

	/*
	 * Check if TX fifo is empty. If empty fill FIFO with samples
	 * NOTE: Only two channels supported
	 */
	if ((isr & ISR_TXFE) && dev->use_pio &&!(imr & ISR_TXFE)) {
		dw_pcm_tdm_push_tx(dev);
		irq_valid = true;
	}

	/*
	 * Data available. Retrieve samples from FIFO
	 * NOTE: Only two channels supported
	 */
	if ((isr & ISR_RXDA) && dev->use_pio &&!(imr & ISR_RXDA)) {
		dw_pcm_tdm_pop_rx(dev);
		irq_valid = true;
	}

	/* Error Handling: TX */
	if (isr & ISR_TXFO) {
		dev_err(dev->dev, "TX overrun\n");
//		send_safety_usrmsg(0xa00301, 1);
		irq_valid = true;
	}

	/* Error Handling: TX */
	if ((isr & ISR_RXFO) && !(isr & ISR_RXDA)) {
		dev_err(dev->dev, "RX overrun\n");
//		send_safety_usrmsg(0xa00302, 1);
		irq_valid = true;
	}

	if (irq_valid)
		return IRQ_HANDLED;
	else
		return IRQ_NONE;
}



static void tdm_start(struct dw_tdm_dev *dev,
		      struct snd_pcm_substream *substream)
{
	struct tdm_clk_config_data *config = &dev->config;
	int chan_nr;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		chan_nr = config->tx_chan_nr;
	} else {
		chan_nr = config->rx_chan_nr;
	}

	//set tdm format: I2S TDM , the slot count of the codec pcm3168a is 8, so default is 8
	tdm_write_reg(dev->tdm_base, IER, ((dev->slot_cnt-1)<<8)|(1<<5)|(1<<1)|(1<<0));
	if (!dev->use_dma)
		tdm_enable_irqs(dev, substream->stream, chan_nr);   //unmask all irqs

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		tdm_write_reg(dev->tdm_base, ITER, 1);
	else
		tdm_write_reg(dev->tdm_base, IRER, 1);

	tdm_write_reg(dev->tdm_base, CER, 1);
}

static void tdm_stop(struct dw_tdm_dev *dev,
		     struct snd_pcm_substream *substream)
{
	struct tdm_clk_config_data *config = &dev->config;
	int chan_nr;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		chan_nr = config->tx_chan_nr;
	} else {
		chan_nr = config->rx_chan_nr;
	}

	mdelay(10);
	tdm_clear_irqs(dev, substream->stream);    //clear tx/rx overrun irq
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		tdm_write_reg(dev->tdm_base, ITER, 0);
	else
		tdm_write_reg(dev->tdm_base, IRER, 0);

	tdm_disable_irqs(dev, substream->stream, chan_nr);    //mask all irqs

	if (!dev->active) {
		tdm_write_reg(dev->tdm_base, CER, 0);
		tdm_write_reg(dev->tdm_base, IER, 0);
	}
}

static void bst_tdm_tdm_sel_init(int lsp_crm)
{
	u32 reg_val;
	if (lsp_crm == 0) {
		writel(0xabcd1234, sc_lsp0_crm_ctrl_base + 0x84);
		reg_val = readl(sc_lsp0_crm_ctrl_base + 0x1c);
		reg_val = reg_val & (~(1<<15));
		writel(reg_val, sc_lsp0_crm_ctrl_base + 0x1c);
		writel(0x0, sc_lsp0_crm_ctrl_base + 0x84);
	}
	if (lsp_crm == 1) {
		writel(0xabcd1234, sc_lsp1_crm_ctrl_base + 0x84);
		reg_val = readl(sc_lsp1_crm_ctrl_base + 0x1c);
		reg_val = reg_val & (~(1<<15));
		writel(reg_val, sc_lsp1_crm_ctrl_base + 0x1c);
		writel(0x0, sc_lsp1_crm_ctrl_base + 0x84);
	}
}

static int dw_tdm_startup(struct snd_pcm_substream *substream,
			  struct snd_soc_dai *cpu_dai)
{
	struct dw_tdm_dev *dev = snd_soc_dai_get_drvdata(cpu_dai);
	union dw_tdm_snd_dma_data *dma_data = NULL;

	if (!(dev->capability & DWC_TDM_RECORD) && (substream->stream == SNDRV_PCM_STREAM_CAPTURE))
		return -EINVAL;

	if (!(dev->capability & DWC_TDM_PLAY) && (substream->stream == SNDRV_PCM_STREAM_PLAYBACK))
		return -EINVAL;

	if(dev->use_dma){
		dma_num++;
		pr_info(" %s, dma_num == %d\n", __func__, dma_num);
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dma_data = &dev->play_dma_data;
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		dma_data = &dev->capture_dma_data;
	snd_soc_dai_set_dma_data(cpu_dai, substream, (void *)dma_data);
	return 0;
}

static void dw_tdm_config(struct dw_tdm_dev *dev, int stream)
{

	struct tdm_clk_config_data *config = &dev->config;
	int chan_nr;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		chan_nr = config->tx_chan_nr;
	} else {
		chan_nr = config->rx_chan_nr;
	}

//	chan_nr = 8; //to test 8 channel

	tdm_disable_channels(dev, stream);

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		tdm_write_reg(dev->tdm_base, TCR(0),		//wlen
				    dev->xfer_resolution);
		tdm_write_reg(dev->tdm_base, TFCR(0),		//tx Data empty interrupt 的触发值
				    dev->fifo_th - 1);
		tdm_write_reg(dev->tdm_base, TER(0),        //Transmit Channel Enable , 使能 tx slot
					(TX_SLOTS_EN(chan_nr)|(1<<0)));   
	} else {
		tdm_write_reg(dev->tdm_base, RCR(0),
				    dev->xfer_resolution);
     	tdm_write_reg(dev->tdm_base, RFCR(0),
				    dev->fifo_th - 1);
		tdm_write_reg(dev->tdm_base, RER(0),
					(RX_SLOTS_EN(chan_nr)|(1<<0)));
	}

	if (dev->use_dma) {

		u32 dmacr_reg = tdm_read_reg(dev->tdm_base, DMACR);
		pr_info("%s DMACR == (0x%x)\n", __func__, dmacr_reg);

		if (stream == SNDRV_PCM_STREAM_PLAYBACK)
			tdm_write_reg(dev->tdm_base, DMACR, dmacr_reg | (BIT(17)));
		else if (stream == SNDRV_PCM_STREAM_CAPTURE)
			tdm_write_reg(dev->tdm_base, DMACR, dmacr_reg | (BIT(16)));

		dmacr_reg = tdm_read_reg(dev->tdm_base, DMACR);
		pr_info("%s DMACR == (0x%x)\n", __func__, dmacr_reg);
	}
}

static int bst_set_tdm_clk(struct dw_tdm_dev *dev, unsigned int freq, int lsp_crm)
{
	u32 tmp = 0;
	u32 clk_div = 0;

	if (freq != 0 ) {
		clk_div = TDM_WCLK / freq;
		if (clk_div < CLK_DIV_MIN || clk_div > CLK_DIV_MAX)
			clk_div = 0x240;

		if (lsp_crm == 0) {
			if(dev->phy_base == 0x2000D000) {
				pr_info("%s set tdm0 sclk\n", __func__);
				tmp = ioread32(sc_lsp0_crm_ctrl_base + 0x0c);
				iowrite32((tmp & (~(0xffff << 0))) | ((clk_div & 0xffff) << 0),
					sc_lsp0_crm_ctrl_base + 0x0c);
			} else if (dev->phy_base == 0x2000E000) {
				pr_info("%s set tdm1 sclk\n", __func__);
				tmp = ioread32(sc_lsp0_crm_ctrl_base + 0x178);
				iowrite32((tmp & (~(0xffff << 0))) | ((clk_div & 0xffff) << 0),
					sc_lsp0_crm_ctrl_base + 0x178);
			} else {
				/* do nothing */
			}
		} else if (lsp_crm == 1) {
			if(dev->phy_base == 0x2002D000) {
				pr_info("%s set tdm2 sclk\n", __func__);
				tmp = ioread32(sc_lsp1_crm_ctrl_base + 0x0c);
				iowrite32((tmp & (~(0xffff << 0))) | ((clk_div & 0xffff) << 0),
					sc_lsp1_crm_ctrl_base + 0x0c);
			} else if (dev->phy_base == 0x2002E000) {
				pr_info("%s set tdm3 sclk\n", __func__);
				tmp = ioread32(sc_lsp1_crm_ctrl_base + 0x178);
				iowrite32((tmp & (~(0xffff << 0))) | ((clk_div & 0xffff) << 0),
					sc_lsp1_crm_ctrl_base + 0x178);
			} else {
				/* do nothing */
			}
		}
		pr_info(" %s, freq:%d , tmp: %d, clk_div: %d\n", __func__, freq, tmp, clk_div);
	}

	return 0;
}

static int dw_tdm_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct dw_tdm_dev *dev = snd_soc_dai_get_drvdata(dai);
	struct tdm_clk_config_data *config = &dev->config;
	int ret;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		/* from synopsys i2s data book */
		/* Slot length in a TDM frame is always fixed to 32 clocks,
		that is, 32 serial clocks correspond to one slot. */
		config->data_width = 32;
		dev->ccr = 0x10;
		dev->xfer_resolution = 0x02;
		break;

	case SNDRV_PCM_FORMAT_S24_LE:
		/* from synopsys i2s data book */
		/* Slot length in a TDM frame is always fixed to 32 clocks,
		that is, 32 serial clocks correspond to one slot. */
		config->data_width = 32;
		dev->ccr = 0x10;
		dev->xfer_resolution = 0x04;
		break;

	case SNDRV_PCM_FORMAT_S32_LE:
		/* from synopsys i2s data book */
		/* Slot length in a TDM frame is always fixed to 32 clocks,
		that is, 32 serial clocks correspond to one slot. */
		config->data_width = 32;
		dev->ccr = 0x10;
		dev->xfer_resolution = 0x05;
		break;

	default:
		dev_err(dev->dev, "designware-tdm: unsupported PCM fmt");
		return -EINVAL;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		config->tx_chan_nr = params_channels(params);
		pr_info(" %s , tx_chan_nr: %d\n", __func__, config->tx_chan_nr);
		if (config->tx_chan_nr < MIN_CHANNEL_NUM || config->tx_chan_nr > MAX_CHANNEL_NUM) {
			dev_err(dev->dev, "tx channel not supported\n");
			return -EINVAL;
		}
	} else {
		config->rx_chan_nr = params_channels(params);
		pr_info(" %s , rx_chan_nr: %d\n", __func__, config->rx_chan_nr);
		if (config->rx_chan_nr < MIN_CHANNEL_NUM || config->rx_chan_nr > MAX_CHANNEL_NUM) {
			dev_err(dev->dev, "rx channel not supported\n");
			return -EINVAL;
		}
	}

	dw_tdm_config(dev, substream->stream);

	tdm_write_reg(dev->tdm_base, CCR, dev->ccr);

	config->sample_rate = params_rate(params);

	if (dev->capability & DW_TDM_MASTER) {
		if (dev->tdm_clk_cfg) {
			ret = dev->tdm_clk_cfg(config);
			if (ret < 0) {
				dev_err(dev->dev,
					"runtime audio clk config fail\n");
				return ret;
			}
		} else {
			u32 bitclk = config->sample_rate *
			    config->data_width * dev->slot_cnt;      //pcm3168a 8 slot

			/*reduce clock frequence in DMA mode*/
			if (dev->lsp_crm == 0) {
				ret = bst_set_tdm_clk(dev, bitclk, 0);
			} else if (dev->lsp_crm == 1) {
				ret = bst_set_tdm_clk(dev, bitclk, 1);
			} else {
				return -EINVAL;
			}

			if (ret) {
				dev_err(dev->dev,
					"Can't set TDM clock rate: %d\n", ret);
				return ret;
			}
		}
	}

	return 0;
}

static void dw_tdm_shutdown(struct snd_pcm_substream *substream,
			    struct snd_soc_dai *dai)
{
	//snd_soc_dai_set_dma_data(dai, substream, NULL);    //close dma , reset dai dma
	struct dw_tdm_dev *dev = snd_soc_dai_get_drvdata(dai);

	if ((dev->use_dma) && (dma_num>0)) {
		dma_num--;
		pr_info(" %s retain dma_data, dma_num ==%d  \n", __func__, dma_num);
	} else {
		snd_soc_dai_set_dma_data(dai, substream, NULL);
	}

}

static int dw_tdm_prepare(struct snd_pcm_substream *substream,
			  struct snd_soc_dai *dai)
{
	struct dw_tdm_dev *dev = snd_soc_dai_get_drvdata(dai);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		tdm_write_reg(dev->tdm_base, TXFFR, 1);   //flush tx block fifo
	else
		tdm_write_reg(dev->tdm_base, RXFFR, 1);   //flush rx block fifo

	return 0;
}

static int dw_tdm_trigger(struct snd_pcm_substream *substream,
			  int cmd, struct snd_soc_dai *dai)
{
	struct dw_tdm_dev *dev = snd_soc_dai_get_drvdata(dai);
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		dev->active++;
		tdm_start(dev, substream);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		dev->active--;
		tdm_stop(dev, substream);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int dw_tdm_set_fmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
	struct dw_tdm_dev *dev = snd_soc_dai_get_drvdata(cpu_dai);
	int ret = 0;

	switch (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_BC_FC:
		if (dev->capability & DW_TDM_SLAVE)
			ret = 0;
		else
			ret = -EINVAL;
		break;
	case SND_SOC_DAIFMT_BP_FP:
		if (dev->capability & DW_TDM_MASTER)
			ret = 0;
		else
			ret = -EINVAL;
		break;
	case SND_SOC_DAIFMT_BC_FP:
	case SND_SOC_DAIFMT_BP_FC:
		ret = -EINVAL;
		break;
	default:
		dev_dbg(dev->dev, "TDM : Invalid master/slave format\n");
		ret = -EINVAL;
		break;
	}
	return ret;
}

static const struct snd_soc_dai_ops dw_tdm_dai_ops = {
	.startup = dw_tdm_startup,
	.shutdown = dw_tdm_shutdown,
	.hw_params = dw_tdm_hw_params,
	.prepare = dw_tdm_prepare,
	.trigger = dw_tdm_trigger,
	.set_fmt = dw_tdm_set_fmt,
};

#ifdef CONFIG_PM
static int dw_tdm_runtime_suspend(struct device *dev)
{
	struct dw_tdm_dev *dw_dev = dev_get_drvdata(dev);
	if (dw_dev->capability & DW_TDM_MASTER)
		clk_disable(dw_dev->clk);
	return 0;
}

static int dw_tdm_runtime_resume(struct device *dev)
{
	struct dw_tdm_dev *dw_dev = dev_get_drvdata(dev);
	if (dw_dev->capability & DW_TDM_MASTER)
		clk_enable(dw_dev->clk);
	return 0;
}

static int dw_tdm_suspend(struct snd_soc_component *component)
{
	struct dw_tdm_dev *dev = snd_soc_component_get_drvdata(component);
	dev->pin->state = NULL;
	if (dev->capability & DW_TDM_MASTER)
		clk_disable(dev->clk);
	return 0;
}

static int dw_tdm_resume(struct snd_soc_component *component)
{
	struct dw_tdm_dev *dev = snd_soc_component_get_drvdata(component);
	struct snd_soc_dai *dai;
	int stream;

	bst_tdm_tdm_sel_init(dev->lsp_crm);

	if (dev->capability & DW_TDM_MASTER)
		clk_enable(dev->clk);

	pinctrl_pm_select_default_state(dev->dev);

	for_each_component_dais(component, dai) {
		for_each_pcm_streams(stream)
			if (snd_soc_dai_stream_active(dai, stream))
				dw_tdm_config(dev, stream);
	}

	return 0;
}

#else
#define dw_tdm_suspend	NULL
#define dw_tdm_resume	NULL
#endif

static const struct snd_soc_component_driver dw_tdm_component = {
	.name = "dw-tdm",
	.suspend = dw_tdm_suspend,
	.resume = dw_tdm_resume,
	.legacy_dai_naming = 1,
};

/*
 * The following tables allow a direct lookup of various parameters
 * defined in the I2S block's configuration in terms of sound system
 * parameters.  Each table is sized to the number of entries possible
 * according to the number of configuration bits describing an I2S
 * block parameter.
 */

/* Maximum bit resolution of a channel - not uniformly spaced */
static const u32 fifo_width[COMP_MAX_WORDSIZE] = {
	12, 16, 20, 24, 32, 0, 0, 0
};

/* Width of (DMA) bus */
static const u32 bus_widths[COMP_MAX_DATA_WIDTH] = {
	DMA_SLAVE_BUSWIDTH_1_BYTE,
	DMA_SLAVE_BUSWIDTH_2_BYTES,
	DMA_SLAVE_BUSWIDTH_4_BYTES,
	DMA_SLAVE_BUSWIDTH_UNDEFINED
};

/* PCM format to support channel resolution */
static const u32 formats[COMP_MAX_WORDSIZE] = {
	SNDRV_PCM_FMTBIT_S16_LE,
	SNDRV_PCM_FMTBIT_S16_LE,
	SNDRV_PCM_FMTBIT_S24_LE,
	SNDRV_PCM_FMTBIT_S24_LE,
	SNDRV_PCM_FMTBIT_S32_LE,
	0,
	0,
	0
};

#define TDM_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | \
                        SNDRV_PCM_FMTBIT_S24_LE | \
                        SNDRV_PCM_FMTBIT_S32_LE)

static int dw_configure_dai(struct dw_tdm_dev *dev,
			    struct snd_soc_dai_driver *dw_tdm_dai,
			    unsigned int rates)
{
	/*
	 * Read component parameter registers to extract
	 * the I2S block's configuration.
	 */
	u32 comp1 = tdm_read_reg(dev->tdm_base, dev->tdm_reg_comp1);
	u32 comp2 = tdm_read_reg(dev->tdm_base, dev->tdm_reg_comp2);
	u32 fifo_depth = 1 << (1 + COMP1_FIFO_DEPTH_GLOBAL(comp1));
	u32 idx;

	if (dev->capability & DWC_TDM_RECORD &&
	    dev->quirks & DW_TDM_QUIRK_COMP_PARAM1)
		comp1 = comp1 & ~BIT(5);

	if (dev->capability & DWC_TDM_PLAY &&
	    dev->quirks & DW_TDM_QUIRK_COMP_PARAM1)
		comp1 = comp1 & ~BIT(6);

	if (COMP1_TX_ENABLED(comp1)) {
		dev_dbg(dev->dev, " designware: play supported\n");
		idx = COMP1_TX_WORDSIZE_0(comp1);
		if (WARN_ON(idx >= ARRAY_SIZE(formats)))
			return -EINVAL;
		if (dev->quirks & DW_TDM_QUIRK_16BIT_IDX_OVERRIDE)
			idx = 1;
		dw_tdm_dai->playback.channels_min = MIN_CHANNEL_NUM;
		dw_tdm_dai->playback.channels_max = MAX_CHANNEL_NUM;
		dw_tdm_dai->playback.formats = TDM_FORMATS;
		dw_tdm_dai->playback.rates = rates;
	}

	if (COMP1_RX_ENABLED(comp1)) {
		dev_dbg(dev->dev, "designware: record supported\n");
		idx = COMP2_RX_WORDSIZE_0(comp2);
		if (WARN_ON(idx >= ARRAY_SIZE(formats)))
			return -EINVAL;
		if (dev->quirks & DW_TDM_QUIRK_16BIT_IDX_OVERRIDE)
			idx = 1;
		dw_tdm_dai->capture.channels_min = MIN_CHANNEL_NUM;
		dw_tdm_dai->capture.channels_max = MAX_CHANNEL_NUM;
		dw_tdm_dai->capture.formats = TDM_FORMATS;
		dw_tdm_dai->capture.rates = rates;
	}

	if (COMP1_MODE_EN(comp1)) {
		dev_dbg(dev->dev, "designware: tdm master mode supported\n");
		pr_info(" tdm master mode supported \n");
		dev->capability |= DW_TDM_MASTER;
	} else {
		dev_dbg(dev->dev, "designware: tdm slave mode supported\n");
		pr_info(" tdm slave mode supported \n");
		dev->capability |= DW_TDM_SLAVE;
	}
//	fifo_depth = 8;
	fifo_depth = 16;   //提高进入中断的频率，处理overrun/underrun的问题
	dev->fifo_th = fifo_depth / 2;
	return 0;
}

static int dw_configure_dai_by_pd(struct dw_tdm_dev *dev,
				  struct snd_soc_dai_driver *dw_tdm_dai,
				  struct resource *res,
				  const struct tdm_platform_data *pdata)
{
	u32 comp1 = tdm_read_reg(dev->tdm_base, dev->tdm_reg_comp1);
	u32 idx = COMP1_APB_DATA_WIDTH(comp1);
	int ret;

	if (WARN_ON(idx >= ARRAY_SIZE(bus_widths)))
		return -EINVAL;

	ret = dw_configure_dai(dev, dw_tdm_dai, pdata->snd_rates);
	if (ret < 0)
		return ret;

	if (dev->quirks & DW_TDM_QUIRK_16BIT_IDX_OVERRIDE)
		idx = 1;
	/* Set DMA slaves info */
	dev->play_dma_data.pd.data = pdata->play_dma_data;
	dev->capture_dma_data.pd.data = pdata->capture_dma_data;
	dev->play_dma_data.pd.addr = res->start + TDM_TXDMA;
	dev->capture_dma_data.pd.addr = res->start + TDM_RXDMA;
	dev->play_dma_data.pd.max_burst = 16;
	dev->capture_dma_data.pd.max_burst = 16;
	dev->play_dma_data.pd.addr_width = bus_widths[idx];
	dev->capture_dma_data.pd.addr_width = bus_widths[idx];
	dev->play_dma_data.pd.filter = pdata->filter;
	dev->capture_dma_data.pd.filter = pdata->filter;

	return 0;
}


static int dw_configure_dai_by_dt(struct dw_tdm_dev *dev,
				  struct snd_soc_dai_driver *dw_tdm_dai,
				  struct resource *res)
{
	u32 comp1 = tdm_read_reg(dev->tdm_base, TDM_COMP_PARAM_1);
	u32 comp2 = tdm_read_reg(dev->tdm_base, TDM_COMP_PARAM_2);
	u32 fifo_depth = 1 << (1 + COMP1_FIFO_DEPTH_GLOBAL(comp1));
	u32 idx = COMP1_APB_DATA_WIDTH(comp1);
	u32 idx2;
	int ret;
	fifo_depth = 8;

	pr_info(" %s comp1:0x%x, comp2:0x%x \n", __func__, comp1,comp2);
	pr_info(" %s fifo_depth:%d, idx:%d \n", __func__, fifo_depth,idx);

	if (WARN_ON(idx >= ARRAY_SIZE(bus_widths)))
		return -EINVAL;

	ret = dw_configure_dai(dev, dw_tdm_dai, SNDRV_PCM_RATE_8000_192000);
	if (ret < 0)
		return ret;

	if (COMP1_TX_ENABLED(comp1)) {
		idx2 = COMP1_TX_WORDSIZE_0(comp1);
		pr_info(" %s COMP1_TX_ENABLED \n", __func__);
		pr_info(" %s idx2:%d \n", __func__, idx2);
		dev->capability |= DWC_TDM_PLAY;
		dev->play_dma_data.dt.addr = res->start + TDM_TXDMA;
		dev->play_dma_data.dt.addr_width = bus_widths[idx];
		dev->play_dma_data.dt.fifo_size = fifo_depth *
			(fifo_width[idx2]) >> 8;
		dev->play_dma_data.dt.maxburst = 16;
		dev->play_dma_data.dt.peripheral_config = &tdm_dma_peripheral_cfg;
		dev->play_dma_data.dt.peripheral_size = sizeof(tdm_dma_peripheral_cfg);
	}
	if (COMP1_RX_ENABLED(comp1)) {
		idx2 = COMP2_RX_WORDSIZE_0(comp2);
		pr_info(" %s COMP1_RX_ENABLED \n", __func__);
		pr_info(" %s idx2:%d \n", __func__, idx2);
		dev->capability |= DWC_TDM_RECORD;
		dev->capture_dma_data.dt.addr = res->start + TDM_RXDMA;
		dev->capture_dma_data.dt.addr_width = bus_widths[idx];
		dev->capture_dma_data.dt.fifo_size = fifo_depth *
			(fifo_width[idx2]) >> 8;
		dev->capture_dma_data.dt.maxburst = 16;
		dev->capture_dma_data.dt.peripheral_config = &tdm_dma_peripheral_cfg;
		dev->capture_dma_data.dt.peripheral_size = sizeof(tdm_dma_peripheral_cfg);
	}

	return 0;

}


static ssize_t dw_tdm_show_regs(struct file *file, char __user *user_buf,
				size_t count, loff_t *ppos)
{
	pr_info("%s\n", __func__);
	return 0;
}

static ssize_t dw_tdm_write_regs(struct file *file,
				 const char __user *user_buf, size_t count,
				 loff_t *ppos)
{
	char info[20];
	int ret = 0;

	memset(info, 0, 20);
	ret = copy_from_user(info, user_buf, 1);
	pr_err("dw_tdm safety test:%c\n", info[0]);
	switch (info[0]) {
	case '1':
		{
			send_safety_usrmsg(0xa00301, 1);
		}
		break;
	case '2':
		{
			send_safety_usrmsg(0xa00302, 1);
		}
		break;
	case '3':
		{
			send_safety_usrmsg(0xa00303, 1);
		}
		break;
		break;
	default:
		break;
	}

	return count;
}

static const struct file_operations dw_tdm_regs_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = dw_tdm_show_regs,
	.write = dw_tdm_write_regs,
	.llseek = default_llseek,
};

static int dw_tdm_debugfs_init(void)
{
	char name[32];

	snprintf(name, 32, "dw_tdm%d", tdm_num);
	tdm_debugfs = debugfs_create_dir(name, NULL);
	if (!tdm_debugfs)
		return -ENOMEM;

	debugfs_create_file("registers", S_IFREG | S_IRUGO,
			    tdm_debugfs, 0, &dw_tdm_regs_ops);
	return 0;
}

static void dw_tdm_debugfs_remove(void)
{
	debugfs_remove_recursive(tdm_debugfs);
}

static int bst_tdm_dai_probe(struct snd_soc_dai *cpu_dai)
{
	struct dw_tdm_dev *dev = snd_soc_dai_get_drvdata(cpu_dai);

	cpu_dai->capture_dma_data = &dev->capture_dma_data;
	cpu_dai->playback_dma_data = &dev->play_dma_data;

	return 0;
}

static int dw_tdm_probe(struct platform_device *pdev)
{
	const struct tdm_platform_data *pdata = pdev->dev.platform_data;
	struct dw_tdm_dev *dev;
	struct resource *res;
	int ret, irq = -1;
	struct snd_soc_dai_driver *dw_tdm_dai;
	u32 tmp_data;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->use_dma = false;

	dw_tdm_dai = devm_kzalloc(&pdev->dev, sizeof(*dw_tdm_dai), GFP_KERNEL);
	if (!dw_tdm_dai)
		return -ENOMEM;

	dw_tdm_dai->ops = &dw_tdm_dai_ops;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dev->tdm_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dev->tdm_base))
		return PTR_ERR(dev->tdm_base);

	dev->phy_base = res->start;
	dev->dev = &pdev->dev;

	//pr_info(" %s phy base: 0x%llx \n", __func__, dev->phy_base);
	if(dev->phy_base==0x2000D000 || dev->phy_base==0x2000E000) {
		//dev_info(&pdev->dev, "tdm sc_lsp0_crm_ctrl_base = 0x20000000\n");
		sc_lsp0_crm_ctrl_base = ioremap(SOC_LSP0_CRM_REG_CTRL_BASE, 0x1000);
		dev->lsp_crm = 0;
	} else if(dev->phy_base==0x2002D000 || dev->phy_base==0x2002E000) {
		//dev_info(&pdev->dev, "tdm sc_lsp1_crm_ctrl_base = 0x20020000\n");
		sc_lsp1_crm_ctrl_base = ioremap(SOC_LSP1_CRM_REG_CTRL_BASE, 0x1000);
		dev->lsp_crm = 1;
	}

	bst_tdm_tdm_sel_init(dev->lsp_crm);

	if (dev->phy_base == 0x2000D000) {
		tmp_data = ioread32(sc_lsp0_crm_ctrl_base + 0x0c);
		iowrite32((tmp_data & (~(0xffff << 0))) | (0x240 << 0), sc_lsp0_crm_ctrl_base + 0x0c);
		//dev_info(&pdev->dev, "set tdm0 sclk\n");
	} else if (dev->phy_base == 0x2000E000) {
		tmp_data = ioread32(sc_lsp0_crm_ctrl_base + 0x178);
		iowrite32((tmp_data & (~(0xffff << 0))) | (0x240 << 0), sc_lsp0_crm_ctrl_base + 0x178);
		//dev_info(&pdev->dev, "set tdm1 sclk\n");
	} else if (dev->phy_base == 0x2002D000) {
		tmp_data = ioread32(sc_lsp1_crm_ctrl_base + 0x0c);
		iowrite32((tmp_data & (~(0xffff << 0))) | (0x240 << 0), sc_lsp1_crm_ctrl_base + 0x0c);
		//dev_info(&pdev->dev, "set tdm2 sclk\n");
	} else if (dev->phy_base == 0x2002E000) {
		tmp_data = ioread32(sc_lsp1_crm_ctrl_base + 0x178);
		iowrite32((tmp_data & (~(0xffff << 0))) | (0x240 << 0), sc_lsp1_crm_ctrl_base + 0x178);
		//dev_info(&pdev->dev, "set tdm3 sclk\n");
	} else {
		return -ENXIO;
	}

	if (device_property_read_bool(dev->dev, "use-dma"))
		dev->use_dma = true;

	if (dev->use_dma) 
		dw_tdm_dai->probe = bst_tdm_dai_probe;

	if (!dev->use_dma) {
		irq = platform_get_irq(pdev, 0);
		pr_info(" %s get irq, irq:%d \n", __func__, irq);
		if (irq >= 0) {
			ret = devm_request_irq(&pdev->dev, irq, tdm_irq_handler, 0,
						pdev->name, dev);
			//pr_info(" %s request irq, ret:%d \n", __func__, ret);
			if (ret < 0) {
				dev_err(&pdev->dev, "failed to request irq\n");
				return ret;
			}
		}
	}

	dev->tdm_reg_comp1 = TDM_COMP_PARAM_1;
	dev->tdm_reg_comp2 = TDM_COMP_PARAM_2;
	if (pdata) {
		dev->capability = pdata->cap;
		dev->quirks = pdata->quirks;
		if (dev->quirks & DW_TDM_QUIRK_COMP_REG_OFFSET) {
			dev->tdm_reg_comp1 = pdata->tdm_reg_comp1;
			dev->tdm_reg_comp2 = pdata->tdm_reg_comp2;
		}
		ret = dw_configure_dai_by_pd(dev, dw_tdm_dai, res, pdata);
	} else {
		ret = dw_configure_dai_by_dt(dev, dw_tdm_dai, res);
	}
	if (ret < 0)
		return ret;

	if (device_property_read_u32(dev->dev, "slot_cnt", &dev->slot_cnt) == 0) {
		dev_info(&pdev->dev, "slot count value: %u\n", dev->slot_cnt);
	} else {
		dev_info(&pdev->dev, "no slot count property config, default value\n");
		dev->slot_cnt = TDM_SLOT_NUM_8;
	}

	if (dev->capability & DW_TDM_MASTER) {
		if (pdata) {
			dev->tdm_clk_cfg = pdata->tdm_clk_cfg;
			if (!dev->tdm_clk_cfg) {
				dev_err(&pdev->dev,
					"no clock configure method\n");
				return -ENODEV;
			}
		}
		dev->clk = devm_clk_get(&pdev->dev, "wclk");

		if (IS_ERR(dev->clk))
			return PTR_ERR(dev->clk);

		ret = clk_prepare_enable(dev->clk);
		if (ret < 0)
			return ret;

		dev->clk = devm_clk_get(&pdev->dev, "pclk");

		if (IS_ERR(dev->clk))
			return PTR_ERR(dev->clk);

		//clk_set_rate(dev->clk, TDM_MCLK);
		//dev_info(&pdev->dev, "TDM-MCLK:%lu\n", clk_get_rate(dev->clk));

		ret = clk_prepare_enable(dev->clk);
		if (ret < 0)
			return ret;

	}

	dev_set_drvdata(&pdev->dev, dev);
	//pr_info(" %s devm_snd_soc_register_component \n", __func__);
	ret = devm_snd_soc_register_component(&pdev->dev, &dw_tdm_component,
					      dw_tdm_dai, 1);
	if (ret != 0) {
		dev_err(&pdev->dev, "not able to register dai\n");
		goto err_clk_disable;
	}

	if (!pdata) {
		if (irq >= 0) {
			dev_info(&pdev->dev, "TDM use IO\n");
			ret = dw_tdm_pcm_register(pdev);
			dev->use_pio = true;
		} else {
			dev_info(&pdev->dev, "TDM use DMA\n");
			ret = devm_snd_dmaengine_pcm_register(&pdev->dev, NULL, 0);
			dev->use_pio = false;
		}

		if (ret) {
			dev_err(&pdev->dev, "could not register pcm: %d\n", ret);
			goto err_clk_disable;
		}
	}

	dev->pin = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(dev->pin)) {
		dev_err(&pdev->dev, "error get tdm pinmux\n");
		return 0;
	}

	dw_tdm_debugfs_init();
	tdm_num++;

	pm_runtime_enable(&pdev->dev);
	return 0;

err_clk_disable:
	pr_info(" %s err_clk_disable \n", __func__);
	if (dev->capability & DW_TDM_MASTER)
		clk_disable_unprepare(dev->clk);
	return ret;
}

int tdm_pr_debug_test(void)
{
	pr_debug("/****** tdm: this is pe_debug in tdm module. ******/\n");
	return 0;
}
EXPORT_SYMBOL(tdm_pr_debug_test);

static int dw_tdm_remove(struct platform_device *pdev)
{
	struct dw_tdm_dev *dev = dev_get_drvdata(&pdev->dev);

	if (dev->capability & DW_TDM_MASTER)
		clk_disable_unprepare(dev->clk);

	pm_runtime_disable(&pdev->dev);
	dw_tdm_debugfs_remove();
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id dw_tdm_of_match[] = {
	{.compatible = "snps,designware-tdm", },
	{ },
};

MODULE_DEVICE_TABLE(of, dw_tdm_of_match);
#endif

static const struct dev_pm_ops dwc_pm_ops = {
	SET_RUNTIME_PM_OPS(dw_tdm_runtime_suspend, dw_tdm_runtime_resume, NULL)
};

static struct platform_driver dw_tdm_driver = {
	.probe = dw_tdm_probe,
	.remove = dw_tdm_remove,
	.driver = {
		   .name = "designware-tdm",
		   .of_match_table = of_match_ptr(dw_tdm_of_match),
		   .pm = &dwc_pm_ops,
		    },
};

module_platform_driver(dw_tdm_driver);

MODULE_AUTHOR("BST Ltd.");
MODULE_DESCRIPTION("DESIGNWARE TDM SoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:designware_tdm");
