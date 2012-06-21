/*
 * ALSA SoC I2S Audio Layer for ST SPEAr processors
 *
 * sound/soc/spear/designware_i2s.c
 *
 * Copyright (C) 2010 ST Microelectronics
 * Rajeev Kumar <rajeev-dlh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/designware_i2s.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spear_dma.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

/* common register for all channel */
#define IER		0x000
#define IRER		0x004
#define ITER		0x008
#define CER		0x00C
#define CCR		0x010
#define RXFFR		0x014
#define TXFFR		0x018

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
#define I2S_COMP_PARAM_2	0x01F0
#define I2S_COMP_PARAM_1	0x01F4
#define I2S_COMP_VERSION	0x01F8
#define I2S_COMP_TYPE		0x01FC

#define MAX_CHANNEL_NUM		8
#define MIN_CHANNEL_NUM		2

struct dw_i2s_dev {
	void __iomem *i2s_base;
	struct resource *res;
	struct clk *clk;
	int active;
	int play_irq;
	int capture_irq;
	unsigned int capability;
	struct device *dev;
	struct snd_soc_dai_driver *dai_driver;

	/* data related to DMA transfers b/w i2s and DMAC */
	struct dma_data play_dma_data;
	struct dma_data capture_dma_data;
	struct i2s_clk_config_data config;
	int (*i2s_clk_cfg)(struct i2s_clk_config_data *config);
};

static inline void i2s_write_reg(void *io_base, int reg, u32 val)
{
	writel(val, io_base + reg);
}

static inline u32 i2s_read_reg(void *io_base, int reg)
{
	return readl(io_base + reg);
}

static inline void
i2s_disable_channels(struct dw_i2s_dev *dev, u32 stream)
{
	u32 i = 0;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		for (i = 0; i < 4; i++)
			i2s_write_reg(dev->i2s_base, TER(i), 0);
	} else {
		for (i = 0; i < 4; i++)
			i2s_write_reg(dev->i2s_base, RER(i), 0);
	}
}

static inline void
i2s_clear_irqs(struct dw_i2s_dev *dev, u32 stream)
{
	u32 i = 0;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		for (i = 0; i < 4; i++)
			i2s_write_reg(dev->i2s_base, TOR(i), 0);
	} else {
		for (i = 0; i < 4; i++)
			i2s_write_reg(dev->i2s_base, ROR(i), 0);
	}
}

void i2s_start(struct dw_i2s_dev *dev, struct snd_pcm_substream *substream)
{

	i2s_write_reg(dev->i2s_base, IER, 1);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		i2s_write_reg(dev->i2s_base, ITER, 1);
	else
		i2s_write_reg(dev->i2s_base, IRER, 1);

	i2s_write_reg(dev->i2s_base, CER, 1);
}

static void
i2s_stop(struct dw_i2s_dev *dev, struct snd_pcm_substream *substream)
{
	u32 i = 0, irq;

	i2s_clear_irqs(dev, substream->stream);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		i2s_write_reg(dev->i2s_base, ITER, 0);

		for (i = 0; i < 4; i++) {
			irq = i2s_read_reg(dev->i2s_base, IMR(i));
			i2s_write_reg(dev->i2s_base, IMR(i), irq | 0x30);
		}
	} else {
		i2s_write_reg(dev->i2s_base, IRER, 0);

		for (i = 0; i < 4; i++) {
			irq = i2s_read_reg(dev->i2s_base, IMR(i));
			i2s_write_reg(dev->i2s_base, IMR(i), irq | 0x03);
		}
	}

	if (!dev->active) {
		i2s_write_reg(dev->i2s_base, CER, 0);
		i2s_write_reg(dev->i2s_base, IER, 0);
	}
}

static irqreturn_t dw_i2s_play_irq(int irq, void *_dev)
{
	struct dw_i2s_dev *dev = (struct dw_i2s_dev *)_dev;
	u32 ch0, ch1;

	/* check for the tx data overrun condition */
	ch0 = i2s_read_reg(dev->i2s_base, ISR(0)) & 0x20;
	ch1 = i2s_read_reg(dev->i2s_base, ISR(1)) & 0x20;
	if (ch0 || ch1) {
		/* disable i2s block */
		i2s_write_reg(dev->i2s_base, IER, 0);

		/* disable tx block */
		i2s_write_reg(dev->i2s_base, ITER, 0);

		/* flush all the tx fifo */
		i2s_write_reg(dev->i2s_base, TXFFR, 1);

		/* clear tx data overrun interrupt */
		i2s_clear_irqs(dev, SNDRV_PCM_STREAM_PLAYBACK);

		/* enable rx block */
		i2s_write_reg(dev->i2s_base, ITER, 1);

		/* enable i2s block */
		i2s_write_reg(dev->i2s_base, IER, 1);
	}

	return IRQ_HANDLED;
}

static irqreturn_t dw_i2s_capture_irq(int irq, void *_dev)
{
	struct dw_i2s_dev *dev = (struct dw_i2s_dev *)_dev;
	u32 ch0, ch1;

	/* check for the rx data overrun condition */
	ch0 = i2s_read_reg(dev->i2s_base, ISR(0)) & 0x02;
	ch1 = i2s_read_reg(dev->i2s_base, ISR(1)) & 0x02;
	if (ch0 || ch1) {
		/* disable i2s block */
		i2s_write_reg(dev->i2s_base, IER, 0);

		/* disable rx block */
		i2s_write_reg(dev->i2s_base, IRER, 0);

		/* flush all the rx fifo */
		i2s_write_reg(dev->i2s_base, RXFFR, 1);

		/* clear rx data overrun interrupt */
		i2s_clear_irqs(dev, SNDRV_PCM_STREAM_CAPTURE);

		/* enable rx block */
		i2s_write_reg(dev->i2s_base, IRER, 1);

		/* enable i2s block */
		i2s_write_reg(dev->i2s_base, IER, 1);
	}

	return IRQ_HANDLED;
}

static int
dw_i2s_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *cpu_dai)
{
	struct dw_i2s_dev *dev = snd_soc_dai_get_drvdata(cpu_dai);
	struct dma_data *dma_data = NULL;

	if (!(dev->capability & RECORD) &&
			(substream->stream == SNDRV_PCM_STREAM_CAPTURE))
		return -EINVAL;

	if (!(dev->capability & PLAY) &&
			(substream->stream == SNDRV_PCM_STREAM_PLAYBACK))
		return -EINVAL;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dma_data = &dev->play_dma_data;
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		dma_data = &dev->capture_dma_data;

	snd_soc_dai_set_dma_data(cpu_dai, substream, (void *)dma_data);

	return 0;
}

static int dw_i2s_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct dw_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);
	struct i2s_clk_config_data *config = &dev->config;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	u32 ccr, xfer_resolution, ch_reg, irq;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		config->data_width = 16;
		ccr = 0x00;
		xfer_resolution = 0x02;
		break;

	case SNDRV_PCM_FORMAT_S24_LE:
		config->data_width = 24;
		ccr = 0x08;
		xfer_resolution = 0x04;
		break;

	case SNDRV_PCM_FORMAT_S32_LE:
		config->data_width = 32;
		ccr = 0x10;
		xfer_resolution = 0x05;
		break;

	default:
		dev_err(dev->dev, "designware-i2s: unsuppted PCM fmt");
		return -EINVAL;
	}

	config->chan_nr = params_channels(params);

	switch (config->chan_nr) {
	case EIGHT_CHANNEL_SUPPORT:
		ch_reg = 3;
	case SIX_CHANNEL_SUPPORT:
		ch_reg = 2;
	case FOUR_CHANNEL_SUPPORT:
		ch_reg = 1;
	case TWO_CHANNEL_SUPPORT:
		ch_reg = 0;
		break;
	default:
		dev_err(dev->dev, "channel not supported\n");
	}

	i2s_disable_channels(dev, substream->stream);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		i2s_write_reg(dev->i2s_base, TCR(ch_reg), xfer_resolution);
		i2s_write_reg(dev->i2s_base, TFCR(ch_reg), 0x02);
		irq = i2s_read_reg(dev->i2s_base, IMR(ch_reg));
		i2s_write_reg(dev->i2s_base, IMR(ch_reg), irq & ~0x30);
		i2s_write_reg(dev->i2s_base, TER(ch_reg), 1);
	} else {
		i2s_write_reg(dev->i2s_base, RCR(ch_reg), xfer_resolution);
		i2s_write_reg(dev->i2s_base, RFCR(ch_reg), 0x07);
		irq = i2s_read_reg(dev->i2s_base, IMR(ch_reg));
		i2s_write_reg(dev->i2s_base, IMR(ch_reg), irq & ~0x03);
		i2s_write_reg(dev->i2s_base, RER(ch_reg), 1);
	}

	i2s_write_reg(dev->i2s_base, CCR, ccr);

	config->sample_rate = params_rate(params);

	if (!dev->i2s_clk_cfg)
		return -EINVAL;

	if (dev->i2s_clk_cfg(config)) {
		dev_err(dev->dev, "runtime audio clk config fail\n");
		if (cpu_dai->driver->ops->trigger) {
			int ret = cpu_dai->driver->ops->trigger(substream,
					SNDRV_PCM_TRIGGER_STOP, cpu_dai);
			if (ret < 0) {
				dev_err(dev->dev, "trigger stop fail\n");
				return ret;
			}
		}
	}

	return 0;
}

static void
dw_i2s_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	snd_soc_dai_set_dma_data(dai, substream, NULL);
}

static int
dw_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
		struct snd_soc_dai *dai)
{
	struct dw_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		dev->active++;
		i2s_start(dev, substream);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		dev->active--;
		i2s_stop(dev, substream);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static struct snd_soc_dai_ops dw_i2s_dai_ops = {
	.startup	= dw_i2s_startup,
	.shutdown	= dw_i2s_shutdown,
	.hw_params	= dw_i2s_hw_params,
	.trigger	= dw_i2s_trigger,
};

#ifdef CONFIG_PM

static int dw_i2s_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dw_i2s_dev *i2s_dev = dev_get_drvdata(&pdev->dev);

	clk_disable(i2s_dev->clk);
	return 0;
}

static int dw_i2s_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dw_i2s_dev *i2s_dev = dev_get_drvdata(&pdev->dev);

	clk_enable(i2s_dev->clk);
	return 0;
}

static const struct dev_pm_ops dw_i2s_dev_pm_ops = {
	.suspend = dw_i2s_suspend,
	.resume = dw_i2s_resume,
};

#define I2S_DW_DEV_PM_OPS (&dw_i2s_dev_pm_ops)
#else
#define I2S_DW_DEV_PM_OPS NULL
#endif

static int
dw_i2s_probe(struct platform_device *pdev)
{
	const struct i2s_platform_data *pdata = pdev->dev.platform_data;
	struct dw_i2s_dev *dev;
	struct resource *res;
	int ret;
	unsigned int cap;
	struct snd_soc_dai_driver *dw_i2s_dai;

	if (!pdata) {
		dev_err(&pdev->dev, "Invalid platform data\n");
		return -EINVAL;
	}

	cap = pdata->cap;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no i2s resource defined\n");
		return -ENODEV;
	}

	if (!request_mem_region(res->start, resource_size(res), pdev->name)) {
		dev_err(&pdev->dev, "i2s region already claimed\n");
		return -EBUSY;
	}
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		ret = -ENOMEM;
		goto err_release_mem_region;
	}

	dev->res = res;
	dev->capability = cap;
	dev->i2s_clk_cfg = pdata->i2s_clk_cfg;

	/* Set DMA slaves info */

	dev->play_dma_data.data = pdata->play_dma_data;
	dev->capture_dma_data.data = pdata->capture_dma_data;
	dev->play_dma_data.addr = res->start + I2S_TXDMA;
	dev->capture_dma_data.addr = res->start + I2S_RXDMA;
	dev->play_dma_data.max_burst = 16;
	dev->capture_dma_data.max_burst = 16;
	dev->play_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	dev->capture_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	dev->play_dma_data.filter = pdata->filter;
	dev->capture_dma_data.filter = pdata->filter;

	dev->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(dev->clk)) {
		ret = PTR_ERR(dev->clk);
		goto err_kfree;
	}

	ret = clk_enable(dev->clk);
	if (ret < 0)
		goto err_clk_put;

	dev->i2s_base = ioremap(res->start, resource_size(res));
	if (!dev->i2s_base) {
		dev_err(&pdev->dev, "ioremap fail for i2s_region\n");
		ret = -ENOMEM;
		goto err_clk_disable;
	}

	dw_i2s_dai = kzalloc(sizeof(*dw_i2s_dai), GFP_KERNEL);
	if (!dw_i2s_dai) {
		dev_err(&pdev->dev, "mem allocation failed for dai driver\n");
		ret = -ENOMEM;
		goto err_soc_dai_driver;
	}

	if (cap & PLAY) {
		dev_dbg(&pdev->dev, " SPEAr: play supported\n");
		dev->play_irq = platform_get_irq_byname(pdev, "play_irq");
		if (dev->play_irq < 0)
			dev_warn(&pdev->dev, "play irq not defined\n");
		else {
			ret = request_irq(dev->play_irq, dw_i2s_play_irq, 0,
					"dw-i2s-play", dev);
			if (ret) {
				dev_err(&pdev->dev,
						"Can't register play irq\n");
				goto err_play_irq;
			}
		}

		dw_i2s_dai->playback.channels_min = MIN_CHANNEL_NUM;
		dw_i2s_dai->playback.channels_max = pdata->channel;
		dw_i2s_dai->playback.formats = pdata->snd_fmts;
		dw_i2s_dai->playback.rates = pdata->snd_rates;
	}

	if (cap & RECORD) {
		dev_dbg(&pdev->dev, "SPEAr: record supported\n");
		dev->capture_irq = platform_get_irq_byname(pdev, "record_irq");
		if (dev->capture_irq < 0)
			dev_warn(&pdev->dev, "record irq not defined\n");
		else {
			ret = request_irq(dev->capture_irq, dw_i2s_capture_irq,
					0, "dw-i2s-rec", dev);
			if (ret) {
				dev_err(&pdev->dev,
						"Can't register capture irq\n");
				goto err_capture_irq;
			}
		}

		dw_i2s_dai->capture.channels_min = MIN_CHANNEL_NUM;
		dw_i2s_dai->capture.channels_max = pdata->channel;
		dw_i2s_dai->capture.formats = pdata->snd_fmts;
		dw_i2s_dai->capture.rates = pdata->snd_rates;
	}

	dw_i2s_dai->ops = &dw_i2s_dai_ops,

	dev->dev = &pdev->dev;
	dev->dai_driver = dw_i2s_dai;
	dev_set_drvdata(&pdev->dev, dev);
	ret = snd_soc_register_dai(&pdev->dev, dw_i2s_dai);
	if (ret != 0) {
		dev_err(&pdev->dev, "not able to register dai\n");
		goto err_set_drvdata;
	}

	return 0;

err_set_drvdata:
	dev_set_drvdata(&pdev->dev, NULL);
	free_irq(dev->capture_irq, pdev);
err_capture_irq:
	free_irq(dev->play_irq, pdev);
err_play_irq:
	kfree(dw_i2s_dai);
err_soc_dai_driver:
	iounmap(dev->i2s_base);
err_clk_disable:
	clk_disable(dev->clk);
err_clk_put:
	clk_put(dev->clk);
err_kfree:
	kfree(dev);
err_release_mem_region:
	release_mem_region(res->start, resource_size(res));
	return ret;
}

static int
dw_i2s_remove(struct platform_device *pdev)
{
	struct dw_i2s_dev *dev = dev_get_drvdata(&pdev->dev);

	snd_soc_unregister_dai(&pdev->dev);

	if (dev->play_irq)
		free_irq(dev->play_irq, dev);

	if (dev->capture_irq)
		free_irq(dev->capture_irq, dev);

	iounmap(dev->i2s_base);
	clk_disable(dev->clk);
	clk_put(dev->clk);
	kfree(dev->dai_driver);
	kfree(dev);
	release_mem_region(dev->res->start, resource_size(dev->res));

	return 0;
}

static struct platform_driver dw_i2s_driver = {
	.probe		= dw_i2s_probe,
	.remove		= dw_i2s_remove,
	.driver		= {
		.name	= "designware-i2s",
		.owner	= THIS_MODULE,
		.pm	= I2S_DW_DEV_PM_OPS,
	},
};

static int __init dw_i2s_init(void)
{
	return platform_driver_register(&dw_i2s_driver);
}
module_init(dw_i2s_init);

static void __exit dw_i2s_exit(void)
{
	platform_driver_unregister(&dw_i2s_driver);
}
module_exit(dw_i2s_exit);

MODULE_AUTHOR("Rajeev Kumar <rajeev-dlh.kumar@st.com>");
MODULE_DESCRIPTION("DESIGNWARE I2S SoC Interface");
MODULE_LICENSE("GPL");
