/*
 * ASoC machine driver for SPEAr evaluation boards
 *
 * sound/soc/spear/spear_evb.c
 *
 * Copyright (C) 2010 ST Microelectronics
 * Rajeev Kumar<rajeev-dlh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/module.h>

#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <mach/hardware.h>
#include <mach/misc_regs.h>

static int sta529_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret = 0;
	u32 channel;

	channel = params_channels(params);

	if (cpu_is_spear1340()) {
#ifdef CONFIG_CPU_SPEAR1340
		u32 mode = 0;
		u32 val = readl(VA_SPEAR1340_PERIP_CFG);

		switch (channel) {
		case 8:
			mode = SPEAR1340_I2S_CHNL_7_1;
			break;
		case 6:
			mode = SPEAR1340_I2S_CHNL_5_1;
			break;
		case 4:
			mode = SPEAR1340_I2S_CHNL_3_1;
			break;
		case 2:
		default:
			mode = SPEAR1340_I2S_CHNL_2_0;
			break;
		}

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			mode = mode << SPEAR1340_I2S_CHNL_PLAY_SHIFT;
			val = (val & ~SPEAR1340_I2S_CHNL_PLAY_MASK) | mode;
		} else {
			mode = mode << SPEAR1340_I2S_CHNL_REC_SHIFT;
			val = (val & ~SPEAR1340_I2S_CHNL_REC_MASK) | mode;
		}
		writel(val, VA_SPEAR1340_PERIP_CFG);
#endif
	} else if (cpu_is_spear1310_reva() || cpu_is_spear1310()) {
#if defined(CONFIG_CPU_SPEAR1300) || defined(CONFIG_CPU_SPEAR1310_REVA) || \
	defined(CONFIG_CPU_SPEAR900) || defined(CONFIG_CPU_SPEAR1310)
		/* setting mode 0 in conf register: 32c offset */
		u32 val = readl(VA_PERIP_CFG);
		val &= ~I2S_MODE_MASK;
		val |= I2S_MODE_I2S2_ONE_PORT;
		writel(val, VA_PERIP_CFG);
#endif
	}

	return 0;
}

/* Audio machine driver for SPEAr evb */
static struct snd_soc_ops sta529_ops = {
	.hw_params	= sta529_hw_params,
};

/* SPEAr audio interface glue - connects codec <--> CPU <--> platform */
static struct snd_soc_dai_link spear_evb_dai[] = {
	{
		.name		= "sta529-pcm",
		.stream_name	= "pcm",
		.cpu_dai_name	= "designware-i2s.0",
		.platform_name	= "spear-pcm-audio",
		.codec_dai_name	= "sta529-audio",
		.codec_name	= "sta529-codec.0-001a",
		.ops		= &sta529_ops,
	},
};

/* SPEAr audio machine driver */
static struct snd_soc_card spear_snd_card = {
	.name		= "spear-evb",
	.dai_link	= spear_evb_dai,
	.num_links	= ARRAY_SIZE(spear_evb_dai),
};

/* SPEAr320s audio interface glue - connects codec <--> CPU <--> platform */
static struct snd_soc_dai_link spear320s_evb_dai[] = {
	{
		.name		= "sta529-pcm",
		.stream_name	= "pcm",
		.cpu_dai_name	= "designware-i2s",
		.platform_name	= "spear-pcm-audio",
		.codec_dai_name	= "sta529-audio",
		.codec_name	= "sta529-codec.0-001a",
		.ops		= &sta529_ops,
	},
};

/* SPEAr320s audio machine driver */
static struct snd_soc_card spear320s_snd_card = {
	.name		= "spear320s-evb",
	.dai_link	= spear320s_evb_dai,
	.num_links	= ARRAY_SIZE(spear320s_evb_dai),
};

/* LCAD audio interface glue - connects codec <--> CPU <--> platform */
static struct snd_soc_dai_link lcad_evb_dai[] = {
	{
		.name		= "sta529-pcm0",
		.stream_name	= "I2S Playback",
		.cpu_dai_name	= "designware-i2s.0",
		.platform_name	= "spear-pcm-audio",
		.codec_dai_name	= "sta529-audio",
		.codec_name	= "sta529-codec.0-001a",
		.ops		= &sta529_ops,
	}, {
		.name		= "sta529-pcm1",
		.stream_name	= "I2S Capture",
		.cpu_dai_name	= "designware-i2s.1",
		.platform_name	= "spear-pcm-audio",
		.codec_dai_name	= "sta529-audio",
		.codec_name	= "sta529-codec.0-001a",
		.ops		= &sta529_ops,
	},
};

static struct snd_soc_card lcad_snd_card = {
	.name		= "lcad-evb",
	.dai_link	= lcad_evb_dai,
	.num_links	= ARRAY_SIZE(lcad_evb_dai),
};

/* Audio machine driver for SPEAr1340 evb */

/* SPEAr1340 audio interface glue - connects codec <--> CPU <--> platform */
static struct snd_soc_dai_link spear1340_evb_dai[] = {
	{
		.name		= "spdif-pcm0",
		.stream_name	= "SPDIF Playback",
		.cpu_dai_name	= "spdif-out",
		.platform_name	= "spear-pcm-audio",
		.codec_dai_name	= "dit-hifi",
		.codec_name	= "spdif-dit",
		.ops		= NULL,
	}, {
		.name		= "spdif-pcm1",
		.stream_name	= "SPDIF Capture",
		.cpu_dai_name	= "spdif-in",
		.platform_name	= "spear-pcm-audio",
		.codec_dai_name	= "dir-hifi",
		.codec_name	= "spdif-dir",
		.ops		= NULL,
	}, {
		.name		= "sta529-pcm0",
		.stream_name	= "I2S Playback",
		.cpu_dai_name	= "designware-i2s.0",
		.platform_name	= "spear-pcm-audio",
		.codec_dai_name	= "sta529-audio",
		.codec_name	= "sta529-codec.0-001a",
		.ops		= &sta529_ops,
	}, {
		.name		= "sta529-pcm1",
		.stream_name	= "I2S Capture",
		.cpu_dai_name	= "designware-i2s.1",
		.platform_name	= "spear-pcm-audio",
		.codec_dai_name	= "sta529-audio",
		.codec_name	= "sta529-codec.0-001a",
		.ops		= &sta529_ops,
	},
};

static struct snd_soc_card spear1340_snd_card = {
	.name		= "spear1340-evb",
	.dai_link	= spear1340_evb_dai,
	.num_links	= ARRAY_SIZE(spear1340_evb_dai),
};

static struct platform_device *evb_snd_device;
#if defined(CONFIG_CPU_SPEAR1340)
static struct platform_device *spdif_dit_device;
static struct platform_device *spdif_dir_device;
#endif

static int __init spear_audio_init(void)
{
	int ret;
	struct snd_soc_card *spear_soc_card;

	if (cpu_is_spear1340())
		spear_soc_card = &spear1340_snd_card;
	else
		spear_soc_card = &spear_snd_card;

#if defined(CONFIG_CPU_SPEAR1340)
	if (cpu_is_spear1340()) {
		/* Create and register spdif platform devices */
		spdif_dit_device = platform_device_alloc("spdif-dit", -1);
		if (!spdif_dit_device) {
			printk(KERN_ERR "spdif transceiver " \
					"platform_device_alloc fails\n");
			return -ENOMEM;
		}
		ret = platform_device_add(spdif_dit_device);
		if (ret) {
			printk(KERN_ERR "Unable to add spdif transceiver " \
					"platform device\n");
			platform_device_put(spdif_dit_device);
		}

		spdif_dir_device = platform_device_alloc("spdif-dir", -1);
		if (!spdif_dir_device) {
			printk(KERN_ERR "spdif receive platform_device_alloc " \
					"fails\n");
			return -ENOMEM;
		}
		ret = platform_device_add(spdif_dir_device);
		if (ret) {
			printk(KERN_ERR "Unable to add spdif receive platform" \
					"device\n");
			platform_device_put(spdif_dir_device);
		}
	}
#endif
	/* Create and register platform device */
	evb_snd_device = platform_device_alloc("soc-audio", -1);
	if (!evb_snd_device) {
		printk(KERN_ERR "soc audio platform_device_alloc fails\n");
		return -ENOMEM;
	}
	platform_set_drvdata(evb_snd_device, spear_soc_card);
	ret = platform_device_add(evb_snd_device);
	if (ret) {
		printk(KERN_ERR "Unable to add platform device\n");
		platform_device_put(evb_snd_device);
	}

	return ret;
}
module_init(spear_audio_init);

static void __exit spear_audio_exit(void)
{
	platform_device_unregister(evb_snd_device);
}
module_exit(spear_audio_exit);

MODULE_AUTHOR("Rajeev Kumar <rajeev-dlh.kumar@st.com>");
MODULE_DESCRIPTION("ST SPEAr EVB ASoC driver");
MODULE_LICENSE("GPL");
