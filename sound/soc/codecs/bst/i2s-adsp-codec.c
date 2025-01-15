// SPDX-License-Identifier: GPL-2.0+
/*
 * Driver for generic adsp-21569 link
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <sound/soc.h>

static const struct snd_soc_dapm_widget adsp_codec_widgets[] = {
	SND_SOC_DAPM_INPUT("RX"),
	SND_SOC_DAPM_OUTPUT("TX"),
};

#define ADSP_RATES SNDRV_PCM_RATE_8000_96000
#define ADSP_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |\
	SNDRV_PCM_FMTBIT_S32_LE)

static const struct snd_soc_dapm_route adsp_codec_routes[] = {
	{ "Capture", NULL, "RX" },
	{ "TX", NULL, "Playback" },
};

static struct snd_soc_dai_driver adsp_codec_dai[] = {
	{
		.name = "adsp-i2s",
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = ADSP_RATES,
			.formats = ADSP_FORMATS,
		},
		.capture = {
			 .stream_name = "Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = ADSP_RATES,
			.formats = ADSP_FORMATS,
		},
	}
};

static const struct snd_soc_component_driver soc_component_dev_adsp_codec = {
	.dapm_widgets		= adsp_codec_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(adsp_codec_widgets),
	.dapm_routes		= adsp_codec_routes,
	.num_dapm_routes	= ARRAY_SIZE(adsp_codec_routes),
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static int adsp_codec_probe(struct platform_device *pdev)
{
	return devm_snd_soc_register_component(&pdev->dev,
				      &soc_component_dev_adsp_codec,
				      adsp_codec_dai, ARRAY_SIZE(adsp_codec_dai));
}

static int adsp_codec_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct platform_device_id adsp_codec_driver_ids[] = {
	{
		.name		= "adsp0",
	},
	{
		.name		= "adsp1",
	},
	{},
};
MODULE_DEVICE_TABLE(platform, adsp_codec_driver_ids);

#if defined(CONFIG_OF)
static const struct of_device_id adsp_codec_codec_of_match[] = {
	{ .compatible = "linux,i2s-adsp-codec0", },
	{ .compatible = "linux,i2s-adsp-codec1", },
	{ .compatible = "linux,i2s-adsp-codec2", },
	{},
};
MODULE_DEVICE_TABLE(of, adsp_codec_codec_of_match);
#endif

static struct platform_driver adsp_codec_driver = {
	.driver = {
		.name = "i2s-adsp-codec",
		.of_match_table = of_match_ptr(adsp_codec_codec_of_match),
	},
	.probe = adsp_codec_probe,
	.remove = adsp_codec_remove,
	.id_table = adsp_codec_driver_ids,
};

module_platform_driver(adsp_codec_driver);

MODULE_AUTHOR("BST Ltd.");
MODULE_DESCRIPTION("ASoC generic adsp-21569 link driver");
MODULE_LICENSE("GPL");
