// SPDX-License-Identifier: GPL-2.0+
/*
 * Driver for generic adsp-21569 link
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <sound/soc.h>

#define TX_MAX_CHANNEL_NUM 8 //3168a codec supported max tx channel number
#define RX_MAX_CHANNEL_NUM 6 //3168a codec supported max rx channel number


static const struct snd_soc_dapm_widget adsp_codec_widgets[] = {
	SND_SOC_DAPM_INPUT("RX"),
	SND_SOC_DAPM_OUTPUT("TX"),
};

#define ADSP_RATES SNDRV_PCM_RATE_8000_48000

#define ADSP_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |\
	SNDRV_PCM_FMTBIT_S32_LE)


static const struct snd_soc_dapm_route adsp_codec_routes[] = {
	{ "Capture", NULL, "RX" },
	{ "TX", NULL, "Playback" },
};

static struct snd_soc_dai_driver adsp_codec_dai[] = {
	{
		.name = "adsp-tdm",
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = TX_MAX_CHANNEL_NUM,
			.rates = ADSP_RATES,
			.formats = ADSP_FORMATS,
		},
		.capture = {
			 .stream_name = "Capture",
			.channels_min = 1,
			.channels_max = RX_MAX_CHANNEL_NUM,
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
	//.non_legacy_dai_naming	= 1,
};

static int adsp_codec_probe(struct platform_device *pdev)
{
	u32 tx_slot_cnt = TX_MAX_CHANNEL_NUM;
	u32 rx_slot_cnt = RX_MAX_CHANNEL_NUM;

	if (device_property_read_u32(&pdev->dev, "tx_slot_cnt", &tx_slot_cnt) == 0)
		dev_info(&pdev->dev, "tx slot count value: %u\n", tx_slot_cnt);
	else
		dev_info(&pdev->dev,
			 "no tx slot count property config, default value\n");

	if (device_property_read_u32(&pdev->dev, "rx_slot_cnt", &rx_slot_cnt) == 0)
		dev_info(&pdev->dev, "rx slot count value: %u\n", rx_slot_cnt);
	else 
		dev_info(&pdev->dev,
			 "no rx slot count property config, default value\n");

	for (u32 index = 0; index < ARRAY_SIZE(adsp_codec_dai); index++) {
		adsp_codec_dai[index].playback.channels_max = tx_slot_cnt;
		adsp_codec_dai[index].capture.channels_max = rx_slot_cnt;
	}

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
	{ .compatible = "linux,tdm-adsp-codec0", },
	{ .compatible = "linux,tdm-adsp-codec1", },
	{ .compatible = "linux,tdm-adsp-codec2", },
	{ .compatible = "linux,tdm-adsp-codec3", },
	{},
};
MODULE_DEVICE_TABLE(of, adsp_codec_codec_of_match);
#endif

static struct platform_driver adsp_codec_driver = {
	.driver = {
		.name = "tdm-adsp-codec",
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
