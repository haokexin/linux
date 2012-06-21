/*
 * ASoC codec driver for spear platform
 *
 * sound/soc/codecs/sta529.c -- spear ALSA Soc codec driver
 *
 * Copyright (C) 2010 ST Microelectronics
 * Rajeev Kumar<rajeev-dlh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <mach/misc_regs.h>
#include "sta529.h"

#define STA529_VERSION "0.1"
static const u8 sta529_reg[STA529_CACHEREGNUM] = {
	0x35, 0xc8, 0x50, 0x00,
	0x00, 0x00, 0x02, 0x00,
	0x02, 0x05, 0xb2, 0x41,
	0x92, 0x41, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x80, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x52, 0x40,
	0x21, 0xef, 0x04, 0x06,
	0x41, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00,
};

struct sta529 {
	enum snd_soc_control_type control_type;
	void *control_data;
};
static const struct snd_kcontrol_new sta529_snd_controls[] = {
	SOC_SINGLE("Master Playback Volume", STA529_MVOL, 0, 127, 1),
	SOC_SINGLE("Left Playback Volume", STA529_LVOL, 0, 127, 1),
	SOC_SINGLE("Right Playback Volume", STA529_RVOL, 0, 127, 1),
	SOC_SINGLE("Adc Capture Gain", STA529_ADCCFG, 5, 7, 0),
	SOC_SINGLE("master mute", STA529_MVOL, 7, 1, 0),
};

/* reading from register cache: sta529 register value */
static inline unsigned int
sta529_read_reg_cache(struct snd_soc_codec *codec, u32 reg)
{
	u8 *cache = codec->reg_cache;

	if (reg >= STA529_CACHEREGNUM)
		return -EINVAL;

	return cache[reg];
}

/* write register cache : sta529 register value. */
static inline void
sta529_write_reg_cache(struct snd_soc_codec *codec, u8 reg, int value)
{
	u8 *cache = codec->reg_cache;

	if (reg >= STA529_CACHEREGNUM)
		return;

	cache[reg] = value;
}

/* write to the sta529 register space */
static int
sta529_write(struct snd_soc_codec *codec, u32 reg, u32 value)
{
	u8 data[2];

	data[0] = reg & 0xff;
	data[1] = value & 0xff;
	sta529_write_reg_cache(codec, data[0], data[1]);
	if (codec->hw_write(codec->control_data, data, NUM_OF_MSG)
			== NUM_OF_MSG)
		return 0;
	else
		return -EIO;
}

static int
spear_sta529_set_dai_fmt(struct snd_soc_dai *codec_dai, u32 fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u8 mode = 0;
	int val;

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_LEFT_J:
		mode = LEFT_J_DATA_FORMAT;
		break;
	case SND_SOC_DAIFMT_I2S:
		mode = I2S_DATA_FORMAT;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		mode = RIGHT_J_DATA_FORMAT;
		break;
	default:
		return -EINVAL;
	}

	val = sta529_read_reg_cache(codec, STA529_S2PCFG0);
	val |= mode;
	/*this setting will be used with actual h/w */
	sta529_write(codec, STA529_S2PCFG0, val);

	/* set serial-to-parallel interface data length to 32 bit */
	sta529_write(codec, STA529_P2SCFG1, 0xC1);

	/* set parallel to-serial interface data length as 32 bit */
	sta529_write(codec, STA529_P2SCFG1, 0xC1);

	return 0;
}

static int
sta529_set_bias_level(struct snd_soc_codec *codec,
		enum snd_soc_bias_level level)
{
	u16 sts;

	sts = sta529_read_reg_cache(codec, STA529_FFXCFG0);

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
		sta529_write(codec, STA529_FFXCFG0, sts & ~POWER_STBY);
		break;
	case SND_SOC_BIAS_STANDBY:
	case SND_SOC_BIAS_OFF:
		sta529_write(codec, STA529_FFXCFG0, sts | POWER_STBY);

		break;
	}

	/*
	 * store the label for powers down audio subsystem for suspend.
	 * This is used by soc core layer
	 */
	codec->dapm.bias_level = level;
	return 0;

}

static int spear_sta529_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	u32 play_freq_val, record_freq_val, val;

	if (cpu_is_spear1340()) {
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
			/*
			 * On SPEAr1340 SoC I2S capture controller is in
			 * slave mode. Hence sta529 must be configured
			 * to provide the bit clock and word select
			 * clock. For this sta529 capture part must be
			 * programmed in master mode.
			 * Another thing is this that unlike other SPEAr
			 * SoCs, bit clock would not be shared for play
			 * and record lines of sta529.
			 */
			sta529_write(codec, STA529_S2PCFG0, 0x12);
			sta529_write(codec, STA529_P2SCFG0, 0x93);
		}
	}

	switch (params_rate(params)) {
	case 8000:
	case 11025:
		play_freq_val = 0;
		record_freq_val = 2;
		break;
	case 16000:
	case 22050:
		play_freq_val = 1;
		record_freq_val = 0;
		break;

	case 32000:
	case 44100:
	case 48000:
		play_freq_val = 2;
		record_freq_val = 0;
		break;
	default:
		dev_err(codec->dev, "bad rate", __func__);
		return -EINVAL;
	}

	val = sta529_read_reg_cache(codec, STA529_MISC);

	/* set FFX audio frequency range */
	val = (((val & 0x83) | (play_freq_val << 4)) | (record_freq_val << 2));

	sta529_write(codec, STA529_MISC, val);

	sta529_set_bias_level(codec, SND_SOC_BIAS_PREPARE);
	mdelay(1);

	return 0;
}

static int spear_sta529_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;

	u8 mute_reg = sta529_read_reg_cache(codec, STA529_FFXCFG0) &
		~CODEC_MUTE_VAL;

	if (mute)
		mute_reg |= CODEC_MUTE_VAL;

	sta529_write(codec, STA529_FFXCFG0, mute_reg);

	return 0;
}

static struct snd_soc_dai_ops sta529_dai_ops = {
	.hw_params	= spear_sta529_hw_params,
	.set_fmt	= spear_sta529_set_dai_fmt,
	.digital_mute	= spear_sta529_mute,
};

static struct snd_soc_dai_driver sta529_dai = {
	.name = "sta529-audio",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SPEAR_PCM_RATES,
		.formats = SPEAR_PCM_FORMAT,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SPEAR_PCM_RATES,
		.formats = SPEAR_PCM_FORMAT,
	},
	.ops	= &sta529_dai_ops,
};

static int spear_sta529_probe(struct snd_soc_codec *codec)
{
	struct sta529 *sta529 = snd_soc_codec_get_drvdata(codec);
	int i ;
	u8 *cache ;
	u8 data[2];

	dev_info(codec->dev, "spear Audio Codec %s", STA529_VERSION);

	codec->control_data = sta529->control_data;
	codec->hw_write = (hw_write_t)i2c_master_send;
	codec->hw_read = NULL;
	sta529_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	snd_soc_add_codec_controls(codec, sta529_snd_controls,
			ARRAY_SIZE(sta529_snd_controls));

	cache = codec->reg_cache;
	for (i = 0; i < ARRAY_SIZE(sta529_reg); i++) {
		data[0] = i;
		data[1] = cache[i];
		codec->hw_write(codec->control_data, data, NUM_OF_MSG);
	}
	return 0;
}

/* power down chip */
static int spear_sta529_remove(struct snd_soc_codec *codec)
{
	sta529_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int spear_sta529_suspend(struct snd_soc_codec *codec)
{
	sta529_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int spear_sta529_resume(struct snd_soc_codec *codec)
{
	int i;
	u8 data[2];
	u8 *cache = codec->reg_cache;

	for (i = 0; i < ARRAY_SIZE(sta529_reg); i++) {
		data[0] = i;
		data[1] = cache[i];
		codec->hw_write(codec->control_data, data, NUM_OF_MSG);
	}

	sta529_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	sta529_set_bias_level(codec, codec->dapm.suspend_bias_level);

	return 0;
}

struct snd_soc_codec_driver soc_codec_dev_sta529 = {
	.reg_cache_size = ARRAY_SIZE(sta529_reg),
	.reg_word_size = sizeof(u8),
	.reg_cache_default = sta529_reg,
	.probe = spear_sta529_probe,
	.remove = spear_sta529_remove,
	.suspend = spear_sta529_suspend,
	.resume = spear_sta529_resume,
	.read = sta529_read_reg_cache,
	.write = sta529_write,
	.set_bias_level = sta529_set_bias_level,
};

static __devinit int sta529_i2c_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
	struct sta529 *sta529;
	int ret;

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EINVAL;

	sta529 = kzalloc(sizeof(struct sta529), GFP_KERNEL);
	if (sta529 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, sta529);
	sta529->control_data = i2c;
	sta529->control_type = SND_SOC_I2C;

	ret = snd_soc_register_codec(&i2c->dev,
			&soc_codec_dev_sta529, &sta529_dai, 1);
	if (ret < 0)
		kfree(sta529);
	return ret;
}

static int sta529_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	kfree(i2c_get_clientdata(client));
	return 0;
}

static const struct i2c_device_id sta529_i2c_id[] = {
	{ "sta529", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sta529_i2c_id);

static struct i2c_driver sta529_i2c_driver = {
	.driver = {
		.name = "sta529-codec",
		.owner = THIS_MODULE,
	},
	.probe		= sta529_i2c_probe,
	.remove		= __devexit_p(sta529_i2c_remove),
	.id_table	= sta529_i2c_id,
};

static int __init sta529_modinit(void)
{
	int ret = 0;

	ret = i2c_add_driver(&sta529_i2c_driver);
	if (ret != 0)
		printk(KERN_ERR "Failed to reg sta529 I2C driver: %d\n", ret);

	return ret;

}
module_init(sta529_modinit);

static void __exit sta529_exit(void)
{
	i2c_del_driver(&sta529_i2c_driver);
}
module_exit(sta529_exit);

MODULE_DESCRIPTION("Soc STA529 driver");
MODULE_AUTHOR("Rajeev Kumar");
MODULE_LICENSE("GPL");
