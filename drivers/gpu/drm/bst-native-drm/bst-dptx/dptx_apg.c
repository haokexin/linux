// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#include "dptx_drv.h"
#include "dptx_apg.h"
#include "avgen.h"

#define I2S0_REG_IER	(0x0)
#define I2S0_REG_ITER   (0x8)
#define I2S0_REG_CER	(0xC)
#define I2S0_REG_CCR	(0x10)
#define I2S0_REG_TXFFR  (0x18)
#define I2S0_REG_LTHR   (0x20)
#define I2S0_REG_RTHR   (0x24)
#define I2S0_REG_TER	(0x2C)
#define I2S0_REG_TCR	(0x34)
#define I2S0_REG_ISR	(0x38)
#define I2S0_REG_RTXDMA (0x1CC)
#define I2S_REG_N(base, n)	((base) + 0x40 * (n))
#define MK_DPTX_I2S_SAMPLE(PR, P, C, U, V, SAMPLE) \
			((PR) << 30 | (P) << 29 | (C) << 28 | (U) << 27 | (V) << 26 | (SAMPLE) << 2)

struct dptx_apg {
	u32* audio_file;
	u32 audio_size;
	bool is_param_valid;
	bool is_apg_enable;
};

static struct dptx_apg _apg_ctx;

int dptx_apg_enable(struct dptx *dptx)
{
	struct device *dev = dptx->dev;
	struct regmap *apg_base = dptx->regs[DPTX_APG];
	u32 i, overrun = 0;
	u32 data_left = 0, data_right = 0;
	u32 *audio_file = _apg_ctx.audio_file;
	u32 audio_size =  _apg_ctx.audio_size;

	if (_apg_ctx.is_param_valid) {
		dev_info(dev, "send audio data begin, size:%d!!", audio_size);
		dptx_write_reg(dptx, apg_base, I2S0_REG_TXFFR, 0x1);
		dptx_write_reg(dptx, apg_base, I2S0_REG_TCR, 0x5);
		dptx_write_reg(dptx, apg_base, I2S0_REG_IER, 0x1);
		dptx_write_reg(dptx, apg_base, I2S0_REG_ITER, 0x1);
		usleep_range(100, 150);
		dptx_write_reg(dptx, apg_base, I2S0_REG_CER, 0x0);
		dptx_write_reg(dptx, apg_base, I2S0_REG_CCR, 0x10);
		dptx_write_reg(dptx, apg_base, I2S0_REG_TXFFR, 0X1);
		dptx_write_reg(dptx, apg_base, I2S0_REG_RTXDMA, 0x0);
		dptx_write_reg(dptx, apg_base, I2S0_REG_TER, 0x1);
		dptx_write_reg(dptx, apg_base, I2S0_REG_CER, 0x1);

		for(i = 0; i < audio_size / 4;) {
			overrun = dptx_read_reg(dptx, apg_base, I2S0_REG_ISR);
			if (((overrun >> 4) & 0x1) == 0x01) {
				if (audio_file)
					data_left = audio_file[i];
				else
					data_left = MK_DPTX_I2S_SAMPLE(1, 0, 0, 0, 0, i);

				dptx_write_reg(dptx, apg_base,
						I2S_REG_N(I2S0_REG_LTHR, 0), data_left);

				if (audio_file)
					data_right = audio_file[i + 1];
				else
					data_right =
						MK_DPTX_I2S_SAMPLE(2, 0, 0, 0, 0, audio_size - i);
				dptx_write_reg(dptx, apg_base,
						I2S_REG_N(I2S0_REG_RTHR, 0), data_right);
				i = i + 2;
			}
			dev_info(dev, "send sample[%d]=L:0x%x-R:0x%x done\n",
					i, data_left, data_right);
		}
		_apg_ctx.is_apg_enable = true;
		dev_info(dev, "enable audio pattern done\n");
	}

	return 0;
}

int dptx_apg_disable(struct dptx *dptx)
{
	struct device *dev = dptx->dev;
	struct regmap *base = dptx->regs[DPTX_APG];

	_apg_ctx.is_apg_enable = false;
	_apg_ctx.is_param_valid = false;
	if (base) {
		dptx_write_reg(dptx, base, I2S0_REG_IER, 0x0);
		dptx_write_reg(dptx, base, I2S0_REG_ITER, 0x0);
		dev_info(dev, "disable audio pattern done\n");
	}  else {
		dev_err(dev, "Failed to disable dptx vpg\n");
		return -1;
	}

	return 0;
}

void dptx_apg_config(struct dptx *dptx, u8 intf_type,
			u8 chan_num, u8 data_width)
{
	struct device *dev = dptx->dev;
	struct regmap *base = dptx->regs[DPTX];
	u32 aud_cfg=0;
	u32 v_ctrl=0;
	u32 h_ctrl=0;

	aud_cfg = dptx_read_reg(dptx, base, AUD_CONFIG1);
	v_ctrl = dptx_read_reg(dptx, base, SDP_VERTICAL_CTRL);
	h_ctrl = dptx_read_reg(dptx, base, SDP_HORIZONTAL_CTRL);

	dev_info(dev, "AUD_CONFIG1=0x%08x\n", aud_cfg);
	dev_info(dev, "SDP_VERTICAL_CTRL=0x%08x\n", v_ctrl);
	dev_info(dev, "SDP_HORIZONTAL_CTRL=0x%08x\n", h_ctrl);

	_apg_ctx.is_param_valid = true;
	_apg_ctx.audio_size = 100;
	_apg_ctx.audio_file = NULL;
}