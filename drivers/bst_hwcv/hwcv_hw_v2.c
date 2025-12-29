// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/seq_file.h>
#include "hwcv_core.h"
#include "hwcv_hw_v2.h"
#include "hwcv_uapi.h"

#define POLL_SLEEP_US 100

static const char *const cv_reg_name[] = { "CV_SYS_CTRL_STATUS",
					   "CV_PARITY_CTRL_REG0",
					   "NOC_S_PORT_CHK_PTY_INTR_REG0",
					   "CV_INTR_EN_REG",
					   "CV_SUBMODULE_INTR_REG" };

static const u32 cv_reg_addr[] = { CV_SYS_CTRL_STATUS, CV_PARITY_CTRL_REG0,
				   NOC_S_PORT_CHK_PTY_INTR_REG0, CV_INTR_EN_REG,
				   CV_SUBMODULE_INTR_REG };

static const char *const scaler_reg_name[] = { "ENABLE",
					       "SYS_CTRL",
					       "MEMCTRL_INTR_STATUS",
					       "ALGO_INTR_STATUS",
					       "SRC_Y_ADDR",
					       "SRC_U_ADDR",
					       "SRC_V_ADDR",
					       "SRC_RESOLUTION",
					       "DST_Y_ADDR",
					       "DST_U_ADDR",
					       "DST_V_ADDR",
					       "DST_RESOLUTION",
					       "X_RATIO",
					       "X_INIT_PHASE",
					       "Y_RATIO",
					       "Y_INIT_PHASE",
					       "AXI_PARAM",
					       "AXI_STRIDE",
					       "COEFF_ADDR",
					       "COEFF_SIZE",
					       "DMA_PARA",
					       "CH0_BANK_REMAP",
					       "CH1_BANK_REMAP",
					       "RESERVED",
					       "DST_LAYER1_Y_ADDR",
					       "DST_LAYER1_U_ADDR",
					       "DST_LAYER1_V_ADDR",
					       "DST_LAYER2_Y_ADDR",
					       "DST_LAYER2_U_ADDR",
					       "DST_LAYER2_V_ADDR" };

static const char *const gwrap_normal_reg_name[] = {
	"ENABLE",	"SYS_CTRL",	"SRC_RESOLUTION", "DST_RESOLUTION",
	"SRC_BASE_CH0", "SRC_BASE_CH1", "SRC_BASE_CH2",	  "DST_BASE_CH0",
	"DST_BASE_CH1", "DST_BASE_CH2", "LUT_BASE",	  "SRC_STRIDE",
	"DST_STRIDE",	"LUT_STRIDE",	"AXI_PARAM",	  "INTR",
	"INTR_EN",	"SAFETY_ERROR", "SAFETY_MASK",	  "SAFETY_INJECT"
};

static const char *const gwrap_debug_reg_name[] = { "CRC_GRP0",
						    "CRC_GRP1",
						    "CRC_GRP2",
						    "CRC_GRP3",
						    "CRC_GRP4",
						    "CRC_GRP5",
						    "CRC_GRP6",
						    "CRC_GRP7",
						    "CFG_VIOLATION_INT",
						    "CFG_VIOLATION_MASK",
						    "DEBUG0",
						    "DEBUG1",
						    "DEBUG2",
						    "DEBUG3",
						    "DEBUG4",
						    "DEBUG5",
						    "DEBUG6",
						    "DEBUG7",
						    "DEBUG8",
						    "DEBUG9",
						    "DEBUG10",
						    "DEBUG11",
						    "DEBUG12",
						    "DEBUG13",
						    "DEBUG14",
						    "DEBUG15",
						    "DEBUG16",
						    "DEBUG17",
						    "DEBUG18",
						    "DEBUG19",
						    "DEBUG20",
						    "DEBUG21",
						    "DEBUG22",
						    "DEBUG23",
						    "DEBUG24" };

static const char *const gwrap_sbs_reg_name[] = {
	"ENABLE",	"SYS_CTRL",	"SRC_RESOLUTION",   "DST_RESOLUTION",
	"SRC_BASE_CH0", "SRC_BASE_CH1", "SRC_BASE_CH2",	    "DST_BASE_CH0",
	"DST_BASE_CH1", "DST_BASE_CH2", "LUT_BASE",	    "SRC_STRIDE",
	"DST_STRIDE",	"LUT_STRIDE",	"LUT_DISTRIBUTION", "UPDATE_SYNC",
	"CIRBUF_RID",	"CIRBUF_WID",	"ROWNUM_OFFSET"
};

static void enable_clk(struct hwcv_core *core)
{
	u32 val;

	val = hwcv_sys_read(core, CV_SYS_CTRL_STATUS);
	val |= (BIT_GWARP0_CLK_EN | BIT_GWARP1_CLK_EN | BIT_SCLR_CLK_EN |
		BIT_DMA_CLK_EN | BIT_SOFT_RST_GWARP0 | BIT_SOFT_RST_GWARP1 |
		BIT_SOFT_RST_SCLR | BIT_SOFT_RST_DMA);
	hwcv_sys_write(core, val, CV_SYS_CTRL_STATUS);
}

static void enable_ecc_pty(struct hwcv_core *core)
{
	u32 val;

	val = hwcv_sys_read(core, CV_PARITY_CTRL_REG0);
	val |= (BIT_INTERNAL_ECC_EN | BIT_INTERNAL_PTY_EN);
	hwcv_sys_write(core, val, CV_PARITY_CTRL_REG0);
}

static void enable_intr(struct hwcv_core *core)
{
	u32 val;

	val = hwcv_sys_read(core, CV_INTR_EN_REG);
	val |= (BIT_SCALER_FUNC_INTR_EN | BIT_GWARP0_FUNC_INTR_EN |
		BIT_GWARP1_FUNC_INTR_EN | BIT_FUNC_INTR_OUTPUT_EN);
	hwcv_sys_write(core, val, CV_INTR_EN_REG);
}

static int init_hw(struct hwcv_core *core)
{
	enable_clk(core);
	enable_ecc_pty(core);
	enable_intr(core);

	return 0;
}

static bool is_ready(struct hwcv_core *core)
{
	u32 val;

	val = hwcv_sys_read(core, CV_INTR_EN_REG);
	if (val & BIT_FUNC_INTR_OUTPUT_EN)
		return true;
	else
		return false;
}

static int debug_sys(struct hwcv_core *core, struct seq_file *m)
{
	int i;

	seq_printf(m, "%-30s | %-10s | %-10s\n", "name", "offset", "value");

	for (i = 0; i < ARRAY_SIZE(cv_reg_name); i++) {
		seq_printf(m, "%-30s | 0x%08x | 0x%08x\n", cv_reg_name[i],
			   cv_reg_addr[i], hwcv_sys_read(core, cv_reg_addr[i]));
	}

	return 0;
}

static int dump_sys_regs(struct hwcv_core *core)
{
	int i;

	dev_err(core->dev, "cv regs:");
	dev_err(core->dev, "%-30s | %-10s | %-10s\n", "name", "offset",
		"value");
	for (i = 0; i < ARRAY_SIZE(cv_reg_name); i++) {
		dev_err(core->dev, "%-30s | 0x%08x | 0x%08x\n", cv_reg_name[i],
			cv_reg_addr[i], hwcv_sys_read(core, cv_reg_addr[i]));
	}
	dev_err(core->dev,
		"--------------------------------------------------\n");

	return 0;
}

static int reset_scaler(struct hwcv_core *core)
{
	u32 val;

	hwcv_scaler_write(core, 0, SCLR_ENABLE);

	val = hwcv_sys_read(core, CV_SYS_CTRL_STATUS);
	val &= ~BIT_SOFT_RST_SCLR;
	hwcv_sys_write(core, val, CV_SYS_CTRL_STATUS);
	val |= BIT_SOFT_RST_SCLR;
	hwcv_sys_write(core, val, CV_SYS_CTRL_STATUS);

	return 0;
}

static int do_polyphase_scaler(struct hwcv_core *core,
			       struct hwcv_scaler_data *data)
{
	u32 val;

	/* sys_ctrl */
	val = 0;
	val = ((0x7 << 13) | (data->format << 3) | (0x0 << 2));
	hwcv_scaler_write(core, val, SCLR_SYS_CTRL);

	/* resolution */
	val = ((data->src_height << 16) | (data->src_width));
	hwcv_scaler_write(core, val, SCLR_SRC_RESOLUTION);
	val = ((data->dst_height << 16) | (data->dst_width));
	hwcv_scaler_write(core, val, SCLR_DST_RESOLUTION);

	/* ratio */
	hwcv_scaler_write(core, data->x_ratio, SCLR_X_RATIO);
	hwcv_scaler_write(core, data->y_ratio, SCLR_Y_RATIO);

	/* phase */
	hwcv_scaler_write(core, data->x_init_phase, SCLR_X_INIT_PHASE);

	/* stride */
	val = ((data->dst_stride << 16) | (data->src_stride));
	hwcv_scaler_write(core, val, SCLR_AXI_STRIDE);

	/* addr */
	hwcv_scaler_write(core, data->coeff_dma_addr, SCLR_COEFF_ADDR);
	hwcv_scaler_write(core, data->coeff_size, SCLR_COEFF_SIZE);
	hwcv_scaler_write(core, data->src_dma_addr[0], SCLR_SRC_Y_ADDR);
	hwcv_scaler_write(core, data->src_dma_addr[1], SCLR_SRC_U_ADDR);
	hwcv_scaler_write(core, data->src_dma_addr[2], SCLR_SRC_V_ADDR);
	hwcv_scaler_write(core, data->dst_dma_addr[0][0], SCLR_DST_Y_ADDR);
	hwcv_scaler_write(core, data->dst_dma_addr[0][1], SCLR_DST_U_ADDR);
	hwcv_scaler_write(core, data->dst_dma_addr[0][2], SCLR_DST_V_ADDR);

	/* enable */
	hwcv_scaler_write(core, 0x1, SCLR_ENABLE);

	return 0;
}

static int do_pyramid_scaler(struct hwcv_core *core,
			     struct hwcv_scaler_data *data)
{
	u32 val;

	/* sys_ctrl */
	val = 0;
	val = ((0x7 << 13) | (data->leftedge_split_flag << 9) |
	       (data->layer_num << 7) | (data->gauss_enable << 6) |
	       (data->format << 3) | (0x1 << 2));
	hwcv_scaler_write(core, val, SCLR_SYS_CTRL);

	/* resolution */
	val = ((data->src_height << 16) | (data->src_width));
	hwcv_scaler_write(core, val, SCLR_SRC_RESOLUTION);

	/* stride */
	val = ((data->dst_stride << 16) | (data->src_stride));
	hwcv_scaler_write(core, val, SCLR_AXI_STRIDE);

	/* addr */
	hwcv_scaler_write(core, data->src_dma_addr[0], SCLR_SRC_Y_ADDR);
	hwcv_scaler_write(core, data->src_dma_addr[1], SCLR_SRC_U_ADDR);
	hwcv_scaler_write(core, data->src_dma_addr[2], SCLR_SRC_V_ADDR);
	hwcv_scaler_write(core, data->dst_dma_addr[0][0], SCLR_DST_Y_ADDR);
	hwcv_scaler_write(core, data->dst_dma_addr[0][1], SCLR_DST_U_ADDR);
	hwcv_scaler_write(core, data->dst_dma_addr[0][2], SCLR_DST_V_ADDR);
	hwcv_scaler_write(core, data->dst_dma_addr[1][0],
			  SCLR_DST_LAYER1_Y_ADDR);
	hwcv_scaler_write(core, data->dst_dma_addr[1][1],
			  SCLR_DST_LAYER1_U_ADDR);
	hwcv_scaler_write(core, data->dst_dma_addr[1][2],
			  SCLR_DST_LAYER1_V_ADDR);
	hwcv_scaler_write(core, data->dst_dma_addr[2][0],
			  SCLR_DST_LAYER2_Y_ADDR);
	hwcv_scaler_write(core, data->dst_dma_addr[2][1],
			  SCLR_DST_LAYER2_U_ADDR);
	hwcv_scaler_write(core, data->dst_dma_addr[2][2],
			  SCLR_DST_LAYER2_V_ADDR);

	/* enable */
	hwcv_scaler_write(core, 1, SCLR_ENABLE);

	return 0;
}

static int do_scaler(struct hwcv_core *core, void *param)
{
	int ret;
	struct hwcv_scaler_data *data = param;

	if (data->mode == HWCV_SCALER_POLYPHASE)
		ret = do_polyphase_scaler(core, data);
	else
		ret = do_pyramid_scaler(core, data);

	return ret;
}

static int debug_scaler(struct hwcv_core *core, struct seq_file *m)
{
	int i;

	seq_printf(m, "%-20s | %-10s | %-10s\n", "name", "offset", "value");
	seq_puts(m, "--------------------------------------------------\n");
	for (i = 0; i < ARRAY_SIZE(scaler_reg_name); i++) {
		seq_printf(m, "%-20s | 0x%08x | 0x%08x\n", scaler_reg_name[i],
			   i * 4, hwcv_scaler_read(core, i * 4));
	}

	return 0;
}

static int dump_scaler_regs(struct hwcv_core *core)
{
	int i;

	dev_err(core->dev, "***SCALER INTERRUPT TIMEOUT REGISTER DUMP ***\n");
	dev_err(core->dev,
		"==================================================\n");
	dump_sys_regs(core);
	dev_err(core->dev, "sclaer regs:\n");
	dev_err(core->dev, "%-20s | %-10s | %-10s\n", "name", "offset",
		"value");
	for (i = 0; i < ARRAY_SIZE(scaler_reg_name); i++) {
		dev_err(core->dev, "%-20s | 0x%08x | 0x%08x\n",
			scaler_reg_name[i], i * 4,
			hwcv_scaler_read(core, i * 4));
	}
	dev_err(core->dev,
		"==================================================\n");

	return 0;
}

static int reset_gwarp(struct hwcv_core *core, u8 id)
{
	int i;
	u32 val;
	u32 mask;

	mask = (id == 0) ? BIT_SOFT_RST_GWARP0 : BIT_SOFT_RST_GWARP1;

	hwcv_gwarp_write(core, id, 0x0, GWC_ENABLE);
	for (i = 0; i < HWCV_GWARP_SBS_SENSOR_NUM; i++) {
		hwcv_gwarp_write(core, id, 0x0,
				 SNR0_GWC_ENABLE + SNR_GWC_OFFSET * i);
	}

	val = hwcv_sys_read(core, CV_SYS_CTRL_STATUS);
	val &= ~mask;
	hwcv_sys_write(core, val, CV_SYS_CTRL_STATUS);
	val |= mask;
	hwcv_sys_write(core, val, CV_SYS_CTRL_STATUS);

	return 0;
}

static int do_normal_gwarp(struct hwcv_core *core, struct hwcv_gwarp_data *data)
{
	u32 val;
	u8 id = data->engine_id;

	/* sys_ctrl */
	val = ((data->src_format) | (data->dst_format << 8) |
	       (data->interpolation << 16) | (0 << 24));
	hwcv_gwarp_write(core, id, val, GWC_SYS_CTRL);

	/* resolution */
	val = ((data->src_height << 16) | (data->src_width));
	hwcv_gwarp_write(core, id, val, GWC_SRC_RESOLUTION);
	val = ((data->dst_height << 16) | (data->dst_width));
	hwcv_gwarp_write(core, id, val, GWC_DST_RESOLUTION);

	/* addr */
	hwcv_gwarp_write(core, id, data->src_dma_addr[0], GWC_SRC_BASE_CH0);
	hwcv_gwarp_write(core, id, data->src_dma_addr[1], GWC_SRC_BASE_CH1);
	hwcv_gwarp_write(core, id, data->src_dma_addr[2], GWC_SRC_BASE_CH2);
	hwcv_gwarp_write(core, id, data->dst_dma_addr[0], GWC_DST_BASE_CH0);
	hwcv_gwarp_write(core, id, data->dst_dma_addr[1], GWC_DST_BASE_CH1);
	hwcv_gwarp_write(core, id, data->dst_dma_addr[2], GWC_DST_BASE_CH2);
	hwcv_gwarp_write(core, id, data->lut_dma_addr, GWC_LUT_BASE);

	/* stride */
	hwcv_gwarp_write(core, id, data->src_stride, GWC_SRC_STRIDE);
	hwcv_gwarp_write(core, id, data->dst_stride, GWC_DST_STRIDE);
	hwcv_gwarp_write(core, id, data->lut_stride, GWC_LUT_STRIDE);

	/* intr */
	hwcv_gwarp_write(core, id, 0x1, GWC_INTR_EN);

	/* enable */
	hwcv_gwarp_write(core, id, 0x1, GWC_ENABLE);

	return 0;
}

static int do_sbs_gwarp(struct hwcv_core *core, struct hwcv_gwarp_data *data)
{
	u32 val;
	u8 id = data->engine_id;
	u8 sid = data->sensor_id;
	u32 offset = sid * SNR_GWC_OFFSET;

	/* normal sys_ctrl */
	val = (0x1 << 24);
	hwcv_gwarp_write(core, id, val, GWC_SYS_CTRL);

	/* sbs sys_ctrl */
	val = ((data->src_format) | (data->dst_format << 8) |
	       (data->interpolation << 16) | (sid << 24));
	hwcv_gwarp_write(core, id, val, SNR0_GWC_SYS_CTRL + offset);

	/* resolution */
	val = ((data->src_height << 16) | (data->src_width));
	hwcv_gwarp_write(core, id, val, SNR0_SRC_RESOLUTION + offset);
	val = ((data->dst_height << 16) | (data->dst_width));
	hwcv_gwarp_write(core, id, val, SNR0_DST_RESOLUTION + offset);

	/* addr */
	hwcv_gwarp_write(core, id, data->src_dma_addr[0],
			 SNR0_GWC_SRC_BASE_CH0 + offset);
	hwcv_gwarp_write(core, id, data->src_dma_addr[1],
			 SNR0_GWC_SRC_BASE_CH1 + offset);
	hwcv_gwarp_write(core, id, data->src_dma_addr[2],
			 SNR0_GWC_SRC_BASE_CH2 + offset);
	hwcv_gwarp_write(core, id, data->dst_dma_addr[0],
			 SNR0_GWC_DST_BASE_CH0 + offset);
	hwcv_gwarp_write(core, id, data->dst_dma_addr[1],
			 SNR0_GWC_DST_BASE_CH1 + offset);
	hwcv_gwarp_write(core, id, data->dst_dma_addr[2],
			 SNR0_GWC_DST_BASE_CH2 + offset);
	hwcv_gwarp_write(core, id, data->lut_dma_addr,
			 SNR0_GWC_LUT_BASE + offset);

	/* stride */
	hwcv_gwarp_write(core, id, data->src_stride,
			 SNR0_GWC_SRC_STRIDE + offset);
	hwcv_gwarp_write(core, id, data->dst_stride,
			 SNR0_GWC_DST_STRIDE + offset);
	hwcv_gwarp_write(core, id, data->lut_stride,
			 SNR0_GWC_LUT_STRIDE + offset);

	/* intr */
	val = (0x1 << (8 + sid));
	hwcv_gwarp_write(core, id, val, GWC_INTR_EN);

	/* enable */
	hwcv_gwarp_write(core, id, 0x1, SNR0_GWC_ENABLE + offset);

	/* update sync */
	hwcv_gwarp_write(core, id, 0x00010300, SNR0_GWC_UPDATE_SYNC + offset);
	hwcv_gwarp_write(core, id, 0x00000300, SNR0_GWC_UPDATE_SYNC + offset);

	return 0;
}

static int do_gwarp(struct hwcv_core *core, void *param)
{
	int ret;
	struct hwcv_gwarp_data *data = param;

	if (data->mode == HWCV_GWARP_NORMAL)
		ret = do_normal_gwarp(core, data);
	else
		ret = do_sbs_gwarp(core, data);

	return ret;
}

static int debug_gwarp(struct hwcv_core *core, struct seq_file *m)
{
	int i;
	int id;
	int sid;

	for (id = 0; id < HWCV_GWARP_NUM; id++) {
		seq_printf(m, "Gwarp %d:\n", id);
		seq_puts(
			m,
			"==================================================\n");

		seq_puts(m, "Normal Part:\n");
		seq_puts(
			m,
			"--------------------------------------------------\n");
		seq_printf(m, "%-20s | %-10s | %-10s\n", "name", "offset",
			   "value");
		for (i = 0; i < ARRAY_SIZE(gwrap_normal_reg_name); i++) {
			seq_printf(m, "%-20s | 0x%08x | 0x%08x\n",
				   gwrap_normal_reg_name[i], i * 4,
				   hwcv_gwarp_read(core, id, i * 4));
		}
		seq_puts(
			m,
			"--------------------------------------------------\n");

		seq_puts(m, "Sbs Part:\n");
		seq_puts(
			m,
			"--------------------------------------------------\n");
		for (sid = 0; sid < HWCV_GWARP_SBS_SENSOR_NUM; sid++) {
			seq_printf(m, "Sensor %d:\n", sid);
			seq_printf(m, "%-20s | %-10s | %-10s\n", "name",
				   "offset", "value");
			for (i = 0; i < ARRAY_SIZE(gwrap_sbs_reg_name); i++) {
				seq_printf(m, "%-20s | 0x%08x | 0x%08x\n",
					   gwrap_sbs_reg_name[i], i * 4,
					   hwcv_gwarp_read(
						   core, id,
						   i * 4 + SNR_GWC_OFFSET *
								   (sid + 1)));
			}
		}
		seq_puts(
			m,
			"--------------------------------------------------\n");

		seq_puts(
			m,
			"==================================================\n");
	}

	return 0;
}

static int dump_gwarp_regs(struct hwcv_core *core, u8 id)
{
	int i;

	dev_err(core->dev, "***GWARP %d INTERRUPT TIMEOUT REGISTER DUMP ***\n",
		id);
	dev_err(core->dev,
		"==================================================\n");
	dump_sys_regs(core);
	dev_err(core->dev, "Gwarp regs:\n");
	dev_err(core->dev, "%-4s | %-20s | %-10s | %-10s\n", "id", "name",
		"offset", "value");
	for (i = 0; i < ARRAY_SIZE(gwrap_normal_reg_name); i++) {
		dev_err(core->dev, "%-4d | %-20s | 0x%08x | 0x%08x\n", id,
			gwrap_normal_reg_name[i], i * 4,
			hwcv_gwarp_read(core, id, i * 4));
	}

	for (i = 0; i < ARRAY_SIZE(gwrap_debug_reg_name); i++) {
		dev_err(core->dev, "%-4d | %-20s | 0x%08x | 0x%08x\n", id,
			gwrap_debug_reg_name[i], i * 4 + GWC_CRC_GRP0,
			hwcv_gwarp_read(core, id, i * 4 + GWC_CRC_GRP0));
	}
	dev_err(core->dev,
		"==================================================\n");

	return 0;
}

static int irq(struct hwcv_core *core)
{
	int i;
	u32 int_status;
	u32 int_clr;
	bool valid_intr = false;
	unsigned long offset =
		HWCV_GWARP1_NORMAL_DONE - HWCV_GWARP0_NORMAL_DONE;

	pr_debug("Receive shared intr, current request state is [0x%lx]\n",
		 core->request_state);

	if (!core->request_state && HWCV_REQ_MASK) {
		pr_debug("No request, maybe from other system\n");
		return IRQ_NONE;
	}

	if (test_bit(HWCV_REQ_SCALER, &core->request_state)) {
		int_status = hwcv_scaler_read(core, SCLR_MEMCTRL_INTR_STATUS);
		if (int_status & BIT_CSR_FRAME_DONE_INTR) {
			core->scaler_profiling.frame_done = ktime_get();
			core->scaler_profiling.work_cycle =
				hwcv_scaler_read(core, SCLR_FRAME_CYCLE_CNT);
			pr_debug("Receive scaler intr\n");

			set_bit(HWCV_SCALER_DONE, &core->job_state);

			/* clear scaler intr */
			int_clr = hwcv_scaler_read(core, SCLR_SYS_CTRL);
			int_clr |= BIT_FRAME_DONE_INTR_CLEAN;
			hwcv_scaler_write(core, int_clr, SCLR_SYS_CTRL);
			int_clr &= ~BIT_FRAME_DONE_INTR_CLEAN;
			hwcv_scaler_write(core, int_clr, SCLR_SYS_CTRL);

			/* reset scaler */
			reset_scaler(core);

			valid_intr = true;
		}
	}

	for (i = 0; i < HWCV_GWARP_NUM; i++) {
		if (test_bit(HWCV_REQ_GWARP0 + i, &core->request_state)) {
			int_status = hwcv_gwarp_read(core, i, GWC_INTR);

			if (int_status & BIT_CSR_GWC_INTR) {
				core->gwarp_profiling[i].frame_done =
					ktime_get();
				core->gwarp_profiling[i].work_cycle =
					hwcv_gwarp_read(core, i,
							CSR_FRAME_CYCLE_CNT);
				pr_debug("Receive gwarp[%d] intr\n", i);

				set_bit(HWCV_GWARP0_NORMAL_DONE + i * offset,
					&core->job_state);

				/* clear scaler intr */
				hwcv_gwarp_write(core, i, BIT_CSR_GWC_INTR,
						 GWC_INTR);

				/* reset gwarp */
				hwcv_gwarp_write(core, i, 0x0, GWC_ENABLE);

				valid_intr = true;
			} else if (int_status & BIT_CSR_SNR0_INTR) {
				set_bit(HWCV_GWARP0_SNR0_DONE + i * offset,
					&core->job_state);
				hwcv_gwarp_write(core, i, BIT_CSR_SNR0_INTR,
						 GWC_INTR);
				hwcv_gwarp_write(core, i, 0x0, SNR0_GWC_ENABLE);
				valid_intr = true;
			} else if (int_status & BIT_CSR_SNR1_INTR) {
				set_bit(HWCV_GWARP0_SNR1_DONE + i * offset,
					&core->job_state);
				hwcv_gwarp_write(core, i, BIT_CSR_SNR1_INTR,
						 GWC_INTR);
				hwcv_gwarp_write(core, i, 0x0,
						 SNR0_GWC_ENABLE +
							 SNR_GWC_OFFSET);
				valid_intr = true;
			} else if (int_status & BIT_CSR_SNR2_INTR) {
				set_bit(HWCV_GWARP0_SNR2_DONE + i * offset,
					&core->job_state);
				hwcv_gwarp_write(core, i, BIT_CSR_SNR2_INTR,
						 GWC_INTR);
				hwcv_gwarp_write(core, i, 0x0,
						 SNR0_GWC_ENABLE +
							 SNR_GWC_OFFSET * 2);
				valid_intr = true;
			} else if (int_status & BIT_CSR_SNR3_INTR) {
				set_bit(HWCV_GWARP0_SNR3_DONE + i * offset,
					&core->job_state);
				hwcv_gwarp_write(core, i, BIT_CSR_SNR3_INTR,
						 GWC_INTR);
				hwcv_gwarp_write(core, i, 0x0,
						 SNR0_GWC_ENABLE +
							 SNR_GWC_OFFSET * 3);
				valid_intr = true;
			}
		}
	}

	if (!valid_intr) {
		pr_debug(
			"Do not match request[0x%lx], maybe from other system\n",
			core->request_state);
		return IRQ_NONE;
	}

	return IRQ_WAKE_THREAD;
}

static int isr_thread(struct hwcv_core *core)
{
	pr_debug("called\n");
	return IRQ_HANDLED;
}

/* poll */
static int poll_scaler(struct hwcv_core *core, u64 timeout_us)
{
	int ret;
	u32 int_status;
	u32 int_clr;

	ret = read_poll_timeout(hwcv_scaler_read, int_status,
				(int_status & BIT_CSR_FRAME_DONE_INTR),
				POLL_SLEEP_US, timeout_us, false, core,
				SCLR_MEMCTRL_INTR_STATUS);
	if (ret < 0)
		return ret;

	core->scaler_profiling.frame_done = ktime_get();
	core->scaler_profiling.work_cycle =
		hwcv_scaler_read(core, SCLR_FRAME_CYCLE_CNT);

	/* clear scaler intr */
	int_clr = hwcv_scaler_read(core, SCLR_SYS_CTRL);
	int_clr |= BIT_FRAME_DONE_INTR_CLEAN;
	hwcv_scaler_write(core, int_clr, SCLR_SYS_CTRL);
	int_clr &= ~BIT_FRAME_DONE_INTR_CLEAN;
	hwcv_scaler_write(core, int_clr, SCLR_SYS_CTRL);

	/* reset scaler */
	reset_scaler(core);

	return 0;
}

static int poll_gwarp(struct hwcv_core *core, u8 id, u64 timeout_us)
{
	int ret;
	u32 int_status;

	ret = read_poll_timeout(hwcv_gwarp_read, int_status,
				(int_status & BIT_CSR_GWC_INTR), POLL_SLEEP_US,
				timeout_us, false, core, id, GWC_INTR);
	if (ret < 0)
		return ret;

	core->gwarp_profiling[id].frame_done = ktime_get();
	core->gwarp_profiling[id].work_cycle =
		hwcv_gwarp_read(core, id, CSR_FRAME_CYCLE_CNT);

	hwcv_gwarp_write(core, id, BIT_CSR_GWC_INTR, GWC_INTR);
	hwcv_gwarp_write(core, id, 0x0, GWC_ENABLE);

	return 0;
}

const struct hwcv_backend_ops hwcv_v2_ops = {
	.init_hw = init_hw,
	.is_ready = is_ready,
	.debug_sys = debug_sys,

	.reset_scaler = reset_scaler,
	.do_scaler = do_scaler,
	.debug_scaler = debug_scaler,
	.poll_scaler = poll_scaler,
	.dump_scaler_regs = dump_scaler_regs,

	.reset_gwarp = reset_gwarp,
	.do_gwarp = do_gwarp,
	.debug_gwarp = debug_gwarp,
	.poll_gwarp = poll_gwarp,
	.dump_gwarp_regs = dump_gwarp_regs,

	.irq = irq,
	.isr_thread = isr_thread,
};

const struct hwcv_hw_data hwcv_v2_data = {
	.gwarp_num = HWCV_GWARP_NUM,
	.support_sbs = true,
};
