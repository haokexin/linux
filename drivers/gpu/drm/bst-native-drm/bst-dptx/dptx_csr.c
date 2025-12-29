// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#include "dptx_drv.h"
#include "dptx_csr.h"
#include "dptx_phy.h"
#include "bst_disp_conn.h"
#include "bst_dpu_csr.h"

void dptx_csr_reset(struct dptx *dptx)
{
	u32 crm_ctrl = 0;
	u32 phy_ctrl;

	phy_ctrl = dptx_read_reg(dptx, dptx->regs[DPTX_CSR], EDP_CSR_PHY_CTRL0);
	phy_ctrl &= ~BIT(9); /* 0: top crm referece clk 1: use pad clk*/
	dptx_write_reg(dptx, dptx->regs[DPTX_CSR], EDP_CSR_PHY_CTRL0, phy_ctrl);
#if defined(CONFIG_C1200_SLT)
	dptx_phy_firmware_load_from(dptx, ONLY_SRAM);
#endif
	crm_ctrl = dptx_read_reg(dptx, dptx->regs[DPTX_CSR], EDP_CSR_CRM_CTRL);
	dptx_write_reg(dptx, dptx->regs[DPTX_CSR], EDP_CSR_CRM_CTRL, 0x102D32D);
#if defined(CONFIG_C1200_SLT)
	dptx_wait_phy_boot_done(dptx, ONLY_SRAM);
#endif
	/* SUP_DIG_LVL_OVRD_IN */
	dptx_u3_phy_write_reg(dptx, 0x22, 0xd5);
}

void dptx_csr_func_irq_dis_all(struct dptx *dptx)
{
	struct device *dev;
	u32 parity_ctrl = 0;

	dev = dptx->dev;
	parity_ctrl =
		dptx_read_reg(dptx, dptx->regs[DPTX_CSR], EDP_CSR_PARITY_CTRL);
	parity_ctrl |= (0x7FU << 22);
	dptx_write_reg(dptx, dptx->regs[DPTX_CSR], EDP_CSR_PARITY_CTRL,
		       parity_ctrl);
	parity_ctrl =
		dptx_read_reg(dptx, dptx->regs[DPTX_CSR], EDP_CSR_PARITY_CTRL);
}

void dptx_csr_func_irq_en_dptx(struct dptx *dptx)
{
	u32 parity_ctrl = 0;

	parity_ctrl =
		dptx_read_reg(dptx, dptx->regs[DPTX_CSR], EDP_CSR_PARITY_CTRL);
	parity_ctrl &= ~BIT(27);
	parity_ctrl &= ~BIT(21); /* parity_int */
	dptx_write_reg(dptx, dptx->regs[DPTX_CSR], EDP_CSR_PARITY_CTRL,
		       parity_ctrl);
	parity_ctrl =
		dptx_read_reg(dptx, dptx->regs[DPTX_CSR], EDP_CSR_PARITY_CTRL);
}

#define EDP_CSR_VIDEO_MODE_DEFAULT    0x0
#define EDP_CSR_VIDEO_MODE_YUV422     0x1
#define EDP_CSR_VIDEO_MODE_YUV420     0x2
#define EDP_CSR_VIDEO_MODE_YONLY      0x4
#define EDP_CSR_IPI_FMT_RGB	      0x0
#define EDP_CSR_IPI_FMT_YCbCr422      0x1
#define EDP_CSR_IPI_FMT_YCbCr444      0x2
#define EDP_CSR_IPI_FMT_YCbCr420      0x3
#define EDP_CSR_IPI_FMT_YONLY	      0x19
#define EDP_CSR_IPI_FMT_RAW	      0x6
#define EDP_CSR_IPI_FMT_DSC	      0xB
#define EDP_CSR_COLOR_DEPTH_6BIT      0x3
#define EDP_CSR_COLOR_DEPTH_8BIT      0x5
#define EDP_CSR_COLOR_DEPTH_10BIT     0x6
#define EDP_CSR_COLOR_DEPTH_12BIT     0x7
#define EDP_CSR_COLOR_DEPTH_16BIT     0x9
#define EDP_CSR_COLOR_DEPTH_DSC_48BIT 0xE
#define EDP_CSR_DEN_POL_HIGH	      0x1
#define EDP_CSR_DEN_POL_LOW	      0x0

static void edp_csr_video_ctrl_clear(struct dptx *dptx)
{
	dptx_write_reg(dptx, dptx->regs[DPTX_CSR], EDP_CSR_W1N_CTRL,
		       EDP_CSR_VIDEO_CTRL_CLR);
}

void dptx_csr_force_hpd(struct dptx *dptx, bool force) {
	u32 value;

	value =	dptx_read_reg(dptx, dptx->regs[DPTX_CSR], EDP_CSR_DPTX_CTRL);
	if (force)
		value |= BIT(3) | BIT(4);
	else
		value = (~(BIT(3) | BIT(4)) & value);
	dptx_write_reg(dptx, dptx->regs[DPTX_CSR], EDP_CSR_DPTX_CTRL, value);
}

int32_t dptx_csr_set_video_ctrl(struct dptx *dptx)
{
	u32 crs_video_ctrl = 0;
	struct video_params *params = &dptx->vparams;

	edp_csr_video_ctrl_clear(dptx);

	switch (params->pix_enc) {
	case YCBCR422:
		if (params->bpc == 8) {
			crs_video_ctrl = EDP_CSR_MK_VIDEO_CTRL(EDP_CSR_VIDEO_MODE_YUV422,
					      EDP_CSR_IPI_FMT_YCbCr422,
					      EDP_CSR_DEN_POL_HIGH,
					      EDP_CSR_COLOR_DEPTH_8BIT);
		} else if (params->bpc == 10) {
			crs_video_ctrl = EDP_CSR_MK_VIDEO_CTRL(
				EDP_CSR_VIDEO_MODE_YUV422,
				EDP_CSR_IPI_FMT_YCbCr422, EDP_CSR_DEN_POL_HIGH,
				EDP_CSR_COLOR_DEPTH_10BIT);
		} else if (params->bpc == 12) {
			crs_video_ctrl = EDP_CSR_MK_VIDEO_CTRL(
							EDP_CSR_VIDEO_MODE_YUV422,
							EDP_CSR_IPI_FMT_YCbCr422, EDP_CSR_DEN_POL_HIGH,
							EDP_CSR_COLOR_DEPTH_12BIT);
		} else if  (params->bpc == 16) {
			crs_video_ctrl = EDP_CSR_MK_VIDEO_CTRL(
							EDP_CSR_VIDEO_MODE_YUV422,
							EDP_CSR_IPI_FMT_YCbCr422, EDP_CSR_DEN_POL_HIGH,
							EDP_CSR_COLOR_DEPTH_16BIT);
		}
		break;
	case YCBCR420:
		if (params->bpc == 8) {
			crs_video_ctrl = EDP_CSR_MK_VIDEO_CTRL(
				EDP_CSR_VIDEO_MODE_YUV420,
				EDP_CSR_IPI_FMT_YCbCr420, EDP_CSR_DEN_POL_HIGH,
				EDP_CSR_COLOR_DEPTH_8BIT);
		} else if (params->bpc == 10) {
			crs_video_ctrl = EDP_CSR_MK_VIDEO_CTRL(
				EDP_CSR_VIDEO_MODE_YUV420,
				EDP_CSR_IPI_FMT_YCbCr420, EDP_CSR_DEN_POL_HIGH,
				EDP_CSR_COLOR_DEPTH_10BIT);
		} else if (params->bpc == 12) {
			crs_video_ctrl = EDP_CSR_MK_VIDEO_CTRL(
							EDP_CSR_VIDEO_MODE_YUV420,
							EDP_CSR_IPI_FMT_YCbCr420, EDP_CSR_DEN_POL_HIGH,
							EDP_CSR_COLOR_DEPTH_12BIT);
		} else if  (params->bpc == 16) {
			crs_video_ctrl = EDP_CSR_MK_VIDEO_CTRL(
							EDP_CSR_VIDEO_MODE_YUV420,
							EDP_CSR_IPI_FMT_YCbCr420, EDP_CSR_DEN_POL_HIGH,
							EDP_CSR_COLOR_DEPTH_16BIT);
		}
		break;
	case RGB:
		if (params->bpc == 8) {
			crs_video_ctrl = EDP_CSR_MK_VIDEO_CTRL(
				EDP_CSR_VIDEO_MODE_DEFAULT, EDP_CSR_IPI_FMT_RGB,
				EDP_CSR_DEN_POL_HIGH, EDP_CSR_COLOR_DEPTH_8BIT);
		} else if (params->bpc == 10) {
			crs_video_ctrl = EDP_CSR_MK_VIDEO_CTRL(
				EDP_CSR_VIDEO_MODE_DEFAULT, EDP_CSR_IPI_FMT_RGB,
				EDP_CSR_DEN_POL_HIGH,
				EDP_CSR_COLOR_DEPTH_10BIT);
		} else if (params->bpc == 12) {
			crs_video_ctrl = EDP_CSR_MK_VIDEO_CTRL(
							EDP_CSR_VIDEO_MODE_DEFAULT,
							EDP_CSR_IPI_FMT_RGB, EDP_CSR_DEN_POL_HIGH,
							EDP_CSR_COLOR_DEPTH_12BIT);
		} else if  (params->bpc == 16) {
			crs_video_ctrl = EDP_CSR_MK_VIDEO_CTRL(
							EDP_CSR_VIDEO_MODE_DEFAULT,
							EDP_CSR_IPI_FMT_RGB, EDP_CSR_DEN_POL_HIGH,
							EDP_CSR_COLOR_DEPTH_16BIT);
		}
	break;
	case YCBCR444:
		if (params->bpc == 8) {
			crs_video_ctrl = EDP_CSR_MK_VIDEO_CTRL(
				EDP_CSR_VIDEO_MODE_DEFAULT, EDP_CSR_IPI_FMT_YCbCr444,
				EDP_CSR_DEN_POL_HIGH, EDP_CSR_COLOR_DEPTH_8BIT);
		} else if (params->bpc == 10) {
			crs_video_ctrl = EDP_CSR_MK_VIDEO_CTRL(
				EDP_CSR_VIDEO_MODE_DEFAULT, EDP_CSR_IPI_FMT_YCbCr444,
				EDP_CSR_DEN_POL_HIGH,
				EDP_CSR_COLOR_DEPTH_10BIT);
		} else if (params->bpc == 12) {
			crs_video_ctrl = EDP_CSR_MK_VIDEO_CTRL(
							EDP_CSR_VIDEO_MODE_DEFAULT,
							EDP_CSR_IPI_FMT_YCbCr444, EDP_CSR_DEN_POL_HIGH,
							EDP_CSR_COLOR_DEPTH_12BIT);
		} else if  (params->bpc == 16) {
			crs_video_ctrl = EDP_CSR_MK_VIDEO_CTRL(
							EDP_CSR_VIDEO_MODE_DEFAULT,
							EDP_CSR_IPI_FMT_YCbCr444, EDP_CSR_DEN_POL_HIGH,
							EDP_CSR_COLOR_DEPTH_16BIT);
		}
	break;
	case YONLY:
		if (params->bpc == 8) {
			crs_video_ctrl = EDP_CSR_MK_VIDEO_CTRL(
				EDP_CSR_VIDEO_MODE_YONLY, EDP_CSR_IPI_FMT_YONLY,
				EDP_CSR_DEN_POL_HIGH, EDP_CSR_COLOR_DEPTH_8BIT);
		} else if (params->bpc == 10) {
			crs_video_ctrl = EDP_CSR_MK_VIDEO_CTRL(
				EDP_CSR_VIDEO_MODE_YONLY, EDP_CSR_IPI_FMT_YONLY,
				EDP_CSR_DEN_POL_HIGH,
				EDP_CSR_COLOR_DEPTH_10BIT);
		}
	break;
	default:
		dptx_err(dptx,"not support bpc[%d] and pix_enc[%d] for edp csr\n",
			    params->bpc, params->pix_enc);
		return -1;
	}
	dev_dbg(dptx->dev, "crs_video_ctrl:%#x\n", crs_video_ctrl);
	dptx_write_reg(dptx, dptx->regs[DPTX_CSR], EDP_CSR_VIDEO_CTRL,
			       crs_video_ctrl);

	return 0;
}

int dptx_init_remote_source(struct dptx *dptx)
{
	struct bst_dpu_connection conn;
	int ret;

	ret = bst_get_remote_dpu_connection_by_port(dptx->dev, 0, &conn);
	if (ret) {
		dev_warn(dptx->dev, "dptx not connect to dpu!\n");
		return ret;
	}
	bst_select_dpu_output_to_edp(&conn);
	dev_dbg(dptx->dev, "dptx connect to dpu:%d, pipe:%d, link:%d!\n",
		conn.port.dpu_id, conn.port.pipeline_id, conn.port.link_id);

	dptx->host_dpu = conn.host;
	bst_dpu_check_and_release(dptx->host_dpu);
	return 0;
}

int dptx_mux_enable(struct dptx *dptx, bool enable) {
	struct bst_dpu_connection conn;
	int ret;

	ret = bst_get_remote_dpu_connection_by_port(dptx->dev, 0, &conn);
	if (ret) {
		dev_warn(dptx->dev, "dptx not connect to dpu!\n");
		return ret;
	}
	if (enable) {
		bst_select_dpu_output_to_edp(&conn);
		dev_dbg(dptx->dev, "dptx connect to dpu:%d, pipe:%d, link:%d!\n",
			conn.port.dpu_id, conn.port.pipeline_id, conn.port.link_id);
	} else {
		conn.port.link_id = 1;
		bst_select_dpu_output_to_edp(&conn);
		dev_dbg(dptx->dev, "dptx connect to dpu:%d, pipe:%d, link:%d!\n",
			conn.port.dpu_id, conn.port.pipeline_id, conn.port.link_id);
	}
	return 0;
}