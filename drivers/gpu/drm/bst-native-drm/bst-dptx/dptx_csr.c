// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#include "dptx_drv.h"
#include "bst_disp_conn.h"
#include "bst_dpu_csr.h"
/*******************************
 *    LOCAL crs registers
********************************/
#define EDP_CSR_CRM_CTRL			 0x00
#define EDP_CSR_PHY_CTRL0			 0x04
#define EDP_CSR_PHY_CTRL1			 0x08
#define EDP_CSR_FREQ_AUXCLK_MONITOR_CTRL0	 0x0C
#define EDP_CSR_FREQ_AUXCLK_MONITOR_CTRL1	 0x10
#define EDP_CSR_PHY0_CR_CTRL0			 0x14
#define EDP_CSR_PHY0_CR_CTRL1			 0x18
#define EDP_CSR_PHY_EXT_CTRL0			 0x1C
#define EDP_CSR_PHY_EXT_CTRL1			 0x20
#define EDP_CSR_PHY_EXT_CTRL2			 0x24
#define EDP_CSR_PHY_EXT_CTRL3			 0x28
#define EDP_CSR_PHY_EXT_CTRL4			 0x2C
#define EDP_CSR_PHY_EXT_CTRL5			 0x30
#define EDP_CSR_PHY_EXT_CTRL6			 0x34
#define EDP_CSR_PHY_EXT_CTRL7			 0x38
#define EDP_CSR_PHY_EXT_CTRL8			 0x3C
#define EDP_CSR_PHY_EXT_CTRL9			 0x40
#define EDP_CSR_PHY_EXT_CTRL10			 0x44
#define EDP_CSR_PHY_EXT_CTRL11			 0x48
#define EDP_CSR_AUX_PHY_CTRL			 0x4C
#define EDP_CSR_ASE_DEBUG_CTRL			 0x50
#define EDP_CSR_ASE_DEBUG_DATA			 0x54
#define EDP_CSR_DPTX_CTRL			 0x58
#define EDP_CSR_PARITY_CTRL			 0x5C
#define EDP_CSR_TRNG_KPF_DATA0			 0x60
#define EDP_CSR_TRNG_KPF_DATA1			 0x64
#define EDP_CSR_TRNG_KPF_DATA2			 0x68
#define EDP_CSR_TRNG_KPF_DATA3			 0x6C
#define EDP_CSR_KPF_REC_CTRL			 0x70
#define EDP_CSR_SINK_CTRL			 0x74
#define EDP_CSR_ATE_CTRL			 0x78
#define EDP_CSR_REG_WR_PROTECT			 0x7C
#define EDP_CSR_RSV0				 0x80
#define EDP_CSR_RSV1				 0x84
#define EDP_CSR_RSV2				 0x88
#define EDP_CSR_RSV3				 0x8C
#define EDP_CSR_FREQ_PIXELCLK_DIV2_MONITOR_CTRL0 0x90
#define EDP_CSR_FREQ_PIXELCLK_DIV2_MONITOR_CTRL1 0x94
#define EDP_CSR_VIDEO_CTRL			 0x98
#define EDP_CSR_FREQ_MONITOR_CTRL		 0x9C
#define EDP_CSR_DPALT_CTRL			 0xA0
#define EDP_CSR_PIPE_LANE0_CTRL			 0xAC
#define EDP_CSR_DEBUG_CTRL			 0xB0
#define EDP_CSR_W1N_CTRL			 0xB4
#define EDP_CSR_PHY_STATE			 0x200
#define EDP_CSR_PHY_MPLL_STATE			 0x204
#define EDP_CSR_FREQ_MONITOR_STATE		 0x208
#define EDP_CSR_PHY0_CR_PARA_STATE		 0x20C
#define EDP_CSR_INTERRUPT_STATUS		 0x210
#define EDP_CSR_DPTX_STA0			 0x214
#define EDP_CSR_DPALT_STATE			 0x218
#define EDP_CSR_DP_PHY_TX_STATE			 0x21C
#define EDP_CSR_RSV_S0				 0x220
#define EDP_CSR_RSV_S1				 0x224
#define EDP_CSR_MON_STATUS00			 0x228
#define EDP_CSR_MON_STATUS01			 0x22C
#define EDP_CSR_MON_STATUS10			 0x230
#define EDP_CSR_MON_STATUS11			 0x234
#define EDP_CSR_MON_STATUS20			 0x238
#define EDP_CSR_MON_STATUS21			 0x23C
#define EDP_CSR_MON_STATUS30			 0x240
#define EDP_CSR_MON_STATUS31			 0x244
#define EDP_CSR_MON_STATUS40			 0x248
#define EDP_CSR_MON_STATUS41			 0x24C
#define EDP_CSR_MON_STATUS50			 0x250
#define EDP_CSR_MON_STATUS51			 0x254
#define EDP_CSR_MON_STATUS60			 0x258
#define EDP_CSR_MON_STATUS61			 0x25C
#define EDP_CSR_MON_STATUS70			 0x260
#define EDP_CSR_MON_STATUS71			 0x264
#define EDP_CSR_DIAG_OUT_STATUS0		 0x268
#define EDP_CSR_DIAG_OUT_STATUS1		 0x26C
#define EDP_CSR_DIAG_OUT_STATUS2		 0x270
#define EDP_CSR_DIAG_OUT_STATUS3		 0x274
#define EDP_CSR_DIAG_OUT_STATUS4		 0x278
#define EDP_CSR_DIAG_OUT_STATUS5		 0x27C
#define EDP_CSR_DIAG_OUT_STATUS6		 0x280
#define EDP_CSR_DIAG_OUT_STATUS7		 0x284
#define EDP_CSR_HDCP_DEBUG_STATUS0		 0x288
#define EDP_CSR_HDCP_DEBUG_STATUS1		 0x28C

#define EDP_CSR_VIDEO_MODE_DEFAULT 0x0
#define EDP_CSR_VIDEO_MODE_YUV422  0x1
#define EDP_CSR_VIDEO_MODE_YUV420  0x2
#define EDP_CSR_VIDEO_MODE_YONLY   0x4

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

#define EDP_CSR_DEN_POL_HIGH 0x1
#define EDP_CSR_DEN_POL_LOW  0x0

#define EDP_CSR_MK_VIDEO_CTRL(video_mode, ipi_fmt, den_pol, color_depth) \
	((video_mode) << 0 | (den_pol) << 4 | (ipi_fmt) << 8 |           \
	 (color_depth) << 16)

#define EDP_CSR_VIDEO_CTRL_CLR BIT(8)
#define EDP_CSR_KPF_REC_REQ_EN BIT(13)

#define EDP_CSR_PARITY_CHK_EN_RDATA	 (0x1)
#define EDP_CSR_PARITY_CHK_EN_PADDR	 (0x2)
#define EDP_CSR_PARITY_CHK_EN_PWDATA	 (0x4)
#define EDP_CSR_PARITY_CHK_EN_VIDEO_DATA (0x8)

#define EDP_CSR_PARITY_INJECT_AWADDR (0x1)
#define EDP_CSR_PARITY_INJECT_WDATA  (0x2)
#define EDP_CSR_PARITY_INJECT_ARADDR (0x4)
#define EDP_CSR_PARITY_INJECT_PRDATA (0x8)

#define EDP_CSR_PARITY_TYPE_ENC_ODD_DEC_ODD   (0x0)
#define EDP_CSR_PARITY_TYPE_ENC_EVEN_DEC_ODD  (0x1)
#define EDP_CSR_PARITY_TYPE_ENC_ODD_DEC_EVEN  (0x2)
#define EDP_CSR_PARITY_TYPE_ENC_EVEN_DEC_EVEN (0x3)

#define MK_EDP_CSR_PARITY_CTRL(func_int_mask, safety_int_mask, parity_mask,    \
			       parity_clean, parity_check_en, parity_inject,   \
			       parity_type)                                    \
	((func_int_mask) << 22 | (safety_int_mask) << 18 |                     \
	 (parity_mask) << 14 | (parity_clean) << 10 | (parity_check_en) << 6 | \
	 (parity_inject) << 2 | (parity_type))

void dptx_csr_reset(struct dptx *dptx)
{
	u32 ver_num = 0, ver_type = 0, dptx_id = 0;
	u32 crm_ctrl = 0;

	crm_ctrl = dptx_read_reg(dptx, dptx->regs[DPTX_CSR], EDP_CSR_CRM_CTRL);
	dptx_write_reg(dptx, dptx->regs[DPTX_CSR], EDP_CSR_CRM_CTRL, 0x102D32D);
	ver_num =
		dptx_read_reg(dptx, dptx->regs[DPTX_CSR], DPTX_VERSION_NUMBER);
	ver_type = dptx_read_reg(dptx, dptx->regs[DPTX_CSR], DPTX_VERSION_TYPE);
	dptx_id = dptx_read_reg(dptx, dptx->regs[DPTX_CSR], DPTX_ID);
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