// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2025 Black Sesame Technologies. All Rights Reserved.
 */

#include <dt-bindings/media/bst-isp.h>
#include <dt-bindings/media/bst-mdev.h>

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>

#include <bst/media-dev.h>

#include "adi_des.h"

#include "utils.h"

#define GAP_PIPE (0x40)
#define GAP_CSI	 (0x40)

/* clang-format off */
static const u32 mfp_ctrl_regs[MAX_MFPS] = {
	0x0300, 0x0303, 0x0306, 0x0309,
	0x030C, 0x0310, 0x0313, 0x0316,
	0x0319, 0x031C, 0x0320, 0x0323,
	0x0326, 0x0329, 0x032C, 0x0330,
	0x0333,
};

static const u32 mfp_tx_id_regs[MAX_RX_PORTS][MAX_MFPS] = {
	{
		0x0301, 0x0304, 0x0307, 0x030A,
		0x030D, 0x0311, 0x0314, 0x0317,
		0x031A, 0x031D, 0x0321, 0x0324,
		0x0327, 0x032A, 0x032D, 0x0331,
		0x0334,
	},
	{
		0x0337, 0x033A, 0x033D, 0x0341,
		0x0344, 0x0347, 0x034A, 0x034D,
		0x0351, 0x0354, 0x0357, 0x035A,
		0x035D, 0x0361, 0x0364, 0x0367,
		0x036A,
	},
	{
		0x036D, 0x0371, 0x0374, 0x0377,
		0x037A, 0x037D, 0x0381, 0x0384,
		0x0387, 0x038A, 0x038D, 0x0391,
		0x0394, 0x0397, 0x039A, 0x039D,
		0x03A1,
	},
	{
		0x03A4, 0x03A7, 0x03AA, 0x03AD,
		0x03B1, 0x03B4, 0x03B7, 0x03BA,
		0x03BD, 0x03C1, 0x03C4, 0x03C7,
		0x03CA, 0x03CD, 0x03D1, 0x03D4,
		0x03D7,
	},
};

/* -----------------------------------------------------------------------------
 * GMSL operations
 */
static bool is_gmsl1_link_locked(struct adi_des *des, int port)
{
	u32 val;
	int rv;

	switch (port) {
	case 0:
		rv = i2cgetwbc(des->i2c_client, 0x0BCB, &val);
		break;
	case 1:
		rv = i2cgetwbc(des->i2c_client, 0x0CCB, &val);
		break;
	case 2:
		rv = i2cgetwbc(des->i2c_client, 0x0DCB, &val);
		break;
	case 3:
		rv = i2cgetwbc(des->i2c_client, 0x0ECB, &val);
		break;
	default:
		dev_err(des->dev, "%s: invalid port: %d\n", __func__, port);
		return false;
	}

	if (rv)
		return false;

	if (val & BIT(0))
		return true;

	return false;
}

static bool is_gmsl2_link_locked(struct adi_des *des, int port)
{
	u32 val;
	int rv;

	switch (port) {
	case 0:
		rv = i2cgetwbc(des->i2c_client, 0x001A, &val);
		break;
	case 1:
		rv = i2cgetwbc(des->i2c_client, 0x000A, &val);
		break;
	case 2:
		rv = i2cgetwbc(des->i2c_client, 0x000B, &val);
		break;
	case 3:
		rv = i2cgetwbc(des->i2c_client, 0x000C, &val);
		break;
	default:
		dev_err(des->dev, "%s: invalid port: %d\n", __func__, port);
		return false;
	}

	if (rv)
		return false;

	if (val & BIT(3))
		return true;

	return false;
}

static bool is_link_locked(struct adi_des *des, int port)
{
	if (des->rx_ports[port].gmsl_ver == GMSL1)
		return is_gmsl1_link_locked(des, port);
	else if (des->rx_ports[port].gmsl_ver == GMSL2)
		return is_gmsl2_link_locked(des, port);
	else
		return false;
}

static bool is_gmsl2_video_locked(struct adi_des *des, int port)
{
	u32 val;
	int rv;

	switch (port) {
	case 0:
		rv = i2cgetwbc(des->i2c_client, 0x01DC, &val);
		break;
	case 1:
		rv = i2cgetwbc(des->i2c_client, 0x01FC, &val);
		break;
	case 2:
		rv = i2cgetwbc(des->i2c_client, 0x021C, &val);
		break;
	case 3:
		rv = i2cgetwbc(des->i2c_client, 0x023C, &val);
		break;
	default:
		dev_err(des->dev, "%s: invalid port: %d\n", __func__, port);
		return false;
	}

	if (rv)
		return false;

	if (val & BIT(0))
		return true;

	return false;
}

static bool is_video_locked(struct adi_des *des, int port)
{
	if (des->rx_ports[port].gmsl_ver == GMSL1)
		return is_gmsl1_link_locked(des, port);
	else if (des->rx_ports[port].gmsl_ver == GMSL2)
		return is_gmsl2_video_locked(des, port);
	else
		return false;
}

static int set_gmsl_link_rate(struct adi_des *des)
{
	u32 val;
	int i;

	val = 0;
	for (i = 0; i < des->param->num_gmsl; ++i) {
		struct rx_port *rxp;

		rxp = &des->rx_ports[i];
		if (!rxp->enable || !rxp->cam)
			continue;

		if (rxp->rx_rate == 3)
			val |= (0x01 << (i * 4));
		else if (rxp->rx_rate == 6)
			val |= (0x02 << (i * 4));
	}

	i2csetwbc(des->i2c_client, 0x0010, val & 0xFF);
	i2csetwbc(des->i2c_client, 0x0011, (val >> 8) & 0xFF);
	/* Reset one-shot for all links */
	i2csetwbc(des->i2c_client, 0x0018, 0x0F);
	/* NOTE: Links are setuped later, we skip delay here */

	return 0;
}

static int set_gmsl_link_en(struct adi_des *des)
{
	u32 val;
	int i;

	val = 0;
	for (i = 0; i < des->param->num_gmsl; ++i) {
		struct rx_port *rxp;

		rxp = &des->rx_ports[i];
		if (!rxp->enable || !rxp->cam)
			continue;

		val |= BIT(i);
		if (rxp->gmsl_ver == GMSL2)
			val |= BIT(i + 4);

		if (rxp->gmsl_ver == GMSL1 && rxp->him)
			i2csetwbc(des->i2c_client, (0x0B06 + i * 0x100), 0xEF);
	}

	i2csetwbc(des->i2c_client, 0x0006, val);
	/* NOTE: Since we only disable links, so no delay is need */

	return 0;
}

/* -----------------------------------------------------------------------------
 * Pipe, routing
 */
static int set_sw_override(struct adi_des *des, int pipe, int vc, int dt)
{
	u32 bpp;

	bpp = adi_des_dt_to_bpp(dt);

	switch (pipe) {
	case 0:
		/* VC */
		i2cupwbc(des->i2c_client, 0x040C, vc, 0xF, 0);
		/* DT */
		i2cupwbc(des->i2c_client, 0x040E, dt, 0x3F, 0);
		/* bpp */
		i2cupwbc(des->i2c_client, 0x040B, bpp, 0x1F, 3);
		/* Enable override bpp, vc, dt */
		i2cupwbc(des->i2c_client, 0x0415, 1, 0x1, 6);
		break;
	case 1:
		/* VC */
		i2cupwbc(des->i2c_client, 0x040C, vc, 0xF, 4);
		/* DT */
		i2cupwbc(des->i2c_client, 0x040E, dt >> 4, 0x3, 6);
		i2cupwbc(des->i2c_client, 0x040F, dt, 0xF, 0);
		/* bpp */
		i2cupwbc(des->i2c_client, 0x0411, bpp, 0x1F, 0);
		/* Enable override bpp, vc, dt */
		i2cupwbc(des->i2c_client, 0x0415, 1, 0x1, 7);
		break;
	case 2:
		/* VC */
		i2cupwbc(des->i2c_client, 0x040D, vc, 0xF, 0);
		/* DT */
		i2cupwbc(des->i2c_client, 0x040F, dt >> 2, 0xF, 4);
		i2cupwbc(des->i2c_client, 0x0410, dt, 0x3, 0);
		/* bpp */
		i2cupwbc(des->i2c_client, 0x0411, bpp >> 2, 0x7, 5);
		i2cupwbc(des->i2c_client, 0x0412, bpp, 0x3, 0);
		/* Enable override bpp, vc, dt */
		i2cupwbc(des->i2c_client, 0x0418, 1, 0x1, 6);
		break;
	case 3:
		/* VC */
		i2cupwbc(des->i2c_client, 0x040D, vc, 0xF, 4);
		/* DT */
		i2cupwbc(des->i2c_client, 0x0410, dt, 0x3F, 2);
		/* bpp */
		i2cupwbc(des->i2c_client, 0x0412, bpp, 0x1F, 2);
		/* Enable override bpp, vc, dt */
		i2cupwbc(des->i2c_client, 0x0418, 1, 0x1, 7);
		break;
	case 4:
		/* VC */
		i2cupwbc(des->i2c_client, 0x042C, vc, 0xF, 0);
		/* DT */
		i2cupwbc(des->i2c_client, 0x042E, dt, 0x3F, 0);
		/* bpp */
		i2cupwbc(des->i2c_client, 0x042B, bpp, 0x1F, 3);
		/* Enable override bpp, vc, dt */
		i2cupwbc(des->i2c_client, 0x041B, 1, 0x1, 6);
		break;
	case 5:
		/* VC */
		i2cupwbc(des->i2c_client, 0x042C, vc, 0xF, 4);
		/* DT */
		i2cupwbc(des->i2c_client, 0x042E, dt >> 4, 0x3, 6);
		i2cupwbc(des->i2c_client, 0x042F, dt, 0xF, 0);
		/* bpp */
		i2cupwbc(des->i2c_client, 0x0431, bpp, 0x1F, 0);
		/* Enable override bpp, vc, dt */
		i2cupwbc(des->i2c_client, 0x041B, 1, 0x1, 7);
		break;
	case 6:
		/* VC */
		i2cupwbc(des->i2c_client, 0x042D, vc, 0xF, 0);
		/* DT */
		i2cupwbc(des->i2c_client, 0x042F, dt >> 2, 0xF, 4);
		i2cupwbc(des->i2c_client, 0x0430, dt, 0x3, 0);
		/* bpp */
		i2cupwbc(des->i2c_client, 0x0431, bpp >> 2, 0x7, 5);
		i2cupwbc(des->i2c_client, 0x0432, bpp, 0x3, 0);
		/* Enable override bpp, vc, dt */
		i2cupwbc(des->i2c_client, 0x041D, 1, 0x1, 6);
		break;
	case 7:
		/* VC */
		i2cupwbc(des->i2c_client, 0x042D, vc, 0xF, 4);
		/* DT */
		i2cupwbc(des->i2c_client, 0x0430, dt, 0x3F, 2);
		/* bpp */
		i2cupwbc(des->i2c_client, 0x0432, bpp, 0x1F, 2);
		/* Enable override bpp, vc, dt */
		i2cupwbc(des->i2c_client, 0x041D, 1, 0x1, 7);
		break;
	default:
		dev_err(des->dev, "Unsupported pipe: %d\n", pipe);
		return -EINVAL;
	}

	return 0;
}

static int set_pipe(struct adi_des *des)
{
	int i;
	u32 pipe_en;
	u32 pipe_sel;

	pipe_en = 0;
	pipe_sel = 0;
	for (i = 0; i < des->param->num_pipe; ++i) {
		struct pipe *pipe;
		struct rx_port *rxp;
		struct camera_dev *cam;
		u32 src;
		u32 dt;
		u32 off;
		u32 csi;

		pipe = &des->pipes[i];
		src = pipe->from_port;
		rxp = &des->rx_ports[src];
		cam = rxp->cam;
		if (!pipe->enable)
			continue;
		if (cam == NULL)
			continue;

		pipe_en |= BIT(i);
		pipe_sel |= ((pipe->from_port << 2) | (pipe->from_sid))
			    << (i * 4);
		off = GAP_PIPE * i;
		dt = cam->data_type;
		csi = pipe->to_csi;

		i2csetwbc(des->i2c_client, 0x090D + off,
			  MK_VC_MAP(pipe->from_vc, DT_FRAME_START));
		i2csetwbc(des->i2c_client, 0x090E + off,
			  MK_VC_MAP(pipe->to_vc, DT_FRAME_START));
		i2csetwbc(des->i2c_client, 0x090F + off,
			  MK_VC_MAP(pipe->from_vc, DT_FRAME_END));
		i2csetwbc(des->i2c_client, 0x0910 + off,
			  MK_VC_MAP(pipe->to_vc, DT_FRAME_END));
		i2csetwbc(des->i2c_client, 0x0911 + off,
			  MK_VC_MAP(pipe->from_vc, DT_LINE_START));
		i2csetwbc(des->i2c_client, 0x0912 + off,
			  MK_VC_MAP(pipe->to_vc, DT_LINE_START));
		i2csetwbc(des->i2c_client, 0x0913 + off,
			  MK_VC_MAP(pipe->from_vc, DT_LINE_END));
		i2csetwbc(des->i2c_client, 0x0914 + off,
			  MK_VC_MAP(pipe->to_vc, DT_LINE_END));
		i2csetwbc(des->i2c_client, 0x0915 + off,
			  MK_VC_MAP(pipe->from_vc, dt));
		i2csetwbc(des->i2c_client, 0x0916 + off,
			  MK_VC_MAP(pipe->to_vc, dt));

		/* Map 5 groups to CSI */
		i2csetwbc(des->i2c_client, 0x090B + off, 0x1F);
		i2csetwbc(des->i2c_client, 0x092D + off,
			  (csi << 6) | (csi << 4) | (csi << 2) | csi);
		i2csetwbc(des->i2c_client, 0x092E + off, csi);
		if (rxp->gmsl_ver == GMSL1) {
			set_sw_override(des, i, 0, dt);
			if (cam->data_type == DT_YUV422_8B ||
			    cam->data_type == DT_YUV422_10B)
				// Enable YUV422 8-bit and 10-bit mux
				i2cupwbc(des->i2c_client, 0x41A + i / 4 * 0x20,
					 1, 0x1, 4 + i % 4);
		}
	}

	i2csetwbc(des->i2c_client, 0x00F0, (pipe_sel & 0xFF));
	i2csetwbc(des->i2c_client, 0x00F1, ((pipe_sel >> 8) & 0xFF));

	if (des->param->des_type == DES_MAX96712) {
		i2csetwbc(des->i2c_client, 0x00F2, ((pipe_sel >> 16) & 0xFF));
		i2csetwbc(des->i2c_client, 0x00F3, ((pipe_sel >> 24) & 0xFF));
		i2csetwbc(des->i2c_client, 0x00F4, pipe_en);
	} else {
		// Legacy MAX96712 mode, turn on Pipe 1,2,3,4
		i2csetwbc(des->i2c_client, 0x00F4, pipe_en);
	}

	return 0;
}

/* -----------------------------------------------------------------------------
 * CSI operations
 */
static int set_csi_mode(struct adi_des *des)
{
	u32 val;

	switch (des->csi_mode) {
	case CSI_MODE_4X2:
		val = 0x01;
		break;
	case CSI_MODE_2X4:
		val = 0x04;
		break;
	case CSI_MODE_1X4A_2X2:
		val = 0x08;
		break;
	case CSI_MODE_1X4B_2X2:
		val = 0x10;
		break;
	default:
		dev_err(des->dev, "Wrong PHY mode: %u\n", des->csi_mode);
		return -EINVAL;
	}

	i2csetwbc(des->i2c_client, 0x08A0, val);

	return 0;
}

static int set_csi_phy(struct adi_des *des)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(des->tx_ports); ++i) {
		u32 val;
		u32 off;
		struct csi_tx_dev *txp;

		txp = &des->tx_ports[i];
		if (!txp->enable)
			continue;

		off = (i - des->param->csi_lo) * 0x100;
		/* Hold DPLL in reset */
		if (des->param->des_type == DES_MAX96712)
			i2csetwbc(des->i2c_client, 0x1C00 + off, 0xF4);

		off = (i - des->param->csi_lo) * 0x3;
		val = BIT(5) | (txp->lane_speed / 100);
		i2cupwbc(des->i2c_client, 0x0415 + off, val, 0x3F, 0);

		off = (i - des->param->csi_lo) * GAP_CSI;
		val = (txp->lane_num - 1) << 6;
		if (txp->phy_if == IF_CPHY)
			val |= BIT(5);
		i2cupwbc(des->i2c_client, 0x90A + off, val, 0xE0, 0);

		if (txp->phy_if == IF_DPHY) {
			/* NOTE: RX spec:
			 * Initial: 2^15 UI ~ 100us
			 * Periodic: Disabled
			 */
			if (txp->lane_speed > 1500) {
				i2csetwbc(des->i2c_client, 0x0903 + off, 0x82);
				i2csetwbc(des->i2c_client, 0x0904 + off, 0x00);
			} else {
				i2csetwbc(des->i2c_client, 0x0903 + off, 0x00);
				i2csetwbc(des->i2c_client, 0x0904 + off, 0x00);
			}
		}

		off = (i - des->param->csi_lo) * 0x100;
		/* Release DPLL reset */
		if (des->param->des_type == DES_MAX96712)
			i2csetwbc(des->i2c_client, 0x1C00 + off, 0xF5);
	}

	return 0;
}

static int set_csi_copy(struct adi_des *des)
{
	int i;
	u32 val;

	for (i = 0; i < ARRAY_SIZE(des->phy_cps); ++i) {
		if (des->phy_cps[i].src == des->phy_cps[i].dst)
			break;

		val = BIT(7) | (des->phy_cps[i].src << 3) | (des->phy_cps[i].dst << 5);
		i2csetwbc(des->i2c_client, 0x8A9 + i, val);
	}

	return 0;
}

static int csi_pre_streamon(struct adi_des *des)
{
	int rv;
	u32 val;

	rv = i2cgetwbc(des->i2c_client, 0x040B, &val);
	if (rv) {
		dev_err(des->dev, "Failed to get 0x040B\n");
		return rv;
	}
	i2csetwbc(des->i2c_client, 0x040B, (val & (~BIT(1))));

	rv = i2cgetwbc(des->i2c_client, 0x08A0, &val);
	if (rv) {
		dev_err(des->dev, "%s: Failed to get 0x08A0\n", __func__);
		return rv;
	}

	return i2csetwbc(des->i2c_client, 0x08A0, (val & (~BIT(7))));
}

static int csi_stream(struct adi_des *des, bool enable)
{
	int rv;
	u32 val;

	rv = i2cgetwbc(des->i2c_client, 0x08A0, &val);
	if (rv) {
		dev_err(des->dev, "%s: Failed to get 0x08A0\n", __func__);
		return rv;
	}

	if (STREAM_DEC_EN(enable)) {
		if (val & BIT(7))
			return 0;
		i2csetwbc(des->i2c_client, 0x08A0, (val | BIT(7)));
		dev_info(des->dev, "Stream 0x%02X\n", enable);
	}
	/* NOTE: never stream off for CSI RX limit */

	return 0;
}

/* NOTE: I2C port 2 is not supported for simplicity */
static int set_remote_i2c_disable(struct adi_des *des)
{
	i2csetwbc(des->i2c_client, 0x0003, 0xFF);

	return 0;
}

static int set_remote_i2c_enable(struct adi_des *des)
{
	if (des->i2c_port == 0) {
		i2csetwbc(des->i2c_client, 0x0003, 0xAA);
		i2csetwbc(des->i2c_client, 0x0007, 0x00);
	} else if (des->i2c_port == 1) {
		i2csetwbc(des->i2c_client, 0x0003, 0x55);
		i2csetwbc(des->i2c_client, 0x0007, 0xF0);
	} else {
		dev_err(des->dev, "I2C Port %d is not supported\n",
			des->i2c_port);
		return -EINVAL;
	}

	return 0;
}

static int set_remote_i2c_enable_by_port(struct adi_des *des, int port)
{
	switch (port) {
	case 0:
		return i2csetwbc(des->i2c_client, 0x0003, 0xFC);
	case 1:
		return i2csetwbc(des->i2c_client, 0x0003, 0xF3);
	case 2:
		return i2csetwbc(des->i2c_client, 0x0003, 0xCF);
	case 3:
		return i2csetwbc(des->i2c_client, 0x0003, 0x3F);
	default:
		dev_err(des->dev, "Port %d is not supported\n", port);
		return -EINVAL;
	}
}

static int set_fsync_inner(struct adi_des *des)
{
	u32 period;
	u32 val;
	bool has_gmsl1;
	int i;
	u32 tx_id;
	u32 gpi_pin;

	period = des->fsync_fps;
	val = 0x5F;
	has_gmsl1 = false;
	tx_id = 0;
	for (i = 0; i < des->param->num_gmsl; ++i) {
		struct rx_port *rxp;

		rxp = &des->rx_ports[i];
		if (!rxp->enable || !rxp->cam)
			continue;

		/* Use last ser-fsync-tx-pin as TX ID */
		tx_id = rxp->cam->ser_fsync_tx_pin;

		if (rxp->gmsl_ver == GMSL1) {
			has_gmsl1 = true;
			gpi_pin = 0x31 | des->fsync_rx_pin << 6;
			i2csetwbc(des->i2c_client, 0x0B08 + i * 0x100, gpi_pin);
		}
	}
	if (!has_gmsl1)
		val |= BIT(7);

	i2csetwbc(des->i2c_client, 0x04AF, val);
	period = SCLK / des->fsync_fps;
	i2csetwbc(des->i2c_client, 0x04A7, (period >> 16) & 0xFF);
	i2csetwbc(des->i2c_client, 0x04A6, (period >> 8) & 0xFF);
	i2csetwbc(des->i2c_client, 0x04A5, period & 0xFF);
	i2csetwbc(des->i2c_client, 0x04B1, (tx_id << 3));

	/* Set internal FSYNC manual mode */
	i2csetwbc(des->i2c_client, 0x04A0, 0x04);

	return 0;
}

static int set_fsync_outer(struct adi_des *des)
{
	u32 val;
	int i;
	u32 rx_pin;
	u32 tx_id;
	bool has_gmsl1;

	/* Set internal FSYNC off */
	i2csetwbc(des->i2c_client, 0x04A0, 0x08);

	rx_pin = des->fsync_rx_pin;
	val = 0x1F;
	has_gmsl1 = false;
	/* Use rx-pin as TX ID */
	tx_id = 0x20 | des->fsync_rx_pin;
	for (i = 0; i < des->param->num_gmsl; ++i) {
		struct rx_port *rxp;

		rxp = &des->rx_ports[i];
		if (!rxp->enable || !rxp->cam)
			continue;

		if (rxp->gmsl_ver == GMSL1) {
			has_gmsl1 = true;
			i2csetwbc(des->i2c_client, 0x0B08 + i * 0x100,
				  0x21 | (rx_pin << 6));
		} else {
			i2csetwbc(des->i2c_client, mfp_tx_id_regs[i][rx_pin],
				  tx_id);
		}
	}
	if (!has_gmsl1)
		val |= BIT(7);
	dev_dbg(des->dev, "has_gmsl1: %d, val: 0x%08X\n", has_gmsl1, val);
	i2csetwbc(des->i2c_client, 0x04AF, val);

	if (has_gmsl1)
		return 0;

	/* Disable default RX cfg for Max96712's MFP0 */
	if (rx_pin != 0)
		i2csetwbc(des->i2c_client, mfp_ctrl_regs[0],
			  FSYNC_OUTER_DES_RX_DISABLE);
	i2csetwbc(des->i2c_client, mfp_ctrl_regs[rx_pin],
		  FSYNC_OUTER_DES_RX_CFG);

	return 0;
}

static int set_fsync(struct adi_des *des)
{
	switch (des->fsync_mode) {
	case FSYNC_OFF:
		return 0;
	case FSYNC_INNER:
		return set_fsync_inner(des);
	case FSYNC_OUTER:
		return set_fsync_outer(des);
	default:
		dev_err(des->dev, "Unsupported fsync mode: %u\n",
			des->fsync_mode);
	}

	return -EINVAL;
}

static int des_setup(struct adi_des *des)
{
	set_remote_i2c_disable(des);
	csi_pre_streamon(des);
	adi_des_set_pre_gmsl(des, i2csetwb);
	set_gmsl_link_en(des);
	set_gmsl_link_rate(des);
	set_pipe(des);
	adi_des_set_post_gmsl(des, i2csetwb);

	set_fsync(des);

	adi_des_set_pre_csi(des, i2csetwb);
	set_csi_mode(des);
	set_csi_phy(des);
	set_csi_copy(des);
	adi_des_set_post_csi(des, i2csetwb);
	if (des->role == ROLE_MASTER)
		set_remote_i2c_enable(des);

	dev_info(des->dev, "Setup done\n");

	return 0;
}

static bool is_des_setuped(struct adi_des *des)
{
	int rv;
	u32 val;

	rv = i2cgetwbc(des->i2c_client, 0x040B, &val);
	dev_dbg(des->dev, "%s: rv: %d, val: 0x%02X\n", __func__, rv, val);
	if (!rv && ((val & BIT(1)) == 0))
		return true;

	return false;
}

/* -----------------------------------------------------------------------------
 * Link setup
 */
static int gmsl_setup(struct adi_des *des, int port)
{
	int rv;
	u32 dis_rem_cc;
	u32 off;
	struct device *dev;
	struct rx_port *rxp;
	struct camera_dev *cam;

	rxp = &des->rx_ports[port];
	cam = rxp->cam;
	if (cam == NULL)
		return 0;

	off = port * 0x100;
	dev = des->dev;
	dev_info(dev, "Setup port %u\n", port);
	/* Save current remote channel control config */
	rv = i2cgetwbc(des->i2c_client, 0x0003, &dis_rem_cc);
	if (rv) {
		dev_err(dev, "Failed to get dis_rem_cc, port: %d, rv: %d\n",
			port, rv);
		return rv;
	}
	rv = set_remote_i2c_enable_by_port(des, port);
	if (rv) {
		dev_err(dev, "Failed to set dis_rem_cc, port: %d, rv: %d\n",
			port, rv);
		goto exit;
	}

	/*  When the serializer is not power off during reboot,
	 *  it keep old alias address and settings.
	 *  If the serializer is initialized form power-off state,
	 *  this action does not take effect.
	 */
	if (cam->ser_reset && cam->ser_type) {
		dev_info(des->dev, "Reset serializer of port %d\n", port);
		adi_ser_reset(des, port);
	}

	if (rxp->gmsl_ver == GMSL1) {
		i2csetwbc(des->i2c_client, 0x0006, BIT(port));
		/* Turn on AutoAck */
		i2csetwbc(des->i2c_client, (0x0B0D + off), 0x80);
		/* Turn on configuration link, turn off video link */
		cam->ser_i2cset(des->i2c_adap, cam->ser_addr, 0x0004, 0x43);
		ursleep(GMSL1_CLINK_LOCK_TIME);
	}

	adi_des_set_pre_ser(des, rxp->node, i2csetwb, rxp->cfg_with_delay);
	rv = adi_ser_set_alias(des, port);
	if (rv) {
		dev_err_ratelimited(
			dev, "Failed to set ser alias, port: %d, rv: %d\n",
			port, rv);
		goto exit;
	}
	/* NOTE: MUST restore remote channel control setting ASAP,
	 * so other ports can be accessed.
	 */
	i2csetwbc(des->i2c_client, 0x0003, dis_rem_cc);
	if (rxp->gmsl_ver == GMSL1)
		i2csetwbc(des->i2c_client, (0x0B0D + off), 0x00); // Off AutoAck

	adi_ser_set_i2c_map(des, port);
	rv = v4l2_subdev_call(&cam->tx_dev.subdev, core, s_power, 1);
	if (rv)
		goto exit;

	if (cam->role == ROLE_MASTER)
		adi_ser_set_fsync(des, port);
	adi_des_set_post_ser(des, rxp->node, i2csetwb, rxp->cfg_with_delay);

exit:
	/* Restore current remote channel control setting */
	i2csetwbc(des->i2c_client, 0x0003, dis_rem_cc);
	if (rxp->gmsl_ver == GMSL1) {
		/* Turn off AutoAck */
		i2csetwbc(des->i2c_client, (0x0B0D + off), 0x00);
		/* Turn on video link */
		cam->ser_i2cset(des->i2c_adap, cam->ser_alias, 0x0004, 0x83);
		i2csetwbc(des->i2c_client, 0x0006, des->link_en_map);
		ursleep(des->param->t_lock);
	}

	return rv;
}

static const struct des_ops max96724_ops = {
	.is_link_locked = is_link_locked,
	.is_video_locked = is_video_locked,
	.csi_pre_streamon = csi_pre_streamon,
	.csi_stream = csi_stream,
	.des_setup = des_setup,
	.is_des_setuped = is_des_setuped,
	.gmsl_setup = gmsl_setup,
};

static int max96724_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	int rv;
	struct adi_des *des;
	struct device *dev;
	const struct des_param *param;

	dev = &client->dev;
	des = devm_kzalloc(dev, sizeof(*des), GFP_KERNEL);
	if (!des)
		return -ENOMEM;

	param = of_device_get_match_data(dev);
	if (!param) {
		dev_err(dev, "No matched hardware params\n");
		return -EINVAL;
	}
	des->param = param;
	des->ops = &max96724_ops;
	des->dev = dev;
	des->i2c_client = client;
	des->i2c_adap = client->adapter;
	i2c_set_clientdata(client, des);
	mutex_init(&des->lock);

	rv = adi_des_parse_dt(des);
	if (rv)
		goto err_parse_dt;

	/* Power up */
	rv = adi_des_power_up(des);
	if (rv)
		goto err_power_up;

	ursleep(des->param->t_i2c_wake);
	/* Detect device */
	if (des->role == ROLE_AUTO) {
		rv = i2cprobec(des->i2c_client, i2cgetwb);
		if (rv) {
			dev_warn(
				dev,
				"Can not detect deser, switch to slave mode, rv: %d\n",
				rv);
			des->role = ROLE_SLAVE;
		} else {
			des->role = ROLE_MASTER;
		}
	}

	rv = adi_des_init_v4l2_dev(des);
	if (rv) {
		dev_err(dev, "Failed to init v4l2 dev\n");
		goto err_init_v4l2_dev;
	}
	adi_des_sysfs_init(des);

	dev_info(dev, "Probe done on CPU %u, role: %s\n", smp_processor_id(),
		 str_role(des->role));

	return 0;

err_init_v4l2_dev:
err_power_up:
err_parse_dt:
	return rv;
}

static void max96724_remove(struct i2c_client *client)
{
	struct adi_des *des = i2c_get_clientdata(client);

	dev_info(des->dev, "Remove\n");
	adi_des_sysfs_exit(des);
}

static void max96724_shutdown(struct i2c_client *client)
{
	struct adi_des *des = i2c_get_clientdata(client);

	dev_info(des->dev, "Shutdown\n");
	if (des->role != ROLE_MASTER)
		return;

	adi_des_exit_lock_handler(des);
}

static int max96724_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct adi_des *des = i2c_get_clientdata(client);

	dev_info(des->dev, "Suspend\n");

	return 0;
}

static int max96724_resume(struct device *dev)
{
	int rv;
	struct i2c_client *client = to_i2c_client(dev);
	struct adi_des *des = i2c_get_clientdata(client);

	dev_info(des->dev, "Resume\n");
	if (des->role == ROLE_MASTER) {
		des->resume = 1;
		if (!is_des_setuped(des)) {
			adi_des_power_up(des);
			ursleep(des->param->t_i2c_wake);
			rv = i2cprobec(des->i2c_client, i2cgetwb);
			if (rv)
				dev_err(dev, "Can not detect deser, rv: %d\n", rv);
			dev_info(des->dev, "Re-Setup des\n");
			des_setup(des);
			ursleep(des->param->t_lock);
		}
		adi_des_setup_links(des);
		des->resume = 0;
	}

	return 0;
}

static const struct dev_pm_ops max96724_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(max96724_suspend, max96724_resume)
};

// clang-format off
static const struct reg_cfg max96712_pre_gmsl[] = {
	/* Enable regulator */
	{ 0x0017, 0x14 },
	{ 0x0019, 0x10 },
	/* Increase CMU voltage to improve robust for following case:
	 * GMSL lock can be lost due to high jitter in a narrow
	 * temperature window.
	 */
	{ 0x06C2, 0x10 },
	/* VgaHiGain_Init_6G & 3G for each PHY */
	{ 0x14D1, 0x03 },
	{ 0x15D1, 0x03 },
	{ 0x16D1, 0x03 },
	{ 0x17D1, 0x03 },
	/* Disable heartbeat, will use 100us to detect loss of video lock */
	{ 0x0106, 0x0A }, // Link A
	{ 0x0118, 0x0A }, // Link B
	{ 0x012A, 0x0A }, // Link C
	{ 0x013C, 0x0A }, // Link D
	/* Optimized reference over reverse channel settings */
	{ 0x148C, 0x20 },
	{ 0x1498, 0xC0 },
	{ 0x158C, 0x20 },
	{ 0x1598, 0xC0 },
	{ 0x168C, 0x20 },
	{ 0x1698, 0xC0 },
	{ 0x178C, 0x20 },
	{ 0x1798, 0xC0 },
};

static const struct reg_cfg max96712_post_gmsl[] = {};
static const struct reg_cfg max96712_pre_csi[] = {};
static const struct reg_cfg max96712_post_csi[] = {};

static const struct reg_cfg max96722_pre_gmsl[] = {};
static const struct reg_cfg max96722_post_gmsl[] = {};
static const struct reg_cfg max96722_pre_csi[] = {};
static const struct reg_cfg max96722_post_csi[] = {};

static const struct reg_cfg max96724_pre_gmsl[] = {
	/* Disable heartbeat, will use 100us to detect loss of video lock */
	{ 0x0106, 0x0A }, // Link A
	{ 0x0118, 0x0A }, // Link B
	{ 0x012A, 0x0A }, // Link C
	{ 0x013C, 0x0A }, // Link D
	/* Error channel is powered on continuously for 6Gbps's robust */
	{ 0x1449, 0x75 }, // Link A
	{ 0x1549, 0x75 }, // Link B
	{ 0x1649, 0x75 }, // Link C
	{ 0x1749, 0x75 }, // Link D
};
static const struct reg_cfg max96724_post_gmsl[] = {};
static const struct reg_cfg max96724_pre_csi[] = {};
static const struct reg_cfg max96724_post_csi[] = {};
// clang-format on

static const struct des_param max96712_params = {
	.des_type = DES_MAX96712,
	.num_gmsl = 4,
	.num_pipe = 8,
	.num_csi = 4,
	.num_i2c = 2, // I2C 2 is not supported
	.num_mfp = 17,
	.t_lock = 100000, // NOTE: XP, should be tLock2, but also see ERRATA
	.t_i2c_wake = 1100,
	.gmsl_ver_lo = GMSL1,
	.gmsl_ver_up = GMSL2,
	.csi_lo = CSI0,
	.csi_up = CSI3,
	.pre_gmsl = __REG_CFGS(max96712_pre_gmsl),
	.post_gmsl = __REG_CFGS(max96712_post_gmsl),
	.pre_csi = __REG_CFGS(max96712_pre_csi),
	.post_csi = __REG_CFGS(max96712_post_csi),
};

static const struct des_param max96722_params = {
	.des_type = DES_MAX96722,
	.num_gmsl = 4,
	.num_pipe = 8,
	.num_csi = 4,
	.num_i2c = 2, // I2C 2 is not supported
	.num_mfp = 17,
	.t_lock = 100000, // NOTE: XP, should be tLock2, but also see ERRATA
	.t_i2c_wake = 2250,
	.gmsl_ver_lo = GMSL1,
	.gmsl_ver_up = GMSL2,
	.csi_lo = CSI0,
	.csi_up = CSI3,
	.pre_gmsl = __REG_CFGS(max96722_pre_gmsl),
	.post_gmsl = __REG_CFGS(max96722_post_gmsl),
	.pre_csi = __REG_CFGS(max96722_pre_csi),
	.post_csi = __REG_CFGS(max96722_post_csi),
};

static const struct des_param max96724_params = {
	.des_type = DES_MAX96724,
	.num_gmsl = 4,
	.num_pipe = 4,
	.num_csi = 4,
	.num_i2c = 2,
	.num_mfp = 9,
	.t_lock = 100000, // NOTE: XP, should be tLock2, but also see ERRATA
	.t_i2c_wake = 2250,
	.gmsl_ver_lo = GMSL1,
	.gmsl_ver_up = GMSL2,
	.csi_lo = CSI0,
	.csi_up = CSI3,
	.pre_gmsl = __REG_CFGS(max96724_pre_gmsl),
	.post_gmsl = __REG_CFGS(max96724_post_gmsl),
	.pre_csi = __REG_CFGS(max96724_pre_csi),
	.post_csi = __REG_CFGS(max96724_post_csi),
};

static const struct of_device_id max96724_of_ids[] = {
	// clang-format off
	{ .compatible = "bst,max96712", .data = &max96712_params, },
	{ .compatible = "bst,max96722", .data = &max96722_params, },
	{ .compatible = "bst,max96724", .data = &max96724_params, },
	{},
	// clang-format on
};
MODULE_DEVICE_TABLE(of, max96724_of_ids);

static struct i2c_driver max96724_driver = {
	.driver = {
		.name = "bst,max96724",
		.of_match_table = of_match_ptr(max96724_of_ids),
		.pm = &max96724_pm_ops,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = max96724_probe,
	.remove = max96724_remove,
	.shutdown = max96724_shutdown,
};
module_i2c_driver(max96724_driver);

MODULE_DESCRIPTION("BST Max96724 driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("BST Ltd.");
