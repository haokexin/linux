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

#define GAP_PIPE (0x18)
#define GAP_CSI	 (0x40)

/* -----------------------------------------------------------------------------
 * GMSL operations
 */
static bool is_link_locked(struct adi_des *des, int port)
{
	int rv;
	u32 val;

	rv = i2cgetwbc(des->i2c_client, 0x0008, &val);
	if (rv)
		return false;

	if (val & BIT(port))
		return true;

	return false;
}

static bool is_video_locked(struct adi_des *des, int port)
{
	u32 val;
	int rv;

	switch (port) {
	case 0:
		rv = i2cgetwbc(des->i2c_client, 0x0108, &val);
		break;
	case 1:
		rv = i2cgetwbc(des->i2c_client, 0x0124, &val);
		break;
	case 2:
		rv = i2cgetwbc(des->i2c_client, 0x0140, &val);
		break;
	case 3:
		rv = i2cgetwbc(des->i2c_client, 0x015C, &val);
		break;
	default:
		dev_err(des->dev, "%s: invalid port: %d\n", __func__, port);
		return false;
	}

	if (rv)
		return false;

	if (val & BIT(6))
		return true;

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
			val |= (0x01 << (i * 2));
		else if (rxp->rx_rate == 6)
			val |= (0x02 << (i * 2));
		else if (rxp->rx_rate == 12)
			val |= (0x03 << (i * 2));
	}

	i2csetwbc(des->i2c_client, 0x0010, val & 0xFF);

	/* Reset one-shot for all links */
	i2csetwbc(des->i2c_client, 0x001E, 0x0F);
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
		if (rxp->gmsl_ver == GMSL3)
			val |= BIT(i + 4);
	}

	i2csetwbc(des->i2c_client, 0x0007, val);
	/* NOTE: Since we only disable links, so no delay is need */

	return 0;
}

/* -----------------------------------------------------------------------------
 * Pipe, routing
 */
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
		csi = pipe->to_csi;

		/* NOTE: Only support SER is DPHY now */
		i2csetwbc(des->i2c_client, 0x0420 + off, csi);
		/* Use VC_REMAP_X to remap VC */
		i2csetwbc(des->i2c_client, 0x0426 + off, 0x20);
		/* NOTE: Only support 1 VC now */
		i2csetwbc(des->i2c_client, 0x0427 + off, pipe->from_vc);
		i2csetwbc(des->i2c_client, 0x0428 + off, pipe->to_vc);
	}

	i2csetwbc(des->i2c_client, 0x00F0, (pipe_sel & 0xFF));
	i2csetwbc(des->i2c_client, 0x00F1, ((pipe_sel >> 8) & 0xFF));
	i2csetwbc(des->i2c_client, 0x00F2, ((pipe_sel >> 16) & 0xFF));
	i2csetwbc(des->i2c_client, 0x00F3, ((pipe_sel >> 24) & 0xFF));
	i2csetwbc(des->i2c_client, 0x00F4, pipe_en);
	i2csetwbc(des->i2c_client, 0x00F5, 0x0);

	return 0;
}

/* -----------------------------------------------------------------------------
 * CSI operations
 */
static int set_csi_mode(struct adi_des *des)
{
	dev_dbg(des->dev, "Only support 2x4 mode\n");
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
		i2csetwbc(des->i2c_client, 0x1D00 + off, 0xF4);

		off = (i - des->param->csi_lo) * 0x3;
		val = 0x80 | (txp->lane_speed / 100);
		i2csetwbc(des->i2c_client, 0x098F + off, val);

		off = (i - des->param->csi_lo) * GAP_CSI;
		val = (txp->lane_num - 1) << 6;
		if (txp->phy_if == IF_CPHY)
			val |= BIT(5);
		i2csetwbc(des->i2c_client, 0x0A06 + off, val);

		if (txp->phy_if == IF_DPHY) {
			/* NOTE: RX spec:
			 * Initial: 2^15 UI ~ 100us
			 * Periodic: Disabled
			 */
			if (txp->lane_speed > 1500) {
				i2csetwbc(des->i2c_client, 0x0A02 + off, 0x82);
				i2csetwbc(des->i2c_client, 0x0A03 + off, 0x00);
			} else {
				i2csetwbc(des->i2c_client, 0x0A02 + off, 0x00);
				i2csetwbc(des->i2c_client, 0x0A03 + off, 0x00);
			}
		}

		off = (i - des->param->csi_lo) * 0x100;
		/* Release DPLL reset */
		i2csetwbc(des->i2c_client, 0x1D00 + off, 0xF5);
	}

	return 0;
}

static int set_csi_timing(struct adi_des *des)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(des->tx_ports); ++i) {
		if (des->tx_ports[i].phy_if != IF_CPHY)
			continue;

		// t3 post config to avoid deskew error
		i2csetwbc(des->i2c_client, 0x08B7, 0x7D);
		if (des->tx_ports[i].lane_speed == 3500) {
			i2csetwbc(des->i2c_client, 0x08B5, 0x3F);
			i2csetwbc(des->i2c_client, 0x08F0, 0x01);
		}
	}

	return 0;
}

static int set_csi_copy(struct adi_des *des)
{
	int rv;
	int i;
	u32 val;

	for (i = 0; i < ARRAY_SIZE(des->phy_cps); ++i) {
		if (des->phy_cps[i].src == des->phy_cps[i].dst)
			break;

		rv = i2cgetwbc(des->i2c_client, 0x08B9, &val);
		if (rv) {
			dev_err(des->dev, "Failed to get 0x08B9\n");
			return rv;
		}
		val |= BIT(7) | BIT(2);
		if (des->tx_ports[des->phy_cps[i].src].lane_speed < 500)
			val |= BIT(0);
		i2csetwbc(des->i2c_client, 0x08B9 + i, val);
	}

	return 0;
}

static int csi_pre_streamon(struct adi_des *des)
{
	int rv;
	u32 val;

	rv = i2cgetwbc(des->i2c_client, 0x0404, &val);
	if (rv) {
		dev_err(des->dev, "Failed to get 0x0404\n");
		return rv;
	}
	i2csetwbc(des->i2c_client, 0x0404, (val & (~BIT(3))));

	rv = i2cgetwbc(des->i2c_client, 0x08B0, &val);
	if (rv) {
		dev_err(des->dev, "%s: Failed to get 0x08B0\n", __func__);
		return rv;
	}

	return i2csetwbc(des->i2c_client, 0x08B0, (val & (~BIT(7))));
}

static int csi_stream(struct adi_des *des, bool enable)
{
	int rv;
	u32 val;

	rv = i2cgetwbc(des->i2c_client, 0x08B0, &val);
	if (rv) {
		dev_err(des->dev, "%s: Failed to get 0x08B0\n", __func__);
		return rv;
	}

	if (STREAM_DEC_EN(enable)) {
		if (val & BIT(7))
			return 0;
		i2csetwbc(des->i2c_client, 0x08B0, (val | BIT(7)));
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
		i2csetwbc(des->i2c_client, 0x00C1, 0x00);
	} else if (des->i2c_port == 1) {
		i2csetwbc(des->i2c_client, 0x0003, 0x55);
		i2csetwbc(des->i2c_client, 0x00C1, 0x0F);
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
	/* TODO: Chip limit, only support link A now */
	dev_err(des->dev, "Unsupported now\n");

	return -EINVAL;
}

static int set_fsync_outer(struct adi_des *des)
{
	u32 tx_id;
	u32 off;
	u32 en;

	tx_id = des->fsync_rx_pin << 2;
	off = des->fsync_rx_pin * 8;
	en = BIT(1);
	i2csetwbc(des->i2c_client, 0x202 + off, tx_id | en); // Link A
	i2csetwbc(des->i2c_client, 0x203 + off, tx_id | en); // Link B
	i2csetwbc(des->i2c_client, 0x204 + off, tx_id | en); // Link C
	i2csetwbc(des->i2c_client, 0x205 + off, tx_id | en); // Link D

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
	set_csi_timing(des);
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

	rv = i2cgetwbc(des->i2c_client, 0x0404, &val);
	dev_info(des->dev, "%s: rv: %d, val: 0x%02X\n", __func__, rv, val);
	if (!rv && ((val & BIT(3)) == 0))
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
	struct device *dev;
	struct rx_port *rxp;
	struct camera_dev *cam;

	rxp = &des->rx_ports[port];
	cam = rxp->cam;
	if (cam == NULL)
		return 0;

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

	return rv;
}

static const struct des_ops max96726_ops = {
	.is_link_locked = is_link_locked,
	.is_video_locked = is_video_locked,
	.csi_pre_streamon = csi_pre_streamon,
	.csi_stream = csi_stream,
	.des_setup = des_setup,
	.is_des_setuped = is_des_setuped,
	.gmsl_setup = gmsl_setup,
};

static int max96726_probe(struct i2c_client *client,
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
	des->ops = &max96726_ops;
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

static void max96726_remove(struct i2c_client *client)
{
	struct adi_des *des = i2c_get_clientdata(client);

	dev_info(des->dev, "Remove\n");
	adi_des_sysfs_exit(des);
}

static void max96726_shutdown(struct i2c_client *client)
{
	struct adi_des *des = i2c_get_clientdata(client);

	dev_info(des->dev, "Shutdown\n");
	if (des->role != ROLE_MASTER)
		return;

	adi_des_exit_lock_handler(des);
}

static int max96726_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct adi_des *des = i2c_get_clientdata(client);

	dev_info(des->dev, "Suspend\n");

	return 0;
}

static int max96726_resume(struct device *dev)
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

static const struct dev_pm_ops max96726_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(max96726_suspend, max96726_resume)
};

// clang-format off
static const struct reg_cfg max96726_pre_gmsl[] = {};
static const struct reg_cfg max96726_post_gmsl[] = {};
static const struct reg_cfg max96726_pre_csi[] = {};
static const struct reg_cfg max96726_post_csi[] = {};
// clang-format on

static const struct des_param max96726_params = {
	.des_type = DES_MAX96726,
	.num_gmsl = 4,
	.num_pipe = 8,
	.num_csi = 2,
	.num_i2c = 2,
	.num_mfp = 16,
	.t_lock = 120000, // NOTE: XP, should be tLock2, but also see ERRATA
	.t_i2c_wake = 2250,
	.gmsl_ver_lo = GMSL2,
	.gmsl_ver_up = GMSL3,
	.csi_lo = CSI1,
	.csi_up = CSI2,
	.pre_gmsl = __REG_CFGS(max96726_pre_gmsl),
	.post_gmsl = __REG_CFGS(max96726_post_gmsl),
	.pre_csi = __REG_CFGS(max96726_pre_csi),
	.post_csi = __REG_CFGS(max96726_post_csi),
};

static const struct of_device_id max96726_of_ids[] = {
	// clang-format off
	{ .compatible = "bst,max96726", .data = &max96726_params, },
	{},
	// clang-format on
};
MODULE_DEVICE_TABLE(of, max96726_of_ids);

static struct i2c_driver max96726_driver = {
	.driver = {
		.name = "bst,max96726",
		.of_match_table = of_match_ptr(max96726_of_ids),
		.pm = &max96726_pm_ops,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = max96726_probe,
	.remove = max96726_remove,
	.shutdown = max96726_shutdown,
};
module_i2c_driver(max96726_driver);

MODULE_DESCRIPTION("BST Max96726 driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("BST Ltd.");
