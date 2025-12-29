// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2025 Black Sesame Technologies. All Rights Reserved.
 */

#include <dt-bindings/media/bst-isp.h>

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-fwnode.h>

#include <bst/media-dev.h>

#include "camera.h"
#include "utils.h"

static int parse_dt(struct camera_dev *cam)
{
	int rv;
	struct device_node *node;

	node = cam->dev->of_node;
	/* Sensor */
	(void)of_property_read_string(node, "compatible", &cam->name);
	(void)of_property_read_u32(node, "sensor-addr", &cam->sensor_addr);
	(void)of_property_read_u32(node, "data-type", &cam->data_type);
	(void)of_property_read_u32(node, "sensor-fps", &cam->sensor_fps);
	(void)of_property_read_u32(node, "row-time", &cam->row_time);
	(void)of_property_read_u32(node, "reset-delay", &cam->reset_delay);
	rv = of_property_read_u32(node, "reg-width", &cam->reg_width);
	if (rv)
		cam->reg_width = -EINVAL;
	(void)of_property_read_string(node, "algo-bin", &cam->algo);
	(void)of_property_read_string(node, "iq-bin", &cam->iq);
	(void)of_property_read_u32(node, "role", &cam->role);
	(void)of_property_read_u32(node, "id-reg", &cam->id_reg);
	(void)of_property_read_u32(node, "id-val", &cam->id_val);

	/* Serializer */
	(void)of_property_read_u32(node, "ser-type", &cam->ser_type);
	(void)of_property_read_u32(node, "ser-addr", &cam->ser_addr);
	(void)of_property_read_u32(node, "ser-alias", &cam->ser_alias);
	(void)of_property_read_u32(node, "ser-fsync-tx-pin",
				   &cam->ser_fsync_tx_pin);
	cam->ser_reset = of_property_read_bool(node, "ser-reset");
	cam->ser_reg_width = cam->reg_width;
	(void)of_property_read_u32(node, "ser-reg-width", &cam->ser_reg_width);
	cam->ser_role = ROLE_MASTER;
	(void)of_property_read_u32(node, "ser-role", &cam->ser_role);

	/* Support direct connection to CSI RX */
	(void)of_property_read_u32(node, "phy-if", &cam->tx_dev.phy_if);
	(void)of_property_read_u32(node, "lane-num", &cam->tx_dev.lane_num);
	(void)of_property_read_u32(node, "lane-speed", &cam->tx_dev.lane_speed);

	/* Extended device */
	(void)of_property_read_u32(node, "ext-addr", &cam->ext_addr);
	(void)of_property_read_u32(node, "ext-alias", &cam->ext_alias);
	cam->ext_reg_width = cam->reg_width;
	(void)of_property_read_u32(node, "ext-reg-width", &cam->ext_reg_width);
	cam->ext_role = ROLE_MASTER;
	(void)of_property_read_u32(node, "ext-role", &cam->ext_role);

	cam->cfg_with_delay = of_property_read_bool(node, "cfg-with-delay");

	cam->sensor_alias = cam->i2c_client->addr;

	return 0;
}

static int set_i2c_ops(struct camera_dev *cam)
{
	switch (cam->reg_width) {
	case WORD_REG_WORD_VAL:
		cam->i2cset = i2csetww;
		cam->i2cget = i2cgetww;
		break;
	case WORD_REG_BYTE_VAL:
		cam->i2cset = i2csetwb;
		cam->i2cget = i2cgetwb;
		break;
	case BYTE_REG_BYTE_VAL:
		cam->i2cset = i2csetbb;
		cam->i2cget = i2cgetbb;
		break;
	default:
		dev_err(cam->dev, "Unsupported I2C operations for sensor\n");
		return -EINVAL;
	}

	switch (cam->ser_reg_width) {
	case WORD_REG_WORD_VAL:
		cam->ser_i2cset = i2csetww;
		cam->ser_i2cget = i2cgetww;
		break;
	case WORD_REG_BYTE_VAL:
		cam->ser_i2cset = i2csetwb;
		cam->ser_i2cget = i2cgetwb;
		break;
	case BYTE_REG_BYTE_VAL:
		cam->ser_i2cset = i2csetbb;
		cam->ser_i2cget = i2cgetbb;
		break;
	default:
		dev_err(cam->dev,
			"Unsupported I2C operations for serializer\n");
		return -EINVAL;
	}

	switch (cam->ext_reg_width) {
	case WORD_REG_WORD_VAL:
		cam->ext_i2cset = i2csetww;
		cam->ext_i2cget = i2cgetww;
		break;
	case WORD_REG_BYTE_VAL:
		cam->ext_i2cset = i2csetwb;
		cam->ext_i2cget = i2cgetwb;
		break;
	case BYTE_REG_BYTE_VAL:
		cam->ext_i2cset = i2csetbb;
		cam->ext_i2cget = i2cgetbb;
		break;
	default:
		dev_err(cam->dev,
			"Unsupported I2C operations for extended device\n");
		return -EINVAL;
	}

	return 0;
}

static int choose_cfg(struct camera_dev *cam)
{
	const struct camera_cfg_set *cfg_set;
	const struct camera_cfg *cfg;
	const struct camera_cfg *best;
	struct device *dev;
	int i;

	dev = cam->dev;
	cfg_set = of_device_get_match_data(dev);
	if (!cfg_set) {
		dev_err(dev, "Failed to find cfg set\n");
		return -EINVAL;
	}
	cam->cfg_set = cfg_set;

	best = NULL;
	for (i = 0; i < cfg_set->num; ++i) {
		cfg = cfg_set->cfgs[i];
		dev_dbg(cam->dev, "DT: 0x%02X vs 0x%02X, fps: %u vs %u\n",
			cfg->data_type, cam->data_type, cfg->fps,
			cam->sensor_fps);
		if (cfg->data_type != cam->data_type)
			continue;

		best = cfg;
		if (cfg->fps == cam->sensor_fps)
			break;
	}
	if (best == NULL) {
		dev_err(dev, "Failed to choose cfg\n");
		return -EINVAL;
	}

	cam->cfg = best;
	if (cam->row_time == 0)
		cam->row_time = cfg->row_time;
	if (cam->reset_delay == 0)
		cam->reset_delay = cfg_set->reset_delay;

	return 0;
}

static int set_ser_init(struct camera_dev *cam)
{
	struct i2c_adapter *adap = cam->i2c_client->adapter;
	struct device_node *node = cam->dev->of_node;
	const struct camera_cfg *cfg = cam->cfg;
	int cfg_num;

	cfg_num = of_property_count_u32_elems(node, "ser-init");
	if (cfg_num > 0)
		return i2cset_from_dt(adap, cam->ser_alias, cam->ser_i2cset,
				      node, "ser-init", cam->cfg_with_delay);
	else
		return i2cset_in_bulk(adap, cam->ser_alias, cam->ser_i2cset,
				      &cfg->ser_init);
}

static int set_ser_pre_stream_on(struct camera_dev *cam)
{
	struct i2c_adapter *adap = cam->i2c_client->adapter;
	struct device_node *node = cam->dev->of_node;
	const struct camera_cfg *cfg = cam->cfg;
	int cfg_num;

	cfg_num = of_property_count_u32_elems(node, "ser-pre-stream-on");
	if (cfg_num > 0)
		return i2cset_from_dt(adap, cam->ser_alias, cam->ser_i2cset,
				      node, "ser-pre-stream-on",
				      cam->cfg_with_delay);
	else
		return i2cset_in_bulk(adap, cam->ser_alias, cam->ser_i2cset,
				      &cfg->ser_pre_stream_on);
}

static int set_ext_init(struct camera_dev *cam)
{
	struct i2c_adapter *adap = cam->i2c_client->adapter;
	struct device_node *node = cam->dev->of_node;
	const struct camera_cfg *cfg = cam->cfg;
	int cfg_num;

	cfg_num = of_property_count_u32_elems(node, "ext-init");
	if (cfg_num > 0)
		return i2cset_from_dt(adap, cam->ext_alias, cam->ext_i2cset,
				      node, "ext-init", cam->cfg_with_delay);
	else
		return i2cset_in_bulk(adap, cam->ext_alias, cam->ext_i2cset,
				      &cfg->ext_init);
}

static int set_sensor_reset(struct camera_dev *cam)
{
	struct i2c_adapter *adap = cam->i2c_client->adapter;
	struct device_node *node = cam->dev->of_node;
	const struct camera_cfg *cfg = cam->cfg;
	int cfg_num;

	cfg_num = of_property_count_u32_elems(node, "sensor-reset");
	if (cfg_num > 0)
		return i2cset_from_dt(adap, cam->sensor_alias, cam->i2cset,
				      node, "sensor-reset",
				      cam->cfg_with_delay);
	else
		return i2cset_in_bulk(adap, cam->sensor_alias, cam->i2cset,
				      &cfg->sensor_reset);
}

static int set_sensor_init(struct camera_dev *cam)
{
	struct i2c_adapter *adap = cam->i2c_client->adapter;
	struct device_node *node = cam->dev->of_node;
	const struct camera_cfg *cfg = cam->cfg;
	int cfg_num;

	cfg_num = of_property_count_u32_elems(node, "sensor-init");
	if (cfg_num > 0)
		return i2cset_from_dt(adap, cam->sensor_alias, cam->i2cset,
				      node, "sensor-init", cam->cfg_with_delay);
	else
		return i2cset_in_bulk(adap, cam->sensor_alias, cam->i2cset,
				      &cfg->sensor_init);
}

static int set_sensor_pre_stream_on(struct camera_dev *cam)
{
	struct i2c_adapter *adap = cam->i2c_client->adapter;
	struct device_node *node = cam->dev->of_node;
	const struct camera_cfg *cfg = cam->cfg;
	int cfg_num;

	cfg_num = of_property_count_u32_elems(node, "sensor-pre-stream-on");
	if (cfg_num > 0)
		return i2cset_from_dt(adap, cam->sensor_alias, cam->i2cset,
				      node, "sensor-pre-stream-on",
				      cam->cfg_with_delay);
	else
		return i2cset_in_bulk(adap, cam->sensor_alias, cam->i2cset,
				      &cfg->sensor_pre_stream_on);
}

static int set_sensor_stream_on(struct camera_dev *cam)
{
	struct i2c_adapter *adap = cam->i2c_client->adapter;
	struct device_node *node = cam->dev->of_node;
	const struct camera_cfg *cfg = cam->cfg;
	int cfg_num;

	cfg_num = of_property_count_u32_elems(node, "sensor-stream-on");
	if (cfg_num > 0)
		return i2cset_from_dt(adap, cam->sensor_alias, cam->i2cset,
				      node, "sensor-stream-on",
				      cam->cfg_with_delay);
	else
		return i2cset_in_bulk(adap, cam->sensor_alias, cam->i2cset,
				      &cfg->sensor_stream_on);
}

static int set_sensor_stream_off(struct camera_dev *cam)
{
	struct i2c_adapter *adap = cam->i2c_client->adapter;
	struct device_node *node = cam->dev->of_node;
	const struct camera_cfg *cfg = cam->cfg;
	int cfg_num;

	cfg_num = of_property_count_u32_elems(node, "sensor-stream-off");
	if (cfg_num > 0)
		return i2cset_from_dt(adap, cam->sensor_alias, cam->i2cset,
				      node, "sensor-stream-off",
				      cam->cfg_with_delay);
	else
		return i2cset_in_bulk(adap, cam->sensor_alias, cam->i2cset,
				      &cfg->sensor_stream_off);
}

static int apply_cfg(struct camera_dev *cam)
{
	int rv;
	struct device *dev = cam->dev;
	struct i2c_adapter *adap = cam->i2c_client->adapter;

	if (cam->ser_role == ROLE_MASTER) {
		rv = set_ser_init(cam);
		if (rv) {
			dev_err_ratelimited(
				dev, "Failed to apply ser_init, rv: %d\n", rv);
			return rv;
		}
	}

	if (cam->ext_role == ROLE_MASTER) {
		rv = set_ext_init(cam);
		if (rv) {
			dev_err_ratelimited(
				dev, "Failed to apply ext_init, rv: %d\n", rv);
			return rv;
		}
	}

	if (cam->id_reg != 0 || cam->id_val) {
		u32 val;

		rv = cam->i2cget(adap, cam->sensor_alias, cam->id_reg, &val);
		if (!rv && val != cam->id_val)
			dev_err_ratelimited(
				dev,
				"Unmatched ID, expect: 0x%04X, real: 0x%04X, sensor may be not %s\n",
				cam->id_val, val, cam->name);
	}

	if (cam->role == ROLE_MASTER) {
		rv = set_sensor_reset(cam);
		if (rv) {
			dev_err_ratelimited(
				dev, "Failed to apply sensor_reset, rv: %d\n",
				rv);
			return rv;
		}
		ursleep(cam->reset_delay);

		rv = set_sensor_init(cam);
		if (rv) {
			dev_err_ratelimited(
				dev, "Failed to apply sensor_init, rv: %d\n",
				rv);
			return rv;
		}

		rv = set_sensor_pre_stream_on(cam);
		if (rv) {
			dev_err_ratelimited(
				dev,
				"Failed to apply sensor_pre_stream_on, rv: %d\n",
				rv);
			return rv;
		}
	}

	if (cam->ser_role == ROLE_MASTER) {
		rv = set_ser_pre_stream_on(cam);
		if (rv) {
			dev_err_ratelimited(
				dev,
				"Failed to apply ser_pre_stream_on, rv: %d\n",
				rv);
			return rv;
		}
	}

	if (cam->role == ROLE_MASTER) {
		rv = set_sensor_stream_on(cam);
		if (rv) {
			dev_err_ratelimited(
				dev,
				"Failed to apply sensor_stream_on, rv: %d\n",
				rv);
			return rv;
		}
	}

	return 0;
}

static int camera_s_power(struct v4l2_subdev *sd, int enable)
{
	struct camera_dev *cam;

	/* NOTE: Never power off now */
	if (!enable)
		return -EINVAL;

	cam = subdev_to_camera_dev(sd);

	if (apply_cfg(cam))
		cam->power_on = false;
	else
		cam->power_on = true;

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Operations
 */
static const struct v4l2_subdev_core_ops camera_core_ops = {
	.s_power = camera_s_power,
};

static int s_stream(struct v4l2_subdev *sd, int enable)
{
	struct camera_dev *cam;

	cam = subdev_to_camera_dev(sd);
	if (cam->role != ROLE_MASTER)
		return 0;

	if (enable)
		return set_sensor_stream_on(cam);
	else
		return set_sensor_stream_off(cam);
}

static int pre_streamon(struct v4l2_subdev *sd, u32 flags)
{
	struct camera_dev *cam;

	cam = subdev_to_camera_dev(sd);
	if (cam->role != ROLE_MASTER)
		return 0;

	return set_sensor_pre_stream_on(cam);
}

static const struct v4l2_subdev_video_ops camera_video_ops = {
	.s_stream = s_stream,
	.pre_streamon = pre_streamon,
};

static const struct v4l2_subdev_pad_ops camera_pad_ops = {
	.get_mbus_config = csi_tx_get_mbus_config,
};

static const struct v4l2_subdev_ops camera_subdev_ops = {
	.core = &camera_core_ops,
	.video = &camera_video_ops,
	.pad = &camera_pad_ops,
};

static int init_v4l2_dev(struct camera_dev *cam)
{
	int rv;
	struct v4l2_subdev *sd;
	struct device *dev;

	dev = cam->dev;
	sd = &(cam->tx_dev.subdev);

	v4l2_subdev_init(sd, &camera_subdev_ops);
	sd->dev = cam->dev;
	sd->fwnode = of_fwnode_handle(cam->dev->of_node);
	snprintf(sd->name, sizeof(sd->name), "%s-%02X", dev_name(cam->dev),
		 cam->i2c_client->addr);
	v4l2_set_subdevdata(sd, cam);

	rv = v4l2_async_register_subdev(sd);
	if (rv < 0)
		dev_err(dev, "Failed to register subdev\n");

	return rv;
}

static void cleanup_v4l2_dev(struct camera_dev *cam)
{
	v4l2_async_unregister_subdev(&cam->tx_dev.subdev);
}

static int camera_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int rv;
	struct camera_dev *cam;
	struct device *dev;

	dev = &client->dev;
	cam = devm_kzalloc(dev, sizeof(*cam), GFP_KERNEL);
	if (!cam)
		return -ENOMEM;

	cam->i2c_client = client;
	cam->dev = dev;
	i2c_set_clientdata(client, cam);

	rv = parse_dt(cam);
	if (rv)
		goto exit;

	rv = set_i2c_ops(cam);
	if (rv)
		goto exit;

	rv = choose_cfg(cam);
	if (rv)
		goto exit;

	rv = init_v4l2_dev(cam);
	if (rv)
		goto exit;

	dev_info(dev, "Probe done on CPU %u\n", smp_processor_id());

	return 0;

exit:
	return rv;
}

static void camera_remove(struct i2c_client *client)
{
	struct camera_dev *cam;

	cam = i2c_get_clientdata(client);
	dev_info(cam->dev, "Remove\n");

	cleanup_v4l2_dev(cam);
}

static const struct of_device_id camera_of_ids[] = {
	/* RAW */
	{ .compatible = "bst,ds_imx623", .data = &imx623_raw_cfg_set },
	{ .compatible = "bst,hk_ox03c", .data = &ox03c_raw_cfg_set },
	{ .compatible = "bst,of_ox03c", .data = &ox03c_raw_cfg_set },
	{ .compatible = "bst,hk_ox03f", .data = &ox03f_raw_cfg_set },
	{ .compatible = "bst,hk_ox08b", .data = &ox08b_raw_cfg_set_hk },
	{ .compatible = "bst,jh_ox08b", .data = &ox08b_raw_cfg_set_jh },
	{ .compatible = "bst,li_ov2311", .data = &ov2311_raw_cfg_set_li },

	/* YUV */
	{ .compatible = "bst,ox03c-yuv-hil", .data = &ox03c_hil_yuv_cfg_set },
	{ .compatible = "bst,ox03f-yuv-hil", .data = &ox03f_hil_yuv_cfg_set },
	{ .compatible = "bst,ox08b-yuv-hil", .data = &ox08b_hil_yuv_cfg_set },

	/* Customized */
	{ .compatible = "bst,custom_camera", .data = &custom_cfg_set },
	{},
};
MODULE_DEVICE_TABLE(of, camera_of_ids);

static struct i2c_driver camera_driver = {
	.driver = {
		.name = "bst,camera",
		.of_match_table = of_match_ptr(camera_of_ids),
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe		= camera_probe,
	.remove		= camera_remove,
};
module_i2c_driver(camera_driver);

MODULE_DESCRIPTION("BST Camera Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("BST Ltd.");
