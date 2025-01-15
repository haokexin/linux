// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/media-entity.h>
#include <media/media-device.h>
#include "ox08b_config.h"
#include "camera_common_op.h"

#define MAX9295_ID_REG		0x0d
#define MAX9295_ID			0x91
#define MAX9295_I2CADDR		0x40
#define MODULE_NAME "bst,ox08b"
#define ROW_TIME_24M_30FPS 29291
#define ROW_TIME_24M_20FPS 29291
#define ROW_TIME_27M_30FPS 24582
#define ROW_TIME_27M_20FPS 24582
#define MODULE_NAME_HKX8B "bst,hk_ox08b"
#define MODULE_NAME_JINHUAX8B "bst,jinghua_ox08b"
/*vts must be same with the value of (0x380e,0x380f),row_time = 10000000000/fps/vts*/
static struct ox08b_sensor_base_cfg jh_x8b_settings[] = {
	//lfm setting
	{0x5d46, 0x00},
	{0x5d47, 0xe0},
	{0x5d48, 0x00},
	{0x5d49, 0xe8},
	{0x5003, 0x7a},
	//preccm ct
	{0x5b7e, 0x01},
	{0x5b78, 0x00},
	{0x5b79, 0x61},
	{0x5b7a, 0x01},
	{0x5b7b, 0x01},
	{0x5b7c, 0x01},
	{0x5b7d, 0x73},
	//D
	{0x5B40, 0x01},
	{0x5B41, 0x7D},
	{0x5B42, 0x0F},
	{0x5B43, 0x96},
	{0x5B44, 0x0F},
	{0x5B45, 0xED},
	{0x5B46, 0x0F},
	{0x5B47, 0xF4},
	{0x5B48, 0x01},
	{0x5B49, 0x41},
	{0x5B4A, 0x0F},
	{0x5B4B, 0xCA},
	{0x5B4C, 0x00},
	{0x5B4D, 0x03},
	{0x5B4E, 0x0F},
	{0x5B4F, 0xAF},
	{0x5B50, 0x01},
	{0x5B51, 0x4E},
	//C
	{0x5B52, 0x01},
	{0x5B53, 0xAA},
	{0x5B54, 0x0F},
	{0x5B55, 0x6D},
	{0x5B56, 0x0F},
	{0x5B57, 0xE9},
	{0x5B58, 0x00},
	{0x5B59, 0x00},
	{0x5B5A, 0x01},
	{0x5B5B, 0x38},
	{0x5B5C, 0x0F},
	{0x5B5D, 0xC8},
	{0x5B5E, 0x00},
	{0x5B5F, 0x08},
	{0x5B60, 0x0F},
	{0x5B61, 0xAD},
	{0x5B62, 0x01},
	{0x5B63, 0x4B},
	//A
	{0x5B64, 0x02},
	{0x5B65, 0x4C},
	{0x5B66, 0x0E},
	{0x5B67, 0xEA},
	{0x5B68, 0x0F},
	{0x5B69, 0xCA},
	{0x5B6A, 0x00},
	{0x5B6B, 0x18},
	{0x5B6C, 0x01},
	{0x5B6D, 0x39},
	{0x5B6E, 0x0F},
	{0x5B6F, 0xAF},
	{0x5B70, 0x00},
	{0x5B71, 0x19},
	{0x5B72, 0x0F},
	{0x5B73, 0x9E},
	{0x5B74, 0x01},
	{0x5B75, 0x4A},
	{0xffff, 0xff} //the register to jude to stop
};
static struct ox08b_sensor_base_cfg hk_x8b_settings[] = {
	{0x5b7e, 0x01},
	{0x5b7e, 0x01},
	{0x5b78, 0x00},
	{0x5b79, 0x50},
	{0x5b7a, 0x01},
	{0x5b7b, 0x0e},
	{0x5b7c, 0x01},
	{0x5b7d, 0x70},
	{0x5b40, 0x01},
	{0x5b41, 0x9c},
	{0x5b42, 0x0f},
	{0x5b43, 0x7b},
	{0x5b44, 0x0f},
	{0x5b45, 0xe9},
	{0x5b46, 0x00},
	{0x5b47, 0x00},
	{0x5b48, 0x01},
	{0x5b49, 0x3e},
	{0x5b4a, 0x0f},
	{0x5b4b, 0xc1},
	{0x5b4c, 0x00},
	{0x5b4d, 0x07},
	{0x5b4e, 0x0f},
	{0x5b4f, 0xaf},
	{0x5b50, 0x01},
	{0x5b51, 0x4a},
	{0x5b52, 0x01},
	{0x5b53, 0xbf},
	{0x5b54, 0x0f},
	{0x5b55, 0x62},
	{0x5b56, 0x0f},
	{0x5b57, 0xdf},
	{0x5b58, 0x00},
	{0x5b59, 0x00},
	{0x5b5a, 0x01},
	{0x5b5b, 0x49},
	{0x5b5c, 0x0f},
	{0x5b5d, 0xb8},
	{0x5b5e, 0x00},
	{0x5b5f, 0x05},
	{0x5b60, 0x0f},
	{0x5b61, 0xb9},
	{0x5b62, 0x01},
	{0x5b63, 0x42},
	{0x5b64, 0x02},
	{0x5b65, 0x9c},
	{0x5b66, 0x0e},
	{0x5b67, 0xa2},
	{0x5b68, 0x0f},
	{0x5b69, 0xc2},
	{0x5b6a, 0x00},
	{0x5b6b, 0x40},
	{0x5b6c, 0x01},
	{0x5b6d, 0x1a},
	{0x5b6e, 0x0f},
	{0x5b6f, 0xa6},
	{0x5b70, 0x00},
	{0x5b71, 0x3a},
	{0x5b72, 0x0f},
	{0x5b73, 0x70},
	{0x5b74, 0x01},
	{0x5b75, 0x57},
	{0xffff, 0xff} //the register to jude to stop
};

static int ox08b_parse_dts(struct camera_dev *cam_dev, struct device_node *node)
{
	if (of_property_read_s32(node, "clock-frequency", &cam_dev->clock_frequency)) {
		dev_err(cam_dev->dev,
			"Invalid DT clock-frequency\n");
		return -EINVAL;
	}
	if (of_property_read_s32(node, "sensor-fps", &cam_dev->sensor_fps)) {
		dev_err(cam_dev->dev,
			"Invalid DT sensor-fps\n");
		return -EINVAL;
	}
	return 0;
}

static int ox08b_ser_cfg(struct camera_dev *ox08b_raw)
{
	int i;
	int retry_times;
	int ret;
	struct i2c_adapter *adap;

	pr_info("ox08b %s(), line %d\n", __func__, __LINE__);

	if (ox08b_raw == NULL) {
		pr_info("%s : camera_dev is NULL\n", __func__);
		return -EINVAL;
	}
	adap = ox08b_raw->i2c_client->adapter;

	if (adap == NULL)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(config_serdes_setting); i++) {
		retry_times = 12;
		ret = -1;
		while (retry_times > 0 && ret) {
			ret = bst_i2c_write_byte_data_word_reg(
				adap, ox08b_raw->ser_alias_id,
				config_serdes_setting[i][0],
				config_serdes_setting[i][1]);
			if (ret) {
				pr_info(": write_max9295_reg failed!\n");
				retry_times--;
				usleep_range(2000, 2500);
				continue;
			}
			pr_info("write max9295 reg:%#x, val:%#x",
			       config_serdes_setting[i][0],
			       config_serdes_setting[i][1]);
		}
	}
	//modify sensor real i2c address
	bst_i2c_write_byte_data_word_reg(adap, ox08b_raw->ser_alias_id, 0x0042,
					 (ox08b_raw->sensor_alias_id << 1));
	bst_i2c_write_byte_data_word_reg(adap, ox08b_raw->ser_alias_id, 0x0043,
					 0x6c);

	return 0;
}

static int ox08b_power_on(struct camera_dev *cam_dev, struct device_node *node)
{
	int i, sensor_cfg_size;
	int ret;
	struct i2c_adapter *adap;
	struct ox08b_sensor_base_cfg *x8b_priv_setting;
	const struct ox08b_sensor_base_cfg *x8b_ltm_exp_setting;
	/*according to fps to chose sensor setting*/
	if (cam_dev->i2c_client == NULL)
		return -EINVAL;

	adap = cam_dev->i2c_client->adapter;
	if (adap == NULL)
		return -EINVAL;

	if (cam_dev->clock_frequency == 24 && cam_dev->sensor_fps == 20) {
		x8b_priv_setting = sensor_base_settings_24m_20fps;
		sensor_cfg_size = ARRAY_SIZE(sensor_base_settings_24m_20fps);
	} else if (cam_dev->clock_frequency == 24 && cam_dev->sensor_fps == 30) {
		x8b_priv_setting = sensor_base_settings_24m_30fps;
		sensor_cfg_size = ARRAY_SIZE(sensor_base_settings_24m_30fps);
	} else if (cam_dev->clock_frequency == 27 && cam_dev->sensor_fps == 20) {
		x8b_priv_setting = sensor_base_settings_27m_20fps;
		sensor_cfg_size = ARRAY_SIZE(sensor_base_settings_27m_20fps);
	}  else if (cam_dev->clock_frequency == 27 && cam_dev->sensor_fps == 30) {
		x8b_priv_setting = sensor_base_settings_27m_30fps;
		sensor_cfg_size = ARRAY_SIZE(sensor_base_settings_27m_30fps);
	} else {
		pr_err("parse clock-frequency or sensor-fps fail, error\n");
		return -EINVAL;
	}
	for (i = 0; i < sensor_cfg_size; i++) {
		ret = bst_i2c_write_byte_data_word_reg(
			adap, cam_dev->sensor_alias_id,
			x8b_priv_setting[i].reg,
			x8b_priv_setting[i].value);
		if (x8b_priv_setting[i].reg == 0x0107)
			usleep_range(10000, 20000);
		if (ret) {
			dev_err(cam_dev->dev, "ox08b sensor %x,%x write failed",
				x8b_priv_setting[i].reg,
				x8b_priv_setting[i].value);
			continue;
		}
	}
	x8b_ltm_exp_setting = of_device_get_match_data(cam_dev->dev);
	if (x8b_ltm_exp_setting == NULL) {
		//this no need to write ltm_exp setting;
		return 0;
	}
	i = 0;
	while ((x8b_ltm_exp_setting[i].reg != 0xffff) || (x8b_ltm_exp_setting[i].value != 0xff)) {
		ret = bst_i2c_write_byte_data_word_reg(
			adap, cam_dev->sensor_alias_id,
			x8b_ltm_exp_setting[i].reg,
			x8b_ltm_exp_setting[i].value);
		if (ret) {
			dev_err(cam_dev->dev, "hk_ox08b sensor %x,%x write failed",
				x8b_ltm_exp_setting[i].reg,
				x8b_ltm_exp_setting[i].value);
			i++;
			continue;
		}
		i++;
	}
	return 0;
}

static int camera_s_power(struct v4l2_subdev *sd, int enable)
{
	int ret;
	struct camera_dev *ox08b_raw =
		container_of(sd, struct camera_dev, subdev);

	pr_info("ox08b %s(), line %d\n", __func__, __LINE__);

	if (!enable)
		return 0;

	ox08b_parse_dts(ox08b_raw, ox08b_raw->dev->of_node);
	if (ox08b_raw->clock_frequency == 24) {
		//20fps,30fps,same row time
		ox08b_raw->row_time = ROW_TIME_24M_30FPS;
	} else if (ox08b_raw->clock_frequency == 27) {
		//20fps,30fps,same row time
		ox08b_raw->row_time = ROW_TIME_27M_30FPS;
	}

	if (!ox08b_raw->maxim_power_on) {
		ox08b_raw->power_on = false;
		return -EINVAL;
	}

	if (!is_slave_soc_model(ox08b_raw)) {
		ret = ox08b_ser_cfg(ox08b_raw);
		if (ret)
			return ret;

		ox08b_power_on(ox08b_raw, ox08b_raw->dev->of_node);
	}
	ox08b_raw->power_on = true;

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Pad Operations
 */

static int camera_get_format(struct v4l2_subdev *subdev,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt)
{
	return 0;
}

static int camera_set_format(struct v4l2_subdev *subdev,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt)
{
	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Operations
 */

static const struct v4l2_subdev_pad_ops camera_pad_ops = {
	.get_fmt = camera_get_format,
	.set_fmt = camera_set_format,
};

static const struct v4l2_subdev_core_ops camera_core_ops = {
	.s_power = camera_s_power,
};

static const struct v4l2_subdev_ops camera_ops = {
	.core = &camera_core_ops,
	.pad = &camera_pad_ops,
};

/* -----------------------------------------------------------------------------
 * Media Operations
 */

static const struct media_entity_operations ox08b_raw_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

// sensor is not sured powered on this time
// don't check camera connect here
static int ox08b_raw_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct camera_dev *ox08b_raw;
	struct device *dev = &client->dev;
	struct resource res;
	int bus_index = 0;
	int ret;

	ox08b_raw = devm_kzalloc(&client->dev, sizeof(struct camera_dev),
				 GFP_KERNEL);
	if (!ox08b_raw)
		return -ENOMEM;

	if (client->adapter == NULL) {
		pr_err("client->adapter == NULL, error\n");
		return -1;
	};
	ox08b_raw->i2c_client = client;
	ox08b_raw->dev = dev;
	bus_index = client->adapter->nr;

	if (of_address_to_resource(client->adapter->dev.of_node, 0, &res)) {
		dev_err(dev, "Can not get adapter's address\n'");
		return -1;
	}

	ox08b_raw->isp_data.i2cRegBase = res.start;
	ox08b_raw->isp_data.sensorRdWrMode =
		BST_SENSOR_RW_MODE_WORD_REG_WORD_DATA;
	ox08b_raw->isp_data.sensorType = BST_SENSOR_TYPE_OX08B_RAW;

	ret = parse_camera_endpoint(ox08b_raw, dev->of_node);
	if (ret) {
		dev_err(dev, "init_camera_dev error, ret = %d\n", ret);
		return ret;
	}

	ret = init_camera_dev(ox08b_raw, &camera_ops, &ox08b_raw_media_ops);
	if (ret) {
		dev_err(dev, "init_camera_dev error, ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static void ox08b_raw_remove(struct i2c_client *client)
{
}

static const struct i2c_device_id ox08b_raw_id[] = {
	{ MODULE_NAME, 0 },
	{},
};

static const struct of_device_id ox08b_raw_of_match[] = {
	{.compatible = MODULE_NAME_HKX8B, .data = hk_x8b_settings},
	{.compatible = MODULE_NAME_JINHUAX8B, .data = jh_x8b_settings},
	{},
};

MODULE_DEVICE_TABLE(of, ox08b_raw_of_match);
MODULE_DEVICE_TABLE(i2c, ox08b_raw_id);

static struct i2c_driver ox08b_raw_driver = {
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(ox08b_raw_of_match),
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe		= ox08b_raw_probe,
	.remove		= ox08b_raw_remove,
	.id_table	= ox08b_raw_id,
};

module_i2c_driver(ox08b_raw_driver);

MODULE_DESCRIPTION("OX08B RAW camera driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("BST Ltd.");
