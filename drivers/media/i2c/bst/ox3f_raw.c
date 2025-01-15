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
#include "ox3f_config.h"
#include "camera_common_op.h"

#define MAX96717f_I2CADDR		0x40

#define MODULE_NAME "bst,ox3f"
#define MAX_SER_DEVICE_ID 0x0000
#define ROW_TIME_25M 49019
/*vts must be same with the value of (0x380e,0x380f),row_time = 10000000000/fps/vts*/
//#define GUANG_ZHEN_GPIO_ANBLE 0X12 //jinghua x8b 0x00,oufei x3c 0x00,maxieye x3c 0x00 ,sunyu x3c 0x00 ,guangzhen x3c 0x12
static struct ox3f_sensor_base_cfg x3f_ltm_exp_settings[] = {
	{0x5d46, 0x00},
	{0x5d47, 0xe0},
	{0x5d48, 0x00},
	{0x5d49, 0xe8},
	{0x5003, 0x7a},
	{0x5b7e, 0x01},
	{0x5b78, 0x00},
	{0x5b79, 0x64},
	{0x5b7a, 0x00},
	{0x5b7b, 0xE1},
	{0x5b7c, 0x01},
	{0x5b7d, 0x18},
	{0x5B40, 0x01},
	{0x5B41, 0x19},
	{0x5B42, 0x0F},
	{0x5B43, 0xE6},
	{0x5B44, 0x00},
	{0x5B45, 0x01},
	{0x5B46, 0x00},
	{0x5B47, 0x04},
	{0x5B48, 0x00},
	{0x5B49, 0xF7},
	{0x5B4A, 0x00},
	{0x5B4B, 0x05},
	{0x5B4C, 0x0F},
	{0x5B4D, 0xFD},
	{0x5B4E, 0x0F},
	{0x5B4F, 0xF0},
	{0x5B50, 0x01},
	{0x5B51, 0x12},
	{0x5B52, 0x01},
	{0x5B53, 0x1B},
	{0x5B54, 0x0F},
	{0x5B55, 0xE6},
	{0x5B56, 0x0F},
	{0x5B57, 0xFF},
	{0x5B58, 0x0F},
	{0x5B59, 0xFF},
	{0x5B5A, 0x00},
	{0x5B5B, 0xFD},
	{0x5B5C, 0x00},
	{0x5B5D, 0x04},
	{0x5B5E, 0x0F},
	{0x5B5F, 0xFA},
	{0x5B60, 0x0F},
	{0x5B61, 0xF6},
	{0x5B62, 0x01},
	{0x5B63, 0x10},
	{0x5B64, 0x01},
	{0x5B65, 0x3D},
	{0x5B66, 0x0F},
	{0x5B67, 0xC6},
	{0x5B68, 0x0F},
	{0x5B69, 0xFD},
	{0x5B6A, 0x00},
	{0x5B6B, 0x02},
	{0x5B6C, 0x00},
	{0x5B6D, 0xED},
	{0x5B6E, 0x00},
	{0x5B6F, 0x11},
	{0x5B70, 0x0F},
	{0x5B71, 0xFE},
	{0x5B72, 0x0F},
	{0x5B73, 0xE8},
	{0x5B74, 0x01},
	{0x5B75, 0x19},
	{0xffff, 0xff} //this register judge to stop
};

static int read_camera_ser_alias_id(struct camera_dev *cam_dev)
{
	int ret = -1;
	int retry_maxtimes = 10;
	u8 reg_value;

	while (retry_maxtimes-- > 0) {
		ret = bst_i2c_read_byte_data_word_reg(
			cam_dev->i2c_client->adapter, cam_dev->ser_alias_id,
			MAX_SER_DEVICE_ID, &reg_value);

		if (ret)
			usleep_range(1000, 2000);
		else if (ret == 0)
			break;
	}
	//modify ser_alias i2c address
	if (reg_value == (cam_dev->ser_alias_id << 1))
		return 0;

	return ret;
}

static int ox3f_parse_dts(struct camera_dev *cam_dev, struct device_node *node)
{
	if (of_property_read_s32(node, "sensor-fps", &cam_dev->sensor_fps)) {
		dev_err(cam_dev->dev,
			"Invalid DT sensor-fps\n");
		return -EINVAL;
	}
	return 0;
}

static int ox3f_ser_cfg(struct camera_dev *ox3f_raw)
{
	int i;
	int retry_times;
	int ret;
	struct i2c_adapter *adap;

	pr_info("ox3f %s(), line %d\n", __func__, __LINE__);

	if (ox3f_raw == NULL) {
		pr_info("%s : camera_dev is NULL\n", __func__);
		return -EINVAL;
	}

	adap = ox3f_raw->i2c_client->adapter;

	if (adap == NULL)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(config_serdes_setting); i++) {
		retry_times = 16;
		ret = -1;
		while (retry_times > 0 && ret) {
			ret = bst_i2c_write_byte_data_word_reg(
				adap, ox3f_raw->ser_alias_id,
				config_serdes_setting[i][0],
				config_serdes_setting[i][1]);
			if (ret) {
				pr_info(": write_MAX96717f_reg failed!\n");
				retry_times--;
				usleep_range(2000, 2500);
				continue;
			}
			pr_debug("write MAX96717f reg:%#x, val:%#x",
			       config_serdes_setting[i][0],
			       config_serdes_setting[i][1]);
		}
	}
#ifdef GUANG_ZHEN_GPIO_ANBLE
	ret = bst_i2c_write_byte_data_word_reg(adap, ox3f_raw->ser_alias_id,
					       0x2d3, GUANG_ZHEN_GPIO_ANBLE);
	if (ret)
		usleep_range(2000, 2500);
#endif
	return 0;
}

static int ox3f_power_on(struct camera_dev *cam_dev, struct device_node *node)
{
	int i, sensor_cfg_size;
	int ret;
	struct i2c_adapter *adap;
	struct ox3f_sensor_base_cfg *x3f_priv_setting;
	/*according to fps chose setting*/
	if (cam_dev->i2c_client == NULL)
		return -EINVAL;

	adap = cam_dev->i2c_client->adapter;
	if (adap == NULL)
		return -EINVAL;

	if (cam_dev->sensor_fps == 20) {
		x3f_priv_setting = sensor_20fps_settings;
		sensor_cfg_size = ARRAY_SIZE(sensor_20fps_settings);
	} else if (cam_dev->sensor_fps == 25) {
		x3f_priv_setting = sensor_25fps_settings;
		sensor_cfg_size = ARRAY_SIZE(sensor_25fps_settings);
	} else if (cam_dev->sensor_fps == 30) {
		x3f_priv_setting = sensor_30fps_settings;
		sensor_cfg_size = ARRAY_SIZE(sensor_30fps_settings);
	} else {
		dev_err(cam_dev->dev, "ox03f sensor, fps not support\n");
		return -EINVAL;
	}

	for (i = 0; i < sensor_cfg_size; i++) {
		ret = bst_i2c_write_byte_data_word_reg(
			adap, cam_dev->sensor_alias_id,
			x3f_priv_setting[i].reg,
			x3f_priv_setting[i].value);
		if (x3f_priv_setting[i].reg == 0x0107)
			usleep_range(10000, 20000);
		if (ret) {
			dev_err(cam_dev->dev, "ox03c sensor %x,%x write failed",
				x3f_priv_setting[i].reg,
				x3f_priv_setting[i].value);
			continue;
		}
	}
	i = 0;
	while ((x3f_ltm_exp_settings[i].reg != 0xffff) || (x3f_ltm_exp_settings[i].value != 0xff)) {
		ret = bst_i2c_write_byte_data_word_reg(
			adap, cam_dev->sensor_alias_id,
			x3f_ltm_exp_settings[i].reg,
			x3f_ltm_exp_settings[i].value);

		if (ret) {
			dev_err(cam_dev->dev, "hk_ox03c sensor %x,%x write failed",
				x3f_ltm_exp_settings[i].reg,
				x3f_ltm_exp_settings[i].value);
			continue;
		}
		i++;
	}
	return 0;
}

static int camera_s_power(struct v4l2_subdev *sd, int enable)
{
	int ret;
	struct camera_dev *ox3f_raw =
		container_of(sd, struct camera_dev, subdev);

	pr_info("ox3f %s(), line %d\n", __func__, __LINE__);

	if (!enable)
		return 0;

	ox3f_parse_dts(ox3f_raw, ox3f_raw->dev->of_node);
	if (ox3f_raw->sensor_fps == 25) {
		//write row time
		ox3f_raw->row_time = ROW_TIME_25M;
	}

	if (!ox3f_raw->maxim_power_on) {
		ox3f_raw->power_on = false;
		return -EINVAL;
	}

	if (!is_slave_soc_model(ox3f_raw)) {
		ret = ox3f_ser_cfg(ox3f_raw);

		if (ret)
			return ret;

		ret = read_camera_ser_alias_id(ox3f_raw);

		if (ret == 0)
			ox3f_power_on(ox3f_raw, ox3f_raw->dev->of_node);
	}
	ox3f_raw->power_on = true;

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

static const struct media_entity_operations ox3f_raw_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

// sensor is not sured powered on this time
// don't check camera connect here
static int ox3f_raw_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct camera_dev *ox3f_raw;
	struct device *dev = &client->dev;
	struct resource res;
	int bus_index = 0;
	int ret;

	ox3f_raw = devm_kzalloc(&client->dev, sizeof(struct camera_dev),
				GFP_KERNEL);
	if (!ox3f_raw)
		return -ENOMEM;

	if (client->adapter == NULL) {
		pr_err("client->adapter == NULL, error\n");
		return -1;
	}
	ox3f_raw->i2c_client = client;
	ox3f_raw->dev = dev;
	bus_index = client->adapter->nr;

	if (of_address_to_resource(client->adapter->dev.of_node, 0, &res)) {
		dev_err(dev, "Can not get adapter's address\n'");
		return -1;
	}
	ox3f_raw->isp_data.i2cRegBase = res.start;
	ox3f_raw->isp_data.sensorRdWrMode =
		BST_SENSOR_RW_MODE_WORD_REG_WORD_DATA;
	ox3f_raw->isp_data.sensorType = BST_SENSOR_TYPE_OX3F_RAW;

	ret = parse_camera_endpoint(ox3f_raw, dev->of_node);
	if (ret) {
		dev_err(dev, "init_camera_dev error, ret = %d\n", ret);
		return ret;
	}

	ret = init_camera_dev(ox3f_raw, &camera_ops, &ox3f_raw_media_ops);
	if (ret) {
		dev_err(dev, "init_camera_dev error, ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static void ox3f_raw_remove(struct i2c_client *client)
{
}

static const struct i2c_device_id ox3f_raw_id[] = {
	{ MODULE_NAME, 0 },
	{},
};

static const struct of_device_id ox3f_raw_of_match[] = {
	{
		.compatible = MODULE_NAME,
	},
	{},
};

MODULE_DEVICE_TABLE(of, ox3f_raw_of_match);
MODULE_DEVICE_TABLE(i2c, ox3f_raw_id);

static struct i2c_driver ox3f_raw_driver = {
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(ox3f_raw_of_match),
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe		= ox3f_raw_probe,
	.remove		= ox3f_raw_remove,
	.id_table	= ox3f_raw_id,
};

module_i2c_driver(ox3f_raw_driver);

MODULE_DESCRIPTION("OX3F RAW camera driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("BST Ltd.");
