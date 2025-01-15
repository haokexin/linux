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
#include "ox3c_config.h"
#include "camera_common_op.h"
#include "common_deser_hub.h"

#define MAX96717f_I2CADDR		0x40

#define MODULE_NAME "bst,ox3c"
#define MAX_SER_DEVICE_ID 0x0000
#define ROW_TIME_30FPS  48239
#define ROW_TIME_20FPS  72886
#define ROW_TIME_60FPS  24190
#define MODULE_NAME_GZ "bst,guangzhen_ox3c"
#define MODULE_NAME_HK "bst,hk_ox3c"
#define MODULE_NAME_OF "bst,ofilm_ox3c"
#define MODULE_NAME_SY "bst,sy_ox3c"
#define MODULE_NAME_MS "bst,ms_ox3c"

/*vts must be same with the value of (0x380e,0x380f),row_time = 10000000000/fps/vts*/
//#define GUANG_ZHEN_GPIO_ANBLE 0X12 //jinghua x8b 0x00,oufei x3c 0x00,maxieye x3c 0x00 ,sunyu x3c 0x00 ,guangzhen x3c 0x12
//guangzhen_x3c fov100,hk_x3c fov100,oflim_x3c fov100
static struct ox3c_sensor_base_cfg x3c_lfm_exp_settings[] = {
	{0x5d46, 0x00},
	{0x5d47, 0xe0},
	{0x5d48, 0x00},
	{0x5d49, 0xe8},
	{0xffff, 0xff} //the register to judge to stop
};

static struct ox3c_sensor_base_cfg hk_x3c_lfm_exp_settings[] = {
	//lfm setting
	{0x5d46, 0x00},
	{0x5d47, 0xe0},
	{0x5d48, 0x00},
	{0x5d49, 0xe8},
	//preccm
	{0x5003, 0x7a},
	{0x5b7e, 0x01},
	//D light
	//nCT
	{0x5B7c, 0x01},
	{0x5B7d, 0x61},
	{0x5B40, 0x01},
	{0x5B41, 0x39},
	{0x5B42, 0x0F},
	{0x5B43, 0xD6},
	{0x5B44, 0x0F},
	{0x5B45, 0xF1},
	{0x5B46, 0x00},
	{0x5B47, 0x0F},
	{0x5B48, 0x01},
	{0x5B49, 0x01},
	{0x5B4A, 0x0F},
	{0x5B4B, 0xF1},
	{0x5B4C, 0x0F},
	{0x5B4D, 0xFE},
	{0x5B4E, 0x0F},
	{0x5B4F, 0xF9},
	{0x5B50, 0x01},
	{0x5B51, 0x09},
	//C light
	//nCT
	{0x5B7a, 0x00},
	{0x5B7b, 0xF1},
	{0x5B52, 0x01},
	{0x5B53, 0x1E},
	{0x5B54, 0x00},
	{0x5B55, 0x00},
	{0x5B56, 0x0F},
	{0x5B57, 0xE2},
	{0x5B58, 0x0F},
	{0x5B59, 0xED},
	{0x5B5A, 0x01},
	{0x5B5B, 0x2F},
	{0x5B5C, 0x0F},
	{0x5B5D, 0xE5},
	{0x5B5E, 0x0F},
	{0x5B5F, 0xE9},
	{0x5B60, 0x00},
	{0x5B61, 0x0C},
	{0x5B62, 0x01},
	{0x5B63, 0x0B},
	//A light
	//nCT
	{0x5B78, 0x00},
	{0x5B79, 0x53},
	////
	{0x5B64, 0x01},
	{0x5B65, 0x73},
	{0x5B66, 0x0F},
	{0x5B67, 0xC1},
	{0x5B68, 0x0F},
	{0x5B69, 0xCC},
	{0x5B6A, 0x0F},
	{0x5B6B, 0xFA},
	{0x5B6C, 0x01},
	{0x5B6D, 0x0D},
	{0x5B6E, 0x0F},
	{0x5B6F, 0xF9},
	{0x5B70, 0x0F},
	{0x5B71, 0xFE},
	{0x5B72, 0x0F},
	{0x5B73, 0xDF},
	{0x5B74, 0x01},
	{0x5B75, 0x23},
	{0xffff, 0xff}
};

//sy_x3c fov100
static struct ox3c_sensor_base_cfg sy_x3c_settings[] = {
	{0x5d46, 0x00},
	{0x5d47, 0xe0},
	{0x5d48, 0x00},
	{0x5d49, 0xe8},

	//preccm
	{0x5003, 0x7a},
	{0x5b7e, 0x01},

	//D light
	//nCT
	{0x5B7c, 0x01},
	{0x5B7d, 0x61},

	{0x5B40, 0x01},
	{0x5B41, 0x39},
	{0x5B42, 0x0F},
	{0x5B43, 0xD6},
	{0x5B44, 0x0F},
	{0x5B45, 0xF1},
	{0x5B46, 0x00},
	{0x5B47, 0x0F},
	{0x5B48, 0x01},
	{0x5B49, 0x01},
	{0x5B4A, 0x0F},
	{0x5B4B, 0xF1},
	{0x5B4C, 0x0F},
	{0x5B4D, 0xFE},
	{0x5B4E, 0x0F},
	{0x5B4F, 0xF9},
	{0x5B50, 0x01},
	{0x5B51, 0x09},
	//C light
	//nCT
	{0x5B7a, 0x00},
	{0x5B7b, 0xF1},

	{0x5B52, 0x01},
	{0x5B53, 0x1E},
	{0x5B54, 0x00},
	{0x5B55, 0x00},
	{0x5B56, 0x0F},
	{0x5B57, 0xE2},
	{0x5B58, 0x0F},
	{0x5B59, 0xED},
	{0x5B5A, 0x01},
	{0x5B5B, 0x2F},
	{0x5B5C, 0x0F},
	{0x5B5D, 0xE5},
	{0x5B5E, 0x0F},
	{0x5B5F, 0xE9},
	{0x5B60, 0x00},
	{0x5B61, 0x0C},
	{0x5B62, 0x01},
	{0x5B63, 0x0B},
	//A light
	//nCT
	{0x5B78, 0x00},
	{0x5B79, 0x53},
	////
	{0x5B64, 0x01},
	{0x5B65, 0x73},
	{0x5B66, 0x0F},
	{0x5B67, 0xC1},
	{0x5B68, 0x0F},
	{0x5B69, 0xCC},
	{0x5B6A, 0x0F},
	{0x5B6B, 0xFA},
	{0x5B6C, 0x01},
	{0x5B6D, 0x0D},
	{0x5B6E, 0x0F},
	{0x5B6F, 0xF9},
	{0x5B70, 0x0F},
	{0x5B71, 0xFE},
	{0x5B72, 0x0F},
	{0x5B73, 0xDF},
	{0x5B74, 0x01},
	{0x5B75, 0x23},
	{0xffff, 0xff} //the register to judge to stop
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

static int ox3c_parse_dts(struct camera_dev *cam_dev, struct device_node *node)
{
	int sensor_fps;

	if (of_property_read_s32(node, "sensor-fps", &sensor_fps)) {
		dev_err(cam_dev->dev,
			"Invalid DT sensor-fps\n");
		return -EINVAL;
	}
	cam_dev->sensor_fps = sensor_fps;
	return 0;
}

static int ox3c_ser_cfg(struct camera_dev *ox3c_raw)
{
	int i;
	int retry_times;
	int ret;
	struct i2c_adapter *adap;

	if (ox3c_raw == NULL) {
		pr_info("%s : camera_dev is NULL\n", __func__);
		return -EINVAL;
	}

	adap = ox3c_raw->i2c_client->adapter;

	if (adap == NULL)
		return -EINVAL;

	if (ox3c_raw->deser_parent->type == DESER_TYPE_MAX96726) {
		pr_info("serdes config for 96726\n");
		for (i = 0; i < ARRAY_SIZE(config_serdes_adt_96726); i++) {
			retry_times = 16;
			ret = -1;
			while (retry_times > 0 && ret) {
				ret = bst_i2c_write_byte_data_word_reg(
					adap, ox3c_raw->ser_alias_id,
					config_serdes_adt_96726[i][0],
					config_serdes_adt_96726[i][1]);
				if (ret) {
					pr_info(": write_MAX96717f_reg failed!\n");
					retry_times--;
					usleep_range(2000, 2500);
					continue;
				}
			}
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(config_serdes_setting); i++) {
			retry_times = 16;
			ret = -1;
			while (retry_times > 0 && ret) {
				ret = bst_i2c_write_byte_data_word_reg(
					adap, ox3c_raw->ser_alias_id,
					config_serdes_setting[i][0],
					config_serdes_setting[i][1]);
				if (ret) {
					pr_info(": write_MAX96717f_reg failed!\n");
					retry_times--;
					usleep_range(2000, 2500);
					continue;
				}
			}
		}
	}


	// 20fps use 2lane
	if (ox3c_raw->sensor_fps == 20) {
		ret = bst_i2c_write_byte_data_word_reg(
		adap, ox3c_raw->ser_alias_id,
		0x0331,
		0x10);
	}
#ifdef GUANG_ZHEN_GPIO_ANBLE
	ret = bst_i2c_write_byte_data_word_reg(adap, ox3c_raw->ser_alias_id,
						   0x2d3, GUANG_ZHEN_GPIO_ANBLE);
	if (ret) {
		pr_info(": write_MAX96717f_reg failed!\n");
		usleep_range(2000, 2500);
	}
#endif
	return 0;
}

static int open_sensor_stream(struct camera_dev *cam_dev)
{
	int ret = 0;
	struct i2c_adapter *adap;

	adap = cam_dev->i2c_client->adapter;
	ret = bst_i2c_write_byte_data_word_reg(
		adap, cam_dev->sensor_alias_id,
		0x0100,
		0x01);
	if (ret) {
		dev_err(cam_dev->dev, "ox03c sensor %x,%x write failed", 0x0100, 0x01);
		return -1;
	}
	return 0;
}

static int ox3c_power_on(struct camera_dev *cam_dev, struct device_node *node)
{
	int i, sensor_cfg_size;
	int ret;
	struct i2c_adapter *adap;
	struct ox3c_sensor_base_cfg *x3c_priv_setting;
	const struct ox3c_sensor_base_cfg *x3c_ltm_exp_setting;
	int write_fail_times;

	/*according to fps chose setting*/
	if (cam_dev->i2c_client == NULL)
		return -EINVAL;

	adap = cam_dev->i2c_client->adapter;
	if (adap == NULL)
		return -EINVAL;

	if (cam_dev->sensor_fps == 20) {
		x3c_priv_setting = sensor_20fps_settings;
		sensor_cfg_size = ARRAY_SIZE(sensor_20fps_settings);
	} else if (cam_dev->sensor_fps == 30) {
		x3c_priv_setting = sensor_30fps_settings;
		sensor_cfg_size = ARRAY_SIZE(sensor_30fps_settings);
	} else if (cam_dev->sensor_fps == 40) {
		x3c_priv_setting = sensor_40fps_settings;
		sensor_cfg_size = ARRAY_SIZE(sensor_40fps_settings);
	} else if (cam_dev->sensor_fps == 60) {
		x3c_priv_setting = sensor_60fps_settings;
		sensor_cfg_size = ARRAY_SIZE(sensor_60fps_settings);
	} else {
		dev_err(cam_dev->dev, "%s, ox3c's sensor_fps not support yet", __func__);
		return 0;
	}

	write_fail_times = 0;
	for (i = 0; i < sensor_cfg_size; i++) {
		ret = bst_i2c_write_byte_data_word_reg(
			adap, cam_dev->sensor_alias_id,
			x3c_priv_setting[i].reg,
			x3c_priv_setting[i].value);
		if (x3c_priv_setting[i].reg == 0x0107)
			usleep_range(10000, 20000);

		if (ret) {
			dev_err(cam_dev->dev, "ox03c sensor %x,%x write failed",
				x3c_priv_setting[i].reg,
				x3c_priv_setting[i].value);
			if (++write_fail_times > 100)
				break;
			continue;
		}
	}

	x3c_ltm_exp_setting = of_device_get_match_data(cam_dev->dev);
	if (x3c_ltm_exp_setting == NULL) {
		//this no need to write ltm_exp setting;
		return 0;
	}
	i = 0;
	while ((x3c_ltm_exp_setting[i].reg != 0xffff) || (x3c_ltm_exp_setting[i].value != 0xff)) {
		ret = bst_i2c_write_byte_data_word_reg(
			adap, cam_dev->sensor_alias_id,
			x3c_ltm_exp_setting[i].reg,
			x3c_ltm_exp_setting[i].value);

		if (ret) {
			dev_err(cam_dev->dev, "hk_ox03c sensor %x,%x write failed",
				x3c_ltm_exp_setting[i].reg,
				x3c_ltm_exp_setting[i].value);
			i++;
			continue;
		}
		i++;
	}
	open_sensor_stream(cam_dev);
	return 0;
	return 0;
}

static int camera_s_power(struct v4l2_subdev *sd, int enable)
{
	int ret;
	struct camera_dev *ox3c_raw =
		container_of(sd, struct camera_dev, subdev);

	if (!enable)
		return 0;

	ox3c_parse_dts(ox3c_raw, ox3c_raw->dev->of_node);
	if (ox3c_raw->sensor_fps == 30) {
		//write row time
		ox3c_raw->row_time = ROW_TIME_30FPS;
	} else if (ox3c_raw->sensor_fps == 20) {
		//write row time
		ox3c_raw->row_time = ROW_TIME_20FPS;
	} else if (ox3c_raw->sensor_fps == 60) {
		ox3c_raw->row_time = ROW_TIME_60FPS;
	}

	if (!ox3c_raw->maxim_power_on) {
		ox3c_raw->power_on = false;
		return -EINVAL;
	}

	if (!is_slave_soc_model(ox3c_raw)) {
		ret = ox3c_ser_cfg(ox3c_raw);

		if (ret)
			return ret;
		ret = read_camera_ser_alias_id(ox3c_raw);
		if (ret == 0)
			ox3c_power_on(ox3c_raw, ox3c_raw->dev->of_node);
	}
	ox3c_raw->power_on = true;

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

static const struct media_entity_operations ox3c_raw_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

// sensor is not sured powered on this time
// don't check camera connect here
static int ox3c_raw_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct camera_dev *ox3c_raw;
	struct device *dev = &client->dev;
	struct resource res;
	int bus_index = 0;
	int ret;

	ox3c_raw = devm_kzalloc(&client->dev, sizeof(struct camera_dev),
				GFP_KERNEL);
	if (!ox3c_raw)
		return -ENOMEM;

	if (client->adapter == NULL) {
		pr_err("client->adapter == NULL, error\n");
		return -1;
	}
	ox3c_raw->i2c_client = client;
	ox3c_raw->dev = dev;
	bus_index = client->adapter->nr;

	if (of_address_to_resource(client->adapter->dev.of_node, 0, &res)) {
		dev_err(dev, "Can not get adapter's address\n'");
		return -1;
	}
	ox3c_raw->isp_data.i2cRegBase = res.start;
	ox3c_raw->isp_data.sensorRdWrMode =
		BST_SENSOR_RW_MODE_WORD_REG_WORD_DATA;
	ox3c_raw->isp_data.sensorType = BST_SENSOR_TYPE_OX3C_RAW;

	ret = parse_camera_endpoint(ox3c_raw, dev->of_node);
	if (ret) {
		dev_err(dev, "init_camera_dev error, ret = %d\n", ret);
		return ret;
	}

	ret = init_camera_dev(ox3c_raw, &camera_ops, &ox3c_raw_media_ops);
	if (ret) {
		dev_err(dev, "init_camera_dev error, ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static void ox3c_raw_remove(struct i2c_client *client)
{
}

static const struct i2c_device_id ox3c_raw_id[] = {
	{ MODULE_NAME, 0 },
	{},
};

static const struct of_device_id ox3c_raw_of_match[] = {
	{.compatible = MODULE_NAME_GZ, .data = x3c_lfm_exp_settings},
	{.compatible = MODULE_NAME_HK, .data = hk_x3c_lfm_exp_settings},
	{.compatible = MODULE_NAME_OF, .data = x3c_lfm_exp_settings},
	{.compatible = MODULE_NAME_SY, .data = sy_x3c_settings},
	{.compatible = MODULE_NAME_MS, .data = NULL},
	{},
};

MODULE_DEVICE_TABLE(of, ox3c_raw_of_match);
MODULE_DEVICE_TABLE(i2c, ox3c_raw_id);

static struct i2c_driver ox3c_raw_driver = {
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(ox3c_raw_of_match),
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe		= ox3c_raw_probe,
	.remove		= ox3c_raw_remove,
	.id_table	= ox3c_raw_id,
};

module_i2c_driver(ox3c_raw_driver);

MODULE_DESCRIPTION("OX3C RAW camera driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("BST Ltd.");
