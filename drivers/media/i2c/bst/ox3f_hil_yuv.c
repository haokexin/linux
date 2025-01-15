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
#include "camera_common_op.h"

#define MAX96717f_I2CADDR		0x40

#define MODULE_NAME "bst,ox3f-yuv-hil"
#define MAX_SER_DEVICE_ID 0x0000
#define ROW_TIME_25M 49019

/*vts must be same with the value of (0x380e,0x380f),row_time = 10000000000/fps/vts*/
//max96717f
static unsigned short config_serdes_setting[][2] = {
	//x3f framesync triggered by deserelizer
	{0x02d3, 0x00},
	{0x0002, 0xf3},
	{0x0383, 0x00},
	{0x0318, 0x5e}, //bit6 enable DT select, bit0~5 DT
	{0x03f1, 0x09},
	{0x03f0, 0x51},
	{0x0570, 0x1c},
	{0x0570, 0x0c},
	{0x0006, 0xb0},
	{0x0041, 0x76},
	{0x02bf, 0x60},
	{0x02be, 0x90},
	{0x02d6, 0x84}, //notify
	{0x02d3, 0x90},
	{0x02cd, 0x12}, //guang zhen 0x12 ,sunyu 0x00
	{0x02be, 0x84}};

static int ox3f_parse_dts(struct camera_dev *cam_dev, struct device_node *node)
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

static int ox3f_ser_cfg(struct camera_dev *ox3f_yuv)
{
	int i;
	int retry_times;
	int ret;
	struct i2c_adapter *adap;

	pr_info("ox3f %s(), line %d\n", __func__, __LINE__);

	if (ox3f_yuv == NULL) {
		pr_info("%s : camera_dev is NULL\n", __func__);
		return -EINVAL;
	}
	adap = ox3f_yuv->i2c_client->adapter;

	if (adap == NULL)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(config_serdes_setting); i++) {
		retry_times = 12;
		ret = -1;
		while (retry_times > 0 && ret) {
			ret = bst_i2c_write_byte_data_word_reg(
				adap, ox3f_yuv->ser_alias_id,
				config_serdes_setting[i][0],
				config_serdes_setting[i][1]);
			if (ret) {
				pr_info(": write_max96717f_reg failed!\n");
				retry_times--;
				usleep_range(2000, 2500);
				continue;
			}
			pr_info("write max96717f reg:%#x, val:%#x",
				   config_serdes_setting[i][0],
				   config_serdes_setting[i][1]);
		}
	}
	if (ret)
		pr_info("ox3f ser_cfg write failed!\n");
	return 0;
}

static int camera_s_power(struct v4l2_subdev *sd, int enable)
{
	struct camera_dev *ox3f_yuv =
		container_of(sd, struct camera_dev, subdev);

	if (!enable)
		return 0;

	ox3f_parse_dts(ox3f_yuv, ox3f_yuv->dev->of_node);
	if (!ox3f_yuv->maxim_power_on) {
		ox3f_yuv->power_on = false;
		return -EINVAL;
	}

	ox3f_ser_cfg(ox3f_yuv);

	ox3f_yuv->power_on = true;

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

static const struct media_entity_operations ox3f_yuv_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

// sensor is not sured powered on this time
// don't check camera connect here
static int ox3f_yuv_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct camera_dev *ox3f_yuv;
	struct device *dev = &client->dev;
	struct resource res;
	int bus_index = 0;
	int ret;

	ox3f_yuv = devm_kzalloc(&client->dev, sizeof(struct camera_dev),
				GFP_KERNEL);
	if (!ox3f_yuv)
		return -ENOMEM;

	if (client->adapter == NULL) {
		pr_err("client->adapter == NULL, error\n");
		return -1;
	}
	ox3f_yuv->i2c_client = client;
	ox3f_yuv->dev = dev;
	bus_index = client->adapter->nr;

	if (of_address_to_resource(client->adapter->dev.of_node, 0, &res)) {
		dev_err(dev, "Can not get adapter's address\n'");
		return -1;
	}
	ox3f_yuv->isp_data.i2cRegBase = res.start;
	ox3f_yuv->isp_data.sensorRdWrMode =
		BST_SENSOR_RW_MODE_WORD_REG_WORD_DATA;
	ox3f_yuv->isp_data.sensorType = BST_SENSOR_TYPE_YUV422;

	ret = parse_camera_endpoint(ox3f_yuv, dev->of_node);
	if (ret) {
		dev_err(dev, "parse_camera_dev error, ret = %d\n", ret);
		return ret;
	}

	ret = init_camera_dev(ox3f_yuv, &camera_ops, &ox3f_yuv_media_ops);
	if (ret) {
		dev_err(dev, "init_camera_dev error, ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static void ox3f_yuv_remove(struct i2c_client *client)
{
}

static const struct i2c_device_id ox3f_yuv_id[] = {
	{ MODULE_NAME, 0 },
	{},
};

static const struct of_device_id ox3f_yuv_of_match[] = {
	{.compatible = MODULE_NAME,
	 .data = NULL},
	{},
};

MODULE_DEVICE_TABLE(of, ox3f_yuv_of_match);
MODULE_DEVICE_TABLE(i2c, ox3f_yuv_id);

static struct i2c_driver ox3f_yuv_driver = {
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(ox3f_yuv_of_match),
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe		= ox3f_yuv_probe,
	.remove		= ox3f_yuv_remove,
	.id_table	= ox3f_yuv_id,
};

module_i2c_driver(ox3f_yuv_driver);

MODULE_DESCRIPTION("OX3F YUV camera driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("BST Ltd.");
