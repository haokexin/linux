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

#define MODULE_NAME "bst,ox08b-yuv-hil"
#define MAX_SER_DEVICE_ID 0x0000
#define ROW_TIME_30FPS  48239
#define ROW_TIME_20FPS  72886
/*vts must be same with the value of (0x380e,0x380f),row_time = 10000000000/fps/vts*/

static unsigned short config_serdes_setting[][2] = {
	{0x0002, 0x43}, //enable all pipe
	{0x0330, 0x00}, //
	{0x0331, 0x33},
	{0x0332, 0xee},
	{0x0333, 0xe4},
	{0x0308, 0x64},
	{0x0311, 0x40},
	{0x0318, 0x5e}, // PipeZ: Datatype 0x1e yuv-8bit
	{0x02d3, 0x84},//for pull up jinghua x8b
};

static int x8b_parse_dts(struct camera_dev *cam_dev, struct device_node *node)
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
	ret = bst_i2c_write_byte_data_word_reg(adap, ox08b_raw->ser_alias_id, 0x0042,
					 (ox08b_raw->sensor_alias_id << 1));
	if (ret)
		pr_info("ox08b ser_cfg write failed!\n");
	ret = bst_i2c_write_byte_data_word_reg(adap, ox08b_raw->ser_alias_id, 0x0043,
					 0x6c);
	if (ret)
		pr_info("ox08b ser_cfg write failed!\n");
	return 0;
}

static int camera_s_power(struct v4l2_subdev *sd, int enable)
{
	struct camera_dev *x8b_yuv =
		container_of(sd, struct camera_dev, subdev);

	if (!enable)
		return 0;

	x8b_parse_dts(x8b_yuv, x8b_yuv->dev->of_node);
	if (!x8b_yuv->maxim_power_on) {
		x8b_yuv->power_on = false;
		return -EINVAL;
	}
	ox08b_ser_cfg(x8b_yuv);

	x8b_yuv->power_on = true;

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

static const struct media_entity_operations x8b_yuv_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

// sensor is not sured powered on this time
// don't check camera connect here
static int x8b_yuv_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct camera_dev *x8b_yuv;
	struct device *dev = &client->dev;
	struct resource res;
	int bus_index = 0;
	int ret;

	x8b_yuv = devm_kzalloc(&client->dev, sizeof(struct camera_dev),
				GFP_KERNEL);
	if (!x8b_yuv)
		return -ENOMEM;

	if (client->adapter == NULL) {
		pr_err("client->adapter == NULL, error\n");
		return -1;
	}
	x8b_yuv->i2c_client = client;
	x8b_yuv->dev = dev;
	bus_index = client->adapter->nr;

	if (of_address_to_resource(client->adapter->dev.of_node, 0, &res)) {
		dev_err(dev, "Can not get adapter's address\n'");
		return -1;
	}

	x8b_yuv->isp_data.i2cRegBase = res.start;
	x8b_yuv->isp_data.sensorRdWrMode =
		BST_SENSOR_RW_MODE_WORD_REG_WORD_DATA;
	x8b_yuv->isp_data.sensorType = BST_SENSOR_TYPE_YUV422;

	ret = parse_camera_endpoint(x8b_yuv, dev->of_node);
	if (ret) {
		dev_err(dev, "parse_camera_dev error, ret = %d\n", ret);
		return ret;
	}

	ret = init_camera_dev(x8b_yuv, &camera_ops, &x8b_yuv_media_ops);
	if (ret) {
		dev_err(dev, "init_camera_dev error, ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static void x8b_yuv_remove(struct i2c_client *client)
{
}

static const struct i2c_device_id x8b_yuv_id[] = {
	{ MODULE_NAME, 0 },
	{},
};

static const struct of_device_id x8b_yuv_of_match[] = {
	{
		.compatible = MODULE_NAME,
	},
	{},
};

MODULE_DEVICE_TABLE(of, x8b_yuv_of_match);
MODULE_DEVICE_TABLE(i2c, x8b_yuv_id);

static struct i2c_driver x8b_yuv_driver = {
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(x8b_yuv_of_match),
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe		= x8b_yuv_probe,
	.remove		= x8b_yuv_remove,
	.id_table	= x8b_yuv_id,
};

module_i2c_driver(x8b_yuv_driver);

MODULE_DESCRIPTION("OX08B YUV camera driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("BST Ltd.");
