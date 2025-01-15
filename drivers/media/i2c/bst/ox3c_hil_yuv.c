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
// #include "../../platform/bst-a1000/isp_core.h"

#define MAX96717f_I2CADDR		0x40

#define MODULE_NAME "bst,ox3c-yuv-hil"
#define MAX_SER_DEVICE_ID 0x0000
#define ROW_TIME_30FPS  48239
#define ROW_TIME_20FPS  72886
/*vts must be same with the value of (0x380e,0x380f),row_time = 10000000000/fps/vts*/

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

static int camera_s_power(struct v4l2_subdev *sd, int enable)
{
	struct camera_dev *ox3c_yuv =
		container_of(sd, struct camera_dev, subdev);

	if (!enable)
		return 0;

	ox3c_parse_dts(ox3c_yuv, ox3c_yuv->dev->of_node);
	if (!ox3c_yuv->maxim_power_on) {
		ox3c_yuv->power_on = false;
		return -EINVAL;
	}

	ox3c_yuv->power_on = true;

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

static const struct media_entity_operations ox3c_yuv_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

// sensor is not sured powered on this time
// don't check camera connect here
static int ox3c_yuv_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct camera_dev *ox3c_yuv;
	struct device *dev = &client->dev;
	struct resource res;
	int bus_index = 0;
	int ret;

	ox3c_yuv = devm_kzalloc(&client->dev, sizeof(struct camera_dev),
				GFP_KERNEL);
	if (!ox3c_yuv)
		return -ENOMEM;

	if (client->adapter == NULL) {
		pr_err("client->adapter == NULL, error\n");
		return -1;
	}
	ox3c_yuv->i2c_client = client;
	ox3c_yuv->dev = dev;
	bus_index = client->adapter->nr;

	if (of_address_to_resource(client->adapter->dev.of_node, 0, &res)) {
		dev_err(dev, "Can not get adapter's address\n'");
		return -1;
	}
	ox3c_yuv->isp_data.i2cRegBase = res.start;
	ox3c_yuv->isp_data.sensorRdWrMode =
		BST_SENSOR_RW_MODE_WORD_REG_WORD_DATA;
	ox3c_yuv->isp_data.sensorType = BST_SENSOR_TYPE_YUV422;

	ret = parse_camera_endpoint(ox3c_yuv, dev->of_node);
	if (ret) {
		dev_err(dev, "parse_camera_dev error, ret = %d\n", ret);
		return ret;
	}

	ret = init_camera_dev(ox3c_yuv, &camera_ops, &ox3c_yuv_media_ops);
	if (ret) {
		dev_err(dev, "init_camera_dev error, ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static void ox3c_yuv_remove(struct i2c_client *client)
{
}

static const struct i2c_device_id ox3c_yuv_id[] = {
	{ MODULE_NAME, 0 },
	{},
};

static const struct of_device_id ox3c_yuv_of_match[] = {
	{.compatible = MODULE_NAME, .data = NULL},
	{},
};

MODULE_DEVICE_TABLE(of, ox3c_yuv_of_match);
MODULE_DEVICE_TABLE(i2c, ox3c_yuv_id);

static struct i2c_driver ox3c_yuv_driver = {
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(ox3c_yuv_of_match),
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe		= ox3c_yuv_probe,
	.remove		= ox3c_yuv_remove,
	.id_table	= ox3c_yuv_id,
};

module_i2c_driver(ox3c_yuv_driver);

MODULE_DESCRIPTION("OX3C YUV camera driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("BST Ltd.");
