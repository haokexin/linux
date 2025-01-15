// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/media-entity.h>
#include <media/media-device.h>

#include "ti_deser_hub.h"
#include "camera_common_op.h"

#define MODULE_NAME "bst,ov10640-yuv"

static int camera_s_power(struct v4l2_subdev *sd, int enable)
{
	struct camera_dev *camera_ov10640;
	int ret = 0;

	if (!enable)
		return 0;


	camera_ov10640 = container_of(sd, struct camera_dev, subdev);
	if (camera_ov10640->i2c_client == NULL)
		return -EINVAL;

	ret = is_camera_connected(camera_ov10640);
	if (!ret)
		return -EINVAL;

	camera_ov10640->power_on = true;

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Operations
 */
static const struct v4l2_subdev_core_ops camera_core_ops = {
	.s_power = camera_s_power,
};

static const struct v4l2_subdev_ops camera_ops = {
	.core = &camera_core_ops,
};

/* -----------------------------------------------------------------------------
 * Media Operations
 */

static const struct media_entity_operations ov10640_isp_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

// sensor is not sured powered on this time
// don't check camera connect here
static int ov10640_isp_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct camera_dev *camera_ov10640;
	struct device *dev = &client->dev;
	int ret;
	int bus_index;

	camera_ov10640 = devm_kzalloc(&client->dev, sizeof(struct camera_dev),
				     GFP_KERNEL);
	if (!camera_ov10640)
		return -ENOMEM;

	if (client->adapter == NULL) {
		pr_err("client->adapter == NULL, error\n");
		return -1;
	}

	camera_ov10640->i2c_client = client;
	camera_ov10640->dev = dev;
	bus_index = client->adapter->nr;
	camera_ov10640->isp_data.i2cRegBase = bus_index * IIC_OFFSET + IIC_BASE;
	camera_ov10640->isp_data.sensorRdWrMode = BST_SENSOR_RW_MODE_BYTE_REG_BYTE_DATA;
	camera_ov10640->isp_data.sensorType = BST_SENSOR_TYPE_YUV422;

	ret = parse_camera_endpoint(camera_ov10640, dev->of_node);
	if (ret) {
		dev_err(dev, "parse_camera_endpoint error, ret = %d\n", ret);
		return ret;
	}

	ret = init_camera_dev(camera_ov10640, &camera_ops
		, &ov10640_isp_media_ops);
	if (ret) {
		dev_err(dev, "init_camera_dev error, ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static void ov10640_isp_remove(struct i2c_client *client)
{
}

static const struct i2c_device_id ov10640_isp_id[] = {
	{ MODULE_NAME, 0 },
	{},
};

static const struct of_device_id ov10640_isp_of_match[] = {
	{
		.compatible = MODULE_NAME,
	},
	{},
};

MODULE_DEVICE_TABLE(of, ov10640_isp_of_match);
MODULE_DEVICE_TABLE(i2c, ov10640_isp_id);

static struct i2c_driver ov10640_isp_driver = {
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(ov10640_isp_of_match),
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = ov10640_isp_probe,
	.remove = ov10640_isp_remove,
	.id_table = ov10640_isp_id,
};

module_i2c_driver(ov10640_isp_driver);

MODULE_DESCRIPTION("OV10640 YUV camera driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("BST Ltd.");
