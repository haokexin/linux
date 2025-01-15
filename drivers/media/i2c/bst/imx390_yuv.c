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

#define MODULE_NAME "bst,imx390-yuv"

static int camera_s_power(struct v4l2_subdev *sd, int enable)
{
	struct camera_dev *camera_imx390;
	int ret = 0;

	if (!enable)
		return 0;

	camera_imx390 = container_of(sd, struct camera_dev, subdev);
	if (camera_imx390->i2c_client == NULL)
		return -EINVAL;

	ret = is_camera_connected(camera_imx390);
	if (!ret)
		return -EINVAL;

	if (!is_slave_soc_model(camera_imx390)) {
		//imx390-isp sensor only support 30 Hz for normal output
		/*
		 *	senyun imx390-isp TI953
		 *		gpio 0 : Frame Sync
		 *		gpio 1 : ISP Reset
		 *	enable  gpo0 gpo1 gpi0
		 *	disable gpi1
		 */
		bst_i2c_write_byte_data_byte_reg(camera_imx390->i2c_client->adapter,
			camera_imx390->ser_alias_id, 0x0e, 0x3e);
	}

	camera_imx390->power_on = true;

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

static const struct media_entity_operations imx390_isp_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

// sensor is not sured powered on this time
// don't check camera connect here
static int imx390_isp_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct camera_dev *camera_imx390;
	struct device *dev = &client->dev;
	int ret;
	int bus_index;

	camera_imx390 = devm_kzalloc(&client->dev, sizeof(struct camera_dev),
				     GFP_KERNEL);
	if (!camera_imx390)
		return -ENOMEM;

	if (client->adapter == NULL) {
		pr_err("client->adapter == NULL, error\n");
		return -1;
	}

	camera_imx390->i2c_client = client;
	camera_imx390->dev = dev;
	bus_index = client->adapter->nr;
	camera_imx390->isp_data.i2cRegBase = bus_index * IIC_OFFSET + IIC_BASE;
	camera_imx390->isp_data.sensorRdWrMode = BST_SENSOR_RW_MODE_BYTE_REG_BYTE_DATA;
	camera_imx390->isp_data.sensorType = BST_SENSOR_TYPE_YUV422;

	ret = parse_camera_endpoint(camera_imx390, dev->of_node);
	if (ret) {
		dev_err(dev, "parse_camera_endpoint error, ret = %d\n", ret);
		return ret;
	}

	ret = init_camera_dev(camera_imx390, &camera_ops
		, &imx390_isp_media_ops);
	if (ret) {
		dev_err(dev, "init_camera_dev error, ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static void imx390_isp_remove(struct i2c_client *client)
{
}

static const struct i2c_device_id imx390_isp_id[] = {
	{ MODULE_NAME, 0 },
	{},
};

static const struct of_device_id imx390_isp_of_match[] = {
	{
		.compatible = MODULE_NAME,
	},
	{},
};

MODULE_DEVICE_TABLE(of, imx390_isp_of_match);
MODULE_DEVICE_TABLE(i2c, imx390_isp_id);

static struct i2c_driver imx390_isp_driver = {
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(imx390_isp_of_match),
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,

	},
	.probe		= imx390_isp_probe,
	.remove		= imx390_isp_remove,
	.id_table	= imx390_isp_id,
};

module_i2c_driver(imx390_isp_driver);

MODULE_DESCRIPTION("IMX390 YUV camera driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("BST Ltd.");
