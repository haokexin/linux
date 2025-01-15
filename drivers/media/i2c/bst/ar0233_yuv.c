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

#define MODULE_NAME "bst,ar0233-yuv"

static unsigned short config_serdes_setting[][2] = {
	{0x2be, 0x10},
	{0x318, 0x5e},
	{0x2d3, 0x84},
};

static int ar0233_ser_cfg(struct camera_dev *ar0233_yuv)
{
	int i;
	int retry_times;
	int ret;
	struct i2c_adapter *adap;

	pr_info("ar0233 %s(), line %d\n", __func__, __LINE__);

	if (ar0233_yuv == NULL) {
		pr_info("%s : camera_dev is NULL\n", __func__);
		return -EINVAL;
	}
	adap = ar0233_yuv->i2c_client->adapter;

	if (adap == NULL)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(config_serdes_setting); i++) {
		retry_times = 12;
		ret = -1;
		while (retry_times > 0 && ret) {
			ret = bst_i2c_write_byte_data_word_reg(
				adap, ar0233_yuv->ser_alias_id,
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
	return 0;
}

static int camera_s_power(struct v4l2_subdev *sd, int enable)
{
	int ret;
	struct camera_dev *ar0233_yuv =
		container_of(sd, struct camera_dev, subdev);

	pr_info("ar0233 %s(), line %d\n", __func__, __LINE__);

	if (!enable)
		return 0;

	if (!is_slave_soc_model(ar0233_yuv)) {
		ret = ar0233_ser_cfg(ar0233_yuv);
		if (ret)
			return ret;

	}
	ar0233_yuv->power_on = true;

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

static const struct media_entity_operations ar0233_yuv_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

// sensor is not sured powered on this time
// don't check camera connect here
static int ar0233_yuv_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct camera_dev *ar0233_yuv;
	struct device *dev = &client->dev;
	struct resource res;
	int bus_index = 0;
	int ret;

	ar0233_yuv = devm_kzalloc(&client->dev, sizeof(struct camera_dev),
				 GFP_KERNEL);
	if (!ar0233_yuv)
		return -ENOMEM;

	if (client->adapter == NULL) {
		pr_err("client->adapter == NULL, error\n");
		return -1;
	};
	ar0233_yuv->i2c_client = client;
	ar0233_yuv->dev = dev;
	bus_index = client->adapter->nr;

	if (of_address_to_resource(client->adapter->dev.of_node, 0, &res)) {
		dev_err(dev, "Can not get adapter's address\n'");
		return -1;
	}

	ar0233_yuv->isp_data.i2cRegBase = res.start;
	ar0233_yuv->isp_data.sensorRdWrMode =
		BST_SENSOR_RW_MODE_WORD_REG_WORD_DATA;
	ar0233_yuv->isp_data.sensorType = BST_SENSOR_TYPE_YUV422;

	ret = parse_camera_endpoint(ar0233_yuv, dev->of_node);
	if (ret) {
		dev_err(dev, "init_camera_dev error, ret = %d\n", ret);
		return ret;
	}

	ret = init_camera_dev(ar0233_yuv, &camera_ops, &ar0233_yuv_media_ops);
	if (ret) {
		dev_err(dev, "init_camera_dev error, ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static void ar0233_yuv_remove(struct i2c_client *client)
{
}

static const struct i2c_device_id ar0233_yuv_id[] = {
	{ MODULE_NAME, 0 },
	{},
};

static const struct of_device_id ar0233_yuv_of_match[] = {
	{
		.compatible = MODULE_NAME,
	},
	{},
};

MODULE_DEVICE_TABLE(of, ar0233_yuv_of_match);
MODULE_DEVICE_TABLE(i2c, ar0233_yuv_id);

static struct i2c_driver ar0233_yuv_driver = {
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(ar0233_yuv_of_match),
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = ar0233_yuv_probe,
	.remove = ar0233_yuv_remove,
	.id_table = ar0233_yuv_id,
};

module_i2c_driver(ar0233_yuv_driver);

MODULE_DESCRIPTION("AR0233 YUV camera driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("BST Ltd.");
