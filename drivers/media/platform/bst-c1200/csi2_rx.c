// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "csi2_rx.h"
#include "csi_cdphy.h"
#include "csi_controller.h"

static int csi_channel_get_port_info(struct bst_csi_device *pcsi_dev,
				     struct device_node *node)
{
	struct device_node *port = NULL;
	struct bst_csi_channel *channel;
	int i;

	for (i = 0; i < MAX_VC_PER_CSI; i++) {
		channel = &pcsi_dev->csi_vc[i];
		channel->csi_dev_id = pcsi_dev->csi_id;
		channel->index = i;
		channel->csi_chn_id = ((pcsi_dev->csi_id << 2) | i);
		channel->sn_in_all_csi =
			((pcsi_dev->csi_id * MAX_VC_PER_CSI) + i);
		channel->csi = pcsi_dev;

		port = of_graph_get_port_by_id(node, i);
		if (port != NULL) {
			dev_info(pcsi_dev->dev, "mipi chn %d connected\n", i);
		} else {
			dev_info(pcsi_dev->dev, "mipi chn %d not connected\n",
				 i);
			continue;
		}
	}

	return 0;
}

static int csi_s_stream(struct v4l2_subdev *subdev, int enable)
{
	return 0;
}

static int csi_device_init(struct bst_csi_device *csi_dev)
{
	csi_cdphy_config_lanes(csi_dev);

	return 0;
}

static int csi_s_power(struct v4l2_subdev *sd, int enable)
{
	struct bst_csi_device *pcsi_dev;
	struct deser_hub_dev *pdeser_dev;

	pcsi_dev = container_of(sd, struct bst_csi_device, subdev);
	if (pcsi_dev == NULL)
		return -1;

	pdeser_dev = pcsi_dev->deser;
	if (pdeser_dev == NULL)
		return -1;

	if (enable)
		csi_device_init(pcsi_dev);

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Pad Operations
 */

static int csi_get_format(struct v4l2_subdev *subdev,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	return 0;
}

static int csi_set_format(struct v4l2_subdev *subdev,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Operations
 */
static const struct v4l2_subdev_video_ops csi_video_ops = {
	.s_stream = csi_s_stream,
};

static const struct v4l2_subdev_pad_ops csi_pad_ops = {
	.get_fmt = csi_get_format,
	.set_fmt = csi_set_format,
};

static const struct v4l2_subdev_core_ops csi_core_ops = {
	.s_power = csi_s_power,
};

static const struct v4l2_subdev_ops csi_sub_ops = {
	.core = &csi_core_ops,
	.video = &csi_video_ops,
	.pad = &csi_pad_ops,
};

/* -----------------------------------------------------------------------------
 * Media Operations
 */

static const struct media_entity_operations csi_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int csi_notify_bound(struct v4l2_async_notifier *notifier,
			    struct v4l2_subdev *sd,
			    struct v4l2_async_subdev *asd)
{
	struct deser_hub_dev *pdeser_dev;
	struct bst_csi_device *pcsi_dev;

	pdeser_dev = container_of(sd, struct deser_hub_dev, subdev);
	pcsi_dev = container_of(asd, struct bst_csi_device, async_dev);
	pcsi_dev->deser = pdeser_dev;
	pdeser_dev->csi_dev = pcsi_dev;
	pdeser_dev->sd_state = BST_SUBDEV_STATE_BOUND;
	pdeser_dev->data_lanes_num = pcsi_dev->num_lanes;
	return 0;
}

static void csi_notify_unbind(struct v4l2_async_notifier *notifier,
			      struct v4l2_subdev *subdev,
			      struct v4l2_async_subdev *asd)
{
}

static const struct v4l2_async_notifier_operations csi_async_ops = {
	.bound = csi_notify_bound,
	.unbind = csi_notify_unbind,
};

int update_camera_status_in_csi(struct bst_csi_device *pcsi_dev)
{
	struct deser_hub_dev *pdeser_dev;
	struct deser_channel *pdeser_chn;
	int i;

	if (pcsi_dev == NULL)
		return -1;

	pdeser_dev = pcsi_dev->deser;
	if (pdeser_dev == NULL)
		return -1;

	for (i = 0; i < MAX_VC_PER_CSI; i++) {
		pdeser_chn = &(pdeser_dev->chn[i]);
		if (pdeser_chn == NULL)
			continue;

		if (pdeser_chn->camera_bound) {
			pcsi_dev->csi_vc[i].cam_dev = pdeser_chn->cam_dev;
			pcsi_dev->csi_vc[i].connected =
				pdeser_chn->cam_dev->power_on;
		}
	}

	return 0;
}

static int init_csi_channel_one(struct bst_csi_channel *pcsi_channel, int index)
{
	int ret;

	pcsi_channel->pads[CSI_CHANNEL_SINK_PAD].flags = MEDIA_PAD_FL_SINK;
	pcsi_channel->pads[CSI_CHANNEL_SOURCE_PAD].flags = MEDIA_PAD_FL_SOURCE;

	pcsi_channel->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	pcsi_channel->entity.ops = &csi_media_ops;

	ret = media_entity_pads_init(&pcsi_channel->entity, 2,
				     pcsi_channel->pads);
	if (ret < 0)
		return ret;

	return 0;
}

static int init_csi_channel_devs(struct bst_csi_device *pcsi_dev)
{
	int i;
	struct bst_csi_channel *pcsi_channel;

	for (i = 0; i < pcsi_dev->num_vc; i++) {
		pcsi_channel = &pcsi_dev->csi_vc[i];
		pcsi_channel->csi = pcsi_dev;
		init_csi_channel_one(pcsi_channel, i);
	}

	return 0;
}

static int bst_csi_parse_dt(struct bst_csi_device *pcsi_dev)
{
	struct device_node *remote_ep;
	struct v4l2_fwnode_endpoint v4l2_ep;
	int ret;
	int id;
	int num_channels = 0;
	int lane_speed;
	int phy_mode = 0;
	struct device_node *node = pcsi_dev->dev->of_node;
	struct device_node *link_dt = NULL;
	struct v4l2_subdev *sd;
	struct resource *iomem;

	if (!node)
		return -EINVAL;

	ret = of_property_read_u32(node, "id", &id);
	if (ret) {
		dev_err(pcsi_dev->dev, "mipi find id error\n");
		return -2;
	}
	pcsi_dev->csi_id = id;
	ret = of_property_read_u32(node, "lane-speed", &lane_speed);
	if (ret) {
		dev_err(pcsi_dev->dev, "mipi find lane-speed error\n");
		return -2;
	}
	pcsi_dev->lane_speed = lane_speed;

	memset(&v4l2_ep, 0, sizeof(v4l2_ep));
	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(node), &v4l2_ep);
	if (ret) {
		dev_err(pcsi_dev->dev,
			"mipi v4l2_fwnode_endpoint_parse error\n");
		return -4;
	}
	pcsi_dev->num_lanes = v4l2_ep.bus.mipi_csi2.num_data_lanes;
	dev_dbg(pcsi_dev->dev, "num_lanes = %d\n", pcsi_dev->num_lanes);

	link_dt = of_get_child_by_name(node, "csi-link");
	if (link_dt == NULL) {
		dev_err(pcsi_dev->dev, "get csi-link error\n");
		return -5;
	}
	remote_ep = of_graph_get_remote_node(link_dt, 0, 0);
	if (remote_ep == NULL) {
		dev_err(pcsi_dev->dev, "can not find remote ep\n");
		return -1;
	}
	dev_dbg(pcsi_dev->dev, "remote_ep name = %s, fullname = %s\n",
		remote_ep->name, remote_ep->full_name);

	pcsi_dev->csi_fwnode = of_fwnode_handle(node);
	pcsi_dev->remote_fwnode = of_fwnode_handle(remote_ep);
	dev_dbg(pcsi_dev->dev,
		"node = %s, csi_fwnode = 0x%p, remote = %s, fwnode = 0x%p\n",
		node->full_name, pcsi_dev->csi_fwnode, remote_ep->full_name,
		pcsi_dev->remote_fwnode);

	ret = of_property_read_u32(remote_ep->parent, "phy-mode", &phy_mode);
	if (ret) {
		dev_warn(pcsi_dev->dev, "mipi find phy-mode fail\n");
		phy_mode = 0;
	}
	pcsi_dev->phy_mode_cfg = phy_mode;
	dev_info(pcsi_dev->dev,
		 "%s: id %d, speed %d, phy mode %d, lane num %d\n", __func__,
		 id, lane_speed, phy_mode, pcsi_dev->num_lanes);

	pcsi_dev->function_irq = platform_get_irq(pcsi_dev->pdev, 0);
	pcsi_dev->fmeda_irq = platform_get_irq(pcsi_dev->pdev, 1);

	iomem = platform_get_resource(pcsi_dev->pdev, IORESOURCE_MEM, 0);
	if (IS_ERR_OR_NULL(iomem)) {
		ret = PTR_ERR_OR_ZERO(iomem);
		dev_err(pcsi_dev->dev, "get IORESOURCE_MEM 0 return %d", ret);
	} else {
		dev_info(pcsi_dev->dev,
			 "memory region 0 start: 0x%08llX, end: 0x%08llX\n",
			 iomem->start, iomem->end);
		pcsi_dev->ctrl_base =
			devm_ioremap_resource(pcsi_dev->dev, iomem);
		if (IS_ERR_OR_NULL(pcsi_dev->ctrl_base)) {
			ret = PTR_ERR_OR_ZERO(pcsi_dev->ctrl_base);
			dev_err(pcsi_dev->dev,
				"Failed to remap control base: %d\n", ret);
		}
	}

	iomem = platform_get_resource(pcsi_dev->pdev, IORESOURCE_MEM, 1);
	if (IS_ERR_OR_NULL(iomem)) {
		ret = PTR_ERR_OR_ZERO(iomem);
		dev_err(pcsi_dev->dev, "get IORESOURCE_MEM 1 return %d", ret);
	} else {
		dev_info(pcsi_dev->dev,
			 "memory region 1 start: 0x%08llX, end: 0x%08llX\n",
			 iomem->start, iomem->end);
		pcsi_dev->top_base =
			devm_ioremap_resource(pcsi_dev->dev, iomem);
		if (IS_ERR_OR_NULL(pcsi_dev->top_base)) {
			ret = PTR_ERR_OR_ZERO(pcsi_dev->top_base);
			dev_err(pcsi_dev->dev,
				"Failed to remap vendor top base: %d\n", ret);
			return ret;
		}
	}
	sd = &pcsi_dev->subdev;
	v4l2_subdev_init(sd, &csi_sub_ops);
	sd->dev = pcsi_dev->dev;
	v4l2_set_subdevdata(sd, pcsi_dev);
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd->entity.ops = &csi_media_ops;
	sd->fwnode = pcsi_dev->csi_fwnode;
	snprintf(sd->name, sizeof(sd->name), "%s", dev_name(pcsi_dev->dev));

	ret = v4l2_async_register_subdev(sd);
	if (ret < 0) {
		dev_err(pcsi_dev->dev, "failed to register subdev\n");
		media_entity_cleanup(&sd->entity);
	}

	pcsi_dev->async_dev.match_type = V4L2_ASYNC_MATCH_FWNODE;
	pcsi_dev->async_dev.match.fwnode = pcsi_dev->remote_fwnode;
	v4l2_async_nf_init(&pcsi_dev->notifier);
	ret = __v4l2_async_nf_add_subdev(&pcsi_dev->notifier,
					 &pcsi_dev->async_dev);
	if (ret < 0)
		dev_err(pcsi_dev->dev,
			"Failed to do __v4l2_async_nf_add_subdev\n");

	pcsi_dev->notifier.ops = &csi_async_ops;
	ret = v4l2_async_subdev_nf_register(&pcsi_dev->subdev,
					    &(pcsi_dev->notifier));
	if (ret < 0)
		dev_err(pcsi_dev->dev,
			"v4l2_async_subdev_nf_register register failed\n");

	num_channels = csi_channel_get_port_info(pcsi_dev, node);
	pcsi_dev->num_vc = num_channels;

	return 0;
}

static int c1200_csi_probe(struct platform_device *pdev)
{
	struct bst_csi_device *pcsi_dev;
	struct device *dev = &pdev->dev;
	// struct reset_control *rst_contrl = NULL;

	pcsi_dev = devm_kzalloc(dev, sizeof(struct bst_csi_device), GFP_KERNEL);
	if (!pcsi_dev)
		return -ENOMEM;

	pcsi_dev->pdev = pdev;
	pcsi_dev->dev = dev;
	mutex_init(&pcsi_dev->mutex);
	// rst_contrl =
	//	devm_reset_control_get_optional_exclusive(&pdev->dev, NULL);
	// reset_control_deassert(rst_contrl);
	bst_csi_parse_dt(pcsi_dev);
	init_csi_channel_devs(pcsi_dev);
	// FMEDA IRQ contains events reported by function IRQ,
	// So, we choose enable FMEDA IRQ first if possible
	// controller_enable_function_irq(pcsi_dev);
	// controller_enable_fmeda_irq(pcsi_dev);

	return 0;
}

static int c1200_csi_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id c1200_csi_of_table[] = {
	{ .compatible = "bst,c1200-csi2-rx" },
	{},
};
MODULE_DEVICE_TABLE(of, c1200_csi_of_table);

static struct platform_driver c1200_csi2_rx_driver = {
	.probe  = c1200_csi_probe,
	.remove = c1200_csi_remove,
	.driver = {
		.name = "bst-csi2-rx",
		.of_match_table = c1200_csi_of_table,
	},
};

module_platform_driver(c1200_csi2_rx_driver);

MODULE_DESCRIPTION("BST C1200 CSI-2 RX driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("BST Ltd.");
