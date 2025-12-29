// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <dt-bindings/media/bst-isp.h>

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#ifdef CONFIG_BST_HEALTH_MONITOR
#include <bst/bst_common_api.h>
#endif

#include "csi_rx.h"

#include "csi_controller.h"
#include "csi_hw.h"
#include "csi_safety.h"
#include "csi_sysfs.h"

/* -----------------------------------------------------------------------------
 * V4L2 sub-device operations
 */
static int csi_pre_streamon(struct v4l2_subdev *sd, u32 flags)
{
	struct csi_device *csi;
	int rv;

	if (sd == NULL)
		return -EIO;

	csi = subdev_to_csi_device(sd);
	mutex_lock(&csi->lock);
	rv = v4l2_subdev_call(csi->remote_sd, video, s_stream,
			      STREAM_ENC(0, 1));
	mutex_unlock(&csi->lock);

	return rv;
}

static int csi_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct csi_device *csi;
	int rv;

	if (sd == NULL)
		return -EIO;

	csi = subdev_to_csi_device(sd);
	mutex_lock(&csi->lock);
	rv = v4l2_subdev_call(csi->remote_sd, video, s_stream, enable);
	if (!rv) {
		if (STREAM_DEC_EN(enable))
			++csi->used_vcs;
		else
			--csi->used_vcs;
		csi_safety_stream(csi, enable);
	}
	mutex_unlock(&csi->lock);

	return rv;
}

static const struct v4l2_subdev_video_ops csi_video_ops = {
	.s_stream = csi_s_stream,
	.pre_streamon = csi_pre_streamon,
};

static int csi_s_power(struct v4l2_subdev *sd, int on)
{
	struct csi_device *csi;
	int rv;

	if (sd == NULL)
		return -EIO;

	csi = subdev_to_csi_device(sd);
	mutex_lock(&csi->lock);
	rv = v4l2_subdev_call(csi->remote_sd, core, s_power, on);
	rv = 0;
	if (on)
		rv = csi_hw_init(csi);
	else
		csi_hw_exit(csi);
	mutex_unlock(&csi->lock);

	return rv;
}

static const struct v4l2_subdev_core_ops csi_core_ops = {
	.s_power = csi_s_power,
};

static const struct v4l2_subdev_ops csi_subdev_ops = {
	.core = &csi_core_ops,
	.video = &csi_video_ops,
};

static int csi_notify_bound(struct v4l2_async_notifier *notifier,
			    struct v4l2_subdev *sd,
			    struct v4l2_async_subdev *asd)
{
	int rv;
	struct v4l2_mbus_config mbus_cfg;
	struct csi_tx_dev *tx_dev;
	struct csi_device *csi;

	csi = notifier_to_csi_device(notifier);
	csi->remote_sd = sd;

	tx_dev = container_of(sd, struct csi_tx_dev, subdev);
	csi->tx_dev = tx_dev;

	rv = v4l2_subdev_call(sd, pad, get_mbus_config, 0, &mbus_cfg);
	dev_info(csi->dev, "Bound: get_mbus_config: %d\n", rv);
	if (rv)
		return 0;

	if (mbus_cfg.bus.mipi_csi2.num_data_lanes)
		csi->lane_num = mbus_cfg.bus.mipi_csi2.num_data_lanes;
	dev_info(csi->dev, "Bound: bus: %d, lanes: %u -> %u\n", mbus_cfg.type,
		 mbus_cfg.bus.mipi_csi2.num_data_lanes, csi->lane_num);
	if (mbus_cfg.type == V4L2_MBUS_CSI2_DPHY)
		csi->phy_if = IF_DPHY;
	else if (mbus_cfg.type == V4L2_MBUS_CSI2_CPHY)
		csi->phy_if = IF_CPHY;
	else
		return -EINVAL;

	if (csi->phy_if == IF_CPHY && csi->lane_num > CSI_MAX_CPHY_LANES)
		csi->lane_num = CSI_MAX_CPHY_LANES;

#ifdef CONFIG_BST_HEALTH_MONITOR
	csi_safety_get_psm(csi);
	if (csi->psm.host_access_confirm)
		csi->host_access_retries = REG_ACCESS_RETRIES;
	if (csi->psm.phy_access_confirm)
		csi->phy_access_retries = REG_ACCESS_RETRIES;
#endif

	return 0;
}

static const struct v4l2_async_notifier_operations csi_async_ops = {
	.bound = csi_notify_bound,
};

static int parse_ports(struct csi_device *csi, struct device_node *node)
{
	int i;
	int num;

	num = 0;
	for (i = 0; i < ARRAY_SIZE(csi->channels); ++i) {
		struct device_node *port;
		struct csi_channel *channel;

		channel = &csi->channels[i];
		channel->csi_dev = csi;
		channel->vc = i;

		port = of_graph_get_port_by_id(node, i);
		if (port == NULL) {
			dev_warn(csi->dev, "VC %d is not defined\n", i);
			continue;
		}
		++num;
		of_node_put(port);
	}

	return num;
}

static int parse_dt(struct csi_device *csi)
{
	int rv;
	struct device *dev;
	struct device_node *node;
	struct device_node *remote_ep;
	struct v4l2_fwnode_endpoint v4l2_ep;
	struct device_node *rx_ports;

	dev = csi->dev;
	node = dev->of_node;

#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	rv = of_property_read_u32(node, "uid", &csi->uid);
	if (rv) {
		dev_err(dev, "Failed to parse uid\n");
		goto err_uid;
	}
	{
		u32 sem[3];

		rv = of_property_read_u32_array(node, "ipc-sem", sem,
						ARRAY_SIZE(sem));
		if (rv) {
			dev_err(dev, "Failed to parse ipc-sem\n");
			goto err_sem;
		}
		csi->sem_master = sem[0];
		csi->sem_bank = sem[1];
		csi->sem_id = sem[2];
	}
#endif
	rv = of_property_read_u32(node, "id", &csi->id);
	if (rv) {
		dev_err(dev, "Failed to parse id\n");
		goto err_id;
	}

	rv = of_property_read_u32(node, "phy-if", &csi->phy_if);
	if (rv) {
		dev_warn(dev,
			 "Unable to parse phy-if, use DPHY mode by default\n");
		csi->phy_if = IF_DPHY;
	}

	csi->fwnode = of_fwnode_handle(node);
	memset(&v4l2_ep, 0, sizeof(v4l2_ep));
	rv = v4l2_fwnode_endpoint_parse(csi->fwnode, &v4l2_ep);
	if (rv) {
		dev_err(dev, "Failed to parse fwnode endpoint\n");
		goto err_parse_ep;
	}
	csi->lane_num = v4l2_ep.bus.mipi_csi2.num_data_lanes;
	if (csi->phy_if == IF_CPHY && csi->lane_num > 3)
		csi->lane_num = 3;

	rv = of_property_read_u32(node, "lane-speed", &csi->lane_speed);
	if (rv) {
		dev_err(dev, "Failed to parse lane-speed\n");
		goto err_speed;
	}

	rv = of_property_read_u32(node, "eq", &csi->eq);
	if (rv) {
		dev_warn(dev, "Unable to parse eq, use %u by default\n",
			 DEFAULT_EQ);
		csi->eq = DEFAULT_EQ;
	}

	csi->recoverable = of_property_read_bool(node, "recoverable");
	rv = of_property_read_u32(node, "recover-threshold",
				  &csi->recover_threshold);
	if (rv)
		csi->recover_threshold = DEFAULT_RECOVER_THRESHOLD;
	rv = of_property_read_u32(node, "recover-window", &csi->recover_window);
	if (rv)
		csi->recover_window = DEFAULT_RECOVER_WINDOW;

	csi->func_irq = platform_get_irq(csi->pdev, 0);
	csi->diag_irq = platform_get_irq(csi->pdev, 1);
	if (csi->func_irq < 0 || csi->diag_irq < 0) {
		dev_err(dev, "Failed to parse function or fmeda IRQ\n");
		rv = -EINVAL;
		goto err_irq;
	}
	csi->func_irq_enable = of_property_read_bool(node, "func-irq-enable");
	csi->diag_irq_enable = of_property_read_bool(node, "diag-irq-enable");

	csi->rstc = devm_reset_control_get_shared(dev, "csi-reset");
	if (IS_ERR_OR_NULL(csi->rstc))
		dev_warn(dev, "Unable to parse csi-reset\n");

	rx_ports = of_get_child_by_name(node, "rx-ports");
	if (!rx_ports) {
		dev_err(dev, "Failed to parse rx-ports\n");
		rv = -EINVAL;
		goto err_rx_ports;
	}
	remote_ep = of_graph_get_remote_node(rx_ports, 0, 0);
	if (!remote_ep) {
		dev_err(dev, "Failed to get remote ep\n");
		rv = -EINVAL;
		goto err_remote;
	}
	csi->remote_fwnode = of_fwnode_handle(remote_ep);
	dev_info(dev, "Remote fwnode: 0x%016llX, name: %s\n",
		 (u64)csi->remote_fwnode, remote_ep->full_name);
	parse_ports(csi, node);
	rv = 0;

	of_node_put(remote_ep);
err_remote:
	of_node_put(rx_ports);
err_rx_ports:
err_irq:
err_speed:
err_parse_ep:
	of_node_put(node);
err_id:
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
err_sem:
err_uid:
#endif
	return rv;
}

static int setup_reg_region(struct csi_device *csi)
{
	int rv;
	struct device *dev;
	struct resource *iomem;

	dev = csi->dev;

	iomem = platform_get_resource(csi->pdev, IORESOURCE_MEM, 0);
	if (iomem == NULL) {
		dev_err(dev, "Failed to get ctrl region\n");
		rv = -EINVAL;
		goto err0;
	}
	dev_info(dev, "Ctrl region start: 0x%08llX, end: 0x%08llX\n",
		 iomem->start, iomem->end);
	csi->ctrl_base = devm_ioremap_resource(dev, iomem);
	if (IS_ERR(csi->ctrl_base)) {
		rv = PTR_ERR(csi->ctrl_base);
		dev_err(dev, "Failed to remap ctrl base: %d\n", rv);
		goto err0;
	}

	iomem = platform_get_resource(csi->pdev, IORESOURCE_MEM, 1);
	if (iomem == NULL) {
		dev_err(dev, "Failed to get top region\n");
		rv = -EINVAL;
		goto err1;
	}
	dev_info(dev, "Top region start: 0x%08llX, end: 0x%08llX\n",
		 iomem->start, iomem->end);
	csi->top_base = devm_ioremap_resource(dev, iomem);
	if (IS_ERR(csi->top_base)) {
		rv = PTR_ERR(csi->top_base);
		dev_err(dev, "Failed to remap top base: %d\n", rv);
		goto err1;
	}

	return 0;

err1:
	devm_iounmap(dev, csi->ctrl_base);
err0:
	return rv;
}

static void finalize_reg_region(struct csi_device *csi)
{
	devm_iounmap(csi->dev, csi->top_base);
	devm_iounmap(csi->dev, csi->ctrl_base);
}

static int init_v4l2_dev(struct csi_device *csi)
{
	int rv;
	struct v4l2_subdev *sd;
	struct device *dev;

	dev = csi->dev;
	sd = &csi->subdev;

	v4l2_subdev_init(sd, &csi_subdev_ops);
	sd->dev = dev;
	sd->fwnode = csi->fwnode;
	snprintf(sd->name, sizeof(sd->name), "%s", dev_name(dev));
	v4l2_set_subdevdata(sd, csi);

	rv = v4l2_async_register_subdev(sd);
	if (rv < 0) {
		dev_err(dev, "Failed to register subdev\n");
		goto err_register_subdev;
	}

	csi->async_dev.match_type = V4L2_ASYNC_MATCH_FWNODE;
	csi->async_dev.match.fwnode = csi->remote_fwnode;
	v4l2_async_nf_init(&csi->notifier);
	rv = __v4l2_async_nf_add_subdev(&csi->notifier, &csi->async_dev);
	if (rv < 0) {
		dev_err(dev, "Failed to add async dev to notifier\n");
		goto err_add_async_dev;
	}

	csi->notifier.ops = &csi_async_ops;
	rv = v4l2_async_subdev_nf_register(&csi->subdev, &csi->notifier);
	if (rv < 0) {
		dev_err(dev, "Failed to register notifier\n");
		goto err_register_nf;
	}

	return 0;

err_register_nf:
	/* NOTE: We does not cleanup since the asd is static in csi_device */
	// v4l2_async_nf_cleanup(&csi->notifier);
err_add_async_dev:
	v4l2_async_unregister_subdev(sd);
err_register_subdev:
	return rv;
}

static void cleanup_v4l2_dev(struct csi_device *csi)
{
	v4l2_async_nf_unregister(&csi->notifier);
	/* NOTE: We does not cleanup since the asd is static in csi_device */
	// v4l2_async_nf_cleanup(&csi->notifier);
	v4l2_async_unregister_subdev(&csi->subdev);
}

/* -----------------------------------------------------------------------------
 * Exported functions
 */
void csi_update_camera_status(struct csi_device *csi)
{
	int i;

	if (csi == NULL)
		return;

	if (csi->tx_dev == NULL) {
		dev_err(csi->dev, "Unbound TX device\n");
		return;
	}
	for (i = 0; i < ARRAY_SIZE(csi->channels); ++i)
		/* NOTE: Channel in CSI RX and TX must be 1:1 matched by VC */
		csi->channels[i].cam_dev = csi->tx_dev->cameras[i];
}

/* -----------------------------------------------------------------------------
 * Driver interfaces
 */
static int csi_probe(struct platform_device *pdev)
{
	struct csi_device *csi;
	struct device *dev;
	int rv;

	dev = &pdev->dev;
	csi = devm_kzalloc(dev, sizeof(struct csi_device), GFP_KERNEL);
	if (!csi)
		return -ENOMEM;

	csi->dev = dev;
	csi->pdev = pdev;

	rv = parse_dt(csi);
	if (rv) {
		dev_err(dev, "Failed to parse device tree\n");
		goto err_dt;
	}

	if (!IS_ERR_OR_NULL(csi->rstc))
		reset_control_deassert(csi->rstc);

	rv = setup_reg_region(csi);
	if (rv) {
		dev_err(dev, "Failed to setup reg regions\n");
		goto err_setup_regs;
	}

	rv = init_v4l2_dev(csi);
	if (rv) {
		dev_err(dev, "Failed to init v4l2 dev\n");
		goto err_init_v4l2_dev;
	}

	mutex_init(&csi->lock);
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	csi->hwlock =
		bst_semaphore_init(csi->sem_master, csi->sem_bank, csi->sem_id);
	if (!csi->hwlock) {
		dev_err(dev, "Failed to claim HW lock\n");
		return -ENOLCK;
	}
#endif
	csi_sysfs_init(csi);
	platform_set_drvdata(pdev, csi);
	dev_info(dev, "Probe done on CPU %u\n", smp_processor_id());

	return 0;

err_init_v4l2_dev:
	finalize_reg_region(csi);
err_setup_regs:
err_dt:
	return rv;
}

static int csi_remove(struct platform_device *pdev)
{
	struct csi_device *csi;

	csi = platform_get_drvdata(pdev);
	dev_info(csi->dev, "Remove\n");

	csi_sysfs_exit(csi);
	cleanup_v4l2_dev(csi);
	finalize_reg_region(csi);
	mutex_destroy(&csi->lock);
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	samphore_lock_remove(csi->hwlock);
#endif

	return 0;
}

static void csi_shutdown(struct platform_device *pdev)
{
	struct csi_device *csi;

	csi = platform_get_drvdata(pdev);
	dev_info(csi->dev, "Shutdown\n");
}

static const struct of_device_id csi_of_table[] = {
	{ .compatible = "bst,c1200-csi2-rx" },
	{},
};
MODULE_DEVICE_TABLE(of, csi_of_table);

static struct platform_driver csi2_rx_driver = {
	.probe  = csi_probe,
	.remove = csi_remove,
	.shutdown = csi_shutdown,
	.driver = {
		.name = "bst-csi2-rx",
		.of_match_table = csi_of_table,
	},
};
module_platform_driver(csi2_rx_driver);

MODULE_DESCRIPTION("BST CSI-2 RX driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("BST Ltd.");
