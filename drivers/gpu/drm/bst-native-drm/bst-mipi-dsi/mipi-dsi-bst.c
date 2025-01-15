// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:
 *   Chris Zhong <zyw@rock-chips.com>
 *   Nickey Yang <nickey.yang@rock-chips.com>
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include "mipi-dsi-bst.h"
#include "mipi_dsi_hal.h"
#include "bst_disp_conn.h"
#include "bst_dpu_csr.h"

static int bst_dsi_init_remote_source(struct dw_mipi_dsi_bst *dsi,
			unsigned int dsi_id)
{
	struct bst_dpu_connection conn;
	int ret;

	ret = bst_get_remote_dpu_connection_by_port(dsi->dev, 0, &conn);
	if (ret) {
		DRM_WARN("dsi%d not connect to dpu!\n", dsi_id);
		return ret;
	}

	bst_select_dpu_output_to_dsi(&conn, dsi_id);
	DRM_INFO("dsi%d connect to dpu:%d, pipe:%d, link:%d!\n", dsi_id,
		conn.port.dpu_id, conn.port.pipeline_id, conn.port.link_id);

	dsi->host_dpu = conn.host;
	bst_dpu_check_and_release(dsi->host_dpu);
	return ret;
}

static int bst_mipi_dsi_remove(struct platform_device *pdev)
{
	struct dw_mipi_dsi_bst *dsi = platform_get_drvdata(pdev);
	DRM_DEBUG("%s line:%d!!",__FUNCTION__,__LINE__);

	dw_mipi_dsi_remove(dsi->dw_dsi);
	return 0;
}

static struct bst_dsi_chip_data dw_mipi_dsi_bst_plat_data[] = {
	{
		.reg = 0x24608000,//mipi dsi0
		.max_data_lanes = 4,
		.dsi_id = 0,
	},
	{
		.reg = 0x24604000,//mipi dsi1
		.max_data_lanes = 4,
		.dsi_id = 1,
	},
};

static int bst_mipi_dsi_phy_init(void *priv_data)
{
	struct dw_mipi_dsi_bst *dsi = priv_data;

	DRM_INFO("dsi_id:%d lane_mbps:%d !!",dsi->cdata->dsi_id,dsi->lane_mbps);
	cfg_dphy_signals(dsi,dsi->cdata->dsi_id,dsi->lane_mbps);
	return 0;
}

static void bst_mipi_dsi_phy_power_on(void *priv_data)
{
	struct dw_mipi_dsi_bst *dsi = priv_data;
	dphy_rate_swtch(dsi,dsi->lane_mbps);
}

static void bst_mipi_dsi_phy_power_off(void *priv_data)
{
}

static int
bst_mipi_dsi_get_lane_mbps(void *priv_data, const struct drm_display_mode *mode,
			  unsigned long mode_flags, u32 lanes, u32 format,
			  unsigned int *lane_mbps)
{
	struct dw_mipi_dsi_bst *dsi = priv_data;
	unsigned int mpclk = 0;
	unsigned int target_mbps = 1000;
	int bpp=0;
	dsi->format = format;
	bpp = mipi_dsi_pixel_format_to_bpp(dsi->format);
	if (bpp < 0) {
		DRM_DEV_ERROR(dsi->dev,
			      "failed to get bpp for pixel format %d\n",
			      dsi->format);
		return bpp;
	}
	mpclk = DIV_ROUND_UP(mode->clock, MSEC_PER_SEC);
	if (mpclk) {
		/* take 1 / 0.8, since mbps must big than bandwidth of RGB */
		target_mbps = mpclk * (bpp / lanes) * 10 / 8;
	}
	dsi->lane_mbps = (target_mbps / 50) * 50;
	*lane_mbps = dsi->lane_mbps;

	return 0;
}

static int
bst_mipi_dsi_phy_get_timing(void *priv_data, unsigned int lane_mbps,
			   struct dw_mipi_dsi_dphy_timing *timing)
{
	return 0;
}

static const struct dw_mipi_dsi_phy_ops dw_mipi_dsi_bst_phy_ops = {
	.init = bst_mipi_dsi_phy_init,
	.power_on = bst_mipi_dsi_phy_power_on,
	.power_off = bst_mipi_dsi_phy_power_off,
	.get_lane_mbps = bst_mipi_dsi_get_lane_mbps,
	.get_timing = bst_mipi_dsi_phy_get_timing,
};

static int
dw_mipi_dsi_encoder_atomic_check(struct drm_encoder *encoder,
				 struct drm_crtc_state *crtc_state,
				 struct drm_connector_state *conn_state)
{
	struct bst_crtc_state *s = to_bst_crtc_state(crtc_state);
	struct dw_mipi_dsi_bst *dsi = to_dsi(encoder);
	DRM_DEBUG("%s line:%d!!",__FUNCTION__,__LINE__);
	switch (dsi->format) {
	case MIPI_DSI_FMT_RGB888:
		break;
	case MIPI_DSI_FMT_RGB666:
		break;
	case MIPI_DSI_FMT_RGB565:
		break;
	default:
		WARN_ON(1);
		return -EINVAL;
	}

	s->output_type = DRM_MODE_CONNECTOR_DSI;
	if (dsi->dsi1)
		s->output_flags = 1;

	return 0;
}

static void dw_mipi_dsi_encoder_enable(struct drm_encoder *encoder)
{
	struct dw_mipi_dsi_bst *dsi = to_dsi(encoder);
	struct bst_dev *mdev = encoder->dev->dev_private;
	int mux;
	bool dual_link=false;

	if(mdev->resume) {
		bst_mipi_dsi_reset(dsi);
		if(!dsi->cdata->dsi_id) {
			bst_dsi_init_remote_source(dsi, MIPI_DSI0_INST);
		} else if(dsi->cdata->dsi_id==MIPI_DSI1_INST) {
			bst_dsi_init_remote_source(dsi, MIPI_DSI1_INST);
		}
	}

	mux = drm_of_encoder_active_endpoint_id(dsi->dev->of_node,
						&dsi->encoder);
	if (mux < 0)
		return;

	pm_runtime_get_sync(dsi->dev);
	if (dsi->dsi1)
		pm_runtime_get_sync(dsi->dsi1->dev);

	dual_link = of_property_read_bool(dsi->dev->of_node,
		"dual-link");

	if(dual_link){
		mipi_dsi_channel(dsi,MIPI_DSI0_INST);
		mipi_dsi_channel(dsi,MIPI_DSI1_INST);
	}else{
		if(!dsi->cdata->dsi_id){
			mipi_dsi_channel(dsi,MIPI_DSI0_INST);
		} else if(dsi->cdata->dsi_id==MIPI_DSI1_INST) {
			mipi_dsi_channel(dsi,MIPI_DSI1_INST);
		}
	}
	if(dsi->lane_mbps > 1500){
		DRM_INFO("do skew calibration!\n");
		bst_dsi_write(dsi, DSI_PHY_CAL, 0x1);
		udelay(50);
		bst_dsi_write(dsi, DSI_PHY_CAL, 0x0);
	}
}

static void dw_mipi_dsi_encoder_disable(struct drm_encoder *encoder)
{
	struct dw_mipi_dsi_bst *dsi = to_dsi(encoder);
	if (dsi->dsi1)
		pm_runtime_put(dsi->dsi1->dev);
	pm_runtime_put(dsi->dev);
}

static const struct drm_encoder_helper_funcs
bst_mipi_dsi_encoder_helper_funcs = {
	.atomic_check = dw_mipi_dsi_encoder_atomic_check,
	.enable = dw_mipi_dsi_encoder_enable,
	.disable = dw_mipi_dsi_encoder_disable,
};

static int bst_dsi_drm_create_encoder(struct dw_mipi_dsi_bst *dsi,
					   struct drm_device *drm_dev)
{
	struct drm_encoder *encoder = &dsi->encoder;
	int ret;

	encoder->possible_crtcs = drm_of_find_possible_crtcs(drm_dev,
							     dsi->dev->of_node);
	ret = drm_simple_encoder_init(drm_dev, encoder, DRM_MODE_ENCODER_DSI);
	if (ret) {
		DRM_ERROR("Failed to initialize encoder with drm\n");
		return ret;
	}
	drm_encoder_helper_add(encoder, &bst_mipi_dsi_encoder_helper_funcs);
	return 0;
}

static struct device
*bst_mipi_dsi_find_second(struct dw_mipi_dsi_bst *dsi)
{
	const struct of_device_id *match;
	struct device_node *node = NULL, *local;

	match = of_match_device(dsi->dev->driver->of_match_table, dsi->dev);
	local = of_graph_get_remote_node(dsi->dev->of_node, 1, 0);
	if (!local)
		return NULL;
	while ((node = of_find_compatible_node(node, NULL,
					       match->compatible))) {
		struct device_node *remote;
		/* found ourself */
		if (node == dsi->dev->of_node)
			continue;

		remote = of_graph_get_remote_node(node, 1, 0);
		if (!remote)
			continue;
		DRM_INFO("remote name:%s full name:%s\n",remote->name,remote->full_name);
		DRM_INFO("local name:%s full name:%s\n",local->name,local->full_name);

		if (remote == local) {
			struct dw_mipi_dsi_bst *dsi2;
			struct platform_device *pdev;

			pdev = of_find_device_by_node(node);

			of_node_put(remote);
			of_node_put(node);
			of_node_put(local);

			if (!pdev)
				return ERR_PTR(-EPROBE_DEFER);

			dsi2 = platform_get_drvdata(pdev);
			if (!dsi2) {
				platform_device_put(pdev);
				return ERR_PTR(-EPROBE_DEFER);
			}
			DRM_INFO("second dsi find!\n");
			return &pdev->dev;
		}

		of_node_put(remote);
	}

	of_node_put(local);

	return NULL;
}

static int dw_mipi_dsi_bst_bind(struct device *dev,
				     struct device *master,
				     void *data)
{
	struct dw_mipi_dsi_bst *dsi = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	struct device *second;
	bool master1, master2;
	int ret;

	second = bst_mipi_dsi_find_second(dsi);
	if (IS_ERR(second))
		return PTR_ERR(second);
	if (second) {
		DRM_INFO(" find second channel %s line:%d!!",__FUNCTION__,__LINE__);
		master1 = of_property_read_bool(dsi->dev->of_node,
						"clock-master");
		master2 = of_property_read_bool(second->of_node,
						"clock-master");

		if (master1 && master2) {
			DRM_DEV_ERROR(dsi->dev, "only one clock-master allowed\n");
			return -EINVAL;
		}

		if (!master1 && !master2) {
			DRM_DEV_ERROR(dsi->dev, "no clock-master defined\n");
			return -EINVAL;
		}

		/* we are the slave in dual-DSI */
		if (!master1) {
			dsi->is_slave = true;
			bst_dsi_init_remote_source(dsi,
				dsi->cdata->dsi_id ? MIPI_DSI1_INST : MIPI_DSI0_INST);
			return 0;
		}

		dsi->dsi1 = dev_get_drvdata(second);
		if (!dsi->dsi1) {
			DRM_DEV_ERROR(dev, "could not get slaves data\n");
			return -ENODEV;
		}

		dsi->dsi1->is_slave = true;
		dw_mipi_dsi_set_slave(dsi->dw_dsi, dsi->dsi1->dw_dsi);
		put_device(second);
	}

	ret = bst_dsi_drm_create_encoder(dsi, drm_dev);
	if (ret) {
		DRM_DEV_ERROR(dev, "Failed to create drm encoder\n");
		return ret;
	}
	ret = dw_mipi_dsi_bind(dsi->dw_dsi, &dsi->encoder);
	if (ret) {
		DRM_ERROR("Failed to dw_mipi_dsi_bind: %d\n", ret);
		return ret;
	}
	bst_mipi_dsi_reset(dsi);
#ifdef CONFIG_DEBUG_FS
	bst_dsi_debugfs_init(dsi);
#endif

	if(!dsi->cdata->dsi_id){
		bst_dsi_init_remote_source(dsi, MIPI_DSI0_INST);
	} else if(dsi->cdata->dsi_id==MIPI_DSI1_INST) {
		bst_dsi_init_remote_source(dsi, MIPI_DSI1_INST);
	}
	return 0;
}

static void dw_mipi_dsi_bst_unbind(struct device *dev,
					struct device *master,
					void *data)
{
	struct dw_mipi_dsi_bst *dsi = dev_get_drvdata(dev);
	DRM_INFO("%s line:%d!",__FUNCTION__,__LINE__);
	if (dsi->host_dpu)
		put_device(dsi->host_dpu);
	if (dsi->is_slave)
		return;

	dw_mipi_dsi_unbind(dsi->dw_dsi);
}


static const struct component_ops dw_mipi_dsi_bst_ops = {
	.bind	= dw_mipi_dsi_bst_bind,
	.unbind	= dw_mipi_dsi_bst_unbind,
};

static int mipi_dsi_bst_host_attach(void *priv_data,
					    struct mipi_dsi_device *device)
{
	struct dw_mipi_dsi_bst *dsi = priv_data;
	struct device *second;
	int ret;
	ret = component_add(dsi->dev, &dw_mipi_dsi_bst_ops);
	if (ret) {
		DRM_DEV_ERROR(dsi->dev, "Failed to register component: %d\n",
					ret);
		return ret;
	}
	second = bst_mipi_dsi_find_second(dsi);
	if (IS_ERR(second)){
		return PTR_ERR(second);
	}
	if (second) {
		ret = component_add(second, &dw_mipi_dsi_bst_ops);
		if (ret) {
			DRM_DEV_ERROR(second,
				      "Failed to register component: %d\n",
				      ret);
			return ret;
		}
	}
	return 0;
}

static int mipi_dsi_bst_host_detach(void *priv_data,
					    struct mipi_dsi_device *device)
{
	struct dw_mipi_dsi_bst *dsi = priv_data;
	struct device *second;

	second = bst_mipi_dsi_find_second(dsi);
	if (second && !IS_ERR(second))
		component_del(second, &dw_mipi_dsi_bst_ops);

	component_del(dsi->dev, &dw_mipi_dsi_bst_ops);
	return 0;
}


static const struct dw_mipi_dsi_host_ops bst_mipi_dsi_bst_host_ops = {
	.attach = mipi_dsi_bst_host_attach,
	.detach = mipi_dsi_bst_host_detach,
};

static int bst_mipi_dsi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct dw_mipi_dsi_bst *dsi;
	struct resource *res;
	int ret, i=0;
	const struct bst_dsi_chip_data *cdata =
				of_device_get_match_data(dev);
	DRM_INFO("bst_mipi_dsi_probe start \n");
	dsi = devm_kzalloc(dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return -ENOMEM;
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dsi-base");
	dsi->dsi_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dsi->dsi_base)) {
		DRM_DEV_ERROR(dev, "Unable to get dsi-base registers\n");
		return PTR_ERR(dsi->dsi_base);
	}

	dsi->csr_regmap = syscon_regmap_lookup_by_compatible("bst,bst-mipi-csr");
	if (IS_ERR(dsi->csr_regmap))
		DRM_DEV_ERROR(dev, "%s: failed to find mipi_csr regmap!\n", __FUNCTION__);

	if(!cdata){
		DRM_DEV_ERROR(dev, "cdata is NULL!\n");
		return -EINVAL;
	}
	while (cdata[i].reg) {
		if (cdata[i].reg == res->start) {
			dsi->cdata = &cdata[i];
			break;
		}
		i++;
	}
	if (!dsi->cdata) {
		DRM_DEV_ERROR(dev, "no dsi-config for %s node\n", np->name);
		return -EINVAL;
	}
	dsi->dev = dev;
	dsi->pdata.base = dsi->dsi_base;
	dsi->pdata.max_data_lanes = dsi->cdata->max_data_lanes;
	dsi->pdata.phy_ops = &dw_mipi_dsi_bst_phy_ops;
	dsi->pdata.host_ops = &bst_mipi_dsi_bst_host_ops;
	dsi->pdata.priv_data = dsi;
	platform_set_drvdata(pdev, dsi);
	dsi->dw_dsi = dw_mipi_dsi_probe(pdev, &dsi->pdata);
	if (IS_ERR(dsi->dw_dsi)) {
		ret = PTR_ERR(dsi->dw_dsi);
		if (ret != -EPROBE_DEFER)
			DRM_DEV_ERROR(dev,
				      "Failed to probe dw_mipi_dsi_bst: %d\n", ret);
		goto err_clkdisable;
	}
	return 0;

err_clkdisable:
	return ret;
}

static const struct of_device_id bst_mipi_dsi[] = {
	{ .compatible = "bst,bst-dsi",
	  .data = &dw_mipi_dsi_bst_plat_data,
	},
	{},
};
MODULE_DEVICE_TABLE(of, bst_mipi_dsi);
static struct platform_driver bst_dsi_platform_driver = {
       .probe = bst_mipi_dsi_probe,
       .remove = bst_mipi_dsi_remove,
       .driver = {
               .name = "bst-dsi",
               .of_match_table = bst_mipi_dsi,
			   .probe_type = PROBE_FORCE_SYNCHRONOUS,
       },
};

static int __init bst_dsi_init(void)
{
	int ret=0;
    ret = platform_driver_register(&bst_dsi_platform_driver);
	return ret;
}
module_init(bst_dsi_init);

MODULE_AUTHOR("BST Ltd.");
MODULE_DESCRIPTION("BST MIPI DSI host controller driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mipi-dsi-bst");
