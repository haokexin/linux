// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#include <linux/component.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/videodev2.h>
#include <linux/media-bus-format.h>
#include <video/of_display_timing.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include "../bst_disp_conn.h"
#include "vout_conn.h"

struct bst_vout_conn {
	struct drm_connector connector;
	struct drm_encoder encoder;
	struct device *dev;
	u32 bus_format;
	u32 bus_flags;
	struct drm_display_mode mode;
	void __iomem *pinmux_base;
	struct videomode *vm[VOUT_TIMING_NUM];
};

static inline struct bst_vout_conn *con_to_bst_vc(struct drm_connector *c)
{
	return container_of(c, struct bst_vout_conn, connector);
}

static inline struct bst_vout_conn *enc_to_bst_vc(struct drm_encoder *e)
{
	return container_of(e, struct bst_vout_conn, encoder);
}

static __maybe_unused void parpare_pmm_reg(void *base, u32 reg, u32 val, u32 mask)
{
	u32 tmp_val = 0x00;
	tmp_val = vout_conn_read_reg(base, reg);
	tmp_val &= ~(mask);
	tmp_val |= val;
	vout_conn_write_reg(base, reg, tmp_val);
}

static int bst_vout_set_display_source(struct bst_vout_conn *vc,
					  struct drm_encoder *encoder)
{
	int ret;

	ret = drm_of_encoder_active_endpoint_id(vc->dev->of_node, encoder);
	if (ret < 0)
		return ret;

	return 0;
}

static int bst_vc_connector_get_modes(struct drm_connector *connector)
{
	struct bst_vout_conn *bst_vc = con_to_bst_vc(connector);
	int i;

	if (bst_vc->vm[0]) {
		struct drm_display_mode *mode;

		for(i = 0; i < VOUT_TIMING_NUM; i ++) {
			mode = drm_mode_create(connector->dev);
			if (!mode) {
				DRM_DEV_ERROR(bst_vc->dev,
					"failed to create a new display mode\n");
				return 0;
			}
			drm_display_mode_from_videomode(bst_vc->vm[i], mode);
			mode->type = DRM_MODE_TYPE_DRIVER;
			if(0 == i) {
				mode->type |= DRM_MODE_TYPE_PREFERRED;
			}
			drm_mode_probed_add(connector, mode);
		}
		return 1;
	}

	return 0;
}

void bst_vout_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static const struct drm_connector_funcs bst_vc_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = bst_vout_connector_destroy,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_connector_helper_funcs bst_vc_connector_helper_funcs = {
	.get_modes = bst_vc_connector_get_modes,
};

static void bst_vc_encoder_enable(struct drm_encoder *encoder)
{
	struct bst_vout_conn *vc = enc_to_bst_vc(encoder);
	bst_vout_set_display_source(vc, encoder);
}

static void bst_vc_encoder_disable(struct drm_encoder *encoder)
{
}

static int
bst_vc_encoder_atomic_check(struct drm_encoder *encoder,
				   struct drm_crtc_state *crtc_state,
				   struct drm_connector_state *conn_state)
{
	return 0;
}

static const
struct drm_encoder_helper_funcs bst_vc_encoder_helper_funcs = {
	.enable = bst_vc_encoder_enable,
	.disable = bst_vc_encoder_disable,
	.atomic_check = bst_vc_encoder_atomic_check,
};

int bst_vout_encoder_parse_of(struct drm_device *drm,
	struct drm_encoder *encoder, struct device_node *np)
{
	uint32_t crtc_mask = drm_of_find_possible_crtcs(drm, np);

	if (crtc_mask == 0)
		return -EPROBE_DEFER;

	encoder->possible_crtcs = crtc_mask;
	encoder->possible_clones = 0;

	return 0;
}

static int bst_vc_register(struct drm_device *drm,
	struct bst_vout_conn *bst_vc)
{
	struct drm_encoder *encoder = &bst_vc->encoder;
	struct drm_connector *connector = &bst_vc->connector;
	int ret;

	ret = bst_vout_encoder_parse_of(drm, encoder, bst_vc->dev->of_node);
	if (ret)
		return ret;

	ret = drm_connector_init(drm, connector,
							&bst_vc_connector_funcs,
							DRM_MODE_CONNECTOR_DPI);
	if (ret) {
		DRM_ERROR("failed to init VOUT connector\n");
		goto err_conn_init;
	}
	drm_connector_helper_add(connector, &bst_vc_connector_helper_funcs);

	ret = drm_connector_register(connector);
	if (ret < 0) {
		DRM_ERROR("failed to register VOUT connector\n");
		goto err_conn_register;
	}

	bst_vc->connector.dpms = DRM_MODE_DPMS_OFF;

	ret = drm_simple_encoder_init(drm, encoder, DRM_MODE_ENCODER_LVDS);
	if (ret < 0) {
		DRM_ERROR("failed to init VOUT encoder\n");
		goto err_encoder_init;
	}
	drm_encoder_helper_add(encoder, &bst_vc_encoder_helper_funcs);

	ret = drm_connector_attach_encoder(connector, encoder);
	if (ret < 0) {
		DRM_ERROR("failed to init VOUT encoder\n");
		goto err_attach;
	}
	drm_mode_config_reset(drm);

	dev_info(drm->dev, "complete vout register!!\n");

	return 0;

err_attach:
err_encoder_init:
	drm_encoder_cleanup(encoder);
err_conn_init:
err_conn_register:
	drm_connector_cleanup(connector);

	return ret;
}

static int bst_vout_conn_parse_dt(struct bst_vout_conn *vc){
	struct device *dev = vc->dev;
	struct device_node *dn = dev->of_node;
	struct device_node *np;
	int i;

	np = of_get_child_by_name(dn, "display-timings");
	if (np) {
		struct videomode *vm;
		int ret;

		if(of_get_child_count(np) != VOUT_TIMING_NUM) {
			DRM_ERROR("need %d timings for test\n", VOUT_TIMING_NUM);
		}

		of_node_put(np);

		for(i = 0; i < VOUT_TIMING_NUM; i ++) {
			vm = devm_kzalloc(dev, sizeof(struct videomode), GFP_KERNEL);
			if (!vm)
				return -ENOMEM;

			ret = of_get_videomode(dn, vm, i);
			if (ret < 0) {
				DRM_ERROR("can't found videomode\n");
				devm_kfree(dev, vm);
				return ret;
			}
			vc->vm[i] = vm;
		}
	} else {
		DRM_ERROR("failed to find display timing device tree\n");
		return -ENOENT;
	}

	return 0;
}

static int bst_vout_link_cfg(struct device *dev)
{
	struct bst_dpu_connection conn;
	int ret;

	ret = bst_get_remote_dpu_connection_by_port(dev, 0, &conn);
	if (ret) {
		dev_warn(dev, "vout not connect to dpu!\n");
		return ret;
	}

	bst_select_dpu_output_to_vout(&conn);
	dev_info(dev, "vout connect to dpu:%d,pipe:%d,link:%d!\n",
		conn.port.dpu_id, conn.port.pipeline_id, conn.port.link_id);
	put_device(conn.host);
	return 0;
}

static int bst_vc_bind(struct device *dev, struct device *master, void *data)
{
	struct drm_device *drm = data;
	struct bst_vout_conn *bst_vc;
	int ret;

	bst_vc = dev_get_drvdata(dev);
	bst_vc->bus_format = MEDIA_BUS_FMT_RGB888_1X24;
	bst_vc->dev = dev;

	ret = bst_vout_conn_parse_dt(bst_vc);
	if (ret < 0) {
		devm_kfree(dev, bst_vc);
		return ret;
	}

	ret = bst_vc_register(drm, bst_vc);
	if (ret)
		return ret;

	bst_vout_link_cfg(dev);

	return 0;
}

static void bst_vc_unbind(struct device *dev, struct device *master, void *data)
{
	struct bst_vout_conn *bst_vc = dev_get_drvdata(dev);
	struct drm_connector *connector = &bst_vc->connector;
	struct drm_encoder *encoder = &bst_vc->encoder;

	encoder->funcs->destroy(encoder);
	connector->funcs->destroy(connector);
}

static const struct component_ops bst_vc_ops = {
	.bind	= bst_vc_bind,
	.unbind	= bst_vc_unbind,
};

static int bst_vc_probe(struct platform_device *pdev)
{
	struct bst_vout_conn *bst_vc;

	bst_vc = devm_kzalloc(&pdev->dev, sizeof(*bst_vc), GFP_KERNEL);
	if (!bst_vc)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, bst_vc);

	return component_add(&pdev->dev, &bst_vc_ops);
}

static int bst_vc_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &bst_vc_ops);

	return 0;
}

static const struct of_device_id bst_vc_dt_ids[] = {
	{ .compatible = "bst,bst-vout-conn", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, bst_vc_dt_ids);

static struct platform_driver bst_vc_driver = {
	.probe		= bst_vc_probe,
	.remove		= bst_vc_remove,
	.driver		= {
		.of_match_table = bst_vc_dt_ids,
		.name	= "bst-vout-conn",
	},
};

module_platform_driver(bst_vc_driver);

MODULE_DESCRIPTION("BST VOUT Connector driver");
MODULE_AUTHOR("BST Ltd.");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bst-vout-conn");
