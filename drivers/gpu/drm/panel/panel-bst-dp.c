/*
 * Copyright (C) 2013, NVIDIA Corporation.  All rights reserved.
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sub license,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/media-bus-format.h>

#include <video/display_timing.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>

#include <drm/display/drm_dp_aux_bus.h>
#include <drm/display/drm_dp_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_panel.h>

/**
 * struct panel_desc - Describes a simple panel.
 */
struct panel_desc {
	/**
	 * @modes: Pointer to array of fixed modes appropriate for this panel.
	 *
	 * If only one mode then this can just be the address of the mode.
	 * NOTE: cannot be used with "timings" and also if this is specified
	 * then you cannot override the mode in the device tree.
	 */
	const struct drm_display_mode *modes;

	/** @num_modes: Number of elements in modes array. */
	unsigned int num_modes;

	/**
	 * @timings: Pointer to array of display timings
	 *
	 * NOTE: cannot be used with "modes" and also these will be used to
	 * validate a device tree override if one is present.
	 */
	const struct display_timing *timings;

	/** @num_timings: Number of elements in timings array. */
	unsigned int num_timings;

	/** @bpc: Bits per color. */
	unsigned int bpc;

	/** @size: Structure containing the physical size of this panel. */
	struct {
		/**
		 * @size.width: Width (in mm) of the active display area.
		 */
		unsigned int width;

		/**
		 * @size.height: Height (in mm) of the active display area.
		 */
		unsigned int height;
	} size;

	/** @delay: Structure containing various delay values for this panel. */
	struct {
		/**
		 * @delay.prepare: Time for the panel to become ready.
		 *
		 * The time (in milliseconds) that it takes for the panel to
		 * become ready and start receiving video data
		 */
		unsigned int prepare;

		/**
		 * @delay.hpd_absent_delay: Time to wait if HPD isn't hooked up.
		 *
		 * Add this to the prepare delay if we know Hot Plug Detect
		 * isn't used.
		 */
		unsigned int hpd_absent_delay;

		/**
		 * @delay.prepare_to_enable: Time between prepare and enable.
		 *
		 * The minimum time, in milliseconds, that needs to have passed
		 * between when prepare finished and enable may begin. If at
		 * enable time less time has passed since prepare finished,
		 * the driver waits for the remaining time.
		 *
		 * If a fixed enable delay is also specified, we'll start
		 * counting before delaying for the fixed delay.
		 *
		 * If a fixed prepare delay is also specified, we won't start
		 * counting until after the fixed delay. We can't overlap this
		 * fixed delay with the min time because the fixed delay
		 * doesn't happen at the end of the function if a HPD GPIO was
		 * specified.
		 *
		 * In other words:
		 *   prepare()
		 *     ...
		 *     // do fixed prepare delay
		 *     // wait for HPD GPIO if applicable
		 *     // start counting for prepare_to_enable
		 *
		 *   enable()
		 *     // do fixed enable delay
		 *     // enforce prepare_to_enable min time
		 */
		unsigned int prepare_to_enable;

		/**
		 * @delay.enable: Time for the panel to display a valid frame.
		 *
		 * The time (in milliseconds) that it takes for the panel to
		 * display the first valid frame after starting to receive
		 * video data.
		 */
		unsigned int enable;

		/**
		 * @delay.disable: Time for the panel to turn the display off.
		 *
		 * The time (in milliseconds) that it takes for the panel to
		 * turn the display off (no content is visible).
		 */
		unsigned int disable;

		/**
		 * @delay.unprepare: Time to power down completely.
		 *
		 * The time (in milliseconds) that it takes for the panel
		 * to power itself down completely.
		 *
		 * This time is used to prevent a future "prepare" from
		 * starting until at least this many milliseconds has passed.
		 * If at prepare time less time has passed since unprepare
		 * finished, the driver waits for the remaining time.
		 */
		unsigned int unprepare;
	} delay;
	/**
	* @bus_format: See MEDIA_BUS_FMT_... defines.
	*/
	u32 bus_format;
	/**
	* @bus_flags: See DRM_BUS_FLAG_... defines.
	*/
	u32 bus_flags;
};

struct panel_dp {
	struct drm_panel base;
	ktime_t prepared_time;
	ktime_t unprepared_time;
	const struct panel_desc *desc;
	struct regulator *supply;
	struct gpio_desc *enable_gpio;
	struct gpio_desc *hpd_gpio;
	struct edid *edid;
	struct drm_display_mode override_mode;
	enum drm_panel_orientation orientation;
	bool is_edp;
	bool enabled;
	bool no_hpd;
	bool prepared;
};

static inline struct panel_dp *to_panel_dp(struct drm_panel *panel)
{
	return container_of(panel, struct panel_dp, base);
}

static unsigned int panel_dp_get_timings_modes(struct panel_dp *panel,
						struct drm_connector *connector)
{
	struct drm_display_mode *mode;
	unsigned int i, num = 0;

	for (i = 0; i < panel->desc->num_timings; i++) {
		const struct display_timing *dt = &panel->desc->timings[i];
		struct videomode vm;

		videomode_from_timing(dt, &vm);
		mode = drm_mode_create(connector->dev);
		if (!mode) {
			dev_err(panel->base.dev, "failed to add mode %ux%u\n",
				dt->hactive.typ, dt->vactive.typ);
			continue;
		}

		drm_display_mode_from_videomode(&vm, mode);

		mode->type |= DRM_MODE_TYPE_DRIVER;

		if (panel->desc->num_timings == 1)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_probed_add(connector, mode);
		num++;
	}

	return num;
}

static unsigned int panel_dp_get_display_modes(struct panel_dp *panel,
						struct drm_connector *connector)
{
	struct drm_display_mode *mode;
	unsigned int i, num = 0;

	for (i = 0; i < panel->desc->num_modes; i++) {
		const struct drm_display_mode *m = &panel->desc->modes[i];

		mode = drm_mode_duplicate(connector->dev, m);
		if (!mode) {
			dev_err(panel->base.dev, "failed to add mode %ux%u@%u\n",
				m->hdisplay, m->vdisplay,
				drm_mode_vrefresh(m));
			continue;
		}

		mode->type |= DRM_MODE_TYPE_DRIVER;

		if (panel->desc->num_modes == 1)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_set_name(mode);

		drm_mode_probed_add(connector, mode);
		num++;
	}

	return num;
}

static int panel_dp_get_fixed_modes(struct panel_dp *panel,
					struct drm_connector *connector)
{
	struct drm_display_mode *mode;
	bool has_override = panel->override_mode.type;
	unsigned int num = 0;

	if (!panel->desc)
		return 0;

	if (has_override) {
		mode = drm_mode_duplicate(connector->dev,
					  &panel->override_mode);
		if (mode) {
			drm_mode_probed_add(connector, mode);
			num = 1;
		} else {
			dev_err(panel->base.dev, "failed to add override mode\n");
		}
	}

	/* Only add timings if override was not there or failed to validate */
	if (num == 0 && panel->desc->num_timings)
		num = panel_dp_get_timings_modes(panel, connector);

	/*
	 * Only add fixed modes if timings/override added no mode.
	 *
	 * We should only ever have either the display timings specified
	 * or a fixed mode. Anything else is rather bogus.
	 */
	WARN_ON(panel->desc->num_timings && panel->desc->num_modes);
	if (num == 0)
		num = panel_dp_get_display_modes(panel, connector);

	connector->display_info.bpc = panel->desc->bpc;
	connector->display_info.width_mm = panel->desc->size.width;
	connector->display_info.height_mm = panel->desc->size.height;

	switch (panel->desc->bus_format) {
		case MEDIA_BUS_FMT_RGB888_1X24:
			connector->display_info.color_formats
						= DRM_COLOR_FORMAT_RGB444;
					break;
		case MEDIA_BUS_FMT_UYYVYY8_0_5X24:
			connector->display_info.color_formats
						= DRM_COLOR_FORMAT_YCBCR420;
					break;
		case MEDIA_BUS_FMT_UYVY8_1X16:
			connector->display_info.color_formats
						= DRM_COLOR_FORMAT_YCBCR422;
					break;
		case MEDIA_BUS_FMT_YUV8_1X24:
			connector->display_info.color_formats
						= DRM_COLOR_FORMAT_YCBCR444;
					break;
		default: dev_info(panel->base.dev,
						"invalid bus_format value=0x%x\n",
						panel->desc->bus_format);

	}
	return num;
}

static void panel_dp_wait(ktime_t start_ktime, unsigned int min_ms)
{
	ktime_t now_ktime, min_ktime;

	if (!min_ms)
		return;

	min_ktime = ktime_add(start_ktime, ms_to_ktime(min_ms));
	now_ktime = ktime_get();

	if (ktime_before(now_ktime, min_ktime))
		msleep(ktime_to_ms(ktime_sub(min_ktime, now_ktime)) + 1);
}

static int panel_dp_disable(struct drm_panel *panel)
{
	struct panel_dp *p = to_panel_dp(panel);

	if (!p->enabled)
		return 0;

	if (p->desc->delay.disable)
		msleep(p->desc->delay.disable);

	p->enabled = false;

	return 0;
}

static int panel_dp_suspend(struct device *dev)
{
	struct panel_dp *p = dev_get_drvdata(dev);

	gpiod_set_value_cansleep(p->enable_gpio, 0);
	regulator_disable(p->supply);
	p->unprepared_time = ktime_get();

	kfree(p->edid);
	p->edid = NULL;

	return 0;
}

static int panel_dp_unprepare(struct drm_panel *panel)
{
	struct panel_dp *p = to_panel_dp(panel);
	int ret;

	/* Unpreparing when already unprepared is a no-op */
	if (!p->prepared)
		return 0;

	pm_runtime_mark_last_busy(panel->dev);
	ret = pm_runtime_put_autosuspend(panel->dev);
	if (ret < 0)
		return ret;
	p->prepared = false;

	return 0;
}

static int __maybe_unused panel_dp_get_hpd_gpio(struct device *dev, struct panel_dp *p)
{
	int err;

	p->hpd_gpio = devm_gpiod_get_optional(dev, "hpd", GPIOD_IN);
	if (IS_ERR(p->hpd_gpio)) {
		err = PTR_ERR(p->hpd_gpio);

		if (err != -EPROBE_DEFER)
			dev_err(dev, "failed to get 'hpd' GPIO: %d\n", err);

		return err;
	}

	return 0;
}

static int panel_dp_prepare_once(struct panel_dp *p)
{
	struct device *dev = p->base.dev;
	unsigned int delay;
	int err;
	int hpd_asserted;
	unsigned long hpd_wait_us;

	panel_dp_wait(p->unprepared_time, p->desc->delay.unprepare);

	err = regulator_enable(p->supply);
	if (err < 0) {
		dev_err(dev, "failed to enable supply: %d\n", err);
		return err;
	}

	gpiod_set_value_cansleep(p->enable_gpio, 1);

	delay = p->desc->delay.prepare;
	if (p->no_hpd)
		delay += p->desc->delay.hpd_absent_delay;
	if (delay)
		msleep(delay);

	if (p->hpd_gpio) {
		if (p->desc->delay.hpd_absent_delay)
			hpd_wait_us = p->desc->delay.hpd_absent_delay * 1000UL;
		else
			hpd_wait_us = 2000000;

		err = readx_poll_timeout(gpiod_get_value_cansleep, p->hpd_gpio,
					 hpd_asserted, hpd_asserted,
					 1000, hpd_wait_us);
		if (hpd_asserted < 0)
			err = hpd_asserted;

		if (err) {
			if (err != -ETIMEDOUT)
				dev_err(dev,
					"error waiting for hpd GPIO: %d\n", err);
			goto error;
		}
	}

	p->prepared_time = ktime_get();

	return 0;

error:
	gpiod_set_value_cansleep(p->enable_gpio, 0);
	regulator_disable(p->supply);
	p->unprepared_time = ktime_get();

	return err;
}

/*
 * Some panels simply don't always come up and need to be power cycled to
 * work properly.  We'll allow for a handful of retries.
 */
#define MAX_PANEL_PREPARE_TRIES		5

static int panel_dp_resume(struct device *dev)
{
	struct panel_dp *p = dev_get_drvdata(dev);
	int ret;
	int try;

	for (try = 0; try < MAX_PANEL_PREPARE_TRIES; try++) {
		ret = panel_dp_prepare_once(p);
		if (ret != -ETIMEDOUT)
			break;
	}

	if (ret == -ETIMEDOUT)
		dev_err(dev, "Prepare timeout after %d tries\n", try);
	else if (try)
		dev_warn(dev, "Prepare needed %d retries\n", try);

	return ret;
}

static int panel_dp_prepare(struct drm_panel *panel)
{
	struct panel_dp *p = to_panel_dp(panel);
	int ret;

	/* Preparing when already prepared is a no-op */
	if (p->prepared)
		return 0;

	ret = pm_runtime_get_sync(panel->dev);
	if (ret < 0) {
		pm_runtime_put_autosuspend(panel->dev);
		return ret;
	}

	p->prepared = true;

	return 0;
}

static int panel_dp_enable(struct drm_panel *panel)
{
	struct panel_dp *p = to_panel_dp(panel);

	if (p->enabled)
		return 0;

	if (p->desc->delay.enable)
		msleep(p->desc->delay.enable);

	panel_dp_wait(p->prepared_time, p->desc->delay.prepare_to_enable);

	p->enabled = true;

	return 0;
}

static int panel_dp_get_modes(struct drm_panel *panel,
				  struct drm_connector *connector)
{
	struct panel_dp *p = to_panel_dp(panel);
	int num = 0;

	/* add fixed panel modes */
	num += panel_dp_get_fixed_modes(p, connector);

	return num;
}

static const struct drm_panel_funcs panel_dp_funcs = {
	.disable = panel_dp_disable,
	.unprepare = panel_dp_unprepare,
	.prepare = panel_dp_prepare,
	.enable = panel_dp_enable,
	.get_modes = panel_dp_get_modes,
};

#define PANEL_DP_BOUNDS_CHECK(to_check, bounds, field) \
	(to_check->field.typ >= bounds->field.min && \
	 to_check->field.typ <= bounds->field.max)
static void panel_dp_parse_panel_timing_node(struct device *dev,
					      struct panel_dp *panel,
					      const struct display_timing *ot)
{
	const struct panel_desc *desc = panel->desc;
	struct videomode vm;
	unsigned int i;

	if (WARN_ON(desc->num_modes)) {
		dev_err(dev, "Reject override mode: panel has a fixed mode\n");
		return;
	}
	if (WARN_ON(!desc->num_timings)) {
		dev_err(dev, "Reject override mode: no timings specified\n");
		return;
	}

	for (i = 0; i < panel->desc->num_timings; i++) {
		const struct display_timing *dt = &panel->desc->timings[i];

		if (!PANEL_DP_BOUNDS_CHECK(ot, dt, hactive) ||
			!PANEL_DP_BOUNDS_CHECK(ot, dt, hfront_porch) ||
			!PANEL_DP_BOUNDS_CHECK(ot, dt, hback_porch) ||
			!PANEL_DP_BOUNDS_CHECK(ot, dt, hsync_len) ||
			!PANEL_DP_BOUNDS_CHECK(ot, dt, vactive) ||
			!PANEL_DP_BOUNDS_CHECK(ot, dt, vfront_porch) ||
			!PANEL_DP_BOUNDS_CHECK(ot, dt, vback_porch) ||
			!PANEL_DP_BOUNDS_CHECK(ot, dt, vsync_len))
			continue;

		if (ot->flags != dt->flags)
			continue;

		videomode_from_timing(ot, &vm);
		drm_display_mode_from_videomode(&vm, &panel->override_mode);
		panel->override_mode.type |= DRM_MODE_TYPE_DRIVER |
					     DRM_MODE_TYPE_PREFERRED;
		break;
	}

	if (WARN_ON(!panel->override_mode.type))
		dev_err(dev, "Reject override mode: No display_timing found\n");
}

static int panel_dp_probe(struct device *dev, const struct panel_desc *desc)
{
	struct panel_dp *panel;
	struct display_timing dt;

	panel = devm_kzalloc(dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	panel->enabled = false;
	panel->prepared_time = 0;
	panel->desc = desc;

	panel->is_edp = of_property_read_bool(dev->of_node, "is_edp");
    /*  TODO: open it when EVM*/
#if  0
	panel->no_hpd = of_property_read_bool(dev->of_node, "no-hpd");
	if (!panel->no_hpd) {
		err = panel_dp_get_hpd_gpio(dev, panel);
		if (err)
			return err;
	}

	panel->supply = devm_regulator_get(dev, "power");
	if (IS_ERR(panel->supply))
		return PTR_ERR(panel->supply);

	panel->enable_gpio = devm_gpiod_get_optional(dev, "enable",
					GPIOD_OUT_LOW);
	if (IS_ERR(panel->enable_gpio)) {
		err = PTR_ERR(panel->enable_gpio);
		if (err != -EPROBE_DEFER)
			dev_err(dev, "failed to request GPIO: %d\n", err);
		return err;
	}

	err = of_drm_get_panel_orientation(dev->of_node, &panel->orientation);
	if (err) {
		dev_err(dev, "%pOF: failed to get orientation %d\n", dev->of_node, err);
		return err;
	}
#endif

	if (!of_get_display_timing(dev->of_node, "panel-timing", &dt))
		panel_dp_parse_panel_timing_node(dev, panel, &dt);

	/* Catch common mistakes for panels. */
	if (desc->bpc != 8 && desc->bpc != 10)
		dev_warn(dev, "Expected bpc in {8,10} but got: %u\n", desc->bpc);

	dev_set_drvdata(dev, panel);

	/*
	 * We use runtime PM for prepare / unprepare since those power the panel
	 * on and off and those can be very slow operations. This is important
	 * to optimize powering the panel on briefly to read the EDID before
	 * fully enabling the panel.
	 */
#if 0
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
#endif
	if (panel->is_edp)
		drm_panel_init(&panel->base, dev, &panel_dp_funcs, DRM_MODE_CONNECTOR_eDP);
	else
		drm_panel_init(&panel->base, dev, &panel_dp_funcs, DRM_MODE_CONNECTOR_DisplayPort);
#if 0
	err = drm_panel_of_backlight(&panel->base);
	if (err)
		goto disable_pm_runtime;

	if (!panel->base.backlight) {
		pm_runtime_get_sync(dev);
		// TODO: do backlight control
		pm_runtime_mark_last_busy(dev);
		pm_runtime_put_autosuspend(dev);
		if (err)
			goto disable_pm_runtime;
	}
#endif
	drm_panel_add(&panel->base);
	dev_info(dev, "panel_dp_probe done bpc=%d, bus_format=%d\n", desc->bpc, desc->bus_format);

	return 0;
#if 0
disable_pm_runtime:
	pm_runtime_dont_use_autosuspend(dev);
	pm_runtime_disable(dev);
	return err;
#endif
}

static int panel_dp_remove(struct device *dev)
{
	struct panel_dp *panel = dev_get_drvdata(dev);

	drm_panel_remove(&panel->base);
	drm_panel_disable(&panel->base);
	drm_panel_unprepare(&panel->base);

	pm_runtime_dont_use_autosuspend(dev);
	pm_runtime_disable(dev);

	return 0;
}

static void panel_dp_shutdown(struct device *dev)
{
	struct panel_dp *panel = dev_get_drvdata(dev);

	drm_panel_disable(&panel->base);
	drm_panel_unprepare(&panel->base);
}

static const struct drm_display_mode boe_nv110wtm_n61_modes[] = {
	{
		.clock = 207800,
		.hdisplay = 2160,
		.hsync_start = 2160 + 48,
		.hsync_end = 2160 + 48 + 32,
		.htotal = 2160 + 48 + 32 + 100,
		.vdisplay = 1440,
		.vsync_start = 1440 + 3,
		.vsync_end = 1440 + 3 + 6,
		.vtotal = 1440 + 3 + 6 + 31,
		.flags = DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC,
	},
	{
		.clock = 138500,
		.hdisplay = 2160,
		.hsync_start = 2160 + 48,
		.hsync_end = 2160 + 48 + 32,
		.htotal = 2160 + 48 + 32 + 100,
		.vdisplay = 1440,
		.vsync_start = 1440 + 3,
		.vsync_end = 1440 + 3 + 6,
		.vtotal = 1440 + 3 + 6 + 31,
		.flags = DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC,
	},
};

static const struct panel_desc boe_nv110wtm_n61 = {
	.modes = boe_nv110wtm_n61_modes,
	.num_modes = ARRAY_SIZE(boe_nv110wtm_n61_modes),
	.bpc = 8,
	.size = {
		.width = 233,
		.height = 155,
	},
	.delay = {
		.hpd_absent_delay = 200,
		.prepare_to_enable = 80,
		.enable = 50,
		.unprepare = 500,
	},
	.bus_flags = DRM_BUS_FLAG_DE_HIGH,
	.bus_format = MEDIA_BUS_FMT_RGB888_1X24,
};

static const struct drm_display_mode bst_dp_panel_modes[] = {
};

static const struct display_timing bst_dp_panel_timing[] = {
	{
		.pixelclock = { 93600000, 93600000, 93600000 },
		.hactive = { 1920, 1920, 1920 },
		.hfront_porch = { 70, 70, 70 },
		.hback_porch = { 47, 47, 47 },
		.hsync_len = { 43, 43, 43 },
		.vactive = { 720, 720, 720 },
		.vfront_porch = { 3, 3, 3 },
		.vback_porch = { 24, 24, 24 },
		.vsync_len = { 3, 3, 3 },
		.flags = DISPLAY_FLAGS_VSYNC_LOW | DISPLAY_FLAGS_HSYNC_LOW,
	},
	{ /* 1920x1080@60Hz */
		.pixelclock = { 148500000, 148500000, 148500000 },
		.hactive = { 1920, 1920, 1920 },
		.hfront_porch = { 88, 88, 88 },
		.hback_porch = { 148, 148, 148 },
		.hsync_len = { 44, 44, 44 },
		.vactive = { 1080, 1080, 1080 },
		.vfront_porch = { 3, 4, 4 },
		.vback_porch = { 36, 36, 36 },
		.vsync_len = { 5, 5, 5 },
		.flags = DISPLAY_FLAGS_VSYNC_LOW | DISPLAY_FLAGS_HSYNC_LOW,
	},
	{ /* 3840x2160@30Hz */
		.pixelclock = { 297000000, 297000000, 297000000 },
		.hactive = { 3840, 3840, 3840 },
		.hfront_porch = { 176, 176, 176 },
		.hback_porch = { 296, 296, 296 },
		.hsync_len = { 88, 88, 88 },
		.vactive = { 2160, 2160, 2160 },
		.vfront_porch = { 8, 8, 8 },
		.vback_porch = { 72, 72, 72 },
		.vsync_len = { 10, 10, 10 },
		.flags = DISPLAY_FLAGS_VSYNC_HIGH | DISPLAY_FLAGS_HSYNC_HIGH,
	},
#if defined(CONFIG_C1200_SLT) || defined(CONFIG_C1200_MASS)
	{ /* 3840x2160@60Hz */
		.pixelclock = { 594000000, 594000000, 594000000 },
		.hactive = { 3840, 3840, 3840 },
		.hfront_porch = { 176, 176, 176 },
		.hback_porch = { 296, 296, 296 },
		.hsync_len = { 88, 88, 88 },
		.vactive = { 2160, 2160, 2160 },
		.vfront_porch = { 8, 8, 8 },
		.vback_porch = { 72, 72, 72 },
		.vsync_len = { 10, 10, 10 },
		.flags = DISPLAY_FLAGS_VSYNC_HIGH | DISPLAY_FLAGS_HSYNC_HIGH,
	},
#endif
	{ /* 2560x1440@59.95hz */
		.pixelclock = { 241500000, 241500000, 241500000 },
		.hactive = { 2560, 2560, 2560 },
		.hfront_porch = { 48, 48, 48 },
		.hback_porch = { 80, 80, 80 },
		.hsync_len = { 32, 32, 32 },
		.vactive = { 1440, 1440, 1440 },
		.vfront_porch = { 3, 3, 3 },
		.vback_porch = { 33, 33, 33 },
		.vsync_len = { 5, 5, 5 },
		.flags = DISPLAY_FLAGS_VSYNC_LOW | DISPLAY_FLAGS_HSYNC_HIGH,
	},
	{ /* 4096x2160@60Hz */
		.pixelclock = { 594000000, 594000000, 594000000 },
		.hactive = { 4096, 4096, 4096 },
		.hfront_porch = { 88, 88, 88 },
		.hback_porch = { 128, 128, 128 },
		.hsync_len = { 88, 88, 88 },
		.vactive = { 2160, 2160, 2160 },
		.vfront_porch = { 8, 8, 8 },
		.vback_porch = { 72, 72, 72 },
		.vsync_len = { 10, 10, 10 },
		.flags = DISPLAY_FLAGS_VSYNC_HIGH | DISPLAY_FLAGS_HSYNC_HIGH,
	},
	{ /* 1280x720@60Hz */
		.pixelclock = { 74250000, 74250000, 74250000 },
		.hactive = { 1280, 1280, 1280 },
		.hfront_porch = { 110, 110, 110 },
		.hback_porch = { 220, 220, 220 },
		.hsync_len = { 40, 40, 40 },
		.vactive = { 720, 720, 720 },
		.vfront_porch = { 5, 5, 5 },
		.vback_porch = { 20, 20, 20 },
		.vsync_len = { 5, 5, 5 },
		.flags = DISPLAY_FLAGS_VSYNC_HIGH | DISPLAY_FLAGS_HSYNC_HIGH,
	},
	{ /* 2560x1080@60Hz */
		.pixelclock = { 198000000, 198000000, 198000000 },
		.hactive = { 2560, 2560, 2560 },
		.hfront_porch = { 248, 248, 248 },
		.hback_porch = { 148, 148, 148 },
		.hsync_len = { 44, 44, 44 },
		.vactive = { 1080, 1080, 1080 },
		.vfront_porch = { 4, 4, 4 },
		.vback_porch = { 11, 11, 11 },
		.vsync_len = { 5, 5, 5 },
		.flags = DISPLAY_FLAGS_VSYNC_HIGH | DISPLAY_FLAGS_HSYNC_HIGH,
	},
};

static const struct panel_desc bst_dp_panel_rgb_24bit = {
	.timings = bst_dp_panel_timing,
	.num_timings = ARRAY_SIZE(bst_dp_panel_timing),
	.bpc = 8,
	.size = {
		.width = 410,
		.height = 230,
	},
	.bus_flags = DRM_BUS_FLAG_DE_HIGH,
	.bus_format = MEDIA_BUS_FMT_RGB888_1X24,
};

static const struct panel_desc bst_dp_panel_yuv422_16bit = {
	.modes = bst_dp_panel_modes,
	.num_modes = ARRAY_SIZE(bst_dp_panel_modes),
	.bpc = 8,
	.size = {
		.width = 260,
		.height = 120,
	},
	.delay = {
		.hpd_absent_delay = 200,
		.prepare_to_enable = 80,
		.enable = 50,
		.unprepare = 500,
	},
	.bus_flags = DRM_BUS_FLAG_DE_HIGH,
	.bus_format = MEDIA_BUS_FMT_UYVY8_1X16,
};

static const struct panel_desc bst_dp_sink_xtor_yuv420_12bit = {
	.modes = bst_dp_panel_modes,
	.num_modes = ARRAY_SIZE(bst_dp_panel_modes),
	.bpc = 8,
	.size = {
		.width = 260,
		.height = 120,
	},
	.delay = {
		.hpd_absent_delay = 200,
		.prepare_to_enable = 80,
		.enable = 50,
		.unprepare = 500,
	},
	.bus_flags = DRM_BUS_FLAG_DE_HIGH,
	.bus_format = MEDIA_BUS_FMT_UYYVYY8_0_5X24,
};

static const struct of_device_id platform_of_match[] = {
	{ .compatible = "bst,dp-panel", .data = &bst_dp_panel_rgb_24bit},
	{
		/* sentinel */
	}
};

MODULE_DEVICE_TABLE(of, platform_of_match);

static int panel_dp_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;

	id = of_match_node(platform_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	return panel_dp_probe(&pdev->dev, id->data);
}

static int panel_dp_platform_remove(struct platform_device *pdev)
{
	return panel_dp_remove(&pdev->dev);
}

static void panel_dp_platform_shutdown(struct platform_device *pdev)
{
	panel_dp_shutdown(&pdev->dev);
}

static const struct dev_pm_ops panel_dp_pm_ops = {
	SET_RUNTIME_PM_OPS(panel_dp_suspend, panel_dp_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};

static struct platform_driver panel_dp_platform_driver = {
	.driver = {
		.name = "panel-dp",
		.of_match_table = platform_of_match,
		.pm = &panel_dp_pm_ops,
	},
	.probe = panel_dp_platform_probe,
	.remove = panel_dp_platform_remove,
	.shutdown = panel_dp_platform_shutdown,
};


static int __init panel_dp_init(void)
{
	int err;

	err = platform_driver_register(&panel_dp_platform_driver);
	if (err < 0)
		return err;

	return 0;
}
module_init(panel_dp_init);

static void __exit panel_dp_exit(void)
{
	platform_driver_unregister(&panel_dp_platform_driver);
}
module_exit(panel_dp_exit);

MODULE_AUTHOR("BST Ltd.");
MODULE_DESCRIPTION("DRM Driver for Simple dp Panels");
MODULE_LICENSE("GPL");