// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2025 Black Sesame Technologies. All Rights Reserved.
 */

#include <dt-bindings/media/bst-isp.h>

#include <linux/sysfs.h>

#include "csi_sysfs.h"

#include "csi_controller.h"
#include "csi_rx.h"
#include "csi_hw.h"

static ssize_t info_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct csi_device *csi;
	int len;
	u32 ver;
	u8 *p;

	csi = dev_get_drvdata(dev);
	ver = csi_ctrl_get(csi, R_CTRL_VERSION);
	p = (u8 *)&ver;
	len = 0;
	len += snprintf(buf + len, PAGE_SIZE - len, "%14s:     %c.%c.%c.%c\n",
			"Controller Ver", p[3], p[2], p[1], p[0]);
	len += snprintf(buf + len, PAGE_SIZE - len, "%14s: %10s\n", "PHY IF",
			(csi->phy_if == IF_DPHY) ?
				"DPHY" :
				((csi->phy_if == IF_CPHY) ? "CPHY" :
							    "Unknown"));
	len += snprintf(buf + len, PAGE_SIZE - len, "%14s: %10u\n", "Lanes",
			csi->lane_num);
	len += snprintf(buf + len, PAGE_SIZE - len, "%14s: %10u\n", "Speed",
			csi->lane_speed);
	len += snprintf(buf + len, PAGE_SIZE - len, "%14s: %10u\n", "EQ",
			csi->eq);
	len += snprintf(buf + len, PAGE_SIZE - len, "%14s: %10s\n", "Inited",
			csi->inited ? "true" : "false");
	len += snprintf(buf + len, PAGE_SIZE - len, "%14s: %10s\n",
			"Recoverable", csi->recoverable ? "true" : "false");
	len += snprintf(buf + len, PAGE_SIZE - len, "%14s: %10u\n", "Threshold",
			csi->recover_threshold);
	len += snprintf(buf + len, PAGE_SIZE - len, "%14s: %10uns\n", "Window",
			csi->recover_window);
	len += snprintf(buf + len, PAGE_SIZE - len, "%14s: %10u\n",
			"Error Total", csi->error_total);
	len += snprintf(buf + len, PAGE_SIZE - len, "%14s: %10u\n",
			"Error Window", csi->error_window);
	len += snprintf(buf + len, PAGE_SIZE - len, "%14s: %10u\n", "Used VC",
			csi->used_vcs);

	return len;
}

static struct device_attribute info_attr = __ATTR(info, 0644, info_show, NULL);

static ssize_t reset_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct csi_device *csi;
	int rv;
	u32 phy_if;
	u32 lanes;
	u32 speed;
	u32 eq;

	csi = dev_get_drvdata(dev);
	rv = sscanf(buf, "%u %u %u %u", &phy_if, &lanes, &speed, &eq);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
	switch (rv) {
	case 1:
		lanes = csi->lane_num;
	case 2:
		speed = csi->lane_speed;
	case 3:
		eq = csi->eq;
	case 4:
		break;
	default:
		dev_err(dev,
			"Invalid cfg: %s, should be <if> [lanes] [speed] [eq]\n",
			buf);
		return -EINVAL;
	}
#pragma GCC diagnostic pop

	if (phy_if == IF_DPHY) {
		if (lanes == 0 || lanes > CSI_MAX_DPHY_LANES) {
			dev_err(dev, "Invalid lane: %u, max: %u\n", lanes,
				CSI_MAX_DPHY_LANES);
			return -EINVAL;
		}
		if (speed > CSI_MAX_DPHY_SPEED) {
			dev_err(dev, "Invalid speed: %u, max: %u\n", speed,
				CSI_MAX_DPHY_SPEED);
			return -EINVAL;
		}
	} else if (phy_if == IF_CPHY) {
		if (lanes == 0 || lanes > CSI_MAX_CPHY_LANES) {
			dev_err(dev, "Invalid lane: %u, max: %u\n", lanes,
				CSI_MAX_CPHY_LANES);
			return -EINVAL;
		}
		if (speed > CSI_MAX_CPHY_SPEED) {
			dev_err(dev, "Invalid speed: %u, max: %u\n", speed,
				CSI_MAX_CPHY_SPEED);
			return -EINVAL;
		}
	} else {
		dev_err(dev, "Invalid PHY IF: %u\n", phy_if);
		return -EINVAL;
	}

	if (eq > CSI_MAX_EQ) {
		dev_err(dev, "Invalid eq: %u, max: %u\n", speed, CSI_MAX_EQ);
		return -EINVAL;
	}

	csi->phy_if = phy_if;
	csi->lane_num = lanes;
	csi->lane_speed = speed / 100 * 100; /* Round to 100 */
	csi->eq = eq;

	mutex_lock(&csi->lock);
	csi_hw_reset(csi);
	rv = csi_hw_init(csi);
	mutex_unlock(&csi->lock);
	if (rv)
		return -EIO;

	return count;
}

static struct device_attribute reset_attr =
	__ATTR(reset, 0644, NULL, reset_store);

/* clang-format off */
static struct attribute *attrs[] = {
	&info_attr.attr,
	&reset_attr.attr,
	NULL,
};
/* clang-format on */

static const struct attribute_group attr_group = {
	.attrs = attrs,
};

int csi_sysfs_init(struct csi_device *csi)
{
	int rv;
	struct device *dev;

	dev = csi->dev;

	rv = sysfs_create_group(&dev->kobj, &attr_group);
	if (rv) {
		dev_err(dev, "Failed to create sysfs group, rv: %d\n", rv);
		return rv;
	}

	return 0;
}

void csi_sysfs_exit(struct csi_device *csi)
{
	sysfs_remove_group(&csi->dev->kobj, &attr_group);
}
