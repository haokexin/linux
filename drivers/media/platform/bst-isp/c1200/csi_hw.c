// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2025 Black Sesame Technologies. All Rights Reserved.
 */

#include <dt-bindings/media/bst-isp.h>

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of_irq.h>

#ifdef CONFIG_BST_HEALTH_MONITOR
#include <bst/bst_common_api.h>
#endif

#include "csi_hw.h"

#include "csi_cdphy.h"
#include "csi_controller.h"
#include "csi_rx.h"

static void pre_config_phy(struct csi_device *csi)
{
	u32 val;

	/* Release host reset */
	csi_top_update(csi, R_TOP_CTRL, 0x1, 0x1);
	usleep_range(10, 11);
	csi_ctrl_update(csi, R_CTRL_PHY_SHUTDOWNZ, 0, 0x1);
	csi_ctrl_update(csi, R_CTRL_DPHY_RSTZ, 0, 0x1);
	csi_ctrl_update(csi, R_CTRL_CSI2_RESETN, 0, 0x1);
	usleep_range(10, 11);

	/* Set lanes' forcerxmode to 1, PPI8 for DPHY, PPI16 for CPHY */
	val = (csi->phy_if == IF_DPHY) ? 0x1 : 0x5;
	csi_top_update(csi, R_TOP_PHY_LANE0, val, 0xF);
	csi_top_update(csi, R_TOP_PHY_LANE1, val, 0xF);
	csi_top_update(csi, R_TOP_PHY_LANE2, val, 0xF);
	csi_top_update(csi, R_TOP_PHY_LANE3, val, 0xF);
	/* Enable PHY clock and forcerxmode clock */
	csi_top_update(csi, R_TOP_PHY_CLK, 0x5, 0x7);
	usleep_range(10, 11);

	csi_ctrl_update(csi, R_CTRL_PHY_MODE, csi->phy_if, 0x1);
	/* DPHY use PPI8, CPHY use PPI16 */
	csi_ctrl_update(csi, R_CTRL_PHY_CFG, (csi->phy_if == IF_DPHY) ? 0 : 1,
			0x1);
	csi_ctrl_update(csi, R_CTRL_N_LANES, csi->lane_num - 1, 0x7);

	usleep_range(10, 11);
	/* Release PHY reset */
	csi_top_update(csi, R_TOP_CTRL, 0x8, 0x8);
	usleep_range(10, 11);
}

static int post_config_phy(struct csi_device *csi)
{
	int retries;
	u32 val;
	u32 stop_state;
	u64 start;
	u64 stop;
	bool ok;
	struct device *dev;

	dev = csi->dev;

	/* Release PHY shutdown, reset, controller reset */
	csi_ctrl_update(csi, R_CTRL_PHY_SHUTDOWNZ, 0x1, 0x1);
	csi_ctrl_update(csi, R_CTRL_DPHY_RSTZ, 0x1, 0x1);
	csi_ctrl_update(csi, R_CTRL_CSI2_RESETN, 0x1, 0x1);
	usleep_range(10, 11);

	/* Wait PHY ready */
	start = ktime_get_boottime_ns();
	retries = 0;
	ok = false;
	do {
		val = csi_top_get(csi, R_TOP_SYS_RD);
		ok = val & 0x80000000;
		if (ok)
			break;
		++retries;
		usleep_range(PHY_WAIT_DELAY, PHY_WAIT_DELAY + 1);
	} while (retries < PHY_WAIT_TIMES);
	stop = ktime_get_boottime_ns();
	dev_info(dev, "Waiting PHY ready cost %11lluns, val: 0x%08X, %s\n",
		 (stop - start), val, ok ? "PASSED" : "FAILED");
	if (!ok)
		return -EIO;

	/* Wait stop state ready */
	if (csi->lane_num == 1)
		stop_state = 0x110;
	else if (csi->lane_num == 2)
		stop_state = 0x130;
	else if (csi->lane_num == 3)
		stop_state = 0x170;
	else
		stop_state = 0x1F0;
	start = ktime_get_boottime_ns();
	retries = 0;
	do {
		val = csi_top_get(csi, R_TOP_IPTEST0);
		ok = ((val & 0x1F0) == stop_state);
		if (ok)
			break;
		++retries;
		usleep_range(PHY_WAIT_DELAY, PHY_WAIT_DELAY + 1);
	} while (retries < PHY_WAIT_TIMES);
	stop = ktime_get_boottime_ns();
	dev_info(dev, "Waiting stop state cost %10lluns, val: 0x%08X, %s\n",
		 (stop - start), val, ok ? "PASSED" : "FAILED");
	if (!ok)
		return -EIO;

	/* Release forcerxmode */
	val = (csi->phy_if == IF_DPHY) ? 0x0 : 0x4;
	csi_top_update(csi, R_TOP_PHY_LANE0, val, 0xF);
	csi_top_update(csi, R_TOP_PHY_LANE1, val, 0xF);
	csi_top_update(csi, R_TOP_PHY_LANE2, val, 0xF);
	csi_top_update(csi, R_TOP_PHY_LANE3, val, 0xF);
	/* Enable PHY clock and disable forcerxmode clock */
	csi_top_update(csi, R_TOP_PHY_CLK, 0x1, 0x7);
	usleep_range(10, 11);

	/* Enable IDI counter */
	csi_top_update(csi, R_TOP_IDI_OUT2, 0x80000000, 0x80000000);

	/* Check DPHY clock lane */
	if (csi->phy_if == IF_DPHY)
		return csi_ctrl_get(csi, R_CTRL_PHY_RX) & BIT(17);

	return 0;
}

static int config_phy(struct csi_device *csi)
{
	pre_config_phy(csi);
	if (csi->phy_if == IF_DPHY)
		csi_phy_config_dphy(csi);
	else
		csi_phy_config_cphy(csi);
	return post_config_phy(csi);
}

void csi_top_func_irq_clear(struct csi_device *csi)
{
	csi_top_update(csi, R_TOP_INT_FUNC_CLR, TOP_FUNC_EN_BITS,
		       TOP_FUNC_EN_BITS);
	csi_top_update(csi, R_TOP_INT_FUNC_CLR, 0, TOP_FUNC_EN_BITS);
	csi_top_update(csi, R_TOP_INT_CONFIG_CLR, BIT(28), BIT(28));
	csi_top_update(csi, R_TOP_INT_CONFIG_CLR, 0, BIT(28));
}

void csi_top_func_irq_enable(struct csi_device *csi)
{
	csi_top_update(csi, R_TOP_INT_FUNC_MUX_SEL, 0, TOP_FUNC_EN_BITS);
	csi_top_update(csi, R_TOP_INT_FUNC_EN, TOP_FUNC_EN_BITS,
		       TOP_FUNC_EN_BITS);
}

void csi_top_func_irq_disable(struct csi_device *csi)
{
	csi_top_update(csi, R_TOP_INT_FUNC_EN, 0, TOP_FUNC_EN_BITS);
}

void csi_top_diag_irq_clear(struct csi_device *csi)
{
	csi_top_update(csi, R_TOP_INT_DIAG_CLR, TOP_DIAG_EN_BITS,
		       TOP_DIAG_EN_BITS);
	csi_top_update(csi, R_TOP_INT_DIAG_CLR, 0, TOP_DIAG_EN_BITS);
}

void csi_top_diag_irq_enable(struct csi_device *csi)
{
	csi_top_update(csi, R_TOP_INT_DIAG_MUX_SEL, 0, TOP_DIAG_EN_BITS);
	csi_top_update(csi, R_TOP_INT_DIAG_EN, TOP_DIAG_EN_BITS,
		       TOP_DIAG_EN_BITS);
}

void csi_top_diag_irq_disable(struct csi_device *csi)
{
	csi_top_update(csi, R_TOP_INT_DIAG_EN, 0, TOP_DIAG_EN_BITS);
}

static irqreturn_t func_irq_handler(int irq, void *p)
{
	struct csi_device *csi;
	struct device *dev;
	u32 st_main;
	u32 top_idi_timeout;
	u32 top_func_state;
	ktime_t now;
	static DEFINE_RATELIMIT_STATE(_rs, CSI_RATELIMIT_INTERVAL,
				      CSI_RATELIMIT_BURST);

	csi = (struct csi_device *)p;
	dev = csi->dev;

	/* Read top state first, since controller's status are read-cleared,
	 * may cause status change.
	 */
	top_func_state = csi_top_get(csi, R_TOP_INT_FUNC_STATE);
	top_idi_timeout = csi_top_get(csi, R_TOP_IDI_TIMEOUT);
	st_main = csi_ctrl_get(csi, R_CTRL_ST_MAIN);

	csi_ctrl_func_irq_clear(csi);
	csi_top_func_irq_clear(csi);

	if (__ratelimit(&_rs))
		dev_err(dev,
			"TOP func: 0x%08X, idi: 0x%08X, CTRL main: 0x%08X\n",
			top_func_state, top_idi_timeout, st_main);

	/* Only IDI timeout occured, this is caused by TX, do nothing */
	if (st_main == 0)
		return IRQ_HANDLED;

	++csi->error_total;
	if (!csi->recoverable)
		return IRQ_HANDLED;

	now = ktime_get_boottime();
	++csi->error_window;
	if (csi->error_window == 1)
		csi->error_start = now;

	/* Current time is out of statistic window, do nothing */
	if (ktime_after(now,
			ktime_add(csi->error_start, csi->recover_window))) {
		dev_dbg_ratelimited(
			dev, "Error is out of statistic window, recount\n");
		csi->error_window = 1;
		csi->error_start = now;
		return IRQ_HANDLED;
	}

	/* Error count is less than threshold, do nothing */
	if (csi->error_window < csi->recover_threshold)
		return IRQ_HANDLED;

	++csi->recover_count;
	dev_crit_ratelimited(dev, "Recover %03u\n", csi->recover_count);
	v4l2_subdev_call(csi->remote_sd, video, pre_streamon, 0);
	dev_info(dev, "Re-initialize CDPHY\n");
	mutex_lock(&csi->lock);
	config_phy(csi);
	if (csi->used_vcs > 0) {
		v4l2_subdev_call(csi->remote_sd, video, s_stream,
				 STREAM_ENC(0, 1));
		if (csi->func_irq_enable) {
			csi_top_func_irq_enable(csi);
			csi_ctrl_func_irq_enable(csi);
		}
		/* Enable diag IRQ always, since it's handled by safety subsystem */
		csi_top_diag_irq_enable(csi);
		csi_ctrl_diag_irq_enable(csi);
	}
	mutex_unlock(&csi->lock);

	/* Reset statistic */
	csi->error_window = 0;

	return IRQ_HANDLED;
}

static irqreturn_t diag_irq_handler(int irq, void *p)
{
	struct csi_device *csi;
	struct device *dev;

	csi = (struct csi_device *)p;
	dev = csi->dev;

	dev_err_ratelimited(dev, "Diag IRQ trigged\n");

	return IRQ_HANDLED;
}

static void config_irq(struct csi_device *csi)
{
	int rv;
	struct device *dev;

	dev = csi->dev;
	csi_top_func_irq_clear(csi);
	csi_ctrl_func_irq_clear(csi);
	if (csi->func_irq_enable) {
		rv = devm_request_threaded_irq(dev, csi->func_irq, NULL,
					       func_irq_handler,
					       IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
					       dev_name(dev), csi);
		if (rv) {
			dev_err(dev, "Failed to request func irq %d, rv: %d\n",
				csi->func_irq, rv);
			csi->func_irq_enable = false;
		}
	}

	csi_top_diag_irq_clear(csi);
	csi_ctrl_diag_irq_clear(csi);
	if (csi->diag_irq_enable) {
		rv = devm_request_threaded_irq(dev, csi->diag_irq, NULL,
					       diag_irq_handler,
					       IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
					       dev_name(dev), csi);
		if (rv) {
			dev_err(dev, "Failed to request diag irq %d, rv: %d\n",
				csi->diag_irq, rv);
			csi->diag_irq_enable = false;
		}
	}
}

/* Caller must implement mutual exclusion protection for csi */
u32 csi_top_get(struct csi_device *csi, u32 reg)
{
	u32 val;

	val = readl_relaxed(csi->top_base + reg);
	dev_dbg(csi->dev, "CTGET: 0x%08X -> 0x%08X\n", reg, val);

	return val;
}

/* Caller must implement mutual exclusion protection for csi */
int csi_top_set(struct csi_device *csi, u32 reg, u32 val)
{
	u32 rval;
	int i;

	dev_dbg(csi->dev, "CTSET: 0x%08X -> 0x%08X\n", reg, val);
	i = 0;
	do {
		writel_relaxed(val, csi->top_base + reg);
		rval = readl_relaxed(csi->top_base + reg);
		if (rval == val)
			return 0;
	} while (++i <= csi->host_access_retries);

#ifdef CONFIG_BST_HEALTH_MONITOR
	if (csi->psm.host_access_confirm)
		send_dtc_to_safety_svc(MKDTC(csi->id, PSM_ID_HOST_REG_ACCESS));
#endif
	dev_err(csi->dev, "CTSET: 0x%08X -> 0x%08X vs 0x%08X, FAILED\n", reg,
		rval, val);

	return -EIO;
}

/* Caller must implement mutual exclusion protection for csi */
int csi_top_update(struct csi_device *csi, u32 reg, u32 val, u32 mask)
{
	u32 rval;
	u32 wval;
	int i;

	rval = readl_relaxed(csi->top_base + reg);
	wval = (rval & ~mask) | (val & mask);
	dev_dbg(csi->dev,
		"CTUP: 0x%08X -> 0x%08X & 0x%08X, rval: 0x%08X, wval: 0x%08X\n",
		reg, val, mask, rval, wval);
	i = 0;
	do {
		writel_relaxed(wval, csi->top_base + reg);
		rval = readl_relaxed(csi->top_base + reg);
		if (rval == wval)
			return 0;
	} while (++i <= csi->host_access_retries);

#ifdef CONFIG_BST_HEALTH_MONITOR
	if (csi->psm.host_access_confirm)
		send_dtc_to_safety_svc(MKDTC(csi->id, PSM_ID_HOST_REG_ACCESS));
#endif
	dev_err(csi->dev,
		"CTUP: 0x%08X -> 0x%08X & 0x%08X, rval: 0x%08X, wval: 0x%08X, FAILED\n",
		reg, val, mask, rval, wval);

	return -EIO;
}

bool csi_hw_has_inited(struct csi_device *csi)
{
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	u32 val;
	struct device *dev;

	dev = csi->dev;
	val = csi_top_get(csi, R_TOP_PHY_CLK);
	dev_info(dev, "CSIINIT: TOP_CLK: 0x%08X\n", val);
	if ((val & 0x7) == 0x1)
		return true;
	else
		return false;
#else /* Return false to re-configure hardware always */
	return false;
#endif
}

int csi_hw_init(struct csi_device *csi)
{
	int rv;
	struct device *dev;
	int i;

	csi->inited = false;
	if (csi_hw_has_inited(csi)) {
		config_irq(csi);
		csi->inited = true;

		return 0;
	}

	dev = csi->dev;
	dev_info(dev, "Initialize CDPHY\n");
	i = 0;
	do {
		v4l2_subdev_call(csi->remote_sd, video, pre_streamon, 0);
		rv = config_phy(csi);
		if (!rv)
			break;
		usleep_range(5000, 5001);
	} while (++i < CSI_HW_INIT_RETRIES);
	config_irq(csi);
	if (rv)
		dev_err(dev, "Failed to initialize CDPHY\n");
	else
		csi->inited = true;

	return rv;
}

void csi_hw_exit(struct csi_device *csi)
{
	if (csi->func_irq_enable) {
		csi_top_func_irq_disable(csi);
		csi_ctrl_func_irq_disable(csi);
		csi_top_func_irq_clear(csi);
		csi_ctrl_func_irq_clear(csi);
		disable_irq(csi->func_irq);
		devm_free_irq(csi->dev, csi->func_irq, csi);
	}
#ifndef CONFIG_VIDEO_BST_ISP_MULTI_OS
	csi_top_diag_irq_disable(csi);
	csi_ctrl_diag_irq_disable(csi);
	csi_top_diag_irq_clear(csi);
	csi_ctrl_diag_irq_clear(csi);
	if (csi->diag_irq_enable) {
		disable_irq(csi->diag_irq);
		devm_free_irq(csi->dev, csi->diag_irq, csi);
	}
	csi_ctrl_update(csi, R_CTRL_PHY_SHUTDOWNZ, 0, 0x1);
	csi_ctrl_update(csi, R_CTRL_DPHY_RSTZ, 0, 0x1);
	csi_ctrl_update(csi, R_CTRL_CSI2_RESETN, 0, 0x1);
	csi_top_update(csi, R_TOP_PHY_CLK, 0x0, 0x7);
#endif
}

void csi_hw_reset(struct csi_device *csi)
{
	csi_hw_exit(csi);
	csi_top_update(csi, R_TOP_PHY_CLK, 0x0, 0x7);
}
