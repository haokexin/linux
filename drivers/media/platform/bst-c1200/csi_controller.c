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

#include "csi2_rx.h"
#include "csi_cdphy.h"
#include "csi_controller.h"

#define ERROR_RECOVER_THRESHOLD 40
#define ERROR_RECOVER_REGION	2000 /* by ms */

static irqreturn_t csi_function_irq_handler(int32_t irq, void *p)
{
	struct bst_csi_device *csi_dev;
	struct device *dev;
	uint32_t value;

	csi_dev = (struct bst_csi_device *)p;
	dev = csi_dev->dev;

	value = readl_relaxed(csi_dev->ctrl_base + REG_INT_ST_MAIN);
	dev_err_ratelimited(dev, "IRQ %d trigged, Status main: 0x%08X\n", irq,
			    value);

	return IRQ_HANDLED;
}

static irqreturn_t csi_fmeda_irq_handler(int32_t irq, void *p)
{
	struct bst_csi_device *csi_dev;
	struct device *dev;
	uint32_t value;

	csi_dev = (struct bst_csi_device *)p;
	dev = csi_dev->dev;

	value = readl_relaxed(csi_dev->ctrl_base + REG_INT_ST_AP_MAIN);
	dev_err_ratelimited(dev, "IRQ %d trigged, Status AP main: 0x%08X\n",
			    irq, value);
	csi_dev->error_count++;
	if (csi_dev->error_count == 1)
		csi_dev->error_ts = jiffies;

	/*  If error occured ERROR_RECOVER_THRESHOLD times
	 *  in ERROR_RECOVER_REGION msecs, reset CSI to recover.
	 */
	if (csi_dev->error_count >= ERROR_RECOVER_THRESHOLD) {
		if (time_before(
			    jiffies,
			    csi_dev->error_ts +
				    msecs_to_jiffies(ERROR_RECOVER_REGION))) {
			csi_dev->recover_times++;
			dev_crit_ratelimited(dev, "try %u times to recover\n",
					     csi_dev->recover_times);
			if (csi_dev->deser->enter_csi_recover)
				csi_dev->deser->enter_csi_recover(
					csi_dev->deser);
			csi_cdphy_config_lanes(csi_dev);
			if (csi_dev->deser->exit_csi_recover)
				csi_dev->deser->exit_csi_recover(
					csi_dev->deser);
		}
		csi_dev->error_count = 0;
	}

	return IRQ_HANDLED;
}

void controller_enable_function_irq(struct bst_csi_device *csi_dev)
{
	struct platform_device *pdev;
	struct device *dev;
	int ret;
	uint32_t value;

	pdev = csi_dev->pdev;
	dev = csi_dev->dev;

	if (IS_ERR_OR_NULL(csi_dev->ctrl_base)) {
		dev_warn(dev, "%s: No controller base defined\n", __func__);
		return;
	}

	if (csi_dev->function_irq < 0) {
		dev_warn(dev, "%s: No function IRQ defined\n", __func__);
		return;
	}
	ret = devm_request_threaded_irq(dev, csi_dev->function_irq, NULL,
					csi_function_irq_handler,
					IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
					dev_name(dev), csi_dev);
	if (ret) {
		dev_err(dev, "failed to request function irq %d, ret = %d",
			csi_dev->function_irq, ret);
		return;
	}

	value = readl_relaxed(csi_dev->ctrl_base + REG_INT_MSK_PHY_FATAL);
	value |= 0x0FF;
	writel_relaxed(value, csi_dev->ctrl_base + REG_INT_MSK_PHY_FATAL);

	value = readl_relaxed(csi_dev->ctrl_base + REG_INT_MSK_PKT_FATAL);
	value |= 0x1;
	writel_relaxed(value, csi_dev->ctrl_base + REG_INT_MSK_PKT_FATAL);

	value = readl_relaxed(csi_dev->ctrl_base + REG_INT_MSK_PHY);
	value |= 0x00FF00FF;
	writel_relaxed(value, csi_dev->ctrl_base + REG_INT_MSK_PHY);

	value = readl_relaxed(csi_dev->ctrl_base + REG_INT_MSK_LINE);
	value |= 0x00FF00FF;
	writel_relaxed(value, csi_dev->ctrl_base + REG_INT_MSK_LINE);

	writel_relaxed(0xFFFFFFFF,
		       csi_dev->ctrl_base + REG_INT_MSK_BNDRY_FRAME_FATAL);

	writel_relaxed(0xFFFFFFFF,
		       csi_dev->ctrl_base + REG_INT_MSK_SEQ_FRAME_FATAL);

	writel_relaxed(0xFFFFFFFF,
		       csi_dev->ctrl_base + REG_INT_MSK_CRC_FRAME_FATAL);

	writel_relaxed(0xFFFFFFFF,
		       csi_dev->ctrl_base + REG_INT_MSK_PLD_CRC_FATAL);

	writel_relaxed(0xFFFFFFFF, csi_dev->ctrl_base + REG_INT_MSK_DATA_ID);

	writel_relaxed(0xFFFFFFFF,
		       csi_dev->ctrl_base + REG_INT_MSK_ECC_CORRECT);
}

void controller_enable_fmeda_irq(struct bst_csi_device *csi_dev)
{
	struct platform_device *pdev;
	struct device *dev;
	int ret;
	uint32_t value;

	pdev = csi_dev->pdev;
	dev = csi_dev->dev;

	if (IS_ERR_OR_NULL(csi_dev->ctrl_base)) {
		dev_warn(dev, "%s: No controller base defined\n", __func__);
		return;
	}

	if (csi_dev->fmeda_irq < 0) {
		dev_warn(dev, "%s: No fmeda IRQ defined\n", __func__);
		return;
	}
	ret = devm_request_threaded_irq(dev, csi_dev->fmeda_irq, NULL,
					csi_fmeda_irq_handler,
					IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
					dev_name(dev), csi_dev);
	if (ret) {
		dev_err(dev, "failed to request fmeda irq %d, ret = %d",
			csi_dev->fmeda_irq, ret);
		return;
	}

	value = readl_relaxed(csi_dev->ctrl_base + REG_INT_MSK_AP_GENERIC);
	value |= 0xFF5F5;
	writel_relaxed(value, csi_dev->ctrl_base + REG_INT_MSK_AP_GENERIC);

	value = readl_relaxed(csi_dev->ctrl_base + REG_INT_MSK_FAP_PHY_FATAL);
	value |= 0x700FF;
	writel_relaxed(value, csi_dev->ctrl_base + REG_INT_MSK_FAP_PHY_FATAL);

	value = readl_relaxed(csi_dev->ctrl_base + REG_INT_MSK_FAP_PKT_FATAL);
	value |= 0x1;
	writel_relaxed(value, csi_dev->ctrl_base + REG_INT_MSK_FAP_PKT_FATAL);

	value = readl_relaxed(csi_dev->ctrl_base + REG_INT_MSK_FAP_PHY);
	value |= 0x00FF00FF;
	writel_relaxed(value, csi_dev->ctrl_base + REG_INT_MSK_FAP_PHY);

	value = readl_relaxed(csi_dev->ctrl_base + REG_INT_MSK_FAP_LINE);
	value |= 0x00FF00FF;
	writel_relaxed(value, csi_dev->ctrl_base + REG_INT_MSK_FAP_LINE);

	writel_relaxed(0xFFFFFFFF,
		       csi_dev->ctrl_base + REG_INT_MSK_FAP_BNDRY_FRAME_FATAL);

	writel_relaxed(0xFFFFFFFF,
		       csi_dev->ctrl_base + REG_INT_MSK_FAP_SEQ_FRAME_FATAL);

	writel_relaxed(0xFFFFFFFF,
		       csi_dev->ctrl_base + REG_INT_MSK_FAP_CRC_FRAME_FATAL);

	writel_relaxed(0xFFFFFFFF,
		       csi_dev->ctrl_base + REG_INT_MSK_FAP_PLD_CRC_FATAL);

	writel_relaxed(0xFFFFFFFF,
		       csi_dev->ctrl_base + REG_INT_MSK_FAP_DATA_ID);

	writel_relaxed(0xFFFFFFFF,
		       csi_dev->ctrl_base + REG_INT_MSK_FAP_ECC_CORRECT);
}
