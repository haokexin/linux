// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include "bst_hwcv_irq_manager.h"
#include "bst_hwcv_gwarp.h"
#include "bst_hwcv_scaler.h"

#define SCALER_INT_REG_BIT 0
#define SCALER_INT_MASK (1 << SCALER_INT_REG_BIT)

#define NORMAL_GWARP_INT_REG_BIT 0
#define SBS_GWARP0_INT_REG_BIT 8
// #define SBS_GWARP1_INT_REG_BIT			9
// #define SBS_GWARP2_INT_REG_BIT			10
// #define SBS_GWARP3_INT_REG_BIT			11

#define NORMAL_GWARP_INT_MASK (1 << NORMAL_GWARP_INT_REG_BIT)
#define SBS_GWARP0_INT_MASK (1 << SBS_GWARP0_INT_REG_BIT)

// #define SBS_GWARP1_INT_MASK				(1 << SBS_GWARP1_INT_REG_BIT)
// #define SBS_GWARP2_INT_MASK				(1 << SBS_GWARP2_INT_REG_BIT)
// #define SBS_GWARP3_INT_MASK				(1 << SBS_GWARP3_INT_REG_BIT)

static irqreturn_t bst_hwcv_interrupt_handler(int irq, void *dev_id)
{
	int i, j;
	int is_happend = 0;
	uint32_t scler_intr_reg = 0;
	uint32_t gwarp_intr_reg = 0;
	struct bst_hwcv_irq_manager *irq_manager = dev_id;
	struct device *dev = irq_manager->dev;

	dev_dbg(dev, "Top: int happened");
	scler_intr_reg = bst_scaler_read_intr(dev);
	if (scler_intr_reg & SCALER_INT_MASK) {
		set_bit(SCALER_INT_REG_BIT, &irq_manager->scaler_irq_status);
		bst_scaler_clear_intr(dev);
		is_happend = 1;
		dev_dbg(dev, "Scaler: Top int");
	}

	for (i = 0; i < BST_HWCV_GWARP_ENGINE_NUM; i++) {
		gwarp_intr_reg = bst_gwarp_read_intr(dev, i);
		if (gwarp_intr_reg & NORMAL_GWARP_INT_MASK) {
			set_bit(NORMAL_GWARP_INT_REG_BIT,
				&irq_manager->gwarp_irq_status[i]);
			bst_gwarp_clear_intr(dev, i);
			is_happend = 1;
			dev_dbg(dev, "Gwarp[%d]: Top int", i);
		} else {
			for (j = 0; j < BST_HWCV_GWARP_SNR_NUM; j++) {
				if (gwarp_intr_reg &
				    (SBS_GWARP0_INT_MASK << j)) {
					set_bit(SBS_GWARP0_INT_REG_BIT + j,
						&irq_manager
							 ->gwarp_irq_status[i]);
					bst_sbs_gwarp_clear_intr(dev, i, j);
					is_happend = 1;
					dev_dbg(dev, "Gwarp[%d][%d]: Top int",
						i, j);
				}
			}
		}
	}

	if (is_happend)
		return IRQ_WAKE_THREAD;

	dev_err(dev, "No Valid interrupt");
	return IRQ_NONE;
}

static irqreturn_t bst_hwcv_interrupt_thread(int irq, void *dev_id)
{
	int i, j;
	struct bst_hwcv_irq_manager *irq_manager = dev_id;
	struct device *dev = irq_manager->dev;

	dev_dbg(dev,
		"Bottom: int happened, scaler: 0x%08lx, gwarp0: 0x%08lx, gwarp1: 0x%08lx",
		irq_manager->scaler_irq_status,
		irq_manager->gwarp_irq_status[0],
		irq_manager->gwarp_irq_status[1]);

	if (irq_manager->scaler_irq_status & SCALER_INT_MASK) {
		clear_bit(SCALER_INT_REG_BIT, &irq_manager->scaler_irq_status);
		complete(&irq_manager->scaler_irq_complete);
		dev_dbg(dev, "Scaler: Bottom int");
	}

	for (i = 0; i < BST_HWCV_GWARP_ENGINE_NUM; i++) {
		if (irq_manager->gwarp_irq_status[i] & NORMAL_GWARP_INT_MASK) {
			clear_bit(NORMAL_GWARP_INT_REG_BIT,
				  &irq_manager->gwarp_irq_status[i]);
			complete(&irq_manager->gwarp_irq_complete[i]);
			dev_dbg(dev, "Gwarp[%d]: Bottom int", i);
		} else {
			for (j = 0; j < BST_HWCV_GWARP_SNR_NUM; j++) {
				if (irq_manager->gwarp_irq_status[i] &
				    (SBS_GWARP0_INT_MASK << j)) {
					clear_bit(
						SBS_GWARP0_INT_REG_BIT + j,
						&irq_manager
							 ->gwarp_irq_status[i]);
					complete(
						&irq_manager
							 ->sbs_gwarp_irq_complete
								 [i][j]);
					dev_dbg(dev,
						"Gwarp[%d][%d]: Bottom int", i,
						j);
				}
			}
		}
	}

	return IRQ_HANDLED;
}

int bst_hwcv_irq_manager_init(struct device *dev,
			      struct bst_hwcv_irq_manager *irq_manager)
{
	int i, j;
	int ret;
	struct platform_device *pdev;

	dev_info(dev, "Init hwcv irq manager.");
	irq_manager->dev = dev;
	pdev = container_of(dev, struct platform_device, dev);

	irq_manager->irq = platform_get_irq_byname(pdev, "bst-hwcv-irq");
	if (irq_manager->irq < 0) {
		dev_err(dev, "Failed to find irq in dts");
		return -EINVAL;
	}
	dev_info(dev, "Hwcv irq number: %d", irq_manager->irq);

	for (i = 0; i < BST_HWCV_GWARP_ENGINE_NUM; i++) {
		irq_manager->gwarp_irq_status[i] = 0;
		init_completion(&irq_manager->gwarp_irq_complete[i]);
	}
	for (i = 0; i < BST_HWCV_GWARP_ENGINE_NUM; i++) {
		for (j = 0; j < BST_HWCV_GWARP_SNR_NUM; j++) {
			init_completion(
				&irq_manager->sbs_gwarp_irq_complete[i][j]);
		}
	}

	irq_manager->scaler_irq_status = 0;
	init_completion(&irq_manager->scaler_irq_complete);

	ret = devm_request_threaded_irq(
		dev, irq_manager->irq, bst_hwcv_interrupt_handler,
		bst_hwcv_interrupt_thread,
		IRQF_TRIGGER_HIGH | IRQF_SHARED | IRQF_ONESHOT, dev_name(dev),
		irq_manager);
	if (ret < 0) {
		dev_err(dev, "Failed to request thread irq");
		return ret;
	}

	return 0;
}

void bst_hwcv_irq_manager_exit(struct bst_hwcv_irq_manager *irq_manager)
{
	struct device *dev = irq_manager->dev;

	dev_info(dev, "Exit hwcv irq manager.");
	devm_free_irq(dev, irq_manager->irq, irq_manager);
}
