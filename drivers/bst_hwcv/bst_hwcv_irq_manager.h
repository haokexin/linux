/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_HWCV_IRQ_MANAGER_H__
#define __BST_HWCV_IRQ_MANAGER_H__
#include "bst_hwcv_common.h"

struct bst_hwcv_irq_manager {
	int irq;
	struct device *dev;
	unsigned long gwarp_irq_status[BST_HWCV_GWARP_ENGINE_NUM];
	struct completion gwarp_irq_complete[BST_HWCV_GWARP_ENGINE_NUM];
	struct completion sbs_gwarp_irq_complete[BST_HWCV_GWARP_ENGINE_NUM]
						[BST_HWCV_GWARP_SNR_NUM];
	unsigned long scaler_irq_status;
	struct completion scaler_irq_complete;
};

int bst_hwcv_irq_manager_init(struct device *dev,
			      struct bst_hwcv_irq_manager *irq_manager);
void bst_hwcv_irq_manager_exit(struct bst_hwcv_irq_manager *irq_manager);

#endif
