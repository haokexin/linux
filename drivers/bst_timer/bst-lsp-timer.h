// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *  Copyright (C) 2017-2018 Synopsys, Inc. (www.synopsys.com)
 */

#ifndef _BST_LSP_TIMER_H
#define _BST_LSP_TIMER_H

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/types.h>


struct bst_timer_priv {
	struct device *dev;
	void __iomem *reg_base;
	int irq_num;
	int timer_chan;
};

#endif /* _BST_LSP_TIMER_H */
