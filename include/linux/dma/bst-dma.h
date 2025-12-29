/* SPDX-License-Identifier: GPL-2.0 */
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *  Copyright (C) 2017-2018 Synopsys, Inc. (www.synopsys.com)
 */

#ifndef _DMA_BST_H
#define _DMA_BST_H

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dmaengine.h>

/**
 * struct bst_dma_snd_peripheral_cfg - - snd config for peripheral
 * @dma_transfer_mode:	determine gdma transfer mode
 */

struct bst_dma_snd_peripheral_cfg {
    u32 dma_transfer_mode;
};

#endif /* _DMA_BST_H */
