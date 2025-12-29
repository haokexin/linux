// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#ifndef __DPTX_PHY_H__
#define __DPTX_PHY_H__

#include "dptx_drv.h"

enum dptx_phy_boot_type {
	ONLY_ROM,
	ONLY_ROM_TRIG,
	ONLY_SRAM,
	OVERRD_SRAM,
	USE_SRAM_PREV_TRIG,
	OVERRD_SRAM_TRIG,
	ROM_TO_SRAM_TRIG,
	SRAM_PREV_POR_TRIG,
	SRAM_PRE_FLASHED_TRIG,
};
void dptx_u3_phy_write_reg(struct dptx *dptx, uint32_t addr,
				     uint32_t data);
uint32_t dptx_u3_phy_read_reg(struct dptx *dptx, uint32_t addr);

void dptx_phy_firmware_load_from(struct dptx *dptx,
				     enum dptx_phy_boot_type type);
int dptx_wait_phy_boot_done(struct dptx *dptx, enum dptx_phy_boot_type boot_type);
#endif /* __DPTX_PHY_H__ */
