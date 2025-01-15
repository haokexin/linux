// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#ifndef __DPTX_APG_H__
#define __DPTX_APG_H__

int dptx_apg_enable(struct dptx *dptx);
int dptx_apg_disable(struct dptx *dptx);
void dptx_apg_config(struct dptx *dptx, u8 intf_type,
            u8 chan_num, u8 data_width);

#endif