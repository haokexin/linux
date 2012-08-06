/*
 * arch/arm/plat-spear/include/plat/pl080.h
 *
 * DMAC pl080 definitions for SPEAr platform
 *
 * Copyright (C) 2011 ST Microelectronics
 * Viresh Kumar <viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __PLAT_PL080_H
#define __PLAT_PL080_H

struct amba_device;
struct pl08x_channel_data;

void pl080_set_slaveinfo(struct amba_device *pl080,
		struct pl08x_channel_data *cd, u32 num);

#endif /* __PLAT_PL080_H */
