/*
 * arch/arm/plat-spear/include/plat/hdlc.h
 *
 * HDLC definitions for SPEAr platform
 *
 * Copyright (C) 2010 ST Microelectronics
 * Frank Shi<frank.shi@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __PLAT_HDLC_H
#define __PLAT_HDLC_H

#include <linux/platform_device.h>

#define SPEAR1310_REVA_TDM_HDLC	0
#define SPEAR310_TDM_HDLC	1
#define SPEAR1310_TDM_HDLC	2

struct tdm_hdlc_platform_data {
	int			ip_type;
	int			nr_channel;
	int			nr_timeslot;
	int			tx_falling_edge;
	int			rx_rising_edge;
	int			ts0_delay;
};

struct rs485_hdlc_platform_data {
	int			tx_falling_edge;
	int			rx_rising_edge;
	int			cts_enable;
	int			cts_delay;
};

int e1phy_init(u32 base, int shift);

static inline void tdm_hdlc_set_plat_data(struct platform_device *pdev, int tsn)
{
	struct tdm_hdlc_platform_data *pdata = dev_get_platdata(&pdev->dev);

	pdata->nr_timeslot = tsn;
}

#endif /* __PLAT_HDLC_H */
