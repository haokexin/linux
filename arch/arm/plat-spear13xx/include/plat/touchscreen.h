/*
 * arch/arm/plat-spear/include/plat/touchscreen.h
 *
 * Header file for touchscreen present on SPEAr platform
 *
 * Copyright (C) 2010 ST Microelectronics
 * Vipul Kumar Samar <vipulkumar.samar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#ifndef __PLAT_TOUCHSCREEN_H
#define __PLAT_TOUCHSCREEN_H

struct spear_touchscreen {
	u32 adc_channel_x;
	u32 adc_channel_y;
	u32 gpio_pin;
};

#endif /* _TOUCHSCREEN_SPEAR_H */
