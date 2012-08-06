/*
 * arch/arm/plat-spear/include/mach/i2c.h
 *
 * Copyright (C) 2012 ST Microelectronics
 * Viresh Kumar <viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __PLAT_I2C_H
#define __PLAT_I2C_H

#include <linux/i2c.h>

struct i2c_dev_info {
	struct i2c_board_info *board;
	int busnum;
};
#endif /* __PLAT_I2C_H */
