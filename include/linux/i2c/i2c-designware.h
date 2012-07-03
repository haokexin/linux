/*
 * Synopsys DesignWare I2C adapter driver's platform data
 *
 * Copyright (C) 2012 ST Microelectronics.
 * Author: Vincenzo Frascino <vincenzo.frascino@st.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef I2C_DESIGNWARE_H
#define I2C_DESIGNWARE_H

#include <linux/platform_device.h>

/* I2C Designware Platform Data */
struct i2c_dw_pdata {
	/*
	 * The scope of this routine is to define i2c bus recovery procedure
	 * as specified in the i2c protocol Rev. 03 section 3.16 titled
	 * "Bus clear".
	 * Its implementation is platform dependant.
	 */
	void (*i2c_recover_bus)(void *);
};

#endif /* I2C_DESIGNWARE_H */
