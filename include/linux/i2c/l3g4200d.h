/*
 * include/linux/i2c/l3g4200d.h
 *
 * ST Microelectronics L3G4200D digital output gyroscope header file
 *
 * Copyright (C) 2010 ST Microelectronics
 *
 * Developed by:
 * Carmine Iascone <carmine.iascone@st.com>
 * Matteo Dameno <matteo.dameno@st.com>
 *
 * Currently maintained by:
 * Amit Virdi <amit.virdi@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __L3G4200D_H__
#define __L3G4200D_H__

#define L3G4200D_DEV_NAME	"l3g4200d_gyr"

#define L3G4200D_RANGE_250DPS	250
#define L3G4200D_RANGE_500DPS	500
#define L3G4200D_RANGE_2000DPS	2000

#define L3G4200D_ENABLED	1
#define L3G4200D_DISABLED	0

#ifdef __KERNEL__

/**
 * struct l3g4200d_gyr_platform_data - L3G4200D gyroscope platform data
 * @poll_interval: The polling interval (in ms)
 * @min_interval: Minimum permitted polling interval
 * @fs_range: Full scale output range selection
 * @axis_map_x, _y, _z: used to map the x, y, z axis to some other plane of
 *	reference
 * @negate_x, _y, _z: flags to negate the output of x, y, z registers
 *
 * l3g4200d_gyr_platform_data specifies the parameters to be used to
 * configure the gyroscope device.
 */
struct l3g4200d_gyr_platform_data {
	int poll_interval;
	int min_interval;

	int fs_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;
};

#endif /* __KERNEL__ */

#endif  /* __L3G4200D_H__ */
