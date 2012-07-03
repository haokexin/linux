/*
 * CAN bus driver platform header for Bosch C_CAN controller
 *
 * Copyright (C) 2010 ST Microelectronics
 * Bhupesh Sharma <bhupesh.sharma@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __CAN_PLATFORM_C_CAN_H__
#define __CAN_PLATFORM_C_CAN_H__

#include <linux/types.h>

/**
 * struct c_can_devtype_data - Depending on the underlying platform,
 *                             the message object configuration
 *                             for c_can controller can change
 * @rx_first: first RX object out of possible 32 msg objs
 * @rx_split: defines the split point for the lower and upper Rx msg
 *            object buckets.
 * @rx_last: last RX object out of possible 32 msg objs
 * @tx_num: number of objs kept aside for TX purposes
 */
struct c_can_devtype_data {
	unsigned int rx_first;
	unsigned int rx_split;
	unsigned int rx_last;
	unsigned int tx_num;
};

/**
 * struct c_can_platform_data - C_CAN Platform Data
 * @is_quirk_required: Depending on the SoC being used
 *                     determine if a SW fix (/quirk) is required
 *                     for the c_can controller.
 * @devtype_data: c_can device specific data
 */
struct c_can_platform_data {
	bool is_quirk_required;
	struct c_can_devtype_data devtype_data;
};

#endif /* __CAN_PLATFORM_C_CAN_H__ */
