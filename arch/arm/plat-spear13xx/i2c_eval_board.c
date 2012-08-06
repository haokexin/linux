/*
 * arch/arm/plat-spear/i2c_eval_board.c
 *
 * Copyright (C) 2010 ST Microelectronics
 * Rajeev Kumar<rajeev-dlh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/i2c.h>
#include <linux/kernel.h>

static struct i2c_board_info __initdata i2c_board_info[] = {
	{
		.type = "eeprom",
		.addr = 0x50,
	}, {
		.type = "eeprom",
		.addr = 0x51,
	}, {
		.type = "sta529",
		.addr = 0x1a,
	},
};

void __init i2c_register_default_devices(void)
{
	i2c_register_board_info(0, i2c_board_info,
				ARRAY_SIZE(i2c_board_info));
}
