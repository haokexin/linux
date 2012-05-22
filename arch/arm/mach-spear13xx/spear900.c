/*
 * arch/arm/mach-spear13xx/spear900.c
 *
 * SPEAr900 machine source file
 *
 * Copyright (C) 2010 ST Microelectronics
 * Viresh Kumar <viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <mach/generic.h>

/* pmx driver structure */
static struct pmx_driver pmx_driver;

/* Add spear900 specific devices here */

void __init spear900_init(struct pmx_mode *pmx_mode, struct pmx_dev **pmx_devs,
		u8 pmx_dev_count)
{
	int ret;

	/* call spear13xx family common init function */
	spear13xx_init();

	/* pmx initialization */
	pmx_driver.mode = pmx_mode;
	pmx_driver.devs = pmx_devs;
	pmx_driver.devs_count = pmx_dev_count;

	ret = pmx_register(&pmx_driver);
	if (ret)
		pr_err("padmux: registeration failed. err no: %d\n", ret);
}
