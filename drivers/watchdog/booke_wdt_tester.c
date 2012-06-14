/*
 * Watchdog timer tester for PowerPC Book-E systems
 *
 * Copyright 2010 Freescale Semiconductor Inc.
 *
 * Author: Jiang Yutang <b14898@freescale.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>

static void __loop_occupy_cpu(void *data)
{
	printk(KERN_INFO "__loop_occupy_cpu id=%08x\n", smp_processor_id());
	while (1);
}

static int __init wdt_tester_init(void)
{
	on_each_cpu(__loop_occupy_cpu, NULL, 0);
	return 0;
}

module_init(wdt_tester_init);

MODULE_AUTHOR("Jiang Yutang");
MODULE_LICENSE("GPL");
