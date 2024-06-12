/* SPDX-License-Identifier: GPL-2.0 */

/*
 * This is based on arch/arm/include/asm/hardware/timer-sp.h
 *
 * See arch/arm/mach-axxia/timers.c for details.
 *
 * Copyright (C) 2021 INTEL Corporation
 */

//#include "timer-sp.h"

#define AXXIA_TIMER_1_BASE 0x00
#define AXXIA_TIMER_2_BASE 0x20
#define AXXIA_TIMER_3_BASE 0x40
#define AXXIA_TIMER_4_BASE 0x60
#define AXXIA_TIMER_5_BASE 0x80
#define AXXIA_TIMER_6_BASE 0xa0
#define AXXIA_TIMER_7_BASE 0xc0
#define AXXIA_TIMER_8_BASE 0xe0

void axxia_timer_clocksource_init(void __iomem *a, const char *b);
void axxia_timer_clockevents_init(void __iomem *a,
				  unsigned int b, const char *c);
