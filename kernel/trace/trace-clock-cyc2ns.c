/*
 * Code for converting cycles to nanosecs
 *
 * Copyright (C) 2010 Windriver Systems, Inc.
 * Author: Wu Zhangjin <wuzhangjin@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/clocksource.h>
#include <linux/trace-clock.h>
#include <linux/module.h>

static u32 precalc_mult __read_mostly;
static u32 precalc_shift __read_mostly;

u64 notrace trace_clock_cyc2ns(u64 cycles)
{
	return ((u64) cycles * precalc_mult) >> precalc_shift;
}
EXPORT_SYMBOL_GPL(trace_clock_cyc2ns);

/*
 * The max time in seconds can be recorded in the virutal register
 * without 64bit-arithmatic overflow.
 *
 * Herein, we set it as one year: 365*24*3600, who will trace the kernel
 * more than one year?
 */

#define MAX_UPDATE_LENGTH	31536000

/*
 * precalc_mult_shift: - Precalculates the mult & shift for trace_clock_cyc2ns()
 */
static int __init precalc_mult_shift(void)
{
	if (trace_clock_frequency() && trace_clock_freq_scale())
		clocks_calc_mult_shift(&precalc_mult, &precalc_shift,
				trace_clock_frequency() *
				trace_clock_freq_scale(), NSEC_PER_SEC,
				MAX_UPDATE_LENGTH);

	pr_info("Init trace_clock_cyc2ns: precalc_mult = %d, precalc_shift = %d\n",
		precalc_mult, precalc_shift);

	return 0;
}

/* Before SMP is up */
early_initcall(precalc_mult_shift);
