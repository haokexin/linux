/*
 * arch/arm/plat-spear/include/plat/cpufreq.h
 *
 * CPU Frequency Scaling definitions for SPEAr platform
 *
 * Copyright (C) 2010-2012 ST Microelectronics
 * Deepak Sikri <deepak.sikri@st.com>
 *
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef _MACH_SPEAR_CPUFREQ_H
#define _MACH_SPEAR_CPUFREQ_H

#include <linux/cpufreq.h>

/* Entry in the table must be in ascending order */
struct spear_cpufreq_pdata {
	unsigned int *cpu_freq_table;
	unsigned int tbl_len;
	unsigned int transition_latency;
};
#endif
