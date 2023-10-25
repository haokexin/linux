/* SPDX-License-Identifier: GPL-2.0 */

/*
 * included from arch/arm/mach-axxia/perf_event_memc.c
 *
 * Support for the INTEL Axxia boards based on ARM cores
 *
 * Copyright (C) 2021 INTEL Corporation
 */

#ifndef __ASM__ARCH_AXXIA_PERF_EVENT_MEMC_H
#define __ASM__ARCH_AXXIA_PERF_EVENT_MEMC_H

#define DDRC0_OFFSET 0x00
#define DDRC0_SMON_MAX (DDRC0_OFFSET + 22)
#define DDRC1_OFFSET 0x100
#define DDRC1_SMON_MAX (DDRC1_OFFSET + 22)

#define ELM0_OFFSET 0x200
#define ELM0_SMON_MAX (ELM0_OFFSET + 15)
#define ELM1_OFFSET 0x300
#define ELM1_SMON_MAX (ELM1_OFFSET + 15)

/* Node */
#define DDRC0 0x0f
#define DDRC1 0x22
/* Target */
#define DDRC_CTRL 0x00
#define DDRC_PERF 0x02
/* Address */
#define CTRL_SMON 0x1fc

#ifdef AXM55XX_R1
#define DDRC_SMON 0x40
#endif
#ifdef AXM55XX_R2
#define DDRC_SMON 0xA0
#endif

/* Settings */
#define SMON_ENABLE 0x20000000

/* Base Address */
#define ELM0 0x2010060000
#define ELM1 0x2010070000
/* SMON Offset */
#define ELM_SMON (0x300 / 4)

struct smon_s ddrc0_smon;
struct smon_s ddrc1_smon;
struct smon_s elm0_smon;
struct smon_s elm1_smon;

#endif
