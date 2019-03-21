// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 INTEL Corporation
/*
 * included from arch/arm/mach-axxia/perf_event_platform.c
 *
 * Support for the INTEL Axxia boards based on ARM cores
 */

/*
 * Generic PCX
 */

static void pcx_startup_init(void)
{
}

static u32 pcx_pmu_event_init(u32 ev, struct perf_event *event)
{
	return 0;
}

static u32 pcx_pmu_event_add(u32 ev, struct perf_event *event)
{
	return 0;
}

static u32 pcx_pmu_event_read(u32 ev, struct perf_event *event,
			      int flags)
{
	return 0;
}

static u32 pcx_pmu_event_del(u32 ev, struct perf_event *event,
			     int flags)
{
	return 0;
}
