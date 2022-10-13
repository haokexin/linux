// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 INTEL Corporation
/*
 * included from arch/arm/mach-axxia/perf_event_platform.c
 *
 * Support for the INTEL Axxia boards based on ARM cores
 */

/*
 * Generic VP
 */

static void vp_startup_init(void)
{
}

static u32 vp_pmu_event_init(u32 event, struct perf_event *pevent)
{
	return 0;
}

static u32 vp_pmu_event_add(u32 event, struct perf_event *pevent)
{
	return 0;
}

static u32 vp_pmu_event_read(u32 event, struct perf_event *pevent,
			     int flags)
{
	return 0;
}

static u32 vp_pmu_event_del(u32 event, struct perf_event *pevent,
			    int flags)
{
	return 0;
}
