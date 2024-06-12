// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 INTEL Corporation
/*
 * included from arch/arm/mach-axxia/perf_event_platform.c
 *
 * Support for the INTEL Axxia boards based on ARM cores
 */

#include "perf_event_memc.h"

static void memc_startup_init(void)
{
	u32 config;

	smon_init_ncp(&ddrc0_smon, DDRC0, DDRC_PERF, DDRC_SMON);
	smon_init_ncp(&ddrc1_smon, DDRC1, DDRC_PERF, DDRC_SMON);
	smon_init_mem(&elm0_smon, ELM0, ELM_SMON);
	smon_init_mem(&elm1_smon, ELM1, ELM_SMON);

	/* enable SMC SMON registers */
	ncr_read(NCP_REGION_ID(DDRC0, DDRC_CTRL), CTRL_SMON,
		 REG_SZ, &config);
	config |= SMON_ENABLE;
	ncr_write(NCP_REGION_ID(DDRC0, DDRC_CTRL), CTRL_SMON,
		  REG_SZ, &config);

	ncr_read(NCP_REGION_ID(DDRC1, DDRC_CTRL), CTRL_SMON,
		 REG_SZ, &config);
	config |= SMON_ENABLE;
	ncr_write(NCP_REGION_ID(DDRC1, DDRC_CTRL), CTRL_SMON,
		  REG_SZ, &config);
}

static u32 memc_pmu_event_init(u32 event, struct perf_event *pevent)
{
	return 0;
}

static u32 memc_pmu_event_add(u32 ev, struct perf_event *pevent)
{
	u32 ret;

	if (ev >= DDRC0_OFFSET && ev <= DDRC0_SMON_MAX) {
		ret = smon_allocate(&ddrc0_smon, ev - DDRC0_OFFSET);
		if (ret != 0)
			return ret;

		ret = smon_start(&ddrc0_smon, ev - DDRC0_OFFSET);
		if (ret != 0)
			return ret;
	} else if (ev >= DDRC1_OFFSET && ev <= DDRC1_SMON_MAX) {
		ret = smon_allocate(&ddrc1_smon, ev - DDRC1_OFFSET);
		if (ret != 0)
			return ret;

		ret = smon_start(&ddrc1_smon, ev - DDRC1_OFFSET);
		if (ret != 0)
			return ret;
	} else if (ev >= ELM0_OFFSET && ev <= ELM0_SMON_MAX) {
		ret = smon_allocate(&elm0_smon, ev - ELM0_OFFSET);
		if (ret != 0)
			return ret;

		ret = smon_start(&elm0_smon, ev - ELM0_OFFSET);
		if (ret != 0)
			return ret;
	} else if (ev >= ELM1_OFFSET && ev <= ELM1_SMON_MAX) {
		ret = smon_allocate(&elm1_smon, ev - ELM1_OFFSET);
		if (ret != 0)
			return ret;

		ret = smon_start(&elm1_smon, ev - ELM1_OFFSET);
		if (ret != 0)
			return ret;
	}

	return 0;
}

/*
 * Return counter update.
 */
static u32 memc_pmu_event_read(u32 ev, struct perf_event *pevent,
			       int flags)
{
	u32 count = 0;

	if (ev >= DDRC0_OFFSET && ev <= DDRC0_SMON_MAX)
		count = smon_read(&ddrc0_smon, ev - DDRC0_OFFSET);
	else if (ev >= DDRC1_OFFSET && ev <= DDRC1_SMON_MAX)
		count = smon_read(&ddrc1_smon, ev - DDRC1_OFFSET);
	else if (ev >= ELM0_OFFSET && ev <= ELM0_SMON_MAX)
		count = smon_read(&elm0_smon, ev - ELM0_OFFSET);
	else if (ev >= ELM1_OFFSET && ev <= ELM1_SMON_MAX)
		count = smon_read(&elm1_smon, ev - ELM1_OFFSET);

	if (count == -ENOEVENT)
		count = 0;

	return count;
}

/*
 * Remove event and return counter update.
 */
static u32 memc_pmu_event_del(u32 ev, struct perf_event *pevent,
			      int flags)
{
	u32 count = 0;

	if (ev >= DDRC0_OFFSET && ev <= DDRC0_SMON_MAX) {
		count = smon_read(&ddrc0_smon, ev - DDRC0_OFFSET);

		smon_deallocate(&ddrc0_smon, ev - DDRC0_OFFSET);
	} else if (ev >= DDRC1_OFFSET && ev <= DDRC1_SMON_MAX) {
		count = smon_read(&ddrc1_smon, ev - DDRC1_OFFSET);

		smon_deallocate(&ddrc1_smon, ev - DDRC1_OFFSET);
	} else if (ev >= ELM0_OFFSET && ev <= ELM0_SMON_MAX) {
		count = smon_read(&elm0_smon, ev - ELM0_OFFSET);

		smon_deallocate(&elm0_smon, ev - ELM0_OFFSET);
	} else if (ev >= ELM1_OFFSET && ev <= ELM1_SMON_MAX) {
		count = smon_read(&elm1_smon, ev - ELM1_OFFSET);

		smon_deallocate(&elm1_smon, ev - ELM1_OFFSET);
	}

	if (count == -ENOEVENT)
		count = 0;

	return count;
}
