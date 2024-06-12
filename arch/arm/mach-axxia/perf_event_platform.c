// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 INTEL Corporation
/*
 * Support for the INTEL Axxia boards based on ARM cores
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <linux/bitmap.h>
#include <linux/cpu_pm.h>
#include <linux/export.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <asm/cputype.h>
#include <asm/irq_regs.h>

#include <linux/perf_event.h>
#include <linux/kthread.h>
#include <linux/sched.h>

#include <linux/cpu.h>
#include <linux/reboot.h>
#include <linux/syscore_ops.h>

#include <linux/proc_fs.h>

#include <linux/io.h>
#include <linux/axxia-ncr.h>
#include <asm/cacheflush.h>

#include "perf_event_platform.h"

#include "smon.h"

/*
 * Include code for individual block support
 */

#include "perf_event_pcx.c"
#include "perf_event_vp.c"
#include "perf_event_memc.c"

static struct axxia_platform_pmu *axm_pmu;

/*
 * General platform perf code, muxed out to individual blocks
 */

int platform_pmu_event_idx(struct perf_event *event)
{
	return 0;
}

int platform_pmu_event_init(struct perf_event *event)
{
	u64 ev = event->attr.config;

	if (event->attr.type != event->pmu->type)
		return -ENOENT;

	if (ev < AXM_55XX_PLATFORM_BASE || ev > AXM_55XX_PLATFORM_MAX)
		return -ENOENT;

	event->hw.config = ev - AXM_55XX_PLATFORM_BASE;

	event->hw.idx = -1;
	event->hw.config_base = 1;

/*
 * if (event->group_leader != event) {
 * printk("This is not the group leader!\n");
 * printk("event->group_leader 0x%x\n", (unsigned int)event->group_leader);
 * }
 */

	if (event->attr.exclude_user)
		return -EOPNOTSUPP;
	if (event->attr.exclude_kernel)
		return -EOPNOTSUPP;
	if (event->attr.exclude_idle)
		return -EOPNOTSUPP;

	event->hw.last_period = event->hw.sample_period;
	local64_set(&event->hw.period_left, event->hw.last_period);
/*
 * event->destroy = hw_perf_event_destroy;
 */
	local64_set(&event->count, 0);

	if (ev >= AXM_55XX_VP_BASE && ev <= AXM_55XX_VP_MAX)
		vp_pmu_event_init(ev - AXM_55XX_VP_BASE, event);
	else if (ev >= AXM_55XX_PCX_BASE && ev <= AXM_55XX_PCX_MAX)
		pcx_pmu_event_init(ev - AXM_55XX_PCX_BASE, event);
	else if (ev >= AXM_55XX_MEMC_BASE && ev <= AXM_55XX_MEMC_MAX)
		memc_pmu_event_init(ev - AXM_55XX_MEMC_BASE, event);
	else
		pr_info("Platform perf, undefined event, %llu\n", ev);

	return 0;
}

static int platform_pmu_event_add(struct perf_event *event, int flags)
{
	u64 ev = event->attr.config;

	if (ev >= AXM_55XX_VP_BASE && ev <= AXM_55XX_VP_MAX)
		vp_pmu_event_add(ev - AXM_55XX_VP_BASE, event);
	else if (ev >= AXM_55XX_PCX_BASE && ev <= AXM_55XX_PCX_MAX)
		pcx_pmu_event_add(ev - AXM_55XX_PCX_BASE, event);
	else if (ev >= AXM_55XX_MEMC_BASE && ev <= AXM_55XX_MEMC_MAX)
		memc_pmu_event_add(ev - AXM_55XX_MEMC_BASE, event);

	return 0;
}

static void platform_pmu_event_del(struct perf_event *event, int flags)
{
	u64 ev = event->attr.config;
	u32 n;

	if (ev >= AXM_55XX_VP_BASE && ev <= AXM_55XX_VP_MAX) {
		n = vp_pmu_event_del(ev - AXM_55XX_VP_BASE, event, flags);
		local64_add(n, &event->count);
	} else if (ev >= AXM_55XX_PCX_BASE && ev <= AXM_55XX_PCX_MAX) {
		n = pcx_pmu_event_del(ev - AXM_55XX_PCX_BASE, event, flags);
		local64_add(n, &event->count);
	} else if (ev >= AXM_55XX_MEMC_BASE && ev <= AXM_55XX_MEMC_MAX) {
		n = memc_pmu_event_del(ev - AXM_55XX_MEMC_BASE, event, flags);
		local64_add(n, &event->count);
	} else {
		local64_set(&event->count, 0);
	}
}

static void platform_pmu_event_start(struct perf_event *event, int flags)
{
}

static void platform_pmu_event_stop(struct perf_event *event, int flags)
{
}

static void platform_pmu_event_read(struct perf_event *event)
{
	u64 ev = event->attr.config;
	u32 n;

	if (ev >= AXM_55XX_VP_BASE && ev <= AXM_55XX_VP_MAX) {
		n = vp_pmu_event_read(ev - AXM_55XX_VP_BASE, event, 0);
		local64_add(n, &event->count);
	} else if (ev >= AXM_55XX_PCX_BASE && ev <= AXM_55XX_PCX_MAX) {
		n = pcx_pmu_event_read(ev - AXM_55XX_PCX_BASE, event, 0);
		local64_add(n, &event->count);
	} else if (ev >= AXM_55XX_MEMC_BASE && ev <= AXM_55XX_MEMC_MAX) {
		n = memc_pmu_event_read(ev - AXM_55XX_MEMC_BASE, event, 0);
		local64_add(n, &event->count);
	}
}

/*
 * Device
 */

static void axmperf_device_release(struct device *dev)
{
	pr_warn("AXM55xxPlatformPerf release device\n");
}

static struct platform_device axmperf_device = {
	.name = "AXM55xxPlatformPerf",
	.id = 0,
	.dev = {
		.release = axmperf_device_release,
		},
};

/*
 * Driver
 */

#define PLATFORM_PMU_NAME_LEN 32

struct axxia_platform_pmu {
	struct pmu pmu;
	char name[PLATFORM_PMU_NAME_LEN];
};

static int axmperf_probe(struct platform_device *dev)
{
	int ret;

	axm_pmu = kzalloc(sizeof(*axm_pmu), GFP_KERNEL);
	if (!axm_pmu)
		return -ENOMEM;

	axm_pmu->pmu = (struct pmu) {
		.task_ctx_nr = perf_invalid_context,
		.attr_groups = 0,
		.event_init = platform_pmu_event_init,
		.add = platform_pmu_event_add,
		.del = platform_pmu_event_del,
		.start = platform_pmu_event_start,
		.stop = platform_pmu_event_stop,
		.read = platform_pmu_event_read,
		.event_idx = platform_pmu_event_idx,
	};

	sprintf(axm_pmu->name, "INTEL Axxia AXM55xx Platform");

	ret = perf_pmu_register(&axm_pmu->pmu, axm_pmu->name, PERF_TYPE_RAW);

	if (ret == 0)
		pr_info("axxia platform perf enabled\n");
	else
		pr_info("axxia platform perf failed\n");

	vp_startup_init();
	pcx_startup_init();
	memc_startup_init();

	return ret;
}

static const struct of_device_id axxia_platformperf_match[] = {
	{ .compatible = "axxia,axm-platformperf", },
	{},
};

static struct platform_driver axmperf_driver = {
	.driver = {
		.name = "AXM55xxPlatformPerf",
		.of_match_table = axxia_platformperf_match,
		.owner = THIS_MODULE,
		},
	.probe = axmperf_probe,
};

static int __init axmperf_init(void)
{
	platform_driver_register(&axmperf_driver);

	return 0;
}

static void __exit axmperf_exit(void)
{
	pr_warn("AXM55xx platform perf exit!\n");
	perf_pmu_unregister(&axm_pmu->pmu);
	platform_driver_unregister(&axmperf_driver);
	platform_device_unregister(&axmperf_device);
}

module_init(axmperf_init);
module_exit(axmperf_exit);
MODULE_LICENSE("GPL");
