// SPDX-License-Identifier: GPL-2.0-only
/*
 * User interface for Resource Allocation in Resource Director Technology(RDT)
 *
 * Copyright (C) 2016 Intel Corporation
 *
 * Author: Fenghua Yu <fenghua.yu@intel.com>
 *
 * More information about RDT be found in the Intel (R) x86 Architecture
 * Software Developer Manual.
 */

#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/cacheinfo.h>
#include <linux/cpu.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/fs_parser.h>
#include <linux/sysfs.h>
#include <linux/kernfs.h>
#include <linux/seq_buf.h>
#include <linux/seq_file.h>
#include <linux/sched/signal.h>
#include <linux/sched/task.h>
#include <linux/slab.h>
#include <linux/task_work.h>
#include <linux/user_namespace.h>

#include <asm/resctrl.h>
#include "internal.h"

DEFINE_STATIC_KEY_FALSE(rdt_enable_key);
DEFINE_STATIC_KEY_FALSE(rdt_mon_enable_key);
DEFINE_STATIC_KEY_FALSE(rdt_alloc_enable_key);

/*
 * This is safe against resctrl_sched_in() called from __switch_to()
 * because __switch_to() is executed with interrupts disabled. A local call
 * from update_closid_rmid() is protected against __switch_to() because
 * preemption is disabled.
 */
void resctrl_arch_sync_cpu_defaults(void *info)
{
	struct resctrl_cpu_sync *r = info;

	if (r) {
		this_cpu_write(pqr_state.default_closid, r->closid);
		this_cpu_write(pqr_state.default_rmid, r->rmid);
	}

	/*
	 * We cannot unconditionally write the MSR because the current
	 * executing task might have its own closid selected. Just reuse
	 * the context switch code.
	 */
	resctrl_sched_in();
}

static void l3_qos_cfg_update(void *arg)
{
	bool *enable = arg;

	wrmsrl(MSR_IA32_L3_QOS_CFG, *enable ? L3_QOS_CDP_ENABLE : 0ULL);
}

static void l2_qos_cfg_update(void *arg)
{
	bool *enable = arg;

	wrmsrl(MSR_IA32_L2_QOS_CFG, *enable ? L2_QOS_CDP_ENABLE : 0ULL);
}

static int set_cache_qos_cfg(int level, bool enable)
{
	void (*update)(void *arg);
	struct rdt_resource *r_l;
	cpumask_var_t cpu_mask;
	struct rdt_domain *d;
	int cpu;

	/* Walking r->domains, ensure it can't race with cpuhp */
	lockdep_assert_cpus_held();

	if (level == RDT_RESOURCE_L3)
		update = l3_qos_cfg_update;
	else if (level == RDT_RESOURCE_L2)
		update = l2_qos_cfg_update;
	else
		return -EINVAL;

	if (!zalloc_cpumask_var(&cpu_mask, GFP_KERNEL))
		return -ENOMEM;

	r_l = &rdt_resources_all[level].r_resctrl;
	list_for_each_entry(d, &r_l->domains, list) {
		if (r_l->cache.arch_has_per_cpu_cfg)
			/* Pick all the CPUs in the domain instance */
			for_each_cpu(cpu, &d->cpu_mask)
				cpumask_set_cpu(cpu, cpu_mask);
		else
			/* Pick one CPU from each domain instance to update MSR */
			cpumask_set_cpu(cpumask_any(&d->cpu_mask), cpu_mask);
	}
	cpu = get_cpu();
	/* Update QOS_CFG MSR on this cpu if it's in cpu_mask. */
	if (cpumask_test_cpu(cpu, cpu_mask))
		update(&enable);
	/* Update QOS_CFG MSR on all other cpus in cpu_mask. */
	smp_call_function_many(cpu_mask, update, &enable, 1);
	put_cpu();

	free_cpumask_var(cpu_mask);

	return 0;
}

/* Restore the qos cfg state when a domain comes online */
void rdt_domain_reconfigure_cdp(struct rdt_resource *r)
{
	struct rdt_hw_resource *hw_res = resctrl_to_arch_res(r);

	if (!r->cdp_capable)
		return;

	if (r->rid == RDT_RESOURCE_L2)
		l2_qos_cfg_update(&hw_res->cdp_enabled);

	if (r->rid == RDT_RESOURCE_L3)
		l3_qos_cfg_update(&hw_res->cdp_enabled);
}

static int cdp_enable(int level)
{
	struct rdt_resource *r_l = &rdt_resources_all[level].r_resctrl;
	int ret;

	if (!r_l->alloc_capable)
		return -EINVAL;

	ret = set_cache_qos_cfg(level, true);
	if (!ret)
		rdt_resources_all[level].cdp_enabled = true;

	return ret;
}

static void cdp_disable(int level)
{
	struct rdt_hw_resource *r_hw = &rdt_resources_all[level];

	if (r_hw->cdp_enabled) {
		set_cache_qos_cfg(level, false);
		r_hw->cdp_enabled = false;
	}
}

int resctrl_arch_set_cdp_enabled(enum resctrl_res_level l, bool enable)
{
	struct rdt_hw_resource *hw_res = &rdt_resources_all[l];

	if (!hw_res->r_resctrl.cdp_capable)
		return -EINVAL;

	if (enable)
		return cdp_enable(l);

	cdp_disable(l);

	return 0;
}

static int reset_all_ctrls(struct rdt_resource *r)
{
	struct rdt_hw_resource *hw_res = resctrl_to_arch_res(r);
	struct rdt_hw_domain *hw_dom;
	struct msr_param msr_param;
	cpumask_var_t cpu_mask;
	struct rdt_domain *d;
	int i, cpu;

	/* Walking r->domains, ensure it can't race with cpuhp */
	lockdep_assert_cpus_held();

	if (!zalloc_cpumask_var(&cpu_mask, GFP_KERNEL))
		return -ENOMEM;

	msr_param.res = r;
	msr_param.low = 0;
	msr_param.high = hw_res->num_closid;

	/*
	 * Disable resource control for this resource by setting all
	 * CBMs in all domains to the maximum mask value. Pick one CPU
	 * from each domain to update the MSRs below.
	 */
	list_for_each_entry(d, &r->domains, list) {
		hw_dom = resctrl_to_arch_dom(d);
		cpumask_set_cpu(cpumask_any(&d->cpu_mask), cpu_mask);

		for (i = 0; i < hw_res->num_closid; i++)
			hw_dom->ctrl_val[i] = r->default_ctrl;
	}
	cpu = get_cpu();
	/* Update CBM on this cpu if it's in cpu_mask. */
	if (cpumask_test_cpu(cpu, cpu_mask))
		rdt_ctrl_update(&msr_param);
	/* Update CBM on all other cpus in cpu_mask. */
	smp_call_function_many(cpu_mask, rdt_ctrl_update, &msr_param, 1);
	put_cpu();

	free_cpumask_var(cpu_mask);

	return 0;
}

void resctrl_arch_reset_resources(void)
{
	struct rdt_resource *r;

	for_each_capable_rdt_resource(r)
		reset_all_ctrls(r);
}

struct mon_config_info {
	u32 evtid;
	u32 mon_config;
};

#define INVALID_CONFIG_INDEX   UINT_MAX
/**
 * mon_event_config_index_get - get the hardware index for the
 *                              configurable event
 * @evtid: event id.
 *
 * Return: 0 for evtid == QOS_L3_MBM_TOTAL_EVENT_ID
 *         1 for evtid == QOS_L3_MBM_LOCAL_EVENT_ID
 *         INVALID_CONFIG_INDEX for invalid evtid
 */
static inline unsigned int mon_event_config_index_get(u32 evtid)
{
	switch (evtid) {
	case QOS_L3_MBM_TOTAL_EVENT_ID:
		return 0;
	case QOS_L3_MBM_LOCAL_EVENT_ID:
		return 1;
	default:
		/* Should never reach here */
		return INVALID_CONFIG_INDEX;
	}
}

static void mon_event_config_read(void *info)
{
	struct mon_config_info *mon_info = info;
	unsigned int index;
	u64 msrval;

	index = mon_event_config_index_get(mon_info->evtid);
	if (index == INVALID_CONFIG_INDEX) {
		pr_warn_once("Invalid event id %d\n", mon_info->evtid);
		return;
	}
	rdmsrl(MSR_IA32_EVT_CFG_BASE + index, msrval);

	/* Report only the valid event configuration bits */
	mon_info->mon_config = msrval & MAX_EVT_CONFIG_BITS;
}

static void mondata_config_read(struct rdt_domain *d, struct mon_config_info *mon_info)
{
	smp_call_function_any(&d->cpu_mask, mon_event_config_read, mon_info, 1);
}

static int mbm_config_show(struct seq_file *s, struct rdt_resource *r, u32 evtid)
{
	struct mon_config_info mon_info = {0};
	struct rdt_domain *dom;
	bool sep = false;

	mutex_lock(&rdtgroup_mutex);

	list_for_each_entry(dom, &r->domains, list) {
		if (sep)
			seq_puts(s, ";");

		memset(&mon_info, 0, sizeof(struct mon_config_info));
		mon_info.evtid = evtid;
		mondata_config_read(dom, &mon_info);

		seq_printf(s, "%d=0x%02x", dom->id, mon_info.mon_config);
		sep = true;
	}
	seq_puts(s, "\n");

	mutex_unlock(&rdtgroup_mutex);

	return 0;
}

static int mbm_total_bytes_config_show(struct kernfs_open_file *of,
				       struct seq_file *seq, void *v)
{
	struct rdt_resource *r = of->kn->parent->priv;

	mbm_config_show(seq, r, QOS_L3_MBM_TOTAL_EVENT_ID);

	return 0;
}

static int mbm_local_bytes_config_show(struct kernfs_open_file *of,
				       struct seq_file *seq, void *v)
{
	struct rdt_resource *r = of->kn->parent->priv;

	mbm_config_show(seq, r, QOS_L3_MBM_LOCAL_EVENT_ID);

	return 0;
}

static void mon_event_config_write(void *info)
{
	struct mon_config_info *mon_info = info;
	unsigned int index;

	index = mon_event_config_index_get(mon_info->evtid);
	if (index == INVALID_CONFIG_INDEX) {
		pr_warn_once("Invalid event id %d\n", mon_info->evtid);
		return;
	}
	wrmsr(MSR_IA32_EVT_CFG_BASE + index, mon_info->mon_config, 0);
}

static int mbm_config_write_domain(struct rdt_resource *r,
				   struct rdt_domain *d, u32 evtid, u32 val)
{
	struct mon_config_info mon_info = {0};
	int ret = 0;

	/* mon_config cannot be more than the supported set of events */
	if (val > MAX_EVT_CONFIG_BITS) {
		rdt_last_cmd_puts("Invalid event configuration\n");
		return -EINVAL;
	}

	/*
	 * Read the current config value first. If both are the same then
	 * no need to write it again.
	 */
	mon_info.evtid = evtid;
	mondata_config_read(d, &mon_info);
	if (mon_info.mon_config == val)
		goto out;

	mon_info.mon_config = val;

	/*
	 * Update MSR_IA32_EVT_CFG_BASE MSR on one of the CPUs in the
	 * domain. The MSRs offset from MSR MSR_IA32_EVT_CFG_BASE
	 * are scoped at the domain level. Writing any of these MSRs
	 * on one CPU is observed by all the CPUs in the domain.
	 */
	smp_call_function_any(&d->cpu_mask, mon_event_config_write,
			      &mon_info, 1);

	/*
	 * When an Event Configuration is changed, the bandwidth counters
	 * for all RMIDs and Events will be cleared by the hardware. The
	 * hardware also sets MSR_IA32_QM_CTR.Unavailable (bit 62) for
	 * every RMID on the next read to any event for every RMID.
	 * Subsequent reads will have MSR_IA32_QM_CTR.Unavailable (bit 62)
	 * cleared while it is tracked by the hardware. Clear the
	 * mbm_local and mbm_total counts for all the RMIDs.
	 */
	resctrl_arch_reset_rmid_all(r, d);

out:
	return ret;
}

static int mon_config_write(struct rdt_resource *r, char *tok, u32 evtid)
{
	char *dom_str = NULL, *id_str;
	unsigned long dom_id, val;
	struct rdt_domain *d;
	int ret = 0;

next:
	if (!tok || tok[0] == '\0')
		return 0;

	/* Start processing the strings for each domain */
	dom_str = strim(strsep(&tok, ";"));
	id_str = strsep(&dom_str, "=");

	if (!id_str || kstrtoul(id_str, 10, &dom_id)) {
		rdt_last_cmd_puts("Missing '=' or non-numeric domain id\n");
		return -EINVAL;
	}

	if (!dom_str || kstrtoul(dom_str, 16, &val)) {
		rdt_last_cmd_puts("Non-numeric event configuration value\n");
		return -EINVAL;
	}

	list_for_each_entry(d, &r->domains, list) {
		if (d->id == dom_id) {
			ret = mbm_config_write_domain(r, d, evtid, val);
			if (ret)
				return -EINVAL;
			goto next;
		}
	}

	return -EINVAL;
}

static ssize_t mbm_total_bytes_config_write(struct kernfs_open_file *of,
					    char *buf, size_t nbytes,
					    loff_t off)
{
	struct rdt_resource *r = of->kn->parent->priv;
	int ret;

	/* Valid input requires a trailing newline */
	if (nbytes == 0 || buf[nbytes - 1] != '\n')
		return -EINVAL;

	mutex_lock(&rdtgroup_mutex);

	rdt_last_cmd_clear();

	buf[nbytes - 1] = '\0';

	ret = mon_config_write(r, buf, QOS_L3_MBM_TOTAL_EVENT_ID);

	mutex_unlock(&rdtgroup_mutex);

	return ret ?: nbytes;
}

static ssize_t mbm_local_bytes_config_write(struct kernfs_open_file *of,
					    char *buf, size_t nbytes,
					    loff_t off)
{
	struct rdt_resource *r = of->kn->parent->priv;
	int ret;

	/* Valid input requires a trailing newline */
	if (nbytes == 0 || buf[nbytes - 1] != '\n')
		return -EINVAL;

	mutex_lock(&rdtgroup_mutex);

	rdt_last_cmd_clear();

	buf[nbytes - 1] = '\0';

	ret = mon_config_write(r, buf, QOS_L3_MBM_LOCAL_EVENT_ID);

	mutex_unlock(&rdtgroup_mutex);

	return ret ?: nbytes;
}


void __init mbm_config_rftype_init(const char *config)
{
	struct rftype *rft;

	rft = rdtgroup_get_rftype_by_name(config);
	if (rft)
		rft->fflags = RF_MON_INFO | RFTYPE_RES_CACHE;
}

static void rdtgroup_kn_get(struct rdtgroup *rdtgrp, struct kernfs_node *kn)
{
	atomic_inc(&rdtgrp->waitcount);
	kernfs_break_active_protection(kn);
}

static void rdtgroup_kn_put(struct rdtgroup *rdtgrp, struct kernfs_node *kn)
{
	if (atomic_dec_and_test(&rdtgrp->waitcount) &&
	    (rdtgrp->flags & RDT_DELETED)) {
		if (rdtgrp->mode == RDT_MODE_PSEUDO_LOCKSETUP ||
		    rdtgrp->mode == RDT_MODE_PSEUDO_LOCKED)
			    rdtgroup_pseudo_lock_remove(rdtgrp);
		kernfs_unbreak_active_protection(kn);
		rdtgroup_remove(rdtgrp);
	} else {
		kernfs_unbreak_active_protection(kn);
	}
}

/**
 * mongrp_reparent() - replace parent CTRL_MON group of a MON group
 * @rdtgrp:		the MON group whose parent should be replaced
 * @new_prdtgrp:	replacement parent CTRL_MON group for @rdtgrp
 * @cpus:		cpumask provided by the caller for use during this call
 *
 * Replaces the parent CTRL_MON group for a MON group, resulting in all member
 * tasks' CLOSID immediately changing to that of the new parent group.
 * Monitoring data for the group is unaffected by this operation.
 */
static void mongrp_reparent(struct rdtgroup *rdtgrp,
			    struct rdtgroup *new_prdtgrp,
			    cpumask_var_t cpus)
{
	struct rdtgroup *prdtgrp = rdtgrp->mon.parent;

	WARN_ON(rdtgrp->type != RDTMON_GROUP);
	WARN_ON(new_prdtgrp->type != RDTCTRL_GROUP);

	/* Nothing to do when simply renaming a MON group. */
	if (prdtgrp == new_prdtgrp)
		return;

	WARN_ON(list_empty(&prdtgrp->mon.crdtgrp_list));
	list_move_tail(&rdtgrp->mon.crdtgrp_list,
		       &new_prdtgrp->mon.crdtgrp_list);

	rdtgrp->mon.parent = new_prdtgrp;
	rdtgrp->closid = new_prdtgrp->closid;

	/* Propagate updated closid to all tasks in this group. */
	rdt_move_group_tasks(rdtgrp, rdtgrp, cpus);

	update_closid_rmid(cpus, NULL);
}

void resctrl_exit(void)
{
	debugfs_remove_recursive(debugfs_resctrl);
	unregister_filesystem(&rdt_fs_type);
	sysfs_remove_mount_point(fs_kobj, "resctrl");
	kernfs_destroy_root(rdt_root);
}
