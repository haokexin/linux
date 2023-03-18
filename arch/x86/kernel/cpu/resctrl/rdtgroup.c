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
	resctrl_sched_in(current);
}

/*
 * Update the PGR_ASSOC MSR on all cpus in @cpu_mask,
 *
 * Per task closids/rmids must have been set up before calling this function.
 */
static void
update_closid_rmid(const struct cpumask *cpu_mask, struct rdtgroup *r)
{
	int cpu = get_cpu();

	if (cpumask_test_cpu(cpu, cpu_mask))
		update_cpu_closid_rmid(r);
	smp_call_function_many(cpu_mask, update_cpu_closid_rmid, r, 1);
	put_cpu();
}

static int cpus_mon_write(struct rdtgroup *rdtgrp, cpumask_var_t newmask,
			  cpumask_var_t tmpmask)
{
	struct rdtgroup *prgrp = rdtgrp->mon.parent, *crgrp;
	struct list_head *head;

	/* Check whether cpus belong to parent ctrl group */
	cpumask_andnot(tmpmask, newmask, &prgrp->cpu_mask);
	if (cpumask_weight(tmpmask)) {
		rdt_last_cmd_puts("Can only add CPUs to mongroup that belong to parent\n");
		return -EINVAL;
	}

	/* Check whether cpus are dropped from this group */
	cpumask_andnot(tmpmask, &rdtgrp->cpu_mask, newmask);
	if (cpumask_weight(tmpmask)) {
		/* Give any dropped cpus to parent rdtgroup */
		cpumask_or(&prgrp->cpu_mask, &prgrp->cpu_mask, tmpmask);
		update_closid_rmid(tmpmask, prgrp);
	}

	/*
	 * If we added cpus, remove them from previous group that owned them
	 * and update per-cpu rmid
	 */
	cpumask_andnot(tmpmask, newmask, &rdtgrp->cpu_mask);
	if (cpumask_weight(tmpmask)) {
		head = &prgrp->mon.crdtgrp_list;
		list_for_each_entry(crgrp, head, mon.crdtgrp_list) {
			if (crgrp == rdtgrp)
				continue;
			cpumask_andnot(&crgrp->cpu_mask, &crgrp->cpu_mask,
				       tmpmask);
		}
		update_closid_rmid(tmpmask, rdtgrp);
	}

	/* Done pushing/pulling - update this group with new mask */
	cpumask_copy(&rdtgrp->cpu_mask, newmask);

	return 0;
}

static void cpumask_rdtgrp_clear(struct rdtgroup *r, struct cpumask *m)
{
	struct rdtgroup *crgrp;

	cpumask_andnot(&r->cpu_mask, &r->cpu_mask, m);
	/* update the child mon group masks as well*/
	list_for_each_entry(crgrp, &r->mon.crdtgrp_list, mon.crdtgrp_list)
		cpumask_and(&crgrp->cpu_mask, &r->cpu_mask, &crgrp->cpu_mask);
}

static int cpus_ctrl_write(struct rdtgroup *rdtgrp, cpumask_var_t newmask,
			   cpumask_var_t tmpmask, cpumask_var_t tmpmask1)
{
	struct rdtgroup *r, *crgrp;
	struct list_head *head;

	/* Check whether cpus are dropped from this group */
	cpumask_andnot(tmpmask, &rdtgrp->cpu_mask, newmask);
	if (cpumask_weight(tmpmask)) {
		/* Can't drop from default group */
		if (rdtgrp == &rdtgroup_default) {
			rdt_last_cmd_puts("Can't drop CPUs from default group\n");
			return -EINVAL;
		}

		/* Give any dropped cpus to rdtgroup_default */
		cpumask_or(&rdtgroup_default.cpu_mask,
			   &rdtgroup_default.cpu_mask, tmpmask);
		update_closid_rmid(tmpmask, &rdtgroup_default);
	}

	/*
	 * If we added cpus, remove them from previous group and
	 * the prev group's child groups that owned them
	 * and update per-cpu closid/rmid.
	 */
	cpumask_andnot(tmpmask, newmask, &rdtgrp->cpu_mask);
	if (cpumask_weight(tmpmask)) {
		list_for_each_entry(r, &rdt_all_groups, rdtgroup_list) {
			if (r == rdtgrp)
				continue;
			cpumask_and(tmpmask1, &r->cpu_mask, tmpmask);
			if (cpumask_weight(tmpmask1))
				cpumask_rdtgrp_clear(r, tmpmask1);
		}
		update_closid_rmid(tmpmask, rdtgrp);
	}

	/* Done pushing/pulling - update this group with new mask */
	cpumask_copy(&rdtgrp->cpu_mask, newmask);

	/*
	 * Clear child mon group masks since there is a new parent mask
	 * now and update the rmid for the cpus the child lost.
	 */
	head = &rdtgrp->mon.crdtgrp_list;
	list_for_each_entry(crgrp, head, mon.crdtgrp_list) {
		cpumask_and(tmpmask, &rdtgrp->cpu_mask, &crgrp->cpu_mask);
		update_closid_rmid(tmpmask, rdtgrp);
		cpumask_clear(&crgrp->cpu_mask);
	}

	return 0;
}

static ssize_t rdtgroup_cpus_write(struct kernfs_open_file *of,
				   char *buf, size_t nbytes, loff_t off)
{
	cpumask_var_t tmpmask, newmask, tmpmask1;
	struct rdtgroup *rdtgrp;
	int ret;

	if (!buf)
		return -EINVAL;

	if (!zalloc_cpumask_var(&tmpmask, GFP_KERNEL))
		return -ENOMEM;
	if (!zalloc_cpumask_var(&newmask, GFP_KERNEL)) {
		free_cpumask_var(tmpmask);
		return -ENOMEM;
	}
	if (!zalloc_cpumask_var(&tmpmask1, GFP_KERNEL)) {
		free_cpumask_var(tmpmask);
		free_cpumask_var(newmask);
		return -ENOMEM;
	}

	rdtgrp = rdtgroup_kn_lock_live(of->kn);
	if (!rdtgrp) {
		ret = -ENOENT;
		goto unlock;
	}

	if (rdtgrp->mode == RDT_MODE_PSEUDO_LOCKED ||
	    rdtgrp->mode == RDT_MODE_PSEUDO_LOCKSETUP) {
		ret = -EINVAL;
		rdt_last_cmd_puts("Pseudo-locking in progress\n");
		goto unlock;
	}

	if (is_cpu_list(of))
		ret = cpulist_parse(buf, newmask);
	else
		ret = cpumask_parse(buf, newmask);

	if (ret) {
		rdt_last_cmd_puts("Bad CPU list/mask\n");
		goto unlock;
	}

	/* check that user didn't specify any offline cpus */
	cpumask_andnot(tmpmask, newmask, cpu_online_mask);
	if (cpumask_weight(tmpmask)) {
		ret = -EINVAL;
		rdt_last_cmd_puts("Can only assign online CPUs\n");
		goto unlock;
	}

	if (rdtgrp->type == RDTCTRL_GROUP)
		ret = cpus_ctrl_write(rdtgrp, newmask, tmpmask, tmpmask1);
	else if (rdtgrp->type == RDTMON_GROUP)
		ret = cpus_mon_write(rdtgrp, newmask, tmpmask);
	else
		ret = -EINVAL;

unlock:
	rdtgroup_kn_unlock(of->kn);
	free_cpumask_var(tmpmask);
	free_cpumask_var(newmask);
	free_cpumask_var(tmpmask1);

	return ret ?: nbytes;
}

/**
 * rdtgroup_remove - the helper to remove resource group safely
 * @rdtgrp: resource group to remove
 *
 * On resource group creation via a mkdir, an extra kernfs_node reference is
 * taken to ensure that the rdtgroup structure remains accessible for the
 * rdtgroup_kn_unlock() calls where it is removed.
 *
 * Drop the extra reference here, then free the rdtgroup structure.
 *
 * Return: void
 */
static void rdtgroup_remove(struct rdtgroup *rdtgrp)
{
	kernfs_put(rdtgrp->kn);
	kfree(rdtgrp);
}

static void _update_task_closid_rmid(void *task)
{
	/*
	 * If the task is still current on this CPU, update PQR_ASSOC MSR.
	 * Otherwise, the MSR is updated when the task is scheduled in.
	 */
	if (task == current)
		resctrl_sched_in(task);
}

static void update_task_closid_rmid(struct task_struct *t)
{
	if (IS_ENABLED(CONFIG_SMP) && task_curr(t))
		smp_call_function_single(task_cpu(t), _update_task_closid_rmid, t, 1);
	else
		_update_task_closid_rmid(t);
}

static int __rdtgroup_move_task(struct task_struct *tsk,
				struct rdtgroup *rdtgrp)
{
	/* If the task is already in rdtgrp, no need to move the task. */
	if ((rdtgrp->type == RDTCTRL_GROUP && tsk->closid == rdtgrp->closid &&
	     tsk->rmid == rdtgrp->mon.rmid) ||
	    (rdtgrp->type == RDTMON_GROUP && tsk->rmid == rdtgrp->mon.rmid &&
	     tsk->closid == rdtgrp->mon.parent->closid))
		return 0;

	/*
	 * Set the task's closid/rmid before the PQR_ASSOC MSR can be
	 * updated by them.
	 *
	 * For ctrl_mon groups, move both closid and rmid.
	 * For monitor groups, can move the tasks only from
	 * their parent CTRL group.
	 */

	if (rdtgrp->type == RDTCTRL_GROUP) {
		WRITE_ONCE(tsk->closid, rdtgrp->closid);
		WRITE_ONCE(tsk->rmid, rdtgrp->mon.rmid);
	} else if (rdtgrp->type == RDTMON_GROUP) {
		if (rdtgrp->mon.parent->closid == tsk->closid) {
			WRITE_ONCE(tsk->rmid, rdtgrp->mon.rmid);
		} else {
			rdt_last_cmd_puts("Can't move task to different control group\n");
			return -EINVAL;
		}
	}

	/*
	 * Ensure the task's closid and rmid are written before determining if
	 * the task is current that will decide if it will be interrupted.
	 * This pairs with the full barrier between the rq->curr update and
	 * resctrl_sched_in() during context switch.
	 */
	smp_mb();

	/*
	 * By now, the task's closid and rmid are set. If the task is current
	 * on a CPU, the PQR_ASSOC MSR needs to be updated to make the resource
	 * group go into effect. If the task is not current, the MSR will be
	 * updated when the task is scheduled in.
	 */
	update_task_closid_rmid(tsk);

	return 0;
}

static bool is_closid_match(struct task_struct *t, struct rdtgroup *r)
{
	return (rdt_alloc_capable &&
	       (r->type == RDTCTRL_GROUP) && (t->closid == r->closid));
}

static bool is_rmid_match(struct task_struct *t, struct rdtgroup *r)
{
	return (rdt_mon_capable &&
	       (r->type == RDTMON_GROUP) && (t->rmid == r->mon.rmid));
}

/**
 * rdtgroup_tasks_assigned - Test if tasks have been assigned to resource group
 * @r: Resource group
 *
 * Return: 1 if tasks have been assigned to @r, 0 otherwise
 */
int rdtgroup_tasks_assigned(struct rdtgroup *r)
{
	struct task_struct *p, *t;
	int ret = 0;

	lockdep_assert_held(&rdtgroup_mutex);

	rcu_read_lock();
	for_each_process_thread(p, t) {
		if (is_closid_match(t, r) || is_rmid_match(t, r)) {
			ret = 1;
			break;
		}
	}
	rcu_read_unlock();

	return ret;
}

static int rdtgroup_task_write_permission(struct task_struct *task,
					  struct kernfs_open_file *of)
{
	const struct cred *tcred = get_task_cred(task);
	const struct cred *cred = current_cred();
	int ret = 0;

	/*
	 * Even if we're attaching all tasks in the thread group, we only
	 * need to check permissions on one of them.
	 */
	if (!uid_eq(cred->euid, GLOBAL_ROOT_UID) &&
	    !uid_eq(cred->euid, tcred->uid) &&
	    !uid_eq(cred->euid, tcred->suid)) {
		rdt_last_cmd_printf("No permission to move task %d\n", task->pid);
		ret = -EPERM;
	}

	put_cred(tcred);
	return ret;
}

static int rdtgroup_move_task(pid_t pid, struct rdtgroup *rdtgrp,
			      struct kernfs_open_file *of)
{
	struct task_struct *tsk;
	int ret;

	rcu_read_lock();
	if (pid) {
		tsk = find_task_by_vpid(pid);
		if (!tsk) {
			rcu_read_unlock();
			rdt_last_cmd_printf("No task %d\n", pid);
			return -ESRCH;
		}
	} else {
		tsk = current;
	}

	get_task_struct(tsk);
	rcu_read_unlock();

	ret = rdtgroup_task_write_permission(tsk, of);
	if (!ret)
		ret = __rdtgroup_move_task(tsk, rdtgrp);

	put_task_struct(tsk);
	return ret;
}

static ssize_t rdtgroup_tasks_write(struct kernfs_open_file *of,
				    char *buf, size_t nbytes, loff_t off)
{
	struct rdtgroup *rdtgrp;
	int ret = 0;
	pid_t pid;

	if (kstrtoint(strstrip(buf), 0, &pid) || pid < 0)
		return -EINVAL;
	rdtgrp = rdtgroup_kn_lock_live(of->kn);
	if (!rdtgrp) {
		rdtgroup_kn_unlock(of->kn);
		return -ENOENT;
	}
	rdt_last_cmd_clear();

	if (rdtgrp->mode == RDT_MODE_PSEUDO_LOCKED ||
	    rdtgrp->mode == RDT_MODE_PSEUDO_LOCKSETUP) {
		ret = -EINVAL;
		rdt_last_cmd_puts("Pseudo-locking in progress\n");
		goto unlock;
	}

	ret = rdtgroup_move_task(pid, rdtgrp, of);

unlock:
	rdtgroup_kn_unlock(of->kn);

	return ret ?: nbytes;
}

static void show_rdt_tasks(struct rdtgroup *r, struct seq_file *s)
{
	struct task_struct *p, *t;

	rcu_read_lock();
	for_each_process_thread(p, t) {
		if (is_closid_match(t, r) || is_rmid_match(t, r))
			seq_printf(s, "%d\n", t->pid);
	}
	rcu_read_unlock();
}

static int rdtgroup_tasks_show(struct kernfs_open_file *of,
			       struct seq_file *s, void *v)
{
	struct rdtgroup *rdtgrp;
	int ret = 0;

	rdtgrp = rdtgroup_kn_lock_live(of->kn);
	if (rdtgrp)
		show_rdt_tasks(rdtgrp, s);
	else
		ret = -ENOENT;
	rdtgroup_kn_unlock(of->kn);

	return ret;
}

#ifdef CONFIG_PROC_CPU_RESCTRL

/*
 * A task can only be part of one resctrl control group and of one monitor
 * group which is associated to that control group.
 *
 * 1)   res:
 *      mon:
 *
 *    resctrl is not available.
 *
 * 2)   res:/
 *      mon:
 *
 *    Task is part of the root resctrl control group, and it is not associated
 *    to any monitor group.
 *
 * 3)  res:/
 *     mon:mon0
 *
 *    Task is part of the root resctrl control group and monitor group mon0.
 *
 * 4)  res:group0
 *     mon:
 *
 *    Task is part of resctrl control group group0, and it is not associated
 *    to any monitor group.
 *
 * 5) res:group0
 *    mon:mon1
 *
 *    Task is part of resctrl control group group0 and monitor group mon1.
 */
int proc_resctrl_show(struct seq_file *s, struct pid_namespace *ns,
		      struct pid *pid, struct task_struct *tsk)
{
	struct rdtgroup *rdtg;
	int ret = 0;

	mutex_lock(&rdtgroup_mutex);

	/* Return empty if resctrl has not been mounted. */
	if (!static_branch_unlikely(&rdt_enable_key)) {
		seq_puts(s, "res:\nmon:\n");
		goto unlock;
	}

	list_for_each_entry(rdtg, &rdt_all_groups, rdtgroup_list) {
		struct rdtgroup *crg;

		/*
		 * Task information is only relevant for shareable
		 * and exclusive groups.
		 */
		if (rdtg->mode != RDT_MODE_SHAREABLE &&
		    rdtg->mode != RDT_MODE_EXCLUSIVE)
			continue;

		if (rdtg->closid != tsk->closid)
			continue;

		seq_printf(s, "res:%s%s\n", (rdtg == &rdtgroup_default) ? "/" : "",
			   rdtg->kn->name);
		seq_puts(s, "mon:");
		list_for_each_entry(crg, &rdtg->mon.crdtgrp_list,
				    mon.crdtgrp_list) {
			if (tsk->rmid != crg->mon.rmid)
				continue;
			seq_printf(s, "%s", crg->kn->name);
			break;
		}
		seq_putc(s, '\n');
		goto unlock;
	}
	/*
	 * The above search should succeed. Otherwise return
	 * with an error.
	 */
	ret = -ENOENT;
unlock:
	mutex_unlock(&rdtgroup_mutex);

	return ret;
}
#endif

static int rdt_last_cmd_status_show(struct kernfs_open_file *of,
				    struct seq_file *seq, void *v)
{
	int len;

	mutex_lock(&rdtgroup_mutex);
	len = seq_buf_used(&last_cmd_status);
	if (len)
		seq_printf(seq, "%.*s", len, last_cmd_status_buf);
	else
		seq_puts(seq, "ok\n");
	mutex_unlock(&rdtgroup_mutex);
	return 0;
}

static int rdt_num_closids_show(struct kernfs_open_file *of,
				struct seq_file *seq, void *v)
{
	struct resctrl_schema *s = of->kn->parent->priv;

	seq_printf(seq, "%u\n", s->num_closid);
	return 0;
}

static int rdt_default_ctrl_show(struct kernfs_open_file *of,
			     struct seq_file *seq, void *v)
{
	struct resctrl_schema *s = of->kn->parent->priv;
	struct rdt_resource *r = s->res;

	seq_printf(seq, "%x\n", r->default_ctrl);
	return 0;
}

static int rdt_min_cbm_bits_show(struct kernfs_open_file *of,
			     struct seq_file *seq, void *v)
{
	struct resctrl_schema *s = of->kn->parent->priv;
	struct rdt_resource *r = s->res;

	seq_printf(seq, "%u\n", r->cache.min_cbm_bits);
	return 0;
}

static int rdt_shareable_bits_show(struct kernfs_open_file *of,
				   struct seq_file *seq, void *v)
{
	struct resctrl_schema *s = of->kn->parent->priv;
	struct rdt_resource *r = s->res;

	seq_printf(seq, "%x\n", r->cache.shareable_bits);
	return 0;
}

/**
 * rdt_bit_usage_show - Display current usage of resources
 *
 * A domain is a shared resource that can now be allocated differently. Here
 * we display the current regions of the domain as an annotated bitmask.
 * For each domain of this resource its allocation bitmask
 * is annotated as below to indicate the current usage of the corresponding bit:
 *   0 - currently unused
 *   X - currently available for sharing and used by software and hardware
 *   H - currently used by hardware only but available for software use
 *   S - currently used and shareable by software only
 *   E - currently used exclusively by one resource group
 *   P - currently pseudo-locked by one resource group
 */
static int rdt_bit_usage_show(struct kernfs_open_file *of,
			      struct seq_file *seq, void *v)
{
	struct resctrl_schema *s = of->kn->parent->priv;
	/*
	 * Use unsigned long even though only 32 bits are used to ensure
	 * test_bit() is used safely.
	 */
	unsigned long sw_shareable = 0, hw_shareable = 0;
	unsigned long exclusive = 0, pseudo_locked = 0;
	struct rdt_resource *r = s->res;
	struct rdt_domain *dom;
	int i, hwb, swb, excl, psl;
	enum rdtgrp_mode mode;
	bool sep = false;
	u32 ctrl_val;

	mutex_lock(&rdtgroup_mutex);
	hw_shareable = r->cache.shareable_bits;
	list_for_each_entry(dom, &r->domains, list) {
		if (sep)
			seq_putc(seq, ';');
		sw_shareable = 0;
		exclusive = 0;
		seq_printf(seq, "%d=", dom->id);
		for (i = 0; i < closids_supported(); i++) {
			if (!closid_allocated(i))
				continue;
			ctrl_val = resctrl_arch_get_config(r, dom, i,
							   s->conf_type);
			mode = rdtgroup_mode_by_closid(i);
			switch (mode) {
			case RDT_MODE_SHAREABLE:
				sw_shareable |= ctrl_val;
				break;
			case RDT_MODE_EXCLUSIVE:
				exclusive |= ctrl_val;
				break;
			case RDT_MODE_PSEUDO_LOCKSETUP:
			/*
			 * RDT_MODE_PSEUDO_LOCKSETUP is possible
			 * here but not included since the CBM
			 * associated with this CLOSID in this mode
			 * is not initialized and no task or cpu can be
			 * assigned this CLOSID.
			 */
				break;
			case RDT_MODE_PSEUDO_LOCKED:
			case RDT_NUM_MODES:
				WARN(1,
				     "invalid mode for closid %d\n", i);
				break;
			}
		}
		for (i = r->cache.cbm_len - 1; i >= 0; i--) {
			pseudo_locked = dom->plr ? dom->plr->cbm : 0;
			hwb = test_bit(i, &hw_shareable);
			swb = test_bit(i, &sw_shareable);
			excl = test_bit(i, &exclusive);
			psl = test_bit(i, &pseudo_locked);
			if (hwb && swb)
				seq_putc(seq, 'X');
			else if (hwb && !swb)
				seq_putc(seq, 'H');
			else if (!hwb && swb)
				seq_putc(seq, 'S');
			else if (excl)
				seq_putc(seq, 'E');
			else if (psl)
				seq_putc(seq, 'P');
			else /* Unused bits remain */
				seq_putc(seq, '0');
		}
		sep = true;
	}
	seq_putc(seq, '\n');
	mutex_unlock(&rdtgroup_mutex);
	return 0;
}

static int rdt_min_bw_show(struct kernfs_open_file *of,
			     struct seq_file *seq, void *v)
{
	struct resctrl_schema *s = of->kn->parent->priv;
	struct rdt_resource *r = s->res;

	seq_printf(seq, "%u\n", r->membw.min_bw);
	return 0;
}

static int rdt_num_rmids_show(struct kernfs_open_file *of,
			      struct seq_file *seq, void *v)
{
	struct rdt_resource *r = of->kn->parent->priv;

	seq_printf(seq, "%d\n", r->num_rmid);

	return 0;
}

static int rdt_mon_features_show(struct kernfs_open_file *of,
				 struct seq_file *seq, void *v)
{
	struct rdt_resource *r = of->kn->parent->priv;
	struct mon_evt *mevt;

	list_for_each_entry(mevt, &r->evt_list, list)
		seq_printf(seq, "%s\n", mevt->name);

	return 0;
}

static int rdt_bw_gran_show(struct kernfs_open_file *of,
			     struct seq_file *seq, void *v)
{
	struct resctrl_schema *s = of->kn->parent->priv;
	struct rdt_resource *r = s->res;

	seq_printf(seq, "%u\n", r->membw.bw_gran);
	return 0;
}

static int rdt_delay_linear_show(struct kernfs_open_file *of,
			     struct seq_file *seq, void *v)
{
	struct resctrl_schema *s = of->kn->parent->priv;
	struct rdt_resource *r = s->res;

	seq_printf(seq, "%u\n", r->membw.delay_linear);
	return 0;
}

static int max_threshold_occ_show(struct kernfs_open_file *of,
				  struct seq_file *seq, void *v)
{
	struct rdt_resource *r = of->kn->parent->priv;
	struct rdt_hw_resource *hw_res = resctrl_to_arch_res(r);

	seq_printf(seq, "%u\n", resctrl_cqm_threshold * hw_res->mon_scale);

	return 0;
}

static int rdt_thread_throttle_mode_show(struct kernfs_open_file *of,
					 struct seq_file *seq, void *v)
{
	struct resctrl_schema *s = of->kn->parent->priv;
	struct rdt_resource *r = s->res;

	if (r->membw.throttle_mode == THREAD_THROTTLE_PER_THREAD)
		seq_puts(seq, "per-thread\n");
	else
		seq_puts(seq, "max\n");

	return 0;
}

static ssize_t max_threshold_occ_write(struct kernfs_open_file *of,
				       char *buf, size_t nbytes, loff_t off)
{
	struct rdt_hw_resource *hw_res;
	unsigned int bytes;
	int ret;

	ret = kstrtouint(buf, 0, &bytes);
	if (ret)
		return ret;

	if (bytes > (boot_cpu_data.x86_cache_size * 1024))
		return -EINVAL;

	hw_res = resctrl_to_arch_res(of->kn->parent->priv);
	resctrl_cqm_threshold = bytes / hw_res->mon_scale;

	return nbytes;
}

/*
 * rdtgroup_mode_show - Display mode of this resource group
 */
static int rdtgroup_mode_show(struct kernfs_open_file *of,
			      struct seq_file *s, void *v)
{
	struct rdtgroup *rdtgrp;

	rdtgrp = rdtgroup_kn_lock_live(of->kn);
	if (!rdtgrp) {
		rdtgroup_kn_unlock(of->kn);
		return -ENOENT;
	}

	seq_printf(s, "%s\n", rdtgroup_mode_str(rdtgrp->mode));

	rdtgroup_kn_unlock(of->kn);
	return 0;
}

static enum resctrl_conf_type resctrl_peer_type(enum resctrl_conf_type my_type)
{
	switch (my_type) {
	case CDP_CODE:
		return CDP_DATA;
	case CDP_DATA:
		return CDP_CODE;
	default:
	case CDP_NONE:
		return CDP_NONE;
	}
}

/**
 * __rdtgroup_cbm_overlaps - Does CBM for intended closid overlap with other
 * @r: Resource to which domain instance @d belongs.
 * @d: The domain instance for which @closid is being tested.
 * @cbm: Capacity bitmask being tested.
 * @closid: Intended closid for @cbm.
 * @exclusive: Only check if overlaps with exclusive resource groups
 *
 * Checks if provided @cbm intended to be used for @closid on domain
 * @d overlaps with any other closids or other hardware usage associated
 * with this domain. If @exclusive is true then only overlaps with
 * resource groups in exclusive mode will be considered. If @exclusive
 * is false then overlaps with any resource group or hardware entities
 * will be considered.
 *
 * @cbm is unsigned long, even if only 32 bits are used, to make the
 * bitmap functions work correctly.
 *
 * Return: false if CBM does not overlap, true if it does.
 */
static bool __rdtgroup_cbm_overlaps(struct rdt_resource *r, struct rdt_domain *d,
				    unsigned long cbm, int closid,
				    enum resctrl_conf_type type, bool exclusive)
{
	enum rdtgrp_mode mode;
	unsigned long ctrl_b;
	int i;

	/* Check for any overlap with regions used by hardware directly */
	if (!exclusive) {
		ctrl_b = r->cache.shareable_bits;
		if (bitmap_intersects(&cbm, &ctrl_b, r->cache.cbm_len))
			return true;
	}

	/* Check for overlap with other resource groups */
	for (i = 0; i < closids_supported(); i++) {
		ctrl_b = resctrl_arch_get_config(r, d, i, type);
		mode = rdtgroup_mode_by_closid(i);
		if (closid_allocated(i) && i != closid &&
		    mode != RDT_MODE_PSEUDO_LOCKSETUP) {
			if (bitmap_intersects(&cbm, &ctrl_b, r->cache.cbm_len)) {
				if (exclusive) {
					if (mode == RDT_MODE_EXCLUSIVE)
						return true;
					continue;
				}
				return true;
			}
		}
	}

	return false;
}

/**
 * rdtgroup_cbm_overlaps - Does CBM overlap with other use of hardware
 * @s: Schema for the resource to which domain instance @d belongs.
 * @d: The domain instance for which @closid is being tested.
 * @cbm: Capacity bitmask being tested.
 * @closid: Intended closid for @cbm.
 * @exclusive: Only check if overlaps with exclusive resource groups
 *
 * Resources that can be allocated using a CBM can use the CBM to control
 * the overlap of these allocations. rdtgroup_cmb_overlaps() is the test
 * for overlap. Overlap test is not limited to the specific resource for
 * which the CBM is intended though - when dealing with CDP resources that
 * share the underlying hardware the overlap check should be performed on
 * the CDP resource sharing the hardware also.
 *
 * Refer to description of __rdtgroup_cbm_overlaps() for the details of the
 * overlap test.
 *
 * Return: true if CBM overlap detected, false if there is no overlap
 */
bool rdtgroup_cbm_overlaps(struct resctrl_schema *s, struct rdt_domain *d,
			   unsigned long cbm, int closid, bool exclusive)
{
	enum resctrl_conf_type peer_type = resctrl_peer_type(s->conf_type);
	struct rdt_resource *r = s->res;

	if (__rdtgroup_cbm_overlaps(r, d, cbm, closid, s->conf_type,
				    exclusive))
		return true;

	if (!resctrl_arch_get_cdp_enabled(r->rid))
		return false;
	return  __rdtgroup_cbm_overlaps(r, d, cbm, closid, peer_type, exclusive);
}

/**
 * rdtgroup_mode_test_exclusive - Test if this resource group can be exclusive
 *
 * An exclusive resource group implies that there should be no sharing of
 * its allocated resources. At the time this group is considered to be
 * exclusive this test can determine if its current schemata supports this
 * setting by testing for overlap with all other resource groups.
 *
 * Return: true if resource group can be exclusive, false if there is overlap
 * with allocations of other resource groups and thus this resource group
 * cannot be exclusive.
 */
static bool rdtgroup_mode_test_exclusive(struct rdtgroup *rdtgrp)
{
	int closid = rdtgrp->closid;
	struct resctrl_schema *s;
	struct rdt_resource *r;
	bool has_cache = false;
	struct rdt_domain *d;
	u32 ctrl;

	list_for_each_entry(s, &resctrl_schema_all, list) {
		r = s->res;
		if (r->rid == RDT_RESOURCE_MBA)
			continue;
		has_cache = true;
		list_for_each_entry(d, &r->domains, list) {
			ctrl = resctrl_arch_get_config(r, d, closid,
						       s->conf_type);
			if (rdtgroup_cbm_overlaps(s, d, ctrl, closid, false)) {
				rdt_last_cmd_puts("Schemata overlaps\n");
				return false;
			}
		}
	}

	if (!has_cache) {
		rdt_last_cmd_puts("Cannot be exclusive without CAT/CDP\n");
		return false;
	}

	return true;
}

/**
 * rdtgroup_mode_write - Modify the resource group's mode
 *
 */
static ssize_t rdtgroup_mode_write(struct kernfs_open_file *of,
				   char *buf, size_t nbytes, loff_t off)
{
	struct rdtgroup *rdtgrp;
	enum rdtgrp_mode mode;
	int ret = 0;

	/* Valid input requires a trailing newline */
	if (nbytes == 0 || buf[nbytes - 1] != '\n')
		return -EINVAL;
	buf[nbytes - 1] = '\0';

	rdtgrp = rdtgroup_kn_lock_live(of->kn);
	if (!rdtgrp) {
		rdtgroup_kn_unlock(of->kn);
		return -ENOENT;
	}

	rdt_last_cmd_clear();

	mode = rdtgrp->mode;

	if ((!strcmp(buf, "shareable") && mode == RDT_MODE_SHAREABLE) ||
	    (!strcmp(buf, "exclusive") && mode == RDT_MODE_EXCLUSIVE) ||
	    (!strcmp(buf, "pseudo-locksetup") &&
	     mode == RDT_MODE_PSEUDO_LOCKSETUP) ||
	    (!strcmp(buf, "pseudo-locked") && mode == RDT_MODE_PSEUDO_LOCKED))
		goto out;

	if (mode == RDT_MODE_PSEUDO_LOCKED) {
		rdt_last_cmd_puts("Cannot change pseudo-locked group\n");
		ret = -EINVAL;
		goto out;
	}

	if (!strcmp(buf, "shareable")) {
		if (rdtgrp->mode == RDT_MODE_PSEUDO_LOCKSETUP) {
			ret = rdtgroup_locksetup_exit(rdtgrp);
			if (ret)
				goto out;
		}
		rdtgrp->mode = RDT_MODE_SHAREABLE;
	} else if (!strcmp(buf, "exclusive")) {
		if (!rdtgroup_mode_test_exclusive(rdtgrp)) {
			ret = -EINVAL;
			goto out;
		}
		if (rdtgrp->mode == RDT_MODE_PSEUDO_LOCKSETUP) {
			ret = rdtgroup_locksetup_exit(rdtgrp);
			if (ret)
				goto out;
		}
		rdtgrp->mode = RDT_MODE_EXCLUSIVE;
	} else if (!strcmp(buf, "pseudo-locksetup")) {
		ret = rdtgroup_locksetup_enter(rdtgrp);
		if (ret)
			goto out;
		rdtgrp->mode = RDT_MODE_PSEUDO_LOCKSETUP;
	} else {
		rdt_last_cmd_puts("Unknown or unsupported mode\n");
		ret = -EINVAL;
	}

out:
	rdtgroup_kn_unlock(of->kn);
	return ret ?: nbytes;
}

/**
 * rdtgroup_cbm_to_size - Translate CBM to size in bytes
 * @r: RDT resource to which @d belongs.
 * @d: RDT domain instance.
 * @cbm: bitmask for which the size should be computed.
 *
 * The bitmask provided associated with the RDT domain instance @d will be
 * translated into how many bytes it represents. The size in bytes is
 * computed by first dividing the total cache size by the CBM length to
 * determine how many bytes each bit in the bitmask represents. The result
 * is multiplied with the number of bits set in the bitmask.
 *
 * @cbm is unsigned long, even if only 32 bits are used to make the
 * bitmap functions work correctly.
 */
unsigned int rdtgroup_cbm_to_size(struct rdt_resource *r,
				  struct rdt_domain *d, unsigned long cbm)
{
	struct cpu_cacheinfo *ci;
	unsigned int size = 0;
	int num_b, i;

	num_b = bitmap_weight(&cbm, r->cache.cbm_len);
	ci = get_cpu_cacheinfo(cpumask_any(&d->cpu_mask));
	for (i = 0; i < ci->num_leaves; i++) {
		if (ci->info_list[i].level == r->cache_level) {
			size = ci->info_list[i].size / r->cache.cbm_len * num_b;
			break;
		}
	}

	return size;
}

/**
 * rdtgroup_size_show - Display size in bytes of allocated regions
 *
 * The "size" file mirrors the layout of the "schemata" file, printing the
 * size in bytes of each region instead of the capacity bitmask.
 *
 */
static int rdtgroup_size_show(struct kernfs_open_file *of,
			      struct seq_file *s, void *v)
{
	struct resctrl_schema *schema;
	struct rdtgroup *rdtgrp;
	struct rdt_resource *r;
	struct rdt_domain *d;
	unsigned int size;
	int ret = 0;
	bool sep;
	u32 ctrl;

	rdtgrp = rdtgroup_kn_lock_live(of->kn);
	if (!rdtgrp) {
		rdtgroup_kn_unlock(of->kn);
		return -ENOENT;
	}

	if (rdtgrp->mode == RDT_MODE_PSEUDO_LOCKED) {
		if (!rdtgrp->plr->d) {
			rdt_last_cmd_clear();
			rdt_last_cmd_puts("Cache domain offline\n");
			ret = -ENODEV;
		} else {
			seq_printf(s, "%*s:", max_name_width,
				   rdtgrp->plr->s->name);
			size = rdtgroup_cbm_to_size(rdtgrp->plr->s->res,
						    rdtgrp->plr->d,
						    rdtgrp->plr->cbm);
			seq_printf(s, "%d=%u\n", rdtgrp->plr->d->id, size);
		}
		goto out;
	}

	list_for_each_entry(schema, &resctrl_schema_all, list) {
		r = schema->res;
		sep = false;
		seq_printf(s, "%*s:", max_name_width, schema->name);
		list_for_each_entry(d, &r->domains, list) {
			if (sep)
				seq_putc(s, ';');
			if (rdtgrp->mode == RDT_MODE_PSEUDO_LOCKSETUP) {
				size = 0;
			} else {
				ctrl = resctrl_arch_get_config(r, d,
							       rdtgrp->closid,
							       schema->conf_type);
				if (r->rid == RDT_RESOURCE_MBA)
					size = ctrl;
				else
					size = rdtgroup_cbm_to_size(r, d, ctrl);
			}
			seq_printf(s, "%d=%u", d->id, size);
			sep = true;
		}
		seq_putc(s, '\n');
	}

out:
	rdtgroup_kn_unlock(of->kn);

	return ret;
}

/* rdtgroup information files for one cache resource. */
static struct rftype res_common_files[] = {
	{
		.name		= "last_cmd_status",
		.mode		= 0444,
		.kf_ops		= &rdtgroup_kf_single_ops,
		.seq_show	= rdt_last_cmd_status_show,
		.fflags		= RF_TOP_INFO,
	},
	{
		.name		= "num_closids",
		.mode		= 0444,
		.kf_ops		= &rdtgroup_kf_single_ops,
		.seq_show	= rdt_num_closids_show,
		.fflags		= RF_CTRL_INFO,
	},
	{
		.name		= "mon_features",
		.mode		= 0444,
		.kf_ops		= &rdtgroup_kf_single_ops,
		.seq_show	= rdt_mon_features_show,
		.fflags		= RF_MON_INFO,
	},
	{
		.name		= "num_rmids",
		.mode		= 0444,
		.kf_ops		= &rdtgroup_kf_single_ops,
		.seq_show	= rdt_num_rmids_show,
		.fflags		= RF_MON_INFO,
	},
	{
		.name		= "cbm_mask",
		.mode		= 0444,
		.kf_ops		= &rdtgroup_kf_single_ops,
		.seq_show	= rdt_default_ctrl_show,
		.fflags		= RF_CTRL_INFO | RFTYPE_RES_CACHE,
	},
	{
		.name		= "min_cbm_bits",
		.mode		= 0444,
		.kf_ops		= &rdtgroup_kf_single_ops,
		.seq_show	= rdt_min_cbm_bits_show,
		.fflags		= RF_CTRL_INFO | RFTYPE_RES_CACHE,
	},
	{
		.name		= "shareable_bits",
		.mode		= 0444,
		.kf_ops		= &rdtgroup_kf_single_ops,
		.seq_show	= rdt_shareable_bits_show,
		.fflags		= RF_CTRL_INFO | RFTYPE_RES_CACHE,
	},
	{
		.name		= "bit_usage",
		.mode		= 0444,
		.kf_ops		= &rdtgroup_kf_single_ops,
		.seq_show	= rdt_bit_usage_show,
		.fflags		= RF_CTRL_INFO | RFTYPE_RES_CACHE,
	},
	{
		.name		= "min_bandwidth",
		.mode		= 0444,
		.kf_ops		= &rdtgroup_kf_single_ops,
		.seq_show	= rdt_min_bw_show,
		.fflags		= RF_CTRL_INFO | RFTYPE_RES_MB,
	},
	{
		.name		= "bandwidth_gran",
		.mode		= 0444,
		.kf_ops		= &rdtgroup_kf_single_ops,
		.seq_show	= rdt_bw_gran_show,
		.fflags		= RF_CTRL_INFO | RFTYPE_RES_MB,
	},
	{
		.name		= "delay_linear",
		.mode		= 0444,
		.kf_ops		= &rdtgroup_kf_single_ops,
		.seq_show	= rdt_delay_linear_show,
		.fflags		= RF_CTRL_INFO | RFTYPE_RES_MB,
	},
	/*
	 * Platform specific which (if any) capabilities are provided by
	 * thread_throttle_mode. Defer "fflags" initialization to platform
	 * discovery.
	 */
	{
		.name		= "thread_throttle_mode",
		.mode		= 0444,
		.kf_ops		= &rdtgroup_kf_single_ops,
		.seq_show	= rdt_thread_throttle_mode_show,
	},
	{
		.name		= "max_threshold_occupancy",
		.mode		= 0644,
		.kf_ops		= &rdtgroup_kf_single_ops,
		.write		= max_threshold_occ_write,
		.seq_show	= max_threshold_occ_show,
		.fflags		= RF_MON_INFO | RFTYPE_RES_CACHE,
	},
	{
		.name		= "cpus",
		.mode		= 0644,
		.kf_ops		= &rdtgroup_kf_single_ops,
		.write		= rdtgroup_cpus_write,
		.seq_show	= rdtgroup_cpus_show,
		.fflags		= RFTYPE_BASE,
	},
	{
		.name		= "cpus_list",
		.mode		= 0644,
		.kf_ops		= &rdtgroup_kf_single_ops,
		.write		= rdtgroup_cpus_write,
		.seq_show	= rdtgroup_cpus_show,
		.flags		= RFTYPE_FLAGS_CPUS_LIST,
		.fflags		= RFTYPE_BASE,
	},
	{
		.name		= "tasks",
		.mode		= 0644,
		.kf_ops		= &rdtgroup_kf_single_ops,
		.write		= rdtgroup_tasks_write,
		.seq_show	= rdtgroup_tasks_show,
		.fflags		= RFTYPE_BASE,
	},
	{
		.name		= "schemata",
		.mode		= 0644,
		.kf_ops		= &rdtgroup_kf_single_ops,
		.write		= rdtgroup_schemata_write,
		.seq_show	= rdtgroup_schemata_show,
		.fflags		= RF_CTRL_BASE,
	},
	{
		.name		= "mode",
		.mode		= 0644,
		.kf_ops		= &rdtgroup_kf_single_ops,
		.write		= rdtgroup_mode_write,
		.seq_show	= rdtgroup_mode_show,
		.fflags		= RF_CTRL_BASE,
	},
	{
		.name		= "size",
		.mode		= 0444,
		.kf_ops		= &rdtgroup_kf_single_ops,
		.seq_show	= rdtgroup_size_show,
		.fflags		= RF_CTRL_BASE,
	},

};

static int rdtgroup_add_files(struct kernfs_node *kn, unsigned long fflags)
{
	struct rftype *rfts, *rft;
	int ret, len;

	rfts = res_common_files;
	len = ARRAY_SIZE(res_common_files);

	lockdep_assert_held(&rdtgroup_mutex);

	for (rft = rfts; rft < rfts + len; rft++) {
		if (rft->fflags && ((fflags & rft->fflags) == rft->fflags)) {
			ret = rdtgroup_add_file(kn, rft);
			if (ret)
				goto error;
		}
	}

	return 0;
error:
	pr_warn("Failed to add %s, err=%d\n", rft->name, ret);
	while (--rft >= rfts) {
		if ((fflags & rft->fflags) == rft->fflags)
			kernfs_remove_by_name(kn, rft->name);
	}
	return ret;
}

static struct rftype *rdtgroup_get_rftype_by_name(const char *name)
{
	struct rftype *rfts, *rft;
	int len;

	rfts = res_common_files;
	len = ARRAY_SIZE(res_common_files);

	for (rft = rfts; rft < rfts + len; rft++) {
		if (!strcmp(rft->name, name))
			return rft;
	}

	return NULL;
}

void __init thread_throttle_mode_init(void)
{
	struct rftype *rft;

	rft = rdtgroup_get_rftype_by_name("thread_throttle_mode");
	if (!rft)
		return;

	rft->fflags = RF_CTRL_INFO | RFTYPE_RES_MB;
}

/**
 * rdtgroup_kn_mode_restrict - Restrict user access to named resctrl file
 * @r: The resource group with which the file is associated.
 * @name: Name of the file
 *
 * The permissions of named resctrl file, directory, or link are modified
 * to not allow read, write, or execute by any user.
 *
 * WARNING: This function is intended to communicate to the user that the
 * resctrl file has been locked down - that it is not relevant to the
 * particular state the system finds itself in. It should not be relied
 * on to protect from user access because after the file's permissions
 * are restricted the user can still change the permissions using chmod
 * from the command line.
 *
 * Return: 0 on success, <0 on failure.
 */
int rdtgroup_kn_mode_restrict(struct rdtgroup *r, const char *name)
{
	struct iattr iattr = {.ia_valid = ATTR_MODE,};
	struct kernfs_node *kn;
	int ret = 0;

	kn = kernfs_find_and_get_ns(r->kn, name, NULL);
	if (!kn)
		return -ENOENT;

	switch (kernfs_type(kn)) {
	case KERNFS_DIR:
		iattr.ia_mode = S_IFDIR;
		break;
	case KERNFS_FILE:
		iattr.ia_mode = S_IFREG;
		break;
	case KERNFS_LINK:
		iattr.ia_mode = S_IFLNK;
		break;
	}

	ret = kernfs_setattr(kn, &iattr);
	kernfs_put(kn);
	return ret;
}

/**
 * rdtgroup_kn_mode_restore - Restore user access to named resctrl file
 * @r: The resource group with which the file is associated.
 * @name: Name of the file
 * @mask: Mask of permissions that should be restored
 *
 * Restore the permissions of the named file. If @name is a directory the
 * permissions of its parent will be used.
 *
 * Return: 0 on success, <0 on failure.
 */
int rdtgroup_kn_mode_restore(struct rdtgroup *r, const char *name,
			     umode_t mask)
{
	struct iattr iattr = {.ia_valid = ATTR_MODE,};
	struct kernfs_node *kn, *parent;
	struct rftype *rfts, *rft;
	int ret, len;

	rfts = res_common_files;
	len = ARRAY_SIZE(res_common_files);

	for (rft = rfts; rft < rfts + len; rft++) {
		if (!strcmp(rft->name, name))
			iattr.ia_mode = rft->mode & mask;
	}

	kn = kernfs_find_and_get_ns(r->kn, name, NULL);
	if (!kn)
		return -ENOENT;

	switch (kernfs_type(kn)) {
	case KERNFS_DIR:
		parent = kernfs_get_parent(kn);
		if (parent) {
			iattr.ia_mode |= parent->mode;
			kernfs_put(parent);
		}
		iattr.ia_mode |= S_IFDIR;
		break;
	case KERNFS_FILE:
		iattr.ia_mode |= S_IFREG;
		break;
	case KERNFS_LINK:
		iattr.ia_mode |= S_IFLNK;
		break;
	}

	ret = kernfs_setattr(kn, &iattr);
	kernfs_put(kn);
	return ret;
}

static int rdtgroup_mkdir_info_resdir(void *priv, char *name,
				      unsigned long fflags)
{
	struct kernfs_node *kn_subdir;
	int ret;

	kn_subdir = kernfs_create_dir(kn_info, name,
				      kn_info->mode, priv);
	if (IS_ERR(kn_subdir))
		return PTR_ERR(kn_subdir);

	ret = rdtgroup_kn_set_ugid(kn_subdir);
	if (ret)
		return ret;

	ret = rdtgroup_add_files(kn_subdir, fflags);
	if (!ret)
		kernfs_activate(kn_subdir);

	return ret;
}

static int rdtgroup_create_info_dir(struct kernfs_node *parent_kn)
{
	struct resctrl_schema *s;
	struct rdt_resource *r;
	unsigned long fflags;
	char name[32];
	int ret;

	/* create the directory */
	kn_info = kernfs_create_dir(parent_kn, "info", parent_kn->mode, NULL);
	if (IS_ERR(kn_info))
		return PTR_ERR(kn_info);

	ret = rdtgroup_add_files(kn_info, RF_TOP_INFO);
	if (ret)
		goto out_destroy;

	/* loop over enabled controls, these are all alloc_enabled */
	list_for_each_entry(s, &resctrl_schema_all, list) {
		r = s->res;
		fflags =  r->fflags | RF_CTRL_INFO;
		ret = rdtgroup_mkdir_info_resdir(s, s->name, fflags);
		if (ret)
			goto out_destroy;
	}

	for_each_mon_enabled_rdt_resource(r) {
		fflags =  r->fflags | RF_MON_INFO;
		sprintf(name, "%s_MON", r->name);
		ret = rdtgroup_mkdir_info_resdir(r, name, fflags);
		if (ret)
			goto out_destroy;
	}

	ret = rdtgroup_kn_set_ugid(kn_info);
	if (ret)
		goto out_destroy;

	kernfs_activate(kn_info);

	return 0;

out_destroy:
	kernfs_remove(kn_info);
	return ret;
}

static int
mongroup_create_dir(struct kernfs_node *parent_kn, struct rdtgroup *prgrp,
		    char *name, struct kernfs_node **dest_kn)
{
	struct kernfs_node *kn;
	int ret;

	/* create the directory */
	kn = kernfs_create_dir(parent_kn, name, parent_kn->mode, prgrp);
	if (IS_ERR(kn))
		return PTR_ERR(kn);

	if (dest_kn)
		*dest_kn = kn;

	ret = rdtgroup_kn_set_ugid(kn);
	if (ret)
		goto out_destroy;

	kernfs_activate(kn);

	return 0;

out_destroy:
	kernfs_remove(kn);
	return ret;
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

/*
 * Move tasks from one to the other group. If @from is NULL, then all tasks
 * in the systems are moved unconditionally (used for teardown).
 *
 * If @mask is not NULL the cpus on which moved tasks are running are set
 * in that mask so the update smp function call is restricted to affected
 * cpus.
 */
static void rdt_move_group_tasks(struct rdtgroup *from, struct rdtgroup *to,
				 struct cpumask *mask)
{
	struct task_struct *p, *t;

	read_lock(&tasklist_lock);
	for_each_process_thread(p, t) {
		if (!from || is_closid_match(t, from) ||
		    is_rmid_match(t, from)) {
			WRITE_ONCE(t->closid, to->closid);
			WRITE_ONCE(t->rmid, to->mon.rmid);

			/*
			 * Order the closid/rmid stores above before the loads
			 * in task_curr(). This pairs with the full barrier
			 * between the rq->curr update and resctrl_sched_in()
			 * during context switch.
			 */
			smp_mb();

			/*
			 * If the task is on a CPU, set the CPU in the mask.
			 * The detection is inaccurate as tasks might move or
			 * schedule before the smp function call takes place.
			 * In such a case the function call is pointless, but
			 * there is no other side effect.
			 */
			if (IS_ENABLED(CONFIG_SMP) && mask && task_curr(t))
				cpumask_set_cpu(task_cpu(t), mask);
		}
	}
	read_unlock(&tasklist_lock);
}

static void free_all_child_rdtgrp(struct rdtgroup *rdtgrp)
{
	struct rdtgroup *sentry, *stmp;
	struct list_head *head;

	head = &rdtgrp->mon.crdtgrp_list;
	list_for_each_entry_safe(sentry, stmp, head, mon.crdtgrp_list) {
		free_rmid(sentry->mon.rmid);
		list_del(&sentry->mon.crdtgrp_list);

		if (atomic_read(&sentry->waitcount) != 0)
			sentry->flags = RDT_DELETED;
		else
			rdtgroup_remove(sentry);
	}
}

/*
 * Forcibly remove all of subdirectories under root.
 */
static void rmdir_all_sub(void)
{
	struct rdtgroup *rdtgrp, *tmp;

	/* Move all tasks to the default resource group */
	rdt_move_group_tasks(NULL, &rdtgroup_default, NULL);

	list_for_each_entry_safe(rdtgrp, tmp, &rdt_all_groups, rdtgroup_list) {
		/* Free any child rmids */
		free_all_child_rdtgrp(rdtgrp);

		/* Remove each rdtgroup other than root */
		if (rdtgrp == &rdtgroup_default)
			continue;

		if (rdtgrp->mode == RDT_MODE_PSEUDO_LOCKSETUP ||
		    rdtgrp->mode == RDT_MODE_PSEUDO_LOCKED)
			rdtgroup_pseudo_lock_remove(rdtgrp);

		/*
		 * Give any CPUs back to the default group. We cannot copy
		 * cpu_online_mask because a CPU might have executed the
		 * offline callback already, but is still marked online.
		 */
		cpumask_or(&rdtgroup_default.cpu_mask,
			   &rdtgroup_default.cpu_mask, &rdtgrp->cpu_mask);

		free_rmid(rdtgrp->mon.rmid);

		kernfs_remove(rdtgrp->kn);
		list_del(&rdtgrp->rdtgroup_list);

		if (atomic_read(&rdtgrp->waitcount) != 0)
			rdtgrp->flags = RDT_DELETED;
		else
			rdtgroup_remove(rdtgrp);
	}
	/* Notify online CPUs to update per cpu storage and PQR_ASSOC MSR */
	update_closid_rmid(cpu_online_mask, &rdtgroup_default);

	kernfs_remove(kn_info);
	kernfs_remove(kn_mongrp);
	kernfs_remove(kn_mondata);
}

static void rdt_kill_sb(struct super_block *sb)
{
	struct rdt_resource *r;

	for_each_capable_rdt_resource(r)
		reset_all_ctrls(r);
}
