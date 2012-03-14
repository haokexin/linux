/* Copyright 2008-2012 Freescale Semiconductor, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "qman_private.h"
#include <sysdev/fsl_pamu.h>

/* Global variable containing revision id (even on non-control plane systems
 * where CCSR isn't available) */
u16 qman_ip_rev;
EXPORT_SYMBOL(qman_ip_rev);

/* size of the fqd region in bytes */
#ifdef CONFIG_FSL_QMAN_FQ_LOOKUP
static u32 fqd_size = (PAGE_SIZE << CONFIG_FSL_QMAN_FQD_SZ);
#endif

/* For these variables, and the portal-initialisation logic, the
 * comments in bman_driver.c apply here so won't be repeated. */
static struct qman_portal *shared_portals[NR_CPUS];
static int num_shared_portals;
static int shared_portals_idx;

/* A SDQCR mask comprising all the available/visible pool channels */
static u32 pools_sdqcr;

static __init int fsl_fqid_range_init(struct device_node *node)
{
	int ret;
	u32 *range = (u32 *)of_get_property(node, "fsl,fqid-range", &ret);
	if (!range) {
		pr_err("No 'fsl,fqid-range' property in node %s\n",
			node->full_name);
		return -EINVAL;
	}
	if (ret != 8) {
		pr_err("'fsl,fqid-range' is not a 2-cell range in node %s\n",
			node->full_name);
		return -EINVAL;
	}
	qman_release_fqid_range(range[0], range[1]);
	pr_info("Qman: FQID allocator includes range %d:%d\n",
		range[0], range[1]);
	return 0;
}

static struct qm_portal_config * __init parse_pcfg(struct device_node *node)
{
	struct qm_portal_config *pcfg;
	const u32 *index, *channel;
	int irq, ret;
	u16 ip_rev = 0;

	pcfg = kmalloc(sizeof(*pcfg), GFP_KERNEL);
	if (!pcfg) {
		pr_err("can't allocate portal config");
		return NULL;
	}

	if (of_device_is_compatible(node, "fsl,qman-portal-1.0"))
		ip_rev = QMAN_REV10;
	else if (of_device_is_compatible(node, "fsl,qman-portal-1.1"))
		ip_rev = QMAN_REV11;
	else if	(of_device_is_compatible(node, "fsl,qman-portal-1.2"))
		ip_rev = QMAN_REV12;
	else if (of_device_is_compatible(node, "fsl,qman-portal-2.0"))
		ip_rev = QMAN_REV20;

	if (!qman_ip_rev) {
		if (ip_rev)
			qman_ip_rev = ip_rev;
		else {
			pr_warning("unknown Qman version, default to rev1.1\n");
			qman_ip_rev = QMAN_REV11;
		}
	} else if (ip_rev && (qman_ip_rev != ip_rev))
		pr_warning("Revision=0x%04x, but portal '%s' has 0x%04x\n",
			qman_ip_rev, node->full_name, ip_rev);

	ret = of_address_to_resource(node, DPA_PORTAL_CE,
				&pcfg->addr_phys[DPA_PORTAL_CE]);
	if (ret) {
		pr_err("Can't get %s property '%s'\n", node->full_name,
			"reg::CE");
		goto err;
	}
	ret = of_address_to_resource(node, DPA_PORTAL_CI,
				&pcfg->addr_phys[DPA_PORTAL_CI]);
	if (ret) {
		pr_err("Can't get %s property '%s'\n", node->full_name,
			"reg::CI");
		goto err;
	}
	index = of_get_property(node, "cell-index", &ret);
	if (!index || (ret != 4)) {
		pr_err("Can't get %s property '%s'\n", node->full_name,
			"cell-index");
		goto err;
	}
	channel = of_get_property(node, "fsl,qman-channel-id", &ret);
	if (!channel || (ret != 4)) {
		pr_err("Can't get %s property '%s'\n", node->full_name,
			"fsl,qman-channel-id");
		goto err;
	}
	if (*channel != (*index + qm_channel_swportal0))
		pr_err("Warning: node %s has mismatched %s and %s\n",
			node->full_name, "cell-index", "fsl,qman-channel-id");
	pcfg->public_cfg.channel = *channel;
	pcfg->public_cfg.cpu = -1;
	irq = irq_of_parse_and_map(node, 0);
	if (irq == NO_IRQ) {
		pr_err("Can't get %s property '%s'\n", node->full_name,
			"interrupts");
		goto err;
	}
	pcfg->public_cfg.irq = irq;
	pcfg->public_cfg.index = *index;
	pcfg->node = node;
#ifdef CONFIG_FSL_QMAN_CONFIG
	/* We need the same LIODN offset for all portals */
	qman_liodn_fixup(pcfg->public_cfg.channel);
#endif

	pcfg->addr_virt[DPA_PORTAL_CE] = ioremap_prot(
				pcfg->addr_phys[DPA_PORTAL_CE].start,
				resource_size(&pcfg->addr_phys[DPA_PORTAL_CE]),
				0);
	pcfg->addr_virt[DPA_PORTAL_CI] = ioremap_prot(
				pcfg->addr_phys[DPA_PORTAL_CI].start,
				resource_size(&pcfg->addr_phys[DPA_PORTAL_CI]),
				_PAGE_GUARDED | _PAGE_NO_CACHE);

	return pcfg;
err:
	kfree(pcfg);
	return NULL;
}

static struct qm_portal_config *get_pcfg(struct list_head *list)
{
	struct qm_portal_config *pcfg;
	if (list_empty(list))
		return NULL;
	pcfg = list_entry(list->prev, struct qm_portal_config, list);
	list_del(&pcfg->list);
	return pcfg;
}

#ifdef CONFIG_FSL_PAMU
static void set_liodns(const struct qm_portal_config *pcfg, int cpu)
{
	unsigned int index = 0;
	int ret;
	do {
		ret = pamu_set_stash_dest(pcfg->node, index++, cpu, 1);
	} while (ret >= 0);
}
#else
#define set_liodns(pcfg, cpu) do { } while (0)
#endif

static struct qman_portal *init_pcfg(struct qm_portal_config *pcfg)
{
	struct qman_portal *p;
	struct cpumask oldmask = *tsk_cpus_allowed(current);

	set_liodns(pcfg, pcfg->public_cfg.cpu);
	set_cpus_allowed_ptr(current, get_cpu_mask(pcfg->public_cfg.cpu));
	p = qman_create_affine_portal(pcfg, NULL);
	if (p) {
		u32 irq_sources = 0;
		/* Determine what should be interrupt-vs-poll driven */
#ifdef CONFIG_FSL_DPA_PIRQ_SLOW
		irq_sources |= QM_PIRQ_EQCI | QM_PIRQ_EQRI | QM_PIRQ_MRI |
			       QM_PIRQ_CSCI;
#endif
#ifdef CONFIG_FSL_DPA_PIRQ_FAST
		irq_sources |= QM_PIRQ_DQRI;
#endif
		qman_irqsource_add(irq_sources);
		pr_info("Qman portal %sinitialised, cpu %d\n",
			pcfg->public_cfg.is_shared ? "(shared) " : "",
			pcfg->public_cfg.cpu);
	} else
		pr_crit("Qman portal failure on cpu %d\n",
			pcfg->public_cfg.cpu);
	set_cpus_allowed_ptr(current, &oldmask);
	return p;
}

static void init_slave(int cpu)
{
	struct qman_portal *p;
	struct cpumask oldmask = *tsk_cpus_allowed(current);
	set_cpus_allowed_ptr(current, get_cpu_mask(cpu));
	p = qman_create_affine_slave(shared_portals[shared_portals_idx++]);
	if (!p)
		pr_err("Qman slave portal failure on cpu %d\n", cpu);
	else
		pr_info("Qman portal %sinitialised, cpu %d\n", "(slave) ", cpu);
	set_cpus_allowed_ptr(current, &oldmask);
	if (shared_portals_idx >= num_shared_portals)
		shared_portals_idx = 0;
}

static struct cpumask want_unshared __initdata;
static struct cpumask want_shared __initdata;

static int __init parse_qportals(char *str)
{
	return parse_portals_bootarg(str, &want_shared, &want_unshared,
				     "qportals");
}
__setup("qportals=", parse_qportals);

static __init int qman_init(void)
{
	struct qman_cgr cgr;
	struct cpumask slave_cpus;
	struct cpumask unshared_cpus = *cpu_none_mask;
	struct cpumask shared_cpus = *cpu_none_mask;
	LIST_HEAD(unused_pcfgs);
	LIST_HEAD(unshared_pcfgs);
	LIST_HEAD(shared_pcfgs);
	struct device_node *dn;
	struct qm_portal_config *pcfg;
	struct qman_portal *p;
	int cpu, ret;

	/* Initialise the Qman (CCSR) device */
	for_each_compatible_node(dn, NULL, "fsl,qman") {
		if (!qman_init_error_int(dn))
			pr_info("Qman err interrupt handler present\n");
		else
			pr_err("Qman err interrupt handler missing\n");
	}
	/* Initialise FQID allocation ranges */
	for_each_compatible_node(dn, NULL, "fsl,fqid-range") {
		ret = fsl_fqid_range_init(dn);
		if (ret)
			return ret;
	}
	/* Parse pool channels */
	for_each_compatible_node(dn, NULL, "fsl,qman-pool-channel") {
		const u32 *index = of_get_property(dn, "cell-index", NULL);
		pools_sdqcr |= QM_SDQCR_CHANNELS_POOL(*index);
	}
#ifdef CONFIG_FSL_QMAN_FQ_LOOKUP
	/* Setup lookup table for FQ demux */
	ret = qman_setup_fq_lookup_table(fqd_size/64);
	if (ret)
		return ret;
#endif
	/* Initialise portals. See bman_driver.c for comments */
	for_each_compatible_node(dn, NULL, "fsl,qman-portal") {
		pcfg = parse_pcfg(dn);
		if (pcfg) {
			pcfg->public_cfg.pools = pools_sdqcr;
			list_add_tail(&pcfg->list, &unused_pcfgs);
		}
	}
	for_each_cpu(cpu, &want_shared) {
		pcfg = get_pcfg(&unused_pcfgs);
		if (!pcfg)
			break;
		pcfg->public_cfg.cpu = cpu;
		list_add_tail(&pcfg->list, &shared_pcfgs);
		cpumask_set_cpu(cpu, &shared_cpus);
	}
	for_each_cpu(cpu, &want_unshared) {
		if (cpumask_test_cpu(cpu, &shared_cpus))
			continue;
		pcfg = get_pcfg(&unused_pcfgs);
		if (!pcfg)
			break;
		pcfg->public_cfg.cpu = cpu;
		list_add_tail(&pcfg->list, &unshared_pcfgs);
		cpumask_set_cpu(cpu, &unshared_cpus);
	}
	if (list_empty(&shared_pcfgs) && list_empty(&unshared_pcfgs)) {
		for_each_online_cpu(cpu) {
			pcfg = get_pcfg(&unused_pcfgs);
			if (!pcfg)
				break;
			pcfg->public_cfg.cpu = cpu;
			list_add_tail(&pcfg->list, &unshared_pcfgs);
			cpumask_set_cpu(cpu, &unshared_cpus);
		}
	}
	cpumask_andnot(&slave_cpus, cpu_online_mask, &shared_cpus);
	cpumask_andnot(&slave_cpus, &slave_cpus, &unshared_cpus);
	if (cpumask_empty(&slave_cpus)) {
		if (!list_empty(&shared_pcfgs)) {
			cpumask_or(&unshared_cpus, &unshared_cpus,
				   &shared_cpus);
			cpumask_clear(&shared_cpus);
			list_splice_tail(&shared_pcfgs, &unshared_pcfgs);
			INIT_LIST_HEAD(&shared_pcfgs);
		}
	} else {
		if (list_empty(&shared_pcfgs)) {
			pcfg = get_pcfg(&unshared_pcfgs);
			if (!pcfg) {
				pr_crit("No QMan portals available!\n");
				return 0;
			}
			cpumask_clear_cpu(pcfg->public_cfg.cpu, &unshared_cpus);
			cpumask_set_cpu(pcfg->public_cfg.cpu, &shared_cpus);
			list_add_tail(&pcfg->list, &shared_pcfgs);
		}
	}
	list_for_each_entry(pcfg, &unshared_pcfgs, list) {
		pcfg->public_cfg.is_shared = 0;
		p = init_pcfg(pcfg);
	}
	list_for_each_entry(pcfg, &shared_pcfgs, list) {
		pcfg->public_cfg.is_shared = 1;
		p = init_pcfg(pcfg);
		if (p)
			shared_portals[num_shared_portals++] = p;
	}
	if (!cpumask_empty(&slave_cpus))
		for_each_cpu(cpu, &slave_cpus)
			init_slave(cpu);
	pr_info("Qman portals initialised\n");

	/* This is to ensure h/w-internal CGR memory is zeroed out. Note that we
	 * do this for all conceivable CGRIDs, not all of which are necessarily
	 * available on the underlying hardware version. We ignore any errors
	 * for this reason. */
	for (cgr.cgrid = 0; cgr.cgrid < __CGR_NUM; cgr.cgrid++)
		qman_modify_cgr(&cgr, QMAN_CGR_FLAG_USE_INIT, NULL);
	return 0;
}
subsys_initcall(qman_init);
