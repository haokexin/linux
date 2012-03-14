/* Copyright 2008-2012 Freescale Semiconductor, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
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

/* Global variable containing revision id (even on non-control plane systems
 * where CCSR isn't available) */
u16 qman_ip_rev;
EXPORT_SYMBOL(qman_ip_rev);

/* size of the fqd region in bytes */
#ifdef CONFIG_FSL_QMAN_FQ_LOOKUP
static u32 fqd_size = (PAGE_SIZE << CONFIG_FSL_QMAN_FQD_SZ);
#endif

#ifdef CONFIG_FSL_QMAN_PORTAL
static __init struct qman_portal *init_affine_portal(
					struct qm_portal_config *pconfig,
					int cpu, struct qman_portal *redirect)
{
	struct qman_portal *portal;
	struct cpumask oldmask = *tsk_cpus_allowed(current);
	const struct cpumask *newmask = get_cpu_mask(cpu);

	set_cpus_allowed_ptr(current, newmask);

	if (redirect)
		portal = qman_create_affine_slave(redirect);
	else {
		portal = qman_create_affine_portal(pconfig, NULL);
		if (portal) {
			u32 irq_sources = 0;
			/* default: enable all (available) pool channels */
			qman_static_dequeue_add(~0);
			/* Determine what should be interrupt-vs-poll driven */
#ifdef CONFIG_FSL_DPA_PIRQ_SLOW
			irq_sources |= QM_PIRQ_EQCI | QM_PIRQ_EQRI |
				QM_PIRQ_MRI | QM_PIRQ_CSCI;
#endif
#ifdef CONFIG_FSL_DPA_PIRQ_FAST
			irq_sources |= QM_PIRQ_DQRI;
#endif
			qman_irqsource_add(irq_sources);
		}
	}

	set_cpus_allowed_ptr(current, &oldmask);
	if (portal)
		pr_info("Qman portal %sinitialised, cpu %d\n",
			redirect ? "(slave) " :
			pconfig->public_cfg.is_shared ? "(shared) " : "", cpu);
	return portal;
}
#endif

/* Parses the device-tree node, extracts the configuration, and if appropriate
 * initialises the portal for use on one or more CPUs. */
static __init struct qm_portal_config *fsl_qman_portal_init(
					struct device_node *node)
{
	struct qm_portal_config *pcfg;
	const u32 *index, *channel;
	const phandle *ph;
	struct device_node *tmp_node;
	int irq, ret, numpools;
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
	/* Parse cpu associations for this portal. This involves dereferencing
	 * to the cpu device-tree nodes, but it also ensures we only try to work
	 * with CPUs that exist. (Eg. under a hypervisor.) */
	ph = of_get_property(node, "cpu-handle", &ret);
	if (ph) {
		if (ret != sizeof(phandle)) {
			pr_err("Malformed %s property '%s'\n", node->full_name,
				"cpu-handle");
			return NULL;
		}
		ret = check_cpu_phandle(*ph);
		if (ret < 0)
			return NULL;
		pcfg->public_cfg.cpu = ret;
	} else
		pcfg->public_cfg.cpu = -1;

	ph = of_get_property(node, "fsl,qman-pool-channels", &ret);
	if (ph && (ret % sizeof(phandle))) {
		pr_err("Malformed %s property '%s'\n", node->full_name,
			"fsl,qman-pool-channels");
		goto err;
	}
	numpools = ph ? (ret / sizeof(phandle)) : 0;
	irq = irq_of_parse_and_map(node, 0);
	if (irq == NO_IRQ) {
		pr_err("Can't get %s property '%s'\n", node->full_name,
			"interrupts");
		goto err;
	}
	pcfg->public_cfg.irq = irq;
	pcfg->public_cfg.index = *index;
	pcfg->public_cfg.pools = 0;
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

	while (numpools--) {
		for_each_compatible_node(tmp_node, NULL,
					 "fsl,qman-pool-channel") {
			phandle *lph = (phandle *)of_get_property(tmp_node,
				 "linux,phandle", &ret);
			if (*lph == *ph) {
				u32 *index = (u32 *)of_get_property(tmp_node,
					"cell-index", &ret);
				pcfg->public_cfg.pools |=
					QM_SDQCR_CHANNELS_POOL(*index);
			}
		}
		ph++;
	}
	if (pcfg->public_cfg.pools == 0)
		panic("Unrecoverable error linking pool channels");

	return pcfg;
err:
	kfree(pcfg);
	return NULL;
}

static void __init fsl_qman_portal_destroy(struct qm_portal_config *pcfg)
{
	iounmap(pcfg->addr_virt[DPA_PORTAL_CE]);
	iounmap(pcfg->addr_virt[DPA_PORTAL_CI]);
	kfree(pcfg);
}

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

/***************/
/* Driver load */
/***************/

static __init int qman_init(void)
{
#ifdef CONFIG_FSL_QMAN_PORTAL
	struct qman_cgr cgr;
	struct cpumask primary_cpus = *cpu_none_mask;
	struct cpumask slave_cpus = *cpu_online_mask;
	struct qman_portal *sharing_portal = NULL;
	int sharing_cpu = -1;
#endif
	struct device_node *dn;
	struct qm_portal_config *pcfg;
	int ret, use_bpid0 = 1;
	LIST_HEAD(cfg_list);

	for_each_compatible_node(dn, NULL, "fsl,qman") {
		if (!qman_init_error_int(dn))
			pr_info("Qman err interrupt handler present\n");
		else
			pr_err("Qman err interrupt handler missing\n");
	}
#ifdef CONFIG_FSL_QMAN_FQ_LOOKUP
	ret = qman_setup_fq_lookup_table(fqd_size/64);
	if (ret)
		return ret;
#endif
#ifdef CONFIG_FSL_QMAN_PORTAL
	for_each_compatible_node(dn, NULL, "fsl,qman-portal") {
		if (!of_device_is_available(dn))
			continue;
		pcfg = fsl_qman_portal_init(dn);
		if (pcfg) {
			if (pcfg->public_cfg.cpu >= 0) {
				cpumask_set_cpu(pcfg->public_cfg.cpu,
						&primary_cpus);
				list_add(&pcfg->list, &cfg_list);
			} else
				fsl_qman_portal_destroy(pcfg);
		}
	}
	/* only consider "online" CPUs */
	cpumask_and(&primary_cpus, &primary_cpus, cpu_online_mask);
	if (cpumask_empty(&primary_cpus))
		/* No portals, we're done */
		return 0;
	if (!cpumask_subset(cpu_online_mask, &primary_cpus)) {
		/* Need to do some sharing. In lieu of anything more scientific
		 * (or configurable), we pick the last-most CPU that has a
		 * portal and share that one. */
		int next = cpumask_first(&primary_cpus);
		while (next < nr_cpu_ids) {
			sharing_cpu = next;
			next = cpumask_next(next, &primary_cpus);
		}
	}
	/* Parsing is done and sharing decisions are made, now initialise the
	 * portals and determine which "slave" CPUs are left over. */
	list_for_each_entry(pcfg, &cfg_list, list) {
		struct qman_portal *p;
		int is_shared = (!sharing_portal && (sharing_cpu >= 0) &&
				(pcfg->public_cfg.cpu == sharing_cpu));
		pcfg->public_cfg.is_shared = is_shared;
		/* If it's not mapped to a CPU, or another portal is already
		 * initialised to the same CPU, skip this portal. */
		if (pcfg->public_cfg.cpu < 0 || !cpumask_test_cpu(
					pcfg->public_cfg.cpu, &slave_cpus))
			continue;
		p = init_affine_portal(pcfg, pcfg->public_cfg.cpu, NULL);
		if (p) {
			if (is_shared)
				sharing_portal = p;
			cpumask_clear_cpu(pcfg->public_cfg.cpu, &slave_cpus);
		}
	}
	if (sharing_portal) {
		int loop;
		for_each_cpu(loop, &slave_cpus) {
			struct qman_portal *p = init_affine_portal(NULL, loop,
					sharing_portal);
			if (!p)
				pr_err("Failed slave Qman portal for cpu %d\n",
					loop);
		}
	}
#else
	for_each_compatible_node(dn, NULL, "fsl,qman-portal") {
		if (!of_device_is_available(dn))
			continue;
		pcfg = fsl_qman_portal_init(dn);
		if (pcfg)
			/* No kernel portal support, so if USDPAA didn't consume
			 * the portal, we've no other use for it. */
			fsl_qman_portal_destroy(pcfg);
	}
#endif
	for_each_compatible_node(dn, NULL, "fsl,fqid-range") {
		use_bpid0 = 0;
		ret = fsl_fqid_range_init(dn);
		if (ret)
			return ret;
	}
#ifdef CONFIG_FSL_QMAN_PORTAL
	for (cgr.cgrid = 0; cgr.cgrid < __CGR_NUM; cgr.cgrid++) {
		/* This is to ensure h/w-internal CGR memory is zeroed out. Note
		 * that we do this for all conceivable CGRIDs, not all of which
		 * are necessarily available on the underlying hardware version.
		 * We ignore any errors for this reason. */
		qman_modify_cgr(&cgr, QMAN_CGR_FLAG_USE_INIT, NULL);
	}
#endif
	ret = fqalloc_init(use_bpid0);
	if (ret)
		return ret;
	pr_info("Qman portals initialised\n");
	return 0;
}
subsys_initcall(qman_init);
