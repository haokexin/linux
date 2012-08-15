/* Copyright (c) 2008-2011 Freescale Semiconductor, Inc.
 * All rights reserved.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/of_platform.h>
#include <linux/bootmem.h>
#include <asm/io.h>
#include <asm/bitops.h>

#include "fsl_pamu.h"

#define PAMUBYPENR 0x604

/* define indexes for each operation mapping scenario */
#define OMI_QMAN        0x00
#define OMI_FMAN        0x01
#define OMI_QMAN_PRIV   0x02
#define OMI_CAAM        0x03

void setup_omt(struct ome *omt)
{
	struct ome *ome;

	/* Configure OMI_QMAN */
	ome = &omt[OMI_QMAN];

	ome->moe[IOE_READ_IDX] = EOE_VALID | EOE_READ;
	ome->moe[IOE_EREAD0_IDX] = EOE_VALID | EOE_RSA;
	ome->moe[IOE_WRITE_IDX] = EOE_VALID | EOE_WRITE;
	ome->moe[IOE_EWRITE0_IDX] = EOE_VALID | EOE_WWSAO;

	/*
	 * When it comes to stashing DIRECTIVEs, the QMan BG says
	 * (1.5.6.7.1:  FQD Context_A field used for dequeued etc.
	 * etc. stashing control):
	 * - AE/DE/CE == 0:  don't stash exclusive.  Use DIRECT0,
	 *                   which should be a non-PE LOADEC.
	 * - AE/DE/CE == 1:  stash exclusive via DIRECT1, i.e.
	 *                   LOADEC-PE
	 * If one desires to alter how the three different types of
	 * stashing are done, please alter rx_conf.exclusive in
	 * ipfwd_a.c (that specifies the 3-bit AE/DE/CE field), and
	 * do not alter the settings here.  - bgrayson
	 */
	ome->moe[IOE_DIRECT0_IDX] = EOE_VALID | EOE_LDEC;
	ome->moe[IOE_DIRECT1_IDX] = EOE_VALID | EOE_LDECPE;

	/* Configure OMI_FMAN */
	ome = &omt[OMI_FMAN];
	ome->moe[IOE_READ_IDX]  = EOE_VALID | EOE_READI;
	ome->moe[IOE_WRITE_IDX] = EOE_VALID | EOE_WRITE;

	/* Configure OMI_QMAN private */
	ome = &omt[OMI_QMAN_PRIV];
	ome->moe[IOE_READ_IDX]  = EOE_VALID | EOE_READ;
	ome->moe[IOE_WRITE_IDX] = EOE_VALID | EOE_WRITE;
	ome->moe[IOE_EREAD0_IDX] = EOE_VALID | EOE_RSA;
	ome->moe[IOE_EWRITE0_IDX] = EOE_VALID | EOE_WWSA;

	/* Configure OMI_CAAM */
	ome = &omt[OMI_CAAM];
	ome->moe[IOE_READ_IDX]  = EOE_VALID | EOE_READI;
	ome->moe[IOE_WRITE_IDX] = EOE_VALID | EOE_WRITE;
}

#define L1 1
#define L2 2
#define L3 3

u32 get_stash_id(u32 stash_dest_hint, struct device_node *portal_dn)
{
	const u32 *prop;
	struct device_node *node;
	u32 cache_level;

	/* Fastpath, exit early if L3/CPC cache is target for stashing */
	if (stash_dest_hint == L3) {
		node = of_find_compatible_node(NULL, NULL,
				"fsl,p4080-l3-cache-controller");
		if (node) {
			prop = of_get_property(node, "cache-stash-id", 0);
			if (!prop) {
				printk(KERN_ERR "missing cache-stash-id at %s\n", node->full_name);
				of_node_put(node);
				return ~(u32)0;
			}
			of_node_put(node);
			return *prop;
		}
		return ~(u32)0;
	}

	prop = of_get_property(portal_dn, "cpu-handle", 0);
	/* if no cpu-phandle assume that this is not a per-cpu portal */
	if (!prop)
		return ~(u32)0;

	node = of_find_node_by_phandle(*prop);
	if (!node) {
		printk(KERN_ERR "bad cpu phandle reference in %s\n",
				 portal_dn->full_name);
		return ~(u32)0;
	}

	/* find the hwnode that represents the cache */
	for (cache_level = L1; cache_level <= L3; cache_level++) {
		if (stash_dest_hint == cache_level) {
			prop = of_get_property(node, "cache-stash-id", 0);
			if (!prop) {
				printk(KERN_ERR "missing cache-stash-id at %s\n", node->full_name);
				of_node_put(node);
				return ~(u32)0;
			}
			of_node_put(node);
			return *prop;
		}

		prop = of_get_property(node, "next-level-cache", 0);
		if (!prop) {
			printk(KERN_ERR "can't find next-level-cache at %s\n",
			          node->full_name);
			of_node_put(node);
			return ~(u32)0;  /* can't traverse any further */
		}
		of_node_put(node);

		/* advance to next node in cache hierarchy */
		node = of_find_node_by_phandle(*prop);
		if (!node) {
			printk(KERN_ERR "bad cpu phandle reference in %s\n",
			          portal_dn->full_name);
			return ~(u32)0;
		}
	}

	printk(KERN_ERR "stash dest not found for %d on %s\n",
	          stash_dest_hint, portal_dn->full_name);
	return ~(u32)0;
}

void setup_liodns(struct ppaace *ppaact)
{
	int i, len;
	struct ppaace *ppaace;
	struct device_node *qman_portal_dn = NULL;
	struct device_node *qman_dn = NULL;
	const u32 *prop;
	u32 cache_id, prop_cnt;

	for (i = 0; i < PAACE_NUMBER_ENTRIES; i++) {
		ppaace = &ppaact[i];
		ppaace->pt = PAACE_PT_PRIMARY;
		ppaace->domain_attr.to_host.coherency_required =
				PAACE_M_COHERENCE_REQ;
		/* window size is 2^(WSE+1) bytes */
		ppaace->wse = 35; /* 36-bit phys. addr space */
		ppaace->wbah = ppaace->wbal = 0;
		ppaace->atm = PAACE_ATM_NO_XLATE;
		ppaace->ap = PAACE_AP_PERMS_ALL;
		mb();
		ppaace->v = 1;
	}

	/*
	 * Now, do specific stashing setup for qman portals.
	 * We need stashing setup for LIODNs for  qman portal(s) dqrr stashing
	 * (DLIODNs), qman portal(s) data stashing (FLIODNs)
	 */

	for_each_compatible_node(qman_portal_dn, NULL, "fsl,qman-portal") {

		pr_debug("qman portal found, name = %s\n",
					qman_portal_dn->full_name);
		prop = of_get_property(qman_portal_dn, "fsl,liodn", &len);
		if (prop) {
			prop_cnt = len / sizeof(u32);
			do {
				pr_debug("liodn = %d\n", *prop);
				ppaace = &ppaact[*prop++];
				ppaace->otm = PAACE_OTM_INDEXED;
				ppaace->op_encode.index_ot.omi = OMI_QMAN;
				cache_id = get_stash_id(L1, qman_portal_dn);
				pr_debug("cache_stash_id = %d\n", cache_id);
				if (~cache_id != 0)
					ppaace->impl_attr.cid = cache_id;
			} while(--prop_cnt);
		} else {
			printk (KERN_ERR "missing fsl,liodn property at %s\n",
			          qman_portal_dn->full_name);
		}
	}

	/*
	 * Next, do stashing setups for qman private memory access
	 */

	qman_dn = of_find_compatible_node(NULL, NULL, "fsl,qman");
	if (qman_dn) {
		prop = of_get_property(qman_dn, "fsl,liodn", NULL);
		if (prop) {
			ppaace = &ppaact[*prop];
			ppaace->otm = PAACE_OTM_INDEXED;
			ppaace->op_encode.index_ot.omi = OMI_QMAN_PRIV;
			cache_id = get_stash_id(L3, qman_dn);
			pr_debug("cache_stash_id = %d\n", cache_id);
			if (~cache_id != 0)
				ppaace->impl_attr.cid = cache_id;
		} else {
			printk (KERN_ERR "missing fsl,liodn property at %s\n",
			          qman_dn->full_name);
		}
		of_node_put(qman_dn);
	}
}

int setup_one_pamu(unsigned long pamu_reg_base, unsigned long pamu_reg_size,
			struct ppaace *ppaact, struct ome *omt)
{
	u32 *pc;
	phys_addr_t phys;
	struct pamu_mmap_regs *pamu_regs;

	pc = (u32 *) (pamu_reg_base + PAMU_PC);
	pamu_regs = (struct pamu_mmap_regs *) 
		(pamu_reg_base + PAMU_MMAP_REGS_BASE);

	/* set up pointers to corenet control blocks */

	phys = virt_to_phys(ppaact);
	out_be32(&pamu_regs->ppbah, ((u64)phys) >> 32);
	out_be32(&pamu_regs->ppbal, phys);
	phys = virt_to_phys(ppaact + PAACE_NUMBER_ENTRIES);
	out_be32(&pamu_regs->pplah, ((u64)phys) >> 32);
	out_be32(&pamu_regs->pplal, phys);

	phys = virt_to_phys(omt);
	out_be32(&pamu_regs->obah, ((u64)phys) >> 32);
	out_be32(&pamu_regs->obal, phys);
	phys = virt_to_phys(omt + OME_NUMBER_ENTRIES);
	out_be32(&pamu_regs->olah, ((u64)phys) >> 32);
	out_be32(&pamu_regs->olal, phys);

	/*
	 * set PAMU enable bit,
	 * allow ppaact & omt to be cached
	 * & enable PAMU access violation interrupts.
	 */

	out_be32((u32 *)(pamu_reg_base + PAMU_PICS),
			PAMU_ACCESS_VIOLATION_ENABLE);
	out_be32(pc, PAMU_PC_PE | PAMU_PC_OCE | PAMU_PC_SPCC | PAMU_PC_PPCC);
	return 0;
}

irqreturn_t pamu_av_isr(int irq, void *arg)
{
	panic("FSL_PAMU: access violation interrupt\n");
	/* NOTREACHED */
}

static int __devinit fsl_of_pamu_probe(struct platform_device *dev)
{
	void __iomem *pamu_regs, *guts_regs;
	u32 pamubypenr, pamu_counter;
	unsigned long pamu_reg_base, pamu_reg_off;
	struct device_node *guts_node;
	u64 size;
	struct ppaace *ppaact = NULL;
	struct ome *omt = NULL;
	int irq;
	struct page *p;

	printk(KERN_INFO "Setting Freescale static PAMU/IOMMU configuration\n");

	/*
	 * enumerate all PAMUs and allocate and setup PAMU tables
	 * for each of them,
	 * NOTE : All PAMUs share the same LIODN tables.
	 */

	pamu_regs = of_iomap(dev->dev.of_node, 0);
	if (!pamu_regs) {
		dev_err(&dev->dev, "ioremap failed\n");
		return -ENOMEM;
	}
	of_get_address(dev->dev.of_node, 0, &size, NULL);

	guts_node = of_find_compatible_node(NULL, NULL,
			"fsl,qoriq-device-config-1.0");
	if (!guts_node) {
		dev_err(&dev->dev, "%s guts devnode not found!\n",
				dev->dev.of_node->full_name);
		iounmap(pamu_regs);
		return -ENODEV;
	}

	guts_regs = of_iomap(guts_node, 0);
	if (!guts_regs) {
		dev_err(&dev->dev, "guts ioremap failed\n");
		iounmap(pamu_regs);
		return -ENOMEM;
	}
	of_node_put(guts_node);

	p = alloc_pages(GFP_KERNEL, get_order(PAACT_SIZE));
	if (!p) {
		printk(KERN_ERR "Unable to allocate space for PAACT table\n");
		iounmap(pamu_regs);
		iounmap(guts_regs);
		return -ENOMEM;
	}
	ppaact = page_address(p);
	memset(ppaact, 0, PAACT_SIZE);

	pr_debug("fsl_pamu, paact_mem, v : %p, p : 0x%lx\n",
			ppaact, virt_to_phys(ppaact));

	p = alloc_pages(GFP_KERNEL, get_order(OMT_SIZE));
	if (!p) {
		printk(KERN_ERR "Unable to allocate space for OMT table\n");
		iounmap(pamu_regs);
		iounmap(guts_regs);
		free_pages((unsigned long)ppaact, get_order(PAACT_SIZE));
		return -ENOMEM;
	}
	omt = page_address(p);
	memset(omt, 0, OMT_SIZE);

	pr_debug("fsl_pamu, omt_mem, v : %p, p : 0x%lx\n", 
			omt, virt_to_phys(omt));

	pamubypenr = in_be32(guts_regs + PAMUBYPENR);

	for (pamu_reg_off = 0, pamu_counter = 0x80000000; pamu_reg_off < size;
	     pamu_reg_off += PAMU_OFFSET, pamu_counter >>= 1) {

		pamu_reg_base = (unsigned long) pamu_regs + pamu_reg_off;
		setup_one_pamu(pamu_reg_base, pamu_reg_off, ppaact, omt);

		/* Disable PAMU bypass for this PAMU */
		pamubypenr &= ~pamu_counter;
	}

	setup_omt(omt);

	irq = irq_of_parse_and_map(dev->dev.of_node, 0);
	if (request_irq(irq, pamu_av_isr, IRQF_DISABLED, "pamu", 0) < 0) {
		printk(KERN_ERR "Cannot request PAMU AV interrupt\n");
		iounmap(pamu_regs);
		iounmap(guts_regs);
		free_pages((unsigned long)ppaact, get_order(PAACT_SIZE));
		free_pages((unsigned long)omt, get_order(OMT_SIZE));
		return -ENODEV;
	}

	/* 
	 * setup all LIODNS(s) to define a 1:1 mapping for the entire
	 * 36-bit physical address space
	 */
	setup_liodns(ppaact);
	mb();

	/* Enable all relevant PAMU(s) */
	out_be32(guts_regs + PAMUBYPENR, pamubypenr);

	return 0;
}

static const struct of_device_id fsl_of_pamu_ids[] = {
	{
		.compatible = "fsl,p4080-pamu",
	},
	{
		.compatible = "fsl,pamu",
	},
	{},
};

static struct platform_driver fsl_of_pamu_driver = {
	.driver = {
		.name = "fsl-of-pamu",
		.of_match_table = fsl_of_pamu_ids,
	},
	.probe = fsl_of_pamu_probe,
};

static __init int fsl_of_pamu_init(void)
{
	return platform_driver_register(&fsl_of_pamu_driver);
}

arch_initcall(fsl_of_pamu_init);
