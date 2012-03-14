/* Copyright 2011 Freescale Semiconductor, Inc.
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

#include "bman_private.h"
#include "qman_private.h"

static const char dpa_uio_version[] = "USDPAA UIO portal driver v0.2";

static LIST_HEAD(uio_portal_list);

struct dpa_uio_info {
	atomic_t ref; /* exclusive, only one open() at a time */
	struct uio_info uio;
	void *addr_ci;
	char name[16]; /* big enough for "qman-uio-xx" */
	struct platform_device *pdev;
	struct list_head node;
};

static int dpa_uio_open(struct uio_info *info, struct inode *inode)
{
	struct dpa_uio_info *i = container_of(info, struct dpa_uio_info, uio);
	if (!atomic_dec_and_test(&i->ref)) {
		atomic_inc(&i->ref);
		return -EBUSY;
	}
	return 0;
}

static int dpa_uio_release(struct uio_info *info, struct inode *inode)
{
	struct dpa_uio_info *i = container_of(info, struct dpa_uio_info, uio);
	atomic_inc(&i->ref);
	return 0;
}

static pgprot_t dpa_uio_pgprot(struct uio_info *info, unsigned int mem_idx,
				   pgprot_t prot)
{
	if (mem_idx == DPA_PORTAL_CE)
		/* It's the cache-enabled portal region. NB, we shouldn't use
		 * pgprot_cached() here because it includes _PAGE_COHERENT. The
		 * region is cachable but *not* coherent - stashing (if enabled)
		 * leads to "coherent-like" behaviour, otherwise the driver
		 * explicitly invalidates/prefetches. */
		return pgprot_cached_noncoherent(prot);
	/* Otherwise it's the cache-inhibited portal region */
	return pgprot_noncached(prot);
}

static irqreturn_t dpa_uio_irq_handler(int irq, struct uio_info *info)
{
	struct dpa_uio_info *i = container_of(info, struct dpa_uio_info, uio);
	/* This is the only code outside the regular portal driver that
	 * manipulates any portal register, so rather than breaking that
	 * encapsulation I am simply hard-coding the offset to the inhibit
	 * register here. */
	out_be32(i->addr_ci + 0xe0c, ~(u32)0);
	return IRQ_HANDLED;
}

static void __init dpa_uio_portal_init(struct dpa_uio_portal *p,
				const struct dpa_uio_class *c)
{
	struct dpa_uio_info *info;
	const struct resource *res;
	u32 index;
	int irq, ret;

	/* allocate 'info' */
	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return;
	atomic_set(&info->ref, 1);
	if (p->type == dpa_uio_portal_bman) {
		res = &p->bm_cfg->addr_phys[0];
		index = p->bm_cfg->public_cfg.index;
		irq = p->bm_cfg->public_cfg.irq;
	} else {
		res = &p->qm_cfg->addr_phys[0];
		index = p->qm_cfg->public_cfg.index;
		irq = p->qm_cfg->public_cfg.irq;
	}
	/* We need to map the cache-inhibited region in the kernel for
	 * interrupt-handling purposes. */
	info->addr_ci = ioremap_prot(res[DPA_PORTAL_CI].start,
				resource_size(&res[DPA_PORTAL_CI]),
				_PAGE_GUARDED | _PAGE_NO_CACHE);
	/* Name the UIO device according to the cell-index. It's supposed to be
	 * unique for each device class (Qman/Bman), and is also a convenient
	 * way for user-space to find the UIO device that corresponds to a given
	 * portal device-tree node. */
	sprintf(info->name, "%s%x", c->dev_prefix, index);
	info->pdev = platform_device_alloc(info->name, -1);
	if (!info->pdev) {
		iounmap(info->addr_ci);
		kfree(info);
		pr_err("dpa_uio_portal: platform_device_alloc() failed\n");
		return;
	}
	ret = platform_device_add(info->pdev);
	if (ret) {
		platform_device_put(info->pdev);
		iounmap(info->addr_ci);
		kfree(info);
		pr_err("dpa_uio_portal: platform_device_add() failed\n");
		return;
	}
	info->uio.name = info->name;
	info->uio.version = dpa_uio_version;
	info->uio.mem[DPA_PORTAL_CE].name = "cena";
	info->uio.mem[DPA_PORTAL_CE].addr = res[DPA_PORTAL_CE].start;
	info->uio.mem[DPA_PORTAL_CE].size = resource_size(&res[DPA_PORTAL_CE]);
	info->uio.mem[DPA_PORTAL_CE].memtype = UIO_MEM_PHYS;
	info->uio.mem[DPA_PORTAL_CI].name = "cinh";
	info->uio.mem[DPA_PORTAL_CI].addr = res[DPA_PORTAL_CI].start;
	info->uio.mem[DPA_PORTAL_CI].size = resource_size(&res[DPA_PORTAL_CI]);
	info->uio.mem[DPA_PORTAL_CI].memtype = UIO_MEM_PHYS;
	info->uio.irq = irq;
	info->uio.handler = dpa_uio_irq_handler;
	info->uio.set_pgprot = dpa_uio_pgprot;
	info->uio.open = dpa_uio_open;
	info->uio.release = dpa_uio_release;
	ret = uio_register_device(&info->pdev->dev, &info->uio);
	if (ret) {
		platform_device_del(info->pdev);
		platform_device_put(info->pdev);
		iounmap(info->addr_ci);
		kfree(info);
		pr_err("dpa_uio_portal: UIO registration failed\n");
		return;
	}
	list_add_tail(&info->node, &uio_portal_list);
	pr_info("USDPAA portal initialised, %s\n", info->name);
}

static int __init dpa_uio_init(void)
{
	const struct dpa_uio_class *classes[3], **c = classes;
	classes[0] = dpa_uio_bman();
	classes[1] = dpa_uio_qman();
	classes[2] = NULL;
	while (*c) {
		struct dpa_uio_portal *p;
		list_for_each_entry(p, &(*c)->list, node)
			dpa_uio_portal_init(p, *c);
		c++;
	}
	pr_info("USDPAA portal layer loaded\n");
	return 0;
}

static void __exit dpa_uio_exit(void)
{
	struct dpa_uio_info *info, *tmp;
	list_for_each_entry_safe(info, tmp, &uio_portal_list, node) {
		list_del(&info->node);
		uio_unregister_device(&info->uio);
		platform_device_del(info->pdev);
		platform_device_put(info->pdev);
		iounmap(info->addr_ci);
		pr_info("USDPAA portal removed, %s\n", info->name);
		kfree(info);
	}
	pr_info("USDPAA portal layer unloaded\n");
}


module_init(dpa_uio_init)
module_exit(dpa_uio_exit)
MODULE_LICENSE("GPL");

