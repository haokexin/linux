/*
 * Copyright 2008-2012 Freescale Semiconductor Inc.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/sort.h>
#include <linux/of_mdio.h>
#include <linux/of_platform.h>
#include <linux/of_net.h>
#include <linux/kthread.h>
#include <linux/io.h>
#include <linux/etherdevice.h>
#include <linux/if_arp.h>	/* arp_hdr_len() */
#include <linux/if_vlan.h>	/* VLAN_HLEN */
#include <linux/icmp.h>		/* struct icmphdr */
#include <linux/ip.h>		/* struct iphdr */
#include <linux/ipv6.h>		/* struct ipv6hdr */
#include <linux/udp.h>		/* struct udphdr */
#include <linux/tcp.h>		/* struct tcphdr */
#include <linux/net.h>		/* net_ratelimit() */
#include <linux/if_ether.h>	/* ETH_P_IP and ETH_P_IPV6 */
#include <linux/highmem.h>
#include <linux/percpu.h>
#include <linux/dma-mapping.h>
#include <asm/smp.h>		/* get_hard_smp_processor_id() */
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif
#include <linux/fsl_bman.h>

#include "fsl_fman.h"
#include "fm_ext.h"
#include "fm_port_ext.h"

#include "mac.h"
#include "dpaa_eth.h"
#include "dpaa_1588.h"

#define ARRAY2_SIZE(arr)	(ARRAY_SIZE(arr) * ARRAY_SIZE((arr)[0]))

#define DEFAULT_COUNT		64

#define DPA_NAPI_WEIGHT		64

/*
 * Size in bytes of the QMan taildrop threshold.
 * If a FQ contains a number of bytes higher than this threshold,
 * it will drop any subsequently enqueued frame.
 *
 * In the future, we may want to have different values for queues
 * belonging to 1G vs 10G ports.
 */
#define DPA_TAILDROP_THRESHOLD	0x100000

/* S/G table requires at least 256 bytes */
#define SGT_BUFFER_SIZE		DPA_BP_SIZE(256)

/* Maximum frame size on Tx for which skb copying is preferrable to
 * creating a S/G frame */
#define DPA_SKB_COPY_MAX_SIZE	256

/* Bootarg used to override the Kconfig DPA_MAX_FRM_SIZE value */
#define FSL_FMAN_PHY_MAXFRM_BOOTARG	"fsl_fman_phy_max_frm"

/* Bootarg used to override DPA_EXTRA_HEADROOM Kconfig value */
#define DPA_EXTRA_HEADROOM_BOOTARG	"dpa_extra_headroom"

/* Valid checksum indication */
#define DPA_CSUM_VALID		0xFFFF

/* Maximum offset value for a contig or sg FD (represented on 9bits) */
#define DPA_MAX_FD_OFFSET	((1 << 9) - 1)

#define DPA_DESCRIPTION "FSL DPAA Ethernet driver"

MODULE_LICENSE("Dual BSD/GPL");

MODULE_AUTHOR("Andy Fleming <afleming@freescale.com>");

MODULE_DESCRIPTION(DPA_DESCRIPTION);

static uint8_t debug = -1;
module_param(debug, byte, S_IRUGO);
MODULE_PARM_DESC(debug, "Module/Driver verbosity level");

static uint16_t __devinitdata tx_timeout = 1000;
module_param(tx_timeout, ushort, S_IRUGO);
MODULE_PARM_DESC(tx_timeout, "The Tx timeout in ms");

#ifdef CONFIG_DEBUG_FS
static struct dentry *dpa_debugfs_root;
extern struct dentry *powerpc_debugfs_root;
#endif

/*
 * Max frame size, across all interfaces.
 * Configurable from Kconfig or bootargs, to avoid allocating
 * oversized (socket) buffers when not using jumbo frames.
 * Must be large enough to accomodate the network MTU, but small enough
 * to avoid wasting skb memory.
 *
 * Could be overridden once, at boot-time, via the
 * fsl_fman_phy_set_max_frm() callback.
 */
int fsl_fman_phy_maxfrm = CONFIG_DPA_MAX_FRM_SIZE;

/*
 * Extra headroom for Rx buffers.
 * FMan is instructed to allocate, on the Rx path, this amount of
 * space at the beginning of a data buffer, beside the DPA private
 * data area and the IC fields.
 * Does not impact Tx buffer layout.
 *
 * Configurable from Kconfig or bootargs. Zero by default, it's needed
 * on particular forwarding scenarios that add extra headers to the
 * forwarded frame to avoid unbounded increase of recycled buffers.
 */
int dpa_rx_extra_headroom = CONFIG_DPA_EXTRA_HEADROOM;

static const char rtx[][3] = {
	[RX] = "RX",
	[TX] = "TX"
};

#if defined(CONFIG_FSL_FMAN_TEST)
/* Defined as weak, to be implemented by fman pcd tester. */
int dpa_alloc_pcd_fqids(struct device *, uint32_t, uint8_t, uint32_t *)
__attribute__((weak));

int dpa_free_pcd_fqids(struct device *, uint32_t) __attribute__((weak));
#endif /* CONFIG_DPAA_FMAN_UNIT_TESTS */

/* BM */

#define DPAA_ETH_MAX_PAD (L1_CACHE_BYTES * 8)

static struct dpa_bp *dpa_bp_array[64];

static struct dpa_bp *default_pool;

/* A set of callbacks for hooking into the fastpath at different points. */
static struct dpaa_eth_hooks_s dpaa_eth_hooks;
/*
 * This function should only be called on the probe paths, since it makes no
 * effort to guarantee consistency of the destination hooks structure.
 */
void fsl_dpaa_eth_set_hooks(struct dpaa_eth_hooks_s *hooks)
{
	if (hooks)
		dpaa_eth_hooks = *hooks;
	else
		pr_err("NULL pointer to hooks!\n");
}
EXPORT_SYMBOL(fsl_dpaa_eth_set_hooks);


struct dpa_bp *dpa_bpid2pool(int bpid)
{
	return dpa_bp_array[bpid];
}

static void dpa_bp_depletion(struct bman_portal	*portal,
		struct bman_pool *pool, void *cb_ctx, int depleted)
{
	if (net_ratelimit())
		pr_err("Invalid Pool depleted notification!\n");
}

#ifndef CONFIG_DPAA_ETH_SG_SUPPORT
static void dpa_bp_add_8(struct dpa_bp *dpa_bp)
{
	struct bm_buffer bmb[8];
	struct sk_buff **skbh;
	dma_addr_t addr;
	int i;
	struct sk_buff *skb;
	int *count_ptr;

	count_ptr = per_cpu_ptr(dpa_bp->percpu_count, smp_processor_id());

	for (i = 0; i < 8; i++) {
		/*
		 * The buffers tend to be aligned all to the same cache
		 * index.  A standard dequeue operation pulls in 15 packets.
		 * This means that when it stashes, it evicts half of the
		 * packets it's stashing. In order to prevent that, we pad
		 * by a variable number of cache lines, to reduce collisions.
		 * We always pad by at least 1 cache line, because we want
		 * a little extra room at the beginning for IPSec and to
		 * accommodate NET_IP_ALIGN.
		 */
		int pad = (i + 1) * L1_CACHE_BYTES;

		skb = dev_alloc_skb(dpa_bp->size + pad);
		if (unlikely(!skb)) {
			printk(KERN_ERR "dev_alloc_skb() failed\n");
			bm_buffer_set64(&bmb[i], 0);
			break;
		}

		skbh = (struct sk_buff **)(skb->head + pad);
		*skbh = skb;

		/*
		 * Here we need to map only for device write (DMA_FROM_DEVICE),
		 * but on Tx recycling we may also get buffers in the pool that
		 * are mapped bidirectionally.
		 * Use DMA_BIDIRECTIONAL here as well to avoid any
		 * inconsistencies when unmapping.
		 */
		addr = dma_map_single(dpa_bp->dev, skb->head + pad,
				dpa_bp->size, DMA_BIDIRECTIONAL);
		if (unlikely(dma_mapping_error(dpa_bp->dev, addr))) {
			dpaa_eth_err(dpa_bp->dev, "DMA mapping failed");
			break;
		}

		bm_buffer_set64(&bmb[i], addr);
	}

	/* Avoid releasing a completely null buffer; bman_release() requires
	 * at least one buf. */
	if (likely(i)) {
		/*
		 * Release the buffers. In case bman is busy, keep trying
		 * until successful. bman_release() is guaranteed to succeed
		 * in a reasonable amount of time
		 */
		while (bman_release(dpa_bp->pool, bmb, i, 0))
			cpu_relax();

		*count_ptr += i;
	}
}

void dpa_make_private_pool(struct dpa_bp *dpa_bp)
{
	int i;

	dpa_bp->percpu_count = __alloc_percpu(sizeof(*dpa_bp->percpu_count),
			__alignof__(*dpa_bp->percpu_count));

	/* Give each cpu an allotment of "count" buffers */
	for_each_online_cpu(i) {
		int *thiscount;
		int *countptr;
		int j;
		thiscount = per_cpu_ptr(dpa_bp->percpu_count,
				smp_processor_id());
		countptr = per_cpu_ptr(dpa_bp->percpu_count, i);

		for (j = 0; j < dpa_bp->target_count; j += 8)
			dpa_bp_add_8(dpa_bp);

		/* Adjust the counts */
		*countptr = j;

		if (countptr != thiscount)
			*thiscount = *thiscount - j;
	}
}
#endif /* CONFIG_DPAA_ETH_SG_SUPPORT */

static void dpaa_eth_seed_pool(struct dpa_bp *bp)
{
	int count = bp->target_count;
	size_t addr = bp->paddr;

	while (count) {
		struct bm_buffer bufs[8];
		int num_bufs = 0;

		do {
			BUG_ON(addr > 0xffffffffffffull);
			bufs[num_bufs].bpid = bp->bpid;
			bm_buffer_set64(&bufs[num_bufs++], addr);
			addr += bp->size;

		} while (--count && (num_bufs < 8));

		while (bman_release(bp->pool, bufs, num_bufs, 0))
			cpu_relax();
	}
}

static int dpa_make_shared_port_pool(struct dpa_bp *bp)
{
	/*
	 * In MAC-less and Shared-MAC scenarios the physical
	 * address of the buffer pool in device tree is set
	 * to 0 to specify that another entity (USDPAA) will
	 * allocate and seed the buffers
	 */
	if (!bp->paddr)
		return 0;

	devm_request_mem_region(bp->dev, bp->paddr,
			bp->size * bp->config_count, KBUILD_MODNAME);
	bp->vaddr = devm_ioremap_prot(bp->dev, bp->paddr,
			bp->size * bp->config_count, 0);
	if (bp->vaddr == NULL) {
		cpu_pr_err("Could not map memory for pool %d\n", bp->bpid);
		return -EIO;
	}

	if (bp->seed_pool)
		dpaa_eth_seed_pool(bp);

	return 0;
}

static int __devinit __must_check __attribute__((nonnull))
dpa_bp_alloc(struct dpa_bp *dpa_bp)
{
	int err;
	struct bman_pool_params	 bp_params;
	struct platform_device *pdev;

	BUG_ON(dpa_bp->size == 0);
	BUG_ON(dpa_bp->config_count == 0);

	bp_params.flags = BMAN_POOL_FLAG_DEPLETION;
	bp_params.cb = dpa_bp_depletion;
	bp_params.cb_ctx = dpa_bp;

	/* We support two options.  Either a global shared pool, or
	 * a specified pool. If the pool is specified, we only
	 * create one per bpid */
	if (dpa_bp->kernel_pool && default_pool) {
		atomic_inc(&default_pool->refs);
		return 0;
	}

	if (dpa_bp_array[dpa_bp->bpid]) {
		atomic_inc(&dpa_bp_array[dpa_bp->bpid]->refs);
		return 0;
	}

	if (dpa_bp->bpid == 0)
		bp_params.flags |= BMAN_POOL_FLAG_DYNAMIC_BPID;
	else
		bp_params.bpid = dpa_bp->bpid;

	dpa_bp->pool = bman_new_pool(&bp_params);
	if (unlikely(dpa_bp->pool == NULL)) {
		cpu_pr_err("bman_new_pool() failed\n");
		return -ENODEV;
	}

	dpa_bp->bpid = bman_get_params(dpa_bp->pool)->bpid;

	pdev = platform_device_register_simple("dpaa_eth_bpool",
			dpa_bp->bpid, NULL, 0);
	if (IS_ERR(pdev)) {
		err = PTR_ERR(pdev);
		goto pdev_register_failed;
	}

	err = dma_set_mask(&pdev->dev, DMA_BIT_MASK(40));
	if (err)
		goto pdev_mask_failed;

	dpa_bp->dev = &pdev->dev;

	if (dpa_bp->kernel_pool) {
		dpa_make_private_pool(dpa_bp);
		if (!default_pool)
			default_pool = dpa_bp;
	} else {
		err = dpa_make_shared_port_pool(dpa_bp);
		if (err)
			goto make_shared_pool_failed;
	}

	dpa_bp_array[dpa_bp->bpid] = dpa_bp;

	atomic_set(&dpa_bp->refs, 1);

	return 0;

make_shared_pool_failed:
pdev_mask_failed:
	platform_device_unregister(pdev);
pdev_register_failed:
	bman_free_pool(dpa_bp->pool);

	return err;
}

#ifndef CONFIG_DPAA_ETH_SG_SUPPORT
static inline void _dpa_bp_free_buf(void *addr)
{
	struct sk_buff **skbh = addr;
	struct sk_buff *skb;

	skb = *skbh;
	dev_kfree_skb_any(skb);
}
#else
static inline void _dpa_bp_free_buf(void *addr)
{
	free_page((unsigned long)addr);
}
#endif

static void __cold __attribute__((nonnull))
_dpa_bp_free(struct dpa_bp *dpa_bp)
{
	struct dpa_bp *bp = dpa_bpid2pool(dpa_bp->bpid);

	if (!atomic_dec_and_test(&bp->refs))
		return;

	if (bp->kernel_pool) {
		int num;

		do {
			struct bm_buffer bmb[8];
			int i;

			num = bman_acquire(bp->pool, bmb, 8, 0);

			for (i = 0; i < num; i++) {
				dma_addr_t addr = bm_buf_addr(&bmb[i]);

				dma_unmap_single(bp->dev, addr, bp->size,
						DMA_BIDIRECTIONAL);

				_dpa_bp_free_buf(phys_to_virt(addr));
			}
		} while (num == 8);
	}

	dpa_bp_array[bp->bpid] = 0;
	bman_free_pool(bp->pool);
}

static void __cold __attribute__((nonnull))
dpa_bp_free(struct dpa_priv_s *priv, struct dpa_bp *dpa_bp)
{
	int i;

	for (i = 0; i < priv->bp_count; i++)
		_dpa_bp_free(&priv->dpa_bp[i]);
}

/* QM */

static int __devinit __must_check __attribute__((nonnull))
_dpa_fq_alloc(struct list_head *list, struct dpa_fq *dpa_fq)
{
	int			 _errno;
	const struct dpa_priv_s	*priv;
	struct device		*dev;
	struct qman_fq		*fq;
	struct qm_mcc_initfq	 initfq;

	priv = netdev_priv(dpa_fq->net_dev);
	dev = dpa_fq->net_dev->dev.parent;

	if (dpa_fq->fqid == 0)
		dpa_fq->flags |= QMAN_FQ_FLAG_DYNAMIC_FQID;

	dpa_fq->init = !(dpa_fq->flags & QMAN_FQ_FLAG_NO_MODIFY);

	_errno = qman_create_fq(dpa_fq->fqid, dpa_fq->flags, &dpa_fq->fq_base);
	if (_errno) {
		dpaa_eth_err(dev, "qman_create_fq() failed\n");
		return _errno;
	}
	fq = &dpa_fq->fq_base;

	if (dpa_fq->init) {
		initfq.we_mask = QM_INITFQ_WE_DESTWQ;
		initfq.fqd.dest.channel	= dpa_fq->channel;
		initfq.fqd.dest.wq = dpa_fq->wq;
		initfq.we_mask |= QM_INITFQ_WE_TDTHRESH | QM_INITFQ_WE_FQCTRL;
		qm_fqd_taildrop_set(&initfq.fqd.td, DPA_TAILDROP_THRESHOLD, 1);
		initfq.fqd.fq_ctrl = QM_FQCTRL_TDE | QM_FQCTRL_PREFERINCACHE;
		if (dpa_fq->flags & QMAN_FQ_FLAG_NO_ENQUEUE) {
			initfq.we_mask |= QM_INITFQ_WE_CONTEXTA;
			initfq.fqd.fq_ctrl |=
				QM_FQCTRL_CTXASTASHING | QM_FQCTRL_AVOIDBLOCK;
			initfq.fqd.context_a.stashing.exclusive =
				QM_STASHING_EXCL_DATA | QM_STASHING_EXCL_CTX |
				QM_STASHING_EXCL_ANNOTATION;
			initfq.fqd.context_a.stashing.data_cl = 2;
			initfq.fqd.context_a.stashing.annotation_cl = 1;
			initfq.fqd.context_a.stashing.context_cl =
				DIV_ROUND_UP(sizeof(struct qman_fq), 64);
		};

		_errno = qman_init_fq(fq, QMAN_INITFQ_FLAG_SCHED, &initfq);
		if (_errno < 0) {
			dpaa_eth_err(dev, "qman_init_fq(%u) = %d\n",
					qman_fq_fqid(fq), _errno);
			qman_destroy_fq(fq, 0);
			return _errno;
		}
	}

	dpa_fq->fqid = qman_fq_fqid(fq);
	list_add_tail(&dpa_fq->list, list);

	return 0;
}

static int __cold __attribute__((nonnull))
_dpa_fq_free(struct device *dev, struct qman_fq *fq)
{
	int			 _errno, __errno;
	struct dpa_fq		*dpa_fq;
	const struct dpa_priv_s	*priv;

	_errno = 0;

	dpa_fq = container_of(fq, struct dpa_fq, fq_base);
	priv = netdev_priv(dpa_fq->net_dev);

	if (dpa_fq->init) {
		_errno = qman_retire_fq(fq, NULL);
		if (unlikely(_errno < 0) && netif_msg_drv(priv))
			dpaa_eth_err(dev, "qman_retire_fq(%u) = %d\n",
					qman_fq_fqid(fq), _errno);

		__errno = qman_oos_fq(fq);
		if (unlikely(__errno < 0) && netif_msg_drv(priv)) {
			dpaa_eth_err(dev, "qman_oos_fq(%u) = %d\n",
					qman_fq_fqid(fq), __errno);
			if (_errno >= 0)
				_errno = __errno;
		}
	}

	qman_destroy_fq(fq, 0);
	list_del(&dpa_fq->list);

	return _errno;
}

static int __cold __attribute__((nonnull))
dpa_fq_free(struct device *dev, struct list_head *list)
{
	int		 _errno, __errno;
	struct dpa_fq	*dpa_fq, *tmp;

	_errno = 0;
	list_for_each_entry_safe(dpa_fq, tmp, list, list) {
		__errno = _dpa_fq_free(dev, (struct qman_fq *)dpa_fq);
		if (unlikely(__errno < 0) && _errno >= 0)
			_errno = __errno;
	}

	return _errno;
}


void __attribute__((nonnull))
dpa_fd_release(const struct net_device *net_dev, const struct qm_fd *fd)
{
	int				 i, j;
	const struct dpa_priv_s		*priv;
	const struct qm_sg_entry	*sgt;
	struct dpa_bp			*_dpa_bp, *dpa_bp;
	struct bm_buffer		 _bmb, bmb[8];

	priv = netdev_priv(net_dev);

	_bmb.hi	= fd->addr_hi;
	_bmb.lo	= fd->addr_lo;

	_dpa_bp = dpa_bpid2pool(fd->bpid);
	BUG_ON(IS_ERR(_dpa_bp));

	if (fd->format == qm_fd_sg) {
		sgt = (phys_to_virt(bm_buf_addr(&_bmb)) + dpa_fd_offset(fd));

		i = 0;
		do {
			dpa_bp = dpa_bpid2pool(sgt[i].bpid);
			BUG_ON(IS_ERR(dpa_bp));

			j = 0;
			do {
				BUG_ON(sgt[i].extension);

				bmb[j].hi	= sgt[i].addr_hi;
				bmb[j].lo	= sgt[i].addr_lo;

				j++; i++;
			} while (j < ARRAY_SIZE(bmb) &&
					!sgt[i-1].final &&
					sgt[i-1].bpid == sgt[i].bpid);

			while (bman_release(dpa_bp->pool, bmb, j, 0))
				cpu_relax();
		} while (!sgt[i-1].final);
	}

	while (bman_release(_dpa_bp->pool, &_bmb, 1, 0))
		cpu_relax();
}

#ifndef CONFIG_DPAA_ETH_SG_SUPPORT
/*
 * Cleanup function for outgoing frame descriptors that were built on Tx path,
 * either contiguous frames or scatter/gather ones with a single data buffer.
 * Skb freeing is not handled here.
 *
 * This function may be called on error paths in the Tx function, so guard
 * against cases when not all fd relevant fields were filled in.
 *
 * Return the skb backpointer, since for S/G frames the buffer containing it
 * gets freed here.
 */
struct sk_buff *_dpa_cleanup_tx_fd(const struct dpa_priv_s *priv,
			       const struct qm_fd *fd)
{
	dma_addr_t addr = qm_fd_addr(fd);
	dma_addr_t sg_addr;
	struct dpa_bp *bp = priv->dpa_bp;
	struct sk_buff **skbh;
	struct sk_buff *skb = NULL;

	BUG_ON(!fd);

	if (unlikely(!addr))
		return skb;

	skbh = (struct sk_buff **)phys_to_virt(addr);

	if (fd->format == qm_fd_contig) {
		/* For contiguous frames, just unmap data buffer;
		 * mapping direction depends on whether the frame was
		 * meant to be recycled or not */
		if (fd->cmd & FM_FD_CMD_FCO)
			dma_unmap_single(bp->dev, addr, bp->size,
					 DMA_BIDIRECTIONAL);
		else
			dma_unmap_single(bp->dev, addr, bp->size,
					 DMA_TO_DEVICE);
		/* Retrieve the skb backpointer */
		skb = *skbh;
	} else {
		/* For s/g, we need to unmap both the SGT buffer and the
		 * data buffer, and also free the SGT buffer */
		struct qm_sg_entry *sg_entry;
		void *vaddr = phys_to_virt(addr);

		/* Unmap first buffer (contains S/G table) */
		dma_unmap_single(bp->dev, addr, SGT_BUFFER_SIZE,
				 DMA_TO_DEVICE);

		/* Unmap data buffer */
		sg_entry = (struct qm_sg_entry *)(vaddr + fd->offset);
		sg_addr = qm_sg_addr(sg_entry);
		if (likely(sg_addr))
			dma_unmap_single(bp->dev, sg_addr, bp->size,
					 DMA_TO_DEVICE);
		/* Retrieve the skb backpointer */
		skb = *skbh;

		/* Free first buffer (which was allocated on Tx) */
		kfree(vaddr);
	}

	return skb;
}
#endif /* CONFIG_DPAA_ETH_SG_SUPPORT */

/* net_device */

static struct net_device_stats * __cold
dpa_get_stats(struct net_device *net_dev)
{
	struct dpa_priv_s *priv = netdev_priv(net_dev);
	unsigned long *netstats;
	unsigned long *cpustats;
	int i, j;
	struct dpa_percpu_priv_s	*percpu_priv;
	int numstats = sizeof(net_dev->stats) / sizeof(unsigned long);

	netstats = (unsigned long *)&net_dev->stats;

	memset(netstats, 0, sizeof(net_dev->stats));

	for_each_online_cpu(i) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, i);

		cpustats = (unsigned long *)&percpu_priv->stats;

		for (j = 0; j < numstats; j++)
			netstats[j] += cpustats[j];
	}

	return &net_dev->stats;
}

static int dpa_change_mtu(struct net_device *net_dev, int new_mtu)
{
	const struct dpa_priv_s *priv;
	const int max_mtu = fsl_fman_phy_maxfrm - (VLAN_ETH_HLEN + ETH_FCS_LEN);
	const int min_mtu = 64;

	priv = netdev_priv(net_dev);

	/* Make sure we don't exceed the Ethernet controller's MAXFRM */
	if (new_mtu < min_mtu || new_mtu > max_mtu) {
		cpu_netdev_err(net_dev, "Invalid L3 mtu %d "
				"(must be between %d and %d).\n",
				new_mtu, min_mtu, max_mtu);
		return -EINVAL;
	}
	net_dev->mtu = new_mtu;

	return 0;
}

static int dpa_set_mac_address(struct net_device *net_dev, void *addr)
{
	const struct dpa_priv_s	*priv;
	int			 _errno;

	priv = netdev_priv(net_dev);

	_errno = eth_mac_addr(net_dev, addr);
	if (_errno < 0) {
		if (netif_msg_drv(priv))
			cpu_netdev_err(net_dev,
				       "eth_mac_addr() = %d\n",
				       _errno);
		return _errno;
	}

	if (!priv->mac_dev)
		/* MAC-less interface, so nothing more to do here */
		return 0;

	_errno = priv->mac_dev->change_addr(priv->mac_dev, net_dev->dev_addr);
	if (_errno < 0) {
		if (netif_msg_drv(priv))
			cpu_netdev_err(net_dev,
				       "mac_dev->change_addr() = %d\n",
				       _errno);
		return _errno;
	}

	return 0;
}

static void dpa_set_rx_mode(struct net_device *net_dev)
{
	int			 _errno;
	const struct dpa_priv_s	*priv;

	priv = netdev_priv(net_dev);

	if (!priv->mac_dev)
		return;

	if (!!(net_dev->flags & IFF_PROMISC) != priv->mac_dev->promisc) {
		_errno = priv->mac_dev->change_promisc(priv->mac_dev);
		if (unlikely(_errno < 0) && netif_msg_drv(priv))
			cpu_netdev_err(net_dev,
					   "mac_dev->change_promisc() = %d\n",
					   _errno);
	}

	_errno = priv->mac_dev->set_multi(net_dev);
	if (unlikely(_errno < 0) && netif_msg_drv(priv))
		cpu_netdev_err(net_dev, "mac_dev->set_multi() = %d\n", _errno);
}

#ifdef CONFIG_FSL_DPA_1588
static int dpa_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct dpa_priv_s *priv = netdev_priv(dev);
	int ret = 0;

	if (!netif_running(dev))
		return -EINVAL;

	if ((cmd >= PTP_ENBL_TXTS_IOCTL) && (cmd <= PTP_CLEANUP_TS)) {
		if (priv->tsu && priv->tsu->valid)
			ret = dpa_ioctl_1588(dev, rq, cmd);
		else
			ret = -ENODEV;
	}

	return ret;
}
#endif

#ifndef CONFIG_DPAA_ETH_SG_SUPPORT
/*
 * When we put the buffer into the pool, we purposefully added
 * some padding to the address so that the buffers wouldn't all
 * be page-aligned. But the skb has been reset to a default state,
 * so it is pointing up to DPAA_ETH_MAX_PAD - L1_CACHE_BYTES bytes
 * before the actual data. We subtract skb->head from the fd addr,
 * and then mask off the translated part to get the actual distance.
 */
static int dpa_process_one(struct dpa_percpu_priv_s *percpu_priv,
		struct sk_buff *skb, struct dpa_bp *bp, const struct qm_fd *fd)
{
	dma_addr_t addr = qm_fd_addr(fd);
	u32 addrlo = lower_32_bits(addr);
	u32 skblo = lower_32_bits((unsigned long)skb->head);
	u32 pad = (addrlo - skblo) & (PAGE_SIZE - 1);
	unsigned int data_start;

	(*percpu_priv->dpa_bp_count)--;

	/*
	 * The skb is currently pointed at head + headroom. The packet
	 * starts at skb->head + pad + fd offset.
	 */
	data_start = pad + dpa_fd_offset(fd) - skb_headroom(skb);
	skb_put(skb, dpa_fd_length(fd) + data_start);
	skb_pull(skb, data_start);

	return 0;
}
#endif

/*
 * Checks whether the checksum field in Parse Results array is valid
 * (equals 0xFFFF) and increments the .cse counter otherwise
 */
static inline void
dpa_csum_validation(const struct dpa_priv_s	*priv,
		struct dpa_percpu_priv_s *percpu_priv,
		const struct qm_fd *fd)
{
	dma_addr_t addr = qm_fd_addr(fd);
	struct dpa_bp *dpa_bp = priv->dpa_bp;
	void *frm = phys_to_virt(addr);
	t_FmPrsResult *parse_result;

	if (unlikely(!frm))
		return;

	dma_unmap_single(dpa_bp->dev, addr, dpa_bp->size, DMA_BIDIRECTIONAL);

	parse_result = (t_FmPrsResult *)(frm + DPA_RX_PRIV_DATA_SIZE);

	if (parse_result->cksum != DPA_CSUM_VALID)
		percpu_priv->rx_errors.cse++;
}

static void _dpa_rx_error(struct net_device *net_dev,
		const struct dpa_priv_s	*priv,
		struct dpa_percpu_priv_s *percpu_priv,
		const struct qm_fd *fd,
		u32 fqid)
{
	if (dpaa_eth_hooks.rx_error &&
		dpaa_eth_hooks.rx_error(net_dev, fd, fqid) == DPAA_ETH_STOLEN)
		/* it's up to the hook to perform resource cleanup */
		return;

	percpu_priv->stats.rx_errors++;

	if (fd->status & FM_PORT_FRM_ERR_DMA)
		percpu_priv->rx_errors.dme++;
	if (fd->status & FM_PORT_FRM_ERR_PHYSICAL)
		percpu_priv->rx_errors.fpe++;
	if (fd->status & FM_PORT_FRM_ERR_SIZE)
		percpu_priv->rx_errors.fse++;
	if (fd->status & FM_PORT_FRM_ERR_PRS_HDR_ERR)
		percpu_priv->rx_errors.phe++;
	if (fd->status & FM_FD_STAT_L4CV)
		dpa_csum_validation(priv, percpu_priv, fd);

	dpa_fd_release(net_dev, fd);
}

static void _dpa_tx_error(struct net_device		*net_dev,
			  const struct dpa_priv_s	*priv,
			  struct dpa_percpu_priv_s	*percpu_priv,
			  const struct qm_fd		*fd,
			  u32				 fqid)
{
	struct sk_buff *skb;

	if (netif_msg_hw(priv) && net_ratelimit())
		cpu_netdev_warn(net_dev, "FD status = 0x%08x\n",
				fd->status & FM_FD_STAT_ERRORS);

	if (dpaa_eth_hooks.tx_error &&
		dpaa_eth_hooks.tx_error(net_dev, fd, fqid) == DPAA_ETH_STOLEN)
		/* now the hook must ensure proper cleanup */
		return;

	percpu_priv->stats.tx_errors++;

	skb = _dpa_cleanup_tx_fd(priv, fd);
	dev_kfree_skb(skb);
}

#ifndef CONFIG_DPAA_ETH_SG_SUPPORT
void __hot _dpa_rx(struct net_device *net_dev,
		const struct dpa_priv_s *priv,
		struct dpa_percpu_priv_s *percpu_priv,
		const struct qm_fd *fd,
		u32 fqid)
{
	struct dpa_bp *dpa_bp;
	struct sk_buff *skb;
	struct sk_buff **skbh;
	dma_addr_t addr = qm_fd_addr(fd);
	u32 fd_status = fd->status;

	skbh = (struct sk_buff **)phys_to_virt(addr);

	if (unlikely(fd_status & FM_FD_STAT_ERRORS) != 0) {
		if (netif_msg_hw(priv) && net_ratelimit())
			cpu_netdev_warn(net_dev, "FD status = 0x%08x\n",
					fd->status & FM_FD_STAT_ERRORS);

		percpu_priv->stats.rx_errors++;

		goto _return_dpa_fd_release;
	}

	if (unlikely(fd->format != qm_fd_contig)) {
		percpu_priv->stats.rx_dropped++;
		if (netif_msg_rx_status(priv) && net_ratelimit())
			cpu_netdev_warn(net_dev, "Dropping a SG frame\n");
		goto _return_dpa_fd_release;
	}

	dpa_bp = dpa_bpid2pool(fd->bpid);

	dma_unmap_single(dpa_bp->dev, addr, dpa_bp->size, DMA_BIDIRECTIONAL);

	skb = *skbh;
	prefetch(skb);

	/* Fill the SKB */
	dpa_process_one(percpu_priv, skb, dpa_bp, fd);

	prefetch(skb_shinfo(skb));

#ifdef CONFIG_FSL_DPA_1588
	if (priv->tsu && priv->tsu->valid && priv->tsu->hwts_rx_en_ioctl)
		dpa_ptp_store_rxstamp(net_dev, skb, fd);
#endif

	skb->protocol = eth_type_trans(skb, net_dev);

	if (unlikely(dpa_check_rx_mtu(skb, net_dev->mtu))) {
		percpu_priv->stats.rx_dropped++;
		goto drop_large_frame;
	}

	/* Check if the FMan Parser has already validated the L4 csum. */
	if (fd_status & FM_FD_STAT_L4CV) {
		/* If we're here, the csum must be valid (if it hadn't,
		 * the frame would have been received on the Error FQ,
		 * respectively on the _dpa_rx_error() path). */
		skb->ip_summed = CHECKSUM_UNNECESSARY;
	} else
		skb->ip_summed = CHECKSUM_NONE;

	/* Execute the Rx processing hook, if it exists. */
	if (dpaa_eth_hooks.rx_default && dpaa_eth_hooks.rx_default(skb,
		net_dev, fqid) == DPAA_ETH_STOLEN)
		/* won't count the rx bytes in */
		goto skb_stolen;

	if (unlikely(netif_receive_skb(skb) == NET_RX_DROP))
		percpu_priv->stats.rx_dropped++;
	else {
		percpu_priv->stats.rx_packets++;
		percpu_priv->stats.rx_bytes += skb->len;
	}

skb_stolen:
	net_dev->last_rx = jiffies;

	return;

drop_large_frame:
	dev_kfree_skb(skb);
	return;
_return_dpa_fd_release:
	dpa_fd_release(net_dev, fd);
}
#endif /* CONFIG_DPAA_ETH_SG_SUPPORT */

static void dpaa_eth_napi_disable(struct dpa_priv_s *priv)
{
	struct dpa_percpu_priv_s *percpu_priv;
	int i;

	if (priv->shared)
		return;

	for_each_online_cpu(i) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, i);
		napi_disable(&percpu_priv->napi);
	}
}

static void dpaa_eth_napi_enable(struct dpa_priv_s *priv)
{
	struct dpa_percpu_priv_s *percpu_priv;
	int i;

	if (priv->shared)
		return;

	for_each_online_cpu(i) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, i);
		napi_enable(&percpu_priv->napi);
	}
}

static int dpaa_eth_poll(struct napi_struct *napi, int budget)
{
	struct dpa_percpu_priv_s *percpu_priv;
	int cleaned = qman_poll_dqrr(budget);
	int count;

	percpu_priv = container_of(napi, struct dpa_percpu_priv_s, napi);

	count = *percpu_priv->dpa_bp_count;

#ifndef CONFIG_DPAA_ETH_SG_SUPPORT
	if (count < DEFAULT_COUNT / 4) {
		int i;

		for (i = count; i < DEFAULT_COUNT; i += 8)
			dpa_bp_add_8(percpu_priv->dpa_bp);
	}
#else
	if (count < DEFAULT_COUNT / 4) {
		int i;

		/* Add pages to the buffer pool */
		for (i = count; i < DEFAULT_COUNT; i += 8)
			dpa_bp_add_8_pages(percpu_priv->dpa_bp,
					   smp_processor_id());
	}

	/* Add skbs to the percpu skb list, reuse var count */
	count = percpu_priv->skb_count;

	if (count < DEFAULT_SKB_COUNT / 4)
		dpa_list_add_skbs(percpu_priv,
				  DEFAULT_SKB_COUNT - count);
#endif


	if (cleaned < budget) {
		int tmp;
		napi_complete(napi);
		tmp = qman_irqsource_add(QM_PIRQ_DQRI);
		BUG_ON(tmp);
	}

	return cleaned;
}

static void __hot _dpa_tx_conf(struct net_device	*net_dev,
			  const struct dpa_priv_s	*priv,
			  struct dpa_percpu_priv_s	*percpu_priv,
			  const struct qm_fd		*fd,
			  u32				 fqid)
{
	struct sk_buff	*skb;

	if (unlikely(fd->status & FM_FD_STAT_ERRORS) != 0) {
		if (netif_msg_hw(priv) && net_ratelimit())
			cpu_netdev_warn(net_dev, "FD status = 0x%08x\n",
					fd->status & FM_FD_STAT_ERRORS);

		percpu_priv->stats.tx_errors++;
	}

	if (dpaa_eth_hooks.tx_confirm && dpaa_eth_hooks.tx_confirm(net_dev,
		fd, fqid) == DPAA_ETH_STOLEN)
		/* it's the hook that must now perform cleanup */
		return;

	/* This might not perfectly reflect the reality, if the core dequeueing
	 * the Tx confirmation is different from the one that did the enqueue,
	 * but at least it'll show up in the total count. */
	percpu_priv->tx_confirm++;

	skb = _dpa_cleanup_tx_fd(priv, fd);

#ifdef CONFIG_FSL_DPA_1588
	if (priv->tsu && priv->tsu->valid && priv->tsu->hwts_tx_en_ioctl)
		dpa_ptp_store_txstamp(net_dev, skb, fd);
#endif
	dev_kfree_skb(skb);
}

static struct dpa_bp *dpa_size2pool(struct dpa_priv_s *priv, size_t size)
{
	int i;

	for (i = 0; i < priv->bp_count; i++)
		if (DPA_BP_SIZE(size) <= priv->dpa_bp[i].size)
			return dpa_bpid2pool(priv->dpa_bp[i].bpid);
	return ERR_PTR(-ENODEV);
}

static inline void * __must_check __attribute__((nonnull))
dpa_phys2virt(const struct dpa_bp *dpa_bp, dma_addr_t addr)
{
	return dpa_bp->vaddr + (addr - dpa_bp->paddr);
}

/**
 * Turn on HW checksum computation for this outgoing frame.
 * If the current protocol is not something we support in this regard
 * (or if the stack has already computed the SW checksum), we do nothing.
 *
 * Returns 0 if all goes well (or HW csum doesn't apply), and a negative value
 * otherwise.
 *
 * Note that this function may modify the fd->cmd field and the skb data buffer
 * (the Parse Results area).
 */
int dpa_enable_tx_csum(struct dpa_priv_s *priv,
	struct sk_buff *skb, struct qm_fd *fd, char *parse_results)
{
	t_FmPrsResult *parse_result;
	struct iphdr *iph;
	struct ipv6hdr *ipv6h = NULL;
	int l4_proto;
	int ethertype = ntohs(skb->protocol);
	int retval = 0;

	if (!priv->mac_dev || skb->ip_summed != CHECKSUM_PARTIAL)
		return 0;

	/* Note: L3 csum seems to be already computed in sw, but we can't choose
	 * L4 alone from the FM configuration anyway. */

	/* Fill in some fields of the Parse Results array, so the FMan
	 * can find them as if they came from the FMan Parser. */
	parse_result = (t_FmPrsResult *)parse_results;

	/* If we're dealing with VLAN, get the real Ethernet type */
	if (ethertype == ETH_P_8021Q) {
		/* We can't always assume the MAC header is set correctly
		 * by the stack, so reset to beginning of skb->data */
		skb_reset_mac_header(skb);
		ethertype = ntohs(vlan_eth_hdr(skb)->h_vlan_encapsulated_proto);
	}

	/* Fill in the relevant L3 parse result fields
	 * and read the L4 protocol type */
	switch (ethertype) {
	case ETH_P_IP:
		parse_result->l3r = FM_L3_PARSE_RESULT_IPV4;
		iph = ip_hdr(skb);
		BUG_ON(iph == NULL);
		l4_proto = ntohs(iph->protocol);
		break;
	case ETH_P_IPV6:
		parse_result->l3r = FM_L3_PARSE_RESULT_IPV6;
		ipv6h = ipv6_hdr(skb);
		BUG_ON(ipv6h == NULL);
		l4_proto = ntohs(ipv6h->nexthdr);
		break;
	default:
		/* We shouldn't even be here */
		if (netif_msg_tx_err(priv) && net_ratelimit())
			cpu_netdev_alert(priv->net_dev, "Can't compute HW csum "
				"for L3 proto 0x%x\n", ntohs(skb->protocol));
		retval = -EIO;
		goto return_error;
	}

	/* Fill in the relevant L4 parse result fields */
	switch (l4_proto) {
	case IPPROTO_UDP:
		parse_result->l4r = FM_L4_PARSE_RESULT_UDP;
		break;
	case IPPROTO_TCP:
		parse_result->l4r = FM_L4_PARSE_RESULT_TCP;
		break;
	default:
		/* This can as well be a BUG() */
		if (netif_msg_tx_err(priv) && net_ratelimit())
			cpu_netdev_alert(priv->net_dev, "Can't compute HW csum "
				"for L4 proto 0x%x\n", l4_proto);
		retval = -EIO;
		goto return_error;
	}

	/* At index 0 is IPOffset_1 as defined in the Parse Results */
	parse_result->ip_off[0] = skb_network_offset(skb);
	parse_result->l4_off = skb_transport_offset(skb);

	/* Enable L3 (and L4, if TCP or UDP) HW checksum. */
	fd->cmd |= FM_FD_CMD_RPD | FM_FD_CMD_DTC;

	/*
	 * On P1023 and similar platforms fd->cmd interpretation could
	 * be disabled by setting CONTEXT_A bit ICMD; currently this bit
	 * is not set so we do not need to check; in the future, if/when
	 * using context_a we need to check this bit
	 */

return_error:
	return retval;
}

static int __hot dpa_shared_tx(struct sk_buff *skb, struct net_device *net_dev)
{
	struct dpa_bp *dpa_bp;
	struct bm_buffer bmb;
	struct dpa_percpu_priv_s *percpu_priv;
	struct dpa_priv_s *priv;
	struct qm_fd fd;
	int queue_mapping;
	int err;
	void *dpa_bp_vaddr, *page_vaddr;
	dma_addr_t addr_offset;
	struct page *page;
	unsigned int offset;
	size_t remain, size, total_size;

	priv = netdev_priv(net_dev);
	percpu_priv = per_cpu_ptr(priv->percpu_priv, smp_processor_id());

	memset(&fd, 0, sizeof(fd));
	fd.format = qm_fd_contig;

	queue_mapping = skb_get_queue_mapping(skb);

	dpa_bp = dpa_size2pool(priv, skb_headlen(skb));
	if (unlikely(IS_ERR(dpa_bp))) {
		percpu_priv->stats.tx_errors++;
		err = PTR_ERR(dpa_bp);
		goto bpools_too_small_error;
	}

	err = bman_acquire(dpa_bp->pool, &bmb, 1, 0);
	if (unlikely(err <= 0)) {
		percpu_priv->stats.tx_errors++;
		if (err == 0)
			err = -ENOMEM;
		goto buf_acquire_failed;
	}
	fd.bpid = dpa_bp->bpid;

	fd.length20 = skb_headlen(skb);
	fd.cmd = FM_FD_CMD_FCO;
	fd.addr_hi = bmb.hi;
	fd.addr_lo = bmb.lo;
	fd.offset = DPA_BP_HEAD;

	/*
	 * The virtual address of the buffer pool is expected to be NULL
	 * in scenarios like MAC-less or Shared-MAC between Linux and
	 * USDPAA. In this case the buffers are dynamically mapped/unmapped.
	 */
	if (dpa_bp->vaddr) {
		dpa_bp_vaddr = dpa_phys2virt(dpa_bp, bm_buf_addr(&bmb));

		/* Copy the packet payload */
		skb_copy_from_linear_data(skb,
		                          dpa_bp_vaddr + dpa_fd_offset(&fd),
		                          dpa_fd_length(&fd));

		/* Enable L3/L4 hardware checksum computation, if applicable */
		err = dpa_enable_tx_csum(priv, skb, &fd,
					 dpa_bp_vaddr + DPA_TX_PRIV_DATA_SIZE);

		goto static_map;
	}

	remain = dpa_fd_offset(&fd) + dpa_fd_length(&fd);
	total_size = remain;
	addr_offset = bm_buf_addr(&bmb);

	while (remain > 0) {
		page = pfn_to_page(addr_offset >> PAGE_SHIFT);
		offset = offset_in_page(addr_offset);
		size = remain;

		if (remain + offset > PAGE_SIZE) {
			size = PAGE_SIZE - offset;
			addr_offset += size;
		}

		page_vaddr = kmap_atomic(page);

		if (!offset && (total_size != remain))
			memcpy(page_vaddr,
				   skb->data + total_size - remain -
				   dpa_fd_offset(&fd),
			       size);
		else {
			memcpy(page_vaddr + dpa_fd_offset(&fd) + offset,
			       skb->data,
			       size - dpa_fd_offset(&fd));

			err = dpa_enable_tx_csum(priv, skb, &fd,
			                         page_vaddr + offset +
						 DPA_TX_PRIV_DATA_SIZE);
		}

		kunmap_atomic(page_vaddr);

		remain -= size;
	}

static_map:
	if (unlikely(err < 0)) {
		if (netif_msg_tx_err(priv) && net_ratelimit())
			cpu_netdev_err(net_dev, "Tx HW csum error: %d\n", err);
		percpu_priv->stats.tx_errors++;
		goto l3_l4_csum_failed;
	}

	err = dpa_xmit(priv, percpu_priv, queue_mapping, &fd);

l3_l4_csum_failed:
bpools_too_small_error:
buf_acquire_failed:
	/* We're done with the skb */
	dev_kfree_skb(skb);

	return NETDEV_TX_OK;
}

#ifndef CONFIG_DPAA_ETH_SG_SUPPORT
static int skb_to_sg_fd(struct dpa_priv_s *priv,
		struct sk_buff *skb, struct qm_fd *fd)
{
	struct dpa_bp *dpa_bp = priv->dpa_bp;
	void *vaddr;
	dma_addr_t paddr;
	struct sk_buff **skbh;
	struct qm_sg_entry *sg_entry;
	struct net_device *net_dev = priv->net_dev;
	int err;

	/* Allocate the first buffer in the FD (used for storing S/G table) */
	vaddr = kmalloc(SGT_BUFFER_SIZE, GFP_ATOMIC);
	if (unlikely(vaddr == NULL)) {
		if (netif_msg_tx_err(priv) && net_ratelimit())
			cpu_netdev_err(net_dev, "Memory allocation failed\n");
		return -ENOMEM;
	}
	/* Store skb backpointer at the beginning of the buffer */
	skbh = (struct sk_buff **)vaddr;
	*skbh = skb;

	/* Fill in FD */
	fd->format = qm_fd_sg;
	fd->offset = DPA_BP_HEAD;
	fd->length20 = skb->len;

	/* Enable hardware checksum computation */
	err = dpa_enable_tx_csum(priv, skb, fd,
		(char *)vaddr + DPA_TX_PRIV_DATA_SIZE);
	if (unlikely(err < 0)) {
		if (netif_msg_tx_err(priv) && net_ratelimit())
			cpu_netdev_err(net_dev, "HW csum error: %d\n", err);
		kfree(vaddr);
		return err;
	}

	/* Map the buffer and store its address in the FD */
	paddr = dma_map_single(dpa_bp->dev, vaddr, SGT_BUFFER_SIZE,
			       DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(dpa_bp->dev, paddr))) {
		if (netif_msg_tx_err(priv) && net_ratelimit())
			cpu_netdev_err(net_dev, "DMA mapping failed\n");
		kfree(vaddr);
		return -EINVAL;
	}

	fd->addr_hi = upper_32_bits(paddr);
	fd->addr_lo = lower_32_bits(paddr);

	/* Fill in S/G entry */
	sg_entry = (struct qm_sg_entry *)(vaddr + fd->offset);

	sg_entry->extension = 0;
	sg_entry->final = 1;
	sg_entry->length = skb->len;
	/*
	 * Put the same offset in the data buffer as in the SGT (first) buffer.
	 * This is the format for S/G frames generated by FMan; the manual is
	 * not clear if same is required of Tx S/G frames, but since we know
	 * for sure we have at least DPA_BP_HEAD bytes of skb headroom, lets not
	 * take any chances.
	 */
	sg_entry->offset = DPA_BP_HEAD;

	paddr = dma_map_single(dpa_bp->dev, skb->data - sg_entry->offset,
			       dpa_bp->size, DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(dpa_bp->dev, paddr))) {
		if (netif_msg_tx_err(priv) && net_ratelimit())
			cpu_netdev_err(net_dev, "DMA mapping failed\n");
		return -EINVAL;
	}
	sg_entry->addr_hi = upper_32_bits(paddr);
	sg_entry->addr_lo = lower_32_bits(paddr);

	return 0;
}

static int skb_to_contig_fd(struct dpa_priv_s *priv,
		struct dpa_percpu_priv_s *percpu_priv,
		struct sk_buff *skb, struct qm_fd *fd)
{
	struct sk_buff **skbh;
	dma_addr_t addr;
	struct dpa_bp *dpa_bp = priv->dpa_bp;
	struct net_device *net_dev = priv->net_dev;
	enum dma_data_direction dma_dir = DMA_TO_DEVICE;
	bool can_recycle = false;
	int offset, extra_offset;
	int err;

	/*
	 * We are guaranteed that we have at least DPA_BP_HEAD of headroom.
	 * Buffers we allocated are padded to improve cache usage. In order
	 * to increase buffer re-use, we aim to keep any such buffers the
	 * same. This means the address passed to the FM should be DPA_BP_HEAD
	 * before the data for forwarded frames.
	 *
	 * However, offer some flexibility in fd layout, to allow originating
	 * (termination) buffers to be also recycled when possible.
	 *
	 * First, see if the conditions needed to recycle the skb are met:
	 * - skb not cloned, not shared
	 * - buffer size is large enough to accomodate a maximum size Rx frame
	 * - buffer address is 16 byte aligned, as per DPAARM
	 * - there's enough room in the buffer pool
	 */
	if (likely(skb_is_recycleable(skb, dpa_bp->size) &&
		   (*percpu_priv->dpa_bp_count < dpa_bp->target_count))) {
		/* Compute the minimum necessary fd offset */
		offset = dpa_bp->size - skb->len - skb_tailroom(skb);

		/*
		 * And make sure the offset is no lower than DPA_BP_HEAD,
		 * as required by FMan
		 */
		offset = max(offset, (int)DPA_BP_HEAD);

		/*
		 * We also need to align the buffer address to 16, such that
		 * Fman will be able to reuse it on Rx.
		 * Since the buffer going to FMan starts at (skb->data - offset)
		 * this is what we'll try to align. We already know that
		 * headroom is at least DPA_BP_HEAD bytes long, but with
		 * the extra offset needed for alignment we may go beyond
		 * the beginning of the buffer.
		 *
		 * Also need to check that we don't go beyond the maximum
		 * offset that can be set for a contiguous FD.
		 */
		extra_offset = (unsigned long)(skb->data - offset) & 0xF;
		if (likely((offset + extra_offset) <= skb_headroom(skb) &&
			   (offset + extra_offset) <= DPA_MAX_FD_OFFSET)) {
			/* We're good to go for recycling*/
			offset += extra_offset;
			can_recycle = true;
		}
	}

	if (likely(can_recycle)) {
		/* Buffer will get recycled, setup fd accordingly */
		fd->cmd |= FM_FD_CMD_FCO;
		fd->bpid = dpa_bp->bpid;
		/*
		 * Since the buffer will get back to the Bman pool
		 * and be re-used on Rx, map it for both read and write
		 */
		dma_dir = DMA_BIDIRECTIONAL;
	} else {
		/*
		 * No recycling here, so we don't care about address alignment.
		 * Just use the smallest offset required by FMan
		 */
		offset = DPA_BP_HEAD;
	}

	skbh = (struct sk_buff **)(skb->data - offset);
	*skbh = skb;


	/* Enable L3/L4 hardware checksum computation.
	 *
	 * We must do this before dma_map_single(), because we may
	 * need to write into the skb. */
	err = dpa_enable_tx_csum(priv, skb, fd,
				 ((char *)skbh) + DPA_TX_PRIV_DATA_SIZE);
	if (unlikely(err < 0)) {
		if (netif_msg_tx_err(priv) && net_ratelimit())
			cpu_netdev_err(net_dev, "HW csum error: %d\n", err);
		return err;
	}

	fd->format = qm_fd_contig;
	fd->length20 = skb->len;
	fd->offset = offset;

	addr = dma_map_single(dpa_bp->dev, skbh, dpa_bp->size, dma_dir);
	if (unlikely(dma_mapping_error(dpa_bp->dev, addr))) {
		if (netif_msg_tx_err(priv)  && net_ratelimit())
			cpu_netdev_err(net_dev, "dma_map_single() failed\n");
		return -EINVAL;
	}

	fd->addr_hi = upper_32_bits(addr);
	fd->addr_lo = lower_32_bits(addr);

	return 0;
}

int __hot dpa_tx(struct sk_buff *skb, struct net_device *net_dev)
{
	struct dpa_priv_s	*priv;
	struct qm_fd		 fd;
	struct dpa_percpu_priv_s *percpu_priv;
	int queue_mapping;
	int err;

	/* If there is a Tx hook, run it. */
	if (dpaa_eth_hooks.tx &&
		dpaa_eth_hooks.tx(skb, net_dev) == DPAA_ETH_STOLEN)
		/* won't update any Tx stats */
		return NETDEV_TX_OK;

	priv = netdev_priv(net_dev);
	percpu_priv = per_cpu_ptr(priv->percpu_priv, smp_processor_id());

	clear_fd(&fd);
	queue_mapping = skb_get_queue_mapping(skb);

	if (skb_headroom(skb) < DPA_BP_HEAD) {
		struct sk_buff *skb_new;

		skb_new = skb_realloc_headroom(skb, DPA_BP_HEAD);
		if (unlikely(!skb_new)) {
			percpu_priv->stats.tx_errors++;
			kfree_skb(skb);
			return NETDEV_TX_OK;
		}
		kfree_skb(skb);
		skb = skb_new;
	}

#ifdef CONFIG_FSL_DPA_1588
	if (priv->tsu && priv->tsu->valid && priv->tsu->hwts_tx_en_ioctl)
		fd.cmd |= FM_FD_CMD_UPD;
#endif

	/*
	 * We have two paths here:
	 *
	 * 1.If the skb is cloned, create a S/G frame to avoid unsharing it.
	 * The S/G table will contain only one entry, pointing to our skb
	 * data buffer.
	 * The private data area containing the skb backpointer will reside
	 * inside the first buffer, such that it won't risk being overwritten
	 * in case a second skb pointing to the same data buffer is being
	 * processed concurently.
	 * No recycling is possible in this case, as the data buffer is shared.
	 *
	 * 2.If skb is not cloned, then the private area inside it can be
	 * safely used to store the skb backpointer. Simply create a contiguous
	 * fd in this case.
	 * Recycling can happen if the right conditions are met.
	 */
	if (skb_cloned(skb) && (skb->len > DPA_SKB_COPY_MAX_SIZE))
		err = skb_to_sg_fd(priv, skb, &fd);
	else {
		/* If cloned skb, but length is below DPA_SKB_COPY_MAX_SIZE,
		 * it's more efficient to unshare it and then use the new skb */
		skb = skb_unshare(skb, GFP_ATOMIC);
		if (unlikely(!skb)) {
			percpu_priv->stats.tx_errors++;
			return NETDEV_TX_OK;
		}
		err = skb_to_contig_fd(priv, percpu_priv, skb, &fd);
	}
	if (unlikely(err < 0)) {
		percpu_priv->stats.tx_errors++;
		goto fd_create_failed;
	}

	if (fd.cmd & FM_FD_CMD_FCO) {
		/* This skb is recycleable, and the fd generated from it
		 * has been filled in accordingly */
		skb_recycle(skb);
		skb = NULL;
		(*percpu_priv->dpa_bp_count)++;
		percpu_priv->tx_returned++;
	}

	if (unlikely(dpa_xmit(priv, percpu_priv, queue_mapping, &fd) < 0))
		goto xmit_failed;

	net_dev->trans_start = jiffies;

	return NETDEV_TX_OK;

xmit_failed:
	if (fd.cmd & FM_FD_CMD_FCO) {
		(*percpu_priv->dpa_bp_count)--;
		percpu_priv->tx_returned--;
	}
fd_create_failed:
	_dpa_cleanup_tx_fd(priv, &fd);
	dev_kfree_skb(skb);

	return NETDEV_TX_OK;
}
#endif /* CONFIG_DPAA_ETH_SG_SUPPORT */

static enum qman_cb_dqrr_result
ingress_rx_error_dqrr(struct qman_portal		*portal,
		      struct qman_fq			*fq,
		      const struct qm_dqrr_entry	*dq)
{
	struct net_device		*net_dev;
	struct dpa_priv_s		*priv;
	struct dpa_percpu_priv_s	*percpu_priv;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);

	percpu_priv = per_cpu_ptr(priv->percpu_priv, smp_processor_id());

	if (dpaa_eth_napi_schedule(percpu_priv)) {
		percpu_priv->in_interrupt++;
		return qman_cb_dqrr_stop;
	}

	_dpa_rx_error(net_dev, priv, percpu_priv, &dq->fd, fq->fqid);

	return qman_cb_dqrr_consume;
}

static enum qman_cb_dqrr_result __hot
shared_rx_dqrr(struct qman_portal *portal, struct qman_fq *fq,
		const struct qm_dqrr_entry *dq)
{
	struct net_device		*net_dev;
	struct dpa_priv_s		*priv;
	struct dpa_percpu_priv_s	*percpu_priv;
	const struct qm_fd *fd = &dq->fd;
	struct dpa_bp *dpa_bp;
	size_t size;
	struct sk_buff *skb;
	void *dpa_bp_vaddr;
	dma_addr_t addr_offset;
	struct page *page;
	unsigned int offset;
	size_t remain, total_size;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);

	percpu_priv = per_cpu_ptr(priv->percpu_priv, smp_processor_id());

	if (unlikely(fd->status & FM_FD_STAT_ERRORS) != 0) {
		if (netif_msg_hw(priv) && net_ratelimit())
			cpu_netdev_warn(net_dev, "FD status = 0x%08x\n",
					fd->status & FM_FD_STAT_ERRORS);

		percpu_priv->stats.rx_errors++;

		goto out;
	}

	dpa_bp = dpa_bpid2pool(fd->bpid);
	BUG_ON(IS_ERR(dpa_bp));

	if (fd->format == qm_fd_sg) {
		percpu_priv->stats.rx_dropped++;
		if (netif_msg_rx_status(priv) && net_ratelimit())
			cpu_netdev_warn(net_dev,
				"%s:%hu:%s(): Dropping a SG frame\n",
				__file__, __LINE__, __func__);
		goto out;
	}

	size = dpa_fd_length(fd);

	skb = __netdev_alloc_skb(net_dev, DPA_BP_HEAD + size, GFP_ATOMIC);
	if (unlikely(skb == NULL)) {
		if (netif_msg_rx_err(priv) && net_ratelimit())
			cpu_netdev_err(net_dev, "Could not alloc skb\n");

		percpu_priv->stats.rx_dropped++;

		goto out;
	}

	skb_reserve(skb, DPA_BP_HEAD);

	if (dpa_bp->vaddr) {
		/* Fill the SKB */
		memcpy(skb_put(skb, dpa_fd_length(fd)),
		       dpa_phys2virt(dpa_bp, qm_fd_addr(fd)) +
		       dpa_fd_offset(fd), dpa_fd_length(fd));
		goto static_map;
	}

	remain = dpa_fd_offset(fd) + dpa_fd_length(fd);
	total_size = remain;
	addr_offset = qm_fd_addr(fd);

	while (remain > 0) {
		page = pfn_to_page(addr_offset >> PAGE_SHIFT);
		offset = offset_in_page(addr_offset);
		size = remain;

		if (remain + offset > PAGE_SIZE) {
			size = PAGE_SIZE - offset;
			addr_offset += size;
		}

		dpa_bp_vaddr = kmap_atomic(page);

		if (!offset && (total_size != remain))
			memcpy(skb_put(skb, size), dpa_bp_vaddr, size);
		else
			memcpy(skb_put(skb, size - dpa_fd_offset(fd)),
			       dpa_bp_vaddr + dpa_fd_offset(fd) +
			       offset,
			       size - dpa_fd_offset(fd));

		kunmap_atomic(dpa_bp_vaddr);

		remain -= size;
	}

static_map:
	skb->protocol = eth_type_trans(skb, net_dev);

	if (unlikely(dpa_check_rx_mtu(skb, net_dev->mtu))) {
		percpu_priv->stats.rx_dropped++;
		dev_kfree_skb_any(skb);
		goto out;
	}

	if (unlikely(netif_rx(skb) != NET_RX_SUCCESS))
		percpu_priv->stats.rx_dropped++;
	else {
		percpu_priv->stats.rx_packets++;
		percpu_priv->stats.rx_bytes += dpa_fd_length(fd);
	}

	net_dev->last_rx = jiffies;

out:
	dpa_fd_release(net_dev, fd);

	return qman_cb_dqrr_consume;
}


static enum qman_cb_dqrr_result __hot
ingress_rx_default_dqrr(struct qman_portal		*portal,
			struct qman_fq			*fq,
			const struct qm_dqrr_entry	*dq)
{
	struct net_device		*net_dev;
	struct dpa_priv_s		*priv;
	struct dpa_percpu_priv_s	*percpu_priv;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);

	percpu_priv = per_cpu_ptr(priv->percpu_priv, smp_processor_id());

	if (unlikely(dpaa_eth_napi_schedule(percpu_priv))) {
		percpu_priv->in_interrupt++;
		return qman_cb_dqrr_stop;
	}

	prefetchw(&percpu_priv->ingress_calls);

	_dpa_rx(net_dev, priv, percpu_priv, &dq->fd, fq->fqid);

	return qman_cb_dqrr_consume;
}

static enum qman_cb_dqrr_result
ingress_tx_error_dqrr(struct qman_portal		*portal,
		      struct qman_fq			*fq,
		      const struct qm_dqrr_entry	*dq)
{
	struct net_device		*net_dev;
	struct dpa_priv_s		*priv;
	struct dpa_percpu_priv_s	*percpu_priv;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);

	percpu_priv = per_cpu_ptr(priv->percpu_priv, smp_processor_id());

	if (dpaa_eth_napi_schedule(percpu_priv)) {
		percpu_priv->in_interrupt++;
		return qman_cb_dqrr_stop;
	}

	_dpa_tx_error(net_dev, priv, percpu_priv, &dq->fd, fq->fqid);

	return qman_cb_dqrr_consume;
}

static enum qman_cb_dqrr_result __hot
ingress_tx_default_dqrr(struct qman_portal		*portal,
			struct qman_fq			*fq,
			const struct qm_dqrr_entry	*dq)
{
	struct net_device		*net_dev;
	struct dpa_priv_s		*priv;
	struct dpa_percpu_priv_s	*percpu_priv;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);

	percpu_priv = per_cpu_ptr(priv->percpu_priv, smp_processor_id());

	if (dpaa_eth_napi_schedule(percpu_priv)) {
		percpu_priv->in_interrupt++;
		return qman_cb_dqrr_stop;
	}

	_dpa_tx_conf(net_dev, priv, percpu_priv, &dq->fd, fq->fqid);

	return qman_cb_dqrr_consume;
}

static void count_ern(struct dpa_percpu_priv_s *percpu_priv,
		      const struct qm_mr_entry *msg)
{
	switch (msg->ern.rc & QM_MR_RC_MASK) {
	case QM_MR_RC_CGR_TAILDROP:
		percpu_priv->ern_cnt.cg_tdrop++;
		break;
	case QM_MR_RC_WRED:
		percpu_priv->ern_cnt.wred++;
		break;
	case QM_MR_RC_ERROR:
		percpu_priv->ern_cnt.err_cond++;
		break;
	case QM_MR_RC_ORPWINDOW_EARLY:
		percpu_priv->ern_cnt.early_window++;
		break;
	case QM_MR_RC_ORPWINDOW_LATE:
		percpu_priv->ern_cnt.late_window++;
		break;
	case QM_MR_RC_FQ_TAILDROP:
		percpu_priv->ern_cnt.fq_tdrop++;
		break;
	case QM_MR_RC_ORPWINDOW_RETIRED:
		percpu_priv->ern_cnt.fq_retired++;
		break;
	case QM_MR_RC_ORP_ZERO:
		percpu_priv->ern_cnt.orp_zero++;
		break;
	}
}

static void shared_ern(struct qman_portal	*portal,
		       struct qman_fq		*fq,
		       const struct qm_mr_entry	*msg)
{
	struct net_device *net_dev;
	const struct dpa_priv_s	*priv;
	struct dpa_percpu_priv_s *percpu_priv;
	struct dpa_fq *dpa_fq = (struct dpa_fq *)fq;

	net_dev = dpa_fq->net_dev;
	priv = netdev_priv(net_dev);
	percpu_priv = per_cpu_ptr(priv->percpu_priv, smp_processor_id());

	dpa_fd_release(net_dev, &msg->ern.fd);

	percpu_priv->stats.tx_dropped++;
	percpu_priv->stats.tx_fifo_errors++;
	count_ern(percpu_priv, msg);
}

static void egress_ern(struct qman_portal	*portal,
		       struct qman_fq		*fq,
		       const struct qm_mr_entry	*msg)
{
	struct net_device	*net_dev;
	const struct dpa_priv_s	*priv;
	struct sk_buff *skb;
	struct dpa_percpu_priv_s	*percpu_priv;
	struct qm_fd fd = msg->ern.fd;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);
	percpu_priv = per_cpu_ptr(priv->percpu_priv, smp_processor_id());

	percpu_priv->stats.tx_dropped++;
	percpu_priv->stats.tx_fifo_errors++;
	count_ern(percpu_priv, msg);

	/*
	 * If we intended this buffer to go into the pool
	 * when the FM was done, we need to put it in
	 * manually.
	 */
	if (msg->ern.fd.cmd & FM_FD_CMD_FCO) {
		dpa_fd_release(net_dev, &fd);
		return;
	}

	skb = _dpa_cleanup_tx_fd(priv, &fd);
	dev_kfree_skb_any(skb);
}

static const struct qman_fq rx_shared_fq __devinitconst = {
	.cb = { .dqrr = shared_rx_dqrr }
};
static const struct qman_fq rx_private_defq __devinitconst = {
	.cb = { .dqrr = ingress_rx_default_dqrr }
};
static const struct qman_fq rx_private_errq __devinitconst = {
	.cb = { .dqrr = ingress_rx_error_dqrr }
};
static const struct qman_fq tx_private_defq __devinitconst = {
	.cb = { .dqrr = ingress_tx_default_dqrr }
};
static const struct qman_fq tx_private_errq __devinitconst = {
	.cb = { .dqrr = ingress_tx_error_dqrr }
};
static const struct qman_fq dummyq __devinitconst = { };
static const struct qman_fq private_egress_fq __devinitconst = {
	.cb = { .ern = egress_ern }
};
static const struct qman_fq shared_egress_fq __devinitconst = {
	.cb = { .ern = shared_ern }
};

#ifdef CONFIG_DPAA_ETH_UNIT_TESTS
static bool __devinitdata tx_unit_test_passed = true;

static void __devinit tx_unit_test_ern(struct qman_portal	*portal,
		       struct qman_fq		*fq,
		       const struct qm_mr_entry	*msg)
{
	struct net_device *net_dev;
	struct dpa_priv_s *priv;
	struct sk_buff **skbh;
	struct sk_buff *skb;
	const struct qm_fd *fd;
	dma_addr_t addr;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);

	tx_unit_test_passed = false;

	fd = &msg->ern.fd;

	addr = qm_fd_addr(fd);

	skbh = (struct sk_buff **)phys_to_virt(addr);
	skb = *skbh;

	if (!skb || !is_kernel_addr((unsigned long)skb))
		panic("Corrupt skb in ERN!\n");

	kfree_skb(skb);
}

static unsigned char __devinitdata *tx_unit_skb_head;
static unsigned char __devinitdata *tx_unit_skb_end;
static int __devinitdata tx_unit_tested;

static enum qman_cb_dqrr_result __devinit tx_unit_test_dqrr(
		struct qman_portal *portal,
		struct qman_fq *fq,
		const struct qm_dqrr_entry *dq)
{
	struct net_device *net_dev;
	struct dpa_priv_s *priv;
	struct sk_buff **skbh;
	struct sk_buff *skb;
	const struct qm_fd *fd;
	dma_addr_t addr;
	unsigned char *startaddr;
	struct dpa_percpu_priv_s *percpu_priv;

	tx_unit_test_passed = false;

	tx_unit_tested++;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);

	percpu_priv = per_cpu_ptr(priv->percpu_priv, smp_processor_id());

	fd = &dq->fd;

	addr = qm_fd_addr(fd);

	skbh = (struct sk_buff **)phys_to_virt(addr);
	startaddr = (unsigned char *)skbh;
	skb = *skbh;

	if (!skb || !is_kernel_addr((unsigned long)skb))
		panic("Invalid skb address in TX Unit Test FD\n");

	/* Make sure we're dealing with the same skb */
	if (skb->head != tx_unit_skb_head
			|| skb_end_pointer(skb) != tx_unit_skb_end)
		goto out;

	/*
	 * If we recycled, then there must be enough room between fd.addr
	 * and skb->end for a new RX buffer
	 */
	if (fd->cmd & FM_FD_CMD_FCO) {
		size_t bufsize = skb_end_pointer(skb) - startaddr;

		if (bufsize < fsl_fman_phy_maxfrm)
			goto out;
	} else {
		/*
		 * If we didn't recycle, but the buffer was big enough,
		 * increment the counter to put it back
		 */
		if (skb_end_pointer(skb) - skb->head >= fsl_fman_phy_maxfrm)
			(*percpu_priv->dpa_bp_count)++;

		/* If we didn't recycle, the data pointer should be good */
		if (skb->data != startaddr + dpa_fd_offset(fd))
			goto out;
	}

	tx_unit_test_passed = true;
out:
	/* The skb is no longer needed, and belongs to us */
	kfree_skb(skb);

	return qman_cb_dqrr_consume;
}

static const struct qman_fq tx_unit_test_fq __devinitconst = {
	.cb = { .dqrr = tx_unit_test_dqrr, .ern = tx_unit_test_ern }
};

static struct __devinitdata dpa_fq unit_fq;

static bool __devinitdata tx_unit_test_ran; /* Starts as false */

static int __devinit dpa_tx_unit_test(struct net_device *net_dev)
{
	/* Create a new FQ */
	struct dpa_priv_s *priv = netdev_priv(net_dev);
	struct qman_fq *oldq;
	int size, headroom;
	struct dpa_percpu_priv_s *percpu_priv;
	cpumask_var_t old_cpumask;
	int test_count = 0;
	int err = 0;
	int tests_failed = 0;
	const cpumask_t *cpus = qman_affine_cpus();

	if (!alloc_cpumask_var(&old_cpumask, GFP_KERNEL)) {
		pr_err("UNIT test cpumask allocation failed\n");
		return -ENOMEM;
	}

	cpumask_copy(old_cpumask, tsk_cpus_allowed(current));
	set_cpus_allowed_ptr(current, cpus);
	/* disable bottom halves */
	local_bh_disable();

	percpu_priv = per_cpu_ptr(priv->percpu_priv, smp_processor_id());

	qman_irqsource_remove(QM_PIRQ_DQRI);
	unit_fq.net_dev = net_dev;
	unit_fq.fq_base = tx_unit_test_fq;

	/* Save old queue */
	oldq = priv->egress_fqs[smp_processor_id()];

	err = qman_create_fq(0, QMAN_FQ_FLAG_DYNAMIC_FQID, &unit_fq.fq_base);

	if (err < 0) {
		pr_err("UNIT test FQ create failed: %d\n", err);
		goto fq_create_fail;
	}

	err = qman_init_fq(&unit_fq.fq_base,
			QMAN_INITFQ_FLAG_SCHED | QMAN_INITFQ_FLAG_LOCAL, NULL);
	if (err < 0) {
		pr_err("UNIT test FQ init failed: %d\n", err);
		goto fq_init_fail;
	}

	pr_err("TX Unit Test using FQ %d\n", qman_fq_fqid(&unit_fq.fq_base));

	/* Replace queue 0 with this queue */
	priv->egress_fqs[smp_processor_id()] = &unit_fq.fq_base;

	/* Try packet sizes from 64-bytes to just above the maximum */
	for (size = 64; size <= 9600 + 128; size += 64) {
		for (headroom = DPA_BP_HEAD; headroom < 0x800; headroom += 16) {
			int ret;
			struct sk_buff *skb;

			test_count++;

			skb = dev_alloc_skb(size + headroom);

			if (!skb) {
				pr_err("Failed to allocate skb\n");
				err = -ENOMEM;
				goto end_test;
			}

			if (skb_end_pointer(skb) - skb->head >=
					fsl_fman_phy_maxfrm)
				(*percpu_priv->dpa_bp_count)--;

			skb_put(skb, size + headroom);
			skb_pull(skb, headroom);

			tx_unit_skb_head = skb->head;
			tx_unit_skb_end = skb_end_pointer(skb);

			skb_set_queue_mapping(skb, smp_processor_id());

			/* tx */
			ret = net_dev->netdev_ops->ndo_start_xmit(skb, net_dev);

			if (ret != NETDEV_TX_OK) {
				pr_err("Failed to TX with err %d\n", ret);
				err = -EIO;
				goto end_test;
			}

			/* Wait for it to arrive */
			ret = spin_event_timeout(qman_poll_dqrr(1) != 0,
					100000, 1);

			if (!ret)
				pr_err("TX Packet never arrived\n");

			/* Was it good? */
			if (tx_unit_test_passed == false) {
				pr_err("Test failed:\n");
				pr_err("size: %d pad: %d head: %p end: %p\n",
					size, headroom, tx_unit_skb_head,
					tx_unit_skb_end);
				tests_failed++;
			}
		}
	}

end_test:
	err = qman_retire_fq(&unit_fq.fq_base, NULL);
	if (unlikely(err < 0))
		pr_err("Could not retire TX Unit Test FQ (%d)\n", err);

	err = qman_oos_fq(&unit_fq.fq_base);
	if (unlikely(err < 0))
		pr_err("Could not OOS TX Unit Test FQ (%d)\n", err);

fq_init_fail:
	qman_destroy_fq(&unit_fq.fq_base, 0);

fq_create_fail:
	priv->egress_fqs[smp_processor_id()] = oldq;
	local_bh_enable();
	qman_irqsource_add(QM_PIRQ_DQRI);
	tx_unit_test_ran = true;
	set_cpus_allowed_ptr(current, old_cpumask);
	free_cpumask_var(old_cpumask);

	pr_err("Tested %d/%d packets. %d failed\n", test_count, tx_unit_tested,
		tests_failed);

	if (tests_failed)
		err = -EINVAL;

	/* Reset counters */
	memset(&percpu_priv->stats, 0, sizeof(percpu_priv->stats));

	return err;
}
#endif

static int __cold dpa_start(struct net_device *net_dev)
{
	int err, i;
	struct dpa_priv_s *priv;
	struct mac_device *mac_dev;

	priv = netdev_priv(net_dev);
	mac_dev = priv->mac_dev;

	if (!mac_dev)
		goto no_mac;

	dpaa_eth_napi_enable(priv);

	err = mac_dev->init_phy(net_dev);
	if (err < 0) {
		if (netif_msg_ifup(priv))
			cpu_netdev_err(net_dev, "init_phy() = %d\n", err);
		goto init_phy_failed;
	}

	for_each_port_device(i, mac_dev->port_dev)
		fm_port_enable(mac_dev->port_dev[i]);

	err = priv->mac_dev->start(mac_dev);
	if (err < 0) {
		if (netif_msg_ifup(priv))
			cpu_netdev_err(net_dev, "mac_dev->start() = %d\n", err);
		goto mac_start_failed;
	}

no_mac:
	netif_tx_start_all_queues(net_dev);

	return 0;

mac_start_failed:
	for_each_port_device(i, mac_dev->port_dev)
		fm_port_disable(mac_dev->port_dev[i]);

init_phy_failed:
	dpaa_eth_napi_disable(priv);

	return err;
}

static int __cold dpa_stop(struct net_device *net_dev)
{
	int _errno, i;
	struct dpa_priv_s *priv;
	struct mac_device *mac_dev;

	priv = netdev_priv(net_dev);
	mac_dev = priv->mac_dev;

	netif_tx_stop_all_queues(net_dev);

	if (!mac_dev)
		return 0;

	_errno = mac_dev->stop(mac_dev);
	if (unlikely(_errno < 0))
		if (netif_msg_ifdown(priv))
			cpu_netdev_err(net_dev, "mac_dev->stop() = %d\n",
					_errno);

	for_each_port_device(i, mac_dev->port_dev)
		fm_port_disable(mac_dev->port_dev[i]);

	if (mac_dev->phy_dev)
		phy_disconnect(mac_dev->phy_dev);
	mac_dev->phy_dev = NULL;

	dpaa_eth_napi_disable(priv);

	return _errno;
}

static void __cold dpa_timeout(struct net_device *net_dev)
{
	const struct dpa_priv_s	*priv;
	struct dpa_percpu_priv_s *percpu_priv;

	priv = netdev_priv(net_dev);
	percpu_priv = per_cpu_ptr(priv->percpu_priv, smp_processor_id());

	if (netif_msg_timer(priv))
		cpu_netdev_crit(net_dev, "Transmit timeout latency: %lu ms\n",
				(jiffies - net_dev->trans_start) * 1000 / HZ);

	percpu_priv->stats.tx_errors++;
}

static int __devinit dpa_bp_cmp(const void *dpa_bp0, const void *dpa_bp1)
{
	return ((struct dpa_bp *)dpa_bp0)->size -
			((struct dpa_bp *)dpa_bp1)->size;
}

static struct dpa_bp * __devinit __cold __must_check __attribute__((nonnull))
dpa_bp_probe(struct platform_device *_of_dev, size_t *count)
{
	int			 i, lenp, na, ns;
	struct device		*dev;
	struct device_node	*dev_node;
	const phandle		*phandle_prop;
	const uint32_t		*bpid;
	const uint32_t		*bpool_cfg;
	struct dpa_bp		*dpa_bp;
	int has_kernel_pool = 0;
	int has_shared_pool = 0;

	dev = &_of_dev->dev;

	/* The default is one, if there's no property */
	*count = 1;

	/* There are three types of buffer pool configuration:
	 * 1) No bp assignment
	 * 2) A static assignment to an empty configuration
	 * 3) A static assignment to one or more configured pools
	 *
	 * We don't support using multiple unconfigured pools.
	 */

	/* Get the buffer pools to be used */
	phandle_prop = of_get_property(dev->of_node,
					"fsl,bman-buffer-pools", &lenp);

	if (phandle_prop)
		*count = lenp / sizeof(phandle);
	else {
		if (default_pool)
			return default_pool;

		has_kernel_pool = 1;
	}

	dpa_bp = devm_kzalloc(dev, *count * sizeof(*dpa_bp), GFP_KERNEL);
	if (unlikely(dpa_bp == NULL)) {
		dpaa_eth_err(dev, "devm_kzalloc() failed\n");
		return ERR_PTR(-ENOMEM);
	}

	dev_node = of_find_node_by_path("/");
	if (unlikely(dev_node == NULL)) {
		dpaa_eth_err(dev, "of_find_node_by_path(/) failed\n");
		return ERR_PTR(-EINVAL);
	}

	na = of_n_addr_cells(dev_node);
	ns = of_n_size_cells(dev_node);

	for (i = 0; i < *count && phandle_prop; i++) {
		of_node_put(dev_node);
		dev_node = of_find_node_by_phandle(phandle_prop[i]);
		if (unlikely(dev_node == NULL)) {
			dpaa_eth_err(dev, "of_find_node_by_phandle() failed\n");
			return ERR_PTR(-EFAULT);
		}

		if (unlikely(!of_device_is_compatible(dev_node, "fsl,bpool"))) {
			dpaa_eth_err(dev,
				"!of_device_is_compatible(%s, fsl,bpool)\n",
				dev_node->full_name);
			dpa_bp = ERR_PTR(-EINVAL);
			goto _return_of_node_put;
		}

		bpid = of_get_property(dev_node, "fsl,bpid", &lenp);
		if ((bpid == NULL) || (lenp != sizeof(*bpid))) {
			dpaa_eth_err(dev, "fsl,bpid property not found.\n");
			dpa_bp = ERR_PTR(-EINVAL);
			goto _return_of_node_put;
		}
		dpa_bp[i].bpid = *bpid;

		bpool_cfg = of_get_property(dev_node, "fsl,bpool-ethernet-cfg",
					&lenp);
		if (bpool_cfg && (lenp == (2 * ns + na) * sizeof(*bpool_cfg))) {
			const uint32_t *seed_pool;

			dpa_bp[i].config_count =
				(int)of_read_number(bpool_cfg, ns);
			dpa_bp[i].size	= of_read_number(bpool_cfg + ns, ns);
			dpa_bp[i].paddr	=
				of_read_number(bpool_cfg + 2 * ns, na);

			seed_pool = of_get_property(dev_node,
					"fsl,bpool-ethernet-seeds", &lenp);
			dpa_bp[i].seed_pool = !!seed_pool;

			has_shared_pool = 1;
		} else {
			has_kernel_pool = 1;
		}

		if (i > 0)
			has_shared_pool = 1;
	}

	if (has_kernel_pool && has_shared_pool) {
		dpaa_eth_err(dev, "Invalid buffer pool configuration "
			"for node %s\n", dev_node->full_name);
		dpa_bp = ERR_PTR(-EINVAL);
		goto _return_of_node_put;
	} else if (has_kernel_pool) {
		dpa_bp->target_count = DEFAULT_COUNT;
		dpa_bp->size = DEFAULT_BUF_SIZE;
#ifdef CONFIG_DPAA_ETH_SG_SUPPORT
		if (dpa_bp->size > PAGE_SIZE) {
			dpaa_eth_warning(dev, "Default buffer size too large. "
				     "Round down to PAGE_SIZE\n");
			dpa_bp->size = PAGE_SIZE;
		}
#endif
		dpa_bp->kernel_pool = 1;
	}

	sort(dpa_bp, *count, sizeof(*dpa_bp), dpa_bp_cmp, NULL);

	return dpa_bp;

_return_of_node_put:
	if (dev_node)
		of_node_put(dev_node);

	return dpa_bp;
}

static int dpa_bp_create(struct net_device *net_dev, struct dpa_bp *dpa_bp,
			size_t count)
{
	struct dpa_priv_s *priv = netdev_priv(net_dev);
	int i;

	if (dpa_bp->kernel_pool) {
		priv->shared = 0;

		if (netif_msg_probe(priv))
			cpu_dev_info(net_dev->dev.parent,
				"Using private BM buffer pools\n");
	} else {
		priv->shared = 1;
	}

	priv->dpa_bp = dpa_bp;
	priv->bp_count = count;

	for (i = 0; i < count; i++) {
		int err;
		err = dpa_bp_alloc(&dpa_bp[i]);
		if (err < 0) {
			dpa_bp_free(priv, dpa_bp);
			priv->dpa_bp = NULL;
			return err;
		}

		/* For now, just point to the default pool.
		 * We can add support for more pools, later
		 */
		if (dpa_bp->kernel_pool)
			priv->dpa_bp = default_pool;
	}

	return 0;
}

static struct mac_device * __devinit __cold __must_check
__attribute__((nonnull))
dpa_mac_probe(struct platform_device *_of_dev)
{
	struct device		*dpa_dev, *dev;
	struct device_node	*mac_node;
	int			 lenp;
	const phandle		*phandle_prop;
	struct platform_device	*of_dev;
	struct mac_device	*mac_dev;
#ifdef CONFIG_FSL_DPA_1588
	struct net_device	*net_dev = NULL;
	struct dpa_priv_s	*priv = NULL;
	struct device_node	*timer_node;
#endif

	phandle_prop = of_get_property(_of_dev->dev.of_node,
					"fsl,fman-mac", &lenp);
	if (phandle_prop == NULL)
		return NULL;

	BUG_ON(lenp != sizeof(phandle));

	dpa_dev = &_of_dev->dev;

	mac_node = of_find_node_by_phandle(*phandle_prop);
	if (unlikely(mac_node == NULL)) {
		dpaa_eth_err(dpa_dev, "of_find_node_by_phandle() failed\n");
		return ERR_PTR(-EFAULT);
	}

	of_dev = of_find_device_by_node(mac_node);
	if (unlikely(of_dev == NULL)) {
		dpaa_eth_err(dpa_dev, "of_find_device_by_node(%s) failed\n",
				mac_node->full_name);
		of_node_put(mac_node);
		return ERR_PTR(-EINVAL);
	}
	of_node_put(mac_node);

	dev = &of_dev->dev;

	mac_dev = dev_get_drvdata(dev);
	if (unlikely(mac_dev == NULL)) {
		dpaa_eth_err(dpa_dev, "dev_get_drvdata(%s) failed\n",
				dev_name(dev));
		return ERR_PTR(-EINVAL);
	}

#ifdef CONFIG_FSL_DPA_1588
	phandle_prop = of_get_property(mac_node, "ptimer-handle", &lenp);
	if (phandle_prop && ((mac_dev->phy_if != PHY_INTERFACE_MODE_SGMII) ||
			((mac_dev->phy_if == PHY_INTERFACE_MODE_SGMII) &&
			 (mac_dev->speed == SPEED_1000)))) {
		timer_node = of_find_node_by_phandle(*phandle_prop);
		if (timer_node && (net_dev = dev_get_drvdata(dpa_dev))) {
			priv = netdev_priv(net_dev);
			if (!dpa_ptp_init(priv))
				dpaa_eth_info(dev, "%s: ptp-timer enabled\n",
						mac_node->full_name);
		}
	}
#endif

	return mac_dev;
}

static const char fsl_qman_frame_queues[][25] __devinitconst = {
	[RX] = "fsl,qman-frame-queues-rx",
	[TX] = "fsl,qman-frame-queues-tx"
};

#ifdef CONFIG_DEBUG_FS
static int __cold dpa_debugfs_show(struct seq_file *file, void *offset)
{
	int				 i;
	struct dpa_priv_s		*priv;
	struct dpa_percpu_priv_s	*percpu_priv, total;
	struct dpa_bp *dpa_bp;
	unsigned int dpa_bp_count = 0;
	unsigned int count_total = 0;

	BUG_ON(offset == NULL);

	priv = netdev_priv((struct net_device *)file->private);

	dpa_bp = priv->dpa_bp;

	memset(&total, 0, sizeof(total));

	seq_printf(file, "\nDPA counters for %s:\n"
		"CPU           irqs        rx        tx   recycle" \
		"   confirm    tx err    rx err  bp count\n",
		priv->net_dev->name);
	for_each_online_cpu(i) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, i);

		/* Only private interfaces have an associated counter for bp
		 * buffers */
		if (!priv->shared)
			dpa_bp_count = *percpu_priv->dpa_bp_count;

		total.in_interrupt += percpu_priv->in_interrupt;
		total.ingress_calls += percpu_priv->stats.rx_packets;
		total.stats.tx_packets += percpu_priv->stats.tx_packets;
		total.tx_returned += percpu_priv->tx_returned;
		total.tx_confirm += percpu_priv->tx_confirm;
		total.stats.tx_errors += percpu_priv->stats.tx_errors;
		total.stats.rx_errors += percpu_priv->stats.rx_errors;
		count_total += dpa_bp_count;

		seq_printf(file, "     %hu/%hu  %8u  %8lu  %8lu  %8u  %8u" \
				"  %8lu  %8lu  %8d\n",
				get_hard_smp_processor_id(i), i,
				percpu_priv->in_interrupt,
				percpu_priv->stats.rx_packets,
				percpu_priv->stats.tx_packets,
				percpu_priv->tx_returned,
				percpu_priv->tx_confirm,
				percpu_priv->stats.tx_errors,
				percpu_priv->stats.rx_errors,
				dpa_bp_count);
	}
	seq_printf(file, "Total     %8u  %8u  %8lu  %8u  %8u  %8lu  %8lu" \
				"  %8d\n",
			total.in_interrupt,
			total.ingress_calls,
			total.stats.tx_packets,
			total.tx_returned,
			total.tx_confirm,
			total.stats.tx_errors,
			total.stats.rx_errors,
			count_total);

	seq_printf(file, "\nDPA RX Errors:\nCPU        dma err  phys err" \
				"  size err   hdr err  csum err\n");
		for_each_online_cpu(i) {
			percpu_priv = per_cpu_ptr(priv->percpu_priv, i);

			total.rx_errors.dme += percpu_priv->rx_errors.dme;
			total.rx_errors.fpe += percpu_priv->rx_errors.fpe;
			total.rx_errors.fse += percpu_priv->rx_errors.fse;
			total.rx_errors.phe += percpu_priv->rx_errors.phe;
			total.rx_errors.cse += percpu_priv->rx_errors.cse;

			seq_printf(file, "     %hu/%hu  %8u  %8u  %8u" \
						"  %8u  %8u\n",
					get_hard_smp_processor_id(i), i,
					percpu_priv->rx_errors.dme,
					percpu_priv->rx_errors.fpe,
					percpu_priv->rx_errors.fse,
					percpu_priv->rx_errors.phe,
					percpu_priv->rx_errors.cse);
		}
		seq_printf(file, "Total     %8u  %8u  %8u  %8u  %8u\n",
				total.rx_errors.dme,
				total.rx_errors.fpe,
				total.rx_errors.fse,
				total.rx_errors.phe,
				total.rx_errors.cse);

	seq_printf(file, "\nDPA ERN counters:\n  CPU     cg_td      wred  " \
			"err_cond   early_w    late_w     fq_td    fq_ret" \
			"     orp_z\n");
	for_each_online_cpu(i) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, i);

		total.ern_cnt.cg_tdrop += percpu_priv->ern_cnt.cg_tdrop;
		total.ern_cnt.wred += percpu_priv->ern_cnt.wred;
		total.ern_cnt.err_cond += percpu_priv->ern_cnt.err_cond;
		total.ern_cnt.early_window += percpu_priv->ern_cnt.early_window;
		total.ern_cnt.late_window += percpu_priv->ern_cnt.late_window;
		total.ern_cnt.fq_tdrop += percpu_priv->ern_cnt.fq_tdrop;
		total.ern_cnt.fq_retired += percpu_priv->ern_cnt.fq_retired;
		total.ern_cnt.orp_zero += percpu_priv->ern_cnt.orp_zero;

		seq_printf(file, "  %hu/%hu  %8u  %8u  %8u  %8u  %8u  %8u" \
			"  %8u  %8u\n",
			get_hard_smp_processor_id(i), i,
			percpu_priv->ern_cnt.cg_tdrop,
			percpu_priv->ern_cnt.wred,
			percpu_priv->ern_cnt.err_cond,
			percpu_priv->ern_cnt.early_window,
			percpu_priv->ern_cnt.late_window,
			percpu_priv->ern_cnt.fq_tdrop,
			percpu_priv->ern_cnt.fq_retired,
			percpu_priv->ern_cnt.orp_zero);
	}
	seq_printf(file, "Total  %8u  %8u  %8u  %8u  %8u  %8u  %8u  %8u\n",
		total.ern_cnt.cg_tdrop,
		total.ern_cnt.wred,
		total.ern_cnt.err_cond,
		total.ern_cnt.early_window,
		total.ern_cnt.late_window,
		total.ern_cnt.fq_tdrop,
		total.ern_cnt.fq_retired,
		total.ern_cnt.orp_zero);

	return 0;
}

static int __cold dpa_debugfs_open(struct inode *inode, struct file *file)
{
	int			 _errno;
	const struct net_device	*net_dev;

	_errno = single_open(file, dpa_debugfs_show, inode->i_private);
	if (unlikely(_errno < 0)) {
		net_dev = (struct net_device *)inode->i_private;

		if (netif_msg_drv((struct dpa_priv_s *)netdev_priv(net_dev)))
			cpu_netdev_err(net_dev, "single_open() = %d\n",
					_errno);
	}
	return _errno;
}

static const struct file_operations dpa_debugfs_fops = {
	.open		= dpa_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif

static u16 dpa_select_queue(struct net_device *net_dev, struct sk_buff *skb)
{
	return smp_processor_id();
}

static const struct net_device_ops dpa_private_ops = {
	.ndo_open = dpa_start,
	.ndo_start_xmit = dpa_tx,
	.ndo_stop = dpa_stop,
	.ndo_tx_timeout = dpa_timeout,
	.ndo_get_stats = dpa_get_stats,
	.ndo_set_mac_address = dpa_set_mac_address,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_select_queue = dpa_select_queue,
	.ndo_change_mtu = dpa_change_mtu,
	.ndo_set_rx_mode = dpa_set_rx_mode,
#ifdef CONFIG_FSL_DPA_1588
	.ndo_do_ioctl = dpa_ioctl,
#endif
};

static const struct net_device_ops dpa_shared_ops = {
	.ndo_open = dpa_start,
	.ndo_start_xmit = dpa_shared_tx,
	.ndo_stop = dpa_stop,
	.ndo_tx_timeout = dpa_timeout,
	.ndo_get_stats = dpa_get_stats,
	.ndo_set_mac_address = dpa_set_mac_address,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_set_rx_mode = dpa_set_rx_mode,
#ifdef CONFIG_FSL_DPA_1588
	.ndo_do_ioctl = dpa_ioctl,
#endif
};

static int __devinit dpa_get_channel(struct device *dev,
					struct device_node *dpa_node)
{
	struct device_node *dev_node;
	const uint32_t *channel_id;
	int lenp;

	dev_node = of_parse_phandle(dpa_node, "fsl,qman-channel", 0);
	if (dev_node == NULL) {
		dpaa_eth_err(dev, "Could not find fsl,qman-channel property\n");
		return -EFAULT;
	}

	channel_id = of_get_property(dev_node, "fsl,qman-channel-id", &lenp);
	if ((channel_id == NULL) || (lenp < sizeof(*channel_id))) {
		dpaa_eth_err(dev, "Could not get fsl,qman-channel-id in %s\n",
				dev_node->full_name);
		of_node_put(dev_node);
		return -EINVAL;
	}
	of_node_put(dev_node);
	return *channel_id;
}

struct fqid_cell {
	uint32_t start;
	uint32_t count;
};

static const struct fqid_cell default_fqids[][3] __devinitconst = {
	[RX] = { {0, 1}, {0, 1}, {0, DPAA_ETH_RX_QUEUES} },
	[TX] = { {0, 1}, {0, 1}, {0, DPAA_ETH_TX_QUEUES} }
};

static int __devinit
dpa_fq_probe(struct platform_device *_of_dev, struct list_head *list,
		struct dpa_fq **defq, struct dpa_fq **errq,
		struct dpa_fq **fqs, int ptype)
{
	struct device *dev = &_of_dev->dev;
	struct device_node *np = dev->of_node;
	const struct fqid_cell *fqids;
	int i, j, lenp;
	int num_fqids;
	struct dpa_fq *dpa_fq;
	int err = 0;

	fqids = of_get_property(np, fsl_qman_frame_queues[ptype], &lenp);
	if (fqids == NULL) {
		fqids = default_fqids[ptype];
		num_fqids = 3;
	} else
		num_fqids = lenp / sizeof(*fqids);

	for (i = 0; i < num_fqids; i++) {
		dpa_fq = devm_kzalloc(dev, sizeof(*dpa_fq) * fqids[i].count,
					GFP_KERNEL);
		if (dpa_fq == NULL) {
			dpaa_eth_err(dev, "devm_kzalloc() failed\n");
			return -ENOMEM;
		}

		/* The first queue is the Error queue */
		if (i == 0 && errq) {
			*errq = dpa_fq;

			if (fqids[i].count != 1) {
				dpaa_eth_err(dev, "Too many error queues!\n");
				err = -EINVAL;
				goto invalid_error_queues;
			}

			dpa_fq[0].fq_type = (ptype == RX ?
				FQ_TYPE_RX_ERROR : FQ_TYPE_TX_ERROR);
		}

		/* The second queue is the the Default queue */
		if (i == 1 && defq) {
			*defq = dpa_fq;

			if (fqids[i].count != 1) {
				dpaa_eth_err(dev, "Too many default queues!\n");
				err = -EINVAL;
				goto invalid_default_queues;
			}

			dpa_fq[0].fq_type = (ptype == RX ?
				FQ_TYPE_RX_DEFAULT : FQ_TYPE_TX_CONFIRM);
		}

		/*
		 * All subsequent queues are gathered together.
		 * The first 8 will be used by the private linux interface
		 * if these are TX queues
		 */
		if (i == 2 || (!errq && i == 0 && fqs)) {
			*fqs = dpa_fq;

			for (j = 0; j < fqids[i].count; j++)
				dpa_fq[j].fq_type = (ptype == RX ?
					FQ_TYPE_RX_PCD : FQ_TYPE_TX);
		}

#warning We lost the 8-queue enforcement

		for (j = 0; j < fqids[i].count; j++) {
			dpa_fq[j].fqid = fqids[i].start ?
				fqids[i].start + j : 0;
			_dpa_assign_wq(dpa_fq + j);
			list_add_tail(&dpa_fq[j].list, list);
		}
	}

invalid_default_queues:
invalid_error_queues:
	return err;
}

static void dpa_setup_ingress(struct dpa_priv_s *priv, struct dpa_fq *fq,
			const struct qman_fq *template)
{
	fq->fq_base = *template;
	fq->net_dev = priv->net_dev;

	fq->flags = QMAN_FQ_FLAG_NO_ENQUEUE;
	fq->channel = priv->channel;
}

static void dpa_setup_egress(struct dpa_priv_s *priv,
				struct list_head *head, struct dpa_fq *fq,
				struct fm_port *port)
{
	struct list_head *ptr = &fq->list;
	int i = 0;

	while (true) {
		struct dpa_fq *iter = list_entry(ptr, struct dpa_fq, list);
		if (priv->shared)
			iter->fq_base = shared_egress_fq;
		else
			iter->fq_base = private_egress_fq;

		iter->net_dev = priv->net_dev;
		priv->egress_fqs[i++] = &iter->fq_base;

		if (port) {
			iter->flags = QMAN_FQ_FLAG_TO_DCPORTAL;
			iter->channel = fm_get_tx_port_channel(port);
		} else
			iter->flags = QMAN_FQ_FLAG_NO_MODIFY;

		if (list_is_last(ptr, head))
			break;

		ptr = ptr->next;
	}
}

static void dpa_setup_ingress_queues(struct dpa_priv_s *priv,
		struct list_head *head, struct dpa_fq *fq)
{
	struct list_head *ptr = &fq->list;
	u32 fqid;
	int portals[NR_CPUS];
	int i, cpu, num_portals = 0;
	const cpumask_t *affine_cpus = qman_affine_cpus();

	for_each_cpu(cpu, affine_cpus)
		portals[num_portals++] = qman_affine_channel(cpu);
	if (num_portals == 0) {
		dpaa_eth_err(fq->net_dev->dev.parent,
			     "No Qman software (affine) channels found");
		return;
	}

	i = 0;
	fqid = 0;
	if (priv->mac_dev)
		fqid = (priv->mac_dev->res->start & 0x1fffff) >> 6;

	while (true) {
		struct dpa_fq *iter = list_entry(ptr, struct dpa_fq, list);

		if (priv->shared)
			dpa_setup_ingress(priv, iter, &rx_shared_fq);
		else
			dpa_setup_ingress(priv, iter, &rx_private_defq);

		if (!iter->fqid)
			iter->fqid = fqid++;

		/* Assign the queues to a channel in a round-robin fashion */
		iter->channel = portals[i];
		i = (i + 1) % num_portals;

		if (list_is_last(ptr, head))
			break;

		ptr = ptr->next;
	}
}

static void __devinit
dpaa_eth_init_tx_port(struct fm_port *port, struct dpa_fq *errq,
		struct dpa_fq *defq, bool has_timer)
{
	struct fm_port_non_rx_params tx_port_param;

	dpaa_eth_init_port(tx, port, tx_port_param, errq->fqid, defq->fqid,
			DPA_TX_PRIV_DATA_SIZE, has_timer);
}

static void __devinit
dpaa_eth_init_rx_port(struct fm_port *port, struct dpa_bp *bp, size_t count,
		struct dpa_fq *errq, struct dpa_fq *defq, bool has_timer)
{
	struct fm_port_rx_params rx_port_param;
	int i;

	count = min(ARRAY_SIZE(rx_port_param.pool_param), count);
	rx_port_param.num_pools = count;
	for (i = 0; i < count; i++) {
		if (i >= rx_port_param.num_pools)
			break;

		rx_port_param.pool_param[i].id = bp[i].bpid;
		rx_port_param.pool_param[i].size = bp[i].size;
	}

	dpaa_eth_init_port(rx, port, rx_port_param, errq->fqid, defq->fqid,
			DPA_RX_PRIV_DATA_SIZE, has_timer);
}

static void dpa_rx_fq_init(struct dpa_priv_s *priv, struct list_head *head,
			struct dpa_fq *defq, struct dpa_fq *errq,
			struct dpa_fq *fqs)
{
	if (fqs)
		dpa_setup_ingress_queues(priv, head, fqs);

	/* Only real devices need default/error queues set up */
	if (!priv->mac_dev)
		return;

	if (defq->fqid == 0 && netif_msg_probe(priv))
		cpu_pr_info("Using dynamic RX QM frame queues\n");

	if (priv->shared) {
		dpa_setup_ingress(priv, defq, &rx_shared_fq);
		dpa_setup_ingress(priv, errq, &rx_shared_fq);
	} else {
		dpa_setup_ingress(priv, defq, &rx_private_defq);
		dpa_setup_ingress(priv, errq, &rx_private_errq);
	}
}

static void dpa_tx_fq_init(struct dpa_priv_s *priv, struct list_head *head,
			struct dpa_fq *defq, struct dpa_fq *errq,
			struct dpa_fq *fqs, struct fm_port *port)
{
	if (fqs)
		dpa_setup_egress(priv, head, fqs, port);

	/* Only real devices need default/error queues set up */
	if (!priv->mac_dev)
		return;

	if (defq->fqid == 0 && netif_msg_probe(priv))
		cpu_pr_info("Using dynamic TX QM frame queues\n");

	/* The shared driver doesn't use tx confirmation */
	if (priv->shared) {
		dpa_setup_ingress(priv, defq, &dummyq);
		dpa_setup_ingress(priv, errq, &dummyq);
	} else {
		dpa_setup_ingress(priv, defq, &tx_private_defq);
		dpa_setup_ingress(priv, errq, &tx_private_errq);
	}
}

static int dpa_netdev_init(struct device_node *dpa_node,
		struct net_device *net_dev)
{
	int err;
	const uint8_t *mac_addr;
	struct dpa_priv_s *priv = netdev_priv(net_dev);
	struct device *dev = net_dev->dev.parent;

	if (!priv->mac_dev) {
		/* Get the MAC address */
		mac_addr = of_get_mac_address(dpa_node);
		if (mac_addr == NULL) {
			if (netif_msg_probe(priv))
				dpaa_eth_err(dev, "No MAC address found!\n");
			return -EINVAL;
		}
	} else {
		net_dev->mem_start = priv->mac_dev->res->start;
		net_dev->mem_end = priv->mac_dev->res->end;

		mac_addr = priv->mac_dev->addr;
		net_dev->features |= (NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM);
		net_dev->vlan_features |= (NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM);
#ifdef CONFIG_DPAA_ETH_SG_SUPPORT
		/* Advertise S/G support for MAC-ful, private interfaces */
		if (!priv->shared) {
			net_dev->features |= NETIF_F_SG;
			net_dev->vlan_features |= NETIF_F_SG;
		}
#endif
	}

	memcpy(net_dev->perm_addr, mac_addr, net_dev->addr_len);
	memcpy(net_dev->dev_addr, mac_addr, net_dev->addr_len);

	SET_ETHTOOL_OPS(net_dev, &dpa_ethtool_ops);
	net_dev->needed_headroom = DPA_BP_HEAD;
	net_dev->watchdog_timeo = tx_timeout * HZ / 1000;

	err = register_netdev(net_dev);
	if (err < 0) {
		dpaa_eth_err(dev, "register_netdev() = %d\n", err);
		return err;
	}

#ifdef CONFIG_DEBUG_FS
	priv->debugfs_file = debugfs_create_file(net_dev->name, S_IRUGO,
						 dpa_debugfs_root, net_dev,
						 &dpa_debugfs_fops);
	if (unlikely(priv->debugfs_file == NULL)) {
		cpu_netdev_err(net_dev, "debugfs_create_file(%s/%s/%s) = %d\n",
				powerpc_debugfs_root->d_iname,
				dpa_debugfs_root->d_iname,
				net_dev->name, err);

		unregister_netdev(net_dev);
		return -ENOMEM;
	}
#endif

	return 0;
}

static int dpa_shared_netdev_init(struct device_node *dpa_node,
				struct net_device *net_dev)
{
	net_dev->netdev_ops = &dpa_shared_ops;

	return dpa_netdev_init(dpa_node, net_dev);
}

static int dpa_private_netdev_init(struct device_node *dpa_node,
				struct net_device *net_dev)
{
	int i;
	struct dpa_priv_s *priv = netdev_priv(net_dev);
	struct dpa_percpu_priv_s *percpu_priv;

	/*
	 * Although we access another CPU's private data here
	 * we do it at initialization so it is safe
	 */
	for_each_online_cpu(i) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, i);
		percpu_priv->net_dev = net_dev;

		percpu_priv->dpa_bp = priv->dpa_bp;
		percpu_priv->dpa_bp_count =
			per_cpu_ptr(priv->dpa_bp->percpu_count, i);
#ifdef CONFIG_DPAA_ETH_SG_SUPPORT
		/* init the percpu list and add some skbs */
		skb_queue_head_init(&percpu_priv->skb_list);

		dpa_list_add_skbs(percpu_priv, DEFAULT_SKB_COUNT);
#endif
		netif_napi_add(net_dev, &percpu_priv->napi, dpaa_eth_poll,
			       DPA_NAPI_WEIGHT);
	}

	net_dev->netdev_ops = &dpa_private_ops;

	return dpa_netdev_init(dpa_node, net_dev);
}

int dpa_alloc_pcd_fqids(struct device *dev, uint32_t num,
				uint8_t alignment, uint32_t *base_fqid)
{
	dpaa_eth_crit(dev, "callback not implemented!\n");
	BUG();

	return 0;
}

int dpa_free_pcd_fqids(struct device *dev, uint32_t base_fqid)
{

	dpaa_eth_crit(dev, "callback not implemented!\n");
	BUG();

	return 0;
}

static ssize_t dpaa_eth_show_addr(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dpa_priv_s *priv = netdev_priv(to_net_dev(dev));
	struct mac_device *mac_dev = priv->mac_dev;

	if (mac_dev)
		return sprintf(buf, "%llx",
				(unsigned long long)mac_dev->res->start);
	else
		return sprintf(buf, "none");
}

static DEVICE_ATTR(device_addr, S_IRUGO, dpaa_eth_show_addr, NULL);

static ssize_t dpaa_eth_show_fqids(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dpa_priv_s *priv = netdev_priv(to_net_dev(dev));
	ssize_t bytes = 0;
	int i = 0;
	char *str;
	struct dpa_fq *fq;
	struct dpa_fq *tmp;
	struct dpa_fq *prev = NULL;
	u32 first_fqid = 0;
	u32 last_fqid = 0;
	char *prevstr = NULL;

	list_for_each_entry_safe(fq, tmp, &priv->dpa_fq_list, list) {
		void *dqrr = fq->fq_base.cb.dqrr;
		if (dqrr == ingress_rx_error_dqrr)
			str = "error";
		else if (i == 1 && dqrr == ingress_rx_default_dqrr)
			str = "default";
		else if (dqrr == ingress_rx_error_dqrr ||
				dqrr == ingress_rx_default_dqrr)
			str = "RX";
		else if (dqrr == ingress_tx_default_dqrr)
			str = "TX confirmation";
		else if (dqrr == ingress_tx_error_dqrr)
			str = "TX error";
		else if (dqrr == NULL)
			str = "TX";
		else
			str = "unknown";

		if (prev && (abs(fq->fqid - prev->fqid) != 1 ||
					str != prevstr)) {
			if (last_fqid == first_fqid)
				bytes += sprintf(buf + bytes,
					"%s: %d\n", prevstr, prev->fqid);
			else
				bytes += sprintf(buf + bytes,
					"%s: %d - %d\n", prevstr,
					first_fqid, last_fqid);
		}

		if (prev && abs(fq->fqid - prev->fqid) == 1 && str == prevstr)
			last_fqid = fq->fqid;
		else
			first_fqid = last_fqid = fq->fqid;

		prev = fq;
		prevstr = str;
		i++;
	}

	if (prev) {
		if (last_fqid == first_fqid)
			bytes += sprintf(buf + bytes, "%s: %d\n", prevstr,
					prev->fqid);
		else
			bytes += sprintf(buf + bytes, "%s: %d - %d\n", prevstr,
					first_fqid, last_fqid);
	}

	return bytes;
}

static DEVICE_ATTR(fqids, S_IRUGO, dpaa_eth_show_fqids, NULL);

static ssize_t dpaa_eth_show_dflt_bpid(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t bytes = 0;
	struct dpa_priv_s *priv = netdev_priv(to_net_dev(dev));
	struct dpa_bp *dpa_bp = priv->dpa_bp;

	if (priv->bp_count != 1)
		bytes += snprintf(buf, PAGE_SIZE, "-1\n");
	else
		bytes += snprintf(buf, PAGE_SIZE, "%u\n", dpa_bp->bpid);

	return bytes;
}

static DEVICE_ATTR(dflt_bpid, S_IRUGO, dpaa_eth_show_dflt_bpid, NULL);

static void __devinit dpaa_eth_sysfs_init(struct device *dev)
{
	if (device_create_file(dev, &dev_attr_device_addr))
		dev_err(dev, "Error creating dpaa_eth addr file\n");
	if (device_create_file(dev, &dev_attr_fqids))
		dev_err(dev, "Error creating dpaa_eth fqids file\n");
	if (device_create_file(dev, &dev_attr_dflt_bpid))
		dev_err(dev, "Error creating dpaa_eth dflt_bpid file\n");
}

static int dpaa_eth_add_channel(void *__arg)
{
	const cpumask_t *cpus = qman_affine_cpus();
	u32 pool = QM_SDQCR_CHANNELS_POOL_CONV((u32)(unsigned long)__arg);
	int cpu;

	for_each_cpu(cpu, cpus) {
		set_cpus_allowed_ptr(current, get_cpu_mask(cpu));
		qman_static_dequeue_add(pool);
	}
	return 0;
}
static const struct of_device_id dpa_match[] __devinitconst ;
static int __devinit
dpaa_eth_probe(struct platform_device *_of_dev)
{
	int err, i;
	struct device *dev;
	struct device_node *dpa_node;
	struct dpa_bp *dpa_bp;
	struct dpa_fq *dpa_fq, *tmp;
	struct list_head rxfqlist;
	struct list_head txfqlist;
	size_t count;
	struct net_device *net_dev = NULL;
	struct dpa_priv_s *priv = NULL;
	struct dpa_fq *rxdefault = NULL;
	struct dpa_fq *txdefault = NULL;
	struct dpa_fq *rxerror = NULL;
	struct dpa_fq *txerror = NULL;
	struct dpa_fq *rxextra = NULL;
	struct dpa_fq *txfqs = NULL;
	struct fm_port *rxport = NULL;
	struct fm_port *txport = NULL;
	bool has_timer = FALSE;
	struct mac_device *mac_dev;
	int proxy_enet;
	const struct of_device_id *match;

	dev = &_of_dev->dev;

	dpa_node = dev->of_node;

	match = of_match_device(dpa_match, dev);
	if (!match)
		return -EINVAL;

	if (!of_device_is_available(dpa_node))
		return -ENODEV;

	/*
	 * If it's not an fsl,dpa-ethernet node, we just serve as a proxy
	 * initializer driver, and don't do any linux device setup
	 */
	proxy_enet = strcmp(match->compatible, "fsl,dpa-ethernet");

	/*
	 * Allocate this early, so we can store relevant information in
	 * the private area
	 */
	if (!proxy_enet) {
		net_dev = alloc_etherdev_mq(sizeof(*priv), DPAA_ETH_TX_QUEUES);
		if (!net_dev) {
			dpaa_eth_err(dev, "alloc_etherdev_mq() failed\n");
			return -ENOMEM;
		}

		/* Do this here, so we can be verbose early */
		SET_NETDEV_DEV(net_dev, dev);
		dev_set_drvdata(dev, net_dev);

		priv = netdev_priv(net_dev);
		priv->net_dev = net_dev;

		priv->msg_enable = netif_msg_init(debug, -1);
	}

	/* Get the buffer pools assigned to this interface */
	dpa_bp = dpa_bp_probe(_of_dev, &count);
	if (IS_ERR(dpa_bp)) {
		err = PTR_ERR(dpa_bp);
		goto bp_probe_failed;
	}

	mac_dev = dpa_mac_probe(_of_dev);
	if (IS_ERR(mac_dev)) {
		err = PTR_ERR(mac_dev);
		goto mac_probe_failed;
	} else if (mac_dev) {
		rxport = mac_dev->port_dev[RX];
		txport = mac_dev->port_dev[TX];
	}

	INIT_LIST_HEAD(&rxfqlist);
	INIT_LIST_HEAD(&txfqlist);

	if (rxport)
		err = dpa_fq_probe(_of_dev, &rxfqlist, &rxdefault, &rxerror,
				&rxextra, RX);
	else
		err = dpa_fq_probe(_of_dev, &rxfqlist, NULL, NULL,
				&rxextra, RX);

	if (err < 0)
		goto rx_fq_probe_failed;

	if (txport)
		err = dpa_fq_probe(_of_dev, &txfqlist, &txdefault, &txerror,
				&txfqs, TX);
	else
		err = dpa_fq_probe(_of_dev, &txfqlist, NULL, NULL, &txfqs, TX);

	if (err < 0)
		goto tx_fq_probe_failed;

	/*
	 * Now we have all of the configuration information.
	 * We support a number of configurations:
	 * 1) Private interface - An optimized linux ethernet driver with
	 *    a real network connection.
	 * 2) Shared interface - A device intended for virtual connections
	 *    or for a real interface that is shared between partitions
	 * 3) Proxy initializer - Just configures the MAC on behalf of
	 *    another partition
	 */

	/* bp init */
	if (net_dev) {
		struct task_struct *kth;

		err = dpa_bp_create(net_dev, dpa_bp, count);

		if (err < 0)
			goto bp_create_failed;

		priv->mac_dev = mac_dev;

		priv->channel = dpa_get_channel(dev, dpa_node);

		if (priv->channel < 0) {
			err = priv->channel;
			goto get_channel_failed;
		}

		/* Start a thread that will walk the cpus with affine portals
		 * and add this pool channel to each's dequeue mask. */
		kth = kthread_run(dpaa_eth_add_channel,
				  (void *)(unsigned long)priv->channel,
				  "dpaa_%p:%d", net_dev, priv->channel);
		if (!kth) {
			err = -ENOMEM;
			goto add_channel_failed;
		}

		dpa_rx_fq_init(priv, &rxfqlist, rxdefault, rxerror, rxextra);
		dpa_tx_fq_init(priv, &txfqlist, txdefault, txerror, txfqs,
				txport);

		/* Add the FQs to the interface, and make them active */
		INIT_LIST_HEAD(&priv->dpa_fq_list);

		list_for_each_entry_safe(dpa_fq, tmp, &rxfqlist, list) {
			err = _dpa_fq_alloc(&priv->dpa_fq_list, dpa_fq);
			if (err < 0)
				goto fq_alloc_failed;
		}

		list_for_each_entry_safe(dpa_fq, tmp, &txfqlist, list) {
			err = _dpa_fq_alloc(&priv->dpa_fq_list, dpa_fq);
			if (err < 0)
				goto fq_alloc_failed;
		}

		if (priv->tsu && priv->tsu->valid)
			has_timer = TRUE;
	}

	/* All real interfaces need their ports initialized */
	if (mac_dev) {
		struct fm_port_pcd_param rx_port_pcd_param;

		dpaa_eth_init_rx_port(rxport, dpa_bp, count, rxerror,
				rxdefault, has_timer);
		dpaa_eth_init_tx_port(txport, txerror, txdefault, has_timer);

		rx_port_pcd_param.cba = dpa_alloc_pcd_fqids;
		rx_port_pcd_param.cbf = dpa_free_pcd_fqids;
		rx_port_pcd_param.dev = dev;
		fm_port_pcd_bind(rxport, &rx_port_pcd_param);
	}

	/*
	 * Proxy interfaces need to be started, and the allocated
	 * memory freed
	 */
	if (!net_dev) {
		devm_kfree(&_of_dev->dev, dpa_bp);
		devm_kfree(&_of_dev->dev, rxdefault);
		devm_kfree(&_of_dev->dev, rxerror);
		devm_kfree(&_of_dev->dev, txdefault);
		devm_kfree(&_of_dev->dev, txerror);

		if (mac_dev)
			for_each_port_device(i, mac_dev->port_dev)
				fm_port_enable(mac_dev->port_dev[i]);

		if (mac_dev->adjust_link)
			mac_dev->adjust_link(mac_dev);

		return 0;
	}

	/* Now we need to initialize either a private or shared interface */
	priv->percpu_priv = __alloc_percpu(sizeof(*priv->percpu_priv),
					   __alignof__(*priv->percpu_priv));
	if (priv->percpu_priv == NULL) {
		dpaa_eth_err(dev, "__alloc_percpu() failed\n");
		err = -ENOMEM;
		goto alloc_percpu_failed;
	}

	if (priv->shared)
		err = dpa_shared_netdev_init(dpa_node, net_dev);
	else
		err = dpa_private_netdev_init(dpa_node, net_dev);

	if (err < 0)
		goto netdev_init_failed;

	dpaa_eth_sysfs_init(&net_dev->dev);

#ifdef CONFIG_DPAA_ETH_UNIT_TESTS
	/* The unit test is designed to test private interfaces */
	if (!priv->shared && !tx_unit_test_ran) {
		err = dpa_tx_unit_test(net_dev);

		WARN_ON(err);
	}
#endif

	return 0;

netdev_init_failed:
	if (net_dev)
		free_percpu(priv->percpu_priv);
alloc_percpu_failed:
fq_alloc_failed:
	if (net_dev)
		dpa_fq_free(dev, &priv->dpa_fq_list);
add_channel_failed:
get_channel_failed:
	if (net_dev)
		dpa_bp_free(priv, priv->dpa_bp);
bp_create_failed:
tx_fq_probe_failed:
rx_fq_probe_failed:
mac_probe_failed:
bp_probe_failed:
	dev_set_drvdata(dev, NULL);
	if (net_dev)
		free_netdev(net_dev);

	return err;
}

static const struct of_device_id dpa_match[] __devinitconst = {
	{
		.compatible	= "fsl,dpa-ethernet"
	},
	{
		.compatible	= "fsl,dpa-ethernet-init"
	},
	{}
};
MODULE_DEVICE_TABLE(of, dpa_match);

static int __devexit __cold dpa_remove(struct platform_device *of_dev)
{
	int			err;
	struct device		*dev;
	struct net_device	*net_dev;
	struct dpa_priv_s	*priv;

	dev = &of_dev->dev;
	net_dev = dev_get_drvdata(dev);
	priv = netdev_priv(net_dev);

	dev_set_drvdata(dev, NULL);
	unregister_netdev(net_dev);

	err = dpa_fq_free(dev, &priv->dpa_fq_list);

	free_percpu(priv->percpu_priv);

	dpa_bp_free(priv, priv->dpa_bp);

#ifdef CONFIG_DEBUG_FS
	debugfs_remove(priv->debugfs_file);
#endif

#ifdef CONFIG_FSL_DPA_1588
	if (priv->tsu && priv->tsu->valid)
		dpa_ptp_cleanup(priv);
#endif

	free_netdev(net_dev);

	return err;
}

static struct platform_driver dpa_driver = {
	.driver = {
		.name		= KBUILD_MODNAME,
		.of_match_table	= dpa_match,
		.owner		= THIS_MODULE,
	},
	.probe		= dpaa_eth_probe,
	.remove		= __devexit_p(dpa_remove)
};

static int __init __cold dpa_load(void)
{
	int	 _errno;

	cpu_pr_info(KBUILD_MODNAME ": " DPA_DESCRIPTION " (" VERSION ")\n");

#ifdef CONFIG_DEBUG_FS
	dpa_debugfs_root = debugfs_create_dir(KBUILD_MODNAME,
					      powerpc_debugfs_root);
	if (unlikely(dpa_debugfs_root == NULL)) {
		_errno = -ENOMEM;
		cpu_pr_err(KBUILD_MODNAME ": %s:%hu:%s(): "
			   "debugfs_create_dir(%s/"KBUILD_MODNAME") = %d\n",
			   __file__, __LINE__, __func__,
			   powerpc_debugfs_root->d_iname, _errno);
		goto _return;
	}
#endif

	_errno = platform_driver_register(&dpa_driver);
	if (unlikely(_errno < 0)) {
		cpu_pr_err(KBUILD_MODNAME
			": %s:%hu:%s(): platform_driver_register() = %d\n",
			__file__, __LINE__, __func__, _errno);
		goto _return_debugfs_remove;
	}

	goto _return;

_return_debugfs_remove:
#ifdef CONFIG_DEBUG_FS
	debugfs_remove(dpa_debugfs_root);
#endif
_return:
	cpu_pr_debug(KBUILD_MODNAME ": %s:%s() ->\n", __file__, __func__);

	return _errno;
}
module_init(dpa_load);

static void __exit __cold dpa_unload(void)
{
	cpu_pr_debug(KBUILD_MODNAME ": -> %s:%s()\n", __file__, __func__);

	platform_driver_unregister(&dpa_driver);

#ifdef CONFIG_DEBUG_FS
	debugfs_remove(dpa_debugfs_root);
#endif

	cpu_pr_debug(KBUILD_MODNAME ": %s:%s() ->\n", __file__, __func__);
}
module_exit(dpa_unload);

static int __init fsl_fman_phy_set_max_frm(char *str)
{
	int ret = 0;

	ret = get_option(&str, &fsl_fman_phy_maxfrm);
	if (ret != 1) {
		/* This will only work if CONFIG_EARLY_PRINTK is compiled in,
		 * and something like "earlyprintk=serial,uart0,115200" is
		 * specified in the bootargs */
		printk(KERN_WARNING "No suitable %s=<int> prop in bootargs; "
			"will use the default DPA_MAX_FRM_SIZE (%d) "
			"from Kconfig.\n",
			FSL_FMAN_PHY_MAXFRM_BOOTARG, CONFIG_DPA_MAX_FRM_SIZE);

		fsl_fman_phy_maxfrm = CONFIG_DPA_MAX_FRM_SIZE;
		return 1;
	}

	/* Don't allow invalid bootargs; fallback to the Kconfig value */
	if (fsl_fman_phy_maxfrm < 64 || fsl_fman_phy_maxfrm > 9600) {
		printk(KERN_WARNING "Invalid %s=%d in bootargs, valid range is "
			"64-9600. Falling back to the DPA_MAX_FRM_SIZE (%d) "
			"from Kconfig.\n",
			FSL_FMAN_PHY_MAXFRM_BOOTARG, fsl_fman_phy_maxfrm,
			CONFIG_DPA_MAX_FRM_SIZE);

		fsl_fman_phy_maxfrm = CONFIG_DPA_MAX_FRM_SIZE;
		return 1;
	}

	printk(KERN_INFO "Using fsl_fman_phy_maxfrm=%d from bootargs\n",
		fsl_fman_phy_maxfrm);
	return 0;
}
early_param(FSL_FMAN_PHY_MAXFRM_BOOTARG, fsl_fman_phy_set_max_frm);

static int __init dpa_set_extra_headroom(char *str)
{
	int ret;

	ret = get_option(&str, &dpa_rx_extra_headroom);
	if (ret != 1) {
		printk(KERN_WARNING "No suitable %s=<int> prop in bootargs; "
			"will use the default DPA_EXTRA_HEADROOM (%d) "
			"from Kconfig.\n",
			DPA_EXTRA_HEADROOM_BOOTARG, CONFIG_DPA_EXTRA_HEADROOM);
		dpa_rx_extra_headroom = CONFIG_DPA_EXTRA_HEADROOM;
		return 1;
	}

	/* Don't allow invalid bootargs; fallback to the Kconfig value */
	if (dpa_rx_extra_headroom + DPA_BP_HEAD > DPA_MAX_FD_OFFSET) {
		printk(KERN_WARNING "Invalid %s=%d in bootargs, valid range is "
			"0-%d. Falling back to the Kconfig value (%d).\n",
			DPA_EXTRA_HEADROOM_BOOTARG, dpa_rx_extra_headroom,
			DPA_MAX_FD_OFFSET - DPA_BP_HEAD,
			CONFIG_DPA_EXTRA_HEADROOM);
		dpa_rx_extra_headroom = CONFIG_DPA_EXTRA_HEADROOM;
		return 1;
	}

	return 0;
}

early_param(DPA_EXTRA_HEADROOM_BOOTARG, dpa_set_extra_headroom);
