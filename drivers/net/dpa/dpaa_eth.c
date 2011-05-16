/*
 * Copyright 2008-2011 Freescale Semiconductor Inc.
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
#include <asm/smp.h>		/* get_hard_smp_processor_id() */
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif
#include <linux/fsl_bman.h>

#include "fsl_fman.h"
#include "fm_ext.h"

#include "mac.h"
#include "dpaa_eth.h"
#include "dpaa_1588.h"

#define ARRAY2_SIZE(arr)	(ARRAY_SIZE(arr) * ARRAY_SIZE((arr)[0]))

#define DPA_NETIF_FEATURES	(NETIF_F_HW_QDISC)
#define DEFAULT_COUNT		64
#define DEFAULT_BUF_SIZE DPA_BP_SIZE(FSL_FMAN_PHY_MAXFRM);
#define DPA_MAX_TX_BACKLOG	512
#define DPA_NAPI_WEIGHT		64

#define DPA_BP_REFILL (1 | (smp_processor_id() << 16))
#define DPA_BP_FINE ((smp_processor_id() << 16))
#define DPA_BP_REFILL_NEEDED 1

/* Bootarg used to override the Kconfig DPA_MAX_FRM_SIZE value */
#define FSL_FMAN_PHY_MAXFRM_BOOTARG	"fsl_fman_phy_max_frm"

/*
 * Values for the L3R field of the FM Parse Results
 */
/* L3 Type field: First IP Present IPv4 */
#define FM_L3_PARSE_RESULT_IPV4	0x8000
/* L3 Type field: First IP Present IPv6 */
#define FM_L3_PARSE_RESULT_IPV6	0x4000

/*
 * Values for the L4R field of the FM Parse Results
 */
/* L4 Type field: UDP */
#define FM_L4_PARSE_RESULT_UDP	0x40
/* L4 Type field: TCP */
#define FM_L4_PARSE_RESULT_TCP	0x20

/*
 * FD status field indicating whether the FM Parser has attempted to validate
 * the L4 csum of the frame.
 * Note that having this bit set doesn't necessarily imply that the checksum
 * is valid. One would have to check the parse results to find that out.
 */
#define FM_FD_STAT_L4CV		0x00000004

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
#endif

/*
 * Max frame size configurable from Kconfig or bootargs to avoid
 * allocating oversized (socket)buffers when not using jumbo frames.
 *
 * Set once, at boot-time via the fsl_fman_phy_set_max_frm() callback.
 * Initialization happens long before online_cpu_mask is even set,so it's safe
 * not to use volatile here.
 */
int fsl_fman_phy_maxfrm;

static const char rtx[][3] = {
	[RX] = "RX",
	[TX] = "TX"
};

struct dpa_fq {
	struct qman_fq		 fq_base;
	struct list_head	 list;
	struct net_device	*net_dev;
	bool			 init;
	uint32_t fqid;
	uint32_t flags;
	uint16_t channel;
	uint8_t wq;
};

/* BM */

#ifdef DEBUG
#define GFP_DPA_BP	(GFP_DMA | __GFP_ZERO | GFP_ATOMIC)
#else
#define GFP_DPA_BP	(GFP_DMA | GFP_ATOMIC)
#endif

#define DPA_BP_HEAD (DPA_PRIV_DATA_SIZE + DPA_PARSE_RESULTS_SIZE + \
			DPA_HASH_RESULTS_SIZE + DPA_TIME_STAMP_SIZE)
#define DPA_BP_SIZE(s)	(DPA_BP_HEAD + (s))

#define DPAA_ETH_MAX_PAD (L1_CACHE_BYTES * 8)

#define FM_FD_STAT_ERRORS						\
	(FM_PORT_FRM_ERR_DMA | FM_PORT_FRM_ERR_PHYSICAL	| \
	 FM_PORT_FRM_ERR_SIZE | FM_PORT_FRM_ERR_CLS_DISCARD | \
	 FM_PORT_FRM_ERR_EXTRACTION | FM_PORT_FRM_ERR_NO_SCHEME	| \
	 FM_PORT_FRM_ERR_ILL_PLCR | FM_PORT_FRM_ERR_PRS_TIMEOUT	| \
	 FM_PORT_FRM_ERR_PRS_ILL_INSTRUCT | FM_PORT_FRM_ERR_PRS_HDR_ERR)

static struct dpa_bp *dpa_bp_array[64];

static struct dpa_bp *default_pool;

static struct dpa_bp *dpa_bpid2pool(int bpid)
{
	return dpa_bp_array[bpid];
}

static void dpa_bp_depletion(struct bman_portal	*portal,
		struct bman_pool *pool, void *cb_ctx, int depleted)
{
	if (net_ratelimit())
		pr_err("Invalid Pool depleted notification!\n");
}

static void bmb_free(struct dpa_bp *bp, struct bm_buffer *bmb)
{
	int i;
	struct sk_buff **skbh;
	struct sk_buff *skb;

	for (i = 0; i < 8; i++) {
		dma_addr_t addr = bm_buf_addr(&bmb[i]);
		if (!addr)
			break;

		skbh = (struct sk_buff **)phys_to_virt(addr);
		skb = *skbh;

		dma_unmap_single(bp->dev, addr, bp->size, DMA_FROM_DEVICE);

		dev_kfree_skb(skb);
	}
}

static void dpa_bp_add_8(struct dpa_bp *dpa_bp)
{
	struct bm_buffer bmb[8];
	struct sk_buff **skbh;
	dma_addr_t addr;
	int i;
	struct sk_buff *skb;
	int err;
	unsigned int *count_ptr;

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

		skb = dev_alloc_skb(dpa_bp->skb_size + pad);
		if (unlikely(!skb)) {
			printk(KERN_ERR "dev_alloc_skb() failed\n");
			bm_buffer_set64(&bmb[i], 0);
			break;
		}

		skbh = (struct sk_buff **)(skb->head + pad);
		*skbh = skb;

		addr = dma_map_single(dpa_bp->dev, skb->head + pad,
				dpa_bp->size, DMA_FROM_DEVICE);

		bm_buffer_set64(&bmb[i], addr);
	}

	/* Avoid releasing a completely null buffer; bman_release() requires
	 * at least one buf. */
	if (likely(i)) {
		err = bman_release(dpa_bp->pool, bmb, i, 0);

		if (unlikely(err < 0))
			bmb_free(dpa_bp, bmb);
		else
			*count_ptr += i;
	}
}

static void dpa_make_private_pool(struct dpa_bp *dpa_bp)
{
	int i;

	dpa_bp->percpu_count = __alloc_percpu(sizeof(*dpa_bp->percpu_count),
			__alignof__(*dpa_bp->percpu_count));

	dpa_bp->skb_size = dpa_bp->size;

	/* Give each cpu an allotment of "count" buffers */
	for_each_online_cpu(i) {
		int *thiscount;
		int *countptr;
		int j;
		thiscount = per_cpu_ptr(dpa_bp->percpu_count,
				smp_processor_id());
		countptr = per_cpu_ptr(dpa_bp->percpu_count, i);

		for (j = 0; j < dpa_bp->count; j += 8)
			dpa_bp_add_8(dpa_bp);

		/* Adjust the counts */
		*countptr = j;

		if (countptr != thiscount)
			*thiscount = *thiscount - j;
	}
}

static int dpa_make_shared_pool(struct dpa_bp *bp)
{
	devm_request_mem_region(bp->dev, bp->paddr, bp->size * bp->count,
			KBUILD_MODNAME);
	bp->vaddr = devm_ioremap_prot(bp->dev, bp->paddr,
			bp->size * bp->count, 0);
	if (bp->vaddr == NULL) {
		cpu_pr_err("Could not map memory for pool %d\n", bp->bpid);
		return -EIO;
	}

	return 0;
}

static int __devinit __must_check __attribute__((nonnull))
dpa_bp_alloc(struct dpa_bp *dpa_bp)
{
	int err;
	struct bman_pool_params	 bp_params;
	struct platform_device *pdev;

	BUG_ON(dpa_bp->size == 0);
	BUG_ON(dpa_bp->count == 0);

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

	if (dma_set_mask(&pdev->dev, DMA_BIT_MASK(40)))
		goto pdev_mask_failed;

	dpa_bp->dev = &pdev->dev;

	if (dpa_bp->kernel_pool) {
		dpa_make_private_pool(dpa_bp);
		if (!default_pool)
			default_pool = dpa_bp;
	} else {
		err = dpa_make_shared_pool(dpa_bp);
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
				struct sk_buff **skbh = phys_to_virt(addr);
				struct sk_buff *skb = *skbh;

				dma_unmap_single(bp->dev, addr, bp->size,
						DMA_FROM_DEVICE);

				dev_kfree_skb_any(skb);
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
	/* Set the QMan taildrop threshold high enough to accomodate
	 * one 64k frame, plus an extra (here, 16k) for
	 * other frames awaiting Tx. */
	const u32		 qman_taildrop_threshold = 0x14000;

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
		qm_fqd_taildrop_set(&initfq.fqd.td, qman_taildrop_threshold, 1);
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


static inline ssize_t __const __must_check __attribute__((nonnull))
dpa_fd_length(const struct qm_fd *fd)
{
	return fd->length20;
}

static inline ssize_t __const __must_check __attribute__((nonnull))
dpa_fd_offset(const struct qm_fd *fd)
{
	return fd->offset;
}

static int __must_check __attribute__((nonnull))
dpa_fd_release(const struct net_device *net_dev, const struct qm_fd *fd)
{
	int				 _errno, __errno, i, j;
	const struct dpa_priv_s		*priv;
	const struct qm_sg_entry	*sgt;
	struct dpa_bp		*_dpa_bp, *dpa_bp;
	struct bm_buffer		 _bmb, bmb[8];

	priv = netdev_priv(net_dev);

	_bmb.hi	= fd->addr_hi;
	_bmb.lo	= fd->addr_lo;

	_dpa_bp = dpa_bpid2pool(fd->bpid);
	BUG_ON(IS_ERR(_dpa_bp));

	_errno = 0;
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

			__errno = bman_release(dpa_bp->pool, bmb, j, 0);
			if (unlikely(__errno < 0)) {
				if (netif_msg_drv(priv) && net_ratelimit())
					cpu_netdev_err(net_dev,
						"bman_release(%hu) = %d\n",
						dpa_bp->bpid, _errno);
				if (_errno >= 0)
					_errno = __errno;
			}
		} while (!sgt[i-1].final);
	}

	__errno = bman_release(_dpa_bp->pool, &_bmb, 1, 0);
	if (unlikely(__errno < 0)) {
		if (netif_msg_drv(priv) && net_ratelimit())
			cpu_netdev_err(net_dev, "bman_release(%hu) = %d\n",
					_dpa_bp->bpid, __errno);
		if (_errno >= 0)
			_errno = __errno;
	}

	return _errno;
}

/* net_device */

#define NN_ALLOCATED_SPACE(net_dev) \
		max((size_t)arp_hdr_len(net_dev),  sizeof(struct iphdr))
#define NN_RESERVED_SPACE(net_dev) \
		min((size_t)arp_hdr_len(net_dev),  sizeof(struct iphdr))

#define TT_ALLOCATED_SPACE(net_dev) \
	max(sizeof(struct icmphdr), max(sizeof(struct udphdr), \
		sizeof(struct tcphdr)))
#define TT_RESERVED_SPACE(net_dev) \
	min(sizeof(struct icmphdr), min(sizeof(struct udphdr), \
		sizeof(struct tcphdr)))

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
	const int max_mtu = FSL_FMAN_PHY_MAXFRM - (VLAN_ETH_HLEN + ETH_FCS_LEN);
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

static void __cold dpa_change_rx_flags(struct net_device *net_dev, int flags)
{
	int			 _errno;
	const struct dpa_priv_s	*priv;

	priv = netdev_priv(net_dev);

	if (!priv->mac_dev)
		return;

	if ((flags & IFF_PROMISC) != 0) {
		_errno = priv->mac_dev->change_promisc(priv->mac_dev);
		if (unlikely(_errno < 0) && netif_msg_drv(priv))
			cpu_netdev_err(net_dev,
				       "mac_dev->change_promisc() = %d\n",
				       _errno);
	}
}

static void dpa_set_multicast_list(struct net_device *net_dev)
{
	int _errno;
	struct dpa_priv_s *priv;

	priv = netdev_priv(net_dev);

	if (!priv->mac_dev) {
		if (netif_msg_drv(priv))
			cpu_netdev_warn(net_dev,
					"%s() called on MAC-less interface\n",
					__func__);
		return;
	}

	_errno = priv->mac_dev->set_multi(net_dev);
	if ((_errno < 0) && netif_msg_drv(priv))
		cpu_netdev_err(net_dev, "mac_dev->set_multi() = %d\n", _errno);
}

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
	int *countptr;
	int cache_fudge = (fd->addr_lo - (u32)skb->head) & (PAGE_SIZE - 1);
	int data_start;

	countptr = per_cpu_ptr(bp->percpu_count, smp_processor_id());

	(*countptr)--;

	/*
	 * The skb is currently pointed at head + NET_SKB_PAD. The packet
	 * starts at skb->head + cache_fudge + fd offset.
	 */
	data_start = cache_fudge + dpa_fd_offset(fd) - NET_SKB_PAD;
	skb_put(skb, dpa_fd_length(fd) + data_start);
	skb_pull(skb, data_start);

	dma_unmap_single(bp->dev, qm_fd_addr(fd), bp->size, DMA_FROM_DEVICE);

	return 0;
}

static void _dpa_rx_error(struct net_device *net_dev,
		const struct dpa_priv_s	*priv,
		struct dpa_percpu_priv_s *percpu_priv,
		const struct qm_fd *fd)
{
	int _errno;

	if (netif_msg_hw(priv) && net_ratelimit())
		cpu_netdev_warn(net_dev, "FD status = 0x%08x\n",
				fd->status & FM_FD_STAT_ERRORS);

	percpu_priv->stats.rx_errors++;
	percpu_priv->stats.rx_packets++;
	percpu_priv->stats.rx_bytes += dpa_fd_length(fd);

	_errno = dpa_fd_release(net_dev, fd);
	if (unlikely(_errno < 0)) {
		dump_stack();
		panic("Can't release buffer to the BM during RX\n");
	}
}

static void _dpa_tx_error(struct net_device		*net_dev,
			  const struct dpa_priv_s	*priv,
			  struct dpa_percpu_priv_s	*percpu_priv,
			  const struct qm_fd		*fd)
{
	struct sk_buff *skb;
	struct sk_buff **skbh;
	dma_addr_t addr = qm_fd_addr(fd);
	struct dpa_bp *bp = priv->dpa_bp;

	if (netif_msg_hw(priv) && net_ratelimit())
		cpu_netdev_warn(net_dev, "FD status = 0x%08x\n",
				fd->status & FM_FD_STAT_ERRORS);

	percpu_priv->stats.tx_errors++;

	skbh = (struct sk_buff **)phys_to_virt(addr);
	skb = *skbh;

	dma_unmap_single(bp->dev, addr, bp->size, DMA_TO_DEVICE);

	dev_kfree_skb(skb);
}

static void __hot _dpa_rx(struct net_device *net_dev,
		const struct dpa_priv_s *priv,
		struct dpa_percpu_priv_s *percpu_priv,
		const struct qm_fd *fd)
{
	int _errno;
	int *countptr;
	struct dpa_bp *dpa_bp;
	struct sk_buff *skb;
	struct sk_buff **skbh;
	dma_addr_t addr = qm_fd_addr(fd);

	skbh = (struct sk_buff **)phys_to_virt(addr);

	if (unlikely(fd->status & FM_FD_STAT_ERRORS) != 0) {
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

	skb = *skbh;
	prefetch(skb);

	/* Fill the SKB */
	dpa_process_one(percpu_priv, skb, dpa_bp, fd);

	prefetch(skb_shinfo(skb));

#ifdef CONFIG_FSL_DPA_1588
	if (priv->tsu && priv->tsu->valid)
		dpa_ptp_store_rxstamp(net_dev, skb, fd);
#endif

	skb->protocol = eth_type_trans(skb, net_dev);

	percpu_priv->stats.rx_packets++;
	percpu_priv->stats.rx_bytes += skb->len;

	if (unlikely(skb->len > net_dev->mtu)) {
		if ((skb->protocol != ETH_P_8021Q) ||
				(skb->len > net_dev->mtu + 4)) {
			percpu_priv->stats.rx_dropped++;
			goto drop_large_frame;
		}
	}

	/* Check if the FMan Parser has already validated the L4 csum.
	 * Note: This bit always reads 0 on Rev1 silicon, so the csum
	 * validation will implicitly be done in software. */
	if (fd->status & FM_FD_STAT_L4CV) {
		/* If we're here, the csum must be valid (if it hadn't,
		 * the frame would have been received on the Error FQ,
		 * respectively on the _dpa_rx_error() path). */
		skb->ip_summed = CHECKSUM_UNNECESSARY;
	} else
		skb->ip_summed = CHECKSUM_NONE;

	if (unlikely(netif_receive_skb(skb) == NET_RX_DROP))
		percpu_priv->stats.rx_dropped++;

	net_dev->last_rx = jiffies;

	return;

drop_large_frame:
	countptr = per_cpu_ptr(dpa_bp->percpu_count, smp_processor_id());
	(*countptr)++;
	skb_recycle(skb);
_return_dpa_fd_release:
	_errno = dpa_fd_release(net_dev, fd);
	if (unlikely(_errno < 0)) {
		dump_stack();
		panic("Can't release buffer to the BM during RX\n");
	}
}

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
	int *countptr;

	percpu_priv = container_of(napi, struct dpa_percpu_priv_s, napi);

	countptr = per_cpu_ptr(percpu_priv->dpa_bp->percpu_count,
				smp_processor_id());

	if (*countptr < DEFAULT_COUNT / 4) {
		int i;

		for (i = 0; i < percpu_priv->dpa_bp->count; i += 8)
			dpa_bp_add_8(percpu_priv->dpa_bp);
	}

	if (cleaned < budget) {
		int tmp;
		napi_complete(napi);
		tmp = qman_irqsource_add(QM_PIRQ_DQRI);
		BUG_ON(tmp);
	}

	return cleaned;
}

static void __hot _dpa_tx(struct net_device		*net_dev,
			  const struct dpa_priv_s	*priv,
			  struct dpa_percpu_priv_s	*percpu_priv,
			  const struct qm_fd		*fd)
{
	struct sk_buff **skbh;
	struct sk_buff	*skb;
	dma_addr_t addr = qm_fd_addr(fd);
	struct dpa_bp *bp = priv->dpa_bp;

	if (unlikely(fd->status & FM_FD_STAT_ERRORS) != 0) {
		if (netif_msg_hw(priv) && net_ratelimit())
			cpu_netdev_warn(net_dev, "FD status = 0x%08x\n",
					fd->status & FM_FD_STAT_ERRORS);

		percpu_priv->stats.tx_errors++;
	}

	skbh = (struct sk_buff **)phys_to_virt(addr);
	skb = *skbh;

#ifdef CONFIG_FSL_DPA_1588
	if (priv->tsu && priv->tsu->valid && dpa_ptp_do_txstamp(skb))
		dpa_ptp_store_txstamp(net_dev, skb, fd);
#endif

	dma_unmap_single(bp->dev, addr, bp->size, DMA_TO_DEVICE);

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
static inline int dpa_enable_tx_csum(struct dpa_priv_s *dpa_priv,
	struct sk_buff *skb, struct qm_fd *fd, char *parse_results)
{
	t_FmPrsResult *parse_result = NULL;
	struct iphdr *iph = NULL;
	struct ipv6hdr *ipv6h = NULL;
	int l4_proto;
	int retval = 0;

	if (!dpa_priv->mac_dev || skb->ip_summed != CHECKSUM_PARTIAL)
		return 0;

	/* Note: L3 csum seems to be already computed in sw, but we can't choose
	 * L4 alone from the FM configuration anyway. */

	/* Fill in some fields of the Parse Results array, so the FMan
	 * can find them as if they came from the FMan Parser. */
	parse_result = (t_FmPrsResult *)parse_results;

	/* Fill in the relevant L3 parse result fields
	 * and read the L4 protocol type */
	switch (ntohs(skb->protocol)) {
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
		if (netif_msg_tx_err(dpa_priv) && net_ratelimit())
			printk(KERN_ALERT
				"%s:%hu:%s(): can't compute HW csum "
				"for L3 proto 0x%x\n",
				__file__, __LINE__, __func__,
				ntohs(skb->protocol));
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
		if (netif_msg_tx_err(dpa_priv) && net_ratelimit())
			printk(KERN_ALERT
				"%s:%hu:%s(): can't compute HW csum "
				"for L4 proto 0x%x\n",
				__file__, __LINE__, __func__, l4_proto);
		retval = -EIO;
		goto return_error;
	}

	/* - Can't assume the MAC header was either set, or zeroed.
	 * - skb->data always starts with the MAC header. */
	skb_set_mac_header(skb, 0);

	/* At index 0 is IPOffset_1 as defined in the Parse Results */
	parse_result->ip_off[0] = skb_network_header(skb) -
		skb_mac_header(skb);
	parse_result->l4_off = skb_transport_header(skb) -
		skb_mac_header(skb);

	/* Enable L3 (and L4, if TCP or UDP) HW checksum. */
	fd->cmd |= FM_FD_CMD_RPD | FM_FD_CMD_DTC;

return_error:
	return retval;
}

static inline int __hot dpa_xmit(struct dpa_priv_s *priv,
			struct dpa_percpu_priv_s *percpu, int queue,
			struct qm_fd *fd)
{
	int err;

	prefetchw(&percpu->start_tx);
	err = qman_enqueue(priv->egress_fqs[queue], fd, 0);
	if (unlikely(err < 0)) {
		if (netif_msg_tx_err(priv) && net_ratelimit())
			cpu_netdev_err(priv->net_dev, "qman_enqueue() = %d\n",
					err);
		percpu->stats.tx_errors++;
		percpu->stats.tx_fifo_errors++;
		return err;
	}

	percpu->stats.tx_packets++;
	percpu->stats.tx_bytes += dpa_fd_length(fd);

	return NETDEV_TX_OK;
}

static int __hot dpa_shared_tx(struct sk_buff *skb, struct net_device *net_dev)
{
	struct dpa_bp *dpa_bp;
	struct bm_buffer bmb;
	struct dpa_percpu_priv_s *percpu_priv;
	struct dpa_priv_s *priv;
	struct device *dev;
	struct qm_fd fd;
	int queue_mapping;
	int err;
	void *dpa_bp_vaddr;

	priv = netdev_priv(net_dev);
	percpu_priv = per_cpu_ptr(priv->percpu_priv, smp_processor_id());
	dev = net_dev->dev.parent;

	memset(&fd, 0, sizeof(fd));
	fd.format = qm_fd_contig;

	queue_mapping = skb_get_queue_mapping(skb);

	dpa_bp = dpa_size2pool(priv, skb_headlen(skb));
	if (unlikely(IS_ERR(dpa_bp))) {
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

	dpa_bp_vaddr = dpa_phys2virt(dpa_bp, bm_buf_addr(&bmb));

	/* Copy the packet payload */
	skb_copy_from_linear_data(skb, dpa_bp_vaddr + dpa_fd_offset(&fd),
		dpa_fd_length(&fd));

	/* Enable L3/L4 hardware checksum computation, if applicable */
	err = dpa_enable_tx_csum(priv, skb, &fd,
		dpa_bp_vaddr + DPA_PRIV_DATA_SIZE);
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

	return err;
}

static int __hot dpa_tx(struct sk_buff *skb, struct net_device *net_dev)
{
	struct dpa_priv_s	*priv;
	struct device		*dev;
	struct qm_fd		 fd;
	unsigned int	headroom;
	struct dpa_percpu_priv_s *percpu_priv;
	struct sk_buff **skbh;
	dma_addr_t addr;
	struct dpa_bp *dpa_bp;
	int queue_mapping;
	int *countptr;
	int err;
	int cache_fudge;
	int needed_headroom;

	priv = netdev_priv(net_dev);
	percpu_priv = per_cpu_ptr(priv->percpu_priv, smp_processor_id());
	dev = net_dev->dev.parent;

	memset(&fd, 0, sizeof(fd));
	fd.format = qm_fd_contig;

	headroom = skb_headroom(skb);
	queue_mapping = skb_get_queue_mapping(skb);

	needed_headroom = (DPA_PRIV_DATA_SIZE + DPA_PARSE_RESULTS_SIZE +
				sizeof(skbh) + DPA_TIME_STAMP_SIZE);

	if (headroom < needed_headroom) {
		struct sk_buff *skb_new;

		skb_new = skb_realloc_headroom(skb, DPA_BP_HEAD);
		if (!skb_new) {
			percpu_priv->stats.tx_errors++;
			kfree_skb(skb);
			return NETDEV_TX_OK;
		}
		kfree_skb(skb);
		skb = skb_new;
		headroom = skb_headroom(skb);
	}

	if (skb_cloned(skb))
		skb = skb_unshare(skb, GFP_ATOMIC);

	cache_fudge = round_down(headroom - needed_headroom, L1_CACHE_BYTES);

	skbh = (struct sk_buff **)(skb->head + cache_fudge);
	*skbh = skb;

	dpa_bp = priv->dpa_bp;

	countptr = per_cpu_ptr(dpa_bp->percpu_count, smp_processor_id());

	/* Enable L3/L4 hardware checksum computation.
	 *
	 * We must do this before dma_map_single(DMA_TO_DEVICE), because we may
	 * need to write into the skb. */
	err = dpa_enable_tx_csum(priv, skb, &fd,
			((char *)skbh) + DPA_PRIV_DATA_SIZE);

	if (unlikely(err < 0)) {
		if (netif_msg_tx_err(priv) && net_ratelimit())
			cpu_netdev_err(net_dev, "HW csum error: %d\n", err);
		percpu_priv->stats.tx_errors++;
		goto l3_l4_csum_failed;
	}

	addr = dma_map_single(dpa_bp->dev, skbh, dpa_bp->size, DMA_TO_DEVICE);
	if (unlikely(addr == 0)) {
		if (netif_msg_tx_err(priv)  && net_ratelimit())
			cpu_netdev_err(net_dev, "dma_map_single() failed\n");
		err = -EIO;
		goto dma_map_failed;
	}

	fd.addr_hi = upper_32_bits(addr);
	fd.addr_lo = lower_32_bits(addr);
	fd.length20 = skb->len;
	fd.offset = headroom - cache_fudge;

#ifdef CONFIG_FSL_DPA_1588
	if (priv->tsu && priv->tsu->valid && dpa_ptp_do_txstamp(skb))
		fd.cmd |= FM_FD_CMD_UPD;
#endif

	if (likely(skb_is_recycleable(skb, dpa_bp->skb_size)
			&& (*countptr + 1 <= dpa_bp->count))) {
		fd.cmd |= FM_FD_CMD_FCO;
		fd.bpid = dpa_bp->bpid;
		skb_recycle(skb);
		skb = NULL;
		(*countptr)++;
		percpu_priv->tx_returned++;
	}

	err = dpa_xmit(priv, percpu_priv, queue_mapping, &fd);

	if (unlikely(err < 0)) {
		if (fd.cmd & FM_FD_CMD_FCO)
			(*countptr)--;
		goto xmit_failed;
	}

	net_dev->trans_start = jiffies;

	return NETDEV_TX_OK;

xmit_failed:
	dma_unmap_single(dev, addr, dpa_bp->size, DMA_TO_DEVICE);
l3_l4_csum_failed:
dma_map_failed:
	if (skb)
		dev_kfree_skb(skb);

	return err;
}

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

	if (dpaa_eth_napi_schedule(percpu_priv))
		return qman_cb_dqrr_stop;

	_dpa_rx_error(net_dev, priv, percpu_priv, &dq->fd);

	return qman_cb_dqrr_consume;
}

static enum qman_cb_dqrr_result __hot
shared_rx_dqrr(struct qman_portal *portal, struct qman_fq *fq,
		const struct qm_dqrr_entry *dq)
{
	struct net_device		*net_dev;
	struct dpa_priv_s		*priv;
	struct dpa_percpu_priv_s	*percpu_priv;
	int err;
	const struct qm_fd *fd = &dq->fd;
	struct dpa_bp *dpa_bp;
	size_t size;
	struct sk_buff *skb;

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

	percpu_priv->stats.rx_packets++;
	percpu_priv->stats.rx_bytes += dpa_fd_length(fd);

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

	/* Fill the SKB */
	memcpy(skb_put(skb, dpa_fd_length(fd)),
			dpa_phys2virt(dpa_bp, qm_fd_addr(fd)) +
			dpa_fd_offset(fd), dpa_fd_length(fd));

	skb->protocol = eth_type_trans(skb, net_dev);

	if (unlikely(skb->len > net_dev->mtu)) {
		if ((skb->protocol != ETH_P_8021Q) ||
				(skb->len > net_dev->mtu + 4)) {
			percpu_priv->stats.rx_dropped++;
			dev_kfree_skb_any(skb);
			goto out;
		}
	}

	if (unlikely(netif_rx(skb) != NET_RX_SUCCESS))
		percpu_priv->stats.rx_dropped++;

	net_dev->last_rx = jiffies;

out:
	err = dpa_fd_release(net_dev, fd);
	if (unlikely(err < 0)) {
		dump_stack();
		panic("Can't release buffer to the BM during RX\n");
	}

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

	_dpa_rx(net_dev, priv, percpu_priv, &dq->fd);

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

	if (dpaa_eth_napi_schedule(percpu_priv))
		return qman_cb_dqrr_stop;

	_dpa_tx_error(net_dev, priv, percpu_priv, &dq->fd);

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

	percpu_priv->tx_confirm++;

	if (dpaa_eth_napi_schedule(percpu_priv))
		return qman_cb_dqrr_stop;

	_dpa_tx(net_dev, priv, percpu_priv, &dq->fd);

	return qman_cb_dqrr_consume;
}

static void shared_ern(struct qman_portal	*portal,
		       struct qman_fq		*fq,
		       const struct qm_mr_entry	*msg)
{
	struct net_device *net_dev;
	const struct dpa_priv_s	*priv;
	int err;
	struct dpa_percpu_priv_s *percpu_priv;
	struct dpa_fq *dpa_fq = (struct dpa_fq *)fq;

	net_dev = dpa_fq->net_dev;
	priv = netdev_priv(net_dev);
	percpu_priv = per_cpu_ptr(priv->percpu_priv, smp_processor_id());

	err = dpa_fd_release(net_dev, &msg->ern.fd);
	if (unlikely(err < 0)) {
		dump_stack();
		panic("Can't release buffer to the BM during a TX\n");
	}

	percpu_priv->stats.tx_dropped++;
	percpu_priv->stats.tx_fifo_errors++;
}

static void egress_ern(struct qman_portal	*portal,
		       struct qman_fq		*fq,
		       const struct qm_mr_entry	*msg)
{
	struct net_device	*net_dev;
	const struct dpa_priv_s	*priv;
	struct sk_buff *skb;
	struct sk_buff **skbh;
	struct dpa_percpu_priv_s	*percpu_priv;
	dma_addr_t addr = qm_fd_addr(&msg->ern.fd);
	struct dpa_bp *bp;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);
	bp = priv->dpa_bp;
	percpu_priv = per_cpu_ptr(priv->percpu_priv, smp_processor_id());

	skbh = (struct sk_buff **)phys_to_virt(addr);
	skb = *skbh;

	dma_unmap_single(bp->dev, addr, bp->size, DMA_TO_DEVICE);

	dev_kfree_skb_any(skb);

	percpu_priv->stats.tx_dropped++;
	percpu_priv->stats.tx_fifo_errors++;
}

static const struct qman_fq rx_shared_fq __devinitconst = {
		.cb = {shared_rx_dqrr, NULL, NULL, NULL}
};
static const struct qman_fq rx_private_defq __devinitconst = {
		.cb = {ingress_rx_default_dqrr, NULL, NULL, NULL}
};
static const struct qman_fq rx_private_errq __devinitconst = {
		.cb = {ingress_rx_error_dqrr, NULL, NULL, NULL}
};
static const struct qman_fq tx_private_defq __devinitconst = {
		.cb = {ingress_tx_default_dqrr, NULL, NULL, NULL}
};
static const struct qman_fq tx_private_errq __devinitconst = {
		.cb = {ingress_tx_error_dqrr, NULL, NULL, NULL}
};
static const struct qman_fq dummyq __devinitconst = {
		.cb = {NULL, NULL, NULL, NULL}
};
static const struct qman_fq private_egress_fq __devinitconst = {
	.cb = {NULL, egress_ern, NULL, NULL}
};
static const struct qman_fq shared_egress_fq __devinitconst = {
	.cb = {NULL, shared_ern, NULL, NULL}
};

static int __cold dpa_start(struct net_device *net_dev)
{
	int err, i;
	struct dpa_priv_s *priv;
	struct mac_device *mac_dev;

	priv = netdev_priv(net_dev);
	mac_dev = priv->mac_dev;

	if (!mac_dev)
		goto no_mac;

	if (priv->tsu && priv->tsu->valid) {
		if (mac_dev->fm_rtc_enable)
			mac_dev->fm_rtc_enable(net_dev);
		if (mac_dev->ptp_enable)
			mac_dev->ptp_enable(mac_dev);
	}

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

	if (priv->tsu && priv->tsu->valid) {
		if (mac_dev->fm_rtc_disable)
			mac_dev->fm_rtc_disable(net_dev);
		if (mac_dev->ptp_disable)
			mac_dev->ptp_disable(mac_dev);
	}

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
dpa_bp_probe(struct of_device *_of_dev, size_t *count)
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
	phandle_prop = of_get_property(_of_dev->node,
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

		bpool_cfg = of_get_property(dev_node, "fsl,bpool-cfg", &lenp);
		if (bpool_cfg && (lenp == (2 * ns + na) * sizeof(*bpool_cfg))) {
			dpa_bp[i].count	= of_read_number(bpool_cfg, ns);
			dpa_bp[i].size	= of_read_number(bpool_cfg + ns, ns);
			dpa_bp[i].paddr	=
				of_read_number(bpool_cfg + 2 * ns, na);

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
		dpa_bp->count = DEFAULT_COUNT;
		dpa_bp->size = DEFAULT_BUF_SIZE;
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
			cpu_netdev_info(net_dev,
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
dpa_mac_probe(struct of_device *_of_dev)
{
	struct device		*dpa_dev, *dev;
	struct device_node	*mac_node;
	int			 lenp;
	const phandle		*phandle_prop;
	struct of_device	*of_dev;
	struct mac_device	*mac_dev;
#ifdef CONFIG_FSL_DPA_1588
	struct net_device	*net_dev = NULL;
	struct dpa_priv_s	*priv = NULL;
	struct device_node	*timer_node;
#endif

	phandle_prop = of_get_property(_of_dev->node, "fsl,fman-mac", &lenp);
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
	if (phandle_prop) {
		if ((mac_dev->phy_if != PHY_INTERFACE_MODE_SGMII) ||
			((mac_dev->phy_if == PHY_INTERFACE_MODE_SGMII) &&
			 (mac_dev->speed == SPEED_1000))) {
			timer_node = of_find_node_by_phandle(*phandle_prop);
			if (timer_node) {
				net_dev = dev_get_drvdata(dpa_dev);
				priv = netdev_priv(net_dev);
				if (!dpa_ptp_init(priv))
					dpaa_eth_info(dev, "%s: ptp-timer "
					    "enabled\n", mac_node->full_name);
			}
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
	int *countptr;
	unsigned int count_total = 0;

	BUG_ON(offset == NULL);

	priv = netdev_priv((struct net_device *)file->private);

	dpa_bp = priv->dpa_bp;

	memset(&total, 0, sizeof(total));

	seq_printf(file, "\tirqs\trx\trecycle\tconfirm\tbp count\tneed\n");
	for_each_online_cpu(i) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, i);
		countptr = per_cpu_ptr(dpa_bp->percpu_count, i);

		total.in_interrupt += percpu_priv->in_interrupt;
		total.ingress_calls += percpu_priv->stats.rx_packets;
		total.tx_returned += percpu_priv->tx_returned;
		total.tx_confirm += percpu_priv->tx_confirm;
		count_total += *countptr;

		seq_printf(file, "%hu/%hu\t%u\t%lu\t%u\t%u\t%d\n",
				get_hard_smp_processor_id(i), i,
				percpu_priv->in_interrupt,
				percpu_priv->stats.rx_packets,
				percpu_priv->tx_returned,
				percpu_priv->tx_confirm,
				*countptr);
	}
	seq_printf(file, "Total\t%u\t%u\t%u\t%u\t%d\n",
			total.in_interrupt,
			total.ingress_calls,
			total.tx_returned,
			total.tx_confirm, count_total);

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
	.ndo_change_rx_flags = dpa_change_rx_flags,
	.ndo_tx_timeout = dpa_timeout,
	.ndo_get_stats = dpa_get_stats,
	.ndo_set_mac_address = dpa_set_mac_address,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_select_queue = dpa_select_queue,
	.ndo_change_mtu = dpa_change_mtu,
	.ndo_set_multicast_list = dpa_set_multicast_list,
};

static const struct net_device_ops dpa_shared_ops = {
	.ndo_open = dpa_start,
	.ndo_start_xmit = dpa_shared_tx,
	.ndo_stop = dpa_stop,
	.ndo_change_rx_flags = dpa_change_rx_flags,
	.ndo_tx_timeout = dpa_timeout,
	.ndo_get_stats = dpa_get_stats,
	.ndo_set_mac_address = dpa_set_mac_address,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_set_multicast_list = dpa_set_multicast_list,
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
dpa_fq_probe(struct of_device *_of_dev, struct list_head *list,
		struct dpa_fq **defq, struct dpa_fq **errq,
		struct dpa_fq **fqs, int ptype)
{
	struct device_node *np = _of_dev->node;
	struct device *dev = &_of_dev->dev;
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
		}

		/* The second queue is the the Default queue */
		if (i == 1 && defq) {
			*defq = dpa_fq;

			if (fqids[i].count != 1) {
				dpaa_eth_err(dev, "Too many default queues!\n");
				err = -EINVAL;
				goto invalid_default_queues;
			}
		}

		/*
		 * All subsequent queues are gathered together.
		 * The first 8 will be used by the private linux interface
		 * if these are TX queues
		 */
		if (i == 2 || (!errq && i == 0 && fqs))
			*fqs = dpa_fq;

#warning We lost the 8-queue enforcement

#define DPA_NUM_WQS 8
		for (j = 0; j < fqids[i].count; j++) {
			dpa_fq[j].fqid = fqids[i].start ?
				fqids[i].start + j : 0;
			dpa_fq[j].wq = dpa_fq[j].fqid ?
				dpa_fq[j].fqid % DPA_NUM_WQS : DPA_NUM_WQS - 1;
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
	u32 fqid = 0;
	int portals[NR_CPUS];
	int cpu;
	int i = 0;

	/* Make a list of the available portals */
	for_each_online_cpu(cpu) {
		portals[i++] = cpu;
	}

	i = 0;

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
		i = (i + 1) % num_online_cpus();

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
			has_timer);
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
			has_timer);
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

	net_dev->features |= DPA_NETIF_FEATURES;

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

	for_each_online_cpu(i) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, i);
		percpu_priv->net_dev = net_dev;

		percpu_priv->dpa_bp = priv->dpa_bp;
		netif_napi_add(net_dev, &percpu_priv->napi, dpaa_eth_poll,
			       DPA_NAPI_WEIGHT);
	}

	net_dev->netdev_ops = &dpa_private_ops;

	return dpa_netdev_init(dpa_node, net_dev);
}

static int dpa_alloc_pcd_fqids(struct device *dev, uint32_t num,
				uint8_t alignment, uint32_t *base_fqid)
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

	if (last_fqid == first_fqid)
		bytes += sprintf(buf + bytes, "%s: %d\n", prevstr, prev->fqid);
	else
		bytes += sprintf(buf + bytes, "%s: %d - %d\n", prevstr,
				first_fqid, last_fqid);

	return bytes;
}

static DEVICE_ATTR(fqids, S_IRUGO, dpaa_eth_show_fqids, NULL);


static void __devinit dpaa_eth_sysfs_init(struct device *dev)
{
	if (device_create_file(dev, &dev_attr_device_addr))
		dev_err(dev, "Error creating dpaa_eth addr file\n");
	if (device_create_file(dev, &dev_attr_fqids))
		dev_err(dev, "Error creating dpaa_eth fqids file\n");
}
static int __devinit
dpaa_eth_probe(struct of_device *_of_dev, const struct of_device_id *match)
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
	struct mac_device *mac_dev;
	int proxy_enet;

	dev = &_of_dev->dev;

	dpa_node = _of_dev->node;

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
		err = dpa_bp_create(net_dev, dpa_bp, count);

		if (err < 0)
			goto bp_create_failed;

		priv->mac_dev = mac_dev;

		priv->channel = dpa_get_channel(dev, dpa_node);

		if (priv->channel < 0) {
			err = priv->channel;
			goto get_channel_failed;
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

	}

	/* All real interfaces need their ports initialized */
	if (mac_dev) {
		struct fm_port_pcd_param rx_port_pcd_param;
		bool has_timer = FALSE;

		if (priv->tsu && priv->tsu->valid)
			has_timer = TRUE;

		dpaa_eth_init_rx_port(rxport, dpa_bp, count, rxerror,
				rxdefault, has_timer);
		dpaa_eth_init_tx_port(txport, txerror, txdefault, has_timer);

		rx_port_pcd_param.cb = dpa_alloc_pcd_fqids;
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

		for_each_port_device(i, mac_dev->port_dev)
			fm_port_enable(mac_dev->port_dev[i]);

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

	return 0;

netdev_init_failed:
	if (net_dev)
		free_percpu(priv->percpu_priv);
alloc_percpu_failed:
fq_alloc_failed:
	if (net_dev)
		dpa_fq_free(dev, &priv->dpa_fq_list);
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

static int __devexit __cold dpa_remove(struct of_device *of_dev)
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

static struct of_platform_driver dpa_driver = {
	.name		= KBUILD_MODNAME,
	.match_table	= dpa_match,
	.owner		= THIS_MODULE,
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

	_errno = of_register_platform_driver(&dpa_driver);
	if (unlikely(_errno < 0)) {
		cpu_pr_err(KBUILD_MODNAME
			": %s:%hu:%s(): of_register_platform_driver() = %d\n",
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

	of_unregister_platform_driver(&dpa_driver);

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
	if (ret != 1 && ret != 2) {
		/* This will only work if CONFIG_EARLY_PRINTK is compiled in,
		 * and something like "earlyprintk=serial,uart0,115200" is
		 * specified in the bootargs */
		printk(KERN_INFO "No suitable %s=<int> prop in bootargs; "
			"will use the default DPA_MAX_FRM_SIZE from Kconfig.\n",
			FSL_FMAN_PHY_MAXFRM_BOOTARG);

		/* Reset it to the default (invalid) value,
		 * in case get_option() parsed a starting range. */
		fsl_fman_phy_maxfrm = 0;
		return 1;
	}

	/* Don't allow invalid bootargs */
	if (fsl_fman_phy_maxfrm < 64 || fsl_fman_phy_maxfrm > 9600) {
		printk(KERN_WARNING "Invalid %s=%d in bootargs, valid range is "
			"64-9600.\n", FSL_FMAN_PHY_MAXFRM_BOOTARG,
			fsl_fman_phy_maxfrm);

		fsl_fman_phy_maxfrm = 0;
		return 1;
	}

	return 0;
}
early_param(FSL_FMAN_PHY_MAXFRM_BOOTARG, fsl_fman_phy_set_max_frm);
