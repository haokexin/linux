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

#ifndef __DPA_H
#define __DPA_H

#include <linux/ethtool.h>	/* struct ethtool_ops */
#include <linux/netdevice.h>
#include <linux/list.h>		/* struct list_head */
#include <linux/workqueue.h>	/* struct work_struct */
#include <linux/skbuff.h>
#include <linux/hardirq.h>
#include <linux/if_vlan.h>	/* vlan_eth_hdr */
#include <linux/ip.h>		/* ip_hdr */
#include <linux/ipv6.h>		/* ipv6_hdr */
#ifdef CONFIG_DEBUG_FS
#include <linux/dcache.h>	/* struct dentry */
#endif

#include <linux/fsl_qman.h>	/* struct qman_fq */

#include "dpaa_eth-common.h"

#include "fsl_fman.h"
#include "fm_ext.h"
#include "fm_port_ext.h" /* FM_PORT_FRM_ERR_* */

#include "mac.h"		/* struct mac_device */

#ifdef CONFIG_DPAA_ETH_SG_SUPPORT
#define dpa_get_rx_extra_headroom() fm_get_rx_extra_headroom()
#else
#define dpa_get_rx_extra_headroom() dpa_rx_extra_headroom
#endif
#define dpa_get_max_frm() dpa_max_frm

/*
 * Currently we have the same max_frm on all interfaces, so these macros
 * don't get a net_device argument. This will change in the future.
 */
#define dpa_get_min_mtu()	64
#define dpa_get_max_mtu()	\
	(dpa_get_max_frm() - (VLAN_ETH_HLEN + ETH_FCS_LEN))

#define DPA_RX_PRIV_DATA_SIZE   (DPA_TX_PRIV_DATA_SIZE + \
					dpa_get_rx_extra_headroom())
/* number of Tx queues to FMan */
#define DPAA_ETH_TX_QUEUES	8
#define DPAA_ETH_RX_QUEUES	128

#if defined(CONFIG_DPAA_FMAN_UNIT_TESTS)
/*TODO: temporary for fman pcd testing */
#define FMAN_PCD_TESTS_MAX_NUM_RANGES	20
#endif

/* return codes for the dpaa-eth hooks */
enum dpaa_eth_hook_result {
	/* fd/skb was retained by the hook.
	 *
	 * On the Rx path, this means the Ethernet driver will _not_
	 * deliver the skb to the stack. Instead, the hook implementation
	 * is expected to properly dispose of the skb.
	 *
	 * On the Tx path, the Ethernet driver's dpa_tx() function will
	 * immediately return NETDEV_TX_OK. The hook implementation is expected
	 * to free the skb. *DO*NOT* release it to BMan, or enqueue it to FMan,
	 * unless you know exactly what you're doing!
	 *
	 * On the confirmation/error paths, the Ethernet driver will _not_
	 * perform any fd cleanup, nor update the interface statistics.
	 */
	DPAA_ETH_STOLEN,
	/*
	 * fd/skb was returned to the Ethernet driver for regular processing.
	 * The hook is not allowed to, for instance, reallocate the skb (as if
	 * by linearizing, copying, cloning or reallocating the headroom).
	 */
	DPAA_ETH_CONTINUE
};

typedef enum dpaa_eth_hook_result (*dpaa_eth_ingress_hook_t)(
		struct sk_buff *skb, struct net_device *net_dev, u32 fqid);
typedef enum dpaa_eth_hook_result (*dpaa_eth_egress_hook_t)(
		struct sk_buff *skb, struct net_device *net_dev);
typedef enum dpaa_eth_hook_result (*dpaa_eth_confirm_hook_t)(
		struct net_device *net_dev, const struct qm_fd *fd, u32 fqid);

/*
 * Various hooks used for unit-testing and/or fastpath optimizations.
 * Currently only one set of such hooks is supported.
 */
struct dpaa_eth_hooks_s {
	/*
	 * Invoked on the Tx private path, immediately after receiving the skb
	 * from the stack.
	 */
	dpaa_eth_egress_hook_t	tx;

	/*
	 * Invoked on the Rx private path, right before passing the skb
	 * up the stack. At that point, the packet's protocol id has already
	 * been set. The skb's data pointer is now at the L3 header, and
	 * skb->mac_header points to the L2 header. skb->len has been adjusted
	 * to be the length of L3+payload (i.e., the length of the
	 * original frame minus the L2 header len).
	 * For more details on what the skb looks like, see eth_type_trans().
	 */
	dpaa_eth_ingress_hook_t	rx_default;

	/* Driver hook for the Rx error private path. */
	dpaa_eth_confirm_hook_t	rx_error;
	/* Driver hook for the Tx confirmation private path. */
	dpaa_eth_confirm_hook_t	tx_confirm;
	/* Driver hook for the Tx error private path. */
	dpaa_eth_confirm_hook_t	tx_error;
};

void fsl_dpaa_eth_set_hooks(struct dpaa_eth_hooks_s *hooks);

#define DPA_BP_HEAD (DPA_TX_PRIV_DATA_SIZE + DPA_PARSE_RESULTS_SIZE + \
			DPA_HASH_RESULTS_SIZE)
#define DPA_BP_SIZE(s)	(DPA_BP_HEAD + dpa_get_rx_extra_headroom() + (s))

#define DPA_SGT_MAX_ENTRIES 16 /* maximum number of entries in SG Table */

#ifdef CONFIG_DPAA_ETH_SG_SUPPORT
#define DEFAULT_SKB_COUNT 64 /* maximum number of SKBs in each percpu list */
/*
 * We may want this value configurable. Must be <= PAGE_SIZE
 * A lower value may help with recycling rates, at least on forwarding
 */
#define DEFAULT_BUF_SIZE	PAGE_SIZE
/*
 * Default amount data to be copied from the beginning of a frame into the
 * linear part of the skb, in case we aren't using the hardware parser.
 */
#define DPA_COPIED_HEADERS_SIZE 128

#else
/*
 * Default buffer size is based on L2 MAX_FRM value, minus the FCS which is
 * stripped down by hardware.
 */
#define DEFAULT_BUF_SIZE DPA_BP_SIZE(dpa_get_max_frm() - ETH_FCS_LEN)
#endif /* CONFIG_DPAA_ETH_SG_SUPPORT */

/*
 * Values for the L3R field of the FM Parse Results
 */
/* L3 Type field: First IP Present IPv4 */
#define FM_L3_PARSE_RESULT_IPV4	0x8000
/* L3 Type field: First IP Present IPv6 */
#define FM_L3_PARSE_RESULT_IPV6	0x4000

/*
 * Values for the L4R field of the FM Parse Results
 * See $8.8.4.7.20 - L4 HXS - L4 Results from DPAA-Rev2 Reference Manual.
 */
/* L4 Type field: UDP */
#define FM_L4_PARSE_RESULT_UDP	0x40
/* L4 Type field: TCP */
#define FM_L4_PARSE_RESULT_TCP	0x20
/*
 * This includes L4 checksum errors, but also other errors that the Hard Parser
 * can detect, such as invalid combinations of TCP control flags, or bad UDP
 * lengths.
 */
#define FM_L4_PARSE_ERROR	0x10
/* Check if the hardware parser has run */
#define FM_L4_HXS_RUN		0xE0

/*
 * FD status field indicating whether the FM Parser has attempted to validate
 * the L4 csum of the frame.
 * Note that having this bit set doesn't necessarily imply that the checksum
 * is valid. One would have to check the parse results to find that out.
 */
#define FM_FD_STAT_L4CV		0x00000004

#define FM_FD_STAT_ERRORS						\
	(FM_PORT_FRM_ERR_DMA | FM_PORT_FRM_ERR_PHYSICAL	| \
	 FM_PORT_FRM_ERR_SIZE | FM_PORT_FRM_ERR_CLS_DISCARD | \
	 FM_PORT_FRM_ERR_EXTRACTION | FM_PORT_FRM_ERR_NO_SCHEME	| \
	 FM_PORT_FRM_ERR_ILL_PLCR | FM_PORT_FRM_ERR_PRS_TIMEOUT	| \
	 FM_PORT_FRM_ERR_PRS_ILL_INSTRUCT | FM_PORT_FRM_ERR_PRS_HDR_ERR)

#define FM_FD_STAT_ERR_PHYSICAL	FM_PORT_FRM_ERR_PHYSICAL

/*
 * Check if the FMan Hardware Parser has run for L4 protocols.
 *
 * @parse_result_ptr must be of type (t_FmPrsResult *).
 */
#define fm_l4_hxs_has_run(parse_result_ptr) \
	((parse_result_ptr)->l4r & FM_L4_HXS_RUN)
/*
 * Iff the FMan Hardware Parser has run for L4 protocols, check error status.
 *
 * @parse_result_ptr must be of type (t_FmPrsResult *).
 */
#define fm_l4_hxs_error(parse_result_ptr) \
	((parse_result_ptr)->l4r & FM_L4_PARSE_ERROR)
/*
 * Check if the parsed frame was found to be a TCP segment.
 *
 * @parse_result_ptr must be of type (t_FmPrsResult *).
 */
#define fm_l4_frame_is_tcp(parse_result_ptr) \
	((parse_result_ptr)->l4r & FM_L4_PARSE_RESULT_TCP)

struct pcd_range {
	uint32_t			 base;
	uint32_t			 count;
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
	enum dpa_fq_type fq_type;
};

struct dpa_bp {
	struct bman_pool		*pool;
	uint8_t				bpid;
	struct device			*dev;
	union {
		/*
		 * The buffer pools used for the private ports are initialized
		 * with target_count buffers for each CPU; at runtime the
		 * number of buffers per CPU is constantly brought back to this
		 * level
		 */
		int target_count;
		/*
		 * The configured value for the number of buffers in the pool,
		 * used for shared port buffer pools
		 */
		int config_count;
	};
	size_t				size;
	bool				seed_pool;
	/*
	 * physical address of the contiguous memory used by the pool to store
	 * the buffers
	 */
	dma_addr_t			paddr;
	/*
	 * virtual address of the contiguous memory used by the pool to store
	 * the buffers
	 */
	void				*vaddr;
	int kernel_pool;
	/* current number of buffers in the bpool alloted to this CPU */
	int *percpu_count;
	atomic_t refs;
};

struct dpa_rx_errors {
	u32 dme;		/* DMA Error */
	u32 fpe;		/* Frame Physical Error */
	u32 fse;		/* Frame Size Error */
	u32 phe;		/* Header Error */
	u32 cse;		/* Checksum Validation Error */
};

/* Counters for QMan ERN frames - one counter per rejection code */
struct dpa_ern_cnt {
	u32 cg_tdrop;		/* Congestion group taildrop */
	u32 wred;		/* WRED congestion */
	u32 err_cond;		/* Error condition */
	u32 early_window;	/* Order restoration, frame too early */
	u32 late_window;	/* Order restoration, frame too late */
	u32 fq_tdrop;		/* FQ taildrop */
	u32 fq_retired;		/* FQ is retired */
	u32 orp_zero;		/* ORP disabled */
};

struct dpa_percpu_priv_s {
	struct net_device *net_dev;
	/*
	 * Pointer to the percpu_count of the shared buffer pool
	 * used for the private ports; this assumes there is only
	 * one bpool used
	 */
	int *dpa_bp_count;
	struct dpa_bp *dpa_bp;
	struct napi_struct napi;
#ifdef CONFIG_DPAA_ETH_SG_SUPPORT
	/* a list of preallocated SKBs for this CPU */
	struct sk_buff_head skb_list;
	/* current number of skbs in the CPU's list */
	int skb_count;
#endif
	u32 start_tx;
	u32 in_interrupt;
	u32 ingress_calls;
	u32 tx_returned;
	u32 tx_confirm;
	/* fragmented (non-linear) skbuffs received from the stack */
	u32 tx_frag_skbuffs;
	/*
	 * Frames identified as L4 packets (by FMan's Hardware Parser, but for
	 * which the parsing failed due to some error condition. If we come
	 * across such frames, we drop them instead of passing them up the
	 * stack, which means the L4 stats in the stack won't increment.
	 */
	u32 l4_hxs_errors;
	struct net_device_stats	 stats;
	struct dpa_rx_errors rx_errors;
	struct dpa_ern_cnt ern_cnt;
};

struct dpa_priv_s {
	struct dpa_bp *dpa_bp;
	size_t bp_count;
	int shared;
	struct net_device *net_dev;

	uint16_t		 channel;	/* "fsl,qman-channel-id" */
	struct list_head	 dpa_fq_list;
	struct qman_fq		*egress_fqs[DPAA_ETH_TX_QUEUES];

	struct mac_device	*mac_dev;

	struct dpa_percpu_priv_s	*percpu_priv;
#ifdef CONFIG_DEBUG_FS
	struct dentry		*debugfs_file;
#endif

	uint32_t		 msg_enable;	/* net_device message level */
	struct dpa_ptp_tsu	 *tsu;

#if defined(CONFIG_DPAA_FMAN_UNIT_TESTS)
/* TODO: this is temporary until pcd support is implemented in dpaa */
	int			priv_pcd_num_ranges;
	struct pcd_range	priv_pcd_ranges[FMAN_PCD_TESTS_MAX_NUM_RANGES];
#endif

	struct {
		/**
		 * All egress queues to a given net device belong to one
		 * (and the same) congestion group.
		 */
		struct qman_cgr cgr;
		/* If congested, when it began. Used for performance stats. */
		u32 congestion_start_jiffies;
		/* Number of jiffies the Tx port was congested. */
		u32 congested_jiffies;
		/**
		 * Counter for the number of times the CGR
		 * entered congestion state
		 */
		u32 cgr_congested_count;
	} cgr_data;
};

extern const struct ethtool_ops dpa_ethtool_ops;

void __attribute__((nonnull))
dpa_fd_release(const struct net_device *net_dev, const struct qm_fd *fd);

void dpa_make_private_pool(struct dpa_bp *dpa_bp);

struct dpa_bp *dpa_bpid2pool(int bpid);

void __hot _dpa_rx(struct net_device *net_dev,
		const struct dpa_priv_s *priv,
		struct dpa_percpu_priv_s *percpu_priv,
		const struct qm_fd *fd,
		u32 fqid);

int __hot dpa_tx(struct sk_buff *skb, struct net_device *net_dev);

struct sk_buff *_dpa_cleanup_tx_fd(const struct dpa_priv_s *priv,
				   const struct qm_fd *fd);

int __hot _dpa_process_parse_results(const t_FmPrsResult *parse_results,
				     const struct qm_fd *fd,
				     struct sk_buff *skb,
				     int *use_gro,
				     unsigned int *hdr_size __maybe_unused);

#ifdef CONFIG_DPAA_ETH_SG_SUPPORT
void dpa_bp_add_8_pages(struct dpa_bp *dpa_bp, int cpu_id);

void dpa_list_add_skbs(struct dpa_percpu_priv_s *cpu_priv, int count);
#endif

/*
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
	struct sk_buff *skb, struct qm_fd *fd, char *parse_results);

static inline int dpaa_eth_napi_schedule(struct dpa_percpu_priv_s *percpu_priv)
{
	/*
	 * In case of threaded ISR for RT enable kernel,
	 * in_irq() does not return appropriate value, so use
	 * in_serving_softirq to distinguish softirq or irq context.
	 */
	if (unlikely(in_irq() || !in_serving_softirq())) {
		/* Disable QMan IRQ and invoke NAPI */
		int ret = qman_irqsource_remove(QM_PIRQ_DQRI);
		if (likely(!ret)) {
			napi_schedule(&percpu_priv->napi);
			return 1;
		}
	}
	return 0;
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

/* Verifies if the skb length is below the interface MTU */
static inline int dpa_check_rx_mtu(struct sk_buff *skb, int mtu)
{
	if (unlikely(skb->len > mtu))
		if ((skb->protocol != ETH_P_8021Q) || (skb->len > mtu + 4))
			return -1;

	return 0;
}

/* Equivalent to a memset(0), but works faster */
static inline void clear_fd(struct qm_fd *fd)
{
	fd->opaque_addr = 0;
	fd->opaque = 0;
	fd->cmd = 0;
}

static inline int __hot dpa_xmit(struct dpa_priv_s *priv,
			struct dpa_percpu_priv_s *percpu, int queue,
			struct qm_fd *fd)
{
	int err, i;

	prefetchw(&percpu->start_tx);
	for (i = 0; i < 100000; i++) {
		err = qman_enqueue(priv->egress_fqs[queue], fd, 0);
		if (err != -EBUSY)
			break;
	}

	if (unlikely(err < 0)) {
		/* TODO differentiate b/w -EBUSY (EQCR full) and other codes? */
		percpu->stats.tx_errors++;
		percpu->stats.tx_fifo_errors++;
		return err;
	}

	percpu->stats.tx_packets++;
	percpu->stats.tx_bytes += dpa_fd_length(fd);

	return 0;
}

#if defined CONFIG_DPA_ETH_WQ_LEGACY
#define DPA_NUM_WQS 8
/*
 * Older WQ assignment: statically-defined FQIDs (such as PCDs) are assigned
 * round-robin to all WQs available. Dynamically-allocated FQIDs go to WQ7.
 *
 * Not necessarily the best scheme, but worked fine so far, so we might want
 * to keep it around for a while.
 */
static inline void _dpa_assign_wq(struct dpa_fq *fq)
{
	fq->wq = fq->fqid ? fq->fqid % DPA_NUM_WQS : DPA_NUM_WQS - 1;
}
#elif defined CONFIG_DPA_ETH_WQ_MULTI
/*
 * Use multiple WQs for FQ assignment:
 *	- Tx Confirmation queues go to WQ1.
 *	- Rx Default, Tx and PCD queues go to WQ3 (no differentiation between
 *	  Rx and Tx traffic, or between Rx Default and Rx PCD frames).
 *	- Rx Error and Tx Error queues go to WQ2 (giving them a better chance
 *	  to be scheduled, in case there are many more FQs in WQ3).
 * This ensures that Tx-confirmed buffers are timely released. In particular,
 * it avoids congestion on the Tx Confirm FQs, which can pile up PFDRs if they
 * are greatly outnumbered by other FQs in the system (usually PCDs), while
 * dequeue scheduling is round-robin.
 */
static inline void _dpa_assign_wq(struct dpa_fq *fq)
{
	switch (fq->fq_type) {
	case FQ_TYPE_TX_CONFIRM:
		fq->wq = 1;
		break;
	case FQ_TYPE_RX_DEFAULT:
	case FQ_TYPE_TX:
	case FQ_TYPE_RX_PCD:
		fq->wq = 3;
		break;
	case FQ_TYPE_RX_ERROR:
	case FQ_TYPE_TX_ERROR:
		fq->wq = 2;
		break;
	default:
		WARN(1, "Invalid FQ type %d for FQID %d!\n",
		       fq->fq_type, fq->fqid);
	}
}
#else
/* This shouldn't happen, since we've made a "default" choice in the Kconfig. */
#error "No WQ assignment scheme chosen; Kconfig out-of-sync?"
#endif /* CONFIG_DPA_ETH_WQ_ASSIGN_* */


#endif	/* __DPA_H */
