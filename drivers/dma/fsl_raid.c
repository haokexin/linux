/*
 * drivers/dma/fsl_raid.c
 *
 * Freescale RAID Engine device driver
 *
 * Author:
 *	Harninder Rai <harninder.rai@freescale.com>
 *	Naveen Burmi <naveenburmi@freescale.com>
 *
 * Copyright (c) 2010-2012 Freescale Semiconductor, Inc.
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
 *
 * Theory of operation:
 *
 * General capabilities:
 *	RAID Engine (RE) block is capable of offloading XOR, memcpy and P/Q
 *	calculations required in RAID5 and RAID6 operations. RE driver
 *	registers with Linux's ASYNC layer as dma driver. RE hardware
 *	maintains strict ordering of the requests through chained
 *	command queueing.
 *
 * Data flow:
 *	Software RAID layer of Linux (MD layer) maintains RAID partitions,
 *	strips, stripes etc. It sends requests to the underlying AYSNC layer
 *	which further passes it to RE driver. ASYNC layer decides which request
 *	goes to which job ring of RE hardware. For every request processed by
 *	RAID Engine, driver gets an interrupt unless coalescing is set. The
 *	per job ring interrupt handler checks the status register for errors,
 *	clears the interrupt and schedules a tasklet. Main request processing
 *	is done in tasklet. A software shadow copy of the HW ring is kept to
 *	maintain virtual to physical translation. Based on the internal indexes
 *	maintained, the tasklet picks the descriptor address from shadow copy,
 *	updates the corresponding cookie, updates the outbound ring job removed
 *	register in RE hardware and eventually calls the callback function. This
 *	callback function gets passed as part of request from MD layer.
 */

#include <linux/interrupt.h>
#include <linux/of_platform.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/dmaengine.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/slab.h>

#include "fsl_raid.h"

#define MAX_RE_JRS		4
#define MAX_XOR_SRCS		16
#define MAX_PQ_SRCS		16
#define MAX_INITIAL_DESCS	256
#define FRAME_FORMAT		0x1
#define MAX_DATA_LENGTH		(1024*1024)

#define to_fsl_re_dma_desc(tx) container_of(tx, \
		struct fsl_re_dma_async_tx_desc, async_tx)

struct re_drv_private {
	struct device *dev;
	u8 total_jrs;
	struct dma_device dma_dev;
	struct re_ctrl *re_regs;
	struct re_jr *re_jrs[MAX_RE_JRS];
};

static struct kmem_cache *re_sw_desc_cache;

/* Add descriptors into per jr software queue - submit_q */
static dma_cookie_t re_jr_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct fsl_re_dma_async_tx_desc *desc = NULL;
	struct re_jr *jr = NULL;
	dma_cookie_t cookie;

	desc = container_of(tx, struct fsl_re_dma_async_tx_desc, async_tx);
	jr = container_of(tx->chan, struct re_jr, chan);

	spin_lock_bh(&jr->desc_lock);

	jr->timer.data = (unsigned long)tx->chan;
	cookie = jr->chan.cookie + 1;
	if (cookie < 0)
		cookie = 1;

	desc->async_tx.cookie = cookie;
	jr->chan.cookie = desc->async_tx.cookie;
	list_add_tail(&desc->node, &jr->submit_q);

	if (!timer_pending(&jr->timer))
		add_timer(&jr->timer);

	spin_unlock_bh(&jr->desc_lock);

	return cookie;
}

static void re_jr_unmap_dest_src(struct fsl_re_dma_async_tx_desc *desc)
{
	int i, j;
	struct cmpnd_frame *cf;
	dma_addr_t dest1 = 0, dest2 = 0, src;
	struct device *dev;
	enum dma_ctrl_flags flags;
	enum dma_data_direction dir;

	BUG_ON(!desc);
	cf = desc->cf_addr;
	dest1 = cf[1].address;
	j = 2;
	if (desc->dest_cnt == 2) {
		dest2 = cf[2].address;
		j = 3;
	}
	dev = desc->jr->chan.device->dev;
	flags = desc->async_tx.flags;
	if (!(flags & DMA_COMPL_SKIP_DEST_UNMAP)) {
		if (desc->cdb_opcode == RE_MOVE_OPCODE)
			dir = DMA_FROM_DEVICE;
		else
			dir = DMA_BIDIRECTIONAL;

		dma_unmap_page(dev, dest1, desc->dma_len, dir);

		if (dest2)
			dma_unmap_page(dev, dest2, desc->dma_len, dir);
	}

	if (!(flags & DMA_COMPL_SKIP_SRC_UNMAP)) {
		dir = DMA_TO_DEVICE;
		for (i = j; i < desc->src_cnt+j; i++) {
			src = cf[i].address;
			if (src == dest1 || src == dest2)
				continue;
			dma_unmap_page(dev, src, desc->dma_len, dir);
		}
	}
}

static void re_jr_desc_done(struct fsl_re_dma_async_tx_desc *desc)
{
	struct re_jr *dma_jr = desc->jr;
	dma_async_tx_callback callback;
	void *callback_param;

	dma_run_dependencies(&desc->async_tx);

	spin_lock_bh(&dma_jr->desc_lock);
	if (dma_jr->completed_cookie < desc->async_tx.cookie) {
		dma_jr->completed_cookie = desc->async_tx.cookie;
		if (dma_jr->completed_cookie == DMA_MAX_COOKIE)
			dma_jr->completed_cookie = DMA_MIN_COOKIE;
	}
	spin_unlock_bh(&dma_jr->desc_lock);

	callback = desc->async_tx.callback;
	callback_param = desc->async_tx.callback_param;

	re_jr_unmap_dest_src(desc);

	/* Free the cf/cdb address stored in descs
	 * unconditionally. These pointers are only
	 * required for RE driver's housekeeping
	 */
	dma_pool_free(dma_jr->desc_pool, desc->cf_addr, desc->cf_paddr);
	dma_pool_free(dma_jr->desc_pool, desc->cdb_addr, desc->cdb_paddr);

	if (callback)
		callback(callback_param);
}

/*
 * Get the virtual address of software desc from virt_addr.
 * Storing the address of software desc like this makes the
 * order of alogorithm as O(1)
 */
static void re_jr_dequeue(unsigned long data)
{
	struct device *dev = (struct device *)data;
	struct re_jr *jr = dev_get_drvdata(dev);
	struct fsl_re_dma_async_tx_desc *desc;
	unsigned int count = 0;
	struct fsl_re_dma_async_tx_desc *ack_desc = NULL, *_ack_desc = NULL;

	while ((count =
		RE_JR_OUB_SLOT_FULL(in_be32(&jr->jrregs->oubring_slot_full)))) {
		while (count--) {
			spin_lock_bh(&jr->desc_lock);

			/* Wrap around index */
			jr->oub_count &= RING_SIZE-1;

			desc = (struct fsl_re_dma_async_tx_desc *)
				jr->virt_arry[jr->oub_count].virt_addr;

			/* Re-initialise so that it can be reused */
			jr->virt_arry[jr->oub_count].virt_addr = 0;
			jr->virt_arry[jr->oub_count++].phys_addr = 0;

			/* One job processed */
			out_be32(&jr->jrregs->oubring_job_rmvd,
						RE_JR_OUB_JOB_REMOVE);

			list_add_tail(&desc->node, &jr->ack_q);

			spin_unlock_bh(&jr->desc_lock);

			re_jr_desc_done(desc);
		}
	}

	/* To save memory, parse the ack_q and free up descs */
	list_for_each_entry_safe(ack_desc, _ack_desc, &jr->ack_q, node) {
		if (async_tx_test_ack(&ack_desc->async_tx)) {
			spin_lock_bh(&jr->desc_lock);
			list_del(&ack_desc->node);
			spin_unlock_bh(&jr->desc_lock);
			kmem_cache_free(re_sw_desc_cache, ack_desc);
		}
	}
}

/* Per Job Ring interrupt handler */
static irqreturn_t re_jr_interrupt(int irq, void *data)
{
	struct device *dev = data;
	struct re_jr *jr = dev_get_drvdata(dev);
	u32 irqstate;

	irqstate = in_be32(&jr->jrregs->jr_interrupt_status);
	if (!irqstate)
		return IRQ_NONE;

	/*
	 * There's no way in upper layer (read MD layer) to recover from
	 * error conditions except restart everything. In long term we
	 * need to do something more than just crashing
	 */
	if (irqstate & RE_JR_ERROR) {
		dev_err(dev, "%s: jr error irqstate: %x\n", __func__,
			irqstate);
		BUG();
	}

	/* Clear interrupt */
	out_be32(&jr->jrregs->jr_interrupt_status, RE_JR_CLEAR_INT);

	tasklet_schedule(&jr->irqtask);

	return IRQ_HANDLED;
}

static int re_jr_alloc_chan_resources(struct dma_chan *chan)
{
	struct re_jr *jr = container_of(chan, struct re_jr, chan);

	jr->desc_pool = dma_pool_create("re_jr_desc_pool", jr->dev,
					RE_CF_CDB_SIZE, RE_CF_CDB_ALIGN, 0);

	if (!jr->desc_pool) {
		pr_err("%s:No memory for re_jr_desc_pool\n", __func__);
		return -ENOMEM;
	}

	return 0;
}

/* This function is just to please the ASYNC layer */
static void re_jr_free_chan_resources(struct dma_chan *chan)
{
	struct re_jr *jr = container_of(chan, struct re_jr, chan);
	dma_pool_destroy(jr->desc_pool);
	return;
}

static enum dma_status re_jr_tx_status(struct dma_chan *chan,
		dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	struct re_jr *jr = NULL;
	dma_cookie_t last_used;
	dma_cookie_t last_complete;

	jr = container_of(chan, struct re_jr, chan);
	last_used = chan->cookie;
	smp_mb();
	last_complete = jr->completed_cookie;

	dma_set_tx_state(txstate, last_complete, last_used, 0);

	return dma_async_is_complete(cookie, last_complete, last_used);
}


/* Copy descriptor from per jr software queue into hardware job ring */
void re_jr_issue_pending(struct dma_chan *chan)
{
	struct re_jr *jr = NULL;
	struct fsl_re_dma_async_tx_desc *desc, *_desc;

	jr = container_of(chan, struct re_jr, chan);
	if (timer_pending(&jr->timer))
		del_timer_sync(&jr->timer);

	spin_lock_bh(&jr->desc_lock);

	list_for_each_entry_safe(desc, _desc, &jr->submit_q, node) {

		if (!in_be32(&jr->jrregs->inbring_slot_avail))
			goto out_unlock;


		/* Wrap around ring index */
		jr->inb_count &= RING_SIZE-1;

		if (jr->virt_arry[jr->inb_count].phys_addr != 0)
			goto out_unlock;


		jr->virt_arry[jr->inb_count].phys_addr =
			(phys_addr_t)desc->hwdesc.address;
		jr->virt_arry[jr->inb_count++].virt_addr = (unsigned long)desc;

		/* Wrap around ring index */
		jr->inb_ring_index &= RING_SIZE-1;

		/* Copy frame descriptor into job ring */
		memcpy(&jr->inb_ring_virt_addr[jr->inb_ring_index],
			&desc->hwdesc, sizeof(struct jr_hw_desc));

		smp_wmb();

		jr->inb_ring_index++;
		list_del_init(&desc->node);

		wmb();

		/* One job has been added into job ring */
		out_be32(&jr->jrregs->inbring_add_job, RE_JR_INB_JOB_ADD);
	}
out_unlock:
	spin_unlock_bh(&jr->desc_lock);
}

/* Per Job Ring timer handler */
static void raide_timer_handler(unsigned long data)
{
	struct dma_chan *chan = NULL;
	chan = (struct dma_chan *)data;

	re_jr_issue_pending(chan);

	return;
}

inline void fill_cfd_frame(struct cmpnd_frame *cf, u8 index,
		size_t length, dma_addr_t addr, bool final)
{
	cf[index].extension = 0;
	cf[index].final = final;
	cf[index].rsvd1 = 0;
	cf[index].rsvd3 = 0;
	cf[index].rsvd4 = 0;
	cf[index].rsvd5 = 0;
	cf[index].length = length;
	cf[index].offset = 0;
	cf[index].address = addr;

}

static struct fsl_re_dma_async_tx_desc *re_jr_alloc_desc(struct re_jr *jr,
					int cf_num, unsigned long flags)
{
	struct fsl_re_dma_async_tx_desc *desc = NULL;
	struct jr_hw_desc *hw_desc = NULL;
	struct cmpnd_frame *cf = NULL;
	void *cdb = NULL;
	dma_addr_t paddr;

	desc = kmem_cache_alloc(re_sw_desc_cache, GFP_KERNEL);
	if (!desc)
		return ERR_PTR(-ENOMEM);

	memset(desc, 0, sizeof(*desc));
	desc->async_tx.tx_submit = re_jr_tx_submit;
	dma_async_tx_descriptor_init(&desc->async_tx, &jr->chan);
	INIT_LIST_HEAD(&desc->node);
	hw_desc = &desc->hwdesc;
	hw_desc->format = FRAME_FORMAT;

	cf = dma_pool_alloc(jr->desc_pool, GFP_ATOMIC, &paddr);
	if (!cf) {
		kmem_cache_free(re_sw_desc_cache, desc);
		return ERR_PTR(-ENOMEM);
	}

	hw_desc->address = paddr;
	desc->cf_addr = cf;
	desc->cf_paddr = paddr;
	desc->cf_len = sizeof(struct cmpnd_frame)*cf_num;
	memset(cf, 0, desc->cf_len);

	cdb = dma_pool_alloc(jr->desc_pool, GFP_ATOMIC, &paddr);
	if (!cdb) {
		dma_pool_free(jr->desc_pool, desc->cf_addr, desc->cf_paddr);
		kmem_cache_free(re_sw_desc_cache, desc);
		return ERR_PTR(-ENOMEM);
	}
	memset(cdb, 0, RE_CF_CDB_SIZE);

	desc->cdb_addr = cdb;
	desc->cdb_paddr = paddr;

	desc->jr = jr;
	desc->async_tx.cookie = -EBUSY;
	desc->async_tx.flags = flags;

	return desc;
}

static struct dma_async_tx_descriptor *re_jr_prep_genq(
		struct dma_chan *chan, dma_addr_t dest, dma_addr_t *src,
		unsigned int src_cnt, const unsigned char *scf, size_t len,
		unsigned long flags)
{
	struct re_jr *jr = NULL;
	struct fsl_re_dma_async_tx_desc *desc = NULL;
	struct xor_cdb *xor = NULL;
	unsigned int cfs_reqd = src_cnt + 2; /* CDB+dest+src_cnt */
	struct cmpnd_frame *cf;
	unsigned int i = 0;
	unsigned int j = 0;

	if (len > MAX_DATA_LENGTH) {
		pr_err("%s: Length greater than %d not supported\n",
				__func__, MAX_DATA_LENGTH);
		return NULL;
	}
	jr = container_of(chan, struct re_jr, chan);
	desc = re_jr_alloc_desc(jr, cfs_reqd, flags);
	if (!desc || desc < 0)
		return NULL;

	desc->dma_len = len;
	desc->dest_cnt = 1;
	desc->src_cnt = src_cnt;

	desc->cdb_opcode = RE_XOR_OPCODE;
	desc->cdb_len = sizeof(struct xor_cdb);

	/* Filling xor CDB */
	xor = desc->cdb_addr;
	xor->opcode = RE_XOR_OPCODE;
	xor->nrcs = (src_cnt - 1);
	xor->blk_size = RE_BLOCK_SIZE;
	xor->cache_attrib = CACHEABLE_INPUT_OUTPUT;
	xor->buffer_attrib = BUFFERABLE_OUTPUT;
	xor->error_attrib = INTERRUPT_ON_ERROR;
	xor->data_depend = DATA_DEPENDENCY;
	xor->dpi = ENABLE_DPI;

	if (scf != NULL) {
		/* compute q = src0*coef0^src1*coef1^..., * is GF(8) mult */
		for (i = 0; i < src_cnt; i++)
			xor->gfm[i] = scf[i];
	} else {
		/* compute P, that is XOR all srcs */
		for (i = 0; i < src_cnt; i++)
			xor->gfm[i] = 1;
	}

	/* Filling frame 0 of compound frame descriptor with CDB */
	cf = desc->cf_addr;
	fill_cfd_frame(cf, 0, desc->cdb_len, desc->cdb_paddr, 0);

	/* Fill CFD's 1st frame with dest buffer */
	fill_cfd_frame(cf, 1, len, dest, 0);

	/* Fill CFD's rest of the frames with source buffers */
	for (i = 2, j = 0; j < src_cnt; i++, j++)
		fill_cfd_frame(cf, i, len, src[j], 0);

	/* Setting the final bit in the last source buffer frame in CFD */
	cf[i - 1].final = 1;

	return &desc->async_tx;
}

/*
 * Prep function for P parity calculation.In RAID Engine terminology,
 * XOR calculation is called GenQ calculation done through GenQ command
 */
static struct dma_async_tx_descriptor *re_jr_prep_dma_xor(
		struct dma_chan *chan, dma_addr_t dest, dma_addr_t *src,
		unsigned int src_cnt, size_t len, unsigned long flags)
{
	/* NULL let genq take all coef as 1 */
	return re_jr_prep_genq(chan, dest, src, src_cnt, NULL, len, flags);
}

/*
 * Prep function for P/Q parity calculation.In RAID Engine terminology,
 * P/Q calculation is called GenQQ done through GenQQ command
 */
static struct dma_async_tx_descriptor *re_jr_prep_pq(
		struct dma_chan *chan, dma_addr_t *dest, dma_addr_t *src,
		unsigned int src_cnt, const unsigned char *scf, size_t len,
		unsigned long flags)
{
	struct re_jr *jr = NULL;
	struct fsl_re_dma_async_tx_desc *desc = NULL;
	struct pq_cdb *pq = NULL;
	u8 cfs_reqd = src_cnt + 3; /* CDB+P+Q+src_cnt */
	struct cmpnd_frame *cf;
	u8 *p;
	int gfmq_len, i, j;

	if (len > MAX_DATA_LENGTH) {
		pr_err("%s: Length greater than %d not supported\n",
				__func__, MAX_DATA_LENGTH);
		return NULL;
	}

	/*
	 * RE requires at least 2 sources, if given only one source, we pass the
	 * second source same as the first one.
	 * With only one source, generate P is meaningless, only care Q.
	 */
	if (src_cnt == 1) {
		struct dma_async_tx_descriptor *tx = NULL;
		dma_addr_t dma_src[2];
		unsigned char coef[2];
		dma_src[0] = *src;
		coef[0] = *scf;
		dma_src[1] = *src;
		coef[1] = 0;
		tx = re_jr_prep_genq(chan, dest[1], dma_src, 2, coef, len,
				flags);
		if (tx) {
			desc = to_fsl_re_dma_desc(tx);
			desc->src_cnt = 1;
		}
		return tx;
	}

	/*
	 * During RAID6 array creation, Linux's MD layer gets P and Q
	 * calculated separately in two steps. But our RAID Engine has
	 * the capability to calculate both P and Q with a single command
	 * Hence to merge well with MD layer, we need to provide a hook
	 * here and call re_jq_prep_genq() function
	 */

	if (flags & DMA_PREP_PQ_DISABLE_P)
		return re_jr_prep_genq(chan, dest[1], src, src_cnt,
				scf, len, flags);

	jr = container_of(chan, struct re_jr, chan);
	desc = re_jr_alloc_desc(jr, cfs_reqd, flags);
	if (!desc || desc < 0)
		return NULL;

	desc->dma_len = len;
	desc->dest_cnt = 2;
	desc->src_cnt = src_cnt;

	desc->cdb_opcode = RE_PQ_OPCODE;
	desc->cdb_len = sizeof(struct pq_cdb);

	/* Filling GenQQ CDB */
	pq = desc->cdb_addr;
	pq->opcode = RE_PQ_OPCODE;
	pq->excl_enable = 0x0; /* Don't exclude for Q1/Q2 parity */
	pq->excl_q1 = 0x0;
	pq->excl_q2 = 0x0;
	pq->blk_size = RE_BLOCK_SIZE;
	pq->cache_attrib = CACHEABLE_INPUT_OUTPUT;
	pq->buffer_attrib = BUFFERABLE_OUTPUT;
	pq->error_attrib = INTERRUPT_ON_ERROR;
	pq->data_depend = DATA_DEPENDENCY;
	pq->dpi = ENABLE_DPI;
	pq->nrcs = (src_cnt - 1);

	p = pq->gfm_q1;
	/* Init gfm_q1[] */
	for (i = 0; i < src_cnt; i++)
		p[i] = 1;

	/* Align gfm[] to 32bit */
	gfmq_len = ((src_cnt+3)/4)*4;

	/* Init gfm_q2[] */
	p += gfmq_len;
	for (i = 0; i < src_cnt; i++)
		p[i] = scf[i];

	/* Filling frame 0 of compound frame descriptor with CDB */
	cf = desc->cf_addr;
	fill_cfd_frame(cf, 0, desc->cdb_len, desc->cdb_paddr, 0);

	/* Fill CFD's 1st & 2nd frame with dest buffers */
	for (i = 1, j = 0; i < 3; i++, j++)
		fill_cfd_frame(cf, i, len, dest[j], 0);

	/* Fill CFD's rest of the frames with source buffers */
	for (i = 3, j = 0; j < src_cnt; i++, j++)
		fill_cfd_frame(cf, i, len, src[j], 0);

	/* Setting the final bit in the last source buffer frame in CFD */
	cf[i - 1].final = 1;

	return &desc->async_tx;
}

/*
 * Prep function for memcpy. In RAID Engine, memcpy is done through MOVE
 * command. Logic of this function will need to be modified once multipage
 * support is added in Linux's MD/ASYNC Layer
 */
static struct dma_async_tx_descriptor *re_jr_prep_memcpy(
		struct dma_chan *chan, dma_addr_t dest, dma_addr_t src,
		size_t len, unsigned long flags)
{
	struct re_jr *jr = NULL;
	struct fsl_re_dma_async_tx_desc *desc = NULL;
	size_t length = 0;
	unsigned int cfs_reqd = 3; /* CDB+dest+src */
	struct cmpnd_frame *cf = NULL;
	struct move_cdb *move = NULL;

	jr = container_of(chan, struct re_jr, chan);

	if (len > MAX_DATA_LENGTH) {
		pr_err("%s: Length greater than %d not supported\n",
				__func__, MAX_DATA_LENGTH);
		return NULL;
	}

	desc = re_jr_alloc_desc(jr, cfs_reqd, flags);
	if (!desc || desc < 0)
		return NULL;

	desc->dma_len = len;
	desc->src_cnt = 1;
	desc->dest_cnt = 1;

	desc->cdb_opcode = RE_MOVE_OPCODE;
	desc->cdb_len = sizeof(struct move_cdb);

	/* Filling move CDB */
	move = desc->cdb_addr;
	move->opcode = RE_MOVE_OPCODE; /* Unicast move */
	move->blk_size = RE_BLOCK_SIZE;
	move->cache_attrib = CACHEABLE_INPUT_OUTPUT;
	move->buffer_attrib = BUFFERABLE_OUTPUT;
	move->error_attrib = INTERRUPT_ON_ERROR;
	move->data_depend = DATA_DEPENDENCY;
	move->dpi = ENABLE_DPI;

	/* Filling frame 0 of CFD with move CDB */
	cf = desc->cf_addr;
	fill_cfd_frame(cf, 0, desc->cdb_len, desc->cdb_paddr, 0);

	length = min_t(size_t, len, MAX_DATA_LENGTH);

	/* Fill CFD's 1st frame with dest buffer */
	fill_cfd_frame(cf, 1, length, dest, 0);

	/* Fill CFD's 2nd frame with src buffer */
	fill_cfd_frame(cf, 2, length, src, 1);

	return &desc->async_tx;
}

/*
 * Job ring probe function. This function gets called for each detected
 * job ring in the system
 */
int re_jr_probe(struct platform_device *ofdev,
		struct device_node *np, u8 q, u32 *off)
{
	struct device *dev = NULL;
	struct re_drv_private *repriv = NULL;
	struct re_jr *jr = NULL;
	struct dma_device *dma_dev = NULL;
	u32 *ptr = NULL;
	u32 status;
	int ret = 0;
	int k = 0;
	struct platform_device *jr_ofdev = NULL;

	dev = &ofdev->dev;
	repriv = dev_get_drvdata(dev);
	dma_dev = &repriv->dma_dev;

	jr = kzalloc(sizeof(struct re_jr), GFP_KERNEL);
	if (!jr) {
		dev_err(dev, "%s: No free memory for allocating JR struct\n",
			__func__);
		return -ENOMEM;
	}

	jr_ofdev = of_platform_device_create(np, NULL, dev);
	if (jr_ofdev == NULL) {
		dev_err(dev, "%s: Not able to create ofdev for jr %d\n",
			__func__, q);
		ret = -EINVAL;
		goto err_free_2;
	}
	dev_set_drvdata(&jr_ofdev->dev, jr);

	ptr = (u32 *)of_get_property(np, "reg", NULL);
	if (!ptr) {
		dev_err(dev, "%s: Reg property not found in JR number %d\n",
			__func__, q);
		ret = -ENODEV;
		goto err_free_2;
	}

	jr->jrregs = (struct jr_config_regs *)((u8 *)repriv->re_regs +
			*off + *ptr);

	jr->irq = irq_of_parse_and_map(np, 0);
	if (jr->irq == NO_IRQ) {
		dev_err(dev, "%s: No IRQ defined for JR %d\n", __func__, q);
		ret = -ENODEV;
		goto err_free_2;
	}

	tasklet_init(&jr->irqtask, re_jr_dequeue,
			(unsigned long)&jr_ofdev->dev);
	ret = request_irq(jr->irq, re_jr_interrupt, 0, "re-jr", &jr_ofdev->dev);
	if (ret) {
		dev_err(dev, "%s: Unable to register JR interrupt for JR %d\n",
			__func__, q);
		ret = -EINVAL;
		goto err_free_2;
	}

	repriv->re_jrs[q] = jr;
	jr->chan.device = dma_dev;
	jr->dev = &ofdev->dev;
	jr->chan.private = jr;

	INIT_LIST_HEAD(&jr->submit_q);
	INIT_LIST_HEAD(&jr->ack_q);
	spin_lock_init(&jr->desc_lock);

	init_timer(&jr->timer);
	jr->timer.expires = jiffies + 10*HZ;
	jr->timer.function = raide_timer_handler;

	list_add_tail(&jr->chan.device_node, &dma_dev->channels);
	dma_dev->chancnt++;

	jr->inb_desc_pool = dma_pool_create("re_jr_inb_desc_pool", jr->dev,
			sizeof(struct jr_hw_desc) * RING_SIZE,
			FRAME_DESC_ALIGNMENT, 0);
	if (!jr->inb_desc_pool) {
		dev_err(dev, "%s:No memory for re_jr_inb_desc_pool\n",
			__func__);
		ret = -ENOMEM;
		goto err_free_2;
	}

	jr->inb_ring_virt_addr = dma_pool_alloc(jr->inb_desc_pool, GFP_ATOMIC,
			&jr->inb_phys_addr);
	if (!jr->inb_ring_virt_addr) {
		dev_err(dev, "%s:No dma mem for inb_ring_virt_addr\n",
			__func__);
		ret = -ENOMEM;
		goto pool_destroy;
	}

	jr->oub_desc_pool = dma_pool_create("re_jr_oub_desc_pool", jr->dev,
			sizeof(struct jr_hw_desc) * RING_SIZE,
			FRAME_DESC_ALIGNMENT, 0);
	if (!jr->oub_desc_pool) {
		dev_err(dev, "%s:No memory for re_jr_oub_desc_pool\n",
			__func__);
		ret = -ENOMEM;
		goto pool_free;
	}

	jr->oub_ring_virt_addr = dma_pool_alloc(jr->oub_desc_pool, GFP_ATOMIC,
			&jr->oub_phys_addr);
	if (!jr->inb_ring_virt_addr) {
		dev_err(dev, "%s:No dma mem for oub_ring_virt_addr\n",
			__func__);
		ret = -ENOMEM;
		goto pool_destroy_2;
	}

	jr->desc_pool = dma_pool_create("re_jr_desc_pool", jr->dev,
			RE_CF_CDB_SIZE, RE_CF_CDB_ALIGN, 0);

	if (!jr->desc_pool) {
		dev_err(dev, "%s:No memory for re_jr_desc_pool\n",
			__func__);
		ret = -ENOMEM;
		goto err_free_2;
	}

	jr->inb_ring_index = 0;
	jr->inb_count = 0;
	jr->oub_count = 0;

	status = in_be32(&jr->jrregs->jr_status);

	if (status & RE_JR_PAUSE) {
		dev_info(dev, "%s: JR is in paused state...enable it\n",
			__func__);
	} else {
		dev_err(dev, "%s: Error:- JR shud be in paused state\n",
			__func__);
		ret = -EINVAL;
		goto pool_free_2;
	}

	/* Program the Inbound/Outbound ring base addresses and size */
	out_be32(&jr->jrregs->inbring_base_h,
			jr->inb_phys_addr & RE_JR_ADDRESS_BIT_MASK);
	out_be32(&jr->jrregs->oubring_base_h,
			jr->oub_phys_addr & RE_JR_ADDRESS_BIT_MASK);
	out_be32(&jr->jrregs->inbring_base_l,
			jr->inb_phys_addr >> RE_JR_ADDRESS_BIT_SHIFT);
	out_be32(&jr->jrregs->oubring_base_l,
			jr->oub_phys_addr >> RE_JR_ADDRESS_BIT_SHIFT);
	out_be32(&jr->jrregs->inbring_size, RING_SIZE << RING_SIZE_SHIFT);
	out_be32(&jr->jrregs->oubring_size, RING_SIZE << RING_SIZE_SHIFT);

	/* Read LIODN value from u-boot */
	status = in_be32(&jr->jrregs->jr_config_1) & RE_JR_REG_LIODN_MASK;

	/* Program the CFG reg */
	out_be32(&jr->jrregs->jr_config_1,
			RE_JR_CFG1_CBSI | RE_JR_CFG1_CBS0 | status);

	/* Enable RE/JR */
	out_be32(&jr->jrregs->jr_command, RE_JR_ENABLE);

	/* Zero'ing the array */
	for (k = 0; k < RING_SIZE; k++) {
		jr->virt_arry[k].virt_addr = 0;
		jr->virt_arry[k].phys_addr = 0;
	}

	return 0;

pool_free_2:
	dma_pool_free(jr->oub_desc_pool, jr->oub_ring_virt_addr,
			jr->oub_phys_addr);
pool_destroy_2:
	dma_pool_destroy(jr->oub_desc_pool);
pool_free:
	dma_pool_free(jr->inb_desc_pool, jr->inb_ring_virt_addr,
			jr->inb_phys_addr);
pool_destroy:
	dma_pool_destroy(jr->inb_desc_pool);
err_free_2:
	kfree(jr);
	return ret;
}

/* Probe function for RAID Engine */
static int __devinit raide_probe(struct platform_device *ofdev)
{
	struct re_drv_private *repriv = NULL;
	struct device *dev = NULL;
	struct device_node *np = NULL;
	struct device_node *child = NULL;
	u32 *off = NULL;
	u8 ridx = 0;
	struct dma_device *dma_dev = NULL;

	dev_info(&ofdev->dev, "Freescale RAID Engine driver\n");

	repriv = kzalloc(sizeof(struct re_drv_private), GFP_KERNEL);
	if (!repriv) {
		dev_err(dev, "%s: No memory for repriv\n", __func__);
		return -ENOMEM;
	}

	dev = &ofdev->dev;
	dev_set_drvdata(dev, repriv);

	/* IOMAP the entire RAID Engine region */
	repriv->re_regs = of_iomap(ofdev->dev.of_node, 0);
	if (repriv->re_regs == NULL) {
		dev_err(dev, "%s: of_iomap failed\n", __func__);
		kfree(repriv);
		return -ENOMEM;
	}

	/* Print the RE version to make sure RE is alive */
	dev_info(dev, "Ver = %x\n", in_be32(&repriv->re_regs->re_version_id));

	/* Program the RE mode */
	out_be32(&repriv->re_regs->global_config, RE_NON_DPAA_MODE);
	dev_info(dev, "%s:RE mode is %x\n", __func__,
			in_be32(&repriv->re_regs->global_config));

	/* Program Galois Field polymomial */
	out_be32(&repriv->re_regs->galois_field_config, RE_GFM_POLY);
	dev_info(dev, "%s:Galois Field Polynomial is %x\n", __func__,
			in_be32(&repriv->re_regs->galois_field_config));

	dma_dev = &repriv->dma_dev;
	dma_dev->dev = dev;
	INIT_LIST_HEAD(&dma_dev->channels);

	dma_dev->device_alloc_chan_resources = re_jr_alloc_chan_resources;
	dma_dev->device_tx_status = re_jr_tx_status;
	dma_dev->device_issue_pending = re_jr_issue_pending;

	dma_dev->max_xor = MAX_XOR_SRCS;
	dma_dev->device_prep_dma_xor = re_jr_prep_dma_xor;
	dma_cap_set(DMA_XOR, dma_dev->cap_mask);

	dma_dev->max_pq = MAX_PQ_SRCS;
	dma_dev->device_prep_dma_pq = re_jr_prep_pq;
	dma_cap_set(DMA_PQ, dma_dev->cap_mask);

	dma_dev->device_prep_dma_memcpy = re_jr_prep_memcpy;
	dma_cap_set(DMA_MEMCPY, dma_dev->cap_mask);

	dma_dev->device_free_chan_resources = re_jr_free_chan_resources;

	repriv->total_jrs = 0;

	/* Parse Device tree to find out the total number of JQs present */
	for_each_compatible_node(np, NULL, "fsl,raideng-v1.0-job-queue") {
		off = (u32 *)of_get_property(np, "reg", NULL);
		if (!off) {
			dev_err(dev, "%s: Reg property not found in JQ node\n",
				__func__);
			return -ENODEV;
		}

		/* Find out the Job Rings present under each JQ */
		for_each_child_of_node(np, child) {
			if (of_device_is_compatible(child,
				"fsl,raideng-v1.0-job-ring")) {
				re_jr_probe(ofdev, child, ridx++, off);
				repriv->total_jrs++;
			}
		}
	}

	re_sw_desc_cache = kmem_cache_create("re_desc_cache",
				sizeof(struct fsl_re_dma_async_tx_desc), 0,
				SLAB_HWCACHE_ALIGN, NULL);

	dma_async_device_register(dma_dev);

	return 0;
}

static void release_jr(struct re_jr *jr)
{
	/* Free the memory allocated from DMA pools and destroy them */
	dma_pool_free(jr->oub_desc_pool, jr->oub_ring_virt_addr,
			jr->oub_phys_addr);
	dma_pool_destroy(jr->oub_desc_pool);
	dma_pool_free(jr->inb_desc_pool, jr->inb_ring_virt_addr,
			jr->inb_phys_addr);
	dma_pool_destroy(jr->inb_desc_pool);

	kfree(jr);
}

static int raide_remove(struct platform_device *ofdev)
{
	struct re_drv_private *repriv = NULL;
	struct device *dev = NULL;
	int i;

	dev = &ofdev->dev;
	repriv = dev_get_drvdata(dev);

	/* Cleanup JR related memory areas */
	for (i = 0; i < repriv->total_jrs; i++)
		release_jr(repriv->re_jrs[i]);

	kmem_cache_destroy(re_sw_desc_cache);

	/* Unregister the driver */
	dma_async_device_unregister(&repriv->dma_dev);

	/* Unmap the RAID Engine region */
	iounmap(repriv->re_regs);

	kfree(repriv);

	return 0;
}

static struct of_device_id raide_ids[] = {
	{ .compatible = "fsl,raideng-v1.0", },
	{}
};

static struct platform_driver raide_driver = {
	.driver = {
		.name = "fsl-raideng",
		.owner = THIS_MODULE,
		.of_match_table = raide_ids,
	},
	.probe = raide_probe,
	.remove = raide_remove,
};

static __init int raide_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&raide_driver);
	if (ret)
		pr_err("fsl-raid: Failed to register platform driver\n");

	return ret;
}

static void __exit raide_exit(void)
{
	platform_driver_unregister(&raide_driver);
}

subsys_initcall(raide_init);
module_exit(raide_exit);

MODULE_AUTHOR("Harninder Rai <harninder.rai@freescale.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Freescale RAID Engine Device Driver");
