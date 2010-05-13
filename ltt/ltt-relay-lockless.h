#ifndef _LTT_LTT_RELAY_LOCKLESS_H
#define _LTT_LTT_RELAY_LOCKLESS_H

/*
 * ltt/ltt-relay-lockless.h
 *
 * (C) Copyright 2005-2008 - Mathieu Desnoyers (mathieu.desnoyers@polymtl.ca)
 *
 * LTTng lockless buffer space management (reader/writer).
 *
 * Author:
 *	Mathieu Desnoyers (mathieu.desnoyers@polymtl.ca)
 *
 * Inspired from LTT :
 *  Karim Yaghmour (karim@opersys.com)
 *  Tom Zanussi (zanussi@us.ibm.com)
 *  Bob Wisniewski (bob@watson.ibm.com)
 * And from K42 :
 *  Bob Wisniewski (bob@watson.ibm.com)
 *
 * Changelog:
 *  08/10/08, Cleanup.
 *  19/10/05, Complete lockless mechanism.
 *  27/05/05, Modular redesign and rewrite.
 *
 * Userspace reader semantic :
 * while (poll fd != POLLHUP) {
 *   - ioctl RELAY_GET_SUBBUF_SIZE
 *   while (1) {
 *     - ioctl GET_SUBBUF
 *     - splice 1 subbuffer worth of data to a pipe
 *     - splice the data from pipe to disk/network
 *     - ioctl PUT_SUBBUF, check error value
 *       if err val < 0, previous subbuffer was corrupted.
 *   }
 * }
 *
 * Dual LGPL v2.1/GPL v2 license.
 */

#include <linux/time.h>
#include <linux/ltt-tracer.h>
#include <linux/ltt-relay.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/rcupdate.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/bitops.h>
#include <linux/fs.h>
#include <linux/smp_lock.h>
#include <linux/debugfs.h>
#include <linux/stat.h>
#include <linux/cpu.h>
#include <linux/pipe_fs_i.h>
#include <linux/splice.h>
#include <asm/atomic.h>
#include <asm/local.h>

#if 0
#define printk_dbg(fmt, args...) printk(fmt, args)
#else
#define printk_dbg(fmt, args...)
#endif

/* LTTng lockless logging buffer info */
struct ltt_channel_buf_struct {
	/* First 32 bytes cache-hot cacheline */
	local_t offset;			/* Current offset in the buffer */
	local_t *commit_count;		/* Commit count per sub-buffer */
	atomic_long_t consumed;		/*
					 * Current offset in the buffer
					 * standard atomic access (shared)
					 */
	unsigned long last_tsc;		/*
					 * Last timestamp written in the buffer.
					 */
	/* End of first 32 bytes cacheline */
	atomic_long_t active_readers;	/*
					 * Active readers count
					 * standard atomic access (shared)
					 */
	local_t events_lost;
	local_t corrupted_subbuffers;
	spinlock_t full_lock;		/*
					 * buffer full condition spinlock, only
					 * for userspace tracing blocking mode
					 * synchronization with reader.
					 */
	wait_queue_head_t write_wait;	/*
					 * Wait queue for blocking user space
					 * writers
					 */
	atomic_t wakeup_readers;	/* Boolean : wakeup readers waiting ? */
	wait_queue_head_t read_wait;	/* reader wait queue */
	unsigned int finalized;		/* buffer has been finalized */
	struct timer_list switch_timer;	/* timer for periodical switch */
	unsigned long switch_timer_interval;	/* in jiffies. 0 unset */
	struct rchan_buf *rbuf;		/* Pointer to rchan_buf */
} ____cacheline_aligned;

/*
 * Last TSC comparison functions. Check if the current TSC overflows
 * LTT_TSC_BITS bits from the last TSC read. Reads and writes last_tsc
 * atomically.
 */

#if (BITS_PER_LONG == 32)
static __inline__ void save_last_tsc(struct ltt_channel_buf_struct *ltt_buf,
					u64 tsc)
{
	ltt_buf->last_tsc = (unsigned long)(tsc >> LTT_TSC_BITS);
}

static __inline__ int last_tsc_overflow(struct ltt_channel_buf_struct *ltt_buf,
					u64 tsc)
{
	unsigned long tsc_shifted = (unsigned long)(tsc >> LTT_TSC_BITS);

	if (unlikely((tsc_shifted - ltt_buf->last_tsc)))
		return 1;
	else
		return 0;
}
#else
static __inline__ void save_last_tsc(struct ltt_channel_buf_struct *ltt_buf,
					u64 tsc)
{
	ltt_buf->last_tsc = (unsigned long)tsc;
}

static __inline__ int last_tsc_overflow(struct ltt_channel_buf_struct *ltt_buf,
					u64 tsc)
{
	if (unlikely((tsc - ltt_buf->last_tsc) >> LTT_TSC_BITS))
		return 1;
	else
		return 0;
}
#endif

/*
 * A switch is done during tracing or as a final flush after tracing (so it
 * won't write in the new sub-buffer).
 */
enum force_switch_mode { FORCE_ACTIVE, FORCE_FLUSH };

static __inline__ void ltt_buffer_begin(struct rchan_buf *buf,
			u64 tsc, unsigned int subbuf_idx)
{
	struct ltt_channel_struct *channel =
		(struct ltt_channel_struct *)buf->chan->private_data;
	struct ltt_subbuffer_header *header =
		(struct ltt_subbuffer_header *)
			ltt_relay_offset_address(buf,
				subbuf_idx * buf->chan->subbuf_size);

	header->cycle_count_begin = tsc;
	header->lost_size = 0xFFFFFFFF; /* for debugging */
	header->buf_size = buf->chan->subbuf_size;
	ltt_write_trace_header(channel->trace, header);
}

/*
 * offset is assumed to never be 0 here : never deliver a completely empty
 * subbuffer. The lost size is between 0 and subbuf_size-1.
 */
static __inline__ void ltt_buffer_end(struct rchan_buf *buf,
		u64 tsc, unsigned int offset, unsigned int subbuf_idx)
{
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;
	struct ltt_subbuffer_header *header =
		(struct ltt_subbuffer_header *)
			ltt_relay_offset_address(buf,
				subbuf_idx * buf->chan->subbuf_size);

	header->lost_size = SUBBUF_OFFSET((buf->chan->subbuf_size - offset),
				buf->chan);
	header->cycle_count_end = tsc;
	header->events_lost = local_read(&ltt_buf->events_lost);
	header->subbuf_corrupt = local_read(&ltt_buf->corrupted_subbuffers);
}

static __inline__ void ltt_deliver(struct rchan_buf *buf, unsigned int subbuf_idx,
		void *subbuf)
{
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;

	atomic_set(&ltt_buf->wakeup_readers, 1);
}

struct ltt_reserve_switch_offsets {
	long begin, end, old;
	long begin_switch, end_switch_current, end_switch_old;
	long commit_count, reserve_commit_diff;
	size_t before_hdr_pad, size;
};

/*
 * Returns :
 * 0 if ok
 * !0 if execution must be aborted.
 */
static __inline__ int ltt_relay_try_reserve(
		struct ltt_channel_struct *ltt_channel,
		struct ltt_channel_buf_struct *ltt_buf, struct rchan *rchan,
		struct rchan_buf *buf,
		struct ltt_reserve_switch_offsets *offsets, size_t data_size,
		u64 *tsc, unsigned int *rflags, int largest_align)
{
	offsets->begin = local_read(&ltt_buf->offset);
	offsets->old = offsets->begin;
	offsets->begin_switch = 0;
	offsets->end_switch_current = 0;
	offsets->end_switch_old = 0;

	*tsc = trace_clock_read64();
	if (last_tsc_overflow(ltt_buf, *tsc))
		*rflags = LTT_RFLAG_ID_SIZE_TSC;

	if (unlikely(SUBBUF_OFFSET(offsets->begin, buf->chan) == 0)) {
		offsets->begin_switch = 1;		/* For offsets->begin */
	} else {
		offsets->size = ltt_get_header_size(ltt_channel,
					offsets->begin, data_size,
					&offsets->before_hdr_pad, *rflags);
		offsets->size += ltt_align(offsets->begin + offsets->size,
					   largest_align)
				 + data_size;
		if (unlikely((SUBBUF_OFFSET(offsets->begin, buf->chan) +
			     offsets->size) > buf->chan->subbuf_size)) {
			offsets->end_switch_old = 1;	/* For offsets->old */
			offsets->begin_switch = 1;	/* For offsets->begin */
		}
	}
	if (unlikely(offsets->begin_switch)) {
		long subbuf_index;

		/*
		 * We are typically not filling the previous buffer completely.
		 */
		if (likely(offsets->end_switch_old))
			offsets->begin = SUBBUF_ALIGN(offsets->begin,
						      buf->chan);
		offsets->begin = offsets->begin + ltt_subbuffer_header_size();
		/* Test new buffer integrity */
		subbuf_index = SUBBUF_INDEX(offsets->begin, buf->chan);
		offsets->reserve_commit_diff =
			(BUFFER_TRUNC(offsets->begin, buf->chan)
			 >> ltt_channel->n_subbufs_order)
			- (local_read(&ltt_buf->commit_count[subbuf_index])
				& ltt_channel->commit_count_mask);
		if (likely(offsets->reserve_commit_diff == 0)) {
			/* Next buffer not corrupted. */
			if (unlikely(!ltt_channel->overwrite &&
				(SUBBUF_TRUNC(offsets->begin, buf->chan)
				 - SUBBUF_TRUNC(atomic_long_read(
							&ltt_buf->consumed),
						buf->chan))
				>= rchan->alloc_size)) {
				/*
				 * We do not overwrite non consumed buffers
				 * and we are full : event is lost.
				 */
				local_inc(&ltt_buf->events_lost);
				return -1;
			} else {
				/*
				 * next buffer not corrupted, we are either in
				 * overwrite mode or the buffer is not full.
				 * It's safe to write in this new subbuffer.
				 */
			}
		} else {
			/*
			 * Next subbuffer corrupted. Force pushing reader even
			 * in normal mode. It's safe to write in this new
			 * subbuffer.
			 */
		}
		offsets->size = ltt_get_header_size(ltt_channel,
					offsets->begin, data_size,
					&offsets->before_hdr_pad, *rflags);
		offsets->size += ltt_align(offsets->begin + offsets->size,
					   largest_align)
				 + data_size;
		if (unlikely((SUBBUF_OFFSET(offsets->begin, buf->chan)
			     + offsets->size) > buf->chan->subbuf_size)) {
			/*
			 * Event too big for subbuffers, report error, don't
			 * complete the sub-buffer switch.
			 */
			local_inc(&ltt_buf->events_lost);
			return -1;
		} else {
			/*
			 * We just made a successful buffer switch and the event
			 * fits in the new subbuffer. Let's write.
			 */
		}
	} else {
		/*
		 * Event fits in the current buffer and we are not on a switch
		 * boundary. It's safe to write.
		 */
	}
	offsets->end = offsets->begin + offsets->size;

	if (unlikely((SUBBUF_OFFSET(offsets->end, buf->chan)) == 0)) {
		/*
		 * The offset_end will fall at the very beginning of the next
		 * subbuffer.
		 */
		offsets->end_switch_current = 1;	/* For offsets->begin */
	}
	return 0;
}

/*
 * Returns :
 * 0 if ok
 * !0 if execution must be aborted.
 */
static __inline__ int ltt_relay_try_switch(
		enum force_switch_mode mode,
		struct ltt_channel_struct *ltt_channel,
		struct ltt_channel_buf_struct *ltt_buf, struct rchan *rchan,
		struct rchan_buf *buf,
		struct ltt_reserve_switch_offsets *offsets,
		u64 *tsc)
{
	long subbuf_index;

	offsets->begin = local_read(&ltt_buf->offset);
	offsets->old = offsets->begin;
	offsets->begin_switch = 0;
	offsets->end_switch_old = 0;

	*tsc = trace_clock_read64();

	if (SUBBUF_OFFSET(offsets->begin, buf->chan) != 0) {
		offsets->begin = SUBBUF_ALIGN(offsets->begin, buf->chan);
		offsets->end_switch_old = 1;
	} else {
		/* we do not have to switch : buffer is empty */
		return -1;
	}
	if (mode == FORCE_ACTIVE)
		offsets->begin += ltt_subbuffer_header_size();
	/*
	 * Always begin_switch in FORCE_ACTIVE mode.
	 * Test new buffer integrity
	 */
	subbuf_index = SUBBUF_INDEX(offsets->begin, buf->chan);
	offsets->reserve_commit_diff =
		(BUFFER_TRUNC(offsets->begin, buf->chan)
		 >> ltt_channel->n_subbufs_order)
		- (local_read(&ltt_buf->commit_count[subbuf_index])
			& ltt_channel->commit_count_mask);
	if (offsets->reserve_commit_diff == 0) {
		/* Next buffer not corrupted. */
		if (mode == FORCE_ACTIVE
		    && !ltt_channel->overwrite
		    && offsets->begin - atomic_long_read(&ltt_buf->consumed)
		       >= rchan->alloc_size) {
			/*
			 * We do not overwrite non consumed buffers and we are
			 * full : ignore switch while tracing is active.
			 */
			return -1;
		}
	} else {
		/*
		 * Next subbuffer corrupted. Force pushing reader even in normal
		 * mode
		 */
	}
	offsets->end = offsets->begin;
	return 0;
}

static __inline__ void ltt_reserve_push_reader(
		struct ltt_channel_struct *ltt_channel,
		struct ltt_channel_buf_struct *ltt_buf,
		struct rchan *rchan,
		struct rchan_buf *buf,
		struct ltt_reserve_switch_offsets *offsets)
{
	long consumed_old, consumed_new;

	do {
		consumed_old = atomic_long_read(&ltt_buf->consumed);
		/*
		 * If buffer is in overwrite mode, push the reader consumed
		 * count if the write position has reached it and we are not
		 * at the first iteration (don't push the reader farther than
		 * the writer). This operation can be done concurrently by many
		 * writers in the same buffer, the writer being at the farthest
		 * write position sub-buffer index in the buffer being the one
		 * which will win this loop.
		 * If the buffer is not in overwrite mode, pushing the reader
		 * only happens if a sub-buffer is corrupted.
		 */
		if (unlikely((SUBBUF_TRUNC(offsets->end-1, buf->chan)
		   - SUBBUF_TRUNC(consumed_old, buf->chan))
		   >= rchan->alloc_size))
			consumed_new = SUBBUF_ALIGN(consumed_old, buf->chan);
		else
			return;
	} while (unlikely(atomic_long_cmpxchg(&ltt_buf->consumed, consumed_old,
			consumed_new) != consumed_old));

	if (unlikely(consumed_old != consumed_new)) {
		/*
		 * Reader pushed : we are the winner of the push, we can
		 * therefore reequilibrate reserve and commit. Atomic increment
		 * of the commit count permits other writers to play around
		 * with this variable before us. We keep track of
		 * corrupted_subbuffers even in overwrite mode :
		 * we never want to write over a non completely committed
		 * sub-buffer : possible causes : the buffer size is too low
		 * compared to the unordered data input, or there is a writer
		 * that died between the reserve and the commit.
		 */
		if (likely(offsets->reserve_commit_diff)) {
			/*
			 * We have to alter the sub-buffer commit count.
			 * We do not deliver the previous subbuffer, given it
			 * was either corrupted or not consumed (overwrite
			 * mode).
			 */
			local_add(offsets->reserve_commit_diff,
				  &ltt_buf->commit_count[
					SUBBUF_INDEX(offsets->begin,
						     buf->chan)]);
			if (unlikely(!ltt_channel->overwrite
			    || offsets->reserve_commit_diff
			       != rchan->subbuf_size)) {
				/*
				 * The reserve commit diff was not subbuf_size :
				 * it means the subbuffer was partly written to
				 * and is therefore corrupted. If it is multiple
				 * of subbuffer size and we are in flight
				 * recorder mode, we are skipping over a whole
				 * subbuffer.
				 */
				local_inc(&ltt_buf->corrupted_subbuffers);
			}
		}
	}
}


/*
 * ltt_reserve_switch_old_subbuf: switch old subbuffer
 *
 * Concurrency safe because we are the last and only thread to alter this
 * sub-buffer. As long as it is not delivered and read, no other thread can
 * alter the offset, alter the reserve_count or call the
 * client_buffer_end_callback on this sub-buffer.
 *
 * The only remaining threads could be the ones with pending commits. They will
 * have to do the deliver themselves.  Not concurrency safe in overwrite mode.
 * We detect corrupted subbuffers with commit and reserve counts. We keep a
 * corrupted sub-buffers count and push the readers across these sub-buffers.
 *
 * Not concurrency safe if a writer is stalled in a subbuffer and another writer
 * switches in, finding out it's corrupted.  The result will be than the old
 * (uncommited) subbuffer will be declared corrupted, and that the new subbuffer
 * will be declared corrupted too because of the commit count adjustment.
 *
 * Note : offset_old should never be 0 here.
 */
static __inline__ void ltt_reserve_switch_old_subbuf(
		struct ltt_channel_struct *ltt_channel,
		struct ltt_channel_buf_struct *ltt_buf, struct rchan *rchan,
		struct rchan_buf *buf,
		struct ltt_reserve_switch_offsets *offsets, u64 *tsc)
{
	long oldidx = SUBBUF_INDEX(offsets->old - 1, rchan);

	ltt_buffer_end(buf, *tsc, offsets->old, oldidx);
	/* Must write buffer end before incrementing commit count */
	smp_wmb();
	offsets->commit_count =
		local_add_return(rchan->subbuf_size
				 - (SUBBUF_OFFSET(offsets->old - 1, rchan)
				 + 1),
				 &ltt_buf->commit_count[oldidx]);
	if (likely((BUFFER_TRUNC(offsets->old - 1, rchan)
			>> ltt_channel->n_subbufs_order)
			- ((offsets->commit_count - rchan->subbuf_size)
				& ltt_channel->commit_count_mask) == 0))
		ltt_deliver(buf, oldidx, NULL);
}

/*
 * ltt_reserve_switch_new_subbuf: Populate new subbuffer.
 *
 * This code can be executed unordered : writers may already have written to the
 * sub-buffer before this code gets executed, caution.  The commit makes sure
 * that this code is executed before the deliver of this sub-buffer.
 */
static __inline__ void ltt_reserve_switch_new_subbuf(
		struct ltt_channel_struct *ltt_channel,
		struct ltt_channel_buf_struct *ltt_buf, struct rchan *rchan,
		struct rchan_buf *buf,
		struct ltt_reserve_switch_offsets *offsets, u64 *tsc)
{
	long beginidx = SUBBUF_INDEX(offsets->begin, rchan);

	ltt_buffer_begin(buf, *tsc, beginidx);
	/* Must write buffer end before incrementing commit count */
	smp_wmb();
	offsets->commit_count = local_add_return(ltt_subbuffer_header_size(),
			&ltt_buf->commit_count[beginidx]);
	/* Check if the written buffer has to be delivered */
	if (unlikely((BUFFER_TRUNC(offsets->begin, rchan)
			>> ltt_channel->n_subbufs_order)
			- ((offsets->commit_count - rchan->subbuf_size)
				& ltt_channel->commit_count_mask) == 0))
		ltt_deliver(buf, beginidx, NULL);
}


/*
 * ltt_reserve_end_switch_current: finish switching current subbuffer
 *
 * Concurrency safe because we are the last and only thread to alter this
 * sub-buffer. As long as it is not delivered and read, no other thread can
 * alter the offset, alter the reserve_count or call the
 * client_buffer_end_callback on this sub-buffer.
 *
 * The only remaining threads could be the ones with pending commits. They will
 * have to do the deliver themselves.  Not concurrency safe in overwrite mode.
 * We detect corrupted subbuffers with commit and reserve counts. We keep a
 * corrupted sub-buffers count and push the readers across these sub-buffers.
 *
 * Not concurrency safe if a writer is stalled in a subbuffer and another writer
 * switches in, finding out it's corrupted.  The result will be than the old
 * (uncommited) subbuffer will be declared corrupted, and that the new subbuffer
 * will be declared corrupted too because of the commit count adjustment.
 */
static __inline__ void ltt_reserve_end_switch_current(
		struct ltt_channel_struct *ltt_channel,
		struct ltt_channel_buf_struct *ltt_buf, struct rchan *rchan,
		struct rchan_buf *buf,
		struct ltt_reserve_switch_offsets *offsets, u64 *tsc)
{
	long endidx = SUBBUF_INDEX(offsets->end - 1, rchan);

	ltt_buffer_end(buf, *tsc, offsets->end, endidx);
	/* Must write buffer begin before incrementing commit count */
	smp_wmb();
	offsets->commit_count =
		local_add_return(rchan->subbuf_size
				 - (SUBBUF_OFFSET(offsets->end - 1, rchan)
				 + 1),
				 &ltt_buf->commit_count[endidx]);
	if (likely((BUFFER_TRUNC(offsets->end - 1, rchan)
			>> ltt_channel->n_subbufs_order)
			- ((offsets->commit_count - rchan->subbuf_size)
				& ltt_channel->commit_count_mask) == 0))
		ltt_deliver(buf, endidx, NULL);
}

/**
 * ltt_relay_reserve_slot - Atomic slot reservation in a LTTng buffer.
 * @trace: the trace structure to log to.
 * @ltt_channel: channel structure
 * @transport_data: data structure specific to ltt relay
 * @data_size: size of the variable length data to log.
 * @slot_size: pointer to total size of the slot (out)
 * @buf_offset : pointer to reserved buffer offset (out)
 * @tsc: pointer to the tsc at the slot reservation (out)
 * @cpu: cpuid
 *
 * Return : -ENOSPC if not enough space, else returns 0.
 * It will take care of sub-buffer switching.
 */
static __inline__ int ltt_reserve_slot(struct ltt_trace_struct *trace,
		struct ltt_channel_struct *ltt_channel, void **transport_data,
		size_t data_size, size_t *slot_size, long *buf_offset, u64 *tsc,
		unsigned int *rflags, int largest_align, int cpu)
{
	struct rchan *rchan = ltt_channel->trans_channel_data;
	struct rchan_buf *buf = *transport_data = rchan->buf[cpu];
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;
	struct ltt_reserve_switch_offsets offsets;

	offsets.reserve_commit_diff = 0;
	offsets.size = 0;

	/*
	 * Perform retryable operations.
	 */
	if (unlikely(__get_cpu_var(ltt_nesting) > 4)) {
		local_inc(&ltt_buf->events_lost);
		return -EPERM;
	}
	do {
		if (unlikely(ltt_relay_try_reserve(ltt_channel, ltt_buf,
				rchan, buf, &offsets, data_size, tsc, rflags,
				largest_align)))
			return -ENOSPC;
	} while (unlikely(local_cmpxchg(&ltt_buf->offset, offsets.old,
			offsets.end) != offsets.old));

	/*
	 * Atomically update last_tsc. This update races against concurrent
	 * atomic updates, but the race will always cause supplementary full TSC
	 * events, never the opposite (missing a full TSC event when it would be
	 * needed).
	 */
	save_last_tsc(ltt_buf, *tsc);

	/*
	 * Push the reader if necessary
	 */
	ltt_reserve_push_reader(ltt_channel, ltt_buf, rchan, buf, &offsets);

	/*
	 * Switch old subbuffer if needed.
	 */
	if (unlikely(offsets.end_switch_old))
		ltt_reserve_switch_old_subbuf(ltt_channel, ltt_buf, rchan, buf,
			&offsets, tsc);

	/*
	 * Populate new subbuffer.
	 */
	if (unlikely(offsets.begin_switch))
		ltt_reserve_switch_new_subbuf(ltt_channel, ltt_buf, rchan,
			buf, &offsets, tsc);

	if (unlikely(offsets.end_switch_current))
		ltt_reserve_end_switch_current(ltt_channel, ltt_buf, rchan,
			buf, &offsets, tsc);

	*slot_size = offsets.size;
	*buf_offset = offsets.begin + offsets.before_hdr_pad;
	return 0;
}

/*
 * Force a sub-buffer switch for a per-cpu buffer. This operation is
 * completely reentrant : can be called while tracing is active with
 * absolutely no lock held.
 *
 * Note, however, that as a local_cmpxchg is used for some atomic
 * operations, this function must be called from the CPU which owns the buffer
 * for a ACTIVE flush.
 */
static __inline__ void ltt_force_switch(struct rchan_buf *buf,
		enum force_switch_mode mode)
{
	struct ltt_channel_struct *ltt_channel =
			(struct ltt_channel_struct *)buf->chan->private_data;
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;
	struct rchan *rchan = ltt_channel->trans_channel_data;
	struct ltt_reserve_switch_offsets offsets;
	u64 tsc;

	offsets.reserve_commit_diff = 0;
	offsets.size = 0;

	/*
	 * Perform retryable operations.
	 */
	do {
		if (ltt_relay_try_switch(mode, ltt_channel, ltt_buf,
				rchan, buf, &offsets, &tsc))
			return;
	} while (local_cmpxchg(&ltt_buf->offset, offsets.old,
			offsets.end) != offsets.old);

	/*
	 * Atomically update last_tsc. This update races against concurrent
	 * atomic updates, but the race will always cause supplementary full TSC
	 * events, never the opposite (missing a full TSC event when it would be
	 * needed).
	 */
	save_last_tsc(ltt_buf, tsc);

	/*
	 * Push the reader if necessary
	 */
	if (mode == FORCE_ACTIVE)
		ltt_reserve_push_reader(ltt_channel, ltt_buf, rchan,
					buf, &offsets);

	/*
	 * Switch old subbuffer if needed.
	 */
	if (offsets.end_switch_old)
		ltt_reserve_switch_old_subbuf(ltt_channel, ltt_buf, rchan, buf,
			&offsets, &tsc);

	/*
	 * Populate new subbuffer.
	 */
	if (mode == FORCE_ACTIVE)
		ltt_reserve_switch_new_subbuf(ltt_channel,
			ltt_buf, rchan, buf, &offsets, &tsc);
}

/*
 * for flight recording. must be called after relay_commit.
 * This function decrements de subbuffer's lost_size each time the commit count
 * reaches back the reserve offset (module subbuffer size). It is useful for
 * crash dump.
 * We use slot_size - 1 to make sure we deal correctly with the case where we
 * fill the subbuffer completely (so the subbuf index stays in the previous
 * subbuffer).
 */
#ifdef CONFIG_LTT_VMCORE
static __inline__ void ltt_write_commit_counter(struct rchan_buf *buf,
		long buf_offset, size_t slot_size)
{
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;
	struct ltt_subbuffer_header *header;
	long offset, subbuf_idx, commit_count;
	uint32_t lost_old, lost_new;

	subbuf_idx = SUBBUF_INDEX(buf_offset - 1, buf->chan);
	offset = buf_offset + slot_size;
	header = (struct ltt_subbuffer_header *)
			ltt_relay_offset_address(buf,
				subbuf_idx * buf->chan->subbuf_size);
	for (;;) {
		lost_old = header->lost_size;
		commit_count =
			local_read(&ltt_buf->commit_count[subbuf_idx]);
		/* SUBBUF_OFFSET includes commit_count_mask */
		if (likely(!SUBBUF_OFFSET(offset - commit_count, buf->chan))) {
			lost_new = (uint32_t)buf->chan->subbuf_size
				   - SUBBUF_OFFSET(commit_count, buf->chan);
			lost_old = cmpxchg_local(&header->lost_size, lost_old,
						lost_new);
			if (likely(lost_old <= lost_new))
				break;
		} else {
			break;
		}
	}
}
#else
static __inline__ void ltt_write_commit_counter(struct rchan_buf *buf,
		long buf_offset, size_t slot_size)
{
}
#endif

/*
 * Atomic unordered slot commit. Increments the commit count in the
 * specified sub-buffer, and delivers it if necessary.
 *
 * Parameters:
 *
 * @ltt_channel : channel structure
 * @transport_data: transport-specific data
 * @buf_offset : offset following the event header.
 * @slot_size : size of the reserved slot.
 */
static __inline__ void ltt_commit_slot(
		struct ltt_channel_struct *ltt_channel,
		void **transport_data, long buf_offset, size_t slot_size)
{
	struct rchan_buf *buf = *transport_data;
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;
	struct rchan *rchan = buf->chan;
	long offset_end = buf_offset;
	long endidx = SUBBUF_INDEX(offset_end - 1, rchan);
	long commit_count;

	/* Must write slot data before incrementing commit count */
	smp_wmb();
	commit_count = local_add_return(slot_size,
		&ltt_buf->commit_count[endidx]);
	/* Check if all commits have been done */
	if (unlikely((BUFFER_TRUNC(offset_end - 1, rchan)
			>> ltt_channel->n_subbufs_order)
			- ((commit_count - rchan->subbuf_size)
			   & ltt_channel->commit_count_mask) == 0))
		ltt_deliver(buf, endidx, NULL);
	/*
	 * Update lost_size for each commit. It's needed only for extracting
	 * ltt buffers from vmcore, after crash.
	 */
	ltt_write_commit_counter(buf, buf_offset, slot_size);
}

#endif //_LTT_LTT_RELAY_LOCKLESS_H
