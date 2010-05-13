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

#include <linux/cache.h>
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
} ____cacheline_internodealigned_in_smp;

/*
 * A switch is done during tracing or as a final flush after tracing (so it
 * won't write in the new sub-buffer).
 */
enum force_switch_mode { FORCE_ACTIVE, FORCE_FLUSH };

extern int ltt_reserve_slot_lockless_slow(struct ltt_trace_struct *trace,
		struct ltt_channel_struct *ltt_channel, void **transport_data,
		size_t data_size, size_t *slot_size, long *buf_offset, u64 *tsc,
		unsigned int *rflags, int largest_align, int cpu);

extern void ltt_force_switch_lockless_slow(struct rchan_buf *buf,
		enum force_switch_mode mode);

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

static __inline__ void ltt_deliver(struct rchan_buf *buf, unsigned int subbuf_idx,
		void *subbuf)
{
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;

	atomic_set(&ltt_buf->wakeup_readers, 1);
}

/*
 * returns 0 if reserve ok, or 1 if the slow path must be taken.
 */
static __inline__ int ltt_relay_try_reserve(
		struct ltt_channel_struct *ltt_channel,
		struct ltt_channel_buf_struct *ltt_buf, struct rchan *rchan,
		struct rchan_buf *buf,
		size_t data_size,
		u64 *tsc, unsigned int *rflags, int largest_align,
		long *o_begin, long *o_end, long *o_old,
		long *before_hdr_pad, long *size)
{
	*o_begin = local_read(&ltt_buf->offset);
	*o_old = *o_begin;

	*tsc = trace_clock_read64();

	prefetch(&ltt_buf->commit_count[SUBBUF_INDEX(*o_begin, rchan)]);

	if (last_tsc_overflow(ltt_buf, *tsc))
		*rflags = LTT_RFLAG_ID_SIZE_TSC;

	if (unlikely(SUBBUF_OFFSET(*o_begin, buf->chan) == 0))
		return 1;

	*size = ltt_get_header_size(ltt_channel,
				*o_begin, data_size,
				before_hdr_pad, *rflags);
	*size += ltt_align(*o_begin + *size, largest_align) + data_size;
	if (unlikely((SUBBUF_OFFSET(*o_begin, buf->chan) + *size)
		     > buf->chan->subbuf_size))
		return 1;

	/*
	 * Event fits in the current buffer and we are not on a switch
	 * boundary. It's safe to write.
	 */
	*o_end = *o_begin + *size;

	if (unlikely((SUBBUF_OFFSET(*o_end, buf->chan)) == 0))
		/*
		 * The offset_end will fall at the very beginning of the next
		 * subbuffer.
		 */
		return 1;

	return 0;
}

static __inline__ int ltt_reserve_slot(struct ltt_trace_struct *trace,
		struct ltt_channel_struct *ltt_channel, void **transport_data,
		size_t data_size, size_t *slot_size, long *buf_offset, u64 *tsc,
		unsigned int *rflags, int largest_align, int cpu)
{
	struct rchan *rchan = ltt_channel->trans_channel_data;
	struct rchan_buf *buf = *transport_data = rchan->buf[cpu];
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;
	long o_begin, o_end, o_old;
	size_t before_hdr_pad;

	/*
	 * Perform retryable operations.
	 */
	if (unlikely(__get_cpu_var(ltt_nesting) > 4)) {
		local_inc(&ltt_buf->events_lost);
		return -EPERM;
	}

	if (unlikely(ltt_relay_try_reserve(ltt_channel, ltt_buf,
			rchan, buf, data_size, tsc, rflags,
			largest_align, &o_begin, &o_end, &o_old,
			&before_hdr_pad, slot_size)))
		goto slow_path;

	if (unlikely(local_cmpxchg(&ltt_buf->offset, o_old, o_end) != o_old))
		goto slow_path;

	/*
	 * Atomically update last_tsc. This update races against concurrent
	 * atomic updates, but the race will always cause supplementary full TSC
	 * events, never the opposite (missing a full TSC event when it would be
	 * needed).
	 */
	save_last_tsc(ltt_buf, *tsc);

	*buf_offset = o_begin + before_hdr_pad;
	return 0;
slow_path:
	return ltt_reserve_slot_lockless_slow(trace, ltt_channel,
		transport_data, data_size, slot_size, buf_offset, tsc,
		rflags, largest_align, cpu);
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
	return ltt_force_switch_lockless_slow(buf, mode);
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
