/*
 * ltt/ltt-relay-lockless.c
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

#include "ltt-relay-lockless.h"

#if 0
#define printk_dbg(fmt, args...) printk(fmt, args)
#else
#define printk_dbg(fmt, args...)
#endif

static int ltt_relay_create_buffer(struct ltt_trace_struct *trace,
		struct ltt_channel_struct *ltt_chan,
		struct rchan_buf *buf,
		unsigned int cpu,
		unsigned int n_subbufs);

static void ltt_relay_destroy_buffer(struct ltt_channel_struct *ltt_chan,
		unsigned int cpu);

static void ltt_force_switch(struct rchan_buf *buf,
		enum force_switch_mode mode);

static const struct file_operations ltt_file_operations;

static struct dentry *ltt_create_buf_file_callback(const char *filename,
		struct dentry *parent, int mode,
		struct rchan_buf *buf)
{
	struct ltt_channel_struct *ltt_chan;
	int err;
	struct dentry *dentry;

	ltt_chan = buf->chan->private_data;
	err = ltt_relay_create_buffer(ltt_chan->trace, ltt_chan,
					buf, buf->cpu,
					buf->chan->n_subbufs);
	if (err)
		return ERR_PTR(err);

	dentry = debugfs_create_file(filename, mode, parent, buf,
			&ltt_file_operations);
	if (!dentry)
		goto error;
	return dentry;
error:
	ltt_relay_destroy_buffer(ltt_chan, buf->cpu);
	return NULL;
}

static int ltt_remove_buf_file_callback(struct dentry *dentry)
{
	struct rchan_buf *buf = dentry->d_inode->i_private;
	struct ltt_channel_struct *ltt_chan = buf->chan->private_data;

	debugfs_remove(dentry);
	ltt_relay_destroy_buffer(ltt_chan, buf->cpu);

	return 0;
}

/*
 * Wake writers :
 *
 * This must be done after the trace is removed from the RCU list so that there
 * are no stalled writers.
 */
static void ltt_relay_wake_writers(struct ltt_channel_buf_struct *ltt_buf)
{

	if (waitqueue_active(&ltt_buf->write_wait))
		wake_up_interruptible(&ltt_buf->write_wait);
}

/*
 * This function should not be called from NMI interrupt context
 */
static void ltt_buf_unfull(struct rchan_buf *buf,
		unsigned int subbuf_idx,
		long offset)
{
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;

	ltt_relay_wake_writers(ltt_buf);
}

/*
 * Reader API.
 */
static unsigned long get_offset(struct rchan_buf *buf)
{
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;
	return local_read(&ltt_buf->offset);
}

static unsigned long get_consumed(struct rchan_buf *buf)
{
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;
	return atomic_long_read(&ltt_buf->consumed);
}

static int _ltt_open(struct rchan_buf *buf)
{
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;

	if (!atomic_long_add_unless(&ltt_buf->active_readers, 1, 1))
		return -EBUSY;
	ltt_relay_get_chan(buf->chan);
	return 0;
}

static int _ltt_release(struct rchan_buf *buf)
{
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;

	ltt_relay_put_chan(buf->chan);
	WARN_ON(atomic_long_read(&ltt_buf->active_readers) != 1);
	atomic_long_dec(&ltt_buf->active_readers);
	return 0;
}

static int get_subbuf(struct rchan_buf *buf, unsigned long *consumed)
{
	struct ltt_channel_struct *ltt_channel =
		(struct ltt_channel_struct *)buf->chan->private_data;
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;
	long consumed_old, consumed_idx, commit_count, write_offset;

	consumed_old = atomic_long_read(&ltt_buf->consumed);
	consumed_idx = SUBBUF_INDEX(consumed_old, buf->chan);
	commit_count = local_read(&ltt_buf->commit_count[consumed_idx]);
	/*
	 * Make sure we read the commit count before reading the buffer
	 * data and the write offset. Correct consumed offset ordering
	 * wrt commit count is insured by the use of cmpxchg to update
	 * the consumed offset.
	 */
	smp_rmb();
	write_offset = local_read(&ltt_buf->offset);
	/*
	 * Check that the subbuffer we are trying to consume has been
	 * already fully committed.
	 */
	if (((commit_count - buf->chan->subbuf_size)
	     & ltt_channel->commit_count_mask)
	    - (BUFFER_TRUNC(consumed_old, buf->chan)
	       >> ltt_channel->n_subbufs_order)
	    != 0) {
		return -EAGAIN;
	}
	/*
	 * Check that we are not about to read the same subbuffer in
	 * which the writer head is.
	 */
	if ((SUBBUF_TRUNC(write_offset, buf->chan)
	   - SUBBUF_TRUNC(consumed_old, buf->chan))
	   == 0) {
		return -EAGAIN;
	}
	*consumed = consumed_old;
	return 0;
}

static int put_subbuf(struct rchan_buf *buf, unsigned long consumed)
{
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;
	long consumed_new, consumed_old;

	WARN_ON(atomic_long_read(&ltt_buf->active_readers) != 1);

	consumed_old = consumed;
	consumed_new = SUBBUF_ALIGN(consumed_old, buf->chan);

	spin_lock(&ltt_buf->full_lock);
	if (atomic_long_cmpxchg(&ltt_buf->consumed, consumed_old,
				consumed_new)
	    != consumed_old) {
		/* We have been pushed by the writer : the last
		 * buffer read _is_ corrupted! It can also
		 * happen if this is a buffer we never got. */
		spin_unlock(&ltt_buf->full_lock);
		return -EIO;
	} else {
		/* tell the client that buffer is now unfull */
		int index;
		long data;
		index = SUBBUF_INDEX(consumed_old, buf->chan);
		data = BUFFER_OFFSET(consumed_old, buf->chan);
		ltt_buf_unfull(buf, index, data);
		spin_unlock(&ltt_buf->full_lock);
	}
	return 0;
}

static unsigned long get_n_subbufs(struct rchan_buf *buf)
{
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;

	WARN_ON(atomic_long_read(&ltt_buf->active_readers) != 1);
	return buf->chan->n_subbufs;
}

static unsigned long get_subbuf_size(struct rchan_buf *buf)
{
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;

	WARN_ON(atomic_long_read(&ltt_buf->active_readers) != 1);
	return buf->chan->subbuf_size;
}

static void switch_buffer(unsigned long data)
{
	struct ltt_channel_buf_struct *ltt_buf =
		(struct ltt_channel_buf_struct *)data;
	struct rchan_buf *buf = ltt_buf->rbuf;

	if (buf)
		ltt_force_switch(buf, FORCE_ACTIVE);

	ltt_buf->switch_timer.expires += ltt_buf->switch_timer_interval;
	add_timer_on(&ltt_buf->switch_timer, smp_processor_id());
}

static void start_switch_timer(struct ltt_channel_struct *ltt_channel)
{
	struct rchan *rchan = ltt_channel->trans_channel_data;
	int cpu;

	if (!ltt_channel->switch_timer_interval)
		return;

	// TODO : hotplug
	for_each_online_cpu(cpu) {
		struct ltt_channel_buf_struct *ltt_buf;
		struct rchan_buf *buf;

		buf = rchan->buf[cpu];
		ltt_buf = buf->chan_private;
		buf->random_access = 1;
		ltt_buf->switch_timer_interval =
			ltt_channel->switch_timer_interval;
		init_timer(&ltt_buf->switch_timer);
		ltt_buf->switch_timer.function = switch_buffer;
		ltt_buf->switch_timer.expires = jiffies +
					ltt_buf->switch_timer_interval;
		ltt_buf->switch_timer.data = (unsigned long)ltt_buf;
		add_timer_on(&ltt_buf->switch_timer, cpu);
	}
}

/*
 * Cannot use del_timer_sync with add_timer_on, so use an IPI to locally
 * delete the timer.
 */
static void stop_switch_timer_ipi(void *info)
{
	struct ltt_channel_buf_struct *ltt_buf =
		(struct ltt_channel_buf_struct *)info;

	del_timer(&ltt_buf->switch_timer);
}

static void stop_switch_timer(struct ltt_channel_struct *ltt_channel)
{
	struct rchan *rchan = ltt_channel->trans_channel_data;
	int cpu;

	if (!ltt_channel->switch_timer_interval)
		return;

	// TODO : hotplug
	for_each_online_cpu(cpu) {
		struct ltt_channel_buf_struct *ltt_buf;
		struct rchan_buf *buf;

		buf = rchan->buf[cpu];
		ltt_buf = buf->chan_private;
		smp_call_function(stop_switch_timer_ipi, ltt_buf, 1);
		buf->random_access = 0;
	}
}

static struct ltt_channel_buf_access_ops ltt_channel_buf_accessor = {
	.get_offset   = get_offset,
	.get_consumed = get_consumed,
	.get_subbuf = get_subbuf,
	.put_subbuf = put_subbuf,
	.get_n_subbufs = get_n_subbufs,
	.get_subbuf_size = get_subbuf_size,
	.open = _ltt_open,
	.release = _ltt_release,
	.start_switch_timer = start_switch_timer,
	.stop_switch_timer = stop_switch_timer,
};

/**
 *	ltt_open - open file op for ltt files
 *	@inode: opened inode
 *	@file: opened file
 *
 *	Open implementation. Makes sure only one open instance of a buffer is
 *	done at a given moment.
 */
static int ltt_open(struct inode *inode, struct file *file)
{
	int ret;
	struct rchan_buf *buf = inode->i_private;

	ret = _ltt_open(buf);
	if (!ret)
		ret = ltt_relay_file_operations.open(inode, file);
	return ret;
}

/**
 *	ltt_release - release file op for ltt files
 *	@inode: opened inode
 *	@file: opened file
 *
 *	Release implementation.
 */
static int ltt_release(struct inode *inode, struct file *file)
{
	struct rchan_buf *buf = inode->i_private;
	int ret;

	_ltt_release(buf);
	ret = ltt_relay_file_operations.release(inode, file);
	WARN_ON(ret);
	return ret;
}

/**
 *	ltt_poll - file op for ltt files
 *	@filp: the file
 *	@wait: poll table
 *
 *	Poll implementation.
 */
static unsigned int ltt_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;
	struct inode *inode = filp->f_dentry->d_inode;
	struct rchan_buf *buf = inode->i_private;
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;

	if (filp->f_mode & FMODE_READ) {
		poll_wait_set_exclusive(wait);
		poll_wait(filp, &ltt_buf->read_wait, wait);

		WARN_ON(atomic_long_read(&ltt_buf->active_readers) != 1);
		if (SUBBUF_TRUNC(local_read(&ltt_buf->offset),
							buf->chan)
		  - SUBBUF_TRUNC(atomic_long_read(&ltt_buf->consumed),
							buf->chan)
		  == 0) {
			if (ltt_buf->finalized)
				return POLLHUP;
			else
				return 0;
		} else {
			struct rchan *rchan = buf->chan;
			if (SUBBUF_TRUNC(local_read(&ltt_buf->offset),
					buf->chan)
			  - SUBBUF_TRUNC(atomic_long_read(
						&ltt_buf->consumed),
					buf->chan)
			  >= rchan->alloc_size)
				return POLLPRI | POLLRDBAND;
			else
				return POLLIN | POLLRDNORM;
		}
	}
	return mask;
}

/**
 *	ltt_ioctl - control on the debugfs file
 *
 *	@inode: the inode
 *	@filp: the file
 *	@cmd: the command
 *	@arg: command arg
 *
 *	This ioctl implements three commands necessary for a minimal
 *	producer/consumer implementation :
 *	RELAY_GET_SUBBUF
 *		Get the next sub buffer that can be read. It never blocks.
 *	RELAY_PUT_SUBBUF
 *		Release the currently read sub-buffer. Parameter is the last
 *		put subbuffer (returned by GET_SUBBUF).
 *	RELAY_GET_N_BUBBUFS
 *		returns the number of sub buffers in the per cpu channel.
 *	RELAY_GET_SUBBUF_SIZE
 *		returns the size of the sub buffers.
 */
static int ltt_ioctl(struct inode *inode, struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	struct rchan_buf *buf = inode->i_private;
	u32 __user *argp = (u32 __user *)arg;

	switch (cmd) {
	case RELAY_GET_SUBBUF:
	{
		unsigned long consumed;
		int ret;

		ret = get_subbuf(buf, &consumed);
		if (ret)
			return ret;
		else
			return put_user((u32)consumed, argp);
		break;
	}
	case RELAY_PUT_SUBBUF:
	{
		struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;
		u32 uconsumed_old;
		int ret;
		long consumed_old;

		ret = get_user(uconsumed_old, argp);
		if (ret)
			return ret; /* will return -EFAULT */

		consumed_old = atomic_long_read(&ltt_buf->consumed);
		consumed_old = consumed_old & (~0xFFFFFFFFL);
		consumed_old = consumed_old | uconsumed_old;
		ret = put_subbuf(buf, consumed_old);
		if (ret)
			return ret;
		break;
	}
	case RELAY_GET_N_SUBBUFS:
		return put_user((u32)get_n_subbufs(buf), argp);
		break;
	case RELAY_GET_SUBBUF_SIZE:
		return put_user((u32)get_subbuf_size(buf), argp);
		break;
	default:
		return -ENOIOCTLCMD;
	}
	return 0;
}

#ifdef CONFIG_COMPAT
static long ltt_compat_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	long ret = -ENOIOCTLCMD;

	lock_kernel();
	ret = ltt_ioctl(file->f_dentry->d_inode, file, cmd, arg);
	unlock_kernel();

	return ret;
}
#endif

static void ltt_relay_pipe_buf_release(struct pipe_inode_info *pipe,
				   struct pipe_buffer *pbuf)
{
}

static struct pipe_buf_operations ltt_relay_pipe_buf_ops = {
	.can_merge = 0,
	.map = generic_pipe_buf_map,
	.unmap = generic_pipe_buf_unmap,
	.confirm = generic_pipe_buf_confirm,
	.release = ltt_relay_pipe_buf_release,
	.steal = generic_pipe_buf_steal,
	.get = generic_pipe_buf_get,
};

static void ltt_relay_page_release(struct splice_pipe_desc *spd, unsigned int i)
{
}

/*
 *	subbuf_splice_actor - splice up to one subbuf's worth of data
 */
static int subbuf_splice_actor(struct file *in,
			       loff_t *ppos,
			       struct pipe_inode_info *pipe,
			       size_t len,
			       unsigned int flags)
{
	struct rchan_buf *buf = in->private_data;
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;
	unsigned int poff, subbuf_pages, nr_pages;
	struct page *pages[PIPE_BUFFERS];
	struct partial_page partial[PIPE_BUFFERS];
	struct splice_pipe_desc spd = {
		.pages = pages,
		.nr_pages = 0,
		.partial = partial,
		.flags = flags,
		.ops = &ltt_relay_pipe_buf_ops,
		.spd_release = ltt_relay_page_release,
	};
	long consumed_old, consumed_idx, roffset;
	unsigned long bytes_avail;

	/*
	 * Check that a GET_SUBBUF ioctl has been done before.
	 */
	WARN_ON(atomic_long_read(&ltt_buf->active_readers) != 1);
	consumed_old = atomic_long_read(&ltt_buf->consumed);
	consumed_old += *ppos;
	consumed_idx = SUBBUF_INDEX(consumed_old, buf->chan);

	/*
	 * Adjust read len, if longer than what is available
	 */
	bytes_avail = SUBBUF_TRUNC(local_read(&ltt_buf->offset), buf->chan)
		    - consumed_old;
	WARN_ON(bytes_avail > buf->chan->alloc_size);
	len = min_t(size_t, len, bytes_avail);
	subbuf_pages = bytes_avail >> PAGE_SHIFT;
	nr_pages = min_t(unsigned int, subbuf_pages, PIPE_BUFFERS);
	roffset = consumed_old & PAGE_MASK;
	poff = consumed_old & ~PAGE_MASK;
	printk_dbg(KERN_DEBUG "SPLICE actor len %zu pos %zd write_pos %ld\n",
		len, (ssize_t)*ppos, local_read(&ltt_buf->offset));

	for (; spd.nr_pages < nr_pages; spd.nr_pages++) {
		unsigned int this_len;
		struct buf_page *page;

		if (!len)
			break;
		printk_dbg(KERN_DEBUG "SPLICE actor loop len %zu roffset %ld\n",
			len, roffset);

		this_len = PAGE_SIZE - poff;
		page = ltt_relay_read_get_page(buf, roffset);
		spd.pages[spd.nr_pages] = page->page;
		spd.partial[spd.nr_pages].offset = poff;
		spd.partial[spd.nr_pages].len = this_len;

		poff = 0;
		roffset += PAGE_SIZE;
		len -= this_len;
	}

	if (!spd.nr_pages)
		return 0;

	return splice_to_pipe(pipe, &spd);
}

static ssize_t ltt_relay_file_splice_read(struct file *in,
				      loff_t *ppos,
				      struct pipe_inode_info *pipe,
				      size_t len,
				      unsigned int flags)
{
	ssize_t spliced;
	int ret;

	ret = 0;
	spliced = 0;

	printk_dbg(KERN_DEBUG "SPLICE read len %zu pos %zd\n",
		len, (ssize_t)*ppos);
	while (len && !spliced) {
		ret = subbuf_splice_actor(in, ppos, pipe, len, flags);
		printk_dbg(KERN_DEBUG "SPLICE read loop ret %d\n", ret);
		if (ret < 0)
			break;
		else if (!ret) {
			if (flags & SPLICE_F_NONBLOCK)
				ret = -EAGAIN;
			break;
		}

		*ppos += ret;
		if (ret > len)
			len = 0;
		else
			len -= ret;
		spliced += ret;
	}

	if (spliced)
		return spliced;

	return ret;
}

static void ltt_relay_print_subbuffer_errors(
		struct ltt_channel_struct *ltt_chan,
		long cons_off, unsigned int cpu)
{
	struct rchan *rchan = ltt_chan->trans_channel_data;
	struct ltt_channel_buf_struct *ltt_buf = rchan->buf[cpu]->chan_private;
	long cons_idx, commit_count, write_offset;

	cons_idx = SUBBUF_INDEX(cons_off, rchan);
	commit_count = local_read(&ltt_buf->commit_count[cons_idx]);
	/*
	 * No need to order commit_count and write_offset reads because we
	 * execute after trace is stopped when there are no readers left.
	 */
	write_offset = local_read(&ltt_buf->offset);
	printk(KERN_WARNING
		"LTT : unread channel %s offset is %ld "
		"and cons_off : %ld (cpu %u)\n",
		ltt_chan->channel_name, write_offset, cons_off, cpu);
	/* Check each sub-buffer for non filled commit count */
	if (((commit_count - rchan->subbuf_size) & ltt_chan->commit_count_mask)
	    - (BUFFER_TRUNC(cons_off, rchan) >> ltt_chan->n_subbufs_order)
	    != 0)
		printk(KERN_ALERT
			"LTT : %s : subbuffer %lu has non filled "
			"commit count %lu.\n",
			ltt_chan->channel_name, cons_idx, commit_count);
	printk(KERN_ALERT "LTT : %s : commit count : %lu, subbuf size %zd\n",
			ltt_chan->channel_name, commit_count,
			rchan->subbuf_size);
}

static void ltt_relay_print_errors(struct ltt_trace_struct *trace,
		struct ltt_channel_struct *ltt_chan, int cpu)
{
	struct rchan *rchan = ltt_chan->trans_channel_data;
	struct ltt_channel_buf_struct *ltt_buf = rchan->buf[cpu]->chan_private;
	long cons_off;

	/*
	 * Can be called in the error path of allocation when
	 * trans_channel_data is not yet set.
	 */
	if (!rchan)
		return;
	for (cons_off = atomic_long_read(&ltt_buf->consumed);
			(SUBBUF_TRUNC(local_read(&ltt_buf->offset),
				      rchan)
			 - cons_off) > 0;
			cons_off = SUBBUF_ALIGN(cons_off, rchan))
		ltt_relay_print_subbuffer_errors(ltt_chan, cons_off, cpu);
}

static void ltt_relay_print_buffer_errors(struct ltt_channel_struct *ltt_chan,
		unsigned int cpu)
{
	struct ltt_trace_struct *trace = ltt_chan->trace;
	struct rchan *rchan = ltt_chan->trans_channel_data;
	struct ltt_channel_buf_struct *ltt_buf = rchan->buf[cpu]->chan_private;

	if (local_read(&ltt_buf->events_lost))
		printk(KERN_ALERT
			"LTT : %s : %ld events lost "
			"in %s channel (cpu %u).\n",
			ltt_chan->channel_name,
			local_read(&ltt_buf->events_lost),
			ltt_chan->channel_name, cpu);
	if (local_read(&ltt_buf->corrupted_subbuffers))
		printk(KERN_ALERT
			"LTT : %s : %ld corrupted subbuffers "
			"in %s channel (cpu %u).\n",
			ltt_chan->channel_name,
			local_read(&ltt_buf->corrupted_subbuffers),
			ltt_chan->channel_name, cpu);

	ltt_relay_print_errors(trace, ltt_chan, cpu);
}

static void ltt_relay_remove_dirs(struct ltt_trace_struct *trace)
{
	debugfs_remove(trace->dentry.trace_root);
}

/*
 * Create ltt buffer.
 */
static int ltt_relay_create_buffer(struct ltt_trace_struct *trace,
		struct ltt_channel_struct *ltt_chan, struct rchan_buf *buf,
		unsigned int cpu, unsigned int n_subbufs)
{
	struct ltt_channel_buf_struct *ltt_buf;
	unsigned int j;

	ltt_buf = kzalloc_node(sizeof(*ltt_buf), GFP_KERNEL, cpu_to_node(cpu));
	if (!ltt_buf)
		return -ENOMEM;

	ltt_buf->commit_count =
		kzalloc_node(sizeof(ltt_buf->commit_count) * n_subbufs,
			GFP_KERNEL, cpu_to_node(cpu));
	if (!ltt_buf->commit_count) {
		kfree(ltt_buf);
		return -ENOMEM;
	}
	buf->chan_private = ltt_buf;

	kref_get(&trace->kref);
	kref_get(&trace->ltt_transport_kref);
	local_set(&ltt_buf->offset, ltt_subbuffer_header_size());
	atomic_long_set(&ltt_buf->consumed, 0);
	atomic_long_set(&ltt_buf->active_readers, 0);
	for (j = 0; j < n_subbufs; j++)
		local_set(&ltt_buf->commit_count[j], 0);
	init_waitqueue_head(&ltt_buf->write_wait);
	init_waitqueue_head(&ltt_buf->read_wait);
	atomic_set(&ltt_buf->wakeup_readers, 0);
	spin_lock_init(&ltt_buf->full_lock);

	ltt_buffer_begin(buf, trace->start_tsc, 0);
	/* atomic_add made on local variable on data that belongs to
	 * various CPUs : ok because tracing not started (for this cpu). */
	local_add(ltt_subbuffer_header_size(), &ltt_buf->commit_count[0]);

	local_set(&ltt_buf->events_lost, 0);
	local_set(&ltt_buf->corrupted_subbuffers, 0);
	ltt_buf->finalized = 0;
	ltt_buf->rbuf = buf;

	return 0;
}

static void ltt_relay_destroy_buffer(struct ltt_channel_struct *ltt_chan,
		unsigned int cpu)
{
	struct ltt_trace_struct *trace = ltt_chan->trace;
	struct rchan *rchan = ltt_chan->trans_channel_data;
	struct ltt_channel_buf_struct *ltt_buf = rchan->buf[cpu]->chan_private;

	kref_put(&ltt_chan->trace->ltt_transport_kref,
		ltt_release_transport);
	ltt_relay_print_buffer_errors(ltt_chan, cpu);
	kfree(ltt_buf->commit_count);
	kfree(ltt_buf);
	kref_put(&trace->kref, ltt_release_trace);
	wake_up_interruptible(&trace->kref_wq);
}

/*
 * Create channel.
 */
static int ltt_relay_create_channel(const char *trace_name,
		struct ltt_trace_struct *trace, struct dentry *dir,
		const char *channel_name, struct ltt_channel_struct *ltt_chan,
		unsigned int subbuf_size, unsigned int n_subbufs,
		int overwrite)
{
	char *tmpname;
	unsigned int tmpname_len;
	int err = 0;

	tmpname = kmalloc(PATH_MAX, GFP_KERNEL);
	if (!tmpname)
		return EPERM;
	if (overwrite) {
		strncpy(tmpname, LTT_FLIGHT_PREFIX, PATH_MAX-1);
		strncat(tmpname, channel_name,
			PATH_MAX-1-sizeof(LTT_FLIGHT_PREFIX));
	} else {
		strncpy(tmpname, channel_name, PATH_MAX-1);
	}
	strncat(tmpname, "_", PATH_MAX-1-strlen(tmpname));

	ltt_chan->trace = trace;
	ltt_chan->overwrite = overwrite;
	ltt_chan->n_subbufs_order = get_count_order(n_subbufs);
	ltt_chan->commit_count_mask = (~0UL >> ltt_chan->n_subbufs_order);
	ltt_chan->trans_channel_data = ltt_relay_open(tmpname,
			dir,
			subbuf_size,
			n_subbufs,
			&trace->callbacks,
			ltt_chan);
	tmpname_len = strlen(tmpname);
	if (tmpname_len > 0) {
		/* Remove final _ for pretty printing */
		tmpname[tmpname_len-1] = '\0';
	}
	if (ltt_chan->trans_channel_data == NULL) {
		printk(KERN_ERR "LTT : Can't open %s channel for trace %s\n",
				tmpname, trace_name);
		goto relay_open_error;
	}

	ltt_chan->buf_access_ops = &ltt_channel_buf_accessor;

	err = 0;
	goto end;

relay_open_error:
	err = EPERM;
end:
	kfree(tmpname);
	return err;
}

static int ltt_relay_create_dirs(struct ltt_trace_struct *new_trace)
{
	struct dentry *ltt_root_dentry;

	ltt_root_dentry = get_ltt_root();
	if (!ltt_root_dentry)
		return ENOENT;

	new_trace->dentry.trace_root = debugfs_create_dir(new_trace->trace_name,
			ltt_root_dentry);
	put_ltt_root();
	if (new_trace->dentry.trace_root == NULL) {
		printk(KERN_ERR "LTT : Trace directory name %s already taken\n",
				new_trace->trace_name);
		return EEXIST;
	}

	new_trace->callbacks.create_buf_file = ltt_create_buf_file_callback;
	new_trace->callbacks.remove_buf_file = ltt_remove_buf_file_callback;

	return 0;
}

/*
 * LTTng channel flush function.
 *
 * Must be called when no tracing is active in the channel, because of
 * accesses across CPUs.
 */
static notrace void ltt_relay_buffer_flush(struct rchan_buf *buf)
{
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;

	ltt_buf->finalized = 1;
	ltt_force_switch(buf, FORCE_FLUSH);
}

static void ltt_relay_async_wakeup_chan(struct ltt_channel_struct *ltt_channel)
{
	struct rchan *rchan = ltt_channel->trans_channel_data;
	unsigned int i;

	for_each_possible_cpu(i) {
		struct ltt_channel_buf_struct *ltt_buf;

		if (!rchan->buf[i])
			continue;

		ltt_buf = rchan->buf[i]->chan_private;
		if (atomic_read(&ltt_buf->wakeup_readers) == 1) {
			atomic_set(&ltt_buf->wakeup_readers, 0);
			wake_up_interruptible(&ltt_buf->read_wait);
		}
	}
}

static void ltt_relay_finish_buffer(struct ltt_channel_struct *ltt_channel,
		unsigned int cpu)
{
	struct rchan *rchan = ltt_channel->trans_channel_data;

	if (rchan->buf[cpu]) {
		struct ltt_channel_buf_struct *ltt_buf =
				rchan->buf[cpu]->chan_private;
		ltt_relay_buffer_flush(rchan->buf[cpu]);
		ltt_relay_wake_writers(ltt_buf);
	}
}


static void ltt_relay_finish_channel(struct ltt_channel_struct *ltt_channel)
{
	unsigned int i;

	for_each_possible_cpu(i)
		ltt_relay_finish_buffer(ltt_channel, i);
}

static void ltt_relay_remove_channel(struct ltt_channel_struct *channel)
{
	struct rchan *rchan = channel->trans_channel_data;

	ltt_relay_close(rchan);
}

/*
 * This is called with preemption disabled when user space has requested
 * blocking mode.  If one of the active traces has free space below a
 * specific threshold value, we reenable preemption and block.
 */
static int ltt_relay_user_blocking(struct ltt_trace_struct *trace,
		unsigned int chan_index, size_t data_size,
		struct user_dbg_data *dbg)
{
	struct rchan *rchan;
	struct ltt_channel_buf_struct *ltt_buf;
	struct ltt_channel_struct *channel;
	struct rchan_buf *relay_buf;
	int cpu;
	DECLARE_WAITQUEUE(wait, current);

	channel = &trace->channels[chan_index];
	rchan = channel->trans_channel_data;
	cpu = smp_processor_id();
	relay_buf = rchan->buf[cpu];
	ltt_buf = relay_buf->chan_private;

	/*
	 * Check if data is too big for the channel : do not
	 * block for it.
	 */
	if (LTT_RESERVE_CRITICAL + data_size > relay_buf->chan->subbuf_size)
		return 0;

	/*
	 * If free space too low, we block. We restart from the
	 * beginning after we resume (cpu id may have changed
	 * while preemption is active).
	 */
	spin_lock(&ltt_buf->full_lock);
	if (!channel->overwrite) {
		dbg->write = local_read(&ltt_buf->offset);
		dbg->read = atomic_long_read(&ltt_buf->consumed);
		dbg->avail_size = dbg->write + LTT_RESERVE_CRITICAL + data_size
				  - SUBBUF_TRUNC(dbg->read,
						 relay_buf->chan);
		if (dbg->avail_size > rchan->alloc_size) {
			__set_current_state(TASK_INTERRUPTIBLE);
			add_wait_queue(&ltt_buf->write_wait, &wait);
			spin_unlock(&ltt_buf->full_lock);
			preempt_enable();
			schedule();
			__set_current_state(TASK_RUNNING);
			remove_wait_queue(&ltt_buf->write_wait, &wait);
			if (signal_pending(current))
				return -ERESTARTSYS;
			preempt_disable();
			return 1;
		}
	}
	spin_unlock(&ltt_buf->full_lock);
	return 0;
}

static void ltt_relay_print_user_errors(struct ltt_trace_struct *trace,
		unsigned int chan_index, size_t data_size,
		struct user_dbg_data *dbg, int cpu)
{
	struct rchan *rchan;
	struct ltt_channel_buf_struct *ltt_buf;
	struct ltt_channel_struct *channel;
	struct rchan_buf *relay_buf;

	channel = &trace->channels[chan_index];
	rchan = channel->trans_channel_data;
	relay_buf = rchan->buf[cpu];
	ltt_buf = relay_buf->chan_private;

	printk(KERN_ERR "Error in LTT usertrace : "
	"buffer full : event lost in blocking "
	"mode. Increase LTT_RESERVE_CRITICAL.\n");
	printk(KERN_ERR "LTT nesting level is %u.\n",
		per_cpu(ltt_nesting, cpu));
	printk(KERN_ERR "LTT avail size %lu.\n",
		dbg->avail_size);
	printk(KERN_ERR "avai write : %lu, read : %lu\n",
			dbg->write, dbg->read);

	dbg->write = local_read(&ltt_buf->offset);
	dbg->read = atomic_long_read(&ltt_buf->consumed);

	printk(KERN_ERR "LTT cur size %lu.\n",
		dbg->write + LTT_RESERVE_CRITICAL + data_size
		- SUBBUF_TRUNC(dbg->read, relay_buf->chan));
	printk(KERN_ERR "cur write : %lu, read : %lu\n",
			dbg->write, dbg->read);
}

static struct ltt_transport ltt_relay_transport = {
	.name = "relay",
	.owner = THIS_MODULE,
	.ops = {
		.create_dirs = ltt_relay_create_dirs,
		.remove_dirs = ltt_relay_remove_dirs,
		.create_channel = ltt_relay_create_channel,
		.finish_channel = ltt_relay_finish_channel,
		.remove_channel = ltt_relay_remove_channel,
		.wakeup_channel = ltt_relay_async_wakeup_chan,
		.user_blocking = ltt_relay_user_blocking,
		.user_errors = ltt_relay_print_user_errors,
	},
};

static const struct file_operations ltt_file_operations = {
	.open = ltt_open,
	.release = ltt_release,
	.poll = ltt_poll,
	.splice_read = ltt_relay_file_splice_read,
	.ioctl = ltt_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = ltt_compat_ioctl,
#endif
};

static int __init ltt_relay_init(void)
{
	printk(KERN_INFO "LTT : ltt-relay init\n");

	ltt_transport_register(&ltt_relay_transport);

	return 0;
}

static void __exit ltt_relay_exit(void)
{
	printk(KERN_INFO "LTT : ltt-relay exit\n");

	ltt_transport_unregister(&ltt_relay_transport);
}

module_init(ltt_relay_init);
module_exit(ltt_relay_exit);

MODULE_LICENSE("GPL and additional rights");
MODULE_AUTHOR("Mathieu Desnoyers");
MODULE_DESCRIPTION("Linux Trace Toolkit Next Generation Lockless Relay");
