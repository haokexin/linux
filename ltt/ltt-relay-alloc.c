/*
 * Public API and common code for kernel->userspace relay file support.
 *
 * Copyright (C) 2002-2005 - Tom Zanussi (zanussi@us.ibm.com), IBM Corp
 * Copyright (C) 1999-2005 - Karim Yaghmour (karim@opersys.com)
 * Copyright (C) 2008 - Mathieu Desnoyers (mathieu.desnoyers@polymtl.ca)
 *
 * Moved to kernel/relay.c by Paul Mundt, 2006.
 * November 2006 - CPU hotplug support by Mathieu Desnoyers
 * 	(mathieu.desnoyers@polymtl.ca)
 *
 * This file is released under the GPL.
 */
#include <linux/errno.h>
#include <linux/stddef.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/ltt-relay.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/cpu.h>
#include <linux/splice.h>
#include <linux/bitops.h>

/* list of open channels, for cpu hotplug */
static DEFINE_MUTEX(relay_channels_mutex);
static LIST_HEAD(relay_channels);

/**
 *	relay_alloc_buf - allocate a channel buffer
 *	@buf: the buffer struct
 *	@size: total size of the buffer
 */
static int relay_alloc_buf(struct rchan_buf *buf, size_t *size)
{
	unsigned int i, n_pages;
	struct buf_page *buf_page, *n;

	*size = PAGE_ALIGN(*size);
	n_pages = *size >> PAGE_SHIFT;

	INIT_LIST_HEAD(&buf->pages);

	for (i = 0; i < n_pages; i++) {
		buf_page = kmalloc_node(sizeof(*buf_page), GFP_KERNEL,
			cpu_to_node(buf->cpu));
		if (unlikely(!buf_page))
			goto depopulate;
		buf_page->page = alloc_pages_node(cpu_to_node(buf->cpu),
			GFP_KERNEL | __GFP_ZERO, 0);
		if (unlikely(!buf_page->page)) {
			kfree(buf_page);
			goto depopulate;
		}
		list_add_tail(&buf_page->list, &buf->pages);
		buf_page->virt = page_address(buf_page->page);
		buf_page->offset = (size_t)i << PAGE_SHIFT;
		set_page_private(buf_page->page, (unsigned long)buf_page);
		if (i == 0) {
			buf->wpage = buf_page;
			buf->hpage[0] = buf_page;
			buf->hpage[1] = buf_page;
			buf->rpage = buf_page;
		}
	}
	buf->page_count = n_pages;
	return 0;

depopulate:
	list_for_each_entry_safe(buf_page, n, &buf->pages, list) {
		list_del_init(&buf_page->list);
		__free_page(buf_page->page);
		kfree(buf_page);
	}
	return -ENOMEM;
}

/**
 *	relay_create_buf - allocate and initialize a channel buffer
 *	@chan: the relay channel
 *	@cpu: cpu the buffer belongs to
 *
 *	Returns channel buffer if successful, %NULL otherwise.
 */
static struct rchan_buf *relay_create_buf(struct rchan *chan, int cpu)
{
	int ret;
	struct rchan_buf *buf = kzalloc(sizeof(struct rchan_buf), GFP_KERNEL);
	if (!buf)
		return NULL;

	buf->cpu = cpu;
	ret = relay_alloc_buf(buf, &chan->alloc_size);
	if (ret)
		goto free_buf;

	buf->chan = chan;
	kref_get(&buf->chan->kref);
	return buf;

free_buf:
	kfree(buf);
	return NULL;
}

/**
 *	relay_destroy_channel - free the channel struct
 *	@kref: target kernel reference that contains the relay channel
 *
 *	Should only be called from kref_put().
 */
static void relay_destroy_channel(struct kref *kref)
{
	struct rchan *chan = container_of(kref, struct rchan, kref);
	kfree(chan);
}

void ltt_relay_get_chan(struct rchan *chan)
{
	kref_get(&chan->kref);
}
EXPORT_SYMBOL_GPL(ltt_relay_get_chan);

void ltt_relay_put_chan(struct rchan *chan)
{
	kref_put(&chan->kref, relay_destroy_channel);
}
EXPORT_SYMBOL_GPL(ltt_relay_put_chan);

/**
 *	relay_destroy_buf - destroy an rchan_buf struct and associated buffer
 *	@buf: the buffer struct
 */
static void relay_destroy_buf(struct rchan_buf *buf)
{
	struct rchan *chan = buf->chan;
	struct buf_page *buf_page, *n;

	list_for_each_entry_safe(buf_page, n, &buf->pages, list) {
		list_del_init(&buf_page->list);
		__free_page(buf_page->page);
		kfree(buf_page);
	}
	chan->buf[buf->cpu] = NULL;
	kfree(buf);
	kref_put(&chan->kref, relay_destroy_channel);
}

/**
 *	relay_remove_buf - remove a channel buffer
 *	@kref: target kernel reference that contains the relay buffer
 *
 *	Removes the file from the fileystem, which also frees the
 *	rchan_buf_struct and the channel buffer.  Should only be called from
 *	kref_put().
 */
static void relay_remove_buf(struct kref *kref)
{
	struct rchan_buf *buf = container_of(kref, struct rchan_buf, kref);
	buf->chan->cb->remove_buf_file(buf->dentry);
	relay_destroy_buf(buf);
}

void ltt_relay_get_chan_buf(struct rchan_buf *buf)
{
	kref_get(&buf->kref);
}
EXPORT_SYMBOL_GPL(ltt_relay_get_chan_buf);

void ltt_relay_put_chan_buf(struct rchan_buf *buf)
{
	kref_put(&buf->kref, relay_remove_buf);
}
EXPORT_SYMBOL_GPL(ltt_relay_put_chan_buf);

/*
 * High-level relay kernel API and associated functions.
 */

/*
 * rchan_callback implementations defining default channel behavior.  Used
 * in place of corresponding NULL values in client callback struct.
 */

/*
 * create_buf_file_create() default callback.  Does nothing.
 */
static struct dentry *create_buf_file_default_callback(const char *filename,
						       struct dentry *parent,
						       int mode,
						       struct rchan_buf *buf)
{
	return NULL;
}

/*
 * remove_buf_file() default callback.  Does nothing.
 */
static int remove_buf_file_default_callback(struct dentry *dentry)
{
	return -EINVAL;
}

/* relay channel default callbacks */
static struct rchan_callbacks default_channel_callbacks = {
	.create_buf_file = create_buf_file_default_callback,
	.remove_buf_file = remove_buf_file_default_callback,
};

/**
 *	__relay_reset - reset a channel buffer
 *	@buf: the channel buffer
 *	@init: 1 if this is a first-time initialization
 *
 *	See relay_reset() for description of effect.
 */
static void __relay_reset(struct rchan_buf *buf, unsigned int init)
{
	if (init)
		kref_init(&buf->kref);
}

/*
 *	relay_open_buf - create a new relay channel buffer
 *
 *	used by relay_open() and CPU hotplug.
 */
static struct rchan_buf *relay_open_buf(struct rchan *chan, unsigned int cpu)
{
	struct rchan_buf *buf = NULL;
	struct dentry *dentry;
	char *tmpname;

	tmpname = kzalloc(NAME_MAX + 1, GFP_KERNEL);
	if (!tmpname)
		goto end;
	snprintf(tmpname, NAME_MAX, "%s%d", chan->base_filename, cpu);

	buf = relay_create_buf(chan, cpu);
	if (!buf)
		goto free_name;

	__relay_reset(buf, 1);

	/* Create file in fs */
	dentry = chan->cb->create_buf_file(tmpname, chan->parent, S_IRUSR,
					   buf);
	if (!dentry)
		goto free_buf;

	buf->dentry = dentry;

	goto free_name;

free_buf:
	relay_destroy_buf(buf);
	buf = NULL;
free_name:
	kfree(tmpname);
end:
	return buf;
}

/**
 *	relay_close_buf - close a channel buffer
 *	@buf: channel buffer
 *
 *	Restores the default callbacks.
 *	The channel buffer and channel buffer data structure are then freed
 *	automatically when the last reference is given up.
 */
static void relay_close_buf(struct rchan_buf *buf)
{
	kref_put(&buf->kref, relay_remove_buf);
}

static void setup_callbacks(struct rchan *chan,
				   struct rchan_callbacks *cb)
{
	if (!cb) {
		chan->cb = &default_channel_callbacks;
		return;
	}

	if (!cb->create_buf_file)
		cb->create_buf_file = create_buf_file_default_callback;
	if (!cb->remove_buf_file)
		cb->remove_buf_file = remove_buf_file_default_callback;
	chan->cb = cb;
}

/**
 * 	relay_hotcpu_callback - CPU hotplug callback
 * 	@nb: notifier block
 * 	@action: hotplug action to take
 * 	@hcpu: CPU number
 *
 * 	Returns the success/failure of the operation. (%NOTIFY_OK, %NOTIFY_BAD)
 */
static int __cpuinit relay_hotcpu_callback(struct notifier_block *nb,
				unsigned long action,
				void *hcpu)
{
	unsigned int hotcpu = (unsigned long)hcpu;
	struct rchan *chan;

	switch (action) {
	case CPU_UP_PREPARE:
	case CPU_UP_PREPARE_FROZEN:
		mutex_lock(&relay_channels_mutex);
		list_for_each_entry(chan, &relay_channels, list) {
			if (chan->buf[hotcpu])
				continue;
			chan->buf[hotcpu] = relay_open_buf(chan, hotcpu);
			if (!chan->buf[hotcpu]) {
				printk(KERN_ERR
					"relay_hotcpu_callback: cpu %d buffer "
					"creation failed\n", hotcpu);
				mutex_unlock(&relay_channels_mutex);
				return NOTIFY_BAD;
			}
		}
		mutex_unlock(&relay_channels_mutex);
		break;
	case CPU_DEAD:
	case CPU_DEAD_FROZEN:
		/* No need to flush the cpu : will be flushed upon
		 * final relay_flush() call. */
		break;
	}
	return NOTIFY_OK;
}

/**
 *	ltt_relay_open - create a new relay channel
 *	@base_filename: base name of files to create
 *	@parent: dentry of parent directory, %NULL for root directory
 *	@subbuf_size: size of sub-buffers
 *	@n_subbufs: number of sub-buffers
 *	@cb: client callback functions
 *	@private_data: user-defined data
 *
 *	Returns channel pointer if successful, %NULL otherwise.
 *
 *	Creates a channel buffer for each cpu using the sizes and
 *	attributes specified.  The created channel buffer files
 *	will be named base_filename0...base_filenameN-1.  File
 *	permissions will be %S_IRUSR.
 */
struct rchan *ltt_relay_open(const char *base_filename,
			 struct dentry *parent,
			 size_t subbuf_size,
			 size_t n_subbufs,
			 struct rchan_callbacks *cb,
			 void *private_data)
{
	unsigned int i;
	struct rchan *chan;
	if (!base_filename)
		return NULL;

	if (!(subbuf_size && n_subbufs))
		return NULL;

	chan = kzalloc(sizeof(struct rchan), GFP_KERNEL);
	if (!chan)
		return NULL;

	chan->version = LTT_RELAY_CHANNEL_VERSION;
	chan->n_subbufs = n_subbufs;
	chan->subbuf_size = subbuf_size;
	chan->subbuf_size_order = get_count_order(subbuf_size);
	chan->alloc_size = FIX_SIZE(subbuf_size * n_subbufs);
	chan->parent = parent;
	chan->private_data = private_data;
	strlcpy(chan->base_filename, base_filename, NAME_MAX);
	setup_callbacks(chan, cb);
	kref_init(&chan->kref);

	mutex_lock(&relay_channels_mutex);
	for_each_online_cpu(i) {
		chan->buf[i] = relay_open_buf(chan, i);
		if (!chan->buf[i])
			goto free_bufs;
	}
	list_add(&chan->list, &relay_channels);
	mutex_unlock(&relay_channels_mutex);

	return chan;

free_bufs:
	for_each_possible_cpu(i) {
		if (!chan->buf[i])
			break;
		relay_close_buf(chan->buf[i]);
	}

	kref_put(&chan->kref, relay_destroy_channel);
	mutex_unlock(&relay_channels_mutex);
	return NULL;
}
EXPORT_SYMBOL_GPL(ltt_relay_open);

/**
 *	ltt_relay_close - close the channel
 *	@chan: the channel
 *
 *	Closes all channel buffers and frees the channel.
 */
void ltt_relay_close(struct rchan *chan)
{
	unsigned int i;

	if (!chan)
		return;

	mutex_lock(&relay_channels_mutex);
	for_each_possible_cpu(i)
		if (chan->buf[i])
			relay_close_buf(chan->buf[i]);

	list_del(&chan->list);
	kref_put(&chan->kref, relay_destroy_channel);
	mutex_unlock(&relay_channels_mutex);
}
EXPORT_SYMBOL_GPL(ltt_relay_close);

/*
 * Start iteration at the previous element. Skip the real list head.
 */
struct buf_page *ltt_relay_find_prev_page(struct rchan_buf *buf,
	struct buf_page *page, size_t offset, ssize_t diff_offset)
{
	struct buf_page *iter;
	size_t orig_iter_off;
	unsigned int i = 0;

	orig_iter_off = page->offset;
	list_for_each_entry_reverse(iter, &page->list, list) {
		/*
		 * Skip the real list head.
		 */
		if (&iter->list == &buf->pages)
			continue;
		i++;
		if (offset >= iter->offset
			&& offset < iter->offset + PAGE_SIZE) {
#ifdef CONFIG_LTT_RELAY_CHECK_RANDOM_ACCESS
			if (!buf->random_access && i > 1) {
				printk(KERN_WARNING
					"Backward random access detected in "
					"ltt_relay. Iterations %u, "
					"offset %zu, orig iter->off %zu, "
					"iter->off %zu diff_offset %zd.\n", i,
					offset, orig_iter_off, iter->offset,
					diff_offset);
				WARN_ON(1);
			}
#endif
			return iter;
		}
	}
	WARN_ON(1);
	return NULL;
}
EXPORT_SYMBOL_GPL(ltt_relay_find_prev_page);

/*
 * Start iteration at the next element. Skip the real list head.
 */
struct buf_page *ltt_relay_find_next_page(struct rchan_buf *buf,
	struct buf_page *page, size_t offset, ssize_t diff_offset)
{
	struct buf_page *iter;
	unsigned int i = 0;
	size_t orig_iter_off;

	orig_iter_off = page->offset;
	list_for_each_entry(iter, &page->list, list) {
		/*
		 * Skip the real list head.
		 */
		if (&iter->list == &buf->pages)
			continue;
		i++;
		if (offset >= iter->offset
			&& offset < iter->offset + PAGE_SIZE) {
#ifdef CONFIG_LTT_RELAY_CHECK_RANDOM_ACCESS
			if (!buf->random_access && i > 1) {
				printk(KERN_WARNING
					"Forward random access detected in "
					"ltt_relay. Iterations %u, "
					"offset %zu, orig iter->off %zu, "
					"iter->off %zu diff_offset %zd.\n", i,
					offset, orig_iter_off, iter->offset,
					diff_offset);
				WARN_ON(1);
			}
#endif
			return iter;
		}
	}
	WARN_ON(1);
	return NULL;
}
EXPORT_SYMBOL_GPL(ltt_relay_find_next_page);

/**
 * ltt_relay_write - write data to a ltt_relay buffer.
 * @buf : buffer
 * @offset : offset within the buffer
 * @src : source address
 * @len : length to write
 * @page : cached buffer page
 * @pagecpy : page size copied so far
 */
void _ltt_relay_write(struct rchan_buf *buf, size_t offset,
	const void *src, size_t len, struct buf_page *page, ssize_t pagecpy)
{
	do {
		len -= pagecpy;
		src += pagecpy;
		offset += pagecpy;
		/*
		 * Underlying layer should never ask for writes across
		 * subbuffers.
		 */
		WARN_ON(offset >= buf->chan->alloc_size);

		page = ltt_relay_cache_page(buf, &buf->wpage, page, offset);
		pagecpy = min_t(size_t, len, PAGE_SIZE - (offset & ~PAGE_MASK));
		ltt_relay_do_copy(page->virt
				+ (offset & ~PAGE_MASK), src, pagecpy);
	} while (unlikely(len != pagecpy));
}
EXPORT_SYMBOL_GPL(_ltt_relay_write);

/**
 * ltt_relay_read - read data from ltt_relay_buffer.
 * @buf : buffer
 * @offset : offset within the buffer
 * @dest : destination address
 * @len : length to write
 */
int ltt_relay_read(struct rchan_buf *buf, size_t offset,
	void *dest, size_t len)
{
	struct buf_page *page;
	ssize_t pagecpy, orig_len;

	orig_len = len;
	offset &= buf->chan->alloc_size - 1;
	page = buf->rpage;
	if (unlikely(!len))
		return 0;
	for (;;) {
		page = ltt_relay_cache_page(buf, &buf->rpage, page, offset);
		pagecpy = min_t(size_t, len, PAGE_SIZE - (offset & ~PAGE_MASK));
		memcpy(dest, page->virt + (offset & ~PAGE_MASK), pagecpy);
		len -= pagecpy;
		if (likely(!len))
			break;
		dest += pagecpy;
		offset += pagecpy;
		/*
		 * Underlying layer should never ask for reads across
		 * subbuffers.
		 */
		WARN_ON(offset >= buf->chan->alloc_size);
	}
	return orig_len;
}
EXPORT_SYMBOL_GPL(ltt_relay_read);

/**
 * ltt_relay_read_cstr - read a C-style string from ltt_relay_buffer.
 * @buf : buffer
 * @offset : offset within the buffer
 * @dest : destination address
 * @len : destination's length
 *
 * return string's length
 */
int ltt_relay_read_cstr(struct rchan_buf *buf, size_t offset,
		void *dest, size_t len)
{
	struct buf_page *page;
	ssize_t pagecpy, pagelen, strpagelen, orig_offset;
	char *str;

	offset &= buf->chan->alloc_size - 1;
	orig_offset = offset;
	page = buf->rpage;
	for (;;) {
		page = ltt_relay_cache_page(buf, &buf->rpage, page, offset);
		str = (char *)page->virt + (offset & ~PAGE_MASK);
		pagelen = PAGE_SIZE - (offset & ~PAGE_MASK);
		strpagelen = strnlen(str, pagelen);
		if (len) {
			pagecpy = min_t(size_t, len, strpagelen);
			memcpy(dest, str, pagecpy);
			len -= pagecpy;
			dest += pagecpy;
		}
		offset += strpagelen;
		if (strpagelen < pagelen)
			break;
		/*
		 * Underlying layer should never ask for reads across
		 * subbuffers.
		 */
		WARN_ON(offset >= buf->chan->alloc_size);
	}
	if (len)
		((char *)dest)[0] = 0;
	return offset - orig_offset;
}
EXPORT_SYMBOL_GPL(ltt_relay_read_cstr);

/**
 * ltt_relay_read_get_page - Get a whole page to read from
 * @buf : buffer
 * @offset : offset within the buffer
 */
struct buf_page *ltt_relay_read_get_page(struct rchan_buf *buf, size_t offset)
{
	struct buf_page *page;

	offset &= buf->chan->alloc_size - 1;
	page = buf->rpage;
	page = ltt_relay_cache_page(buf, &buf->rpage, page, offset);
	return page;
}
EXPORT_SYMBOL_GPL(ltt_relay_read_get_page);

/**
 * ltt_relay_offset_address - get address of a location within the buffer
 * @buf : buffer
 * @offset : offset within the buffer.
 *
 * Return the address where a given offset is located.
 * Should be used to get the current subbuffer header pointer. Given we know
 * it's never on a page boundary, it's safe to write directly to this address,
 * as long as the write is never bigger than a page size.
 */
void *ltt_relay_offset_address(struct rchan_buf *buf, size_t offset)
{
	struct buf_page *page;
	unsigned int odd;

	offset &= buf->chan->alloc_size - 1;
	odd = !!(offset & buf->chan->subbuf_size);
	page = buf->hpage[odd];
	if (offset < page->offset || offset >= page->offset + PAGE_SIZE)
		buf->hpage[odd] = page = buf->wpage;
	page = ltt_relay_cache_page(buf, &buf->hpage[odd], page, offset);
	return page->virt + (offset & ~PAGE_MASK);
}
EXPORT_SYMBOL_GPL(ltt_relay_offset_address);

/**
 *	relay_file_open - open file op for relay files
 *	@inode: the inode
 *	@filp: the file
 *
 *	Increments the channel buffer refcount.
 */
static int relay_file_open(struct inode *inode, struct file *filp)
{
	struct rchan_buf *buf = inode->i_private;
	kref_get(&buf->kref);
	filp->private_data = buf;

	return nonseekable_open(inode, filp);
}

/**
 *	relay_file_release - release file op for relay files
 *	@inode: the inode
 *	@filp: the file
 *
 *	Decrements the channel refcount, as the filesystem is
 *	no longer using it.
 */
static int relay_file_release(struct inode *inode, struct file *filp)
{
	struct rchan_buf *buf = filp->private_data;
	kref_put(&buf->kref, relay_remove_buf);

	return 0;
}

const struct file_operations ltt_relay_file_operations = {
	.open		= relay_file_open,
	.release	= relay_file_release,
};
EXPORT_SYMBOL_GPL(ltt_relay_file_operations);

static __init int relay_init(void)
{
	hotcpu_notifier(relay_hotcpu_callback, 5);
	return 0;
}

module_init(relay_init);
