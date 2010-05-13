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
static int relay_alloc_buf(struct rchan_buf *buf, size_t *size,
			   size_t n_subbufs, int extra_reader_sb)
{
	long i, j, n_pages, n_pages_per_sb, page_idx = 0;
	struct page **pages;
	void **virt;

	n_pages = *size >> PAGE_SHIFT;
	n_pages_per_sb = n_pages >> get_count_order(n_subbufs);
	if (extra_reader_sb)
		n_pages += n_pages_per_sb;	/* Add pages for reader */

	pages = kmalloc_node(max_t(size_t, sizeof(*pages) * n_pages,
				   1 << INTERNODE_CACHE_SHIFT),
			GFP_KERNEL, cpu_to_node(buf->cpu));
	if (unlikely(!pages))
		goto pages_error;

	virt = kmalloc_node(ALIGN(sizeof(*virt) * n_pages,
				  1 << INTERNODE_CACHE_SHIFT),
			GFP_KERNEL, cpu_to_node(buf->cpu));
	if (unlikely(!virt))
		goto virt_error;

	for (i = 0; i < n_pages; i++) {
		pages[i] = alloc_pages_node(cpu_to_node(buf->cpu),
			GFP_KERNEL | __GFP_ZERO, 0);
		if (unlikely(!pages[i]))
			goto depopulate;
		virt[i] = page_address(pages[i]);
	}
	buf->page_count = n_pages;
	buf->_pages = pages;
	buf->_virt = virt;

	/* Allocate write-side page index */
	buf->rchan_wsb = kzalloc_node(max_t(size_t,
				sizeof(struct rchan_sb) * n_subbufs,
				1 << INTERNODE_CACHE_SHIFT),
				GFP_KERNEL, cpu_to_node(buf->cpu));
	if (unlikely(!buf->rchan_wsb))
		goto depopulate;

	for (i = 0; i < n_subbufs; i++) {
		buf->rchan_wsb[i].pages =
			kzalloc_node(max_t(size_t,
				sizeof(struct rchan_page) * n_pages_per_sb,
				1 << INTERNODE_CACHE_SHIFT),
				GFP_KERNEL, cpu_to_node(buf->cpu));
		if (!buf->rchan_wsb[i].pages)
			goto free_rchan_wsb;
	}

	if (extra_reader_sb) {
		/* Allocate read-side page index */
		buf->rchan_rsb.pages =
			kzalloc_node(max_t(size_t,
				sizeof(struct rchan_page) * n_pages_per_sb,
				1 << INTERNODE_CACHE_SHIFT),
				GFP_KERNEL, cpu_to_node(buf->cpu));
		if (unlikely(!buf->rchan_rsb.pages))
			goto free_rchan_wsb;
	} else {
		buf->rchan_rsb.pages = NULL;
	}

	/* Assign pages to write-side page index */
	for (i = 0; i < n_subbufs; i++) {
		for (j = 0; j < n_pages_per_sb; j++) {
			WARN_ON(page_idx > n_pages);
			buf->rchan_wsb[i].pages[j].virt = virt[page_idx];
			buf->rchan_wsb[i].pages[j].page = pages[page_idx];
			page_idx++;
		}
		RCHAN_SB_SET_NOREF(buf->rchan_wsb[i].pages);
	}

	if (extra_reader_sb) {
		for (j = 0; j < n_pages_per_sb; j++) {
			WARN_ON(page_idx > n_pages);
			buf->rchan_rsb.pages[j].virt = virt[page_idx];
			buf->rchan_rsb.pages[j].page = pages[page_idx];
			page_idx++;
		}
		RCHAN_SB_SET_NOREF(buf->rchan_rsb.pages);
	}

	/*
	 * If kmalloc ever uses vmalloc underneath, make sure the buffer pages
	 * will not fault.
	 */
	vmalloc_sync_all();
	return 0;

free_rchan_wsb:
	for (i = 0; i < n_subbufs; i++) {
		RCHAN_SB_CLEAR_NOREF(buf->rchan_wsb[i].pages);
		kfree(buf->rchan_wsb[i].pages);
	}
	kfree(buf->rchan_wsb);
depopulate:
	/*
	 * Free all pages from [ i - 1 down to 0 ].
	 * If i = 0, don't free anything.
	 */
	for (i--; i >= 0; i--)
		__free_page(pages[i]);
	kfree(virt);
virt_error:
	kfree(pages);
pages_error:
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
	ret = relay_alloc_buf(buf, &chan->alloc_size, chan->n_subbufs,
			      chan->extra_reader_sb);
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
	struct page **pages;
	long i;

	/* Destroy index */
	if (chan->extra_reader_sb) {
		RCHAN_SB_CLEAR_NOREF(buf->rchan_rsb.pages);
		kfree(buf->rchan_rsb.pages);
	}
	for (i = 0; i < chan->n_subbufs; i++) {
		RCHAN_SB_CLEAR_NOREF(buf->rchan_wsb[i].pages);
		kfree(buf->rchan_wsb[i].pages);
	}
	kfree(buf->rchan_wsb);

	/* Destroy pages */
	pages = buf->_pages;
	for (i = 0; i < buf->page_count; i++)
		__free_page(pages[i]);
	kfree(buf->_pages);
	kfree(buf->_virt);
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
 *	@subbuf_size: size of sub-buffers (> PAGE_SIZE, power of 2)
 *	@n_subbufs: number of sub-buffers (power of 2)
 *	@cb: client callback functions
 *	@private_data: user-defined data
 *	@extra_reader_sb: allocate an extra subbuffer for the reader
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
			 void *private_data,
			 int extra_reader_sb)
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

	/* Check that the subbuffer size is larger than a page. */
	WARN_ON_ONCE(subbuf_size < PAGE_SIZE);

	/*
	 * Make sure the number of subbuffers and subbuffer size are power of 2.
	 */
	WARN_ON_ONCE(hweight32(subbuf_size) != 1);
	WARN_ON(hweight32(n_subbufs) != 1);

	chan->version = LTT_RELAY_CHANNEL_VERSION;
	chan->n_subbufs = n_subbufs;
	chan->subbuf_size = subbuf_size;
	chan->subbuf_size_order = get_count_order(subbuf_size);
	chan->alloc_size = subbuf_size * n_subbufs;
	chan->parent = parent;
	chan->private_data = private_data;
	chan->extra_reader_sb = extra_reader_sb;
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
	const void *src, size_t len, ssize_t pagecpy)
{
	size_t sbidx, index;
	struct rchan_page *rpages;

	do {
		len -= pagecpy;
		src += pagecpy;
		offset += pagecpy;
		sbidx = offset >> buf->chan->subbuf_size_order;
		index = (offset & (buf->chan->subbuf_size - 1)) >> PAGE_SHIFT;

		/*
		 * Underlying layer should never ask for writes across
		 * subbuffers.
		 */
		WARN_ON(offset >= buf->chan->alloc_size);

		pagecpy = min_t(size_t, len, PAGE_SIZE - (offset & ~PAGE_MASK));
		rpages = buf->rchan_wsb[sbidx].pages;
		WARN_ON_ONCE(RCHAN_SB_IS_NOREF(rpages));
		ltt_relay_do_copy(rpages[index].virt + (offset & ~PAGE_MASK),
				  src, pagecpy);
	} while (unlikely(len != pagecpy));
}
EXPORT_SYMBOL_GPL(_ltt_relay_write);

/**
 * ltt_relay_read - read data from ltt_relay_buffer.
 * @buf : buffer
 * @offset : offset within the buffer
 * @dest : destination address
 * @len : length to write
 *
 * Should be protected by get_subbuf/put_subbuf.
 */
int ltt_relay_read(struct rchan_buf *buf, size_t offset,
	void *dest, size_t len)
{
	size_t index;
	ssize_t pagecpy, orig_len;
	struct rchan_page *rpages;

	orig_len = len;
	offset &= buf->chan->alloc_size - 1;
	index = (offset & (buf->chan->subbuf_size - 1)) >> PAGE_SHIFT;
	if (unlikely(!len))
		return 0;
	for (;;) {
		pagecpy = min_t(size_t, len, PAGE_SIZE - (offset & ~PAGE_MASK));
		rpages = buf->rchan_rsb.pages;
		WARN_ON_ONCE(RCHAN_SB_IS_NOREF(rpages));
		memcpy(dest, rpages[index].virt + (offset & ~PAGE_MASK),
		       pagecpy);
		len -= pagecpy;
		if (likely(!len))
			break;
		dest += pagecpy;
		offset += pagecpy;
		index = (offset & (buf->chan->subbuf_size - 1)) >> PAGE_SHIFT;
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
 * Should be protected by get_subbuf/put_subbuf.
 */
int ltt_relay_read_cstr(struct rchan_buf *buf, size_t offset,
		void *dest, size_t len)
{
	size_t index;
	ssize_t pagecpy, pagelen, strpagelen, orig_offset;
	char *str;
	struct rchan_page *rpages;

	offset &= buf->chan->alloc_size - 1;
	index = (offset & (buf->chan->subbuf_size - 1)) >> PAGE_SHIFT;
	orig_offset = offset;
	for (;;) {
		rpages = buf->rchan_rsb.pages;
		WARN_ON_ONCE(RCHAN_SB_IS_NOREF(rpages));
		str = (char *)rpages[index].virt + (offset & ~PAGE_MASK);
		pagelen = PAGE_SIZE - (offset & ~PAGE_MASK);
		strpagelen = strnlen(str, pagelen);
		if (len) {
			pagecpy = min_t(size_t, len, strpagelen);
			if (dest) {
				memcpy(dest, str, pagecpy);
				dest += pagecpy;
			}
			len -= pagecpy;
		}
		offset += strpagelen;
		index = (offset & (buf->chan->subbuf_size - 1)) >> PAGE_SHIFT;
		if (strpagelen < pagelen)
			break;
		/*
		 * Underlying layer should never ask for reads across
		 * subbuffers.
		 */
		WARN_ON(offset >= buf->chan->alloc_size);
	}
	if (dest && len)
		((char *)dest)[0] = 0;
	return offset - orig_offset;
}
EXPORT_SYMBOL_GPL(ltt_relay_read_cstr);

/**
 * ltt_relay_read_get_page - Get a whole page to read from
 * @buf : buffer
 * @offset : offset within the buffer
 *
 * Should be protected by get_subbuf/put_subbuf.
 */
struct page *ltt_relay_read_get_page(struct rchan_buf *buf, size_t offset)
{
	size_t index;
	struct rchan_page *rpages;

	offset &= buf->chan->alloc_size - 1;
	index = (offset & (buf->chan->subbuf_size - 1)) >> PAGE_SHIFT;
	rpages = buf->rchan_rsb.pages;
	WARN_ON_ONCE(RCHAN_SB_IS_NOREF(rpages));
	return rpages[index].page;
}
EXPORT_SYMBOL_GPL(ltt_relay_read_get_page);

/**
 * ltt_relay_read_offset_address - get address of a location within the buffer
 * @buf : buffer
 * @offset : offset within the buffer.
 *
 * Return the address where a given offset is located (for read).
 * Should be used to get the current subbuffer header pointer. Given we know
 * it's never on a page boundary, it's safe to write directly to this address,
 * as long as the write is never bigger than a page size.
 */
void *ltt_relay_read_offset_address(struct rchan_buf *buf, size_t offset)
{
	size_t index;
	struct rchan_page *rpages;

	offset &= buf->chan->alloc_size - 1;
	index = (offset & (buf->chan->subbuf_size - 1)) >> PAGE_SHIFT;
	rpages = buf->rchan_rsb.pages;
	WARN_ON_ONCE(RCHAN_SB_IS_NOREF(rpages));
	return rpages[index].virt + (offset & ~PAGE_MASK);
}
EXPORT_SYMBOL_GPL(ltt_relay_read_offset_address);

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
	size_t sbidx, index;
	struct rchan_page *rpages;

	offset &= buf->chan->alloc_size - 1;
	sbidx = offset >> buf->chan->subbuf_size_order;
	index = (offset & (buf->chan->subbuf_size - 1)) >> PAGE_SHIFT;
	rpages = buf->rchan_wsb[sbidx].pages;
	WARN_ON_ONCE(RCHAN_SB_IS_NOREF(rpages));
	return rpages[index].virt + (offset & ~PAGE_MASK);
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
