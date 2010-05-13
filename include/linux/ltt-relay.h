/*
 * linux/include/linux/ltt-relay.h
 *
 * Copyright (C) 2002, 2003 - Tom Zanussi (zanussi@us.ibm.com), IBM Corp
 * Copyright (C) 1999, 2000, 2001, 2002 - Karim Yaghmour (karim@opersys.com)
 * Copyright (C) 2008 - Mathieu Desnoyers (mathieu.desnoyers@polymtl.ca)
 *
 * CONFIG_RELAY definitions and declarations.
 *
 * Credits to Steven Rostedt for proposing to use an extra-subbuffer owned by
 * the reader in flight recorder mode.
 */

#ifndef _LINUX_LTT_RELAY_H
#define _LINUX_LTT_RELAY_H

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/kref.h>
#include <linux/mm.h>
#include <linux/ltt-core.h>

/* Use lowest pointer bit to show the sub-buffer has no reference. */
#define RCHAN_NOREF_FLAG	0x1UL

#define RCHAN_SB_IS_NOREF(x)	((unsigned long)(x) & RCHAN_NOREF_FLAG)
#define RCHAN_SB_SET_NOREF(x)	\
	(x = (struct rchan_page *)((unsigned long)(x) | RCHAN_NOREF_FLAG))
#define RCHAN_SB_CLEAR_NOREF(x)	\
	(x = (struct rchan_page *)((unsigned long)(x) & ~RCHAN_NOREF_FLAG))

/*
 * Tracks changes to rchan/rchan_buf structs
 */
#define LTT_RELAY_CHANNEL_VERSION		8

struct rchan_page {
	void *virt;			/* page virtual address (cached) */
	struct page *page;		/* pointer to page structure */
};

struct rchan_sb {
	struct rchan_page *pages;	/* Pointer to rchan pages for subbuf */
};

/*
 * Per-cpu relay channel buffer
 */
struct rchan_buf {
	void *chan_private;		/* private data for this buf */
	struct rchan_sb *rchan_wsb;	/* Array of rchan_sb for writer */
	struct rchan_sb rchan_rsb;	/* rchan_sb for reader */
	struct rchan *chan;		/* associated channel */
	struct dentry *dentry;		/* channel file dentry */
	struct kref kref;		/* channel buffer refcount */
	void **_virt;			/* Array of pointers to page addr */
	struct page **_pages;		/* Array of pointers to pages */
	unsigned int page_count;	/* number of current buffer pages */
	unsigned int cpu;		/* this buf's cpu */
	unsigned int random_access;	/* buffer performs random page access */
	struct dentry *ascii_dentry;	/* Text output dentry */
} ____cacheline_aligned;

/*
 * Relay channel data structure
 */
struct rchan {
	u32 version;			/* the version of this struct */
	size_t subbuf_size;		/* sub-buffer size */
	size_t n_subbufs;		/* number of sub-buffers per buffer */
	size_t alloc_size;		/* total buffer size allocated */
	struct rchan_callbacks *cb;	/* client callbacks */
	struct kref kref;		/* channel refcount */
	void *private_data;		/* for user-defined data */
	struct rchan_buf *buf[NR_CPUS]; /* per-cpu channel buffers */
	struct list_head list;		/* for channel list */
	struct dentry *parent;		/* parent dentry passed to open */
	int subbuf_size_order;		/* order of sub-buffer size */
	int extra_reader_sb;		/* bool: has extra reader subbuffer */
	char base_filename[NAME_MAX];	/* saved base filename */
};

/*
 * Relay channel client callbacks
 */
struct rchan_callbacks {
	/*
	 * subbuf_start - called on buffer-switch to a new sub-buffer
	 * @buf: the channel buffer containing the new sub-buffer
	 * @subbuf: the start of the new sub-buffer
	 * @prev_subbuf: the start of the previous sub-buffer
	 * @prev_padding: unused space at the end of previous sub-buffer
	 *
	 * The client should return 1 to continue logging, 0 to stop
	 * logging.
	 *
	 * NOTE: subbuf_start will also be invoked when the buffer is
	 *       created, so that the first sub-buffer can be initialized
	 *       if necessary.  In this case, prev_subbuf will be NULL.
	 *
	 * NOTE: the client can reserve bytes at the beginning of the new
	 *       sub-buffer by calling subbuf_start_reserve() in this callback.
	 */
	int (*subbuf_start) (struct rchan_buf *buf,
			     void *subbuf,
			     void *prev_subbuf,
			     size_t prev_padding);

	/*
	 * create_buf_file - create file to represent a relay channel buffer
	 * @filename: the name of the file to create
	 * @parent: the parent of the file to create
	 * @mode: the mode of the file to create
	 * @buf: the channel buffer
	 *
	 * Called during relay_open(), once for each per-cpu buffer,
	 * to allow the client to create a file to be used to
	 * represent the corresponding channel buffer.  If the file is
	 * created outside of relay, the parent must also exist in
	 * that filesystem.
	 *
	 * The callback should return the dentry of the file created
	 * to represent the relay buffer.
	 *
	 * Setting the is_global outparam to a non-zero value will
	 * cause relay_open() to create a single global buffer rather
	 * than the default set of per-cpu buffers.
	 *
	 * See Documentation/filesystems/relayfs.txt for more info.
	 */
	struct dentry *(*create_buf_file)(const char *filename,
					  struct dentry *parent,
					  int mode,
					  struct rchan_buf *buf);

	/*
	 * remove_buf_file - remove file representing a relay channel buffer
	 * @dentry: the dentry of the file to remove
	 *
	 * Called during relay_close(), once for each per-cpu buffer,
	 * to allow the client to remove a file used to represent a
	 * channel buffer.
	 *
	 * The callback should return 0 if successful, negative if not.
	 */
	int (*remove_buf_file)(struct dentry *dentry);
};

extern void _ltt_relay_write(struct rchan_buf *buf, size_t offset,
	const void *src, size_t len, ssize_t pagecpy);

extern int ltt_relay_read(struct rchan_buf *buf, size_t offset,
	void *dest, size_t len);

extern int ltt_relay_read_cstr(struct rchan_buf *buf, size_t offset,
	void *dest, size_t len);

extern struct page *ltt_relay_read_get_page(struct rchan_buf *buf,
	size_t offset);

/*
 * Return the address where a given offset is located.
 * Should be used to get the current subbuffer header pointer. Given we know
 * it's never on a page boundary, it's safe to write directly to this address,
 * as long as the write is never bigger than a page size.
 */
extern void *ltt_relay_offset_address(struct rchan_buf *buf,
	size_t offset);
extern void *ltt_relay_read_offset_address(struct rchan_buf *buf,
	size_t offset);

#ifdef CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS
static __inline__ void ltt_relay_do_copy(void *dest, const void *src, size_t len)
{
	switch (len) {
	case 0:
		break;
	case 1:
		*(u8 *)dest = *(const u8 *)src;
		break;
	case 2:
		*(u16 *)dest = *(const u16 *)src;
		break;
	case 4:
		*(u32 *)dest = *(const u32 *)src;
		break;
	case 8:
		*(u64 *)dest = *(const u64 *)src;
		break;
	default:
		/*
		 * What we really want here is an __inline__ memcpy, but we don't
		 * have constants, so gcc generally uses a function call.
		 */
		for (; len > 0; len--)
			*(u8 *)dest++ = *(const u8 *)src++;
	}
}
#else
/*
 * Returns whether the dest and src addresses are aligned on
 * min(sizeof(void *), len). Call this with statically known len for efficiency.
 */
static __inline__ int addr_aligned(const void *dest, const void *src, size_t len)
{
	if (ltt_align((size_t)dest, len))
		return 0;
	if (ltt_align((size_t)src, len))
		return 0;
	return 1;
}

static __inline__ void ltt_relay_do_copy(void *dest, const void *src, size_t len)
{
	switch (len) {
	case 0:
		break;
	case 1:
		*(u8 *)dest = *(const u8 *)src;
		break;
	case 2:
		if (unlikely(!addr_aligned(dest, src, 2)))
			goto memcpy_fallback;
		*(u16 *)dest = *(const u16 *)src;
		break;
	case 4:
		if (unlikely(!addr_aligned(dest, src, 4)))
			goto memcpy_fallback;
		*(u32 *)dest = *(const u32 *)src;
		break;
	case 8:
		if (unlikely(!addr_aligned(dest, src, 8)))
			goto memcpy_fallback;
		*(u64 *)dest = *(const u64 *)src;
		break;
	default:
		goto memcpy_fallback;
	}
	return;

memcpy_fallback:
	/*
	 * What we really want here is an inline memcpy, but we don't
	 * have constants, so gcc generally uses a function call.
	 */
	for (; len > 0; len--)
		*(u8 *)dest++ = *(const u8 *)src++;
}
#endif

static __inline__ int ltt_relay_write(struct rchan_buf *buf, size_t offset,
	const void *src, size_t len)
{
	size_t sbidx, index;
	ssize_t pagecpy;
	struct rchan_page *rpages;

	offset &= buf->chan->alloc_size - 1;
	sbidx = offset >> buf->chan->subbuf_size_order;
	index = (offset & (buf->chan->subbuf_size - 1)) >> PAGE_SHIFT;
	pagecpy = min_t(size_t, len, (- offset) & ~PAGE_MASK);
	rpages = buf->rchan_wsb[sbidx].pages;
	WARN_ON_ONCE(RCHAN_SB_IS_NOREF(rpages));
	ltt_relay_do_copy(rpages[index].virt + (offset & ~PAGE_MASK),
			  src, pagecpy);

	if (unlikely(len != pagecpy))
		_ltt_relay_write(buf, offset, src, len, pagecpy);
	return len;
}

/**
 * ltt_clear_noref_flag - Clear the noref subbuffer flag, for writer.
 */
static __inline__ void ltt_clear_noref_flag(struct rchan *rchan,
					    struct rchan_buf *buf,
					    long idx)
{
	struct rchan_page *sb_pages, *new_sb_pages;

	sb_pages = buf->rchan_wsb[idx].pages;
	for (;;) {
		if (!RCHAN_SB_IS_NOREF(sb_pages))
			return;	/* Already writing to this buffer */
		new_sb_pages = sb_pages;
		RCHAN_SB_CLEAR_NOREF(new_sb_pages);
		new_sb_pages = cmpxchg(&buf->rchan_wsb[idx].pages,
			sb_pages, new_sb_pages);
		if (likely(new_sb_pages == sb_pages))
			break;
		sb_pages = new_sb_pages;
	}
}

/**
 * ltt_set_noref_flag - Set the noref subbuffer flag, for writer.
 */
static __inline__ void ltt_set_noref_flag(struct rchan *rchan,
					  struct rchan_buf *buf,
					  long idx)
{
	struct rchan_page *sb_pages, *new_sb_pages;

	sb_pages = buf->rchan_wsb[idx].pages;
	for (;;) {
		if (RCHAN_SB_IS_NOREF(sb_pages))
			return;	/* Already set */
		new_sb_pages = sb_pages;
		RCHAN_SB_SET_NOREF(new_sb_pages);
		new_sb_pages = cmpxchg(&buf->rchan_wsb[idx].pages,
			sb_pages, new_sb_pages);
		if (likely(new_sb_pages == sb_pages))
			break;
		sb_pages = new_sb_pages;
	}
}

/**
 * update_read_sb_index - Read-side subbuffer index update.
 */
static __inline__ int update_read_sb_index(struct rchan_buf *buf,
					   long consumed_idx)
{
	struct rchan_page *old_wpage, *new_wpage;

	if (unlikely(buf->chan->extra_reader_sb)) {
		/*
		 * Exchange the target writer subbuffer with our own unused
		 * subbuffer.
		 */
		old_wpage = buf->rchan_wsb[consumed_idx].pages;
		if (unlikely(!RCHAN_SB_IS_NOREF(old_wpage)))
			return -EAGAIN;
		WARN_ON_ONCE(!RCHAN_SB_IS_NOREF(buf->rchan_rsb.pages));
		new_wpage = cmpxchg(&buf->rchan_wsb[consumed_idx].pages,
				old_wpage,
				buf->rchan_rsb.pages);
		if (unlikely(old_wpage != new_wpage))
			return -EAGAIN;
		buf->rchan_rsb.pages = new_wpage;
		RCHAN_SB_CLEAR_NOREF(buf->rchan_rsb.pages);
	} else {
		/* No page exchange, use the writer page directly */
		buf->rchan_rsb.pages = buf->rchan_wsb[consumed_idx].pages;
		RCHAN_SB_CLEAR_NOREF(buf->rchan_rsb.pages);
	}
	return 0;
}

/*
 * CONFIG_LTT_RELAY kernel API, ltt/ltt-relay-alloc.c
 */

struct rchan *ltt_relay_open(const char *base_filename,
			 struct dentry *parent,
			 size_t subbuf_size,
			 size_t n_subbufs,
			 struct rchan_callbacks *cb,
			 void *private_data,
			 int extra_reader_sb);
extern void ltt_relay_close(struct rchan *chan);

void ltt_relay_get_chan(struct rchan *chan);
void ltt_relay_put_chan(struct rchan *chan);

void ltt_relay_get_chan_buf(struct rchan_buf *buf);
void ltt_relay_put_chan_buf(struct rchan_buf *buf);

/*
 * exported ltt_relay file operations, ltt/ltt-relay-alloc.c
 */
extern const struct file_operations ltt_relay_file_operations;

#endif /* _LINUX_LTT_RELAY_H */

