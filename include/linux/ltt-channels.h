#ifndef _LTT_CHANNELS_H
#define _LTT_CHANNELS_H

/*
 * Copyright (C) 2008 Mathieu Desnoyers (mathieu.desnoyers@polymtl.ca)
 *
 * Dynamic tracer channel allocation.

 * Dual LGPL v2.1/GPL v2 license.
 */

#include <linux/limits.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/timer.h>

#define EVENTS_PER_CHANNEL	65536

struct ltt_trace_struct;
struct rchan_buf;

struct ltt_channel_struct {
	/* First 32 bytes cache-hot cacheline */
	struct ltt_trace_struct	*trace;
	void *buf;
	void *trans_channel_data;
	int overwrite:1;
	int active:1;
	unsigned int n_subbufs_order;
	unsigned long commit_count_mask;	/*
						 * Commit count mask, removing
						 * the MSBs corresponding to
						 * bits used to represent the
						 * subbuffer index.
						 */
	/* End of first 32 bytes cacheline */

	/*
	 * buffer_begin - called on buffer-switch to a new sub-buffer
	 * @buf: the channel buffer containing the new sub-buffer
	 */
	void (*buffer_begin) (struct rchan_buf *buf,
			u64 tsc, unsigned int subbuf_idx);
	/*
	 * buffer_end - called on buffer-switch to a new sub-buffer
	 * @buf: the channel buffer containing the previous sub-buffer
	 */
	void (*buffer_end) (struct rchan_buf *buf,
			u64 tsc, unsigned int offset, unsigned int subbuf_idx);
	struct kref kref;	/* Channel transport reference count */
	struct ltt_channel_buf_access_ops *buf_access_ops;
	unsigned int subbuf_size;
	unsigned int subbuf_cnt;
	unsigned long switch_timer_interval;
	const char *channel_name;
} ____cacheline_aligned;

/*
 * ops for accessing struct channel data.
 * Only meant to be used in slow-path code (ascii formatter code,
 * buffer read-side access through a system call).
 */
struct ltt_channel_buf_access_ops {
	unsigned long (*get_offset)(struct rchan_buf *buf);
	unsigned long (*get_consumed)(struct rchan_buf *buf);
	int (*open)(struct rchan_buf *buf);
	int (*release)(struct rchan_buf *buf);
	int (*get_subbuf)(struct rchan_buf *buf, unsigned long *consumed);
	int (*put_subbuf)(struct rchan_buf *buf, unsigned long consumed);
	unsigned long (*get_n_subbufs)(struct rchan_buf *buf);
	unsigned long (*get_subbuf_size)(struct rchan_buf *buf);
	void (*start_switch_timer)(struct ltt_channel_struct *ltt_channel);
	void (*stop_switch_timer)(struct ltt_channel_struct *ltt_channel);
};

struct ltt_channel_setting {
	unsigned int subbuf_size;
	unsigned int subbuf_cnt;
	struct kref kref;	/* Number of references to structure content */
	struct list_head list;
	unsigned int index;	/* index of channel in trace channel array */
	u16 free_event_id;	/* Next event ID to allocate */
	char name[PATH_MAX];
};

int ltt_channels_register(const char *name);
int ltt_channels_unregister(const char *name);
int ltt_channels_set_default(const char *name,
			     unsigned int subbuf_size,
			     unsigned int subbuf_cnt);
const char *ltt_channels_get_name_from_index(unsigned int index);
int ltt_channels_get_index_from_name(const char *name);
int ltt_channels_trace_ref(void);
struct ltt_channel_struct *ltt_channels_trace_alloc(unsigned int *nr_channels,
						    int overwrite,
						    int active);
void ltt_channels_trace_free(struct ltt_channel_struct *channels,
	unsigned int nr_channels);
void ltt_channels_trace_set_timer(struct ltt_channel_struct *channel,
				  unsigned long interval);
void ltt_channels_trace_start_timer(struct ltt_channel_struct *channels,
	unsigned int nr_channels);
void ltt_channels_trace_stop_timer(struct ltt_channel_struct *channels,
	unsigned int nr_channels);

int _ltt_channels_get_event_id(const char *channel, const char *name);
int ltt_channels_get_event_id(const char *channel, const char *name);

#endif /* _LTT_CHANNELS_H */
