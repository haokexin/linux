/*
 * Copyright (C) 2008-2009 VA Linux Systems Japan K.K.
 *
 *  I/O bandwidth control
 *
 * This file is released under the GPL.
 */

#ifndef DM_IOBAND_H
#define DM_IOBAND_H

#include <linux/version.h>
#include <linux/wait.h>

#define DM_MSG_PREFIX "ioband"

#define DEFAULT_IO_THROTTLE	4
#define IOBAND_NAME_MAX		31
#define IOBAND_ID_ANY		(-1)
#define POLICY_PARAM_START	6
#define POLICY_PARAM_DELIM	"=:,"

#define MAX_BW_OVER             1
#define MAX_BW_UNDER            0
#define NO_IO_MODE              4

#define TIME_COMPENSATOR        10

struct ioband_group;

struct ioband_device {
	struct list_head g_groups;
	struct delayed_work g_conductor;
	struct workqueue_struct *g_ioband_wq;
	struct bio_list g_urgent_bios;
	int g_io_throttle;
	int g_io_limit;
	int g_issued[2];
	int g_blocked[2];
	spinlock_t g_lock;
	wait_queue_head_t g_waitq[2];
	wait_queue_head_t g_waitq_suspend;
	wait_queue_head_t g_waitq_flush;

	int g_ref;
	struct list_head g_list;
	struct list_head g_heads;
	struct list_head g_root_groups;
	int g_flags;
	char g_name[IOBAND_NAME_MAX + 1];
	const struct ioband_policy_type *g_policy;

	/* policy dependent */
	int (*g_can_submit) (struct ioband_group *);
	int (*g_prepare_bio) (struct ioband_group *, struct bio *, int);
	int (*g_restart_bios) (struct ioband_device *);
	void (*g_hold_bio) (struct ioband_group *, struct bio *);
	struct bio *(*g_pop_bio) (struct ioband_group *);
	int (*g_group_ctr) (struct ioband_group *, const char *);
	void (*g_group_dtr) (struct ioband_group *);
	int (*g_set_param) (struct ioband_group *, const char *, const char *);
	int (*g_should_block) (struct ioband_group *, int);
	void (*g_show) (struct ioband_group *, int *, char *, unsigned);
	void (*g_show_device) (struct seq_file *, struct ioband_device *);
	void (*g_show_group) (struct seq_file *, struct ioband_group *);

	/* members for weight balancing policy */
	int g_epoch;
	int g_weight_total;
	/* the number of tokens which can be used in every epoch */
	int g_token_bucket;
	/* how many epochs tokens can be carried over */
	int g_carryover;
	/* how many tokens should be used for one page-sized I/O */
	int g_token_unit;
	/* the last group which used a token */
	struct ioband_group *g_current;
	/* give another group a chance to be scheduled when the rest
	   of tokens of the current group reaches this mark */
	int g_yield_mark;
	/* the latest group which used up its tokens */
	struct ioband_group *g_expired;
	/* the group which has the largest number of tokens in the
	   active groups */
	struct ioband_group *g_dominant;
	/* the number of unused tokens in this epoch */
	int g_token_left;
	/* left-over tokens from the previous epoch */
	int g_token_extra;

	/* members for range-bw policy */
	int     g_min_bw_total;
	int     g_max_bw_total;
	unsigned long   g_next_time_period;
	int     g_time_period_expired;
	struct ioband_group *g_running_gp;
	int     g_total_min_bw_token;
	int     g_consumed_min_bw_token;
	int     g_io_mode;

};

struct ioband_group {
	struct list_head c_list;
	struct list_head c_heads;
	struct list_head c_sibling;
	struct list_head c_children;
	struct ioband_group *c_parent;
	struct ioband_device *c_banddev;
	struct dm_dev *c_dev;
	struct dm_target *c_target;
	struct bio_list c_blocked_bios;
	struct bio_list c_prio_bios;
	struct rb_root c_group_root;
	struct rb_node c_group_node;
	int c_id;	/* should be unsigned long or unsigned long long */
	char c_name[IOBAND_NAME_MAX + 1];	/* rfu */
	int c_blocked[2];
	int c_prio_blocked;
	wait_queue_head_t c_waitq[2];
	int c_flags;
	struct disk_stats c_stats;		/* hold rd/wr status */
	const struct ioband_group_type *c_type;

	/* members for weight balancing policy */
	int c_weight;
	int c_my_epoch;
	int c_token;
	int c_token_initial;
	int c_token_bucket;
	int c_limit;
	int c_limit_bucket;
	int c_consumed;

	/* rfu */
	/* struct bio_list	c_ordered_tag_bios; */

	/* members for range-bw policy */
	wait_queue_head_t       c_max_bw_over_waitq;
	struct timer_list *c_timer;
	int     timer_set;
	int     c_min_bw;
	int     c_max_bw;
	int     c_time_slice_expired;
	int     c_min_bw_token;
	int     c_max_bw_token;
	int     c_consumed_min_bw_token;
	int     c_is_over_max_bw;
	int     c_io_mode;
	unsigned long   c_time_slice;
	unsigned long   c_time_slice_start;
	unsigned long   c_time_slice_end;
	int     c_wait_p_count;

};

struct blkio_cgroup;

struct ioband_cgroup_ops {
	void (*show_device)(struct seq_file *);
	int (*config_device)(int, char **);
	void (*show_group)(struct seq_file *, int, int);
	int (*config_group)(int, char **, int, int);
	int (*reset_group_stats)(int, char **, int);
	void (*remove_group)(int);
};

#define IOG_INFO_CONFIG	0
#define IOG_INFO_STATS	1

#define IOBAND_URGENT 1

#define DEV_BIO_BLOCKED_ASYNC	1
#define DEV_BIO_BLOCKED_SYNC	2
#define DEV_SUSPENDED		4

#define set_device_blocked(dp, sync)	\
	((dp)->g_flags |=		\
		((sync) ? DEV_BIO_BLOCKED_SYNC : DEV_BIO_BLOCKED_ASYNC))
#define clear_device_blocked(dp, sync)	\
	((dp)->g_flags &=		\
		((sync) ? ~DEV_BIO_BLOCKED_SYNC : ~DEV_BIO_BLOCKED_ASYNC))
#define is_device_blocked(dp, sync)	\
	((dp)->g_flags &		\
		((sync) ? DEV_BIO_BLOCKED_SYNC : DEV_BIO_BLOCKED_ASYNC))

#define set_device_suspended(dp)	((dp)->g_flags |= DEV_SUSPENDED)
#define clear_device_suspended(dp)	((dp)->g_flags &= ~DEV_SUSPENDED)
#define is_device_suspended(dp)		((dp)->g_flags & DEV_SUSPENDED)

#define IOG_PRIO_BIO_SYNC	1
#define IOG_PRIO_QUEUE		2
#define IOG_BIO_BLOCKED_ASYNC	4
#define IOG_BIO_BLOCKED_SYNC	8
#define IOG_GOING_DOWN		16
#define IOG_SUSPENDED		32
#define IOG_NEED_UP		64

#define R_OK		0
#define R_BLOCK		1
#define R_YIELD		2

#define set_group_blocked(gp, sync)	\
	((gp)->c_flags |=		\
		((sync) ? IOG_BIO_BLOCKED_SYNC : IOG_BIO_BLOCKED_ASYNC))
#define clear_group_blocked(gp, sync)	\
	((gp)->c_flags &=		\
		((sync) ? ~IOG_BIO_BLOCKED_SYNC : ~IOG_BIO_BLOCKED_ASYNC))
#define is_group_blocked(gp, sync)	\
	((gp)->c_flags &		\
		((sync) ? IOG_BIO_BLOCKED_SYNC : IOG_BIO_BLOCKED_ASYNC))

#define set_group_down(gp)		((gp)->c_flags |= IOG_GOING_DOWN)
#define clear_group_down(gp)		((gp)->c_flags &= ~IOG_GOING_DOWN)
#define is_group_down(gp)		((gp)->c_flags & IOG_GOING_DOWN)

#define set_group_suspended(gp)		((gp)->c_flags |= IOG_SUSPENDED)
#define clear_group_suspended(gp)	((gp)->c_flags &= ~IOG_SUSPENDED)
#define is_group_suspended(gp)		((gp)->c_flags & IOG_SUSPENDED)

#define set_group_need_up(gp)		((gp)->c_flags |= IOG_NEED_UP)
#define clear_group_need_up(gp)		((gp)->c_flags &= ~IOG_NEED_UP)
#define group_need_up(gp)		((gp)->c_flags & IOG_NEED_UP)

#define set_prio_async(gp)		((gp)->c_flags |= IOG_PRIO_QUEUE)
#define clear_prio_async(gp)		((gp)->c_flags &= ~IOG_PRIO_QUEUE)
#define is_prio_async(gp) \
	((gp)->c_flags & (IOG_PRIO_QUEUE|IOG_PRIO_BIO_SYNC) == IOG_PRIO_QUEUE)

#define set_prio_sync(gp) \
	((gp)->c_flags |= (IOG_PRIO_QUEUE|IOG_PRIO_BIO_SYNC))
#define clear_prio_sync(gp) \
	((gp)->c_flags &= ~(IOG_PRIO_QUEUE|IOG_PRIO_BIO_SYNC))
#define is_prio_sync(gp) \
	((gp)->c_flags & (IOG_PRIO_QUEUE|IOG_PRIO_BIO_SYNC) == \
		(IOG_PRIO_QUEUE|IOG_PRIO_BIO_SYNC))

#define set_prio_queue(gp, sync) \
	((gp)->c_flags |= (IOG_PRIO_QUEUE|sync))
#define clear_prio_queue(gp)		clear_prio_sync(gp)
#define is_prio_queue(gp)		((gp)->c_flags & IOG_PRIO_QUEUE)
#define prio_queue_sync(gp)		((gp)->c_flags & IOG_PRIO_BIO_SYNC)

#define nr_issued(dp) \
	((dp)->g_issued[BLK_RW_SYNC] + (dp)->g_issued[BLK_RW_ASYNC])
#define nr_blocked(dp) \
	((dp)->g_blocked[BLK_RW_SYNC] + (dp)->g_blocked[BLK_RW_ASYNC])
#define nr_blocked_group(gp) \
	((gp)->c_blocked[BLK_RW_SYNC] + (gp)->c_blocked[BLK_RW_ASYNC])

struct ioband_policy_type {
	const char *p_name;
	int (*p_policy_init) (struct ioband_device *, int, char **);
};

extern const struct ioband_policy_type dm_ioband_policy_type[];

struct ioband_group_type {
	const char *t_name;
	int (*t_getid) (struct bio *);
};

extern const struct ioband_group_type dm_ioband_group_type[];

extern int policy_range_bw_init(struct ioband_device *, int, char **);

#endif /* DM_IOBAND_H */
