/*
 * Copyright (C) 2008-2009 VA Linux Systems Japan K.K.
 * Authors: Hirokazu Takahashi <taka@valinux.co.jp>
 *          Ryo Tsuruta <ryov@valinux.co.jp>
 *
 *  I/O bandwidth control
 *
 * Some blktrace messages were added by Alan D. Brunelle <Alan.Brunelle@hp.com>
 *
 * This file is released under the GPL.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/bio.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/rbtree.h>
#include <linux/biotrack.h>
#include <linux/dm-ioctl.h>
#include "dm.h"
#include "md.h"
#include "dm-ioband.h"

#define CREATE_TRACE_POINTS
#include <trace/events/dm-ioband.h>

static LIST_HEAD(ioband_device_list);
/* lock up during configuration */
static DEFINE_MUTEX(ioband_lock);

static void suspend_ioband_device(struct ioband_device *, unsigned long, int);
static void resume_ioband_device(struct ioband_device *);
static void ioband_conduct(struct work_struct *);
static void ioband_hold_bio(struct ioband_group *, struct bio *);
static struct bio *ioband_pop_bio(struct ioband_group *);
static int ioband_set_param(struct ioband_group *, const char *, const char *);
static int ioband_group_attach(struct ioband_group *, int, int, const char *);
static int ioband_group_type_select(struct ioband_group *, const char *);

static void do_nothing(void) {}

static int policy_init(struct ioband_device *dp, const char *name,
						int argc, char **argv)
{
	const struct ioband_policy_type *p;
	struct ioband_group *gp;
	unsigned long flags;
	int r;

	for (p = dm_ioband_policy_type; p->p_name; p++) {
		if (!strcmp(name, p->p_name))
			break;
	}
	if (!p->p_name)
		return -EINVAL;
	/* do nothing if the same policy is already set */
	if (dp->g_policy == p)
		return 0;

	spin_lock_irqsave(&dp->g_lock, flags);
	suspend_ioband_device(dp, flags, 1);
	list_for_each_entry(gp, &dp->g_groups, c_list)
		dp->g_group_dtr(gp);

	/* switch to the new policy */
	dp->g_policy = p;
	r = p->p_policy_init(dp, argc, argv);
	if (!r) {
		if (!dp->g_hold_bio)
			dp->g_hold_bio = ioband_hold_bio;
		if (!dp->g_pop_bio)
			dp->g_pop_bio = ioband_pop_bio;

		list_for_each_entry(gp, &dp->g_groups, c_list)
			dp->g_group_ctr(gp, NULL);
	}
	resume_ioband_device(dp);
	spin_unlock_irqrestore(&dp->g_lock, flags);
	return r;
}

static struct ioband_device *alloc_ioband_device(const char *name,
						int io_throttle, int io_limit)
{
	struct ioband_device *dp, *new_dp;

	new_dp = kzalloc(sizeof(struct ioband_device), GFP_KERNEL);
	if (!new_dp)
		return NULL;

	/*
	 * Prepare its own workqueue as generic_make_request() may
	 * potentially block the workqueue when submitting BIOs.
	 */
	new_dp->g_ioband_wq = create_workqueue("kioband");
	if (!new_dp->g_ioband_wq) {
		kfree(new_dp);
		return NULL;
	}

	list_for_each_entry(dp, &ioband_device_list, g_list) {
		if (!strcmp(dp->g_name, name)) {
			dp->g_ref++;
			destroy_workqueue(new_dp->g_ioband_wq);
			kfree(new_dp);
			return dp;
		}
	}

	INIT_DELAYED_WORK(&new_dp->g_conductor, ioband_conduct);
	INIT_LIST_HEAD(&new_dp->g_groups);
	INIT_LIST_HEAD(&new_dp->g_list);
	INIT_LIST_HEAD(&new_dp->g_heads);
	INIT_LIST_HEAD(&new_dp->g_root_groups);
	spin_lock_init(&new_dp->g_lock);
	bio_list_init(&new_dp->g_urgent_bios);
	new_dp->g_io_throttle = io_throttle;
	new_dp->g_io_limit = io_limit;
	new_dp->g_issued[BLK_RW_SYNC] = 0;
	new_dp->g_issued[BLK_RW_ASYNC] = 0;
	new_dp->g_blocked[BLK_RW_SYNC] = 0;
	new_dp->g_blocked[BLK_RW_ASYNC] = 0;
	new_dp->g_ref = 1;
	new_dp->g_flags = 0;
	strlcpy(new_dp->g_name, name, sizeof(new_dp->g_name));
	new_dp->g_policy = NULL;
	new_dp->g_hold_bio = NULL;
	new_dp->g_pop_bio = NULL;
	init_waitqueue_head(&new_dp->g_waitq[BLK_RW_ASYNC]);
	init_waitqueue_head(&new_dp->g_waitq[BLK_RW_SYNC]);
	init_waitqueue_head(&new_dp->g_waitq_suspend);
	init_waitqueue_head(&new_dp->g_waitq_flush);
	list_add_tail(&new_dp->g_list, &ioband_device_list);
	return new_dp;
}

static void release_ioband_device(struct ioband_device *dp)
{
	dp->g_ref--;
	if (dp->g_ref > 0)
		return;
	list_del(&dp->g_list);
	destroy_workqueue(dp->g_ioband_wq);
	kfree(dp);
}

static int is_ioband_device_flushed(struct ioband_device *dp,
				    int wait_completion)
{
	struct ioband_group *gp;

	if (wait_completion && nr_issued(dp) > 0)
		return 0;
	if (nr_blocked(dp) ||
	    waitqueue_active(&dp->g_waitq[BLK_RW_ASYNC]) ||
	    waitqueue_active(&dp->g_waitq[BLK_RW_SYNC]))
		return 0;
	list_for_each_entry(gp, &dp->g_groups, c_list)
		if (waitqueue_active(&gp->c_waitq[BLK_RW_ASYNC]) ||
		    waitqueue_active(&gp->c_waitq[BLK_RW_SYNC]))
			return 0;
	return 1;
}

static void suspend_ioband_device(struct ioband_device *dp,
				  unsigned long flags, int wait_completion)
{
	struct ioband_group *gp;

	/* block incoming bios */
	set_device_suspended(dp);

	/* wake up all blocked processes and go down all ioband groups */
	wake_up_all(&dp->g_waitq[BLK_RW_ASYNC]);
	wake_up_all(&dp->g_waitq[BLK_RW_SYNC]);
	list_for_each_entry(gp, &dp->g_groups, c_list) {
		if (!is_group_down(gp)) {
			set_group_down(gp);
			set_group_need_up(gp);
		}
		wake_up_all(&gp->c_waitq[BLK_RW_ASYNC]);
		wake_up_all(&gp->c_waitq[BLK_RW_SYNC]);
	}

	/* flush the already mapped bios */
	spin_unlock_irqrestore(&dp->g_lock, flags);
	queue_delayed_work(dp->g_ioband_wq, &dp->g_conductor, 0);
	flush_workqueue(dp->g_ioband_wq);

	/* wait for all processes to wake up and bios to release */
	spin_lock_irqsave(&dp->g_lock, flags);
	wait_event_lock_irq(dp->g_waitq_flush,
			    is_ioband_device_flushed(dp, wait_completion),
			    dp->g_lock, do_nothing());
}

static void resume_ioband_device(struct ioband_device *dp)
{
	struct ioband_group *gp;

	/* go up ioband groups */
	list_for_each_entry(gp, &dp->g_groups, c_list) {
		if (group_need_up(gp)) {
			clear_group_need_up(gp);
			clear_group_down(gp);
		}
	}

	/* accept incoming bios */
	wake_up_all(&dp->g_waitq_suspend);
	clear_device_suspended(dp);
}

static struct ioband_group *ioband_group_find(struct ioband_group *head, int id)
{
	struct rb_node *node = head->c_group_root.rb_node;

	while (node) {
		struct ioband_group *p =
			rb_entry(node, struct ioband_group, c_group_node);

		if (p->c_id == id || id == IOBAND_ID_ANY)
			return p;
		node = (id < p->c_id) ? node->rb_left : node->rb_right;
	}
	return NULL;
}

static void ioband_group_add_node(struct rb_root *root, struct ioband_group *gp)
{
	struct rb_node **node = &root->rb_node, *parent = NULL;
	struct ioband_group *p;

	while (*node) {
		p = rb_entry(*node, struct ioband_group, c_group_node);
		parent = *node;
		node = (gp->c_id < p->c_id) ?
				&(*node)->rb_left : &(*node)->rb_right;
	}

	rb_link_node(&gp->c_group_node, parent, node);
	rb_insert_color(&gp->c_group_node, root);
}

static int ioband_group_init(struct ioband_device *dp,
			     struct ioband_group *head,
			     struct ioband_group *parent,
			     struct ioband_group *gp,
			     int id, const char *param)
{
	unsigned long flags;
	int r;

	INIT_LIST_HEAD(&gp->c_list);
	INIT_LIST_HEAD(&gp->c_heads);
	INIT_LIST_HEAD(&gp->c_sibling);
	INIT_LIST_HEAD(&gp->c_children);
	gp->c_parent = parent;
	bio_list_init(&gp->c_blocked_bios);
	bio_list_init(&gp->c_prio_bios);
	gp->c_id = id;	/* should be verified */
	gp->c_blocked[BLK_RW_ASYNC] = 0;
	gp->c_blocked[BLK_RW_SYNC] = 0;
	gp->c_prio_blocked = 0;
	memset(&gp->c_stats, 0, sizeof(gp->c_stats));
	init_waitqueue_head(&gp->c_waitq[BLK_RW_ASYNC]);
	init_waitqueue_head(&gp->c_waitq[BLK_RW_SYNC]);
	gp->c_flags = 0;
	gp->c_group_root = RB_ROOT;
	gp->c_banddev = dp;

	spin_lock_irqsave(&dp->g_lock, flags);
	if (head && ioband_group_find(head, id)) {
		spin_unlock_irqrestore(&dp->g_lock, flags);
		DMWARN("%s: id=%d already exists.", __func__, id);
		return -EEXIST;
	}

	list_add_tail(&gp->c_list, &dp->g_groups);

	if (!parent)
		list_add_tail(&gp->c_sibling, &dp->g_root_groups);
	else
		list_add_tail(&gp->c_sibling, &parent->c_children);

	r = dp->g_group_ctr(gp, param);
	if (r) {
		list_del(&gp->c_list);
		list_del(&gp->c_sibling);
		spin_unlock_irqrestore(&dp->g_lock, flags);
		return r;
	}

	if (head) {
		ioband_group_add_node(&head->c_group_root, gp);
		gp->c_dev = head->c_dev;
		gp->c_target = head->c_target;
	} else
		list_add_tail(&gp->c_heads, &dp->g_heads);

	spin_unlock_irqrestore(&dp->g_lock, flags);
	return 0;
}

static void ioband_group_release(struct ioband_group *head,
				 struct ioband_group *gp)
{
	struct ioband_device *dp = gp->c_banddev;

	list_del(&gp->c_list);
	list_del(&gp->c_sibling);
	if (head)
		rb_erase(&gp->c_group_node, &head->c_group_root);
	else
		list_del(&gp->c_heads);
	dp->g_group_dtr(gp);
	kfree(gp);
}

static void ioband_group_destroy_all(struct ioband_group *gp)
{
	struct ioband_device *dp = gp->c_banddev;
	struct ioband_group *p;
	unsigned long flags;

	spin_lock_irqsave(&dp->g_lock, flags);
	while ((p = ioband_group_find(gp, IOBAND_ID_ANY)))
		ioband_group_release(gp, p);
	ioband_group_release(NULL, gp);
	spin_unlock_irqrestore(&dp->g_lock, flags);
}

static void ioband_group_stop_all(struct ioband_group *head, int suspend)
{
	struct ioband_device *dp = head->c_banddev;
	struct ioband_group *p;
	struct rb_node *node;
	unsigned long flags;

	spin_lock_irqsave(&dp->g_lock, flags);
	for (node = rb_first(&head->c_group_root); node; node = rb_next(node)) {
		p = rb_entry(node, struct ioband_group, c_group_node);
		set_group_down(p);
		if (suspend)
			set_group_suspended(p);
	}
	set_group_down(head);
	if (suspend)
		set_group_suspended(head);
	spin_unlock_irqrestore(&dp->g_lock, flags);
	queue_delayed_work(dp->g_ioband_wq, &dp->g_conductor, 0);
	flush_workqueue(dp->g_ioband_wq);
}

static void ioband_group_resume_all(struct ioband_group *head)
{
	struct ioband_device *dp = head->c_banddev;
	struct ioband_group *p;
	struct rb_node *node;
	unsigned long flags;

	spin_lock_irqsave(&dp->g_lock, flags);
	for (node = rb_first(&head->c_group_root); node; node = rb_next(node)) {
		p = rb_entry(node, struct ioband_group, c_group_node);
		clear_group_down(p);
		clear_group_suspended(p);
	}
	clear_group_down(head);
	clear_group_suspended(head);
	spin_unlock_irqrestore(&dp->g_lock, flags);
}

static int parse_group_param(const char *param, long *id, char const **value)
{
	char *s, *endp;
	long n;

	s = strpbrk(param, POLICY_PARAM_DELIM);
	if (!s) {
		*id = IOBAND_ID_ANY;
		*value = param;
		return 0;
	}

	n = simple_strtol(param, &endp, 0);
	if (endp != s)
		return -EINVAL;

	*id = (endp == param) ? IOBAND_ID_ANY : n;
	*value = endp + 1;
	return 0;
}

/*
 * Create a new band device:
 *   parameters:  <device> <device-group-id> <io_throttle> <io_limit>
 *     <type> <policy> <policy-param...> <group-id:group-param...>
 */
static int ioband_ctr(struct dm_target *ti, unsigned argc, char **argv)
{
	struct ioband_group *gp;
	struct ioband_device *dp;
	struct dm_dev *dev;
	int io_throttle;
	int io_limit;
	int i, r, start;
	long val, id;
	const char *param;
	char *s;

	if (argc < POLICY_PARAM_START) {
		ti->error = "Requires " __stringify(POLICY_PARAM_START)
							" or more arguments";
		return -EINVAL;
	}

	if (strlen(argv[1]) > IOBAND_NAME_MAX) {
		ti->error = "Ioband device name is too long";
		return -EINVAL;
	}

	r = strict_strtol(argv[2], 0, &val);
	if (r || val < 0 || val > SHORT_MAX) {
		ti->error = "Invalid io_throttle";
		return -EINVAL;
	}
	io_throttle = (val == 0) ? DEFAULT_IO_THROTTLE : val;

	r = strict_strtol(argv[3], 0, &val);
	if (r || val < 0 || val > SHORT_MAX) {
		ti->error = "Invalid io_limit";
		return -EINVAL;
	}
	io_limit = val;

	r = dm_get_device(ti, argv[0], dm_table_get_mode(ti->table), &dev);
	if (r) {
		ti->error = "Device lookup failed";
		return r;
	}

	if (io_limit == 0) {
		struct request_queue *q;

		q = bdev_get_queue(dev->bdev);
		if (!q) {
			ti->error = "Can't get queue size";
			r = -ENXIO;
			goto release_dm_device;
		}
		/*
		 * The block layer accepts I/O requests up to 50% over
		 * nr_requests when the requests are issued from a
		 * "batcher" process.
		 */
		io_limit = (3 * q->nr_requests / 2);
	}

	if (io_limit < io_throttle)
		io_limit = io_throttle;

	mutex_lock(&ioband_lock);
	dp = alloc_ioband_device(argv[1], io_throttle, io_limit);
	if (!dp) {
		ti->error = "Cannot create ioband device";
		r = -EINVAL;
		mutex_unlock(&ioband_lock);
		goto release_dm_device;
	}

	r = policy_init(dp, argv[POLICY_PARAM_START - 1],
			argc - POLICY_PARAM_START, &argv[POLICY_PARAM_START]);
	if (r) {
		ti->error = "Invalid policy parameter";
		goto release_ioband_device;
	}

	gp = kzalloc(sizeof(struct ioband_group), GFP_KERNEL);
	if (!gp) {
		ti->error = "Cannot allocate memory for ioband group";
		r = -ENOMEM;
		goto release_ioband_device;
	}

	ti->num_flush_requests = 1;
	ti->private = gp;
	gp->c_target = ti;
	gp->c_dev = dev;

	/* Find a default group parameter */
	for (start = POLICY_PARAM_START; start < argc; start++) {
		s = strpbrk(argv[start], POLICY_PARAM_DELIM);
		if (s == argv[start])
			break;
	}
	param = (start < argc) ? &argv[start][1] : NULL;

	/* Create a default ioband group */
	r = ioband_group_init(dp, NULL, NULL, gp, IOBAND_ID_ANY, param);
	if (r) {
		kfree(gp);
		ti->error = "Cannot create default ioband group";
		goto release_ioband_device;
	}

	r = ioband_group_type_select(gp, argv[4]);
	if (r) {
		ti->error = "Cannot set ioband group type";
		goto release_ioband_group;
	}

	/* Create sub ioband groups */
	for (i = start + 1; i < argc; i++) {
		r = parse_group_param(argv[i], &id, &param);
		if (r) {
			ti->error = "Invalid ioband group parameter";
			goto release_ioband_group;
		}
		r = ioband_group_attach(gp, 0, id, param);
		if (r) {
			ti->error = "Cannot create ioband group";
			goto release_ioband_group;
		}
	}
	mutex_unlock(&ioband_lock);
	return 0;

release_ioband_group:
	ioband_group_destroy_all(gp);
release_ioband_device:
	release_ioband_device(dp);
	mutex_unlock(&ioband_lock);
release_dm_device:
	dm_put_device(ti, dev);
	return r;
}

static void ioband_dtr(struct dm_target *ti)
{
	struct ioband_group *gp = ti->private;
	struct ioband_device *dp = gp->c_banddev;
	struct dm_dev *dev = gp->c_dev;

	mutex_lock(&ioband_lock);

	ioband_group_stop_all(gp, 0);
	cancel_delayed_work_sync(&dp->g_conductor);
	ioband_group_destroy_all(gp);

	release_ioband_device(dp);
	mutex_unlock(&ioband_lock);

	dm_put_device(ti, dev);
}

static void ioband_hold_bio(struct ioband_group *gp, struct bio *bio)
{
	/* Todo: The list should be split into a sync list and an async list */
	bio_list_add(&gp->c_blocked_bios, bio);
}

static struct bio *ioband_pop_bio(struct ioband_group *gp)
{
	return bio_list_pop(&gp->c_blocked_bios);
}

static int is_urgent_bio(struct bio *bio)
{
	struct page *page = bio_iovec_idx(bio, 0)->bv_page;
	/*
	 * ToDo: A new flag should be added to struct bio, which indicates
	 *       it contains urgent I/O requests.
	 */
	if (!PageReclaim(page))
		return 0;
	if (PageSwapCache(page))
		return 2;
	return 1;
}

static inline int device_should_block(struct ioband_group *gp, int sync)
{
	struct ioband_device *dp = gp->c_banddev;

	if (is_group_down(gp))
		return 0;
	if (is_device_blocked(dp, sync))
		return 1;
	if (dp->g_blocked[sync] >= dp->g_io_limit) {
		set_device_blocked(dp, sync);
		return 1;
	}
	return 0;
}

static inline int group_should_block(struct ioband_group *gp, int sync)
{
	struct ioband_device *dp = gp->c_banddev;

	if (is_group_down(gp))
		return 0;
	if (is_group_blocked(gp, sync))
		return 1;
	if (dp->g_should_block(gp, sync)) {
		set_group_blocked(gp, sync);
		return 1;
	}
	return 0;
}

static void prevent_burst_bios(struct ioband_group *gp,
			       struct bio *bio, int sync)
{
	struct ioband_device *dp = gp->c_banddev;

	if (current->flags & PF_KTHREAD || is_urgent_bio(bio)) {
		/*
		 * Kernel threads shouldn't be blocked easily since each of
		 * them may handle BIOs for several groups on several
		 * partitions.
		 */
		wait_event_lock_irq(dp->g_waitq[sync],
				    !device_should_block(gp, sync),
				    dp->g_lock, do_nothing());
	} else {
		wait_event_lock_irq(gp->c_waitq[sync],
				    !group_should_block(gp, sync),
				    dp->g_lock, do_nothing());
	}
}

static inline int should_pushback_bio(struct ioband_group *gp)
{
	return is_group_suspended(gp) && dm_noflush_suspending(gp->c_target);
}

static inline bool bio_is_sync(struct bio *bio)
{
	/* Must be the same condition as rw_is_sync() in blkdev.h */
	return !bio_data_dir(bio) || bio_rw_flagged(bio, BIO_RW_SYNCIO);
}

static inline int prepare_to_issue(struct ioband_group *gp, struct bio *bio)
{
	struct ioband_device *dp = gp->c_banddev;

	dp->g_issued[bio_is_sync(bio)]++;
	return dp->g_prepare_bio(gp, bio, 0);
}

static inline int room_for_bio(struct ioband_device *dp)
{
	return dp->g_issued[BLK_RW_SYNC] < dp->g_io_limit
		|| dp->g_issued[BLK_RW_ASYNC] < dp->g_io_limit;
}

static void hold_bio(struct ioband_group *gp, struct bio *bio, int sync)
{
	struct ioband_device *dp = gp->c_banddev;

	dp->g_blocked[sync]++;
	if (is_urgent_bio(bio)) {
		dp->g_prepare_bio(gp, bio, IOBAND_URGENT);
		bio_list_add(&dp->g_urgent_bios, bio);
		trace_ioband_hold_urgent_bio(gp, bio);
	} else {
		gp->c_blocked[sync]++;
		dp->g_hold_bio(gp, bio);
		trace_ioband_hold_bio(gp, bio);
	}
}

static inline int room_for_bio_sync(struct ioband_device *dp, int sync)
{
	return dp->g_issued[sync] < dp->g_io_limit;
}

static void push_prio_bio(struct ioband_group *gp, struct bio *bio, int sync)
{
	if (bio_list_empty(&gp->c_prio_bios))
		set_prio_queue(gp, sync);
	bio_list_add(&gp->c_prio_bios, bio);
	gp->c_prio_blocked++;
}

static struct bio *pop_prio_bio(struct ioband_group *gp)
{
	struct bio *bio = bio_list_pop(&gp->c_prio_bios);

	if (bio_list_empty(&gp->c_prio_bios))
		clear_prio_queue(gp);

	if (bio)
		gp->c_prio_blocked--;
	return bio;
}

static int make_issue_list(struct ioband_group *gp, struct bio *bio, int sync,
			   struct bio_list *issue_list,
			   struct bio_list *pushback_list)
{
	struct ioband_device *dp = gp->c_banddev;

	dp->g_blocked[sync]--;
	gp->c_blocked[sync]--;
	if (!gp->c_blocked[sync] && is_group_blocked(gp, sync)) {
		clear_group_blocked(gp, sync);
		wake_up_all(&gp->c_waitq[sync]);
	}
	if (should_pushback_bio(gp)) {
		bio_list_add(pushback_list, bio);
		trace_ioband_make_pback_list(gp, bio);
	} else {
		int rw = bio_data_dir(bio);

		gp->c_stats.sectors[rw] += bio_sectors(bio);
		gp->c_stats.ios[rw]++;
		bio_list_add(issue_list, bio);
		trace_ioband_make_issue_list(gp, bio);
	}
	return prepare_to_issue(gp, bio);
}

static void release_urgent_bios(struct ioband_device *dp,
				struct bio_list *issue_list,
				struct bio_list *pushback_list)
{
	struct bio *bio;
	int sync;

	if (bio_list_empty(&dp->g_urgent_bios))
		return;
	while (room_for_bio_sync(dp, BLK_RW_ASYNC)) {
		bio = bio_list_pop(&dp->g_urgent_bios);
		if (!bio)
			return;
		sync = bio_is_sync(bio);
		dp->g_blocked[sync]--;
		dp->g_issued[sync]++;
		bio_list_add(issue_list, bio);
		trace_ioband_release_urgent_bios(dp, bio);
	}
}

static int release_prio_bios(struct ioband_group *gp,
			     struct bio_list *issue_list,
			     struct bio_list *pback_list)
{
	struct ioband_device *dp = gp->c_banddev;
	struct bio *bio;
	int sync, ret;

	if (bio_list_empty(&gp->c_prio_bios))
		return R_OK;
	sync = prio_queue_sync(gp);
	while (gp->c_prio_blocked) {
		if (!dp->g_can_submit(gp))
			return R_BLOCK;
		if (!room_for_bio_sync(dp, sync))
			return R_OK;
		bio = pop_prio_bio(gp);
		if (!bio)
			return R_OK;
		ret = make_issue_list(gp, bio, sync, issue_list, pback_list);
		if (ret)
			return ret;
	}
	return R_OK;
}

static int release_norm_bios(struct ioband_group *gp,
			     struct bio_list *issue_list,
			     struct bio_list *pback_list)
{
	struct ioband_device *dp = gp->c_banddev;
	struct bio *bio;
	int sync, ret;

	while (nr_blocked_group(gp) - gp->c_prio_blocked) {
		if (!dp->g_can_submit(gp))
			return R_BLOCK;
		if (!room_for_bio(dp))
			return R_OK;
		bio = dp->g_pop_bio(gp);
		if (!bio)
			return R_OK;

		sync = bio_is_sync(bio);
		if (!room_for_bio_sync(dp, sync)) {
			push_prio_bio(gp, bio, sync);
			continue;
		}
		ret = make_issue_list(gp, bio, sync, issue_list, pback_list);
		if (ret)
			return ret;
	}
	return R_OK;
}

static inline int release_bios(struct ioband_group *gp,
			       struct bio_list *issue_list,
			       struct bio_list *pushback_list)
{
	int ret = release_prio_bios(gp, issue_list, pushback_list);
	if (ret)
		return ret;
	return release_norm_bios(gp, issue_list, pushback_list);
}

static struct ioband_group *ioband_group_get(struct ioband_group *head,
					     struct bio *bio)
{
	struct ioband_group *gp;

	if (!head->c_type->t_getid)
		return head;

	gp = ioband_group_find(head, head->c_type->t_getid(bio));

	if (!gp)
		gp = head;
	return gp;
}

/*
 * Start to control the bandwidth once the number of uncompleted BIOs
 * exceeds the value of "io_throttle".
 */
static int ioband_map(struct dm_target *ti, struct bio *bio,
		      union map_info *map_context)
{
	struct ioband_group *gp = ti->private;
	struct ioband_device *dp = gp->c_banddev;
	unsigned long flags;
	int sync, rw;

	spin_lock_irqsave(&dp->g_lock, flags);

	/*
	 * The device is suspended while some of the ioband device
	 * configurations are being changed.
	 */
	if (is_device_suspended(dp))
		wait_event_lock_irq(dp->g_waitq_suspend,
				    !is_device_suspended(dp), dp->g_lock,
				    do_nothing());

	gp = ioband_group_get(gp, bio);
	sync = bio_is_sync(bio);
	prevent_burst_bios(gp, bio, sync);
	if (should_pushback_bio(gp)) {
		spin_unlock_irqrestore(&dp->g_lock, flags);
		return DM_MAPIO_REQUEUE;
	}

	bio->bi_bdev = gp->c_dev->bdev;
	if (bio_sectors(bio))
		bio->bi_sector -= ti->begin;

	if (!gp->c_blocked[sync] && room_for_bio_sync(dp, sync)) {
		if (dp->g_can_submit(gp)) {
			prepare_to_issue(gp, bio);
			rw = bio_data_dir(bio);
			gp->c_stats.sectors[rw] += bio_sectors(bio);
			gp->c_stats.ios[rw]++;
			spin_unlock_irqrestore(&dp->g_lock, flags);
			return DM_MAPIO_REMAPPED;
		} else if (nr_blocked(dp) == 0 && nr_issued(dp) == 0) {
			DMDEBUG("%s: token expired gp:%p", __func__, gp);
			queue_delayed_work(dp->g_ioband_wq,
					   &dp->g_conductor, 1);
		}
	}
	hold_bio(gp, bio, sync);
	spin_unlock_irqrestore(&dp->g_lock, flags);

	return DM_MAPIO_SUBMITTED;
}

/*
 * Select the best group to resubmit its BIOs.
 */
static struct ioband_group *choose_best_group(struct ioband_device *dp)
{
	struct ioband_group *gp;
	struct ioband_group *best = NULL;
	int highest = 0;
	int pri;

	/* Todo: The algorithm should be optimized.
	 *       It would be better to use rbtree.
	 */
	list_for_each_entry(gp, &dp->g_groups, c_list) {
		if (!nr_blocked_group(gp) || !room_for_bio(dp))
			continue;
		if (nr_blocked_group(gp) == gp->c_prio_blocked &&
		    !room_for_bio_sync(dp, prio_queue_sync(gp)))
			continue;
		pri = dp->g_can_submit(gp);
		if (pri > highest) {
			highest = pri;
			best = gp;
		}
	}

	return best;
}

/*
 * This function is called right after it becomes able to resubmit BIOs.
 * It selects the best BIOs and passes them to the underlying layer.
 */
static void ioband_conduct(struct work_struct *work)
{
	struct ioband_device *dp =
		container_of(work, struct ioband_device, g_conductor.work);
	struct ioband_group *gp = NULL;
	struct bio *bio;
	unsigned long flags;
	struct bio_list issue_list, pushback_list;
	int sync;

	bio_list_init(&issue_list);
	bio_list_init(&pushback_list);

	spin_lock_irqsave(&dp->g_lock, flags);
	release_urgent_bios(dp, &issue_list, &pushback_list);
	if (nr_blocked(dp)) {
		gp = choose_best_group(dp);
		if (gp &&
		    release_bios(gp, &issue_list, &pushback_list) == R_YIELD)
			queue_delayed_work(dp->g_ioband_wq,
					   &dp->g_conductor, 0);
	}

	for (sync = 0; sync < 2; sync++) {
		if (is_device_blocked(dp, sync) &&
		    dp->g_blocked[sync] < dp->g_io_limit) {
			clear_device_blocked(dp, sync);
			wake_up_all(&dp->g_waitq[sync]);
		}
	}

	if (nr_blocked(dp) &&
	    room_for_bio_sync(dp, BLK_RW_SYNC) &&
	    room_for_bio_sync(dp, BLK_RW_ASYNC) &&
	    bio_list_empty(&issue_list) && bio_list_empty(&pushback_list) &&
	    dp->g_restart_bios(dp)) {
		DMDEBUG("%s: token expired dp:%p issued(%d,%d) g_blocked(%d)",
			__func__, dp,
			dp->g_issued[BLK_RW_SYNC], dp->g_issued[BLK_RW_ASYNC],
			nr_blocked(dp));
		queue_delayed_work(dp->g_ioband_wq, &dp->g_conductor, 0);
	}

	spin_unlock_irqrestore(&dp->g_lock, flags);

	while ((bio = bio_list_pop(&issue_list))) {
		trace_ioband_make_request(dp, bio);
		generic_make_request(bio);
	}

	while ((bio = bio_list_pop(&pushback_list))) {
		trace_ioband_pushback_bio(dp, bio);
		bio_endio(bio, -EIO);
	}
}

static int ioband_end_io(struct dm_target *ti, struct bio *bio,
			 int error, union map_info *map_context)
{
	struct ioband_group *gp = ti->private;
	struct ioband_device *dp = gp->c_banddev;
	unsigned long flags;
	int r = error;

	/*
	 *  XXX: A new error code for device mapper devices should be used
	 *       rather than EIO.
	 */
	if (error == -EIO && should_pushback_bio(gp)) {
		/* This ioband device is suspending */
		r = DM_ENDIO_REQUEUE;
	}
	/*
	 * Todo: The algorithm should be optimized to eliminate the spinlock.
	 */
	spin_lock_irqsave(&dp->g_lock, flags);
	dp->g_issued[bio_is_sync(bio)]--;

	/*
	 * Todo: It would be better to introduce high/low water marks here
	 *       not to kick the workqueues so often.
	 */
	if (nr_blocked(dp))
		queue_delayed_work(dp->g_ioband_wq, &dp->g_conductor, 0);
	else if (is_device_suspended(dp) && nr_issued(dp) == 0)
		wake_up_all(&dp->g_waitq_flush);
	spin_unlock_irqrestore(&dp->g_lock, flags);
	return r;
}

static void ioband_presuspend(struct dm_target *ti)
{
	struct ioband_group *gp = ti->private;

	ioband_group_stop_all(gp, 1);
}

static void ioband_resume(struct dm_target *ti)
{
	struct ioband_group *gp = ti->private;

	ioband_group_resume_all(gp);
}

static void ioband_group_status(struct ioband_group *gp, int *szp,
				char *result, unsigned maxlen)
{
	int sz = *szp; /* used in DMEMIT() */
	struct disk_stats *st = &gp->c_stats;

	DMEMIT(" %d %lu %lu %lu %lu %lu %lu %lu %lu %d %lu %lu",
	       gp->c_id,
	       st->ios[0], st->merges[0], st->sectors[0], st->ticks[0],
	       st->ios[1], st->merges[1], st->sectors[1], st->ticks[1],
	       nr_blocked_group(gp), st->io_ticks, st->time_in_queue);
	*szp = sz;
}

static int ioband_status(struct dm_target *ti, status_type_t type,
			 char *result, unsigned maxlen)
{
	struct ioband_group *gp = ti->private, *p;
	struct ioband_device *dp = gp->c_banddev;
	struct rb_node *node;
	int sz = 0;	/* used in DMEMIT() */
	unsigned long flags;

	spin_lock_irqsave(&dp->g_lock, flags);

	switch (type) {
	case STATUSTYPE_INFO:
		DMEMIT("%s", dp->g_name);
		ioband_group_status(gp, &sz, result, maxlen);
		for (node = rb_first(&gp->c_group_root); node;
		     node = rb_next(node)) {
			p = rb_entry(node, struct ioband_group, c_group_node);
			ioband_group_status(p, &sz, result, maxlen);
		}
		break;

	case STATUSTYPE_TABLE:
		DMEMIT("%s %s %d %d %s %s",
		       gp->c_dev->name, dp->g_name,
		       dp->g_io_throttle, dp->g_io_limit,
		       gp->c_type->t_name, dp->g_policy->p_name);
		dp->g_show(gp, &sz, result, maxlen);
		break;
	}

	spin_unlock_irqrestore(&dp->g_lock, flags);
	return 0;
}

static int ioband_group_type_select(struct ioband_group *gp, const char *name)
{
	struct ioband_device *dp = gp->c_banddev;
	const struct ioband_group_type *t;
	unsigned long flags;

	for (t = dm_ioband_group_type; (t->t_name); t++) {
		if (!strcmp(name, t->t_name))
			break;
	}
	if (!t->t_name) {
		DMWARN("%s: %s isn't supported.", __func__, name);
		return -EINVAL;
	}
	spin_lock_irqsave(&dp->g_lock, flags);
	if (!RB_EMPTY_ROOT(&gp->c_group_root)) {
		spin_unlock_irqrestore(&dp->g_lock, flags);
		return -EBUSY;
	}
	gp->c_type = t;
	spin_unlock_irqrestore(&dp->g_lock, flags);

	return 0;
}

static int ioband_set_param(struct ioband_group *gp,
				const char *cmd, const char *value)
{
	struct ioband_device *dp = gp->c_banddev;
	const char *val_str;
	long id;
	unsigned long flags;
	int r;

	r = parse_group_param(value, &id, &val_str);
	if (r)
		return r;

	spin_lock_irqsave(&dp->g_lock, flags);
	if (id != IOBAND_ID_ANY) {
		gp = ioband_group_find(gp, id);
		if (!gp) {
			spin_unlock_irqrestore(&dp->g_lock, flags);
			DMWARN("%s: id=%ld not found.", __func__, id);
			return -EINVAL;
		}
	}
	r = dp->g_set_param(gp, cmd, val_str);
	spin_unlock_irqrestore(&dp->g_lock, flags);
	return r;
}

static int ioband_group_attach(struct ioband_group *head, int parent_id,
					int id, const char *param)
{
	struct ioband_device *dp = head->c_banddev;
	struct ioband_group *parent, *gp;
	int r;

	if (id < 0) {
		DMWARN("%s: invalid id:%d", __func__, id);
		return -EINVAL;
	}
	if (!head->c_type->t_getid) {
		DMWARN("%s: no ioband group type is specified", __func__);
		return -EINVAL;
	}

	/* Determines a parent ioband group */
	switch (parent_id) {
	case 0:
		/* Non-hierarchical configuration */
		parent = NULL;
		break;
	case 1:
		/* The root of a tree, the parent is a default ioband group */
		parent = head;
		break;
	default:
		/* The node in a tree. */
		parent = ioband_group_find(head, parent_id);
		if (!parent) {
			DMWARN("%s: parent group is not configured", __func__);
			return -EINVAL;
		}
		break;
	}

	gp = kzalloc(sizeof(struct ioband_group), GFP_KERNEL);
	if (!gp)
		return -ENOMEM;

	r = ioband_group_init(dp, head, parent, gp, id, param);
	if (r < 0) {
		kfree(gp);
		return r;
	}
	return 0;
}

static int ioband_group_detach(struct ioband_group *head, int id)
{
	struct ioband_device *dp = head->c_banddev;
	struct ioband_group *gp;
	unsigned long flags;
	int r = 0;

	if (id < 0) {
		DMWARN("%s: invalid id:%d", __func__, id);
		return -EINVAL;
	}
	spin_lock_irqsave(&dp->g_lock, flags);
	gp = ioband_group_find(head, id);
	if (!gp) {
		DMWARN("%s: invalid id:%d", __func__, id);
		r = -EINVAL;
		goto out;
	}

	if (!list_empty(&gp->c_children)) {
		DMWARN("%s: group has children", __func__);
		r = -EBUSY;
		goto out;
	}

	/*
	 * Todo: Calling suspend_ioband_device() before releasing the
	 *       ioband group has a large overhead. Need improvement.
	 */
	suspend_ioband_device(dp, flags, 0);
	ioband_group_release(head, gp);
	resume_ioband_device(dp);
out:
	spin_unlock_irqrestore(&dp->g_lock, flags);
	return r;
}

/*
 * Message parameters:
 *	"policy"      <name>
 *       ex)
 *		"policy" "weight"
 *	"type"        "none"|"pid"|"pgrp"|"node"|"cpuset"|"cgroup"|"user"|"gid"
 * 	"io_throttle" <value>
 * 	"io_limit"    <value>
 *	"attach"      <group id>
 *	"detach"      <group id>
 *	"any-command" <group id>:<value>
 *       ex)
 *		"weight" 0:<value>
 *		"token"  24:<value>
 */
static int __ioband_message(struct dm_target *ti, unsigned argc, char **argv)
{
	struct ioband_group *gp = ti->private, *p;
	struct ioband_device *dp = gp->c_banddev;
	struct rb_node *node;
	long val;
	int r = 0;
	unsigned long flags;

	if (argc == 1 && !strcmp(argv[0], "reset")) {
		spin_lock_irqsave(&dp->g_lock, flags);
		memset(&gp->c_stats, 0, sizeof(gp->c_stats));
		for (node = rb_first(&gp->c_group_root); node;
		     node = rb_next(node)) {
			p = rb_entry(node, struct ioband_group, c_group_node);
			memset(&p->c_stats, 0, sizeof(p->c_stats));
		}
		spin_unlock_irqrestore(&dp->g_lock, flags);
		return 0;
	}

	if (argc != 2) {
		DMWARN("Unrecognised band message received.");
		return -EINVAL;
	}
	if (!strcmp(argv[0], "io_throttle")) {
		r = strict_strtol(argv[1], 0, &val);
		if (r || val < 0 || val > SHORT_MAX)
			return -EINVAL;
		if (val == 0)
			val = DEFAULT_IO_THROTTLE;
		spin_lock_irqsave(&dp->g_lock, flags);
		if (val > dp->g_io_limit) {
			spin_unlock_irqrestore(&dp->g_lock, flags);
			return -EINVAL;
		}
		dp->g_io_throttle = val;
		spin_unlock_irqrestore(&dp->g_lock, flags);
		ioband_set_param(gp, argv[0], argv[1]);
		return 0;
	} else if (!strcmp(argv[0], "io_limit")) {
		r = strict_strtol(argv[1], 0, &val);
		if (r || val < 0 || val > SHORT_MAX)
			return -EINVAL;
		spin_lock_irqsave(&dp->g_lock, flags);
		if (val == 0) {
			struct request_queue *q;

			q = bdev_get_queue(gp->c_dev->bdev);
			if (!q) {
				spin_unlock_irqrestore(&dp->g_lock, flags);
				return -ENXIO;
			}
			/*
			 * The block layer accepts I/O requests up to
			 * 50% over nr_requests when the requests are
			 * issued from a "batcher" process.
			 */
			val = (3 * q->nr_requests / 2);
		}
		if (val < dp->g_io_throttle) {
			spin_unlock_irqrestore(&dp->g_lock, flags);
			return -EINVAL;
		}
		dp->g_io_limit = val;
		spin_unlock_irqrestore(&dp->g_lock, flags);
		ioband_set_param(gp, argv[0], argv[1]);
		return 0;
	} else if (!strcmp(argv[0], "type")) {
		return ioband_group_type_select(gp, argv[1]);
	} else if (!strcmp(argv[0], "attach")) {
		r = strict_strtol(argv[1], 0, &val);
		if (r)
			return r;
		return ioband_group_attach(gp, 0, val, NULL);
	} else if (!strcmp(argv[0], "detach")) {
		r = strict_strtol(argv[1], 0, &val);
		if (r)
			return r;
		return ioband_group_detach(gp, val);
	} else if (!strcmp(argv[0], "policy")) {
		r = policy_init(dp, argv[1], 0, &argv[2]);
		return r;
	} else {
		/* message anycommand <group-id>:<value> */
		r = ioband_set_param(gp, argv[0], argv[1]);
		if (r < 0)
			DMWARN("Unrecognised band message received.");
		return r;
	}
	return 0;
}

static int ioband_message(struct dm_target *ti, unsigned argc, char **argv)
{
	int r;

	mutex_lock(&ioband_lock);
	r = __ioband_message(ti, argc, argv);
	mutex_unlock(&ioband_lock);
	return r;
}

static int ioband_merge(struct dm_target *ti, struct bvec_merge_data *bvm,
			struct bio_vec *biovec, int max_size)
{
	struct ioband_group *gp = ti->private;
	struct request_queue *q = bdev_get_queue(gp->c_dev->bdev);

	if (!q->merge_bvec_fn)
		return max_size;

	bvm->bi_bdev = gp->c_dev->bdev;
	bvm->bi_sector -= ti->begin;

	return min(max_size, q->merge_bvec_fn(q, bvm, biovec));
}

static int ioband_iterate_devices(struct dm_target *ti,
				  iterate_devices_callout_fn fn, void *data)
{
	struct ioband_group *gp = ti->private;

	return fn(ti, gp->c_dev, 0, ti->len, data);
}

static struct target_type ioband_target = {
	.name	     = "ioband",
	.module      = THIS_MODULE,
	.version     = {1, 14, 0},
	.ctr	     = ioband_ctr,
	.dtr	     = ioband_dtr,
	.map	     = ioband_map,
	.end_io	     = ioband_end_io,
	.presuspend  = ioband_presuspend,
	.resume	     = ioband_resume,
	.status	     = ioband_status,
	.message     = ioband_message,
	.merge       = ioband_merge,
	.iterate_devices = ioband_iterate_devices,
};

#ifdef CONFIG_CGROUP_BLKIO
/* Copy mapped device name into supplied buffers */
static void ioband_copy_name(struct ioband_group *gp, char *name)
{
	struct mapped_device *md;

	md = dm_table_get_md(gp->c_target->table);
	dm_copy_name_and_uuid(md, name, NULL);
	dm_put(md);
}

/* Show all ioband devices and their settings */
static void ioband_cgroup_show_device(struct seq_file *m)
{
	struct ioband_device *dp;
	struct ioband_group *head;
	char name[DM_NAME_LEN];

	mutex_lock(&ioband_lock);
	list_for_each_entry(dp, &ioband_device_list, g_list) {
		seq_printf(m, "%s policy=%s io_throttle=%d io_limit=%d",
			   dp->g_name, dp->g_policy->p_name,
			   dp->g_io_throttle, dp->g_io_limit);
		if (dp->g_show_device)
			dp->g_show_device(m, dp);
		seq_putc(m, '\n');

		list_for_each_entry(head, &dp->g_heads, c_heads) {
			if (strcmp(head->c_type->t_name, "cgroup"))
				continue;
			ioband_copy_name(head, name);
			seq_printf(m, "  %s\n", name);
		}
	}
	mutex_unlock(&ioband_lock);
}

/* Configure the ioband device specified by share name or device name */
static int ioband_cgroup_config_device(int argc, char **argv)
{
	struct ioband_device *dp;
	struct ioband_group *head;
	char name[DM_NAME_LEN];
	int r;

	if (argc < 1)
		return -EINVAL;

	mutex_lock(&ioband_lock);
	list_for_each_entry(dp, &ioband_device_list, g_list) {
		/* lookup by share name */
		if (!strcmp(dp->g_name, argv[0])) {
			head = list_first_entry(&dp->g_heads,
					      struct ioband_group, c_heads);
			goto found;
		}

		/* lookup by device name */
		list_for_each_entry(head, &dp->g_heads, c_heads) {
			ioband_copy_name(head, name);
			if (!strcmp(name, argv[0]))
				goto found;
		}
	}
	mutex_unlock(&ioband_lock);
	return -ENODEV;

found:
	if (!strcmp(head->c_type->t_name, "cgroup"))
		r = __ioband_message(head->c_target, --argc, &argv[1]);
	else
		r = -ENODEV;

	mutex_unlock(&ioband_lock);
	return r;
}

/* Show the settings of the blkio cgroup specified by ID */
static void ioband_cgroup_show_group(struct seq_file *m, int type, int id)
{
	struct ioband_device *dp;
	struct ioband_group *head, *gp;
	struct disk_stats *st;
	char name[DM_NAME_LEN];
	unsigned long flags;

	mutex_lock(&ioband_lock);
	list_for_each_entry(dp, &ioband_device_list, g_list) {
		list_for_each_entry(head, &dp->g_heads, c_heads) {
			if (strcmp(head->c_type->t_name, "cgroup"))
				continue;

			gp = (id == 1) ? head : ioband_group_find(head, id);
			if (!gp)
				continue;

			ioband_copy_name(head, name);
			seq_puts(m, name);

			switch (type) {
			case IOG_INFO_CONFIG:
				if (dp->g_show_group)
					dp->g_show_group(m, gp);
				break;
			case IOG_INFO_STATS:
				st = &gp->c_stats;
				spin_lock_irqsave(&dp->g_lock, flags);
				seq_printf(m, " %lu %lu %lu %lu"
					   " %lu %lu %lu %lu %d %lu %lu",
					   st->ios[0], st->merges[0],
					   st->sectors[0], st->ticks[0],
					   st->ios[1], st->merges[1],
					   st->sectors[1], st->ticks[1],
					   nr_blocked_group(gp),
					   st->io_ticks, st->time_in_queue);
				spin_unlock_irqrestore(&dp->g_lock, flags);
				break;
			}
			seq_putc(m, '\n');
		}
	}
	mutex_unlock(&ioband_lock);
}

/* Configure the blkio cgroup specified by device name and group ID */
static int ioband_cgroup_config_group(int argc, char **argv,
				      int parent, int id)
{
	struct ioband_device *dp;
	struct ioband_group *head, *gp;
	char name[DM_NAME_LEN];
	int r;

	if (argc != 1 && argc != 2)
		return -EINVAL;

	mutex_lock(&ioband_lock);
	list_for_each_entry(dp, &ioband_device_list, g_list) {
		list_for_each_entry(head, &dp->g_heads, c_heads) {
			if (strcmp(head->c_type->t_name, "cgroup"))
				continue;
			ioband_copy_name(head, name);
			if (!strcmp(name, argv[0]))
				goto found;
		}
	}
	mutex_unlock(&ioband_lock);
	return -ENODEV;

found:
	if (argc == 1) {
		/* remove the group unless it is not a root cgroup */
		r = (id == 1) ? -EINVAL : ioband_group_detach(head, id);
	} else {
		/* create a group or modify the group settings */
		gp = (id == 1) ? head : ioband_group_find(head, id);

		if (!gp)
			r = ioband_group_attach(head, parent, id, argv[1]);
		else
			r = gp->c_banddev->g_set_param(gp, NULL, argv[1]);
	}

	mutex_unlock(&ioband_lock);
	return r;
}

/*
 * Reset the statistics counter of the blkio cgroup specified by
 * device name and group ID.
 */
static int ioband_cgroup_reset_group_stats(int argc, char **argv, int id)
{
	struct ioband_device *dp;
	struct ioband_group *head, *gp;
	char name[DM_NAME_LEN];

	if (argc != 1)
		return -EINVAL;

	mutex_lock(&ioband_lock);
	list_for_each_entry(dp, &ioband_device_list, g_list) {
		list_for_each_entry(head, &dp->g_heads, c_heads) {
			if (strcmp(head->c_type->t_name, "cgroup"))
				continue;
			ioband_copy_name(head, name);
			if (strcmp(name, argv[0]))
				continue;

			gp = (id == 1) ? head : ioband_group_find(head, id);
			if (gp)
				memset(&gp->c_stats, 0, sizeof(gp->c_stats));

			mutex_unlock(&ioband_lock);
			return 0;
		}
	}
	mutex_unlock(&ioband_lock);
	return -ENODEV;
}

/* Remove the blkio cgroup specified by ID */
static void ioband_cgroup_remove_group(int id)
{
	struct ioband_device *dp;
	struct ioband_group *head;

	mutex_lock(&ioband_lock);
	list_for_each_entry(dp, &ioband_device_list, g_list) {
		list_for_each_entry(head, &dp->g_heads, c_heads) {
			if (strcmp(head->c_type->t_name, "cgroup"))
				continue;
			if (ioband_group_find(head, id))
				ioband_group_detach(head, id);
		}
	}
	mutex_unlock(&ioband_lock);
}

static const struct ioband_cgroup_ops ioband_ops = {
	.show_device		= ioband_cgroup_show_device,
	.config_device		= ioband_cgroup_config_device,
	.show_group		= ioband_cgroup_show_group,
	.config_group		= ioband_cgroup_config_group,
	.reset_group_stats 	= ioband_cgroup_reset_group_stats,
	.remove_group		= ioband_cgroup_remove_group,
};
#endif

static int __init dm_ioband_init(void)
{
	int r;

	r = dm_register_target(&ioband_target);
	if (r < 0)
		DMERR("register failed %d", r);
#ifdef CONFIG_CGROUP_BLKIO
	else
		r = blkio_cgroup_register_ioband(&ioband_ops);
#endif
	return r;
}

static void __exit dm_ioband_exit(void)
{
#ifdef CONFIG_CGROUP_BLKIO
	blkio_cgroup_unregister_ioband();
#endif
	dm_unregister_target(&ioband_target);
}

module_init(dm_ioband_init);
module_exit(dm_ioband_exit);

MODULE_DESCRIPTION(DM_NAME " I/O bandwidth control");
MODULE_AUTHOR("Hirokazu Takahashi, Ryo Tsuruta, Dong-Jae Kang");
MODULE_LICENSE("GPL");
