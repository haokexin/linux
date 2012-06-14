/*
 * Copyright (C) 2008-2009 VA Linux Systems Japan K.K.
 *
 *  I/O bandwidth control
 *
 * This file is released under the GPL.
 */
#include <linux/bio.h>
#include <linux/workqueue.h>
#include <linux/rbtree.h>
#include <linux/seq_file.h>
#include "dm.h"
#include "dm-ioband.h"

/*
 * The following functions determine when and which BIOs should
 * be submitted to control the I/O flow.
 * It is possible to add a new BIO scheduling policy with it.
 */

/*
 * Functions for weight balancing policy based on the number of I/Os.
 */
#define DEFAULT_WEIGHT		100
#define DEFAULT_TOKENPOOL	2048
#define DEFAULT_BUCKET		2
#define IOBAND_IOPRIO_BASE	100
#define TOKEN_BATCH_UNIT	20
#define PROCEED_THRESHOLD	8
#define LOCAL_ACTIVE_RATIO	8
#define GLOBAL_ACTIVE_RATIO	16
#define OVERCOMMIT_RATE		4
#define WEIGHT_MAX		100

/*
 * Calculate the effective number of tokens this group has.
 */
static int get_token(struct ioband_group *gp)
{
	struct ioband_device *dp = gp->c_banddev;
	int token = gp->c_token;
	int allowance = dp->g_epoch - gp->c_my_epoch;

	if (allowance) {
		if (allowance > dp->g_carryover)
			allowance = dp->g_carryover;
		token += gp->c_token_initial * allowance;
	}
	if (is_group_down(gp))
		token += gp->c_token_initial * dp->g_carryover * 2;

	return token;
}

/*
 * Calculate the priority of a given group.
 */
static int iopriority(struct ioband_group *gp)
{
	return get_token(gp) * IOBAND_IOPRIO_BASE / gp->c_token_initial + 1;
}

/*
 * This function is called when all the active group on the same ioband
 * device has used up their tokens. It makes a new global epoch so that
 * all groups on this device will get freshly assigned tokens.
 */
static int make_global_epoch(struct ioband_device *dp)
{
	struct ioband_group *gp = dp->g_dominant;

	/*
	 * Don't make a new epoch if the dominant group still has a lot of
	 * tokens, except when the I/O load is low.
	 */
	if (gp) {
		int iopri = iopriority(gp);
		if (iopri * PROCEED_THRESHOLD > IOBAND_IOPRIO_BASE &&
		    nr_issued(dp) >= dp->g_io_throttle)
			return 0;
	}

	dp->g_epoch++;
	DMDEBUG("make_epoch %d", dp->g_epoch);

	/* The leftover tokens will be used in the next epoch. */
	dp->g_token_extra = dp->g_token_left;
	if (dp->g_token_extra < 0)
		dp->g_token_extra = 0;
	dp->g_token_left = dp->g_token_bucket;

	dp->g_expired = NULL;
	dp->g_dominant = NULL;

	return 1;
}

/*
 * This function is called when this group has used up its own tokens.
 * It will check whether it's possible to make a new epoch of this group.
 */
static inline int make_epoch(struct ioband_group *gp)
{
	struct ioband_device *dp = gp->c_banddev;
	int allowance = dp->g_epoch - gp->c_my_epoch;

	if (!allowance)
		return 0;
	if (allowance > dp->g_carryover)
		allowance = dp->g_carryover;
	gp->c_my_epoch = dp->g_epoch;
	return allowance;
}

/*
 * Check whether this group has tokens to issue an I/O. Return 0 if it
 * doesn't have any, otherwise return the priority of this group.
 */
static int is_token_left(struct ioband_group *gp)
{
	struct ioband_device *dp = gp->c_banddev;
	int allowance;
	int delta;
	int extra;

	if (gp->c_token > 0)
		return iopriority(gp);

	if (is_group_down(gp)) {
		gp->c_token = gp->c_token_initial;
		return iopriority(gp);
	}
	allowance = make_epoch(gp);
	if (!allowance)
		return 0;
	/*
	 * If this group has the right to get tokens for several epochs,
	 * give all of them to the group here.
	 */
	delta = gp->c_token_initial * allowance;
	dp->g_token_left -= delta;
	/*
	 * Give some extra tokens to this group when there have left unused
	 * tokens on this ioband device from the previous epoch.
	 */
	extra = dp->g_token_extra * gp->c_token_initial /
	    (dp->g_token_bucket - dp->g_token_extra / 2);
	delta += extra;
	gp->c_token += delta;
	gp->c_consumed = 0;

	if (gp == dp->g_current)
		dp->g_yield_mark += delta;
	DMDEBUG("refill token: gp:%p token:%d->%d extra(%d) allowance(%d)",
		gp, gp->c_token - delta, gp->c_token, extra, allowance);
	if (gp->c_token > 0)
		return iopriority(gp);
	DMDEBUG("refill token: yet empty gp:%p token:%d", gp, gp->c_token);
	return 0;
}

/*
 * Use tokens to issue an I/O. After the operation, the number of tokens left
 * on this group may become negative value, which will be treated as debt.
 */
static int consume_token(struct ioband_group *gp, int count, int flag)
{
	struct ioband_device *dp = gp->c_banddev;

	if (gp->c_consumed * LOCAL_ACTIVE_RATIO < gp->c_token_initial &&
	    gp->c_consumed * GLOBAL_ACTIVE_RATIO < dp->g_token_bucket) {
		; /* Do nothing unless this group is really active. */
	} else if (!dp->g_dominant ||
		   get_token(gp) > get_token(dp->g_dominant)) {
		/*
		 * Regard this group as the dominant group on this
		 * ioband device when it has larger number of tokens
		 * than those of the previous one.
		 */
		dp->g_dominant = gp;
	}
	if (dp->g_epoch == gp->c_my_epoch &&
	    gp->c_token > 0 && gp->c_token - count <= 0) {
		/* Remember the last group which used up its own tokens. */
		dp->g_expired = gp;
		if (dp->g_dominant == gp)
			dp->g_dominant = NULL;
	}

	if (gp != dp->g_current) {
		/* This group is the current already. */
		dp->g_current = gp;
		dp->g_yield_mark =
		    gp->c_token - (TOKEN_BATCH_UNIT << dp->g_token_unit);
	}
	gp->c_token -= count;
	gp->c_consumed += count;
	if (gp->c_token <= dp->g_yield_mark && !(flag & IOBAND_URGENT)) {
		/*
		 * Return-value 1 means that this policy requests dm-ioband
		 * to give a chance to another group to be selected since
		 * this group has already issued enough amount of I/Os.
		 */
		dp->g_current = NULL;
		return R_YIELD;
	}
	/*
	 * Return-value 0 means that this policy allows dm-ioband to select
	 * this group to issue I/Os without a break.
	 */
	return R_OK;
}

/*
 * Consume one token on each I/O.
 */
static int prepare_token(struct ioband_group *gp, struct bio *bio, int flag)
{
	return consume_token(gp, 1, flag);
}

/*
 * Check if this group is able to receive a new bio.
 */
static int is_queue_full(struct ioband_group *gp, int sync)
{
	return gp->c_blocked[sync] >= gp->c_limit;
}

static void __set_weight(struct ioband_group *gp, int weight_total,
				int token_bucket, int limit_bucket)
{
	int token, limit;

	if (weight_total > 0) {
		token = token_bucket * gp->c_weight / weight_total;
		if (token < 1)
			token = 1;
		limit = limit_bucket * gp->c_weight / weight_total;
		if (limit < 1)
			limit = 1;

		/*
		 * In the hierarchical configuration,
		 * child's tokens are distributed from the parent.
		 */
		if (gp->c_parent) {
			gp->c_parent->c_token_initial -= token;
			if (gp->c_parent->c_token_initial < 1)
				gp->c_parent->c_token_initial = 1;

			gp->c_parent->c_limit -= limit / OVERCOMMIT_RATE;
			if (gp->c_parent->c_limit < 1)
				gp->c_parent->c_limit = 1;
		}
	} else
		token = limit = 1;

	gp->c_token = gp->c_token_initial = gp->c_token_bucket = token;
	gp->c_limit_bucket = limit;
	gp->c_limit = limit / OVERCOMMIT_RATE;
	if (gp->c_limit < 1)
		gp->c_limit = 1;
}

static int set_weight(struct ioband_group *group, int new)
{
	struct ioband_device *dp = group->c_banddev;
	struct ioband_group *parent = group->c_parent, *gp;
	struct list_head *siblings;
	int weight_total = 0, token_bucket, limit;

	group->c_weight = new;

	if (!parent) {
		siblings = &dp->g_root_groups;
		token_bucket = dp->g_token_bucket;
		limit = dp->g_io_limit;
	} else {
		siblings = &parent->c_children;
		token_bucket = parent->c_token_bucket;
		limit = parent->c_limit_bucket;
	}

	list_for_each_entry(gp, siblings, c_sibling)
		weight_total += gp->c_weight;

	if (parent) {
		/*
		 * In the hierarchical configuration, each child's
		 * weight is evaluated as a percentage of its parent's
		 * bandwidth.
		 */
		if (weight_total > WEIGHT_MAX)
			return -EINVAL;
		weight_total = WEIGHT_MAX;
	}

	list_for_each_entry(parent, siblings, c_sibling) {
		struct ioband_group *this_parent = parent;
		struct list_head *next;

		__set_weight(parent, weight_total, token_bucket, limit);

	repeat:
		next = this_parent->c_children.next;
	resume:
		while (next != &this_parent->c_children) {
			/* Descend the hierarchy */
			struct list_head *tmp = next;

			gp = list_entry(tmp, struct ioband_group, c_sibling);
			next = tmp->next;

			__set_weight(gp, WEIGHT_MAX,
				     this_parent->c_token_bucket,
				     this_parent->c_limit_bucket);

			if (!list_empty(&gp->c_children)) {
				this_parent = gp;
				goto repeat;
			}
		}

		if (this_parent != parent) {
			/* Ascend and resume the search */
			next = this_parent->c_sibling.next;
			this_parent = this_parent->c_parent;
			goto resume;
		}
	}

	return 0;
}

static void init_token_bucket(struct ioband_device *dp,
			      int token_bucket, int carryover)
{
	if (!token_bucket)
		dp->g_token_bucket = (dp->g_io_limit * 2 * DEFAULT_BUCKET) <<
							dp->g_token_unit;
	else
		dp->g_token_bucket = token_bucket;
	if (!carryover)
		dp->g_carryover = (DEFAULT_TOKENPOOL << dp->g_token_unit) /
							dp->g_token_bucket;
	else
		dp->g_carryover = carryover;
	if (dp->g_carryover < 1)
		dp->g_carryover = 1;
	dp->g_token_left = 0;
}

static int policy_weight_param(struct ioband_group *gp,
				const char *cmd, const char *value)
{
	struct ioband_device *dp = gp->c_banddev;
	long val = 0;
	int r = 0, err = 0;

	if (value)
		err = strict_strtol(value, 0, &val);

	if (!cmd || !strcmp(cmd, "weight")) {
		if (!value)
			r = set_weight(gp, DEFAULT_WEIGHT);
		else if (!err && 0 < val && val <= SHORT_MAX)
			r = set_weight(gp, val);
		else
			r = -EINVAL;
	} else if (!strcmp(cmd, "token")) {
		if (!err && 0 <= val && val <= INT_MAX) {
			init_token_bucket(dp, val, 0);
			set_weight(gp, gp->c_weight);
			dp->g_token_extra = 0;
		} else
			r = -EINVAL;
	} else if (!strcmp(cmd, "carryover")) {
		if (!err && 0 <= val && val <= INT_MAX) {
			init_token_bucket(dp, dp->g_token_bucket, val);
			set_weight(gp, gp->c_weight);
			dp->g_token_extra = 0;
		} else
			r = -EINVAL;
	} else if (!strcmp(cmd, "io_limit")) {
		init_token_bucket(dp, 0, 0);
		set_weight(gp, gp->c_weight);
	} else {
		r = -EINVAL;
	}
	return r;
}

static int policy_weight_ctr(struct ioband_group *gp, const char *arg)
{
	struct ioband_device *dp = gp->c_banddev;

	gp->c_my_epoch = dp->g_epoch;
	gp->c_weight = 0;
	gp->c_consumed = 0;
	return policy_weight_param(gp, "weight", arg);
}

static void policy_weight_dtr(struct ioband_group *gp)
{
	struct ioband_device *dp = gp->c_banddev;
	set_weight(gp, 0);
	dp->g_dominant = NULL;
	dp->g_expired = NULL;
}

static void policy_weight_show(struct ioband_group *gp, int *szp,
			       char *result, unsigned maxlen)
{
	struct ioband_group *p;
	struct ioband_device *dp = gp->c_banddev;
	struct rb_node *node;
	int sz = *szp;	/* used in DMEMIT() */

	DMEMIT(" %d :%d", dp->g_token_bucket, gp->c_weight);

	for (node = rb_first(&gp->c_group_root); node; node = rb_next(node)) {
		p = rb_entry(node, struct ioband_group, c_group_node);
		DMEMIT(" %d:%d", p->c_id, p->c_weight);
	}
	*szp = sz;
}

static void policy_weight_show_device(struct seq_file *m,
				      struct ioband_device *dp)
{
	seq_printf(m, " token=%d carryover=%d",
				dp->g_token_bucket, dp->g_carryover);
}

static void policy_weight_show_group(struct seq_file *m,
				     struct ioband_group *gp)
{
	seq_printf(m, " weight=%d%%", gp->c_weight);
}

/*
 *  <Method>      <description>
 * g_can_submit   : To determine whether a given group has the right to
 *                  submit BIOs. The larger the return value the higher the
 *                  priority to submit. Zero means it has no right.
 * g_prepare_bio  : Called right before submitting each BIO.
 * g_restart_bios : Called if this ioband device has some BIOs blocked but none
 *                  of them can be submitted now. This method has to
 *                  reinitialize the data to restart to submit BIOs and return
 *                  0 or 1.
 *                  The return value 0 means that it has become able to submit
 *                  them now so that this ioband device will continue its work.
 *                  The return value 1 means that it is still unable to submit
 *                  them so that this device will stop its work. And this
 *                  policy module has to reactivate the device when it gets
 *                  to be able to submit BIOs.
 * g_hold_bio     : To hold a given BIO until it is submitted.
 *                  The default function is used when this method is undefined.
 * g_pop_bio      : To select and get the best BIO to submit.
 * g_group_ctr    : To initalize the policy own members of struct ioband_group.
 * g_group_dtr    : Called when struct ioband_group is removed.
 * g_set_param    : To update the policy own date.
 *                  The parameters can be passed through "dmsetup message"
 *                  command.
 * g_should_block : Called every time this ioband device receive a BIO.
 *                  Return 1 if a given group can't receive any more BIOs,
 *                  otherwise return 0.
 * g_show         : Show the configuration.
 * g_show_device  : Show the configuration of the specified ioband device.
 * g_show_group   : Show the configuration of the spacified ioband group.
 */
static int policy_weight_init(struct ioband_device *dp, int argc, char **argv)
{
	long val;
	int r = 0;

	if (argc < 1)
		val = 0;
	else {
		r = strict_strtol(argv[0], 0, &val);
		if (r || val < 0 || val > INT_MAX)
			return -EINVAL;
	}

	dp->g_can_submit = is_token_left;
	dp->g_prepare_bio = prepare_token;
	dp->g_restart_bios = make_global_epoch;
	dp->g_group_ctr = policy_weight_ctr;
	dp->g_group_dtr = policy_weight_dtr;
	dp->g_set_param = policy_weight_param;
	dp->g_should_block = is_queue_full;
	dp->g_show = policy_weight_show;
	dp->g_show_device = policy_weight_show_device;
	dp->g_show_group = policy_weight_show_group;

	dp->g_epoch = 0;
	dp->g_weight_total = 0;
	dp->g_current = NULL;
	dp->g_dominant = NULL;
	dp->g_expired = NULL;
	dp->g_token_extra = 0;
	dp->g_token_unit = 0;
	init_token_bucket(dp, val, 0);
	dp->g_token_left = dp->g_token_bucket;

	return 0;
}

/* weight balancing policy based on the number of I/Os. --- End --- */

/*
 * Functions for weight balancing policy based on I/O size.
 * It just borrows a lot of functions from the regular weight balancing policy.
 */
static int iosize_prepare_token(struct ioband_group *gp,
					struct bio *bio, int flag)
{
	/* Consume tokens depending on the size of a given bio. */
	return consume_token(gp, bio_sectors(bio), flag);
}

static int policy_weight_iosize_init(struct ioband_device *dp,
						int argc, char **argv)
{
	long val;
	int r = 0;

	if (argc < 1)
		val = 0;
	else {
		r = strict_strtol(argv[0], 0, &val);
		if (r || val < 0 || val > INT_MAX)
			return -EINVAL;
	}

	r = policy_weight_init(dp, argc, argv);
	if (r < 0)
		return r;

	dp->g_prepare_bio = iosize_prepare_token;
	dp->g_token_unit = PAGE_SHIFT - 9;
	init_token_bucket(dp, val, 0);
	dp->g_token_left = dp->g_token_bucket;
	return 0;
}

/* weight balancing policy based on I/O size. --- End --- */

static int policy_default_init(struct ioband_device *dp, int argc, char **argv)
{
	return policy_weight_init(dp, argc, argv);
}

const struct ioband_policy_type dm_ioband_policy_type[] = {
	{ "default",		policy_default_init		},
	{ "weight",		policy_weight_init		},
	{ "weight-iosize",	policy_weight_iosize_init	},
	{ "range-bw",		policy_range_bw_init		},
	{ NULL,			policy_default_init		}
};
