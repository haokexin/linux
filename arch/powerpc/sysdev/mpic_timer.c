/*
 * Copyright 2012 Freescale Semiconductor, Inc.
 * Author: Dongsheng Wang <Dongsheng.Wang@freescale.com>
 *	   Li Yang <leoli@freescale.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <asm/io.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <sysdev/fsl_soc.h>
#include <asm/mpic_timer.h>

#define FSL_GLOBAL_TIMER		0x1

#define MPIC_TIMER_TCR_CLKDIV_64	0x00000300
#define MPIC_TIMER_TCR_ROVR_OFFSET	24

#define TIMER_STOP			0x80000000
#define TIMERS_PER_GROUP		4
#define MAX_TICKS			(~0U>>1)
#define MAX_TICKS_CASCADE		(~0U)
#define TIMER_OFFSET(num)		(1 << (TIMERS_PER_GROUP - 1 - num))

/* tv_usec should be less than ONE_SECOND, otherwise use tv_sec */
#define ONE_SECOND			1000000

struct timer_regs {
	u32	gtccr;
	u32	res0[3];
	u32	gtbcr;
	u32	res1[3];
	u32	gtvpr;
	u32	res2[3];
	u32	gtdr;
	u32	res3[3];
};

struct cascade_priv {
	u32 tcr_value;			/* TCR register: CASC & ROVR value */
	unsigned int cascade_map;	/* cascade map */
	unsigned int timer_num;		/* cascade control timer */
};

struct mpic_timer {
	void			*dev;
	struct cascade_priv	*cascade_handle;
	unsigned int		num;
	int			irq;
};

struct timer_group_priv {
	struct timer_regs __iomem	*regs;
	struct mpic_timer		timer[TIMERS_PER_GROUP];
	struct list_head		node;
	u32				ccbfreq;
	unsigned int			timerfreq;
	unsigned int			idle;
	unsigned int			flags;
	spinlock_t			lock;
	void __iomem			*group_tcr;
};

static struct cascade_priv cascade_timer[] = {
	/* cascade timer 0 and 1 */
	{0x1, 0xc, 0x1},
	/* cascade timer 1 and 2 */
	{0x2, 0x6, 0x2},
	/* cascade timer 2 and 3 */
	{0x4, 0x3, 0x3}
};

static LIST_HEAD(timer_group_list);

/* the time set by the user is converted to "ticks" */
static int convert_ticks(struct timer_group_priv *priv,
		const struct timeval *time, u64 *ticks)
{
	u64 max_value;		/* prevent u64 overflow */
	u64 tmp = 0;

	u64 tmp_sec = 0;
	u64 tmp_ms = 0;
	u64 tmp_us = 0;
	u32 div = 0;

	if (priv->flags & FSL_GLOBAL_TIMER)
		max_value = div_u64(ULLONG_MAX, priv->ccbfreq);
	else
		max_value = div_u64(ULLONG_MAX, priv->timerfreq);

	if (time->tv_sec > max_value ||
			(time->tv_sec == max_value && time->tv_usec > 0))
		return -EINVAL;

	if (!(priv->flags & FSL_GLOBAL_TIMER)) {
		tmp = time->tv_sec * priv->timerfreq;
		*ticks = tmp;

		return 0;
	}

	div = (1 << (MPIC_TIMER_TCR_CLKDIV_64 >> 8)) * 8;

	tmp_sec = div_u64((u64)time->tv_sec * (u64)priv->ccbfreq, div);
	tmp += tmp_sec;

	tmp_ms = time->tv_usec / 1000;
	tmp_ms = div_u64((u64)tmp_ms * (u64)priv->ccbfreq, div * 1000);
	tmp += tmp_ms;

	tmp_us = time->tv_usec % 1000;
	tmp_us = div_u64((u64)tmp_us * (u64)priv->ccbfreq, div * 1000000);
	tmp += tmp_us;

	*ticks = tmp;

	return 0;
}

/* detect whether there is a cascade timer available */
static struct mpic_timer *detect_idle_cascade_timer(
					struct timer_group_priv *priv)
{
	struct cascade_priv *casc_priv;
	unsigned int tmp;
	unsigned int array_size = ARRAY_SIZE(cascade_timer);
	unsigned int num;
	unsigned int i;
	unsigned long flags;

	casc_priv = cascade_timer;
	for (i = 0; i < array_size; i++) {
		spin_lock_irqsave(&priv->lock, flags);
		tmp = casc_priv->cascade_map & priv->idle;
		if (tmp == casc_priv->cascade_map) {
			num = casc_priv->timer_num;
			priv->timer[num].cascade_handle = casc_priv;

			/* set timer busy */
			priv->idle &= ~casc_priv->cascade_map;
			spin_unlock_irqrestore(&priv->lock, flags);
			return &priv->timer[num];
		}
		spin_unlock_irqrestore(&priv->lock, flags);
		casc_priv++;
	}

	return NULL;
}

static int set_cascade_timer(struct timer_group_priv *priv, u64 ticks,
		unsigned int num)
{
	struct cascade_priv *casc_priv;
	u32 tmp;
	u32 tmp_ticks;
	u32 rem_ticks;

	/* set group tcr reg for cascade */
	casc_priv = priv->timer[num].cascade_handle;
	if (!casc_priv)
		return -EINVAL;

	tmp = casc_priv->tcr_value |
		(casc_priv->tcr_value << MPIC_TIMER_TCR_ROVR_OFFSET);
	setbits32(priv->group_tcr, tmp);

	tmp_ticks = div_u64_rem(ticks, MAX_TICKS_CASCADE, &rem_ticks);

	out_be32(&priv->regs[num].gtccr, 0);
	out_be32(&priv->regs[num].gtbcr, tmp_ticks | TIMER_STOP);

	out_be32(&priv->regs[num - 1].gtccr, 0);
	out_be32(&priv->regs[num - 1].gtbcr, rem_ticks);

	return 0;
}

static struct mpic_timer *get_cascade_timer(struct timer_group_priv *priv,
					u64 ticks)
{
	struct mpic_timer *allocated_timer = NULL;

	/* Two cascade timers: Support the maximum time */
	const u64 max_ticks = (u64)MAX_TICKS * (u64)MAX_TICKS_CASCADE;
	int ret;

	if (ticks > max_ticks)
		return NULL;

	/* detect idle timer */
	allocated_timer = detect_idle_cascade_timer(priv);
	if (!allocated_timer)
		return NULL;

	/* set ticks to timer */
	ret = set_cascade_timer(priv, ticks, allocated_timer->num);
	if (ret < 0)
		return NULL;

	return allocated_timer;
}

static struct mpic_timer *get_timer(const struct timeval *time)
{
	struct timer_group_priv *priv;
	struct mpic_timer *timer = NULL;

	u64 ticks = 0;
	unsigned int num;
	unsigned int i;
	unsigned long flags;
	int ret = 0;

	list_for_each_entry(priv, &timer_group_list, node) {
		ret = convert_ticks(priv, time, &ticks);
		if (ret < 0)
			return NULL;

		if (ticks > MAX_TICKS) {
			if (!(priv->flags & FSL_GLOBAL_TIMER))
				return NULL;

			timer = get_cascade_timer(priv, ticks);
			if (!timer)
				continue;
			else
				return timer;
		}

		for (i = 0; i < TIMERS_PER_GROUP; i++) {
			/* one timer: Reverse allocation */
			num = TIMERS_PER_GROUP - 1 - i;
			spin_lock_irqsave(&priv->lock, flags);
			if (priv->idle & (1 << i)) {
				/* set timer busy */
				priv->idle &= ~(1 << i);
				/* set ticks & stop timer */
				out_be32(&priv->regs[num].gtbcr,
					ticks | TIMER_STOP);
				out_be32(&priv->regs[num].gtccr, 0);
				priv->timer[num].cascade_handle = NULL;
				spin_unlock_irqrestore(&priv->lock, flags);
				return &priv->timer[num];
			}
			spin_unlock_irqrestore(&priv->lock, flags);
		}
	}

	return NULL;
}

/**
 * mpic_start_timer - start hardware timer
 * @handle: the timer to be started.
 *
 * It will do ->fn(->dev) callback from the hardware interrupt at
 * the ->timeval point in the future.
 */
void mpic_start_timer(struct mpic_timer *handle)
{
	struct timer_group_priv *priv = container_of(handle,
			struct timer_group_priv, timer[handle->num]);

	clrbits32(&priv->regs[handle->num].gtbcr, TIMER_STOP);
}
EXPORT_SYMBOL(mpic_start_timer);

/**
 * mpic_stop_timer - stop hardware timer
 * @handle: the timer to be stoped
 *
 * The timer periodically generates an interrupt. Unless user stops the timer.
 */
void mpic_stop_timer(struct mpic_timer *handle)
{
	struct timer_group_priv *priv = container_of(handle,
			struct timer_group_priv, timer[handle->num]);

	setbits32(&priv->regs[handle->num].gtbcr, TIMER_STOP);
}
EXPORT_SYMBOL(mpic_stop_timer);

/**
 * mpic_free_timer - free hardware timer
 * @handle: the timer to be removed.
 *
 * Free the timer.
 *
 * Note: can not be used in interrupt context.
 */
void mpic_free_timer(struct mpic_timer *handle)
{
	struct timer_group_priv *priv = container_of(handle,
			struct timer_group_priv, timer[handle->num]);

	struct cascade_priv *casc_priv = NULL;
	unsigned long flags;

	mpic_stop_timer(handle);

	casc_priv = priv->timer[handle->num].cascade_handle;

	free_irq(priv->timer[handle->num].irq, priv->timer[handle->num].dev);

	spin_lock_irqsave(&priv->lock, flags);
	if (casc_priv) {
		u32 tmp;
		tmp = casc_priv->tcr_value | (casc_priv->tcr_value <<
					MPIC_TIMER_TCR_ROVR_OFFSET);
		clrbits32(priv->group_tcr, tmp);
		priv->idle |= casc_priv->cascade_map;
		priv->timer[handle->num].cascade_handle = NULL;
	} else {
		priv->idle |= TIMER_OFFSET(handle->num);
	}
	spin_unlock_irqrestore(&priv->lock, flags);
}
EXPORT_SYMBOL(mpic_free_timer);

/**
 * mpic_request_timer - get a hardware timer
 * @fn: interrupt handler function
 * @dev: callback function of the data
 * @time: time for timer
 *
 * This executes the "request_irq", returning NULL
 * else "handle" on success.
 */
struct mpic_timer *mpic_request_timer(irq_handler_t fn, void *dev,
					const struct timeval *time)
{
	struct mpic_timer *allocated_timer = NULL;

	int ret = 0;

	if (list_empty(&timer_group_list))
		return NULL;

	if ((time->tv_sec + time->tv_usec) == 0 ||
			time->tv_sec < 0 || time->tv_usec < 0)
		return NULL;

	if (time->tv_usec > ONE_SECOND)
		return NULL;

	allocated_timer = get_timer(time);
	if (!allocated_timer)
		return NULL;

	ret = request_irq(allocated_timer->irq, fn,
			IRQF_TRIGGER_LOW | IRQF_NO_SUSPEND,
			"global-timer", dev);
	if (ret) {
		mpic_free_timer(allocated_timer);
		return NULL;
	}

	allocated_timer->dev = dev;

	return allocated_timer;
}
EXPORT_SYMBOL(mpic_request_timer);

static int timer_group_get_freq(struct device_node *np,
			struct timer_group_priv *priv)
{
	if (priv->flags & FSL_GLOBAL_TIMER) {
		priv->ccbfreq = fsl_get_sys_freq();
		priv->timerfreq = priv->ccbfreq;
	} else {
		const u32 *timerfreq;
		int size;

		timerfreq = of_get_property(np, "clock-frequency", &size);
		if (!timerfreq || size != sizeof(*timerfreq) ||
				*timerfreq == 0)
			priv->timerfreq = 0;
		else
			priv->timerfreq = *timerfreq;
	}

	if (priv->timerfreq == 0)
		return -EINVAL;

	return 0;
}

static int timer_group_get_irq(struct device_node *np,
		struct timer_group_priv *priv)
{
	const u32 all_timer[] = { 0, TIMERS_PER_GROUP };
	const u32 *p = NULL;
	u32 offset;
	u32 count;

	unsigned int i = 0;
	unsigned int j = 0;
	unsigned int irq_index = 0;
	int irq = 0;
	int len = 0;

	p = of_get_property(np, "available-ranges", &len);
	if (!p)
		p = of_get_property(np, "fsl,available-ranges", &len);

	if (p && len % (2 * sizeof(u32)) != 0) {
		pr_err("%s: malformed available-ranges property.\n",
				np->full_name);
		return -EINVAL;
	}

	if (!p) {
		p = all_timer;
		len = sizeof(all_timer);
	}

	len /= 2 * sizeof(u32);

	for (i = 0; i < len; i++) {
		offset = p[i * 2];
		count = p[i * 2 + 1];
		for (j = 0; j < count; j++) {
			irq = irq_of_parse_and_map(np, irq_index);
			if (!irq)
				break;
			/* Set timer idle */
			priv->idle |= TIMER_OFFSET((offset + j));
			priv->timer[offset + j].irq = irq;
			priv->timer[offset + j].num = offset + j;
			irq_index++;
		}
	}

	return 0;
}

static void timer_group_init(struct device_node *np)
{
	struct timer_group_priv *priv = NULL;
	unsigned int i = 0;
	int ret = 0;

	priv = kzalloc(sizeof(struct timer_group_priv), GFP_KERNEL);
	if (!priv) {
		pr_err("%s: cannot allocate memory for group.\n",
				np->full_name);
		return;
	}

	if (of_device_is_compatible(np, "fsl,mpic-global-timer"))
		priv->flags |= FSL_GLOBAL_TIMER;

	priv->regs = of_iomap(np, i++);
	if (!priv->regs) {
		pr_err("%s: cannot ioremap timer register address.\n",
				np->full_name);
		goto out;
	}

	if (priv->flags & FSL_GLOBAL_TIMER) {
		priv->group_tcr = of_iomap(np, i++);
		if (!priv->group_tcr) {
			pr_err("%s: cannot ioremap tcr address.\n",
					np->full_name);
			goto out;
		}
	}

	ret = timer_group_get_freq(np, priv);
	if (ret < 0)
		goto out;

	ret = timer_group_get_irq(np, priv);
	if (ret < 0)
		goto out;

	spin_lock_init(&priv->lock);

	/* Init FSL timer hardware */
	if (priv->flags & FSL_GLOBAL_TIMER)
		setbits32(priv->group_tcr, MPIC_TIMER_TCR_CLKDIV_64);

	list_add_tail(&priv->node, &timer_group_list);

	return;

out:
	if (priv->regs)
		iounmap(priv->regs);

	if (priv->group_tcr)
		iounmap(priv->group_tcr);

	kfree(priv);
}

static const struct of_device_id mpic_timer_ids[] = {
	{ .compatible = "fsl,mpic-global-timer", },
	{},
};

static int __init mpic_timer_init(void)
{
	struct device_node *np = NULL;

	for_each_matching_node(np, mpic_timer_ids)
		timer_group_init(np);

	if (list_empty(&timer_group_list))
		return -ENODEV;

	return 0;
}
subsys_initcall(mpic_timer_init);
