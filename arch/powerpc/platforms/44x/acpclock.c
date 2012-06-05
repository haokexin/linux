/*
 * Copyright (C) 2010-2012 Wind River Systems
 *
 * Some drivers like pl022 need clk interface. But now these drivers
 * can work without it. So define a dummy clk driver for compile issue
 * and future extend.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/export.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/kref.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <asm/clk_interface.h>

struct clk {
	struct list_head node;
	struct device *dev;
	struct kref kref;
};

static LIST_HEAD(clocks);
static DEFINE_MUTEX(clocks_mutex);

struct clk *acp_clk_get(struct device *dev, const char *id)
{
	struct clk *clk;

	if (!dev)
		return NULL;

	mutex_lock(&clocks_mutex);
	list_for_each_entry(clk, &clocks, node) {
		if (clk->dev == dev) {
			kref_get(&clk->kref);
			mutex_unlock(&clocks_mutex);
			return clk;
		}
	}

	clk = kzalloc(sizeof(*clk), GFP_KERNEL);
	if (!clk) {
		dev_err(dev, "Create clock error\n");
		return ERR_PTR(-ENOMEM);
	}

	INIT_LIST_HEAD(&clk->node);
	kref_init(&clk->kref);
	clk->dev = dev;
	list_add(&clk->node, &clocks);
	mutex_unlock(&clocks_mutex);

	return clk;
}

static void acp_clk_release(struct kref *kref)
{
	struct clk *clk = container_of(kref, struct clk, kref);

	mutex_lock(&clocks_mutex);
	list_del(&clk->node);
	mutex_lock(&clocks_mutex);

	kfree(clk);
}

void acp_clk_put(struct clk *clk)
{
	kref_put(&clk->kref, acp_clk_release);
	return;
}

int acp_clk_enable(struct clk *clk)
{
	return 0;
}

void acp_clk_disable(struct clk *clk)
{
	return;
}

unsigned long acp_clk_get_rate(struct clk *clk)
{
	struct device_node *np = clk->dev->of_node;
	const int *prop;
	int len;

	if (!np)
		return 0;

	prop = of_get_property(np, "clock-frequency", &len);
	if (!prop || len != sizeof(*prop)) {
		dev_err(clk->dev, "Get the clock frequency error\n");
		return -1;
	}

	return *prop;
}

long acp_clk_round_rate(struct clk *clk, unsigned long rate)
{
	return 0;
}

int acp_clk_set_rate(struct clk *clk, unsigned long rate)
{
	return 0;
}

static struct clk_interface acp_clk_functions = {
	.clk_get                = acp_clk_get,
	.clk_enable             = acp_clk_enable,
	.clk_disable            = acp_clk_disable,
	.clk_get_rate           = acp_clk_get_rate,
	.clk_put                = acp_clk_put,
	.clk_round_rate         = acp_clk_round_rate,
	.clk_set_rate           = acp_clk_set_rate,
	.clk_set_parent         = NULL,
	.clk_get_parent         = NULL,
};

int __init acp_clk_init(void)
{
	clk_functions = acp_clk_functions;
	return 0;
}
EXPORT_SYMBOL(acp_clk_init);
