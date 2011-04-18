/*
 * Copyright (C) 2010 Wind River Systems
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
#include <asm/clk_interface.h>

struct clk *acp_clk_get(struct device *dev, const char *id)
{
	return NULL;
}

void acp_clk_put(struct clk *clk)
{
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
	return 0;
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
