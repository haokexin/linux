/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Author: Marek Bykowski <marek.bykowski@intel.com>
 *
 * Copyright (C) 2021 INTEL Corporation
 */

#ifndef __LINUX_ARM_CCN_H
#define __LINUX_ARM_CCN_H

#include <linux/list.h>
#include <linux/irqreturn.h>
#include <asm/arm-ccn.h>

struct error_reporting {
	const char *match;
	void *data;
	irqreturn_t (*handler)(void *data);
	struct list_head child;
};

#if defined(CONFIG_ARM_CCN)
void arm_ccn_handler_add(struct list_head *new);
#else
void arm_ccn_handler_add(struct list_head *new)
{ return; }
#endif

#endif
