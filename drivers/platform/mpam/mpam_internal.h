// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 Arm Ltd.

#ifndef MPAM_INTERNAL_H
#define MPAM_INTERNAL_H

#include <linux/cpumask.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/resctrl.h>
#include <linux/sizes.h>

struct mpam_msc
{
	/* member of mpam_all_msc */
	struct list_head        glbl_list;

	int			id;
	struct platform_device *pdev;
	cpumask_t		accessibility;

	spinlock_t		lock;
	u32			nrdy_usec;
	unsigned long		ris_idxs[128 / BITS_PER_LONG];
	u32			ris_max;

	/* mpam_msc_ris of this component */
	struct list_head	ris;

	/*
	 * hw_lock protects access to the MSC hardware registers. Take
	 * msc->lock first.
	 */
	spinlock_t		hw_lock;
	void __iomem *		mapped_hwpage;
	size_t			mapped_hwpage_sz;
};

#endif /* MPAM_INTERNAL_H */
