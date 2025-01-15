// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#ifndef _BST_DRM_UTILS_
#define _BST_DRM_UTILS_

#include <linux/delay.h>
#include <linux/errno.h>

#define has_bit(nr, mask)	(BIT(nr) & (mask))
#define has_bits(bits, mask)	(((bits) & (mask)) == (bits))

#define dp_wait_cond(__cond, __tries, __min_range, __max_range)	\
({							\
	int num_tries = __tries;			\
	while (!__cond && (num_tries > 0)) {		\
		usleep_range(__min_range, __max_range);	\
		num_tries--;				\
	}						\
	(__cond) ? 0 : -ETIMEDOUT;			\
})

struct bstdc_range {
	u32 start;
	u32 end;
};

static inline void set_range(struct bstdc_range *rg, u32 start, u32 end)
{
	rg->start = start;
	rg->end   = end;
}

static inline bool in_range(struct bstdc_range *rg, u32 v)
{
	return (v >= rg->start) && (v <= rg->end);
}

#endif /* _BST_DRM_UTILS_ */
