// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#ifndef _DC_IO_H_
#define _DC_IO_H_

#include <linux/io.h>

static inline u32
bstdc_read32(u32 __iomem *reg_base, u32 reg_offset)
{
	return readl((reg_base + (reg_offset >> 2)));
}

static inline void
bstdc_write32(u32 __iomem *reg_base, u32 reg_offset, u32 value)
{
	writel(value, (reg_base + (reg_offset >> 2)));
}

static inline void
bstdc_write64(u32 __iomem *reg_base, u32 reg_offset, u64 value)
{
	writel(lower_32_bits(value), (reg_base + (reg_offset >> 2)));
	writel(upper_32_bits(value), (reg_base + (reg_offset >> 2) + 1));
}

static inline void
bstdc_write32_mask(u32 __iomem *reg_base, u32 reg_offset, u32 m, u32 value)
{
	u32 tmp = bstdc_read32(reg_base, reg_offset);

	tmp &= (~m);
	bstdc_write32(reg_base, reg_offset, value | tmp);
}

static inline void
bstdc_write_group(u32 __iomem *reg_base, u32 reg_offset, int num, const u32 *values)
{
	int i;

	for (i = 0; i < num; i++)
		bstdc_write32(reg_base, reg_offset + i * 4, values[i]);
}

#endif /*_DC_IO_H_*/
