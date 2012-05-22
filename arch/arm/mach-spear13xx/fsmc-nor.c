/*
 * arch/arm/mach-spear13xx/fsmc-nor.c
 *
 * FSMC (Flexible Static Memory Controller) interface for NOR
 *
 * Copyright (C) 2010 ST Microelectronics
 * Vipin Kumar<vipin.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/mtd/fsmc.h>
#include <linux/platform_device.h>

int __init fsmc_nor_init(struct platform_device *pdev, unsigned long base,
		u32 bank, u32 width)
{
	void __iomem *fsmc_nor_base;
	struct clk *clk;
	int ret;
	u32 ctrl;

	if (bank > (FSMC_MAX_NOR_BANKS - 1))
		return -EINVAL;

	fsmc_nor_base = ioremap(base, FSMC_NOR_REG_SIZE);
	if (!fsmc_nor_base)
		return -ENOMEM;

	clk = clk_get_sys("fsmc-nor", NULL);
	if (IS_ERR(clk)) {
		iounmap(fsmc_nor_base);
		return PTR_ERR(clk);
	}

	ret = clk_enable(clk);
	if (ret) {
		iounmap(fsmc_nor_base);
		return ret;
	}

	ctrl = WAIT_ENB | WRT_ENABLE | WPROT | NOR_DEV | BANK_ENABLE;

	switch (width) {
	case FSMC_FLASH_WIDTH8:
		ctrl |= WIDTH_8;
		break;

	case FSMC_FLASH_WIDTH16:
		ctrl |= WIDTH_16;
		break;

	case FSMC_FLASH_WIDTH32:
		ctrl |= WIDTH_32;
		break;

	default:
		ctrl |= WIDTH_8;
		break;
	}

	writel(ctrl, FSMC_NOR_REG(fsmc_nor_base, bank, CTRL));
	writel(0x0FFFFFFF, FSMC_NOR_REG(fsmc_nor_base, bank, CTRL_TIM));
	writel(ctrl | RSTPWRDWN, FSMC_NOR_REG(fsmc_nor_base, bank, CTRL));

	iounmap(fsmc_nor_base);

	return 0;
}
