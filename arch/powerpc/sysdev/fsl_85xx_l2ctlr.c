/*
 * Copyright 2009-2010 Freescale Semiconductor, Inc.
 *
 * QorIQ (P1/P2) L2 controller init for Cache-SRAM instantiation
 *
 * Author: Vivek Mahajan <vivek.mahajan@freescale.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/of_platform.h>
#include <asm/io.h>

#include "fsl_85xx_cache_ctlr.h"

static char *param;
struct mpc85xx_l2ctlr __iomem *l2ctlr;

static long get_cache_sram_size(void)
{
	unsigned long val;

	if (!param || (strict_strtoul(param, 0, &val) < 0))
		return -EINVAL;

	return val;
}

static int __init get_cmdline_param(char *str)
{
	if (!str)
		return 0;

	param = str;
	return 1;
}

__setup("cache-sram-size=", get_cmdline_param);

static int __devinit mpc85xx_l2ctlr_of_probe(struct of_device *dev,
					  const struct of_device_id *match)
{
	long rval;
	unsigned int rem;
	unsigned char ways;
	const unsigned int *prop;
	unsigned int l2cache_size;
	unsigned int sram_size;
	struct device_node *np;
	int rc = 1;
	struct resource rsrc;
	int i = 0;
	bool amp = 0;
	static char *compatible_list[] = {
					"fsl,MPC85XXRDB-CAMP",
					"fsl,P2020DS-CAMP",
					NULL
				};

	if (!dev->node) {
		dev_err(&dev->dev, "Device's OF-node is NULL\n");
		return -EINVAL;
	}

	prop = of_get_property(dev->node, "cache-size", NULL);
	if (!prop) {
		dev_err(&dev->dev, "Missing L2 cache-size\n");
		return -EINVAL;
	}
	l2cache_size = *prop;

	rval = get_cache_sram_size();
	if (rval <= 0) {
		dev_err(&dev->dev,
			"Entire L2 as cache, Aborting Cache-SRAM stuff\n");
		return -EINVAL;
	}

	rem = l2cache_size % (unsigned int)rval;
	ways = LOCK_WAYS_FULL * (unsigned int)rval / l2cache_size;
	if (rem || (ways & (ways - 1))) {
		dev_err(&dev->dev, "Illegal cache-sram-size in command line\n");
		return -EINVAL;
	}

	sram_size = (unsigned int)rval;

	l2ctlr = of_iomap(dev->node, 0);
	if (!l2ctlr) {
		dev_err(&dev->dev, "Can't map L2 controller\n");
		return -EINVAL;
	}

	for_each_compatible_node(np, NULL, "fsl,l2sram")
			rc = of_address_to_resource(np, 0, &rsrc);

	if (rc) {
		dev_err(&dev->dev, "l2sram node not found\n");
		iounmap(l2ctlr);
		return -EFAULT;
	}

	/*
	 * Write bits[0-17] to srbar0
	 */
	out_be32(&l2ctlr->srbar0,
		rsrc.start & L2SRAM_BAR_MSK_LO18);

	/*
	 * Write bits[18-21] to srbare0
	 */
#ifdef CONFIG_PHYS_64BIT
	out_be32(&l2ctlr->srbarea0,
		(rsrc.start >> 32) & L2SRAM_BARE_MSK_HI4);
#endif

	rsrc.end = rsrc.start + sram_size - 1;

	clrsetbits_be32(&l2ctlr->ctl, L2CR_L2E, L2CR_L2FI);

	switch (ways) {
	case LOCK_WAYS_EIGHTH:
		setbits32(&l2ctlr->ctl,
			L2CR_L2E | L2CR_L2FI | L2CR_SRAM_EIGHTH);
		break;

	case LOCK_WAYS_TWO_EIGHTH:
		setbits32(&l2ctlr->ctl,
			L2CR_L2E | L2CR_L2FI | L2CR_SRAM_QUART);
		break;

	case LOCK_WAYS_HALF:
		setbits32(&l2ctlr->ctl,
			L2CR_L2E | L2CR_L2FI | L2CR_SRAM_HALF);
		break;

	case LOCK_WAYS_FULL:
	default:
		setbits32(&l2ctlr->ctl,
			L2CR_L2E | L2CR_L2FI | L2CR_SRAM_FULL);
		break;
	}
	eieio();

	for (i = 0; compatible_list[i] != NULL; i++) {
		np = of_find_compatible_node(NULL, NULL, compatible_list[i]);
		if (np) {
			amp = 1;
			break;
		}
	}

	rval = instantiate_cache_sram(dev, &rsrc, amp);
	if (rval < 0) {
		dev_err(&dev->dev, "Can't instantiate Cache-SRAM\n");
		iounmap(l2ctlr);
		return -EINVAL;
	}

	return 0;
}

static int __devexit mpc85xx_l2ctlr_of_remove(struct of_device *dev)
{
	BUG_ON(!l2ctlr);

	iounmap(l2ctlr);
	remove_cache_sram(dev);
	dev_info(&dev->dev, "MPC85xx L2 controller unloaded\n");

	return 0;
}

static struct of_device_id mpc85xx_l2ctlr_of_match[] = {
	{
		.compatible = "fsl,p2020-l2-cache-controller",
	},
	{
		.compatible = "fsl,p2010-l2-cache-controller",
	},
	{
		.compatible = "fsl,p1020-l2-cache-controller",
	},
	{
		.compatible = "fsl,p1011-l2-cache-controller",
	},
	{
		.compatible = "fsl,p1013-l2-cache-controller",
	},
	{
		.compatible = "fsl,p1022-l2-cache-controller",
	},
	{},
};

static struct of_platform_driver mpc85xx_l2ctlr_of_platform_driver = {
	.driver	= {
		.name		= "fsl-l2ctlr",
		.owner		= THIS_MODULE,
		.of_match_table	= mpc85xx_l2ctlr_of_match,
	},
	.probe		= mpc85xx_l2ctlr_of_probe,
	.remove		= __devexit_p(mpc85xx_l2ctlr_of_remove),
};

static __init int mpc85xx_l2ctlr_of_init(void)
{
	return of_register_platform_driver(&mpc85xx_l2ctlr_of_platform_driver);
}

static void __exit mpc85xx_l2ctlr_of_exit(void)
{
	of_unregister_platform_driver(&mpc85xx_l2ctlr_of_platform_driver);
}

subsys_initcall(mpc85xx_l2ctlr_of_init);
module_exit(mpc85xx_l2ctlr_of_exit);

MODULE_DESCRIPTION("Freescale MPC85xx L2 controller init");
MODULE_LICENSE("GPL v2");
