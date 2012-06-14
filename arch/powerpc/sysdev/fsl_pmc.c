/*
 * Suspend/resume support
 *
 * Copyright 2009  MontaVista Software, Inc.
 * Copyright 2007-2011 Freescale Semiconductor Inc.
 *
 * Author: Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/suspend.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/of_platform.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <sysdev/fsl_soc.h>

struct pmc_regs {
	__be32 devdisr;
	__be32:32;
	__be32:32;
	__be32:32;
	__be32 pmcsr;
	__be32:32;
	__be32:32;
	__be32 pmcdr;
};
static struct device *pmc_dev;
static struct pmc_regs __iomem *pmc_regs;

#define PMCSR_SLP	0x00020000
#define PMCSR_LOSSLESS	0x00400000
static int has_deep_sleep, has_lossless;

/* Cast the ccsrbar to 64-bit parameter so that the assembly
 * code can be compatible with both 32-bit & 36-bit */
void mpc85xx_enter_deep_sleep(u64 ccsrbar, u32 powmgtreq);

/**
 * pmc_enable_wake - enable OF device as wakeup event source
 * @ofdev: OF device affected
 * @state: PM state from which device will issue wakeup events
 * @enable: True to enable event generation; false to disable
 *
 * This enables the device as a wakeup event source, or disables it.
 *
 * RETURN VALUE:
 * 0 is returned on success
 * -EINVAL is returned if device is not supposed to wake up the system
 * Error code depending on the platform is returned if both the platform and
 * the native mechanism fail to enable the generation of wake-up events
 */
int pmc_enable_wake(struct of_device *ofdev, suspend_state_t state, bool enable)
{
	int ret = 0;
	struct device_node *clk_np;
	u32 *pmcdr_mask;

	if (enable && !device_may_wakeup(&ofdev->dev))
		return -EINVAL;

	clk_np = of_parse_phandle(ofdev->node, "clk-handle", 0);
	if (!clk_np)
		return -EINVAL;

	pmcdr_mask = (u32 *)of_get_property(clk_np, "fsl,pmcdr-mask", NULL);
	if (!pmcdr_mask) {
		ret = -EINVAL;
		goto out;
	}

	/* clear to enable clock in low power mode */
	if (enable)
		clrbits32(&pmc_regs->pmcdr, *pmcdr_mask);
	else
		setbits32(&pmc_regs->pmcdr, *pmcdr_mask);

out:
	of_node_put(clk_np);
	return ret;
}
EXPORT_SYMBOL_GPL(pmc_enable_wake);

/**
 * pmc_enable_lossless - enable lossless ethernet in low power mode
 * @enable: True to enable event generation; false to disable
 */
void pmc_enable_lossless(int enable)
{
	if (enable && has_lossless)
		setbits32(&pmc_regs->pmcsr, PMCSR_LOSSLESS);
	else
		clrbits32(&pmc_regs->pmcsr, PMCSR_LOSSLESS);
}
EXPORT_SYMBOL_GPL(pmc_enable_lossless);

static int pmc_suspend_enter(suspend_state_t state)
{
	int ret;
	u32 powmgtreq = 0x00500000;

	switch (state) {
	case PM_SUSPEND_MEM:
#ifdef CONFIG_SPE
		enable_kernel_spe();
#endif
		pr_debug("Entering deep sleep\n");

		local_irq_disable();
		mpc85xx_enter_deep_sleep(get_immrbase(),
				powmgtreq);
		pr_debug("Resumed from deep sleep\n");

		return 0;

	/* else fall-through */
	case PM_SUSPEND_STANDBY:
		local_irq_disable();

		setbits32(&pmc_regs->pmcsr, PMCSR_SLP);

		/* At this point, the CPU is asleep. */
		/* Upon resume, wait for SLP bit to be clear. */
		ret = spin_event_timeout((in_be32(&pmc_regs->pmcsr) & PMCSR_SLP)
				== 0, 10000, 10) ? 0 : -ETIMEDOUT;
		if (ret)
			dev_err(pmc_dev,
				"timeout waiting for SLP bit to be cleared\n");

		return 0;

	default:
		return -EINVAL;

	}

}

static int pmc_suspend_valid(suspend_state_t state)
{
	if (state == PM_SUSPEND_STANDBY)
		return 1;
	if (has_deep_sleep && (state == PM_SUSPEND_MEM))
		return 1;
	return 0;
}

static struct platform_suspend_ops pmc_suspend_ops = {
	.valid = pmc_suspend_valid,
	.enter = pmc_suspend_enter,
};

static int pmc_probe(struct of_device *ofdev, const struct of_device_id *id)
{
	struct device_node *np = ofdev->node;
	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "fsl,mpc8548-pmc");

	if (node) {
		pmc_regs = of_iomap(ofdev->node, 0);
		if (!pmc_regs)
			return -ENOMEM;

		if (of_device_is_compatible(np, "fsl,mpc8536-pmc"))
			has_deep_sleep = 1;

		if (of_device_is_compatible(np, "fsl,p1022-pmc"))
			has_lossless = 1;

		of_node_put(node);
	}

	pmc_dev = &ofdev->dev;
	suspend_set_ops(&pmc_suspend_ops);
	return 0;
}

static const struct of_device_id pmc_ids[] = {
	{ .compatible = "fsl,mpc8548-pmc", },
	{ .compatible = "fsl,mpc8641d-pmc", },
	{ },
};

static struct of_platform_driver pmc_driver = {
	.driver.name = "fsl-pmc",
	.match_table = pmc_ids,
	.probe = pmc_probe,
};

static int __init pmc_init(void)
{
	return of_register_platform_driver(&pmc_driver);
}
device_initcall(pmc_init);
