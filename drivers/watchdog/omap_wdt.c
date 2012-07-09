/*
 * omap_wdt.c
 *
 * Watchdog driver for the TI OMAP 16xx & 24xx/34xx 32KHz (non-secure) watchdog
 *
 * Author: MontaVista Software, Inc.
 *	 <gdavis@mvista.com> or <source@mvista.com>
 *
 * 2003 (c) MontaVista Software, Inc. This file is licensed under the
 * terms of the GNU General Public License version 2. This program is
 * licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * History:
 *
 * 20030527: George G. Davis <gdavis@mvista.com>
 *	Initially based on linux-2.4.19-rmk7-pxa1/drivers/char/sa1100_wdt.c
 *	(c) Copyright 2000 Oleg Drokin <green@crimea.edu>
 *	Based on SoftDog driver by Alan Cox <alan@lxorguk.ukuu.org.uk>
 *
 * Copyright (c) 2004 Texas Instruments.
 *	1. Modified to support OMAP1610 32-KHz watchdog timer
 *	2. Ported to 2.6 kernel
 *
 * Copyright (c) 2005 David Brownell
 *	Use the driver model and standard identifiers; handle bigger timeouts.
 *
 * Copyright (c) 2012 WindRiver
 *	Changes cater for the current watchdog framework.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/watchdog.h>
#include <linux/reboot.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/moduleparam.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <mach/hardware.h>
#include <plat/prcm.h>

#include "omap_wdt.h"

static unsigned timer_margin;
module_param(timer_margin, uint, 0);
MODULE_PARM_DESC(timer_margin, "initial watchdog timeout (in seconds)");

static unsigned int wdt_trgr_pattern = 0x1234;
static DEFINE_SPINLOCK(wdt_lock);

struct omap_wdt_drvdata {
	struct watchdog_device wdt;
	void __iomem    *base;          /* physical */
	struct device   *dev;
	struct resource *mem;
};

static void omap_wdt_enable(struct omap_wdt_drvdata *wdev)
{
	void __iomem *base = wdev->base;

	/* Sequence to enable the watchdog */
	__raw_writel(0xBBBB, base + OMAP_WATCHDOG_SPR);
	while ((__raw_readl(base + OMAP_WATCHDOG_WPS)) & 0x10)
		cpu_relax();

	__raw_writel(0x4444, base + OMAP_WATCHDOG_SPR);
	while ((__raw_readl(base + OMAP_WATCHDOG_WPS)) & 0x10)
		cpu_relax();
}

static void omap_wdt_disable(struct omap_wdt_drvdata *wdev)
{
	void __iomem *base = wdev->base;

	/* sequence required to disable watchdog */
	__raw_writel(0xAAAA, base + OMAP_WATCHDOG_SPR);	/* TIMER_MODE */
	while (__raw_readl(base + OMAP_WATCHDOG_WPS) & 0x10)
		cpu_relax();

	__raw_writel(0x5555, base + OMAP_WATCHDOG_SPR);	/* TIMER_MODE */
	while (__raw_readl(base + OMAP_WATCHDOG_WPS) & 0x10)
		cpu_relax();
}

static void omap_wdt_adjust_timeout(unsigned new_timeout)
{
	if (new_timeout < TIMER_MARGIN_MIN)
		new_timeout = TIMER_MARGIN_DEFAULT;
	if (new_timeout > TIMER_MARGIN_MAX)
		new_timeout = TIMER_MARGIN_MAX;
	timer_margin = new_timeout;
}

static int omap_wdt_set_timeout(struct watchdog_device *wdt_dev,
				    unsigned int new_timeout)
{
	u32 pre_margin;
	struct omap_wdt_drvdata *omap_wdev = watchdog_get_drvdata(wdt_dev);
	void __iomem *base = omap_wdev->base;

	pm_runtime_get_sync(omap_wdev->dev);
	omap_wdt_disable(omap_wdev);

	/* adjust timeout based on the new timeout */
	omap_wdt_adjust_timeout(new_timeout);
	pre_margin = GET_WLDR_VAL(timer_margin);

	/* just count up at 32 KHz */
	while (__raw_readl(base + OMAP_WATCHDOG_WPS) & 0x04)
		cpu_relax();

	__raw_writel(pre_margin, base + OMAP_WATCHDOG_LDR);
	while (__raw_readl(base + OMAP_WATCHDOG_WPS) & 0x04)
		cpu_relax();

	omap_wdt_enable(omap_wdev);
	wdt_dev->timeout = timer_margin;
	pm_runtime_put_sync(omap_wdev->dev);
	return 0;
}

static int omap_wdt_ping(struct watchdog_device *wdt_dev)
{
	struct omap_wdt_drvdata *omap_wdev = watchdog_get_drvdata(wdt_dev);
	void __iomem    *base = omap_wdev->base;

	pm_runtime_get_sync(omap_wdev->dev);
	spin_lock(&wdt_lock);

	/* wait for posted write to complete */
	while ((__raw_readl(base + OMAP_WATCHDOG_WPS)) & 0x08)
		cpu_relax();

	wdt_trgr_pattern = ~wdt_trgr_pattern;
	__raw_writel(wdt_trgr_pattern, (base + OMAP_WATCHDOG_TGR));

	/* wait for posted write to complete */
	/* reloaded WCRR from WLDR */
	while ((__raw_readl(base + OMAP_WATCHDOG_WPS)) & 0x08)
		cpu_relax();

	spin_unlock(&wdt_lock);
	pm_runtime_put_sync(omap_wdev->dev);

	return 0;
}

static int omap_wdt_start(struct watchdog_device *wdt_dev)
{
	struct omap_wdt_drvdata *omap_wdev = watchdog_get_drvdata(wdt_dev);
	void __iomem *base = omap_wdev->base;

	pm_runtime_get_sync(omap_wdev->dev);
	/* initialize prescaler */
	while (__raw_readl(base + OMAP_WATCHDOG_WPS) & 0x01)
		cpu_relax();

	__raw_writel((1 << 5) | (PTV << 2), base + OMAP_WATCHDOG_CNTRL);
	while (__raw_readl(base + OMAP_WATCHDOG_WPS) & 0x01)
		cpu_relax();

	omap_wdt_set_timeout(&omap_wdev->wdt, timer_margin);
	pm_runtime_put_sync(omap_wdev->dev);

	return 0;
}

static int omap_wdt_stop(struct watchdog_device *wdt_dev)
{
	struct omap_wdt_drvdata *omap_wdev = watchdog_get_drvdata(wdt_dev);

	pm_runtime_get_sync(omap_wdev->dev);
	omap_wdt_disable(omap_wdev);
	pm_runtime_put_sync(omap_wdev->dev);

	return 0;
}

static const struct watchdog_ops omap_wdt_ops = {
	.owner = THIS_MODULE,
	.start = omap_wdt_start,
	.stop = omap_wdt_stop,
	.ping = omap_wdt_ping,
	.set_timeout = omap_wdt_set_timeout,
};

static const struct watchdog_info omap_wdt_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_MAGICCLOSE | WDIOF_KEEPALIVEPING |
		   WDIOF_CARDRESET,
	.identity = "Omap Watchdog",
};

static int __devinit omap_wdt_probe(struct platform_device *pdev)
{
	struct omap_wdt_drvdata *omap_drvdata;
	struct watchdog_device *omap_wdev;
	struct resource *res, *mem;
	int ret;

	/* reserve static register mappings */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -ENOENT;
		goto err_get_resource;
	}

	mem = request_mem_region(res->start, resource_size(res), pdev->name);
	if (!mem) {
		ret = -EBUSY;
		goto err_busy;
	}

	omap_drvdata = devm_kzalloc(&pdev->dev, sizeof(struct omap_wdt_drvdata),
			       GFP_KERNEL);
	if (!omap_drvdata) {
		dev_err(&pdev->dev, "Unable to allocate watchdog device\n");
		ret = -ENOMEM;
		goto err_kzalloc;
	}

	omap_drvdata->dev = &pdev->dev;
	omap_wdev = &omap_drvdata->wdt;
	omap_wdev->info = &omap_wdt_info;
	omap_wdev->ops = &omap_wdt_ops;
	omap_wdt_adjust_timeout(timer_margin);
	omap_wdev->min_timeout = 1;
	omap_wdev->max_timeout = TIMER_MARGIN_MAX;
#ifdef CONFIG_WATCHDOG_NOWAYOUT
	watchdog_set_nowayout(omap_wdev, true);
#endif
	watchdog_set_drvdata(omap_wdev, omap_drvdata);

	omap_drvdata->base = ioremap(res->start, resource_size(res));
	if (!omap_drvdata->base) {
		ret = -ENOMEM;
		goto err_kzalloc;
	}

	ret = watchdog_register_device(&omap_drvdata->wdt);
	if (ret < 0)
		goto err_register_wd;

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);
	omap_wdt_disable(omap_drvdata);
	pm_runtime_put_sync(&pdev->dev);
	platform_set_drvdata(pdev, omap_drvdata);

	/* For omap16xx we just keep it original as-is */
	if (cpu_is_omap16xx())
		omap_wdev->bootstatus = __raw_readw(ARM_SYSST);
	else
		omap_wdev->bootstatus = (omap_prcm_get_reset_sources() & 0x10)
					 >> OMAP3_PRM_RSTST_WD_BIT;
	pr_info("OMAP WDTimer Rev 0x%02x: Initial timeout %dsec status= 0x%x\n",
		 __raw_readl(omap_drvdata->base + OMAP_WATCHDOG_REV)
		 & 0xFF, timer_margin, omap_wdev->bootstatus);

	return 0;

err_register_wd:
	iounmap(omap_drvdata->base);

err_kzalloc:
	release_mem_region(res->start, resource_size(res));

err_busy:
err_get_resource:

	return ret;
}

static int __devexit omap_wdt_remove(struct platform_device *pdev)
{
	struct omap_wdt_drvdata *omap_wdev = platform_get_drvdata(pdev);
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	pm_runtime_disable(&pdev->dev);

	release_mem_region(res->start, resource_size(res));
	platform_set_drvdata(pdev, NULL);

	iounmap(omap_wdev->base);

	return 0;
}

#ifdef	CONFIG_PM

/* REVISIT ... not clear this is the best way to handle system suspend; and
 * it's very inappropriate for selective device suspend (e.g. suspending this
 * through sysfs rather than by stopping the watchdog daemon).  Also, this
 * may not play well enough with NOWAYOUT...
 */

static int omap_wdt_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct omap_wdt_drvdata *omap_wdev = platform_get_drvdata(pdev);

	if (test_bit(WDOG_DEV_OPEN, &omap_wdev->wdt.status)) {
		pm_runtime_get_sync(&pdev->dev);
		omap_wdt_disable(omap_wdev);
		pm_runtime_put_sync(&pdev->dev);
	}

	return 0;
}

static int omap_wdt_resume(struct platform_device *pdev)
{
	struct omap_wdt_drvdata *omap_wdev = platform_get_drvdata(pdev);
	if (test_bit(WDOG_DEV_OPEN, &omap_wdev->wdt.status)) {
		pm_runtime_get_sync(&pdev->dev);
		omap_wdt_enable(omap_wdev);
		omap_wdt_ping(&omap_wdev->wdt);
		pm_runtime_put_sync(&pdev->dev);
	}

	return 0;
}

#else
#define	omap_wdt_suspend	NULL
#define	omap_wdt_resume		NULL
#endif

static struct platform_driver omap_wdt_driver = {
	.probe		= omap_wdt_probe,
	.remove		= __devexit_p(omap_wdt_remove),
	.suspend	= omap_wdt_suspend,
	.resume		= omap_wdt_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "omap_wdt",
	},
};

module_platform_driver(omap_wdt_driver);

MODULE_AUTHOR("George G. Davis");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:omap_wdt");
