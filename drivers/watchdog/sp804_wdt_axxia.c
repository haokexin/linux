// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 INTEL Corporation

/*
 * Watchdog driver for ARM SP804 watchdog module on Axxia evice
 *
 * John Logan <john.logan@intel.com>
 *
 * Based on SP805 driver by Viresh Kumar
 * Copyright (C) 2010 ST Microelectronics
 * Viresh Kumar <viresh.linux@gmail.com>
 */

#include <linux/device.h>
#include <linux/resource.h>
#include <linux/amba/bus.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/watchdog.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/of.h>

#include <linux/platform_device.h>
#include <linux/interrupt.h>

/* default timeout in seconds */
#define DEFAULT_TIMEOUT		17

#define MODULE_NAME		"sp804_wdt_axxia"

/* watchdog register offsets and masks */
#define WDTLOAD			0x000
#define LOAD_MIN	0x00000001
#define LOAD_MAX	0xFFFFFFFF
#define WDTVALUE		0x004
#define WDTCONTROL		0x008
/* control register masks */
#define ONE_SHOT_MODE   BIT(0)
#define TIM_SIZE	BIT(1)
#define PRESCALE	BIT(2)
#define	INT_ENABLE	BIT(5)
#define	TIMER_MODE	BIT(6)
#define TIMER_ENABLE    BIT(7)

#define WDTINTCLR	0x00C
#define WDTRIS		0x010
#define WDTMIS		0x014
#define INT_MASK	BIT(0)

#define SYSCON_CTL_WR_KEY			0x2000
#define SYSCON_CTL_RST_REG			0x2008
#define SYSCON_RST_WD0_EN			BIT(8)
#define SYSCON_RST_WD0_MASK			GENMASK(8, 8)
#define SYSCON_RST_WD1_EN			BIT(9)
#define SYSCON_RST_WD1_MASK			GENMASK(9, 9)

/**
 * struct sp804_wdt_axxia: sp804 wdt device structure
 * @wdd: instance of struct watchdog_device
 * @lock: spin lock protecting dev structure and io access
 * @base: base address of wdt
 * @clk: clock structure of wdt
 * @adev: amba device structure of wdt
 * @status: current status of wdt
 * @load_val: load value to be set for current timeout
 */
struct sp804_wdt_axxia {
	struct watchdog_device		wdd;
	spinlock_t			lock; /* */
	void __iomem			*base;
	struct clk			*clk;
	struct amba_device		*adev;
	unsigned int			load_val;
	int id;
	struct regmap *syscon;
};

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout,
		 "Set to 1 to keep watchdog running after device release");

/* This routine finds load value that will reset system in required timeout */
static int wdt_setload(struct watchdog_device *wdd, unsigned int timeout)
{
	struct sp804_wdt_axxia *wdt = watchdog_get_drvdata(wdd);
	u64 load, rate;

	rate = clk_get_rate(wdt->clk);

	/*
	 * sp805 runs counter with given value twice, after the end of first
	 * counter it gives an interrupt and then starts counter again. If
	 * interrupt already occurred then it resets the system. This is why
	 * load is half of what should be required.

	 * sp804 only runs counter once - when it times out, a reset occurs
	 */
	load = rate * timeout - 1;

	load = (load > LOAD_MAX) ? LOAD_MAX : load;
	load = (load < LOAD_MIN) ? LOAD_MIN : load;

	spin_lock(&wdt->lock);
	wdt->load_val = load;
	/* roundup timeout to closest positive integer value */
	wdd->timeout = div_u64((load + 1) + (rate / 2), rate);
	spin_unlock(&wdt->lock);

	return 0;
}

/* returns number of seconds left for reset to occur */
static unsigned int wdt_timeleft(struct watchdog_device *wdd)
{
	struct sp804_wdt_axxia *wdt = watchdog_get_drvdata(wdd);
	u64 load, rate;

	rate = clk_get_rate(wdt->clk);
	if (!rate) {
		dev_err(&wdt->adev->dev, "rate is 0");
		return 0;
	}

	spin_lock(&wdt->lock);
	load = readl_relaxed(wdt->base + WDTVALUE);

	spin_unlock(&wdt->lock);

	return div_u64(load, rate);
}

static int wdt_config(struct watchdog_device *wdd, bool ping)
{
	struct sp804_wdt_axxia *wdt = watchdog_get_drvdata(wdd);
	int ret;

	if (!ping) {
		ret = clk_prepare_enable(wdt->clk);
		if (ret) {
			dev_err(&wdt->adev->dev, "clock enable fail");
			return ret;
		}
	}

	spin_lock(&wdt->lock);

	writel_relaxed(wdt->load_val, wdt->base + WDTLOAD);

	if (!ping) {
		writel_relaxed(INT_MASK, wdt->base + WDTINTCLR);
		writel_relaxed(TIM_SIZE | INT_ENABLE | TIMER_MODE |
			       TIMER_ENABLE, wdt->base + WDTCONTROL);
	}

	/* Flush posted writes. */
	readl_relaxed(wdt->base + WDTCONTROL);
	spin_unlock(&wdt->lock);

	return 0;
}

static int wdt_ping(struct watchdog_device *wdd)
{
	return wdt_config(wdd, true);
}

/* enables watchdog timers reset */
static int wdt_enable(struct watchdog_device *wdd)
{
	return wdt_config(wdd, false);
}

/* disables watchdog timers reset */
static int wdt_disable(struct watchdog_device *wdd)
{
	struct sp804_wdt_axxia *wdt = watchdog_get_drvdata(wdd);

	spin_lock(&wdt->lock);

	writel_relaxed(0, wdt->base + WDTCONTROL);

	/* Flush posted writes. */
	readl_relaxed(wdt->base + WDTCONTROL);
	spin_unlock(&wdt->lock);

	clk_disable_unprepare(wdt->clk);

	return 0;
}

/* interrupt handler code */

static irqreturn_t sp804_wdt_axxia_irq(int irqno, void *param)
{
	pr_err("Watchdog expired! Should not reach this line!\n");

	return IRQ_HANDLED;
}

static const struct watchdog_info wdt_info = {
	.options = WDIOF_MAGICCLOSE | WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING,
	.identity = MODULE_NAME,
};

static const struct watchdog_ops wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= wdt_enable,
	.stop		= wdt_disable,
	.ping		= wdt_ping,
	.set_timeout	= wdt_setload,
	.get_timeleft	= wdt_timeleft,
};

static int
sp804_wdt_axxia_probe(struct amba_device *adev, const struct amba_id *id)
{
	struct sp804_wdt_axxia *wdt;
	struct device_node *np = adev->dev.of_node;
	int ret = 0;
	unsigned int irq;
	unsigned int mask, value;

	if (!of_device_is_compatible(np, "arm,sp804_wdt_axxia"))
		return -ENODEV;

	dev_info(&adev->dev, "Probing sp804_wdt_axxia\n");

	wdt = devm_kzalloc(&adev->dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt) {
		ret = -ENOMEM;
		goto err;
	}

	wdt->syscon = syscon_regmap_lookup_by_phandle(np, "syscon");
	if (IS_ERR(wdt->syscon)) {
		dev_err(&adev->dev, "sp804_wdt: syscon lookup failed\n");
		ret = PTR_ERR(wdt->syscon);
		goto err;
	}

	ret = of_property_read_u32(np, "watchdog_id", &wdt->id);
	if (ret < 0) {
		dev_err(&adev->dev, "sp804_wdt: get watchdog_id failed\n");
		goto err;
	}

	wdt->base = devm_ioremap_resource(&adev->dev, &adev->res);
	if (IS_ERR(wdt->base))
		return PTR_ERR(wdt->base);

	wdt->clk = devm_clk_get(&adev->dev, NULL);
	if (IS_ERR(wdt->clk)) {
		dev_warn(&adev->dev, "Clock not found\n");
		ret = PTR_ERR(wdt->clk);
		goto err;
	}

	irq = adev->irq[0];

	ret = devm_request_irq(&adev->dev, irq, sp804_wdt_axxia_irq, 0,
			       dev_name(&adev->dev), wdt);
	if (ret != 0) {
		dev_err(&adev->dev, "failed to install irq (%d)\n", ret);
		goto err;
	}

	/*
	 * Write the magic number 0xab to 0x2000 before writing to other
	 * critical system control registers.
	 */
	regmap_write(wdt->syscon, SYSCON_CTL_WR_KEY, 0xab);

	/* Update enable and select bits */
	mask = wdt->id ? SYSCON_RST_WD1_MASK : SYSCON_RST_WD0_MASK;
	value = wdt->id ? SYSCON_RST_WD1_EN : SYSCON_RST_WD0_EN;
	regmap_update_bits(wdt->syscon, SYSCON_CTL_RST_REG, mask, value);

	/* Restore it to 0 to keep safe. */
	regmap_write(wdt->syscon, SYSCON_CTL_WR_KEY, 0x0);

	wdt->adev = adev;
	wdt->wdd.info = &wdt_info;
	wdt->wdd.ops = &wdt_ops;
	wdt->wdd.parent = &adev->dev;

	spin_lock_init(&wdt->lock);
	watchdog_set_nowayout(&wdt->wdd, nowayout);
	watchdog_set_drvdata(&wdt->wdd, wdt);
	wdt_setload(&wdt->wdd, DEFAULT_TIMEOUT);

	ret = watchdog_register_device(&wdt->wdd);
	if (ret) {
		dev_err(&adev->dev, "watchdog_register_device() failed: %d\n",
			ret);
		goto err;
	}
	amba_set_drvdata(adev, wdt);

	dev_info(&adev->dev, "registration successful\n");
	return 0;

err:
	dev_err(&adev->dev, "Probe Failed!!!\n");
	return ret;
}

static void sp804_wdt_axxia_remove(struct amba_device *adev)
{
	struct sp804_wdt_axxia *wdt = amba_get_drvdata(adev);
	unsigned int mask, value;

	watchdog_unregister_device(&wdt->wdd);
	watchdog_set_drvdata(&wdt->wdd, NULL);

	/* Disable watchdog reset */
	regmap_write(wdt->syscon, SYSCON_CTL_WR_KEY, 0xab);
	mask = wdt->id ? SYSCON_RST_WD1_MASK : SYSCON_RST_WD0_MASK;
	value = 0;
	regmap_update_bits(wdt->syscon, SYSCON_CTL_RST_REG, mask, value);
	regmap_write(wdt->syscon, SYSCON_CTL_WR_KEY, 0x0);
}

static int __maybe_unused sp804_wdt_axxia_suspend(struct device *dev)
{
	struct sp804_wdt_axxia *wdt = dev_get_drvdata(dev);

	if (watchdog_active(&wdt->wdd))
		return wdt_disable(&wdt->wdd);

	return 0;
}

static int __maybe_unused sp804_wdt_axxia_resume(struct device *dev)
{
	struct sp804_wdt_axxia *wdt = dev_get_drvdata(dev);

	if (watchdog_active(&wdt->wdd))
		return wdt_enable(&wdt->wdd);

	return 0;
}

static SIMPLE_DEV_PM_OPS(sp804_wdt_axxia_dev_pm_ops, sp804_wdt_axxia_suspend,
			 sp804_wdt_axxia_resume);

static struct amba_id sp804_wdt_axxia_ids[] = {
	{
		.id	= 0x00141804,
		.mask	= 0x00ffffff,

	},
	{ 0, 0 },
};

MODULE_DEVICE_TABLE(amba, sp804_wdt_axxia_ids);

static struct amba_driver sp804_wdt_axxia_driver = {
	.drv = {
		.name	= MODULE_NAME,
		.pm	= &sp804_wdt_axxia_dev_pm_ops,
	},
	.id_table	= sp804_wdt_axxia_ids,
	.probe		= sp804_wdt_axxia_probe,
	.remove = sp804_wdt_axxia_remove,
};

module_amba_driver(sp804_wdt_axxia_driver);

MODULE_AUTHOR("John Logan <john.logan@intel.com>");
MODULE_DESCRIPTION("ARM SP804 Watchdog Driver For Axxia");
MODULE_LICENSE("GPL");
