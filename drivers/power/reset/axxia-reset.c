// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 INTEL Corporation
/*
 * Reset driver for Axxia devices
 */

#include <linux/init.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <asm/system_misc.h>

#define SC_PSCRATCH            0x00dc
#define SC_CRIT_WRITE_KEY      0x2000
#define SC_RESET_CONTROL       0x2008
#define RSTCTL_RST_CHIP        BIT(1)
#define RSTCTL_RST_SYS         BIT(0)

static struct regmap *syscon;

static int ddr_retention_enabled;

void
initiate_retention_reset(void)
{
	if (ddr_retention_enabled == 0) {
		pr_info("DDR Retention Reset is Not Enabled\n");
		return;
	}

	if (WARN_ON(!syscon))
		return;

	/* set retention reset bit in pscratch */
	regmap_update_bits(syscon, SC_PSCRATCH, 1, 1);
}
EXPORT_SYMBOL(initiate_retention_reset);

static ssize_t
axxia_ddr_retention_trigger(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	machine_restart(NULL);
	return count;
}

static const struct proc_ops axxia_ddr_retention_proc_ops = {
	.proc_write      = axxia_ddr_retention_trigger,
	.proc_lseek      = noop_llseek,
};

void
axxia_ddr_retention_init(void)
{
	struct device_node *ddr_retention;
	const unsigned int *state;

	/*
	 * this feature is only meaningful on ASIC systems,
	 * but for now we allow it on simulator
	 */

	/*
	 * Newer versions of Axxia U-Boot will add a property to the
	 * device tree to indicate whether or not DDR retention is
	 * enabled. If that property exists, use it; otherwise, use the
	 * Linux configuration option.
	 */

	ddr_retention = of_find_node_by_name(NULL, "ddr_retention");
	state = of_get_property(ddr_retention, "state", NULL);

	if (ddr_retention && state) {
		ddr_retention_enabled = (be32_to_cpu(*state) == 0 ? 0 : 1);
	}
	else {
#ifdef CONFIG_POWER_RESET_AXXIA_DDR_RETENTION
		ddr_retention_enabled = 1;
#else
		ddr_retention_enabled = 0;
#endif
	}

	if (ddr_retention_enabled) {
		/* Create /proc entry */
		if (!proc_create("driver/axxia_ddr_retention_reset", 0200, NULL, &axxia_ddr_retention_proc_ops)) {
			pr_info("Failed to register DDR retention proc entry\n");
		}
		else {
			pr_info("DDR Retention Reset Initialized\n");
		}
	}
	else {
		pr_info("DDR Retention Reset is Not Available\n");
	}
}

static int axxia_restart_handler(struct notifier_block *this, unsigned long mode, void *cmd)
{
	if (ddr_retention_enabled) {
		initiate_retention_reset();
	}
	else {
		/* Access Key (0xab) */
		regmap_write(syscon, SC_CRIT_WRITE_KEY, 0xab);

		/* Assert chip reset */
		regmap_update_bits(syscon, SC_RESET_CONTROL, RSTCTL_RST_CHIP, RSTCTL_RST_CHIP);
	}

	return NOTIFY_DONE;
}

static struct notifier_block axxia_restart_nb = {
	.notifier_call = axxia_restart_handler,
};

static int axxia_reset_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int err;

	syscon = syscon_regmap_lookup_by_phandle(dev->of_node, "syscon");
	if (IS_ERR(syscon)) {
		pr_err("%pOFn: syscon lookup failed\n", dev->of_node);
		return PTR_ERR(syscon);
	}

	axxia_restart_nb.priority = ddr_retention_enabled ? 130 : 128;
	err = register_restart_handler(&axxia_restart_nb);
	if (err)
		dev_err(dev, "cannot register restart handler (err=%d)\n", err);

	return err;
}

static const struct of_device_id of_axxia_reset_match[] = {
	{ .compatible = "intel,axm56xx-reset",},
	{},
};
MODULE_DEVICE_TABLE(of, of_axxia_reset_match);

static struct platform_driver axxia_reset_driver = {
	.probe = axxia_reset_probe,
	.driver = {
		.name = "axxia-reset",
		.of_match_table = of_axxia_reset_match,
	},
};

static int __init axxia_reset_init(void)
{
	axxia_ddr_retention_init();
	return platform_driver_register(&axxia_reset_driver);
}
device_initcall(axxia_reset_init);
