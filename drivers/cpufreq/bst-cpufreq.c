// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
 *
 * Cpufreq driver for the Black Sesame Soc
 *
 * Copyright (C) 2024 Black Sesame Technologies, Inc.
 * Author: Xuran Yang <xuran.yang@bst.ai>
 *
 */

#include <linux/bitfield.h>
#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/interconnect.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/pm_opp.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/units.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>

enum {
	CPU_25MHZ = 0,
	CPU_1700MHZ,
	CPU_1200MHZ,
	CPU_500MHZ,
	CPU_INVALID
};

struct bst_soc_cpufreq_data {
	struct cpufreq_frequency_table *freq_table;
};

struct bst_cpufreq_dev {
	u64 freq_sel_base;
	struct clk *clk;
	int domain;
	struct bst_soc_cpufreq_data *data;
};

struct cpufreq_frequency_table c1200_cpu_clock_table[] = {
	{0, CPU_25MHZ, 25000},
	{0, CPU_1700MHZ, 1700000},
	{0, CPU_1200MHZ, 1200000},
	{0, CPU_500MHZ, 500000},
	{0, CPU_INVALID, CPUFREQ_TABLE_END},
};

static const struct bst_soc_cpufreq_data bst_c1200_data = {
	.freq_table = c1200_cpu_clock_table,
};

static const struct of_device_id bst_cpufreq_match[] = {
	{ .compatible = "bst,cpufreq-c1200", .data = &bst_c1200_data },
	{}
};
MODULE_DEVICE_TABLE(of, bst_cpufreq_match);

static void bst_get_related_cpus(int index, struct cpumask *m)
{
	struct device_node *cpu_np;
	struct of_phandle_args args;
	int cpu, ret;

	for_each_possible_cpu(cpu) {
		cpu_np = of_cpu_device_node_get(cpu);
		if (!cpu_np)
			continue;

		ret = of_parse_phandle_with_args(cpu_np, "freq-domain",
						 "#freq-domain-cells", 0,
						 &args);
		of_node_put(cpu_np);
		if (ret < 0)
			continue;

		if (index == args.args[0])
			cpumask_set_cpu(cpu, m);
	}
}

static int bst_cpufreq_cpu_init(struct cpufreq_policy *policy)
{
	struct platform_device *pdev = cpufreq_get_driver_data();
	struct device *dev = &pdev->dev;
	struct bst_soc_cpufreq_data *data = (struct bst_soc_cpufreq_data *)
					    of_device_get_match_data(&pdev->dev);
	struct bst_cpufreq_dev *bst_cpufreq_dev;
	struct of_phandle_args args;
	struct device_node *cpu_np;
	struct device *cpu_dev;
	struct resource *res;
	int ret, index;
	char cluster_name[32];

	cpu_dev = get_cpu_device(policy->cpu);
	if (!cpu_dev) {
		pr_err("%s: failed to get cpu%d device\n", __func__,
		       policy->cpu);
		return -ENODEV;
	}

	cpu_np = of_cpu_device_node_get(policy->cpu);
	if (!cpu_np)
		return -EINVAL;

	ret = of_parse_phandle_with_args(cpu_np, "freq-domain",
					 "#freq-domain-cells", 0, &args);
	of_node_put(cpu_np);
	if (ret)
		return ret;

	index = args.args[0];
	pr_info("get cpu clk domain index %d\n", index);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "failed to get cpu clock mem resource\n");
		return -ENODEV;
	}

	bst_cpufreq_dev = devm_kzalloc(dev, sizeof(*bst_cpufreq_dev), GFP_KERNEL);
	if (!bst_cpufreq_dev)
		return -ENOMEM;

	bst_cpufreq_dev->freq_sel_base = res->start;
	bst_cpufreq_dev->data = data;
	bst_cpufreq_dev->domain = index;
	bst_get_related_cpus(index, policy->cpus);
	if (cpumask_empty(policy->cpus)) {
		dev_err(dev, "Domain-%d failed to get related CPUs\n", index);
		kfree(bst_cpufreq_dev);
		return -ENOENT;
	}
	snprintf(cluster_name, sizeof(cluster_name), "cluster%d", index);
	bst_cpufreq_dev->clk = devm_clk_get_optional(dev, cluster_name);
	if (IS_ERR(bst_cpufreq_dev->clk)) {
		dev_err(dev, "failed to get %s clock\n", cluster_name);
		kfree(bst_cpufreq_dev);
		return -EPERM;
	}
	policy->driver_data = bst_cpufreq_dev;
	policy->dvfs_possible_from_any_cpu = true;
	policy->freq_table = data->freq_table;
	return 0;
}

static int bst_cpufreq_cpu_exit(struct cpufreq_policy *policy)
{
	return 0;
}

static unsigned int bst_cpufreq_cpu_get(unsigned int cpu)
{
	struct cpufreq_policy *policy;
	struct bst_cpufreq_dev *bst_cpufreq_dev;

	policy = cpufreq_cpu_get_raw(cpu);
	if (!policy)
		return 0;

	bst_cpufreq_dev = policy->driver_data;
	return clk_get_rate(bst_cpufreq_dev->clk) / 1000;
}

static int bst_cpufreq_cpu_target(struct cpufreq_policy *policy,
				   unsigned int index)
{
	struct bst_cpufreq_dev *bst_cpufreq_dev = policy->driver_data;

	return clk_set_rate(bst_cpufreq_dev->clk,
		    (unsigned long)policy->freq_table[index].frequency * 1000) ?
		-EINVAL : 0;
}

static struct cpufreq_driver cpufreq_bst_driver = {
	.verify		= cpufreq_generic_frequency_table_verify,
	.get		= bst_cpufreq_cpu_get,
	.target_index	= bst_cpufreq_cpu_target,
	.init		= bst_cpufreq_cpu_init,
	.exit		= bst_cpufreq_cpu_exit,
	.name		= "bst-cpufreq",
	.attr		= cpufreq_generic_attr,
};

static int bst_cpufreq_driver_probe(struct platform_device *pdev)
{
	struct device *cpu_dev;
	int ret;

	cpufreq_bst_driver.driver_data = pdev;

	/* Check for optional interconnect paths on CPU0 */
	cpu_dev = get_cpu_device(0);
	if (!cpu_dev)
		return -EPROBE_DEFER;

	ret = cpufreq_register_driver(&cpufreq_bst_driver);
	if (ret)
		dev_err(&pdev->dev, "CPUFreq driver failed to register\n");

	return ret;
}

static int bst_cpufreq_driver_remove(struct platform_device *pdev)
{
	return cpufreq_unregister_driver(&cpufreq_bst_driver);
}

static struct platform_driver bst_cpufreq_driver = {
	.probe = bst_cpufreq_driver_probe,
	.remove = bst_cpufreq_driver_remove,
	.driver = {
		.name = "bst-cpufreq",
		.of_match_table = bst_cpufreq_match,
	},
};

static int __init bst_cpufreq_init(void)
{
	return platform_driver_register(&bst_cpufreq_driver);
}
postcore_initcall(bst_cpufreq_init);

static void __exit bst_cpufreq_exit(void)
{
	platform_driver_unregister(&bst_cpufreq_driver);
}
module_exit(bst_cpufreq_exit);

MODULE_AUTHOR("Xuran Yang");
MODULE_DESCRIPTION("cpufreq driver for bst-soc");
MODULE_LICENSE("GPL");
