/*
 * arch/arm/plat-spear/cpufreq.c
 *
 * CPU Frequency Scaling for SPEAr platform
 *
 * Copyright (C) 2010-2012 ST Microelectronics
 * Deepak Sikri <deepak.sikri@st.com>
 *
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <asm/system.h>
#include <plat/cpufreq.h>
#include <mach/hardware.h>
#include <mach/system.h>

struct {
	struct clk *cpu_clk;
	struct cpufreq_frequency_table *freq_tbl;
	u32 freq_tbl_len;
	unsigned int min_freq;
	unsigned int max_freq;
	unsigned int transition_latency;
} spear_cpufreq;

int spear_cpufreq_verify(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, spear_cpufreq.freq_tbl);
}

static unsigned int spear_cpufreq_get(unsigned int cpu)
{
	return cpu ? 0 : clk_get_rate(spear_cpufreq.cpu_clk) / 1000;
}

static struct clk *spear1340_cpu_get_possible_parent(unsigned long newfreq)
{
	int pclk;
	struct clk *sys_pclk;
	/*
	 * In SPEAr1340, cpu clk's parent sys clk can take input from
	 * following sources
	 */
	const char *sys_clk_src[] = {
		"sys_synth_clk",
		"pll1_clk",
		"pll2_clk",
		"pll3_clk",
	};

	/*
	 * As sys clk can have multiple source with their own range
	 * limitation so we choose possible sources accordingly
	 */
	if (newfreq <= 250000000)
		pclk = 0; /* src is sys_synth_clk */
	else if (newfreq <= 600000000)
		pclk = 3; /* src is pll3_clk */
	else
		return ERR_PTR(-EINVAL);

	/* Get parent to sys clock */
	sys_pclk = clk_get(NULL, sys_clk_src[pclk]);
	if (IS_ERR(sys_pclk))
		pr_err("SPEAr1340: Failed to get %s clock\n",
				sys_clk_src[pclk]);

	return sys_pclk;
}

static int spear1340_set_cpu_rate(struct clk *sys_pclk, unsigned long newfreq)
{
	struct clk *sys_clk;
	int ret = 0;

	sys_clk = clk_get_parent(spear_cpufreq.cpu_clk);
	if (IS_ERR(sys_clk)) {
		pr_err("failed to get cpu's parent (sys) clock\n");
		return PTR_ERR(sys_clk);
	}

	/*
	 * Set the rate of the source clock before changing the parent
	 * Note: newfreq = intended cpu_clk * 2 in case of SPEAr1340
	 */
	ret = clk_set_rate(sys_pclk, newfreq);
	if (ret) {
		pr_err("SPEAr1340: Failed to set sys clk rate to %lu\n",
				newfreq);
		return ret;
	}

	ret = clk_set_parent(sys_clk, sys_pclk);
	if (ret) {
		pr_err("SPEAr1340: Failed to set sys clk parent\n");
		return ret;
	}

	return 0;
}

static bool slow_mode_required(struct clk *clk)
{
	struct clk *sys_pclk;

	if (cpu_is_spear1340()) {
		sys_pclk = clk_get(NULL, "sys_synth_clk");
		if (IS_ERR(sys_pclk))
			WARN(1, "couldn't get system synthesizer clk");
		else
			clk_put(sys_pclk);
		/*
		 * slow mode not required if cpu is on synth.
		 * Also to be on safe side let system change to slow
		 * mode if sys_pclk has error
		 */
		return (clk == sys_pclk) ? false: true;
	} else if (arch_is_spear13xx()) {
		return true;
	} else {
		/*
		 * case of spear3xx/6xx is separatly handled as we need
		 * to put ddr into self refresh before changing pll rate
		 */
		return false;
	}
}

static int spear_cpufreq_target(struct cpufreq_policy *policy,
		unsigned int target_freq, unsigned int relation)
{
	struct cpufreq_freqs freqs;
	int index, ret, slow_mode;
	unsigned long newfreq, srcfreq;
	struct clk *srcclk;

	if (policy->cpu != 0)
		return -EINVAL;

	if (cpufreq_frequency_table_target(policy, spear_cpufreq.freq_tbl,
				target_freq, relation, &index))
		return -EINVAL;

	freqs.old = spear_cpufreq_get(0);
	freqs.cpu = policy->cpu;

	if (freqs.old == target_freq)
		return 0;

	newfreq = spear_cpufreq.freq_tbl[index].frequency * 1000;
	if (cpu_is_spear1340()) {
		/*
		 * SPEAr1340 is special in the sense that due to the
		 * possibility of multiple clock sources for cpu clk's
		 * parent we can have different clock source for
		 * different frequency of cpu clk.
		 * Hence we need to choose one from amongst these
		 * possible clock sources.
		 */
		srcclk = spear1340_cpu_get_possible_parent(newfreq);
		if (IS_ERR(srcclk)) {
			pr_err("Failed to get src clk\n");
			return PTR_ERR(srcclk);
		}

		/* SPEAr1340: src clk is always 2 * intended cpu clk */
		srcfreq = newfreq * 2;
	} else {
		/*
		 * Rest: src clock to be altered is ancestor of cpu
		 * clock. Hence we can directly work on cpu clk
		 */
		srcclk = spear_cpufreq.cpu_clk;
		srcfreq = newfreq;
	}

	/*
	 * In SPEAr1340, we cannot use newfreq directly because we need
	 * to actually access a source clock (clk) which might not be
	 * ancestor of cpu at present.
	 * Hence in SPEAr1340 we would operate on source clock directly
	 * before switching cpu clock to it.
	 */
	srcfreq = clk_round_rate(srcclk, srcfreq);
	if (srcfreq < 0) {
		pr_err("CPU Freq: clk_round_rate failed for cpu src clock\n");
		return srcfreq;
	}

	freqs.new = srcfreq / 1000;
	freqs.new /= cpu_is_spear1340() ? 2 : 1;
	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	slow_mode = slow_mode_required(srcclk);
	if (slow_mode) {
		ret = arch_change_mode(SYS_MODE_SLOW);
		if (ret) {
			pr_err("couldn't cange system to slow mode\n");
			return ret;
		}
	}

	if (cpu_is_spear1340())
		ret = spear1340_set_cpu_rate(srcclk, srcfreq);
	else
		ret = clk_set_rate(spear_cpufreq.cpu_clk, srcfreq);

	/* Get current rate after clk_set_rate, in case of failure */
	if (ret) {
		pr_err("CPU Freq: cpu clk_set_rate failed: %d\n", ret);
		freqs.new = clk_get_rate(spear_cpufreq.cpu_clk) / 1000;
	}

	/* Now switch back to normal mode */
	if (slow_mode) {
		ret = arch_change_mode(SYS_MODE_NORMAL);
		if (ret) {
			pr_err("Couldnot change back to normal mode\n");
			BUG();
		}
	}

	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	return ret;
}

#ifdef CONFIG_PM
static int spear_cpufreq_suspend(struct cpufreq_policy *policy)
{
	return 0;
}

static int spear_cpufreq_resume(struct cpufreq_policy *policy)
{
	return 0;
}
#endif

static int spear_cpufreq_init(struct cpufreq_policy *policy)
{
	if (policy->cpu != 0)
		return -EINVAL;

	policy->cpuinfo.min_freq = spear_cpufreq.min_freq;
	policy->cpuinfo.max_freq = spear_cpufreq.max_freq;

	policy->cur = policy->min = policy->max = spear_cpufreq_get(0);

	if (!cpufreq_frequency_table_cpuinfo(policy, spear_cpufreq.freq_tbl))
		cpufreq_frequency_table_get_attr(spear_cpufreq.freq_tbl,
				policy->cpu);

	policy->cpuinfo.transition_latency = spear_cpufreq.transition_latency;

	return 0;
}

static struct freq_attr *spear_cpufreq_attr[] = {
	 &cpufreq_freq_attr_scaling_available_freqs,
	 NULL,
};

static struct cpufreq_driver spear_cpufreq_driver = {
	.flags		= CPUFREQ_STICKY,
	.verify		= spear_cpufreq_verify,
	.target		= spear_cpufreq_target,
	.get		= spear_cpufreq_get,
	.init		= spear_cpufreq_init,
	.name		= "spear_cpufreq",
	.attr		= spear_cpufreq_attr,
#ifdef CONFIG_PM
	.suspend	= spear_cpufreq_suspend,
	.resume		= spear_cpufreq_resume,
#endif
};

static int __init spear_cpufreq_probe(struct platform_device *pdev)
{
	struct spear_cpufreq_pdata *pdata = dev_get_platdata(&pdev->dev);
	int i;

	if (!pdata || !pdata->cpu_freq_table)
		return -EINVAL;

	spear_cpufreq.freq_tbl = kmalloc(sizeof(*spear_cpufreq.freq_tbl) *
			pdata->tbl_len, GFP_KERNEL);
	if (!spear_cpufreq.freq_tbl) {
		dev_err(&pdev->dev, "kzalloc fail\n");
		return -ENOMEM;
	}

	for (i = 0; i < pdata->tbl_len; i++) {
		spear_cpufreq.freq_tbl[i].index = i;
		spear_cpufreq.freq_tbl[i].frequency = pdata->cpu_freq_table[i];
	}

	spear_cpufreq.min_freq = spear_cpufreq.freq_tbl[0].frequency;
	spear_cpufreq.max_freq = spear_cpufreq.freq_tbl[i-1].frequency;
	spear_cpufreq.freq_tbl[i].index = i;
	spear_cpufreq.freq_tbl[i].frequency = CPUFREQ_TABLE_END;
	spear_cpufreq.transition_latency = pdata->transition_latency;
	if (pdata->transition_latency)
		spear_cpufreq.transition_latency = pdata->transition_latency;
	else
		spear_cpufreq.transition_latency = 300 * 1000; /*300 us*/

	spear_cpufreq.cpu_clk = clk_get(NULL, "cpu_clk");
	if (IS_ERR(spear_cpufreq.cpu_clk)) {
		dev_err(&pdev->dev, "Unable to get CPU clock\n");
		return PTR_ERR(spear_cpufreq.cpu_clk);
	}

	return cpufreq_register_driver(&spear_cpufreq_driver);
}

static int __exit spear_cpufreq_remove(struct platform_device *pdev)
{
	clk_put(spear_cpufreq.cpu_clk);

	return cpufreq_unregister_driver(&spear_cpufreq_driver);
}

static struct platform_driver spear_cpufreq_pdrv = {
	.probe = spear_cpufreq_probe,
	.remove = __exit_p(spear_cpufreq_remove),
	.driver = {
		.name	 = "cpufreq-spear",
		.owner	 = THIS_MODULE,
	},
};

static int __init spear_cpufreq_register(void)
{
	return platform_driver_register(&spear_cpufreq_pdrv);
}
arch_initcall(spear_cpufreq_register);
