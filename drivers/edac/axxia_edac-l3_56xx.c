// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 INTEL Corporation
/*
 * EDAC Driver for Intel's Axxia 5600 L3 (DICKENS)
 * EDAC Driver for Intel's Axxia 6700 L3 (SHELLEY)
 */

#define CREATE_TRACE_POINTS

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/axxia-ncr.h>
#include <linux/edac.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/arm-smccc.h>
#include <trace/events/edacl3.h>
#include <linux/arm-ccn.h>
#include "edac_module.h"
#include "axxia_edac.h"

#if defined(CONFIG_EDAC_AXXIA_L3_5600)
#define INTEL_EDAC_MOD_STR     "axxia56xx_edac"
#define CCN_XP_NODES			11
#define	CCN_HNI_NODES			1
#endif

#if defined(CONFIG_EDAC_AXXIA_L3_6700)
#define INTEL_EDAC_MOD_STR     "axxia67xx_edac"
#define CCN_XP_NODES			18
#define	CCN_HNI_NODES			2
#endif

#define SYSCON_PERSIST_SCRATCH 0xdc
#define L3_PERSIST_SCRATCH_BIT (0x1 << 4)

#define CCN_DT_PMOVSR			0x0198
#define CCN_DT_PMOVSR_CLR		0x01a0

#define CCN_MN_ERRINT_STATUS		0x0008
#define CCN_MN_ERRINT_STATUS__INTREQ__DESSERT		0x11
#define CCN_MN_ERRINT_STATUS__ALL_ERRORS__ENABLE	0x02
#define CCN_MN_ERRINT_STATUS__ALL_ERRORS__DISABLED	0x20
#define CCN_MN_ERRINT_STATUS__ALL_ERRORS__DISABLE	0x22
#define CCN_MN_ERRINT_STATUS__CORRECTED_ERRORS_ENABLE	0x04
#define CCN_MN_ERRINT_STATUS__CORRECTED_ERRORS_DISABLED	0x40
#define CCN_MN_ERRINT_STATUS__CORRECTED_ERRORS_DISABLE	0x44
#define CCN_MN_ERRINT_STATUS__PMU_EVENTS__ENABLE	0x08
#define CCN_MN_ERRINT_STATUS__PMU_EVENTS__DISABLED	0x80
#define CCN_MN_ERRINT_STATUS__PMU_EVENTS__DISABLE	0x88

#define CCN_MN_ERR_SIG_VAL_63_0		0x0300
#define CCN_MN_ERR_SIG_VAL_63_0__DT			BIT(1)
#define CCN_MN_ERROR_TYPE_VALUE		0x0320

#define CCN_REGION_SIZE	0x10000

#define CCN_HNI_NODE_BIT		8
#define CCN_HNF_NODES			8
#define CCN_DT_NODE_BASE_ADDR		(1 * CCN_REGION_SIZE)
#define CCN_HNI_NODE_BASE_ADDR(i)	(0x80000 + (i) * CCN_REGION_SIZE)
#define CCN_HNF_NODE_BASE_ADDR(i)	(0x200000 + (i) * CCN_REGION_SIZE)
#define CCN_XP_NODE_BASE_ADDR(i)	(0x400000 + (i) * CCN_REGION_SIZE)

#define CCN_NODE_ERR_SYND_REG0		0x400
#define CCN_NODE_ERR_SYND_REG1		0x408
#define CCN_NODE_ERR_SYND_CLR		0x480

static int l3_pmode;

union dickens_hnf_err_syndrome_reg0 {
	struct __packed {
	#ifdef CPU_BIG_ENDIAN
		unsigned long long err_extnd			: 1;
		unsigned long long first_err_vld		: 1;
		unsigned long long err_class			: 2;
		unsigned long long mult_err			: 1;
		unsigned long long err_count			: 16;
		unsigned long long reserved_err_syndrome_reg0_20: 23;
		unsigned long long err_count_set		: 12;
		unsigned long long err_count_ovrflw		: 1;
		unsigned long long err_count_match		: 1;
		unsigned long long err_count_type		: 2;
		unsigned long long par_err_id			: 1;
		unsigned long long err_id			: 3;
	#else
		unsigned long long err_id			: 3;
		unsigned long long par_err_id			: 1;
		unsigned long long err_count_type		: 2;
		unsigned long long err_count_match		: 1;
		unsigned long long err_count_ovrflw		: 1;
		unsigned long long err_count_set		: 12;
		unsigned long long reserved_err_syndrome_reg0_20: 23;
		unsigned long long err_count			: 16;
		unsigned long long mult_err			: 1;
		unsigned long long err_class			: 2;
		unsigned long long first_err_vld		: 1;
		unsigned long long err_extnd			: 1;
	#endif
	} reg0;
	u64 value;
};

union dickens_hnf_err_syndrome_reg1 {
	struct __packed {
	#ifdef CPU_BIG_ENDIAN
		unsigned long long reserved_err_syndrome_reg1_55: 9;
		unsigned long long err_srcid			: 7;
		unsigned long long reserved_err_syndrome_reg1_46: 2;
		unsigned long long err_optype			: 2;
		unsigned long long err_addr			: 44;
	#else
		unsigned long long err_addr			: 44;
		unsigned long long err_optype			: 2;
		unsigned long long reserved_err_syndrome_reg1_46: 2;
		unsigned long long err_srcid			: 7;
		unsigned long long reserved_err_syndrome_reg1_55: 9;
	#endif
	} reg1;
	u64 value;
};

union dickens_hnf_err_syndrome_clr {
	struct __packed {
	#ifdef CPU_BIG_ENDIAN
		unsigned long long reserved_err_syndrome_clr_63	: 1;
		unsigned long long first_err_vld_clr		: 1;
		unsigned long long reserved_err_syndrome_clr_60	: 2;
		unsigned long long mult_err_clr			: 1;
		unsigned long long reserved_err_syndrome_clr_0	: 59;
	#else
		unsigned long long reserved_err_syndrome_clr_0	: 59;
		unsigned long long mult_err_clr			: 1;
		unsigned long long reserved_err_syndrome_clr_60	: 2;
		unsigned long long first_err_vld_clr		: 1;
		unsigned long long reserved_err_syndrome_clr_63	: 1;
	#endif
	} clr;
	u64 value;
};

struct event_data {
	u64 err_synd_reg0;
	u64 err_synd_reg1;
	int idx;
};

/* Private structure for common edac device */
struct intel_edac_dev_info {
	struct device *dev;
	struct platform_device *pdev;
	char *ctl_name;
	char *blk_name;
	int edac_idx;
	int irq_used;
	struct event_data data[CCN_HNF_NODES];
	struct event_data data_xp[CCN_XP_NODES];
	struct event_data data_hni[CCN_HNI_NODES];
	struct regmap *syscon;
	void __iomem *dickens_L3;
	struct edac_device_ctl_info *edac_dev;
	void (*check)(struct edac_device_ctl_info *edac_dev);
		struct error_reporting *er;
	};

static int __init l3_polling_mode(char *str)
{
	int polling_mode;

	if (get_option(&str, &polling_mode)) {
		l3_pmode = polling_mode;
		return 0;
	}
	return -EINVAL;
}

early_param("l3_polling_mode", l3_polling_mode);

static void clear_node_error(void __iomem *addr)
{
	union dickens_hnf_err_syndrome_clr err_syndrome_clr;

	err_syndrome_clr.value = 0x0;
	err_syndrome_clr.clr.first_err_vld_clr = 0x1;
	err_syndrome_clr.clr.mult_err_clr = 0x1;
	writeq(err_syndrome_clr.value, addr);
}

#define ERR_SYND_FIRST_ERR_VLD	BIT_ULL(62)
#define ERR_SYND_ERR_CLASS	(0x2ULL << 60)
#define ERR_SYND_ERR_COUNT_SET	(0xffff << 8)

/* Dispatch handler for hnf errors */
irqreturn_t arm_ccn_error_handler(void *ptr)
{
	struct intel_edac_dev_info *dev_info =
		(struct intel_edac_dev_info *)ptr;
	int instance;
	unsigned long count = 0;
	u64 err_syndrome_reg0, err_syndrome_reg1;
	void __iomem *addr = dev_info->dickens_L3 + CCN_HNF_NODE_BASE_ADDR(0);

	for (instance = 0;
		instance < CCN_HNF_NODES;
		instance++, addr += CCN_REGION_SIZE) {
		err_syndrome_reg0 = readq(addr + CCN_NODE_ERR_SYND_REG0);
		err_syndrome_reg1 = readq(addr + CCN_NODE_ERR_SYND_REG1);

		dev_dbg(dev_info->dev, "err_syndrome_reg0 %16llx err_syndrome_reg1 %16llx\n",
			err_syndrome_reg0, err_syndrome_reg1);

		/* First error valid */
		if (err_syndrome_reg0 & ERR_SYND_FIRST_ERR_VLD) {
			if (((err_syndrome_reg0 & ERR_SYND_ERR_CLASS) >> 60)
					== 0x3) {
				regmap_update_bits(dev_info->syscon,
						   SYSCON_PERSIST_SCRATCH,
						   L3_PERSIST_SCRATCH_BIT,
						   L3_PERSIST_SCRATCH_BIT);
				/* Fatal error */
				pr_emerg("L3 uncorrectable error\n");
				machine_restart(NULL);
			}
			count = (err_syndrome_reg0 & ERR_SYND_ERR_COUNT_SET)
						>> 8;
			if (count == 0)
				continue;

			edac_device_handle_multi_ce
				(dev_info->edac_dev, 0,
				 dev_info->data[instance].idx,
				 (int)count,
				 dev_info->edac_dev->ctl_name);

			writeq(~0x0, addr + CCN_NODE_ERR_SYND_CLR);
		}
	}

	return IRQ_HANDLED;
}

/* Check for L3 Errors */
static void intel_l3_error_check(struct edac_device_ctl_info *edac_dev)
{
	void __iomem *addr;
	union dickens_hnf_err_syndrome_reg0 err_syndrome_reg0;
	union dickens_hnf_err_syndrome_clr err_syndrome_clr;
	unsigned int count = 0;
	int instance;
	struct intel_edac_dev_info *dev_info;

	err_syndrome_clr.value = 0x0;
	err_syndrome_clr.clr.first_err_vld_clr = 0x1;
	err_syndrome_clr.clr.mult_err_clr = 0x1;

	dev_info = (struct intel_edac_dev_info *)edac_dev->pvt_info;
	addr = dev_info->dickens_L3 + CCN_HNF_NODE_BASE_ADDR(0);

	for (instance = 0;
		instance < CCN_HNF_NODES;
		instance++, addr += CCN_REGION_SIZE) {
		err_syndrome_reg0.value =
			readq(addr + CCN_NODE_ERR_SYND_REG0);

		trace_edacl3_syndromes(err_syndrome_reg0.value,
				       (u64)0);
		/* First error valid */
		if (err_syndrome_reg0.reg0.first_err_vld) {
			if (err_syndrome_reg0.reg0.err_class & 0x3) {
				regmap_update_bits(dev_info->syscon,
						   SYSCON_PERSIST_SCRATCH,
						   L3_PERSIST_SCRATCH_BIT,
						   L3_PERSIST_SCRATCH_BIT);
				/* Fatal error */
				pr_emerg("L3 uncorrectable error\n");
				machine_restart(NULL);
			}
			count = err_syndrome_reg0.reg0.err_count;
			if (count)
				edac_device_handle_multi_ce(edac_dev, 0,
							    dev_info->data[instance].idx,
							    count,
							    edac_dev->ctl_name);

			/* clear the valid bit */
			clear_node_error(addr + CCN_NODE_ERR_SYND_CLR);
		}
	}
}

static const struct of_device_id intel_edac_l3_match[] = {
	{ .compatible = "arm,ccn-504-l3", },
	{ .compatible = "arm,ccn-512-l3", },
	{},
};

static int intel_edac_l3_probe(struct platform_device *pdev)
{
	struct intel_edac_dev_info *dev_info = NULL;
	struct device_node *np = pdev->dev.of_node;
	struct resource *r;

	dev_info = devm_kzalloc(&pdev->dev, sizeof(*dev_info), GFP_KERNEL);
	if (!dev_info)
		return -ENOMEM;

	dev_info->dev = &pdev->dev;
	platform_set_drvdata(pdev, dev_info);

	dev_info->ctl_name = kstrdup(np->name, GFP_KERNEL);
	dev_info->blk_name = "l3merrsr";
	dev_info->pdev = pdev;
	dev_info->edac_idx = edac_device_alloc_index();

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r)
		return -ENOMEM;

	dev_info->dickens_L3 = devm_ioremap(&pdev->dev, r->start,
					    resource_size(r));
	if (IS_ERR(dev_info->dickens_L3))
		return PTR_ERR(dev_info->dickens_L3);

	dev_info->syscon =
		syscon_regmap_lookup_by_phandle(np, "syscon");
	if (IS_ERR(dev_info->syscon)) {
		dev_err(dev_info->dev, "%s: syscon lookup failed\n",
			np->name);
		return PTR_ERR(dev_info->syscon);
	}

	/* for the moment only HNF errors are reported via sysfs */
	dev_info->edac_dev =
		edac_device_alloc_ctl_info(0, dev_info->ctl_name,
					   1, dev_info->blk_name,
					   CCN_HNF_NODES, 0, NULL, 0,
					   dev_info->edac_idx);
	if (!dev_info->edac_dev)
		return -ENOMEM;

	dev_info->edac_dev->log_ce = 0;

	/* Irq number populated from parent */
	dev_info->irq_used = *(unsigned int *)(&pdev->dev)->platform_data;

	if (l3_pmode) {
		dev_dbg(dev_info->dev, "'poll-mode' selected\n");
		dev_info->irq_used = 0;
	}

	dev_info->edac_dev->pvt_info = dev_info;
	dev_info->edac_dev->dev = &dev_info->pdev->dev;
	dev_info->edac_dev->ctl_name = dev_info->ctl_name;
	dev_info->edac_dev->mod_name = INTEL_EDAC_MOD_STR;
	dev_info->edac_dev->dev_name = dev_name(&dev_info->pdev->dev);

	if (dev_info->irq_used) {
		edac_op_state = EDAC_OPSTATE_INT;
		dev_info->edac_dev->edac_check = NULL;
		pr_info("L3 cache EDAC - interrupt mode.\n");
	} else {
		edac_op_state = EDAC_OPSTATE_POLL;
		dev_info->edac_dev->edac_check = intel_l3_error_check;
		pr_info("L3 cache EDAC - polling mode.\n");
	}

	if (edac_device_add_device(dev_info->edac_dev) != 0) {
		pr_info("Unable to add edac device for %s\n",
			dev_info->ctl_name);
		goto err;
	}

	if (dev_info->irq_used) {
		struct error_reporting *er;

		dev_info->er = devm_kzalloc(&pdev->dev,
					    sizeof(*dev_info->er),
					    GFP_KERNEL);
		if (!dev_info->er)
			return -ENOMEM;

		er = dev_info->er;

		er->data = (void *)dev_info;
		er->handler = arm_ccn_error_handler;
		er->match = of_match_node(intel_edac_l3_match,
					  dev_info->dev->of_node)->compatible;
		dev_dbg(dev_info->dev, "register handler '%s'\n", er->match);
		arm_ccn_handler_add(&er->child);

#define TEST_ERROR_FLOW 0
		if (TEST_ERROR_FLOW) {
			#define SRID 32
			#define LPID 0
			#define EN 1
			u32 val = SRID << 16 | LPID << 4 | EN;

			dev_dbg(dev_info->dev, "injecting HNF error %08x @ %p\n",
				val,
				dev_info->dickens_L3
				+ CCN_HNF_NODE_BASE_ADDR(0) + 0x038);
			writel(val, dev_info->dickens_L3
				+ CCN_HNF_NODE_BASE_ADDR(0) + 0x038);
		}
	}

	return 0;

err:
	edac_device_free_ctl_info(dev_info->edac_dev);
	return -ENOMEM;
}

static int intel_edac_l3_remove(struct platform_device *pdev)
{
	struct intel_edac_dev_info *dev_info = platform_get_drvdata(pdev);

	edac_device_free_ctl_info(dev_info->edac_dev);
	return 0;
}


static struct platform_driver intel_edac_l3_driver = {
	.probe = intel_edac_l3_probe,
	.remove = intel_edac_l3_remove,
	.driver = {
		.name = "intel_edac_l3",
		.of_match_table = intel_edac_l3_match,
	}
};

module_platform_driver(intel_edac_l3_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marek Majtyka <marekx.majtyka@intel.com>");
