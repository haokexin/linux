// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 INTEL Corporation
/*
 * CCN cache coherent interconnect driver
 *
 * Author: Marek Bykowski <marek.bykowski@intel.com>
 */

#include <linux/arm-ccn.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/of_irq.h>
#include <linux/arm-smccc.h>

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
#define CCN_MN_OLY_COMP_LIST_63_0	0x01e0
#define CCN_MN_ERR_SIG_VAL_63_0		0x0300
#define CCN_MN_ERR_SIG_VAL_63_0__DT			BIT(1)
#define CCN_MN_ERR_SIG_VAL_63_0__HNF			0xff

LIST_HEAD(arm_ccn_head);

struct arm_ccn {
	struct device *dev;
	void __iomem *base;
	unsigned int irq;
};

void arm_ccn_handler_add(struct list_head *new)
{
	list_add(new, &arm_ccn_head);
};

struct error_reporting *arm_ccn_handler_find(char *name)
{
	struct error_reporting *er;

	list_for_each_entry(er, &arm_ccn_head, child) {
		if (strcmp(er->match, name) == 0)
			return er;
	}
	return NULL;
}

/* Read/write secure Error Interrupt Status register */
#define SECURE_CCN_MN_READ_ERRINT		0xc4000026
#define SECURE_CCN_MN_WRITE_ERRINT		0xc4000027

unsigned long (*errint_read)(const void __iomem *reg);
void (*errint_write)(u32 val, void __iomem *addr);

static unsigned long secure_errint_read(const void __iomem *reg)
{
	struct arm_smccc_res res = {0};

	arm_smccc_smc(SECURE_CCN_MN_READ_ERRINT,
		      0, 0, 0, 0, 0, 0, 0, &res);
	return res.a0;
}

static unsigned long nonsecure_errint_read(const void __iomem *reg)
{
	return readl(reg);
}

static void secure_errint_write(u32 val, void __iomem *addr)
{
	struct arm_smccc_res res = {0};

	arm_smccc_smc(SECURE_CCN_MN_WRITE_ERRINT,
		      val, 0, 0, 0, 0, 0, 0, &res);
}

static void nonsecure_errint_write(u32 val, void __iomem *addr)
{
	writel(val, addr);
}

static unsigned int ccn_ctrl_irq __ro_after_init;

/* Pass on extra data to the child/ren populated */
static const struct of_dev_auxdata arm_ccn_auxdata[] = {
	/* One of the below can be present at the time */
	OF_DEV_AUXDATA("arm,ccn-504-l3", 0, NULL, &ccn_ctrl_irq),
	OF_DEV_AUXDATA("arm,ccn-512-l3", 0, NULL, &ccn_ctrl_irq),
	/* Also one can be present at the time */
	OF_DEV_AUXDATA("arm,ccn-502-pmu", 0, NULL, &ccn_ctrl_irq),
	OF_DEV_AUXDATA("arm,ccn-504-pmu", 0, NULL, &ccn_ctrl_irq),
	OF_DEV_AUXDATA("arm,ccn-512-pmu", 0, NULL, &ccn_ctrl_irq),
	{},
};

/* Compatible string of the parent node */
static const struct of_device_id arm_ccn_matches[] = {
	{ .compatible = "arm,ccn-502", },
	{ .compatible = "arm,ccn-504", },
	{ .compatible = "arm,ccn-512", },
	{},
};
MODULE_DEVICE_TABLE(of, arm_ccn_matches);

static irqreturn_t arm_ccn_irq_handler(int irq, void *dev_id)
{
	struct arm_ccn *ccn = dev_id;
	u32 err_sig_val[6], err_or;
	irqreturn_t res = IRQ_NONE;
	struct error_reporting *er;
	int i;

	err_sig_val[0] = readl(ccn->base + CCN_MN_ERR_SIG_VAL_63_0);
	err_or = err_sig_val[0];
	if (err_or & CCN_MN_ERR_SIG_VAL_63_0__DT) {
		err_or &= ~CCN_MN_ERR_SIG_VAL_63_0__DT;
		list_for_each_entry(er, &arm_ccn_head, child)
			if (strstr(er->match, "pmu"))
				res = er->handler(er->data);
	}

	/* To my best knowledge for having the interrupt serviced at minimum one
	 * has to clear an MN error signal by reading Error Signal Valid
	 * regs and deassert the interrupt (INTREQ).
	 */
	for (i = 2; i < ARRAY_SIZE(err_sig_val); i++) {
		err_sig_val[i] = readl(ccn->base +
			CCN_MN_ERR_SIG_VAL_63_0 + i * 4);
		err_or |= err_sig_val[i];
		if (err_or)
			res |= IRQ_HANDLED;
	}

	/* Call err_sig_val_hnf handler */
	err_sig_val[1] = readl(ccn->base + CCN_MN_ERR_SIG_VAL_63_0 + 4);
	if (err_sig_val[1] & CCN_MN_ERR_SIG_VAL_63_0__HNF) {
		list_for_each_entry(er, &arm_ccn_head, child)
			if (strstr(er->match, "l3"))
				res |= er->handler(er->data);
	}

	dev_dbg(ccn->dev, "err_sig_val[191-0]: %08x%08x%08x%08x%08x%08x\n",
		err_sig_val[5], err_sig_val[4], err_sig_val[3],
		err_sig_val[2], err_sig_val[1], err_sig_val[0]);

	if (res != IRQ_NONE)
		errint_write(CCN_MN_ERRINT_STATUS__INTREQ__DESSERT,
			     ccn->base + CCN_MN_ERRINT_STATUS);

	return res;
}

static int ccn_platform_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *res;
	unsigned int irq;
	struct arm_ccn *ccn;

	ccn = devm_kzalloc(&pdev->dev, sizeof(*ccn), GFP_KERNEL);
	if (!ccn)
		return -ENOMEM;

	ccn->dev = &pdev->dev;
	platform_set_drvdata(pdev, ccn);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ccn->base = devm_ioremap(ccn->dev, res->start, resource_size(res));
	if (IS_ERR(ccn->base))
		return PTR_ERR(ccn->base);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res)
		return -EINVAL;

	irq = res->start;

	if (platform_has_secure_ccn_access()) {
		errint_read = nonsecure_errint_read;
		errint_write = nonsecure_errint_write;
	} else {
		errint_read = secure_errint_read;
		errint_write = secure_errint_write;
	}

	/* Check if we can use the interrupt */
	errint_write(CCN_MN_ERRINT_STATUS__PMU_EVENTS__DISABLE,
		     ccn->base + CCN_MN_ERRINT_STATUS);

	if (errint_read(ccn->base + CCN_MN_ERRINT_STATUS) &
			CCN_MN_ERRINT_STATUS__PMU_EVENTS__DISABLED) {
		/* Can acknowledge the interrupt so can use it */
		errint_write(CCN_MN_ERRINT_STATUS__PMU_EVENTS__ENABLE,
			     ccn->base + CCN_MN_ERRINT_STATUS);

		ret = request_irq(irq, arm_ccn_irq_handler,
				  IRQF_NOBALANCING | IRQF_NO_THREAD,
				  dev_name(ccn->dev), ccn);
		if (ret) {
			dev_err(ccn->dev, "Failed to register interrupt\n");
			return ret;
		}

		 ccn->irq = irq;
		 ccn_ctrl_irq = ccn->irq;
	}

	dev_dbg(ccn->dev,
		"of_platform_populate(): populate irq %u\n", ccn_ctrl_irq);
	ret = of_platform_populate(pdev->dev.of_node, NULL,
				   arm_ccn_auxdata, &pdev->dev);

	if (ret) {
		dev_err(&pdev->dev,
			"failed to populate bus devices\n");
		return ret;
	}

#define TEST_ERROR_FLOW 0
		if (TEST_ERROR_FLOW) {
			#define SRID 11
			#define LPID 0
			#define EN 1
			u32 val = SRID << 16 | LPID << 4 | EN;

			dev_dbg(ccn->dev, "injecting HNF error %08x@%p\n",
				val,
				ccn->base + 0x200000 + 0x038);
			writel(val, ccn->base + 0x200000 + 0x038);
		}

	dev_info(ccn->dev, "ARM CCN driver probed\n");

	return 0;
}

static struct platform_driver ccn_platform_driver = {
	.driver = {
		   .name = "arm-ccn",
		   .of_match_table = arm_ccn_matches,
		   .owner = THIS_MODULE,
		  },
	.probe = ccn_platform_probe,
};

static int __init ccn_platform_init(void)
{
	return platform_driver_register(&ccn_platform_driver);
}

core_initcall(ccn_platform_init);
MODULE_AUTHOR("Marek Bykowski <marek.bykowski@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ARM CCN support");
