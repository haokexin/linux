// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *  Copyright (C) 2017-2018 Synopsys, Inc. (www.synopsys.com)
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dmaengine.h>
#include <linux/dmapool.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/of_platform.h>
#include <linux/reset.h>
#include <linux/of_irq.h>

#include "bst-lsp-timer.h"


#define LSPTMR_CONTROL_INT		(1 << 2)
#define LSPTMR_LOADCOUNT        (0x0)
#define LSPTMR_CONTROLREG       (0x8)
#define LSPTMR_COUNT_500_US		(500*200)

static void clear_lsp_timer_irq(struct bst_timer_priv *priv)
{
	u32 ctrl;
	ctrl = readl(priv->reg_base+(0x14*priv->timer_chan)+LSPTMR_CONTROLREG);
	ctrl |= LSPTMR_CONTROL_INT;
	writel(ctrl,priv->reg_base+(0x14*priv->timer_chan)+LSPTMR_CONTROLREG);
}


static void start_lsp_timer(struct bst_timer_priv *priv)
{
	writel(0x0,priv->reg_base+(0x14*priv->timer_chan)+LSPTMR_CONTROLREG);
	writel(LSPTMR_COUNT_500_US,priv->reg_base+(0x14*priv->timer_chan)+LSPTMR_LOADCOUNT);
	writel(0x3,priv->reg_base+(0x14*priv->timer_chan)+LSPTMR_CONTROLREG);
}

//static long long timer_count =0; 
static irqreturn_t timer_intr(int irq, void *data)
{
	struct bst_timer_priv *priv = (struct bst_timer_priv *)data;

//	timer_count ++;
//	if ((timer_count % 30000) == 0)
//		 pr_info("%s,in: timer_count= %lld \n", __func__, timer_count );

	clear_lsp_timer_irq(priv);

	start_lsp_timer(priv);

	return IRQ_HANDLED;	
}


static void bst_lsp_timer_shutdown(struct platform_device *pdev)
{
	struct bst_timer_priv *priv = (struct bst_timer_priv *)platform_get_drvdata(pdev);
	
	free_irq(priv->irq_num, priv);
	clear_lsp_timer_irq(priv);

//	pr_info("jun:%s \n",__func__);
}


static int bst_lsp_timer_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bst_timer_priv *priv;
	int err;

//	dev_err(dev, "timer init Start !!!\n");

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
    if (!priv) {
		return -ENOMEM;
	}

	priv->dev = dev;


	priv->reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->reg_base)) {
		dev_err(dev, "Failed to map registers\n");
		return PTR_ERR(priv->reg_base);
	}

	err = device_property_read_u32(dev, "timer-chan", &priv->timer_chan);
	if (err) {
		dev_err(dev, "%s: get timer channel failed %d\n",__func__, err);
		return err;
	}
	
	dev_set_drvdata(dev, priv);

	priv->irq_num = irq_of_parse_and_map(dev->of_node, 0);
	if (priv->irq_num < 0) {
		dev_err(dev, "get timer irq failed\n");
		return priv->irq_num;
	}	

	err = devm_request_irq(dev, priv->irq_num, timer_intr, IRQF_SHARED, "bst-lsp-timer", priv);
	if (err) {
		dev_err(dev, "request timer irq failed\n");
		return err;
	}

	start_lsp_timer(priv);

	dev_err(dev, "timer init OK !!!\n");

	return 0;

}



static int bst_lsp_timer_remove(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct bst_timer_priv *priv = dev_get_drvdata(dev);

    if (priv) {
        clear_lsp_timer_irq(priv); 
        dev_set_drvdata(dev, NULL);
    }

    return 0;
}


static const struct of_device_id bst_lsp_timer_of_match[] = {
	{ .compatible = "bst,bst-lsp-timer" },
	{}
};
MODULE_DEVICE_TABLE(of, bst_lsp_timer_of_match);



int bst_timer_system_suspend(struct device *dev)
{
	struct bst_timer_priv *priv = (struct bst_timer_priv *)dev_get_drvdata(dev);

//	dev_err(dev, "bst_timer_system_suspend start!! \n");

	disable_irq(priv->irq_num);
	clear_lsp_timer_irq(priv);

	return 0;

}
int bst_timer_system_resume(struct device *dev)
{
	struct bst_timer_priv *priv = (struct bst_timer_priv *)dev_get_drvdata(dev);

	enable_irq(priv->irq_num);
	start_lsp_timer(priv);

//	dev_err(dev, "bst_timer_system_resume done!! \n");

	return 0;

}


static const struct dev_pm_ops bst_lsp_timer_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(bst_timer_system_suspend, bst_timer_system_resume)
};

static struct platform_driver bst_lsp_timer_driver = {
	.probe		= bst_lsp_timer_probe,
	.remove		= bst_lsp_timer_remove,
	.shutdown = bst_lsp_timer_shutdown,
	.driver = {
		.name	= "bst-lsp-timer",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(bst_lsp_timer_of_match),
		.pm = &bst_lsp_timer_pm_ops,
	},
};

module_platform_driver(bst_lsp_timer_driver);

#if 0
static int __init driver_a_init(void)
{
    return platform_driver_register(&bst_lsp_timer_driver);
}
subsys_initcall(driver_a_init); 
#endif

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Black Sesame LSP TIMER platform driver");
MODULE_AUTHOR("BST Ltd.");
