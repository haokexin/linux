// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 * Copyright (C) 2007-2011  STMicroelectronics Ltd
 * Copyright (C) 2015 Joachim Eastwood <manabian@gmail.com>
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/uio_driver.h>
#include <linux/proc_fs.h>

#include "bstgmac.h"
#include "dwmac_platform.h"

#if defined(CONFIG_ARCH_BSTA1000A)
static int bstgmac_pre_init(struct plat_stmmacenet_data *plat_dat)
{
	void __iomem *top_crm = NULL;
	void __iomem *pmm_reg = NULL;
	u32 reg_value;
	u32 id = plat_dat->bus_id;

	/* select PHY */
	top_crm = ioremap(0x33000000, 0x1000);
	if (!top_crm)
		return (-EPROBE_DEFER);

	reg_value = readl(top_crm + 0x54);
	reg_value = (reg_value | (1 << (id))); //bit 0: 0(GMII/MII), 1(RGMII)
	writel(reg_value, top_crm + 0x54);
	iounmap(top_crm);

	/* TRIG_INxx and PPSx pinmux */
	pmm_reg = ioremap(0x33001004, 32);
	if (!pmm_reg)
		return (-EPROBE_DEFER);
	reg_value = readl(pmm_reg);
	if (id == 0) //gmac0: IN00:bit4 IN01:bit5  pps0:bit8
		reg_value = (reg_value & (~((1 << 4) | (1 << 8))));
	else if (id == 1) //gmac1: IN10:bit6 IN11:bit7  pps1:bit9
		reg_value = (reg_value & (~((1 << 6) | (1 << 9))));

	writel(reg_value, pmm_reg);
	iounmap(pmm_reg);

	return 0;
}
#endif
#if defined(CONFIG_ARCH_BSTA1000B) || C1200_RUN_A1000B
static int bstgmac_pre_init(struct plat_stmmacenet_data *plat_dat)
{
	void __iomem *top_crm = NULL;
	void __iomem *pmm_reg = NULL;
	u32 reg_value;
	u32 id = plat_dat->bus_id;

	/* select PHY */
	top_crm = ioremap(0x33000000, 0x4000);
	if (!top_crm)
		return (-EPROBE_DEFER);

	reg_value = readl(top_crm + 0x54);
	reg_value = (reg_value | (1 << (id))); //bit 0: 0(GMII/MII), 1(RGMII)
	writel(reg_value, top_crm + 0x54);
	iounmap(top_crm);

	/* TRIG_INxx and PPSx pinmux */
	pmm_reg = ioremap(0x33001008, 32);
	if (!pmm_reg)
		return (-EPROBE_DEFER);
	reg_value = readl(pmm_reg);
	if (id == 0)
		reg_value = (reg_value & (~((1 << 11) | (1 << 15)))); //gmac0: IN00:bit11 IN01:bit12  pps0:bit15
	else if (id == 1)
		reg_value = (reg_value & (~((1 << 13) | (1 << 16)))); //gmac1: IN10:bit13 IN11:bit14  pps1:bit16

	writel(reg_value, pmm_reg);
	iounmap(pmm_reg);
#if C1200_RUN_A1000B
	void __iomem *clk = NULL;
	clk = ioremap(0x33002000, 0x200);
	reg_value = readl(clk+0x15c);
	reg_value |= 0x1;
	writel(reg_value, clk+0x15c);	
	reg_value = readl(clk+0x154);
	reg_value |= (1 << 14);
	writel(reg_value, clk+0x154);

	reg_value = readl(clk+0x16c);
	reg_value |= ((1 << 22) | (1 << 20) | (1 << 14) | (1 << 12));
	writel(reg_value, clk+0x16c);

	reg_value = readl(clk+0x180);
	reg_value |= (1 << 3);
	writel(reg_value, clk+0x180);
	iounmap(clk);
#endif
	return 0;
}

#endif
#if defined(CONFIG_ARCH_BSTC1200) && (!C1200_RUN_A1000B)
static int bstgmac_pre_init(struct plat_stmmacenet_data *plat_dat)
{
	return 0;
}
#endif
#ifdef CONFIG_UIO
static struct uio_info *info;
static int bstgmac_register_uio(struct platform_device *pdev,
				struct bstgmac_resources *bst_res)
{
	const char *gmac_name;
	struct device_node *np = pdev->dev.of_node;
	int ret;

	ret = of_property_read_string(np, "eth-name", &gmac_name);
	if (ret < 0) {
		pr_err("can't get eth-name from dts\n");
		return ret;
	}

	info = kzalloc(sizeof(struct uio_info), GFP_KERNEL);
	if (!info)
		return -EINVAL;

	info->name = gmac_name;
	info->version = "0.0.1";
	info->irq = UIO_IRQ_NONE;
	info->mem[0].addr = bst_res->res->start;
	info->mem[0].size = resource_size(bst_res->res);
	info->mem[0].memtype = UIO_MEM_PHYS;


	ret = uio_register_device(&pdev->dev, info);
	if (ret) {
		pr_err("can't register uio for %s, ret %d\n", info->name, ret);
		kfree(info);
		bst_res->info = NULL;
		return ret;
	}

	bst_res->info = info;
	return ret;
}
#endif

static int bstgmac_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat_dat = NULL;
	struct bstgmac_resources bstgmac_res;
	int ret = 0;

	ret = bstgmac_get_platform_resources(pdev, &bstgmac_res);
	if (ret)
		return ret;

	if (pdev->dev.of_node) {
		plat_dat = bstgmac_probe_config_dt(pdev, bstgmac_res.mac);
		if (IS_ERR(plat_dat)) {
			dev_err(&pdev->dev, "dt configuration failed\n");
			return PTR_ERR(plat_dat);
		}
	} else {
		plat_dat = dev_get_platdata(&pdev->dev);
		if (!plat_dat) {
			dev_err(&pdev->dev, "no platform data provided\n");
			return  -EINVAL;
		}

		/* Set default value for multicast hash bins */
		plat_dat->multicast_filter_bins = HASH_TABLE_SIZE;

		/* Set default value for unicast filter entries */
		plat_dat->unicast_filter_entries = 1;
	}

	ret = bstgmac_pre_init(plat_dat);
	if (ret < 0) {
		dev_err(&pdev->dev, "select phy failed\n");
		goto err_remove_config_dt;
	}

	/* Custom initialisation (if needed) */
	if (plat_dat->init) {
		ret = plat_dat->init(pdev, plat_dat->bsp_priv);
		if (ret)
			goto err_remove_config_dt;
	}

#ifdef CONFIG_UIO
	ret = bstgmac_register_uio(pdev, &bstgmac_res);
	if (ret) {
		pr_err("can't register uio\n");
		return ret;
	}
#endif

	ret = bstgmac_dvr_probe(pdev, plat_dat, &bstgmac_res);
	if (ret)
		goto err_exit;

	return 0;

err_exit:
	if (plat_dat->exit)
		plat_dat->exit(pdev, plat_dat->bsp_priv);
err_remove_config_dt:
	if (pdev->dev.of_node)
		bstgmac_remove_config_dt(pdev, plat_dat);

	return ret;
}

static const struct of_device_id bst_gmac_match[] = {
	{ .compatible = "bst,dw-eqos-eth"},
	{ .compatible = "bst,sw-gmac"},
	{ .compatible = "bst,dwxgmac"},
	{ }
};
MODULE_DEVICE_TABLE(of, bst_gmac_match);

static struct platform_driver dwmac_generic_driver = {
	.probe  = bstgmac_probe,
	.remove = bstgmac_pltfr_remove,
	.shutdown = bstgmac_pltfr_shutdown,
	.driver = {
		.name           = BSTGMAC_RESOURCE_NAME,
		.pm		= &bstgmac_pltfr_pm_ops,
		.of_match_table = of_match_ptr(bst_gmac_match),
	},
};
#ifdef CONFIG_UIO
static int __init dwmac_generic_drv_init(void)
{
	return platform_driver_register(&dwmac_generic_driver);
}
static void dwmac_generic_drv_exit(void)
{
	platform_driver_unregister(&dwmac_generic_driver);
}
late_initcall(dwmac_generic_drv_init);
module_exit(dwmac_generic_drv_exit);
#else
module_platform_driver(dwmac_generic_driver);
#endif

MODULE_DESCRIPTION("Bst Gmac driver");
MODULE_LICENSE("GPL v2");
