/*
 * fp_platform_driver.c
 *
 * SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (C)2024Black Sesame Technologies. All Rights Reserved.
 */
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/of_net.h>
#include <linux/etherdevice.h>
#include <linux/rtnetlink.h>
#include "fp_net_driver.h"

int hif_get_platform_resources(struct platform_device *pdev,
				   struct hif_resources *hif_res)
{
	struct resource *res;
	int i;
	char irq_name[128] = { 0 };

	memset(hif_res, 0, sizeof(*hif_res));

	for (i = 0; i < HIF_MAX_IRQ; i++) {
		snprintf(irq_name, sizeof(irq_name), "hif_irq%d", i);

		hif_res->irq[i] = platform_get_irq_byname(pdev, irq_name);
		if (hif_res->irq[i] < 0) {
			if (hif_res->irq[i] != -EPROBE_DEFER) {
				dev_err(&pdev->dev,
						"HIF IRQ configuration information not found\n");
			}
			return hif_res->irq[i];
		}
//	pr_err("hif_re->irq[i] = %d, i = %d.\n", hif_res->irq[i],i); 
	}


	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		pr_err("get io resource failed.\n"); 
	}

//	pr_err("get io resource start address = %px.\n", res->start); 
//	pr_err("get io resource end address = %px.\n", res->end); 
	hif_res->addr = devm_ioremap_resource(&pdev->dev, res);

//	pr_err("get hif_res->addr = %px.\n", hif_res->addr); 

	return PTR_ERR_OR_ZERO(hif_res->addr);
}

struct plat_hif_data *hif_probe_config_dt(struct platform_device
						     *pdev, u8 *mac)
{
	struct device_node *np = pdev->dev.of_node;
	struct plat_hif_data *plat;
	int rc;

	rc = of_get_mac_address(np, mac);
	if (rc) {
		if (rc == -EPROBE_DEFER)
			return ERR_PTR(rc);

		eth_zero_addr(mac);
	}
	plat = devm_kzalloc(&pdev->dev, sizeof(*plat), GFP_KERNEL);
	if (!plat)
		return ERR_PTR(-ENOMEM);
	of_property_read_u64(np, "channel-bit", &plat->hif_channel_bit);
//	pr_err("hif_channel_bit = %llx.\n", plat->hif_channel_bit); 
	of_property_read_u32(np, "max-frame-size", &plat->hif_max_mtu);
//	pr_err("max-frame-size = %px.\n", plat->hif_max_mtu); 

	return plat;
}

/**
 * hif_pltfr_remove
 * @pdev: platform device pointer
 * Description: this function calls the main to free the net resources
 * and calls the platforms hook and release the resources (e.g. mem).
 */
int hif_pltfr_remove(struct platform_device *pdev)
{
	int ret = hif_dvr_remove(&pdev->dev);

	platform_set_drvdata(pdev, NULL);

	return ret;
}

void hif_pltfr_shutdown(struct platform_device *pdev)
{
	int ret = hif_dvr_remove(&pdev->dev);
	if (ret) {
		pr_err("%s: hif driver remove fail\n", __func__);
	}

	platform_set_drvdata(pdev, NULL);
}

static int hif_probe(struct platform_device *pdev)
{
	struct hif_resources hif_res;
	struct plat_hif_data *plat_dat;
	struct net_device *dev;
	int ret;

	/*-----------------------------------------------------------------------------
	 *  in this function we should use ioremap get va.
	 *-----------------------------------------------------------------------------*/
	ret = hif_get_platform_resources(pdev, &hif_res);
	if (ret)
		return ret;

	if (pdev->dev.of_node) {
		plat_dat = hif_probe_config_dt(pdev, hif_res.mac);
//		pr_err("hif_res.mac = %02x:%02x:%02x:%02x:%02x:%02x.\n", hif_res.mac[0],hif_res.mac[1],hif_res.mac[2],
//				hif_res.mac[3],hif_res.mac[4],hif_res.mac[5]);
		if (IS_ERR(plat_dat)) {
			dev_err(&pdev->dev, "dt configuration failed\n");
			return PTR_ERR(plat_dat);
		} else {
			printk("%s dt config success\n", __func__);
		}
	}

	dev = fp_netdev_init(pdev, plat_dat, &hif_res);
	if (dev != NULL) {
		//platform_set_drvdata(pdev, dev);
		//SET_NETDEV_DEV(dev, &pdev->dev);
		return 0;
	}	
	else {
		ret = -1;
	}

	return ret;
}

static const struct of_device_id bst_hif_match[] = {
	{ .compatible = "bst,vmac-hif"},
	{ }
};
MODULE_DEVICE_TABLE(of, bst_hif_match);

static struct platform_driver hif_generic_driver = {
	.probe  = hif_probe,
	.remove = hif_pltfr_remove,
	.shutdown = hif_pltfr_shutdown,
	.driver = {
		.name           = HIF_RESOURCE_NAME,
		.pm		= &hif_pltfr_pm_ops,
		.of_match_table = of_match_ptr(bst_hif_match),
	},
};
module_platform_driver(hif_generic_driver);

#ifdef CONFIG_PM_SLEEP
/**
 * hif_pltfr_suspend
 * @dev: device pointer
 * Description: this function is invoked when suspend the driver and it directly
 * call the main suspend function and then, if required, on some platform, it
 * can call an exit helper.
 */
static int hif_pltfr_suspend(struct device *dev)
{
	int ret;
	struct ifreq ifr;
	struct net_device *ndev = dev_get_drvdata(dev);
	struct fp_private *fp = netdev_priv(ndev);

	set_bit(HIFVMAC_NOE_EXEC_SW_RESET, &fp->state);
	rcu_read_lock();
	ifr.ifr_flags = (short) dev_get_flags(ndev);
	rcu_read_unlock();

	ifr.ifr_flags &= ~IFF_UP;

	rtnl_lock();
	ret = dev_change_flags(ndev, ifr.ifr_flags, NULL);
	rtnl_unlock();

	clear_bit(HIFVMAC_NOE_EXEC_SW_RESET, &fp->state);

	return ret;
}

/**
 * hif_pltfr_resume
 * @dev: device pointer
 * Description: this function is invoked when resume the driver before calling
 * the main resume function, on some platforms, it can call own init helper
 * if required.
 */
static int hif_pltfr_resume(struct device *dev)
{
	int ret;
	struct ifreq ifr;
	struct net_device *ndev = dev_get_drvdata(dev);

	mdelay(6000);

	rcu_read_lock();
	ifr.ifr_flags = (short) dev_get_flags(ndev);
	rcu_read_unlock();

	ifr.ifr_flags &= ~IFF_UP;
	ifr.ifr_flags |= IFF_UP;

	rtnl_lock();
	ret = dev_change_flags(ndev, ifr.ifr_flags, NULL);
	rtnl_unlock();

	return ret;
}
#endif /* CONFIG_PM_SLEEP */

SIMPLE_DEV_PM_OPS(hif_pltfr_pm_ops, hif_pltfr_suspend,
		  hif_pltfr_resume);

MODULE_DESCRIPTION("Bst HIF driver");
MODULE_LICENSE("GPL v2");
