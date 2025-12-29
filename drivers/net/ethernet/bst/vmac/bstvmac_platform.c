// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021-2024 Black Sesame Technologies. All Rights Reserved.
 * Copyright (C) 2007-2011 STMicroelectronics Ltd
 */

#include <linux/of_net.h>
#include "bstvmac.h"

#ifdef CONFIG_OF
/**
 * bstvmac_hif_setup - parse DT parameters for multiple queues configuration
 * @pdev: platform device
 */
static int bstvmac_hif_setup(struct platform_device *pdev,
			     struct plat_vmacenet_data *plat)
{
	int ret = 0;
	struct device_node *rx_node;
	struct device_node *tx_node;

    rx_node = of_parse_phandle(pdev->dev.of_node, "hif-rx-config", 0);
	if (!rx_node) {
		return -EINVAL;
	}

	tx_node = of_parse_phandle(pdev->dev.of_node, "hif-tx-config", 0);
	if (!tx_node) {
		of_node_put(rx_node);
		return -EINVAL;
	}

	/* Processing RX queues common config */
	if (of_property_read_u32(rx_node, "rx-queues-to-use",
			&plat->rx_queues_to_use)) {
		plat->rx_queues_to_use = 1;
	}

	/* Processing TX queues common config */
	if (of_property_read_u32(tx_node, "tx-queues-to-use",
			&plat->tx_queues_to_use)) {
		plat->tx_queues_to_use = 1;
	}

	of_node_put(rx_node);
	of_node_put(tx_node);

	return ret;
}

/**
 * bstvmac_probe_config_dt - parse device-tree driver parameters
 * @pdev: platform_device structure
 * @mac: MAC address to use
 * Description:
 * this function is to read the driver parameters from device-tree and
 * set some private fields that will be used by the main at runtime.
 */
struct plat_vmacenet_data *bstvmac_probe_config_dt(struct platform_device
						     *pdev, u8 *mac)
{
	struct device_node *np = pdev->dev.of_node;
	struct plat_vmacenet_data *plat;
	int rc;

	plat = devm_kzalloc(&pdev->dev, sizeof(*plat), GFP_KERNEL);
	if (!plat)
		return ERR_PTR(-ENOMEM);

	rc = of_get_mac_address(np, mac);
	if (rc) {
		if (rc == -EPROBE_DEFER)
			return ERR_PTR(rc);

		eth_zero_addr(mac);
	}

    if (of_device_is_compatible(np, "bst,vmac-hif")) {
		plat->has_vmac = 1;
	}

	of_property_read_u32(np, "ethernet-id", &plat->bus_id);

	rc = bstvmac_hif_setup(pdev, plat);
	if (rc) {
		return ERR_PTR(rc);
	}

	/* Set the maxmtu to a default of JUMBO_LEN in case the
	 * parameter is not present in the device tree.
	 */
	plat->maxmtu = JUMBO_LEN;

	return plat;
}
EXPORT_SYMBOL_GPL(bstvmac_probe_config_dt);
#endif /* CONFIG_OF */

int bstvmac_get_platform_resources(struct platform_device *pdev,
				   struct bstvmac_resources *bstvmac_res)
{
	int i;
	struct resource *res;
	char irq_name[128] = { 0 };
	DECLARE_BITMAP(_channel_mask, 64);
    struct device_node *np = pdev->dev.of_node;

	memset(bstvmac_res, 0, sizeof(*bstvmac_res));

	for (i = 0; i < BSTVMAC_MAX_IRQ_NUM; i++) {
		snprintf(irq_name, sizeof(irq_name), "vmac_irq%d", i);
		bstvmac_res->perch_irq[i] = platform_get_irq_byname(pdev, irq_name);
		if (bstvmac_res->perch_irq[i] < 0) {
			if (i == 0) {
				return -EPROBE_DEFER;
			}
			break;
		}
	}

	if (of_property_read_u64(np, "channel-bit", &bstvmac_res->hif_channel_bit)) {
#if defined(CONFIG_BST_C1200_DB)
		bstvmac_res->hif_channel_bit = 0x100;
#elif defined(CONFIG_BST_C1200_IVI)
		bstvmac_res->hif_channel_bit = 0x10000;
#endif
	}

	bitmap_from_u64(_channel_mask, bstvmac_res->hif_channel_bit);
	i = find_first_bit(_channel_mask, 64) / BSTVMAC_CHANNELS_PER_IRQ;
    if(i >= BSTVMAC_MAX_IRQ_NUM) {
        i = 0;
    }
	bstvmac_res->irq = bstvmac_res->perch_irq[i];
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	bstvmac_res->addr = devm_ioremap_resource(&pdev->dev, res);

	return PTR_ERR_OR_ZERO(bstvmac_res->addr);
}
EXPORT_SYMBOL_GPL(bstvmac_get_platform_resources);

/**
 * bstvmac_pltfr_remove
 * @pdev: platform device pointer
 * Description: this function calls the main to free the net resources
 * and calls the platforms hook and release the resources (e.g. mem).
 */
int bstvmac_pltfr_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct bstvmac_priv *priv = netdev_priv(ndev);
	struct plat_vmacenet_data *plat = priv->plat;
	int ret = bstvmac_dvr_remove(&pdev->dev);

	if (plat->exit)
		plat->exit(pdev, plat->bsp_priv);

	return ret;
}
EXPORT_SYMBOL_GPL(bstvmac_pltfr_remove);

void bstvmac_pltfr_shutdown(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct bstvmac_priv *priv = netdev_priv(ndev);
	struct plat_vmacenet_data *plat = priv->plat;

	netdev_info(priv->dev, "%s", __func__);

	bstvmac_dvr_remove(&pdev->dev);

	if (plat->exit)
		plat->exit(pdev, plat->bsp_priv);
}
EXPORT_SYMBOL_GPL(bstvmac_pltfr_shutdown);

#ifdef CONFIG_PM_SLEEP
/**
 * bstvmac_pltfr_suspend
 * @dev: device pointer
 * Description: this function is invoked when suspend the driver and it directly
 * call the main suspend function and then, if required, on some platform, it
 * can call an exit helper.
 */
static int bstvmac_pltfr_suspend(struct device *dev)
{
	int ret;
	struct net_device *ndev = dev_get_drvdata(dev);
	struct bstvmac_priv *priv = netdev_priv(ndev);
	struct platform_device *pdev = to_platform_device(dev);

	ret = bstvmac_suspend(dev);
	if (priv->plat->exit)
		priv->plat->exit(pdev, priv->plat->bsp_priv);

	return ret;
}

/**
 * bstvmac_pltfr_resume
 * @dev: device pointer
 * Description: this function is invoked when resume the driver before calling
 * the main resume function, on some platforms, it can call own init helper
 * if required.
 */
static int bstvmac_pltfr_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct bstvmac_priv *priv = netdev_priv(ndev);
	struct platform_device *pdev = to_platform_device(dev);

	if (priv->plat->init)
		priv->plat->init(pdev, priv->plat->bsp_priv);

	return bstvmac_resume(dev);
}

SIMPLE_DEV_PM_OPS(bstvmac_pltfr_pm_ops, bstvmac_pltfr_suspend,
		  bstvmac_pltfr_resume);

#endif /* CONFIG_PM_SLEEP */

MODULE_DESCRIPTION("BSTVMAC 1000 Ethernet platform support");
MODULE_AUTHOR("BST");
MODULE_LICENSE("GPL");
