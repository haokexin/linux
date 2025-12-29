// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021-2024 Black Sesame Technologies. All Rights Reserved.
 * Copyright (C) 2007-2009 STMicroelectronics Ltd
 */

#include <linux/ethtool.h>
#include "bstvmac.h"

#define VMAC_ETHTOOL_NAME	"hif_vmac"

static u32 bstvmac_ethtool_getmsglevel(struct net_device *dev)
{
	struct bstvmac_priv *priv = netdev_priv(dev);

	return priv->msg_enable;
}

static void bstvmac_ethtool_setmsglevel(struct net_device *dev, u32 level)
{
	struct bstvmac_priv *priv = netdev_priv(dev);
	unsigned long msglvl = (unsigned long)level;

	netdev_dbg(dev, "%s(), level = 0x%x\n", __func__, level);
	priv->msg_enable = (u32)msglvl;

	return;
}

static int bstvmac_check_if_running(struct net_device *dev)
{
	if (!netif_running(dev))
		return -EBUSY;
	return 0;
}

static void bstvmac_ethtool_getdrvinfo(struct net_device *dev,
				       struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, VMAC_ETHTOOL_NAME, sizeof(info->driver));
	strlcpy(info->version, DRV_MODULE_VERSION, sizeof(info->version));
}

static const struct ethtool_ops bstvmac_ethtool_ops = {
	.supported_coalesce_params = ETHTOOL_COALESCE_USECS |
	    ETHTOOL_COALESCE_MAX_FRAMES,
	.begin = bstvmac_check_if_running,
	.get_drvinfo = bstvmac_ethtool_getdrvinfo,
	.get_msglevel = bstvmac_ethtool_getmsglevel,
	.set_msglevel = bstvmac_ethtool_setmsglevel,
};

void bstvmac_set_ethtool_ops(struct net_device *netdev)
{
	netdev->ethtool_ops = &bstvmac_ethtool_ops;
};
