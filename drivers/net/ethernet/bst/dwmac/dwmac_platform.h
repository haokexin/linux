// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 * Copyright (C) 2007-2009 STMicroelectronics Ltd
 */

#ifndef __BSTGMAC_PLATFORM_H__
#define __BSTGMAC_PLATFORM_H__

#include "bstgmac.h"

struct plat_stmmacenet_data *
bstgmac_probe_config_dt(struct platform_device *pdev, u8 *mac);
void bstgmac_remove_config_dt(struct platform_device *pdev,
			      struct plat_stmmacenet_data *plat);

int bstgmac_get_platform_resources(struct platform_device *pdev,
				   struct bstgmac_resources *bstgmac_res);

int bstgmac_pltfr_remove(struct platform_device *pdev);
void bstgmac_pltfr_shutdown(struct platform_device *pdev);
extern const struct dev_pm_ops bstgmac_pltfr_pm_ops;

static inline void *get_bstgmac_bsp_priv(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct bstgmac_priv *priv = netdev_priv(ndev);

	return priv->plat->bsp_priv;
}

#endif /* __BSTGMAC_PLATFORM_H__ */
