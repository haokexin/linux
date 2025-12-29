// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021-2024 Black Sesame Technologies. All Rights Reserved.
 * Copyright (C) 2007-2009 STMicroelectronics Ltd
 */

#ifndef __BSTVMAC_PLATFORM_H__
#define __BSTVMAC_PLATFORM_H__

#include <linux/platform_device.h>

#define MAX_RX_QUEUES	1
#define MAX_TX_QUEUES	1
#define IMGMAC_CH_MAX	1

struct plat_vmacenet_data {
	int bus_id;
	int has_vmac;
	int maxmtu;
	u32 rx_queues_to_use;
	u32 tx_queues_to_use;
	int (*init)(struct platform_device *pdev, void *priv_ptr);
	void (*exit)(struct platform_device *pdev, void *priv);
	void *bsp_priv;
};

struct plat_vmacenet_data *
bstvmac_probe_config_dt(struct platform_device *pdev, u8 *mac);
int bstvmac_pltfr_remove(struct platform_device *pdev);
void bstvmac_pltfr_shutdown(struct platform_device *pdev);
#ifdef CONFIG_PM_SLEEP
extern const struct dev_pm_ops bstvmac_pltfr_pm_ops;
#endif

#endif /* __BSTVMAC_PLATFORM_H__ */
