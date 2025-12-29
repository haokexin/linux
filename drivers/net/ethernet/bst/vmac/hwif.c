// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021-2024 Black Sesame Technologies. All Rights Reserved.
 * Copyright (C) 2018 Synopsys, Inc. and/or its affiliates.
 */

#include "bstvmac.h"

static u32 bstvmac_get_id(struct bstvmac_priv *priv, u32 id_reg)
{
	u32 reg = readl(priv->ioaddr + id_reg);

	if (!reg) {
		dev_info(priv->device, "Version ID not available\n");
		return 0x0;
	}

	dev_info(priv->device, "Hif ID: 0x%x, Hif Version: 0x%x\n",
		 (unsigned int)(reg & GENMASK(15, 0)),
			(unsigned int)(reg & GENMASK(23, 16)) >> 16);
	return reg & GENMASK(15, 0);
}

static const struct bstvmac_hwif_entry {
	bool vmac;
	u32 min_id;
	const struct bstvmac_regs_off regs;
	const void *desc;
	const void *dma;
	const void *mac;
	const void *mode;
	int (*setup)(struct bstvmac_priv *priv);
	int (*quirks)(struct bstvmac_priv *priv);
} bstvmac_hw[] = {
	/* NOTE: New HW versions shall go to the end of this table */
	{
		.vmac = true,
		.min_id = BSTVMAC_CORE_10,
		.regs = {
			.base_off = HIF_BASE_ADDR,
			.chanl_start_off = HIF_CHANL_START_ADDR,
		},
		.desc = &dwvmac10_desc_ops,
		.dma = &dwvmac10_dma_ops,
		.mac = &dwvmac10_ops,
		.mode = &dwvmac10_chain_ops,
		.quirks = NULL,
	},
};

int bstvmac_hwif_init(struct bstvmac_priv *priv)
{
	bool needs_vmac = priv->plat->has_vmac;
	const struct bstvmac_hwif_entry *entry;
	struct mac_device_info *mac;
	bool needs_setup = false;
	int i, ret;
	u32 id;

	if (needs_vmac)
		id = bstvmac_get_id(priv, HIF_BASE_ADDR + HIF_VERSION_OFFSET);
	else
		id = 0;

	/* Save ID for later use */
	priv->vmac_id = id;

	/* Lets assume some safe values first */
	priv->hif_base_addr = priv->ioaddr + HIF_BASE_ADDR;
	priv->chanl_start_addr = priv->ioaddr + HIF_CHANL_START_ADDR;

	/* allocate mac structure first */
	mac = devm_kzalloc(priv->device, sizeof(*mac), GFP_KERNEL);
	if (!mac)
		return -ENOMEM;

	/* Fallback to generic HW */
	for (i = ARRAY_SIZE(bstvmac_hw) - 1; i >= 0; i--) {
		entry = &bstvmac_hw[i];

		if (needs_vmac ^ entry->vmac)
			continue;

		/* Use vmac_id var because some setups can override this */
		if (priv->vmac_id < entry->min_id)
			continue;

		/* Only use generic HW helpers if needed */
		mac->desc = mac->desc ? : entry->desc;
		mac->dma = mac->dma ? : entry->dma;
		mac->mac = mac->mac ? : entry->mac;
		mac->mode = mac->mode ? : entry->mode;

		priv->hw = mac;
		priv->hif_base_addr = priv->ioaddr + entry->regs.base_off;
		priv->chanl_start_addr = priv->ioaddr + entry->regs.chanl_start_off;

		/* Entry found */
		if (needs_setup) {
			ret = entry->setup(priv);
			if (ret)
				return ret;
		}

		/* Save quirks, if needed for posterior use */
		priv->hwif_quirks = entry->quirks;
		return 0;
	}

	dev_err(priv->device, "Failed to find HW IF (vmac=%d)\n",
		needs_vmac);
	return -EINVAL;
}
