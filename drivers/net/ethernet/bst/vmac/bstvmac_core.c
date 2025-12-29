// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021-2024 Black Sesame Technologies. All Rights Reserved.
 * Copyright (C) 2018 Synopsys, Inc. and/or its affiliates.
 */
#include "bstvmac.h"
#include "bstvmac_hif.h"

static void dwvmac10_timeout_en(void __iomem *ioaddr, bool enable)
{
	u32 regval;

	regval = readl(ioaddr + HIF_MISC);

	regval &= ~CSR_HIF_TIMEOUT_EN;
	if (enable) {
		regval |= CSR_HIF_TIMEOUT_EN;
	} else {
		regval &= ~CSR_HIF_TIMEOUT_EN;
	}

	writel(regval, ioaddr + HIF_MISC);
}

static void dwvmac10_set_start_seqnum(void __iomem *ioaddr, u32 value)
{
	u32 regval;

	regval = readl(ioaddr + HIF_MISC);
	regval |= (value << 16) & CSR_BD_START_SEQ_NUM;
	writel(regval, ioaddr + HIF_MISC);
}

static void dwvmac10_set_axi_write_done(void __iomem *ioaddr, u32 value)
{
	u32 regval;

	regval = readl(ioaddr + HIF_MISC);
	regval |= value & CSR_AXI_WRITE_DONE;
	writel(regval, ioaddr + HIF_MISC);
}

static void dwvmac10_seqnum_check_en(void __iomem *ioaddr, bool enable)
{
	u32 regval;

	regval = readl(ioaddr + HIF_MISC);

	regval &= ~CSR_SEQ_NUM_CHECK_EN;
	if (enable) {
		regval |= CSR_SEQ_NUM_CHECK_EN;
	} else {
		regval &= ~CSR_SEQ_NUM_CHECK_EN;
	}

	writel(regval, ioaddr + HIF_MISC);
}

static int dwvmac10_set_umac_addr(struct net_device *ndev, int flag)
{
	int ret = 0;

	ret = bstvmac_sendmsg_to_switch(ndev, flag);

	return ret;
}

const struct bstvmac_ops dwvmac10_ops = {
	.timeout_en = dwvmac10_timeout_en,
	.set_start_seqnum = dwvmac10_set_start_seqnum,
	.set_axi_write_done = dwvmac10_set_axi_write_done,
	.seqnum_check_en = dwvmac10_seqnum_check_en,
	.set_umac_addr = dwvmac10_set_umac_addr,
};