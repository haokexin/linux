// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021-2024 Black Sesame Technologies. All Rights Reserved.
 * Copyright (C) 2011 STMicroelectronics Ltd
 */
#include "common.h"
#include "bstvmac_hif.h"

static void init_dma_chain(void *des, dma_addr_t phy_addr, unsigned int size, bool tx)
{
	int i;
	dma_addr_t next_bd_pa;
	struct dma_bd_desc *bd_va;

	bd_va = (struct dma_bd_desc *)des;
	for (i = 0; i < size; i++) {
		if (tx)
			bd_va->des0 &= ~VMAC_BD_DES0_CTRL;

		if (i == size - 1) {
			bd_va->des3 = (u32)phy_addr;
			bd_va->des1 |= (phy_addr >> 8) & VMAC_BD_DES1_NEXTPTR;
		} else {
			next_bd_pa = phy_addr + ((i + 1) * sizeof(struct dma_bd_desc));
			bd_va->des3 = (u32)next_bd_pa;
			bd_va->des1 |= (next_bd_pa >> 8) & VMAC_BD_DES1_NEXTPTR;
		}

		bd_va = bd_va + 1;
	}
}

static void clean_desc0(void *priv_ptr, struct dma_bd_desc *p)
{
	p->des0 &= ~VMAC_BD_DES0_SEQNUM;
	p->des0 &= ~VMAC_BD_DES0_CTRL;
}

const struct bstvmac_mode_ops dwvmac10_chain_ops = {
	.init = init_dma_chain,
	.clean_desc0 = clean_desc0,
};
