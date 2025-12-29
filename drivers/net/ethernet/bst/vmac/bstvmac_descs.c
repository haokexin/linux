// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021-2024 Black Sesame Technologies. All Rights Reserved.
 * Copyright (C) 2018 Synopsys, Inc. and/or its affiliates.
 */
#include "common.h"
#include "bstvmac_hif.h"

static int dwvmac10_get_tx_status(void *data, struct bstvmac_extra_stats *x,
				  struct dma_bd_desc *p, struct dma_wrbd_desc *wp, void __iomem *ioaddr)
{
	unsigned int wtdes0 = le32_to_cpu(wp->des0);
	int ret = tx_done;

	if (unlikely(!(wtdes0 & VMAC_WRBD_DES0_CTRL)))
		return tx_not_done;

	if (unlikely(!(wtdes0 & VMAC_WRBD_DES0_LIFM)))
		return tx_not_ls;

	return ret;
}

static int dwvmac10_get_rx_status(void __iomem *ioaddr, u32 chan, u32 index,struct dma_wrbd_desc *wp)
{
	unsigned int wrdes0 = le32_to_cpu(wp->des0);

	if (unlikely(!(wrdes0 & VMAC_WRBD_DES0_CTRL))) {
		return no_frame;
	}

	if (unlikely(!(wrdes0 & VMAC_WRBD_DES0_LIFM)))
		return rx_not_ls;

	return good_frame;
}

static void dwvmac10_clear_status(struct dma_wrbd_desc *wp)
{
	wp->des0 &= ~VMAC_WRBD_DES0_CTRL;
}

static void dwvmac10_set_last(struct dma_bd_desc *p, bool last)
{
	if (last) {
		p->des0 |= cpu_to_le32(VMAC_BD_DES0_LAST_BD);
	}
}

static void dwvmac10_set_intr(struct dma_bd_desc *p, bool on)
{
    if (on) {
        p->des0 |= cpu_to_le32(VMAC_BD_DES0_PKT_INT_EN);
        p->des0 |= cpu_to_le32(VMAC_BD_DES0_CBD_INT_EN);
    } else {
        p->des0 &= ~cpu_to_le32(VMAC_BD_DES0_PKT_INT_EN);
        p->des0 &= ~cpu_to_le32(VMAC_BD_DES0_CBD_INT_EN);
    }
}

static void dwvmac10_set_dir(struct dma_bd_desc *p)
{
	p->des0 |= cpu_to_le32(VMAC_BD_DES0_DIR);
}

static void dwvmac10_set_sz(struct dma_bd_desc *p, u32 size)
{
	p->des1 |= cpu_to_le32(size & VMAC_BD_DES1_BUFLEN);
}

static void dwvmac10_set_seqnum(struct dma_bd_desc *p, u32 seq_num)
{
	p->des0 &= ~VMAC_BD_DES0_SEQNUM;
	p->des0 |= cpu_to_le32(seq_num & VMAC_BD_DES0_SEQNUM);
}

static void dwvmac10_set_owner(struct dma_bd_desc *p, bool en)
{
	if (en)
		p->des0 |= VMAC_BD_DES0_DESC_EN;
	else
		p->des0 &= ~VMAC_BD_DES0_DESC_EN;
}

static int dwvmac10_get_rx_frame_len(struct dma_wrbd_desc *wp)
{
	return (le32_to_cpu(wp->des1) & VMAC_WRBD_DES1_BUFLEN);
}

static void dwvmac10_init_bd_rx_desc(struct dma_bd_desc *p, u32 seq_num, bool last,
						unsigned int buf_sz)
{
	dwvmac10_set_dir(p);

	dwvmac10_set_intr(p, true);

	dwvmac10_set_sz(p, buf_sz);
	dwvmac10_set_seqnum(p, seq_num);
	dwvmac10_set_last(p, last);
	dwvmac10_set_owner(p, true);
}

static void dwvmac10_init_wrbd_desc(struct dma_wrbd_desc *p, u32 seq_num)
{
	p->des0 = 0;
	p->des1 = (seq_num << 16) & VMAC_WRBD_DES1_SEQNUM;
}

static void dwvmac10_init_bd_tx_desc(struct dma_bd_desc *p, int end)
{
	p->des0 = 0;
	p->des1 = 0;
	p->des2 = 0;
	p->des3 = 0;
}

static void dwvmac10_prepare_tx_desc(struct dma_bd_desc *p, int len,
				     bool ls_bd, bool tx_own, bool ls,
					 unsigned int seq_num)
{
	unsigned int tdes0 = le32_to_cpu(p->des0);

	p->des1 |= cpu_to_le32(len & VMAC_BD_DES1_BUFLEN);
	tdes0 &= ~VMAC_BD_DES0_SEQNUM;
	tdes0 |= cpu_to_le32(seq_num & VMAC_BD_DES0_SEQNUM);

	if (ls_bd)
		tdes0 |= VMAC_BD_DES0_LAST_BD;
	else
		tdes0 &= ~VMAC_BD_DES0_LAST_BD;

	if (ls)
		tdes0 |= VMAC_BD_DES0_LIFM;
	else
		tdes0 &= ~VMAC_BD_DES0_LIFM;

	/* Finally set the OWN bit. Later the DMA will start! */
	if (tx_own)
		tdes0 |= VMAC_BD_DES0_DESC_EN;
	else
		tdes0 &= ~VMAC_BD_DES0_DESC_EN;

	p->des0 = cpu_to_le32(tdes0);

	dma_wmb();
}

static void dwvmac10_release_tx_desc(struct dma_bd_desc *p, struct dma_wrbd_desc *wp, int mode)
{
	p->des0 = 0;
	p->des1 &= ~(VMAC_BD_DES1_BUFLEN | VMAC_BD_DES1_HADDR);
	p->des2 = 0;
	wp->des0 &= ~VMAC_WRBD_DES0_CTRL;
}

static void dwvmac10_set_tx_ic(struct dma_bd_desc *p)
{
	p->des0 |= cpu_to_le32(VMAC_BD_DES0_PKT_INT_EN | VMAC_BD_DES0_CBD_INT_EN);
}

static void dwvmac10_set_addr(struct dma_bd_desc *p, dma_addr_t addr)
{
	p->des2 = cpu_to_le32(lower_32_bits(addr));
	p->des1 |= cpu_to_le32((upper_32_bits(addr) << 16) & VMAC_BD_DES1_HADDR);
}

const struct bstvmac_desc_ops dwvmac10_desc_ops = {
	.tx_status = dwvmac10_get_tx_status,
	.rx_status = dwvmac10_get_rx_status,
	.clear_status = dwvmac10_clear_status,
	.set_tx_owner = dwvmac10_set_owner,
	.set_rx_owner = dwvmac10_set_owner,
	.get_rx_frame_len = dwvmac10_get_rx_frame_len,
	.set_tx_ic = dwvmac10_set_tx_ic,
	.prepare_tx_desc = dwvmac10_prepare_tx_desc,
	.release_tx_desc = dwvmac10_release_tx_desc,
	.init_bd_rx_desc = dwvmac10_init_bd_rx_desc,
	.init_wrbd_desc = dwvmac10_init_wrbd_desc,
	.init_bd_tx_desc = dwvmac10_init_bd_tx_desc,
	.set_addr = dwvmac10_set_addr,
	.set_rx_dir = dwvmac10_set_dir,
	.set_rx_intr = dwvmac10_set_intr,
	.set_rx_bufsz = dwvmac10_set_sz,
	.set_rx_seqnum = dwvmac10_set_seqnum,
	.set_rx_bd_last = dwvmac10_set_last,
};
