// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021-2024 Black Sesame Technologies. All Rights Reserved.
 * Copyright (C) 2018 Synopsys, Inc. and/or its affiliates.
 */
#include "common.h"
#include "bstvmac_hif.h"

static void dwvmac10_dma_init_rx_chan(void __iomem *ioaddr,
					dma_addr_t bd_phy, dma_addr_t wrbd_phy, u32 chan)
{
	writel(upper_32_bits(wrbd_phy), ioaddr + HIF_RX_BDP_WR_HIGH_ADDR_CH(chan));
	writel(lower_32_bits(wrbd_phy), ioaddr + HIF_RX_BDP_WR_LOW_ADDR_CH(chan));

	writel(upper_32_bits(bd_phy), ioaddr + HIF_RX_BDP_RD_HIGH_ADDR_CH(chan));
	writel(lower_32_bits(bd_phy), ioaddr + HIF_RX_BDP_RD_LOW_ADDR_CH(chan));
}

static void dwvmac10_dma_init_tx_chan(void __iomem *ioaddr,
				    dma_addr_t bd_phy, dma_addr_t wrbd_phy, u32 chan)
{
	writel(upper_32_bits(wrbd_phy), ioaddr + HIF_TX_BDP_WR_HIGH_ADDR_CH(chan));
	writel(lower_32_bits(wrbd_phy), ioaddr + HIF_TX_BDP_WR_LOW_ADDR_CH(chan));

	writel(upper_32_bits(bd_phy), ioaddr + HIF_TX_BDP_RD_HIGH_ADDR_CH(chan));
	writel(lower_32_bits(bd_phy), ioaddr + HIF_TX_BDP_RD_LOW_ADDR_CH(chan));
}

static void dwvmac10_enable_dma_irq(void __iomem *ioaddr, u32 chan,
				    bool rx, bool tx)
{
	u32 value = readl(ioaddr + HIF_CH_INT_EN(chan));

	value |= HIF_CH_INT_ENABLE;
	if (rx)
		value |= BDP_CSR_RX_PKT_CH_INT_EN;
	if (tx)
		value |= BDP_CSR_TX_PKT_CH_INT_EN;

	writel(value, ioaddr + HIF_CH_INT_EN(chan));
	value = readl(ioaddr + HIF_CH_INT_EN(chan));

	if (rx && !(value & BDP_CSR_RX_PKT_CH_INT_EN)) {
		value |= HIF_CH_INT_ENABLE;
		value |= BDP_CSR_RX_PKT_CH_INT_EN;
		writel(value, ioaddr + HIF_CH_INT_EN(chan));
	}
	if (tx && !(value & BDP_CSR_TX_PKT_CH_INT_EN)) {
		value |= HIF_CH_INT_ENABLE;
		value |= BDP_CSR_TX_PKT_CH_INT_EN;
		writel(value, ioaddr + HIF_CH_INT_EN(chan));
	}
	
}

static void dwvmac10_disable_dma_irq(void __iomem *ioaddr, u32 chan,
				     bool rx, bool tx)
{
	u32 value = readl(ioaddr + HIF_CH_INT_EN(chan));

	if (rx) {
		value &= ~BDP_CSR_RX_CBD_CH_INT_EN;
		value &= ~BDP_CSR_RX_PKT_CH_INT_EN;
	}
	if (tx) {
		value &= ~BDP_CSR_TX_CBD_CH_INT_EN;
		value &= ~BDP_CSR_TX_PKT_CH_INT_EN;
	}
	writel(value, ioaddr + HIF_CH_INT_EN(chan));

	value = readl(ioaddr + HIF_CH_INT_EN(chan));
	if (rx && (value & BDP_CSR_RX_PKT_CH_INT_EN)) {
		value &= ~BDP_CSR_RX_CBD_CH_INT_EN;
		value &= ~BDP_CSR_RX_PKT_CH_INT_EN;
		writel(value, ioaddr + HIF_CH_INT_EN(chan));
	}

	if (tx && (value & BDP_CSR_TX_PKT_CH_INT_EN)) {
		value &= ~BDP_CSR_TX_CBD_CH_INT_EN;
		value &= ~BDP_CSR_TX_PKT_CH_INT_EN;
		writel(value, ioaddr + HIF_CH_INT_EN(chan));
	}
}

static void dwvmac10_dma_start_tx(void __iomem *ioaddr, u32 chan)
{
	u32 value;

	value = readl(ioaddr + HIF_CTRL_CH(chan));
	value |= CSR_TX_DMA_EN_CH_OUT | CSR_TX_BDP_POLL_CNTR_EN_CH_OUT;
	writel(value, ioaddr + HIF_CTRL_CH(chan));

	value = CSR_TX_BDP_CH_START_OUT;
	writel(value, ioaddr + HIF_TX_CH_START(chan));
}

static void dwvmac10_dma_stop_tx(void __iomem *ioaddr, u32 chan)
{
	u32 value;

	value = readl(ioaddr + HIF_TX_CH_START(chan));
	value &= ~CSR_TX_BDP_CH_START_OUT;
	writel(value, ioaddr + HIF_TX_CH_START(chan));
}

static void dwvmac10_dma_start_rx(void __iomem *ioaddr, u32 chan)
{
	u32 value;

	value = readl(ioaddr + HIF_CTRL_CH(chan));
	value |= CSR_RX_DMA_EN_CH_OUT | CSR_RX_BDP_POLL_CNTR_EN_CH_OUT;
	writel(value, ioaddr + HIF_CTRL_CH(chan));

	value = CSR_RX_BDP_CH_START_OUT;
	writel(value, ioaddr + HIF_RX_CH_START(chan));
}

static void dwvmac10_dma_stop_rx(void __iomem *ioaddr, u32 chan)
{
	u32 value;

	value = readl(ioaddr + HIF_CTRL_CH(chan));
	value &= ~(CSR_RX_DMA_EN_CH_OUT | CSR_RX_BDP_POLL_CNTR_EN_CH_OUT);
	writel(value, ioaddr + HIF_CTRL_CH(chan));

	value &= ~CSR_RX_BDP_CH_START_OUT;
	writel(value, ioaddr + HIF_RX_CH_START(chan));
}

static int dwvmac10_dma_ri_interrupt(void __iomem *ioaddr,
			     struct bstvmac_extra_stats *x, u32 chan)
{
    int ret = 0;
	u32 intr_status = readl(ioaddr + HIF_CH_INT_SRC(chan));

	/* RX Complete interrupts */
	if (likely(intr_status & BDP_CSR_RX_PKT_CH_INT_STS)) {
		x->rx_normal_irq_n++;
		ret |= handle_rx;
		/* Clear the interrupt by writing a logic 1 to the chanX interrupt
		 *  XGMAC_RI
		 */
		writel(BDP_CSR_RX_PKT_CH_INT_STS | BDP_CSR_RX_CBD_CH_INT_STS, ioaddr + HIF_CH_INT_SRC(chan));
	}

	dma_wmb();
	return ret;
}

static int dwvmac10_dma_ti_interrupt(void __iomem *ioaddr,
			     struct bstvmac_extra_stats *x, u32 chan)
{
    int ret = 0;
	u32 intr_status = readl(ioaddr + HIF_CH_INT_SRC(chan));

	/* TX Complete interrupts */
	if (likely(intr_status & BDP_CSR_TX_PKT_CH_INT_STS)) {
		x->tx_normal_irq_n++;
		ret |= handle_tx;
		/* Clear the interrupt by writing a logic 1 to the chanX interrupt
		 * DMA_CHAN_STATUS_TI
		 */
		writel(BDP_CSR_TX_PKT_CH_INT_STS | BDP_CSR_TX_CBD_CH_INT_STS, ioaddr + HIF_CH_INT_SRC(chan));
	}

	return ret;
}

static void dwvmac10_set_rx_bfsize(void __iomem *ioaddr, int bfsize, u32 chan)
{
	writel(bfsize, ioaddr + HIF_RX_WRBK_BD_CH_BUFFER_SIZE(chan));
}

static void dwvmac10_set_tx_bfsize(void __iomem *ioaddr, int bfsize, u32 chan)
{
	writel(bfsize, ioaddr + HIF_TX_WRBK_BD_CH_BUFFER_SIZE(chan));
}

static bool dwvmac10_rx_fifo_clear_status(void __iomem *ioaddr, u32 chan)
{
	unsigned int curr_wb_addr, init_wb_addr, rx_fifo_cnt;
	bool ret = false;

	curr_wb_addr = readl(ioaddr + HIF_RX_WR_CURR_BD_LOW_ADDR_CH(chan));
	init_wb_addr = readl(ioaddr + HIF_RX_BDP_WR_LOW_ADDR_CH(chan));
	rx_fifo_cnt = readl(ioaddr + HIF_BDP_CH_RX_FIFO_CNT(chan));

	if ((curr_wb_addr == init_wb_addr) && !rx_fifo_cnt) {
		ret = true;
	}

	return ret;
}

static bool dwvmac10_tx_fifo_clear_status(void __iomem *ioaddr, u32 chan)
{
	unsigned int curr_wb_addr, init_wb_addr;
	bool ret = false;

	curr_wb_addr = readl(ioaddr + HIF_TX_WR_CURR_BD_LOW_ADDR_CH(chan));
	init_wb_addr = readl(ioaddr + HIF_TX_BDP_WR_LOW_ADDR_CH(chan));

	if (curr_wb_addr == init_wb_addr) {
		ret = true;
	}

	return ret;
}

static void dwvmac10_recover_dma_irq(void __iomem *ioaddr, u32 chan)
{
	u32 value = readl(ioaddr + HIF_CH_INT_EN(chan));

	value &= (HIF_CH_INT_ENABLE | BDP_CSR_TX_PKT_CH_INT_EN | BDP_CSR_RX_PKT_CH_INT_EN);

	if (!value)
		dwvmac10_enable_dma_irq(ioaddr, chan, 1, 1);
}

const struct bstvmac_dma_ops dwvmac10_dma_ops = {
	.init_rx_chan = dwvmac10_dma_init_rx_chan,
	.init_tx_chan = dwvmac10_dma_init_tx_chan,
	.enable_dma_irq = dwvmac10_enable_dma_irq,
	.disable_dma_irq = dwvmac10_disable_dma_irq,
	.start_tx = dwvmac10_dma_start_tx,
	.stop_tx = dwvmac10_dma_stop_tx,
	.start_rx = dwvmac10_dma_start_rx,
	.stop_rx = dwvmac10_dma_stop_rx,
	.dma_ri_interrupt = dwvmac10_dma_ri_interrupt,
	.dma_ti_interrupt = dwvmac10_dma_ti_interrupt,
	.set_rx_bfsize = dwvmac10_set_rx_bfsize,
	.set_tx_bfsize = dwvmac10_set_tx_bfsize,
	.rx_fifo_clear_status = dwvmac10_rx_fifo_clear_status,
	.tx_fifo_clear_status = dwvmac10_tx_fifo_clear_status,
	.recover_dma_irq = dwvmac10_recover_dma_irq,
};
