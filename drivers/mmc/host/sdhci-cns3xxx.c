/*
 * SDHCI support for CNS3xxx SoC
 *
 * Copyright 2008 Cavium Networks
 * Copyright 2010 MontaVista Software, LLC.
 *
 * Authors: Scott Shu
 *	    Anton Vorontsov <avorontsov@mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/mmc/host.h>
#include <linux/sdhci-pltfm.h>
#include <mach/cns3xxx.h>
#include "sdhci.h"
#include "sdhci-pltfm.h"

static unsigned int sdhci_cns3xxx_get_max_clk(struct sdhci_host *host)
{
	int clk = 50000000;

	return clk;
}

static unsigned int sdhci_cns3xxx_get_timeout_clk(struct sdhci_host *host)
{
	return sdhci_cns3xxx_get_max_clk(host) / 100000;
}

/*
 * sdhci_cns3xxx_set_clock - callback on clock change
 *
 * When the card's clock is going to be changed, look at the new frequency
 * and find the best clock source to go with it.
 */
static void sdhci_cns3xxx_set_clock(struct sdhci_host *host, unsigned int clock)
{
	u16 clk;
	unsigned long timeout;
	int max_speed, div = 1;
	int hclk = cns3xxx_cpu_clock()/4;

	if (clock == host->clock)
		return;

	sdhci_writew(host, 0, SDHCI_CLOCK_CONTROL);

	if (clock == 0)
		goto out;

   /*
	* SD frequency divider should be determined by HCLK
	* the maximum SD clock is 25MHz (normal speed mode)
	* or 50MHz (high speed mode)
	* formula => SD clock = (HCLK / divisor)
	*/
	div = 1;
	hclk = cns3xxx_cpu_clock()/4;

	if (0x4 & sdhci_readw(host, 0x28))
		max_speed = 50;
	else
		max_speed = 25;

	while (max_speed < (hclk/div))
		div++;

	switch (div) {
	case 1:
		clk = 0x00 << SDHCI_DIVIDER_SHIFT;
		break;
	case 2:
		/* base clock divided by 2 */
		clk = 0x01 << SDHCI_DIVIDER_SHIFT;
		break;
	case 3:
		/* base clock divided by 3 */
		clk = 0x03 << SDHCI_DIVIDER_SHIFT;
		break;
	case 4:
		/* base clock divided by 4 */
		clk = 0x02 << SDHCI_DIVIDER_SHIFT;
		break;
	case 5:
	case 6:
	case 7:
	case 8:
		/* base clock divided by 8 */
		clk = 0x04 << SDHCI_DIVIDER_SHIFT;
		break;
	default:
		/* base clock divided by 16 */
		clk = 0x08 << SDHCI_DIVIDER_SHIFT;
		break;
	}

	clk |= SDHCI_CLOCK_INT_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	timeout = 10;
	while (!((clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL))
		& SDHCI_CLOCK_INT_STABLE)) {
		if (timeout == 0)
			return;

		timeout--;
		mdelay(1);
	}

	clk |= SDHCI_CLOCK_CARD_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	host->timeout_clk = sdhci_cns3xxx_get_timeout_clk(host);
out:
	host->clock = clock;
}

static struct sdhci_ops sdhci_cns3xxx_ops = {
	.get_max_clock	= sdhci_cns3xxx_get_max_clk,
	.set_clock	= sdhci_cns3xxx_set_clock,
};

int sdhci_cns3xxx_init(struct sdhci_host *host)
{
	u32 gpioa_pins = __raw_readl(MISC_GPIOA_PIN_ENABLE_REG);
	/* MMC/SD pins share with GPIOA */
	gpioa_pins |= 0x1f0f0004;
	__raw_writel(gpioa_pins, MISC_GPIOA_PIN_ENABLE_REG);

	cns3xxx_pwr_clk_en(CNS3XXX_PWR_CLK_EN(SDIO));
	cns3xxx_pwr_soft_rst(CNS3XXX_PWR_SOFTWARE_RST(SDIO));
	return 0;
}

struct sdhci_pltfm_data sdhci_cns3xxx_pdata = {
	.ops = &sdhci_cns3xxx_ops,
	.quirks = SDHCI_QUIRK_BROKEN_DMA |
		  SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK |
		  SDHCI_QUIRK_INVERTED_WRITE_PROTECT |
		  SDHCI_QUIRK_NONSTANDARD_CLOCK,
	.init = sdhci_cns3xxx_init,
};
