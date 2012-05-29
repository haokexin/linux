/*******************************************************************************
  STMMAC Ethernet Driver -- MDIO bus implementation
  Provides Bus interface for MII registers

  Copyright (C) 2007-2009  STMicroelectronics Ltd

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  this program; if not, write to the Free Software Foundation, Inc.,
  51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".

  Author: Carl Shaw <carl.shaw@st.com>
  Maintainer: Giuseppe Cavallaro <peppe.cavallaro@st.com>
*******************************************************************************/

#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/slab.h>

#include "stmmac.h"

#define MII_BUSY 0x00000001
#define MII_WRITE 0x00000002

/* CSR Frequency Access*/
#define F_20M	20000000
#define F_35M	35000000
#define F_60M	60000000
#define F_100M	100000000
#define F_150M	150000000
#define F_250M	50000000
#define F_300M	300000000

/* MDC Clock Selection */
#define	STMMAC_CLK_RANGE_60_100M	0	/* MDC = Clk/42 */
#define	STMMAC_CLK_RANGE_100_150M	1	/* MDC = Clk/62 */
#define	STMMAC_CLK_RANGE_20_35M		2	/* MDC = Clk/16 */
#define	STMMAC_CLK_RANGE_35_60M		3	/* MDC = Clk/26 */
#define	STMMAC_CLK_RANGE_150_250M	4	/* MDC = Clk/102 */
#define	STMMAC_CLK_RANGE_250_300M	5	/* MDC = Clk/122 */
u32 *mac1_bus;

static int stmmac_mdio_busy_wait(unsigned long ioaddr, unsigned int mii_addr)
{
	unsigned long finish = jiffies + 3 * HZ;

	do {
		if (readl(ioaddr + mii_addr) & MII_BUSY)
			cpu_relax();
		else
			return 0;
	} while (!time_after_eq(jiffies, finish));

	return -EBUSY;
}

static int stmmac_get_mac_clk(struct stmmac_priv *priv)
{
	u32 clk_rate = clk_get_rate(priv->stmmac_clk);

	/*
	 * Decide on the MDC clock dynamically based on the
	 * csr clock input.
	 * This is helpfull in case the cpu frequency is changed
	 * on the run using the cpu freq framework, and based
	 * on that the bus frequency is also changed.
	 */
	if ((clk_rate >= F_20M) && (clk_rate < F_35M))
		return STMMAC_CLK_RANGE_20_35M;
	else if ((clk_rate >= F_35M) && (clk_rate < F_60M))
		return STMMAC_CLK_RANGE_35_60M;
	else if ((clk_rate >= F_60M) && (clk_rate < F_100M))
		return STMMAC_CLK_RANGE_60_100M;
	else if ((clk_rate >= F_100M) && (clk_rate < F_150M))
		return STMMAC_CLK_RANGE_100_150M;
	else if ((clk_rate >= F_150M) && (clk_rate < F_250M))
		return STMMAC_CLK_RANGE_150_250M;
	else if ((clk_rate >= F_250M) && (clk_rate < F_300M))
		return STMMAC_CLK_RANGE_250_300M;
	else
		return STMMAC_CLK_RANGE_150_250M;

}
/**
 * stmmac_mdio_read
 * @bus: points to the mii_bus structure
 * @phyaddr: MII addr reg bits 15-11
 * @phyreg: MII addr reg bits 10-6
 * Description: it reads data from the MII register from within the phy device.
 * For the 7111 GMAC, we must set the bit 0 in the MII address register while
 * accessing the PHY registers.
 * Fortunately, it seems this has no drawback for the 7109 MAC.
 */
static int stmmac_mdio_read(struct mii_bus *bus, int phyaddr, int phyreg)
{
	int data;
	u16 regValue;
	struct net_device *ndev;
	struct stmmac_priv *priv;
	unsigned long ioaddr;
	unsigned int mii_address;
	unsigned int mii_data;

	ndev = bus->priv;
	priv = netdev_priv(ndev);

#ifdef CONFIG_ARCH_SPEAR13XX
	if (cpu_is_spear1310_reva() && mac1_bus)
		ndev = (struct net_device *)mac1_bus;
#endif

	ioaddr = ndev->base_addr;
	mii_address = priv->hw->mii.addr;
	mii_data = priv->hw->mii.data;

	/*
	 * If the clock framework is supported in the architecture code
	 * then dynamically select the mdio clock,
	 * Else the platform code would provide the csr clock
	 */
	if (priv->stmmac_clk)
		priv->mii_clk_csr = stmmac_get_mac_clk(priv);

	regValue = (((phyaddr << 11) & (0x0000F800)) |
			((phyreg << 6) & (0x000007C0)));
	regValue |= MII_BUSY | ((priv->mii_clk_csr & 7) << 2);

	if (stmmac_mdio_busy_wait(ioaddr, mii_address))
		return -EBUSY;

	writel(regValue, ioaddr + mii_address);

	if (stmmac_mdio_busy_wait(ioaddr, mii_address))
		return -EBUSY;

	/* Read the data from the MII data register */
	data = (int)readl(ioaddr + mii_data);

	return data;
}

/**
 * stmmac_mdio_write
 * @bus: points to the mii_bus structure
 * @phyaddr: MII addr reg bits 15-11
 * @phyreg: MII addr reg bits 10-6
 * @phydata: phy data
 * Description: it writes the data into the MII register from within the device.
 */
static int stmmac_mdio_write(struct mii_bus *bus, int phyaddr, int phyreg,
			     u16 phydata)
{
	struct net_device *ndev;
	struct stmmac_priv *priv;
	unsigned long ioaddr;
	unsigned int mii_address;
	unsigned int mii_data;
	u16 value;

	ndev = bus->priv;
	priv = netdev_priv(ndev);

#ifdef CONFIG_ARCH_SPEAR13XX
	if (cpu_is_spear1310_reva() && mac1_bus)
		ndev = (struct net_device *)mac1_bus;
#endif

	mii_address = priv->hw->mii.addr;
	mii_data = priv->hw->mii.data;
	ioaddr = ndev->base_addr;

	/*
	 * If the clock framework is supported in the architecture code
	 * then dynamically select the mdio clock,
	 * Else the platform code would provide the csr clock
	 */
	if (priv->stmmac_clk)
		priv->mii_clk_csr = stmmac_get_mac_clk(priv);

	value =
	(((phyaddr << 11) & (0x0000F800)) | ((phyreg << 6) & (0x000007C0)))
		| MII_WRITE;

	value |= MII_BUSY | ((priv->mii_clk_csr & 7) << 2);

	if (stmmac_mdio_busy_wait(ioaddr, mii_address))
		return -EBUSY;

	/* Set the MII address register to write */
	writel(phydata, ioaddr + mii_data);
	writel(value, ioaddr + mii_address);

	return stmmac_mdio_busy_wait(ioaddr, mii_address);
}

/**
 * stmmac_mdio_reset
 * @bus: points to the mii_bus structure
 * Description: reset the MII bus
 */
static int stmmac_mdio_reset(struct mii_bus *bus)
{
	struct net_device *ndev = bus->priv;
	struct stmmac_priv *priv = netdev_priv(ndev);
	unsigned int mii_address = priv->hw->mii.addr;

	if (priv->phy_reset) {
		pr_debug("stmmac_mdio_reset: calling phy_reset\n");
		priv->phy_reset(bus);
	}

	/* This is a workaround for problems with the STE101P PHY.
	 * It doesn't complete its reset until at least one clock cycle
	 * on MDC, so perform a dummy mdio read.
	 */
	writel(0, priv->ioaddr + mii_address);

	return 0;
}

/**
 * stmmac_mdio_register
 * @ndev: net device structure
 * Description: it registers the MII bus
 */
int stmmac_mdio_register(struct net_device *ndev)
{
	int err = 0;
	struct mii_bus *new_bus;
	int *irqlist;
	struct stmmac_priv *priv = netdev_priv(ndev);
	int addr, found;

	new_bus = mdiobus_alloc();
	if (new_bus == NULL)
		return -ENOMEM;

	irqlist = kzalloc(sizeof(int) * PHY_MAX_ADDR, GFP_KERNEL);
	if (irqlist == NULL) {
		err = -ENOMEM;
		goto irqlist_alloc_fail;
	}

	/* Assign IRQ to phy at address phy_addr */
	if (priv->phy_addr != -1)
		irqlist[priv->phy_addr] = priv->phy_irq;

	new_bus->name = "STMMAC MII Bus";
	new_bus->read = &stmmac_mdio_read;
	new_bus->write = &stmmac_mdio_write;
	new_bus->reset = &stmmac_mdio_reset;
	snprintf(new_bus->id, MII_BUS_ID_SIZE, "%x", priv->bus_id);
	new_bus->priv = ndev;
	new_bus->irq = irqlist;
	new_bus->phy_mask = priv->phy_mask;
	new_bus->parent = priv->device;

#ifdef CONFIG_ARCH_SPEAR13XX
	if (cpu_is_spear1310_reva() && (priv->bus_id == 0))
		mac1_bus = (u32 *)ndev;
#endif

	err = mdiobus_register(new_bus);
	if (err != 0) {
		pr_err("%s: Cannot register as MDIO bus\n", new_bus->name);
		goto bus_register_fail;
	}

	priv->mii = new_bus;

	found = 0;
	for (addr = 0; addr < 32; addr++) {
		struct phy_device *phydev = new_bus->phy_map[addr];
		if (phydev) {
			if (priv->phy_addr == -1) {
				priv->phy_addr = addr;
				phydev->irq = priv->phy_irq;
				irqlist[addr] = priv->phy_irq;
			}
			pr_info("%s: PHY ID %08x at %d IRQ %d (%s)%s\n",
			       ndev->name, phydev->phy_id, addr,
			       phydev->irq, dev_name(&phydev->dev),
			       (addr == priv->phy_addr) ? " active" : "");
			found = 1;
		}
	}

	if (!found)
		pr_warning("%s: No PHY found\n", ndev->name);

	return 0;
bus_register_fail:
	kfree(irqlist);
irqlist_alloc_fail:
	kfree(new_bus);
	return err;
}

/**
 * stmmac_mdio_unregister
 * @ndev: net device structure
 * Description: it unregisters the MII bus
 */
int stmmac_mdio_unregister(struct net_device *ndev)
{
	struct stmmac_priv *priv = netdev_priv(ndev);

	mdiobus_unregister(priv->mii);
	priv->mii->priv = NULL;
	kfree(priv->mii);

	return 0;
}
