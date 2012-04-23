/* Copyright 2009-2011 Freescale Semiconductor, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * QorIQ 10-G MDIO Controller
 *
 * Author: Andy Fleming <afleming@freescale.com>
 *
 * Based on fsl_pq_mdio.c
 */

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/unistd.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/crc32.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/mdio.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_mdio.h>
#include <linux/io.h>
#include <linux/uaccess.h>

#include <asm/irq.h>

#include "xgmac_mdio.h"

/* Write value to the PHY for this device to the register at regnum, */
/* waiting until the write is done before it returns.  All PHY */
/* configuration has to be done through the TSEC1 MIIM regs */
int xgmac_mdio_write(struct mii_bus *bus, int port_addr,
			int dev_addr, int regnum, u16 value)
{
	struct tgec_mdio_controller __iomem *regs = bus->priv;
	u32 mdio_ctl, mdio_stat;

	if (dev_addr == MDIO_DEVAD_NONE)
		return 0xffff;

	/* Setup the MII Mgmt clock speed */
	mdio_stat = MDIO_STAT_CLKDIV(100);
	out_be32(&regs->mdio_stat, mdio_stat);

	/* Wait till the bus is free */
	while ((in_be32(&regs->mdio_stat)) & MDIO_STAT_BSY)
		cpu_relax();

	/* Set the port and dev addr */
	mdio_ctl = MDIO_CTL_PORT_ADDR(port_addr) | MDIO_CTL_DEV_ADDR(dev_addr);
	out_be32(&regs->mdio_ctl, mdio_ctl);

	/* Set the register address */
	out_be32(&regs->mdio_addr, regnum & 0xffff);

	/* Wait till the bus is free */
	while ((in_be32(&regs->mdio_stat)) & MDIO_STAT_BSY)
		cpu_relax();

	/* Write the value to the register */
	out_be32(&regs->mdio_data, MDIO_DATA(value));

	/* Wait till the MDIO write is complete */
	while ((in_be32(&regs->mdio_data)) & MDIO_DATA_BSY)
		cpu_relax();

	return 0;
}


/* Reads from register regnum in the PHY for device dev, */
/* returning the value.  Clears miimcom first.  All PHY */
/* configuration has to be done through the TSEC1 MIIM regs */
int xgmac_mdio_read(struct mii_bus *bus, int port_addr, int dev_addr,
			int regnum)
{
	struct tgec_mdio_controller __iomem *regs = bus->priv;
	u32 mdio_ctl, mdio_stat;

	/* Setup the MII Mgmt clock speed */
	mdio_stat = MDIO_STAT_CLKDIV(100);
	out_be32(&regs->mdio_stat, mdio_stat);

	/* Wait till the bus is free */
	while ((in_be32(&regs->mdio_stat)) & MDIO_STAT_BSY)
		cpu_relax();

	/* Set the Port and Device Addrs */
	mdio_ctl = MDIO_CTL_PORT_ADDR(port_addr) | MDIO_CTL_DEV_ADDR(dev_addr);
	out_be32(&regs->mdio_ctl, mdio_ctl);

	/* Set the register address */
	out_be32(&regs->mdio_addr, regnum & 0xffff);

	/* Wait till the bus is free */
	while ((in_be32(&regs->mdio_stat)) & MDIO_STAT_BSY)
		cpu_relax();

	/* Initiate the read */
	mdio_ctl |= MDIO_CTL_READ;
	out_be32(&regs->mdio_ctl, mdio_ctl);

	/* Wait till the MDIO write is complete */
	while ((in_be32(&regs->mdio_data)) & MDIO_DATA_BSY)
		cpu_relax();

	/* Return all Fs if nothing was there */
	if (in_be32(&regs->mdio_stat) & MDIO_STAT_RD_ER)
		return 0xffff;

	return in_be32(&regs->mdio_data) & 0xffff;
}


/* Reset the MIIM registers, and wait for the bus to free */
static int xgmac_mdio_reset(struct mii_bus *bus)
{
	struct tgec_mdio_controller __iomem *regs = bus->priv;
	int timeout = PHY_INIT_TIMEOUT;
	u32 mdio_stat;

	mutex_lock(&bus->mdio_lock);

	/* Setup the MII Mgmt clock speed */
	mdio_stat = MDIO_STAT_CLKDIV(100);
	out_be32(&regs->mdio_stat, mdio_stat);

	/* Wait till the bus is free */
	while (((in_be32(&regs->mdio_stat)) & MDIO_STAT_BSY) && timeout--)
		cpu_relax();

	mutex_unlock(&bus->mdio_lock);

	if (timeout < 0) {
		printk(KERN_ERR "%s: The MII Bus is stuck!\n",
				bus->name);
		return -EBUSY;
	}

	return 0;
}


static int xgmac_mdio_probe(struct platform_device *ofdev)
{
	struct tgec_mdio_controller __iomem *regs;
	struct device_node *np = ofdev->dev.of_node;
	struct mii_bus *new_bus;
	u64 addr, size;
	int err = 0;

	if (!of_device_is_available(np))
		return -ENODEV;

	new_bus = mdiobus_alloc();
	if (NULL == new_bus)
		return -ENOMEM;

	new_bus->name = "Freescale XGMAC MDIO Bus",
	new_bus->read = &xgmac_mdio_read,
	new_bus->write = &xgmac_mdio_write,
	new_bus->reset = &xgmac_mdio_reset,

	/* Set the PHY base address */
	addr = of_translate_address(np, of_get_address(np, 0, &size, NULL));
	regs = ioremap(addr, size);

	if (NULL == regs) {
		err = -ENOMEM;
		goto err_ioremap;
	}

	new_bus->priv = (void __force *)regs;

	new_bus->irq = kcalloc(PHY_MAX_ADDR, sizeof(int), GFP_KERNEL);

	if (NULL == new_bus->irq) {
		err = -ENOMEM;
		goto err_irq_alloc;
	}

	new_bus->parent = &ofdev->dev;
	dev_set_drvdata(&ofdev->dev, new_bus);

	sprintf(new_bus->id, "%s", np->name);

	err = of_mdiobus_register(new_bus, np);

	if (err) {
		printk(KERN_ERR "%s: Cannot register as MDIO bus\n",
				new_bus->name);
		goto err_registration;
	}

	return 0;

err_registration:
	kfree(new_bus->irq);
err_irq_alloc:
	iounmap(regs);
err_ioremap:
	return err;
}


static int xgmac_mdio_remove(struct platform_device *ofdev)
{
	struct device *device = &ofdev->dev;
	struct mii_bus *bus = dev_get_drvdata(device);

	mdiobus_unregister(bus);

	dev_set_drvdata(device, NULL);

	iounmap((void __iomem *)bus->priv);
	bus->priv = NULL;
	mdiobus_free(bus);

	return 0;
}

static struct of_device_id xgmac_mdio_match[] = {
	{
		.compatible = "fsl,fman-xmdio",
	},
	{},
};

static struct platform_driver xgmac_mdio_driver = {
	.driver = {
		.name = "fsl-fman_xmdio",
		.of_match_table = xgmac_mdio_match,
	},
	.probe = xgmac_mdio_probe,
	.remove = xgmac_mdio_remove,
};

int __init xgmac_mdio_init(void)
{
	return platform_driver_register(&xgmac_mdio_driver);
}

void xgmac_mdio_exit(void)
{
	platform_driver_unregister(&xgmac_mdio_driver);
}
subsys_initcall_sync(xgmac_mdio_init);
module_exit(xgmac_mdio_exit);
