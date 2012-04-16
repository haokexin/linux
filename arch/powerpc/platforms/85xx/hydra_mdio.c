/*
 * Provides PIXIS-based MDIO muxing the Hydra and Super Hydra boards
 *
 * Copyright 2010-2012 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * Hydra is the codename for the P3041 DS & P5020 DS boards
 * Super Hydra is the codename for the P5040 DS boards
 */

#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/of_platform.h>
#include <asm/io.h>

#define BRDCFG1	0x9
#define BRDCFG1_EMI1_SEL_MASK	0x70
#define BRDCFG1_EMI1_EN		0x08


struct hydra_mdio {
	struct mii_bus *real_bus;
	int irqs[PHY_MAX_ADDR];
	u8 *pixis;
	u8 value;
	u8 mask;
};

/* Mutex for the EMI register in PIXIS */
static DEFINE_MUTEX(emi_lock);

/* Set the BRDCFG1 pixis register, and then write the MDIO regs */
static int hydra_mdio_write(struct mii_bus *bus, int port_addr, int dev_addr,
			int regnum, u16 value)
{
	struct hydra_mdio *priv = bus->priv;
	struct mii_bus *real = priv->real_bus;
	int ret;

	mutex_lock(&emi_lock);

	/* Write BRDCFG1 to select this bus */
	clrsetbits_8(priv->pixis + BRDCFG1, priv->mask, priv->value);

	/* Write through to the attached MDIO bus */
	ret = real->write(real, port_addr, dev_addr, regnum, value);

	mutex_unlock(&emi_lock);

	return ret;
}

/* Set the BRDCFG1 pixis register, and then read from the MDIO bus */
static int hydra_mdio_read(struct mii_bus *bus, int port_addr, int dev_addr,
			int regnum)
{
	struct hydra_mdio *priv = bus->priv;
	struct mii_bus *real = priv->real_bus;
	int ret;

	mutex_lock(&emi_lock);

	clrsetbits_8(priv->pixis + BRDCFG1, priv->mask, priv->value);

	ret = real->read(real, port_addr, dev_addr, regnum);

	mutex_unlock(&emi_lock);
 
	return ret;
}


/* Reset the MIIM registers, and wait for the bus to free */
static int hydra_mdio_reset(struct mii_bus *bus)
{
	struct hydra_mdio *priv = bus->priv;
	struct mii_bus *real = priv->real_bus;
	int ret;

	mutex_lock(&emi_lock);

	clrsetbits_8(priv->pixis + BRDCFG1, priv->mask, priv->value);

	ret = real->reset(real);

	mutex_unlock(&emi_lock);

	return ret;
}

static struct of_device_id hydra_pixis_match[] = {
	{
		.compatible = "fsl,p3041ds-pixis",
	},
	{
		.compatible = "fsl,p5020ds-pixis",
	},
	{
		.compatible = "fsl,p5040ds-pixis",
	},
};

static int __devinit hydra_mdio_probe(struct of_device *ofdev,
		const struct of_device_id *match)
{
	struct device_node *np = ofdev->node;
	struct mii_bus *new_bus;
	struct hydra_mdio *priv;
	struct device_node *mdio_np, *pixis_np;
	struct of_device *ofmdiodev;
	const u32 *iprop;
	int ret;

	if (!of_device_is_available(np))
		return -ENODEV;

	/* TODO: Replace with mdiobus_alloc_size() upstream */
	new_bus = mdiobus_alloc();
	if (!new_bus)
		return -ENOMEM;

	new_bus->name = "Freescale Hydra MDIO Bus";
	new_bus->read = hydra_mdio_read;
	new_bus->write = hydra_mdio_write;
	new_bus->reset = hydra_mdio_reset;
	strncpy(new_bus->id, np->name, MII_BUS_ID_SIZE);

	priv = kzalloc(sizeof(struct hydra_mdio), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto err_priv_alloc;
	}

	new_bus->priv = priv;
	new_bus->irq = priv->irqs; /* Initialized by of_mdiobus_register() */
	new_bus->parent = &ofdev->dev;

	/* Get the parent (real) mdio bus */
	mdio_np = of_parse_phandle(np, "fsl,mdio-handle", 0);
	if (!mdio_np) {
		dev_err(&ofdev->dev, "could not find real MDIO bus for %s\n",
		       np->full_name);
		ret = -ENODEV;
		goto err_no_mdio_node;
	}

	ofmdiodev = of_find_device_by_node(mdio_np);
	if (!ofmdiodev) {
		dev_err(&ofdev->dev, "could not find device for MDIO node %s\n",
		       mdio_np->full_name);
		ret = -ENODEV;
		goto err_no_mdio_dev;
	}

	priv->real_bus = dev_get_drvdata(&ofmdiodev->dev);
	if (!priv->real_bus) {
		dev_err(&ofdev->dev, "MDIO node %s has not been probed\n",
			mdio_np->full_name);
		ret = -ENODEV;
		goto err_no_pdev;
	}

	/* Find the pixis node */
	pixis_np = of_find_matching_node(NULL, hydra_pixis_match);
	if (!pixis_np) {
		dev_err(&ofdev->dev, "could not find PIXIS node\n");
		ret = -ENODEV;
		goto err_no_pixis;
	}

	priv->pixis = of_iomap(pixis_np, 0);
	if (!priv->pixis) {
		dev_err(&ofdev->dev, "could not map PIXIS node %s\n",
			pixis_np->full_name);
		ret = -ENOMEM;
		goto err_pixis_iomap;
	}

	/* Get the MDIO MUX mask and value */
	iprop = of_get_property(np, "fsl,hydra-mdio-mux-val", NULL);
	if (!iprop)
		/* Older device trees used -muxval instead of -mux-val */
		iprop = of_get_property(np, "fsl,hydra-mdio-muxval", NULL);
	if (!iprop) {
		dev_err(&ofdev->dev, "no MUX value found for %s\n",
			np->full_name);
		ret = -ENODEV;
		goto err_get_muxval;
	}
	/* Note: some trees already have BRDCFG1_EMI1_EN in the value */
	priv->value = BRDCFG1_EMI1_EN | be32_to_cpup(iprop);

	iprop = of_get_property(np, "fsl,hydra-mdio-mux-mask", NULL);
	if (iprop)
		priv->mask = be32_to_cpup(iprop);
	else
		/* Older device trees assumed a hard-coded constant mask */
		priv->mask = BRDCFG1_EMI1_SEL_MASK;

	/* Finally, register our new bus */
	ret = of_mdiobus_register(new_bus, np);
	if (ret) {
		dev_err(&ofdev->dev, "cannot register MDIO bus %s (ret = %i)\n",
		       new_bus->name, ret);
		goto err_registration;
	}

	return 0;

err_registration:
err_get_muxval:
	iounmap(priv->pixis);
err_pixis_iomap:
	of_node_put(pixis_np);
err_no_pixis:
err_no_pdev:
err_no_mdio_dev:
	of_node_put(mdio_np);
err_no_mdio_node:
	kfree(priv);
err_priv_alloc:
	mdiobus_free(new_bus);

	return ret;
}


static int __devexit hydra_mdio_remove(struct of_device *ofdev)
{
	struct device *device = &ofdev->dev;
	struct mii_bus *bus = dev_get_drvdata(device);
	struct hydra_mdio *priv = bus->priv;

	mdiobus_unregister(bus);
	dev_set_drvdata(device, NULL);

	iounmap(priv->pixis);
	kfree(bus->priv);

	mdiobus_free(bus);

	return 0;
}

static struct of_device_id hydra_mdio_match[] = {
	{
		.compatible = "fsl,hydra-mdio",
	},
	{
		.compatible = "fsl,hydra-xmdio",
	},
	{}
};
MODULE_DEVICE_TABLE(of, mdio_match);

static struct of_platform_driver hydra_mdio_driver = {
	.name = "hydra_mdio",
	.probe = hydra_mdio_probe,
	.remove = hydra_mdio_remove,
	.match_table = hydra_mdio_match,
};

static int __init hydra_mdio_init(void)
{
	return of_register_platform_driver(&hydra_mdio_driver);
}

subsys_initcall_sync(hydra_mdio_init);
