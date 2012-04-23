/*
 * Provides proper gpio muxing for the MDIO buses on the corenet DS boards
 *
 * Author: Andy Fleming <afleming@freescale.com>
 *
 * Copyright (c) 2009-2010 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
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
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/gpio.h>


struct p4080ds_mdio {
	struct mii_bus *real_bus;
	unsigned int num_gpios;
	unsigned int *gpios;
	int *gpio_values;
};

/* Set the GPIO mux, and then write the MDIO regs */
int p4080ds_mdio_write(struct mii_bus *bus, int port_addr, int dev_addr,
			int regnum, u16 value)
{
	struct p4080ds_mdio *priv = bus->priv;
	int i;

	/* Write the GPIO regs to select this bus */
	for (i = 0; i < priv->num_gpios; i++)
		gpio_set_value(priv->gpios[i], priv->gpio_values[i]);

	/* Write through to the attached MDIO bus */
	return priv->real_bus->write(priv->real_bus, port_addr, dev_addr,
					regnum, value);
}

/* Set the GPIO muxing, and then read from the MDIO bus */
int p4080ds_mdio_read(struct mii_bus *bus, int port_addr, int dev_addr,
			int regnum)
{
	struct p4080ds_mdio *priv = bus->priv;
	int i;

	/* Write the GPIO regs to select this bus */
	for (i = 0; i < priv->num_gpios; i++)
		gpio_set_value(priv->gpios[i], priv->gpio_values[i]);

	return priv->real_bus->read(priv->real_bus, port_addr, dev_addr,
					regnum);
}


/* Reset the MIIM registers, and wait for the bus to free */
static int p4080ds_mdio_reset(struct mii_bus *bus)
{
	struct p4080ds_mdio *priv = bus->priv;

	mutex_lock(&bus->mdio_lock);
	priv->real_bus->reset(priv->real_bus);
	mutex_unlock(&bus->mdio_lock);

	return 0;
}


static int p4080ds_mdio_probe(struct platform_device *ofdev)
{
	struct device_node *np = ofdev->dev.of_node;
	struct mii_bus *new_bus;
	struct p4080ds_mdio *priv;
	struct device_node *mdio;
	struct platform_device *ofmdiodev;
	int i;
	const u32 *val;
	u32 gpio_mask;
	int gpio;
	int err = 0;

	new_bus = mdiobus_alloc();
	if (NULL == new_bus)
		return -ENOMEM;

	new_bus->name = "Freescale P4080DS MDIO Bus",
	new_bus->read = &p4080ds_mdio_read,
	new_bus->write = &p4080ds_mdio_write,
	new_bus->reset = &p4080ds_mdio_reset,

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		err = -ENOMEM;
		goto err_priv_alloc;
	}

	new_bus->priv = priv;

	mdio = of_parse_phandle(np, "fsl,mdio-handle", 0);

	if (mdio == NULL) {
		printk(KERN_ERR "Could not find real MDIO bus for %s\n",
			new_bus->id);
		err = -ENODEV;
		goto err_no_mdio_node;
	}

	ofmdiodev = of_find_device_by_node(mdio);

	if (!ofmdiodev) {
		printk(KERN_ERR "No of_device for MDIO node %s\n", mdio->name);
		err = -ENODEV;
		goto err_no_mdio_dev;
	}

	of_node_put(mdio);

	priv->real_bus = dev_get_drvdata(&ofmdiodev->dev);

	if (!priv->real_bus) {
		printk(KERN_ERR "The MDIO bus has no ofdev!\n");
		err = -ENODEV;
		goto err_no_ofdev;
	}

	new_bus->irq = kcalloc(PHY_MAX_ADDR, sizeof(int), GFP_KERNEL);

	if (NULL == new_bus->irq) {
		err = -ENOMEM;
		goto err_irq_alloc;
	}

	for (i = 0; i < PHY_MAX_ADDR; i++)
		new_bus->irq[i] = PHY_POLL;

	new_bus->parent = &ofdev->dev;
	dev_set_drvdata(&ofdev->dev, new_bus);

	/* Get the number of GPIO pins specified for this MDIO node */
	priv->num_gpios = of_gpio_count(mdio);
	if (priv->num_gpios == 0) {
		printk(KERN_ERR "No GPIO pins specified for MDIO node %s\n",
			mdio->name);
		err = -ENODEV;
		goto err_no_gpios;
	}

	priv->gpios = kzalloc(sizeof(unsigned int) * priv->num_gpios,
			      GFP_KERNEL);
	if (!priv->gpios) {
		err = -ENOMEM;
		goto err_gpios_alloc;
	}

	priv->gpio_values = kzalloc(sizeof(int) * priv->num_gpios, GFP_KERNEL);
	if (!priv->gpio_values) {
		err = -ENOMEM;
		goto err_gpio_values_alloc;
	}

	/* Grab the value to write to the GPIO */
	val = of_get_property(np, "fsl,muxval", NULL);

	if (!val) {
		printk(KERN_ERR "No mux value found for %s\n", np->full_name);
		err = -ENODEV;
		goto err_get_muxval;
	}
	if (*val >= (1 << priv->num_gpios)) {
		printk(KERN_ERR "Mux value for %s is out of range: [0 - %d]\n",
				np->full_name, (1 << priv->num_gpios) - 1);
		err = -EINVAL;
		goto err_get_muxval;
	}

	for (i = 0; i < priv->num_gpios; i++) {
		/* Based on GPIO pin specified in DT, get a pin number relative
		 * to the GPIO controller's dynamically assigned base */
		gpio = of_get_gpio_flags(mdio, i, NULL);
		if (gpio < 0) {
			printk(KERN_ERR "Failed to get GPIO pin value\n");
			err = gpio;
			goto err_gpio_get;
		}
		priv->gpios[i] = gpio;
		if (!gpio_is_valid(priv->gpios[i])) {
			printk(KERN_ERR "Invalid GPIO pin value: %d\n",
				priv->gpios[i]);
			err = -EINVAL;
			goto err_gpio_get;
		}

		/* Set the value to be written to the GPIO pin
		 * in order to select this mdio bus */
		gpio_mask = 1 << (priv->num_gpios - i - 1);
		priv->gpio_values[i] = !!(*val & gpio_mask);

		/* Request the pin (check that pin number is valid & in range).
		 * More than one P4080 MDIO bus will use these pins; if already
		 * requested, continue without error */
		err = gpio_request(priv->gpios[i], NULL);
		if ((err < 0) && (err != -EBUSY)) {
			printk(KERN_ERR "Failed to reserve GPIO pin %d\n",
					priv->gpios[i]);
			goto err_gpio_req;
		}

		/* Set the direction to output (we will write to this pin).
		 * Should be already set from u-boot, but make sure
		 * it's correct nonetheless */
		err = gpio_direction_output(priv->gpios[i], 0);
		if (err < 0) {
			printk(KERN_ERR "Failed to configure direction for "
					"GPIO pin %d\n", priv->gpios[i]);
			goto err_gpio_dir;
		}
	}

	sprintf(new_bus->id, "%s@%d", np->name, *val);

	err = of_mdiobus_register(new_bus, np);

	if (err) {
		printk(KERN_ERR "%s: Cannot register as MDIO bus\n",
				new_bus->name);
		goto err_registration;
	}

	return 0;

err_registration:
err_gpio_dir:
	for (i = 0; i < priv->num_gpios; i++)
		gpio_free(priv->gpios[i]);
err_gpio_req:
err_gpio_get:
err_get_muxval:
	kfree(priv->gpio_values);
err_gpio_values_alloc:
	kfree(priv->gpios);
err_gpios_alloc:
err_no_gpios:
	kfree(new_bus->irq);
err_irq_alloc:
err_no_ofdev:
err_no_mdio_dev:
err_no_mdio_node:
	kfree(priv);
err_priv_alloc:
	kfree(new_bus);

	return err;
}

static int p4080ds_mdio_remove(struct platform_device *ofdev)
{
	struct device *device = &ofdev->dev;
	struct mii_bus *bus = dev_get_drvdata(device);
	struct p4080ds_mdio *priv = bus->priv;
	int i;

	for (i = 0; i < priv->num_gpios; i++)
		gpio_free(priv->gpios[i]);
	kfree(priv->gpios);
	kfree(priv->gpio_values);

	mdiobus_unregister(bus);

	dev_set_drvdata(device, NULL);

	bus->priv = NULL;
	mdiobus_free(bus);

	return 0;
}

static struct of_device_id p4080ds_mdio_match[] = {
	{
		.compatible = "fsl,p4080ds-mdio",
	},
	{
		.compatible = "fsl,p4080ds-xmdio",
	},
	{}
};

static struct platform_driver p4080ds_mdio_driver = {
	.driver = {
		.name = "p4080ds_mdio",
		.of_match_table = p4080ds_mdio_match,
	},
	.probe = p4080ds_mdio_probe,
	.remove = p4080ds_mdio_remove,
};

int __init p4080ds_mdio_init(void)
{
	return platform_driver_register(&p4080ds_mdio_driver);
}

void p4080ds_mdio_exit(void)
{
	platform_driver_unregister(&p4080ds_mdio_driver);
}
subsys_initcall_sync(p4080ds_mdio_init);
module_exit(p4080ds_mdio_exit);
