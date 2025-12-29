// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 * This file contains proprietary information that is the sole intellectual
 * property of Black Sesame Technologies, Inc. and its affiliates.
 * No portions of this material may be reproduced in any
 * form without the written permission of:
 * Black Sesame Technologies, Inc. and its affiliates
 * 2255 Martin Ave. Suite D
 * Santa Clara, CA 95050
 * Copyright @2016: all right reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#include "bstccgx.h"
#include <linux/of_gpio.h>

static int ccgx_set_usb_role(struct ccgx_data *ctx, enum usb_role role)
{
	if (!ctx->role_sw)
		return 0;

	return usb_role_switch_set_role(ctx->role_sw, role);
}

void ccgx_unregister_partner(struct ccgx_data *ctx)
{
	if (ctx->partner) {
		typec_unregister_partner(ctx->partner);
		ctx->partner = NULL;
	}
}

static int ccgx_get_power_mode(struct ccgx_data *ctx, u32 status)
{
	return ((status & STATUS_PORT_PD) ? TYPEC_PWR_MODE_PD : TYPEC_PWR_MODE_USB);
}

static int ccgx_get_typec_accessory(struct ccgx_data *ctx)
{
	return TYPEC_ACCESSORY_NONE;
}

static struct usb_pd_identity *ccgx_get_pd_identity(struct ccgx_data *ctx)
{
	return NULL;
}

static int ccgx_connect(struct ccgx_data *ctx, u32 status)
{
	struct typec_partner_desc desc;
	enum typec_pwr_opmode mode;
	enum typec_orientation orientation;
	enum typec_data_role data_role;
//	enum typec_role power_role;
//	enum typec_role vconn_role;

	mode = ccgx_get_power_mode(ctx, status);

	desc.usb_pd = mode == TYPEC_PWR_MODE_PD;
	desc.accessory = ccgx_get_typec_accessory(ctx);
	desc.identity = NULL;

	if (desc.usb_pd)
		desc.identity = ccgx_get_pd_identity(ctx);

	//vconn_role = (status & STATUS_VCONN_ROLE) ? TYPEC_SOURCE : TYPEC_SINK;
	data_role = (status & STATUS_DATA_ROLE) ? TYPEC_HOST : TYPEC_DEVICE;
	//power_role = (status & STATUS_PWR_ROLE) ? TYPEC_SOURCE : TYPEC_SINK;
	orientation = (status & STATUS_ORIENTATION) ? TYPEC_ORIENTATION_REVERSE :
	    TYPEC_ORIENTATION_NORMAL;
	typec_set_orientation(ctx->port, orientation);
	typec_set_pwr_opmode(ctx->port, mode);
	//typec_set_pwr_role(ctx->port, power_role);
	//typec_set_vconn_role(ctx->port, vconn_role);
	typec_set_data_role(ctx->port, data_role);
	if (data_role == TYPEC_HOST)
		ccgx_set_usb_role(ctx, USB_ROLE_HOST);
	else
		ccgx_set_usb_role(ctx, USB_ROLE_DEVICE);
	if (ctx->partner == NULL) {
		ctx->partner = typec_register_partner(ctx->port, &desc);
		if (IS_ERR(ctx->partner))
			return PTR_ERR(ctx->partner);
	}
	if (desc.identity)
		typec_partner_set_identity(ctx->partner);
	return 0;
}

static int ccgx_disconnect(struct ccgx_data *ctx, u32 status)
{
	enum typec_orientation orientation;
	enum typec_data_role data_role;
	//enum typec_role power_role;
	//enum typec_role vconn_role;

	ccgx_unregister_partner(ctx);
	//vconn_role = (status & STATUS_VCONN_ROLE) ? TYPEC_SOURCE : TYPEC_SINK;
	data_role = (status & STATUS_DATA_ROLE) ? TYPEC_HOST : TYPEC_DEVICE;
	//power_role = (status & STATUS_PWR_ROLE) ? TYPEC_SOURCE : TYPEC_SINK;
	orientation =
	    (status & STATUS_ORIENTATION) ? TYPEC_ORIENTATION_REVERSE :
	    TYPEC_ORIENTATION_NORMAL;
	typec_set_orientation(ctx->port, orientation);
	typec_set_pwr_opmode(ctx->port, TYPEC_PWR_MODE_USB);
	//typec_set_pwr_role(ctx->port, power_role);
	//typec_set_vconn_role(ctx->port, vconn_role);
	ccgx_set_usb_role(ctx, USB_ROLE_NONE);
	typec_set_data_role(ctx->port, data_role);
	return 0;
}

static void ccgx_work_func(struct work_struct *work)
{
	int ret;
	u32 int_change = 0;	/* Interrupt change */
	u32 int_status = 0;	/* interrupt status */
	u32 data = 0;
	struct ccgx_data *ctx = container_of(work, struct ccgx_data, work);

	ret = regmap_read(ctx->regmap, USER_INT_REG, &int_change);
	//printk("%s %d  int_change%x\n",__FUNCTION__,__LINE__,int_change);

	if (int_change & PORT0_INT_STATUS) {
		ret = regmap_read(ctx->regmap, PORT0_STATUS, &data);
		//printk("%s %d  data%x\n",__FUNCTION__,__LINE__,data);
		if (ctx->irq_test_data)
			data = ctx->irq_test_data;
		if (data & STATUS_CONNECTED)
			ret = ccgx_connect(ctx, data);
		else
			ret = ccgx_disconnect(ctx, data);
		int_status |= PORT0_INT_STATUS;
	}

	regmap_write(ctx->regmap, USER_INT_REG, int_status);
}

static irqreturn_t ccgx_intr_isr(int irq, void *data)
{
	struct ccgx_data *ctx = (struct ccgx_data *)data;
	//printk("%s %d  ctx->irq%x\n",__FUNCTION__,__LINE__,irq);
	queue_work(ctx->workqueue, &ctx->work);

	return IRQ_HANDLED;
}

static int ccgx_dr_swap(struct typec_port *port, enum typec_data_role role)
{
	struct ccgx_data *ctx = typec_get_drvdata(port);

	dev_err(ctx->dev, "dr_swap not support.\n");
	return 0;
}

static const struct typec_operations ccgx_ops = {
	.dr_set = ccgx_dr_swap
};

static int ccgx_typec_port_probe(struct ccgx_data *ctx, struct device *dev)
{
	struct typec_capability *cap = &ctx->caps;
	struct fwnode_handle *fwnode;
	const char *buf;
	int ret, role;

	fwnode = device_get_named_child_node(dev, "connector");
	if (!fwnode) {
		dev_err(dev, "connector not found\n");
		return -EINVAL;
	}
	ctx->role_sw = fwnode_usb_role_switch_get(fwnode);
	if (IS_ERR_OR_NULL(ctx->role_sw)) {
		dev_err(dev, "USB role switch not found.\n");
		ctx->role_sw = NULL;
	}
	ret = fwnode_property_read_string(fwnode, "power-role", &buf);
	if (!ret) {
		role = typec_find_port_power_role(buf);
		if (role < 0)
			role = TYPEC_PORT_SRC;
	} else {
		role = TYPEC_PORT_SRC;
	}
	cap->type = role;
	cap->ops = &ccgx_ops;
	cap->driver_data = ctx;
	ret = fwnode_property_read_string(fwnode, "data-role", &buf);
	if (!ret) {
		role = typec_find_port_data_role(buf);
		if (role < 0)
			role = TYPEC_PORT_DFP;
	} else {
		role = TYPEC_PORT_DFP;
	}
	cap->data = role;

	ret = fwnode_property_read_string(fwnode, "try-power-role", &buf);
	if (!ret) {
		role = typec_find_power_role(buf);
		if (role < 0)
			role = TYPEC_PORT_DFP;
	} else {
		role = TYPEC_PORT_DFP;
	}
	cap->prefer_role = role;
	cap->fwnode = fwnode;
	ctx->port = typec_register_port(dev, cap);
	if (IS_ERR(ctx->port)) {
		ret = PTR_ERR(ctx->port);
		ctx->port = NULL;
		dev_err(dev, "Failed to register type c port %d\n", ret);
		return ret;
	}
	return 0;
}

static int __maybe_unused ccgx_runtime_pm_suspend(struct device *dev)
{
	struct ccgx_data *ctx = dev_get_drvdata(dev);
	//ccgx_partner_unregister_altmode(ctx);
	ccgx_unregister_partner(ctx);
	return 0;
}

static int __maybe_unused ccgx_runtime_pm_resume(struct device *dev)
{
	struct ccgx_data *ctx = dev_get_drvdata(dev);

	queue_work(ctx->workqueue, &ctx->work);
	return 0;
}

static const struct dev_pm_ops ccgx_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	    SET_RUNTIME_PM_OPS(ccgx_runtime_pm_suspend,
			       ccgx_runtime_pm_resume, NULL)
};

static void ccgx_get_gpio_irq(struct ccgx_data *ctx)
{
	struct device *dev = ctx->dev;
	int intc_gpio;
	enum of_gpio_flags gpio_flags = 0;
	int ret = 0;

	intc_gpio = of_get_named_gpio_flags(dev->of_node, "int-gpio", 0,
					    &gpio_flags);
	if (gpio_is_valid(intc_gpio)) {
		unsigned int flags = GPIOF_IN;
		struct gpio_desc *gpiod = NULL;

		flags |= (gpio_flags & OF_GPIO_ACTIVE_LOW ? GPIOF_ACTIVE_LOW : 0);

		ret = devm_gpio_request_one(dev, intc_gpio, flags, dev_name(dev));
		//ret = devm_gpio_request(dev, intc_gpio, dev_name(dev));
		if (ret) {
			dev_err(dev, "could not request gpio,ret %d\n", ret);
			return;
		}
		gpiod = gpio_to_desc(intc_gpio);
		if (gpiod == NULL) {
			dev_err(dev, "gpio_to_desc fail,ret %d\n", ret);
			return;
		}

		ret = gpiod_to_irq(gpiod);
		if (ret > 0) {
			ctx->irq = ret;
		} else {
			dev_err(dev, "__gpio_to_irq(%d) fail, ret %d\n",
				intc_gpio, ret);
		}

	} else {
		dev_err(dev, "could not get valid gpio.\n");
	}
}

static ssize_t do_flash_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t n)
{
	struct ccgx_data *ctx = i2c_get_clientdata(to_i2c_client(dev));
	int flash;

	if (kstrtoint(buf, 10, &flash))
		return -EINVAL;

	if (!flash)
		return n;

	if (ctx->update_fw_flag) {
		dev_info(dev, "fw is updating\n");
		return n;
	}

	ctx->update_fw_flag = flash;
	schedule_work(&ctx->update_work);
	return n;
}

static DEVICE_ATTR_WO(do_flash);

static ssize_t do_irq_test_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t n)
{
	struct ccgx_data *ctx = i2c_get_clientdata(to_i2c_client(dev));

	if (kstrtouint(buf, 16, &(ctx->irq_test_data)))
		return -EINVAL;
	schedule_work(&ctx->work);
	return n;
}

static DEVICE_ATTR_WO(do_irq_test);

static struct attribute *bst_ccg_attrs[] = {
	&dev_attr_do_flash.attr,
	&dev_attr_do_irq_test.attr,
	NULL,
};

static const struct attribute_group bst_ccg_group = {
	.attrs = bst_ccg_attrs,
};

static const struct regmap_config i2c_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.reg_format_endian = REGMAP_ENDIAN_LITTLE,
};

static int ccgx_i2c_probe(struct i2c_client *client)
{
	struct ccgx_data *ctx;
	struct device *dev = &client->dev;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(dev, "i2c not support SMBUS_I2C_BLOCK\n");
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->tcpc_client = client;
	i2c_set_clientdata(client, ctx);

	ctx->regmap = devm_regmap_init_i2c(client, &i2c_config);
	if (IS_ERR(ctx->regmap))
		return PTR_ERR(ctx->regmap);

	mutex_init(&ctx->lock);
	ctx->dev = dev;
	if (dev->of_node == NULL)
		dev->of_node = of_find_compatible_node(NULL, NULL, "bst,ccgx");

	ret = ccgx_typec_port_probe(ctx, dev);
	if (ret) {
		dev_err(dev, "fail to probe typec property.\n");
		ret = -ENODEV;
		return ret;
	}

	ctx->irq = client->irq;

	if (!ctx->irq)
		ccgx_get_gpio_irq(ctx);

	if (!ctx->irq) {
		dev_err(dev, "fail to get interrupt IRQ\n");
		ret = -EINVAL;
		goto free_typec_port;
	}

	INIT_WORK(&ctx->update_work, ccgx_update_firmware);
	INIT_WORK(&ctx->work, ccgx_work_func);
	ctx->workqueue = alloc_workqueue("ccgx_work",
					 WQ_FREEZABLE | WQ_MEM_RECLAIM, 1);
	if (!ctx->workqueue) {
		dev_err(dev, "fail to create work queue\n");
		ret = -ENOMEM;
		goto free_typec_port;
	}
	ret = devm_request_any_context_irq(dev, ctx->irq,
					ccgx_intr_isr, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					     "ccgx-intp", ctx);
	//ret = devm_request_threaded_irq(dev, ctx->irq,
	//				NULL, ccgx_intr_isr,
	//				IRQF_TRIGGER_FALLING |
	//				IRQF_ONESHOT, "ccgx-intp", ctx);
	if (ret) {
		dev_err(dev, "fail to request irq\n");
		goto free_wq;
	}
	queue_work(ctx->workqueue, &ctx->work);

	ret = sysfs_create_group(&dev->kobj, &bst_ccg_group);
	if (ret < 0)
		dev_err(dev, "Can't register sysfs attr group: %d\n", ret);
	pm_runtime_enable(dev);

	return 0;

free_wq:
	destroy_workqueue(ctx->workqueue);

free_typec_port:
	typec_unregister_port(ctx->port);
	//ccgx_port_unregister_altmodes(ctx->port_amode);

	return ret;
}

static void ccgx_i2c_remove(struct i2c_client *client)
{
	struct ccgx_data *ctx = i2c_get_clientdata(client);
	struct device *dev = &client->dev;

	sysfs_remove_group(&dev->kobj, &bst_ccg_group);

	disable_irq(ctx->irq);
	//cancel_work_sync(&ctx->work);
	//cancel_work_sync(&ctx->update_work);
	if (ctx->workqueue)
		flush_workqueue(ctx->workqueue);

	//ccgx_partner_unregister_altmode(ctx);
	ccgx_unregister_partner(ctx);

	if (ctx->workqueue)
		destroy_workqueue(ctx->workqueue);
	if (ctx->role_sw)
		usb_role_switch_put(ctx->role_sw);
	if (ctx->port)
		typec_unregister_port(ctx->port);
}

static const struct of_device_id bst_match_table[] = {
	{.compatible = "bst,ccgx", },
	{ },
};

static struct i2c_driver ccgx_driver = {
	.driver = {
		   .name = "usb_bst_ccgx",
		   .of_match_table = bst_match_table,
		   .pm = &ccgx_pm_ops,
		    },
	.probe_new = ccgx_i2c_probe,
	.remove = ccgx_i2c_remove,
};

module_i2c_driver(ccgx_driver);

MODULE_DESCRIPTION("CCG7D USB Type-C PD driver");
MODULE_AUTHOR("BST Ltd.");
MODULE_LICENSE("GPL v2");

