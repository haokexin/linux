/*
 * Copyright (C) 2010 Wind River Systems
 *
 * I2C master mode controller driver, used in LSI ACP34xx board.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/io.h>

#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/of_i2c.h>

#include "i2c-acp.h"

struct acp_i2c {
	struct device		*dev;
	struct i2c_adapter	adap;
	void __iomem *base;
	void __iomem *timer_base;
};

static int acp_i2c_xfer_one_read(struct acp_i2c *i2c, struct i2c_msg *msg)
{
	struct xfer_config xfer_config = {0};
	struct recv_config recv_config = {0};
	struct recv_status recv_status = {0};
	struct clk_config clk_config = {0};
	struct hold_setup_clk start_condition = {0};
	struct hold_setup_clk stop_condition = {0};
	struct hold_setup_clk data_condition = {0};
	unsigned *v;

	uint32_t data = 0;
	uint8_t *cp = (uint8_t *)&data;
	int timeout = 2;
	int i;

	if (msg->len > 4)
		msg->len = 4;

	v = (unsigned *)&clk_config;
	clk_config.pclk_low = CFG_CLK_CONFIG_SCL_LOW_RD;
	clk_config.pclk_high = CFG_CLK_CONFIG_SCL_HIGH;
	writel(*v, i2c->base + CLK_CONFIG);
	dev_dbg(i2c->dev, "read clk config: %x\n", *v);

	v = (unsigned *)&start_condition;
	start_condition.pclk_setup = CFG_START_SETUP_PERIOD;
	start_condition.pclk_hold = CFG_START_HOLD_PERIOD;
	writel(*v, i2c->base + START_SETUP_HOLD_CONFIG);
	dev_dbg(i2c->dev, "start cfg: %x\n", *v);

	v = (unsigned *)&stop_condition;
	stop_condition.pclk_setup = CFG_STOP_SETUP_PERIOD;
	stop_condition.pclk_hold = CFG_STOP_HOLD_PERIOD;
	writel(*v, i2c->base + STOP_SETUP_HOLD_CONFIG);
	dev_dbg(i2c->dev, "stop cfg: %x\n", *v);

	v = (unsigned *)&data_condition;
	data_condition.pclk_setup = CFG_DATA_SETUP_PERIOD;
	data_condition.pclk_hold = CFG_DATA_HOLD_PERIOD;
	writel(*v, i2c->base + DATA_SETUP_HOLD_CONFIG);
	dev_dbg(i2c->dev, "data cfg: %x\n", *v);

	writel(msg->addr, i2c->base + SLAVE_ADDRESS);

	writel(0x0, i2c->base + INTR_ENABLE);

	v = (unsigned *)&recv_config;
	recv_config.ready = 1;
	recv_config.nbytes = 4;
	writel(*v, i2c->base + RECV_CONFIG);
	dev_dbg(i2c->dev, "recv cfg: %x\n", *v);

	v = (unsigned *)&xfer_config;
	xfer_config.ready = 1;
	xfer_config.nbytes = 0;
	xfer_config.master_mode_active = 1;
	xfer_config.command = 1;
	writel(*v, i2c->base + XFER_CONFIG);
	dev_dbg(i2c->dev, "xfer cfg: %x\n", *v);

	do {
		v = (unsigned *)&recv_status;
		*v = readl(i2c->base + RECV_STATUS);
		dev_dbg(i2c->dev, "recv status: %x\n", *v);
		if (recv_status.error) {
			dev_warn(i2c->dev, "acp i2c read error\n");
			return -EIO;
		} else if (recv_status.done)
			break;

		udelay(200);
	} while (--timeout);

	if (timeout <= 0) {
		dev_warn(i2c->dev, "acp i2c read timeout\n");
		return -EIO;
	}

	data = __raw_readl(i2c->base + RECV_DATA0);

	dev_dbg(i2c->dev, "nbytes done: %d\n", recv_status.nbytes_done);
	for (i = 0; i < recv_status.nbytes_done; i++)
		msg->buf[recv_status.nbytes_done - i - 1] = cp[i];

	msg->len = recv_status.nbytes_done;
	return 0;
}

static int acp_i2c_xfer_one_write(struct acp_i2c *i2c, struct i2c_msg *msg)
{
	struct xfer_config xfer_config = {0};
	struct xfer_status xfer_status = {0};
	struct clk_config clk_config = {0};
	struct hold_setup_clk start_condition = {0};
	struct hold_setup_clk stop_condition = {0};
	struct hold_setup_clk data_condition = {0};
	unsigned *v;

	uint32_t buf[2] = {0};
	uint8_t *cp = (uint8_t *)buf;
	int timeout = 2;
	int i;

	BUG_ON(msg->len > 8);

	for (i = 0; i < msg->len; i++)
		cp[msg->len - i - 1] = msg->buf[i];

	v = (unsigned *)&clk_config;
	clk_config.pclk_low = CFG_CLK_CONFIG_SCL_LOW;
	clk_config.pclk_high = CFG_CLK_CONFIG_SCL_HIGH;
	writel(*v, i2c->base + CLK_CONFIG);
	dev_dbg(i2c->dev, "write clk config: %x\n", *v);

	v = (unsigned *)&start_condition;
	start_condition.pclk_setup = CFG_START_SETUP_PERIOD;
	start_condition.pclk_hold = CFG_START_HOLD_PERIOD;
	writel(*v, i2c->base + START_SETUP_HOLD_CONFIG);
	dev_dbg(i2c->dev, "start cfg: %x\n", *v);

	v = (unsigned *)&stop_condition;
	stop_condition.pclk_setup = CFG_STOP_SETUP_PERIOD;
	stop_condition.pclk_hold = CFG_STOP_HOLD_PERIOD;
	writel(*v, i2c->base + STOP_SETUP_HOLD_CONFIG);
	dev_dbg(i2c->dev, "stop cfg: %x\n", *v);

	v = (unsigned *)&data_condition;
	data_condition.pclk_setup = CFG_DATA_SETUP_PERIOD;
	data_condition.pclk_hold = CFG_DATA_HOLD_PERIOD;
	writel(*v, i2c->base + DATA_SETUP_HOLD_CONFIG);
	dev_dbg(i2c->dev, "data cfg: %x\n", *v);

	writel(msg->addr, i2c->base + SLAVE_ADDRESS);

	__raw_writel(buf[0], i2c->base + XFER_DATA0);
	__raw_writel(buf[1], i2c->base + XFER_DATA1);

	writel(0x0, i2c->base + INTR_ENABLE);

	v = (unsigned *)&xfer_config;
	xfer_config.ready = 1;
	xfer_config.nbytes = msg->len;
	xfer_config.master_mode_active = 1;
	writel(*v, i2c->base + XFER_CONFIG);
	dev_dbg(i2c->dev, "xfer cfg: %x\n", *v);

	do {
		v = (unsigned *)&xfer_status;
		*v = readl(i2c->base + XFER_STATUS);
		dev_dbg(i2c->dev, "xfer status: %x\n", *v);
		if (xfer_status.error) {
			dev_warn(i2c->dev, "acp i2c write error\n");
			return -EIO;
		} else if (xfer_status.done)
			break;

		udelay(200);
	} while (--timeout);

	if (timeout <= 0) {
		dev_warn(i2c->dev, "acp i2c write timeout\n");
		return -EIO;
	}

	return 0;
}

static int acp_i2c_xfer_one(struct acp_i2c *i2c, struct i2c_msg *msg)
{
	struct xfer_config xfer_config = {0};
	unsigned *v;
	int ret;

	v = (unsigned *)&xfer_config;
	xfer_config.zero_all = 1;
	xfer_config.master_mode_active = 1;
	writel(*v, i2c->base + XFER_CONFIG);

	if (msg->flags & I2C_M_RD)
		ret = acp_i2c_xfer_one_read(i2c, msg);
	else
		ret = acp_i2c_xfer_one_write(i2c, msg);

	return ret;
}

static int acp_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct acp_i2c *i2c = (struct acp_i2c *)adap->algo_data;
	int ret = 0;
	int i;

	for (i = 0; i < num; i++) {
		ret = acp_i2c_xfer_one(i2c, &msgs[i]);
		if (ret != 0)
			break;
	}

	if (ret == 0)
		ret = num;

	return ret;
}

static u32 acp_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_QUICK |
		I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA;
}

static const struct i2c_algorithm acp_i2c_algorithm = {
	.master_xfer		= acp_i2c_xfer,
	.functionality		= acp_i2c_func,
};

static int acp_i2c_probe(struct platform_device *dev)
{
	struct acp_i2c *i2c;
	int ret;

	i2c = kzalloc(sizeof(struct acp_i2c), GFP_KERNEL);
	if (!i2c) {
		dev_err(&dev->dev, "no memory for state\n");
		return -ENOMEM;
	}

	strlcpy(i2c->adap.name, "acp-i2c", sizeof(i2c->adap.name));
	i2c->adap.owner   = THIS_MODULE;
	i2c->adap.algo    = &acp_i2c_algorithm;
	i2c->adap.retries = 2;
	i2c->adap.class   = I2C_CLASS_HWMON | I2C_CLASS_SPD;

	i2c->dev = &dev->dev;

	i2c->adap.algo_data = i2c;
	i2c->adap.dev.parent = &dev->dev;
	i2c->adap.dev.of_node = of_node_get(dev->dev.of_node);
	i2c->adap.nr = 0;

	ret = i2c_add_numbered_adapter(&i2c->adap);
	if (ret < 0) {
		dev_err(&dev->dev, "failed to add bus to i2c core\n");
		goto err;
	}

	i2c->base = of_iomap(dev->dev.of_node, 0);
	if (!i2c->base) {
		ret = -ENOMEM;
		goto err;
	}

	i2c->timer_base = of_iomap(dev->dev.of_node, 1);
	if (!i2c->timer_base) {
		ret = -ENOMEM;
		goto err;
	}

	/* enable the i2c clock */
	writel(0x0, i2c->timer_base + I2C_TIMER_CONTROL);
	writel(0x10, i2c->timer_base + I2C_TIMER_LOAD);
	writel(0xc0, i2c->timer_base + I2C_TIMER_CONTROL);

	dev_set_drvdata(&dev->dev, i2c);

	dev_info(&dev->dev, "%s: ACP I2C adapter\n", dev_name(&i2c->adap.dev));

	of_i2c_register_devices(&i2c->adap);

	return 0;
err:
	kfree(i2c);
	return ret;
}

static int acp_i2c_remove(struct platform_device *dev)
{
	struct acp_i2c *i2c = dev_get_drvdata(&dev->dev);

	/* disable the i2c clock */
	writel(0x0, i2c->timer_base + I2C_TIMER_CONTROL);

	i2c_del_adapter(&i2c->adap);
	kfree(i2c);

	return 0;
}

static struct of_device_id acp_i2c_match[] = {
	{
		.compatible = "acp-i2c",
	},
	{ /* end of list */ },
};

static struct platform_driver acp_i2c_driver = {
	.driver = {
		.name = "acp-i2c",
		.owner = THIS_MODULE,
		.of_match_table = acp_i2c_match,
	},
	.probe		= acp_i2c_probe,
	.remove		= acp_i2c_remove,
};

module_platform_driver(acp_i2c_driver);

MODULE_DESCRIPTION("ACP I2C Bus driver");
MODULE_AUTHOR("Wu Fei <fei.wu@windriver.com>");
MODULE_LICENSE("GPL");
