// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C)2024Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/reset.h>

#define ENABLE_BURST 1

#define BST_I2CQ_INTERNAL_CLK 600000000

enum i2cq_cmdmode {
	CMD_MODE_QUEUE,
	CMD_MODE_FAST,
};

enum i2cq_cmddir {
	CMD_DIR_WRITE,
	CMD_DIR_READ,
};

enum i2cq_addr_format {
	CMD_ADDR_8BIT_DATA_8BIT,
	CMD_ADDR_8BIT_DATA_16BIT,
	CMD_ADDR_16BIT_DATA_8BIT,
	CMD_ADDR_16BIT_DATA_16BIT,
};

#define I2CQ0_BASE_ADDR (0x5203E000)
#define I2CQ1_BASE_ADDR (0x5203E800)
#define I2CQ2_BASE_ADDR (0x5203F000)

#define BST_I2CQ_CMDH_FAST_MODE	  13
#define BST_I2CQ_CMDH_DIRECTION	  10
#define BST_I2CQ_CMDH_FORMAT	  8
#define BST_I2CQ_CMDH_SLAVE_DEVID 0

#define BST_I2CQ_CMDL_REG_ADDR 16
#define BST_I2CQ_CMDL_REG_DATA 0

#define BST_I2CQ_HOST_CMD_RECV_DATH  0x020
#define BST_I2CQ_HOST_CMD_RECV_DATL  0x024
#define BST_I2CQ_HOST_CMD_RECV_READY 0x028
#define BST_I2CQ_MS1_SP		     0x09c
#define BST_I2CQ_MS1_CONFIG	     0x0a0
#define BST_I2CQ_MS1_BURST_EN	     0x0a4
#define BST_I2CQ_MCU_RDY_MASK	     0x0bc
#define BST_I2CQ_HOST_RDY_MASK	     0x0c0
#define BST_I2CQ_HOST_Debug_REG0     0x0d8
#define BST_I2CQ_HOST_Debug_REG1     0x0dc
#define BST_I2CQ_CTRL78		     0x0f8
#define BST_I2CQ_MCU_INT_CRTL	     0x104
#define BST_I2CQ_HOST_INT_CRTL	     0x114
#define BST_I2CQ_HOST_INT_CLR	     0x118
#define BST_I2CQ_EMPT_THRD	     0x100
#define BST_I2CQ_HOST_INT_STATUS     0x11c

#define BST_I2CQ_HOST_HOST_RDBACK_BUFL 0x400
#define BST_I2CQ_HOST_HOST_RDBACK_BUFH 0x500
#define BST_I2CQ_HOST_RDBACK_BUF_VLD_L 0x580
#define BST_I2CQ_HOST_RDBACK_BUF_VLD_H 0x584

#define BST_I2CQ_INT_HOST_READ_BACK_DONE     BIT(0)
#define BST_I2CQ_INT_HOST_QUEUE_FULL_EMPTY   BIT(1)
#define BST_I2CQ_INT_HOST_QUEUE_ALMOST_EMPTY BIT(2)
#define BST_I2CQ_INT_HOST_MASTER_TIMEOUT     BIT(3)
#define BST_I2CQ_INT_HOST_WRITE_FULL_ERROR   BIT(4)
#define BST_I2CQ_INT_HOST_READ_EMPTY_ERROR   BIT(5)
#define BST_I2CQ_INT_HOST_ACK_ERROR	     BIT(6)

enum i2cq_opmode {
	MODE_MCU_ONLY,
	MODE_HOST_ONLY,
	MODE_HYBIRD,
};

enum i2c_state {
	STATE_IDLE,
	STATE_START,
	STATE_READ,
	STATE_WRITE
};

struct bst_i2cq {
	struct completion completion;

	int irq;
	int idx;
	struct device *dev;
	void __iomem *base;
	struct clk *pclk;
	u32 pclkrate;
	u32 speed;
	u32 timeout_ms;
	struct i2c_msg *readbk_msg;
	u16 readbk_reg;
	u16 readbk_devid;
	enum i2c_state state;
	enum i2cq_opmode mode;
	struct i2c_adapter adapter;
	struct reset_control *rstc;
};

static int reset_i2cq(struct bst_i2cq *i2cq);

static void bst_i2cq_hw_init(struct bst_i2cq *i2cq)
{
	u32 speed_reg;

	reset_i2cq(i2cq);
	udelay(5);
	speed_reg = BST_I2CQ_INTERNAL_CLK / i2cq->speed / 64;
	dev_err(i2cq->dev, "real speed = %d\n",
		BST_I2CQ_INTERNAL_CLK / 64 / speed_reg);

	writel_relaxed(speed_reg & 0xff, i2cq->base + BST_I2CQ_MS1_SP);

#if ENABLE_BURST
	writel_relaxed(1, i2cq->base + BST_I2CQ_MS1_BURST_EN);
#else
	writel_relaxed(0, i2cq->base + BST_I2CQ_MS1_BURST_EN);
#endif
	if (i2cq->mode == MODE_MCU_ONLY) {
		writel_relaxed(0, i2cq->base + BST_I2CQ_MCU_RDY_MASK);
		writel_relaxed(0x3, i2cq->base + BST_I2CQ_HOST_RDY_MASK);
	} else if (i2cq->mode == MODE_HOST_ONLY) {
		writel_relaxed(0x3, i2cq->base + BST_I2CQ_MCU_RDY_MASK);
		writel_relaxed(0x0, i2cq->base + BST_I2CQ_HOST_RDY_MASK);
	}
	writel_relaxed(0x18, i2cq->base + BST_I2CQ_MS1_CONFIG);
	//config interrupt
	//use almost empty
	writel_relaxed(0xce, i2cq->base + BST_I2CQ_HOST_INT_CRTL);
	writel_relaxed(0xce, i2cq->base + BST_I2CQ_MCU_INT_CRTL);

	//support clock stretch
	writel_relaxed(0x8010f, i2cq->base + BST_I2CQ_CTRL78);
	//writel_relaxed(0x180, i2cq->base + BST_I2CQ_EMPT_THRD);
}

static int bst_i2cq_doxfer(struct bst_i2cq *i2cq, struct i2c_msg *msgs, int num)
{
	u32 val, reg;
	unsigned long timeout;
	int i;
	u16 slave_id, reg_val, data_val, direction, reg_data_sz;

	reinit_completion(&i2cq->completion);

	slave_id = (msgs->addr << 1);
	// dev_err(i2cq->dev, "bst_i2cq_doxfer num = %d addr=0x%x!\n", num, slave_id);

	if (num == 1) {
		direction = CMD_DIR_WRITE;
		//reg_data_sz = (msgs -> flags & 0xC ) >> 2;

		switch (msgs->len) {
		case 2:
			reg_data_sz = CMD_ADDR_8BIT_DATA_8BIT;
			reg_val = msgs->buf[0];
			data_val = msgs->buf[1];
			break;
		case 3:
			reg_data_sz = CMD_ADDR_16BIT_DATA_8BIT;
			reg_val = (msgs->buf[0] << 8) | msgs->buf[1];
			data_val = msgs->buf[2];
			break;
		case 4:
			reg_data_sz = CMD_ADDR_16BIT_DATA_16BIT;
			reg_val = (msgs->buf[0] << 8) | msgs->buf[1];
			data_val = (msgs->buf[2] << 8) | msgs->buf[3];
			break;
		default:
			dev_err(i2cq->dev, "msg format error!\n");
			return -EAGAIN;
		}
		// dev_err(i2cq->dev, "bst_i2cq_doxfer num = %d reg_val=0x%x data_val=0x%x!\n", num, reg_val, data_val);
	}

	if ((num == 2)) {
		data_val = 0;
		//reg_data_sz = (msgs[0].flags & 0xC ) >> 2 ;
		// dev_err(i2cq->dev, "bst_i2cq_doxfer num = %d msgs -> len=%d!\n", num, msgs[0].len);
		direction = CMD_DIR_READ;

		switch (msgs[0].len) {
		case 1:
			if (msgs[1].len == 1)
				reg_data_sz = CMD_ADDR_8BIT_DATA_8BIT;
			else
				reg_data_sz = CMD_ADDR_8BIT_DATA_16BIT;
			reg_val = msgs[0].buf[0];
			break;
		case 2:
			if (msgs[1].len == 1)
				reg_data_sz = CMD_ADDR_16BIT_DATA_8BIT;
			else
				reg_data_sz = CMD_ADDR_16BIT_DATA_16BIT;
			reg_val = (msgs[0].buf[0] << 8) | msgs[0].buf[1];
			break;
		default:
			dev_err(i2cq->dev, "msg format error!\n");
			return -EAGAIN;
		}

		if (msgs[1].flags & I2C_M_RD) {
			i2cq->readbk_msg = &msgs[1];
			i2cq->readbk_devid = slave_id;
			i2cq->readbk_reg = reg_val;
		}

		// dev_err(i2cq->dev, "bst_i2cq_doxfer num = %d reg_val=0x%x data_val=0x%x!\n", num, reg_val, data_val);
	}

	for (i = 0; i < 3; i++) {
		val = readl_relaxed(i2cq->base + BST_I2CQ_HOST_CMD_RECV_READY);
		if (val == 0x1) {
			reg = CMD_MODE_QUEUE << BST_I2CQ_CMDH_FAST_MODE |
			      direction << BST_I2CQ_CMDH_DIRECTION |
			      reg_data_sz << BST_I2CQ_CMDH_FORMAT | slave_id;
			// dev_err(i2cq->dev, "bst_i2cq_doxfer write reg = 0x%x reg_val=0x%x !\n", I2CQ2_BASE_ADDR + BST_I2CQ_HOST_CMD_RECV_DATH, reg);
			writel_relaxed(
				reg, i2cq->base + BST_I2CQ_HOST_CMD_RECV_DATH);
			reg = reg_val << BST_I2CQ_CMDL_REG_ADDR | data_val;
			// dev_err(i2cq->dev, "bst_i2cq_doxfer write reg = 0x%x reg_val=0x%x !\n", I2CQ2_BASE_ADDR + BST_I2CQ_HOST_CMD_RECV_DATL, reg);
			writel_relaxed(
				reg, i2cq->base + BST_I2CQ_HOST_CMD_RECV_DATL);
			break;
		}

		mdelay(5);
	}

	if (direction == CMD_DIR_WRITE)
		return num;

	timeout = wait_for_completion_timeout(
		&i2cq->completion, msecs_to_jiffies(i2cq->timeout_ms));
	if (timeout == 0) {
		dev_err(i2cq->dev, "timeout\n");
		return -EAGAIN;
	}

	return num;
}

static irqreturn_t bst_i2cq_isr(int irq, void *dev_id)
{
	struct bst_i2cq *i2cq = dev_id;

	unsigned int int_state, regval;
	unsigned int clr_reg = 0;
	unsigned int fifo_state, fifo_addr, fifo_val, fifo_valh;
	int i, index;

	int_state = readl_relaxed(i2cq->base + BST_I2CQ_HOST_INT_STATUS);
	// dev_err(i2cq->dev, "int sta:0x%08x\n", int_state);

	if (int_state & BST_I2CQ_INT_HOST_QUEUE_FULL_EMPTY) {
		writel_relaxed(0xce, i2cq->base + BST_I2CQ_HOST_INT_CRTL);
		clr_reg |= 0x08;
		goto out;
	}

	if (int_state & BST_I2CQ_INT_HOST_ACK_ERROR) {
		regval = readl_relaxed(i2cq->base + 0xd4);
		dev_err(i2cq->dev, "ack error d4:0x%x\n", regval);
		regval = readl_relaxed(i2cq->base + 0xd8);
		dev_err(i2cq->dev, "ack error d8:0x%x\n", regval);
		regval = readl_relaxed(i2cq->base + 0xdc);
		dev_err(i2cq->dev, "ack error dc:0x%x\n", regval);
		regval = readl_relaxed(i2cq->base + 0x120);
		dev_err(i2cq->dev, "ack error 120:0x%x\n", regval);
		clr_reg |= 0x04;
		goto need_reset;
	}

	if (int_state & BST_I2CQ_INT_HOST_MASTER_TIMEOUT) {
		regval = readl_relaxed(i2cq->base + 0xd4);
		dev_err(i2cq->dev, "host master timeout error d4:0x%x\n",
			regval);
		regval = readl_relaxed(i2cq->base + 0xd8);
		dev_err(i2cq->dev, "host master timeout error d8:0x%x\n",
			regval);
		regval = readl_relaxed(i2cq->base + 0xdc);
		dev_err(i2cq->dev, "host master timeout error dc:0x%x\n",
			regval);
		clr_reg |= 0x02;
		goto out;
	}

	if (int_state & BST_I2CQ_INT_HOST_READ_EMPTY_ERROR) {
		dev_err(i2cq->dev, "read empty error\n");
		clr_reg |= 0x40;
		goto out;
	}

	if (int_state & BST_I2CQ_INT_HOST_WRITE_FULL_ERROR) {
		dev_err(i2cq->dev, "write full error\n");
		clr_reg |= 0x20;
		goto out;
	}

	if (int_state & BST_I2CQ_INT_HOST_READ_BACK_DONE) {
		// dev_err(i2cq->dev, "read back done\n");
		clr_reg |= 0x01;
		fifo_state = readl_relaxed(i2cq->base +
					   BST_I2CQ_HOST_RDBACK_BUF_VLD_L);
		index = ffs(fifo_state);
		if (index > 0)
			fifo_addr = BST_I2CQ_HOST_HOST_RDBACK_BUFL +
				    (index - 1) * 0x8;
		else {
			fifo_state = readl_relaxed(
				i2cq->base + BST_I2CQ_HOST_RDBACK_BUF_VLD_H);
			index = ffs(fifo_state);
			if (index > 0)
				fifo_addr = BST_I2CQ_HOST_HOST_RDBACK_BUFH +
					    (index - 1) * 0x8;
			else
				dev_err(i2cq->dev, "read i2c error!!\n");
		}
		fifo_val = readl_relaxed(i2cq->base + fifo_addr);
		fifo_valh = readl_relaxed(i2cq->base + fifo_addr + 0x4);
		if (((fifo_valh & 0xff) == i2cq->readbk_devid) &&
		    (((fifo_val >> 16) & 0xffff) == i2cq->readbk_reg)) {
			for (i = 0; i < i2cq->readbk_msg->len; i++)
				i2cq->readbk_msg->buf[i] =
					(fifo_val >> (i * 8)) & 0xff;
		}
		goto out;
	}

out:
	if (clr_reg != 0) {
		writel_relaxed(clr_reg, i2cq->base + BST_I2CQ_HOST_INT_CLR);
		writel_relaxed(0x00, i2cq->base + BST_I2CQ_HOST_INT_CLR);
	}
	complete(&i2cq->completion);
	return IRQ_HANDLED;

need_reset:
	bst_i2cq_hw_init(i2cq);
	complete(&i2cq->completion);
	return IRQ_HANDLED;
}

static int bst_i2cq_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
			 int num)
{
	struct bst_i2cq *i2cq;
	int retry;
	int ret;

	i2cq = i2c_get_adapdata(adap);
	i2cq->timeout_ms = 50;

	// dev_err(i2cq->dev, "calculated timeout %d ms\n", i2cq->timeout_ms);

	for (retry = 0; retry <= adap->retries; retry++) {
		ret = bst_i2cq_doxfer(i2cq, msgs, num);
		if (ret != -EAGAIN)
			return ret;

		dev_err(i2cq->dev, "Retrying transmission (%d)\n", retry);
	}
	return -EIO;
}

static u32 bst_i2cq_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C;
}

static const struct i2c_algorithm bst_i2cq_algo = {
	.master_xfer = bst_i2cq_xfer,
	.functionality = bst_i2cq_functionality,
};

static const struct i2c_adapter bst_i2cq_ops = {
	.owner = THIS_MODULE,
	.name = "bst_i2cq-adapter",
	.algo = &bst_i2cq_algo,
	.retries = 1,
};

#define C1200_ISP_CORE_TOP_CTRL0 0x52030000
#define C1200_ISP_CORE_TOP_CTRL1 0x52030004

static int reset_i2cq(struct bst_i2cq *i2cq)
{
	uint32_t status;
	void *isp_top_ctl1 = NULL;

	isp_top_ctl1 = ioremap(C1200_ISP_CORE_TOP_CTRL1, 0x4);
	if (isp_top_ctl1 == NULL) {
		pr_err("ioremap(C1200_ISP_CORE_TOP_CTRL1 failed\n");
		return -1;
	}

	status = readl_relaxed(isp_top_ctl1);
	status &= (~(0x1 << (25 + i2cq->idx)));
	udelay(5);
	status |= (0x1 << (25 + i2cq->idx));
	writel_relaxed(status, isp_top_ctl1);
	pr_err("reset DONE\n");
	return 0;
}

static int enable_i2cq(struct bst_i2cq *i2cq)
{
	uint32_t status;
	void *isp_top_ctl0 = NULL;

	isp_top_ctl0 = ioremap(C1200_ISP_CORE_TOP_CTRL0, 0x4);

	if (isp_top_ctl0 == NULL) {
		pr_err("ioremap(C1200_ISP_CORE_TOP_CTRL0 failed\n");
		return -1;
	}
	status = readl_relaxed(isp_top_ctl0);
	status |= (0x1 << (29 + i2cq->idx));
	writel_relaxed(status, isp_top_ctl0);
	pr_err("clk enable done\n");

	//reset_i2cq(i2cq);
	return 0;
}

static int disable_i2cq(struct bst_i2cq *i2cq)
{
	uint32_t status;
	void *isp_top_ctl0 = NULL;
	void *isp_top_ctl1 = NULL;

	isp_top_ctl0 = ioremap(C1200_ISP_CORE_TOP_CTRL0, 0x4);

	if (isp_top_ctl0 == NULL) {
		pr_err("ioremap(C1200_ISP_CORE_TOP_CTRL0 failed\n");
		return -1;
	}
	status = readl_relaxed(isp_top_ctl0);
	status &= (~(0x1 << (29 + i2cq->idx)));
	writel_relaxed(status, isp_top_ctl0);
	pr_err("clk disable done\n");
	isp_top_ctl1 = ioremap(C1200_ISP_CORE_TOP_CTRL1, 0x4);

	if (isp_top_ctl1 == NULL) {
		pr_err("ioremap(C1200_ISP_CORE_TOP_CTRL1 failed\n");
		return -1;
	}

	status = readl_relaxed(isp_top_ctl1);
	status &= (~(0x1 << (25 + i2cq->idx)));
	writel_relaxed(status, isp_top_ctl1);
	pr_err("reset DONE\n");
	return 0;
}

static void bst_i2cq_close(struct bst_i2cq *i2cq)
{
	u32 val;

	val = readl_relaxed(i2cq->base + BST_I2CQ_HOST_Debug_REG0);
	while ((val & 0xff) != 0x09) {
		mdelay(1);
		val = readl_relaxed(i2cq->base + BST_I2CQ_HOST_Debug_REG0);
	}
	val = readl_relaxed(i2cq->base + BST_I2CQ_HOST_Debug_REG0);
	while ((val & 0x01500001) != 0x1) {
		mdelay(1);
		val = readl_relaxed(i2cq->base + BST_I2CQ_HOST_Debug_REG0);
	}
	disable_i2cq(i2cq);
}

static int bst_i2cq_probe(struct platform_device *pdev)
{
	struct bst_i2cq *i2cq;
	u32 bus_speed;
	int ret;
	struct resource *res;
	resource_size_t phy_base;

	i2cq = devm_kzalloc(&pdev->dev, sizeof(*i2cq), GFP_KERNEL);
	if (!i2cq)
		return -ENOMEM;

	device_property_read_u32(&pdev->dev, "clock-frequency", &bus_speed);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	phy_base = res->start;
	if (phy_base == I2CQ2_BASE_ADDR)
		i2cq->idx = 2;
	else if (phy_base == I2CQ1_BASE_ADDR)
		i2cq->idx = 1;
	else
		i2cq->idx = 0;

	i2cq->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(i2cq->base))
		return PTR_ERR(i2cq->base);

	i2cq->rstc = devm_reset_control_get_shared(&pdev->dev, "i2cq-reset");
	if (IS_ERR(i2cq->rstc)) {
		dev_err(&pdev->dev, "reset is not defined\n");
		return -EINVAL;
	}
	// reset_control_deassert(i2cq->rstc);

	//todo : enable clk & reset
	enable_i2cq(i2cq);

	i2cq->irq = platform_get_irq(pdev, 0);
	if (i2cq->irq < 0)
		return -ENODEV;

	ret = devm_request_irq(&pdev->dev, i2cq->irq, bst_i2cq_isr, 0,
			       dev_name(&pdev->dev), i2cq);
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot claim IRQ %d\n", i2cq->irq);
		return ret;
	}

	i2cq->state = STATE_IDLE;
	i2cq->dev = &pdev->dev;
	i2cq->adapter = bst_i2cq_ops;
	i2c_set_adapdata(&i2cq->adapter, i2cq);
	i2cq->adapter.dev.parent = &pdev->dev;
	i2cq->adapter.dev.of_node = pdev->dev.of_node;
	//i2cq->adapter.nr = pdev->id;
	init_completion(&i2cq->completion);

	i2cq->speed = bus_speed;
	i2cq->mode = MODE_HYBIRD;

	if (of_property_read_s32(pdev->dev.of_node, "i2c-nr",
				 &(i2cq->adapter.nr))) {
		dev_err(&pdev->dev, "no i2c-nr\n");
	}

	bst_i2cq_hw_init(i2cq);

	ret = i2c_add_numbered_adapter(&i2cq->adapter);
	if (ret) {
		dev_err(&pdev->dev, "failed to add bus to i2c core\n");
		return ret;
	}

	platform_set_drvdata(pdev, i2cq);

	dev_err(&pdev->dev, "%s: bst_i2cq adapter\n",
		dev_name(&i2cq->adapter.dev));

	return 0;
}

static int bst_i2cq_remove(struct platform_device *pdev)
{
	struct bst_i2cq *i2cq = platform_get_drvdata(pdev);

	i2c_del_adapter(&i2cq->adapter);

	bst_i2cq_close(i2cq);
	return 0;
};

static void bst_i2cq_shutdown(struct platform_device *pdev)
{
	struct bst_i2cq *i2cq = platform_get_drvdata(pdev);

	i2cq = platform_get_drvdata(pdev);

	/* We reset all here to support reboot */
	// reset_control_assert(i2cq->rstc);
}

static const struct of_device_id bst_i2cq_dt_ids[] = {
	{ .compatible = "bst,isp-i2cq" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, bst_i2cq_dt_ids);

static struct platform_driver bst_i2cq_driver = {
	.probe = bst_i2cq_probe,
	.remove = bst_i2cq_remove,
	.shutdown = bst_i2cq_shutdown,
	.driver = {
		.name = "bst_i2cq",
		.of_match_table = of_match_ptr(bst_i2cq_dt_ids),
	},
};
module_platform_driver(bst_i2cq_driver);

MODULE_AUTHOR("BST Ltd.");
MODULE_DESCRIPTION("BST ISP I2C-Q Driver");
MODULE_LICENSE("GPL v2");
