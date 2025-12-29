// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <dt-bindings/i2c/bst-i2cq.h>
#include <dt-bindings/media/bst-isp.h>

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
#include <linux/bst_samphore.h>
#endif

/* Core top register definitions */
#define R_TOP_CTRL0		(0x000)
#define R_TOP_CTRL1		(0x004)
#define R_TOP_UID		(0x08C)
/* I2CQ register definitions */
#define R_HOST_CMD_RECV_DATH	(0x020)
#define R_HOST_CMD_RECV_DATL	(0x024)
#define R_HOST_CMD_RECV_READY	(0x028)
#define R_MS1_SP		(0x09C)
#define R_MS1_CONFIG		(0x0A0)
#define R_MS1_BURST_EN		(0x0A4)
#define R_MCU_RDY_MASK		(0x0BC)
#define R_HOST_RDY_MASK		(0x0C0)
#define R_HOST_DEBUG_REG0	(0x0D8)
#define R_HOST_DEBUG_REG1	(0x0DC)
#define R_CTRL78		(0x0F8)
#define R_EMPT_THRD		(0x100)
#define R_MCU_INT_CTRL		(0x104)
#define R_MCU_INT_STATUS	(0x10C)
#define R_MCU_ERROR_STATUS	(0x110)
#define R_HOST_INT_CTRL		(0x114)
#define R_HOST_INT_CLR		(0x118)
#define R_HOST_INT_STATUS	(0x11C)
#define R_HOST_TIMEOUT_STATUS	(0x120)
#define R_HOST_HOST_RDBACK_BUFL (0x400)
#define R_HOST_HOST_RDBACK_BUFH (0x500)
#define R_HOST_RDBACK_BUF_VLD_L (0x580)
#define R_HOST_RDBACK_BUF_VLD_H (0x584)

/* Int status bits */
#define BIT_HOST_INT_READ_BACK_DONE	BIT(0)
#define BIT_HOST_INT_QUEUE_FULL_EMPTY	BIT(1)
#define BIT_HOST_INT_QUEUE_ALMOST_EMPTY BIT(2)
#define BIT_HOST_INT_MASTER_TIMEOUT	BIT(3)
#define BIT_HOST_INT_WRITE_FULL_ERROR	BIT(4)
#define BIT_HOST_INT_READ_EMPTY_ERROR	BIT(5)
#define BIT_HOST_INT_ACK_ERROR		BIT(6)
/* Software defined error bits */
#define BIT_SW_EBUSY			BIT(30)
#define BIT_SW_EINVAL			BIT(31)

/* Int clear bits */
#define BIT_HOST_CLR_READ_BACK_DONE	BIT(0)
#define BIT_HOST_CLR_QUEUE_FULL_EMPTY	BIT(3)
#define BIT_HOST_CLR_QUEUE_ALMOST_EMPTY BIT(4)
#define BIT_HOST_CLR_MASTER_TIMEOUT	BIT(1)
#define BIT_HOST_CLR_WRITE_FULL_ERROR	BIT(5)
#define BIT_HOST_CLR_READ_EMPTY_ERROR	BIT(6)
#define BIT_HOST_CLR_ACK_ERROR		BIT(2)

/* Field offset */
#define OFF_CMDH_FAST_MODE   (13)
#define OFF_CMDH_DIRECTION   (10)
#define OFF_CMDH_FORMAT	     (8)
#define OFF_CMDH_SLAVE_DEVID (0)
#define OFF_CMDL_REG_ADDR    (16)
#define OFF_CMDL_REG_DATA    (0)

/* Default parameters */
#define POLL_IDLE_US	  (2000)
#define CLEAR_INT_US	  (2)
#define MAX_CLOCK_STRETCH (255)
#define MAX_SPIKE	  (255)
#define XFER_TIMEOUT_MSEC (50)
#define XFER_RETRIES	  (3)

enum cmd_dir {
	CMD_DIR_WRITE,
	CMD_DIR_READ,
};

enum addr_data_format {
	A1B_D1B,
	A1B_D2B,
	A2B_D1B,
	A2B_D2B,
};

struct hw_params {
	u32 sclk;
	u32 min_speed;
	u32 max_speed;
	u32 freq_div_factor;
	u32 host_int_ctrl;
	u32 mcu_int_ctrl;
};

struct bst_i2cq {
	struct device *dev;
	struct i2c_adapter adapter;
	spinlock_t lock;
	u32 role;
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	u32 sem_master;
	u32 sem_bank;
	u32 sem_id;
	struct bst_samphore *hwlock;
	u32 uid;
#endif

	struct completion wait_comp;
	bool atomic;
	u32 poll_comp;

	/* Critical area begin */
	struct i2c_msg *msgs;
	int nmsg;
	u16 curr_dev;
	u16 curr_reg;
	/* Critical area end */
	u32 reset_times;

	/* NOTE: Update index carefully, we use it to know:
	 * The last message is READ or WRITE
	 * Whether the last message be executed successfully
	 */
	u32 wr_index;
	u32 rd_index;
	u32 error;

	/* Properties */
	u32 id;
	s32 i2c_nr;
	u32 speed;
	u32 clock_stretch;
	u32 spike;
	u32 ctrl_mode;
	u32 cmd_mode;
	u32 timeout;
	u32 retries;

	/* Hardware control */
	void __iomem *core_base;
	void __iomem *i2cq_base;
	struct reset_control *rstc;
	int irq;
	const struct hw_params *params;
};

static void enable_clk(struct bst_i2cq *i2cq)
{
	u32 val;

	val = readl_relaxed(i2cq->core_base + R_TOP_CTRL0);
	val |= 1 << (i2cq->id + 29);
	writel_relaxed(val, i2cq->core_base + R_TOP_CTRL0);
}

#ifndef CONFIG_VIDEO_BST_ISP_MULTI_OS
static void disable_clk(struct bst_i2cq *i2cq)
{
	u32 val;

	val = readl_relaxed(i2cq->core_base + R_TOP_CTRL0);
	val &= ~(1 << (i2cq->id + 29));
	writel_relaxed(val, i2cq->core_base + R_TOP_CTRL0);
}
#endif

static void reset(struct bst_i2cq *i2cq)
{
	u32 val;

	val = readl_relaxed(i2cq->core_base + R_TOP_CTRL1);
	val &= ~(1 << (i2cq->id + 25));
	writel_relaxed(val, i2cq->core_base + R_TOP_CTRL1);
	udelay(CLEAR_INT_US);

	val |= 1 << (i2cq->id + 25);
	writel_relaxed(val, i2cq->core_base + R_TOP_CTRL1);
	udelay(CLEAR_INT_US);
	++i2cq->reset_times;
}

static void init_hw(struct bst_i2cq *i2cq)
{
	u32 val;
	const struct hw_params *params;

	reset(i2cq);
	/* After reset, RDY_MASK is 3, CMD_RECV_READY is 1,
	 * Disable access first to avoid MCU's disturbance
	 */
	writel_relaxed(0x0, i2cq->i2cq_base + R_MCU_RDY_MASK);
	writel_relaxed(0x0, i2cq->i2cq_base + R_HOST_RDY_MASK);

	params = i2cq->params;
	val = params->sclk / i2cq->speed / params->freq_div_factor;
	writel_relaxed(val & 0xFF, i2cq->i2cq_base + R_MS1_SP);

	/* Enable burst mode */
	writel_relaxed(1, i2cq->i2cq_base + R_MS1_BURST_EN);

	/* NOTE: Disable master send ack for high byte when read word data */
	// writel_relaxed(0x18, i2cq->i2cq_base + R_MS1_CONFIG);
	writel_relaxed(params->mcu_int_ctrl, i2cq->i2cq_base + R_MCU_INT_CTRL);

	/* Set clock stretch */
	val = readl_relaxed(i2cq->i2cq_base + R_CTRL78);
	val &= ~(0xFF << 12);
	val |= (i2cq->clock_stretch & 0xFF) << 12;
	/* Set spike length */
	if (i2cq->spike <= MAX_SPIKE) {
		val &= ~(0xFF << 4);
		val |= (i2cq->spike & 0xFF) << 4;
	}
	writel_relaxed(val, i2cq->i2cq_base + R_CTRL78);

	/* Enable access */
	if (i2cq->ctrl_mode == CTRL_MODE_MCU) {
		writel_relaxed(0x3, i2cq->i2cq_base + R_MCU_RDY_MASK);
		writel_relaxed(0x0, i2cq->i2cq_base + R_HOST_RDY_MASK);
	} else if (i2cq->ctrl_mode == CTRL_MODE_HOST) {
		writel_relaxed(0x0, i2cq->i2cq_base + R_MCU_RDY_MASK);
		writel_relaxed(0x3, i2cq->i2cq_base + R_HOST_RDY_MASK);
	} else {
		writel_relaxed(0x3, i2cq->i2cq_base + R_MCU_RDY_MASK);
		writel_relaxed(0x3, i2cq->i2cq_base + R_HOST_RDY_MASK);
	}
}

static bool poll_idle(struct bst_i2cq *i2cq, unsigned int us)
{
	u32 val;
	unsigned int i;
	struct device *dev = i2cq->dev;

	for (i = 0; i < us; ++i) {
		val = readl_relaxed(i2cq->i2cq_base + R_HOST_DEBUG_REG0);
		if ((val & 0xFF) == 0x09)
			break;
		udelay(1);
	}

	if (i == us) {
		dev_dbg(dev, "Timeout to wait idle state: 0x%08X\n", val);
		return false;
	}

	for (i = 0; i < us; ++i) {
		val = readl_relaxed(i2cq->i2cq_base + R_HOST_DEBUG_REG1);
		if ((val & 0x01500001) == 0x1)
			break;
		udelay(1);
	}
	if (i == us) {
		dev_dbg(dev, "Timeout to wait empty fifo: 0x%08X\n", val);
		return false;
	}

	return true;
}

#ifndef CONFIG_VIDEO_BST_ISP_MULTI_OS
static void close_hw(struct bst_i2cq *i2cq)
{
	poll_idle(i2cq, POLL_IDLE_US);

	disable_clk(i2cq);
}
#endif

static void read_data(struct bst_i2cq *i2cq)
{
	u32 vld, buf_addr, vall, valh;
	int i, pos;
	unsigned long flags;
	struct i2c_msg *rmsg;

	vld = readl_relaxed(i2cq->i2cq_base + R_HOST_RDBACK_BUF_VLD_L);
	pos = ffs(vld);
	dev_dbg(i2cq->dev, "%s: vldl: 0x%08X, pos: %d\n", __func__, vld, pos);
	if (pos > 0)
		buf_addr = R_HOST_HOST_RDBACK_BUFL + (pos - 1) * 0x8;
	else {
		vld = readl_relaxed(i2cq->i2cq_base + R_HOST_RDBACK_BUF_VLD_H);
		pos = ffs(vld);
		dev_dbg(i2cq->dev, "%s: vldh: 0x%08X, pos: %d\n", __func__, vld,
			pos);
		if (pos > 0)
			buf_addr = R_HOST_HOST_RDBACK_BUFH + (pos - 1) * 0x8;
		else {
			dev_err(i2cq->dev, "Invalid read back buffer\n");
			i2cq->error |= BIT_HOST_INT_READ_EMPTY_ERROR;
			return;
		}
	}

	vall = readl_relaxed(i2cq->i2cq_base + buf_addr);
	valh = readl_relaxed(i2cq->i2cq_base + buf_addr + 0x4);
	dev_dbg(i2cq->dev,
		"%s: valh: 0x%02X, dev: 0x%02X, vall: 0x%08X, reg: 0x%02X, index: %d\n",
		__func__, valh, i2cq->curr_dev, vall, i2cq->curr_reg,
		i2cq->rd_index);

	spin_lock_irqsave(&i2cq->lock, flags);
	rmsg = i2cq->msgs + i2cq->rd_index;
	if (((valh & 0xFF) == i2cq->curr_dev) &&
	    (((vall >> 16) & 0xFFFF) == i2cq->curr_reg) && (i2cq->msgs != NULL))
		for (i = 0; i < rmsg->len; ++i)
			rmsg->buf[i] = (vall >> (i * 8)) & 0xFF;
	else
		i2cq->error |= BIT_HOST_INT_READ_EMPTY_ERROR;
	spin_unlock_irqrestore(&i2cq->lock, flags);
}

static irqreturn_t isr_handler(int irq, void *dev_id)
{
	struct bst_i2cq *i2cq = dev_id;
	struct device *dev = i2cq->dev;
	u32 int_status;
	u32 int_clr;
	bool reset;

	int_clr = 0;
	reset = false;
	int_status = readl_relaxed(i2cq->i2cq_base + R_HOST_INT_STATUS);
	dev_dbg(dev, "%s: int status: 0x%08X\n", __func__, int_status);
	if (int_status & BIT_HOST_INT_ACK_ERROR) {
		dev_dbg(dev,
			"ACK Error: 0x0D8: 0x%08X, 0x0DC: 0x%08X, 0x120: 0x%08X\n",
			readl_relaxed(i2cq->i2cq_base + 0x0D8),
			readl_relaxed(i2cq->i2cq_base + 0x0DC),
			readl_relaxed(i2cq->i2cq_base + 0x120));
		int_clr |= BIT_HOST_CLR_ACK_ERROR;
		i2cq->error |= BIT_HOST_INT_ACK_ERROR;
		reset = true;
		goto done;
	}

	if (int_status & BIT_HOST_INT_READ_EMPTY_ERROR) {
		dev_dbg(dev, "Read Empty Error\n");
		int_clr |= BIT_HOST_CLR_READ_EMPTY_ERROR;
		i2cq->error |= BIT_HOST_INT_READ_EMPTY_ERROR;
		goto done;
	}

	if (int_status & BIT_HOST_INT_WRITE_FULL_ERROR) {
		dev_dbg(dev, "Write Full Error\n");
		int_clr |= BIT_HOST_CLR_WRITE_FULL_ERROR;
		i2cq->error |= BIT_HOST_INT_WRITE_FULL_ERROR;
		goto done;
	}

	if (int_status & BIT_HOST_INT_MASTER_TIMEOUT) {
		dev_dbg(dev,
			"Master Timeout Error: 0x0D8: 0x%08X, 0x0DC: 0x%08X, 0x120: 0x%08X\n",
			readl_relaxed(i2cq->i2cq_base + 0x0D8),
			readl_relaxed(i2cq->i2cq_base + 0x0DC),
			readl_relaxed(i2cq->i2cq_base + 0x120));
		int_clr |= BIT_HOST_CLR_MASTER_TIMEOUT;
		i2cq->error |= BIT_HOST_INT_MASTER_TIMEOUT;
		goto done;
	}

	if (int_status & BIT_HOST_INT_QUEUE_ALMOST_EMPTY) {
		int_clr |= BIT_HOST_CLR_QUEUE_ALMOST_EMPTY;
		goto done;
	}

	if (int_status & BIT_HOST_INT_QUEUE_FULL_EMPTY) {
		u32 val;

		/* Mask full empty interrupt to avoid reentry */
		writel_relaxed(i2cq->params->host_int_ctrl |
				       BIT_HOST_INT_QUEUE_FULL_EMPTY,
			       i2cq->i2cq_base + R_HOST_INT_CTRL);
		int_clr |= BIT_HOST_CLR_QUEUE_FULL_EMPTY;
		/* NOTE: ignore full empty int for READ OP if misreported */
		if (i2cq->rd_index > i2cq->wr_index) {
			writel_relaxed(int_clr,
				       i2cq->i2cq_base + R_HOST_INT_CLR);
			udelay(CLEAR_INT_US);
			writel_relaxed(0, i2cq->i2cq_base + R_HOST_INT_CLR);
			return IRQ_HANDLED;
		}
		poll_idle(i2cq, POLL_IDLE_US);
		val = readl_relaxed(i2cq->i2cq_base + R_HOST_TIMEOUT_STATUS);
		dev_dbg(dev, "HOST_TIMEOUT_STATUS: 0x%08X\n", val);
		if (val & 0x1F00) {
			i2cq->error |= BIT_HOST_INT_ACK_ERROR;
			reset = true;
		}

		goto done;
	}

	if (int_status & BIT_HOST_INT_READ_BACK_DONE) {
		read_data(i2cq);
		int_clr |= BIT_HOST_CLR_READ_BACK_DONE;
	}

done:
	if (int_clr != 0) {
		writel_relaxed(int_clr, i2cq->i2cq_base + R_HOST_INT_CLR);
		udelay(CLEAR_INT_US);
		writel_relaxed(0, i2cq->i2cq_base + R_HOST_INT_CLR);
	}

	if (reset) {
		unsigned long flags;

		spin_lock_irqsave(&i2cq->lock, flags);
		init_hw(i2cq);
		spin_unlock_irqrestore(&i2cq->lock, flags);
	}

	if (!i2cq->atomic)
		complete(&i2cq->wait_comp);
	else
		WRITE_ONCE(i2cq->poll_comp, 1);

	return IRQ_HANDLED;
}

/* clang-format off */
/*
 *  |-----------------------------------------------------------------------------------------------------------|
 *  | field     | write bb  | read bb   | write wb  | read wb   | write ww  | read ww   | write bw  | read bw   |
 *  |-----------|-----------|-----------|-----------|-----------|-----------|-----------|-----------|-----------|
 *  | nmsg      | 1         | 2         | 1         | 2         | 1         | 2         | 1         | 2         |
 *  | msg0.addr | dev addr  | dev addr  | dev addr  | dev addr  | dev addr  | dev addr  | dev addr  | dev addr  |
 *  | msg0.flags| 0         | 0         | 0         | 0         | 0         | 0         | 0         | 0         |
 *  | msg0.len  | 2         | 1         | 3         | 2         | 4         | 2         | 3         | 1         |
 *  | msg0.buf0 | reg       | reg       | regH      | regH      | regH      | regH      | reg       | reg       |
 *  | msg0.buf1 | val       | X         | regL      | regL      | regL      | regL      | valH      | X         |
 *  | msg0.buf2 | X         | X         | val       | X         | valH      | X         | valL      | X         |
 *  | msg0.buf3 | X         | X         | X         | X         | valL      | X         | X         | X         |
 *  | msg1.addr | X         | dev addr  | X         | dev addr  | X         | dev addr  | X         | dev addr  |
 *  | msg1.flags| X         | I2C_M_RD  | X         | I2C_M_RD  | X         | I2C_M_RD  | X         | I2C_M_RD  |
 *  | msg1.len  | X         | 1         | X         | 1         | X         | 2         | X         | 2         |
 *  | msg1.buf0 | X         | Out       | X         | Out       | X         | OutH      | X         | OutH      |
 *  | msg1.buf1 | X         | X         | X         | X         | X         | OutL      | X         | OutL      |
 *  |-----------------------------------------------------------------------------------------------------------|
 *
 * NOTE: writebw is conflict with writewb, and will not be supported.
 */
/* clang-format on */

static int xfer_next(struct bst_i2cq *i2cq)
{
	u16 dev, reg, val, dir, fmt;
	struct i2c_msg *wmsg;
	struct i2c_msg *rmsg;
	u32 ready;
	u32 cmd;
	u32 mcu_int;
	u32 mcu_err;
	u32 host_int;
	u32 host_err;
	unsigned long flags;

	if (i2cq->wr_index >= i2cq->nmsg)
		return -ENOENT;

	wmsg = i2cq->msgs + i2cq->wr_index;
	i2cq->rd_index = i2cq->wr_index;
	if (wmsg->flags & I2C_M_RD) {
		dev_err(i2cq->dev, "Invalid message %d, flags: 0x%04X\n",
			i2cq->wr_index, wmsg->flags);
		return -EINVAL;
	}

	dev = wmsg->addr << 1;
	val = 0;
	switch (wmsg->len) {
	case 4:
		dir = CMD_DIR_WRITE;
		fmt = A2B_D2B;
		reg = (wmsg->buf[0] << 8) | wmsg->buf[1];
		val = (wmsg->buf[2] << 8) | wmsg->buf[3];
		break;
	case 3:
		dir = CMD_DIR_WRITE;
		fmt = A2B_D1B;
		reg = (wmsg->buf[0] << 8) | wmsg->buf[1];
		val = wmsg->buf[2];
		break;
	case 2:
		rmsg = wmsg + 1;
		if (i2cq->wr_index + 1 >= i2cq->nmsg ||
		    !(rmsg->flags & I2C_M_RD)) {
			dir = CMD_DIR_WRITE;
			fmt = A1B_D1B;
			reg = wmsg->buf[0];
			val = wmsg->buf[1];
			break;
		}
		dir = CMD_DIR_READ;
		reg = (wmsg->buf[0] << 8) | wmsg->buf[1];
		if (rmsg->len == 1)
			fmt = A2B_D1B;
		else if (rmsg->len == 2)
			fmt = A2B_D2B;
		else {
			dev_err(i2cq->dev,
				"Invalid read word reg message %d, len: %d, flags: 0x%04X\n",
				i2cq->wr_index + 1, rmsg->len, rmsg->flags);
			return -EINVAL;
		}
		++i2cq->rd_index;
		break;
	case 1:
		rmsg = wmsg + 1;
		if (i2cq->wr_index + 1 >= i2cq->nmsg ||
		    !(rmsg->flags & I2C_M_RD)) {
			dev_err(i2cq->dev, "Invalid read message %d\n",
				i2cq->wr_index + 1);
			return -EINVAL;
		}
		dir = CMD_DIR_READ;
		reg = wmsg->buf[0];
		if (rmsg->len == 1)
			fmt = A1B_D1B;
		else if (rmsg->len == 2)
			fmt = A1B_D2B;
		else {
			dev_err(i2cq->dev,
				"Invalid read byte reg message %d, len: %d, flags: 0x%04X\n",
				i2cq->wr_index + 1, rmsg->len, rmsg->flags);
			return -EINVAL;
		}
		++i2cq->rd_index;
		break;
	default:
		dev_err(i2cq->dev, "Invalid message %d, len: %d\n",
			i2cq->wr_index, wmsg->len);
		return -EINVAL;
	}

	spin_lock_irqsave(&i2cq->lock, flags);
	i2cq->curr_dev = dev;
	i2cq->curr_reg = reg;
	spin_unlock_irqrestore(&i2cq->lock, flags);

	ready = readl_relaxed(i2cq->i2cq_base + R_HOST_CMD_RECV_READY);
	host_int = readl_relaxed(i2cq->i2cq_base + R_HOST_INT_STATUS);
	host_err = readl_relaxed(i2cq->i2cq_base + R_HOST_TIMEOUT_STATUS);
	mcu_int = readl_relaxed(i2cq->i2cq_base + R_MCU_INT_STATUS);
	mcu_err = readl_relaxed(i2cq->i2cq_base + R_MCU_ERROR_STATUS);

	dev_dbg(i2cq->dev,
		"dev: 0x%02X, reg: 0x%04X, val: 0x%04X, dir: %d, wri: %d, rdi: %d, ready: %d, int: 0x%08X/0x%08X, err: 0x%08X/0x%08X\n",
		dev >> 1, reg, val, dir, i2cq->wr_index, i2cq->rd_index, ready,
		host_int, mcu_int, host_err, mcu_err);
	/* If hardware is not ready to recv command, or exists error
	 * return to do next
	 */
	if ((ready != 0x1) || ((mcu_int | host_int) & 0x78) ||
	    ((mcu_err | host_err) & 0x1F00)) {
		dev_dbg(i2cq->dev, "%s: cmd queue is not ready, reset!\n",
			__func__);

		spin_lock_irqsave(&i2cq->lock, flags);
		init_hw(i2cq);
		spin_unlock_irqrestore(&i2cq->lock, flags);
	}

	spin_lock_irqsave(&i2cq->lock, flags);
	if (dir == CMD_DIR_WRITE) {
		writel_relaxed(i2cq->params->host_int_ctrl,
			       i2cq->i2cq_base + R_HOST_INT_CTRL);
	} else {
		/* Mask full empty interrupts for read operation.
		 * we can use ack error and read back interrupts.
		 */
		writel_relaxed(i2cq->params->host_int_ctrl |
				       BIT_HOST_INT_QUEUE_FULL_EMPTY,
			       i2cq->i2cq_base + R_HOST_INT_CTRL);
	}

	cmd = i2cq->cmd_mode << OFF_CMDH_FAST_MODE | dir << OFF_CMDH_DIRECTION |
	      fmt << OFF_CMDH_FORMAT | dev;
	writel_relaxed(cmd, i2cq->i2cq_base + R_HOST_CMD_RECV_DATH);
	cmd = reg << OFF_CMDL_REG_ADDR | val;
	writel_relaxed(cmd, i2cq->i2cq_base + R_HOST_CMD_RECV_DATL);
	spin_unlock_irqrestore(&i2cq->lock, flags);

	return 0;
}

static int xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num,
		bool atomic)
{
	unsigned long flags;
	struct bst_i2cq *i2cq;

	if (num <= 0)
		return -EINVAL;

	i2cq = i2c_get_adapdata(adap);
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	if (i2cq->role != ROLE_MASTER)
		return -EBUSY;
#endif

	spin_lock_irqsave(&i2cq->lock, flags);
	i2cq->msgs = msgs;
	i2cq->nmsg = num;
	i2cq->wr_index = 0;
	i2cq->rd_index = 0;
	i2cq->error = 0;
	i2cq->atomic = atomic;
	spin_unlock_irqrestore(&i2cq->lock, flags);

	while (true) {
		if (!atomic)
			reinit_completion(&i2cq->wait_comp);
		else
			WRITE_ONCE(i2cq->poll_comp, 0);

		if (xfer_next(i2cq))
			break;

		if (!atomic) {
			unsigned long timeout;

			timeout = wait_for_completion_timeout(
				&i2cq->wait_comp,
				msecs_to_jiffies(i2cq->timeout));
			if (timeout == 0) {
				i2cq->error |= BIT_HOST_INT_MASTER_TIMEOUT;
				dev_dbg(i2cq->dev, "wait timeout\n");
				break;
			}
		} else {
			int rv;
			u32 poll_comp;

			rv = read_poll_timeout_atomic(READ_ONCE, poll_comp,
						      poll_comp, 10,
						      i2cq->timeout * 1000,
						      false, i2cq->poll_comp);
			if (rv < 0) {
				i2cq->error |= BIT_HOST_INT_MASTER_TIMEOUT;
				dev_dbg(i2cq->dev, "poll timeout\n");
				break;
			}
		}
		if (i2cq->error)
			break;
		i2cq->wr_index = i2cq->rd_index + 1;
	}

	dev_dbg(i2cq->dev, "wri: %d, rdi: %d, error: 0x%08X\n", i2cq->wr_index,
		i2cq->rd_index, i2cq->error);

	/* Before we return, clear the data to avoid incorrect using
	 * by delayed interrupt or interrupt from MCU
	 */
	spin_lock_irqsave(&i2cq->lock, flags);
	i2cq->msgs = NULL;
	i2cq->nmsg = 0;
	i2cq->curr_dev = 0;
	i2cq->curr_reg = 0;
	spin_unlock_irqrestore(&i2cq->lock, flags);

	if (i2cq->wr_index > 0)
		return i2cq->wr_index;

	if (i2cq->error & BIT_HOST_INT_READ_EMPTY_ERROR ||
	    i2cq->error & BIT_HOST_INT_WRITE_FULL_ERROR ||
	    i2cq->error & BIT_SW_EBUSY)
		return -EAGAIN;

	if (i2cq->error & BIT_HOST_INT_MASTER_TIMEOUT)
		return -ETIMEDOUT;

	if (i2cq->error & BIT_HOST_INT_ACK_ERROR)
		return -EIO;

	return -EINVAL;
}

static int bst_i2cq_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
			 int num)
{
	return xfer(adap, msgs, num, false);
}

static int bst_i2cq_xfer_atomic(struct i2c_adapter *adap, struct i2c_msg *msgs,
				int num)
{
	return xfer(adap, msgs, num, true);
}

static u32 bst_i2cq_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C;
}

static const struct i2c_algorithm bst_i2cq_algo = {
	.master_xfer = bst_i2cq_xfer,
	.master_xfer_atomic = bst_i2cq_xfer_atomic,
	.functionality = bst_i2cq_functionality,
};

static int parse_dt(struct bst_i2cq *i2cq)
{
	int rv;
	struct device *dev;
	struct device_node *node;

	dev = i2cq->dev;
	node = dev->of_node;
	i2cq->rstc = devm_reset_control_get_shared(dev, "i2cq-reset");
	if (IS_ERR_OR_NULL(i2cq->rstc))
		dev_warn(dev, "Failed to parse i2cq-reset\n");

	i2cq->irq = platform_get_irq(to_platform_device(dev), 0);
	if (i2cq->irq < 0) {
		dev_err(dev, "Failed to parse irq\n");
		return -EINVAL;
	}

	(void)of_property_read_s32(node, "i2c-nr", &i2cq->i2c_nr);

	rv = of_property_read_u32(node, "id", &i2cq->id);
	if (rv) {
		dev_err(dev, "Failed to parse id\n");
		goto err_id;
	}

#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	rv = of_property_read_u32(node, "uid", &i2cq->uid);
	if (rv) {
		dev_err(dev, "Failed to parse uid\n");
		goto err_id;
	}
	rv = of_property_read_u32(node, "role", &i2cq->role);
	if (rv)
		i2cq->role = ROLE_AUTO;
	{
		u32 sem[3];

		rv = of_property_read_u32_array(node, "ipc-sem", sem,
						ARRAY_SIZE(sem));
		if (rv) {
			dev_err(dev, "Failed to parse ipc-sem\n");
			goto err_id;
		}
		i2cq->sem_master = sem[0];
		i2cq->sem_bank = sem[1];
		i2cq->sem_id = sem[2];
	}
#endif
	rv = of_property_read_u32(node, "clock-frequency", &i2cq->speed);
	if (rv) {
		dev_err(dev, "Failed to parse clock-frequency\n");
		goto err_clk;
	}

	rv = of_property_read_u32(node, "clock-stretch", &i2cq->clock_stretch);
	if (rv || i2cq->clock_stretch > MAX_CLOCK_STRETCH)
		i2cq->clock_stretch = MAX_CLOCK_STRETCH / 2;

	rv = of_property_read_u32(node, "spike", &i2cq->spike);
	if (rv)
		i2cq->spike = MAX_SPIKE + 1;

	rv = of_property_read_u32(node, "ctrl-mode", &i2cq->ctrl_mode);
	if (rv)
		i2cq->ctrl_mode = CTRL_MODE_HYBRID;

	rv = of_property_read_u32(node, "cmd-mode", &i2cq->cmd_mode);
	if (rv)
		i2cq->cmd_mode = CMD_MODE_QUEUE;

	rv = of_property_read_u32(node, "timeout", &i2cq->timeout);
	if (rv)
		i2cq->timeout = XFER_TIMEOUT_MSEC;

	rv = of_property_read_u32(node, "retries", &i2cq->retries);
	if (rv)
		i2cq->retries = XFER_RETRIES;

	i2cq->params = of_device_get_match_data(dev);

	return 0;

err_clk:
err_id:
	return -EINVAL;
}

static int register_adapter(struct bst_i2cq *i2cq)
{
	i2cq->adapter.owner = THIS_MODULE;
	strscpy(i2cq->adapter.name, "bst-i2cq-adapter",
		sizeof(i2cq->adapter.name));
	i2cq->adapter.nr = i2cq->i2c_nr;
	i2cq->adapter.algo = &bst_i2cq_algo;
	i2cq->adapter.retries = i2cq->retries;
	i2cq->adapter.timeout = msecs_to_jiffies(i2cq->timeout * 100);
	i2cq->adapter.dev.parent = i2cq->dev;
	i2cq->adapter.dev.of_node = i2cq->dev->of_node;
	i2c_set_adapdata(&i2cq->adapter, i2cq);

	return i2c_add_numbered_adapter(&i2cq->adapter);
}

static int bst_i2cq_probe(struct platform_device *pdev)
{
	struct bst_i2cq *i2cq;
	struct device *dev;
	int rv;
	struct resource *res;
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	u32 uid;
#endif

	dev = &pdev->dev;
	i2cq = devm_kzalloc(dev, sizeof(*i2cq), GFP_KERNEL);
	if (!i2cq)
		return -ENOMEM;
	i2cq->dev = dev;
	platform_set_drvdata(pdev, i2cq);

	if (parse_dt(i2cq) < 0) {
		dev_err(dev, "Failed to parse dt\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	i2cq->i2cq_base =
		devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (IS_ERR(i2cq->i2cq_base))
		return PTR_ERR(i2cq->i2cq_base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	i2cq->core_base =
		devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (IS_ERR(i2cq->core_base))
		return PTR_ERR(i2cq->core_base);

	if (!IS_ERR_OR_NULL(i2cq->rstc))
		reset_control_deassert(i2cq->rstc);

	spin_lock_init(&i2cq->lock);
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	i2cq->hwlock = bst_semaphore_init(i2cq->sem_master, i2cq->sem_bank,
					  i2cq->sem_id);
	if (!i2cq->hwlock) {
		dev_err(dev, "Failed to claim HW lock\n");
		return -ENOLCK;
	}
	rv = get_sem_lock_with_timeout(i2cq->hwlock, 1);
	if (rv != 0 && rv != i2cq->sem_master) {
		dev_info(dev, "Sem %u of bank %u is hold by master %u\n",
			 i2cq->sem_id, i2cq->sem_bank, rv);
		samphore_lock_remove(i2cq->hwlock);
		return -EPROBE_DEFER;
	}
	enable_clk(i2cq);
	uid = readl_relaxed(i2cq->core_base + R_TOP_UID);
	dev_info(dev, "Current UID: %u, I: %u\n", uid, i2cq->uid);
	if (i2cq->role == ROLE_AUTO) {
		if (uid != 0 && uid != i2cq->uid) {
			i2cq->role = ROLE_SLAVE;
		} else {
			i2cq->role = ROLE_MASTER;
			dev_info(dev, "Set as master\n");
		}
	}
	if (i2cq->role == ROLE_SLAVE) {
		rv = register_adapter(i2cq);
		release_sem_lock(i2cq->hwlock);
		if (rv) {
			dev_err(dev, "Failed to add bus to i2c core, rv: %d\n",
				rv);
			samphore_lock_remove(i2cq->hwlock);
		}
		return rv;
	}
#else
	enable_clk(i2cq);
	i2cq->role = ROLE_MASTER;
#endif
	dev_dbg(i2cq->dev, "BURST_EN: 0x%08X\n",
		readl_relaxed(i2cq->i2cq_base + R_MS1_BURST_EN));
	if (!readl_relaxed(i2cq->i2cq_base + R_MS1_BURST_EN)) {
		dev_info(dev, "Initializing hardware\n");
		init_hw(i2cq);
	}

	rv = devm_request_irq(dev, i2cq->irq, isr_handler, 0, dev_name(dev),
			      i2cq);
	if (rv < 0) {
		dev_err(dev, "Can not claim IRQ %d\n", i2cq->irq);
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
		release_sem_lock(i2cq->hwlock);
		samphore_lock_remove(i2cq->hwlock);
#endif
		return rv;
	}
	init_completion(&i2cq->wait_comp);
	rv = register_adapter(i2cq);
	if (rv) {
		dev_err(dev, "Failed to add bus to i2c core, rv: %d\n", rv);
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
		release_sem_lock(i2cq->hwlock);
		samphore_lock_remove(i2cq->hwlock);
#endif
		return rv;
	}

#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	writel_relaxed(i2cq->uid, i2cq->core_base + R_TOP_UID);
	release_sem_lock(i2cq->hwlock);
#endif
	dev_info(dev, "Probe done on CPU %u\n", smp_processor_id());

	return 0;
}

static int bst_i2cq_remove(struct platform_device *pdev)
{
	struct bst_i2cq *i2cq = platform_get_drvdata(pdev);

	dev_info(i2cq->dev, "Remove\n");
	i2c_del_adapter(&i2cq->adapter);

#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	samphore_lock_remove(i2cq->hwlock);
#endif

	return 0;
};

static void bst_i2cq_shutdown(struct platform_device *pdev)
{
#ifndef CONFIG_VIDEO_BST_ISP_MULTI_OS
	unsigned long flags;
#endif
	struct bst_i2cq *i2cq = platform_get_drvdata(pdev);

	dev_info(i2cq->dev, "Shutdown\n");

#ifndef CONFIG_VIDEO_BST_ISP_MULTI_OS
	spin_lock_irqsave(&i2cq->lock, flags);
	close_hw(i2cq);
	spin_unlock_irqrestore(&i2cq->lock, flags);
	if (!IS_ERR_OR_NULL(i2cq->rstc))
		reset_control_assert(i2cq->rstc);
#endif
}

static int bst_i2cq_suspend(struct device *dev)
{
	return 0;
}

static int bst_i2cq_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bst_i2cq *i2cq = platform_get_drvdata(pdev);

	dev_info(dev, "Resume\n");
	if (i2cq->role != ROLE_MASTER)
		return 0;

	enable_clk(i2cq);
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	writel_relaxed(i2cq->uid, i2cq->core_base + R_TOP_UID);
#endif
	if (!readl_relaxed(i2cq->i2cq_base + R_MS1_BURST_EN)) {
		dev_info(dev, "Re-Initializing hardware\n");
		init_hw(i2cq);
	}

	return 0;
}

// clang-format off
static const struct dev_pm_ops bst_i2cq_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(bst_i2cq_suspend, bst_i2cq_resume)
};
// clang-format on

static const struct hw_params c1200_params = {
	.sclk = 600000000,
	.min_speed = 100800,
	.max_speed = 937500,
	.freq_div_factor = 64,
	/* Mask almost empty interrupts, since we
	 * can not receive ack success interrupt from hardware,
	 * We use full empty and polling method for write operation.
	 */
	.host_int_ctrl = 0x0C,
	/* Enable these interrupts, triggered by high level signal:
	 * host timeout & ack error,
	 * host read back valid,
	 */
	.mcu_int_ctrl = 0xCE,
};

static const struct of_device_id bst_i2cq_dt_ids[] = {
	{
		.compatible = "bst,c1200-i2cq",
		.data = &c1200_params,
	},
	{}
};
MODULE_DEVICE_TABLE(of, bst_i2cq_dt_ids);

static struct platform_driver bst_i2cq_driver = {
	.probe = bst_i2cq_probe,
	.remove = bst_i2cq_remove,
	.shutdown = bst_i2cq_shutdown,
	.driver = {
		.name = "bst-i2cq",
		.of_match_table = of_match_ptr(bst_i2cq_dt_ids),
		.pm = &bst_i2cq_pm_ops,
	},
};
module_platform_driver(bst_i2cq_driver);

MODULE_AUTHOR("BST Ltd.");
MODULE_DESCRIPTION("BST ISP I2C-Q Driver");
MODULE_LICENSE("GPL v2");
