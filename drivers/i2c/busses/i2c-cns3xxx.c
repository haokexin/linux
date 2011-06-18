#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/i2c.h>

#include <linux/slab.h>

/*
 * We need the memory map
 */
#include <mach/cns3xxx.h>

#define MISC_I2C_SCL_PIN	((0x1 << 12))
#define MISC_I2C_SDA_PIN	((0x1 << 13))

#define I2C_PCLK                15000000

#define I2C_MEM_MAP_ADDR(x)         (CNS3XXX_SSP_BASE_VIRT + x)

#define I2C_CONTROLLER_REG				I2C_MEM_MAP_ADDR(0x20)
#define I2C_TIME_OUT_REG				I2C_MEM_MAP_ADDR(0x24)
#define I2C_SLAVE_ADDRESS_REG			I2C_MEM_MAP_ADDR(0x28)
#define I2C_WRITE_DATA_REG				I2C_MEM_MAP_ADDR(0x2C)
#define I2C_READ_DATA_REG				I2C_MEM_MAP_ADDR(0x30)
#define I2C_INTERRUPT_STATUS_REG		I2C_MEM_MAP_ADDR(0x34)
#define I2C_INTERRUPT_ENABLE_REG		I2C_MEM_MAP_ADDR(0x38)
#define I2C_TWI_OUT_DLY_REG				I2C_MEM_MAP_ADDR(0x3C)

#define I2C_READ_ONLY_CMD		(0)
#define I2C_WRITE_ONLY_CMD		(1)
#define I2C_WRITE_READ_CMD		(2)
#define I2C_READ_WRITE_CMD		(3)

#define I2C_DATA_LEN_1_BYTE		(0)
#define I2C_DATA_LEN_2_BYTE		(1)
#define I2C_DATA_LEN_3_BYTE		(2)
#define I2C_DATA_LEN_4_BYTE		(3)

#define I2C_BUS_ERROR_FLAG		(0x1)
#define I2C_ACTION_DONE_FLAG	(0x2)

#define CNS3xxx_I2C_ENABLE() do {\
	u32 reg = __raw_readl(I2C_CONTROLLER_REG); \
	reg |= ((unsigned int)0x1 << 31); \
	__raw_writel(reg, I2C_CONTROLLER_REG); \
} while (0)

#define CNS3xxx_I2C_DISABLE() do {\
	u32 reg = __raw_readl(I2C_CONTROLLER_REG); \
	reg &= ~((unsigned int)0x1 << 31); \
	__raw_writel(reg, I2C_CONTROLLER_REG); \
} while (0)

#define CNS3xxx_I2C_ENABLE_INTR() do {\
	u32 reg = __raw_readl(I2C_INTERRUPT_ENABLE_REG); \
	reg |= 0x03; \
	__raw_writel(reg, I2C_INTERRUPT_ENABLE_REG); \
} while (0)

#define CNS3xxx_I2C_DISABLE_INTR()  do {\
	u32 reg = __raw_readl(I2C_INTERRUPT_ENABLE_REG); \
	reg &= 0xfc; \
	__raw_writel(reg, I2C_INTERRUPT_ENABLE_REG); \
} while (0)

#define CNS3xxx_I2C_ENABLE_DATA_SWAP()  do {\
	u32 reg = __raw_readl(I2C_CONTROLLER_REG); \
	reg |= (0x1 << 24); \
	__raw_writel(reg, I2C_CONTROLLER_REG); \
} while (0)

#define CNS3xxx_I2C_DISABLE_DATA_SWAP()  do {\
	u32 reg = __raw_readl(I2C_CONTROLLER_REG); \
	reg &= ~(0x1 << 24); \
	__raw_writel(reg, I2C_CONTROLLER_REG); \
} while (0)

#define CNS3xxx_I2C_START_TRANSFER()  do {\
	u32 reg = __raw_readl(I2C_CONTROLLER_REG); \
	reg |= (0x1 << 6); \
	__raw_writel(reg, I2C_CONTROLLER_REG); \
} while (0)

#define CNS3xxx_I2C_STOP_TRANSFER()  do {\
	u32 reg = __raw_readl(I2C_CONTROLLER_REG); \
	reg &= ~(0x1 << 6); \
	__raw_writel(reg, I2C_CONTROLLER_REG); \
} while (0)

#define TWI_TIMEOUT		(10*HZ)
#define I2C_100KHZ		100000
#define I2C_200KHZ		200000
#define I2C_300KHZ		300000
#define I2C_400KHZ		400000

#define CNS3xxx_I2C_CLK     I2C_100KHZ

#define STATE_DONE		0
#define STATE_START		1
#define STATE_ERROR		4

struct cns3xxx_i2c {
	void __iomem *base;
	wait_queue_head_t wait;
	struct i2c_adapter adap;
	int state;

	__u16 readlen;
	__u16 writelen;
	__u8 *readbuf;
	__u8 *writebuf;
};

static u32 cns3xxx_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static void cns3xxx_i2c_setread(struct cns3xxx_i2c *i2c,
	struct i2c_msg *msg)
{
	i2c->readlen = msg->len;
	i2c->readbuf = msg->buf;
}

static void cns3xxx_i2c_setwrite(struct cns3xxx_i2c *i2c,
	struct i2c_msg *msg)
{
	i2c->writelen = msg->len;
	i2c->writebuf = msg->buf;
}

static int cns3xxx_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
			    int num)
{
	struct cns3xxx_i2c *i2c = i2c_get_adapdata(adap);

	int ret = num;
	int msgpos, nmsgs;
	uint32_t data = 0, read_data_len = 0, write_data_len = 0, i2c_cmd_type;
	unsigned long orig_jiffies = jiffies;
	/*Perform Message format Check*/
	for (msgpos = 0; msgpos < num; msgpos++) {
		if (msgs[msgpos].len > 4) {
			printk(KERN_ERR
				"%s:%d: Presently the driver does not handle"
				"i2c transaction larger than 4 bytes\n",
		     __func__, __LINE__);
			return -ENOSPC;
		}
		if (msgs[msgpos].flags & I2C_M_TEN) {
			printk(KERN_ERR
				"%s:%d: Presently the driver does not handle"
				" extended addressing\n",
		     __func__, __LINE__);
			return -EIO;
		}
	}

	while (i2c->state != STATE_DONE)	{
		if (signal_pending(current))
			return -EINTR;
		if (time_after(jiffies, orig_jiffies + HZ)) {
			printk(KERN_ERR "%s, Wait lock timeout.\n", __func__);
			return -EIO;
		}
		schedule();
	}

	msgpos = 0;
	while (num > 0) {
		nmsgs = 1;
		num -= 1;

		i2c->readbuf = NULL;
		i2c->readlen = 0;
		i2c->writebuf = NULL;
		i2c->writelen = 0;
		if (msgs[msgpos].flags & I2C_M_RD) {
			i2c_cmd_type = I2C_READ_ONLY_CMD;
			cns3xxx_i2c_setread(i2c, &msgs[msgpos]);
		} else {
			i2c_cmd_type = I2C_WRITE_ONLY_CMD;
			cns3xxx_i2c_setwrite(i2c, &msgs[msgpos]);
		}

		/* Check next message */
		if (num > 0) {
			if ((msgs[msgpos].addr == msgs[msgpos+1].addr) &&
				((msgs[msgpos].flags & I2C_M_RD) !=
				(msgs[msgpos+1].flags & I2C_M_RD))) {
				nmsgs = 2;
				num -= 1;

				if (msgs[msgpos+1].flags & I2C_M_RD) {
					i2c_cmd_type = I2C_WRITE_READ_CMD;
					cns3xxx_i2c_setread(i2c,
						&msgs[msgpos+1]);
				} else {
					i2c_cmd_type = I2C_READ_WRITE_CMD;
					cns3xxx_i2c_setwrite(i2c,
						&msgs[msgpos+1]);
				}
			}
		}
		/* Prepare I2C regs */
		switch (i2c->readlen) {
		case 1:
			read_data_len = I2C_DATA_LEN_1_BYTE;
			break;
		case 2:
			read_data_len = I2C_DATA_LEN_2_BYTE;
			break;
		case 3:
			read_data_len = I2C_DATA_LEN_3_BYTE;
			break;
		case 4:
			read_data_len = I2C_DATA_LEN_4_BYTE;
			break;
		}

		__raw_writel((msgs[msgpos].addr << 1), I2C_SLAVE_ADDRESS_REG);

		if (i2c->writelen) {
			data = 0;
			switch (i2c->writelen) {
			case 4:
				write_data_len = I2C_DATA_LEN_4_BYTE;
				data = (i2c->writebuf[3] << 24);
			case 3:
				if (i2c->writelen == 3)
					write_data_len = I2C_DATA_LEN_3_BYTE;
				data |= (i2c->writebuf[2] << 16);
			case 2:
				if (i2c->writelen == 2)
					write_data_len = I2C_DATA_LEN_2_BYTE;
				data |= (i2c->writebuf[1] << 8);
			case 1:
				if (i2c->writelen == 1)
					write_data_len = I2C_DATA_LEN_1_BYTE;
				data |= i2c->writebuf[0];
				break;
			}
			__raw_writel(data, I2C_WRITE_DATA_REG);
		}
		__raw_writel(
			((0x1 << 31) | (i2c_cmd_type << 4) |
			(write_data_len << 2) | (read_data_len << 0)),
			I2C_CONTROLLER_REG);

		i2c->state = STATE_START;
		CNS3xxx_I2C_START_TRANSFER();

		/* Send Cmd & Wait */
		if (wait_event_timeout(i2c->wait, (i2c->state == STATE_ERROR) ||
			       (i2c->state == STATE_DONE), TWI_TIMEOUT)) {
			if (i2c->state != STATE_DONE) {
				ret = -EIO;
				goto OUT;
			}
		} else {
			ret = -ETIMEDOUT;
			goto OUT;
		}

		msgpos += nmsgs;
	}
OUT:
	i2c->state = STATE_DONE;
	return ret;
}

static struct i2c_algorithm cns3xxx_i2c_algo = {
	.master_xfer = cns3xxx_i2c_xfer,
	.functionality = cns3xxx_i2c_func,
};

struct i2c_adapter cns3xxx_i2c_adapter = {
	.owner = THIS_MODULE,
	.algo = &cns3xxx_i2c_algo,
	.algo_data = NULL,
	.nr = 0,
	.name = "CNS3xxx I2C 0",
};

static inline void HAL_MISC_ENABLE_I2C_PINS(void)
{
	u32 reg = __raw_readl(MISC_GPIOB_PIN_ENABLE_REG);
	reg |= (MISC_I2C_SCL_PIN | MISC_I2C_SDA_PIN);
	__raw_writel(reg, MISC_GPIOB_PIN_ENABLE_REG);
}

static void cns3xxx_i2c_adapter_init(struct cns3xxx_i2c *i2c)
{

/* Steps
 * 1. Check if the power is enabled to the module (PMU_BASE + 0x010)
 * 2. Enable the clock (Enabled by default (PMU doc
 *    but check clk status anyway PMU_BASE + 0X00C)
 * 3. Configure the registers of i2c
 */
	u32 reg;
	i2c->state = STATE_DONE;

	cns3xxx_pwr_clk_en(0x1 << PM_CLK_GATE_REG_OFFSET_SPI_PCM_I2C);
	cns3xxx_pwr_power_up(0x1 << PM_CLK_GATE_REG_OFFSET_SPI_PCM_I2C);
	cns3xxx_pwr_soft_rst(0x1 << PM_CLK_GATE_REG_OFFSET_SPI_PCM_I2C);

	/* Disable the I2C */
	__raw_writel(0, I2C_CONTROLLER_REG);

	/* enable SCL and SDA which share pin with GPIOB_PIN_EN(0x18)
	 * GPIOB[12]: SCL
	 * GPIOB[13]: SDA
	 */
	HAL_MISC_ENABLE_I2C_PINS();

	reg = __raw_readl(MISC_IO_PAD_DRIVE_STRENGTH_CTRL_B);
	reg &= ~0x300;
	__raw_writel(reg, MISC_IO_PAD_DRIVE_STRENGTH_CTRL_B);

	reg = __raw_readl(MISC_IO_PAD_DRIVE_STRENGTH_CTRL_B);
	reg  |= 0x300; /* 21mA... */
	__raw_writel(reg, MISC_IO_PAD_DRIVE_STRENGTH_CTRL_B);

	/* Check the Reg Dump when testing */
	__raw_writel(
	    ((((((cns3xxx_cpu_clock()*(1000000/8)) / (2 * CNS3xxx_I2C_CLK)) -
		1) & 0x3FF) << 8) | (1 << 7) | 0x7F),
		I2C_TIME_OUT_REG);

	reg = __raw_readl(I2C_TWI_OUT_DLY_REG);
	reg |= 0x3;
	__raw_writel(reg, I2C_TWI_OUT_DLY_REG);
	/* Clear Interrupt Status (0x2 | 0x1) */
	reg = __raw_readl(I2C_INTERRUPT_STATUS_REG);
	reg |= (I2C_ACTION_DONE_FLAG | I2C_BUS_ERROR_FLAG);
	__raw_writel(reg, I2C_INTERRUPT_STATUS_REG);

	/* Enable The Interrupt */
	CNS3xxx_I2C_ENABLE_INTR();

	/* Enable the I2C Controller */
	CNS3xxx_I2C_ENABLE();
}

static irqreturn_t cns3xxx_i2c_isr(int irq, void *dev_id)
{
	struct cns3xxx_i2c *i2c = dev_id;
	uint32_t data = 0;
	uint32_t stat = __raw_readl(I2C_INTERRUPT_STATUS_REG);

	if (stat & I2C_BUS_ERROR_FLAG) {
		i2c->state = STATE_ERROR;
		goto IRQ_OUT;
	}

	if (i2c->readlen) {
		data = __raw_readl(I2C_READ_DATA_REG);
		switch (i2c->readlen) {
		case 4:
			i2c->readbuf[3] = ((data & 0xFF000000) >> 24);
		case 3:
			i2c->readbuf[2] = ((data & 0x00FF0000) >> 16);
		case 2:
			i2c->readbuf[1] = ((data & 0x0000FF00) >> 8);
		case 1:
			i2c->readbuf[0] = data & 0x000000FF;
			break;
		}
	}
	i2c->state = STATE_DONE;

IRQ_OUT:
	/* Clear Interrupt */
	stat |= 0x1;
	__raw_writel(stat, I2C_INTERRUPT_STATUS_REG);
	wake_up(&i2c->wait);

	return IRQ_HANDLED;
}

static int __devinit cns3xxx_i2c_probe(struct platform_device *pdev)
{
	struct cns3xxx_i2c *i2c;
	struct resource *res, *res2;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		printk(KERN_ERR "%s: IORESOURCE_MEM not defined\n", __func__);
		return -ENODEV;
	}

	res2 = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res2) {
		printk(KERN_ERR "%s: IORESOURCE_IRQ not defined\n", __func__);
		return -ENODEV;
	}

	i2c = kzalloc(sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	if (!request_mem_region(res->start, res->end - res->start + 1,
				pdev->name)) {
		dev_err(&pdev->dev, "Memory region busy\n");
		ret = -EBUSY;
		goto request_mem_failed;
	}

	i2c->base = ioremap(res->start, res->end - res->start + 1);
	if (!i2c->base) {
		dev_err(&pdev->dev, "Unable to map registers\n");
		ret = -EIO;
		goto map_failed;
	}

	cns3xxx_i2c_adapter_init(i2c);

	init_waitqueue_head(&i2c->wait);
	ret = request_irq(res2->start, cns3xxx_i2c_isr, 0, pdev->name, i2c);
	if (ret) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		goto request_irq_failed;
	}

	platform_set_drvdata(pdev, i2c);
	i2c->adap = cns3xxx_i2c_adapter;
	i2c_set_adapdata(&i2c->adap, i2c);
	i2c->adap.dev.parent = &pdev->dev;

	/* add i2c adapter to i2c tree */
	ret = i2c_add_numbered_adapter(&i2c->adap);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add adapter\n");
		goto add_adapter_failed;
	}

	return 0;

add_adapter_failed:
	free_irq(res2->start, i2c);
request_irq_failed:
	iounmap(i2c->base);
map_failed:
	release_mem_region(res->start, res->end - res->start + 1);
request_mem_failed:
	kfree(i2c);
	return ret;
}

static int __devexit cns3xxx_i2c_remove(struct platform_device *pdev)
{
	struct cns3xxx_i2c *i2c = platform_get_drvdata(pdev);
	struct resource *res;

	/* disable i2c logic */
	CNS3xxx_I2C_DISABLE_INTR();
	CNS3xxx_I2C_DISABLE();
	cns3xxx_pwr_clk_dis(0x1 << PM_CLK_GATE_REG_OFFSET_SPI_PCM_I2C);
	/* remove adapter & data */
	i2c_del_adapter(&i2c->adap);
	platform_set_drvdata(pdev, NULL);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res)
		free_irq(res->start, i2c);

	iounmap(i2c->base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res)
		release_mem_region(res->start, res->end - res->start + 1);

	kfree(i2c);

	return 0;
}

#ifdef CONFIG_PM
static int cns3xxx_i2c_suspend(struct platform_device *pdev, pm_message_t msg)
{
	return 0;
}

static int cns3xxx_i2c_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define cns3xxx_i2c_suspend	NULL
#define cns3xxx_i2c_resume	NULL
#endif

static struct platform_driver cns3xxx_i2c_driver = {
	.probe = cns3xxx_i2c_probe,
	.remove = cns3xxx_i2c_remove,
	.suspend = cns3xxx_i2c_suspend,
	.resume = cns3xxx_i2c_resume,
	.driver = {
		.owner = THIS_MODULE,
		.name = "cns3xxx-i2c",
	},
};

static int __init cns3xxx_i2c_init(void)
{
	return platform_driver_register(&cns3xxx_i2c_driver);
}

static void __exit cns3xxx_i2c_exit(void)
{
	platform_driver_unregister(&cns3xxx_i2c_driver);
}

module_init(cns3xxx_i2c_init);
module_exit(cns3xxx_i2c_exit);

MODULE_AUTHOR("Cavium Networks");
MODULE_DESCRIPTION("Cavium CNS3XXX I2C Controller");
MODULE_LICENSE("GPL");
