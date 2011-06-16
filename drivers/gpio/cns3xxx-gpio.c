/*******************************************************************************
 *
 *  drivers/gpio/cns3xxx-gpio.c
 *
 *  GPIO driver for the CNS3XXX SOCs
 *
 *  Copyright (c) 2009 Cavium Networks
 *
 *  This file is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License, Version 2, as
 *  published by the Free Software Foundation.
 *
 *  This file is distributed in the hope that it will be useful,
 *  but AS-IS and WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE, TITLE, or
 *  NONINFRINGEMENT.  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this file; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA or
 *  visit http://www.gnu.org/licenses/.
 *
 *  This file may also be available under a different license from Cavium.
 *  Contact Cavium Networks for more information
 *
 ******************************************************************************/

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <asm/mach-types.h>

#include <mach/cns3xxx.h>
#include <mach/gpio.h>

/***********************************************************************
 * The CNS3XXX has GPIOA(32) and GPIOB(32) total 64 GPIO. For the
 * generic GPIO interface, the GPIO pin number count from GPIOA to GPIOB.
 * For example:
 *      0 -> GPIOA[0]
 *      1 -> GPIOA[1]
 *     ......
 *     31 -> GPIOA[31]
 *     32 -> GPIOB[0]
 *     33 -> GPIOB[1]
 *     ......
 *     63 -> GPIOB[31]
 **********************************************************************/

/*
 * Configure the GPIO line as an input.
 */
int cns3xxx_set_gpio_direction_input(unsigned int pin)
{
	volatile __u32 reg;
	unsigned long flags;

	if (pin >= MAX_GPIO_NO)
		return -EINVAL;

	local_irq_save(flags);


	/* Clear register bit to set as input pin. */
	if (pin < GPIOA_PIN_NO) { /* GPIOA */
		reg = __raw_readl(CNS3XXX_GPIOA_BASE_VIRT + GPIO_DIR);
		reg &= ~(1 << pin);
		__raw_writel(reg, CNS3XXX_GPIOA_BASE_VIRT + GPIO_DIR);
	} else {		  /* GPIOB */
		reg = __raw_readl(CNS3XXX_GPIOB_BASE_VIRT + GPIO_DIR);
		reg &= ~(1 << (pin - GPIOA_PIN_NO));
		__raw_writel(reg, CNS3XXX_GPIOB_BASE_VIRT + GPIO_DIR);
	}

	local_irq_restore(flags);

	return 0;
}

static int cns3xxx_direction_in(struct gpio_chip *chip, unsigned offset)
{
	return (offset < CNS3XXX_GPIO_MAX)
		? cns3xxx_set_gpio_direction_input(offset)
		: -EINVAL;
}
/*
 * Configure the GPIO line as an output, with default state.
 */
int __init_or_module cns3xxx_set_gpio_direction_output(
	unsigned int pin, unsigned int state)
{
	volatile __u32 reg;
	u32 base, gpio_index;
	unsigned long flags;
	if (pin < 0 || pin >= MAX_GPIO_NO)
		return -EINVAL;

	local_irq_save(flags);

	if (pin < GPIOA_PIN_NO) { /* GPIOA */
		base = CNS3XXX_GPIOA_BASE_VIRT;
		gpio_index = 1 << pin;
	} else {		  /* GPIOB */
		base = CNS3XXX_GPIOB_BASE_VIRT;
		gpio_index = 1 << (pin - GPIOA_PIN_NO);
	}

	/* Set register bit to set as output pin. */
	reg = __raw_readl(base + GPIO_DIR);
	reg |= gpio_index;
	__raw_writel(reg, base + GPIO_DIR);

	if (state)
		__raw_writel(gpio_index, base + GPIO_BIT_SET);
	else
		__raw_writel(gpio_index, base + GPIO_BIT_CLEAR);

	local_irq_restore(flags);

	return 0;
}

int __init_or_module cns3xxx_set_gpio_direction_output_only(unsigned int pin)
{
   volatile __u32 reg;
   u32 base, gpio_index;
   unsigned long flags;
   if (pin < 0 || pin >= MAX_GPIO_NO)
	   return -EINVAL;

   local_irq_save(flags);

   if (pin < GPIOA_PIN_NO) { /* GPIOA */
	   base = CNS3XXX_GPIOA_BASE_VIRT;
	   gpio_index = 1 << pin;
   } else { /* GPIOB */
	   base = CNS3XXX_GPIOB_BASE_VIRT;
	   gpio_index = 1 << (pin - GPIOA_PIN_NO);
   }

   /* Set register bit to set as output pin. */
   reg = __raw_readl(base + GPIO_DIR);
   reg |= gpio_index;
   __raw_writel(reg, base + GPIO_DIR);

   local_irq_restore(flags);

   return 0;
}

static int cns3xxx_direction_out(
	struct gpio_chip *chip, unsigned offset, int value)
{
	return (offset < CNS3XXX_GPIO_MAX)
		? cns3xxx_set_gpio_direction_output(offset, value)
		: -EINVAL;
}

/*
 * Set the state of an output GPIO line.
 */
void cns3xxx_gpio_set_value(unsigned int pin, unsigned int state)
{
	if (pin < 0 || pin >= MAX_GPIO_NO)
		return;

	if (pin < GPIOA_PIN_NO)	{ /* GPIOA */
		if (state)
			__raw_writel(1 << pin,
				CNS3XXX_GPIOA_BASE_VIRT + GPIO_BIT_SET);
		else
			__raw_writel(1 << pin,
				CNS3XXX_GPIOA_BASE_VIRT + GPIO_BIT_CLEAR);
	} else {		  /* GPIOB */
		if (state)
			__raw_writel(1 << (pin - GPIOA_PIN_NO),
				CNS3XXX_GPIOB_BASE_VIRT + GPIO_BIT_SET);
		else
			__raw_writel(1 << (pin - GPIOA_PIN_NO),
				CNS3XXX_GPIOB_BASE_VIRT + GPIO_BIT_CLEAR);
	}
}

static void cns3xxx_set(struct gpio_chip *chip, unsigned offset, int value)
{
	cns3xxx_gpio_set_value(offset, value);
}

/*
 * Read the state of a GPIO line.
 */
int cns3xxx_gpio_get_value(unsigned int pin)
{
	volatile __u32 reg;
	bool bret = 0;

	if (pin < 0 || pin >= MAX_GPIO_NO)
		return -EINVAL;

	if (pin < GPIOA_PIN_NO) { /* GPIOA */
		reg = __raw_readl(CNS3XXX_GPIOA_BASE_VIRT + GPIO_INPUT);
		bret = (reg & (1 << pin)) != 0;
	} else {		  /* GPIOB */
		reg = __raw_readl(CNS3XXX_GPIOB_BASE_VIRT + GPIO_INPUT);
		bret = (reg & (1 << (pin - GPIOA_PIN_NO))) != 0;
	}

	return bret;
}

static int cns3xxx_get(struct gpio_chip *chip, unsigned offset)
{
	return cns3xxx_gpio_get_value(offset);
}

static int cns3xxx_request(struct gpio_chip *chip, unsigned offset)
{
	if (0 <= offset && offset <= 31) {
		if (((__raw_readl(MISC_GPIOA_PIN_ENABLE_REG) >> offset) & 1) == 1)
			return -EINVAL;
		else
			return 0;
	} else if (32 <= offset && offset <= 63) {
		if (((__raw_readl(MISC_GPIOB_PIN_ENABLE_REG) >> (offset-32)) & 1) == 1)
			return -EINVAL;
		else
			return 0;
	} else {
		return -EINVAL;
	}
}

static void cns3xxx_free(struct gpio_chip *chip, unsigned offset)
{
}

static int cns3xxx_to_irq(struct gpio_chip *chip, unsigned offset)
{
	if (0 <= offset && offset <= 31)
		return IRQ_CNS3XXX_GPIOA;
	else if (32 <= offset && offset <= 63)
		return IRQ_CNS3XXX_GPIOB;
	else
		return -EINVAL;
}

static struct gpio_chip cns3xxx_gpiochip = {
	.label			= "cns3xxx_gpio",
	.owner			= THIS_MODULE,
	.request		= cns3xxx_request,
	.free			= cns3xxx_free,
	.direction_input	= cns3xxx_direction_in,
	.get			= cns3xxx_get,
	.direction_output	= cns3xxx_direction_out,
	.set			= cns3xxx_set,
	.to_irq			= cns3xxx_to_irq,
	.can_sleep		= 0,
};

int __init gpio_init(void)
{
	cns3xxx_pwr_clk_en(0x1 << PM_CLK_GATE_REG_OFFSET_GPIO);
	cns3xxx_pwr_soft_rst(0x1 << PM_CLK_GATE_REG_OFFSET_GPIO);

	return 0;
}

static int __devinit gpio_cns3xxx_probe(struct platform_device *pdev)
{
	int ret;

	cns3xxx_gpiochip.label = "cns3xxx_gpio_chip";
	cns3xxx_gpiochip.base = -1;
	cns3xxx_gpiochip.dev = &pdev->dev;
	cns3xxx_gpiochip.ngpio = CNS3XXX_GPIO_MAX;

	ret = gpiochip_add(&cns3xxx_gpiochip);
	if (ret < 0) {
		dev_err(&pdev->dev,
				"could not register gpiochip, %d\n",
				ret);
	}
	return gpio_init();
}

static int __devexit gpio_cns3xxx_remove(struct platform_device *dev)
{
	return 0;
}

static struct platform_driver gpio_cns3xxx_driver = {
	.driver.name	= "cns3xxx-gpio",
	.driver.owner	= THIS_MODULE,
	.probe		= gpio_cns3xxx_probe,
	.remove		= __devexit_p(gpio_cns3xxx_remove),
};

int __init cns3xxx_gpio_init(void)
{
	return platform_driver_register(&gpio_cns3xxx_driver);
}

void __exit cns3xxx_gpio_exit(void)
{
	platform_driver_unregister(&gpio_cns3xxx_driver);
}

module_init(cns3xxx_gpio_init);
module_exit(cns3xxx_gpio_exit);

MODULE_LICENSE("GPL");
