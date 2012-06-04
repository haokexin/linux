/*
 *  PCA953x 4/8/16 bit I/O ports
 *
 *  Copyright (C) 2005 Ben Gardner <bgardner@wabtec.com>
 *  Copyright (C) 2007 Marvell International Ltd.
 *  Copyright (c) 2011 Kontron Global Software Center
 *
 *  Derived from drivers/i2c/chips/pca9539.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#ifdef CONFIG_OF_GPIO
#include <linux/of_platform.h>
#endif

#include <linux/io.h>
#include <linux/acpi.h>
#define PCA953X_INPUT		0
#define PCA953X_OUTPUT		1
#define PCA953X_INVERT		2
#define PCA953X_DIRECTION	3

#define PCA957X_IN		0
#define PCA957X_INVRT		1
#define PCA957X_BKEN		2
#define PCA957X_PUPD		3
#define PCA957X_CFG		4
#define PCA957X_OUT		5
#define PCA957X_MSK		6
#define PCA957X_INTS		7

#define PCA_GPIO_MASK		0x00FF
#define PCA_INT			0x0100
#define PCA953X_TYPE		0x1000
#define PCA957X_TYPE		0x2000

static const struct i2c_device_id pca953x_id[] = {
	{"pca9534", 8 | PCA953X_TYPE | PCA_INT,},
	{"pca9535", 16 | PCA953X_TYPE | PCA_INT,},
	{"pca9536", 4 | PCA953X_TYPE,},
	{"pca9537", 4 | PCA953X_TYPE | PCA_INT,},
	{"pca9538", 8 | PCA953X_TYPE | PCA_INT,},
	{"pca9539", 16 | PCA953X_TYPE | PCA_INT,},
	{"pca9554", 8 | PCA953X_TYPE | PCA_INT,},
	{"pca9555", 16 | PCA953X_TYPE | PCA_INT,},
	{"pca9556", 8 | PCA953X_TYPE,},
	{"pca9557", 8 | PCA953X_TYPE,},
	{"pca9574", 8 | PCA957X_TYPE | PCA_INT,},
	{"pca9575", 16 | PCA957X_TYPE | PCA_INT,},

	{"max7310", 8 | PCA953X_TYPE,},
	{"max7312", 16 | PCA953X_TYPE | PCA_INT,},
	{"max7313", 16 | PCA953X_TYPE | PCA_INT,},
	{"max7315", 8 | PCA953X_TYPE | PCA_INT,},
	{"pca6107", 8 | PCA953X_TYPE | PCA_INT,},
	{"tca6408", 8 | PCA953X_TYPE | PCA_INT,},
	{"tca6416", 16 | PCA953X_TYPE | PCA_INT,},
	/* NYET:  { "tca6424", 24, }, */
	{}
};

MODULE_DEVICE_TABLE(i2c, pca953x_id);

struct pca953x_chip {
	unsigned gpio_start;
	uint16_t reg_output;
	uint16_t reg_direction;
	struct mutex i2c_lock;

#ifdef CONFIG_GPIO_PCA953X_IRQ
	struct mutex irq_lock;
	uint16_t irq_mask;
	uint16_t irq_stat;
	uint16_t irq_trig_raise;
	uint16_t irq_trig_fall;
	int irq_base;
#endif

	struct i2c_client *client;
	struct gpio_chip gpio_chip;
	const char *const *names;
	int chip_type;
};

#ifdef CONFIG_GPIO_PCA9555_FRI2

#define GPIO_GPE	0x0B	/* SMB_ALERT GPE number on FRI2 */
#define GPEC		0x18	/* GPE Control register (used to reset GPE) */
#define	FRI2_GPIO_IRQ_BASE	32

/*
  Board revision:
  0x11 - MP1 or older
  0x10 - MP2 or newer (default)
*/
static int fri2_revision = 0x10;

int fri2_get_revision(void)
{
	return fri2_revision;
}
EXPORT_SYMBOL_GPL(fri2_get_revision);

/* We can install only _one_ handler for SMB_ALERT GPE, but there are
 * _two_ pca9555 chips on FRI2.
 * so we need a way to test all GPIOs in one handler. fri2_chips should
 * contain info about both pca9555.
 */
static struct pca953x_chip *fri2_chips[] = { NULL, NULL };

static int fri2_chip_count;

/* only pins 0 1 3 4 6 and 15 are allowed to write on FRI2 */

#define EXP_A_1_0 1
#define EXP_A_1_1 (1<<1)
#define EXP_A_1_3 (1<<3)
#define EXP_A_1_4 (1<<4)
#define EXP_A_1_6 (1<<6)
#define EXP_A_1_15 (1<<15)

#define FRI2_OUT_MASK (EXP_A_1_0 |\
EXP_A_1_1 | EXP_A_1_3 | EXP_A_1_4 | EXP_A_1_6 | EXP_A_1_15)

/* use i2c device 0x18 to access accelerometer (workaround -
 * accelerometer holds SMB_ALERT low) 0x20 and 0x21 are pca9555
 * chips
 */
static const unsigned short normal_i2c[] = { 0x18, 0x20, 0x21, I2C_CLIENT_END };

static struct pca953x_platform_data fri2_gpio_expander_a_info = {
	.gpio_base = 0,
	.irq_base = FRI2_GPIO_IRQ_BASE,
	.invert = 0,
};

static struct pca953x_platform_data fri2_gpio_expander_b_info = {
	.gpio_base = 16,
	.irq_base = FRI2_GPIO_IRQ_BASE + 16,
	.invert = 0,
};

/* Return 0 if detection is successful, -ENODEV otherwise */
static int pca953x_i2c_detect(struct i2c_client *client,
			      struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -ENODEV;

	if (client->addr == 0x20) {
		/* IO Expander A */
		info->platform_data = &fri2_gpio_expander_a_info;
	} else if (client->addr == 0x21) {
		/* IO expander B */
		int val;
		info->platform_data = &fri2_gpio_expander_b_info;
		val = i2c_smbus_read_word_data(client, PCA953X_INPUT << 1);
		if (val >= 0)
			fri2_revision = (val >> 14) & 0x3;
	} else if (client->addr == 0x18) {
		/* Hack !! set accelerometer to not pull smb_alert */
		int val;
		val = i2c_smbus_read_byte_data(client, 0x22);
		printk(KERN_INFO "PCA95xx: Set accelerometer to not pull \
			SMB_ALERT. pca953x_i2c_detect should return -ENODEV.");
		i2c_smbus_write_byte_data(client, 0x22, 0x40);
		/* always return error - we're not using accelerometer here. */
		return -ENODEV;
	}
	strlcpy(info->type, "pca9555", I2C_NAME_SIZE);
	return 0;
}

#endif

static int pca953x_write_reg(struct pca953x_chip *chip, int reg, uint16_t val)
{
	int ret = 0;

	if (chip->gpio_chip.ngpio <= 8)
		ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	else {
		switch (chip->chip_type) {
		case PCA953X_TYPE:
			ret = i2c_smbus_write_word_data(chip->client,
							reg << 1, val);
			break;
		case PCA957X_TYPE:
			ret = i2c_smbus_write_byte_data(chip->client, reg << 1,
							val & 0xff);
			if (ret < 0)
				break;
			ret = i2c_smbus_write_byte_data(chip->client,
							(reg << 1) + 1,
							(val & 0xff00) >> 8);
			break;
		}
	}

	if (ret < 0) {
		dev_err(&chip->client->dev, "failed writing register\n");
		return ret;
	}

	return 0;
}

static int pca953x_read_reg(struct pca953x_chip *chip, int reg, uint16_t *val)
{
	int ret;

	if (chip->gpio_chip.ngpio <= 8)
		ret = i2c_smbus_read_byte_data(chip->client, reg);
	else
		ret = i2c_smbus_read_word_data(chip->client, reg << 1);

	if (ret < 0) {
		dev_err(&chip->client->dev, "failed reading register\n");
		return ret;
	}

	*val = (uint16_t) ret;
	return 0;
}

static int pca953x_gpio_direction_input(struct gpio_chip *gc, unsigned off)
{
	struct pca953x_chip *chip;
	uint16_t reg_val;
	int ret, offset = 0;

	chip = container_of(gc, struct pca953x_chip, gpio_chip);

	mutex_lock(&chip->i2c_lock);
	reg_val = chip->reg_direction | (1u << off);

	switch (chip->chip_type) {
	case PCA953X_TYPE:
		offset = PCA953X_DIRECTION;
		break;
	case PCA957X_TYPE:
		offset = PCA957X_CFG;
		break;
	}
	ret = pca953x_write_reg(chip, offset, reg_val);
	if (ret)
		goto exit;

	chip->reg_direction = reg_val;
	ret = 0;
exit:
	mutex_unlock(&chip->i2c_lock);
	return ret;
}

static int pca953x_gpio_direction_output(struct gpio_chip *gc,
					 unsigned off, int val)
{
	struct pca953x_chip *chip;
	uint16_t reg_val;
	int ret, offset = 0;

	chip = container_of(gc, struct pca953x_chip, gpio_chip);

	mutex_lock(&chip->i2c_lock);
#ifdef CONFIG_GPIO_PCA9555_FRI2
	if (!(FRI2_OUT_MASK & (1 << (chip->gpio_start + off))))
		return -EINVAL;
#endif

	/* set output level */
	if (val)
		reg_val = chip->reg_output | (1u << off);
	else
		reg_val = chip->reg_output & ~(1u << off);

	switch (chip->chip_type) {
	case PCA953X_TYPE:
		offset = PCA953X_OUTPUT;
		break;
	case PCA957X_TYPE:
		offset = PCA957X_OUT;
		break;
	}
	ret = pca953x_write_reg(chip, offset, reg_val);
	if (ret)
		goto exit;

	chip->reg_output = reg_val;

	/* then direction */
	reg_val = chip->reg_direction & ~(1u << off);
	switch (chip->chip_type) {
	case PCA953X_TYPE:
		offset = PCA953X_DIRECTION;
		break;
	case PCA957X_TYPE:
		offset = PCA957X_CFG;
		break;
	}
	ret = pca953x_write_reg(chip, offset, reg_val);
	if (ret)
		goto exit;

	chip->reg_direction = reg_val;
	ret = 0;
exit:
	mutex_unlock(&chip->i2c_lock);
	return ret;
}

static int pca953x_gpio_get_value(struct gpio_chip *gc, unsigned off)
{
	struct pca953x_chip *chip;
	uint16_t reg_val;
	int ret, offset = 0;

	chip = container_of(gc, struct pca953x_chip, gpio_chip);

	mutex_lock(&chip->i2c_lock);
	switch (chip->chip_type) {
	case PCA953X_TYPE:
		offset = PCA953X_INPUT;
		break;
	case PCA957X_TYPE:
		offset = PCA957X_IN;
		break;
	}
	ret = pca953x_read_reg(chip, offset, &reg_val);
	mutex_unlock(&chip->i2c_lock);
	if (ret < 0) {
		/* NOTE:  diagnostic already emitted; that's all we should
		 * do unless gpio_*_value_cansleep() calls become different
		 * from their nonsleeping siblings (and report faults).
		 */
		return 0;
	}

	return (reg_val & (1u << off)) ? 1 : 0;
}

static void pca953x_gpio_set_value(struct gpio_chip *gc, unsigned off, int val)
{
	struct pca953x_chip *chip;
	uint16_t reg_val;
	int ret, offset = 0;

	chip = container_of(gc, struct pca953x_chip, gpio_chip);

	mutex_lock(&chip->i2c_lock);
	if (val)
		reg_val = chip->reg_output | (1u << off);
	else
		reg_val = chip->reg_output & ~(1u << off);

	switch (chip->chip_type) {
	case PCA953X_TYPE:
		offset = PCA953X_OUTPUT;
		break;
	case PCA957X_TYPE:
		offset = PCA957X_OUT;
		break;
	}
	ret = pca953x_write_reg(chip, offset, reg_val);
	if (ret)
		goto exit;

	chip->reg_output = reg_val;
exit:
	mutex_unlock(&chip->i2c_lock);
}

static void pca953x_setup_gpio(struct pca953x_chip *chip, int gpios)
{
	struct gpio_chip *gc;

	gc = &chip->gpio_chip;

	gc->direction_input = pca953x_gpio_direction_input;
	gc->direction_output = pca953x_gpio_direction_output;
	gc->get = pca953x_gpio_get_value;
	gc->set = pca953x_gpio_set_value;
	gc->can_sleep = 1;

	gc->base = chip->gpio_start;
	gc->ngpio = gpios;
	gc->label = chip->client->name;
	gc->dev = &chip->client->dev;
	gc->owner = THIS_MODULE;
	gc->names = chip->names;
}

#ifdef CONFIG_GPIO_PCA953X_IRQ
static int pca953x_gpio_to_irq(struct gpio_chip *gc, unsigned off)
{
	struct pca953x_chip *chip;

	chip = container_of(gc, struct pca953x_chip, gpio_chip);
	return chip->irq_base + off;
}

static void pca953x_irq_mask(struct irq_data *d)
{
	struct pca953x_chip *chip = irq_data_get_irq_chip_data(d);

	chip->irq_mask &= ~(1 << (d->irq - chip->irq_base));
}

static void pca953x_irq_unmask(struct irq_data *d)
{
	struct pca953x_chip *chip = irq_data_get_irq_chip_data(d);

	chip->irq_mask |= 1 << (d->irq - chip->irq_base);
}

static void pca953x_irq_bus_lock(struct irq_data *d)
{
	struct pca953x_chip *chip = irq_data_get_irq_chip_data(d);

	mutex_lock(&chip->irq_lock);
}

static void pca953x_irq_bus_sync_unlock(struct irq_data *d)
{
	struct pca953x_chip *chip = irq_data_get_irq_chip_data(d);
	uint16_t new_irqs;
	uint16_t level;

	/* Look for any newly setup interrupt */
	new_irqs = chip->irq_trig_fall | chip->irq_trig_raise;
	new_irqs &= ~chip->reg_direction;

	while (new_irqs) {
		level = __ffs(new_irqs);
		pca953x_gpio_direction_input(&chip->gpio_chip, level);
		new_irqs &= ~(1 << level);
	}

	mutex_unlock(&chip->irq_lock);
}

static int pca953x_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct pca953x_chip *chip = irq_data_get_irq_chip_data(d);
	uint16_t level = d->irq - chip->irq_base;
	uint16_t mask = 1 << level;

	if (!(type & IRQ_TYPE_EDGE_BOTH)) {
		dev_err(&chip->client->dev, "irq %d: unsupported type %d\n",
			d->irq, type);
		return -EINVAL;
	}

	if (type & IRQ_TYPE_EDGE_FALLING)
		chip->irq_trig_fall |= mask;
	else
		chip->irq_trig_fall &= ~mask;

	if (type & IRQ_TYPE_EDGE_RISING)
		chip->irq_trig_raise |= mask;
	else
		chip->irq_trig_raise &= ~mask;

	return 0;
}

static struct irq_chip pca953x_irq_chip = {
	.name = "pca953x",
	.irq_mask = pca953x_irq_mask,
	.irq_unmask = pca953x_irq_unmask,
	.irq_bus_lock = pca953x_irq_bus_lock,
	.irq_bus_sync_unlock = pca953x_irq_bus_sync_unlock,
	.irq_set_type = pca953x_irq_set_type,
};

static uint16_t pca953x_irq_pending(struct pca953x_chip *chip)
{
	uint16_t cur_stat;
	uint16_t old_stat;
	uint16_t pending;
	uint16_t trigger;
	int ret, offset = 0;

	switch (chip->chip_type) {
	case PCA953X_TYPE:
		offset = PCA953X_INPUT;
		break;
	case PCA957X_TYPE:
		offset = PCA957X_IN;
		break;
	}
	ret = pca953x_read_reg(chip, offset, &cur_stat);
	if (ret)
		return 0;

	/* Remove output pins from the equation */
	cur_stat &= chip->reg_direction;

	old_stat = chip->irq_stat;
	trigger = (cur_stat ^ old_stat) & chip->irq_mask;

	if (!trigger)
		return 0;

	chip->irq_stat = cur_stat;

	pending = (old_stat & chip->irq_trig_fall) |
	    (cur_stat & chip->irq_trig_raise);
	pending &= trigger;

	return pending;
}

#ifndef CONFIG_GPIO_PCA9555_FRI2

static irqreturn_t pca953x_irq_handler(int irq, void *devid)
{
	struct pca953x_chip *chip = devid;
	uint16_t pending;
	uint16_t level;

	pending = pca953x_irq_pending(chip);

	if (!pending)
		return IRQ_HANDLED;

	do {
		level = __ffs(pending);
		handle_nested_irq(level + chip->irq_base);

		pending &= ~(1 << level);
	} while (pending);

	return IRQ_HANDLED;
}

#else

static struct task_struct *pca95xx_irq_thread;

static void pca95xx_gpe_test_chip(struct pca953x_chip *chip)
{
	uint16_t pending;
	uint16_t level;

	pending = pca953x_irq_pending(chip);
	if (!pending)
		return;

	do {
		level = __ffs(pending);
		handle_nested_irq(level + chip->irq_base);
		pending &= ~(1 << level);
	} while (pending);

}

static unsigned long flgs;
enum {
	FLGS_RUNTHREAD,
	FLGS_DIED
};

static int pca95xx_wait_for_gpe(void)
{
	struct sched_param param = {.sched_priority = MAX_USER_RT_PRIO / 2, };

	sched_setscheduler(current, SCHED_FIFO, &param);

	while (!kthread_should_stop()) {
		set_current_state(TASK_INTERRUPTIBLE);

		if (test_and_clear_bit(FLGS_RUNTHREAD, &flgs)) {
			__set_current_state(TASK_RUNNING);
			return 0;
		}
		schedule();
	}
	return -1;
}

static int pca95xx_gpe_test_chips(void *data)
{
	int i;

	while (!pca95xx_wait_for_gpe()) {
		for (i = 0; i < fri2_chip_count; i++)
			pca95xx_gpe_test_chip(fri2_chips[i]);
	}

	return 0;
}

static int pca95xx_irq_thread_setup(void)
{
	pca95xx_irq_thread = kthread_create(pca95xx_gpe_test_chips,
					    fri2_chips, "pca95xx-irq");
	if (IS_ERR(pca95xx_irq_thread))
		return PTR_ERR(pca95xx_irq_thread);
	else
		return 0;
}

/*
 * one GPE handler for BOTH pca9555 chips. call pca95xx_gpe_test_chip
 * for each chip.
 */
static u32 pca95xx_gpe_handler(void *context)
{

	inb(acpi_gbl_FADT.gpe0_block + GPEC);
	outb(0, acpi_gbl_FADT.gpe0_block + GPEC);
	set_bit(FLGS_RUNTHREAD, &flgs);
	wake_up_process(pca95xx_irq_thread);

	return ACPI_INTERRUPT_HANDLED;

}

#endif

static int pca953x_irq_setup(struct pca953x_chip *chip,
			     const struct i2c_device_id *id, int irq_base)
{
	struct i2c_client *client = chip->client;
	int ret, offset = 0;

	if (irq_base != -1 && (id->driver_data & PCA_INT)) {
		int lvl;

		switch (chip->chip_type) {
		case PCA953X_TYPE:
			offset = PCA953X_INPUT;
			break;
		case PCA957X_TYPE:
			offset = PCA957X_IN;
			break;
		}
		ret = pca953x_read_reg(chip, offset, &chip->irq_stat);
		if (ret)
			goto out_failed;

		/*
		 * There is no way to know which GPIO line generated the
		 * interrupt.  We have to rely on the previous read for
		 * this purpose.
		 */
		chip->irq_stat &= chip->reg_direction;
		mutex_init(&chip->irq_lock);

		chip->irq_base =
		    irq_alloc_descs(-1, irq_base, chip->gpio_chip.ngpio, -1);
		if (chip->irq_base < 0)
			goto out_failed;

		for (lvl = 0; lvl < chip->gpio_chip.ngpio; lvl++) {
			int irq = lvl + chip->irq_base;

			irq_clear_status_flags(irq, IRQ_NOREQUEST);
			irq_set_chip_data(irq, chip);
			irq_set_chip(irq, &pca953x_irq_chip);
			irq_set_nested_thread(irq, true);
#ifdef CONFIG_ARM
			set_irq_flags(irq, IRQF_VALID);
#else
			irq_set_noprobe(irq);
#endif
		}

#ifndef CONFIG_GPIO_PCA9555_FRI2
		ret = request_threaded_irq(client->irq,
					   NULL,
					   pca953x_irq_handler,
					   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					   dev_name(&client->dev), chip);
		if (ret) {
			dev_err(&client->dev, "failed to request irq %d\n",
				client->irq);
			goto out_failed;
		}
#else
		/* install handler only once - for te first pca9555 chip. */
		if (fri2_chip_count == 1) {
			ret = pca95xx_irq_thread_setup();
			if (ret) {
				printk(KERN_WARNING
				       "pca95xx: unable to create GPE thread\n");
				goto out_failed;
			}

			ret = acpi_install_gpe_handler(NULL,
						       GPIO_GPE,
						       ACPI_GPE_EDGE_TRIGGERED,
						       &pca95xx_gpe_handler,
						       &fri2_chips);
			if (ret != AE_OK) {
				printk(KERN_WARNING
				       "pca95xx: unable to claim ACPI GPE %d\n",
				       GPIO_GPE);
				goto out_failed;
			} else {
				printk("  Using ACPI GPE %d\n", GPIO_GPE);
			}
		}
#endif

		chip->gpio_chip.to_irq = pca953x_gpio_to_irq;
	}

	return 0;

out_failed:
#ifdef CONFIG_GPIO_PCA9555_FRI2
	if (pca95xx_irq_thread) {
		kthread_stop(pca95xx_irq_thread);
		kfree(pca95xx_irq_thread);
		pca95xx_irq_thread = 0;
	}
#endif
	chip->irq_base = -1;
	return ret;
}

static void pca953x_irq_teardown(struct pca953x_chip *chip)
{
#ifndef CONFIG_GPIO_PCA9555_FRI2
	if (chip->irq_base != -1) {
		irq_free_descs(chip->irq_base, chip->gpio_chip.ngpio);
		free_irq(chip->client->irq, chip);
	}
#else
	if (fri2_chip_count-- == 0) {
		acpi_remove_gpe_handler(NULL, GPIO_GPE, &pca95xx_gpe_handler);
		if (pca95xx_irq_thread) {
			kthread_stop(pca95xx_irq_thread);
			kfree(pca95xx_irq_thread);
			pca95xx_irq_thread = 0;
		}
	}
#endif
}
#else /* CONFIG_GPIO_PCA953X_IRQ */
static int pca953x_irq_setup(struct pca953x_chip *chip,
			     const struct i2c_device_id *id, int irq_base)
{
	struct i2c_client *client = chip->client;

	if (irq_base != -1 && (id->driver_data & PCA_INT))
		dev_warn(&client->dev, "interrupt support not compiled in\n");

	return 0;
}

static void pca953x_irq_teardown(struct pca953x_chip *chip)
{
}
#endif

/*
 * Handlers for alternative sources of platform_data
 */
#ifdef CONFIG_OF_GPIO
/*
 * Translate OpenFirmware node properties into platform_data
 * WARNING: This is DEPRECATED and will be removed eventually!
 */
static void
pca953x_get_alt_pdata(struct i2c_client *client, int *gpio_base, int *invert)
{
	struct device_node *node;
	const __be32 *val;
	int size;

	node = client->dev.of_node;
	if (node == NULL)
		return;

	*gpio_base = -1;
	val = of_get_property(node, "linux,gpio-base", &size);
	WARN(val, "%s: device-tree property 'linux,gpio-base' is deprecated!",
	     __func__);
	if (val) {
		if (size != sizeof(*val))
			dev_warn(&client->dev, "%s: wrong linux,gpio-base\n",
				 node->full_name);
		else
			*gpio_base = be32_to_cpup(val);
	}

	val = of_get_property(node, "polarity", NULL);
	WARN(val, "%s: device-tree property 'polarity' is deprecated!",
	     __func__);
	if (val)
		*invert = *val;
}
#else
static void
pca953x_get_alt_pdata(struct i2c_client *client, int *gpio_base, int *invert)
{
	*gpio_base = -1;
}
#endif

static int __devinit device_pca953x_init(struct pca953x_chip *chip, int invert)
{
	int ret;

	ret = pca953x_read_reg(chip, PCA953X_OUTPUT, &chip->reg_output);
	if (ret)
		goto out;

	ret = pca953x_read_reg(chip, PCA953X_DIRECTION, &chip->reg_direction);
	if (ret)
		goto out;

	/* set platform specific polarity inversion */
	ret = pca953x_write_reg(chip, PCA953X_INVERT, invert);
out:
	return ret;
}

static int __devinit device_pca957x_init(struct pca953x_chip *chip, int invert)
{
	int ret;
	uint16_t val = 0;

	/* Let every port in proper state, that could save power */
	pca953x_write_reg(chip, PCA957X_PUPD, 0x0);
	pca953x_write_reg(chip, PCA957X_CFG, 0xffff);
	pca953x_write_reg(chip, PCA957X_OUT, 0x0);

	ret = pca953x_read_reg(chip, PCA957X_IN, &val);
	if (ret)
		goto out;
	ret = pca953x_read_reg(chip, PCA957X_OUT, &chip->reg_output);
	if (ret)
		goto out;
	ret = pca953x_read_reg(chip, PCA957X_CFG, &chip->reg_direction);
	if (ret)
		goto out;

	/* set platform specific polarity inversion */
	pca953x_write_reg(chip, PCA957X_INVRT, invert);

	/* To enable register 6, 7 to controll pull up and pull down */
	pca953x_write_reg(chip, PCA957X_BKEN, 0x202);

	return 0;
out:
	return ret;
}

static int __devinit pca953x_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct pca953x_platform_data *pdata;
	struct pca953x_chip *chip;
	int irq_base = 0, invert = 0;
	int ret;

	chip = kzalloc(sizeof(struct pca953x_chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

#ifdef CONFIG_GPIO_PCA9555_FRI2
	fri2_chips[fri2_chip_count++] = chip;
#endif
	pdata = client->dev.platform_data;
	if (pdata) {
		irq_base = pdata->irq_base;
		chip->gpio_start = pdata->gpio_base;
		invert = pdata->invert;
		chip->names = pdata->names;
	} else {
		pca953x_get_alt_pdata(client, &chip->gpio_start, &invert);
#ifdef CONFIG_OF_GPIO
		/* If I2C node has no interrupts property, disable GPIO interrupts */
		if (of_find_property(client->dev.of_node, "interrupts", NULL) ==
		    NULL)
			irq_base = -1;
#endif
	}

	chip->client = client;

	chip->chip_type = id->driver_data & (PCA953X_TYPE | PCA957X_TYPE);

	mutex_init(&chip->i2c_lock);

	/* initialize cached registers from their original values.
	 * we can't share this chip with another i2c master.
	 */
	pca953x_setup_gpio(chip, id->driver_data & PCA_GPIO_MASK);

	if (chip->chip_type == PCA953X_TYPE)
		ret = device_pca953x_init(chip, invert);
	else
		ret = device_pca957x_init(chip, invert);
	if (ret)
		goto out_failed;

	ret = pca953x_irq_setup(chip, id, irq_base);
	printk(KERN_INFO "chip->irq_base is %d\n", chip->irq_base);
	if (ret)
		goto out_failed;

	ret = gpiochip_add(&chip->gpio_chip);
	if (ret)
		goto out_failed_irq;

	if (pdata && pdata->setup) {
		ret = pdata->setup(client, chip->gpio_chip.base,
				   chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0)
			dev_warn(&client->dev, "setup failed, %d\n", ret);
	}

	i2c_set_clientdata(client, chip);
	return 0;

out_failed_irq:
	pca953x_irq_teardown(chip);
out_failed:
	kfree(chip);
	return ret;
}

static int pca953x_remove(struct i2c_client *client)
{
	struct pca953x_platform_data *pdata = client->dev.platform_data;
	struct pca953x_chip *chip = i2c_get_clientdata(client);
	int ret = 0;

	if (pdata && pdata->teardown) {
		ret = pdata->teardown(client, chip->gpio_chip.base,
				      chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0) {
			dev_err(&client->dev, "%s failed, %d\n",
				"teardown", ret);
			return ret;
		}
	}

	ret = gpiochip_remove(&chip->gpio_chip);
	if (ret) {
		dev_err(&client->dev, "%s failed, %d\n",
			"gpiochip_remove()", ret);
		return ret;
	}

	pca953x_irq_teardown(chip);
	kfree(chip);
	return 0;
}

static struct i2c_driver pca953x_driver = {
	.driver = {
		   .name = "pca953x",
		   },
	.probe = pca953x_probe,
	.remove = pca953x_remove,
	.id_table = pca953x_id,
#ifdef CONFIG_GPIO_PCA9555_FRI2
	.class = I2C_CLASS_HWMON,
	.detect = pca953x_i2c_detect,
	.address_list = normal_i2c,
#endif
};

static int __init pca953x_init(void)
{
	return i2c_add_driver(&pca953x_driver);
}

#ifndef CONFIG_GPIO_PCA9555_FRI2
/* register after i2c postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
subsys_initcall(pca953x_init);
#else
/* on FRI2 we can init gpio _after_ kontron CPLD
 * drivers kempld-core and i2c-kempld
 */
module_init(pca953x_init);
#endif
static void __exit pca953x_exit(void)
{
	i2c_del_driver(&pca953x_driver);
}

module_exit(pca953x_exit);

MODULE_AUTHOR("eric miao <eric.miao@marvell.com>");
MODULE_DESCRIPTION("GPIO expander driver for PCA953x");
MODULE_LICENSE("GPL");
