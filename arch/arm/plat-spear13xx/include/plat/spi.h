/*
 * arch/arm/plat-spear/include/plat/spi.h
 *
 * SPI board specific definitions common to multiple boards on multiple
 * machines.
 *
 * Copyright (C) 2010 ST Microelectronics
 * Viresh Kumar<viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __PLAT_SPI_H
#define __PLAT_SPI_H

#include <linux/amba/pl022.h>
#include <linux/gpio.h>
#include <linux/types.h>

/* Chip select control using SSP peripherals inbuilt chip select lines */
/*
 * DECLARE_SPI_CS_CFG: chip select configuration routine for spi master
 * @id: ssp id
 * @reg: virtual address of register for configuring
 * @cs_sel_mask: chip select lines selection bit mask
 * @cs_sel_shift: chip select lines selection bit shift
 * @cs_ctl_mask: chip selection ctl mask (h/w or s/w)
 * @cs_ctl_shift: chip selection ctl shift (h/w or s/w)
 * @cs_val_mask: chip selection value bit mask
 * @cs_val_shift: chip selection value bit shift
 */
#define DECLARE_SPI_CS_CFG(id, reg, cs_sel_mask, cs_sel_shift, cs_ctl_mask, \
		cs_ctl_shift, cs_ctl_val, cs_val_mask, cs_val_shift)	\
/* cs_sel: cs line to select, val: value to write on cs line */		\
static void spi##id##_cs_cfg(u32 cs_sel, u32 val)			\
{									\
	static int count;						\
	u32 tmp = readl(reg);						\
									\
	if (unlikely(!count)) {						\
		count++;						\
		tmp &= ~(cs_ctl_mask << cs_ctl_shift);			\
		tmp |= cs_ctl_val << cs_ctl_shift;			\
		writel(tmp, reg);					\
	}								\
									\
	tmp &= ~(cs_sel_mask << cs_sel_shift);				\
	tmp |= cs_sel << cs_sel_shift;					\
	writel(tmp, reg);						\
	tmp &= ~(cs_val_mask << cs_val_shift);				\
	tmp |= val << cs_val_shift;					\
	writel(tmp, reg);						\
}

/*
 * cs_control function for a specific spi slave using spi's cs signals
 * @id: spi master id
 * @type: slave type, will used in routines name
 * @cs_sel: chip select number
 */
#define DECLARE_SPI_CS_CONTROL(id, type, cs_sel)	\
static void spi##id##_##type##_cs_control(u32 control)	\
{							\
	spi##id##_cs_cfg(cs_sel, control);		\
}

/* Chip select control using external GPIO pins */
/* spi board information */
static inline int spi_cs_gpio_request(u32 gpio_pin)
{
	int ret;

	ret = gpio_request(gpio_pin, "SPI_CS");
	if (ret < 0) {
		printk(KERN_ERR "SPI: gpio:%d request fail\n", gpio_pin);
		return ret;
	} else {
		ret = gpio_direction_output(gpio_pin, 1);
		if (ret) {
			printk(KERN_ERR "SPI: gpio:%d direction set fail\n",
					gpio_pin);
			return ret;
		}
	}
	return 0;
}

/* This will define cs_control function for a specific spi slave using gpios */
#define DECLARE_SPI_CS_GPIO_CONTROL(id, type, gpio)	\
static void spi##id##_##type##_cs_gpio_control(u32 control)	\
{							\
	static int count, ret;				\
							\
	if (unlikely(!count)) {				\
		count++;				\
		ret = spi_cs_gpio_request(gpio);	\
	}						\
							\
	if (!ret)					\
		gpio_set_value(gpio, control);		\
}

/* This will define CHIP_INFO structure for a specific spi slave */
#define DECLARE_SPI_CHIP_INFO(id, type, chip_select_control)	\
static struct pl022_config_chip spi##id##_##type##_chip_info = {\
	.iface = SSP_INTERFACE_MOTOROLA_SPI,		\
	.hierarchy = SSP_MASTER,			\
	.slave_tx_disable = 0,				\
	.com_mode = DMA_TRANSFER,			\
	.rx_lev_trig = SSP_RX_1_OR_MORE_ELEM,		\
	.tx_lev_trig = SSP_TX_1_OR_MORE_EMPTY_LOC,	\
	.ctrl_len = SSP_BITS_12,			\
	.wait_state = SSP_MWIRE_WAIT_ZERO,		\
	.duplex = SSP_MICROWIRE_CHANNEL_FULL_DUPLEX,	\
	.cs_control = chip_select_control,		\
};

#define DECLARE_SPI_CHIP_INFO_NULL_ID(chip_select_control)	\
DECLARE_SPI_CHIP_INFO(, chip_select_control)

#endif /* __PLAT_SPI_H */
