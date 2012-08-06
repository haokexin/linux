/*
 * arch/arm/plat-spear/include/plat/gpio.h
 *
 * GPIO macros for SPEAr platform
 *
 * Copyright (C) 2009 ST Microelectronics
 * Viresh Kumar<viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __PLAT_GPIO_H
#define __PLAT_GPIO_H

#include <asm-generic/gpio.h>
#include <linux/types.h>

/* plgpio driver declarations */
/*
 * plgpio pins in all machines are not one to one mapped, bitwise with
 * registers bits. These set of macros define register masks for which below
 * functions (pin_to_offset and offset_to_pin) are required to be called.
 */
#define PTO_ENB_REG		0x001
#define PTO_WDATA_REG		0x002
#define PTO_DIR_REG		0x004
#define PTO_IE_REG		0x008
#define PTO_RDATA_REG		0x010
#define PTO_MIS_REG		0x020

struct plgpio_regs {
	u32 enb;		/* enable register */
	u32 wdata;		/* write data register */
	u32 dir;		/* direction set register */
	u32 rdata;		/* read data register */
	u32 ie;			/* interrupt enable register */
	u32 mis;		/* mask interrupt status register */
	u32 eit;		/* edge interrupt type */
};

/* functions for converting pin to correct offset in register and vice versa */
/**
 * struct plgpio_platform_data: plgpio driver platform data
 *
 * gpio_base: gpio start number of plgpios
 * irq_base: irq number of plgpio0
 * gpio_count: total count of plgpios
 * grp_size: number of gpio's in a group for interrupt registers
 * p2o: function ptr for pin to offset conversion. This is required only for
 * machines where mapping b/w pin and offset is not 1-to-1.
 * o2p: function ptr for offset to pin conversion. This is required only for
 * machines where mapping b/w pin and offset is not 1-to-1.
 * p2o_regs: mask of registers for which p2o and o2p are applicable
 */
struct plgpio_platform_data {
	u32 gpio_base;
	u32 irq_base;
	u32 gpio_count;
	u32 grp_size;
	int (*p2o)(int pin);		/* pin_to_offset */
	int (*o2p)(int offset);		/* offset_to_pin */
	u32 p2o_regs;
	struct plgpio_regs regs;
};

struct gpio_req_list {
	int start;
	int end;
};

static inline int request_gpio(struct gpio_req_list *gpio_list,
		unsigned long flags, int cnt)
{
	int i, j, err;

	for (j = 0; j < cnt; j++) {
		for (i = gpio_list[j].start; i <= gpio_list[j].end; i++) {
			err = gpio_request_one(i, flags, "gpio");
			if (err)
				pr_err("GPIO request is fail %d", i);
		}
	}
	return 0;
}

#endif /* __PLAT_GPIO_H */
