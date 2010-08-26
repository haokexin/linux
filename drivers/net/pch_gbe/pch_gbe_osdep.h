/*!
 * @file pch_gbe_osdep.h
 * @brief Linux PCH Gigabit Ethernet Driver OS independent header file
 *
 *        glue for the OS independent part of pch
 *        includes register access macros
 *
 * @version 1.00
 *
 * @section
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307, USA.
 */

/*
 * History:
 * Copyright (C) 2010 OKI SEMICONDUCTOR CO., LTD.
 *
 * created:
 *   OKI SEMICONDUCTOR 04/13/2010
 * modified:
 *
 */
#ifndef _PCH_GBE_OSDEP_H_
#define _PCH_GBE_OSDEP_H_

#include <linux/delay.h>
#include <linux/io.h>

#define usec_delay(x) udelay(x)
#ifndef msec_delay
#define msec_delay(x) \
  do {if (in_interrupt()) { \
   /* Don't mdelay in interrupt context! */ \
   BUG(); \
  } else { \
   msleep(x); \
  } } while (0)

/* Some workarounds require millisecond delays and are run during interrupt
 * context.  Most notably, when establishing link, the phy may need tweaking
 * but cannot process phy register reads/writes faster than millisecond
 * intervals...and we establish link due to a "link status change" interrupt.
 */
#define msec_delay_irq(x) mdelay(x)
#endif

#undef FALSE
#define FALSE 0
#undef TRUE
#define TRUE 1


#define PCH_GBE_WRITE_REG(a, reg, value) ( \
  writel((value), ((a)->hw_addr + PCH_GBE_##reg)))

#define PCH_GBE_READ_REG(a, reg) ( \
  readl((a)->hw_addr + PCH_GBE_##reg))

#define PCH_GBE_WRITE_REG_ARRAY(a, reg, offset, value) ( \
  writel((value), \
  ((a)->hw_addr + PCH_GBE_##reg + ((offset) << 2))))

#endif /* _PCH_GBE_OSDEP_H_ */
