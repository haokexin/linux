/*
 * drivers/acp/common/debug.h
 *
 * Copyright (C) 2010 LSI
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307	 USA
 */

#ifndef __DRIVERS_LSI_COMMON_DEBUG_H
#define __DRIVERS_LSI_COMMON_DEBUG_H

/*
  DEBUG
*/

#if defined(DEBUG)
#define DEBUG_PRINT(format, args...) do { \
printk("%s:%s:%d - DEBUG - ", __FILE__, __FUNCTION__, __LINE__); \
printk(format, ##args); \
} while (0);
#else
#define DEBUG_PRINT(format, args...) { }
#endif

/*
  WARN
*/

#if defined(WARN)
#define WARN_PRINT(format, args...) do { \
printk("%s:%s:%d - WARN - ", __FILE__, __FUNCTION__, __LINE__); \
printk(format, ##args); \
} while (0);
#else
#define WARN_PRINT(format, args...) { }
#endif

/*
  ERROR
*/

#if defined(ERROR)
#define ERROR_PRINT(format, args...) do { \
printk("%s:%s:%d - ERROR - ", __FILE__, __FUNCTION__, __LINE__); \
printk(format, ##args); \
} while (0);
#else
#define ERROR_PRINT(format, args...) { }
#endif

#endif /* __DRIVERS_LSI_COMMON_DEBUG_H */
