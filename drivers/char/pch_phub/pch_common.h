/*!
 * @file pch_common.h
 * @brief Provides the macro definitions used by all files.
 * @version 1.0.0.0
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
 * Copyright (C) 2010 OKI SEMICONDUCTOR Co., LTD.
 *
 * created:
 * OKI SEMICONDUCTOR 04/14/2010
 * modified:
 *
 */

#ifndef __PCH_COMMON_H__
#define __PCH_COMMON_H__

/*! @ingroup Global
@def      PCH_WRITE8
@brief   Macro for writing 8 bit data to an io/mem address
*/
#define PCH_WRITE8(val, addr)   iowrite8((val), (void __iomem *)(addr))
/*! @ingroup Global
@def      PCH_LOG
@brief   Macro for writing 16 bit data to an io/mem address
*/
#define PCH_WRITE16(val, addr)  iowrite16((val), (void __iomem *)(addr))
/*! @ingroup Global
@def      PCH_LOG
@brief   Macro for writing 32 bit data to an io/mem address
*/
#define PCH_WRITE32(val, addr)  iowrite32((val), (void __iomem *)(addr))

/*! @ingroup Global
@def      PCH_READ8
@brief   Macro for reading 8 bit data from an io/mem address
*/
#define PCH_READ8(addr)   ioread8((void __iomem *)(addr))
/*! @ingroup Global
@def      PCH_READ16
@brief   Macro for reading 16 bit data from an io/mem address
*/
#define PCH_READ16(addr)  ioread16((void __iomem *)(addr))
/*! @ingroup Global
@def      PCH_READ32
@brief   Macro for reading 32 bit data from an io/mem address
*/
#define PCH_READ32(addr)  ioread32((void __iomem *)(addr))
/*! @ingroup Global
@def      PCH_WRITE32_F
@brief   Macro for writing 32 bit data to an io/mem address
*/
#define PCH_WRITE32_F(val, addr) \
do { \
	PCH_WRITE32((val), (addr)); \
	(void)PCH_READ32((addr)); \
} while (0);

/*! @ingroup Global
@def      PCH_WRITE_BYTE
@brief   Macro for writing 1 byte data to an io/mem address
*/
#define PCH_WRITE_BYTE PCH_WRITE8
/*! @ingroup Global
@def      PCH_WRITE_WORD
@brief   Macro for writing 1 word data to an io/mem address
*/
#define PCH_WRITE_WORD PCH_WRITE16
/*! @ingroup Global
@def      PCH_WRITE_LONG
@brief   Macro for writing long data to an io/mem address
*/
#define PCH_WRITE_LONG PCH_WRITE32

/*! @ingroup Global
@def      PCH_READ_BYTE
@brief   Macro for reading 1 byte data from an io/mem address
*/
#define PCH_READ_BYTE  PCH_READ8
/*! @ingroup Global
@def      PCH_READ_WORD
@brief   Macro for reading 1 word data from an io/mem address
*/
#define PCH_READ_WORD  PCH_READ16
/*! @ingroup Global
@def      PCH_READ_LONG
@brief   Macro for reading long data from an io/mem address
*/
#define PCH_READ_LONG  PCH_READ32

/* Bit Manipulation Macros */

/*! @ingroup Global
@def      PCH_READ_LONG
@brief   macro to set a specified bit(mask) at the
   specified address
*/
#define PCH_SET_ADDR_BIT(addr, bitmask) PCH_WRITE_LONG((PCH_READ_LONG(addr) |\
	(bitmask)), (addr))

/*! @ingroup Global
@def     PCH_READ_LONG
@brief  macro to clear a specified bit(mask) at the specified address
*/
#define PCH_CLR_ADDR_BIT(addr, bitmask) PCH_WRITE_LONG((PCH_READ_LONG(addr) &\
	~(bitmask)), (addr))

/*! @ingroup Global
@def      PCH_READ_LONG
@brief   macro to set a specified bitmask for a variable
*/
#define PCH_SET_BITMSK(var, bitmask) ((var) |= (bitmask))

/*! @ingroup Global
@def      PCH_READ_LONG
@brief   macro to clear a specified bitmask for a variable
*/
#define PCH_CLR_BITMSK(var, bitmask) ((var) &= (~(bitmask)))

/*! @ingroup Global
@def      PCH_READ_LONG
@brief   macro to set a specified bit for a variable
*/
#define PCH_SET_BIT(var, bit) ((var) |= (1<<(bit)))

/*! @ingroup Global
@def      PCH_READ_LONG
@brief   macro to clear a specified bit for a variable
*/
#define PCH_CLR_BIT(var, bit) ((var) &= ~(1<<(bit)))

#endif
