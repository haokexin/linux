#ifndef __PCH_PHUB_HAL_H__
#define __PCH_PHUB_HAL_H__
/*!
 * @file pch_phub_hal.h
 * @brief Provides all the interfaces pertaining to the HAL.
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

/* exported function prototypes */
/*! @ingroup PHUB_HALLayerAPI
  @fn void pch_phub_read_reg(unsigned long reg_addr_offset,
	unsigned long *data)
  @brief  Provides the functionality of reading register
  */
void pch_phub_read_reg(unsigned long reg_addr_offset, unsigned long *data);

/*! @ingroup PHUB_HALLayerAPI
  @fn pch_phub_write_reg(unsigned long reg_addr_offset, unsigned long data)
  @brief  Provides the functionality of writing register
  */
void pch_phub_write_reg(unsigned long reg_addr_offset, unsigned long data);

/*! @ingroup PHUB_HALLayerAPI
  @fn pch_phub_read_modify_write_reg(unsigned long reg_addr_offset,
      unsigned long data, unsigned long mask)
  @brief  Provides the functionality of reading, modifying and writing register
  */
void pch_phub_read_modify_write_reg(unsigned long reg_addr_offset,
	unsigned long data, unsigned long mask);

/*! @ingroup PHUB_HALLayerAPI
  @fn int pch_phub_read_gbe_mac_addr(unsigned long offset_address,
	unsigned char *data)
  @brief  Provides the functionality of reading Gigabit Ethernet MAC address
  */
int pch_phub_read_gbe_mac_addr(unsigned long offset_address,
	unsigned char *data);

/*! @ingroup PHUB_HALLayerAPI
  @fn int pch_phub_write_gbe_mac_addr(unsigned long offset_address,
	unsigned char data)
  @brief  Provides the functionality of writing Gigabit Ethernet MAC address
  */
int pch_phub_write_gbe_mac_addr(unsigned long offset_address,
	unsigned char data);

/*! @ingroup PHUB_HALLayerAPI
  @fn void pch_phub_save_reg_conf(void)
  @brief  saves register configuration
  */
void pch_phub_save_reg_conf(void);

/*! @ingroup PHUB_HALLayerAPI
  @fn void pch_phub_restore_reg_conf(void)
  @brief  restores register configuration
  */
void pch_phub_restore_reg_conf(void);

/*! @ingroup PHUB_HALLayerAPI
  @fn int pch_phub_read_serial_rom(unsigned long offset_address,
	unsigned char *data)
  @brief  Provides the functionality of reading Serial ROM
  */
int pch_phub_read_serial_rom(unsigned long offset_address,
	unsigned char *data);

/*! @ingroup PHUB_HALLayerAPI
  @fn int pch_phub_write_serial_rom(unsigned long offset_address,
	unsigned char data)
  @brief  Provides the functionality of writing Serial ROM
  */
int pch_phub_write_serial_rom(unsigned long offset_address,
	unsigned char data);

/*! @ingroup PHUB_HALLayerAPI
  @fn int pch_phub_read_serial_rom_val(unsigned long offset_address,
	unsigned char *data)
  @brief  Provides the functionality of reading Serial ROM value
  */
int pch_phub_read_serial_rom_val(unsigned long offset_address,
	unsigned char *data);

/*! @ingroup PHUB_HALLayerAPI
  @fn int pch_phub_write_serial_rom_val(unsigned long offset_address,
	unsigned char data)
  @brief  Provides the functionality of writing Serial ROM value
  */
int pch_phub_write_serial_rom_val(unsigned long offset_address,
	unsigned char data);

/*! @ingroup PHUB_HALLayerAPI
  @fn int pch_phub_gbe_serial_rom_conf(void)
  @brief  makes Serial ROM data format configuration for Gigabit Ethernet
	MAC address
  */
int pch_phub_gbe_serial_rom_conf(void);

/* global variables */
extern u32 pch_phub_base_address;
extern u32 pch_phub_extrom_base_address;
#endif
