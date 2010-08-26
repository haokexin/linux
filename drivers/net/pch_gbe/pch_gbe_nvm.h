/*!
 * @file pch_gbe_nvm.h
 * @brief Linux PCH Gigabit Ethernet Driver HAL internal function (NVM) header file
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
#ifndef _PCH_GBE_NVM_H_
#define _PCH_GBE_NVM_H_

#ifdef CONFIG_PCH_PHUB
/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_nvm_read_mem(struct pch_gbe_hw *hw,
 *                                   u32 offset, u8 *data)
 * @brief   Read EEPROM
 */
s32 pch_gbe_nvm_read_mem(struct pch_gbe_hw *hw, u32 offset, u8 * data);

/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_nvm_write_mem(struct pch_gbe_hw *hw,
 *                                    u32 offset, u8 *data)
 * @brief   Write EEPROM
 */
s32 pch_gbe_nvm_write_mem(struct pch_gbe_hw *hw, u32 offset, u8 * data);

/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_nvm_read_mac_addr(struct pch_gbe_hw *hw)
 * @brief   Read device MAC address
 */
s32 pch_gbe_nvm_read_mac_addr(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_nvm_validate_checksum(struct pch_gbe_hw *hw)
 * @brief   Validate EEPROM checksum
 */
s32 pch_gbe_nvm_validate_checksum(struct pch_gbe_hw *hw);

/*!
 * @ingroup PCIe QoS Driver function
 * @fn      int pch_phub_read_gbe_mac_addr (unsigned long offset_address,
 *                                              unsigned char *data);
 * @brief   Read MAC address from NVM
 */
int pch_phub_read_gbe_mac_addr(unsigned long offset_address,
      unsigned char *data);

/*!
 * @ingroup PCIe QoS Driver function
 * @fn      int pch_phub_write_gbe_mac_addr(unsigned long offset_address,
 *                                              unsigned char data);
 * @brief   Write MAC address from NVM
 */
int pch_phub_write_gbe_mac_addr(unsigned long offset_address,
      unsigned char data);
#endif /* CONFIG_PCH_PHUB */

#endif /* _PCH_GBE_NVM_H_ */
