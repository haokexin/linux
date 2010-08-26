/*!
 * @file pch_gbe_nvm.c
 * @brief Linux PCH Gigabit Ethernet Driver HAL internal function (NVM) source file
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
#include "pch_debug.h"
#include "pch_gbe_osdep.h"
#include "pch_gbe_defines.h"
#include "pch_gbe_hw.h"
#include "pch_gbe_nvm.h"

#ifdef CONFIG_PCH_PHUB
/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_nvm_read_mem(struct pch_gbe_hw *hw,
 *                                   u32 offset, u8 *data)
 * @brief   Read EEPROM
 * @param   hw     [INOUT] Pointer to the HW structure
 * @param   offset [IN] Offset of word in the EEPROM to read
 * @param   data   [OUT] Word read from the EEPROM
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:   Failed
 */
s32 pch_gbe_nvm_read_mem(struct pch_gbe_hw *hw, u32 offset, u8 *data)
{
 s32 ret;

 PCH_DEBUG("pch_gbe_nvm_read_mem\n");
 PCH_DEBUG("offset : 0x%04x\n", offset);
 ret = pch_phub_read_gbe_mac_addr(offset, data);
 return ret;
}

/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_nvm_write_mem(struct pch_gbe_hw *hw,
 *                                    u32 offset, u8 *data)
 * @brief   Write EEPROM
 * @param   hw     [INOUT] Pointer to the HW structure
 * @param   offset [IN] Offset of word in the EEPROM to read
 * @param   data   [IN] 8bit word(s) to be written to the EEPROM
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:   Failed
 */
s32 pch_gbe_nvm_write_mem(struct pch_gbe_hw *hw, u32 offset, u8 *data)
{
 s32 ret;

 PCH_DEBUG("pch_gbe_nvm_write_mem\n");
 PCH_DEBUG("offset : 0x%04x\n", offset);
 ret = pch_phub_write_gbe_mac_addr(offset, *data);
 return ret;
}

/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_nvm_read_mac_addr(struct pch_gbe_hw *hw)
 * @brief   Read device MAC address
 * @param   hw  [INOUT] Pointer to the HW structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:   Failed
 */
s32 pch_gbe_nvm_read_mac_addr(struct pch_gbe_hw *hw)
{
 s32 ret;
 u8 i;
 u8 *data;

 PCH_DEBUG("pch_gbe_nvm_read_mac_addr\n");

#ifdef NVM_MAC_FIX
 hw->mac.addr[0] = (u8) (0x00);
 hw->mac.addr[1] = (u8) (0x21);
 hw->mac.addr[2] = (u8) (0x97);
 hw->mac.addr[3] = (u8) (0x77);
 hw->mac.addr[4] = (u8) (0x65);
 hw->mac.addr[5] = (u8) (0x13);
#else
 data = hw->mac.addr;
 for (i = 0; i < (hw->nvm.word_size * 2); i++) {
  ret = pch_phub_read_gbe_mac_addr((u32) i, (data + i));
  if (ret != 0)
   break;
 }
#endif

 PCH_DEBUG("hw->mac.addr : 0x%02x %02x %02x %02x %02x %02x\n",
   hw->mac.addr[0], hw->mac.addr[1], hw->mac.addr[2],
   hw->mac.addr[3], hw->mac.addr[4], hw->mac.addr[5]);
 return ret;
}
#endif
/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_nvm_validate_checksum(struct pch_gbe_hw *hw)
 * @brief   Validate EEPROM checksum
 * @param   hw  [INOUT] Pointer to the HW structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 */
s32 pch_gbe_nvm_validate_checksum(struct pch_gbe_hw *hw)
{
 PCH_DEBUG("pch_gbe_nvm_validate_checksum\n");
 return PCH_GBE_SUCCESS;
}
