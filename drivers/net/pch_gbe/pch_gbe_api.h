/*!
 * @file pch_gbe_api.h
 * @brief Linux PCH Gigabit Ethernet Driver HAL API header file
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

#ifndef _PCH_GBE_API_H_
#define _PCH_GBE_API_H_

/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_set_mac_type(struct pch_gbe_hw *hw)
 * @brief   Sets MAC type
 */
s32 pch_gbe_hal_set_mac_type(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_setup_init_funcs(struct pch_gbe_hw *hw)
 * @brief   Initializes function pointers
 */
s32 pch_gbe_hal_setup_init_funcs(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL API Layer
 * @fn      void pch_gbe_hal_get_bus_info(struct pch_gbe_hw *hw)
 * @brief   Obtain bus information for adapter
 */
void pch_gbe_hal_get_bus_info(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL API Layer
 * @fn      void pch_gbe_hal_mc_addr_list_update(struct pch_gbe_hw *hw,
 *               u8 *mc_addr_list, u32 mc_addr_count,
 *               u32 mar_used_count, u32 mar_count)
 * @brief   Update Multicast addresses
 */
void pch_gbe_hal_mc_addr_list_update(struct pch_gbe_hw *hw,
     u8 *mc_addr_list, u32 mc_addr_count,
     u32 mar_used_count, u32 mar_count);

/*
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_force_mac_fc(struct pch_gbe_hw *hw)
 * @brief   Force MAC flow control
 */
s32 pch_gbe_hal_force_mac_fc(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_reset_hw(struct pch_gbe_hw *hw)
 * @brief   Reset hardware
 */
s32 pch_gbe_hal_reset_hw(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_init_hw(struct pch_gbe_hw *hw)
 * @brief   Initialize hardware
 */
s32 pch_gbe_hal_init_hw(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_setup_link(struct pch_gbe_hw *hw)
 * @brief   Configures link and flow control
 */
s32 pch_gbe_hal_setup_link(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_setup_led(struct pch_gbe_hw *hw)
 * @brief   Configures SW controllable LED
 */
s32 pch_gbe_hal_setup_led(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_cleanup_led(struct pch_gbe_hw *hw)
 * @brief   Restores SW controllable LED
 */
s32 pch_gbe_hal_cleanup_led(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_led_on(struct pch_gbe_hw *hw)
 * @brief   Turn on SW controllable LED
 */
s32 pch_gbe_hal_led_on(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_led_off(struct pch_gbe_hw *hw)
 * @brief   Turn off SW controllable LED
 */
s32 pch_gbe_hal_led_off(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL API Layer
 * @fn      void pch_gbe_hal_mar_set(struct pch_gbe_hw *hw, u8 *addr, u32 index)
 * @brief   Sets a MAC address register
 */
void pch_gbe_hal_mar_set(struct pch_gbe_hw *hw, u8 *addr, u32 index);

/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_read_phy_reg(struct pch_gbe_hw *hw,
 *      u32 offset, u16 *data)
 * @brief   Reads PHY register
 */
s32 pch_gbe_hal_read_phy_reg(struct pch_gbe_hw *hw, u32 offset, u16 *data);

/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_write_phy_reg(struct pch_gbe_hw *hw,
 *      u32 offset, u16 data)
 * @brief   Writes PHY register
 */
s32 pch_gbe_hal_write_phy_reg(struct pch_gbe_hw *hw, u32 offset, u16 data);

/*!
 * @ingroup HAL API Layer
 * @fn      void pch_gbe_hal_phy_hw_reset(struct pch_gbe_hw *hw)
 * @brief   Hard PHY reset
 */
void pch_gbe_hal_phy_hw_reset(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL API Layer
 * @fn      void pch_gbe_hal_phy_sw_reset(struct pch_gbe_hw *hw)
 * @brief   Soft PHY reset
 */
void pch_gbe_hal_phy_sw_reset(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_read_mac_addr(struct pch_gbe_hw *hw)
 * @brief   Reads MAC address
 */
s32 pch_gbe_hal_read_mac_addr(struct pch_gbe_hw *hw);

#ifdef CONFIG_PCH_PHUB
/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_validate_nvm_checksum(struct pch_gbe_hw *hw)
 * @brief   Verifies NVM (EEPROM) checksum
 */
s32 pch_gbe_hal_validate_nvm_checksum(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_read_nvm(struct pch_gbe_hw *hw,
 *     u32 offset, u8 *data)
 * @brief   Reads NVM (EEPROM)
 */
s32 pch_gbe_hal_read_nvm(struct pch_gbe_hw *hw, u32 offset, u8 *data);

/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_write_nvm(struct pch_gbe_hw *hw,
 *     u32 offset, u8 *data)
 * @brief   Writes to NVM (EEPROM)
 */
s32 pch_gbe_hal_write_nvm(struct pch_gbe_hw *hw, u32 offset, u8 *data);
#endif /* CONFIG_PCH_PHUB */

/*!
 * @ingroup HAL API Layer
 * @fn      void pch_gbe_hal_set_wol_event(struct pch_gbe_hw *hw, u32 wu_evt)
 * @brief   Set wake-on-lan event
 */
void pch_gbe_hal_set_wol_event(struct pch_gbe_hw *hw, u32 wu_evt);

/*!
 * @ingroup HAL API Layer
 * @fn      void pch_gbe_hal_power_up_phy(struct pch_gbe_hw *hw)
 * @brief   Power up PHY
 */
void pch_gbe_hal_power_up_phy(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL API Layer
 * @fn      void pch_gbe_hal_power_down_phy(struct pch_gbe_hw *hw)
 * @brief   Power down PHY
 */
void pch_gbe_hal_power_down_phy(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL API Layer
 * @fn      u16 pch_gbe_hal_ctrl_miim(struct pch_gbe_hw *hw,
 *                                    u32 addr, u32 dir, u32 reg, u16 data)
 * @brief   Control MII Management IF
 */
u16 pch_gbe_hal_ctrl_miim(struct pch_gbe_hw *hw, u32 addr, u32 dir, u32 reg,
     u16 data);

/*!
 * @ingroup HAL API Layer
 * @fn      void pch_gbe_hal_set_pause_packet(struct pch_gbe_hw *hw)
 * @brief   Set pause packet
 */
void pch_gbe_hal_set_pause_packet(struct pch_gbe_hw *hw);

/*!
 * @ingroup  HAL API Layer
 * @def      PCH_GBE_HAL_MIIM_READ
 * @brief    Read operation is done through MII Management IF
 */
#define PCH_GBE_HAL_MIIM_READ          ((u32)0x00000000)

/*!
 * @ingroup  HAL API Layer
 * @def      PCH_GBE_HAL_MIIM_WRITE
 * @brief    Write operation is done through MII Management IF
 */
#define PCH_GBE_HAL_MIIM_WRITE         ((u32)0x04000000)

/* pch_gbe_plat.c */
/*!
 * @ingroup HAL internal functions
 * @fn      void pch_gbe_plat_init_function_pointers(struct pch_gbe_hw *hw)
 * @brief   Init func ptrs.
 */
void pch_gbe_plat_init_function_pointers(struct pch_gbe_hw *hw);

#endif
