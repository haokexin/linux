/*!
 * @file pch_gbe_phy.h
 * @brief Linux PCH Gigabit Ethernet Driver HAL internal function (PHY) header file
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
#ifndef _PCH_GBE_PHY_H_
#define _PCH_GBE_PHY_H_

/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_phy_get_id(struct pch_gbe_hw *hw)
 * @brief   Retrieve the PHY ID and revision
 */
s32  pch_gbe_phy_get_id(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_phy_read_reg_miic(struct pch_gbe_hw *hw,
 *                                        u32 offset, u16 *data)
 * @brief   Read MII control register
 */
s32 pch_gbe_phy_read_reg_miic(struct pch_gbe_hw *hw, u32 offset, u16 *data);

/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_phy_write_reg_miic(struct pch_gbe_hw *hw,
 *                                         u32 offset, u16 data)
 * @brief   Write MII control register
 */
s32 pch_gbe_phy_write_reg_miic(struct pch_gbe_hw *hw, u32 offset, u16 data);

/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_phy_setup_link_fpga(struct pch_gbe_hw *hw)
 * @brief   Configure link settings for FPGA
 */
s32  pch_gbe_phy_setup_link_fpga(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL internal function
 * @fn      void pch_gbe_phy_sw_reset(struct pch_gbe_hw *hw)
 * @brief   PHY software reset
 */
void pch_gbe_phy_sw_reset(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL internal function
 * @fn      void pch_gbe_phy_hw_reset(struct pch_gbe_hw *hw)
 * @brief   PHY hardware reset
 */
void pch_gbe_phy_hw_reset(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_phy_led_on(struct pch_gbe_hw *hw)
 * @brief   Set ting of led on
 */
s32 pch_gbe_phy_led_on(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_phy_led_off(struct pch_gbe_hw *hw)
 * @brief   Set ting of led off
 */
s32 pch_gbe_phy_led_off(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_phy_led_cleanup(struct pch_gbe_hw *hw)
 * @brief   Cleanup led control
 */
s32 pch_gbe_phy_led_cleanup(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_phy_led_setup(struct pch_gbe_hw *hw)
 * @brief   Setup led control
 */
s32 pch_gbe_phy_led_setup(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL internal function
 * @fn      void pch_gbe_phy_power_up(struct pch_gbe_hw *hw)
 * @brief   restore link in case the phy was powered down
 */
void pch_gbe_phy_power_up(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL internal function
 * @fn      void pch_gbe_phy_power_down(struct pch_gbe_hw *hw)
 * @brief   Power down PHY
 */
void pch_gbe_phy_power_down(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL internal function
 * @fn      void pch_gbe_phy_set_rgmii(struct pch_gbe_hw *hw)
 * @brief   RGMII interface setting
 */
void pch_gbe_phy_set_rgmii(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL internal function
 * @fn      void pch_gbe_phy_init_setting(struct pch_gbe_hw *hw)
 * @brief   PHY initial setting
 */
void pch_gbe_phy_init_setting(struct pch_gbe_hw *hw);


#endif /* _PCH_GBE_PHY_H_ */
