/*!
 * @file pch_gbe_mac.h
 * @brief Linux PCH Gigabit Ethernet Driver HAL internal function (MAC) header file
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
#ifndef _PCH_GBE_MAC_H_
#define _PCH_GBE_MAC_H_

/*!
 * @ingroup HAL internal function
 * @fn      void pch_gbe_mac_reset_hw(struct pch_gbe_hw *hw)
 * @brief   Reset hardware
 */
void pch_gbe_mac_reset_hw(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL internal function
 * @fn      void pch_gbe_mac_init_rx_addrs(struct pch_gbe_hw *hw,
 *                                         u16 mar_count)
 * @brief   Initialize receive address's
 */
void pch_gbe_mac_init_rx_addrs(struct pch_gbe_hw *hw, u16 mar_count);

/*!
 * @ingroup HAL internal function
 * @fn      void pch_gbe_mac_mar_set(struct pch_gbe_hw *hw, u8 *addr, u32 index)
 * @brief   Set MAC address register
 */
void pch_gbe_mac_mar_set(struct pch_gbe_hw *hw, u8 *addr, u32 index);

/*!
 * @ingroup HAL internal function
 * @fn      void pch_gbe_mac_mc_addr_list_update(struct pch_gbe_hw *hw,
 *                                u8 *mc_addr_list,   u32 mc_addr_count,
 *                                u32 mar_used_count, u32 mar_total_num)
 * @brief   Update Multicast addresses
 */
void pch_gbe_mac_mc_addr_list_update(struct pch_gbe_hw *hw,
	u8 *mc_addr_list, u32 mc_addr_count,
	u32 mar_used_count, u32 mar_count);

/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_mac_setup_link(struct pch_gbe_hw *hw)
 * @brief   Setup flow control and link settings
 */
s32 pch_gbe_mac_setup_link(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_mac_force_mac_fc(struct pch_gbe_hw *hw)
 * @brief   Force the MAC's flow control settings
 */
s32 pch_gbe_mac_force_mac_fc(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_mac_config_fc_after_link_up(struct pch_gbe_hw *hw)
 * @brief   Configures flow control after link
 */
s32 pch_gbe_mac_config_fc_after_link_up(struct pch_gbe_hw *hw);

/*!
 * @ingroup HAL internal function
 * @fn      void pch_gbe_mac_set_wol_event(struct pch_gbe_hw *hw, u32 wu_evt)
 * @brief   Set wake-on-lan event
 */
void pch_gbe_mac_set_wol_event(struct pch_gbe_hw *hw, u32 wu_evt);

/*!
 * @ingroup HAL internal function
 * @fn      u16 pch_gbe_mac_ctrl_miim(struct pch_gbe_hw *hw, u32 addr,
 *                                    u32 dir, u32 reg, u16 data)
 * @brief   Set wake-on-lan event
 */
u16 pch_gbe_mac_ctrl_miim(struct pch_gbe_hw *hw,
     u32 addr, u32 dir, u32 reg, u16 data);

/*!
 * @ingroup HAL internal function
 * @fn      void pch_gbe_mac_set_pause_packet(struct pch_gbe_hw *hw)
 * @brief   Set pause packet
 */
void pch_gbe_mac_set_pause_packet(struct pch_gbe_hw *hw);

#ifndef CONFIG_PCH_PHUB
/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_mac_read_mac_addr(struct pch_gbe_hw *hw)
 * @brief   Read MAC address
 */
s32 pch_gbe_mac_read_mac_addr(struct pch_gbe_hw *hw);
#endif /* CONFIG_PCH_PHUB */

#endif /* _PCH_GBE_MAC_H_ */
