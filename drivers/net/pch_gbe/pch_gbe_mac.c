/*!
 * @file pch_gbe_mac.c
 * @brief Linux PCH Gigabit Ethernet Driver HAL internal function (MAC) source file
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
#include <linux/ethtool.h>
#include "pch_debug.h"
#include "pch_gbe_osdep.h"
#include "pch_gbe_regs.h"
#include "pch_gbe_defines.h"
#include "pch_gbe_hw.h"
#include "pch_gbe_mac.h"
#include "pch_gbe_api.h"

/* Pause packet value */
#define PCH_GBE_PAUSE_PKT1_VALUE    0x00C28001
#define PCH_GBE_PAUSE_PKT2_VALUE    0x00000100
#define PCH_GBE_PAUSE_PKT4_VALUE    0x01000888
#define PCH_GBE_PAUSE_PKT5_VALUE    0x0000FFFF

/*!
 * @ingroup HAL internal function
 * @fn      void pch_gbe_mac_reset_hw(struct pch_gbe_hw *hw)
 * @brief   Reset hardware
 * @param   hw [INOUT] Pointer to the HW structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:   Failed
 * @remarks
 *  This resets the hardware into a known state (Reset only MAC).
 *  This is a function pointer entry point called by the api module.
 */
void pch_gbe_mac_reset_hw(struct pch_gbe_hw *hw)
{
 u32 tmp = 0;

 PCH_DEBUG("pch_gbe_mac_reset_hw\n");

#ifndef CONFIG_PCH_PHUB
 /* Read the MAC address. and store to the private data */
 pch_gbe_mac_read_mac_addr(hw);
#endif

 PCH_GBE_WRITE_REG(hw, RESET, PCH_GBE_ALL_RST);
#ifdef PCH_GBE_MAC_IFOP_RGMII
 PCH_GBE_WRITE_REG(hw, MODE, PCH_GBE_MODE_GMII_ETHER);
#endif
 while ((PCH_GBE_READ_REG(hw, RESET)) != 0) {
  udelay(1);
  tmp++;
  if (tmp == 5) {
   PCH_LOG(KERN_ERR, "MAC HW RESET\n");
   break;
  }
 }
#ifndef CONFIG_PCH_PHUB
 /* Setup the receive address */
 pch_gbe_mac_mar_set(hw, hw->mac.addr, 0);
#endif
 return;
}

/*!
 * @ingroup HAL internal function
 * @fn      void pch_gbe_mac_init_rx_addrs(struct pch_gbe_hw *hw,
 *                                         u16 mar_count)
 * @brief   Initialize receive address's
 * @param   hw        [INOUT] Pointer to the HW structure
 * @param   mar_count [IN] Receive address registers
 * @return  None
 * @remarks
 *  Setups the receive address registers by setting the base receive address
 *  register to the devices MAC address and clearing all the other receive
 *  address registers to 0.
 *  This is a function pointer entry point called by the api module.
 */
void pch_gbe_mac_init_rx_addrs(struct pch_gbe_hw *hw, u16 mar_count)
{
 u32 i;

 PCH_DEBUG("pch_gbe_mac_init_rx_addrs\n");
 PCH_DEBUG("Programming MAC Address into MAC_ADDR[0]\n");
 PCH_DEBUG("Clearing MAC_ADDR[1-%u]\n", mar_count - 1);

 /* Setup the receive address */
 pch_gbe_hal_mar_set(hw, hw->mac.addr, 0);

 /* Zero out the other (mar_entry_count - 1) receive addresses */
 for (i = 1; i < mar_count; i++) {
  PCH_GBE_WRITE_REG_ARRAY(hw, MAC_ADR, (i << 1), 0);
  PCH_GBE_WRITE_REG_ARRAY(hw, MAC_ADR, ((i << 1) + 1), 0);
 }
 PCH_GBE_WRITE_REG(hw, ADDR_MASK, 0xFFFE);
 /* wait busy */
 while ((PCH_GBE_READ_REG(hw, ADDR_MASK) & PCH_GBE_BUSY) != 0)
  ;
#ifdef DEBUG_TEST
 {
  unsigned char ti;
  PCH_DEBUG("ADDR_MASK reg(check index bit) : 0x%08x\n",
    PCH_GBE_READ_REG(hw, ADDR_MASK));
  for (ti = 0; ti < 16; ti++) {
   PCH_DEBUG("MAC_ADR%dAB reg : 0x%08x 0x%08x\n",
     (ti + 1),
     PCH_GBE_READ_REG(hw,
	MAC_ADR1A +
	(0x08 * ti)),
     PCH_GBE_READ_REG(hw,
	MAC_ADR1B +
	(0x08 * ti)));
  }
 }
#endif
}

/*!
 * @ingroup HAL internal function
 * @fn      void pch_gbe_mac_mar_set(struct pch_gbe_hw *hw, u8 *addr, u32 index)
 * @brief   Set MAC address register
 * @param   hw     [INOUT] Pointer to the HW structure
 * @param   addr   [IN] Pointer to the MAC address
 * @param   index  [IN] MAC address array register
 * @return  None
 */
void pch_gbe_mac_mar_set(struct pch_gbe_hw *hw, u8 * addr, u32 index)
{
 u32 mar_low, mar_high, adrmask;

 PCH_DEBUG("pch_gbe_mac_mar_set\n");
 PCH_DEBUG("index : 0x%x\n", index);

 /* HW expects these in little endian so we reverse the byte order
  * from network order (big endian) to little endian
  */
 mar_low = ((u32) addr[0] |
     ((u32) addr[1] << 8) |
     ((u32) addr[2] << 16) | ((u32) addr[3] << 24));

 mar_high = ((u32) addr[4] | ((u32) addr[5] << 8));
 /* Stop the MAC Address of index. */
 adrmask = PCH_GBE_READ_REG(hw, ADDR_MASK);
 PCH_GBE_WRITE_REG(hw, ADDR_MASK, (adrmask | (0x0001 << index)));

 PCH_DEBUG("ADDR_MASK reg : 0x%08x\n", adrmask);
 PCH_DEBUG("ADDR_MASK reg(check index bit) : 0x%08x\n",
   PCH_GBE_READ_REG(hw, ADDR_MASK));
 /* wait busy */
 while ((PCH_GBE_READ_REG(hw, ADDR_MASK) & PCH_GBE_BUSY) != 0)
  ;
 PCH_DEBUG("ADDR_MASK reg(check BUSY bit:1) : 0x%08x\n",
   PCH_GBE_READ_REG(hw, ADDR_MASK));

 /* Set the MAC address to the MAC address 1A/1B register */
 PCH_GBE_WRITE_REG_ARRAY(hw, MAC_ADR, (index << 1), mar_low);
 PCH_GBE_WRITE_REG_ARRAY(hw, MAC_ADR, ((index << 1) + 1), mar_high);
 /* Start the MAC address of index */
 PCH_GBE_WRITE_REG(hw, ADDR_MASK, (adrmask & ~(0x0001 << index)));
 PCH_DEBUG("ADDR_MASK reg(check index bit:0) : 0x%08x\n",
   PCH_GBE_READ_REG(hw, ADDR_MASK));
 /* wait busy */
 while ((PCH_GBE_READ_REG(hw, ADDR_MASK) & PCH_GBE_BUSY) != 0)
  ;
 PCH_DEBUG("ADDR_MASK reg(check BUSY bit) : 0x%08x\n",
   PCH_GBE_READ_REG(hw, ADDR_MASK));
 PCH_DEBUG("pch_gbe_mac_mar_set:End\n");
}

/*!
 * @ingroup HAL internal function
 * @fn      void pch_gbe_mac_mc_addr_list_update(struct pch_gbe_hw *hw,
 *                               u8 *mc_addr_list,   u32 mc_addr_count,
 *                               u32 mar_used_count, u32 mar_total_num)
 * @brief   Update Multicast addresses
 * @param   hw             [INOUT] Pointer to the HW structure
 * @param   mc_addr_list   [IN] Array of multicast addresses to program
 * @param   mc_addr_count  [IN] Number of multicast addresses to program
 * @param   mar_used_count [IN] The first MAC Address register free to program
 * @param   mar_total_num  [IN] Total number of supported MAC Address Registers
 * @return  None
 */
void
pch_gbe_mac_mc_addr_list_update(struct pch_gbe_hw *hw,
    u8 *mc_addr_list, u32 mc_addr_count,
    u32 mar_used_count, u32 mar_total_num)
{
 u32 i, adrmask;

 PCH_DEBUG("pch_gbe_mac_mc_addr_list_update\n");

#ifdef DEBUG_TEST
 {
  u32 ti, tj;
  PCH_DEBUG
      ("mc_addr_count = %d  mar_used_count = %d  "
       "mar_total_num = %d\n",
       mc_addr_count, mar_used_count, mar_total_num);
  for (ti = 0; ti < mc_addr_count; ti++) {
   tj = ti * PCH_GBE_ETH_ALEN;
   PCH_DEBUG
       ("mc_addr_list[%d] = 0x%02x %02x %02x "
	"%02x %02x %02x \n",
	ti, mc_addr_list[tj], mc_addr_list[tj + 1],
	mc_addr_list[tj + 2], mc_addr_list[tj + 3],
	mc_addr_list[tj + 4], mc_addr_list[tj + 5]);
  }
 }
#endif
 /* Load the first set of multicast addresses into the exact
  * filters (RAR).  If there are not enough to fill the RAR
  * array, clear the filters.
  */
 for (i = mar_used_count; i < mar_total_num; i++) {
  if (mc_addr_count != 0) {
   pch_gbe_mac_mar_set(hw, mc_addr_list, i);
   mc_addr_count--;
   mc_addr_list += PCH_GBE_ETH_ALEN;
  } else {
   /* Clear MAC address mask */
   adrmask = PCH_GBE_READ_REG(hw, ADDR_MASK);
   PCH_GBE_WRITE_REG(hw, ADDR_MASK,
       (adrmask | (0x0001 << i)));
   /* wait busy */
   while ((PCH_GBE_READ_REG(hw, ADDR_MASK) & PCH_GBE_BUSY)
	!= 0) {
    ;
   }
   /* Clear MAC address */
   PCH_GBE_WRITE_REG_ARRAY(hw, MAC_ADR, i << 1, 0);
   PCH_GBE_WRITE_REG_ARRAY(hw, MAC_ADR, (i << 1) + 1, 0);
  }
 }
#ifdef DEBUG_TEST
 {
  unsigned char ti;
  PCH_DEBUG("ADDR_MASK reg(check index bit) : 0x%08x\n",
    PCH_GBE_READ_REG(hw, ADDR_MASK));
  for (ti = 0; ti < 16; ti++) {
   PCH_DEBUG("MAC_ADR%dAB reg : 0x%08x 0x%08x\n",
     (ti + 1),
     PCH_GBE_READ_REG(hw,
	MAC_ADR1A +
	(0x08 * ti)),
     PCH_GBE_READ_REG(hw,
	MAC_ADR1B +
	(0x08 * ti)));
  }
 }
#endif
}

/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_mac_setup_link(struct pch_gbe_hw *hw)
 * @brief   Setup flow control and link settings
 * @param   hw [INOUT] Pointer to the HW structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:   Failed
 */
s32 pch_gbe_mac_setup_link(struct pch_gbe_hw *hw)
{
 struct pch_gbe_functions *func = &hw->func;
 s32 ret_val = PCH_GBE_SUCCESS;

 PCH_DEBUG("pch_gbe_mac_setup_link\n");

 /* Call the necessary media_type subroutine to configure the link. */
 ret_val = func->setup_physical_interface(hw);

 return ret_val;
}

/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_mac_force_mac_fc(struct pch_gbe_hw *hw)
 * @brief   Force the MAC's flow control settings
 * @param   hw [INOUT] Pointer to the HW structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:   Failed
 */
s32 pch_gbe_mac_force_mac_fc(struct pch_gbe_hw *hw)
{
 struct pch_gbe_mac_info *mac = &hw->mac;
 u32 rx_fctrl;

 PCH_DEBUG("pch_gbe_mac_force_mac_fc\n");
 PCH_DEBUG("mac->fc = %u\n", mac->fc);

 rx_fctrl = PCH_GBE_READ_REG(hw, RX_FCTRL);

 switch (mac->fc) {
 case pch_gbe_fc_none:
  rx_fctrl &= ~PCH_GBE_FL_CTRL_EN;
  mac->tx_fc_enable = FALSE;
  break;
 case pch_gbe_fc_rx_pause:
  rx_fctrl |= PCH_GBE_FL_CTRL_EN;
  mac->tx_fc_enable = FALSE;
  break;
 case pch_gbe_fc_tx_pause:
  rx_fctrl &= ~PCH_GBE_FL_CTRL_EN;
  mac->tx_fc_enable = TRUE;
  break;
 case pch_gbe_fc_full:
  rx_fctrl |= PCH_GBE_FL_CTRL_EN;
  mac->tx_fc_enable = TRUE;
  break;
 default:
  PCH_LOG(KERN_ERR, "Flow control param set incorrectly\n");
  return -PCH_GBE_ERR_CONFIG;
 }
 if (mac->link_duplex == DUPLEX_HALF)
  rx_fctrl &= ~PCH_GBE_FL_CTRL_EN;
 PCH_GBE_WRITE_REG(hw, RX_FCTRL, rx_fctrl);
 PCH_DEBUG("RX_FCTRL reg : 0x%08x  mac->tx_fc_enable : %d\n",
   PCH_GBE_READ_REG(hw, RX_FCTRL), mac->tx_fc_enable);
 return PCH_GBE_SUCCESS;
}

/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_mac_config_fc_after_link_up(struct pch_gbe_hw *hw)
 * @brief   Configures flow control after link
 * @param   hw [INOUT] Pointer to the HW structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 */
s32 pch_gbe_mac_config_fc_after_link_up(struct pch_gbe_hw *hw)
{
 PCH_DEBUG("pch_gbe_mac_config_fc_after_link_up\n");
 return PCH_GBE_SUCCESS;
}

/*!
 * @ingroup HAL internal function
 * @fn      void pch_gbe_mac_set_wol_event(struct pch_gbe_hw *hw, u32 wu_evt)
 * @brief   Set wake-on-lan event
 * @param   hw     [INOUT] Pointer to the HW structure
 * @param   wu_evt [IN] Wake up event
 * @return  None
 */
void pch_gbe_mac_set_wol_event(struct pch_gbe_hw *hw, u32 wu_evt)
{
 u32 addr_mask;

 PCH_DEBUG("pch_gbe_mac_set_wol_event\n");
 PCH_DEBUG("wu_evt : 0x%08x  ADDR_MASK reg : 0x%08x\n",
   wu_evt, PCH_GBE_READ_REG(hw, ADDR_MASK));

 if (wu_evt != 0) {
  /* Set Wake-On-Lan address mask */
  addr_mask = PCH_GBE_READ_REG(hw, ADDR_MASK);
  PCH_GBE_WRITE_REG(hw, WOL_ADDR_MASK, addr_mask);
  /* wait busy */
  while ((PCH_GBE_READ_REG(hw, WOL_ADDR_MASK) & PCH_GBE_WLA_BUSY)
	!= 0) {
   ;
  }
  PCH_GBE_WRITE_REG(hw, WOL_ST, 0);
  PCH_GBE_WRITE_REG(hw, WOL_CTRL,
      (wu_evt | PCH_GBE_WLC_WOL_MODE));
  PCH_GBE_WRITE_REG(hw, INT_EN, PCH_GBE_INT_ENABLE_MASK);
 } else {
  PCH_GBE_WRITE_REG(hw, WOL_CTRL, 0);
  PCH_GBE_WRITE_REG(hw, WOL_ST, 0);
 }

#ifdef DEBUG_TEST
 PCH_DEBUG
     ("WOL_ADDR_MASK reg : 0x%08x  WOL_CTRL reg : 0x%08x  "
      "WOL_ST reg : 0x%08x\n",
      PCH_GBE_READ_REG(hw, WOL_ADDR_MASK),
      PCH_GBE_READ_REG(hw, WOL_CTRL),
      PCH_GBE_READ_REG(hw, WOL_ST));
#endif
 return;
}

/*!
 * @ingroup HAL internal function
 * @fn      u16 pch_gbe_mac_ctrl_miim(struct pch_gbe_hw *hw, u32 addr,
 *                                    u32 dir, u32 reg, u16 data)
 * @brief   Set wake-on-lan event
 * @param   hw   [INOUT] Pointer to the HW structure
 * @param   addr [IN] Address of PHY
 * @param   dir  [IN] Operetion. (Write or Read)
 * @param   reg  [IN] Access register of PHY
 * @param   data [IN] Write data.
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:   Failed
 */
u16
pch_gbe_mac_ctrl_miim(struct pch_gbe_hw *hw, u32 addr, u32 dir, u32 reg,
   u16 data)
{
 u32 data_out = 0;
 unsigned int i;
 unsigned long flags;

#ifdef DEBUG_TEST
 PCH_DEBUG("pch_gbe_mac_ctrl_miim\n");
#endif
 spin_lock_irqsave(&hw->miim_lock, flags);

 for (i = 100; i; --i) {
  if ((PCH_GBE_READ_REG(hw, MIIM) & PCH_GBE_MIIM_OPER_READY) != 0)
   break;
  udelay(20);
 }
 if (i == 0) {
  PCH_LOG(KERN_ERR, "pch-gbe.miim won't go Ready\n");
  spin_unlock_irqrestore(&hw->miim_lock, flags);
  return PCH_GBE_SUCCESS; /* No way to indicate timeout error */
 }
 PCH_GBE_WRITE_REG(hw, MIIM, ((reg << PCH_GBE_MIIM_REG_ADDR_SHIFT) |
	(addr << PCH_GBE_MIIM_PHY_ADDR_SHIFT) |
	dir | data));
 for (i = 0; i < 100; i++) {
  udelay(20);
  data_out = PCH_GBE_READ_REG(hw, MIIM);
  if ((data_out & PCH_GBE_MIIM_OPER_READY) != 0)
   break;
 }
 spin_unlock_irqrestore(&hw->miim_lock, flags);

 PCH_DEBUG("%s:addr=%d, reg=%d, data_in=0x%04X, data_out=0x%04X\n",
   dir == PCH_GBE_MIIM_OPER_READ ? "READ" : "WRITE",
   addr, reg, data, data_out);
 return (u16) data_out;
}

/*!
 * @ingroup HAL internal function
 * @fn      void pch_gbe_mac_set_pause_packet(struct pch_gbe_hw *hw)
 * @brief   Set pause packet
 * @param   hw [INOUT] Pointer to the HW structure
 * @return  None
 */
void pch_gbe_mac_set_pause_packet(struct pch_gbe_hw *hw)
{
 unsigned long tmp2, tmp3;

 PCH_DEBUG("pch_gbe_mac_set_pause_packet\n");

 /* Set Pause packet */
 tmp2 = hw->mac.addr[1];
 tmp2 = (tmp2 << 8) | hw->mac.addr[0];
 tmp2 = PCH_GBE_PAUSE_PKT2_VALUE | (tmp2 << 16);

 tmp3 = hw->mac.addr[5];
 tmp3 = (tmp3 << 8) | hw->mac.addr[4];
 tmp3 = (tmp3 << 8) | hw->mac.addr[3];
 tmp3 = (tmp3 << 8) | hw->mac.addr[2];

 PCH_GBE_WRITE_REG(hw, PAUSE_PKT1, PCH_GBE_PAUSE_PKT1_VALUE);
 PCH_GBE_WRITE_REG(hw, PAUSE_PKT2, tmp2);
 PCH_GBE_WRITE_REG(hw, PAUSE_PKT3, tmp3);
 PCH_GBE_WRITE_REG(hw, PAUSE_PKT4, PCH_GBE_PAUSE_PKT4_VALUE);
 PCH_GBE_WRITE_REG(hw, PAUSE_PKT5, PCH_GBE_PAUSE_PKT5_VALUE);

 /* Transmit Pause Packet */
 PCH_GBE_WRITE_REG(hw, PAUSE_REQ, PCH_GBE_PS_PKT_RQ);

 PCH_DEBUG
     ("PAUSE_PKT1-5 reg : 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n",
      PCH_GBE_READ_REG(hw, PAUSE_PKT1), PCH_GBE_READ_REG(hw, PAUSE_PKT2),
      PCH_GBE_READ_REG(hw, PAUSE_PKT3), PCH_GBE_READ_REG(hw, PAUSE_PKT4),
      PCH_GBE_READ_REG(hw, PAUSE_PKT5));

 return;
}

#ifndef CONFIG_PCH_PHUB
/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_mac_read_mac_addr(struct pch_gbe_hw *hw)
 * @brief   Read MAC address
 * @param   hw [INOUT] Pointer to the HW structure
 * @return  PCH_GBE_SUCCESS
 */
s32 pch_gbe_mac_read_mac_addr(struct pch_gbe_hw *hw)
{
 u32  adr1a, adr1b;

 PCH_DEBUG("pch_gbe_mac_read_mac_addr\n");

 adr1a = PCH_GBE_READ_REG(hw, MAC_ADR1A);
 adr1b = PCH_GBE_READ_REG(hw, MAC_ADR1B);

 hw->mac.addr[0] = (u8)(adr1a & 0xFF);
 hw->mac.addr[1] = (u8)((adr1a >> 8) & 0xFF);
 hw->mac.addr[2] = (u8)((adr1a >> 16) & 0xFF);
 hw->mac.addr[3] = (u8)((adr1a >> 24) & 0xFF);
 hw->mac.addr[4] = (u8)(adr1b & 0xFF);
 hw->mac.addr[5] = (u8)((adr1b >> 8) & 0xFF);

 PCH_DEBUG("hw->mac.addr : 0x%02x %02x %02x %02x %02x %02x\n",
  hw->mac.addr[0], hw->mac.addr[1], hw->mac.addr[2],
  hw->mac.addr[3], hw->mac.addr[4], hw->mac.addr[5]);
 return PCH_GBE_SUCCESS;
}
#endif /* CONFIG_PCH_PHUB */
