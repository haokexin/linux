/*!
 * @file pch_gbe_plat.c
 * @brief Linux PCH Gigabit Ethernet Driver HAL internal function (platform) source file
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
#include "pch_gbe_mac.h"
#include "pch_gbe_nvm.h"
#include "pch_gbe_phy.h"
#include "pch_gbe_api.h"


static void pch_gbe_plat_get_bus_info(struct pch_gbe_hw *hw);
static s32 pch_gbe_plat_init_hw(struct pch_gbe_hw *hw);


/*!
 * @ingroup HAL internal functions
 * @fn      void pch_gbe_plat_init_function_pointers(struct pch_gbe_hw *hw)
 * @brief   Init func ptrs.
 * @param   hw  [OUT] Pointer to the HW structure
 * @return  None
 * @remarks
 *  The only function explicitly called by the api module to initialize
 *  all function pointers and parameters.
 */
void
pch_gbe_plat_init_function_pointers(struct pch_gbe_hw *hw)
{
 struct pch_gbe_mac_info *mac = &hw->mac;
 struct pch_gbe_phy_info *phy = &hw->phy;
 struct pch_gbe_nvm_info *nvm = &hw->nvm;
 struct pch_gbe_functions *func = &hw->func;

 PCH_DEBUG("pch_gbe_plat_init_function_pointers\n");

 /* Set MAC address registers entry count */
 mac->mar_entry_count    = PCH_GBE_MAR_ENTRIES;
 /* Set PHY parameter */
 phy->reset_delay_us     = PCH_GBE_PHY_RESET_DELAY_US;
 /* Set NVM parameter */
 nvm->word_size          = PCH_GBE_NVM_WORD_SIZE;

 /* Set function pointers */
 func->get_bus_info             = pch_gbe_plat_get_bus_info;
 func->reset_hw                 = pch_gbe_mac_reset_hw;
 func->init_hw                  = pch_gbe_plat_init_hw;
 func->setup_link               = pch_gbe_mac_setup_link;
 func->setup_physical_interface = pch_gbe_phy_setup_link_fpga;
 func->mc_addr_list_update      = pch_gbe_mac_mc_addr_list_update;
 func->setup_led                = pch_gbe_phy_led_setup;
 func->cleanup_led              = pch_gbe_phy_led_cleanup;
 func->led_on                   = pch_gbe_phy_led_on;
 func->led_off                  = pch_gbe_phy_led_off;
 func->read_phy_reg             = pch_gbe_phy_read_reg_miic;
 func->write_phy_reg            = pch_gbe_phy_write_reg_miic;
 func->reset_phy                = pch_gbe_phy_hw_reset;
 func->sw_reset_phy             = pch_gbe_phy_sw_reset;
 func->power_up_phy             = pch_gbe_phy_power_up;
 func->power_down_phy           = pch_gbe_phy_power_down;
#ifdef CONFIG_PCH_PHUB
 func->read_nvm                 = pch_gbe_nvm_read_mem;
 func->write_nvm                = pch_gbe_nvm_write_mem;
 func->validate_nvm             = pch_gbe_nvm_validate_checksum;
 func->read_mac_addr            = pch_gbe_nvm_read_mac_addr;
#else
 func->read_mac_addr            = pch_gbe_mac_read_mac_addr;
#endif
 func->ctrl_miim                = pch_gbe_mac_ctrl_miim;
 func->pause_packet             = pch_gbe_mac_set_pause_packet;

#ifdef DEBUG_TEST
 PCH_DEBUG("[MAC]mar_entry_count:%d /[PHY] reset_delay_us:%d\n",
   mac->mar_entry_count, phy->reset_delay_us);
 PCH_DEBUG("[NVM] word_size:0x%08x\n", nvm->word_size);
#endif
}


/*!
 * @ingroup HAL internal functions
 * @fn      static void pch_gbe_plat_get_bus_info(struct pch_gbe_hw *hw)
 * @brief   Obtain bus information for adapter
 * @param   hw  [OUT] Pointer to the HW structure
 * @return  None
 * @remarks
 *  This will obtain information about the HW bus for which the
 *  adaper is attached and stores it in the hw structure.  This is a function
 *  pointer entry point called by the api module.
 */
static void
pch_gbe_plat_get_bus_info(struct pch_gbe_hw *hw)
{
 PCH_DEBUG("pch_gbe_plat_get_bus_info\n");

 hw->bus.type  = pch_gbe_bus_type_pci_express;
 hw->bus.speed = pch_gbe_bus_speed_2500;
 hw->bus.width = pch_gbe_bus_width_pcie_x1;

#ifdef DEBUG_TEST
 PCH_DEBUG("[BUS] type:0x%08x  speed:0x%08x  width:0x%08x\n",
   hw->bus.type, hw->bus.speed, hw->bus.width);
#endif
}

/*!
 * @ingroup HAL internal functions
 * @fn      static s32 pch_gbe_plat_init_hw(struct pch_gbe_hw *hw)
 * @brief   Initialize hardware
 * @param   hw  [INOUT] Pointer to the HW structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:  Failed
 * @remarks
 *  This inits the hardware readying it for operation.  This is a
 *  function pointer entry point called by the api module.
 */
static s32
pch_gbe_plat_init_hw(struct pch_gbe_hw *hw)
{
 struct pch_gbe_mac_info *mac = &hw->mac;
 s32 ret_val;

 PCH_DEBUG("pch_gbe_plat_init_hw\n");

 /* Setup the receive address. */
 pch_gbe_mac_init_rx_addrs(hw, mac->mar_entry_count);

 ret_val = pch_gbe_phy_get_id(hw);
 if (ret_val) {
  PCH_LOG(KERN_ERR, "pch_gbe_phy_get_id error\n");
  return ret_val;
 }
 pch_gbe_phy_init_setting(hw);
 /* Setup Mac interface option RGMII */
#ifdef PCH_GBE_MAC_IFOP_RGMII
 pch_gbe_phy_set_rgmii(hw);
#endif
 /* Setup link and flow control */
 ret_val = pch_gbe_hal_setup_link(hw);
#ifdef DEBUG_TEST
 if (ret_val)
  PCH_LOG(KERN_ERR, "pch_gbe_phy_get_id error\n");
#endif
 return ret_val;
}

