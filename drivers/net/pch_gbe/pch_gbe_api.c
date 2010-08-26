/*!
 * @file pch_gbe_api.c
 * @brief Linux PCH Gigabit Ethernet Driver HAL API source file
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
#include "pch_gbe_api.h"

/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_set_mac_type(struct pch_gbe_hw *hw)
 * @brief   Sets MAC type
 * @param   hw [INOUT] Pointer to the HW structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:  Failed
 * @remarks This function sets the mac type of the adapter based on the
 *          device ID stored in the hw structure.
 *          MUST BE FIRST FUNCTION CALLED (explicitly or through
 *          pch_gbe_hal_setup_init_funcs()).
 */
s32 pch_gbe_hal_set_mac_type(struct pch_gbe_hw *hw)
{
 struct pch_gbe_mac_info *mac = &hw->mac;
 s32 ret_val = PCH_GBE_SUCCESS;

 PCH_DEBUG("pch_gbe_hal_set_mac_type\n");

 switch ((u16) hw->device_id) {
 case PCI_DEVICE_ID_INTEL_IOH1_GBE:
  mac->type = PCH_GBE_MAC_TYPE_PCH1;
  break;
 default:
  /* Should never have loaded on this device */
  mac->type = PCH_GBE_MAC_TYPE_UNDEFINED;
  ret_val = -PCH_GBE_ERR_MAC_INIT;
  break;
 }
 PCH_DEBUG("mac->type:0x%x  ret_val:0x%x\n", mac->type, ret_val);
 return ret_val;
}

/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_setup_init_funcs(struct pch_gbe_hw *hw)
 * @brief   Initializes function pointers
 * @param   hw [INOUT] pointer to the HW structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:  Failed
 * @remarks This function must be called by a driver in order to use the rest
 *          of the 'shared' code files. Called by drivers only.
 */
s32 pch_gbe_hal_setup_init_funcs(struct pch_gbe_hw *hw)
{
 s32 ret_val;

 PCH_DEBUG("pch_gbe_hal_setup_init_funcs\n");

 /* Can't do much good without knowing the MAC type.
  */
 ret_val = pch_gbe_hal_set_mac_type(hw);
 if (ret_val) {
  PCH_LOG(KERN_ERR,
   "ERROR: MAC type could not be set properly.\n");
  goto out;
 }

 if (!hw->hw_addr) {
  PCH_LOG(KERN_ERR, "ERROR: Registers not mapped\n");
  ret_val = -PCH_GBE_ERR_CONFIG;
  goto out;
 }

 /* Set up the init function pointers. These are functions within the
  * adapter family file that sets up function pointers for the rest of
  * the functions in that family.
  */
 switch (hw->mac.type) {
 case PCH_GBE_MAC_TYPE_PCH1:
 case PCH_GBE_MAC_TYPE_PCH2:
  pch_gbe_plat_init_function_pointers(hw);
  break;
 default:
  PCH_LOG(KERN_ERR, "Hardware not supported\n");
  ret_val = -PCH_GBE_ERR_CONFIG;
  break;
 }
out:
 PCH_DEBUG("ret_val:0x%x\n", ret_val);
 return ret_val;
}

/*!
 * @ingroup HAL API Layer
 * @fn      void pch_gbe_hal_get_bus_info(struct pch_gbe_hw *hw)
 * @brief   Obtain bus information for adapter
 * @param   hw [INOUT] pointer to the HW structure
 * @return  None
 * @remarks This will obtain information about the HW bus for which the
 *          adaper is attached and stores it in the hw structure. This is a
 *          function pointer entry point called by drivers.
 */
void pch_gbe_hal_get_bus_info(struct pch_gbe_hw *hw)
{
 PCH_DEBUG("pch_gbe_hal_get_bus_info\n");

 if (hw->func.get_bus_info != NULL)
  hw->func.get_bus_info(hw);
 else
  PCH_LOG(KERN_ERR, "Error: configuration\n");
}

/*!
 * @ingroup HAL API Layer
 * @fn      void pch_gbe_hal_mc_addr_list_update(struct pch_gbe_hw *hw,
 *               u8 *mc_addr_list, u32 mc_addr_count,
 *               u32 mar_used_count, u32 mar_count)
 * @brief   Update Multicast addresses
 * @param   hw        [INOUT] Pointer to the HW structure
 * @param   mc_addr_list   [IN]Array of multicast addresses to program
 * @param   mc_addr_count  [IN]Number of multicast addresses to program
 * @param   mar_used_count [IN]The first MAC Address register free to program
 * @param   mar_count [IN]Total number of supported MAC Address Registers
 * @return  None
 * @remarks
 *  Updates the MAC Address Registers and Multicast Table Array.
 *  The caller must have a packed mc_addr_list of multicast addresses.
 *  The parameter mar_count will usually be hw->mac.mar_entry_count
 *  unless there are workarounds that change this.  Currently no func pointer
 *  exists and all implementations are handled in the generic version of this
 *  function.
 */
void
pch_gbe_hal_mc_addr_list_update(struct pch_gbe_hw *hw,
    u8 *mc_addr_list,
    u32 mc_addr_count,
    u32 mar_used_count, u32 mar_count)
{
 PCH_DEBUG("pch_gbe_hal_mc_addr_list_update\n");

 if (hw->func.mc_addr_list_update != NULL) {
  hw->func.mc_addr_list_update(hw,
	mc_addr_list,
	mc_addr_count,
	mar_used_count, mar_count);
 }
}

/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_force_mac_fc(struct pch_gbe_hw *hw)
 * @brief   Force MAC flow control
 * @param   hw [INOUT] Pointer to the HW structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:  Failed
 * @remarks
 *  Force the MAC's flow control settings. Currently no func pointer exists
 *  and all implementations are handled in the generic version of this
 *  function.
 */
s32 pch_gbe_hal_force_mac_fc(struct pch_gbe_hw *hw)
{
 PCH_DEBUG("pch_gbe_hal_force_mac_fc\n");

 return pch_gbe_mac_force_mac_fc(hw);
}

/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_reset_hw(struct pch_gbe_hw *hw)
 * @brief   Reset hardware
 * @param   hw [INOUT] Pointer to the HW structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:  Failed
 * @remarks
 *  This resets the hardware into a known state. This is a function pointer
 *  entry point called by drivers.
 */
s32 pch_gbe_hal_reset_hw(struct pch_gbe_hw *hw)
{
 PCH_DEBUG("pch_gbe_hal_reset_hw\n");

 if (hw->func.reset_hw != NULL) {
  hw->func.reset_hw(hw);
  return PCH_GBE_SUCCESS;
 } else {
  PCH_LOG(KERN_ERR, "Error: configuration\n");
  return -PCH_GBE_ERR_CONFIG;
 }
}

/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_init_hw(struct pch_gbe_hw *hw)
 * @brief   Initialize hardware
 * @param   hw [INOUT] Pointer to the HW structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:  Failed
 * @remarks
 *  This inits the hardware readying it for operation. This is a function
 *  pointer entry point called by drivers.
 */
s32 pch_gbe_hal_init_hw(struct pch_gbe_hw *hw)
{
 PCH_DEBUG("pch_gbe_hal_init_hw\n");

 if (hw->func.init_hw != NULL) {
  return hw->func.init_hw(hw);
 } else {
  PCH_LOG(KERN_ERR, "Error: configuration\n");
  return -PCH_GBE_ERR_CONFIG;
 }
}

/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_setup_link(struct pch_gbe_hw *hw)
 * @brief   Configures link and flow control
 * @param   hw [INOUT] Pointer to the HW structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:  Failed
 * @remarks
 *  This configures link and flow control settings for the adapter. This
 *  is a function pointer entry point called by drivers. While modules can
 *  also call this, they probably call their own version of this function.
 */
s32 pch_gbe_hal_setup_link(struct pch_gbe_hw *hw)
{
 PCH_DEBUG("pch_gbe_hal_setup_link\n");

 if (hw->func.setup_link != NULL) {
  return hw->func.setup_link(hw);
 } else {
  PCH_LOG(KERN_ERR, "Error: configuration\n");
  return -PCH_GBE_ERR_CONFIG;
 }
}

/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_setup_led(struct pch_gbe_hw *hw)
 * @brief   Configures SW controllable LED
 * @param   hw [INOUT] Pointer to the HW structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:  Failed
 * @remarks
 *  This prepares the SW controllable LED for use and saves the current state
 *  of the LED so it can be later restored. This is a function pointer entry
 *  point called by drivers.
 */
s32 pch_gbe_hal_setup_led(struct pch_gbe_hw *hw)
{
 PCH_DEBUG("pch_gbe_hal_setup_led\n");

 if (hw->func.setup_led != NULL)
  return hw->func.setup_led(hw);
 else
  return PCH_GBE_SUCCESS;
}

/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_cleanup_led(struct pch_gbe_hw *hw)
 * @brief   Restores SW controllable LED
 * @param   hw [INOUT] Pointer to the HW structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:  Failed
 * @remarks
 *  This restores the SW controllable LED to the value saved off by
 *  pch_gbe_hal_setup_led.
 *  This is a function pointer entry point called by drivers.
 */
s32 pch_gbe_hal_cleanup_led(struct pch_gbe_hw *hw)
{
 PCH_DEBUG("pch_gbe_hal_cleanup_led\n");

 if (hw->func.cleanup_led != NULL)
  return hw->func.cleanup_led(hw);
 else
  return PCH_GBE_SUCCESS;
}

/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_led_on(struct pch_gbe_hw *hw)
 * @brief   Turn on SW controllable LED
 * @param   hw [INOUT] Pointer to the HW structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:  Failed
 * @remarks
 *  Turns the SW defined LED on. This is a function pointer entry point
 *  called by drivers.
 */
s32 pch_gbe_hal_led_on(struct pch_gbe_hw *hw)
{
 PCH_DEBUG("pch_gbe_hal_led_on\n");

 if (hw->func.led_on != NULL)
  return hw->func.led_on(hw);
 else
  return PCH_GBE_SUCCESS;
}

/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_led_off(struct pch_gbe_hw *hw)
 * @brief   Turn off SW controllable LED
 * @param   hw [INOUT] Pointer to the HW structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:  Failed
 * @remarks
 *  Turns the SW defined LED off. This is a function pointer entry point
 *  called by drivers.
 */
s32 pch_gbe_hal_led_off(struct pch_gbe_hw *hw)
{
 PCH_DEBUG("pch_gbe_hal_led_off\n");

 if (hw->func.led_off != NULL)
  return hw->func.led_off(hw);
 else
  return PCH_GBE_SUCCESS;
}

/*!
 * @ingroup HAL API Layer
 * @fn      void pch_gbe_hal_mar_set(struct pch_gbe_hw *hw, u8 *addr, u32 index)
 * @brief   Sets a MAC address register
 * @param   hw    [INOUT] Pointer to the HW structure
 * @param   addr  [IN] Address to set the RAR to
 * @param   index [IN] The RAR to set
 * @return  None
 * @remarks
 *  Sets a MAC Address Register (RAR) to the specified address.
 *  Currently no func pointer exists and all implementations are
 *  handled in the generic version of this function.
 */
void pch_gbe_hal_mar_set(struct pch_gbe_hw *hw, u8 *addr, u32 index)
{
 PCH_DEBUG("pch_gbe_hal_mar_set\n");

 pch_gbe_mac_mar_set(hw, addr, index);
}

/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_read_phy_reg(struct pch_gbe_hw *hw,
 *                                       u32 offset, u16 *data)
 * @brief   Reads PHY register
 * @param   hw     [INOUT] Pointer to the HW structure
 * @param   offset [IN] The register to read
 * @param   data   [IN] The buffer to store the 16-bit read.
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:  Failed
 * @remarks
 *  Reads the PHY register and returns the value in data.
 *  This is a function pointer entry point called by drivers.
 */
s32 pch_gbe_hal_read_phy_reg(struct pch_gbe_hw *hw, u32 offset, u16 *data)
{
 PCH_DEBUG("pch_gbe_hal_read_phy_reg\n");

 if (hw->func.read_phy_reg != NULL)
  return hw->func.read_phy_reg(hw, offset, data);
 else
  return PCH_GBE_SUCCESS;
}

/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_write_phy_reg(struct pch_gbe_hw *hw,
 *                                        u32 offset, u16 data)
 * @brief   Writes PHY register
 * @param   hw     [INOUT] Pointer to the HW structure
 * @param   offset [IN] The register to write
 * @param   data   [IN] The value to write.
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:  Failed
 * @remarks
 *  Writes the PHY register at offset with the value in data.
 *  This is a function pointer entry point called by drivers.
 */
s32 pch_gbe_hal_write_phy_reg(struct pch_gbe_hw *hw, u32 offset, u16 data)
{
 PCH_DEBUG("pch_gbe_hal_write_phy_reg\n");

 if (hw->func.write_phy_reg != NULL)
  return hw->func.write_phy_reg(hw, offset, data);
 else
  return PCH_GBE_SUCCESS;
}

/*!
 * @ingroup HAL API Layer
 * @fn      void pch_gbe_hal_phy_hw_reset(struct pch_gbe_hw *hw)
 * @brief   Hard PHY reset
 * @param   hw [INOUT] Pointer to the HW structure
 * @return  None
 * @remarks
 *  Performs a hard PHY reset. This is a function pointer entry point called
 *  by drivers.
 */
void pch_gbe_hal_phy_hw_reset(struct pch_gbe_hw *hw)
{
 PCH_DEBUG("pch_gbe_hal_phy_hw_reset\n");

 if (hw->func.reset_phy != NULL)
  hw->func.reset_phy(hw);
 else
  PCH_LOG(KERN_ERR, "Error: configuration\n");
}

/*!
 * @ingroup HAL API Layer
 * @fn      void pch_gbe_hal_phy_sw_reset(struct pch_gbe_hw *hw)
 * @brief   Soft PHY reset
 * @param   hw [INOUT] Pointer to the HW structure
 * @return  None
 * @remarks
 *  Performs a soft PHY reset on those that apply. This is a function pointer
 *  entry point called by drivers.
 */
void pch_gbe_hal_phy_sw_reset(struct pch_gbe_hw *hw)
{
 PCH_DEBUG("pch_gbe_hal_phy_sw_reset\n");

 if (hw->func.sw_reset_phy != NULL)
  hw->func.sw_reset_phy(hw);
 else
  PCH_LOG(KERN_ERR, "Error: configuration\n");
}

/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_read_mac_addr(struct pch_gbe_hw *hw)
 * @brief   Reads MAC address
 * @param   hw [INOUT] Pointer to the HW structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:  Failed
 * @remarks
 *  Reads the MAC address out of the adapter and stores it in the HW structure.
 *  Currently no func pointer exists and all implementations are handled in the
 *  generic version of this function.
 */
s32 pch_gbe_hal_read_mac_addr(struct pch_gbe_hw *hw)
{
 PCH_DEBUG("pch_gbe_hal_read_mac_addr\n");

 if (hw->func.read_mac_addr != NULL) {
  return hw->func.read_mac_addr(hw);
 } else {
  PCH_LOG(KERN_ERR, "Error: configuration\n");
  return -PCH_GBE_ERR_CONFIG;
 }
}

#ifdef CONFIG_PCH_PHUB
/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_validate_nvm_checksum(struct pch_gbe_hw *hw)
 * @brief   Verifies NVM (EEPROM) checksum
 * @param   hw [INOUT] Pointer to the HW structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:  Failed
 * @remarks
 *  Validates the NVM checksum is correct. This is a function pointer entry
 *  point called by drivers.
 */
s32 pch_gbe_hal_validate_nvm_checksum(struct pch_gbe_hw *hw)
{
 PCH_DEBUG("pch_gbe_hal_validate_nvm_checksum\n");

 if (hw->func.validate_nvm != NULL) {
  return hw->func.validate_nvm(hw);
 } else {
  PCH_LOG(KERN_ERR, "Error: configuration\n");
  return -PCH_GBE_ERR_CONFIG;
 }
}

/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_read_nvm(struct pch_gbe_hw *hw,
 *                                   u32 offset, u8 *data)
 * @brief   Reads NVM (EEPROM)
 * @param   hw     [INOUT] Pointer to the HW structure
 * @param   offset [IN] The word offset to read
 * @param   data   [IN] Pointer to the properly sized buffer for the data.
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:  Failed
 * @remarks
 *  Reads 16-bit chunks of data from the NVM (EEPROM). This is a function
 *  pointer entry point called by drivers.
 */
s32 pch_gbe_hal_read_nvm(struct pch_gbe_hw *hw, u32 offset, u8 *data)
{
 PCH_DEBUG("pch_gbe_hal_read_nvm\n");

 if (hw->func.read_nvm != NULL) {
  return hw->func.read_nvm(hw, offset, data);
 } else {
  PCH_LOG(KERN_ERR, "Error: configuration\n");
  return -PCH_GBE_ERR_CONFIG;
 }
}

/*!
 * @ingroup HAL API Layer
 * @fn      s32 pch_gbe_hal_write_nvm(struct pch_gbe_hw *hw,
 *                                    u32 offset, u8 *data)
 * @brief   Writes to NVM (EEPROM)
 * @param   hw     [INOUT] Pointer to the HW structure
 * @param   offset [IN] The word offset to read
 * @param   data   [IN] Pointer to the properly sized buffer for the data.
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:  Failed
 * @remarks
 *  Writes 16-bit chunks of data to the NVM (EEPROM). This is a function
 *  pointer entry point called by drivers.
 */
s32 pch_gbe_hal_write_nvm(struct pch_gbe_hw *hw, u32 offset, u8 *data)
{
 PCH_DEBUG("pch_gbe_hal_write_nvm\n");

 if (hw->func.write_nvm != NULL)
  return hw->func.write_nvm(hw, offset, data);
 else
  return PCH_GBE_SUCCESS;
}
#endif /* CONFIG_PCH_PHUB */

/*!
 * @ingroup HAL API Layer
 * @fn      void pch_gbe_hal_set_wol_event(struct pch_gbe_hw *hw, u32 wu_evt)
 * @brief   Set wake-on-lan event
 * @param   hw     [INOUT] Pointer to the HW structure
 * @param   wu_evt [IN] Wake up event
 * @return  None
 */
void pch_gbe_hal_set_wol_event(struct pch_gbe_hw *hw, u32 wu_evt)
{
 PCH_DEBUG("pch_gbe_hal_set_wol_event\n");

 pch_gbe_mac_set_wol_event(hw, wu_evt);
}

/*!
 * @ingroup HAL API Layer
 * @fn      void pch_gbe_hal_power_up_phy(struct pch_gbe_hw *hw)
 * @brief   Power up PHY
 * @param   hw [INOUT] Pointer to the HW structure
 * @return  None
 */
void pch_gbe_hal_power_up_phy(struct pch_gbe_hw *hw)
{
 PCH_DEBUG("pch_gbe_hal_power_up_phy\n");

 if (hw->func.power_up_phy != NULL)
  hw->func.power_up_phy(hw);
}

/*!
 * @ingroup HAL API Layer
 * @fn      void pch_gbe_hal_power_down_phy(struct pch_gbe_hw *hw)
 * @brief   Power down PHY
 * @param   hw [INOUT] Pointer to the HW structure
 * @return  None
 */
void pch_gbe_hal_power_down_phy(struct pch_gbe_hw *hw)
{
 PCH_DEBUG("pch_gbe_hal_power_down_phy\n");

 if (hw->func.power_down_phy != NULL)
  hw->func.power_down_phy(hw);
}

/*!
 * @ingroup HAL API Layer
 * @fn      u16 pch_gbe_hal_ctrl_miim(struct pch_gbe_hw *hw,
 *                                    u32 addr, u32 dir, u32 reg, u16 data)
 * @brief   Control MII Management IF
 * @param   hw   [INOUT] Pointer to the HW structure
 * @param   addr [IN] Address of PHY
 * @param   dir  [IN] Operetion. (Write or Read)
 * @param   reg  [IN] Access register of PHY
 * @param   data [IN] Write data
 * @return  None
 */
u16
pch_gbe_hal_ctrl_miim(struct pch_gbe_hw *hw, u32 addr, u32 dir, u32 reg,
	u16 data)
{
#ifdef DEBUG_TEST
 PCH_DEBUG("pch_gbe_hal_ctrl_miim\n");
#endif
 if (hw->func.ctrl_miim != NULL) {
  return hw->func.ctrl_miim(hw, addr, dir, reg, data);
 } else {
  PCH_LOG(KERN_ERR, "Error: configuration\n");
  return PCH_GBE_SUCCESS;
 }
}

/*!
 * @ingroup HAL API Layer
 * @fn      void pch_gbe_hal_set_pause_packet(struct pch_gbe_hw *hw)
 * @brief   Set pause packet
 * @param   hw [INOUT] Pointer to the HW structure
 * @return  None
 */
void pch_gbe_hal_set_pause_packet(struct pch_gbe_hw *hw)
{
 PCH_DEBUG("pch_gbe_hal_set_pause_packet\n");

 if (hw->func.pause_packet != NULL)
  hw->func.pause_packet(hw);
 else
  PCH_LOG(KERN_ERR, "Error: configuration\n");
}
