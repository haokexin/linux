/*!
 * @file pch_gbe_phy.c
 * @brief Linux PCH Gigabit Ethernet Driver HAL internal function (PHY) source file
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
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/ethtool.h>
#include <linux/mii.h>

#include "pch_debug.h"
#include "pch_gbe_osdep.h"
#include "pch_gbe_defines.h"
#include "pch_gbe_hw.h"
#include "pch_gbe_phy.h"
#include "pch_gbe_api.h"
#include "pch_gbe_regs.h"
#include "pch_gbe.h"


/*!
 * @ingroup  HAL internal function
 * @def      PHY_CONTROL_DEFAULT
 * @brief    Default value of CONTROL register of PHY
 */
#define PHY_CONTROL_DEFAULT           0x1140 /* Control Register */

/*!
 * @ingroup  HAL internal function
 * @def      PHY_AUTONEG_ADV_DEFAULT
 * @brief    Default value of AUTONEG_ADV register of PHY
 */
#define PHY_AUTONEG_ADV_DEFAULT       0x01e0 /* Autoneg Advertisement */

/*!
 * @ingroup  HAL internal function
 * @def      PHY_NEXT_PAGE_TX_DEFAULT
 * @brief    Default value of NEXT_PAGE_TX register of PHY
 */
#define PHY_NEXT_PAGE_TX_DEFAULT      0x2001 /* Next Page TX */

/*!
 * @ingroup  HAL internal function
 * @def      PHY_1000T_CTRL_DEFAULT
 * @brief    Default value of 1000T_CTRL register of PHY
 */
#define PHY_1000T_CTRL_DEFAULT        0x0300 /* 1000Base-T Control Register */

/*!
 * @ingroup  HAL internal function
 * @def      PHY_PHYSP_CONTROL_DEFAULT
 * @brief    Default value of PHYSP_CONTROL register of PHY
 */
#ifdef FPGA
#define PHY_PHYSP_CONTROL_DEFAULT     0x0078 /* PHY Specific Control Register */
#else
#define PHY_PHYSP_CONTROL_DEFAULT     0x01EE /* PHY Specific Control Register */
#endif

/*!
 * @ingroup  HAL internal function
 * @def      PHY_EXT_PHYSP_CONTROL_DEFAULT
 * @brief    Default value of EXT_PHYSP_CONTROL register of PHY
 */
#define PHY_EXT_PHYSP_CONTROL_DEFAULT 0x0c60

/*!
 * @ingroup  HAL internal function
 * @def      PHY_LED_CONTROL_DEFAULT
 * @brief    Default value of LED_CONTROL register of PHY
 */
#define PHY_LED_CONTROL_DEFAULT       0x4100


/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_phy_get_id(struct pch_gbe_hw *hw)
 * @brief   Retrieve the PHY ID and revision
 * @param   hw   [IN] Pointer to the HW structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:   Failed
 * @remarks
 *  Reads the PHY registers and stores the PHY ID and possibly the PHY
 *  revision in the hardware structure.
 */
s32
pch_gbe_phy_get_id(struct pch_gbe_hw *hw)
{
 struct pch_gbe_phy_info *phy = &hw->phy;
 s32 ret;
 u16 phy_id1;
 u16 phy_id2;

 PCH_DEBUG("pch_gbe_phy_get_id\n");

 ret = pch_gbe_hal_read_phy_reg(hw, PHY_ID1, &phy_id1);
 if (ret != 0)
  return ret;
 ret = pch_gbe_hal_read_phy_reg(hw, PHY_ID2, &phy_id2);
 if (ret != 0)
  return ret;
 /*
  * PHY_ID1: [bit15-0:ID(21-6)]
  * PHY_ID2: [bit15-10:ID(5-0)][bit9-4:Model][bit3-0:revision]
  */
 phy->id = (u32)phy_id1;
 phy->id = ((phy->id << 6) | ((phy_id2 & 0xFC00) >> 10));
 phy->revision = (u32) (phy_id2 & 0x000F);

#ifdef DEBUG_TEST
 PCH_DEBUG("phy->id : 0x%08x  phy->revision : 0x%08x\n",
   phy->id, phy->revision);
#endif
 return PCH_GBE_SUCCESS;
}

/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_phy_read_reg_miic(struct pch_gbe_hw *hw,
 *                                        u32 offset, u16 *data)
 * @brief   Read MII control register
 * @param   hw      [IN] Pointer to the HW structure
 * @param   offset  [IN] Register offset to be read
 * @param   data    [OUT] Pointer to the read data
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:   Failed
 * @remarks
 *  Reads the PHY registers and stores the PHY ID and possibly the PHY
 *  revision in the hardware structure.
 */
s32
pch_gbe_phy_read_reg_miic(struct pch_gbe_hw *hw, u32 offset, u16 *data)
{
 struct pch_gbe_phy_info *phy = &hw->phy;
 s32 ret_val = PCH_GBE_SUCCESS;

 if (offset > PHY_MAX_REG_ADDRESS) {
  PCH_DEBUG("PHY Address %d is out of range\n", offset);
  ret_val = -PCH_GBE_ERR_PARAM;
 } else {
  *data = pch_gbe_hal_ctrl_miim(hw, phy->addr,
     PCH_GBE_HAL_MIIM_READ, offset, (u16)0);
 }
 return ret_val;
}

/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_phy_write_reg_miic(struct pch_gbe_hw *hw,
 *                                         u32 offset, u16 data)
 * @brief   Write MII control register
 * @param   hw      [IN] Pointer to the HW structure
 * @param   offset  [IN] Register offset to be read
 * @param   data    [IN] data to write to register at offset
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:   Failed
 * @remarks
 *  Writes data to MDI control register in the PHY at offset.
 */
s32
pch_gbe_phy_write_reg_miic(struct pch_gbe_hw *hw, u32 offset, u16 data)
{
 struct pch_gbe_phy_info *phy = &hw->phy;
 s32 ret_val = PCH_GBE_SUCCESS;

 if (offset > PHY_MAX_REG_ADDRESS) {
  PCH_DEBUG("PHY Address %d is out of range\n", offset);
  ret_val = -PCH_GBE_ERR_PARAM;
 } else {
  pch_gbe_hal_ctrl_miim(hw, phy->addr,
     PCH_GBE_HAL_MIIM_WRITE, offset, data);
 }
 return ret_val;
}

/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_phy_setup_link_fpga(struct pch_gbe_hw *hw)
 * @brief   Configure link settings for FPGA
 * @param   hw      [IN] Pointer to the HW structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:   Failed
 * @remarks
 *  Calls the appropriate function to configure the link for auto-neg or forced
 *  speed and duplex.  Then we check for link, once link is established calls
 *  to configure collision distance and flow control are called.  If link is
 *  not established, we return -PCH_GBE_ERR_PHY (-2).
 */
s32
pch_gbe_phy_setup_link_fpga(struct pch_gbe_hw *hw)
{

 PCH_DEBUG("pch_gbe_phy_setup_link_fpga\n");
 return PCH_GBE_SUCCESS;
}

/*!
 * @ingroup HAL internal function
 * @fn      void pch_gbe_phy_sw_reset(struct pch_gbe_hw *hw)
 * @brief   PHY software reset
 * @param   hw      [IN] Pointer to the HW structure
 * @return  None
 * @remarks
 *  Does a software reset of the PHY by reading the PHY control register and
 *  setting/write the control register reset bit to the PHY.
 */
void
pch_gbe_phy_sw_reset(struct pch_gbe_hw *hw)
{
 u16 phy_ctrl;

 PCH_DEBUG("pch_gbe_phy_sw_reset\n");

 pch_gbe_hal_read_phy_reg(hw, PHY_CONTROL, &phy_ctrl);
 phy_ctrl |= MII_CR_RESET;
 pch_gbe_hal_write_phy_reg(hw, PHY_CONTROL, phy_ctrl);
 usec_delay(1);
}

/*!
 * @ingroup HAL internal function
 * @fn      void pch_gbe_phy_hw_reset(struct pch_gbe_hw *hw)
 * @brief   PHY hardware reset
 * @param   hw      [IN] Pointer to the HW structure
 * @return  None
 * @remarks
 *  Verify the reset block is not blocking us from resetting.  Acquire
 *  semaphore (if necessary) and read/set/write the device control reset
 *  bit in the PHY.  Wait the appropriate delay time for the device to
 *  reset and relase the semaphore (if necessary).
 */
void
pch_gbe_phy_hw_reset(struct pch_gbe_hw *hw)
{
#ifndef PHY_RESET_REG_INIT
 struct pch_gbe_phy_info *phy = &hw->phy;

 PCH_DEBUG("pch_gbe_phy_hw_reset\n");

 /* ISSUE: reset used GPIO driver */
 usec_delay(phy->reset_delay_us);
 /* ISSUE: release reset used GPIO driver */
#else
 PCH_DEBUG("pch_gbe_phy_hw_reset\n");

 pch_gbe_hal_write_phy_reg(hw, PHY_CONTROL, PHY_CONTROL_DEFAULT);
 pch_gbe_hal_write_phy_reg(hw, PHY_AUTONEG_ADV,
     PHY_AUTONEG_ADV_DEFAULT);
 pch_gbe_hal_write_phy_reg(hw, PHY_NEXT_PAGE_TX,
     PHY_NEXT_PAGE_TX_DEFAULT);
 pch_gbe_hal_write_phy_reg(hw, PHY_1000T_CTRL, PHY_1000T_CTRL_DEFAULT);
 pch_gbe_hal_write_phy_reg(hw, PHY_PHYSP_CONTROL,
     PHY_PHYSP_CONTROL_DEFAULT);
#ifdef FPGA
 pch_gbe_hal_write_phy_reg(hw, PHY_EXT_PHYSP_CONTROL,
     PHY_EXT_PHYSP_CONTROL_DEFAULT);
 pch_gbe_hal_write_phy_reg(hw, PHY_LED_CONTROL, PHY_LED_CONTROL_DEFAULT);
#endif

#endif

}

/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_phy_led_on(struct pch_gbe_hw *hw)
 * @brief   Set ting of led on
 * @param   hw      [IN] Pointer to the HW structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 */
s32
pch_gbe_phy_led_on(struct pch_gbe_hw *hw)
{
 PCH_DEBUG("pch_gbe_phy_led_on\n");

 pch_gbe_hal_write_phy_reg(hw, PHY_LED_CONTROL, PHY_LED_CTRL_ON);
 return PCH_GBE_SUCCESS;
}

/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_phy_led_off(struct pch_gbe_hw *hw)
 * @brief   Set ting of led off
 * @param   hw      [IN] Pointer to the HW structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 */
s32
pch_gbe_phy_led_off(struct pch_gbe_hw *hw)
{
 PCH_DEBUG("pch_gbe_phy_led_off\n");

 pch_gbe_hal_write_phy_reg(hw, PHY_LED_CONTROL, PHY_LED_CTRL_OFF);
 return PCH_GBE_SUCCESS;
}

/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_phy_led_cleanup(struct pch_gbe_hw *hw)
 * @brief   Cleanup led control
 * @param   hw      [IN] Pointer to the HW structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 */
s32
pch_gbe_phy_led_cleanup(struct pch_gbe_hw *hw)
{
 PCH_DEBUG("pch_gbe_phy_led_cleanup\n");

 pch_gbe_hal_write_phy_reg(hw, PHY_LED_CONTROL, PHY_LED_CTRL_CLEANUP);
 return PCH_GBE_SUCCESS;
}

/*!
 * @ingroup HAL internal function
 * @fn      s32 pch_gbe_phy_led_setup(struct pch_gbe_hw *hw)
 * @brief   Setup led control
 * @param   hw      [IN] Pointer to the HW structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 */
s32
pch_gbe_phy_led_setup(struct pch_gbe_hw *hw)
{
 PCH_DEBUG("pch_gbe_phy_led_setup\n");

 return PCH_GBE_SUCCESS;
}

/*!
 * @ingroup HAL internal function
 * @fn      void pch_gbe_phy_power_up(struct pch_gbe_hw *hw)
 * @brief   restore link in case the phy was powered down
 * @param   hw      [IN] Pointer to the HW structure
 * @return  None
 * @remarks
 *  The phy may be powered down to save power and turn off link when the
 *  driver is unloaded and wake on lan is not enabled (among others)
 *  *** this routine MUST be followed by a call to pch_gbe_reset ***
 */
void pch_gbe_phy_power_up(struct pch_gbe_hw *hw)
{
 u16 mii_reg;

 PCH_DEBUG("pch_gbe_phy_power_up\n");

 mii_reg = 0;
 /* Just clear the power down bit to wake the phy back up */
 /* according to the manual, the phy will retain its
  * settings across a power-down/up cycle */
 pch_gbe_hal_read_phy_reg(hw, PHY_CONTROL, &mii_reg);
 mii_reg &= ~MII_CR_POWER_DOWN;
 pch_gbe_hal_write_phy_reg(hw, PHY_CONTROL, mii_reg);

#ifdef DEBUG_TEST
 pch_gbe_hal_read_phy_reg(hw, PHY_CONTROL, &mii_reg);
 PCH_DEBUG("PHY_CONTROL reg : 0x%08x\n", mii_reg);
#endif
 return;
}

/*!
 * @ingroup HAL internal function
 * @fn      void pch_gbe_phy_power_down(struct pch_gbe_hw *hw)
 * @brief   Power down PHY
 * @param   hw      [IN] Pointer to the HW structure
 * @return  None
 */
void pch_gbe_phy_power_down(struct pch_gbe_hw *hw)
{
 u16 mii_reg;

 PCH_DEBUG("pch_gbe_phy_power_down\n");

 mii_reg = 0;
 /* Power down the PHY so no link is implied when interface is down *
  * The PHY cannot be powered down if any of the following is TRUE *
  * (a) WoL is enabled
  * (b) AMT is active
  */
 pch_gbe_hal_read_phy_reg(hw, PHY_CONTROL, &mii_reg);
 mii_reg |= MII_CR_POWER_DOWN;
 pch_gbe_hal_write_phy_reg(hw, PHY_CONTROL, mii_reg);
 mdelay(1);

#ifdef DEBUG_TEST
 pch_gbe_hal_read_phy_reg(hw, PHY_CONTROL, &mii_reg);
 PCH_DEBUG("PHY_CONTROL reg : 0x%08x\n", mii_reg);
#endif
 return;
}

/*!
 * @ingroup HAL internal function
 * @fn      void pch_gbe_phy_set_rgmii(struct pch_gbe_hw *hw)
 * @brief   RGMII interface setting
 * @param   hw      [IN] Pointer to the HW structure
 * @return  None
 */
#ifdef FPGA
void pch_gbe_phy_set_rgmii(struct pch_gbe_hw *hw)
{
 u16 mii_reg;

 PCH_DEBUG("pch_gbe_phy_set_rgmii\n");

 pch_gbe_hal_read_phy_reg(hw, PHY_EXT_PHYSP_STATUS, &mii_reg);
 mii_reg &= ~HWCFG_MODE_MASK;
 mii_reg |= HWCFG_MODE_RGMII_COPPER;
 pch_gbe_hal_write_phy_reg(hw, PHY_EXT_PHYSP_STATUS, mii_reg);
 pch_gbe_hal_read_phy_reg(hw, PHY_EXT_PHYSP_CONTROL, &mii_reg);
 mii_reg |= 0x01; /* Transfer enable */
 mii_reg |= 0x02; /* add delay to GTX_CLK */
 mii_reg |= 0x80; /* add delay to RX_CLK */
 pch_gbe_hal_write_phy_reg(hw, PHY_EXT_PHYSP_CONTROL, mii_reg);
 pch_gbe_hal_phy_sw_reset(hw);

#ifdef DEBUG_TEST
 pch_gbe_hal_read_phy_reg(hw, PHY_EXT_PHYSP_STATUS, &mii_reg);
 PCH_DEBUG("PHY_EXT_PHYSP_STATUS reg : 0x%08x\n", mii_reg);
 pch_gbe_hal_read_phy_reg(hw, PHY_EXT_PHYSP_CONTROL, &mii_reg);
 PCH_DEBUG("PHY_EXT_PHYSP_CONTROL reg : 0x%08x\n", mii_reg);
#endif
 return;
}
#else
void pch_gbe_phy_set_rgmii(struct pch_gbe_hw *hw)
{
 PCH_DEBUG("pch_gbe_phy_set_rgmii\n");

 pch_gbe_hal_phy_sw_reset(hw);
 return;
}
#endif
/*!
 * @ingroup HAL internal function
 * @fn      void pch_gbe_phy_init_setting(struct pch_gbe_hw *hw)
 * @brief   PHY initial setting
 * @param   hw      [IN] Pointer to the HW structure
 * @return  None
 */
void pch_gbe_phy_init_setting(struct pch_gbe_hw *hw)
{
 struct pch_gbe_adapter *adapter;
 struct ethtool_cmd     cmd;
 int ret;
 u16 mii_reg;

 PCH_DEBUG("pch_gbe_phy_init_setting\n");

 adapter = container_of(hw, struct pch_gbe_adapter, hw);
 ret = mii_ethtool_gset(&adapter->mii, &cmd);
 if (ret != 0)
  PCH_LOG(KERN_ERR, "Error: mii_ethtool_gset\n");

 cmd.speed = hw->mac.link_speed;
 cmd.duplex = hw->mac.link_duplex;
 cmd.advertising = hw->phy.autoneg_advertised;
 cmd.autoneg = hw->mac.autoneg;
 pch_gbe_hal_write_phy_reg(hw, MII_BMCR, BMCR_RESET);
 ret = mii_ethtool_sset(&adapter->mii, &cmd);
 if (ret != 0)
  PCH_LOG(KERN_ERR, "Error: mii_ethtool_sset\n");

 pch_gbe_hal_phy_sw_reset(hw);

 pch_gbe_hal_read_phy_reg(hw, PHY_PHYSP_CONTROL, &mii_reg);
 mii_reg |= PHYSP_CTRL_ASSERT_CRS_TX;
 pch_gbe_hal_write_phy_reg(hw, PHY_PHYSP_CONTROL, mii_reg);

}
