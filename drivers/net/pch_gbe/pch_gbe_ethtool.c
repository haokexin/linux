/*!
 * @file pch_gbe_ethtool.c
 * @brief Linux PCH Gigabit Ethernet Ethtool Driver source file
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
#include <linux/sched.h>
#include <linux/uaccess.h>

#include "pch_debug.h"
#include "pch_gbe_osdep.h"
#include "pch_gbe_regs.h"
#include "pch_gbe_defines.h"
#include "pch_gbe_hw.h"
#include "pch_gbe_api.h"
#include "pch_gbe.h"


/* ---------------------------------------------------------------------------
 Function prototype
--------------------------------------------------------------------------- */
static int pch_gbe_get_settings(struct net_device *netdev,
    struct ethtool_cmd *ecmd);
static int pch_gbe_set_settings(struct net_device *netdev,
    struct ethtool_cmd *ecmd);
static void pch_gbe_get_drvinfo(struct net_device *netdev,
    struct ethtool_drvinfo *drvinfo);
static int pch_gbe_get_regs_len(struct net_device *netdev);
static void pch_gbe_get_regs(struct net_device *netdev,
	struct ethtool_regs *regs, void *p);
static void pch_gbe_get_wol(struct net_device *netdev,
	struct ethtool_wolinfo *wol);
static int pch_gbe_set_wol(struct net_device *netdev,
	struct ethtool_wolinfo *wol);
static u32 pch_gbe_get_msglevel(struct net_device *netdev);
static void pch_gbe_set_msglevel(struct net_device *netdev, u32 data);
static int pch_gbe_nway_reset(struct net_device *netdev);
#ifdef CONFIG_PCH_PHUB
static int pch_gbe_get_eeprom_len(struct net_device *netdev);
static int pch_gbe_get_eeprom(struct net_device *netdev,
	struct ethtool_eeprom *eeprom, u8 *bytes);
static int pch_gbe_set_eeprom(struct net_device *netdev,
	struct ethtool_eeprom *eeprom, u8 *bytes);
#endif
static void pch_gbe_get_ringparam(struct net_device *netdev,
      struct ethtool_ringparam *ring);
static int pch_gbe_set_ringparam(struct net_device *netdev,
     struct ethtool_ringparam *ring);
static void pch_gbe_get_pauseparam(struct net_device *netdev,
       struct ethtool_pauseparam *pause);
static int pch_gbe_set_pauseparam(struct net_device *netdev,
      struct ethtool_pauseparam *pause);
static u32 pch_gbe_get_rx_csum(struct net_device *netdev);
static int pch_gbe_set_rx_csum(struct net_device *netdev, u32 data);
static u32 pch_gbe_get_tx_csum(struct net_device *netdev);
static int pch_gbe_set_tx_csum(struct net_device *netdev, u32 data);
static void pch_gbe_diag_test(struct net_device *netdev,
	struct ethtool_test *eth_test, u64 *data);
static void pch_gbe_get_strings(struct net_device *netdev,
    u32 stringset, u8 *data);
static void pch_gbe_led_blink_callback(unsigned long data);
static int pch_gbe_phys_id(struct net_device *netdev, u32 data);
static void pch_gbe_get_ethtool_stats(struct net_device *netdev,
	struct ethtool_stats *stats, u64 *data);
static int pch_gbe_reg_test(struct pch_gbe_adapter *adapter, uint64_t *data);
static bool reg_pattern_test(struct pch_gbe_adapter *adapter, uint64_t *data,
	int reg, uint32_t mask, uint32_t write);

/* ----------------------------------------------------------------------------
 Data
---------------------------------------------------------------------------- */
/*!
 * @ingroup Ethtool driver Layer
 * @struct  pch_gbe_stats
 * @brief   Stats item infomation
 */
struct pch_gbe_stats {
 signed char stat_string[ETH_GSTRING_LEN];
 int sizeof_stat;
 int stat_offset;
};

#define PCH_GBE_STAT1(m) (int)(sizeof(((struct pch_gbe_adapter *)0)->m))
#define PCH_GBE_STAT2(m) offsetof(struct pch_gbe_adapter, m)

/*!
 * @ingroup Ethtool driver Layer
 * @struct  pch_gbe_gstrings_stats
 * @brief   ethtool information status name list
 */
static const struct pch_gbe_stats pch_gbe_gstrings_stats[] = {
 {"rx_packets", PCH_GBE_STAT1(stats.rx_packets),
  PCH_GBE_STAT2(stats.rx_packets)},
 {"tx_packets", PCH_GBE_STAT1(stats.tx_packets),
  PCH_GBE_STAT2(stats.tx_packets)},
 {"rx_bytes", PCH_GBE_STAT1(stats.rx_bytes),
  PCH_GBE_STAT2(stats.rx_bytes)},
 {"tx_bytes", PCH_GBE_STAT1(stats.tx_bytes),
  PCH_GBE_STAT2(stats.tx_bytes)},
 {"rx_errors", PCH_GBE_STAT1(stats.rx_errors),
  PCH_GBE_STAT2(stats.rx_errors)},
 {"tx_errors", PCH_GBE_STAT1(stats.tx_errors),
  PCH_GBE_STAT2(stats.tx_errors)},
 {"rx_dropped", PCH_GBE_STAT1(stats.rx_dropped),
  PCH_GBE_STAT2(stats.rx_dropped)},
 {"tx_dropped", PCH_GBE_STAT1(stats.tx_dropped),
  PCH_GBE_STAT2(stats.tx_dropped)},
 {"multicast", PCH_GBE_STAT1(stats.multicast),
  PCH_GBE_STAT2(stats.multicast)},
 {"collisions", PCH_GBE_STAT1(stats.collisions),
  PCH_GBE_STAT2(stats.collisions)},
 {"rx_crc_errors", PCH_GBE_STAT1(stats.rx_crc_errors),
  PCH_GBE_STAT2(stats.rx_crc_errors)},
 {"rx_frame_errors", PCH_GBE_STAT1(stats.rx_frame_errors),
  PCH_GBE_STAT2(stats.rx_frame_errors)},
 {"rx_buff_failed", PCH_GBE_STAT1(stats.rx_alloc_buff_failed),
  PCH_GBE_STAT2(stats.rx_alloc_buff_failed)},
 {"tx_length_errors", PCH_GBE_STAT1(stats.tx_length_errors),
  PCH_GBE_STAT2(stats.tx_length_errors)},
 {"tx_aborted_errors", PCH_GBE_STAT1(stats.tx_aborted_errors),
  PCH_GBE_STAT2(stats.tx_aborted_errors)},
 {"tx_carrier_errors", PCH_GBE_STAT1(stats.tx_carrier_errors),
  PCH_GBE_STAT2(stats.tx_carrier_errors)},
 {"tx_timeout_count", PCH_GBE_STAT1(stats.tx_timeout_count),
  PCH_GBE_STAT2(stats.tx_timeout_count)},
 {"tx_restart_count", PCH_GBE_STAT1(stats.tx_restart_count),
  PCH_GBE_STAT2(stats.tx_restart_count)},
 {"intr_rx_dsc_empty_count",
  PCH_GBE_STAT1(stats.intr_rx_dsc_empty_count),
  PCH_GBE_STAT2(stats.intr_rx_dsc_empty_count)},
 {"intr_rx_frame_err_count",
  PCH_GBE_STAT1(stats.intr_rx_frame_err_count),
  PCH_GBE_STAT2(stats.intr_rx_frame_err_count)},
 {"intr_rx_fifo_err_count", PCH_GBE_STAT1(stats.intr_rx_fifo_err_count),
  PCH_GBE_STAT2(stats.intr_rx_fifo_err_count)},
 {"intr_rx_dma_err_count", PCH_GBE_STAT1(stats.intr_rx_dma_err_count),
  PCH_GBE_STAT2(stats.intr_rx_dma_err_count)},
 {"intr_tx_fifo_err_count", PCH_GBE_STAT1(stats.intr_tx_fifo_err_count),
  PCH_GBE_STAT2(stats.intr_tx_fifo_err_count)},
 {"intr_tx_dma_err_count", PCH_GBE_STAT1(stats.intr_tx_dma_err_count),
  PCH_GBE_STAT2(stats.intr_tx_dma_err_count)},
 {"intr_tcpip_err_count", PCH_GBE_STAT1(stats.intr_tcpip_err_count),
  PCH_GBE_STAT2(stats.intr_tcpip_err_count)}
};

#define PCH_GBE_QUEUE_STATS_LEN 0
#define PCH_GBE_GLOBAL_STATS_LEN \
((int)sizeof(pch_gbe_gstrings_stats) / (int)sizeof(struct pch_gbe_stats))

/*!
 * @ingroup  Ethtool driver
 * @def      PCH_GBE_STATS_LEN
 * @brief    The size of status
*/
#define PCH_GBE_STATS_LEN (PCH_GBE_GLOBAL_STATS_LEN + PCH_GBE_QUEUE_STATS_LEN)

/*!
 * @ingroup Ethtool driver Layer
 * @struct  pch_gbe_gstrings_test
 * @brief   self test item name list
 */
static const signed char pch_gbe_gstrings_test[][ETH_GSTRING_LEN] = {
 "Register test  (offline)"
};

/*!
 * @ingroup  Ethtool driver
 * @def      PCH_GBE_TEST_LEN
 * @brief    The size of test packet
*/
#define PCH_GBE_TEST_LEN \
 ((int)sizeof(pch_gbe_gstrings_test) / ETH_GSTRING_LEN)

/*!
 * @ingroup  Ethtool driver
 * @def      PCH_GBE_ID_INTERVAL
 * @brief    Toggle LED 4 times per second = 2 "blinks" per second
*/
#define PCH_GBE_ID_INTERVAL (HZ/4)

/*!
 * @ingroup  Ethtool driver
 * @def      PCH_GBE_LED_ON
 * @brief    Bit defines for adapter->led_status
*/
#define PCH_GBE_LED_ON  0

/* ----------------------------------------------------------------------------
 Function
---------------------------------------------------------------------------- */
/*!
 * @ingroup Ethtool driver
 * @fn      static int pch_gbe_get_settings(struct net_device *netdev,
 *                                          struct ethtool_cmd *ecmd)
 * @brief   Get device-specific settings
 * @param   netdev [INOUT] Network interface device structure
 * @param   ecmd   [INOUT] Ethtool command
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:   Failed
 */
static int
pch_gbe_get_settings(struct net_device *netdev, struct ethtool_cmd *ecmd)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);
 int ret;

 PCH_DEBUG("ethtool: pch_gbe_get_settings\n");

 ret = mii_ethtool_gset(&adapter->mii, ecmd);
 ecmd->supported &= (u32) (~SUPPORTED_TP);
 ecmd->supported &= (u32) (~SUPPORTED_1000baseT_Half);
 ecmd->advertising &= (u32) (~ADVERTISED_TP);
 ecmd->advertising &= (u32) (~ADVERTISED_1000baseT_Half);

 if (netif_carrier_ok(adapter->netdev)) {
  ;
 } else {
  ecmd->speed = 0xFFFF;
  ecmd->duplex = 0xFF;
 }
 return ret;
}

/*!
 * @ingroup Ethtool driver
 * @fn      static int pch_gbe_set_settings(struct net_device *netdev,
 *                                          struct ethtool_cmd *ecmd)
 * @brief   Set device-specific settings
 * @param   netdev [INOUT] Network interface device structure
 * @param   ecmd   [INOUT] Ethtool command
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:   Failed
 */
static int
pch_gbe_set_settings(struct net_device *netdev, struct ethtool_cmd *ecmd)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);
 struct pch_gbe_hw *hw = &adapter->hw;
 int ret;

 PCH_DEBUG("ethtool: pch_gbe_set_settings\n");

 while (test_and_set_bit(__PCH_GBE_RESETTING, &adapter->flags) != 0)
  msleep(1);
 pch_gbe_hal_write_phy_reg(hw, MII_BMCR, BMCR_RESET);

 if (ecmd->speed == 0xFFFF)
  ecmd->speed = SPEED_1000;
 if (ecmd->duplex == 0xFF)
  ecmd->duplex = DUPLEX_FULL;
 ret = mii_ethtool_sset(&adapter->mii, ecmd);
 if (ret != 0) {
  PCH_LOG(KERN_ERR, "Error: mii_ethtool_sset\n");
  clear_bit(__PCH_GBE_RESETTING, &adapter->flags);
  return ret;
 }
 hw->mac.link_speed = ecmd->speed;
 hw->mac.link_duplex = ecmd->duplex;
 hw->phy.autoneg_advertised = ecmd->advertising;
 hw->mac.autoneg = ecmd->autoneg;
 pch_gbe_hal_phy_sw_reset(hw);

 /* reset the link */
 if (netif_running(adapter->netdev) != 0) {
  pch_gbe_down(adapter);
  ret = pch_gbe_up(adapter);
 } else {
  pch_gbe_reset(adapter);
 }

 clear_bit(__PCH_GBE_RESETTING, &adapter->flags);
 return ret;
}

/*!
 * @ingroup Ethtool driver
 * @fn      static void pch_gbe_get_drvinfo(struct net_device *netdev,
 *                                          struct ethtool_drvinfo *drvinfo)
 * @brief   Report driver information
 * @param   netdev  [INOUT] Network interface device structure
 * @param   drvinfo [INOUT] Driver information structure
 * @return  None
 */
static void
pch_gbe_get_drvinfo(struct net_device *netdev, struct ethtool_drvinfo *drvinfo)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);

 PCH_DEBUG("ethtool: pch_gbe_get_drvinfo\n");

 strcpy(drvinfo->driver, DRV_NAME);
 strcpy(drvinfo->version, DRV_VERSION);
 strcpy(drvinfo->fw_version, FIRM_VERSION);
 strcpy(drvinfo->bus_info, pci_name(adapter->pdev));

 drvinfo->n_stats = PCH_GBE_STATS_LEN;
 drvinfo->testinfo_len = PCH_GBE_TEST_LEN;
 drvinfo->regdump_len = pch_gbe_get_regs_len(netdev);
#ifdef CONFIG_PCH_PHUB
 drvinfo->eedump_len = pch_gbe_get_eeprom_len(netdev);
#endif
}

/*!
 * @ingroup Ethtool driver
 * @fn      static int pch_gbe_get_regs_len(struct net_device *netdev)
 * @brief   Report the size of device registers
 * @param   netdev [INOUT] Network interface device structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:   Failed
 */
static int pch_gbe_get_regs_len(struct net_device *netdev)
{
 PCH_DEBUG("ethtool: pch_gbe_get_regs_len\n");

 return PCH_GBE_REGS_LEN * (int)sizeof(u32);
}

/*!
 * @ingroup Ethtool driver
 * @fn      static void pch_gbe_get_regs(struct net_device *netdev,
 *                                       struct ethtool_regs *regs, void *p)
 * @brief   Get device registers
 * @param   netdev [INOUT] Network interface device structure
 * @param   regs   [INOUT] Ethtool register structure
 * @param   p      [INOUT] Buffer pointer of read device register date
 * @return  None
 */
static void
pch_gbe_get_regs(struct net_device *netdev, struct ethtool_regs *regs, void *p)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);
 struct pch_gbe_hw *hw = &adapter->hw;
 u32 *regs_buff = p;
 u16 i, reg, tmp;

 PCH_DEBUG("ethtool: pch_gbe_get_regs\n");

 regs->version = hw->revision_id;
 regs->version = 0x1000000 | (regs->version << 16) | hw->device_id;

 memset(p, 0, PCH_GBE_REGS_LEN * (int)sizeof(u32));
 /* 000: */
 regs_buff[0] = PCH_GBE_READ_REG(hw, INT_ST);
 regs_buff[1] = PCH_GBE_READ_REG(hw, INT_EN);
 regs_buff[2] = PCH_GBE_READ_REG(hw, MODE);
 regs_buff[3] = PCH_GBE_READ_REG(hw, RESET);
 /* 010: */
 regs_buff[4] = PCH_GBE_READ_REG(hw, TCPIP_ACC);
 regs_buff[5] = PCH_GBE_READ_REG(hw, EX_LIST);
 regs_buff[6] = PCH_GBE_READ_REG(hw, INT_ST_HOLD);
 regs_buff[7] = PCH_GBE_READ_REG(hw, PHY_INT_CTRL);
 /* 020: */
 regs_buff[8] = PCH_GBE_READ_REG(hw, MAC_RX_EN);
 regs_buff[9] = PCH_GBE_READ_REG(hw, RX_FCTRL);
 regs_buff[10] = PCH_GBE_READ_REG(hw, PAUSE_REQ);
 regs_buff[11] = PCH_GBE_READ_REG(hw, RX_MODE);
 /* 030: */
 regs_buff[12] = PCH_GBE_READ_REG(hw, TX_MODE);
 regs_buff[13] = PCH_GBE_READ_REG(hw, RX_FIFO_ST);
 regs_buff[14] = PCH_GBE_READ_REG(hw, TX_FIFO_ST);
 regs_buff[15] = PCH_GBE_READ_REG(hw, TX_FID);
 /* 040: */
 regs_buff[16] = PCH_GBE_READ_REG(hw, TX_RESULT);
 regs_buff[17] = PCH_GBE_READ_REG(hw, PAUSE_PKT1);
 regs_buff[18] = PCH_GBE_READ_REG(hw, PAUSE_PKT2);
 regs_buff[19] = PCH_GBE_READ_REG(hw, PAUSE_PKT3);
 /* 050: */
 regs_buff[20] = PCH_GBE_READ_REG(hw, PAUSE_PKT4);
 regs_buff[21] = PCH_GBE_READ_REG(hw, PAUSE_PKT5);
 regs_buff[22] = PCH_GBE_READ_REG(hw, MAC_ADR);
 regs_buff[23] = PCH_GBE_READ_REG(hw, MAC_ADR1A);
 /* 060: */
 regs_buff[24] = PCH_GBE_READ_REG(hw, MAC_ADR1B);
 regs_buff[25] = PCH_GBE_READ_REG(hw, MAC_ADR2A);
 regs_buff[26] = PCH_GBE_READ_REG(hw, MAC_ADR2B);
 regs_buff[27] = PCH_GBE_READ_REG(hw, MAC_ADR3A);
 /* 070: */
 regs_buff[28] = PCH_GBE_READ_REG(hw, MAC_ADR3B);
 regs_buff[29] = PCH_GBE_READ_REG(hw, MAC_ADR4A);
 regs_buff[30] = PCH_GBE_READ_REG(hw, MAC_ADR4B);
 regs_buff[31] = PCH_GBE_READ_REG(hw, MAC_ADR5A);
 /* 080: */
 regs_buff[32] = PCH_GBE_READ_REG(hw, MAC_ADR5B);
 regs_buff[33] = PCH_GBE_READ_REG(hw, MAC_ADR6A);
 regs_buff[34] = PCH_GBE_READ_REG(hw, MAC_ADR6B);
 regs_buff[35] = PCH_GBE_READ_REG(hw, MAC_ADR7A);
 /* 090: */
 regs_buff[36] = PCH_GBE_READ_REG(hw, MAC_ADR7B);
 regs_buff[37] = PCH_GBE_READ_REG(hw, MAC_ADR8A);
 regs_buff[38] = PCH_GBE_READ_REG(hw, MAC_ADR8B);
 regs_buff[39] = PCH_GBE_READ_REG(hw, MAC_ADR9A);
 /* 0a0: */
 regs_buff[40] = PCH_GBE_READ_REG(hw, MAC_ADR9B);
 regs_buff[41] = PCH_GBE_READ_REG(hw, MAC_ADR10A);
 regs_buff[42] = PCH_GBE_READ_REG(hw, MAC_ADR10B);
 regs_buff[43] = PCH_GBE_READ_REG(hw, MAC_ADR11A);
 /* 0b0: */
 regs_buff[44] = PCH_GBE_READ_REG(hw, MAC_ADR11B);
 regs_buff[45] = PCH_GBE_READ_REG(hw, MAC_ADR12A);
 regs_buff[46] = PCH_GBE_READ_REG(hw, MAC_ADR12B);
 regs_buff[47] = PCH_GBE_READ_REG(hw, MAC_ADR13A);
 /* 0c0: */
 regs_buff[48] = PCH_GBE_READ_REG(hw, MAC_ADR13B);
 regs_buff[49] = PCH_GBE_READ_REG(hw, MAC_ADR14A);
 regs_buff[50] = PCH_GBE_READ_REG(hw, MAC_ADR14B);
 regs_buff[51] = PCH_GBE_READ_REG(hw, MAC_ADR15A);
 /* 0d0: */
 regs_buff[52] = PCH_GBE_READ_REG(hw, MAC_ADR15B);
 regs_buff[53] = PCH_GBE_READ_REG(hw, MAC_ADR16A);
 regs_buff[54] = PCH_GBE_READ_REG(hw, MAC_ADR16B);
 regs_buff[55] = PCH_GBE_READ_REG(hw, ADDR_MASK);
 /* 0e0: */
 regs_buff[56] = PCH_GBE_READ_REG(hw, MIIM);
 regs_buff[57] = PCH_GBE_READ_REG(hw, RGMII_ST);
 regs_buff[58] = PCH_GBE_READ_REG(hw, RGMII_CTRL);
 regs_buff[59] = PCH_GBE_READ_REG(hw, DMA_CTRL);
 /* 0f0: */
 regs_buff[60] = PCH_GBE_READ_REG(hw, RX_DSC_BASE);
 regs_buff[61] = PCH_GBE_READ_REG(hw, RX_DSC_SIZE);
 regs_buff[62] = PCH_GBE_READ_REG(hw, RX_DSC_HW_P);
 regs_buff[63] = PCH_GBE_READ_REG(hw, RX_DSC_HW_P_HLD);
 /* 100: */
 regs_buff[64] = PCH_GBE_READ_REG(hw, RX_DSC_SW_P);
 regs_buff[65] = PCH_GBE_READ_REG(hw, TX_DSC_BASE);
 regs_buff[66] = PCH_GBE_READ_REG(hw, TX_DSC_SIZE);
 regs_buff[67] = PCH_GBE_READ_REG(hw, TX_DSC_HW_P);
 /* 110: */
 regs_buff[68] = PCH_GBE_READ_REG(hw, TX_DSC_HW_P_HLD);
 regs_buff[69] = PCH_GBE_READ_REG(hw, TX_DSC_SW_P);
 regs_buff[70] = PCH_GBE_READ_REG(hw, RX_DMA_ST);
 regs_buff[71] = PCH_GBE_READ_REG(hw, TX_DMA_ST);
 /* 120: */
 regs_buff[72] = PCH_GBE_READ_REG(hw, WOL_ST);
 regs_buff[73] = PCH_GBE_READ_REG(hw, WOL_CTRL);
 regs_buff[74] = PCH_GBE_READ_REG(hw, WOL_ADDR_MASK);
 regs_buff[75] = 0x00000000; /* Dummy read */

 /* 130: */
 /* PHY register */
 for (i = PCH_GBE_MAC_REGS_LEN, reg = 0; reg < PCH_GBE_PHY_REGS_LEN;
      i++, reg++) {
  pch_gbe_hal_read_phy_reg(&adapter->hw, reg, &tmp);
  regs_buff[i] = tmp;
 }

}

/*!
 * @ingroup Ethtool driver
 * @fn      static void pch_gbe_get_wol(struct net_device *netdev,
 *                                      struct ethtool_wolinfo *wol)
 * @brief   Report whether Wake-on-Lan is enabled
 * @param   netdev [INOUT] Network interface device structure
 * @param   wol    [OUT] Wake-on-Lan information
 * @return  None
 */
static void
pch_gbe_get_wol(struct net_device *netdev, struct ethtool_wolinfo *wol)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);

 PCH_DEBUG("ethtool: pch_gbe_get_wol\n");

 wol->supported = WAKE_UCAST | WAKE_MCAST | WAKE_BCAST | WAKE_MAGIC;
 wol->wolopts = 0;

 if ((adapter->wake_up_evt & PCH_GBE_WLC_IND) != 0)
  wol->wolopts |= WAKE_UCAST;
 if ((adapter->wake_up_evt & PCH_GBE_WLC_MLT) != 0)
  wol->wolopts |= WAKE_MCAST;
 if ((adapter->wake_up_evt & PCH_GBE_WLC_BR) != 0)
  wol->wolopts |= WAKE_BCAST;
 if ((adapter->wake_up_evt & PCH_GBE_WLC_MP) != 0)
  wol->wolopts |= WAKE_MAGIC;
 return;
}

/*!
 * @ingroup Ethtool driver
 * @fn      static int pch_gbe_set_wol(struct net_device *netdev,
 *                                     struct ethtool_wolinfo *wol)
 * @brief   Turn Wake-on-Lan on or off
 * @param   netdev [INOUT] Network interface device structure
 * @param   wol    [IN] Pointer of wake-on-Lan information straucture
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:   Failed
 */
static int
pch_gbe_set_wol(struct net_device *netdev, struct ethtool_wolinfo *wol)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);

 PCH_DEBUG("ethtool: pch_gbe_set_wol\n");

 if ((wol->wolopts & (WAKE_PHY | WAKE_ARP | WAKE_MAGICSECURE)) != 0)
  return -EOPNOTSUPP;
 /* these settings will always override what we currently have */
 adapter->wake_up_evt = 0;

 if ((wol->wolopts & WAKE_UCAST) != 0)
  adapter->wake_up_evt |= PCH_GBE_WLC_IND;
 if ((wol->wolopts & WAKE_MCAST) != 0)
  adapter->wake_up_evt |= PCH_GBE_WLC_MLT;
 if ((wol->wolopts & WAKE_BCAST) != 0)
  adapter->wake_up_evt |= PCH_GBE_WLC_BR;
 if ((wol->wolopts & WAKE_MAGIC) != 0)
  adapter->wake_up_evt |= PCH_GBE_WLC_MP;
 return PCH_GBE_SUCCESS;
}

/*!
 * @ingroup Ethtool driver
 * @fn      static u32 pch_gbe_get_msglevel(struct net_device *netdev)
 * @brief   Report driver message level
 * @param   netdev [INOUT] Network interface device structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:   Failed
 */
static u32 pch_gbe_get_msglevel(struct net_device *netdev)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);

 PCH_DEBUG("ethtool: pch_gbe_get_msglevel\n");

 return adapter->msg_enable;
}

/*!
 * @ingroup Ethtool driver
 * @fn      static void pch_gbe_set_msglevel(struct net_device *netdev,
 *                                           u32 data)
 * @brief   Set driver message level
 * @param   netdev [INOUT] Network interface device structure
 * @param   data   [IN] Driver message level
 * @return  None
 */
static void pch_gbe_set_msglevel(struct net_device *netdev, u32 data)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);

 PCH_DEBUG("ethtool: pch_gbe_set_msglevel\n");

 adapter->msg_enable = data;
}

/*!
 * @ingroup Ethtool driver
 * @fn      static int pch_gbe_nway_reset(struct net_device *netdev)
 * @brief   Restart autonegotiation
 * @param   netdev [INOUT] Network interface device structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:   Failed
 */
static int pch_gbe_nway_reset(struct net_device *netdev)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);

 PCH_DEBUG("ethtool: pch_gbe_nway_reset\n");

 return mii_nway_restart(&adapter->mii);
}

#ifdef CONFIG_PCH_PHUB
/*!
 * @ingroup Ethtool driver
 * @fn      static int pch_gbe_get_eeprom_len(struct net_device *netdev)
 * @brief   Report the device EEPROM memory size
 * @param   netdev [INOUT] Network interface device structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:   Failed
 */
static int pch_gbe_get_eeprom_len(struct net_device *netdev)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);

 PCH_DEBUG("ethtool: pch_gbe_get_eeprom_len\n");

 return adapter->hw.nvm.word_size * 2;
}

/*!
 * @ingroup Ethtool driver
 * @fn      static int pch_gbe_get_eeprom(struct net_device *netdev,
 *                         struct ethtool_eeprom *eeprom, u8 *bytes)
 * @brief   Read data from the device EEPROM
 * @param   netdev [INOUT] Network interface device structure
 * @param   eeprom [INOUT] EEPROM get information structur
 * @param   bytes  [OUT] Pointer of read data
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:   Failed
 */
static int
pch_gbe_get_eeprom(struct net_device *netdev,
     struct ethtool_eeprom *eeprom, u8 *bytes)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);
 struct pch_gbe_hw *hw = &adapter->hw;
 int ret = PCH_GBE_SUCCESS;
 u32 offset;
 u8 i;

 PCH_DEBUG("ethtool: pch_gbe_get_eeprom\n");

 if (eeprom->len == 0)
  return -EINVAL;
 eeprom->magic = (hw->vendor_id);

 for (i = 0, offset = eeprom->offset; i < (eeprom->len); i++, offset++) {
  ret = pch_gbe_hal_read_nvm(hw, offset, (bytes + i));
  if (ret)
   break;
 }
 return ret;
}

/*!
 * @ingroup Ethtool driver
 * @fn      static int pch_gbe_set_eeprom(struct net_device *netdev,
 *                         struct ethtool_eeprom *eeprom, u8 *bytes)
 * @brief   Write data to the device EEPROM
 * @param   netdev [INOUT] Network interface device structure
 * @param   eeprom [INOUT] EEPROM get information structur
 * @param   bytes  [IN] Pointer of write data
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:   Failed
 */
static int
pch_gbe_set_eeprom(struct net_device *netdev,
     struct ethtool_eeprom *eeprom, u8 *bytes)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);
 struct pch_gbe_hw *hw = &adapter->hw;
 int ret;
 u32 offset;
 u8 i;

 PCH_DEBUG("ethtool: pch_gbe_set_eeprom\n");

 if (eeprom->len == 0) {
  PCH_LOG(KERN_ERR, "EOPNOTSUPP\n");
  return -EOPNOTSUPP;
 }
 if (eeprom->magic != (hw->vendor_id)) {
  PCH_LOG(KERN_ERR, "EFAULT\n");
  PCH_DEBUG("eeprom->magic : 0x%08x  magic : %0x\n",
    eeprom->magic, (hw->vendor_id));
  return -EFAULT;
 }

 for (i = 0, offset = eeprom->offset; i < (eeprom->len); i++, offset++) {
  ret = pch_gbe_hal_write_nvm(hw, offset, (bytes + i));
  if (ret)
   return ret;
 }
 return PCH_GBE_SUCCESS;
}
#endif

/*!
 * @ingroup Ethtool driver
 * @fn      static void pch_gbe_get_ringparam(struct net_device *netdev,
 *                                            struct ethtool_ringparam *ring)
 * @brief   Report ring sizes
 * @param   netdev [INOUT] Network interface device structure
 * @param   ring   [OUT] Ring param structure
 * @return  None
 */
static void
pch_gbe_get_ringparam(struct net_device *netdev, struct ethtool_ringparam *ring)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);
 struct pch_gbe_tx_ring *txdr = adapter->tx_ring;
 struct pch_gbe_rx_ring *rxdr = adapter->rx_ring;

 PCH_DEBUG("ethtool: pch_gbe_get_ringparam\n");

 ring->rx_max_pending = PCH_GBE_MAX_RXD;
 ring->tx_max_pending = PCH_GBE_MAX_TXD;
 ring->rx_mini_max_pending = 0;
 ring->rx_jumbo_max_pending = 0;
 ring->rx_pending = rxdr->count;
 ring->tx_pending = txdr->count;
 ring->rx_mini_pending = 0;
 ring->rx_jumbo_pending = 0;
}

/*!
 * @ingroup Ethtool driver
 * @fn      static int pch_gbe_set_ringparam(struct net_device *netdev,
 *                                           struct ethtool_ringparam *ring)
 * @brief   Set ring sizes
 * @param   netdev [INOUT] Network interface device structure
 * @param   ring   [IN] Ring param structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:   Failed
 */
static int
pch_gbe_set_ringparam(struct net_device *netdev, struct ethtool_ringparam *ring)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);
 struct pch_gbe_tx_ring *txdr, *tx_old;
 struct pch_gbe_rx_ring *rxdr, *rx_old;
 int tx_ring_size, rx_ring_size;
 int err = PCH_GBE_SUCCESS;

 PCH_DEBUG("ethtool: pch_gbe_set_ringparam\n");

 if ((ring->rx_mini_pending) || (ring->rx_jumbo_pending))
  return -EINVAL;
 tx_ring_size = (int)sizeof(struct pch_gbe_tx_ring);
 rx_ring_size = (int)sizeof(struct pch_gbe_rx_ring);

 while ((test_and_set_bit(__PCH_GBE_RESETTING, &adapter->flags)) != 0)
  msleep(1);
 if ((netif_running(adapter->netdev)) != 0)
  pch_gbe_down(adapter);
 tx_old = adapter->tx_ring;
 rx_old = adapter->rx_ring;

 txdr = kzalloc(tx_ring_size, GFP_KERNEL);
 if (!txdr) {
  err = -ENOMEM;
  goto err_alloc_tx;
 }
 rxdr = kzalloc(rx_ring_size, GFP_KERNEL);
 if (!rxdr) {
  err = -ENOMEM;
  goto err_alloc_rx;
 }
 adapter->tx_ring = txdr;
 adapter->rx_ring = rxdr;

 rxdr->count = max(ring->rx_pending, (u32) PCH_GBE_MIN_RXD);
 rxdr->count = min(rxdr->count, (u32) PCH_GBE_MAX_RXD);
 PCH_GBE_ROUNDUP(rxdr->count, PCH_GBE_RX_DESC_MULTIPLE);

 txdr->count = max(ring->tx_pending, (u32) PCH_GBE_MIN_TXD);
 txdr->count = min(txdr->count, (u32) PCH_GBE_MAX_TXD);
 PCH_GBE_ROUNDUP(txdr->count, PCH_GBE_TX_DESC_MULTIPLE);

 if ((netif_running(adapter->netdev)) != 0) {
  /* Try to get new resources before deleting old */
  err = pch_gbe_setup_rx_resources(adapter, adapter->rx_ring);
  if (err != 0)
   goto err_setup_rx;
  err = pch_gbe_setup_tx_resources(adapter, adapter->tx_ring);
  if (err != 0)
   goto err_setup_tx;
  /* save the new, restore the old in order to free it,
   * then restore the new back again */
  adapter->rx_ring = rx_old;
  adapter->tx_ring = tx_old;
  pch_gbe_free_rx_resources(adapter, adapter->rx_ring);
  pch_gbe_free_tx_resources(adapter, adapter->tx_ring);
  kfree(tx_old);
  kfree(rx_old);
  adapter->rx_ring = rxdr;
  adapter->tx_ring = txdr;
  err = pch_gbe_up(adapter);
 }

 clear_bit(__PCH_GBE_RESETTING, &adapter->flags);
 return err;

err_setup_tx:
 pch_gbe_free_rx_resources(adapter, adapter->rx_ring);
err_setup_rx:
 adapter->rx_ring = rx_old;
 adapter->tx_ring = tx_old;
 kfree(rxdr);
err_alloc_rx:
 kfree(txdr);
err_alloc_tx:
 if (netif_running(adapter->netdev))
  pch_gbe_up(adapter);
 clear_bit(__PCH_GBE_RESETTING, &adapter->flags);
 return err;
}

/*!
 * @ingroup Ethtool driver
 * @fn      static void pch_gbe_get_pauseparam(struct net_device *netdev,
 *                                            struct ethtool_pauseparam *pause)
 * @brief   Report pause parameters
 * @param   netdev [INOUT] Network interface device structure
 * @param   pause  [OUT] Pause parameters structure
 * @return  None
 */
static void
pch_gbe_get_pauseparam(struct net_device *netdev,
	struct ethtool_pauseparam *pause)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);
 struct pch_gbe_hw *hw = &adapter->hw;

 PCH_DEBUG("ethtool: pch_gbe_get_pauseparam\n");

 pause->autoneg =
     ((hw->mac.fc_autoneg != 0) ? AUTONEG_ENABLE : AUTONEG_DISABLE);

 if (hw->mac.fc == pch_gbe_fc_rx_pause) {
  pause->rx_pause = 1;
 } else if (hw->mac.fc == pch_gbe_fc_tx_pause) {
  pause->tx_pause = 1;
 } else if (hw->mac.fc == pch_gbe_fc_full) {
  pause->rx_pause = 1;
  pause->tx_pause = 1;
 }
}

/*!
 * @ingroup Ethtool driver
 * @fn      static int pch_gbe_set_pauseparam(struct net_device *netdev,
 *                                     struct ethtool_pauseparam *pause)
 * @brief   Set pause paramters
 * @param   netdev [INOUT] Network interface device structure
 * @param   pause  [IN] Pause parameters structure
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:   Failed
 */
static int
pch_gbe_set_pauseparam(struct net_device *netdev,
	struct ethtool_pauseparam *pause)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);
 struct pch_gbe_hw *hw = &adapter->hw;
 int ret = PCH_GBE_SUCCESS;

 PCH_DEBUG("ethtool: pch_gbe_set_pauseparam\n");

 hw->mac.fc_autoneg = pause->autoneg;

 while ((test_and_set_bit(__PCH_GBE_RESETTING, &adapter->flags)) != 0)
  msleep(1);
 if ((pause->rx_pause) && (pause->tx_pause))
  hw->mac.fc = pch_gbe_fc_full;
 else if ((pause->rx_pause) && (!pause->tx_pause))
  hw->mac.fc = pch_gbe_fc_rx_pause;
 else if ((!pause->rx_pause) && (pause->tx_pause))
  hw->mac.fc = pch_gbe_fc_tx_pause;
 else if ((!pause->rx_pause) && (!pause->tx_pause))
  hw->mac.fc = pch_gbe_fc_none;

 if (hw->mac.fc_autoneg == AUTONEG_ENABLE) {
  if ((netif_running(adapter->netdev)) != 0) {
   pch_gbe_down(adapter);
   ret = pch_gbe_up(adapter);
  } else {
   pch_gbe_reset(adapter);
  }
 } else {
  ret = pch_gbe_hal_force_mac_fc(hw);
 }
 clear_bit(__PCH_GBE_RESETTING, &adapter->flags);
 return ret;
}

/*!
 * @ingroup Ethtool driver
 * @fn      static u32 pch_gbe_get_rx_csum(struct net_device *netdev)
 * @brief   Report whether receive checksums are turned on or off
 * @param   netdev [INOUT] Network interface device structure
 * @return  TRUE(1):  Checksum On
 * @return  FALSE(0): Checksum Off
 */
static u32 pch_gbe_get_rx_csum(struct net_device *netdev)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);

 PCH_DEBUG("ethtool: pch_gbe_get_rx_csum\n");

 return adapter->rx_csum;
}

/*!
 * @ingroup Ethtool driver
 * @fn      static int pch_gbe_set_rx_csum(struct net_device *netdev, u32 data)
 * @brief   Turn receive checksum on or off
 * @param   netdev [INOUT] Network interface device structure
 * @param   data   [IN] Checksum On[TRUE] or Off[FALSE]
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:   Failed
 */
static int pch_gbe_set_rx_csum(struct net_device *netdev, u32 data)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);

 PCH_DEBUG("ethtool: pch_gbe_set_rx_csum\n");

 adapter->rx_csum = data;
 if ((netif_running(netdev)) != 0)
  pch_gbe_reinit_locked(adapter);
 else
  pch_gbe_reset(adapter);

 return PCH_GBE_SUCCESS;
}

/*!
 * @ingroup Ethtool driver
 * @fn      static u32 pch_gbe_get_tx_csum(struct net_device *netdev)
 * @brief   Report whether transmit checksums are turned on or off
 * @param   netdev [INOUT] Network interface device structure
 * @return  TRUE(1):  Checksum On
 * @return  FALSE(0): Checksum Off
 */
static u32 pch_gbe_get_tx_csum(struct net_device *netdev)
{
 PCH_DEBUG("ethtool: pch_gbe_get_tx_csum\n");

 return (netdev->features & NETIF_F_HW_CSUM) != 0;
}

/*!
 * @ingroup Ethtool driver
 * @fn      static int pch_gbe_set_tx_csum(struct net_device *netdev, u32 data)
 * @brief   Turn transmit checksums on or off
 * @param   netdev [INOUT] Network interface device structure
 * @param   data   [IN] Checksum on[TRUE] or off[FALSE]
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:   Failed
 */
static int pch_gbe_set_tx_csum(struct net_device *netdev, u32 data)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);

 PCH_DEBUG("ethtool: pch_gbe_set_tx_csum\n");

 adapter->tx_csum = data;

 if (data != 0)
  netdev->features |= NETIF_F_HW_CSUM;
 else
  netdev->features &= ~NETIF_F_HW_CSUM;

 return PCH_GBE_SUCCESS;
}

/*!
 * @ingroup Ethtool driver
 * @fn      static void pch_gbe_diag_test(struct net_device *netdev,
 *                         struct ethtool_test *eth_test, u64 *data)
 * @brief   Run specified self-tests
 * @param   netdev   [IN] Network interface device structure
 * @param   eth_test [IN] Ethtool test structure
 * @param   data     [OUT] Data for test result.
 * @return  None
 */
static void
pch_gbe_diag_test(struct net_device *netdev,
    struct ethtool_test *eth_test, u64 *data)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);
 unsigned char if_running = netif_running(netdev);

 PCH_DEBUG("ethtool: pch_gbe_diag_test\n");

 set_bit(__PCH_GBE_TESTING, &adapter->flags);

 if (eth_test->flags == ETH_TEST_FL_OFFLINE) {
  /* Offline tests */
  DPRINTK(HW, INFO, "offline testing starting\n");

  if (if_running) {
   /* indicate we're in test mode */
   dev_close(netdev);
  } else {
   pch_gbe_reset(adapter);
  }
  /* Register test */
  if (pch_gbe_reg_test(adapter, &data[0]))
   eth_test->flags |= ETH_TEST_FL_FAILED;

  pch_gbe_reset(adapter);
  clear_bit(__PCH_GBE_TESTING, &adapter->flags);
  if (if_running)
   dev_open(netdev);

 } else {
  /* Online tests */
  DPRINTK(HW, INFO, "online testing starting\n");
  data[0] = 0;
  clear_bit(__PCH_GBE_TESTING, &adapter->flags);
 }
}

/*!
 * @ingroup Ethtool driver
 * @fn      static void pch_gbe_get_strings(struct net_device *netdev,
 *                                             u32 stringset, u8 *data)
 * @brief   Return a set of strings that describe the requested objects
 * @param   netdev    [INOUT] Network interface device structure
 * @param   stringset [IN] Select the stringset. [ETH_SS_TEST] [ETH_SS_STATS]
 * @param   data      [OUT]Pointer of read string data.
 * @return  None
 */
static void
pch_gbe_get_strings(struct net_device *netdev, u32 stringset, u8 *data)
{
 u8 *p = data;
 int i;

 PCH_DEBUG("ethtool: pch_gbe_get_strings\n");

 switch (stringset) {
 case (u32) ETH_SS_TEST:
  memcpy(data, *pch_gbe_gstrings_test,
	(int)sizeof(pch_gbe_gstrings_test));
  break;
 case (u32) ETH_SS_STATS:
  for (i = 0; i < PCH_GBE_GLOBAL_STATS_LEN; i++) {
   memcpy(p, pch_gbe_gstrings_stats[i].stat_string,
	ETH_GSTRING_LEN);
   p += ETH_GSTRING_LEN;
  }
  break;
 }
}

/*!
 * @ingroup Ethtool driver
 * @fn      static void pch_gbe_led_blink_callback(unsigned long data)
 * @brief   Callback function for blink led
 * @param   data [IN] Pointer Address of Board private structure
 * @return  None
 */
static void pch_gbe_led_blink_callback(unsigned long data)
{
 struct pch_gbe_adapter *adapter = (struct pch_gbe_adapter *)data;

 PCH_DEBUG("ethtool: pch_gbe_led_blink_callback\n");

 if ((test_and_change_bit(PCH_GBE_LED_ON, &adapter->led_status)) != 0)
  pch_gbe_hal_led_off(&adapter->hw);
 else
  pch_gbe_hal_led_on(&adapter->hw);
 mod_timer(&adapter->blink_timer, jiffies + PCH_GBE_ID_INTERVAL);
}

/*!
 * @ingroup Ethtool driver
 * @fn      static int pch_gbe_phys_id(struct net_device *netdev, u32 data)
 * @brief   Identify the device
 * @param   netdev [INOUT] Network interface device structure
 * @param   data   [IN] Sleep time[ms]
 * @return  PCH_GBE_SUCCESS:  Successfully
 * @return  Negative value:   Failed
 */
static int pch_gbe_phys_id(struct net_device *netdev, u32 data)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);

 PCH_DEBUG("ethtool: pch_gbe_phys_id\n");

 if (!data || data > (u32) (MAX_SCHEDULE_TIMEOUT / HZ))
  data = (u32) (MAX_SCHEDULE_TIMEOUT / HZ);
 if (!adapter->blink_timer.function) {
  init_timer(&adapter->blink_timer);
  adapter->blink_timer.function = pch_gbe_led_blink_callback;
  adapter->blink_timer.data = (unsigned long)adapter;
 }
 pch_gbe_hal_setup_led(&adapter->hw);
 mod_timer(&adapter->blink_timer, jiffies);
 msleep_interruptible(data * 1000);
 del_timer_sync(&adapter->blink_timer);

 pch_gbe_hal_led_off(&adapter->hw);
 clear_bit(PCH_GBE_LED_ON, &adapter->led_status);
 pch_gbe_hal_cleanup_led(&adapter->hw);
 return PCH_GBE_SUCCESS;
}

/*!
 * @ingroup Ethtool driver
 * @fn      static void pch_gbe_get_ethtool_stats(struct net_device *netdev,
 *                                   struct ethtool_stats *stats, u64 *data)
 * @brief   Return statistics about the device
 * @param   netdev [INOUT] Network interface device structure
 * @param   stats  [INOUT] Ethtool statue structure
 * @param   data   [OUT] Pointer of read status area
 * @return  None
 */
static void
pch_gbe_get_ethtool_stats(struct net_device *netdev,
     struct ethtool_stats *stats, u64 *data)
{
 struct pch_gbe_adapter *adapter = netdev_priv(netdev);
 int i;

 PCH_DEBUG("ethtool: pch_gbe_get_ethtool_stats\n");

 pch_gbe_update_stats(adapter);
 for (i = 0; i < PCH_GBE_GLOBAL_STATS_LEN; i++) {
  signed char *p =
   (signed char *)adapter +
   pch_gbe_gstrings_stats[i].stat_offset;
  data[i] =
   (pch_gbe_gstrings_stats[i].sizeof_stat ==
   (int)sizeof(u64)) ? *(u64 *) p:(*(u32 *) p);
 }
}

/*!
 * @ingroup Ethtool driver Layer
 * @struct  pch_gbe_ethtool_ops
 * @brief   Store the pointers of ethtool interfaces to kernel
 */
static struct ethtool_ops pch_gbe_ethtool_ops = {
 .get_settings = pch_gbe_get_settings,
 .set_settings = pch_gbe_set_settings,
 .get_drvinfo = pch_gbe_get_drvinfo,
 .get_regs_len = pch_gbe_get_regs_len,
 .get_regs = pch_gbe_get_regs,
 .get_wol = pch_gbe_get_wol,
 .set_wol = pch_gbe_set_wol,
 .get_msglevel = pch_gbe_get_msglevel,
 .set_msglevel = pch_gbe_set_msglevel,
 .nway_reset = pch_gbe_nway_reset,
 .get_link = ethtool_op_get_link,
#ifdef CONFIG_PCH_PHUB
 .get_eeprom_len = pch_gbe_get_eeprom_len,
 .get_eeprom = pch_gbe_get_eeprom,
 .set_eeprom = pch_gbe_set_eeprom,
#endif
 .get_ringparam = pch_gbe_get_ringparam,
 .set_ringparam = pch_gbe_set_ringparam,
 .get_pauseparam = pch_gbe_get_pauseparam,
 .set_pauseparam = pch_gbe_set_pauseparam,
 .get_rx_csum = pch_gbe_get_rx_csum,
 .set_rx_csum = pch_gbe_set_rx_csum,
 .get_tx_csum = pch_gbe_get_tx_csum,
 .set_tx_csum = pch_gbe_set_tx_csum,
 .self_test = pch_gbe_diag_test,
 .get_strings = pch_gbe_get_strings,
 .phys_id = pch_gbe_phys_id,
 .get_ethtool_stats = pch_gbe_get_ethtool_stats,
};

/*!
 * @ingroup Ethtool driver internal functions
 * @fn      void pch_gbe_set_ethtool_ops(struct net_device *netdev)
 * @brief   Set the Ethtool to network device data
 * @param   netdev [INOUT] Network interface device structure
 * @return  None
 */
void pch_gbe_set_ethtool_ops(struct net_device *netdev)
{
 PCH_DEBUG("ethtool: pch_gbe_set_ethtool_ops\n");

 SET_ETHTOOL_OPS(netdev, &pch_gbe_ethtool_ops);
}

#define PCH_GBE_REG_PATTERN_TEST(reg, mask, write) \
 do { \
  if (reg_pattern_test(adapter, data, \
   PCH_GBE_##reg, mask, write)) \
   return 1; \
 } while (0)

/*!
 * @ingroup Ethtool driver internal functions
 * @fn      static int pch_gbe_reg_test(struct pch_gbe_adapter *adapter, uint64_t *data)
 * @brief   Register test
 * @param   adapter  [IN] Board private structure
 * @param   data     [INOUT] Pointer to test result data
 * @return  PCH_GBE_SUCCESS
 */
static int pch_gbe_reg_test(struct pch_gbe_adapter *adapter, uint64_t *data)
{
 PCH_DEBUG("ethtool: pch_gbe_reg_test\n");

 pch_gbe_reset(adapter);
 PCH_GBE_REG_PATTERN_TEST(INT_EN, 0x11111F3F, 0x11111F3F);
 PCH_GBE_REG_PATTERN_TEST(MODE, 0xC2000000, 0xC2000000);
 PCH_GBE_REG_PATTERN_TEST(TCPIP_ACC, 0x0000000F, 0x0000000F);
 PCH_GBE_REG_PATTERN_TEST(EX_LIST, 0xFFFFFFFF, 0xFFFFFFFF);
 PCH_GBE_REG_PATTERN_TEST(PHY_INT_CTRL, 0x00010003, 0x00010003);
 PCH_GBE_REG_PATTERN_TEST(MAC_RX_EN, 0x00000001, 0x00000001);
 PCH_GBE_REG_PATTERN_TEST(RX_FCTRL, 0x80000000, 0x80000000);
 PCH_GBE_REG_PATTERN_TEST(PAUSE_REQ, 0x80000000, 0x80000000);
 PCH_GBE_REG_PATTERN_TEST(RX_MODE, 0xC000FE00, 0xC000FE00);
 PCH_GBE_REG_PATTERN_TEST(TX_MODE, 0xF800FE00, 0xF800FE00);
 PCH_GBE_REG_PATTERN_TEST(PAUSE_PKT1, 0xFFFFFFFF, 0xFFFFFFFF);
 PCH_GBE_REG_PATTERN_TEST(PAUSE_PKT2, 0xFFFFFFFF, 0xFFFFFFFF);
 PCH_GBE_REG_PATTERN_TEST(PAUSE_PKT3, 0xFFFFFFFF, 0xFFFFFFFF);
 PCH_GBE_REG_PATTERN_TEST(PAUSE_PKT4, 0xFFFFFFFF, 0xFFFFFFFF);
 PCH_GBE_REG_PATTERN_TEST(PAUSE_PKT5, 0xFFFFFFFF, 0xFFFFFFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR1A, 0xFFFFFFFF, 0xFFFFFFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR1B, 0x0000FFFF, 0x0000FFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR2A, 0xFFFFFFFF, 0xFFFFFFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR2B, 0x0000FFFF, 0x0000FFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR3A, 0xFFFFFFFF, 0xFFFFFFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR3B, 0x0000FFFF, 0x0000FFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR4A, 0xFFFFFFFF, 0xFFFFFFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR4B, 0x0000FFFF, 0x0000FFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR5A, 0xFFFFFFFF, 0xFFFFFFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR5B, 0x0000FFFF, 0x0000FFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR6A, 0xFFFFFFFF, 0xFFFFFFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR6B, 0x0000FFFF, 0x0000FFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR7A, 0xFFFFFFFF, 0xFFFFFFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR7B, 0x0000FFFF, 0x0000FFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR8A, 0xFFFFFFFF, 0xFFFFFFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR8B, 0x0000FFFF, 0x0000FFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR9A, 0xFFFFFFFF, 0xFFFFFFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR9B, 0x0000FFFF, 0x0000FFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR10A, 0xFFFFFFFF, 0xFFFFFFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR10B, 0x0000FFFF, 0x0000FFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR11A, 0xFFFFFFFF, 0xFFFFFFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR11B, 0x0000FFFF, 0x0000FFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR12A, 0xFFFFFFFF, 0xFFFFFFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR12B, 0x0000FFFF, 0x0000FFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR13A, 0xFFFFFFFF, 0xFFFFFFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR13B, 0x0000FFFF, 0x0000FFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR14A, 0xFFFFFFFF, 0xFFFFFFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR14B, 0x0000FFFF, 0x0000FFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR15A, 0xFFFFFFFF, 0xFFFFFFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR15B, 0x0000FFFF, 0x0000FFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR16A, 0xFFFFFFFF, 0xFFFFFFFF);
 PCH_GBE_REG_PATTERN_TEST(MAC_ADR16B, 0x0000FFFF, 0x0000FFFF);
 PCH_GBE_REG_PATTERN_TEST(ADDR_MASK, 0x0000FFFF, 0x0000FFFF);
 PCH_GBE_REG_PATTERN_TEST(RGMII_CTRL, 0x0000001F, 0x0000001F);
 PCH_GBE_REG_PATTERN_TEST(DMA_CTRL, 0x00000003, 0x00000003);
 PCH_GBE_REG_PATTERN_TEST(RX_DSC_BASE, 0xFFFFFFF0, 0xFFFFFFF0);
 PCH_GBE_REG_PATTERN_TEST(RX_DSC_SIZE, 0x0000FFF0, 0x0000FFF0);
 PCH_GBE_REG_PATTERN_TEST(RX_DSC_HW_P, 0xFFFFFFF0, 0xFFFFFFF0);
 PCH_GBE_REG_PATTERN_TEST(RX_DSC_SW_P, 0xFFFFFFF0, 0xFFFFFFF0);
 PCH_GBE_REG_PATTERN_TEST(TX_DSC_BASE, 0xFFFFFFF0, 0xFFFFFFF0);
 PCH_GBE_REG_PATTERN_TEST(TX_DSC_SIZE, 0x0000FFF0, 0x0000FFF0);
 PCH_GBE_REG_PATTERN_TEST(TX_DSC_HW_P, 0xFFFFFFF0, 0xFFFFFFF0);
 PCH_GBE_REG_PATTERN_TEST(TX_DSC_SW_P, 0xFFFFFFF0, 0xFFFFFFF0);
 PCH_GBE_REG_PATTERN_TEST(WOL_ST, 0x0000000F, 0x0000000F);
 PCH_GBE_REG_PATTERN_TEST(WOL_CTRL, 0x0001017F, 0x0001017F);
 PCH_GBE_REG_PATTERN_TEST(WOL_ADDR_MASK, 0x0000FFFF, 0x0000FFFF);

 *data = 0;
 return PCH_GBE_SUCCESS;
}

/*!
 * @ingroup Ethtool driver internal functions
 * @fn      static bool reg_pattern_test(struct pch_gbe_adapter *adapter,
 *                                       uint64_t *data,   int reg,
 *                                       uint32_t mask,    uint32_t write)
 * @brief   Register pattern test
 * @param   adapter  [IN] Board private structure
 * @param   data     [INOUT] Pointer to test result data
 * @param   reg      [INOUT] Register address
 * @param   mask     [INOUT] Mask pattern
 * @param   write    [INOUT] Write data
 * @return  true     : Successfully
 * @return  false    : Failed
 */
static bool
reg_pattern_test(struct pch_gbe_adapter *adapter,
   uint64_t *data, int reg, uint32_t mask, uint32_t write)
{
 static const uint32_t test[] = {
  0x5A5A5A5A, 0xA5A5A5A5, 0x00000000, 0xFFFFFFFF
 };
 uint8_t __iomem *address = adapter->hw.hw_addr + reg;
 uint32_t read;
 int i;

 for (i = 0; i < ARRAY_SIZE(test); i++) {
  writel(write & test[i], address);
  read = (readl(address) & mask);
  if (read != (write & test[i] & mask)) {
   DPRINTK(HW, ERR, "pattern test reg %04X failed: "
    "got 0x%08X expected 0x%08X\n",
    reg, read, (write & test[i] & mask));
   *data = reg;
   return true;
  }
 }
 return false;
}
