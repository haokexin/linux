/*!
 * @file pch_gbe_defines.h
 * @brief Linux PCH Gigabit Ethernet Driver defines macro header file
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
#ifndef _PCH_GBE_DEFINES_H_
#define _PCH_GBE_DEFINES_H_

#include "pch_gbe_pci_ids.h" /* Pci vender/device ID */

/* DEBUG OPTION */
/* #define DEBUG_TEST */
/* #define NVM_MAC_FIX *//* MAC: 00 21 97 77 65 13 */

#define PHY_RESET_REG_INIT

#ifdef DEBUG_TEST
#define PCH_GBE_NETIF_MSG_DEFAULT       0x7fff /* ALL Enable */
#else
#define PCH_GBE_NETIF_MSG_DEFAULT       0x0000 /* All Disable */
#endif
/*-- Kind of Messege --------------------------
 NETIF_MSG_DRV  = 0x0001,
 NETIF_MSG_PROBE  = 0x0002,
 NETIF_MSG_LINK  = 0x0004,
 NETIF_MSG_TIMER  = 0x0008,
 NETIF_MSG_IFDOWN = 0x0010,
 NETIF_MSG_IFUP  = 0x0020,
 NETIF_MSG_RX_ERR = 0x0040,
 NETIF_MSG_TX_ERR = 0x0080,
 NETIF_MSG_TX_QUEUED = 0x0100,
 NETIF_MSG_INTR  = 0x0200,
 NETIF_MSG_TX_DONE = 0x0400,
 NETIF_MSG_RX_STATUS = 0x0800,
 NETIF_MSG_PKTDATA = 0x1000,
 NETIF_MSG_HW  = 0x2000,
 NETIF_MSG_WOL  = 0x4000,
-----------------------------------------------*/

/* IF OPTION */
#define PCH_GBE_MAC_IFOP_RGMII
#define PCH_GBE_MAC_RGMII_CTRL_SETTING ( \
    PCH_GBE_CHIP_TYPE_INTERNAL | \
    PCH_GBE_RGMII_MODE_RGMII   | \
    PCH_GBE_CRS_SEL              \
    )

/* TX/RX descriptor defines */
#define PCH_GBE_DEFAULT_TXD                  256
#define PCH_GBE_MAX_TXD                     4096
#define PCH_GBE_MIN_TXD                        8
#define PCH_GBE_DEFAULT_RXD                  256
#define PCH_GBE_MAX_RXD                     4096
#define PCH_GBE_MIN_RXD                        8
/* Number of Transmit and Receive Descriptors must be a multiple of 8 */
#define PCH_GBE_TX_DESC_MULTIPLE               8
#define PCH_GBE_RX_DESC_MULTIPLE               8

/* Checksum Offload defines Enable/Disable */
#define PCH_GBE_DEFAULT_RX_CSUM             TRUE /* TRUEorFALSE */
#define PCH_GBE_DEFAULT_TX_CSUM             TRUE /* TRUEorFALSE */

/* Copybreak default */
#define PCH_GBE_COPYBREAK_DEFAULT       256
#define PCH_GBE_PCI_BAR                 1

/* Device Driver infomation */
#define DRV_NAME        "pch_gbe"
#define DRV_STRING      "PCH Network Driver"
#define DRV_EXT         "-NAPI"
#define DRV_VERSION     "0.91"DRV_EXT
#define DRV_DESCRIPTION \
 "OKI semiconductor sample Linux driver for PCH Gigabit ethernet"
#define DRV_COPYRIGHT   "Copyright(c) 2009 OKI semiconductor"
#define FIRM_VERSION    "N/A"

#define PCH_GBE_MAC_REGS_LEN    76
#define PCH_GBE_PHY_REGS_LEN    32
#define PCH_GBE_REGS_LEN        (PCH_GBE_MAC_REGS_LEN + PCH_GBE_PHY_REGS_LEN)

#define PCH_GBE_DMA_ALIGN           (32)
#define PCH_GBE_ETH_ALEN            6

/* Initialize the wake-on-LAN settings */
#define PCH_GBE_WL_INIT_SETTING    ( \
    PCH_GBE_WLC_BR  |\
    PCH_GBE_WLC_MLT |\
    PCH_GBE_WLC_IND |\
    PCH_GBE_WLC_MP   \
    )

/* This defines the bits that are set in the Interrupt Mask
 * Set/Read Register.  Each bit is documented below:
 *   o RXT0   = Receiver Timer Interrupt (ring 0)
 *   o TXDW   = Transmit Descriptor Written Back
 *   o RXDMT0 = Receive Descriptor Minimum Threshold hit (ring 0)
 *   o RXSEQ  = Receive Sequence Error
 *   o LSC    = Link Status Change
 */
#define PCH_GBE_INT_ENABLE_MASK ( \
 PCH_GBE_INT_RX_DMA_CMPLT |    \
 PCH_GBE_INT_RX_DSC_EMP   |    \
 PCH_GBE_INT_WOL_DET      |    \
 PCH_GBE_INT_TX_CMPLT          \
    )

/* Ethertype field values */
#define PCH_GBE_MAX_JUMBO_FRAME_SIZE    (10318)
#define PCH_GBE_FRAME_SIZE_2048         (2048)
#define PCH_GBE_FRAME_SIZE_4096         (4096)
#define PCH_GBE_FRAME_SIZE_8192         (8192)

/* watchdog time */
#define PCH_GBE_WATCHDOG_PERIOD        (1 * HZ)

#define PCH_GBE_TX_WEIGHT         64
#define PCH_GBE_RX_WEIGHT         64
#define PCH_GBE_RX_BUFFER_WRITE   16

#define DSC_INIT16  0xC000

/* MAC Address */
/* Number of high/low register pairs in the MAC_ADR. The MAC_ADR (MAC Address
 * Registers) holds the directed and multicast addresses that we monitor.
 * Technically, we have 16 spots.  However, we reserve one of these spots
 * (MAC_ADR[15]) for our directed address used by controllers with
 * manageability enabled, allowing us room for 15 multicast addresses.
 */
#define PCH_GBE_MAR_ENTRIES      16
#define PCH_GBE_SHORT_PKT        64

/* PHY param */
#define PCH_GBE_PHY_RESET_DELAY_US  10
/* NVM param */
#define PCH_GBE_NVM_WORD_SIZE       3 /* 16bit word size */

/* Error Codes */
#define PCH_GBE_SUCCESS                      0
#define PCH_GBE_ERR_NVM                      1
#define PCH_GBE_ERR_PHY                      2
#define PCH_GBE_ERR_CONFIG                   3
#define PCH_GBE_ERR_PARAM                    4
#define PCH_GBE_ERR_MAC_INIT                 5
#define PCH_GBE_ERR_PHY_TYPE                 6
#define PCH_GBE_ERR_RESET                    9
#define PCH_GBE_ERR_MASTER_REQUESTS_PENDING 10
#define PCH_GBE_ERR_HOST_INTERFACE_COMMAND  11
#define PCH_GBE_BLK_PHY_RESET               12
#define PCH_GBE_ERR_SWFW_SYNC               13
#define PCH_GBE_NOT_IMPLEMENTED             14

#define PHY_MAX_REG_ADDRESS   0x1F /* 5 bit address bus (0-0x1F) */
/* PHY 1000 MII Register/Bit Definitions */
/* PHY Registers defined by IEEE */
#define PHY_CONTROL           0x00  /* Control Register */
#define PHY_STATUS            0x01  /* Status Regiser */
#define PHY_ID1               0x02  /* Phy Id Register (word 1) */
#define PHY_ID2               0x03  /* Phy Id Register (word 2) */
#define PHY_AUTONEG_ADV       0x04  /* Autoneg Advertisement */
#define PHY_LP_ABILITY        0x05  /* Link Partner Ability (Base Page) */
#define PHY_AUTONEG_EXP       0x06  /* Autoneg Expansion Register */
#define PHY_NEXT_PAGE_TX      0x07  /* Next Page TX */
#define PHY_LP_NEXT_PAGE      0x08  /* Link Partner Next Page */
#define PHY_1000T_CTRL        0x09  /* 1000Base-T Control Register */
#define PHY_1000T_STATUS      0x0A  /* 1000Base-T Status Register */
#define PHY_EXT_STATUS        0x0F  /* Extended Status Register */
#define PHY_PHYSP_CONTROL     0x10  /* PHY Specific Control Register */
#define PHY_EXT_PHYSP_CONTROL 0x14  /* Extended PHY Specific Control Register */
#define PHY_LED_CONTROL       0x18  /* LED Control Register */
#define PHY_EXT_PHYSP_STATUS  0x1B  /* Extended PHY Specific Status Register */

/* PHY Control Register */
#define MII_CR_SPEED_SELECT_MSB 0x0040 /* bits 6,13: 10=1000, 01=100, 00=10 */
#define MII_CR_COLL_TEST_ENABLE 0x0080 /* Collision test enable */
#define MII_CR_FULL_DUPLEX      0x0100 /* FDX =1, half duplex =0 */
#define MII_CR_RESTART_AUTO_NEG 0x0200 /* Restart auto negotiation */
#define MII_CR_ISOLATE          0x0400 /* Isolate PHY from MII */
#define MII_CR_POWER_DOWN       0x0800 /* Power down */
#define MII_CR_AUTO_NEG_EN      0x1000 /* Auto Neg Enable */
#define MII_CR_SPEED_SELECT_LSB 0x2000 /* bits 6,13: 10=1000, 01=100, 00=10 */
#define MII_CR_LOOPBACK         0x4000 /* 0 = normal, 1 = loopback */
#define MII_CR_RESET            0x8000 /* 0 = normal, 1 = PHY reset */
#define MII_CR_SPEED_1000       0x0040
#define MII_CR_SPEED_100        0x2000
#define MII_CR_SPEED_10         0x0000

/* PHY Status Register */
#define MII_SR_EXTENDED_CAPS     0x0001 /* Extended register capabilities */
#define MII_SR_JABBER_DETECT     0x0002 /* Jabber Detected */
#define MII_SR_LINK_STATUS       0x0004 /* Link Status 1 = link */
#define MII_SR_AUTONEG_CAPS      0x0008 /* Auto Neg Capable */
#define MII_SR_REMOTE_FAULT      0x0010 /* Remote Fault Detect */
#define MII_SR_AUTONEG_COMPLETE  0x0020 /* Auto Neg Complete */
#define MII_SR_PREAMBLE_SUPPRESS 0x0040 /* Preamble may be suppressed */
#define MII_SR_EXTENDED_STATUS   0x0100 /* Ext. status info in Reg 0x0F */
#define MII_SR_100T2_HD_CAPS     0x0200 /* 100T2 Half Duplex Capable */
#define MII_SR_100T2_FD_CAPS     0x0400 /* 100T2 Full Duplex Capable */
#define MII_SR_10T_HD_CAPS       0x0800 /* 10T   Half Duplex Capable */
#define MII_SR_10T_FD_CAPS       0x1000 /* 10T   Full Duplex Capable */
#define MII_SR_100X_HD_CAPS      0x2000 /* 100X  Half Duplex Capable */
#define MII_SR_100X_FD_CAPS      0x4000 /* 100X  Full Duplex Capable */
#define MII_SR_100T4_CAPS        0x8000 /* 100T4 Capable */

/* Phy Id Register (word 2) */
#define PHY_REVISION_MASK        0x000F

/* Autoneg Advertisement Register */
#define NWAY_AR_SELECTOR_FIELD   0x0001 /* indicates IEEE 802.3 CSMA/CD */
#define NWAY_AR_10T_HD_CAPS      0x0020 /* 10T   Half Duplex Capable */
#define NWAY_AR_10T_FD_CAPS      0x0040 /* 10T   Full Duplex Capable */
#define NWAY_AR_100TX_HD_CAPS    0x0080 /* 100TX Half Duplex Capable */
#define NWAY_AR_100TX_FD_CAPS    0x0100 /* 100TX Full Duplex Capable */
#define NWAY_AR_100T4_CAPS       0x0200 /* 100T4 Capable */
#define NWAY_AR_PAUSE            0x0400 /* Pause operation desired */
#define NWAY_AR_ASM_DIR          0x0800 /* Asymmetric Pause Direction bit */
#define NWAY_AR_REMOTE_FAULT     0x2000 /* Remote Fault detected */
#define NWAY_AR_NEXT_PAGE        0x8000 /* Next Page ability supported */

/* Link Partner Ability Register (Base Page) */
#define NWAY_LPAR_SELECTOR_FIELD 0x0000 /* LP protocol selector field */
#define NWAY_LPAR_10T_HD_CAPS    0x0020 /* LP is 10T   Half Duplex Capable */
#define NWAY_LPAR_10T_FD_CAPS    0x0040 /* LP is 10T   Full Duplex Capable */
#define NWAY_LPAR_100TX_HD_CAPS  0x0080 /* LP is 100TX Half Duplex Capable */
#define NWAY_LPAR_100TX_FD_CAPS  0x0100 /* LP is 100TX Full Duplex Capable */
#define NWAY_LPAR_100T4_CAPS     0x0200 /* LP is 100T4 Capable */
#define NWAY_LPAR_PAUSE          0x0400 /* LP Pause operation desired */
#define NWAY_LPAR_ASM_DIR        0x0800 /* LP Asymmetric Pause Direction bit */
#define NWAY_LPAR_REMOTE_FAULT   0x2000 /* LP has detected Remote Fault */
#define NWAY_LPAR_ACKNOWLEDGE    0x4000 /* LP has rx'd link code word */
#define NWAY_LPAR_NEXT_PAGE      0x8000 /* Next Page ability supported */

/* Autoneg Expansion Register */
#define NWAY_ER_LP_NWAY_CAPS      0x0001 /* LP has Auto Neg Capability */
#define NWAY_ER_PAGE_RXD          0x0002 /* LP is 10T   Half Duplex Capable */
#define NWAY_ER_NEXT_PAGE_CAPS    0x0004 /* LP is 10T   Full Duplex Capable */
#define NWAY_ER_LP_NEXT_PAGE_CAPS 0x0008 /* LP is 100TX Half Duplex Capable */
#define NWAY_ER_PAR_DETECT_FAULT  0x0010 /* LP is 100TX Full Duplex Capable */

/* 1000BASE-T Control Register */
#define CR_1000T_ASYM_PAUSE      0x0080 /* Advertise asymmetric pause bit */
#define CR_1000T_HD_CAPS         0x0100 /* Advertise 1000T HD capability */
#define CR_1000T_FD_CAPS         0x0200 /* Advertise 1000T FD capability */
#define CR_1000T_REPEATER_DTE    0x0400 /* 1=Repeater/switch device port */
     /* 0=DTE device */
#define CR_1000T_MS_VALUE        0x0800 /* 1=Configure PHY as Master */
     /* 0=Configure PHY as Slave */
#define CR_1000T_MS_ENABLE       0x1000 /* 1=Master/Slave manual config value */
     /* 0=Automatic Master/Slave config */
#define CR_1000T_TEST_MODE_NORMAL 0x0000 /* Normal Operation */
#define CR_1000T_TEST_MODE_1      0x2000 /* Transmit Waveform test */
#define CR_1000T_TEST_MODE_2      0x4000 /* Master Transmit Jitter test */
#define CR_1000T_TEST_MODE_3      0x6000 /* Slave Transmit Jitter test */
#define CR_1000T_TEST_MODE_4      0x8000 /* Transmitter Distortion test */

/* 1000BASE-T Status Register */
#define SR_1000T_IDLE_ERROR_CNT   0x00FF /* Num idle errors since last read */
#define SR_1000T_ASYM_PAUSE_DIR   0x0100 /* LP asymmetric pause direction bit */
#define SR_1000T_LP_HD_CAPS       0x0400 /* LP is 1000T HD capable */
#define SR_1000T_LP_FD_CAPS       0x0800 /* LP is 1000T FD capable */
#define SR_1000T_REMOTE_RX_STATUS 0x1000 /* Remote receiver OK */
#define SR_1000T_LOCAL_RX_STATUS  0x2000 /* Local receiver OK */
#define SR_1000T_MS_CONFIG_RES    0x4000 /* 1=Local TX is Master, 0=Slave*/
#define SR_1000T_MS_CONFIG_FAULT  0x8000 /* Master/Slave config fault */

/* PHY Specific Control Register */
#define PHYSP_CTRL_ASSERT_CRS_TX  0x0800

/* LED Control Register */
#define PHY_LED_CTRL_ON           0x4103
#define PHY_LED_CTRL_OFF          0x4102
#define PHY_LED_CTRL_CLEANUP      0x4100

/* Extended PHY Specific Status Register */
#define HWCFG_MODE_GMII_COPPER    0x000F /* GMII to Copper */
#define HWCFG_MODE_RGMII_COPPER   0x000B /* RGMII/Modiffied MII to Copper */
#define HWCFG_MODE_GMII_FIBER     0x0007 /* GMII to Fiber */
#define HWCFG_MODE_RGMII_FIBER    0x0003 /* RGMII to Fiber */
#define HWCFG_MODE_GMII_SGMII     0x000E /* GMII to SGMII */
#define HWCFG_MODE_RGMII_SGMII    0x0006 /* RGMII to SGMII */
#define HWCFG_MODE_TBI_COPPER     0x000D /* TBI to Copper */
#define HWCFG_MODE_RTBI_COPPER    0x0009 /* RTBI to Copper */
#define HWCFG_MODE_MASK           0x000F

#define PHY_SPEED_10    10
#define PHY_SPEED_100   100
#define PHY_SPEED_1000  1000
#define PHY_HALF_DUPLEX 1
#define PHY_FULL_DUPLEX 2

#define PHY_ADVERTISE_10_HALF      0x0001
#define PHY_ADVERTISE_10_FULL      0x0002
#define PHY_ADVERTISE_100_HALF     0x0004
#define PHY_ADVERTISE_100_FULL     0x0008
#define PHY_ADVERTISE_1000_HALF    0x0010 /* Not used, just FYI */
#define PHY_ADVERTISE_1000_FULL    0x0020

/* 1000/H is not supported, nor spec-compliant. */
#define PCH_GBE_ALL_SPEED_DUPLEX (PHY_ADVERTISE_10_HALF  | \
      PHY_ADVERTISE_10_FULL  | \
      PHY_ADVERTISE_100_HALF | \
      PHY_ADVERTISE_100_FULL | \
      PHY_ADVERTISE_1000_FULL)
#define PCH_GBE_ALL_NOT_GIG  (PHY_ADVERTISE_10_HALF  | \
      PHY_ADVERTISE_10_FULL  | \
      PHY_ADVERTISE_100_HALF | \
      PHY_ADVERTISE_100_FULL)
#define PCH_GBE_ALL_100_SPEED  (PHY_ADVERTISE_100_HALF | \
      PHY_ADVERTISE_100_FULL)
#define PCH_GBE_ALL_10_SPEED  (PHY_ADVERTISE_10_HALF  | \
      PHY_ADVERTISE_10_FULL)
#define PCH_GBE_ALL_FULL_DUPLEX  (PHY_ADVERTISE_10_FULL  | \
      PHY_ADVERTISE_100_FULL | \
      PHY_ADVERTISE_1000_FULL)
#define PCH_GBE_ALL_HALF_DUPLEX  (PHY_ADVERTISE_10_HALF  | \
      PHY_ADVERTISE_100_HALF)

#define AUTONEG_ADVERTISE_SPEED_DEFAULT   PCH_GBE_ALL_SPEED_DUPLEX

#endif
