/*!
 * @file pch_gbe_hw.h
 * @brief Linux PCH Gigabit Ethernet Driver Hardware layer header file
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
#ifndef _PCH_GBE_HW_H_
#define _PCH_GBE_HW_H_

struct pch_gbe_hw;

/* mac type values */
#define PCH_GBE_MAC_TYPE_UNDEFINED 0
#define PCH_GBE_MAC_TYPE_PCH1  1
#define PCH_GBE_MAC_TYPE_PCH2  2

/* bus type values */
#define pch_gbe_bus_type_unknown 0
#define pch_gbe_bus_type_pci  1
#define pch_gbe_bus_type_pcix  2
#define pch_gbe_bus_type_pci_express 3
#define pch_gbe_bus_type_reserved 4

/* bus speed values */
#define pch_gbe_bus_speed_unknown 0
#define pch_gbe_bus_speed_33  1
#define pch_gbe_bus_speed_66  2
#define pch_gbe_bus_speed_100  3
#define pch_gbe_bus_speed_120  4
#define pch_gbe_bus_speed_133  5
#define pch_gbe_bus_speed_2500  6
#define pch_gbe_bus_speed_reserved 7

/* bus width values */
#define pch_gbe_bus_width_unknown 0
#define pch_gbe_bus_width_pcie_x1 1
#define pch_gbe_bus_width_pcie_x2 2
#define pch_gbe_bus_width_pcie_x4 4
#define pch_gbe_bus_width_32  5
#define pch_gbe_bus_width_64  6
#define pch_gbe_bus_width_reserved 7

/* flow control values */
#define pch_gbe_fc_none   0
#define pch_gbe_fc_rx_pause  1
#define pch_gbe_fc_tx_pause  2
#define pch_gbe_fc_full   3

#define PCH_GBE_FC_DEFAULT  pch_gbe_fc_full

/*!
 * @ingroup Gigabit Ether driver Layer
 * @struct  pch_gbe_rx_desc
 * @brief   Receive Descriptor
 */
struct pch_gbe_rx_desc {
 u32 buffer_addr; /** RX Frame Buffer Address */
 u32 tcp_ip_status; /** TCP/IP Accelerator Status */
 u16 rx_words_eob; /** RX word count and Byte position */
 u16 gbec_status; /** GMAC Status */
 u8 dma_status;  /** DMA Status */
 u8 reserved1;  /** Reserved */
 u16 reserved2;  /** Reserved */
};

/*!
 * @ingroup Gigabit Ether driver Layer
 * @struct  pch_gbe_tx_desc
 * @brief   Transmit Descriptor
 */
struct pch_gbe_tx_desc {
 u32 buffer_addr; /** TX Frame Buffer Address */
 u16 length;  /** Data buffer length */
 u16 reserved1;  /** Reserved */
 u16 tx_words_eob; /** TX word count and Byte position */
 u16 tx_frame_ctrl; /** TX Frame Control */
 u8 dma_status;  /** DMA Status */
 u8 reserved2;  /** Reserved */
 u16 gbec_status; /** GMAC Status */
};

/*!
 * @ingroup Gigabit Ether driver Layer
 * @struct  pch_gbe_functions
 * @brief   HAL APi function pointer
 */
struct pch_gbe_functions {
 /* Function pointers for the MAC. */
 s32(*cleanup_led) (struct pch_gbe_hw *);
  /** for pch_gbe_hal_cleanup_led */

 void (*get_bus_info) (struct pch_gbe_hw *);
  /** for pch_gbe_hal_get_bus_info */

 s32(*led_on) (struct pch_gbe_hw *);
  /** for pch_gbe_hal_led_on */

 s32(*led_off) (struct pch_gbe_hw *);
  /** for pch_gbe_hal_led_off */

 void (*mc_addr_list_update) (struct pch_gbe_hw *, u8 *, u32, u32, u32);
  /** for pch_gbe_hal_mc_addr_list_update */

 void (*reset_hw) (struct pch_gbe_hw *);
  /** for pch_gbe_hal_reset_hw */

 s32(*init_hw) (struct pch_gbe_hw *);
  /** for pch_gbe_hal_init_hw */

 s32(*setup_link) (struct pch_gbe_hw *);
  /** for pch_gbe_hal_setup_link */

 s32(*setup_physical_interface) (struct pch_gbe_hw *);
  /** for setup link of PHY */

 s32(*setup_led) (struct pch_gbe_hw *);
  /** for pch_gbe_hal_setup_led */

 void (*pause_packet) (struct pch_gbe_hw *);
  /** for pch_gbe_hal_set_pause_packet */

 /* Function pointers for the PHY. */
 s32(*read_phy_reg) (struct pch_gbe_hw *, u32, u16 *);
  /** for pch_gbe_hal_read_phy_reg */

 s32(*write_phy_reg) (struct pch_gbe_hw *, u32, u16);
  /** for pch_gbe_hal_write_phy_reg */

 void (*reset_phy) (struct pch_gbe_hw *);
  /** for pch_gbe_hal_phy_hw_reset */

 void (*sw_reset_phy) (struct pch_gbe_hw *);
  /** for pch_gbe_hal_phy_sw_reset */

 void (*power_up_phy) (struct pch_gbe_hw *hw);
  /** for pch_gbe_hal_power_up_phy */

 void (*power_down_phy) (struct pch_gbe_hw *hw);
  /** for pch_gbe_hal_power_down_phy */
#ifdef CONFIG_PCH_PHUB
 /* Function pointers for the NVM. */
 s32(*validate_nvm) (struct pch_gbe_hw *);
  /** for pch_gbe_hal_validate_nvm_checksum */

 s32(*read_nvm) (struct pch_gbe_hw *, u32, u8 *);
  /** for pch_gbe_hal_read_nvm */

 s32(*write_nvm) (struct pch_gbe_hw *, u32, u8 *);
  /** for pch_gbe_hal_write_nvm */
#endif
 s32(*read_mac_addr) (struct pch_gbe_hw *);
  /** for pch_gbe_hal_read_mac_addr */

 u16(*ctrl_miim) (struct pch_gbe_hw *, u32, u32, u32, u16);
  /** for pch_gbe_hal_ctrl_miim */
};

/*!
 * @ingroup Gigabit Ether driver Layer
 * @struct  pch_gbe_mac_info
 * @brief   MAC infomation
 */
struct pch_gbe_mac_info {
 u8 addr[6];  /** Store the MAC address */
 u8 type;  /** Type of MAC */
 u8 fc;   /** Mode of flow control */
 u8 fc_autoneg;   /** Auto negotiation enable for flow control setting */
 u8 tx_fc_enable; /** Enable flag of Transmit flow control */
 u32 max_frame_size; /** Max transmit frame size */
 u32 min_frame_size; /** Min transmit frame size */
 u16 mar_entry_count; /** Entry count of MAC address registers */
 u8 autoneg; /** Auto negotiation enable */
 u16 link_speed;  /** Link speed */
 u16 link_duplex; /** Link duplex */
};

/*!
 * @ingroup Gigabit Ether driver Layer
 * @struct  pch_gbe_phy_info
 * @brief   PHY infomation
 */
struct pch_gbe_phy_info {
 u32 addr;  /** PHY address */
 u32 id;   /** PHY's identifier */
 u32 revision;  /** PHY's revision */
 u32 reset_delay_us; /** HW reset delay time[us] */
 u16 autoneg_advertised; /** Autoneg advertised */
};

/*!
 * @ingroup Gigabit Ether driver Layer
 * @struct  pch_gbe_nvm_info
 * @brief   NVM infomation
 */
struct pch_gbe_nvm_info {
 u16 word_size;
};

/*!
 * @ingroup Gigabit Ether driver Layer
 * @struct  pch_gbe_bus_info
 * @brief   Bus infomation
 */
struct pch_gbe_bus_info {
 u8 type;
 u8 speed;
 u8 width;
};

/*!
 * @ingroup Gigabit Ether driver Layer
 * @struct  pch_gbe_hw
 * @brief   Hardware infomation
 */
struct pch_gbe_hw {
 void *back;

 u8 *hw_addr;
 spinlock_t miim_lock;

 struct pch_gbe_functions func;
 struct pch_gbe_mac_info mac;
 struct pch_gbe_phy_info phy;
 struct pch_gbe_nvm_info nvm;
 struct pch_gbe_bus_info bus;

 u16 vendor_id;
 u16 device_id;
 u16 subsystem_vendor_id;
 u16 subsystem_device_id;
 u8 revision_id;
};

#endif
