/*!
 * @file pch_gbe_regs.h
 * @brief Linux PCH Gigabit Ethernet Driver register macro header file
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
#ifndef _PCH_GBE_REGS_H_
#define _PCH_GBE_REGS_H_

/* MAC registers */
#define PCH_GBE_INT_ST  0x0000 /* Interrupt Status */
#define PCH_GBE_INT_EN  0x0004 /* Interrupt Enable */
#define PCH_GBE_MODE  0x0008 /* Mode */
#define PCH_GBE_RESET  0x000C /* Reset */
#define PCH_GBE_TCPIP_ACC 0x0010 /* TCP/IP Accelerator Control */
#define PCH_GBE_EX_LIST  0x0014 /* External List */
#define PCH_GBE_INT_ST_HOLD 0x0018 /* Interrupt Status Hold */
#define PCH_GBE_PHY_INT_CTRL 0x001C /* PHY Interrupt Control */
#define PCH_GBE_MAC_RX_EN 0x0020 /* MAC RX Enable */
#define PCH_GBE_RX_FCTRL 0x0024 /* RX Flow Control */
#define PCH_GBE_PAUSE_REQ 0x0028 /* Pause Packet Request */
#define PCH_GBE_RX_MODE  0x002C /* RX Mode */
#define PCH_GBE_TX_MODE  0x0030 /* TX Mode */
#define PCH_GBE_RX_FIFO_ST 0x0034 /* RX FIFO Status */
#define PCH_GBE_TX_FIFO_ST 0x0038 /* TX FIFO Status */
#define PCH_GBE_TX_FID  0x003C /* TX Frame ID */
#define PCH_GBE_TX_RESULT 0x0040 /* TX Result */
#define PCH_GBE_PAUSE_PKT1 0x0044 /* Pause Packet1 */
#define PCH_GBE_PAUSE_PKT2 0x0048 /* Pause Packet2 */
#define PCH_GBE_PAUSE_PKT3 0x004C /* Pause Packet3 */
#define PCH_GBE_PAUSE_PKT4 0x0050 /* Pause Packet4 */
#define PCH_GBE_PAUSE_PKT5 0x0054 /* Pause Packet5 */
#define PCH_GBE_MAC_ADR  0x0060 /* MAC Address */
#define PCH_GBE_MAC_ADR1A 0x0060 /* MAC Address 1A */
#define PCH_GBE_MAC_ADR1B 0x0064 /* MAC Address 1B */
#define PCH_GBE_MAC_ADR2A 0x0068 /* MAC Address 2A */
#define PCH_GBE_MAC_ADR2B 0x006C /* MAC Address 2B */
#define PCH_GBE_MAC_ADR3A 0x0070 /* MAC Address 3A */
#define PCH_GBE_MAC_ADR3B 0x0074 /* MAC Address 3B */
#define PCH_GBE_MAC_ADR4A 0x0078 /* MAC Address 4A */
#define PCH_GBE_MAC_ADR4B 0x007C /* MAC Address 4B */
#define PCH_GBE_MAC_ADR5A 0x0080 /* MAC Address 5A */
#define PCH_GBE_MAC_ADR5B 0x0084 /* MAC Address 5B */
#define PCH_GBE_MAC_ADR6A 0x0088 /* MAC Address 6A */
#define PCH_GBE_MAC_ADR6B 0x008C /* MAC Address 6B */
#define PCH_GBE_MAC_ADR7A 0x0090 /* MAC Address 7A */
#define PCH_GBE_MAC_ADR7B 0x0094 /* MAC Address 7B */
#define PCH_GBE_MAC_ADR8A 0x0098 /* MAC Address 8A */
#define PCH_GBE_MAC_ADR8B 0x009C /* MAC Address 8B */
#define PCH_GBE_MAC_ADR9A 0x00A0 /* MAC Address 9A */
#define PCH_GBE_MAC_ADR9B 0x00A4 /* MAC Address 9B */
#define PCH_GBE_MAC_ADR10A 0x00A8 /* MAC Address 10A */
#define PCH_GBE_MAC_ADR10B 0x00AC /* MAC Address 10B */
#define PCH_GBE_MAC_ADR11A 0x00B0 /* MAC Address 11A */
#define PCH_GBE_MAC_ADR11B 0x00B4 /* MAC Address 11B */
#define PCH_GBE_MAC_ADR12A 0x00B8 /* MAC Address 12A */
#define PCH_GBE_MAC_ADR12B 0x00BC /* MAC Address 12B */
#define PCH_GBE_MAC_ADR13A 0x00C0 /* MAC Address 13A */
#define PCH_GBE_MAC_ADR13B 0x00C4 /* MAC Address 13B */
#define PCH_GBE_MAC_ADR14A 0x00C8 /* MAC Address 14A */
#define PCH_GBE_MAC_ADR14B 0x0CC /* MAC Address 14B */
#define PCH_GBE_MAC_ADR15A 0x00D0 /* MAC Address 15A */
#define PCH_GBE_MAC_ADR15B 0x00D4 /* MAC Address 15B */
#define PCH_GBE_MAC_ADR16A 0x00D8 /* MAC Address 16A */
#define PCH_GBE_MAC_ADR16B 0x00DC /* MAC Address 16B */
#define PCH_GBE_ADDR_MASK 0x00E0 /* MAC Address Mask */
#define PCH_GBE_MIIM  0x00E4 /* MIIM  */
#define PCH_GBE_RGMII_ST 0x00EC /* RGMII Status */
#define PCH_GBE_RGMII_CTRL 0x00F0 /* RGMII Control */
#define PCH_GBE_DMA_CTRL 0x0100 /* DMA Control */
#define PCH_GBE_RX_DSC_BASE 0x0110 /* RX Descriptor Base Address */
#define PCH_GBE_RX_DSC_SIZE 0x0114 /* RX Descriptor Size */
#define PCH_GBE_RX_DSC_HW_P 0x0118 /* RX Descriptor Hard Pointer  */
#define PCH_GBE_RX_DSC_HW_P_HLD 0x011C /* RX Descriptor Hard Pointer Hold */
#define PCH_GBE_RX_DSC_SW_P 0x0120 /* RX Descriptor Soft Pointer  */
#define PCH_GBE_TX_DSC_BASE 0x0130 /* TX Descriptor Base Address */
#define PCH_GBE_TX_DSC_SIZE 0x0134 /* TX Descriptor Size */
#define PCH_GBE_TX_DSC_HW_P 0x0138 /* TX Descriptor Hard Pointer  */
#define PCH_GBE_TX_DSC_HW_P_HLD 0x013C /* TX Descriptor Hard Pointer Hold */
#define PCH_GBE_TX_DSC_SW_P 0x0140 /* TX Descriptor Soft Pointer  */
#define PCH_GBE_RX_DMA_ST 0x0150 /* RX DMA Status */
#define PCH_GBE_TX_DMA_ST 0x0154 /* TX DMA Status */
#define PCH_GBE_WOL_ST  0x0160 /* Wake On LAN Status */
#define PCH_GBE_WOL_CTRL 0x0164 /* Wake On LAN Control */
#define PCH_GBE_WOL_ADDR_MASK 0x0168 /* Wake On LAN Address Mask */


/* Definitions for MAC registers */

/* Interrupt Status */
/* Interrupt Status Hold */
/* Interrupt Enable */
#define PCH_GBE_INT_RX_DMA_CMPLT  0x00000001 /* Receive DMA Transfer Complete */
#define PCH_GBE_INT_RX_VALID      0x00000002 /* MAC Normal Receive Complete */
#define PCH_GBE_INT_RX_FRAME_ERR  0x00000004 /* Receive frame error */
#define PCH_GBE_INT_RX_FIFO_ERR   0x00000008 /* Receive FIFO Overflow */
#define PCH_GBE_INT_RX_DMA_ERR    0x00000010 /* Receive DMA Transfer Error */
#define PCH_GBE_INT_RX_DSC_EMP    0x00000020 /* Receive Descriptor Empty */
#define PCH_GBE_INT_TX_CMPLT      0x00000100 /* MAC Transmission Complete */
#define PCH_GBE_INT_TX_DMA_CMPLT  0x00000200 /* DMA Transfer Complete */
#define PCH_GBE_INT_TX_FIFO_ERR   0x00000400 /* Transmission FIFO underflow. */
#define PCH_GBE_INT_TX_DMA_ERR    0x00000800 /* Transmission DMA Error */
#define PCH_GBE_INT_PAUSE_CMPLT   0x00001000 /* Pause Transmission complete */
#define PCH_GBE_INT_MIIM_CMPLT    0x00010000 /* MIIM I/F Read completion */
#define PCH_GBE_INT_PHY_INT       0x00100000 /* Interruption from PHY */
#define PCH_GBE_INT_WOL_DET       0x01000000 /* Wake On LAN Event detection. */
#define PCH_GBE_INT_TCPIP_ERR     0x10000000 /* TCP/IP Accelerator Error */

/* Mode */
#define PCH_GBE_MODE_MII_ETHER      0x00000000  /* GIGA Ethernet Mode [MII] */
#define PCH_GBE_MODE_GMII_ETHER     0x80000000  /* GIGA Ethernet Mode [GMII] */
#define PCH_GBE_MODE_HALF_DUPLEX    0x00000000  /* Duplex Mode [half duplex] */
#define PCH_GBE_MODE_FULL_DUPLEX    0x40000000  /* Duplex Mode [full duplex] */
#define PCH_GBE_MODE_FR_BST         0x04000000  /* Frame bursting is done */

/* Reset */
#define PCH_GBE_ALL_RST         0x80000000  /* All reset */
#define PCH_GBE_TX_RST          0x40000000  /* TX MAC, TX FIFO, TX DMA reset */
#define PCH_GBE_RX_RST          0x04000000  /* RX MAC, RX FIFO, RX DMA reset */

/* TCP/IP Accelerator Control */
/* External List Enable */
#define PCH_GBE_EX_LIST_EN      0x00000008
/* RX TCP/IP accelerator Disabled (Padding Enable) */
#define PCH_GBE_RX_TCPIPACC_OFF 0x00000004
/* TX TCP/IP accelerator and Padding Enable */
#define PCH_GBE_TX_TCPIPACC_EN  0x00000002
/* RX TCP/IP accelerator and Padding Enable */
#define PCH_GBE_RX_TCPIPACC_EN  0x00000001

/* External List */
/* Interrupt Status Hold */
/* PHY Interrupt Control */

/* MAC RX Enable */
#define PCH_GBE_MRE_MAC_RX_EN   0x00000001      /* MAC Receive Enable */

/* RX Flow Control */
/* Pause packet and Transmission Pause are enabled */
#define PCH_GBE_FL_CTRL_EN      0x80000000

/* Pause Packet Request */
#define PCH_GBE_PS_PKT_RQ       0x80000000      /* Pause packet Request */

/* RX Mode */
/* Address Filtering Enable */
#define PCH_GBE_ADD_FIL_EN      0x80000000
/* Multicast Filtering Enable */
#define PCH_GBE_MLT_FIL_EN      0x40000000
/* Receive Almost Empty Threshold */
#define PCH_GBE_RH_ALM_EMP_4    0x00000000      /* 4 words */
#define PCH_GBE_RH_ALM_EMP_8    0x00004000      /* 8 words */
#define PCH_GBE_RH_ALM_EMP_16   0x00008000      /* 16 words */
#define PCH_GBE_RH_ALM_EMP_32   0x0000C000      /* 32 words */
/* Receive Almost Full Threshold */
#define PCH_GBE_RH_ALM_FULL_4   0x00000000      /* 4 words */
#define PCH_GBE_RH_ALM_FULL_8   0x00001000      /* 8 words */
#define PCH_GBE_RH_ALM_FULL_16  0x00002000      /* 16 words */
#define PCH_GBE_RH_ALM_FULL_32  0x00003000      /* 32 words */
/* RX FIFO Read Triger Threshold */
#define PCH_GBE_RH_RD_TRG_4     0x00000000      /* 4 words */
#define PCH_GBE_RH_RD_TRG_8     0x00000200      /* 8 words */
#define PCH_GBE_RH_RD_TRG_16    0x00000400      /* 16 words */
#define PCH_GBE_RH_RD_TRG_32    0x00000600      /* 32 words */
#define PCH_GBE_RH_RD_TRG_64    0x00000800      /* 64 words */
#define PCH_GBE_RH_RD_TRG_128   0x00000A00      /* 128 words */
#define PCH_GBE_RH_RD_TRG_256   0x00000C00      /* 256 words */
#define PCH_GBE_RH_RD_TRG_512   0x00000E00      /* 512 words */

 /* Receive Descriptor bit definitions */
 #define PCH_GBE_RXD_ACC_STAT_BCAST          0x00000400
 #define PCH_GBE_RXD_ACC_STAT_MCAST          0x00000200
 #define PCH_GBE_RXD_ACC_STAT_UCAST          0x00000100
 #define PCH_GBE_RXD_ACC_STAT_TCPIPOK        0x000000C0
 #define PCH_GBE_RXD_ACC_STAT_IPOK           0x00000080
 #define PCH_GBE_RXD_ACC_STAT_TCPOK          0x00000040
 #define PCH_GBE_RXD_ACC_STAT_IP6ERR         0x00000020
 #define PCH_GBE_RXD_ACC_STAT_OFLIST         0x00000010
 #define PCH_GBE_RXD_ACC_STAT_TYPEIP         0x00000008
 #define PCH_GBE_RXD_ACC_STAT_MACL           0x00000004
 #define PCH_GBE_RXD_ACC_STAT_PPPOE          0x00000002
 #define PCH_GBE_RXD_ACC_STAT_VTAGT          0x00000001
 #define PCH_GBE_RXD_GMAC_STAT_PAUSE         0x0200
 #define PCH_GBE_RXD_GMAC_STAT_MARBR         0x0100
 #define PCH_GBE_RXD_GMAC_STAT_MARMLT        0x0080
 #define PCH_GBE_RXD_GMAC_STAT_MARIND        0x0040
 #define PCH_GBE_RXD_GMAC_STAT_MARNOTMT      0x0020
 #define PCH_GBE_RXD_GMAC_STAT_TLONG         0x0010
 #define PCH_GBE_RXD_GMAC_STAT_TSHRT         0x0008
 #define PCH_GBE_RXD_GMAC_STAT_NOTOCTAL      0x0004
 #define PCH_GBE_RXD_GMAC_STAT_NBLERR        0x0002
 #define PCH_GBE_RXD_GMAC_STAT_CRCERR        0x0001

 /* Transmit Descriptor bit definitions */
 #define PCH_GBE_TXD_CTRL_TCPIP_ACC_OFF      0x0008
 #define PCH_GBE_TXD_CTRL_ITAG               0x0004
 #define PCH_GBE_TXD_CTRL_ICRC               0x0002
 #define PCH_GBE_TXD_CTRL_APAD               0x0001
 #define PCH_GBE_TXD_WORDS_SHIFT             2
 #define PCH_GBE_TXD_GMAC_STAT_CMPLT         0x2000
 #define PCH_GBE_TXD_GMAC_STAT_ABT           0x1000
 #define PCH_GBE_TXD_GMAC_STAT_EXCOL         0x0800
 #define PCH_GBE_TXD_GMAC_STAT_SNGCOL        0x0400
 #define PCH_GBE_TXD_GMAC_STAT_MLTCOL        0x0200
 #define PCH_GBE_TXD_GMAC_STAT_CRSER         0x0100
 #define PCH_GBE_TXD_GMAC_STAT_TLNG          0x0080
 #define PCH_GBE_TXD_GMAC_STAT_TSHRT         0x0040
 #define PCH_GBE_TXD_GMAC_STAT_LTCOL         0x0020
 #define PCH_GBE_TXD_GMAC_STAT_TFUNDFLW      0x0010
 #define PCH_GBE_TXD_GMAC_STAT_RTYCNT_MASK   0x000F

/* TX Mode */
#define PCH_GBE_TM_NO_RTRY     0x80000000 /* No Retransmission */
#define PCH_GBE_TM_LONG_PKT    0x40000000 /* Long Packt TX Enable */
#define PCH_GBE_TM_ST_AND_FD   0x20000000 /* Stare and Forward */
#define PCH_GBE_TM_SHORT_PKT   0x10000000 /* Short Packet TX Enable */
#define PCH_GBE_TM_LTCOL_RETX  0x08000000 /* Retransmission at Late Collision */
/* Frame Start Threshold */
#define PCH_GBE_TM_TH_TX_STRT_4    0x00000000    /* 4 words */
#define PCH_GBE_TM_TH_TX_STRT_8    0x00004000    /* 8 words */
#define PCH_GBE_TM_TH_TX_STRT_16   0x00008000    /* 16 words */
#define PCH_GBE_TM_TH_TX_STRT_32   0x0000C000    /* 32 words */
/* Transmit Almost Empty Threshold */
#define PCH_GBE_TM_TH_ALM_EMP_4    0x00000000    /* 4 words */
#define PCH_GBE_TM_TH_ALM_EMP_8    0x00000800    /* 8 words */
#define PCH_GBE_TM_TH_ALM_EMP_16   0x00001000    /* 16 words */
#define PCH_GBE_TM_TH_ALM_EMP_32   0x00001800    /* 32 words */
#define PCH_GBE_TM_TH_ALM_EMP_64   0x00002000    /* 64 words */
#define PCH_GBE_TM_TH_ALM_EMP_128  0x00002800    /* 128 words */
#define PCH_GBE_TM_TH_ALM_EMP_256  0x00003000    /* 256 words */
#define PCH_GBE_TM_TH_ALM_EMP_512  0x00003800    /* 512 words */
/* Transmit Almost Full Threshold */
#define PCH_GBE_TM_TH_ALM_FULL_4   0x00000000    /* 4 words */
#define PCH_GBE_TM_TH_ALM_FULL_8   0x00000200    /* 8 words */
#define PCH_GBE_TM_TH_ALM_FULL_16  0x00000400    /* 16 words */
#define PCH_GBE_TM_TH_ALM_FULL_32  0x00000600    /* 32 words */

/* RX FIFO Status */
/* RX FIFO is almost full. */
#define PCH_GBE_RF_ALM_FULL     0x80000000
/* RX FIFO is almost empty. */
#define PCH_GBE_RF_ALM_EMP      0x40000000
/* that the data within RX FIFO has become more than RH_RD_TRG. */
#define PCH_GBE_RF_RD_TRG       0x20000000
/* The word count of the data existing within RX FIFO. */
#define PCH_GBE_RF_STRWD        0x1FFE0000
/* that frame which is currently valid/enabled is stored in RX FIFO. */
#define PCH_GBE_RF_RCVING       0x00010000

/* TX FIFO Status */
/* TX Frame ID */
/* TX Result */
/* Pause Packet1-5 */
/* MAC Address 1(A/B)- 16*/
/* MAC Address Mask */
#define PCH_GBE_BUSY                0x80000000

/* MIIM  */
#define PCH_GBE_MIIM_OPER_WRITE     0x04000000
#define PCH_GBE_MIIM_OPER_READ      0x00000000
#define PCH_GBE_MIIM_OPER_READY     0x04000000
#define PCH_GBE_MIIM_PHY_ADDR_SHIFT 21
#define PCH_GBE_MIIM_REG_ADDR_SHIFT 16

/* RGMII Status */
#define PCH_GBE_LINK_UP             0x80000008
#define PCH_GBE_RXC_SPEED_MSK       0x00000006
#define PCH_GBE_RXC_SPEED_2_5M      0x00000000    /* 2.5MHz */
#define PCH_GBE_RXC_SPEED_25M       0x00000002    /* 25MHz  */
#define PCH_GBE_RXC_SPEED_125M      0x00000004    /* 100MHz */
#define PCH_GBE_DUPLEX_FULL         0x00000001

/* RGMII Control */
#define PCH_GBE_CRS_SEL             0x00000010
#define PCH_GBE_RGMII_RATE_125M     0x00000000
#define PCH_GBE_RGMII_RATE_25M      0x00000008
#define PCH_GBE_RGMII_RATE_2_5M     0x0000000C
#define PCH_GBE_RGMII_MODE_GMII     0x00000000
#define PCH_GBE_RGMII_MODE_RGMII    0x00000002
#define PCH_GBE_CHIP_TYPE_EXTERNAL  0x00000000
#define PCH_GBE_CHIP_TYPE_INTERNAL  0x00000001

/* DMA Control */
#define PCH_GBE_RX_DMA_EN           0x00000002   /* Enables Receive DMA */
#define PCH_GBE_TX_DMA_EN           0x00000001   /* Enables Transmission DMA */



/* RX Descriptor Base Address */
/* RX Descriptor Size */
/* RX Descriptor Hard Pointer  */
/* RX Descriptor Hard Pointer Hold */
/* RX Descriptor Soft Pointer  */
/* TX Descriptor Base Address */
/* TX Descriptor Size */
/* TX Descriptor Hard Pointer  */
/* TX Descriptor Hard Pointer Hold */
/* TX Descriptor Soft Pointer  */

/* RX DMA Status */
/* TX DMA Status */


/* Wake On LAN Status */
#define PCH_GBE_WLS_BR          0x00000008 /* Broadcas Address */
#define PCH_GBE_WLS_MLT         0x00000004 /* Multicast Address */
/* The Frame registered in Address Recognizer */
#define PCH_GBE_WLS_IND         0x00000002
#define PCH_GBE_WLS_MP          0x00000001 /* Magic packet Address */

/* Wake On LAN Control */
#define PCH_GBE_WLC_WOL_MODE    0x00010000
#define PCH_GBE_WLC_IGN_TLONG   0x00000100
#define PCH_GBE_WLC_IGN_TSHRT   0x00000080
#define PCH_GBE_WLC_IGN_OCTER   0x00000040
#define PCH_GBE_WLC_IGN_NBLER   0x00000020
#define PCH_GBE_WLC_IGN_CRCER   0x00000010
#define PCH_GBE_WLC_BR          0x00000008
#define PCH_GBE_WLC_MLT         0x00000004
#define PCH_GBE_WLC_IND         0x00000002
#define PCH_GBE_WLC_MP          0x00000001

/* Wake On LAN Address Mask */
#define PCH_GBE_WLA_BUSY        0x80000000

#endif
