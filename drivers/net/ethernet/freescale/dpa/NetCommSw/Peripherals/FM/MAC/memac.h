/*
 * Copyright 2008-2012 Freescale Semiconductor Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/******************************************************************************
 @File          memac.h

 @Description   FM Multirate Ethernet MAC (mEMAC)
*//***************************************************************************/
#ifndef __MEMAC_H
#define __MEMAC_H

#include "std_ext.h"
#include "error_ext.h"
#include "list_ext.h"
#include "memac_mii_acc.h"
#include "fm_mac.h"


/* Interrupt Mask Register (IMASK) */
#define IMASK_PCS                   0x80000000  /* 0 PCS
                                                 * XGMII - PCS event
                                                 * GMII - link synchronization event
                                                 */
#define IMASK_AN                    0x40000000  /* 1 AN
                                                 * XGMII - Auto-negotiation event
                                                 * GMII - Auto-negotiation status
                                                 */
#define IMASK_LT                    0x20000000  /* 2 LT
                                                 * XGMII - Link Training event
                                                 * GMII - new page received by auto-negotiation function
                                                 */
#define IMASK_MGI                   0x00004000  /* 17 Magic packet detection indication event */
#define IMASK_RX_FIFO_OVFL          0x00001000  /* 19 Receive FIFO overflow event */
#define IMASK_TX_FIFO_UNFL          0x00000800  /* 20 Transmit FIFO underflow event */
#define IMASK_TX_FIFO_OVFL          0x00000400  /* 21 Transmit FIFO overflow event */
#define IMASK_TX_ECC_ER             0x00000200  /* 22 Transmit frame ECC error event */
#define IMASK_RX_ECC_ER             0x00000100  /* 23 Receive frame ECC error event */
#define IMASK_LI_FAULT              0x00000080  /* 24 Link Interruption fault event (XGMII) */
#define IMASK_RX_EMPTY              0x00000040  /* 25 Receive FIFO empty event */
#define IMASK_TX_EMPTY              0x00000020  /* 26 Transmit FIFO empty event */
#define IMASK_RX_LOWP               0x00000010  /* 27 Low Power Idle event */
#define IMASK_PHY_LOS               0x00000004  /* 29 Phy loss of signal event */
#define IMASK_REM_FAULT             0x00000002  /* 30 Remote fault event (XGMII) */
#define IMASK_LOC_FAULT             0x00000001  /* 31 Local fault event (XGMII) */

#define EVENTS_MASK                 ((uint32_t)(IMASK_PCS           |  \
                                                IMASK_AN            |  \
                                                IMASK_LT            |  \
                                                IMASK_MGI           |  \
                                                IMASK_RX_FIFO_OVFL  |  \
                                                IMASK_TX_FIFO_UNFL  |  \
                                                IMASK_TX_FIFO_OVFL  |  \
                                                IMASK_TX_ECC_ER     |  \
                                                IMASK_RX_ECC_ER     |  \
                                                IMASK_LI_FAULT      |  \
                                                IMASK_RX_EMPTY      |  \
                                                IMASK_TX_EMPTY      |  \
                                                IMASK_RX_LOWP       |  \
                                                IMASK_PHY_LOS       |  \
                                                IMASK_REM_FAULT     |  \
                                                IMASK_LOC_FAULT))

#define GET_EXCEPTION_FLAG(bitMask, exception) \
    (bitMask = EVENTS_MASK)


#define DEFAULT_pauseForwardEnable          FALSE
#define DEFAULT_txAddrInsEnable             FALSE
#define DEFAULT_cmdFrameEnable              FALSE
#define DEFAULT_rxErrorDiscard              FALSE
#define DEFAULT_phyTxenaOn                  FALSE
#define DEFAULT_sendIdleEnable              FALSE
#define DEFAULT_lgthCheckNostdr             FALSE
#define DEFAULT_rxSfdAny                    FALSE
#define DEFAULT_rxPblFwd                    FALSE
#define DEFAULT_txPblFwd                    FALSE
#define DEFAULT_txIpgLength                 12
#define DEFAULT_debugMode                   FALSE
#define DEFAULT_timeStampEnable             FALSE

#define DEFAULT_exceptions          ((uint32_t)(IMASK_RX_FIFO_OVFL  |  \
                                                IMASK_TX_FIFO_UNFL  |  \
                                                IMASK_TX_FIFO_OVFL  |  \
                                                IMASK_TX_ECC_ER     |  \
                                                IMASK_RX_ECC_ER     |  \
                                                IMASK_RX_EMPTY      |  \
                                                IMASK_TX_EMPTY      |  \
                                                IMASK_REM_FAULT     |  \
                                                IMASK_LOC_FAULT))


/* Control and Configuration Register (COMMAND_CONFIG) */
#define CMD_CFG_MG                  0x80000000  /* 00 Magic Packet detection */
#define CMD_CFG_REG_LOWP_RXETY      0x01000000  /* 07 Rx low power indication */
#define CMD_CFG_TX_LOWP_ENA         0x00800000  /* 08 Transmit Low Power Idle Enable */
#define CMD_CFG_SFD_ANY             0x00200000  /* 10 Disable check of SFD (0xd5) character at frame start */
#define CMD_CFG_PFC_MODE            0x00080000  /* 12 Enable Priority Flow Control (PFC) mode of operation */
#define CMD_CFG_NO_LEN_CHK          0x00020000  /* 14 Payload length check disable */
#define CMD_CFG_SEND_IDLE           0x00010000  /* 15 Force idle generation */
#define CMD_CFG_CNT_FRM_EN          0x00002000  /* 18 Control frame reception enable */
#define CMD_CFG_SW_RESET            0x00001000  /* 19 Software Reset, self clearing bit */
#define CMD_CFG_TX_PAD_EN           0x00000800  /* 20 Enable padding of frames in transmit direction */
#define CMD_CFG_LOOPBACK_EN         0x00000400  /* 21 XGMII/GMII loopback enable */
#define CMD_CFG_TX_ADDR_INS         0x00000200  /* 22 Transmit source MAC address insertion */
#define CMD_CFG_PAUSE_IGNORE        0x00000100  /* 23 Ignore Pause frame quanta */
#define CMD_CFG_PAUSE_FWD           0x00000080  /* 24 Terminate/forward received Pause frames */
#define CMD_CFG_CRC_FWD             0x00000040  /* 25 Terminate/forward CRC of received frames */
#define CMD_CFG_PAD_EN              0x00000020  /* 26 Frame padding removal in receive path enable */
#define CMD_CFG_PROMIS_EN           0x00000010  /* 27 Promiscuous operation enable */
#define CMD_CFG_WAN_MODE            0x00000008  /* 28 WAN mode enable */
#define CMD_CFG_RX_EN               0x00000002  /* 30 MAC receive path enable */
#define CMD_CFG_TX_EN               0x00000001  /* 31 MAC transmit path enable */

/* Interface Mode Register (IF_MODE) */
#define IF_MODE_MASK                0x00000003  /* 30-31 Mask on interface mode bits */
#define IF_MODE_XGMII               0x00000000  /* 30-31 XGMII (10G) interface */
#define IF_MODE_GMII                0x00000002  /* 30-31 GMII (1G) interface */

/* Hash table Control Register (HASHTABLE_CTRL) */
#define HASH_CTRL_MCAST_SHIFT       26
#define HASH_CTRL_MCAST_EN          0x00000100  /* 23 Multicast frame reception for the hash entry */
#define HASH_CTRL_ADDR_MASK         0x0000003F  /* 26-31 Hash table address code */

#define GROUP_ADDRESS               0x0000010000000000LL /* MAC multicast address bit indication */
#define HASH_TABLE_SIZE             64          /* Hash table size (bits 26-31 in HASHTABLE_CTRL allows for 2^6 entries) */

/* Transmit Inter-Packet Gap Length Register (TX_IPG_LENGTH) */
#define TX_IPG_LENGTH_MASK          0x0000003F

/* Statistics Configuration Register (STATN_CONFIG) */
#define STATS_CFG_CLR               0x00000004  /* 29 Reset all counters */
#define STATS_CFG_CLR_ON_RD         0x00000002  /* 30 Clear on read */
#define STATS_CFG_SATURATE          0x00000001  /* 31 Saturate at the maximum value */

/* Internal PHY access */
#define PHY_MDIO_ADDR               0
/* SGMII registers */
#define PHY_SGMII_CR_PHY_RESET      0x8000
#define PHY_SGMII_CR_RESET_AN       0x0200
#define PHY_SGMII_CR_DEF_VAL        0x1140
#define PHY_SGMII_DEV_ABILITY_SGMII 0x4001
#define PHY_SGMII_IF_MODE_AN        0x0002
#define PHY_SGMII_IF_MODE_SGMII     0x0001


#define MEMAC_NUM_OF_PADDRS         7           /* Number of additional exact match MAC address registers */
typedef _Packed struct t_MacAddr
{
    volatile uint32_t   mac_addr_l;         /* Lower 32 bits of 48-bit MAC address */
    volatile uint32_t   mac_addr_u;         /* Upper 16 bits of 48-bit MAC address */
} t_MacAddr;

#if defined(__MWERKS__) && !defined(__GNUC__)
#pragma pack(push,1)
#endif /* defined(__MWERKS__) && ... */


typedef _Packed struct t_MemacMemMap
{
    /* General Control and Status */
    volatile uint32_t   reserved1[2];
    volatile uint32_t   command_config;     /* 0x008 COMMAND_CONFIG - Control and configuration register */
    t_MacAddr mac_addr0;                    /* 0x00C-0x010 MAC_ADDR_0...MAC_ADDR_1 */
    volatile uint32_t   maxfrm;             /* 0x014 MAXFRM - Maximum frame length register */
    volatile uint32_t   reserved2[5];
    volatile uint32_t   hashtable_ctrl;     /* 0x02C HASHTABLE_CTRL - Hash table control register */
    volatile uint32_t   reserved3[4];
    volatile uint32_t   ievent;             /* 0x040 IEVENT - Interrupt event register */
    volatile uint32_t   tx_ipg_length;      /* 0x044 TX_IPG_LENGTH - Transmitter inter-packet-gap register */
    volatile uint32_t   reserved4;
    volatile uint32_t   imask;              /* 0x04C IMASK - Interrupt mask register */
    volatile uint32_t   reserved5;
    volatile uint32_t   pause_quanta[4];    /* 0x054 CL0x_PAUSE_QUANTA - CL0-7 Pause quanta register */
    volatile uint32_t   pause_thresh[4];    /* 0x064 CL0x_PAUSE_THRESH - CL0-7 Pause quanta threshold register */
    volatile uint32_t   rx_pause_status;    /* 0x074 RX_PAUSE_STATUS - Receive pause status register */
    volatile uint32_t   reserved6[2];
    t_MacAddr mac_addr[MEMAC_NUM_OF_PADDRS]; /* 0x80-0x0B4 MAC_ADDR_2...MAC_ADDR_15 */
    volatile uint32_t   lpwake_timer;       /* 0x0B8 LPWAKE_TIMER - EEE Low Power Wakeup Timer register */
    volatile uint32_t   sleep_timer;        /* 0x0BC SLEEP_TIMER - Transmit EEE Low Power Timer register */
    volatile uint32_t   reserved7[8];
    volatile uint32_t   statn_config;       /* 0x0E0 STATN_CONFIG - Statistics configuration register */
    volatile uint32_t   reserved8[7];

    /* Rx Statistics Counter */
    volatile uint32_t   reoct_l;
    volatile uint32_t   reoct_u;
    volatile uint32_t   roct_l;
    volatile uint32_t   roct_u;
    volatile uint32_t   raln_l;
    volatile uint32_t   raln_u;
    volatile uint32_t   rxpf_l;
    volatile uint32_t   rxpf_u;
    volatile uint32_t   rfrm_l;
    volatile uint32_t   rfrm_u;
    volatile uint32_t   rfcs_l;
    volatile uint32_t   rfcs_u;
    volatile uint32_t   rvlan_l;
    volatile uint32_t   rvlan_u;
    volatile uint32_t   rerr_l;
    volatile uint32_t   rerr_u;
    volatile uint32_t   ruca_l;
    volatile uint32_t   ruca_u;
    volatile uint32_t   rmca_l;
    volatile uint32_t   rmca_u;
    volatile uint32_t   rbca_l;
    volatile uint32_t   rbca_u;
    volatile uint32_t   rdrp_l;
    volatile uint32_t   rdrp_u;
    volatile uint32_t   rpkt_l;
    volatile uint32_t   rpkt_u;
    volatile uint32_t   rund_l;
    volatile uint32_t   rund_u;
    volatile uint32_t   r64_l;
    volatile uint32_t   r64_u;
    volatile uint32_t   r127_l;
    volatile uint32_t   r127_u;
    volatile uint32_t   r255_l;
    volatile uint32_t   r255_u;
    volatile uint32_t   r511_l;
    volatile uint32_t   r511_u;
    volatile uint32_t   r1023_l;
    volatile uint32_t   r1023_u;
    volatile uint32_t   r1518_l;
    volatile uint32_t   r1518_u;
    volatile uint32_t   r1519x_l;
    volatile uint32_t   r1519x_u;
    volatile uint32_t   rovr_l;
    volatile uint32_t   rovr_u;
    volatile uint32_t   rjbr_l;
    volatile uint32_t   rjbr_u;
    volatile uint32_t   rfrg_l;
    volatile uint32_t   rfrg_u;
    volatile uint32_t   rcnp_l;
    volatile uint32_t   rcnp_u;
    volatile uint32_t   rdrntp_l;
    volatile uint32_t   rdrntp_u;
    volatile uint32_t   reserved9[12];

    /* Tx Statistics Counter */
    volatile uint32_t   teoct_l;
    volatile uint32_t   teoct_u;
    volatile uint32_t   toct_l;
    volatile uint32_t   toct_u;
    volatile uint32_t   reserved10[2];
    volatile uint32_t   txpf_l;
    volatile uint32_t   txpf_u;
    volatile uint32_t   tfrm_l;
    volatile uint32_t   tfrm_u;
    volatile uint32_t   tfcs_l;
    volatile uint32_t   tfcs_u;
    volatile uint32_t   tvlan_l;
    volatile uint32_t   tvlan_u;
    volatile uint32_t   terr_l;
    volatile uint32_t   terr_u;
    volatile uint32_t   tuca_l;
    volatile uint32_t   tuca_u;
    volatile uint32_t   tmca_l;
    volatile uint32_t   tmca_u;
    volatile uint32_t   tbca_l;
    volatile uint32_t   tbca_u;
    volatile uint32_t   reserved11[2];
    volatile uint32_t   tpkt_l;
    volatile uint32_t   tpkt_u;
    volatile uint32_t   tund_l;
    volatile uint32_t   tund_u;
    volatile uint32_t   t64_l;
    volatile uint32_t   t64_u;
    volatile uint32_t   t127_l;
    volatile uint32_t   t127_u;
    volatile uint32_t   t255_l;
    volatile uint32_t   t255_u;
    volatile uint32_t   t511_l;
    volatile uint32_t   t511_u;
    volatile uint32_t   t1023_l;
    volatile uint32_t   t1023_u;
    volatile uint32_t   t1518_l;
    volatile uint32_t   t1518_u;
    volatile uint32_t   t1519x_l;
    volatile uint32_t   t1519x_u;
    volatile uint32_t   reserved12[6];
    volatile uint32_t   tcnp_l;
    volatile uint32_t   tcnp_u;
    volatile uint32_t   reserved13[14];

    /* Line Interface Control */
    volatile uint32_t   if_mode;            /* 0x300 IF_MODE - Interface Mode Control register */
    volatile uint32_t   if_status;          /* 0x304 IF_STATUS - Interface Status register */
    volatile uint32_t   reserved14[14];

    /* HiGig/2 */
    volatile uint32_t   hg_config;          /* 0x340 HG_CONFIG - HiGig/2 Control and configuration register */
    volatile uint32_t   reserved15[3];
    volatile uint32_t   hg_pause_quanta;    /* 0x350 HG_PAUSE_QUANTA - HiGig2 Pause quanta register */
    volatile uint32_t   reserved16[3];
    volatile uint32_t   hg_pause_thresh;    /* 0x360 HG_PAUSE_THRESH - HiGig2 Pause quanta threshold register */
    volatile uint32_t   reserved17[3];
    volatile uint32_t   hgrx_pause_status;  /* 0x370 HGRX_PAUSE_STATUS - HiGig2 Receive pause status register*/
    volatile uint32_t   hg_fifos_status;    /* 0x374 HG_FIFOS_STATUS - HiGig2 fifos status register */
    volatile uint32_t   rhm;                /* 0x378 RHM - Receive HiGig2 messages counter register */
    volatile uint32_t   thm;                /* 0x37C THM - Transmit HiGig2 messages counter register */
} _PackedType t_MemacMemMap;

#if defined(__MWERKS__) && !defined(__GNUC__)
#pragma pack(pop)
#endif /* defined(__MWERKS__) && ... */


typedef struct t_MemacDriverParam
{
    bool resetOnInit;
    bool wanModeEnable;             /* WAN Mode Enable. Sets WAN mode (1) or LAN mode (0, default) of operation. */
    bool promiscuousModeEnable;     /* Enables MAC promiscuous operation. When set to '1', all frames are received without any MAC address filtering, when set to '0' (Reset value) Unicast Frames with a destination address not matching the Core MAC Address (MAC Address programmed in Registers MAC_ADDR_0 and MAC_ADDR_1 or the MAC address programmed in Registers MAC_ADDR_2 and MAC_ADDR_3 ) are rejected. */
    bool pauseForwardEnable;        /* Terminate / Forward Pause Frames. If set to '1' pause frames are forwarded to the user application. When set to '0' (Reset value) pause frames are terminated and discarded within the MAC. */
    bool pauseIgnore;               /* Ignore Pause Frame Quanta. If set to '1' received pause frames are ignored by the MAC. When set to '0' (Reset value) the transmit process is stopped for the amount of time specified in the pause quanta received within a pause frame. */
    bool txAddrInsEnable;           /* Set Source MAC Address on Transmit.
                                        If set to '1' the MAC overwrites the source MAC address received from the Client Interface with one of the MAC addresses (Refer to section 10.4)
                                        If set to '0' (Reset value), the source MAC address from the Client Interface is transmitted unmodified to the line. */
    bool loopbackEnable;            /* PHY Interface Loopback. When set to '1', the signal loop_ena is set to '1', when set to '0' (Reset value) the signal loop_ena is set to '0'. */
    bool cmdFrameEnable;            /* Enables reception of all command frames. When set to '1' all Command Frames are accepted, when set to '0' (Reset Value) only Pause Frames are accepted and all other Command Frames are rejected. */
    bool rxErrorDiscard;            /* Receive Errored Frame Discard Enable. When set to ‘1’, any frame received with an error is discarded in the Core and not forwarded to the Client interface. When set to ‘0’ (Reset value), errored Frames are forwarded to the Client interface with ff_rx_err asserted. */
    bool phyTxenaOn;                /* PHY Transmit Enable. When set to '1', the signal phy_txena is set to '1', when set to '0' (Reset value) the signal phy_txena is set to '0' */
    bool sendIdleEnable;            /* Force Idle Generation. When set to '1', the MAC permanently sends XGMII Idle sequences even when faults are received. */
    bool noLengthCheckEnable;       /* Payload Length Check Disable. When set to ‘0’ (Reset value), the Core checks the frame's payload length with the Frame Length/Type field, when set to ‘1’, the payload length check is disabled. */
    bool lgthCheckNostdr;           /* The Core interprets the Length/Type field differently depending on the value of this Bit */
    bool timeStampEnable;           /* This bit selects between enabling and disabling the IEEE 1588 functionality.
                                        1: IEEE 1588 is enabled.
                                        0: IEEE 1588 is disabled. */
    bool padAndCrcEnable;
    bool rxSfdAny;                  /* Enables, when set, that any character is allowed at the SFD position of the preamble and the frame will be accepted.
                                        If cleared (default) the frame is accepted only if the 8th byte of the preamble contains the SFD value 0xd5. If another value is received, the frame is discarded and the alignment error counter increments. */
    bool rxPblFwd;                  /* Receive Preamble Forwarding (custom preamble).
                                        If set, the first word (ff_rx_sop) of every received frame contains the preamble of the frame. The frame data starts with the 2nd word from the FIFO.
                                        If the bit is cleared (default) the preamble is removed from the frame before it is written into the receive FIFO. */
    bool txPblFwd;                  /* Transmit Preamble Forwarding (custom preamble).
                                        If set, the first word written into the TX FIFO is considered as frame preamble. The MAC will not add a preamble in front of the frame. Note that bits 7:0 of the preamble word will still be overwritten with the XGMII start character upon transmission.
                                        If cleared (default) the MAC */
    uint32_t txIpgLength;           /*Transmit Inter-Packet-Gap (IPG) value.
                                      A 6-bit value: Depending on LAN or WAN mode of operation (see COMMAND_CONFIG, 19.2.1 page 91) the value has the following meaning:
                                        - LAN Mode: Number of octets in steps of 4. Valid values are 8, 12, 16, ... 100. DIC is fully supported (see 10.6.1 page 49) for any setting. A default of 12 (reset value) must be set to conform to IEEE802.3ae. Warning: When set to 8, PCS layers may not be able to perform clock rate compensation.
                                        - WAN Mode: Stretch factor. Valid values are 4..15. The stretch factor is calculated as (value+1)*8. A default of 12 (reset value) must be set to conform to IEEE 802.3ae (i.e. 13*8=104). A larger value shrinks the IPG (increasing bandwidth). */
/*.. */
    uint16_t    maxFrameLength;
    bool        debugMode;
    uint16_t    pauseTime;
#ifdef FM_TX_ECC_FRMS_ERRATA_10GMAC_A004
    bool        skipFman11Workaround;
#endif /* FM_TX_ECC_FRMS_ERRATA_10GMAC_A004 */
} t_MemacDriverParam;

typedef struct t_Memac
{
    t_FmMacControllerDriver     fmMacControllerDriver;               /**< Upper Mac control block */
    t_Handle                    h_App;                               /**< Handle to the upper layer application  */
    t_MemacMemMap               *p_MemMap;                           /**< Pointer to MAC memory mapped registers */
    t_MemacMiiAccessMemMap      *p_MiiMemMap;                        /**< Pointer to MII memory mapped registers */
    uint64_t                    addr;                                /**< MAC address of device */
    e_EnetMode                  enetMode;                            /**< Ethernet physical interface  */
    t_FmMacExceptionCallback    *f_Exception;
    int                         mdioIrq;
    t_FmMacExceptionCallback    *f_Event;
    bool                        indAddrRegUsed[MEMAC_NUM_OF_PADDRS]; /**< Whether a particular individual address recognition register is being used */
    uint64_t                    paddr[MEMAC_NUM_OF_PADDRS];          /**< MAC address for particular individual address recognition register */
    uint8_t                     numOfIndAddrInRegs;                  /**< Number of individual addresses in registers for this station. */
    t_EthHash                   *p_MulticastAddrHash;                /**< Pointer to driver's global address hash table  */
    t_EthHash                   *p_UnicastAddrHash;                  /**< Pointer to driver's individual address hash table  */
    bool                        debugMode;
    uint8_t                     macId;
    uint32_t                    exceptions;
    t_MemacDriverParam          *p_MemacDriverParam;
} t_Memac;

#define MEMAC_TO_MII_OFFSET         0x030       /* Offset from the MEM map to the MDIO mem map */

t_Error MEMAC_MII_WritePhyReg(t_Handle h_Memac, uint8_t phyAddr, uint8_t reg, uint16_t data);
t_Error MEMAC_MII_ReadPhyReg(t_Handle h_Memac,  uint8_t phyAddr, uint8_t reg, uint16_t *p_Data);


#endif /* __MEMAC_H */
