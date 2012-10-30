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


#ifndef __FSL_FMAN_MEMAC_H
#define __FSL_FMAN_MEMAC_H

#include "common/general.h"
#include "fsl_enet.h"


#define MEMAC_NUM_OF_PADDRS         7           /* Number of additional exact match MAC address registers */

typedef struct t_MacAddr
{
    uint32_t   mac_addr_l;         /* Lower 32 bits of 48-bit MAC address */
    uint32_t   mac_addr_u;         /* Upper 16 bits of 48-bit MAC address */
} t_MacAddr;

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
#define IF_MODE_RGMII               0x00000004
#define IF_MODE_RGMII_AUTO          0x00008000

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

/* Internal PHY Registers - SGMII */
#define PHY_SGMII_CR_PHY_RESET      0x8000
#define PHY_SGMII_CR_RESET_AN       0x0200
#define PHY_SGMII_CR_DEF_VAL        0x1140
#define PHY_SGMII_DEV_ABILITY_SGMII 0x4001
#define PHY_SGMII_IF_MODE_AN        0x0002
#define PHY_SGMII_IF_MODE_SGMII     0x0001

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

enum memac_counters {
    E_MEMAC_COUNTER_R64,
    E_MEMAC_COUNTER_R127,
    E_MEMAC_COUNTER_R255,
    E_MEMAC_COUNTER_R511,
    E_MEMAC_COUNTER_R1023,
    E_MEMAC_COUNTER_R1518,
    E_MEMAC_COUNTER_R1519X,
    E_MEMAC_COUNTER_RFRG,
    E_MEMAC_COUNTER_RJBR,
    E_MEMAC_COUNTER_RDRP,
    E_MEMAC_COUNTER_RALN,
    E_MEMAC_COUNTER_TUND,
    E_MEMAC_COUNTER_ROVR,
    E_MEMAC_COUNTER_RXPF,
    E_MEMAC_COUNTER_TXPF,
    E_MEMAC_COUNTER_ROCT,
    E_MEMAC_COUNTER_RMCA,
    E_MEMAC_COUNTER_RBCA,
    E_MEMAC_COUNTER_RPKT,
    E_MEMAC_COUNTER_RUCA,
    E_MEMAC_COUNTER_RERR,
    E_MEMAC_COUNTER_TOCT,
    E_MEMAC_COUNTER_TMCA,
    E_MEMAC_COUNTER_TBCA,
    E_MEMAC_COUNTER_TUCA,
    E_MEMAC_COUNTER_TERR
};

/*
 * memory map
 */
struct memac_regs {
    /* General Control and Status */
     uint32_t   reserved1[2];
     uint32_t   command_config;     /* 0x008 COMMAND_CONFIG - Control and configuration register */
     struct t_MacAddr mac_addr0;                    /* 0x00C-0x010 MAC_ADDR_0...MAC_ADDR_1 */
     uint32_t   maxfrm;             /* 0x014 MAXFRM - Maximum frame length register */
     uint32_t   reserved2[5];
     uint32_t   hashtable_ctrl;     /* 0x02C HASHTABLE_CTRL - Hash table control register */
     uint32_t   reserved3[4];
     uint32_t   ievent;             /* 0x040 IEVENT - Interrupt event register */
     uint32_t   tx_ipg_length;      /* 0x044 TX_IPG_LENGTH - Transmitter inter-packet-gap register */
     uint32_t   reserved4;
     uint32_t   imask;              /* 0x04C IMASK - Interrupt mask register */
     uint32_t   reserved5;

     uint32_t   pause_quanta[4];    /* 0x054 CL0x_PAUSE_QUANTA - CL0-7 Pause quanta register */
     uint32_t   pause_thresh[4];    /* 0x064 CL0x_PAUSE_THRESH - CL0-7 Pause quanta threshold register */
     uint32_t   rx_pause_status;    /* 0x074 RX_PAUSE_STATUS - Receive pause status register */
     uint32_t   reserved6[2];
     struct t_MacAddr mac_addr[MEMAC_NUM_OF_PADDRS]; /* 0x80-0x0B4 MAC_ADDR_2...MAC_ADDR_15 */
     uint32_t   lpwake_timer;       /* 0x0B8 LPWAKE_TIMER - EEE Low Power Wakeup Timer register */
     uint32_t   sleep_timer;        /* 0x0BC SLEEP_TIMER - Transmit EEE Low Power Timer register */
     uint32_t   reserved7[8];
     uint32_t   statn_config;       /* 0x0E0 STATN_CONFIG - Statistics configuration register */
     uint32_t   reserved8[7];

    /* Rx Statistics Counter */
     uint32_t   reoct_l;
     uint32_t   reoct_u;
     uint32_t   roct_l;
     uint32_t   roct_u;
     uint32_t   raln_l;
     uint32_t   raln_u;
     uint32_t   rxpf_l;
     uint32_t   rxpf_u;
     uint32_t   rfrm_l;
     uint32_t   rfrm_u;
     uint32_t   rfcs_l;
     uint32_t   rfcs_u;
     uint32_t   rvlan_l;
     uint32_t   rvlan_u;
     uint32_t   rerr_l;
     uint32_t   rerr_u;
     uint32_t   ruca_l;
     uint32_t   ruca_u;
     uint32_t   rmca_l;
     uint32_t   rmca_u;
     uint32_t   rbca_l;
     uint32_t   rbca_u;
     uint32_t   rdrp_l;
     uint32_t   rdrp_u;
     uint32_t   rpkt_l;
     uint32_t   rpkt_u;
     uint32_t   rund_l;
     uint32_t   rund_u;
     uint32_t   r64_l;
     uint32_t   r64_u;
     uint32_t   r127_l;
     uint32_t   r127_u;
     uint32_t   r255_l;
     uint32_t   r255_u;
     uint32_t   r511_l;
     uint32_t   r511_u;
     uint32_t   r1023_l;
     uint32_t   r1023_u;
     uint32_t   r1518_l;
     uint32_t   r1518_u;
     uint32_t   r1519x_l;
     uint32_t   r1519x_u;
     uint32_t   rovr_l;
     uint32_t   rovr_u;
     uint32_t   rjbr_l;
     uint32_t   rjbr_u;
     uint32_t   rfrg_l;
     uint32_t   rfrg_u;
     uint32_t   rcnp_l;
     uint32_t   rcnp_u;
     uint32_t   rdrntp_l;
     uint32_t   rdrntp_u;
     uint32_t   reserved9[12];

    /* Tx Statistics Counter */
     uint32_t   teoct_l;
     uint32_t   teoct_u;
     uint32_t   toct_l;
     uint32_t   toct_u;
     uint32_t   reserved10[2];
     uint32_t   txpf_l;
     uint32_t   txpf_u;
     uint32_t   tfrm_l;
     uint32_t   tfrm_u;
     uint32_t   tfcs_l;
     uint32_t   tfcs_u;
     uint32_t   tvlan_l;
     uint32_t   tvlan_u;
     uint32_t   terr_l;
     uint32_t   terr_u;
     uint32_t   tuca_l;
     uint32_t   tuca_u;
     uint32_t   tmca_l;
     uint32_t   tmca_u;
     uint32_t   tbca_l;
     uint32_t   tbca_u;
     uint32_t   reserved11[2];
     uint32_t   tpkt_l;
     uint32_t   tpkt_u;
     uint32_t   tund_l;
     uint32_t   tund_u;
     uint32_t   t64_l;
     uint32_t   t64_u;
     uint32_t   t127_l;
     uint32_t   t127_u;
     uint32_t   t255_l;
     uint32_t   t255_u;
     uint32_t   t511_l;
     uint32_t   t511_u;
     uint32_t   t1023_l;
     uint32_t   t1023_u;
     uint32_t   t1518_l;
     uint32_t   t1518_u;
     uint32_t   t1519x_l;
     uint32_t   t1519x_u;
     uint32_t   reserved12[6];
     uint32_t   tcnp_l;
     uint32_t   tcnp_u;
     uint32_t   reserved13[14];

    /* Line Interface Control */
     uint32_t   if_mode;            /* 0x300 IF_MODE - Interface Mode Control register */
     uint32_t   if_status;          /* 0x304 IF_STATUS - Interface Status register */
     uint32_t   reserved14[14];

    /* HiGig/2 */
     uint32_t   hg_config;          /* 0x340 HG_CONFIG - HiGig/2 Control and configuration register */
     uint32_t   reserved15[3];
     uint32_t   hg_pause_quanta;    /* 0x350 HG_PAUSE_QUANTA - HiGig2 Pause quanta register */
     uint32_t   reserved16[3];
     uint32_t   hg_pause_thresh;    /* 0x360 HG_PAUSE_THRESH - HiGig2 Pause quanta threshold register */
     uint32_t   reserved17[3];
     uint32_t   hgrx_pause_status;  /* 0x370 HGRX_PAUSE_STATUS - HiGig2 Receive pause status register*/
     uint32_t   hg_fifos_status;    /* 0x374 HG_FIFOS_STATUS - HiGig2 fifos status register */
     uint32_t   rhm;                /* 0x378 RHM - Receive HiGig2 messages counter register */
     uint32_t   thm;                /* 0x37C THM - Transmit HiGig2 messages counter register */
};


struct memac_cfg {

    bool        reset_on_init;
    bool        rx_error_discard;
    bool        pause_ignore;
    bool        pause_forward_enable;
    bool        no_length_check_enable;
    bool        cmd_frame_enable;
    bool        send_idle_enable;
    bool        wan_mode_enable;
    bool        promiscuous_mode_enable;
    bool        tx_addr_ins_enable;
    bool        loopback_enable;
    bool        lgth_check_nostdr;
    bool        time_stamp_enable;
    bool        pad_enable;
    bool        phy_tx_ena_on;
    bool        rx_sfd_any;
    bool        rxPblFwd;
    bool        txPblFwd;
    bool        debugMode;
    uint16_t    max_frame_length;
    uint16_t    pause_quant;
    uint32_t    tx_ipg_length;
#ifdef FM_TX_ECC_FRMS_ERRATA_10GMAC_A004
    bool        skip_fman11_workaround;
#endif /* FM_TX_ECC_FRMS_ERRATA_10GMAC_A004 */
};

/**
 * memac_defconfig() - Get default MEMAC configuration
 * @cfg:    pointer to configuration structure.
 *
 * Call this function to obtain a default set of configuration values for
 * initializing MEMAC. The user can overwrite any of the values before calling
 * memac_init(), if specific configuration needs to be applied.
 */
void memac_defconfig(struct memac_cfg *cfg);
void memac_set_promiscuous(struct memac_regs *regs, bool val);
void memac_hardware_add_addr_in_paddr(struct memac_regs *regs, uint8_t *adr, uint8_t paddr_num);
void memac_hardware_clear_addr_in_paddr(struct memac_regs *regs, uint8_t paddr_num);
void memac_enable(struct memac_regs *regs, bool apply_rx, bool apply_tx);
void memac_disable(struct memac_regs *regs, bool apply_rx, bool apply_tx);
uint64_t memac_get_counter(struct memac_regs *regs, enum memac_counters reg_name);
void memac_set_tx_pause_frames(struct memac_regs    *regs,
                               uint8_t              priority,
                               uint16_t             pauseTime,
                               uint16_t             threshTime);
uint16_t memac_get_max_frame_length(struct memac_regs *regs);
void memac_init(struct memac_regs *regs, struct memac_cfg *cfg,
        enum enet_interface enet_interface, enum enet_speed enet_speed,
        uint32_t exceptions);
void memac_set_exception(struct memac_regs *regs, uint32_t val, bool enable);
void memac_reset_counter(struct memac_regs *regs);
void memac_reset(struct memac_regs *regs);
void memac_set_hash_table(struct memac_regs *regs, uint32_t val);
void memac_set_rx_ignore_pause_frames(struct memac_regs  *regs,bool enable);
void memac_set_loopback(struct memac_regs *regs, bool enable);
void memac_reset_counter(struct memac_regs *regs);


#endif /*__FSL_FMAN_MEMAC_H*/
