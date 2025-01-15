// SPDX-License-Identifier: GPL-2.0-only
/*
 * PTP Header file
 * Copyright (C) 2013  Vayavya Labs Pvt Ltd
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef	__BSTGMAC_PTP_H__
#define	__BSTGMAC_PTP_H__

#include "bstgmac.h"

#define PTP_XGMAC_OFFSET	0xd00
#define	PTP_GMAC4_OFFSET	0xb00
#define	PTP_GMAC3_X_OFFSET	0x700

#define PTP_TS_INTERVAL         2
#define PTP_MAX_TRIG_IN_NUM     4

/* Interrupt register offsets */
#define	PTP_MAC_INTR_EN		0xb4	/* MAC Interrupt Enable Reg */

/* IEEE 1588 PTP register offsets */
#define	PTP_TCR		0x00	/* Timestamp Control Reg */
#define	PTP_SSIR	0x04	/* Sub-Second Increment Reg */
#define	PTP_STSR	0x08	/* System Time – Seconds Regr */
#define	PTP_STNSR	0x0c	/* System Time – Nanoseconds Reg */
#define	PTP_STSUR	0x10	/* System Time – Seconds Update Reg */
#define	PTP_STNSUR	0x14	/* System Time – Nanoseconds Update Reg */
#define	PTP_TAR		0x18	/* Timestamp Addend Reg */
#define	PTP_TSR		0x20	/* Timestamp Status Reg */
#define	PTP_ACR	    0x40	/* Auxiliary Control Reg */
#define	PTP_ATNSR	0x48	/* Auxiliary Time - Nanoseconds Reg */
#define	PTP_ATSR	0x4c	/* Auxiliary Time - Seconds Reg */
#define	PTP_PPSCR	0x70	/* PPS Control Reg */

#define	PTP_STNSUR_ADDSUB_SHIFT	31
#define	PTP_DIGITAL_ROLLOVER_MODE	0x3B9ACA00	/* 10e9-1 ns */
#define	PTP_BINARY_ROLLOVER_MODE	0x80000000	/* ~0.466 ns */

/* PTP Timestamp control register defines */
#define	PTP_TCR_TSENA		BIT(0)	/* Timestamp Enable */
#define	PTP_TCR_TSCFUPDT	BIT(1)	/* Timestamp Fine/Coarse Update */
#define	PTP_TCR_TSINIT		BIT(2)	/* Timestamp Initialize */
#define	PTP_TCR_TSUPDT		BIT(3)	/* Timestamp Update */
#define	PTP_TCR_TSTRIG		BIT(4)	/* Timestamp Interrupt Trigger Enable */
#define	PTP_TCR_TSADDREG	BIT(5)	/* Addend Reg Update */
#define	PTP_TCR_TSENALL		BIT(8)	/* Enable Timestamp for All Frames */
#define	PTP_TCR_TSCTRLSSR	BIT(9)	/* Digital or Binary Rollover Control */
/* Enable PTP packet Processing for Version 2 Format */
#define	PTP_TCR_TSVER2ENA	BIT(10)
/* Enable Processing of PTP over Ethernet Frames */
#define	PTP_TCR_TSIPENA		BIT(11)
/* Enable Processing of PTP Frames Sent over IPv6-UDP */
#define	PTP_TCR_TSIPV6ENA	BIT(12)
/* Enable Processing of PTP Frames Sent over IPv4-UDP */
#define	PTP_TCR_TSIPV4ENA	BIT(13)
/* Enable Timestamp Snapshot for Event Messages */
#define	PTP_TCR_TSEVNTENA	BIT(14)
/* Enable Snapshot for Messages Relevant to Master */
#define	PTP_TCR_TSMSTRENA	BIT(15)
/* Select PTP packets for Taking Snapshots */
#define	PTP_TCR_SNAPTYPSEL_1	BIT(16)
#define	PTP_GMAC4_TCR_SNAPTYPSEL_1	GENMASK(17, 16)
/* PTP Aux Snapshot */
#define	PTP_ACR_MASK		GENMASK(7, 4)	/* Aux Snapshot Mask */
/* Enable MAC address for PTP Frame Filtering */
#define	PTP_TCR_TSENMACADDR	BIT(18)

/* SSIR defines */
#define	PTP_SSIR_SSINC_MASK		0xff
#define	GMAC4_PTP_SSIR_SSINC_SHIFT	16

#define BST_SC_PMM_REG_BASE_ADDR    (0x30001000)
#define	BST_SC_PMM_REG_MAP_RANGE	(0x330)

/* SC PMM Register Offsets */
#define SC_PMM_REG_R_SC_PMM_REG10	(0x28)

/* SC_PMM_REG10 Defines */
#define SC_PMM_REG10_8_6            GENMASK(8, 6)
#define SC_PMM_REG10_5_3            GENMASK(5, 3)
#define SOC_XGMAC_PTP_TS_TRIG_IN0   (0x20)
#define SOC_XGMAC_PTP_TS_TRIG_IN1   (0x100)

/* MAC Interrupt Enable register defines */
#define	MAC_INTR_EN_TSIE		BIT(12)	/*  Timestamp Interrupt Enable */

extern struct bstgmac_priv *gmac_priv_g[BSTGMAC_CORE_NUM];
void bstptp_extts_interrupt(int irq, struct bstgmac_priv *priv);

#endif	/* __BSTGMAC_PTP_H__ */
