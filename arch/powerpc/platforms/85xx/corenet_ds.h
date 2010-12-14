/*
 * Corenet based SoC DS Setup
 *
 * Copyright 2009-2010 Freescale Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef CORENET_DS_H
#define CORENET_DS_H

#define SRDS_RSTCTL_RST		0x80000000
#define SRDS_RSTCTL_RSTDONE	0x40000000
#define SRDS_RSTCTL_RSTERR	0x20000000
#define SRDS_RSTCTL_SDPD	0x00000020

#define SRDS_PLLCR0_RFCK_SEL_MASK	0x30000000
#define SRDS_PLLCR0_RFCK_SEL_100	0x00000000
#define SRDS_PLLCR0_RFCK_SEL_125	0x10000000
#define SRDS_PLLCR0_RFCK_SEL_156_25	0x20000000
#define SRDS_PLLCR0_FRATE_SEL_MASK	0x00030000
#define SRDS_PLLCR0_FRATE_SEL_5		0x00000000
#define SRDS_PLLCR0_FRATE_SEL_6_25	0x00010000

#define SRDS_PLLCR1_PLL_BWSEL	0x08000000

#define SRDS_PCCR2_RST_XGMII1		0x00800000
#define SRDS_PCCR2_RST_XGMII2		0x00400000

#define SRDS_GCR0_RRST			0x00400000
#define SRDS_GCR0_1STLANE		0x00010000

#define SRDS_GCR1_REIDL_CTL_MASK	0x001f0000
#define SRDS_GCR1_REIDL_CTL_PCIE	0x00100000
#define SRDS_GCR1_REIDL_CTL_SRIO	0x00000000
#define SRDS_GCR1_REIDL_CTL_SGMII	0x00040000
#define SRDS_GCR1_OPAD_CTL		0x04000000

#define SRDS_TECR0_TEQ_TYPE_MASK	0x30000000
#define SRDS_TECR0_TEQ_TYPE_2LVL	0x10000000

#define SRDS_TTLCR0_FLT_SEL_MASK	0x3f000000
#define SRDS_TTLCR0_PM_DIS		0x00004000

struct corenet_serdes {
	struct {
		u32	rstctl;	/* Reset Control Register */
		u32	pllcr0; /* PLL Control Register 0 */
		u32	pllcr1; /* PLL Control Register 1 */
		u32	res[5];
	} bank[3];
	u32	res1[12];
	u32	srdstcalcr;	/* TX Calibration Control */
	u32	res2[3];
	u32	srdsrcalcr;	/* RX Calibration Control */
	u32	res3[3];
	u32	srdsgr0;	/* General Register 0 */
	u32	res4[11];
	u32	srdspccr0;	/* Protocol Converter Config 0 */
	u32	srdspccr1;	/* Protocol Converter Config 1 */
	u32	srdspccr2;	/* Protocol Converter Config 2 */
	u32	res5[197];
	struct {
		u32	gcr0;	/* General Control Register 0 */
		u32	gcr1;	/* General Control Register 1 */
		u32	res1[4];
		u32	tecr0;	/* TX Equalization Control Reg 0 */
		u32	res3;
		u32	ttlcr0;	/* Transition Tracking Loop Ctrl 0 */
		u32	res4[7];
	} lane[24];
	u32 res6[384];
};

extern void __init corenet_ds_pic_init(void);
extern void __init corenet_ds_setup_arch(void);
extern int __init corenet_ds_publish_devices(void);
extern int __init declare_of_platform_devices(void);
extern void __init corenet_ds_init_early(void);

#endif
