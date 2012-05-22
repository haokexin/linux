/*
 * arch/arm/mach-spear13xx/include/mach/spear1340.h
 *
 * SPEAr1340 Machine specific definition
 *
 * Copyright (C) 2011 ST Microelectronics
 * Viresh Kumar <viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifdef CONFIG_CPU_SPEAR1340

#ifndef __MACH_SPEAR1340_H
#define __MACH_SPEAR1340_H

#define SPEAR1340_FSMC_NAND_BASE	UL(0xB0800000)
#define SPEAR1340_SATA_BASE		UL(0xB1000000) /* Configuration space */
#define SPEAR1340_I2S_REC_BASE		UL(0xB2000000)
#define SPEAR1340_I2S_PLAY_BASE		UL(0xB2400000)
#define SPEAR1340_I2C1_BASE		UL(0xB4000000)
#define SPEAR1340_UART1_BASE		UL(0xB4100000)
#define SPEAR1340_SPDIF_OUT_BASE	UL(0xD0000000)
#define SPEAR1340_SPDIF_IN_BASE		UL(0xD0100000)
#define SPEAR1340_SPDIF_IN_FIFO_BASE	UL(0xD0110000)
#define SPEAR1340_CAM0_BASE		UL(0xD0200000)
#define SPEAR1340_CAM1_BASE		UL(0xD0300000)
#define SPEAR1340_CAM2_BASE		UL(0xD0400000)
#define SPEAR1340_CAM3_BASE		UL(0xD0500000)
#define SPEAR1340_CEC0_BASE		UL(0xD0600000)
#define SPEAR1340_CEC1_BASE		UL(0xD0700000)
#define SPEAR1340_VIP_BASE		UL(0xD0800000)
#define SPEAR1340_GPU_BASE		UL(0xD0900000)
#define SPEAR1340_PWM_BASE		UL(0xE0180000)

#define SPEAR1340_PLGPIO_BASE		UL(0xE2800000)
/* PLGPIO Registers */
/* gpio direction */
#define SPEAR1340_PLGPIO_DIR_OFF	0x000
/* read data */
#define SPEAR1340_PLGPIO_RDATA_OFF	0x020
/* write data */
#define SPEAR1340_PLGPIO_WDATA_OFF	0x040
/* edge interrupt type, 0: falling, 1: rising */
#define SPEAR1340_PLGPIO_EIT_OFF	0x060
/* interrupt enable */
#define SPEAR1340_PLGPIO_IE_OFF		0x080
/* Masked interrupt status */
#define SPEAR1340_PLGPIO_MIS_OFF	0x0A0

#define SPEAR1340_UOC_BASE		UL(0xE3800000)
#define SPEAR1340_VIDEO_ENC_BASE	UL(0xEBC00000)
#define SPEAR1340_VIDEO_DEC_BASE	UL(0xEBD00000)

#endif /* __MACH_SPEAR1340_H */

#endif /* CONFIG_CPU_SPEAR1340 */
