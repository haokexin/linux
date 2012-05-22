/*
 * arch/arm/mach-spear13xx/include/mach/generic.h
 *
 * spear13xx machine family generic header file
 *
 * Copyright (C) 2010 ST Microelectronics
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_GENERIC_H
#define __MACH_GENERIC_H

#include <linux/designware_i2s.h>
#include <linux/dmaengine.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/amba/bus.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/map.h>
#include <asm/setup.h>
#include <plat/padmux.h>
#include <plat/plug_board.h>

#if defined(CONFIG_CPU_SPEAR1300) || defined(CONFIG_CPU_SPEAR1310_REVA) || \
			defined(CONFIG_CPU_SPEAR900) || \
			defined(CONFIG_CPU_SPEAR1310)
/*
 * Function enable (Pad multiplexing register) offsets
 */
/* pad mux declarations */
#define SPEAR13XX_PMX_I2S1_MASK		(1 << 3)
#define SPEAR13XX_PMX_I2S2_MASK		(1 << 16)	/* Offset 4 */
#define SPEAR13XX_PMX_CLCD1_MASK	(1 << 5)
#define SPEAR13XX_PMX_CLCD2_MASK	(1 << 3)	/* Offset 4 */
#define SPEAR13XX_PMX_EGPIO00_MASK	(1 << 6)
#define SPEAR13XX_PMX_EGPIO01_MASK	(1 << 7)
#define SPEAR13XX_PMX_EGPIO02_MASK	(1 << 8)
#define SPEAR13XX_PMX_EGPIO03_MASK	(1 << 9)
#define SPEAR13XX_PMX_EGPIO04_MASK	(1 << 10)
#define SPEAR13XX_PMX_EGPIO05_MASK	(1 << 11)
#define SPEAR13XX_PMX_EGPIO06_MASK	(1 << 12)
#define SPEAR13XX_PMX_EGPIO07_MASK	(1 << 13)
#define SPEAR13XX_PMX_EGPIO08_MASK	(1 << 14)
#define SPEAR13XX_PMX_EGPIO09_MASK	(1 << 15)
#define SPEAR13XX_PMX_EGPIO10_MASK	(1 << 5)	/* Offset 4 */
#define SPEAR13XX_PMX_EGPIO11_MASK	(1 << 6)	/* Offset 4 */
#define SPEAR13XX_PMX_EGPIO12_MASK	(1 << 7)	/* Offset 4 */
#define SPEAR13XX_PMX_EGPIO13_MASK	(1 << 8)	/* Offset 4 */
#define SPEAR13XX_PMX_EGPIO14_MASK	(1 << 9)	/* Offset 4 */
#define SPEAR13XX_PMX_EGPIO15_MASK	(1 << 10)	/* Offset 4 */
#define SPEAR13XX_PMX_EGPIO_0_GRP_MASK (SPEAR13XX_PMX_EGPIO00_MASK | \
		SPEAR13XX_PMX_EGPIO01_MASK | SPEAR13XX_PMX_EGPIO02_MASK | \
		SPEAR13XX_PMX_EGPIO03_MASK | SPEAR13XX_PMX_EGPIO04_MASK | \
		SPEAR13XX_PMX_EGPIO05_MASK | SPEAR13XX_PMX_EGPIO06_MASK | \
		SPEAR13XX_PMX_EGPIO07_MASK | SPEAR13XX_PMX_EGPIO08_MASK | \
		SPEAR13XX_PMX_EGPIO09_MASK)
#define SPEAR13XX_PMX_EGPIO_1_GRP_MASK (SPEAR13XX_PMX_EGPIO10_MASK | \
		SPEAR13XX_PMX_EGPIO11_MASK | SPEAR13XX_PMX_EGPIO12_MASK | \
		SPEAR13XX_PMX_EGPIO13_MASK | SPEAR13XX_PMX_EGPIO14_MASK | \
		SPEAR13XX_PMX_EGPIO15_MASK)

#define SPEAR13XX_PMX_SMI_MASK		(1 << 16)
#define SPEAR13XX_PMX_SMINCS2_MASK	(1 << 1)	/* Offset 4 */
#define SPEAR13XX_PMX_SMINCS3_MASK	(1 << 2)	/* Offset 4 */

#define SPEAR13XX_PMX_GMIICLK_MASK			(1 << 18)
#define SPEAR13XX_PMX_GMIICOL_CRS_XFERER_MIITXCLK_MASK	(1 << 19)
#define SPEAR13XX_PMX_RXCLK_RDV_TXEN_D03_MASK		(1 << 20)
#define SPEAR13XX_PMX_GMIID47_MASK			(1 << 21)
#define SPEAR13XX_PMX_MDC_MDIO_MASK			(1 << 22)
#define SPEAR13XX_PMX_MCI_DATA8_15_MASK			(1 << 23)

#define SPEAR13XX_PMX_GMII_MASK (SPEAR13XX_PMX_GMIICLK_MASK | \
		SPEAR13XX_PMX_GMIICOL_CRS_XFERER_MIITXCLK_MASK | \
		SPEAR13XX_PMX_RXCLK_RDV_TXEN_D03_MASK | \
		SPEAR13XX_PMX_GMIID47_MASK | SPEAR13XX_PMX_MDC_MDIO_MASK)

#define SPEAR13XX_PMX_NAND8_MASK	(1 << 17)
#define SPEAR13XX_PMX_NFAD023_MASK	(1 << 24)
#define SPEAR13XX_PMX_NFAD24_MASK	(1 << 25)
#define SPEAR13XX_PMX_NFAD25_MASK	(1 << 26)
#define SPEAR13XX_PMX_NFWPRT1_MASK	(1 << 24)	/* Offset 4 */
#define SPEAR13XX_PMX_NFWPRT2_MASK	(1 << 26)	/* Offset 4 */
#define SPEAR13XX_PMX_NFWPRT3_MASK	(1 << 28)
#define SPEAR13XX_PMX_NFRSTPWDWN0_MASK	(1 << 29)
#define SPEAR13XX_PMX_NFRSTPWDWN1_MASK	(1 << 30)
#define SPEAR13XX_PMX_NFRSTPWDWN2_MASK	(1 << 31)
#define SPEAR13XX_PMX_NFRSTPWDWN3_MASK	(1 << 0)	/* Offset 4 */
#define SPEAR13XX_PMX_NFCE1_MASK	(1 << 20)	/* Offset 4 */
#define SPEAR13XX_PMX_NFCE2_MASK	(1 << 22)	/* Offset 4 */
#define SPEAR13XX_PMX_NFCE3_MASK	(1 << 27)
#define SPEAR13XX_PMX_NFIO815_MASK	(1 << 18)	/* Offset 4 */

#define SPEAR13XX_PMX_NAND8BIT_0_MASK (SPEAR13XX_PMX_NAND8_MASK | \
		SPEAR13XX_PMX_NFAD023_MASK | SPEAR13XX_PMX_NFAD24_MASK | \
		SPEAR13XX_PMX_NFAD25_MASK | SPEAR13XX_PMX_NFWPRT3_MASK | \
		SPEAR13XX_PMX_NFRSTPWDWN0_MASK | SPEAR13XX_PMX_NFRSTPWDWN1_MASK\
		| SPEAR13XX_PMX_NFRSTPWDWN2_MASK | SPEAR13XX_PMX_NFCE3_MASK)
#define SPEAR13XX_PMX_NAND8BIT_1_MASK	(SPEAR13XX_PMX_NFRSTPWDWN3_MASK)

#define SPEAR13XX_PMX_NAND8BIT4DEV_0_MASK	SPEAR13XX_PMX_NAND8BIT_0_MASK
#define SPEAR13XX_PMX_NAND8BIT4DEV_1_MASK	(SPEAR13XX_PMX_NAND8BIT_1_MASK\
		| SPEAR13XX_PMX_NFCE1_MASK | SPEAR13XX_PMX_NFCE2_MASK | \
		SPEAR13XX_PMX_NFWPRT1_MASK | SPEAR13XX_PMX_NFWPRT2_MASK)

#define SPEAR13XX_PMX_NAND16BIT_0_MASK	(SPEAR13XX_PMX_NAND8BIT_0_MASK)
#define SPEAR13XX_PMX_NAND16BIT_1_MASK	(SPEAR13XX_PMX_NAND8BIT_1_MASK | \
		SPEAR13XX_PMX_NFIO815_MASK)
#define SPEAR13XX_PMX_NAND16BIT4DEV_0_MASK SPEAR13XX_PMX_NAND8BIT4DEV_0_MASK
#define SPEAR13XX_PMX_NAND16BIT4DEV_1_MASK	\
	(SPEAR13XX_PMX_NAND8BIT4DEV_1_MASK | SPEAR13XX_PMX_NFIO815_MASK)

#define SPEAR13XX_PMX_KBD_ROW0_MASK	(1 << 25)	/* Offset 4 */
#define SPEAR13XX_PMX_KBD_ROW1_MASK	(1 << 23)	/* Offset 4 */
#define SPEAR13XX_PMX_KBD_ROWCOL25_MASK	(1 << 17)	/* Offset 4 */
#define SPEAR13XX_PMX_KBD_ROWCOL68_MASK	(1 << 4)	/* Offset 4 */
#define SPEAR13XX_PMX_KBD_COL0_MASK	(1 << 21)	/* Offset 4 */
#define SPEAR13XX_PMX_KBD_COL1_MASK	(1 << 19)	/* Offset 4 */
#define SPEAR13XX_PMX_KEYBOARD_6X6_MASK	(SPEAR13XX_PMX_KBD_ROW0_MASK | \
		SPEAR13XX_PMX_KBD_ROW1_MASK | SPEAR13XX_PMX_KBD_ROWCOL25_MASK \
		| SPEAR13XX_PMX_KBD_COL0_MASK | SPEAR13XX_PMX_KBD_COL1_MASK)

#define SPEAR13XX_PMX_UART0_MASK	(1 << 1)
#define SPEAR13XX_PMX_I2C_MASK		(1 << 2)
#define SPEAR13XX_PMX_SSP_MASK		(1 << 4)
#define SPEAR13XX_PMX_UART0_MODEM_MASK	(1 << 11)	/* Offset 4 */
#define SPEAR13XX_PMX_GPT0_TMR1_MASK	(1 << 12)	/* Offset 4 */
#define SPEAR13XX_PMX_GPT0_TMR2_MASK	(1 << 13)	/* Offset 4 */
#define SPEAR13XX_PMX_GPT1_TMR1_MASK	(1 << 14)	/* Offset 4 */
#define SPEAR13XX_PMX_GPT1_TMR2_MASK	(1 << 15)	/* Offset 4 */

#define SPEAR13XX_PMX_MCIDATA0_MASK	(1 << 27)	/* Offset 4 */
#define SPEAR13XX_PMX_MCIDATA1_MASK	(1 << 28)	/* Offset 4 */
#define SPEAR13XX_PMX_MCIDATA2_MASK	(1 << 29)	/* Offset 4 */
#define SPEAR13XX_PMX_MCIDATA3_MASK	(1 << 30)	/* Offset 4 */
#define SPEAR13XX_PMX_MCIDATA4_MASK	(1 << 31)	/* Offset 4 */
#define SPEAR13XX_PMX_MCIDATA5_MASK	(1 << 0)	/* Offset 8 */
#define SPEAR13XX_PMX_MCIDATA6_MASK	(1 << 1)	/* Offset 8 */
#define SPEAR13XX_PMX_MCIDATA7_MASK	(1 << 2)	/* Offset 8 */
#define SPEAR13XX_PMX_MCIDATA1SD_MASK	(1 << 3)	/* Offset 8 */
#define SPEAR13XX_PMX_MCIDATA2SD_MASK	(1 << 4)	/* Offset 8 */
#define SPEAR13XX_PMX_MCIDATA3SD_MASK	(1 << 5)	/* Offset 8 */
#define SPEAR13XX_PMX_MCIADDR0ALE_MASK	(1 << 6)	/* Offset 8 */
#define SPEAR13XX_PMX_MCIADDR1CLECLK_MASK (1 << 7)	/* Offset 8 */
#define SPEAR13XX_PMX_MCIADDR2_MASK	(1 << 8)	/* Offset 8 */
#define SPEAR13XX_PMX_MCICECF_MASK	(1 << 9)	/* Offset 8 */
#define SPEAR13XX_PMX_MCICEXD_MASK	(1 << 10)	/* Offset 8 */
#define SPEAR13XX_PMX_MCICESDMMC_MASK	(1 << 11)	/* Offset 8 */
#define SPEAR13XX_PMX_MCICDCF1_MASK	(1 << 12)	/* Offset 8 */
#define SPEAR13XX_PMX_MCICDCF2_MASK	(1 << 13)	/* Offset 8 */
#define SPEAR13XX_PMX_MCICDXD_MASK	(1 << 14)	/* Offset 8 */
#define SPEAR13XX_PMX_MCICDSDMMC_MASK	(1 << 15)	/* Offset 8 */
#define SPEAR13XX_PMX_MCIDATADIR_MASK	(1 << 16)	/* Offset 8 */
#define SPEAR13XX_PMX_MCIDMARQWP_MASK	(1 << 17)	/* Offset 8 */
#define SPEAR13XX_PMX_MCIIORDRE_MASK	(1 << 18)	/* Offset 8 */
#define SPEAR13XX_PMX_MCIIOWRWE_MASK	(1 << 19)	/* Offset 8 */
#define SPEAR13XX_PMX_MCIRESETCF_MASK	(1 << 20)	/* Offset 8 */
#define SPEAR13XX_PMX_MCICS0CE_MASK	(1 << 21)	/* Offset 8 */
#define SPEAR13XX_PMX_MCICFINTR_MASK	(1 << 22)	/* Offset 8 */
#define SPEAR13XX_PMX_MCIIORDY_MASK	(1 << 23)	/* Offset 8 */
#define SPEAR13XX_PMX_MCICS1_MASK	(1 << 24)	/* Offset 8 */
#define SPEAR13XX_PMX_MCIDMAACK_MASK	(1 << 25)	/* Offset 8 */
#define SPEAR13XX_PMX_MCISDCMD_MASK	(1 << 26)	/* Offset 8 */
#define SPEAR13XX_PMX_MCILEDS_MASK	(1 << 27)	/* Offset 8 */
#define SPEAR13XX_PMX_TOUCH_XY_MASK	(1 << 28)	/* Offset 8 */
#define SPEAR13XX_PMX_SSP0_CS0_MASK	(1 << 29)	/* Offset 8 */
#define SPEAR13XX_PMX_SSP0_CS1_2_MASK	(1 << 30)	/* Offset 8 */

#define SPEAR13XX_PMX_MCIFALL_1_MASK	(0xF8000000)
#define SPEAR13XX_PMX_MCIFALL_2_MASK	(0x0FFFFFFF)

/* pad mux devices */
extern struct pmx_dev spear13xx_pmx_i2c;
extern struct pmx_dev spear13xx_pmx_ssp;
extern struct pmx_dev spear13xx_pmx_i2s1;
extern struct pmx_dev spear13xx_pmx_i2s2;
extern struct pmx_dev spear13xx_pmx_clcd;
extern struct pmx_dev spear13xx_pmx_clcd_hires;
extern struct pmx_dev spear13xx_pmx_egpio_grp;
extern struct pmx_dev spear13xx_pmx_smi_2_chips;
extern struct pmx_dev spear13xx_pmx_smi_4_chips;
extern struct pmx_dev spear13xx_pmx_gmii;
extern struct pmx_dev spear13xx_pmx_nand_8bit;
extern struct pmx_dev spear13xx_pmx_nand_16bit;
extern struct pmx_dev spear13xx_pmx_keyboard_6x6;
extern struct pmx_dev spear13xx_pmx_keyboard_9x9;
extern struct pmx_dev spear13xx_pmx_uart0;
extern struct pmx_dev spear13xx_pmx_uart0_modem;
extern struct pmx_dev spear13xx_pmx_gpt_0_1;
extern struct pmx_dev spear13xx_pmx_gpt_0_2;
extern struct pmx_dev spear13xx_pmx_gpt_1_1;
extern struct pmx_dev spear13xx_pmx_gpt_1_2;
extern struct pmx_dev spear13xx_pmx_mcif;
extern struct pmx_dev spear13xx_pmx_sdhci;
extern struct pmx_dev spear13xx_pmx_cf;
extern struct pmx_dev spear13xx_pmx_xd;
#endif

#if defined(CONFIG_CPU_SPEAR1310_REVA)
#define SPEAR1310_REVA_FUNC_CNTL_0	UL(0x6C800000)

#define SPEAR1310_REVA_PMX_SMII_MASK	(1 << 24)	/* Func cntl reg0 */
#define SPEAR1310_REVA_PMX_EGPIO7_MASK	(1 << 2)	/* Pcm cfg reg */

extern struct pmx_dev spear1310_reva_pmx_uart1_modem;
extern struct pmx_dev spear1310_reva_pmx_uart_1;
extern struct pmx_dev spear1310_reva_pmx_uart_2;
extern struct pmx_dev spear1310_reva_pmx_uart_3_4_5;
extern struct pmx_dev spear1310_reva_pmx_rs485_hdlc_1_2;
extern struct pmx_dev spear1310_reva_pmx_tdm_hdlc_1_2;
extern struct pmx_dev spear1310_reva_pmx_nand32bit;
extern struct pmx_dev spear1310_reva_pmx_fsmc16bit_4_chips;
extern struct pmx_dev spear1310_reva_pmx_fsmc32bit_4_chips;
extern struct pmx_dev spear1310_reva_pmx_gmii1;
extern struct pmx_dev spear1310_reva_pmx_rgmii;
extern struct pmx_dev spear1310_reva_pmx_i2c1;
extern struct pmx_dev spear1310_reva_pmx_smii_0_1_2;
extern struct pmx_dev spear1310_reva_pmx_can;
#endif

#ifdef CONFIG_CPU_SPEAR1310

/* PCI Macros */
#define SPEAR1310_PMX_PCI_REG0_MASK (SPEAR13XX_PMX_MCI_DATA8_15_MASK)
#define SPEAR1310_PMX_PCI_REG1_MASK (SPEAR13XX_PMX_SMINCS2_MASK | \
		SPEAR13XX_PMX_SMINCS3_MASK | SPEAR13XX_PMX_CLCD2_MASK | \
		SPEAR13XX_PMX_KBD_ROWCOL68_MASK | \
		SPEAR13XX_PMX_EGPIO_1_GRP_MASK | SPEAR13XX_PMX_GPT0_TMR1_MASK |\
		SPEAR13XX_PMX_GPT0_TMR2_MASK | SPEAR13XX_PMX_GPT1_TMR1_MASK | \
		SPEAR13XX_PMX_GPT1_TMR2_MASK | SPEAR13XX_PMX_I2S2_MASK | \
		SPEAR13XX_PMX_NFCE2_MASK)
#define SPEAR1310_PMX_PCI_REG2_MASK (SPEAR13XX_PMX_TOUCH_XY_MASK | \
		SPEAR13XX_PMX_SSP0_CS0_MASK | SPEAR13XX_PMX_SSP0_CS1_2_MASK)

/* SMII Macros */
#define SPEAR1310_PMX_SMII_0_1_2_MASK (SPEAR13XX_PMX_CLCD2_MASK | \
		SPEAR13XX_PMX_KBD_ROWCOL68_MASK)

/* RGMII Macros */
#define SPEAR1310_PMX_RGMII_REG0_MASK (SPEAR13XX_PMX_MCI_DATA8_15_MASK | \
		SPEAR13XX_PMX_GMIICOL_CRS_XFERER_MIITXCLK_MASK | \
		SPEAR13XX_PMX_GMIID47_MASK)
#define SPEAR1310_PMX_RGMII_REG1_MASK (SPEAR13XX_PMX_KBD_ROWCOL68_MASK | \
		SPEAR13XX_PMX_EGPIO_1_GRP_MASK | SPEAR13XX_PMX_KBD_ROW1_MASK | \
		SPEAR13XX_PMX_KBD_ROW0_MASK | SPEAR13XX_PMX_NFCE2_MASK)
#define SPEAR1310_PMX_RGMII_REG2_MASK (SPEAR13XX_PMX_TOUCH_XY_MASK | \
		SPEAR13XX_PMX_SSP0_CS0_MASK | SPEAR13XX_PMX_SSP0_CS1_2_MASK)

extern struct pmx_dev spear1310_pmx_uart_1_dis_i2c;
extern struct pmx_dev spear1310_pmx_uart_1_dis_sd;
extern struct pmx_dev spear1310_pmx_uart_2_3;
extern struct pmx_dev spear1310_pmx_uart_4;
extern struct pmx_dev spear1310_pmx_uart_5;
extern struct pmx_dev spear1310_pmx_rs485_0_1_tdm_0_1;
extern struct pmx_dev spear1310_pmx_i2c_1_2;
extern struct pmx_dev spear1310_pmx_i2c3_dis_smi_clcd;
extern struct pmx_dev spear1310_pmx_i2c3_dis_sd_i2s1;
extern struct pmx_dev spear1310_pmx_i2c_4_5_dis_smi;
extern struct pmx_dev spear1310_pmx_i2c4_dis_sd;
extern struct pmx_dev spear1310_pmx_i2c5_dis_sd;
extern struct pmx_dev spear1310_pmx_i2c_6_7_dis_kbd;
extern struct pmx_dev spear1310_pmx_i2c6_dis_sd;
extern struct pmx_dev spear1310_pmx_i2c7_dis_sd;
extern struct pmx_dev spear1310_pmx_rgmii;
extern struct pmx_dev spear1310_pmx_can0_dis_nor;
extern struct pmx_dev spear1310_pmx_can0_dis_sd;
extern struct pmx_dev spear1310_pmx_can1_dis_sd;
extern struct pmx_dev spear1310_pmx_can1_dis_kbd;
extern struct pmx_dev spear1310_pmx_pci;
extern struct pmx_dev spear1310_pmx_smii_0_1_2;
extern struct pmx_dev spear1310_pmx_ssp1_dis_kbd;
extern struct pmx_dev spear1310_pmx_ssp1_dis_sd;
extern struct pmx_dev spear1310_pmx_gpt64;
extern struct pmx_dev spear1310_pmx_ras_mii_txclk;
extern struct pmx_dev spear1310_pmx_pcie0;
extern struct pmx_dev spear1310_pmx_pcie1;
extern struct pmx_dev spear1310_pmx_pcie2;
extern struct pmx_dev spear1310_pmx_sata0;
extern struct pmx_dev spear1310_pmx_sata1;
extern struct pmx_dev spear1310_pmx_sata2;
#endif

#if defined(CONFIG_CPU_SPEAR1340)
/* pad mux declarations */
/*
 * In SPEAr1340 there are two levels of pad muxing
 * - pads as gpio OR peripherals
 * - If peripherals, then primary OR alternate peripheral.
 */

/*
 * Macro's for first level of pmx - pads as gpio OR peripherals
 * There are 8 registers with 32 bits each for handling gpio pads, register 8
 * has only 26 relevant bits
 */
#define SPEAR1340_PMX_FSMC_16_BIT_AND_KBD_ROW_COL_REG1_MASK	0x00000FFE
#define SPEAR1340_PMX_UART0_ENH_AND_GPT_REG1_MASK		0x0003F000
#define SPEAR1340_PMX_PWM1_AND_KBD_COL5_REG1_MASK		0x00040000
#define SPEAR1340_PMX_I2C1_REG1_MASK				0x01080000
#define SPEAR1340_PMX_SPDIF_IN_REG1_MASK			0x00100000
#define SPEAR1340_PMX_PWM2_AND_GPT0_TMR0_CPT_REG1_MASK		0x00400000
#define SPEAR1340_PMX_PWM3_AND_GPT0_TMR1_CLK_REG1_MASK		0x00800000
#define SPEAR1340_PMX_PWM0_AND_SSP0_CS1_REG1_MASK		0x02000000
#define SPEAR1340_PMX_VIP_AND_CAM3_REG1_MASK			0xFC200000
#define SPEAR1340_PMX_VIP_AND_CAM3_REG2_MASK			0x0000000F
#define SPEAR1340_PMX_VIP_REG2_MASK				0x00001EF0
#define SPEAR1340_PMX_VIP_AND_CAM2_REG2_MASK			0x007FE100
#define SPEAR1340_PMX_VIP_AND_CAM1_REG2_MASK			0xFF800000
#define SPEAR1340_PMX_VIP_AND_CAM1_REG3_MASK			0x00000003
#define SPEAR1340_PMX_VIP_AND_CAM0_REG3_MASK			0x00001FFC
#define SPEAR1340_PMX_SMI_REG3_MASK				0x0021E000
#define SPEAR1340_PMX_SSP0_REG3_MASK				0x001E0000
#define SPEAR1340_PMX_TS_AND_SSP0_CS2_REG3_MASK			0x00400000
#define SPEAR1340_PMX_UART0_REG3_MASK				0x01800000
#define SPEAR1340_PMX_UART1_REG3_MASK				0x06000000
#define SPEAR1340_PMX_I2S_IN_REG3_MASK				0xF8000000
#define SPEAR1340_PMX_DEVS_GRP_AND_MIPHY_DBG_REG4_MASK		0x000001FE
#define SPEAR1340_PMX_I2S_OUT_REG4_MASK				0x000001EF
#define SPEAR1340_PMX_I2S_IN_REG4_MASK				0x00000010
#define SPEAR1340_PMX_GMAC_REG4_MASK				0xFFFFFE00
#define SPEAR1340_PMX_GMAC_REG5_MASK				0x0000001F
#define SPEAR1340_PMX_DEVS_GRP_AND_MIPHY_DBG_REG5_MASK		0x7FFFFF20
#define SPEAR1340_PMX_SSP0_CS3_REG5_MASK			0x00000020
#define SPEAR1340_PMX_I2C0_REG5_MASK				0x000000C0
#define SPEAR1340_PMX_CEC0_REG5_MASK				0x00000100
#define SPEAR1340_PMX_CEC1_REG5_MASK				0x00000200
#define SPEAR1340_PMX_SPDIF_OUT_REG5_MASK			0x00000400
#define SPEAR1340_PMX_CLCD_REG5_MASK				0x7FFFF800
#define SPEAR1340_PMX_CLCD_AND_ARM_TRACE_REG5_MASK		0x80000000
#define SPEAR1340_PMX_CLCD_AND_ARM_TRACE_REG6_MASK		0xFFFFFFFF
#define SPEAR1340_PMX_CLCD_AND_ARM_TRACE_REG7_MASK		0x00000001
#define SPEAR1340_PMX_FSMC_PNOR_AND_MCIF_REG7_MASK		0x073FFFFE
#define SPEAR1340_PMX_MCIF_REG7_MASK				0xF8C00000
#define SPEAR1340_PMX_MCIF_REG8_MASK				0x000043FF
#define SPEAR1340_PMX_FSMC_8BIT_REG8_MASK			0x07FFBC00

/* macro's for making pads as gpio's */
#define SPEAR1340_PMX_PADS_AS_GPIO_REG1_MASK			0xFFFFFFFE
#define SPEAR1340_PMX_PADS_AS_GPIO_REGS_MASK			0xFFFFFFFF
#define SPEAR1340_PMX_PADS_AS_GPIO_REG8_MASK			0x07FFFFFF

/* Macro's for second level of pmx - pads as primary OR alternate peripheral */
/* Write 0 to enable FSMC_16_BIT */
#define SPEAR1340_PMX_KBD_ROW_COL_MASK		(1 << 0)

/* Write 0 to enable UART0_ENH */
#define SPEAR1340_PMX_GPT_MASK			(1 << 1) /* Only clk & cpt */

/* Write 0 to enable PWM1 */
#define SPEAR1340_PMX_KBD_COL5_MASK		(1 << 2)

/* Write 0 to enable PWM2 */
#define SPEAR1340_PMX_GPT0_TMR0_CPT_MASK	(1 << 3) /* Only clk & cpt */

/* Write 0 to enable PWM3 */
#define SPEAR1340_PMX_GPT0_TMR1_CLK_MASK	(1 << 4) /* Only clk & cpt */

/* Write 0 to enable PWM0 */
#define SPEAR1340_PMX_SSP0_CS1_MASK		(1 << 5)

/* Write 0 to enable VIP */
#define SPEAR1340_PMX_CAM3_MASK			(1 << 6)

/* Write 0 to enable VIP */
#define SPEAR1340_PMX_CAM2_MASK			(1 << 7)

/* Write 0 to enable VIP */
#define SPEAR1340_PMX_CAM1_MASK			(1 << 8)

/* Write 0 to enable VIP */
#define SPEAR1340_PMX_CAM0_MASK			(1 << 9)

/* Write 0 to enable TS */
#define SPEAR1340_PMX_SSP0_CS2_MASK		(1 << 10)

/* Write 0 to enable FSMC PNOR */
#define SPEAR1340_PMX_MCIF_MASK			(1 << 11)

/* Write 0 to enable CLCD */
#define SPEAR1340_PMX_ARM_TRACE_MASK		(1 << 12)

/* Write 0 to enable I2S, SSP0_CS2, CEC0, 1, SPDIF out, CLCD */
#define SPEAR1340_PMX_MIPHY_DBG_MASK		(1 << 13)

extern struct pmx_dev clcd_pmx_plgpios;
extern struct pmx_dev spear1340_pmx_pads_as_gpio;
extern struct pmx_dev spear1340_pmx_fsmc_8bit;
extern struct pmx_dev spear1340_pmx_fsmc_16bit;
extern struct pmx_dev spear1340_pmx_keyboard_row_col;
extern struct pmx_dev spear1340_pmx_keyboard_col5;
extern struct pmx_dev spear1340_pmx_uart0_enh;
extern struct pmx_dev spear1340_pmx_i2c1;
extern struct pmx_dev spear1340_pmx_spdif_in;
extern struct pmx_dev spear1340_pmx_gpt_0_1;
extern struct pmx_dev spear1340_pmx_pwm0;
extern struct pmx_dev spear1340_pmx_pwm1;
extern struct pmx_dev spear1340_pmx_pwm2;
extern struct pmx_dev spear1340_pmx_pwm3;
extern struct pmx_dev spear1340_pmx_ssp0_cs1;
extern struct pmx_dev spear1340_pmx_vip_mux_cam0;
extern struct pmx_dev spear1340_pmx_vip_mux_cam1;
extern struct pmx_dev spear1340_pmx_vip_mux_cam2;
extern struct pmx_dev spear1340_pmx_vip_mux_cam3;
extern struct pmx_dev spear1340_pmx_cam0;
extern struct pmx_dev spear1340_pmx_cam1;
extern struct pmx_dev spear1340_pmx_cam2;
extern struct pmx_dev spear1340_pmx_cam3;
extern struct pmx_dev spear1340_pmx_smi;
extern struct pmx_dev spear1340_pmx_ssp0;
extern struct pmx_dev spear1340_pmx_ssp0_cs2;
extern struct pmx_dev spear1340_pmx_uart0;
extern struct pmx_dev spear1340_pmx_uart1;
extern struct pmx_dev spear1340_pmx_i2s_in;
extern struct pmx_dev spear1340_pmx_i2s_out;
extern struct pmx_dev spear1340_pmx_gmac;
extern struct pmx_dev spear1340_pmx_ssp0_cs3;
extern struct pmx_dev spear1340_pmx_i2c0;
extern struct pmx_dev spear1340_pmx_cec0;
extern struct pmx_dev spear1340_pmx_cec1;
extern struct pmx_dev spear1340_pmx_spdif_out;
extern struct pmx_dev spear1340_pmx_fsmc_pnor;
extern struct pmx_dev spear1340_pmx_mcif;
extern struct pmx_dev spear1340_pmx_sdhci;
extern struct pmx_dev spear1340_pmx_cf;
extern struct pmx_dev spear1340_pmx_xd;
extern struct pmx_dev spear1340_pmx_clcd;
extern struct pmx_dev spear1340_pmx_clcd_gpio_pd;
extern struct pmx_dev spear1340_pmx_arm_trace;
extern struct pmx_dev spear1340_pmx_devs_grp;
extern struct pmx_dev spear1340_pmx_miphy_dbg;
extern struct pmx_dev spear1340_pmx_gmii;
extern struct pmx_dev spear1340_pmx_rgmii;
extern struct pmx_dev spear1340_pmx_rmii;
extern struct pmx_dev spear1340_pmx_sgmii;
extern struct pmx_dev spear1340_pmx_pcie;
extern struct pmx_dev spear1340_pmx_sata;
#endif

/*
 * Each GPT has 2 timer channels
 * Following GPT channels will be used as clock source and clockevent
 */
#define SPEAR_GPT0_BASE		SPEAR13XX_GPT0_BASE
#define SPEAR_GPT0_CHAN0_IRQ	SPEAR13XX_IRQ_GPT0_TMR0
#define SPEAR_GPT0_CHAN1_IRQ	SPEAR13XX_IRQ_GPT0_TMR1

/* Add spear13xx family device structure declarations here */
extern struct amba_device spear13xx_gpio_device[];
extern struct amba_device spear13xx_ssp_device;
extern struct amba_device spear13xx_uart_device;
extern struct platform_device spear13xx_adc_device;
extern struct platform_device spear13xx_cf_device;
extern struct platform_device spear13xx_db9000_clcd_device;
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
extern struct platform_device spear13xx_device_gpiokeys;
#endif
extern struct platform_device spear13xx_dmac_device[];
extern struct platform_device spear13xx_ehci0_device;
extern struct platform_device spear13xx_ehci1_device;
extern struct platform_device spear13xx_eth_device;
extern struct platform_device spear13xx_fsmc_nor_device;
extern struct platform_device spear13xx_i2c_device;
extern struct platform_device spear13xx_kbd_device;
extern struct platform_device spear13xx_nand_device;
extern struct platform_device spear13xx_ohci0_device;
extern struct platform_device spear13xx_ohci1_device;
extern struct platform_device spear13xx_pmu_device;
extern struct platform_device spear13xx_pcie_gadget0_device;
extern struct platform_device spear13xx_pcie_host0_device;
extern struct platform_device spear13xx_pcm_device;
extern struct platform_device spear13xx_rtc_device;
extern struct platform_device spear13xx_sdhci_device;
extern struct platform_device spear13xx_smi_device;
extern struct platform_device spear13xx_thermal_device;
extern struct platform_device spear13xx_wdt_device;

#if defined(CONFIG_CPU_SPEAR1300) || defined(CONFIG_CPU_SPEAR1310_REVA) || \
			defined(CONFIG_CPU_SPEAR900) || \
			defined(CONFIG_CPU_SPEAR1310)
extern struct platform_device spear13xx_cpufreq_device;
extern struct platform_device spear13xx_i2s0_device;
extern struct platform_device spear13xx_i2s1_device;
extern struct platform_device spear13xx_jpeg_device;
extern struct platform_device spear13xx_pcie_gadget1_device;
extern struct platform_device spear13xx_pcie_gadget2_device;
extern struct platform_device spear13xx_pcie_host1_device;
extern struct platform_device spear13xx_pcie_host2_device;
#endif

#if defined(CONFIG_CPU_SPEAR1300) || defined(CONFIG_CPU_SPEAR1310_REVA) || \
			defined(CONFIG_CPU_SPEAR900)
extern struct platform_device spear13xx_udc_device;
#endif

extern struct sys_timer spear13xx_timer;

/* Add spear13xx structure declarations here */
extern struct dw_dma_slave cf_dma_priv;
extern struct dw_dma_slave nand_read_dma_priv;
extern struct dw_dma_slave nand_write_dma_priv;

/* Add spear13xx family function declarations here */
int audio_clk_config(struct i2s_clk_config_data *config);
bool dw_dma_filter(struct dma_chan *chan, void *slave);
void __init spear13xx_clk_init(void);
void __init i2c_register_default_devices(void);
void spear_restart(char, const char *);
void __init spear_setup_timer(void);
void __init spear13xx_map_io(void);
void __init spear13xx_init_irq(void);
void __init spear13xx_init(void);
void __init nand_mach_init(u32 busw);
void nand_select_bank(u32 bank, u32 busw);
void spear13xx_l2x0_init(void);
int spear13xx_eth_phy_clk_cfg(struct platform_device *);
void spear13xx_secondary_startup(void);
void spear13xx_panel_fixup(struct meminfo *mi);
void spear13xx_panel_init(struct platform_device *pdev);
unsigned long reserve_mem(struct meminfo *mi, unsigned long size);

/* spear1300 declarations */
#ifdef CONFIG_CPU_SPEAR1300
/* Add spear1300 machine function declarations here */
void __init spear1300_init(struct pmx_mode *pmx_mode, struct pmx_dev **pmx_devs,
		u8 pmx_dev_count);

#endif /* CONFIG_CPU_SPEAR1300 */

/* spear1310_reva declarations */
#ifdef CONFIG_CPU_SPEAR1310_REVA
/* Add spear1310_reva machine device structure declarations here */
extern struct amba_device spear1310_reva_uart1_device;
extern struct amba_device spear1310_reva_uart2_device;
extern struct amba_device spear1310_reva_uart3_device;
extern struct amba_device spear1310_reva_uart4_device;
extern struct amba_device spear1310_reva_uart5_device;
extern struct platform_device spear1310_reva_can0_device;
extern struct platform_device spear1310_reva_can1_device;
extern struct platform_device spear1310_reva_eth1_device;
extern struct platform_device spear1310_reva_eth2_device;
extern struct platform_device spear1310_reva_eth3_device;
extern struct platform_device spear1310_reva_eth4_device;
extern struct platform_device spear1310_reva_i2c1_device;
extern struct platform_device spear1310_reva_plgpio_device;
extern struct platform_device spear1310_reva_ras_fsmc_nor_device;
extern struct platform_device spear1310_reva_rs485_0_device;
extern struct platform_device spear1310_reva_rs485_1_device;
extern struct platform_device spear1310_reva_tdm_hdlc_0_device;
extern struct platform_device spear1310_reva_tdm_hdlc_1_device;

/* Add spear1310_reva machine function declarations here */
void __init spear1310_reva_init(struct pmx_mode *pmx_mode,
		struct pmx_dev **pmx_devs, u8 pmx_dev_count);
void __init spear1310_reva_map_io(void);
int spear1310_reva_eth_phy_clk_cfg(struct platform_device *);

#endif /* CONFIG_CPU_SPEAR1310_REVA */

/* spear1310 declarations */
#ifdef CONFIG_CPU_SPEAR1310
/* Add spear1310 machine device structure declarations here */
extern struct amba_device spear1310_ssp1_device;
extern struct amba_device spear1310_uart1_device;
extern struct amba_device spear1310_uart2_device;
extern struct amba_device spear1310_uart3_device;
extern struct amba_device spear1310_uart4_device;
extern struct amba_device spear1310_uart5_device;
extern struct platform_device spear1310_can0_device;
extern struct platform_device spear1310_can1_device;
extern struct platform_device spear1310_i2c1_device;
extern struct platform_device spear1310_i2c2_device;
extern struct platform_device spear1310_i2c3_device;
extern struct platform_device spear1310_i2c4_device;
extern struct platform_device spear1310_i2c5_device;
extern struct platform_device spear1310_i2c6_device;
extern struct platform_device spear1310_i2c7_device;
extern struct platform_device spear1310_nand_device;
extern struct platform_device spear1310_otg_device;
extern struct platform_device spear1310_plgpio_device;
extern struct platform_device spear1310_rs485_0_device;
extern struct platform_device spear1310_rs485_1_device;
extern struct platform_device spear1310_sata0_device;
extern struct platform_device spear1310_sata1_device;
extern struct platform_device spear1310_sata2_device;
extern struct platform_device spear1310_tdm_hdlc_0_device;
extern struct platform_device spear1310_tdm_hdlc_1_device;

/* Add spear1310 machine function declarations here */
void __init spear1310_clk_init(void);
void __init spear1310_init(struct pmx_mode *pmx_mode,
		struct pmx_dev **pmx_devs, u8 pmx_dev_count);
void __init spear1310_map_io(void);
void spear1310_nand_select_bank(u32 bank, u32 busw);

#endif /* CONFIG_CPU_SPEAR1310 */

/* spear1340 declarations */
#ifdef CONFIG_CPU_SPEAR1340
/* Add spear1340 machine device structure declarations here */
extern struct amba_device spear1340_uart1_device;
extern struct platform_device spear1340_camif0_device;
extern struct platform_device spear1340_camif1_device;
extern struct platform_device spear1340_camif2_device;
extern struct platform_device spear1340_camif3_device;
extern struct platform_device spear1340_cec0_device;
extern struct platform_device spear1340_cec1_device;
extern struct platform_device spear1340_cpufreq_device;
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
extern struct platform_device spear1340_gpiokeys_device;
#endif
extern struct platform_device spear1340_i2c1_device;
extern struct platform_device spear1340_i2s_play_device;
extern struct platform_device spear1340_i2s_record_device;
extern struct platform_device spear1340_nand_device;
extern struct platform_device spear1340_otg_device;
extern struct platform_device spear1340_phy0_device;
extern struct platform_device spear1340_plgpio_device;
extern struct platform_device spear1340_pwm_device;
extern struct platform_device spear1340_sata0_device;
extern struct platform_device spear1340_spdif_in_device;
extern struct platform_device spear1340_spdif_out_device;
extern struct platform_device spear1340_thermal_device;
extern struct platform_device spear1340_vip_device;
extern struct platform_device spear1340_device_mali_drm;
extern struct platform_device spear1340_video_dec_device;

/* Add spear1340 spi devices structure declaration */
extern struct spi_board_info spear1340_evb_spi_m25p80;
extern struct spi_board_info spear1340_evb_spi_stmpe610;
extern struct spi_board_info spear1340_evb_spi_spidev;

/* Add spear1340 i2c devices structure declaration */
extern struct i2c_dev_info spear1340_evb_i2c_l3g4200d_gyr;
extern struct i2c_dev_info spear1340_evb_i2c_eeprom0;
extern struct i2c_dev_info spear1340_evb_i2c_eeprom1;
extern struct i2c_dev_info spear1340_evb_i2c_sta529;

int __init spear1340_pb_init(struct plug_board_info *pb_info);

/* Add spear1340 machine function declarations here */
void __init spear1340_clk_init(void);
void __init spear1340_init(struct pmx_mode *pmx_mode, struct pmx_dev **pmx_devs,
		u8 pmx_dev_count);
void __init spear1340_map_io(void);
void __init spear1340_pcie_board_init(struct device *dev);
void config_clcd_gpio_pads(bool);

#endif /* CONFIG_CPU_SPEAR1340 */

/* spear900 declarations */
#ifdef CONFIG_CPU_SPEAR900
/* Add spear900 machine function declarations here */
void __init spear900_init(struct pmx_mode *pmx_mode, struct pmx_dev **pmx_devs,
		u8 pmx_dev_count);

#endif /* CONFIG_CPU_SPEAR900 */

#endif /* __MACH_GENERIC_H */
