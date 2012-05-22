/*
 * arch/arm/mach-spear13xx/include/mach/misc_regs.h
 *
 * Miscellaneous registers definitions for spear13xx machine family
 *
 * Copyright (C) 2010 ST Microelectronics
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_MISC_REGS_H
#define __MACH_MISC_REGS_H

#include <mach/hardware.h>

#ifdef CONFIG_CPU_SPEAR1340
#include <mach/spear1340_misc_regs.h>
#undef VA_MISC_BASE
#endif

#ifdef CONFIG_CPU_SPEAR1310
#include <mach/spear1310_misc_regs.h>
#undef VA_MISC_BASE
#endif

#define VA_MISC_BASE			IOMEM(VA_SPEAR13XX_MISC_BASE)

/* General Configuration */
#define VA_SOC_CFG			(VA_MISC_BASE + 0x000)
#define VA_BOOTSTRAP_CFG		(VA_MISC_BASE + 0x004)

/* Power Management Registers */
#define VA_PCM_CFG			(VA_MISC_BASE + 0x100)
#define PCM_CFG				(SPEAR13XX_MISC_BASE + 0x100)
#define VA_PCM_WKUP_CFG			(VA_MISC_BASE + 0x104)
#define VA_SWITCH_CTR			(VA_MISC_BASE + 0x108)

/* Clock Configuration Registers */
#define VA_SYS_CLK_CTRL			(VA_MISC_BASE + 0x200)
	#define SYS_MODE_MASK			(0x7 << 0)
	#define SYS_MODE_DOZE			(0x1 << 0)
	#define SYS_MODE_SLOW			(0x2 << 0)
	#define SYS_MODE_NORMAL			(0x4 << 0)

	#define SYS_MODE_STS_MASK		(0xF << 16)
	#define SYS_MODE_STS_DOZE		(0x0 << 16)
	#define SYS_MODE_STS_SLOW		(0xA << 16)
	#define SYS_MODE_STS_NORMAL		(0xF << 16)

#define VA_SYS_SW_RES			(VA_MISC_BASE + 0x204)
#define VA_SYS_CLK_PLLTIMER		(VA_MISC_BASE + 0x208)
#define VA_SYS_CLK_OSCITIMER		(VA_MISC_BASE + 0x20C)

/* PLL related registers and bit values */
#define VA_PLL_CFG			(VA_MISC_BASE + 0x210)
	/* PLL_CFG bit values */
	#define CLCD_SYNT_VCO1_DIV4_VAL		0
	#define CLCD_SYNT_PLL2_VAL		1
	#define CLCD_SYNT_CLK_MASK		1
	#define CLCD_SYNT_CLK_SHIFT		31
	#define RAS_SYNT2_3_VCO1_DIV4_VAL	0
	#define RAS_SYNT2_3_VCO2_DIV2_VAL	1
	#define RAS_SYNT2_3_PLL2_VAL		2
	#define RAS_SYNT2_3_CLK_MASK		3
	#define RAS_SYNT2_3_CLK_SHIFT		29
	#define RAS_SYNT0_1_VCO1_DIV4_VAL	0
	#define RAS_SYNT0_1_VCO3_DIV2_VAL	1
	#define RAS_SYNT0_1_PLL3_VAL		2
	#define RAS_SYNT0_1_CLK_MASK		3
	#define RAS_SYNT0_1_CLK_SHIFT		27
	#define OSC_24M_VAL			0
	#define OSC_25M_VAL			1
	#define PLL_CLK_MASK			3
	#define PLL3_CLK_SHIFT			24
	#define PLL2_CLK_SHIFT			22
	#define PLL1_CLK_SHIFT			20

#define VA_PLL1_CTR			(VA_MISC_BASE + 0x214)
#define VA_PLL1_FRQ			(VA_MISC_BASE + 0x218)
#define VA_PLL1_MOD			(VA_MISC_BASE + 0x21C)
#define VA_PLL2_CTR			(VA_MISC_BASE + 0x220)
#define VA_PLL2_FRQ			(VA_MISC_BASE + 0x224)
#define VA_PLL2_MOD			(VA_MISC_BASE + 0x228)
#define VA_PLL3_CTR			(VA_MISC_BASE + 0x22C)
#define VA_PLL3_FRQ			(VA_MISC_BASE + 0x230)
#define VA_PLL3_MOD			(VA_MISC_BASE + 0x234)
#define VA_PLL4_CTR			(VA_MISC_BASE + 0x238)
	/* PLL_CTR register masks */
	#define PLL_MODE_NORMAL		0
	#define PLL_MODE_FRACTION	1
	#define PLL_MODE_DITH_DSM	2
	#define PLL_MODE_DITH_SSM	3
	#define PLL_MODE_MASK		3
	#define PLL_MODE_SHIFT		3
	#define PLL_ENABLE		1

	#define PLL_LOCK_SHIFT		0
	#define PLL_LOCK_MASK		1

#define VA_PLL4_FRQ			(VA_MISC_BASE + 0x23C)
	/* PLL FRQ register masks */
	#define PLL_NORM_FDBK_M_MASK	0xFF
	#define PLL_NORM_FDBK_M_SHIFT	24
	#define PLL_DITH_FDBK_M_MASK	0xFFFF
	#define PLL_DITH_FDBK_M_SHIFT	16
	#define PLL_DIV_P_MASK		0x7
	#define PLL_DIV_P_SHIFT		8
	#define PLL_DIV_N_MASK		0xFF
	#define PLL_DIV_N_SHIFT		0

#define VA_PLL4_MOD			(VA_MISC_BASE + 0x240)
#define VA_PERIP_CLK_CFG		(VA_MISC_BASE + 0x244)
	/* PERIP_CLK_CFG bit values */
	#define GPT3_CLK_SHIFT		13
	#define GPT2_CLK_SHIFT		12
	#define MCTR_CLK_PLL1_VAL	0x0
	#define MCTR_CLK_PLL4_VAL	0x1
	#define MCTR_CLK_MASK		0x1
	#define MCTR_CLK_SHIFT		10
	#define GPT_APB_VAL		1
	#define GPT_OSC24_VAL		0
	#define GPT_CLK_MASK		1
	#define GPT1_CLK_SHIFT		9
	#define GPT0_CLK_SHIFT		8

	#define AUX_CLK_PLL5_VAL	0
	#define AUX_CLK_SYNT_VAL	1
	#define UART_CLK_MASK		1
	#define UART_CLK_SHIFT		4
	#define CLCD_CLK_MASK		3
	#define CLCD_CLK_SHIFT		2
	#define C3_CLK_MASK		1
	#define C3_CLK_SHIFT		1

#define VA_GMAC_CLK_CFG			(VA_MISC_BASE + 0x248)
	#define GMAC_PHY_IF_GMII_VAL		0
	#define GMAC_PHY_IF_RGMII_VAL		1
	#define GMAC_PHY_IF_RMII_VAL		4
	#define GMAC_PHY_IF_SEL_MASK		7
	#define GMAC_PHY_IF_SEL_SHIFT		4
	#define GMAC_PHY_INPUT_ENB_VAL		0
	#define GMAC_PHY_SYNT_ENB_VAL		1
	#define GMAC_PHY_CLK_MASK		1
	#define GMAC_PHY_CLK_SHIFT		3
	#define GMAC_PHY_125M_PAD_VAL		0
	#define GMAC_PHY_PLL2_VAL		1
	#define GMAC_PHY_OSC3_VAL		2
	#define GMAC_PHY_INPUT_CLK_MASK		3
	#define GMAC_PHY_INPUT_CLK_SHIFT	1

#define VA_C3_CLK_SYNT			(VA_MISC_BASE + 0x24C)
	/* refer AUX_* macros for reg masks */
#define VA_CLCD_CLK_SYNT		(VA_MISC_BASE + 0x250)
	/* Fractional synthesizer reg masks */
	#define FRAC_SYNT_DIV_FACTOR_MASK	0x1FFFF
	#define FRAC_SYNT_DIV_FACTOR_SHIFT	0

#define VA_UART_CLK_SYNT		(VA_MISC_BASE + 0x254)
#define VA_GMAC_CLK_SYNT		(VA_MISC_BASE + 0x258)
#define VA_SDHCI_CLK_SYNT		(VA_MISC_BASE + 0x25C)
#define VA_CFXD_CLK_SYNT		(VA_MISC_BASE + 0x260)
	/* aux clk synthesizer register masks */
	#define AUX_SYNT_ENB		31
	#define AUX_EQ_SEL_SHIFT	30
	#define AUX_EQ_SEL_MASK		1
	#define AUX_EQ1_SEL		0
	#define AUX_EQ2_SEL		1
	#define AUX_XSCALE_SHIFT	16
	#define AUX_XSCALE_MASK		0xFFF
	#define AUX_YSCALE_SHIFT	0
	#define AUX_YSCALE_MASK		0xFFF

#define VA_RAS_CLK_SYNT0		(VA_MISC_BASE + 0x264)
#define VA_RAS_CLK_SYNT1		(VA_MISC_BASE + 0x268)
#define VA_RAS_CLK_SYNT2		(VA_MISC_BASE + 0x26C)
#define VA_RAS_CLK_SYNT3		(VA_MISC_BASE + 0x270)
	/* Check Fractional synthesizer reg masks */

#define VA_PERIP1_CLK_ENB		(VA_MISC_BASE + 0x274)
	/* PERIP1_CLK_ENB register masks */
	#define RTC_CLK_ENB		31
	#define ADC_CLK_ENB		30
	#define C3_CLK_ENB		29
	#define JPEG_CLK_ENB		28
	#define CLCD_CLK_ENB		27
	#define DMA_CLK_ENB		25
	#define GPIO1_CLK_ENB		24
	#define GPIO0_CLK_ENB		23
	#define GPT1_CLK_ENB		22
	#define GPT0_CLK_ENB		21
	#define I2S0_CLK_ENB		20
	#define I2S1_CLK_ENB		19
	#define I2C_CLK_ENB		18
	#define SSP_CLK_ENB		17
	#define UART_CLK_ENB		15
	#define PCIE2_CLK_ENB		14
	#define PCIE1_CLK_ENB		13
	#define PCIE0_CLK_ENB		12
	#define USBD_CLK_ENB		11
	#define UHC1_CLK_ENB		10
	#define UHC0_CLK_ENB		9
	#define GMAC_CLK_ENB		8
	#define CFXD_CLK_ENB		7
	#define SDHCI_CLK_ENB		6
	#define SMI_CLK_ENB		5
	#define FSMC_CLK_ENB		4
	#define SYSRAM0_CLK_ENB		3
	#define SYSRAM1_CLK_ENB		2
	#define SYSROM_CLK_ENB		1
	#define BUS_CLK_ENB		0

#define VA_PERIP2_CLK_ENB		(VA_MISC_BASE + 0x278)
	/* PERIP2_CLK_ENB register masks */
	#define THSENS_CLK_ENB		8
	#define I2S_REF_PAD_CLK_ENB	7
	#define ACP_CLK_ENB		6
	#define GPT3_CLK_ENB		5
	#define GPT2_CLK_ENB		4
	#define KBD_CLK_ENB		3
	#define CPU_DBG_CLK_ENB		2
	#define DDR_CORE_CLK_ENB	1
	#define DDR_CTRL_CLK_ENB	0

#define VA_PERIP1_SW_RST		(VA_MISC_BASE + 0x27C)
	#define JPEG_SOF_RST		28
#define VA_PERIP2_SW_RST		(VA_MISC_BASE + 0x280)
#define VA_RAS_CLK_ENB			(VA_MISC_BASE + 0x284)
	/* RAS_CLK_ENB register masks */
	#define SYNT3_CLK_ENB		17
	#define SYNT2_CLK_ENB		16
	#define SYNT1_CLK_ENB		15
	#define SYNT0_CLK_ENB		14
	#define PCLK3_CLK_ENB		13
	#define PCLK2_CLK_ENB		12
	#define PCLK1_CLK_ENB		11
	#define PCLK0_CLK_ENB		10
	#define PLL3_CLK_ENB		9
	#define PLL2_CLK_ENB		8
	#define C125M_PAD_CLK_ENB	7
	#define C30M_CLK_ENB		6
	#define C48M_CLK_ENB		5
	#define OSC3_CLK_ENB		4
	#define OSC2_CLK_ENB		3
	#define OSC1_CLK_ENB		2
	#define PCLK_CLK_ENB		1
	#define ACLK_CLK_ENB		0
#define VA_RAS_SW_RST			(VA_MISC_BASE + 0x288)
#define VA_PLL1_SYNT			(VA_MISC_BASE + 0x28C)
	/* Check Fractional synthesizer reg masks */

#define VA_I2S_CLK_CFG			(VA_MISC_BASE + 0x290)
	/* I2S_CLK_CFG register mask */
	#define I2S_SCLK_X_MASK		0x1F
	#define I2S_SCLK_X_SHIFT	27
	#define I2S_SCLK_Y_MASK		0x1F
	#define I2S_SCLK_Y_SHIFT	22
	#define I2S_SCLK_EQ_SEL_SHIFT	21
	#define I2S_SCLK_SYNTH_ENB	20
	#define I2S_PRS1_CLK_X_MASK	0xFF
	#define I2S_PRS1_CLK_X_SHIFT	12
	#define I2S_PRS1_CLK_Y_MASK	0xFF
	#define I2S_PRS1_CLK_Y_SHIFT	4
	#define I2S_PRS1_EQ_SEL_SHIFT	3
	#define I2S_REF_SRC_VAL		0
	#define I2S_REF_PRS1_VAL	1
	#define I2S_REF_SEL_MASK	1
	#define I2S_REF_SHIFT		2
	#define I2S_SRC_VCODIV2_VAL	0
	#define I2S_SRC_PLL3_VAL	1
	#define I2S_SRC_PL_CLK1_VAL	2
	#define I2S_SRC_CLK_MASK	3
	#define I2S_SRC_CLK_SHIFT	0

/* Peripheral Configuration Registers */
#define VA_DMAC_HS_SEL			(VA_MISC_BASE + 0x300)
#define VA_DMAC_SEL			(VA_MISC_BASE + 0x304)
#define VA_DMAC_FLOW_SEL		(VA_MISC_BASE + 0x308)
#define VA_DMAC_DIR_SEL			(VA_MISC_BASE + 0x30C)
#define VA_DMAC_CFG			(VA_MISC_BASE + 0x310)
#define VA_USBPHY_GEN_CFG		(VA_MISC_BASE + 0x314)
#define VA_USBPHY_P1_CFG		(VA_MISC_BASE + 0x318)
#define VA_USBPHY_P2_CFG		(VA_MISC_BASE + 0x31C)
#define VA_USBPHY_P3_CFG		(VA_MISC_BASE + 0x320)
#define VA_PCIE_CFG			(VA_MISC_BASE + 0x324)
	/* PCIE CFG MASks */
	#define PCIE0_CFG_DEVICE_PRESENT	(1 << 11)
	#define PCIE1_CFG_DEVICE_PRESENT	(1 << 10)
	#define PCIE2_CFG_DEVICE_PRESENT	(1 << 9)
	#define PCIE0_CFG_POWERUP_RESET	(1 << 8)
	#define PCIE1_CFG_POWERUP_RESET	(1 << 7)
	#define PCIE2_CFG_POWERUP_RESET	(1 << 6)
	#define PCIE0_CFG_CORE_CLK_EN	(1 << 5)
	#define PCIE1_CFG_CORE_CLK_EN	(1 << 4)
	#define PCIE2_CFG_CORE_CLK_EN	(1 << 3)
	#define PCIE0_CFG_AUX_CLK_EN	(1 << 2)
	#define PCIE1_CFG_AUX_CLK_EN	(1 << 1)
	#define PCIE2_CFG_AUX_CLK_EN	(1 << 0)
	#define PCIE0_CFG_VAL	(PCIE0_CFG_AUX_CLK_EN | PCIE0_CFG_CORE_CLK_EN \
			| PCIE0_CFG_POWERUP_RESET | PCIE0_CFG_DEVICE_PRESENT)
	#define PCIE1_CFG_VAL	(PCIE1_CFG_AUX_CLK_EN | PCIE1_CFG_CORE_CLK_EN \
			| PCIE1_CFG_POWERUP_RESET | PCIE1_CFG_DEVICE_PRESENT)
	#define PCIE2_CFG_VAL	(PCIE2_CFG_AUX_CLK_EN | PCIE2_CFG_CORE_CLK_EN \
			| PCIE2_CFG_POWERUP_RESET | PCIE2_CFG_DEVICE_PRESENT)

#define VA_PCIE_MIPHY_CFG		(VA_MISC_BASE + 0x328)
#define VA_PERIP_CFG			(VA_MISC_BASE + 0x32C)
#define PERIP_CFG		(SPEAR13XX_MISC_BASE + 0x32C)
	/* PERIP_CFG register masks */
	#define MCIF_SEL_SD	(0x1 << MCIF_SEL_SHIFT)
	#define MCIF_SEL_CF	(0x2 << MCIF_SEL_SHIFT)
	#define MCIF_SEL_XD	(0x3 << MCIF_SEL_SHIFT)
	#define MCIF_SEL_MASK	(0x3 << MCIF_SEL_SHIFT)
	#define MCIF_SEL_SHIFT	3

	#define I2S_MODE_I2S2_ONE_PORT	(0 << 0)
	#define I2S_MODE_I2S1_ONE_PORT	(1 << 0)
	#define I2S_MODE_I2S2_TWO_PORT	(2 << 0)
	#define I2S_MODE_I2S1_TWO_PORT	(3 << 0)
	#define I2S_MODE_BOTH_ONE_PORT	(4 << 0)
	#define I2S_MODE_MASK		(7 << 0)

#define VA_FSMC_CFG			(VA_MISC_BASE + 0x330)
	/* FSMC_CFG register masks */
	#define NAND_DEV_WIDTH16	4
	#define NAND_BANK_MASK		3
	#define NAND_BANK_SHIFT		2
	#define FSMC_MEM_NOR		0
	#define FSMC_MEM_NAND		1
	#define FSMC_MEM_SRAM		2
	#define FSMC_MEMSEL_MASK	3
	#define FSMC_MEMSEL_SHIFT	0

#define VA_MPMC_CTR_STS			(VA_MISC_BASE + 0x334)

/* Inter-Processor Communication Registers */
#define VA_PRC1_LOCK_CTR		(VA_MISC_BASE + 0x500)
#define VA_PRC2_LOCK_CTR		(VA_MISC_BASE + 0x504)
#define VA_PRC1_IRQ_CTR			(VA_MISC_BASE + 0x508)
#define VA_PRC2_IRQ_CTR			(VA_MISC_BASE + 0x51C)

/* Pad Configuration Registers */
#define VA_PAD_PU_CFG_1			(VA_MISC_BASE + 0x600)
#define VA_PAD_PU_CFG_2			(VA_MISC_BASE + 0x604)
#define VA_PAD_PU_CFG_3			(VA_MISC_BASE + 0x608)
#define VA_PAD_PU_CFG_4			(VA_MISC_BASE + 0x60C)
#define VA_PAD_PU_CFG_5			(VA_MISC_BASE + 0x610)
#define VA_PAD_PU_CFG_6			(VA_MISC_BASE + 0x614)
#define VA_PAD_PU_CFG_7			(VA_MISC_BASE + 0x618)
#define VA_PAD_PU_CFG_8			(VA_MISC_BASE + 0x61C)
#define VA_PAD_PD_CFG_1			(VA_MISC_BASE + 0x620)
#define VA_PAD_PD_CFG_2			(VA_MISC_BASE + 0x624)
#define VA_PAD_PD_CFG_3			(VA_MISC_BASE + 0x628)
#define VA_PAD_PD_CFG_4			(VA_MISC_BASE + 0x62C)
#define VA_PAD_PD_CFG_5			(VA_MISC_BASE + 0x630)
#define VA_PAD_PD_CFG_6			(VA_MISC_BASE + 0x634)
#define VA_PAD_PD_CFG_7			(VA_MISC_BASE + 0x638)
#define VA_PAD_PD_CFG_8			(VA_MISC_BASE + 0x63C)
#define VA_PAD_SLEEP_CFG		(VA_MISC_BASE + 0x640)
#define VA_PAD_HYST_CFG			(VA_MISC_BASE + 0x644)
#define VA_PAD_DRV_CFG			(VA_MISC_BASE + 0x648)
#define VA_PAD_SLEW_CFG			(VA_MISC_BASE + 0x64C)
#define VA_PAD_FUNCTION_EN_1		(VA_MISC_BASE + 0x650)
#define PAD_FUNCTION_EN_1		(SPEAR13XX_MISC_BASE + 0x650)
#define VA_PAD_FUNCTION_EN_2		(VA_MISC_BASE + 0x654)
#define PAD_FUNCTION_EN_2		(SPEAR13XX_MISC_BASE + 0x654)
#define VA_PAD_FUNCTION_EN_3		(VA_MISC_BASE + 0x658)
#define PAD_FUNCTION_EN_3		(SPEAR13XX_MISC_BASE + 0x658)
#define VA_DDR_PAD_CFG			(VA_MISC_BASE + 0x65C)
#define THSENS_CFG			(SPEAR13XX_MISC_BASE + 0x6C4)
#define VA_THSENS_CFG			(VA_MISC_BASE + 0x6C4)
	#define THERMAL_CONFIG_FLAGS		0x7000

/* Compensation Configuration Registers */
#define VA_COMP_1V8_2V5_3V3__1_CFG	(VA_MISC_BASE + 0x700)
#define VA_COMP_1V8_2V5_3V3__2_CFG	(VA_MISC_BASE + 0x704)
#define VA_COMP_3V3_1_CFG		(VA_MISC_BASE + 0x708)
#define VA_COMP_3V3_2_CFG		(VA_MISC_BASE + 0x70C)
#define VA_COMP_DDR_CFG			(VA_MISC_BASE + 0x710)

/* OTP Programming Registers */
#define VA_OTP_PROG_CTR			(VA_MISC_BASE + 0x800)
#define VA_OTP_WDATA1_1			(VA_MISC_BASE + 0x804)
#define VA_OTP_WDATA1_2			(VA_MISC_BASE + 0x808)
#define VA_OTP_WDATA1_3			(VA_MISC_BASE + 0x80C)
#define VA_OTP_WDATA1_4			(VA_MISC_BASE + 0x810)
#define VA_OTP_WDATA1_5			(VA_MISC_BASE + 0x814)
#define VA_OTP_WDATA1_6			(VA_MISC_BASE + 0x818)
#define VA_OTP_WDATA1_7			(VA_MISC_BASE + 0x81C)
#define VA_OTP_WDATA1_8			(VA_MISC_BASE + 0x820)
#define VA_OTP_WDATA2_1			(VA_MISC_BASE + 0x824)
#define VA_OTP_WDATA2_2			(VA_MISC_BASE + 0x828)
#define VA_OTP_WDATA2_3			(VA_MISC_BASE + 0x82C)
#define VA_OTP_WDATA2_4			(VA_MISC_BASE + 0x830)
#define VA_OTP_WDATA2_5			(VA_MISC_BASE + 0x834)
#define VA_OTP_WDATA2_6			(VA_MISC_BASE + 0x838)
#define VA_OTP_WDATA2_7			(VA_MISC_BASE + 0x83C)
#define VA_OTP_WDATA2_8			(VA_MISC_BASE + 0x840)
#define VA_OTP_MASK_1			(VA_MISC_BASE + 0x844)
#define VA_OTP_MASK_2			(VA_MISC_BASE + 0x848)
#define VA_OTP_MASK_3			(VA_MISC_BASE + 0x84C)
#define VA_OTP_MASK_4			(VA_MISC_BASE + 0x850)
#define VA_OTP_MASK_5			(VA_MISC_BASE + 0x854)
#define VA_OTP_MASK_6			(VA_MISC_BASE + 0x858)
#define VA_OTP_MASK_7			(VA_MISC_BASE + 0x85C)
#define VA_OTP_MASK_8			(VA_MISC_BASE + 0x860)
#define VA_OTP_RDATA1_1			(VA_MISC_BASE + 0x864)
#define VA_OTP_RDATA1_2			(VA_MISC_BASE + 0x868)
#define VA_OTP_RDATA1_3			(VA_MISC_BASE + 0x86C)
#define VA_OTP_RDATA1_4			(VA_MISC_BASE + 0x870)
#define VA_OTP_RDATA1_5			(VA_MISC_BASE + 0x874)
#define VA_OTP_RDATA1_6			(VA_MISC_BASE + 0x878)
#define VA_OTP_RDATA1_7			(VA_MISC_BASE + 0x87C)
#define VA_OTP_RDATA1_8			(VA_MISC_BASE + 0x880)
#define VA_OTP_RDATA2_1			(VA_MISC_BASE + 0x884)
#define VA_OTP_RDATA2_2			(VA_MISC_BASE + 0x888)
#define VA_OTP_RDATA2_3			(VA_MISC_BASE + 0x88C)
#define VA_OTP_RDATA2_4			(VA_MISC_BASE + 0x890)
#define VA_OTP_RDATA2_5			(VA_MISC_BASE + 0x894)
#define VA_OTP_RDATA2_6			(VA_MISC_BASE + 0x898)
#define VA_OTP_RDATA2_7			(VA_MISC_BASE + 0x89C)
#define VA_OTP_RDATA2_8			(VA_MISC_BASE + 0x8A0)
#define VA_OTP_RDATAM_1			(VA_MISC_BASE + 0x8A4)
#define VA_OTP_RDATAM_2			(VA_MISC_BASE + 0x8A8)
#define VA_OTP_RDATAM_3			(VA_MISC_BASE + 0x8AC)
#define VA_OTP_RDATAM_4			(VA_MISC_BASE + 0x8B0)
#define VA_OTP_RDATAM_5			(VA_MISC_BASE + 0x8B4)
#define VA_OTP_RDATAM_6			(VA_MISC_BASE + 0x8B8)
#define VA_OTP_RDATAM_7			(VA_MISC_BASE + 0x8BC)
#define VA_OTP_RDATAM_8			(VA_MISC_BASE + 0x8C0)

/* A9SM Registers */
#define VA_A9SM_CLUSTERID		(VA_MISC_BASE + 0x900)
#define VA_A9SM_STATUS			(VA_MISC_BASE + 0x904)
#define VA_A9SM_DEBUG			(VA_MISC_BASE + 0x908)
#define VA_A9SM_FILTER			(VA_MISC_BASE + 0x90C)
#define VA_A9SM_PARITY_CFG		(VA_MISC_BASE + 0x910)
#define VA_A9SM_PARITY_ERR		(VA_MISC_BASE + 0x914)

/* SOC ID Registers */
#define VA_DIE_ID_1			(VA_MISC_BASE + 0xA00)
#define VA_DIE_ID_2			(VA_MISC_BASE + 0xA04)
#define VA_DIE_ID_3			(VA_MISC_BASE + 0xA08)
#define VA_DIE_ID_4			(VA_MISC_BASE + 0xA0C)
#define VA_DIE_ID_VALID			(VA_MISC_BASE + 0xA10)

/* SOC TEST & DEBUG Registers */
#define VA_MIPHY_TEST			(VA_MISC_BASE + 0x1000)
#define VA_PCIE_MSTR_P0			(VA_MISC_BASE + 0x1004)
#define VA_PCIE_AWMISC_P0		(VA_MISC_BASE + 0x1008)
#define VA_PCIE_ARMISC_P0		(VA_MISC_BASE + 0x100C)
#define VA_PCIE_MSTR_P1			(VA_MISC_BASE + 0x1010)
#define VA_PCIE_AWMISC_P1		(VA_MISC_BASE + 0x1014)
#define VA_PCIE_ARMISC_P1		(VA_MISC_BASE + 0x1018)
#define VA_PCIE_MSTR_P2			(VA_MISC_BASE + 0x101C)
#define VA_PCIE_AWMISC_P2		(VA_MISC_BASE + 0x1020)
#define VA_PCIE_ARMISC_P2		(VA_MISC_BASE + 0x1024)

#endif /* __MACH_MISC_REGS_H */
