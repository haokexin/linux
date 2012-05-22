/*
 * arch/arm/mach-spear13xx/spear1340.c
 *
 * SPEAr1340 machine source file
 *
 * Copyright (C) 2011 ST Microelectronics
 * Viresh Kumar <viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/ahci_platform.h>
#include <linux/amba/serial.h>
#include <linux/delay.h>
#include <linux/designware_i2s.h>
#include <linux/dw_dmac.h>
#include <linux/gpio.h>

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#include <linux/gpio_keys.h>
#include <linux/input.h>
#endif

#include <linux/i2c/i2c-designware.h>
#include <linux/irq.h>
#include <linux/mtd/fsmc.h>
#include <linux/platform_data/spear_thermal.h>
#include <linux/usb/dwc_otg.h>
#include <plat/camif.h>
#include <plat/clock.h>
#include <plat/cpufreq.h>
#include <mach/dma.h>
#include <mach/generic.h>
#include <mach/hardware.h>
#include <mach/spdif.h>
#include <mach/spear1340_misc_regs.h>
#include <mach/spear_pcie.h>
#include <media/vip.h>
#include <sound/pcm.h>

/* pmx driver structure */
static struct pmx_driver pmx_driver;

/*
 * Pad multiplexing for making all pads as gpio's. This is done to override the
 * values passed from bootloader and start from scratch.
 */
static struct pmx_mux_reg pmx_pads_as_gpio_mux[] = {
	{
		.address = SPEAR1340_PAD_FUNCTION_EN_1,
		.mask = SPEAR1340_PMX_PADS_AS_GPIO_REG1_MASK,
		.value = 0x0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_2,
		.mask = SPEAR1340_PMX_PADS_AS_GPIO_REGS_MASK,
		.value = 0x0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_3,
		.mask = SPEAR1340_PMX_PADS_AS_GPIO_REGS_MASK,
		.value = 0x0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_4,
		.mask = SPEAR1340_PMX_PADS_AS_GPIO_REGS_MASK,
		.value = 0x0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_5,
		.mask = SPEAR1340_PMX_PADS_AS_GPIO_REGS_MASK,
		.value = 0x0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_6,
		.mask = SPEAR1340_PMX_PADS_AS_GPIO_REGS_MASK,
		.value = 0x0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_7,
		.mask = SPEAR1340_PMX_PADS_AS_GPIO_REGS_MASK,
		.value = 0x0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_8,
		.mask = SPEAR1340_PMX_PADS_AS_GPIO_REG8_MASK,
		.value = 0x0,
	},
};

static struct pmx_dev_mode pmx_pads_as_gpio_modes[] = {
	{
		.mux_regs = pmx_pads_as_gpio_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_pads_as_gpio_mux),
	},
};

struct pmx_dev spear1340_pmx_pads_as_gpio = {
	.name = "pads_as_gpio",
	.modes = pmx_pads_as_gpio_modes,
	.mode_count = ARRAY_SIZE(pmx_pads_as_gpio_modes),
};

/* Pad multiplexing for fsmc_8bit device */
static struct pmx_mux_reg pmx_fsmc_8bit_mux[] = {
	{
		.address = SPEAR1340_PAD_FUNCTION_EN_8,
		.mask = SPEAR1340_PMX_FSMC_8BIT_REG8_MASK,
		.value = SPEAR1340_PMX_FSMC_8BIT_REG8_MASK,
	}
};

static struct pmx_dev_mode pmx_fsmc_8bit_modes[] = {
	{
		.mux_regs = pmx_fsmc_8bit_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_fsmc_8bit_mux),
	},
};

struct pmx_dev spear1340_pmx_fsmc_8bit = {
	.name = "fsmc_8bit",
	.modes = pmx_fsmc_8bit_modes,
	.mode_count = ARRAY_SIZE(pmx_fsmc_8bit_modes),
};

/* Pad multiplexing for fsmc_16bit device */
static struct pmx_mux_reg pmx_fsmc_16bit_mux[] = {
	{
		.address = SPEAR1340_PAD_SHARED_IP_EN_1,
		.mask = SPEAR1340_PMX_KBD_ROW_COL_MASK,
		.value = 0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_1,
		.mask = SPEAR1340_PMX_FSMC_16_BIT_AND_KBD_ROW_COL_REG1_MASK,
		.value = SPEAR1340_PMX_FSMC_16_BIT_AND_KBD_ROW_COL_REG1_MASK,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_8,
		.mask = SPEAR1340_PMX_FSMC_8BIT_REG8_MASK,
		.value = SPEAR1340_PMX_FSMC_8BIT_REG8_MASK,
	},
};

static struct pmx_dev_mode pmx_fsmc_16bit_modes[] = {
	{
		.mux_regs = pmx_fsmc_16bit_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_fsmc_16bit_mux),
	},
};

struct pmx_dev spear1340_pmx_fsmc_16bit = {
	.name = "fsmc_16bit",
	.modes = pmx_fsmc_16bit_modes,
	.mode_count = ARRAY_SIZE(pmx_fsmc_16bit_modes),
};

/* pad multiplexing for keyboard rows-cols device */
static struct pmx_mux_reg pmx_keyboard_row_col_mux[] = {
	{
		.address = SPEAR1340_PAD_SHARED_IP_EN_1,
		.mask = SPEAR1340_PMX_KBD_ROW_COL_MASK,
		.value = SPEAR1340_PMX_KBD_ROW_COL_MASK,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_1,
		.mask = SPEAR1340_PMX_FSMC_16_BIT_AND_KBD_ROW_COL_REG1_MASK,
		.value = SPEAR1340_PMX_FSMC_16_BIT_AND_KBD_ROW_COL_REG1_MASK,
	},
};

static struct pmx_dev_mode pmx_keyboard_row_col_modes[] = {
	{
		.mux_regs = pmx_keyboard_row_col_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_keyboard_row_col_mux),
	},
};

struct pmx_dev spear1340_pmx_keyboard_row_col = {
	.name = "keyboard_row_col",
	.modes = pmx_keyboard_row_col_modes,
	.mode_count = ARRAY_SIZE(pmx_keyboard_row_col_modes),
};

/* pad multiplexing for keyboard col5 device */
static struct pmx_mux_reg pmx_keyboard_col5_mux[] = {
	{
		.address = SPEAR1340_PAD_SHARED_IP_EN_1,
		.mask = SPEAR1340_PMX_KBD_COL5_MASK,
		.value = SPEAR1340_PMX_KBD_COL5_MASK,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_1,
		.mask = SPEAR1340_PMX_PWM1_AND_KBD_COL5_REG1_MASK,
		.value = SPEAR1340_PMX_PWM1_AND_KBD_COL5_REG1_MASK,
	},
};

static struct pmx_dev_mode pmx_keyboard_col5_modes[] = {
	{
		.mux_regs = pmx_keyboard_col5_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_keyboard_col5_mux),
	},
};

struct pmx_dev spear1340_pmx_keyboard_col5 = {
	.name = "keyboard_col5",
	.modes = pmx_keyboard_col5_modes,
	.mode_count = ARRAY_SIZE(pmx_keyboard_col5_modes),
};

/* pad multiplexing for uart0_enh device */
static struct pmx_mux_reg pmx_uart0_enh_mux[] = {
	{
		.address = SPEAR1340_PAD_SHARED_IP_EN_1,
		.mask = SPEAR1340_PMX_GPT_MASK,
		.value = 0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_1,
		.mask = SPEAR1340_PMX_UART0_ENH_AND_GPT_REG1_MASK,
		.value = SPEAR1340_PMX_UART0_ENH_AND_GPT_REG1_MASK,
	},
};

static struct pmx_dev_mode pmx_uart0_enh_modes[] = {
	{
		.mux_regs = pmx_uart0_enh_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart0_enh_mux),
	},
};

struct pmx_dev spear1340_pmx_uart0_enh = {
	.name = "uart0_enh",
	.modes = pmx_uart0_enh_modes,
	.mode_count = ARRAY_SIZE(pmx_uart0_enh_modes),
};

/* pad multiplexing for i2c1 device */
static struct pmx_mux_reg pmx_i2c1_mux[] = {
	{
		.address = SPEAR1340_PAD_FUNCTION_EN_1,
		.mask = SPEAR1340_PMX_I2C1_REG1_MASK,
		.value = SPEAR1340_PMX_I2C1_REG1_MASK,
	},
};

static struct pmx_dev_mode pmx_i2c1_modes[] = {
	{
		.mux_regs = pmx_i2c1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_i2c1_mux),
	},
};

struct pmx_dev spear1340_pmx_i2c1 = {
	.name = "i2c1",
	.modes = pmx_i2c1_modes,
	.mode_count = ARRAY_SIZE(pmx_i2c1_modes),
};

/* pad multiplexing for spdif_in device */
static struct pmx_mux_reg pmx_spdif_in_mux[] = {
	{
		.address = SPEAR1340_PAD_FUNCTION_EN_1,
		.mask = SPEAR1340_PMX_SPDIF_IN_REG1_MASK,
		.value = SPEAR1340_PMX_SPDIF_IN_REG1_MASK,
	},
};

static struct pmx_dev_mode pmx_spdif_in_modes[] = {
	{
		.mux_regs = pmx_spdif_in_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_spdif_in_mux),
	},
};

struct pmx_dev spear1340_pmx_spdif_in = {
	.name = "spdif_in",
	.modes = pmx_spdif_in_modes,
	.mode_count = ARRAY_SIZE(pmx_spdif_in_modes),
};

/* pad multiplexing for gpt_0_1 device */
static struct pmx_mux_reg pmx_gpt_0_1_mux[] = {
	{
		.address = SPEAR1340_PAD_SHARED_IP_EN_1,
		.mask = SPEAR1340_PMX_GPT_MASK |
			SPEAR1340_PMX_GPT0_TMR0_CPT_MASK |
			SPEAR1340_PMX_GPT0_TMR1_CLK_MASK,
		.value = SPEAR1340_PMX_GPT_MASK |
			SPEAR1340_PMX_GPT0_TMR0_CPT_MASK |
			SPEAR1340_PMX_GPT0_TMR1_CLK_MASK,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_1,
		.mask = SPEAR1340_PMX_UART0_ENH_AND_GPT_REG1_MASK |
			SPEAR1340_PMX_PWM2_AND_GPT0_TMR0_CPT_REG1_MASK |
			SPEAR1340_PMX_PWM3_AND_GPT0_TMR1_CLK_REG1_MASK,
		.value = SPEAR1340_PMX_UART0_ENH_AND_GPT_REG1_MASK |
			SPEAR1340_PMX_PWM2_AND_GPT0_TMR0_CPT_REG1_MASK |
			SPEAR1340_PMX_PWM3_AND_GPT0_TMR1_CLK_REG1_MASK,
	},
};

static struct pmx_dev_mode pmx_gpt_0_1_modes[] = {
	{
		.mux_regs = pmx_gpt_0_1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_gpt_0_1_mux),
	},
};

struct pmx_dev spear1340_pmx_gpt_0_1 = {
	.name = "gpt_0_1",
	.modes = pmx_gpt_0_1_modes,
	.mode_count = ARRAY_SIZE(pmx_gpt_0_1_modes),
};

/* pad multiplexing for pwm1 device */
static struct pmx_mux_reg pmx_pwm1_mux[] = {
	{
		.address = SPEAR1340_PAD_SHARED_IP_EN_1,
		.mask = SPEAR1340_PMX_KBD_COL5_MASK,
		.value = 0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_1,
		.mask = SPEAR1340_PMX_PWM1_AND_KBD_COL5_REG1_MASK,
		.value = SPEAR1340_PMX_PWM1_AND_KBD_COL5_REG1_MASK,
	},
};

static struct pmx_dev_mode pmx_pwm1_modes[] = {
	{
		.mux_regs = pmx_pwm1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_pwm1_mux),
	},
};

struct pmx_dev spear1340_pmx_pwm1 = {
	.name = "pwm1",
	.modes = pmx_pwm1_modes,
	.mode_count = ARRAY_SIZE(pmx_pwm1_modes),
};

/* pad multiplexing for pwm2 device */
static struct pmx_mux_reg pmx_pwm2_mux[] = {
	{
		.address = SPEAR1340_PAD_SHARED_IP_EN_1,
		.mask = SPEAR1340_PMX_GPT0_TMR0_CPT_MASK,
		.value = 0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_1,
		.mask = SPEAR1340_PMX_PWM2_AND_GPT0_TMR0_CPT_REG1_MASK,
		.value = SPEAR1340_PMX_PWM2_AND_GPT0_TMR0_CPT_REG1_MASK,
	},
};

static struct pmx_dev_mode pmx_pwm2_modes[] = {
	{
		.mux_regs = pmx_pwm2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_pwm2_mux),
	},
};

struct pmx_dev spear1340_pmx_pwm2 = {
	.name = "pwm2",
	.modes = pmx_pwm2_modes,
	.mode_count = ARRAY_SIZE(pmx_pwm2_modes),
};

/* pad multiplexing for pwm3 device */
static struct pmx_mux_reg pmx_pwm3_mux[] = {
	{
		.address = SPEAR1340_PAD_SHARED_IP_EN_1,
		.mask = SPEAR1340_PMX_GPT0_TMR1_CLK_MASK,
		.value = 0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_1,
		.mask = SPEAR1340_PMX_PWM3_AND_GPT0_TMR1_CLK_REG1_MASK,
		.value = SPEAR1340_PMX_PWM3_AND_GPT0_TMR1_CLK_REG1_MASK,
	},
};

static struct pmx_dev_mode pmx_pwm3_modes[] = {
	{
		.mux_regs = pmx_pwm3_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_pwm3_mux),
	},
};

struct pmx_dev spear1340_pmx_pwm3 = {
	.name = "pwm3",
	.modes = pmx_pwm3_modes,
	.mode_count = ARRAY_SIZE(pmx_pwm3_modes),
};

/* pad multiplexing for pwm0 device */
static struct pmx_mux_reg pmx_pwm0_mux[] = {
	{
		.address = SPEAR1340_PAD_SHARED_IP_EN_1,
		.mask = SPEAR1340_PMX_SSP0_CS1_MASK,
		.value = 0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_1,
		.mask = SPEAR1340_PMX_PWM0_AND_SSP0_CS1_REG1_MASK,
		.value = SPEAR1340_PMX_PWM0_AND_SSP0_CS1_REG1_MASK,
	},
};

static struct pmx_dev_mode pmx_pwm0_modes[] = {
	{
		.mux_regs = pmx_pwm0_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_pwm0_mux),
	},
};

struct pmx_dev spear1340_pmx_pwm0 = {
	.name = "pwm0",
	.modes = pmx_pwm0_modes,
	.mode_count = ARRAY_SIZE(pmx_pwm0_modes),
};

/* pad multiplexing for ssp0_cs1 device */
static struct pmx_mux_reg pmx_ssp0_cs1_mux[] = {
	{
		.address = SPEAR1340_PAD_SHARED_IP_EN_1,
		.mask = SPEAR1340_PMX_SSP0_CS1_MASK,
		.value = SPEAR1340_PMX_SSP0_CS1_MASK,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_1,
		.mask = SPEAR1340_PMX_PWM0_AND_SSP0_CS1_REG1_MASK,
		.value = SPEAR1340_PMX_PWM0_AND_SSP0_CS1_REG1_MASK,
	},
};

static struct pmx_dev_mode pmx_ssp0_cs1_modes[] = {
	{
		.mux_regs = pmx_ssp0_cs1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_ssp0_cs1_mux),
	},
};

struct pmx_dev spear1340_pmx_ssp0_cs1 = {
	.name = "ssp0_cs1",
	.modes = pmx_ssp0_cs1_modes,
	.mode_count = ARRAY_SIZE(pmx_ssp0_cs1_modes),
};

/* pad multiplexing for vip_mux_cam0 (disables cam0) device */
static struct pmx_mux_reg pmx_vip_mux_cam0_mux[] = {
	{
		.address = SPEAR1340_PAD_SHARED_IP_EN_1,
		.mask = SPEAR1340_PMX_CAM0_MASK,
		.value = 0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_2,
		.mask = SPEAR1340_PMX_VIP_REG2_MASK,
		.value = SPEAR1340_PMX_VIP_REG2_MASK,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_3,
		.mask = SPEAR1340_PMX_VIP_AND_CAM0_REG3_MASK,
		.value = SPEAR1340_PMX_VIP_AND_CAM0_REG3_MASK,
	},
};

static struct pmx_dev_mode pmx_vip_mux_cam0_modes[] = {
	{
		.mux_regs = pmx_vip_mux_cam0_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_vip_mux_cam0_mux),
	},
};

struct pmx_dev spear1340_pmx_vip_mux_cam0 = {
	.name = "vip_mux_cam0",
	.modes = pmx_vip_mux_cam0_modes,
	.mode_count = ARRAY_SIZE(pmx_vip_mux_cam0_modes),
};

/* pad multiplexing for vip_mux_cam1 (disables cam1) device */
static struct pmx_mux_reg pmx_vip_mux_cam1_mux[] = {
	{
		.address = SPEAR1340_PAD_SHARED_IP_EN_1,
		.mask = SPEAR1340_PMX_CAM1_MASK,
		.value = 0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_2,
		.mask = SPEAR1340_PMX_VIP_REG2_MASK |
			SPEAR1340_PMX_VIP_AND_CAM1_REG2_MASK,
		.value = SPEAR1340_PMX_VIP_REG2_MASK |
			SPEAR1340_PMX_VIP_AND_CAM1_REG2_MASK,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_3,
		.mask = SPEAR1340_PMX_VIP_AND_CAM1_REG3_MASK,
		.value = SPEAR1340_PMX_VIP_AND_CAM1_REG3_MASK,
	},
};

static struct pmx_dev_mode pmx_vip_mux_cam1_modes[] = {
	{
		.mux_regs = pmx_vip_mux_cam1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_vip_mux_cam1_mux),
	},
};

struct pmx_dev spear1340_pmx_vip_mux_cam1 = {
	.name = "vip_mux_cam1",
	.modes = pmx_vip_mux_cam1_modes,
	.mode_count = ARRAY_SIZE(pmx_vip_mux_cam1_modes),
};

/* pad multiplexing for vip_mux_cam2 (disables cam2) device */
static struct pmx_mux_reg pmx_vip_mux_cam2_mux[] = {
	{
		.address = SPEAR1340_PAD_SHARED_IP_EN_1,
		.mask = SPEAR1340_PMX_CAM2_MASK,
		.value = 0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_2,
		.mask = SPEAR1340_PMX_VIP_REG2_MASK |
			SPEAR1340_PMX_VIP_AND_CAM2_REG2_MASK,
		.value = SPEAR1340_PMX_VIP_REG2_MASK |
			SPEAR1340_PMX_VIP_AND_CAM2_REG2_MASK,
	},
};

static struct pmx_dev_mode pmx_vip_mux_cam2_modes[] = {
	{
		.mux_regs = pmx_vip_mux_cam2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_vip_mux_cam2_mux),
	},
};

struct pmx_dev spear1340_pmx_vip_mux_cam2 = {
	.name = "vip_mux_cam2",
	.modes = pmx_vip_mux_cam2_modes,
	.mode_count = ARRAY_SIZE(pmx_vip_mux_cam2_modes),
};

/* pad multiplexing for vip_mux_cam3 (disables cam3) device */
static struct pmx_mux_reg pmx_vip_mux_cam3_mux[] = {
	{
		.address = SPEAR1340_PAD_SHARED_IP_EN_1,
		.mask = SPEAR1340_PMX_CAM3_MASK,
		.value = 0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_1,
		.mask = SPEAR1340_PMX_VIP_AND_CAM3_REG1_MASK,
		.value = SPEAR1340_PMX_VIP_AND_CAM3_REG1_MASK,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_2,
		.mask = SPEAR1340_PMX_VIP_REG2_MASK |
			SPEAR1340_PMX_VIP_AND_CAM3_REG2_MASK,
		.value = SPEAR1340_PMX_VIP_REG2_MASK |
			SPEAR1340_PMX_VIP_AND_CAM3_REG2_MASK,
	},
};

static struct pmx_dev_mode pmx_vip_mux_cam3_modes[] = {
	{
		.mux_regs = pmx_vip_mux_cam3_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_vip_mux_cam3_mux),
	},
};

struct pmx_dev spear1340_pmx_vip_mux_cam3 = {
	.name = "vip_mux_cam3",
	.modes = pmx_vip_mux_cam3_modes,
	.mode_count = ARRAY_SIZE(pmx_vip_mux_cam3_modes),
};

/* pad multiplexing for cam3 device */
static struct pmx_mux_reg pmx_cam3_mux[] = {
	{
		.address = SPEAR1340_PAD_SHARED_IP_EN_1,
		.mask = SPEAR1340_PMX_CAM3_MASK,
		.value = SPEAR1340_PMX_CAM3_MASK,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_1,
		.mask = SPEAR1340_PMX_VIP_AND_CAM3_REG1_MASK,
		.value = SPEAR1340_PMX_VIP_AND_CAM3_REG1_MASK,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_2,
		.mask = SPEAR1340_PMX_VIP_AND_CAM3_REG2_MASK,
		.value = SPEAR1340_PMX_VIP_AND_CAM3_REG2_MASK,
	},
};

static struct pmx_dev_mode pmx_cam3_modes[] = {
	{
		.mux_regs = pmx_cam3_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_cam3_mux),
	},
};

struct pmx_dev spear1340_pmx_cam3 = {
	.name = "cam3",
	.modes = pmx_cam3_modes,
	.mode_count = ARRAY_SIZE(pmx_cam3_modes),
};

/* pad multiplexing for cam2 device */
static struct pmx_mux_reg pmx_cam2_mux[] = {
	{
		.address = SPEAR1340_PAD_SHARED_IP_EN_1,
		.mask = SPEAR1340_PMX_CAM2_MASK,
		.value = SPEAR1340_PMX_CAM2_MASK,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_2,
		.mask = SPEAR1340_PMX_VIP_AND_CAM2_REG2_MASK,
		.value = SPEAR1340_PMX_VIP_AND_CAM2_REG2_MASK,
	},
};

static struct pmx_dev_mode pmx_cam2_modes[] = {
	{
		.mux_regs = pmx_cam2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_cam2_mux),
	},
};

struct pmx_dev spear1340_pmx_cam2 = {
	.name = "cam2",
	.modes = pmx_cam2_modes,
	.mode_count = ARRAY_SIZE(pmx_cam2_modes),
};

/* pad multiplexing for cam1 device */
static struct pmx_mux_reg pmx_cam1_mux[] = {
	{
		.address = SPEAR1340_PAD_SHARED_IP_EN_1,
		.mask = SPEAR1340_PMX_CAM1_MASK,
		.value = SPEAR1340_PMX_CAM1_MASK,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_2,
		.mask = SPEAR1340_PMX_VIP_AND_CAM1_REG2_MASK,
		.value = SPEAR1340_PMX_VIP_AND_CAM1_REG2_MASK,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_3,
		.mask = SPEAR1340_PMX_VIP_AND_CAM1_REG3_MASK,
		.value = SPEAR1340_PMX_VIP_AND_CAM1_REG3_MASK,
	},
};

static struct pmx_dev_mode pmx_cam1_modes[] = {
	{
		.mux_regs = pmx_cam1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_cam1_mux),
	},
};

struct pmx_dev spear1340_pmx_cam1 = {
	.name = "cam1",
	.modes = pmx_cam1_modes,
	.mode_count = ARRAY_SIZE(pmx_cam1_modes),
};

/* pad multiplexing for cam0 device */
static struct pmx_mux_reg pmx_cam0_mux[] = {
	{
		.address = SPEAR1340_PAD_SHARED_IP_EN_1,
		.mask = SPEAR1340_PMX_CAM0_MASK,
		.value = SPEAR1340_PMX_CAM0_MASK,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_3,
		.mask = SPEAR1340_PMX_VIP_AND_CAM0_REG3_MASK,
		.value = SPEAR1340_PMX_VIP_AND_CAM0_REG3_MASK,
	},
};

static struct pmx_dev_mode pmx_cam0_modes[] = {
	{
		.mux_regs = pmx_cam0_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_cam0_mux),
	},
};

struct pmx_dev spear1340_pmx_cam0 = {
	.name = "cam0",
	.modes = pmx_cam0_modes,
	.mode_count = ARRAY_SIZE(pmx_cam0_modes),
};

/* pad multiplexing for smi device */
static struct pmx_mux_reg pmx_smi_mux[] = {
	{
		.address = SPEAR1340_PAD_FUNCTION_EN_3,
		.mask = SPEAR1340_PMX_SMI_REG3_MASK,
		.value = SPEAR1340_PMX_SMI_REG3_MASK,
	},
};

static struct pmx_dev_mode pmx_smi_modes[] = {
	{
		.mux_regs = pmx_smi_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_smi_mux),
	},
};

struct pmx_dev spear1340_pmx_smi = {
	.name = "smi",
	.modes = pmx_smi_modes,
	.mode_count = ARRAY_SIZE(pmx_smi_modes),
};

/* pad multiplexing for ssp0 device */
static struct pmx_mux_reg pmx_ssp0_mux[] = {
	{
		.address = SPEAR1340_PAD_FUNCTION_EN_3,
		.mask = SPEAR1340_PMX_SSP0_REG3_MASK,
		.value = SPEAR1340_PMX_SSP0_REG3_MASK,
	},
};

static struct pmx_dev_mode pmx_ssp0_modes[] = {
	{
		.mux_regs = pmx_ssp0_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_ssp0_mux),
	},
};

struct pmx_dev spear1340_pmx_ssp0 = {
	.name = "ssp0",
	.modes = pmx_ssp0_modes,
	.mode_count = ARRAY_SIZE(pmx_ssp0_modes),
};

/* pad multiplexing for ssp0_cs2 device */
static struct pmx_mux_reg pmx_ssp0_cs2_mux[] = {
	{
		.address = SPEAR1340_PAD_SHARED_IP_EN_1,
		.mask = SPEAR1340_PMX_SSP0_CS2_MASK,
		.value = SPEAR1340_PMX_SSP0_CS2_MASK,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_3,
		.mask = SPEAR1340_PMX_TS_AND_SSP0_CS2_REG3_MASK,
		.value = SPEAR1340_PMX_TS_AND_SSP0_CS2_REG3_MASK,
	},
};

static struct pmx_dev_mode pmx_ssp0_cs2_modes[] = {
	{
		.mux_regs = pmx_ssp0_cs2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_ssp0_cs2_mux),
	},
};

struct pmx_dev spear1340_pmx_ssp0_cs2 = {
	.name = "ssp0_cs2",
	.modes = pmx_ssp0_cs2_modes,
	.mode_count = ARRAY_SIZE(pmx_ssp0_cs2_modes),
};

/* pad multiplexing for uart0 device */
static struct pmx_mux_reg pmx_uart0_mux[] = {
	{
		.address = SPEAR1340_PAD_FUNCTION_EN_3,
		.mask = SPEAR1340_PMX_UART0_REG3_MASK,
		.value = SPEAR1340_PMX_UART0_REG3_MASK,
	},
};

static struct pmx_dev_mode pmx_uart0_modes[] = {
	{
		.mux_regs = pmx_uart0_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart0_mux),
	},
};

struct pmx_dev spear1340_pmx_uart0 = {
	.name = "uart0",
	.modes = pmx_uart0_modes,
	.mode_count = ARRAY_SIZE(pmx_uart0_modes),
};

/* pad multiplexing for uart1 device */
static struct pmx_mux_reg pmx_uart1_mux[] = {
	{
		.address = SPEAR1340_PAD_FUNCTION_EN_3,
		.mask = SPEAR1340_PMX_UART1_REG3_MASK,
		.value = SPEAR1340_PMX_UART1_REG3_MASK,
	},
};

static struct pmx_dev_mode pmx_uart1_modes[] = {
	{
		.mux_regs = pmx_uart1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart1_mux),
	},
};

struct pmx_dev spear1340_pmx_uart1 = {
	.name = "uart1",
	.modes = pmx_uart1_modes,
	.mode_count = ARRAY_SIZE(pmx_uart1_modes),
};

/* pad multiplexing for i2s_in device */
static struct pmx_mux_reg pmx_i2s_in_mux[] = {
	{
		.address = SPEAR1340_PAD_FUNCTION_EN_3,
		.mask = SPEAR1340_PMX_I2S_IN_REG3_MASK,
		.value = SPEAR1340_PMX_I2S_IN_REG3_MASK,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_4,
		.mask = SPEAR1340_PMX_I2S_IN_REG4_MASK,
		.value = SPEAR1340_PMX_I2S_IN_REG4_MASK,
	},
};

static struct pmx_dev_mode pmx_i2s_in_modes[] = {
	{
		.mux_regs = pmx_i2s_in_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_i2s_in_mux),
	},
};

struct pmx_dev spear1340_pmx_i2s_in = {
	.name = "i2s_in",
	.modes = pmx_i2s_in_modes,
	.mode_count = ARRAY_SIZE(pmx_i2s_in_modes),
};

/* pad multiplexing for i2s_out device */
static struct pmx_mux_reg pmx_i2s_out_mux[] = {
	{
		.address = SPEAR1340_PAD_FUNCTION_EN_4,
		.mask = SPEAR1340_PMX_I2S_OUT_REG4_MASK,
		.value = SPEAR1340_PMX_I2S_OUT_REG4_MASK,
	},
};

static struct pmx_dev_mode pmx_i2s_out_modes[] = {
	{
		.mux_regs = pmx_i2s_out_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_i2s_out_mux),
	},
};

struct pmx_dev spear1340_pmx_i2s_out = {
	.name = "i2s_out",
	.modes = pmx_i2s_out_modes,
	.mode_count = ARRAY_SIZE(pmx_i2s_out_modes),
};

/* pad multiplexing for gmac device */
static struct pmx_mux_reg pmx_gmac_mux[] = {
	{
		.address = SPEAR1340_PAD_FUNCTION_EN_4,
		.mask = SPEAR1340_PMX_GMAC_REG4_MASK,
		.value = SPEAR1340_PMX_GMAC_REG4_MASK,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_5,
		.mask = SPEAR1340_PMX_GMAC_REG5_MASK,
		.value = SPEAR1340_PMX_GMAC_REG5_MASK,
	},
};

static struct pmx_dev_mode pmx_gmac_modes[] = {
	{
		.mux_regs = pmx_gmac_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_gmac_mux),
	},
};

struct pmx_dev spear1340_pmx_gmac = {
	.name = "gmac",
	.modes = pmx_gmac_modes,
	.mode_count = ARRAY_SIZE(pmx_gmac_modes),
};

/* pad multiplexing for ssp0_cs3 device */
static struct pmx_mux_reg pmx_ssp0_cs3_mux[] = {
	{
		.address = SPEAR1340_PAD_FUNCTION_EN_5,
		.mask = SPEAR1340_PMX_SSP0_CS3_REG5_MASK,
		.value = SPEAR1340_PMX_SSP0_CS3_REG5_MASK,
	},
};

static struct pmx_dev_mode pmx_ssp0_cs3_modes[] = {
	{
		.mux_regs = pmx_ssp0_cs3_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_ssp0_cs3_mux),
	},
};

struct pmx_dev spear1340_pmx_ssp0_cs3 = {
	.name = "ssp0_cs3",
	.modes = pmx_ssp0_cs3_modes,
	.mode_count = ARRAY_SIZE(pmx_ssp0_cs3_modes),
};

/* pad multiplexing for i2c0 device */
static struct pmx_mux_reg pmx_i2c0_mux[] = {
	{
		.address = SPEAR1340_PAD_FUNCTION_EN_5,
		.mask = SPEAR1340_PMX_I2C0_REG5_MASK,
		.value = SPEAR1340_PMX_I2C0_REG5_MASK,
	},
};

static struct pmx_dev_mode pmx_i2c0_modes[] = {
	{
		.mux_regs = pmx_i2c0_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_i2c0_mux),
	},
};

struct pmx_dev spear1340_pmx_i2c0 = {
	.name = "i2c0",
	.modes = pmx_i2c0_modes,
	.mode_count = ARRAY_SIZE(pmx_i2c0_modes),
};

/* pad multiplexing for cec0 device */
static struct pmx_mux_reg pmx_cec0_mux[] = {
	{
		.address = SPEAR1340_PAD_FUNCTION_EN_5,
		.mask = SPEAR1340_PMX_CEC0_REG5_MASK,
		.value = SPEAR1340_PMX_CEC0_REG5_MASK,
	},
};

static struct pmx_dev_mode pmx_cec0_modes[] = {
	{
		.mux_regs = pmx_cec0_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_cec0_mux),
	},
};

struct pmx_dev spear1340_pmx_cec0 = {
	.name = "cec0",
	.modes = pmx_cec0_modes,
	.mode_count = ARRAY_SIZE(pmx_cec0_modes),
};

/* pad multiplexing for cec1 device */
static struct pmx_mux_reg pmx_cec1_mux[] = {
	{
		.address = SPEAR1340_PAD_FUNCTION_EN_5,
		.mask = SPEAR1340_PMX_CEC1_REG5_MASK,
		.value = SPEAR1340_PMX_CEC1_REG5_MASK,
	},
};

static struct pmx_dev_mode pmx_cec1_modes[] = {
	{
		.mux_regs = pmx_cec1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_cec1_mux),
	},
};

struct pmx_dev spear1340_pmx_cec1 = {
	.name = "cec1",
	.modes = pmx_cec1_modes,
	.mode_count = ARRAY_SIZE(pmx_cec1_modes),
};

/* pad multiplexing for spdif_out device */
static struct pmx_mux_reg pmx_spdif_out_mux[] = {
	{
		.address = SPEAR1340_PAD_FUNCTION_EN_5,
		.mask = SPEAR1340_PMX_SPDIF_OUT_REG5_MASK,
		.value = SPEAR1340_PMX_SPDIF_OUT_REG5_MASK,
	}, {
		.address = SPEAR1340_PERIP_CFG,
		.mask = SPEAR1340_SPDIF_OUT_ENB_MASK,
		.value = SPEAR1340_SPDIF_OUT_ENB_MASK,
	}
};

static struct pmx_dev_mode pmx_spdif_out_modes[] = {
	{
		.mux_regs = pmx_spdif_out_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_spdif_out_mux),
	},
};

struct pmx_dev spear1340_pmx_spdif_out = {
	.name = "spdif_out",
	.modes = pmx_spdif_out_modes,
	.mode_count = ARRAY_SIZE(pmx_spdif_out_modes),
};

/* pad multiplexing for fsmc_pnor device */
static struct pmx_mux_reg pmx_fsmc_pnor_mux[] = {
	{
		.address = SPEAR1340_PAD_SHARED_IP_EN_1,
		.mask = SPEAR1340_PMX_MCIF_MASK,
		.value = 0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_7,
		.mask = SPEAR1340_PMX_FSMC_PNOR_AND_MCIF_REG7_MASK,
		.value = SPEAR1340_PMX_FSMC_PNOR_AND_MCIF_REG7_MASK,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_8,
		.mask = SPEAR1340_PMX_FSMC_8BIT_REG8_MASK,
		.value = SPEAR1340_PMX_FSMC_8BIT_REG8_MASK,
	},
};

static struct pmx_dev_mode pmx_fsmc_pnor_modes[] = {
	{
		.mux_regs = pmx_fsmc_pnor_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_fsmc_pnor_mux),
	},
};

struct pmx_dev spear1340_pmx_fsmc_pnor = {
	.name = "fsmc_pnor",
	.modes = pmx_fsmc_pnor_modes,
	.mode_count = ARRAY_SIZE(pmx_fsmc_pnor_modes),
};

/* pad multiplexing for mcif device */
static struct pmx_mux_reg pmx_mcif_mux[] = {
	{
		.address = SPEAR1340_PAD_SHARED_IP_EN_1,
		.mask = SPEAR1340_PMX_MCIF_MASK,
		.value = SPEAR1340_PMX_MCIF_MASK,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_7,
		.mask = SPEAR1340_PMX_FSMC_PNOR_AND_MCIF_REG7_MASK |
			SPEAR1340_PMX_MCIF_REG7_MASK,
		.value = SPEAR1340_PMX_FSMC_PNOR_AND_MCIF_REG7_MASK |
			SPEAR1340_PMX_MCIF_REG7_MASK,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_8,
		.mask = SPEAR1340_PMX_MCIF_REG8_MASK,
		.value = SPEAR1340_PMX_MCIF_REG8_MASK,
	},
};

static struct pmx_dev_mode pmx_mcif_modes[] = {
	{
		.mux_regs = pmx_mcif_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_mcif_mux),
	},
};

struct pmx_dev spear1340_pmx_mcif = {
	.name = "mcif",
	.modes = pmx_mcif_modes,
	.mode_count = ARRAY_SIZE(pmx_mcif_modes),
};

/* Pad multiplexing for sdhci device */
static struct pmx_mux_reg pmx_sdhci_mux[] = {
	{
		.address = SPEAR1340_PERIP_CFG,
		.mask = SPEAR1340_MCIF_SEL_MASK,
		.value = SPEAR1340_MCIF_SEL_SD,
	},
};

static struct pmx_dev_mode pmx_sdhci_modes[] = {
	{
		.mux_regs = pmx_sdhci_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_sdhci_mux),
	},
};

struct pmx_dev spear1340_pmx_sdhci = {
	.name = "sdhci",
	.modes = pmx_sdhci_modes,
	.mode_count = ARRAY_SIZE(pmx_sdhci_modes),
};

/* Pad multiplexing for cf device */
static struct pmx_mux_reg pmx_cf_mux[] = {
	{
		.address = SPEAR1340_PERIP_CFG,
		.mask = SPEAR1340_MCIF_SEL_MASK,
		.value = SPEAR1340_MCIF_SEL_CF,
	},
};

static struct pmx_dev_mode pmx_cf_modes[] = {
	{
		.mux_regs = pmx_cf_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_cf_mux),
	},
};

struct pmx_dev spear1340_pmx_cf = {
	.name = "cf",
	.modes = pmx_cf_modes,
	.mode_count = ARRAY_SIZE(pmx_cf_modes),
};

/* Pad multiplexing for xd device */
static struct pmx_mux_reg pmx_xd_mux[] = {
	{
		.address = SPEAR1340_PERIP_CFG,
		.mask = SPEAR1340_MCIF_SEL_MASK,
		.value = SPEAR1340_MCIF_SEL_XD,
	},
};

static struct pmx_dev_mode pmx_xd_modes[] = {
	{
		.mux_regs = pmx_xd_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_xd_mux),
	},
};

struct pmx_dev spear1340_pmx_xd = {
	.name = "xd",
	.modes = pmx_xd_modes,
	.mode_count = ARRAY_SIZE(pmx_xd_modes),
};

/* pad multiplexing for clcd device */
static struct pmx_mux_reg pmx_clcd_mux[] = {
	{
		.address = SPEAR1340_PAD_SHARED_IP_EN_1,
		.mask = SPEAR1340_PMX_ARM_TRACE_MASK |
			SPEAR1340_PMX_MIPHY_DBG_MASK,
		.value = 0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_5,
		.mask = SPEAR1340_PMX_CLCD_REG5_MASK |
			SPEAR1340_PMX_CLCD_AND_ARM_TRACE_REG5_MASK,
		.value = SPEAR1340_PMX_CLCD_REG5_MASK |
			SPEAR1340_PMX_CLCD_AND_ARM_TRACE_REG5_MASK,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_6,
		.mask = SPEAR1340_PMX_CLCD_AND_ARM_TRACE_REG6_MASK,
		.value = SPEAR1340_PMX_CLCD_AND_ARM_TRACE_REG6_MASK,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_7,
		.mask = SPEAR1340_PMX_CLCD_AND_ARM_TRACE_REG7_MASK,
		.value = SPEAR1340_PMX_CLCD_AND_ARM_TRACE_REG7_MASK,
	},
};

static struct pmx_dev_mode pmx_clcd_modes[] = {
	{
		.mux_regs = pmx_clcd_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_clcd_mux),
	},
};

struct pmx_dev spear1340_pmx_clcd = {
	.name = "clcd",
	.modes = pmx_clcd_modes,
	.mode_count = ARRAY_SIZE(pmx_clcd_modes),
};

/* pad multiplexing for arm_trace device */
static struct pmx_mux_reg pmx_arm_trace_mux[] = {
	{
		.address = SPEAR1340_PAD_SHARED_IP_EN_1,
		.mask = SPEAR1340_PMX_ARM_TRACE_MASK,
		.value = SPEAR1340_PMX_ARM_TRACE_MASK,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_5,
		.mask = SPEAR1340_PMX_CLCD_AND_ARM_TRACE_REG5_MASK,
		.value = SPEAR1340_PMX_CLCD_AND_ARM_TRACE_REG5_MASK,
	},
};

static struct pmx_dev_mode pmx_arm_trace_modes[] = {
	{
		.mux_regs = pmx_arm_trace_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_arm_trace_mux),
	},
};

struct pmx_dev spear1340_pmx_arm_trace = {
	.name = "arm_trace",
	.modes = pmx_arm_trace_modes,
	.mode_count = ARRAY_SIZE(pmx_arm_trace_modes),
};

/* pad multiplexing for device group: I2S, SSP0_CS2, CEC0-1, SPDIF out, CLCD */
static struct pmx_mux_reg pmx_devs_grp_mux[] = {
	{
		.address = SPEAR1340_PAD_SHARED_IP_EN_1,
		.mask = SPEAR1340_PMX_MIPHY_DBG_MASK,
		.value = 0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_5,
		.mask = SPEAR1340_PMX_DEVS_GRP_AND_MIPHY_DBG_REG5_MASK,
		.value = SPEAR1340_PMX_DEVS_GRP_AND_MIPHY_DBG_REG5_MASK,
	},
};

static struct pmx_dev_mode pmx_devs_grp_modes[] = {
	{
		.mux_regs = pmx_devs_grp_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_devs_grp_mux),
	},
};

struct pmx_dev spear1340_pmx_devs_grp = {
	.name = "devs_grp",
	.modes = pmx_devs_grp_modes,
	.mode_count = ARRAY_SIZE(pmx_devs_grp_modes),
};

/* pad multiplexing for miphy_dbg device */
static struct pmx_mux_reg pmx_miphy_dbg_mux[] = {
	{
		.address = SPEAR1340_PAD_SHARED_IP_EN_1,
		.mask = SPEAR1340_PMX_MIPHY_DBG_MASK,
		.value = SPEAR1340_PMX_MIPHY_DBG_MASK,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_5,
		.mask = SPEAR1340_PMX_DEVS_GRP_AND_MIPHY_DBG_REG5_MASK,
		.value = SPEAR1340_PMX_DEVS_GRP_AND_MIPHY_DBG_REG5_MASK,
	},
};

static struct pmx_dev_mode pmx_miphy_dbg_modes[] = {
	{
		.mux_regs = pmx_miphy_dbg_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_miphy_dbg_mux),
	},
};

struct pmx_dev spear1340_pmx_miphy_dbg = {
	.name = "miphy_dbg",
	.modes = pmx_miphy_dbg_modes,
	.mode_count = ARRAY_SIZE(pmx_miphy_dbg_modes),
};

/* pad multiplexing for gmii device */
static struct pmx_mux_reg pmx_gmii_mux[] = {
	{
		.address = SPEAR1340_GMAC_CLK_CFG,
		.mask = SPEAR1340_GMAC_PHY_IF_SEL_MASK,
		.value = SPEAR1340_GMAC_PHY_IF_GMII_VAL,
	},
};

static struct pmx_dev_mode pmx_gmii_modes[] = {
	{
		.mux_regs = pmx_gmii_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_gmii_mux),
	},
};

struct pmx_dev spear1340_pmx_gmii = {
	.name = "gmii",
	.modes = pmx_gmii_modes,
	.mode_count = ARRAY_SIZE(pmx_gmii_modes),
};

/* pad multiplexing for rgmii device */
static struct pmx_mux_reg pmx_rgmii_mux[] = {
	{
		.address = SPEAR1340_GMAC_CLK_CFG,
		.mask = SPEAR1340_GMAC_PHY_IF_SEL_MASK,
		.value = SPEAR1340_GMAC_PHY_IF_RGMII_VAL,
	},
};

static struct pmx_dev_mode pmx_rgmii_modes[] = {
	{
		.mux_regs = pmx_rgmii_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_rgmii_mux),
	},
};

struct pmx_dev spear1340_pmx_rgmii = {
	.name = "rgmii",
	.modes = pmx_rgmii_modes,
	.mode_count = ARRAY_SIZE(pmx_rgmii_modes),
};

/* pad multiplexing for rmii device */
static struct pmx_mux_reg pmx_rmii_mux[] = {
	{
		.address = SPEAR1340_GMAC_CLK_CFG,
		.mask = SPEAR1340_GMAC_PHY_IF_SEL_MASK,
		.value = SPEAR1340_GMAC_PHY_IF_RMII_VAL,
	},
};

static struct pmx_dev_mode pmx_rmii_modes[] = {
	{
		.mux_regs = pmx_rmii_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_rmii_mux),
	},
};

struct pmx_dev spear1340_pmx_rmii = {
	.name = "rmii",
	.modes = pmx_rmii_modes,
	.mode_count = ARRAY_SIZE(pmx_rmii_modes),
};

/* pad multiplexing for sgmii device */
static struct pmx_mux_reg pmx_sgmii_mux[] = {
	{
		.address = SPEAR1340_GMAC_CLK_CFG,
		.mask = SPEAR1340_GMAC_PHY_IF_SEL_MASK,
		.value = SPEAR1340_GMAC_PHY_IF_SGMII_VAL,
	},
};

static struct pmx_dev_mode pmx_sgmii_modes[] = {
	{
		.mux_regs = pmx_sgmii_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_sgmii_mux),
	},
};

struct pmx_dev spear1340_pmx_sgmii = {
	.name = "sgmii",
	.modes = pmx_sgmii_modes,
	.mode_count = ARRAY_SIZE(pmx_sgmii_modes),
};

/* pad multiplexing for pcie device */
static struct pmx_mux_reg pmx_pcie_mux[] = {
	{
		.address = SPEAR1340_PCIE_SATA_CFG,
		.mask = SPEAR1340_SATA_PCIE_CFG_MASK,
		.value = SPEAR1340_PCIE_CFG_VAL,
	},
};

static struct pmx_dev_mode pmx_pcie_modes[] = {
	{
		.mux_regs = pmx_pcie_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_pcie_mux),
	},
};

struct pmx_dev spear1340_pmx_pcie = {
	.name = "pcie",
	.modes = pmx_pcie_modes,
	.mode_count = ARRAY_SIZE(pmx_pcie_modes),
};

/* pad multiplexing for sata device */
static struct pmx_mux_reg pmx_sata_mux[] = {
	{
		.address = SPEAR1340_PCIE_SATA_CFG,
		.mask = SPEAR1340_SATA_PCIE_CFG_MASK,
		.value = SPEAR1340_SATA_CFG_VAL,
	},
};

static struct pmx_dev_mode pmx_sata_modes[] = {
	{
		.mux_regs = pmx_sata_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_sata_mux),
	},
};

struct pmx_dev spear1340_pmx_sata = {
	.name = "sata",
	.modes = pmx_sata_modes,
	.mode_count = ARRAY_SIZE(pmx_sata_modes),
};

/* padmux devices to enable */
static void config_io_pads(struct pmx_dev **devs, u8 count, bool to_device)
{
	struct pmx_mux_reg *mux_reg;
	int ret, i, j, k;

	/*
	 * Use pas mux framework to program device pads as gpios or let
	 * them under device control. Turn them to device pads if
	 * to_device is true else reset to make them as gpio.
	 */
	for (i = 0; i < count; i++) {
		for (j = 0; j < devs[i]->mode_count; j++) {
			for (k = 0; k < devs[i]->modes[j].mux_reg_cnt; k++) {
				mux_reg = &devs[i]->modes[j].mux_regs[k];
				mux_reg->value = to_device? mux_reg->mask : 0x0;
			}
		}
	}

	ret = pmx_devs_enable(devs, count);
	if (ret)
		pr_err("padmux: registeration failed. err no: %d\n", ret);
}

/* Add spear1340 specific devices here */
/* Add Amba Devices */
/* uart device registeration */
static struct dw_dma_slave uart1_dma_param[] = {
	{
		/* Tx */
		.dma_dev = &spear13xx_dmac_device[0].dev,
		.cfg_hi = DWC_CFGH_DST_PER(SPEAR1340_DMA_REQ_UART1_TX),
		.cfg_lo = 0,
		.src_master = SPEAR1340_DMA_MASTER_MEMORY,
		.dst_master = SPEAR1340_DMA_MASTER_UART1,
	}, {
		/* Rx */
		.dma_dev = &spear13xx_dmac_device[0].dev,
		.cfg_hi = DWC_CFGH_SRC_PER(SPEAR1340_DMA_REQ_UART1_RX),
		.cfg_lo = 0,
		.src_master = SPEAR1340_DMA_MASTER_UART1,
		.dst_master = SPEAR1340_DMA_MASTER_MEMORY,
	}
};

static struct amba_pl011_data uart1_data = {
	.dma_filter = dw_dma_filter,
	.dma_tx_param = &uart1_dma_param[0],
	.dma_rx_param = &uart1_dma_param[1],
};

/* uart1 device registeration */
struct amba_device spear1340_uart1_device = {
	.dev = {
		.init_name = "uart1",
		.platform_data = &uart1_data,
	},
	.res = {
		.start = SPEAR1340_UART1_BASE,
		.end = SPEAR1340_UART1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {SPEAR1340_IRQ_UART1, NO_IRQ},
};

/* Add Platform Devices */

/* camera reset handler */
static void camif_enable(int cam_id, bool enable)
{
	u32 val = readl(VA_SPEAR1340_PERIP3_SW_RST);
	u32 mask = 0;

	switch (cam_id) {
	case 0:
		mask = SPEAR1340_CAM0_RST;
		break;
	case 1:
		mask = SPEAR1340_CAM1_RST;
		break;
	case 2:
		mask = SPEAR1340_CAM2_RST;
		break;
	case 3:
		mask = SPEAR1340_CAM3_RST;
		break;
	}

	if (!enable)
		writel(val | mask, VA_SPEAR1340_PERIP3_SW_RST);
	else
		writel(val & ~mask, VA_SPEAR1340_PERIP3_SW_RST);
}

/* camera interface 0 device registeration */
static struct camif_config_data cam0_data = {
	.sync_type = EXTERNAL_SYNC,
	.vsync_polarity = ACTIVE_HIGH,
	.hsync_polarity = ACTIVE_HIGH,
	.pclk_polarity = ACTIVE_LOW,
	.capture_mode = VIDEO_MODE_ALL_FRAMES,
	.burst_size = BURST_SIZE_128,
	.channel = EVEN_CHANNEL,
	.camif_module_enable = camif_enable,
};

static struct dw_dma_slave camif0_dma_param[] = {
	{
		/* odd line */
		.dma_dev = &spear13xx_dmac_device[1].dev,
		.cfg_hi = DWC_CFGH_SRC_PER(SPEAR1340_DMA_REQ_CAM0_ODD),
		.cfg_lo = 0,
		.src_master = SPEAR1340_DMA_MASTER_CAM,
		.dst_master = SPEAR1340_DMA_MASTER_MEMORY,
	}, {
		/* even line */
		.dma_dev = &spear13xx_dmac_device[1].dev,
		.cfg_hi = DWC_CFGH_SRC_PER(SPEAR1340_DMA_REQ_CAM0_EVEN),
		.cfg_lo = 0,
		.src_master = SPEAR1340_DMA_MASTER_CAM,
		.dst_master = SPEAR1340_DMA_MASTER_MEMORY,
	}
};

static struct camif_controller camif0_platform_data = {
	.dma_filter = dw_dma_filter,
	.dma_odd_param = &camif0_dma_param[0],
	.dma_even_param = &camif0_dma_param[1],
	.config = &cam0_data,
};

static struct resource camif0_resources[] = {
	{
		.start = SPEAR1340_CAM0_BASE,
		.end = SPEAR1340_CAM0_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.name = "line_end_irq",
		.start = SPEAR1340_IRQ_CAM0_CE,
		.flags = IORESOURCE_IRQ,
	}, {
		.name = "frame_start_frame_end_irq",
		.start = SPEAR1340_IRQ_CAM0_FVE,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1340_camif0_device = {
	.name = "spear_camif",
	.id = 0,
	.dev = {
		.coherent_dma_mask = ~0,
		.platform_data = &camif0_platform_data,
	},
	.num_resources = ARRAY_SIZE(camif0_resources),
	.resource = camif0_resources,
};

/* camera interface 1 device registeration */
static struct camif_config_data cam1_data = {
	.sync_type = EXTERNAL_SYNC,
	.vsync_polarity = ACTIVE_HIGH,
	.hsync_polarity = ACTIVE_HIGH,
	.pclk_polarity = ACTIVE_LOW,
	.capture_mode = VIDEO_MODE_ALL_FRAMES,
	.burst_size = BURST_SIZE_128,
	.channel = EVEN_CHANNEL,
	.camif_module_enable = camif_enable,
};

static struct dw_dma_slave camif1_dma_param[] = {
	{
		/* odd line */
		.dma_dev = &spear13xx_dmac_device[1].dev,
		.cfg_hi = DWC_CFGH_SRC_PER(SPEAR1340_DMA_REQ_CAM1_ODD),
		.cfg_lo = 0,
		.src_master = SPEAR1340_DMA_MASTER_CAM,
		.dst_master = SPEAR1340_DMA_MASTER_MEMORY,
	}, {
		/* even line */
		.dma_dev = &spear13xx_dmac_device[1].dev,
		.cfg_hi = DWC_CFGH_SRC_PER(SPEAR1340_DMA_REQ_CAM1_EVEN),
		.cfg_lo = 0,
		.src_master = SPEAR1340_DMA_MASTER_CAM,
		.dst_master = SPEAR1340_DMA_MASTER_MEMORY,
	}
};

static struct camif_controller camif1_platform_data = {
	.dma_filter = dw_dma_filter,
	.dma_odd_param = &camif1_dma_param[0],
	.dma_even_param = &camif1_dma_param[1],
	.config = &cam1_data,
};

static struct resource camif1_resources[] = {
	{
		.start = SPEAR1340_CAM1_BASE,
		.end = SPEAR1340_CAM1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.name = "line_end_irq",
		.start = SPEAR1340_IRQ_CAM1_CE,
		.flags = IORESOURCE_IRQ,
	}, {
		.name = "frame_start_frame_end_irq",
		.start = SPEAR1340_IRQ_CAM1_FVE,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1340_camif1_device = {
	.name = "spear_camif",
	.id = 1,
	.dev = {
		.coherent_dma_mask = ~0,
		.platform_data = &camif1_platform_data,
	},
	.num_resources = ARRAY_SIZE(camif1_resources),
	.resource = camif1_resources,
};

/* camera interface 2 device registeration */
static struct camif_config_data cam2_data = {
	.sync_type = EXTERNAL_SYNC,
	.vsync_polarity = ACTIVE_HIGH,
	.hsync_polarity = ACTIVE_HIGH,
	.pclk_polarity = ACTIVE_LOW,
	.capture_mode = VIDEO_MODE_ALL_FRAMES,
	.burst_size = BURST_SIZE_128,
	.channel = EVEN_CHANNEL,
	.camif_module_enable = camif_enable,
};

static struct dw_dma_slave camif2_dma_param[] = {
	{
		/* odd line */
		.dma_dev = &spear13xx_dmac_device[1].dev,
		.cfg_hi = DWC_CFGH_SRC_PER(SPEAR1340_DMA_REQ_CAM2_ODD),
		.cfg_lo = 0,
		.src_master = SPEAR1340_DMA_MASTER_CAM,
		.dst_master = SPEAR1340_DMA_MASTER_MEMORY,
	}, {
		/* even line */
		.dma_dev = &spear13xx_dmac_device[1].dev,
		.cfg_hi = DWC_CFGH_SRC_PER(SPEAR1340_DMA_REQ_CAM2_EVEN),
		.cfg_lo = 0,
		.src_master = SPEAR1340_DMA_MASTER_CAM,
		.dst_master = SPEAR1340_DMA_MASTER_MEMORY,
	}
};

static struct camif_controller camif2_platform_data = {
	.dma_filter = dw_dma_filter,
	.dma_odd_param = &camif2_dma_param[0],
	.dma_even_param = &camif2_dma_param[1],
	.config = &cam2_data,
};

static struct resource camif2_resources[] = {
	{
		.start = SPEAR1340_CAM2_BASE,
		.end = SPEAR1340_CAM2_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.name = "line_end_irq",
		.start = SPEAR1340_IRQ_CAM2_CE,
		.flags = IORESOURCE_IRQ,
	}, {
		.name = "frame_start_frame_end_irq",
		.start = SPEAR1340_IRQ_CAM2_FVE,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1340_camif2_device = {
	.name = "spear_camif",
	.id = 2,
	.dev = {
		.coherent_dma_mask = ~0,
		.platform_data = &camif2_platform_data,
	},
	.num_resources = ARRAY_SIZE(camif2_resources),
	.resource = camif2_resources,
};


/* camera interface 3 device registeration */
static struct camif_config_data cam3_data = {
	.sync_type = EXTERNAL_SYNC,
	.vsync_polarity = ACTIVE_HIGH,
	.hsync_polarity = ACTIVE_HIGH,
	.pclk_polarity = ACTIVE_LOW,
	.capture_mode = VIDEO_MODE_ALL_FRAMES,
	.burst_size = BURST_SIZE_128,
	.channel = EVEN_CHANNEL,
	.camif_module_enable = camif_enable,
};

static struct dw_dma_slave camif3_dma_param[] = {
	{
		/* odd line */
		.dma_dev = &spear13xx_dmac_device[1].dev,
		.cfg_hi = DWC_CFGH_SRC_PER(SPEAR1340_DMA_REQ_CAM3_ODD),
		.cfg_lo = 0,
		.src_master = SPEAR1340_DMA_MASTER_CAM,
		.dst_master = SPEAR1340_DMA_MASTER_MEMORY,
	}, {
		/* even line */
		.dma_dev = &spear13xx_dmac_device[1].dev,
		.cfg_hi = DWC_CFGH_SRC_PER(SPEAR1340_DMA_REQ_CAM3_EVEN),
		.cfg_lo = 0,
		.src_master = SPEAR1340_DMA_MASTER_CAM,
		.dst_master = SPEAR1340_DMA_MASTER_MEMORY,
	}
};

static struct camif_controller camif3_platform_data = {
	.dma_filter = dw_dma_filter,
	.dma_odd_param = &camif3_dma_param[0],
	.dma_even_param = &camif3_dma_param[1],
	.config = &cam3_data,
};

static struct resource camif3_resources[] = {
	{
		.start = SPEAR1340_CAM3_BASE,
		.end = SPEAR1340_CAM3_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.name = "line_end_irq",
		.start = SPEAR1340_IRQ_CAM3_CE,
		.flags = IORESOURCE_IRQ,
	}, {
		.name = "frame_start_frame_end_irq",
		.start = SPEAR1340_IRQ_CAM3_FVE,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1340_camif3_device = {
	.name = "spear_camif",
	.id = 3,
	.dev = {
		.coherent_dma_mask = ~0,
		.platform_data = &camif3_platform_data,
	},
	.num_resources = ARRAY_SIZE(camif3_resources),
	.resource = camif3_resources,
};

/* CEC device registration */
static struct resource cec0_resources[] = {
	{
		.start = SPEAR1340_CEC0_BASE,
		.end = SPEAR1340_CEC0_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1340_IRQ_CEC0,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1340_cec0_device = {
	.name = "spear_cec",
	.id = 0,
	.num_resources = ARRAY_SIZE(cec0_resources),
	.resource = cec0_resources,
};

/* CEC1 device registration */
static struct resource cec1_resources[] = {
	{
		.start = SPEAR1340_CEC1_BASE,
		.end = SPEAR1340_CEC1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1340_IRQ_CEC1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1340_cec1_device = {
	.name = "spear_cec",
	.id = 1,
	.num_resources = ARRAY_SIZE(cec1_resources),
	.resource = cec1_resources,
};

/* cpufreq platform device */
static u32 cpu_freq_tbl[] = {
	166000, /* 166 MHZ */
	200000, /* 200 MHZ */
	250000, /* 250 MHZ */
	332000, /* 332 MHZ */
	400000, /* 400 MHZ */
	500000, /* 500 MHZ */
	600000, /* 600 MHZ */
};

static struct spear_cpufreq_pdata cpufreq_pdata = {
	.cpu_freq_table = cpu_freq_tbl,
	.tbl_len = ARRAY_SIZE(cpu_freq_tbl),
	/* Program the actual transition time for worstcase */
	.transition_latency = 250 * 1000, /*250 us*/
};

struct platform_device spear1340_cpufreq_device = {
	.name = "cpufreq-spear",
	.id = -1,
	.dev = {
		.platform_data = &cpufreq_pdata,
	},
};

/* SPEAr GPIO Buttons Info */
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
/* SPEAr GPIO Buttons definition */
#define SPEAR_GPIO_BTN9	9

static struct gpio_keys_button spear_gpio_keys_table[] = {
	{
		.code = BTN_0,
		.gpio = SPEAR_GPIO_BTN9,
		.active_low = 0,
		.desc = "gpio-keys: BTN0",
		.type = EV_KEY,
		.wakeup = 1,
		.debounce_interval = 20,
	},
};

static struct gpio_keys_platform_data spear_gpio_keys_data = {
	.buttons = spear_gpio_keys_table,
	.nbuttons = ARRAY_SIZE(spear_gpio_keys_table),
};

struct platform_device spear1340_gpiokeys_device = {
	.name = "gpio-keys",
	.dev = {
		.platform_data = &spear_gpio_keys_data,
	},
};
#endif

static struct resource nand_resources[] = {
	{
		.name = "nand_data",
		.start = SPEAR1340_FSMC_NAND_BASE,
		.end = SPEAR1340_FSMC_NAND_BASE + SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.name = "fsmc_regs",
		.start = SPEAR13XX_FSMC_BASE,
		.end = SPEAR13XX_FSMC_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device spear1340_nand_device = {
	.name = "fsmc-nand",
	.id = -1,
	.resource = nand_resources,
	.num_resources = ARRAY_SIZE(nand_resources),
};

/*
 * This routine does i2c bus recovery as specified in the
 * i2c protocol Rev. 03 section 3.16 titled "Bus clear"
 */
static void spear1340_i2c_dw_recover_bus(void *data)
{
	struct platform_device *pdev = data;
	int i2c_clk_gpio, i = 0, ret = 0;
	int val = 0;
	struct pmx_dev *pmxdev;

	if (pdev->id == 0) {
		i2c_clk_gpio = PLGPIO_134;
		pmxdev = &spear1340_pmx_i2c0;
	} else if (pdev->id == 1) {
		i2c_clk_gpio = PLGPIO_23;
		pmxdev = &spear1340_pmx_i2c1;
	} else {
		pr_err("Invalid I2C device id\n");
		return;
	}

	ret = gpio_request(i2c_clk_gpio, "i2c-sclk");
	if (ret) {
		pr_err("couldn't req gpio %d\n", ret);
		return;
	}

	ret = gpio_direction_output(i2c_clk_gpio, 0);
	if (ret) {
		pr_err("couldn't set i2s-sclk low: %d\n", ret);
		goto free_gpio;
	}

	/* take I2C SLCK control as pl-gpio */
	config_io_pads(&pmxdev, 1, false);

	/* Send high and low on the I2C SCL line */
	for (i = 0; i < 18; i++) {
		gpio_set_value(i2c_clk_gpio, val);
		val = !val;
		msleep(100);
	}

	/* restore I2C SLCK control to I2C controller*/
	config_io_pads(&pmxdev, 1, true);

free_gpio:
	gpio_free(i2c_clk_gpio);

}

/* i2c device registeration */
static struct resource i2c1_resources[] = {
	{
		.start = SPEAR1340_I2C1_BASE,
		.end = SPEAR1340_I2C1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1340_IRQ_I2C1,
		.flags = IORESOURCE_IRQ,
	},
};

static struct i2c_dw_pdata spear1340_i2c_dw_pdata = {
	.i2c_recover_bus = spear1340_i2c_dw_recover_bus,
};

struct platform_device spear1340_i2c1_device = {
	.name = "i2c_designware",
	.id = 1,
	.dev = {
		.coherent_dma_mask = ~0,
		.platform_data = &spear1340_i2c_dw_pdata,
	},
	.num_resources = ARRAY_SIZE(i2c1_resources),
	.resource = i2c1_resources,
};

/* i2s:play device registration */
static struct dw_dma_slave i2s_play_dma_data = {
	/* Play */
	.dma_dev = &spear13xx_dmac_device[0].dev,
	.cfg_hi = DWC_CFGH_DST_PER(SPEAR1340_DMA_REQ_I2S_TX),
	.cfg_lo = 0,
	.src_master = SPEAR1340_DMA_MASTER_MEMORY,
	.dst_master = SPEAR1340_DMA_MASTER_I2S_PLAY,
};

static struct i2s_platform_data i2s_play_data = {
	.cap = PLAY,
	.channel = 8,
	.play_dma_data = &i2s_play_dma_data,
	.snd_fmts = SNDRV_PCM_FMTBIT_S16_LE,
	.snd_rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_48000),
	.filter = dw_dma_filter,
	.i2s_clk_cfg = audio_clk_config,
};

static struct resource i2s_play_resources[] = {
	{
		.start	= SPEAR1340_I2S_PLAY_BASE,
		.end	= SPEAR1340_I2S_PLAY_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	}, {
		.name	= "play_irq",
		.start	= SPEAR1340_IRQ_I2S_PLAY_OR_M,
		.flags	= IORESOURCE_IRQ,
	}
};

struct platform_device spear1340_i2s_play_device = {
	.name = "designware-i2s",
	.id = 0,
	.dev = {
		.coherent_dma_mask = ~0,
		.platform_data = &i2s_play_data,
	},
	.num_resources = ARRAY_SIZE(i2s_play_resources),
	.resource = i2s_play_resources,
};

/* i2s:record device registeration */
static struct dw_dma_slave i2s_capture_dma_data = {
	/* Record */
	.dma_dev = &spear13xx_dmac_device[0].dev,
	.cfg_hi = DWC_CFGH_SRC_PER(SPEAR1340_DMA_REQ_I2S_RX),
	.cfg_lo = 0,
	.src_master = SPEAR1340_DMA_MASTER_I2S_REC,
	.dst_master = SPEAR1340_DMA_MASTER_MEMORY,
};

static struct i2s_platform_data i2s_capture_data = {
	.cap = RECORD,
	.channel = 8,
	.capture_dma_data = &i2s_capture_dma_data,
	.snd_fmts = SNDRV_PCM_FMTBIT_S16_LE,
	.snd_rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_48000),
	.filter = dw_dma_filter,
	.i2s_clk_cfg = audio_clk_config,
};

static struct resource i2s_record_resources[] = {
	{
		.start	= SPEAR1340_I2S_REC_BASE,
		.end	= SPEAR1340_I2S_REC_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	}, {
		.name	= "record_irq",
		.start	= SPEAR1340_IRQ_I2S_REC_OR_S,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device spear1340_i2s_record_device = {
	.name = "designware-i2s",
	.id = 1,
	.dev = {
		.coherent_dma_mask = ~0,
		.platform_data = &i2s_capture_data,
	},
	.num_resources = ARRAY_SIZE(i2s_record_resources),
	.resource = i2s_record_resources,
};

/* plgpio */
static struct plgpio_platform_data plgpio_plat_data = {
	.gpio_base = 16,
	.irq_base = SPEAR_PLGPIO_INT_BASE,
	.gpio_count = SPEAR_PLGPIO_COUNT,
	.regs = {
		.enb = -1,
		.wdata = SPEAR1340_PLGPIO_WDATA_OFF,
		.dir = SPEAR1340_PLGPIO_DIR_OFF,
		.rdata = SPEAR1340_PLGPIO_RDATA_OFF,
		.ie = SPEAR1340_PLGPIO_IE_OFF,
		.mis = SPEAR1340_PLGPIO_MIS_OFF,
		.eit = SPEAR1340_PLGPIO_EIT_OFF,
	},
};

static struct resource plgpio_resources[] = {
	{
		.start = SPEAR1340_PLGPIO_BASE,
		.end = SPEAR1340_PLGPIO_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1340_IRQ_PLGPIO,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1340_plgpio_device = {
	.name = "plgpio",
	.id = -1,
	.dev = {
		.platform_data = &plgpio_plat_data,
	},
	.num_resources = ARRAY_SIZE(plgpio_resources),
	.resource = plgpio_resources,
};

/* pwm device registeration */
static struct resource pwm_resources[] = {
	{
		.start = SPEAR1340_PWM_BASE,
		.end = SPEAR1340_PWM_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device spear1340_pwm_device = {
	.name = "pwm",
	.id = -1,
	.num_resources = ARRAY_SIZE(pwm_resources),
	.resource = pwm_resources,
};

/* SATA device registration */
void sata_miphy_exit(struct device *dev)
{
	writel(0, VA_SPEAR1340_PCIE_SATA_CFG);
	writel(0, VA_SPEAR1340_PCIE_MIPHY_CFG);

	/* Enable PCIE SATA Controller reset */
	writel((readl(VA_SPEAR1340_PERIP1_SW_RST) | (0x1000)),
			VA_SPEAR1340_PERIP1_SW_RST);
	msleep(20);
	/* Switch off sata power domain */
	writel((readl(VA_SPEAR1340_PCM_CFG) & (~0x800)),
			VA_SPEAR1340_PCM_CFG);
	msleep(20);
}

static int sata_miphy_init(struct device *dev, void __iomem *addr)
{
	writel(SPEAR1340_SATA_CFG_VAL, VA_SPEAR1340_PCIE_SATA_CFG);
	writel(SPEAR1340_PCIE_SATA_MIPHY_CFG_SATA_25M_CRYSTAL_CLK,
			VA_SPEAR1340_PCIE_MIPHY_CFG);
	/* Switch on sata power domain */
	writel((readl(VA_SPEAR1340_PCM_CFG) | (0x800)),
			VA_SPEAR1340_PCM_CFG);
	msleep(20);
	/* Disable PCIE SATA Controller reset */
	writel((readl(VA_SPEAR1340_PERIP1_SW_RST) & (~0x1000)),
			VA_SPEAR1340_PERIP1_SW_RST);
	msleep(20);

	return 0;
}

int sata_suspend(struct device *dev)
{
	if (dev->power.power_state.event == PM_EVENT_FREEZE)
		return 0;

	sata_miphy_exit(dev);

	return 0;
}

int sata_resume(struct device *dev)
{
	if (dev->power.power_state.event == PM_EVENT_THAW)
		return 0;

	return sata_miphy_init(dev, NULL);
}

static struct resource sata_resources[] = {
	{
		.start = SPEAR1340_SATA_BASE,
		.end = SPEAR1340_SATA_BASE + SZ_8K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1340_IRQ_SATA,
		.flags = IORESOURCE_IRQ,
	},
};

static struct ahci_platform_data sata_pdata = {
	.init = sata_miphy_init,
	.exit = sata_miphy_exit,
	.suspend = sata_suspend,
	.resume = sata_resume,
};

static u64 ahci_dmamask = DMA_BIT_MASK(32);

struct platform_device spear1340_sata0_device = {
	.name = "ahci",
	.id = -1,
	.num_resources = ARRAY_SIZE(sata_resources),
	.resource = sata_resources,
	.dev = {
		.platform_data = &sata_pdata,
		.dma_mask = &ahci_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

/* spdif-in device registeration */
static void spdif_in_reset(void)
{
	writel(readl(VA_SPEAR1340_PERIP3_SW_RST) | SPEAR1340_SPDIF_IN_RST,
		VA_SPEAR1340_PERIP3_SW_RST);

	writel(readl(VA_SPEAR1340_PERIP3_SW_RST) & ~SPEAR1340_SPDIF_IN_RST,
		VA_SPEAR1340_PERIP3_SW_RST);
}

static struct dw_dma_slave spdif_in_dma_data = {
	/* Record */
	.dma_dev = &spear13xx_dmac_device[0].dev,
	.cfg_hi = DWC_CFGH_SRC_PER(SPEAR1340_DMA_REQ_SPDIF_RX),
	.cfg_lo = 0,
	.src_master = SPEAR1340_DMA_MASTER_SPDIF,
	.dst_master = SPEAR1340_DMA_MASTER_MEMORY,
};

static struct spdif_platform_data spdif_in_data = {
	.dma_params = &spdif_in_dma_data,
	.filter = dw_dma_filter,
	.reset_perip = spdif_in_reset,
};

static struct resource spdif_in_resources[] = {
	{
		.start = SPEAR1340_SPDIF_IN_BASE,
		.end = SPEAR1340_SPDIF_IN_BASE + SZ_128 - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1340_SPDIF_IN_FIFO_BASE,
		.end = SPEAR1340_SPDIF_IN_FIFO_BASE + SZ_64 - 1,
		.flags = IORESOURCE_IO,
	}, {
		.start = SPEAR1340_IRQ_SPDIF_IN,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1340_spdif_in_device = {
	.name = "spdif-in",
	.id = -1,
	.dev.platform_data = &spdif_in_data,
	.num_resources = ARRAY_SIZE(spdif_in_resources),
	.resource = spdif_in_resources,
};

/* spdif-out device registeration */
static struct dw_dma_slave spdif_out_dma_data = {
	/* Play */
	.dma_dev = &spear13xx_dmac_device[0].dev,
	.cfg_hi = DWC_CFGH_DST_PER(SPEAR1340_DMA_REQ_SPDIF_TX),
	.cfg_lo = 0,
	.src_master = SPEAR1340_DMA_MASTER_MEMORY,
	.dst_master = SPEAR1340_DMA_MASTER_SPDIF,
};

static struct spdif_platform_data spdif_out_data = {
	.dma_params = &spdif_out_dma_data,
	.filter = dw_dma_filter,
};

static struct resource spdif_out_resources[] = {
	{
		.start = SPEAR1340_SPDIF_OUT_BASE,
		.end = SPEAR1340_SPDIF_OUT_BASE + SZ_128 - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1340_IRQ_SPDIF_OUT,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1340_spdif_out_device = {
	.name = "spdif-out",
	.id = -1,
	.dev.platform_data = &spdif_out_data,
	.num_resources = ARRAY_SIZE(spdif_out_resources),
	.resource = spdif_out_resources,
};

/* SPEAr Thermal Sensor Platform Data for 1340 */
static struct resource spear1340_thermal_resources[] = {
	{
		.start = SPEAR1340_THSENS_CFG,
		.end = SPEAR1340_THSENS_CFG + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct spear_thermal_pdata spear1340_thermal_pdata = {
	.thermal_flags = SPEAR1340_THERMAL_CONFIG_FLAGS,
};

struct platform_device spear1340_thermal_device = {
	.name = "spear_thermal",
	.id = -1,
	.dev = {
		.platform_data = &spear1340_thermal_pdata,
	},
	.num_resources = ARRAY_SIZE(spear1340_thermal_resources),
	.resource = spear1340_thermal_resources,
};

static int spear1340_otg_phy_init(void)
{
	u32 temp, msec = 1000;

	/* phy por deassert */
	temp = readl(VA_SPEAR1340_USBPHY_GEN_CFG);
	temp &= ~SPEAR1340_USBPHYPOR;
	writel(temp, VA_SPEAR1340_USBPHY_GEN_CFG);

	/* phy clock enable */
	temp = readl(VA_SPEAR1340_USBPHY_GEN_CFG);
	temp |= SPEAR1340_USBPHYRST;
	writel(temp, VA_SPEAR1340_USBPHY_GEN_CFG);

	/* wait for pll lock */
	while (!(readl(VA_SPEAR1340_USBPHY_GEN_CFG) & SPEAR1340_USBPLLLOCK)) {
		if (msec--) {
			pr_err(" Problem with USB PHY PLL Lock\n");
			return -ETIMEDOUT;
		}
		udelay(1);
	}

	/* otg prstnt deassert */
	temp = readl(VA_SPEAR1340_USBPHY_GEN_CFG);
	temp |= SPEAR1340_USBPRSNT;
	writel(temp, VA_SPEAR1340_USBPHY_GEN_CFG);

	/* OTG HCLK Disable */
	temp = readl(VA_SPEAR1340_PERIP1_CLK_ENB);
	temp &= ~(1 << SPEAR1340_UOC_CLK_ENB);
	writel(temp, VA_SPEAR1340_PERIP1_CLK_ENB);

	/* OTG HRESET deassert */
	temp = readl(VA_SPEAR1340_PERIP1_SW_RST);
	temp &= ~(1 << SPEAR1340_UOC_RST_ENB);
	writel(temp, VA_SPEAR1340_PERIP1_SW_RST);

	/* OTG HCLK Enable */
	temp = readl(VA_SPEAR1340_PERIP1_CLK_ENB);
	temp |= (1 << SPEAR1340_UOC_CLK_ENB);
	writel(temp, VA_SPEAR1340_PERIP1_CLK_ENB);

	return 0;
}

static int spear1340_otg_param_init(struct core_params *params)
{
	int i;

	/* Common Dev RX fifo Size : 0x400 */
	params->dev_rx_fifo_size = 0x400;
	/* Dev TX fifo Size for fifo 0: 0x300 */
	params->dev_nperio_tx_fifo_size = 0x300;
	/* TX fifo Size for fifo 1-7: 0x200 */
	params->fifo_number = 7;
	for (i = 1; i <= 7; i++)
		params->dev_tx_fifo_size[i - 1] = 0x200;

	/* Common Host RX fifo Size : 0x400 */
	params->host_rx_fifo_size = 0x400;
	/* Host TX fifo Size for fifo 0: 0x400 */
	params->host_nperio_tx_fifo_size = 0x400;
	/* Host Periodic TX fifo Size for fifo 0: 0x400 */
	params->host_perio_tx_fifo_size = 0x400;

	return 0;
}

/* OTG device registration */
static struct resource otg_resources[] = {
	{
		.start = SPEAR1340_UOC_BASE,
		.end = SPEAR1340_UOC_BASE + SZ_256K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1340_IRQ_UOC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct dwc_otg_plat_data otg_platform_data = {
	.phy_init = spear1340_otg_phy_init,
	.param_init = spear1340_otg_param_init,
};

static u64 otg_dmamask = ~0;
struct platform_device spear1340_otg_device = {
	.name = "dwc_otg",
	.id = -1,
	.dev = {
		.coherent_dma_mask = ~0,
		.dma_mask = &otg_dmamask,
		.platform_data = &otg_platform_data,
	},
	.num_resources = ARRAY_SIZE(otg_resources),
	.resource = otg_resources,
};

/* video input parallel port registeration */
static unsigned long vb_base;
static unsigned long vb_size = VIP_TOTAL_BUFFER_SIZE;

unsigned long vip_buffer_fixup(struct meminfo *mi)
{
	vb_base = reserve_mem(mi, ALIGN(vb_size, SZ_1M));
	if (vb_base == ~0) {
		pr_err("Unable to allocate videobufs in fixup\n");
		return -1;
	}

	return 0;
}

void vip_set_vb_base(struct vip_plat_data *pdata)
{
	pdata->vb_base = vb_base;
	pdata->vb_size = vb_size;
}

static struct resource vip_resources[] = {
	{
		.start = SPEAR1340_VIP_BASE,
		.end = SPEAR1340_VIP_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.name = "fifo_overflow_irq",
		.start = SPEAR1340_IRQ_VIP,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1340_vip_device = {
	.name = "spear_vip",
	.id = -1,
	.dev = {
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(vip_resources),
	.resource = vip_resources,
};

#ifdef CONFIG_DRM_MALI
struct platform_device spear1340_device_mali_drm = {
        .name = "mali_drm",
        .id   = -1,
};
#endif

static struct resource spear1340_video_dec_resources[] = {
	{
		.start = SPEAR1340_VIDEO_DEC_BASE,
		.end =  SPEAR1340_VIDEO_DEC_BASE + (4*100),
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1340_IRQ_VIDEO_DEC,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1340_video_dec_device = {
	.name = "video_dec",
	.id   = -1,
	.num_resources = ARRAY_SIZE(spear1340_video_dec_resources),
	.resource = spear1340_video_dec_resources,
};

static int spear1340_sys_clk_init(void)
{
	struct clk *sys_pclk, *ahb_pclk, *sys_clk, *ahb_clk;
	int ret = 0;
	const char *sys_clk_src[] = {
		"sys_synth_clk",
		"pll1_clk",
		"pll2_clk",
		"pll3_clk",
	};
	const char *ahb_clk_src[] = {
		"amba_synth_clk",
		"cpu_clk_div3",
	};

	/*
	 * If we need to run cpu at 600 MHz we need to have an alternate
	 * pll (other than pll1)
	 * So,
	 * pll1 - would be used to feed all synthesizers and AHB
	 * (through AHB synthesizer)
	 * pll2 - would feed gmac (for 125 MHz)
	 * pll3 - would be used to feed cpu
	 *
	 * Note: I2S which was using PLL3 as the source clock is
	 * impacted by this. The possibility is to use an external
	 * oscillator (not preset on board as of now) for I2S.
	 */
	sys_pclk = clk_get(NULL, sys_clk_src[3]);
	if (IS_ERR(sys_pclk)) {
		ret = PTR_ERR(sys_pclk);
		goto fail_get_sys_pclk;
	}

	/* Get the amba synthesizer as the clock source for ahb clock */
	ahb_pclk = clk_get(NULL, ahb_clk_src[0]);
	if (IS_ERR(ahb_pclk)) {
		ret = PTR_ERR(ahb_pclk);
		goto fail_get_ahb_pclk;
	}

	/* Get the system clock */
	sys_clk = clk_get(NULL, "sys_clk");
	if (IS_ERR(sys_clk)) {
		ret = PTR_ERR(sys_clk);
		goto fail_get_sysclk;
	}

	/* Get the ahb clock */
	ahb_clk = clk_get(NULL, "ahb_clk");
	if (IS_ERR(ahb_clk)) {
		ret = PTR_ERR(ahb_clk);
		goto fail_get_ahb_clk;
	}

	if (clk_set_rate(sys_pclk, 1200000000)) {
		ret = -EPERM;
		pr_err("SPEAr1340: Failed to set system clock rate\n");
		goto fail_sys_set_rate;
	}

	if (clk_set_rate(ahb_pclk, 166000000)) {
		ret = -EPERM;
		pr_err("SPEAr1340: Failed to set AHB rate\n");
		goto fail_sys_set_rate;
	}

	if (clk_enable(sys_pclk)) {
		ret = -ENODEV;
		goto fail_sys_set_rate;
	}

	if (clk_enable(ahb_pclk)) {
		ret = -ENODEV;
		goto fail_en_sys_clk;
	}

	/* Set the amba synth as parent for ahb clk */
	if (clk_set_parent(ahb_clk, ahb_pclk)) {
		ret = -EPERM;
		pr_err("SPEAr1340: Failed to set AHB parent\n");
		goto fail_ahb_set_parent;
	}

	/* Set the sys synth as parent for sys clk */
	if (clk_set_parent(sys_clk, sys_pclk)) {
		ret = -EPERM;
		pr_err("SPEAr1340: Failed to set sys clk parent\n");
		goto fail_sys_set_parent;
	}

	/* put back all the clocks */
	goto fail_sys_set_rate;

fail_sys_set_parent:
fail_ahb_set_parent:
	clk_disable(ahb_pclk);
fail_en_sys_clk:
	clk_disable(sys_pclk);
fail_sys_set_rate:
	clk_put(ahb_clk);
fail_get_ahb_clk:
	clk_put(sys_clk);
fail_get_sysclk:
	clk_put(ahb_pclk);
fail_get_ahb_pclk:
	clk_put(sys_pclk);
fail_get_sys_pclk:
	return ret;
}

/* gpio Configure in pull-down mode for clcd */
static struct pmx_mux_reg gpio_pd_mux[] = {
	{
		.address = SPEAR1340_PAD_PU_CFG_5,
		.mask = 0xFFFFFC00,
		.value = 0XFFFFFC00,
	}, {
		.address = SPEAR1340_PAD_PU_CFG_6,
		.mask = 0xFFFFFFFF,
		.value = 0XFFFFFFFF,
	}, {
		.address = SPEAR1340_PAD_PD_CFG_5,
		.mask = 0xFFFFFC00,
		.value = ~0XFFFFFC00,
	}, {
		.address = SPEAR1340_PAD_PD_CFG_6,
		.mask = 0xFFFFFFFF,
		.value = ~0XFFFFFFFF,
	},
};

static struct pmx_dev_mode gpio_pd_modes[] = {
	{
		.mux_regs = gpio_pd_mux,
		.mux_reg_cnt = ARRAY_SIZE(gpio_pd_mux),
	},
};

struct pmx_dev spear1340_pmx_clcd_gpio_pd = {
	.name = "gpio",
	.modes = gpio_pd_modes,
	.mode_count = ARRAY_SIZE(gpio_pd_modes),
};

/* clcd/gpio pad selection */
static struct pmx_mux_reg clcd_plgpios_mux[] = {
	{
		.address = SPEAR1340_PAD_FUNCTION_EN_5,
		.mask = 0xFFFFF800,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_6,
		.mask = 0xFFFFFFFF,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_7,
		.mask = 0x00000001,
	},

};

static struct pmx_dev_mode clcd_plgpios_modes[] = {
	{
		.mux_regs = clcd_plgpios_mux,
		.mux_reg_cnt = ARRAY_SIZE(clcd_plgpios_mux),
	},
};

struct pmx_dev clcd_pmx_plgpios = {
	.name = "clcd",
	.modes = clcd_plgpios_modes,
	.mode_count = ARRAY_SIZE(clcd_plgpios_modes),
};

static struct pmx_dev *clcd_pmx_devs[] = {
	&clcd_pmx_plgpios,
};

static struct gpio_req_list clcd_gpio_list[] = {
	{
		.start = PLGPIO_138,
		.end = PLGPIO_191,
	},
};

/* Switch pads to plgpio or clcd now */
void config_clcd_gpio_pads(bool on)
{
	int ret;
	static bool gpio_avail;

	if (!gpio_avail) {
		ret = request_gpio(clcd_gpio_list, GPIOF_IN,
				ARRAY_SIZE(clcd_gpio_list));
		if (ret)
			pr_err("clcd request for gpio is failed. err no:%d\n",
					ret);

		gpio_avail = true;
	}

	config_io_pads(clcd_pmx_devs, ARRAY_SIZE(clcd_pmx_devs), on);
}

#ifdef CONFIG_SND_SPEAR_SPDIF_IN
static int spdif_in_clk_init(void)
{
	int ret;

	ret = clk_set_parent_sys("gen_synth3_clk", NULL, NULL, "vco1div4_clk");
	if (ret)
		return ret;

	ret = clk_set_parent_sys("spdif-in", NULL, "gen_synth3_clk", NULL);
	if (ret)
		return ret;

	/* Set SPDIF IN for a fixed clock = 200 MHz */
	return clk_set_rate_sys("spdif-in", NULL, 200000000);
}
#endif

#ifdef CONFIG_SND_SPEAR_SPDIF_OUT
static int spdif_out_clk_init(void)
{
	int ret;

	ret = clk_set_parent_sys("gen_synth2_clk", NULL, NULL, "vco1div4_clk");
	if (ret)
		return ret;

	return clk_set_parent_sys("spdif-out", NULL, "gen_synth2_clk", NULL);
}
#endif

#ifdef CONFIG_SPEAR_PCIE_REV370
/* This function is needed for board specific PCIe initilization */
void __init spear1340_pcie_board_init(struct device *dev)
{
	void *plat_data;

	plat_data = dev_get_platdata(dev);
	PCIE_PORT_INIT((struct pcie_port_info *)plat_data, SPEAR_PCIE_REV_3_70);
}
#endif

void __init spear1340_init(struct pmx_mode *pmx_mode, struct pmx_dev **pmx_devs,
		u8 pmx_dev_count)
{
	int ret;

	/* call spear13xx family common init function */
	spear13xx_init();

	/* call spear1340 system clock init function */
	ret = spear1340_sys_clk_init();
	if (ret)
		pr_err("SPEAr1340: sysclock init failed, err no: %d\n", ret);

#ifdef CONFIG_SND_SPEAR_SPDIF_IN
	/* call spdif in clock init */
	ret = spdif_in_clk_init();
	if (ret)
		pr_err("SPEAr1340: spdif in clock init failed, err no: %d\n",
				ret);
#endif

#ifdef CONFIG_SND_SPEAR_SPDIF_OUT
	/* call spdif out clock init */
	ret = spdif_out_clk_init();
	if (ret)
		pr_err("SPEAr1340: spdif out clock init failed, err no: %d\n",
				ret);
#endif
	spear13xx_i2c_device.dev.platform_data = &spear1340_i2c_dw_pdata;

	/* pmx initialization */
	pmx_driver.mode = pmx_mode;
	pmx_driver.devs = pmx_devs;
	pmx_driver.devs_count = pmx_dev_count;

	ret = pmx_register(&pmx_driver);
	if (ret)
		pr_err("padmux: registeration failed. err no: %d\n", ret);
}
