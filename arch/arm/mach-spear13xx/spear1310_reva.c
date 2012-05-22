/*
 * arch/arm/mach-spear13xx/spear1310_reva.c
 *
 * SPEAr1310 machine source file
 *
 * Copyright (C) 2010 ST Microelectronics
 * Bhupesh Sharma <bhupesh.sharma@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/can/platform/c_can.h>
#include <linux/clk.h>
#include <linux/netdevice.h>
#include <linux/ptrace.h>
#include <linux/phy.h>
#include <linux/stmmac.h>
#include <asm/irq.h>
#include <asm/delay.h>
#include <plat/hdlc.h>
#include <mach/generic.h>
#include <mach/gpio.h>
#include <mach/hardware.h>
#include <mach/misc_regs.h>

/* pmx driver structure */
static struct pmx_driver pmx_driver;

/* Pad multiplexing for uart1_modem device */
static struct pmx_mux_reg pmx_uart1_modem_mux[] = {
	{
		.address = PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_I2S1_MASK | SPEAR13XX_PMX_SSP_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_uart1_modem_modes[] = {
	{
		.mux_regs = pmx_uart1_modem_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart1_modem_mux),
	},
};

struct pmx_dev spear1310_reva_pmx_uart1_modem = {
	.name = "uart1_modem",
	.modes = pmx_uart1_modem_modes,
	.mode_count = ARRAY_SIZE(pmx_uart1_modem_modes),
};

/* Pad multiplexing for uart1 device */
static struct pmx_mux_reg pmx_uart1_mux[] = {
	{
		.address = PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_SSP_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_uart1_modes[] = {
	{
		.mux_regs = pmx_uart1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart1_mux),
	},
};

struct pmx_dev spear1310_reva_pmx_uart_1 = {
	.name = "uart1",
	.modes = pmx_uart1_modes,
	.mode_count = ARRAY_SIZE(pmx_uart1_modes),
};

/* Pad multiplexing for uart2 device */
static struct pmx_mux_reg pmx_uart2_mux[] = {
	{
		.address = PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_SSP_MASK | SPEAR13XX_PMX_CLCD1_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_uart2_modes[] = {
	{
		.mux_regs = pmx_uart2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart2_mux),
	},
};

struct pmx_dev spear1310_reva_pmx_uart_2 = {
	.name = "uart2",
	.modes = pmx_uart2_modes,
	.mode_count = ARRAY_SIZE(pmx_uart2_modes),
};

/* Pad multiplexing for uart_3_4_5 device */
static struct pmx_mux_reg pmx_uart_3_4_5_mux[] = {
	{
		.address = PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_CLCD1_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_uart_3_4_5_modes[] = {
	{
		.mux_regs = pmx_uart_3_4_5_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart_3_4_5_mux),
	},
};

struct pmx_dev spear1310_reva_pmx_uart_3_4_5 = {
	.name = "uart_3_4_5",
	.modes = pmx_uart_3_4_5_modes,
	.mode_count = ARRAY_SIZE(pmx_uart_3_4_5_modes),
};

/* Pad multiplexing for rs485_hdlc_1_2 device */
static struct pmx_mux_reg pmx_rs485_hdlc_1_2_mux[] = {
	{
		.address = PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_CLCD1_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_rs485_hdlc_1_2_modes[] = {
	{
		.mux_regs = pmx_rs485_hdlc_1_2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_rs485_hdlc_1_2_mux),
	},
};

struct pmx_dev spear1310_reva_pmx_rs485_hdlc_1_2 = {
	.name = "rs485_hdlc_1_2",
	.modes = pmx_rs485_hdlc_1_2_modes,
	.mode_count = ARRAY_SIZE(pmx_rs485_hdlc_1_2_modes),
};

/* Pad multiplexing for tdm_hdlc_1_2 device */
static struct pmx_mux_reg pmx_tdm_hdlc_1_2_mux[] = {
	{
		.address = PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_CLCD1_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_tdm_hdlc_1_2_modes[] = {
	{
		.mux_regs = pmx_tdm_hdlc_1_2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_tdm_hdlc_1_2_mux),
	},
};

struct pmx_dev spear1310_reva_pmx_tdm_hdlc_1_2 = {
	.name = "tdm_hdlc_1_2",
	.modes = pmx_tdm_hdlc_1_2_modes,
	.mode_count = ARRAY_SIZE(pmx_tdm_hdlc_1_2_modes),
};

/* Pad multiplexing for fsmc32bit device */
static struct pmx_mux_reg pmx_fsmc32bit_mux[] = {
	{
		.address = PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_EGPIO_0_GRP_MASK | \
			SPEAR13XX_PMX_SMI_MASK | SPEAR13XX_PMX_CLCD1_MASK | \
			SPEAR13XX_PMX_NAND16BIT4DEV_0_MASK,
		.value = 0,
	}, {
		.address = PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_KEYBOARD_6X6_MASK | \
			SPEAR13XX_PMX_NAND16BIT4DEV_1_MASK,
		.value = 0,
	}, {
		.address = PCM_CFG,
		.mask = SPEAR1310_REVA_PMX_EGPIO7_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_fsmc32bit_modes[] = {
	{
		.mux_regs = pmx_fsmc32bit_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_fsmc32bit_mux),
	},
};

struct pmx_dev spear1310_reva_pmx_fsmc32bit_4_chips = {
	.name = "fsmc32bit",
	.modes = pmx_fsmc32bit_modes,
	.mode_count = ARRAY_SIZE(pmx_fsmc32bit_modes),
};

/* Pad multiplexing for fsmc16bit device */
static struct pmx_mux_reg pmx_fsmc16bit_mux[] = {
	{
		.address = PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_NAND16BIT4DEV_0_MASK,
		.value = 0,
	}, {
		.address = PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_KEYBOARD_6X6_MASK | \
			SPEAR13XX_PMX_NAND16BIT4DEV_1_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_fsmc16bit_modes[] = {
	{
		.mux_regs = pmx_fsmc16bit_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_fsmc16bit_mux),
	},
};

struct pmx_dev spear1310_reva_pmx_fsmc16bit_4_chips = {
	.name = "fsmc16bit",
	.modes = pmx_fsmc16bit_modes,
	.mode_count = ARRAY_SIZE(pmx_fsmc16bit_modes),
};

/* Pad multiplexing for gmii1 device */
static struct pmx_mux_reg pmx_gmii1_mux[] = {
	{
		.address = PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_GMII_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_gmii1_modes[] = {
	{
		.mux_regs = pmx_gmii1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_gmii1_mux),
	},
};

struct pmx_dev spear1310_reva_pmx_gmii1 = {
	.name = "gmii1",
	.modes = pmx_gmii1_modes,
	.mode_count = ARRAY_SIZE(pmx_gmii1_modes),
};

/* Pad multiplexing for rgmii device */
static struct pmx_mux_reg pmx_rgmii_mux[] = {
	{
		.address = PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_GMII_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_rgmii_modes[] = {
	{
		.mux_regs = pmx_rgmii_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_rgmii_mux),
	},
};

struct pmx_dev spear1310_reva_pmx_rgmii = {
	.name = "rgmii",
	.modes = pmx_rgmii_modes,
	.mode_count = ARRAY_SIZE(pmx_rgmii_modes),
};

/* Pad multiplexing for i2c1 device */
static struct pmx_mux_reg pmx_i2c1_mux[] = {
	{
		.address = PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_SMINCS2_MASK | SPEAR13XX_PMX_SMINCS3_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_i2c1_modes[] = {
	{
		.mux_regs = pmx_i2c1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_i2c1_mux),
	},
};

struct pmx_dev spear1310_reva_pmx_i2c1 = {
	.name = "i2c1",
	.modes = pmx_i2c1_modes,
	.mode_count = ARRAY_SIZE(pmx_i2c1_modes),
};

/* Pad multiplexing for smii_0_1_2 device */
static struct pmx_mux_reg pmx_smii_0_1_2_mux[] = {
	{
		.address = PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_CLCD2_MASK | \
			SPEAR13XX_PMX_KBD_ROWCOL68_MASK | \
			SPEAR13XX_PMX_EGPIO_1_GRP_MASK | \
			SPEAR13XX_PMX_GPT0_TMR1_MASK | \
			SPEAR13XX_PMX_GPT0_TMR2_MASK | \
			SPEAR13XX_PMX_GPT1_TMR1_MASK | \
			SPEAR13XX_PMX_GPT1_TMR2_MASK,
		.value = 0,
	}, {
		.address = SPEAR1310_REVA_FUNC_CNTL_0,
		.mask = SPEAR1310_REVA_PMX_SMII_MASK,
		.value = SPEAR1310_REVA_PMX_SMII_MASK,
	},
};

static struct pmx_dev_mode pmx_smii_0_1_2_modes[] = {
	{
		.mux_regs = pmx_smii_0_1_2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_smii_0_1_2_mux),
	},
};

struct pmx_dev spear1310_reva_pmx_smii_0_1_2 = {
	.name = "smii_0_1_2",
	.modes = pmx_smii_0_1_2_modes,
	.mode_count = ARRAY_SIZE(pmx_smii_0_1_2_modes),
};

/* Pad multiplexing for pci1 device */
static struct pmx_mux_reg pmx_pci1_mux[] = {
	{
		.address = PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_CLCD2_MASK | \
			SPEAR13XX_PMX_KBD_ROWCOL68_MASK | \
			SPEAR13XX_PMX_EGPIO_1_GRP_MASK | \
			SPEAR13XX_PMX_GPT0_TMR1_MASK | \
			SPEAR13XX_PMX_GPT0_TMR2_MASK | \
			SPEAR13XX_PMX_GPT1_TMR1_MASK | \
			SPEAR13XX_PMX_GPT1_TMR2_MASK,
		.value = 0,
	}, {
		.address = SPEAR1310_REVA_FUNC_CNTL_0,
		.mask = SPEAR1310_REVA_PMX_SMII_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_pci1_modes[] = {
	{
		.mux_regs = pmx_pci1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_pci1_mux),
	},
};

struct pmx_dev spear1310_reva_pmx_pci1 = {
	.name = "pci1",
	.modes = pmx_pci1_modes,
	.mode_count = ARRAY_SIZE(pmx_pci1_modes),
};

/* Pad multiplexing for can device */
static struct pmx_mux_reg pmx_can_mux[] = {
	{
		.address = PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_I2S2_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_can_modes[] = {
	{
		.mux_regs = pmx_can_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_can_mux),
	},
};

struct pmx_dev spear1310_reva_pmx_can = {
	.name = "can",
	.modes = pmx_can_modes,
	.mode_count = ARRAY_SIZE(pmx_can_modes),
};

/* Add spear1310_reva specific devices here */
/* uart1 device registeration */
struct amba_device spear1310_reva_uart1_device = {
	.dev = {
		.init_name = "uart1",
	},
	.res = {
		.start = SPEAR1310_REVA_UART1_BASE,
		.end = SPEAR1310_REVA_UART1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {SPEAR1310_REVA_IRQ_UART1, NO_IRQ},
};

/* uart2 device registeration */
struct amba_device spear1310_reva_uart2_device = {
	.dev = {
		.init_name = "uart2",
	},
	.res = {
		.start = SPEAR1310_REVA_UART2_BASE,
		.end = SPEAR1310_REVA_UART2_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {SPEAR1310_REVA_IRQ_UART2, NO_IRQ},
};

/* uart3 device registeration */
struct amba_device spear1310_reva_uart3_device = {
	.dev = {
		.init_name = "uart3",
	},
	.res = {
		.start = SPEAR1310_REVA_UART3_BASE,
		.end = SPEAR1310_REVA_UART3_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {SPEAR1310_REVA_IRQ_UART3, NO_IRQ},
};

/* uart4 device registeration */
struct amba_device spear1310_reva_uart4_device = {
	.dev = {
		.init_name = "uart4",
	},
	.res = {
		.start = SPEAR1310_REVA_UART4_BASE,
		.end = SPEAR1310_REVA_UART4_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {SPEAR1310_REVA_IRQ_UART4, NO_IRQ},
};

/* uart5 device registeration */
struct amba_device spear1310_reva_uart5_device = {
	.dev = {
		.init_name = "uart5",
	},
	.res = {
		.start = SPEAR1310_REVA_UART5_BASE,
		.end = SPEAR1310_REVA_UART5_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {SPEAR1310_REVA_IRQ_UART5, NO_IRQ},
};

/* CAN device registeration */
static struct c_can_platform_data can0_pdata = {
	.is_quirk_required = true,
	.devtype_data = {
		.rx_first = 1,
		.rx_split = 25,
		.rx_last = 31,
		.tx_num = 1,
	},
};

static struct resource can0_resources[] = {
	{
		.start = SPEAR1310_REVA_CAN0_BASE,
		.end = SPEAR1310_REVA_CAN0_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1310_REVA_IRQ_CCAN0,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1310_reva_can0_device = {
	.name = "c_can_platform",
	.id = 0,
	.num_resources = ARRAY_SIZE(can0_resources),
	.resource = can0_resources,
	.dev.platform_data = &can0_pdata,
};

static struct c_can_platform_data can1_pdata = {
	.is_quirk_required = true,
	.devtype_data = {
		.rx_first = 1,
		.rx_split = 25,
		.rx_last = 31,
		.tx_num = 1,
	},
};

static struct resource can1_resources[] = {
	{
		.start = SPEAR1310_REVA_CAN1_BASE,
		.end = SPEAR1310_REVA_CAN1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1310_REVA_IRQ_CCAN1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1310_reva_can1_device = {
	.name = "c_can_platform",
	.id = 1,
	.num_resources = ARRAY_SIZE(can1_resources),
	.resource = can1_resources,
	.dev.platform_data = &can1_pdata,
};

static struct resource eth1_resources[] = {
	[0] = {
		.start = SPEAR1310_REVA_GETH1_BASE,
		.end = SPEAR1310_REVA_GETH1_BASE + SZ_32K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = SPEAR1310_REVA_IRQ_GETH1_SBD,
		.flags = IORESOURCE_IRQ,
		.name = "macirq",
	},
	[2] = {
		.start = SPEAR1310_REVA_IRQ_GETH1_PMT,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 eth1_dma_mask = ~(u32) 0;
struct platform_device spear1310_reva_eth1_device = {
	.name = "stmmaceth",
	.id = 1,
	.num_resources = ARRAY_SIZE(eth1_resources),
	.resource = eth1_resources,
	.dev = {
		.dma_mask = &eth1_dma_mask,
		.coherent_dma_mask = ~0,
	},
};

/* Ethernet GETH-2 device registeration */
static struct resource eth2_resources[] = {
	[0] = {
		.start = SPEAR1310_REVA_GETH2_BASE,
		.end = SPEAR1310_REVA_GETH2_BASE + SZ_32K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = SPEAR1310_REVA_IRQ_GETH2_SBD,
		.flags = IORESOURCE_IRQ,
		.name = "macirq",
	},
	[2] = {
		.start = SPEAR1310_REVA_IRQ_GETH2_PMT,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 eth2_dma_mask = ~(u32) 0;
struct platform_device spear1310_reva_eth2_device = {
	.name = "stmmaceth",
	.id = 2,
	.num_resources = ARRAY_SIZE(eth2_resources),
	.resource = eth2_resources,
	.dev = {
		.dma_mask = &eth2_dma_mask,
		.coherent_dma_mask = ~0,
	},
};

/* Ethernet GETH-3 device registeration */
static struct resource eth3_resources[] = {
	[0] = {
		.start = SPEAR1310_REVA_GETH3_BASE,
		.end = SPEAR1310_REVA_GETH3_BASE + SZ_32K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = SPEAR1310_REVA_IRQ_GETH3_SBD,
		.flags = IORESOURCE_IRQ,
		.name = "macirq",
	},
	[2] = {
		.start = SPEAR1310_REVA_IRQ_GETH3_PMT,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 eth3_dma_mask = ~(u32) 0;
struct platform_device spear1310_reva_eth3_device = {
	.name = "stmmaceth",
	.id = 3,
	.num_resources = ARRAY_SIZE(eth3_resources),
	.resource = eth3_resources,
	.dev = {
		.dma_mask = &eth3_dma_mask,
		.coherent_dma_mask = ~0,
	},
};

/* Ethernet GETH-4 device registeration */
static struct resource eth4_resources[] = {
	[0] = {
		.start = SPEAR1310_REVA_GETH4_BASE,
		.end = SPEAR1310_REVA_GETH4_BASE + SZ_32K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = SPEAR1310_REVA_IRQ_GETH4_SBD,
		.flags = IORESOURCE_IRQ,
		.name = "macirq",
	},
	[2] = {
		.start = SPEAR1310_REVA_IRQ_GETH4_PMT,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 eth4_dma_mask = ~(u32) 0;
struct platform_device spear1310_reva_eth4_device = {
	.name = "stmmaceth",
	.id = 4,
	.num_resources = ARRAY_SIZE(eth4_resources),
	.resource = eth4_resources,
	.dev = {
		.dma_mask = &eth4_dma_mask,
		.coherent_dma_mask = ~0,
	},
};

/* i2c1 device registeration */
static struct resource i2c1_resources[] = {
	{
		.start = SPEAR1310_REVA_I2C1_BASE,
		.end = SPEAR1310_REVA_I2C1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1310_REVA_IRQ_I2C_CNTR,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1310_reva_i2c1_device = {
	.name = "i2c_designware",
	.id = 1,
	.dev = {
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(i2c1_resources),
	.resource = i2c1_resources,
};

/* plgpio */
static struct plgpio_platform_data plgpio_plat_data = {
	.gpio_base = 16,
	.irq_base = SPEAR_PLGPIO_INT_BASE,
	.gpio_count = SPEAR_PLGPIO_COUNT,
	.grp_size = PLGPIO_GROUP_SIZE,
	.regs = {
		.enb = SPEAR13XX_PLGPIO_ENB_OFF,
		.wdata = SPEAR13XX_PLGPIO_WDATA_OFF,
		.dir = SPEAR13XX_PLGPIO_DIR_OFF,
		.ie = SPEAR13XX_PLGPIO_IE_OFF,
		.rdata = SPEAR13XX_PLGPIO_RDATA_OFF,
		.mis = SPEAR13XX_PLGPIO_MIS_OFF,
		.eit = -1,
	},
};

static struct resource plgpio_resources[] = {
	{
		.start = SPEAR1310_REVA_RAS_BASE,
		.end = SPEAR1310_REVA_RAS_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1310_REVA_IRQ_PLGPIO,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1310_reva_plgpio_device = {
	.name = "plgpio",
	.id = -1,
	.dev = {
		.platform_data = &plgpio_plat_data,
	},
	.num_resources = ARRAY_SIZE(plgpio_resources),
	.resource = plgpio_resources,
};

/* fsmc nor flash device registeration */
static struct resource ras_fsmc_nor_resources[] = {
	{
		.start	= SPEAR1310_REVA_FSMC1_CS3_BASE,
		.end	= SPEAR1310_REVA_FSMC1_CS3_BASE + SZ_64M - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device spear1310_reva_ras_fsmc_nor_device = {
	.name	= "physmap-flash",
	.id	= -1,
	.resource = ras_fsmc_nor_resources,
	.num_resources = ARRAY_SIZE(ras_fsmc_nor_resources),
};

static struct rs485_hdlc_platform_data rs485_0_plat_data = {
	.tx_falling_edge = 1,
	.rx_rising_edge = 1,
	.cts_enable = 1,
	.cts_delay = 50,
};

static struct rs485_hdlc_platform_data rs485_1_plat_data = {
	.tx_falling_edge = 1,
	.rx_rising_edge = 1,
	.cts_enable = 1,
	.cts_delay = 50,
};

/* hdlc/tdm device registration */
static struct resource rs485_0_resources[] = {
	{
		.start = SPEAR1310_REVA_RS485_0_BASE,
		.end = SPEAR1310_REVA_RS485_0_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1310_REVA_IRQ_RS4850,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource rs485_1_resources[] = {
	{
		.start = SPEAR1310_REVA_RS485_1_BASE,
		.end = SPEAR1310_REVA_RS485_1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1310_REVA_IRQ_RS4851,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1310_reva_rs485_0_device = {
	.name = "rs485_hdlc",
	.id = 0,
	.dev = {
		.platform_data = &rs485_0_plat_data,
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(rs485_0_resources),
	.resource = rs485_0_resources,
};

struct platform_device spear1310_reva_rs485_1_device = {
	.name = "rs485_hdlc",
	.id = 1,
	.dev = {
		.platform_data = &rs485_1_plat_data,
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(rs485_1_resources),
	.resource = rs485_1_resources,
};

static struct tdm_hdlc_platform_data tdm_hdlc_0_plat_data = {
	.ip_type = SPEAR1310_REVA_TDM_HDLC,
	.nr_channel = 2,
	.nr_timeslot = 128,
};

static struct resource tdm_hdlc_0_resources[] = {
	{
		.start = SPEAR1310_REVA_TDM_E1_0_BASE,
		.end = SPEAR1310_REVA_TDM_E1_0_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1310_REVA_IRQ_TDM0,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1310_reva_tdm_hdlc_0_device = {
	.name = "tdm_hdlc",
	.id = 0,
	.dev = {
		.platform_data = &tdm_hdlc_0_plat_data,
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(tdm_hdlc_0_resources),
	.resource = tdm_hdlc_0_resources,
};

static struct tdm_hdlc_platform_data tdm_hdlc_1_plat_data = {
	.ip_type = SPEAR1310_REVA_TDM_HDLC,
	.nr_channel = 2,
	.nr_timeslot = 128,
};

static struct resource tdm_hdlc_1_resources[] = {
	{
		.start = SPEAR1310_REVA_TDM_E1_1_BASE,
		.end = SPEAR1310_REVA_TDM_E1_1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1310_REVA_IRQ_TDM1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1310_reva_tdm_hdlc_1_device = {
	.name = "tdm_hdlc",
	.id = 1,
	.dev = {
		.platform_data = &tdm_hdlc_1_plat_data,
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(tdm_hdlc_1_resources),
	.resource = tdm_hdlc_1_resources,
};

static void tdm_hdlc_setup(void)
{
	struct clk *synth_clk, *vco_clk, *tdm_clk;
	char *synth_clk_name = "ras_synth1_clk";
	char *vco_clk_name = "vco1div4_clk";
	int ret;

	vco_clk = clk_get(NULL, vco_clk_name);
	if (IS_ERR(vco_clk)) {
		pr_err("Failed to get %s\n", vco_clk_name);
		return;
	}

	synth_clk = clk_get(NULL, synth_clk_name);
	if (IS_ERR(synth_clk)) {
		pr_err("Failed to get %s\n", synth_clk_name);
		goto free_vco_clk;
	}

	/* use vco1div4 source for ras_clk_synt1 */
	ret = clk_set_parent(synth_clk, vco_clk);
	if (ret < 0) {
		pr_err("Failed to set parent %s to %s\n", vco_clk_name,
				synth_clk_name);
		goto free_synth_clk;
	}

	/* select ras_clk_synt1 as source for TDM */
	tdm_clk = clk_get_sys(NULL, "tdm_hdlc");
	if (IS_ERR(tdm_clk)) {
		pr_err("Failed to get tdm clock\n");
		goto free_synth_clk;
	}
	ret = clk_set_parent(tdm_clk, synth_clk);
	if (ret < 0) {
		pr_err("Failed to set parent %s to tdm clock\n",
				synth_clk_name);
		goto free_tdm_clk;
	}

free_tdm_clk:
	clk_put(tdm_clk);
free_synth_clk:
	clk_put(synth_clk);
free_vco_clk:
	clk_put(vco_clk);
}

int spear1310_reva_eth_phy_clk_cfg(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *pdata = dev_get_platdata(&pdev->dev);
	void __iomem *addr = IOMEM(IO_ADDRESS(SPEAR1310_REVA_RAS_CTRL_REG1));
	struct clk *clk, *phy_clk = NULL;
	u32 tmp;
	int ret;
	char *pclk_name[] = {
		"ras_pll2_clk",
		"ras_tx125_clk",
		"ras_tx50_clk",
		"ras_synth0_clk",
	};
	const char *phy_clk_name[] = {
		"stmmacphy.0",
		"stmmacphy.1",
		"stmmacphy.2",
		"stmmacphy.3",
		"stmmacphy.4",
	};

	phy_clk = clk_get(NULL, phy_clk_name[pdata->bus_id]);
	if (IS_ERR(phy_clk)) {
		ret = PTR_ERR(phy_clk);
		goto fail_get_phy_clk;
	}

	/*
	 * Select 125 MHz clock for SMII mode, else the clock
	 * for RMII mode is 50 Mhz.
	 * The default clock for the GMAC is driven by pll-2
	 * set to 125Mhz. In case the clock source is required to
	 * be from tx pad, the gmac0 interface should select that
	 * to pad clock.
	 */
	tmp = (pdata->interface == PHY_INTERFACE_MODE_RMII) ? 3 : 0;
	clk = clk_get(NULL, pclk_name[tmp]);
	if (IS_ERR(clk)) {
		pr_err("%s:couldn't get %s as parent for MAC\n",
				__func__, pclk_name[tmp]);
		ret = PTR_ERR(clk);
		goto fail_get_pclk;
	}

	tmp = readl(addr);
	switch (pdata->bus_id) {
	case 1:
		tmp &= (~SPEAR1310_REVA_GETH1_PHY_INTF_MASK);
		tmp |= (pdata->interface == PHY_INTERFACE_MODE_MII) ?
			(SPEAR1310_REVA_PHY_SMII_VAL << 4) :
			(SPEAR1310_REVA_PHY_RMII_VAL << 4);
		break;
	case 2:
		tmp &= (~SPEAR1310_REVA_GETH2_PHY_INTF_MASK);
		tmp |= (pdata->interface == PHY_INTERFACE_MODE_MII) ?
			(SPEAR1310_REVA_PHY_SMII_VAL << 7) :
			(SPEAR1310_REVA_PHY_RMII_VAL << 7);
		break;
	case 3:
		tmp &= (~SPEAR1310_REVA_GETH3_PHY_INTF_MASK);
		tmp |= (pdata->interface == PHY_INTERFACE_MODE_MII) ?
			(SPEAR1310_REVA_PHY_SMII_VAL << 10) :
			(SPEAR1310_REVA_PHY_RMII_VAL << 10);
		break;
	case 4:
		tmp &= (~SPEAR1310_REVA_GETH4_PHY_INTF_MASK);
		tmp |= SPEAR1310_REVA_PHY_RGMII_VAL << 13;
		break;
	default:
		clk_put(clk);
		return -EINVAL;
		break;
	}

	writel(tmp, addr);
	clk_set_parent(phy_clk, clk);
	if (pdata->interface == PHY_INTERFACE_MODE_RMII)
		ret = clk_set_rate(clk, 50000000);

	ret = clk_enable(phy_clk);

	return ret;
fail_get_pclk:
	clk_put(phy_clk);
fail_get_phy_clk:
	return ret;
}

/* Following will create 1310 specific static virtual/physical mappings */
static struct map_desc spear1310_reva_io_desc[] __initdata = {
	{
		.virtual	= IO_ADDRESS(SPEAR1310_REVA_RAS_BASE),
		.pfn		= __phys_to_pfn(SPEAR1310_REVA_RAS_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	},
};

/* This will create static memory mapping for selected devices */
void __init spear1310_reva_map_io(void)
{
	iotable_init(spear1310_reva_io_desc,
			ARRAY_SIZE(spear1310_reva_io_desc));
	spear13xx_map_io();
}

void __init spear1310_reva_init(struct pmx_mode *pmx_mode,
		struct pmx_dev **pmx_devs, u8 pmx_dev_count)
{
	int ret;

	/* call spear13xx family common init function */
	spear13xx_init();

	tdm_hdlc_setup();

	/* pmx initialization */
	pmx_driver.mode = pmx_mode;
	pmx_driver.devs = pmx_devs;
	pmx_driver.devs_count = pmx_dev_count;

	ret = pmx_register(&pmx_driver);
	if (ret)
		pr_err("padmux: registeration failed. err no: %d\n", ret);
}
