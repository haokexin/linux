/*
 * arch/arm/mach-spear13xx/spear1300_evb.c
 *
 * SPEAr1300 evaluation board source file
 *
 * Copyright (C) 2010 ST Microelectronics
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/mfd/stmpe.h>
#include <linux/mtd/fsmc.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/spear_smi.h>
#include <linux/pata_arasan_cf_data.h>
#include <linux/phy.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
#include <linux/stmmac.h>
#include <video/db9000fb.h>
#include <asm/hardware/gic.h>
#include <asm/mach-types.h>
#include <plat/keyboard.h>
#include <plat/spi.h>
#include <mach/generic.h>
#include <mach/hardware.h>
#include <mach/spear_pcie.h>

/* padmux devices to enable */
static struct pmx_dev *pmx_devs[] = {
	/* spear13xx specific devices */
	&spear13xx_pmx_i2c,
	&spear13xx_pmx_i2s1,
	&spear13xx_pmx_i2s2,
	&spear13xx_pmx_clcd,
	&spear13xx_pmx_egpio_grp,
	&spear13xx_pmx_gmii,
	&spear13xx_pmx_keyboard_6x6,
	&spear13xx_pmx_mcif,
	&spear13xx_pmx_nand_8bit,
	&spear13xx_pmx_smi_4_chips,
	&spear13xx_pmx_ssp,
	&spear13xx_pmx_uart0,
	&spear13xx_pmx_sdhci,

	/* spear1300 specific devices */
};

static struct amba_device *amba_devs[] __initdata = {
	&spear13xx_gpio_device[0],
	&spear13xx_gpio_device[1],
	&spear13xx_ssp_device,
	&spear13xx_uart_device,
};

static struct platform_device *plat_devs[] __initdata = {
	/* spear13xx specific devices */
	&spear13xx_adc_device,
	&spear13xx_cpufreq_device,
	&spear13xx_db9000_clcd_device,
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
	&spear13xx_device_gpiokeys,
#endif
	&spear13xx_dmac_device[0],
	&spear13xx_dmac_device[1],
	&spear13xx_ehci0_device,
	&spear13xx_ehci1_device,
	&spear13xx_eth_device,
	&spear13xx_i2c_device,
	&spear13xx_i2s0_device,
	&spear13xx_jpeg_device,
	&spear13xx_kbd_device,
	&spear13xx_nand_device,
	&spear13xx_ohci0_device,
	&spear13xx_ohci1_device,
	&spear13xx_pcie_gadget0_device,
	&spear13xx_pcie_host1_device,
	&spear13xx_pcie_host2_device,
	&spear13xx_pcm_device,
	&spear13xx_rtc_device,
	&spear13xx_sdhci_device,
	&spear13xx_smi_device,
	&spear13xx_thermal_device,
	&spear13xx_udc_device,
	&spear13xx_wdt_device,
	/* spear1300 specific devices */

};

/* Ethernet PLatform data */
/* MDIO Bus Data */
static struct stmmac_mdio_bus_data mdio0_private_data = {
	.bus_id = 0,
	.phy_mask = 0,
};

static struct stmmac_dma_cfg dma0_private_data = {
	.pbl = 8,
	.fixed_burst = 1,
	.burst_len = DMA_AXI_BLEN_ALL,
};

static struct plat_stmmacenet_data eth_data = {
	.bus_id = 0,
	.phy_addr = 5,
	.interface = PHY_INTERFACE_MODE_GMII,
	.has_gmac = 1,
	.enh_desc = 1,
	.tx_coe = 1,
	.dma_cfg = &dma0_private_data,
	.rx_coe = STMMAC_RX_COE_TYPE2,
	.bugged_jumbo = 1,
	.pmt = 1,
	.mdio_bus_data = &mdio0_private_data,
	.init = spear13xx_eth_phy_clk_cfg,
	.clk_csr = STMMAC_CSR_150_250M,
};

static struct mtd_partition nand_partition_info[] __initdata = {
	{
		.name = "X-loader",
		.offset = 0,
		.size = 4 * 0x20000,
	}, {
		.name = "U-Boot",
		.offset = 4 * 0x20000,
		.size = 12 * 0x20000,
	}, {
		.name = "Kernel",
		.offset = (4 + 12) * 0x20000,
		.size = 48 * 0x20000,
	}, {
		.name = "Root File System",
		.offset = (4 + 12 + 48) * 0x20000,
		.size = MTDPART_SIZ_FULL,
	}
};

/* fsmc platform data */
static const struct fsmc_nand_platform_data nand_plat_data __initconst = {
	.select_bank = nand_select_bank,
	.partitions = nand_partition_info,
	.nr_partitions = ARRAY_SIZE(nand_partition_info),
	.options = NAND_SKIP_BBTSCAN,
	.width = FSMC_NAND_BW8,
	.ale_off = PLAT_NAND_ALE,
	.cle_off = PLAT_NAND_CLE,
	.mode = USE_DMA_ACCESS,
	.read_dma_priv = &nand_read_dma_priv,
	.write_dma_priv = &nand_write_dma_priv,
	.max_banks = 1,
};

#if 0
/* fsmc nor partition info */
#define PARTITION(n, off, sz)	{.name = n, .offset = off, .size = sz}
static struct mtd_partition partition_info[] = {
	PARTITION("X-loader", 0, 1 * 0x20000),
	PARTITION("U-Boot", 0x20000, 3 * 0x20000),
	PARTITION("Kernel", 0x80000, 24 * 0x20000),
	PARTITION("Root File System", 0x380000, 84 * 0x20000),
};

/* fsmc nor platform data */
static const struct physmap_flash_data nor_plat_data __initconst = {
	.parts = partition_info,
	.nr_parts = ARRAY_SIZE(partition_info),
	.width = FSMC_FLASH_WIDTH8,
};
#endif

/* arasan compact flash controller's platform data */
static struct arasan_cf_pdata cf_pdata = {
	.cf_if_clk = CF_IF_CLK_166M,
	.quirk = CF_BROKEN_UDMA,
	.dma_priv = &cf_dma_priv,
};

/* keyboard specific platform data */
static const __initconst DECLARE_9x9_KEYMAP(keymap);
static const struct matrix_keymap_data keymap_data __initconst = {
	.keymap = keymap,
	.keymap_size = ARRAY_SIZE(keymap),
};

static const struct kbd_platform_data kbd_data __initconst = {
	.keymap = &keymap_data,
	.rep = 1,
	.mode = KEYPAD_9x9,
};

/* Currently no gpios are free on eval board so it is kept commented */
#if 0
/* spi0 flash Chip Select Control function, controlled by gpio pin mentioned */
DECLARE_SPI_CS_GPIO_CONTROL(0, flash, /* mention gpio number here */);
/* spi0 flash Chip Info structure */
DECLARE_SPI_CHIP_INFO(0, flash, spi0_flash_cs_gpio_control);

/* spi0 spidev Chip Select Control function, controlled by gpio pin mentioned */
DECLARE_SPI_CS_GPIO_CONTROL(0, dev, /* mention gpio number here */);
/* spi0 spidev Chip Info structure */
DECLARE_SPI_CHIP_INFO(0, dev, spi0_dev_cs_gpio_control);
#endif

/* spi0 touch screen Chip Select Control function, controlled by gpio pin */
DECLARE_SPI_CS_GPIO_CONTROL(0, ts, GPIO1_7);
/* spi0 touch screen Info structure */
static struct pl022_config_chip spi0_ts_chip_info = {
	.iface = SSP_INTERFACE_MOTOROLA_SPI,
	.hierarchy = SSP_MASTER,
	.slave_tx_disable = 0,
	.com_mode = INTERRUPT_TRANSFER,
	.rx_lev_trig = SSP_RX_1_OR_MORE_ELEM,
	.tx_lev_trig = SSP_TX_1_OR_MORE_EMPTY_LOC,
	.ctrl_len = SSP_BITS_8,
	.wait_state = SSP_MWIRE_WAIT_ZERO,
	.duplex = SSP_MICROWIRE_CHANNEL_FULL_DUPLEX,
	.cs_control = spi0_ts_cs_gpio_control,
};

static struct stmpe_ts_platform_data stmpe610_ts_pdata = {
	.sample_time = 4, /* 80 clocks */
	.mod_12b = 1, /* 12 bit */
	.ref_sel = 0, /* Internal */
	.adc_freq = 1, /* 3.25 MHz */
	.ave_ctrl = 1, /* 2 samples */
	.touch_det_delay = 2, /* 100 us */
	.settling = 2, /* 500 us */
	.fraction_z = 7,
	.i_drive = 1, /* 50 to 80 mA */
};

static struct stmpe_platform_data stmpe610_pdata = {
	.id = 0,
	.blocks = STMPE_BLOCK_TOUCHSCREEN,
	.irq_base = SPEAR_STMPE610_INT_BASE,
	.irq_trigger = IRQ_TYPE_EDGE_FALLING,
	.irq_invert_polarity = false,
	.autosleep = false,
	.irq_over_gpio = true,
	.irq_gpio = GPIO1_6,
	.ts = &stmpe610_ts_pdata,
};

static struct spi_board_info __initdata spi_board_info[] = {
	/* spi0 board info */
	{
		.modalias = "stmpe610",
		.platform_data = &stmpe610_pdata,
		.controller_data = &spi0_ts_chip_info,
		.max_speed_hz = 1000000,
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_1,
	},
#if 0
	/* spi0 board info */
	{
		.modalias = "spidev",
		.controller_data = &spi0_dev_chip_info,
		.max_speed_hz = 25000000,
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_1,
	}, {
		.modalias = "m25p80",
		.controller_data = &spi0_flash_chip_info,
		.max_speed_hz = 12000000,
		.bus_num = 0,
		.chip_select = 1,
		.mode = SPI_MODE_3,
	}
#endif
};

#ifdef CONFIG_SPEAR_PCIE_REV341
/* This function is needed for board specific PCIe initilization */
static void __init spear1300_pcie_board_init(void)
{
	void *plat_data;

	plat_data = dev_get_platdata(&spear13xx_pcie_host1_device.dev);
	PCIE_PORT_INIT((struct pcie_port_info *)plat_data, SPEAR_PCIE_REV_3_41);

	plat_data = dev_get_platdata(&spear13xx_pcie_host2_device.dev);
	PCIE_PORT_INIT((struct pcie_port_info *)plat_data, SPEAR_PCIE_REV_3_41);
}
#endif

static void
spear1300_evb_fixup(struct tag *tags, char **cmdline, struct meminfo *mi)
{
#if defined(CONFIG_FB_DB9000) || defined(CONFIG_FB_DB9000_MODULE)
	spear13xx_panel_fixup(mi);
#endif
}

static void __init spear1300_evb_init(void)
{
	unsigned int i;

	/* set compact flash plat data */
	set_arasan_cf_pdata(&spear13xx_cf_device, &cf_pdata);

#if defined(CONFIG_FB_DB9000) || defined(CONFIG_FB_DB9000_MODULE)
	/* db9000_clcd plat data */
	spear13xx_panel_init(&spear13xx_db9000_clcd_device);
#endif

	/* call spear1300 machine init function */
	spear1300_init(NULL, pmx_devs, ARRAY_SIZE(pmx_devs));

	/* Set stmmac plat data */
	if (platform_device_add_data(&spear13xx_eth_device, &eth_data,
			sizeof(eth_data)))
		printk(KERN_WARNING "%s: couldn't add plat_data",
				spear13xx_eth_device.name);


	/* set nand device's plat data */
	nand_mach_init(FSMC_NAND_BW8);
	if (platform_device_add_data(&spear13xx_nand_device, &nand_plat_data,
				sizeof(nand_plat_data)))
		printk(KERN_WARNING "%s: couldn't add plat_data",
				spear13xx_nand_device.name);

	/*
	 * FSMC cannot used as NOR and NAND at the same time For the moment,
	 * disable NOR and use NAND only. If NOR is needed, enable the following
	 * code and disable all code for NAND. Also enable nand in padmux
	 * configuration to use it
	 */
#if 0
	/* Initialize fsmc regiters */
	fsmc_nor_init(&spear13xx_fsmc_nor_device, SPEAR13XX_FSMC_BASE, 0,
			FSMC_FLASH_WIDTH8);
	/* initialize fsmc related data in fsmc plat data */
	if (platform_device_add_data(&spear13xx_fsmc_nor_device, &nor_plat_data,
				sizeof(nor_plat_data)))
		printk(KERN_WARNING "%s: couldn't add plat_data",
				spear13xx_fsmc_nor_device.name);
#endif

	/* set keyboard plat data */
	if (platform_device_add_data(&spear13xx_kbd_device, &kbd_data,
				sizeof(kbd_data)))
		printk(KERN_WARNING "%s: couldn't add plat_data",
				spear13xx_kbd_device.name);

#ifdef CONFIG_SPEAR_PCIE_REV341
	spear1300_pcie_board_init();
#endif

	/* initialize serial nor related data in smi plat data */
	smi_init_board_info(&spear13xx_smi_device);

	/* Register slave devices on the I2C buses */
	i2c_register_default_devices();

	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));

	/* Add Platform Devices */
	platform_add_devices(plat_devs, ARRAY_SIZE(plat_devs));

	/* Add Amba Devices */
	for (i = 0; i < ARRAY_SIZE(amba_devs); i++)
		amba_device_register(amba_devs[i], &iomem_resource);
}

MACHINE_START(SPEAR1300_EVB, "ST-SPEAR1300-EVB")
	.atag_offset	=	0x100,
	.fixup		=	spear1300_evb_fixup,
	.map_io		=	spear13xx_map_io,
	.init_irq	=	spear13xx_init_irq,
	.handle_irq	=	gic_handle_irq,
	.timer		=	&spear13xx_timer,
	.init_machine	=	spear1300_evb_init,
	.restart	=	spear_restart,
MACHINE_END
