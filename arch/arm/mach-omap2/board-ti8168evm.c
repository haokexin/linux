/*
 * arch/arm/mach-omap2/board-ti8168evm.c
 *
 * Code for TI8168 EVM.
 *
 * Copyright (C) 2010 Texas Instruments, Inc. - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/phy.h>
#include <linux/err.h>
#include <linux/clk.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/cacheflush.h>
#include <asm/fiq.h>

#include <plat/mcspi.h>
#include <plat/irqs.h>
#include <plat/mux.h>
#include <plat/usb.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/timer-gp.h>
#include <plat/asp.h>
#include <plat/mmc.h>
#include <plat/dmtimer.h>
#include <plat/gpmc.h>
#include <plat/nand.h>
#include "clock.h"
#include "mux.h"
#include "hsmmc.h"

/* SPI fLash information */
struct mtd_partition ti816x_spi_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "U-Boot",
		.offset		= 0,	/* Offset = 0x0 */
		.size		= 64 * SZ_4K,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot Env",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x40000 */
		.size		= 2 * SZ_4K,
	},
	{
		.name		= "Kernel",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x42000 */
		.size		= 640 * SZ_4K,
	},
	{
		.name		= "File System",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x2C2000 */
		.size		= MTDPART_SIZ_FULL,	/* size = 1.24 MiB */
	}
};

const struct flash_platform_data ti816x_spi_flash = {
	.type		= "w25x32",
	.name		= "spi_flash",
	.parts		= ti816x_spi_partitions,
	.nr_parts	= ARRAY_SIZE(ti816x_spi_partitions),
};

struct spi_board_info __initdata ti816x_spi_slave_info[] = {
	{
		.modalias	= "m25p80",
		.platform_data	= &ti816x_spi_flash,
		.irq		= -1,
		.max_speed_hz	= 75000000,
		.bus_num	= 1,
		.chip_select	= 0,
	},
};

static void __init ti816x_spi_init(void)
{
	spi_register_board_info(ti816x_spi_slave_info,
				ARRAY_SIZE(ti816x_spi_slave_info));
}

/* NAND flash information */
static struct mtd_partition ti816x_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "U-Boot",
		.offset		= 0,	/* Offset = 0x0 */
		.size		= 19 * SZ_128K,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot Env",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x260000 */
		.size		= 1 * SZ_128K,
	},
	{
		.name		= "Kernel",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
		.size		= 34 * SZ_128K,
	},
	{
		.name		= "File System",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x6C0000 */
		.size		= 1601 * SZ_128K,
	},
	{
		.name		= "Reserved",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0xCEE0000 */
		.size		= MTDPART_SIZ_FULL,
	},

};

static struct omap_nand_platform_data ti816x_nand_data = {
	.options	= NAND_BUSWIDTH_16,
	.parts		= ti816x_nand_partitions,
	.nr_parts	= ARRAY_SIZE(ti816x_nand_partitions),
	.dma_channel	= -1,		/* disable DMA in OMAP NAND driver */
	.nand_setup	= NULL,
	.dev_ready	= NULL,
};

static struct resource ti816x_nand_resource = {
	.flags		= IORESOURCE_MEM,
};

static struct platform_device ti816x_nand_device = {
	.name		= "omap2-nand",
	.id		= -1,
	.dev		= {
		.platform_data	= &ti816x_nand_data,
	},
	.num_resources	= 1,
	.resource	= &ti816x_nand_resource,
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.wires		= 4,
		.gpio_cd	= -EINVAL,/* Dedicated pins for CD and WP */
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_33_34,
	},
	{}	/* Terminator */
};

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
#ifdef CONFIG_USB_MUSB_OTG
	.mode           = MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode           = MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode           = MUSB_PERIPHERAL,
#endif
	.power			= 500,
};

static void __init ti8168_evm_init_irq(void)
{
	omap2_gp_clockevent_set_gptimer(2);
	omap2_init_common_hw(NULL, NULL);
	omap_init_irq();
}

static struct omap_board_config_kernel generic_config[] = {
};

int __init ti_ahci_register(u8 num_inst);
void __init ti816x_init_pcie(void);
int __init ti816x_register_edma(void);
void ti816x_ethernet_init(void);

static struct i2c_board_info __initdata ti816x_i2c_boardinfo0[] = {
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x18),
	},

};

#define TI816X_EVM_CIR_UART BIT(5)

static int __init ti816x_evm_i2c_init(void)
{
	omap_register_i2c_bus(1, 100, ti816x_i2c_boardinfo0,
		ARRAY_SIZE(ti816x_i2c_boardinfo0));
	return 0;
}

static u8 ti8168_iis_serializer_direction[] = {
	TX_MODE,	RX_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
};

static struct snd_platform_data ti8168_evm_snd_data = {
	.tx_dma_offset = 0x46800000,
	.rx_dma_offset = 0x46800000,
	.op_mode = DAVINCI_MCASP_IIS_MODE,
	.num_serializer = ARRAY_SIZE(ti8168_iis_serializer_direction),
	.tdm_slots = 2,
	.serial_dir = ti8168_iis_serializer_direction,
	.eventq_no = EVENTQ_3,
	.version = MCASP_VERSION_2,
	.txnumevt = 1,
	.rxnumevt = 1,
};

static void __init ti816x_nand_init(void)
{
	u8 cs = 0;
	u8 nandcs = GPMC_CS_NUM + 1;
	u32 gpmc_base_add = TI816X_GPMC_VIRT;

	/* find out the chip-select on which NAND exists */
	while (cs < GPMC_CS_NUM) {
		u32 ret = 0;
		ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

		if ((ret & 0xC00) == 0x800) {
			printk(KERN_INFO "Found NAND on CS%d\n", cs);
			if (nandcs > GPMC_CS_NUM)
				nandcs = cs;
		}
		cs++;
	}

	if (nandcs > GPMC_CS_NUM) {
		printk(KERN_INFO "NAND: Unable to find configuration "
				 "in GPMC\n ");
		return;
	}

	if (nandcs < GPMC_CS_NUM) {
		ti816x_nand_data.cs = nandcs;
		ti816x_nand_data.phys_base = TI816X_GPMC_BASE;
		ti816x_nand_data.devsize = NAND_BUSWIDTH_16;
		ti816x_nand_data.gpmc_cs_baseaddr = (void *)
			(gpmc_base_add + GPMC_CS0_BASE + nandcs * GPMC_CS_SIZE);
		ti816x_nand_data.gpmc_baseaddr = (void *) (gpmc_base_add);

		printk(KERN_INFO "Registering NAND on CS%d\n", nandcs);
		if (platform_device_register(&ti816x_nand_device) < 0)
			printk(KERN_ERR "Unable to register NAND device\n");
	}
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

static void __init ti8168_evm_init(void)
{
	ti816x_mux_init(board_mux);
	omap_board_config = generic_config;
	omap_board_config_size = ARRAY_SIZE(generic_config);
	omap_serial_init();

	omap2_hsmmc_init(mmc);
	/* initialize usb */
	usb_musb_init(&musb_board_data);
	/* register ahci interface for 2 SATA ports */
	ti_ahci_register(2);
	ti816x_init_pcie();
	ti816x_register_edma();
	ti816x_ethernet_init();

	ti816x_evm_i2c_init();
	ti816x_nand_init();
	ti816x_spi_init();
	ti816x_register_mcasp(0, &ti8168_evm_snd_data);
}

static void __init ti8168_evm_map_io(void)
{
	omap2_set_globals_ti816x();
	ti816x_map_common_io();
}

MACHINE_START(TI8168EVM, "ti8168evm")
	/* Maintainer: Texas Instruments */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= ti8168_evm_map_io,
	.init_irq	 = ti8168_evm_init_irq,
	.init_machine	= ti8168_evm_init,
	.timer		= &omap_timer,
MACHINE_END
