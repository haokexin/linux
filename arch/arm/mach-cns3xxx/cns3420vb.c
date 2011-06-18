/*
 * Cavium Networks CNS3420 Validation Board
 *
 * Copyright 2000 Deep Blue Solutions Ltd
 * Copyright 2008 ARM Limited
 * Copyright 2008 Cavium Networks
 *		  Scott Shu
 * Copyright 2010 MontaVista Software, LLC.
 *		  Anton Vorontsov <avorontsov@mvista.com>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, Version 2, as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/compiler.h>
#include <linux/io.h>
#include <linux/serial_core.h>
#include <linux/serial_8250.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/partitions.h>

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>

#include <linux/i2c.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <mach/hardware.h>
#include <mach/cns3xxx.h>
#include <mach/irqs.h>
#ifdef CONFIG_CNS3XXX_DMAC
#include <asm/dma.h>
#include <mach/dmac.h>
#endif
#include "core.h"

/*
 * NOR Flash
 */
static struct mtd_partition cns3420_nor_partitions[] = {
	{
		.name		= "uboot",
		.size		= 0x00040000,
		.offset		= 0,
		.mask_flags	= MTD_WRITEABLE,
	}, {
		.name		= "kernel",
		.size		= 0x004C0000,
		.offset		= MTDPART_OFS_APPEND,
	}, {
		.name		= "filesystem",
		.size		= 0x7000000,
		.offset		= MTDPART_OFS_APPEND,
	}, {
		.name		= "filesystem2",
		.size		= 0x0AE0000,
		.offset		= MTDPART_OFS_APPEND,
	}, {
		.name		= "ubootenv",
		.size		= MTDPART_SIZ_FULL,
		.offset		= MTDPART_OFS_APPEND,
	},
};

static struct physmap_flash_data cns3420_nor_pdata = {
	.width = 2,
	.parts = cns3420_nor_partitions,
	.nr_parts = ARRAY_SIZE(cns3420_nor_partitions),
};

static struct resource cns3420_nor_res = {
	.start = CNS3XXX_FLASH_BASE,
	.end = CNS3XXX_FLASH_BASE + SZ_128M - 1,
	.flags = IORESOURCE_MEM | IORESOURCE_MEM_32BIT,
};

static struct platform_device cns3420_nor_pdev = {
	.name = "physmap-flash",
	.id = 0,
	.resource = &cns3420_nor_res,
	.num_resources = 1,
	.dev = {
		.platform_data = &cns3420_nor_pdata,
	},
};

/*
 * UART
 */
static void __init cns3420_early_serial_setup(void)
{
#ifdef CONFIG_SERIAL_8250_CONSOLE
	static struct uart_port cns3420_serial_port = {
		.membase        = (void __iomem *)CNS3XXX_UART0_BASE_VIRT,
		.mapbase        = CNS3XXX_UART0_BASE,
		.irq            = IRQ_CNS3XXX_UART0,
		.iotype         = UPIO_MEM,
		.flags          = UPF_BOOT_AUTOCONF | UPF_FIXED_TYPE,
		.regshift       = 2,
		.uartclk        = 24000000,
		.line           = 0,
		.type           = PORT_16550A,
		.fifosize       = 16,
	};

	early_serial_setup(&cns3420_serial_port);
#endif
}

/* GPIO */
static struct resource cns3xxx_gpio_resources[] = {
	[0] = {
		.start = CNS3XXX_GPIOA_BASE,
		.end   = CNS3XXX_GPIOA_BASE + 0x44,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = CNS3XXX_GPIOB_BASE,
		.end   = CNS3XXX_GPIOB_BASE + 0x44,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.start = IRQ_CNS3XXX_GPIOA,
		.end   = IRQ_CNS3XXX_GPIOA,
		.flags = IORESOURCE_IRQ,
	},
	[3] = {
		.start = IRQ_CNS3XXX_GPIOB,
		.end   = IRQ_CNS3XXX_GPIOB,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device cns3xxx_gpio_device = {
	.name		= "cns3xxx-gpio",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(cns3xxx_gpio_resources),
	.resource	= cns3xxx_gpio_resources,
};

/* SPI */
static struct mtd_partition cns3xxx_spi_partitions[] = {
	{
		.name		= "SPI-UBoot",
		.offset		= 0,
		.size		= 0x40000,
		.mask_flags	= MTD_WRITEABLE,
	},
	{
		.name		= "SPI-FileSystem",
		.offset		= 0x40000,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0
	},
};

static struct flash_platform_data cns3xxx_spi_flash_data = {
	.parts		= cns3xxx_spi_partitions,
	.nr_parts	= ARRAY_SIZE(cns3xxx_spi_partitions),
};

static struct spi_board_info __initdata cns3xxx_spi_devices[] = {
	[0] = {
		.modalias		= "m25p128",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 25 * 1000 * 1000,
		.platform_data		= &cns3xxx_spi_flash_data,
	}
};

static struct platform_device cns3xxx_spi_controller_device = {
	.name		= "cns3xxx_spi",
};

/* I2C */
static struct i2c_board_info __initdata cns3xxx_i2c_devices[] = {
	{
		I2C_BOARD_INFO("24c16", 0x50),
	},
};

static struct resource cns3xxx_i2c_resource[] = {
	[0] = {
		.start		= CNS3XXX_SSP_BASE + 0x20,
		.end		= 0x7100003f,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_CNS3XXX_I2C,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device cns3xxx_i2c_controller_device = {
	.name = "cns3xxx-i2c",
	.num_resources = 2,
	.resource	= cns3xxx_i2c_resource,
};

/* USB */
static struct resource cns3xxx_usb_ehci_resource[] = {
	[0] = {
		.start = CNS3XXX_USB_BASE,
		.end   = CNS3XXX_USB_BASE + SZ_16M - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_CNS3XXX_USB_EHCI,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 cns3xxx_usb_dma_mask = 0xffffffffULL;

static struct platform_device cns3xxx_usb_ehci_device = {
	.name		= "cns3xxx-ehci",
	.num_resources	= ARRAY_SIZE(cns3xxx_usb_ehci_resource),
	.resource	= cns3xxx_usb_ehci_resource,
	.dev		= {
		.dma_mask		= &cns3xxx_usb_dma_mask,
		.coherent_dma_mask	= 0xffffffffULL,
	},
};

/*
 * Initialization
 */
static struct platform_device *cns3420_pdevs[] __initdata = {
	&cns3420_nor_pdev,
	&cns3xxx_gpio_device,
	&cns3xxx_spi_controller_device,
	&cns3xxx_i2c_controller_device,
	&cns3xxx_usb_ehci_device,
};

static void __init cns3420_init(void)
{
#ifdef CONFIG_CNS3XXX_DMAC
	dmac_init();
#endif

	platform_add_devices(cns3420_pdevs, ARRAY_SIZE(cns3420_pdevs));

	spi_register_board_info(cns3xxx_spi_devices,
		ARRAY_SIZE(cns3xxx_spi_devices));

	i2c_register_board_info(0, cns3xxx_i2c_devices,
		ARRAY_SIZE(cns3xxx_i2c_devices));

	pm_power_off = cns3xxx_power_off;
}

static struct map_desc cns3420_io_desc[] __initdata = {
	{
		.virtual	= CNS3XXX_SWITCH_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_SWITCH_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= CNS3XXX_SSP_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_SSP_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= CNS3XXX_UART0_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_UART0_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= CNS3XXX_DMAC_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_DMAC_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= CNS3XXX_USB_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_USB_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	},
};

static void __init cns3420_map_io(void)
{
	cns3xxx_map_io();
	iotable_init(cns3420_io_desc, ARRAY_SIZE(cns3420_io_desc));

	cns3420_early_serial_setup();
}

MACHINE_START(CNS3420VB, "Cavium Networks CNS3420 Validation Board")
	.phys_io	= CNS3XXX_UART0_BASE,
	.io_pg_offst	= (CNS3XXX_UART0_BASE_VIRT >> 18) & 0xfffc,
	.boot_params	= 0x00000100,
	.map_io		= cns3420_map_io,
	.init_irq	= cns3xxx_init_irq,
	.timer		= &cns3xxx_timer,
	.init_machine	= cns3420_init,
MACHINE_END
