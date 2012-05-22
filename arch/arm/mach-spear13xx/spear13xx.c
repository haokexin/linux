/*
 * arch/arm/mach-spear13xx/spear13xx.c
 *
 * SPEAr13XX machines common source file
 *
 * Copyright (C) 2010 ST Microelectronics
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#define pr_fmt(fmt) "spear13xx: " fmt

#include <linux/types.h>
#include <linux/amba/pl022.h>
#include <linux/amba/pl061.h>
#include <linux/amba/serial.h>
#include <linux/dw_dmac.h>
#include <linux/designware_i2s.h>
#include <linux/mtd/fsmc.h>
#include <linux/ptrace.h>
#include <linux/phy.h>
#include <linux/io.h>
#include <linux/netdevice.h>
#include <linux/platform_data/spear_thermal.h>
#include <linux/stmmac.h>
#include <asm/hardware/gic.h>
#include <asm/irq.h>
#include <asm/pmu.h>
#include <asm/setup.h>
#include <asm/localtimer.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/smp_twd.h>
#include <plat/adc.h>
#include <plat/clock.h>
#include <plat/cpufreq.h>
#include <plat/jpeg.h>
#include <plat/udc.h>
#include <mach/dma.h>
#include <mach/generic.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/misc_regs.h>
#include <mach/spear_pcie.h>
#include <sound/pcm.h>

/* SPEAr GPIO Buttons Info */
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#include <linux/gpio_keys.h>
#include <linux/input.h>

/* SPEAr GPIO Buttons definition */
#define SPEAR_GPIO_BTN7	7

static struct gpio_keys_button spear_gpio_keys_table[] = {
	{
		.code = BTN_0,
		.gpio = SPEAR_GPIO_BTN7,
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

struct platform_device spear13xx_device_gpiokeys = {
	.name = "gpio-keys",
	.dev = {
		.platform_data = &spear_gpio_keys_data,
	},
};
#endif

/* Add spear13xx machines common devices here */
/* common dw_dma filter routine to be used by peripherals */
bool dw_dma_filter(struct dma_chan *chan, void *slave)
{
	struct dw_dma_slave *dws = (struct dw_dma_slave *)slave;

	if (chan->device->dev == dws->dma_dev) {
		chan->private = slave;
		return true;
	} else {
		return false;
	}
}

/* SPEAr Thermal Sensor Platform Data */
static struct resource spear13xx_thermal_resources[] = {
	{
		.start = THSENS_CFG,
		.end = THSENS_CFG + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct spear_thermal_pdata spear13xx_thermal_pdata = {
	.thermal_flags = THERMAL_CONFIG_FLAGS,
};

struct platform_device spear13xx_thermal_device = {
	.name = "spear_thermal",
	.id = -1,
	.dev = {
		.platform_data = &spear13xx_thermal_pdata,
	},
	.num_resources = ARRAY_SIZE(spear13xx_thermal_resources),
	.resource = spear13xx_thermal_resources,
};

/* gpio device registeration */
static struct pl061_platform_data gpio_plat_data[] = {
	{
		.gpio_base	= 0,
		.irq_base	= SPEAR_GPIO0_INT_BASE,
	}, {
		.gpio_base	= 8,
		.irq_base	= SPEAR_GPIO1_INT_BASE,
	},
};

struct amba_device spear13xx_gpio_device[] = {
	{
		.dev = {
			.init_name = "gpio0",
			.platform_data = &gpio_plat_data[0],
		},
		.res = {
			.start = SPEAR13XX_GPIO0_BASE,
			.end = SPEAR13XX_GPIO0_BASE + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		},
		.irq = {SPEAR13XX_IRQ_GPIO0, NO_IRQ},
	}, {
		.dev = {
			.init_name = "gpio1",
			.platform_data = &gpio_plat_data[1],
		},
		.res = {
			.start = SPEAR13XX_GPIO1_BASE,
			.end = SPEAR13XX_GPIO1_BASE + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		},
		.irq = {SPEAR13XX_IRQ_GPIO1, NO_IRQ},
	}
};

/* ssp device registeration */
#if 0
static struct dw_dma_slave ssp_dma_param[] = {
	{
		/* Tx */
		.dma_dev = &spear13xx_dmac_device[0].dev,
		.cfg_hi = DWC_CFGH_DST_PER(SPEAR13XX_DMA_REQ_SSP0_TX),
		.cfg_lo = 0,
		.src_master = SPEAR13XX_DMA_MASTER_MEMORY,
		.dst_master = SPEAR13XX_DMA_MASTER_SSP0,
	}, {
		/* Rx */
		.dma_dev = &spear13xx_dmac_device[0].dev,
		.cfg_hi = DWC_CFGH_SRC_PER(SPEAR13XX_DMA_REQ_SSP0_RX),
		.cfg_lo = 0,
		.src_master = SPEAR13XX_DMA_MASTER_SSP0,
		.dst_master = SPEAR13XX_DMA_MASTER_MEMORY,
	}
};
#endif

static struct pl022_ssp_controller ssp_platform_data = {
	.bus_id = 0,
	.enable_dma = 0,
#if 0
	.dma_filter = dw_dma_filter,
	.dma_rx_param = &ssp_dma_param[1],
	.dma_tx_param = &ssp_dma_param[0],
#endif
	/*
	 * Following is the number of chip selects from spi controller
	 * to which spi devices can be connected.
	 * There are two type of chipselects on which slave devices can
	 * work. One is chip select provided by spi masters other is
	 * controlled through external gpio's.
	 */
	.num_chipselect = 3,
};

struct amba_device spear13xx_ssp_device = {
	.dev = {
		.coherent_dma_mask = ~0,
		.init_name = "ssp-pl022",
		.platform_data = &ssp_platform_data,
	},
	.res = {
		.start = SPEAR13XX_SSP_BASE,
		.end = SPEAR13XX_SSP_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {SPEAR13XX_IRQ_SSP, NO_IRQ},
};

/* uart device registeration */
/* As uart0 is used for console, so disable DMA here */
#if 0
static struct dw_dma_slave uart_dma_param[] = {
	{
		/* Tx */
		.dma_dev = &spear13xx_dmac_device[0].dev,
		.cfg_hi = DWC_CFGH_DST_PER(SPEAR13XX_DMA_REQ_UART0_TX),
		.cfg_lo = 0,
		.src_master = SPEAR13XX_DMA_MASTER_MEMORY,
		.dst_master = SPEAR13XX_DMA_MASTER_UART0,
	}, {
		/* Rx */
		.dma_dev = &spear13xx_dmac_device[0].dev,
		.cfg_hi = DWC_CFGH_SRC_PER(SPEAR13XX_DMA_REQ_UART0_RX),
		.cfg_lo = 0,
		.src_master = SPEAR13XX_DMA_MASTER_UART0,
		.dst_master = SPEAR13XX_DMA_MASTER_MEMORY,
	}
};

static struct amba_pl011_data uart_data = {
	.dma_filter = dw_dma_filter,
	.dma_tx_param = &uart_dma_param[0],
	.dma_rx_param = &uart_dma_param[1],
};
#endif

struct amba_device spear13xx_uart_device = {
	.dev = {
		.init_name = "uart",
/*		.platform_data = &uart_data, */
	},
	.res = {
		.start = SPEAR13XX_UART_BASE,
		.end = SPEAR13XX_UART_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {SPEAR13XX_IRQ_UART, NO_IRQ},
};

/* adc device registeration */
static struct dw_dma_slave adc_dma_param[] = {
	{
		.dma_dev = &spear13xx_dmac_device[0].dev,
		.cfg_hi = DWC_CFGH_SRC_PER(SPEAR13XX_DMA_REQ_ADC),
		.cfg_lo = 0,
		.src_master = SPEAR13XX_DMA_MASTER_ADC,
		.dst_master = SPEAR13XX_DMA_MASTER_MEMORY,
	}
};

static struct adc_plat_data adc_pdata = {
	.dma_filter = dw_dma_filter,
	.dma_data = &adc_dma_param,
	.config = {CONTINUOUS_CONVERSION, EXTERNAL_VOLT, 2500, INTERNAL_SCAN,
		NORMAL_RESOLUTION, 14000000, 0},
};

static struct resource adc_resources[] = {
	{
		.start = SPEAR13XX_ADC_BASE,
		.end = SPEAR13XX_ADC_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR13XX_IRQ_ADC,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear13xx_adc_device = {
	.name = "adc",
	.id = -1,
	.dev = {
		.coherent_dma_mask = ~0,
		.platform_data = &adc_pdata,
	},
	.num_resources = ARRAY_SIZE(adc_resources),
	.resource = adc_resources,
};

/* cf device registeration */
struct dw_dma_slave cf_dma_priv = {
	.dma_dev = &spear13xx_dmac_device[0].dev,
	.cfg_hi = 0,
	.cfg_lo = 0,
	.src_master = 0,
	.dst_master = 0,
};

static struct resource cf_resources[] = {
	{
		.start = SPEAR13XX_MCIF_CF_BASE,
		.end = SPEAR13XX_MCIF_CF_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR13XX_IRQ_CF,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear13xx_cf_device = {
	.name = "arasan_cf",
	.id = -1,
	.dev = {
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(cf_resources),
	.resource = cf_resources,
};

/* clcd db9000 devide registration */
static struct resource db9000fb_resources[] = {
	[0] = {
		.start = SPEAR13XX_CLCD_BASE,
		.end = SPEAR13XX_CLCD_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = SPEAR13XX_IRQ_CLCD,
		.end = SPEAR13XX_IRQ_CLCD,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 fb_dma_mask = ~(u64)0;

struct platform_device spear13xx_db9000_clcd_device = {
	.name = "clcd-db9000",
	.id = -1,
	.dev = {
		.parent = NULL,
		.dma_mask = &fb_dma_mask,
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(db9000fb_resources),
	.resource = db9000fb_resources,
};

/* dmac device registeration */
static struct dw_dma_platform_data dmac_plat_data = {
	.nr_channels = 8,
	.chan_allocation_order = CHAN_ALLOCATION_DESCENDING,
	.chan_priority = CHAN_PRIORITY_DESCENDING,
};

static struct resource dmac_resources[][2] = {
	[0] = {
		{
			.start = SPEAR13XX_DMAC0_BASE,
			.end = SPEAR13XX_DMAC0_BASE + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.start = SPEAR13XX_IRQ_DMAC0_COMBINED,
			.flags = IORESOURCE_IRQ,
		},
	},
	[1] = {
		{
			.start = SPEAR13XX_DMAC1_BASE,
			.end = SPEAR13XX_DMAC1_BASE + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.start = SPEAR13XX_IRQ_DMAC1_COMBINED,
			.flags = IORESOURCE_IRQ,
		},
	},
};

struct platform_device spear13xx_dmac_device[] = {
	[0] = {
		.name = "dw_dmac",
		.id = 0,
		.dev = {
			.coherent_dma_mask = ~0,
			.platform_data = &dmac_plat_data,
		},
		.num_resources = ARRAY_SIZE(dmac_resources[0]),
		.resource = dmac_resources[0],
	},
	[1] = {
		.name = "dw_dmac",
		.id = 1,
		.dev = {
			.coherent_dma_mask = ~0,
			.platform_data = &dmac_plat_data,
		},
		.num_resources = ARRAY_SIZE(dmac_resources[1]),
		.resource = dmac_resources[1],
	},
};

/* Ethernet device registeration */
static struct resource eth_resources[] = {
	[0] = {
		.start = SPEAR13XX_GETH_BASE,
		.end = SPEAR13XX_GETH_BASE + SZ_32K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = SPEAR13XX_IRQ_GETH_SBD,
		.flags = IORESOURCE_IRQ,
		.name = "macirq",
	},
	[2] = {
		.start = SPEAR13XX_IRQ_GETH_PMT,
		.flags = IORESOURCE_IRQ,
		.name = "eth_wake_irq",
	},
};

static u64 eth_dma_mask = ~(u32) 0;
struct platform_device spear13xx_eth_device = {
	.name = "stmmaceth",
	.id = 0,
	.num_resources = ARRAY_SIZE(eth_resources),
	.resource = eth_resources,
	.dev = {
		.dma_mask = &eth_dma_mask,
		.coherent_dma_mask = ~0,
	},
};

/* i2c device registeration */
static struct resource i2c_resources[] = {
	{
		.start = SPEAR13XX_I2C_BASE,
		.end = SPEAR13XX_I2C_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR13XX_IRQ_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear13xx_i2c_device = {
	.name = "i2c_designware",
	.id = 0,
	.dev = {
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(i2c_resources),
	.resource = i2c_resources,
};

/* fsmc nor flash device registeration */
static struct resource fsmc_nor_resources[] = {
	{
		.start	= SPEAR13XX_FSMC_MEM_BASE,
		.end	= SPEAR13XX_FSMC_MEM_BASE + SZ_16M - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device spear13xx_fsmc_nor_device = {
	.name	= "physmap-flash",
	.id	= -1,
	.resource = fsmc_nor_resources,
	.num_resources = ARRAY_SIZE(fsmc_nor_resources),
};

/* nand device registeration */
void __init nand_mach_init(u32 busw)
{
	void __iomem *reg;
	u32 fsmc_cfg;

	if (cpu_is_spear1340()) {
#ifdef CONFIG_CPU_SPEAR1340
		reg = VA_SPEAR1340_FSMC_CFG;
#endif
	} else
		reg = VA_FSMC_CFG;

	fsmc_cfg = readl(reg);
	fsmc_cfg &= ~(FSMC_MEMSEL_MASK << FSMC_MEMSEL_SHIFT);
	fsmc_cfg |= (FSMC_MEM_NAND << FSMC_MEMSEL_SHIFT);

	if (busw == FSMC_NAND_BW16)
		fsmc_cfg |= 1 << NAND_DEV_WIDTH16;
	else
		fsmc_cfg &= ~(1 << NAND_DEV_WIDTH16);

	writel(fsmc_cfg, reg);
}

void nand_select_bank(u32 bank, u32 busw)
{
	u32 fsmc_cfg;

	if (cpu_is_spear1340()) {
#ifdef CONFIG_CPU_SPEAR1340
		fsmc_cfg = readl(VA_SPEAR1340_FSMC_CFG);
#endif
	} else
		fsmc_cfg = readl(VA_FSMC_CFG);

	fsmc_cfg &= ~(NAND_BANK_MASK << NAND_BANK_SHIFT);
	fsmc_cfg |= (bank << NAND_BANK_SHIFT);

	if (busw)
		fsmc_cfg |= 1 << NAND_DEV_WIDTH16;
	else
		fsmc_cfg &= ~(1 << NAND_DEV_WIDTH16);

	if (cpu_is_spear1340()) {
#ifdef CONFIG_CPU_SPEAR1340
		writel(fsmc_cfg, VA_SPEAR1340_FSMC_CFG);
#endif
	} else
		writel(fsmc_cfg, VA_FSMC_CFG);
}

struct dw_dma_slave nand_read_dma_priv = {
	.dma_dev = &spear13xx_dmac_device[0].dev,
	.src_master = SPEAR13XX_DMA_MASTER_FSMC,
	.dst_master = SPEAR13XX_DMA_MASTER_MEMORY,
};

struct dw_dma_slave nand_write_dma_priv = {
	.dma_dev = &spear13xx_dmac_device[0].dev,
	.src_master = SPEAR13XX_DMA_MASTER_MEMORY,
	.dst_master = SPEAR13XX_DMA_MASTER_FSMC,
};

static struct resource nand_resources[] = {
	{
		.name = "nand_data",
		.start = SPEAR13XX_FSMC_MEM_BASE,
		.end = SPEAR13XX_FSMC_MEM_BASE + SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.name = "fsmc_regs",
		.start = SPEAR13XX_FSMC_BASE,
		.end = SPEAR13XX_FSMC_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device spear13xx_nand_device = {
	.name = "fsmc-nand",
	.id = -1,
	.resource = nand_resources,
	.num_resources = ARRAY_SIZE(nand_resources),
};

/* pmu device */
static struct resource spear13xx_pmu_resources[] = {
	{
		.start	= SPEAR13XX_IRQ_PMU0,
		.end	= SPEAR13XX_IRQ_PMU0,
		.flags	= IORESOURCE_IRQ,
	}, {
		.start	= SPEAR13XX_IRQ_PMU1,
		.end	= SPEAR13XX_IRQ_PMU1,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device spear13xx_pmu_device = {
	.name		= "arm-pmu",
	.id		= ARM_PMU_DEVICE_CPU,
	.num_resources	= ARRAY_SIZE(spear13xx_pmu_resources),
	.resource	= spear13xx_pmu_resources,
};

/* usb host device registeration */
static struct resource ehci0_resources[] = {
	[0] = {
		.start = SPEAR13XX_UHC0_EHCI_BASE,
		.end = SPEAR13XX_UHC0_EHCI_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = SPEAR13XX_IRQ_USBH_EHCI0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource ehci1_resources[] = {
	[0] = {
		.start = SPEAR13XX_UHC1_EHCI_BASE,
		.end = SPEAR13XX_UHC1_EHCI_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = SPEAR13XX_IRQ_USBH_EHCI1,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource ohci0_resources[] = {
	[0] = {
		.start = SPEAR13XX_UHC0_OHCI_BASE,
		.end = SPEAR13XX_UHC0_OHCI_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = SPEAR13XX_IRQ_USBH_OHCI0,
		.flags = IORESOURCE_IRQ,
	},
};
static struct resource ohci1_resources[] = {
	[0] = {
		.start = SPEAR13XX_UHC1_OHCI_BASE,
		.end = SPEAR13XX_UHC1_OHCI_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = SPEAR13XX_IRQ_USBH_OHCI1,
		.flags = IORESOURCE_IRQ,
	},
};

/* usbh0_id defaults to 0, being static variable */
static int usbh0_id;
static int usbh1_id = 1;
static u64 ehci0_dmamask = ~0;

struct platform_device spear13xx_ehci0_device = {
	.name = "spear-ehci",
	.id = 0,
	.dev = {
		.coherent_dma_mask = ~0,
		.dma_mask = &ehci0_dmamask,
		.platform_data = &usbh0_id,
	},
	.num_resources = ARRAY_SIZE(ehci0_resources),
	.resource = ehci0_resources,
};

static u64 ehci1_dmamask = ~0;

struct platform_device spear13xx_ehci1_device = {
	.name = "spear-ehci",
	.id = 1,
	.dev = {
		.coherent_dma_mask = ~0,
		.dma_mask = &ehci1_dmamask,
		.platform_data = &usbh1_id,
	},
	.num_resources = ARRAY_SIZE(ehci1_resources),
	.resource = ehci1_resources,
};

static u64 ohci0_dmamask = ~0;

struct platform_device spear13xx_ohci0_device = {
	.name = "spear-ohci",
	.id = 0,
	.dev = {
		.coherent_dma_mask = ~0,
		.dma_mask = &ohci0_dmamask,
		.platform_data = &usbh0_id,
	},
	.num_resources = ARRAY_SIZE(ohci0_resources),
	.resource = ohci0_resources,
};

static u64 ohci1_dmamask = ~0;
struct platform_device spear13xx_ohci1_device = {
	.name = "spear-ohci",
	.id = 1,
	.dev = {
		.coherent_dma_mask = ~0,
		.dma_mask = &ohci1_dmamask,
		.platform_data = &usbh1_id,
	},
	.num_resources = ARRAY_SIZE(ohci1_resources),
	.resource = ohci1_resources,
};

/* keyboard device registration */
static struct resource kbd_resources[] = {
	{
		.start = SPEAR13XX_KBD_BASE,
		.end = SPEAR13XX_KBD_BASE + SZ_1K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR13XX_IRQ_KBD,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear13xx_kbd_device = {
	.name = "keyboard",
	.id = -1,
	.num_resources = ARRAY_SIZE(kbd_resources),
	.resource = kbd_resources,
};

/* rtc device registration */
static struct resource rtc_resources[] = {
	{
		.start = SPEAR13XX_RTC_BASE,
		.end = SPEAR13XX_RTC_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR13XX_IRQ_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear13xx_rtc_device = {
	.name = "rtc-spear",
	.id = -1,
	.num_resources = ARRAY_SIZE(rtc_resources),
	.resource = rtc_resources,
};

/* sdhci (sdio) device declaration */
static struct resource sdhci_resources[] = {
	{
		.start	= SPEAR13XX_MCIF_SDHCI_BASE,
		.end	= SPEAR13XX_MCIF_SDHCI_BASE + SZ_256 - 1,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= SPEAR13XX_IRQ_SDHCI,
		.flags	= IORESOURCE_IRQ,
	}
};

struct platform_device spear13xx_sdhci_device = {
	.dev = {
		.coherent_dma_mask = ~0,
	},
	.name = "sdhci",
	.id = -1,
	.num_resources = ARRAY_SIZE(sdhci_resources),
	.resource = sdhci_resources,
};

/* smi device registration */
static struct resource smi_resources[] = {
	{
		.start = SPEAR13XX_SMI_CTRL_BASE,
		.end = SPEAR13XX_SMI_CTRL_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR13XX_IRQ_SMI,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear13xx_smi_device = {
	.name = "smi",
	.id = -1,
	.num_resources = ARRAY_SIZE(smi_resources),
	.resource = smi_resources,
};

/* wdt device registration */
static struct resource wdt_resources[] = {
	{
		.start = SPEAR13XX_WDT_BASE,
		.end = SPEAR13XX_WDT_BASE + SZ_256 - 1,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device spear13xx_wdt_device = {
	.name = "cortexa9-wdt",
	.id = -1,
	.num_resources = ARRAY_SIZE(wdt_resources),
	.resource = wdt_resources,
};

struct platform_device spear13xx_pcm_device = {
	.name		= "spear-pcm-audio",
	.id		= -1,
};

/* pcie host/gadget registration */
static int pcie_gadget0_id;

static u64 pcie_gadget0_dmamask = ~0;

static struct pcie_port_info	pcie_host0_info;

static u64 pcie_host0_dmamask = ~0;

static struct resource pcie0_resources[] = {
	{
		.start = SPEAR13XX_PCIE0_APP_BASE,
		.end = SPEAR13XX_PCIE0_APP_BASE + SZ_16K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR13XX_PCIE0_BASE,
		.end = SPEAR13XX_PCIE0_BASE + SZ_8K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR13XX_IRQ_PCIE0,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear13xx_pcie_gadget0_device = {
	.name = "pcie-gadget-spear",
	.id = 0,
	.dev = {
		.coherent_dma_mask = ~0,
		.dma_mask = &pcie_gadget0_dmamask,
		.platform_data = &pcie_gadget0_id,
	},
	.num_resources = ARRAY_SIZE(pcie0_resources),
	.resource = pcie0_resources,
};

struct platform_device spear13xx_pcie_host0_device = {
	.name = "dw_pcie",
	.id = 0,
	.dev = {
		.coherent_dma_mask = ~0,
		.dma_mask = &pcie_host0_dmamask,
		.platform_data = &pcie_host0_info,
	},
	.num_resources = ARRAY_SIZE(pcie0_resources),
	.resource = pcie0_resources,
};

/*
 * Devices present on CPU_SPEAR1300, CPU_SPEAR1310, CPU_SPEAR1310_REVA or
 * CPU_SPEAR900
 */
#if defined(CONFIG_CPU_SPEAR1300) || defined(CONFIG_CPU_SPEAR1310_REVA) || \
			defined(CONFIG_CPU_SPEAR900) || \
			defined(CONFIG_CPU_SPEAR1310)
/* cpufreq platform device */
static u32 cpu_freq_tbl[] = {
	200000, /* 200 MHZ */
	250000, /* 250 MHZ */
	332000, /* 332 MHZ */
	400000, /* 400 MHZ */
	500000, /* 500 MHZ */
};

static struct spear_cpufreq_pdata cpufreq_pdata = {
	.cpu_freq_table = cpu_freq_tbl,
	.tbl_len = ARRAY_SIZE(cpu_freq_tbl),
};

struct platform_device spear13xx_cpufreq_device = {
	.name = "cpufreq-spear",
	.id = -1,
	.dev = {
		.platform_data = &cpufreq_pdata,
	},
};

/*
 * configure i2s ref clk and sclk
 *
 * Depending on these parameters sclk and ref clock will be configured.
 * For sclk:
 * sclk = channel_num * data_len * sample_rate
 *
 * For ref clock:
 *
 * ref_clock = 256 * sample_rate
 */

int audio_clk_config(struct i2s_clk_config_data *config)
{
	struct clk *i2s_sclk_clk, *i2s_ref_clk;
	int ret;
	u32 bclk;

	i2s_sclk_clk = clk_get_sys(NULL, "i2s_sclk_clk");
	if (IS_ERR(i2s_sclk_clk)) {
		pr_err("%s:couldn't get i2s_sclk_clk\n", __func__);
		return PTR_ERR(i2s_sclk_clk);
	}

	i2s_ref_clk = clk_get_sys(NULL, "i2s_ref_clk");
	if (IS_ERR(i2s_ref_clk)) {
		pr_err("%s:couldn't get i2s_ref_clk\n", __func__);
		ret = PTR_ERR(i2s_ref_clk);
		goto put_i2s_sclk_clk;
	}

	if (machine_is_spear1340_evb()) {
		if (config->sample_rate != 48000) {
			ret = clk_set_parent_sys(NULL, "i2s_ref_clk", NULL,
					"i2s_prs1_clk");
			if (ret) {
				pr_err("%s:set_parent of ref_clk fail\n",
						__func__);
				goto put_i2s_sclk_clk;
			}
		} else {
			ret = clk_set_parent_sys(NULL, "i2s_ref_clk", NULL,
					"i2s_src_clk");
			if (ret) {
				pr_err("%s:set_parent of ref_clk fail\n",
						__func__);
				goto put_i2s_sclk_clk;
			}
			goto config_bclk;
		}
	}

	ret = clk_set_rate(i2s_ref_clk, 256 * config->sample_rate);
	if (ret) {
		pr_err("%s:couldn't set i2s_ref_clk rate\n", __func__);
		goto put_i2s_ref_clk;
	}

config_bclk:
	bclk = config->chan_nr * config->data_width * config->sample_rate;

	ret = clk_set_rate(i2s_sclk_clk, bclk);
	if (ret) {
		pr_err("%s:couldn't set i2s_sclk_clk rate\n", __func__);
		goto put_i2s_ref_clk;
	}

	ret = clk_enable(i2s_sclk_clk);
	if (ret) {
		pr_err("%s:enabling i2s_sclk_clk\n", __func__);
		goto put_i2s_ref_clk;
	}

	return 0;

put_i2s_ref_clk:
	clk_put(i2s_ref_clk);
put_i2s_sclk_clk:
	clk_put(i2s_sclk_clk);

	return ret;

}

/* i2s0 device registeration */
static struct dw_dma_slave i2s0_dma_data[] = {
	{
		/* Play */
		.dma_dev = &spear13xx_dmac_device[0].dev,
		.cfg_hi = DWC_CFGH_DST_PER(SPEAR13XX_DMA_REQ_I2S_TX),
		.cfg_lo = 0,
		.src_master = SPEAR13XX_DMA_MASTER_MEMORY,
		.dst_master = SPEAR13XX_DMA_MASTER_I2S,
	}, {
		/* Record */
		.dma_dev = &spear13xx_dmac_device[0].dev,
		.cfg_hi = DWC_CFGH_SRC_PER(SPEAR13XX_DMA_REQ_I2S_RX),
		.cfg_lo = 0,
		.src_master = SPEAR13XX_DMA_MASTER_I2S,
		.dst_master = SPEAR13XX_DMA_MASTER_MEMORY,
	}
};

static struct i2s_platform_data i2s0_data = {
	.cap = PLAY | RECORD,
	.channel = 4,
	.snd_fmts = SNDRV_PCM_FMTBIT_S16_LE,
	.snd_rates = (SNDRV_PCM_RATE_8000 | \
		 SNDRV_PCM_RATE_11025 | \
		 SNDRV_PCM_RATE_16000 | \
		 SNDRV_PCM_RATE_22050 | \
		 SNDRV_PCM_RATE_32000 | \
		 SNDRV_PCM_RATE_44100 | \
		 SNDRV_PCM_RATE_48000),
	.play_dma_data = &i2s0_dma_data[0],
	.capture_dma_data = &i2s0_dma_data[1],
	.filter = dw_dma_filter,
	.i2s_clk_cfg = audio_clk_config,
};

static struct resource i2s0_resources[] = {
	{
		.start	= SPEAR13XX_I2S0_BASE,
		.end	= SPEAR13XX_I2S0_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	}, {

		.name	= "play_irq",
		.start	= SPEAR13XX_IRQ_PLAY_I2S0,
		.flags	= IORESOURCE_IRQ,
	}, {
		.name	= "record_irq",
		.start	= SPEAR13XX_IRQ_REC_I2S0,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device spear13xx_i2s0_device = {
	.name = "designware-i2s",
	.id = 0,
	.dev = {
		.coherent_dma_mask = ~0,
		.platform_data = &i2s0_data,
	},
	.num_resources = ARRAY_SIZE(i2s0_resources),
	.resource = i2s0_resources,
};

/* i2s1 device registeration */
static struct dw_dma_slave i2s1_dma_data[] = {
	{
		/* Play */
		.dma_dev = &spear13xx_dmac_device[0].dev,
		.cfg_hi = DWC_CFGH_DST_PER(SPEAR13XX_DMA_REQ_I2S_TX),
		.cfg_lo = 0,
		.src_master = 0,
		.dst_master = 1,
	}, {
		/* Record */
		.dma_dev = &spear13xx_dmac_device[0].dev,
		.cfg_hi = DWC_CFGH_SRC_PER(SPEAR13XX_DMA_REQ_I2S_RX),
		.cfg_lo = 0,
		.src_master = 1,
		.dst_master = 0,
	}
};

static struct i2s_platform_data i2s1_data = {
	.cap = PLAY | RECORD,
	.channel = 4,
	.snd_fmts = SNDRV_PCM_FMTBIT_S16_LE,
	.snd_rates = (SNDRV_PCM_RATE_8000 | \
		 SNDRV_PCM_RATE_11025 | \
		 SNDRV_PCM_RATE_16000 | \
		 SNDRV_PCM_RATE_22050 | \
		 SNDRV_PCM_RATE_32000 | \
		 SNDRV_PCM_RATE_44100 | \
		 SNDRV_PCM_RATE_48000),
	.play_dma_data = &i2s1_dma_data[0],
	.capture_dma_data = &i2s1_dma_data[1],
	.filter = dw_dma_filter,
	.i2s_clk_cfg = audio_clk_config,
};

static struct resource i2s1_resources[] = {
	{
		.start	= SPEAR13XX_I2S1_BASE,
		.end	= SPEAR13XX_I2S1_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	}, {

		.name	= "play_irq",
		.start	= SPEAR13XX_IRQ_PLAY_I2S1,
		.flags	= IORESOURCE_IRQ,
	}, {
		.name	= "record_irq",
		.start	= SPEAR13XX_IRQ_REC_I2S1,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device spear13xx_i2s1_device = {
	.name = "designware-i2s",
	.id = 1,
	.dev = {
		.coherent_dma_mask = ~0,
		.platform_data = &i2s1_data,
	},
	.num_resources = ARRAY_SIZE(i2s1_resources),
	.resource = i2s1_resources,
};

/* jpeg device registeration */
static struct dw_dma_slave jpeg_dma_param[] = {
	{
		/* mem2jpeg */
		.dma_dev = &spear13xx_dmac_device[0].dev,
		.cfg_hi = DWC_CFGH_DST_PER(SPEAR13XX_DMA_REQ_TO_JPEG),
		.cfg_lo = 0,
		.src_master = SPEAR13XX_DMA_MASTER_MEMORY,
		.dst_master = SPEAR13XX_DMA_MASTER_JPEG,
	}, {
		/* jpeg2mem */
		.dma_dev = &spear13xx_dmac_device[0].dev,
		.cfg_hi = DWC_CFGH_SRC_PER(SPEAR13XX_DMA_REQ_FROM_JPEG),
		.cfg_lo = 0,
		.src_master = SPEAR13XX_DMA_MASTER_JPEG,
		.dst_master = SPEAR13XX_DMA_MASTER_MEMORY,
	}
};

static struct jpeg_plat_data jpeg_pdata = {
	.dma_filter = dw_dma_filter,
	.mem2jpeg_slave = &jpeg_dma_param[0],
	.jpeg2mem_slave = &jpeg_dma_param[1],
};

static struct resource jpeg_resources[] = {
	{
		.start = SPEAR13XX_JPEG_BASE,
		.end = SPEAR13XX_JPEG_BASE + SZ_8K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR13XX_IRQ_JPEG,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear13xx_jpeg_device = {
	.name = "jpeg-designware",
	.id = -1,
	.dev = {
		.coherent_dma_mask = ~0,
		.platform_data = &jpeg_pdata,
	},
	.num_resources = ARRAY_SIZE(jpeg_resources),
	.resource = jpeg_resources,
};

/* pcie host/gadget registration */
static int pcie_gadget1_id;
static int pcie_gadget2_id;

static u64 pcie_gadget1_dmamask = ~0;
static u64 pcie_gadget2_dmamask = ~0;

static struct pcie_port_info	pcie_host1_info;
static struct pcie_port_info	pcie_host2_info;

static u64 pcie_host1_dmamask = ~0;
static u64 pcie_host2_dmamask = ~0;

static struct resource pcie1_resources[] = {
	{
		.start = SPEAR13XX_PCIE1_APP_BASE,
		.end = SPEAR13XX_PCIE1_APP_BASE + SZ_16K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR13XX_PCIE1_BASE,
		.end = SPEAR13XX_PCIE1_BASE + SZ_8K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR13XX_IRQ_PCIE1,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource pcie2_resources[] = {
	{
		.start = SPEAR13XX_PCIE2_APP_BASE,
		.end = SPEAR13XX_PCIE2_APP_BASE + SZ_16K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR13XX_PCIE2_BASE,
		.end = SPEAR13XX_PCIE2_BASE + SZ_8K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR13XX_IRQ_PCIE2,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear13xx_pcie_gadget1_device = {
	.name = "pcie-gadget-spear",
	.id = 1,
	.dev = {
		.coherent_dma_mask = ~0,
		.dma_mask = &pcie_gadget1_dmamask,
		.platform_data = &pcie_gadget1_id,
	},
	.num_resources = ARRAY_SIZE(pcie1_resources),
	.resource = pcie1_resources,
};

struct platform_device spear13xx_pcie_gadget2_device = {
	.name = "pcie-gadget-spear",
	.id = 2,
	.dev = {
		.coherent_dma_mask = ~0,
		.dma_mask = &pcie_gadget2_dmamask,
		.platform_data = &pcie_gadget2_id,
	},
	.num_resources = ARRAY_SIZE(pcie2_resources),
	.resource = pcie2_resources,
};

struct platform_device spear13xx_pcie_host1_device = {
	.name = "dw_pcie",
	.id = 1,
	.dev = {
		.coherent_dma_mask = ~0,
		.dma_mask = &pcie_host1_dmamask,
		.platform_data = &pcie_host1_info,
	},
	.num_resources = ARRAY_SIZE(pcie1_resources),
	.resource = pcie1_resources,
};

struct platform_device spear13xx_pcie_host2_device = {
	.name = "dw_pcie",
	.id = 2,
	.dev = {
		.coherent_dma_mask = ~0,
		.dma_mask = &pcie_host2_dmamask,
		.platform_data = &pcie_host2_info,
	},
	.num_resources = ARRAY_SIZE(pcie2_resources),
	.resource = pcie2_resources,
};

#endif

/*Devices present on CPU_SPEAR1300, CPU_SPEAR1310_REVA or CPU_SPEAR900 */
#if defined(CONFIG_CPU_SPEAR1300) || defined(CONFIG_CPU_SPEAR1310_REVA) || \
			defined(CONFIG_CPU_SPEAR900)
/* usb device registeration */
static struct resource udc_resources[] = {
	[0] = {
		.start = SPEAR13XX_UDC_BASE,
		.end = SPEAR13XX_UDC_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = SPEAR13XX_UPD_BASE,
		.end = SPEAR13XX_UPD_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[3] = {
		.start = SPEAR13XX_IRQ_UDC,
		.end = SPEAR13XX_IRQ_UDC,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear13xx_udc_device = {
	.name = "designware_udc",
	.id = -1,
	.dev = {
		.coherent_dma_mask = 0xffffffff,
	},
	.num_resources = ARRAY_SIZE(udc_resources),
	.resource = udc_resources,
};
#endif

static void dmac_setup(void)
{
	/*
	 * This function does static initialization of few misc regs for dmac
	 * operations.
	 */
	/* setting Peripheral flow controller for jpeg */
	writel(1 << SPEAR13XX_DMA_REQ_FROM_JPEG, VA_DMAC_FLOW_SEL);
}

/*
 * Generic function to configure ethernet phy clock as per the selected
 * interface
 */
int spear13xx_eth_phy_clk_cfg(struct platform_device *pdev)
{
	int ret;
	struct clk *input_clk, *input_pclk, *phy_pclk, *phy_clk;
	struct plat_stmmacenet_data *pdata = dev_get_platdata(&pdev->dev);
	const char *phy_clk_src_name[] = {
		"gmac_phy_input_clk",
		"gmac_phy_synth_clk",
	};
	const char *input_clk_src_name[] = {
		"pll2_clk",
		"gmii_125m_pad_clk",
		"osc3_25m_clk",
	};
	const char *phy_clk_name[] = {
		"stmmacphy.0"
	};

	if (pdata == NULL)
		return -EFAULT;

	/* Get the Pll-2 Clock as parent for PHY Input Clock Source */
	input_pclk = clk_get(NULL, input_clk_src_name[0]);
	if (IS_ERR(input_pclk)) {
		ret = PTR_ERR(input_pclk);
		goto fail_get_input_pclk;
	}

	/*
	 * Get the Phy Input clock source as parent for Phy clock. Default
	 * selection is gmac_phy_input_clk. This selection would be driving both
	 * the synthesizer and phy clock.
	 */
	input_clk = clk_get(NULL, phy_clk_src_name[0]);
	if (IS_ERR(input_clk)) {
		ret = PTR_ERR(input_clk);
		goto fail_get_input_clk;
	}

	/* Fetch the phy clock */
	phy_clk = clk_get(NULL, phy_clk_name[pdata->bus_id]);
	if (IS_ERR(phy_clk)) {
		ret = PTR_ERR(phy_clk);
		goto fail_get_phy_clk;
	}

	/* Set the pll-2 to 125 MHz */
	clk_set_rate(input_pclk, 125000000);

	/* Set the Pll-2 as parent for gmac_phy_input_clk */
	clk_set_parent(input_clk, input_pclk);

	if (pdata->interface == PHY_INTERFACE_MODE_RMII) {
		/*
		 * For the rmii interface select gmac_phy_synth_clk
		 * as the parent and set the clock to 50 Mhz
		 */
		phy_pclk = clk_get(NULL, phy_clk_src_name[1]);
		clk_set_rate(phy_pclk, 50000000);
	} else {
		/*
		 * Set the gmac_phy_input_clk as the parent,
		 * and pll-2 is already running as parent of
		 * gmac_phy_input_clk at 125 Mhz
		 */
		phy_pclk = input_clk;
	}

	/* Select the parent for phy clock */
	clk_set_parent(phy_clk, phy_pclk);
	ret = clk_enable(phy_clk);

	return ret;
fail_get_phy_clk:
	clk_put(input_clk);
fail_get_input_clk:
	clk_put(input_pclk);
fail_get_input_pclk:
	return ret;
}

#ifdef CONFIG_SND_SOC_STA529
static void i2s_clk_init(void)
{
	struct clk *i2s_ref_pad_clk, *i2s_sclk_clk;
	char *src_pclk_name, *ref_pclk_name;

	if (machine_is_spear1340_lcad() || !cpu_is_spear1340()) {
		if (machine_is_spear1340_lcad())
			src_pclk_name = "pll2_clk";
		else
			src_pclk_name = "pll3_clk";

		 ref_pclk_name = "i2s_prs1_clk";

		/* set pll to 49.15 Mhz */
		if (clk_set_rate_sys(NULL, src_pclk_name, 49152000)) {
			pr_err("%s:set_rate of %s fail\n", __func__,
					src_pclk_name);
			return;
		}
	} else {
		src_pclk_name = "i2s_src_pad_clk";
		ref_pclk_name = "i2s_src_clk";
	}

	/*
	 * After this this src_clk is correctly programmed, either to
	 * pll2, pll3 or pad.
	 */
	if (clk_set_parent_sys(NULL, "i2s_src_clk", NULL, src_pclk_name)) {
		pr_err("%s:set_parent to %s for i2s_src_clk fail\n",
				__func__, src_pclk_name);
		return;
	}

	/* program prescalar if required */
	if (machine_is_spear1340_lcad() || !cpu_is_spear1340()) {
		/* set to 12.288 Mhz */
		if (clk_set_rate_sys(NULL, ref_pclk_name, 12288000)) {
			pr_err("%s:set_rate of %s fail\n", __func__,
					ref_pclk_name);
			return;
		}
	}

	/*
	 * After this this ref_clk is correctly programmed to 12.288 and
	 * sclk_clk to 1.536 MHz
	 */
	if (clk_set_parent_sys(NULL, "i2s_ref_clk", NULL, ref_pclk_name)) {
		pr_err("%s:set_parent to %s of ref_clk fail\n",
				__func__, ref_pclk_name);
		return;
	}

	i2s_sclk_clk = clk_get_sys(NULL, "i2s_sclk_clk");
	if (IS_ERR(i2s_sclk_clk)) {
		pr_err("%s:couldn't get i2s_sclk_clk\n", __func__);
		return;
	}

	i2s_ref_pad_clk = clk_get_sys(NULL, "i2s_ref_pad_clk");
	if (IS_ERR(i2s_ref_pad_clk)) {
		pr_err("%s:couldn't get i2s_ref_pad_clk\n", __func__);
		goto put_sclk_clk;
	}

	if (clk_enable(i2s_ref_pad_clk)) {
		pr_err("%s:enabling i2s_ref_pad_clk_fail\n", __func__);
		goto put_ref_pad_clk;
	}

	if (clk_enable(i2s_sclk_clk)) {
		pr_err("%s:enabling i2s_sclk_clk\n", __func__);
		goto put_ref_pad_clk;
	}

put_ref_pad_clk:
	clk_put(i2s_ref_pad_clk);
put_sclk_clk:
	clk_put(i2s_sclk_clk);
}
#endif

void spear13xx_l2x0_init(void)
{
#ifdef CONFIG_CACHE_L2X0
	/*
	 * 512KB (64KB/way), 8-way associativity, parity supported
	 *
	 * FIXME: 9th bit, of Auxillary Controller register must be set
	 * for some spear13xx devices for stable L2 operation.
	 *
	 * Enable Early BRESP, L2 prefetch for Instruction and Data,
	 * write alloc and 'Full line of zero' options
	 *
	 */

	writel_relaxed(0x06, __io_address(SPEAR13XX_L2CC_BASE)
			+ L2X0_PREFETCH_CTRL);

	if (cpu_is_spear1340() || cpu_is_spear1310()) {
		/*
		 * Program following latencies in order to make
		 * SPEAr1340 work at 600 MHz
		 */
		writel_relaxed(0x221, __io_address(SPEAR13XX_L2CC_BASE)
				+ L2X0_TAG_LATENCY_CTRL);
		writel_relaxed(0x441, __io_address(SPEAR13XX_L2CC_BASE)
				+ L2X0_DATA_LATENCY_CTRL);
		l2x0_init(__io_address(SPEAR13XX_L2CC_BASE), 0x70A60001,
				0xfe00ffff);
	} else {
		l2x0_init(__io_address(SPEAR13XX_L2CC_BASE), 0x70A60201,
				0xfe00ffff);
	}
#endif
}

/* Do spear13xx familiy common initialization part here */
void spear13xx_init(void)
{
	int ret;

	spear13xx_l2x0_init();

#ifdef CONFIG_SND_SOC_STA529
	i2s_clk_init();
#endif

	dmac_setup();
#if defined(CONFIG_CPU_SPEAR1300) || defined(CONFIG_CPU_SPEAR1310_REVA) || \
			defined(CONFIG_CPU_SPEAR900)
	if (!cpu_is_spear1340() && !cpu_is_spear1310())
		set_udc_plat_data(&spear13xx_udc_device);
#endif

	ret = clk_set_rate_sys("sdhci", NULL, 50000000);
	if (ret)
		pr_err("clk_set_rate failed for sdhci: %d\n", ret);

#ifdef CONFIG_PCI
	/* PCI specific initializations */
	pcibios_min_io = 0x1000;
	pcibios_min_mem = 0;
	pci_set_flags(0);
#endif
}

static int spear13xx_set_wake(struct irq_data *data, unsigned int on)
{
	return 0;
}

/* This will initialize gic */
void __init spear13xx_init_irq(void)
{
	gic_init(0, 29, __io_address(SPEAR13XX_GIC_DIST_BASE),
			__io_address(SPEAR13XX_GIC_CPU_BASE));
	gic_arch_extn.irq_set_wake = spear13xx_set_wake;
}

unsigned long reserve_mem(struct meminfo *mi, unsigned long size)
{
	unsigned long addr = ~0;
	int i;
	for (i = mi->nr_banks - 1; i >= 0; i--)
		if (mi->bank[i].size >= size) {
			mi->bank[i].size -= size;
			addr = mi->bank[i].start + mi->bank[i].size;
			break;
		}

	return addr;
}

/* Following will create static virtual/physical mappings */
static struct map_desc spear13xx_io_desc[] __initdata = {
	{
		.virtual	= IO_ADDRESS(SPEAR13XX_UART_BASE),
		.pfn		= __phys_to_pfn(SPEAR13XX_UART_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	}, {
		.virtual	= IO_ADDRESS(SPEAR13XX_A9SM_PERIP_BASE),
		.pfn		= __phys_to_pfn(SPEAR13XX_A9SM_PERIP_BASE),
		.length		= SZ_8K,
		.type		= MT_DEVICE
#ifdef CONFIG_CACHE_L2X0
	}, {
		.virtual	= IO_ADDRESS(SPEAR13XX_L2CC_BASE),
		.pfn		= __phys_to_pfn(SPEAR13XX_L2CC_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE
#endif
	}, {
		.virtual	= IO_ADDRESS(SPEAR13XX_MISC_BASE),
		.pfn		= __phys_to_pfn(SPEAR13XX_MISC_BASE),
		.length		= SZ_8K,
		.type		= MT_DEVICE
	}, {
		.virtual	= IO_ADDRESS(SPEAR13XX_SYSRAM0_BASE),
		.pfn		= __phys_to_pfn(SPEAR13XX_SYSRAM0_BASE),
		.length		= SZ_32K,
		.type		= MT_DEVICE
	}, {
		.virtual	= IO_ADDRESS(SPEAR13XX_SYSRAM1_BASE),
		.pfn		= __phys_to_pfn(SPEAR13XX_SYSRAM1_BASE),
		.length		= SZ_4K,
		.type		= MT_MEMORY_NONCACHED
	}, {
		.virtual	= IO_ADDRESS(SPEAR13XX_PCIE0_BASE),
		.pfn		= __phys_to_pfn(SPEAR13XX_PCIE0_BASE),
		.length		= SZ_64K,
		.type		= MT_DEVICE
	}, {
		.virtual	= IO_ADDRESS(SPEAR13XX_PCIE1_BASE),
		.pfn		= __phys_to_pfn(SPEAR13XX_PCIE1_BASE),
		.length		= SZ_64K,
		.type		= MT_DEVICE
	}, {
		.virtual	= IO_ADDRESS(SPEAR13XX_PCIE2_BASE),
		.pfn		= __phys_to_pfn(SPEAR13XX_PCIE2_BASE),
		.length		= SZ_64K,
		.type		= MT_DEVICE
	}
};

/* This will create static memory mapping for selected devices */
void __init spear13xx_map_io(void)
{
	iotable_init(spear13xx_io_desc, ARRAY_SIZE(spear13xx_io_desc));

	/* This will initialize clock framework */
	if (cpu_is_spear1340()) {
#ifdef CONFIG_CPU_SPEAR1340
		spear1340_clk_init();
#endif
	} else if (cpu_is_spear1310()) {
#ifdef CONFIG_CPU_SPEAR1310
		spear1310_clk_init();
#endif
	} else
		spear13xx_clk_init();
}

static void __init spear13xx_timer_init(void)
{
	char pclk_name[] = "osc1_24m_clk";
	struct clk *gpt_clk, *pclk;

#ifdef CONFIG_LOCAL_TIMERS
	/* Setup the local timer base */
	twd_base = __io_address(SPEAR13XX_LOCAL_TMR_BASE);
#endif

	/* get the system timer clock */
	gpt_clk = clk_get_sys("gpt0", NULL);
	if (IS_ERR(gpt_clk)) {
		pr_err("%s:couldn't get clk for gpt\n", __func__);
		BUG();
	}

	/* get the suitable parent clock for timer*/
	pclk = clk_get(NULL, pclk_name);
	if (IS_ERR(pclk)) {
		pr_err("%s:couldn't get %s as parent for gpt\n",
				__func__, pclk_name);
		BUG();
	}

	clk_set_parent(gpt_clk, pclk);
	clk_put(gpt_clk);
	clk_put(pclk);

	spear_setup_timer();
}

struct sys_timer spear13xx_timer = {
	.init = spear13xx_timer_init,
};

#if defined(CONFIG_CPU_SPEAR1300) || defined(CONFIG_CPU_SPEAR1310_REVA) || \
			defined(CONFIG_CPU_SPEAR900) || \
			defined(CONFIG_CPU_SPEAR1310)
/* pad multiplexing support */
/* devices */

/* Pad multiplexing for i2c device */
static struct pmx_mux_reg pmx_i2c_mux[] = {
	{
		.address = PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_I2C_MASK,
		.value = SPEAR13XX_PMX_I2C_MASK,
	},
};

static struct pmx_dev_mode pmx_i2c_modes[] = {
	{
		.mux_regs = pmx_i2c_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_i2c_mux),
	},
};

struct pmx_dev spear13xx_pmx_i2c = {
	.name = "i2c",
	.modes = pmx_i2c_modes,
	.mode_count = ARRAY_SIZE(pmx_i2c_modes),
};

/* Pad multiplexing for ssp device */
static struct pmx_mux_reg pmx_ssp_mux[] = {
	{
		.address = PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_SSP_MASK,
		.value = SPEAR13XX_PMX_SSP_MASK,
	},
};

static struct pmx_dev_mode pmx_ssp_modes[] = {
	{
		.mux_regs = pmx_ssp_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_ssp_mux),
	},
};

struct pmx_dev spear13xx_pmx_ssp = {
	.name = "ssp",
	.modes = pmx_ssp_modes,
	.mode_count = ARRAY_SIZE(pmx_ssp_modes),
};

/* Pad multiplexing for i2s1 device */
static struct pmx_mux_reg pmx_i2s1_mux[] = {
	{
		.address = PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_I2S1_MASK,
		.value = SPEAR13XX_PMX_I2S1_MASK,
	},
};

static struct pmx_dev_mode pmx_i2s1_modes[] = {
	{
		.mux_regs = pmx_i2s1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_i2s1_mux),
	},
};

struct pmx_dev spear13xx_pmx_i2s1 = {
	.name = "i2s1",
	.modes = pmx_i2s1_modes,
	.mode_count = ARRAY_SIZE(pmx_i2s1_modes),
};

/* Pad multiplexing for i2s2 device */
static struct pmx_mux_reg pmx_i2s2_mux[] = {
	{
		.address = PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_I2S2_MASK,
		.value = SPEAR13XX_PMX_I2S2_MASK,
	},
};

static struct pmx_dev_mode pmx_i2s2_modes[] = {
	{
		.mux_regs = pmx_i2s2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_i2s2_mux),
	},
};

struct pmx_dev spear13xx_pmx_i2s2 = {
	.name = "i2s2",
	.modes = pmx_i2s2_modes,
	.mode_count = ARRAY_SIZE(pmx_i2s2_modes),
};

/* Pad multiplexing for clcd device */
static struct pmx_mux_reg pmx_clcd_mux[] = {
	{
		.address = PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_CLCD1_MASK,
		.value = SPEAR13XX_PMX_CLCD1_MASK,
	},
};

static struct pmx_dev_mode pmx_clcd_modes[] = {
	{
		.mux_regs = pmx_clcd_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_clcd_mux),
	},
};

struct pmx_dev spear13xx_pmx_clcd = {
	.name = "clcd",
	.modes = pmx_clcd_modes,
	.mode_count = ARRAY_SIZE(pmx_clcd_modes),
};

/* Pad multiplexing for clcd_hires device */
static struct pmx_mux_reg pmx_clcd_hires_mux[] = {
	{
		.address = PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_CLCD1_MASK,
		.value = SPEAR13XX_PMX_CLCD1_MASK,
	}, {
		.address = PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_CLCD2_MASK,
		.value = SPEAR13XX_PMX_CLCD2_MASK,
	},
};

static struct pmx_dev_mode pmx_clcd_hires_modes[] = {
	{
		.mux_regs = pmx_clcd_hires_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_clcd_hires_mux),
	},
};

struct pmx_dev spear13xx_pmx_clcd_hires = {
	.name = "clcd_high_res",
	.modes = pmx_clcd_hires_modes,
	.mode_count = ARRAY_SIZE(pmx_clcd_hires_modes),
};

/*
 * By default, all EGPIOs are enabled.
 * TBD : Board specific enabling of specific GPIOs only
 */
static struct pmx_mux_reg pmx_egpio_grp_mux[] = {
	{
		.address = PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_EGPIO_0_GRP_MASK,
		.value = SPEAR13XX_PMX_EGPIO_0_GRP_MASK,
	}, {
		.address = PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_EGPIO_1_GRP_MASK,
		.value = SPEAR13XX_PMX_EGPIO_1_GRP_MASK,
	},
};

static struct pmx_dev_mode pmx_egpio_grp_modes[] = {
	{
		.mux_regs = pmx_egpio_grp_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_egpio_grp_mux),
	},
};

struct pmx_dev spear13xx_pmx_egpio_grp = {
	.name = "egpios",
	.modes = pmx_egpio_grp_modes,
	.mode_count = ARRAY_SIZE(pmx_egpio_grp_modes),
};

/* Pad multiplexing for smi 2 chips device */
static struct pmx_mux_reg pmx_smi_2_mux[] = {
	{
		.address = PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_SMI_MASK,
		.value = SPEAR13XX_PMX_SMI_MASK,
	},
};

static struct pmx_dev_mode pmx_smi_2_modes[] = {
	{
		.mux_regs = pmx_smi_2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_smi_2_mux),
	},
};

struct pmx_dev spear13xx_pmx_smi_2_chips = {
	.name = "smi_2_chips",
	.modes = pmx_smi_2_modes,
	.mode_count = ARRAY_SIZE(pmx_smi_2_modes),
};

/* Pad multiplexing for smi 4 chips device */
static struct pmx_mux_reg pmx_smi_4_mux[] = {
	{
		.address = PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_SMI_MASK,
		.value = SPEAR13XX_PMX_SMI_MASK,
	}, {
		.address = PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_SMINCS2_MASK | SPEAR13XX_PMX_SMINCS3_MASK,
		.value = SPEAR13XX_PMX_SMINCS2_MASK | \
			 SPEAR13XX_PMX_SMINCS3_MASK,
	},
};

static struct pmx_dev_mode pmx_smi_4_modes[] = {
	{
		.mux_regs = pmx_smi_4_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_smi_4_mux),
	},
};

struct pmx_dev spear13xx_pmx_smi_4_chips = {
	.name = "smi_4_chips",
	.modes = pmx_smi_4_modes,
	.mode_count = ARRAY_SIZE(pmx_smi_4_modes),
};

/* Pad multiplexing for gmii device */
static struct pmx_mux_reg pmx_gmii_mux[] = {
	{
		.address = PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_GMII_MASK,
		.value = SPEAR13XX_PMX_GMII_MASK,
	},
};

static struct pmx_dev_mode pmx_gmii_modes[] = {
	{
		.mux_regs = pmx_gmii_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_gmii_mux),
	},
};

struct pmx_dev spear13xx_pmx_gmii = {
	.name = "gmii",
	.modes = pmx_gmii_modes,
	.mode_count = ARRAY_SIZE(pmx_gmii_modes),
};

/* Pad multiplexing for nand 8bit (4 chips) */
static struct pmx_mux_reg pmx_nand8_4_mux[] = {
	{
		.address = PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_NAND8BIT4DEV_0_MASK,
		.value = SPEAR13XX_PMX_NAND8BIT4DEV_0_MASK,
	}, {
		.address = PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_NAND8BIT4DEV_1_MASK | \
			SPEAR13XX_PMX_KEYBOARD_6X6_MASK,
		.value = SPEAR13XX_PMX_NAND8BIT4DEV_1_MASK | \
			 SPEAR13XX_PMX_KEYBOARD_6X6_MASK,
	},
};

static struct pmx_dev_mode pmx_nand8_4_modes[] = {
	{
		.mux_regs = pmx_nand8_4_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_nand8_4_mux),
	},
};

struct pmx_dev spear13xx_pmx_nand_8bit_4_chips = {
	.name = "nand-8bit_4_chips",
	.modes = pmx_nand8_4_modes,
	.mode_count = ARRAY_SIZE(pmx_nand8_4_modes),
};

/* Pad multiplexing for nand 8bit device (cs0 only) */
static struct pmx_mux_reg pmx_nand8_mux[] = {
	{
		.address = PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_NAND8BIT_0_MASK,
		.value = SPEAR13XX_PMX_NAND8BIT_0_MASK,
	}, {
		.address = PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_NAND8BIT_1_MASK | \
			SPEAR13XX_PMX_KEYBOARD_6X6_MASK,
		.value = SPEAR13XX_PMX_NAND8BIT_1_MASK | \
			 SPEAR13XX_PMX_KEYBOARD_6X6_MASK,
	},
};

static struct pmx_dev_mode pmx_nand8_modes[] = {
	{
		.mux_regs = pmx_nand8_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_nand8_mux),
	},
};

struct pmx_dev spear13xx_pmx_nand_8bit = {
	.name = "nand-8bit",
	.modes = pmx_nand8_modes,
	.mode_count = ARRAY_SIZE(pmx_nand8_modes),
};

/*
 * Pad multiplexing for nand 16bit device
 * Note : Enabling pmx_nand_16bit means that all the required pads for
 *   16bit nand device operations are enabled. These also include pads
 *   for 8bit devices
 */
static struct pmx_mux_reg pmx_nand16_4_mux[] = {
	{
		.address = PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_NAND16BIT4DEV_0_MASK,
		.value = SPEAR13XX_PMX_NAND16BIT4DEV_0_MASK,
	}, {
		.address = PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_NAND16BIT4DEV_1_MASK | \
			SPEAR13XX_PMX_KEYBOARD_6X6_MASK,
		.value = SPEAR13XX_PMX_NAND16BIT4DEV_1_MASK | \
			 SPEAR13XX_PMX_KEYBOARD_6X6_MASK,
	},
};

static struct pmx_dev_mode pmx_nand16_4_modes[] = {
	{
		.mux_regs = pmx_nand16_4_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_nand16_4_mux),
	},
};

struct pmx_dev spear13xx_pmx_nand_16bit_4_chips = {
	.name = "nand-16bit_4_chips",
	.modes = pmx_nand16_4_modes,
	.mode_count = ARRAY_SIZE(pmx_nand16_4_modes),
};

/* Pad multiplexing for nand 16bit device (cs0 only) */
static struct pmx_mux_reg pmx_nand16_mux[] = {
	{
		.address = PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_NAND16BIT_0_MASK,
		.value = SPEAR13XX_PMX_NAND16BIT_0_MASK,
	}, {
		.address = PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_NAND16BIT_1_MASK | \
			SPEAR13XX_PMX_KEYBOARD_6X6_MASK,
		.value = SPEAR13XX_PMX_NAND16BIT_1_MASK | \
			 SPEAR13XX_PMX_KEYBOARD_6X6_MASK,
	},
};

static struct pmx_dev_mode pmx_nand16_modes[] = {
	{
		.mux_regs = pmx_nand16_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_nand16_mux),
	},
};

struct pmx_dev spear13xx_pmx_nand_16bit = {
	.name = "nand-16bit",
	.modes = pmx_nand16_modes,
	.mode_count = ARRAY_SIZE(pmx_nand16_modes),
};

/* Pad multiplexing for keyboard_6x6 device */
static struct pmx_mux_reg pmx_keyboard_6x6_mux[] = {
	{
		.address = PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_KEYBOARD_6X6_MASK,
		.value = SPEAR13XX_PMX_KEYBOARD_6X6_MASK,
	}, {
		.address = PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_NFIO815_MASK | SPEAR13XX_PMX_NFCE1_MASK |\
			SPEAR13XX_PMX_NFCE2_MASK | SPEAR13XX_PMX_NFWPRT1_MASK |\
			SPEAR13XX_PMX_NFWPRT2_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_keyboard_6x6_modes[] = {
	{
		.mux_regs = pmx_keyboard_6x6_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_keyboard_6x6_mux),
	},
};

struct pmx_dev spear13xx_pmx_keyboard_6x6 = {
	.name = "keyboard_6x6",
	.modes = pmx_keyboard_6x6_modes,
	.mode_count = ARRAY_SIZE(pmx_keyboard_6x6_modes),
};

/* Pad multiplexing for keyboard_9x9 device */
static struct pmx_mux_reg pmx_keyboard_9x9_mux[] = {
	{
		.address = PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_KEYBOARD_6X6_MASK | \
			SPEAR13XX_PMX_KBD_ROWCOL68_MASK,
		.value = SPEAR13XX_PMX_KEYBOARD_6X6_MASK | \
			 SPEAR13XX_PMX_KBD_ROWCOL68_MASK,
	}, {
		.address = PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_NFIO815_MASK | SPEAR13XX_PMX_NFCE1_MASK |\
			SPEAR13XX_PMX_NFCE2_MASK | SPEAR13XX_PMX_NFWPRT1_MASK |\
			SPEAR13XX_PMX_NFWPRT2_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_keyboard_9x9_modes[] = {
	{
		.mux_regs = pmx_keyboard_9x9_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_keyboard_9x9_mux),
	},
};

struct pmx_dev spear13xx_pmx_keyboard_9x9 = {
	.name = "keyboard_9x9",
	.modes = pmx_keyboard_9x9_modes,
	.mode_count = ARRAY_SIZE(pmx_keyboard_9x9_modes),
};

/* Pad multiplexing for uart0 device */
static struct pmx_mux_reg pmx_uart0_mux[] = {
	{
		.address = PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_UART0_MASK,
		.value = SPEAR13XX_PMX_UART0_MASK,
	},
};

static struct pmx_dev_mode pmx_uart0_modes[] = {
	{
		.mux_regs = pmx_uart0_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart0_mux),
	},
};

struct pmx_dev spear13xx_pmx_uart0 = {
	.name = "uart0",
	.modes = pmx_uart0_modes,
	.mode_count = ARRAY_SIZE(pmx_uart0_modes),
};

/* Pad multiplexing for uart0_modem device */
static struct pmx_mux_reg pmx_uart0_modem_mux[] = {
	{
		.address = PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_UART0_MODEM_MASK,
		.value = SPEAR13XX_PMX_UART0_MODEM_MASK,
	},
};

static struct pmx_dev_mode pmx_uart0_modem_modes[] = {
	{
		.mux_regs = pmx_uart0_modem_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart0_modem_mux),
	},
};

struct pmx_dev spear13xx_pmx_uart0_modem = {
	.name = "uart0_modem",
	.modes = pmx_uart0_modem_modes,
	.mode_count = ARRAY_SIZE(pmx_uart0_modem_modes),
};

/* Pad multiplexing for gpt_0_1 device */
static struct pmx_mux_reg pmx_gpt_0_1_mux[] = {
	{
		.address = PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_GPT0_TMR1_MASK,
		.value = SPEAR13XX_PMX_GPT0_TMR1_MASK,
	},
};

static struct pmx_dev_mode pmx_gpt_0_1_modes[] = {
	{
		.mux_regs = pmx_gpt_0_1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_gpt_0_1_mux),
	},
};

struct pmx_dev spear13xx_pmx_gpt_0_1 = {
	.name = "gpt_0_1",
	.modes = pmx_gpt_0_1_modes,
	.mode_count = ARRAY_SIZE(pmx_gpt_0_1_modes),
};

/* Pad multiplexing for gpt_0_2 device */
static struct pmx_mux_reg pmx_gpt_0_2_mux[] = {
	{
		.address = PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_GPT0_TMR2_MASK,
		.value = SPEAR13XX_PMX_GPT0_TMR2_MASK,
	},
};

static struct pmx_dev_mode pmx_gpt_0_2_modes[] = {
	{
		.mux_regs = pmx_gpt_0_2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_gpt_0_2_mux),
	},
};

struct pmx_dev spear13xx_pmx_gpt_0_2 = {
	.name = "gpt_0_2",
	.modes = pmx_gpt_0_2_modes,
	.mode_count = ARRAY_SIZE(pmx_gpt_0_2_modes),
};

/* Pad multiplexing for gpt_1_1 device */
static struct pmx_mux_reg pmx_gpt_1_1_mux[] = {
	{
		.address = PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_GPT1_TMR1_MASK,
		.value = SPEAR13XX_PMX_GPT1_TMR1_MASK,
	},
};

static struct pmx_dev_mode pmx_gpt_1_1_modes[] = {
	{
		.mux_regs = pmx_gpt_1_1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_gpt_1_1_mux),
	},
};

struct pmx_dev spear13xx_pmx_gpt_1_1 = {
	.name = "gpt_1_1",
	.modes = pmx_gpt_1_1_modes,
	.mode_count = ARRAY_SIZE(pmx_gpt_1_1_modes),
};

/* Pad multiplexing for gpt_1_2 device */
static struct pmx_mux_reg pmx_gpt_1_2_mux[] = {
	{
		.address = PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_GPT1_TMR2_MASK,
		.value = SPEAR13XX_PMX_GPT1_TMR2_MASK,
	},
};

static struct pmx_dev_mode pmx_gpt_1_2_modes[] = {
	{
		.mux_regs = pmx_gpt_1_2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_gpt_1_2_mux),
	},
};

struct pmx_dev spear13xx_pmx_gpt_1_2 = {
	.name = "gpt_1_2",
	.modes = pmx_gpt_1_2_modes,
	.mode_count = ARRAY_SIZE(pmx_gpt_1_2_modes),
};

/* Pad multiplexing for mcif device */
static struct pmx_mux_reg pmx_mcif_mux[] = {
	{
		.address = PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_MCI_DATA8_15_MASK,
		.value = SPEAR13XX_PMX_MCI_DATA8_15_MASK,
	}, {
		.address = PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_MCIFALL_1_MASK,
		.value = SPEAR13XX_PMX_MCIFALL_1_MASK,
	}, {
		.address = PAD_FUNCTION_EN_3,
		.mask = SPEAR13XX_PMX_MCIFALL_2_MASK,
		.value = SPEAR13XX_PMX_MCIFALL_2_MASK,
	},
};

static struct pmx_dev_mode pmx_mcif_modes[] = {
	{
		.mux_regs = pmx_mcif_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_mcif_mux),
	},
};

struct pmx_dev spear13xx_pmx_mcif = {
	.name = "mcif",
	.modes = pmx_mcif_modes,
	.mode_count = ARRAY_SIZE(pmx_mcif_modes),
};

/* Pad multiplexing for sdhci device */
static struct pmx_mux_reg pmx_sdhci_mux[] = {
	{
		.address = PERIP_CFG,
		.mask = MCIF_SEL_MASK,
		.value = MCIF_SEL_SD,
	},
};

static struct pmx_dev_mode pmx_sdhci_modes[] = {
	{
		.mux_regs = pmx_sdhci_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_sdhci_mux),
	},
};

struct pmx_dev spear13xx_pmx_sdhci = {
	.name = "sdhci",
	.modes = pmx_sdhci_modes,
	.mode_count = ARRAY_SIZE(pmx_sdhci_modes),
};

/* Pad multiplexing for cf device */
static struct pmx_mux_reg pmx_cf_mux[] = {
	{
		.address = PERIP_CFG,
		.mask = MCIF_SEL_MASK,
		.value = MCIF_SEL_CF,
	},
};

static struct pmx_dev_mode pmx_cf_modes[] = {
	{
		.mux_regs = pmx_cf_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_cf_mux),
	},
};

struct pmx_dev spear13xx_pmx_cf = {
	.name = "cf",
	.modes = pmx_cf_modes,
	.mode_count = ARRAY_SIZE(pmx_cf_modes),
};

/* Pad multiplexing for xd device */
static struct pmx_mux_reg pmx_xd_mux[] = {
	{
		.address = PERIP_CFG,
		.mask = MCIF_SEL_MASK,
		.value = MCIF_SEL_XD,
	},
};

static struct pmx_dev_mode pmx_xd_modes[] = {
	{
		.mux_regs = pmx_xd_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_xd_mux),
	},
};

struct pmx_dev spear13xx_pmx_xd = {
	.name = "xd",
	.modes = pmx_xd_modes,
	.mode_count = ARRAY_SIZE(pmx_xd_modes),
};

/* Pad multiplexing for touch_xy device */
static struct pmx_mux_reg pmx_touch_xy_mux[] = {
	{
		.address = PAD_FUNCTION_EN_3,
		.mask = SPEAR13XX_PMX_TOUCH_XY_MASK,
		.value = SPEAR13XX_PMX_TOUCH_XY_MASK,
	},
};

static struct pmx_dev_mode pmx_touch_xy_modes[] = {
	{
		.mux_regs = pmx_touch_xy_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_touch_xy_mux),
	},
};

struct pmx_dev spear13xx_pmx_touch_xy = {
	.name = "touch_xy",
	.modes = pmx_touch_xy_modes,
	.mode_count = ARRAY_SIZE(pmx_touch_xy_modes),
};

/* Pad multiplexing for ssp0_cs0 device */
static struct pmx_mux_reg pmx_ssp0_cs0_mux[] = {
	{
		.address = PAD_FUNCTION_EN_3,
		.mask = SPEAR13XX_PMX_SSP0_CS0_MASK,
		.value = SPEAR13XX_PMX_SSP0_CS0_MASK,
	},
};

static struct pmx_dev_mode pmx_ssp0_cs0_modes[] = {
	{
		.mux_regs = pmx_ssp0_cs0_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_ssp0_cs0_mux),
	},
};

struct pmx_dev spear13xx_pmx_ssp0_cs0 = {
	.name = "ssp0_cs0",
	.modes = pmx_ssp0_cs0_modes,
	.mode_count = ARRAY_SIZE(pmx_ssp0_cs0_modes),
};

/* Pad multiplexing for ssp0_cs1_2 device */
static struct pmx_mux_reg pmx_ssp0_cs1_2_mux[] = {
	{
		.address = PAD_FUNCTION_EN_3,
		.mask = SPEAR13XX_PMX_SSP0_CS1_2_MASK,
		.value = SPEAR13XX_PMX_SSP0_CS1_2_MASK,
	},
};

static struct pmx_dev_mode pmx_ssp0_cs1_2_modes[] = {
	{
		.mux_regs = pmx_ssp0_cs1_2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_ssp0_cs1_2_mux),
	},
};

struct pmx_dev spear13xx_pmx_ssp0_cs1_2 = {
	.name = "ssp0_cs1_2",
	.modes = pmx_ssp0_cs1_2_modes,
	.mode_count = ARRAY_SIZE(pmx_ssp0_cs1_2_modes),
};
#endif
