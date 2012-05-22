/*
 * arch/arm/mach-spear13xx/spear1340_lcad.c
 *
 * SPEAr1340 based Low Cost Access Device (LCAD) Board's source file
 *
 * Copyright (C) 2012 ST Microelectronics
 * Viresh Kumar <viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/amba/pl061.h>
#include <linux/clk.h>
#include <linux/types.h>
#include <linux/gpio.h>

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#include <linux/gpio_keys.h>
#include <linux/input.h>
#endif

#include <linux/i2c.h>
#include <linux/i2c/lsm303dlh.h>
#include <linux/irq.h>
#include <linux/mtd/fsmc.h>
#include <linux/mtd/nand.h>
#include <linux/spi/spi.h>
#include <media/soc_camera.h>
#include <video/db9000fb.h>
#include <asm/hardware/gic.h>
#include <plat/i2c.h>
#include <plat/spi.h>
#include <mach/generic.h>
#include <mach/hardware.h>
#include <mach/spear1340_misc_regs.h>

/* camera sensor registeration */
static struct i2c_board_info hi704_camera_sensor_info = {
	I2C_BOARD_INFO("hi704", 0x30),
};

/* Camera power: default is ON */
static int hi704_cam_power(struct device *dev, int val)
{
	int ret;
	static bool hi704_powered;

	if (!hi704_powered) {

		/*
		 * hi704 is chip enable pin is connected to 2 gpio's on board
		 * Keep one gpio always in input to keep the board safe.
		 * Similarly, 2 spear gpio's connect to vsync. So, keep 1 in
		 * input mode.
		 */
		ret = gpio_request(PLGPIO_40, "hi704-ce");
		ret |= gpio_request(PLGPIO_51, "hi704-ce-psuedo");
		ret |= gpio_request(PLGPIO_32, "hi704-vsync-psuedo");

		if (!ret) {
			gpio_direction_input(PLGPIO_51);
			gpio_direction_input(PLGPIO_32);
			gpio_direction_output(PLGPIO_40, 1);
		} else {
			pr_err("gpio request failed\n");
			return ret;
		}
		hi704_powered = true;
	}

	/* turn on/off the CE pin for camera sensor */
	if (val)
		gpio_set_value_cansleep(PLGPIO_40, 0);
	else
		gpio_set_value_cansleep(PLGPIO_40, 1);

	return 0;
}

static struct soc_camera_link hi704_cam0_sensor_iclink = {
	.bus_id = 0,	/* sensor is connected to camera device */
	.i2c_adapter_id = 1, /* sensor is connected to i2c controller 0 */
	.board_info = &hi704_camera_sensor_info,
	.power = hi704_cam_power,
	.module_name = "hi704",
};

static struct platform_device cam0_sensor_device = {
	.name = "soc-camera-pdrv",
	.id = -1,
	.dev = {
		.platform_data = &hi704_cam0_sensor_iclink,
	},
};

/*
 * Pad multiplexing for making few pads as plgpio's.
 * Please retain original values and addresses, and update only mask as
 * required.
 * For example: if we need to enable plgpio's on pads: 15, 28, 45 & 102.
 * They corresponds to following bits in registers: 16, 29, 46 & 103
 * So following mask entries will solve this purpose:
 * Reg1: .mask = 0x20010000,
 * Reg2: .mask = 0x00004000,
 * Reg4: .mask = 0x00000080,
 *
 * Note: Counting of bits and pads start from 0.
 */
static struct pmx_mux_reg pmx_plgpios_mux[] = {
	{
		.address = SPEAR1340_PAD_FUNCTION_EN_1,
		.mask = 0x0,
		.value = 0x0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_2,
		.mask = 0x0,
		.value = 0x0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_3,
		.mask = 0x0,
		.value = 0x0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_4,
		.mask = 0x0,
		.value = 0x0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_5,
		.mask = 0x0,
		.value = 0x0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_6,
		.mask = 0x0,
		.value = 0x0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_7,
		.mask = 0x0,
		.value = 0x0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_8,
		.mask = 0x0,
		.value = 0x0,
	},
};

static struct pmx_dev_mode pmx_plgpios_modes[] = {
	{
		.mux_regs = pmx_plgpios_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_plgpios_mux),
	},
};

static struct pmx_dev pmx_plgpios = {
	.name = "plgpios",
	.modes = pmx_plgpios_modes,
	.mode_count = ARRAY_SIZE(pmx_plgpios_modes),
};

/* SPEAr GPIO Buttons Info */
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
/* SPEAr GPIO Buttons definition */
#define SPEAR_GPIO_BTN9	9

static struct gpio_keys_button spear1340_lcad_gpio_keys_table[] = {
	{
		.code = BTN_0,
		.gpio = SPEAR_GPIO_BTN9,
		.active_low = 0,
		.desc = "gpio-keys: BTN0",
		.type = EV_KEY,
		.wakeup = 1,
		.debounce_interval = 20,
	}, {
		.code = KEY_BACK,
		.gpio = PLGPIO_53,
		.active_low = 1,
		.desc = "gpio-keys: BTN1:BACK",
		.type = EV_KEY,
		.wakeup = 0,
		.debounce_interval = 20,
	}, {
		.code = KEY_MENU,
		.gpio = PLGPIO_52,
		.active_low = 1,
		.desc = "gpio-keys: BTN2:MENU",
		.type = EV_KEY,
		.wakeup = 0,
		.debounce_interval = 20,
	}, {
		.code = KEY_HOME,
		.gpio = PLGPIO_48,
		.active_low = 1,
		.desc = "gpio-keys: BTN3:HOME",
		.type = EV_KEY,
		.wakeup = 0,
		.debounce_interval = 20,
	}, {
		.code = KEY_VOLUMEDOWN,
		.gpio = PLGPIO_37,
		.active_low = 1,
		.desc = "gpio-keys: BTN4:VOLUMEDOWN",
		.type = EV_KEY,
		.wakeup = 0,
		.debounce_interval = 20,
	}, {
		.code = KEY_VOLUMEUP,
		.gpio = PLGPIO_42,
		.active_low = 1,
		.desc = "gpio-keys: BTN5:VOLUMEUP",
		.type = EV_KEY,
		.wakeup = 0,
		.debounce_interval = 20,
	},
};

static struct gpio_keys_platform_data spear1340_lcad_gpio_keys_data = {
	.buttons = spear1340_lcad_gpio_keys_table,
	.nbuttons = ARRAY_SIZE(spear1340_lcad_gpio_keys_table),
};

struct platform_device spear1340_lcad_gpiokeys_device = {
	.name = "gpio-keys",
	.dev = {
		.platform_data = &spear1340_lcad_gpio_keys_data,
	},
};
#endif

/* Pad Multiplexing for LSM303DLH Accelerometer device */
static struct pmx_mux_reg lsm303_plgpios_mux[] = {
	{
		.address = SPEAR1340_PAD_FUNCTION_EN_2,
		.mask = 0x42100, /* PLGPIO: 39, 44, 49 */
		.value = 0x0,
	},
};

static struct pmx_dev_mode lsm303_plgpios_modes[] = {
	{
		.mux_regs = lsm303_plgpios_mux,
		.mux_reg_cnt = ARRAY_SIZE(lsm303_plgpios_mux),
	},
};

struct pmx_dev pmx_lsm303 = {
	.name = "lsm303_acc_mag",
	.modes = lsm303_plgpios_modes,
	.mode_count = ARRAY_SIZE(lsm303_plgpios_modes),
};

/* padmux devices to enable */
static struct pmx_dev *pmx_devs[] = {
	/*
	 * Keep pads_as_gpio as the first element in this array. Don't ever
	 * remove it. It makes all pads as gpio's in starting, and then pads are
	 * configured as peripherals wherever required.
	 */
	&spear1340_pmx_pads_as_gpio,
	&spear1340_pmx_fsmc_8bit,
	&spear1340_pmx_i2c1,
	&spear1340_pmx_ssp0_cs1,
	&spear1340_pmx_ssp0,
	&spear1340_pmx_uart0,
	&spear1340_pmx_i2s_in,
	&spear1340_pmx_i2s_out,
	&spear1340_pmx_i2c0,
	&spear1340_pmx_mcif,
	&spear1340_pmx_sdhci,
	&spear1340_pmx_cam0,
	&spear1340_pmx_clcd,
	&spear1340_pmx_clcd_gpio_pd,
	&spear1340_pmx_devs_grp,
	&pmx_lsm303,

	/* Keep this entry at the bottom of table to override earlier setting */
	&pmx_plgpios,
};

static struct amba_device *amba_devs[] __initdata = {
	/* spear13xx specific devices */
	&spear13xx_gpio_device[0],
	&spear13xx_gpio_device[1],
	&spear13xx_ssp_device,
	&spear13xx_uart_device,
};

static struct platform_device *plat_devs[] __initdata = {
	/* spear13xx specific devices */
	&spear13xx_adc_device,
	&spear13xx_db9000_clcd_device,
	&spear13xx_dmac_device[0],
	&spear13xx_dmac_device[1],
	&spear13xx_ehci0_device,
	&spear13xx_ehci1_device,
	&spear13xx_i2c_device,
	&spear1340_nand_device,
	&spear1340_i2s_play_device,
	&spear1340_i2s_record_device,
	&spear13xx_ohci0_device,
	&spear13xx_ohci1_device,
	&spear13xx_pcm_device,
	&spear13xx_rtc_device,
	&spear13xx_sdhci_device,
	&spear13xx_wdt_device,

	/* spear1340 specific devices */
	&spear1340_camif0_device,
	&cam0_sensor_device,
	&spear1340_i2c1_device,
	&spear1340_plgpio_device,
	&spear1340_otg_device,
	&spear1340_thermal_device,

	/* spear1340 lcad specific devices */
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
	&spear1340_lcad_gpiokeys_device,
#endif

};

static struct mtd_partition nand_partition_info[] __initdata = {
	{
		.name = "X-loader",
		.offset = 0,
		.size = 4 * 0x80000,
	}, {
		.name = "U-Boot",
		.offset = 4 * 0x80000,
		.size = 6 * 0x80000,
	}, {
		.name = "Kernel",
		.offset = (4 + 6) * 0x80000,
		.size = 24 * 0x80000,
	}, {
		.name = "Root File System",
		.offset = (4 + 12 + 24) * 0x80000,
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

/* I2S STA529 i2c board info */
static struct i2c_board_info i2c_sta529 = {
	.type = "sta529",
	.addr = 0x1a,
};

static struct i2c_dev_info i2c_dev_sta529 = {
	.board = &i2c_sta529,
	.busnum = 0,
};

/* lsm303dlh accelerometer board info */
static struct lsm303dlh_platform_data lsm303dlh_a_pdata = {
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.name_a = "lsm303dlh_a",
#ifdef CONFIG_INPUT_ST_LSM303DLH_INPUT_DEVICE
	.irq_a1 = PLGPIO_49,
	.irq_a2 = PLGPIO_44,
#endif
};

static struct i2c_board_info i2c_lsm303dlh_acc = {
	.type = "lsm303dlh_a",
	.addr = 0x19,
	.platform_data = &lsm303dlh_a_pdata,
};

static struct i2c_dev_info i2c_dev_lsm303dlh_acc = {
	.board = &i2c_lsm303dlh_acc,
	.busnum = 1,
};

/* lsm303dlh magnetometer board info */
static struct lsm303dlh_platform_data lsm303dlh_m_pdata = {
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.name_m = "lsm303dlh_m",
#ifdef CONFIG_INPUT_ST_LSM303DLH_INPUT_DEVICE
	.irq_m = PLGPIO_40,
#endif
};

static struct i2c_board_info i2c_lsm303dlh_mag = {
	.type = "lsm303dlh_m",
	.addr = 0x1E,
	.platform_data = &lsm303dlh_m_pdata,
};

static struct i2c_dev_info i2c_dev_lsm303dlh_mag = {
	.board = &i2c_lsm303dlh_mag,
	.busnum = 1,
};

static struct i2c_dev_info *i2c_devs[] __initdata = {
	&i2c_dev_sta529,
	&i2c_dev_lsm303dlh_acc,
	&i2c_dev_lsm303dlh_mag,
};

static void
lcad_fixup(struct tag *tags, char **cmdline, struct meminfo *mi)
{
#if defined(CONFIG_FB_DB9000) || defined(CONFIG_FB_DB9000_MODULE)
	spear13xx_panel_fixup(mi);
#endif
}

static void __init lcad_init(void)
{
	unsigned int i;
	struct pl061_platform_data *gpio0_pdata;

#if (defined(CONFIG_FB_DB9000) || defined(CONFIG_FB_DB9000_MODULE))
	/* db9000_clcd plat data */
	spear13xx_panel_init(&spear13xx_db9000_clcd_device);
#endif

	/* Set GPIO0_3 as 1 and others in input mode */
	gpio0_pdata = dev_get_platdata(&spear13xx_gpio_device[0].dev);
	gpio0_pdata->directions = 0x08;
	gpio0_pdata->values = 0x08;

	/* set nand device's plat data */
	nand_mach_init(FSMC_NAND_BW8);
	if (platform_device_add_data(&spear1340_nand_device, &nand_plat_data,
				sizeof(nand_plat_data)))
		printk(KERN_WARNING "%s: couldn't add plat_data",
				spear1340_nand_device.name);

	/* call spear1340 machine init function */
	spear1340_init(NULL, pmx_devs, ARRAY_SIZE(pmx_devs));

	/* Register spear1340 evb board specific i2c slave devices */
	for (i = 0; i < ARRAY_SIZE(i2c_devs); i++)
		i2c_register_board_info(i2c_devs[i]->busnum, i2c_devs[i]->board,
				1);

	/* Add Platform Devices */
	platform_add_devices(plat_devs, ARRAY_SIZE(plat_devs));

	/* Add Amba Devices */
	for (i = 0; i < ARRAY_SIZE(amba_devs); i++)
		amba_device_register(amba_devs[i], &iomem_resource);
}

MACHINE_START(SPEAR1340_LCAD, "SPEAR1340-LCAD")
	.atag_offset	=	0x100,
	.fixup		=	lcad_fixup,
	.map_io		=	spear13xx_map_io,
	.init_irq	=	spear13xx_init_irq,
	.handle_irq	=	gic_handle_irq,
	.timer		=	&spear13xx_timer,
	.init_machine	=	lcad_init,
	.restart	=	spear_restart,
MACHINE_END
