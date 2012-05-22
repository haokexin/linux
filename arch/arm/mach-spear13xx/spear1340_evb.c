/*
 * arch/arm/mach-spear13xx/spear1340_evb.c
 *
 * SPEAr1340 evaluation board source file
 *
 * Copyright (C) 2011 ST Microelectronics
 * Viresh Kumar <viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c/l3g4200d.h>
#include <linux/i2c/lsm303dlh.h>
#include <linux/irq.h>
#include <linux/mfd/stmpe.h>
#include <linux/mtd/fsmc.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/spear_smi.h>
#include <linux/netdevice.h>
#include <linux/pata_arasan_cf_data.h>
#include <linux/phy.h>
#include <linux/spi/spi.h>
#include <linux/stmmac.h>
#include <video/db9000fb.h>
#include <asm/hardware/gic.h>
#include <plat/i2c.h>
#include <plat/keyboard.h>
#include <plat/plug_board.h>
#include <plat/spi.h>
#include <mach/generic.h>
#include <mach/hardware.h>
#include <mach/spear1340_misc_regs.h>
#include <media/soc_camera.h>
#include <media/vip.h>

/* camera sensor registeration */
static struct i2c_board_info vs6725_camera_sensor_info = {
	I2C_BOARD_INFO("vs6725", 0x10),
};

/* Camera power: default is ON */
static int vs6725_cam_power(struct device *dev, int val)
{
	int ret = 0;
	static bool gpio_avail;

	if (!gpio_avail) {

		ret = gpio_request(STMPE801_GPIO_6, "vs6725-power");
		if (!ret) {
			gpio_direction_output(STMPE801_GPIO_6, 0);
		} else {
			pr_err("gpio request fail for STMPE801_GPIO_6\n");
			goto out;
		}

		gpio_avail = true;
	}

	/* turn on/off the CE pin for camera sensor */
	gpio_set_value_cansleep(STMPE801_GPIO_6, val);

	/*
	 * Now check if we really were able to set the desired value on CE
	 * pin of the sensor
	 */
	ret = gpio_get_value_cansleep(STMPE801_GPIO_6);
	if (ret != val) {
		pr_err("gpio get_val returned %d but expected %d\n", ret, val);
		ret = -ERESTARTSYS;
	}

out:
	return ret;
}

static struct soc_camera_link vs6725_cam3_sensor_iclink = {
	.bus_id = 3,	/* sensor is connected to camera device 3 */
	.i2c_adapter_id = 0, /* sensor is connected to i2c controller 0 */
	.board_info = &vs6725_camera_sensor_info,
	.power = vs6725_cam_power,
	.module_name = "vs6725",
};

static struct platform_device spear1340_cam3_sensor_device = {
	.name = "soc-camera-pdrv",
	.id = -1,
	.dev = {
		.platform_data = &vs6725_cam3_sensor_iclink,
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
		/*
		 * PLGPIO 12 is used for interrupt reception from
		 * STMPE801.
		 */
		.mask = 0x2000,
		.value = 0x0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_2,
		.mask = 0x0,
		.value = 0x0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_4,
		.mask = 0x00000020, /* enabling I2S_OUT_DATA_3 as PL-GPIO */
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
		.mask = 0x1000000,
		.value = 0x0,
	},
};

static struct pmx_dev_mode pmx_plgpios_modes[] = {
	{
		.mux_regs = pmx_plgpios_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_plgpios_mux),
	},
};

static struct pmx_dev spear1340_pmx_plgpios = {
	.name = "plgpios",
	.modes = pmx_plgpios_modes,
	.mode_count = ARRAY_SIZE(pmx_plgpios_modes),
};

/* Pad Multiplexing for LSM303DLH Accelerometer and Magnetometer device */
static struct pmx_mux_reg lsm303_plgpios_mux[] = {
	{
		.address = SPEAR1340_PAD_FUNCTION_EN_3,
		.mask = 0x1180, /* PLGPIO: 70, 71, 75 */
		.value = 0x0,
	},
};

static struct pmx_dev_mode lsm303_plgpios_modes[] = {
	{
		.mux_regs = lsm303_plgpios_mux,
		.mux_reg_cnt = ARRAY_SIZE(lsm303_plgpios_mux),
	},
};

struct pmx_dev spear1340_pmx_lsm303 = {
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
	&spear1340_pmx_keyboard_row_col,
#if !defined(CONFIG_PM)
	&spear1340_pmx_keyboard_col5,
#endif
	&spear1340_pmx_uart0_enh,
	&spear1340_pmx_i2c1,
	&spear1340_pmx_spdif_in,
	&spear1340_pmx_ssp0_cs1,
#if !defined(CONFIG_PM)
	&spear1340_pmx_pwm2,
#endif
	&spear1340_pmx_pwm3,
	&spear1340_pmx_smi,
	&spear1340_pmx_ssp0,
	&spear1340_pmx_uart0,
	&spear1340_pmx_i2s_in,
	&spear1340_pmx_i2s_out,
	&spear1340_pmx_gmac,
	&spear1340_pmx_ssp0_cs3,
	&spear1340_pmx_i2c0,
	&spear1340_pmx_cam3,
	&spear1340_pmx_cec0,
	&spear1340_pmx_cec1,
	&spear1340_pmx_spdif_out,
	&spear1340_pmx_mcif,
	&spear1340_pmx_sdhci,
	&spear1340_pmx_clcd,
	&spear1340_pmx_clcd_gpio_pd,
	&spear1340_pmx_devs_grp,
	&spear1340_pmx_rgmii,
	&spear1340_pmx_sata,
	&spear1340_pmx_lsm303,

	/* Keep this entry at the bottom of table to override earlier setting */
	&spear1340_pmx_plgpios,
};

static struct amba_device *amba_devs[] __initdata = {
	/* spear13xx specific devices */
	&spear13xx_gpio_device[0],
	&spear13xx_gpio_device[1],
	&spear13xx_ssp_device,
	&spear13xx_uart_device,

	/* spear1340 specific devices */
	&spear1340_uart1_device,
};

static struct platform_device *plat_devs[] __initdata = {
	/* spear13xx specific devices */
	&spear13xx_adc_device,
	&spear13xx_db9000_clcd_device,
	&spear13xx_dmac_device[0],
	&spear13xx_dmac_device[1],
	&spear13xx_ehci0_device,
	&spear13xx_ehci1_device,
	&spear13xx_eth_device,
	&spear13xx_i2c_device,
	&spear13xx_ohci0_device,
	&spear13xx_ohci1_device,
	&spear13xx_pcm_device,
	&spear13xx_rtc_device,
	&spear13xx_sdhci_device,
	&spear13xx_smi_device,
	&spear13xx_wdt_device,

	/* spear1340 specific devices */
	&spear1340_camif3_device,
	&spear1340_cam3_sensor_device,
	&spear1340_cec0_device,
	&spear1340_cec1_device,
	&spear1340_cpufreq_device,
#ifdef CONFIG_DRM_MALI
	&spear1340_device_mali_drm,
#endif
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
	&spear1340_gpiokeys_device,
#endif
	&spear1340_i2c1_device,
	&spear1340_i2s_play_device,
	&spear1340_i2s_record_device,
	&spear1340_nand_device,
	&spear1340_otg_device,
	&spear1340_plgpio_device,
	&spear1340_pwm_device,
	&spear1340_sata0_device,
	&spear1340_spdif_in_device,
	&spear1340_spdif_out_device,
	&spear1340_thermal_device,
	&spear1340_video_dec_device,
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

static const struct fsmc_rbpin fsmc_rbpin __initconst = {
	/*
	 * PLGPIO 247 is also enabled along with this ready/busy feature.
	 * Disable the plgpio when ready/busy pin is being used by controller
	 * itself
	 */
	.use_pin = FSMC_RB_GPIO,
	.gpio_pin = PLGPIO_247,
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
	.rbpin = &fsmc_rbpin,
};

/* arasan compact flash controller's platform data */
static struct arasan_cf_pdata cf_pdata = {
	.cf_if_clk = CF_IF_CLK_166M,
	.quirk = CF_BROKEN_UDMA,
	.dma_priv = &cf_dma_priv,
};

/* keyboard specific platform data */
static const __initconst DECLARE_6x6_KEYMAP(keymap);
static const struct matrix_keymap_data keymap_data __initconst = {
	.keymap = keymap,
	.keymap_size = ARRAY_SIZE(keymap),
};

static const struct kbd_platform_data kbd_data __initconst = {
	.keymap = &keymap_data,
	.rep = 1,
	.mode = KEYPAD_2x2,
};

/* Ethernet specific plat data */
/* MDIO Bus Data */
static struct stmmac_mdio_bus_data mdio0_private_data = {
	.bus_id = 0,
	.phy_mask = 0,
};

static struct stmmac_dma_cfg dma0_private_data = {
	.pbl = 16,
	.fixed_burst = 1,
	.burst_len = DMA_AXI_BLEN_ALL,
};

static struct plat_stmmacenet_data eth_data = {
	.bus_id = 0,
	.phy_addr = -1,
	.interface = PHY_INTERFACE_MODE_RGMII,
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

/* Initializing platform data for spear1340 evb specific I2C devices */

/* STMPE801 platform data */
static struct stmpe_gpio_platform_data stmpe801_gpio = {
	.gpio_base = SPEAR_STMPE801_GPIO_BASE,
	.norequest_mask = 0,
};
static struct stmpe_platform_data stmpe801_pdata = {
	.id = 0,
	.blocks = STMPE_BLOCK_GPIO,
	.irq_base = SPEAR_STMPE801_GPIO_INT_BASE,
	.irq_gpio = GPIO0_4,
	.irq_over_gpio = true,
	.irq_trigger = IRQF_TRIGGER_FALLING,
	.irq_invert_polarity = false,
	.autosleep = false,
	.gpio = &stmpe801_gpio,
};

static struct i2c_board_info spear1340_evb_i2c_board_stmpe801 = {
	I2C_BOARD_INFO("stmpe801", 0x41),
	.platform_data = &stmpe801_pdata,
};

struct i2c_dev_info spear1340_evb_i2c_stmpe801 = {
	.board = &spear1340_evb_i2c_board_stmpe801,
	.busnum = 1,
};

/* Gyroscope platform data */
static struct l3g4200d_gyr_platform_data l3g4200d_pdata = {
	.poll_interval = 5,
	.min_interval = 2,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
};

/* LSM303DLH Accelerometer and Magnetometer platform data */
static struct lsm303dlh_platform_data lsm303dlh_a_pdata = {
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.input_poll_dev = 1,
	.name_a = "lsm303dlh_a",
#ifdef CONFIG_INPUT_ST_LSM303DLH_INPUT_DEVICE
	.irq_a1 = PLGPIO_71,
	.irq_a2 = PLGPIO_75,
#endif
};

static struct lsm303dlh_platform_data lsm303dlh_m_pdata = {
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.name_m = "lsm303dlh_m",
#ifdef CONFIG_INPUT_ST_LSM303DLH_INPUT_DEVICE
	.irq_m = PLGPIO_70,
#endif
};

static struct i2c_board_info spear1340_evb_i2c_board_l3g4200d_gyr = {
	/* gyroscope board info */
	.type = "l3g4200d_gyr",
	.addr = 0x69,
	.platform_data = &l3g4200d_pdata,
};

struct i2c_dev_info spear1340_evb_i2c_l3g4200d_gyr = {
	.board = &spear1340_evb_i2c_board_l3g4200d_gyr,
	.busnum = 0,
};

/* lsm303dlh accelerometer board info */
static struct i2c_board_info spear1340_evb_i2c_lsm303dlh_acc = {
	.type = "lsm303dlh_a",
	.addr = 0x19,
	.platform_data = &lsm303dlh_a_pdata,
};

struct i2c_dev_info spear1340_evb_i2c_lsm303dlh_a = {
	.board = &spear1340_evb_i2c_lsm303dlh_acc,
	.busnum = 0,
};

/* lsm303dlh magnetometer board info */
static struct i2c_board_info spear1340_evb_i2c_lsm303dlh_mag = {
	.type = "lsm303dlh_m",
	.addr = 0x1E,
	.platform_data = &lsm303dlh_m_pdata,
};

struct i2c_dev_info spear1340_evb_i2c_lsm303dlh_m = {
	.board = &spear1340_evb_i2c_lsm303dlh_mag,
	.busnum = 0,
};

static struct i2c_board_info spear1340_evb_i2c_board_eeprom0 = {
	.type = "eeprom",
	.addr = 0x50,
};

struct i2c_dev_info spear1340_evb_i2c_eeprom0 = {
	.board = &spear1340_evb_i2c_board_eeprom0,
	.busnum = 0,
};

static struct i2c_board_info spear1340_evb_i2c_board_eeprom1 = {
	.type = "eeprom",
	.addr = 0x51,
};

struct i2c_dev_info spear1340_evb_i2c_eeprom1 = {
	.board = &spear1340_evb_i2c_board_eeprom1,
	.busnum = 0,
};

static struct i2c_board_info spear1340_evb_i2c_board_sta529 = {
	.type = "sta529",
	.addr = 0x1a,
};

struct i2c_dev_info spear1340_evb_i2c_sta529 = {
	.board = &spear1340_evb_i2c_board_sta529,
	.busnum = 0,
};

static struct i2c_dev_info *i2c_devs[] __initdata = {
	&spear1340_evb_i2c_stmpe801,
	&spear1340_evb_i2c_l3g4200d_gyr,
	&spear1340_evb_i2c_eeprom0,
	&spear1340_evb_i2c_eeprom1,
	&spear1340_evb_i2c_lsm303dlh_a,
	&spear1340_evb_i2c_lsm303dlh_m,
	&spear1340_evb_i2c_sta529,
};

/* Definitions for SPI Devices*/

/* spi master's configuration routine */
DECLARE_SPI_CS_CFG(0, VA_SPEAR1340_PERIP_CFG, SPEAR1340_SSP_CS_SEL_MASK,
		SPEAR1340_SSP_CS_SEL_SHIFT, SPEAR1340_SSP_CS_CTL_MASK,
		SPEAR1340_SSP_CS_CTL_SHIFT, SPEAR1340_SSP_CS_CTL_SW,
		SPEAR1340_SSP_CS_VAL_MASK, SPEAR1340_SSP_CS_VAL_SHIFT);

/* spi0 flash Chip Select Control function */
DECLARE_SPI_CS_CONTROL(0, flash, SPEAR1340_SSP_CS_SEL_CS0);
/* spi0 flash Chip Info structure */
DECLARE_SPI_CHIP_INFO(0, flash, spi0_flash_cs_control);

/* spi0 spidev Chip Select Control function */
DECLARE_SPI_CS_CONTROL(0, dev, SPEAR1340_SSP_CS_SEL_CS2);
/* spi0 spidev Chip Info structure */
DECLARE_SPI_CHIP_INFO(0, dev, spi0_dev_cs_control);

/* spi0 touch screen Chip Select Control function, controlled by gpio pin */
DECLARE_SPI_CS_CONTROL(0, ts, SPEAR1340_SSP_CS_SEL_CS1);
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
	.cs_control = spi0_ts_cs_control,
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
	.irq_gpio = PLGPIO_100,
	.ts = &stmpe610_ts_pdata,
};

struct spi_board_info spear1340_evb_spi_m25p80 = {
	.modalias = "m25p80",
	.controller_data = &spi0_flash_chip_info,
	.max_speed_hz = 12000000,
	.bus_num = 0,
	.chip_select = SPEAR1340_SSP_CS_SEL_CS0,
	.mode = SPI_MODE_3,
};
struct spi_board_info spear1340_evb_spi_stmpe610 = {
	.modalias = "stmpe610",
	.platform_data = &stmpe610_pdata,
	.controller_data = &spi0_ts_chip_info,
	.max_speed_hz = 1000000,
	.bus_num = 0,
	.chip_select = SPEAR1340_SSP_CS_SEL_CS1,
	.mode = SPI_MODE_1,
};
struct spi_board_info spear1340_evb_spi_spidev = {
	.modalias = "spidev",
	.controller_data = &spi0_dev_chip_info,
	.max_speed_hz = 25000000,
	.bus_num = 0,
	.chip_select = SPEAR1340_SSP_CS_SEL_CS2,
	.mode = SPI_MODE_1,
};

static struct spi_board_info *spi_board[] __initdata = {
	&spear1340_evb_spi_m25p80,
	&spear1340_evb_spi_stmpe610,
	&spear1340_evb_spi_spidev,
};

static void
spear1340_evb_fixup(struct tag *tags, char **cmdline, struct meminfo *mi)
{
#if defined(CONFIG_FB_DB9000) || defined(CONFIG_FB_DB9000_MODULE)
	spear13xx_panel_fixup(mi);
#endif

#ifdef CONFIG_VIDEO_SPEAR_VIP
	vip_buffer_fixup(mi);
#endif
}

static void __init spear1340_evb_init(void)
{
	unsigned int i;

	/* set compact flash plat data */
	set_arasan_cf_pdata(&spear13xx_cf_device, &cf_pdata);

#if (defined(CONFIG_FB_DB9000) || defined(CONFIG_FB_DB9000_MODULE))
	/* db9000_clcd plat data */
	spear13xx_panel_init(&spear13xx_db9000_clcd_device);
#endif

	/* call spear1340 machine init function */
	spear1340_init(NULL, pmx_devs, ARRAY_SIZE(pmx_devs));

	/* Set stmmac plat data */
	if (platform_device_add_data(&spear13xx_eth_device, &eth_data,
			sizeof(eth_data)))
		printk(KERN_WARNING "%s: couldn't add plat_data",
				spear13xx_eth_device.name);

	/* initialize serial nor related data in smi plat data */
	smi_init_board_info(&spear13xx_smi_device);

	/* set keyboard plat data */
	if (platform_device_add_data(&spear13xx_kbd_device, &kbd_data,
				sizeof(kbd_data)))
		printk(KERN_WARNING "%s: couldn't add plat_data",
				spear13xx_kbd_device.name);

	/*
	 * SPEAr1340 FSMC cannot used as NOR and NAND at the same time
	 * For the moment, disable NOR and use NAND only
	 * If NOR is needed, enable NOR's code and disable all code for NOR.
	 */
	/* set nand device's plat data */
	nand_mach_init(FSMC_NAND_BW8);
	if (platform_device_add_data(&spear1340_nand_device, &nand_plat_data,
				sizeof(nand_plat_data)))
		printk(KERN_WARNING "%s: couldn't add plat_data",
				spear1340_nand_device.name);

#ifdef CONFIG_SPEAR1340_PLUG_BOARDS
	/* Check if plug boards are requested or not */
	if (spear_pb_present()) {
		struct plug_board_info pb_info;
		int ret;

		pb_info.pdevs = plat_devs;
		pb_info.pcnt = ARRAY_SIZE(plat_devs);
		pb_info.adevs = amba_devs;
		pb_info.acnt = ARRAY_SIZE(amba_devs);
		pb_info.spi_devs = spi_board;
		pb_info.spi_cnt = ARRAY_SIZE(spi_board);
		pb_info.i2c_devs = i2c_devs;
		pb_info.i2c_cnt = ARRAY_SIZE(i2c_devs);
		ret = spear1340_pb_init(&pb_info);
		if (!ret)
			return;
	}
#endif

	/* Register spear1340 evb board specific i2c slave devices */
	for (i = 0; i < ARRAY_SIZE(i2c_devs); i++)
		i2c_register_board_info(i2c_devs[i]->busnum,
				i2c_devs[i]->board, 1);

	/* Register SPI Board */
	for (i = 0; i < ARRAY_SIZE(spi_board); i++)
		spi_register_board_info(spi_board[i], 1);

	/* Add Platform Devices */
	platform_add_devices(plat_devs, ARRAY_SIZE(plat_devs));

	/* Add Amba Devices */
	for (i = 0; i < ARRAY_SIZE(amba_devs); i++)
		amba_device_register(amba_devs[i], &iomem_resource);
}

MACHINE_START(SPEAR1340_EVB, "ST-SPEAR1340-EVB")
	.atag_offset	=	0x100,
	.fixup		=	spear1340_evb_fixup,
	.map_io		=	spear13xx_map_io,
	.init_irq	=	spear13xx_init_irq,
	.handle_irq	=	gic_handle_irq,
	.timer		=	&spear13xx_timer,
	.init_machine	=	spear1340_evb_init,
	.restart	=	spear_restart,
MACHINE_END
