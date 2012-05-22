/*
 * arch/arm/mach-spear13xx/spear1340_plug_boards.c
 *
 * SPEAr1340 plug boards source file
 *
 * Copyright (C) 2011 ST Microelectronics
 * Viresh Kumar <viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#define pr_fmt(fmt) "spear1340_pb: " fmt

/*
 * Following are the plug boards available with SPEAr1340:
 * - gmii: gmii interface present for ethernet
 * - rgmii: rgmii interface present for ethernet
 * - etm: etm trace module
 * - hdmi_rx: hdmi receiver
 * - hdmi_tx: hdmi transmitter
 * - cam0: camera sensor connected to camera device 0 of SoC
 * - vga:
 * - sata: It is not a separate physical plug board but a board
 *   configuration
 * - pcie: It is not a separate physical plug board but a board
 *   configuration. sata and pcie are muxed and cannot be used together.
 *
 * Plug boards details can be found at:
 * https://codex.cro.st.com/plugins/docman/?group_id=1309&action=show&id=164214
 */

#include <linux/ad9889b.h>
#include <linux/bug.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/designware_i2s.h>
#include <linux/list.h>
#include <linux/phy.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/stmmac.h>
#include <sound/pcm.h>
#include <video/db9000fb.h>
#include <plat/plug_board.h>
#include <media/soc_camera.h>
#include <media/vip.h>
#include <plat/i2c.h>
#include <mach/generic.h>
#include <mach/hardware.h>
#include <mach/spear1340_misc_regs.h>
#include <mach/spear_pcie.h>

/*
 * FIXME: Update this later when the HDMI receiver chip driver is available:
 *
 * 1. Here, we assume that the SIL9135A HDMI receiver chip supports
 *    two inputs and a single output. The names of these macros can be
 *    updated later.
 * 2. Assume that SIL HDMI Rx chip supports all DV standards.
 *    Also note that analog standards like PAL and NTSC is also supported by
 *    VIP IP. So the STD supported by VIP driver will be a super-set
 *    of these DV and analog standards.
 */
#define SIL9135A_INPUT		1
#define SIL9135A_OUTPUT		2
#define SIL9135A_I2C_ADDR	0x18
#define SIL9135A_STD_ALL	(V4L2_DV_480P59_94 | V4L2_DV_576P50 |	\
				V4L2_DV_720P24 | V4L2_DV_720P25 |	\
				V4L2_DV_720P30 | V4L2_DV_720P50 |	\
				V4L2_DV_720P59_94 | V4L2_DV_720P60 |	\
				V4L2_DV_1080I29_97 | V4L2_DV_1080I30 |	\
				V4L2_DV_1080I25 | V4L2_DV_1080I50 |	\
				V4L2_DV_1080I60 | V4L2_DV_1080P24 |	\
				V4L2_DV_1080P25 | V4L2_DV_1080P30 |	\
				V4L2_DV_1080P50 | V4L2_DV_1080P60)
static char pb_empty_array[] __initdata = {};


/* Definitions specific to GMII plug board */
#define gmii_pb_rm_adevs		pb_empty_array
#define gmii_pb_rm_pdevs		pb_empty_array
#define gmii_pb_add_adevs		pb_empty_array
#define gmii_pb_add_pdevs		pb_empty_array
#define gmii_pb_rm_spi_devs		pb_empty_array
#define gmii_pb_add_spi_devs		pb_empty_array
#define gmii_pb_rm_i2c_devs		pb_empty_array
#define gmii_pb_add_i2c_devs		pb_empty_array

/* padmux devices to enable */
static struct pmx_dev *gmii_pb_pmx_devs[] = {
	&spear1340_pmx_gmii,
};

static void __init gmii_pb_init(void)
{
	struct plat_stmmacenet_data *phy_data =
		dev_get_platdata(&spear13xx_eth_device.dev);

	phy_data->interface = PHY_INTERFACE_MODE_GMII;
}


/* Definitions specific to RGMII plug board */
#define rgmii_pb_rm_adevs		pb_empty_array
#define rgmii_pb_rm_pdevs		pb_empty_array
#define rgmii_pb_add_adevs		pb_empty_array
#define rgmii_pb_add_pdevs		pb_empty_array
#define rgmii_pb_rm_spi_devs		pb_empty_array
#define rgmii_pb_add_spi_devs		pb_empty_array
#define rgmii_pb_rm_i2c_devs		pb_empty_array
#define rgmii_pb_add_i2c_devs		pb_empty_array

/* padmux devices to enable */
static struct pmx_dev *rgmii_pb_pmx_devs[] = {
	&spear1340_pmx_rgmii,
};

static void __init rgmii_pb_init(void)
{
	struct plat_stmmacenet_data *phy_data =
		dev_get_platdata(&spear13xx_eth_device.dev);

	phy_data->interface = PHY_INTERFACE_MODE_RGMII;
}


/* Definitions specific to ETM plug board */
#define etm_pb_pmx_devs			pb_empty_array
#define etm_pb_rm_adevs			pb_empty_array
#define etm_pb_rm_pdevs			pb_empty_array
#define etm_pb_add_adevs		pb_empty_array
#define etm_pb_add_pdevs		pb_empty_array
#define etm_pb_rm_spi_devs		pb_empty_array
#define etm_pb_add_spi_devs		pb_empty_array
#define etm_pb_rm_i2c_devs		pb_empty_array
#define etm_pb_add_i2c_devs		pb_empty_array
#define etm_pb_init			NULL


/* Definitions specific to HDMI RX plug board */
#define hdmi_rx_pb_rm_adevs		pb_empty_array
#define hdmi_rx_pb_rm_pdevs		pb_empty_array
#define hdmi_rx_pb_add_adevs		pb_empty_array
#define hdmi_rx_pb_rm_spi_devs		pb_empty_array
#define hdmi_rx_pb_add_spi_devs		pb_empty_array
#define hdmi_rx_pb_rm_i2c_devs		pb_empty_array
#define hdmi_rx_pb_add_i2c_devs		pb_empty_array

/* sil9135a hdmi rx chip related */

/*
 * As HDMI RX is a dummy subdev for completing the flow to test VideoIN IP
 * So, only single path is sufficient for the testing
 */
static struct v4l2_input sil9135a_inputs[] = {
	{
		.index = 0,
		.name = "VideoIN",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.std = SIL9135A_STD_ALL,
	}
};

/*
 * this is the route info for connecting each input of the SIL9135A
 * hdmi receiver to its output which eventually goes to vip.
 * There is a one to one correspondence with sil9135a_inputs.
 */
static struct vip_subdev_route sil9135a_routes[] = {
	{
		.input = SIL9135A_INPUT,
		.output = SIL9135A_OUTPUT,
	}
};

/* info regarding the various subdevs connected to VIP */
static struct vip_subdev_info vip_sdev_info[] = {
	/* SIL9135A hdmi receiver */
	{
		.name = "sil9135a",
		.grp_id = 0,
		.num_inputs = ARRAY_SIZE(sil9135a_inputs),
		.inputs = sil9135a_inputs,
		.routes = sil9135a_routes,
		.can_route = 1,
		.board_info = {
			I2C_BOARD_INFO("sil9135a", SIL9135A_I2C_ADDR),
			/*
			 * TODO: we can add some platform specific
			 * data for HDMI receiver chip here, if needed.
			 */
		},
	},
};

/*
 * some of the VIP features cannot be programmed via standard V4L2
 * ioctls, so we configure them here.
 */
static struct vip_config vip_config_info = {
	.vsync_pol = POL_ACTIVE_LOW,
	.hsync_pol = POL_ACTIVE_LOW,
	.rgb_width = THIRTYTWO_BIT,
	.vdo_mode = SINGLE_PORT,
	.pix_clk_pol = POL_ACTIVE_LOW,
};

/*
 * a lot of VIP subdev specific params can change with a change in the
 * EVB being used, so we need to be careful while populating these
 * details.
 */
static struct vip_plat_data vip_board_specific_data = {
	.card_name = "spear_vip",
	.config = &vip_config_info,
	.subdev_info = vip_sdev_info,
	.subdev_count = ARRAY_SIZE(vip_sdev_info),
	.i2c_adapter_id = 0,
	.is_field_end_gpio_based = 1,
	.gpio_for_frame_end_intr = PLGPIO_100, /* I2S_OUT_DATA_3 */
};

/* padmux devices to enable */
static struct pmx_dev *hdmi_rx_pb_pmx_devs[] = {
	&spear1340_pmx_vip_mux_cam0,
	&spear1340_pmx_vip_mux_cam1,
	&spear1340_pmx_vip_mux_cam2,
	&spear1340_pmx_vip_mux_cam3,
};

static struct platform_device *hdmi_rx_pb_add_pdevs[] __initdata = {
	&spear1340_vip_device,
};

static void __init hdmi_rx_pb_init(void)
{
	vip_set_vb_base(&vip_board_specific_data);

	/* set vip plat data */
	vip_set_plat_data(&spear1340_vip_device,
				&vip_board_specific_data);
}


/* Definitions specific to HDMI TX plug board */
#define hdmi_tx_pb_pmx_devs		pb_empty_array
#define hdmi_tx_pb_rm_adevs		pb_empty_array
#define hdmi_tx_pb_rm_pdevs		pb_empty_array
#define hdmi_tx_pb_add_adevs		pb_empty_array
#define hdmi_tx_pb_add_pdevs		pb_empty_array
#define hdmi_tx_pb_rm_spi_devs		pb_empty_array
#define hdmi_tx_pb_add_spi_devs		pb_empty_array
#define hdmi_tx_pb_rm_i2c_devs		pb_empty_array

static struct ad9889b_pdata ad9889b_platdata = {
	.irq_gpio = STMPE801_GPIO_7,
	.irq_type = IRQF_DISABLED | IRQF_SHARED | IRQF_TRIGGER_FALLING,
	.fb = 0,
	.ain = HDMI_AUDIO_IN_SPDIF,
};

static struct i2c_board_info spear1340_pb_i2c_board_hdmi_tx = {
		.type = "adi9889_i2c",
		.addr = 0x39,
		.platform_data = &ad9889b_platdata,
};

static struct i2c_dev_info spear1340_pb_i2c_hdmi_tx = {
	.board = &spear1340_pb_i2c_board_hdmi_tx,
	.busnum = 0,
};

/* I2C devices to be added */
static struct i2c_dev_info *hdmi_tx_pb_add_i2c_devs[] __initdata = {
	&spear1340_pb_i2c_hdmi_tx,
};

static void __init hdmi_tx_pb_init(void)
{
	struct ad9889b_pdata *ad9889b_pdata
		= spear1340_pb_i2c_board_hdmi_tx.platform_data;

	if (ad9889b_pdata->ain == HDMI_AUDIO_IN_I2S) {
		struct i2s_platform_data *i2s_pdata
			= dev_get_platdata(&spear1340_i2s_play_device.dev);

		i2s_pdata->snd_fmts = SNDRV_PCM_FMTBIT_S32_LE;
	}
}


/* Definitions specific to CAM plug board with single sensor mounted */
#define cam0_pb_pmx_devs		pb_empty_array
#define cam0_pb_rm_adevs		pb_empty_array
#define cam0_pb_rm_pdevs		pb_empty_array
#define cam0_pb_add_adevs		pb_empty_array
#define cam0_pb_add_pdevs		pb_empty_array
#define cam0_pb_rm_spi_devs		pb_empty_array
#define cam0_pb_add_spi_devs		pb_empty_array
#define cam0_pb_rm_i2c_devs		pb_empty_array
#define cam0_pb_add_i2c_devs		pb_empty_array
#define cam0_pb_init			NULL


/* Definitions specific to VGA plug board */
#define vga_pb_pmx_devs			pb_empty_array
#define vga_pb_rm_adevs			pb_empty_array
#define vga_pb_rm_pdevs			pb_empty_array
#define vga_pb_add_adevs		pb_empty_array
#define vga_pb_add_pdevs		pb_empty_array
#define vga_pb_rm_spi_devs		pb_empty_array
#define vga_pb_add_spi_devs		pb_empty_array
#define vga_pb_rm_i2c_devs		pb_empty_array
#define vga_pb_add_i2c_devs		pb_empty_array
#define vga_pb_init			NULL

/*
 * Definitions specific to SATA configuration
 * This is an exception as SATA is not a separate plug board but is a
 * change in normal evaulation board for supporting SATA.
 */
#define sata_pb_rm_adevs		pb_empty_array
#define sata_pb_add_adevs		pb_empty_array
#define sata_pb_rm_spi_devs		pb_empty_array
#define sata_pb_add_spi_devs		pb_empty_array
#define sata_pb_rm_i2c_devs		pb_empty_array
#define sata_pb_add_i2c_devs		pb_empty_array

/* padmux devices to enable */
static struct pmx_dev *sata_pb_pmx_devs[] = {
	&spear1340_pmx_sata,
};

static struct platform_device *sata_pb_rm_pdevs[] __initdata = {
	&spear13xx_pcie_host0_device,
};

static struct platform_device *sata_pb_add_pdevs[] __initdata = {
	&spear1340_sata0_device,
};

static void __init sata_pb_init(void)
{
	/* Miphy configuration for SATA */
	writel(SPEAR1340_PCIE_SATA_MIPHY_CFG_SATA_25M_CRYSTAL_CLK,
			VA_SPEAR1340_PCIE_MIPHY_CFG);
}

/*
 * Definitions specific to PCIe configuration
 * This is an exception as PCIe is not a separate plug board but is a
 * change in normal evaulation board for supporting PCIe.
 */
#define pcie_pb_rm_adevs		pb_empty_array
#define pcie_pb_add_adevs		pb_empty_array
#define pcie_pb_rm_spi_devs		pb_empty_array
#define pcie_pb_add_spi_devs		pb_empty_array
#define pcie_pb_rm_i2c_devs		pb_empty_array
#define pcie_pb_add_i2c_devs		pb_empty_array

/* padmux devices to enable */
static struct pmx_dev *pcie_pb_pmx_devs[] = {
	&spear1340_pmx_pcie,
};

static struct platform_device *pcie_pb_rm_pdevs[] __initdata = {
	&spear1340_sata0_device,
};

static struct platform_device *pcie_pb_add_pdevs[] __initdata = {
	&spear13xx_pcie_host0_device,
};

static void __init pcie_pb_init(void)
{
#ifdef CONFIG_SPEAR_PCIE_REV370
	spear1340_pcie_board_init(&spear13xx_pcie_host0_device.dev);
#endif
}

static int make_pb_list(struct list_head *pb_list)
{
	char *pb_name;
	struct plug_board *pb = NULL;

	while ((pb_name = get_spear_pb()) != NULL) {
		if (!strcmp(pb_name, "gmii")) {
			INIT_PB(gmii, pb);
		} else if (!strcmp(pb_name, "rgmii")) {
			INIT_PB(rgmii, pb);
		} else if (!strcmp(pb_name, "etm")) {
			INIT_PB(etm, pb);
		} else if (!strcmp(pb_name, "hdmi_rx")) {
			INIT_PB(hdmi_rx, pb);
		} else if (!strcmp(pb_name, "hdmi_tx")) {
			INIT_PB(hdmi_tx, pb);
		} else if (!strcmp(pb_name, "cam0")) {
			INIT_PB(cam0, pb);
		} else if (!strcmp(pb_name, "vga")) {
			INIT_PB(vga, pb);
		} else if (!strcmp(pb_name, "sata")) {
			INIT_PB(sata, pb);
		} else if (!strcmp(pb_name, "pcie")) {
			INIT_PB(pcie, pb);
		} else {
			pr_err("Invalid plug board requested: %s\n", pb_name);
			goto release_pb;
		}

		if (!pb)
			goto release_pb;

		list_add_tail(&pb->node, pb_list);
	}

	if (list_empty(pb_list)) {
		pr_err("Board list can't be empty\n");
			goto release_pb;
	}

	return 0;

release_pb:
	list_for_each_entry(pb, pb_list, node)
		kfree(pb);

	return -EINVAL;
}

int __init spear1340_pb_init(struct plug_board_info *pb_info)
{
	return spear_pb_init(pb_info, make_pb_list);
}
