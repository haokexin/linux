// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <linux/of_gpio.h>
#include <media/media-entity.h>
#include <media/media-device.h>
#include <linux/kernel.h>

#include "maxim_deser_hub.h"
#include "camera_common_op.h"

#define MODULE_NAME "bst,maxim-deser-hub"
#define MAXIM_HUB_DEBUG
static unsigned long crossbar = 0xba9876543210;   /* default crossbar */


// Max96712 MFP register
static uint16_t max96712_mfp_ctrl_regs[] = {
	0x0300, 0x0303, 0x0306, 0x0309,
	0x030C, 0x0310, 0x0313, 0x0316,
	0x0319, 0x031C, 0x0320, 0x0323,
	0x0326, 0x0329, 0x032C, 0x0330,
	0x0333};
static uint16_t max96712_mfp_tx_id_a_regs[] = {
	0x0301, 0x0304, 0x0307, 0x030A,
	0x030D, 0x0311, 0x0314, 0x0317,
	0x031A, 0x031D, 0x0321, 0x0324,
	0x0327, 0x032A, 0x032D, 0x0331,
	0x0334};
static uint16_t max96712_mfp_tx_id_b_regs[] = {
	0x0337, 0x033A, 0x033D, 0x0341,
	0x0344, 0x0347, 0x034A, 0x034D,
	0x0351, 0x0354, 0x0357, 0x035A,
	0x035D, 0x0361, 0x0364, 0x0367,
	0x036A};
static uint16_t max96712_mfp_tx_id_c_regs[] = {
	0x036D, 0x0371, 0x0374, 0x0377,
	0x037A, 0x037D, 0x0381, 0x0384,
	0x0387, 0x038A, 0x038D, 0x0391,
	0x0394, 0x0397, 0x039A, 0x039D,
	0x03A1};
static uint16_t max96712_mfp_tx_id_d_regs[] = {
	0x03A4, 0x03A7, 0x03AA, 0x03AD,
	0x03B1, 0x03B4, 0x03B7, 0x03BA,
	0x03BD, 0x03C1, 0x03C4, 0x03C7,
	0x03CA, 0x03CD, 0x03D1, 0x03D4,
	0x03D7};
// Max96717、Max96717f、Max9295 GPIO register
static uint16_t max96717_mfp_ctrl_regs[] = {
	0x02BE, 0x02C1, 0x02C4, 0x02C7,
	0x02CA, 0x02CD, 0x02D0, 0x02D3,
	0x02D6, 0x02D9, 0x02DC};
static uint16_t max96717_mfp_rx_id_regs[] = {
	0x02C0, 0x02C3, 0x02C6, 0x02C9,
	0x02CC, 0x02CF, 0x02D2, 0x02D5,
	0x02D8, 0x02DB, 0x02DE};

int max96712_enter_csi_recover(struct deser_hub_dev *hub)
{
	v4l2_subdev_notify(&hub->subdev, MAXIM_DESER_LINK_HOTPLUG_START,
			   hub->chn[0].cam_dev);
	max96712_reg_write(hub, 0x08A0, 0x04);

	return 0;
}

int max96712_exit_csi_recover(struct deser_hub_dev *hub)
{
	v4l2_subdev_notify(&hub->subdev, MAXIM_DESER_LINK_HOTPLUG_STOP,
			   hub->chn[0].cam_dev);
	max96712_reg_write(hub, 0x08A0, 0x84);

	return 0;
}

static void max_open_deskew(struct maxim_hub_priv *priv)
{
	struct deser_hub_dev *hub = &priv->hub;
	//mipi0
	// Initial deskew width
	// [2:0]: (0b001) 2*32k UI
	max96712_reg_write(hub, 0x903, 0x87);
	max96712_reg_write(hub, 0x904, 0xA7);
	//mipi1
	max96712_reg_write(hub, 0x943, 0x87);
	max96712_reg_write(hub, 0x944, 0xA7);
	//mipi2
	max96712_reg_write(hub, 0x983, 0x87);
	max96712_reg_write(hub, 0x984, 0xA7);
	//mipi3
	max96712_reg_write(hub, 0x9c3, 0x87);
	max96712_reg_write(hub, 0x9c4, 0xA7);
}

static void max_close_deskew(struct maxim_hub_priv *priv)
{
	struct deser_hub_dev *hub = &priv->hub;
	//mipi0
	// Initial deskew width
	// [2:0]: (0b001) 2*32k UI
	max96712_reg_write(hub, 0x903, 0x01);
	max96712_reg_write(hub, 0x904, 0x11);
	//mipi1
	max96712_reg_write(hub, 0x943, 0x01);
	max96712_reg_write(hub, 0x944, 0x11);
	//mipi2
	max96712_reg_write(hub, 0x983, 0x01);
	max96712_reg_write(hub, 0x984, 0x11);
	//mipi3
	max96712_reg_write(hub, 0x9c3, 0x01);
	max96712_reg_write(hub, 0x9c4, 0x11);
}

static int set_max96712_csi_phy_speed(struct maxim_hub_priv *priv)
{
	struct deser_hub_dev *hub;
	int lane_speed;

	hub = &priv->hub;
	//Set mipi dphy lane speed [4:0]
	if (priv->lane_speed == 0) {
		dev_err(hub->dev, "lane speed is 0\n");
		return -1;
	}
	lane_speed =  0x20 | (priv->lane_speed / 100);

	if (priv->lane_speed >= 1600)
		max_open_deskew(priv);
	else
		max_close_deskew(priv);

	max96712_reg_write(hub, 0x415, lane_speed);
	max96712_reg_write(hub, 0x418, lane_speed);
	max96712_reg_write(hub, 0x41b, lane_speed);
	max96712_reg_write(hub, 0x41e, lane_speed);

	return 0;
}

static int set_max96712_csi_tx_gap(struct maxim_hub_priv *priv)
{
	struct deser_hub_dev *hub;

	hub = &priv->hub;

	/* Set this to combine different camera for Max96724,
	 * this only apply to DPHY mode
	 */
	if (priv->phy_mode_cfg == 1)
		return 0;

	max96712_reg_write(hub, 0x907, 0xFF);
	max96712_reg_write(hub, 0x947, 0xFF);
	max96712_reg_write(hub, 0x987, 0xFF);
	max96712_reg_write(hub, 0x9B7, 0xFF);

	return 0;
}

static int max_gmsl2_config(struct maxim_hub_priv *priv)
{
	struct deser_hub_dev *hub =  &priv->hub;
	int controller_value;
	int offset = 0;
	int i = 0;
	int evb_cmp;

	write_reg(hub->i2c_client, 0x40b, 0x00); //Disable MIPI CSI-2
	write_reg(hub->i2c_client, 0x6C2, 0x10); // Increase CMU voltage to for
						 // wide temperature range
	write_reg(hub->i2c_client, 0x14D1, 0x03); // VGAHiGain
	write_reg(hub->i2c_client, 0x15D1, 0x03); // VGAHiGain
	write_reg(hub->i2c_client, 0x16D1, 0x03); // VGAHiGain
	write_reg(hub->i2c_client, 0x17D1, 0x03); // VGAHiGain
	write_reg(hub->i2c_client, 0x06, 0xFF);	// Enable all 4 Links in
	//write_reg(hub->i2c_client, 0x06, 0xF0);
	// mdelay(10);
	// Video Pipe Selection
	write_reg(hub->i2c_client, 0xF0, 0x62); //40/62 // pipe X in link B to video pipe 1)pipe X in link A to video pipe 0
	write_reg(hub->i2c_client, 0xF1, 0xea); //ea/c8 // pipe X in link D to video pipe 3)pipe X in link C to video pipe 2
	//register16_write(hub,0xF2,0x51);	// pipe Y in link B to video pipe 4)pipe Y in link A to video pipe 5
	//register16_write(hub,0xF3,0xD9); // pipe Y in link D to video pipe 6)pipe Y in link C to video pipe 7
	write_reg(hub->i2c_client, 0xF4, 0x0F); // Turn on 4 pipes

	// Efficiency updates (disable HEARTBEAT Mode); for image data pipes 0-3
	write_reg(hub->i2c_client, 0x0106, 0x0A);
	write_reg(hub->i2c_client, 0x0118, 0x0A);
	write_reg(hub->i2c_client, 0x012A, 0x0A);
	write_reg(hub->i2c_client, 0x013C, 0x0A);
	//for modify csi_controller change camera stream to port a or port b
	if (priv->i2c_port == 0)
		controller_value = 0x15;
	else if (priv->i2c_port == 1)
		controller_value = 0xaa;
	for (i = 0; i < hub->max_port; i++) {
		offset = 0x40 * i;
		// YUV422 8bit);video pipe 0,1,2,3);map FS/FE
		write_reg(hub->i2c_client, 0x090B + offset, 0x07);
		write_reg(hub->i2c_client, 0x092D + offset, controller_value);// map to MIPI Controller 1
		write_reg(hub->i2c_client, 0x090D + offset, priv->data_type);
		write_reg(hub->i2c_client, 0x090E + offset, priv->data_type|(hub->chn[i].csi_vc << 6));// map to VC0-3
		write_reg(hub->i2c_client, 0x090F + offset, 0x00);
		write_reg(hub->i2c_client, 0x0910 + offset, hub->chn[i].csi_vc<<6);
		write_reg(hub->i2c_client, 0x0911 + offset, 0x01);
		write_reg(hub->i2c_client, 0x0912 + offset,
			  (hub->chn[i].csi_vc << 6) | 0x01);
	}
	// MIPI PHY Setting
	// Set Des in 2x4 mode
	write_reg(hub->i2c_client, 0x08A0, 0x04);
	// Set Lane Mapping for 4-lane port A
	write_reg(hub->i2c_client, 0x08A3, 0xE4);
	write_reg(hub->i2c_client, 0x08A4, 0xE4);
	// Set default 4 lane D-PHY or 3 lane C-PHY
	if (priv->phy_mode_cfg == CSI_CONFIG_CPHY) {
		if (hub->data_lanes_num == 1) {
			write_reg(hub->i2c_client, 0x090A, 0x20);
			write_reg(hub->i2c_client, 0x094A, 0x20);
			write_reg(hub->i2c_client, 0x098A, 0x20);
			write_reg(hub->i2c_client, 0x09CA, 0x20);
		} else if (hub->data_lanes_num == 2) {
			write_reg(hub->i2c_client, 0x090A, 0x60);
			write_reg(hub->i2c_client, 0x094A, 0x60);
			write_reg(hub->i2c_client, 0x098A, 0x60);
			write_reg(hub->i2c_client, 0x09CA, 0x60);
		} else {
			write_reg(hub->i2c_client, 0x090A, 0xa0);
			write_reg(hub->i2c_client, 0x094A, 0xa0);
			write_reg(hub->i2c_client, 0x098A, 0xa0);
			write_reg(hub->i2c_client, 0x09CA, 0xa0);
		}
	} else {
		if (hub->data_lanes_num == 2) {
			write_reg(hub->i2c_client, 0x090A, 0x40);
			write_reg(hub->i2c_client, 0x094A, 0x40);
			write_reg(hub->i2c_client, 0x098A, 0x40);
			write_reg(hub->i2c_client, 0x09CA, 0x40);
		} else {
			write_reg(hub->i2c_client, 0x090A, 0xC0);
			write_reg(hub->i2c_client, 0x094A, 0xC0);
			write_reg(hub->i2c_client, 0x098A, 0xC0);
			write_reg(hub->i2c_client, 0x09CA, 0xC0);
		}
	}
	// Turn on MIPI PHYs
	evb_cmp = strncmp(&hub->ctl_level[0], "evb", MAX_DTS_STRING_LEN);
	// different HW board may have different setting
	if (evb_cmp == 0) {
		// evb has special HW design
		write_reg(hub->i2c_client, 0x08A2, 0xF0);
	} else {
		// FAD case
		write_reg(hub->i2c_client, 0x08A2, 0xF4);
	}
	// Hold DPLL in reset (config_soft_rst_n = 0); before changing the rate
	write_reg(hub->i2c_client, 0x1C00, 0xF4);
	write_reg(hub->i2c_client, 0x1D00, 0xF4);
	write_reg(hub->i2c_client, 0x1E00, 0xF4);
	write_reg(hub->i2c_client, 0x1F00, 0xF4);
	// Set Data rate to be 1500Mbps/lane for port A and enable software override
	set_max96712_csi_phy_speed(priv);
	set_max96712_csi_tx_gap(priv);
	// Release reset to DPLL (config_soft_rst_n = 1);
	write_reg(hub->i2c_client, 0x1C00, 0xF5);
	write_reg(hub->i2c_client, 0x1D00, 0xF5);
	write_reg(hub->i2c_client, 0x1E00, 0xF5);
	write_reg(hub->i2c_client, 0x1F00, 0xF5);
	// PHY 2 copy PHY 0 output
	//write_reg(hub->i2c_client,0x08A9,0xC8);	//C0
	return 0;
}

static int max_channel_open(struct maxim_hub_priv *priv)
{
	struct deser_hub_dev *hub =  &priv->hub;
#ifdef MAX96712_DEBUG
	dev_err(hub->dev, "%s() line:%d\n", __func__, __LINE__);
#endif
	/*Enbale all link*/
	/*Reset one-shot*/
	max96712_reg_write(hub, 0x0018, 0x0F);
	mdelay(100);
	/*open GSML1 LINK A B C D*/
	max96712_reg_write(hub, 0x0006, 0xFF);
	mdelay(100);
	/*enable mipi output*/
	return 0;
}


static int modify_serdes_address(struct maxim_hub_priv *priv)
{
	int i;
	struct i2c_adapter *adap;
	struct deser_hub_dev *hub;
	uint8_t mask, value;

	hub = &priv->hub;
	adap = hub->i2c_client->adapter;
#ifdef MAXIM_HUB_DEBUG
	dev_info(hub->dev, "%s() %d\n", __func__, __LINE__);
#endif
	//disable all remote port
	max96712_reg_write(hub, 0x03, 0xff);
	mdelay(300);
	//modify serlias i2c address
	for (i = 0; i < hub->max_port; i++) {
		if (!hub->chn[i].camera_bound || (hub->chn[i].cam_dev == NULL)) {
			pr_info("%s() cam_dev [%d] is NULL, break\n", __func__, i);
			continue;
		}
		if (!is_gmsl2_video_connected(hub, i))
			continue;
		//0x03,disable remote control channel register
		switch (priv->i2c_port) {
		case 0:
			switch (i) {
			case 0:
				max96712_reg_write(hub, 0x03, 0xfe); //enable port0,open linka,close linkb,linkc,linkd
				break;
			case 1:
				max96712_reg_write(hub, 0x03, 0xfb); //enable port0,open linkb,close linka,linkc,linkd
				break;
			case 2:
				max96712_reg_write(hub, 0x03, 0xef); //enable port0,open linkc,close linka,linkb,linkd
				break;
			case 3:
				max96712_reg_write(hub, 0x03, 0xbf); //enable port0,open linkd,close linka,linkb,linkc
				break;
			default:
				dev_err(hub->dev, "%s() Not Support\n", __func__);
				break;
			}
			break;
		case 1:
			switch (i) {
			case 0:
				max96712_reg_write(hub, 0x03, 0xfd); //enable port1,open linka,close linkb,linkc,linkd
				break;
			case 1:
				max96712_reg_write(hub, 0x03, 0xf7); //enable port1,open linkb,close linka,linkc,linkd
				break;
			case 2:
				max96712_reg_write(hub, 0x03, 0xdf); //enable port1,open linkc,close linka,linkb,linkd
				break;
			case 3:
				max96712_reg_write(hub, 0x03, 0x7f); //enable port1,open linkd,close linka,linkb,linkc
				break;
			default:
				dev_err(hub->dev, "%s() Not Support\n", __func__);
				break;
			}
			break;
		case 2:
			pr_info("%s() %d, i2c_port2 not support", __func__, __LINE__);
			break;
		default:
			dev_err(hub->dev, "%s() Not Support such i2c_port\n", __func__);
			break;
		}
#ifdef MAXIM_HUB_DEBUG
		dev_info(hub->dev, "===== value = %x, mask = %x\n", value, mask);
		dev_info(hub->dev, "modify test ser_addr=0x%x,ser_alias=0x%x\n",
				priv->ser_addr[i], priv->ser_alias_addr[i]);
		dev_info(hub->dev, "modify test ser_alias_addr=0x%x,sensor_alias_addr=0x%x\n",
				priv->ser_alias_addr[i], priv->sensor_alias_addr[i]);
#endif
		/*  When the serializer is not power off during reboot,
		 *  it keep old alias address and settings, we should
		 *  reset it here.
		 *  If the serializer is initialized form power-off state,
		 *  this action does not take effect.
		 */
		write_register(adap, priv->ser_alias_addr[i], 0x10, 0x80);
		mdelay(100);

		write_register(adap, priv->ser_addr[i], 0x0000, (priv->ser_alias_addr[i] << 1));
		write_register(adap, priv->ser_alias_addr[i], 0x0042, (priv->sensor_alias_addr[i] << 1));
		write_register(adap, priv->ser_alias_addr[i], 0x0043, (priv->sensor_addr[i] << 1));

		//add eeprom alias addr to i2c bus
		write_register(adap, priv->ser_alias_addr[i], 0x0044, (priv->eeprom_alias_addr[i] << 1));
		write_register(adap, priv->ser_alias_addr[i], 0x0045, (priv->eeprom_addr[i] << 1));
	}
	if (priv->i2c_port == 0) {
		//enable linka,b,c,d of port0
		max96712_reg_write(hub, 0x03, 0xaa);
	} else if (priv->i2c_port == 1) {
		//enable linka,b,c,d of port1
		max96712_reg_write(hub, 0x03, 0x55);
	}
	mdelay(20);
	return 0;
}

static void maxim_hub_set_mipi_output(struct deser_hub_dev *hub, bool enable)
{
	uint8_t rd_value, wr_value;

	switch (hub->type) {
	case DESER_TYPE_MAX96712:
	case DESER_TYPE_MAX96722:
		max96712_reg_read(hub, 0x08A0, &rd_value);
		wr_value = rd_value;
		if (enable)
			wr_value |= (1 << 7);
		else 
			wr_value &= ~(1 << 7);
		max96712_reg_write(hub, 0x08A0, wr_value);
		dev_info(hub->dev, "%s: %d, reg[0x8a0]:0x%x->0x%x",
			__func__, enable, rd_value, wr_value);
		break;
	default:
		dev_err(hub->dev, "%s() Not Support\n", __func__);
		break;
	}
}

static int config_serial_fsync_gpio(struct maxim_hub_priv *priv, struct deser_trigger_info *trig_info)
{
	int i;
	struct i2c_adapter *adap;
	struct deser_hub_dev *hub;

	hub = &priv->hub;
	adap = hub->i2c_client->adapter;
	for (i = 0; i < hub->max_port; i++) {
		if (!is_gmsl2_video_connected(hub, i)) {
			// not found camera
			continue;
		}
		write_register(adap, priv->ser_alias_addr[i], max96717_mfp_rx_id_regs[trig_info->trigger_tx_gpio[i]], trig_info->trigger_rx_gpio); //modify
		write_register(adap, priv->ser_alias_addr[i], max96717_mfp_ctrl_regs[trig_info->trigger_tx_gpio[i]], EXT_TRIGGER_SER_TX_CFG);
	}

	return 0;
}

static int maxim_gmls2_internal_fsync_config(struct maxim_hub_priv *priv, struct deser_trigger_info *trig_info)
{
	struct deser_hub_dev *hub =  &priv->hub;
	int fsync_period;
	uint8_t gpio_id;

	/* Save this for hotplug */
	hub->trig_info.trigger_rx_gpio = trig_info->trigger_rx_gpio;
	max96712_reg_write(hub, 0x04AF, 0xd0);
	/*PCLK config*/
	/*MAXIM internal crystal oscillator is 25Mhz*/
	fsync_period = 25000000 / trig_info->trigger_fps;
	max96712_reg_write(hub, 0x04A7, fsync_period >> 16);
	max96712_reg_write(hub, 0x04A6, (fsync_period >> 8) & 0xff);
	max96712_reg_write(hub, 0x04A5, fsync_period & 0xff);
	/*gpio id used for transmitting fsync signal [7:3]*/
	gpio_id = 0;
	gpio_id = 0x00 | (trig_info->trigger_tx_gpio[0] << 3);
	max96712_reg_write(hub, 0x04b1, gpio_id);
	return 0;
}


static int maxim_gmls1_external_fsync_config(struct maxim_hub_priv *priv, struct deser_trigger_info *trig_info)
{
	struct deser_hub_dev *hub =  &priv->hub;
	uint8_t gpi_conf;
	/* Save this for hotplug */

	hub->trig_info.trigger_rx_gpio = trig_info->trigger_rx_gpio;
	max96712_reg_write(hub, 0x04A0, 0x0D);
	gpi_conf = 0x21 | (trig_info->trigger_rx_gpio << 6);
	max96712_reg_write(hub, 0x0B08, gpi_conf);
	max96712_reg_write(hub, 0x0C08, gpi_conf);
	max96712_reg_write(hub, 0x0D08, gpi_conf);
	max96712_reg_write(hub, 0x0E08, gpi_conf);

	return 0;
}

static int maxim_gmls1_internal_fsync_config(struct maxim_hub_priv *priv, struct deser_trigger_info *trig_info)
{
	struct deser_hub_dev *hub =  &priv->hub;
	int fsync_period;
	uint8_t gpio_id;
	uint8_t gpi_conf;

	/* Save this for hotplug */
	hub->trig_info.trigger_rx_gpio = trig_info->trigger_rx_gpio;
	max96712_reg_write(hub, 0x04AF, 0x4f);
	gpi_conf = 0x31 | (trig_info->trigger_rx_gpio << 6);
	max96712_reg_write(hub, 0x0B08, gpi_conf);
	max96712_reg_write(hub, 0x0C08, gpi_conf);
	max96712_reg_write(hub, 0x0D08, gpi_conf);
	max96712_reg_write(hub, 0x0E08, gpi_conf);
	/*PCLK config*/
	/*MAXIM internal crystal oscillator is 25Mhz*/
	fsync_period = 25000000 / trig_info->trigger_fps;
	max96712_reg_write(hub, 0x04A7, fsync_period >> 16);
	max96712_reg_write(hub, 0x04A6, (fsync_period >> 8) & 0xff);
	max96712_reg_write(hub, 0x04A5, fsync_period & 0xff);
	/*gpio id used for transmitting fsync signal [7:3]*/
	gpio_id = 0;
	gpio_id = 0x00 | (trig_info->trigger_tx_gpio[0] << 3);
	max96712_reg_write(hub, 0x04b1, gpio_id);
	return 0;
}


static int maxim_gmls2_external_fsync_config(struct maxim_hub_priv *priv, struct deser_trigger_info *trig_info)
{
	struct deser_hub_dev *hub =  &priv->hub;
	uint8_t value;

	/* Save this for hotplug */
	hub->trig_info.trigger_rx_gpio = trig_info->trigger_rx_gpio;
	value = 0x20|trig_info->trigger_rx_gpio;
	max96712_reg_write(hub, 0x04a0, 0x08);
	max96712_reg_write(hub, 0x04af, 0x9f);
	max96712_reg_write(hub, max96712_mfp_ctrl_regs[trig_info->trigger_rx_gpio], EXT_TRIGGER_DES_RX_CFG);
	max96712_reg_write(hub, max96712_mfp_tx_id_a_regs[trig_info->trigger_rx_gpio], value);
	max96712_reg_write(hub, max96712_mfp_tx_id_b_regs[trig_info->trigger_rx_gpio], value);
	max96712_reg_write(hub, max96712_mfp_tx_id_c_regs[trig_info->trigger_rx_gpio], value);
	max96712_reg_write(hub, max96712_mfp_tx_id_d_regs[trig_info->trigger_rx_gpio], value);
	//set serial fysnc gpio
	config_serial_fsync_gpio(priv, trig_info);
	return 0;
}
static int max96712_fsync_config(struct maxim_hub_priv *priv, struct deser_trigger_info *trig_info)
{
	struct deser_hub_dev *hub =  &priv->hub;

	if (trig_info->trigger_mode == DESER_TRIGGER_MODE_NONE || trig_info->trigger_tx_gpio[0] < 0 || trig_info->trigger_tx_gpio[1] < 0 || trig_info->trigger_tx_gpio[2] < 0 || trig_info->trigger_tx_gpio[3] < 0) {
		pr_err("===trigger info param error trigger_mode:%d trigger_tx_gpio[0]:%d trigger_tx_gpio[1]:%d trigger_tx_gpio[2]:%d trigger_tx_gpio[3]:%d\n",
				trig_info->trigger_mode, trig_info->trigger_tx_gpio[0], trig_info->trigger_tx_gpio[1], trig_info->trigger_tx_gpio[2], trig_info->trigger_tx_gpio[3]);
		return -1;
	}
	/*Set Manual mode*/
	max96712_reg_write(hub, 0x04A0, 0x04);
	/*Turn off auto master link selection*/
	max96712_reg_write(hub, 0x04A2, 0x00);
	/*Disable overlap window*/
	max96712_reg_write(hub, 0x04AA, 0x00);
	max96712_reg_write(hub, 0x04AB, 0x00);

	switch (trig_info->trigger_mode) {
	case DESER_TRIGGER_MODE_NONE:
		dev_info(hub->dev, "NONE TRIGGER MODE\n");
		//Setting default fsync mode
		max96712_reg_write(hub, 0x04A0, 0x0d);
		break;
	case DESER_TRIGGER_MODE_INTERNAL: //delete gms1
		if (priv->link_mode == MAXIM_LINK_MODE_GMSL2)
			maxim_gmls2_internal_fsync_config(priv, trig_info);
		if (priv->link_mode == MAXIM_LINK_MODE_GMSL1)
			maxim_gmls1_internal_fsync_config(priv, trig_info);
		break;
	case DESER_TRIGGER_MODE_EXTERNAL: //delete gmsl1
		if (priv->link_mode == MAXIM_LINK_MODE_GMSL2)
			maxim_gmls2_external_fsync_config(priv, trig_info);
		if (priv->link_mode == MAXIM_LINK_MODE_GMSL1)
			maxim_gmls1_external_fsync_config(priv, trig_info);
		break;
	default:
		pr_err("Wrong trigger mode!");
		return 1;
	}
	return 0;
}

static int config_max_fix_rate(struct maxim_hub_priv *priv)
{
	uint8_t value;
	struct deser_hub_dev *hub =  &priv->hub;
	//csi_out disable
	max96712_reg_write(hub, 0x040b, 0x00);
	max96712_reg_write(hub, 0x0010, 0x00);
	max96712_reg_write(hub, 0x0011, 0x00);
	//linka 3gpbs or 6gpbs
	if (priv->linkrx_rate[0] == 3)
		max96712_reg_write(hub, 0x0010, 0x01);
	else if (priv->linkrx_rate[0] == 6)
		max96712_reg_write(hub, 0x0010, 0x02);
	//linkb 3gpbs or 6gpbs
	if (priv->linkrx_rate[1] == 3) {
		max96712_reg_read(hub, 0x0010, &value);
		max96712_reg_write(hub, 0x0010, value|0x10);
	} else if (priv->linkrx_rate[1] == 6) {
		max96712_reg_read(hub, 0x0010, &value);
		max96712_reg_write(hub, 0x0010, value|0x20);
	}
	//linkc 3gpbs or 6gpbs
	if (priv->linkrx_rate[2] == 3)
		max96712_reg_write(hub, 0x0011, 0x01);
	else if (priv->linkrx_rate[2] == 6)
		max96712_reg_write(hub, 0x0011, 0x02);
	//linkd 3gpbs or 6gpbs
	if (priv->linkrx_rate[3] == 3) {
		max96712_reg_read(hub, 0x0011, &value);
		max96712_reg_write(hub, 0x0011, value|0x10);
	} else if (priv->linkrx_rate[3] == 6) {
		max96712_reg_read(hub, 0x0011, &value);
		max96712_reg_write(hub, 0x0011, value|0x20);
	}
	max96712_reg_write(hub, 0x0006, 0xff);
	max96712_reg_write(hub, 0x0018, 0x0f);
	return 0;
}

void max967XX_replicate_mode(struct deser_hub_dev *hub)
{
	struct maxim_hub_priv *priv =
		container_of(hub, struct maxim_hub_priv, hub);
	dev_info(hub->dev, "%s() line:%d\n", __func__, __LINE__);
	/* open 0~3 PHY
	 * max96712_reg_write(hub, 0x08A2, 0xF0);
	 */
	//copy port A to port B
	if (priv->i2c_port == 0)
		max96712_reg_write(hub, 0x8A9, 0xC8);
}

static void maxim_hub_open_link(struct deser_hub_dev *hub)
{
	int link_bit = 0;
	int i;
	struct maxim_hub_priv *priv =
		container_of(hub, struct maxim_hub_priv, hub);

	pr_err(" XXXXXXXX %s XXXXXXx\n", __func__);
	for (i = 0; i < MAX_CAMERAS_PER_SERDES; i++) {
		if (is_gmsl2_video_connected(hub, i))
			link_bit |= 1 << i;
	}
	pr_err(" XXXXXXXX %s link_bit = %d XXXXXXx\n", __func__, link_bit);
	if (priv->link_mode == MAXIM_LINK_MODE_GMSL1)
		max96712_reg_write(hub, 0x06, 0x0f);
	else
		max96712_reg_write(hub, 0x06, 0xf0 | link_bit);
}

static int max96712_gsml2_config(struct maxim_hub_priv *priv)
{
	struct deser_hub_dev *hub =  &priv->hub;

	if (priv->i2c_port == 1) {
		//enable port1 linka,linkb,linkc,linkd
		max96712_reg_write(hub, 0x0003, 0x55);
		//control channel port crossover selector,soc b find camera,enable linka,linkb,linkc,linkd
		max96712_reg_write(hub, 0x0007, 0xf0);
	}

	max_gmsl2_config(priv);

	// if (priv->i2c_port == 0)
	// max967XX_replicate_mode(hub);
	modify_serdes_address(priv);

	max_channel_open(priv);
	/*for write serdes register*/
	max96712_fsync_config(priv, &priv->hub.trig_info);
	//open enabled Link
	maxim_hub_open_link(hub);

	/* Link start-up take approximately 100ms maximum for any channel that
	 * meet the GMSL2 channel specification
	 */
	mdelay(100);

	return 0;
}

static int maxim_hub_s_stream(struct v4l2_subdev *subdev, int enable)
{
	int subdev_port, index;
	struct deser_hub_dev *hub;

	pr_info("%s() %d, set stream", __func__, __LINE__);
	hub = container_of(subdev, struct deser_hub_dev, subdev);
	subdev_port = (enable & MAXIM_STREAM_SUB_PORT_MASK) >> 4;
	index = subdev_port % hub->max_port;
	// is_enable = (enable & MAXIM_STREAM_ENABLE_MASK) == 1 ? 1 : 0;

	pr_info("%s() device index:[%d]\n", __func__, index);
	if (!hub->chn[index].camera_bound || (hub->chn[index].cam_dev == NULL)) {
		pr_err("%s() cam_dev [%d] is NULL\n", __func__, index);
		return 1;
	}
	mutex_lock(&hub->deser_mutex);

	/* enable CSI output*/
	if (hub->deser_stream_flag == false) {
		mdelay(5);
		maxim_hub_set_mipi_output(hub, true);
		hub->deser_stream_flag = true;
	}

	mutex_unlock(&hub->deser_mutex);
	return 0;
}

static int max96712_preinit(struct maxim_hub_priv *priv)
{
	struct deser_hub_dev *hub = &priv->hub;
#ifdef MAXIM_HUB_DEBUG
	dev_err(hub->dev, "%s() line:%d\n", __func__, __LINE__);
#endif
	/*Begin preset registers*/
	/*Reset Chip*/
	max96712_reg_write(hub, 0x0013, 0x75);
	mdelay(2);
	/*Disable mipi output*/
	max96712_reg_write(hub, 0x040B, 0x00);
	/*Disable mipi input*/
	max96712_reg_write(hub, 0x0006, 0x00);
	//one-shot Reset
	max96712_reg_write(hub, 0x0018, 0x0F);
	// /*Set i2c fast plus mode*/
	// max96712_reg_write(hub, 0x0640, 0x00);
	/*Turn on HIM*/
	max96712_reg_write(hub, 0x0B06, 0xEF);
	max96712_reg_write(hub, 0x0C06, 0xEF);
	max96712_reg_write(hub, 0x0D06, 0xEF);
	max96712_reg_write(hub, 0x0E06, 0xEF);
	/*Disable HS/VS processing*/
	max96712_reg_write(hub, 0x0B0F, 0x01);
	max96712_reg_write(hub, 0x0C0F, 0x01);
	max96712_reg_write(hub, 0x0D0F, 0x01);
	max96712_reg_write(hub, 0x0E0F, 0x01);
	/*Enable HS/VS processing*/
	max96712_reg_write(hub, 0x0B07, 0x84);
	max96712_reg_write(hub, 0x0C07, 0x84);
	max96712_reg_write(hub, 0x0D07, 0x84);
	max96712_reg_write(hub, 0x0E07, 0x84);
	mdelay(5);
	return 0;
}

static int max96712_gsm1_mipi_config(struct maxim_hub_priv *priv)
{
	struct deser_hub_dev *hub =  &priv->hub;
	char data_type;
	char bpp;
#ifdef MAXIM_HUB_DEBUG
	dev_err(hub->dev, "%s() line:%d\n", __func__, __LINE__);
#endif
	max96712_reg_write(hub, 0x041A, 0xF0);
	// mipi output set at 2*4
	max96712_reg_write(hub, 0x08A0, 0x04);
	// enable 2 output phy
	max96712_reg_write(hub, 0x08A2, 0x30);
	// enable 4 pipeline
	max96712_reg_write(hub, 0x00F4, 0x0F);
	//phy1 lane no. at 4
	max96712_reg_write(hub, 0x094A, 0xC0);
	//phy0/1 lane map #default:4E
	max96712_reg_write(hub, 0x08A3, 0xE4);
	//VC for pipe line 0/1 set as 0 and //all =0
	max96712_reg_write(hub, 0x040C, 0x00);
	//VC for pipe line 2/3 set as 0 and //all=0
	max96712_reg_write(hub, 0x040D, 0x00);
	/*
	 * Setting Datatype
	 */

	data_type = priv->data_type;
	max96712_reg_write(hub, 0x040E, ((data_type >> 4) << 6) | data_type);
	//dt for pipe line 1/2 set as YUV422
	max96712_reg_write(hub, 0x040F, ((data_type >> 2) << 4) | (data_type & 0xf));
	//dt for pipe line 2/3 set as YUV422
	max96712_reg_write(hub, 0x0410, (data_type << 2) | (data_type & 0x03));

	//BPP for pipe line 1/2 set as 1E(YUV422)
	if (priv->data_type == 0x1E)
		bpp = 0x8;
	else if (priv->data_type == 0x2B)
		bpp = 0xA;
	else if (priv->data_type == 0x2C)
		bpp = 0xC;
	else if (priv->data_type == 0x2D)
		bpp = 0xE;
	else if (priv->data_type == 0x2E)
		bpp = 0x10;

	max96712_reg_write(hub, 0x0411, ((bpp >> 2) << 5) | (bpp & 0x1f));
	max96712_reg_write(hub, 0x0412, (bpp << 2) | (bpp & 0x03));
	/*
	 * set CSI PHY Speed
	 */
	set_max96712_csi_phy_speed(priv);
	//following is for pipeline mapping. Needed for FCFS mode, and not needed for concatenate mode.
	max96712_reg_write(hub, 0x090B, 0x07);//enable 3 mappings for pipeline 0
	max96712_reg_write(hub, 0x092D, 0x15);// map to destination controller 1
	max96712_reg_write(hub, 0x090F, 0x00);//source dt(00,frame start) and vc(0)
	max96712_reg_write(hub, 0x0910, 0x00);//destination dt(00,frame start) and vc(0)
	max96712_reg_write(hub, 0x0911, 0x01);//source dt(01,frame end) and vc(0)
	max96712_reg_write(hub, 0x0912, 0x01);//destination dt(01,frame end) and vc(0)
	max96712_reg_write(hub, 0x094B, 0x07);//enable 3 mappings for pipeline 1
	max96712_reg_write(hub, 0x096D, 0x15);// CSI2 controller 1
	max96712_reg_write(hub, 0x094F, 0x00);//Source VC=0
	max96712_reg_write(hub, 0x0950, 0x40);//DES: VC=1
	max96712_reg_write(hub, 0x0951, 0x01);//Source VC=1
	max96712_reg_write(hub, 0x0952, 0x41);
	max96712_reg_write(hub, 0x098B, 0x07);//enable 3 mappings for pipeline 2
	max96712_reg_write(hub, 0x09AD, 0x15);// CSI2 controller 1
	max96712_reg_write(hub, 0x098F, 0x00);//SOURCE DT AND VC
	max96712_reg_write(hub, 0x0990, 0x80);//DES DT AND VC
	max96712_reg_write(hub, 0x0991, 0x01);//SOURCE DT AND VC
	max96712_reg_write(hub, 0x0992, 0x81);//DES DT AND VC
	max96712_reg_write(hub, 0x09CB, 0x07);//enable 3 mappings for pipeline 3
	max96712_reg_write(hub, 0x09ED, 0x15);// CSI2 controller 1
	max96712_reg_write(hub, 0x09CF, 0x00);//source dt and vc
	max96712_reg_write(hub, 0x09D0, 0xC0);//des dt and vc
	max96712_reg_write(hub, 0x09D1, 0x01);//source dt and vc
	max96712_reg_write(hub, 0x09D2, 0xC1);//des dt and vc

	max96712_reg_write(hub, 0x090D, priv->data_type);//source data type (1E) and VC(0)
	max96712_reg_write(hub, 0x090E, priv->data_type);//destination dt(1E) and vc(0)
	max96712_reg_write(hub, 0x094D, priv->data_type);//SRC vc=0
	max96712_reg_write(hub, 0x094E, (0x40 | priv->data_type));//DES: VC=1
	max96712_reg_write(hub, 0x098D, priv->data_type);//SRC vc=0
	max96712_reg_write(hub, 0x098E, (0x80 | priv->data_type));//DES DTVC=2
	max96712_reg_write(hub, 0x09CD, priv->data_type);//SRC vc=0
	max96712_reg_write(hub, 0x09CE, (0xC0 | priv->data_type));//des vc =3
	return 0;
}

static int max96701_config(struct maxim_hub_priv *priv)
{
	int i;
	struct deser_hub_dev *hub = &priv->hub;
#ifdef MAXIM_HUB_DEBUG
	dev_err(hub->dev, "%s() line:%d\n", __func__, __LINE__);
#endif
	for (i = 0; i < hub->max_port; i++) {
		//open Link
		max96712_reg_write(hub, 0x0006, (1 << i));
		mdelay(5);
		//set Link
		ser_write(hub, priv->ser_addr[i], 0x47, 0x2d);
		ser_write(hub, priv->ser_addr[i], 0x43, 0x25);
		ser_write(hub, priv->ser_addr[i], 0x67, 0xc4);
		ser_write(hub, priv->ser_addr[i], 0x0f, 0xbf);
		ser_write(hub, priv->ser_addr[i], 0x00,
			  (priv->ser_alias_addr[i] << 1));
		ser_write(hub, priv->ser_alias_addr[i], 0x01, priv->des_addr << 1);
		//ser_write(hub, priv->ser_addr[i], 0x0b, MAX96701_BROADCAST << 1);
		ser_write(hub, priv->ser_alias_addr[i], 0x0C, priv->ser_alias_addr[i] << 1);
		mdelay(10);
		ser_write(hub, priv->ser_alias_addr[i], 0x04, 0x83);
		mdelay(5);
	}

	return 0;
}

static int max96705_config(struct maxim_hub_priv *priv)
{
	int i;
	struct deser_hub_dev *hub = &priv->hub;
#ifdef MAXIM_HUB_DEBUG
	dev_err(hub->dev, "%s() line:%d\n", __func__, __LINE__);
#endif
	for (i = 0; i < hub->max_port; i++) {
		{
			//chose Link
			max96712_reg_write(hub, 0x0006, (1 << i));
			mdelay(5);

			if (ser_write(hub, priv->ser_addr[i], 0x00,
				      (priv->ser_alias_addr[i] << 1))) {
				continue;
			}
		}
	}
	return 0;
}

static int max_channel_mapping(struct maxim_hub_priv *priv)
{
#ifdef MAXIM_HUB_DEBUG
	dev_err(priv->hub.dev, "%s() line:%d\n", __func__, __LINE__);
#endif
	if (priv->serial_type == SER_TYPE_MAX96705)
		max96705_config(priv);
	else if (priv->serial_type == SER_TYPE_MAX96701)
		max96701_config(priv);

	return 0;
}

static int max_channel_link(struct maxim_hub_priv *priv, int enable)
{
	struct deser_hub_dev *hub =  &priv->hub;
	uint8_t is_enable;
#ifdef MAXIM_HUB_DEBUG
	dev_err(hub->dev, "%s() line:%d\n", __func__, __LINE__);
#endif
	if (enable)
		is_enable = 0xF;
	else
		is_enable = 0x0;
	/*Enable all link*/
	udelay(1 * 1000);
	/*open GSML1 LINK A B C D*/
	if (priv->link_mode == MAXIM_LINK_MODE_GMSL1)
		max96712_reg_write(hub, 0x0006, 0x0 | is_enable);
	else if (priv->link_mode == MAXIM_LINK_MODE_GMSL2)
		max96712_reg_write(hub, 0x0006, 0xF0 | is_enable);

	udelay(1 * 1000);
	max96712_reg_write(hub, 0x0018, 0x0F);
	return 0;
}

static int max96712_gsml1_config(struct maxim_hub_priv *priv)
{
	int ret;
#ifdef MAXIM_HUB_DEBUG
	dev_err(priv->hub.dev, "%s() line:%d\n", __func__, __LINE__);
#endif
	/*1.preinit: reset_register*/
	ret = max96712_preinit(priv);
	if (ret) {
		pr_err("%s(), line %d, max96712_preinit failed!\n", __func__,
		       __LINE__);
		return -EINVAL;
	}
	/*2.mipi config*/
	ret = max96712_gsm1_mipi_config(priv);
	if (ret) {
		pr_err("%s(), line %d, max96712_gsm1_mipi_config failed!\n",
		       __func__, __LINE__);
		return -EINVAL;
	}
	// max96712_reg_write(&(priv->hub), 0x0006, 0x0f);
	// max96712_reg_write(&(priv->hub), 0x0018, 0x0f);
	/*3.fsync config*/
	ret = max96712_fsync_config(priv, &priv->hub.trig_info);
	if (ret) {
		pr_err("%s(), line %d, max96712_fsync_config failed!\n",
		       __func__, __LINE__);
		return -EINVAL;
	}

	/*4. camera_mapping*/
	ret = max_channel_mapping(priv);
	if (ret) {
		pr_err("%s(), line %d, max_channel_mapping failed!\n", __func__,
		       __LINE__);
		return -EINVAL;
	}
	/*5 Disable all link & enable mipi output*/
	ret = max_channel_link(priv, 1);
	if (ret) {
		pr_err("%s(), line %d, max_channel_stream failed!\n", __func__,
		       __LINE__);
		return -EINVAL;
	}
	/*6. disable mipi output*/
	maxim_hub_set_mipi_output(&priv->hub, false);
	/*open camera link*/
	max96712_reg_write(&(priv->hub), 0x0006, 0x0f);
	return 0;
}

static void recover_link_ser_cfg(struct maxim_hub_priv *priv, int index)
{
	struct deser_hub_dev *hub;
	struct i2c_adapter *adap;
	int i;
	int ret;
	uint8_t val;
	uint8_t dis_rem_cc;

	hub = &priv->hub;
	adap = hub->i2c_client->adapter;

	/* Save current remote channel control setting */
	for (i = 0; i < MAXIM_DESER_DETECT_TIMES; i++) {
		ret = max96712_reg_read(hub, 0x0003, &val);
		if (ret == 0)
			break;

		mdelay(MAXIM_DETECT_DELAY_MS);
	}
	if (i >= MAXIM_DESER_DETECT_TIMES) {
		dev_err(hub->dev, "Failed to read remote controll status\n");
		return;
	}

	dis_rem_cc = ~(0b11 << (index * 2));
	/* Enable unique remote channel control */
	max96712_reg_write(hub, 0x0003, dis_rem_cc);

	write_register(adap, priv->ser_addr[index], 0x0042,
		       (priv->sensor_alias_addr[index] << 1));
	write_register(adap, priv->ser_addr[index], 0x0043,
		       (priv->sensor_addr[index] << 1));
	// add eeprom alias addr to i2c bus
	write_register(adap, priv->ser_addr[index], 0x0044,
		       (priv->eeprom_alias_addr[index] << 1));
	write_register(adap, priv->ser_addr[index], 0x0045,
		       (priv->eeprom_addr[index] << 1));

	write_register(adap, priv->ser_addr[index], 0x0000,
		       (priv->ser_alias_addr[index] << 1));

	max96712_reg_write(hub, 0x0018, (1 << index));

	/* Restore current remote channel control setting */
	max96712_reg_write(hub, 0x0003, val);
	usleep_range(100000, 200000);
}

/*
 *  Returns:
 *      LINK_STATUS_UNKNOWN: Error
 *      LINK_STATUS_UNLOCK: unlocked
 *      LINK_STATUS_LOCKED: locked
 */
static int get_link_lock_status(struct maxim_hub_priv *priv, int link_index)
{
	struct deser_hub_dev *hub;
	int ret;
	uint8_t link_lock;

	hub = &priv->hub;
	if (priv->link_mode == MAXIM_LINK_MODE_GMSL1) {
		/* TODO: support GMSL1 */
		return LINK_STATUS_UNKNOWN;
	}

	switch (link_index) {
	case 0:
		ret = max96712_reg_read(hub, 0x001A, &link_lock);
		break;
	case 1:
		ret = max96712_reg_read(hub, 0x000A, &link_lock);
		break;
	case 2:
		ret = max96712_reg_read(hub, 0x000B, &link_lock);
		break;
	case 3:
		ret = max96712_reg_read(hub, 0x000C, &link_lock);
		break;
	default:
		ret = -1;
	}

	if (ret)
		return LINK_STATUS_UNKNOWN;

	return (link_lock & 0x08) ? LINK_STATUS_LOCKED : LINK_STATUS_UNLOCK;
}

/*
 *  Returns:
 *      LINK_STATUS_UNKNOWN: Error
 *      LINK_STATUS_UNLOCK: unlocked
 *      LINK_STATUS_LOCKED: locked
 */
static int get_video_lock_status(struct maxim_hub_priv *priv, int link_index)
{
	struct deser_hub_dev *hub;
	int ret;
	uint8_t video_lock;

	hub = &priv->hub;
	if (priv->link_mode == MAXIM_LINK_MODE_GMSL1) {
		/* TODO: support GMSL1 */
		return LINK_STATUS_UNKNOWN;
	}

	switch (link_index) {
	case 0:
		ret = max96712_reg_read(hub, 0x01DC, &video_lock);
		break;
	case 1:
		ret = max96712_reg_read(hub, 0x01FC, &video_lock);
		break;
	case 2:
		ret = max96712_reg_read(hub, 0x021C, &video_lock);
		break;
	case 3:
		ret = max96712_reg_read(hub, 0x023C, &video_lock);
		break;
	default:
		ret = -1;
	}

	if (ret)
		return LINK_STATUS_UNKNOWN;

	return (video_lock & 0x01) ? LINK_STATUS_LOCKED : LINK_STATUS_UNLOCK;
}

static void link_recover_work(struct work_struct *work)
{
	struct maxim_hub_priv *priv;
	struct deser_hub_dev *hub;
	struct camera_dev *cam_dev;
	int i;
	int ret;
	uint8_t link_en;

	priv = (struct maxim_hub_priv *)container_of(
		work, struct maxim_hub_priv, link_recover_work.work);
	hub = &priv->hub;

	mutex_lock(&hub->deser_mutex);
	cam_dev = NULL;
	for (i = 0; i < ARRAY_SIZE(hub->chn); i++) {
		if (hub->chn[i].cam_dev != NULL) {
			cam_dev = hub->chn[i].cam_dev;
			break;
		}
	}

	if (!cam_dev) {
		mutex_unlock(&hub->deser_mutex);
		return;
	}

	v4l2_subdev_notify(&hub->subdev, MAXIM_DESER_LINK_HOTPLUG_START,
			   cam_dev);
	for (i = 0; i < MAXIM_DESER_DETECT_TIMES; i++) {
		ret = max96712_reg_read(hub, 0x06, &link_en);
		if (ret == 0)
			break;

		mdelay(MAXIM_DETECT_DELAY_MS);
	}

	if (i >= MAXIM_DESER_DETECT_TIMES) {
		dev_err(hub->dev, "Read link status timeout\n");
		goto HOTPLUG_STOP;
	}

	for (i = 0; i < MAX_CAMERAS_PER_SERDES; i++) {
		uint8_t link_lock;
		uint8_t video_lock;

		if (hub->chn[i].cam_dev == NULL) {
			dev_info(hub->dev, "hub->chn[%d].cam_dev is not enable\n", i);
			continue;
		}

		/* Skip disabled link */
		if (!((0x1 << i) & link_en))
			continue;

		link_lock = get_link_lock_status(priv, i);
		video_lock = get_video_lock_status(priv, i);
		dev_dbg(hub->dev,
			"link %d: link_lock: %d, video_lock: %d, map: %u\n", i,
			link_lock, video_lock, priv->link_unlock_map);

		if (link_lock == LINK_STATUS_UNKNOWN ||
		    video_lock == LINK_STATUS_UNKNOWN)
			continue;

		/*
		 *  Case 1: the camera is pluged out, link is unlock
		 *  Case 2: the camera is pluged out and pluged in soon,
		 *      the link is locked, but video is unlocked.
		 */
		if (video_lock == LINK_STATUS_UNLOCK) {
			/* video is not locked, disconnect at the first time */
			if (!((1 << i) & priv->link_unlock_map)) {
				dev_info(hub->dev, "%s: disconnected link %d\n",
					 __func__, i);
				v4l2_subdev_notify(&hub->subdev,
						   MAXIM_DESER_LINK_DISCONNECT,
						   hub->chn[i].cam_dev);
				priv->link_unlock_map |= (1 << i);
				hub->chn[i].cam_dev->power_on = false;
			}
		}

		if (link_lock == LINK_STATUS_LOCKED) {
			/* GMSL 2 link is locked, connect procedure */
			if (((1 << i) & priv->link_unlock_map)) {
				dev_info(hub->dev, "%s: recover link %d\n",
					 __func__, i);
				if (!(strncmp(&hub->ctl_level[0], "fad-lis",
					      MAX_DTS_STRING_LEN) == 0))
					recover_link_ser_cfg(priv, i);
				v4l2_subdev_notify(&hub->subdev,
						   MAXIM_DESER_LINK_CONNECT,
						   hub->chn[i].cam_dev);
				mdelay(200);
				write_register(
					hub->i2c_client->adapter,
					priv->ser_alias_addr[i],
					max96717_mfp_rx_id_regs
						[priv->hub.trig_info
							 .trigger_tx_gpio[i]],
					hub->trig_info.trigger_rx_gpio);
				write_register(
					hub->i2c_client->adapter,
					priv->ser_alias_addr[i],
					max96717_mfp_ctrl_regs
						[priv->hub.trig_info
							 .trigger_tx_gpio[i]],
					EXT_TRIGGER_SER_TX_CFG);
				mdelay(200);

				priv->link_unlock_map &= ~(1 << i);
				hub->chn[i].cam_dev->power_on = true;
			}
		}
	}

HOTPLUG_STOP:
	v4l2_subdev_notify(&hub->subdev, MAXIM_DESER_LINK_HOTPLUG_STOP,
			   cam_dev);
	/*
	 *  If all links are recovered and there is lock irq
	 *  defined, enable IRQ, otherwise run work periodically
	 */
	if ((priv->link_unlock_map == 0) && (priv->lock_irq) &&
	    priv->lock_irq_disabled) {
		dev_info(hub->dev,
			 "All links are locked, re-enable IRQ again\n");
		priv->lock_irq_disabled = false;
		enable_irq(priv->lock_irq);
	} else {
		schedule_delayed_work(
			&priv->link_recover_work,
			priv->check_link_period ?
				msecs_to_jiffies(priv->check_link_period) :
				msecs_to_jiffies(CHECK_LINK_PERIOD));
	}
	mutex_unlock(&hub->deser_mutex);
}

static irqreturn_t maxim_lock_irq_handler(int irq, void *p)
{
	struct maxim_hub_priv *priv;

	priv = (struct maxim_hub_priv *)p;
	mutex_lock(&priv->hub.deser_mutex);
	if (!priv->lock_irq_disabled) {
		disable_irq_nosync(irq);
		priv->lock_irq_disabled = true;
	}
	mutex_unlock(&priv->hub.deser_mutex);
	schedule_delayed_work(&priv->link_recover_work, 0);

	return IRQ_HANDLED;
}

static int maxim_hub_s_power(struct v4l2_subdev *sd, int enable)
{
	int i;
	int ret;
	struct deser_hub_dev *hub =
		container_of(sd, struct deser_hub_dev, subdev);
	struct maxim_hub_priv *priv =
		container_of(hub, struct maxim_hub_priv, hub);
#ifdef MAXIM_HUB_DEBUG
	dev_info(hub->dev, "%s: %d", __func__, enable);
#endif

	if (!enable) {
		maxim_hub_set_mipi_output(hub, false);
		msleep(10);
		return 0;
	}

	if (strncmp(&hub->ctl_level[0], "fad-lis", MAX_DTS_STRING_LEN) != 0) {
		if (priv->link_mode == MAXIM_LINK_MODE_GMSL2) {
			dev_info(hub->dev, "%s() line:%d GMSL2\n", __func__,
				__LINE__);
			if (max96712_gsml2_config(priv)) {
				dev_info(
					priv->hub.dev,
					"%s(), line %d, max96712_gsml2_config failed!\n",
					__func__, __LINE__);
				return -EINVAL;
			}
		} else if (priv->link_mode == MAXIM_LINK_MODE_GMSL1) {
			dev_info(hub->dev, "%s() line:%d GMSL1\n", __func__,
				__LINE__);
			if (max96712_gsml1_config(priv)) {
				pr_info("%s(), line %d, max96712_gsml1_config failed!\n",
					__func__, __LINE__);
				return -EINVAL;
			}
		}
		/*FADB not detect camera status*/
		for (i = 0; i < hub->max_port; i++) {
			if (hub->chn[i].cam_dev == NULL)
				continue;
			ret = is_gmsl2_video_connected(hub, i);
			if (ret) {
				hub->chn[i].cam_dev->power_on = true;
			} else {
				hub->chn[i].cam_dev->power_on = false;
				priv->link_unlock_map |= (1 << i);
			}
		}
	}

	dev_info(priv->hub.dev, "%s(), line %d, max96712 s_power success!\n",
		 __func__, __LINE__);

	return 0;
}

static int maxim_hub_reset(struct v4l2_subdev *sd, u32 val)
{
	int ret;
	struct deser_hub_dev *hub;
	struct maxim_hub_priv *priv;

	hub = container_of(sd, struct deser_hub_dev, subdev);
	priv = container_of(hub, struct maxim_hub_priv, hub);

	pr_info("%s, %d\n", __func__, __LINE__);
	mutex_lock(&hub->deser_mutex);
	if (!hub->deser_boot_flag) {
		/* Register link recover related on first */
		ret = devm_request_threaded_irq(
			priv->hub.dev, priv->lock_irq, NULL,
			maxim_lock_irq_handler,
			IRQF_TRIGGER_FALLING | IRQF_SHARED | IRQF_ONESHOT,
			dev_name(priv->hub.dev), priv);
		if (ret) {
			dev_err(priv->hub.dev, "Failed to request irq %d\n",
				priv->lock_irq);
			if (priv->check_link_period) {
				dev_info(
					priv->hub.dev,
					"Lock IRQ does not work, check link periodically\n");
				schedule_delayed_work(
					&priv->link_recover_work,
					msecs_to_jiffies(
						priv->check_link_period));
			}
		} else if (priv->link_unlock_map) {
			schedule_delayed_work(
				&priv->link_recover_work,
				msecs_to_jiffies(priv->check_link_period));
		}
	}

	hub->deser_boot_flag = true;
	mutex_unlock(&hub->deser_mutex);

	return 0;
}

static const struct v4l2_subdev_video_ops maxim_deser_v4l2_video_ops = {
	.s_stream = maxim_hub_s_stream,
};

static const struct v4l2_subdev_core_ops maxim_deser_v4l2_core_ops = {
	.s_power = maxim_hub_s_power,
	.reset = maxim_hub_reset,
};

static const struct v4l2_subdev_ops maxim_deser_v4l2_ops = {
	.core = &maxim_deser_v4l2_core_ops,
	.video = &maxim_deser_v4l2_video_ops,
};

static int deser_notify_bound(struct v4l2_async_notifier *notifier,
			      struct v4l2_subdev *sd,
			      struct v4l2_async_subdev *asd)
{
	struct camera_dev *cam_dev;
	struct deser_channel *deser_chn;
	//uint8_t value;

	cam_dev = container_of(sd, struct camera_dev, subdev);
	deser_chn = container_of(asd, struct deser_channel, async_dev);

	cam_dev->sd_state = BST_SUBDEV_STATE_BOUND;
	cam_dev->deser_parent = deser_chn->deser_dev;
	cam_dev->index_in_serdes = deser_chn->index;
	deser_chn->cam_dev = cam_dev;
	deser_chn->camera_bound = true;

	pr_info("%s(),line %d channel[%d]", __func__, __LINE__, cam_dev->index_in_serdes);
	return 0;
}

static void deser_notify_unbind(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *subdev,
				struct v4l2_async_subdev *asd)
{
}

static const struct v4l2_async_notifier_operations deser_async_ops = {
	.bound = deser_notify_bound,
	.unbind = deser_notify_unbind,
};

static int parse_input_dt(struct deser_hub_dev *hub, struct device_node *node)
{
	int i;
	int ret;
	u32 value;

	for (i = 0; i < hub->max_port; i++) {
		struct device_node *port;
		struct device_node *remote;

		port = of_graph_get_port_by_id(node, i);
		if (!port) {
			dev_err(hub->dev, "%s:: input port%d not found\n ", __func__, i);
			continue;
		}

		remote = of_graph_get_remote_node(node, i, 0);
		if (!remote) {
			dev_err(hub->dev, "%s:: input device%d not found\n", __func__, i);
			continue;
		}

		ret = of_property_read_u32(port, "virtual-channel", &value);
		if (ret == 0)
			hub->chn[i].csi_vc = value;
		else
			hub->chn[i].csi_vc = i;

		hub->chn[i].index = i;
		hub->chn[i].camera_node = remote;
		hub->chn[i].camera_fwnode = of_fwnode_handle(remote);
		hub->num_cameras++;
		if (parse_camera_serdes(hub, remote, i))
			dev_err(hub->dev, "%s:: parse_camera_serdes [%d]\n", __func__, i);
	}

	return 0;
}

static int parse_output_dt(struct deser_hub_dev *hub, struct device_node *node)
{
	struct device_node *csi2 = of_get_child_by_name(node, "csi-link");

	if (!csi2) {
		dev_err(hub->dev, "csi-link not found\n");
		return -EINVAL;
	}

	hub->subdev.fwnode = of_fwnode_handle(csi2);

	return 0;
}

static int maxim_hub_parse_dt(struct i2c_client *client)
{
	struct maxim_hub_priv *priv = i2c_get_clientdata(client);
	struct device_node *np = client->dev.of_node;
	struct deser_hub_dev *hub = &priv->hub;
	const char *link_mode = NULL;
	const char *ctl_level = NULL;
	const char *trigger_mode = NULL;
	int i;
	int phy_mode = 0;
	u32 port_num;

	if (!np)
		return -EINVAL;

	if (of_property_read_s32(np, "reg", &priv->des_addr)) {
		dev_err(&client->dev, "Invalid DT reg property\n");
		return -EINVAL;
	}

	// for single soc, ctl_level string is NULL
	if (of_property_read_string(np, "ctl-mode", &ctl_level))
		dev_dbg(hub->dev, " Failed to find ctl-mode\n");
	else
		strscpy(&hub->ctl_level[0], ctl_level, MAX_DTS_STRING_LEN);

	if (strncmp(priv->hub.ctl_level, "fad-lis", 7) == 0)
		priv->hub.ctl_mode = FAD_LIS_MODE;
	else if (strncmp(priv->hub.ctl_level, "fad-ctl", 7) == 0)
		priv->hub.ctl_mode = FAD_CTL_MODE;

	if (!of_property_read_string(np, "maxim,link-mode", &link_mode)) {
		if (!strncmp(link_mode, "GMSL2", 5)) {
			dev_info(hub->dev, "%s() line:%d GMSL2\n", __func__, __LINE__);
			priv->link_mode = MAXIM_LINK_MODE_GMSL2;
		} else if (!strncmp(link_mode, "GMSL1", 5)) {
			dev_info(hub->dev, "%s() line:%d GMSL1\n", __func__, __LINE__);
			priv->link_mode = MAXIM_LINK_MODE_GMSL1;
		}
	} else {
		dev_err(&client->dev, "Invalid DT link-mode property\n");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "phy-mode", &phy_mode)) {
		dev_warn(&client->dev, "not find mipi phy-mode config\n");
		phy_mode = 0;
	}
	priv->phy_mode_cfg = phy_mode;

	if (of_property_read_s32(np, "lane-speed", &priv->lane_speed)) {
		dev_err(&client->dev, "Invalid DT lane-speed property\n");
		return -EINVAL;
	}

	if (of_property_read_s32(np, "csi2-port", &priv->csi2_port))
		priv->csi2_port = 0;

	if (of_property_read_s32(np, "i2c-port", &priv->i2c_port))
		priv->i2c_port = 0;

	if (of_property_read_u32_array(np, "maxim,linkrx-rate", priv->linkrx_rate, 4)) {
		dev_err(&client->dev, "Invalid DT maxim,linkrx-rate\n");
		return -EINVAL;
	}

	hub->type = DESER_TYPE_INVALID;
	if (!of_property_read_string(np, "type", &priv->deser_type)) {
		if (priv->deser_type != NULL) {
			strscpy(hub->name, priv->deser_type,
				MAX_DESER_NAME_LEN);
			if (strncmp(priv->deser_type, "max96722", 8) == 0) {
				hub->src_mask = 0x0f;
				hub->max_port = 4;
				hub->type = DESER_TYPE_MAX96712;
			} else if (strncmp(priv->deser_type, "max96712", 8) ==
				   0) {
				hub->src_mask = 0x0f;
				if (of_property_read_u32(np, "port-num",
							 &port_num) == 0)
					hub->max_port = port_num;
				else
					hub->max_port = 4;
				hub->type = DESER_TYPE_MAX96712;
			} else if (strncmp(priv->deser_type, "max96724", 8) ==
				   0) {
				hub->src_mask = 0x0f;
				if (of_property_read_u32(np, "port-num",
							 &port_num) == 0)
					hub->max_port = port_num;
				else
					hub->max_port = 4;
				hub->type = DESER_TYPE_MAX96712;
			}
		}
	}

	/*Optional field*/
	if (!of_property_read_u32(np, "lane-num",
				 &hub->data_lanes_num))
		dev_info(hub->dev, "lane-num = %d", hub->data_lanes_num);
	if (of_property_read_string(np, "trigger-mode", &trigger_mode)) {
		hub->trig_info.trigger_mode = DESER_TRIGGER_MODE_NONE;
	} else {
		if (strncmp(trigger_mode, "internal", 8) == 0) {
		//internal trigger
			hub->trig_info.trigger_mode = DESER_TRIGGER_MODE_INTERNAL;
		} else if (strncmp(trigger_mode, "external", 8) == 0) {
			hub->trig_info.trigger_mode = DESER_TRIGGER_MODE_EXTERNAL;
		} else if (strncmp(trigger_mode, "default", 7) == 0) {
			hub->trig_info.trigger_mode = DESER_TRIGGER_MODE_NONE;
		}
	}
	if (of_property_read_u32(np, "trigger-fps",
				 &hub->trig_info.trigger_fps))
		hub->trig_info.trigger_fps = 25;
	if (of_property_read_u32(np, "trigger-rx-pin",
				 &hub->trig_info.trigger_rx_gpio))
		hub->trig_info.trigger_rx_gpio = 0;
	if (of_property_read_u32(np, "maxim,hsync-invert", &priv->hsync))
		priv->hsync = 0;
	if (of_property_read_u32(np, "maxim,vsync-invert", &priv->vsync))
		priv->vsync = 0;
	if (of_property_read_u64(np, "maxim,crossbar", &priv->crossbar))
		priv->crossbar = crossbar;

	if (of_property_read_u32(np, "check-link-period",
				 &priv->check_link_period))
		priv->check_link_period = 0;

	/* parse crossbar setup */
	for (i = 0; i < 16; i++) {
		priv->cb[i] = priv->crossbar % 16;
		priv->crossbar /= 16;
	}

	if (parse_input_dt(hub, np)) {
		dev_err(hub->dev, ":parse input dt failed\n");
		return -1;
	}
	if (parse_output_dt(hub, np)) {
		dev_err(hub->dev, ":parse output dt failed\n");
		return -1;
	}
#ifdef MAXIM_HUB_DEBUG
	//TODO: dump_device_info
#endif
	return 0;
}

static int register_subdev(struct maxim_hub_priv *priv)
{
	int ret;
	struct deser_hub_dev *hub;
	struct v4l2_subdev *sd;

	hub = &priv->hub;

	sd = &hub->subdev;
	v4l2_subdev_init(sd, &maxim_deser_v4l2_ops);
	v4l2_set_subdevdata(sd, hub);

	sd->dev = hub->dev;
	// sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, sizeof(sd->name), "%s", dev_name(hub->dev));
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	priv->pads[MAXIM_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	priv->pads[MAXIM_SINK_LINK0].flags = MEDIA_PAD_FL_SINK;
	priv->pads[MAXIM_SINK_LINK1].flags = MEDIA_PAD_FL_SINK;
	priv->pads[MAXIM_SINK_LINK2].flags = MEDIA_PAD_FL_SINK;
	priv->pads[MAXIM_SINK_LINK3].flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&sd->entity, MAXIM_N_PADS, priv->pads);
	if (ret)
		return ret;

	ret = v4l2_async_register_subdev(sd);
	if (ret) {
		dev_err(hub->dev, ":register subdev failed\n");
		return -1;
	}
	// dev_err(&hub->subdev.devnode->dev,"Device name is %s", hub->subdev.devnode->dev.kobj.name);
	return 0;
}

static int register_subdev_notifier(struct maxim_hub_priv *priv)
{
	int ret;
	int i;
	int index;
	struct deser_hub_dev *hub = &priv->hub;

	if (!hub->num_cameras) {
		dev_err(hub->dev, "%s: :no input device found\n", __func__);
		return -1;
	}

	v4l2_async_nf_init(&hub->notifier);
	hub->notifier.ops = &deser_async_ops;
	index = 0;
	for (i = 0; i < hub->max_port; i++) {
		if (!hub->chn[i].camera_fwnode)
			continue;

		hub->chn[i].deser_dev = hub;
		hub->chn[i].async_dev.match_type = V4L2_ASYNC_MATCH_FWNODE;
		hub->chn[i].async_dev.match.fwnode = hub->chn[i].camera_fwnode;
		__v4l2_async_nf_add_subdev(&hub->notifier,
					       &(hub->chn[i].async_dev));
		index++;
	}

	ret = v4l2_async_subdev_nf_register(&hub->subdev, &hub->notifier);
	if (ret) {
		dev_err(hub->dev, "%s: :register subdev notifier failed\n",
			__func__);
		return -1;
	}

	return 0;
}

int maxim_deser_hub_set_internal_frame_sync(struct deser_hub_dev *deser_hub,
					    int trigger_gpio, int fps)
{
	struct maxim_hub_priv *priv =
		container_of(deser_hub, struct maxim_hub_priv, hub);
	struct deser_trigger_info *trig_info;

	trig_info = &priv->hub.trig_info;
	trig_info->trigger_mode = DESER_TRIGGER_MODE_INTERNAL;
	trig_info->trigger_fps = fps;
	trig_info->trigger_tx_gpio[0] = trigger_gpio;

	maxim_hub_set_mipi_output(deser_hub, false);
	switch (deser_hub->type) {
	case DESER_TYPE_MAX96712:
	case DESER_TYPE_MAX96722:
		max96712_fsync_config(priv, trig_info);
		break;
	default:
		dev_err(priv->hub.dev, "Device type not support for now\n");
	}
	maxim_hub_set_mipi_output(deser_hub, true);
	return 0;
}

int maxim_deser_hub_set_external_frame_sync(struct deser_hub_dev *deser_hub,
											int camera_trigger_gpio, int deser_trigger_gpio)
{
	int ret = 0;
	struct maxim_hub_priv *priv =
		container_of(deser_hub, struct maxim_hub_priv, hub);
	struct deser_trigger_info *trig_info;
//modify
	trig_info = &priv->hub.trig_info;
	trig_info->trigger_mode = DESER_TRIGGER_MODE_EXTERNAL;
	switch (deser_hub->type) {
	case DESER_TYPE_MAX96712:
	case DESER_TYPE_MAX96722:
		max96712_fsync_config(priv, trig_info);
		break;
	default:
		dev_err(priv->hub.dev, "Device type support for now\n");
	}
	if (ret) {
		pr_err("maxim_close_internal_fsync error\n");
		return -1;
	}
	return 0;
}

static int maxim_detect_deserial(struct maxim_hub_priv *priv)
{
	int ret;
	struct deser_hub_dev *hub;
	uint8_t value = 0;
	struct i2c_client *client;
	int i;

	hub = &priv->hub;
	client = hub->i2c_client;
	for (i = 0; i < MAXIM_DESER_DETECT_TIMES; i++) {
		ret = max96712_reg_read(hub, 0x00, &value);
		if (ret < 0) {
			// retry
			msleep(MAXIM_DETECT_DELAY_MS);
			continue;
		} else {
			if (client->addr == (value >> 1)) {
				// detect success
				break;
			}
			dev_err(priv->hub.dev, "max96712 have not found\n");
			return -1;
		}
	}

	if (i >= MAXIM_DESER_DETECT_TIMES) {
		dev_err(priv->hub.dev, "detect max96712 timeout\n");
		return -1;
	} else {
		return 0;
	}
}

static void maxim_vdd_regulator_enable(struct maxim_hub_priv *priv)
{
	struct deser_hub_dev *hub;
	uint8_t value;

	hub = &priv->hub;
	max96712_reg_write(hub, 0x0017, 0x14);
	max96712_reg_write(hub, 0x0019, 0x10);
	//readback
	max96712_reg_read(hub, 0x0017, &value);
	dev_info(priv->hub.dev, "read_back REG_ENABLE : 0x%x\n", value);
	max96712_reg_read(hub, 0x0019, &value);
	dev_info(priv->hub.dev, "read_back REG_MNL : 0x%x\n", value);
}

static int maxim_hub_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	int ret;
	struct maxim_hub_priv *priv;
	int pdb_gpio = -1;
	int lock_gpio = -1;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	i2c_set_clientdata(client, priv);
	priv->hub.i2c_client = client;
	priv->hub.dev = &client->dev;
	priv->hub.deser_boot_flag = false;
	priv->hub.deser_stream_flag = false;
	priv->hub.internal_trigger_sync =
		maxim_deser_hub_set_internal_frame_sync;
	priv->hub.external_trigger_sync =
		maxim_deser_hub_set_external_frame_sync;
	priv->hub.enter_csi_recover = max96712_enter_csi_recover;
	priv->hub.exit_csi_recover = max96712_exit_csi_recover;
	mutex_init(&priv->hub.deser_mutex);

	ret = maxim_hub_parse_dt(client);
	if (ret) {
		dev_err(priv->hub.dev, "%s: :parse dt failed\n", __func__);
		return -EINVAL;
	}

	//power up deserializer at first
	if (strncmp(priv->hub.ctl_level, "fad-lis", 7) != 0) {
		pdb_gpio = of_get_named_gpio(client->dev.of_node, "pdb-gpio", 0);
		if (gpio_is_valid(pdb_gpio)) {
			ret = devm_gpio_request(&client->dev, pdb_gpio,
						dev_name(&client->dev));
			if (ret) {
				dev_err(&client->dev,
					"failed to request gpio %d\n", pdb_gpio);
				return ret;
			}

			gpio_direction_output(pdb_gpio, 0);
			mdelay(5);
			gpio_direction_output(pdb_gpio, 1);
			mdelay(20);
		}
		/*
		 * Only when VDD 1.2V call this func
		 * FAD is 1.2V and EVB is not
		 */
		if (strncmp(priv->hub.ctl_level, "fad-ctl", 7) == 0)
			maxim_vdd_regulator_enable(priv);
	}

	INIT_DELAYED_WORK(&priv->link_recover_work, link_recover_work);

	lock_gpio = of_get_named_gpio(client->dev.of_node, "lock-gpio", 0);
	if (gpio_is_valid(lock_gpio)) {
		ret = devm_gpio_request(&client->dev, lock_gpio,
					dev_name(&client->dev));
		if (ret) {
			dev_err(&client->dev, "Failed to request lock pin %d\n",
				lock_gpio);
		} else {
			gpio_direction_input(lock_gpio);
			priv->lock_gpio = lock_gpio;
			priv->lock_irq = gpio_to_irq(lock_gpio);
			dev_info(&client->dev, "%s: lock irq is %d\n", __func__,
				 priv->lock_irq);
		}
	} else {
		dev_err(&client->dev, "%s: lock gpio %d is invalid\n", __func__,
			lock_gpio);
	}

	/*FADB don't detect deser status*/
	if (strncmp(priv->hub.ctl_level, "fad-lis", 7) != 0) {
		ret = maxim_detect_deserial(priv);
		if (ret) {
			dev_err(priv->hub.dev, "%s: not found max96712\n", __func__);
			return -EINVAL;
		}
	}

	ret = register_subdev(priv);
	if (ret) {
		dev_err(priv->hub.dev, "%s() line %d: register subdev failed\n",
			__func__, (int)__LINE__);
		return -EINVAL;
	}

	ret = register_subdev_notifier(priv);
	if (ret) {
		dev_err(priv->hub.dev, "%s: register subdev notifier failed\n",
			__func__);
		return -EINVAL;
	}

	if (strncmp(priv->hub.ctl_level, "fad-lis", 7) != 0) {
		config_max_fix_rate(priv);
		/*wait for start up GMSL Link*/
		mdelay(20);
		/*Disable mipi output*/
		max96712_reg_write(&priv->hub, 0x040B, 0x00);
		max96712_reg_write(&priv->hub, 0x08A0, 0x04);
		max96712_reg_write(&priv->hub, 0x0003, 0xff);
	}
	pr_info("maxim hub probe done\n");
	return 0;
}

static void maxim_hub_remove(struct i2c_client *client)
{
}

static const struct i2c_device_id maxim_deser_id[] = {
	{ MODULE_NAME, 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, maxim_deser_id);

static const struct of_device_id maxim_of_match[] = {
	{
		.compatible = MODULE_NAME,
	},
	{},
};
MODULE_DEVICE_TABLE(of, maxim_derser_hub);

static struct i2c_driver maxim_derser_hub = {
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(maxim_of_match),
	},
	.probe		= maxim_hub_probe,
	.remove		= maxim_hub_remove,
	.id_table	= maxim_deser_id,
};

module_i2c_driver(maxim_derser_hub);

MODULE_DESCRIPTION("Maxim deserializer driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("BST Ltd.");
