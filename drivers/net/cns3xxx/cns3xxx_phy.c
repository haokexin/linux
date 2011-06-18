/*******************************************************************************
 *
 * Copyright (c) 2009 Cavium Networks
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59
 * Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 * The full GNU General Public License is included in this distribution in the
 * file called LICENSE.
 *
 ******************************************************************************/

#include "cns3xxx_phy.h"
#include "cns3xxx_symbol.h"

#include <linux/gpio.h>

#include "cns3xxx_tool.h"
#include "switch_api.h" /* for CAVM_OK ... macro */
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/bootmem.h>
#include <linux/sched.h>
#include <linux/types.h>

ushort jumbo_frame;
module_param(jumbo_frame, ushort, S_IRUGO|S_IWUSR);
EXPORT_SYMBOL_GPL(jumbo_frame);

int cns3xxx_phy_reset(u8 phy_addr)
{
	u16 phy_data = 0;

	if (cns3xxx_read_phy(phy_addr, 0, &phy_data) != CAVM_OK)
		return CAVM_ERR;
	phy_data |= (0x1 << 15);
	if (cns3xxx_write_phy(phy_addr, 0, phy_data) != CAVM_OK)
		return CAVM_ERR;

	return CAVM_OK;
}

/* mac_port: 0, 1, 2 */
int cns3xxx_enable_mac_clock(u8 mac_port, u8 en)
{
	switch (mac_port) {
	case 0:
		(en == 1) ?
			(PHY_AUTO_ADDR_REG |= 1 << 7) :
			(PHY_AUTO_ADDR_REG &= (~(1 << 7)));
		break;
	case 1:
		(en == 1) ?
			(PHY_AUTO_ADDR_REG |= (1 << 15)) :
			(PHY_AUTO_ADDR_REG &= (~(1 << 15)));
		break;
	case 2:
		(en == 1) ?
			(PHY_AUTO_ADDR_REG |= (1 << 23)) :
			(PHY_AUTO_ADDR_REG &= (~(1 << 23)));
		break;
	}
	return CAVM_OK;
}

/* dis: 1 disable; dis: 0 enable*/
int cns3xxx_phy_auto_polling_enable(u8 port, u8 en)
{
	u8 phy_addr[] = {5, 13, 21};

	PHY_AUTO_ADDR_REG &= (~(1 << phy_addr[port]));
	if (en)
		PHY_AUTO_ADDR_REG |= (1 << phy_addr[port]);
	return CAVM_OK;
}

/* dis: 1 disable, 0 enable */
int cns3xxx_mdc_mdio_disable(u8 dis)
{
	PHY_CTRL_REG &= (~(1 << 7));
	if (dis)
		PHY_CTRL_REG |= (1 << 7);

	return CAVM_OK;
}

int cns3xxx_phy_auto_polling_conf(int mac_port, u8 phy_addr)
{
	if ((mac_port < 0) || (mac_port > 2)) {
		printk(KERN_ERR "err\n");
		return CAVM_ERR;
	}
	switch (mac_port) {
	case 0:
		PHY_AUTO_ADDR_REG &= (~0x1f);
		PHY_AUTO_ADDR_REG |= phy_addr;
		break;
	case 1:
		PHY_AUTO_ADDR_REG &= (~(0x1f << 8));
		PHY_AUTO_ADDR_REG |= (phy_addr << 8);
		break;
	case 2:
		PHY_AUTO_ADDR_REG &= (~(0x1f << 16));
		PHY_AUTO_ADDR_REG |= (phy_addr << 16);
		break;
	}
	cns3xxx_phy_auto_polling_enable(mac_port, 1);
	return CAVM_OK;
}

int cns3xxx_read_phy(u8 phy_addr, u8 phy_reg, u16 *read_data)
{
	int delay = 0;
	u32 volatile tmp = PHY_CTRL_REG;

	PHY_CTRL_REG |= (1 << 15); /* clear "command completed" bit */

	for (delay = 0; delay < 10; delay++)
		;

	tmp &= (~0x1f);
	tmp |= phy_addr;

	tmp &= (~(0x1f << 8));
	tmp |= (phy_reg << 8);

	tmp |= (1 << 14); /* read command */

	PHY_CTRL_REG = tmp;

	/* wait command complete */
	while (((PHY_CTRL_REG >> 15) & 1) == 0)
		;

	*read_data = (PHY_CTRL_REG >> 16);
	PHY_CTRL_REG |= (1 << 15); /* clear "command completed" bit */

	return CAVM_OK;
}

int cns3xxx_write_phy(u8 phy_addr, u8 phy_reg, u16 write_data)
{
	int delay = 0;
	u32 tmp = PHY_CTRL_REG;

	PHY_CTRL_REG |= (1 << 15); /* clear "command completed" bit */

	for (delay = 0; delay < 10; delay++)
		;

	tmp &= (~(0xffff << 16));
	tmp |= (write_data << 16);

	tmp &= (~0x1f);
	tmp |= phy_addr;

	tmp &= (~(0x1f << 8));
	tmp |= (phy_reg << 8);

	tmp |= (1 << 13); /* write command */

	PHY_CTRL_REG = tmp;

	/* wait command complete */
	while (((PHY_CTRL_REG >> 15) & 1) == 0)
		;

	return CAVM_OK;
}

/* port 0,1,2 */
void cns3xxx_rxc_dly(u8 port, u8 val)
{
	switch (port) {
	case 0:
		SLK_SKEW_CTRL_REG &= (~(0x3 << 4));
		SLK_SKEW_CTRL_REG |= (val << 4);
		break;
	case 1:
		SLK_SKEW_CTRL_REG &= (~(0x3 << 12));
		SLK_SKEW_CTRL_REG |= (val << 12);
		break;
	case 2:
		SLK_SKEW_CTRL_REG &= (~(0x3 << 20));
		SLK_SKEW_CTRL_REG |= (val << 20);
		break;
	}
}

/* port 0,1,2 */
void cns3xxx_txc_dly(u8 port, u8 val)
{
	switch (port) {
	case 0:
		SLK_SKEW_CTRL_REG &= (~(0x3 << 6));
		SLK_SKEW_CTRL_REG |= (val << 6);
		break;
	case 1:
		SLK_SKEW_CTRL_REG &= (~(0x3 << 14));
		SLK_SKEW_CTRL_REG |= (val << 14);
		break;
	case 2:
		SLK_SKEW_CTRL_REG &= (~(0x3 << 22));
		SLK_SKEW_CTRL_REG |= (val << 22);
		break;
	}
}

void cns3xxx_mac2_gtxd_dly(u8 val)
{
	SLK_SKEW_CTRL_REG &= (~(0x3 << 24));
	SLK_SKEW_CTRL_REG |= (val << 24);
}

/* port : 0 => port0 ; port : 1 => port1
 * y = 1 ; disable AN
 */
void disable_AN(int port, int y)
{
	u32 mac_port_config = 0;

	switch (port) {
	case 0:
		mac_port_config = MAC0_CFG_REG;
		break;
	case 1:
		mac_port_config = MAC1_CFG_REG;
		break;
	case 2:
		mac_port_config = MAC2_CFG_REG;
		break;
	}

	/* disable PHY's AN */
	if (y == 1)
		mac_port_config &= ~(0x1 << 7);

	/* enable PHY's AN */
	if (y == 0)
		mac_port_config |= (0x1 << 7);

	switch (port) {
	case 0:
		MAC0_CFG_REG = mac_port_config;
		break;
	case 1:
		MAC1_CFG_REG = mac_port_config;
		break;
	case 2:
		MAC2_CFG_REG = mac_port_config;
		break;
	}
}

int cns3xxx_std_phy_power_down(int phy_addr, int y)
{
	u16 phy_data = 0;
	/* power-down or up the PHY */
	cns3xxx_read_phy(phy_addr, 0, &phy_data);
	if (y == 1) /* down */
		phy_data |= (0x1 << 11);
	if (y == 0) /* up */
		phy_data &= (~(0x1 << 11));
	cns3xxx_write_phy(phy_addr, 0, phy_data);

	phy_data = 0;
	cns3xxx_read_phy(phy_addr, 0, &phy_data);

	return 0;
}

#if defined(CONFIG_CNS3XXX_GSW_VB)
extern u8 cns3xxx_spi_tx_rx(
	u8 tx_channel, u8 tx_eof, u32 tx_data, u32 *rx_data);

int cns3xxx_spi_tx_rx_n(
	u32 tx_data, u32 *rx_data, u32 tx_channel, u32 tx_eof_flag)
{
	return cns3xxx_spi_tx_rx(tx_channel, tx_eof_flag, tx_data, rx_data);
}

int bcm53115M_reg_read(int page, int offset, u8 *buf, int len)
{
	u32 ch = BCM53115_SPI_CHANNEL;
	u8 cmd_byte;
	u32	dumy_word;
	u32 spi_status;
	int i;

	/* Normal SPI Mode (Command Byte)
	 * Bit7 Bit6 Bit5 Bit4 Bit3 Bit2 Bit1 Bit0
	 * 0	1	1	Mode=0 CHIP_ID2	ID1	ID0(lsb) Rd/Wr(0/1)
	 */

	/* Normal Read Operation */
	/* 1. Issue a normal read command(0x60) to poll the SPIF bit in the
	 * SPI status register(0XFE) to determine the
	 * operation can start */
	do {
		cmd_byte = 0x60;
		cns3xxx_spi_tx_rx_n(cmd_byte, &dumy_word, ch, 0);
		cns3xxx_spi_tx_rx_n(0xFE, &dumy_word, ch, 0);
		cns3xxx_spi_tx_rx_n(0x00, &spi_status, ch, 1);
	} while ((spi_status >> ROBO_SPIF_BIT) & 1);

	/* 2. Issue a normal write command(0x61) to write
	 * the register page value into the SPI page register(0xFF)	*/
	cmd_byte = 0x61;
	cns3xxx_spi_tx_rx_n(cmd_byte, &dumy_word, ch, 0);
	cns3xxx_spi_tx_rx_n(0xFF, &dumy_word, ch, 0);
	cns3xxx_spi_tx_rx_n(page, &dumy_word, ch, 1);

	/* 3. Issue a normal read command(0x60) to setup the
	 * required RobiSwitch register address	 */
	cmd_byte = 0x60;
	cns3xxx_spi_tx_rx_n(cmd_byte, &dumy_word, ch, 0);
	cns3xxx_spi_tx_rx_n(offset, &dumy_word, ch, 0);
	cns3xxx_spi_tx_rx_n(0x00, &dumy_word, ch, 1);

	/* 4. Issue a normal read command(0x60) to poll the
	 * RACK bit in the SPI status register(0XFE) to determine
	 * the completion of read*/
	do {
		cmd_byte = 0x60;
		cns3xxx_spi_tx_rx_n(cmd_byte, &dumy_word, ch, 0);
		cns3xxx_spi_tx_rx_n(0xFE, &dumy_word, ch, 0);
		cns3xxx_spi_tx_rx_n(0x00, &spi_status, ch, 1);
	} while (((spi_status >> ROBO_RACK_BIT) & 1) == 0);

	/* 5. Issue a normal read command(0x60) to read the
	 * specific register's conternt placed in the SPI
	 * data I/O register(0xF0)	 */
	cmd_byte = 0x60;
	cns3xxx_spi_tx_rx_n(cmd_byte, &dumy_word, ch, 0);
	cns3xxx_spi_tx_rx_n(0xF0, &dumy_word, ch, 0);
	/* read content */
	for (i = 0; i < len; i++) {
		cns3xxx_spi_tx_rx_n(
			0x00, &dumy_word, ch, ((i == (len - 1)) ? 1 : 0));
		buf[i] = (u8)dumy_word;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(bcm53115M_reg_read);

int bcm53115M_reg_write(int page, int offset, u8 *buf, int len)
{
	u32 ch = BCM53115_SPI_CHANNEL;
	u8 cmd_byte;
	u32 dumy_word;
	u32 spi_status;
	int i;

	/* Normal SPI Mode (Command Byte)
	 * Bit7 Bit6 Bit5 Bit4 Bit3 Bit2 Bit1 Bit0
	 * 0	1	1	Mode=0 CHIP_ID2	ID1	ID0(lsb) Rd/Wr(0/1)
	 */

	/* Normal Write Operation */
	/* 1. Issue a normal read command(0x60) to poll the SPIF bit in the
	 * SPI status register(0XFE) to determine the operation can start */
	do {
		cmd_byte = 0x60;
		cns3xxx_spi_tx_rx_n(cmd_byte, &dumy_word, ch, 0);
		cns3xxx_spi_tx_rx_n(0xFE, &dumy_word, ch, 0);
		cns3xxx_spi_tx_rx_n(0x00, &spi_status, ch, 1);
	} while ((spi_status >> ROBO_SPIF_BIT) & 1) ;

	/* 2. Issue a normal write command(0x61) to write
	 * the register page valueinto the SPI page register(0xFF)	*/
	cmd_byte = 0x61;
	cns3xxx_spi_tx_rx_n((u32)cmd_byte, &dumy_word, ch, 0);
	cns3xxx_spi_tx_rx_n(0xFF, &dumy_word, ch, 0);
	cns3xxx_spi_tx_rx_n(page, &dumy_word, ch, 1);

	/* 3. Issue a normal write command(0x61)
	 * and write the address of the accessed
	 * register followed by the write content
	 * starting from a lower byte */
	cmd_byte = 0x61;
	cns3xxx_spi_tx_rx_n((u32)cmd_byte, &dumy_word, ch, 0);
	cns3xxx_spi_tx_rx_n(offset, &dumy_word, ch, 0);
	/* write content */
	for (i = 0; i < len; i++)
		cns3xxx_spi_tx_rx_n((u32)buf[i],
			&dumy_word, ch, ((i == (len-1)) ? 1 : 0));

	return 0;
}
EXPORT_SYMBOL_GPL(bcm53115M_reg_write);

void bcm53115M_init_mac(u8 mac_port, u16 phy_addr)
{
	u32 mac_port_config = 0;
	u8 mac_addr[] = {0x0c, 0x10, 0x18};

	cns3xxx_enable_mac_clock(mac_port, 1);
	cns3xxx_phy_auto_polling_enable(mac_port, 0);

	mac_port_config = SWITCH_REG_VALUE(mac_addr[mac_port]);

	/* enable GMII, MII, reverse MII*/
	mac_port_config &= (~(1 << 15));

	/* enable RGMII*/
	mac_port_config |= (1 << 15);

	/* disable GIGA mode */
	mac_port_config &= (~(1<<16));

	/* enable GIGA mode*/
	mac_port_config |= (1<<16);

	/* disable PHY's AN */
	mac_port_config &= (~(0x1 << 7));

	/* force 1000Mbps*/
	mac_port_config &= (~(0x3 << 8));
	mac_port_config |= (0x2 << 8);

	/* force duplex*/
	mac_port_config |= (0x1 << 10);

	/* TX flow control on*/
	mac_port_config |= (0x1 << 12);

	/* RX flow control on*/
	mac_port_config |= (0x1 << 11);

	/* Turn off GSW_PORT_TX_CHECK_EN_BIT*/
	mac_port_config &= (~(0x1 << 13));

	/* Turn on GSW_PORT_TX_CHECK_EN_BIT*/
	mac_port_config |= (0x1 << 13);

	SWITCH_REG_VALUE(mac_addr[mac_port]) = mac_port_config;
}

struct bcm53115M_vlan_entry {
	u16 vid;
	u16 forward_map;
	u16 untag_map;
};

int bcm53115M_write_vlan(struct bcm53115M_vlan_entry *v)
{
	u8 bval;
	u16 wval;
	u32 dwval;

	/* fill vid*/
	wval = (u16)v->vid;
	bcm53115M_reg_write(0x05, 0x81, (u8 *)&wval, 2);

	/* fill table content*/
	dwval = 0;
	dwval |= (v->forward_map & 0x1FF);
	dwval |= ((v->untag_map & 0x1FF) << 9);
	bcm53115M_reg_write(0x05, 0x83, (u8 *)&wval, 4);

	/* write cmd */
	bval = VLAN_WRITE_CMD;
	bval |= (1 << VLAN_START_BIT);
	bcm53115M_reg_write(0x05, 0x80, (u8 *)&bval, 1);

	/* wait cmd complete */
	while (1) {
		bcm53115M_reg_read(0x05, 0x80, (u8 *)&bval, 1);
		if (((bval >> VLAN_START_BIT) & 1) == 0)
			break;
	}
	return CAVM_OK;
}

struct bcm_port_cfg {
	u8 link;
	u8 fdx;
	enum BCM_PORT_SPEED speed;
	u8 rx_flw_ctrl;
	u8 tx_flw_ctrl;
	u8 ow;
};

int bcm53115M_mac_port_config(int port, struct bcm_port_cfg *cfg)
{
	u8 bval = 0;
	int page, addr;

	if (cfg->link)
		bval |= (1<<0);
	if (cfg->fdx)
		bval |= (1<<1);
	bval |= ((cfg->speed & 0x3) << 2);
	if (cfg->rx_flw_ctrl)
		bval |= (1<<4);
	if (cfg->tx_flw_ctrl)
		bval |= (1<<5);

	if (port == BCM_PORT_IMP) {
		bval |= (1 << 7); /* Use content of this register*/
		page = 0x00;
		addr = 0x0E;
	} else {
		page = 0x00;
		addr = 0x58+port;
	}

	bcm53115M_reg_write(page, addr, &bval, 1);

	return 0;
}

int bcm53115M_init_internal_phy(void)
{
	int p, page;
	u16 wval;

	for (p = BCM_PORT_0; p <= BCM_PORT_4; p++) {
		page = 0x10 + p;

		/* reset phy*/
		bcm53115M_reg_read(page, 0x00, (u8 *)&wval, 2);
		wval |= 0x8000;
		bcm53115M_reg_write(page, 0x00, (u8 *)&wval, 2);

		/* config auto-nego & all advertisement*/
		bcm53115M_reg_read(page, 0x00, (u8 *)&wval, 2);
		wval |= (1<<12); /* auto-nego */
		bcm53115M_reg_write(page, 0x00, (u8 *)&wval, 2);

		bcm53115M_reg_read(page, 0x08, (u8 *)&wval, 2);
		wval |= 0x1E0; /* advertisement all*/
		bcm53115M_reg_write(page, 0x08, (u8 *)&wval, 2);

		/* 1000BASE-T*/
		bcm53115M_reg_read(page, 0x12, (u8 *)&wval, 2);
		wval &= ~(1<<12); /*automatic master/slave configuration*/
		wval |= 0x300; /*1000-base full/half advertisements*/
		bcm53115M_reg_write(page, 0x12, (u8 *)&wval, 2);
	}

	return 0;
}

int bcm53115M_led_init(void)
{
	u16 led_func, bval, wval;

	/* LED function 1G/ACT, 100M/ACT, 10M/ACT, not used */
	led_func = 0x2C00;
	bcm53115M_reg_write(0x00, 0x10, (u8 *)&led_func, 2);
	bcm53115M_reg_write(0x00, 0x12, (u8 *)&led_func, 2);

	/* LED map enable */
	wval = 0x1F; /* port0~4*/
	bcm53115M_reg_write(0x00, 0x16, (u8 *)&wval, 2);

	/* LED mode map */
	wval = 0x1F; /* led auto mode*/
	bcm53115M_reg_write(0x00, 0x18, (u8 *)&wval, 2);
	bcm53115M_reg_write(0x00, 0x1A, (u8 *)&wval, 2);

	/* LED enable */
	bcm53115M_reg_read(0x00, 0x0F, (u8 *)&bval, 1);
	bval |= 0x80;
	bcm53115M_reg_write(0x00, 0x0F, (u8 *)&bval, 1);

	return 0;
}

int bcm53115M_init(u8 mac_port, u16 phy_addr)
{
	u32 u32_val = 0;
	u16 u16_val = 0;
	u8 bval = 0;
	int i = 0;
	struct bcm53115M_vlan_entry v_ent;
	struct bcm_port_cfg pc;
	u8 page = 0, offset = 0;
	u8 gpio_pin = 50; /* for control bcm53115, gpio B pin 18*/

	printk(KERN_INFO "Initialize BCM53115M\n");

	memset(&v_ent, 0, sizeof(struct bcm53115M_vlan_entry));
	/* gpio B pin 18*/
	if (gpio_request(gpio_pin, "cns3xxx_gpio") == 0) {
		gpio_direction_output(gpio_pin, 0);
		gpio_free(gpio_pin);
	}

	bcm53115M_init_mac(0, 0);
	bcm53115M_init_mac(1, 1);
	/*read device id*/
	bcm53115M_reg_read(0x02, 0x30, (u8 *)&u32_val, 4);
	printk(KERN_INFO "bcm53115M device id:(0x%x)\r\n", u32_val);

	if (u32_val != 0x53115) {
		printk(KERN_INFO "bad device id(0x%x)\r\n", u32_val);
		return -1;
	}

	u16_val = 0;
	/* read phy id*/
	bcm53115M_reg_read(0x10, 0x04, (u8 *)&u16_val, 2);
	printk(KERN_INFO "bcm53115M phy id_1:(0x%x)\r\n", u16_val);

	if (u16_val != 0x143) {
		printk(KERN_INFO "bad phy id1(0x%x)\r\n", u16_val);
		return CAVM_ERR;
	}

	u16_val = 0;
	/* read phy id2*/
	bcm53115M_reg_read(0x10, 0x06, (u8 *)&u16_val, 2);
	printk(KERN_INFO "bcm53115M phy id_2:(0x%x)\r\n", u16_val);

	/* Loop detection disable */
	bcm53115M_reg_read(0x72, 0x00, (u8 *)&u16_val, 2);
	u16_val &= ~(0x3<<11);
	bcm53115M_reg_write(0x72, 0x00, (u8 *)&u16_val, 2);


	/* VLAN forwarding mask
	 * Bit8 IMP port, Bits[5:0] correspond to ports[5:0]
	 * port 0 <-> port IMP
	 */
	u16_val = 0x103;
	bcm53115M_reg_write(0x31, 0x0, (u8 *)&u16_val, 2); /* port 0*/
	u16_val = 0x103;
	bcm53115M_reg_write(0x31, 0x10, (u8 *)&u16_val, 2); /* IMP*/

	/* port 4 <-> port 5*/
	u16_val = 0x3c;
	bcm53115M_reg_write(0x31, 0x08, (u8 *)&u16_val, 2); /* port 4*/
	u16_val = 0x3c;
	bcm53115M_reg_write(0x31, 0x0A, (u8 *)&u16_val, 2); /* port 5*/

	/* others <-> none*/
	u16_val = 0x00;
	bcm53115M_reg_write(0x31, 0x02, (u8 *)&u16_val, 2); /* port 1*/
	bcm53115M_reg_write(0x31, 0x04, (u8 *)&u16_val, 2); /* port 2*/
	bcm53115M_reg_write(0x31, 0x06, (u8 *)&u16_val, 2); /* port 3*/

	/* port 1 <-> port IMP*/
	u16_val = 0x103;
	bcm53115M_reg_write(0x31, 0x2, (u8 *)&u16_val, 2); /* port 1*/

	/* port 2 <-> port 5*/
	u16_val = 0x3c;
	bcm53115M_reg_write(0x31, 0x4, (u8 *)&u16_val, 2); /* port 2*/

	/* port 3 <-> port 5*/
	u16_val = 0x3c;
	bcm53115M_reg_write(0x31, 0x6, (u8 *)&u16_val, 2); /* port 3*/

	/* Create VLAN1 for default port pvid */

	/* Unmanagement mode
	 * Switch Mode. Page 00h,Address 0Bh
	 */
	bval = 0x02; /* forward enable, unmanaged mode*/
	bcm53115M_reg_write(0x0, 0xb, &bval, 1);

	/* Init port5 & IMP  (test giga mode first)
	 * IMP port control. Page 00h,Address 08h
	 */
	bval = 0x1C; /* RX UCST/MCST/BCST enable*/
	bcm53115M_reg_write(0x0, 0x8, &bval, 1);

	offset = 0x5d; /* port 5*/
	bval = 0x7b;
	bcm53115M_reg_write(page, offset, (u8 *)&bval, 1);
	bcm53115M_reg_read(page, offset, (u8 *)&bval, 1);

	/* Speed, dulplex......etc
	 * setting in Gsw_Configure_Gsw_Hardware()
	 */

	/* Mgmt configuration, Page 02h, Address 00h*/
	bval = 0;
	bcm53115M_reg_write(0x02, 0x00, &bval, 1);
	/* BRCM header, Page 02h, Address 03h*/
	bval = 0; /* without additional header information*/
	bcm53115M_reg_write(0x02, 0x03, &bval, 1);

	/* Init front ports, port0-4 */
	/* MAC*/
	pc.speed = BCM_PORT_1G;
	pc.link = 0; /* link detect by robo_port_update()*/
	pc.ow = 0;
	for (i = BCM_PORT_0; i <= BCM_PORT_4; i++)
		bcm53115M_mac_port_config(i, &pc);
	/* Internal Phy*/
	bcm53115M_init_internal_phy();

	/* Enable all port, STP_STATE=No spanning tree, TX/RX enable */
	/* Page 00h, Address 00h-05h*/
	bval = 0x0;
	for (i = 0; i <= 5; i++)
		bcm53115M_reg_write(0x0, i, &bval, 1);

	/* Disable broadcast storm control due to h/w strap pin BC_SUPP_EN
	 * Page 41h, Address 10h-13h, bit28&22
	 */

	/* for port 0 ~ 5*/
	for (i = 0; i <= 0x14; i += 4) {
		bcm53115M_reg_read(0x41, 0x10+i, (u8 *)&u32_val, 4);
		u32_val &= ~((1<<28) | (1<<22));
		bcm53115M_reg_write(0x41, 0x10+i, (u8 *)&u32_val, 4);
	}

	/* for IMP port*/
	bcm53115M_reg_read(0x41, 0x30, (u8 *)&u32_val, 4);
	u32_val &= ~((1<<28) | (1<<22));
	bcm53115M_reg_write(0x41, 0x30, (u8 *)&u32_val, 4);

	/* Misc */
	/* led*/
	bcm53115M_led_init();
	/* multicast fwd rule, Page 00h, Address 2Fh*/
	bval = 0;
	bcm53115M_reg_write(0x00, 0x2F, &bval, 1);

	/* enable IMF port flow control function.*/
	page = 0x00;
	offset = 0x0e;
	bval = 0;
	bcm53115M_reg_read(page, offset, (u8 *)&bval, 1);
	bval |= (1<<4);
	bval |= (1<<5);
	bcm53115M_reg_write(page, offset, (u8 *)&bval, 1);
	bcm53115M_reg_read(page, offset, (u8 *)&bval, 1);

	return CAVM_OK;
}
#endif

#define CNS3XXX_MAC2_IP1001_GIGA_MODE

void icp_ip1001_init_mac(u8 mac_port, u16 phy_addr)
{
	u32 mac_port_config = 0;
	u8 mac_addr[] = {0x0c, 0x10, 0x18};

	cns3xxx_enable_mac_clock(mac_port, 1);

	mac_port_config = SWITCH_REG_VALUE(mac_addr[mac_port]);

	/* enable GMII, MII, reverse MII*/
	mac_port_config &= (~(1 << 15));

#ifdef MAC2_RGMII
	mac_port_config |= (1 << 15);
#endif

	/* disable GIGA mode*/
	mac_port_config &= (~(1<<16));

#ifdef CNS3XXX_MAC2_IP1001_GIGA_MODE
	/* enable GIGA mode*/
	mac_port_config |= (1<<16);
#endif
	/* disable PHY's AN*/
	mac_port_config &= (~(0x1 << 7));

	/* enable PHY's AN*/
	mac_port_config |= (0x1 << 7);
	SWITCH_REG_VALUE(mac_addr[mac_port]) = mac_port_config;
}

#define PROBE_PHY_ID 0x0243

int icp_ip1001_init(u8 mac_port, u8 phy_addr)
{
	u16 phy_data = 0;

	printk(KERN_INFO "Initialize ICPLUS IP1001\n");

	cns3xxx_mdc_mdio_disable(0);

	phy_data = get_phy_id(phy_addr); /* should be 0x243*/

	if (phy_data != PROBE_PHY_ID) {
		printk(KERN_ERR "wrong phy id: %x!! Should be %x\n",
			phy_data, PROBE_PHY_ID);
		return CAVM_ERR;
	}

	cns3xxx_phy_reset(phy_addr);

	icp_ip1001_init_mac(mac_port, phy_addr);

	/* read advertisement register*/
	cns3xxx_read_phy(phy_addr, 0x4, &phy_data);

	/* enable PAUSE frame capability*/
	phy_data |= (0x1 << 10);

	phy_data &= (~(0x1 << 5));
	phy_data &= (~(0x1 << 6));
	phy_data &= (~(0x1 << 7));
	phy_data &= (~(0x1 << 8));

	phy_data |= (0x1 << 5);
	phy_data |= (0x1 << 6);
	phy_data |= (0x1 << 7);
	phy_data |= (0x1 << 8);

	cns3xxx_write_phy(phy_addr, 0x4, phy_data);

	cns3xxx_read_phy(phy_addr, 9, &phy_data);

	phy_data &= (~(1<<8)); /* remove advertise 1000 half duples*/
	phy_data &= (~(1<<9)); /* remove advertise 1000 full duples*/

#ifdef CNS3XXX_MAC2_IP1001_GIGA_MODE
	phy_data |= (1<<9); /* add advertise 1000 full duples*/
#endif
	cns3xxx_write_phy(phy_addr, 9, phy_data);
	cns3xxx_read_phy(phy_addr, 9, &phy_data);
	cns3xxx_read_phy(phy_addr, 0, &phy_data);

	/* AN enable*/
	phy_data |= (0x1 << 12);
	cns3xxx_write_phy(phy_addr, 0, phy_data);
	cns3xxx_read_phy(phy_addr, 0, &phy_data);
	/* restart AN*/
	phy_data |= (0x1 << 9);
	cns3xxx_write_phy(phy_addr, 0, phy_data);

	/* If mac port AN turns on, auto polling needs to turn on.*/
	cns3xxx_phy_auto_polling_conf(mac_port, phy_addr);
	return 0;
}

#define PHY_CONTROL_REG_ADDR 0x00
#define PHY_AN_ADVERTISEMENT_REG_ADDR 0x04

int icp_101a_init_mac(u8 port, u8 phy_addr)
{
	u32 mac_port_config = 0;
	cns3xxx_enable_mac_clock(port, 1);
	switch (port) {
	case 0:
		mac_port_config = MAC0_CFG_REG;
		break;
	case 1:
		mac_port_config = MAC1_CFG_REG;
		break;
	case 2:
		mac_port_config = MAC2_CFG_REG;
		break;
	}

	/* enable GMII, MII, reverse MII*/
	mac_port_config &= (~(1 << 15));

	/* disable PHY's AN, use force mode*/
	mac_port_config &= (~(0x1 << 7));

	/* enable PHY's AN*/
	mac_port_config |= (0x1 << 7);
	/* If mac port AN turns on, auto polling needs to turn on.*/
	cns3xxx_phy_auto_polling_conf(port, phy_addr);

	/* normal MII*/
	mac_port_config &= (~(1 << 14));

	switch (port) {
	case 0:
		MAC0_CFG_REG = mac_port_config;
		break;
	case 1:
		MAC1_CFG_REG = mac_port_config;
		break;
	case 2:
		MAC2_CFG_REG = mac_port_config;
		break;
	}
	return CAVM_OK;
}

int icp_101a_init(u8 mac_port, u8 phy_addr)
{
	u32 mac_port_config = 0;
	u16 phy_data = 0;

	cns3xxx_mdc_mdio_disable(0);
	cns3xxx_phy_reset(phy_addr);

	phy_data = get_phy_id(mac_port);
	if (phy_data != 0x0243) {
		printk(KERN_ERR "ICPLUS 101A phy id should be 0x243,"
			"but the phy id is : %x\n", phy_data);
		return CAVM_ERR;
	}
	printk(KERN_INFO "phy id : %x\n", phy_data);
	printk(KERN_INFO "init IC+101A\n");

	icp_101a_init_mac(mac_port, phy_addr);

	/* read advertisement register*/
	cns3xxx_read_phy(phy_addr, 0x4, &phy_data);

	/* enable PAUSE frame capability*/
	phy_data |= (0x1 << 10);

	cns3xxx_write_phy(phy_addr, 0x4, phy_data);

	switch (mac_port) {
	case 0:
		mac_port_config = MAC0_CFG_REG;
		break;
	case 1:
		mac_port_config = MAC1_CFG_REG;
		break;
	case 2:
		mac_port_config = MAC2_CFG_REG;
		break;
	}

	cns3xxx_read_phy(phy_addr, 0, &phy_data);
	/* an enable*/
	phy_data |= (0x1 << 12);

	/* restart AN*/
	phy_data |= (0x1 << 9);
	cns3xxx_write_phy(phy_addr, 0, phy_data);

	while (1) {
		cns3xxx_read_phy(phy_data, 0, &phy_data);

		if (phy_data & (0x1 << 9))
			continue;
		else
			break;
	}

	return CAVM_OK;
}

u16 get_phy_id(u8 phy_addr)
{
	u16 read_data;

	cns3xxx_read_phy(phy_addr, 2, &read_data);

	return read_data;
}

u32 get_vsc8601_recv_err_counter(u8 phy_addr)
{
	u16 read_data = 0;
	cns3xxx_read_phy(phy_addr, 19, &read_data);
	return read_data;
}

u32 get_crc_good_counter(u8 phy_addr)
{
	u16 read_data = 0;
	/* enter extended register mode*/
	cns3xxx_write_phy(phy_addr, 31, 0x0001);
	cns3xxx_read_phy(phy_addr, 18, &read_data);

	/* back to normal register mode*/
	cns3xxx_write_phy(phy_addr, 31, 0x0000);
	return read_data;
}
