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

#include <linux/init.h>
#include <linux/module.h>
#include <mach/cns3xxx.h>

#include "cns3xxx.h"
#include "cns3xxx_tool.h"
#include "cns3xxx_init_config.h"
#include "pse_init_common.h"

#ifdef CONFIG_CNS3XXX_GSW_VB
#include "vb.h"
#endif

int init_port = 7; /* bit map 7 means port 0, 1 and 2, default is 7. */
module_param(init_port, int, 0);

void take_off_vlan_header(struct sk_buff *skb)
{
	memmove(skb->data + 4, skb->data, 12);
	skb->data += 4;
	skb->len -= 4; /* minus 4 byte vlan tag */
}

int rx_port_base(
	struct sk_buff *skb, struct RXDesc *rx_desc_ptr,
	const struct CNS3XXXPrivate *priv)
{
	if (skb->data[12] == 0x81 && skb->data[13] == 0x00) {
		/*VLAN Header */
		printk(KERN_INFO "take_off_vlan_header\n");
		take_off_vlan_header(skb);
	}
	return 0;
}

int rx_vlan_base(
	struct sk_buff *skb, struct RXDesc *rx_desc_ptr,
	const struct CNS3XXXPrivate *priv)
{
	return 0;
}

int tx_port_base(
	struct TXDesc *tx_desc_ptr,
	const struct CNS3XXXPrivate *priv,
	struct sk_buff *skb)
{
#if defined(CNS3XXX_VLAN_8021Q) && defined(CNS3XXX_8021Q_HW_TX)
	if (skb && priv->vlgrp != NULL && vlan_tx_tag_present(skb)) {
		tx_desc_ptr->c_vid = cpu_to_le16(vlan_tx_tag_get(skb));
		tx_desc_ptr->ctv = 1;
		tx_desc_ptr->fr	= 0;
	} else
#endif
	{
		tx_desc_ptr->ctv = 0;
		tx_desc_ptr->pmap = priv->net_device_priv->pmap;
		tx_desc_ptr->fr	= 1;
	}
	return 0;
}

int tx_vlan_base(
	struct TXDesc *tx_desc_ptr,
	const struct CNS3XXXPrivate *priv,
	struct sk_buff *skb)
{
#if defined(CNS3XXX_VLAN_8021Q)
	if (skb && priv->vlgrp != NULL && vlan_tx_tag_present(skb))
		tx_desc_ptr->c_vid = cpu_to_le16(vlan_tx_tag_get(skb));
#else
	tx_desc_ptr->c_vid = priv->net_device_priv->vlan_tag;
#endif
	tx_desc_ptr->ctv = 1;
	tx_desc_ptr->fr	= 0;

	return 0;
}

/* port: 0, 1, 2 ; port0, port1 and port2
 * config general mac port configuration
 */
void cns3xxx_general_mac_cfg(u8 port)
{
	u32 cfg = 0;

	switch (port) {
	case 0:
		cfg = MAC0_CFG_REG;
		break;
	case 1:
		cfg = MAC1_CFG_REG;
		break;
	case 2:
		cfg = MAC2_CFG_REG;
		break;
	}

	cfg |= (1 << 13);	/* txc_check_en: 1  */
	cfg |= (1 << 17);	/* bp_en: 1 */

#ifdef CNS3XXX_LEARN_ENABLE
	cfg &= (~(1 << 19)); /* learn_dis: 0 */
#else
	cfg |= (1 << 19);	/* learn disable, learn_dis: 1*/
#endif
	cfg &= (~(1 << 20));	/* blocking_state: 0 */

	cfg &= (~(1 << 21));	/* block_mode: 0 */

#ifdef CNS3XXX_AGE_ENABLE
	cfg |= (1 << 22);	/* age_en: 1 */
#else
	cfg &= (~(1 << 22));	/* age disable,age_en: 1 */
#endif
	cfg &= (~(1 << 23)); /* SA_secured: 0 */
	switch (port) {
	case 0:
		MAC0_CFG_REG = cfg;
		break;
	case 1:
		MAC1_CFG_REG = cfg;
		break;
	case 2:
		MAC2_CFG_REG = cfg;
		break;
	}
}

void cns3xxx_configu_cpu_port(void)
{
/* Set CPU port to general configuration */

#ifdef CNS3XXX_LEARN_ENABLE
	CPU_CFG_REG &= (~(1 << 19)); /* learn_dis: 0 */
#else
	CPU_CFG_REG |= (1 << 19); /* learn_dis: 1 */
#endif

#ifdef CNS3XXX_AGE_ENABLE
	CPU_CFG_REG |= (1 << 22); /* age_en: 1 */
#else
	CPU_CFG_REG &= (~(1 << 22)); /* age disable age_en: 0 */
#endif
	CPU_CFG_REG &= (~(1 << 23)); /* SA_secured: 0 */
	CPU_CFG_REG |= (1 << 29); /* go to hnat:1 */
	CPU_CFG_REG &= (~(1 << 30)); /* offset 4N +2 */
#ifdef CNS3XXX_4N
	CPU_CFG_REG |= (1 << 30);
#endif
	CPU_CFG_REG &= (~(1 << 31)); /* cpu flow control disable */
#ifdef CNS3XXX_CPU_PORT_FC
	CPU_CFG_REG |= (1 << 31); /* cpu flow control enable */
#endif
}

void cns3xxx_gsw_hw_init(void)
{
	int i;
	u32 reg;

	reg = __raw_readl(MISC_GPIOB_PIN_ENABLE_REG);
	reg |= (1 << 14); /*enable GMII2_CRS*/
	reg |= (1 << 15); /*enable GMII2_COL*/
	reg |= (1 << 20); /*enable MDC*/
	reg |= (1 << 21); /*enable MDIO*/
	__raw_writel(reg, MISC_GPIOB_PIN_ENABLE_REG);

	cns3xxx_gsw_power_enable();
	cns3xxx_gsw_software_reset();

	/* RGMII0 high speed drive strength */
	reg = __raw_readl(MISC_IO_PAD_DRIVE_STRENGTH_CTRL_A);
	reg &= (~(3 << 2));
	reg |= (2 << 2);

	/* RGMII1 high speed drive strength */
	reg &= (~(3 << 6));
	reg |= (2 << 6);

	/* GMII2 high speed drive strength*/
	reg &= (~(3 << 10));
	reg |= (2 << 10);
	__raw_writel(reg, MISC_IO_PAD_DRIVE_STRENGTH_CTRL_A);

	/* RGMII0 no pull*/
	reg = __raw_readl(MISC_IO_PULL_CTRL_REG);
	reg &= (~(3 << 0));

	/* RGMII1 no pull*/
	reg &= (~(3 << 2));

	/* GMII2 no pull*/
	reg &= (~(3 << 4));
	__raw_writel(reg, MISC_IO_PULL_CTRL_REG);

	/* disable all ports auto polling */
	cns3xxx_phy_auto_polling_enable(0, 0);
	cns3xxx_phy_auto_polling_enable(1, 0);
	cns3xxx_phy_auto_polling_enable(2, 0);

	while (((SRAM_TEST_REG >> 20) & 1) == 0)
		;
	INTR_STAT_REG = 0xffffffff; /* write 1 for clear. */

	/* Set general value for MAC_GLOB_CFG_REG
	 * age_time: 2 ^(1-1) * 300 sec
	 */
	MAC_GLOB_CFG_REG &= (~0xf);
	MAC_GLOB_CFG_REG |= 1;

	/* bkoff_mode: 111 follow standard */
	MAC_GLOB_CFG_REG &= (~(0x7 << 9));
	MAC_GLOB_CFG_REG |= (0x7 << 9);

	/* jam_no: 1010: */
	MAC_GLOB_CFG_REG &= (~(0xf << 12));
	MAC_GLOB_CFG_REG |= (0xa << 12);

	/* bp_mode: 10: */
	MAC_GLOB_CFG_REG &= (~(0x3 << 16));
	MAC_GLOB_CFG_REG |= (0x2 << 16);

	/* res_mc_flt: 0 */
	MAC_GLOB_CFG_REG &= (~(0x1 << 28));

	/* col_mode: 11 */
	MAC_GLOB_CFG_REG &= (~(0x3 << 18));
	MAC_GLOB_CFG_REG |= (0x3 << 18);

	/*crc_stripping: 1*/
	MAC_GLOB_CFG_REG |= (0x1 << 20);


	/* ACCEPT_CRC_BAD_PKT : 0*/
	MAC_GLOB_CFG_REG &= (~(0x1 << 21));

#ifdef ACCEPT_CRC_BAD_PKT
	MAC_GLOB_CFG_REG |= (0x1 << 21);
#endif

	/* SVL */
	MAC_GLOB_CFG_REG &= (~(0x1 << 7));

#ifdef IVL
	/* IVL: 1 (IVL), 0 (SVL) */
	MAC_GLOB_CFG_REG |= (0x1 << 7);
#endif

	/* HNAT_en: 0 */
	MAC_GLOB_CFG_REG &= (~(0x1 << 26));

	/* Firewall_mode: 0 */
	MAC_GLOB_CFG_REG &= (~(0x1 << 27));

	cns3xxx_general_mac_cfg(0);
	cns3xxx_general_mac_cfg(1);
	cns3xxx_general_mac_cfg(2);
	cns3xxx_configu_cpu_port();

	/* write vlan table
	 * set cpu port vlan table
	 */
	cns3xxx_vlan_table_add(&cpu_vlan_table_entry);
	for (i = 0;
		i < sizeof(vlan_table_entry) / sizeof(struct VLANTableEntry);
		++i)
		cns3xxx_vlan_table_add(&vlan_table_entry[i]);

	cns3xxx_set_pvid(0, PORT0_PVID);
	cns3xxx_set_pvid(1, PORT1_PVID);
	cns3xxx_set_pvid(2, PORT2_PVID);
	cns3xxx_set_pvid(3, CPU_PVID);

#ifdef CNS3XXX_SET_ARL_TABLE
	/* set arl table */
	cns3xxx_arl_table_flush();
#endif
}

int cns3xxx_gsw_config_mac_port0(void)
{
	INIT_PORT0_PHY
	INIT_PORT0_MAC
	PORT0_LINK_DOWN
	return 0;
}

int cns3xxx_gsw_config_mac_port1(void)
{
	INIT_PORT1_PHY
	INIT_PORT1_MAC
	PORT1_LINK_DOWN
	return 0;
}

int cns3xxx_gsw_config_mac_port2(void)
{
	INIT_PORT2_PHY
	INIT_PORT2_MAC
	PORT2_LINK_DOWN
	return 0;
}

void cns3xxx_gsw_up_init(void)
{
	cns3xxx_gsw_hw_init();

	printk(KERN_INFO "CNS3XXX PSE: Initialize\n");

	if ((init_port & 1) == 1) {
		printk(KERN_INFO "MAC 0\n");
		cns3xxx_gsw_config_mac_port0();
	}

	if (((init_port >> 1) & 1) == 1) {
		printk(KERN_INFO "MAC 1\n");
		cns3xxx_gsw_config_mac_port1();
	}

	if (((init_port >> 2) & 1) == 1) {
		printk(KERN_INFO "MAC 2\n");
		cns3xxx_gsw_config_mac_port2();
	}
}
EXPORT_SYMBOL_GPL(cns3xxx_gsw_up_init);

static int __init cns3xxx_pse_init_init_module(void)
{
	return 0;
}

static void __exit cns3xxx_pse_init_exit_module(void)
{
	/* disable phy auto-poll */
	PHY_AUTO_ADDR_REG &= ~((1<<5) | (1<<13) | (1<<21));
	/* wait state machine idle	*/
	mdelay(1000);
}

MODULE_AUTHOR("Cavium Networks, <tech@XXXX.com>");
MODULE_DESCRIPTION("CNS3XXX PSE Init");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

module_init(cns3xxx_pse_init_init_module);
module_exit(cns3xxx_pse_init_exit_module);
