/*******************************************************************************
 *
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

#ifndef CNS3XXX_TOOL_H
#define CNS3XXX_TOOL_H

#define PRINT_INFO printk

#include "cns3xxx.h"
#include <linux/kernel.h>

#define SHOW_DEBUG_MESSAGE
#ifdef SHOW_DEBUG_MESSAGE

extern int MSG_LEVEL;

#define NO_MSG 0
#define NORMAL_MSG 1
#define WARNING_MSG (1 << 1)
#define CRITICAL_MSG (1 << 2)
#define DUMP_RX_PKT_INFO (1 << 3)
#define DUMP_TX_PKT_INFO (1 << 4)

#define DEBUG_MSG(msg_level, fmt, args...)\
{\
	int i = 0;\
	for (i = 0 ; i < 3 ; ++i) { \
		if ((MSG_LEVEL & msg_level) >> i) \
			printk(KERN_INFO "*cns3xxx gsw debug* " fmt, ## args); \
	} \
}

#endif

#define GET_MAC_PORT_CFG(port, cfg) \
{ \
	switch (port) { \
	case MAC_PORT0: \
		cfg = MAC0_CFG_REG; \
		break; \
	case MAC_PORT1: \
		cfg = MAC1_CFG_REG; \
		break; \
	case MAC_PORT2: \
		cfg = MAC2_CFG_REG; \
		break; \
	} \
}

#define SET_MAC_PORT_CFG(port, cfg) \
{ \
	switch (port) { \
	case MAC_PORT0: \
		MAC0_CFG_REG = cfg; \
		break; \
	case MAC_PORT1: \
		MAC1_CFG_REG = cfg; \
		break; \
	case MAC_PORT2: \
		MAC2_CFG_REG = cfg; \
		break; \
	} \
}

#define between(x, start, end) ((x) >= (start) && (x) <= (end))
static inline void print_packet(unsigned char *data, int len)
{
	int i, j;

	printk(KERN_INFO "packet length: %d%s:\n",
		len, len > 128 ? "(only show the first 128 bytes)" : "");

	for (i = 0; len; ) {
		printk(KERN_INFO "\t");
		if (len >= 16) {
			for (j = 0; j < 16; j++)
				printk("%02x ", data[i++]);
			printk("| ");

			i -= 16;
			for (j = 0; j < 16; j++) {
				if (between(data[i], 0x21, 0x7e))
					printk("%c", data[i++]);
				else {
					printk(".");
					i++;
				}
			}
			len -= 16;
		} else {
			/* last line */
			for (j = 0; j < len; j++)
				printk("%02x ", data[i++]);

			for (; j < 16; j++)
				printk("   ");

			printk("| ");

			i -= len;
			for (j = 0; j < len; j++) {
				if (between(data[i], 0x21, 0x7e))
					printk("%c", data[i++]);
				else {
					printk(".");
					i++;
				}
			}
			len = 0;
		}
		printk("\n");
    }
    return;
}

static inline void cns3xxx_gsw_power_enable(void)
{
	u32 reg = __raw_readl(PM_PLL_HM_PD_CTRL_REG);

	reg &= (~(1 << 2)); /* power up PLL_RGMII (for MAC)*/
	__raw_writel(reg, PM_PLL_HM_PD_CTRL_REG);

	reg = __raw_readl(PM_CLK_GATE_REG);
	reg |= (1 << 11); /* enable switch clock*/
	__raw_writel(reg, PM_CLK_GATE_REG);
}

static inline void cns3xxx_gsw_software_reset(void)
{
	u32 reg = __raw_readl(PM_SOFT_RST_REG);
	reg &= (~(1 << 11));
	__raw_writel(reg, PM_SOFT_RST_REG);

	reg = __raw_readl(PM_SOFT_RST_REG);
	reg |= (1 << 11);
	__raw_writel(reg, PM_SOFT_RST_REG);
}

/* port:
 * 0 : mac port0
 * 1 : mac port1
 * 2 : mac port2
 * 3 : cpu port
 */
static inline void enable_port(u8 port, u8 enable)
{
	switch (port) {
	case 0:
		(enable == 1) ?
			(MAC0_CFG_REG &= (~(1 << 18))) :
			(MAC0_CFG_REG |= (1 << 18)) ;
		break;
	case 1:
		(enable == 1) ?
			(MAC1_CFG_REG &= (~(1 << 18))) :
			(MAC1_CFG_REG |= (1 << 18)) ;
		break;
	case 2:
		(enable == 1) ?
			(MAC2_CFG_REG &= (~(1 << 18))) :
			(MAC2_CFG_REG |= (1 << 18)) ;
		break;
	case 3:
		(enable == 1) ?
			(CPU_CFG_REG &= (~(1 << 18))) :
			(CPU_CFG_REG |= (1 << 18)) ;
		break;
	}
}

static inline int cns3xxx_vlan_table_lookup(struct VLANTableEntry *entry)
{
	VLAN_CTRL2_REG |= entry->vid;
	ARL_VLAN_CMD_REG |= (1 << 8); /* look up vlan table command*/

	/* wait for vlan command complete*/
	while (((ARL_VLAN_CMD_REG >> 9) & 1) == 0)
		;

	if (!((ARL_VLAN_CMD_REG >> 10) & 1))
		return CAVM_NOT_FOUND;

	entry->valid = ((VLAN_CTRL0_REG >> 31) & 0x1);
	entry->vid = ((VLAN_CTRL2_REG >> 31) & 0xfff);
	entry->wan_side = ((VLAN_CTRL0_REG >> 30) & 0x1);
	entry->etag_pmap = ((VLAN_CTRL0_REG >> 25) & 0x1f);
	entry->mb_pmap = ((VLAN_CTRL0_REG >> 9) & 0x1f);

	entry->my_mac[0] = ((VLAN_CTRL1_REG >> 24) & 0xff);
	entry->my_mac[1] = ((VLAN_CTRL1_REG >> 16) & 0xff);
	entry->my_mac[2] = ((VLAN_CTRL1_REG >> 8) & 0xff);
	entry->my_mac[3] = (VLAN_CTRL1_REG & 0xff);

	entry->my_mac[4] = ((VLAN_CTRL2_REG >> 24) & 0xff);
	entry->my_mac[5] = ((VLAN_CTRL2_REG >> 16) & 0xff);

	return CAVM_FOUND;
}

static inline int cns3xxx_vlan_table_read(struct VLANTableEntry *entry)
{
	ARL_VLAN_CMD_REG &= (~0x3f);
	ARL_VLAN_CMD_REG |= (entry->vlan_index);
	ARL_VLAN_CMD_REG |= (1 << 7); /* read vlan table command*/

	/* wait for vlan command complete*/
	while (((ARL_VLAN_CMD_REG >> 9) & 1) == 0)
		;

	entry->valid = ((VLAN_CTRL0_REG >> 31) & 0x1);
	entry->vid = ((VLAN_CTRL2_REG) & 0xfff);
	entry->wan_side = ((VLAN_CTRL0_REG >> 30) & 0x1);
	entry->etag_pmap = ((VLAN_CTRL0_REG >> 25) & 0x1f);
	entry->mb_pmap = ((VLAN_CTRL0_REG >> 9) & 0x1f);

	entry->my_mac[0] = ((VLAN_CTRL1_REG >> 24) & 0xff);
	entry->my_mac[1] = ((VLAN_CTRL1_REG >> 16) & 0xff);
	entry->my_mac[2] = ((VLAN_CTRL1_REG >> 8) & 0xff);
	entry->my_mac[3] = (VLAN_CTRL1_REG & 0xff);

	entry->my_mac[4] = ((VLAN_CTRL2_REG >> 24) & 0xff);
	entry->my_mac[5] = ((VLAN_CTRL2_REG >> 16) & 0xff);

	return CAVM_OK;
}

/* add a entry in the vlan table */
static inline int cns3xxx_vlan_table_add(struct VLANTableEntry *entry)
{
	VLAN_CTRL0_REG = 0;
	VLAN_CTRL1_REG = 0;
	VLAN_CTRL2_REG = 0;

	VLAN_CTRL0_REG |= (entry->valid << 31);
	VLAN_CTRL0_REG |= (entry->wan_side << 30);
	VLAN_CTRL0_REG |= (entry->etag_pmap << 25);
	VLAN_CTRL0_REG |= (entry->mb_pmap << 9);

	VLAN_CTRL1_REG |= (entry->my_mac[0] << 24);
	VLAN_CTRL1_REG |= (entry->my_mac[1] << 16);
	VLAN_CTRL1_REG |= (entry->my_mac[2] << 8);
	VLAN_CTRL1_REG |= (entry->my_mac[3]);

	VLAN_CTRL2_REG |= (entry->my_mac[4] << 24);
	VLAN_CTRL2_REG |= (entry->my_mac[5] << 16);
	VLAN_CTRL2_REG |= entry->vid;

	ARL_VLAN_CMD_REG &= (~0x3f);
	ARL_VLAN_CMD_REG |= (entry->vlan_index);
	ARL_VLAN_CMD_REG |= (1 << 6); /* write vlan table command */

	/* wait for vlan command complete*/
	while (((ARL_VLAN_CMD_REG >> 9) & 1) == 0)
		;

	return CAVM_OK;
}

static inline int cns3xxx_arl_table_lookup(struct ARLTableEntry *entry)
{
	ARL_CTRL0_REG = 0;
	ARL_CTRL1_REG = 0;
	ARL_CTRL2_REG = 0;

	ARL_CTRL0_REG |= (entry->vid << 16);

	ARL_CTRL1_REG |= (entry->mac[0] << 24);
	ARL_CTRL1_REG |= (entry->mac[1] << 16);
	ARL_CTRL1_REG |= (entry->mac[2] << 8);
	ARL_CTRL1_REG |= entry->mac[3];

	ARL_CTRL2_REG |= (entry->mac[4] << 24);
	ARL_CTRL2_REG |= (entry->mac[5] << 16);

	ARL_VLAN_CMD_REG |= (1 << 18); /* arl table lookup command*/

	/* wait arl command complete */
	while (((ARL_VLAN_CMD_REG >> 21) & 1) == 0)
		;

	if (((ARL_VLAN_CMD_REG >> 23) & 1)) {
		entry->vid = ((ARL_CTRL0_REG >> 16) & 0xfff);
		entry->pmap = ((ARL_CTRL0_REG >> 9) & 0x1f);

		entry->age_field = ((ARL_CTRL2_REG >> 4) & 0x7);
		entry->vlan_mac = ((ARL_CTRL2_REG >> 1) & 0x1);
		entry->filter = (ARL_CTRL2_REG & 0x1);
		return CAVM_FOUND;
	} else
		return CAVM_NOT_FOUND;


}

static inline int
	cns3xxx_arl_table_search_again(struct ARLTableEntry *entry)
{
	ARL_CTRL0_REG = 0;
	ARL_CTRL1_REG = 0;
	ARL_CTRL2_REG = 0;

	ARL_VLAN_CMD_REG |= (1 << 17); /* arl table search again command*/

	/* wait arl command complete */
	while (((ARL_VLAN_CMD_REG >> 21) & 1) == 0)
		;

	if ((ARL_VLAN_CMD_REG >> 23) & 1) {
		entry->vid = ((ARL_CTRL0_REG >> 16) & 0xfff);
		entry->pmap = ((ARL_CTRL0_REG >> 9) & 0x1f);

		entry->age_field = ((ARL_CTRL2_REG >> 4) & 0x7);
		entry->vlan_mac = ((ARL_CTRL2_REG >> 1) & 0x1);
		entry->filter = (ARL_CTRL2_REG & 0x1);

		entry->mac[0] = (ARL_CTRL1_REG >> 24);
		entry->mac[1] = (ARL_CTRL1_REG >> 16);
		entry->mac[2] = (ARL_CTRL1_REG >> 8);
		entry->mac[3] = ARL_CTRL1_REG;

		entry->mac[4] = (ARL_CTRL2_REG >> 24);
		entry->mac[5] = (ARL_CTRL2_REG >> 16);

		return CAVM_FOUND;
	} else
		return CAVM_NOT_FOUND;
}

static inline int cns3xxx_is_arl_table_end(void)
{
	ARL_CTRL0_REG = 0;
	ARL_CTRL1_REG = 0;
	ARL_CTRL2_REG = 0;

	if (((ARL_VLAN_CMD_REG >> 22) & 1)) /* search to table end*/
		return CAVM_OK;
	else
		return CAVM_ERR;
}

static inline int cns3xxx_arl_table_search(struct ARLTableEntry *entry)
{
	ARL_CTRL0_REG = 0;
	ARL_CTRL1_REG = 0;
	ARL_CTRL2_REG = 0;

	printk(KERN_INFO "ARL_VLAN_CMD_REG: %x\n", ARL_VLAN_CMD_REG);
	ARL_VLAN_CMD_REG |= (1 << 16); /* arl table search start command */
	printk(KERN_INFO "11 ARL_VLAN_CMD_REG: %x\n", ARL_VLAN_CMD_REG);

	/* wait arl command complete */
	while (((ARL_VLAN_CMD_REG >> 21) & 1) == 0)
		;

	if (((ARL_VLAN_CMD_REG >> 23) & 1)) {
		entry->vid = ((ARL_CTRL0_REG >> 16) & 0xfff);
		entry->pmap = ((ARL_CTRL0_REG >> 9) & 0x1f);

		entry->age_field = ((ARL_CTRL2_REG >> 4) & 0x7);
		entry->vlan_mac = ((ARL_CTRL2_REG >> 1) & 0x1);
		entry->filter = (ARL_CTRL2_REG & 0x1);

		entry->mac[0] = (ARL_CTRL1_REG >> 24);
		entry->mac[1] = (ARL_CTRL1_REG >> 16);
		entry->mac[2] = (ARL_CTRL1_REG >> 8);
		entry->mac[3] = ARL_CTRL1_REG;

		entry->mac[4] = (ARL_CTRL2_REG >> 24);
		entry->mac[5] = (ARL_CTRL2_REG >> 16);

		return CAVM_FOUND;
	} else
		return CAVM_NOT_FOUND;
}


/* flush all age out entries except static entries*/
static inline int cns3xxx_arl_table_flush(void)
{
	ARL_VLAN_CMD_REG |= (1 << 20); /*flush arl table command*/

	/* wait arl command complete */
	while (((ARL_VLAN_CMD_REG >> 21) & 1) == 0)
		;

	return CAVM_OK;
}


/* add a entry in the arl table */
static inline int cns3xxx_arl_table_add(struct ARLTableEntry *entry)
{
	ARL_CTRL0_REG = 0;
	ARL_CTRL1_REG = 0;
	ARL_CTRL2_REG = 0;

	entry->age_field = 7; /* static entry */
	ARL_CTRL0_REG |= (entry->vid << 16);
	ARL_CTRL0_REG |= (entry->pmap << 9);

	ARL_CTRL1_REG |= (entry->mac[0] << 24);
	ARL_CTRL1_REG |= (entry->mac[1] << 16);
	ARL_CTRL1_REG |= (entry->mac[2] << 8);
	ARL_CTRL1_REG |= entry->mac[3];

	ARL_CTRL2_REG |= (entry->mac[4] << 24);
	ARL_CTRL2_REG |= (entry->mac[5] << 16);

	ARL_CTRL2_REG |= (entry->age_field << 4);
	ARL_CTRL2_REG |= (entry->vlan_mac << 1);
	ARL_CTRL2_REG |= (entry->filter);

	ARL_VLAN_CMD_REG |= (1 << 19); /* arl table write command*/

	/* wait arl command complete */
	while (((ARL_VLAN_CMD_REG >> 21) & 1) == 0)
		;

	return CAVM_OK;
}

/* invalid a entry in the arl table*/
static inline int cns3xxx_arl_table_invalid(struct ARLTableEntry *entry)
{
	entry->age_field = 0;
	return cns3xxx_arl_table_add(entry);
}

/* port:
 * 0 : mac port0
 * 1 : mac port1
 * 2 : mac port2
 * 3 : cpu port
 */
static inline void cns3xxx_set_pvid(u8 port, u16 pvid)
{
	switch (port) {
	case 0:
		MAC1_MAC0_PVID_REG &= (~0x0fff);
		MAC1_MAC0_PVID_REG |= pvid;
		break;
	case 1:
		MAC1_MAC0_PVID_REG &= (~(0x0fff << 16));
		MAC1_MAC0_PVID_REG |= (pvid << 16);
		break;
	case 2:
		MAC2_CPU_PVID_REG &= (~(0x0fff << 16));
		MAC2_CPU_PVID_REG |= (pvid << 16);
		break;
	case 3:
		MAC2_CPU_PVID_REG &= (~0x0fff);
		MAC2_CPU_PVID_REG |= pvid;
		break;
	}
}

static inline u16 cns3xxx_get_pvid(u8 port)
{
	/* 0,     1,   2,    cpu port */
	u16 port_offset[] = {0x9c, 0x9c, 0xa0, 0xa0};
	u16 port_shift[] = {0, 16, 16, 0};

	return (SWITCH_REG_VALUE(port_offset[port]) >> port_shift[port])
		& 0xfff;
}

/* which : 0 or 1
 * enable: 0 or 1
 */
static inline int enable_rx_dma(u8 which, u8 enable)
{
	if (which == 0)
		FS_DMA_CTRL0_REG = enable;
	else if (which == 1)
		FS_DMA_CTRL1_REG = enable;
	else
		return CAVM_ERR;
	return CAVM_OK;
}

/* which : 0 or 1
 * enable: 0 or 1
 */
static inline int enable_tx_dma(u8 which, u8 enable)
{
	if (which == 0)
		TS_DMA_CTRL0_REG = enable;
	else if (which == 1)
		TS_DMA_CTRL1_REG = enable;
	else
		return CAVM_ERR;
	return CAVM_OK;
}

/* clear: 0 normal
 * clear: 1 clear
 */
static inline void clear_fs_dma_state(u8 clear)
{
}

/* enable: 1 -> IVL
 * enable: 0 -> SVL
 */
static inline void cns3xxx_ivl(u8 enable)
{
	MAC_GLOB_CFG_REG &= (~(0x1 << 7));
	if (enable == 1)
		MAC_GLOB_CFG_REG |= (0x1 << 7);
}

static inline void cns3xxx_disable_irq(u32 irq)
{
	disable_irq_nosync(irq);
}

static inline void cns3xxx_enable_irq(u32 irq)
{
	enable_irq(irq);
}

static inline int cns3xxx_get_tx_hw_index(u8 ring_index)
{
	if (ring_index == 0)
		return (TS_DESC_PTR0_REG - TS_DESC_BASE_ADDR0_REG) /
			sizeof(struct TXDesc);
	else if (ring_index == 1)
		return (TS_DESC_PTR1_REG - TS_DESC_BASE_ADDR1_REG) /
			sizeof(struct TXDesc);
	else
		return CAVM_ERR;
}

static inline struct TXBuffer *get_tx_buffer_by_index(
	struct TXRing *tx_ring, int i)
{
	int index = i;
	index = ((index + get_tx_ring_size(tx_ring)) %
		get_tx_ring_size(tx_ring));

	return tx_ring->head + index;
}

static inline int cns3xxx_is_untag_packet(const struct RXDesc *rx_desc)
{
	return rx_desc->crc_err;
}

static inline int cns3xxx_min_mtu(void)
{
	return 64;
}

static inline int cns3xxx_max_mtu(void)
{
	int max_len[] = {1518, 1522, 1536, 9600};
	return max_len[((PHY_AUTO_ADDR_REG >> 30) & 0x3)];
}

#endif
