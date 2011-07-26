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

#include <linux/module.h>
#include <mach/cns3xxx.h>

#include "cns3xxx.h"
#include "cns3xxx_tool.h"
#include "cns3xxx_config.h"
#include "pse_init_common.h"

#if defined(CONFIG_CNS3XXX_ETHADDR_IN_FLASH)
#include <linux/mtd/mtd.h>
#endif

#define RX_SDP_ALIGN 64

ushort MAX_PACKET_LEN = 1536;
#define CPU_CACHE_BYTES         64
#define CPU_CACHE_ALIGN(X) \
	(((X) + (CPU_CACHE_BYTES-1)) & ~(CPU_CACHE_BYTES-1))


#define QUEUE_WEIGHT_SET(port, ctl) \
{ \
	MAC##port##_PRI_CTRL_REG &= ~(0x3ffff); \
	MAC##port##_PRI_CTRL_REG |= (ctl.sch_mode << 16); \
	MAC##port##_PRI_CTRL_REG |= (ctl.q0_w); \
	MAC##port##_PRI_CTRL_REG |= (ctl.q1_w << 4); \
	MAC##port##_PRI_CTRL_REG |= (ctl.q2_w << 8); \
	MAC##port##_PRI_CTRL_REG |= (ctl.q3_w << 12); \
}

#define QUEUE_WEIGHT_GET(port, ctl) \
{ \
	ctl.sch_mode = ((MAC##port##_PRI_CTRL_REG >> 16) & 0x3); \
	ctl.q0_w = ((MAC##port##_PRI_CTRL_REG >> 0) & 0x7); \
	ctl.q1_w = ((MAC##port##_PRI_CTRL_REG >> 4) & 0x7); \
	ctl.q2_w = ((MAC##port##_PRI_CTRL_REG >> 8) & 0x7); \
	ctl.q3_w = ((MAC##port##_PRI_CTRL_REG >> 12) & 0x7); \
}

#if defined(CONFIG_CNS3XXX_ETHADDR_IN_FLASH)

#define MTD_READ(mtd, args...) (*(mtd->read))(mtd, args)

#ifdef CONFIG_CNS3XXX_MAC_IN_SPI_FLASH
#define ENV_OFFSET 0x30000
#define PARTITION_NAME "SPI-UBoot"
#else
#define ENV_OFFSET 0x0
#define PARTITION_NAME "UBootEnv"
#endif

#define MTD_READ_LEN 1024

static char mtd_str[MTD_READ_LEN], ethaddr[12];

static int init_mtd_env(void)
{
	struct mtd_info *mtd;
	size_t retlen = 0;

	mtd = get_mtd_device_nm(PARTITION_NAME);

	if (IS_ERR(mtd))
		return -ENODEV;

	MTD_READ(mtd, ENV_OFFSET, MTD_READ_LEN, &retlen, mtd_str);
	return 0;
}

int fmg_get(const char *name, int *ret_len)
{
	int i, j, x, z;
	int nlen = strlen(name);
	char tmp_str[20];

	memset(ethaddr, 0x0, 12);

	for (i = 0; i < MTD_READ_LEN - nlen; i++) {
		z = 0;
		for (x = 0; x < nlen; x++) {
			if (mtd_str[i+x] == name[x])
				z++;
		}
		if (z == nlen) {
			memcpy(tmp_str, mtd_str + i + nlen, 17);
			tmp_str[17] = '\0';
			for (j = 0; j < 17; j++) {
				if (tmp_str[j] != ':')
					sprintf(ethaddr,
						"%s%c", ethaddr, tmp_str[j]);
			}
			*ret_len = strlen(ethaddr);
			return 0;
		}
	}
	return -1;
}

int mac_str_to_int(
	const char *mac_str, int mac_str_len, u8 *mac_int, int mac_len)
{
	int i = 0, j = 0;
	char mac_s[3] = {0, 0, 0};

	for (i = 0; i < mac_str_len; i += 2) {
		mac_s[0] = mac_str[i];
		mac_s[1] = mac_str[i + 1];
		mac_int[j++] = simple_strtol(mac_s, NULL, 16);
	}
	return 0;
}
#endif

static int install_isr_rc;
static int rc_setup_rx_tx; /* rc means reference counting.*/
static struct net_device *intr_netdev;
struct net_device *net_dev_array[NETDEV_SIZE];
spinlock_t tx_lock;
spinlock_t rx_lock;

u8 show_rx_proc;
u8 show_tx_proc;

const int MAX_RX_DESC_SIZE = 512;
const int MAX_TX_DESC_SIZE = 512;
int RX_DESC_SIZE = 128;
int TX_DESC_SIZE = 120;
module_param(RX_DESC_SIZE, int, 0);
module_param(TX_DESC_SIZE, int, 0);

u8 ring_index; /* 0 or 1*/

#ifdef CNS3XXX_DELAYED_INTERRUPT
static u32 max_pend_int_cnt = MAX_PEND_INT_CNT;
static u32 max_pend_time = MAX_PEND_TIME;
#endif

#ifdef CONFIG_CNS3XXX_NAPI
struct net_device *napi_dev;
	#ifdef CNS3XXX_DOUBLE_RX_RING
	struct net_device *r1_napi_dev;
	#endif
#endif

struct RingInfo g_ring_info;

int MSG_LEVEL = NORMAL_MSG;

#ifdef CNS3XXX_STATUS_ISR
const char *cns3xxx_gsw_status_tbl[] = {
	"\nMAC0_Q_FULL\n",
	"\nMAC1_Q_FULL\n",
	"\nCPU_Q_FULL\n",
	"\nHNAT_Q_FULL\n",
	"\nMAC2_Q_FULL\n",
	"\nMAC0_Q_EXT_FULL\n",
	"\nGLOBAL_Q_FULL\n",
	"\nBUFFER_FULL\n",
	"\nMIB_COUNTER_TH\n",
	"\n",
	"\nMAC0_INTRUDER\n",
	"\nMAC1_INTRUDER\n",
	"\nCPU_INTRUDER\n",
	"\nMAC2_INTRUDER\n",
	"\nMAC0_STATUS_CHG\n",
	"\nMAC1_STATUS_CHG\n",
	"\nMAC2_STATUS_CHG\n",
	"\nMAC0_NO_LINK_DROP\n",
	"\nMAC1_NO_LINK_DROP\n",
	"\nMAC2_NO_LINK_DROP\n",
	"\nMAC0_RX_ERROR_DROP\n",
	"\nMAC1_RX_ERROR_DROP\n",
	"\nMAC2_RX_ERROR_DROP\n",
	"\nMAC0_NO_DESTINATION_DROP\n",
	"\nMAC1_NO_DESTINATION_DROP\n",
	"\nMAC2_NO_DESTINATION_DROP\n",
	"\nMAC0_RMC_PAUSE_DROP\n",
	"\nMAC1_RMC_PAUSE_DROP\n",
	"\nMAC2_RMC_PAUSE_DROP\n",
	"\nMAC0_LOCAL_DROP\n",
	"\nMAC1_LOCAL_DROP\n",
	"\nMAC2_LOCAL_DROP\n",
};
#endif

#define MIN_PACKET_LEN 14

static int cns3xxx_notify_reboot(
	struct notifier_block *nb, unsigned long event, void *ptr);

static struct notifier_block cns3xxx_notifier_reboot = {
	.notifier_call	= cns3xxx_notify_reboot,
	.next		= NULL,
	.priority	= 0
};

void __take_off_vlan_header(struct sk_buff *skb)
{
	/* take off VLAN header,minus 4 byte vlan tag*/
	memmove(skb->data + 4, skb->data, 12);
	skb->data += 4;
	skb->len -= 4;
}

static struct sk_buff *cns3xxx_alloc_skb(void)
{
	struct sk_buff *skb;
	u32 align_64;
	skb = dev_alloc_skb(MAX_PACKET_LEN + 2 + RX_SDP_ALIGN);

	if (unlikely(!skb)) {
		printk(KERN_ERR
			"dev_alloc_skb fail!! while allocate RFD ring !!\n");
		return NULL;
	}

	align_64 = CPU_CACHE_ALIGN((u32)skb->data);
	skb_reserve(skb, align_64-(u32)skb->data);	/* 16 bytes alignment */

	skb_reserve(skb, NET_IP_ALIGN);	/* 16 bytes alignment */

	return skb;
}

static int free_rx_skb(struct RXRing *rx_ring)
{
	int i = 0;
	struct RXBuffer *rx_buffer = rx_ring->head;

	for (i = 0; i < get_rx_ring_size(rx_ring); ++i) {
		if (rx_buffer->skb) {
			dma_unmap_single(NULL,
					 rx_buffer->rx_desc->sdp,
					 rx_buffer->rx_desc->sdl,
					 DMA_FROM_DEVICE);
			dev_kfree_skb(rx_buffer->skb);
			rx_buffer->skb = 0;
		}
		++rx_buffer;
	}
	return 0;
}

int cns3xxx_setup_all_rx_resources(struct RXRing *rx_ring, u8 ring_num)
{
	int i = 0;
	struct RXBuffer *rx_buffer = 0;
	struct RXDesc *rx_desc = 0;
	dma_addr_t dmap;

	/* alloc RXDesc array */
	rx_ring->rx_desc_head_vir_addr =
		dma_alloc_coherent(NULL,
			sizeof(struct RXDesc) * (get_rx_ring_size(rx_ring)),
			&rx_ring->rx_desc_head_phy_addr, GFP_KERNEL);
	if (!rx_ring->rx_desc_head_vir_addr) {
		printk(KERN_ERR
			"rx_ring->rx_desc_head_vir_addr alloc memory fail!\n");
		return -ENOMEM;
	}

	memset(rx_ring->rx_desc_head_vir_addr, 0,
		sizeof(struct RXDesc) * get_rx_ring_size(rx_ring));

	/* alloc RXBuffer array */
	rx_ring->head =
		kmalloc(
			sizeof(struct RXBuffer) * get_rx_ring_size(rx_ring),
			GFP_KERNEL);

	if (!rx_ring->head) {
		printk("rx_ring.head alloc memory fail!\n");
		return -ENOMEM;
	}

	rx_buffer = rx_ring->head;
	for (i = 0; i < get_rx_ring_size(rx_ring); ++i) {
		rx_buffer->skb = 0;
		++rx_buffer;
	}

	rx_buffer = rx_ring->head;
	rx_desc = rx_ring->rx_desc_head_vir_addr;
	for (i = 0; i < get_rx_ring_size(rx_ring);
			++i, ++rx_buffer, ++rx_desc) {
		rx_buffer->rx_desc = rx_desc;
		rx_buffer->skb = cns3xxx_alloc_skb();

		if (!rx_buffer->skb) {
			free_rx_skb(rx_ring);
			kfree(rx_ring->head);
			dma_free_coherent(NULL,
				sizeof(struct RXDesc) *
					get_rx_ring_size(rx_ring),
				rx_ring->rx_desc_head_vir_addr,
				rx_ring->rx_desc_head_phy_addr);
			printk(KERN_ERR "cannot alloc rx skb!!");
			return -ENOMEM;
		}

		rx_buffer->rx_desc->sdl = MAX_PACKET_LEN;
		if (i == (get_rx_ring_size(rx_ring)-1))
			rx_buffer->rx_desc->eor = 1;

		rx_buffer->rx_desc->fsd = 1;
		rx_buffer->rx_desc->lsd = 1;

		dmap = dma_map_single(NULL,
			rx_buffer->skb->data, MAX_PACKET_LEN, DMA_FROM_DEVICE);
		rx_buffer->rx_desc->sdp = dmap;
	}

	rx_ring->cur_index = 0 ;

	if (ring_num == 0) {
		FS_DESC_PTR0_REG = rx_ring->rx_desc_head_phy_addr;
		FS_DESC_BASE_ADDR0_REG = rx_ring->rx_desc_head_phy_addr;
	} else if (ring_num == 1) {
		FS_DESC_PTR1_REG = rx_ring->rx_desc_head_phy_addr;
		FS_DESC_BASE_ADDR1_REG = rx_ring->rx_desc_head_phy_addr;
	}

	return CAVM_OK;
}

static int cns3xxx_setup_all_tx_resources(struct TXRing *tx_ring, u8 ring_num)
{
	int i = 0;
	struct TXBuffer *tx_buffer = 0;
	struct TXDesc *tx_desc = 0;

	spin_lock_init(&(tx_ring->tx_lock));

	tx_ring->tx_desc_head_vir_addr =
		dma_alloc_coherent(NULL,
			sizeof(struct TXDesc) * get_tx_ring_size(tx_ring),
			&tx_ring->tx_desc_head_phy_addr, GFP_KERNEL);

	if (!tx_ring->tx_desc_head_vir_addr) {
		printk(KERN_ERR
			"tx_ring->tx_desc_head_vir_addr alloc memory fail!\n");
		return -ENOMEM;
	}

	memset(tx_ring->tx_desc_head_vir_addr, 0,
		sizeof(struct TXDesc) * get_tx_ring_size(tx_ring));
	tx_ring->head = kmalloc(
		sizeof(struct TXBuffer) * get_tx_ring_size(tx_ring),
		GFP_KERNEL);

	tx_buffer = tx_ring->head;
	tx_desc = tx_ring->tx_desc_head_vir_addr;

	for (i = 0; i < get_tx_ring_size(tx_ring);
			++i, ++tx_buffer, ++tx_desc) {
		tx_buffer->tx_desc = tx_desc;

		tx_buffer->tx_desc->cown = 1;
		tx_buffer->skb = 0;
		if (i == (get_tx_ring_size(tx_ring)-1))
			tx_buffer->tx_desc->eor = 1;
	}
	tx_ring->num_free_desc = get_tx_ring_size(tx_ring);
	tx_ring->cur_index = 0 ;

	if (ring_num == 0) {
		TS_DESC_PTR0_REG = tx_ring->tx_desc_head_phy_addr;
		TS_DESC_BASE_ADDR0_REG = tx_ring->tx_desc_head_phy_addr;
	} else if (ring_num == 1) {
		TS_DESC_PTR1_REG = tx_ring->tx_desc_head_phy_addr;
		TS_DESC_BASE_ADDR1_REG = tx_ring->tx_desc_head_phy_addr;
	}
	return CAVM_OK;
}

int cns3xxx_free_all_rx_resources(struct RXRing *rx_ring)
{
	free_rx_skb(rx_ring);
	kfree(rx_ring->head);
	dma_free_coherent(NULL,
		sizeof(struct RXDesc) * get_rx_ring_size(rx_ring),
		rx_ring->rx_desc_head_vir_addr,
		rx_ring->rx_desc_head_phy_addr);
	return 0;
}

static int free_tx_skb(struct TXRing *tx_ring)
{
	int i = 0;
	struct TXBuffer *tx_buffer = tx_ring->head;

	for (i = 0; i < get_tx_ring_size(tx_ring); ++i) {
		if (tx_buffer->skb) {
			dma_unmap_single(NULL,
					 tx_buffer->tx_desc->sdp,
					 tx_buffer->tx_desc->sdl,
					 DMA_TO_DEVICE);

			dev_kfree_skb(tx_buffer->skb);
			tx_buffer->skb = 0;
		}
		++tx_buffer;
	}
	return 0;
}

int cns3xxx_free_all_tx_resources(struct TXRing *tx_ring)
{
	free_tx_skb(tx_ring);
	kfree(tx_ring->head);
	dma_free_coherent(NULL,
		sizeof(struct TXDesc) * get_tx_ring_size(tx_ring),
		tx_ring->tx_desc_head_vir_addr, tx_ring->tx_desc_head_phy_addr);
	return 0;
}

static int cns3xxx_free_rx_tx_res(struct CNS3XXXPrivate *priv)
{
	int i = 0;

	--rc_setup_rx_tx;
	if (rc_setup_rx_tx == 0) {
		clear_fs_dma_state(1);
		printk(KERN_INFO "free tx/rx resource\n");
		enable_port(3, 0); /* disable cpu port */

		for (i = 0 ; i < priv->num_rx_queues ; ++i) {
			/* stop RX dma */
			enable_rx_dma(i, 0);
			cns3xxx_free_all_rx_resources(priv->rx_ring+i);
			memset(priv->rx_ring + i, 0, sizeof(struct RXRing));
		}

		for (i = 0; i < priv->num_tx_queues; ++i) {
			/* stop TX dma*/
			enable_tx_dma(i, 0);
			cns3xxx_free_all_tx_resources(priv->tx_ring+i);
			memset(priv->tx_ring + i, 0, sizeof(struct TXRing));
		}
	}
	return 0;
}


static int cns3xxx_setup_rx_tx_res(struct CNS3XXXPrivate *priv)
{
	int i = 0;

	if (rc_setup_rx_tx == 0) {
		printk(KERN_INFO "alloc tx/rx resource\n");
		clear_fs_dma_state(1);
		FS_DESC_PTR0_REG = 0;
		FS_DESC_BASE_ADDR0_REG = 0;
		FS_DESC_PTR1_REG = 0;
		FS_DESC_BASE_ADDR1_REG = 0;
		TS_DESC_PTR0_REG = 0;
		TS_DESC_BASE_ADDR0_REG = 0;
		TS_DESC_PTR1_REG = 0;
		TS_DESC_BASE_ADDR1_REG = 0;

		for (i = 0; i < priv->num_tx_queues; ++i) {
			spin_lock_init(&((priv->tx_ring+i)->tx_lock));
			(priv->tx_ring+i)->max_ring_size = MAX_TX_DESC_SIZE;
			(priv->tx_ring+i)->ring_size = TX_DESC_SIZE;
			if (cns3xxx_setup_all_tx_resources(priv->tx_ring+i, i)
				!= CAVM_OK)
				return CAVM_ERR;
		}

		for (i = 0; i < priv->num_rx_queues; ++i) {
			(priv->rx_ring+i)->max_ring_size = MAX_RX_DESC_SIZE;
			(priv->rx_ring+i)->ring_size = RX_DESC_SIZE;
			if (cns3xxx_setup_all_rx_resources(priv->rx_ring+i, i)
				!= CAVM_OK)
				return CAVM_ERR;
		}
		clear_fs_dma_state(0);
	}
	++rc_setup_rx_tx;
	return CAVM_OK;
}

static int free_tx_desc_skb(struct TXRing *tx_ring, u8 ring_num)
{
	int i;
	struct TXBuffer *tx_buffer;
	u32 tx_ring_size = get_tx_ring_size(tx_ring);
	/* check curent hw index previous tx descriptor */
	u32 cur_index = cns3xxx_get_tx_hw_index(ring_num) - 1;
	struct TXDesc *tx_desc_ptr;

	tx_buffer = get_tx_buffer_by_index(tx_ring, cur_index);

	for (i = 0; i < tx_ring_size; ++i) {
		if (tx_buffer->tx_desc->cown == 1 && tx_buffer->skb) {
			tx_desc_ptr = tx_buffer->tx_desc;
			dma_unmap_single(NULL,
					 tx_desc_ptr->sdp,
					 tx_desc_ptr->sdl,
					 DMA_TO_DEVICE);
			dev_kfree_skb_any(tx_buffer->skb);
			tx_buffer->skb = 0;
			tx_ring->num_free_desc++;
		} else {
			break;
		}
		--cur_index;
		tx_buffer = get_tx_buffer_by_index(tx_ring, cur_index);
	}
	return 0;
}

void assign_netdev(volatile struct RXBuffer *rx_buffer)
{
	struct RXDesc *rx_desc = 0;
	rx_desc = rx_buffer->rx_desc;

	if (is_config_cns3xxx_port_base()) {
		switch (rx_desc->sp) {
		case 0:
			rx_buffer->skb->dev = PORT0_NETDEV;
			break;
		case 1:
			rx_buffer->skb->dev = PORT1_NETDEV;
			break;
		case 4:
			rx_buffer->skb->dev = PORT2_NETDEV;
			break;
		}
	}

	if (is_config_cns3xxx_vlan_base()) {
		u16 vlan_tag;

		vlan_tag = rx_desc->c_vid;
		rx_buffer->skb->dev = net_dev_array[vlan_tag];
	}
}

/* old_priv has ring index information,
 * current version only uses the information.
 */
static int cns3xxx_get_rfd_buff(
	struct RXDesc *rx_desc, volatile struct RXBuffer *rx_buffer,
	struct CNS3XXXPrivate *old_priv)
{
	struct CNS3XXXPrivate *priv = 0;
	struct sk_buff *skb;
	u32 len;

	skb = rx_buffer->skb;
	len = rx_desc->sdl;

	dma_unmap_single(NULL,
		 rx_desc->sdp,
		 rx_desc->sdl,
		 DMA_FROM_DEVICE);

	if (is_cns3xxx_non_nic_mode_8021q()) {
		if (cns3xxx_is_untag_packet(rx_desc) == 1)
			__take_off_vlan_header(skb);
	}

	if (is_config_cns3xxx_port_base()) {
		assign_netdev(rx_buffer);

		if (rx_buffer->skb->dev)
			/* if skb->dev is 0, means VLAN base*/
			goto determine_dev_ok;
	}

	if (is_config_cns3xxx_vlan_base()) {
		if (is_config_have_vlan_tag()) {
			assign_netdev(rx_buffer);

			__take_off_vlan_header(skb);
			if (MSG_LEVEL == 5)
				print_packet(skb->data, 32);

			if (rx_buffer->skb->dev == 0)
				goto freepacket;
		}
	}

determine_dev_ok:
	skb_put(skb, len);
	if (skb->dev)
		priv = netdev_priv(skb->dev);
	else {
		DEBUG_MSG(WARNING_MSG, "skb_ptr->dev==NULL\n");
		goto freepacket;
	}

#ifdef CNS3XXX_RX_HW_CHECKSUM
	switch (rx_desc->prot) {
	case 1:
	case 2:
	case 5:
	case 6:
		if (rx_desc->l4f == 0)
			/* tcp/udp checksum is correct */
			skb->ip_summed = CHECKSUM_UNNECESSARY;
		else
			skb->ip_summed = CHECKSUM_NONE;
		break;
	default:
		skb->ip_summed = CHECKSUM_NONE;
		break;
	}
#else
	skb->ip_summed = CHECKSUM_NONE;
#endif

	/* this line must, if no,
	 * packet will not send to network layer
	 */
	skb->protocol = eth_type_trans(skb, skb->dev);

	skb->dev->last_rx = jiffies;
	priv->stats.rx_packets++;
	priv->stats.rx_bytes += len;

#ifdef CONFIG_CNS3XXX_NAPI
	netif_receive_skb(skb);
#else
	netif_rx(skb);
#endif
	return 0;

freepacket:
	dev_kfree_skb_any(skb);
	return 0;
}

/* index from 1 */
u32 get_rx_hw_index(struct CNS3XXXPrivate *priv)
{
	return (FS_DESC_PTR0_REG - get_rx_head_phy_addr(&RX_RING0(priv))) /
		sizeof(struct RXDesc);
}

int get_rx_hw_index_by_reg(u8 ring_num)
{
	if (ring_num == 0)
		return (FS_DESC_PTR0_REG - FS_DESC_BASE_ADDR0_REG) /
			sizeof(struct RXDesc);
	else if (ring_num == 1)
		return (FS_DESC_PTR1_REG - FS_DESC_BASE_ADDR1_REG) /
			sizeof(struct RXDesc);
	return CAVM_FAIL;
}

#ifdef CONFIG_CNS3XXX_NAPI
void cns3xxx_receive_packet(
	struct CNS3XXXPrivate *priv, int mode, int *work_done, int work_to_do)
#else
void cns3xxx_receive_packet(struct CNS3XXXPrivate *priv, int mode)
#endif
{
	register int fssd_index;
	register struct RXBuffer volatile *rx_buffer = 0;
	register struct RXDesc volatile *rx_desc = 0;
	register struct sk_buff *skb;
#ifndef CONFIG_CNS3XXX_NAPI
	register int fsqf = 0; /* Queue Full Mode =0 */
#endif
	register int i, rxcount = 0;
	register u8 queue_index = priv->ring_index;
	register dma_addr_t dmap;

	register volatile struct RXDesc *rx_desc_ori_ptr = 0;
	volatile struct RXDesc rx_desc_tmp;

	rx_buffer = get_cur_rx_buffer(&(priv->rx_ring[queue_index]));
	rx_desc = rx_buffer->rx_desc;

	fssd_index = get_rx_hw_index_by_reg(queue_index);

	if (fssd_index > get_rx_cur_index(&priv->rx_ring[queue_index]))
		rxcount = fssd_index -
			get_rx_cur_index(&priv->rx_ring[queue_index]);
	else if (fssd_index < get_rx_cur_index(&priv->rx_ring[queue_index]))
		rxcount = (get_rx_ring_size(&priv->rx_ring[queue_index]) -
			get_rx_cur_index(&priv->rx_ring[queue_index])) +
			fssd_index;
	else {
		if (rx_desc->cown == 0) {
			/* if rx_desc->cown is 1,
			 * we can receive the RX descriptor.
			 */
			enable_rx_dma(0, 1);
			goto receive_packet_exit;
		} else {
			/* Queue Full*/
#ifndef CONFIG_CNS3XXX_NAPI
			fsqf = 1;
#endif
			rxcount = get_rx_ring_size(&priv->rx_ring[queue_index]);
		}
	}

#ifndef CONFIG_CNS3XXX_NAPI
	if (mode == 1) {
		fsqf = 1;
		rxcount = get_rx_ring_size(&priv->rx_ring[queue_index]);
	}
#endif

#ifdef CNS3XXX_FREE_TX_IN_RX_PATH
	free_tx_desc_skb(priv->tx_ring + 0, 0);
#ifdef CNS3XXX_DOUBLE_TX_RING
	free_tx_desc_skb(priv->tx_ring + 1, 1);
#endif
#endif

	for (i = 0; i < rxcount; i++) {
		if (rx_desc->cown != 0) {
			/* start to get packet
			 * Alloc New skb_buff*/
			skb = cns3xxx_alloc_skb();
			/* Check skb_buff*/
			if (skb) {
				rx_desc_ori_ptr = rx_desc;
				rx_desc = &rx_desc_tmp;
				rx_desc_tmp = *rx_desc_ori_ptr;

				cns3xxx_get_rfd_buff(
					(struct RXDesc *)rx_desc,
					rx_buffer, priv);

				rx_buffer->skb = skb;

				dmap = dma_map_single(NULL, skb->data,
					MAX_PACKET_LEN, DMA_FROM_DEVICE);

				rx_desc->sdp = dmap;
				rx_desc->sdl = MAX_PACKET_LEN;
				rx_desc->fsd = 1;
				rx_desc->lsd = 1;

				*rx_desc_ori_ptr = rx_desc_tmp;
				rx_desc = rx_desc_ori_ptr;

				rx_desc->cown = 0; /* set cbit to 0*/

#ifdef CONFIG_CNS3XXX_NAPI
				++(*work_done);
				if (*work_done >= work_to_do) {
					rx_index_next(
						&priv->rx_ring[queue_index]);
					/*rx_ring.cur_index points to next*/
					rx_buffer = get_cur_rx_buffer(
						&priv->rx_ring[queue_index]);
					rx_desc = rx_buffer->rx_desc;
					break;
				}
#endif
			} else {
				printk(KERN_ERR
					"%s: Alloc sk_buf fail, reuse buffer\n",
					__func__);
				rx_desc->cown = 0; /* set cbit to 0*/
				return;
			}
		} else  /* cown is 0, no packets */
			return;

		rx_index_next(&priv->rx_ring[queue_index]);
		/* rx_ring.cur_index points to next*/
		rx_buffer = get_cur_rx_buffer(&priv->rx_ring[queue_index]);
		rx_desc = rx_buffer->rx_desc;
	} /* end for (i = 0; i < rxcount; i++) */

#ifndef CONFIG_CNS3XXX_NAPI
	if (fsqf) {
		priv->rx_ring[queue_index].cur_index = fssd_index;
		mb();
		enable_rx_dma(0, 1);
	}
#endif

receive_packet_exit:
	return;
}

irqreturn_t cns3xxx_fsrc_ring0_isr(int irq, void *dev_id)
{
#ifdef CONFIG_CNS3XXX_NAPI
	struct CNS3XXXPrivate *priv = netdev_priv(napi_dev);
	priv->ring_index = 0;

	cns3xxx_disable_irq(FSRC_RING0_INTERRUPT_ID);

	if (likely(napi_schedule_prep(&priv->napi)))
		__napi_schedule(&priv->napi);
	else
		cns3xxx_enable_irq(FSRC_RING0_INTERRUPT_ID);

#else
	struct net_device *netdev = dev_id;
	struct CNS3XXXPrivate *priv = netdev_priv(netdev);

	priv->ring_index = 0;

	cns3xxx_disable_irq(FSRC_RING0_INTERRUPT_ID);
	cns3xxx_disable_irq(FSQF_RING0_INTERRUPT_ID);

	cns3xxx_receive_packet(priv, 0); /* Receive Once*/

	cns3xxx_enable_irq(FSRC_RING0_INTERRUPT_ID);
	cns3xxx_enable_irq(FSQF_RING0_INTERRUPT_ID);

	enable_rx_dma(0, 1);
#endif
	return IRQ_HANDLED;
}

#if defined(CNS3XXX_DOUBLE_RX_RING)
irqreturn_t cns3xxx_fsrc_ring1_isr(int irq, void *dev_id)
{
#if defined(CONFIG_CNS3XXX_NAPI) && defined(CNS3XXX_DOUBLE_RX_RING)
	struct CNS3XXXPrivate *priv = netdev_priv(r1_napi_dev);
	priv->ring_index = 1;

	cns3xxx_disable_irq(FSRC_RING1_INTERRUPT_ID);

	if (likely(napi_schedule_prep(&priv->napi)))
		__napi_schedule(&priv->napi);
	else
		cns3xxx_enable_irq(FSRC_RING1_INTERRUPT_ID);
#else
	struct net_device *netdev = dev_id;
	struct CNS3XXXPrivate *priv = netdev_priv(netdev);
	priv->ring_index = 1;

	cns3xxx_disable_irq(CNS3XXX_FSRC_RING1_INTERRUPT_ID);
	cns3xxx_disable_irq(CNS3XXX_FSQF_RING1_INTERRUPT_ID);
	cns3xxx_receive_packet(priv, 0); /* Receive Once */
	enable_rx_dma(1, 1);

	cns3xxx_enable_irq(CNS3XXX_FSRC_RING1_INTERRUPT_ID);
	cns3xxx_enable_irq(CNS3XXX_FSQF_RING1_INTERRUPT_ID);
#endif
	return IRQ_HANDLED;
}
#endif

static int cns3xxx_check_enough_tx_descriptor(
	struct TXRing *tx_ring, int need_free_tx_desc)
{
	int i = 0;
	struct TXDesc *tx_desc = 0;
	u32 cur_index = get_tx_cur_index(tx_ring);
	struct TXBuffer *tx_buffer = get_tx_buffer_by_index(tx_ring, cur_index);

	tx_desc = tx_buffer->tx_desc;

	for (i = 0; i < need_free_tx_desc; ++i) {
		if (tx_desc->cown == 0)
			return 0; /* no free TX descriptor*/

		tx_buffer = get_tx_buffer_by_index(tx_ring, ++cur_index);
		tx_desc = tx_buffer->tx_desc;
	}

	return 1;
}

/* if return CAVM_ERR, means pad is fail, the packet cannot send by switch.*/
static int fill_a_skb_to_tx_desc(
	struct TXBuffer *tx_buffer, u8 *data, int len, struct sk_buff *skb,
	const struct CNS3XXXPrivate *priv, int sg, int fsd, int lsd)
{
	register dma_addr_t dmap;
	register struct TXRing *tx_ring;

	register struct TXDesc *tx_desc_ptr = 0;
	register struct TXDesc *tx_desc_ori_ptr = 0;
	struct TXDesc tx_desc_tmp;

	tx_desc_ptr = tx_buffer->tx_desc;

	tx_desc_ori_ptr = tx_desc_ptr;
	tx_desc_ptr = &tx_desc_tmp;
	tx_desc_tmp = *tx_desc_ori_ptr;


	if (tx_buffer->skb) {
		dma_unmap_single(NULL,
				tx_desc_ptr->sdp,
				tx_desc_ptr->sdl,
				DMA_TO_DEVICE);
		dev_kfree_skb_any(tx_buffer->skb);
		tx_buffer->skb = 0 ;
	}

	tx_buffer->skb = skb;  /* for free skb */
	tx_buffer->tx_index = cns3xxx_get_tx_hw_index(0);

#ifdef CNS3XXX_TX_HW_CHECKSUM
	tx_desc_ptr->ico = 1;
	tx_desc_ptr->uco = 1;
	tx_desc_ptr->tco = 1;
#else
	tx_desc_ptr->ico = 0;
	tx_desc_ptr->uco = 0;
	tx_desc_ptr->tco = 0;
#endif
	/* Wake interrupt */
#ifdef CNS3XXX_TSTC_RING0_ISR
	tx_desc_ptr->interrupt = 1;
#else
	tx_desc_ptr->interrupt = 0;
#endif

	/* fill 0 to MIN_PACKET_LEN size
	 * can change MIN_PACKET_LEN to 14
	 */
	if (sg == 0 && len < MIN_PACKET_LEN) {
		if (skb_padto(skb, MIN_PACKET_LEN)) {
			printk(KERN_ERR "padding skb error.\n");
			return CAVM_ERR;
		}
		tx_desc_ptr->sdl = MIN_PACKET_LEN;
	} else {
		tx_desc_ptr->sdl = len;
	}

	dmap = dma_map_single(NULL, data, len, DMA_TO_DEVICE);
	tx_desc_ptr->sdp = dmap;

	/* VLAN base or port base function to set TX descriptor */
	/* reference: tx_//port_base(), tx_vlan_base() */
	priv->net_device_priv->tx_func(tx_desc_ptr, priv, skb);
	tx_desc_ptr->fsd = fsd;
	tx_desc_ptr->lsd = lsd;

	*tx_desc_ori_ptr = tx_desc_tmp;
	tx_desc_ptr = tx_desc_ori_ptr;

	/* NOT SG packet */
	if (!sg)
		tx_desc_ptr->cown = 0;

	tx_ring = priv->tx_ring + ring_index;

	if (tx_ring->num_free_desc)
		tx_ring->num_free_desc--;

	return CAVM_OK;
}

int cns3xxx_send_packet(struct sk_buff *skb, struct net_device *netdev)
{
	register struct CNS3XXXPrivate *priv = netdev_priv(netdev);
	register struct TXBuffer *tx_buffer = 0;
	register unsigned long flags;

	register int nr_frags = skb_shinfo(skb)->nr_frags;
	struct TXDesc *tx_desc[10];
	register int tx_desc_count = 0;
	register int i = 0;

	spin_lock_irqsave(&tx_lock, flags);

	if (cns3xxx_check_enough_tx_descriptor(priv->tx_ring + ring_index,
			(nr_frags == 0) ? 1 : nr_frags) == 0) {
		/* no enough tx descriptor */
		spin_unlock_irqrestore(&tx_lock, flags);
		/* re-queue the skb */
		return NETDEV_TX_BUSY;
	}
	tx_buffer = get_cur_tx_buffer(priv->tx_ring + ring_index);

	if (nr_frags == 0) { /* non scatter/gather I/O */
		fill_a_skb_to_tx_desc(tx_buffer, skb->data,
			skb->len, skb, priv, 0, 1, 1);
		tx_index_next(priv->tx_ring + ring_index);
	} else { /* scatter/gather I/O */
		struct skb_frag_struct *frag = 0;

		fill_a_skb_to_tx_desc(tx_buffer, skb->data,
			skb->len - skb->data_len, 0, priv, 1, 1, 0);
		tx_desc[tx_desc_count++] = tx_buffer->tx_desc;
		tx_index_next(priv->tx_ring + ring_index);
		tx_buffer = get_cur_tx_buffer(priv->tx_ring + ring_index);

		for (i = 0; i < nr_frags - 1 ; ++i) {
			frag = &skb_shinfo(skb)->frags[i];

			fill_a_skb_to_tx_desc(tx_buffer,
				page_address(frag->page) + frag->page_offset,
				frag->size, 0, priv, 1, 0, 0);
			tx_desc[tx_desc_count++] = tx_buffer->tx_desc;

			tx_index_next(priv->tx_ring + ring_index);
			tx_buffer =
				get_cur_tx_buffer(priv->tx_ring + ring_index);
		}
		frag = &skb_shinfo(skb)->frags[nr_frags-1];

		/* last fragment */
		fill_a_skb_to_tx_desc(tx_buffer,
			page_address(frag->page) + frag->page_offset,
			frag->size, skb, priv, 1, 0, 1);
		tx_desc[tx_desc_count++] = tx_buffer->tx_desc;

		tx_index_next(priv->tx_ring + ring_index);
		tx_buffer = get_cur_tx_buffer(priv->tx_ring + ring_index);

		for (i = 0; i < tx_desc_count ; i++)
			tx_desc[i]->cown = 0 ;
	}

	mb();

	enable_tx_dma(ring_index, 1);

	priv->stats.tx_packets++;
	priv->stats.tx_bytes += skb->len;
	netdev->trans_start = jiffies;

	spin_unlock_irqrestore(&tx_lock, flags);

	return NETDEV_TX_OK;
}

#ifdef CNS3XXX_FSQF_RING0_ISR
irqreturn_t cns3xxx_fsqf_ring0_isr(int irq, void *dev_id)
{
#ifndef CONFIG_CNS3XXX_NAPI
	struct net_device *netdev = dev_id;
	struct CNS3XXXPrivate *priv = netdev_priv(netdev);
#endif

#ifdef CONFIG_CNS3XXX_NAPI
{
	struct CNS3XXXPrivate *priv = netdev_priv(napi_dev);
	/* because in normal state,
	 * fsqf only invoke once and set_bit is atomic function.
	 * so don't mask it.
	 */
	set_bit(0, &priv->is_qf);
}
#else
	cns3xxx_disable_irq(FSRC_RING0_INTERRUPT_ID);
	cns3xxx_disable_irq(FSQF_RING0_INTERRUPT_ID);

	cns3xxx_receive_packet(priv, 1); /* Receive at Queue Full Mode*/

	cns3xxx_enable_irq(FSRC_RING0_INTERRUPT_ID);
	cns3xxx_enable_irq(FSQF_RING0_INTERRUPT_ID);

	enable_rx_dma(0, 1);
#endif
	return IRQ_HANDLED;
}
#endif

#if defined(CNS3XXX_DOUBLE_RX_RING)
#ifdef CNS3XXX_FSQF_RING1_ISR
irqreturn_t cns3xxx_fsqf_ring1_isr(int irq, void *dev_id)
{
	struct net_device *netdev = dev_id;
	struct CNS3XXXPrivate *priv = netdev_priv(netdev);

#ifdef CONFIG_CNS3XXX_NAPI
{
	struct CNS3XXXPrivate *priv = netdev_priv(r1_napi_dev);
	/* because in normal state,
	 * fsqf only invoke once and set_bit is atomic function.
	 * so don't mask it.
	 */
	set_bit(0, &priv->is_qf);
}
#else
	cns3xxx_disable_irq(FSRC_RING1_INTERRUPT_ID);
	cns3xxx_disable_irq(FSQF_RING1_INTERRUPT_ID);

	cns3xxx_receive_packet(priv, 1); /* Receive at Queue Full Mode*/
	enable_rx_dma(1, 1);

	cns3xxx_enable_irq(FSRC_RING1_INTERRUPT_ID);
	cns3xxx_enable_irq(FSQF_RING1_INTERRUPT_ID);
#endif
	return IRQ_HANDLED;
}
#endif
#endif


#ifdef CNS3XXX_STATUS_ISR
irqreturn_t cns3xxx_status_isr(int irq, void *dev_id)
{
	u32 int_status = INTR_STAT_REG;
	u32 i = 0;

	cns3xxx_disable_irq(STATUS_INTERRUPT_ID);
#ifdef CNS3XXX_SHOW_LINK_STATUS
	for (i = 14; i <= 16 ; ++i) {
		u32 cfg = 0;
		u8 mac_cfg[] = {0xc, 0x10, 0x18};
		u32 reg;

		if (((int_status >> i) & 1)) {
			if ((SWITCH_REG_VALUE(mac_cfg[i-14]) & 1)) {
				/* GMII2 high speed drive strength*/
				reg = __raw_readl(MISC_IO_PAD_DRIVE_STRENGTH_CTRL_A);
				reg &= (~(3 << 10));

				if (((SWITCH_REG_VALUE(mac_cfg[i-14]) >> 2)
						&0x3) == 2)
					reg |= (2 << 10);/* 1000 Mbps*/
				__raw_writel(MISC_IO_PAD_DRIVE_STRENGTH_CTRL_A);
			}
		}
	}
#else
	for (i = 0; i < 32; i++) {
		if (int_status & (1 << i))
			PRINT_INFO(cns3xxx_gsw_status_tbl[i]);
	}
#endif
	INTR_STAT_REG = 0xffffffff;
	cns3xxx_enable_irq(STATUS_INTERRUPT_ID);
	return IRQ_HANDLED;
}
#endif


#ifdef CNS3XXX_TSTC_RING0_ISR
irqreturn_t cns3xxx_tstc_ring0_isr(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}
#endif

static int cns3xxx_install_isr(struct net_device *dev)
{
	int retval;
	struct CNS3XXXPrivate *priv = netdev_priv(dev);

	if (install_isr_rc == 0) {
		printk(KERN_INFO
			"request FSRC_RING0_INTERRUPT_ID %d\n",
			FSRC_RING0_INTERRUPT_ID);
		retval = request_irq(FSRC_RING0_INTERRUPT_ID,
			cns3xxx_fsrc_ring0_isr, IRQF_SHARED,
			"FSRC_RING0", intr_netdev);

		if (retval) {
			printk(KERN_ERR "%s: unable to get IRQ %d (irqval=%d).\n",
				"FSRC_RING0", FSRC_RING0_INTERRUPT_ID, retval);
			return 1;
		}

#ifdef CNS3XXX_FSQF_RING0_ISR
		printk(KERN_INFO "request FSQF_RING0_INTERRUPT_ID %d\n",
			FSQF_RING0_INTERRUPT_ID);
		retval = request_irq(FSQF_RING0_INTERRUPT_ID,
			cns3xxx_fsqf_ring0_isr, IRQF_SHARED,
			"FSQF_RING0", intr_netdev);

		if (retval) {
			PRINT_INFO("%s: unable to get IRQ %d (irqval=%d).\n",
				"FSQF_RING0", FSQF_RING0_INTERRUPT_ID, retval);
			return 2;
		}
#endif

#ifdef CNS3XXX_TSTC_RING0_ISR
		printk(KERN_INFO "request TSTC_RING0_INTERRUPT_ID %d\n",
			TSTC_RING0_INTERRUPT_ID);
		retval = request_irq(TSTC_RING0_INTERRUPT_ID,
			cns3xxx_tstc_ring0_isr, IRQF_SHARED,
			"TSTC_RING0", intr_netdev);

		if (retval) {
			PRINT_INFO("%s: unable to get IRQ %d (irqval=%d).\n",
				"TSTC_RING0", FSQF_RING0_INTERRUPT_ID, retval);
			return 3;
		}
#endif

	if (priv->num_rx_queues == 2) {
#if defined(CNS3XXX_DOUBLE_RX_RING)
		printk(KERN_INFO "request FSRC_RING1_INTERRUPT_ID %d\n",
		FSRC_RING1_INTERRUPT_ID);
		retval = request_irq(FSRC_RING1_INTERRUPT_ID,
			cns3xxx_fsrc_ring1_isr, IRQF_SHARED,
			"FSRC_RING1", intr_netdev);

		if (retval) {
			printk(KERN_INFO
				"%s: unable to get IRQ %d (irqval=%d).\n",
				"FSRC_RING1", FSRC_RING1_INTERRUPT_ID, retval);
			return 1;
		}

#ifdef CNS3XXX_FSQF_RING1_ISR
		printk(KERN_INFO "request FSQF_RING1_INTERRUPT_ID %d\n",
			FSQF_RING1_INTERRUPT_ID);
		retval = request_irq(FSQF_RING1_INTERRUPT_ID,
			cns3xxx_fsqf_ring1_isr, IRQF_SHARED,
			"FSQF_RING1", intr_netdev);

		if (retval) {
			PRINT_INFO("%s: unable to get IRQ %d (irqval=%d).\n",
				"FSQF_RING1", FSQF_RING1_INTERRUPT_ID, retval);
			return 2;
		}
#endif

#endif
	}

#ifdef CNS3XXX_STATUS_ISR
#ifdef CNS3XXX_SHOW_LINK_STATUS

	INTR_MASK_REG = 0xffffffff;
	/* only enable mac link status */
	INTR_MASK_REG &= (~(1 << 14));
	INTR_MASK_REG &= (~(1 << 15));
	INTR_MASK_REG &= (~(1 << 16));

#endif
	printk(KERN_ERR "request STATUS_INTERRUPT_ID %d\n",
		STATUS_INTERRUPT_ID);
	retval = request_irq(STATUS_INTERRUPT_ID,
		cns3xxx_status_isr, IRQF_SHARED, "GSW_STATUS", intr_netdev);

	if (retval) {
		PRINT_INFO("%s: unable to get IRQ %d (irqval=%d).\n",
			"GSW STATUS INT", STATUS_INTERRUPT_ID, retval);
		return 3;
	}
	INTR_MASK_REG = 0;
#endif

#ifdef CONFIG_CNS3XXX_NAPI
{
	struct CNS3XXXPrivate *sp = netdev_priv(napi_dev);
	napi_enable(&sp->napi);
	netif_start_queue(napi_dev);

#ifdef CNS3XXX_DOUBLE_RX_RING
	sp = netdev_priv(r1_napi_dev);
	napi_enable(&sp->napi);
	netif_start_queue(r1_napi_dev);
#endif
}
#endif
	/* enable cpu port */
	enable_port(3, 1);

	} /* end if (install_isr_rc == 0)*/

	++install_isr_rc;

	return 0;
}

int cns3xxx_open(struct net_device *dev)
{
	struct CNS3XXXPrivate *priv = netdev_priv(dev);
	if (cns3xxx_setup_rx_tx_res(priv) != CAVM_OK) {
		printk(KERN_ERR "alloc rx/tx ring fail!!\n");
		return -1;
	}

	netif_start_queue(dev);

	cns3xxx_install_isr(dev);

	enable_rx_dma(0, 1);

	if (priv->num_rx_queues == 2)
		enable_rx_dma(1, 1);

	netif_carrier_on(dev);
	priv->net_device_priv->open();

	return 0;
}

static int cns3xxx_uninstall_isr(struct net_device *dev)
{
	struct CNS3XXXPrivate *priv = netdev_priv(dev);
	--install_isr_rc;
	if (install_isr_rc == 0) {
		enable_port(3, 0);
		free_irq(FSRC_RING0_INTERRUPT_ID, intr_netdev);
#ifdef CNS3XXX_STATUS_ISR
		free_irq(STATUS_INTERRUPT_ID, intr_netdev);
#endif

#ifdef CNS3XXX_FSQF_RING0_ISR
		free_irq(FSQF_RING0_INTERRUPT_ID, intr_netdev);
#endif

#ifdef CNS3XXX_TSTC_RING0_ISR
		free_irq(TSTC_RING0_INTERRUPT_ID, intr_netdev);
#endif

	if (priv->num_rx_queues == 2) {
		free_irq(FSRC_RING1_INTERRUPT_ID, intr_netdev);

#ifdef CNS3XXX_FSQF_RING1_ISR
		free_irq(FSQF_RING1_INTERRUPT_ID, intr_netdev);
#endif
	}

#ifdef CONFIG_CNS3XXX_NAPI
{
	struct CNS3XXXPrivate *sp = netdev_priv(napi_dev);

	napi_disable(&sp->napi);
	netif_stop_queue(napi_dev);
#ifdef CNS3XXX_DOUBLE_RX_RING
	sp = netdev_priv(r1_napi_dev);

	napi_disable(&sp->napi);
	netif_stop_queue(r1_napi_dev);
#endif
}
#endif
	}

	return 0;
}

int cns3xxx_close(struct net_device *dev)
{
	struct CNS3XXXPrivate *priv = netdev_priv(dev);

	enable_rx_dma(0, 0);
	enable_tx_dma(0, 0);

	if (priv->num_rx_queues == 2)
		enable_tx_dma(1, 0);

	if (priv->num_tx_queues == 2)
		enable_rx_dma(1, 0);

	netif_stop_queue(dev);

	priv->net_device_priv->close();
	cns3xxx_uninstall_isr(dev);
	cns3xxx_free_rx_tx_res(priv);
	netif_carrier_off(dev);
	return 0;
}

void broadcast_storm_cfg(u8 port, u8 boradcast, u8 multicast, u8 unknown)
{
	switch (port) {
	case 0:
		(boradcast == 1) ?
			(MAC0_CFG_REG |= (1 << 30)) :
			(MAC0_CFG_REG &= (~(1 << 30))) ;
		(multicast == 1) ?
			(MAC0_CFG_REG |= (1 << 29)) :
			(MAC0_CFG_REG &= (~(1 << 29))) ;
		(unknown == 1) ?
			(MAC0_CFG_REG |= (1 << 28)) :
			(MAC0_CFG_REG &= (~(1 << 28))) ;
		break;
	case 1:
		(boradcast == 1) ?
			(MAC1_CFG_REG |= (1 << 30)) :
			(MAC1_CFG_REG &= (~(1 << 30))) ;
		(multicast == 1) ?
			(MAC1_CFG_REG |= (1 << 29)) :
			(MAC1_CFG_REG &= (~(1 << 29))) ;
		(unknown == 1) ?
			(MAC1_CFG_REG |= (1 << 28)) :
			(MAC1_CFG_REG &= (~(1 << 28))) ;
		break;
	case 2:
		(boradcast == 1) ?
			(MAC2_CFG_REG |= (1 << 30)) :
			(MAC2_CFG_REG &= (~(1 << 30))) ;
		(multicast == 1) ?
			(MAC2_CFG_REG |= (1 << 29)) :
			(MAC2_CFG_REG &= (~(1 << 29))) ;
		(unknown == 1) ?
			(MAC2_CFG_REG |= (1 << 28)) :
			(MAC2_CFG_REG &= (~(1 << 28))) ;
		break;
	}
}

void broadcast_storm_rate(u8 rate)
{
	TC_CTRL_REG &= (~(0xf << 24));
	TC_CTRL_REG |= (rate << 24);
}

static int cns3xxx_set_mac_addr(struct net_device *dev, void *p)
{
	struct CNS3XXXPrivate *priv = netdev_priv(dev);

	struct sockaddr *addr = p;

	spin_lock_irq(&priv->lock);

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	/* 1. delete old arl mac entry
	 * 2. add new arl mac entry
	 * 3. copy new mac to netdev field
	 */
	if (priv->net_device_priv->arl_table_entry) {
		cns3xxx_arl_table_invalid(
			priv->net_device_priv->arl_table_entry);
		memcpy(priv->net_device_priv->arl_table_entry->mac,
			addr->sa_data, dev->addr_len);
		cns3xxx_arl_table_add(priv->net_device_priv->arl_table_entry);
	}
	memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);

	spin_unlock_irq(&priv->lock);
	return 0;
}


int set_fc_rls(struct ifreq *ifr)
{
	struct CNS3XXXSARLEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data,
			sizeof(struct CNS3XXXSARLEntry)))
		return -EFAULT;
	FC_GLOB_THRS_REG &= (~(0x1ff << 16));
	FC_GLOB_THRS_REG |= (ctl.val << 16);
	return CAVM_OK;
}

int get_fc_rls(struct ifreq *ifr)
{
	struct CNS3XXXSARLEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data,
			sizeof(struct CNS3XXXSARLEntry)))
		return -EFAULT;

	ctl.val = ((FC_GLOB_THRS_REG >> 16) & 0x1ff);

	if (copy_to_user(ifr->ifr_data, &ctl,
			sizeof(struct CNS3XXXSARLEntry)))
		return -EFAULT;

	return CAVM_OK;
}

int set_fc_set(struct ifreq *ifr)
{
	struct CNS3XXXSARLEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data,
			sizeof(struct CNS3XXXSARLEntry)))
		return -EFAULT;
	FC_GLOB_THRS_REG &= (~0x1ff);
	FC_GLOB_THRS_REG |= ctl.val;
	return CAVM_OK;
}

int get_fc_set(struct ifreq *ifr)
{
	struct CNS3XXXSARLEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data,
			sizeof(struct CNS3XXXSARLEntry)))
		return -EFAULT;

	ctl.val = ((FC_GLOB_THRS_REG) & 0x1ff);

	if (copy_to_user(ifr->ifr_data, &ctl,
			sizeof(struct CNS3XXXSARLEntry)))
		return -EFAULT;

	return CAVM_OK;
}


int set_sarl_rls(struct ifreq *ifr)
{
	struct CNS3XXXSARLEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data,
			sizeof(struct CNS3XXXSARLEntry)))
		return -EFAULT;
	SARL_CTRL_REG &= (~(0x1ff << 12));
	SARL_CTRL_REG |= (ctl.val << 12);
	return CAVM_OK;
}

int get_sarl_rls(struct ifreq *ifr)
{
	struct CNS3XXXSARLEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data,
			sizeof(struct CNS3XXXSARLEntry)))
		return -EFAULT;

	ctl.val = ((SARL_CTRL_REG >> 12) & 0x1ff);

	if (copy_to_user(ifr->ifr_data, &ctl,
			sizeof(struct CNS3XXXSARLEntry)))
		return -EFAULT;
	return CAVM_OK;
}

int set_sarl_enable(struct ifreq *ifr)
{
	struct CNS3XXXSARLEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data,
			sizeof(struct CNS3XXXSARLEntry)))
		return -EFAULT;
	SARL_CTRL_REG &= (~(0x1 << 31));
	SARL_CTRL_REG |= (ctl.val << 31);
	return CAVM_OK;
}

int get_sarl_enable(struct ifreq *ifr)
{
	struct CNS3XXXSARLEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data,
			sizeof(struct CNS3XXXSARLEntry)))
		return -EFAULT;
	ctl.val = ((SARL_CTRL_REG >> 31) & 0x1);
	if (copy_to_user(ifr->ifr_data, &ctl,
			sizeof(struct CNS3XXXSARLEntry)))
		return -EFAULT;

	return CAVM_OK;
}
int set_sarl_set(struct ifreq *ifr)
{
	struct CNS3XXXSARLEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data,
			sizeof(struct CNS3XXXSARLEntry)))
		return -EFAULT;
	SARL_CTRL_REG &= (~0x1ff);
	SARL_CTRL_REG |= ctl.val;
	return CAVM_OK;
}

int get_sarl_set(struct ifreq *ifr)
{
	struct CNS3XXXSARLEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data,
			sizeof(struct CNS3XXXSARLEntry)))
		return -EFAULT;

	ctl.val = ((SARL_CTRL_REG) & 0x1ff);

	if (copy_to_user(ifr->ifr_data, &ctl,
			sizeof(struct CNS3XXXSARLEntry)))
		return -EFAULT;
	return CAVM_OK;
}

int set_sarl_oq(struct ifreq *ifr)
{
	struct CNS3XXXSARLEntry ctl;
	if (copy_from_user(&ctl, ifr->ifr_data,
			sizeof(struct CNS3XXXSARLEntry)))
		return -EFAULT;

	switch (ctl.gyr) {
	case 0: /* green*/
		SARL_OQ_GTH_REG &= (~(0xff << ctl.tc*8));
		SARL_OQ_GTH_REG |= (ctl.val << ctl.tc*8);
		break;
	case 1: /* yellow */
		SARL_OQ_YTH_REG &= (~(0xff << ctl.tc*8));
		SARL_OQ_YTH_REG |= (ctl.val << ctl.tc*8);
		break;
	case 2: /* red */
		SARL_OQ_RTH_REG &= (~(0xff << ctl.tc*8));
		SARL_OQ_RTH_REG |= (ctl.val << ctl.tc*8);
		break;
	}
	return CAVM_OK;
}

int get_sarl_oq(struct ifreq *ifr)
{
	struct CNS3XXXSARLEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data,
			sizeof(struct CNS3XXXSARLEntry)))
		return -EFAULT;

	switch (ctl.gyr) {
	case 0: /* green*/
		ctl.val = ((SARL_OQ_GTH_REG >> ctl.tc*8) & 0xff);
		break;
	case 1: /* yellow */
		ctl.val = ((SARL_OQ_YTH_REG >> ctl.tc*8) & 0xff);
		break;
	case 2: /* red */
		ctl.val = ((SARL_OQ_RTH_REG >> ctl.tc*8) & 0xff);
		break;
	}

	if (copy_to_user(ifr->ifr_data, &ctl,
			sizeof(struct CNS3XXXSARLEntry)))
		return -EFAULT;
	return CAVM_OK;
}

int set_queue_weight(struct ifreq *ifr)
{
	struct CNS3XXXQueueWeightEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data,
			sizeof(struct CNS3XXXQueueWeightEntry)))
		return -EFAULT;
	switch (ctl.which_port) {
	case 0:
		QUEUE_WEIGHT_SET(0, ctl)
		break;
	case 1:
		QUEUE_WEIGHT_SET(1, ctl)
		break;
	case 2:
		QUEUE_WEIGHT_SET(2, ctl)
		break;
	case 3: /* cpu port */
		CPU_PRI_CTRL_REG &= ~(0x3ffff);
		CPU_PRI_CTRL_REG |= (ctl.sch_mode << 16);
		CPU_PRI_CTRL_REG |= (ctl.q0_w);
		CPU_PRI_CTRL_REG |= (ctl.q1_w << 4);
		CPU_PRI_CTRL_REG |= (ctl.q2_w << 8);
		CPU_PRI_CTRL_REG |= (ctl.q3_w << 12);
		break;
	case 4: /* PPE port */
		HNAT_PRI_CTRL_REG &= ~(0x3ffff);
		HNAT_PRI_CTRL_REG |= (ctl.sch_mode << 16);
		HNAT_PRI_CTRL_REG |= (ctl.q0_w);
		HNAT_PRI_CTRL_REG |= (ctl.q1_w << 4);
		HNAT_PRI_CTRL_REG |= (ctl.q2_w << 8);
		HNAT_PRI_CTRL_REG |= (ctl.q3_w << 12);
		break;
	default:
		return -EFAULT;
	}
	return CAVM_OK;
}

int get_queue_weight(struct ifreq *ifr)
{
	struct CNS3XXXQueueWeightEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data,
			sizeof(struct CNS3XXXQueueWeightEntry)))
		return -EFAULT;

	switch (ctl.which_port)	{
	case 0:
		QUEUE_WEIGHT_GET(0, ctl)
		break;
	case 1:
		QUEUE_WEIGHT_GET(1, ctl)
		break;
	case 2:
		QUEUE_WEIGHT_GET(2, ctl)
		break;
	case 3:
		ctl.sch_mode = ((CPU_PRI_CTRL_REG >> 16) & 0x3);
		ctl.q0_w = ((CPU_PRI_CTRL_REG >> 0) & 0x7);
		ctl.q1_w = ((CPU_PRI_CTRL_REG >> 4) & 0x7);
		ctl.q2_w = ((CPU_PRI_CTRL_REG >> 8) & 0x7);
		ctl.q3_w = ((CPU_PRI_CTRL_REG >> 12) & 0x7);
		break;
	case 4:
		ctl.sch_mode = ((HNAT_PRI_CTRL_REG >> 16) & 0x3);
		ctl.q0_w = ((HNAT_PRI_CTRL_REG >> 0) & 0x7);
		ctl.q1_w = ((HNAT_PRI_CTRL_REG >> 4) & 0x7);
		ctl.q2_w = ((HNAT_PRI_CTRL_REG >> 8) & 0x7);
		ctl.q3_w = ((HNAT_PRI_CTRL_REG >> 12) & 0x7);
		break;
	}

	if (copy_to_user(ifr->ifr_data, &ctl,
			sizeof(struct CNS3XXXQueueWeightEntry)))
		return -EFAULT;

	return CAVM_OK;
}

int set_rate_limit(struct ifreq *ifr)
{
	struct CNS3XXXRateLimitEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data,
			sizeof(struct CNS3XXXRateLimitEntry)))
		return -EFAULT;
	switch (ctl.which_port)	{
	case 0:
		RATE_CTRL_REG &= (~(0x7f << 8));
		RATE_CTRL_REG |= (ctl.band_width << 8);
		RATE_CTRL_REG &= (~(0x3));
		RATE_CTRL_REG |= ctl.base_rate;
		break;
	case 1:
		RATE_CTRL_REG &= (~(0x7f << 16));
		RATE_CTRL_REG |= (ctl.band_width << 16);
		RATE_CTRL_REG &= (~(0x3 << 2));
		RATE_CTRL_REG |= (ctl.base_rate << 2);
		break;
	case 2:
		RATE_CTRL_REG &= (~(0x7f << 24));
		RATE_CTRL_REG |= (ctl.band_width << 24);
		RATE_CTRL_REG &= (~(0x3 << 4));
		RATE_CTRL_REG |= (ctl.base_rate << 4);
		break;
	case 3: /* port 0 extra dma*/
		TC_CTRL_REG &= (~0x7f);
		TC_CTRL_REG |= ctl.band_width;
		RATE_CTRL_REG &= (~(0x3 << 6));
		RATE_CTRL_REG |= (ctl.base_rate << 6);
		break;
	default:
		return -EFAULT;
	}
	return CAVM_OK;
}

int get_rate_limit(struct ifreq *ifr)
{
	struct CNS3XXXRateLimitEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data,
			sizeof(struct CNS3XXXRateLimitEntry)))
		return -EFAULT;
	switch (ctl.which_port) {
	case 0:
		ctl.band_width = (RATE_CTRL_REG >> 8) & 0x7f;
		ctl.base_rate = RATE_CTRL_REG & 0x3;
		break;
	case 1:
		ctl.band_width = (RATE_CTRL_REG >> 16) & 0x7f;
		ctl.base_rate = (RATE_CTRL_REG >> 2) & 0x3;
		break;
	case 2:
		ctl.band_width = (RATE_CTRL_REG >> 24) & 0x7f;
		ctl.base_rate = (RATE_CTRL_REG >> 4) & 0x3;
		break;
	case 3: /* port 0 extra dma*/
		ctl.band_width = (TC_CTRL_REG) & 0x7f;
		ctl.base_rate = (RATE_CTRL_REG >> 6) & 0x3;
		break;
	default:
		return -EFAULT;
	}

	if (copy_to_user(ifr->ifr_data, &ctl,
			sizeof(struct CNS3XXXRateLimitEntry)))
		return -EFAULT;

	return CAVM_OK;
}

int set_fc(struct ifreq *ifr)
{
	struct CNS3XXXFCEntry ctl;
	u32 port_offset[] = {0x0c, 0x10, 0x18, 0x14};
		/* 0x14 is cpu port offset */
	u32 val = 0;

	if (copy_from_user(&ctl, ifr->ifr_data,
			sizeof(struct CNS3XXXFCEntry)))
		return -EFAULT;

	val = SWITCH_REG_VALUE(port_offset[ctl.port]);
	if (ctl.port == 3) { /* cpu port, only can set rx fc */
		val &= (~(1 << 31));
		if (ctl.fc_en)
			val |= (1 << 31);
	} else {
		val &= (~(1 << 11)); /* disable rx fc */
		val &= (~(1 << 12)); /* disable tx fc */
		val |= (ctl.fc_en << 11);
	}

	SWITCH_REG_VALUE(port_offset[ctl.port]) = val;
	return CAVM_OK;
}

int get_fc(struct ifreq *ifr)
{
	struct CNS3XXXFCEntry ctl;
	u32 port_offset[] = {0x0c, 0x10, 0x18, 0x14};
		/* 0x14 is cpu port offset */
	u32 val = 0;

	if (copy_from_user(&ctl, ifr->ifr_data,
			sizeof(struct CNS3XXXFCEntry)))
		return -EFAULT;

	val = SWITCH_REG_VALUE(port_offset[ctl.port]);
	printk(KERN_INFO "port_offset[%d]: %x\n",
			ctl.port, port_offset[ctl.port]);
	printk(KERN_INFO "val: %x\n", val);
	if (ctl.port == 3) /* cpu port, only can set rx fc */
		ctl.fc_en = ((val >> 31) & 1);
	else
		ctl.fc_en = ((val >> 11) & 3);

	if (copy_to_user(ifr->ifr_data, &ctl,
			sizeof(struct CNS3XXXFCEntry)))
		return -EFAULT;

	return CAVM_OK;
}

int set_ivl(struct ifreq *ifr)
{
	struct CNS3XXXIVLEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data,
			sizeof(struct CNS3XXXIVLEntry)))
		return -EFAULT;

	cns3xxx_ivl(ctl.enable);

	return CAVM_OK;
}

int get_ivl(struct ifreq *ifr)
{
	struct CNS3XXXIVLEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data,
			sizeof(struct CNS3XXXIVLEntry)))
		return -EFAULT;

	ctl.enable = ((MAC_GLOB_CFG_REG >> 7) & 0x1);

	if (copy_to_user(ifr->ifr_data, &ctl,
			sizeof(struct CNS3XXXIVLEntry)))
		return -EFAULT;

	return CAVM_OK;
}

int set_wan_port(struct ifreq *ifr)
{
	struct CNS3XXXWANPortEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data,
			sizeof(struct CNS3XXXWANPortEntry)))
		return -EFAULT;
	VLAN_CFG &= (~(0x1f << 8));
	VLAN_CFG |= (ctl.wan_port << 8);

	return CAVM_OK;
}
int get_wan_port(struct ifreq *ifr)
{
	struct CNS3XXXWANPortEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data,
			sizeof(struct CNS3XXXWANPortEntry)))
		return -EFAULT;

	ctl.wan_port = ((VLAN_CFG >> 8) & 0x1f);

	if (copy_to_user(ifr->ifr_data, &ctl,
			sizeof(struct CNS3XXXWANPortEntry)))
		return -EFAULT;

	return CAVM_OK;
}

int set_pvid(struct ifreq *ifr)
{
	struct CNS3XXXPVIDEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data,
			sizeof(struct CNS3XXXPVIDEntry)))
		return -EFAULT;
	cns3xxx_set_pvid(ctl.which_port, ctl.pvid);

	return CAVM_OK;
}

int get_pvid(struct ifreq *ifr)
{
	struct CNS3XXXPVIDEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data,
			sizeof(struct CNS3XXXPVIDEntry)))
		return -EFAULT;

	ctl.pvid = cns3xxx_get_pvid(ctl.which_port);
	if (copy_to_user(ifr->ifr_data, &ctl,
			sizeof(struct CNS3XXXPVIDEntry)))
		return -EFAULT;
	return CAVM_OK;
}

int set_qa(struct ifreq *ifr)
{
	struct CNS3XXXQAEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data,
			sizeof(struct CNS3XXXQAEntry)))
		return -EFAULT;

	MAC_GLOB_CFG_EXT_REG &= ~(0x7 << 27);
	MAC_GLOB_CFG_EXT_REG |= (ctl.qa << 27);

	return CAVM_OK;
}

int get_qa(struct ifreq *ifr)
{
	struct CNS3XXXQAEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data,
			sizeof(struct CNS3XXXQAEntry)))
		return -EFAULT;

	ctl.qa = (MAC_GLOB_CFG_EXT_REG >> 27) & 0x7;

	if (copy_to_user(ifr->ifr_data, &ctl,
			sizeof(struct CNS3XXXQAEntry)))
		return -EFAULT;
	return CAVM_OK;
}

int get_packet_max_len(struct ifreq *ifr)
{
	struct CNS3XXXMaxLenEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data,
			sizeof(struct CNS3XXXMaxLenEntry)))
		return -EFAULT;

	ctl.max_len = (PHY_AUTO_ADDR_REG >> 30) & 0x3;

	if (copy_to_user(ifr->ifr_data, &ctl,
			sizeof(struct CNS3XXXMaxLenEntry)))
		return -EFAULT;
	return CAVM_OK;
}

int set_packet_max_len(struct ifreq *ifr)
{
	struct CNS3XXXMaxLenEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data,
			sizeof(struct CNS3XXXMaxLenEntry)))
		return -EFAULT;

	PHY_AUTO_ADDR_REG &= (~(3 << 30));
	PHY_AUTO_ADDR_REG |= (ctl.max_len << 30);

	return CAVM_OK;
}

int set_udp_range(struct ifreq *ifr)
{
	struct CNS3XXXUdpRangeEtypeControl conf;

	if (copy_from_user(&conf, ifr->ifr_data,
			sizeof(struct CNS3XXXUdpRangeEtypeControl)))
		return -EFAULT;

	switch (conf.udp_range_num) {
	case 0:
		UDP_RANGE0_REG = 0;
		UDP_RANGE0_REG |= conf.port_start;
		UDP_RANGE0_REG |= (conf.port_end << 16);

		PRIO_ETYPE_UDP_REG &= (~(7 << 16));
		PRIO_ETYPE_UDP_REG |= (conf.pri << 16);
		break;
	case 1:
		UDP_RANGE1_REG = 0;
		UDP_RANGE1_REG |= conf.port_start;
		UDP_RANGE1_REG |= (conf.port_end << 16);

		PRIO_ETYPE_UDP_REG &= (~(7 << 20));
		PRIO_ETYPE_UDP_REG |= (conf.pri << 20);
		break;
	case 2:
		UDP_RANGE2_REG = 0;
		UDP_RANGE2_REG |= conf.port_start;
		UDP_RANGE2_REG |= (conf.port_end << 16);

		PRIO_ETYPE_UDP_REG &= (~(7 << 24));
		PRIO_ETYPE_UDP_REG |= (conf.pri << 24);
		break;
	case 3:
		UDP_RANGE3_REG = 0;
		UDP_RANGE3_REG |= conf.port_start;
		UDP_RANGE3_REG |= (conf.port_end << 16);

		PRIO_ETYPE_UDP_REG &= (~(7 << 28));
		PRIO_ETYPE_UDP_REG |= (conf.pri << 28);
		break;
	}

	return CAVM_OK;
}

int get_udp_range(struct ifreq *ifr)
{
	struct CNS3XXXUdpRangeEtypeControl conf;

	if (copy_from_user(&conf, ifr->ifr_data,
			sizeof(struct CNS3XXXUdpRangeEtypeControl)))
		return -EFAULT;

	switch (conf.udp_range_num)	{
	case 0:
		conf.port_start = (UDP_RANGE0_REG & 0xffff);
		conf.port_end = ((UDP_RANGE0_REG >> 16) & 0xffff);
		conf.pri = ((PRIO_ETYPE_UDP_REG >> 16) & 0x7);
		break;
	case 1:
		conf.port_start = (UDP_RANGE1_REG & 0xffff);
		conf.port_end = ((UDP_RANGE1_REG >> 16) & 0xffff);
		conf.pri = ((PRIO_ETYPE_UDP_REG >> 20) & 0x7);
		break;
	case 2:
		conf.port_start = (UDP_RANGE2_REG & 0xffff);
		conf.port_end = ((UDP_RANGE2_REG >> 16) & 0xffff);
		conf.pri = ((PRIO_ETYPE_UDP_REG >> 24) & 0x7);
		break;
	case 3:
		conf.port_start = (UDP_RANGE3_REG & 0xffff);
		conf.port_end = ((UDP_RANGE3_REG >> 16) & 0xffff);
		conf.pri = ((PRIO_ETYPE_UDP_REG >> 28) & 0x7);
		break;
	}

	if (copy_to_user(ifr->ifr_data, &conf,
			sizeof(struct CNS3XXXUdpRangeEtypeControl)))
		return -EFAULT;

	return CAVM_OK;
}

int get_etype(struct ifreq *ifr)
{
	struct CNS3XXXEtypeControl conf;

	if (copy_from_user(&conf, ifr->ifr_data,
			sizeof(struct CNS3XXXEtypeControl)))
		return -EFAULT;
	switch (conf.etype_num)	{
	case 0:
		conf.val = (ETYPE1_ETYPE0_REG & 0xffff);
		conf.pri = (PRIO_ETYPE_UDP_REG & 0x7);
		break;
	case 1:
		conf.val = ((ETYPE1_ETYPE0_REG >> 16) & 0xffff);
		conf.pri = ((PRIO_ETYPE_UDP_REG >> 4) & 0x7);
		break;
	case 2:
		conf.val = (ETYPE3_ETYPE2_REG & 0xffff);
		conf.pri = ((PRIO_ETYPE_UDP_REG >> 8) & 0x7);
		break;
	case 3:
		conf.val = ((ETYPE3_ETYPE2_REG >> 16) & 0xffff);
		conf.pri = ((PRIO_ETYPE_UDP_REG >> 12) & 0x7);
		break;
	}
	if (copy_to_user(ifr->ifr_data, &conf,
			sizeof(struct CNS3XXXEtypeControl)))
		return -EFAULT;

	return CAVM_OK;
}

int set_etype(struct ifreq *ifr)
{
	struct CNS3XXXEtypeControl conf;

	if (copy_from_user(&conf, ifr->ifr_data,
			sizeof(struct CNS3XXXEtypeControl)))
		return -EFAULT;

	switch (conf.etype_num) {
	case 0:
		ETYPE1_ETYPE0_REG &= (~0xffff);
		ETYPE1_ETYPE0_REG |= conf.val;

		PRIO_ETYPE_UDP_REG &= (~7);
		PRIO_ETYPE_UDP_REG |= (conf.pri);
		break;
	case 1:
		ETYPE1_ETYPE0_REG &= (~(0xffff << 16));
		ETYPE1_ETYPE0_REG |= (conf.val << 16);

		PRIO_ETYPE_UDP_REG &= (~(7 << 4));
		PRIO_ETYPE_UDP_REG |= (conf.pri << 4);
		break;
	case 2:
		ETYPE3_ETYPE2_REG &= (~0xffff);
		ETYPE3_ETYPE2_REG |= conf.val;

		PRIO_ETYPE_UDP_REG &= (~(7 << 8));
		PRIO_ETYPE_UDP_REG |= (conf.pri << 8);
		break;
	case 3:
		ETYPE3_ETYPE2_REG &= (~(0xffff << 16));
		ETYPE3_ETYPE2_REG |= (conf.val << 16);

		PRIO_ETYPE_UDP_REG &= (~(7 << 12));
		PRIO_ETYPE_UDP_REG |= (conf.pri << 12);
		break;
	}
	return CAVM_OK;
}

int get_pri_ip_dscp(struct ifreq *ifr)
{
	struct CNS3XXXPriIpDscpControl conf;

	if (copy_from_user(&conf, ifr->ifr_data,
			sizeof(struct CNS3XXXPriIpDscpControl)))
		return -EFAULT;

	if (0 <= conf.ip_dscp_num && conf.ip_dscp_num <= 7)
		conf.pri =
			(PRIO_IPDSCP_7_0_REG >> (conf.ip_dscp_num * 4))
			& 0x7;
	else if (8 <= conf.ip_dscp_num && conf.ip_dscp_num <= 15)
		conf.pri =
			(PRIO_IPDSCP_15_8_REG >> ((conf.ip_dscp_num-8) * 4))
			& 0x7;
	else if (16 <= conf.ip_dscp_num && conf.ip_dscp_num <= 23)
		conf.pri =
			(PRIO_IPDSCP_23_16_REG >> ((conf.ip_dscp_num-16) * 4))
			& 0x7;
	else if (24 <= conf.ip_dscp_num && conf.ip_dscp_num <= 31)
		conf.pri =
			(PRIO_IPDSCP_31_24_REG >> ((conf.ip_dscp_num-24) * 4))
			& 0x7;
	else if (32 <= conf.ip_dscp_num && conf.ip_dscp_num <= 39)
		conf.pri =
			(PRIO_IPDSCP_39_32_REG >> ((conf.ip_dscp_num-32) * 4))
			& 0x7;
	else if (40 <= conf.ip_dscp_num && conf.ip_dscp_num <= 47)
		conf.pri =
			(PRIO_IPDSCP_47_40_REG >> ((conf.ip_dscp_num-40) * 4))
			& 0x7;
	else if (48 <= conf.ip_dscp_num && conf.ip_dscp_num <= 55)
		conf.pri =
			(PRIO_IPDSCP_55_48_REG >> ((conf.ip_dscp_num-48) * 4))
			& 0x7;
	else if (56 <= conf.ip_dscp_num && conf.ip_dscp_num <= 63)
		conf.pri =
			(PRIO_IPDSCP_63_56_REG >> ((conf.ip_dscp_num-56) * 4))
			& 0x7;
	else
		return CAVM_ERR;

	if (copy_to_user(ifr->ifr_data, &conf,
			sizeof(struct CNS3XXXPriIpDscpControl)))
		return -EFAULT;
	return CAVM_OK;
}

int set_pri_ip_dscp(struct ifreq *ifr)
{
	struct CNS3XXXPriIpDscpControl conf;

	if (copy_from_user(&conf, ifr->ifr_data,
			sizeof(struct CNS3XXXPriIpDscpControl)))
		return -EFAULT;

	if (0 <= conf.ip_dscp_num && conf.ip_dscp_num <= 7) {
		PRIO_IPDSCP_7_0_REG &=
			(~(0x7 << (conf.ip_dscp_num * 4)));
		PRIO_IPDSCP_7_0_REG |=
			(conf.pri << (conf.ip_dscp_num * 4));
	} else if (8 <= conf.ip_dscp_num && conf.ip_dscp_num <= 15) {
		PRIO_IPDSCP_15_8_REG &=
			(~(0x7 << ((conf.ip_dscp_num-8) * 4)));
		PRIO_IPDSCP_15_8_REG |=
			(conf.pri << ((conf.ip_dscp_num-8) * 4));
	} else if (16 <= conf.ip_dscp_num && conf.ip_dscp_num <= 23) {
		PRIO_IPDSCP_23_16_REG &=
			(~(0x7 << ((conf.ip_dscp_num-16) * 4)));
		PRIO_IPDSCP_23_16_REG |=
			(conf.pri << ((conf.ip_dscp_num-16) * 4));
	} else if (24 <= conf.ip_dscp_num && conf.ip_dscp_num <= 31) {
		PRIO_IPDSCP_31_24_REG &=
			(~(0x7 << ((conf.ip_dscp_num-24) * 4)));
		PRIO_IPDSCP_31_24_REG |=
			(conf.pri << ((conf.ip_dscp_num-24) * 4));
	} else if (32 <= conf.ip_dscp_num && conf.ip_dscp_num <= 39) {
		PRIO_IPDSCP_39_32_REG &=
			(~(0x7 << ((conf.ip_dscp_num-32) * 4)));
		PRIO_IPDSCP_39_32_REG |=
			(conf.pri << ((conf.ip_dscp_num-32) * 4));
	} else if (40 <= conf.ip_dscp_num && conf.ip_dscp_num <= 47) {
		PRIO_IPDSCP_47_40_REG &=
			(~(0x7 << ((conf.ip_dscp_num-40) * 4)));
		PRIO_IPDSCP_47_40_REG |=
			(conf.pri << ((conf.ip_dscp_num-40) * 4));
	} else if (48 <= conf.ip_dscp_num && conf.ip_dscp_num <= 55) {
		PRIO_IPDSCP_55_48_REG &=
			(~(0x7 << ((conf.ip_dscp_num-48) * 4)));
		PRIO_IPDSCP_55_48_REG |=
			(conf.pri << ((conf.ip_dscp_num-48) * 4));
	} else if (56 <= conf.ip_dscp_num && conf.ip_dscp_num <= 63) {
		PRIO_IPDSCP_63_56_REG &=
			(~(0x7 << ((conf.ip_dscp_num-56) * 4)));
		PRIO_IPDSCP_63_56_REG |=
			(conf.pri << ((conf.ip_dscp_num-56) * 4));
	} else
		return CAVM_ERR;

	return CAVM_OK;
}

#if defined(CONFIG_CNS3XXX_GSW_VB)
extern int bcm53115M_reg_read(int page, int offset, u8 *buf, int len);
extern int bcm53115M_reg_write(int page, int offset, u8 *buf, int len);

int bcm53115M_reg_read_ioctl(struct ifreq *ifr)
{
	struct CNS3XXXBCM53115M conf;

	if (copy_from_user(&conf, ifr->ifr_data,
			sizeof(struct CNS3XXXBCM53115M)))
		return -EFAULT;

	printk(KERN_INFO "conf.page: %x\n", conf.page);
	printk(KERN_INFO "conf.offset: %x\n", conf.offset);
	printk(KERN_INFO "conf.data_len: %x\n", conf.data_len);
	switch (conf.data_len) {
	case 1:
		bcm53115M_reg_read(conf.page, conf.offset,
			(u8 *)&conf.u8_val, 1);
		printk(KERN_INFO "conf.u8_val: %x\n", conf.u8_val);
		break;
	case 2:
		bcm53115M_reg_read(conf.page, conf.offset,
			(u8 *)&conf.u16_val, 2);
		printk(KERN_INFO "conf.u16_val: %x\n", conf.u16_val);
		break;
	case 4:
		bcm53115M_reg_read(conf.page, conf.offset,
			(u8 *)&conf.u32_val, 4);
		printk(KERN_INFO "conf.u32_val: %x\n", conf.u32_val);
		break;
	default:
		printk(KERN_ERR
			"[kernel mode]: don't support date length: %d\n",
			conf.data_len);
		break;
	}

	if (copy_to_user(ifr->ifr_data, &conf,
			sizeof(struct CNS3XXXBCM53115M)))
		return -EFAULT;
	return CAVM_OK;
}

int bcm53115M_reg_write_ioctl(struct ifreq *ifr)
{
	struct CNS3XXXBCM53115M conf;

	if (copy_from_user(&conf, ifr->ifr_data,
			sizeof(struct CNS3XXXBCM53115M)))
		return -EFAULT;

	switch (conf.data_len) {
	case 1:
		bcm53115M_reg_write(conf.page, conf.offset,
			(u8 *)&conf.u8_val, 1);
		break;
	case 2:
		bcm53115M_reg_write(conf.page, conf.offset,
			(u8 *)&conf.u16_val, 2);
		break;
	case 4:
		bcm53115M_reg_write(conf.page, conf.offset,
			(u8 *)&conf.u32_val, 4);
		break;
	default:
		printk(KERN_ERR
			"[kernel mode]: don't support date length: %d\n",
			conf.data_len);
		break;
	}
	return CAVM_OK;
}
#endif

int dump_mib_counter(struct ifreq *ifr)
{
	struct CNS3XXXMIBCounter conf;
	int addr = 0, i = 0;

	if (copy_from_user(&conf, ifr->ifr_data,
			sizeof(struct CNS3XXXMIBCounter)))
		return -EFAULT;

	for (addr = 0x300; addr <= 0x334 ; addr += 4)
		conf.mib[i++] = SWITCH_REG_VALUE(addr);
	for (addr = 0x400; addr <= 0x434 ; addr += 4)
		conf.mib[i++] = SWITCH_REG_VALUE(addr);
	for (addr = 0x600; addr <= 0x634 ; addr += 4)
		conf.mib[i++] = SWITCH_REG_VALUE(addr);
	/* cpu mib counter */
	for (addr = 0x500; addr <= 0x528 ; addr += 4)
		conf.mib[i++] = SWITCH_REG_VALUE(addr);
	conf.mib_len = i;
	if (copy_to_user(ifr->ifr_data, &conf,
			sizeof(struct CNS3XXXMIBCounter)))
		return -EFAULT;
	return 0;
}

int cns3xxx_do_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd)
{
	enum CNS3XXXIoctlCmd ioctl_cmd;

	if (cmd != SIOCDEVPRIVATE)
		return -EOPNOTSUPP;

	if (copy_from_user(&ioctl_cmd, ifr->ifr_data,
			sizeof(enum CNS3XXXIoctlCmd)))
		return -EFAULT;

	switch (ioctl_cmd) {
	case CNS3XXX_ARP_REQUEST_SET:
	{
		struct CNS3XXXArpRequestControl ctl;
		if (copy_from_user(&ctl, ifr->ifr_data,
				sizeof(struct CNS3XXXArpRequestControl)))
			return -EFAULT;
		(ctl.val == 0) ?
			(MAC_GLOB_CFG_REG &= (~(1 << 23))) :
			(MAC_GLOB_CFG_REG |= (1 << 23));
		return CAVM_OK;
	}
	case CNS3XXX_ARP_REQUEST_GET:
	{
		struct CNS3XXXArpRequestControl ctl;
		if (copy_from_user(&ctl, ifr->ifr_data,
				sizeof(struct CNS3XXXArpRequestControl)))
			return -EFAULT;
		ctl.val = ((MAC_GLOB_CFG_REG >> 23) & 1);

		if (copy_to_user(ifr->ifr_data, &ctl,
			sizeof(struct CNS3XXXArpRequestControl)))
			return -EFAULT;
		return CAVM_OK;
	}
	case CNS3XXX_HOL_PREVENT_SET:
	{
		struct CNS3XXXHOLPreventControl ctl;
		if (copy_from_user(&ctl, ifr->ifr_data,
				sizeof(struct CNS3XXXHOLPreventControl)))
			return -EFAULT;
		(ctl.enable == 1) ?
			(TC_CTRL_REG |= (1 << 29)) :
			(TC_CTRL_REG &= (~(1 << 29))) ;
		return CAVM_OK;
	}
	case CNS3XXX_HOL_PREVENT_GET:
	{
		struct CNS3XXXHOLPreventControl ctl;
		if (copy_from_user(&ctl, ifr->ifr_data,
				sizeof(struct CNS3XXXHOLPreventControl)))
			return -EFAULT;

		ctl.enable = ((TC_CTRL_REG >> 29) & 0x1);
		if (copy_to_user(ifr->ifr_data, &ctl,
				sizeof(struct CNS3XXXHOLPreventControl)))
			return -EFAULT;
		return CAVM_OK;
	}
	case CNS3XXX_BRIDGE_SET:
	{
		struct CNS3XXXBridgeControl ctl;

		if (copy_from_user(&ctl, ifr->ifr_data,
				sizeof(struct CNS3XXXBridgeControl)))
			return -EFAULT;
		(ctl.type == 1) ?
			(VLAN_CFG |= (1 << 1)) :
			(VLAN_CFG &= (~(1 << 1)));
		return CAVM_OK;
	}
	case CNS3XXX_BRIDGE_GET:
	{
		struct CNS3XXXBridgeControl ctl;
		ctl.type = ((VLAN_CFG >> 1) & 0x1);
		printk(KERN_INFO "[kernel mode] ctl.type: %d\n", ctl.type);

		if (copy_to_user(ifr->ifr_data, &ctl,
				sizeof(struct CNS3XXXBridgeControl)))
			return -EFAULT;
		return CAVM_OK;
	}
	case CNS3XXX_PORT_NEIGHBOR_SET:
	{
		struct CNS3XXXPortNeighborControl ctl;

		if (copy_from_user(&ctl, ifr->ifr_data,
				sizeof(struct CNS3XXXPortNeighborControl)))
			return -EFAULT;

		switch (ctl.which_port)	{
		case 0:
			(ctl.type == 1) ?
				(VLAN_CFG |= (1 << 4)) :
				(VLAN_CFG &= (~(1 << 4)));
			return CAVM_OK;
		case 1:
			(ctl.type == 1) ?
				(VLAN_CFG |= (1 << 5)) :
				(VLAN_CFG &= (~(1 << 5)));
			return CAVM_OK;
		case 2:
			(ctl.type == 1) ?
				(VLAN_CFG |= (1 << 7)) :
				(VLAN_CFG &= (~(1 << 7)));
			return CAVM_OK;
		case 3: /* cpu port */
			(ctl.type == 1) ?
				(VLAN_CFG |= (1 << 6)) :
				(VLAN_CFG &= (~(1 << 6)));
			return CAVM_OK;
		default:
			return -EFAULT;
		}
	}
	case CNS3XXX_PORT_NEIGHBOR_GET:
	{
		struct CNS3XXXPortNeighborControl ctl;

		if (copy_from_user(&ctl, ifr->ifr_data,
				sizeof(struct CNS3XXXPortNeighborControl)))
			return -EFAULT;
		switch (ctl.which_port) {
		case 0:
			ctl.type = ((VLAN_CFG >> 4) & 0x1);
			break;
		case 1:
			ctl.type = ((VLAN_CFG >> 5) & 0x1);
			break;
		case 2:
			ctl.type = ((VLAN_CFG >> 7) & 0x1);
			break;
		case 3: /* cpu port */
			ctl.type = ((VLAN_CFG >> 6) & 0x1);
			break;
		}
		if (copy_to_user(ifr->ifr_data, &ctl,
				sizeof(struct CNS3XXXPortNeighborControl)))
			return -EFAULT;

		return CAVM_OK;
	}
	case CNS3XXX_VLAN_TABLE_LOOKUP:
	{
		struct CNS3XXXVLANTableEntry ctl;

		if (copy_from_user(&ctl, ifr->ifr_data,
				sizeof(struct CNS3XXXVLANTableEntry)))
			return -EFAULT;
		if (cns3xxx_vlan_table_lookup(&ctl.entry) ==  CAVM_NOT_FOUND)
			return CAVM_NOT_FOUND;

		if (copy_to_user(ifr->ifr_data, &ctl,
				sizeof(struct CNS3XXXVLANTableEntry)))
			return -EFAULT;

		return CAVM_FOUND;
	}
	case CNS3XXX_VLAN_TABLE_READ:
	{
		struct CNS3XXXVLANTableEntry ctl;

		if (copy_from_user(&ctl, ifr->ifr_data,
				sizeof(struct CNS3XXXVLANTableEntry)))
			return -EFAULT;

		cns3xxx_vlan_table_read(&ctl.entry);
		if (copy_to_user(ifr->ifr_data, &ctl,
				sizeof(struct CNS3XXXVLANTableEntry)))
			return -EFAULT;
		return CAVM_OK;
	}
	case CNS3XXX_VLAN_TABLE_ADD:
	{
		struct CNS3XXXVLANTableEntry ctl;

		if (copy_from_user(&ctl, ifr->ifr_data,
				sizeof(struct CNS3XXXVLANTableEntry)))
			return -EFAULT;

		cns3xxx_vlan_table_add(&ctl.entry);

		if (copy_to_user(ifr->ifr_data, &ctl,
				sizeof(struct CNS3XXXVLANTableEntry)))
			return -EFAULT;

		return CAVM_OK;
	}
	case CNS3XXX_ARL_TABLE_ADD:
	{
		struct CNS3XXXARLTableEntry ctl;

		printk(KERN_INFO "[kernel mode] CNS3XXX_ARL_TABLE_ADD\n");

		if (copy_from_user(&ctl, ifr->ifr_data,
				sizeof(struct CNS3XXXARLTableEntry)))
			return -EFAULT;

		cns3xxx_arl_table_add(&ctl.entry);

		if (copy_to_user(ifr->ifr_data, &ctl,
				sizeof(struct CNS3XXXARLTableEntry)))
			return -EFAULT;
		return CAVM_OK;
	}
	case CNS3XXX_ARL_TABLE_DEL:
	{
		struct CNS3XXXARLTableEntry ctl;

		if (copy_from_user(&ctl, ifr->ifr_data,
				sizeof(struct CNS3XXXARLTableEntry)))
			return -EFAULT;

		cns3xxx_arl_table_invalid(&ctl.entry);

		if (copy_to_user(ifr->ifr_data, &ctl,
				sizeof(struct CNS3XXXARLTableEntry)))
			return -EFAULT;

		return CAVM_OK;
	}
	case CNS3XXX_VLAN_TABLE_DEL:
	{
		struct CNS3XXXARLTableEntry ctl;

		if (copy_from_user(&ctl, ifr->ifr_data,
				sizeof(struct CNS3XXXARLTableEntry)))
			return -EFAULT;

		cns3xxx_arl_table_invalid(&ctl.entry);

		if (copy_to_user(ifr->ifr_data, &ctl,
				sizeof(struct CNS3XXXARLTableEntry)))
			return -EFAULT;
		return CAVM_FOUND;
	}
	case CNS3XXX_ARL_TABLE_SEARCH:
	{
		struct CNS3XXXARLTableEntry ctl;

		if (copy_from_user(&ctl, ifr->ifr_data,
				sizeof(struct CNS3XXXARLTableEntry)))
			return -EFAULT;
		if (cns3xxx_arl_table_search(&ctl.entry) == CAVM_NOT_FOUND) {
			printk(KERN_INFO "[kernel mode] not found\n");
			return CAVM_NOT_FOUND;
		}
		if (copy_to_user(ifr->ifr_data, &ctl,
				sizeof(struct CNS3XXXARLTableEntry)))
			return -EFAULT;

		return CAVM_FOUND;
	}
	case CNS3XXX_ARL_IS_TABLE_END:
	{
		struct CNS3XXXARLTableEntry ctl;
		if (copy_from_user(&ctl, ifr->ifr_data,
				sizeof(struct CNS3XXXARLTableEntry)))
			return -EFAULT;
		if (cns3xxx_is_arl_table_end() == CAVM_ERR)
			return CAVM_ERR;

		if (copy_to_user(ifr->ifr_data, &ctl,
				sizeof(struct CNS3XXXARLTableEntry)))
			return -EFAULT;

		return CAVM_OK;
	}

	case CNS3XXX_ARL_TABLE_SEARCH_AGAIN:
	{
		struct CNS3XXXARLTableEntry ctl;
		if (copy_from_user(&ctl, ifr->ifr_data,
				sizeof(struct CNS3XXXARLTableEntry)))
			return -EFAULT;
		if (cns3xxx_arl_table_search_again(&ctl.entry) ==
				CAVM_NOT_FOUND)
			return CAVM_NOT_FOUND;
		if (copy_to_user(ifr->ifr_data, &ctl,
				sizeof(struct CNS3XXXARLTableEntry)))
			return -EFAULT;
		return CAVM_FOUND;
	}

	case CNS3XXX_ARL_TABLE_FLUSH:
	{
		struct CNS3XXXARLTableEntry ctl;

		if (copy_from_user(&ctl, ifr->ifr_data,
				sizeof(struct CNS3XXXARLTableEntry)))
			return -EFAULT;

		cns3xxx_arl_table_flush();

		if (copy_to_user(ifr->ifr_data, &ctl,
				sizeof(struct CNS3XXXARLTableEntry)))
			return -EFAULT;

		return CAVM_FOUND;
	}
	case CNS3XXX_ARL_TABLE_LOOKUP:
	{
		struct CNS3XXXARLTableEntry ctl;

		printk(KERN_INFO "[kernel mode] in CNS3XXX_ARL_TABLE_LOOKUP\n");
		if (copy_from_user(&ctl, ifr->ifr_data,
				sizeof(struct CNS3XXXARLTableEntry)))
			return -EFAULT;
		if (cns3xxx_arl_table_lookup(&ctl.entry) == CAVM_NOT_FOUND)
			return CAVM_NOT_FOUND;

		if (copy_to_user(ifr->ifr_data, &ctl,
				sizeof(struct CNS3XXXARLTableEntry)))
			return -EFAULT;
		return CAVM_FOUND;
	}
	case CNS3XXX_TC_SET:
	{
		struct CNS3XXXTrafficClassControl ctl;

		if (copy_from_user(&ctl, ifr->ifr_data,
				sizeof(struct CNS3XXXTrafficClassControl)))
			return -EFAULT;

		TC_CTRL_REG &= (~(0x3 << 30));
		TC_CTRL_REG |= (ctl.tc << 30);
		return CAVM_OK;
	}
	case CNS3XXX_TC_GET:
	{
		struct CNS3XXXTrafficClassControl ctl;

		if (copy_from_user(&ctl, ifr->ifr_data,
				sizeof(struct CNS3XXXTrafficClassControl)))
			return -EFAULT;

		ctl.tc = ((TC_CTRL_REG >> 30) & 0x3);

		if (copy_to_user(ifr->ifr_data, &ctl,
				sizeof(struct CNS3XXXTrafficClassControl)))
			return -EFAULT;
		return CAVM_OK;
	}

	case CNS3XXX_PRI_CTRL_SET:
	{
		struct CNS3XXXPriCtrlControl ctl;

		if (copy_from_user(&ctl, ifr->ifr_data,
				sizeof(struct CNS3XXXPriCtrlControl)))
			return -EFAULT;

		switch (ctl.which_port) {
		case 0:
			MAC0_PRI_CTRL_REG &= (~(0x7 << 24));
			MAC0_PRI_CTRL_REG &= (~(0xf << 18));

			MAC0_PRI_CTRL_REG |= (ctl.port_pri << 24);

			MAC0_PRI_CTRL_REG |= (ctl.ether_pri_en << 18);
			MAC0_PRI_CTRL_REG |= (ctl.vlan_pri_en << 19);
			MAC0_PRI_CTRL_REG |= (ctl.dscp_pri_en << 20);
			MAC0_PRI_CTRL_REG |= (ctl.udp_pri_en << 21);
			break;
		case 1:
			MAC1_PRI_CTRL_REG &= (~(0x7 << 24));
			MAC1_PRI_CTRL_REG &= (~(0xf << 18));

			MAC1_PRI_CTRL_REG |= (ctl.port_pri << 24);

			MAC1_PRI_CTRL_REG |= (ctl.ether_pri_en << 18);
			MAC1_PRI_CTRL_REG |= (ctl.vlan_pri_en << 19);
			MAC1_PRI_CTRL_REG |= (ctl.dscp_pri_en << 20);
			MAC1_PRI_CTRL_REG |= (ctl.udp_pri_en << 21);
			break;
		case 2:
			MAC2_PRI_CTRL_REG &= (~(0x7 << 24));
			MAC2_PRI_CTRL_REG &= (~(0xf << 18));

			MAC2_PRI_CTRL_REG |= (ctl.port_pri << 24);

			MAC2_PRI_CTRL_REG |= (ctl.ether_pri_en << 18);
			MAC2_PRI_CTRL_REG |= (ctl.vlan_pri_en << 19);
			MAC2_PRI_CTRL_REG |= (ctl.dscp_pri_en << 20);
			MAC2_PRI_CTRL_REG |= (ctl.udp_pri_en << 21);
			break;
		case 3: /* cpu */
			printk(KERN_INFO
				"[kernel mode] CPU_PRI_CTRL_REG: %#x\n",
				CPU_PRI_CTRL_REG);
			CPU_PRI_CTRL_REG &= (~(0x7 << 24));
			CPU_PRI_CTRL_REG &= (~(0xf << 18));

			CPU_PRI_CTRL_REG |= (ctl.port_pri << 24);

			CPU_PRI_CTRL_REG |= (ctl.ether_pri_en << 18);
			CPU_PRI_CTRL_REG |= (ctl.vlan_pri_en << 19);
			CPU_PRI_CTRL_REG |= (ctl.dscp_pri_en << 20);
			CPU_PRI_CTRL_REG |= (ctl.udp_pri_en << 21);
			break;
		}
		return CAVM_OK;
	}
	case CNS3XXX_PRI_CTRL_GET:
	{
		struct CNS3XXXPriCtrlControl ctl;

		if (copy_from_user(&ctl, ifr->ifr_data,
				sizeof(struct CNS3XXXPriCtrlControl)))
			return -EFAULT;
		switch (ctl.which_port) {
		case 0:
			ctl.port_pri = (unsigned char)
				((MAC0_PRI_CTRL_REG >> 24) & 0x7);
			ctl.ether_pri_en = (unsigned char)
				((MAC0_PRI_CTRL_REG >> 18) & 0x1);
			ctl.vlan_pri_en = (unsigned char)
				((MAC0_PRI_CTRL_REG >> 19) & 0x1);
			ctl.dscp_pri_en = (unsigned char)
				((MAC0_PRI_CTRL_REG >> 20) & 0x1);
			ctl.udp_pri_en = (unsigned char)
				((MAC0_PRI_CTRL_REG >> 21) & 0x1);
			break;
		case 1:
			ctl.port_pri = (unsigned char)
				((MAC1_PRI_CTRL_REG >> 24) & 0x7);
			ctl.ether_pri_en = (unsigned char)
				((MAC1_PRI_CTRL_REG >> 18) & 0x1);
			ctl.vlan_pri_en = (unsigned char)
				((MAC1_PRI_CTRL_REG >> 19) & 0x1);
			ctl.dscp_pri_en = (unsigned char)
				((MAC1_PRI_CTRL_REG >> 20) & 0x1);
			ctl.udp_pri_en = (unsigned char)
				((MAC1_PRI_CTRL_REG >> 21) & 0x1);
			break;
		case 2:
			ctl.port_pri = (unsigned char)
				((MAC2_PRI_CTRL_REG >> 24) & 0x7);
			ctl.ether_pri_en = (unsigned char)
				((MAC2_PRI_CTRL_REG >> 18) & 0x1);
			ctl.vlan_pri_en = (unsigned char)
				((MAC2_PRI_CTRL_REG >> 19) & 0x1);
			ctl.dscp_pri_en = (unsigned char)
				((MAC2_PRI_CTRL_REG >> 20) & 0x1);
			ctl.udp_pri_en = (unsigned char)
				((MAC2_PRI_CTRL_REG >> 21) & 0x1);
			break;
		case 3:
			ctl.port_pri =
				(unsigned char)((CPU_PRI_CTRL_REG >> 24) & 0x7);
			ctl.ether_pri_en =
				(unsigned char)((CPU_PRI_CTRL_REG >> 18) & 0x1);
			ctl.vlan_pri_en =
				(unsigned char)((CPU_PRI_CTRL_REG >> 19) & 0x1);
			ctl.dscp_pri_en =
				(unsigned char)((CPU_PRI_CTRL_REG >> 20) & 0x1);
			ctl.udp_pri_en =
				(unsigned char)((CPU_PRI_CTRL_REG >> 21) & 0x1);
			break;
		}

		if (copy_to_user(ifr->ifr_data, &ctl,
				sizeof(struct CNS3XXXPriCtrlControl)))
			return -EFAULT;

		return CAVM_OK;
	}
	case CNS3XXX_DMA_RING_CTRL_SET:
	{
		struct CNS3XXXDmaRingCtrlControl ctl;

		if (copy_from_user(&ctl, ifr->ifr_data,
				sizeof(struct CNS3XXXDmaRingCtrlControl)))
			return -EFAULT;

		(ctl.ts_double_ring_en == 0) ?
			DMA_RING_CTRL_REG &= (~(0x1 << 16)) :
			(DMA_RING_CTRL_REG |= (ctl.ts_double_ring_en << 16));
		(ctl.fs_double_ring_en == 0) ?
			DMA_RING_CTRL_REG &= (~(0x1 << 0)) :
			(DMA_RING_CTRL_REG |= (ctl.fs_double_ring_en << 0));
		(ctl.fs_pkt_allocate == 0) ?
			DMA_RING_CTRL_REG &= (~(0x1 << 1)) :
			(DMA_RING_CTRL_REG |= (ctl.fs_pkt_allocate << 1));
		return CAVM_OK;
	}
	case CNS3XXX_PRI_IP_DSCP_SET:
		return set_pri_ip_dscp(ifr);
	case CNS3XXX_PRI_IP_DSCP_GET:
		return get_pri_ip_dscp(ifr);
	case CNS3XXX_ETYPE_SET:
		return set_etype(ifr);
	case CNS3XXX_ETYPE_GET:
		return get_etype(ifr);
	case CNS3XXX_UDP_RANGE_SET:
		return set_udp_range(ifr);
	case CNS3XXX_UDP_RANGE_GET:
		return get_udp_range(ifr);
	case CNS3XXX_RATE_LIMIT_SET:
		return set_rate_limit(ifr);
	case CNS3XXX_RATE_LIMIT_GET:
		return get_rate_limit(ifr);
	case CNS3XXX_QUEUE_WEIGHT_SET:
		return set_queue_weight(ifr);
	case CNS3XXX_QUEUE_WEIGHT_GET:
		return get_queue_weight(ifr);
	case CNS3XXX_FC_RLS_SET:
		return set_fc_rls(ifr);
	case CNS3XXX_FC_RLS_GET:
		return get_fc_rls(ifr);
	case CNS3XXX_FC_SET_SET:
		return set_fc_set(ifr);
	case CNS3XXX_FC_SET_GET:
		return get_fc_set(ifr);
	case CNS3XXX_SARL_RLS_SET:
		return set_sarl_rls(ifr);
	case CNS3XXX_SARL_RLS_GET:
		return get_sarl_rls(ifr);
	case CNS3XXX_SARL_SET_SET:
		return set_sarl_set(ifr);
	case CNS3XXX_SARL_SET_GET:
		return get_sarl_set(ifr);
	case CNS3XXX_SARL_OQ_SET:
		return set_sarl_oq(ifr);
	case CNS3XXX_SARL_OQ_GET:
		return get_sarl_oq(ifr);
	case CNS3XXX_SARL_ENABLE_SET:
		return set_sarl_enable(ifr);
	case CNS3XXX_SARL_ENABLE_GET:
		return get_sarl_enable(ifr);
	case CNS3XXX_FC_SET:
		return set_fc(ifr);
	case CNS3XXX_FC_GET:
		return get_fc(ifr);
	case CNS3XXX_IVL_SET:
		return set_ivl(ifr);
	case CNS3XXX_IVL_GET:
		return get_ivl(ifr);
	case CNS3XXX_WAN_PORT_SET:
		return set_wan_port(ifr);
	case CNS3XXX_WAN_PORT_GET:
		return get_wan_port(ifr);
	case CNS3XXX_PVID_SET:
		return set_pvid(ifr);
	case CNS3XXX_PVID_GET:
		return get_pvid(ifr);
	case CNS3XXX_QA_GET:
		return get_qa(ifr);
	case CNS3XXX_QA_SET:
		return set_qa(ifr);
	case CNS3XXX_PACKET_MAX_LEN_GET:
		return get_packet_max_len(ifr);
	case CNS3XXX_PACKET_MAX_LEN_SET:
		return set_packet_max_len(ifr);
#if defined(CONFIG_CNS3XXX_GSW_VB)
	case CNS3XXX_BCM53115M_REG_READ:
		return bcm53115M_reg_read_ioctl(ifr);
	case CNS3XXX_BCM53115M_REG_WRITE:
		return bcm53115M_reg_write_ioctl(ifr);
#endif
	case CNS3XXX_DUMP_MIB_COUNTER:
		return dump_mib_counter(ifr);
	default:
		printk(KERN_ERR "[kernel mode] don't match any command\n");
		break;
	} /* end switch (ioctl_cmd)  */
	return 0;
}

#ifdef CONFIG_CNS3XXX_NAPI
static int cns3xxx_poll(struct napi_struct *napi, int budget)
{
	struct CNS3XXXPrivate *sp =
		container_of(napi, struct CNS3XXXPrivate, napi);
	int work_done = 0;
	int work_to_do = budget; /* define minima value */

	cns3xxx_receive_packet(sp, 0, &work_done, work_to_do);

	budget -= work_done;

    if (work_done) {
		if (test_bit(0, (unsigned long *)&sp->is_qf) == 1) {
			clear_bit(0, (unsigned long *)&sp->is_qf);
			enable_rx_dma(sp->ring_index, 1);
			return work_done;
	}
	} else {
		napi_complete(napi);

		if (sp->ring_index == 0)
			cns3xxx_enable_irq(FSRC_RING0_INTERRUPT_ID);
		else
			cns3xxx_enable_irq(FSRC_RING1_INTERRUPT_ID);
		return 0;
	}

	return work_done;
}
#endif

static struct net_device_stats *cns3xxx_get_stats(struct net_device *dev)
{
	struct CNS3XXXPrivate *priv = netdev_priv(dev);

	return &priv->stats;
}

static int cns3xxx_change_mtu(struct net_device *dev, int new_mtu)
{
	if (new_mtu < cns3xxx_min_mtu() || new_mtu > cns3xxx_max_mtu())
		return -EINVAL;

	dev->mtu = new_mtu;

	return 0;
}

static void cns3xxx_timeout(struct net_device *dev)
{
	printk(KERN_INFO "%s:cns3xxx gsw timeout\n", dev->name);
	netif_wake_queue(dev);
	dev->trans_start = jiffies;
}

#ifdef CONFIG_NET_POLL_CONTROLLER
/*
 * Polling 'interrupt' - used by things like netconsole to send skbs
 * without having to re-enable interrupts. It's not called while
 * the interrupt routine is executing.
 */
static void cns3xxx_netpoll(struct net_device *dev)
{
	disable_irq(FSRC_RING0_INTERRUPT_ID);
	cns3xxx_fsrc_ring0_isr(FSRC_RING0_INTERRUPT_ID, dev);
	enable_irq(FSRC_RING0_INTERRUPT_ID);

	disable_irq(FSQF_RING0_INTERRUPT_ID);
	cns3xxx_fsqf_ring0_isr(FSQF_RING0_INTERRUPT_ID, dev);
	enable_irq(FSQF_RING0_INTERRUPT_ID);
}
#endif

static const struct net_device_ops cns3xxx_netdev_ops = {
	.ndo_open               = cns3xxx_open,
	.ndo_stop               = cns3xxx_close,
	.ndo_start_xmit         = cns3xxx_send_packet,
	/*.ndo_validate_addr      = eth_validate_addr,
	.ndo_set_multicast_list = cns3xxx_set_multicast_list,*/
	.ndo_set_mac_address    = cns3xxx_set_mac_addr,
	.ndo_change_mtu         = cns3xxx_change_mtu,
	.ndo_do_ioctl           = cns3xxx_do_ioctl,
	.ndo_tx_timeout         = cns3xxx_timeout,
	.ndo_get_stats		= cns3xxx_get_stats,

#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller    = cns3xxx_netpoll,
#endif
};

extern void cns3xxx_set_ethtool_ops(struct net_device *netdev);

static int __init cns3xxx_probe(struct RingInfo ring_info)
{

	int netdev_size = num_net_dev_priv;
	int i = 0, err = 0;
	struct net_device *netdev;
	struct CNS3XXXPrivate *priv = NULL;
	struct sockaddr sock_addr;
#ifdef CONFIG_CNS3XXX_ETHADDR_IN_FLASH
    u8 mac_int[6];
    int val_len;
    int do_new_mac = 0;
    if (0 == init_mtd_env())
		do_new_mac = 1;
#endif

	for (i = 0; i < netdev_size; ++i) {
		netdev = alloc_etherdev(sizeof(struct CNS3XXXPrivate));
		if (!netdev) {
			err = -ENOMEM;
			goto err_alloc_etherdev;
		}
		if (net_device_prive[i].name)
			strcpy(netdev->name, net_device_prive[i].name);

		net_dev_array[net_device_prive[i].vlan_tag] = netdev;
		if (intr_netdev == 0)
			intr_netdev = netdev;

		SET_NETDEV_DEV(netdev, NULL);
		priv = netdev_priv(netdev);
		spin_lock_init(&priv->lock);
		memset(priv, 0, sizeof(struct CNS3XXXPrivate));

		priv->num_rx_queues = ring_info.num_rx_queues;
		priv->num_tx_queues = ring_info.num_tx_queues;
		priv->rx_ring = ring_info.rx_ring;
		priv->tx_ring = ring_info.tx_ring;

		priv->net_device_priv = &net_device_prive[i];
		/* set netdev MAC address */
#if defined(CONFIG_CNS3XXX_ETHADDR_IN_FLASH)
		if (do_new_mac) {
			char name[20];
			sprintf(name, "ethaddr%d=", i);
			if (0 == fmg_get(name, &val_len)) {
				mac_str_to_int(ethaddr, val_len, mac_int, 6);
				memcpy(sock_addr.sa_data, mac_int, 6);
			} else {
				memcpy(sock_addr.sa_data,
					net_device_prive[i].mac, 6);
			}
		} else
			memcpy(sock_addr.sa_data, net_device_prive[i].mac, 6);
#else
		memcpy(sock_addr.sa_data, net_device_prive[i].mac, 6);
#endif
		cns3xxx_set_mac_addr(netdev, &sock_addr);

		netdev->netdev_ops = &cns3xxx_netdev_ops;

		cns3xxx_set_ethtool_ops(netdev);

#if defined(CNS3XXX_TX_HW_CHECKSUM)
		netdev->features |= NETIF_F_IP_CSUM;
		netdev->features |= NETIF_F_SG;
#endif

		err = register_netdev(netdev);
		if (err) {
			printk(KERN_ERR "Register network dev :%s failed\n",
				netdev->name);
			goto err_register_netdev;
		}
		printk(KERN_INFO "Register network dev :%s\n", netdev->name);
		netif_carrier_off(netdev);
		netdev = 0;
	}

	return 0;

err_register_netdev:
	free_netdev(netdev);

err_alloc_etherdev:
	return err;
}

static int cns3xxx_notify_reboot(
	struct notifier_block *nb, unsigned long event, void *ptr)
{
/* stop the DMA*/
	enable_rx_dma(0, 0);
	enable_tx_dma(0, 0);
	enable_rx_dma(1, 0);
	enable_tx_dma(1, 0);

/* disable Port 0*/
	enable_port(0, 0);
	enable_port(1, 0);
	enable_port(2, 0);
	enable_port(3, 0);

/* disable phy auto-poll*/
	PHY_AUTO_ADDR_REG &= ~((1<<5) | (1<<13) | (1<<21));
/* wait state machine idle*/
	mdelay(1000);

	return NOTIFY_DONE;
}

#ifdef CONFIG_CNS3XXX_NAPI
static struct net_device *init_napi_dev(struct net_device *ndev)
{
	struct CNS3XXXPrivate *priv;
	priv = netdev_priv(ndev);

	netif_napi_add(ndev, &priv->napi , cns3xxx_poll, CNS3XXX_NAPI_WEIGHT);
	dev_hold(ndev);
	set_bit(__LINK_STATE_START, &ndev->state);

	return ndev;
}
#endif

#include <asm/hardware/gic.h>

/* type: level or edge
 * 0 - level high active, 1 - rising edge sensitive
 */
void set_interrupt_type_by_base(void __iomem *base, int id, u32 type)
{
	unsigned char int_type_bit = 0;
	u32 gic_v = 0;

	/* judge gic offset */
	int_type_bit = (id % 16 * 2 + 1);

	gic_v = readl(base + GIC_DIST_CONFIG + id / 16 * 4);

	gic_v &= (~(1 << int_type_bit));
	gic_v |= (type << int_type_bit);

	writel(gic_v, base + GIC_DIST_CONFIG + id / 16 * 4);
}

/* type: level or edge
 * 0 - level high active, 1 - rising edge sensitive
 */
void set_interrupt_type(int id, u32 type)
{
	set_interrupt_type_by_base(
		(void __iomem *) CNS3XXX_TC11MP_GIC_DIST_BASE_VIRT, id, type);
}

void get_interrupt_type_by_base(void __iomem *base, u32 id, u32 *type)
{
	unsigned char int_type_bit = 0;
	u32 gic_v = 0;

	/* judge gic offset */
	int_type_bit = (id % 16 * 2 + 1);

	gic_v = readl(base + GIC_DIST_CONFIG + id / 16 * 4);

	*type = ((gic_v >> int_type_bit) & 0x1);
}

void get_interrupt_type(u32 id, u32 *type)
{
	get_interrupt_type_by_base(
		(void __iomem *) CNS3XXX_TC11MP_GIC_DIST_BASE_VIRT, id, type);
}


void cns3xxx_config_intr(void)
{
	u32 v = 0xffffffff;

	get_interrupt_type(FSRC_RING0_INTERRUPT_ID, &v);

	set_interrupt_type(FSRC_RING0_INTERRUPT_ID, RISING_EDGE);
	get_interrupt_type(FSRC_RING0_INTERRUPT_ID, &v);

	get_interrupt_type(FSRC_RING1_INTERRUPT_ID, &v);
	set_interrupt_type(FSRC_RING1_INTERRUPT_ID, RISING_EDGE);
	get_interrupt_type(FSRC_RING1_INTERRUPT_ID, &v);

	get_interrupt_type(FSQF_RING0_INTERRUPT_ID, &v);
	set_interrupt_type(FSQF_RING0_INTERRUPT_ID, RISING_EDGE);
	get_interrupt_type(FSQF_RING0_INTERRUPT_ID, &v);

	get_interrupt_type(FSQF_RING1_INTERRUPT_ID, &v);
	set_interrupt_type(FSQF_RING1_INTERRUPT_ID, RISING_EDGE);
	get_interrupt_type(FSQF_RING1_INTERRUPT_ID, &v);
}

static int __init cns3xxx_init_module(void)
{
/* when tx_ring/rx_ring alloc memory,
 * don't free them until cns3xxx_exit_module
 */
	struct RingInfo ring_info;
	int i = 0;
	u32 reg_config = 0;

#ifdef CNS3XXX_DOUBLE_RX_RING
	ring_info.num_rx_queues = 2;
#else
	ring_info.num_rx_queues = 1;
#endif

#ifdef CNS3XXX_DOUBLE_TX_RING
	ring_info.num_tx_queues = 2;
#else
	ring_info.num_tx_queues = 1;
#endif
	ring_info.rx_ring = kcalloc(
		ring_info.num_rx_queues, sizeof(struct RXRing), GFP_KERNEL);
	if (!ring_info.rx_ring)
		return -ENOMEM;
	for (i = 0 ; i < ring_info.num_rx_queues ; ++i)
		memset(ring_info.rx_ring + i, 0, sizeof(struct RXRing));

	ring_info.tx_ring = kcalloc(
		ring_info.num_tx_queues, sizeof(struct TXRing), GFP_KERNEL);
	if (!ring_info.tx_ring)
		return -ENOMEM;
	for (i = 0 ; i < ring_info.num_tx_queues ; ++i)
		memset(ring_info.tx_ring + i, 0, sizeof(struct TXRing));

	g_ring_info = ring_info;

	cns3xxx_gsw_up_init();

	reg_config = PHY_AUTO_ADDR_REG;
	reg_config &= ~(3 << 30);
	if (jumbo_frame)
		reg_config |= (3 << 30); /* maximum frame length: 9600 bytes */
	else
		reg_config |= (2 << 30); /* maximum frame length: 1536 bytes */
	PHY_AUTO_ADDR_REG = reg_config;

	if (jumbo_frame) {
		MAX_PACKET_LEN = 9600;
		printk(KERN_INFO "jumbo_frame on\n");
		printk(KERN_INFO "MAX_PACKET_LEN:%d\n", MAX_PACKET_LEN);
	} else {
		MAX_PACKET_LEN = 1536;
		printk(KERN_INFO "jumbo_frame off\n");
		printk(KERN_INFO "MAX_PACKET_LEN:%d\n", MAX_PACKET_LEN);
	}

	cns3xxx_probe(ring_info);
	cns3xxx_config_intr();

	spin_lock_init(&tx_lock);
	spin_lock_init(&rx_lock);

#ifdef CONFIG_CNS3XXX_NAPI
	napi_dev = init_napi_dev(intr_netdev);
#ifdef CNS3XXX_DOUBLE_RX_RING
	r1_napi_dev = init_napi_dev(intr_netdev);
#endif
#endif
	register_reboot_notifier(&cns3xxx_notifier_reboot);
	clear_fs_dma_state(0);
	if (ring_info.num_rx_queues == 2)
		DMA_RING_CTRL_REG |= 1; /* enable RX dobule ring*/

	if (ring_info.num_tx_queues == 2)
		DMA_RING_CTRL_REG |= (1 << 16); /* enable TX dobule ring*/

	return 0;
}

static void __exit cns3xxx_exit_module(void)
{
	int i = 0;

	kfree(g_ring_info.rx_ring);
	kfree(g_ring_info.tx_ring);

	for (i = 0; i < NETDEV_SIZE; ++i) {
		struct CNS3XXXPrivate *priv = 0;

		if (net_dev_array[i]) {
			priv = netdev_priv(net_dev_array[i]);

			unregister_netdev(net_dev_array[i]);
			free_netdev(net_dev_array[i]);
		}
	}

#ifdef CONFIG_CNS3XXX_NAPI
	free_netdev(napi_dev);
#ifdef CNS3XXX_DOUBLE_RX_RING
	free_netdev(r1_napi_dev);
#endif
#endif

	unregister_reboot_notifier(&cns3xxx_notifier_reboot);
	printk(KERN_INFO "remove cns3xxx pse module\n");
}

MODULE_AUTHOR("Cavium Networks, <tech@XXXX.com>");
MODULE_DESCRIPTION("CNS3XXX Switch Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

module_init(cns3xxx_init_module);
module_exit(cns3xxx_exit_module);
