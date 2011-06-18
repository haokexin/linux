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

#ifndef CNS3XXX_H
#define CNS3XXX_H

#include "cns3xxx_symbol.h"
#include "cns3xxx_config.h"
#include "switch_api.h"

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/bootmem.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/ioport.h>
#include <linux/in.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/reboot.h>
#include <linux/bitops.h>
#include <linux/io.h>

#include <linux/pci.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/ip.h>
#include <linux/if_ether.h>
#include <linux/icmp.h>
#include <linux/udp.h>
#include <linux/tcp.h>
#include <linux/if_arp.h>
#include <net/arp.h>
#include <asm/irq.h>
#include <mach/cns3xxx.h>

#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
#include <linux/if_vlan.h>
#endif

struct TXDesc {
	int32_t sdp; /* segment data pointer*/

	u32 sdl:16; /* segment data length */
	u32 tco:1;
	u32 uco:1;
	u32 ico:1;
	u32 rsv_1:3; /* reserve */
	u32 pri:3;
	u32 fp:1; /* force priority*/
	u32 fr:1;
	u32 interrupt:1;
	u32 lsd:1;
	u32 fsd:1;
	u32 eor:1;
	u32 cown:1;


	u32 ctv:1;
	u32 stv:1;
	u32 sid:4;
	u32 inss:1;
	u32 dels:1;
	u32 rsv_2:9;
	u32 pmap:5;
	u32 mark:3;
	u32 ewan:1;
	u32 fewan:1;
	u32 rsv_3:5;

	u32 c_vid:12;
	u32 c_cfs:1;
	u32 c_pri:3;
	u32 s_vid:12;
	u32 s_dei:1;
	u32 s_pri:3;

	u8 alignment[16]; /* for alignment 32 byte*/
} __attribute__((packed));

struct RXDesc {
	u32 sdp;

	u32 sdl:16;
	u32 l4f:1;
	u32 ipf:1;
	u32 prot:4;
	u32 hr:6;
	u32 lsd:1;
	u32 fsd:1;
	u32 eor:1;
	u32 cown:1;

	u32 ctv:1;
	u32 stv:1;
	u32 unv:1;
	u32 iwan:1;
	u32 exdv:1;
	u32 e_wan:1;
	u32 rsv_1:2;
	u32 sp:3;
	u32 crc_err:1;
	u32 un_eth:1;
	u32 tc:2;
	u32 rsv_2:1;
	u32 ip_offset:5;
	u32 rsv_3:11;

	u32 c_vid:12;
	u32 c_cfs:1;
	u32 c_pri:3;
	u32 s_vid:12;
	u32 s_dei:1;
	u32 s_pri:3;

	u8 alignment[16]; /* for alignment 32 byte*/

} __attribute__((packed));

struct TXBuffer {
	struct TXDesc *tx_desc;
	struct sk_buff *skb; /* for free skb*/
	u32 pri;
	unsigned long tx_index;
};

struct RXBuffer {
	struct RXDesc *rx_desc;
	struct sk_buff *skb;
	/* rx path need to fill some skb field, ex: length ...*/
};


struct TXRing {
	struct TXBuffer *head;
	struct TXDesc *tx_desc_head_vir_addr;
	dma_addr_t tx_desc_head_phy_addr;
	u32 cur_index; /* for put send packet*/
	spinlock_t tx_lock;
	u32 non_free_tx_skb;
	u32 free_tx_skb_index;
	u32 ring_size;
	u32 max_ring_size;
	u32 num_free_desc;
};


struct RXRing {
	struct RXBuffer *head;
	struct RXDesc *rx_desc_head_vir_addr;
	dma_addr_t rx_desc_head_phy_addr;
	u32 cur_index;
	u32 ring_size;
	u32 max_ring_size;
};


#define RX_RING0(priv) (priv->rx_ring[0])
#define TX_RING0(priv) (priv->tx_ring[0])

#define get_rx_ring_size(ring) \
	(((struct RXRing *)ring)->ring_size)

#define get_tx_ring_size(ring) \
	(((struct TXRing *)ring)->ring_size)

#define get_rx_ring_head(rx_ring) \
	(((struct RXRing *)rx_ring)->head)

#define get_tx_ring_head(tx_ring) \
	(((struct TXRing *)tx_ring)->head)

#define get_cur_rx_buffer(rx_ring) \
(((struct RXRing *)rx_ring)->head + ((struct RXRing *)rx_ring)->cur_index)

#define get_cur_tx_buffer(tx_ring) \
(((struct TXRing *)tx_ring)->head + ((struct TXRing *)tx_ring)->cur_index)

#define get_rx_head_phy_addr(rx_ring) \
	(((struct RXRing *)rx_ring)->rx_desc_head_phy_addr)

#define get_tx_ring_head_phy_addr(tx_ring) \
	(((struct TXRing *)tx_ring)->tx_desc_head_phy_addr)

#define get_rx_cur_index(rx_ring) (((struct RXRing *)rx_ring)->cur_index)

#define get_tx_cur_index(tx_ring) (((struct TXRing *)tx_ring)->cur_index)

static inline u32 get_tx_cur_phy_addr(u8 ring_num)
{
	if (ring_num == 0)
		return TS_DESC_PTR0_REG;
	if (ring_num == 1)
		return TS_DESC_PTR1_REG;
	return 0; /* fail */
}

#define rx_index_next(ring) \
	(((struct RXRing *)ring)->cur_index =\
	((((struct RXRing *)ring)->cur_index + 1) %\
	((struct RXRing *)ring)->ring_size))

#define tx_index_next(ring) \
	(((struct TXRing *)ring)->cur_index =\
	((((struct TXRing *)ring)->cur_index + 1) %\
	((struct TXRing *)ring)->ring_size))

struct CNS3XXXPrivate;

/* for ethtool set operate */
struct NICSetting {

};

struct NetDevicePriv {
	int pmap; /* for port base, force route*/
	int is_wan; /* mean the net device is WAN side.*/
	u16 s_tag;
	u16 vlan_tag;

/* do port base mode and vlan base mode work */
	int (*rx_func)(struct sk_buff *skb,
		struct RXDesc *tx_desc_ptr, const struct CNS3XXXPrivate *);
	int (*tx_func)(struct TXDesc *tx_desc_ptr,
		const struct CNS3XXXPrivate *, struct sk_buff *);
	void (*open)(void);
	void (*close)(void);
	u8 which_port;

	u8 mac[6];
	struct VLANTableEntry *vlan_table_entry;
	struct ARLTableEntry *arl_table_entry;
	struct NICSetting *nic_setting;
	const char *name;
	/* 16 bytes, reference include/linux/netdevice.h IFNAMSIZ*/
};

struct RingInfo {
	u8 num_rx_queues;
	u8 num_tx_queues;
	struct TXRing *tx_ring;
	struct RXRing *rx_ring;
};


/* store this information for the driver.. */
struct CNS3XXXPrivate {
	u8 num_rx_queues;
	u8 num_tx_queues;
	struct TXRing *tx_ring;
	struct RXRing *rx_ring;
	struct net_device_stats stats;
	spinlock_t lock;
	int pmap;
	int is_wan; /* mean the net device is WAN side.*/
	u16 gid;
	u8 mac_type; /* VLAN base, or port base;*/
	u16 vlan_tag;
	struct napi_struct napi;
	struct work_struct reset_task;

	u8 which_port;

	char name[IFNAMSIZ];
	/* 16 bytes, reference include/linux/netdevice.h IFNAMSIZ*/

	struct NetDevicePriv *net_device_priv;
	u8 ring_index;

	u32 rx_s_vid[4096]; /* record receive s vid (0x9100 ...)*/
	u32 rx_c_vid[4096]; /* record receive c vid (0x8100 ...)*/
	volatile unsigned long is_qf; /* determine rx ring queue full state*/

#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
	struct vlan_group               *vlgrp;
#endif
};

int rx_port_base(struct sk_buff *skb,
	struct RXDesc *rx_desc_ptr, const struct CNS3XXXPrivate *priv);

int rx_vlan_base(struct sk_buff *skb,
	struct RXDesc *rx_desc_ptr, const struct CNS3XXXPrivate *priv);

int tx_port_base(struct TXDesc *tx_desc_ptr,
	const struct CNS3XXXPrivate *priv, struct sk_buff *skb);

int tx_vlan_base(struct TXDesc *tx_desc_ptr,
	const struct CNS3XXXPrivate *priv, struct sk_buff *skb);

int cns3xxx_close(struct net_device *dev);
int cns3xxx_open(struct net_device *dev);

#endif
