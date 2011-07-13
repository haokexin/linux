/*
 * drivers/net/gianfar.c
 *
 * Gianfar Ethernet Driver
 * This driver is designed for the non-CPM ethernet controllers
 * on the 85xx and 83xx family of integrated processors
 * Based on 8260_io/fcc_enet.c
 *
 * Author: Andy Fleming
 * Maintainer: Kumar Gala
 * Modifier: Sandeep Gopalpet <sandeep.kumar@freescale.com>
 *
 * Copyright 2002-2010 Freescale Semiconductor, Inc.
 * Copyright 2007 MontaVista Software, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 *  Gianfar:  AKA Lambda Draconis, "Dragon"
 *  RA 11 31 24.2
 *  Dec +69 19 52
 *  V 3.84
 *  B-V +1.62
 *
 *  Theory of operation
 *
 *  The driver is initialized through of_device. Configuration information
 *  is therefore conveyed through an OF-style device tree.
 *
 *  The Gianfar Ethernet Controller uses a ring of buffer
 *  descriptors.  The beginning is indicated by a register
 *  pointing to the physical address of the start of the ring.
 *  The end is determined by a "wrap" bit being set in the
 *  last descriptor of the ring.
 *
 *  When a packet is received, the RXF bit in the
 *  IEVENT register is set, triggering an interrupt when the
 *  corresponding bit in the IMASK register is also set (if
 *  interrupt coalescing is active, then the interrupt may not
 *  happen immediately, but will wait until either a set number
 *  of frames or amount of time have passed).  In NAPI, the
 *  interrupt handler will signal there is work to be done, and
 *  exit. This method will start at the last known empty
 *  descriptor, and process every subsequent descriptor until there
 *  are none left with data (NAPI will stop after a set number of
 *  packets to give time to other tasks, but will eventually
 *  process all the packets).  The data arrives inside a
 *  pre-allocated skb, and so after the skb is passed up to the
 *  stack, a new skb must be allocated, and the address field in
 *  the buffer descriptor must be updated to indicate this new
 *  skb.
 *
 *  When the kernel requests that a packet be transmitted, the
 *  driver starts where it left off last time, and points the
 *  descriptor at the buffer which was passed in.  The driver
 *  then informs the DMA engine that there are packets ready to
 *  be transmitted.  Once the controller is finished transmitting
 *  the packet, an interrupt may be triggered (under the same
 *  conditions as for reception, but depending on the TXF bit).
 *  The driver then cleans up the buffer.
 */

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/unistd.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/if_vlan.h>
#include <linux/spinlock.h>
#include <linux/mm.h>
#include <linux/of_mdio.h>
#include <linux/of_platform.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <linux/in.h>
#include <net/ip.h>
#include <linux/inetdevice.h>
#include <sysdev/fsl_soc.h>

#include <asm/io.h>
#include <asm/reg.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/crc32.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/phy_fixed.h>
#include <linux/of.h>

#include "gianfar.h"
#include "fsl_pq_mdio.h"

#ifdef CONFIG_AS_FASTPATH
#include <linux/sched.h>

devfp_hook_t	devfp_rx_hook;
EXPORT_SYMBOL(devfp_rx_hook);

devfp_hook_t	devfp_tx_hook;
EXPORT_SYMBOL(devfp_tx_hook);
#endif

#define TX_TIMEOUT      (1*HZ)
#undef BRIEF_GFAR_ERRORS
#undef VERBOSE_GFAR_ERRORS

#ifdef CONFIG_WRHV
#define NIC_STR_LEN	15
extern char wrhv_macaddr[MAC_ADDR_LEN];
extern char wrhv_net_name[NIC_STR_LEN]; /* eth0, eth1, eth2... */
extern int wrhv_nic_num;
extern int wrhv_nic_start; /* which index should we start at */
#endif

const char gfar_driver_name[] = "Gianfar Ethernet";
const char gfar_driver_version[] = "1.3";

/* Used for determining hardware time stamps support */
int ptp_1588_present = FALSE;
/* Used for determining when to disable PTP timer */
static int eth_counter = 0;

static int gfar_enet_open(struct net_device *dev);
static int gfar_start_xmit(struct sk_buff *skb, struct net_device *dev);
static void gfar_reset_task(struct work_struct *work);
static void gfar_timeout(struct net_device *dev);
static int gfar_close(struct net_device *dev);
struct sk_buff *gfar_new_skb(struct net_device *dev);
static void gfar_new_rxbdp(struct gfar_priv_rx_q *rx_queue, struct rxbd8 *bdp,
		struct sk_buff *skb);
static int gfar_set_mac_address(struct net_device *dev);
static int gfar_change_mtu(struct net_device *dev, int new_mtu);
static irqreturn_t gfar_error(int irq, void *dev_id);
static irqreturn_t gfar_transmit(int irq, void *dev_id);
static irqreturn_t gfar_interrupt(int irq, void *dev_id);
static void adjust_link(struct net_device *dev);
static void init_registers(struct net_device *dev);
static int init_phy(struct net_device *dev);
static int gfar_probe(struct of_device *ofdev,
		const struct of_device_id *match);
static int gfar_remove(struct of_device *ofdev);
static void free_skb_resources(struct gfar_private *priv);
static void gfar_set_multi(struct net_device *dev);
static void gfar_set_hash_for_addr(struct net_device *dev, u8 *addr);
static void gfar_configure_serdes(struct net_device *dev);
#ifdef CONFIG_GIANFAR_TXNAPI
static int gfar_poll_tx(struct napi_struct *napi, int budget);
static int gfar_poll_rx(struct napi_struct *napi, int budget);
#else
static int gfar_poll(struct napi_struct *napi, int budget);
#endif
#ifdef CONFIG_NET_POLL_CONTROLLER
static void gfar_netpoll(struct net_device *dev);
#endif
int gfar_clean_rx_ring(struct gfar_priv_rx_q *rx_queue, int rx_work_limit);
#ifdef CONFIG_GIANFAR_TXNAPI
static int gfar_clean_tx_ring(struct gfar_priv_tx_q *tx_queue, int tx_work_limit);
#else
static int gfar_clean_tx_ring(struct gfar_priv_tx_q *tx_queue);
#endif
static int gfar_process_frame(struct net_device *dev, struct sk_buff *skb,
			      int amount_pull);
static void gfar_vlan_rx_register(struct net_device *netdev,
		                struct vlan_group *grp);
void gfar_halt(struct net_device *dev);
static void gfar_halt_nodisable(struct net_device *dev);
void gfar_start(struct net_device *dev);
static void gfar_clear_exact_match(struct net_device *dev);
static void gfar_set_mac_for_addr(struct net_device *dev, int num, u8 *addr);
static int gfar_ioctl(struct net_device *dev, struct ifreq *rq, int cmd);

#ifdef CONFIG_PM
static void gfar_halt_rx(struct net_device *dev);
static void gfar_halt_tx_nodisable(struct net_device *dev);
static void gfar_rx_start(struct net_device *dev);
static void gfar_tx_start(struct net_device *dev);
static void gfar_enable_filer(struct net_device *dev);
static int gfar_get_ip(struct net_device *dev);
static void gfar_config_filer_table(struct net_device *dev);
#endif

MODULE_AUTHOR("Freescale Semiconductor, Inc");
MODULE_DESCRIPTION("Gianfar Ethernet Driver");
MODULE_LICENSE("GPL");

static void gfar_init_rxbdp(struct gfar_priv_rx_q *rx_queue, struct rxbd8 *bdp,
			    dma_addr_t buf)
{
	u32 lstatus;

	bdp->bufPtr = buf;

	lstatus = BD_LFLAG(RXBD_EMPTY | RXBD_INTERRUPT);
	if (bdp == rx_queue->rx_bd_base + rx_queue->rx_ring_size - 1)
		lstatus |= BD_LFLAG(RXBD_WRAP);

	eieio();

	bdp->lstatus = lstatus;
}

static int gfar_init_bds(struct net_device *ndev)
{
	struct gfar_private *priv = netdev_priv(ndev);
	struct gfar_priv_tx_q *tx_queue = NULL;
	struct gfar_priv_rx_q *rx_queue = NULL;
	struct txbd8 *txbdp;
	struct rxbd8 *rxbdp;
	int i, j, num;

	for (i = 0; i < priv->num_tx_queues; i++) {
		tx_queue = priv->tx_queue[i];
		/* Initialize some variables in our dev structure */
		tx_queue->num_txbdfree = tx_queue->tx_ring_size;
		tx_queue->dirty_tx = tx_queue->tx_bd_base;
		tx_queue->cur_tx = tx_queue->tx_bd_base;
		tx_queue->skb_curtx = 0;
		tx_queue->skb_dirtytx = 0;

		/* Initialize Transmit Descriptor Ring */
		txbdp = tx_queue->tx_bd_base;
		for (j = 0; j < tx_queue->tx_ring_size; j++) {
			txbdp->lstatus = 0;
			txbdp->bufPtr = 0;
			txbdp++;
		}

		/* Set the last descriptor in the ring to indicate wrap */
		txbdp--;
		txbdp->status |= TXBD_WRAP;
	}

	if ((priv->device_flags & FSL_GIANFAR_DEV_HAS_ARP_PACKET))
		num = priv->num_rx_queues - 1;
	else
		num = priv->num_rx_queues;

	for (i = 0; i < num; i++) {
		rx_queue = priv->rx_queue[i];
		rx_queue->cur_rx = rx_queue->rx_bd_base;
		rx_queue->skb_currx = 0;
		rxbdp = rx_queue->rx_bd_base;

		for (j = 0; j < rx_queue->rx_ring_size; j++) {
			struct sk_buff *skb = rx_queue->rx_skbuff[j];

			if (skb) {
				gfar_init_rxbdp(rx_queue, rxbdp,
						rxbdp->bufPtr);
			} else {
				skb = gfar_new_skb(ndev);
				if (!skb) {
					pr_err("%s: Can't allocate RX buffers\n",
							ndev->name);
					goto err_rxalloc_fail;
				}
				rx_queue->rx_skbuff[j] = skb;

				gfar_new_rxbdp(rx_queue, rxbdp, skb);
			}

			rxbdp++;
		}

	}

	return 0;

err_rxalloc_fail:
	free_skb_resources(priv);
	return -ENOMEM;
}

unsigned long alloc_bds(struct gfar_private *priv, dma_addr_t *addr)
{
	unsigned long vaddr;
	unsigned long region_size;

#ifdef CONFIG_GIANFAR_L2SRAM
	region_size = (sizeof(struct txbd8) + sizeof(struct sk_buff *)) *
			priv->total_tx_ring_size +
			(sizeof(struct rxbd8) + sizeof(struct sk_buff *)) *
			priv->total_rx_ring_size;
	vaddr =  (unsigned long) mpc85xx_cache_sram_alloc(region_size,
					(phys_addr_t *)addr, ALIGNMENT);
	if (vaddr == NULL) {
		/* fallback to normal memory rather than stop working */
		vaddr = (unsigned long) dma_alloc_coherent(&priv->ofdev->dev,
				region_size, addr, GFP_KERNEL);
		priv->bd_in_ram = 1;
	} else {
		priv->bd_in_ram = 0;
	}
#else
	region_size = sizeof(struct txbd8) * priv->total_tx_ring_size +
			sizeof(struct rxbd8) * priv->total_rx_ring_size;
	vaddr = (unsigned long) dma_alloc_coherent(&priv->ofdev->dev,
				region_size, addr, GFP_KERNEL);
#endif
	return vaddr;
}

static int gfar_alloc_skb_resources(struct net_device *ndev)
{
	void *vaddr;
	dma_addr_t addr;
	int i, j, k;
	struct gfar_private *priv = netdev_priv(ndev);
	struct gfar_priv_tx_q *tx_queue = NULL;
	struct gfar_priv_rx_q *rx_queue = NULL;
	struct rxbd8 *wkbdp;
	unsigned long wk_buf_paddr;
	unsigned long wk_buf_vaddr;
	int err = 0;

	priv->total_tx_ring_size = 0;
	for (i = 0; i < priv->num_tx_queues; i++)
		priv->total_tx_ring_size += priv->tx_queue[i]->tx_ring_size;

	priv->total_rx_ring_size = 0;
	for (i = 0; i < priv->num_rx_queues; i++)
		priv->total_rx_ring_size += priv->rx_queue[i]->rx_ring_size;

	/* Allocate memory for the buffer descriptors */
	vaddr = alloc_bds(priv, &addr);

	if (!vaddr) {
		if (netif_msg_ifup(priv))
			pr_err("%s: Could not allocate buffer descriptors!\n",
			       ndev->name);
		return -ENOMEM;
	}

	for (i = 0; i < priv->num_tx_queues; i++) {
		tx_queue = priv->tx_queue[i];
		tx_queue->tx_bd_base = (struct txbd8 *) vaddr;
		tx_queue->tx_bd_dma_base = addr;
		tx_queue->dev = ndev;
		/* enet DMA only understands physical addresses */
		addr    += sizeof(struct txbd8) *tx_queue->tx_ring_size;
		vaddr   += sizeof(struct txbd8) *tx_queue->tx_ring_size;
	}

	/* Start the rx descriptor ring where the tx ring leaves off */
	for (i = 0; i < priv->num_rx_queues; i++) {
		rx_queue = priv->rx_queue[i];
		rx_queue->rx_bd_base = (struct rxbd8 *) vaddr;
		rx_queue->rx_bd_dma_base = addr;
		rx_queue->dev = ndev;
		addr    += sizeof (struct rxbd8) * rx_queue->rx_ring_size;
		vaddr   += sizeof (struct rxbd8) * rx_queue->rx_ring_size;
	}

	/* Setup the skbuff rings */
	for (i = 0; i < priv->num_tx_queues; i++) {
		tx_queue = priv->tx_queue[i];
#ifdef CONFIG_GIANFAR_L2SRAM
		tx_queue->tx_skbuff = (struct sk_buff **) vaddr;
		vaddr += sizeof(struct sk_buff **) * tx_queue->tx_ring_size;
#else
		tx_queue->tx_skbuff = kmalloc(sizeof(*tx_queue->tx_skbuff) *
				  tx_queue->tx_ring_size, GFP_KERNEL);
#endif
		if (!tx_queue->tx_skbuff) {
			if (netif_msg_ifup(priv))
				pr_err("%s: Could not allocate tx_skbuff\n",
						ndev->name);
			goto cleanup;
		}

		for (k = 0; k < tx_queue->tx_ring_size; k++)
			tx_queue->tx_skbuff[k] = NULL;
	}

	for (i = 0; i < priv->num_rx_queues; i++) {
		rx_queue = priv->rx_queue[i];
#ifdef CONFIG_GIANFAR_L2SRAM
		rx_queue->rx_skbuff = (struct sk_buff **) vaddr;
		vaddr += sizeof(struct sk_buff **) * rx_queue->rx_ring_size;
#else
		rx_queue->rx_skbuff = kmalloc(sizeof(*rx_queue->rx_skbuff) *
				  rx_queue->rx_ring_size, GFP_KERNEL);
#endif
		if (!rx_queue->rx_skbuff) {
			if (netif_msg_ifup(priv))
				pr_err("%s: Could not allocate rx_skbuff\n",
				       ndev->name);
			goto cleanup;
		}

		for (j = 0; j < rx_queue->rx_ring_size; j++)
			rx_queue->rx_skbuff[j] = NULL;
	}

	if (gfar_init_bds(ndev))
		goto cleanup;

	if ((priv->device_flags & FSL_GIANFAR_DEV_HAS_ARP_PACKET)) {
	/* Alloc wake up rx buffer, wake up buffer need 64 bytes aligned */
		rx_queue = priv->rx_queue[priv->num_rx_queues - 1];
		rx_queue->cur_rx = rx_queue->rx_bd_base;
		vaddr = (unsigned long) dma_alloc_coherent(&priv->ofdev->dev,
			priv->wk_buffer_size * rx_queue->rx_ring_size  \
			+ RXBUF_ALIGNMENT, &addr, GFP_KERNEL);
		if (vaddr == 0) {
			if (netif_msg_ifup(priv))
				printk(KERN_ERR
					"%s:Could not allocate wakeup buffer!\n", ndev->name);
			err = -ENOMEM;
			goto wk_buf_fail;
		}

		priv->wk_buf_vaddr = vaddr;
		priv->wk_buf_paddr = addr;
		wk_buf_vaddr = (unsigned long)(vaddr + RXBUF_ALIGNMENT) \
						& ~(RXBUF_ALIGNMENT - 1);
		wk_buf_paddr = (unsigned long)(addr + RXBUF_ALIGNMENT) \
						& ~(RXBUF_ALIGNMENT - 1);
		priv->wk_buf_align_vaddr = wk_buf_vaddr;
		priv->wk_buf_align_paddr = wk_buf_paddr;

		/* Setup wake up rx ring and buffer */
		wkbdp = rx_queue->rx_bd_base;
		for (i = 0; i < rx_queue->rx_ring_size; i++) {
			wkbdp->status = RXBD_EMPTY | RXBD_INTERRUPT;
			wkbdp->length = 0;
			wkbdp->bufPtr = wk_buf_paddr + priv->wk_buffer_size * i;
			wkbdp++;
		}

		/* Set the last descriptor in the ring to wrap */
		wkbdp--;
		wkbdp->status |= RXBD_WRAP;
	}
	return 0;

wk_buf_fail:
	dma_free_coherent(&priv->ofdev->dev,
			priv->wk_buffer_size * rx_queue->rx_ring_size \
			+ RXBUF_ALIGNMENT, (void *)priv->wk_buf_vaddr,
			priv->wk_buf_paddr);
cleanup:
	free_skb_resources(priv);
	return -ENOMEM;
}

static void gfar_init_tx_rx_base(struct gfar_private *priv)
{
	struct gfar __iomem *regs = priv->gfargrp[0].regs;
	u32 __iomem *baddr;
	int i;
#ifdef CONFIG_PHYS_64BIT
	dma_addr_t addr;

	addr = priv->tx_queue[0]->tx_bd_dma_base;
	gfar_write(&regs->tbaseh, ((addr >> 32) & GFAR_TX_BASE_H));
	addr = priv->rx_queue[0]->rx_bd_dma_base;
	gfar_write(&regs->rbaseh, ((addr >> 32) & GFAR_RX_BASE_H));
#endif
	baddr = &regs->tbase0;
	for(i = 0; i < priv->num_tx_queues; i++) {
		gfar_write(baddr, priv->tx_queue[i]->tx_bd_dma_base);
		baddr	+= 2;
	}

	baddr = &regs->rbase0;
	for(i = 0; i < priv->num_rx_queues; i++) {
		gfar_write(baddr, priv->rx_queue[i]->rx_bd_dma_base);
		baddr   += 2;
	}
}

static void gfar_init_mac(struct net_device *ndev)
{
	struct gfar_private *priv = netdev_priv(ndev);
	struct gfar __iomem *regs = priv->gfargrp[0].regs;
	u32 rctrl = 0;
	u32 tctrl = 0;
	u32 attrs = 0;

	/* write the tx/rx base registers */
	gfar_init_tx_rx_base(priv);

	/* Configure the coalescing support */
	gfar_configure_tx_coalescing(priv, 0xFF);
	gfar_configure_rx_coalescing(priv, 0xFF);

	if (priv->rx_filer_enable) {
		rctrl |= RCTRL_FILREN;
		/* Program the RIR0 reg with the required distribution */
		gfar_write(&regs->rir0, DEFAULT_RIR0);
	}

	if (priv->rx_csum_enable)
		rctrl |= RCTRL_CHECKSUMMING;

	if (priv->extended_hash) {
		rctrl |= RCTRL_EXTHASH;

		gfar_clear_exact_match(ndev);
		rctrl |= RCTRL_EMEN;
	}

	if (priv->padding) {
		rctrl &= ~RCTRL_PAL_MASK;
		rctrl |= RCTRL_PADDING(priv->padding);
	}

	if (priv->ptimer_present) {

		/* Enable Filer and Rx Packet Parsing capability of eTSEC */
		/* Set Filer Table */
		gfar_1588_start(ndev);
		if (priv->device_flags & FSL_GIANFAR_DEV_HAS_PADDING)
			rctrl &= ~RCTRL_PAL_MASK;
		/* Enable Filer for Rx Queue */
		rctrl |= RCTRL_PRSDEP_INIT |
			RCTRL_TS_ENABLE | RCTRL_PADDING(8);
		priv->padding = 0x8;
	}

	/* keep vlan related bits if it's enabled */
	if (priv->vlgrp) {
		rctrl |= RCTRL_VLEX | RCTRL_PRSDEP_INIT;
		tctrl |= TCTRL_VLINS;
	}

	/* Init rctrl based on our settings */
	gfar_write(&regs->rctrl, rctrl);

	if (ndev->features & NETIF_F_IP_CSUM)
		tctrl |= TCTRL_INIT_CSUM;

	tctrl |= TCTRL_TXSCHED_WRRS;

	gfar_write(&regs->tr03wt, WRRS_TR03WT);
	gfar_write(&regs->tr47wt, WRRS_TR47WT);

	gfar_write(&regs->tctrl, tctrl);

	/* Set the extraction length and index */
	attrs = ATTRELI_EL(priv->rx_stash_size) |
		ATTRELI_EI(priv->rx_stash_index);

	gfar_write(&regs->attreli, attrs);

	/* Start with defaults, and add stashing or locking
	 * depending on the approprate variables */
	attrs = ATTR_INIT_SETTINGS;

	if (priv->bd_stash_en)
		attrs |= ATTR_BDSTASH;

	if (priv->rx_stash_size != 0)
		attrs |= ATTR_BUFSTASH;

	gfar_write(&regs->attr, attrs);

}

static struct net_device_stats *gfar_get_stats(struct net_device *dev)
{
	struct gfar_private *priv = netdev_priv(dev);
	struct netdev_queue *txq;
	unsigned long rx_packets = 0, rx_bytes = 0, rx_dropped = 0;
	unsigned long tx_packets = 0, tx_bytes = 0;
	int i = 0;

	for (i = 0; i < priv->num_rx_queues; i++) {
		rx_packets += priv->rx_queue[i]->stats.rx_packets;
		rx_bytes += priv->rx_queue[i]->stats.rx_bytes;
		rx_dropped += priv->rx_queue[i]->stats.rx_dropped;
	}

	dev->stats.rx_packets = rx_packets;
	dev->stats.rx_bytes = rx_bytes;
	dev->stats.rx_dropped = rx_dropped;

	for (i = 0; i < priv->num_tx_queues; i++) {
		txq = netdev_get_tx_queue(dev, i);
		tx_bytes += txq->tx_bytes;
		tx_packets += txq->tx_packets;
	}

	dev->stats.tx_bytes = tx_bytes;
	dev->stats.tx_packets = tx_packets;

	return &dev->stats;
}

static const struct net_device_ops gfar_netdev_ops = {
	.ndo_open = gfar_enet_open,
	.ndo_start_xmit = gfar_start_xmit,
	.ndo_stop = gfar_close,
	.ndo_change_mtu = gfar_change_mtu,
	.ndo_set_multicast_list = gfar_set_multi,
	.ndo_tx_timeout = gfar_timeout,
	.ndo_do_ioctl = gfar_ioctl,
	.ndo_get_stats = gfar_get_stats,
	.ndo_vlan_rx_register = gfar_vlan_rx_register,
	.ndo_set_mac_address = eth_mac_addr,
	.ndo_validate_addr = eth_validate_addr,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller = gfar_netpoll,
#endif
};

void lock_rx_qs(struct gfar_private *priv)
{
	int i = 0x0;

	for (i = 0; i < priv->num_rx_queues; i++)
		spin_lock(&priv->rx_queue[i]->rxlock);
}

void lock_tx_qs(struct gfar_private *priv)
{
	int i = 0x0;

	for (i = 0; i < priv->num_tx_queues; i++)
		spin_lock(&priv->tx_queue[i]->txlock);
}

void unlock_rx_qs(struct gfar_private *priv)
{
	int i = 0x0;

	for (i = 0; i < priv->num_rx_queues; i++)
		spin_unlock(&priv->rx_queue[i]->rxlock);
}

void unlock_tx_qs(struct gfar_private *priv)
{
	int i = 0x0;

	for (i = 0; i < priv->num_tx_queues; i++)
		spin_unlock(&priv->tx_queue[i]->txlock);
}

/* Returns 1 if incoming frames use an FCB */
static inline int gfar_uses_fcb(struct gfar_private *priv)
{
	return priv->vlgrp || priv->rx_csum_enable;
}

static void free_tx_pointers(struct gfar_private *priv)
{
	int i = 0;

	for (i = 0; i < priv->num_tx_queues; i++)
		kfree(priv->tx_queue[i]);
}

static void free_rx_pointers(struct gfar_private *priv)
{
	int i = 0;

	for (i = 0; i < priv->num_rx_queues; i++)
		kfree(priv->rx_queue[i]);
}

static void unmap_group_regs(struct gfar_private *priv)
{
	int i = 0;

	for (i = 0; i < MAXGROUPS; i++)
		if (priv->gfargrp[i].regs)
			iounmap(priv->gfargrp[i].regs);
}

static void disable_napi(struct gfar_private *priv)
{
	int i = 0;
#ifdef CONFIG_GIANFAR_TXNAPI
	for (i = 0; i < priv->num_grps; i++) {
		napi_disable(&priv->gfargrp[i].napi_tx);
		napi_disable(&priv->gfargrp[i].napi_rx);
	}
#else
	for (i = 0; i < priv->num_grps; i++)
		napi_disable(&priv->gfargrp[i].napi);
#endif
}

static void enable_napi(struct gfar_private *priv)
{
	int i = 0;

#ifdef CONFIG_GIANFAR_TXNAPI
	for (i = 0; i < priv->num_grps; i++) {
		napi_enable(&priv->gfargrp[i].napi_tx);
		napi_enable(&priv->gfargrp[i].napi_rx);
	}
#else
	for (i = 0; i < priv->num_grps; i++)
		napi_enable(&priv->gfargrp[i].napi);
#endif
}

static int gfar_parse_group(struct device_node *np,
		struct gfar_private *priv, const char *model)
{
	u32 *queue_mask;

	priv->gfargrp[priv->num_grps].regs = of_iomap(np, 0);
	if (!priv->gfargrp[priv->num_grps].regs)
		return -ENOMEM;

	priv->gfargrp[priv->num_grps].interruptTransmit =
			irq_of_parse_and_map(np, 0);

	/* If we aren't the FEC we have multiple interrupts */
	if (model && strcasecmp(model, "FEC")) {
		priv->gfargrp[priv->num_grps].interruptReceive =
			irq_of_parse_and_map(np, 1);
		priv->gfargrp[priv->num_grps].interruptError =
			irq_of_parse_and_map(np,2);
		if (priv->gfargrp[priv->num_grps].interruptTransmit < 0 ||
			priv->gfargrp[priv->num_grps].interruptReceive < 0 ||
			priv->gfargrp[priv->num_grps].interruptError < 0) {
			return -EINVAL;
		}
	}

	priv->gfargrp[priv->num_grps].grp_id = priv->num_grps;
	priv->gfargrp[priv->num_grps].priv = priv;
	spin_lock_init(&priv->gfargrp[priv->num_grps].grplock);
	if(priv->mode == MQ_MG_MODE) {
		queue_mask = (u32 *)of_get_property(np,
					"rx-bit-map", NULL);
		priv->gfargrp[priv->num_grps].rx_bit_map =
			queue_mask ?  *queue_mask :(DEFAULT_MAPPING >> priv->num_grps);
		queue_mask = (u32 *)of_get_property(np,
					"tx-bit-map", NULL);
		priv->gfargrp[priv->num_grps].tx_bit_map =
			queue_mask ? *queue_mask : (DEFAULT_MAPPING >> priv->num_grps);
	} else {
		priv->gfargrp[priv->num_grps].rx_bit_map = 0xFF;
		priv->gfargrp[priv->num_grps].tx_bit_map = 0xFF;
	}
	priv->num_grps++;

	return 0;
}

static int gfar_of_init(struct of_device *ofdev, struct net_device **pdev)
{
	const char *model;
	const char *ctype;
	const void *mac_addr;
	int err = 0, i, ret = 0;
	struct net_device *dev = NULL;
	struct gfar_private *priv = NULL;
	struct device_node *np = ofdev->node;
	struct device_node *child = NULL;
	struct device_node *timer_node;
	const phandle *timer_handle;
	const u32 *stash;
	const u32 *stash_len;
	const u32 *stash_idx;
	unsigned int num_tx_qs, num_rx_qs;
	u32 *tx_queues, *rx_queues;
	u32 *busFreq;
	u32 etsec_clk;
	u32 max_filer_rules;

	if (!np || !of_device_is_available(np))
		return -ENODEV;

	/* parse the num of tx and rx queues */
	tx_queues = (u32 *)of_get_property(np, "fsl,num_tx_queues", NULL);
	num_tx_qs = tx_queues ? *tx_queues : 1;

	if (num_tx_qs > MAX_TX_QS) {
		printk(KERN_ERR "num_tx_qs(=%d) greater than MAX_TX_QS(=%d)\n",
				num_tx_qs, MAX_TX_QS);
		printk(KERN_ERR "Cannot do alloc_etherdev, aborting\n");
		return -EINVAL;
	}

	rx_queues = (u32 *)of_get_property(np, "fsl,num_rx_queues", NULL);
	num_rx_qs = rx_queues ? *rx_queues : 1;

	if (num_rx_qs > MAX_RX_QS) {
		printk(KERN_ERR "num_rx_qs(=%d) greater than MAX_RX_QS(=%d)\n",
				num_tx_qs, MAX_TX_QS);
		printk(KERN_ERR "Cannot do alloc_etherdev, aborting\n");
		return -EINVAL;
	}

	*pdev = alloc_etherdev_mq(sizeof(*priv), num_tx_qs);
	dev = *pdev;
	if (NULL == dev)
		return -ENOMEM;

	priv = netdev_priv(dev);
	priv->node = ofdev->node;
	priv->ndev = dev;

	busFreq = (u32 *)of_get_property
			(of_get_parent(np), "bus-frequency", NULL);
	if (busFreq) {
		/* etsec_clk is CCB/2 */
		etsec_clk = *busFreq/2;
		/* Divide by 1000000 to get freq in MHz */
		etsec_clk /= 1000000;
		/*
		 * eTSEC searches the table at a rate of two entries every
		 * eTSEC clock cycle, so for the worst case all 256 entries
		 * can be searched in the time taken to receive a 64-byte
		 * Ethernet frame which comes out to be 672 ns at 1Gbps rate
		 * including inter frame gap and preamble.
		 * Hence max_filer_rules = etsec_clk * reception time for one
		 * packet * 2. Divide by 1000 to match the units.
		 */
		max_filer_rules = etsec_clk * 672 * 2 / 1000;
		if (max_filer_rules > MAX_FILER_IDX)
			priv->max_filer_rules = MAX_FILER_IDX;
		else
			priv->max_filer_rules = max_filer_rules;
	} else {
		printk(KERN_INFO "Bus Frequency not found in DTS, "
				"setting max_filer_rules to %d\n",
				MAX_FILER_IDX);
		priv->max_filer_rules = MAX_FILER_IDX;
	}
	dev->num_tx_queues = num_tx_qs;
	dev->real_num_tx_queues = num_tx_qs;
	priv->num_tx_queues = num_tx_qs;
	priv->num_rx_queues = num_rx_qs;
	priv->num_grps = 0x0;

	model = of_get_property(np, "model", NULL);

	for (i = 0; i < MAXGROUPS; i++)
		priv->gfargrp[i].regs = NULL;

	/* Parse and initialize group specific information */
	if (of_device_is_compatible(np, "fsl,etsec2")) {
		priv->mode = MQ_MG_MODE;
		for_each_child_of_node(np, child) {
			if (of_device_is_compatible
				(child, "fsl,etsec2-mdio") ||
				of_device_is_compatible
				(child, "fsl,etsec2-tbi"))
				continue;
			err = gfar_parse_group(child, priv, model);
			if (err)
				goto err_grp_init;
		}
	} else {
		priv->mode = SQ_SG_MODE;
		err = gfar_parse_group(np, priv, model);
		if(err)
			goto err_grp_init;
	}

	for (i = 0; i < priv->num_tx_queues; i++)
	       priv->tx_queue[i] = NULL;
	for (i = 0; i < priv->num_rx_queues; i++)
		priv->rx_queue[i] = NULL;

	for (i = 0; i < priv->num_tx_queues; i++) {
		priv->tx_queue[i] =  (struct gfar_priv_tx_q *)kzalloc(
				sizeof (struct gfar_priv_tx_q), GFP_KERNEL);
		if (!priv->tx_queue[i]) {
			err = -ENOMEM;
			goto tx_alloc_failed;
		}
		priv->tx_queue[i]->tx_skbuff = NULL;
		priv->tx_queue[i]->qindex = i;
		priv->tx_queue[i]->dev = dev;
		spin_lock_init(&(priv->tx_queue[i]->txlock));
	}

	for (i = 0; i < priv->num_rx_queues; i++) {
		priv->rx_queue[i] = (struct gfar_priv_rx_q *)kzalloc(
					sizeof (struct gfar_priv_rx_q), GFP_KERNEL);
		if (!priv->rx_queue[i]) {
			err = -ENOMEM;
			goto rx_alloc_failed;
		}
		priv->rx_queue[i]->rx_skbuff = NULL;
		priv->rx_queue[i]->qindex = i;
		priv->rx_queue[i]->dev = dev;
		spin_lock_init(&(priv->rx_queue[i]->rxlock));
	}


	stash = of_get_property(np, "bd-stash", NULL);

	if (stash) {
		priv->device_flags |= FSL_GIANFAR_DEV_HAS_BD_STASHING;
		priv->bd_stash_en = 1;
	}

	stash_len = of_get_property(np, "rx-stash-len", NULL);

	if (stash_len)
		priv->rx_stash_size = *stash_len;

	stash_idx = of_get_property(np, "rx-stash-idx", NULL);

	if (stash_idx)
		priv->rx_stash_index = *stash_idx;

	if (stash_len || stash_idx)
		priv->device_flags |= FSL_GIANFAR_DEV_HAS_BUF_STASHING;

	/* Handle IEEE1588 node */
	timer_handle = of_get_property(np, "ptimer-handle", NULL);
	if (timer_handle) {
		timer_node = of_find_node_by_phandle(*timer_handle);
		if (timer_node) {
			ret = of_address_to_resource(timer_node, 0,
					&priv->timer_resource);
			if (!ret) {
				priv->ptimer_present = 1;
				printk(KERN_INFO "IEEE1588: ptp-timer device"
						"present in the system\n");
			}
		}
	} else
		printk(KERN_INFO "IEEE1588: disable on the system.\n");

	mac_addr = of_get_mac_address(np);
	if (mac_addr)
		memcpy(dev->dev_addr, mac_addr, MAC_ADDR_LEN);

	if (model && !strcasecmp(model, "TSEC"))
		priv->device_flags =
			FSL_GIANFAR_DEV_HAS_GIGABIT |
			FSL_GIANFAR_DEV_HAS_COALESCE |
			FSL_GIANFAR_DEV_HAS_RMON |
			FSL_GIANFAR_DEV_HAS_MULTI_INTR;
	if (model && !strcasecmp(model, "eTSEC")) {
		priv->device_flags =
			FSL_GIANFAR_DEV_HAS_GIGABIT |
			FSL_GIANFAR_DEV_HAS_COALESCE |
			FSL_GIANFAR_DEV_HAS_RMON |
			FSL_GIANFAR_DEV_HAS_MULTI_INTR |
			FSL_GIANFAR_DEV_HAS_PADDING |
			FSL_GIANFAR_DEV_HAS_CSUM |
			FSL_GIANFAR_DEV_HAS_EXTENDED_HASH;
#ifndef CONFIG_GFAR_SW_VLAN
		priv->device_flags |=
			FSL_GIANFAR_DEV_HAS_VLAN;
#endif
	}

	ctype = of_get_property(np, "phy-connection-type", NULL);

	/* We only care about rgmii-id.  The rest are autodetected */
	if (ctype && !strcmp(ctype, "rgmii-id"))
		priv->interface = PHY_INTERFACE_MODE_RGMII_ID;
	else
		priv->interface = PHY_INTERFACE_MODE_MII;

	if (of_get_property(np, "fsl,magic-packet", NULL))
		priv->device_flags |= FSL_GIANFAR_DEV_HAS_MAGIC_PACKET;

	if (of_get_property(np, "fsl,wake-on-filer", NULL))
		priv->device_flags |= FSL_GIANFAR_DEV_HAS_ARP_PACKET;

	priv->phy_node = of_parse_phandle(np, "phy-handle", 0);

	/* Find the TBI PHY.  If it's not there, we don't support SGMII */
	priv->tbi_node = of_parse_phandle(np, "tbi-handle", 0);

	return 0;

rx_alloc_failed:
	free_rx_pointers(priv);
tx_alloc_failed:
	free_tx_pointers(priv);
err_grp_init:
	unmap_group_regs(priv);
	free_netdev(dev);
	return err;
}

/* Ioctl MII Interface */
static int gfar_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct gfar_private *priv = netdev_priv(dev);
	int retVal = 0;

	if (!netif_running(dev))
		return -EINVAL;

	if (!priv->phydev)
		return -ENODEV;

	if ((cmd >= PTP_GET_RX_TIMESTAMP_SYNC) &&
			(cmd <= PTP_CLEANUP_TIMESTAMP_BUFFERS)) {
		if (priv->ptimer_present)
			retVal = gfar_ioctl_1588(dev, rq, cmd);
		else
			retVal = -ENODEV;
	} else
		retVal = phy_mii_ioctl(priv->phydev, if_mii(rq), cmd);

	return retVal;
}

static unsigned int reverse_bitmap(unsigned int bit_map, unsigned int max_qs)
{
	unsigned int new_bit_map = 0x0;
	int mask = 0x1 << (max_qs - 1), i;
	for (i = 0; i < max_qs; i++) {
		if (bit_map & mask)
			new_bit_map = new_bit_map + (1 << i);
		mask = mask >> 0x1;
	}
	return new_bit_map;
}

static u32 cluster_entry_per_class(struct gfar_private *priv, u32 rqfar,
				   u32 class)
{
	u32 rqfpr = FPR_FILER_MASK;
	u32 rqfcr = 0x0;

	rqfar--;
	rqfcr = RQFCR_CLE | RQFCR_PID_MASK | RQFCR_CMP_EXACT;
	priv->ftp_rqfpr[rqfar] = rqfpr;
	priv->ftp_rqfcr[rqfar] = rqfcr;
	gfar_write_filer(priv, rqfar, rqfcr, rqfpr);

	rqfar--;
	rqfcr = RQFCR_CMP_NOMATCH;
	priv->ftp_rqfpr[rqfar] = rqfpr;
	priv->ftp_rqfcr[rqfar] = rqfcr;
	gfar_write_filer(priv, rqfar, rqfcr, rqfpr);

	rqfar--;
	rqfcr = RQFCR_CMP_EXACT | RQFCR_PID_PARSE | RQFCR_CLE | RQFCR_AND;
	rqfpr = class;
	priv->ftp_rqfcr[rqfar] = rqfcr;
	priv->ftp_rqfpr[rqfar] = rqfpr;
	gfar_write_filer(priv, rqfar, rqfcr, rqfpr);

	rqfar--;
	rqfcr = RQFCR_CMP_EXACT | RQFCR_PID_MASK | RQFCR_AND;
	rqfpr = class;
	priv->ftp_rqfcr[rqfar] = rqfcr;
	priv->ftp_rqfpr[rqfar] = rqfpr;
	gfar_write_filer(priv, rqfar, rqfcr, rqfpr);

	return rqfar;
}

static void gfar_init_filer_table(struct gfar_private *priv)
{
	int i = 0x0;
	u32 rqfar = priv->max_filer_rules;
	u32 rqfcr = 0x0;
	u32 rqfpr = FPR_FILER_MASK;

	if (!priv->ftp_rqfpr) {
		priv->ftp_rqfpr = kmalloc((priv->max_filer_rules + 1)*sizeof
					(u32), GFP_KERNEL);
		if (!priv->ftp_rqfpr) {
			pr_err("Could not allocate ftp_rqfpr\n");
			goto out;
		}
	}
	if (!priv->ftp_rqfcr) {
		priv->ftp_rqfcr = kmalloc((priv->max_filer_rules + 1)*sizeof
					(u32), GFP_KERNEL);
		if (!priv->ftp_rqfcr) {
			pr_err("Could not allocate ftp_rqfcr\n");
			goto out;
		}
	}
	/* Default rule */
	rqfcr = RQFCR_CMP_MATCH;
	priv->ftp_rqfcr[rqfar] = rqfcr;
	priv->ftp_rqfpr[rqfar] = rqfpr;
	gfar_write_filer(priv, rqfar, rqfcr, rqfpr);

	rqfar = cluster_entry_per_class(priv, rqfar, RQFPR_IPV6);
	rqfar = cluster_entry_per_class(priv, rqfar, RQFPR_IPV6 | RQFPR_UDP);
	rqfar = cluster_entry_per_class(priv, rqfar, RQFPR_IPV6 | RQFPR_TCP);
	rqfar = cluster_entry_per_class(priv, rqfar, RQFPR_IPV4);
	rqfar = cluster_entry_per_class(priv, rqfar, RQFPR_IPV4 | RQFPR_UDP);
	rqfar = cluster_entry_per_class(priv, rqfar, RQFPR_IPV4 | RQFPR_TCP);

	/* cur_filer_idx indicated the fisrt non-masked rule */
	priv->cur_filer_idx = rqfar;

	/* Program the RIR0 reg with the required distribution */
	priv->gfargrp[0].regs->rir0 = DEFAULT_RIR0;

	/* Rest are masked rules */
	rqfcr = RQFCR_CMP_NOMATCH;
	for (i = 0; i < rqfar; i++) {
		priv->ftp_rqfcr[i] = rqfcr;
		priv->ftp_rqfpr[i] = rqfpr;
		gfar_write_filer(priv, i, rqfcr, rqfpr);
	}
	return;
out:
	kfree(priv->ftp_rqfcr);
	kfree(priv->ftp_rqfpr);
	priv->ftp_rqfpr = priv->ftp_rqfcr = NULL;
}

extern unsigned int get_pvr(void);
extern unsigned int get_svr(void);
static void gfar_detect_errata(struct gfar_private *priv)
{
	struct device *dev = &priv->ofdev->dev;
	unsigned int pvr = get_pvr();
	unsigned int svr = get_svr();
	unsigned int mod = (svr >> 16) & 0xfff6; /* w/o E suffix */
	unsigned int rev = svr & 0xffff;

	/* MPC8313 Rev 2.0 and higher; All MPC837x */
	if ((pvr == 0x80850010 && mod == 0x80b0 && rev >= 0x0020) ||
			(pvr == 0x80861010 && (mod & 0xfff9) == 0x80c0))
		priv->errata |= GFAR_ERRATA_74;

	/* MPC8313 and MPC837x all rev */
	if ((pvr == 0x80850010 && mod == 0x80b0) ||
			(pvr == 0x80861010 && (mod & 0xfff9) == 0x80c0))
		priv->errata |= GFAR_ERRATA_76;

	/* MPC8313 and MPC837x all rev */
	if ((pvr == 0x80850010 && mod == 0x80b0) ||
			(pvr == 0x80861010 && (mod & 0xfff9) == 0x80c0))
		priv->errata |= GFAR_ERRATA_A002;

	/* P2020 Rev 2.0 */
	if (priv->ptimer_present && pvr == 0x80211050)
		priv->errata |= GFAR_ERRATA_A002;

	if (priv->errata)
		dev_info(dev, "enabled errata workarounds, flags: 0x%x\n",
			 priv->errata);
}

static int get_cpu_number(unsigned char *eth_pkt, int len)
{
	u32 addr1, addr2, ports;
	struct ipv6hdr *ip6;
	struct iphdr *ip;
	u32 hash, ihl;
	u8 ip_proto;
	int cpu;
	struct ethhdr *eth;
	static u32 simple_hashrnd;
	static int simple_hashrnd_initialized;

	if (len < ETH_HLEN)
		return -1;
	else
		eth = eth_pkt;

	if (unlikely(!simple_hashrnd_initialized)) {
		get_random_bytes(&simple_hashrnd, 4);
		simple_hashrnd_initialized = 1;
	}

	switch (eth->h_proto) {
	case __constant_htons(ETH_P_IP):
		if (len < (ETH_HLEN + sizeof(*ip)))
			return -1;

		ip = (struct iphdr *) (eth_pkt + ETH_HLEN);
		ip_proto = ip->protocol;
		addr1 = ip->saddr;
		addr2 = ip->daddr;
		ihl = ip->ihl;
		break;
	case __constant_htons(ETH_P_IPV6):
		if (len < (ETH_HLEN + sizeof(*ip6)))
			return -1;

		ip6 = (struct ipv6hdr *)(eth_pkt + ETH_HLEN);
		ip_proto = ip6->nexthdr;
		addr1 = ip6->saddr.s6_addr32[3];
		addr2 = ip6->daddr.s6_addr32[3];
		ihl = (40 >> 2);
		break;
	default:
		return -EINVAL;
	}
	ports = 0;
	switch (ip_proto) {
	case IPPROTO_TCP:
	case IPPROTO_UDP:
	case IPPROTO_DCCP:
	case IPPROTO_ESP:
	case IPPROTO_AH:
	case IPPROTO_SCTP:
	case IPPROTO_UDPLITE:
		if (len < (ETH_HLEN + (ihl * 4) + 4))
			ports = *((u32 *) (eth_pkt + ETH_HLEN + (ihl * 4)));
		break;

	default:
		break;
	}

	hash = jhash_3words(addr1, addr2, ports, simple_hashrnd);
	cpu = hash & 0x1;

	return cpu_online(cpu) ? cpu : -1;
}

/* Set up the ethernet device structure, private data,
 * and anything else we need before we start */
static int gfar_probe(struct of_device *ofdev,
		const struct of_device_id *match)
{
	u32 tempval;
	struct net_device *dev = NULL;
	struct gfar_private *priv = NULL;
	struct gfar __iomem *regs = NULL;
	int err = 0, i, grp_idx = 0;
	int len_devname;
	u32 rstat = 0, tstat = 0, rqueue = 0, tqueue = 0;
	u32 isrg = 0;
	u32 __iomem *baddr;
	int j;

	err = gfar_of_init(ofdev, &dev);

	if (err)
		return err;

	priv = netdev_priv(dev);
	priv->ndev = dev;
	priv->ofdev = ofdev;
	priv->node = ofdev->node;
	SET_NETDEV_DEV(dev, &ofdev->dev);

	if (priv->ptimer_present) {
		err = gfar_ptp_init(priv);
		if (err) {
			priv->ptimer_present = 0;
			printk(KERN_ERR "IEEE1588: ptp-timer init failed\n");
		}

		if (!ptp_1588_present) {
			gfar_1588_proc_init();
			ptp_1588_present = TRUE;
		}

		priv->rx_filer_enable = 1;
		pmuxcr_guts_write();
		printk(KERN_INFO "IEEE1588: ptp-timer initialized\n");
	}

	spin_lock_init(&priv->bflock);
	INIT_WORK(&priv->reset_task, gfar_reset_task);

	dev_set_drvdata(&ofdev->dev, priv);
	regs = priv->gfargrp[0].regs;

	gfar_detect_errata(priv);

	/* Stop the DMA engine now, in case it was running before */
	/* (The firmware could have used it, and left it running). */
	gfar_halt(dev);

	/* Reset MAC layer */
	gfar_write(&regs->maccfg1, MACCFG1_SOFT_RESET);

	/* We need to delay at least 3 TX clocks */
	udelay(2);

	tempval = (MACCFG1_TX_FLOW | MACCFG1_RX_FLOW);
	gfar_write(&regs->maccfg1, tempval);

	/* Initialize MACCFG2. */
	tempval = MACCFG2_INIT_SETTINGS;
	if (gfar_has_errata(priv, GFAR_ERRATA_74))
		tempval |= MACCFG2_HUGEFRAME | MACCFG2_LENGTHCHECK;
	gfar_write(&regs->maccfg2, tempval);

	/* Initialize ECNTRL */
	gfar_write(&regs->ecntrl, ECNTRL_INIT_SETTINGS);

	/* Set the dev->base_addr to the gfar reg region */
	dev->base_addr = (unsigned long) regs;

	SET_NETDEV_DEV(dev, &ofdev->dev);

	/* Fill in the dev structure */
	dev->watchdog_timeo = TX_TIMEOUT;
	dev->mtu = 1500;
	dev->netdev_ops = &gfar_netdev_ops;
	dev->ethtool_ops = &gfar_ethtool_ops;

#ifdef CONFIG_GIANFAR_TXNAPI
	/* Seperate napi for tx and rx for each group */
	for (i = 0; i < priv->num_grps; i++) {
		netif_napi_add(dev, &priv->gfargrp[i].napi_tx, gfar_poll_tx,
				GFAR_DEV_WEIGHT);
		netif_napi_add(dev, &priv->gfargrp[i].napi_rx, gfar_poll_rx,
				GFAR_DEV_WEIGHT);
	}
#else
	/* Register for napi ...We are registering NAPI for each grp */
	for (i = 0; i < priv->num_grps; i++)
		netif_napi_add(dev, &priv->gfargrp[i].napi, gfar_poll, GFAR_DEV_WEIGHT);
#endif

	if (priv->device_flags & FSL_GIANFAR_DEV_HAS_CSUM) {
		priv->rx_csum_enable = 1;
		dev->features |= NETIF_F_IP_CSUM | NETIF_F_SG | NETIF_F_HIGHDMA;
	} else
		priv->rx_csum_enable = 0;

	priv->vlgrp = NULL;

	if (priv->device_flags & FSL_GIANFAR_DEV_HAS_VLAN)
		dev->features |= NETIF_F_HW_VLAN_TX | NETIF_F_HW_VLAN_RX;

	if (priv->device_flags & FSL_GIANFAR_DEV_HAS_EXTENDED_HASH) {
		priv->extended_hash = 1;
		priv->hash_width = 9;

		priv->hash_regs[0] = &regs->igaddr0;
		priv->hash_regs[1] = &regs->igaddr1;
		priv->hash_regs[2] = &regs->igaddr2;
		priv->hash_regs[3] = &regs->igaddr3;
		priv->hash_regs[4] = &regs->igaddr4;
		priv->hash_regs[5] = &regs->igaddr5;
		priv->hash_regs[6] = &regs->igaddr6;
		priv->hash_regs[7] = &regs->igaddr7;
		priv->hash_regs[8] = &regs->gaddr0;
		priv->hash_regs[9] = &regs->gaddr1;
		priv->hash_regs[10] = &regs->gaddr2;
		priv->hash_regs[11] = &regs->gaddr3;
		priv->hash_regs[12] = &regs->gaddr4;
		priv->hash_regs[13] = &regs->gaddr5;
		priv->hash_regs[14] = &regs->gaddr6;
		priv->hash_regs[15] = &regs->gaddr7;

	} else {
		priv->extended_hash = 0;
		priv->hash_width = 8;

		priv->hash_regs[0] = &regs->gaddr0;
		priv->hash_regs[1] = &regs->gaddr1;
		priv->hash_regs[2] = &regs->gaddr2;
		priv->hash_regs[3] = &regs->gaddr3;
		priv->hash_regs[4] = &regs->gaddr4;
		priv->hash_regs[5] = &regs->gaddr5;
		priv->hash_regs[6] = &regs->gaddr6;
		priv->hash_regs[7] = &regs->gaddr7;
	}

	if (priv->device_flags & FSL_GIANFAR_DEV_HAS_PADDING)
		priv->padding = DEFAULT_PADDING;
	else
		priv->padding = 0;

	if (dev->features & NETIF_F_IP_CSUM  || priv->ptimer_present) {
		priv->padding = 0x8;
		dev->hard_header_len += GMAC_FCB_LEN;
	}

	/* Program the isrg regs only if number of grps > 1 */
	if (priv->num_grps > 1) {
		baddr = &regs->isrg0;
		for (i = 0; i < priv->num_grps; i++) {
			isrg |= (priv->gfargrp[i].rx_bit_map << ISRG_SHIFT_RX);
			isrg |= (priv->gfargrp[i].tx_bit_map << ISRG_SHIFT_TX);
			gfar_write(baddr, isrg);
			baddr++;
			isrg = 0x0;
		}
	}

	/* Need to reverse the bit maps as  bit_map's MSB is q0
	 * but, for_each_set_bit parses from right to left, which
	 * basically reverses the queue numbers */
	for (i = 0; i< priv->num_grps; i++) {
		priv->gfargrp[i].tx_bit_map = reverse_bitmap(
				priv->gfargrp[i].tx_bit_map, MAX_TX_QS);
		priv->gfargrp[i].rx_bit_map = reverse_bitmap(
				priv->gfargrp[i].rx_bit_map, MAX_RX_QS);
	}

	/* Calculate RSTAT, TSTAT, RQUEUE and TQUEUE values,
	 * also assign queues to groups */
	for (grp_idx = 0; grp_idx < priv->num_grps; grp_idx++) {
		priv->gfargrp[grp_idx].num_rx_queues = 0x0;
		for_each_set_bit(i, &priv->gfargrp[grp_idx].rx_bit_map,
				priv->num_rx_queues) {
			priv->gfargrp[grp_idx].num_rx_queues++;
			priv->rx_queue[i]->grp = &priv->gfargrp[grp_idx];
			rstat = rstat | (RSTAT_CLEAR_RHALT >> i);
			rqueue = rqueue | ((RQUEUE_EN0 | RQUEUE_EX0) >> i);
		}
		priv->gfargrp[grp_idx].num_tx_queues = 0x0;
		for_each_set_bit(i, &priv->gfargrp[grp_idx].tx_bit_map,
				priv->num_tx_queues) {
			priv->gfargrp[grp_idx].num_tx_queues++;
			priv->tx_queue[i]->grp = &priv->gfargrp[grp_idx];
			tstat = tstat | (TSTAT_CLEAR_THALT >> i);
			tqueue = tqueue | (TQUEUE_EN0 >> i);
		}
		priv->gfargrp[grp_idx].rstat = rstat;
		priv->gfargrp[grp_idx].tstat = tstat;
		rstat = tstat =0;
	}

	gfar_write(&regs->rqueue, rqueue);
	gfar_write(&regs->tqueue, tqueue);

	priv->rx_buffer_size = DEFAULT_RX_BUFFER_SIZE;
	priv->wk_buffer_size = DEFAULT_WK_BUFFER_SIZE;

	/* Initializing some of the rx/tx queue level parameters */
	for (i = 0; i < priv->num_tx_queues; i++) {
		priv->tx_queue[i]->tx_ring_size = DEFAULT_TX_RING_SIZE;
		priv->tx_queue[i]->num_txbdfree = DEFAULT_TX_RING_SIZE;
		priv->tx_queue[i]->txcoalescing = DEFAULT_TX_COALESCE;
		priv->tx_queue[i]->txic = DEFAULT_TXIC;
	}

	priv->rx_queue[priv->num_rx_queues - 1]->rx_ring_size = DEFAULT_WK_RING_SIZE;

	/* enable filer if using multiple RX queues*/
	if (priv->num_rx_queues > 1)
		priv->rx_filer_enable = 1;

	for (i = 0; i < priv->num_rx_queues; i++) {
		priv->rx_queue[i]->rx_ring_size = DEFAULT_RX_RING_SIZE;
		priv->rx_queue[i]->rxcoalescing = DEFAULT_RX_COALESCE;
		priv->rx_queue[i]->rxic = DEFAULT_RXIC;
	}

	/* Enable most messages by default */
	priv->msg_enable = (NETIF_MSG_IFUP << 1 ) - 1;

	/* Carrier starts down, phylib will bring it up */
	netif_carrier_off(dev);

	err = register_netdev(dev);

#ifdef CONFIG_WRHV
	if (wrhv_nic_start > wrhv_nic_num) {
		printk(KERN_ERR " WRHV: bootline NIC setup error\n");
		return -ENODEV;
	}

	for (j = wrhv_nic_start; j <= wrhv_nic_num; j++) {
		char nic_num[NIC_STR_LEN] = "";
		char net_sub_name[NIC_STR_LEN]; /* eth on most platforms except
		on cavium they call there network devices mgmtX */

		if (!is_valid_ether_addr(wrhv_macaddr))
				break;
		else if (!wrhv_net_name[0])
			strcpy(wrhv_net_name, "eth0");

		/* eth0 --> we only want the eth part so we
		   Simply append to the ifname the nic number */

		/* Clear out the buffer for the next iteration */
		memset(net_sub_name, '\0', NIC_STR_LEN);

		/* get rid of the number on the end 'ethX' */
		strncpy(net_sub_name, wrhv_net_name, strlen(wrhv_net_name) - 1);
		sprintf(nic_num, "%d", j);
		strcat(net_sub_name, nic_num);

		if (strcmp(dev->name, net_sub_name) == 0) {
			char local_mac[MAC_ADDR_LEN];
			char last_byte = wrhv_macaddr[MAC_ADDR_LEN - 1];

			memcpy(local_mac, wrhv_macaddr, MAC_ADDR_LEN);
			/* Extract the last byte of the MAC address */
			/* One limitation is that we do not check to see if
			the last byte needs to wrap around.  We expect sane
			values are being passed in.  This limitation has been
			documented in the README.bootline */

			local_mac[MAC_ADDR_LEN - 1] = (last_byte + j) & 0xff;
			memcpy(dev->dev_addr, local_mac, MAC_ADDR_LEN);
		}
	}
#endif

	if (err) {
		printk(KERN_ERR "%s: Cannot register net device, aborting.\n",
				dev->name);
		goto register_fail;
	}

	if ((priv->device_flags & FSL_GIANFAR_DEV_HAS_MAGIC_PACKET) ||
	    (priv->device_flags & FSL_GIANFAR_DEV_HAS_ARP_PACKET)) {
		device_set_wakeup_capable(&ofdev->dev, true);
		device_set_wakeup_enable(&ofdev->dev, false);
	}	

	/* fill out IRQ number and name fields */
	len_devname = strlen(dev->name);
	for (i = 0; i < priv->num_grps; i++) {
		strncpy(&priv->gfargrp[i].int_name_tx[0], dev->name,
				len_devname);
		if (priv->device_flags & FSL_GIANFAR_DEV_HAS_MULTI_INTR) {
			strncpy(&priv->gfargrp[i].int_name_tx[len_devname],
				"_g", sizeof("_g"));
			priv->gfargrp[i].int_name_tx[
				strlen(priv->gfargrp[i].int_name_tx)] = i+48;
			strncpy(&priv->gfargrp[i].int_name_tx[strlen(
				priv->gfargrp[i].int_name_tx)],
				"_tx", sizeof("_tx") + 1);

			strncpy(&priv->gfargrp[i].int_name_rx[0], dev->name,
					len_devname);
			strncpy(&priv->gfargrp[i].int_name_rx[len_devname],
					"_g", sizeof("_g"));
			priv->gfargrp[i].int_name_rx[
				strlen(priv->gfargrp[i].int_name_rx)] = i+48;
			strncpy(&priv->gfargrp[i].int_name_rx[strlen(
				priv->gfargrp[i].int_name_rx)],
				"_rx", sizeof("_rx") + 1);

			strncpy(&priv->gfargrp[i].int_name_er[0], dev->name,
					len_devname);
			strncpy(&priv->gfargrp[i].int_name_er[len_devname],
				"_g", sizeof("_g"));
			priv->gfargrp[i].int_name_er[strlen(
					priv->gfargrp[i].int_name_er)] = i+48;
			strncpy(&priv->gfargrp[i].int_name_er[strlen(\
				priv->gfargrp[i].int_name_er)],
				"_er", sizeof("_er") + 1);
		} else
			priv->gfargrp[i].int_name_tx[len_devname] = '\0';
	}

	/* Initialize the filer table */
	gfar_init_filer_table(priv);

	/* Create all the sysfs files */
	gfar_init_sysfs(dev);

	/* Print out the device info */
	printk(KERN_INFO DEVICE_NAME "%pM\n", dev->name, dev->dev_addr);

	/* Even more device info helps when determining which kernel */
	/* provided which set of benchmarks. */
	printk(KERN_INFO "%s: Running with NAPI enabled\n", dev->name);
	for (i = 0; i < priv->num_rx_queues; i++)
		printk(KERN_INFO "%s: RX BD ring size for Q[%d]: %d\n",
			dev->name, i, priv->rx_queue[i]->rx_ring_size);
	for(i = 0; i < priv->num_tx_queues; i++)
		 printk(KERN_INFO "%s: TX BD ring size for Q[%d]: %d\n",
			dev->name, i, priv->tx_queue[i]->tx_ring_size);

	return 0;

register_fail:
	if (priv->ptimer_present)
		gfar_ptp_cleanup(priv);
	unmap_group_regs(priv);
	free_tx_pointers(priv);
	free_rx_pointers(priv);
	if (priv->phy_node)
		of_node_put(priv->phy_node);
	if (priv->tbi_node)
		of_node_put(priv->tbi_node);
	free_netdev(dev);
	return err;
}

static int gfar_remove(struct of_device *ofdev)
{
	struct gfar_private *priv = dev_get_drvdata(&ofdev->dev);

	if (priv->phy_node)
		of_node_put(priv->phy_node);
	if (priv->tbi_node)
		of_node_put(priv->tbi_node);

	dev_set_drvdata(&ofdev->dev, NULL);

	unregister_netdev(priv->ndev);
	unmap_group_regs(priv);

	kfree(priv->ftp_rqfpr);
	kfree(priv->ftp_rqfcr);
	free_netdev(priv->ndev);
	return 0;
}

#ifdef CONFIG_PM
static void gfar_enable_filer(struct net_device *dev)
{
	struct gfar_private *priv = netdev_priv(dev);
	struct gfar __iomem *regs = priv->gfargrp[0].regs;
	u32 temp;

	lock_rx_qs(priv);

	temp = gfar_read(&regs->rctrl);
	temp |= RCTRL_FILREN;
	temp &= ~RCTRL_FSQEN;
	temp &= ~RCTRL_PRSDEP_MASK;
	temp |= RCTRL_PRSDEP_L2L3;
	gfar_write(&regs->rctrl, temp);

	unlock_rx_qs(priv);
}

static int gfar_get_ip(struct net_device *dev)
{
	struct gfar_private *priv = netdev_priv(dev);
	struct in_device *in_dev = (struct in_device *)dev->ip_ptr;
	struct in_ifaddr *ifa;

	if (in_dev != NULL) {
		ifa = (struct in_ifaddr *)in_dev->ifa_list;
		if (ifa != NULL) {
			memcpy(priv->ip_addr, &ifa->ifa_address, 4);
			return 0;
		}
	}
	return -ENOENT;
}

static void gfar_config_filer_table(struct net_device *dev)
{
	struct gfar_private *priv = netdev_priv(dev);
	u8 *ip_addr;
	u32 wakeup_ip, dest_mac_addr_h, dest_mac_addr_l;
	u32 rqfpr = 0x0;
	u32 rqfcr = RQFCR_RJE | RQFCR_CMP_MATCH;
	u8  rqfcr_queue = priv->num_rx_queues - 1;
	int i;

	if (gfar_get_ip(dev))
		printk(KERN_ERR "WOL: get the ip address error\n");
	ip_addr = priv->ip_addr;

	wakeup_ip = (*ip_addr << 24) | (*(ip_addr + 1) << 16) | \
		    (*(ip_addr + 2) << 8) | (*(ip_addr + 3));

	dest_mac_addr_h = (dev->dev_addr[0] << 16) | \
			  (dev->dev_addr[1] << 8) | dev->dev_addr[2];
	dest_mac_addr_l = (dev->dev_addr[3] << 16) | \
			  (dev->dev_addr[4] << 8) | dev->dev_addr[5];

	lock_rx_qs(priv);

	for (i = 0; i <= priv->max_filer_rules; i++)
		gfar_write_filer(priv, i, rqfcr, rqfpr);

	/* ARP request filer, filling the packet to queue #1 */
	rqfcr = (rqfcr_queue << 10) | RQFCR_AND | RQFCR_CMP_EXACT | RQFCR_PID_MASK;
	rqfpr = RQFPR_ARQ;
	gfar_write_filer(priv, 0, rqfcr, rqfpr);

	rqfcr = (rqfcr_queue << 10) | RQFCR_AND | RQFCR_CMP_EXACT | RQFCR_PID_PARSE;
	rqfpr = RQFPR_ARQ;
	gfar_write_filer(priv, 1, rqfcr, rqfpr);

	/* DEST_IP address in ARP packet, filling it to queue #1 */
	rqfcr = (rqfcr_queue << 10) | RQFCR_AND | RQFCR_CMP_EXACT | RQFCR_PID_MASK;
	rqfpr = FPR_FILER_MASK;
	gfar_write_filer(priv, 2, rqfcr, rqfpr);

	rqfcr = RQFCR_GPI | (rqfcr_queue << 10) | RQFCR_CMP_EXACT | RQFCR_PID_DIA;
	rqfpr = wakeup_ip;
	gfar_write_filer(priv, 3, rqfcr, rqfpr);

	/* Unicast packet, filling it to queue #1 */
	rqfcr = (rqfcr_queue << 10) | RQFCR_AND | RQFCR_CMP_EXACT | RQFCR_PID_DAH;
	rqfpr = dest_mac_addr_h;
	gfar_write_filer(priv, 4, rqfcr, rqfpr);

	rqfcr = RQFCR_GPI | (rqfcr_queue << 10) | RQFCR_CMP_EXACT | RQFCR_PID_DAL;
	mb();
	rqfpr = dest_mac_addr_l;
	gfar_write_filer(priv, 5, rqfcr, rqfpr);

	unlock_rx_qs(priv);
}

static int gfar_arp_suspend(struct net_device *dev)
{
	struct gfar_private *priv = netdev_priv(dev);
	struct gfar __iomem *regs = priv->gfargrp[0].regs;
	unsigned long flags;
	u32 tempval;

	netif_device_detach(dev);

	if (netif_running(dev)) {
		local_irq_save(flags);
		lock_tx_qs(priv);
		lock_rx_qs(priv);

		gfar_halt_tx_nodisable(dev);

		/* Disable Tx */
		tempval = gfar_read(&regs->maccfg1);
		tempval &= ~MACCFG1_TX_EN;
		gfar_write(&regs->maccfg1, tempval);

		unlock_rx_qs(priv);
		unlock_tx_qs(priv);
		local_irq_restore(flags);

		disable_napi(priv);

		gfar_halt_rx(dev);
		gfar_config_filer_table(dev);
		gfar_enable_filer(dev);
		gfar_rx_start(dev);
	}

	return 0;
}


static int gfar_suspend(struct device *dev)
{
	struct gfar_private *priv = dev_get_drvdata(dev);
	struct net_device *ndev = priv->ndev;
	struct gfar __iomem *regs = priv->gfargrp[0].regs;
	unsigned long flags;
	u32 tempval;

	int magic_packet = priv->wol_en &&
		(priv->wol_opts & GIANFAR_WOL_MAGIC) &&
		(priv->device_flags & FSL_GIANFAR_DEV_HAS_MAGIC_PACKET);
	int arp_packet = priv->wol_en &&
		(priv->wol_opts & GIANFAR_WOL_ARP) &&
		(priv->device_flags & FSL_GIANFAR_DEV_HAS_ARP_PACKET);

	if (arp_packet) {
		pmc_enable_wake(priv->ofdev, PM_SUSPEND_MEM, 1);
		pmc_enable_lossless(1);
		gfar_arp_suspend(ndev);
		return 0;
	}

	netif_device_detach(ndev);

	if (netif_running(ndev)) {

		local_irq_save(flags);
		lock_tx_qs(priv);
		lock_rx_qs(priv);

		gfar_halt_nodisable(ndev);

		/* Disable Tx, and Rx if wake-on-LAN is disabled. */
		tempval = gfar_read(&regs->maccfg1);

		tempval &= ~MACCFG1_TX_EN;

		if (!magic_packet)
			tempval &= ~MACCFG1_RX_EN;

		gfar_write(&regs->maccfg1, tempval);

		unlock_rx_qs(priv);
		unlock_tx_qs(priv);
		local_irq_restore(flags);

		disable_napi(priv);

		if (magic_packet) {
			pmc_enable_wake(priv->ofdev, PM_SUSPEND_MEM, 1);
			/* Enable interrupt on Magic Packet */
			gfar_write(&regs->imask, IMASK_MAG);

			/* Enable Magic Packet mode */
			tempval = gfar_read(&regs->maccfg2);
			tempval |= MACCFG2_MPEN;
			gfar_write(&regs->maccfg2, tempval);
		} else {
			phy_stop(priv->phydev);
		}
	}

	return 0;
}

static int gfar_arp_resume(struct net_device *dev)
{
	struct gfar_private *priv = netdev_priv(dev);

	if (!netif_running(dev)) {
		netif_device_attach(dev);
		return 0;
	}

	gfar_tx_start(dev);
	stop_gfar(dev);
	gfar_halt_rx(dev);
	gfar_init_filer_table(priv);
	startup_gfar(dev);
	gfar_rx_start(dev);

	netif_device_attach(dev);
	enable_napi(priv);

	return 0;
}

static int gfar_resume(struct device *dev)
{
	struct gfar_private *priv = dev_get_drvdata(dev);
	struct net_device *ndev = priv->ndev;
	struct gfar __iomem *regs = priv->gfargrp[0].regs;
	unsigned long flags;
	u32 tempval;

	int magic_packet = priv->wol_en &&
		(priv->wol_opts & GIANFAR_WOL_MAGIC) &&
		(priv->device_flags & FSL_GIANFAR_DEV_HAS_MAGIC_PACKET);
	int arp_packet = priv->wol_en &&
		(priv->wol_opts & GIANFAR_WOL_ARP) &&
		(priv->device_flags & FSL_GIANFAR_DEV_HAS_ARP_PACKET);

	if (arp_packet) {
		pmc_enable_wake(priv->ofdev, PM_SUSPEND_MEM, 0);
		pmc_enable_lossless(0);
		gfar_arp_resume(ndev);
		return 0;
	} else if (magic_packet) {
		pmc_enable_wake(priv->ofdev, PM_SUSPEND_MEM, 0);
	}

	if (!netif_running(ndev)) {
		netif_device_attach(ndev);
		return 0;
	}

	if (!magic_packet && priv->phydev)
		phy_start(priv->phydev);

	/* Disable Magic Packet mode, in case something
	 * else woke us up.
	 */
	local_irq_save(flags);
	lock_tx_qs(priv);
	lock_rx_qs(priv);

	tempval = gfar_read(&regs->maccfg2);
	tempval &= ~MACCFG2_MPEN;
	gfar_write(&regs->maccfg2, tempval);

	gfar_start(ndev);

	unlock_rx_qs(priv);
	unlock_tx_qs(priv);
	local_irq_restore(flags);

	netif_device_attach(ndev);

	enable_napi(priv);

	return 0;
}

static int gfar_restore(struct device *dev)
{
	struct gfar_private *priv = dev_get_drvdata(dev);
	struct net_device *ndev = priv->ndev;

	if (!netif_running(ndev))
		return 0;

	gfar_init_bds(ndev);
	init_registers(ndev);
	gfar_set_mac_address(ndev);
	gfar_init_mac(ndev);
	gfar_start(ndev);

	priv->oldlink = 0;
	priv->oldspeed = 0;
	priv->oldduplex = -1;

	if (priv->phydev)
		phy_start(priv->phydev);

	netif_device_attach(ndev);
	enable_napi(priv);

	return 0;
}

static struct dev_pm_ops gfar_pm_ops = {
	.suspend = gfar_suspend,
	.resume = gfar_resume,
	.freeze = gfar_suspend,
	.thaw = gfar_resume,
	.restore = gfar_restore,
};

#define GFAR_PM_OPS (&gfar_pm_ops)

static int gfar_legacy_suspend(struct of_device *ofdev, pm_message_t state)
{
	return gfar_suspend(&ofdev->dev);
}

static int gfar_legacy_resume(struct of_device *ofdev)
{
	return gfar_resume(&ofdev->dev);
}

#else

#define GFAR_PM_OPS NULL
#define gfar_legacy_suspend NULL
#define gfar_legacy_resume NULL

#endif

/* Reads the controller's registers to determine what interface
 * connects it to the PHY.
 */
static phy_interface_t gfar_get_interface(struct net_device *dev)
{
	struct gfar_private *priv = netdev_priv(dev);
	struct gfar __iomem *regs = priv->gfargrp[0].regs;
	u32 ecntrl;

	ecntrl = gfar_read(&regs->ecntrl);

	if (ecntrl & ECNTRL_SGMII_MODE)
		return PHY_INTERFACE_MODE_SGMII;

	if (ecntrl & ECNTRL_TBI_MODE) {
		if (ecntrl & ECNTRL_REDUCED_MODE)
			return PHY_INTERFACE_MODE_RTBI;
		else
			return PHY_INTERFACE_MODE_TBI;
	}

	if (ecntrl & ECNTRL_REDUCED_MODE) {
		if (ecntrl & ECNTRL_REDUCED_MII_MODE)
			return PHY_INTERFACE_MODE_RMII;
		else {
			phy_interface_t interface = priv->interface;

			/*
			 * This isn't autodetected right now, so it must
			 * be set by the device tree or platform code.
			 */
			if (interface == PHY_INTERFACE_MODE_RGMII_ID)
				return PHY_INTERFACE_MODE_RGMII_ID;

			return PHY_INTERFACE_MODE_RGMII;
		}
	}

	if (priv->device_flags & FSL_GIANFAR_DEV_HAS_GIGABIT)
		return PHY_INTERFACE_MODE_GMII;

	return PHY_INTERFACE_MODE_MII;
}


/* Initializes driver's PHY state, and attaches to the PHY.
 * Returns 0 on success.
 */
static int init_phy(struct net_device *dev)
{
	struct gfar_private *priv = netdev_priv(dev);
	uint gigabit_support =
		priv->device_flags & FSL_GIANFAR_DEV_HAS_GIGABIT ?
		SUPPORTED_1000baseT_Full : 0;
	phy_interface_t interface;

	priv->oldlink = 0;
	priv->oldspeed = 0;
	priv->oldduplex = -1;

	interface = gfar_get_interface(dev);

	priv->phydev = of_phy_connect(dev, priv->phy_node, &adjust_link, 0,
				      interface);
	if (!priv->phydev)
		priv->phydev = of_phy_connect_fixed_link(dev, &adjust_link,
							 interface);
	if (!priv->phydev) {
		dev_err(&dev->dev, "could not attach to PHY\n");
		return -ENODEV;
	}

	if (interface == PHY_INTERFACE_MODE_SGMII)
		gfar_configure_serdes(dev);

	/* Remove any features not supported by the controller */
	priv->phydev->supported &= (GFAR_SUPPORTED | gigabit_support);
	priv->phydev->advertising = priv->phydev->supported;

	return 0;
}

/*
 * Initialize TBI PHY interface for communicating with the
 * SERDES lynx PHY on the chip.  We communicate with this PHY
 * through the MDIO bus on each controller, treating it as a
 * "normal" PHY at the address found in the TBIPA register.  We assume
 * that the TBIPA register is valid.  Either the MDIO bus code will set
 * it to a value that doesn't conflict with other PHYs on the bus, or the
 * value doesn't matter, as there are no other PHYs on the bus.
 */
static void gfar_configure_serdes(struct net_device *dev)
{
	struct gfar_private *priv = netdev_priv(dev);
	struct phy_device *tbiphy;

	if (!priv->tbi_node) {
		dev_warn(&dev->dev, "error: SGMII mode requires that the "
				    "device tree specify a tbi-handle\n");
		return;
	}

	tbiphy = of_phy_find_device(priv->tbi_node);
	if (!tbiphy) {
		dev_err(&dev->dev, "error: Could not get TBI device\n");
		return;
	}

	/*
	 * If the link is already up, we must already be ok, and don't need to
	 * configure and reset the TBI<->SerDes link.  Maybe U-Boot configured
	 * everything for us?  Resetting it takes the link down and requires
	 * several seconds for it to come back.
	 */
	if (phy_read(tbiphy, MII_BMSR) & BMSR_LSTATUS)
		return;

	/* Single clk mode, mii mode off(for serdes communication) */
	phy_write(tbiphy, MII_TBICON, TBICON_CLK_SELECT);

	phy_write(tbiphy, MII_ADVERTISE,
			ADVERTISE_1000XFULL | ADVERTISE_1000XPAUSE |
			ADVERTISE_1000XPSE_ASYM);

	phy_write(tbiphy, MII_BMCR, BMCR_ANENABLE |
			BMCR_ANRESTART | BMCR_FULLDPLX | BMCR_SPEED1000);
}

static void init_registers(struct net_device *dev)
{
	struct gfar_private *priv = netdev_priv(dev);
	struct gfar __iomem *regs = NULL;
	int i = 0;

	for (i = 0; i < priv->num_grps; i++) {
		regs = priv->gfargrp[i].regs;
		/* Clear IEVENT */
		gfar_write(&regs->ievent, IEVENT_INIT_CLEAR);

		/* Initialize IMASK */
		gfar_write(&regs->imask, IMASK_INIT_CLEAR);
	}

	regs = priv->gfargrp[0].regs;
	/* Init hash registers to zero */
	gfar_write(&regs->igaddr0, 0);
	gfar_write(&regs->igaddr1, 0);
	gfar_write(&regs->igaddr2, 0);
	gfar_write(&regs->igaddr3, 0);
	gfar_write(&regs->igaddr4, 0);
	gfar_write(&regs->igaddr5, 0);
	gfar_write(&regs->igaddr6, 0);
	gfar_write(&regs->igaddr7, 0);

	gfar_write(&regs->gaddr0, 0);
	gfar_write(&regs->gaddr1, 0);
	gfar_write(&regs->gaddr2, 0);
	gfar_write(&regs->gaddr3, 0);
	gfar_write(&regs->gaddr4, 0);
	gfar_write(&regs->gaddr5, 0);
	gfar_write(&regs->gaddr6, 0);
	gfar_write(&regs->gaddr7, 0);

	/* Zero out the rmon mib registers if it has them */
	if (priv->device_flags & FSL_GIANFAR_DEV_HAS_RMON) {
		memset_io(&(regs->rmon), 0, sizeof (struct rmon_mib));

		/* Mask off the CAM interrupts */
		gfar_write(&regs->rmon.cam1, 0xffffffff);
		gfar_write(&regs->rmon.cam2, 0xffffffff);
	}

	/* Initialize the max receive buffer length */
	gfar_write(&regs->mrblr, priv->rx_buffer_size);

	/* Initialize the Minimum Frame Length Register */
	gfar_write(&regs->minflr, MINFLR_INIT_SETTINGS);
}

static int __gfar_is_rx_idle(struct gfar_private *priv)
{
	u32 res;

	/*
	 * Normaly TSEC should not hang on GRS commands, so we should
	 * actually wait for IEVENT_GRSC flag.
	 */
	if (likely(!gfar_has_errata(priv, GFAR_ERRATA_A002)))
		return 0;

	/*
	 * Read the eTSEC register at offset 0xD1C. If bits 7-14 are
	 * the same as bits 23-30, the eTSEC Rx is assumed to be idle
	 * and the Rx can be safely reset.
	 */
	res = gfar_read((void __iomem *)priv->gfargrp[0].regs + 0xd1c);
	res &= 0x7f807f80;
	if ((res & 0xffff) == (res >> 16))
		return 1;

	return 0;
}

/* Halt the receive and transmit queues */
static void gfar_halt_nodisable(struct net_device *dev)
{
	struct gfar_private *priv = netdev_priv(dev);
	struct gfar __iomem *regs = NULL;
	u32 tempval;
	int i = 0;

	for (i = 0; i < priv->num_grps; i++) {
		regs = priv->gfargrp[i].regs;
		/* Mask all interrupts */
		gfar_write(&regs->imask, IMASK_INIT_CLEAR);

		/* Clear all interrupts */
		gfar_write(&regs->ievent, IEVENT_INIT_CLEAR);
	}

	regs = priv->gfargrp[0].regs;
	/* Stop the DMA, and wait for it to stop */
	tempval = gfar_read(&regs->dmactrl);
	if ((tempval & (DMACTRL_GRS | DMACTRL_GTS))
	    != (DMACTRL_GRS | DMACTRL_GTS)) {
		int ret;

		tempval |= (DMACTRL_GRS | DMACTRL_GTS);
		gfar_write(&regs->dmactrl, tempval);

		do {
			ret = spin_event_timeout(((gfar_read(&regs->ievent) &
				 (IEVENT_GRSC | IEVENT_GTSC)) ==
				 (IEVENT_GRSC | IEVENT_GTSC)), 1000000, 0);
			if (!ret && !(gfar_read(&regs->ievent) & IEVENT_GRSC))
				ret = __gfar_is_rx_idle(priv);
		} while (!ret);
	}
}

#ifdef CONFIG_PM
/* Halt the receive queues */
static void gfar_halt_rx(struct net_device *dev)
{
	struct gfar_private *priv = netdev_priv(dev);
	struct gfar __iomem *regs = priv->gfargrp[0].regs;
	u32 tempval;
	int i = 0;

	/* Disable Rx in MACCFG1  */
	tempval = gfar_read(&regs->maccfg1);
	tempval &= ~MACCFG1_RX_EN;
	gfar_write(&regs->maccfg1, tempval);

	for (i = 0; i < priv->num_grps; i++) {
		regs = priv->gfargrp[i].regs;
		/* Mask all interrupts */
		gfar_write(&regs->imask, IMASK_INIT_CLEAR | IMASK_FGPI);

		/* Clear all interrupts */
		gfar_write(&regs->ievent, IEVENT_INIT_CLEAR);
	}

	regs = priv->gfargrp[0].regs;
	/* Stop the DMA, and wait for it to stop */
	tempval = gfar_read(&regs->dmactrl);
	if ((tempval & DMACTRL_GRS) != DMACTRL_GRS) {
		tempval |= DMACTRL_GRS;
		gfar_write(&regs->dmactrl, tempval);

		while (!(gfar_read(&regs->ievent) & IEVENT_GRSC))
			cpu_relax();
	}
}

/* Halt the transmit queues */
static void gfar_halt_tx_nodisable(struct net_device *dev)
{
	struct gfar_private *priv = netdev_priv(dev);
	struct gfar __iomem *regs = NULL;
	u32 tempval;
	int i = 0;

	for (i = 0; i < priv->num_grps; i++) {
		regs = priv->gfargrp[i].regs;
		/* Mask all interrupts */
		gfar_write(&regs->imask, IMASK_INIT_CLEAR | IMASK_FGPI);

		/* Clear all interrupts */
		gfar_write(&regs->ievent, IEVENT_INIT_CLEAR);
	}

	regs = priv->gfargrp[0].regs;
	/* Stop the DMA, and wait for it to stop */
	tempval = gfar_read(&regs->dmactrl);
	if ((tempval & DMACTRL_GTS) != DMACTRL_GTS) {
		tempval |= DMACTRL_GTS;
		gfar_write(&regs->dmactrl, tempval);

		while (!(gfar_read(&regs->ievent) & IEVENT_GTSC))
			cpu_relax();
	}
}
#endif

/* Halt the receive and transmit queues */
void gfar_halt(struct net_device *dev)
{
	struct gfar_private *priv = netdev_priv(dev);
	struct gfar __iomem *regs = priv->gfargrp[0].regs;
	u32 tempval;

	gfar_halt_nodisable(dev);

	/* Disable Rx and Tx */
	tempval = gfar_read(&regs->maccfg1);
	tempval &= ~(MACCFG1_RX_EN | MACCFG1_TX_EN);
	gfar_write(&regs->maccfg1, tempval);
}

static void free_grp_irqs(struct gfar_priv_grp *grp)
{
	free_irq(grp->interruptError, grp);
	free_irq(grp->interruptTransmit, grp);
	free_irq(grp->interruptReceive, grp);
}

void free_bds(struct gfar_private *priv)
{
#ifdef CONFIG_GIANFAR_L2SRAM
	if (priv->bd_in_ram) {
		dma_free_coherent(&priv->ofdev->dev,
			sizeof(struct txbd8) * priv->total_tx_ring_size +
			sizeof(struct rxbd8) * priv->total_rx_ring_size,
			priv->tx_queue[0]->tx_bd_base,
			priv->tx_queue[0]->tx_bd_dma_base);
	} else {
		mpc85xx_cache_sram_free(priv->tx_queue[0]->tx_bd_base);
	}
#else
	dma_free_coherent(&priv->ofdev->dev,
			sizeof(struct txbd8) * priv->total_tx_ring_size +
			sizeof(struct rxbd8) * priv->total_rx_ring_size,
			priv->tx_queue[0]->tx_bd_base,
			priv->tx_queue[0]->tx_bd_dma_base);
#endif
}

void stop_gfar(struct net_device *dev)
{
	struct gfar_private *priv = netdev_priv(dev);
	unsigned long flags;
	int i;

	phy_stop(priv->phydev);


	/* Lock it down */
	local_irq_save(flags);
	lock_tx_qs(priv);
	lock_rx_qs(priv);

	gfar_halt(dev);

	unlock_rx_qs(priv);
	unlock_tx_qs(priv);
	local_irq_restore(flags);

	if (priv->ptimer_present && !eth_counter)
		gfar_1588_stop(dev);

	/* Free the IRQs */
	if (priv->device_flags & FSL_GIANFAR_DEV_HAS_MULTI_INTR) {
		for (i = 0; i < priv->num_grps; i++)
			free_grp_irqs(&priv->gfargrp[i]);
	} else {
		for (i = 0; i < priv->num_grps; i++)
			free_irq(priv->gfargrp[i].interruptTransmit,
					&priv->gfargrp[i]);
	}

	free_skb_resources(priv);
}

static void free_skb_tx_queue(struct gfar_priv_tx_q *tx_queue)
{
	struct txbd8 *txbdp;
	struct gfar_private *priv = netdev_priv(tx_queue->dev);
	int i, j;

	txbdp = tx_queue->tx_bd_base;

	for (i = 0; i < tx_queue->tx_ring_size; i++) {
		if (!tx_queue->tx_skbuff[i])
			continue;

		dma_unmap_single(&priv->ofdev->dev, txbdp->bufPtr,
				txbdp->length, DMA_TO_DEVICE);
		txbdp->lstatus = 0;
		for (j = 0; j < skb_shinfo(tx_queue->tx_skbuff[i])->nr_frags;
				j++) {
			txbdp++;
			dma_unmap_page(&priv->ofdev->dev, txbdp->bufPtr,
					txbdp->length, DMA_TO_DEVICE);
		}
		txbdp++;
		dev_kfree_skb_any(tx_queue->tx_skbuff[i]);
		tx_queue->tx_skbuff[i] = NULL;
	}
#ifndef CONFIG_GIANFAR_L2SRAM
	kfree(tx_queue->tx_skbuff);
#endif
}

static void free_skb_rx_queue(struct gfar_priv_rx_q *rx_queue)
{
	struct rxbd8 *rxbdp;
	struct gfar_private *priv = netdev_priv(rx_queue->dev);
	int i;

	rxbdp = rx_queue->rx_bd_base;

	for (i = 0; i < rx_queue->rx_ring_size; i++) {
		if (rx_queue->rx_skbuff[i]) {
			dma_unmap_single(&priv->ofdev->dev,
					rxbdp->bufPtr, priv->rx_buffer_size,
					DMA_FROM_DEVICE);
			dev_kfree_skb_any(rx_queue->rx_skbuff[i]);
			rx_queue->rx_skbuff[i] = NULL;
		}
		rxbdp->lstatus = 0;
		rxbdp->bufPtr = 0;
		rxbdp++;
	}
#ifndef CONFIG_GIANFAR_L2SRAM
	kfree(rx_queue->rx_skbuff);
#endif
}

/* If there are any tx skbs or rx skbs still around, free them.
 * Then free tx_skbuff and rx_skbuff */
static void free_skb_resources(struct gfar_private *priv)
{
	struct gfar_priv_tx_q *tx_queue = NULL;
	struct gfar_priv_rx_q *rx_queue = NULL;
	int i;

	/* Go through all the buffer descriptors and free their data buffers */
	for (i = 0; i < priv->num_tx_queues; i++) {
		tx_queue = priv->tx_queue[i];
		if(tx_queue->tx_skbuff)
			free_skb_tx_queue(tx_queue);
	}

	for (i = 0; i < priv->num_rx_queues; i++) {
		rx_queue = priv->rx_queue[i];
		if(rx_queue->rx_skbuff)
			free_skb_rx_queue(rx_queue);
	}

	free_bds(priv);
	skb_queue_purge(&priv->rx_recycle);
}

void gfar_start(struct net_device *dev)
{
	struct gfar_private *priv = netdev_priv(dev);
	struct gfar __iomem *regs = priv->gfargrp[0].regs;
	u32 tempval;
	int i = 0;

	/* Enable Rx and Tx in MACCFG1 */
	tempval = gfar_read(&regs->maccfg1);
	tempval |= (MACCFG1_RX_EN | MACCFG1_TX_EN);
	gfar_write(&regs->maccfg1, tempval);

	/* Initialize DMACTRL to have WWR and WOP */
	tempval = gfar_read(&regs->dmactrl);
	tempval |= DMACTRL_INIT_SETTINGS;
	gfar_write(&regs->dmactrl, tempval);

	/* Make sure we aren't stopped */
	tempval = gfar_read(&regs->dmactrl);
	tempval &= ~(DMACTRL_GRS | DMACTRL_GTS);
	gfar_write(&regs->dmactrl, tempval);

	for (i = 0; i < priv->num_grps; i++) {
		regs = priv->gfargrp[i].regs;
		/* Clear THLT/RHLT, so that the DMA starts polling now */
		gfar_write(&regs->tstat, priv->gfargrp[i].tstat);
		gfar_write(&regs->rstat, priv->gfargrp[i].rstat);
		/* Unmask the interrupts we look for */
		gfar_write(&regs->imask, IMASK_DEFAULT);
	}

	dev->trans_start = jiffies;
}

#ifdef CONFIG_PM
void gfar_rx_start(struct net_device *dev)
{
	struct gfar_private *priv = netdev_priv(dev);
	struct gfar __iomem *regs = priv->gfargrp[0].regs;
	u32 tempval;
	int i = 0;

	/* Enable Rx in MACCFG1 */
	tempval = gfar_read(&regs->maccfg1);
	tempval |= MACCFG1_RX_EN;
	gfar_write(&regs->maccfg1, tempval);

	/* Make sure we aren't stopped */
	tempval = gfar_read(&regs->dmactrl);
	tempval &= ~DMACTRL_GRS;
	gfar_write(&regs->dmactrl, tempval);

	for (i = 0; i < priv->num_grps; i++) {
		regs = priv->gfargrp[i].regs;
		/* Clear RHLT, so that the DMA starts polling now */
		gfar_write(&regs->rstat, priv->gfargrp[i].rstat);
	}
}

void gfar_tx_start(struct net_device *dev)
{
	struct gfar_private *priv = netdev_priv(dev);
	struct gfar __iomem *regs = priv->gfargrp[0].regs;
	u32 tempval;
	int i = 0;

	/* Enable Tx in MACCFG1 */
	tempval = gfar_read(&regs->maccfg1);
	tempval |= MACCFG1_TX_EN;
	gfar_write(&regs->maccfg1, tempval);

	/* Make sure we aren't stopped */
	tempval = gfar_read(&regs->dmactrl);
	tempval &= ~DMACTRL_GTS;
	gfar_write(&regs->dmactrl, tempval);

	for (i = 0; i < priv->num_grps; i++) {
		regs = priv->gfargrp[i].regs;
		/* Clear THLT, so that the DMA starts polling now */
		gfar_write(&regs->rstat, priv->gfargrp[i].tstat);
	}
}
#endif

void gfar_configure_tx_coalescing(struct gfar_private *priv,
				long unsigned int tx_mask)
{
	struct gfar __iomem *regs = priv->gfargrp[0].regs;
	u32 __iomem *baddr;
	int i = 0, mask = 0x1;

	/* Backward compatible case ---- even if we enable
	 * multiple queues, there's only single reg to program
	 */
	if (priv->mode == SQ_SG_MODE) {
		gfar_write(&regs->txic, 0);
		if (likely(priv->tx_queue[0]->txcoalescing))
			gfar_write(&regs->txic, priv->tx_queue[0]->txic);
	}

	if (priv->mode == MQ_MG_MODE) {
		baddr = &regs->txic0;
		for (i = 0; i < priv->num_tx_queues; i++) {
			if (tx_mask & mask) {
				if (likely(priv->tx_queue[i]->txcoalescing)) {
					gfar_write(baddr + i, 0);
					gfar_write(baddr + i,
						 priv->tx_queue[i]->txic);
				}
			}
			mask = mask << 0x1;
		}
	}
}

void gfar_configure_rx_coalescing(struct gfar_private *priv,
				long unsigned int rx_mask)
{
	struct gfar __iomem *regs = priv->gfargrp[0].regs;
	u32 __iomem *baddr;
	int i = 0, mask = 0x1;

	/* Backward compatible case ---- even if we enable
	 * multiple queues, there's only single reg to program
	 */
	if (priv->mode == SQ_SG_MODE) {
		gfar_write(&regs->rxic, 0);
		if (unlikely(priv->rx_queue[0]->rxcoalescing))
			gfar_write(&regs->rxic, priv->rx_queue[0]->rxic);
	}

	if (priv->mode == MQ_MG_MODE) {
		baddr = &regs->rxic0;
		for (i = 0; i < priv->num_rx_queues; i++) {
			if (rx_mask & mask) {
				if (likely(priv->rx_queue[i]->rxcoalescing)) {
					gfar_write(baddr + i, 0);
					gfar_write(baddr + i,
						priv->rx_queue[i]->rxic);
				}
			}
			mask = mask << 0x1;
		}
	}
}

static int register_grp_irqs(struct gfar_priv_grp *grp)
{
	struct gfar_private *priv = grp->priv;
	struct net_device *dev = priv->ndev;
	int err;

	/* If the device has multiple interrupts, register for
	 * them.  Otherwise, only register for the one */
	if (priv->device_flags & FSL_GIANFAR_DEV_HAS_MULTI_INTR) {
		/* Install our interrupt handlers for Error,
		 * Transmit, and Receive */
		if ((err = request_irq(grp->interruptError, gfar_error, 0,
				grp->int_name_er,grp)) < 0) {
			if (netif_msg_intr(priv))
				printk(KERN_ERR "%s: Can't get IRQ %d\n",
					dev->name, grp->interruptError);

				goto err_irq_fail;
		}

		if ((err = request_irq(grp->interruptTransmit, gfar_transmit,
				0, grp->int_name_tx, grp)) < 0) {
			if (netif_msg_intr(priv))
				printk(KERN_ERR "%s: Can't get IRQ %d\n",
					dev->name, grp->interruptTransmit);
			goto tx_irq_fail;
		}

		if ((err = request_irq(grp->interruptReceive, gfar_receive, 0,
				grp->int_name_rx, grp)) < 0) {
			if (netif_msg_intr(priv))
				printk(KERN_ERR "%s: Can't get IRQ %d\n",
					dev->name, grp->interruptReceive);
			goto rx_irq_fail;
		}
	} else {
		if ((err = request_irq(grp->interruptTransmit, gfar_interrupt, 0,
				grp->int_name_tx, grp)) < 0) {
			if (netif_msg_intr(priv))
				printk(KERN_ERR "%s: Can't get IRQ %d\n",
					dev->name, grp->interruptTransmit);
			goto err_irq_fail;
		}
	}

	return 0;

rx_irq_fail:
	free_irq(grp->interruptTransmit, grp);
tx_irq_fail:
	free_irq(grp->interruptError, grp);
err_irq_fail:
	return err;

}

/* Bring the controller up and running */
int startup_gfar(struct net_device *ndev)
{
	struct gfar_private *priv = netdev_priv(ndev);
	struct gfar __iomem *regs = NULL;
	int err, i, j;

	for (i = 0; i < priv->num_grps; i++) {
		regs= priv->gfargrp[i].regs;
		gfar_write(&regs->imask, IMASK_INIT_CLEAR);
	}

	regs= priv->gfargrp[0].regs;
	err = gfar_alloc_skb_resources(ndev);
	if (err)
		return err;

	gfar_init_mac(ndev);

	for (i = 0; i < priv->num_grps; i++) {
		err = register_grp_irqs(&priv->gfargrp[i]);
		if (err) {
			for (j = 0; j < i; j++)
				free_grp_irqs(&priv->gfargrp[j]);
				goto irq_fail;
		}
	}

	/* Start the controller */
	gfar_start(ndev);

	phy_start(priv->phydev);

	gfar_configure_tx_coalescing(priv, 0xFF);
	gfar_configure_rx_coalescing(priv, 0xFF);

	return 0;

irq_fail:
	free_skb_resources(priv);
	return err;
}

/* Called when something needs to use the ethernet device */
/* Returns 0 for success. */
static int gfar_enet_open(struct net_device *dev)
{
	struct gfar_private *priv = netdev_priv(dev);
	int err;

	enable_napi(priv);

	skb_queue_head_init(&priv->rx_recycle);

	/* Initialize a bunch of registers */
	init_registers(dev);

	gfar_set_mac_address(dev);

	err = init_phy(dev);

	if (err) {
		disable_napi(priv);
		return err;
	}

	err = startup_gfar(dev);
	if (err) {
		disable_napi(priv);
		return err;
	}

	eth_counter++;

	netif_tx_start_all_queues(dev);

	device_set_wakeup_enable(&priv->ofdev->dev, priv->wol_en);

	return err;
}

static inline struct txfcb *gfar_add_fcb(struct sk_buff *skb)
{
	struct txfcb *fcb = (struct txfcb *)skb_push(skb, GMAC_FCB_LEN);

	memset(fcb, 0, GMAC_FCB_LEN);

	return fcb;
}

static inline void gfar_tx_checksum(struct sk_buff *skb, struct txfcb *fcb)
{
	u8 flags = 0;

	/* If we're here, it's a IP packet with a TCP or UDP
	 * payload.  We set it to checksum, using a pseudo-header
	 * we provide
	 */
	flags = TXFCB_DEFAULT;

	/* Tell the controller what the protocol is */
	/* And provide the already calculated phcs */
	if (!((ip_hdr(skb)->frag_off) & htons(IP_MF|IP_OFFSET))) {
		/* If not fragmented packet */
		if (ip_hdr(skb)->protocol == IPPROTO_UDP) {
			if (udp_hdr(skb)->check) {
				fcb->phcs = udp_hdr(skb)->check;
				flags |= TXFCB_NPH;
			}
			flags |= TXFCB_UDP | TXFCB_TUP | TXFCB_CTU;
		} else if (ip_hdr(skb)->protocol == IPPROTO_TCP) {
			if (tcp_hdr(skb)->check) {
				flags |= TXFCB_NPH;
				fcb->phcs = tcp_hdr(skb)->check;
			}
			flags |= TXFCB_TUP | TXFCB_CTU;
		}
	}

	/* l3os is the distance between the start of the
	 * frame (skb->data) and the start of the IP hdr.
	 * l4os is the distance between the start of the
	 * l3 hdr and the l4 hdr */
	fcb->l3os = (u16)(skb_network_offset(skb) - GMAC_FCB_LEN);
	fcb->l4os = skb_network_header_len(skb);

	fcb->flags = flags;
}

void inline gfar_tx_vlan(struct sk_buff *skb, struct txfcb *fcb)
{
	fcb->flags |= TXFCB_VLN;
	fcb->vlctl = vlan_tx_tag_get(skb);
}

static inline struct txbd8 *skip_txbd(struct txbd8 *bdp, int stride,
			       struct txbd8 *base, int ring_size)
{
	struct txbd8 *new_bd = bdp + stride;

	return (new_bd >= (base + ring_size)) ? (new_bd - ring_size) : new_bd;
}

static inline struct txbd8 *next_txbd(struct txbd8 *bdp, struct txbd8 *base,
		int ring_size)
{
	return skip_txbd(bdp, 1, base, ring_size);
}

static int gfar_xmit_skb(struct sk_buff *skb, struct net_device *dev, int rq)
{
		struct gfar_private *priv = netdev_priv(dev);
		struct gfar_priv_tx_q *tx_queue = NULL;
		struct netdev_queue *txq;
		struct gfar __iomem *regs = NULL;
		struct txfcb *fcb = NULL;
		struct txbd8 *txbdp, *txbdp_start, *base;
		u32 lstatus;
		int i;
		u32 bufaddr;
		unsigned long flags;
		unsigned int nr_frags, length;

		tx_queue = priv->tx_queue[rq];
		txq = netdev_get_tx_queue(dev, rq);
		base = tx_queue->tx_bd_base;
		regs = tx_queue->grp->regs;

		/* total number of fragments in the SKB */
		nr_frags = skb_shinfo(skb)->nr_frags;

		/* check if there is space to queue this packet */
		if ((nr_frags+1) > tx_queue->num_txbdfree) {
			/* no space, stop the queue */
			netif_tx_stop_queue(txq);
			dev->stats.tx_fifo_errors++;
			return NETDEV_TX_BUSY;
		}

		/* Update transmit stats */
		txq->tx_bytes += skb->len;
		txq->tx_packets++;

		txbdp = txbdp_start = tx_queue->cur_tx;

		if (nr_frags == 0) {
			lstatus = txbdp->lstatus | BD_LFLAG(TXBD_LAST | TXBD_INTERRUPT);
		} else {
			/* Place the fragment addresses and lengths into the TxBDs */
			for (i = 0; i < nr_frags; i++) {
				/* Point at the next BD, wrapping as needed */
				txbdp = next_txbd(txbdp, base, tx_queue->tx_ring_size);

				length = skb_shinfo(skb)->frags[i].size;

				lstatus = txbdp->lstatus | length |
					BD_LFLAG(TXBD_READY);

				/* Handle the last BD specially */
				if (i == nr_frags - 1)
					lstatus |= BD_LFLAG(TXBD_LAST | TXBD_INTERRUPT);

				bufaddr = dma_map_page(&priv->ofdev->dev,
						skb_shinfo(skb)->frags[i].page,
						skb_shinfo(skb)->frags[i].page_offset,
						length,
						DMA_TO_DEVICE);

				/* set the TxBD length and buffer pointer */
				txbdp->bufPtr = bufaddr;
				txbdp->lstatus = lstatus;
			}

			lstatus = txbdp_start->lstatus;
		}

		/* Set up checksumming */
		if (CHECKSUM_PARTIAL == skb->ip_summed) {
			fcb = gfar_add_fcb(skb);
			lstatus |= BD_LFLAG(TXBD_TOE);
			gfar_tx_checksum(skb, fcb);
		}

		if (priv->vlgrp && vlan_tx_tag_present(skb)) {
			if (unlikely(NULL == fcb)) {
				fcb = gfar_add_fcb(skb);
				lstatus |= BD_LFLAG(TXBD_TOE);
			}

			gfar_tx_vlan(skb, fcb);
		}

		/* setup the TxBD length and buffer pointer for the first BD */
		tx_queue->tx_skbuff[tx_queue->skb_curtx] = skb;
		txbdp_start->bufPtr = dma_map_single(&priv->ofdev->dev, skb->data,
				skb_headlen(skb), DMA_TO_DEVICE);

		lstatus |= BD_LFLAG(TXBD_CRC | TXBD_READY) | skb_headlen(skb);

		/*
		* We can work in parallel with gfar_clean_tx_ring(), except
		* when modifying num_txbdfree. Note that we didn't grab the lock
		* when we were reading the num_txbdfree and checking for available
		* space, that's because outside of this function it can only grow,
		* and once we've got needed space, it cannot suddenly disappear.
		*
		* The lock also protects us from gfar_error(), which can modify
		* regs->tstat and thus retrigger the transfers, which is why we
		* also must grab the lock before setting ready bit for the first
		* to be transmitted BD.
		*/
		spin_lock_irqsave(&tx_queue->txlock, flags);

		/*
		 * The powerpc-specific eieio() is used, as wmb() has too strong
		 * semantics (it requires synchronization between cacheable and
		 * uncacheable mappings, which eieio doesn't provide and which we
		 * don't need), thus requiring a more expensive sync instruction.  At
		 * some point, the set of architecture-independent barrier functions
		 * should be expanded to include weaker barriers.
		 */
		eieio();

		txbdp_start->lstatus = lstatus;

		/* Update the current skb pointer to the next entry we will use
		 * (wrapping if necessary) */
		tx_queue->skb_curtx = (tx_queue->skb_curtx + 1) &
			TX_RING_MOD_MASK(tx_queue->tx_ring_size);

		tx_queue->cur_tx = next_txbd(txbdp, base, tx_queue->tx_ring_size);

		/* reduce TxBD free count */
		tx_queue->num_txbdfree -= (nr_frags + 1);

		txq->trans_start = jiffies;

		/* If the next BD still needs to be cleaned up, then the bds
		   are full.  We need to tell the kernel to stop sending us stuff. */
		if (!tx_queue->num_txbdfree) {
			netif_stop_subqueue(dev, tx_queue->qindex);

			dev->stats.tx_fifo_errors++;
		}

		/* Tell the DMA to go go go */
		gfar_write(&regs->tstat, TSTAT_CLEAR_THALT >> tx_queue->qindex);

		/* Unlock priv */
		spin_unlock_irqrestore(&tx_queue->txlock, flags);

		return NETDEV_TX_OK;

}

/*software TCP segmentation offload*/
static int gfar_tso(struct sk_buff *skb, struct net_device *dev, int rq)
{
	int i = 0;
	struct iphdr *iph;
	int ihl;
	int id;
	unsigned int offset = 0;
	struct tcphdr *th;
	unsigned thlen;
	unsigned int seq;
	__be32 delta;
	unsigned int oldlen;
	unsigned int mss;
	unsigned int doffset;
	unsigned int headroom;
	unsigned int len;
	int nfrags;
	int pos;
	int hsize;
	int ret;

	/*processing mac header*/
	skb_reset_mac_header(skb);
	skb->mac_len = skb->network_header - skb->mac_header;
	__skb_pull(skb, skb->mac_len);

	/*processing IP header*/
	iph = ip_hdr(skb);
	ihl = iph->ihl * 4;
	__skb_pull(skb, ihl);
	skb_reset_transport_header(skb);
	iph = ip_hdr(skb);
	id = ntohs(iph->id);

	/*processing TCP header*/
	th = tcp_hdr(skb);
	thlen = th->doff * 4;
	oldlen = (u16)~skb->len;
	__skb_pull(skb, thlen);
	mss = skb_shinfo(skb)->gso_size;
	seq = ntohl(th->seq);
	delta = htonl(oldlen + (thlen + mss));

	/*processing SKB*/
	doffset = skb->data - skb_mac_header(skb);
	offset = doffset;
	nfrags = skb_shinfo(skb)->nr_frags;
	__skb_push(skb, doffset);
	headroom = skb_headroom(skb);
	pos = skb_headlen(skb);

	/*duplicating SKB*/
	hsize = skb_headlen(skb) - offset;
	if (hsize < 0)
		hsize = 0;

	do {
		struct sk_buff *nskb;
		skb_frag_t *frag;
		int size;

		len = skb->len - offset;
		if (len > mss)
			len = mss;

		nskb = alloc_skb(hsize + doffset + headroom,
					 GFP_ATOMIC);
		skb_reserve(nskb, headroom);
		__skb_put(nskb, doffset+hsize);

		nskb->ip_summed = skb->ip_summed;
		nskb->vlan_tci = skb->vlan_tci;
		nskb->mac_len = skb->mac_len;

		skb_reset_mac_header(nskb);
		skb_set_network_header(nskb, skb->mac_len);
		nskb->transport_header = (nskb->network_header +
					  skb_network_header_len(skb));
		skb_copy_from_linear_data(skb, nskb->data, doffset+hsize);
		frag = skb_shinfo(nskb)->frags;

		/*move skb data*/
		while (pos < offset + len && i < nfrags) {
			*frag = skb_shinfo(skb)->frags[i];
			get_page(frag->page);
			size = frag->size;

			if (pos < offset) {
				frag->page_offset += offset - pos;
				frag->size -= offset - pos;
			}

			skb_shinfo(nskb)->nr_frags++;

			if (pos + size <= offset + len) {
				i++;
				pos += size;
			} else {
				frag->size -= pos + size - (offset + len);
				goto skip_fraglist;
			}

			frag++;
		}

skip_fraglist:
		nskb->data_len = len - hsize;
		nskb->len += nskb->data_len;

		/*update TCP header*/
		if ((offset + len) >= skb->len)
			delta = htonl(oldlen + (nskb->tail -
				nskb->transport_header) + nskb->data_len);

		th = tcp_hdr(nskb);
		th->fin = th->psh = 0;
		th->seq = htonl(seq);
		th->cwr = 0;
		seq += mss;
		th->check = ~csum_fold((__force __wsum)((__force u32)th->check
				+ (__force u32)delta));

		/*update IP header*/
		iph = ip_hdr(nskb);
		iph->id = htons(id++);
		iph->tot_len = htons(nskb->len - nskb->mac_len);
		iph->check = 0;
		iph->check = ip_fast_csum(skb_network_header(nskb), iph->ihl);
		ret = gfar_xmit_skb(nskb, dev, rq);
		if (unlikely(ret != NETDEV_TX_OK)) {
			skb = nskb;
			goto out_tso;
		}
	} while ((offset += len) < skb->len);

out_tso:
	dev_kfree_skb_any(skb);
	return ret;
}

/* This is called by the kernel when a frame is ready for transmission. */
/* It is pointed to by the dev->hard_start_xmit function pointer */
static int gfar_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct gfar_private *priv = netdev_priv(dev);
	struct gfar_priv_tx_q *tx_queue = NULL;
	struct netdev_queue *txq;
	struct gfar __iomem *regs = NULL;
	struct txfcb *fcb = NULL;
	struct txbd8 *txbdp, *txbdp_start, *base;
	u32 lstatus;
	int i, rq = 0;
	u32 bufaddr;
	unsigned long flags;
	unsigned int nr_frags, length;

#ifdef CONFIG_AS_FASTPATH
	if (devfp_tx_hook && (skb->pkt_type != PACKET_FASTROUTE))
		if (devfp_tx_hook(skb, dev) == AS_FP_STOLEN)
			return 0;
#endif

	/*
	 * TOE=1 frames larger than 2500 bytes may see excess delays
	 * before start of transmission.
	 */
	if (unlikely(gfar_has_errata(priv, GFAR_ERRATA_76) &&
			skb->ip_summed == CHECKSUM_PARTIAL &&
			skb->len > 2500)) {
		int ret;

		ret = skb_checksum_help(skb);
		if (ret)
			return ret;
	}

	rq = skb->queue_mapping;
	tx_queue = priv->tx_queue[rq];
	txq = netdev_get_tx_queue(dev, rq);
	base = tx_queue->tx_bd_base;
	regs = tx_queue->grp->regs;

	/* make space for additional header when fcb is needed */
	if (((skb->ip_summed == CHECKSUM_PARTIAL) ||
			(priv->vlgrp && vlan_tx_tag_present(skb))) &&
			(skb_headroom(skb) < GMAC_FCB_LEN)) {
		struct sk_buff *skb_new;

		skb_new = skb_realloc_headroom(skb, GMAC_FCB_LEN);
		if (!skb_new) {
			dev->stats.tx_errors++;
			kfree_skb(skb);
			return NETDEV_TX_OK;
		}
		kfree_skb(skb);
		skb = skb_new;
	}

	if (skb_shinfo(skb)->gso_size)
		return gfar_tso(skb, dev, rq);

	/* total number of fragments in the SKB */
	nr_frags = skb_shinfo(skb)->nr_frags;

	/* check if there is space to queue this packet */
	if ((nr_frags+1) > tx_queue->num_txbdfree) {
		/* no space, stop the queue */
		netif_tx_stop_queue(txq);
		dev->stats.tx_fifo_errors++;
		return NETDEV_TX_BUSY;
	}

	/* Update transmit stats */
	txq->tx_bytes += skb->len;
	txq->tx_packets ++;

	txbdp = txbdp_start = tx_queue->cur_tx;

	if (nr_frags == 0) {
		lstatus = txbdp->lstatus | BD_LFLAG(TXBD_LAST | TXBD_INTERRUPT);
	} else {
		/* Place the fragment addresses and lengths into the TxBDs */
		for (i = 0; i < nr_frags; i++) {
			/* Point at the next BD, wrapping as needed */
			txbdp = next_txbd(txbdp, base, tx_queue->tx_ring_size);

			length = skb_shinfo(skb)->frags[i].size;

			lstatus = txbdp->lstatus | length |
				BD_LFLAG(TXBD_READY);

			/* Handle the last BD specially */
			if (i == nr_frags - 1)
				lstatus |= BD_LFLAG(TXBD_LAST | TXBD_INTERRUPT);

			bufaddr = dma_map_page(&priv->ofdev->dev,
					skb_shinfo(skb)->frags[i].page,
					skb_shinfo(skb)->frags[i].page_offset,
					length,
					DMA_TO_DEVICE);

			/* set the TxBD length and buffer pointer */
			txbdp->bufPtr = bufaddr;
			txbdp->lstatus = lstatus;
		}

		lstatus = txbdp_start->lstatus;
	}

	/* Set up checksumming */
	if (CHECKSUM_PARTIAL == skb->ip_summed) {
		fcb = gfar_add_fcb(skb);
		lstatus |= BD_LFLAG(TXBD_TOE);
		gfar_tx_checksum(skb, fcb);
	}

	if (priv->vlgrp && vlan_tx_tag_present(skb)) {
		if (unlikely(NULL == fcb)) {
			fcb = gfar_add_fcb(skb);
			lstatus |= BD_LFLAG(TXBD_TOE);
		}

		gfar_tx_vlan(skb, fcb);
	}

	if (priv->ptimer_present) {
		/* Enable ptp flag so that Tx time stamping happens */
		if (gfar_ptp_do_txstamp(skb)) {
			if (fcb == NULL)
				fcb = gfar_add_fcb(skb);
			fcb->ptp = 0x01;
			lstatus |= BD_LFLAG(TXBD_TOE);
		}
	}

	/* setup the TxBD length and buffer pointer for the first BD */
	txbdp_start->bufPtr = dma_map_single(&priv->ofdev->dev, skb->data,
			skb_headlen(skb), DMA_TO_DEVICE);

	lstatus |= BD_LFLAG(TXBD_CRC | TXBD_READY) | skb_headlen(skb);

	/*
	 * We can work in parallel with gfar_clean_tx_ring(), except
	 * when modifying num_txbdfree. Note that we didn't grab the lock
	 * when we were reading the num_txbdfree and checking for available
	 * space, that's because outside of this function it can only grow,
	 * and once we've got needed space, it cannot suddenly disappear.
	 *
	 * The lock also protects us from gfar_error(), which can modify
	 * regs->tstat and thus retrigger the transfers, which is why we
	 * also must grab the lock before setting ready bit for the first
	 * to be transmitted BD.
	 */
	spin_lock_irqsave(&tx_queue->txlock, flags);

	/*
	 * The powerpc-specific eieio() is used, as wmb() has too strong
	 * semantics (it requires synchronization between cacheable and
	 * uncacheable mappings, which eieio doesn't provide and which we
	 * don't need), thus requiring a more expensive sync instruction.  At
	 * some point, the set of architecture-independent barrier functions
	 * should be expanded to include weaker barriers.
	 */
	eieio();

	txbdp_start->lstatus = lstatus;

	eieio(); /* force lstatus write before tx_skbuff */

	tx_queue->tx_skbuff[tx_queue->skb_curtx] = skb;

	/* Update the current skb pointer to the next entry we will use
	 * (wrapping if necessary) */
	tx_queue->skb_curtx = (tx_queue->skb_curtx + 1) &
		TX_RING_MOD_MASK(tx_queue->tx_ring_size);

	tx_queue->cur_tx = next_txbd(txbdp, base, tx_queue->tx_ring_size);

	/* reduce TxBD free count */
	tx_queue->num_txbdfree -= (nr_frags + 1);

	dev->trans_start = jiffies;

	/* If the next BD still needs to be cleaned up, then the bds
	   are full.  We need to tell the kernel to stop sending us stuff. */
	if (!tx_queue->num_txbdfree) {
		netif_tx_stop_queue(txq);

		dev->stats.tx_fifo_errors++;
	}

	/* Tell the DMA to go go go */
	gfar_write(&regs->tstat, TSTAT_CLEAR_THALT >> tx_queue->qindex);

	/* Unlock priv */
	spin_unlock_irqrestore(&tx_queue->txlock, flags);

	return NETDEV_TX_OK;
}

/* Stops the kernel queue, and halts the controller */
static int gfar_close(struct net_device *dev)
{
	struct gfar_private *priv = netdev_priv(dev);

	eth_counter--;

	disable_napi(priv);

	cancel_work_sync(&priv->reset_task);
	stop_gfar(dev);

	/* Disconnect from the PHY */
	phy_disconnect(priv->phydev);
	priv->phydev = NULL;

	netif_tx_stop_all_queues(dev);

	return 0;
}

/* Changes the mac address if the controller is not running. */
static int gfar_set_mac_address(struct net_device *dev)
{
	gfar_set_mac_for_addr(dev, 0, dev->dev_addr);

	return 0;
}


/* Enables and disables VLAN insertion/extraction */
static void gfar_vlan_rx_register(struct net_device *dev,
		struct vlan_group *grp)
{
	struct gfar_private *priv = netdev_priv(dev);
	struct gfar __iomem *regs = NULL;
	unsigned long flags;
	u32 tempval;

	regs = priv->gfargrp[0].regs;
	local_irq_save(flags);
	lock_rx_qs(priv);

	priv->vlgrp = grp;

	if (grp) {
		/* Enable VLAN tag insertion */
		tempval = gfar_read(&regs->tctrl);
		tempval |= TCTRL_VLINS;

		gfar_write(&regs->tctrl, tempval);

		/* Enable VLAN tag extraction */
		tempval = gfar_read(&regs->rctrl);
		tempval |= (RCTRL_VLEX | RCTRL_PRSDEP_INIT);
		gfar_write(&regs->rctrl, tempval);
	} else {
		/* Disable VLAN tag insertion */
		tempval = gfar_read(&regs->tctrl);
		tempval &= ~TCTRL_VLINS;
		gfar_write(&regs->tctrl, tempval);

		/* Disable VLAN tag extraction */
		tempval = gfar_read(&regs->rctrl);
		tempval &= ~RCTRL_VLEX;
		/* If parse is no longer required, then disable parser */
		if (tempval & RCTRL_REQ_PARSER)
			tempval |= RCTRL_PRSDEP_INIT;
		else
			tempval &= ~RCTRL_PRSDEP_INIT;
		gfar_write(&regs->rctrl, tempval);
	}

	gfar_change_mtu(dev, dev->mtu);

	unlock_rx_qs(priv);
	local_irq_restore(flags);
}

static int gfar_change_mtu(struct net_device *dev, int new_mtu)
{
	int tempsize, tempval;
	struct gfar_private *priv = netdev_priv(dev);
	struct gfar __iomem *regs = priv->gfargrp[0].regs;
	int oldsize = priv->rx_buffer_size;
	int frame_size = new_mtu + ETH_HLEN;

	if ((new_mtu < 68) || (new_mtu > JUMBO_FRAME_SIZE)) {
		if (netif_msg_drv(priv))
			printk(KERN_ERR "%s: Invalid MTU setting\n",
					dev->name);
		return -EINVAL;
	}

	if (priv->vlgrp)
		frame_size += VLAN_HLEN;

	if (gfar_uses_fcb(priv))
		frame_size += GMAC_FCB_LEN;

	frame_size += priv->padding;

	tempsize =
	    (frame_size & ~(INCREMENTAL_BUFFER_SIZE - 1)) +
	    INCREMENTAL_BUFFER_SIZE;

	if (tempsize > JUMBO_BUFFER_SIZE)
		tempsize = JUMBO_BUFFER_SIZE;

	/* Only stop and start the controller if it isn't already
	 * stopped, and we changed something */
	if ((oldsize != tempsize) && (dev->flags & IFF_UP))
		stop_gfar(dev);

	priv->rx_buffer_size = tempsize;

	dev->mtu = new_mtu;

	gfar_write(&regs->mrblr, priv->rx_buffer_size);
	gfar_write(&regs->maxfrm, priv->rx_buffer_size);

	/* If the mtu is larger than the max size for standard
	 * ethernet frames (ie, a jumbo frame), then set maccfg2
	 * to allow huge frames, and to check the length */
	tempval = gfar_read(&regs->maccfg2);

	if (priv->rx_buffer_size > DEFAULT_RX_BUFFER_SIZE ||
			gfar_has_errata(priv, GFAR_ERRATA_74))
		tempval |= (MACCFG2_HUGEFRAME | MACCFG2_LENGTHCHECK);
	else
		tempval &= ~(MACCFG2_HUGEFRAME | MACCFG2_LENGTHCHECK);

	gfar_write(&regs->maccfg2, tempval);

	if ((oldsize != tempsize) && (dev->flags & IFF_UP))
		startup_gfar(dev);

	return 0;
}

/* gfar_reset_task gets scheduled when a packet has not been
 * transmitted after a set amount of time.
 * For now, assume that clearing out all the structures, and
 * starting over will fix the problem.
 */
static void gfar_reset_task(struct work_struct *work)
{
	struct gfar_private *priv = container_of(work, struct gfar_private,
			reset_task);
	struct net_device *dev = priv->ndev;

	if (dev->flags & IFF_UP) {
		netif_tx_stop_all_queues(dev);
		stop_gfar(dev);
		startup_gfar(dev);
		netif_tx_start_all_queues(dev);
	}

	netif_tx_schedule_all(dev);
}

static void gfar_timeout(struct net_device *dev)
{
	struct gfar_private *priv = netdev_priv(dev);

	dev->stats.tx_errors++;
	schedule_work(&priv->reset_task);
}

/* Interrupt Handler for Transmit complete */
#ifdef CONFIG_GIANFAR_TXNAPI
static int gfar_clean_tx_ring(struct gfar_priv_tx_q *tx_queue, int tx_work_limit)
#else
static int gfar_clean_tx_ring(struct gfar_priv_tx_q *tx_queue)
#endif
{
	struct net_device *dev = tx_queue->dev;
	struct gfar_private *priv = netdev_priv(dev);
	struct gfar_priv_rx_q *rx_queue = NULL;
	struct txbd8 *bdp;
	struct txbd8 *lbdp = NULL;
	struct txbd8 *base = tx_queue->tx_bd_base;
	struct sk_buff *skb;
	int skb_dirtytx;
	int tx_ring_size = tx_queue->tx_ring_size;
	int frags = 0;
	int i;
	int howmany = 0;
	u32 lstatus;

	rx_queue = priv->rx_queue[tx_queue->qindex];
	bdp = tx_queue->dirty_tx;
	skb_dirtytx = tx_queue->skb_dirtytx;

	while ((skb = tx_queue->tx_skbuff[skb_dirtytx])) {
		unsigned long flags;

		frags = skb_shinfo(skb)->nr_frags;
		lbdp = skip_txbd(bdp, frags, base, tx_ring_size);

		lstatus = lbdp->lstatus;

		/* Only clean completed frames */
		if ((lstatus & BD_LFLAG(TXBD_READY)) &&
				(lstatus & BD_LENGTH_MASK))
			break;

		dma_unmap_single(&priv->ofdev->dev,
				bdp->bufPtr,
				bdp->length,
				DMA_TO_DEVICE);

		bdp->lstatus &= BD_LFLAG(TXBD_WRAP);
		bdp = next_txbd(bdp, base, tx_ring_size);

		for (i = 0; i < frags; i++) {
			dma_unmap_page(&priv->ofdev->dev,
					bdp->bufPtr,
					bdp->length,
					DMA_TO_DEVICE);
			bdp->lstatus &= BD_LFLAG(TXBD_WRAP);
			bdp = next_txbd(bdp, base, tx_ring_size);
		}

		/*
		 * If there's room in the queue (limit it to rx_buffer_size)
		 * we add this skb back into the pool, if it's the right size
		 */
		if (skb_queue_len(&priv->rx_recycle) < rx_queue->rx_ring_size &&
				skb_recycle_check(skb, priv->rx_buffer_size +
					RXBUF_ALIGNMENT))
			skb_queue_head(&priv->rx_recycle, skb);
		else
			dev_kfree_skb_any(skb);

		tx_queue->tx_skbuff[skb_dirtytx] = NULL;

		skb_dirtytx = (skb_dirtytx + 1) &
			TX_RING_MOD_MASK(tx_ring_size);

		howmany++;
#ifndef CONFIG_GIANFAR_TXNAPI
		spin_lock_irqsave(&tx_queue->txlock, flags);
		tx_queue->num_txbdfree += frags + 1;
		spin_unlock_irqrestore(&tx_queue->txlock, flags);
#else
		tx_queue->num_txbdfree += frags + 1;
#endif
	}

	/* If we freed a buffer, we can restart transmission, if necessary */
	if (__netif_subqueue_stopped(dev, tx_queue->qindex) && tx_queue->num_txbdfree)
		netif_wake_subqueue(dev, tx_queue->qindex);

	/* Update dirty indicators */
	tx_queue->skb_dirtytx = skb_dirtytx;
	tx_queue->dirty_tx = bdp;

	return howmany;
}

#ifdef CONFIG_GIANFAR_TXNAPI
static void gfar_schedule_cleanup_rx(struct gfar_priv_grp *gfargrp)
{
	unsigned long flags;
	u32 imask = 0;

	spin_lock_irqsave(&gfargrp->grplock, flags);
	if (napi_schedule_prep(&gfargrp->napi_rx)) {
		imask = gfar_read(&gfargrp->regs->imask);
		imask = imask & IMASK_RX_DISABLED;
		gfar_write(&gfargrp->regs->imask, imask);
		__napi_schedule(&gfargrp->napi_rx);
	} else {
		gfar_write(&gfargrp->regs->ievent, IEVENT_RX_MASK);
	}
	spin_unlock_irqrestore(&gfargrp->grplock, flags);
}

static void gfar_schedule_cleanup_tx(struct gfar_priv_grp *gfargrp)
{
	unsigned long flags;
	u32 imask = 0;

	spin_lock_irqsave(&gfargrp->grplock, flags);
	if (napi_schedule_prep(&gfargrp->napi_tx)) {
		imask = gfar_read(&gfargrp->regs->imask);
		imask = imask & IMASK_TX_DISABLED;
		gfar_write(&gfargrp->regs->imask, imask);
		__napi_schedule(&gfargrp->napi_tx);
	} else {
		gfar_write(&gfargrp->regs->ievent, IEVENT_TX_MASK);
	}
	spin_unlock_irqrestore(&gfargrp->grplock, flags);
}
#else
static void gfar_schedule_cleanup(struct gfar_priv_grp *gfargrp)
{
	unsigned long flags;

	spin_lock_irqsave(&gfargrp->grplock, flags);
	if (napi_schedule_prep(&gfargrp->napi)) {
		gfar_write(&gfargrp->regs->imask, IMASK_RTX_DISABLED);
		__napi_schedule(&gfargrp->napi);
	} else {
		/*
		 * Clear IEVENT, so interrupts aren't called again
		 * because of the packets that have already arrived.
		 */
		gfar_write(&gfargrp->regs->ievent, IEVENT_RTX_MASK);
	}
	spin_unlock_irqrestore(&gfargrp->grplock, flags);

}
#endif

/* Interrupt Handler for Transmit complete */
static irqreturn_t gfar_transmit(int irq, void *grp_id)
{
#ifdef CONFIG_GIANFAR_TXNAPI
	gfar_schedule_cleanup_tx((struct gfar_priv_grp *)grp_id);
#else
#ifdef CONFIG_GFAR_TX_NONAPI
	struct gfar_priv_grp *grp = (struct gfar_priv_grp *)grp_id;
	struct gfar_private *priv = priv = grp->priv;
	unsigned int mask = TSTAT_TXF0_MASK;
	unsigned int tstat  = gfar_read(&grp->regs->tstat);
	int i;
	struct gfar_priv_tx_q *tx_queue = NULL;

	tstat = gfar_read(&grp->regs->tstat);
	tstat = tstat & TSTAT_TXF_MASK_ALL;
	/* Clear IEVENT */
	gfar_write(&grp->regs->ievent, IEVENT_TX_MASK);

	for (i = 0; i < priv->num_tx_queues; i++) {
		if (tstat & mask) {
			tx_queue = priv->tx_queue[i];
			gfar_clean_tx_ring(tx_queue);
		}
		mask = mask >> 0x1;
	}

	gfar_configure_tx_coalescing(priv, grp->tx_bit_map);
#else
	gfar_schedule_cleanup((struct gfar_priv_grp *)grp_id);
#endif
#endif
	return IRQ_HANDLED;
}

static void gfar_new_rxbdp(struct gfar_priv_rx_q *rx_queue, struct rxbd8 *bdp,
		struct sk_buff *skb)
{
	struct net_device *dev = rx_queue->dev;
	struct gfar_private *priv = netdev_priv(dev);
	dma_addr_t buf;

	buf = dma_map_single(&priv->ofdev->dev, skb->data,
			     priv->rx_buffer_size, DMA_FROM_DEVICE);
	gfar_init_rxbdp(rx_queue, bdp, buf);
}


struct sk_buff * gfar_new_skb(struct net_device *dev)
{
	unsigned int alignamount;
	struct gfar_private *priv = netdev_priv(dev);
	struct sk_buff *skb = NULL;

	skb = skb_dequeue(&priv->rx_recycle);
	if (!skb)
		skb = netdev_alloc_skb(dev,
				priv->rx_buffer_size + RXBUF_ALIGNMENT);

	if (!skb)
		return NULL;

	alignamount = RXBUF_ALIGNMENT -
		(((unsigned long) skb->data) & (RXBUF_ALIGNMENT - 1));

	/* We need the data buffer to be aligned properly.  We will reserve
	 * as many bytes as needed to align the data properly
	 * Do only if not already aligned
	 */
	if (alignamount != RXBUF_ALIGNMENT)
		skb_reserve(skb, alignamount);
	GFAR_CB(skb)->alignamount = alignamount;

	return skb;
}
EXPORT_SYMBOL(gfar_new_skb);

static inline void count_errors(unsigned short status, struct net_device *dev)
{
	struct gfar_private *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct gfar_extra_stats *estats = &priv->extra_stats;

	/* If the packet was truncated, none of the other errors
	 * matter */
	if (status & RXBD_TRUNCATED) {
		stats->rx_length_errors++;

		estats->rx_trunc++;

		return;
	}
	/* Count the errors, if there were any */
	if (status & (RXBD_LARGE | RXBD_SHORT)) {
		stats->rx_length_errors++;

		if (status & RXBD_LARGE)
			estats->rx_large++;
		else
			estats->rx_short++;
	}
	if (status & RXBD_NONOCTET) {
		stats->rx_frame_errors++;
		estats->rx_nonoctet++;
	}
	if (status & RXBD_CRCERR) {
		estats->rx_crcerr++;
		stats->rx_crc_errors++;
	}
	if (status & RXBD_OVERRUN) {
		estats->rx_overrun++;
		stats->rx_crc_errors++;
	}
}

static inline unsigned long __wk_phy_to_virt(struct net_device *dev,
				unsigned long phy)
{
	struct gfar_private *priv = netdev_priv(dev);
	unsigned long virt, offset;

	offset = phy - priv->wk_buf_align_paddr;
	virt = priv->wk_buf_align_vaddr + offset;
	return virt;
}

static void gfar_receive_wakeup(struct net_device *dev)
{
	struct gfar_private *priv = netdev_priv(dev);
	struct gfar_priv_rx_q *rx_queue = priv->rx_queue[priv->num_rx_queues-1];
	struct rxbd8 *bdp = rx_queue->cur_rx;
	struct sk_buff *skb;
	unsigned char *data;
	u16 len;
	int ret;

	/* get the first full descriptor */
	while (!(bdp->status & RXBD_EMPTY)) {
		rmb();
		if (bdp->status & RXBD_ERR) {
			printk(KERN_ERR "Wake up packet error!\n");
			goto out;
		}

		data = (u8 *)__wk_phy_to_virt(dev, bdp->bufPtr);
		len = bdp->length;
		/* allocate the skb */
		skb = netdev_alloc_skb(dev, len);
		if (!skb) {
			dev->stats.rx_dropped++;
			priv->extra_stats.rx_skbmissing++;
			goto out;
		}
		/* The wake up packet has the FCB */
		data += (GMAC_FCB_LEN + priv->padding);
		len -= (GMAC_FCB_LEN + priv->padding);
		/* remove the FCS from the packet length */
		len -= 4;
		/* copy received packet to skb buffer */
		memcpy(skb->data, data, len);
		/* Prep the skb for the packet */
		skb_put(skb, len);
		/* Tell the skb what kind of packet this is */
		skb->protocol = eth_type_trans(skb, dev);

		ret = netif_rx(skb);
		if (NET_RX_DROP == ret) {
			priv->extra_stats.kernel_dropped++;
		} else {
			/* Increment the number of packets */
			dev->stats.rx_packets++;
			dev->stats.rx_bytes += len;
		}

out:
		bdp->status &= RXBD_CLEAN;
		bdp->status |= RXBD_EMPTY;
		bdp->length = 0;

		mb();
		/* Update to the next pointer */
		if (bdp->status & RXBD_WRAP)
			bdp = priv->wk_bd_base;
		else
			bdp++;

	}
	rx_queue->cur_rx = bdp;
}

irqreturn_t gfar_receive(int irq, void *grp_id)
{
	struct gfar_priv_grp *gfargrp = grp_id;
	struct gfar __iomem *regs = gfargrp->regs;
	struct gfar_private *priv = gfargrp->priv;
	struct net_device *dev = priv->ndev;
	u32 ievent;

	ievent = gfar_read(&regs->ievent);

	if ((ievent & IEVENT_FGPI) == IEVENT_FGPI) {
		gfar_write(&regs->ievent, ievent & IEVENT_RX_MASK);
		gfar_receive_wakeup(dev);
		return IRQ_HANDLED;
	}

#ifdef CONFIG_GIANFAR_TXNAPI
	gfar_schedule_cleanup_rx((struct gfar_priv_grp *)grp_id);
#else
#ifdef CONFIG_GFAR_TX_NONAPI
	struct gfar_priv_grp *grp = (struct gfar_priv_grp *)grp_id;
	u32 tempval;

	/*
	 * Clear IEVENT, so interrupts aren't called again
	 * because of the packets that have already arrived.
	 */
	gfar_write(&grp->regs->ievent, IEVENT_RX_MASK);

	if (napi_schedule_prep(&grp->napi)) {
		tempval = gfar_read(&grp->regs->imask);
		tempval &= IMASK_RX_DISABLED;
		gfar_write(&grp->regs->imask, tempval);
		__napi_schedule(&grp->napi);
	} else {
		if (netif_msg_rx_err(grp->priv))
			printk(KERN_DEBUG "%s: receive called twice (%x)[%x]\n",
				dev->name, gfar_read(&grp->regs->ievent),
				gfar_read(&grp->regs->imask));
	}

#else
	gfar_schedule_cleanup((struct gfar_priv_grp *)grp_id);
#endif
#endif
	return IRQ_HANDLED;
}

static inline void gfar_rx_checksum(struct sk_buff *skb, struct rxfcb *fcb)
{
	/* If valid headers were found, and valid sums
	 * were verified, then we tell the kernel that no
	 * checksumming is necessary.  Otherwise, it is */
	if ((fcb->flags & RXFCB_CSUM_MASK) == (RXFCB_CIP | RXFCB_CTU))
		skb->ip_summed = CHECKSUM_UNNECESSARY;
	else
		skb->ip_summed = CHECKSUM_NONE;
}


/* gfar_process_frame() -- handle one incoming packet if skb
 * isn't NULL.  */
static int gfar_process_frame(struct net_device *dev, struct sk_buff *skb,
			      int amount_pull)
{
	struct gfar_private *priv = netdev_priv(dev);
	struct rxfcb *fcb = NULL;

	int ret;

	/* fcb is at the beginning if exists */
	fcb = (struct rxfcb *)skb->data;

	/* Remove the FCB from the skb */
	/* Remove the padded bytes, if there are any */
	if (amount_pull) {
		int queue_map;

		/* If FCB->QT field contains a value > num_rx_queues then
		   a direct mapping to virtual queues using QT as index will
		   result in a CRASH, when we are going to free the SKB.
		   So, need to map the QT value to existing virtual queues only.
		   using modulo by num_rx_queues.
		 */
		queue_map = fcb->rq - (priv->num_rx_queues *
				(fcb->rq/priv->num_rx_queues));
		skb_record_rx_queue(skb, queue_map);
		skb_pull(skb, amount_pull);
	}

	if (priv->ptimer_present) {
		gfar_ptp_store_rxstamp(dev, skb);
		skb_pull(skb, 8);
	}

	if (priv->rx_csum_enable)
		gfar_rx_checksum(skb, fcb);

#ifdef CONFIG_AS_FASTPATH
	if (devfp_rx_hook) {
		int drop = 0;

		/* Drop the packet silently if IP Checksum is not correct */
		if ((fcb->flags & RXFCB_CIP) && (fcb->flags & RXFCB_EIP)) {
			drop = 1;
			goto drop_pkt;
		}

		if (priv->vlgrp && (fcb->flags & RXFCB_VLN)) {
			struct net_device *vlan_dev = NULL;

			vlan_dev = vlan_group_get_device(priv->vlgrp,
					fcb->vlctl & VLAN_VID_MASK);

			if (vlan_dev) {
				skb->vlan_tci = fcb->vlctl;
				skb->dev = vlan_dev;
			} else {
				drop = 1;
			}
		} else {
			skb->dev = dev;
		}

drop_pkt:
		if (drop) {
			dev_kfree_skb_any(skb);
			return 0;
		}

		if (devfp_rx_hook(skb, dev) == AS_FP_STOLEN)
			return 0;
	}
#endif

	/* Tell the skb what kind of packet this is */
	skb->protocol = eth_type_trans(skb, dev);

	/* Send the packet up the stack */
	if (unlikely(priv->vlgrp && (fcb->flags & RXFCB_VLN)))
		ret = vlan_hwaccel_receive_skb(skb, priv->vlgrp, fcb->vlctl);
	else
		ret = netif_receive_skb(skb);

	if (NET_RX_DROP == ret)
		priv->extra_stats.kernel_dropped++;

	return 0;
}

/* gfar_clean_rx_ring() -- Processes each frame in the rx ring
 *   until the budget/quota has been reached. Returns the number
 *   of frames handled
 */
int gfar_clean_rx_ring(struct gfar_priv_rx_q *rx_queue, int rx_work_limit)
{
	struct net_device *dev = rx_queue->dev;
	struct rxbd8 *bdp, *base;
	struct sk_buff *skb;
	int pkt_len;
	int amount_pull;
	int howmany = 0;
	struct gfar_private *priv = netdev_priv(dev);

	/* Get the first full descriptor */
	bdp = rx_queue->cur_rx;
	base = rx_queue->rx_bd_base;

	if (priv->ptimer_present)
		amount_pull = (gfar_uses_fcb(priv) ? GMAC_FCB_LEN : 0);
	else
		amount_pull = (gfar_uses_fcb(priv) ? GMAC_FCB_LEN : 0) +
				priv->padding;

	while (!((bdp->status & RXBD_EMPTY) || (--rx_work_limit < 0))) {
		struct sk_buff *newskb;
		rmb();

		/* Add another skb for the future */
		newskb = gfar_new_skb(dev);

		skb = rx_queue->rx_skbuff[rx_queue->skb_currx];

		dma_unmap_single(&priv->ofdev->dev, bdp->bufPtr,
				priv->rx_buffer_size, DMA_FROM_DEVICE);

		/* We drop the frame if we failed to allocate a new buffer */
		if (unlikely(!newskb || !(bdp->status & RXBD_LAST) ||
				 bdp->status & RXBD_ERR)) {
			count_errors(bdp->status, dev);

			if (unlikely(!newskb))
				newskb = skb;
			else if (skb) {
				/*
				 * We need to un-reserve() the skb to what it
				 * was before gfar_new_skb() re-aligned
				 * it to an RXBUF_ALIGNMENT boundary
				 * before we put the skb back on the
				 * recycle list.
				 */
				skb_reserve(skb, -GFAR_CB(skb)->alignamount);
				skb_queue_head(&priv->rx_recycle, skb);
			}
		} else {
			/* Increment the number of packets */
			rx_queue->stats.rx_packets++;
			howmany++;

			if (likely(skb)) {
				pkt_len = bdp->length - ETH_FCS_LEN;
				/* Remove the FCS from the packet length */
				skb_put(skb, pkt_len);
				rx_queue->stats.rx_bytes += pkt_len;
				skb_record_rx_queue(skb, rx_queue->qindex);
				gfar_process_frame(dev, skb, amount_pull);

			} else {
				if (netif_msg_rx_err(priv))
					printk(KERN_WARNING
					       "%s: Missing skb!\n", dev->name);
				rx_queue->stats.rx_dropped++;
				priv->extra_stats.rx_skbmissing++;
			}

		}

		rx_queue->rx_skbuff[rx_queue->skb_currx] = newskb;

		/* Setup the new bdp */
		gfar_new_rxbdp(rx_queue, bdp, newskb);

		/* Update to the next pointer */
		bdp = next_bd(bdp, base, rx_queue->rx_ring_size);

		/* update to point at the next skb */
		rx_queue->skb_currx =
		    (rx_queue->skb_currx + 1) &
		    RX_RING_MOD_MASK(rx_queue->rx_ring_size);
	}

	/* Update the current rxbd pointer to be the next one */
	rx_queue->cur_rx = bdp;

	return howmany;
}

#ifdef CONFIG_GIANFAR_TXNAPI
static int gfar_poll_tx(struct napi_struct *napi, int budget)
{
	struct gfar_priv_grp *gfargrp = container_of(napi,
					struct gfar_priv_grp, napi_tx);
	struct gfar_private *priv = gfargrp->priv;
	struct gfar __iomem *regs = gfargrp->regs;
	struct gfar_priv_tx_q *tx_queue = NULL;
	int budget_per_queue = 0, tx_cleaned = 0, i = 0, num_act_qs = 0;
	int tx_cleaned_per_queue = 0, mask = TSTAT_TXF0_MASK;
	unsigned long flags;
	u32 imask, tstat, tstat_local;

	tstat = gfar_read(&regs->tstat);
	tstat = tstat & TSTAT_TXF_MASK_ALL;
	tstat_local = tstat;

	while (tstat_local) {
		num_act_qs++;
		tstat_local &= (tstat_local - 1);
	}

	budget_per_queue = budget / num_act_qs;

	gfar_write(&regs->ievent, IEVENT_TX_MASK);

	for_each_set_bit(i, &gfargrp->tx_bit_map, priv->num_tx_queues) {
		mask = mask >> i;
		if (tstat & mask) {
			tx_queue = priv->tx_queue[i];
			spin_lock_irqsave(&tx_queue->txlock, flags);
			tx_cleaned_per_queue =
					gfar_clean_tx_ring(tx_queue,
							budget_per_queue);
			spin_unlock_irqrestore(&tx_queue->txlock,
							flags);
			tx_cleaned += tx_cleaned_per_queue;
			tx_cleaned_per_queue = 0;
		}
		mask = TSTAT_TXF0_MASK;
	}

	budget = (num_act_qs * DEFAULT_TX_RING_SIZE) + 1;
	if (tx_cleaned < budget) {
		napi_complete(napi);
		spin_lock_irq(&gfargrp->grplock);
		gfar_write(&regs->tstat, tstat);
		imask = gfar_read(&regs->imask);
		imask |= IMASK_DEFAULT_TX;
		gfar_write(&regs->ievent, IEVENT_TX_MASK);
		gfar_write(&regs->imask, imask);
		spin_unlock_irq(&gfargrp->grplock);
		gfar_configure_tx_coalescing(priv, gfargrp->tx_bit_map);
		return 1;
	}

	return tx_cleaned;
}

static int gfar_poll_rx(struct napi_struct *napi, int budget)
{
	struct gfar_priv_grp *gfargrp = container_of(napi,
			struct gfar_priv_grp, napi_rx);
	struct gfar_private *priv = gfargrp->priv;
	struct gfar __iomem *regs = gfargrp->regs;
	struct gfar_priv_rx_q *rx_queue = NULL;
	int rx_cleaned = 0, budget_per_queue = 0, rx_cleaned_per_queue = 0;
	int num_act_qs = 0, mask = RSTAT_RXF0_MASK, i;
	u32 imask, rstat, rstat_local, rstat_rhalt = 0;

	rstat = gfar_read(&regs->rstat);
	rstat = rstat & RSTAT_RXF_ALL_MASK;
	rstat_local = rstat;

	while (rstat_local) {
		num_act_qs++;
		rstat_local &= (rstat_local - 1);
	}

	budget_per_queue = budget / num_act_qs;

	gfar_write(&regs->ievent, IEVENT_RX_MASK);

	for_each_set_bit(i, &gfargrp->rx_bit_map, priv->num_rx_queues) {
		mask = mask >> i;
		if (rstat & mask) {
			rstat_rhalt |= (RSTAT_CLEAR_RHALT >> i);
			rx_queue = priv->rx_queue[i];
			rx_cleaned_per_queue = gfar_clean_rx_ring(rx_queue,
							budget_per_queue);
			rx_cleaned += rx_cleaned_per_queue;
		}
		mask = RSTAT_RXF0_MASK;
	}

	if (rx_cleaned < budget) {
		napi_complete(napi);

		/* Clear the halt bit in RSTAT */
		spin_lock_irq(&gfargrp->grplock);
		gfar_write(&regs->rstat, rstat_rhalt);
		gfar_write(&regs->rstat, rstat);
		imask = gfar_read(&regs->imask);
		gfar_write(&regs->ievent, IEVENT_RX_MASK);
		imask |= IMASK_DEFAULT_RX;
		gfar_write(&regs->imask, imask);
		spin_unlock_irq(&gfargrp->grplock);

		gfar_configure_rx_coalescing(priv, gfargrp->rx_bit_map);
	}

	return rx_cleaned;
}
#else
static int gfar_poll(struct napi_struct *napi, int budget)
{
	struct gfar_priv_grp *gfargrp = container_of(napi,
			struct gfar_priv_grp, napi);
	struct gfar_private *priv = gfargrp->priv;
	struct gfar __iomem *regs = gfargrp->regs;
	struct gfar_priv_tx_q *tx_queue = NULL;
	struct gfar_priv_rx_q *rx_queue = NULL;
	int rx_cleaned = 0, budget_per_queue = 0, rx_cleaned_per_queue = 0;
	int tx_cleaned = 0, i, left_over_budget = budget;
	unsigned long serviced_queues = 0;
	int num_queues = 0;

	num_queues = gfargrp->num_rx_queues;
	budget_per_queue = budget/num_queues;

	/* Clear IEVENT, so interrupts aren't called again
	 * because of the packets that have already arrived */
#ifdef CONFIG_GFAR_TX_NONAPI
	gfar_write(&gfargrp->regs->ievent, IEVENT_RX_MASK);
#else
	gfar_write(&regs->ievent, IEVENT_RTX_MASK);
#endif

	while (num_queues && left_over_budget) {

		budget_per_queue = left_over_budget/num_queues;
		left_over_budget = 0;

		for_each_set_bit(i, &gfargrp->rx_bit_map, priv->num_rx_queues) {
			if (test_bit(i, &serviced_queues))
				continue;
			rx_queue = priv->rx_queue[i];

#ifndef CONFIG_GFAR_TX_NONAPI
			tx_queue = priv->tx_queue[rx_queue->qindex];

			tx_cleaned += gfar_clean_tx_ring(tx_queue);
#endif
			rx_cleaned_per_queue = gfar_clean_rx_ring(rx_queue,
							budget_per_queue);
			rx_cleaned += rx_cleaned_per_queue;
			if(rx_cleaned_per_queue < budget_per_queue) {
				left_over_budget = left_over_budget +
					(budget_per_queue - rx_cleaned_per_queue);
				set_bit(i, &serviced_queues);
				num_queues--;
			}
		}
	}

#ifndef CONFIG_GFAR_TX_NONAPI
	if (tx_cleaned)
		return budget;
#endif

	if (rx_cleaned < budget) {
		napi_complete(napi);

		/* Clear the halt bit in RSTAT */
		gfar_write(&regs->rstat, gfargrp->rstat);

		gfar_write(&regs->imask, IMASK_DEFAULT);

		/* If we are coalescing interrupts, update the timer */
		/* Otherwise, clear it */
		gfar_configure_rx_coalescing(priv, gfargrp->rx_bit_map);
#ifndef CONFIG_GFAR_TX_NONAPI
		gfar_configure_tx_coalescing(priv, gfargrp->tx_bit_map);
#endif
	}

	return rx_cleaned;
}
#endif

#ifdef CONFIG_NET_POLL_CONTROLLER
/*
 * Polling 'interrupt' - used by things like netconsole to send skbs
 * without having to re-enable interrupts. It's not called while
 * the interrupt routine is executing.
 */
static void gfar_netpoll(struct net_device *dev)
{
	struct gfar_private *priv = netdev_priv(dev);
	int i = 0;

	/* If the device has multiple interrupts, run tx/rx */
	if (priv->device_flags & FSL_GIANFAR_DEV_HAS_MULTI_INTR) {
		for (i = 0; i < priv->num_grps; i++) {
			disable_irq(priv->gfargrp[i].interruptTransmit);
			disable_irq(priv->gfargrp[i].interruptReceive);
			disable_irq(priv->gfargrp[i].interruptError);
			gfar_interrupt(priv->gfargrp[i].interruptTransmit,
						&priv->gfargrp[i]);
			enable_irq(priv->gfargrp[i].interruptError);
			enable_irq(priv->gfargrp[i].interruptReceive);
			enable_irq(priv->gfargrp[i].interruptTransmit);
		}
	} else {
		for (i = 0; i < priv->num_grps; i++) {
			disable_irq(priv->gfargrp[i].interruptTransmit);
			gfar_interrupt(priv->gfargrp[i].interruptTransmit,
						&priv->gfargrp[i]);
			enable_irq(priv->gfargrp[i].interruptTransmit);
		}
	}
}
#endif

/* The interrupt handler for devices with one interrupt */
static irqreturn_t gfar_interrupt(int irq, void *grp_id)
{
	struct gfar_priv_grp *gfargrp = grp_id;

	/* Save ievent for future reference */
	u32 events = gfar_read(&gfargrp->regs->ievent);

	/* Check for reception */
	if (events & IEVENT_RX_MASK)
		gfar_receive(irq, grp_id);

	/* Check for transmit completion */
	if (events & IEVENT_TX_MASK)
		gfar_transmit(irq, grp_id);

	/* Check for errors */
	if (events & IEVENT_ERR_MASK)
		gfar_error(irq, grp_id);

	return IRQ_HANDLED;
}

/* Called every time the controller might need to be made
 * aware of new link state.  The PHY code conveys this
 * information through variables in the phydev structure, and this
 * function converts those variables into the appropriate
 * register values, and can bring down the device if needed.
 */
static void adjust_link(struct net_device *dev)
{
	struct gfar_private *priv = netdev_priv(dev);
	struct gfar __iomem *regs = priv->gfargrp[0].regs;
	unsigned long flags;
	struct phy_device *phydev = priv->phydev;
	int new_state = 0;

	local_irq_save(flags);
	lock_tx_qs(priv);

	if (phydev->link) {
		u32 tempval = gfar_read(&regs->maccfg2);
		u32 ecntrl = gfar_read(&regs->ecntrl);

		/* Now we make sure that we can be in full duplex mode.
		 * If not, we operate in half-duplex mode. */
		if (phydev->duplex != priv->oldduplex) {
			new_state = 1;
			if (!(phydev->duplex))
				tempval &= ~(MACCFG2_FULL_DUPLEX);
			else
				tempval |= MACCFG2_FULL_DUPLEX;

			priv->oldduplex = phydev->duplex;
		}

		if (phydev->speed != priv->oldspeed) {
			new_state = 1;
			switch (phydev->speed) {
			case 1000:
				tempval =
				    ((tempval & ~(MACCFG2_IF)) | MACCFG2_GMII);

				ecntrl &= ~(ECNTRL_R100);
				break;
			case 100:
			case 10:
				tempval =
				    ((tempval & ~(MACCFG2_IF)) | MACCFG2_MII);

				/* Reduced mode distinguishes
				 * between 10 and 100 */
				if (phydev->speed == SPEED_100)
					ecntrl |= ECNTRL_R100;
				else
					ecntrl &= ~(ECNTRL_R100);
				break;
			default:
				if (netif_msg_link(priv))
					printk(KERN_WARNING
						"%s: Ack!  Speed (%d) is not 10/100/1000!\n",
						dev->name, phydev->speed);
				break;
			}

			priv->oldspeed = phydev->speed;
		}

		gfar_write(&regs->maccfg2, tempval);
		gfar_write(&regs->ecntrl, ecntrl);

		if (!priv->oldlink) {
			new_state = 1;
			priv->oldlink = 1;
		}
	} else if (priv->oldlink) {
		new_state = 1;
		priv->oldlink = 0;
		priv->oldspeed = 0;
		priv->oldduplex = -1;
	}

	if (new_state && netif_msg_link(priv))
		phy_print_status(phydev);
	unlock_tx_qs(priv);
	local_irq_restore(flags);
}

/* Update the hash table based on the current list of multicast
 * addresses we subscribe to.  Also, change the promiscuity of
 * the device based on the flags (this function is called
 * whenever dev->flags is changed */
static void gfar_set_multi(struct net_device *dev)
{
	struct dev_mc_list *mc_ptr;
	struct gfar_private *priv = netdev_priv(dev);
	struct gfar __iomem *regs = priv->gfargrp[0].regs;
	u32 tempval;

	if (dev->flags & IFF_PROMISC) {
		/* Set RCTRL to PROM */
		tempval = gfar_read(&regs->rctrl);
		tempval |= RCTRL_PROM;
		gfar_write(&regs->rctrl, tempval);
	} else {
		/* Set RCTRL to not PROM */
		tempval = gfar_read(&regs->rctrl);
		tempval &= ~(RCTRL_PROM);
		gfar_write(&regs->rctrl, tempval);
	}

	if (dev->flags & IFF_ALLMULTI) {
		/* Set the hash to rx all multicast frames */
		gfar_write(&regs->igaddr0, 0xffffffff);
		gfar_write(&regs->igaddr1, 0xffffffff);
		gfar_write(&regs->igaddr2, 0xffffffff);
		gfar_write(&regs->igaddr3, 0xffffffff);
		gfar_write(&regs->igaddr4, 0xffffffff);
		gfar_write(&regs->igaddr5, 0xffffffff);
		gfar_write(&regs->igaddr6, 0xffffffff);
		gfar_write(&regs->igaddr7, 0xffffffff);
		gfar_write(&regs->gaddr0, 0xffffffff);
		gfar_write(&regs->gaddr1, 0xffffffff);
		gfar_write(&regs->gaddr2, 0xffffffff);
		gfar_write(&regs->gaddr3, 0xffffffff);
		gfar_write(&regs->gaddr4, 0xffffffff);
		gfar_write(&regs->gaddr5, 0xffffffff);
		gfar_write(&regs->gaddr6, 0xffffffff);
		gfar_write(&regs->gaddr7, 0xffffffff);
	} else {
		int em_num;
		int idx;

		/* zero out the hash */
		gfar_write(&regs->igaddr0, 0x0);
		gfar_write(&regs->igaddr1, 0x0);
		gfar_write(&regs->igaddr2, 0x0);
		gfar_write(&regs->igaddr3, 0x0);
		gfar_write(&regs->igaddr4, 0x0);
		gfar_write(&regs->igaddr5, 0x0);
		gfar_write(&regs->igaddr6, 0x0);
		gfar_write(&regs->igaddr7, 0x0);
		gfar_write(&regs->gaddr0, 0x0);
		gfar_write(&regs->gaddr1, 0x0);
		gfar_write(&regs->gaddr2, 0x0);
		gfar_write(&regs->gaddr3, 0x0);
		gfar_write(&regs->gaddr4, 0x0);
		gfar_write(&regs->gaddr5, 0x0);
		gfar_write(&regs->gaddr6, 0x0);
		gfar_write(&regs->gaddr7, 0x0);

		/* If we have extended hash tables, we need to
		 * clear the exact match registers to prepare for
		 * setting them */
		if (priv->extended_hash) {
			em_num = GFAR_EM_NUM + 1;
			gfar_clear_exact_match(dev);
			idx = 1;
		} else {
			idx = 0;
			em_num = 0;
		}

		if (netdev_mc_empty(dev))
			return;

		/* Parse the list, and set the appropriate bits */
		netdev_for_each_mc_addr(mc_ptr, dev) {
			if (idx < em_num) {
				gfar_set_mac_for_addr(dev, idx,
						mc_ptr->dmi_addr);
				idx++;
			} else
				gfar_set_hash_for_addr(dev, mc_ptr->dmi_addr);
		}
	}

	return;
}


/* Clears each of the exact match registers to zero, so they
 * don't interfere with normal reception */
static void gfar_clear_exact_match(struct net_device *dev)
{
	int idx;
	u8 zero_arr[MAC_ADDR_LEN] = {0,0,0,0,0,0};

	for(idx = 1;idx < GFAR_EM_NUM + 1;idx++)
		gfar_set_mac_for_addr(dev, idx, (u8 *)zero_arr);
}

/* Set the appropriate hash bit for the given addr */
/* The algorithm works like so:
 * 1) Take the Destination Address (ie the multicast address), and
 * do a CRC on it (little endian), and reverse the bits of the
 * result.
 * 2) Use the 8 most significant bits as a hash into a 256-entry
 * table.  The table is controlled through 8 32-bit registers:
 * gaddr0-7.  gaddr0's MSB is entry 0, and gaddr7's LSB is
 * gaddr7.  This means that the 3 most significant bits in the
 * hash index which gaddr register to use, and the 5 other bits
 * indicate which bit (assuming an IBM numbering scheme, which
 * for PowerPC (tm) is usually the case) in the register holds
 * the entry. */
static void gfar_set_hash_for_addr(struct net_device *dev, u8 *addr)
{
	u32 tempval;
	struct gfar_private *priv = netdev_priv(dev);
	u32 result = ether_crc(MAC_ADDR_LEN, addr);
	int width = priv->hash_width;
	u8 whichbit = (result >> (32 - width)) & 0x1f;
	u8 whichreg = result >> (32 - width + 5);
	u32 value = (1 << (31-whichbit));

	tempval = gfar_read(priv->hash_regs[whichreg]);
	tempval |= value;
	gfar_write(priv->hash_regs[whichreg], tempval);

	return;
}


/* There are multiple MAC Address register pairs on some controllers
 * This function sets the numth pair to a given address
 */
static void gfar_set_mac_for_addr(struct net_device *dev, int num, u8 *addr)
{
	struct gfar_private *priv = netdev_priv(dev);
	struct gfar __iomem *regs = priv->gfargrp[0].regs;
	int idx;
	char tmpbuf[MAC_ADDR_LEN];
	u32 tempval;
	u32 __iomem *macptr = &regs->macstnaddr1;

	macptr += num*2;

	/* Now copy it into the mac registers backwards, cuz */
	/* little endian is silly */
	for (idx = 0; idx < MAC_ADDR_LEN; idx++)
		tmpbuf[MAC_ADDR_LEN - 1 - idx] = addr[idx];

	gfar_write(macptr, *((u32 *) (tmpbuf)));

	tempval = *((u32 *) (tmpbuf + 4));

	gfar_write(macptr+1, tempval);
}

/* GFAR error interrupt handler */
static irqreturn_t gfar_error(int irq, void *grp_id)
{
	struct gfar_priv_grp *gfargrp = grp_id;
	struct gfar __iomem *regs = gfargrp->regs;
	struct gfar_private *priv= gfargrp->priv;
	struct net_device *dev = priv->ndev;

	/* Save ievent for future reference */
	u32 events = gfar_read(&regs->ievent);

	/* Clear IEVENT */
	gfar_write(&regs->ievent, events & IEVENT_ERR_MASK);

	/* Magic Packet is not an error. */
	if ((priv->device_flags & FSL_GIANFAR_DEV_HAS_MAGIC_PACKET) &&
	    (events & IEVENT_MAG))
		events &= ~IEVENT_MAG;

	/* Hmm... */
	if (netif_msg_rx_err(priv) || netif_msg_tx_err(priv))
		printk(KERN_DEBUG "%s: error interrupt (ievent=0x%08x imask=0x%08x)\n",
		       dev->name, events, gfar_read(&regs->imask));

	/* Update the error counters */
	if (events & IEVENT_TXE) {
		dev->stats.tx_errors++;

		if (events & IEVENT_LC)
			dev->stats.tx_window_errors++;
		if (events & IEVENT_CRL)
			dev->stats.tx_aborted_errors++;
		if (events & IEVENT_XFUN) {
			unsigned long flags;

			if (netif_msg_tx_err(priv))
				printk(KERN_DEBUG "%s: TX FIFO underrun, "
				       "packet dropped.\n", dev->name);
			dev->stats.tx_dropped++;
			priv->extra_stats.tx_underrun++;

			local_irq_save(flags);
			lock_tx_qs(priv);

			/* Reactivate the Tx Queues */
			gfar_write(&regs->tstat, gfargrp->tstat);

			unlock_tx_qs(priv);
			local_irq_restore(flags);
		}
		if (netif_msg_tx_err(priv))
			printk(KERN_DEBUG "%s: Transmit Error\n", dev->name);
	}
	if (events & IEVENT_BSY) {
		dev->stats.rx_errors++;
		priv->extra_stats.rx_bsy++;

		gfar_receive(irq, grp_id);

		if (netif_msg_rx_err(priv))
			printk(KERN_DEBUG "%s: busy error (rstat: %x)\n",
			       dev->name, gfar_read(&regs->rstat));
	}
	if (events & IEVENT_BABR) {
		dev->stats.rx_errors++;
		priv->extra_stats.rx_babr++;

		if (netif_msg_rx_err(priv))
			printk(KERN_DEBUG "%s: babbling RX error\n", dev->name);
	}
	if (events & IEVENT_EBERR) {
		priv->extra_stats.eberr++;
		if (netif_msg_rx_err(priv))
			printk(KERN_DEBUG "%s: bus error\n", dev->name);
	}
	if ((events & IEVENT_RXC) && netif_msg_rx_status(priv))
		printk(KERN_DEBUG "%s: control frame\n", dev->name);

	if (events & IEVENT_BABT) {
		priv->extra_stats.tx_babt++;
		if (netif_msg_tx_err(priv))
			printk(KERN_DEBUG "%s: babbling TX error\n", dev->name);
	}
	return IRQ_HANDLED;
}

static struct of_device_id gfar_match[] =
{
	{
		.type = "network",
		.compatible = "gianfar",
	},
	{
		.compatible = "fsl,etsec2",
	},
	{},
};
MODULE_DEVICE_TABLE(of, gfar_match);

/* Structure for a device driver */
static struct of_platform_driver gfar_driver = {
	.name = "fsl-gianfar",
	.match_table = gfar_match,

	.probe = gfar_probe,
	.remove = gfar_remove,
	.suspend = gfar_legacy_suspend,
	.resume = gfar_legacy_resume,
	.driver.pm = GFAR_PM_OPS,
};

static int __init gfar_init(void)
{
	gfar_1588_node_init(gfar_match, sizeof(gfar_match));
	return of_register_platform_driver(&gfar_driver);
}

static void __exit gfar_exit(void)
{
	if (ptp_1588_present)
		gfar_1588_proc_exit();

	of_unregister_platform_driver(&gfar_driver);
}

module_init(gfar_init);
module_exit(gfar_exit);

