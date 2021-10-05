// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 INTEL Corporation

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/in.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/skbuff.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_net.h>
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/axxia-ncr.h>
#include <linux/of_mdio.h>
#include <linux/crc32.h>
#include <linux/phy_fixed.h>
#include <net/ip.h>
#include <asm/dma.h>
#define CREATE_TRACE_POINTS
#include <trace/events/femac.h>

#include "axxia_acp_net.h"

static void axxianet_reset(struct net_device *);

/*******************************************************************************
 *  1) This driver is used by both ACP (PPC) and AXM (ARM) platforms.
 *
 *  2) This driver parses the DTB for driver specific settings. A few of
 *  them can be overridden by setting environment variables in
 *  U-boot:
 *
 *  ethaddr - MAC address of interface, in xx:xx:xx:xx:xx:xx format
 *
 *  phy-addr - Specific address of PHY (0 - 0x20). If not specified,
 *  the driver will scan the bus and will attach to the
 *  first PHY it finds.
 *
 *  ad-value - PHY advertise value. Can be set to one of these or
 *  they be OR'ed together. If not set, the driver sets
 *  the advertised value equal to what the driver
 *  supports.
 *
 *  0x101 - 100/Full 0x81 - 100/Half 0x41 - 10/Full 0x21 -
 *  10/Half
 *
 *  3) This driver allows the option to disable auto negotiation and
 *  manually specify the speed and duplex setting, with the use of
 *  the device tree variable "phy-link". Legal values for this
 *  variable are:
 *
 *  "auto" - auto negotiation enabled "100MF" - auto negotiation
 *  disabled, set to 100MB Full Duplex "10MH" - auto negotiation
 *  disabled, set to 100MB Half Duplex "10MF" - auto negotiation
 *  disabled, set to 10MB Full Duplex "10MH" - auto negotiation
 *  disabled, set to 10MB Half Duplex
 *
 *  NOTE: If the phy-link variable is not present in the device tree,
 *  or if an invalid value is used, the driver defaults to auto
 *  negotiation mode.
 ******************************************************************************/

#define AXXIA_DRV_NAME           "acp-femac"
#define AXXIA_MDIO_NAME          "acp-femac-mdio"
#define AXXIA_DRV_VERSION        "2020-07-16"

MODULE_AUTHOR("John Jacques");
MODULE_DESCRIPTION("AXXIA ACP-FEMAC Ethernet driver");
MODULE_LICENSE("GPL");

/* ----------------------------------------------------------------------
 * appnic_handle_link_change
 *
 * Called periodically when PHY is in polling mode.
 */

static void appnic_handle_link_change(struct net_device *ndev)
{
	struct appnic_device *pdata = netdev_priv(ndev);
	struct phy_device *phydev = pdata->phy_dev;
	int status_change = 0;
	unsigned long rx_configuration;
	unsigned long tx_configuration = 0;

	rx_configuration =
		APPNIC_RX_CONF_STRIPCRC;
	tx_configuration =
		(APPNIC_TX_CONF_ENABLE_SWAP_SA |
		 APPNIC_TX_CONF_APP_CRC_ENABLE |
		 APPNIC_TX_CONF_PAD_ENABLE);

	TX_CONF_SET_IFG(tx_configuration, 0xf);

	if (phydev->link) {
		if (pdata->speed != phydev->speed ||
		    pdata->duplex != phydev->duplex) {
			if (phydev->duplex) {
				rx_configuration |= APPNIC_RX_CONF_DUPLEX;
				tx_configuration |= APPNIC_TX_CONF_DUPLEX;
			}
			if (phydev->speed == SPEED_100) {
				rx_configuration |= APPNIC_RX_CONF_SPEED;
				tx_configuration |= APPNIC_TX_CONF_SPEED;
			}

			rx_configuration |= (APPNIC_RX_CONF_ENABLE |
					     APPNIC_RX_CONF_LINK);
			tx_configuration |= (APPNIC_TX_CONF_LINK |
					     APPNIC_TX_CONF_ENABLE);

			pdata->speed = phydev->speed;
			pdata->duplex = phydev->duplex;
			status_change = 1;
		}
	}
	if (phydev->link != pdata->link) {
		if (!phydev->link) {
			pdata->speed = 0;
			pdata->duplex = -1;
		}
		pdata->link = phydev->link;
		status_change = 1;
	}

	if (status_change) {
		if (phydev->link) {
			netif_carrier_on(ndev);
			netdev_info(ndev, "link up (%d/%s)\n",
				    phydev->speed,
				    phydev->duplex == DUPLEX_FULL ?
				    "Full" : "Half");
		} else {
			netif_carrier_off(ndev);
			netdev_info(ndev, "link down\n");
		}

		if (rx_configuration != read_mac(APPNIC_RX_CONF))
			write_mac(rx_configuration, APPNIC_RX_CONF);

		if (tx_configuration != read_mac(APPNIC_TX_CONF))
			write_mac(tx_configuration, APPNIC_TX_CONF);
	}
}

/* ======================================================================
 * NIC Interface
 * ======================================================================
 */

#define DESCRIPTOR_GRANULARITY 64
#define BUFFER_ALIGNMENT 64

#define ALIGN64B(address) (PTR_ALIGN((address), BUFFER_ALIGNMENT))
#define ALIGN64B_OFFSET(address) \
	(ALIGN64B(address) - (unsigned long)(address))

/*  ----- Note On Buffer Space -----
 *
 *  Minimum number of descriptors is 64 for the receiver and 64 for the
 *  transmitter; therefore, 2048 bytes (16 bytes each).
 *  This driver uses the following parameters, all of which may be set on
 *  the command line if this drivers is used as a module.
 *
 *  - rx_num_desc : Number of receive descriptors. This  must be a multiple
 *                  of 64.
 *  - tx_num_desc : Number of transmit descriptors. This must be a multiple
 *                  of 64.
 *
 *  The scheme used will be as follows:
 *
 *  - num_[rt]x_desc will be adjusted to be a multiple of 64 (if necessary).
 *  - An skb (with the data area 64 byte aligned) will be allocated for each rx
 *    descriptor.
 */

/* Receiver */

int rx_num_desc = (CONFIG_AXXIA_NET_NUM_RX_DESC * DESCRIPTOR_GRANULARITY);
module_param(rx_num_desc, int, 0000);
MODULE_PARM_DESC(rx_num_desc, "appnic : Number of receive descriptors");

int rx_buf_sz = CONFIG_AXXIA_NET_RX_BUF_SZ;
module_param(rx_buf_sz, int, 0000);
MODULE_PARM_DESC(rx_buf_sz, "appnic : Receive buffer size");

/* Transmitter */

int tx_num_desc = (CONFIG_AXXIA_NET_NUM_TX_DESC * DESCRIPTOR_GRANULARITY);
module_param(tx_num_desc, int, 0000);
MODULE_PARM_DESC(tx_num_desc, "appnic : Number of receive descriptors");

int tx_buf_sz = CONFIG_AXXIA_NET_TX_BUF_SZ;
module_param(tx_buf_sz, int, 0000);
MODULE_PARM_DESC(tx_buf_sz, "Appnic : Receive buffer size");

/* ======================================================================
 * Utility Functions
 * ======================================================================
 */

/* ----------------------------------------------------------------------
 * mac_addr_valid
 *
 * If mac address is multicast, broadcast, or matches our mac address,
 * it's a valid address. Otherwise, it's not.
 */

static bool mac_addr_valid(struct net_device *dev, u8 *mac_addr)
{
	bool is_valid = false;

	if (is_multicast_ether_addr(mac_addr))
		is_valid = true;
	else if (is_broadcast_ether_addr(mac_addr))
		is_valid = true;
	else if (ether_addr_equal(mac_addr, &dev->dev_addr[0]))
		is_valid = true;

	return is_valid;
}

/* ----------------------------------------------------------------------
 * clear_statistics
 *
 * NOTE: The hardware clears the statistics registers after a read.
 */

static void clear_statistics(struct appnic_device *pdata)
{
	/* Clear memory. */

	memset((void *)&pdata->stats, 0, sizeof(struct net_device_stats));

	/* Clear counters by reading them. */

	/* stats.rx_packets */
	read_mac(APPNIC_RX_STAT_PACKET_OK);

	/* stats.tx_packets */
	read_mac(APPNIC_TX_STAT_PACKET_OK);

	/* stats.rx_bytes - Updated by this driver.
	 * stats.tx_bytes - Updated by this driver.
	 * stats.rx_errors - The sum of all RX errors available.
	 * stats.tx_errors - The sum of all TX errors available.
	 * stats.rx_dropped (unable to allocate skb) - Updated by the stack.
	 * stats.tx_dropped (unable to allocate skb) - Updated by the stack.
	 */

	/* stats.multicast */
	read_mac(APPNIC_RX_STAT_MULTICAST);

	/* stats.collisions - The sum of the following driver stats. */
	read_mac(APPNIC_TX_STATUS_LATE_COLLISION);
	read_mac(APPNIC_TX_STATUS_EXCESSIVE_COLLISION);
	read_mac(APPNIC_TX_STAT_COLLISION_ABOVE_WATERMARK);

	/* stats.rx_length_errors - The sum of the following driver stats. */
	read_mac(APPNIC_RX_STAT_UNDERSIZE);
	read_mac(APPNIC_RX_STAT_OVERSIZE);

	/* stats.rx_over_errors - Not maintained by this driver. */

	/* stats.rx_crc_errors */
	read_mac(APPNIC_RX_STAT_CRC_ERROR);

	/* stats.rx_frame_errors */
	read_mac(APPNIC_RX_STAT_ALIGN_ERROR);

	/* stats.rx_fifo_errors */
	read_mac(APPNIC_RX_STAT_OVERFLOW);

	/* stats.rx_missed - Not maintained by this driver.
	 * stats.tx_aborted_errors - Not maintained by this driver.
	 * stats.tx_carrier_errors - Not maintained by this driver.
	 */

	/* stats.tx_fifo_errors */
	read_mac(APPNIC_TX_STAT_UNDERRUN);

	/* stats.tx_heartbeat_errors - Not maintained by this driver.
	 * stats.tx_window_errors - Not mainteaned by this driver.
	 * stats.rx_compressed - Not maintained by this driver.
	 * stats.tx_compressed - Not maintained by this driver.
	 */

	pdata->not_for_us = 0;
}

/* ----------------------------------------------------------------------
 * get_hw_statistics
 *
 * NOTE: The hardware clears the statistics registers after a read.
 */

static void get_hw_statistics(struct appnic_device *pdata)
{
	unsigned long flags;
	u32 rx_under;
	u32 rx_over;
	u32 tx_under;

	/* stats.tx_packets */
	pdata->stats.tx_packets += read_mac(APPNIC_TX_STAT_PACKET_OK);

	/* stats.multicast */
	pdata->stats.multicast += read_mac(APPNIC_RX_STAT_MULTICAST);

	/* stats.collision */
	pdata->stats.collisions += read_mac(APPNIC_TX_STATUS_LATE_COLLISION);
	pdata->stats.collisions +=
		read_mac(APPNIC_TX_STATUS_EXCESSIVE_COLLISION);
	pdata->stats.collisions +=
		read_mac(APPNIC_TX_STAT_COLLISION_ABOVE_WATERMARK);

	/* stats.rx_length_errors */
	rx_under = read_mac(APPNIC_RX_STAT_UNDERSIZE);
	pdata->stats.rx_length_errors += rx_under;
	rx_over = read_mac(APPNIC_RX_STAT_OVERSIZE);
	pdata->stats.rx_length_errors += rx_over;

	/* stats.tx_fifo_errors */
	tx_under = read_mac(APPNIC_TX_STAT_UNDERRUN);
	pdata->stats.tx_fifo_errors += tx_under;

	/* Lock this section out so the statistics maintained by the driver
	 * don't get clobbered.
	 */

	spin_lock_irqsave(&pdata->dev_lock, flags);

	/* Update the cumulative rx_errors. */
	pdata->stats.rx_errors += (rx_under + rx_over);

	/* Update the cumulative tx_errors. */
	pdata->stats.tx_errors += tx_under;

	spin_unlock_irqrestore(&pdata->dev_lock, flags);
}

/* ----------------------------------------------------------------------
 * queue_initialized
 *
 * Returns the number of descriptors that are ready to receive packets
 * or are waiting to transmit packets.  (from tail to head).
 */

static int queue_initialized(union appnic_queue_pointer head,
			     union appnic_queue_pointer tail,
			     int size)
{
	int initialized;

	/* Calculate the number of descriptors currently initialized. */
	if (head.bits.generation_bit == tail.bits.generation_bit) {
		/* same generation */
		initialized = (head.bits.offset - tail.bits.offset);
	} else {
		/* different generation */
		initialized = head.bits.offset +
			(size * sizeof(struct appnic_dma_descriptor) -
			 tail.bits.offset);
	}

	/* Number of descriptors is offset / sizeof(a descriptor). */
	initialized /= sizeof(struct appnic_dma_descriptor);

	return initialized;
}

/* ----------------------------------------------------------------------
 * queue_uninitialzed
 *
 * Returns the number of unused/uninitialized descriptors. (from head to tail).
 */

static int queue_uninitialized(union appnic_queue_pointer head,
			       union appnic_queue_pointer tail,
			       int size)
{
	int allocated;

	/* Calculate the number of descriptors currently unused/uninitialized */
	if (head.bits.generation_bit == tail.bits.generation_bit)
		/* Same generation. */
		allocated = ((size * sizeof(struct appnic_dma_descriptor)) -
			 head.bits.offset) + tail.bits.offset;
	else
		/* Different generation. */
		allocated = tail.bits.offset - head.bits.offset;

	/* Number of descriptors is offset / sizeof(a descriptor). */
	allocated /= sizeof(struct appnic_dma_descriptor);

	/* That's all. */
	return allocated;
}

/* ----------------------------------------------------------------------
 * queue_increment
 */

static void queue_increment(union appnic_queue_pointer *queue,
			    int number_of_descriptors)
{
	queue->bits.offset += sizeof(struct appnic_dma_descriptor);

	if ((number_of_descriptors * sizeof(struct appnic_dma_descriptor)) ==
		queue->bits.offset) {
		queue->bits.offset = 0;
		queue->bits.generation_bit =
			(queue->bits.generation_bit == 0) ? 1 : 0;
	}
}

/* ----------------------------------------------------------------------
 * queue_decrement
 */

static void queue_decrement(union appnic_queue_pointer *queue,
			    int number_of_descriptors)
{
	if (queue->bits.offset == 0) {
		queue->bits.offset =
			((number_of_descriptors - 1) *
			 sizeof(struct appnic_dma_descriptor));
		queue->bits.generation_bit =
			(queue->bits.generation_bit == 0) ? 1 : 0;
	} else {
		queue->bits.offset -= sizeof(struct appnic_dma_descriptor);
	}
}

/* ----------------------------------------------------------------------
 * disable_rx_tx
 */

static void disable_rx_tx(struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);
	unsigned long tx_configuration;
	unsigned long rx_configuration;

	rx_configuration = read_mac(APPNIC_RX_CONF);
	rx_configuration &= ~APPNIC_RX_CONF_ENABLE;
	write_mac(rx_configuration, APPNIC_RX_CONF);

	tx_configuration = read_mac(APPNIC_TX_CONF);
	tx_configuration &= ~APPNIC_TX_CONF_ENABLE;

	write_mac(tx_configuration, APPNIC_TX_CONF);
}

/* ======================================================================
 *  Linux Network Driver Interface
 *  ======================================================================
 */

/* ----------------------------------------------------------------------
 * handle_transmit_interrupt
 */

static void handle_transmit_interrupt(struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);
	union appnic_queue_pointer queue;

	/* The hardware's tail pointer should be one descriptor (or more)
	 * ahead of software's copy.
	 */

	queue = swab_queue_pointer(pdata->tx_tail);
	while (queue_initialized(queue, pdata->tx_tail_copy,
				 pdata->tx_num_desc) > 0) {
		queue_increment(&pdata->tx_tail_copy, pdata->tx_num_desc);
		queue = swab_queue_pointer(pdata->tx_tail);
	}
}

/* ----------------------------------------------------------------------
 * axxianet_rx_packet
 */

static int axxianet_rx_packet(struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);
	struct appnic_dma_descriptor descriptor;
	struct sk_buff *sk_buff;
	unsigned int bytes_copied = 0;
	unsigned int error_num = 0;
	unsigned long ok_stat = 0, overflow_stat = 0;
	unsigned long crc_stat = 0, align_stat = 0;
	union appnic_queue_pointer queue;
	int queue_loops = 0;
	int detected_overrun = 0;
	struct device *d = &dev->dev;

	readdescriptor(((unsigned long)pdata->rx_desc +
			pdata->rx_tail_copy.bits.offset), &descriptor);

	sk_buff = netdev_alloc_skb(dev, AXXIANET_MAX_MTU);

	if (sk_buff == (struct sk_buff *)0) {
		dev_info_ratelimited(d,
				     "packet dropped (no buffer available)\n");
		pdata->stats.rx_dropped++;
		return 0;
	}

	trace_rx_packet_start(skb_tailroom(sk_buff), descriptor.pdu_length);

	/* Needs to be reviewed. This fixed an alignment
	 * exception when pinging to the target from a host.
	 */

	/* Align IP on 16 byte boundaries */
	skb_reserve(sk_buff, 2);

	ok_stat = read_mac(APPNIC_RX_STAT_PACKET_OK);
	overflow_stat = read_mac(APPNIC_RX_STAT_OVERFLOW);
	crc_stat = read_mac(APPNIC_RX_STAT_CRC_ERROR);
	align_stat = read_mac(APPNIC_RX_STAT_ALIGN_ERROR);

	/* Copy the received packet into the skb. */

	queue = swab_queue_pointer(pdata->rx_tail);
	while (queue_initialized(queue, pdata->rx_tail_copy,
				 pdata->rx_num_desc) > 0) {
		queue_loops++;

		trace_rx_packet_loop(skb_tailroom(sk_buff),
				     descriptor.pdu_length, queue_loops);

		if (skb_tailroom(sk_buff) >= descriptor.pdu_length) {
			unsigned char *buffer;

			buffer = skb_put(sk_buff, descriptor.pdu_length);
			memcpy((void *)buffer,
			       (void *)(descriptor.host_data_memory_pointer +
				 pdata->dma_alloc_offset_rx),
			       descriptor.pdu_length);
		} else {
			dev_err_ratelimited(d, "%s: PDU overrun (%u/%u, %d)\n",
					    AXXIA_DRV_NAME,
					    descriptor.pdu_length,
					    bytes_copied,
					    descriptor.error);

			trace_rx_packet_pdu_overrun(bytes_copied,
						    (unsigned int *)&descriptor);
			detected_overrun = 1;
			ftrace_dump(DUMP_ALL);
			return -1; /* Indicate that NIC should be reset! */
		}
		bytes_copied += descriptor.pdu_length;
		descriptor.data_transfer_length = pdata->rx_buf_per_desc;
		writedescriptor(((unsigned long)pdata->rx_desc +
					pdata->rx_tail_copy.bits.offset),
				&descriptor);
		if (descriptor.error != 0) {
			error_num = 1;
			trace_rx_packet_error(1);
		}

		queue_increment(&pdata->rx_tail_copy, pdata->rx_num_desc);
		if (descriptor.end_of_packet != 0) {
			if (detected_overrun != 0)
				trace_rx_packet_overrun_eop(bytes_copied);

			break;
		}
		readdescriptor(((unsigned long)pdata->rx_desc +
					pdata->rx_tail_copy.bits.offset),
			       &descriptor);
		queue = swab_queue_pointer(pdata->rx_tail);
	}

	trace_rx_packet_finish(queue_loops);

	if (descriptor.end_of_packet == 0) {
		dev_err(d, "%s: No end of packet! %lu/%lu/%lu/%lu\n",
			AXXIA_DRV_NAME, ok_stat, overflow_stat,
			crc_stat, align_stat);
		dev_kfree_skb(sk_buff);
		return -1;    /* Indicate that NIC should be reset! */
	} else if (error_num == 0) {
		struct ethhdr *ethhdr = (struct ethhdr *)sk_buff->data;

		if (pdata->promiscuous_mode != 0 ||
		    mac_addr_valid(dev, &ethhdr->h_dest[0])) {
			pdata->stats.rx_bytes += bytes_copied;
			++pdata->stats.rx_packets;
			sk_buff->dev = dev;
			sk_buff->protocol = eth_type_trans(sk_buff, dev);

			if (netif_receive_skb(sk_buff) == NET_RX_DROP)
				++pdata->dropped_by_stack;
		} else {
			dev_warn_once(d, "packet not for us dropped\n");
			++pdata->stats.rx_dropped;
			++pdata->not_for_us;
			trace_rx_packet_not_for_us(pdata->not_for_us);
			dev_kfree_skb(sk_buff);
		}
	} else {
		dev_kfree_skb(sk_buff);

		pdata->stats.rx_errors +=
			(overflow_stat + crc_stat + align_stat);

		if (descriptor.end_of_packet == 0 && ok_stat != 0) {
			dev_warn_ratelimited(d,
					     "%lu packet(s) dropped\n", ok_stat);
			pdata->stats.rx_dropped += ok_stat;
		} else if (overflow_stat != 0) {
			dev_err(d, "%lu overflow(s)\n", overflow_stat);
			pdata->stats.rx_fifo_errors += overflow_stat;
		} else if (crc_stat != 0) {
			dev_err(d, "%lu crc error(s)\n", crc_stat);
			pdata->stats.rx_crc_errors += crc_stat;
		} else if (align_stat != 0) {
			dev_err(d, "%lu crc error(s)\n", crc_stat);
			pdata->stats.rx_frame_errors += align_stat;
		}
	}

	return 0;
}

/* ----------------------------------------------------------------------
 * axxianet_rx_packets
 */

static int axxianet_rx_packets(struct net_device *dev, int max)
{
	struct appnic_device *pdata = netdev_priv(dev);
	union appnic_queue_pointer orig_queue, new_queue;
	int updated_head_pointer = 0;
	int packets = 0;
	int rc;

	new_queue.raw = pdata->rx_tail_copy.raw;

	/* Receive Packets */
	trace_rx_packets_start(max);

	orig_queue = swab_queue_pointer(pdata->rx_tail);
	while (queue_initialized(orig_queue, new_queue,
				 pdata->rx_num_desc) > 0) {
		struct appnic_dma_descriptor descriptor;

		readdescriptor(((unsigned long)pdata->rx_desc +
				  new_queue.bits.offset),
				&descriptor);

		if (descriptor.end_of_packet != 0) {
			rc = axxianet_rx_packet(dev);

			if (unlikely(rc == -1)) {
				axxianet_reset(dev);

				return 0;
			}

			packets++;
			new_queue.raw = pdata->rx_tail_copy.raw;

			if (-1 != max && packets == max)
				break;
		} else {
			queue_increment(&new_queue, pdata->rx_num_desc);
		}
		orig_queue = swab_queue_pointer(pdata->rx_tail);
	}

	trace_rx_packets_finish(packets);

	/* Update the Head Pointer */

	while (queue_uninitialized(pdata->rx_head,
				   pdata->rx_tail_copy,
				   pdata->rx_num_desc) > 1) {
		struct appnic_dma_descriptor descriptor;

		readdescriptor(((unsigned long)pdata->rx_desc +
				  pdata->rx_head.bits.offset), &descriptor);
		descriptor.data_transfer_length = pdata->rx_buf_per_desc;
		descriptor.write = 1;
		descriptor.pdu_length = 0;
		descriptor.start_of_packet = 0;
		descriptor.end_of_packet = 0;
		descriptor.interrupt_on_completion = 1;
		writedescriptor(((unsigned long)pdata->rx_desc +
				   pdata->rx_head.bits.offset),
				 &descriptor);
		queue_increment(&pdata->rx_head, pdata->rx_num_desc);
		updated_head_pointer = 1;
	}

	if (updated_head_pointer != 0)
		write_mac(pdata->rx_head.raw, APPNIC_DMA_RX_HEAD_POINTER);

	return packets;
}

/* ----------------------------------------------------------------------
 * axxianet_poll
 */

static int axxianet_poll(struct napi_struct *napi, int budget)
{
	struct appnic_device *pdata =
		container_of(napi, struct appnic_device, napi);
	struct net_device *dev = pdata->netdev;

	int work_done = 0;
	unsigned long dma_interrupt_status;

	do {
		/* Acknowledge the RX interrupt. */
		write_mac(~APPNIC_DMA_INTERRUPT_ENABLE_RECEIVE,
			  APPNIC_DMA_INTERRUPT_STATUS);

		/* Get Rx packets. */
		work_done += axxianet_rx_packets(dev, budget - work_done);

		/* We've hit the budget limit. */
		if (work_done == budget)
			break;

		dma_interrupt_status = read_mac(APPNIC_DMA_INTERRUPT_STATUS);

	} while (RX_INTERRUPT(dma_interrupt_status));

	if (work_done < budget) {
		napi_complete(napi);

		/* Re-enable receive interrupts (and preserve
		 * the already enabled TX interrupt).
		 */
		write_mac((APPNIC_DMA_INTERRUPT_ENABLE_RECEIVE |
			   APPNIC_DMA_INTERRUPT_ENABLE_TRANSMIT),
			  APPNIC_DMA_INTERRUPT_ENABLE);
	}

	return work_done;
}

/* ----------------------------------------------------------------------
 * appnic_isr
 */

static irqreturn_t appnic_isr(int irq, void *device_id)
{
	struct net_device *dev = (struct net_device *)device_id;
	struct appnic_device *pdata = netdev_priv(dev);
	unsigned long dma_interrupt_status;
	unsigned long dev_flags;
	unsigned long tx_flags;

	/* Acquire the lock. */
	spin_lock_irqsave(&pdata->dev_lock, dev_flags);

	/* Get the status. */
	dma_interrupt_status = read_mac(APPNIC_DMA_INTERRUPT_STATUS);

	/* NAPI - don't ack RX interrupt */
	write_mac(APPNIC_DMA_INTERRUPT_ENABLE_RECEIVE,
		  APPNIC_DMA_INTERRUPT_STATUS);

	/* Handle interrupts. */
	if (TX_INTERRUPT(dma_interrupt_status)) {
		/* transmition complete */
		pdata->transmit_interrupts++;
		spin_lock_irqsave(&pdata->tx_lock, tx_flags);
		handle_transmit_interrupt(dev);
		spin_unlock_irqrestore(&pdata->tx_lock, tx_flags);
	}

	if (RX_INTERRUPT(dma_interrupt_status)) {
		pdata->receive_interrupts++;
		if (napi_schedule_prep(&pdata->napi)) {
			/* Disable RX interrupts and tell the
			 * system we've got work
			 */
			write_mac(APPNIC_DMA_INTERRUPT_ENABLE_TRANSMIT,
				  APPNIC_DMA_INTERRUPT_ENABLE);
			__napi_schedule(&pdata->napi);
		} else {
			write_mac(APPNIC_DMA_INTERRUPT_ENABLE_TRANSMIT,
				  APPNIC_DMA_INTERRUPT_ENABLE);
		}
	}

	/* Release the lock */
	spin_unlock_irqrestore(&pdata->dev_lock, dev_flags);

	return IRQ_HANDLED;
}

#ifdef CONFIG_NET_POLL_CONTROLLER

/* ----------------------------------------------------------------------
 * appnic_poll_controller
 *
 * Polling receive - used by netconsole and other diagnostic tools
 * to allow network i/o with interrupts disabled.
 */

static void appnic_poll_controller(struct net_device *dev)
{
	disable_irq(dev->irq);
	appnic_isr(dev->irq, dev);
	enable_irq(dev->irq);
}

#endif

/* ----------------------------------------------------------------------
 * appnic_open
 *
 * Opens the interface.  The interface is opened whenever ifconfig
 * activates it.  The open method should register any system resource
 * it needs (I/O ports, IRQ, DMA, etc.) turn on the hardware, and
 * increment the module usage count.
 */

static int appnic_open(struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);
	int return_code = 0;
	struct device *d = &dev->dev;

	/* Bring the PHY up. */
	phy_start(pdata->phy_dev);

	/* Enable NAPI. */
	napi_enable(&pdata->napi);

	/* Install the interrupt handlers. */
	return_code =
		request_irq(dev->irq, appnic_isr, 0x00, AXXIA_DRV_NAME, dev);
	if (return_code != 0) {
		dev_err(d, "request_irq() failed, returned 0x%x/%d\n",
			return_code, return_code);
		return return_code;
	}

	/* Enable interrupts. */
	write_mac((APPNIC_DMA_INTERRUPT_ENABLE_RECEIVE |
		   APPNIC_DMA_INTERRUPT_ENABLE_TRANSMIT),
		   APPNIC_DMA_INTERRUPT_ENABLE);

	/* Let the OS know we are ready to send packets. */
	netif_start_queue(dev);

	/* That's all. */
	return 0;
}

/* ----------------------------------------------------------------------
 * appnic_stop
 *
 * Stops the interface.  The interface is stopped when it is brought
 * down; operations performed at open time should be reversed.
 */

static int appnic_stop(struct net_device *ndev)
{
	struct appnic_device *pdata = netdev_priv(ndev);
	struct device *dev = &ndev->dev;

	dev_info(dev, "Stopping the interface.\n");

	/* Disable interrupts. Note that disable_irq() will wait for
	 * any interrupt handlers that are currently executing to
	 * complete.
	 */
	write_mac(0, APPNIC_DMA_INTERRUPT_ENABLE);
	disable_irq(ndev->irq);
	free_irq(ndev->irq, ndev);

	/* Indicate to the OS that no more packets should be sent.  */
	netif_stop_queue(ndev);
	napi_disable(&pdata->napi);

	/* Stop the receiver and transmitter. */
	disable_rx_tx(ndev);

	/* Bring the PHY down. */
	if (pdata->phy_dev)
		phy_stop(pdata->phy_dev);

	/* That's all. */
	return 0;
}

/* ----------------------------------------------------------------------
 * appnic_hard_start_xmit
 *
 * The method initiates the transmission of a packet.  The full packet
 * (protocol headers and all) is contained in a socket buffer (sk_buff)
 * structure.
 *
 * ----- NOTES -----
 *
 * 1) This will not get called again by the kernel until it returns.
 */

static int appnic_hard_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);
	int length;
	int buf_per_desc;
	union appnic_queue_pointer queue;
	unsigned long flags;
	struct device *d = &dev->dev;

	spin_lock_irqsave(&pdata->tx_lock, flags);

	if (eth_skb_pad(skb))
		return NETDEV_TX_OK;

	length = skb->len;
	buf_per_desc = pdata->tx_buf_sz / pdata->tx_num_desc;

	/* If enough transmit descriptors are available, copy and transmit. */

	queue = swab_queue_pointer(pdata->tx_tail);
	while (((length / buf_per_desc) + 1) >=
		queue_uninitialized(pdata->tx_head,
				    queue,
				    pdata->tx_num_desc)) {
		handle_transmit_interrupt(dev);
		queue = swab_queue_pointer(pdata->tx_tail);
	}

	if (((length / buf_per_desc) + 1) <
		queue_uninitialized(pdata->tx_head, queue,
				    pdata->tx_num_desc)) {
		int bytes_copied = 0;
		struct appnic_dma_descriptor descriptor;

		readdescriptor(((unsigned long)pdata->tx_desc +
				pdata->tx_head.bits.offset), &descriptor);
		descriptor.start_of_packet = 1;

		while (bytes_copied < length) {
			descriptor.write = 1;
			descriptor.pdu_length = length;

			if ((length - bytes_copied) > buf_per_desc) {
				memcpy((void *)
					(descriptor.host_data_memory_pointer +
					 pdata->dma_alloc_offset_tx),
				       (void *)((unsigned long)skb->data +
					bytes_copied),
					buf_per_desc);
				descriptor.data_transfer_length = buf_per_desc;
				descriptor.end_of_packet = 0;
				descriptor.interrupt_on_completion = 0;
				bytes_copied += buf_per_desc;
			} else {
				memcpy((void *)
					(descriptor.host_data_memory_pointer +
					 pdata->dma_alloc_offset_tx),
				       (void *)((unsigned long)skb->data +
					bytes_copied),
					(length - bytes_copied));
				descriptor.data_transfer_length =
				 (length - bytes_copied);
				descriptor.end_of_packet = 1;
				/* Leave TX interrupts disabled. We work
				 * the same with or w/o them. Set to "1"
				 * if we ever want to enable them though.
				 */
				descriptor.interrupt_on_completion = 0;
				bytes_copied = length;
			}

			pdata->stats.tx_bytes += bytes_copied;
			writedescriptor(((unsigned long)pdata->tx_desc +
				pdata->tx_head.bits.offset), &descriptor);
			queue_increment(&pdata->tx_head, pdata->tx_num_desc);
			readdescriptor(((unsigned long)pdata->tx_desc +
					 pdata->tx_head.bits.offset),
					&descriptor);
			descriptor.start_of_packet = 0;
		}

		/* Data sync barrier. */
		rmb();

		write_mac(pdata->tx_head.raw, APPNIC_DMA_TX_HEAD_POINTER);
		netif_trans_update(dev);

	} else {
		pdata->out_of_tx_descriptors++;
		dev_err(d, "No transmit descriptors available!\n");
		spin_unlock_irqrestore(&pdata->tx_lock, flags);
		return NETDEV_TX_BUSY;
	}

	spin_unlock_irqrestore(&pdata->tx_lock, flags);
	/* Free the socket buffer. */
	dev_kfree_skb(skb);

	return NETDEV_TX_OK;
}

/* ----------------------------------------------------------------------
 * appnic_net_device_stats
 *
 * Whenever an application needs to get statistics for the interface,
 * this method is called.  This happens, for example, when ifconfig or
 * nstat -i is run.
 */

static struct net_device_stats *appnic_get_stats(struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);

	/* Update the statistics structure. */

	get_hw_statistics(pdata);

	return &pdata->stats;
}

/* ----------------------------------------------------------------------
 * appnic_set_mac_address
 */

static int appnic_set_mac_address(struct net_device *dev, void *data)
{
	struct appnic_device *pdata = netdev_priv(dev);
	struct sockaddr *address = data;
	unsigned long swap_source_address;

	if (netif_running(dev))
		return -EBUSY;

	if (!is_valid_ether_addr(address->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(dev->dev_addr, address->sa_data, ETH_ALEN);
	memcpy(dev->perm_addr, address->sa_data, ETH_ALEN);

	swap_source_address = ((address->sa_data[4]) << 8) |
			       address->sa_data[5];
	write_mac(swap_source_address, APPNIC_SWAP_SOURCE_ADDRESS_2);
	swap_source_address = ((address->sa_data[2]) << 8) |
			address->sa_data[3];
	write_mac(swap_source_address, APPNIC_SWAP_SOURCE_ADDRESS_1);
	swap_source_address = ((address->sa_data[0]) << 8) |
			       address->sa_data[1];
	write_mac(swap_source_address, APPNIC_SWAP_SOURCE_ADDRESS_0);
	memcpy(dev->dev_addr, address->sa_data, dev->addr_len);

	return 0;
}

/* ----------------------------------------------------------------------
 * appnic_set_rx_mode
 */

static void appnic_set_rx_mode(struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);
	unsigned long dev_flags;

	/* Acquire the lock. */
	spin_lock_irqsave(&pdata->dev_lock, dev_flags);

	if (dev->flags & IFF_PROMISC)
		pdata->promiscuous_mode = 1;
	else
		pdata->promiscuous_mode = 0;

	/* Release the lock */
	spin_unlock_irqrestore(&pdata->dev_lock, dev_flags);
}

 /* ======================================================================
 * ETHTOOL Operations
 * ======================================================================
 */

enum {
	NETDEV_STATS, APPNIC_STATS
};

struct appnic_stats {
	char stat_string[ETH_GSTRING_LEN];
	int sizeof_stat;
	int stat_offset;
};

#define APPNIC_STAT(str, m) {	    \
		.stat_string = str,					\
		.sizeof_stat = sizeof(((struct appnic_device *)0)->m), \
		.stat_offset = offsetof(struct appnic_device, m) }

static const struct appnic_stats appnic_gstrings_stats[] = {
	APPNIC_STAT("rx_packets", stats.rx_packets),
	APPNIC_STAT("tx_packets", stats.tx_packets),
	APPNIC_STAT("rx_bytes", stats.rx_bytes),
	APPNIC_STAT("tx_bytes", stats.tx_bytes),
	APPNIC_STAT("rx_errors", stats.rx_errors),
	APPNIC_STAT("tx_errors", stats.tx_errors),
	APPNIC_STAT("rx_dropped", stats.rx_dropped),
	APPNIC_STAT("tx_dropped", stats.tx_dropped),
	APPNIC_STAT("multicast", stats.multicast),
	APPNIC_STAT("collisions", stats.collisions),
	APPNIC_STAT("rx_length_errors", stats.rx_length_errors),
	APPNIC_STAT("rx_crc_errors", stats.rx_crc_errors),
	APPNIC_STAT("rx_frame_errors", stats.rx_frame_errors),
	APPNIC_STAT("rx_fifo_errors", stats.rx_fifo_errors),
	APPNIC_STAT("tx_fifo_errors", stats.tx_fifo_errors),

	APPNIC_STAT("dropped_by_stack", dropped_by_stack),
	APPNIC_STAT("out_of_tx_descriptors", out_of_tx_descriptors),
	APPNIC_STAT("transmit_interrupts", transmit_interrupts),
	APPNIC_STAT("receive_interrupts", receive_interrupts),
};

#define APPNIC_GLOBAL_STATS_LEN  ARRAY_SIZE(appnic_gstrings_stats)
#define APPNIC_STATS_LEN (APPNIC_GLOBAL_STATS_LEN)

/* ----------------------------------------------------------------------
 * appnic_get_ethtool_stats
 */

static void appnic_get_ethtool_stats(struct net_device *dev,
				     struct ethtool_stats *stats,
				     u64 *data)
{
	struct appnic_device *pdata = netdev_priv(dev);
	int i;
	char *p = NULL;

	get_hw_statistics(pdata);
	for (i = 0; i < APPNIC_GLOBAL_STATS_LEN; i++) {
		p = (char *)pdata + appnic_gstrings_stats[i].stat_offset;
		data[i] = (appnic_gstrings_stats[i].sizeof_stat ==
			sizeof(u64)) ? *(u64 *)p : *(u32 *)p;
	}
}

/* ----------------------------------------------------------------------
 * appnic_get_strings
 */

static void appnic_get_strings(struct net_device *netdev, u32 stringset,
			       u8 *data)
{
	u8 *p = data;
	int i;

	switch (stringset) {
	case ETH_SS_STATS:
		for (i = 0; i < APPNIC_GLOBAL_STATS_LEN; i++) {
			memcpy(p, appnic_gstrings_stats[i].stat_string,
			       ETH_GSTRING_LEN);
			p += ETH_GSTRING_LEN;
		}
		break;
	}
}

/* ----------------------------------------------------------------------
 * appnic_get_sset_count
 */

static int appnic_get_sset_count(struct net_device *netdev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return APPNIC_STATS_LEN;
	default:
		return -EOPNOTSUPP;
	}
}

/* ----------------------------------------------------------------------
 * appnic_get_drvinfo
 */

static void appnic_get_drvinfo(struct net_device *dev,
			       struct ethtool_drvinfo *info)
{
	strcpy(info->driver, AXXIA_DRV_NAME);
	strcpy(info->version, AXXIA_DRV_VERSION);
	strlcpy(info->bus_info, dev_name(dev->dev.parent),
		sizeof(info->bus_info));
}

/* Fill in the struture...  */

static const struct ethtool_ops appnic_ethtool_ops = {
	.get_drvinfo		= appnic_get_drvinfo,
	.get_ethtool_stats	= appnic_get_ethtool_stats,
	.get_strings		= appnic_get_strings,
	.get_sset_count		= appnic_get_sset_count,
	.get_link_ksettings	= phy_ethtool_get_link_ksettings,
};

/* ======================================================================
 * Linux Module Interface.
 * ======================================================================
 */

static const struct net_device_ops appnic_netdev_ops = {
	.ndo_open = appnic_open,
	.ndo_stop = appnic_stop,
	.ndo_get_stats = appnic_get_stats,
	.ndo_set_mac_address = appnic_set_mac_address,
	.ndo_start_xmit = appnic_hard_start_xmit,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller = appnic_poll_controller,
#endif
	.ndo_set_rx_mode = appnic_set_rx_mode,
};

/* ----------------------------------------------------------------------
 * appnic_init
 */

int appnic_init(struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);
	void *dma_offset;
	int index;
	unsigned long buf;
	struct appnic_dma_descriptor descriptor;
	struct sockaddr address;
	unsigned long node_cfg;
	int rc = 0;
	struct device *d = &dev->dev;

	/* Set FEMAC to uncached */
	femac_uncache(pdata);

	/* Reset the MAC. */

	write_mac(0x80000000, APPNIC_DMA_PCI_CONTROL);

	/* Allocate memory and initialize the descriptors. */

	/* fixup num_[rt]x_desc. */

	if (0 != (rx_num_desc % DESCRIPTOR_GRANULARITY)) {
		dev_err(d, "rx_num_desc was not a multiple of %d.\n",
			DESCRIPTOR_GRANULARITY);
		rc = -EINVAL;
		goto err_param;
	}

	pdata->rx_num_desc = rx_num_desc;

	if (0 != (tx_num_desc % DESCRIPTOR_GRANULARITY)) {
		dev_err(d, "tx_num_desc was not a multiple of %d.\n",
			DESCRIPTOR_GRANULARITY);
		rc = -EINVAL;
		goto err_param;
	}

	pdata->tx_num_desc = tx_num_desc;

	/* up [rt]x_buf_sz. Must be some multiple of 64 bytes
	 * per descriptor.
	 */

	if (0 != (rx_buf_sz % (BUFFER_ALIGNMENT * rx_num_desc))) {
		dev_err(d, "rx_buf_sz was not a multiple of %d.\n",
			(BUFFER_ALIGNMENT * rx_num_desc));
		rc = -EINVAL;
		goto err_param;
	}

	pdata->rx_buf_sz = rx_buf_sz;

	if (0 != (tx_buf_sz % (BUFFER_ALIGNMENT * tx_num_desc))) {
		dev_err(d, "tx_buf_sz was not a multiple of %d.\n",
			(BUFFER_ALIGNMENT * tx_num_desc));
		rc = -EINVAL;
		goto err_param;
	}

	pdata->tx_buf_sz = tx_buf_sz;

	/* Allocate dma-able memory. Broken into smaller parts to keep
	 * from allocating a single large chunk of memory, but not too
	 * small since mappings obtained from dma_alloc_coherent() have
	 * a minimum size of one page.
	 */

	pdata->dma_alloc_size =
		/* The tail pointers (rx and tx) */
		(sizeof(union appnic_queue_pointer) * 2) +
		/* The RX descriptor ring (and padding to allow
		 * 64 byte alignment)
		 */
		(sizeof(struct appnic_dma_descriptor) * pdata->rx_num_desc) +
		(DESCRIPTOR_GRANULARITY) +
		/* The TX descriptor ring (and padding...) */
		(sizeof(struct appnic_dma_descriptor) * pdata->tx_num_desc) +
		(DESCRIPTOR_GRANULARITY);

	pdata->dma_alloc_size_rx =
		/* The RX buffer (and padding...) */
		(pdata->rx_buf_sz) + (BUFFER_ALIGNMENT);

	pdata->dma_alloc_size_tx =
		/* The TX buffer (and padding...) */
		(pdata->tx_buf_sz) + (BUFFER_ALIGNMENT);

	/* Allocate the buffers. */
	rc = femac_alloc_mem_buffers(dev);
	if (rc != 0) {
		dev_err(d, "Can't allocate DMA-able memory!\n");
		goto err_mem_buffers;
	}

	/* Initialize the tail pointers. */

	dma_offset = pdata->dma_alloc;

	pdata->rx_tail = (union appnic_queue_pointer *)dma_offset;
	pdata->rx_tail_dma = (int)pdata->rx_tail - (int)pdata->dma_alloc_offset;
	dma_offset += sizeof(union appnic_queue_pointer);
	memset((void *)pdata->rx_tail, 0, sizeof(union appnic_queue_pointer));

	pdata->tx_tail = (union appnic_queue_pointer *)dma_offset;
	pdata->tx_tail_dma = (int)pdata->tx_tail - (int)pdata->dma_alloc_offset;
	dma_offset += sizeof(union appnic_queue_pointer);
	memset((void *)pdata->tx_tail, 0, sizeof(union appnic_queue_pointer));

	/* Initialize the descriptor pointers. */

	pdata->rx_desc = (struct appnic_dma_descriptor *)ALIGN64B(dma_offset);
	pdata->rx_desc_dma = (int)pdata->rx_desc - (int)pdata->dma_alloc_offset;
	dma_offset += (sizeof(struct appnic_dma_descriptor) *
			pdata->rx_num_desc) + (DESCRIPTOR_GRANULARITY);
	memset((void *)pdata->rx_desc, 0,
	       (sizeof(struct appnic_dma_descriptor) * pdata->rx_num_desc));

	pdata->tx_desc = (struct appnic_dma_descriptor *)ALIGN64B(dma_offset);
	pdata->tx_desc_dma = (int)pdata->tx_desc - (int)pdata->dma_alloc_offset;
	dma_offset += (sizeof(struct appnic_dma_descriptor) *
			pdata->tx_num_desc) + (DESCRIPTOR_GRANULARITY);
	memset((void *)pdata->tx_desc, 0,
	       (sizeof(struct appnic_dma_descriptor) * pdata->tx_num_desc));

	/* Initialize the buffer pointers. */

	dma_offset = pdata->dma_alloc_rx;

	pdata->rx_buf = (void *)ALIGN64B(dma_offset);
	pdata->rx_buf_dma = (int)pdata->rx_buf -
				(int)pdata->dma_alloc_offset_rx;
	pdata->rx_buf_per_desc = pdata->rx_buf_sz / pdata->rx_num_desc;

	dma_offset = pdata->dma_alloc_tx;

	pdata->tx_buf = (void *)ALIGN64B(dma_offset);
	pdata->tx_buf_dma = (int)pdata->tx_buf -
				(int)pdata->dma_alloc_offset_tx;
	pdata->tx_buf_per_desc = pdata->tx_buf_sz / pdata->tx_num_desc;

	/* Initialize the descriptors. */

	buf = (unsigned long)pdata->rx_buf_dma;
	for (index = 0; index < pdata->rx_num_desc; ++index) {
		memset((void *)&descriptor, 0,
		       sizeof(struct appnic_dma_descriptor));
		descriptor.write = 1;
		descriptor.interrupt_on_completion = 1;
		descriptor.host_data_memory_pointer = buf;
		descriptor.data_transfer_length = pdata->rx_buf_per_desc;

		writedescriptor(((unsigned long)pdata->rx_desc + (index *
				sizeof(struct appnic_dma_descriptor))),
				&descriptor);

		buf += pdata->rx_buf_per_desc;
	}

	buf = (unsigned long)pdata->tx_buf_dma;

	for (index = 0; index < pdata->tx_num_desc; ++index) {
		memset((void *)&descriptor, 0,
		       sizeof(struct appnic_dma_descriptor));
		descriptor.write = 1;
		descriptor.interrupt_on_completion = 1;
		descriptor.host_data_memory_pointer = buf;

		writedescriptor(((unsigned long)pdata->tx_desc + (index *
				 sizeof(struct appnic_dma_descriptor))),
				&descriptor);

		buf += pdata->tx_buf_per_desc;
	}

	/* Set promiscuous mode to off. */
	pdata->promiscuous_mode = 0;

	/* Initialize the spinlocks. */

	spin_lock_init(&pdata->dev_lock);
	spin_lock_init(&pdata->tx_lock);

	/* Take MAC out of reset. */

	write_mac(0x0, APPNIC_RX_SOFT_RESET);
	write_mac(0x1, APPNIC_RX_MODE);
	write_mac(0x0, APPNIC_TX_SOFT_RESET);
	write_mac(0x1, APPNIC_TX_MODE);

	/* Set the watermark. */

	ncr_read(NCP_REGION_ID(0x16, 0xff), 0x10, 4, &node_cfg);

	if (0 == (0x80000000 & node_cfg))
		write_mac(0x300a, APPNIC_TX_WATERMARK);
	else
		write_mac(0xc00096, APPNIC_TX_WATERMARK);

	write_mac(0x1, APPNIC_TX_HALF_DUPLEX_CONF);
	write_mac(0xffff, APPNIC_TX_TIME_VALUE_CONF);
	write_mac(0x1, APPNIC_TX_INTERRUPT_CONTROL);
	write_mac(0x5275, APPNIC_TX_EXTENDED_CONF);
	write_mac(0x1, APPNIC_RX_INTERNAL_INTERRUPT_CONTROL);
	write_mac(0x1, APPNIC_RX_EXTERNAL_INTERRUPT_CONTROL);
	write_mac(0x40010000, APPNIC_DMA_PCI_CONTROL);
	write_mac(0x30000, APPNIC_DMA_CONTROL);
	writel(0x280044,
	       (void __iomem *)((unsigned long)pdata->dma_base + 0x60));
	writel(0xc0,
	       (void __iomem *)((unsigned long)pdata->dma_base + 0x64));

	/* Set the MAC address. */
	dev_info(d, "MAC %pM\n", dev->dev_addr);

	memcpy(&address.sa_data[0], dev->dev_addr, ETH_ALEN);
	rc = appnic_set_mac_address(dev, &address);
	if (rc != 0) {
		dev_err(d, "Unable to set MAC address!\n");
		goto err_set_mac_addr;
	}

	/* Initialize the queue pointers. */

	/* Receiver. */

	memset((void *)&pdata->rx_tail_copy, 0,
	       sizeof(union appnic_queue_pointer));
	memset((void *)&pdata->rx_head, 0,
	       sizeof(union appnic_queue_pointer));

	write_mac(pdata->rx_desc_dma, APPNIC_DMA_RX_QUEUE_BASE_ADDRESS);
	write_mac((pdata->rx_num_desc *
		   sizeof(struct appnic_dma_descriptor)) / 1024,
		  APPNIC_DMA_RX_QUEUE_SIZE);

	/* Indicate that all of the receive descriptors
	 * are ready.
	 */

	pdata->rx_head.bits.offset = (pdata->rx_num_desc - 1) *
					sizeof(struct appnic_dma_descriptor);
	write_mac(pdata->rx_tail_dma, APPNIC_DMA_RX_TAIL_POINTER_ADDRESS);

	/* N.B.
	 *
	 * The boot loader may have used the NIC.  If so, the
	 * tail pointer must be read and the head pointer (and
	 * local copy of the tail) based on it.
	 */

	pdata->rx_tail->raw =
		  read_mac(APPNIC_DMA_RX_TAIL_POINTER_LOCAL_COPY);
	pdata->rx_tail_copy.raw = pdata->rx_tail->raw;
	pdata->rx_head.raw = pdata->rx_tail->raw;
	queue_decrement(&pdata->rx_head, pdata->rx_num_desc);
	pdata->rx_head.bits.generation_bit =
		(pdata->rx_head.bits.generation_bit == 0) ? 1 : 0;
	write_mac(pdata->rx_head.raw, APPNIC_DMA_RX_HEAD_POINTER);

	/* Transmitter. */

	memset((void *)&pdata->tx_tail_copy, 0,
	       sizeof(union appnic_queue_pointer));
	memset((void *)&pdata->tx_head, 0,
	       sizeof(union appnic_queue_pointer));

	write_mac(pdata->tx_desc_dma, APPNIC_DMA_TX_QUEUE_BASE_ADDRESS);
	write_mac((pdata->tx_num_desc *
		   sizeof(struct appnic_dma_descriptor)) / 1024,
		  APPNIC_DMA_TX_QUEUE_SIZE);
	write_mac(pdata->tx_tail_dma, APPNIC_DMA_TX_TAIL_POINTER_ADDRESS);

	/* N.B.
	 *
	 * The boot loader may have used the NIC.  If so, the
	 * tail pointer must be read and the head pointer (and
	 * local copy of the tail) based on it.
	 */

	pdata->tx_tail->raw = read_mac(APPNIC_DMA_TX_TAIL_POINTER_LOCAL_COPY);
	pdata->tx_tail_copy.raw = pdata->tx_tail->raw;
	pdata->tx_head.raw = pdata->tx_tail->raw;
	write_mac(pdata->tx_head.raw, APPNIC_DMA_TX_HEAD_POINTER);

	/* Clear statistics. */

	clear_statistics(pdata);

	/* Fill in the net_device structure. */

	ether_setup(dev);

	/* Setup IRQ. */
	rc = femac_irq_setup(dev);
	if (rc != 0) {
		dev_err(d, "IRQ setup failed!\n");
		goto err_irq_setup;
	}

	dev->netdev_ops = &appnic_netdev_ops;
	dev->ethtool_ops = &appnic_ethtool_ops;
	dev->features |= NETIF_F_LLTX;
	dev->min_mtu = ETH_ZLEN - ETH_HLEN;
	dev->max_mtu = 0x3f00 - (ETH_HLEN + ETH_FCS_LEN);

	memset((void *)&pdata->napi, 0, sizeof(struct napi_struct));
	netif_napi_add(dev, &pdata->napi,
		       axxianet_poll, AXXIANET_NAPI_WEIGHT);

	return 0;

err_irq_setup:
err_set_mac_addr:
	femac_free_mem_buffers(dev);
err_mem_buffers:
err_param:
	return rc;
}

/* ----------------------------------------------------------------------
 * appnic_probe_config_dt
 */

#ifdef CONFIG_OF
static int appnic_probe_config_dt(struct net_device *dev,
				  struct device_node *np)
{
	struct appnic_device *pdata = netdev_priv(dev);
	u8 mac[ETH_ALEN];
	struct device_node *gp_node;
	struct device *d = &dev->dev;
	int err;

	if (!np)
		return -ENODEV;

	gp_node = of_find_compatible_node(NULL, NULL, "axxia,gpreg");
	if (!gp_node) {
		dev_err(d, "DTS is missing mode 'gpreg'\n");
		return -ENODEV;
	}
	pdata->gpreg_base = of_iomap(gp_node, 0);

	pdata->rx_base = of_iomap(np, 0);
	pdata->tx_base = of_iomap(np, 1);
	pdata->dma_base = of_iomap(np, 2);

	pdata->tx_interrupt = irq_of_parse_and_map(np, 0);
	pdata->rx_interrupt = irq_of_parse_and_map(np, 1);
	pdata->dma_interrupt = irq_of_parse_and_map(np, 2);

	err = of_get_mac_address(np, mac);
	if (err)
		goto device_tree_failed;

	memcpy(&pdata->mac_addr[0], mac, ETH_ALEN);
	memcpy(dev->dev_addr, mac, ETH_ALEN);
	memcpy(dev->perm_addr, mac, ETH_ALEN);

	return 0;

device_tree_failed:
	dev_err(d, "Reading Device Tree Failed\n");
	iounmap(pdata->gpreg_base);
	iounmap(pdata->rx_base);
	iounmap(pdata->tx_base);
	iounmap(pdata->dma_base);

	return -EINVAL;
}
#else
static inline int appnic_probe_config_dt(struct net_device *dev,
					 struct device_node *np)
{
	return -ENODEV;
}
#endif /* CONFIG_OF */

/* ----------------------------------------------------------------------
 * appnic_drv_probe
 */

static int appnic_drv_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct device_node *np = pdev->dev.of_node;
	struct net_device *ndev;
	struct appnic_device *pdata;
	struct device *d;

	pr_info("%s: INTEL(R) 10/100 Network Driver - version %s\n",
		AXXIA_DRV_NAME, AXXIA_DRV_VERSION);
	dev_info(&pdev->dev, "JURA: PDU overrun tracing enabled\n");

	/* Allocate space for the device. */
	ndev = alloc_etherdev(sizeof(struct appnic_device));
	if (!ndev) {
		pr_err("%s: Couldn't allocate net device.\n", AXXIA_DRV_NAME);
		rc = -ENOMEM;
		goto err_alloc_etherdev;
	}

	SET_NETDEV_DEV(ndev, &pdev->dev);
	platform_set_drvdata(pdev, ndev);

	pdata = netdev_priv(ndev);
	pdata->netdev = ndev;
	pdata->dev = &pdev->dev;
	d = &ndev->dev;

	if (dma_set_mask(&pdev->dev, DMA_BIT_MASK(32)) ||
	    dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32))) {
		dev_err(d, "No usable DMA configuration, aborting\n");
		rc = -EIO;
		goto err_inval;
	}

	/* In the case of a fixed PHY, the DT node associated
	 * to the PHY is the Ethernet MAC DT node.
	 */
	if (of_phy_is_fixed_link(np)) {
		rc = of_phy_register_fixed_link(np);
		if (rc) {
			dev_err(d, "Failed to register fixed PHY\n");
			goto err_inval;
		}
		pdata->phy_dn = of_node_get(np);
	} else {
		pdata->phy_dn = of_parse_phandle(np, "phy-handle", 0);

		if (!pdata->phy_dn) {
			dev_err(d, "No phy-handle\n");
			rc = -ENODEV;
			goto err_inval;
		}
	}

	/* Get the physical addresses, interrupt number, etc. from the
	 * device tree.  If no entry exists (older boot loader...) just
	 * use the pre-devicetree method.
	 */
	rc = appnic_probe_config_dt(ndev, np);

	if (rc == -EINVAL) {
		goto err_inval;
	} else if (rc == -EINVAL) {
#ifdef CONFIG_MTD_NAND_EP501X_UBOOTENV

		/* The attempt to get device settings from the DTB failed, so
		 * try to grab the ethernet MAC from the u-boot environment
		 * and use hard-coded values for device base addresses.
		 */

		unsigned char ethaddr_string[20];

		if (ubootenv_get("ethaddr", ethaddr_string) != 0) {
			dev_err(d, "Could not read ethernet address!\n");
			rc = -EINVAL;
			goto err_inval;
		} else {
			u8 mac_address[ETH_ALEN];
			int i = 0;
			char *string = ethaddr_string;

			while ((string != 0) && (i < ETH_ALEN)) {
				char *value;
				unsigned long res;

				value = strsep(&string, ":");
				if (kstrtoul(value, 16, &res))
					return -EBUSY;
				mac_address[i++] = (u8)res;
			}

			memcpy(ndev->dev_addr, mac_address, ETH_ALEN);
			memcpy(ndev->perm_addr, mac_address, ETH_ALEN);
			ndev->addr_len = ETH_ALEN;

			dev_info(d, "Using Static Addresses and Interrupts");
			pdata->rx_base = ioremap(0x002000480000ULL, 0x1000);
			pdata->tx_base = ioremap(0x002000481000ULL, 0x1000);
			pdata->dma_base = ioremap(0x002000482000ULL, 0x1000);
			pdata->dma_interrupt = 33;
		}
#else
		/* Neither dtb info nor ubootenv driver found. */
		dev_err(d, "Could not read ethernet address!");
		rc = -EINVAL;
		goto err_inval;
#endif
	}

#ifdef CONFIG_MTD_NAND_EP501X_UBOOTENV

	{
		unsigned char uboot_env_string[20];

		/* Override ad_value with u-boot environment variable if set. */
		if (ubootenv_get("ad_value", uboot_env_string) == 0) {
			/* Assume ad_value is always entered as a hex value,
			 * since u-boot defaults this value as hex.
			 */
			unsigned long res;

			if (kstrtoul(uboot_env_string, 16, &res)) {
				rc = -EINVAL;
				goto err_inval;
			}
			pdata->ad_value = res;
		}
	}

#endif

	/* Initialize the device. */
	rc = appnic_init(ndev);
	if (rc != 0) {
		dev_err(d, "appnic_init() failed: %d\n", rc);
		rc = -ENODEV;
		goto err_nodev;
	}

	/* Register the device. */
	rc = register_netdev(ndev);
	if (rc != 0) {
		dev_err(d, "register_netdev() failed: %d\n", rc);
		rc = -ENODEV;
		goto err_nodev;
	}

	pdata->phy_dev = of_phy_connect(pdata->netdev, pdata->phy_dn,
					appnic_handle_link_change, 0,
					PHY_INTERFACE_MODE_MII);

	if (IS_ERR_OR_NULL(pdata->phy_dev)) {
		pr_err("Could not attach to PHY\n");
		return -ENODEV;
	}

	return 0;

err_nodev:
err_inval:
	free_netdev(ndev);
err_alloc_etherdev:
	return rc;
}

/* ----------------------------------------------------------------------
 * appnic_drv_remove
 */

static int appnic_drv_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct appnic_device *pdata = NULL;

	dev_info(&dev->dev, "stopping driver");

	WARN_ON(!dev);
	if (!dev)
		return -ENODEV;

	pdata = netdev_priv(dev);

	WARN_ON(!pdata);
	if (!pdata)
		return -ENODEV;

	WARN_ON(!pdata->phy_dev);
	if (!pdata->phy_dev)
		return -ENODEV;

	phy_disconnect(pdata->phy_dev);
	pdata->phy_dev = NULL;
	platform_set_drvdata(pdev, NULL);
	unregister_netdev(dev);
	free_irq(dev->irq, dev);
	femac_free_mem_buffers(dev);
	free_netdev(dev);

	iounmap(pdata->rx_base);
	iounmap(pdata->tx_base);
	iounmap(pdata->dma_base);
	iounmap(pdata->gpreg_base);

	return 0;
}

/* ----------------------------------------------------------------------
 * axxianet_reset
 */

static void axxianet_reset(struct net_device *ndev)
{
	struct appnic_device *pdata = netdev_priv(ndev);
	struct appnic_dma_descriptor descriptor;
	struct device *dev = &ndev->dev;
	unsigned long swap_source_address;
	struct sockaddr address;
	unsigned long node_cfg;
	unsigned long buf;
	unsigned long flags;
	int index;

	dev_warn(dev, "Resetting Axxia FEMAC");

	/* Block transmits until done! */
	spin_lock_irqsave(&pdata->tx_lock, flags);

	/* Reset the MAC */
	write_mac(0x80000000, APPNIC_DMA_PCI_CONTROL);

	/* Initialize the Descriptor Rings and Buffers */
	memset((void *)pdata->rx_tail, 0, sizeof(union appnic_queue_pointer));
	memset((void *)pdata->tx_tail, 0, sizeof(union appnic_queue_pointer));
	memset((void *)pdata->rx_desc, 0,
	       (sizeof(struct appnic_dma_descriptor) * pdata->rx_num_desc));
	memset((void *)pdata->tx_desc, 0,
	       (sizeof(struct appnic_dma_descriptor) * pdata->tx_num_desc));

	buf = (unsigned long)pdata->rx_buf_dma;
	for (index = 0; index < pdata->rx_num_desc; ++index) {
		memset((void *)&descriptor, 0,
		       sizeof(struct appnic_dma_descriptor));
		descriptor.write = 1;
		descriptor.interrupt_on_completion = 1;
		descriptor.host_data_memory_pointer = buf;
		descriptor.data_transfer_length = pdata->rx_buf_per_desc;

		writedescriptor(((unsigned long)pdata->rx_desc + (index *
				sizeof(struct appnic_dma_descriptor))),
				&descriptor);

		buf += pdata->rx_buf_per_desc;
	}

	buf = (unsigned long)pdata->tx_buf_dma;

	for (index = 0; index < pdata->tx_num_desc; ++index) {
		memset((void *)&descriptor, 0,
		       sizeof(struct appnic_dma_descriptor));
		descriptor.write = 1;
		descriptor.interrupt_on_completion = 1;
		descriptor.host_data_memory_pointer = buf;

		writedescriptor(((unsigned long)pdata->tx_desc + (index *
				 sizeof(struct appnic_dma_descriptor))),
				&descriptor);

		buf += pdata->tx_buf_per_desc;
	}

	/* Take MAC out of reset. */
	write_mac(0x0, APPNIC_RX_SOFT_RESET);
	write_mac(0x1, APPNIC_RX_MODE);
	write_mac(0x0, APPNIC_TX_SOFT_RESET);
	write_mac(0x1, APPNIC_TX_MODE);

	/* Set the watermark. */
	ncr_read(NCP_REGION_ID(0x16, 0xff), 0x10, 4, &node_cfg);

	if (0 == (0x80000000 & node_cfg))
		write_mac(0x300a, APPNIC_TX_WATERMARK);
	else
		write_mac(0xc00096, APPNIC_TX_WATERMARK);

	write_mac(0x1, APPNIC_TX_HALF_DUPLEX_CONF);
	write_mac(0xffff, APPNIC_TX_TIME_VALUE_CONF);
	write_mac(0x1, APPNIC_TX_INTERRUPT_CONTROL);
	write_mac(0x5275, APPNIC_TX_EXTENDED_CONF);
	write_mac(0x1, APPNIC_RX_INTERNAL_INTERRUPT_CONTROL);
	write_mac(0x1, APPNIC_RX_EXTERNAL_INTERRUPT_CONTROL);
	write_mac(0x40010000, APPNIC_DMA_PCI_CONTROL);
	write_mac(0x30000, APPNIC_DMA_CONTROL);
	writel(0x280044,
	       (void __iomem *)((unsigned long)pdata->dma_base + 0x60));
	writel(0xc0,
	       (void __iomem *)((unsigned long)pdata->dma_base + 0x64));

	/* Write the MAC address back. */
	memcpy(&address.sa_data, ndev->dev_addr, ETH_ALEN);
	swap_source_address = ((address.sa_data[4]) << 8) |
			       address.sa_data[5];
	write_mac(swap_source_address, APPNIC_SWAP_SOURCE_ADDRESS_2);
	swap_source_address = ((address.sa_data[2]) << 8) |
			address.sa_data[3];
	write_mac(swap_source_address, APPNIC_SWAP_SOURCE_ADDRESS_1);
	swap_source_address = ((address.sa_data[0]) << 8) |
			       address.sa_data[1];
	write_mac(swap_source_address, APPNIC_SWAP_SOURCE_ADDRESS_0);

	/* Initialize the queue pointers. */

	/* Receiver. */
	memset((void *)&pdata->rx_tail_copy, 0,
	       sizeof(union appnic_queue_pointer));
	memset((void *)&pdata->rx_head, 0,
	       sizeof(union appnic_queue_pointer));

	write_mac(pdata->rx_desc_dma, APPNIC_DMA_RX_QUEUE_BASE_ADDRESS);
	write_mac((pdata->rx_num_desc *
		   sizeof(struct appnic_dma_descriptor)) / 1024,
		  APPNIC_DMA_RX_QUEUE_SIZE);

	/* Indicate that all of the receive descriptors are ready. */
	pdata->rx_head.bits.offset = (pdata->rx_num_desc - 1) *
					sizeof(struct appnic_dma_descriptor);
	write_mac(pdata->rx_tail_dma, APPNIC_DMA_RX_TAIL_POINTER_ADDRESS);

	/* Match RX pointers. */
	pdata->rx_tail->raw = read_mac(APPNIC_DMA_RX_TAIL_POINTER_LOCAL_COPY);
	pdata->rx_tail_copy.raw = pdata->rx_tail->raw;
	pdata->rx_head.raw = pdata->rx_tail->raw;
	queue_decrement(&pdata->rx_head, pdata->rx_num_desc);
	pdata->rx_head.bits.generation_bit =
		(pdata->rx_head.bits.generation_bit == 0) ? 1 : 0;
	write_mac(pdata->rx_head.raw, APPNIC_DMA_RX_HEAD_POINTER);

	/* Transmitter. */
	memset((void *)&pdata->tx_tail_copy, 0,
	       sizeof(union appnic_queue_pointer));
	memset((void *)&pdata->tx_head, 0,
	       sizeof(union appnic_queue_pointer));

	write_mac(pdata->tx_desc_dma, APPNIC_DMA_TX_QUEUE_BASE_ADDRESS);
	write_mac((pdata->tx_num_desc *
		   sizeof(struct appnic_dma_descriptor)) / 1024,
		  APPNIC_DMA_TX_QUEUE_SIZE);
	write_mac(pdata->tx_tail_dma, APPNIC_DMA_TX_TAIL_POINTER_ADDRESS);

	/* Match TX pointers. */
	pdata->tx_tail->raw = read_mac(APPNIC_DMA_TX_TAIL_POINTER_LOCAL_COPY);
	pdata->tx_tail_copy.raw = pdata->tx_tail->raw;
	pdata->tx_head.raw = pdata->tx_tail->raw;
	write_mac(pdata->tx_head.raw, APPNIC_DMA_TX_HEAD_POINTER);

	/* Sync up the MAC and the PHY */
	appnic_handle_link_change(ndev);

	/* Enable interrupts. */
	write_mac((APPNIC_DMA_INTERRUPT_ENABLE_RECEIVE |
		   APPNIC_DMA_INTERRUPT_ENABLE_TRANSMIT),
		   APPNIC_DMA_INTERRUPT_ENABLE);

	/* Unblock transmits. */
	spin_unlock_irqrestore(&pdata->tx_lock, flags);
}

static const struct of_device_id appnic_dt_ids[] = {
	{ .compatible = "axxia,acp-femac", },
	{ .compatible = "acp-femac", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, appnic_dt_ids);

static struct platform_driver appnic_driver = {
	.probe = appnic_drv_probe,
	.remove = appnic_drv_remove,
	.driver = {
		.name   = AXXIA_DRV_NAME,
		.owner  = THIS_MODULE,
		.pm     = NULL,
		.of_match_table = appnic_dt_ids,
	},
};

module_platform_driver(appnic_driver);
