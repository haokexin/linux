/*
 * fp_net_driver.c
 *
 * SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (C)2024Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <linux/if_ether.h>
#include <linux/crc32.h>
#include <linux/mii.h>
#include <linux/if.h>
#include <linux/if_vlan.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/prefetch.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_reserved_mem.h>
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif /* CONFIG_DEBUG_FS */
#include <linux/net_tstamp.h>
#include <net/pkt_cls.h>
#include <linux/reset.h>
#include <linux/of_mdio.h>
#include <linux/bst_boardconfig.h>
#include <linux/phylink.h>
#include "fp_net_driver.h"
#include "fp_types.h"
#include "fp_bd_api.h"
#include "fp_library.h"
#if defined(CONFIG_BST_C1200_DB)
#include "db_client.h"
#elif defined(CONFIG_BST_C1200_IVI)
#include "ivi_client.h"
#endif

UCHAR  *fp_baseAddr = NULL;

int g_fp_initial = 0;
#if defined(CONFIG_BST_C1200_DB)
db_client_t *client = NULL;
db_client_data_t client_data = {0};
#elif defined(CONFIG_BST_C1200_IVI)
ivi_client_t *client = NULL;
ivi_client_data_t client_data = {0};
#endif

#define TX_TIMEO	5000
static int watchdog = TX_TIMEO;
module_param(watchdog, int, 0644);
MODULE_PARM_DESC(watchdog, "Hif transmit timeout in milliseconds (default 5s)");

static void hif_netif_del_napi(struct fp_private *priv);
static void hif_netif_disable_napi(struct fp_private *priv);
netdev_tx_t fp_xmit_frame_ring(struct fp_private *fp, struct sk_buff *skb,
				struct bd_tx_ring *tx_ring);
netdev_tx_t fp_xmit_frame_ring_broadcast(struct fp_private *fp, struct sk_buff *skb,
				struct bd_tx_ring *tx_ring);

/**********************************************************************
 * Function Name  : fp_disable_interrupts
 * Description    : clear all pending interrupts
 * Inputs
 *   Parameters   : struct fp_private *
 * Outputs        :
 *   Parameters   : -
 *   Returns      : -
 * Changes        :
 *********************************************************************/
void fp_disable_interrupts(struct fp_private *fp)
{
	UINT  int_src;
	volatile struct hif_regs *regs;
	UINT hif_index = 0, ch_index = 0;

	regs = fp->hif[hif_index].regs;


	/* Clear the pending interrupts */
	int_src = CSR_REG_READ(fp_baseAddr, (ULONG)&regs->hif_int_src);
	CSR_REG_WRITE(fp_baseAddr, (ULONG)&regs->hif_int_src, int_src);

	for (ch_index = HIF_CHAN_START; ch_index < NUM_HIF_CHANNELS; ch_index++) {
		/* disable the channel[ch_index] interrupt enable */
		CSR_REG_WRITE(fp_baseAddr,
				(ULONG)&fp->hif[hif_index].ch_r[ch_index]->hif_ch_int_en, 0);

		/* Clear the channel[ch_index]
		   pending interrupt sources */
		int_src = CSR_REG_READ(fp_baseAddr,
				(ULONG)&fp->hif[hif_index].ch_r[ch_index]->hif_ch_int_src);
		CSR_REG_WRITE(fp_baseAddr,
				(ULONG)&fp->hif[hif_index].ch_r[ch_index]->hif_ch_int_src,
				int_src);
	}

	dev_info(fp->device, "%s: int_src is 0x%x\n", __func__, int_src);
}

void fp_range_disable_interrupts(struct fp_private *fp, int start, int end, int index)
{
	UINT hif_index = 0, ch_index = 0;


	start += index * HIF_CHANNELS_PER_IRQ;
	end += index * HIF_CHANNELS_PER_IRQ;

//	int_src = 0;
	/* Clear the pending interrupts */
//	int_src = CSR_REG_READ(fp_baseAddr, (ULONG)&regs->hif_int_src);

	for (ch_index = start; ch_index < end + 1; ch_index++) {

//		int_src & (HIF_CH_INT << ch_index);
		/* disable the channel[ch_index] interrupt enable */
		CSR_REG_WRITE(fp_baseAddr,
				(ULONG)&fp->hif[hif_index].ch_r[ch_index]->hif_ch_int_en, 0);
//		pr_err("ch_index = %d, hif_ch_int en address = %px, hif_ch_int_src = %px \n", ch_index, 
//				(ULONG)&fp->hif[hif_index].ch_r[ch_index]->hif_ch_int_en,
//				(ULONG)&fp->hif[hif_index].ch_r[ch_index]->hif_ch_int_src);

#if 0
		/* Clear the channel[ch_index]
		   pending interrupt sources */
		int_src = CSR_REG_READ(fp_baseAddr,
				(ULONG)&regs->ch[ch_index].hif_ch_int_src);
		CSR_REG_WRITE(fp_baseAddr,
				(ULONG)&regs->ch[ch_index].hif_ch_int_src,
				int_src);
#endif
	}

//	CSR_REG_WRITE(fp_baseAddr, (ULONG)&regs->hif_int_src, int_src);

#if 0 
	dev_info(fp->device, "%s: int_src is 0x%x\n", __func__, int_src);
#endif
}

void fp_enable_interrupts(struct fp_private *fp)
{
	UINT hif_index = 0, ch_index = 0;

	for (ch_index = HIF_CHAN_START; ch_index < NUM_HIF_CHANNELS; ch_index++) {
		/* Enable all hif channel interrupts */
		CSR_REG_WRITE(fp_baseAddr,
				(ULONG)&fp->hif[hif_index].ch_r[ch_index]->hif_ch_int_en,
				(HIF_CH_INT_EN | HIF_CH_RXPKT_INT_EN |
				 HIF_CH_TXPKT_INT_EN));
	}
}

void fp_range_enable_interrupts(struct fp_private *fp, int start, int end, int index)
{
	volatile struct hif_regs *regs;
	UINT hif_index = 0, ch_index = 0;
	u32 int_src = 0;


//	pr_err("rangel enable.\n");
	start += index * HIF_CHANNELS_PER_IRQ;
	end += index * HIF_CHANNELS_PER_IRQ;

	regs = fp->hif[hif_index].regs;
//	int_src = CSR_REG_READ(fp_baseAddr, (ULONG)&regs->hif_int_src);
	for (ch_index = start; ch_index < end + 1; ch_index++) {
		/* init the DMA */
#if 0
		rx_ctrl = CSR_REG_READ(fp_baseAddr,
				(ULONG)&fp->hif[hif_index].ch_r[ch_index]->hif_ctrl_ch);
		rx_ctrl |= HIF_CH_RX_CTRL_DMA_EN;
		CSR_REG_WRITE(fp_baseAddr,
				(ULONG)&fp->hif[hif_index].ch_r[ch_index]->hif_ctrl_ch, rx_ctrl);

		rx_ctrl = CSR_REG_READ(fp_baseAddr,
				(ULONG)&fp->hif[hif_index].ch_r[ch_index]->hif_rx_ch_start);
		rx_ctrl |= HIF_CH_RX_START;
		CSR_REG_WRITE(fp_baseAddr,
				(ULONG)&fp->hif[hif_index].ch_r[ch_index]->hif_rx_ch_start, rx_ctrl);
#endif

		int_src = CSR_REG_READ(fp_baseAddr,
				(ULONG)&fp->hif[hif_index].ch_r[ch_index]->hif_ch_int_src);
		CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[hif_index].ch_r[ch_index]->hif_ch_int_src, int_src);
//		pr_err("in range enabel hif_ch_int_src = %px, address = %px.\n", int_src, (ULONG)&fp->hif[hif_index].ch_r[ch_index]->hif_ch_int_src);
		/* Enable all hif channel interrupts */
		CSR_REG_WRITE(fp_baseAddr,
				(ULONG)&fp->hif[hif_index].ch_r[ch_index]->hif_ch_int_en,
				(HIF_CH_INT_EN | HIF_CH_RXPKT_INT_EN |
				 HIF_CH_TXPKT_INT_EN));
		dma_wmb();

//		int_src = 0;
//		int_src |= (HIF_CH_INT << ch_index);
	}
#if 0
	pr_err("int range enable int_src = %px.\n", int_src);
	CSR_REG_WRITE(fp_baseAddr, (ULONG)&regs->hif_int_src, int_src);

	int_src = CSR_REG_READ(fp_baseAddr, (ULONG)&regs->hif_int_src);
	pr_err("renage enable hif_int_src = %px.\n", int_src);
#endif
}

static int hif_match_ch_irq(struct fp_private *fp, int irq)
{
	int i;

	for (i = 0; i < HIF_MAX_IRQ; i++) {
		if (fp->channel[i].irq == irq) {
			return fp->channel[i].index;
		}
	}

	return -1;
}




static u32 fp_get_hif_int_src_value(struct fp_private *fp, u32 index, volatile int *offset)
{
	u32 addr = 0;
	volatile struct hif_regs *regs = fp->hif[0].regs;


	if (index >= 0 && index < 2) {
		addr = (ULONG)&regs->hif_int_src;
	    *offset = index;
	}
	else if (index >= 2 && index < 6) {
		addr = (ULONG)&regs->hif_int_src_reg2;
	    *offset = index -2;
	}
	else if (index >= 6 && index < 8) {
		addr = (ULONG)&regs->hif_int_src_reg3;
	    *offset = index -6;
	}

	return addr;
}

#if 0
static void fp_rx_dma_reinit(struct fp_private *fp, u32 ch_index)
{
	UINT rx_ctrl = 0;

	/* init the DMA */
	rx_ctrl = CSR_REG_READ(fp_baseAddr,
				(ULONG)&fp->hif[0].ch_r[ch_index]->hif_ctrl_ch);
	rx_ctrl |= HIF_CH_RX_CTRL_DMA_EN;
	CSR_REG_WRITE(fp_baseAddr,
			(ULONG)&fp->hif[0].ch_r[ch_index]->hif_ctrl_ch, rx_ctrl);

	rx_ctrl = CSR_REG_READ(fp_baseAddr,
			(ULONG)&fp->hif[0].ch_r[ch_index]->hif_rx_ch_start);
	rx_ctrl |= HIF_CH_RX_START;
	CSR_REG_WRITE(fp_baseAddr,
			(ULONG)&fp->hif[0].ch_r[ch_index]->hif_rx_ch_start, rx_ctrl);

	/* Re-Enable interrupts */
	CSR_REG_WRITE(fp_baseAddr,
			(ULONG)&fp->hif[0].ch_r[ch_index]->hif_ch_int_en,
			HIF_CH_INT_EN | HIF_CH_RXPKT_INT_EN |
			HIF_CH_TXPKT_INT_EN);

}
#endif

/**
 * fp_consume_page - helper function for Rx path
 * @bi: software descriptor shadow data
 * @skb: skb being modified
 * @length: length of data being added
 **/
static void fp_consume_page(struct bd_rx_buffer *bi, struct sk_buff *skb,
			       u16 length)
{
//	bi->rxbuf.page = NULL;
	WRITE_ONCE(bi->rxbuf.page, NULL);
	skb->len += length;
	skb->data_len += length;
	skb->truesize += PAGE_SIZE;
}

/**
 * fp_channel_recv - Send received data up the network stack
 * @fp_private *fp: the fp private data structure
 * @index: channel index 
 * @work_done: amount of napi work completed this call
 * @work_to_do: max amount of work allowed for this call to do
 *
 */

static void fp_channel_recv(struct fp_private *fp, int index, int * work_done, int work_to_do)
{
	struct bd_rx_ring *ring = fp->hif[0].ch[index].rxring;
	int ch_index = ring->ch_index;
	struct napi_struct *napi = &fp->channel[ch_index].napi;
	struct bd *rxbd, *next_rxbd = NULL;
	struct bd_rx_buffer *buffer_info, *next_buffer = NULL;
	bool lifm = false, next_lifm = false;
	volatile int cleaned_count = 0;
	UINT wb_buflen = 0, next_wb_buflen = 0;
	unsigned int total_rx_bytes = 0, total_rx_packets = 0;
	unsigned int i;
	int len;
#define cleaned_start ring->clean_start

#if 0
	char *addr;
	int j;
#endif



	while (likely( *work_done < work_to_do)) {

		(*work_done)++;

		if (likely(next_rxbd)) {
			/* use prefetched values */
			rxbd = next_rxbd;
			buffer_info = next_buffer;
			wb_buflen = next_wb_buflen;
			lifm = next_lifm;
		}
		else {
			rxbd = fp_deque_rx_bd(ring, &wb_buflen, &lifm);
			if (rxbd == NULL) {
//				pr_err("channel %d: RX wb bd not ready\n", index);
				break;
			}
			/* Get the curr rx bd index */
			i = fp_get_curr_rx_bd_index(ring, rxbd);
			buffer_info = &ring->buffer_info[i];
			cleaned_count++;
		}

		if (test_bit(HIFVMAC_RX_FIFO_CLEAR, &fp->state)) {
			__free_page(buffer_info->rxbuf.page);
			buffer_info->rxbuf.page = NULL;
			WRITE_ONCE(buffer_info->dma, 0);
			goto next_desc;
		}

		if (!lifm) {
			next_rxbd = fp_deque_rx_bd(ring, &next_wb_buflen, &next_lifm);
			if (next_rxbd == NULL) {
//				pr_err("channel %d: RX wb bd  next not ready\n", index);
				break;
			}
			else {
				prefetch(next_rxbd);
				i = fp_get_curr_rx_bd_index(ring, next_rxbd);
				next_buffer = &ring->buffer_info[i];
				cleaned_count++;
			}
		}
		else {
			next_rxbd = NULL;
		}

//	pr_err("lifm = %d, i = %d, buffer_info->dma = %px.\n", lifm,  i, buffer_info->dma);
//	pr_err("rxbd address = %px, rxbd->bd_buffaddr = %px\n", rxbd, rxbd->bd_bufaddr);

	    dma_rmb();
		dma_unmap_page(fp->device, buffer_info->dma,
				MAX_RX_BUFF_SIZE, DMA_FROM_DEVICE);
//		cleaned_count++;

#define rxtop ring->rx_skb_top
//process_skb:
		if (!lifm) {
			/* this descriptor is only the beginning (or middle) */
			if (!rxtop) {
				/* this is the beginning of a chain */
				rxtop = napi_get_frags(napi);
				if (!rxtop) {
					__free_page(buffer_info->rxbuf.page);
					buffer_info->rxbuf.page = NULL;
//					buffer_info->dma = 0;
					WRITE_ONCE(buffer_info->dma, 0);
					ring->stats.alloc_failed++;
					break;
				}


#if 0
				addr = (char *)page_address(buffer_info->rxbuf.page);
//				pr_err("multi first frag wb_buflen = %d.\n",wb_buflen);
				for(j = 0; j < wb_buflen; j++) {
					pr_cont("%02x ", addr[j]); 
				}
				pr_err("\n\r");
#endif


				skb_fill_page_desc(rxtop, 0,
						buffer_info->rxbuf.page,
						16, wb_buflen - 16);
//						0, wb_buflen);

			fp_consume_page(buffer_info, rxtop, wb_buflen - 16);
			} else {
#if 0
				addr = (char *)page_address(buffer_info->rxbuf.page);
				pr_err("multi middle frag wb_buflen = %d.\n",wb_buflen);
				for(j = 0; j < wb_buflen; j++) {
					pr_cont("%02x ", addr[j]); 
				}
				pr_err("\n\r");
#endif

				/* this is the middle of a chain */
				skb_fill_page_desc(rxtop,
						skb_shinfo(rxtop)->nr_frags,
//						buffer_info->rxbuf.page, 16, wb_buflen - 16);
						buffer_info->rxbuf.page, 0, wb_buflen);
			fp_consume_page(buffer_info, rxtop, wb_buflen);
			}
//			buffer_info->dma = 0;
			WRITE_ONCE(buffer_info->dma, 0);
			goto next_desc;
		} else {
			if (rxtop) {

#if 0
				addr = (char *)page_address(buffer_info->rxbuf.page);
				pr_err("multi last frag wb_buflen = %d.\n",wb_buflen);
				for(j = 0; j < wb_buflen; j++) {
					pr_cont("%02x ", addr[j]); 
				}
				pr_err("\n\r");
#endif


				/* end of the chain */
				skb_fill_page_desc(rxtop,
						skb_shinfo(rxtop)->nr_frags,
//						buffer_info->rxbuf.page, 16, wb_buflen - 16);
						buffer_info->rxbuf.page, 0, wb_buflen);
				fp_consume_page(buffer_info, rxtop, wb_buflen);
//				buffer_info->dma = 0;
				WRITE_ONCE(buffer_info->dma, 0);
//				fp_consume_page(buffer_info, rxtop, wb_buflen);
			} else {
				if (wb_buflen <= 30) {
					__free_page(buffer_info->rxbuf.page);
					buffer_info->rxbuf.page = NULL;
//					buffer_info->dma = 0;
					WRITE_ONCE(buffer_info->dma, 0);
#if  0
				u8 *vaddr;
				struct sk_buff *skb = napi_alloc_skb(napi, wb_buflen);
				if (!skb)
					pr_crit("alloc skb failed.\n");

					vaddr = kmap_atomic(buffer_info->rxbuf.page);
					memcpy(skb_tail_pointer(skb), vaddr + 16,
					       wb_buflen - 16);
				for(j = 0; j < wb_buflen; j++) {
					pr_cont("%02x ", vaddr[j]); 
				}
				pr_err("\n\r");
					kunmap_atomic(vaddr);
					/* re-use the page, so don't erase
					 * buffer_info->rxbuf.page
					 */
					skb->ip_summed = CHECKSUM_UNNECESSARY;
					skb_put(skb, wb_buflen - 16);
					skb->protocol = eth_type_trans(skb, fp->dev);
					netif_receive_skb(skb);
//					 napi_gro_receive(napi, skb);
					pr_err("debbug hit :recevive short packet.\n");
#endif
					goto next_desc;
				}

				rxtop = napi_get_frags(napi);
				if (!rxtop) {
					__free_page(buffer_info->rxbuf.page);
					buffer_info->rxbuf.page = NULL;
//					buffer_info->dma = 0;
					WRITE_ONCE(buffer_info->dma, 0);
					ring->stats.alloc_failed++;
					break;
				}

#if 0
				addr = (char *)page_address(buffer_info->rxbuf.page);
				pr_err("receive pakcet count %d.\n",total_rx_packets);
				pr_err("single frag wb_buflen = %d.\n",wb_buflen);
				for(j = 0; j < wb_buflen; j++) {
					pr_cont("%02x ", addr[j]); 
				}
				pr_err("\n\r");
#endif



				skb_fill_page_desc(rxtop, 0,
						buffer_info->rxbuf.page,
						16, wb_buflen - 16);
//						0, wb_buflen);
				fp_consume_page(buffer_info, rxtop, wb_buflen - 16);
//				buffer_info->dma = 0;
				WRITE_ONCE(buffer_info->dma, 0);
//				fp_consume_page(buffer_info, rxtop, wb_buflen);
			}
		}


		rxtop->ip_summed = CHECKSUM_UNNECESSARY;
//		fp->dev->stats.rx_bytes += (rxtop->len - 4); /* don't count FCS */
//		fp->dev->stats.rx_packets ++; 

		total_rx_bytes += (rxtop->len - 4); /* don't count FCS */
		total_rx_packets++;
		rxtop = NULL;
		napi_gro_frags(napi);

next_desc:

	/*-----------------------------------------------------------------------------
	 * NUM_RX_BD_LIMIT <= ring->ring_len 
	 *-----------------------------------------------------------------------------*/
		/* return some buffers to hardware, one at a time is too slow */
		if (unlikely(cleaned_count >= NUM_RX_BD_LIMIT)) {
			len = fp_bd_init_rx_ring(fp, ring, cleaned_start, cleaned_count, false);
			cleaned_start = (cleaned_start + cleaned_count - len) % ring->ring_len;
			cleaned_count = 0;
		}

	}

	/* place incomplete frames back on ring for completion */
//	ring->rx_skb_top = rxtop;

	cleaned_count = fp_rx_bd_used(ring);
//	pr_err("cleaned_cout = %d, clean_start = %d.\n", cleaned_count, cleaned_start);
//	pr_err("work_done = %d, work_todo = %d.\n", *work_done, work_to_do);

	if (cleaned_count) {
		len = fp_bd_init_rx_ring(fp, ring, cleaned_start, cleaned_count, false);
		cleaned_start = (cleaned_start + cleaned_count - len) % ring->ring_len;
		(*work_done) = 64;
	}

//	ring->stats.bytes += total_rx_bytes;
//	ring->stats.packets += total_rx_packets;
	fp->dev->stats.rx_bytes += total_rx_bytes;
	fp->dev->stats.rx_packets += total_rx_packets;

	return;
}

static void handle_rx_channel(struct fp_private *fp, int index, int * work_done, int work_to_do)
{
	UINT hif_ch_int_src = CSR_REG_READ(fp_baseAddr,
		(ULONG)&fp->hif[0].ch_r[index]->hif_ch_int_src);

	if (hif_ch_int_src & HIF_CH_RXPKT_INT) {
//		CSR_REG_WRITE(fp_baseAddr, (ULONG)&regs->ch[index].hif_ch_int_src, 
//				(1 << HIF_CH_RXPKT_INT_EN) |(1 << HIF_CH_RXBD_INT));
		pr_debug("%s: RX channel %d : GOT CALLED, INTSRC IS 0x%x\n",
				__func__, index, hif_ch_int_src);

		/* Schedule the deferred intr processing routine */
		fp_channel_recv(fp, index, work_done, work_to_do);
	}
}

static int hif_clean_rx_irq(struct hif_channel *ch, int work_to_do)
{
	int work_done = 0;
	int i , ch_index;

	/* return if interrupt is not ours */
	if (!ch->hif_int_src)
		return -1;

//	pr_err("hif_int_src = %px.\n", ch->hif_int_src);
	for (i = ch->start; work_done < work_to_do && 
			i < ch->end + 1; i++) {
		if (ch->hif_int_src & (HIF_CH_INT << (i + 8 * ch->offset))) {
			ch_index = i + 8 * ch->index;
			handle_rx_channel(ch->fp, ch_index, &work_done, work_to_do);
		}
	}

	return work_done;
}

/*-----------------------------------------------------------------------------
 * Reclaim the tx channel bd resource.
 *-----------------------------------------------------------------------------*/
static void fp_channel_send(struct fp_private *fp, int index, int * work_done, int work_to_do)
{
	struct bd *txbd;
	u32 wb_buflen = 0, buff_index;
	//struct netdev_queue *queue = netdev_get_tx_queue(fp->dev, index);
	struct bd_tx_ring *txring = fp->hif[0].ch[index].txring;
	struct net_device *netdev = fp->dev;
	struct bd_tx_buffer *buffer;
	bool lifm = false;

	__netif_tx_lock_bh(netdev_get_tx_queue(fp->dev, txring->queue_index));

//	spin_lock(&fp->lock);
	while (likely(*work_done < work_to_do)) {

		(*work_done)++;
		txbd = fp_deque_tx_bd(txring, &wb_buflen, &lifm);
		if (txbd == NULL) {
//			pr_err("TX wb bd not ready\n");
			break;
		}
		buff_index = fp_get_curr_tx_bd_index(txring, txbd);
		buffer = &txring->buffer_info[buff_index];
		if (buffer->skb) {
//			pr_err("release skb.\n");
			dma_unmap_single(fp->device, buffer->dma, buffer->len,
					DMA_TO_DEVICE);
			napi_consume_skb(buffer->skb, work_to_do);
//			dev_consume_skb_any(buffer->skb);
			buffer->skb = NULL;
		}

		//pr_err("tx wb_buflen = %d, lifm = %d.\n", wb_buflen, lifm);
		netdev->stats.tx_bytes += (wb_buflen - 16);
		if (lifm)
			netdev->stats.tx_packets++;
	}
//	spin_unlock(&fp->lock);

	if (unlikely(netif_tx_queue_stopped(netdev_get_tx_queue(fp->dev,
								txring->queue_index))) &&
		fp_tx_bd_unused(txring) > HIF_TX_THRESH) {
		netif_tx_wake_queue(netdev_get_tx_queue(fp->dev, txring->queue_index));
		pr_err("%s: wake tx queue\n", __func__);
	}

#if 0
	CSR_REG_WRITE(fp_baseAddr,
			(ULONG)&fp->hif[0].regs->ch[index].hif_ch_int_en,
			HIF_CH_INT_EN | HIF_CH_RXPKT_INT_EN |
			HIF_CH_TXPKT_INT_EN);
#endif
	__netif_tx_unlock_bh(netdev_get_tx_queue(fp->dev, txring->queue_index));
}

static void handle_tx_channel(struct fp_private *fp, int index, int * work_done, int work_to_do)
{
	UINT hif_ch_int_src = CSR_REG_READ(fp_baseAddr,
		(ULONG)&fp->hif[0].ch_r[index]->hif_ch_int_src);
//	pr_err("hif_ch_int_src = %px, index = %d.\n", hif_ch_int_src, index);

	if (hif_ch_int_src & HIF_CH_TXPKT_INT) {
		pr_debug("%s: RX channel %d : GOT CALLED, INTSRC IS 0x%x\n",
				__func__, index, hif_ch_int_src);

		/* Schedule the deferred intr processing routine */
		fp_channel_send(fp, index, work_done, work_to_do);
	}
}

static bool hif_clean_tx_irq(struct hif_channel *ch, int work_to_do)
{
	int work_done = 0;
	int i , ch_index;

//	pr_err("ch->hif_int_src = %px.\n", ch->hif_int_src);

	/* return if interrupt is not ours */
	if (!ch->hif_int_src)
		return false;

	for (i = ch->start; work_done < work_to_do && 
			i < ch->end + 1; i++) {
		if (ch->hif_int_src & (HIF_CH_INT << (i + 8 * ch->offset))) {
			ch_index = i + 8 * ch->index;
			handle_tx_channel(ch->fp, ch_index, &work_done, work_to_do);
		}
	}

	if (work_done >= work_to_do)
		return false;

	return true;
}


/**
 *  igb_poll - NAPI Rx polling callback
 *  @napi: napi polling structure
 *  @budget: count of how many packets we should handle
 **/
static int hif_napi_poll(struct napi_struct *napi, int budget)
{
	struct hif_channel *ch = container_of(napi,
						     struct hif_channel,
						     napi);
	bool clean_complete = true;
	int hif_index = 0, ch_index;
	int cleaned = 0;
	u32 rx_ctrl = 0;

//	clean_complete = hif_clean_tx_irq(ch, ch->tx_clean_limit);
	clean_complete = hif_clean_tx_irq(ch, budget);
//	pr_err("tx clean_complete = %d.\n", clean_complete);


	cleaned = hif_clean_rx_irq(ch, budget);


//	pr_err("cleand = %d, budget = %d.\n", cleaned, budget);
	if (cleaned >= budget)
		clean_complete = false;
	else if(cleaned) {
		for (ch_index = ch->start; ch_index < ch->end + 1; ch_index++) {
			/* init the DMA */
			rx_ctrl = CSR_REG_READ(fp_baseAddr,
					(ULONG)&ch->fp->hif[hif_index].ch_r[ch_index]->hif_ctrl_ch);
			rx_ctrl |= HIF_CH_RX_CTRL_DMA_EN;
			CSR_REG_WRITE(fp_baseAddr,
					(ULONG)&ch->fp->hif[hif_index].ch_r[ch_index]->hif_ctrl_ch, rx_ctrl);

			rx_ctrl = CSR_REG_READ(fp_baseAddr,
					(ULONG)&ch->fp->hif[hif_index].ch_r[ch_index]->hif_rx_ch_start);
			rx_ctrl |= HIF_CH_RX_START;
			CSR_REG_WRITE(fp_baseAddr,
					(ULONG)&ch->fp->hif[hif_index].ch_r[ch_index]->hif_rx_ch_start, rx_ctrl);
		}
	}

	/* If all work not completed, return budget and keep polling */
	if (!clean_complete)
		return budget;

	/* Exit the polling mode, but don't re-enable interrupts if stack might
	 * poll us due to busy-polling
	 */
	if (likely(napi_complete_done(napi, cleaned))) {
//		pr_err("tx clean done.\n");
		fp_range_enable_interrupts(ch->fp, ch->start, ch->end, ch->index);
	}
//	pr_err("(%s): index  = %d, start = %d, end = %d\n", __func__, ch->index, ch->start, ch->end);

	return cleaned;
}

/**
 * fp_intr - Interrupt Handler
 * @irq: interrupt number
 * @data: pointer to fp private structure
 **/
static irqreturn_t fp_intr(int irq, void *data)
{
	struct fp_private *fp = data;
	struct hif_channel *ch = NULL;
	u32 addr = 0;
	int index;
	


	index = hif_match_ch_irq(fp, irq);

	if (index == -1) {
		pr_err("%s: Not found the channel index.\n", __func__);
		return IRQ_NONE;
	}

	ch = &fp->channel[index];
//	pr_err("(%s): index  = %d, start = %d, end = %d\n", __func__, index, ch->start, ch->end);

	addr = fp_get_hif_int_src_value(ch->fp, ch->index, &ch->offset);
	ch->hif_int_src = CSR_REG_READ(fp_baseAddr, addr);
//	pr_err("(%s): addr  = %px, offset = %d, ch->hif_int_src= %px\n", __func__, addr, ch->offset, ch->hif_int_src);

	fp_range_disable_interrupts(fp, ch->start, ch->end, index); 


	if (likely(napi_schedule_prep(&fp->channel[index].napi))) {
#if 0
		fp->total_tx_bytes = 0;
		fp->total_tx_packets = 0;
		fp->total_rx_bytes = 0;
		fp->total_rx_packets = 0;
#endif
		__napi_schedule(&fp->channel[index].napi);
	} else {
		/* this really should not happen! if it does it is basically a
		 * bug, but not a hard error, so enable ints and continue
		 */
#if 0
		fp_range_enable_interrupts(fp, ch->start, ch->end, index);
#endif
	}

	return IRQ_HANDLED;
}

static int fp_request_irq(struct fp_private *fp)
{
	struct net_device *netdev = fp->dev;
	irq_handler_t handler = fp_intr;
	int irq_flags = IRQF_SHARED;
	u32 i;
	int err;

	for (i = 0; i < HIF_MAX_IRQ; i++) {
		if (fp->channel[i].irq) {
//			pr_err("request irq num = %d.\n", fp->channel[i].irq);
			err = request_irq(fp->channel[i].irq, handler, irq_flags, netdev->name,
					fp);
			if (err) {
				pr_err("Unable to allocate channle %d interrupt Error: %d\n", i, err);
				for (i-- ; i >= 0; i--) {
					if (fp->channel[i].irq) {
						free_irq(fp->channel[i].irq, fp);
					}
				}
				break;
			}
		}
	}

	return err;
}

static void fp_free_irq(struct fp_private *fp)
{
//	struct net_device *netdev = fp->dev;
	u32 i;

	for (i = 0; i < HIF_MAX_IRQ; i++) {
		synchronize_irq(fp->channel[i].irq);
		free_irq(fp->channel[i].irq, fp);
	}

	return;
}

static void fp_napi_enable(struct fp_private *fp)
{
	u32 i;

	for (i = 0; i < HIF_MAX_IRQ; i++) {
		napi_enable(&fp->channel[i].napi);
	}
	return;
}

/**
 * fp_send_message
 * @ndev: net device struct
 * Description: send message to sw
 * Return:
 * returns 0 on success, otherwise errno.
*/

int fp_send_message(struct net_device *ndev, int flag)
{
	int ret = 0;
	unsigned char mac_data[6];
#if defined(CONFIG_BST_C1200_DB)
	db_switch_MyArray_t mac;
	db_switch_ErrorEnum_t msg_err = 0;
#elif defined(CONFIG_BST_C1200_IVI)
	ivi_switch_MyArray_t mac;
	ivi_switch_ErrorEnum_t msg_err = 0;
#endif

	if (!ndev) {
		ret = -1;
		pr_err("ERROR: vmac net device struct is NULL\n");
		goto end;
	}

	memcpy(mac_data, ndev->dev_addr, ndev->addr_len);
	mac.data = &mac_data[0];
	mac.size = ndev->addr_len;

	if (client) {
#if defined(CONFIG_BST_C1200_DB)
		client->db_switch_client.vmac_method_sync(mac, flag,  &msg_err, 1000, NULL);
#elif defined(CONFIG_BST_C1200_IVI)
		client->ivi_switch_client.vmac_method_sync(mac, flag,  &msg_err, 1000, NULL);
#endif
		printk("%s vmac msgbox send success\n", __func__);
	} else {
		printk("WARNING: vmac msgbox uninit\n");
		ret = -2;
    }

end:
	return ret;
}

/**
 * fp_sw_init - Initialize general software structures (struct fp_private)
 * @fppriv: hif private structure to initialize
 *
 * fp_sw_init initializes the hif private data structure.
 **/
static int fp_sw_init(struct fp_private *fppriv)
{
//	fppriv->rx_buffer_len = MAXIMUM_ETHERNET_VLAN_SIZE;

//	fppriv->num_tx_queues = 1;
//	fppriv->num_rx_queues = 1;

	if (fp_bd_create_ring(fppriv)) {
		pr_err("Unable to allocate memory for queues\n");
		return -ENOMEM;
	}

	spin_lock_init(&fppriv->lock);
	spin_lock_init(&fppriv->tx_clean_lock);
	spin_lock_init(&fppriv->rx_lock);

	return 0;
}

/**
 * fp_net_open - Called when a network interface is made active
 * @netdev: network interface device structure
 *
 * Returns 0 on success, negative value on failure
 *
 * The open entry point is called when a network interface is made
 * active by the system (IFF_UP).  At this point all resources needed
 * for transmit and receive operations are allocated, the interrupt
 * handler is registered with the OS.
 **/
int fp_net_open(struct net_device *dev)
{
	struct fp_private *fp = netdev_priv(dev);
	int err;

	if(!g_fp_initial) {

		err = fp_sw_init(fp);
		if (err)
			goto err_sw_init;

		/* allocate transmit descriptors */
		err = fp_bd_setup_all_tx_rings(fp);
		if (err)
			goto err_setup_tx;

		/* allocate receive descriptors */
		err = fp_bd_setup_all_rx_rings(fp);
		if (err)
			goto err_setup_rx;

		fp_bd_init_all_tx_rings(fp);
		fp_bd_init_all_rx_rings(fp);


		err = fp_request_irq(fp);
		if (err)
			goto err_req_irq;

		/* Notify the stack of the actual queue counts. */
		err = netif_set_real_num_tx_queues(fp->dev, fp->q_num);
	//					   HIF_MAX_IRQ);
		if (err)
			goto err_set_queues;

		err = netif_set_real_num_rx_queues(fp->dev, fp->q_num);
	//					   HIF_MAX_IRQ);
		if (err)
			goto err_set_queues;

		fp_napi_enable(fp);


		fp_disable_interrupts(fp);
		/* CLEAR - Program the bdp reg base addresses */
		fp_init_bdp_base_reg(fp);
		fp_enable_interrupts(fp);

		fp_bd_reinit(fp);
		g_fp_initial = 1;

		printk("%s res init done\n", __func__);
	}
	
	dma_wmb();
	netif_tx_start_all_queues(fp->dev);

#if defined(CONFIG_BST_C1200_DB)
	err = fp_send_message(fp->dev, DB_MAC_ADDR_FLAG);
#elif defined(CONFIG_BST_C1200_IVI)
	err = fp_send_message(fp->dev, IVI_MAC_ADDR_FLAG);
#endif
	if (err) {
		printk("%s vmac mac addr send fail, errno=%d\n", __func__, err);
	}

	set_bit(HIFVMAC_RUNING, &fp->state);

//	fp_bd_reinit(fp);


	/*-----------------------------------------------------------------------------
	 *  todo : GPI_DTX_ASEQ && set_queue_chan_mapping alreay done at R5 core,
	 *  maybe we should not repeat again.
	 *-----------------------------------------------------------------------------*/

	/*-----------------------------------------------------------------------------
	 *  todo : xgmac already initial at r5 core.
	 *-----------------------------------------------------------------------------*/
	return 0;

err_set_queues:
	fp_free_irq(fp);
err_req_irq:
	fp_bd_uninit_all_rx_rings(fp);
	fp_bd_free_all_rx_rings(fp);
err_setup_rx:
	fp_bd_free_all_tx_rings(fp);
err_setup_tx:
err_sw_init:
	fp_bd_destroy_ring(fp);
	return err;
}

static inline struct netdev_queue *txring_txq(struct fp_private *fp, 
		const struct bd_tx_ring *tx_ring)
{
	return netdev_get_tx_queue(fp->dev, tx_ring->ring_index);
}

static int fp_add_tx_pkt(struct fp_private *fp, struct bd_tx_ring *txring, 
		struct sk_buff *skb)
{
	struct bd *txbd;
//	UCHAR *ptr, *ptr1;
	uint32_t ret, size, index;
	dma_addr_t dma;
	struct bd_tx_buffer *buffer;

	dma_mb();
	txbd = fp_get_next_free_tx_bd(txring);
	if (!txbd) {
		netif_stop_subqueue(fp->dev, txring->ring_index);
		/* this is a hard error */
		return -ENOSPC;
	}


	index = fp_get_curr_tx_bd_index(txring, txbd);
	buffer = &txring->buffer_info[index];


	size = skb_headlen(skb);
	dma = dma_map_single(fp->device, skb->data, size, DMA_TO_DEVICE);
	if (dma_mapping_error(fp->device, dma)) {
		pr_debug("tx dma map error.\n");
		return -EFAULT;
	}

	if (!buffer->skb) {
		buffer->skb = skb;
		buffer->dma = dma;
		buffer->len = size;
	}
	else {
		pr_err("tx skb not released.\n");
	}


	txbd->bd_bufaddr = (u32) (dma);
	txbd->msb8_bd_bufaddr = (u8) (dma >> 32);

	/* update the ctrl word necessary proper bd flags */
	txbd->bd_ctrl |= (BD_CTRL_PKT_INT_EN | BD_CTRL_CBD_INT_EN | BD_CTRL_LIFM);
	txbd->bd_buflen = size;

	/* enqueue bd to hardware */
	ret = fp_enque_tx_bd(fp, txring, txbd);
	if (ret < 0) {
		pr_debug("tx bd not avail this should never happen\n");
		return -EBUSY;
	}

	return NETDEV_TX_OK;
}

static inline struct bd_tx_ring * fp_tx_queue_mapping(struct fp_private *fp,
						    struct sk_buff *skb)
{
	unsigned int r_idx = skb->queue_mapping;
	unsigned long bit;

//	pr_err("before r_idx = %d.\n", r_idx);

	if (r_idx >= NUM_HIF_CHANNELS)
		r_idx = r_idx % NUM_HIF_CHANNELS;

//	pr_err("after r_idx = %d.\n", r_idx);

	for_each_set_bit(bit, fp->_channel_mask, NUM_MAX_HIF_CHANNELS) {
		if (r_idx-- == 0) {
			break;
		}
	}

//	pr_err("bit = %ld.\n", bit);
	
	return fp->hif[0].ch[bit].txring;
}

#if defined(CONFIG_BST_C1200_DB)
//unsigned char ivi_dest[ETH_ALEN] = {0x52, 0xAE, 0xCD, 0x28, 0x26, 0x4B};
unsigned char ivi_dest[ETH_ALEN] = {0x01, 0x00, 0x5E, 0x40, 0x50, 0x60};
#elif defined(CONFIG_BST_C1200_IVI)
//unsigned char db_dest[ETH_ALEN] = {0x52, 0xAE, 0xCD, 0x28, 0xC1, 0x40};
unsigned char db_dest[ETH_ALEN] = {0x01, 0x00, 0x5e, 0x40, 0x50, 0x61};
#endif


static netdev_tx_t fp_net_xmit(struct sk_buff *skb, struct net_device *netdev)
{
	struct fp_private *fp = netdev_priv(netdev);
	struct sk_buff *skb_p = NULL;

	struct ethhdr *eth_hdr = (struct ethhdr *)skb->data;

	if (!test_bit(HIFVMAC_RUNING, &fp->state)) {
		return NETDEV_TX_BUSY;
	}

//	pr_err("dst mac = %2x:%2x:%2x:%2x:%2x:%2x.\n", eth_hdr->h_dest[0], eth_hdr->h_dest[1],eth_hdr->h_dest[2], eth_hdr->h_dest[3], eth_hdr->h_dest[4], eth_hdr->h_dest[5]);


	if (unlikely(is_multicast_ether_addr(eth_hdr->h_dest))) {

		skb_p = pskb_copy(skb, GFP_ATOMIC);
		if (!skb_p) {
			pr_err("Copy skb failed for broadcast.\n");
			return NETDEV_TX_BUSY;
		}
		eth_hdr = (struct ethhdr *)skb_p->data;

#if defined(CONFIG_BST_C1200_DB)
		memcpy(eth_hdr->h_dest, ivi_dest, ETH_ALEN);
#elif defined(CONFIG_BST_C1200_IVI)
		memcpy(eth_hdr->h_dest, db_dest, ETH_ALEN);
#endif

		fp_xmit_frame_ring_broadcast(fp, skb_p, fp_tx_queue_mapping(fp, skb_p));
	}


	return fp_xmit_frame_ring(fp, skb, fp_tx_queue_mapping(fp, skb));
}


struct sk_buff *fp_init_frame(struct fp_private *priv)
{
	struct sk_buff *skb = NULL;
	struct net_device *dev;
	struct ethhdr *ethdr;
	int length;

	char smac[6] = {0x10, 0x21, 0x32, 0x43, 0x54, 0x65};

	//ip tcp
	char data[] = {
		0x45, 0x00, 0x00, 0x28, 0x84, 0xf0, 0x40, 0x00,
		0x32, 0x06, 0x6e, 0x5e, 0x11, 0x11, 0x11, 0x11,
		0x22, 0x22, 0x22, 0x22, 0x11, 0x11, 0x22, 0x22,
		0xbd, 0x01, 0xb7, 0x36, 0x93, 0x6e, 0x13, 0x24,
		0x50, 0x10, 0x09, 0x46, 0x35, 0x24, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};

	dev = priv->dev;
	length = sizeof(struct ethhdr) + sizeof(data) + 10;

	if (!(skb = dev_alloc_skb(length))) {
		pr_err("dev_alloc_skb malloc skb error\n");
		return NULL;
	}

	skb_reserve(skb,length);
	skb->len =  0;

	//fill payload
	skb_push(skb, sizeof(data));
	memcpy(skb->data, data, sizeof(data));
	skb->len = sizeof(data);

	//fille eth header
	skb_push(skb, sizeof(struct ethhdr));
	ethdr = (struct ethhdr *)skb->data;
	skb->len += sizeof(struct ethhdr);

	memcpy(ethdr->h_source, smac, ETH_ALEN);
	memcpy(ethdr->h_dest, priv->dev->dev_addr, ETH_ALEN);

	ethdr->h_proto = htons(ETH_P_IP);
	skb->protocol = htons(ETH_P_IP);

	//fill skb
	skb->pkt_type =  PACKET_OTHERHOST;
	skb->dev = dev;

	return skb;
}

void fp_prefetch_fifo_clear(struct fp_private *fp)
{
	unsigned int i;
	struct sk_buff *frame[MAX_SEND_FRAME_NUM] = {NULL};

	for(i = 0; i < MAX_SEND_FRAME_NUM; i++) {
		frame[i] = fp_init_frame(fp);
		if (frame[i]) {
			fp_net_xmit(frame[i], fp->dev);
		}
	}
}

bool fp_hif_sw_reset(struct fp_private *fp)
{
	int ret = 0, cnt = 0;
	unsigned int wr_bd_val, rx_fifo_val;
	bool tx_cleared = false, rx_cleared = false;

	set_bit(HIFVMAC_RX_FIFO_CLEAR, &fp->state);

rx_fifo_clear_retry:
	fp_prefetch_fifo_clear(fp);
	cnt++;
	msleep(500);
	if (test_bit(HIFVMAC_RX_FIFO_CLEARED, &fp->state)) {
		clear_bit(HIFVMAC_RX_FIFO_CLEARED, &fp->state);
		wr_bd_val = CSR_REG_READ(fp_baseAddr, (ULONG)&fp->hif[0].
						ch_r[HIF_CHAN_START]->hif_rx_wr_curr_bd_low_addr_ch);
		rx_fifo_val = CSR_REG_READ(fp_baseAddr, (ULONG)&fp->hif[0].
						ch_r[HIF_CHAN_START]->hif_bdp_ch_rx_fifo_cnt);
		if ((wr_bd_val == (u32)fp->hif[0].ch[HIF_CHAN_START].
						rxring->wb_bd_tbl_pa) && (rx_fifo_val == 0)) {
			rx_cleared = true;
			printk("%s: rx prefetch fifo clear success\n", __func__);
		}
	}

	if (!rx_cleared && (cnt < 4)) {
		goto rx_fifo_clear_retry;
	} else {
		cnt = 0;
	}

	clear_bit(HIFVMAC_RX_FIFO_CLEAR, &fp->state);
	set_bit(HIFVMAC_TX_FIFO_CLEAR, &fp->state);

tx_fifo_clear_retry:
	fp_prefetch_fifo_clear(fp);
	cnt++;
	msleep(500);
	if (test_bit(HIFVMAC_TX_FIFO_CLEARED, &fp->state)) {
		clear_bit(HIFVMAC_TX_FIFO_CLEARED, &fp->state);
		wr_bd_val = CSR_REG_READ(fp_baseAddr, (ULONG)&fp->hif[0].
						ch_r[HIF_CHAN_START]->hif_tx_wr_curr_bd_low_addr_ch);
		if (wr_bd_val == (u32)fp->hif[0].ch[HIF_CHAN_START].txring->wb_bd_tbl_pa) {
			tx_cleared = true;
			printk("%s: tx prefetch fifo clear success\n", __func__);
		}
	}

	if (!tx_cleared && (cnt < 4)) {
		goto tx_fifo_clear_retry;
	}

	if (tx_cleared && rx_cleared) {
		ret = true;
	}

	clear_bit(HIFVMAC_TX_FIFO_CLEAR, &fp->state);
	return ret;
}

int fp_add_prepend_tx_header(struct sk_buff *skb, unsigned int port)
{
	struct tx_header *txhdr;

	if (skb_headroom(skb) < FP_TX_HDR_LEN) {
		skb = skb_realloc_headroom(skb, HIF_HEADROOM);
		if (skb_headroom(skb) < FP_TX_HDR_LEN) {
			pr_err("%s: Not enough headroom for TX HDR, skb headroom = %d\n", __func__, skb_headroom(skb));
			return -1;
		}
	}

	skb_push(skb, FP_TX_HDR_LEN);
	txhdr = (struct tx_header *)skb->data;
	memset(txhdr, 0, FP_TX_HDR_LEN);

	if (0 != port) {
//		pr_debug("port: %d\r\n",port);
		//txhdr->ctrl = FP_TX_PKT_INJECT_EN;
		txhdr->txport_map = 1 << port;
		txhdr->txport_map |= 1 << 6;
		txhdr->txport_map |= 1 << 0;
		txhdr->txport_map |= 1 << 1;
		txhdr->txport_map |= 1 << 7;
		txhdr->txport_map |= 1 << 8;
		txhdr->txport_map |= 1 << 9;
		txhdr->seq_num = 0xaad8;
	}

	return 0;
}

int fp_add_prepend_tx_header_broadcast(struct sk_buff *skb, unsigned int port)
{
	struct tx_header *txhdr;

	if (skb_headroom(skb) < FP_TX_HDR_LEN) {
		skb = skb_realloc_headroom(skb, HIF_HEADROOM);
		if (skb_headroom(skb) < FP_TX_HDR_LEN) {
			pr_err("%s: Not enough headroom for TX HDR, skb headroom = %d\n", __func__, skb_headroom(skb));
			return -1;
		}
	}

	skb_push(skb, FP_TX_HDR_LEN);
	txhdr = (struct tx_header *)skb->data;
	memset(txhdr, 0, FP_TX_HDR_LEN);

#if 0
	if (0 != port) {
//		pr_debug("port: %d\r\n",port);
		txhdr->ctrl = FP_TX_PKT_INJECT_EN;
		txhdr->txport_map = 1 << port;
		txhdr->rx_ch_no = 17;
		txhdr->seq_num = 0xaad8;
	}
#endif

	return 0;
}
netdev_tx_t fp_xmit_frame_ring_broadcast(struct fp_private *fp, struct sk_buff *skb,
				struct bd_tx_ring *tx_ring)
{
	unsigned int  size;
	int tx_ctrl, ret = 0;
	u16 count;

//	pr_err("before packet size = %d.\n", skb_headlen(skb));

	if (fp_add_prepend_tx_header_broadcast(skb, 10) < 0 ) {
//		return NETDEV_TX_BUSY;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	count = TX_BD_USE_COUNT(skb_headlen(skb));

//			pr_err("tx count = %d.\n",count);
//			pr_err("nr_frags = %d.\n",skb_shinfo(skb)->nr_frags);
//
//
#if 0
	for (f = 0; f < skb_shinfo(skb)->nr_frags; f++) {
		count += TX_BD_USE_COUNT(skb_frag_size(
						&skb_shinfo(skb)->frags[f]));
		pr_err("tx faliled.\n");
	}
#endif


//	pr_err("tx count = %d.\n",count);
//	spin_lock(&fp->lock);
	if (fp_tx_bd_unused(tx_ring) < count) {
		if (!netif_tx_queue_stopped(netdev_get_tx_queue(fp->dev, skb_get_queue_mapping(skb)))) {
			netif_tx_stop_queue(netdev_get_tx_queue(fp->dev,
								skb_get_queue_mapping(skb)));
			tx_ring->queue_index = skb_get_queue_mapping(skb);
			/* This is a hard error, log it. */
			pr_err("%s: Tx Ring full when queue awake\n", __func__);
		}
		return NETDEV_TX_BUSY;
	}

	ret = fp_add_tx_pkt(fp, tx_ring, skb);
	if (ret) {
//		spin_unlock(&fp->lock);
		pr_err("add skb head to tx buffer error.\n");
		return ret;
	}


	netdev_tx_sent_queue(txring_txq(fp, tx_ring), size);
	/*-----------------------------------------------------------------------------
	 *  ensure all the bd are prepared before start transmit.
	 *-----------------------------------------------------------------------------*/
	dma_wmb();
  

//			pr_err("tx_ring->ring_index = %d .\n", tx_ring->ring_index);
	tx_ctrl = CSR_REG_READ(fp_baseAddr,
			(ULONG)&fp->hif[0].ch_r[tx_ring->ring_index]->hif_ctrl_ch);
	tx_ctrl |= HIF_CH_TX_BDP_POLL_CNTR_EN;
	tx_ctrl |= HIF_CH_TX_CTRL_DMA_EN;
	CSR_REG_WRITE(fp_baseAddr,
			(ULONG)&fp->hif[0].ch_r[tx_ring->ring_index]->hif_ctrl_ch, tx_ctrl);

	tx_ctrl = HIF_CH_TX_START;
	CSR_REG_WRITE(fp_baseAddr,
			(ULONG)&fp->hif[0].ch_r[tx_ring->ring_index]->hif_tx_ch_start, tx_ctrl);

#if 0
	fp->netstats.tx_packets++;
	fp->netstats.tx_bytes += (size - 16);
	tx_ring->stats.packets++;
	tx_ring->stats.bytes += (size -16);
#endif

//	spin_unlock(&fp->lock);
	return NETDEV_TX_OK;

}


netdev_tx_t fp_xmit_frame_ring(struct fp_private *fp, struct sk_buff *skb,
				struct bd_tx_ring *tx_ring)
{
	unsigned int  size;
	int tx_ctrl, ret = 0;
	u16 count;

//	pr_err("before packet size = %d.\n", skb_headlen(skb));

	if (fp_add_prepend_tx_header(skb, 5) < 0 ) {
//		return NETDEV_TX_BUSY;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	count = TX_BD_USE_COUNT(skb_headlen(skb));

//			pr_err("tx count = %d.\n",count);
//			pr_err("nr_frags = %d.\n",skb_shinfo(skb)->nr_frags);
//
//
#if 0
	for (f = 0; f < skb_shinfo(skb)->nr_frags; f++) {
		count += TX_BD_USE_COUNT(skb_frag_size(
						&skb_shinfo(skb)->frags[f]));
		pr_err("tx faliled.\n");
	}
#endif


//	pr_err("tx count = %d.\n",count);
//	spin_lock(&fp->lock);
	if (fp_tx_bd_unused(tx_ring) < count) {
		if (!netif_tx_queue_stopped(netdev_get_tx_queue(fp->dev, skb_get_queue_mapping(skb)))) {
			netif_tx_stop_queue(netdev_get_tx_queue(fp->dev,
								skb_get_queue_mapping(skb)));
			tx_ring->queue_index = skb_get_queue_mapping(skb);
			/* This is a hard error, log it. */
			pr_err("%s: Tx Ring full when queue awake\n", __func__);
		}
		return NETDEV_TX_BUSY;
	}


	ret = fp_add_tx_pkt(fp, tx_ring, skb);
	if (ret) {
//		spin_unlock(&fp->lock);
		pr_err("add skb head to tx buffer error, ret = %d.\n", ret);
		return ret;
	}


	netdev_tx_sent_queue(txring_txq(fp, tx_ring), size);
	/*-----------------------------------------------------------------------------
	 *  ensure all the bd are prepared before start transmit.
	 *-----------------------------------------------------------------------------*/
	dma_wmb();
  

//			pr_err("tx_ring->ring_index = %d .\n", tx_ring->ring_index);
	tx_ctrl = CSR_REG_READ(fp_baseAddr,
			(ULONG)&fp->hif[0].ch_r[tx_ring->ring_index]->hif_ctrl_ch);
	tx_ctrl |= HIF_CH_TX_BDP_POLL_CNTR_EN;
	tx_ctrl |= HIF_CH_TX_CTRL_DMA_EN;
	CSR_REG_WRITE(fp_baseAddr,
			(ULONG)&fp->hif[0].ch_r[tx_ring->ring_index]->hif_ctrl_ch, tx_ctrl);

	tx_ctrl = HIF_CH_TX_START;
	CSR_REG_WRITE(fp_baseAddr,
			(ULONG)&fp->hif[0].ch_r[tx_ring->ring_index]->hif_tx_ch_start, tx_ctrl);

#if 0
	fp->netstats.tx_packets++;
	fp->netstats.tx_bytes += (size - 16);
	tx_ring->stats.packets++;
	tx_ring->stats.bytes += (size -16);
#endif

//	spin_unlock(&fp->lock);
	return NETDEV_TX_OK;

}

void fp_reset_tx_rx_bd_ring(struct fp_private *fp)
{
	CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[0].ch_r[HIF_CHAN_START]->hif_rx_bdp_wr_low_addr_ch, 0x0);
	CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[0].ch_r[HIF_CHAN_START]->hif_rx_bdp_wr_high_addr_ch, 0x0);
	CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[0].ch_r[HIF_CHAN_START]->hif_rx_bdp_rd_low_addr_ch, 0x0);
	CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[0].ch_r[HIF_CHAN_START]->hif_rx_bdp_rd_high_addr_ch, 0x0);

	CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[0].ch_r[HIF_CHAN_START]->hif_tx_bdp_wr_low_addr_ch, 0x0);
	CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[0].ch_r[HIF_CHAN_START]->hif_tx_bdp_wr_high_addr_ch, 0x0);
	CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[0].ch_r[HIF_CHAN_START]->hif_tx_bdp_rd_low_addr_ch, 0x0);
	CSR_REG_WRITE(fp_baseAddr, (ULONG)&fp->hif[0].ch_r[HIF_CHAN_START]->hif_tx_bdp_rd_high_addr_ch, 0x0);
}

/**********************************************************************
 * Function Name  : fp_net_release
 * Description    : unregister the logical fp_phy interfaces and free the
 *                  ring buffer descriptors
 * Inputs
 *   Parameters   : struct net_device *
 * Outputs        :
 *   Parameters   : -
 *   Returns      : -
 * Changes        :
 *********************************************************************/
int fp_net_release(struct net_device *dev)
{
	//int ret;
	struct fp_private *fp;

	printk("%s start\n", __func__);
	fp = netdev_priv(dev);

	if (!fp) {
		return -1;
	}

	unregister_netdev(dev);

	hif_netif_del_napi(fp);

#if 0
#if defined(CONFIG_BST_C1200_DB)
	ret = fp_send_message(fp->dev, DB_HIF_REINIT_FLAG);
#elif defined(CONFIG_BST_C1200_IVI)
	ret = fp_send_message(fp->dev, IVI_HIF_REINIT_FLAG);
#endif
	if (ret) {
		printk("%s vmac reinit msg send fail, errno=%d\n", __func__, ret);
	}
#endif

	free_netdev(dev);

	fp_reset_tx_rx_bd_ring(fp);

	printk("%s end\n", __func__);

	return 0;
}

int fp_net_close(struct net_device *dev)
{
	int ret;
	bool reinit = false;
	struct fp_private *fp;

	printk("%s start\n", __func__);
	fp = netdev_priv(dev);

	if (!fp) {
		return -1;
	}

	if (!test_bit(HIFVMAC_NOE_EXEC_SW_RESET, &fp->state)) {
		ret = fp_hif_sw_reset(fp);
		if (!ret) {
			reinit = true;
			pr_err("%s: hif prefetch fifo clear fail\n", __func__);
		} else {
			pr_err("%s: hif prefetch fifo clear success\n", __func__);
		}
	}

	set_bit(HIFVMAC_DOWN, &fp->state);
	netif_tx_stop_all_queues(dev);

	/*  Disable hif tx/rx interrupt */
	fp_disable_interrupts(fp);

	/*  Disable hif tx dma engine. */
	fp_disable_hif_dma_engine(fp);

	//mdelay(100);

	hif_netif_disable_napi(fp);
	fp_free_irq(fp);

	/* Cleanup the bd rings */
	fp_bd_uninit_all_rx_rings(fp);
	//fp_bd_free_all_rx_rings(fp);
	//fp_bd_free_all_tx_rings(fp);
	fp_bd_destroy_ring(fp);
	fp->tx_fifo_clear = false;
	fp->rx_fifo_clear = false;
	fp->state = 0;
	g_fp_initial = 0;
	printk("%s end\n", __func__);

	return 0;
}

static int fp_set_mac_address(struct net_device *ndev, void *addr)
{
//	struct fp_private *fp = netdev_priv(ndev);
	unsigned char mac_data[6];
#if defined(CONFIG_BST_C1200_DB)
	db_switch_MyArray_t mac; 
	db_switch_ErrorEnum_t err = 0;
#elif defined(CONFIG_BST_C1200_IVI)
	ivi_switch_MyArray_t mac; 
	ivi_switch_ErrorEnum_t err = 0;
#endif

	int ret = 0;

	ret = eth_mac_addr(ndev, addr);
	if (ret)
		return ret;

	memcpy(mac_data, ndev->dev_addr, ndev->addr_len);
	mac.data = &mac_data[0];
	mac.size = ndev->addr_len;
	if (client) {
#if defined(CONFIG_BST_C1200_DB)
		client->db_switch_client.vmac_method_sync(mac, 1,  &err, 1000, NULL);
#elif defined(CONFIG_BST_C1200_IVI)
		client->ivi_switch_client.vmac_method_sync(mac, 2,  &err, 1000, NULL);
#endif

	}

	return ret;
}

#if 0
static void hif_reset_subtask(struct fp_private *priv)
{
	if (!test_and_clear_bit(HIFVMAC_RESET_REQUESTED, &priv->state))
		return;
	if (test_bit(HIFVMAC_DOWN, &priv->state))
		return;

	netdev_err(priv->dev, "Reset hif fp private.\n");
	rtnl_lock();
	netif_trans_update(priv->dev);
	while (test_and_set_bit(HIFVMAC_RESETTING, &priv->state))
		usleep_range(1000, 2000);

	set_bit(HIFVMAC_DOWN, &priv->state);
	dev_close(priv->dev);
	dev_open(priv->dev, NULL);
	clear_bit(HIFVMAC_DOWN, &priv->state);
	clear_bit(HIFVMAC_RESETTING, &priv->state);
	rtnl_unlock();
}

static void hif_service_task(struct work_struct *work)
{
	struct fp_private *priv = container_of(work, struct fp_private,
						 service_task);

	hif_reset_subtask(priv);
	clear_bit(HIFVMAC_SERVICE_SCHED, &priv->state);
}

static void fp_service_event_schedule(struct fp_private *priv)
{
	if (!test_bit(HIFVMAC_DOWN, &priv->state) &&
	    !test_and_set_bit(HIFVMAC_SERVICE_SCHED, &priv->state))
		queue_work(priv->wdt_wq, &priv->service_task);
}

static void fp_global_err(struct fp_private *priv)
{
	netif_carrier_off(priv->dev);
	set_bit(HIFVMAC_RESET_REQUESTED, &priv->state);
	fp_service_event_schedule(priv);
}

static void fp_tx_timeout(struct net_device *dev, unsigned int txqueue)
{
	struct fp_private *fp = netdev_priv(dev);

	fp_global_err(fp);
}
#endif

const struct net_device_ops fp_netdev_ops = {
	.ndo_start_xmit		= fp_net_xmit,
	.ndo_open		= fp_net_open,
	.ndo_stop       = fp_net_close,
#if 0
	.ndo_select_queue	= fp_select_queue,
	.ndo_do_ioctl		= fp_net_ioctl,
	.ndo_get_stats		= fp_get_stats,
	.ndo_change_mtu		= fp_change_mtu
	.ndo_tx_timeout = fp_tx_timeout,
#endif
	.ndo_set_mac_address = fp_set_mac_address,
};

struct ethtool_ops fp_ethtool_ops = {
	.begin			= NULL,
	.complete		= NULL, /* fp_ethtool_complete */
	//.get_settings		= NULL, /* fp_get_settings */
	//.set_settings		= NULL /* fp_set_settings */
};

static void vmac_reinit_triggered(const uint32_t channel, void *ext, const ext_info_t *info)
{
	struct fp_private *priv;

	priv = (struct fp_private *)ext;
	pr_err("%s: receive vmac reinit broadcast, channel is %d\n", __func__, channel);

	rtnl_lock();
	set_bit(HIFVMAC_DOWN, &priv->state);
	dev_close(priv->dev);
	dev_open(priv->dev, NULL);
	clear_bit(HIFVMAC_DOWN, &priv->state);
	rtnl_unlock();

	pr_err("%s: vmac reinit down\n", __func__);
}

static void vmac_reinit_sub_reply(int err, void *ext, const ext_info_t *info)
{
	if (err == 0)
		pr_err("%s: subscribe vmac reinit success\n", __func__);
	else
		pr_err("%s: ubscribe vmac reinit success, ret is %d\n", __func__, err);
}

/**
 * fp_netdev_init
 * @device: device pointer
 * @plat_dat: platform data pointer
 * @res: bsthif resource pointer
 * Description: this is the main probe function used to
 * call the alloc_etherdev, allocate the priv structure.
 * Return:
 * returns 0 on success, otherwise errno.
 */
struct net_device * fp_netdev_init(struct platform_device *pdev,
		struct plat_hif_data *plat_dat,
		struct hif_resources *res)
{
	struct net_device *ndev = NULL;
	struct fp_private *fppriv;
	u32 index;
	int ret = 0;
	struct hif_channel *ch = NULL;
	struct device *device;
//	DECLARE_BITMAP(_channel_mask, 64);

#if defined(CONFIG_BST_C1200_DB)
	client = db_client_init(&client_data);
	if (!client) {
		pr_err("db 2 switch init client fail.\n");
	} else {
		pr_err("db to switch init client success.\n");
	}
#elif defined(CONFIG_BST_C1200_IVI)
	client = ivi_client_init(&client_data);
	if (!client) {
		pr_err("ivi 2 switch init client fail.\n");
	} else {
		pr_err("ivi to switch init client success.\n");
	}
#endif

	ret = client->start();
	if (ret < 0) {
		pr_err("start test client failed!\n");
	}

	ndev = alloc_etherdev_mqs(sizeof(struct fp_private),
				  HIF_MAX_TX_QUEUES, HIF_MAX_RX_QUEUES);
	if (!ndev)
		goto err_alloc_netdev;

	device = &pdev->dev;
	SET_NETDEV_DEV(ndev, device);
	ndev->flags &= ~IFF_UP;
	ndev->flags &= ~IFF_MULTICAST;
	ndev->netdev_ops   = &fp_netdev_ops;
	ndev->ethtool_ops = &fp_ethtool_ops;
	ndev->rtnl_link_ops = NULL;
	ndev->min_mtu = ETH_ZLEN - ETH_HLEN;
	ndev->max_mtu = MAX_HIF_JUMBO_FRAME_SIZE - (ETH_HLEN + ETH_FCS_LEN);
//	ndev->mtu = 9212;
	ndev->mtu = 1500;
	ndev->needed_headroom = HIF_HEADROOM;
	ndev->watchdog_timeo = msecs_to_jiffies(watchdog);

	if (!IS_ERR_OR_NULL(res->mac)){

//		memcpy(ndev->dev_addr, res->mac, ndev->addr_len);
		eth_hw_addr_random(ndev);

		if (!is_valid_ether_addr(ndev->dev_addr)) {
			pr_err("failed to init i2c interface\n");
			goto err_dev_addr;
		}
	}

	ndev->base_addr = (unsigned long)res->addr;

	ret = of_reserved_mem_device_init(&pdev->dev);
	if (ret) {
		pr_err("%s: could not get hif reserved memory\n", __func__);
		goto err_dev_addr;
	}

	dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(36));
//	pr_err("dma_set_mask_coherent = %d.\n", err);

	fppriv = (struct fp_private *)netdev_priv(ndev);
	fppriv->dev = ndev;
	fppriv->device = &pdev->dev;
	fp_baseAddr = fppriv->mem_base = (unsigned char *)res->addr;
//	pr_err("init fp_baseAddr = %px.\n", fp_baseAddr);
	fppriv->plat = plat_dat;

	fp_assign_hif_base_addr(fppriv);
	dev_set_drvdata(device, fppriv->dev);
//	pr_err("max index  = %d.\n", BITS_TO_LONGS(NUM_MAX_HIF_CHANNELS));
//	pr_err("max index  = %d.\n", BITS_TO_LONGS(HIF_CHANNELS_PER_IRQ));
//	pr_err("plat hif_channel_bit  = %px.\n", plat_dat->hif_channel_bit);
//	pr_err("before _channel_mask = %px.\n", fppriv->_channel_mask[0]);
	bitmap_zero(fppriv->_channel_mask, NUM_MAX_HIF_CHANNELS);
	bitmap_from_u64(fppriv->_channel_mask, plat_dat->hif_channel_bit);
	fppriv->q_num = hweight64(plat_dat->hif_channel_bit);
//	pr_err("q-num = %d.\n", fppriv->q_num);
//	pr_err("_channel_mask = %px.\n", fppriv->_channel_mask[0]);
	for (index = 0; index < HIF_MAX_IRQ; index++) {

		ch = &fppriv->channel[index];
		ch->index = index;
		ch->tx_clean_limit = 64;
		ch->irq = res->irq[index];
		ch->fp = fppriv;
//		pr_err("before ch->mask = %px.\n", ch->_mask[0]);
		bitmap_zero(ch->_mask, HIF_CHANNELS_PER_IRQ);
		bitmap_set_value8(ch->_mask, *((u8 *)&plat_dat->hif_channel_bit + index), 0); 
//		pr_err("((u8 *)&plat_dat->hif_channel_bit + index) = %px.\n", *((u8 *)&plat_dat->hif_channel_bit + index) );
//		pr_err("ch->mask = %px.\n", ch->_mask[0]);
		ch->start = find_first_bit(ch->_mask, HIF_CHANNELS_PER_IRQ); 
		ch->end = find_last_bit(ch->_mask, HIF_CHANNELS_PER_IRQ);
//		pr_err("index  = %d. start = %d, end = %d.\n", index, ch->start, ch->end);

		if ( ch->end == 8 && ch->start == ch->end ) {
			ch->irq = 0;
		}

		netif_napi_add(ndev, &ch->napi, hif_napi_poll);
	}

	strncpy(ndev->name, "eth%d", sizeof(ndev->name) - 1);

	ret = register_netdev(ndev);
	if (ret) {
		goto err_register;
	}

	// subscribe broadcast
	if (client) {
#if defined(CONFIG_BST_C1200_DB)
		ret = client->db_switch_client.notify_vmac_reinit_sub(vmac_reinit_triggered, (void *)fppriv, NULL, vmac_reinit_sub_reply, NULL);
#elif defined(CONFIG_BST_C1200_IVI)
		ret = client->ivi_switch_client.notify_vmac_reinit_sub(vmac_reinit_triggered, (void *)fppriv, NULL, vmac_reinit_sub_reply, NULL);
#endif
		if (ret < 0) {
			pr_err("vmac send subscribe msg fail, ret = %d\n", ret);
		}
	}

#if 0
	fppriv->wdt_wq = create_workqueue("hif_wdt");
	if (!fppriv->wdt_wq) {
		dev_err(fppriv->device, "failed to create hif wdt workqueue\n");
		ret = -ENOMEM;
		goto err_dev_addr;
	}

	INIT_WORK(&fppriv->service_task, hif_service_task);
#endif

	return ndev;

err_register:
	hif_netif_del_napi(fppriv);
err_dev_addr:
	free_netdev(ndev);
err_alloc_netdev:
	return NULL;

}


static void hif_netif_del_napi(struct fp_private *priv)
{
	u32 queue;

	for (queue = 0; queue < HIF_MAX_IRQ; queue++) {
		struct hif_channel *ch = &priv->channel[queue];

		netif_napi_del(&ch->napi);
	}
}

static void hif_netif_disable_napi(struct fp_private *priv)
{
	u32 queue;

	for (queue = 0; queue < HIF_MAX_IRQ; queue++) {
		struct hif_channel *ch = &priv->channel[queue];

		napi_disable(&ch->napi);
	}
}

/**
 * hif_dvr_remove
 * @dev: device pointer
 * Description: this function resets the TX/RX processes, disables the MAC RX/TX
 * changes the link status, releases the DMA descriptor rings.
 */
int hif_dvr_remove(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	//struct fp_private *priv = netdev_priv(ndev);
	uint32_t ret;

	fp_net_release(ndev);

	//destroy_workqueue(priv->wdt_wq);

	if (client) {
		ret = client->stop();
		if (ret < 0) {
#if defined(CONFIG_BST_C1200_DB)
			pr_err("stop db to switch client thread failed!\n");
#elif defined(CONFIG_BST_C1200_IVI)
			pr_err("stop ivi to switch client thread failed!\n");
#endif
			return -1;
		}

#if defined(CONFIG_BST_C1200_DB)
		ret = db_client_destroy();
		if (ret < 0) {
			pr_err("destroy db to switch client fail.\n");
			return -2;
		}
#elif defined(CONFIG_BST_C1200_IVI)
		ret = ivi_client_destroy();
		if (ret < 0) {
			pr_err("destroy ivi to switch client fail.\n");
			return -2;
		}
#endif
	}

	return 0;
}


