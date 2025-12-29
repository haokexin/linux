// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021-2024 Black Sesame Technologies. All Rights Reserved.
 * Copyright (C) 2007-2011 STMicroelectronics Ltd
 */

#include "bstvmac.h"
#include "bstvmac_hif.h"
#include <linux/of_reserved_mem.h>
#include <linux/dma-mapping.h>
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif /* CONFIG_DEBUG_FS */
#if defined(CONFIG_BST_C1200_DB)
#include "db_client.h"
#elif defined(CONFIG_BST_C1200_IVI)
#include "ivi_client.h"
#endif

#define	BSTVMAC_ALIGN(x)		__ALIGN_KERNEL(x, SMP_CACHE_BYTES)
#define	TSO_MAX_BUFF_SIZE		(SZ_16K - 1)
#define BSTMAC_PTP_KFIFO_NUM	128
/* Module parameters */
#define TX_TIMEO				5000
static int watchdog = TX_TIMEO;
module_param(watchdog, int, 0644);
MODULE_PARM_DESC(watchdog, "Transmit timeout in milliseconds (default 5s)");

static int debug = -1;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Message Level (-1: default, 0: no output, 16: all)");

#define BSTVMAC_TX_THRESH		(DMA_TX_SIZE / 4)

static int pause = PAUSE_TIME;
module_param(pause, int, 0644);
MODULE_PARM_DESC(pause, "Flow Control Pause Time");

#define	DEFAULT_BUFSIZE	1536
static int buf_sz = DEFAULT_BUFSIZE;
module_param(buf_sz, int, 0644);
MODULE_PARM_DESC(buf_sz, "DMA buffer size");

#define SIOCGMCFILTER	(SIOCDEVPRIVATE)

static const u32 default_msg_level = (NETIF_MSG_DRV | NETIF_MSG_PROBE |
				      NETIF_MSG_LINK | NETIF_MSG_IFUP |
				      NETIF_MSG_IFDOWN | NETIF_MSG_TIMER);

#define BSTVMAC_TRANS_TIMEOUT_MS	3000

/* By default the driver will use the ring mode to manage tx and rx descriptors,
 * but allow user to force to use the chain instead of the ring
 */
static unsigned int chain_mode;
module_param(chain_mode, int, 0444);
MODULE_PARM_DESC(chain_mode, "To use chain instead of ring mode");

static u32 perf_debug;
module_param(perf_debug, uint, 0644);
MODULE_PARM_DESC(perf_debug, "udp performance test");

spinlock_t vmac_irqbits_lock;

static struct sk_buff_head *vmac_delivery_skblist[BSTVMAC_CORE_NUM *
						 BSTVMAC_RXCHAN_NUM];
struct bstvmac_priv *vmac_priv_g[BSTVMAC_CORE_NUM] = { 0 };

static irqreturn_t bstvmac_interrupt(int irq, void *dev_id);
static void bstvmac_tx_timer_arm(struct bstvmac_priv *priv, u32 queue);
static void bstvmac_flush_tx_descriptors(struct bstvmac_priv *priv, int queue);
static netdev_tx_t bstvmac_xmit(struct sk_buff *skb, struct net_device *dev);

#ifdef CONFIG_DEBUG_FS
static int bstvmac_init_fs(struct net_device *dev);
static void bstvmac_exit_fs(struct net_device *dev);
#endif

#define BSTVMAC_COAL_TIMER(x) (ns_to_ktime((x) * NSEC_PER_USEC))
#define BSTVMAC_RX_NAPI 1
#define BSTVMAC_ADJUST_LINK 0
#define BSTVMAC_JUMBO_TXFRAMES 4
#define BSTMAC_TXWQ_DEF_CPU	0

#if defined(CONFIG_BST_C1200_DB)
unsigned char db_dest[ETH_ALEN] = {0x01, 0x00, 0x5E, 0x40, 0x50, 0x60};
#elif defined(CONFIG_BST_C1200_IVI)
unsigned char ivi_dest[ETH_ALEN] = {0x01, 0x00, 0x5e, 0x40, 0x50, 0x61};
#endif

#if defined(CONFIG_BST_C1200_DB)
db_client_t *client = NULL;
db_client_data_t client_data = {0};
#elif defined(CONFIG_BST_C1200_IVI)
ivi_client_t *client = NULL;
ivi_client_data_t client_data = {0};
#endif

/**
 * bstvmac_verify_args - verify the driver parameters.
 * Description: it checks the driver parameters and set a default in case of
 * errors.
 */
static void bstvmac_verify_args(void)
{
	if (unlikely(watchdog < 0))
		watchdog = TX_TIMEO;
	if (unlikely(buf_sz < DEFAULT_BUFSIZE || buf_sz > BUF_SIZE_16KiB))
		buf_sz = DEFAULT_BUFSIZE;
	if (unlikely(pause < 0 || pause > 0xffff))
		pause = PAUSE_TIME;
}

/**
 * bstvmac_disable_all_queues - Disable all queues
 * @priv: driver private structure
 */
static void bstvmac_disable_all_queues(struct bstvmac_priv *priv)
{
	u32 rx_queues_cnt = priv->plat->rx_queues_to_use;
	u32 queue;

	for (queue = 0; queue < rx_queues_cnt; queue++) {
		struct bstvmac_channel *ch = &priv->rx_channel[queue];
#if BSTVMAC_RX_NAPI
		if (queue < rx_queues_cnt)
			napi_disable(&ch->rnapi);
#endif
	}
}

/**
 * bstvmac_enable_all_queues - Enable all queues
 * @priv: driver private structure
 */
static void bstvmac_enable_all_queues(struct bstvmac_priv *priv)
{
	u32 rx_queues_cnt = priv->plat->rx_queues_to_use;
	u32 queue;

	for (queue = 0; queue < rx_queues_cnt; queue++) {
		struct bstvmac_channel *ch = &priv->rx_channel[queue];
#if BSTVMAC_RX_NAPI
		napi_enable(&ch->rnapi);
#endif
	}
}
#if 0
static void bstvmac_service_event_schedule(struct bstvmac_priv *priv)
{
	if (!test_bit(BSTVMAC_DOWN, &priv->state) &&
	    !test_and_set_bit(BSTVMAC_SERVICE_SCHED, &priv->state))
		queue_work(priv->wq, &priv->service_task);
}

static void bstvmac_global_err(struct bstvmac_priv *priv)
{
	netif_carrier_off(priv->dev);
	set_bit(BSTVMAC_RESET_REQUESTED, &priv->state);
	bstvmac_service_event_schedule(priv);
}
#endif
static void print_pkt(unsigned char *buf, int len)
{
	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_OFFSET, 16, 1, buf, len, true);
}

static inline u32 bstvmac_tx_avail(struct bstvmac_priv *priv, u32 queue)
{
	struct bstvmac_tx_queue *tx_q = &priv->tx_queue[queue];
	u32 avail;

	if (tx_q->dirty_tx > tx_q->cur_tx)
		avail = tx_q->dirty_tx - tx_q->cur_tx - 1;
	else
		avail = DMA_TX_SIZE - tx_q->cur_tx + tx_q->dirty_tx - 1;

	return avail;
}

/**
 * bstvmac_rx_dirty - Get RX queue dirty
 * @priv: driver private structure
 * @queue: RX queue index
 */
static inline u32 bstvmac_rx_dirty(struct bstvmac_priv *priv, u32 queue)
{
	struct bstvmac_rx_queue *rx_q = &priv->rx_queue[queue];
	u32 dirty;

	if (rx_q->dirty_rx <= rx_q->cur_rx)
		dirty = rx_q->cur_rx - rx_q->dirty_rx;
	else
		dirty = DMA_RX_SIZE - rx_q->dirty_rx + rx_q->cur_rx;

	return dirty;
}

int bstvmac_sendmsg_to_switch(struct net_device *ndev, unsigned char flag)
{
	int ret = 0;
	unsigned char mac_data[6];
#if defined(CONFIG_BST_C1200_DB)
	db_switch_MyArray_t mac;
	//db_switch_ErrorEnum_t msg_err = 0;
#elif defined(CONFIG_BST_C1200_IVI)
	ivi_switch_MyArray_t mac;
	//ivi_switch_ErrorEnum_t msg_err = 0;
#endif

	if (!ndev) {
		ret = -1;
		pr_err("%s net_device is null\n", __func__);
		goto end;
	}

	memcpy(mac_data, ndev->dev_addr, ndev->addr_len);
	mac.data = &mac_data[0];
	mac.size = ndev->addr_len;

	if (client) {
// #if defined(CONFIG_BST_C1200_DB)
// 		ret = client->db_switch_client.vmac_method_sync(mac, flag,  &msg_err, 1000, NULL);
// #elif defined(CONFIG_BST_C1200_IVI)
// 		ret = client->ivi_switch_client.vmac_method_sync(mac, flag,  &msg_err, 1000, NULL);
// #endif

#if defined(CONFIG_BST_C1200_DB)
		ret = client->db_switch_client.vmac_method_async(mac, flag,  NULL, NULL, NULL);
#elif defined(CONFIG_BST_C1200_IVI)
		ret = client->ivi_switch_client.vmac_method_async(mac, flag,  NULL, NULL, NULL);
#endif
	}

end:
	return ret;
}

static int bstvmac_set_bfsize(int mtu, int bufsize)
{
	int ret = bufsize;

	if (mtu >= BUF_SIZE_4KiB)
		ret = BUF_SIZE_9KiB;
	else if (mtu >= BUF_SIZE_2KiB)
		ret = BUF_SIZE_4KiB;
	else if (mtu > DEFAULT_BUFSIZE)
		ret = BUF_SIZE_2KiB;
	else
		ret = BUF_SIZE_4KiB;

	return ret;
}

static int bstvmac_set_16kib_bfsize(int mtu)
{
	int ret = 0;

	if (unlikely(mtu > BUF_SIZE_8KiB))
		ret = BUF_SIZE_16KiB;

	return ret;
}

/**
 * bstvmac_init_rx_descriptors - clear RX descriptors
 * @priv: driver private structure
 * @queue: RX queue index
 * Description: this function is called to clear the RX descriptors
 * in case of both basic and extended descriptors are used.
 */
static void bstvmac_init_rx_descriptors(struct bstvmac_priv *priv, u32 queue)
{
	struct bstvmac_rx_queue *rx_q = &priv->rx_queue[queue];
	int i;

	/* Clear the RX descriptors */
	for (i = 0; i < priv->dma_rx_size; i++) {
		bool last = (i == (priv->dma_rx_size - 1));
		bstvmac_init_bd_rx_desc(priv, &rx_q->dma_bd_rx[i], rx_q->seq_num, last,
						priv->dma_buf_sz);
		bstvmac_init_wrbd_desc(priv, &rx_q->dma_wrbd_rx[i], 0x0);
		rx_q->seq_num++;
	}
}

/**
 * bstvmac_init_tx_descriptors - clear tx descriptors
 * @priv: driver private structure
 * @queue: TX queue index.
 * Description: this function is called to clear the TX descriptors
 * in case of both basic and extended descriptors are used.
 */
static void bstvmac_init_tx_descriptors(struct bstvmac_priv *priv, u32 queue)
{
	struct bstvmac_tx_queue *tx_q = &priv->tx_queue[queue];
	int i;

	/* Clear the TX descriptors */
	for (i = 0; i < priv->dma_tx_size; i++) {
		int last = (i == (priv->dma_tx_size - 1));
		bstvmac_init_bd_tx_desc(priv,  &tx_q->dma_bd_tx[i], last);
		bstvmac_init_wrbd_desc(priv, &tx_q->dma_wrbd_tx[i], 0xff);
	}
}

/**
 * bstvmac_init_rx_buffers - init the RX descriptor buffer.
 * @priv: driver private structure
 * @p: descriptor pointer
 * @i: descriptor index
 * @flags: gfp flag
 * @queue: RX queue index
 * Description: this function is called to allocate a receive buffer, perform
 * the DMA mapping and init the descriptor.
 */

static int bstvmac_init_rx_buffers(struct bstvmac_priv *priv,
				   struct dma_bd_desc *p, int i, gfp_t flags,
				   u32 queue)
{
	struct bstvmac_rx_queue *rx_q = &priv->rx_queue[queue];
	struct sk_buff *skb;

	void *dma_area;
	dma_area = dma_alloc_coherent(priv->device, priv->dma_buf_sz, &rx_q->rx_skbuff_dma[i], GFP_KERNEL);
	if (!dma_area)
		return -ENOMEM;

	skb = netdev_alloc_skb(priv->dev, 0);
	if (!skb) {
		dma_free_coherent(priv->device, priv->dma_buf_sz, dma_area, rx_q->rx_skbuff_dma[i]);
		return -ENOMEM;
	}
    skb->data = dma_area;
    // skb->tail = (sk_buff_data_t)(uintptr_t)(dma_area + priv->dma_buf_sz);
    // skb->end = skb->tail;
	rx_q->rx_skbuff[i] = skb;

	bstvmac_set_desc_addr(priv, p, rx_q->rx_skbuff_dma[i]);
	return 0;
}
/**
 * bstvmac_free_rx_buffer - free RX dma buffers
 * @priv: private structure
 * @queue: RX queue index
 * @i: buffer index.
 */
static void bstvmac_free_rx_buffer(struct bstvmac_priv *priv, u32 queue, int i)
{
    struct bstvmac_rx_queue *rx_q = &priv->rx_queue[queue];

	if (rx_q->rx_skbuff[i]) {
		// dma_unmap_single(priv->device, rx_q->rx_skbuff_dma[i],
		// 		 priv->dma_buf_sz, DMA_FROM_DEVICE);
		// // dev_kfree_skb_any(rx_q->rx_skbuff[i]);
		// dma_free_coherent(priv->device,  priv->dma_buf_sz, rx_q->rx_skbuff[i], rx_q->rx_skbuff_phy[i]);
		if (rx_q->rx_skbuff[i]->data) {
			dma_free_coherent(priv->device, priv->dma_buf_sz, rx_q->rx_skbuff[i]->data, rx_q->rx_skbuff_dma[i]);
			rx_q->rx_skbuff[i]->data = NULL;
			
		}
		dev_kfree_skb(rx_q->rx_skbuff[i]);
	}
	rx_q->rx_skbuff[i] = NULL;

}

/**
 * bstvmac_free_tx_buffer - free RX dma buffers
 * @priv: private structure
 * @queue: RX queue index
 * @i: buffer index.
 */
static void bstvmac_free_tx_buffer(struct bstvmac_priv *priv, u32 queue, int i)
{
	struct bstvmac_tx_queue *tx_q = &priv->tx_queue[queue];

	if (tx_q->tx_skbuff_dma[i].buf) {
		if (tx_q->tx_skbuff_dma[i].map_as_page) {
			dma_unmap_page(priv->device,
				       tx_q->tx_skbuff_dma[i].buf,
				       tx_q->tx_skbuff_dma[i].len,
				       DMA_TO_DEVICE);
		} else {
			dma_unmap_single(priv->device,
					 tx_q->tx_skbuff_dma[i].buf,
					 tx_q->tx_skbuff_dma[i].len,
					 DMA_TO_DEVICE);
		}
	} 

	if (tx_q->tx_skbuff[i]) {
		dev_kfree_skb_any(tx_q->tx_skbuff[i]);
		tx_q->tx_skbuff[i] = NULL;
		tx_q->tx_skbuff_dma[i].buf = 0;
		tx_q->tx_skbuff_dma[i].map_as_page = false;
	}
}

/**
 * init_dma_rx_desc_rings - init the RX descriptor rings
 * @dev: net device structure
 * @flags: gfp flag.
 * Description: this function initializes the DMA RX descriptors
 * and allocates the socket buffers. It supports the chained and ring
 * modes.
 */
static int init_dma_rx_desc_rings(struct net_device *dev, gfp_t flags)
{
	struct bstvmac_priv *priv = netdev_priv(dev);
	u32 rx_count = priv->plat->rx_queues_to_use;
	int ret = -ENOMEM;
	int bfsize = 0;
	int queue;
	int i;

	bfsize = bstvmac_set_16kib_bfsize(dev->mtu);
	if (bfsize < 0)
		bfsize = 0;

	if (bfsize < BUF_SIZE_16KiB)
		bfsize = bstvmac_set_bfsize(dev->mtu, priv->dma_buf_sz);

	priv->dma_buf_sz = bfsize;
	printk("%s: dma_buf_sz = %d\n", __func__, priv->dma_buf_sz);

	/* RX INITIALIZATION */
	for (queue = 0; queue < rx_count; queue++) {
		struct bstvmac_rx_queue *rx_q = &priv->rx_queue[queue];

		for (i = 0; i < DMA_RX_SIZE; i++) {
			struct dma_bd_desc *p;

			p = rx_q->dma_bd_rx + i;
			ret = bstvmac_init_rx_buffers(priv, p, i, flags, queue);
			if (ret)
				goto err_init_rx_buffers;

			netif_dbg(priv, probe, priv->dev, "[%p]\t[%p]\t[%x]\n",
				  rx_q->rx_skbuff[i], rx_q->rx_skbuff[i]->data,
				  (unsigned int)rx_q->rx_skbuff_dma[i]);
		}
		spin_lock_init(&rx_q->que_lock);
	}

	buf_sz = bfsize;

	return 0;

err_init_rx_buffers:
	while (queue >= 0) {
		while (--i >= 0)
			bstvmac_free_rx_buffer(priv, queue, i);

		if (queue == 0)
			break;

		i = DMA_RX_SIZE;
		queue--;
	}

	return ret;
}

/**
 * init_dma_tx_desc_rings - init the TX descriptor rings
 * @dev: net device structure.
 * Description: this function initializes the DMA TX descriptors
 * and allocates the socket buffers. It supports the chained and ring
 * modes.
 */
static int init_dma_tx_desc_rings(struct net_device *dev)
{
	struct bstvmac_priv *priv = netdev_priv(dev);
	u32 tx_queue_cnt = priv->plat->tx_queues_to_use;
	u32 queue;
	int i;

	for (queue = 0; queue < tx_queue_cnt; queue++) {
		struct bstvmac_tx_queue *tx_q = &priv->tx_queue[queue];

		netif_dbg(priv, probe, priv->dev,
			  "(%s) dma_bd_tx_phy=0x%08x\n", __func__,
			  (u32)tx_q->dma_bd_tx_phy);

		bstvmac_init_tx_descriptors(priv, queue);

		/* Setup the chained descriptor addresses */
		bstvmac_mode_init(priv, tx_q->dma_bd_tx,
					tx_q->dma_bd_tx_phy,
					priv->dma_tx_size, 1);

		for (i = 0; i < DMA_TX_SIZE; i++) {
			tx_q->tx_skbuff_dma[i].buf = 0;
			tx_q->tx_skbuff_dma[i].map_as_page = false;
			tx_q->tx_skbuff_dma[i].len = 0;
			tx_q->tx_skbuff_dma[i].last_segment = false;
			tx_q->tx_skbuff[i] = NULL;
		}

		tx_q->dirty_tx = 0;
		tx_q->cur_tx = 0;
		tx_q->mss = 0;

		netdev_tx_reset_queue(netdev_get_tx_queue(priv->dev, queue));
	}

	return 0;
}

/**
 * uninit_dma_tx_desc_rings - uninit the TX descriptor rings
 * @dev: net device structure.
 * Description: this function uninit the DMA TX descriptors
 */
static int uninit_dma_tx_desc_rings(struct net_device *dev)
{
	struct bstvmac_priv *priv = netdev_priv(dev);
	u32 tx_queue_cnt = priv->plat->tx_queues_to_use;
	struct dma_bd_desc *p;
	struct dma_wrbd_desc *wp;
	u32 queue;
	int i;

	for (queue = 0; queue < tx_queue_cnt; queue++) {
		struct bstvmac_tx_queue *tx_q = &priv->tx_queue[queue];
		for (i = 0; i < DMA_TX_SIZE; i++) {
			p = tx_q->dma_bd_tx + i;
			wp = tx_q->dma_wrbd_tx + i;
			p->des0 = 0;
			p->des1 = 0;
			p->des2 = 0;
			p->des3 = 0;
			wp->des0 = 0;
			wp->des1 = 0;
		}
	}

	return 0;
}

/**
 * uninit_dma_rx_desc_rings - uninit the TX descriptor rings
 * @dev: net device structure.
 * Description: this function uninit the DMA TX descriptors
 */
static int uninit_dma_rx_desc_rings(struct net_device *dev)
{
	struct bstvmac_priv *priv = netdev_priv(dev);
	u32 rx_queue_cnt = priv->plat->rx_queues_to_use;
	struct dma_bd_desc *p;
	struct dma_wrbd_desc *wp;
	u32 queue;
	int i;

	for (queue = 0; queue < rx_queue_cnt; queue++) {
		struct bstvmac_rx_queue *rx_q = &priv->rx_queue[queue];
		for (i = 0; i < DMA_TX_SIZE; i++) {
			p = rx_q->dma_bd_rx + i;
			wp = rx_q->dma_wrbd_rx + i;
			p->des0 = 0;
			p->des1 = 0;
			p->des2 = 0;
			p->des3 = 0;
			wp->des0 = 0;
			wp->des1 = 0;
		}
	}

	return 0;
}

/**
 * init_dma_desc_rings - init the RX/TX descriptor rings
 * @dev: net device structure
 * @flags: gfp flag.
 * Description: this function initializes the DMA RX/TX descriptors
 * and allocates the socket buffers. It supports the chained and ring
 * modes.
 */
static int init_dma_desc_rings(struct net_device *dev, gfp_t flags)
{
	int ret;

	ret = init_dma_rx_desc_rings(dev, flags);
	if (ret)
		return ret;

	ret = init_dma_tx_desc_rings(dev);

	return ret;
}

/**
 * uninit_dma_desc_rings - uninit the RX/TX descriptor rings
 * @dev: net device structure
 * @flags: gfp flag.
 * Description: this function uninit the DMA RX/TX descriptors
 */
static int uninit_dma_desc_rings(struct net_device *dev)
{
	int ret;

	ret = uninit_dma_rx_desc_rings(dev);
	if (ret)
		return ret;

	ret = uninit_dma_tx_desc_rings(dev);

	return ret;
}

/**
 * dma_free_rx_skbufs - free RX dma buffers
 * @priv: private structure
 * @queue: RX queue index
 */
static void dma_free_rx_skbufs(struct bstvmac_priv *priv, u32 queue)
{
	int i;

	for (i = 0; i < DMA_RX_SIZE; i++)
		bstvmac_free_rx_buffer(priv, queue, i);
}

/**
 * dma_free_tx_skbufs - free TX dma buffers
 * @priv: private structure
 * @queue: TX queue index
 */
static void dma_free_tx_skbufs(struct bstvmac_priv *priv, u32 queue)
{
	struct bstvmac_tx_queue *tx_q = &priv->tx_queue[queue];
	int i;

	for (i = 0; i < DMA_TX_SIZE; i++)
		bstvmac_free_tx_buffer(priv, queue, i);

	tx_q->cur_tx = 0;
	tx_q->dirty_tx = 0;
	tx_q->mss = 0;
}

static void bstvmac_free_rxmem(struct bstvmac_priv *priv)
{
	int chan, busid, index;
	struct sk_buff_head *list;
	struct sk_buff *skb;
	dma_addr_t *buf;

	busid = priv->plat->bus_id;
	for (chan = 0; chan < priv->plat->rx_queues_to_use; chan++) {
		index = busid * BSTVMAC_RXCHAN_NUM + chan;
		list = vmac_delivery_skblist[index];
		while (list && skb_queue_len(list)) {
			skb = skb_dequeue(list);
			if (skb) {
				buf = (dma_addr_t *)skb->cb;
				if (*buf) {
					dma_unmap_single(priv->device, *buf,
							 priv->dma_buf_sz,
							 DMA_FROM_DEVICE);
					*buf = 0;
					dma_wmb();
					dev_kfree_skb_any(skb);
				}
			}
		}
	}
}

static void free_dma_rx_mem_res(struct bstvmac_priv *priv)
{
	int i;
	int it_ms = 10, cnt = (BSTVMAC_TRANS_TIMEOUT_MS + it_ms) / it_ms;

	i = 0;
	while ((i < cnt) && (test_bit(BSTVMAC_RXMEM_WORK_RUN, &priv->state))) {
		msleep(it_ms);
		i++;
	}
	if (i != cnt)
		bstvmac_free_rxmem(priv);
	else
		pr_err("An exception occurred, so rxmem buf was not released\n");
}

/**
 * free_dma_rx_desc_resources - free RX dma desc resources
 * @priv: private structure
 */
static void free_dma_rx_desc_resources(struct bstvmac_priv *priv)
{
	u32 rx_count = priv->plat->rx_queues_to_use;
	u32 queue;

	/* Free RX queue resources */
	for (queue = 0; queue < rx_count; queue++) {
		struct bstvmac_rx_queue *rx_q = &priv->rx_queue[queue];

		/* Release the DMA RX socket buffers */
		dma_free_rx_skbufs(priv, queue);

		/* Free DMA regions of consistent memory previously allocated */
		// dma_free_coherent(priv->device,
		// 			DMA_RX_SIZE * sizeof(struct dma_bd_desc),
		// 			rx_q->dma_bd_rx, rx_q->dma_bd_rx_phy);

		// dma_free_coherent(priv->device, DMA_RX_SIZE *
		// 			sizeof(struct dma_wrbd_desc),
		// 			rx_q->dma_wrbd_rx, rx_q->dma_wrbd_rx_phy);

		dma_free_attrs(priv->device, DMA_RX_SIZE * sizeof(struct dma_bd_desc), rx_q->dma_bd_rx, rx_q->dma_bd_rx_phy, 0);
		dma_free_attrs(priv->device, DMA_RX_SIZE * sizeof(struct dma_wrbd_desc), rx_q->dma_wrbd_rx, rx_q->dma_wrbd_rx_phy, 0);

		kfree(rx_q->rx_skbuff_dma);
		kfree(rx_q->rx_skbuff);
	}
}

/**
 * free_dma_tx_desc_resources - free TX dma desc resources
 * @priv: private structure
 */
static void free_dma_tx_desc_resources(struct bstvmac_priv *priv)
{
	u32 tx_count = priv->plat->tx_queues_to_use;
	u32 queue;

	/* Free TX queue resources */
	for (queue = 0; queue < tx_count; queue++) {
		struct bstvmac_tx_queue *tx_q = &priv->tx_queue[queue];
		size_t size;
		void *addr;

		/* Release the DMA TX socket buffers */
		dma_free_tx_skbufs(priv, queue);

		size = sizeof(struct dma_bd_desc);
		addr = tx_q->dma_bd_tx;
		size *= priv->dma_tx_size;
		dma_free_coherent(priv->device, size, addr, tx_q->dma_bd_tx_phy);

		size = sizeof(struct dma_wrbd_desc);
		addr = tx_q->dma_wrbd_tx;
		size *= priv->dma_tx_size;
		dma_free_coherent(priv->device, size, addr, tx_q->dma_wrbd_tx_phy);

		kfree(tx_q->tx_skbuff_dma);
		kfree(tx_q->tx_skbuff);
	}
}

/**
 * alloc_dma_rx_desc_resources - alloc RX resources.
 * @priv: private structure
 * Description: according to which descriptor can be used (extend or basic)
 * this function allocates the resources for TX and RX paths. In case of
 * reception, for example, it pre-allocated the RX socket buffer in order to
 * allow zero-copy mechanism.
 */

static int alloc_dma_rx_desc_resources(struct bstvmac_priv *priv)
{
	u32 rx_count = priv->plat->rx_queues_to_use;
	int ret = -ENOMEM;
	u32 queue;

	/* RX queues buffers and DMA */
	for (queue = 0; queue < rx_count; queue++) {
		struct bstvmac_rx_queue *rx_q = &priv->rx_queue[queue];
		struct bstvmac_channel *ch = &priv->rx_channel[queue];
		u32 regval;

		ch->base_addr = priv->chanl_start_addr + (queue * 0x1000);
		rx_q->queue_index = queue;
		rx_q->priv_data = priv;

		rx_q->rx_skbuff_dma = kmalloc_array(DMA_RX_SIZE,
						    sizeof(dma_addr_t),
						    GFP_KERNEL);
		if (!rx_q->rx_skbuff_dma)
			goto err_dma;

		rx_q->rx_skbuff = kmalloc_array(DMA_RX_SIZE,
						sizeof(struct sk_buff *),
						GFP_KERNEL);
		if (!rx_q->rx_skbuff)
			goto err_dma;


		rx_q->dma_bd_rx = dma_alloc_attrs(priv->device, DMA_RX_SIZE *sizeof(struct dma_bd_desc),&rx_q->dma_bd_rx_phy, GFP_KERNEL, 1);
		// rx_q->dma_bd_rx = dma_alloc_coherent(priv->device,
		// 					DMA_RX_SIZE *
		// 					sizeof(struct
		// 						dma_bd_desc),
		// 					&rx_q->dma_bd_rx_phy,
		// 					GFP_KERNEL);
		if (!rx_q->dma_bd_rx)
			goto err_dma;

		printk("%s: dma_bd_rx = %px, dma_bd_rx_phy = %px\n", __func__,
			  (void *)rx_q->dma_bd_rx, (void *)rx_q->dma_bd_rx_phy);
		rx_q->dma_wrbd_rx = dma_alloc_attrs(priv->device, DMA_RX_SIZE *sizeof(struct dma_wrbd_desc),&rx_q->dma_wrbd_rx_phy, GFP_KERNEL, 1);
		// rx_q->dma_wrbd_rx = dma_alloc_coherent(priv->device,
		// 					DMA_RX_SIZE *
		// 					sizeof(struct
		// 						dma_wrbd_desc),
		// 					&rx_q->dma_wrbd_rx_phy,
		// 					GFP_KERNEL);
		if (!rx_q->dma_wrbd_rx)
			goto err_dma;

		printk("%s: dma_wrbd_rx = %px, dma_wrbd_rx_phy = %px\n", __func__,
			  (void *)rx_q->dma_wrbd_rx, (void *)rx_q->dma_wrbd_rx_phy);

		regval = readl(ch->base_addr + HIF_RX_STATUS_0_CH(queue));
		if (regval) {
			rx_q->seq_num = (regval & 0xffff) + 1;
		} else {
			rx_q->seq_num = 1;
		}
		printk("%s: rx seq_num = %d\n", __func__, rx_q->seq_num);
	}

	return 0;

err_dma:
	free_dma_rx_desc_resources(priv);

	return ret;
}

/**
 * alloc_dma_tx_desc_resources - alloc TX resources.
 * @priv: private structure
 * Description: according to which descriptor can be used (extend or basic)
 * this function allocates the resources for TX and RX paths. In case of
 * reception, for example, it pre-allocated the RX socket buffer in order to
 * allow zero-copy mechanism.
 */
static int alloc_dma_tx_desc_resources(struct bstvmac_priv *priv)
{
	u32 tx_count = priv->plat->tx_queues_to_use;
	int ret = -ENOMEM;
	u32 queue;

	/* TX queues buffers and DMA */
	for (queue = 0; queue < tx_count; queue++) {
		struct bstvmac_tx_queue *tx_q = &priv->tx_queue[queue];
		struct bstvmac_channel *ch = &priv->tx_channel[queue];
		size_t size;
		void *addr;
		u32 regval;

		ch->base_addr = priv->chanl_start_addr + (queue * 0x1000);
		tx_q->queue_index = queue;
		tx_q->priv_data = priv;

		tx_q->tx_skbuff_dma = kmalloc_array(DMA_TX_SIZE,
						    sizeof
						    (*tx_q->tx_skbuff_dma),
						    GFP_KERNEL);
		if (!tx_q->tx_skbuff_dma)
			goto err_dma;

		tx_q->tx_skbuff = kmalloc_array(DMA_TX_SIZE,
						sizeof(struct sk_buff *),
						GFP_KERNEL);
		if (!tx_q->tx_skbuff)
			goto err_dma;

		size = sizeof(struct dma_bd_desc);
		size *= priv->dma_tx_size;
		addr = dma_alloc_coherent(priv->device, size,
					  &tx_q->dma_bd_tx_phy, GFP_KERNEL);
		if (!addr)
			goto err_dma;

		tx_q->dma_bd_tx = addr;

		printk("%s: dma_bd_tx = %px, dma_bd_tx_phy = %px\n", __func__,
			  (void *)tx_q->dma_bd_tx, (void *)tx_q->dma_bd_tx_phy);

		size = sizeof(struct dma_wrbd_desc);
		size *= priv->dma_tx_size;
		addr = dma_alloc_coherent(priv->device, size,
					  &tx_q->dma_wrbd_tx_phy, GFP_KERNEL);
		if (!addr)
			goto err_dma;

		tx_q->dma_wrbd_tx = addr;

		printk("%s: dma_wrbd_tx = %px, dma_wrbd_tx_phy = %px\n", __func__,
			  (void *)tx_q->dma_wrbd_tx, (void *)tx_q->dma_wrbd_tx_phy);

		regval = readl(ch->base_addr + HIF_TX_STATUS_1_CH(queue));
		if (regval) {
			tx_q->seq_num = (regval & 0xffff) + 1;
		} else {
			tx_q->seq_num = 1;
		}
		printk("%s: tx seq_num = %d\n", __func__, tx_q->seq_num);
	}

	return 0;

err_dma:
	free_dma_tx_desc_resources(priv);

	return ret;
}

/**
 * alloc_dma_desc_resources - alloc TX/RX resources.
 * @priv: private structure
 * Description: according to which descriptor can be used (extend or basic)
 * this function allocates the resources for TX and RX paths. In case of
 * reception, for example, it pre-allocated the RX socket buffer in order to
 * allow zero-copy mechanism.
 */
static int alloc_dma_desc_resources(struct bstvmac_priv *priv)
{
	/* RX Allocation */
	int ret = alloc_dma_rx_desc_resources(priv);

	if (ret)
		return ret;

	ret = alloc_dma_tx_desc_resources(priv);

	return ret;
}

/**
 * free_dma_desc_resources - free dma desc resources
 * @priv: private structure
 */
static void free_dma_desc_resources(struct bstvmac_priv *priv)
{
	/* Release the DMA RX socket buffers */
	free_dma_rx_desc_resources(priv);

	/* Release the DMA TX socket buffers */
	free_dma_tx_desc_resources(priv);
}

/**
 * bstvmac_start_rx_dma - start RX DMA channel
 * @priv: driver private structure
 * @chan: RX channel index
 * Description:
 * This starts a RX DMA channel
 */
static void bstvmac_start_rx_dma(struct bstvmac_priv *priv, u32 chan)
{
	netdev_dbg(priv->dev, "DMA RX processes started in channel %d\n", chan);
	bstvmac_start_rx(priv, priv->chanl_start_addr, chan);
}

/**
 * bstvmac_start_tx_dma - start TX DMA channel
 * @priv: driver private structure
 * @chan: TX channel index
 * Description:
 * This starts a TX DMA channel
 */
static void bstvmac_start_tx_dma(struct bstvmac_priv *priv, u32 chan)
{
	netdev_dbg(priv->dev, "DMA TX processes started in channel %d\n", chan);
	bstvmac_start_tx(priv, priv->chanl_start_addr, chan);
}

/**
 * bstvmac_stop_rx_dma - stop RX DMA channel
 * @priv: driver private structure
 * @chan: RX channel index
 * Description:
 * This stops a RX DMA channel
 */
static void bstvmac_stop_rx_dma(struct bstvmac_priv *priv, u32 chan)
{
	netdev_dbg(priv->dev, "DMA RX processes stopped in channel %d\n", chan);
	bstvmac_stop_rx(priv, priv->chanl_start_addr, chan);
}

/**
 * bstvmac_stop_tx_dma - stop TX DMA channel
 * @priv: driver private structure
 * @chan: TX channel index
 * Description:
 * This stops a TX DMA channel
 */
static void bstvmac_stop_tx_dma(struct bstvmac_priv *priv, u32 chan)
{
	netdev_dbg(priv->dev, "DMA TX processes stopped in channel %d\n", chan);
	bstvmac_stop_tx(priv, priv->chanl_start_addr, chan);
}

/**
 * bstvmac_start_all_dma - start all RX and TX DMA channels
 * @priv: driver private structure
 * Description:
 * This starts all the RX and TX DMA channels
 */
static void bstvmac_start_all_dma(struct bstvmac_priv *priv)
{
	u32 rx_channels_count = priv->plat->rx_queues_to_use;
	u32 tx_channels_count = priv->plat->tx_queues_to_use;
	u32 chan = 0;

	for (chan = 0; chan < rx_channels_count; chan++)
		bstvmac_start_rx_dma(priv, chan);

	for (chan = 0; chan < tx_channels_count; chan++)
		bstvmac_start_tx_dma(priv, chan);
}

/**
 * bstvmac_stop_all_dma - stop all RX and TX DMA channels
 * @priv: driver private structure
 * Description:
 * This stops the RX and TX DMA channels
 */
static void bstvmac_stop_all_dma(struct bstvmac_priv *priv)
{
	u32 rx_channels_count = priv->plat->rx_queues_to_use;
	u32 tx_channels_count = priv->plat->tx_queues_to_use;
	u32 chan = 0;

	for (chan = 0; chan < rx_channels_count; chan++)
		bstvmac_stop_rx_dma(priv, chan);

	for (chan = 0; chan < tx_channels_count; chan++)
		bstvmac_stop_tx_dma(priv, chan);
}

/**
 *  bstvmac_dma_operation_mode - HW DMA operation mode
 *  @priv: driver private structure
 *  Description: it is used for configuring the DMA operation mode register in
 *  order to program the tx/rx DMA thresholds or Store-And-Forward mode.
 */
static void bstvmac_dma_operation_mode(struct bstvmac_priv *priv)
{
	u32 rx_channels_count = priv->plat->rx_queues_to_use;
	u32 tx_channels_count = priv->plat->tx_queues_to_use;
	u32 chan = 0;

	/* configure all rx channels */
	for (chan = 0; chan < rx_channels_count; chan++) {
		/* Config Rx Ring Len */
		bstvmac_set_rx_dma_bfsize(priv, priv->chanl_start_addr,
					DMA_RX_SIZE, chan);
		/* Enable DMA Rx Interrupt */
		bstvmac_enable_dma_irq(priv, priv->chanl_start_addr, chan, 1, 0);
	}

	/* configure all tx channels */
	for (chan = 0; chan < tx_channels_count; chan++) {
		/* Config Tx Ring Len */
		bstvmac_set_tx_dma_bfsize(priv, priv->chanl_start_addr,
					DMA_TX_SIZE, chan);
		/* Enable DMA Tx Interrupt */
		bstvmac_enable_dma_irq(priv, priv->chanl_start_addr, chan, 0, 1);
	}
}

/**
 * bstvmac_tx_clean - to manage the transmission completion
 * @priv: driver private structure
 * @queue: TX queue index
 * Description: it reclaims the transmit resources after transmission completes.
 */
static int bstvmac_tx_clean(struct bstvmac_priv *priv, u32 queue, int *rem_cnt)
{
	struct bstvmac_tx_queue *tx_q = &priv->tx_queue[queue];
	unsigned int bytes_compl = 0, pkts_compl = 0;
	unsigned int entry, count = 0;
	int budget = DMA_TX_SIZE;

	__netif_tx_lock_bh(netdev_get_tx_queue(priv->dev, queue));

	if (test_bit(BSTVMAC_DOWN, &priv->state))
		goto end;
	else
		tx_q->run_status = 1;

	priv->xstats.tx_clean++;

	entry = tx_q->dirty_tx;
	while ((entry != tx_q->cur_tx) && (count < budget)) {
		struct sk_buff *skb = tx_q->tx_skbuff[entry];
		struct dma_bd_desc *p;
		struct dma_wrbd_desc *wp;
		int status;

		p = tx_q->dma_bd_tx + entry;
		wp = tx_q->dma_wrbd_tx + entry;
		status = bstvmac_tx_status(priv, &priv->dev->stats,
					   &priv->xstats, p, wp, priv->chanl_start_addr);

		/* Check if the descriptor is owned by the DMA */
		if (unlikely(status & tx_not_done)) {
			break;
		}

		count++;
		/* Make sure descriptor fields are read after reading
		 * the own bit.
		 */
		dma_rmb();

		/* Just consider the last segment and ... */
		if (likely(!(status & tx_not_ls))) {
			/* ... verify the status error condition */
			if (unlikely(status & tx_err)) {
				priv->dev->stats.tx_errors++;
			} else {
				priv->dev->stats.tx_packets++;
				priv->xstats.tx_pkt_n++;
			}
		}

		if (likely(tx_q->tx_skbuff_dma[entry].buf)) {
			if (tx_q->tx_skbuff_dma[entry].map_as_page)
				dma_unmap_page(priv->device,
					       tx_q->tx_skbuff_dma[entry].buf,
					       tx_q->tx_skbuff_dma[entry].len,
					       DMA_TO_DEVICE);
			else
				dma_unmap_single(priv->device,
						 tx_q->tx_skbuff_dma[entry].buf,
						 tx_q->tx_skbuff_dma[entry].len,
						 DMA_TO_DEVICE);
			tx_q->tx_skbuff_dma[entry].buf = 0;
			tx_q->tx_skbuff_dma[entry].len = 0;
			tx_q->tx_skbuff_dma[entry].map_as_page = false;
		}

		tx_q->tx_skbuff_dma[entry].last_segment = false;
		tx_q->tx_skbuff_dma[entry].is_jumbo = false;

		if (likely(skb)) {
			pkts_compl++;
			bytes_compl += skb->len;
			if (refcount_read(&skb->users))
				dev_consume_skb_any(skb);
			tx_q->tx_skbuff[entry] = NULL;
		}

		if (likely(p))
			bstvmac_release_tx_desc(priv, p, wp, priv->mode);

		if (entry != tx_q->cur_tx)
			entry = BSTVMAC_GET_ENTRY(entry, DMA_TX_SIZE);
	}
	tx_q->dirty_tx = entry;

	if (!test_bit(BSTVMAC_RX_FIFO_CLEAR, &priv->state))
		netdev_tx_completed_queue(netdev_get_tx_queue(priv->dev, queue),
				  	pkts_compl, bytes_compl);
	
	if (unlikely(netif_tx_queue_stopped(netdev_get_tx_queue(priv->dev,
								queue))) &&
	    bstvmac_tx_avail(priv, queue) > BSTVMAC_TX_THRESH) {
		netdev_dbg(priv->dev, "%s: restart transmit (dirty_tx=%d cur_tx=%d cnt=%d)\n",
				__func__, priv->tx_queue[queue].dirty_tx, priv->tx_queue[queue].cur_tx,
				bstvmac_tx_avail(priv, queue));
		netif_tx_wake_queue(netdev_get_tx_queue(priv->dev, queue));
	}

	/* We still have pending packets, let's call for a new scheduling */
	if (tx_q->dirty_tx != tx_q->cur_tx)
		bstvmac_tx_timer_arm(priv, queue);

	tx_q->run_status = 0;
end:
	__netif_tx_unlock_bh(netdev_get_tx_queue(priv->dev, queue));

	return count;
}

/**
 * bstvmac_check_ether_addr - check if the MAC addr is valid
 * @priv: driver private structure
 * Description:
 * it is to verify if the MAC address is valid, in case of failures it
 * generates a random MAC address
 */
static void bstvmac_check_ether_addr(struct bstvmac_priv *priv)
{
	if (is_valid_ether_addr(priv->dev->dev_addr)) {
		eth_hw_addr_set(priv->dev, priv->dev->dev_addr);
	} else {
		eth_hw_addr_random(priv->dev);
	}

	dev_info(priv->device, "device MAC address %pM\n",
			priv->dev->dev_addr);
}

/**
 * bstvmac_init_dma_engine - DMA init.
 * @priv: driver private structure
 * Description:
 * It inits the DMA invoking the specific MAC/GMAC callback.
 * Some DMA parameters can be passed from the platform;
 * in case of these are not passed a default is kept for the MAC or GMAC.
 */
static int bstvmac_init_dma_engine(struct bstvmac_priv *priv)
{
	u32 rx_channels_count = priv->plat->rx_queues_to_use;
	u32 tx_channels_count = priv->plat->tx_queues_to_use;
	struct bstvmac_tx_queue *tx_q;
	struct bstvmac_rx_queue *rx_q;
	int ret = 0;
	u32 chan = 0;

	/* DMA RX Channel Configuration */
	for (chan = 0; chan < rx_channels_count; chan++) {
		rx_q = &priv->rx_queue[chan];
		bstvmac_init_rx_chan(priv, priv->chanl_start_addr,
				    rx_q->dma_bd_rx_phy, rx_q->dma_wrbd_rx_phy, chan);
		rx_q->rx_tail_addr = rx_q->dma_bd_rx_phy +
		    (DMA_RX_SIZE * sizeof(struct dma_bd_desc));
	}

	/* DMA TX Channel Configuration */
	for (chan = 0; chan < tx_channels_count; chan++) {
		tx_q = &priv->tx_queue[chan];
		bstvmac_init_tx_chan(priv, priv->chanl_start_addr,
				    tx_q->dma_bd_tx_phy, tx_q->dma_wrbd_tx_phy, chan);
		tx_q->tx_tail_addr = tx_q->dma_bd_tx_phy;
	}

	return ret;
}

static void bstvmac_mac_init(struct bstvmac_priv *priv)
{
	u32 regval;

	regval = CSR_BDPRD_AXI_WRITE_DONE | CSR_BDPWR_AXI_WRITE_DONE |
		CSR_RXDXR_AXI_WRITE_DONE | CSR_TXDXR_AXI_WRITE_DONE;
	bstvmac_axi_write_done_set(priv, priv->hif_base_addr, regval);
	bstvmac_timeout_en(priv, priv->hif_base_addr, true);
	bstvmac_start_seqnum_set(priv, priv->hif_base_addr, 1);
	bstvmac_seqnum_check_en(priv, priv->hif_base_addr, false);
}


static void bstvmac_tx_timer_arm(struct bstvmac_priv *priv, u32 queue)
{
	struct bstvmac_tx_queue *tx_q = &priv->tx_queue[queue];
	u32 tx_coal_timer = priv->tx_coal_timer[queue];

	if (!tx_coal_timer)
		return;

	hrtimer_start(&tx_q->txtimer,
		      BSTVMAC_COAL_TIMER(tx_coal_timer),
		      HRTIMER_MODE_REL);
}

/**
 * bstvmac_tx_timer - mitigation sw timer for tx.
 * @data: data pointer
 * Description:
 * This is the timer handler to directly invoke the bstvmac_tx_clean.
 */
static enum hrtimer_restart bstvmac_tx_timer(struct hrtimer *t)
{
	struct bstvmac_tx_queue *tx_q = container_of(t, struct bstvmac_tx_queue, txtimer);
	struct bstvmac_priv *priv = tx_q->priv_data;
	struct bstvmac_channel *ch;
	int cpuid;

	ch = &priv->tx_channel[tx_q->queue_index];
	cpuid = (num_online_cpus() > 1) ? BSTMAC_TXWQ_DEF_CPU : 1;
	queue_work_on(cpuid, priv->tx_wq, &ch->tx_work);

	return HRTIMER_NORESTART;
}
static unsigned int rx_work_mode = 0;
static enum hrtimer_restart bstvmac_rx_poll_work(struct bstvmac_priv *priv);
static enum hrtimer_restart bstvmac_rx_timer(struct hrtimer *t)
{
	struct bstvmac_rx_queue *rx_q = container_of(t, struct bstvmac_rx_queue, rxtimer);
	struct bstvmac_priv *priv = rx_q->priv_data;
    // if(rx_cnt%1000 == 0) {
    //     printk("priv->xstats.tx_set_ic_bit:%ld,%ld,%ld,%ld", priv->xstats.tx_set_ic_bit,  priv->xstats.tx_normal_irq_n, priv->xstats.txwork_poll, priv->xstats.tx_pkt_n);
    //     printk("priv->rx_normal_irq_n:%ld,%ld,%ld,%ld", priv->xstats.rx_normal_irq_n, priv->xstats.rnapi_poll, priv->xstats.rx_pkt_n,priv->xstats.rx_memwork_poll);
    // }
    //bstvmac_rx_poll_work(priv);
    //rx_cnt++;
    return bstvmac_rx_poll_work(priv);
}

static enum hrtimer_restart bstvmac_rx_timer2(struct hrtimer *t)
{
	struct bstvmac_rx_queue *rx_q = container_of(t, struct bstvmac_rx_queue, rxtimer2);
	struct bstvmac_priv *priv = rx_q->priv_data;

    static unsigned long rx_packets = 0;

    if(rx_work_mode == 0 && priv->dev->stats.rx_packets - rx_packets > 400) {
        rx_work_mode = 1;
        for (int i = 0; i < priv->dma_rx_size; i++) {
            if(i%BSTVMAC_RX_FRAMES != 0) {
                bstvmac_set_bd_rx_intr(priv, &rx_q->dma_bd_rx[i], false);
            }
        }
        hrtimer_start(&priv->rx_queue[0].rxtimer,
            BSTVMAC_COAL_TIMER(200),
            HRTIMER_MODE_REL);

    }else if(rx_work_mode == 1 && priv->dev->stats.rx_packets - rx_packets <= 300) {
        for (int i = 0; i < priv->dma_rx_size; i++) {
            bstvmac_set_bd_rx_intr(priv, &rx_q->dma_bd_rx[i], true);
        }
        rx_work_mode = 0;
    }
    rx_packets = priv->dev->stats.rx_packets;
    hrtimer_forward_now(&priv->rx_queue[0].rxtimer2, BSTVMAC_COAL_TIMER(10000));
	return HRTIMER_RESTART;
}

static void bstvmac_init_rxmem(struct bstvmac_priv *priv)
{
	int i, chan, index;
	struct sk_buff *skb;
	dma_addr_t buf;
	int size = priv->dma_buf_sz;
	struct sk_buff_head *list;
	dma_addr_t *addr;

	for (chan = 0; chan < priv->plat->rx_queues_to_use; chan++) {
		index = priv->plat->bus_id * BSTVMAC_RXCHAN_NUM + chan;
		list = vmac_delivery_skblist[index];
		for (i = 0; i < BSTVMAC_RXMEM_MAX; i++) {
			skb = netdev_alloc_skb_ip_align(priv->dev, size);
			if (unlikely(!skb))
				break;

			buf = dma_map_single(priv->device, skb->data, size,
					     DMA_FROM_DEVICE);
			if (dma_mapping_error(priv->device, buf)) {
				netdev_err(priv->dev, "Gmac map list failed\n");
				dev_kfree_skb(skb);
				break;
			}
			addr = (dma_addr_t *)skb->cb;
			*addr = buf;
			dma_wmb();
			skb_queue_tail(list, skb);
		}
	}
}

/**
 * bstvmac_init_coalesce - init mitigation options.
 * @priv: driver private structure
 * Description:
 * This inits the coalesce parameters: i.e. timer rate,
 * timer handler and default threshold used for enabling the
 * interrupt on completion bit.
 */
static void bstvmac_init_coalesce(struct bstvmac_priv *priv)
{
	u32 tx_channel_count = priv->plat->tx_queues_to_use;
	u32 rx_channel_count = priv->plat->rx_queues_to_use;
	u32 chan;

	for (chan = 0; chan < tx_channel_count; chan++) {
		struct bstvmac_tx_queue *tx_q = &priv->tx_queue[chan];

		priv->tx_coal_frames[chan] = BSTVMAC_TX_FRAMES;
		priv->tx_coal_timer[chan] = BSTVMAC_COAL_TX_TIMER;

		hrtimer_init(&tx_q->txtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		tx_q->txtimer.function = bstvmac_tx_timer;
	}

	for (chan = 0; chan < rx_channel_count; chan++) {
        struct bstvmac_rx_queue *rx_q = &priv->rx_queue[chan];
        hrtimer_init(&rx_q->rxtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        hrtimer_init(&rx_q->rxtimer2, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        rx_q->rxtimer.function = bstvmac_rx_timer;
        rx_q->rxtimer2.function = bstvmac_rx_timer2;
		//priv->rx_coal_frames[chan] = BSTVMAC_RX_FRAMES;
    }
}

/**
 * bstvmac_hw_setup - setup mac in a usable state.
 *  @dev : pointer to the device structure.
 *  Description:
 *  this is the main function to setup the HW in a usable state because the
 *  dma engine is reset, the core registers are configured (e.g. AXI,
 *  Checksum features, timers). The DMA is ready to start receiving and
 *  transmitting.
 *  Return value:
 *  0 on success and an appropriate (-)ve integer as defined in errno.h
 *  file on failure.
 */
static int bstvmac_hw_setup(struct net_device *dev, bool init_ptp)
{
	struct bstvmac_priv *priv = netdev_priv(dev);
	int ret;

	/* DMA initialization */
	ret = bstvmac_init_dma_engine(priv);
	if (ret < 0) {
		netdev_err(priv->dev, "%s: DMA engine initialization failed\n",
			   __func__);
		return ret;
	}

	/* Set the HW DMA Tx/Rx Buf Size */
	bstvmac_dma_operation_mode(priv);

	/* Enable Seqnum Check & set start seqnum */
	bstvmac_mac_init(priv);

	return 0;
}

/**
 * bstvmac_syncmac_to_switch - send mac addr to sw.
 *  @priv: driver private structure
 *  Description:
 *  this is the main function to send mac addr to sw system
 */
static void bstvmac_syncmac_to_switch(struct bstvmac_priv *priv, uint32_t flag)
{
	int ret = 0, cnt = 0;
	char mac_addr[6] = {0};
	void __iomem * mac_io_map_addr = NULL;
	u32 mac_h,mac_l;
	memcpy(mac_addr, priv->dev->dev_addr, priv->dev->addr_len);
	if(DB_MAC_ADDR_FLAG == flag)
	{
	    mac_io_map_addr = ioremap(0x80cb01000,8);
		if (!mac_io_map_addr) {
			pr_err("Failed to map db mac_io_map_addr memory\n");
		} 
	}
	else if(IVI_MAC_ADDR_FLAG == flag)
	{
	    mac_io_map_addr = ioremap(0x80cb02000,8);
		if (!mac_io_map_addr) {
			pr_err("Failed to map ivi mac_io_map_addr memory\n");
		} 
	}

	if (mac_io_map_addr){
		mac_h = (mac_addr[0]<<24)|(mac_addr[1]<<16)|(mac_addr[2]<<8)|(mac_addr[3]);
		mac_l = (mac_addr[4]<<24)|(mac_addr[5]<<16);
		writel(mac_h,mac_io_map_addr);
		writel(mac_l,mac_io_map_addr+4);
		// read_mac = readl(mac_io_map_addr);
		// printk("%s readmac_h:%x\n",__func__, read_mac);
		// read_mac = readl(mac_io_map_addr+4);
		// printk("%s readmac_l:%x\n",__func__, read_mac);
		iounmap(mac_io_map_addr);
	}
retry:
    ret = bstvmac_set_umac_addr(priv, priv->dev, flag);
	if (ret) {
		cnt++;
		if (cnt <= BSTVMAC_MSGBOX_MAX_CNT) {
			netdev_dbg(priv->dev, "%s msgbox try again (ret = %d)\n", __func__, ret);
			msleep(500);
			goto retry;
		}
	}
}

/**
 *  bstvmac_open - open entry point of the driver
 *  @dev : pointer to the device structure.
 *  Description:
 *  This function is the open entry point of the driver.
 *  Return value:
 *  0 on success and an appropriate (-)ve integer as defined in errno.h
 *  file on failure.
 */
static int bstvmac_open(struct net_device *dev)
{
	int ret, chan;
	struct cpumask mask;
	struct bstvmac_priv *priv = netdev_priv(dev);
	u32 rx_wrbd_addr_star,rx_bd_addr_star,tx_wrbd_addr_star,tx_bd_addr_star;
	dma_addr_t value,temp;
	u32 tx_index,rx_index;
	u32 rx_count = priv->plat->rx_queues_to_use;
	u32 tx_count = priv->plat->tx_queues_to_use;
	void __iomem * tx_cur_bd_addr;
	void __iomem * rx_cur_bd_addr;
	void __iomem * rx_wrbd_addr_star_v;
	void __iomem * rx_bd_addr_star_v;
	void __iomem * tx_wrbd_addr_star_v;
	void __iomem * tx_bd_addr_star_v;
	int queue;
	int reset_flag = 0;
	rx_wrbd_addr_star_v = priv->chanl_start_addr +  0x04;
	rx_wrbd_addr_star =readl(rx_wrbd_addr_star_v);
	rx_bd_addr_star_v = priv->chanl_start_addr +  0x0c;
	rx_bd_addr_star =readl(rx_bd_addr_star_v);
	tx_wrbd_addr_star_v = priv->chanl_start_addr +  0x14;
	tx_wrbd_addr_star =readl(tx_wrbd_addr_star_v);
	tx_bd_addr_star_v = priv->chanl_start_addr +  0x1c;
	tx_bd_addr_star =readl(tx_bd_addr_star_v);
	if((rx_wrbd_addr_star != 0)&&(rx_bd_addr_star != 0)&&(tx_wrbd_addr_star != 0)&&(tx_bd_addr_star != 0))
	{
		reset_flag = 1;
		bstvmac_stop_all_dma(priv);
		printk("%s: (reset),bstvmac_stop_all_dma\n", __func__);
	}
	/* Extra statistics */
	memset(&priv->xstats, 0, sizeof(struct bstvmac_extra_stats));
	priv->dma_buf_sz = BSTVMAC_ALIGN(buf_sz);
	
	if (!priv->dma_tx_size)
		priv->dma_tx_size = DMA_TX_SIZE;
	if (!priv->dma_rx_size)
		priv->dma_rx_size = DMA_RX_SIZE;

	ret = alloc_dma_desc_resources(priv);
	if (ret < 0) {
		netdev_err(priv->dev, "%s: DMA descriptors allocation failed\n",
			   __func__);
		goto dma_desc_error;
	}

	ret = init_dma_desc_rings(dev, GFP_KERNEL);
	if (ret < 0) {
		netdev_err(priv->dev,
			   "%s: DMA descriptors initialization failed\n",
			   __func__);
		goto init_error;
	}

	if(1 == reset_flag)
	{
		for (queue = 0; queue < rx_count; queue++) {
			struct bstvmac_rx_queue *rx_q = &priv->rx_queue[queue];
			rx_cur_bd_addr = priv->chanl_start_addr +  0xc8;
			value =(dma_addr_t) readl(rx_cur_bd_addr);
			printk("%s: (reset)rx_cur_bd_addr is :%px,rx_q->dma_wrbd_rx_phy is:%px\n", __func__,(void *)value,(void *)rx_q->dma_wrbd_rx_phy);
			temp = rx_q->dma_wrbd_rx_phy &0xffffffff;
			rx_index =  (u32)(value - temp)/8;
			printk("%s: (reset)rx_index is :%x\n", __func__,rx_index);
			rx_q->cur_rx = rx_index;
			rx_q->dirty_rx = rx_index;
			bstvmac_init_rx_descriptors(priv, queue);
			bstvmac_mode_init(priv, rx_q->dma_bd_rx,
				rx_q->dma_bd_rx_phy, DMA_RX_SIZE, 0);
		}
		for (queue = 0; queue < tx_count; queue++) {
			struct bstvmac_tx_queue *tx_q = &priv->tx_queue[queue];
			tx_cur_bd_addr = priv->chanl_start_addr +  0x88;
			value = (dma_addr_t)readl(tx_cur_bd_addr);
			printk("%s: (reset)tx_cur_bd_addr is :%px,tx_q->dma_wrbd_tx_phy is %px\n", __func__,(void *)value,(void *)tx_q->dma_wrbd_tx_phy);
			temp = tx_q->dma_wrbd_tx_phy &0xffffffff;
			tx_index =  (u32)(value - temp)/8;
			printk("%s: (reset)tx_index is :%x\n", __func__,tx_index);
			tx_q->dirty_tx = tx_index;
			tx_q->cur_tx = tx_index;
		}
	}
	else{
		//设置rx bd0-3
		for (queue = 0; queue < rx_count; queue++) {
			struct bstvmac_rx_queue *rx_q = &priv->rx_queue[queue];
			rx_q->cur_rx = 0;
			rx_q->dirty_rx = 0;
			bstvmac_init_rx_descriptors(priv, queue);
			/* Setup the chained descriptor addresses */
			bstvmac_mode_init(priv, rx_q->dma_bd_rx,
				rx_q->dma_bd_rx_phy, DMA_RX_SIZE, 0);
		}
		
		//设置epp寄存器
		ret = bstvmac_hw_setup(dev, true);
		if (ret < 0) {
			netdev_err(priv->dev, "%s: Hw setup failed\n", __func__);
			goto init_error;
		}
	}

	bstvmac_init_coalesce(priv);

	/* Request the Tx/Rx chan IRQ lines */
	ret = request_irq(dev->irq, bstvmac_interrupt,
			IRQF_SHARED, dev->name, dev);
	if (unlikely(ret < 0)) {
		netdev_err(priv->dev, "%s: ERROR: allocating the IRQ %d (%d) fail\n",
				__func__, dev->irq, ret);
		goto irq_error;
	} else {
		cpumask_clear(&mask);
		cpumask_set_cpu(1, &mask);
		ret = irq_set_affinity_hint(dev->irq, &mask);
		printk("%s: irq request success\n", __func__);
	}
    clear_bit(BSTVMAC_DOWN, &priv->state);
	bstvmac_init_rxmem(priv);
	bstvmac_enable_all_queues(priv);
	/* Start the ball rolling... */
	bstvmac_start_all_dma(priv);
	netif_tx_start_all_queues(priv->dev);
    	/* Send MAC addr to sw  */
#if defined(CONFIG_BST_C1200_DB)
	bstvmac_syncmac_to_switch(priv, DB_MAC_ADDR_FLAG);
#elif defined(CONFIG_BST_C1200_IVI)
	bstvmac_syncmac_to_switch(priv, IVI_MAC_ADDR_FLAG);
#endif
	netif_carrier_on(priv->dev);
	set_bit(BSTVMAC_RUNNING, &priv->state);
    hrtimer_start(&priv->rx_queue[0].rxtimer2,
        BSTVMAC_COAL_TIMER(10000),
        HRTIMER_MODE_REL);
    // hrtimer_start(&priv->rx_queue[0].rxtimer,
    //         BSTVMAC_COAL_TIMER(200),
    //         HRTIMER_MODE_REL);
    //queue_delayed_work_on(0, priv->rxmem, &priv->dwork, msecs_to_jiffies(10));
	return 0;

irq_error:
	for (chan = 0; chan < priv->plat->tx_queues_to_use; chan++)
		hrtimer_cancel(&priv->tx_queue[chan].txtimer);
init_error:
	free_dma_desc_resources(priv);
dma_desc_error:
	return ret;
}

static int bstvmac_softreset_open(struct net_device *dev)
{
	int ret, chan;
	struct cpumask mask;
	struct bstvmac_priv *priv = netdev_priv(dev);
	int queue;
	u32 rx_count = priv->plat->rx_queues_to_use;
	/* Extra statistics */
	memset(&priv->xstats, 0, sizeof(struct bstvmac_extra_stats));

	priv->dma_buf_sz = BSTVMAC_ALIGN(buf_sz);
	
	if (!priv->dma_tx_size)
		priv->dma_tx_size = DMA_TX_SIZE;
	if (!priv->dma_rx_size)
		priv->dma_rx_size = DMA_RX_SIZE;

	ret = alloc_dma_desc_resources(priv);
	if (ret < 0) {
		netdev_err(priv->dev, "%s: DMA descriptors allocation failed\n",
			   __func__);
		goto dma_desc_error;
	}

	ret = init_dma_desc_rings(dev, GFP_KERNEL);
	if (ret < 0) {
		netdev_err(priv->dev,
			   "%s: DMA descriptors initialization failed\n",
			   __func__);
		goto init_error;
	}
	for (queue = 0; queue < rx_count; queue++) {
		struct bstvmac_rx_queue *rx_q = &priv->rx_queue[queue];
		rx_q->cur_rx = 0;
		rx_q->dirty_rx = 0;
		bstvmac_init_rx_descriptors(priv, queue);
		/* Setup the chained descriptor addresses */
		bstvmac_mode_init(priv, rx_q->dma_bd_rx,
			rx_q->dma_bd_rx_phy, DMA_RX_SIZE, 0);
	}

	ret = bstvmac_hw_setup(dev, true);
	if (ret < 0) {
		netdev_err(priv->dev, "%s: Hw setup failed\n", __func__);
		goto init_error;
	}

	bstvmac_init_coalesce(priv);

	/* Request the Tx/Rx chan IRQ lines */
	ret = request_irq(dev->irq, bstvmac_interrupt,
			IRQF_SHARED, dev->name, dev);
	if (unlikely(ret < 0)) {
		netdev_err(priv->dev, "%s: ERROR: allocating the IRQ %d (%d) fail\n",
				__func__, dev->irq, ret);
		goto irq_error;
	} else {
		cpumask_clear(&mask);
		cpumask_set_cpu(1, &mask);
		ret = irq_set_affinity_hint(dev->irq, &mask);
		printk("%s: irq request success\n", __func__);
	}
	clear_bit(BSTVMAC_DOWN, &priv->state);
	bstvmac_init_rxmem(priv);
	bstvmac_enable_all_queues(priv);
	/* Start the ball rolling... */
	bstvmac_start_all_dma(priv);
	netif_tx_start_all_queues(priv->dev);
	netif_carrier_on(priv->dev);
	set_bit(BSTVMAC_RUNNING, &priv->state);
	hrtimer_start(&priv->rx_queue[0].rxtimer2,
        BSTVMAC_COAL_TIMER(10000),
        HRTIMER_MODE_REL);
    // hrtimer_start(&priv->rx_queue[0].rxtimer,
    //         BSTVMAC_COAL_TIMER(200),
    //         HRTIMER_MODE_REL);
    //queue_delayed_work_on(0, priv->rxmem, &priv->dwork, msecs_to_jiffies(10));
	return 0;

irq_error:
	for (chan = 0; chan < priv->plat->tx_queues_to_use; chan++)
		hrtimer_cancel(&priv->tx_queue[chan].txtimer);
init_error:
	free_dma_desc_resources(priv);
dma_desc_error:
	return ret;
}
/**
 *  bstvmac_init_frame - construct skb pkt contents
 *  @priv: private structure
 *  Description:
 *  This is the construct skb pkt contents driver.
 */
struct sk_buff *bstvmac_init_frame(struct bstvmac_priv *priv)
{
	struct sk_buff *skb = NULL;
	struct net_device *dev;
	struct ethhdr *ethdr;
	int length;

	char smac[6] = {0x10, 0x21, 0x32, 0x43, 0x54, 0x65};
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
	skb_push(skb, sizeof(data));
	memcpy(skb->data, data, sizeof(data));
	skb->len = sizeof(data);
	skb_push(skb, sizeof(struct ethhdr));
	ethdr = (struct ethhdr *)skb->data;
	skb->len += sizeof(struct ethhdr);

	memcpy(ethdr->h_source, smac, ETH_ALEN);
	memcpy(ethdr->h_dest, priv->dev->dev_addr, ETH_ALEN);

	ethdr->h_proto = htons(ETH_P_IP);
	skb->protocol = htons(ETH_P_IP);
	skb->pkt_type =  PACKET_OTHERHOST;
	skb->dev = dev;

	return skb;
}

/**
 *  bstvmac_hif_sw_reset - clear hif prefetch fifo contents
 *  @priv: private structure
 *  Description:
 *  This is the clear hif prefetch fifo contents driver.
 */
void bstvmac_prefetch_fifo_clear(struct bstvmac_priv *priv, u32 chan)
{
	unsigned int i;
	netdev_tx_t ret;
	struct sk_buff *frame[BSTVMAC_SEND_FRAME_NUM] = {NULL};

	for(i = 0; i < BSTVMAC_SEND_FRAME_NUM; i++) {
        if( (test_bit(BSTVMAC_RX_FIFO_CLEAR, &priv->state) || test_bit(BSTVMAC_RX_FIFO_CLEAR_DONE, &priv->state)) && \
        bstvmac_rx_fifo_clear_status(priv, priv->chanl_start_addr, chan)){
            break;
        }
		frame[i] = bstvmac_init_frame(priv);
		if (frame[i]) {
			__netif_tx_lock_bh(netdev_get_tx_queue(priv->dev, chan));
			ret = bstvmac_xmit(frame[i], priv->dev);
			if (ret == NETDEV_TX_BUSY)
				dev_kfree_skb_any(frame[i]);
			__netif_tx_unlock_bh(netdev_get_tx_queue(priv->dev, chan));
		}
	}
}

/**
 *  bstvmac_hif_sw_reset - reset hif and return it to initial state
 *  @priv: private structure
 *  Description:
 *  This is for return hif to initial state.
 */
int bstvmac_hif_sw_reset(struct bstvmac_priv *priv)
{
	struct dma_bd_desc *p;
	struct bstvmac_rx_queue *rx_q;
	int i = 0, ret = -1, wait_cnt = 5;
	bool tx_cleared = false, rx_cleared = false;
	u32 chan = 0, tx_clear = 0, rx_clear = 0, all_txq_clear = 0, all_rxq_clear = 0;

	if (test_bit(BSTVMAC_RUNNING, &priv->state))
		clear_bit(BSTVMAC_RUNNING, &priv->state);
	else
		return -2;

	for (i = 0; i < priv->plat->tx_queues_to_use; i++)
		all_txq_clear |= (0x1 << i);

	for (i = 0; i < priv->plat->rx_queues_to_use; i++)
		all_rxq_clear |= (0x1 << i);

	set_bit(BSTVMAC_RX_FIFO_CLEAR, &priv->state);

	for (i = 0; i < wait_cnt; i++) {
		for (chan = 0; chan < priv->plat->rx_queues_to_use; chan++) {
			rx_q = &priv->rx_queue[chan];
			p = rx_q->dma_bd_rx;
			bstvmac_set_rx_owner(priv, p, false);
			bstvmac_prefetch_fifo_clear(priv, chan);
			rx_cleared = bstvmac_rx_fifo_clear_status(priv,
						priv->chanl_start_addr, chan);
			if (rx_cleared) {
				rx_clear |= (0x1 << chan);
			} else {
				rx_clear &= ~(0x1 << chan);
				bstvmac_recover_dma_irq(priv, priv->chanl_start_addr, chan);
			}
		}

		if ((rx_clear == all_rxq_clear) && test_bit(BSTVMAC_RX_FIFO_CLEAR_DONE, &priv->state))
			break;
        else {
            printk("rx_phy:%llx, rx_wr_phy:%llx, curr:%d, dirty:%d\n", rx_q->dma_bd_rx_phy, rx_q->dma_wrbd_rx_phy, rx_q->cur_rx, rx_q->dirty_rx);
            printk("HIF_RX_PACKET_DROP_EN: %x\n", readl(priv->hif_base_addr + HIF_RX_PACKET_DROP_EN));
            printk("HIF_RX_PACKET_DROP_CNT: %x\n", readl(priv->hif_base_addr + HIF_RX_PACKET_DROP_CNT));
            printk("HIF_RX_PACKET_DROP_CNT: %x\n", readl(priv->hif_base_addr + HIF_RX_PACKET_DROP_CNT));
            printk("HIF_RX_PACKET_DROP_CNT_CH: %x\n", readl(priv->chanl_start_addr + HIF_RX_PACKET_DROP_CNT_CH(0)));
            printk("HIF_BDP_CH_RX_FIFO_CNT: %x\n", readl(priv->chanl_start_addr + HIF_BDP_CH_RX_FIFO_CNT(0)));
            printk("HIF_RX_DMA_STATUS_0_CH: %x\n", readl(priv->chanl_start_addr + HIF_RX_DMA_STATUS_0_CH(0)));
            printk("HIF_RX_PKT_CNT0_CH: %x\n", readl(priv->chanl_start_addr + HIF_RX_PKT_CNT0_CH(0)));
            printk("HIF_RX_PKT_CNT1_CH: %x\n", readl(priv->chanl_start_addr + HIF_RX_PKT_CNT1_CH(0)));
            printk("HIF_RX_RD_CURR_BD_LOW_ADDR_CH: %x\n", readl(priv->chanl_start_addr + HIF_RX_RD_CURR_BD_LOW_ADDR_CH(0)));
            printk("HIF_RX_BDP_RD_LOW_ADDR_CH: %x\n", readl(priv->chanl_start_addr + HIF_RX_BDP_RD_LOW_ADDR_CH(0)));
            printk("HIF_RX_WR_CURR_BD_LOW_ADDR_CH: %x\n", readl(priv->chanl_start_addr + HIF_RX_WR_CURR_BD_LOW_ADDR_CH(0)));
            printk("HIF_RX_BDP_WR_LOW_ADDR_CH: %x\n", readl(priv->chanl_start_addr + HIF_RX_BDP_WR_LOW_ADDR_CH(0)));
        }
        msleep(20);
	}

	printk("%s: rx fifo clear done (rx_clear = %d, all_rxq_clear = %d)\n",
			 __func__, rx_clear, all_rxq_clear);
#if defined(CONFIG_BST_C1200_DB)
	bstvmac_syncmac_to_switch(priv, DB_MAC_STOP_FLAG);
#elif defined(CONFIG_BST_C1200_IVI)
	bstvmac_syncmac_to_switch(priv, IVI_MAC_STOP_FLAG);
#endif
	clear_bit(BSTVMAC_RX_FIFO_CLEAR_DONE, &priv->state);
	clear_bit(BSTVMAC_RX_FIFO_CLEAR, &priv->state);

	set_bit(BSTVMAC_TX_FIFO_CLEAR, &priv->state);

	for (i = 0; i < wait_cnt; i++) {
		for (chan = 0; chan < priv->plat->tx_queues_to_use; chan++) {
			bstvmac_prefetch_fifo_clear(priv, chan);
			tx_cleared = bstvmac_tx_fifo_clear_status(priv,
						priv->chanl_start_addr, chan);
			if (tx_cleared) {
				tx_clear |= (0x1 << chan);
			} else {
				tx_clear &= ~(0x1 << chan);
				bstvmac_recover_dma_irq(priv, priv->chanl_start_addr, chan);
			}
		}

		if (tx_clear == all_txq_clear)
			break;

		msleep(20);
	}

	printk("%s: tx fifo clear done (tx_clear = %d, all_txq_clear = %d)\n",
			 __func__, tx_clear, all_txq_clear);

	if ((rx_clear == all_rxq_clear) && (tx_clear == all_txq_clear))
		ret = 0;

	clear_bit(BSTVMAC_TX_FIFO_CLEAR, &priv->state);

	return ret;
}

/**
 *  bstvmac_release - close entry point of the driver
 *  @dev : device pointer.
 *  Description:
 *  This is the stop entry point of the driver.
 */
static int bstvmac_release(struct net_device *dev)
{
	struct bstvmac_priv *priv = netdev_priv(dev);
	int  ret, chan;
    if(!test_bit(BSTVMAC_RESETTING, &priv->state))
    {
        ret = bstvmac_hif_sw_reset(priv);
        printk("%s: hif prefetch fifo clear done (ret = %d)\n", __func__, ret);
    }
    set_bit(BSTVMAC_DOWN, &priv->state);
	bstvmac_disable_all_queues(priv);

	for (chan = 0; chan < priv->plat->tx_queues_to_use; chan++)
		hrtimer_cancel(&priv->tx_queue[chan].txtimer);
    for (chan = 0; chan < priv->plat->rx_queues_to_use; chan++)
    {
		hrtimer_cancel(&priv->rx_queue[chan].rxtimer);
        hrtimer_cancel(&priv->rx_queue[chan].rxtimer2);
    }
	/* Stop TX/RX DMA */
	bstvmac_stop_all_dma(priv);

	/* Free the IRQ lines */
	irq_set_affinity_hint(dev->irq, NULL);
	if(dev->irq != 0)
	{
		free_irq(dev->irq, dev);
	}

	/* Release and free the Rx/Tx resources */
	uninit_dma_desc_rings(dev);
	free_dma_desc_resources(priv);
	free_dma_rx_mem_res(priv);

	priv->tx_fifo_clear = false;
	priv->rx_fifo_clear = false;
	netif_carrier_off(dev);
	printk("%s done\n", __func__);

	return 0;
}

static int bstvmac_softreset_release(struct net_device *dev)
{
	struct bstvmac_priv *priv = netdev_priv(dev);
	int   chan;
	// bstvmac_disable_dma_irq(priv, priv->chanl_start_addr, chan, 1, 1);
	set_bit(BSTVMAC_DOWN, &priv->state);
	/* Free the IRQ lines */
	irq_set_affinity_hint(dev->irq, NULL);
	if(dev->irq != 0)
	{
		free_irq(dev->irq, dev);
	}

	bstvmac_disable_all_queues(priv);

	for (chan = 0; chan < priv->plat->tx_queues_to_use; chan++)
		hrtimer_cancel(&priv->tx_queue[chan].txtimer);
    for (chan = 0; chan < priv->plat->rx_queues_to_use; chan++)
    {
		hrtimer_cancel(&priv->rx_queue[chan].rxtimer);
        hrtimer_cancel(&priv->rx_queue[chan].rxtimer2);
    }
	/* Stop TX/RX DMA */
	bstvmac_stop_all_dma(priv);


	/* Release and free the Rx/Tx resources */
	uninit_dma_desc_rings(dev);
	free_dma_desc_resources(priv);
	free_dma_rx_mem_res(priv);

	priv->tx_fifo_clear = false;
	priv->rx_fifo_clear = false;
	netif_carrier_off(dev);
	printk("%s done\n", __func__);

	return 0;
}

static void bstvmac_flush_tx_descriptors(struct bstvmac_priv *priv, int queue)
{
	struct bstvmac_tx_queue *tx_q = &priv->tx_queue[queue];
	int desc_size;

	desc_size = sizeof(struct dma_bd_desc);

	/* The own bit must be the latest setting done when prepare the
	 * descriptor and then barrier is needed to make sure that
	 * all is coherent before granting the DMA engine.
	 */
	wmb();

	tx_q->tx_tail_addr = tx_q->dma_bd_tx_phy + (tx_q->cur_tx * desc_size);
}

static netdev_tx_t bstvmac_do_xmit(struct sk_buff *skb, struct net_device *dev)
{
	int i, is_jumbo = 0;
	unsigned int first_entry, tx_packets;
	struct bstvmac_priv *priv = netdev_priv(dev);
	unsigned int nopaged_len = skb_headlen(skb);
	u32 queue = skb_get_queue_mapping(skb);
	int nfrags = skb_shinfo(skb)->nr_frags;
	int entry, first_tx;
	struct dma_bd_desc *desc, *first;
	struct bstvmac_tx_queue *tx_q;
	dma_addr_t des;
	bool set_ic, last_bd;

	tx_q = &priv->tx_queue[queue];
	first_tx = tx_q->cur_tx;

	if (unlikely(bstvmac_tx_avail(priv, queue) < nfrags + 1)) {
		if (!netif_tx_queue_stopped(netdev_get_tx_queue(dev, queue))) {
			netif_tx_stop_queue(netdev_get_tx_queue(priv->dev,
								queue));
			/* This is a hard error, log it. */
			netdev_dbg(priv->dev, "%s: Tx Ring full when queue awake (dirty_tx=%d cur_tx=%d cnt=%d)\n",
				__func__, priv->tx_queue[queue].dirty_tx, priv->tx_queue[queue].cur_tx,
				bstvmac_tx_avail(priv, queue));
		}
		return NETDEV_TX_BUSY;
	}

	entry = tx_q->cur_tx;
	first_entry = entry;
	WARN_ON(tx_q->tx_skbuff[first_entry]);

	desc = tx_q->dma_bd_tx + entry;
	first = desc;

	if (!entry && test_bit(BSTVMAC_TX_FIFO_CLEAR, &priv->state)) {
		priv->tx_fifo_clear = true;
		printk("%s: tx fifo clear start\n", __func__);
	}

	for (i = 0; i < nfrags; i++) {
		const skb_frag_t *frag = &skb_shinfo(skb)->frags[i];
		int len = skb_frag_size(frag);
		bool last_segment = (i == (nfrags - 1));

		entry = BSTVMAC_GET_ENTRY(entry, DMA_TX_SIZE);
		WARN_ON(tx_q->tx_skbuff[entry]);
		if (netif_msg_tx_err(priv)) {
			pr_err("nfrags %d i %d frag_len %d\n", nfrags, i, len);
		}

		desc = tx_q->dma_bd_tx + entry;
		des = skb_frag_dma_map(priv->device, frag, 0, len,
				       DMA_TO_DEVICE);
		if (dma_mapping_error(priv->device, des))
			goto dma_map_err;	/* should reuse desc w/o issues */
		if (des <= 0xffffffff)
			pr_err("%s line %d des 0x%llx\n", __func__, __LINE__, des);

		tx_q->tx_skbuff_dma[entry].buf = des;

		bstvmac_set_desc_addr(priv, desc, des);

		tx_q->tx_skbuff_dma[entry].map_as_page = true;
		tx_q->tx_skbuff_dma[entry].len = len;
		tx_q->tx_skbuff_dma[entry].last_segment = last_segment;
		tx_q->tx_skbuff_dma[entry].buf_type = BSTVMAC_TXBUF_T_SKB;
		if (entry == (DMA_TX_SIZE - 1))
			last_bd = true;
		else
			last_bd = false;

		mb();
		/* Prepare the descriptor and set the own bit too */
		bstvmac_prepare_tx_desc(priv, desc, len, last_bd, priv->tx_fifo_clear ? false : true,
				 last_segment, tx_q->seq_num);
		tx_q->seq_num = tx_q->seq_num + 1;
	}

	/* Only the last descriptor gets to point to the skb. */
	tx_q->tx_skbuff[entry] = skb;
	tx_q->tx_skbuff_dma[entry].buf_type = BSTVMAC_TXBUF_T_SKB;

	/* According to the coalesce parameter the IC bit for the latest
	 * segment is reset and the timer re-started to clean the tx status.
	 * This approach takes care about the fragments: desc is the first
	 * element in case of no SG.
	 */
	tx_packets = (entry + 1) - first_tx;
	tx_q->tx_count_frames += tx_packets;

	if (!priv->tx_coal_frames[queue])
		set_ic = false;
	else if (tx_packets > priv->tx_coal_frames[queue])
		set_ic = true;
	else if ((tx_q->tx_count_frames %
		  priv->tx_coal_frames[queue]) < tx_packets)
		set_ic = true;
	else
		set_ic = false;
    //printk("tx_q->tx_count_frames:%d,%d,%d", tx_q->tx_count_frames, priv->tx_coal_frames[queue], tx_packets);
	if (set_ic) {
		desc = &tx_q->dma_bd_tx[entry];
		tx_q->tx_count_frames = 0;
		bstvmac_set_tx_ic(priv, desc);
		priv->xstats.tx_set_ic_bit++;
	}

	/* We've used all descriptors we need for this skb, however,
	 * advance cur_tx so that it references a fresh descriptor.
	 * ndo_start_xmit will fill this descriptor the next time it's
	 * called and bstvmac_tx_clean may clean up to this descriptor.
	 */
	entry = BSTVMAC_GET_ENTRY(entry, DMA_TX_SIZE);
	tx_q->cur_tx = entry;

	if (netif_msg_tx_err(priv)) {
		netdev_err(priv->dev,
			   "%s: curr=%d dirty=%d f=%d, e=%d, first=%p skb len %d",
			   __func__, tx_q->cur_tx, tx_q->dirty_tx, first_entry,
			   entry, first, skb->len);
	}
	if (netif_msg_tx_queued(priv)) {
		netdev_err(priv->dev, ">>> frame to be transmitted: ");
		print_pkt(skb->data, skb->len);
	}

	if (unlikely(bstvmac_tx_avail(priv, queue) <= (MAX_SKB_FRAGS + 1))) {
		if (!(test_bit(BSTVMAC_RX_FIFO_CLEAR, &priv->state))) {
			netdev_dbg(priv->dev, "%s: stop transmitted packets\n", __func__);
			netif_tx_stop_queue(netdev_get_tx_queue(priv->dev, queue));
		}
	}

	dev->stats.tx_bytes += skb->len;

	/* Ready to fill the first descriptor and set the OWN bit w/o any
	 * problems because all the descriptors are actually ready to be
	 * passed to the DMA engine.
	 */
	if (likely(!is_jumbo)) {
		bool last_segment = (nfrags == 0);

		des = dma_map_single(priv->device, skb->data,
				     nopaged_len, DMA_TO_DEVICE);
		if (dma_mapping_error(priv->device, des))
			goto dma_map_err;
		if (des <= 0xffffffff)
			pr_err("%s line %d des 0x%llx\n", __func__, __LINE__, des);

		tx_q->tx_skbuff_dma[first_entry].buf = des;
		tx_q->tx_skbuff_dma[first_entry].buf_type = BSTVMAC_TXBUF_T_SKB;
		tx_q->tx_skbuff_dma[first_entry].map_as_page = false;

		bstvmac_set_desc_addr(priv, first, des);

		tx_q->tx_skbuff_dma[first_entry].len = nopaged_len;
		tx_q->tx_skbuff_dma[first_entry].last_segment = last_segment;
		if (first_entry == (DMA_TX_SIZE - 1))
			last_bd = true;
		else
			last_bd = false;

		/* Prepare the first descriptor setting the OWN bit too */
		bstvmac_prepare_tx_desc(priv, first, nopaged_len, last_bd, false,
				 last_segment, tx_q->seq_num);
		tx_q->seq_num = tx_q->seq_num + 1;
	}

	mb();
	bstvmac_set_tx_owner(priv, first, priv->tx_fifo_clear ? false : true);
	netdev_tx_sent_queue(netdev_get_tx_queue(dev, queue), skb->len);
	bstvmac_flush_tx_descriptors(priv, queue);
	bstvmac_tx_timer_arm(priv, queue);

	return NETDEV_TX_OK;

dma_map_err:
	netdev_err(priv->dev, "Tx DMA map failed\n");
	dev_kfree_skb(skb);
	priv->dev->stats.tx_dropped++;
	return NETDEV_TX_OK;
}

static void bstvmac_prepare_tx_header(struct sk_buff *skb, unsigned int port)
{
	struct tx_header *txhdr;

	txhdr = (struct tx_header *)skb->data;
	memset(txhdr, 0, BSTVMAC_TX_HDRLEN);

	if (0 != port) {
		txhdr->txport_map = 1 << port;
		txhdr->txport_map |= 1 << 6;
		txhdr->txport_map |= 1 << 0;
		txhdr->txport_map |= 1 << 1;
		txhdr->txport_map |= 1 << 7;
		txhdr->txport_map |= 1 << 8;
		txhdr->txport_map |= 1 << 9;
		txhdr->seq_num = 0xaad8;
	}
}

/**
 *  bstvmac_xmit - Tx entry point of the driver
 *  @skb : the socket buffer
 *  @dev : device pointer
 *  Description : this is the tx entry point of the driver.
 *  It programs the chain or the ring and supports oversized frames
 *  and SG feature.
 */
static netdev_tx_t bstvmac_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct bstvmac_priv *priv = netdev_priv(dev);
	struct ethhdr *eth_hdr = (struct ethhdr *)skb->data;
	struct sk_buff *skb_p = NULL;

	if (test_bit(BSTVMAC_DOWN, &priv->state)) {
		return NETDEV_TX_BUSY;
	}

	if (skb_headroom(skb) < BSTVMAC_TX_HDRLEN) {
		skb = skb_realloc_headroom(skb, BSTVMAC_HEADROOM);
		if (skb_headroom(skb) < BSTVMAC_TX_HDRLEN) {
			pr_err("%s: Not enough headroom for TX HDR, skb headroom = %d\n",
					 __func__, skb_headroom(skb));
			dev_kfree_skb_any(skb);
			return NETDEV_TX_OK;
		}
	}

	skb_push(skb, BSTVMAC_TX_HDRLEN);
	if (unlikely(is_multicast_ether_addr(eth_hdr->h_dest))) {
		skb_p = pskb_copy(skb, GFP_ATOMIC);
		if (!skb_p) {
			pr_err("%s copy skb failed for broadcast.\n", __func__);
			return NETDEV_TX_BUSY;
		}
		eth_hdr = (struct ethhdr *)(skb_p->data + BSTVMAC_TX_HDRLEN);
#if defined(CONFIG_BST_C1200_DB)
		memcpy(eth_hdr->h_dest, db_dest, ETH_ALEN);
#elif defined(CONFIG_BST_C1200_IVI)
		memcpy(eth_hdr->h_dest, ivi_dest, ETH_ALEN);
#endif
		bstvmac_prepare_tx_header(skb_p, 0);
		bstvmac_do_xmit(skb_p, dev);
	}

	bstvmac_prepare_tx_header(skb, 5);
	return bstvmac_do_xmit(skb, dev);

}

/**
 * bstvmac_rx_refill - refill used skb preallocated buffers
 * @priv: driver private structure
 * @queue: RX queue index
 * Description : this is to reallocate the skb for the reception process
 * that is based on zero-copy.
 */
static inline void bstvmac_rx_refill(struct bstvmac_priv *priv, u32 queue)
{
	struct bstvmac_rx_queue *rx_q = &priv->rx_queue[queue];
	unsigned int entry = rx_q->dirty_rx;
	int bus_id = priv->plat->bus_id, cpuid;
	struct dma_bd_desc *p;
	int index, dirty;
	struct sk_buff_head *list;
	
	dirty = bstvmac_rx_dirty(priv, queue);
	index = bus_id * BSTVMAC_RXCHAN_NUM + queue;
	list = vmac_delivery_skblist[index];

	while (dirty-- > 0) {
		p = rx_q->dma_bd_rx + entry;
		bstvmac_set_bd_rx_seqnum(priv, p, rx_q->seq_num);
		rx_q->seq_num++;
		entry = BSTVMAC_GET_ENTRY(entry, DMA_RX_SIZE);

		rx_q->rx_count_frames++;
		rx_q->rx_count_frames += priv->rx_coal_frames[queue];
		if (rx_q->rx_count_frames > priv->rx_coal_frames[queue])
				rx_q->rx_count_frames = 0;
	}

	rx_q->dirty_rx = entry;
	if (skb_queue_len(list) < BSTVMAC_RXMEM_THRE) {
		cpuid = (num_online_cpus() > 1) ? BSTMAC_TXWQ_DEF_CPU : 1;
		queue_work_on(cpuid, priv->rxmem, &priv->mem_mgmt_work); //eth0:cpu1 eth1:cpu0
	}
}

/**
 * bstvmac_rx_func - manage the receive process
 * @priv: driver private structure
 * @limit: napi bugget
 * @queue: RX queue index.
 * Description :  this the function called by the napi poll method.
 * It gets all the frames inside the ring.
 */
static int bstvmac_rx_func(struct bstvmac_priv *priv, int limit, u32 queue)
{
	struct bstvmac_rx_queue *rx_q = &priv->rx_queue[queue];
	struct bstvmac_channel *ch = &priv->rx_channel[queue];
	unsigned int next_entry = rx_q->cur_rx;
	int bus_id = priv->plat->bus_id;
	unsigned int count = 0, curr_wb_addr, init_wb_addr;
	struct sk_buff *skb;
	struct sk_buff *new_skb;
	int index, frame_len, i;
	struct sk_buff_head *list;
	unsigned int tmp_next_entry;

	if (test_bit(BSTVMAC_DOWN, &priv->state))
		goto end;
	else
		rx_q->run_status = 1;

	index = bus_id * BSTVMAC_RXCHAN_NUM + queue;
	list = vmac_delivery_skblist[index];

	init_wb_addr = readl(priv->chanl_start_addr + HIF_RX_BDP_WR_LOW_ADDR_CH(queue));
	while (count < limit) {
		int entry, status;
		struct dma_bd_desc *p, *pbd;
		struct dma_wrbd_desc *wp;

		entry = next_entry;
		p = rx_q->dma_bd_rx + entry;
		wp = rx_q->dma_wrbd_rx + entry;
		/* read the status of the incoming frame */
		status = bstvmac_rx_status(priv, priv->chanl_start_addr,
					   queue, entry, wp);

		/* check if managed by the DMA otherwise go ahead */
		if (unlikely(status & no_frame))
		{
			struct dma_wrbd_desc *tmp_wp;
			int tmp_status;
			tmp_next_entry = BSTVMAC_GET_ENTRY(rx_q->cur_rx, DMA_RX_SIZE);
			tmp_wp = rx_q->dma_wrbd_rx + tmp_next_entry;
			tmp_status = bstvmac_rx_status(priv, priv->chanl_start_addr,
					   queue, tmp_next_entry, tmp_wp);
			if(tmp_status & no_frame)
				break;
		}
		
		if (perf_debug)
			udelay(perf_debug);

		if (!test_bit(BSTVMAC_RX_FIFO_CLEAR, &priv->state))
			bstvmac_clear_status(priv, wp);

		rx_q->cur_rx = BSTVMAC_GET_ENTRY(rx_q->cur_rx, DMA_RX_SIZE);
		next_entry = rx_q->cur_rx;

		if (test_bit(BSTVMAC_RX_FIFO_CLEAR, &priv->state)) {
			curr_wb_addr = readl(priv->chanl_start_addr + HIF_RX_WR_CURR_BD_LOW_ADDR_CH(0));
			if (curr_wb_addr == init_wb_addr && !test_bit(BSTVMAC_RX_FIFO_CLEAR_DONE, &priv->state)) {
				printk("%s: desc en clear, curr_wb_addr = 0x%x\n", __func__,
					readl(priv->chanl_start_addr + HIF_RX_WR_CURR_BD_LOW_ADDR_CH(0)));
				for (i = 0; i < DMA_RX_SIZE; i++) {
					pbd = rx_q->dma_bd_rx + i;
					bstvmac_set_rx_owner(priv, pbd, false);
				}
				set_bit(BSTVMAC_RX_FIFO_CLEAR_DONE, &priv->state);
			}
			count++;
			continue;
		}

		frame_len = bstvmac_get_rx_frame_len(priv, wp);
		/*  If frame length is greater than skb buffer size
		*  (preallocated during init) then the packet is
		*  ignored
		*/
		if (frame_len > priv->dma_buf_sz) {
			if (net_ratelimit())
				netdev_err(priv->dev,
						"len %d larger than size (%d)\n",
						frame_len, priv->dma_buf_sz);
			dma_unmap_single(priv->device,
				rx_q->rx_skbuff_dma[entry],
				priv->dma_buf_sz, DMA_FROM_DEVICE);
			priv->dev->stats.rx_length_errors++;
			continue;
		}

		skb = rx_q->rx_skbuff[entry];
		if (unlikely(!skb)) {
			if (net_ratelimit())
				netdev_err(priv->dev,
					"%s: Inconsistent Rx chain\n",
					priv->dev->name);
			priv->dev->stats.rx_fifo_errors++;
			break;
		}
		// dma_unmap_single(priv->device,
		// 	rx_q->rx_skbuff_dma[entry],
		// 	priv->dma_buf_sz, DMA_FROM_DEVICE);
		new_skb = skb_dequeue(list);
		if (unlikely(!new_skb)) {
			new_skb =
				netdev_alloc_skb_ip_align(priv->dev,
								priv->dma_buf_sz);
			if (unlikely(!new_skb)) {
				priv->dev->stats.rx_fifo_errors++;
				continue;
			}
		}
		memcpy(new_skb->data, skb->data, frame_len);
		memset(skb->data, 0, frame_len);
		skb_put(new_skb, frame_len);
		skb_pull(new_skb, BSTVMAC_TX_HDRLEN);

		if (netif_msg_rx_err(priv)) {
			netdev_err(priv->dev,
				   "%s: curr=%d dirty=%d, e=%d, skb data paddr=0x%llx",
				   __func__, rx_q->cur_rx, rx_q->dirty_rx,
				   entry, virt_to_phys((void *)new_skb->data));
		}
		if (netif_msg_pktdata(priv)) {
			pr_err(">>> frame to be transmitted: len %d\n", new_skb->len);
			print_pkt(new_skb->data, new_skb->len);
		}

		new_skb->protocol = eth_type_trans(new_skb, priv->dev);
		//new_skb->ip_summed = CHECKSUM_UNNECESSARY;

		skb_record_rx_queue(new_skb, queue);

#if BSTVMAC_RX_NAPI
		napi_gro_receive(&ch->rnapi, new_skb);
#else
		netif_receive_skb(new_skb);
#endif

		priv->dev->stats.rx_packets++;
		priv->dev->stats.rx_bytes += frame_len;
		count++;
	}

	bstvmac_rx_refill(priv, queue);
	priv->xstats.rx_pkt_n += count;
	rx_q->run_status = 0;

end:
	return count;
}

/**
 *  bstvmac_rx_napi_poll - bstvmac rx poll method (NAPI)
 *  @napi : pointer to the napi structure.
 *  @budget : maximum number of packets that the current CPU can receive from
 *	      all interfaces.
 *  Description :
 *  To look at the incoming frames.
 */

static int bstvmac_rx_napi_poll(struct napi_struct *rnapi, int budget)
{
	struct bstvmac_channel *ch =
	    container_of(rnapi, struct bstvmac_channel, rnapi);
	struct bstvmac_priv *priv = ch->priv_data;
	int work_done, rx_done = 0;
	u32 chan = ch->index;
	ulong flags;

	priv->xstats.rnapi_poll++;


	if (ch->has_rx)
		rx_done = bstvmac_rx_func(priv, budget, chan);
	
	work_done = min(rx_done, budget);

	if (work_done < budget && napi_complete_done(rnapi, work_done)) {
		int stat;

		spin_lock_irqsave(&vmac_irqbits_lock, flags);
		bstvmac_enable_dma_irq(priv, priv->chanl_start_addr, chan, 1, 0);
		spin_unlock_irqrestore(&vmac_irqbits_lock, flags);

		stat = bstvmac_dma_ri_interrupt_status(priv, priv->chanl_start_addr,
						       &priv->xstats, chan);
		if (stat && napi_reschedule(rnapi)) {
			spin_lock_irqsave(&vmac_irqbits_lock, flags);
			bstvmac_disable_dma_irq(priv, priv->chanl_start_addr, chan, 1, 0);
			spin_unlock_irqrestore(&vmac_irqbits_lock, flags);
		} else {
            hrtimer_start(&priv->rx_queue[0].rxtimer,
                BSTVMAC_COAL_TIMER(200),
                HRTIMER_MODE_REL);
            //queue_delayed_work_on(0, priv->rxmem, &priv->dwork, msecs_to_jiffies(10));
        }
	}

	return work_done;
}

static void bstvmac_refill_rxmem(struct bstvmac_priv *priv, int chan)
{
	int cnt, busid, index;
	struct sk_buff *skb;
	dma_addr_t buf;
	int size = priv->dma_buf_sz;
	dma_addr_t *addr;
	struct sk_buff_head *list;

	busid = priv->plat->bus_id;
	index = busid * BSTVMAC_RXCHAN_NUM + chan;
	list = vmac_delivery_skblist[index];
	cnt = 0;

	while (cnt < (BSTVMAC_RXMEM_THRE - 1)) {
		skb = netdev_alloc_skb_ip_align(priv->dev, size);
		if (unlikely(!skb))
			break;

		buf =
		    dma_map_single(priv->device, skb->data, size,
				   DMA_FROM_DEVICE);
		if (dma_mapping_error(priv->device, buf)) {
			netdev_err(priv->dev, "Gmac map list failed\n");
			dev_kfree_skb(skb);
			break;
		}

		addr = (dma_addr_t *)skb->cb;
		*addr = buf;
		dma_wmb();
		skb_queue_tail(list, skb);
		cnt++;
	}
}

static void bstvmac_mem_mgmt_work(struct work_struct *work)
{
	struct bstvmac_priv *priv =
	    container_of(work, struct bstvmac_priv, mem_mgmt_work);
	int chan, busid, index;
	struct sk_buff_head *list;

	priv->xstats.rx_memwork_poll++;
	busid = priv->plat->bus_id;
	set_bit(BSTVMAC_RXMEM_WORK_RUN, &priv->state);
	for (chan = 0; chan < priv->plat->rx_queues_to_use; chan++) {
		index = busid * BSTVMAC_RXCHAN_NUM + chan;
		list = vmac_delivery_skblist[index];
		if (skb_queue_len(list) < BSTVMAC_RXMEM_THRE)
			bstvmac_refill_rxmem(priv, chan);
	}
	clear_bit(BSTVMAC_RXMEM_WORK_RUN, &priv->state);
}
static  bool is_napi_pending(struct napi_struct *napi) {
    return test_bit(NAPI_STATE_SCHED, &napi->state);
}
static enum hrtimer_restart bstvmac_rx_poll_work(struct bstvmac_priv *priv)
{
//    struct delayed_work *dwork1 = to_delayed_work(work);
	// struct bstvmac_priv *priv =
	//     container_of(dwork1, struct bstvmac_priv, dwork);

	struct bstvmac_rx_queue *rx_q = &priv->rx_queue[0];
	struct bstvmac_channel *ch = &priv->rx_channel[0];
	unsigned int next_entry = rx_q->cur_rx;
    int entry, status;
    struct dma_bd_desc *p;
    struct dma_wrbd_desc *wp;
	ulong flags;
    if (priv->rx_channel[0].has_rx && !is_napi_pending(&priv->rx_channel[0].rnapi))
    {
        entry = next_entry;
        p = rx_q->dma_bd_rx + entry;
        wp = rx_q->dma_wrbd_rx + entry;
        /* read the status of the incoming frame */
        status = bstvmac_rx_status(priv, priv->chanl_start_addr,
                        0, entry, wp);

        /* check if managed by the DMA otherwise go ahead */
        if (likely(status & no_frame))
        {
            struct dma_wrbd_desc *tmp_wp;
            int tmp_status;
            int tmp_next_entry = BSTVMAC_GET_ENTRY(rx_q->cur_rx, DMA_RX_SIZE);
            tmp_wp = rx_q->dma_wrbd_rx + tmp_next_entry;
            tmp_status = bstvmac_rx_status(priv, priv->chanl_start_addr,
                        0, tmp_next_entry, tmp_wp);
            if(tmp_status & no_frame) {
                //queue_delayed_work_on(0, priv->rxmem, &priv->dwork, msecs_to_jiffies(10));
//                if(rx_work_mode == 1)
                {
                    hrtimer_start(&priv->rx_queue[0].rxtimer,
                        BSTVMAC_COAL_TIMER(200),
                        HRTIMER_MODE_REL);
                    return HRTIMER_RESTART;
                }
            }
        }
        if (napi_schedule_prep(&ch->rnapi)) {
            spin_lock_irqsave(&vmac_irqbits_lock, flags);
            bstvmac_disable_dma_irq(priv, priv->chanl_start_addr, 0, 1, 0);
            spin_unlock_irqrestore(&vmac_irqbits_lock, flags);
            __napi_schedule(&ch->rnapi);
        }else{
            //printk("%s: napi_schedule_prep fail.\n", __func__);
        }
    }else {
        //printk("%s: is_napi_pending fail.\n", __func__);
    }
    return HRTIMER_NORESTART;
}
static void bstvmac_tx_work(struct work_struct *tx_work)
{
	struct bstvmac_channel *ch =
	    container_of(tx_work, struct bstvmac_channel, tx_work);
	struct bstvmac_priv *priv = ch->priv_data;
	int work_done, tx_done = 0;
	u32 chan = ch->index;
	int stat, cpuid;
	unsigned long flags;
	int budget = DMA_TX_SIZE;
	int rem_cnt = 0;

	priv->xstats.txwork_poll++;

	if (ch->has_tx)
		tx_done = bstvmac_tx_clean(priv, chan, &rem_cnt);

	work_done = min(tx_done, budget);
	if (work_done <= budget) {
        spin_lock_irqsave(&vmac_irqbits_lock, flags);
        bstvmac_enable_dma_irq(priv, priv->chanl_start_addr, chan, 0, 1);
        spin_unlock_irqrestore(&vmac_irqbits_lock, flags);
		stat = bstvmac_dma_ti_interrupt_status(priv, priv->chanl_start_addr,
						    &priv->xstats, chan);
		if (stat & handle_tx) {
			cpuid = (num_online_cpus() > 1) ? BSTMAC_TXWQ_DEF_CPU : 1;
			queue_work_on(cpuid, priv->tx_wq, &ch->tx_work);
            spin_lock_irqsave(&vmac_irqbits_lock, flags);
			bstvmac_disable_dma_irq(priv, priv->chanl_start_addr, chan, 0, 1);
			spin_unlock_irqrestore(&vmac_irqbits_lock, flags);
		} else {
			spin_lock_irqsave(&vmac_irqbits_lock, flags);
			bstvmac_enable_dma_irq(priv, priv->chanl_start_addr, chan, 0, 1);
			spin_unlock_irqrestore(&vmac_irqbits_lock, flags);
		}
	}
}

#if 0
/**
 *  bstvmac_tx_timeout
 *  @dev : Pointer to net device structure
 *  Description: this function is called when a packet transmission fails to
 *   complete within a reasonable time. The driver will mark the error in the
 *   netdev structure and arrange for the device to be reset to a sane state
 *   in order to transmit a new packet.
 */
static void bstvmac_tx_timeout(struct net_device *dev, unsigned int txqueue)
{
	struct bstvmac_priv *priv = netdev_priv(dev);

	bstvmac_global_err(priv);
}
#endif
/**
 *  bstvmac_change_mtu - entry point to change MTU size for the device.
 *  @dev : device pointer.
 *  @new_mtu : the new MTU size for the device.
 *  Description: the Maximum Transfer Unit (MTU) is used by the network layer
 *  to drive packet transmission. Ethernet has an MTU of 1500 octets
 *  (ETH_DATA_LEN). This value can be changed with ifconfig.
 *  Return value:
 *  0 on success and an appropriate (-)ve integer as defined in errno.h
 *  file on failure.
 */
static int bstvmac_change_mtu(struct net_device *dev, int new_mtu)
{
	struct bstvmac_priv *priv = netdev_priv(dev);

	if (netif_running(dev)) {
		netdev_err(priv->dev, "must be stopped to change its MTU\n");
		return -EBUSY;
	}

	if (new_mtu > 1500) {
		netdev_err(priv->dev, "cannot more than 1500\n");
		return -EINVAL;
	}
	dev->mtu = new_mtu;

	netdev_update_features(dev);

	return 0;
}

static int bstvmac_match_ch_irq(struct bstvmac_priv *priv, int irq, struct net_device *ndev)
{
	int chan = -1;
	u32 int_src;

#if defined(CONFIG_BST_C1200_DB)
	int_src = readl(priv->hif_base_addr + HIF_INT_SRC);
#elif defined(CONFIG_BST_C1200_IVI)
	int_src = readl(priv->hif_base_addr + HIF_INT_SRC_REG2);
#endif

	if ((irq == ndev->irq) && (int_src & HIF_CHAN_MASK)) {
		chan = 0;
	}

	return chan;
}


/* interrupt form gmac intr signal:sbd_perch_rx_intr_o[3:0]
 */
static irqreturn_t bstvmac_rx_interrupt(struct net_device *dev, int chan)
{
	struct bstvmac_priv *priv = netdev_priv(dev);
	int status;
	struct bstvmac_channel *ch;
	bool needs_work;
	unsigned long flags;

	if (chan < 0 || chan >= MAX_RX_QUEUES)
		return IRQ_NONE;

	status = bstvmac_dma_ri_interrupt_status(priv, priv->chanl_start_addr,
						 &priv->xstats, chan);

	ch = &priv->rx_channel[chan];
	needs_work = false;

	if ((status & handle_rx) && ch->has_rx)
		needs_work = true;

	if (needs_work && napi_schedule_prep(&ch->rnapi)) {
		spin_lock_irqsave(&vmac_irqbits_lock, flags);
		bstvmac_disable_dma_irq(priv, priv->chanl_start_addr, chan, 1, 0);
		spin_unlock_irqrestore(&vmac_irqbits_lock, flags);
        hrtimer_cancel(&priv->rx_queue[0].rxtimer);
		__napi_schedule(&ch->rnapi);
	}

	return IRQ_HANDLED;
}


/* interrupt form gmac intr signal:sbd_perch_tx_intr_o[3:0]
 */
static irqreturn_t bstvmac_tx_interrupt(struct net_device *dev, int chan)
{
	struct bstvmac_priv *priv = netdev_priv(dev);
	int status, cpuid;
	struct bstvmac_channel *ch;
	bool needs_work;
	unsigned long flags;

	if (chan < 0 || chan >= MAX_TX_QUEUES)
		return IRQ_NONE;

	status = bstvmac_dma_ti_interrupt_status(priv, priv->chanl_start_addr,
						 &priv->xstats, chan);

	ch = &priv->tx_channel[chan];
	needs_work = false;

	if ((status & handle_tx) && ch->has_tx)
		needs_work = true;

	if (needs_work) {
        //printk("tx interrupt");
		spin_lock_irqsave(&vmac_irqbits_lock, flags);
		bstvmac_disable_dma_irq(priv, priv->chanl_start_addr, chan, 0, 1);
		spin_unlock_irqrestore(&vmac_irqbits_lock, flags);

		cpuid = (num_online_cpus() > 1) ? BSTMAC_TXWQ_DEF_CPU : 1;
		queue_work_on(cpuid, priv->tx_wq, &ch->tx_work);
	}

	return IRQ_HANDLED;
}

static irqreturn_t bstvmac_interrupt(int irq, void *dev_id)
{
	int chan;
	struct net_device *dev = (struct net_device *)dev_id;
	struct bstvmac_priv *priv = netdev_priv(dev);

	if (test_bit(BSTVMAC_DOWN, &priv->state))
		return IRQ_NONE;

	chan = bstvmac_match_ch_irq(priv, irq, dev);
	if (chan < 0)
		return IRQ_NONE;

	//tx interrupr handler
	bstvmac_tx_interrupt(dev, chan);

	//rx interrupr handler
	bstvmac_rx_interrupt(dev, chan);

	return IRQ_HANDLED;
}

/**
 *  bstvmac_ioctl - Entry point for the Ioctl
 *  @dev: Device pointer.
 *  @rq: An IOCTL specefic structure, that can contain a pointer to
 *  a proprietary structure used to pass information to the driver.
 *  @cmd: IOCTL command
 *  Description:
 *  Currently it supports the phy_mii_ioctl(...) and HW time stamping.
 */
static int bstvmac_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	int ret = -EOPNOTSUPP;

	if (!netif_running(dev))
		return -EINVAL;

	return ret;
}

static u16 bstvmac_select_queue(struct net_device *dev,
				struct sk_buff *skb, struct net_device *sb_dev)
{
	if (skb_shinfo(skb)->gso_type &
	    (SKB_GSO_TCPV4 | SKB_GSO_TCPV6 | SKB_GSO_UDP_L4))
		/* There is no way to determine the number of TSO
		 * capable Queues. Let's use always the Queue 0
		 * because if TSO is supported then at least this
		 * one will be capable.
		 */
		return 0;

	return netdev_pick_tx(dev, skb, NULL) % dev->real_num_tx_queues;
}

static int bstvmac_set_mac_address(struct net_device *ndev, void *addr)
{
	struct bstvmac_priv *priv = netdev_priv(ndev);
	int ret = 0;

	ret = eth_mac_addr(ndev, addr);
	if (ret)
		return ret;
#if defined(CONFIG_BST_C1200_DB)
	bstvmac_syncmac_to_switch(priv, DB_MAC_ADDR_FLAG);
#elif defined(CONFIG_BST_C1200_IVI)
	bstvmac_syncmac_to_switch(priv, IVI_MAC_ADDR_FLAG);
#endif

	return ret;
}

#ifdef CONFIG_DEBUG_FS
static struct dentry *bstvmac_fs_dir = NULL;

static void debugfs_display_ring(void *head, void *head_phy, int idx, bool bd, struct seq_file *seq)
{
	int i = 0, num = 0;
	struct dma_bd_desc *p = NULL, *p1 = NULL;
	struct dma_wrbd_desc *wp = NULL, *wp1 = NULL;

	if (idx < 0 || idx >= DEBUGFS_RING_SIZE) {
		return;
	} else if (!idx) {
		num = DEBUGFS_RING_SIZE;
	} else {
		num = (idx + DEBUGFS_DUMP_RING_NUM) > DEBUGFS_RING_SIZE ?
				DEBUGFS_RING_SIZE : (idx + DEBUGFS_DUMP_RING_NUM);
	}

	if (bd) {
		p = (struct dma_bd_desc *)((char *)head + idx * sizeof(struct dma_bd_desc));
		p1 = (struct dma_bd_desc *)((char *)head_phy + idx * sizeof(struct dma_bd_desc));
		for (i = idx; i < num; i++) {
			seq_printf(seq, "%d [0x%llx]: 0x%x 0x%x 0x%x 0x%x\n",
					i, (dma_addr_t)p1,
					le32_to_cpu(p->des0),
					le32_to_cpu(p->des1),
					le32_to_cpu(p->des2),
					le32_to_cpu(p->des3));
			p++;
			p1++;
			seq_puts(seq, "\n");
		}
	} else {
		wp = (struct dma_wrbd_desc *)((char *)head + idx * sizeof(struct dma_wrbd_desc));
		wp1 = (struct dma_wrbd_desc *)((char *)head_phy + idx * sizeof(struct dma_wrbd_desc));
		for (i = idx; i < num; i++) {
			seq_printf(seq, "%d [0x%llx]: 0x%x 0x%x\n",
					i, (dma_addr_t)wp1,
					le32_to_cpu(wp->des0),
					le32_to_cpu(wp->des1));
			wp++;
			wp1++;
			seq_puts(seq, "\n");
		}
	}
}

static int bstvmac_debugfs_ring_read(struct seq_file *seq, void *v)
{
	struct net_device *dev = seq->private;
	struct bstvmac_priv *priv = netdev_priv(dev);
	struct bstvmac_rx_queue *rx_q = NULL;
	struct bstvmac_tx_queue *tx_q = NULL;
	u32 rx_count = priv->plat->rx_queues_to_use;
	u32 tx_count = priv->plat->tx_queues_to_use;
	u32 queue;

	if ((dev->flags & IFF_UP) == 0)
		return 0;

	for (queue = 0; queue < tx_count; queue++) {
		tx_q = &priv->tx_queue[queue];

		if (priv->dbgfs_parm[TX_SW_BIT] & TX_SW_DEBUGFS_FLAG) {
			seq_printf(seq, "TX Queue %d: cur_tx %d dirty_tx %d\n",
						queue, tx_q->cur_tx, tx_q->dirty_tx);
		}
		if (priv->dbgfs_parm[TX_SW_BIT] & TX_BD_DEBUGFS_FLAG) {
			seq_puts(seq, "============ Tx BD Descriptor Ring ============\n");
			debugfs_display_ring((void *)tx_q->dma_bd_tx,
						(void *)tx_q->dma_bd_tx_phy,
						priv->dbgfs_parm[TX_BD_BIT], true, seq);
			seq_puts(seq, "===============================================\n\n");
		}
		if (priv->dbgfs_parm[TX_SW_BIT] & TX_WRBD_DEBUGFS_FLAG) {
			seq_puts(seq, "=========== Tx Wrbd Descriptor Ring ===========\n");
			debugfs_display_ring((void *)tx_q->dma_wrbd_tx,
						(void *)tx_q->dma_wrbd_tx_phy,
						priv->dbgfs_parm[TX_WRBD_BIT], false, seq);
			seq_puts(seq, "===============================================\n\n");
		}
	}

	for (queue = 0; queue < rx_count; queue++) {
		rx_q = &priv->rx_queue[queue];
		if (priv->dbgfs_parm[TX_SW_BIT] & RX_SW_DEBUGFS_FLAG) {
			seq_printf(seq, "RX Queue %d: cur_rx %d dirty_rx %d\n",
						queue, rx_q->cur_rx, rx_q->dirty_rx);
		}
		if (priv->dbgfs_parm[TX_SW_BIT] & RX_BD_DEBUGFS_FLAG) {
			seq_puts(seq, "============ Rx BD Descriptor Ring ============\n");
			debugfs_display_ring((void *)rx_q->dma_bd_rx,
						(void *)rx_q->dma_bd_rx_phy,
						priv->dbgfs_parm[RX_BD_BIT], true, seq);
			seq_puts(seq, "===============================================\n\n");
		}
		if (priv->dbgfs_parm[TX_SW_BIT] & RX_WRBD_DEBUGFS_FLAG) {
			seq_puts(seq, "=========== Rx Wrbd Descriptor Ring ===========\n");
			debugfs_display_ring((void *)rx_q->dma_wrbd_rx,
						(void *)rx_q->dma_wrbd_rx_phy,
						priv->dbgfs_parm[RX_WRBD_BIT], false, seq);
			seq_puts(seq, "===============================================\n");
		}
	}

	return 0;
}

static ssize_t bstvmac_debugfs_write(struct file *flip,
			const char __user *buf, size_t count, loff_t *ppos)
{
	struct seq_file *seq = flip->private_data;
	struct net_device *dev = seq->private;
	struct bstvmac_priv *priv = netdev_priv(dev);
	char buffer[DEBUGFS_MAX_BUF], *p, *ep;
    long i = 0, value;

	if (count >= sizeof(buffer)) {
		return -EINVAL;
	}

	if (copy_from_user(buffer, buf, count)) {
        return -EFAULT;
	}

	buffer[count] = '\0';
	memset(priv->dbgfs_parm, 0, sizeof(priv->dbgfs_parm));

	p = buffer;
    while (*p && i < DEBUGFS_MAX_PARAMS) {
        while (*p == ' ' || *p == '\t') {
            p++;
		}

        if (*p == '\0') {
            break;
		}
        value = simple_strtol(p, &ep, 0);
        if (p == ep) {
            break;
		}

        priv->dbgfs_parm[i++] = value;
        p = ep;
    }

	DEBUGFS_PARAM(TX_BD_BIT, DMA_TX_SIZE);
	DEBUGFS_PARAM(TX_WRBD_BIT, DMA_TX_SIZE);
	DEBUGFS_PARAM(RX_BD_BIT, DMA_RX_SIZE);
	DEBUGFS_PARAM(RX_WRBD_BIT, DMA_RX_SIZE);

    printk("vmac dbgfs_parm updated to %d-%d-%d-%d-%d\n", priv->dbgfs_parm[0],
		priv->dbgfs_parm[1], priv->dbgfs_parm[2], priv->dbgfs_parm[3], priv->dbgfs_parm[4]);

    return count;
}

static int bstvmac_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, bstvmac_debugfs_ring_read, inode->i_private);
}

static const struct file_operations bstvmac_rings_status_fops = {
	.owner = THIS_MODULE,
	.open = bstvmac_debugfs_open,
	.write = bstvmac_debugfs_write,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};

static int bstvmac_init_fs(struct net_device *dev)
{
	struct bstvmac_priv *priv = netdev_priv(dev);

	/* Create per netdev entries */
	priv->dbgfs_dir = debugfs_create_dir(dev->name, bstvmac_fs_dir);

	if (!priv->dbgfs_dir || IS_ERR(priv->dbgfs_dir)) {
		netdev_err(priv->dev, "ERROR failed to create debugfs directory\n");
		return -ENOMEM;
	}

	/* Entry to report DMA RX/TX rings */
	priv->dbgfs_rings_status = debugfs_create_file("desc_status", 0444,
				priv->dbgfs_dir, dev, &bstvmac_rings_status_fops);

	if (!priv->dbgfs_rings_status || IS_ERR(priv->dbgfs_rings_status)) {
		netdev_err(priv->dev, "ERROR creating bstgmac ring debugfs file\n");
		debugfs_remove_recursive(priv->dbgfs_dir);
		return -ENOMEM;
	}

	return 0;
}

static void bstvmac_exit_fs(struct net_device *dev)
{
	struct bstvmac_priv *priv = netdev_priv(dev);

	debugfs_remove_recursive(priv->dbgfs_dir);
}
#endif /* CONFIG_DEBUG_FS */

static const struct net_device_ops bstvmac_netdev_ops = {
	.ndo_open = bstvmac_open,
	.ndo_start_xmit = bstvmac_xmit,
	.ndo_stop = bstvmac_release,
	.ndo_change_mtu = bstvmac_change_mtu,
	//.ndo_tx_timeout = bstvmac_tx_timeout,
	.ndo_eth_ioctl = bstvmac_ioctl,
	.ndo_select_queue = bstvmac_select_queue,
	.ndo_set_mac_address = bstvmac_set_mac_address,
};

static void bstvmac_reset_subtask(struct bstvmac_priv *priv)
{
	if (!test_and_clear_bit(BSTVMAC_RESET_REQUESTED, &priv->state))
		return;
	if (test_bit(BSTVMAC_DOWN, &priv->state))
		return;

	netdev_err(priv->dev, "Reset adapter.\n");
	//pr_emerg("[%s]%d.",__func__,__LINE__);
	rtnl_lock();
	netif_trans_update(priv->dev);
	while (test_and_set_bit(BSTVMAC_RESETTING, &priv->state))
		usleep_range(1000, 2000);

	set_bit(BSTVMAC_DOWN, &priv->state);
	dev_close(priv->dev);
	dev_open(priv->dev, NULL);
	clear_bit(BSTVMAC_DOWN, &priv->state);
	clear_bit(BSTVMAC_RESETTING, &priv->state);
	rtnl_unlock();
}

static void bstvmac_service_task(struct work_struct *work)
{
	struct bstvmac_priv *priv = container_of(work, struct bstvmac_priv,
						 service_task);

	bstvmac_reset_subtask(priv);
	clear_bit(BSTVMAC_SERVICE_SCHED, &priv->state);
}

/**
 *  bstvmac_hw_init - Init the MAC device
 *  @priv: driver private structure
 *  Description: this function is to configure the MAC device according to
 *  some platform parameters or the HW capability register. It prepares the
 *  driver to use either ring or chain modes and to setup either enhanced or
 *  normal descriptors.
 */
static int bstvmac_hw_init(struct bstvmac_priv *priv)
{
	int ret;

	/* Initialize HW Interface */
	ret = bstvmac_hwif_init(priv);
	if (ret)
		return ret;

	/* Run HW quirks, if any */
	if (priv->hwif_quirks) {
		ret = priv->hwif_quirks(priv);
		if (ret)
			return ret;
	}

	return 0;
}

static void bstvmac_netif_del_napi(struct bstvmac_priv *priv)
{
	u32 queue, maxq;

	maxq = max(priv->plat->rx_queues_to_use, priv->plat->tx_queues_to_use);
	for (queue = 0; queue < maxq; queue++) {
		struct bstvmac_channel *ch = &priv->rx_channel[queue];
#if BSTVMAC_RX_NAPI
		if (queue < priv->plat->rx_queues_to_use)
			netif_napi_del(&ch->rnapi);
#endif
	}
}

static void bstvmac_reinit_triggered(const uint32_t channel,const uint8_t cmd, void *ext, const ext_info_t *info)
{
	struct bstvmac_priv *priv;

	priv = (struct bstvmac_priv *)ext;
	pr_err("%s: receive vmac reinit broadcast, channel is %d\n", __func__, channel);
	if(0 == cmd)
	{
		rtnl_lock();
		bstvmac_softreset_release(priv->dev);
		rtnl_unlock();
	}
	if(1 == cmd )
	{
		 printk("%s: 1 == cmd .\n", __func__);
		rtnl_lock();
		bstvmac_softreset_open(priv->dev);
		rtnl_unlock();
	}

}

static void bstvmac_reinit_reply(int err, void *ext, const ext_info_t *info)
{
	if (err == 0)
		pr_err("%s: subscribe vmac reinit success\n", __func__);
	else
		pr_err("%s: ubscribe vmac reinit success, ret is %d\n", __func__, err);
}


static void on_server_changed(bool avail, void *ext)
{
    struct bstvmac_priv *priv = (struct bstvmac_priv *)ext;

    if(avail) {
        if(test_bit(BSTVMAC_RESETTING, &priv->state))
        {
            mdelay(200);
            bstvmac_resume(priv->device);
            clear_bit(BSTVMAC_RESETTING, &priv->state);
        }
    }else{
        set_bit(BSTVMAC_RESETTING, &priv->state);
        bstvmac_suspend(priv->device);
    }
}
/**
 * bstvmac_dvr_probe
 * @device: device pointer
 * @plat_dat: platform data pointer
 * @res: bstvmac resource pointer
 * Description: this is the main probe function used to
 * call the alloc_etherdev, allocate the priv structure.
 * Return:
 * returns 0 on success, otherwise errno.
 */
int bstvmac_dvr_probe(struct platform_device *pdev,
		      struct plat_vmacenet_data *plat_dat,
		      struct bstvmac_resources *res)
{
	u32 queue;
	int ret = 0;
	struct device *device;
	struct bstvmac_priv *priv;
	struct net_device *ndev = NULL;

#if defined(CONFIG_BST_C1200_DB)
	client = db_client_init(&client_data);
	if (!client) {
		printk("%s: db client init fail.\n", __func__);
	} else {
		printk("%s: db client init success.\n", __func__);
	}
#elif defined(CONFIG_BST_C1200_IVI)
	client = ivi_client_init(&client_data);
	if (!client) {
		printk("%s: ivi client init fail.\n", __func__);
	} else {
		printk("%s: ivi client init success.\n", __func__);
	}
#endif

	if (client) {
		ret = client->start();
		if (ret < 0) {
			printk("%s: msgbox client start failed\n", __func__);
		} else {
			printk("%s: msgbox client start success\n", __func__);
		}
	}


	ndev = alloc_etherdev_mqs(sizeof(struct bstvmac_priv),
				plat_dat->tx_queues_to_use, plat_dat->rx_queues_to_use);
	if (!ndev)
		return -ENOMEM;

	device = &pdev->dev;
	SET_NETDEV_DEV(ndev, device);

	priv = netdev_priv(ndev);
	priv->device = device;
	priv->dev = ndev;

	bstvmac_set_ethtool_ops(ndev);
	priv->pause = pause;
	priv->plat = plat_dat;
	priv->ioaddr = res->addr;
	priv->dev->base_addr = (unsigned long)res->addr;

	priv->dev->irq = res->irq;
	memcpy(priv->perch_irq, res->perch_irq,
	        sizeof(int) * BSTVMAC_MAX_IRQ_NUM);

	if (!is_zero_ether_addr(res->mac))
		eth_hw_addr_set(priv->dev, res->mac);

	dev_set_drvdata(device, priv->dev);

	ret = of_reserved_mem_device_init(&pdev->dev);
	if (ret) {
		pr_err("%s get vmac reserved memory fail\n", __func__);
		goto error_wq;
	}

	dma_set_mask(priv->device, DMA_BIT_MASK(36));
	dma_set_mask_and_coherent(priv->device, DMA_BIT_MASK(36));

	/* Verify driver arguments */
	bstvmac_verify_args();

	/* Allocate workqueue */
	priv->wq = create_workqueue("vmac");
	if (!priv->wq) {
		printk("%s: failed to create wq workqueue\n", __func__);
		ret = -ENOMEM;
		goto error_wq;
	}

	priv->rxmem = create_workqueue("vmac_rxmem");
	if (!priv->rxmem) {
		printk("%s: failed to create rxmem workqueue\n", __func__);
		ret = -ENOMEM;
		goto error_wq;
	}

	priv->tx_wq = create_workqueue("vmac_tx");
	if (!priv->tx_wq) {
		printk("%s: failed to create tx_wq workqueue\n", __func__);
		ret = -ENOMEM;
		goto error_wq;
	}

	INIT_WORK(&priv->service_task, bstvmac_service_task);
	INIT_WORK(&priv->mem_mgmt_work, bstvmac_mem_mgmt_work);
	/* Init MAC and get the capabilities */
	ret = bstvmac_hw_init(priv);
	if (ret)
		goto error_hw_init;

	bstvmac_check_ether_addr(priv);

	/* Configure real RX and TX queues */
	netif_set_real_num_rx_queues(ndev, priv->plat->rx_queues_to_use);
	netif_set_real_num_tx_queues(ndev, priv->plat->tx_queues_to_use);
	printk("%s rx_queues_to_use=%d, tx_queues_to_use=%d\n", __func__,
			 priv->plat->rx_queues_to_use, priv->plat->tx_queues_to_use);

	ndev->netdev_ops = &bstvmac_netdev_ops;
	ndev->flags &= ~IFF_UP;
	ndev->flags &= ~IFF_MULTICAST;
	ndev->rtnl_link_ops = NULL;
	ndev->needed_headroom = BSTVMAC_HEADROOM;
	//ndev->watchdog_timeo = msecs_to_jiffies(watchdog);

	priv->msg_enable = netif_msg_init(debug, default_msg_level);
	/* MTU range: 46 - hw-specific max */
	ndev->min_mtu = ETH_ZLEN - ETH_HLEN;
	if (priv->vmac_id >= BSTVMAC_CORE_10)
		ndev->max_mtu = JUMBO_LEN;
	else
		ndev->max_mtu = SKB_MAX_HEAD(NET_SKB_PAD + NET_IP_ALIGN);

	/* Will not overwrite ndev->max_mtu if plat->maxmtu > ndev->max_mtu
	 * as well as plat->maxmtu < ndev->min_mtu which is a invalid range.
	 */
	if (priv->plat->maxmtu < ndev->max_mtu &&
	    priv->plat->maxmtu >= ndev->min_mtu)
		ndev->max_mtu = priv->plat->maxmtu;

	printk("%s min_mtu=%d, max_mtu=%d\n", __func__,
			 ndev->min_mtu, ndev->max_mtu);

	/* Setup channels NAPI */
	for (queue = 0; queue < priv->plat->rx_queues_to_use; queue++) {
		struct bstvmac_channel *ch = &priv->rx_channel[queue];
		static struct sk_buff_head *head = NULL;

		ch->priv_data = priv;
		ch->index = queue;

		if (queue < priv->plat->rx_queues_to_use)
			ch->has_rx = true;
#if BSTVMAC_RX_NAPI
		//separate rx tx napi. by well 2020/01/20
		//BSTGMAC_RX_POLL_WEIGHT
		netif_napi_add_weight(ndev, &ch->rnapi,
					bstvmac_rx_napi_poll, DMA_RX_SIZE/16);
#endif
		head = kmalloc(sizeof(*head), GFP_KERNEL);
		if (head) {
			vmac_delivery_skblist[priv->plat->bus_id *
						BSTVMAC_RXCHAN_NUM + queue] = head;
			skb_queue_head_init(head);
		}
	}

	for (queue = 0; queue < priv->plat->tx_queues_to_use; queue++) {
		struct bstvmac_channel *ch = &priv->tx_channel[queue];

		ch->priv_data = priv;
		ch->index = queue;

		if (queue < priv->plat->tx_queues_to_use)
			ch->has_tx = true;

		INIT_WORK(&ch->tx_work, bstvmac_tx_work);
	}

	mutex_init(&priv->lock);

	ret = register_netdev(ndev);
	if (ret) {
		dev_err(priv->device,
			"%s: ERROR %i registering the device\n", __func__, ret);
		goto error_netdev_register;
	}

	vmac_priv_g[plat_dat->bus_id] = priv;
	if (plat_dat->bus_id == BSTVMAC_BUS_ID) {
		spin_lock_init(&vmac_irqbits_lock);
	}
	plat_dat->bsp_priv = priv;

	// subscribe broadcast
	if (client) {
#if defined(CONFIG_BST_C1200_DB)
		ret = client->db_switch_client.notify_vmac_reinit_sub(bstvmac_reinit_triggered,
						(void *)priv, NULL, bstvmac_reinit_reply, NULL);
#elif defined(CONFIG_BST_C1200_IVI)
		ret = client->ivi_switch_client.notify_vmac_reinit_sub(bstvmac_reinit_triggered,
						(void *)priv, NULL, bstvmac_reinit_reply, NULL);
#endif
		if (ret < 0) {
			printk("%s send subscribe msg fail, ret = %d\n", __func__, ret);
		} else {
			printk("%s send subscribe msg success\n", __func__);
		}
	}
#if defined(CONFIG_BST_C1200_DB)
    ret = client->db_switch_client.register_avail_changed(
        on_server_changed, priv);
#elif defined(CONFIG_BST_C1200_IVI)
    ret = client->ivi_switch_client.register_avail_changed(
        on_server_changed, priv);
#endif
    if (ret < 0) {
        printk("%s: msgbox register_avail_changed failed\n", __func__);
    } else {
        printk("%s: msgbox register_avail_changed success\n", __func__);
    }
#ifdef CONFIG_DEBUG_FS
	ret = bstvmac_init_fs(ndev);
	if (ret < 0)
		netdev_warn(priv->dev,
			    "%s: failed debugFS registration\n", __func__);
#endif

	return ret;

error_netdev_register:
	for (queue = 0; queue < priv->plat->rx_queues_to_use; queue++) {
		kfree(vmac_delivery_skblist[priv->plat->bus_id *
							BSTVMAC_RXCHAN_NUM + queue]);
	}
	bstvmac_netif_del_napi(priv);
error_hw_init:
	destroy_workqueue(priv->wq);
	destroy_workqueue(priv->rxmem);
	destroy_workqueue(priv->tx_wq);
error_wq:
	free_netdev(ndev);
	return ret;
}
EXPORT_SYMBOL_GPL(bstvmac_dvr_probe);

/**
 * bstvmac_dvr_remove
 * @dev: device pointer
 * Description: this function resets the TX/RX processes, disables the MAC RX/TX
 * changes the link status, releases the DMA descriptor rings.
 */
int bstvmac_dvr_remove(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct bstvmac_priv *priv = netdev_priv(ndev);
	int queue;
//    int ret;

	netdev_info(priv->dev, "%s: removing driver", __func__);
	
//	netif_carrier_off(ndev);

//	ret = bstvmac_hif_sw_reset(priv);
//	printk("%s: hif prefetch fifo clear done (ret = %d)\n", __func__, ret);

//	set_bit(BSTVMAC_DOWN, &priv->state);
#ifdef CONFIG_DEBUG_FS
	bstvmac_exit_fs(ndev);
#endif
//	bstvmac_stop_all_dma(priv);

    bstvmac_netif_del_napi(priv);

	unregister_netdev(ndev);

	destroy_workqueue(priv->wq);
	destroy_workqueue(priv->rxmem);
	destroy_workqueue(priv->tx_wq);
	mutex_destroy(&priv->lock);

	perf_debug = 0;

	for (queue = 0; queue < priv->plat->rx_queues_to_use; queue++) {
		kfree(vmac_delivery_skblist[priv->plat->bus_id *
							BSTVMAC_RXCHAN_NUM + queue]);
		vmac_delivery_skblist[priv->plat->bus_id *
							BSTVMAC_RXCHAN_NUM + queue] = NULL;
	}

	free_netdev(ndev);
	printk("%s done\n", __func__);

	return 0;
}
EXPORT_SYMBOL_GPL(bstvmac_dvr_remove);

/**
 * bstvmac_suspend - suspend callback
 * @dev: device pointer
 * Description: this is the function to suspend the device and it is called
 * by the platform driver to stop the network queue, release the resources,
 * program the PMT register (for WoL), clean and release driver resources.
 */
int bstvmac_suspend(struct device *dev)
{
	int ret;
	struct ifreq ifr;
	struct net_device *ndev = dev_get_drvdata(dev);

	rcu_read_lock();
	ifr.ifr_flags = (short) dev_get_flags(ndev);
	rcu_read_unlock();

	ifr.ifr_flags &= ~IFF_UP;

	rtnl_lock();
	ret = dev_change_flags(ndev, ifr.ifr_flags, NULL);
	rtnl_unlock();

	return ret;
}
EXPORT_SYMBOL_GPL(bstvmac_suspend);

/**
 * bstvmac_resume - resume callback
 * @dev: device pointer
 * Description: when resume this function is invoked to setup the DMA and CORE
 * in a usable state.
 */
int bstvmac_resume(struct device *dev)
{
	int ret;
	struct ifreq ifr;
	struct net_device *ndev = dev_get_drvdata(dev);

	rcu_read_lock();
	ifr.ifr_flags = (short) dev_get_flags(ndev);
	rcu_read_unlock();

	ifr.ifr_flags &= ~IFF_UP;
	ifr.ifr_flags |= IFF_UP;

	rtnl_lock();
	ret = dev_change_flags(ndev, ifr.ifr_flags, NULL);
	rtnl_unlock();

	return ret;
}
EXPORT_SYMBOL_GPL(bstvmac_resume);

#ifndef MODULE
static int __init bstvmac_cmdline_opt(char *str)
{
	char *opt;

	if (!str || !*str)
		return -EINVAL;
	while ((opt = strsep(&str, ",")) != NULL) {
		if (!strncmp(opt, "debug:", 6)) {
			if (kstrtoint(opt + 6, 0, &debug))
				goto err;
		} else if (!strncmp(opt, "buf_sz:", 7)) {
			if (kstrtoint(opt + 7, 0, &buf_sz))
				goto err;
		} else if (!strncmp(opt, "watchdog:", 9)) {
			if (kstrtoint(opt + 9, 0, &watchdog))
				goto err;
		} else if (!strncmp(opt, "pause:", 6)) {
			if (kstrtoint(opt + 6, 0, &pause))
				goto err;
		} else if (!strncmp(opt, "chain_mode:", 11)) {
			if (kstrtoint(opt + 11, 0, &chain_mode))
				goto err;
		}
	}
	return 0;

err:
	pr_err("%s: ERROR broken module parameter conversion", __func__);
	return -EINVAL;
}

__setup("bstvmaceth=", bstvmac_cmdline_opt);
#endif /* MODULE */

static int __init bstvmac_init(void)
{
#ifdef CONFIG_DEBUG_FS
	/* Create debugfs main directory if it doesn't exist yet */
	if (!bstvmac_fs_dir) {
		bstvmac_fs_dir =
		    debugfs_create_dir(BSTVMAC_RESOURCE_NAME, NULL);

		if (!bstvmac_fs_dir || IS_ERR(bstvmac_fs_dir)) {
			pr_err
			    ("ERROR %s, debugfs create directory failed\n",
			     BSTVMAC_RESOURCE_NAME);

			return -ENOMEM;
		}
	}
#endif

	return 0;
}

static void __exit bstvmac_exit(void)
{
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(bstvmac_fs_dir);
#endif
}

module_init(bstvmac_init)
module_exit(bstvmac_exit)

MODULE_DESCRIPTION("BSTMAC Ethernet device driver");
MODULE_AUTHOR("BST Ltd.");
MODULE_LICENSE("GPL");
