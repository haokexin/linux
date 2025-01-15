// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023~2024 Black Sesame Technologies, Inc.
 *
 * Author: Xuran Yang <xuran.yang@bst.ai>
 */

#define pr_fmt(fmt) "virtnet-epf: " fmt
#include "queue.h"
#include "virtnet.h"
#include <linux/compiler.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/etherdevice.h>
#include <linux/gfp.h>
#include <linux/hrtimer.h>
#include <linux/io.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/pci-epc.h>
#include <linux/pci-epf.h>
#include <linux/pci_ids.h>
#include <linux/pci_regs.h>
#include <linux/printk.h>
#include <linux/skbuff.h>
#include <linux/stddef.h>
#include <linux/time64.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/kfifo.h>
#include <linux/dma/edma.h>
#include <linux/bst-pcie.h>

#define DRV_MODULE_NAME "pci-epf-virtnet"

struct virtnet_adapter;

struct epf_dma_filter {
	struct device *dev;
	u32 dma_mask;
};

static struct pci_epf_header epf_virtnet_header = {
	.vendorid = PCI_VENDOR_ID_BST,
	.deviceid = PCI_DEVICE_ID_BST_VNET,
	.baseclass_code = PCI_BASE_CLASS_NETWORK,
	.interrupt_pin = PCI_INTERRUPT_INTA,
};

static size_t bar_size[] = { 0x100000, 0x100000, 0, 0, 0, 0 };

struct virtnet_quirks {
	bool poll;
	int irq_type;
	int (*request_irq)(struct pci_epf *epf);
	int (*free_irq)(struct pci_epf *epf);
	int (*disable_irq)(struct pci_epf *epf);
	int (*enable_irq)(struct pci_epf *epf);
	int (*get_irq_msg)(struct pci_epf *epf, int *bar, u32 *offset, u32 *msg);
};

struct virtnet_skb {
	struct list_head node;
	struct sk_buff *skb;
	u32 skb_len;
	dma_addr_t paddr;
};

struct pci_epf_virtnet {
	void *reg[PCI_STD_NUM_BARS];
	struct pci_epf *epf;
	const struct pci_epc_features *epc_features;
	struct list_head node;
	bool is_init;

	bool poll;
	int doorbell_irq;
	u32 irq_cnt;
	struct hrtimer hr_timer;
	struct work_struct raise_irq_work;
	struct work_struct xmit_done_irq_work;
	struct work_struct dma_receive_work;
	struct virtnet_adapter *adapter;
	const struct virtnet_quirks *quirks;
	struct task_struct *poll_int_task;

	/* The DMA channel arg uses the direction of EP  */
	struct dma_chan *tx;
	struct dma_chan *rx;

	struct virtnet_queue rc_queue; /* RC->EP */
	struct virtnet_queue ep_queue; /* EP->RC */
	struct kfifo xmit_done;

	struct pci_epf_header *header;
};

struct virtnet_adapter {
	struct net_device *netdev;
	struct pci_epf_virtnet *epf_virtnet;

	unsigned long *flags; /* Pointed to bar reg ep_status */
	struct napi_struct napi;
	struct net_device_stats stats;
};

struct virtnet_dma_region {
	dma_addr_t paddr;
	void __iomem *vaddr;
	size_t sz;
};

struct virtnet_xmit {
	struct sk_buff *l_skb; /* local skb */
	int nr_frags;

	dma_addr_t paddr;
	struct sg_table *sgt;

	struct virtnet_desc *desc;
	struct virtnet_adapter *adapter;
	struct virtnet_dma_region *region;
	void (*done_cb)(void *arg);
};

/* contains all registered pci_epf_virtnet */
static LIST_HEAD(epf_virtnet_list);
/* This struct is for free all skb
 * when module exit
 */
static LIST_HEAD(skb_list);
static const struct pci_epf_device_id pci_epf_virtnet_ids[];

static void virtnet_start_poll_timer(struct pci_epf_virtnet *epf_virtnet);
static void virtnet_stop_poll_timer(struct pci_epf_virtnet *epf_virtnet);
static int pci_epf_virtnet_irqhandler(int irq, void *arg);

static const struct virtnet_quirks virtnet_poll = {.poll = true, .irq_type = PCI_EPC_IRQ_MSIX};

static int bst_pcie_request_irq(struct pci_epf *epf)
{
	int ret;
	struct pci_epf_virtnet *epf_virtnet = epf_get_drvdata(epf);

	ret = bst_pcie_ep_db_irq_alloc(epf->epc, epf->func_no, epf->vfunc_no);
	if (ret < 0)
		return ret;

	epf_virtnet->doorbell_irq = ret;
	bst_pcie_ep_db_irq_request(epf->epc, epf->func_no, epf->vfunc_no, epf_virtnet->doorbell_irq,
				   pci_epf_virtnet_irqhandler, epf_virtnet);
	return 0;
}

static int bst_pcie_get_irq_msg(struct pci_epf *epf, int *bar, u32 *offset, u32 *msg)
{
	int ret;
	struct pci_epf_virtnet *epf_virtnet = epf_get_drvdata(epf);

	ret = bst_pcie_ep_db_info_get(epf->epc, epf->func_no, epf->vfunc_no,
				      epf_virtnet->doorbell_irq, bar, offset, msg);
	if (ret < 0)
		return ret;

	return 0;
}

static int bst_pcie_free_irq(struct pci_epf *epf)
{
	struct pci_epf_virtnet *epf_virtnet = epf_get_drvdata(epf);

	if (epf_virtnet->doorbell_irq >= 0) {
		bst_pcie_ep_db_irq_free(epf->epc, epf->func_no, epf->vfunc_no,
					epf_virtnet->doorbell_irq);
		epf_virtnet->doorbell_irq = -1;
	}
	return 0;
}

static const struct virtnet_quirks virtnet_bst_doorbell = {
	.poll = false,
	.request_irq = bst_pcie_request_irq,
	.get_irq_msg = bst_pcie_get_irq_msg,
	.free_irq = bst_pcie_free_irq,
	.irq_type = PCI_EPC_IRQ_MSIX
};

static int pci_epf_set_bar(struct pci_epf *epf)
{
	int bar, add;
	int ret;
	struct pci_epf_bar *epf_bar;
	struct pci_epc *epc = epf->epc;
	struct device *dev = &epf->dev;
	struct pci_epf_virtnet *epf_virtnet = epf_get_drvdata(epf);
	const struct pci_epc_features *epc_features = epf_virtnet->epc_features;

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar += add) {
		epf_bar = &epf->bar[bar];
		/* pci_epc_set_bar() sets PCI_BASE_ADDRESS_MEM_TYPE_64
		 * if the specific implementation required a 64-bit BAR,
		 * even if we only requested a 32-bit BAR.
		 */
		add = (epf_bar->flags & PCI_BASE_ADDRESS_MEM_TYPE_64) ? 2 : 1;

		if (!!(epc_features->reserved_bar & (1 << bar)))
			continue;

		ret = pci_epc_set_bar(epc, epf->func_no, epf->vfunc_no, epf_bar);
		if (ret) {
			pci_epf_free_space(epf, epf_virtnet->reg[bar], bar, PRIMARY_INTERFACE);
			dev_err(dev, "Failed to set BAR%d\n", bar);
		}
	}

	return 0;
}

static int pci_epf_free_bar(struct pci_epf *epf)
{
	int bar, add;
	struct pci_epf_bar *epf_bar;
	struct pci_epf_virtnet *epf_virtnet = epf_get_drvdata(epf);
	const struct pci_epc_features *epc_features = epf_virtnet->epc_features;

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar += add) {
		epf_bar = &epf->bar[bar];
		add = (epf_bar->flags & PCI_BASE_ADDRESS_MEM_TYPE_64) ? 2 : 1;

		if (!!(epc_features->reserved_bar & (1 << bar)))
			continue;

		pci_epf_free_space(epf, epf_virtnet->reg[bar], bar, PRIMARY_INTERFACE);
		pci_epc_clear_bar(epf->epc, epf->func_no, epf->vfunc_no, epf_bar);
	}
	return 0;
}

static void pci_epf_virtnet_raise_irq(struct pci_epf_virtnet *epf_virtnet, u16 irq)
{
	int ret;
	struct pci_epf *epf = epf_virtnet->epf;
	struct pci_epc *epc = epf->epc;

	ret = pci_epc_raise_irq(epc, epf->func_no, epf->vfunc_no,
				epf_virtnet->quirks->irq_type, irq);
	if (unlikely(ret))
		pr_err("raise irq err:%d\n", ret);
}

/* need raise irq in softirq, because pci_epc_raise_irq has mutex in it */
static void pci_epf_virtnet_raise_irq_work(struct work_struct *work)
{
	struct pci_epf_virtnet *epf_virtnet =
		container_of(work, struct pci_epf_virtnet, raise_irq_work);

	pci_epf_virtnet_raise_irq(epf_virtnet, VNET_NOTIFY_MSI + 1);
}

static void pci_epf_virtnet_raise_xmit_down_irq(struct work_struct *work)
{
	struct pci_epf_virtnet *epf_virtnet =
		container_of(work, struct pci_epf_virtnet, xmit_done_irq_work);

	pci_epf_virtnet_raise_irq(epf_virtnet, VNET_NOTIFY_XMIT_DONE + 1);
}

static void pci_epf_virtnet_set_irqmsg(struct pci_epf_virtnet *epf_virtnet, int bar, u32 offset,
				       u32 msg)
{
	struct virtnet_bar *reg = epf_virtnet->reg[VNET_USED_BAR];

	reg->ep_int_bar = bar;
	reg->ep_int_base = offset;
	reg->ep_int_msg = msg;
	reg->ep_int_cnt = 0;
}

static bool pcie_dma_filter(struct dma_chan *chan, void *node)
{
	struct epf_dma_filter *filter = node;
	struct dma_slave_caps caps;

	memset(&caps, 0, sizeof(caps));
	dma_get_slave_caps(chan, &caps);

	return chan->device->dev == filter->dev && (filter->dma_mask & caps.directions);
}

static int pci_epf_virtnet_dma_alloc(struct pci_epf *epf)
{
	struct device *dev = epf->epc->dev.parent;
	struct pci_epf_virtnet *epf_virtnet = epf_get_drvdata(epf);
	struct dma_chan *txchan;
	struct dma_chan *rxchan;
	dma_cap_mask_t mask;
	struct epf_dma_filter filter;

	filter.dev = dev;
	filter.dma_mask = BIT(DMA_DEV_TO_MEM);
	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	rxchan = dma_request_channel(mask, pcie_dma_filter, &filter);
	if (!rxchan) {
		pr_err("Failed to get private DMA Rx channel.\n");
		goto err_alloc_rx;
	} else {
		pr_info("RX chan name:%s\n", dma_chan_name(rxchan));
	}
	dw_edma_func_num_set(rxchan, epf->func_no, epf->vfunc_no);

	filter.dma_mask = BIT(DMA_MEM_TO_DEV);
	txchan = dma_request_channel(mask, pcie_dma_filter, &filter);
	if (!txchan) {
		pr_err("Failed to get private DMA Tx channel.\n");
		goto err_alloc_tx;
	} else {
		pr_info("TX chan name:%s\n", dma_chan_name(txchan));
	}
	dw_edma_func_num_set(txchan, epf->func_no, epf->vfunc_no);

	dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));

	epf_virtnet->tx = txchan;
	epf_virtnet->rx = rxchan;
	return 0;

err_alloc_tx:
	dma_release_channel(rxchan);
err_alloc_rx:
	return -1;
}

static void pci_epf_virtnet_dma_free(struct pci_epf *epf)
{
	struct pci_epf_virtnet *epf_virtnet = epf_get_drvdata(epf);

	dmaengine_terminate_all(epf_virtnet->tx);
	dmaengine_terminate_all(epf_virtnet->rx);
	dma_release_channel(epf_virtnet->tx);
	dma_release_channel(epf_virtnet->rx);
}

static struct virtnet_skb *virtnet_alloc_skb(struct virtnet_adapter *adapter)
{
	dma_addr_t paddr;
	struct sk_buff *skb;
	struct virtnet_skb *skb_node;
	struct device *dev = adapter->epf_virtnet->epf->epc->dev.parent;
	u32 mtu = adapter->netdev->mtu;
	size_t len = mtu + VNET_SKB_REV;

	skb = napi_alloc_skb(&adapter->napi, len);
	if (unlikely(!skb))
		goto alloc_skb;
	paddr = dma_map_single(dev, skb->data, len, DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(dev, paddr))) {
		pr_err("dma_map_single failed\n");
		goto dma_map;
	}
	skb_node = kmalloc(sizeof(*skb_node), GFP_KERNEL);
	if (unlikely(!skb_node))
		goto alloc_node;

	skb_node->skb = skb;
	skb_node->paddr = paddr;
	skb_node->skb_len = len;
	list_add_tail(&skb_node->node, &skb_list);
	return skb_node;
alloc_node:
	dma_unmap_single(dev, paddr, len, DMA_FROM_DEVICE);
dma_map:
	dev_kfree_skb(skb);
alloc_skb:
	return NULL;
}

static void virtnet_free_skb_node(struct virtnet_skb *skb_node)
{
	list_del(&skb_node->node);
	kfree(skb_node);
}

static void virtnet_free_skb(struct virtnet_adapter *adapter, struct virtnet_skb *skb_node)
{
	struct device *dev = adapter->epf_virtnet->epf->epc->dev.parent;

	dma_unmap_single(dev, skb_node->paddr, skb_node->skb_len, DMA_FROM_DEVICE);
	dev_kfree_skb(skb_node->skb);
	virtnet_free_skb_node(skb_node);
}

static void pci_epf_virtnet_free_all_skb(struct virtnet_adapter *adapter)
{
	struct virtnet_skb *skb_node, *next;

	list_for_each_entry_safe(skb_node, next, &skb_list, node)
		virtnet_free_skb(adapter, skb_node);
}

static int pci_epf_virtnet_fill_queue(struct virtnet_adapter *adapter, struct virtnet_queue *queue)
{
	struct virtnet_skb *skb_node;
	struct virtnet_desc *desc;
	int p = -1;

	while ((desc = get_each_desc(queue, &p)) != NULL) {
		skb_node = virtnet_alloc_skb(adapter);
		if (unlikely(!skb_node)) {
			pr_err("alloc skb failed, packet drop!\n");
			goto err_alloc_skb;
		}
		desc->skb1 = skb_node;
	}
	return 0;

err_alloc_skb:
	return -1;
}

static int virtnet_napi_recv(struct virtnet_adapter *adapter, int buget)
{
	struct device *dev = adapter->epf_virtnet->epf->epc->dev.parent;
	struct pci_epf_virtnet *epf_virtnet = adapter->epf_virtnet;
	struct net_device *netdev = adapter->netdev;
	struct virtnet_xmit *xmit;
	struct virtnet_skb *skb_node;
	struct sk_buff *skb;
	int i, cnt = 0;

	for (i = 0; i < buget; i++) {
		if (kfifo_out(&epf_virtnet->xmit_done, &xmit, sizeof(struct virtnet_xmit *))
		    != sizeof(struct virtnet_xmit *))
			break;

		skb_node = xmit->desc->skb1;
		skb = skb_node->skb;
		dma_unmap_single(dev, skb_node->paddr, skb_node->skb_len, DMA_FROM_DEVICE);

		cnt++;
		skb_put(skb, xmit->desc->size);
		skb->protocol = eth_type_trans(skb, netdev);
		skb->dev = netdev;
		skb->ip_summed = CHECKSUM_UNNECESSARY; /* don't check it */
		netdev->stats.rx_packets++;
		netdev->stats.rx_bytes += skb->len;
		napi_gro_receive(&adapter->napi, skb);
		virtnet_free_skb_node(skb_node);
		set_desc_owner_bit(&adapter->epf_virtnet->rc_queue, xmit->desc, false);
		skb_node = virtnet_alloc_skb(adapter);
		if (unlikely(!skb_node)) {
			pr_err("alloc skb failed\n");
			break;
		}
		xmit->desc->skb1 = skb_node;
		kfree(xmit);
	}
	schedule_work(&adapter->epf_virtnet->xmit_done_irq_work);
	return cnt;
}

static int pci_epf_virtnet_poll(struct napi_struct *napi, int budget)
{
	struct virtnet_adapter *adapter = container_of(napi, struct virtnet_adapter, napi);
	int work_done = 0;

	work_done = virtnet_napi_recv(adapter, budget);
	if (work_done == budget)
		return budget;
	napi_complete_done(napi, work_done);

	return work_done;
}

static void virtnet_receive_done_cb(void *arg)
{
	struct virtnet_xmit *xmit = arg;
	struct virtnet_adapter *adapter = xmit->adapter;
	struct net_device *netdev = adapter->netdev;
	int ret;

	ret = kfifo_in(&adapter->epf_virtnet->xmit_done, &xmit, sizeof(struct virtnet_xmit *));
	if (ret < sizeof(struct virtnet_xmit *)) {
		netdev->stats.rx_dropped++;
		kfree(xmit);
		return;
	}

	if (likely(napi_schedule_prep(&adapter->napi)))
		__napi_schedule(&adapter->napi);
}

static int virtnet_dma_receive(struct virtnet_adapter *adapter, struct virtnet_xmit *xmit)
{
	struct device *dev = adapter->epf_virtnet->epf->epc->dev.parent;
	struct dma_async_tx_descriptor *desc;
	struct dma_slave_config sconf = {};
	dma_cookie_t cookie;
	struct dma_chan *chan;
	struct virtnet_skb *skb_node = xmit->desc->skb1;
	dma_addr_t paddr = skb_node->paddr;

	sconf.direction = DMA_DEV_TO_MEM;
	sconf.src_addr = xmit->desc->data;
	chan = adapter->epf_virtnet->rx;
	dmaengine_slave_config(chan, &sconf);

	desc = dmaengine_prep_slave_single(chan, paddr, xmit->desc->size, DMA_DEV_TO_MEM,
					   DMA_PREP_INTERRUPT);
	if (unlikely(!desc)) {
		pr_err("dma prep signle failed\n");
		goto err_prep_signle;
	}
	desc->callback = xmit->done_cb;
	desc->callback_param = xmit->done_cb ? xmit : NULL;
	cookie = dmaengine_submit(desc);

	if (unlikely(dma_submit_error(cookie))) {
		pr_err("dma sumbit err\n");
		goto err_submit;
	}

	/* Start DMA transfer */
	dma_async_issue_pending(chan);

	return 0;

err_submit:
err_prep_signle:
	dma_unmap_single(dev, paddr, xmit->desc->size, DMA_FROM_DEVICE);
	return -1;
}

static void pci_epf_virtnet_clean_rx(struct work_struct *work)
{
	struct pci_epf_virtnet *epf_virtnet =
		container_of(work, struct pci_epf_virtnet, dma_receive_work);
	struct virtnet_adapter *adapter = epf_virtnet->adapter;
	struct virtnet_queue *queue = &epf_virtnet->rc_queue;
	struct net_device *netdev = epf_virtnet->adapter->netdev;
	struct virtnet_bar *reg = (struct virtnet_bar *)epf_virtnet->reg[VNET_USED_BAR];
	struct virtnet_xmit *xmit;
	struct virtnet_desc *desc;

	while (true) {
		desc = get_owned_descriptor(queue);
		if (!desc)
			break;

		xmit = kmalloc(sizeof(*xmit), GFP_KERNEL);
		if (!xmit)
			goto err_alloc;

		xmit->desc = desc;
		xmit->adapter = adapter;
		xmit->done_cb = virtnet_receive_done_cb;
		if (unlikely(virtnet_dma_receive(adapter, xmit)))
			goto err_dma;
	}
	reg->ep_int_mask &= ~(1 << VNET_NOTIFY_EP);
	return;
err_dma:
	pr_err("unknown err when receive\n");
	kfree(xmit);
err_alloc:
	set_desc_owner_bit(queue, desc, false);
	netdev->stats.tx_dropped++;
	reg->ep_int_mask &= ~(1 << VNET_NOTIFY_EP);
}

static int pci_epf_virtnet_irqhandler(int irq, void *arg)
{
	struct pci_epf_virtnet *epf_virtnet = arg;
	struct virtnet_bar *reg = epf_virtnet->reg[VNET_USED_BAR];
	struct virtnet_adapter *adapter = epf_virtnet->adapter;

	reg->ep_int_mask |= (1 << VNET_NOTIFY_EP);
	schedule_work(&adapter->epf_virtnet->dma_receive_work);
	return IRQ_HANDLED;
}

#ifdef CONFIG_NO_SG
static void virtnet_xmit_done_cb(void *arg)
{
	struct virtnet_xmit *xmit = arg;
	struct sk_buff *skb = xmit->l_skb;
	struct virtnet_desc *desc = xmit->desc;
	struct net_device *netdev = xmit->adapter->netdev;
	struct device *dev = xmit->adapter->epf_virtnet->epf->epc->dev.parent;
	struct virtnet_bar *reg = xmit->adapter->epf_virtnet->reg[VNET_USED_BAR];

	/* Let it know the data size */
	desc->size = skb->len;
	netdev->stats.tx_packets++;
	netdev->stats.tx_bytes += skb->len;
	/* Notify RC, the data is ready */
	set_desc_owner_bit(&xmit->adapter->epf_virtnet->ep_queue, desc, true);
	wmb(); /* Makesure descripter is set */
	if (!(reg->rc_int_mask & (1 << VNET_NOTIFY_MSI)))
		schedule_work(&xmit->adapter->epf_virtnet->raise_irq_work);
	dma_unmap_single(dev, xmit->paddr, skb->len, DMA_TO_DEVICE);
	dev_kfree_skb_any(skb);
	kfree(xmit);
}

static int virtnet_dma_xmit(struct virtnet_adapter *adapter, struct virtnet_xmit *xmit)
{
	struct dma_async_tx_descriptor *desc;
	struct dma_slave_config sconf = {};
	struct dma_chan *chan;
	struct sk_buff *skb = xmit->l_skb;
	struct device *dev = adapter->epf_virtnet->epf->epc->dev.parent;
	dma_addr_t paddr;
	dma_cookie_t cookie;

	sconf.direction = DMA_MEM_TO_DEV;
	sconf.dst_addr = xmit->desc->data;
	chan = adapter->epf_virtnet->tx;

	paddr = dma_map_single(dev, skb->data, skb->len, DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(dev, paddr))) {
		pr_err("dma_map_single failed\n");
		goto err_dma_map;
	}
	xmit->paddr = paddr; /* For callback to unmap */
	dmaengine_slave_config(chan, &sconf);
	desc = dmaengine_prep_slave_single(chan, paddr, skb->len, DMA_MEM_TO_DEV, 0);
	if (unlikely(!desc)) {
		pr_err("dma prep signle failed\n");
		goto err_prep;
	}

	desc->callback = xmit->done_cb;
	desc->callback_param = xmit->done_cb ? xmit : NULL;
	cookie = dmaengine_submit(desc);

	if (unlikely(dma_submit_error(cookie))) {
		pr_err("dma sumbit err\n");
		goto err_submit;
	}

	/* Start DMA transfer */
	// dma_async_issue_pending(chan);

	return 0;

err_submit:
err_prep:
	dma_unmap_single(dev, paddr, skb->len, DMA_TO_DEVICE);
err_dma_map:
	return -1;
}

#else

static void virtnet_xmit_done_cb(void *arg)
{
	struct virtnet_xmit *xmit = arg;
	struct sk_buff *skb = xmit->l_skb;
	struct virtnet_desc *desc = xmit->desc;
	struct sg_table *sgt = xmit->sgt;
	struct net_device *netdev = xmit->adapter->netdev;
	struct virtnet_bar *reg = xmit->adapter->epf_virtnet->reg[VNET_USED_BAR];

	/* Let it know the data size */
	desc->size = skb->len;
	netdev->stats.tx_packets++;
	netdev->stats.tx_bytes += skb->len;
	/* Notify RC, the data is ready */
	set_desc_owner_bit(&xmit->adapter->epf_virtnet->ep_queue,
			   desc, true);
	wmb(); /* Makesure descripter is set */
	if (!(reg->rc_int_mask & (1 << VNET_NOTIFY_MSI)))
		schedule_work(&xmit->adapter->epf_virtnet->raise_irq_work);

	sg_free_table(sgt);
	kvfree(sgt);
	dev_kfree_skb_any(skb);
	kfree(xmit);
}

static int virtnet_dma_xmit(struct virtnet_adapter *adapter, struct virtnet_xmit *xmit)
{
	struct dma_async_tx_descriptor *desc;
	struct dma_slave_config sconf = {};
	struct dma_chan *chan;
	struct sk_buff *skb = xmit->l_skb;
	struct device *dev = adapter->epf_virtnet->epf->epc->dev.parent;
	dma_cookie_t cookie;
	int nr_frags, i, err;
	struct sg_table *sgt;
	struct scatterlist *sg;

	chan = adapter->epf_virtnet->tx;

	nr_frags = skb_shinfo(skb)->nr_frags;

	/* Allocate scatter-gather table */
	sgt = kvmalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		goto err_end;

	xmit->sgt = sgt;

	err = sg_alloc_table(sgt, nr_frags + 1, GFP_KERNEL);
	if (err)
		goto err_sg_alloc_table;

	sg = &sgt->sgl[0];
	if (!sg)
		goto err_alloc_descs;

	sg_set_buf(sg, (void *)skb->data, skb_headlen(skb));
	sg = sg_next(sg);

	for (i = 0; sg && i < nr_frags; i++) {
		skb_frag_t *frag = &skb_shinfo(skb)->frags[i];

		sg_set_page(sg, skb_frag_page(frag), skb_frag_size(frag), skb_frag_off(frag));
		sg = sg_next(sg);
	}

	sconf.direction = DMA_MEM_TO_DEV;
	sconf.dst_addr = xmit->desc->data;

	err = dma_map_sg(dev, sgt->sgl, sgt->nents, DMA_TO_DEVICE);
	if (!err)
		goto err_dma_map;

	sgt->nents = err;
	dmaengine_slave_config(chan, &sconf);
	desc = dmaengine_prep_slave_sg(chan, sgt->sgl, sgt->nents, DMA_MEM_TO_DEV,
				       DMA_PREP_INTERRUPT);
	if (unlikely(!desc)) {
		pr_err("dma prep sg failed\n");
		goto err_prep;
	}

	desc->callback = xmit->done_cb;
	desc->callback_param = xmit->done_cb ? xmit : NULL;
	cookie = dmaengine_submit(desc);

	if (unlikely(dma_submit_error(cookie))) {
		pr_err("dma sumbit err\n");
		goto err_submit;
	}

	/* Start DMA transfer */
	dma_async_issue_pending(chan);

	return 0;

err_submit:
err_prep:
	dma_unmap_sg(dev, sgt->sgl, sgt->nents, DMA_TO_DEVICE);
err_dma_map:
err_alloc_descs:
	sg_free_table(sgt);
err_sg_alloc_table:
	kvfree(sgt);
err_end:
	return -1;
}
#endif

static netdev_tx_t virtnet_xmit_frame(struct sk_buff *skb, struct net_device *netdev)
{
	struct virtnet_adapter *adapter = netdev_priv(netdev);
	struct virtnet_queue *queue = &adapter->epf_virtnet->ep_queue;
	struct virtnet_bar *reg = adapter->epf_virtnet->reg[VNET_USED_BAR];
	struct virtnet_desc *desc;
	struct virtnet_xmit *xmit;

	/* Check if remote is under maintenance */
	if (unlikely(reg->rc_status & (1 << __VNET_DOWN)))
		return NETDEV_TX_BUSY;

	desc = get_free_descriptor(queue);
	if (unlikely(!desc)) {
		if (!(reg->rc_int_mask & (1 << VNET_NOTIFY_MSI)))
			schedule_work(&adapter->epf_virtnet->raise_irq_work);
		dma_async_issue_pending(adapter->epf_virtnet->tx);
		return NETDEV_TX_BUSY;
	}

	xmit = kmalloc(sizeof(*xmit), GFP_KERNEL);
	xmit->l_skb = skb;
	xmit->nr_frags = skb_shinfo(skb)->nr_frags;
	xmit->desc = desc;
	xmit->adapter = adapter;
	xmit->done_cb = virtnet_xmit_done_cb;
	if (unlikely(virtnet_dma_xmit(adapter, xmit))) {
		pr_err("dma xmit failed\n");
		dev_kfree_skb_any(skb);
		kfree(xmit);
		return NETDEV_TX_OK;
	}

	dma_async_issue_pending(adapter->epf_virtnet->tx);

	return NETDEV_TX_OK;
}

static int virtnet_open(struct net_device *netdev)
{
	struct virtnet_adapter *adapter = netdev_priv(netdev);
	struct pci_epf_virtnet *epf_virtnet = adapter->epf_virtnet;

	if (epf_virtnet->poll)
		virtnet_start_poll_timer(epf_virtnet);
	__clear_bit(__VNET_DOWN, adapter->flags);
	napi_enable(&adapter->napi);
	netif_carrier_on(netdev);
	return 0;
}

static int virtnet_close(struct net_device *netdev)
{
	struct virtnet_adapter *adapter = netdev_priv(netdev);
	struct pci_epf_virtnet *epf_virtnet = adapter->epf_virtnet;

	if (epf_virtnet->poll)
		virtnet_stop_poll_timer(epf_virtnet);
	__set_bit(__VNET_DOWN, adapter->flags);
	napi_disable(&adapter->napi);
	netif_carrier_off(netdev);
	return 0;
}

static int virtnet_change_mtu(struct net_device *netdev, int new_mtu)
{
	struct virtnet_adapter *adapter = netdev_priv(netdev);

	if (!test_bit(__VNET_DOWN, adapter->flags)) {
		pr_err("virtnet is running\n");
		return -1;
	}
	netdev->mtu = new_mtu;
	netdev_update_features(netdev);

	return 0;
}

static const struct net_device_ops virtnetdev_ops = {
	.ndo_start_xmit = virtnet_xmit_frame,
	.ndo_change_mtu = virtnet_change_mtu,
	.ndo_open = virtnet_open,
	.ndo_stop = virtnet_close,
};

static void pci_epf_virtnet_setup(struct net_device *netdev)
{
	ether_setup(netdev);
	netdev->flags |= IFF_NOARP;
#ifdef CONFIG_NO_SG
	netdev->features |= (NETIF_F_HW_CSUM | NETIF_F_HIGHDMA | NETIF_F_GRO);
#else
	netdev->features |=
		(NETIF_F_HW_CSUM | NETIF_F_HIGHDMA | NETIF_F_SG | NETIF_F_GSO);
	netdev->hw_features = netdev->features;
#endif
	netdev->mtu = 65536;
	netdev->max_mtu = VNET_MAX_MTU;
	netdev->netdev_ops = &virtnetdev_ops;
}

static int pci_epf_virtnet_core_init(struct pci_epf *epf)
{
	int ret = -1, bar;
	void *base;
	u32 offset, msg;
	struct pci_epc *epc = epf->epc;
	struct net_device *netdev;
	struct virtnet_adapter *adapter;
	struct virtnet_bar *reg;
	struct pci_epf_virtnet *epf_virtnet = epf_get_drvdata(epf);
	const struct pci_epc_features *epc_features;
	bool msix_capable = false;
	bool msi_capable = true;

	base = epf_virtnet->reg[VNET_USED_BAR];

	reg = base;
	epc_features = pci_epc_get_features(epc, epf->func_no, epf->vfunc_no);
	if (epc_features) {
		msix_capable = epc_features->msix_capable;
		msi_capable = epc_features->msi_capable;
	}
	if (epf->vfunc_no <= 1) {
		ret = pci_epc_write_header(epc, epf->func_no, epf->vfunc_no, epf->header);
		if (ret) {
			pr_err("Configuration header write failed\n");
			return ret;
		}
	}
	pci_epf_set_bar(epf);

	if (msi_capable) {
		epf->msi_interrupts = VNET_MSI_TOTAL;
		ret = pci_epc_set_msi(epc, epf->func_no, epf->vfunc_no,
				      epf->msi_interrupts);
		if (ret) {
			pr_err("MSI configuration failed\n");
			return ret;
		}
	}

	if (msix_capable) {
		epf->msix_interrupts = VNET_MSI_TOTAL;
		ret = pci_epc_set_msix(epc, epf->func_no, epf->vfunc_no,
				       epf->msix_interrupts,
				       VNET_USED_BAR,
				       VNET_MSIX_TABLE);
		if (ret) {
			pr_err("MSI-X configuration failed\n");
			return ret;
		}
	}

#ifdef CONFIG_ARCH_BSTC1200
	epf_virtnet->quirks = (struct virtnet_quirks *)&virtnet_bst_doorbell;
#else
	epf_virtnet->quirks = (struct virtnet_quirks *)&virtnet_poll;
#endif
	epf_virtnet->poll = epf_virtnet->quirks->poll;

	if (epf_virtnet->poll) {
		pr_info("virtnet poll mode\n");
	} else {
		ret = epf_virtnet->quirks->request_irq(epf);
		if (ret)
			goto error_request_irq;
		epf_virtnet->quirks->get_irq_msg(epf, &bar, &offset, &msg);
		/* RC->EP INT is not standard method,
		 * we must let RC know how to trigger EP INT
		 */
		pci_epf_virtnet_set_irqmsg(epf_virtnet, bar, offset, msg);
	}

	pci_epf_virtnet_dma_alloc(epf);
	INIT_WORK(&epf_virtnet->raise_irq_work, pci_epf_virtnet_raise_irq_work);
	INIT_WORK(&epf_virtnet->xmit_done_irq_work, pci_epf_virtnet_raise_xmit_down_irq);
	INIT_WORK(&epf_virtnet->dma_receive_work, pci_epf_virtnet_clean_rx);
	ret = kfifo_alloc(&epf_virtnet->xmit_done, sizeof(void *) * 128, GFP_KERNEL);
	if (ret)
		goto error_alloc_netdev;

	init_queue(&epf_virtnet->rc_queue, base + VNET_RC_DESC_HEAD, VNET_DESC_NUM);
	init_all_desc(&epf_virtnet->rc_queue, false);
	init_queue(&epf_virtnet->ep_queue, base + VNET_EP_DESC_HEAD, VNET_DESC_NUM);
	init_all_desc(&epf_virtnet->ep_queue, true);

	netdev = alloc_netdev(sizeof(struct virtnet_adapter), "veth%d", NET_NAME_UNKNOWN,
			      pci_epf_virtnet_setup);
	if (!netdev) {
		pr_err("alloc netdev failed\n");
		goto error_alloc_netdev;
	}
	ret = register_netdev(netdev);
	if (ret)
		goto err_register_netdev;

	adapter = netdev_priv(netdev);
	adapter->netdev = netdev;
	adapter->epf_virtnet = epf_virtnet;
	epf_virtnet->adapter = adapter;
	adapter->flags = (void *)&reg->ep_status; /* In self mem, as the same */
	netif_napi_add(netdev, &adapter->napi, pci_epf_virtnet_poll);
	pci_epf_virtnet_fill_queue(adapter, &epf_virtnet->rc_queue);
	/* Set magic code avoid to prevent abnormal data interference */
	*adapter->flags = __VNET_MAGIC;
	__set_bit(__VNET_INIT, adapter->flags);
	if (epf_virtnet->poll)
		__set_bit(__VNET_POLLING, adapter->flags);
	epf_virtnet->is_init = true;

	return 0;

err_register_netdev:
	free_netdev(netdev);
error_alloc_netdev:
	pci_epf_virtnet_dma_free(epf);
error_request_irq:
	pci_epf_free_space(epf, base, VNET_USED_BAR, PRIMARY_INTERFACE);
	return ret;
}

static int pci_epf_virtnet_core_exit(struct pci_epf *epf)
{
	struct pci_epf_virtnet *epf_virtnet = epf_get_drvdata(epf);
	struct virtnet_adapter *adapter = epf_virtnet->adapter;
	struct net_device *netdev = adapter->netdev;

	__clear_bit(__VNET_INIT, adapter->flags);
	if (!epf_virtnet->poll)
		epf_virtnet->quirks->free_irq(epf);

	pci_epf_virtnet_free_all_skb(adapter);
	unregister_netdev(netdev);
	kfifo_free(&epf_virtnet->xmit_done);
	pci_epf_virtnet_dma_free(epf);
	free_netdev(netdev);
	epf_virtnet->is_init = false;

	return 0;
}

static void pci_epf_configure_bar(struct pci_epf *epf,
				  const struct pci_epc_features *epc_features)
{
	struct pci_epf_bar *epf_bar;
	bool bar_fixed_64bit;
	int i;

	for (i = 0; i < PCI_STD_NUM_BARS; i++) {
		epf_bar = &epf->bar[i];
		bar_fixed_64bit = !!(epc_features->bar_fixed_64bit & (1 << i));
		if (bar_fixed_64bit)
			epf_bar->flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;
		if (epc_features->bar_fixed_size[i])
			bar_size[i] = epc_features->bar_fixed_size[i];
	}
}

static int pci_epf_virtnet_alloc_space(struct pci_epf *epf)
{
	struct pci_epf_virtnet *epf_virtnet = epf_get_drvdata(epf);
	struct device *dev = &epf->dev;
	struct pci_epf_bar *epf_bar;
	void *base;
	int bar, add;

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar += add) {
		epf_bar = &epf->bar[bar];
		add = (epf_bar->flags & PCI_BASE_ADDRESS_MEM_TYPE_64) ? 2 : 1;

		if (!!(epf_virtnet->epc_features->reserved_bar & (1 << bar)))
			continue;
		if (bar_size[bar] == 0) {
			epf_bar->size = 0;
			epf_bar->barno = bar;
		} else {
			base = pci_epf_alloc_space(epf, bar_size[bar], bar,
						   epf_virtnet->epc_features->align,
						   PRIMARY_INTERFACE);
			if (!base)
				dev_err(dev, "Failed to allocate space for BAR%d\n",
					bar);
			epf_virtnet->reg[bar] = base;
		}
	}

	return 0;
}

static int pci_epf_virtnet_notifier(struct notifier_block *nb, unsigned long val, void *data)
{
	struct pci_epf *epf = container_of(nb, struct pci_epf, nb);
	struct pci_epf_virtnet *epf_virtnet = epf_get_drvdata(epf);

	switch (val) {
	case CORE_INIT:
		if (!epf_virtnet->is_init) {
			pr_info("PCIe link up, virtnet register\n");
			pci_epf_virtnet_core_init(epf);
		}
		break;

	case LINK_UP:
		break;
	default:
		dev_err(&epf->dev, "Invalid EPF virtnet notifier event\n");
		return NOTIFY_BAD;
	}

	return NOTIFY_OK;
}

static int pci_epf_virtnet_bind(struct pci_epf *epf)
{
	int ret;
	struct pci_epf_virtnet *epf_virtnet = epf_get_drvdata(epf);
	const struct pci_epc_features *epc_features;
	struct pci_epc *epc = epf->epc;
	bool linkup_notifier = false;
	bool core_init_notifier = false;

	if (WARN_ON_ONCE(!epc))
		return -EINVAL;

	epc_features = pci_epc_get_features(epc, epf->func_no, epf->vfunc_no);
	if (!epc_features) {
		dev_err(&epf->dev, "epc_features not implemented\n");
		return -EOPNOTSUPP;
	}

	linkup_notifier = epc_features->linkup_notifier;
	core_init_notifier = epc_features->core_init_notifier;

	pci_epf_configure_bar(epf, epc_features);

	epf_virtnet->epc_features = epc_features;

	ret = pci_epf_virtnet_alloc_space(epf);
	if (ret)
		return ret;

	if (!core_init_notifier) {
		ret = pci_epf_virtnet_core_init(epf);
		if (ret)
			return ret;
	}

	if (linkup_notifier || core_init_notifier) {
		epf->nb.notifier_call = pci_epf_virtnet_notifier;
		pci_epc_register_notifier(epc, &epf->nb);
	}

	return 0;
}

static void pci_epf_virtnet_unbind(struct pci_epf *epf)
{
	pci_epf_virtnet_core_exit(epf);
	pci_epf_free_bar(epf);
}

static int pci_epf_virtnet_probe(struct pci_epf *epf)
{
	struct device *dev = &epf->dev;
	struct pci_epf_virtnet *epf_virtnet;

	epf_virtnet = devm_kzalloc(dev, sizeof(*epf_virtnet), GFP_KERNEL);
	if (!epf_virtnet)
		return -ENOMEM;

	epf->header = &epf_virtnet_header;
	epf_virtnet->epf = epf;
	epf_set_drvdata(epf, epf_virtnet);

	/* Store driver data so that it is
	 * properly freed when the module is unloaded
	 */
	list_add(&epf_virtnet->node, &epf_virtnet_list);
	return 0;
}

static enum hrtimer_restart virtnet_poll_timer(struct hrtimer *timer)
{
	struct pci_epf_virtnet *epf_virtnet = container_of(timer, struct pci_epf_virtnet, hr_timer);
	struct virtnet_bar *reg = epf_virtnet->reg[VNET_USED_BAR];
	u32 irq_cnt_current, mask;

	mask = __raw_readl(&reg->ep_int_mask);
	if (!(mask & (1 << VNET_NOTIFY_EP))) {
		irq_cnt_current = __raw_readl(&reg->ep_int_cnt);
		if (epf_virtnet->irq_cnt != irq_cnt_current) {
			epf_virtnet->irq_cnt++;
			pci_epf_virtnet_irqhandler(0, epf_virtnet);
		}
	}
	hrtimer_forward_now(timer, ns_to_ktime(VNET_POLL_INTERVAL) * 1000);
	return HRTIMER_RESTART;
}

static void virtnet_start_poll_timer(struct pci_epf_virtnet *epf_virtnet)
{
	struct hrtimer *hr_timer = &epf_virtnet->hr_timer;
	ktime_t ktime;

	ktime = ns_to_ktime(VNET_POLL_INTERVAL) * 1000;
	hrtimer_init(hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hr_timer->function = virtnet_poll_timer;
	hrtimer_start(hr_timer, ktime, HRTIMER_MODE_REL);
}

static void virtnet_stop_poll_timer(struct pci_epf_virtnet *epf_virtnet)
{
	hrtimer_cancel(&epf_virtnet->hr_timer);
}

static const struct pci_epf_device_id pci_epf_virtnet_ids[] = {
	{
		.name = "pcie-vnet",
	},
	{},
};

static struct pci_epf_ops ops = {
	.bind = pci_epf_virtnet_bind,
	.unbind = pci_epf_virtnet_unbind,
};

static struct pci_epf_driver virtnet_driver = {
	.driver = {.name = DRV_MODULE_NAME},
	.probe = pci_epf_virtnet_probe,
	.id_table = pci_epf_virtnet_ids,
	.ops = &ops,
	.owner = THIS_MODULE,
};

static int __init pcie_virtnet_driver_init(void)
{
	int ret;

	ret = pci_epf_register_driver(&virtnet_driver);
	if (ret) {
		pr_err("Failed to register pci virtnet driver --> %d\n", ret);
		return ret;
	}

	return 0;
}
module_init(pcie_virtnet_driver_init);

static void __exit pcie_virtnet_driver_exit(void)
{
	struct pci_epf_virtnet *epf_virtnet, *next;

	list_for_each_entry_safe(epf_virtnet, next, &epf_virtnet_list, node) {
		atomic_notifier_chain_unregister(&epf_virtnet->epf->epc->notifier,
						 &epf_virtnet->epf->nb);
		pci_epf_virtnet_core_exit(epf_virtnet->epf);
		pci_epf_free_bar(epf_virtnet->epf);
		list_del(&epf_virtnet->node);
		devm_kfree(&epf_virtnet->epf->dev, epf_virtnet);
	}
	pci_epf_unregister_driver(&virtnet_driver);
}
module_exit(pcie_virtnet_driver_exit);

MODULE_DESCRIPTION("PCIE VIRTNET EPF");
MODULE_AUTHOR("Xuran Yang <xuran.yang@bst.ai>");
MODULE_LICENSE("GPL v2");
