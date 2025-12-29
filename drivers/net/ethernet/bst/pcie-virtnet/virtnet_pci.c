// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023~2024 Black Sesame Technologies, Inc.
 *
 * Author: Xuran Yang <xuran.yang@bst.ai>
 */

#define pr_fmt(fmt) "virtnet-pci: " fmt
#include "queue.h"
#include "virtnet.h"
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/pci-epf.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/pci_regs.h>
#include <linux/skbuff.h>
#include <linux/stddef.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#define DRV_MODULE_NAME "pci-virtnet"

static DEFINE_IDA(pci_virtnet_ida);

/* This struct is for free all skb
 * when module exit
 */
static LIST_HEAD(skb_list);
struct virtnet_skb {
	struct list_head node;
	struct sk_buff *skb;
	u32 skb_len;
	dma_addr_t paddr;
};

struct virtnet_int_desc {
	bool is_valid;
	void __iomem *base;
	u32 msg;
	u32 count;
};

struct virtnet_adapter {
	struct pci_dev *pdev;
	struct net_device *netdev;
	struct napi_struct napi;
	char name[24];
	void __iomem *base;
	void __iomem *bar[PCI_STD_NUM_BARS];
	u32 num_irqs;
	unsigned long flags;
	struct task_struct *wait_ep_task;
	struct work_struct maintain_work;

	struct virtnet_queue rc_queue; /* RC->EP */
	struct virtnet_queue ep_queue; /* EP->RC */

	struct virtnet_int_desc int_desc;
};

static void pci_virtnet_empty_queue(struct virtnet_adapter *adapter, struct virtnet_queue *queue);
static int pci_virtnet_fill_queue(struct virtnet_adapter *adapter, struct virtnet_queue *queue);
static struct virtnet_skb *virtnet_alloc_skb(struct virtnet_adapter *adapter);
static void virtnet_free_skb_node(struct virtnet_skb *skb_node);

static void pci_virtnet_raise_irq(struct virtnet_adapter *adapter)
{
	struct virtnet_bar *reg = adapter->base;

	reg->ep_int_cnt++;
	wmb();	/* Makesure int_cnt reg is set */
	if (!adapter->int_desc.is_valid)
		return;
	if (reg->ep_int_mask & (1 << VNET_NOTIFY_EP))
		return;
	/* Ensure that the queue is written before trigger
	 * the interruputs
	 */
	writel(adapter->int_desc.msg, adapter->int_desc.base);
}

static netdev_tx_t virtnet_xmit_frame(struct sk_buff *skb, struct net_device *netdev)
{
	struct virtnet_adapter *adapter = netdev_priv(netdev);
	struct device *dev = &adapter->pdev->dev;
	struct virtnet_queue *queue = &adapter->rc_queue;
	struct virtnet_desc *desc;
	dma_addr_t paddr;

	desc = get_free_skb_descriptor(queue);
	if (unlikely(!desc))
		return NETDEV_TX_BUSY;

	paddr = dma_map_single(dev, skb->data, skb->len, DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(dev, paddr))) {
		pr_err("dma_map_single failed\n");
		goto err_dma_map;
	}

	desc->skb = skb;
	desc->data = paddr;
	desc->size = skb->len;
	set_desc_owner_bit(queue, desc, true);
	wmb();	/* Makesure descripter is set */
	pci_virtnet_raise_irq(adapter);

	netdev->stats.tx_packets++;
	netdev->stats.tx_bytes += skb->len;

	return NETDEV_TX_OK;
err_dma_map:
	netdev->stats.tx_dropped++;
	dev_kfree_skb(skb);
	return NETDEV_TX_OK;
}

/* When TCP sends, the NIC driver needs to free the SKB immediately after sending,
 * so that the TCP sending window is released.
 */
static void pci_virtnet_maintain_work(struct work_struct *work)
{
	struct virtnet_adapter *adapter =
		container_of(work, struct virtnet_adapter, maintain_work);
	struct device *dev = &adapter->pdev->dev;
	struct virtnet_queue *queue = &adapter->rc_queue;
	struct virtnet_desc *desc = NULL;

	while (true) {
		desc = get_maintain_descriptor(&adapter->rc_queue);
		if (!desc)
			break;
		dma_unmap_single(dev, desc->data, desc->size, DMA_TO_DEVICE);
		dev_kfree_skb(desc->skb);
		desc->skb = NULL;
		set_desc_taken_bit(queue, desc, F_TAKEN, false);
	}
}

static irqreturn_t pci_virtnet_xmit_done(int irq, void *arg)
{
	struct virtnet_adapter *adapter = arg;

	schedule_work(&adapter->maintain_work);

	return IRQ_HANDLED;
}

static int virtnet_open(struct net_device *netdev)
{
	struct virtnet_adapter *adapter = netdev_priv(netdev);

	__clear_bit(__VNET_DOWN, &adapter->flags);
	napi_enable(&adapter->napi);
	netif_carrier_on(netdev);
	return 0;
}

static int virtnet_close(struct net_device *netdev)
{
	struct virtnet_adapter *adapter = netdev_priv(netdev);

	__set_bit(__VNET_DOWN, &adapter->flags);
	napi_disable(&adapter->napi);
	netif_carrier_off(netdev);
	return 0;
}

static int virtnet_change_mtu(struct net_device *netdev, int new_mtu)
{
	struct virtnet_adapter *adapter = netdev_priv(netdev);
	struct virtnet_queue *queue = &adapter->ep_queue;
	struct virtnet_bar *reg = (struct virtnet_bar *)adapter->base;

	if (!test_bit(__VNET_DOWN, &adapter->flags))
		return -1;

	/* Enter mainten */
	reg->rc_status |= __VNET_MAINTEN;
	msleep(100);
	pci_virtnet_empty_queue(adapter, queue);
	netdev->mtu = new_mtu;
	netdev_update_features(netdev);
	pci_virtnet_fill_queue(adapter, queue);
	reg->rc_status &= ~(__VNET_MAINTEN);

	return 0;
}

static const struct net_device_ops vnetdev_ops = {
	.ndo_start_xmit = virtnet_xmit_frame,
	.ndo_change_mtu = virtnet_change_mtu,
	.ndo_open = virtnet_open,
	.ndo_stop = virtnet_close,
};

static void pci_virtnet_setup(struct net_device *netdev)
{
	ether_setup(netdev);
	netdev->flags |= IFF_NOARP;
	netdev->features |= (NETIF_F_HW_CSUM | NETIF_F_HIGHDMA | NETIF_F_GSO | NETIF_F_TSO);
	netdev->hw_features = netdev->features;
	netdev->mtu = 65536;
	netdev->max_mtu = VNET_MAX_MTU;
	netdev->netdev_ops = &vnetdev_ops;
}

static int pci_virtnet_get_intmsg(struct virtnet_adapter *adapter)
{
	struct virtnet_bar *bar = (struct virtnet_bar *)adapter->base;
	u32 bar_num = bar->ep_int_bar;
	unsigned long ep_status = bar->ep_status;

	if (test_bit(__VNET_POLLING, &ep_status)) {
		adapter->int_desc.is_valid = false;
	} else {
		adapter->int_desc.is_valid = true;
		adapter->int_desc.base = adapter->bar[bar_num] + bar->ep_int_base;
		adapter->int_desc.msg = bar->ep_int_msg;
		adapter->int_desc.count = bar->ep_int_cnt;
	}

	return 0;
}

static irqreturn_t pci_virtnet_irq_handler(int irq, void *arg)
{
	struct virtnet_adapter *adapter = arg;
	struct virtnet_bar *reg = (struct virtnet_bar *)adapter->base;

	reg->rc_int_mask |= (1 << VNET_NOTIFY_MSI);
	if (likely(napi_schedule_prep(&adapter->napi)))
		__napi_schedule(&adapter->napi);
	else
		reg->rc_int_mask &= ~(1 << VNET_NOTIFY_MSI);

	return IRQ_HANDLED;
}

static int pci_virtnet_clean_rx(struct virtnet_adapter *adapter, int budget, int *cur)
{
	struct device *dev = &adapter->pdev->dev;
	struct net_device *netdev = adapter->netdev;
	struct virtnet_queue *queue = &adapter->ep_queue;
	struct virtnet_desc *desc;
	struct virtnet_skb *skb_node;
	struct sk_buff *skb;
	int i;

	for (i = 0; i < budget; i++) {
		desc = get_owned_descriptor(queue);
		if (!desc)
			return 0;
		*cur += 1;
		skb_node = desc->skb;
		skb = skb_node->skb;
		dma_unmap_single(dev, skb_node->paddr, skb_node->skb_len, DMA_FROM_DEVICE);

		skb_put(skb, desc->size);
		skb->protocol = eth_type_trans(skb, netdev);
		skb->dev = netdev;
		skb->ip_summed = CHECKSUM_UNNECESSARY; /* don't check it */

		netdev->stats.rx_packets++;
		netdev->stats.rx_bytes += skb->len;
		napi_gro_receive(&adapter->napi, skb);
		virtnet_free_skb_node(skb_node);

		/* Alloc new skb */
		skb_node = virtnet_alloc_skb(adapter);
		if (unlikely(!skb_node)) {
			pr_err("alloc skb failed\n");
			goto err_alloc_skb;
		}

		desc->skb = skb_node;
		desc->data = skb_node->paddr;
		desc->size = skb_node->skb_len;
		wmb();	/* Makesure descripter is set */
		set_desc_owner_bit(queue, desc, false);
	}
	return 1;

err_alloc_skb:
	desc->skb = NULL;
	/* How to fix? */
	set_desc_owner_bit(queue, desc, false);
	pr_err("unknown err when receive\n");
	return -1;
}

static int pci_virtnet_poll(struct napi_struct *napi, int budget)
{
	struct virtnet_adapter *adapter = container_of(napi, struct virtnet_adapter, napi);
	struct virtnet_bar *reg = (struct virtnet_bar *)adapter->base;
	int tx_clean_complete = 0, work_done = 0;

	tx_clean_complete = pci_virtnet_clean_rx(adapter, budget, &work_done);

	if (work_done == budget)
		return budget;

	if (likely(napi_complete_done(napi, work_done)))
		reg->rc_int_mask &= ~(1 << VNET_NOTIFY_MSI);

	return work_done;
}

static void pci_virtnet_release_irq(struct virtnet_adapter *adapter)
{
	int i;
	struct pci_dev *pdev = adapter->pdev;
	struct device *dev = &pdev->dev;

	for (i = 0; i < adapter->num_irqs; i++)
		devm_free_irq(dev, pci_irq_vector(pdev, i), adapter);

	adapter->num_irqs = 0;
}

static void pci_virtnet_free_irq_vectors(struct virtnet_adapter *adapter)
{
	struct pci_dev *pdev = adapter->pdev;

	pci_free_irq_vectors(pdev);
	adapter->num_irqs = 0;
}

static void pci_virtnet_request_irq(struct virtnet_adapter *adapter)
{
	int err;
	struct pci_dev *pdev = adapter->pdev;
	struct device *dev = &pdev->dev;
	int irq;

	/* recv notify */
	irq = pci_irq_vector(pdev, VNET_NOTIFY_MSI);
	err = devm_request_irq(dev, irq, pci_virtnet_irq_handler, IRQF_SHARED, adapter->name,
			       adapter);
	if (err)
		pr_err("request irq failed\n");

	/* xmit done notify */
	irq = pci_irq_vector(pdev, VNET_NOTIFY_XMIT_DONE);
	err = devm_request_irq(dev, irq, pci_virtnet_xmit_done, IRQF_SHARED, adapter->name,
			       adapter);
	if (err)
		pr_err("request irq failed\n");
}

static struct virtnet_skb *virtnet_alloc_skb(struct virtnet_adapter *adapter)
{
	dma_addr_t paddr;
	struct sk_buff *skb;
	struct virtnet_skb *skb_node;
	struct device *dev = &adapter->pdev->dev;
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
	struct device *dev = &adapter->pdev->dev;

	dma_unmap_single(dev, skb_node->paddr, skb_node->skb_len, DMA_FROM_DEVICE);
	dev_kfree_skb(skb_node->skb);
	virtnet_free_skb_node(skb_node);
}

static int pci_virtnet_fill_queue(struct virtnet_adapter *adapter, struct virtnet_queue *queue)
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
		desc->skb = skb_node;
		desc->data = skb_node->paddr;
		desc->size = skb_node->skb_len;
		set_desc_owner_bit(queue, desc, false);
	}
	return 0;

err_alloc_skb:
	return -1;
}

static void pci_virtnet_empty_queue(struct virtnet_adapter *adapter, struct virtnet_queue *queue)
{
	struct virtnet_desc *desc;
	int p = -1;

	while ((desc = get_each_desc(queue, &p)) != NULL) {
		desc->skb = NULL;
		desc->data = 0;
		desc->size = 0;
	}
}

static void pci_virtnet_free_all_skb(struct virtnet_adapter *adapter)
{
	struct virtnet_skb *skb_node, *next;

	list_for_each_entry_safe(skb_node, next, &skb_list, node)
		virtnet_free_skb(adapter, skb_node);
}

static int pci_virtnet_core_init(void *arg)
{
	int err;
	struct virtnet_adapter *adapter = arg;
	struct virtnet_bar *reg = adapter->base;
	struct net_device *netdev = adapter->netdev;

	__set_bit(__VNET_WAITING, &adapter->flags);

	while (true) {
		if ((reg->ep_status & 0xFFFF0000) == __VNET_MAGIC &&
		    reg->ep_status & (1 << __VNET_INIT)) {
			pr_info("ep virtnet is ready, continue to initialize\n");
			break;
		}
		if (kthread_should_stop()) {
			__clear_bit(__VNET_WAITING, &adapter->flags);
			return 0;
		}

		usleep_range(200, 500);
	}

	init_queue(&adapter->rc_queue, adapter->base + VNET_RC_DESC_HEAD, VNET_DESC_NUM);
	init_queue(&adapter->ep_queue, adapter->base + VNET_EP_DESC_HEAD, VNET_DESC_NUM);
	/* RC need fill descriptor first */
	pci_virtnet_fill_queue(adapter, &adapter->ep_queue);

	pci_virtnet_get_intmsg(adapter);

	err = register_netdev(netdev);
	if (err)
		goto err_register_netdev;
	reg->rc_status = 0;
	__set_bit(__VNET_INIT, &adapter->flags);
	__clear_bit(__VNET_WAITING, &adapter->flags);
	return 0;

err_register_netdev:
	return -1;
}

static int pci_virtnet_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	int err;
	int irq;
	int id;
	enum pci_barno bar;
	void __iomem *base;
	struct device *dev = &pdev->dev;
	struct net_device *netdev;
	struct virtnet_adapter *adapter = NULL;
	struct virtnet_bar *reg;

	if (pci_is_bridge(pdev))
		return -ENODEV;

	if (dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64)) != 0) {
		dev_err(dev, "Cannot set DMA mask\n");
		return -EINVAL;
	}

	err = pci_enable_device(pdev);
	if (err) {
		dev_err(dev, "Cannot enable PCI device\n");
		return err;
	}

	err = pci_request_regions(pdev, DRV_MODULE_NAME);
	if (err) {
		dev_err(dev, "Cannot obtain PCI resources\n");
		goto err_disable_pdev;
	}

	pci_set_master(pdev);
	err = pci_save_state(pdev);
	if (err)
		goto err_alloc_etherdev;

	netdev = alloc_netdev(sizeof(struct virtnet_adapter), "veth%d", NET_NAME_UNKNOWN,
			      pci_virtnet_setup);
	if (!netdev)
		goto err_alloc_etherdev;
	SET_NETDEV_DEV(netdev, &pdev->dev);

	pci_set_drvdata(pdev, netdev);
	adapter = netdev_priv(netdev);
	adapter->netdev = netdev;
	adapter->pdev = pdev;
	id = ida_simple_get(&pci_virtnet_ida, 0, 0, GFP_KERNEL);
	if (id < 0) {
		err = id;
		dev_err(dev, "Unable to get id\n");
		goto err_get_id;
	}
	netif_napi_add(netdev, &adapter->napi, pci_virtnet_poll);
	snprintf(adapter->name, sizeof(adapter->name), DRV_MODULE_NAME ".%d", id);

	for (bar = 0; bar < VNET_MAPED_BAR; bar++) {
		if (pci_resource_flags(pdev, bar) & IORESOURCE_MEM) {
			base = pci_ioremap_bar(pdev, bar);
			if (!base) {
				dev_err(dev, "Failed to read BAR%d\n", bar);
				WARN_ON(bar == VNET_USED_BAR);
				goto err_iounmap;
			}
			adapter->bar[bar] = base;
		}
	}

	adapter->base = adapter->bar[VNET_USED_BAR];
	if (!adapter->base) {
		err = -ENOMEM;
		dev_err(dev, "Cannot perform PCI test without BAR%d\n", VNET_USED_BAR);
		goto err_iounmap;
	}
	reg = adapter->base;

	INIT_WORK(&adapter->maintain_work, pci_virtnet_maintain_work);
	irq = pci_alloc_irq_vectors(pdev, 2, 2, PCI_IRQ_MSIX);
	if (irq < 0) {
		dev_err(dev, "Failed to get MSI-X interrupts\n");
		goto err_alloc_irq;
	}
	adapter->num_irqs = irq;
	pci_virtnet_request_irq(adapter);

	adapter->wait_ep_task = kthread_run(pci_virtnet_core_init, adapter, "virtnet_init");

	return 0;

err_get_id:
err_alloc_irq:
	pci_virtnet_free_irq_vectors(adapter);
	pci_release_regions(pdev);
err_iounmap:
	for (bar = 0; bar < VNET_MAPED_BAR; bar++) {
		if (adapter->bar[bar])
			pci_iounmap(pdev, adapter->bar[bar]);
	}
err_alloc_etherdev:
err_disable_pdev:
	pci_disable_device(pdev);

	return err;
}

static void pci_virtnet_remove(struct pci_dev *pdev)
{
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct virtnet_adapter *adapter = netdev_priv(netdev);
	struct virtnet_bar *reg = adapter->base;

	if (!test_bit(__VNET_INIT, &adapter->flags) && test_bit(__VNET_WAITING, &adapter->flags))
		kthread_stop(adapter->wait_ep_task);

	reg->rc_status |= __VNET_MAINTEN;
	msleep(100);

	unregister_netdev(netdev);
	pci_virtnet_free_all_skb(adapter);
	pci_virtnet_release_irq(adapter);
	pci_virtnet_free_irq_vectors(adapter);

	free_netdev(netdev);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
}

static const struct default_data {
	int data;
} default_data = {
	.data = 0,
};

static const struct pci_device_id pci_virtnet_tbl[] = {
	{
		PCI_DEVICE(PCI_VENDOR_ID_BST, PCI_DEVICE_ID_BST_VNET),
		.driver_data = (kernel_ulong_t)&default_data,
	},
	{},
};
MODULE_DEVICE_TABLE(pci, pci_virtnet_tbl);

static struct pci_driver pci_bst_virtnet_driver = {
	.name = DRV_MODULE_NAME,
	.id_table = pci_virtnet_tbl,
	.probe = pci_virtnet_probe,
	.remove = pci_virtnet_remove,
};
module_pci_driver(pci_bst_virtnet_driver);

MODULE_DESCRIPTION("PCI VIRTUAL NET");
MODULE_AUTHOR("Xuran Yang <xuran.yang@bst.ai>");
MODULE_LICENSE("GPL v2");
