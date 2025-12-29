// SPDX-License-Identifier: GPL-2.0
/*
 * System Control and Management Interface (SCMI) Message msgbox/HVC
 * Transport driver
 *
 *  Copyright 2020 NXP
  * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <msgbox_send.h>
#include "common.h"
#include <linux/bst_samphore.h>
#include "bst/ipc_interface.h"




static int scmi_ipc_send(struct scmi_chan_info *cinfo)
{
	ipc_msg msg;

	msg.data = 0;
	msg.cmd = 0;
	msg.type = IPC_MSG_TYPE_SIGNAL;

	ipc_send_sync(cinfo->ipc_session, &msg);

	return 0;
}




static bool msgbox_chan_available(struct device *dev, int idx)
{
	struct device_node *np = of_parse_phandle(dev->of_node, "shmem", 0);
	if (!np)
		return false;

	of_node_put(np);
	
	return true;
}




static int msgbox_chan_setup(struct scmi_chan_info *cinfo, struct device *dev,
			  bool tx)
{
	struct device *cdev = cinfo->dev;
	struct scmi_msgbox *scmi_info;
	resource_size_t size;
	struct resource res;
	struct device_node *np;
	int ret;


	#ifdef CONFIG_BST_C1200_IVI
	uint32_t cpu_id = IPC_CORE_ARM0;
	#endif

	#ifdef CONFIG_BST_C1200_ADAS
	uint32_t cpu_id = IPC_CORE_ARM2;
	#endif

	#ifdef CONFIG_BST_C1200_DB
	uint32_t cpu_id = IPC_CORE_DB0;
	#endif


	if (!tx)
		return -ENODEV;


	scmi_info = devm_kzalloc(dev, sizeof(*scmi_info), GFP_KERNEL);
	if (!scmi_info)
		return -ENOMEM;

	np = of_parse_phandle(cdev->of_node, "shmem", 0);
	ret = of_address_to_resource(np, 0, &res);
	of_node_put(np);
	if (ret) {
		dev_err(cdev, "failed to get SCMI Tx shared memory\n");
		return ret;
	}

	size = resource_size(&res);
	scmi_info->shmem = devm_ioremap_wc(dev, res.start, size);
	if (!scmi_info->shmem) {
		dev_err(dev, "failed to ioremap SCMI Tx shared memory\n");
		return -EADDRNOTAVAIL;
	}

	memset(scmi_info->shmem,0,size);

	iowrite32(0x1, &scmi_info->shmem->channel_status);
	

	scmi_info->cinfo = cinfo;

	spin_lock_init(&scmi_info->shmem_lock);

	cinfo->transport_info = scmi_info;


	cinfo->ipc_session =  ipc_init(IPC_CORE_SAFE,cpu_id,NULL);


	return 0;
}

static int msgbox_chan_free(int id, void *p, void *data)
{
	struct scmi_chan_info *cinfo = p;
	struct scmi_msgbox *scmi_info = cinfo->transport_info;
	cinfo->transport_info = NULL;
	scmi_info->cinfo = NULL;

	scmi_free_channel(cinfo, data, id);

	return 0;
}




static bool
msgbox_poll_done(struct scmi_chan_info *cinfo, struct scmi_xfer *xfer)
{
	struct scmi_msgbox *scmi_info = cinfo->transport_info;
	return shmem_poll_done(scmi_info->shmem, xfer);
}





extern uint64_t __iomem *base;
static int msgbox_send_message(struct scmi_chan_info *cinfo,
			    struct scmi_xfer *xfer)
{
	int ret = 0;
	unsigned long flags;

	struct scmi_msgbox *scmi_mbx = cinfo->transport_info;
	


	spin_lock_irqsave(&scmi_mbx->shmem_lock, flags);

	shmem_tx_prepare(scmi_mbx->shmem, xfer, cinfo);

	scmi_ipc_send(cinfo);

	scmi_rx_callback(scmi_mbx->cinfo, shmem_read_header(scmi_mbx->shmem), NULL);

	spin_unlock_irqrestore(&scmi_mbx->shmem_lock, flags);


	return ret;
}

static void msgbox_fetch_response(struct scmi_chan_info *cinfo,
			       struct scmi_xfer *xfer)
{
	struct scmi_msgbox *scmi_info = cinfo->transport_info;
	shmem_fetch_response(scmi_info->shmem, xfer);
}

static void msgbox_clear_channel(struct scmi_chan_info *cinfo)
{
	struct scmi_msgbox *smbox = cinfo->transport_info;
	shmem_clear_channel(smbox->shmem);
}


static int msgbox_resume_channel(struct scmi_chan_info *cinfo)
{
	struct scmi_msgbox *smbox = cinfo->transport_info;


	iowrite32(0x1, &smbox->shmem->channel_status);

	return 0;
}


static int msgbox_get_count(struct scmi_chan_info *cinfo)
{
	struct scmi_msgbox *smbox = cinfo->transport_info;

	return ioread32(&smbox->shmem->reserved1[0]);
}



static const struct scmi_transport_ops scmi_msgbox_ops = {
	.chan_available = msgbox_chan_available,
	.chan_setup = msgbox_chan_setup,
	.chan_free = msgbox_chan_free,
	.send_message = msgbox_send_message,
	.fetch_response = msgbox_fetch_response,
	.poll_done = msgbox_poll_done,
    .clear_channel = msgbox_clear_channel,
	.chan_resume = msgbox_resume_channel,
	.get_count = msgbox_get_count,
};

const struct scmi_desc scmi_msgbox_desc = {
	.ops = &scmi_msgbox_ops,
	.max_rx_timeout_ms = 500,
	.max_msg = 20,
	.max_msg_size = 2048,
	.force_polling = true,
};
