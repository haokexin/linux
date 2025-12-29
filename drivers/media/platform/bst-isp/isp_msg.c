// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/delay.h>

#ifdef CONFIG_BST_IPC
#include <bst/ipc_interface.h>
#endif

#ifdef CONFIG_BST_IPC_MSGBX
#include "msgbox/video_client.h"
#endif

#include "isp_msg.h"

#ifdef CONFIG_BST_IPC
static int ipc_recv_entry(void *data)
{
	struct isp_device *isp;

	isp = (struct isp_device *)data;
	while (!kthread_should_stop()) {
		int32_t rv;
		struct _ipc_msg msg;

		rv = ipc_recv(isp->ipc.sid, &msg, -1);
		if (rv < 0) {
			dev_err_ratelimited(
				isp->dev,
				"Failed to receive IPC message, rv: %d\n", rv);
			continue;
		}
		isp->ops.msg_rx(isp, msg.data, msg.timestamp);
	}

	return 0;
}

static int ipc_tx(struct isp_device *isp, struct media_command *mc)
{
	ipc_msg msg;

	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = isp_msg_addr_dma(isp, mc);

	return ipc_send_sync(isp->ipc.sid, &msg);
}

int isp_msg_init_ipc(struct isp_device *isp)
{
	struct device *dev;

	dev = isp->dev;
	isp->ipc.sid = ipc_init(IPC_CORE_ISP, isp->ipc.cpu, dev);
	if (isp->ipc.sid < 0) {
		dev_err(dev, "Failed to init IPC, rv: %d\n", isp->ipc.sid);
		return -1;
	}

	/* TODO: Support define run on which CPU: ipc-rx-cpu */
	isp->ipc.rx_task = kthread_run(ipc_recv_entry, isp, "isp-rx-msg");
	if (IS_ERR(isp->ipc.rx_task)) {
		dev_err(dev, "Failed to create ipc-rx-task, rv: %ld\n",
			PTR_ERR(isp->ipc.rx_task));
		return -1;
	}

	isp->ops.msg_tx = ipc_tx;

	return 0;
}

void isp_msg_exit_ipc(struct isp_device *isp)
{
	kthread_stop(isp->ipc.rx_task);
	ipc_close(isp->ipc.sid);
}
#endif /* CONFIG_BST_IPC */

#ifdef CONFIG_BST_IPC_MSGBX
static void msgbox_recv_cb(u32 msgAddr, void *ext, const ext_info_t *info)
{
	struct isp_device *isp;
	struct msgbx_rx_msg *rx_msg;

	isp = (struct isp_device *)ext;
	rx_msg = devm_kzalloc(isp->dev, sizeof(*rx_msg), GFP_KERNEL);
	rx_msg->data = msgAddr;
	rx_msg->timestamp = info->timestamp;
	INIT_LIST_HEAD(&rx_msg->queue_node);

	mutex_lock(&isp->msgbx.rx_queue.lock);
	list_add_tail(&rx_msg->queue_node, &isp->msgbx.rx_queue.head);
	complete(&isp->msgbx.rx_comp);
	mutex_unlock(&isp->msgbx.rx_queue.lock);
}

static int msgbox_rx_entry(void *data)
{
	struct isp_device *isp;
	struct msgbx_rx_msg *rx_msg;
	struct msgbx_rx_msg *p, *n;

	isp = (struct isp_device *)data;
	while (!kthread_should_stop()) {
		rx_msg = ERR_PTR(-ENOENT);
		mutex_lock(&isp->msgbx.rx_queue.lock);
		list_for_each_entry_safe(p, n, &isp->msgbx.rx_queue.head,
					 queue_node) {
			list_del(&p->queue_node);
			rx_msg = p;
			break;
		}
		mutex_unlock(&isp->msgbx.rx_queue.lock);

		if (IS_ERR(rx_msg)) {
			wait_for_completion_interruptible(&isp->msgbx.rx_comp);
			reinit_completion(&isp->msgbx.rx_comp);
			/* Ensure completion is cleared before continuing */
			smp_mb__after_atomic();
			continue;
		}

		isp->ops.msg_rx(isp, rx_msg->data, rx_msg->timestamp);
		devm_kfree(isp->dev, rx_msg);
	}

	return 0;
}

static void on_avail_changed(bool flag, void *ext)
{
	struct isp_device *isp;

	isp = (struct isp_device *)ext;

	if (flag)
		complete(&isp->msgbx.avail_comp);

	dev_info(isp->dev, "Msgbox of firmware is %savailable\n",
		 flag ? "" : "un");
}

static void on_isp2arm_sub_reply(int32_t err, void *ext, const ext_info_t *info)
{
	struct isp_device *isp;

	isp = (struct isp_device *)ext;
	isp->msgbx.subscribed = true;
	dev_info(isp->dev, "Receive firmware sub reply, err: %d\n", err);
}

static void on_isp2arm_unsub_reply(int32_t err, void *ext,
				   const ext_info_t *info)
{
	struct isp_device *isp;

	isp = (struct isp_device *)ext;
	dev_info(isp->dev, "Receive firmware unsub reply, err: %d\n", err);
}

static int msgbox_tx(struct isp_device *isp, struct media_command *cmd)
{
	int rv;
	u32 message;
	video_error_e_t err = 0;
	des_buf_t buffer = { 0 };

	if (!isp->msgbx.client) {
		dev_err_ratelimited(isp->dev,
				    "No valid msgbox client to send message\n");
		return -ENODEV;
	}

	/* NOTE: Drop message implicitly when client is not subscribed,
	 * this is used to support suspend/resume
	 */
	if (isp->shared->fw.stage < FS_BOOTED || !isp->msgbx.subscribed)
		return -EPERM;

	rv = isp->msgbx.client->video_client.arm2isp_sync(
		isp_msg_addr_dma(isp, cmd), &message, &err, 1000, &buffer);
	if (rv < 0) {
		dev_err_ratelimited(isp->dev,
				    "Failed to send message, rv: %d\n", rv);
		return -EIO;
	}

	return 0;
}

static int msgbox_handshake_entry(void *data)
{
	int rv;
	struct isp_device *isp;

	isp = (struct isp_device *)data;

	wait_for_completion(&isp->msgbx.avail_comp);
	dev_info(isp->dev, "Handshake with firmware successfully\n");

	while (!isp->msgbx.subscribed) {
		rv = isp->msgbx.client->video_client.isp2arm_sub(
			msgbox_recv_cb, isp, NULL, on_isp2arm_sub_reply, isp);
		if (rv < 0) {
			dev_err(isp->dev, "Failed to subscribe\n");
			return rv;
		}

		msleep(500);
	}

	return 0;
}

int isp_msg_init_msgbox(struct isp_device *isp)
{
	int rv;
	ipc_inf_version_t version;
	struct task_struct *msgbx_handshake_task;
	struct device *dev;

	dev = isp->dev;
	mutex_init(&isp->msgbx.rx_queue.lock);
	INIT_LIST_HEAD(&isp->msgbx.rx_queue.head);
	init_completion(&isp->msgbx.avail_comp);
	init_completion(&isp->msgbx.rx_comp);

	isp->msgbx.client = isp_msgbx_client_init(&isp->msgbx.data);
	if (!isp->msgbx.client) {
		dev_err(dev, "Failed to init msgbox client\n");
		return -1;
	}

	version = isp->msgbx.client->video_client.version();
	dev_info(dev, "Msgbox interface version: %d.%d\n", version.major,
		 version.minor);

	rv = isp->msgbx.client->video_client.register_avail_changed(
		on_avail_changed, isp);
	if (rv < 0) {
		dev_err(dev, "Failed to register avail changed, rv: %d\n", rv);
		return rv;
	}

	rv = isp->msgbx.client->start();
	if (rv < 0) {
		dev_err(dev, "Failed to start msgbox client, rv: %d\n", rv);
		return rv;
	}

	isp->ops.msg_tx = msgbox_tx;
	isp->msgbx.rx_task = kthread_run(msgbox_rx_entry, isp, "isp-rx-msg");
	if (IS_ERR(isp->msgbx.rx_task)) {
		dev_err(dev, "Failed to create msgbox-rx-task, rv: %ld\n",
			PTR_ERR(isp->msgbx.rx_task));
		return -1;
	}

	msgbx_handshake_task =
		kthread_run(msgbox_handshake_entry, isp, "isp-handshake");
	if (IS_ERR(msgbx_handshake_task)) {
		dev_err(dev,
			"Failed to create msgbox-handshake-task, rv: %ld\n",
			PTR_ERR(msgbx_handshake_task));
		return -1;
	}

	return 0;
}

void isp_msg_exit_msgbox(struct isp_device *isp)
{
	int rv;
	struct device *dev;

	dev = isp->dev;
	if (!isp->msgbx.client) {
		dev_err(dev, "No valid msgbox client to stop\n");
		return;
	}

	rv = isp->msgbx.client->video_client.isp2arm_unsub(
		on_isp2arm_unsub_reply, isp);
	if (rv < 0) {
		dev_err(dev, "Failed to register unsubscribe isp, rv: %d\n",
			rv);
		return;
	}

	rv = isp->msgbx.client->stop();
	if (rv < 0) {
		dev_err(dev, "Failed to stop msgbox client, rv: %d\n", rv);
		return;
	}

	rv = isp_msgbx_client_destroy();
	if (rv < 0)
		dev_err(dev, "Failed to destroy msgbox client, rv: %d\n", rv);
}
#endif /* CONFIG_BST_IPC_MSGBX */

static int isp_fw_init_fn(void *data)
{
	struct isp_device *isp;

	isp = (struct isp_device *)data;

	return isp_fw_init(isp);
}

int isp_msg_resume(struct isp_device *isp)
{
	struct task_struct *task;
	struct device *dev;

	dev = isp->dev;

#ifdef CONFIG_BST_IPC_MSGBX
	if (!isp->use_ipc) {
		isp->msgbx.subscribed = false;
		task = kthread_run(msgbox_handshake_entry, isp,
				   "isp-handshake");
		if (IS_ERR(task)) {
			dev_err(dev,
				"Failed to create msgbox-handshake-task, rv: %ld\n",
				PTR_ERR(task));
			return PTR_ERR(task);
		}
	}
#endif

	if (isp->role == ROLE_MASTER) {
		task = kthread_run(isp_fw_init_fn, isp, "isp-fw-init");
		if (IS_ERR(task)) {
			dev_err(dev, "Failed to create isp-fw-init, rv: %ld\n",
				PTR_ERR(task));
			return PTR_ERR(task);
		}
	}

	return 0;
}

static struct media_command *get_media_cmds(struct isp_device *isp, int num)
{
	struct media_command *mc;

	mutex_lock(&isp->msg.tx_lock);
	mc = isp->msg.cmds + isp->msg.cmd_index;
	isp->msg.cmd_index += num;
	isp->msg.cmd_index %= isp->msg.cmd_size;
	mutex_unlock(&isp->msg.tx_lock);

	return mc;
}

static inline struct media_command *get_media_cmd(struct isp_device *isp)
{
	return get_media_cmds(isp, 1);
}

/* TODO: Support threaded */
int isp_msg_tx(struct isp_device *isp, struct media_command *mc, bool ack)
{
	int rv;
	int tries;
	struct device *dev;
	u32 dma_addr;
	u32 minor;

	/* Drop message when system is not running */
	if (system_state > SYSTEM_RUNNING)
		return 0;

	dev = isp->dev;
	dma_addr = isp_msg_addr_dma(isp, mc);
	minor = mc->cmd_hdr.hdr_info.cmd_type_minor;

	mutex_lock(&isp->msg.tx_lock);
	++isp->msg.tx_all;
	if (ack)
		reinit_completion(&isp->msg.tx_comp);
	for (tries = 1; tries <= ISP_MSG_TX_TRY_TIMES; ++tries) {
		rv = isp->ops.msg_tx(isp, mc);
		if (!rv)
			break;
		usleep_range(ISP_MSG_TX_TRY_DELAY, ISP_MSG_TX_TRY_DELAY + 10);
	}
	if (likely(!rv)) {
		++isp->msg.tx_done;
	} else {
		++isp->msg.tx_fail;
		if (rv != -EPERM)
			dev_err_ratelimited(
				dev,
				"MSGTX: sn: %8lu/%8lu, DMA: 0x%08X, minor: 0x%02X, tries: %d, rv: %d, FAILED\n",
				isp->msg.tx_all, isp->msg.tx_fail, dma_addr,
				minor, tries, rv);
		if (ack)
			complete(&isp->msg.tx_comp);
	}

	if (ack) {
		unsigned long timeout;

		timeout = wait_for_completion_timeout(
			&isp->msg.tx_comp,
			msecs_to_jiffies(ISP_MSG_ACK_TIMEOUT));
		if (timeout == 0) {
			rv = -ETIMEDOUT;
			dev_err_ratelimited(
				dev,
				"MSGTX: sn: %8lu/%8lu, DMA: 0x%08X, minor: 0x%02X, tries: %d, rv: %d, timed out\n",
				isp->msg.tx_all, isp->msg.tx_done, dma_addr,
				minor, tries, rv);
		}
	}
	mutex_unlock(&isp->msg.tx_lock);
	dev_dbg(dev,
		"MSGTX: sn: %8lu/%8lu, DMA: 0x%08X, minor: 0x%02X, tries: %d, rv: %d\n",
		isp->msg.tx_all, isp->msg.tx_done, dma_addr, minor, tries, rv);

	return rv;
}

int isp_msg_dev_start(struct isp_device *isp)
{
	struct media_command *mc;
	isp_start_t *start_cmd;

	mc = get_media_cmd(isp);
	memset(mc, 0, sizeof(*mc));
	mc->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_START;

	start_cmd = (isp_start_t *)(&mc->user_cmd_data[0]);
	start_cmd->reconfDDRBase = isp->fw.rsv_dma;
	start_cmd->reconfDDRSize = isp->fw.rsv_size;
	start_cmd->psm = isp->shared->fw.psm_all;
	start_cmd->txMsgMode = AttachMsg_TimeOut;

	dev_dbg(isp->dev,
		"DMA: 0x%08X, ISPSTART: DDRBase: 0x%08X, size: 0x%08X, psm: 0x%08X\n",
		isp_msg_addr_dma(isp, mc), start_cmd->reconfDDRBase,
		start_cmd->reconfDDRSize, start_cmd->psm);

	return isp_msg_tx(isp, mc, false);
}

int isp_msg_rw_addr(struct isp_device *isp)
{
	struct media_command *mc;
	rw_addr_t *rw_cmd;

	if (isp->shared->fw.stage != FS_RUNNING)
		return -EPERM;

	mc = get_media_cmd(isp);
	memset(mc, 0, sizeof(*mc));
	mc->cmd_hdr.hdr_info.cmd_type_minor = MINOR_RW_ADDR;

	rw_cmd = (rw_addr_t *)(&mc->user_cmd_data[0]);
	rw_cmd->addr = isp->rw_addr_msg.addr;
	rw_cmd->value = isp->rw_addr_msg.value;
	rw_cmd->flag = isp->rw_addr_msg.flag;

	dev_dbg(isp->dev,
		"DMA: 0x%08X, RW_ADDR: addr: 0x%08X, value: 0x%08X, flag: 0x%X\n",
		isp_msg_addr_dma(isp, mc), rw_cmd->addr, rw_cmd->value,
		rw_cmd->flag);

	return isp_msg_tx(isp, mc, true);
}

int isp_msg_channel_set_cfg(struct isp_channel *channel)
{
	struct media_command *mc;
	isp_ld_reconf_t *config_cmd;
	ipc_reconf_t *cfg;
	struct camera_dev *cam_dev;
	struct isp_device *isp;
	int cid;

	isp = channel->isp;
	cam_dev = channel->cam_dev;
	cid = channel->cid;

	mc = get_media_cmd(isp);
	memset(mc, 0, sizeof(struct media_command));
	mc->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_BOOTLD_RECONF;

	cfg = isp->msg.conf_va + (isp->msg.conf_size * cid);
	memcpy(cfg, channel->cfg, sizeof(*channel->cfg));
	cfg->sensorOnline = cam_dev->power_on;
	config_cmd = (isp_ld_reconf_t *)(&mc->user_cmd_data[0]);
	config_cmd->mipiSensorIndex = cid;
	config_cmd->payloadAddr = isp_msg_addr_dma(isp, cfg);
	config_cmd->payloadSize = sizeof(*cfg);
	if (channel->algo) {
		cfg->algo_addr = channel->algo->dma;
		cfg->algo_size = channel->algo->size;
	}
	if (channel->iq) {
		cfg->iq_addr = channel->iq->dma;
		cfg->iq_size = channel->iq->size;
	}

	dev_dbg(isp->dev,
		"DMA: 0x%08X, C%02d SETCFG, IRB: 0x%08X, MSI: %u, SI: %u, SD: 0x%02X, SM: %u, SO: %u, LTMC: %u, %u, AA: 0x%08X, AS: %u, IA: 0x%08X, IS: %u, ISPIC: %u, %u, %u, %u\n",
		isp_msg_addr_dma(isp, mc), cid, cfg->i2cRegBase,
		cfg->mipiSensorIndex, cfg->sensorIndex, cfg->sensorDevID,
		cfg->sensorRdWrMode, cfg->sensorOnline, cfg->ltmVinTopCrop,
		cfg->ltmVinBotCrop, cfg->algo_addr, cfg->algo_size,
		cfg->iq_addr, cfg->iq_size, cfg->ispInTopCrop,
		cfg->ispInBotCrop, cfg->ispInLefCrop, cfg->ispInRigCrop);

	return isp_msg_tx(isp, mc, false);
}

int isp_msg_channel_tx_algo(struct isp_channel *channel,
			    const struct isp_file *file)
{
	struct media_command *mc;
	isp_ld_algo_bin_t *config_cmd;
	struct isp_device *isp;
	int cid;

	isp = channel->isp;
	cid = channel->cid;

	mc = get_media_cmd(isp);
	memset(mc, 0, sizeof(struct media_command));
	mc->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_BOOTLD_ALGO_BIN;

	config_cmd = (isp_ld_algo_bin_t *)(&mc->user_cmd_data[0]);
	config_cmd->mipiSensorIndex = cid;
	config_cmd->payloadAddr = file->dma;
	config_cmd->payloadSize = file->size;

	dev_dbg(isp->dev,
		"DMA: 0x%08X, C%02d TXALGO, payload: 0x%08X, size: %u\n",
		isp_msg_addr_dma(isp, mc), cid, config_cmd->payloadAddr,
		config_cmd->payloadSize);

	return isp_msg_tx(isp, mc, true);
}

int isp_msg_channel_tx_iq(struct isp_channel *channel,
			  const struct isp_file *file)
{
	struct media_command *mc;
	isp_ld_iq_bin_t *config_cmd;
	struct isp_device *isp;
	int cid;

	isp = channel->isp;
	cid = channel->cid;

	mc = get_media_cmd(isp);
	memset(mc, 0, sizeof(struct media_command));
	mc->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_BOOTLD_IQ_BIN;

	config_cmd = (isp_ld_iq_bin_t *)(&mc->user_cmd_data[0]);
	config_cmd->mipiSensorIndex = cid;
	config_cmd->payloadAddr = file->dma;
	config_cmd->payloadSize = file->size;

	dev_dbg(isp->dev,
		"DMA: 0x%08X, C%02d TXIQ, payload: 0x%08X, size: %u\n",
		isp_msg_addr_dma(isp, mc), channel->cid,
		config_cmd->payloadAddr, config_cmd->payloadSize);

	return isp_msg_tx(isp, mc, true);
}

static void fill_buf_mc(struct isp_video *video, u32 *dma_addrs,
			struct media_command *mc)
{
	int i;
	isp_new_frame_buf_t *buf;

	memset(mc, 0, sizeof(*mc));
	buf = (isp_new_frame_buf_t *)(&mc->user_cmd_data[0]);
	buf->mipiSensorIndex = video->cid;

	if (video->is_raw) {
		mc->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_NEW_RAW_BUF;
		buf->viewMode = ViewRaw;
		buf->viewbuf[0] = dma_addrs[0];
		buf->bufFlag = 0;
	} else {
		mc->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_NEW_VIEW_BUF;
		buf->viewMode = noView;
		for (i = 0; i < ARRAY_SIZE(video->views); ++i) {
			if (video->views[i].used) {
				buf->viewMode |= 1 << i;
				buf->viewbuf[i] = dma_addrs[i];
			}
		}
	}
}

int isp_msg_video_open(struct isp_video *video)
{
	int rv;
	unsigned int i;
	struct media_command *mc;
	isp_cam_open_t *cam_open_cmd;
	unsigned int meta_vid;
	struct device *dev;
	int num;
	struct isp_buffer *buf;
	struct isp_buffer *tmp;

	dev = video->isp->dev;

	/* Merge buffers message */
	mc = get_media_cmds(video->isp, video->vb2_queue->num_buffers + 1);
	mutex_lock(&video->free_buf_lock);
	mutex_lock(&video->wait_buf_lock);
	num = 0;
	list_for_each_entry_safe(buf, tmp, &video->free_buf_queue, node) {
		/* Add to wait queue first since ISP FW may use this
		 * before TX API returns
		 */
		list_del(&buf->node);
		list_add_tail(&buf->node, &video->wait_buf_queue);
		fill_buf_mc(video, buf->dma, mc);
		++video->stat.buf_num_fw;
		++num;
		++mc;
	}
	mutex_unlock(&video->wait_buf_lock);
	mutex_unlock(&video->free_buf_lock);

	/* CAM_OPEN message */
	memset(mc, 0, sizeof(*mc));
	mc->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_CAM_OPEN;

	cam_open_cmd = (isp_cam_open_t *)&(mc->user_cmd_data[0]);
	cam_open_cmd->mipiSensorIndex = video->cid;
	cam_open_cmd->viewMode = noView;

	meta_vid = -1;
	for (i = 0; i < ARRAY_SIZE(video->views); ++i) {
		if (!video->views[i].used)
			continue;
		cam_open_cmd->viewMode |= 1 << i;

		if (video->views[i].meta)
			meta_vid = min(meta_vid, i);
	}

	if (meta_vid < ARRAY_SIZE(video->views)) {
		cam_open_cmd->embedded_view = meta_vid;
		cam_open_cmd->isp_meta_offset =
			video->views[meta_vid].sizeimage +
			video->isp_meta.offset;
		dev_dbg(dev,
			"V%02d CAMOPEN, cid: %02u, meta view: %u, isp offset: %8u\n",
			video->vid, cam_open_cmd->mipiSensorIndex,
			cam_open_cmd->embedded_view,
			cam_open_cmd->isp_meta_offset);
		for (i = 0; i < ARRAY_SIZE(cam_open_cmd->embedded_offset);
		     ++i) {
			if (video->sensor_meta[i].offset)
				cam_open_cmd->embedded_offset[i] =
					video->views[meta_vid].sizeimage +
					video->sensor_meta[i].offset;
			else
				cam_open_cmd->embedded_offset[i] = 0;
			dev_dbg(dev,
				"V%02d CAMOPEN, cid: %02u, meta view: %u, offset %u: %8u\n",
				video->vid, cam_open_cmd->mipiSensorIndex,
				cam_open_cmd->embedded_view, i,
				cam_open_cmd->embedded_offset[i]);
		}
	}

	/* Rewind to start */
	mc -= num;
	mc->cmd_hdr.hdr_info.follow_pack_num = num;
	dev_dbg(dev,
		"DMA: 0x%08X, V%02d CAMOPEN, cid: %02u, viewMode: 0x%X, meta view: %u, follow: %u\n",
		isp_msg_addr_dma(video->isp, mc), video->vid,
		cam_open_cmd->mipiSensorIndex, cam_open_cmd->viewMode,
		cam_open_cmd->embedded_view, num);

	rv = isp_msg_tx(video->isp, mc, true);
	if (rv) {
		dev_err(dev, "V%02d CAMOPEN error, rv: %d\n", video->vid, rv);
		goto err;
	}
	if (video->stream_op) {
		dev_err(dev,
			"V%02d CAMOPEN: Failed, may owned by other client\n",
			video->vid);
		rv = -EBUSY;
		goto err;
	}

	return 0;

err:
	/* Release buffers added to wait queue */
	mutex_lock(&video->wait_buf_lock);
	mutex_lock(&video->free_buf_lock);
	list_for_each_entry_safe_reverse(buf, tmp, &video->wait_buf_queue,
					 node) {
		if (num == 0)
			break;
		/* Add to wait queue first since ISP FW may use this
		 * before TX API returns
		 */
		list_del(&buf->node);
		list_add_tail(&buf->node, &video->free_buf_queue);
		--num;
		--video->stat.buf_num_fw;
	}
	mutex_unlock(&video->free_buf_lock);
	mutex_unlock(&video->wait_buf_lock);

	return rv;
}

int isp_msg_video_close(struct isp_video *video)
{
	int rv;
	int i;
	struct media_command *mc;
	isp_cam_close_t *cam_close_cmd;
	struct device *dev;
	struct isp_buffer *buf;
	struct isp_buffer *tmp;

	dev = video->isp->dev;

	mc = get_media_cmd(video->isp);
	memset(mc, 0, sizeof(*mc));
	mc->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_CAM_CLOSE;

	cam_close_cmd = (isp_cam_close_t *)&(mc->user_cmd_data[0]);
	cam_close_cmd->mipiSensorIndex = video->cid;
	cam_close_cmd->viewMode = noView;
	for (i = 0; i < ARRAY_SIZE(video->views); ++i)
		if (video->views[i].used)
			cam_close_cmd->viewMode |= (1 << i);

	dev_dbg(dev, "DMA: 0x%08X, V%02d CAMCLOSE, cid: %02u, viweMode: 0x%X\n",
		isp_msg_addr_dma(video->isp, mc), video->vid,
		cam_close_cmd->mipiSensorIndex, cam_close_cmd->viewMode);
	rv = isp_msg_tx(video->isp, mc, true);
	if (rv) {
		dev_err(dev, "V%02d CAMCLOSE error, rv: %d\n", video->vid, rv);
		return rv;
	}

	if (video->stream_op) {
		dev_err(dev,
			"V%02d CAMCLOSE: Failed, may owned by other client\n",
			video->vid);
		return -EBUSY;
	}

	/* Release buffers added to wait queue */
	mutex_lock(&video->wait_buf_lock);
	mutex_lock(&video->free_buf_lock);
	list_for_each_entry_safe_reverse(buf, tmp, &video->wait_buf_queue,
					 node) {
		/* Add to wait queue first since ISP FW may use this
		 * before TX API returns
		 */
		list_del(&buf->node);
		list_add_tail(&buf->node, &video->free_buf_queue);
	}
	mutex_unlock(&video->free_buf_lock);
	mutex_unlock(&video->wait_buf_lock);

	return 0;
}

int isp_msg_video_open_raw(struct isp_video *video)
{
	int rv;
	struct media_command *mc;
	isp_cam_open_t *raw_open_cmd;
	struct device *dev;
	int num;
	struct isp_buffer *buf;
	struct isp_buffer *tmp;

	dev = video->isp->dev;

	/* Merge buffers message */
	mc = get_media_cmds(video->isp, video->vb2_queue->num_buffers + 1);
	mutex_lock(&video->free_buf_lock);
	mutex_lock(&video->wait_buf_lock);
	num = 0;
	list_for_each_entry_safe(buf, tmp, &video->free_buf_queue, node) {
		/* Add to wait queue first since ISP FW may use this
		 * before TX API returns
		 */
		list_del(&buf->node);
		list_add_tail(&buf->node, &video->wait_buf_queue);
		fill_buf_mc(video, buf->dma, mc);
		++num;
		++mc;
	}
	mutex_unlock(&video->wait_buf_lock);
	mutex_unlock(&video->free_buf_lock);

	/* RAW_OPEN message */
	memset(mc, 0, sizeof(*mc));
	mc->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_RAW_OPEN;

	raw_open_cmd = (isp_cam_open_t *)&(mc->user_cmd_data[0]);
	raw_open_cmd->mipiSensorIndex = video->cid;
	raw_open_cmd->viewMode = ViewRaw;

	/* Rewind to start */
	mc -= num;
	mc->cmd_hdr.hdr_info.follow_pack_num = num;
	dev_dbg(dev, "DMA: 0x%08X, V%02d RAWOPEN, cid: %02u, viweMode: 0x%X\n",
		isp_msg_addr_dma(video->isp, mc), video->vid,
		raw_open_cmd->mipiSensorIndex, raw_open_cmd->viewMode);

	rv = isp_msg_tx(video->isp, mc, true);
	if (rv) {
		dev_err(dev, "V%02d RAWOPEN error, rv: %d\n", video->vid, rv);
		goto err;
	}
	if (video->stream_op) {
		dev_err(dev,
			"V%02d RAWOPEN: Failed, may owned by other client\n",
			video->vid);
		rv = -EBUSY;
		goto err;
	}

	return 0;

err:
	/* Release buffers added to wait queue */
	mutex_lock(&video->wait_buf_lock);
	mutex_lock(&video->free_buf_lock);
	list_for_each_entry_safe_reverse(buf, tmp, &video->wait_buf_queue,
					 node) {
		if (num == 0)
			break;
		/* Add to wait queue first since ISP FW may use this
		 * before TX API returns
		 */
		list_del(&buf->node);
		list_add_tail(&buf->node, &video->free_buf_queue);
		--num;
	}
	mutex_unlock(&video->free_buf_lock);
	mutex_unlock(&video->wait_buf_lock);

	return rv;
}

int isp_msg_video_close_raw(struct isp_video *video)
{
	int rv;
	struct media_command *mc;
	isp_cam_close_t *raw_close_cmd;
	struct device *dev;
	struct isp_buffer *buf;
	struct isp_buffer *tmp;

	dev = video->isp->dev;
	mc = get_media_cmd(video->isp);
	memset(mc, 0, sizeof(*mc));
	mc->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_RAW_CLOSE;

	raw_close_cmd = (isp_cam_close_t *)&(mc->user_cmd_data[0]);
	raw_close_cmd->mipiSensorIndex = video->cid;

	dev_dbg(dev, "DMA: 0x%08X, V%02d RAWCLOSE, cid: %02u\n",
		isp_msg_addr_dma(video->isp, mc), video->vid,
		raw_close_cmd->mipiSensorIndex);

	rv = isp_msg_tx(video->isp, mc, true);
	if (rv) {
		dev_err(dev, "V%02d RAWCLOSE error, rv: %d\n", video->vid, rv);
		return rv;
	}

	if (video->stream_op) {
		dev_err(dev,
			"V%02d RAWCLOSE: Failed, may owned by other client\n",
			video->vid);
		return -EBUSY;
	}

	/* Release buffers added to wait queue */
	mutex_lock(&video->wait_buf_lock);
	mutex_lock(&video->free_buf_lock);
	list_for_each_entry_safe_reverse(buf, tmp, &video->wait_buf_queue,
					 node) {
		/* Add to wait queue first since ISP FW may use this
		 * before TX API returns
		 */
		list_del(&buf->node);
		list_add_tail(&buf->node, &video->free_buf_queue);
	}
	mutex_unlock(&video->free_buf_lock);
	mutex_unlock(&video->wait_buf_lock);

	return 0;
}

int isp_msg_video_plugout(struct isp_video *video)
{
	struct media_command *mc;
	isp_cam_pluginout_t *plugout_cmd;

	if (video->isp->shared->fw.stage != FS_RUNNING)
		return -EPERM;

	mc = get_media_cmd(video->isp);
	memset(mc, 0, sizeof(*mc));
	mc->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_CAM_PLUGOUT;

	plugout_cmd = (isp_cam_pluginout_t *)&(mc->user_cmd_data[0]);
	plugout_cmd->mipiSensorIndex = video->cid;

	dev_dbg(video->isp->dev, "DMA: 0x%08X, V%02d PLUGOUT\n",
		isp_msg_addr_dma(video->isp, mc), video->vid);

	return isp_msg_tx(video->isp, mc, false);
}

int isp_msg_video_plugin(struct isp_video *video)
{
	struct media_command *mc;
	isp_cam_pluginout_t *plugin_cmd;

	if (video->isp->shared->fw.stage != FS_RUNNING)
		return -EPERM;

	mc = get_media_cmd(video->isp);
	memset(mc, 0, sizeof(*mc));
	mc->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_CAM_PLUGIN;

	plugin_cmd = (isp_cam_pluginout_t *)&(mc->user_cmd_data[0]);
	plugin_cmd->mipiSensorIndex = video->cid;

	dev_dbg(video->isp->dev, "DMA: 0x%08X, V%02d PLUGIN\n",
		isp_msg_addr_dma(video->isp, mc), video->vid);

	return isp_msg_tx(video->isp, mc, false);
}

int isp_msg_video_txbuf(struct isp_video *video, u32 *dma_addrs)
{
	struct media_command *mc;
	int rv;
	struct device *dev;

	if (video->isp->shared->fw.stage != FS_RUNNING)
		return -EPERM;

	dev = video->isp->dev;
	mc = get_media_cmd(video->isp);
	fill_buf_mc(video, dma_addrs, mc);
	rv = isp_msg_tx(video->isp, mc, false);
	if (!rv)
		dev_dbg(dev, "V%02d TXBUF: 0x%08X, 0x%08X, 0x%08X\n",
			video->vid, dma_addrs[0], dma_addrs[1], dma_addrs[2]);
	else if (rv != -EPERM)
		dev_err_ratelimited(
			dev,
			"V%02d TXBUF: 0x%08X, 0x%08X, 0x%08X, rv: %d, FAILED\n",
			video->vid, dma_addrs[0], dma_addrs[1], dma_addrs[2],
			rv);

	return rv;
}
