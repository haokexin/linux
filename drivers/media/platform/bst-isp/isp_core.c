// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/dma-buf.h>
#include <linux/dma-direct.h>
#include <linux/dma-map-ops.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_graph.h>
#include <linux/property.h>
#include <linux/reset.h>
#include <linux/sched.h>
#include <linux/string.h>

#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>

#ifdef CONFIG_BST_IPC
#include <bst/ipc_interface.h>
#endif

#include "isp_core.h"

#include "isp_hw.h"
#include "isp_msg.h"
#include "isp_misc.h"
#include "isp_proto_ipc.h"
#include "isp_sysfs.h"
#include "isp_video.h"

static void handle_camera_event(struct isp_device *isp,
				unsigned int notification,
				struct camera_dev *cam_dev);

/*
 * Get next isp channel to be configured.
 *
 * @isp: ISP device.
 *
 * Returns isp channel pointer or NULL when there is no more channel.
 */
static struct isp_channel *isp_cfg_next(struct isp_device *isp)
{
	u32 index;

	mutex_lock(&isp->fw.cfg_lock);
	index = isp->fw.cfg_index;
	mutex_unlock(&isp->fw.cfg_lock);

	if (index >= ARRAY_SIZE(isp->channels) || !isp->channels[index].enabled)
		return NULL;
	else
		return &isp->channels[index];
}

/*
 * Move the pos of isp channel to be configured to next.
 *
 * @isp: ISP device.
 */
static inline void isp_cfg_iter(struct isp_device *isp)
{
	u32 i;

	mutex_lock(&isp->fw.cfg_lock);
	for (i = isp->fw.cfg_index + 1; i < ARRAY_SIZE(isp->channels); ++i)
		if (isp->channels[i].enabled)
			break;
	isp->fw.cfg_index = i;
	mutex_unlock(&isp->fw.cfg_lock);
}

/*
 * Reset the pos of isp channel to be configured to first.
 *
 * @isp: ISP device.
 */
static inline void isp_cfg_reset(struct isp_device *isp)
{
	u32 i;

	mutex_lock(&isp->fw.cfg_lock);
	for (i = 0; i < ARRAY_SIZE(isp->channels); ++i)
		if (isp->channels[i].enabled)
			break;
	isp->fw.cfg_index = i;
	mutex_unlock(&isp->fw.cfg_lock);
}

/*
 * Load ISP file to memory and setup management information.
 * @isp: ISP device.
 * @path: The persistent path related to system's firmware directory.
 */
static int isp_file_load(struct isp_device *isp, const char *path)
{
	int rv;
	struct device *dev;
	const struct firmware *firmware;
	int pos;
	dma_addr_t start;

	if (path == NULL)
		return -EINVAL;

	rv = -EIO;
	dev = isp->dev;
	dev_dbg(dev, "Will load '%s'\n", path);
	mutex_lock(&isp->fw.cfg_lock);
	pos = isp->shared->file.num;
	if (pos >= ARRAY_SIZE(isp->shared->file.files)) {
		dev_err(dev, "Full, '%s' is not loaded, pos: %u, cap: %zu\n",
			path, pos, ARRAY_SIZE(isp->shared->file.files));
		goto err_pos;
	}

	rv = request_firmware(&firmware, path, dev);
	if (rv) {
		dev_err(dev, "Failed to request '%s', rv: %d\n", path, rv);
		goto err_req;
	}

	start = isp->shared->file.next_dma;
	if (start + firmware->size > isp->msg.init_dma + isp->msg.init_size) {
		dev_err(dev, "File '%s' size %zu exceed remaining size %llu\n",
			path, firmware->size,
			isp->msg.init_dma + isp->msg.init_size - start);
		goto err_size;
	}

	dev_dbg(dev, "Loaded '%s' to dma: 0x%08llX, pa: 0x%llX, va: 0x%llX\n",
		path, start, dma_to_phys(isp->dev, start),
		(u64)isp_msg_addr_cpu(isp, start));
	memcpy(isp_msg_addr_cpu(isp, start), firmware->data, firmware->size);
	strscpy(isp->shared->file.files[pos].path, path,
		sizeof(isp->shared->file.files[pos].path));
	isp->shared->file.files[pos].dma = start;
	isp->shared->file.files[pos].size = firmware->size;

	/* Update metadata */
	start += firmware->size;
	start = PTR_ALIGN(start, ISP_MSG_PAYLOAD_ALIGN);
	isp->shared->file.next_dma = start;
	++isp->shared->file.num;
	rv = 0;

err_size:
	release_firmware(firmware);
err_req:
err_pos:
	mutex_unlock(&isp->fw.cfg_lock);

	return rv;
}

/*
 * Get ISP file information from loaded files
 * @isp: ISP device
 * @path: The persistent path related to system's firmware directory
 */
static const struct isp_file *isp_file_get(struct isp_device *isp,
					   const char *path)
{
	int i;
	struct isp_file *file;

	if (path == NULL)
		return NULL;

	file = NULL;
	mutex_lock(&isp->fw.cfg_lock);
	for (i = 0; i < isp->shared->file.num; ++i) {
		if (strcmp(path, isp->shared->file.files[i].path) == 0) {
			file = &isp->shared->file.files[i];
			break;
		}
	}
	mutex_unlock(&isp->fw.cfg_lock);

	return file;
}

static void isp_file_load_all(struct isp_device *isp)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(isp->channels); ++i) {
		const struct isp_file *file;
		struct isp_channel *channel;

		channel = &isp->channels[i];
		if (!channel->enabled)
			continue;

		file = isp_file_get(isp, channel->cam_dev->algo);
		if (!file) {
			isp_file_load(isp, channel->cam_dev->algo);
			file = isp_file_get(isp, channel->cam_dev->algo);
		}
		channel->algo = file;

		file = isp_file_get(isp, channel->cam_dev->iq);
		if (!file) {
			isp_file_load(isp, channel->cam_dev->iq);
			file = isp_file_get(isp, channel->cam_dev->iq);
		}
		channel->iq = file;
	}
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

/*
 * Send ISP algo file to firmware
 * @isp: ISP device
 * @channel: The ISP channel to send algo file
 */
static int send_algo_file(struct isp_device *isp, struct isp_channel *channel)
{
	const struct isp_file *file;

	file = isp_file_get(isp, channel->cam_dev->algo);
	if (!file) {
		isp_file_load(isp, channel->cam_dev->algo);
		file = isp_file_get(isp, channel->cam_dev->algo);
	}

	if (!file) {
		dev_err(isp->dev, "No valid algo file '%s' for channel %d\n",
			channel->cam_dev->algo, channel->cid);
		return -EIO;
	}

	return isp_msg_channel_tx_algo(channel, file);
}

/*
 * Send ISP IQ file to firmware
 * @isp: ISP device
 * @channel: The ISP channel to send algo file
 */
static int send_iq_file(struct isp_device *isp, struct isp_channel *channel)
{
	const struct isp_file *file;

	file = isp_file_get(isp, channel->cam_dev->iq);
	if (!file) {
		isp_file_load(isp, channel->cam_dev->iq);
		file = isp_file_get(isp, channel->cam_dev->iq);
	}

	if (!file) {
		dev_err(isp->dev, "No valid iq file '%s' for channel %d\n",
			channel->cam_dev->algo, channel->cid);
		return -EIO;
	}

	return isp_msg_channel_tx_iq(channel, file);
}

#pragma GCC diagnostic pop

static void parse_channel_cfg(struct isp_device *isp, struct media_command *mc)
{
	struct isp_channel *channel;
	ipc_reconf_t *cfg;
	isp_ld_reconf_t *config_cmd;
	u8 cid;

	config_cmd = (isp_ld_reconf_t *)(&mc->user_cmd_data[0]);
	cid = config_cmd->mipiSensorIndex;
	if (cid >= ARRAY_SIZE(isp->channels)) {
		dev_err(isp->dev, "PARSECFG: invalid channel %02u\n", cid);
		return;
	}

	cfg = (ipc_reconf_t *)isp_msg_addr_cpu(isp, config_cmd->payloadAddr);
	/* Update channel's cfg */
	channel = &isp->channels[cid];
	memcpy(channel->cfg, cfg, sizeof(*channel->cfg));
	dev_dbg(isp->dev,
		"PARSECFG: cid: %02u, DMA: 0x%08X, raw: %ux%u, v0: %u-%ux%u-%u, v1: %u-%ux%u-%u, v2: %u-%ux%u-%u, e0: %u-%u, e1: %u-%u, sem: %u-%u\n",
		cid, config_cmd->payloadAddr, cfg->rawinfo.width,
		cfg->rawinfo.height, cfg->viewinfo[0].viewFmt,
		cfg->viewinfo[0].width, cfg->viewinfo[0].height,
		cfg->viewinfo[0].lineAlign, cfg->viewinfo[1].viewFmt,
		cfg->viewinfo[1].width, cfg->viewinfo[1].height,
		cfg->viewinfo[1].lineAlign, cfg->viewinfo[2].viewFmt,
		cfg->viewinfo[2].width, cfg->viewinfo[2].height,
		cfg->viewinfo[2].lineAlign, cfg->embeddedInfo[0].line_start,
		cfg->embeddedInfo[0].line_end, cfg->embeddedInfo[1].line_start,
		cfg->embeddedInfo[1].line_end, cfg->semBank, cfg->semId);

	isp_video_update_cfg(channel);
}

void isp_update_channels_cfg(struct isp_device *isp)
{
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	int i;
	struct isp_channel *channel;

	mutex_lock(&isp->fw.cfg_lock);
	if (isp->cfg_updated)
		goto exit;

	dev_info(isp->dev, "Update channels' cfg from shared\n");
	for (i = 0; i < ARRAY_SIZE(isp->channels); ++i) {
		channel = &isp->channels[i];
		/* NOTE: Consider as enabled when the mipiSensorIndex is matched */
		if (channel->enabled && channel->cfg->mipiSensorIndex == i)
			isp_video_update_cfg(&isp->channels[i]);
	}
	isp->cfg_updated = true;
exit:
	mutex_unlock(&isp->fw.cfg_lock);
#endif
}

const char *isp_str_fw_stage(enum fw_stage stage)
{
	switch (stage) {
	case FS_UNUSED:
		return "unused";
	case FS_LOADED:
		return "loaded";
	case FS_BOOTED:
		return "booted";
	case FS_RUNNING:
		return "running";
	default:
		return "unknown";
	}
}

const char *isp_str_role(u32 role)
{
	switch (role) {
	case ROLE_MASTER:
		return "master";
	case ROLE_SLAVE:
		return "slave";
	case ROLE_AUTO:
		return "auto";
	default:
		return "unknown";
	}
}

/*
 * Judge whether media_cmd addr is valid
 * @isp: ISP device
 * @mc: The media_command's CPU address converted from DMA
 *
 * Returns true if the address is legal
 */
static bool is_valid_cmd(struct isp_device *isp, void *mc)
{
	tSoneCmdp *cmdp;
	void *fmc_start;
	size_t fmc_size;

	cmdp = (tSoneCmdp *)isp->msg.cmdp_va;
	fmc_start = (void *)cmdp->ch[FW_CH_INDEX].cqueue.c0;
	fmc_size = sizeof(cmdp->ch[FW_CH_INDEX].cqueue.c0);
	if (mc < fmc_start || mc >= (fmc_start + fmc_size))
		return false;

	return true;
}

static void handle_boot_done(struct isp_device *isp, struct media_command *mc,
			     u64 ts)
{
	int rv;
	struct isp_channel *channel;
	struct device *dev;

	dev = isp->dev;
	dev_info(dev, "role: %s: Handle boot done, ts: %llu, stage: %s\n",
		 isp_str_role(isp->role), ts,
		 isp_str_fw_stage(isp->shared->fw.stage));

	if (isp->role != ROLE_MASTER)
		return;
	if (isp->shared->fw.stage >= FS_BOOTED)
		return;

#ifdef CONFIG_BST_IPC_MSGBX
	isp->msgbx.subscribed = true;
#endif
	isp->shared->fw.stage = FS_BOOTED;
	/* Load all files, then start config */
	isp_file_load_all(isp);

	isp_cfg_reset(isp);
	channel = isp_cfg_next(isp);
	if (!channel) {
		dev_info(dev, "No channel to config, start ISP\n");
		rv = isp_msg_dev_start(isp);
		if (rv)
			dev_err(dev, "Failed to start ISP, rv: %d\n", rv);
		return;
	}

	rv = isp_msg_channel_set_cfg(channel);
	if (rv)
		dev_err(dev, "Failed to config channel %d, rv: %d\n",
			channel->cid, rv);
}

static void handle_abnormal(struct isp_device *isp, struct media_command *mc,
			    u64 ts)
{
	int cid;

	cid = mc->cmd_hdr.magic.uid;
	if (cid < 0 || cid >= ARRAY_SIZE(isp->channels)) {
		dev_err_ratelimited(isp->dev,
				    "Video buffer for invalid channel: %d\n",
				    cid);
		return;
	}

	if (isp->channels[cid].views_video.status == VS_STREAM_ON)
		isp_video_dispatch_msg(&(isp->channels[cid].views_video), mc,
				       ts);
}

static void handle_rw_addr(struct isp_device *isp, struct media_command *mc,
			   u64 ts)
{
	rw_addr_t *rw_cmd;

	dev_dbg(isp->dev, "Handle rw addr, ts: %llu\n", ts);

	rw_cmd = (rw_addr_t *)(&mc->user_cmd_data[0]);
	if (!rw_cmd->flag)
		isp->rw_addr_msg.value = rw_cmd->value;

	complete(&isp->msg.tx_comp);
	dev_dbg(isp->dev, "RW_ADDR: addr: 0x%08X, value: 0x%08X, flag: 0x%X\n",
		rw_cmd->addr, rw_cmd->value, rw_cmd->flag);
}

static void handle_bootld_reconf(struct isp_device *isp,
				 struct media_command *mc, u64 ts)
{
	int rv;
	struct isp_channel *channel;
	struct device *dev;

	dev = isp->dev;
	dev_dbg(dev, "role: %s: Handle bootld reconf, ts: %llu\n",
		isp_str_role(isp->role), ts);
	if (isp->role != ROLE_MASTER)
		return;

	parse_channel_cfg(isp, mc);

	isp_cfg_iter(isp);
	channel = isp_cfg_next(isp);
	if (!channel) {
		dev_info(dev, "No more channel to config, start ISP\n");
		rv = isp_msg_dev_start(isp);
		if (rv)
			dev_err(dev, "Failed to start ISP, rv: %d\n", rv);
		return;
	}

	rv = isp_msg_channel_set_cfg(channel);
	if (rv)
		dev_err(dev, "Failed to config channel %d, rv: %d\n",
			channel->cid, rv);
}

static void handle_bootld_algo(struct isp_device *isp, struct media_command *mc,
			       u64 ts)
{
	dev_dbg(isp->dev, "Handle bootld algo, ts: %llu\n", ts);
	complete(&isp->msg.tx_comp);
}

static void handle_bootld_iq(struct isp_device *isp, struct media_command *mc,
			     u64 ts)
{
	dev_dbg(isp->dev, "Handle bootld iq, ts: %llu\n", ts);
	complete(&isp->msg.tx_comp);
}

static void handle_dev_start(struct isp_device *isp, struct media_command *mc,
			     u64 ts)
{
	int i;
	struct device *dev;

	dev = isp->dev;
	dev_info(dev, "role: %s: Handle start, ts: %llu\n",
		 isp_str_role(isp->role), ts);

	if (isp->role == ROLE_MASTER && isp->shared->fw.stage != FS_RUNNING) {
		complete_all(&isp->fw.boot_comp);

		for (i = 0; i < ARRAY_SIZE(isp->csi_asd); ++i)
			if (isp->csi_asd[i].csi_dev)
				v4l2_subdev_call(
					&isp->csi_asd[i].csi_dev->subdev, video,
					pre_streamon, 0);
	}

	/* NOTE: This is used for resume from STR */
	for (i = 0; i < ARRAY_SIZE(isp->channels); ++i) {
		struct isp_channel *channel;
		struct isp_video *video;

		channel = &isp->channels[i];
		if (!channel->enabled)
			continue;

		channel->cam_dev->power_on = channel->cfg->sensorOnline;
		video = &channel->views_video;
		if (video->status == VS_STREAM_ON && !video->hw_on) {
			isp_video_recycle_wait_bufs(video);
			if (channel->cfg->sensorOnline)
				isp_video_streamon_hw(video);
		}

		video = &channel->raw_video;
		if (video->status == VS_STREAM_ON && !video->hw_on) {
			isp_video_recycle_wait_bufs(video);
			if (channel->cfg->sensorOnline)
				isp_video_streamon_hw(video);
		}
	}
}

static void handle_video_open(struct isp_device *isp, struct media_command *mc,
			      u64 ts)
{
	int cid;
	struct isp_video *video;

	cid = mc->cmd_hdr.magic.uid;
	if (cid < 0 || cid >= ARRAY_SIZE(isp->channels)) {
		dev_err_ratelimited(isp->dev, "Open invalid channel: %d\n",
				    cid);
		return;
	}

	dev_dbg(isp->dev, "V%02d opened\n", cid);
	video = &isp->channels[cid].views_video;
	video->stream_op = mc->cmd_hdr.magic.cmd_status;
	complete(&isp->msg.tx_comp);
}

static void handle_video_close(struct isp_device *isp, struct media_command *mc,
			       u64 ts)
{
	int cid;
	struct isp_video *video;

	cid = mc->cmd_hdr.magic.uid;
	if (cid < 0 || cid >= ARRAY_SIZE(isp->channels)) {
		dev_err_ratelimited(isp->dev, "Close invalid channel: %d\n",
				    cid);
		return;
	}

	dev_dbg(isp->dev, "V%02d closed\n", cid);
	video = &isp->channels[cid].views_video;
	video->stream_op = mc->cmd_hdr.magic.cmd_status;
	complete(&isp->msg.tx_comp);
}

static void handle_video_plugout(struct isp_device *isp,
				 struct media_command *mc, u64 ts)
{
	int cid;
	struct isp_video *video;
	struct camera_dev *cam_dev;

	cid = mc->cmd_hdr.magic.uid;
	if (cid < 0 || cid >= ARRAY_SIZE(isp->channels)) {
		dev_err_ratelimited(isp->dev, "Plugout invalid channel: %d\n",
				    cid);
		return;
	}

	video = &isp->channels[cid].views_video;
	cam_dev = isp->channels[cid].cam_dev;
	dev_dbg(isp->dev, "V%02d plugout recv, status: %u, power: %u\n", cid,
		video->status, cam_dev->power_on);
	if (video->status == VS_STREAM_ON && cam_dev->power_on) {
		dev_dbg(isp->dev, "V%02d plugout sent\n", cid);
		handle_camera_event(isp, ISP_EVENT_CAMERA_DISCONNECT, cam_dev);
	}
}

static void handle_video_plugin(struct isp_device *isp,
				struct media_command *mc, u64 ts)
{
	int cid;
	struct isp_video *video;
	struct camera_dev *cam_dev;

	cid = mc->cmd_hdr.magic.uid;
	if (cid < 0 || cid >= ARRAY_SIZE(isp->channels)) {
		dev_err_ratelimited(isp->dev, "plugin invalid channel: %d\n",
				    cid);
		return;
	}

	video = &isp->channels[cid].views_video;
	cam_dev = isp->channels[cid].cam_dev;
	dev_dbg(isp->dev, "V%02d plugin recv, status: %u, power: %u\n", cid,
		video->status, cam_dev->power_on);
	if (video->status == VS_STREAM_ON && !cam_dev->power_on) {
		dev_dbg(isp->dev, "V%02d plugin sent\n", cid);
		handle_camera_event(isp, ISP_EVENT_CAMERA_CONNECT, cam_dev);
	}
}

static void handle_video_open_raw(struct isp_device *isp,
				  struct media_command *mc, u64 ts)
{
	int cid;
	struct device *dev;
	struct isp_video *video;

	dev = isp->dev;
	cid = mc->cmd_hdr.magic.uid;
	if (cid < 0 || cid >= ARRAY_SIZE(isp->channels)) {
		dev_err_ratelimited(dev, "Open invalid channel: %d\n", cid);
		return;
	}

	if (isp->channels[cid].raw_video.status != VS_UNUSED) {
		dev_dbg(dev, "R%02d opened\n", cid);
		video = &isp->channels[cid].raw_video;
	} else if (isp->channels[cid].views_video.status != VS_UNUSED &&
		   isp->channels[cid].views_video.mirror_raw) {
		dev_dbg(dev, "P%02d opened\n", cid);
		video = &isp->channels[cid].views_video;
	} else {
		dev_err(dev, "C%02d improper video open message\n", cid);
		return;
	}

	video->stream_op = mc->cmd_hdr.magic.cmd_status;
	complete(&isp->msg.tx_comp);
}

static inline void handle_video_close_raw(struct isp_device *isp,
					  struct media_command *mc, u64 ts)
{
	int cid;
	struct device *dev;
	struct isp_video *video;

	dev = isp->dev;
	cid = mc->cmd_hdr.magic.uid;
	if (cid < 0 || cid >= ARRAY_SIZE(isp->channels)) {
		dev_err_ratelimited(dev, "Close invalid channel: %d\n", cid);
		return;
	}

	if (isp->channels[cid].raw_video.status != VS_UNUSED) {
		dev_dbg(dev, "R%02d closed\n", cid);
		video = &isp->channels[cid].raw_video;
	} else if (isp->channels[cid].views_video.status != VS_UNUSED &&
		   isp->channels[cid].views_video.mirror_raw) {
		dev_dbg(dev, "P%02d closed\n", cid);
		video = &isp->channels[cid].views_video;
	} else {
		dev_err(dev, "C%02d improper video close message\n", cid);
		return;
	}

	video->stream_op = mc->cmd_hdr.magic.cmd_status;
	complete(&isp->msg.tx_comp);
}

static void handle_video_buf(struct isp_device *isp, struct media_command *mc,
			     u64 ts)
{
	int cid;
	u16 cmd_minor;

	cid = mc->cmd_hdr.magic.uid;
	if (cid < 0 || cid >= ARRAY_SIZE(isp->channels)) {
		dev_err_ratelimited(isp->dev,
				    "Video buffer for invalid channel: %d\n",
				    cid);
		return;
	}

	cmd_minor = mc->cmd_hdr.hdr_info.cmd_type_minor;
	if (cmd_minor == MINOR_ISP_RAW_FRAME_DONE &&
	    isp->channels[cid].raw_video.status == VS_STREAM_ON)
		isp_video_dispatch_msg(&(isp->channels[cid].raw_video), mc, ts);
	else if (isp->channels[cid].views_video.status == VS_STREAM_ON)
		isp_video_dispatch_msg(&(isp->channels[cid].views_video), mc,
				       ts);
}

/*
 * Handle one message from ISP firmware
 * @isp: ISP device
 * @msg_dma: The address from Firmware view
 * @ts: The timestamp of receiving the message by monotonic ns
 */
static void isp_msg_handle_one(struct isp_device *isp, struct media_command *mc,
			       u64 ts)
{
	u32 cmd_minor;
	struct isp_msg msg;

	cmd_minor = mc->cmd_hdr.hdr_info.cmd_type_minor;
	dev_dbg(isp->dev,
		"MSGRX: sn: %8lu/%8lu, cid: %02u, minor: 0x%02X, status: %u, data: 0x%08X, 0x%08X, 0x%08X, 0x%08X, stage: %d\n",
		isp->msg.rx_all, isp->msg.rx_good, mc->cmd_hdr.magic.uid,
		cmd_minor, mc->cmd_hdr.magic.cmd_status, mc->user_cmd_data[0],
		mc->user_cmd_data[1], mc->user_cmd_data[2],
		mc->user_cmd_data[3], isp->shared->fw.stage);

	/* Drop message when system is not running */
	if (system_state > SYSTEM_RUNNING)
		return;

	switch (cmd_minor) {
	case MINOR_RW_ADDR:
		handle_rw_addr(isp, mc, ts);
		break;
	case MINOR_ISP_CAM_OPEN:
		handle_video_open(isp, mc, ts);
		break;
	case MINOR_ISP_CAM_CLOSE:
		handle_video_close(isp, mc, ts);
		break;
	case MINOR_ISP_RAW_OPEN:
		handle_video_open_raw(isp, mc, ts);
		break;
	case MINOR_ISP_RAW_CLOSE:
		handle_video_close_raw(isp, mc, ts);
		break;
	default:
		msg.mc = mc;
		msg.ts = ts;
		if (kfifo_in(&isp->async_msg_fifo, &msg, 1) != 1)
			dev_err_ratelimited(isp->dev,
					    "ASYNC msg fifo is full\n");
		complete(&isp->async_msg_comp);
	}
}

static int isp_msg_handler_async(void *data)
{
	struct isp_device *isp = (struct isp_device *)data;
	struct device *dev = isp->dev;

	dev_info(dev, "ASYNC: init\n");
	while (!kthread_should_stop()) {
		wait_for_completion_interruptible(&isp->async_msg_comp);
		reinit_completion(&isp->async_msg_comp);

		while (!kfifo_is_empty(&isp->async_msg_fifo)) {
			struct isp_msg msg;
			struct media_command *mc;
			u64 ts;
			u32 cmd_minor;

			if (kfifo_out(&isp->async_msg_fifo, &msg, 1) != 1)
				continue;

			mc = msg.mc;
			ts = msg.ts;
			cmd_minor = msg.mc->cmd_hdr.hdr_info.cmd_type_minor;
			switch (cmd_minor) {
			case MINOR_BOOT_DONE:
				handle_boot_done(isp, mc, ts);
				break;
			case MINOR_ABNORMAL:
				handle_abnormal(isp, mc, ts);
				break;
			case MINOR_ISP_BOOTLD_RECONF:
				handle_bootld_reconf(isp, mc, ts);
				break;
			case MINOR_ISP_BOOTLD_ALGO_BIN:
				handle_bootld_algo(isp, mc, ts);
				break;
			case MINOR_ISP_BOOTLD_IQ_BIN:
				handle_bootld_iq(isp, mc, ts);
				break;
			case MINOR_ISP_START:
				handle_dev_start(isp, mc, ts);
				break;
			case MINOR_ISP_CAM_PLUGOUT:
				handle_video_plugout(isp, mc, ts);
				break;
			case MINOR_ISP_CAM_PLUGIN:
				handle_video_plugin(isp, mc, ts);
				break;
			case MINOR_ISP_VIEW_FRAME_DONE:
				handle_video_buf(isp, mc, ts);
				break;
			case MINOR_ISP_RAW_FRAME_DONE:
				handle_video_buf(isp, mc, ts);
				break;
			default:
				dev_err_ratelimited(
					isp->dev,
					"MSGRX: sn: %8lu/%8lu, cid: %02u, minor: 0x%02X, stage: %d, unexpected message\n",
					isp->msg.rx_all, isp->msg.rx_good,
					mc->cmd_hdr.magic.uid, cmd_minor,
					isp->shared->fw.stage);
				break;
			}
		}
	}
	dev_info(dev, "ASYNC: exit\n");

	return 0;
}

/*
 * The main handler for message from ISP firmware
 * @isp: ISP device
 * @msg_dma: The address from Firmware view
 * @ts: The timestamp of receiving the message by monotonic ns
 */
static void isp_msg_handler_main(struct isp_device *isp, u32 msg_dma, u64 ts)
{
	struct media_command *mc;
	u32 cmd_minor;
	u32 follow_pack_num;
	u32 i;
	struct device *dev;

	dev = isp->dev;

	++isp->msg.rx_all;
	mc = (struct media_command *)isp_msg_addr_cpu(isp, msg_dma);
	if (!is_valid_cmd(isp, mc)) {
		++isp->msg.rx_bad;
		dev_err_ratelimited(
			dev,
			"MSGRX: sn: %8lu/%8lu, DMA: 0x%08X, CPU: 0x%016llX, stage: %d, illegal message\n",
			isp->msg.rx_all, isp->msg.rx_bad, msg_dma,
			(long long)mc, isp->shared->fw.stage);
		return;
	}

	++isp->msg.rx_good;
	cmd_minor = mc->cmd_hdr.hdr_info.cmd_type_minor;
	follow_pack_num = mc->cmd_hdr.hdr_info.follow_pack_num;
	dev_dbg(dev,
		"MSGRX: sn: %8lu/%8lu, DMA: 0x%08X, minor: 0x%02X, pack num: %u, stage: %d\n",
		isp->msg.rx_all, isp->msg.rx_good, msg_dma, cmd_minor,
		follow_pack_num, isp->shared->fw.stage);

	/* Handle the entry message */
	isp_msg_handle_one(isp, mc, ts);

	/* Handle follow message */
	for (i = 0; i < follow_pack_num; ++i) {
		tSoneCmdp *cmdp;
		struct media_command *queue_tail;

		cmdp = (tSoneCmdp *)isp->msg.cmdp_va;
		queue_tail = &cmdp->ch[FW_CH_INDEX].cqueue.c0[0] +
			     ARRAY_SIZE(cmdp->ch[FW_CH_INDEX].cqueue.c0);
		++mc;
		if (mc >= queue_tail) {
			dev_dbg(dev,
				"MSGRX: sn: %8lu/%8lu, DMA: 0x%08X exceed tail 0x%08X, rewind to start\n",
				isp->msg.rx_all, isp->msg.rx_good,
				isp_msg_addr_dma(isp, mc),
				isp_msg_addr_dma(isp, queue_tail));
			mc = &cmdp->ch[FW_CH_INDEX].cqueue.c0[0];
		}

		isp_msg_handle_one(isp, mc, ts);
	}
}

static int isp_notify_bound(struct v4l2_async_notifier *notifier,
			    struct v4l2_subdev *sd,
			    struct v4l2_async_subdev *asd)
{
	struct csi_device *csi_dev;
	struct csi_async_dev *csi_asd;
	struct isp_device *isp;
	struct device *dev;
	int i;

	isp = container_of(notifier, struct isp_device, notifier);
	dev = isp->dev;

	csi_dev = container_of(sd, struct csi_device, subdev);
	csi_asd = container_of(asd, struct csi_async_dev, async_dev);
	csi_asd->csi_dev = csi_dev;

	for (i = 0; i < ARRAY_SIZE(isp->channels); ++i) {
		struct isp_channel *channel;

		channel = &isp->channels[i];
		if (channel->csi_id == csi_dev->id) {
			int vc;

			dev_info(dev, "Channel %02d bound\n", channel->cid);
			vc = channel->csi_vc;
			channel->csi_channel = &(csi_dev->channels[vc]);
		}
	}

	return 0;
}

static void isp_notify_unbind(struct v4l2_async_notifier *notifier,
			      struct v4l2_subdev *sd,
			      struct v4l2_async_subdev *asd)
{
}

static int isp_notify_complete(struct v4l2_async_notifier *notifier)
{
	struct isp_device *isp;

	isp = container_of(notifier, struct isp_device, notifier);
	dev_info(isp->dev, "Notify completed\n");
	/* See also @update_camera_status */

	return 0;
};

static const struct v4l2_async_notifier_operations isp_v4l2_async_ops = {
	.bound = isp_notify_bound,
	.unbind = isp_notify_unbind,
	.complete = isp_notify_complete,
};

static void handle_camera_event(struct isp_device *isp,
				unsigned int notification,
				struct camera_dev *cam_dev)
{
	int i;
	struct device *dev;
	struct isp_channel *channel;
	struct isp_video *video;

	dev = isp->dev;

	dev_dbg(dev, "Event: 0x%08X, cam_dev: 0x%016llX\n", notification,
		(u64)cam_dev);
	if (cam_dev == NULL) {
		dev_err(dev, "Invalid camera event 0x%08X, cam_dev is NULL\n",
			notification);
		return;
	}

	channel = NULL;
	/* Find which channel binds this camera */
	for (i = 0; i < ARRAY_SIZE(isp->channels); ++i)
		if (isp->channels[i].cam_dev == cam_dev)
			channel = &isp->channels[i];

	if (channel == NULL) {
		dev_err(dev, "Can not find channel bound to camera 0x%02X\n",
			cam_dev->sensor_alias);
		return;
	}

	/* We only handle views video, since raw video is not normal scene */
	video = &channel->views_video;
	mutex_lock(&video->stream_lock);
	switch (notification) {
	case ISP_EVENT_CAMERA_DISCONNECT:
		cam_dev->power_on = false;
		video->channel->cfg->sensorOnline = false;
		if (video->status == VS_STREAM_ON && video->hw_on)
			isp_video_streamoff_hw(video);
		isp_msg_video_plugout(video);
		video->fw_ab_info.abnormalId = ABN_SW_CAMERA_PLUG_OUT;
		video->fw_ab_info.abnormalType = 0;
		video->error = true;
		mutex_lock(&video->vb2_lock);
		if (video->vb2_queue)
			wake_up(&video->vb2_queue->done_wq);
		mutex_unlock(&video->vb2_lock);
		break;
	case ISP_EVENT_CAMERA_CONNECT:
		cam_dev->power_on = true;
		video->channel->cfg->sensorOnline = true;
		isp_msg_video_plugin(video);
		if (video->status == VS_STREAM_ON) {
			isp_video_recycle_wait_bufs(video);
			isp_video_streamon_hw(video);
		}
		video->error = false;
		video->fw_ab_info.abnormalId = 0;
		video->fw_ab_info.abnormalType = 0;
		break;
	default:
		break;
	}
	mutex_unlock(&video->stream_lock);
}

static void isp_v4l2_dev_notify(struct v4l2_subdev *sd,
				unsigned int notification, void *arg)
{
	struct isp_device *isp;

	if (sd == NULL)
		return;

	isp = v4l2_to_isp_dev(sd->v4l2_dev);
	dev_dbg(isp->dev, "sd: %s, notification: 0x%X, arg: 0x%lX\n", sd->name,
		notification, (unsigned long)arg);

	switch (notification) {
	case ISP_EVENT_CAMERA_DISCONNECT:
	case ISP_EVENT_CAMERA_CONNECT:
		handle_camera_event(isp, notification, arg);
		break;
	default:
		break;
	}
}

static int init_v4l2_dev(struct isp_device *isp)
{
	int i;
	int rv;

	snprintf(isp->v4l2_dev.name, sizeof(isp->v4l2_dev.name),
		 ISP_DRIVER_NAME);
	rv = v4l2_device_register(isp->dev, &isp->v4l2_dev);
	if (rv < 0) {
		dev_err(isp->dev, "Failed to register V4L2 device: %d\n", rv);
		return -1;
	}

	isp->v4l2_dev.notify = isp_v4l2_dev_notify;
	v4l2_async_nf_init(&isp->notifier);
	isp->notifier.ops = &isp_v4l2_async_ops;

	for (i = 0; i < ARRAY_SIZE(isp->csi_asd); ++i) {
		if (isp->csi_asd[i].fwnode == NULL) {
			dev_dbg(isp->dev, "CSI %d not connected\n", i);
			continue;
		}

		isp->csi_asd[i].async_dev.match_type = V4L2_ASYNC_MATCH_FWNODE;
		isp->csi_asd[i].async_dev.match.fwnode = isp->csi_asd[i].fwnode;
		__v4l2_async_nf_add_subdev(&isp->notifier,
					   &(isp->csi_asd[i].async_dev));
	}

	rv = v4l2_async_nf_register(&isp->v4l2_dev, &(isp->notifier));
	if (rv < 0) {
		dev_err(isp->dev, "Failed to register notifier: %d\n", rv);
		goto err;
	}

	return 0;
err:
	/* NOTE: We does not cleanup since the asd is static in isp_device */
	// v4l2_async_nf_cleanup(&isp->notifier);
	v4l2_device_unregister(&isp->v4l2_dev);

	return -1;
}

static void cleanup_v4l2_dev(struct isp_device *isp)
{
	v4l2_async_nf_unregister(&isp->notifier);
	/* NOTE: We does not cleanup since the asd is static in isp_device */
	// v4l2_async_nf_cleanup(&isp->notifier);
	v4l2_device_unregister(&isp->v4l2_dev);
}

static void parse_camera_dt(struct isp_channel *channel, struct camera_dev *cam)
{
	int rv;
	u32 sem[3];
	struct resource res;
	ipc_reconf_t *cfg = channel->cfg;
	struct device *dev = channel->isp->dev;
	struct device_node *node = cam->i2c_client->adapter->dev.of_node;

	if (cam->role != ROLE_MASTER || !dt_is_raw(cam->data_type) ||
	    of_address_to_resource(node, 0, &res)) {
		cfg->i2cRegBase = 0;
		cfg->sensorDevID = 0;
	} else {
		cfg->i2cRegBase = phys_to_dma(dev, res.start);
		cfg->sensorDevID = cam->sensor_alias;
	}
	dev_dbg(dev, "i2cRegBase: 0x%08X. res.start: 0x%08llX\n",
		cfg->i2cRegBase, res.start);
	rv = of_property_read_u32_array(node, "ipc-sem", sem, ARRAY_SIZE(sem));
	if (rv) {
		dev_err(dev, "Failed to parse ipc-sem for %02d: %s\n",
			channel->cid, cam->name);
		/* NOTE: invalid sem bank and ID */
		cfg->semBank = ~0;
		cfg->semId = ~0;
		return;
	}
	/* NOTE: Sem master is ignored */
	cfg->semBank = sem[1];
	cfg->semId = sem[2];
}

static void init_channels(struct isp_device *isp)
{
	int i;
	struct device *dev = isp->dev;

	/* Update CSI devices' camera status first according to bound order */
	for (i = 0; i < ARRAY_SIZE(isp->csi_asd); ++i) {
		struct csi_device *csi_dev;

		csi_dev = isp->csi_asd[i].csi_dev;
		if (csi_dev)
			csi_update_camera_status(csi_dev);
	}

	/* Update channels' camera status from CSI */
	for (i = 0; i < ARRAY_SIZE(isp->channels); ++i) {
		struct isp_channel *channel;
		struct camera_dev *cam_dev;
		ipc_reconf_t *cfg;

		channel = &isp->channels[i];
		cfg = channel->cfg;
		mutex_init(&channel->lock);
		if (!channel->csi_channel || !channel->csi_channel->cam_dev)
			continue;

		channel->enabled = true;
		cam_dev = channel->csi_channel->cam_dev;
		channel->cam_dev = cam_dev;
		if (isp->role == ROLE_MASTER) {
			cfg->mipiSensorIndex = channel->cid;
			cfg->rawinfo.dataType = cam_dev->data_type;
			cfg->sensorOnline = cam_dev->power_on;
			parse_camera_dt(channel, cam_dev);
		} else {
			cam_dev->power_on = cfg->sensorOnline;
		}
		dev_dbg(dev, "INITCHANNELS: %02d, power: %u/%u\n", i,
			channel->cam_dev->power_on, channel->cfg->sensorOnline);
	}
}

static void cleanup_channels(struct isp_device *isp)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(isp->channels); ++i) {
		struct isp_channel *channel;

		channel = &isp->channels[i];
		mutex_destroy(&channel->lock);
	}
}

static int register_channel_videos(struct isp_device *isp)
{
	int i;
	struct device *dev;

	dev = isp->dev;
	for (i = 0; i < ARRAY_SIZE(isp->channels); ++i) {
		struct isp_channel *channel;
		struct isp_video *video;
		int cid;
		int rv;

		channel = &isp->channels[i];
		if (!channel->enabled) {
			dev_info(
				dev,
				"Channel %02d is not enabled, skip register videos\n",
				i);
			continue;
		}

		cid = channel->cid;
		video = &(channel->views_video);
		isp_video_init(&channel->views_video, channel, VIEW_VIDEO);
		rv = isp_video_register(video, &isp->v4l2_dev);
		if (rv < 0) {
			dev_err(dev,
				"Channel %02d: Failed to register view video\n",
				cid);
			return -1;
		}

		video = &(channel->raw_video);
		isp_video_init(&channel->raw_video, channel, RAW_VIDEO);
		rv = isp_video_register(video, &isp->v4l2_dev);
		if (rv < 0) {
			dev_err(dev,
				"Channel %02d: Failed to register raw video\n",
				cid);
			return -1;
		}
	}

	return 0;
}

static void unregister_channel_videos(struct isp_device *isp)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(isp->channels); ++i) {
		struct isp_channel *channel;

		channel = &isp->channels[i];
		if (!channel->enabled)
			continue;

		isp_video_unregister(&channel->views_video);
		isp_video_cleanup(&channel->views_video);
		isp_video_unregister(&channel->raw_video);
		isp_video_cleanup(&channel->raw_video);
	}
}

/* Parse ISP ports
 * @isp: ISP device
 *
 * Return number of "port" nodes found in "ports" node
 */
static int parse_ports(struct isp_device *isp)
{
	int i;
	int port_num;
	struct device *dev;
	struct device_node *node;

	dev = isp->dev;
	node = dev->of_node;
	port_num = 0;
	for (i = 0; i < ARRAY_SIZE(isp->channels); ++i) {
		int rv;
		int id;
		struct device_node *port;
		struct device_node *ep;
		struct isp_channel *channel;
		struct device_node *remote_port;
		struct device_node *remote_node;

		channel = &isp->channels[i];
		channel->cid = i;
		channel->isp = isp;
		/* NOTE:
		 * Why we set CSI ID and VC like this? Since it will be
		 * 1:1 matched with firmware's mipiSensorIndex, and can
		 * use it quickly.
		 * The channel is only a logical concept.
		 */
		channel->csi_id = channel->cid / MAX_VC_PER_CSI;
		channel->csi_vc = channel->cid % MAX_VC_PER_CSI;

		port = of_graph_get_port_by_id(node, i);
		if (!port)
			continue;

		ep = of_get_child_by_name(port, "endpoint");
		if (!ep) {
			dev_err(dev, "Port %02d does not have valid endpoint\n",
				i);
			of_node_put(port);
			continue;
		}

		remote_port = of_graph_get_remote_port(ep);
		if (!remote_port) {
			dev_err(dev,
				"Port %02d does not have valid remote port\n",
				i);
			of_node_put(port);
			of_node_put(ep);
			continue;
		}

		remote_node = of_graph_get_remote_port_parent(ep);
		if (!remote_node) {
			dev_err(dev,
				"Port %02d does not have valid remote node\n",
				i);
			of_node_put(port);
			of_node_put(ep);
			of_node_put(remote_port);
			continue;
		}
		dev_dbg(isp->dev, "Port %02d, remote node: %s, port: %s\n", i,
			remote_node->full_name, remote_port->full_name);

		rv = of_property_read_u32(remote_node, "id", &id);
		if (rv < 0) {
			dev_err(dev,
				"Port %02d, remote node does not have valid id\n",
				i);
			of_node_put(port);
			of_node_put(ep);
			of_node_put(remote_port);
			of_node_put(remote_node);
			return -EINVAL;
		}

		of_node_put(port);
		of_node_put(ep);
		of_node_put(remote_port);
		of_node_put(remote_node);

		if (id >= ARRAY_SIZE(isp->csi_asd))
			return -EINVAL;
		if (isp->csi_asd[id].used_ports == 0)
			isp->csi_asd[id].fwnode = of_fwnode_handle(remote_node);
		if (isp->csi_asd[id].fwnode == of_fwnode_handle(remote_node)) {
			++isp->csi_asd[id].used_ports;
		} else {
			dev_err(dev,
				"Port %02d, remote node has repetitive id\n",
				i);
			return -EINVAL;
		}
		++port_num;
	}

	return port_num;
}

static int parse_dt(struct isp_device *isp)
{
	int num_ports;
	struct device *dev;
	struct device_node *node;

	dev = isp->dev;
	node = dev->of_node;
	if (!node)
		return -EINVAL;

#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	if (of_property_read_u32(node, "uid", &isp->uid)) {
		dev_err(dev, "Failed to parse uid\n");
		return -EINVAL;
	}
	if (of_property_read_u32(node, "role", &isp->role))
		isp->role = ROLE_AUTO;
	{
		u32 sem[3];
		u32 rv;

		rv = of_property_read_u32_array(node, "ipc-sem", sem,
						ARRAY_SIZE(sem));
		if (rv) {
			dev_err(dev, "Failed to parse ipc-sem\n");

			return rv;
		}
		isp->sem_master = sem[0];
		isp->sem_bank = sem[1];
		isp->sem_id = sem[2];
	}
#endif
	num_ports = parse_ports(isp);
	if (num_ports <= 0) {
		dev_err(dev, "Failed to parse ports: %d\n", num_ports);
		return -EINVAL;
	}

	isp->use_ipc = of_property_read_bool(node, "use-ipc");
	if (isp->use_ipc) {
#ifdef CONFIG_BST_IPC
		dev_info(dev, "Use IPC mode\n");

		isp->ipc.cpu = IPC_CORE_ARM3;
		(void)of_property_read_u32(node, "ipc-cpu", &isp->ipc.cpu);
		dev_info(dev, "IPC id is specified as %u\n", isp->ipc.cpu);
#endif
	} else {
#ifdef CONFIG_BST_IPC_MSGBX
		u32 msgbox_cpu[3];
		u32 num;

		dev_info(dev, "Use MsgBox mode\n");
		num = of_property_read_variable_u32_array(node, "msgbox-cpu",
							  msgbox_cpu, 1, 3);
		if (num >= 3)
			isp->msgbx.data.com_data.sid = msgbox_cpu[2];
		if (num >= 2)
			isp->msgbx.data.com_data.fid = msgbox_cpu[1];
		if (num >= 1) {
			isp->msgbx.data.com_data.pid = msgbox_cpu[0];
			dev_info(
				dev,
				"MsgBox pid/fid/sid is specified as 0x%02X/0x%02X/0x%02X\n",
				isp->msgbx.data.com_data.pid,
				isp->msgbx.data.com_data.fid,
				isp->msgbx.data.com_data.sid);
		}
#endif
	}

	if (dev_is_dma_coherent(dev)) {
		isp->cache_mode = CM_HARDWARE;
		dev_info(dev, "Ensure cache consistency by hardware\n");
	} else if (!of_property_read_bool(node, "no-cache")) {
		isp->cache_mode = CM_SOFTWARE;
		/* NOTE: let core framework enable cache */
		dev->dma_coherent = true;
		dev_info(dev, "Ensure cache consistency by software\n");
	} else {
		isp->cache_mode = CM_NONE;
		dev_info(dev, "Cache is disabled\n");
	}

	isp->rstc = devm_reset_control_get_shared(isp->dev, "isp-reset");
	if (IS_ERR_OR_NULL(isp->rstc))
		dev_warn(dev, "Unable to parse isp-reset\n");

	isp->fw.bin = ISP_FW_BIN_PATH;
	isp->fw.slab = ISP_FW_SLAB_PATH;
	(void)of_property_read_string(node, "fw-bin", &isp->fw.bin);
	(void)of_property_read_string(node, "slab", &isp->fw.slab);
	dev_info(dev, "Use firmware: %s, slab: %s\n", isp->fw.bin,
		 isp->fw.slab);

	return 0;
}

static int of_mem_region_to_resource(struct device_node *node, unsigned int idx,
				     struct resource *res)
{
	int rv;
	struct device_node *region;

	region = of_parse_phandle(node, "memory-region", idx);
	if (!region)
		return -EINVAL;

	rv = of_address_to_resource(region, 0, res);
	of_node_put(region);

	return rv;
}

static int setup_fixed_mmu_map(struct isp_device *isp, unsigned long iova,
			       phys_addr_t pa, size_t size)
{
	int rv;
	size_t off;

	off = iova - PTR_ALIGN_DOWN(iova, IOVA_ALIGN_SIZE);
	iova -= off;
	pa -= off;
	size = ALIGN(size + off, IOVA_ALIGN_SIZE);

	dev_info(isp->dev, "Map 0x%08lX -> 0x%llX, size: %10lu/0x%08zX\n", iova,
		 pa, size, size);
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	rv = 0;
	if (!isp_hw_has_inited(isp))
		rv = iommu_map_by_proxy(COREIP_ISP_SID, iova, pa, size);
#else
	rv = iommu_map(isp->iommud, iova, pa, size, IOMMU_READ | IOMMU_WRITE);
#endif
	if (rv && rv != -EEXIST) {
		dev_err(isp->dev,
			"Map 0x%08lX -> 0x%llX, size: %10lu/0x%08zX, rv: %d, FAILED\n",
			iova, pa, size, size, rv);

		rv = -EPROBE_DEFER;
	}

	return rv;
}

static void finalize_fixed_mmu_map(struct isp_device *isp, unsigned long iova,
				   size_t size)
{
	size_t off;

	off = iova - PTR_ALIGN_DOWN(iova, IOVA_ALIGN_SIZE);
	iova -= off;
	size = ALIGN(size + off, IOVA_ALIGN_SIZE);
#ifndef CONFIG_VIDEO_BST_ISP_MULTI_OS
	iommu_unmap(isp->iommud, iova, size);
#endif
}

static int setup_inner_reg_region(struct isp_device *isp)
{
	struct device *dev;
	struct platform_device *pdev;
	struct resource *iomem;

	dev = isp->dev;
	pdev = isp->pdev;

	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!iomem) {
		dev_err(dev, "Failed to get IORESOURCE_MEM 0\n");
		goto err_reg0;
	}
	dev_info(dev, "Reg region 0 start: 0x%08llX, end: 0x%08llX\n",
		 iomem->start, iomem->end);
	isp->ctrl = devm_ioremap_resource(dev, iomem);
	if (IS_ERR(isp->ctrl)) {
		dev_err(dev, "Failed to map ctrl base: %ld\n",
			PTR_ERR(isp->ctrl));
		goto err_reg0;
	}

	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!iomem) {
		dev_err(dev, "Failed to get IORESOURCE_MEM 1\n");
		goto err_reg1;
	}
	dev_info(dev, "Reg region 1 start: 0x%08llX, end: 0x%08llX\n",
		 iomem->start, iomem->end);
	isp->pram = devm_ioremap_resource(dev, iomem);
	if (IS_ERR(isp->pram)) {
		dev_err(dev, "Failed to map pram base: %ld\n",
			PTR_ERR(isp->pram));
		goto err_reg1;
	}

	return 0;

err_reg1:
	devm_iounmap(dev, isp->ctrl);
err_reg0:
	return -1;
}

static void finalize_inner_reg_region(struct isp_device *isp)
{
	devm_iounmap(isp->dev, isp->pram);
	devm_iounmap(isp->dev, isp->ctrl);
}

static int setup_outer_reg_region(struct isp_device *isp)
{
	int rv;
	struct device *dev;
	struct platform_device *pdev;
	int i;
	int idx;

	dev = isp->dev;
	pdev = isp->pdev;
	rv = 0;
	idx = OUTER_REG_START;
	for (i = 0; i < ARRAY_SIZE(isp->outer_reg); ++i) {
		struct resource *iomem;
		size_t size;

		/* If IOMMU is enabled, we should map GTC and IPC registers for ISP firmware */
		iomem = platform_get_resource(pdev, IORESOURCE_MEM, idx);
		if (!iomem) {
			dev_err(dev, "Failed to get IORESOURCE_MEM %d\n", idx);
			rv = -EINVAL;
			break;
		}
		dev_info(dev, "Reg region %d start: 0x%08llX, end: 0x%08llX\n",
			 idx, iomem->start, iomem->end);
		size = resource_size(iomem);
		if (isp->iommud) {
			rv = setup_fixed_mmu_map(isp, iomem->start,
						 iomem->start, size);
			if (rv && rv != -EEXIST)
				break;
		}
		isp->outer_reg[i].start = iomem->start;
		isp->outer_reg[i].size = size;
		++idx;
	}

	return rv;
}

static void finalize_outer_reg_region(struct isp_device *isp)
{
	int i;

	if (!isp->iommud)
		return;

	for (i = 0; i < ARRAY_SIZE(isp->outer_reg); ++i)
		if (isp->outer_reg[i].start)
			finalize_fixed_mmu_map(isp, isp->outer_reg[i].start,
					       isp->outer_reg[i].size);
}

/*
 * Parse and setup memory regions.
 * The memory regions layout, must be in order:
 * Message memory between driver and firmware, required.
 * Reserved memory for firmware, required.
 * IPC message memory, required when IPC is used.
 * Frame buffers memory, required when IPC is used.
 *
 * @isp: ISP device.
 *
 * Returns 0 for success
 */
static int setup_mem_region(struct isp_device *isp)
{
	int rv;
	int idx;
	struct device *dev;
	struct device_node *node;
	struct resource iomem;

	dev = isp->dev;
	node = dev->of_node;

	idx = 0;
	rv = of_mem_region_to_resource(node, idx++, &iomem);
	if (rv) {
		dev_err(dev, "No memory assigned for message exchange\n");
		goto err_mem_msg;
	}
	dev_info(dev, "Reserved 0x%08llX - 0x%08llX for message exchange\n",
		 iomem.start, iomem.end);
	isp->msg.init_pa = iomem.start;
	isp->msg.init_dma = phys_to_dma(dev, isp->msg.init_pa);
	isp->msg.init_size = resource_size(&iomem);
	isp->msg.init_va = devm_memremap(dev, isp->msg.init_pa,
					 isp->msg.init_size, MEMREMAP_WC);
	if (IS_ERR(isp->msg.init_va)) {
		dev_err(dev, "Failed to map message region: %ld\n",
			PTR_ERR(isp->msg.init_va));
		goto err_mem_msg;
	}
	if (!isp_hw_has_inited(isp))
		memset(isp->msg.init_va, 0, isp->msg.init_size);

	rv = of_mem_region_to_resource(node, idx++, &iomem);
	if (rv) {
		dev_err(dev, "No memory assigned for reserved buffer\n");
		goto err_mem_rsv;
	}
	dev_info(dev, "Reserved 0x%08llX - 0x%08llX for firmware\n",
		 iomem.start, iomem.end);
	isp->fw.rsv_pa = iomem.start;
	isp->fw.rsv_dma = phys_to_dma(dev, iomem.start);
	isp->fw.rsv_size = resource_size(&iomem);
	isp->fw.rsv_va = devm_memremap(dev, isp->fw.rsv_pa, isp->fw.rsv_size,
				       MEMREMAP_WC);
	if (IS_ERR(isp->fw.rsv_va)) {
		dev_err(dev, "Failed to map reserved region: %ld\n",
			PTR_ERR(isp->fw.rsv_va));
		goto err_mem_rsv;
	}
	if (!isp_hw_has_inited(isp))
		memset(isp->fw.rsv_va, 0, isp->fw.rsv_size);

#ifdef CONFIG_BST_IPC
	if (isp->use_ipc) {
		rv = of_mem_region_to_resource(node, idx++, &iomem);
		if (rv) {
			dev_err(dev, "No memory assigned for IPC\n");
			goto err_mem_ipc;
		}
		dev_info(dev, "Reserved 0x%08llX - 0x%08llX for IPC\n",
			 iomem.start, iomem.end);

		isp->ipc.msg_pa = iomem.start;
		isp->ipc.msg_dma = phys_to_dma(dev, iomem.start);
		isp->ipc.msg_size = resource_size(&iomem);
	}
#endif

	/* If IOMMU is enabled, we should map memories for ISP firmware */
	if (isp->iommud) {
		rv = setup_fixed_mmu_map(isp, isp->msg.init_dma,
					 isp->msg.init_pa, isp->msg.init_size);
		if (rv && rv != -EEXIST)
			goto err_map_msg;

		rv = setup_fixed_mmu_map(isp, isp->fw.rsv_dma, isp->fw.rsv_pa,
					 isp->fw.rsv_size);
		if (rv && rv != -EEXIST)
			goto err_map_rsv;

#ifdef CONFIG_BST_IPC
		if (isp->use_ipc) {
			rv = setup_fixed_mmu_map(isp, isp->ipc.msg_dma,
						 isp->ipc.msg_pa,
						 isp->ipc.msg_size);
			if (rv && rv != -EEXIST)
				goto err_map_ipc;
		}
#endif
	}

	dma_set_mask_and_coherent(isp->dev, DMA_MASK);
	dma_set_max_seg_size(isp->dev, DMA_MAX_SIZE);

	return 0;

#ifdef CONFIG_BST_IPC
err_map_ipc:
	if (isp->iommud)
		finalize_fixed_mmu_map(isp, isp->fw.rsv_dma, isp->fw.rsv_size);
#endif
err_map_rsv:
	if (isp->iommud)
		finalize_fixed_mmu_map(isp, isp->msg.init_dma,
				       isp->msg.init_size);
err_map_msg:
#ifdef CONFIG_BST_IPC
err_mem_ipc:
#endif
err_mem_rsv:
	devm_memunmap(dev, isp->msg.init_va);
err_mem_msg:
	return -1;
}

static void finalize_mem_region(struct isp_device *isp)
{
	if (isp->iommud) {
#ifdef CONFIG_BST_IPC
		finalize_fixed_mmu_map(isp, isp->ipc.msg_dma,
				       isp->ipc.msg_size);
#endif
		finalize_fixed_mmu_map(isp, isp->msg.init_dma,
				       isp->msg.init_size);
		finalize_fixed_mmu_map(isp, isp->fw.rsv_dma, isp->fw.rsv_size);
	}
	devm_memunmap(isp->dev, isp->msg.init_va);
}

/*
 * Setup message area between driver and firmware.
 * This should be called after setup_mem_region
 *
 * @isp: ISP device.
 */
static void setup_msg_area(struct isp_device *isp)
{
	int i;
	struct device *dev;
	void *va;
	dma_addr_t dma;
	size_t size_origin;
	size_t size_align;
	tSoneInit *init;
	tSoneCmdp *cmdp;

	dev = isp->dev;
	va = isp->msg.init_va;
	dma = isp->msg.init_dma;

	/* Setup init partition */
	size_origin = sizeof(*init);
	size_align = ALIGN(size_origin, SONE_PART_ALIGN_SIZE);
	dev_info(dev,
		 "Init: dma: 0x%08llX, size: %8zu, align to %8zu/0x%08lX\n",
		 dma, size_origin, size_align, size_align);
	init = (tSoneInit *)va;
	if (!isp_hw_has_inited(isp))
		setup_init_parti(init, (dma & LOW_32_BIT_MASK),
				 ((dma >> 32) & LOW_32_BIT_MASK), "LNX", "ARM",
				 0xFFFFFFFF, "ISP", 'I');
	va += size_align;
	dma += size_align;

	/* Setup cmd partition */
	isp->msg.cmdp_va = va;
	size_origin = sizeof(*cmdp);
	size_align = ALIGN(size_origin, SONE_PART_ALIGN_SIZE);
	dev_info(dev,
		 "Cmdp: dma: 0x%08llX, size: %8zu, align to %8zu/0x%08lX\n",
		 dma, size_origin, size_align, size_align);
	cmdp = (tSoneCmdp *)va;
	if (!isp_hw_has_inited(isp))
		setup_cmdp_parti(init, cmdp, dma);
	va += size_align;
	dma += size_align;

	/* Setup slab partition */
	isp->msg.slab_va = va;
	size_origin = SONE_MEDIA_SLAB_BUFSIZE;
	size_align = ALIGN(size_origin, SONE_BUF_ADDR_ALIGN_SIZE);
	dev_info(dev,
		 "Slab: dma: 0x%08llX, size: %8zu, align to %8zu/0x%08lX\n",
		 dma, size_origin, size_align, size_align);
	if (!isp_hw_has_inited(isp))
		setup_slab_parti(init, va, (dma & LOW_32_BIT_MASK));
	va += size_align;
	dma += size_align;

	/* Setup config payload partition */
	isp->msg.conf_va = va;
	size_origin = sizeof(ipc_reconf_t);
	size_align = ALIGN(size_origin, ISP_MSG_PAYLOAD_ALIGN);
	isp->msg.conf_size = size_align;
	dev_info(
		dev,
		"Conf: dma: 0x%08llX, size: %8zu, align to %8zu/0x%08lX, num: %zu\n",
		dma, size_origin, size_align, size_align,
		ARRAY_SIZE(isp->channels));
	for (i = 0; i < ARRAY_SIZE(isp->channels); ++i) {
		isp->channels[i].cfg = va;
		va += size_align;
		dma += size_align;
	}

	isp->shared = va;
	size_origin = sizeof(struct isp_shared);
	size_align = ALIGN(size_origin, ISP_MSG_PAYLOAD_ALIGN);
	dev_info(dev,
		 "Shrd: dma: 0x%08llX, size: %8zu, align to %8zu/0x%08lX\n",
		 dma, size_origin, size_align, size_align);
	va += size_align;
	dma += size_align;

	/* Setup file(ALGO, IQ) partition */
	isp->shared->file.next_dma = dma;
	size_origin = isp->msg.init_size - (va - isp->msg.init_va);
	size_align = ALIGN(size_origin, ISP_MSG_PAYLOAD_ALIGN);
	dev_info(dev,
		 "File: dma: 0x%08llX, size: %8zu, align to %8zu/0x%08lX\n",
		 dma, size_origin, size_align, size_align);

	/* Setup shared */
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	if (isp->role == ROLE_MASTER) {
		isp->msg.cmds = cmdp->ch[DRV_CH_INDEX].cqueue.c0;
		isp->msg.cmd_size =
			ARRAY_SIZE(cmdp->ch[DRV_CH_INDEX].cqueue.c0);
	} else {
		u32 i;

		i = isp->uid % ARRAY_SIZE(isp->shared->mcs);
		isp->shared->mcs[i].uid = isp->uid;
		isp->msg.cmds = isp->shared->mcs[i].cmds;
		isp->msg.cmd_size = ARRAY_SIZE(isp->shared->mcs[i].cmds);
	}
#else
	isp->msg.cmds = cmdp->ch[DRV_CH_INDEX].cqueue.c0;
	isp->msg.cmd_size = ARRAY_SIZE(cmdp->ch[DRV_CH_INDEX].cqueue.c0);
#endif
}

bool isp_is_mapped_addr(struct isp_device *isp, dma_addr_t addr)
{
	int i;

	if (!isp->iommud)
		return true;

	if (addr >= isp->msg.init_dma &&
	    addr < isp->msg.init_dma + isp->msg.init_size)
		return true;

	if (addr >= isp->fw.rsv_dma &&
	    addr < isp->fw.rsv_dma + isp->fw.rsv_size)
		return true;

#ifdef CONFIG_BST_IPC
	if (addr >= isp->ipc.msg_dma &&
	    addr < isp->ipc.msg_dma + isp->ipc.msg_size)
		return true;
#endif

	for (i = 0; i < ARRAY_SIZE(isp->outer_reg); ++i)
		if (addr >= isp->outer_reg[i].start &&
		    addr < isp->outer_reg[i].start + isp->outer_reg[i].size)
			return true;

	return false;
}

static int isp_probe(struct platform_device *pdev)
{
	struct isp_device *isp;
	struct device *dev;
	int rv;
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	u32 uid;
#endif

	dev = &pdev->dev;
	isp = devm_kzalloc(dev, sizeof(*isp), GFP_KERNEL);
	if (!isp)
		return -ENOMEM;

	isp->dev = dev;
	isp->pdev = pdev;
	isp->iommud = iommu_get_domain_for_dev(dev);
	if (isp->iommud)
		dev_info(dev, "Use SMMU, page size bitmap: 0x%lX\n",
			 isp->iommud->pgsize_bitmap);

	rv = parse_dt(isp);
	if (rv) {
		dev_err(dev, "Failed to parse device tree\n");
		return -EINVAL;
	}

	if (!IS_ERR_OR_NULL(isp->rstc))
		reset_control_deassert(isp->rstc);

	rv = setup_inner_reg_region(isp);
	if (rv) {
		dev_err(dev, "Failed to setup inner reg region\n");
		return -EIO;
	}

#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	isp->hwlock =
		bst_semaphore_init(isp->sem_master, isp->sem_bank, isp->sem_id);
	if (!isp->hwlock) {
		dev_err(dev, "Failed to claim HW lock\n");
		rv = -ENOLCK;
		goto err_init_lock;
	}
	rv = get_sem_lock_with_timeout(isp->hwlock, 1);
	if (rv != 0 && rv != isp->sem_master) {
		dev_info(dev, "Sem %u of bank %u is hold by master %u\n",
			 isp->sem_id, isp->sem_bank, rv);
		rv = -EPROBE_DEFER;
		goto err_lock;
	}
	uid = readl_relaxed(isp->ctrl + R_TOP_UID);
	dev_info(dev, "Current UID: %u, I: %u\n", uid, isp->uid);
	if (isp->role == ROLE_AUTO) {
		if (uid != 0 && uid != isp->uid)
			isp->role = ROLE_SLAVE;
		else
			isp->role = ROLE_MASTER;
	}
#else
	isp->role = ROLE_MASTER;
#endif

	rv = setup_outer_reg_region(isp);
	if (rv) {
		dev_err(dev, "Failed to setup outer reg region\n");
		goto err_setup_outer_reg;
	}

	rv = setup_mem_region(isp);
	if (rv) {
		dev_err(dev, "Failed to setup mem region\n");
		goto err_setup_mem;
	}

	setup_msg_area(isp);

	/* Initialize V4l2 and videos */
	rv = init_v4l2_dev(isp);
	if (rv)
		goto err_init_v4l2_dev;

	mutex_init(&isp->lock);
	mutex_init(&isp->msg.tx_lock);
	init_completion(&isp->msg.tx_comp);
	mutex_init(&isp->fw.cfg_lock);
	init_completion(&isp->fw.boot_comp);

	init_channels(isp);
	register_channel_videos(isp);

#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	if (!isp_hw_has_inited(isp))
#endif
		isp_fsync_setup(isp);

	rv = isp_misc_init(isp);
	if (rv < 0)
		dev_err(dev, "Failed to init misc device\n");
	isp_sysfs_init(isp);
	init_completion(&isp->async_msg_comp);
	INIT_KFIFO(isp->async_msg_fifo);
	isp->async_msg_task =
		kthread_run(isp_msg_handler_async, isp, "isp-async-mc");
	if (IS_ERR(isp->async_msg_task)) {
		rv = PTR_ERR(isp->async_msg_task);
		dev_err(dev, "Failed to create async cmd handler, rv: %d\n",
			rv);
		goto err_setup_async_msg_handler;
	}

#ifdef CONFIG_BST_IPC
	if (isp->use_ipc) {
		rv = isp_msg_init_ipc(isp);
		if (rv)
			goto err_setup_msg;
	}
#endif
#ifdef CONFIG_BST_IPC_MSGBX
	if (!isp->use_ipc) {
		rv = isp_msg_init_msgbox(isp);
		if (rv)
			goto err_setup_msg;
	}
#endif
	isp->ops.msg_rx = isp_msg_handler_main;

	/* NOTE:
	 * Why we update devices' status here?
	 * Since all pipelines share 1 V4L2 device, consider 1 subdev is
	 * unbound, the notify_complete will not be called, then all channels
	 * are unusable. If we use 1 V4L2 device for 1 video,
	 * the following statements should be called in notify_complete.
	 */
	if (isp->role == ROLE_MASTER)
		isp_power_peer_devs(isp, 1);

#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	if (isp->role == ROLE_MASTER) {
		writel_relaxed(isp->uid, isp->ctrl + R_TOP_UID);
		writel_relaxed(isp->msg.init_dma, isp->ctrl + R_TOP_MAILBOX_IN);
	}
	release_sem_lock(isp->hwlock);
#endif

	/* Enable default features */
	isp->merge_msg = 1;
	platform_set_drvdata(pdev, isp);
	dev_info(dev, "Probe done on CPU %u, role: %s\n", smp_processor_id(),
		 isp_str_role(isp->role));

	return 0;

#if defined(CONFIG_BST_IPC_MSGBX) || defined(CONFIG_BST_IPC)
err_setup_msg:
#endif
	kthread_stop(isp->async_msg_task);
err_setup_async_msg_handler:
	kfifo_reset(&isp->async_msg_fifo);
	isp_sysfs_exit(isp);
	isp_misc_exit(isp);
	unregister_channel_videos(isp);
	cleanup_channels(isp);
	cleanup_v4l2_dev(isp);
err_init_v4l2_dev:
	finalize_mem_region(isp);
err_setup_mem:
	finalize_outer_reg_region(isp);
err_setup_outer_reg:
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	release_sem_lock(isp->hwlock);
err_lock:
	samphore_lock_remove(isp->hwlock);
err_init_lock:
#endif
	finalize_inner_reg_region(isp);

	return rv;
}

/*
 * isp_remove - Remove ISP platform device
 * @pdev: Pointer to ISP platform device
 *
 * Returns 0 always
 */
static int isp_remove(struct platform_device *pdev)
{
	struct isp_device *isp;
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	int i;
#endif

	isp = platform_get_drvdata(pdev);
	dev_info(&pdev->dev, "Remove\n");

#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	for (i = 0; i < ARRAY_SIZE(isp->csi_asd); ++i)
		if (isp->csi_asd[i].csi_dev)
			v4l2_subdev_call(&isp->csi_asd[i].csi_dev->subdev,
					 video, post_streamoff);
#endif

#ifdef CONFIG_BST_IPC
	if (isp->use_ipc)
		isp_msg_exit_ipc(isp);
#endif
#ifdef CONFIG_BST_IPC_MSGBX
	if (!isp->use_ipc)
		isp_msg_exit_msgbox(isp);
#endif
	kthread_stop(isp->async_msg_task);
	kfifo_reset(&isp->async_msg_fifo);
	isp_sysfs_exit(isp);
	isp_misc_exit(isp);

	unregister_channel_videos(isp);
	cleanup_channels(isp);
	cleanup_v4l2_dev(isp);

#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	release_sem_lock(isp->hwlock);
#else
	finalize_mem_region(isp);
	finalize_outer_reg_region(isp);
	finalize_inner_reg_region(isp);
#endif
	return 0;
}

static void isp_shutdown(struct platform_device *pdev)
{
	struct isp_device *isp;

	isp = platform_get_drvdata(pdev);
	dev_info(isp->dev, "Shutdown\n");

#ifndef CONFIG_VIDEO_BST_ISP_MULTI_OS
	isp_power_peer_devs(isp, 0);
	if (!IS_ERR_OR_NULL(isp->rstc))
		reset_control_assert(isp->rstc);
	finalize_mem_region(isp);
	finalize_outer_reg_region(isp);
	finalize_inner_reg_region(isp);
#endif
}

static int isp_suspend(struct device *dev)
{
	int i;
	struct platform_device *pdev = to_platform_device(dev);
	struct isp_device *isp = platform_get_drvdata(pdev);

	dev_info(isp->dev, "Suspend\n");
	if (isp->role == ROLE_MASTER) {
		// Re-initialize firmware
		isp->shared->fw.stage = FS_UNUSED;
		isp_power_peer_devs(isp, 0);
	}

	// Clean all FW related data
	/* NOTE: This is used for resume from STR */
	for (i = 0; i < ARRAY_SIZE(isp->channels); ++i) {
		struct isp_channel *channel;
		struct isp_video *video;

		channel = &isp->channels[i];
		if (!channel->enabled)
			continue;

		video = &channel->views_video;
		video->hw_on = 0;

		video = &channel->raw_video;
		video->hw_on = 0;
	}

	return 0;
}

static int isp_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct isp_device *isp = platform_get_drvdata(pdev);

	dev_info(isp->dev, "Resume\n");
	isp_msg_resume(isp);
	if (isp->role == ROLE_MASTER) {
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
		writel_relaxed(isp->uid, isp->ctrl + R_TOP_UID);
#endif
		if (!isp_hw_has_inited(isp))
			isp_fsync_setup(isp);
		isp_power_peer_devs(isp, 1);
	}

	return 0;
}

static const struct dev_pm_ops isp_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(isp_suspend, isp_resume)
};

static const struct of_device_id isp_of_table[] = {
	{ .compatible = "bst,c1200-isp" },
	{},
};
MODULE_DEVICE_TABLE(of, isp_of_table);

static struct platform_driver isp_driver = {
	.probe = isp_probe,
	.remove = isp_remove,
	.shutdown = isp_shutdown,
	.driver = {
		.name = ISP_DRIVER_NAME,
		.of_match_table = isp_of_table,
		.pm = &isp_pm_ops,
	},
};
module_platform_driver(isp_driver);

MODULE_VERSION(ISP_DRIVER_VERSION);
MODULE_DESCRIPTION("BST ISP driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("BST Ltd.");
