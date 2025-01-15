// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"

#include <linux/cacheflush.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-map-ops.h>
#include <linux/i2c.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/pagemap.h>
#include <linux/scatterlist.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-vmalloc.h>
#include <media/mipi-csi2.h>

#include <linux/coreip/proto_api_common.h>

#include "cam_entity.h"
#include "isp_core.h"
#include "isp_fw_loader.h"
#include "isp_video.h"
#include "proto_isp_ipc.h"

#define MAX_DELAY_CYCLE_TO_RESEND 8
#define MIN_FRAME_BUFFER_NUM	  3
#define DEFAULT_IMAGE_WIDTH	  1920
#define DEFAULT_IMAGE_HEIGHT	  1080
#define MAX_ENTITY_NAME		  16
#define ISP_LINE_ALIGIN		  256

static void resend_wait_bufs_over_cycle(struct c1200_isp_video *video,
					int cycle);
static int send_next_buf_to_fw(struct c1200_isp_video *video);

/* -----------------------------------------------------------------------------
 * Helper functions
 */
static int handle_video_sync_msg(struct c1200_isp_video *video,
				 struct internal_msg *cache_msg);

static int get_view_bytesperline(struct isp_view *pview);
static int get_view_sizeimage(struct isp_view *pview);
static int get_extend_size(struct c1200_isp_video *video);

static inline int is_reserved_buf(struct c1200_isp_video *video, uint32_t fbuf)
{
	uint64_t fbuf_paddr_start = video->isp->fbuf_paddr;
	uint64_t fbuf_paddr_end = fbuf_paddr_start + video->isp->fbuf_psize;

	return ((fbuf >= fbuf_paddr_start) && (fbuf < fbuf_paddr_end));
}

/* -----------------------------------------------------------------------------
 * Video queue operations
 * this function will be called by videobuf2 when user reqbuffer,
 * call it to enture the request buffer num and plane is supported by driver
 */

static int isp_video_queue_setup(struct vb2_queue *queue, unsigned int *count,
				 unsigned int *num_planes, unsigned int sizes[],
				 struct device *alloc_devs[])
{
	struct c1200_isp_video_fh *vfh = vb2_get_drv_priv(queue);
	struct c1200_isp_video *video = vfh->video;
	unsigned int buf_num = MAX_FRAME_BUF_NUM;
	int i;

	for (i = 0; i < vfh->format.fmt.pix_mp.num_planes; i++) {
		int index = video->current_views_list[i];

		sizes[i] = vfh->format.fmt.pix_mp.plane_fmt[index].sizeimage +
			   get_extend_size(video);
		if (sizes[i] == 0) {
			dev_err(video->isp->dev,
				"video %02d: plane %d, invalid sizeimage 0\n",
				video->video_index, i);
			return -EINVAL;
		}
	}

	*num_planes = vfh->format.fmt.pix_mp.num_planes;
	*count = min(*count, buf_num);

	return 0;
}

static void isp_video_buffer_finish(struct vb2_buffer *buf)
{
	struct c1200_isp_video_fh *vfh = vb2_get_drv_priv(buf->vb2_queue);
	struct c1200_isp_video *video = vfh->video;
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(buf);
	struct isp_buffer *buffer = to_isp_buffer(vbuf);
	dma_addr_t addr;
	int i;
	int video_status;

	for (i = 0; i < vfh->format.fmt.pix_mp.num_planes; i++) {
		if (buf->memory == VB2_MEMORY_MMAP) {
			addr = vb2_dma_contig_plane_dma_addr(buf, i);

			video_status = atomic_read(&video->status);
			if (video_status >= VIDEO_STATUS_STREAMOFFING) {
				dev_dbg(video->isp->dev,
					"%s: video %02d: status = %d\n",
					__func__, video->video_index,
					video_status);
			} else {
				if (!video->isp->iommud) {
					dma_sync_single_for_cpu(
						&video->video.dev, addr,
						buf->planes[i].length,
						DMA_FROM_DEVICE);
				} else {
					struct sg_table sgt;
					void *vaddr;

					vaddr = vb2_plane_vaddr(buf, i);
					if (dma_get_sgtable(
						    video->isp->dev, &sgt,
						    vaddr, addr,
						    buf->planes[i].length) <
					    0) {
						dev_err(video->isp->dev,
							"failed to get scatterlist from DMA API\n");
						return;
					}
					dma_sync_sgtable_for_cpu(
						video->isp->dev, &sgt,
						DMA_FROM_DEVICE);
					sg_free_table(&sgt);
				}

				dev_dbg(&video->video.dev,
					"%s: video: %02d, plane: %d, mmap_uaddr: %p, addr %x, plane_size = %d\n",
					__func__, video->video_index, i,
					buffer->mmap_uaddr[i],
					buffer->dma[video->current_views_list[i]],
					buf->planes[i].length);
			}
		}
	}
}

/*-----------------------------------------------------------------------------
 *this will be called by videobuf2 after vb2 prepare buffer successed.
 *vb2 will do prepare before the buffer could be used.
 *this function just need to check vb2 buffer with user setting
 */
static int isp_video_buffer_prepare(struct vb2_buffer *buf)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(buf);
	struct c1200_isp_video_fh *vfh = vb2_get_drv_priv(buf->vb2_queue);
	struct isp_buffer *buffer = to_isp_buffer(vbuf);
	struct c1200_isp_video *video = vfh->video;
	dma_addr_t addr;
	int i;

	for (i = 0; i < vfh->format.fmt.pix_mp.num_planes; i++) {
		int index = video->current_views_list[i];

		if (vb2_plane_size(buf, i) <
		    vfh->format.fmt.pix_mp.plane_fmt[index].sizeimage) {
			dev_dbg(&video->video.dev,
				"%s: video: %02d, plane: %d, size: %lu, %u\n",
				__func__, video->video_index, i,
				vb2_plane_size(buf, i),
				vfh->format.fmt.pix_mp.plane_fmt[index]
					.sizeimage);
			return -EINVAL;
		}
		vb2_set_plane_payload(
			buf, i,
			vfh->format.fmt.pix_mp.plane_fmt[index].sizeimage);
		// to do, only need do it at first time
		addr = vb2_dma_contig_plane_dma_addr(buf, i);
		if (!video->isp->iommud)
			buffer->dma[video->current_views_list[i]] =
				0xc0000000 +
				(uint32_t)(addr &
					   LOW_32_BIT_MASK); // windy add here
		else
			buffer->dma[video->current_views_list[i]] = addr;
	}

	return 0;
}

/*
 * isp_video_buffer_queue - Add buffer to streaming queue
 * @buf: Video buffer
 * this will be called by vb2 qbuf,vb2 add this buffer to it's free list
 * here we add buffer to driver used list ,when one buffer is done,we put
 * one buffer to fw from driver list
 */
static void isp_video_buffer_queue(struct vb2_buffer *buf)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(buf);
	struct c1200_isp_video_fh *vfh = vb2_get_drv_priv(buf->vb2_queue);
	struct isp_buffer *buffer = to_isp_buffer(vbuf);
	struct c1200_isp_video *video = vfh->video;
	unsigned int empty;

	dev_dbg(video->isp->dev, "%s: video %02d\n", __func__,
		video->video_index);
	if (unlikely(video->error)) {
		vb2_buffer_done(&buffer->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		return;
	}

	if (video->isp->data_mode == DATA_MODE_FEED_DRIVER) {
		mutex_lock(&video->wait_queue_lock);
		if (unlikely(video->error))
			dev_err(video->isp->dev, "%s: video%02d is error",
				__func__, video->video_index);
		else
			list_add_tail(&buffer->node, &video->wait_buf_queue);
		mutex_unlock(&video->wait_queue_lock);

		// dev_info(video->isp->dev, "[ISP_FEED]: video_id = %d, paddr =
		// 0x%x, size = %d, state = %d, index = %d",
		//	video->device_index, buffer->dma[0],
		//buffer->plane_size[0], buffer->vb.vb2_buf.state,
		//buffer->vb.vb2_buf.index);
		complete(&video->feed_wait_buf_completion);
		// dev_info(video->isp->dev, "[ISP_FEED]: push buf to wait
		// queue, wake up feed_wait_buf_completion");
		return;
	}

	mutex_lock(&video->free_queue_lock);
	empty = list_empty(&video->free_buf_queue);
	buffer->cycle_count = 0;
	list_add_tail(&buffer->node, &video->free_buf_queue);
	mutex_unlock(&video->free_queue_lock);
	send_next_buf_to_fw(video);
}

/*
 * c1200_isp_video_return_buffers - Return all queued buffers to videobuf2
 * @video: ISP video object
 * @state: new state for the returned buffers
 *
 * Return all buffers queued on the video node to videobuf2 in the given state.
 * The buffer state should be VB2_BUF_STATE_QUEUED if called due to an error
 * when starting the stream, or VB2_BUF_STATE_ERROR otherwise.
 *
 * The function must be called with the video irqlock held.
 */
static void c1200_isp_video_return_buffers(struct c1200_isp_video *video,
					   enum vb2_buffer_state state)
{
	int free_count = 0;
	int wait_count = 0;
	int max_wait_fw_return_cnt = 8;
	int is_empty = 0;

	mutex_lock(&video->free_queue_lock);
	while (!list_empty(&video->free_buf_queue)) {
		struct isp_buffer *buf;

		free_count++;
		buf = list_first_entry(&video->free_buf_queue,
				       struct isp_buffer, node);
		list_del(&buf->node);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
		dev_dbg(&video->video.dev,
			"free list count %d ,buffer address  is %x\n",
			free_count, buf->dma[0]);
	}
	mutex_unlock(&video->free_queue_lock);

	if (video->isp->data_mode != DATA_MODE_FEED_DRIVER) {
		while (!is_empty) {
			mutex_lock(&video->wait_queue_lock);
			is_empty = list_empty(&video->wait_buf_queue);
			mutex_unlock(&video->wait_queue_lock);

			if (!is_empty) {
				max_wait_fw_return_cnt--;
				dev_dbg(&video->video.dev,
					"wait for fw return buf %d",
					max_wait_fw_return_cnt);
				msleep(100);
				if (!max_wait_fw_return_cnt)
					break;
			}
		}
	}

	mutex_lock(&video->wait_queue_lock);
	while (!list_empty(&video->wait_buf_queue)) {
		struct isp_buffer *buf;

		wait_count++;
		buf = list_first_entry(&video->wait_buf_queue,
				       struct isp_buffer, node);
		list_del(&buf->node);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
		dev_dbg(&video->video.dev,
			"wait list count %d ,buffer address  is %x\n",
			wait_count, buf->dma[0]);
	}
	mutex_unlock(&video->wait_queue_lock);

	dev_dbg(&video->video.dev, "kernel buffer number is %d",
		free_count + wait_count);
}

int send_video_buf_to_fw(struct c1200_isp_video *video, uint32_t *buf_paddr,
			 int cmd_index, int reserve_flag)
{
	struct media_command *cmd;
	ipc_msg msg;
	int ret;
	int video_status;
	int i;
	isp_raw_buf_t *new_raw_buffer; // raw video
	isp_new_frame_buf_t *new_view_buffer; // view video

	video_status = atomic_read(&(video->status));
	if (video_status >= VIDEO_STATUS_STREAMOFFING) {
		dev_err(video->isp->dev, "%s: video %02d, status %d\n",
			__func__, video->video_index, video_status);
		return -1;
	}

	cmd = isp_get_media_cmd(video->isp);
	memset(cmd, 0, sizeof(*cmd));

	if (video->is_raw_video) { // raw video
		cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_RAW_BUF;
		new_raw_buffer = (isp_raw_buf_t *)(&cmd->user_cmd_data[0]);
		new_raw_buffer->sensorId = S00RawId + video->chn_index;
		new_raw_buffer->rawBuf = buf_paddr[0];
		new_raw_buffer->bufFlag = reserve_flag;
	} else { // view video
		cmd->cmd_hdr.hdr_info.cmd_type_minor =
			MINOR_ISP_NEW_VIEW_FRAME_BUF;
		new_view_buffer =
			(isp_new_frame_buf_t *)(&cmd->user_cmd_data[0]);

		for (i = 0; i < ISP_CHANNEL_VIEW_NUM; i++) {
			new_view_buffer->viewId[i] = video->current_views_id[i];
			if (new_view_buffer->viewId[i])
				new_view_buffer->viewbuf[i] = buf_paddr[i];
		}
	}

	/*************************************************
	 *step2 : fill ipc message
	 *************************************************/
	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = isp_cmd_pa(video->isp, cmd);
	ret = send_cmd_to_fw(video->isp, &msg);
	if (ret < 0) {
		dev_err(video->isp->dev,
			"%s: video %02d, send_cmd_to_fw error, ret = %d\n",
			__func__, video->video_index, ret);
	} else {
		++video->tx_buf_count;
		dev_dbg(video->isp->dev,
			"SENDBUF: video %02d, 0x%08X, 0x%08X, 0x%08X, cmd_index: %d\n",
			video->video_index, buf_paddr[0], buf_paddr[1],
			buf_paddr[2], cmd_index);
	}

	return ret;
}

static int isp_video_stream_on_subdevs(struct c1200_isp_video *video,
				       bool enable, int port)
{
	/*
	 * stream_ctrl 32bit
	 * 0~3 bit for enable 1 or 0
	 * 4~7 bit for port 0~11
	 */
	int stream_ctrl = enable;
	struct bst_isp_channel *temp_channel;
	struct camera_dev *temp_cam_dev;
	struct deser_hub_dev *temp_deser_parent;
	const struct v4l2_subdev_ops *temp_ops;
	int stream_status;

	stream_ctrl |= port << 4;
	temp_channel = video->channel;
	if (temp_channel == NULL)
		return 1;

	stream_status = atomic_read(&(temp_channel->is_streaming));
	if (stream_status) {
		dev_dbg(video->isp->dev, "video %02d stream status = %d\n",
			video->video_index, stream_status);
		return 0;
	}

	temp_cam_dev = temp_channel->cam_dev;
	if (temp_cam_dev == NULL)
		return 1;

	temp_deser_parent = temp_cam_dev->deser_parent;
	if (temp_deser_parent == NULL)
		return 1;

	temp_ops = temp_deser_parent->subdev.ops;
	if (temp_ops != NULL) {
		temp_ops->video->s_stream(&temp_deser_parent->subdev,
					  stream_ctrl);
	} else {
		dev_err(video->isp->dev, "Failed to streamon video %02d",
			video->video_index);
		return -1;
	}

	atomic_set(&temp_channel->is_streaming, 1);
	dev_info(video->isp->dev, "V%02d stream on sub devices done\n",
		 video->video_index);

	return 0;
}

int raw_video_start_stream(struct c1200_isp_video *video)
{
	ipc_msg msg;
	struct media_command *cmd;
	isp_raw_open_t *raw_streamon_cmd;
	int ret;

	cmd = isp_get_media_cmd(video->isp);
	memset(cmd, 0, sizeof(*cmd));
	cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_RAW_OPEN;
	raw_streamon_cmd = (isp_raw_open_t *)&(cmd->user_cmd_data[0]);
	raw_streamon_cmd->sensorId = S00RawId + video->chn_index;
	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = isp_cmd_pa(video->isp, cmd);
	ret = send_cmd_to_fw(video->isp, &msg);

	return ret;
}

static int video_streamon(struct c1200_isp_video *video)
{
	int i;
	int ret;

	if (video->isp->data_mode == DATA_MODE_FEED_DRIVER) {
		isp_inc_streamon_count(video->isp);
		return 0;
	}

	if (video->is_raw_video) { // raw video
		// send_video_buf_to_fw(video, &video->reserve_buf, 0, 1);
		for (i = 0; i < IPC_RETRY_TIMES; ++i) {
			ret = raw_video_start_stream(video);
			if (ret == 0)
				break;
		}
		if (ret < 0)
			dev_err(&video->video.dev,
				"%s: raw_video_start_stream error, ret = %d\n",
				__func__, ret);
	} else { // view video
		for (i = 0; i < IPC_RETRY_TIMES; ++i) {
			ret = notify_fw_start_stream(video);
			if (ret == 0)
				break;
		}
		if (ret < 0)
			dev_err(&video->video.dev,
				"%s: notify_fw_start_stream error, ret = %d\n",
				__func__, ret);
	}

	isp_inc_streamon_count(video->isp);

	return 0;
}

static int isp_video_start_streaming(struct vb2_queue *queue,
				     unsigned int count)
{
	struct c1200_isp_video_fh *vfh = vb2_get_drv_priv(queue);
	struct c1200_isp_video *video = vfh->video;
	struct isp_buffer *buf;
	int ret = 0;

	video->tx_buf_count = 0;
	video->rx_buf_count = 0;
	video->tx_drop_count = 0;
	video->rx_reserved_count = 0;
	video->first_buf_received = false;
	video->last_timestamp = 0;
	video->last_good_sequence = 0;
	video->total_bad_frames = 0;
	memset(&video->fw_ab_info, 0, sizeof(video->fw_ab_info));
	buf = NULL;

	video_streamon(video);

	if (video->isp->data_mode == DATA_MODE_FEED_DRIVER)
		dev_info(video->isp->dev,
			 "[ISP_FEED]: %s, skip stream_on_subdevs", __func__);
	else
		isp_video_stream_on_subdevs(video, true, video->video_index);

	return ret;
}

int notify_fw_cam_plugout(struct c1200_isp_video *video)
{
	ipc_msg msg;
	struct media_command *cmd;
	isp_cam_pluginout_t *plugoff_cmd;
	int ret;

	dev_info(video->isp->dev, "send sensorIndex[%d] plugout info to FW",
		 video->chn_index);
	cmd = isp_get_media_cmd(video->isp);
	memset(cmd, 0, sizeof(*cmd));
	cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_CAM_PLUGOUT;
	plugoff_cmd = (isp_cam_pluginout_t *)&(cmd->user_cmd_data[0]);
	plugoff_cmd->sensorIndex =
		video->channel->cam_dev->isp_data.sensorIndex;
	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = isp_cmd_pa(video->isp, cmd);
	ret = send_cmd_to_fw(video->isp, &msg);
	if (ret < 0)
		dev_err(&video->video.dev,
			"%s: send_cmd_to_fw error, ret = %d\n", __func__, ret);

	notify_fw_stop_stream(video);
	return ret;
}

int notify_fw_cam_plugin(struct c1200_isp_video *video)
{
	ipc_msg msg;
	struct media_command *cmd;
	isp_cam_pluginout_t *plugin_cmd;
	int ret;

	dev_info(video->isp->dev, "send sensorIndex[%d] plugin info to FW",
		 video->chn_index);
	cmd = isp_get_media_cmd(video->isp);
	memset(cmd, 0, sizeof(*cmd));
	cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_CAM_PLUGIN;
	plugin_cmd = (isp_cam_pluginout_t *)&(cmd->user_cmd_data[0]);
	plugin_cmd->sensorIndex = video->chn_index;
	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = isp_cmd_pa(video->isp, cmd);
	ret = send_cmd_to_fw(video->isp, &msg);
	if (ret < 0)
		dev_err(&video->video.dev,
			"%s: send_cmd_to_fw error, ret = %d\n", __func__, ret);

	notify_fw_start_stream(video);
	return ret;
}

int notify_fw_start_stream(struct c1200_isp_video *video)
{
	ipc_msg msg;
	struct media_command *cmd;
	isp_cam_open_t *streamon_cmd;
	unsigned long timeout;
	uint32_t *pview_id;
	uint32_t *pcurrent_id;
	int ret;

	// set to 3 for disabled embedded data
	int embedded_view_id = 3;
	int i = 0;

	cmd = isp_get_media_cmd(video->isp);
	memset(cmd, 0, sizeof(*cmd));
	cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_CAM_OPEN;
	streamon_cmd = (isp_cam_open_t *)&(cmd->user_cmd_data[0]);

	pview_id = (uint32_t *)streamon_cmd->viewId;
	pcurrent_id = (uint32_t *)video->current_views_id;
	*pview_id = *pcurrent_id;

	for (i = 0; i < ISP_CHANNEL_VIEW_NUM; i++) {
		if (video->views[i].enable) {
			if (video->current_views_id[i]) {
				embedded_view_id = min(embedded_view_id, i);
				if (i == 1) {
					embedded_view_id = i;
					break;
				}
			}
		}
	}

	streamon_cmd->embedded_view = embedded_view_id;
	video->channel->cam_dev->emd_view_info.embedded_view = embedded_view_id;
	for (i = 0; i < MAX_EMBEDDED_ZONE_NUM; i++)
		streamon_cmd->embedded_offset[i] =
			video->channel->cam_dev->emd_view_info.isp_embed_info
				.emd_zone_offset[embedded_view_id][i];

	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = isp_cmd_pa(video->isp, cmd);
	reinit_completion(&(video->stream_comp));
	ret = send_cmd_to_fw(video->isp, &msg);
	if (ret < 0) {
		dev_err(&video->video.dev,
			"%s: send_cmd_to_fw error, ret = %d\n", __func__, ret);
		return ret;
	}
	timeout = wait_for_completion_io_timeout(
		&(video->stream_comp),
		msecs_to_jiffies(ISP_STREAM_CMD_TIMEOUT));
	if (timeout == 0) {
		dev_err(&video->video.dev, "%s: timeout to wait stream start\n",
			__func__);
		return -EIO;
	}

	return ret;
}

int notify_fw_stop_stream(struct c1200_isp_video *video)
{
	ipc_msg msg;
	struct media_command *cmd;
	isp_cam_close_t *streamoff_cmd;
	unsigned long timeout;
	uint32_t *pview_id;
	uint32_t *pcurrent_id;
	int ret;

	cmd = isp_get_media_cmd(video->isp);
	memset(cmd, 0, sizeof(*cmd));
	cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_CAM_CLOSE;
	streamoff_cmd = (isp_cam_close_t *)&(cmd->user_cmd_data[0]);

	pview_id = (uint32_t *)streamoff_cmd->viewId;
	pcurrent_id = (uint32_t *)video->current_views_id;
	*pview_id = *pcurrent_id;

	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = isp_cmd_pa(video->isp, cmd);
	reinit_completion(&(video->stream_comp));
	ret = send_cmd_to_fw(video->isp, &msg);
	if (ret < 0) {
		dev_err(&video->video.dev,
			"%s: send_cmd_to_fw error, ret = %d\n", __func__, ret);
		return ret;
	}

	timeout = wait_for_completion_io_timeout(
		&(video->stream_comp),
		msecs_to_jiffies(ISP_STREAM_CMD_TIMEOUT));
	if (timeout == 0) {
		dev_err(&video->video.dev, "%s: timeout to wait stream stop\n",
			__func__);
		return -EIO;
	}

	return ret;
}

int raw_video_stop_stream(struct c1200_isp_video *video)
{
	ipc_msg msg;
	struct media_command *cmd;
	isp_raw_close_t *streamoff_cmd;
	int ret;

	cmd = isp_get_media_cmd(video->isp);
	memset(cmd, 0, sizeof(*cmd));
	cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_RAW_CLOSE;
	streamoff_cmd = (isp_raw_close_t *)&(cmd->user_cmd_data[0]);
	streamoff_cmd->sensorId = S00RawId + video->chn_index;

	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.data = isp_cmd_pa(video->isp, cmd);
	ret = send_cmd_to_fw(video->isp, &msg);
	if (ret < 0)
		pr_err("%s: send_cmd_to_fw error, ret = %d\n", __func__, ret);

	return ret;
}

/*
 *this func will be called back after vb2 framework finish off
 *step 1: set video status to VIDEO_STATUS_STREAMOFFING
 *step 2:give all buffer to vb2 done list,which have done by isp_video_streamoff
 *step 3:free video->reserve_buf alloced when streamon
 *step 4:maybe should do close disable irq ,which should cooperation with IPC
 *team step 5:sleep before FW ack
 */
static void isp_video_stop_streaming(struct vb2_queue *queue)
{
	struct c1200_isp_video_fh *vfh = vb2_get_drv_priv(queue);
	struct c1200_isp_video *video = vfh->video;
	int ret;
	int i;

	pr_err("%s, tx_count = %lld, rx_count = %lld, tx_drop = %lld, rx_reserved = %lld ipc_rx: %lld\n",
	       __func__, video->tx_buf_count, video->rx_buf_count,
	       video->tx_drop_count, video->rx_reserved_count,
	       video->isp->ipc_rx_count);
	pr_err("isp kthread status = %d\n", video->isp->kthread_status);

	if (video->isp->data_mode == DATA_MODE_FEED_DRIVER) {
		// dev_info(video->isp->dev, "[ISP_FEED]: skip cam close to
		// fw");
		isp_dec_streamon_count(video->isp);
		return;
	}

	if (video->is_raw_video) {
		ret = raw_video_stop_stream(video);
		if (ret < 0)
			pr_err("raw_video_stop_stream error, ret = %d\n", ret);
	} else {
		// notify fw to stop at first
		for (i = 0; i < IPC_RETRY_TIMES; ++i) {
			ret = notify_fw_stop_stream(video);
			if (ret == 0)
				break;
		}
		if (ret < 0)
			dev_err(&video->video.dev,
				"notify_fw_stop_stream error, ret = %d\n", ret);
	}
	isp_dec_streamon_count(video->isp);
}

static const struct vb2_ops isp_video_queue_ops = {
	.queue_setup = isp_video_queue_setup,
	//.wait_prepare = isp_video_wait_prepare,
	//.wait_finish = isp_video_wait_finish,
	//.buf_init = isp_video_buf_init,
	.buf_prepare = isp_video_buffer_prepare,
	.buf_finish = isp_video_buffer_finish,
	//.buf_cleanup = isp_video_buffer_cleanup,
	.start_streaming = isp_video_start_streaming,
	.stop_streaming = isp_video_stop_streaming,
	.buf_queue = isp_video_buffer_queue,
};

/*
 * c1200_isp_video_cancel_stream - Cancel stream on a video node
 * @video: ISP video object
 *
 * Cancelling a stream returns all buffers queued on the video node to videobuf2
 * in the erroneous state and makes sure no new buffer can be queued.
 */
void c1200_isp_video_cancel_stream(struct c1200_isp_video *video)
{
	c1200_isp_video_return_buffers(video, VB2_BUF_STATE_ERROR);
	video->error = true;
}

/* -----------------------------------------------------------------------------
 * V4L2 ioctls
 */

static int isp_video_querycap(struct file *file, void *fh,
			      struct v4l2_capability *cap)
{
	struct c1200_isp_video *video = video_drvdata(file);

	strscpy(cap->driver, ISP_VIDEO_DRIVER_NAME, sizeof(cap->driver));
	strscpy(cap->card, video->video.name, sizeof(cap->card));
	strscpy(cap->bus_info, "media", sizeof(cap->bus_info));

	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE |
			    V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING |
			    V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int isp_video_enum_input(struct file *file, void *fh,
				struct v4l2_input *input)
{
	struct c1200_isp_video *video = video_drvdata(file);
	int i;

#ifdef V4L2_COMPAT
	if (input->index > 0)
		return -EINVAL;
	strscpy(input->name, "camera", sizeof(input->name));
	input->type = V4L2_INPUT_TYPE_CAMERA;
#else
	if (video->is_pdns) {
		i = video->pdns_input_view;
		input->index = video->views[i].enable ? ISP_VIEW_MASK(i) : 0;
		return 0;
	}

	input->index = 0;
	for (i = 0; i < ISP_CHANNEL_VIEW_NUM; i++) {
		if (video->pdns_input_view == i)
			continue;
		input->index |= video->views[i].enable ? ISP_VIEW_MASK(i) : 0;
	}
#endif

	return 0;
}

static int isp_video_g_input(struct file *file, void *fh, unsigned int *views)
{
#ifdef V4L2_COMPAT
	*views = 0;
	return 0;
#else
	struct c1200_isp_video *video = video_drvdata(file);
	int i;

	if (video->is_pdns) {
		i = video->pdns_input_view;
		*views = video->current_views_id[i] ? ISP_VIEW_MASK(i) : 0;
		return 0;
	}

	*views = 0;
	for (i = 0; i < ISP_CHANNEL_VIEW_NUM; i++) {
		if (video->pdns_input_view != i) {
			*views |= video->current_views_id[i] ?
					  ISP_VIEW_MASK(i) :
					  0;
		}
	}

	return 0;
#endif
}

static int isp_video_s_input(struct file *file, void *fh, unsigned int comb)
{
#ifdef V4L2_COMPAT
	return comb == 0 ? 0 : -EINVAL;
#else
	struct c1200_isp_video_fh *vfh;
	struct c1200_isp_video *video;
	int i;

	vfh = to_c1200_isp_video_fh(fh);
	video = video_drvdata(file);
	if (comb > ISP_VIEW0_VIEW1_VIEW2) {
		pr_err("set input: invalid view option\n");
		return -EINVAL;
	}

	video->current_views_num = 0;
	// pdns case
	if (video->is_pdns) {
		// valid check
		i = video->pdns_input_view;
		if (comb != ISP_VIEW_MASK(i)) {
			pr_err("set input: invalid view %d selection for pdns\n",
			       i);
			return -EINVAL;
		}
		if (!video->views[i].enable) {
			pr_err("set input: seletced view %d not available\n",
			       i);
			return -EINVAL;
		}
		// view setting
		video->current_views_id[i] = video->views[i].cmd_view_id;
		video->current_views_list[video->current_views_num] = i;
		video->current_views_num++;
		goto done;
	}

	// normal case
	// valid check
	for (i = 0; i < ISP_CHANNEL_VIEW_NUM; i++) {
		if (comb & ISP_VIEW_MASK(i)) {
			if (i == video->pdns_input_view) {
				pr_err("set input: seletced view %d occupied by pdns\n",
				       i);
				return -EINVAL;
			}
			if (!video->views[i].enable) {
				pr_err("set input: seletced view %d not available\n",
				       i);
				return -EINVAL;
			}
		}
	}
	// view setting
	for (i = 0; i < ISP_CHANNEL_VIEW_NUM; i++) {
		if (comb & ISP_VIEW_MASK(i)) {
			video->current_views_id[i] =
				video->views[i].cmd_view_id;
			video->current_views_list[video->current_views_num] = i;
			video->current_views_num++;
		} else {
			video->current_views_id[i] = 0;
		}
	}

done:
	mutex_lock(&video->mutex);
	vfh->format.fmt.pix_mp.num_planes = video->current_views_num;
	mutex_unlock(&video->mutex);
#endif
	return 0;
}

static int isp_video_get_format_mplane(struct file *file, void *fh,
				       struct v4l2_format *format)
{
	struct c1200_isp_video *video = video_drvdata(file);
	struct isp_view_format *f;
	int i;

	if (format->type != video->type) {
		pr_err("get format: invalid, format typeset %d video typeset %d\n",
		       format->type, video->type);
		return -EINVAL;
	}

	if (video->is_raw_video) {
		f = (struct isp_view_format *)&format->fmt.pix_mp.plane_fmt[0];
		f->pixelformat = video->views[0].format;
		f->width = video->channel->camera_raw_width;
		f->height = video->views[0].height;
		pr_debug("get format: view %d width %d height %d\n", 0,
			 f->width, f->height);
		return 0;
	}

	for (i = 0; i < ISP_CHANNEL_VIEW_NUM; i++) {
		if ((video->pdns_input_view != i) && video->views[i].enable) {
			f = (struct isp_view_format *)&format->fmt.pix_mp
				    .plane_fmt[i];
			f->pixelformat = video->views[i].format;
			f->width = video->views[i].width;
			f->height = video->views[i].height;
			pr_debug("get format: view %d width %d height %d\n", i,
				 f->width, f->height);
		}
	}

	return 0;
}

static int isp_video_set_format_mplane(struct file *file, void *fh,
				       struct v4l2_format *format)
{
	struct c1200_isp_video_fh *vfh;
	struct c1200_isp_video *video;
	int index;
	uint32_t linebytes;
	uint32_t imagesize;

	pr_debug("set format: view %d\n", format->fmt.pix_mp.num_planes);
	vfh = to_c1200_isp_video_fh(fh);
	video = video_drvdata(file);
	if (format->type != video->type) {
		pr_err("get format: invalid, format typeset %d video typeset %d\n",
		       format->type, video->type);
		return -EINVAL;
	}

	// interpret num_planes as view index
	index = format->fmt.pix_mp.num_planes;
	if (index < 0 || index >= MAX_VIEWS_PER_CAMERA) {
		pr_err("set format: invalid plane index %d\n", index);
		return -EINVAL;
	}

	if (!video->views[index].enable) {
		pr_err("set format: disabled plane index %d\n", index);
		return -EINVAL;
	}

	if ((format->fmt.pix_mp.width != video->views[index].width) ||
	    (format->fmt.pix_mp.height != video->views[index].height) ||
	    (format->fmt.pix_mp.pixelformat != video->views[index].format)) {
		pr_err("set format: width, height or pixelformat not supported\n");
		return -EINVAL;
	}
	linebytes = get_view_bytesperline(&(video->views[index]));
	imagesize = get_view_sizeimage(&(video->views[index]));
	if (!imagesize) {
		pr_err("set format: plane %d, sizeimage zero\n", index);
		return -EINVAL;
	}

	mutex_lock(&video->mutex);
	vfh->format.fmt.pix_mp.plane_fmt[index].bytesperline = linebytes;
	vfh->format.fmt.pix_mp.plane_fmt[index].sizeimage = imagesize;

	mutex_unlock(&video->mutex);

	return 0;
}

static int isp_video_try_format_mplane(struct file *file, void *fh,
				       struct v4l2_format *format)
{
	pr_info("ENTER: %s not implemented\n", __func__);

	return 0;
}

static int isp_video_try_format(struct file *file, void *fh,
				struct v4l2_format *format)
{
	pr_info("ENTER: %s not implemented\n", __func__);

	return 0;
}

static int isp_video_get_format(struct file *file, void *fh,
				struct v4l2_format *format)
{
	int i;
	struct c1200_isp_video *video;

	video = video_drvdata(file);
	mutex_lock(&video->mutex);
	for (i = 0; i < ARRAY_SIZE(format->fmt.pix_mv); ++i) {
		if (i >= MAX_VIEWS_PER_CAMERA) {
			memset(&format->fmt.pix_mv[i], 0,
			       sizeof(format->fmt.pix_mv[i]));
			dev_dbg(video->isp->dev,
				"video %02d: view %u out of range\n",
				video->video_index, i);
			continue;
		}
		if (!video->views[i].enable) {
			memset(&format->fmt.pix_mv[i], 0,
			       sizeof(format->fmt.pix_mv[i]));
			dev_dbg(video->isp->dev,
				"video %02d: view %u is disabled\n",
				video->video_index, i);
			continue;
		}

		dev_err(video->isp->dev,
			"video %02d: view %u: w: %u, h: %u, f: %u, id: %u\n",
			video->video_index, i, video->views[i].width,
			video->views[i].height, video->views[i].format,
			video->current_views_id[i]);

		format->fmt.pix_mv[i].width = video->views[i].width;
		format->fmt.pix_mv[i].height = video->views[i].height;
		format->fmt.pix_mv[i].pixelformat = video->views[i].format;
		format->fmt.pix_mv[i].bytesperline =
			get_view_bytesperline(&(video->views[i]));
		format->fmt.pix_mv[i].sizeimage =
			PAGE_ALIGN(get_view_sizeimage(&(video->views[i])) +
				   get_extend_size(video));
		if (video->current_views_id[i] != 0)
			format->fmt.pix_mv[i].flags |=
				V4L2_PIX_FMT_FLAG_VIEW_OPENED;
	}
	mutex_unlock(&video->mutex);

	return 0;
}

static int isp_video_set_format(struct file *file, void *fh,
				struct v4l2_format *format)
{
	int i;
	struct c1200_isp_video_fh *vfh;
	struct c1200_isp_video *video;

	video = video_drvdata(file);
	vfh = to_c1200_isp_video_fh(fh);
	mutex_lock(&video->mutex);
	video->current_views_num = 0;
	/* 2024-02-20, Oak Chen:
	 * Only permit limited set, that is:
	 * 1. Enable/Disable the view
	 * Then, we override the format with actual set following definition of
	 * VIDIOC_S_FMT.
	 */
	for (i = 0; i < ARRAY_SIZE(format->fmt.pix_mv); ++i) {
		uint32_t linebytes;
		uint32_t imagesize;
		__u32 pixelformat;

		if (i >= MAX_VIEWS_PER_CAMERA) {
			memset(&format->fmt.pix_mv[i], 0,
			       sizeof(format->fmt.pix_mv[i]));
			dev_dbg(video->isp->dev,
				"video %02d: view %d out of range\n",
				video->video_index, i);
			continue;
		}
		pixelformat = format->fmt.pix_mv[i].pixelformat;
		if (!video->views[i].enable) {
			if (pixelformat != 0) {
				mutex_unlock(&video->mutex);
				return -EINVAL;
			}
			video->current_views_id[i] = 0;
			memset(&format->fmt.pix_mv[i], 0,
			       sizeof(format->fmt.pix_mv[i]));
			dev_dbg(video->isp->dev,
				"video %02d: view %d is disabled\n",
				video->video_index, i);
			continue;
		}
		format->fmt.pix_mv[i].width = video->views[i].width;
		format->fmt.pix_mv[i].height = video->views[i].height;
		format->fmt.pix_mv[i].pixelformat = video->views[i].format;
		linebytes = get_view_bytesperline(&(video->views[i]));
		imagesize = get_view_sizeimage(&(video->views[i]));
		format->fmt.pix_mv[i].bytesperline = linebytes;
		format->fmt.pix_mv[i].sizeimage =
			PAGE_ALIGN(imagesize + get_extend_size(video));
		/* Disable view when pixelformat is 0 */
		if (pixelformat == 0) {
			video->current_views_id[i] = 0;
			continue;
		}
		format->fmt.pix_mv[i].flags |= V4L2_PIX_FMT_FLAG_VIEW_OPENED;
		vfh->format.fmt.pix_mp.plane_fmt[i].sizeimage = imagesize;
		vfh->format.fmt.pix_mp.plane_fmt[i].bytesperline = linebytes;
		video->current_views_id[i] = video->views[i].cmd_view_id;
		video->current_views_list[video->current_views_num] = i;
		++video->current_views_num;
	}
	vfh->format.fmt.pix_mp.num_planes = video->current_views_num;

	mutex_unlock(&video->mutex);

	return 0;
}

static int isp_video_get_selection(struct file *file, void *fh,
				   struct v4l2_selection *sel)
{
	return 0;
}

static int isp_video_set_selection(struct file *file, void *fh,
				   struct v4l2_selection *sel)
{
	return 0;
}

static int isp_video_get_param(struct file *file, void *fh,
			       struct v4l2_streamparm *a)
{
	return 0;
}

static int isp_video_set_param(struct file *file, void *fh,
			       struct v4l2_streamparm *a)
{
	return 0;
}

static int isp_video_reqbufs(struct file *file, void *fh,
			     struct v4l2_requestbuffers *rb)
{
	struct c1200_isp_video_fh *vfh = to_c1200_isp_video_fh(fh);
	struct c1200_isp_video *video = video_drvdata(file);
	int ret;
	enum v4l2_buf_type type;

	dev_dbg(video->isp->dev,
		"V%02d reqbufs: type: %d, count: %d, memory: %d\n",
		video->video_index, rb->type, rb->count, rb->memory);
	type = rb->type;
	rb->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	mutex_lock(&video->queue_lock);
	ret = vb2_reqbufs(&vfh->queue, rb);
	mutex_unlock(&video->queue_lock);
	rb->type = type;

	return ret;
}

static int isp_video_querybuf_overlay(struct c1200_isp_video *video,
				      struct vb2_queue *q,
				      struct v4l2_buffer *b)
{
	int ret;
	int v;
	int p;
	struct v4l2_buffer *views;
	enum v4l2_buf_type type;
	__u32 length;
	struct v4l2_plane planes[MAX_VIEWS_PER_CAMERA];

	type = b->type;
	length = b->length;
	views = b->m.views;
	b->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	b->m.planes = planes;
	b->length = ARRAY_SIZE(planes);

	memset(planes, 0, sizeof(planes));
	ret = vb2_querybuf(q, b);
	b->type = type;
	b->length = length;
	b->m.views = views;

	for (v = 0, p = 0; v < b->length; ++v) {
		dev_dbg(video->isp->dev,
			"V%02d querybuf: p: %u, length: %10u, offset: 0x%08X\n",
			video->video_index, p, planes[p].length,
			planes[p].m.mem_offset);
		put_user(b->memory, &(views[v].memory));
		put_user(b->flags, &(views[v].flags));
		put_user(planes[p].length, &(views[v].length));
		put_user(planes[p].m.mem_offset, &(views[v].m.offset));
		++p;
	}

	return ret;
}

static int isp_video_querybuf(struct file *file, void *fh,
			      struct v4l2_buffer *b)
{
	struct c1200_isp_video_fh *vfh = to_c1200_isp_video_fh(fh);
	struct c1200_isp_video *video = video_drvdata(file);
	int ret;

	dev_dbg(video->isp->dev,
		"V%02d querybuf: type: %d, index: %d, length: %d, mem: %d\n",
		video->video_index, b->type, b->index, b->length, b->memory);
	mutex_lock(&video->queue_lock);
	if (b->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		ret = vb2_querybuf(&vfh->queue, b);
	else
		ret = isp_video_querybuf_overlay(video, &vfh->queue, b);
	mutex_unlock(&video->queue_lock);

	return ret;
}

static int isp_video_qbuf_overlay(struct c1200_isp_video *video,
				  struct vb2_queue *q,
				  struct media_device *mdev,
				  struct v4l2_buffer *b)
{
	int ret;
	int v;
	int p;
	enum v4l2_buf_type type;
	__u32 length;
	struct v4l2_buffer *views;
	struct v4l2_plane planes[MAX_VIEWS_PER_CAMERA];

	type = b->type;
	length = b->length;
	views = b->m.views;
	b->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

	memset(planes, 0, sizeof(planes));
	for (v = 0, p = 0; v < length; ++v) {
		if (v >= ARRAY_SIZE(planes))
			break;
		get_user(planes[p].m.fd, &b->m.views[v].m.fd);
		++p;
	}
	b->m.planes = planes;
	b->length = p;

	ret = vb2_qbuf(q, mdev, b);
	b->type = type;
	b->length = length;
	b->m.views = views;

	return ret;
}

static int isp_video_qbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct c1200_isp_video_fh *vfh = to_c1200_isp_video_fh(fh);
	struct c1200_isp_video *video = video_drvdata(file);
	int ret;

	dev_dbg(video->isp->dev,
		"V%02d qbuf: type: %d, index: %u, memory: %d, length: %u\n",
		video->video_index, b->type, b->index, b->memory, b->length);
	mutex_lock(&video->queue_lock);
	if (b->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		ret = vb2_qbuf(&vfh->queue, &video->isp->media_dev, b);
	else
		ret = isp_video_qbuf_overlay(video, &vfh->queue,
					     &video->isp->media_dev, b);
	mutex_unlock(&video->queue_lock);

	return ret;
}

static int isp_video_dqbuf_overlay(struct c1200_isp_video *video,
				   struct vb2_queue *q, struct v4l2_buffer *b,
				   bool nonblocking)
{
	int ret;
	int v;
	int p;
	enum v4l2_buf_type type;
	__u32 length;
	struct v4l2_buffer *views;
	struct v4l2_plane planes[MAX_VIEWS_PER_CAMERA];

	type = b->type;
	length = b->length;
	views = b->m.views;

	b->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	b->length = ARRAY_SIZE(planes);
	b->m.planes = planes;
	memset(planes, 0, sizeof(planes));

	ret = vb2_dqbuf(q, b, nonblocking);
	b->type = type;
	b->length = length;
	b->m.views = views;

	dev_dbg(video->isp->dev, "V%02d dqbuf: resp index: %u\n",
		video->video_index, b->index);
	for (v = 0, p = 0; v < length; ++v) {
		ret = copy_to_user(views + v, b, sizeof(*views));
		if (ret)
			return ret;
		put_user(planes[p].bytesused, &views[v].bytesused);
		put_user(planes[p].length, &views[v].length);
		++p;
	}

	return ret;
}

static int isp_video_dqbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct c1200_isp_video_fh *vfh = to_c1200_isp_video_fh(fh);
	struct c1200_isp_video *video = video_drvdata(file);
	int ret;
	int row_time;
	int sensor_height;

	if (video->channel->cam_dev == NULL)
		return -1;

	sensor_height = video->channel->cam_dev->isp_data.rawinfo.height / 2;
	row_time = video->channel->cam_dev->row_time;
	mutex_lock(&video->queue_lock);
	if (video->error) {
		b->flags = V4L2_BUF_FLAG_ERROR;
		b->sequence = video->last_good_sequence;
		v4l2_buffer_set_timestamp(b, video->last_timestamp);
		ret = -EFAULT;
	} else {
		if (b->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
			ret = vb2_dqbuf(&vfh->queue, b,
					file->f_flags & O_NONBLOCK);
		else
			ret = isp_video_dqbuf_overlay(video, &vfh->queue, b,
						      file->f_flags &
							      O_NONBLOCK);

		if (video->isp->data_mode == DATA_MODE_FEED_DRIVER)
			b->reserved = 0;
		else
			b->reserved = ((sensor_height * row_time) +
				       video->sensor_exp) /
				      1000; // change to us
	}
	mutex_unlock(&video->queue_lock);
	dev_dbg(video->isp->dev, "%s: video %02d, buffer: %d\n", __func__,
		video->video_index, b->index);

	return ret;
}

static int isp_video_streamon(struct file *file, void *fh,
			      enum v4l2_buf_type type)
{
	struct c1200_isp_video_fh *vfh = to_c1200_isp_video_fh(fh);
	struct c1200_isp_video *video = video_drvdata(file);
	int ret;

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	if (type != video->type)
		return -EINVAL;

	mutex_lock(&video->isp->isp_mutex);
	atomic_set(&(video->status), VIDEO_STATUS_STREAMONING);

	video->queue = &vfh->queue;
	INIT_LIST_HEAD(&video->free_buf_queue);

	mutex_lock(&video->queue_lock);
	ret = vb2_streamon(&vfh->queue, type);
	mutex_unlock(&video->queue_lock);
	if (ret < 0)
		goto err_check_format;

	atomic_set(&(video->status), VIDEO_STATUS_STREAMON_DONE);
	mutex_unlock(&video->isp->isp_mutex);
	return 0;

err_check_format:
	INIT_LIST_HEAD(&video->free_buf_queue);
	video->queue = NULL;
	mutex_unlock(&video->isp->isp_mutex);
	return ret;
}

static int isp_video_streamoff(struct file *file, void *fh,
			       enum v4l2_buf_type type)
{
	struct c1200_isp_video_fh *vfh = to_c1200_isp_video_fh(fh);
	struct c1200_isp_video *video = video_drvdata(file);
	unsigned int streaming;

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	if (type != video->type)
		return -EINVAL;

	/* Make sure we're not streaming yet. */
	mutex_lock(&video->queue_lock);
	streaming = vb2_is_streaming(&vfh->queue);
	mutex_unlock(&video->queue_lock);

	if (!streaming)
		goto done;

	c1200_isp_video_cancel_stream(video);

	atomic_set(&(video->status), VIDEO_STATUS_STREAMOFFING);

	mutex_lock(&video->queue_lock);
	vb2_streamoff(&vfh->queue, type);
	mutex_unlock(&video->queue_lock);
	video->queue = NULL;
	video->error = false;

done:
	atomic_set(&(video->status), VIDEO_STATUS_STREAMOFF_DONE);
	dev_dbg(video->isp->dev, "V%02d stream off\n", video->video_index);
	return 0;
}

static int isp_video_expbuf_overlay(struct c1200_isp_video *video,
				    struct vb2_queue *q,
				    struct v4l2_exportbuffer *p)
{
	int ret;
	enum v4l2_buf_type type;

	type = p->type;
	p->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	ret = vb2_expbuf(q, p);
	p->type = type;

	return ret;
}

static int isp_video_expbuf(struct file *file, void *fh,
			    struct v4l2_exportbuffer *p)
{
	struct c1200_isp_video_fh *vfh = to_c1200_isp_video_fh(fh);
	struct c1200_isp_video *video = video_drvdata(file);
	int ret;

	dev_dbg(video->isp->dev,
		"V%02d expbuf: type: %u, index: %u, view: %u\n",
		video->video_index, p->type, p->index, p->view);
	mutex_lock(&video->queue_lock);
	if (p->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		ret = vb2_expbuf(&vfh->queue, p);
	else
		ret = isp_video_expbuf_overlay(video, &vfh->queue, p);
	mutex_unlock(&video->queue_lock);

	return ret;
}

static int isp_get_camera_info(struct c1200_isp_video *video,
			       struct camera_info_t_user *pcamer_info)
{
	pcamer_info->camera_data_type =
		video->channel->cam_dev->isp_data.rawinfo.dataType;
	pcamer_info->camera_fps = video->channel->cam_dev->sensor_fps;
	pcamer_info->camera_raw_width =
		video->channel->cam_dev->isp_data.rawinfo.width;
	pcamer_info->camera_raw_height =
		video->channel->cam_dev->isp_data.rawinfo.height;

	return 0;
}

static int isp_get_embed_info(struct c1200_isp_video *video,
			      struct isp_emd_view_info *pembed_view_info)
{
	memcpy(pembed_view_info, &(video->channel->cam_dev->emd_view_info),
	       sizeof(struct isp_emd_view_info));

	return 0;
}

static long isp_video_default(struct file *file, void *fh, bool valid_prio,
			      unsigned int cmd, void *arg)
{
	struct c1200_isp_video *video;
	int *data_mode;

	switch (cmd) {
	case ISPIOC_QUERY_CAM_INFO:
		video = video_drvdata(file);
		return isp_get_camera_info(video,
					   (struct camera_info_t_user *)arg);

	case ISPIOC_QUERY_EMBED_INFO:
		video = video_drvdata(file);
		return isp_get_embed_info(video,
					  (struct isp_emd_view_info *)arg);

	case ISPIOC_G_ABNORMAL_INFO:
		{
			struct c1200_isp_video *video = video_drvdata(file);
			struct abnormal_info *ab_info =
				(struct abnormal_info *)arg;

			ab_info->abnormal_id = video->fw_ab_info.abnormalId;
			ab_info->abnormal_type = video->fw_ab_info.abnormalType;
			ab_info->last_good_sequence = video->last_good_sequence;
			ab_info->total_bad_frames = video->total_bad_frames;
			ab_info->total_frames = video->rx_buf_count;
			video->error = false;
			return 0;
		}

	case ISPIOC_G_DATA_MODE:
		video = video_drvdata(file);
		data_mode = (int *)arg;
		*data_mode = video->isp->data_mode;

		return 0;

	default:
		return -ENOTTY;
	}
}

static const struct v4l2_ioctl_ops isp_video_ioctl_ops = {
	.vidioc_querycap = isp_video_querycap,
	.vidioc_try_fmt_vid_cap_mplane = isp_video_try_format_mplane,
	.vidioc_s_fmt_vid_cap_mplane = isp_video_set_format_mplane,
	.vidioc_g_fmt_vid_cap_mplane = isp_video_get_format_mplane,
	.vidioc_try_fmt_vid_cap = isp_video_try_format,
	.vidioc_s_fmt_vid_cap = isp_video_set_format,
	.vidioc_g_fmt_vid_cap = isp_video_get_format,
	.vidioc_g_selection = isp_video_get_selection,
	.vidioc_s_selection = isp_video_set_selection,
	.vidioc_g_parm = isp_video_get_param,
	.vidioc_s_parm = isp_video_set_param,
	.vidioc_reqbufs = isp_video_reqbufs,
	.vidioc_querybuf = isp_video_querybuf,
	.vidioc_qbuf = isp_video_qbuf,
	.vidioc_dqbuf = isp_video_dqbuf,
	.vidioc_streamon = isp_video_streamon,
	.vidioc_streamoff = isp_video_streamoff,
	.vidioc_enum_input = isp_video_enum_input,
	.vidioc_g_input = isp_video_g_input,
	.vidioc_s_input = isp_video_s_input,
	.vidioc_expbuf = isp_video_expbuf,
	.vidioc_default = isp_video_default,
};

/* -----------------------------------------------------------------------------
 * V4L2 file operations
 */

static int get_view_bytesperline(struct isp_view *pview)
{
	int format = pview->format;

	switch (format) {
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
		return (pview->width);
	case V4L2_PIX_FMT_RGB24:
		return (pview->width * 3);
	case V4L2_PIX_FMT_GREY:
		return (pview->width);
	case V4L2_PIX_FMT_SBGGR12P:
		return ALIGN((pview->width * 12 / 8), ISP_LINE_ALIGIN);
	case V4L2_PIX_FMT_SBGGR14P:
		return ALIGN((pview->width * 14 / 8), ISP_LINE_ALIGIN);
	case V4L2_PIX_FMT_SBGGR16:
		return ALIGN((pview->width * 16 / 8), ISP_LINE_ALIGIN);
	default:
		pr_err("%s ERROR: unknown view format = %d\n", __func__,
		       format);
		return 0;
	}
}

static int get_view_sizeimage(struct isp_view *pview)
{
	int format = pview->format;

	switch (format) {
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
		return pview->width * pview->height * 3 / 2;
	case V4L2_PIX_FMT_RGB24:
		return pview->width * pview->height * 3;
	case V4L2_PIX_FMT_GREY:
		return pview->width * pview->height;
	case V4L2_PIX_FMT_SBGGR12P:
	case V4L2_PIX_FMT_SBGGR14P:
	case V4L2_PIX_FMT_SBGGR16:
		return get_view_bytesperline(pview) * pview->height;
	default:
		pr_err("%s ERROR: unknown view format = %d\n", __func__,
		       format);
		return 0;
	}
}

static int get_extend_size(struct c1200_isp_video *video)
{
	int i;

	uint32_t embed_size_expand;
	uint32_t embed_line_sum = 0;

	for (i = 0; i < MAX_EMBEDDED_ZONE_NUM; i++)
		embed_line_sum += video->channel->cam_dev->emd_view_info
					  .isp_embed_info.emd_line_num[i];

	embed_size_expand =
		(video->channel->camera_raw_width * embed_line_sum) +
		YUV_STAT_BUF_EXPAND + EMBED_DATA_BUF_RESERVE_SIZE;

	return embed_size_expand;
}

static int isp_video_open(struct file *file)
{
	struct c1200_isp_video *video = video_drvdata(file);
	struct c1200_isp_video_fh *handle;
	struct vb2_queue *queue;
	struct device *dev;
	int ret = 0;
	char work_queue_name[64];
	int default_view;
	int video_status;
	int video_index;
	int i;

	video_index = video->video_index;
	dev = video->isp->dev;
	dev_info(dev, "%s: video %02d Enter\n", __func__, video_index);
	mutex_lock(&(video->mutex));
	/*Judge whether camera is connected*/
	if (video->isp->data_mode == DATA_MODE_FEED_DRIVER) {
		dev_dbg(dev, "[ISP_FEED]: skip camera power_on judge\n");
	} else if (!video->channel->cam_dev->power_on) {
		dev_err(dev, "%s: video %02d disabled\n", __func__,
			video_index);
		ret = -ENOENT;
		goto err_no_dev;
	}
	video_status = atomic_read(&(video->status));
	dev_dbg(dev, "%s: video %02d status: %d\n", __func__, video_index,
		video_status);
	if ((video_status >= VIDEO_STATUS_OPENING) &&
	    (video_status < VIDEO_STATUS_CLOSE_DONE)) {
		dev_err(dev, "%s: video %02d is busy\n", __func__, video_index);
		ret = -EBUSY;
		goto err_dev_busy;
	}
	handle = devm_kzalloc(dev, sizeof(*handle), GFP_KERNEL);
	if (handle == NULL) {
		ret = -ENOMEM;
		goto err_no_mem;
	}
	/* maybe use v4l2_fh_open is better,
	 * here also ok.
	 */
	v4l2_fh_init(&handle->vfh, &video->video);
	v4l2_fh_add(&handle->vfh);

	queue = &handle->queue;
	queue->type = video->type;
	queue->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	queue->drv_priv = handle;
	queue->ops = &isp_video_queue_ops;
	queue->mem_ops = &vb2_dma_contig_memops;
	queue->dma_attrs = DMA_ATTR_FORCE_CONTIGUOUS;
	queue->buf_struct_size = sizeof(struct isp_buffer);
	queue->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	queue->dev = video->isp->dev;
	/* min_buffers_needed will be used at reqbuf and streamon of vb2
	 * framework
	 */
	queue->min_buffers_needed = MIN_FRAME_BUFFER_NUM;

	ret = vb2_queue_init(&handle->queue);
	if (ret < 0) {
		dev_err(dev,
			"%s: video %02d, vb2_queue_init failed, ret = %d\n",
			__func__, video_index, ret);
		goto err_vb2_queue_init;
	}

	isp_boot_fw_api(video->isp);

	// Check FW whether boot done
	if (atomic_read(&video->isp->FW_load_started) != 1) {
		dev_err(dev, "video %02d: isp fw not started", video_index);
		ret = -EPIPE;
		goto err_fw_boot;
	}
	default_view = -1;
	// default to first enabled view
	for (i = 0; i < MAX_VIEWS_PER_CAMERA; i++) {
		if (video->views[i].enable) {
			default_view = i;
			break;
		}
	}
	if (default_view < 0) {
		dev_err(dev, "video %02d: no view enabled\n", video_index);
		ret = -ENOENT;
		goto err_no_view_enabled;
	}
	handle->timeperframe.denominator = 1;
	handle->format.type = video->type;
	handle->format.fmt.pix_mp.num_planes = 1;
	handle->format.fmt.pix_mp.width = video->views[default_view].width;
	handle->format.fmt.pix_mp.height = video->views[default_view].height;
	handle->format.fmt.pix_mp.pixelformat =
		video->views[default_view].format;
	handle->format.fmt.pix_mp.field = V4L2_FIELD_ANY;
	handle->format.fmt.pix_mp.colorspace = V4L2_COLORSPACE_DEFAULT;

	if (video->is_raw_video) {
		handle->format.fmt.pix_mp.plane_fmt[0].bytesperline =
			video->channel->camera_raw_width * 2;
		handle->format.fmt.pix_mp.plane_fmt[0].sizeimage =
			video->channel->camera_raw_width *
			video->channel->camera_raw_height * 2;
	} else {
		handle->format.fmt.pix_mp.plane_fmt[0].bytesperline =
			get_view_bytesperline(&(video->views[default_view]));
		handle->format.fmt.pix_mp.plane_fmt[0].sizeimage =
			get_view_sizeimage(&(video->views[default_view]));
	}
	video->current_views_num = 1;
	video->current_views_list[0] = 0;
	video->current_views_list[1] = 0;
	video->current_views_list[2] = 0;
	video->current_views_id[0] = video->views[default_view].cmd_view_id;
	video->current_views_id[1] = 0;
	video->current_views_id[2] = 0;
	video->current_views_id[3] = 0;
	video->drv_sequence = 0;
	video->v4l2_handle = handle;

	handle->video = video;
	file->private_data = &handle->vfh;
	memset(work_queue_name, 0, sizeof(work_queue_name));
	snprintf(work_queue_name, sizeof(work_queue_name), "%s%d",
		 "msg_work_queue", video->video_index);

	video->msg_work_queue = create_singlethread_workqueue(work_queue_name);
	if (video->msg_work_queue == NULL) {
		dev_err(dev, "video %02d: could not create msg_work_queue\n",
			video_index);
		ret = -ENOMEM;
		goto err_create_wq;
	}

	atomic_set(&(video->status), VIDEO_STATUS_OPEN_DONE);
	mutex_unlock(&(video->mutex));

	return 0;

err_create_wq:
err_fw_boot:
err_vb2_queue_init:
	v4l2_fh_del(&handle->vfh);
	v4l2_fh_exit(&handle->vfh);
	devm_kfree(dev, handle);
err_no_mem:
err_no_view_enabled:
err_dev_busy:
err_no_dev:
	mutex_unlock(&(video->mutex));

	return ret;
}

static int isp_video_release(struct file *file)
{
	struct c1200_isp_video *video = video_drvdata(file);
	struct v4l2_fh *vfh = file->private_data;
	struct c1200_isp_video_fh *handle = to_c1200_isp_video_fh(vfh);
	int video_status;

	video_status = atomic_read(&(video->status));
	if (video_status == VIDEO_STATUS_STREAMON_DONE) {
		/* Disable streaming and free the buffers queue resources. */
		isp_video_streamoff(file, vfh, video->type);
	}

	mutex_lock(&video->msg_queue_lock);
	flush_workqueue(video->msg_work_queue);
	atomic_set(&(video->status), VIDEO_STATUS_CLOSING);
	destroy_workqueue(video->msg_work_queue);
	mutex_unlock(&video->msg_queue_lock);

	mutex_lock(&video->queue_lock);
	vb2_queue_release(&handle->queue);
	mutex_unlock(&video->queue_lock);

	/* Release the file handle. */
	v4l2_fh_del(vfh);
	v4l2_fh_exit(vfh);
	devm_kfree(video->isp->dev, handle);
	file->private_data = NULL;
	atomic_set(&(video->status), VIDEO_STATUS_CLOSE_DONE);
	return 0;
}

static __poll_t isp_video_poll(struct file *file, poll_table *wait)
{
	struct c1200_isp_video_fh *vfh =
		to_c1200_isp_video_fh(file->private_data);
	struct c1200_isp_video *video = video_drvdata(file);
	__poll_t ret;

	mutex_lock(&video->queue_lock);
	ret = vb2_poll(&vfh->queue, file, wait);
	if (video->error)
		ret = EPOLLERR | EPOLLNVAL;
	mutex_unlock(&video->queue_lock);

	return ret;
}

static int isp_video_mmap_work(struct file *file, struct vm_area_struct *vma)
{
	int ret;
	struct c1200_isp_video_fh *vfh;
	struct vb2_queue *q;

	vfh = to_c1200_isp_video_fh(file->private_data);
	q = &vfh->queue;

	dev_dbg(q->dev,
		"%s: O: vm_start: 0x%08lX, vm_pgoff: 0x%08lX, size: 0x%08lX, dma_coherent: %d\n",
		__func__, vma->vm_start, vma->vm_pgoff,
		vma->vm_end - vma->vm_start, dev_is_dma_coherent(q->dev));

	ret = vb2_mmap(&vfh->queue, vma);
	if (ret)
		dev_err(q->dev, "vb2_mmap failed");

	return ret;
}

static int isp_video_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct c1200_isp_video *video = video_drvdata(file);

	if (video->isp->state == ISPDRV_STATE_WORK)
		return isp_video_mmap_work(file, vma);

	return 0;
}

static const struct v4l2_file_operations isp_video_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = video_ioctl2,
	.open = isp_video_open,
	.release = isp_video_release,
	.poll = isp_video_poll,
	.mmap = isp_video_mmap,
};

static void ipc_msg_handler(struct work_struct *work)
{
	struct c1200_isp_video *video;
	struct internal_msg *cache_msg;

	cache_msg = container_of(work, struct internal_msg, msg_work);
	video = cache_msg->video_ptr;
	switch (cache_msg->cmd_minor) {
	case MINOR_SYNC_ISP_VIEW_FRAME_DONE:
	case MINOR_ISP_RAW_BUF_DONE:
		{
			if (VIDEO_STATUS_STREAMON_DONE ==
			    atomic_read(&video->status)) {
				video->rx_buf_count++;
				handle_video_sync_msg(video, cache_msg);
			} else {
				dev_err_ratelimited(
					video->isp->dev,
					"video %02d is not stream on status\n",
					video->video_index);
			}
			// received new frame buffer, clear error flag
			video->error = false;
			break;
		}

	case MINOR_ABNORMAL:
		{
			int i;
			int buf_matched = 0;
			struct isp_buffer *last_wait_buf;
			struct list_head *pos;
			abnormal_t *fw_ab_info =
				(abnormal_t *)&(cache_msg->user_data[0]);

			video->fw_ab_info = *fw_ab_info;
			if (VIDEO_STATUS_STREAMON_DONE !=
			    atomic_read(&video->status))
				break;

			dev_err_ratelimited(
				&video->video.dev,
				"ABNORMAL: 0x%02X 0x%02X 0x%02X 0x%02X 0x%08X 0x%08X 0x%08X\n",
				fw_ab_info->abnormalId,
				fw_ab_info->abnormalType, fw_ab_info->rsv0[0],
				fw_ab_info->rsv0[1], fw_ab_info->rsv[0],
				fw_ab_info->rsv[1], fw_ab_info->rsv[2]);
			mutex_lock(&video->wait_queue_lock);
			list_for_each(pos, &video->wait_buf_queue) {
				last_wait_buf = list_entry(
					pos, struct isp_buffer, node);
				if (last_wait_buf == NULL)
					continue;
				for (i = 0; i < ISP_CHANNEL_VIEW_NUM; i++) {
					if (video->current_views_id[i]) {
						buf_matched +=
							(fw_ab_info->rsv[i] ==
							 last_wait_buf->dma[i]) ?
								1 :
								0;
					}
				}
				if (buf_matched) {
					list_del(&last_wait_buf->node);
					list_add_tail(&last_wait_buf->node,
						      &video->free_buf_queue);
					dev_dbg(&video->video.dev,
						"abnormal_recover: add node to free queue\n");
					break;
				}
			}
			mutex_unlock(&video->wait_queue_lock);

			video->total_bad_frames++;
			video->error = true;
			wake_up(&video->queue->done_wq);
			break;
		}
	default:
		dev_err_ratelimited(
			&video->video.dev,
			"ERROR: video %02d, unknow cmd cmd_minor 0x%02X\n",
			video->video_index, cache_msg->cmd_minor);
		break;
	}
	// done, put cache msg to free cache
	cache_msg->cmd_main = 0;
	cache_msg->cmd_minor = 0;
	cache_msg->tick = 0;
	cache_msg->user_data[0] = 0;
	cache_msg->user_data[1] = 0;
	cache_msg->user_data[2] = 0;
	cache_msg->user_data[3] = 0;
	mutex_lock(&video->cache_queue_lock);
	list_add_tail(&cache_msg->node, &video->free_cache_queue);
	mutex_unlock(&video->cache_queue_lock);
}

static void resend_wait_bufs_over_cycle(struct c1200_isp_video *video,
					int cycle)
{
	struct isp_buffer *buf;
	struct isp_buffer *tmp;

	mutex_lock(&video->wait_queue_lock);
	list_for_each_entry_safe(buf, tmp, &video->wait_buf_queue, node) {
		int ret;

		if (buf->cycle_count < cycle)
			continue;

		ret = send_video_buf_to_fw(video, buf->dma,
					   buf->vb.vb2_buf.index, 0);
		if (!ret) {
			if (cycle > 0) {
				list_del(&buf->node);
				buf->cycle_count = 0;
				list_add_tail(&buf->node,
					      &video->wait_buf_queue);
			}
		} else {
			/* Send fail, wait next chance */
			dev_err_ratelimited(
				video->isp->dev,
				"%s: video %02d send wait buf to fw failed!\n",
				__func__, video->video_index);
			break;
		}
	}
	mutex_unlock(&video->wait_queue_lock);
}

static int send_next_buf_to_fw(struct c1200_isp_video *video)
{
	int empty;
	struct isp_buffer *buf;
	struct isp_buffer *tmp;
	int wait_count;
	int flag;
	int ret;

	// send next framebuffer to fw
	mutex_lock(&video->free_queue_lock);
	empty = list_empty(&video->free_buf_queue);
	if (empty) {
		mutex_unlock(&video->free_queue_lock);
		flag = 0;
		wait_count = 0;
		// free queue is empty, check wait queue
		mutex_lock(&video->wait_queue_lock);
		list_for_each_entry(buf, &video->wait_buf_queue, node) {
			wait_count++;
			if (buf->cycle_count > MIN_FRAME_BUFFER_NUM) {
				flag = 1;
				// delete this node from head and insert it to
				// tail
				list_del(&buf->node);
				list_add_tail(&buf->node,
					      &video->wait_buf_queue);
				break;
			}
		}
		mutex_unlock(&video->wait_queue_lock);

		if (flag) {
			dev_err_ratelimited(
				video->isp->dev,
				"video %02d: resend buf 0x%08X, 0x%08X, 0x%08X, cycle: %d\n",
				video->video_index, buf->dma[0], buf->dma[1],
				buf->dma[2], buf->cycle_count);
			buf->cycle_count = 0;
			ret = send_video_buf_to_fw(video, buf->dma,
						   buf->vb.vb2_buf.index, 0);
			if (ret) {
				dev_err_ratelimited(
					video->isp->dev,
					"video %02d: send wait buf %d failed\n",
					video->video_index,
					buf->vb.vb2_buf.index);
			}
		} else {
			if (video->tx_drop_count == 0) {
				// driver received fw reserved buf at first
				// because the fw delayed to fill the sent buf
				// reset the count here
				video->rx_reserved_count = 0;
			}
			video->tx_drop_count++;
			return -1;
		}
	} else {
		list_for_each_entry_safe(buf, tmp, &video->free_buf_queue,
					 node) {
			/* Add to wait queue first since ISP FW may use this
			 * before ipc_send return
			 */
			list_del(&buf->node);
			mutex_lock(&video->wait_queue_lock);
			list_add_tail(&buf->node, &video->wait_buf_queue);
			mutex_unlock(&video->wait_queue_lock);
			ret = send_video_buf_to_fw(video, buf->dma,
						   buf->vb.vb2_buf.index, 0);
			if (!ret) {
				/* Send all free buffers to FW on success */
				buf->cycle_count = 0;
			} else {
				/* Send fail, remove from wait queue, add back
				 * to free queue, wait next chance
				 */
				mutex_lock(&video->wait_queue_lock);
				list_del(&buf->node);
				mutex_unlock(&video->wait_queue_lock);
				list_add(&buf->node, &video->free_buf_queue);
				dev_err_ratelimited(
					video->isp->dev,
					"video %02d: send free buf %d failed\n",
					video->video_index,
					buf->vb.vb2_buf.index);
				break;
			}
		}
		mutex_unlock(&video->free_queue_lock);
	}

	return 0;
}

static int handle_video_sync_msg(struct c1200_isp_video *video,
				 struct internal_msg *cache_msg)
{
	int duplicate_msg;
	struct isp_buffer *buf;
	int buf_reserved = 0;
	u64 isp_capture_ktime = 0;
	uint64_t tick;
	uint64_t ktime;
	u32 sequence;
	int i;
	new_rawframe_done_t *raw_frame; // for raw video
	uint32_t raw_fbuf;
	new_frame_done_t *view_frame; // for view video
	uint32_t *view_fbuf;

	duplicate_msg = 0;
	if (video->is_raw_video) { // raw video
		raw_frame = (new_rawframe_done_t *)&(cache_msg->user_data[0]);
		raw_fbuf = raw_frame->rawBuf;
	} else { // view video
		view_frame = (new_frame_done_t *)&(cache_msg->user_data[0]);
		view_fbuf = view_frame->viewBuf;
	}
	tick = cache_msg->tick;
	ktime = cache_msg->ktime;
	if (video->hdmi_video) {
		// hdmi has no vsync, so using driver time and count
		isp_capture_ktime = (uint64_t)ktime;
		sequence = ++video->drv_sequence;
	} else {
		isp_capture_ktime = (uint64_t)ktime - tick * ISP_TICK_TO_NS;
		sequence = video->is_raw_video ? raw_frame->vsyncCnt : // raw
						 // video
				   view_frame->viewReply.vsyncCnt; // view video
		/* After ISP reset, drv_sequence saved the last sequence, add it for continuity */
		sequence += video->drv_sequence;
	}
	video->last_timestamp = isp_capture_ktime;
	if (video->is_raw_video) { // raw video
		dev_dbg(video->isp->dev,
			"RECVBUF: raw video %02d, 0x%08X,, tick: 0x%08llX, sequence: %u\n",
			video->video_index, raw_fbuf, tick, sequence);

		if (raw_fbuf == video->reserve_buf)
			buf_reserved = 1;
	} else { // view video
		dev_dbg(video->isp->dev,
			"RECVBUF: video %02d, 0x%08X, 0x%08X, 0x%08X, tick: 0x%08llX, sequence: %u\n",
			video->video_index, view_fbuf[0], view_fbuf[1],
			view_fbuf[2], tick, sequence);

		for (i = 0; i < ISP_CHANNEL_VIEW_NUM; i++) {
			if (video->current_views_id[i])
				buf_reserved +=
					is_reserved_buf(video, view_fbuf[i]) ?
						1 :
						0;
		}
	}

	if (buf_reserved) {
		/* Since it is not possible to keep consist identical
		 * execution sequence between Driver and Firmware, we DO NOT
		 * consider the first received reserved buffer as abnormal,
		 * typically,
		 * 1. Driver stream on video 0, this cause all video datas from
		 * same MIPI enter ISP
		 * 2. Firmware will use reserved buffer to keep video 1~3's data
		 * 3. Driver stream on video 1, Firmware send the reserved
		 * buffer to Driver if it utilize the buffer send by driver
		 * promptly.
		 */
		if (video->rx_reserved_count) {
			dev_err_ratelimited(
				video->isp->dev,
				"%s: video %02d, receive reserved buf: %d, sequence: %d\n",
				__func__, video->video_index, buf_reserved,
				sequence);
			mutex_lock(&video->wait_queue_lock);
			list_for_each_entry(buf, &video->wait_buf_queue, node) {
				buf->cycle_count++;
			}
			mutex_unlock(&video->wait_queue_lock);
			if (list_empty(&video->free_buf_queue))
				resend_wait_bufs_over_cycle(
					video, MAX_DELAY_CYCLE_TO_RESEND);
			else
				send_next_buf_to_fw(video);
		}
		video->rx_reserved_count++;
	} else {
		int buf_matched;
		struct isp_buffer *last_wait_buf;
		struct list_head *pos;

		if (!video->first_buf_received)
			video->first_buf_received = true;

		// compare the returned buf address with the wait buffer
		mutex_lock(&video->wait_queue_lock);
		list_for_each(pos, &video->wait_buf_queue) {
			last_wait_buf =
				list_entry(pos, struct isp_buffer, node);

			if (video->is_raw_video) { // raw video
				buf_matched =
					(raw_fbuf == last_wait_buf->dma[0]) ?
						1 :
						0;
			} else { // view video
				buf_matched = 0;
				for (i = 0; i < ISP_CHANNEL_VIEW_NUM; i++) {
					if (video->current_views_id[i]) {
						buf_matched +=
							(view_fbuf[i] ==
							 last_wait_buf->dma[i]) ?
								1 :
								0;
					}
				}
			}
			if (buf_matched) {
				list_del(&last_wait_buf->node);
				break;
			}
			last_wait_buf->cycle_count++;
		}
		mutex_unlock(&video->wait_queue_lock);
		if (buf_matched) {
			if (video->is_raw_video) // raw video
				video->last_done_buf[0] = raw_fbuf;
			else { // view video
				if (buf_matched != video->current_views_num)
					dev_err_ratelimited(
						video->isp->dev,
						"video %02d, matched buf is NOT consistent with views, buf_matched: %d, current_views_num: %d\n",
						video->video_index, buf_matched,
						video->current_views_num);
				for (i = 0;
				     i < ARRAY_SIZE(video->last_done_buf); i++)
					video->last_done_buf[i] = view_fbuf[i];
			}
			last_wait_buf->cycle_count = 0;
			/*save timestamp to timecode*/
			last_wait_buf->vb.vb2_buf.timestamp = isp_capture_ktime;
			last_wait_buf->vb.sequence = sequence;
			vb2_buffer_done(&last_wait_buf->vb.vb2_buf,
					VB2_BUF_STATE_DONE);
			video->last_good_sequence = sequence;
		} else {
			// not found buffer in wait list
			if (video->is_raw_video) { // raw video
				if (video->last_done_buf[0] == raw_fbuf)
					duplicate_msg = 1;
				dev_err_ratelimited(
					video->isp->dev,
					"video %02d, buf not matched, last: 0x%08X, buf: 0x%08X, duplicated: %d\n",
					video->video_index,
					video->last_done_buf[0], raw_fbuf,
					duplicate_msg);
			} else { // view video
				for (i = 0;
				     i < ARRAY_SIZE(video->last_done_buf); i++)
					if (video->current_views_id[i]) {
						duplicate_msg =
							video->last_done_buf[i] ==
									view_fbuf[i] ?
								1 :
								0;
						dev_err_ratelimited(
							video->isp->dev,
							"video %02d, view: %d, buf not matched, last: 0x%08X, buf: 0x%08X, duplicated: %d\n",
							video->video_index, i,
							video->last_done_buf[i],
							view_fbuf[i],
							duplicate_msg);
					}
			}
		}
	}

	return 0;
}

// get element from free cache queue, copy data
int copy_msg_to_video_cache(struct c1200_isp_video *video, uint16_t main,
			    uint16_t minor, uint32_t tick, int64_t ktime,
			    uint32_t *data, int num, int line_length)
{
	struct internal_msg *cache_msg;
	int empty;
	int video_status;
	int row_time;
	int ret;

	if (video == NULL)
		return -1;

	if (video->channel == NULL)
		return -1;
	if (video->channel->cam_dev == NULL)
		return -1;

	row_time = video->channel->cam_dev->row_time;
	video->sensor_exp = line_length * row_time;

	mutex_lock(&video->cache_queue_lock);
	empty = list_empty(&video->free_cache_queue);
	if (empty) {
		mutex_unlock(&video->cache_queue_lock);
		dev_err_ratelimited(
			video->isp->dev,
			"ERROR: video %02d NO free cache msg element\n",
			video->video_index);
		return -1;
	}
	cache_msg = list_first_entry(&video->free_cache_queue,
				     struct internal_msg, node);
	list_del(&cache_msg->node);
	mutex_unlock(&video->cache_queue_lock);

	// copy msg to cache
	cache_msg->cmd_main = main;
	cache_msg->cmd_minor = minor;
	cache_msg->tick = tick;
	cache_msg->ktime = ktime;
	memcpy(&cache_msg->user_data[0], data,
	       num * sizeof(uint32_t)); // 4 byte per data
	mutex_lock(&video->msg_queue_lock);
	video_status = atomic_read(&video->status);
	if (video_status != VIDEO_STATUS_STREAMON_DONE) {
		mutex_lock(&video->cache_queue_lock);
		list_add_tail(&cache_msg->node, &video->free_cache_queue);
		mutex_unlock(&video->cache_queue_lock);
		dev_dbg(video->isp->dev,
			"video %02d: status = %d, can't handle FW message, main: %d, minor = %d, 0x%08X, 0x%08X, 0x%08X, 0x%08X\n",
			video->video_index, video_status, main, minor, data[0],
			data[1], data[2], data[3]);
		ret = -1;
	} else {
		if (video->msg_work_queue != NULL)
			queue_work(video->msg_work_queue, &cache_msg->msg_work);
		ret = 0;
	}
	mutex_unlock(&video->msg_queue_lock);

	return ret;
}

static __u32 get_v4l2_pix_format_from_dt(int data_type)
{
	switch (data_type) {
	case MIPI_CSI2_DT_RAW12:
		return V4L2_PIX_FMT_SBGGR12P;
	case MIPI_CSI2_DT_RAW14:
		return V4L2_PIX_FMT_SBGGR14P;
	case MIPI_CSI2_DT_RAW16:
		return V4L2_PIX_FMT_SBGGR16;
	default:
		return 0;
	}
}

// 0/1/2/3/4/5 YUV420(3 channels,Y/U/V)/NV12(2 channels,Y/UV)/
// NV21(2 channels,Y/VU)/YUV422(1 channel,YUYV)/RGB888(1 channel)
static int convert_view_format_to_v4l2(int view_id, int view_format)
{
	if (view_id == 0) {
		switch (view_format) {
		case 0:
			return V4L2_PIX_FMT_YUV420;
		case 1:
			return V4L2_PIX_FMT_NV12;
		case 2:
			return V4L2_PIX_FMT_NV21;
		case 3:
			return V4L2_PIX_FMT_YUYV;
		case 4:
			return V4L2_PIX_FMT_RGB24;
		default:
			pr_err("ERROR: unsupported format %d\n", view_format);
			return -1;
		}
	} else if (view_id == 1) {
		// 0/1/2/3/4/5 YUV420(3 channels,Y/U/V)/NV12(2 channels,Y/UV)
		//  /NV21(2 channels,Y/VU)/YUV422(1 channel,YUYV)/Raw(1
		//  channel)/disabled
		switch (view_format) {
		case 0:
			return V4L2_PIX_FMT_YUV420;
		case 1:
			return V4L2_PIX_FMT_NV12;
		case 2:
			return V4L2_PIX_FMT_NV21;
		case 3:
			return V4L2_PIX_FMT_YUYV;
		case 4:
			// to do: should return RAW
			return -1;
		default:
			pr_err("ERROR: unsupported format %d\n", view_format);
			return -1;
		}
	} else if (view_id == 2) {
		switch (view_format) {
		case 1:
			return V4L2_PIX_FMT_NV12;
		default:
			return V4L2_PIX_FMT_GREY;
		}
	}

	pr_err("ERROR: wrong view id %d\n", view_id);
	return -1;
}

void update_raw_video_from_camera_config(struct c1200_isp_video *video,
					 struct camera_dev *cam_dev)
{
	int dt;

	/* RAW video only has view 0 now */
	video->views[1].enable = false;
	video->views[2].enable = false;

	dt = cam_dev->isp_data.rawinfo.dataType;
	if (!dt) {
		video->views[0].enable = false;
		return;
	}

	video->views[0].enable = true;
	video->views[0].is_pdns_input = false; // false by default
	video->views[0].index = 0;
	video->views[0].video = video;
	video->views[0].width = cam_dev->isp_data.rawinfo.width;
	video->views[0].height = cam_dev->isp_data.rawinfo.height;
	video->views[0].format = get_v4l2_pix_format_from_dt(dt);
	video->views[0].cmd_view_id = video->chn_index + S00RawId;
}

int update_video_from_camera_config(struct c1200_isp_video *video,
				    struct camera_dev *cam_dev)
{
	int view_fmt;

	view_fmt = cam_dev->isp_data.viewinfo[0].viewFmt;
	if (view_fmt != View0_Dis) {
		video->views[0].enable = true;
		video->views[0].is_pdns_input = false; // false by default
		video->views[0].index = 0;
		video->views[0].video = video;
		video->views[0].width = cam_dev->isp_data.viewinfo[0].width;
		video->views[0].height = cam_dev->isp_data.viewinfo[0].height;
		video->views[0].format =
			convert_view_format_to_v4l2(0, view_fmt);
		video->views[0].cmd_view_id =
			((video->chn_index * VIEW_ID_NUM_PER_CAMERA) +
			 S00View0);
		// pr_info("v0_w = %d, v0_h = %d\n",
		//	video->views[0].width, video->views[0].height);
	}
	view_fmt = cam_dev->isp_data.viewinfo[1].viewFmt;
	if (view_fmt != View1_Dis) {
		video->views[1].enable = true;
		video->views[1].is_pdns_input = false; // false by default
		video->views[1].index = 1;
		video->views[1].video = video;
		video->views[1].width = cam_dev->isp_data.viewinfo[1].width;
		video->views[1].height = cam_dev->isp_data.viewinfo[1].height;
		video->views[1].format =
			convert_view_format_to_v4l2(1, view_fmt);
		video->views[1].cmd_view_id =
			((video->chn_index * VIEW_ID_NUM_PER_CAMERA) +
			 S00View1);
		// pr_info("v1_w = %d, v1_h = %d\n",
		//	video->views[1].width, video->views[1].height);
	}
	view_fmt = cam_dev->isp_data.viewinfo[2].viewFmt;
	if (view_fmt != View2_Dis) {
		video->views[2].enable = true;
		video->views[2].is_pdns_input = false; // false by default
		video->views[2].index = 2;
		video->views[2].video = video;
		video->views[2].width = cam_dev->isp_data.viewinfo[2].width;
		video->views[2].height = cam_dev->isp_data.viewinfo[2].height;
		video->views[2].format =
			convert_view_format_to_v4l2(2, view_fmt);
		video->views[2].cmd_view_id =
			((video->chn_index * VIEW_ID_NUM_PER_CAMERA) +
			 S00View2);
		// pr_info("v2_w = %d, v2_h = %d\n",
		//	video->views[2].width, video->views[2].height);
	}
	video->pdns_input_view = -1; // default to invalid value;

	if (!video->is_raw_video) // for view video
		if (cam_dev->isp_data.pdnsinfo.pdnsMode) {
			int inputview;

			inputview = cam_dev->isp_data.pdnsinfo.pdnsViewSel;
			if ((inputview >= 0) &&
			    (inputview < MAX_VIEWS_PER_CAMERA)) {
				video->views[inputview].is_pdns_input = true;
				video->pdns_input_view = inputview;
			}
		}

	return 0;
}

static int c1200_isp_video_init(struct c1200_isp_device *isp,
				struct c1200_isp_video *video, int index)
{
	int i;

	// default entity to isp view
	atomic_set(&(video->status), VIDEO_STATUS_INVALID);
	video->error = false;
	video->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

	INIT_LIST_HEAD(&video->free_buf_queue);
	INIT_LIST_HEAD(&video->wait_buf_queue);
	INIT_LIST_HEAD(&video->free_cache_queue);
	init_completion(&video->stream_comp);

	for (i = 0; i < MAX_MEDIA_COMMAND_PER_VIDEO; i++) {
		INIT_WORK(&video->cache_msg[i].msg_work, ipc_msg_handler);
		video->cache_msg[i].video_ptr = video;
		list_add_tail(&video->cache_msg[i].node,
			      &video->free_cache_queue);
	}

	mutex_init(&video->mutex);
	mutex_init(&video->msg_queue_lock);
	mutex_init(&video->queue_lock);
	mutex_init(&video->ctrl_lock);
	mutex_init(&video->cache_queue_lock);
	mutex_init(&video->free_queue_lock);
	mutex_init(&video->wait_queue_lock);
	video->video.fops = &isp_video_fops;
	video->video.vfl_type = VFL_TYPE_VIDEO;
	video->video.release = video_device_release_empty;
	video->video.ioctl_ops = &isp_video_ioctl_ops;
	video->video.device_caps = V4L2_CAP_VIDEO_CAPTURE |
				   V4L2_CAP_VIDEO_CAPTURE_MPLANE |
				   V4L2_CAP_STREAMING;
	video_set_drvdata(&video->video, video);

	return 0;
}

int isp_channel_init_video(struct c1200_isp_device *isp,
			   struct bst_isp_channel *isp_channel,
			   enum c1200_isp_video_type video_type, int chn_sn)
{
	int video_sn;
	int ret;
	struct camera_dev *cam;
	struct c1200_isp_video *video;

	cam = isp_channel->cam_dev;
	if (!cam) {
		dev_info(isp->dev, "%s %d , cam is NULL", __func__, __LINE__);
		return 0;
	}

	if (video_type == RAW_VIDEO) { // raw video
		video = &(isp_channel->raw_video);
		video->is_pdns = false;
		video->is_raw_video = true; // raw video means not processed by
			// ISP, include yuyv input
		snprintf(video->video.name, sizeof(video->video.name),
			 "isp-channel-%d-raw-video", chn_sn);

		video_sn = chn_sn + MAX_ISP_CHANNEL; // 12 ~ 23
	} else if (video_type == VIEW_VIDEO) { // view video
		video = &(isp_channel->views_video);
		video->is_pdns = false;
		if (isp_channel->is_hdmi)
			video->hdmi_video = true;
		else
			video->hdmi_video = false;
		// default entity to isp view
		snprintf(video->video.name, sizeof(video->video.name),
			 "isp-channel-%d-views-video", chn_sn);

		video_sn = chn_sn; // 0 ~ 11
	} else {
		pr_err("ERROR! video_type (raw or view) is not certain");
		return -EINVAL;
	}

	video->isp = isp;
	video->enabled = true; // will be updated from config
	video->chn_index = chn_sn;
	video->video_index = video_sn;
	video->channel = isp_channel;
	init_completion(&video->feed_wait_buf_completion);
	c1200_isp_video_init(isp, video, video_sn);
	video->pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&video->video.entity, 1, &video->pad);

	return 0;
}

void c1200_isp_video_cleanup(struct c1200_isp_video *video)
{
	mutex_destroy(&video->queue_lock);
	mutex_destroy(&video->msg_queue_lock);
	mutex_destroy(&video->mutex);
	mutex_destroy(&video->ctrl_lock);
}

int c1200_isp_video_register(struct c1200_isp_video *video,
			     struct v4l2_device *vdev,
			     enum c1200_isp_video_type video_type)
{
	int ret;
	int device_index;

	video->video.v4l2_dev = vdev;
	device_index = video->channel->remote_mipi_id * MAX_MIPI_DEVICE_NUM +
		       video->channel->remote_mipi_vc_index;

	/*Raw video index start with number 16*/
	if (video_type == RAW_VIDEO)
		device_index += MAX_ISP_MIPI_VC;

	dev_dbg(video->isp->dev, "register v4l2 device mipi_id:%d vc:%d",
		video->channel->remote_mipi_id,
		video->channel->remote_mipi_vc_index);

	ret = video_register_device(&video->video, VFL_TYPE_VIDEO,
				    device_index);
	if (ret < 0)
		dev_err(video->isp->dev,
			"%s: could not register video device (%d)\n", __func__,
			ret);

	dev_dbg(video->isp->dev,
		"%s() line %d , sensor_index %d, registered /dev/video%d\n",
		__func__, __LINE__, video->video_index, device_index);

	video->device_index = device_index;

	return ret;
}

void c1200_isp_video_unregister(struct c1200_isp_video *video)
{
	if (video_is_registered(&video->video))
		video_unregister_device(&video->video);
}

/*
 * Show the mapping info for videoX with channel_index
 * Devices index May be different with isp channel index
 */
void dump_video_index_mapping_info(struct c1200_isp_device *isp_dev, char *buf)
{
	int i;
	struct bst_isp_channel *channel;
	struct c1200_isp_video *video;
	int buf_len;

	buf_len = 0;
	for (i = 0; i < MAX_ISP_CHANNEL; i++) {
		channel = &isp_dev->channels[i];
		if (!channel)
			continue;
		video = &channel->views_video;
		if (!video)
			continue;
		if (!channel->enable)
			continue;
		/*Dump video info*/
		buf_len += snprintf(buf + buf_len, PAGE_SIZE - buf_len,
				    "===== video %02d Mapping info =====\n",
				    video->device_index);
		buf_len += snprintf(buf + buf_len, PAGE_SIZE - buf_len,
				    "%-20s:%-36s\n", "Camera name",
				    video->channel->cam_dev->camera_name);
		buf_len += snprintf(buf + buf_len, PAGE_SIZE - buf_len,
				    "%-20s:%-3d\n", "Camera fps",
				    video->channel->cam_dev->sensor_fps);
		buf_len += snprintf(buf + buf_len, PAGE_SIZE - buf_len,
				    "%-20s:%-3d\n", "isp_channel_index",
				    video->chn_index);
		buf_len += snprintf(buf + buf_len, PAGE_SIZE - buf_len,
				    "%-20s:%-3d\n", "mipi_index",
				    channel->cam_dev->isp_data.mipiSensorIndex);
		buf_len += snprintf(buf + buf_len, PAGE_SIZE - buf_len,
				    "%-20s:%-10d\n", "row_time",
				    video->channel->cam_dev->row_time);
		if (channel->cam_dev->power_on)
			buf_len += snprintf(buf + buf_len, PAGE_SIZE - buf_len,
					    "%-20s:%-5s\n", "connected",
					    "true");
		else
			buf_len += snprintf(buf + buf_len, PAGE_SIZE - buf_len,
					    "%-20s:%-5s\n", "connected",
					    "false");

		buf_len += snprintf(buf + buf_len, PAGE_SIZE - buf_len,
				    "=================================\n");
	}

	for (i = 0; i < MAX_ISP_CHANNEL; i++) {
		channel = &isp_dev->channels[i];
		if (!channel) {
			buf_len += snprintf(buf + buf_len, PAGE_SIZE - buf_len,
					    "ispChannel[%d] is disabled\n", i);
			continue;
		}
		if (!channel->enable) {
			buf_len += snprintf(buf + buf_len, PAGE_SIZE - buf_len,
					    "ispChannel[%d] is disabled\n", i);
			continue;
		}
		if (channel->cam_dev == NULL) {
			buf_len += snprintf(
				buf + buf_len, PAGE_SIZE - buf_len,
				"ispChannel[%d]'s cam_dev is disabled\n", i);
			continue;
		}
	}
}

void dump_video_debug_info(struct c1200_isp_video *video)
{
	dev_err(&video->video.dev, "===== video %02d debug info =====\n",
		video->video_index);
	dev_err(&video->video.dev,
		"video tx_count = %lld,video rx_count = %lld, isp rx count = %lld, isp tx count = %lld\n",
		video->tx_buf_count, video->rx_buf_count,
		video->isp->ipc_rx_count, video->isp->ipc_tx_count);
	dev_err(&video->video.dev, "isp kthread status = %d\n",
		video->isp->kthread_status);

	dev_err(&video->video.dev, "~~~~~ debug info end ~~~~~\n");

	dev_err(&video->video.dev,
		"video index = %d, tx buf = %lld, rx buf = %lld\n",
		video->video_index, video->tx_buf_count, video->rx_buf_count);
}
