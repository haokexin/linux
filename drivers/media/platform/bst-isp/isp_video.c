// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <dt-bindings/media/bst-isp.h>

#include <linux/delay.h>
#include <linux/dma-direct.h>
#include <linux/dma-direction.h>
#include <linux/dma-map-ops.h>
#include <linux/iommu.h>
#include <linux/math.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/swiotlb.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/videobuf2-dma-contig.h>

#ifdef CONFIG_BST_GTC
#include <bst/bst_gtc.h>
#endif
#include <bst/media-dev.h>

#include "isp_video.h"

#include "isp_core.h"
#include "isp_hw.h"
#include "isp_msg.h"
#include "isp_proto_base.h"
#include "isp_proto_ipc.h"

/* -----------------------------------------------------------------------------
 * Helper functions
 */
/* See also:
 *   dma_direct_sync_single_for_device
 *   iommu_dma_sync_single_for_device
 */
static void dma_sync_for_dev(struct isp_device *isp, dma_addr_t addr,
			     size_t size, enum dma_data_direction dir)
{
	struct device *dev;
	phys_addr_t pa;

	dev = isp->dev;
	if (isp->iommud)
		pa = isp->iommud->ops->iova_to_phys(isp->iommud, addr);
	else
		pa = dma_to_phys(isp->dev, addr);

	/* Since we has set DMA_ATTR_FORCE_CONTIGUOUS,
	 * we do not need a sg_table here.
	 */
	if (unlikely(is_swiotlb_buffer(dev, pa)))
		swiotlb_sync_single_for_device(dev, pa, size, dir);

	if (isp->cache_mode != CM_SOFTWARE)
		return;

	arch_sync_dma_for_device(pa, size, dir);
}

/* See also:
 *   dma_direct_sync_single_for_cpu
 *   iommu_dma_sync_single_for_cpu
 */
static void dma_sync_for_cpu(struct isp_device *isp, dma_addr_t addr,
			     size_t size, enum dma_data_direction dir)
{
	struct device *dev;
	phys_addr_t pa;

	dev = isp->dev;
	if (isp->iommud)
		pa = isp->iommud->ops->iova_to_phys(isp->iommud, addr);
	else
		pa = dma_to_phys(isp->dev, addr);

	/* Since we has set DMA_ATTR_FORCE_CONTIGUOUS,
	 * we do not need a sg_table here.
	 */
	if (isp->cache_mode == CM_SOFTWARE) {
		arch_sync_dma_for_cpu(pa, size, dir);
		arch_sync_dma_for_cpu_all();
	}

	if (unlikely(is_swiotlb_buffer(dev, pa)))
		swiotlb_sync_single_for_cpu(dev, pa, size, dir);

	if (dir == DMA_FROM_DEVICE)
		arch_dma_mark_clean(pa, size);
}

static u32 get_bpp(u32 v4l2_fmt)
{
	switch (v4l2_fmt) {
	case V4L2_PIX_FMT_GREY:
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_NV61:
	case V4L2_PIX_FMT_NV24:
	case V4L2_PIX_FMT_NV42:
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
		return 8;
	case V4L2_PIX_FMT_SBGGR10P:
		return 10;
	case V4L2_PIX_FMT_SBGGR12P:
		return 12;
	case V4L2_PIX_FMT_SBGGR14P:
		return 14;
	case V4L2_PIX_FMT_ARGB444:
	case V4L2_PIX_FMT_XRGB444:
	case V4L2_PIX_FMT_ABGR444:
	case V4L2_PIX_FMT_XBGR444:
	case V4L2_PIX_FMT_RGBA444:
	case V4L2_PIX_FMT_RGBX444:
	case V4L2_PIX_FMT_BGRA444:
	case V4L2_PIX_FMT_BGRX444:
	case V4L2_PIX_FMT_ARGB555:
	case V4L2_PIX_FMT_XRGB555:
	case V4L2_PIX_FMT_XRGB555X:
	case V4L2_PIX_FMT_ABGR555:
	case V4L2_PIX_FMT_XBGR555:
	case V4L2_PIX_FMT_RGBA555:
	case V4L2_PIX_FMT_RGBX555:
	case V4L2_PIX_FMT_BGRA555:
	case V4L2_PIX_FMT_BGRX555:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB565X:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YYUV:
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
	case V4L2_PIX_FMT_YUV422P:
	case V4L2_PIX_FMT_SBGGR16:
		return 16;

	case V4L2_PIX_FMT_RGB24:
		return 24;

	case V4L2_PIX_FMT_BGR32:
	case V4L2_PIX_FMT_ABGR32:
	case V4L2_PIX_FMT_XBGR32:
	case V4L2_PIX_FMT_BGRA32:
	case V4L2_PIX_FMT_BGRX32:
	case V4L2_PIX_FMT_RGB32:
	case V4L2_PIX_FMT_RGBA32:
	case V4L2_PIX_FMT_RGBX32:
	case V4L2_PIX_FMT_ARGB32:
	case V4L2_PIX_FMT_XRGB32:
		return 32;

	default:
		pr_err("%s: unsupported pixel format: 0x%08X\n", __func__,
		       v4l2_fmt);
		return 0;
	}
}

static u32 get_view_stride(u32 v4l2_fmt, u32 width, u32 align)
{
	return ALIGN(width * get_bpp(v4l2_fmt) / 8, align);
}

static u32 get_plane_lines(u32 v4l2_fmt, u32 height)
{
	switch (v4l2_fmt) {
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
		return height * 3 / 2;

	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_NV61:
		return height * 2;

	case V4L2_PIX_FMT_NV24:
	case V4L2_PIX_FMT_NV42:
		return height * 3;

	default:
		return height;
	}
}

static u32 get_raw_stride(u32 v4l2_fmt, u32 width, u32 burst, u32 pack)
{
	u32 bpp;
	u32 pixels_per_burst;
	u32 line_bytes;

	bpp = get_bpp(v4l2_fmt);
	if (bpp == 0)
		return 0;

	/* Re-calculate for multi-planes */
	switch (v4l2_fmt) {
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
		bpp = bpp * 3 / 2;
		break;
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_NV61:
		bpp = bpp * 2;
		break;
	case V4L2_PIX_FMT_NV24:
	case V4L2_PIX_FMT_NV42:
		bpp = bpp * 3;
		break;
	default:
		break;
	}

	pixels_per_burst = burst * 8 / bpp;
	line_bytes = (width + pixels_per_burst - 1) / pixels_per_burst * burst;

	return ALIGN(line_bytes, pack);
}

static u32 get_meta_bytes(struct isp_video *video)
{
	int i;
	u32 size;

	size = sizeof(struct meta_desc) + video->isp_meta.size;
	for (i = 0; i < ARRAY_SIZE(video->sensor_meta); ++i)
		size += video->sensor_meta[i].size;

	return size;
}

static u32 dt_to_v4l2_pix_format(int dt)
{
	switch (dt) {
	case DT_YUV422_8B:
		return V4L2_PIX_FMT_YUV422P;
	case DT_RGB888:
		return V4L2_PIX_FMT_RGB24;
	case DT_RAW10:
		return V4L2_PIX_FMT_SBGGR10P;
	case DT_RAW12:
		return V4L2_PIX_FMT_SBGGR12P;
	case DT_RAW14:
		return V4L2_PIX_FMT_SBGGR14P;
	case DT_RAW16:
		return V4L2_PIX_FMT_SBGGR16;
	default:
		pr_err("%s: unsupported data type: 0x%02X\n", __func__, dt);
		return 0;
	}
}

static u32 view_fmt_to_v4l2_pix_format(int view_fmt)
{
	switch (view_fmt) {
	case View0_YUVSep_Fmt:
		return V4L2_PIX_FMT_YUV420;
	case View0_NV12_Fmt:
		return V4L2_PIX_FMT_NV12;
	case View0_NV21_Fmt:
		return V4L2_PIX_FMT_NV21;
	case View0_YUV422_Fmt:
		return V4L2_PIX_FMT_YUYV;
	case View0_RGB888_Fmt:
		return V4L2_PIX_FMT_RGBA32;
	default:
		pr_err("%s: unsupported view format: %d\n", __func__, view_fmt);
		return 0;
	}
}

/*
 * Send free buffers to firmware
 *
 * @video: ISP video.
 *
 * Returns:
 *   0: No free buffer
 *  <0: Error
 *  >0: Sent buffer number
 */
static int send_free_bufs(struct isp_video *video)
{
	int rv;
	int num;
	struct isp_buffer *buf;
	struct isp_buffer *tmp;

	mutex_lock(&video->free_buf_lock);
	if (list_empty(&video->free_buf_queue)) {
		mutex_unlock(&video->free_buf_lock);
		return 0;
	}

	num = 0;
	list_for_each_entry_safe(buf, tmp, &video->free_buf_queue, node) {
		/* Add to wait queue first since ISP FW may use this
		 * before TX API returns when there is only 1 buffer
		 */
		list_del(&buf->node);
		mutex_lock(&video->wait_buf_lock);
		list_add_tail(&buf->node, &video->wait_buf_queue);
		++video->stat.buf_num_fw;
		mutex_unlock(&video->wait_buf_lock);
		rv = isp_msg_video_txbuf(video, buf->dma);
		if (!rv) {
			buf->cycle = 0; /* On success */
			++num;
			++video->stat.tx_buf_done;
		} else {
			/* On fail, remove from wait queue, add back
			 * to free queue, wait next chance
			 */
			++video->stat.tx_buf_fail;
			mutex_lock(&video->wait_buf_lock);
			list_del(&buf->node);
			--video->stat.buf_num_fw;
			mutex_unlock(&video->wait_buf_lock);
			list_add(&buf->node, &video->free_buf_queue);
			if (rv != -EPERM)
				dev_err_ratelimited(
					video->isp->dev,
					"V%02d: Failed to send free buf %d: 0x%08X, 0x%08X, 0x%08X\n",
					video->vid, buf->vb.vb2_buf.index,
					buf->dma[0], buf->dma[1], buf->dma[2]);
			break;
		}
	}
	mutex_unlock(&video->free_buf_lock);

	if (num > 0)
		return num;

	return -1;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

/*
 * Send wait buffers to firmware
 *
 * @video: ISP video.
 * @cycle: The buffer has waited over how much reserved buffers
 *
 * Returns:
 *   0: No free buffer
 *  <0: Error
 *  >0: Sent buffer number
 */
static int send_wait_bufs(struct isp_video *video, int cycle)
{
	int num;
	struct isp_buffer *buf;
	struct isp_buffer *tmp;

	num = 0;
	mutex_lock(&video->wait_buf_lock);
	list_for_each_entry_safe(buf, tmp, &video->wait_buf_queue, node) {
		/* NOTE: 0 vs 0 should be sent too */
		if (buf->cycle < cycle)
			continue;

		if (!isp_msg_video_txbuf(video, buf->dma)) {
			buf->cycle = 0;
			++num;
		} else {
			/* Send fail, wait next chance */
			dev_err_ratelimited(
				video->isp->dev,
				"V%02d: Failed to send wait buf %d: 0x%08X, 0x%08X, 0x%08X\n",
				video->vid, buf->vb.vb2_buf.index, buf->dma[0],
				buf->dma[1], buf->dma[2]);
			break;
		}
	}
	mutex_unlock(&video->wait_buf_lock);

	if (num > 0)
		return num;

	return -1;
}

#pragma GCC diagnostic pop

/*
 * Send next buffers to firmware
 *
 * @video: ISP video.
 *
 * Returns:
 *   0: No free buffer
 *  <0: Error
 *  >0: Sent buffer number
 */
static inline int send_next_bufs(struct isp_video *video)
{
	int num;

	num = send_free_bufs(video);
	if (num > 0)
		return num;

	/* The deprecated implemention for two reasons:
	 * 1.It thinks firmware leak the buffers, so resend wait buffers.
	 * 2.Camera is pluged out, firmware clear the buffers queue.
	 */
	return send_wait_bufs(video, MIN_FRAME_BUFFER_NUM);
}

static void update_wait_bufs_cycle(struct isp_video *video)
{
	struct isp_buffer *buf;

	mutex_lock(&video->wait_buf_lock);
	list_for_each_entry(buf, &video->wait_buf_queue, node) {
		++buf->cycle;
	}
	mutex_unlock(&video->wait_buf_lock);
}

/*
 * Wait firmware stop using buffers from driver
 *
 * @video: ISP video.
 *
 */
static void wait_bufs_idle(struct isp_video *video)
{
	/* Wait firmware return buffer */
	u32 old;
	int retries;

	retries = 1;
	while (retries <= WAIT_RETURN_BUF_TIMES) {
		old = video->stat.buf_num_fw;
		usleep_range(WAIT_RETURN_BUF_DELAY, WAIT_RETURN_BUF_DELAY + 10);
		dev_dbg(video->isp->dev,
			"V%02d RETURNBUF: retries: %d, buf in fw old: %u, current: %u\n",
			video->vid, retries, old, video->stat.buf_num_fw);
		/* The firmware don't send buffer back never more,
		 * we think it DO NOT use the buffers and can recycle.
		 */
		if (old == video->stat.buf_num_fw)
			break;
		++retries;
	}
}

/*
 * Return all queued buffers to videobuf2
 *
 * @video: ISP video object
 * @state: new state for the returned buffers
 *
 * Return all buffers queued on the video node to videobuf2 in the given state.
 * The buffer state should be VB2_BUF_STATE_QUEUED if called due to an error
 * when starting the stream, or VB2_BUF_STATE_ERROR otherwise.
 */
static void return_buffers(struct isp_video *video, enum vb2_buffer_state state)
{
	int free_count = 0;
	int wait_count = 0;
	struct device *dev = video->isp->dev;

	mutex_lock(&video->free_buf_lock);
	while (!list_empty(&video->free_buf_queue)) {
		struct isp_buffer *buf;

		++free_count;
		buf = list_first_entry(&video->free_buf_queue,
				       struct isp_buffer, node);
		list_del(&buf->node);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
		dev_dbg(dev,
			"V%02d RETURNFREEBUF: index: %d, 0x%08X, 0x%08X, 0x%08X\n",
			video->vid, buf->vb.vb2_buf.index, buf->dma[0],
			buf->dma[1], buf->dma[2]);
	}
	mutex_unlock(&video->free_buf_lock);

	/* If firmware don't send wait bufs back, recycle all forcibly. */
	mutex_lock(&video->wait_buf_lock);
	while (!list_empty(&video->wait_buf_queue)) {
		struct isp_buffer *buf;

		++wait_count;
		buf = list_first_entry(&video->wait_buf_queue,
				       struct isp_buffer, node);
		list_del(&buf->node);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
		dev_dbg(dev,
			"V%02d RETURNWAITBUF: index: %d, 0x%08X, 0x%08X, 0x%08X\n",
			video->vid, buf->vb.vb2_buf.index, buf->dma[0],
			buf->dma[1], buf->dma[2]);
	}
	mutex_unlock(&video->wait_buf_lock);
	dev_dbg(dev, "V%02d RETURNBUF: free: %d, wait: %d\n", video->vid,
		free_count, wait_count);
}

static int get_rsv_buf_num(struct isp_video *video, struct video_msg *msg)
{
	u32 i;
	int num;
	new_frame_done_t *frame;

	frame = (new_frame_done_t *)&(msg->data);
	num = 0;
	for (i = 0; i < ARRAY_SIZE(video->views); ++i)
		if (video->views[i].used)
			num += is_reserved_buf(video->isp, frame->viewBuf[i]);

	return num;
}

static int get_dup_buf_num(struct isp_video *video, struct video_msg *msg)
{
	u32 i;
	int num;
	new_frame_done_t *frame;

	frame = (new_frame_done_t *)&(msg->data);
	num = 0;

	for (i = 0; i < ARRAY_SIZE(video->views); ++i)
		if (video->views[i].used)
			num += (video->stat.last_buf[i] == frame->viewBuf[i]);

	return num;
}

/*
 * Compare the returned buffer addresses with the wait buffer
 *
 * @video:	ISP video.
 * @msg:	Video message
 *
 * Returns matched buffer or NULL
 */
static struct isp_buffer *match_and_update_wait_bufs(struct isp_video *video,
						     struct video_msg *msg)
{
	u32 i;
	struct isp_buffer *buf;
	new_frame_done_t *frame;
	u32 *dma_addrs;

	frame = (new_frame_done_t *)&(msg->data);
	dma_addrs = frame->viewBuf;

	mutex_lock(&video->wait_buf_lock);
	list_for_each_entry(buf, &video->wait_buf_queue, node) {
		int num;

		num = 0;
		for (i = 0; i < ARRAY_SIZE(video->views); ++i)
			if (video->views[i].used)
				num += (dma_addrs[i] == buf->dma[i]);

		if (num == video->used_views_num) {
			list_del(&buf->node);
			--video->stat.buf_num_fw;
			mutex_unlock(&video->wait_buf_lock);
			return buf;
		}
	}
	mutex_unlock(&video->wait_buf_lock);

	return NULL;
}

static u32 get_exposure_time(struct isp_video *video, u32 exp_lines)
{
	u32 row_time;
	u32 sampling_lines;

	sampling_lines = video->channel->cfg->ispInBotCrop / 2;
	row_time = video->channel->cam_dev->row_time;

	return (sampling_lines + exp_lines) * row_time / 1000;
}

static const char *get_abnormal_msg(u8 abn_id)
{
	switch (abn_id) {
	case ABN_HW_QUEUE_IRQ:
		return "DDR queue fail";
	case ABN_SW_NO_SYNC_FRAME_DATA:
		return "Frames are not synced";
	case ABN_SW_RAW_BADFRAME:
		return "Frame is bad";
	default:
		return "Undefined";
	}
}

static void handle_frame_done(struct isp_video *video, struct video_msg *msg)
{
	int i;
	struct isp_buffer *buf;
	int rsv_buf_num;
	int dup_buf_num;
	u64 ts;
	u32 sn;
	new_frame_done_t *frame;
	u32 *dma_addrs;

	/* Clear error flag when received new frame buffer */
	video->error = false;

	++video->stat.rx_buf_all;
	frame = (new_frame_done_t *)&(msg->data);
	dma_addrs = frame->viewBuf;
	sn = frame->viewReply.vsyncCnt;
	/* After ISP reset, driver sequence is saved the last sequence,
	 * add it for continuity.
	 */
	sn += video->stat.sn_drv;
	ts = msg->timestmap;
	if (video->stat.last_sn_all != 0) {
		if (sn == video->stat.last_sn_all)
			++video->stat.dup_num;
		else if (sn > video->stat.last_sn_all)
			video->stat.drop_num += (sn - video->stat.last_sn_all - 1);
		else
			++video->stat.reverse_num;
	}

	video->stat.last_sn_all = sn;
	video->stat.last_ts_all = ts;
	dev_dbg(video->isp->dev,
		"V%02d RXBUF: 0x%08X, 0x%08X, 0x%08X, sn: %8u, ts: %12lluus/%12lluus\n",
		video->vid, dma_addrs[0], dma_addrs[1], dma_addrs[2], sn,
		ts / 1000, ktime_get_boottime_ns() / 1000);
	rsv_buf_num = get_rsv_buf_num(video, msg);
	if (rsv_buf_num) {
		/* Since it is not possible to keep consist identical
		 * execution sequence between Driver and Firmware, we DO NOT
		 * consider the first received reserved buffer as abnormal.
		 * Typically,
		 * 1. Driver stream on video 0, this cause all video datas from
		 * same MIPI enter ISP
		 * 2. Firmware will use reserved buffer to keep video 1~3's data
		 * 3. Driver stream on video 1, Firmware send the reserved
		 * buffer to Driver if it CAN NOT utilize the buffer sent by
		 * driver promptly.
		 */
		if (video->stat.rx_buf_rsv > MIN_FRAME_BUFFER_NUM) {
			static DEFINE_RATELIMIT_STATE(_rs,
						      ISP_RATELIMIT_INTERVAL,
						      ISP_RATELIMIT_BURST);

			if (__ratelimit(&_rs))
				dev_warn(
					video->isp->dev,
					"V%02d RSVBUF: %d, 0x%08X, 0x%08X, 0x%08X, sn: %8u, ts: %12lluus\n",
					video->vid, rsv_buf_num, dma_addrs[0],
					dma_addrs[1], dma_addrs[2], sn,
					ts / 1000);
			update_wait_bufs_cycle(video);

			mutex_lock(&video->stream_lock);
			if (video->status == VS_STREAM_ON)
				send_next_bufs(video);
			mutex_unlock(&video->stream_lock);
		}
		++video->stat.rx_buf_rsv;

		return;
	}

	buf = match_and_update_wait_bufs(video, msg);
	if (buf) {
		for (i = 0; i < ARRAY_SIZE(video->stat.last_buf); ++i)
			video->stat.last_buf[i] = dma_addrs[i];
		/* Fill customed fields */
		buf->snr_exp_us = get_exposure_time(video, msg->exp_lines);
		buf->vb.vb2_buf.timestamp = ts;
		buf->vb.sequence = sn;
		video->stat.last_sn_app = sn;
		video->stat.last_ts_app = ts;
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);

		return;
	}

	dup_buf_num = get_dup_buf_num(video, msg);
	if (dup_buf_num)
		dev_err_ratelimited(
			video->isp->dev,
			"V%02d DUPBUF: %d, 0x%08X, 0x%08X, 0x%08X, sn: %8u, ts: %12lluus, last: 0x%08X, 0x%08X, 0x%08X\n",
			video->vid, dup_buf_num, dma_addrs[0], dma_addrs[1],
			dma_addrs[2], sn, ts / 1000, video->stat.last_buf[0],
			video->stat.last_buf[1], video->stat.last_buf[2]);
}

static void handle_abnormal_msg(struct isp_video *video, struct video_msg *msg)
{
	abnormal_t *fw_ab_info;

	++video->stat.rx_msg_bad;
	fw_ab_info = (abnormal_t *)&(msg->data);
	video->fw_ab_info = *fw_ab_info;

	dev_err_ratelimited(video->isp->dev,
			    "V%02d ABNORMAL: id: 0x%02X, msg: %s\n", video->vid,
			    fw_ab_info->abnormalId,
			    get_abnormal_msg(fw_ab_info->abnormalId));

	video->error = true;
	mutex_lock(&video->vb2_lock);
	if (video->vb2_queue)
		wake_up(&video->vb2_queue->done_wq);
	mutex_unlock(&video->vb2_lock);
}

static void handle_msg_entry(struct work_struct *work)
{
	struct isp_video *video;
	struct video_msg *msg;
	struct device *dev;

	msg = container_of(work, struct video_msg, msg_work);
	video = msg->video;
	dev = video->isp->dev;

	switch (msg->cmd_minor) {
	case MINOR_ISP_VIEW_FRAME_DONE:
	case MINOR_ISP_RAW_FRAME_DONE:
		handle_frame_done(video, msg);
		break;
	case MINOR_ABNORMAL:
		handle_abnormal_msg(video, msg);
		break;
	default:
		dev_err_ratelimited(dev,
				    "V%02d: unsupported cmd minor 0x%02X\n",
				    video->vid, msg->cmd_minor);
		break;
	}

	/* Put the msg to free queue after done */
	clear_video_msg(msg);
	mutex_lock(&video->free_msg_lock);
	list_add_tail(&msg->node, &video->free_msg_queue);
	mutex_unlock(&video->free_msg_lock);
}

/* -----------------------------------------------------------------------------
 * Queue operations
 */
static int isp_video_queue_setup(struct vb2_queue *queue, unsigned int *count,
				 unsigned int *num_planes, unsigned int sizes[],
				 struct device *alloc_devs[])
{
	struct isp_video_fh *vfh = vb2_get_drv_priv(queue);
	struct isp_video *video = vfh->video;
	unsigned int buf_num = MAX_FRAME_BUFFER_NUM;
	u8 i;

	dev_dbg(video->isp->dev, "V%02d QSETUP: view num: %u\n", video->vid,
		video->used_views_num);
	for (i = 0; i < vfh->format.fmt.pix_mp.num_planes; ++i) {
		u8 vidx;
		struct isp_view *view;

		vidx = video->used_views_map[i];
		view = &video->views[vidx];
		sizes[i] = view->sizetotal;
		dev_dbg(video->isp->dev,
			"V%02d QSETUP: view: %u, size: %8u, meta: %8u\n",
			video->vid, vidx, sizes[i], view->sizemeta);
		view->used = true;
	}

	*num_planes = vfh->format.fmt.pix_mp.num_planes;
	*count = min(*count, buf_num);

	return 0;
}

static int isp_video_buf_prepare(struct vb2_buffer *buf)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(buf);
	struct isp_buffer *buffer = to_isp_buffer(vbuf);
	struct isp_video_fh *vfh = vb2_get_drv_priv(buf->vb2_queue);
	struct isp_video *video = vfh->video;
	u8 i;

	dev_dbg(video->isp->dev,
		"V%02d QBUFPREPARE: index: %u, memory: %d, ts: %12lluus\n",
		video->vid, buf->index, buf->memory,
		ktime_get_boottime_ns() / 1000);

	for (i = 0; i < vfh->format.fmt.pix_mp.num_planes; ++i) {
		u8 view;
		u32 sizeimage;
		u8 *va;
		dma_addr_t addr;

		view = video->used_views_map[i];
		sizeimage = vfh->format.fmt.pix_mp.plane_fmt[view].sizeimage;
		vb2_set_plane_payload(buf, i, sizeimage);
		addr = vb2_dma_contig_plane_dma_addr(buf, i);
		buffer->dma[view] = addr;
		va = isp_buf_dma_to_va(video->isp, addr);
		if (!virt_addr_valid(va))
			continue;

		dev_dbg(video->isp->dev,
			"V%02d QBUFPREPARE: index: %u, view: %u, size: %8u, dma: 0x%08llX, va: 0x%llX, data: 0x%02X 0x%02X 0x%02X 0x%02X\n",
			video->vid, buf->index, view, sizeimage, addr, (u64)va,
			*va, *(va + 1), *(va + 2), *(va + 3));

		/* See also vb2_dc_prepare */
		if (buf->memory == V4L2_MEMORY_MMAP ||
		    buf->memory == V4L2_MEMORY_DMABUF)
			dma_sync_for_dev(video->isp, buffer->dma[view],
					 buf->planes[i].length, DMA_TO_DEVICE);
	}

	return 0;
}

static void isp_video_buf_finish(struct vb2_buffer *buf)
{
	struct isp_video_fh *vfh = vb2_get_drv_priv(buf->vb2_queue);
	struct isp_video *video = vfh->video;
	u8 i;

	dev_dbg(video->isp->dev,
		"V%02d QBUFFINISH: index: %u, memory: %d, ts: %12lluus\n",
		video->vid, buf->index, buf->memory,
		ktime_get_boottime_ns() / 1000);

	for (i = 0; i < vfh->format.fmt.pix_mp.num_planes; ++i) {
		u8 view_idx;
		struct isp_view *view;
		u8 *va;
		dma_addr_t addr;

		view_idx = video->used_views_map[i];
		view = video->views + view_idx;
		if (view->meta && buf->memory != V4L2_MEMORY_USERPTR) {
			/* Fill meta data */
			struct meta_desc *desc;
			struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(buf);
			struct isp_buffer *isp_buf = to_isp_buffer(vbuf);

			va = vb2_plane_vaddr(buf, i);
			desc = (struct meta_desc *)(va + view->sizeimage);
			desc->snr_exp_us = isp_buf->snr_exp_us;
			desc->isp = video->isp_meta;
			memcpy(&desc->sensor, video->sensor_meta,
			       sizeof(desc->sensor));
		}

		addr = vb2_dma_contig_plane_dma_addr(buf, i);
		va = isp_buf_dma_to_va(video->isp, addr);
		if (!virt_addr_valid(va))
			continue;

		/* See also vb2_dc_finish */
		if (buf->memory == V4L2_MEMORY_MMAP ||
		    buf->memory == V4L2_MEMORY_DMABUF)
			dma_sync_for_cpu(video->isp, addr,
					 buf->planes[i].length,
					 DMA_FROM_DEVICE);
		dev_dbg(video->isp->dev,
			"V%02d QBUFFINISH: index: %u, view: %u, dma: 0x%08llX, va: 0x%llX, data: 0x%02X 0x%02X 0x%02X 0x%02X\n",
			video->vid, buf->index, view_idx, addr, (u64)va, *va,
			*(va + 1), *(va + 2), *(va + 3));
	}
}

static int stream_subdevs(struct isp_video *video, bool enable)
{
	int rv;
	struct isp_channel *channel;
	struct csi_device *csi;

	channel = video->channel;
	mutex_lock(&channel->lock);
	if (channel->on && enable)
		goto done;

	csi = channel->csi_channel->csi_dev;
	rv = v4l2_subdev_call(&csi->subdev, video, s_stream,
			      STREAM_ENC(channel->csi_vc, enable));
	if (rv < 0)
		dev_err(video->isp->dev, "V%02d: Failed to stream %s CSI dev\n",
			video->vid, enable ? "on" : "off");
	else
		channel->on = enable;
done:
	mutex_unlock(&channel->lock);

	return 0;
}

static int isp_video_start_streaming(struct vb2_queue *queue,
				     unsigned int count)
{
	struct isp_video_fh *vfh = vb2_get_drv_priv(queue);
	struct isp_video *video = vfh->video;
	int rv;

	if (video->isp->feed_mode)
		return 0;

	rv = isp_video_streamon_hw(video);
	if (rv) {
		return_buffers(video, VB2_BUF_STATE_QUEUED);
		return rv;
	}

	return stream_subdevs(video, true);
}

static void isp_video_buf_queue(struct vb2_buffer *buf)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(buf);
	struct isp_video_fh *vfh = vb2_get_drv_priv(buf->vb2_queue);
	struct isp_buffer *buffer = to_isp_buffer(vbuf);
	struct isp_video *video = vfh->video;
	struct device *dev = video->isp->dev;

	dev_dbg(dev, "V%02d QBUFQUEUE: index: %u\n", video->vid, buf->index);
	if (unlikely(video->error)) {
		vb2_buffer_done(&buffer->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		return;
	}

	if (video->isp->feed_mode) {
		mutex_lock(&video->wait_buf_lock);
		list_add_tail(&buffer->node, &video->wait_buf_queue);
		mutex_unlock(&video->wait_buf_lock);

		complete(&video->feed_buf_avail_comp);
		return;
	}

	mutex_lock(&video->free_buf_lock);
	list_add_tail(&buffer->node, &video->free_buf_queue);
	mutex_unlock(&video->free_buf_lock);
	if (!video->isp->merge_msg || buf->vb2_queue->start_streaming_called)
		send_next_bufs(video);
}

static const struct vb2_ops isp_video_vb2_ops = {
	.queue_setup = isp_video_queue_setup,
	//.wait_prepare = isp_video_wait_prepare,
	//.wait_finish = isp_video_wait_finish,
	//.buf_out_validate = isp_video_buf_out_validate,
	//.buf_init = isp_video_buf_init,
	.buf_prepare = isp_video_buf_prepare,
	.buf_finish = isp_video_buf_finish,
	//.buf_cleanup = isp_video_buf_cleanup,
	.start_streaming = isp_video_start_streaming,
	//.stop_streaming = isp_video_stop_streaming,
	.buf_queue = isp_video_buf_queue,
	//.buf_request_complete = isp_video_buf_request_complete,
};

/* -----------------------------------------------------------------------------
 * V4L2 ioctl Operations
 */
static int isp_video_querycap(struct file *file, void *fh,
			      struct v4l2_capability *cap)
{
	struct isp_video *video = video_drvdata(file);

	strscpy(cap->driver, ISP_VIDEO_DRIVER_NAME, sizeof(cap->driver));
	strscpy(cap->card, video->video.name, sizeof(cap->card));
	strscpy(cap->bus_info, "media", sizeof(cap->bus_info));

	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE |
			    V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING |
			    V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int isp_video_get_format(struct file *file, void *fh,
				struct v4l2_format *format)
{
	int i;
	struct isp_video *video = video_drvdata(file);

	mutex_lock(&video->lock);
	for (i = 0; i < ARRAY_SIZE(format->fmt.pix_mv); ++i) {
		struct isp_view *view;
		struct v4l2_pix_format *pix;

		pix = &format->fmt.pix_mv[i];
		if (i >= ARRAY_SIZE(video->views)) {
			memset(pix, 0, sizeof(*pix));
			dev_dbg(video->isp->dev,
				"V%02d G_FMT: view %u out of range\n",
				video->vid, i);
			continue;
		}

		view = &video->views[i];
		if (!view->enabled) {
			memset(pix, 0, sizeof(*pix));
			dev_dbg(video->isp->dev,
				"V%02d G_FMT: view %u is disabled\n",
				video->vid, i);
			continue;
		}
		dev_dbg(video->isp->dev,
			"V%02d G_FMT: view %u: w: %4u, h: %4u, f: 0x%08X, id: %2u\n",
			video->vid, i, view->width, view->height,
			view->v4l2_pixel_fmt, view->view_id);

		pix->width = view->width;
		pix->height = view->height;
		pix->pixelformat = view->v4l2_pixel_fmt;
		pix->bytesperline = view->bytesperline;
		pix->sizeimage = view->sizetotal;

		if (view->used)
			pix->flags |= V4L2_PIX_FMT_FLAG_VIEW_OPENED;
		if (view->meta)
			pix->flags |= V4L2_PIX_FMT_FLAG_META_ENABLED;
	}
	mutex_unlock(&video->lock);

	return 0;
}

static int isp_video_get_format_mplane(struct file *file, void *fh,
				       struct v4l2_format *format)
{
	struct isp_video *video = video_drvdata(file);
	struct device *dev = video->isp->dev;
	struct view_format *f;
	int i;

	if (format->type != video->buf_type) {
		dev_err(dev,
			"V%02d G_FMT_MP: invalid buf type: 0x%02X, require: 0x%02X\n",
			video->vid, format->type, video->buf_type);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(video->views); ++i) {
		if (video->views[i].enabled) {
			f = (struct view_format *)&format->fmt.pix_mp
				    .plane_fmt[i];
			f->pixelformat = video->views[i].v4l2_pixel_fmt;
			f->width = video->views[i].width;
			f->height = video->views[i].height;
			dev_dbg(dev, "V%02d G_FMT_MP: view: %u size: %4dx%4d\n",
				video->vid, i, f->width, f->height);
		}
	}

	return 0;
}

static int isp_video_set_format(struct file *file, void *fh,
				struct v4l2_format *format)
{
	int i;
	struct isp_video *video = video_drvdata(file);
	struct isp_video_fh *vfh = to_isp_video_fh(fh);

	mutex_lock(&video->lock);
	video->used_views_num = 0;
	/* 2024-02-20, Oak Chen:
	 * Only permit limited set, that is:
	 * 1. Enable/Disable the view
	 * Then, we override the format with actual set following definition of
	 * VIDIOC_S_FMT.
	 */
	for (i = 0; i < ARRAY_SIZE(format->fmt.pix_mv); ++i) {
		struct isp_view *view;
		struct v4l2_pix_format *pix;

		pix = &format->fmt.pix_mv[i];
		if (i >= ARRAY_SIZE(video->views)) {
			memset(pix, 0, sizeof(*pix));
			dev_dbg(video->isp->dev,
				"V%02d S_FMT: view %u out of range\n",
				video->vid, i);
			continue;
		}

		view = &video->views[i];
		if (!view->enabled) {
			if (pix->pixelformat != 0) {
				mutex_unlock(&video->lock);
				return -EINVAL;
			}
			memset(pix, 0, sizeof(*pix));
			dev_dbg(video->isp->dev,
				"V%02d S_FMT: view %u is disabled\n",
				video->vid, i);
			continue;
		}
		dev_dbg(video->isp->dev,
			"V%02d S_FMT: view %u, fmt: 0x%08X, enabled: %u\n",
			video->vid, i, pix->pixelformat, view->enabled);
		/* Disable view when pixelformat is 0 */
		if (pix->pixelformat == 0) {
			view->used = false;
			continue;
		}
		pix->width = view->width;
		pix->height = view->height;
		pix->pixelformat = view->v4l2_pixel_fmt;
		pix->bytesperline = view->bytesperline;
		pix->sizeimage = view->sizetotal;
		pix->flags |= V4L2_PIX_FMT_FLAG_VIEW_OPENED;
		if (view->meta)
			pix->flags |= V4L2_PIX_FMT_FLAG_META_ENABLED;

		/* Update internal data */
		vfh->format.fmt.pix_mp.plane_fmt[i].sizeimage = view->sizeimage;
		vfh->format.fmt.pix_mp.plane_fmt[i].bytesperline =
			view->bytesperline;
		video->used_views_map[video->used_views_num] = i;
		++video->used_views_num;
	}
	vfh->format.fmt.pix_mp.num_planes = video->used_views_num;
	mutex_unlock(&video->lock);

	return 0;
}

static int isp_video_set_format_mplane(struct file *file, void *fh,
				       struct v4l2_format *format)
{
	struct isp_video *video = video_drvdata(file);
	struct device *dev = video->isp->dev;
	struct isp_video_fh *vfh = to_isp_video_fh(fh);
	u8 view;

	/* Interpret num_planes as view index */
	view = format->fmt.pix_mp.num_planes;
	if (format->type != video->buf_type) {
		dev_err(dev,
			"V%02d S_FMT_MP: invalid buf type: 0x%02X, require: 0x%02X, view: %u\n",
			video->vid, format->type, video->buf_type, view);
		return -EINVAL;
	}

	if (view >= ARRAY_SIZE(video->views)) {
		dev_err(dev, "V%02d S_FMT_MP: invalid view: %u\n", video->vid,
			view);
		return -EINVAL;
	}

	if (!video->views[view].enabled) {
		dev_err(dev, "V%02d S_FMT_MP: disabled view: %u\n", video->vid,
			view);
		return -EINVAL;
	}

	if ((format->fmt.pix_mp.width != video->views[view].width) ||
	    (format->fmt.pix_mp.height != video->views[view].height) ||
	    (format->fmt.pix_mp.pixelformat !=
	     video->views[view].v4l2_pixel_fmt)) {
		dev_err(dev,
			"V%02d S_FMT_MP: view %u, unsupported format: width: %u, height: %u, format: 0x%08X\n",
			video->vid, view, format->fmt.pix_mp.width,
			format->fmt.pix_mp.height,
			format->fmt.pix_mp.pixelformat);
		return -EINVAL;
	}
	mutex_lock(&video->lock);
	vfh->format.fmt.pix_mp.plane_fmt[view].bytesperline =
		video->views[view].bytesperline;
	vfh->format.fmt.pix_mp.plane_fmt[view].sizeimage =
		video->views[view].sizeimage;
	mutex_unlock(&video->lock);

	return 0;
}

static int isp_video_reqbufs(struct file *file, void *fh,
			     struct v4l2_requestbuffers *rb)
{
	struct isp_video *video = video_drvdata(file);
	struct isp_video_fh *vfh = to_isp_video_fh(fh);
	int rv;
	enum v4l2_buf_type type;

	dev_dbg(video->isp->dev,
		"V%02d REQBUFS: count: %u, type: 0x%02X, memory: %u\n",
		video->vid, rb->count, rb->type, rb->memory);
	type = rb->type;
	rb->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

	mutex_lock(&video->vb2_lock);
	rv = vb2_reqbufs(&vfh->queue, rb);
	video->stat.buf_num_app = rb->count;
	mutex_unlock(&video->vb2_lock);
	/* We DO NOT get lock here, since it should not be concurrency */
	INIT_LIST_HEAD(&video->free_buf_queue);
	INIT_LIST_HEAD(&video->wait_buf_queue);

	rb->type = type;

	return rv;
}

static int querybuf_mview(struct isp_video *video, struct vb2_queue *q,
			  struct v4l2_buffer *b)
{
	int rv;
	int v;
	struct v4l2_buffer *views;
	enum v4l2_buf_type type;
	struct v4l2_plane planes[VIDEO_MAX_VIEWS];

	type = b->type;
	views = b->m.views;

	b->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	b->m.planes = planes;
	memset(planes, 0, sizeof(planes));
	rv = vb2_querybuf(q, b);
	b->type = type;
	b->m.views = views;

	for (v = 0; v < b->length; ++v) {
		dev_dbg(video->isp->dev,
			"V%02d QUERYBUF: bytesused: %u/%u, length: %u/%8u, userptr: 0x%012lX/0x%012lX, data_offset: 0x%08X\n",
			video->vid, views[v].bytesused, planes[v].bytesused,
			views[v].length, planes[v].length, views[v].m.userptr,
			planes[v].m.userptr, planes[v].data_offset);
		/* NOTE: Copy the main v4l2_buffer's data to every view? */
		views[v].bytesused = planes[v].bytesused;
		views[v].length = planes[v].length;
		views[v].m.userptr = planes[v].m.userptr;
	}

	return rv;
}

static int isp_video_querybuf(struct file *file, void *fh,
			      struct v4l2_buffer *b)
{
	struct isp_video_fh *vfh = to_isp_video_fh(fh);
	struct isp_video *video = video_drvdata(file);
	int rv;

	dev_dbg(video->isp->dev,
		"V%02d QUERYBUF: index: %u, type: 0x%02X, flags: 0x%08X, memory: %d, length: %u\n",
		video->vid, b->index, b->type, b->flags, b->memory, b->length);
	mutex_lock(&video->vb2_lock);
	if (b->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		rv = vb2_querybuf(&vfh->queue, b);
	else
		rv = querybuf_mview(video, &vfh->queue, b);
	mutex_unlock(&video->vb2_lock);

	return rv;
}

static int qbuf_mview(struct isp_video *video, struct vb2_queue *q,
		      struct media_device *mdev, struct v4l2_buffer *b)
{
	int rv;
	u32 v;
	enum v4l2_buf_type type;
	struct v4l2_buffer *views;
	struct v4l2_plane planes[VIDEO_MAX_VIEWS];

	type = b->type;
	views = b->m.views;

	b->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	b->m.planes = planes;
	memset(planes, 0, sizeof(planes));
	for (v = 0; v < b->length; ++v) {
		dev_dbg(video->isp->dev,
			"V%02d QBUF: view: %u, user_ptr: 0x%012lX, length: %8u\n",
			video->vid, v, views[v].m.userptr, views[v].length);
		planes[v].m.userptr = views[v].m.userptr;
		planes[v].length = views[v].length;
	}

	rv = vb2_qbuf(q, mdev, b);

	b->type = type;
	b->m.views = views;

	return rv;
}

static int isp_video_qbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct isp_video_fh *vfh = to_isp_video_fh(fh);
	struct isp_video *video = video_drvdata(file);
	int rv;

	dev_dbg(video->isp->dev,
		"V%02d QBUF: index: %u, type: 0x%02X, flags: 0x%08X, memory: %d, length: %u, sn: %8u\n",
		video->vid, b->index, b->type, b->flags, b->memory, b->length,
		b->sequence);
	mutex_lock(&video->vb2_lock);
	if (b->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		rv = vb2_qbuf(&vfh->queue, NULL, b);
	else
		rv = qbuf_mview(video, &vfh->queue, NULL, b);
	if (rv == 0) {
		++video->stat.buf_num_drv;
		--video->stat.buf_num_app;
		++video->stat.qbuf_num;
	}
	mutex_unlock(&video->vb2_lock);

	return rv;
}

static int expbuf_mview(struct isp_video *video, struct vb2_queue *q,
			struct v4l2_exportbuffer *p)
{
	int rv;
	enum v4l2_buf_type type;

	type = p->type;
	p->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

	rv = vb2_expbuf(q, p);

	p->type = type;

	return rv;
}

static int isp_video_expbuf(struct file *file, void *fh,
			    struct v4l2_exportbuffer *p)
{
	struct isp_video_fh *vfh = to_isp_video_fh(fh);
	struct isp_video *video = video_drvdata(file);
	int rv;

	dev_dbg(video->isp->dev,
		"V%02d EXPBUF: type: 0x%02X, index: %u, view: %u, flags: 0x%08X\n",
		video->vid, p->type, p->index, p->view, p->flags);
	mutex_lock(&video->vb2_lock);
	if (p->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		rv = vb2_expbuf(&vfh->queue, p);
	else
		rv = expbuf_mview(video, &vfh->queue, p);
	mutex_unlock(&video->vb2_lock);

	return rv;
}

static int dqbuf_mview(struct isp_video *video, struct vb2_queue *q,
		       struct v4l2_buffer *b, bool nonblocking)
{
	int rv;
	int v;
	enum v4l2_buf_type type;
	struct v4l2_buffer *views;
	struct v4l2_plane planes[VIDEO_MAX_VIEWS];

	type = b->type;
	views = b->m.views;

	b->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	b->m.planes = planes;
	memset(planes, 0, sizeof(planes));
	rv = vb2_dqbuf(q, b, nonblocking);

	b->type = type;
	b->m.views = views;

	for (v = 0; v < b->length; ++v) {
		dev_dbg(video->isp->dev,
			"V%02d DQBUF: bytesused: %8u/%8u, length: %8u/%8u, userptr: 0x%012lX/0x%012lX, data_offset: 0x%08X\n",
			video->vid, views[v].bytesused, planes[v].bytesused,
			views[v].length, planes[v].length, views[v].m.userptr,
			planes[v].m.userptr, planes[v].data_offset);
		/* NOTE: Copy the main v4l2_buffer's data to every view? */
		views[v].bytesused = planes[v].bytesused;
		views[v].length = planes[v].length;
		views[v].m.userptr = planes[v].m.userptr;
	}

	return rv;
}

static int isp_video_dqbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct isp_video_fh *vfh = to_isp_video_fh(fh);
	struct isp_video *video = video_drvdata(file);
	int rv;

	mutex_lock(&video->vb2_lock);
	if (video->error) {
		b->abnormal_id = video->fw_ab_info.abnormalId;
		b->flags = V4L2_BUF_FLAG_ERROR;
		b->sequence = video->stat.last_sn_all;
		v4l2_buffer_set_timestamp(b, video->stat.last_ts_all);
		mutex_unlock(&video->vb2_lock);
		dev_dbg(video->isp->dev, "V%02d DQBUF: Error: index: %u\n",
			video->vid, b->index);
		video->error = false;
		return -EIO;
	}

	if (b->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		rv = vb2_dqbuf(&vfh->queue, b, file->f_flags & O_NONBLOCK);
	else
		rv = dqbuf_mview(video, &vfh->queue, b,
				 file->f_flags & O_NONBLOCK);

	if (rv == 0) {
		--video->stat.buf_num_drv;
		++video->stat.buf_num_app;
		++video->stat.dqbuf_num;
		if (!video->isp->feed_mode) {
			struct vb2_buffer *vb = vfh->queue.bufs[b->index];
			struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
			struct isp_buffer *isp_buf = to_isp_buffer(vbuf);

			b->reserved = isp_buf->snr_exp_us;
		}
	}

	mutex_unlock(&video->vb2_lock);
	dev_dbg(video->isp->dev,
		"V%02d DQBUF: index: %u, type: 0x%02X, flags: 0x%08X, memory: %d, length: %u, sn: %8u\n",
		video->vid, b->index, b->type, b->flags, b->memory, b->length,
		b->sequence);

	return rv;
}

static int isp_video_streamon(struct file *file, void *fh,
			      enum v4l2_buf_type type)
{
	struct isp_video_fh *vfh = to_isp_video_fh(fh);
	struct isp_video *video = video_drvdata(file);
	int rv;
	u32 buf_num_drv;
	u32 buf_num_fw;
	u32 buf_num_app;

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	if (type != video->buf_type)
		return -EINVAL;

	mutex_lock(&video->stream_lock);
	/* Set to ON status first, since firmware may send message
	 * before this function exit
	 */
	video->status = VS_STREAM_ON;
	video->vb2_queue = &vfh->queue;

	buf_num_drv = video->stat.buf_num_drv;
	buf_num_fw = video->stat.buf_num_fw;
	buf_num_app = video->stat.buf_num_app;
	/* Do this again for stream off and on during runtime */
	memset(&video->stat, 0, sizeof(video->stat));
	memset(&video->fw_ab_info, 0, sizeof(video->fw_ab_info));
	video->stat.ts_streamon = ktime_get_boottime_ns();
	video->stat.buf_num_drv = buf_num_drv;
	video->stat.buf_num_fw = buf_num_fw;
	video->stat.buf_num_app = buf_num_app;

	mutex_lock(&video->vb2_lock);
	rv = vb2_streamon(&vfh->queue, type);
	mutex_unlock(&video->vb2_lock);
	if (rv < 0)
		goto err;

	mutex_unlock(&video->stream_lock);
	return 0;

err:
	video->vb2_queue = NULL;
	video->status = VS_OPENED;
	mutex_unlock(&video->stream_lock);
	dev_dbg(video->isp->dev, "V%02d STREAMON\n", video->vid);

	return rv;
}

static int isp_video_streamoff(struct file *file, void *fh,
			       enum v4l2_buf_type type)
{
	struct isp_video_fh *vfh = to_isp_video_fh(fh);
	struct isp_video *video = video_drvdata(file);
	unsigned int streaming;

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

	mutex_lock(&video->stream_lock);
	video->status = VS_STREAM_OFF;

	/* Make sure we're not streaming yet. */
	mutex_lock(&video->vb2_lock);
	streaming = vb2_is_streaming(&vfh->queue);
	mutex_unlock(&video->vb2_lock);

	if (!streaming)
		goto done;

	isp_video_streamoff_hw(video);
	stream_subdevs(video, false);
	if (!video->isp->feed_mode)
		wait_bufs_idle(video);
	/*
	 * Returns all buffers queued on the video node to videobuf2
	 * in the erroneous state and makes sure no new buffer can be queued.
	 */
	return_buffers(video, VB2_BUF_STATE_ERROR);

	mutex_lock(&video->vb2_lock);
	vb2_streamoff(&vfh->queue, type);
	video->vb2_queue = NULL;
	mutex_unlock(&video->vb2_lock);

	video->error = false;

done:
	mutex_unlock(&video->stream_lock);
	dev_dbg(video->isp->dev, "V%02d STREAMOFF\n", video->vid);

	return 0;
}

static int isp_video_enum_input(struct file *file, void *fh,
				struct v4l2_input *input)
{
	if (input->index > 0)
		return -EINVAL;
	strscpy(input->name, "camera", sizeof(input->name));
	input->type = V4L2_INPUT_TYPE_CAMERA;

	return 0;
}

static int isp_video_g_input(struct file *file, void *fh, unsigned int *input)
{
	*input = 0;

	return 0;
}

static int isp_video_s_input(struct file *file, void *fh, unsigned int input)
{
	return input == 0 ? 0 : -EINVAL;
}

static int get_abnormal_info(struct isp_video *video,
			     struct abnormal_info *info)
{
	info->id = video->fw_ab_info.abnormalId;
	info->type = video->fw_ab_info.abnormalType;
	info->last_good_sn = video->stat.last_sn_app;
	info->total_bad_frames = video->stat.rx_msg_bad;
	info->total_frames = video->stat.rx_buf_all;
	video->error = false;

	return 0;
}

static int get_camera_info(struct isp_video *video, struct camera_info *info)
{
	ipc_reconf_t *cfg;

	cfg = video->channel->cfg;
	/* TODO: Missing name, id, or remove this deprecated API */
	info->data_type = cfg->rawinfo.dataType;
	info->fps = video->channel->cam_dev->sensor_fps;
	info->raw_width = cfg->rawinfo.width;
	info->raw_height = cfg->rawinfo.height;

	return 0;
}

static int get_meta_info(struct isp_video *video,
			 struct embedded_view_info *view_info)
{
	ipc_reconf_t *cfg;
	struct embedded_info *info;
	int i;

	memset(view_info, 0, sizeof(*view_info));
	info = &view_info->embedded_info;
	cfg = video->channel->cfg;
	for (i = 0; i < ARRAY_SIZE(cfg->embeddedInfo); ++i) {
		int j;
		struct isp_view *view;

		info->line_start[i] = cfg->embeddedInfo[i].line_start;
		info->line_num[i] = cfg->embeddedInfo[i].line_end -
				    cfg->embeddedInfo[i].line_start;
		for (j = 0; j < ARRAY_SIZE(info->emd_zone_offset[0]); ++j) {
			view = &video->views[j];
			if (view->meta) {
				view_info->embedded_view = j;
				info->emd_zone_offset[i][j] =
					view->sizeimage +
					video->sensor_meta[i].offset;
			}
		}
	}

	return 0;
}

static long isp_video_default(struct file *file, void *fh, bool valid_prio,
			      unsigned int cmd, void *arg)
{
	struct isp_video *video = video_drvdata(file);

	switch (cmd) {
	case ISPIOC_G_ABNORMAL_INFO:
		return get_abnormal_info(video, (struct abnormal_info *)arg);
	case ISPIOC_G_CAMERA_INFO:
		return get_camera_info(video, (struct camera_info *)arg);
	case ISPIOC_G_EMBEDDED_INFO:
		return get_meta_info(video, (struct embedded_view_info *)arg);
	case ISPIOC_G_DATA_MODE:
		*(int *)arg = video->isp->feed_mode;
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct v4l2_ioctl_ops isp_video_ioctl_ops = {
	.vidioc_querycap = isp_video_querycap,

	.vidioc_g_fmt_vid_cap = isp_video_get_format,
	.vidioc_g_fmt_vid_cap_mplane = isp_video_get_format_mplane,

	.vidioc_s_fmt_vid_cap = isp_video_set_format,
	.vidioc_s_fmt_vid_cap_mplane = isp_video_set_format_mplane,

	.vidioc_reqbufs = isp_video_reqbufs,
	.vidioc_querybuf = isp_video_querybuf,
	.vidioc_qbuf = isp_video_qbuf,
	.vidioc_expbuf = isp_video_expbuf,
	.vidioc_dqbuf = isp_video_dqbuf,

	.vidioc_streamon = isp_video_streamon,
	.vidioc_streamoff = isp_video_streamoff,

	.vidioc_enum_input = isp_video_enum_input,
	.vidioc_g_input = isp_video_g_input,
	.vidioc_s_input = isp_video_s_input,

	.vidioc_default = isp_video_default,
};

/* -----------------------------------------------------------------------------
 * V4L2 file operations
 */
static __poll_t isp_video_poll(struct file *file, poll_table *wait)
{
	struct isp_video_fh *vfh = to_isp_video_fh(file->private_data);
	struct isp_video *video = video_drvdata(file);
	__poll_t rv;

	mutex_lock(&video->vb2_lock);
	rv = vb2_poll(&vfh->queue, file, wait);
	/* If this is waked by abnormal message */
	if (video->error)
		rv = EPOLLERR | EPOLLNVAL;
	mutex_unlock(&video->vb2_lock);

	return rv;
}

static int isp_video_mmap(struct file *file, struct vm_area_struct *vma)
{
	int rv;
	struct isp_video *video = video_drvdata(file);
	struct isp_video_fh *vfh = to_isp_video_fh(file->private_data);
	struct vb2_queue *q = &vfh->queue;

	dev_dbg(q->dev,
		"V%02d MMAP: start: 0x%08lX, pgoff: 0x%08lX, size: 0x%08lX, dma_coherent: %d\n",
		video->vid, vma->vm_start, vma->vm_pgoff,
		vma->vm_end - vma->vm_start, dev_is_dma_coherent(q->dev));

	rv = vb2_mmap(&vfh->queue, vma);
	if (rv)
		dev_err(q->dev, "V%02d: Failed to mmap\n", video->vid);

	return rv;
}

static int isp_video_open(struct file *file)
{
	struct isp_video *video = video_drvdata(file);
	struct device *dev = video->isp->dev;
	int vid = video->vid;
	int rv = 0;
	struct isp_video_fh *handle;
	struct vb2_queue *queue;
	char work_queue_name[64];
	int default_view = -1;
	struct isp_view *view;
	int i;

	/* Init firmware first */
	rv = isp_fw_init(video->isp);
	if (rv) {
		if (rv != -EACCES)
			dev_err(dev,
				"V%02d OPEN: Failed to init firmware, rv: %d\n",
				vid, rv);
		goto err_fw_boot;
	}

	isp_update_channels_cfg(video->isp);

	mutex_lock(&(video->lock));
	dev_dbg(dev, "V%02d OPEN: status: %s\n", vid, isp_video_status(video));
	if (video->status > VS_UNUSED) {
		dev_err(dev, "V%02d OPEN: Video is busy\n", vid);
		rv = -EBUSY;
		goto err_dev_busy;
	}

	/* Set default view to first enabled view */
	for (i = ARRAY_SIZE(video->views) - 1; i >= 0; --i) {
		if (video->views[i].enabled)
			default_view = i;
		video->views[i].used = false;
	}
	if (default_view < 0) {
		dev_err(dev, "V%02d OPEN: No enabled view\n", vid);
		rv = -EINVAL;
		goto err_no_view_enabled;
	}
	view = &video->views[default_view];

	/* We have found the video and views are enabled, init others */
	handle = devm_kzalloc(dev, sizeof(*handle), GFP_KERNEL);
	if (handle == NULL) {
		rv = -ENOMEM;
		goto err_no_mem;
	}

	/* We customize the v4l2_fh, so initialize it manually */
	v4l2_fh_init(&handle->vfh, &video->video);
	v4l2_fh_add(&handle->vfh);
	handle->video = video;

	queue = &handle->queue;
	queue->type = video->buf_type;
	queue->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	queue->dev = video->isp->dev;
	queue->dma_attrs = DMA_ATTR_FORCE_CONTIGUOUS;
	queue->bidirectional = 1;
	queue->ops = &isp_video_vb2_ops;
	queue->mem_ops = &vb2_dma_contig_memops;
	queue->drv_priv = handle;
	queue->buf_struct_size = sizeof(struct isp_buffer);
	queue->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	queue->min_buffers_needed = MIN_FRAME_BUFFER_NUM;
	rv = vb2_queue_init(&handle->queue);
	if (rv < 0) {
		dev_err(dev, "V%02d OPEN: Failed to init vb2 queue, rv: %d\n",
			vid, rv);
		goto err_vb2_queue_init;
	}

	handle->format.type = video->buf_type;
	handle->format.fmt.pix_mp.width = view->width;
	handle->format.fmt.pix_mp.height = view->height;
	handle->format.fmt.pix_mp.pixelformat = view->v4l2_pixel_fmt;
	handle->format.fmt.pix_mp.field = V4L2_FIELD_ANY;
	handle->format.fmt.pix_mp.colorspace = V4L2_COLORSPACE_DEFAULT;
	handle->format.fmt.pix_mp.plane_fmt[0].bytesperline =
		view->bytesperline;
	handle->format.fmt.pix_mp.plane_fmt[0].sizeimage = view->sizeimage;
	handle->format.fmt.pix_mp.num_planes = 1;

	memset(video->used_views_map, 0, sizeof(video->used_views_map));
	video->used_views_map[0] = default_view;
	video->used_views_num = 1;

	snprintf(work_queue_name, sizeof(work_queue_name), "V%02d-msg-handle",
		 video->vid);
	mutex_lock(&video->todo_msg_lock);
	video->todo_msg_queue = create_singlethread_workqueue(work_queue_name);
	if (video->todo_msg_queue == NULL) {
		dev_err(dev,
			"V%02d: Failed to create message handle work queue\n",
			vid);
		rv = -ENOMEM;
		mutex_unlock(&video->todo_msg_lock);
		goto err_create_wq;
	}
	mutex_unlock(&video->todo_msg_lock);

	mutex_lock(&video->free_msg_lock);
	INIT_LIST_HEAD(&video->free_msg_queue);
	for (i = 0; i < ARRAY_SIZE(video->cache_msg); ++i)
		list_add_tail(&video->cache_msg[i].node,
			      &video->free_msg_queue);
	mutex_unlock(&video->free_msg_lock);

	video->status = VS_OPENED;
	file->private_data = &handle->vfh;
	memset(&video->stat, 0, sizeof(video->stat));
	memset(&video->fw_ab_info, 0, sizeof(video->fw_ab_info));
	mutex_unlock(&(video->lock));

	return 0;

err_create_wq:
err_vb2_queue_init:
	v4l2_fh_del(&handle->vfh);
	v4l2_fh_exit(&handle->vfh);
	devm_kfree(dev, handle);
err_no_mem:
err_no_view_enabled:
err_dev_busy:
	mutex_unlock(&(video->lock));

err_fw_boot:
	return rv;
}

static int isp_video_release(struct file *file)
{
	struct isp_video *video = video_drvdata(file);
	struct v4l2_fh *vfh = file->private_data;
	struct isp_video_fh *handle = to_isp_video_fh(vfh);

	mutex_lock(&(video->lock));
	if (video->status == VS_STREAM_ON) {
		/* Disable streaming and free the buffers queue resources. */
		isp_video_streamoff(file, vfh, video->buf_type);
	}

	mutex_lock(&video->todo_msg_lock);
	flush_workqueue(video->todo_msg_queue);
	destroy_workqueue(video->todo_msg_queue);
	video->todo_msg_queue = NULL;
	mutex_unlock(&video->todo_msg_lock);

	mutex_lock(&video->vb2_lock);
	vb2_queue_release(&handle->queue);
	mutex_unlock(&video->vb2_lock);

	/* Release the file handle. */
	v4l2_fh_del(vfh);
	v4l2_fh_exit(vfh);
	devm_kfree(video->isp->dev, handle);
	file->private_data = NULL;
	video->status = VS_UNUSED;
	mutex_unlock(&(video->lock));

	return 0;
}

static const struct v4l2_file_operations isp_video_fops = {
	.owner = THIS_MODULE,
	.poll = isp_video_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = isp_video_mmap,
	.open = isp_video_open,
	.release = isp_video_release,
};

/* -----------------------------------------------------------------------------
 * Exported functions
 */
void isp_video_init(struct isp_video *video, struct isp_channel *channel,
		    enum isp_video_type video_type)
{
	int i;
	int cid;

	/* Common and flags */
	mutex_init(&video->lock);
	mutex_init(&video->stream_lock);
	video->isp = channel->isp;
	video->channel = channel;

	cid = channel->cid;
	if (video_type == RAW_VIDEO) {
		video->is_raw = true;
		snprintf(video->video.name, sizeof(video->video.name),
			 "isp-channel-%d-raw-video", cid);
		video->vid = cid + MAX_ISP_CHANNEL;
		video->is_raw = true;
	} else {
		video = &(channel->views_video);
		snprintf(video->video.name, sizeof(video->video.name),
			 "isp-channel-%d-views-video", cid);
		video->vid = cid;
	}

	video->cid = cid;
	video->status = VS_UNUSED;

	/* V4L2 */
	video->video.fops = &isp_video_fops;
	video->video.vfl_type = VFL_TYPE_VIDEO;
	video->video.release = video_device_release_empty;
	video->video.ioctl_ops = &isp_video_ioctl_ops;
	video->video.device_caps = V4L2_CAP_VIDEO_CAPTURE |
				   V4L2_CAP_VIDEO_CAPTURE_MPLANE |
				   V4L2_CAP_STREAMING;
	video_set_drvdata(&video->video, video);
	video->buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	mutex_init(&video->vb2_lock);

	/* Buffer management */
	mutex_init(&video->free_buf_lock);
	mutex_init(&video->wait_buf_lock);

	/* Message */
	mutex_init(&video->free_msg_lock);
	mutex_init(&video->todo_msg_lock);
	for (i = 0; i < ARRAY_SIZE(video->cache_msg); ++i) {
		INIT_WORK(&video->cache_msg[i].msg_work, handle_msg_entry);
		video->cache_msg[i].video = video;
	}

	/* Misc */
	init_completion(&video->feed_buf_avail_comp);
}

void isp_video_cleanup(struct isp_video *video)
{
	mutex_destroy(&video->todo_msg_lock);
	mutex_destroy(&video->free_msg_lock);
	mutex_destroy(&video->wait_buf_lock);
	mutex_destroy(&video->free_buf_lock);
	mutex_destroy(&video->vb2_lock);
	mutex_destroy(&video->stream_lock);
	mutex_destroy(&video->lock);
}

static ssize_t buffer_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct video_device *vdev;
	struct isp_video *video;
	struct vb2_queue *q;
	int len;
	unsigned int i;

	vdev = container_of(dev, struct video_device, dev);
	video = container_of(vdev, struct isp_video, video);
	q = video->vb2_queue;

	if (q == NULL)
		return snprintf(buf, PAGE_SIZE, "Not running\n");

	len = 0;
	len += snprintf(buf + len, PAGE_SIZE - len,
			"%2s | pa iova/dma-start end sizeimage sizetotal\n",
			"id");
	for (i = 0; i < q->num_buffers; ++i) {
		int j;
		struct vb2_buffer *vb = q->bufs[i];
		struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
		struct isp_buffer *isp_buf = to_isp_buffer(vbuf);

		len += snprintf(buf + len, PAGE_SIZE - len, "%2d", i);
		for (j = 0; j < ARRAY_SIZE(isp_buf->dma); ++j) {
			u32 dma;
			phys_addr_t pa;

			dma = isp_buf->dma[j];
			if (!dma)
				pa = 0;
			else if (video->isp->iommud)
				pa = video->isp->iommud->ops->iova_to_phys(
					video->isp->iommud, dma);
			else
				pa = dma_to_phys(video->isp->dev, dma);

			len += snprintf(
				buf + len, PAGE_SIZE - len,
				" | 0x%llX 0x%X 0x%X %u %u", pa, dma,
				dma ? (dma + video->views[j].sizetotal - 1) : 0,
				video->views[j].sizeimage,
				video->views[j].sizetotal);
		}
		len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	}

	return len;
}

static struct device_attribute buffer_attr =
	__ATTR(buffer, 0444, buffer_show, NULL);

static ssize_t cfg_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct video_device *vdev;
	struct isp_video *video;
	int len;
	struct isp_channel *channel;
	ipc_reconf_t *cfg;

	vdev = container_of(dev, struct video_device, dev);
	video = container_of(vdev, struct isp_video, video);
	channel = video->channel;
	cfg = channel->cfg;

	len = 0;
	len += snprintf(buf + len, PAGE_SIZE - len, "%-32s: 0x%08X\n",
			"I2C controller", cfg->i2cRegBase);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-32s: 0x%02X\n",
			"I2C address", cfg->sensorDevID);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-32s: %u\n",
			"I2C rw mode", cfg->sensorRdWrMode);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-32s: %u\n", "CSI ID",
			cfg->mipiSensorIndex);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-32s: %u\n", "Online",
			cfg->sensorOnline);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-32s: 0x%02X, %ux%u\n",
			"RAW dt, size", cfg->rawinfo.dataType,
			cfg->rawinfo.width, cfg->rawinfo.height);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-32s: %u-%u, %u-%u\n",
			"Input crop(T-B, L-R)", cfg->ispInTopCrop,
			cfg->ispInBotCrop, cfg->ispInLefCrop,
			cfg->ispInRigCrop);
	len += snprintf(buf + len, PAGE_SIZE - len,
			"%-32s: 0x%02X, %2u, %ux%u\n", "View 0 dt, align, size",
			cfg->viewinfo[0].viewFmt, cfg->viewinfo[0].lineAlign,
			cfg->viewinfo[0].width, cfg->viewinfo[0].height);
	len += snprintf(buf + len, PAGE_SIZE - len,
			"%-32s: 0x%02X, %2u, %ux%u\n", "View 1 dt, align, size",
			cfg->viewinfo[1].viewFmt, cfg->viewinfo[1].lineAlign,
			cfg->viewinfo[1].width, cfg->viewinfo[1].height);
	len += snprintf(buf + len, PAGE_SIZE - len,
			"%-32s: 0x%02X, %2u, %ux%u\n", "View 2 dt, align, size",
			cfg->viewinfo[2].viewFmt, cfg->viewinfo[2].lineAlign,
			cfg->viewinfo[2].width, cfg->viewinfo[2].height);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-32s: %u, %2u\n",
			"Semaphore <bank, ID>", cfg->semBank, cfg->semId);

	return len;
}

static struct device_attribute cfg_attr = __ATTR(cfg, 0444, cfg_show, NULL);

static ssize_t stat_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct video_device *vdev;
	struct isp_video *video;
	struct video_stat stat;
	int len;
	u64 curr;
	u32 diff_sec;

	vdev = container_of(dev, struct video_device, dev);
	video = container_of(vdev, struct isp_video, video);

	memcpy(&stat, &video->stat, sizeof(stat));
	len = 0;
	curr = ktime_get_boottime_ns();
	diff_sec = ((curr - stat.ts_streamon)) / NS_PER_SECOND;
	len += snprintf(buf + len, PAGE_SIZE - len, "%-15s: %10u/%u\n",
			"rx buf all/fps", stat.rx_buf_all,
			DIV_ROUND_CLOSEST(stat.rx_buf_all, diff_sec));
	len += snprintf(buf + len, PAGE_SIZE - len, "%-15s: %10u\n",
			"rx buf rsv", stat.rx_buf_rsv);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-15s: %10u\n",
			"rx msg bad", stat.rx_msg_bad);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-15s: %10u/%u\n",
			"tx buf done/fps", stat.tx_buf_done,
			DIV_ROUND_CLOSEST(stat.tx_buf_done, diff_sec));
	len += snprintf(buf + len, PAGE_SIZE - len, "%-15s: %10u\n",
			"tx buf fail", stat.tx_buf_fail);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-15s: %10u\n",
			"last sn all", stat.last_sn_all);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-15s: %10u\n",
			"last sn app", stat.last_sn_app);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-15s: %10lluus\n",
			"last ts all", stat.last_ts_all / 1000);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-15s: %10lluus\n",
			"last ts app", stat.last_ts_app / 1000);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-15s: %10u\n",
			"buf num drv", stat.buf_num_drv - stat.buf_num_fw);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-15s: %10u\n",
			"buf num fw", stat.buf_num_fw);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-15s: %10u\n",
			"buf num app", stat.buf_num_app);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-15s: %10lluus\n",
			"stream on ts", stat.ts_streamon / 1000);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-15s: %10u/%u\n",
			"qbuf num/fps", stat.qbuf_num,
			DIV_ROUND_CLOSEST(stat.qbuf_num, diff_sec));
	len += snprintf(buf + len, PAGE_SIZE - len, "%-15s: %10u/%u\n",
			"dqbuf num/fps", stat.dqbuf_num,
			DIV_ROUND_CLOSEST(stat.dqbuf_num, diff_sec));
	len += snprintf(buf + len, PAGE_SIZE - len, "%-15s: %10u\n", "dup num",
			stat.dup_num);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-15s: %10u\n", "drop num",
			stat.drop_num);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-15s: %10u\n",
			"reverse num", stat.reverse_num);

	return len;
}

static struct device_attribute stat_attr = __ATTR(stat, 0444, stat_show, NULL);

static struct attribute *video_attrs[] = {
	&buffer_attr.attr,
	&cfg_attr.attr,
	&stat_attr.attr,
	NULL
};

static const struct attribute_group video_attr_group = {
	.attrs = video_attrs,
};

int isp_video_register(struct isp_video *video, struct v4l2_device *vdev)
{
	int rv;
	struct device *dev;

	dev = video->isp->dev;
	video->video.v4l2_dev = vdev;
	rv = video_register_device(&video->video, VFL_TYPE_VIDEO, video->vid);
	if (rv < 0)
		dev_err(dev, "V%02d: Failed to register video device, rv: %d\n",
			video->vid, rv);
	else
		dev_dbg(dev, "V%02d: Registered\n", video->vid);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
	devm_device_add_group(&video->video.dev, &video_attr_group);
#pragma GCC diagnostic pop

	return rv;
}

void isp_video_unregister(struct isp_video *video)
{
	if (video_is_registered(&video->video))
		video_unregister_device(&video->video);
}

int isp_video_dispatch_msg(struct isp_video *video, struct media_command *mc,
			   u64 ts)
{
	int rv;
	struct video_msg *msg;
	u16 cmd_main;
	u16 cmd_minor;
	u16 exp_lines;
	u64 gtc;
	u32 *data;

	mutex_lock(&video->free_msg_lock);
	if (list_empty(&video->free_msg_queue)) {
		static DEFINE_RATELIMIT_STATE(_rs, ISP_RATELIMIT_INTERVAL,
					      ISP_RATELIMIT_BURST);

		mutex_unlock(&video->free_msg_lock);
		if (__ratelimit(&_rs))
			dev_err(video->isp->dev, "V%02d: NO free msg element\n",
				video->vid);
		return -1;
	}
	msg = list_first_entry(&video->free_msg_queue, struct video_msg, node);
	list_del(&msg->node);
	mutex_unlock(&video->free_msg_lock);

	/* Convert message */
	cmd_main = mc->cmd_hdr.hdr_info.cmd_type_main;
	cmd_minor = mc->cmd_hdr.hdr_info.cmd_type_minor;
	exp_lines = mc->cmd_hdr.magic.exp_lines;
	gtc = mc->cmd_hdr.gtc;

	/* The msg is existent before removing driver, we can use it safely */
	msg->cmd_main = cmd_main;
	msg->cmd_minor = cmd_minor;
	msg->exp_lines = exp_lines;
#ifdef CONFIG_BST_GTC
	msg->timestmap = bst_gtc_cnt_to_sys_mono(gtc);
#else
	msg->timestmap = GTC_TO_NS(gtc);
#endif
	memcpy(&msg->data, mc->user_cmd_data, sizeof(msg->data));
	data = msg->data.data;

	dev_dbg(video->isp->dev,
		"V%02d MSG: main: 0x%02X, minor: 0x%02X, data: 0x%08X, 0x%08X, 0x%08X, 0x%08X, gtc: %llu, exp_lines: %4u\n",
		video->vid, cmd_main, cmd_minor, data[0], data[1], data[2],
		data[3], gtc, exp_lines);

	mutex_lock(&video->todo_msg_lock);
	if (video->todo_msg_queue != NULL) {
		/* todo_msg_queue is valid to handle message */
		queue_work(video->todo_msg_queue, &msg->msg_work);
		rv = 0;
	} else {
		/* todo_msg_queue is invalid, video is closing or closed */
		mutex_lock(&video->free_msg_lock);
		list_add_tail(&msg->node, &video->free_msg_queue);
		mutex_unlock(&video->free_msg_lock);
		dev_dbg(video->isp->dev,
			"V%02d: can not handle message, status: %s\n",
			video->vid, isp_video_status(video));
		rv = -1;
	}
	mutex_unlock(&video->todo_msg_lock);

	return rv;
}

static void update_raw_cfg(struct isp_video *video, ipc_reconf_t *cfg)
{
	int dt;
	raw_cfg_t *raw_cfg;
	struct isp_view *view;

	/* RAW video only has view 0 only */
	video->views[1].enabled = false;
	video->views[2].enabled = false;

	raw_cfg = &cfg->rawinfo;
	dt = raw_cfg->dataType;
	if (!dt) {
		video->views[0].enabled = false;
		return;
	}
	video->is_raw = true;
	view = &video->views[0];
	view->enabled = true;
	view->view_id = video->cid + VIEW_ID_CHANNEL0_RAW0;
	view->v4l2_pixel_fmt = dt_to_v4l2_pix_format(dt);
	view->width = raw_cfg->width;
	view->height = raw_cfg->height;
	view->bytesperline = get_raw_stride(view->v4l2_pixel_fmt, view->width,
					    ISP_BURST_BYTES, ISP_PACK_BYTES);
	view->sizeimage = view->bytesperline * view->height;
	view->sizetotal = ISP_BUFFER_ALIGN(view->sizeimage);
	dev_dbg(video->isp->dev, "V%02d UPVIEW: size: %8u/%8u\n", video->vid,
		view->bytesperline, view->sizeimage);
}

/*
 * Update video's meta info, should be called after video cfgs are updated
 *
 * @video:	ISP video.
 * @cfg:	ISP config data.
 */
static void update_meta_info(struct isp_video *video, ipc_reconf_t *cfg)
{
	struct meta_zone *zone;
	u32 offset;
	struct isp_video *raw_video;
	struct isp_view *view;
	int i;

	raw_video = &video->channel->raw_video;

	zone = &video->isp_meta;
	offset = sizeof(struct meta_desc);
	zone->offset = offset;
	zone->size = ISP_META_BYTES;
	offset += ISP_META_BYTES;

	/* TODO: Fixed to view 1 by firmware */
	view = &video->views[1];
	view->meta = true;

	for (i = 0; i < ARRAY_SIZE(cfg->embeddedInfo); ++i) {
		u32 lines;
		u32 bytes;

		zone = &video->sensor_meta[i];
		lines = cfg->embeddedInfo[i].line_end -
			cfg->embeddedInfo[i].line_start;
		dev_dbg(video->isp->dev,
			"V%02d UPMETA: lines: %u, raw fmt: 0x%08X, width: %4u\n",
			video->vid, lines, raw_video->views[0].v4l2_pixel_fmt,
			raw_video->views[0].width);
		if (lines) {
			bytes = raw_video->views[0].bytesperline * lines;
			zone->offset = offset;
			zone->size = bytes;
			offset += bytes;
		} else {
			zone->offset = 0;
			zone->size = 0;
		}
		dev_dbg(video->isp->dev,
			"V%02d UPMETA: index: %u, offset: %u, size: %u\n",
			video->vid, i, zone->offset, zone->size);
	}
	view->sizemeta = get_meta_bytes(video);
	view->sizetotal = ISP_BUFFER_ALIGN(view->sizeimage + view->sizemeta);
}

static void update_view_cfg(struct isp_video *video, ipc_reconf_t *cfg)
{
	int i;

	video->mirror_raw = true;
	for (i = 0; i < ARRAY_SIZE(video->views); ++i) {
		int view_fmt;
		view_cfg_t *view_cfg;
		struct isp_view *view;

		view = &video->views[i];
		view_cfg = cfg->viewinfo;
		view_fmt = view_cfg[i].viewFmt;
		if (view_fmt == View0_Dis) {
			view->enabled = false;
			continue;
		}
		video->mirror_raw = false;
		view->enabled = true;
		view->view_id = ((video->cid * VIEW_ID_NUM_PER_CAMERA) +
				 VIEW_ID_CHANNEL0_VIEW0 + i);
		view->v4l2_pixel_fmt = view_fmt_to_v4l2_pix_format(view_fmt);
		view->width = view_cfg[i].width;
		view->height = view_cfg[i].height;
		view->bytesperline = get_view_stride(
			view->v4l2_pixel_fmt, view->width,
			view_cfg[i].lineAlign ? view_cfg[i].lineAlign :
						ISP_BURST_BYTES);
		view->sizeimage =
			view->bytesperline *
			get_plane_lines(view->v4l2_pixel_fmt, view->height);
		view->sizetotal = ISP_BUFFER_ALIGN(view->sizeimage);
		dev_dbg(video->isp->dev, "V%02d UPVIEW: size: %8u/%8u\n",
			video->vid, view->bytesperline, view->sizeimage);
	}
	if (video->mirror_raw) {
		video->is_raw = true;
		memcpy(&video->views[0], &video->channel->raw_video.views[0],
		       sizeof(video->views[0]));
	}

	update_meta_info(video, cfg);
}

void isp_video_update_cfg(struct isp_channel *channel)
{
	struct device *dev;
	ipc_reconf_t *cfg;

	dev = channel->isp->dev;
	cfg = channel->cfg;

	dev_dbg(dev,
		"IRB: 0x%08X, MSI: %u, SI: %u, SD: 0x%02X, SM: %u, SO: %u, LTMC: %u, %u, AA: 0x%08X, AS: %u, IA: 0x%08X, IS: %u, ISPIC: %u, %u, %u, %u\n",
		cfg->i2cRegBase, cfg->mipiSensorIndex, cfg->sensorIndex,
		cfg->sensorDevID, cfg->sensorRdWrMode, cfg->sensorOnline,
		cfg->ltmVinTopCrop, cfg->ltmVinBotCrop, cfg->algo_addr,
		cfg->algo_size, cfg->iq_addr, cfg->iq_size, cfg->ispInTopCrop,
		cfg->ispInBotCrop, cfg->ispInLefCrop, cfg->ispInRigCrop);
	/* NOTE: Must keep this order, since we use the raw video
	 * to caculate meta zone.
	 */
	update_raw_cfg(&channel->raw_video, cfg);
	update_view_cfg(&channel->views_video, cfg);
}

const char *isp_video_status(struct isp_video *video)
{
	switch (video->status) {
	case VS_UNUSED:
		return "unused";
	case VS_OPENED:
		return "opened";
	case VS_STREAM_ON:
		return "streamon";
	case VS_STREAM_OFF:
		return "streamoff";
	default:
		return "invalid";
	}
}

void isp_video_recycle_wait_bufs(struct isp_video *video)
{
	struct isp_buffer *buf;
	struct isp_buffer *tmp;

	mutex_lock(&video->free_buf_lock);
	mutex_lock(&video->wait_buf_lock);
	list_for_each_entry_safe(buf, tmp, &video->wait_buf_queue, node) {
		list_del(&buf->node);
		list_add_tail(&buf->node, &video->free_buf_queue);
		--video->stat.buf_num_fw;
	}
	mutex_unlock(&video->wait_buf_lock);
	mutex_unlock(&video->free_buf_lock);
}

int isp_video_streamon_hw(struct isp_video *video)
{
	int tries;
	int rv;

	if (video->isp->feed_mode || !video->channel->cfg->sensorOnline ||
	    video->hw_on)
		return 0;

	for (tries = 1; tries <= ISP_MSG_NOACK_TRY_TIMES; ++tries) {
		if (video->is_raw)
			rv = isp_msg_video_open_raw(video);
		else
			rv = isp_msg_video_open(video);
		if (rv != -ETIMEDOUT)
			break;
	}
	if (rv < 0)
		dev_err(video->isp->dev,
			"V%02d: Failed to notify firmware start stream, tries: %d, rv: %d\n",
			video->vid, tries, rv);
	else
		video->hw_on = 1;

	return rv;
}

int isp_video_streamoff_hw(struct isp_video *video)
{
	int rv;
	int tries;

	if (video->isp->feed_mode || !video->hw_on)
		return 0;

	for (tries = 1; tries <= ISP_MSG_NOACK_TRY_TIMES; ++tries) {
		if (video->is_raw)
			rv = isp_msg_video_close_raw(video);
		else
			rv = isp_msg_video_close(video);
		if (rv != -ETIMEDOUT)
			break;
	}
	if (rv < 0)
		dev_err(video->isp->dev,
			"V%02d: Failed to notify firmware stop stream, tries: %d, rv: %d\n",
			video->vid, tries, rv);
	else
		video->hw_on = 0;

	return rv;
}
