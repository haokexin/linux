// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 * Copyright (C) 2013-2015 Ideas on Board
 * Copyright (C) 2013-2015 Xilinx, Inc.
 */

#include <linux/list.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <media/v4l2-dev.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-dma-sg.h>

#include "csi_tx.h"
#include "csitx_video.h"

/* -----------------------------------------------------------------------------
 * V4L2 ioctls
 */

static int
csitx_dma_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	struct v4l2_fh *vfh = file->private_data;
	struct bst_csitx_video *tx_video = to_csitx_video(vfh->vdev);

	cap->capabilities = V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_STREAMING |
			    V4L2_CAP_DEVICE_CAPS;
	strscpy(cap->driver, "bst_csitx", sizeof(cap->driver));
	strscpy(cap->card, tx_video->video.name, sizeof(cap->card));

	return 0;
}

static int
csitx_dma_enum_format(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	struct v4l2_fh *vfh = file->private_data;
	struct bst_csitx_video *tx_video = to_csitx_video(vfh->vdev);

	if (f->index > 0)
		return -EINVAL;

	f->pixelformat = tx_video->format.pixelformat;

	return 0;
}

static int
csitx_dma_get_format(struct file *file, void *fh, struct v4l2_format *format)
{
	struct v4l2_fh *vfh = file->private_data;
	struct bst_csitx_video *tx_video = to_csitx_video(vfh->vdev);

	format->fmt.pix = tx_video->format;

	return 0;
}

static int
csitx_dma_set_format(struct file *file, void *fh, struct v4l2_format *format)
{
	struct v4l2_fh *vfh = file->private_data;
	struct bst_csitx_video *tx_video = to_csitx_video(vfh->vdev);

	if (vb2_is_busy(&tx_video->queue))
		return -EBUSY;

	tx_video->format = format->fmt.pix;
	dev_err(tx_video->queue.dev, "%s %d w:%d h:%d imgsize:%d\n", __func__, __LINE__,
			tx_video->format.width, tx_video->format.height, tx_video->format.sizeimage);
	return 0;
}


static const struct v4l2_ioctl_ops csitx_dma_ioctl_ops = {
	.vidioc_querycap		= csitx_dma_querycap,
	.vidioc_enum_fmt_vid_cap	= csitx_dma_enum_format,
	.vidioc_g_fmt_vid_out		= csitx_dma_get_format,
	.vidioc_s_fmt_vid_out		= csitx_dma_set_format,
	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	.vidioc_expbuf			= vb2_ioctl_expbuf,
	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations csitx_dma_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= video_ioctl2,
	.open		= v4l2_fh_open,
	.release	= vb2_fop_release,
	.poll		= vb2_fop_poll,
	.mmap		= vb2_fop_mmap,
};

static int
csitx_dma_queue_setup(struct vb2_queue *vq,
		     unsigned int *nbuffers, unsigned int *nplanes,
		     unsigned int sizes[], struct device *alloc_devs[])
{
	struct bst_csitx_video *tx_video = vb2_get_drv_priv(vq);

	dev_err(tx_video->queue.dev, "%s %d\n", __func__, __LINE__);
	/* Make sure the image size is large enough. */
	if (*nplanes)
		return sizes[0] < tx_video->format.sizeimage ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = tx_video->format.sizeimage;
	dev_err(tx_video->queue.dev, "%s %d\n", __func__, __LINE__);
	return 0;
}

static int csitx_dma_buffer_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct bst_csitx_video *tx_video = vb2_get_drv_priv(vb->vb2_queue);
	struct csitx_dma_buffer *buf = to_csitx_dma_buffer(vbuf);

	buf->tx_vdo = tx_video;
	return 0;
}

static void csitx_dma_buffer_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct bst_csitx_video *tx_video = vb2_get_drv_priv(vb->vb2_queue);
	struct csitx_dma_buffer *buf = to_csitx_dma_buffer(vbuf);

	spin_lock_irq(&tx_video->queued_lock);
	list_add_tail(&buf->queue, &tx_video->queued_bufs);
	spin_unlock_irq(&tx_video->queued_lock);
}

static int csitx_dma_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct bst_csitx_video *tx_video = vb2_get_drv_priv(vq);
	// struct csitx_dma_buffer *buf, *nbuf;
	int ret;

	tx_video->sequence = 0;
	dev_err(tx_video->queue.dev, "%s %d\n", __func__, __LINE__);
	atomic_set(&(tx_video->status), CSITX_STATUS_STREAMONING);
	tx_video->tx_dev->tx_mode = 1;
	tx_video->tx_dev->vc_enable |=  BIT(tx_video->video_index);
	tx_video->tx_dev->timming_index = 9;
	tx_video->tx_dev->ipi_mode = 0;

	tx_video->tx_dev->data_type = CSI_FORMAT_RGB888;
	ret = csitx_dmac_config(tx_video->tx_dev);
	if (ret < 0)
		return -1;
	atomic_set(&(tx_video->status), CSITX_STATUS_STREAMON_DONE);

	dev_err(tx_video->queue.dev, "%s %d\n", __func__, __LINE__);
	return 0;

}

static void csitx_dma_stop_streaming(struct vb2_queue *vq)
{
	struct bst_csitx_video *tx_video = vb2_get_drv_priv(vq);
	struct csitx_dma_buffer *buf, *nbuf;
	int ret;

	dev_err(tx_video->queue.dev, "%s %d\n", __func__, __LINE__);
	atomic_set(&(tx_video->status), CSITX_STATUS_STREAMOFFING);

	/* Give back all queued buffers to videobuf2. */
	spin_lock_irq(&tx_video->queued_lock);
	list_for_each_entry_safe(buf, nbuf, &tx_video->queued_bufs, queue) {
		vb2_buffer_done(&buf->buf.vb2_buf, VB2_BUF_STATE_ERROR);
		list_del(&buf->queue);
	}
	spin_unlock_irq(&tx_video->queued_lock);

	ret = csitx_dmac_stop(tx_video->tx_dev);
	if (ret < 0)
		return;
	atomic_set(&(tx_video->status), CSITX_STATUS_STREAMOFF_DONE);
	dev_err(tx_video->queue.dev, "%s %d\n", __func__, __LINE__);
}

static const struct vb2_ops csitx_dma_queue_qops = {
	.queue_setup = csitx_dma_queue_setup,
	.buf_prepare = csitx_dma_buffer_prepare,
	.buf_queue = csitx_dma_buffer_queue,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
	.start_streaming = csitx_dma_start_streaming,
	.stop_streaming = csitx_dma_stop_streaming,
};

int csitx_video_init(struct bst_csitx_device *tx_dev,
				struct bst_csitx_video *tx_video, int index)
{
	int ret;

	mutex_init(&tx_video->lock);
	INIT_LIST_HEAD(&tx_video->queued_bufs);
	spin_lock_init(&tx_video->queued_lock);
	atomic_set(&(tx_video->status), CSITX_STATUS_INVALID);

	dev_err(tx_dev->dev, "VC%d start init\n", index);

	tx_video->tx_dev = tx_dev;
	tx_video->format.pixelformat = V4L2_PIX_FMT_YUYV;
	tx_video->format.colorspace = V4L2_COLORSPACE_SRGB;
	tx_video->format.field = V4L2_FIELD_NONE;
	tx_video->format.width = 1920;
	tx_video->format.height = 1280;
	tx_video->format.bytesperline = tx_video->format.width * 2;
	tx_video->format.sizeimage = tx_video->format.bytesperline * tx_video->format.height;

	tx_video->buff_type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	tx_video->video_index = index;
	tx_video->video.fops = &csitx_dma_fops;
	tx_video->video.v4l2_dev = &tx_dev->v4l2_dev;
	tx_video->video.vfl_type = VFL_TYPE_VIDEO;
	tx_video->video.queue = &tx_video->queue;
	snprintf(tx_video->video.name, sizeof(tx_video->video.name), "csitx_vc%d",
		tx_video->video_index);

	tx_video->video.vfl_type = VFL_TYPE_VIDEO;
	tx_video->video.vfl_dir = VFL_DIR_TX;
	tx_video->video.release = video_device_release_empty;
	tx_video->video.ioctl_ops = &csitx_dma_ioctl_ops;
	tx_video->video.lock = &tx_video->lock;
	tx_video->video.device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_OUTPUT;

	video_set_drvdata(&tx_video->video, tx_video);

	tx_video->queue.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	tx_video->queue.io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	tx_video->queue.lock = &tx_video->lock;
	tx_video->queue.drv_priv = tx_video;
	tx_video->queue.buf_struct_size = sizeof(struct csitx_dma_buffer);
	tx_video->queue.ops = &csitx_dma_queue_qops;
	tx_video->queue.mem_ops = &vb2_dma_contig_memops;
	tx_video->queue.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC
					| V4L2_BUF_FLAG_TSTAMP_SRC_EOF;
	tx_video->queue.dev = tx_dev->dev;
	ret = vb2_queue_init(&tx_video->queue);
	if (ret < 0) {
		dev_err(tx_dev->dev, "VC%d failed to initialize VB2 queue\n", index);
		return -1;
	}

	ret = video_register_device(&tx_video->video, VFL_TYPE_VIDEO, 60 + index);
	if (ret < 0) {
		dev_err(tx_dev->dev, "VC%d failed to register video device\n", index);
		return -1;
	}
	dev_err(tx_dev->dev, "VC%d start init end\n", index);
	return 0;
}

void csitx_video_cleanup(struct bst_csitx_video *tx_video)
{
	if (video_is_registered(&tx_video->video))
		video_unregister_device(&tx_video->video);

	mutex_destroy(&tx_video->lock);
}
