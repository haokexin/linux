// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/iommu.h>
#include <media/videobuf2-dma-contig.h>

#include <uapi/linux/ispdev.h>

#include "isp_misc.h"

static int isp_feed_query_bufs(struct isp_device *isp,
			       struct feed_query_buffers *qry)
{
	unsigned int i;
	struct isp_video *video;
	struct vb2_queue *vb2_q;

	dev_dbg(isp->dev, "V%02d FEED_QUERY_BUFS\n", qry->video_id);

	if (qry->video_id >= ARRAY_SIZE(isp->channels))
		return -EINVAL;

	video = &isp->channels[qry->video_id].views_video;
	vb2_q = video->vb2_queue;
	if (!vb2_q) {
		qry->num = 0;

		/* Maybe return -EXX is better, return 0 for compatibility */
		return 0;
	}

	qry->num = vb2_q->num_buffers;
	for (i = 0; i < vb2_q->num_buffers; ++i) {
		unsigned int j;
		struct vb2_buffer *vb2_buf;

		vb2_buf = vb2_get_buffer(vb2_q, i);
		qry->bufs[i].index = i;
		for (j = 0; j < vb2_buf->num_planes; ++j) {
			u8 vidx;
			dma_addr_t dma_addr;

			vidx = video->used_views_map[j];
			qry->bufs[i].bytesused[vidx] =
				vb2_get_plane_payload(vb2_buf, j);
			qry->bufs[i].size[vidx] = vb2_plane_size(vb2_buf, j);
			dma_addr = vb2_dma_contig_plane_dma_addr(vb2_buf, j);
			if (isp->iommud)
				qry->bufs[i].pa[vidx] = iommu_iova_to_phys(
					isp->iommud, dma_addr);
			else
				qry->bufs[i].pa[vidx] = dma_addr;

			dev_dbg(isp->dev,
				"V%02d FEED_QUERY_BUFS: index: %u, view: %u, size: %8d, bytesused: %8d, pa: 0x%016llX\n",
				qry->video_id, i, vidx, qry->bufs[i].size[vidx],
				qry->bufs[i].bytesused[vidx],
				qry->bufs[i].pa[vidx]);
		}
	}

	return 0;
}

static int isp_feed_query_free_buf(struct isp_device *isp,
				   struct feed_query_free_buf *qry)
{
	int rv;
	struct isp_buffer *wait_buf;
	struct isp_video *video;

	dev_dbg(isp->dev, "V%02d FEED_QUERY_FREE_BUF\n", qry->video_id);
	if (qry->video_id >= ARRAY_SIZE(isp->channels))
		return -EINVAL;

	video = &isp->channels[qry->video_id].views_video;
	while (true) {
		mutex_lock(&video->wait_buf_lock);
		if (!list_empty(&video->wait_buf_queue)) {
			wait_buf = list_first_entry(&video->wait_buf_queue,
						    struct isp_buffer, node);
			mutex_unlock(&video->wait_buf_lock);
			break;
		}
		mutex_unlock(&video->wait_buf_lock);
		reinit_completion(&video->feed_buf_avail_comp);
		if (qry->timeout_ms == 0) {
			return -ETIMEDOUT;
		} else if (qry->timeout_ms < 0) {
			wait_for_completion_interruptible(
				&video->feed_buf_avail_comp);
		} else {
			rv = wait_for_completion_interruptible_timeout(
				&video->feed_buf_avail_comp,
				msecs_to_jiffies(qry->timeout_ms));
			if (rv == 0) {
				dev_err(isp->dev,
					"V%02d FEED_QUERY_FREE_BUF: timed out\n",
					qry->video_id);
				return -ETIMEDOUT;
			}
		}
	}

	qry->index = wait_buf->vb.vb2_buf.index;
	dev_dbg(isp->dev, "V%02d FEED_QUERY_FREE_BUF: index: %u\n",
		qry->video_id, wait_buf->vb.vb2_buf.index);

	return 0;
}

static int isp_feed(struct isp_device *isp, struct feed_control *feed_ctrl)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(isp->channels); ++i) {
		struct isp_video *video;
		struct isp_buffer *wait_buf;

		if (!(feed_ctrl->video_bitmap & (1 << i)))
			continue;

		dev_dbg(isp->dev, "V%02d FEED\n", i);
		video = &isp->channels[i].views_video;
		mutex_lock(&video->wait_buf_lock);
		if (!list_empty(&video->wait_buf_queue)) {
			wait_buf = list_first_entry(&video->wait_buf_queue,
						    struct isp_buffer, node);
			list_del(&wait_buf->node);
			mutex_unlock(&video->wait_buf_lock);
		} else {
			mutex_unlock(&video->wait_buf_lock);
			return -EIO;
		}

		if (wait_buf->vb.vb2_buf.state != VB2_BUF_STATE_ACTIVE) {
			dev_err(isp->dev,
				"V%02d FEED: invalid buffer state: %d\n", i,
				wait_buf->vb.vb2_buf.state);
			return -EIO;
		}

		wait_buf->cycle = 0;
		wait_buf->vb.vb2_buf.timestamp =
			feed_ctrl->bufs[i].timestamp * 1000;
		wait_buf->vb.sequence = feed_ctrl->bufs[i].sequence;

		vb2_buffer_done(&wait_buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
		dev_dbg(isp->dev,
			"V%02d FEED: index: %u, ts: %12llu, sn: %8u\n", i,
			wait_buf->vb.vb2_buf.index,
			wait_buf->vb.vb2_buf.timestamp, wait_buf->vb.sequence);
	}

	return 0;
}

static int isp_feed_enable(struct isp_device *isp, int enable)
{
	if (enable)
		isp->feed_mode = true;
	else
		isp->feed_mode = false;

	return 0;
}

static long isp_misc_ioctl(struct file *filp, unsigned int cmd,
			   unsigned long arg)
{
	int rv;
	struct isp_device *isp;
	struct device *dev;
	int enable;

	isp = container_of(filp->private_data, struct isp_device, miscdev);
	dev = isp->dev;
	mutex_lock(&isp->miscdev_lock);
	switch (cmd) {
	case IOC_FEED_ENABLE:
		if (get_user(enable, (int __user *)arg)) {
			dev_err(dev, "FEED ENABLE: Failed to copy from arg\n");
			rv = -EFAULT;
			break;
		}
		rv = isp_feed_enable(isp, enable);
		break;
	case IOC_FEED_QUERY_BUFS:
		if (!isp->feed_mode) {
			dev_err(dev, "FEED QUERYBUFS: Not feed mode\n");
			rv = -EINVAL;
			break;
		}

		if (copy_from_user(&isp->qry_bufs, (void *)arg,
				   sizeof(isp->qry_bufs))) {
			dev_err(dev,
				"FEED QUERYBUFS: Failed to copy from arg\n");
			rv = -EFAULT;
			break;
		}

		rv = isp_feed_query_bufs(isp, &isp->qry_bufs);
		if (rv)
			break;

		if (copy_to_user((void *)arg, &isp->qry_bufs,
				 sizeof(isp->qry_bufs))) {
			dev_err(dev, "FEED QUERYBUFS: Failed to copy to arg\n");
			rv = -EFAULT;
		}
		break;
	case IOC_FEED_QUERY_FREE_BUF:
		if (!isp->feed_mode) {
			dev_err(dev, "FEED QUERYBUF: Not feed mode\n");
			rv = -EINVAL;
			break;
		}

		if (copy_from_user(&isp->qry_free_buf, (void *)arg,
				   sizeof(isp->qry_free_buf))) {
			dev_err(dev,
				"FEED QUERYFREEBUF: Failed to copy from arg\n");
			rv = -EFAULT;
			break;
		}

		rv = isp_feed_query_free_buf(isp, &isp->qry_free_buf);
		if (rv)
			break;

		if (copy_to_user((void *)arg, &isp->qry_free_buf,
				 sizeof(isp->qry_free_buf))) {
			dev_err(dev,
				"FEED QUERYFREEBUF: Failed to copy to arg\n");
			rv = -EFAULT;
		}
		break;
	case IOC_FEED_DO:
		if (!isp->feed_mode) {
			dev_err(dev, "FEED: Not feed mode\n");
			rv = -EINVAL;
			break;
		}

		if (copy_from_user(&isp->feed_ctrl, (void *)arg,
				   sizeof(isp->feed_ctrl))) {
			dev_err(dev, "FEED: Failed to copy from arg\n");
			rv = -EFAULT;
			break;
		}

		rv = isp_feed(isp, &isp->feed_ctrl);
		break;
	default:
		rv = -EINVAL;
	}

	mutex_unlock(&isp->miscdev_lock);
	return rv;
}

static const struct file_operations isp_misc_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = isp_misc_ioctl,
};

int isp_misc_init(struct isp_device *isp)
{
	mutex_init(&isp->miscdev_lock);
	isp->miscdev.minor = MISC_DYNAMIC_MINOR;
	/* TODO: this should be isp%d. Keep this name only for compatibility */
	isp->miscdev.name = "isp_misc";
	isp->miscdev.minor = MISC_DYNAMIC_MINOR;
	isp->miscdev.fops = &isp_misc_fops;

	return misc_register(&isp->miscdev);
}

void isp_misc_exit(struct isp_device *isp)
{
	misc_deregister(&isp->miscdev);
	mutex_destroy(&isp->miscdev_lock);
}
