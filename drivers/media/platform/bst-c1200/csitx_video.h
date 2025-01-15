/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef _CSI_TX_VIDEO_H_
#define _CSI_TX_VIDEO_H_

#include <linux/dmaengine.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/videodev2.h>

#include <media/v4l2-dev.h>
#include <media/videobuf2-v4l2.h>

struct csitx_dma_buffer {
	struct vb2_v4l2_buffer buf;
	struct list_head queue;
	struct bst_csitx_video *tx_vdo;
};

#define to_csitx_dma_buffer(vb)	container_of(vb, struct csitx_dma_buffer, buf)

struct bst_video_format {
	unsigned int vf_code;
	unsigned int width;
	const char *pattern;
	unsigned int code;
	unsigned int bpp;
	u32 fourcc;
};

static const struct bst_video_format bst_video_formats[] = {
	{ 2, 8, NULL, 0x24, 3, 0 },
};

enum bst_csitx_video_status {
	CSITX_STATUS_INVALID = 0,
	CSITX_STATUS_OPENING,
	CSITX_STATUS_OPEN_DONE,
	CSITX_STATUS_STREAMONING,
	CSITX_STATUS_STREAMON_DONE,
	CSITX_STATUS_STREAMOFFING,
	CSITX_STATUS_STREAMOFF_DONE,
	CSITX_STATUS_CLOSING,
	CSITX_STATUS_CLOSE_DONE,
};


struct bst_csitx_video {
	// struct dma_async_tx_descriptor *dma_desc;
	struct video_device video;
	int video_index;
	// size_t buf_len;
	// enum dma_transfer_direction dir;
	// dma_addr_t csitx_dmaaddr;
	// struct dma_chan *dma_chan;
	struct bst_csitx_device *tx_dev;

	enum v4l2_buf_type buff_type;
	atomic_t status;
	struct mutex lock;
	struct vb2_queue queue;
	unsigned int sequence;

	// unsigned int initial;
	// struct dma_async_tx_descriptor *desc;

	struct list_head queued_bufs;
	spinlock_t queued_lock;
	struct v4l2_pix_format format;
};

#define to_csitx_video(vdev)	container_of(vdev, struct bst_csitx_video, video)

int csitx_video_init(struct bst_csitx_device *tx_dev,
		struct bst_csitx_video *tx_video, int index);
void csitx_video_cleanup(struct bst_csitx_video *tx_video);
#endif
