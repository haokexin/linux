/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_ISP_VIDEO_H__
#define __BST_ISP_VIDEO_H__

#include <media/v4l2-dev.h>
#include <media/v4l2-fh.h>
#include <media/videobuf2-v4l2.h>

#include <bst/media-dev.h>

#include "isp_hw.h"
#include "isp_proto_base.h"
#include "isp_proto_ipc.h"

#define ISP_VIDEO_DRIVER_NAME "bst-isp-video"

/* Firmware and hardware dependent parameters */
#define MIN_FRAME_BUFFER_NUM   (3)
#define MAX_FRAME_BUFFER_NUM   (8)
#define MAX_MESSAGES_PER_VIDEO (16)
/* When caller streamoff video, we must wait firmware return buffers to
 * avoid SMMU error, since we send CAMCLOSE first, we think there are 2
 * buffers still be used at most.
 */
#define WAIT_RETURN_BUF_TIMES  (10)
#define WAIT_RETURN_BUF_DELAY  (100000)

struct isp_channel;
struct isp_device;
struct isp_video;

enum video_status {
	VS_UNUSED = 0,
	VS_OPENED,
	VS_STREAM_ON,
	VS_STREAM_OFF,
};

enum isp_video_type {
	VIEW_VIDEO = 0,
	RAW_VIDEO = 1,
};

/**
 * struct isp_buffer - ISP video buffer
 * @vb:		videobuf2 buffer
 * @node:	List head for insertion into buffer queue
 * @dma:	Address from ISP hardware/firmware's view
 * @size:	Buffer size
 * @cycle:	Reserved buffer count after buffer is sent to firmware
 * @snr_exp_us:	Sensor exposure time by us
 */
struct isp_buffer {
	struct vb2_v4l2_buffer vb; /* NOTE: Must be first */
	struct list_head node;
	u32 dma[MAX_VIEWS_PER_VIDEO];
	int cycle;
	u32 snr_exp_us;
};

#define to_isp_buffer(buf) container_of(buf, struct isp_buffer, vb)

struct video_msg {
	u16 cmd_main;
	u16 cmd_minor;
	u16 exp_lines;
	u64 timestmap; /* by ns */
	ispusrdata data;
	struct work_struct msg_work;
	struct isp_video *video;
	struct list_head node;
};

/**
 * @enabled:		The view is enabled by config
 * @used:		The view is used actually
 * @meta:		The view will append meta datas, such as sensor embedded lines
 * @view_id		The view ID interact with firmware
 * @v4l2_pix_fmt	V4L2_PIX_FMT_XXX
 * @width		Image width by pixel
 * @height		Image height by pixel
 * @bytesperline	Bytes for 1 line
 * @sizeimage		Bytes for valid image
 * @sizetotal		Bytes for allocated size
 */
struct isp_view {
	u8 enabled : 1;
	u8 used	   : 1;
	u8 meta	   : 1;

	u8 view_id;
	u32 v4l2_pixel_fmt;
	u32 width;
	u32 height;
	u32 bytesperline;
	u32 sizeimage;
	u32 sizemeta;
	u32 sizetotal;
};

struct video_stat {
	u32 sn_drv;
	u32 rx_buf_all;
	u32 rx_buf_rsv;
	u32 rx_msg_bad;
	u32 tx_buf_done;
	u32 tx_buf_fail;
	u32 last_sn_all;
	u32 last_sn_app;
	u64 last_ts_all;
	u64 last_ts_app;
	u32 buf_num_drv;
	u32 buf_num_fw;
	u32 buf_num_app;
	u64 ts_streamon;
	u32 qbuf_num;
	u32 dqbuf_num;
	u32 dup_num;
	u32 drop_num;
	u32 reverse_num;
	u32 last_buf[MAX_VIEWS_PER_VIDEO];
};

/**
 * @lock:		Lock for video file's ioctl
 * @stream_lock:	Lock for video streamon/off
 * @vid:		The video ID which will be used to /dev/videoX
 * @cid:		The related channel's id, re-define it for convenience
 */
struct isp_video {
	/* Common */
	struct mutex lock;
	struct mutex stream_lock;
	struct isp_device *isp;
	struct isp_channel *channel;
	int vid;
	int cid;
	enum video_status status;

	/* Flags */
	u8 is_raw     : 1;
	u8 error      : 1;
	u8 mirror_raw : 1;
	u8 stream_op  : 1; /* 0: success, 1: failure */
	u8 hw_on      : 1; /* Firmware is working for this video */

	/* V4L2 */
	struct video_device video;
	enum v4l2_buf_type buf_type;
	struct vb2_queue *vb2_queue;
	struct mutex vb2_lock;

	/* View Management */
	struct isp_view views[MAX_VIEWS_PER_VIDEO];
	u8 used_views_num;
	u8 used_views_map[MAX_VIEWS_PER_VIDEO]; /* Current used view index */

	/* Buffer management */
	struct mutex free_buf_lock;
	struct list_head free_buf_queue;
	struct mutex wait_buf_lock;
	struct list_head wait_buf_queue;

	/* Message */
	struct mutex free_msg_lock;
	struct list_head free_msg_queue;
	struct mutex todo_msg_lock;
	struct workqueue_struct *todo_msg_queue;
	struct video_msg cache_msg[MAX_MESSAGES_PER_VIDEO];

	/* Statistics */
	struct video_stat stat;

	/* Misc */
	struct completion feed_buf_avail_comp;
	abnormal_t fw_ab_info;
	struct meta_zone isp_meta;
	struct meta_zone sensor_meta[MAX_SENSOR_EMBEDDED_ZONE];
};

struct isp_video_fh {
	struct v4l2_fh vfh;
	struct isp_video *video;
	struct vb2_queue queue;
	struct v4l2_format format;
};

#define to_isp_video_fh(fh) container_of(fh, struct isp_video_fh, vfh)

static inline void clear_video_msg(struct video_msg *msg)
{
	msg->cmd_main = 0;
	msg->cmd_minor = 0;
	msg->exp_lines = 0;
	msg->timestmap = 0;
	memset(&msg->data, 0, sizeof(msg->data));
}

void isp_video_init(struct isp_video *video, struct isp_channel *channel,
		    enum isp_video_type video_type);
void isp_video_cleanup(struct isp_video *video);

int isp_video_register(struct isp_video *video, struct v4l2_device *vdev);
void isp_video_unregister(struct isp_video *video);

int isp_video_dispatch_msg(struct isp_video *video, struct media_command *mc,
			   u64 ts);
void isp_video_update_cfg(struct isp_channel *channel);
const char *isp_video_status(struct isp_video *video);
void isp_video_recycle_wait_bufs(struct isp_video *video);
int isp_video_streamon_hw(struct isp_video *video);
int isp_video_streamoff_hw(struct isp_video *video);

#endif /* __BST_ISP_VIDEO_H__ */
