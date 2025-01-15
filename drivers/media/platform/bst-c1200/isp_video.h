/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef C1200_ISP_VIDEO_H
#define C1200_ISP_VIDEO_H

#include <linux/v4l2-mediabus.h>
#include <linux/workqueue.h>
#include <media/media-entity.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-fh.h>
#include <media/videobuf2-v4l2.h>

// #define TEST_CODE

#include "cam_entity.h"
#include "isp_video_uapi.h"

#define ISP_VIDEO_DRIVER_NAME	 "c1200_ispvideo"
#define ISP_VIDEO_DRIVER_VERSION "0.0.1"

#define MAX_FRAME_BUF_NUM	    8
#define MAX_MEDIA_COMMAND_PER_VIDEO 10

// media command element array
// 0 ~ 7 is for normal frame buffer
// 8 is for reserve buffer
// 9 is for control command
#define RESERVE_BUF_CMD_INDEX 8
#define CTRL_CMD_INDEX	      9

#define ISP_RX_CNT_ADDR	      0x520B6800
#define ISP_RX_ERROR_CNT_ADDR 0x520B6804
#define ISP_PROCESS_CNT_ADDR  0x520B6810

#define ISP_TX_CNT_ADDR	      0x520B6820
#define ISP_TX_ERROR_CNT_ADDR 0x520B6824
#define ISP_IPC_PACK_CNT_ADDR 0x520B6830

#define LTM_ISP_CTRL_BASE_ADDR	0x52098800
#define LTM_ISP_SENSOR_REG_SIZE 0x400

#define VIEW_ID_NUM_PER_CAMERA 4 // PDNS included in view id

#define ISP_TICK_TO_NS	       333
#define ISP_STREAM_CMD_TIMEOUT (200)

struct c1200_isp_video;
struct c1200_isp_device;
struct bst_isp_channel;
struct v4l2_mbus_framefmt;
struct v4l2_pix_format;

/**
 * struct isp_buffer - ISP video buffer
 * @vb: videobuf2 buffer
 * @node: List head for insertion into buffer queue
 * @dma: DMA address
 */
struct isp_buffer {
	struct vb2_v4l2_buffer vb; // must be first element
	struct list_head node;
	uint32_t dma[MAX_VIEWS_PER_CAMERA];
	int cycle_count;
	void *mmap_uaddr[MAX_VIEWS_PER_CAMERA];
	uint32_t plane_size[MAX_VIEWS_PER_CAMERA];
	// dma_addr_t dma;
	// uint32_t dma_offset; // offset of dma buf to isp base
};

#define to_isp_buffer(buf) container_of(buf, struct isp_buffer, vb)

enum c1200_isp_video_dmaqueue_flags {
	/* Set if DMA queue becomes empty when ISP_PIPELINE_STREAM_CONTINUOUS */
	ISP_VIDEO_DMAQUEUE_UNDERRUN = (1 << 0),
	/* Set when queuing buffer to an empty DMA queue */
	ISP_VIDEO_DMAQUEUE_QUEUED = (1 << 1),
};

enum c1200_isp_video_status {
	VIDEO_STATUS_INVALID = 0,
	VIDEO_STATUS_OPENING,
	VIDEO_STATUS_FW_DONE,
	VIDEO_STATUS_OPEN_DONE,
	VIDEO_STATUS_STREAMONING,
	VIDEO_STATUS_STREAMON_DONE,
	VIDEO_STATUS_STREAMOFFING,
	VIDEO_STATUS_STREAMOFF_DONE,
	VIDEO_STATUS_CLOSING,
	VIDEO_STATUS_CLOSE_DONE,
};

enum c1200_isp_channel_views {
	ISP_VIEW_INVALID = 0,
	ISP_VIEW0 = 1,
	ISP_VIEW1 = 2,
	ISP_VIEW2 = 4,
	ISP_VIEW0_VIEW1 = 3,
	ISP_VIEW0_VIEW2 = 5,
	ISP_VIEW1_VIEW2 = 6,
	ISP_VIEW0_VIEW1_VIEW2 = 7,
};

/*
 * PENDING   wait for signal
 * WAIT_SEND wait for Tx to FW
 * WAIT_DONE wait for Rx from FW
 */
enum c1200_isp_resize_status {
	ISP_RESIZE_PENDING = 0,
	ISP_RESIZE_WAIT_SEND = 1,
	ISP_RESZIE_WAIT_DONE = 2
};

#define ISP_VIEW_MASK(i) (1 << i)

#define c1200_isp_video_dmaqueue_flags_clr(video) \
	({ (video)->dmaqueue_flags = 0; })

struct isp_view_format {
	uint32_t sizeimage;
	uint32_t bytesperline;
	uint32_t pixelformat;
	uint16_t width;
	uint16_t height;
} __packed;

struct cmd_element {
	void *media_cmd_vaddr;
	uint32_t media_cmd_paddr; // only use low 32 bit absolute address
	int index;
	//	struct list_head node;
};

struct internal_msg {
	uint16_t cmd_main;
	uint16_t cmd_minor;
	uint32_t tick;
	int64_t ktime;
	uint32_t user_data[4];
	struct work_struct msg_work;
	struct c1200_isp_video *video_ptr;
	struct list_head node;
};

struct isp_view {
	struct c1200_isp_video *video;
	bool enable;
	bool is_pdns_input;
	int index;
	uint32_t format;
	int width;
	int height;
	uint8_t cmd_view_id;
	uint32_t isp_view_fmt;
};

struct c1200_isp_video {
	struct video_device video;
	struct v4l2_ctrl_handler ctrl_handler;
	struct internal_msg cache_msg[MAX_MEDIA_COMMAND_PER_VIDEO];
	struct isp_view views[MAX_VIEWS_PER_CAMERA];
	struct media_pad pad;
	// struct camera_dev *camera;
	struct resolution_resize resize;
	enum v4l2_buf_type type;
	struct mutex mutex; /* format and crop settings */
	int enabled;
	bool is_pdns; // views video or pdns video
	bool is_raw_camera; // input data raw or yuv
	bool is_raw_video;
	int busy;
	atomic_t status;
	int chn_index; // isp channel index
	int video_index;
	int device_index;
	int current_views_num;
	uint8_t current_views_list[3]; // save current view index
	uint8_t current_views_id[4]; // 3 views id and one byte reserve
	int pdns_input_view; // which view is pdns input
	struct bst_isp_channel *channel;
	struct c1200_isp_device *isp;
	struct c1200_isp_video_fh *v4l2_handle;
	bool error;
	bool first_buf_received;
	bool hdmi_video;
	/* Video buffers queue */
	struct vb2_queue *queue;
	struct mutex queue_lock; /* protects the queue */
	struct mutex ctrl_lock; /* protects ctrls */

	struct mutex free_queue_lock; /* protects free_buf_queue */
	struct list_head free_buf_queue;
	struct mutex wait_queue_lock; /* protects wait_buf_queue */
	struct list_head wait_buf_queue;
	enum c1200_isp_video_dmaqueue_flags dmaqueue_flags;
	/* protects both free_cache_queue and busy_cache_queue */
	struct mutex cache_queue_lock;
	struct list_head free_cache_queue; // first in first out, like a queue
	struct mutex msg_queue_lock;
	struct workqueue_struct *msg_work_queue;
	uint64_t rx_buf_count;
	uint64_t tx_buf_count;
	uint64_t tx_drop_count; // no buf send to firmware
	uint64_t rx_reserved_count; // received fw reserved buf
	uint32_t last_done_buf[MAX_VIEWS_PER_CAMERA];
	uint32_t reserve_buf;
	u64 last_timestamp;
	u32 last_good_sequence;
	u32 total_bad_frames;
	abnormal_t fw_ab_info;
	u32 drv_sequence;
	int sensor_exp;
	struct completion stream_comp;
	struct completion feed_wait_buf_completion;
};

enum c1200_isp_video_type {
	VIEW_VIDEO = 0,
	RAW_VIDEO = 1,
};

#define to_c1200_isp_video(vdev) \
	container_of(vdev, struct c1200_isp_video, video)

struct c1200_isp_video_fh {
	struct v4l2_fh vfh;
	struct c1200_isp_video *video;
	struct vb2_queue queue;
	struct v4l2_format format;
	struct v4l2_fract timeperframe;
};

#define to_c1200_isp_video_fh(fh) \
	container_of(fh, struct c1200_isp_video_fh, vfh)

#define c1200_isp_video_queue_to_c1200_isp_video_fh(q) \
	container_of(q, struct c1200_isp_video_fh, queue)

int isp_channel_init_video(struct c1200_isp_device *isp,
			   struct bst_isp_channel *isp_channel,
			   enum c1200_isp_video_type video_type, int chn_sn);

void c1200_isp_video_cleanup(struct c1200_isp_video *video);

int c1200_isp_video_register(struct c1200_isp_video *video,
			     struct v4l2_device *vdev,
			     enum c1200_isp_video_type video_type);

void c1200_isp_video_unregister(struct c1200_isp_video *video);

void c1200_isp_video_cancel_stream(struct c1200_isp_video *video);

int copy_msg_to_video_cache(struct c1200_isp_video *video, uint16_t main,
			    uint16_t minor, uint32_t tick, int64_t ktime,
			    uint32_t *data, int num, int line_length);

void dump_video_index_mapping_info(struct c1200_isp_device *isp_dev, char *buf);

void dump_video_debug_info(struct c1200_isp_video *video);

int isp_video_debug_isp_wait(struct media_command *cmd);

int isp_video_debug_isp_done(struct media_command *cmd);

int notify_fw_start_stream(struct c1200_isp_video *video);

int notify_fw_stop_stream(struct c1200_isp_video *video);
int notify_fw_cam_plugout(struct c1200_isp_video *video);
int notify_fw_cam_plugin(struct c1200_isp_video *video);

int raw_video_start_stream(struct c1200_isp_video *video);
int raw_video_stop_stream(struct c1200_isp_video *video);
int send_video_buf_to_fw(struct c1200_isp_video *video, uint32_t *buf_paddr,
			 int cmd_index, int reserve_flag);
void update_raw_video_from_camera_config(struct c1200_isp_video *video,
					 struct camera_dev *cam_dev);
int update_video_from_camera_config(struct c1200_isp_video *video,
				    struct camera_dev *cam_dev);
#endif /* C1200_ISP_VIDEO_H */
