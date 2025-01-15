/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __ISP_VIDEO_UAPI_H__
#define __ISP_VIDEO_UAPI_H__

/* for struct media_command definition */
#include <linux/coreip/proto_api_common.h>

#define MAX_CAMERA_NAME_LEN_USER (16)
#define MAX_EMBEDDED_ZONE_NUM	 3
#define MAX_VIEWS_PER_CAMERA	 3

struct isp_ipcmsg {
	struct media_command cmd;
	int reply;
};

struct isp_regcfg {
	uint16_t regaddr;
	uint16_t regval;
};

struct abnormal_info {
	uint8_t abnormal_id;
	uint8_t abnormal_type;
	uint32_t last_good_sequence;
	uint32_t total_bad_frames;
	uint32_t total_frames;
} __packed;

struct scale_size {
	uint16_t width;
	uint16_t height;
};

struct crop_size {
	uint16_t topCropBefore;
	uint16_t botCropBefore;
	uint16_t lefCropBefore;
	uint16_t rigCropBefore;
	uint16_t topCropAfter;
	uint16_t botCropAfter;
	uint16_t lefCropAfter;
	uint16_t rigCropAfter;
};

struct resolution_resize {
	struct crop_size crop_size;
	struct scale_size scale_size;
	uint8_t view_id;
};

struct camera_info_t_user {
	char camera_name[MAX_CAMERA_NAME_LEN_USER];
	int camera_id;
	int is_streaming;
	uint16_t camera_data_type;
	uint16_t camera_fps;
	uint16_t camera_raw_width;
	uint16_t camera_raw_height;
};

struct isp_embed_info {
	uint32_t emd_line_num[MAX_EMBEDDED_ZONE_NUM];
	uint32_t emd_line_start[MAX_EMBEDDED_ZONE_NUM];
	uint32_t emd_zone_offset[MAX_VIEWS_PER_CAMERA][MAX_EMBEDDED_ZONE_NUM];
};

struct isp_emd_view_info {
	uint8_t embedded_view;
	struct isp_embed_info isp_embed_info;
};

/* v4l2 ioclt private definitions for bst isp */
/* print debug mode status info */
#define ISPIOC_PRINT_DBGINFO _IO('V', BASE_VIDIOC_PRIVATE + 0)
/* enter debug mode */
#define ISPIOC_ENTER_DBGMODE _IO('V', BASE_VIDIOC_PRIVATE + 1)
/* lease debug mode */
#define ISPIOC_LEAVE_DBGMODE _IO('V', BASE_VIDIOC_PRIVATE + 2)
/* alloc debug memory */
#define ISPIOC_ALLOC_DBGMEM  _IOW('V', BASE_VIDIOC_PRIVATE + 3, size_t)
/* release debug memory */
#define ISPIOC_FREE_DBGMEM   _IO('V', BASE_VIDIOC_PRIVATE + 4)
/* query debug memory physical address */
#define ISPIOC_QUERY_DBGMEM  _IOR('V', BASE_VIDIOC_PRIVATE + 5, uint32_t)
/* transfer isp firmware ipc message */
#define ISPIOC_TRANS_IPCMSG \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 6, struct isp_ipcmsg)

/* transfer isp firmware ipc message */
#define ISPIOC_G_CAMREG _IOWR('V', BASE_VIDIOC_PRIVATE + 10, struct isp_regcfg)
/* transfer isp firmware ipc message */
#define ISPIOC_S_CAMREG _IOWR('V', BASE_VIDIOC_PRIVATE + 11, struct isp_regcfg)
/* get abnormal info */
#define ISPIOC_G_ABNORMAL_INFO \
	_IOR('V', BASE_VIDIOC_PRIVATE + 12, struct abnormal_info)
/* set resize res info */
#define ISPIOC_RESIZE_RESOLUTION \
	_IOW('V', BASE_VIDIOC_PRIVATE + 15, struct resolution_resize)

#define ISPIOC_QUERY_CAM_INFO \
	_IOR('V', BASE_VIDIOC_PRIVATE + 16, struct camera_info_t_user)
#define ISPIOC_QUERY_EMBED_INFO \
	_IOR('V', BASE_VIDIOC_PRIVATE + 17, struct isp_emd_view_info)

/* get data mode */
#define ISPIOC_G_DATA_MODE _IOR('V', BASE_VIDIOC_PRIVATE + 20, int)

struct isp_ctrl {
	uint32_t value; // isp_set_iqinfo_t  iqVal  ,if ctrl_addr == 0  ; use
		// value
	uint16_t aecManualExp[3];
	uint16_t aecManualGain[3];
	uint16_t manualAWBGain[3][3]; // set mannual white balance value
};

/* The base for isp v4l2 controls. */
#define V4L2_CID_USER_ISP_BASE		     (V4L2_CID_USER_BASE + 0x1090)
#define V4L2_CID_ISP_TEST		     (V4L2_CID_USER_ISP_BASE + 0)
#define V4L2_CID_ISP_MANUAL_WB		     (V4L2_CID_USER_ISP_BASE + 1)
#define V4L2_CID_ISP_MANUAL_EXPOSURE	     (V4L2_CID_USER_ISP_BASE + 2)
#define V4L2_CID_ISP_YDNS		     (V4L2_CID_USER_ISP_BASE + 3)
#define V4L2_CID_ISP_UVDNS		     (V4L2_CID_USER_ISP_BASE + 4)
/* The base for isp v4l2 controls types */
#define V4L2_CTRL_ISP_TYPE_BASE		     (V4L2_CTRL_COMPOUND_TYPES + 0x10)
#define V4L2_CTRL_ISP_TYPE_TEST		     (V4L2_CTRL_ISP_TYPE_BASE + 0)
#define V4L2_CTRL_ISP_TYPE_MANUAL_WB	     (V4L2_CTRL_ISP_TYPE_BASE + 1)
#define V4L2_CTRL_ISP_TYPE_MANUAL_EXPOSURE   (V4L2_CTRL_ISP_TYPE_BASE + 2)
#define V4L2_CTRL_ISP_TYPE_RESIZE_RESOLUTION (V4L2_CTRL_ISP_TYPE_BASE + 3)
#endif
