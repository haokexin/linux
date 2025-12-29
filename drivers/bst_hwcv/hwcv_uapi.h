/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_HWCV_UAPI_H__
#define __BST_HWCV_UAPI_H__

#include <linux/types.h>

/* Use 'H' as magic number */
#define HWCV_IOC_MAGIC 'H'
#define HWCV_IOW(nr, type) _IOW(HWCV_IOC_MAGIC, nr, type)
#define HWCV_IOR(nr, type) _IOR(HWCV_IOC_MAGIC, nr, type)
#define HWCV_IOWR(nr, type) _IOWR(HWCV_IOC_MAGIC, nr, type)

#define HWCV_IOCTL_ALLOC HWCV_IOWR(0x1, struct hwcv_allocation_data)
#define HWCV_IOCTL_IMPORT HWCV_IOWR(0x2, struct hwcv_import_data)
#define HWCV_IOCTL_SYNC HWCV_IOW(0x3, struct hwcv_sync_data)
#define HWCV_IOCTL_RELEASE HWCV_IOW(0x4, uint32_t)
#define HWCV_IOCTL_SCALER HWCV_IOW(0x5, struct hwcv_scaler_data)
#define HWCV_IOCTL_GWARP HWCV_IOW(0x6, struct hwcv_gwarp_data)
#define HWCV_IOCTL_GET_DRVIER_VERSION HWCV_IOR(0x7, struct hwcv_version)

#define HWCV_MAX_PLANAR_NUM 3
#define HWCV_MAX_LAYER_NUM 3
#define HWCV_VERSION_SIZE 16

enum hwcv_format {
	HWCV_FMT_RGB888 = 0x0,
	HWCV_FMT_RGB888_PLANAR = 0x1,
	HWCV_FMT_YUV422_YUYV = 0x2,
	HWCV_FMT_YUV422_UYVY = 0x3,
	HWCV_FMT_NV12 = 0x4,
	HWCV_FMT_NV21 = 0x5,
	HWCV_FMT_YUV420_PLANAR = 0x6,
	HWCV_FMT_YUV422_PLANAR = 0x7,
};

enum hwcv_buf_sync_dir {
	HWCV_SYNC_FOR_DEVICE = 0x0,
	HWCV_SYNC_FOR_CPU = 0x1,
};

enum hwcv_scaler_mode {
	HWCV_SCALER_POLYPHASE = 0x0,
	HWCV_SCALER_PYRAMID = 0x1,
};

enum hwcv_gwarp_mode {
	HWCV_GWARP_NORMAL = 0x0,
	HWCV_GWARP_SBS = 0x1,
};

enum hwcv_gwarp_interpolation {
	HWCV_GWARP_BICUBIC = 0x0,
	HWCV_GWARP_BILINEAR = 0x1,
};

struct hwcv_version {
	uint32_t major;
	uint32_t minor;
	uint32_t revision;
	uint8_t str[HWCV_VERSION_SIZE];
};

/* 64byte */
struct hwcv_allocation_data {
	/* requeset */
	uint32_t length;

	/* respose */
	uint32_t fd;
	uint32_t handle;
	uint32_t dma_addr;

	/* rsv */
	uint8_t rsv[48];
};

struct hwcv_import_data {
	/* requeset */
	uint32_t fd;
	uint32_t length;

	/* respose */
	uint32_t handle;
	uint32_t dma_addr;

	/* rsv */
	uint8_t rsv[48];
};

struct hwcv_sync_data {
	/* requeset */
	uint64_t handle_ptr;
	uint32_t size;
	uint8_t dir;

	/* rsv */
	uint8_t rsv[51];
};

/* 196byte */
struct hwcv_scaler_data {
	uint8_t mode;
	uint8_t gauss_enable;
	uint8_t leftedge_split_flag;
	uint8_t layer_num;
	uint8_t format;
	uint32_t src_width;
	uint32_t src_height;
	uint32_t dst_width;
	uint32_t dst_height;
	uint32_t src_stride;
	uint32_t dst_stride;
	uint32_t src_dma_addr[HWCV_MAX_PLANAR_NUM];
	uint32_t dst_dma_addr[HWCV_MAX_LAYER_NUM][HWCV_MAX_PLANAR_NUM];
	uint32_t coeff_dma_addr;
	uint32_t coeff_size;
	uint32_t x_ratio;
	uint32_t y_ratio;
	uint32_t x_init_phase;
	uint32_t y_init_phase;

	/* rsv */
	uint8_t rsv[92];
};

struct hwcv_gwarp_data {
	/* requeset */
	uint8_t mode;
	uint8_t engine_id;
	uint8_t sensor_id;
	uint8_t interpolation;
	uint8_t src_format;
	uint8_t dst_format;
	uint32_t src_width;
	uint32_t dst_width;
	uint32_t src_height;
	uint32_t dst_height;
	uint32_t src_stride;
	uint32_t dst_stride;
	uint32_t lut_stride;
	uint32_t src_dma_addr[HWCV_MAX_PLANAR_NUM];
	uint32_t dst_dma_addr[HWCV_MAX_PLANAR_NUM];
	uint32_t lut_dma_addr;

	/* rsv */
	uint8_t rsv[132];
};

#endif
