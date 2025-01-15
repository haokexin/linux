/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_HWCV_IOCTL_H__
#define __BST_HWCV_IOCTL_H__
#include <linux/types.h>

#define BST_HWCV_DRIVER_NAME "bst_hwcv"
#define MAX_HWCV_PLANAR_NUM 3
#define MAX_HWCV_LAYER_NUM 3
#define MAX_SBS_GWARP_SENSOR_NUM 4

struct hwcv_allocation_data {
	uint32_t length;
	uint32_t paddr;
	uint32_t fd;
	uint64_t uaddr;
};

struct hwcv_fd_data {
	int fd;
	uint32_t paddr;
	uint32_t length;
};

struct hwcv_handle_data {
	uint32_t paddr;
};

struct hwcv_sync_data {
	uint32_t paddr;
	uint8_t dir;
};

struct hwcv_gwarp_data {
	uint8_t engine_id;
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
	uint32_t src_phy_addr[MAX_HWCV_PLANAR_NUM];
	uint32_t dst_phy_addr[MAX_HWCV_PLANAR_NUM];
	uint32_t lut_phy_addr;
};

struct hwcv_sbs_gwarp_data {
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
	uint32_t src_phy_addr[MAX_HWCV_PLANAR_NUM];
	uint32_t dst_phy_addr[MAX_HWCV_PLANAR_NUM];
	uint32_t lut_phy_addr;
};

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
	uint32_t src_phy_addr[MAX_HWCV_PLANAR_NUM];
	uint32_t dst_phy_addr[MAX_HWCV_LAYER_NUM][MAX_HWCV_PLANAR_NUM];
	uint32_t coeff_phy_addr;
	uint32_t coeff_size;
	uint32_t x_ratio;
	uint32_t y_ratio;
	uint32_t x_init_phase;
	uint32_t y_init_phase;
};

#define HWCV_IOCTL_MAGIC 'H'
#define HWCV_IOCTL_ALLOC _IOWR(HWCV_IOCTL_MAGIC, 1, struct hwcv_allocation_data)
#define HWCV_IOCTL_FREE _IOW(HWCV_IOCTL_MAGIC, 2, struct hwcv_handle_data)
#define HWCV_IOCTL_IMPORT _IOWR(HWCV_IOCTL_MAGIC, 3, struct hwcv_fd_data)
#define HWCV_IOCTL_IFREE _IOW(HWCV_IOCTL_MAGIC, 4, struct hwcv_handle_data)
#define HWCV_IOCTL_SYNC _IOW(HWCV_IOCTL_MAGIC, 5, struct hwcv_sync_data)

#define HWCV_IOCTL_GWARP _IOW(HWCV_IOCTL_MAGIC, 10, struct hwcv_gwarp_data)
#define HWCV_IOCTL_SBS_GWARP \
	_IOW(HWCV_IOCTL_MAGIC, 11, struct hwcv_sbs_gwarp_data)
#define HWCV_IOCTL_SCALER _IOW(HWCV_IOCTL_MAGIC, 12, struct hwcv_scaler_data)

#endif
