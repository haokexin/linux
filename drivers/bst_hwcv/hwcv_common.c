// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include "hwcv_common.h"
#include "hwcv_uapi.h"

const char *hwcv_get_format_name(uint32_t format)
{
	switch (format) {
	case HWCV_FMT_RGB888:
		return "RGB888";
	case HWCV_FMT_RGB888_PLANAR:
		return "RGB888_PLANAR";
	case HWCV_FMT_YUV422_YUYV:
		return "YUV422_YUYV";
	case HWCV_FMT_YUV422_UYVY:
		return "YUV422_UYVY";
	case HWCV_FMT_NV12:
		return "NV12";
	case HWCV_FMT_NV21:
		return "NV21";
	case HWCV_FMT_YUV420_PLANAR:
		return "YUV420_PLANAR";
	case HWCV_FMT_YUV422_PLANAR:
		return "YUV422_PLANAR";
	default:
		return "UNF";
	}
}

const char *hwcv_get_scaler_mode_str(uint8_t mode)
{
	switch (mode) {
	case 0x0:
		return "polyphase";
	case 0x1:
		return "pyramid";
	default:
		return "UNF";
	}
}

const char *hwcv_get_gwarp_mode_str(uint8_t mode)
{
	switch (mode) {
	case 0x0:
		return "normal";
	case 0x1:
		return "sbs";
	default:
		return "UNF";
	}
}

const char *hwcv_get_gwarp_algo_str(uint8_t algo)
{
	switch (algo) {
	case 0x0:
		return "bicubic";
	case 0x1:
		return "bilinear";
	default:
		return "UNF";
	}
}
