/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/types.h>

#ifndef __BST_HWCV_COMMON_H__
#define __BST_HWCV_COMMON_H__

const char *hwcv_get_format_name(uint32_t format);

const char *hwcv_get_scaler_mode_str(uint8_t mode);
const char *hwcv_get_gwarp_mode_str(uint8_t mode);
const char *hwcv_get_gwarp_algo_str(uint8_t algo);

#endif
