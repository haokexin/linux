// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
#ifndef BST_DISPLAY_OSAL_H
#define BST_DISPLAY_OSAL_H

#ifdef __QNX__
# define BST_DISPLAY_QNX 1
#else
# define BST_DISPLAY_QNX 0
#endif

#ifdef __KERNEL__
# define BST_DISPLAY_LINUX 1
#else
# define BST_DISPLAY_LINUX 0
#endif

#if BST_DISPLAY_QNX
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include <bst/bst_slog.h>
#elif BST_DISPLAY_LINUX
#include <linux/string.h>
#include <linux/completion.h>
#else
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "elog_cfg.h"
#include "debug.h"
#endif

#define DISP_CMD_DBG_TAG  "disp-cmd debug:"
#define DISP_CMD_INFO_TAG "disp-cmd info:"
#define DISP_CMD_WARN_TAG "disp-cmd warning:"
#define DISP_CMD_ERR_TAG  "disp-cmd error:"

#if BST_DISPLAY_QNX
#undef BST_TAG
#define BST_TAG   DISP_CMD_INFO_TAG
#define DISP_INFO blog_info
#define DISP_WARN blog_warning
#define DISP_ERR  blog_error
#ifdef __cplusplus
}
#endif
#elif BST_DISPLAY_LINUX

#define DISP_INFO(fmt, ...) \
	pr_info("[%s] [%s:%d]" fmt, DISP_CMD_INFO_TAG, __func__, __LINE__, ##__VA_ARGS__)
#define DISP_WARN(fmt, ...) \
	pr_warn("[%s] [%s:%d]" fmt, DISP_CMD_WARN_TAG, __func__, __LINE__, ##__VA_ARGS__)
#define DISP_ERR(fmt, ...) \
	pr_err("[%s] [%s:%d]" fmt, DISP_CMD_ERR_TAG, __func__, __LINE__, ##__VA_ARGS__)
#define DISP_DBG(fmt, ...) \
	pr_debug("[%s] [%s:%d]" fmt, DISP_CMD_DBG_TAG, __func__, __LINE__, ##__VA_ARGS__)
#else

#define DISP_INFO(fmt, ...) \
	LOG(DBG_INFO, (DISP_CMD_INFO_TAG), (fmt), ##__VA_ARGS__)

#define DISP_WARN(fmt, ...) \
	LOG(DBG_WARN, (DISP_CMD_WARN_TAG), (fmt), ##__VA_ARGS__)

#define DISP_ERR(fmt, ...) \
	LOG(DBG_ERROR, (DISP_CMD_ERR_TAG), (fmt), ##__VA_ARGS__)

#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))
#define ALIGN_KERNEL_MASK(x, mask) (((x) + (mask)) & ~(mask))
#define ALIGN_KERNEL(x, a) ALIGN_KERNEL_MASK(x, (typeof(x))(a) - 1)
#define ALIGN(x, a) ALIGN_KERNEL((x), (a))
static inline void swap(int a,int b)
{
    int temp;
    temp=a; a=b; b=temp;
}
#define has_bit(nr, mask)	(BIT(nr) & (mask))
#define has_bits(bits, mask)	(((bits) & (mask)) == (bits))
#endif

#define MAX2(a, b)    ((a) > (b) ? (a) : (b))
#define MAX3(a, b, c) (MAX2(MAX2(a, b), c))

// clang-format on

#endif /* BST_DISPLAY_OSAL_H */
