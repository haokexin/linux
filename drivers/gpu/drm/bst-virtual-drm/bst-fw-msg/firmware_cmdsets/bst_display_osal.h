// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
#ifndef _BST_DISPLAY_OSAL_H_
#define _BST_DISPLAY_OSAL_H_

#ifdef __QNX__
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include <bst/bst_slog.h>
#elif __KERNEL__
#include <linux/string.h>
#include <linux/completion.h>
#else
#include "Std_Types.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#endif

#define DISP_TAG "bst_display_cmd"

#ifdef __QNX__
#undef BST_TAG
#define BST_TAG   DISP_TAG
#define DISP_INFO blog_info
#define DISP_WARN blog_warning
#define DISP_ERR  blog_error
#ifdef __cplusplus
}
#endif
#elif __KERNEL__

#define DISP_INFO(fmt, ...) \
	pr_info("[%s] [%s:%d]" fmt, DISP_TAG, __func__, __LINE__, ##__VA_ARGS__)
#define DISP_WARN(fmt, ...) \
	pr_warn("[%s] [%s:%d]" fmt, DISP_TAG, __func__, __LINE__, ##__VA_ARGS__)
#define DISP_ERR(fmt, ...) \
	pr_err("[%s] [%s:%d]" fmt, DISP_TAG, __func__, __LINE__, ##__VA_ARGS__)
#define DISP_DBG(fmt, ...) \
	pr_debug("[%s] [%s:%d]" fmt, DISP_TAG, __func__, __LINE__, ##__VA_ARGS__)
#else

#define DISP_INFO(fmt, ...) \
	myprintf("INFO: [%s] [%s:%d]" fmt, DISP_TAG, __func__, __LINE__, ##__VA_ARGS__)
#define DISP_WARN(fmt, ...) \
	myprintf("WARM: [%s] [%s:%d]" fmt, DISP_TAG, __func__, __LINE__, ##__VA_ARGS__)
#define DISP_ERR(fmt, ...) \
	myprintf("ERROR: [%s] [%s:%d]" fmt, DISP_TAG, __func__, __LINE__, ##__VA_ARGS__)
#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))
#define __ALIGN_KERNEL_MASK(x, mask) (((x) + (mask)) & ~(mask))
#define __ALIGN_KERNEL(x, a) __ALIGN_KERNEL_MASK(x, (typeof(x))(a) - 1)
#define ALIGN(x, a) __ALIGN_KERNEL((x), (a))
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

#endif /* _BST_DISPLAY_OSAL_H_ */
