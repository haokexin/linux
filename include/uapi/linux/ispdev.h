/* SPDX-License-Identifier: ((GPL-2.0+ WITH Linux-syscall-note) OR BSD-3-Clause) */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef _UAPI__LINUX_ISPDEV_H
#define _UAPI__LINUX_ISPDEV_H

#include <linux/types.h>

#define MAX_CHANNEL_NUM (16)
#define MAX_VIEW_NUM	(3)
#define MAX_BUF_NUM	(8)

struct feed_buf {
	__u32 index;
	__s32 fd[MAX_VIEW_NUM];
	__u64 pa[MAX_VIEW_NUM];
	__u32 size[MAX_VIEW_NUM];
	__u32 bytesused[MAX_VIEW_NUM];
	__u64 timestamp;
	__u64 sequence;
	__u32 exposure_time;
	__s32 reserved[8];
};

/*
 *@video_id:    Which video id will be queried
 *@num:         Buffer numbers, filled by driver
 *@bufs:        Detail buffer information, filled by driver
 */
struct feed_query_buffers {
	__u32 video_id;
	__u32 num;
	struct feed_buf bufs[MAX_BUF_NUM];
};

/*
 *@video_id:    Which video id will be queried
 *@timeout_ms:  Timeout by ms, -1 means blocked until get a free buffer, 0: non-block
 *@index:       Which buffer is free, filled by driver
 */
struct feed_query_free_buf {
	__u32 video_id;
	__s32 timeout_ms;
	__u32 index;
};

struct feed_control {
	__u32 video_bitmap;
	struct feed_buf bufs[MAX_CHANNEL_NUM];
	__u32 reserved[3];
};

/* clang-format off */
#define IOC_FEED_ENABLE                 _IOW('f', 0x10, int)
#define IOC_FEED_QUERY_FREE_BUF        _IOWR('f', 0x11, struct feed_query_free_buf)
#define IOC_FEED_QUERY_BUFS            _IOWR('f', 0x12, struct feed_query_buffers)
#define IOC_FEED_DO                     _IOW('f', 0x13, struct feed_control)
/* clang-format on */

#endif /* _UAPI__LINUX_ISPDEV_H */
