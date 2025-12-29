/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __DT_BINDINGS_MEDIA_BST_ISP_H__
#define __DT_BINDINGS_MEDIA_BST_ISP_H__

/* clang-format off */
/* ISP FSync Source ID */
#define FSYNC_INNER0		(0)
#define FSYNC_INNER1		(1)
#define FSYNC_OUTER0		(2)
#define FSYNC_OUTER1		(3)
#define FSYNC_OUTER2		(4)
#define FSYNC_OUTER3		(5)

/* Data type consistent with <Data Formats> in MIPI Specification for CSI-2 */
#define DT_FRAME_START		(0x00)
#define DT_FRAME_END		(0x01)
#define DT_LINE_START		(0x02)
#define DT_LINE_END		(0x03)
#define DT_BLANKING		(0x11)
#define DT_EMBEDDED_8B		(0x12)
#define DT_YUV420_8B		(0x18)
#define DT_YUV420_10B		(0x19)
#define DT_YUV420_8B_LEGACY	(0x1A)
#define DT_YUV420_8B_CS		(0x1C)
#define DT_YUV420_10B_CS	(0x1D)
#define DT_YUV422_8B		(0x1E)
#define DT_YUV422_10B		(0x1F)
#define DT_RGB444		(0x20)
#define DT_RGB555		(0x21)
#define DT_RGB565		(0x22)
#define DT_RGB666		(0x23)
#define DT_RGB888		(0x24)
#define DT_RAW28		(0x26)
#define DT_RAW24		(0x27)
#define DT_RAW6			(0x28)
#define DT_RAW7			(0x29)
#define DT_RAW8			(0x2A)
#define DT_RAW10		(0x2B)
#define DT_RAW12		(0x2C)
#define DT_RAW14		(0x2D)
#define DT_RAW16		(0x2E)
#define DT_RAW20		(0x2F)
#define DT_YUV422_12B		(0x30)

/* Extended data type, reflects bit order */
#define DT_UYVY			(0x1E)
#define DT_YUYV			(0x5E)

/* Physical Interface */
#define IF_DPHY			(0)
#define IF_CPHY			(1)
#define IF_MPHY			(2)

/* I2C register address and data width */
#define WORD_REG_WORD_VAL	(0)
#define WORD_REG_BYTE_VAL	(1)
#define BYTE_REG_BYTE_VAL	(2)

/* De-Serializer forward mode */
#define PIXEL_MODE		(0)
#define TUNNEL_MODE		(1)

/* When device is used by multiple nodes, only MASTER can control it. */
#define ROLE_MASTER		(0)
#define ROLE_SLAVE		(1)
#define ROLE_AUTO		(2)
/* clang-format on */

#endif /* __DT_BINDINGS_MEDIA_BST_ISP_H__ */
