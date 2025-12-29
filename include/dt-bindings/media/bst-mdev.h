/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/*
 * Copyright (C) 2025 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __DT_BINDINGS_MEDIA_BST_MDEV_H__
#define __DT_BINDINGS_MEDIA_BST_MDEV_H__

/* clang-format off */
#define DES_UNKNOWN		(0)
#define DES_MAX9286		(20)
#define DES_MAX9296		(21)
#define DES_MAX96712		(22)
#define DES_MAX96722		(23)
#define DES_MAX96724		(24)
#define DES_MAX96726		(25)

#define SER_UNKNOWN		(0)
#define SER_MAX96701		(1)
#define SER_MAX96705		(2)
#define SER_MAX9295		(20)
#define SER_MAX9295A		(21)
#define SER_MAX9295E		(22)
#define SER_MAX96717		(30)
#define SER_MAX96717F		(31)
#define SER_MAX96717R		(32)

/* ADI deserialier and serializer */
#define GMSL1			(1)
#define GMSL2			(2)
#define GMSL3			(3)
#define PORTA			(0)
#define PORTB			(1)
#define PORTC			(2)
#define PORTD			(3)
#define PIPEX			(0)
#define PIPEY			(1)
#define PIPEZ			(2)
#define PIPEU			(3)

#define CSI_MODE_4X2		(0)
#define CSI_MODE_2X4		(1)
#define CSI_MODE_1X4A_2X2	(2)
#define CSI_MODE_1X4B_2X2	(3)
#define CSI0			(0)
#define CSI1			(1)
#define CSI2			(2)
#define CSI3			(3)

#define FSYNC_OFF		(0)
#define FSYNC_INNER		(1)
#define FSYNC_OUTER		(2)
/* clang-format on */

#endif /* __DT_BINDINGS_MEDIA_BST_MDEV_H__ */
