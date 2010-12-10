/*
 * errors.h - vbi errors
 *
 * Copyright (c) 2009 Wind River Systems, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 */

#ifndef _VBI_ERRORS_H
#define _VBI_ERRORS_H

#define VBI_ERRORS_START		0

/* generic vbi errors */

#define VBI_INVALID_VERSION		(VBI_ERRORS_START + (-1))
#define VBI_INVALID_IRQ			(VBI_ERRORS_START + (-2))
#define VBI_INVALID_SHMEM		(VBI_ERRORS_START + (-3))
#define VBI_INVALID_MEM			(VBI_ERRORS_START + (-4))

/* vbi Exceptions error */

#define VBI_EXC_ERROR_START		(VBI_ERRORS_START + (-1000))
#define VBI_EXCBASE_SET_ERROR		(VBI_EXC_ERROR_START + (-1))

/* VIOAPIC errors   */
#define VBI_VIOAPIC_ERROR_START		(VBI_EXC_ERROR_START + (-100))
#define VBI_VIOAPIC_NULL		(VBI_VIOAPIC_ERROR_START + (-1))
#define VBI_VIOAPIC_IRQ_OUTBOUND	(VBI_VIOAPIC_ERROR_START + (-2))
#define VBI_VIOAPIC_IRQ_INVALID_DIR	(VBI_VIOAPIC_ERROR_START + (-3))
#define VBI_VIOAPIC_UNAVAIL		(VBI_VIOAPIC_ERROR_START + (-4))


/* VB MANAGEMENT errors */

#define VBI_VBMGMT_ERROR_START		(VBI_VIOAPIC_ERROR_START +  (-100))
#define VBI_ERR_VBMGMT_VB_INVALID	VBI_VBMGMT_ERROR_START +    (-1)
#define VBI_ERR_VBMGMT_INVALID_ARG	VBI_VBMGMT_ERROR_START +    (-2)
#define VBI_ERR_VBGMT_RESET_FAILED	VBI_VBMGMT_ERROR_START +    (-3)
#define VBI_ERR_VBMGMT_CTX_INVALID	VBI_VBMGMT_ERROR_START +    (-4)

/* VB Device Driver errors */

#define VBI_DEVDRV_ERROR_START		(VBI_VBMGMT_ERROR_START +  (-100))
#define VBI_ERR_ADD_RETRY		(VBI_DEVDRV_ERROR_START +  (-1))
#define VBI_ERR_DEVDRV_NULL_NAME	(VBI_DEVDRV_ERROR_START +  (-2))
#define VBI_ERR_DEVDRV_NULL_HANDLE	(VBI_DEVDRV_ERROR_START +  (-3))

#endif /* _VBI_ERRORS_H */
