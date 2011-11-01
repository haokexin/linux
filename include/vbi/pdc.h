/* pdc.h - Process Data Channel definitions */

/* Copyright 2009-2010 Wind River Systems, Inc.
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


/* DESCRIPTION
 *
 * Message format for Process Data Channel (PDC)
 * from Stub Device Driver (SDD)
 * to Actual Device Driver Guest Interface (ADD-GI)
 */

#ifndef _VBI_PDC_H
#define _VBI_PDC_H


#ifndef _ASMLANGUAGE


typedef VBI_NS_HANDLE vbi_pdc_handle;

struct vbi_msg_hdr
{
	uint32_t msgId;
	uint32_t msgArg;
};

struct msg_pdc
{
	struct vbi_msg_hdr hdr;
	VB_ALIGN_FIELD_64 (void *buf, pad1);
	VB_ALIGN_FIELD_64 (size_t buflen, pad2);
	uint16_t request;
	uint16_t op;
#ifdef DEV_ASYNC_IOCTL
	VB_ALIGN_FIELD_64 (uint64_t token, pad3); /* token */
#endif
};

struct msg_pdc_reply
{
	struct vbi_msg_hdr hdr;
	uint32_t   status;
	uint32_t   dataVal;
};

#ifdef DEV_ASYNC_IOCTL
struct ioctlOp
{
	void *buf;
	uint64_t token;
	VB_ALIGN_FIELD_64 (uint32_t op, pad7);
}
#endif /* DEV_ASYNC_IOCTL */

/* following structure is taken from hypervisor
 * include/sys/wrhvDevCore.h
 */
struct intr_device_channel_buffer
{
	VB_ALIGN_FIELD_64 (void *rxBuf, pad1);      /* rx buffer to rx  */
	VB_ALIGN_FIELD_64 (size_t rxBufLen, pad2);  /* rx buffer length */
	VB_ALIGN_FIELD_64 (void *txBuf, pad3);      /* tx buffer to tx  */
	VB_ALIGN_FIELD_64 (size_t txBufLen, pad4);  /* tx buffer length */
	uint32_t rxBufRdPtr;                /* rx buffer read pointer  */
	uint32_t rxBufWrPtr;                /* rx buffer write pointer */
	uint32_t txBufRdPtr;                /* tx buffer read pointer  */
	uint32_t txBufWrPtr;                /* tx buffer write pointer */
	uint32_t intStatus;                 /* intStatus, or what intr */
};

#ifdef CONFIG_WRHV_SAFETY_PROFILE
struct pdc_buf_set
{
	uint64_t gpaBufBase;
	uint64_t gpaBufLen;
};
#endif

#define SYS_PDC_REQUEST_OK		0
#define SYS_PDC_REQUEST_FAILED		-1

#define PDC_IOCTL_SIO_BAUD_SET		1
#define PDC_IOCTL_SIO_MODE_SET		2
#define PDC_IOCTL_SIO_HW_OPTS_SET	3
#define PDC_IOCTL_SIO_HUP		4
#define PDC_IOCTL_SIO_OPEN		5
#ifdef CONFIG_WRHV_SAFETY_PROFILE
#define PDC_IOCTL_BUF_GPA_GET		10
#define PDC_IOCTL_BUF_GPA_SET		11
#define PDC_IOCTL_BUF_SIZE_GET		12
#endif

#define PDC_IOCTL_AMIO_CHANNEL_SET	20

#define PDC_IOCTL_SIO_REQUEST_RESPONSE  0x8000
#define PDC_IOCTL_SIO_GET_IER		(PDC_IOCTL_SIO_REQUEST_RESPONSE | 2)

typedef enum {
	PDC_REQUEST_MIN			= 1,
	PDC_REQUEST_IOCTL		= 1,
	PDC_REQUEST_READ		= 2,
	PDC_REQUEST_WRITE		= 3,
	PDC_REQUEST_INIT		= 4,
	PDC_REQUEST_ATTACH_GUEST	= 5,
	PDC_REQUEST_DETACH_GUEST	= 6,
	PDC_REQUEST_CORE_MSG		= 7,
	PDC_REQUEST_MAX			= 7
} vbi_pdc_request;

extern int32_t vbi_pdc_op(vbi_pdc_handle pdcHandle,
			 vbi_pdc_request requestType, uint32_t ioctlOp,
			 void *bufferPtr, size_t bufferLen, uint64_t token);

extern int32_t vbi_pdc_init(const char *instanceName,
			  vbi_pdc_handle *pPdcHandle);

#endif

#endif  /* _VBI_PDC_H */
