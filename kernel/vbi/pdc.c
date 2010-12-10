/* pdc.c - Para-virtualized Device Channel VBI */

/*
 * Copyright (c) 2009-2010 Wind River Systems, Inc.
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


/*
DESCRIPTION
These modules provide interfaces to connect paravirtualized
device driver (PDD) with the Guest Interface of Actual
Device Driver in the hypervisor (ADD-GI)

*/

#include <linux/types.h>
#include <linux/module.h>
#include <vbi/vbi.h>
#include <vbi/syscall.h>
#include <vbi/pdc.h>
#include <vbi/syscalls.h>

uint8_t pdcHandleLookup[(VB_MAX_VIRTUAL_BOARDS + (sizeof(uint8_t) - 1)) 
				/ sizeof(uint8_t)]; 

#undef DEBUG
#ifdef DEBUG
#define DEBUG_PRINTF(fmt, args...) do { printk("%s: %d: %s: " fmt, \
					VBI_BOARD_NAME_GET(), 	     \
					VBI_BOARD_ID_GET(), __FUNCTION__, \
					## args); } while (0)


#define PRINT_IF_CALLED() DEBUG_PRINTF("called\n")
#else
#define DEBUG_PRINTF(fmt, args...)
#define PRINT_IF_CALLED()
#endif

static void pdcHandleSet(vbi_pdc_handle pdcHandle)
{
	pdcHandleLookup[pdcHandle >> 3] |= (uint8_t)(1 << (pdcHandle % 8));  
}

static uint32_t pdcHandleGet(vbi_pdc_handle pdcHandle)
{
	if (pdcHandleLookup[pdcHandle >> 3] & (uint8_t)(1 << (pdcHandle % 8))) {
		return 0;
	}
	return -1;
}

/*
 *
 * vbi_pdc_op - send message on Paravirtualized Device channel
 *
 */

int32_t vbi_pdc_op(vbi_pdc_handle pdcHandle, vbi_pdc_request requestType,
			uint32_t ioctlOp, void *bufferPtr, size_t bufferLen,
			uint64_t token)
{
	struct msg_pdc requestMsg;
	struct msg_pdc_reply replyMsg;

	if (pdcHandle == 0
		|| (pdcHandle >= VB_MAX_VIRTUAL_BOARDS)
		|| -1 == pdcHandleGet(pdcHandle)) {
			DEBUG_PRINTF("vbi_pdc_op: invalid handle\n");
			return (-1);
	}

	if ((requestType < PDC_REQUEST_MIN)
		|| (requestType > PDC_REQUEST_MAX)) {
		DEBUG_PRINTF("vbi_pdc_op: invalid request\n");
		return (-1);
	}

	requestMsg.hdr.msgId = 0;
	requestMsg.hdr.msgArg = 0;
	requestMsg.request = requestType;
	requestMsg.op = ioctlOp;
	requestMsg.buf = bufferPtr;
	requestMsg.buflen = bufferLen;

	if (vbi_send (pdcHandle, &requestMsg, sizeof(requestMsg),
		&replyMsg, sizeof(replyMsg), NULL, NULL) != 0) {
		DEBUG_PRINTF("vbi_send fails\n");
		return -1;
	}

	if (replyMsg.status != 0) {	
		DEBUG_PRINTF("vbi_send reply != OK\n");
		return (VBI_ERR_ADD_RETRY);
	}

	if (ioctlOp & PDC_IOCTL_SIO_REQUEST_RESPONSE)
		return replyMsg.dataVal;

	return 0;
}

/*
 *
 * vbi_pdc_init - Initialize Paravirtualized Device channel
 *
 */

int32_t vbi_pdc_init(const char *instanceName, vbi_pdc_handle *pPdcHandle)
{
	int32_t 	retStatus;

	/* validate ADD Guest Interface name */

	if (instanceName == NULL) {
		DEBUG_PRINTF("vbi_pdc_init: instanceName NULL\n");
		return VBI_ERR_DEVDRV_NULL_NAME;
	}
	if (pPdcHandle == NULL) {
		DEBUG_PRINTF("vbi_pdc_init: pPdcHandle NULL\n");
		return VBI_ERR_DEVDRV_NULL_HANDLE;
	}
	/* call Name Server to lookup ADD Guest Interface */
	retStatus = vbi_ns_lookup((char *)instanceName, 0, pPdcHandle,
				VBI_NS_NO_TIMEOUT, VBI_NS_OPTION_NONE);
	if (retStatus == 0) {
		DEBUG_PRINTF("Successfully connected to %s\n", instanceName);
		pdcHandleSet(*pPdcHandle);
	}
	else
		DEBUG_PRINTF("Failed to connected to %s\n", instanceName);
	return retStatus;
}
