/*
 * msg.c - vbi Message library
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

#include <linux/types.h>
#include <linux/kernel.h>
#include <vbi/private.h>

#define DEBUGM(fmt, args...)
#define _DEBUGM(fmt, args...) printk(fmt, ##args)

/*
 * vbi_receive - Receive a message from another context
 *
 * This routine makes a hypercall and waits for a message to be received from
 * another context. It blocks until a message is received. This operation may
 * be aborted if an interrupt is delivered to the waiting Virtual board. If the
 * "flags" field in the control structure passed to this function is set to be
 * VBI_MSG_CTL_FLAG_RETRY the receive operation will be retried in case it was
 * aborted before the expected message was received successfully.
 *
 */

int32_t vbi_receive(void *rmsg, uint32_t rlen, struct vbi_msg_info *info,
			struct vbi_msg_ctl *ctl)
{
	int32_t retval;

	if (is_corevbi_only()) {
		VBISTAT_VERBOSE(vbi_receive);
		return -1;
	}

	while (1)
	{
		retval = vbi_rx_op(rmsg, rlen, info, ctl);

		/* if retry flag set, continue */
		if (!ctl ||
			(ctl && ((ctl->flags & VBI_MSG_CTL_FLAG_RETRY)
				!= VBI_MSG_CTL_FLAG_RETRY)))
			break;

		/* if retry flag is set _AND_ msg was aborted, retry */
		if (!info ||
			(info && (info->error != VBI_MSG_ERROR_ABORTED)))
			break;

		DEBUGM("%s aborted, retrying \n", __FUNCTION__);
	}
	return retval;
}
