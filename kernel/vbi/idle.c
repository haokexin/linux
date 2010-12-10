/* idle.c - context idle code
 *
 * Copyright (c) 2007-2009 Wind River Systems, Inc.
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
 * This provides an interface to put the currently running virtual board
 * into the idle state.
 */

#include <linux/types.h>
#include <vbi/vbi.h>

/*
 * vbi_idle - inform the hypervisor scheduler that the virtual board is
 * idle
 *
 * This function informs the hypervisor that the virtual board core's is
 * idle and also provides a timeout at which it needs to be made ready to
 * run again. This routine halts the active core untill the specified
 * timeout expires or and an asynchronous event like a interrupt is
 * delivered to the calling core. An zero timeout means to halt the core
 * indefinetly until an asynchronous event occurs.
 *
 */

void vbi_idle(uint64_t timeStamp)
{
	uint32_t tick_count = timeStamp & 0xFFFFFFFF;

	/* use the hypervisor system call to go into the idle state */

	vbi_ctx_ctl(VBI_CTXCTL_IDLE, tick_count, 0);
}
