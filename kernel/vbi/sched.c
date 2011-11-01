/*
 * sched.c - hypervisor scheduled transition service
 *
 * Copyright (c) 2011 Wind River Systems, Inc.
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

#include <linux/string.h>
#include <vbi/vbi.h>
#include <vbi/private.h>

#ifdef DEBUG
#define DEBUGM(fmt, args...)    printk(fmt, ##args)
#else
#define DEBUGM(fmt, args...)
#endif

/*
 * vbi_sched_transition - transition to a new schedule
 *
 * This routine makes a hypercall to  transition to a new schedule <name>.
 *
 * Possible scheduler transitions are:
 *
 * SCHEDULER_TRANSITON_MAJOR
 * SCHEDULER_TRANSITION_MINOR
 * SCHEDULER_TRANSITION_TICK
 *
 *
 */
int32_t vbi_sched_transition(char *name, uint32_t transition_type, uint32_t core_id)
{
	int32_t status;
	struct vb_control *p = VBI_CNTRL_ADDR_GET();
	char sched_name[VB_NAMELEN];
	
	/* only safety profile hypervisor's control structure has the name field */
	if ((name == NULL) || (p == NULL))
		return -1;

	/* copy the name to vb_control's name field */
	strncpy(sched_name, name, sizeof(sched_name));
	vbi_vcore_irq_lock();
	status = vbi_sched_control_op(SCHED_CONTROL_TRANSITION, sched_name,
			(void *)transition_type, (void *)core_id, 0);
	vbi_vcore_irq_unlock();
	return status;
}
