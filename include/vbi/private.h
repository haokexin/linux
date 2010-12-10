/*
 * private.h - virtual board interface private definitions
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

#ifndef _VBI_PRIVATE_H
#define _VBI_PRIVATE_H

#include <linux/types.h>
#include <linux/linkage.h>
#include <vbi/syscalls.h>
#include <vbi/syscall.h>
#include <vbi/types.h>
#include <vbi/stats.h>

#ifndef _ASMLANGUAGE

/* private ioapic operator */
extern asmlinkage int32_t vbi_io_apic_op(uint32_t ioctl, int32_t irq, uint32_t filter, uint32_t vbId);

/* private interrupt redirection operator */
extern asmlinkage int32_t vbi_vcore_irq_redirect(int32_t irq, int32_t CoreId);

/* Name service private operator */
extern asmlinkage int32_t vbi_ns_op(uint32_t op, char* name, uint32_t rev,
			VBI_NS_HANDLE *handle, uint32_t timeout,
			uint32_t options);

#ifdef CONFIG_WRHV_CERT
/* Transition to a new schedule */
extern asmlinkage int32_t vbi_sched_control_op(uint32_t transition,
			char *sched_name, void *transition_type, void *core_id,
			void *args);
extern asmlinkage int32_t vbi_port_op(uint32_t operation_type,
			uint32_t vb_port_id, uint32_t length);
#endif

/* Message receive private operator */
extern asmlinkage int32_t vbi_rx_op(void *rmsg, uint32_t rlen,
			struct vbi_msg_info *info, struct vbi_msg_ctl *ctl);

#if defined(CONFIG_WRHV_COREVBI_ONLY)
static inline int32_t vbi_vb_remote(uint32_t op, uint32_t board_id,
			int32_t core_id,  void *out)
{
	VBISTAT_VERBOSE(vbi_vb_remote);
	return -1;
}
#else
extern asmlinkage int32_t vbi_vb_remote(uint32_t op, uint32_t board_id,
			int32_t core_id,  void *out);
#endif

#endif

#endif  /* _VBI_PRIVATE_H */
