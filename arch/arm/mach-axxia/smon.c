// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 INTEL Corporation
/*
 * Platform perf helper module for generic VP statistical monitor
 */

#include <linux/io.h>

#include <linux/axxia-ncr.h>

#include "smon.h"

static void memcpy32_fromio(u32 *dest, u32 *src, u32 len)
{
	u32 i;

	for (i = 0; i < len; i++)
		dest[i] = ioread32(src + i);
}

static void memcpy32_toio(u32 *dest, u32 *src, u32 len)
{
	u32 i;

	for (i = 0; i < len; i++)
		iowrite32(src[i], dest + i);
}

void smon_init_ncp(struct smon_s *smon, u32 node, u32 target,
		   u32 offset)
{
	smon->assigned[0] = UNASSIGNED;
	smon->assigned[1] = UNASSIGNED;
	smon->type = NCP_SMON;
	smon->node = node;
	smon->target = target;
	smon->offset = offset;
}

void smon_init_mem(struct smon_s *smon, u64 addr, u32 offset)
{
	smon->assigned[0] = UNASSIGNED;
	smon->assigned[1] = UNASSIGNED;
	smon->type = MEM_SMON;
	smon->addr = ioremap(addr, SZ_4K);
	smon->offset = offset;

	if (!smon->addr)
		pr_err("axxia perf, smon can't remap memory %lld\n", addr);
}

void smon_stop_if_unassigned(struct smon_s *smon)
{
	u32 control = 0;

	if (smon->assigned[0] == UNASSIGNED &&
	    smon->assigned[1] == UNASSIGNED) {
		ncr_write(NCP_REGION_ID(smon->node, smon->target), smon->offset,
			  1 * REG_SZ, &control);
	}
}

u32 smon_allocate(struct smon_s *smon, u8 event)
{
	if (smon->assigned[0] == UNASSIGNED) {
		smon->events[0] = event;
		smon->assigned[0] = ASSIGNED;
	} else if (smon->assigned[1] == UNASSIGNED) {
		smon->events[1] = event;
		smon->assigned[1] = ASSIGNED;
	} else {
		pr_warn("%s, no counter available\n", __func__);
		return -ENOCOUNTER;
	}

	return 0;
}

u32 smon_deallocate(struct smon_s *smon, u8 event)
{
	if (smon->assigned[0] == ASSIGNED && smon->events[0] == event)
		smon->assigned[0] = UNASSIGNED;
	else if ((smon->assigned[1] == ASSIGNED) && (smon->events[1] == event))
		smon->assigned[1] = UNASSIGNED;
	else
		return -ENOCOUNTER;

	return 0;
}

u32 smon_event_active(struct smon_s *smon, u8 event)
{
	if (smon->assigned[0] == ASSIGNED && smon->events[0] == event)
		return 0;
	else if ((smon->assigned[1] == ASSIGNED) && (smon->events[1] == event))
		return 0;

	return -ENOCOUNTER;
}

u32 smon_read(struct smon_s *smon, u8 event)
{
	u32 deltacount;

	if (smon->type == NCP_SMON)
		ncr_read(NCP_REGION_ID(smon->node, smon->target), smon->offset,
			 8 * REG_SZ, &smon->regs);
	else if (smon->type == MEM_SMON)
		memcpy32_fromio((u32 *)&smon->regs,
				(u32 *)smon->addr + smon->offset, 8);

	if (smon->assigned[0] == ASSIGNED &&
	    smon->events[0] == event) {
		if (smon->regs.count0 >= smon->lastread[0])
			deltacount = smon->regs.count0 - smon->lastread[0];
		else
			deltacount = 0xffffffff - smon->lastread[0]
					+ smon->regs.count0;

		smon->lastread[0] = smon->regs.count0;

		return deltacount;
	} else if ((smon->assigned[1] == ASSIGNED) &&
			(smon->events[1] == event)) {
		if (smon->regs.count1 >= smon->lastread[1])
			deltacount = smon->regs.count1 - smon->lastread[1];
		else
			deltacount = 0xffffffff - smon->lastread[1]
					+ smon->regs.count1;

		smon->lastread[1] = smon->regs.count1;

		return deltacount;
	}

	return -ENOEVENT;
}

u32 smon_start(struct smon_s *smon, u8 event)
{
	/* get currect configuration */
	if (smon->type == NCP_SMON)
		ncr_read(NCP_REGION_ID(smon->node, smon->target), smon->offset,
			 8 * REG_SZ, &smon->regs);
	else if (smon->type == MEM_SMON)
		memcpy32_fromio((u32 *)&smon->regs,
				(u32 *)smon->addr + smon->offset, 8);

	smon->regs.control = 1;	/* run counters */

	if (smon->assigned[0] == ASSIGNED && smon->events[0] == event) {
		smon->regs.event0 = event;
		smon->regs.count0 = 0;
		smon->lastread[0] = 0;

		if (smon->type == NCP_SMON) {
			/* write configuration, but do not change count reg */
			ncr_write(NCP_REGION_ID(smon->node, smon->target),
				  smon->offset, 2 * REG_SZ, &smon->regs);

			/* clear this events counter register */
			ncr_write(NCP_REGION_ID(smon->node, smon->target),
				  smon->offset + 4 * REG_SZ, 1 * REG_SZ,
				&smon->regs.count0);
		} else if (smon->type == MEM_SMON) {
			/* write configuration, but do not change count reg */
			memcpy32_toio((u32 *)smon->addr + smon->offset,
				      (u32 *)&smon->regs, 2);

			/* clear this events counter register */
			memcpy32_toio((u32 *)smon->addr + smon->offset + 4,
				      (u32 *)&smon->regs.count0, 1);
		}

	} else if ((smon->assigned[1] == ASSIGNED) &&
		(smon->events[1] == event)) {
		smon->regs.event1 = event;
		smon->regs.count1 = 0;
		smon->lastread[1] = 0;

		if (smon->type == NCP_SMON) {
			/* write configuration, but do not change count reg */
			ncr_write(NCP_REGION_ID(smon->node, smon->target),
				  smon->offset, 2 * REG_SZ, &smon->regs);

			/* clear this events counter register */
			ncr_write(NCP_REGION_ID(smon->node, smon->target),
				  smon->offset + 5 * REG_SZ, 1 * REG_SZ,
				  &smon->regs.count1);
		} else if (smon->type == MEM_SMON) {
			/* write configuration, but do not change count reg */
			memcpy32_toio((u32 *)smon->addr + smon->offset,
				      (u32 *)&smon->regs, 2);

			/* clear this events counter register */
			memcpy32_toio((u32 *)smon->addr + smon->offset + 5,
				      (u32 *)&smon->regs.count1, 1);
		}

	} else {
		pr_warn("%s, no counter available\n", __func__);
		return -ENOCOUNTER;
	}

	return 0;
}
