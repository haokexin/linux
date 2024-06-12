/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Helper module for board specific I2C bus registration
 *
 * Copyright (C) 2021 INTEL Corporation
 */

#ifndef __ASM__ARCH_AXXIA_SMON_H
#define __ASM__ARCH_AXXIA_SMON_H

#include <linux/kernel.h>

struct smon_reg_s {
	u32 control;
	u8 event0;
	u8 event1;
	u16 reserved;
	u32 compare0;
	u32 compare1;
	u32 count0;
	u32 count1;
	u32 time;
	u32 maxtime;
};

struct smon_s {
	struct smon_reg_s regs;
	u32 type; /* NCP_SMON or MEM_SMON */
	u32 *addr; /* MEM_SMON */
	u32 node; /* NCP_SMON */
	u32 target; /* " */
	u32 offset;
	u32 lastread[2];
	u8 assigned[2];
	u8 events[2];
};

#define REG_SZ 4

#define MEM_SMON 0
#define NCP_SMON 1

#define UNASSIGNED 0
#define ASSIGNED 1

#define ENOCOUNTER 1
#define ENOEVENT 2

void smon_init_ncp(struct smon_s *smon, u32 node, u32 target,
		   u32 offset);
void smon_init_mem(struct smon_s *smon, u64 addr, u32 offset);
void smon_stop_if_unassigned(struct smon_s *smon);
u32 smon_allocate(struct smon_s *smon, u8 event);
u32 smon_deallocate(struct smon_s *smon, u8 event);
u32 smon_event_active(struct smon_s *smon, u8 event);
u32 smon_read(struct smon_s *smon, u8 event);
u32 smon_start(struct smon_s *smon, u8 event);

#endif /* __ASM__ARCH_AXXIA_SMON_H */
