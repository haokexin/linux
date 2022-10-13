/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (C) 2021 INTEL Corporation
 */

#ifndef __DRIVERS_AXXIA_ACP_NCR_H
#define __DRIVERS_AXXIA_ACP_NCR_H

#ifndef NCP_REGION_ID
#define NCP_REGION_ID(node, target) \
((unsigned long)((((node) & 0xffff) << 16) | ((target) & 0xffff)))
#endif

#ifndef NCP_NODE_ID
#define NCP_NODE_ID(region) (((region) >> 16) & 0xffff)
#endif

#ifndef NCP_TARGET_ID
#define NCP_TARGET_ID(region) ((region) & 0xffff)
#endif

unsigned int ncr_register_read(unsigned int *address);
void ncr_register_write(const unsigned int value, unsigned int *address);
int ncr_read(unsigned int region, unsigned int address, int number,
	     void *buffer);
int ncr_read32(unsigned int region, unsigned int offset, unsigned int *value);
int ncr_write(unsigned int region, unsigned int address, int number,
	      void *buffer);
int ncr_write32(unsigned int region, unsigned int offset, unsigned int value);
int ncr_read_nolock(unsigned int region, unsigned int address, int number,
		    void *buffer);
int ncr_write_nolock(unsigned int region, unsigned int address, int number,
		     void *buffer);

void ncr_start_trace(void);
void ncr_stop_trace(void);

 /*
  * when defined, the RTE driver module will set/clear
  * the ncr_reset_active flag to indicate when Axxia device
  * reset is in progress. This flag will be checked by the
  * kernel axxia-ncr driver and ddr_retention code.
  */
#ifdef CONFIG_ARCH_AXXIA_NCR_RESET_CHECK
extern int ncr_reset_active;
#endif

#endif	/* __DRIVERS_AXXIA_ACP_NCR_H */
