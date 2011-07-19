/*
 * shmem.c - shared memory utility functions
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
DESCRIPTION
This module contains the code for retriving the memory regions assigned to
a virtual board. The information is passed in to the virtual board via the
configuration structure. There is a descriptor table for each type of memory.

MEMORY TYPES

There is a memory memory region assigned only to each board which can be
retrieved by calling vbi_find_mem().

There is a shared memory which may be shared with other boards in the
system. The shared memory can be retrieved by using vbi_find_shmem()

Each core in the system is assigned a private memory region. The base address
and the length of this region may obtained by calling vbi_find_core_prv_mem ()
Alternatively the following macros can be used to obtained similar information

*/

#include <linux/types.h>
#include <linux/string.h>
#include <linux/module.h>
#include <vbi/vbi.h>
#include <vbi/private.h>
#include <vbi/stats.h>

/*
 * vbi_find_shmem - locate the shared memory parameters for a given region
 *
 * This function finds the shared memory region associated with the name
 * specified. The first argument to this function indicates the name of the
 * shared to find. If this region exists in the list of configured regions the
 * address, length and MMU attributes are set in the pointers passed in as
 * arguments.
 *
 */
int32_t vbi_find_shmem(int8_t *smName, void **addr,
				uint32_t *length, uint32_t *attr)
{
	struct vb_config *config = VBI_CONFIG_ADDR_GET();
	struct vb_sm_info *info = config->sharedMemoryRegionsConfigAddress;
	int32_t num = config->num_sm;
	int32_t i;

	if (is_corevbi_only()) {
		VBISTAT_VERBOSE(vbi_find_shmem);
		return VBI_INVALID_SHMEM;
	}

	for (i = 0; i < num; i++, info++) {
		if (!strncmp((char *)smName, (char *)info->name, VB_NAMELEN)) {
			/* Found */
			*addr = info->addr;
			*length = info->length;
			*attr = info->attr;
			return 0;
		}
	}
	*addr = 0;
	return VBI_INVALID_SHMEM;
}

/*
 * vbi_find_mem - locate the memory parameters for a given region
 *
 * This function finds the memory region associated with the name specified.
 * The first argument to this function indicates the name of the memory to find
 * If this region exists in the list of configured regions the address,
 * length and MMU attributes are set in the pointers passed in as
 * arguments.
 *
 */
int32_t vbi_find_mem(int8_t *name, void **addr,
				uint32_t *length, uint32_t *attr)
{
	struct vb_config *config = VBI_CONFIG_ADDR_GET();
	struct vb_mem_info *info = config->memoryRegionsConfigAddress;
	int32_t num = config->num_mem;
	int32_t i;

	if (is_corevbi_only()) {
		VBISTAT_VERBOSE(vbi_find_mem);
		return VBI_INVALID_SHMEM;
	}

	for (i = 0; i < num; i++, info++) {
		if (!strncmp((char *)name, (char *)info->name, VB_NAMELEN)) {
			/* Found */
			*addr = info->addr;
			*length = info->length;
			*attr = info->attr;
			return 0;
		}
	}
	*addr = 0;
	return VBI_INVALID_SHMEM;
}

/*
 * vbi_find_core_prv_mem - locate the private memory for a core
 *
 * This function gets the base address of the private memory region assigned to
 * the running core.
 *
 */
int32_t vbi_find_core_prv_mem(void **addr, size_t *length)
{
	*addr = (void *)VBI_VCORE_PRIVMEM_BASE_GET();
	*length = VBI_VCORE_PRIVMEM_SIZE_GET();

	return 0;
}

/*
 * vbi_vb_find_ram_size - return RAM size of VB
 *
 * This function gets the RAM size of the VB's configuration
 *
 */
int32_t vbi_vb_find_ram_size(uint32_t board_id, size_t *phys_mem_size)
{
	if (is_corevbi_only()) {
		VBISTAT_VERBOSE(vbi_vb_find_ram_size);
		return -1;
	}

	 return vbi_vb_remote(VBI_VBREMOTE_RAMSIZE, board_id, 0,
				phys_mem_size);
}

