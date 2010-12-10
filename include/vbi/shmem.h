/*
 * shmem.h - Virtual Interface memory utility functions
 *
 * Copyright 2008 Wind River Systems, Inc.
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
This header file declares the vbi API for managing memory regions
*/

#ifndef _VBI_SHMEM_H
#define _VBI_SHMEM_H

#ifndef	_ASMLANGUAGE

extern int32_t vbi_find_shmem(int8_t *smName, void **addr,
		uint32_t *length, uint32_t *attr);

extern int32_t vbi_find_mem( int8_t *name, void **addr,
		uint32_t *length, uint32_t *attr);

/* find the base address and length of a core's private memory */
extern int32_t vbi_find_core_prv_mem(void **addr, size_t *length);

extern int32_t vbi_vb_find_ram_size(uint32_t board_id, size_t *phys_mem_size);

#endif /* _ASMLANGUAGE */

#endif  /* _VBI_SHMEM_H */
