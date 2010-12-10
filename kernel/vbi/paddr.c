/*
 * paddr.c - translate virtual address to a physical address
 *
 * Copyright (c) 2008 Wind River Systems, Inc.
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
These modules provide interfaces to translate the current context's guest
physical address into the physical machine address. vbi_get_guest_dma_addr is
specifically for a guest wanting an address that can be used by a DMA device.
vbi_guest_phys_to_phys is used to return an absolute physical address to used
perhaps to communicate with the hypervisor itself maybe buffer pointers.
*/

#include <linux/types.h>
#include <vbi/private.h>

/*
 * vbi_guest_phys_to_phys - translate the spcified guest physical to physical
 *                          address
 *
 * This function makes a hypervisor call to translate the specified guest
 * physical address to physical address. This may be required for quick buffer
 * transfer that requires the physical address of a memory region. The
 * hypervisor may be running with > 4GB memory so the phys address is always
 * 64-bit.
 *
 */
int32_t vbi_guest_phys_to_phys(void *gaddr, uint64_t *paddr)
{
	return vbi_hy_ioctl(VBI_HYIOCTL_PADDR, gaddr, paddr,
				(void*)VBI_HYIOCTL_PADDR_PHYS, 0 );
}

/*
 * vbi_get_guest_dma_addr - translate the spcified guest physical to DMA address
 *
 * This function makes a hypervisor call to translate the specified guest
 * physical address  to a physical address. This may be required for a device
 * driver that requires the address it can use for DMA. The hypervisor may be
 * running with > 4GB memory so the phys address is always 64-bit.
 *
 */
int32_t vbi_get_guest_dma_addr(void *gaddr, uint64_t *paddr)
{
	return vbi_hy_ioctl(VBI_HYIOCTL_PADDR, gaddr, paddr,
				VBI_HYIOCTL_PADDR_DMA, 0);
}
