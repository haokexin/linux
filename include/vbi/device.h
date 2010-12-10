/* device.h - Device Configuration VBI */

/* Copyright 2007-2010 Wind River Systems, Inc.
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


/* DESCRIPTION
 *
 * VBI calls used by Guest OSs to discover Guest Devices
 *
 */

#ifndef _VBI_DEVICE_H
#define _VBI_DEVICE_H


#ifndef _ASMLANGUAGE

extern uint32_t vbi_dev_count(void);
extern uint32_t vbi_get_dev(uint32_t deviceIndex,
				struct vb_dev_info **vbiDevInfo);
extern uint32_t vbi_get_dev_interrupt(uint32_t deviceIndex, uint32_t intIndex,
					struct vb_dev_int_info **vbiIntDetails);
extern uint32_t vbi_get_dev_registers(uint32_t deviceIndex,
					uint32_t regSetIndex,
					struct vb_dev_regset_info
						**vbiRegSetDetails);
extern uint32_t vbi_get_dev_device_tree_source(uint32_t deviceIndex,
					uint32_t deviceTreeSourceIndex,
					struct vb_dev_device_tree_source_info
						**vbiDeviceTreeSourceDetails);
#endif

#endif  /* _VBI_DEVICE_H */
