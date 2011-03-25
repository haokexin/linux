#ifndef _LINUX_OSCB_H
#define _LINUX_OSCB_H

/*Copyright (c) 2011 Wind River Systems, Inc.

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License version 2 as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

/* The OS Control Block (OSCB) is a block of memory than an execution
 * environment (operating system, executive, hypervisor etc) can use to
 * describe itself to the world. It is particularily useful for tools to
 * adapt how they interact with a target. */

#include <linux/types.h>

#define OSCB_VENDOR_NAME "WR\0"
#define OSCB_OS_NAME "WR Linux\0"
#define OSCB_SIGNATURE 0x05CBBC50
#define OSCB_VERSION 0
#define OSCB_BIG_ENDIAN 0
#define OSCB_LITTLE_ENDIAN 1
#define OSCB_ILP32  0x0
#define OSCB_LP64   0x1
#define OSCB_ILP64  0x2

struct oscb_header {
	uint32_t	signature;

	/* This field identifies the programming model of the compiler.
	   Only the 4 least significant bits are used. The upper bits
	   are reserved. The following values are used:
	       0 = ILP32
	       1 = LP64
	       2 = ILP64
	*/
	uint8_t		programming_model;
	/* 0=big, 1=little */
	uint8_t		endianess;
	uint16_t	version;
	char*		vendor_id;
	char*		os_name;
	char*		os_version;
	void*		data;
};

#endif

