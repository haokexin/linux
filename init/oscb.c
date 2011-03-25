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

#include <linux/oscb.h>
#include <generated/autoconf.h>
#include <generated/utsrelease.h>

#if defined(CONFIG_ARM)
#ifdef __ARMEB__
#define OSCB_ENDIAN OSCB_BIG_ENDIAN
#else
#define OSCB_ENDIAN OSCB_LITTLE_ENDIAN
#endif
#endif /* CONFIG_ARM */

#if defined(CONFIG_MIPS)
#if defined(__MIPSEB__)
#define OSCB_ENDIAN OSCB_BIG_ENDIAN
#else defined(__MIPSEL__)
#define OSCB_ENDIAN OSCB_LITTLE_ENDIAN
#endif
#endif

#if defined(CONFIG_X86)
#define OSCB_ENDIAN OSCB_LITTLE_ENDIAN
#endif

#if defined(CONFIG_PPC32)
#define OSCB_ENDIAN OSCB_BIG_ENDIAN
#endif

#if defined(CONFIG_PPC64)
#define OSCB_ENDIAN OSCB_BIG_ENDIAN
#endif

#ifndef OSCB_ENDIAN
#warning WARNING. defining a default endianess
#define OSCB_ENDIAN OSCB_LITTLE_ENDIAN
#endif

/* This table lists the breakdown of sizes in the various programming models.
Datatype	LP64	ILP64	LLP64	ILP32	LP32
char	          8	 8	 8	 8	 8
short	         16	 16	 16	 16	 16
_int	         32	 --	 32	 --	 --
int	         32	 64	 32	 32	 16
long	         64	 64	 32	 32	 32
long long	 --	 --	 64	 --	 --
pointer	         64	 64	 64	 32	 32
*/
#if BITS_PER_LONG == 32
#define OSCB_MODEL OSCB_ILP32
#else
#define OSCB_MODEL OSCB_ILP64
#endif

struct oscb_header __oscb_hdr = {
	.signature		= OSCB_SIGNATURE,
	.programming_model	= OSCB_MODEL,
	.endianess		= OSCB_ENDIAN,
	.version		= OSCB_VERSION,
	.os_name		= OSCB_OS_NAME,
	.os_version		= UTS_RELEASE,
	.data			= NULL
};


