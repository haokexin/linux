/* SPDX-License-Identifier: GPL-2.0 */
/*
 * arch/arm64/include/asm/arm-ccn.h
 *
 * Author: Marek Bykowski <marek.bykowski@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ASM_ARM_CCN_H
#define __ASM_ARM_CCN_H

static inline bool platform_has_secure_ccn_access(void)
{
	return false;
}

#endif
