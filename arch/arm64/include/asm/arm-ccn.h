/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Author: Marek Bykowski <marek.bykowski@intel.com>
 *
 * Copyright (C) 2021 INTEL Corporation
 */

#ifndef __ASM_ARM_CCN_H
#define __ASM_ARM_CCN_H

static inline bool platform_has_secure_ccn_access(void)
{
	return false;
}

#endif
