/*
 * arch/arm/mach-spear13xx/include/mach/spear_pcie_rev_370.h
 *
 * Copyright (C) 2010-2011 ST Microelectronics
 * Pratyush Anand <pratyush.anand@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#ifndef __MACH_SPEAR_PCIE_REV_370_H
#define __MACH_SPEAR_PCIE_REV_370_H
#include "spear_pcie.h"

/*CR3 ID*/
#define XMLH_LTSSM_STATE_ID			0
#define XMLH_LTSSM_STATE_L0	((u32)0x11 << XMLH_LTSSM_STATE_ID)
#define XMLH_LTSSM_STATE_MASK	((u32)0x3F << XMLH_LTSSM_STATE_ID)
#define XMLH_LINK_UP_ID				6

#endif
