/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (C) 2021 INTEL Corporation
 */

#ifndef __AXXIA_PEI_H
#define __AXXIA_PEI_H

int axxia_pei_reset(unsigned int a);
int axxia_pei_setup(unsigned int a, unsigned int b);
int axxia_pcie_reset(void);

unsigned int axxia_pei_get_control(void);
int axxia_pei_is_control_set(void);

#endif	/* __AXXIA_PEI_H */
