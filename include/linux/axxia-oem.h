/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (C) 2021 INTEL Corporation
 */

#ifndef __DRIVERS_MISC_AXXIA_DSPC_H
#define __DRIVERS_MISC_AXXIA_DSPC_H

/*
 * DSP Cluster Control -- Only on 6700
 */

unsigned long axxia_dspc_get_state(void);
void axxia_dspc_set_state(unsigned long a);

/*
 * ACTLR_EL3/ACTLR_EL2 Access -- For Performance Testing
 */

unsigned long axxia_actlr_el3_get(void);
void axxia_actlr_el3_set(unsigned long a);
unsigned long axxia_actlr_el2_get(void);
void axxia_actlr_el2_set(unsigned long a);

/*
 * CCN504 (5600)
 * CCN512 (6700)
 *
 * 64 bit registers.
 */

unsigned long axxia_ccn_get(unsigned int offset);
void axxia_ccn_set(unsigned int offset, unsigned long value);

#endif /* __DRIVERS_MISC_AXXIA_DSPC_H */
