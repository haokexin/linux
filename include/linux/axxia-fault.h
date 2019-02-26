/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (C) 2021 INTEL Corporation
 */

#ifndef __DRIVERS_MISC_AXXIA_FAULT_H
#define __DRIVERS_MISC_AXXIA_FAULT_H

asmlinkage int axxia_x9xlf_fault(struct pt_regs *, int, unsigned long);

int axxia_fault_get_mask(void);
void axxia_fault_set_mask(int new_mask);

#endif /* __DRIVERS_MISC_AXXIA_FAULT_H */
