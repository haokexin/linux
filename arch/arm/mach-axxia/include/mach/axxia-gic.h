/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (C) 2021 INTEL Corporation
 */

#ifndef __AXXIA_GIC_H
#define __AXXIA_GIC_H

void axxia_gic_raise_softirq(const struct cpumask *mask, unsigned int irq);
void axxia_gic_secondary_init(void);
int __init axxia_gic_of_init(struct device_node *node,
			     struct device_node *parent);

#endif
