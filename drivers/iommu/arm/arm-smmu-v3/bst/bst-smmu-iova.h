/* SPDX-License-Identifier: GPL-2.0 */
/*
 * IOVA API for bst smmu implementations.
 *
 * Copyright (C) 2025 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef _BST_SMMU_IOVA_H
#define _BST_SMMU_IOVA_H

#ifdef CONFIG_COREIP_SMMMU_MULTIOS_BST
#include "msgbox/smmu-client/src-gen/smmu_client.h"
extern int smmu_msgbox_init(void);
extern void smmu_msgbox_exit(void);
#else
static inline int smmu_msgbox_init(void) { return -1; }
static inline void smmu_msgbox_exit(void) { }
#endif

#endif /* _BST_SMMU_IOVA_H */
