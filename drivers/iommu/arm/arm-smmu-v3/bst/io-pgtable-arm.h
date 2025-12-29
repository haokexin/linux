/* SPDX-License-Identifier: GPL-2.0-only */

/*
 * Copyright (C) 2025 Black Sesame Technologies. All Rights Reserved.
 */
#ifndef IO_PGTABLE_ARM_H_
#define IO_PGTABLE_ARM_H_

#define ARM_LPAE_TCR_TG0_4K		0
#define ARM_LPAE_TCR_TG0_64K		1
#define ARM_LPAE_TCR_TG0_16K		2

#define ARM_LPAE_TCR_TG1_16K		1
#define ARM_LPAE_TCR_TG1_4K		2
#define ARM_LPAE_TCR_TG1_64K		3

#define ARM_LPAE_TCR_SH_NS		0
#define ARM_LPAE_TCR_SH_OS		2
#define ARM_LPAE_TCR_SH_IS		3

#define ARM_LPAE_TCR_RGN_NC		0
#define ARM_LPAE_TCR_RGN_WBWA		1
#define ARM_LPAE_TCR_RGN_WT		2
#define ARM_LPAE_TCR_RGN_WB		3

#define ARM_LPAE_TCR_PS_32_BIT		0x0ULL
#define ARM_LPAE_TCR_PS_36_BIT		0x1ULL
#define ARM_LPAE_TCR_PS_40_BIT		0x2ULL
#define ARM_LPAE_TCR_PS_42_BIT		0x3ULL
#define ARM_LPAE_TCR_PS_44_BIT		0x4ULL
#define ARM_LPAE_TCR_PS_48_BIT		0x5ULL
#define ARM_LPAE_TCR_PS_52_BIT		0x6ULL

/*
 * include/linux/io-pgtable.h, struct io_pgtable_cfg->quirks.
 * BIT8 : SMMU_FEAT_MULTI_OS_S2 : ARM_SMMU_FEAT_MULTI_OS && ARM_SMMU_DOMAIN_S2 is only used by coreip
 * BIT16~31 : streamID
 */
#define SMMU_FEAT_MULTI_OS_S2		BIT(8)

/*
 * Configure this SMMU device to access DDR directly through the CMN.
 * The 'smmu-cmn-dev-offset' property in the device tree is used to enable this direct path.
 */
#define SMMU_FEAT_SPECIAL_CMN_DEV_FLAG		BIT(9)

#endif /* IO_PGTABLE_ARM_H_ */
