/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * Copyright (C) 2024 Black Sesame Technologies. Inc.
 */


#ifndef _BST_SECURE_API_H_
#define _BST_SECURE_API_H_

#include <linux/kernel.h>


/* Error Code */
#define SIP_SVC_E_SUCCESS               0
#define SIP_SVC_E_NOT_SUPPORTED         -1
#define SIP_SVC_E_INVALID_PARAMS        -2
#define SIP_SVC_E_INVALID_Range         -3
#define SIP_SVC_E_PERMISSION_DENY       -4

#ifdef CONFIG_ARM64
#define BST_SIP_SMC_AARCH_BIT			0x40000000
#else
#define BST_SIP_SMC_AARCH_BIT			0x00000000
#endif

/*	0x82000200 -	0x820003FF &	0xC2000300 -	0xC20003FF */
/* Debug feature and ATF related */
#define BST_SIP_KERNEL_WDT \
	(0x82000200 | BST_SIP_SMC_AARCH_BIT)
#define BST_SIP_KERNEL_GIC_DUMP \
	(0x82000201 | BST_SIP_SMC_AARCH_BIT)
#define BST_SIP_KERNEL_TIME_SYNC \
	(0x82000202 | BST_SIP_SMC_AARCH_BIT)
#define BST_SIP_KERNEL_AEE_DUMP \
	(0x82000203 | BST_SIP_SMC_AARCH_BIT)
#define BST_SIP_KERNEL_ATF_DEBUG \
	(0x82000204 | BST_SIP_SMC_AARCH_BIT)
#define BST_SIP_KERNEL_DFD \
	(0x82000205 | BST_SIP_SMC_AARCH_BIT)
#define BST_SIP_KERNEL_GET_RND \
	(0x82000206 | BST_SIP_SMC_AARCH_BIT)
#define BST_SIP_KERNEL_DAPC_DUMP \
	(0x82000207 | BST_SIP_SMC_AARCH_BIT)

/* CPU operations */
#define BST_SIP_POWER_DOWN_CLUSTER \
	(0x82000210 | BST_SIP_SMC_AARCH_BIT)
#define BST_SIP_POWER_UP_CLUSTER \
	(0x82000211 | BST_SIP_SMC_AARCH_BIT)
#define BST_SIP_POWER_DOWN_CORE	\
	(0x82000212 | BST_SIP_SMC_AARCH_BIT)
#define BST_SIP_POWER_UP_CORE \
	(0x82000213 | BST_SIP_SMC_AARCH_BIT)
#define BST_SIP_POWER_FLOW_DEBUG \
	(0x82000214 | BST_SIP_SMC_AARCH_BIT)

/* SPM related SMC call */
#define BST_SIP_KERNEL_SPM_SUSPEND_ARGS \
	(0x82000220 | BST_SIP_SMC_AARCH_BIT)
#define BST_SIP_KERNEL_SPM_FIRMWARE_STATUS \
	(0x82000221 | BST_SIP_SMC_AARCH_BIT)
#define BST_SIP_KERNEL_SPM_IRQ0_HANDLER	\
	(0x82000222 | BST_SIP_SMC_AARCH_BIT)
#define BST_SIP_KERNEL_SPM_AP_MDSRC_REQ	\
	(0x82000223 | BST_SIP_SMC_AARCH_BIT)
#define BST_SIP_KERNEL_SPM_PWR_CTRL_ARGS \
	(0x82000224 | BST_SIP_SMC_AARCH_BIT)
#define BST_SIP_KERNEL_SPM_LEGACY_SLEEP	\
	(0x82000225 | BST_SIP_SMC_AARCH_BIT)
#define BST_SIP_KERNEL_SPM_VCOREFS_ARGS	\
	(0x82000226 | BST_SIP_SMC_AARCH_BIT)

/* SPM deepidle related SMC call */
#define BST_SIP_KERNEL_SPM_DPIDLE_ARGS \
	(0x82000227 | BST_SIP_SMC_AARCH_BIT)

/* SPM SODI related SMC call */
#define BST_SIP_KERNEL_SPM_SODI_ARGS \
	(0x82000228 | BST_SIP_SMC_AARCH_BIT)

/* SPM sleep deepidle related SMC call */
#define BST_SIP_KERNEL_SPM_SLEEP_DPIDLE_ARGS \
	(0x82000229 | BST_SIP_SMC_AARCH_BIT)
/* SPM ARGS */
#define BST_SIP_KERNEL_SPM_ARGS	\
	(0x8200022A | BST_SIP_SMC_AARCH_BIT)

/* SPM get pwr_ctrl args */
#define BST_SIP_KERNEL_SPM_GET_PWR_CTRL_ARGS \
	(0x8200022B | BST_SIP_SMC_AARCH_BIT)

/* SPM Check security CG (for deepidle/SODI) */
#define BST_SIP_KERNEL_CHECK_SECURE_CG \
	(0x8200022D | BST_SIP_SMC_AARCH_BIT)

/* DCM SMC call */
#define BST_SIP_KERNEL_DCM \
	(0x82000230 | BST_SIP_SMC_AARCH_BIT)

/* MCDI related SMC call */
#define BST_SIP_KERNEL_MCDI_ARGS \
	(0x82000240 | BST_SIP_SMC_AARCH_BIT)

/* AMMS related SMC call */
#define BST_SIP_KERNEL_AMMS_GET_FREE_ADDR \
	(0x82000250 | BST_SIP_SMC_AARCH_BIT)
#define BST_SIP_KERNEL_AMMS_GET_FREE_LENGTH \
	(0x82000251 | BST_SIP_SMC_AARCH_BIT)

/* MPU */
#define BST_SIP_KERNEL_EMIMPU_WRITE (0x82000260 | BST_SIP_SMC_AARCH_BIT)
#define BST_SIP_KERNEL_EMIMPU_READ  (0x82000261 | BST_SIP_SMC_AARCH_BIT)
#define BST_SIP_KERNEL_EMIMPU_SET \
	(0x82000262 | BST_SIP_SMC_AARCH_BIT)
#define BST_SIP_KERNEL_EMIMPU_CLEAR \
	(0x82000263 | BST_SIP_SMC_AARCH_BIT)
#define BST_SIP_KERNEL_DEVMPU_VIO_GET \
	(0x82000264 | BST_SIP_SMC_AARCH_BIT)
#define BST_SIP_KERNEL_DEVMPU_PERM_GET \
	(0x82000265 | BST_SIP_SMC_AARCH_BIT)

/* Storage Encryption related SMC call */
/* HW FDE related SMC call */
#define BST_SIP_KERNEL_HW_FDE_UFS_CTL \
	(0x82000270 | BST_SIP_SMC_AARCH_BIT)
#define BST_SIP_KERNEL_HW_FDE_AES_INIT \
	(0x82000271 | BST_SIP_SMC_AARCH_BIT)
#define BST_SIP_KERNEL_HW_FDE_KEY   \
	(0x82000272 | BST_SIP_SMC_AARCH_BIT)
#define BST_SIP_KERNEL_HW_FDE_MSDC_CTL \
	(0x82000273 | BST_SIP_SMC_AARCH_BIT)
/* HIE related SMC call */
#define BST_SIP_KERNEL_CRYPTO_HIE_CFG_REQUEST \
	(0x82000274 | BST_SIP_SMC_AARCH_BIT)
#define BST_SIP_KERNEL_CRYPTO_HIE_INIT \
	(0x82000275 | BST_SIP_SMC_AARCH_BIT)
/* UFS generic SMC call */
#define BST_SIP_KERNEL_UFS_CTL \
	(0x82000276 | BST_SIP_SMC_AARCH_BIT)
/* Cache related SMC call */
#define BST_SIP_KERNEL_CACHE_FLUSH_FIQ \
	(0x82000280 | BST_SIP_SMC_AARCH_BIT)

#define BST_SIP_KERNEL_CACHE_FLUSH_INIT \
	(0x82000281 | BST_SIP_SMC_AARCH_BIT)

/* SCP DVFS related SMC call */
#define BST_SIP_KERNEL_SCP_DVFS_CTRL \
	(0x82000290 | BST_SIP_SMC_AARCH_BIT)

/* Pheripheral related SMC call */
#define BST_SIP_KERNEL_I2C_SEC_WRITE \
	(0x820002A0 | BST_SIP_SMC_AARCH_BIT)

extern size_t bst_secure_call_all(size_t function_id,
	size_t arg0, size_t arg1, size_t arg2,
	size_t arg3, size_t *r1, size_t *r2, size_t *r3);

#define bst_secure_call(_fun_id, _arg0, _arg1, _arg2, _arg3) \
	bst_secure_call_all(_fun_id, _arg0, _arg1, _arg2, _arg3, 0, 0, 0)
#define bst_secure_call_ret1(_fun_id, _arg0, _arg1, _arg2, _arg3) \
	bst_secure_call_all(_fun_id, _arg0, _arg1, _arg2, _arg3, 0, 0, 0)
#define bst_secure_call_ret2(_fun_id, _arg0, _arg1, _arg2, _arg3, _r1) \
	bst_secure_call_all(_fun_id, _arg0, _arg1, _arg2, _arg3, _r1, 0, 0)
#define bst_secure_call_ret3(_fun_id, _arg0, _arg1, _arg2, _arg3, _r1, _r2) \
	bst_secure_call_all(_fun_id, _arg0, _arg1, _arg2, _arg3, _r1, _r2, 0)
#define bst_secure_call_ret4(_fun_id, _arg0,\
	_arg1, _arg2, _arg3, _r1, _r2, _r3) \
	bst_secure_call_all(_fun_id, _arg0, _arg1, _arg2, _arg3, _r1, _r2, _r3)

#endif				/* _BST_SECURE_API_H_ */

#define emi_mpu_smc_write(offset, val) \
bst_secure_call(BST_SIP_KERNEL_EMIMPU_WRITE, offset, val, 0, 0)

#define emi_mpu_smc_read(offset) \
bst_secure_call(BST_SIP_KERNEL_EMIMPU_READ, offset, 0, 0, 0)

#ifdef CONFIG_ARM64
#define emi_mpu_smc_set(start, end, region_permission) \
bst_secure_call(BST_SIP_KERNEL_EMIMPU_SET, start, end, region_permission, 0)
#else
#define emi_mpu_smc_set(start, end, apc8, apc0) \
bst_secure_call_all(BST_SIP_KERNEL_EMIMPU_SET,\
start, end, apc8, apc0, 0, 0, 0, 0)
#endif

#define emi_mpu_smc_clear(region) \
bst_secure_call(BST_SIP_KERNEL_EMIMPU_CLEAR, region, 0, 0, 0)

#define emi_mpu_smc_protect(start, end, apc) \
bst_secure_call(BST_SIP_KERNEL_EMIMPU_SET, start, end, apc, 0)
