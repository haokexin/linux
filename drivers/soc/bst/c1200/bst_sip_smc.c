// SPDX-License-Identifier: GPL-2.0+
/* 
 * BST SiP Driver
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include "bst_sip_smc.h"

/*
 * This function is used to handle read
 * operation have to excute in EL3
 * x1: address
 */
unsigned long bst_sip_special_address_read(u_int64_t x1)
{
	struct arm_smccc_res res;
	arm_smccc_smc(BST_SIP_SPECIAL_ADDR_READ, x1, 0, 0, 0, 0, 0, 0, &res);
	return res.a0;
}
EXPORT_SYMBOL(bst_sip_special_address_read);
