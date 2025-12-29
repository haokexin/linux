// SPDX-License-Identifier: GPL-2.0+
/* 
 * BST SiP Driver
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
#include <linux/arm-smccc.h>
#define BST_SIP_SPECIAL_ADDR_READ          0xc2000003

unsigned long bst_sip_special_address_rw(u_int64_t x1);
