/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef _ISP_SYSFILE_H_
#define _ISP_SYSFILE_H_

#include "isp_core.h"

#define TEXT_SECTION_NAME   (".text")
#define RODATA_SECTION_NAME (".rodata")

enum {
	ISP_FW_LOAD_RUN = 0,
	ISP_ECHO_TEST,
	ISP_CATCH_TEST,
};

int isp_sysfs_init(struct c1200_isp_device *isp);
void isp_echo_test_callback(void);
#endif
