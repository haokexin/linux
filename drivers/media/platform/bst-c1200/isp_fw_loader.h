/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef _ISP_FW_LOADER_H_
#define _ISP_FW_LOADER_H_
#include <dt-bindings/media/bst-isp.h>
#include "isp_core.h"

#define ISP_FW_BIN_PATH	    "isp/core_isp.exe"
#define ISP_FW_SLAB_PATH    "isp/common/slab.bin"
#define TEXT_SECTION_NAME   (".text")
#define RODATA_SECTION_NAME (".rodata")

#define ISP_PACK_OFFSET	      0X80
#define ISP_SVN_OFFSET	      0x84
#define ISP_BUILD_DATE_OFFSET 0x88
#define ISP_SLAB_OFFSET	      0x80

#define NS_PER_SECOND 1000000000
#define ISP_CORE_TOP_R_BASE   (0x52030000)
#define ISP_CORE_TOP_R_SIZE   (0x174)
#define REG_SRC_EN	      (0x100)
#define REG_INNER_SRC1_PERIOD (0x104)
#define REG_OUTER_SRC1_PERIOD (0x10C)
#define REG_PULSE0_WIDTH      (0x11C)
#define REG_INTX_SEL	      (0x15C)
#define REG_FSYNX_OEN	      (0x160)
#define REG_DECREASE_PERIOD1  (0x16C)
#define REG_ORG_MULT (0x3b0)
/* 1 counter is 2.5 ns, / 25 MUST be prior to avoid overflow */
#define FREQ_TO_PERIOD(freq)	  (NS_PER_SECOND / 167 * 100 / (freq))
#define NS_TO_PERIOD(ns)	  ((ns) / 167 * 100)
#define FSYNC_PIN_MIN		  (0)
#define FSYNC_PIN_MAX		  (7)
#define FSYNC_DEFAULT_PULSE_WIDTH (4000)
#define FSYNC_INNER_SRC_MIN	  (0)
#define FSYNC_INNER_SRC_MAX	  (1)
#define FSYNC_OUTER_SRC_MIN	  (2)
#define FSYNC_OUTER_SRC_MAX	  (5)
int bst_load_dsp_fw(const char *fw_name, struct c1200_isp_device *isp);
void bst_start_dsp_fw(struct c1200_isp_device *isp);
int bst_load_isp_fw(const char *fw_name, const char *slab_name,
		    struct c1200_isp_device *isp);
void bst_start_isp_fw(struct c1200_isp_device *isp);
int bst_boot_isp_fw(const char *fw_name, const char *slab_name,
		    struct c1200_isp_device *isp);
int isp_internal_trigger(struct c1200_isp_device *isp, int target_freq, int fsync_out, uint32_t pulse_width,
						 int fsync_source);
int isp_external_trigger(struct c1200_isp_device *isp, int external_freq, int target_freq, int fsync_in,
						 int fsync_out, uint32_t pulse_width, int fsync_source);
int isp_fsync_work(struct c1200_isp_device *isp_dev);
#endif
