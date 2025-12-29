/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_ISP_HW_H__
#define __BST_ISP_HW_H__

#include <dt-bindings/media/bst-isp.h>

#include <linux/dma-mapping.h>

struct isp_device;

#define DMA_MASK	       DMA_BIT_MASK(32)
#define DMA_MAX_SIZE	       DMA_MASK
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
#define IOVA_ALIGN_SIZE	       (2 * 1024 * 1024)
#else
#define IOVA_ALIGN_SIZE	       (PAGE_SIZE)
#endif
#define IOVA_FIXED_LOW	       (0x40000000)
#define IOVA_FIXED_HIGH	       (0x5FFFFFFF)
#define OUTER_REG_START	       (2)
#define OUTER_REG_GROUP	       (2)
#define MAX_CSI_DEVICE_NUM     (3)
#define MAX_ISP_CORE	       (4)
#define MAX_ISP_CHANNEL	       (16)
/* NOTE: Aligned to firmware, this contains PDNS view */
#define VIEW_ID_NUM_PER_CAMERA (4)
#define VIEW_ID_CHANNEL0_VIEW0 (1)
#define VIEW_ID_CHANNEL0_RAW0 \
	(VIEW_ID_CHANNEL0_VIEW0 + (MAX_ISP_CHANNEL * VIEW_ID_NUM_PER_CAMERA))
#define MAX_VIEWS_PER_VIDEO    (3)
#define ISP_BURST_BYTES	       (16)
#define ISP_PACK_BYTES	       (256)
#define ISP_META_BYTES	       (1024)
#define ISP_BUFFER_ALIGN(size) (ALIGN((size), PAGE_SIZE))
#define GTC_TO_NS(counter)     ((counter) / 5)
#ifdef CONFIG_BST_HEALTH_MONITOR
#define PSM_BLOCK_ID_ISP_BASE (0x3E)
#define PSM_BLOCK_CFG_SIZE    (4)
#endif

/* Firmware and elf file definitions */
#define ISP_FW_BIN_PATH	      "isp/core_isp.exe"
#define ISP_FW_SLAB_PATH      "isp/common/slab.bin"
#define ISP_FW_BOOT_TIME      (5000) /* by ms */
#define ELF_CODE_SEC_NAME     (".text")
#define FW_VERSION_OFFSET     (0x80)
#define FW_SCM_ID_OFFSET      (0x84)
#define FW_BUILD_DATE_OFFSET  (0x88)
#define FW_VER_MAJOR(ver)     (((ver) >> 24) & 0xFF)
#define FW_VER_MINOR(ver)     (((ver) >> 16) & 0xFF)
#define FW_VER_PATCH(ver)     (((ver) >> 8) & 0xFF)
#define FW_VER_CUST(ver)      ((ver) & 0xFF)
/* year: [31:16], month: [15:8], day: [7:0] */
#define FW_DATE_YEAR(date)    (((date) >> 16) & 0xFFFF)
#define FW_DATE_MONTH(date)   (((date) >> 8) & 0xFF)
#define FW_DATE_DAY(date)     ((date) & 0xFF)

/* ISP fsync funtion definitions */
#define SIZE_1M			  (1000 * 1000)
#define ISP_CLK_PERIOD		  (600 * SIZE_1M)
#define NS_PER_SECOND		  (1000 * 1000 * 1000)
#define FREQ_TO_PERIOD(freq)	  (ISP_CLK_PERIOD / (freq))
/* It should be (ns * ISP_CLK_PERIOD / NS_PER_SECOND)
 * We use this just for avoid overflow for 32 bits' interger
 */
#define NS_TO_PERIOD(ns) \
	((ns) * (ISP_CLK_PERIOD / SIZE_1M) / (NS_PER_SECOND / SIZE_1M))
#define FSYNC_PIN_MIN		  (0)
#define FSYNC_PIN_MAX		  (7)
#define FSYNC_PIN_INVALID	  (FSYNC_PIN_MAX + 1)
#define FSYNC_DEFAULT_PULSE_WIDTH (4000)
#define FSYNC_DEFAULT_SOC_FPS	  (1)
#define FSYNC_INNER_SRC_MIN	  (FSYNC_INNER0)
#define FSYNC_INNER_SRC_MAX	  (FSYNC_INNER1)
#define FSYNC_OUTER_SRC_MIN	  (FSYNC_OUTER0)
#define FSYNC_OUTER_SRC_MAX	  (FSYNC_OUTER3)
#define FSYNC_OUTER_SRC_INVALID	  (FSYNC_OUTER_SRC_MAX + 1)

/* Register offset definitions */
#define R_TOP_CLK_EN		  (0x0000)
#define R_TOP_SW_RST		  (0x0004)
#define R_TOP_PARITY_ECC	  (0x0030)
#define R_TOP_RSV_BUF		  (0x0084)
#define R_TOP_UID		  (0x008C)
#define R_TOP_MAILBOX_IN	  (0x0090)
#define R_TOP_MAILBOX_OUT	  (0x0094)
#define R_TOP_IPC_ID		  (0x0098)
#define R_FSYNC_SRC_EN		  (0x0100)
#define R_FSYNC_INNER_SRC1_PERIOD (0x0104)
#define R_FSYNC_OUTER_SRC1_PERIOD (0x010C)
#define R_FSYNC_PULSE0_WIDTH	  (0x011C)
#define R_FSYNC_INTX_SEL	  (0x015C)
#define R_FSYNC_FSYNX_OEN	  (0x0160)
#define R_FSYNC_DECREASE_PERIOD1  (0x016C)
#define R_FSYNC_SEL_SOC_FSYNC	  (0x01F0)
#define R_OUT_ORG1_MULT		  (0x03B0)
/* Register config definitions */
#define CLK_EN_BITS		  (0x1FFFFFFF)
#define RESET_DEASSERT_BITS	  (0x71FFFFFF)
#define MEM_ECC_BITS		  (0x20202020)
#define MEM_PARITY_BITS		  (0x40404040)
#define CTRL_TIMEOUT_BITS	  (0x80808080)

static inline bool isp_is_fixed_iova(u32 iova)
{
	return (iova >= IOVA_FIXED_LOW && iova <= IOVA_FIXED_HIGH);
}

bool isp_hw_has_inited(struct isp_device *isp);
int isp_fw_init(struct isp_device *isp);
void isp_fw_exit(struct isp_device *isp);
int isp_fsync_setup(struct isp_device *isp);

#endif /* __BST_ISP_HW_H__ */
