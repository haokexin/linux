// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#ifndef _BST_VOUT_CONN_H_
#define _BST_VOUT_CONN_H_

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/irq.h>
#include <drm/drm_device.h>
#include <drm/drm_connector.h>
#include <drm/drm_drv.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_fixed.h>
#include <drm/drm_simple_kms_helper.h>
#include <linux/regmap.h>
#include <linux/of_device.h>
#include <linux/clk.h>

#define FIMD_PORT_RGB 0
#define VOUT_TIMING_NUM	(2)

/*********************************************************************
 *                   Display Out Pinmux
 ********************************************************************/
// #define PMM_SW_IO
#ifdef PMM_SW_IO  /*SW IO*/
#define VOUT_PMM_REG0               (0x0)
#define VOUT_PMM_REG1               (0x4)
#define VOUT_PMM_REG2               (0x8)
#define VOUT_PMM_REG3               (0xC)
#define VOUT_PMM_PIXCLK_REG         (VOUT_PMM_REG0)
#define VOUT_PMM_PIXCLK_VAL         (0x3 << 0)
#define VOUT_PMM_PIXCLK_MASK        (0x7 << 0)  // 0~2
#define VOUT_PMM_DATAEN_REG         (VOUT_PMM_REG0)
#define VOUT_PMM_DATAEN_VAL         (0x3 << 3)
#define VOUT_PMM_DATAEN_MASK        (0x7 << 3)  // 3~5
#define VOUT_PMM_VSYNC_REG          (VOUT_PMM_REG0)
#define VOUT_PMM_VSYNC_VAL          (0x3 << 6)
#define VOUT_PMM_VSYNC_MASK         (0x7 << 6)  // 6~8
#define VOUT_PMM_HSYNC_REG          (VOUT_PMM_REG0)
#define VOUT_PMM_HSYNC_VAL          (0x3 << 9)
#define VOUT_PMM_HSYNC_MASK         (0x7 << 9)  // 9~11
//VOUT pixel data red channel
#define VOUT_PMM_PIXDATA_R0_REG     (VOUT_PMM_REG0)
#define VOUT_PMM_PIXDATA_R1_REG     (VOUT_PMM_REG0)
#define VOUT_PMM_PIXDATA_R2_REG     (VOUT_PMM_REG0)
#define VOUT_PMM_PIXDATA_R3_REG     (VOUT_PMM_REG0)
#define VOUT_PMM_PIXDATA_R4_REG     (VOUT_PMM_REG0)
#define VOUT_PMM_PIXDATA_R5_REG     (VOUT_PMM_REG0)
#define VOUT_PMM_PIXDATA_R6_REG     (VOUT_PMM_REG1)
#define VOUT_PMM_PIXDATA_R7_REG     (VOUT_PMM_REG1)
#define VOUT_PMM_PIXDATA_R8_REG     (VOUT_PMM_REG1)
#define VOUT_PMM_PIXDATA_R9_REG     (VOUT_PMM_REG1)
#define VOUT_PMM_PIXDATA_R0_MASK    (0x7 << 12)  // 12~14
#define VOUT_PMM_PIXDATA_R1_MASK    (0x7 << 15)  // 15~17
#define VOUT_PMM_PIXDATA_R2_MASK    (0x7 << 18)  // 18~20
#define VOUT_PMM_PIXDATA_R3_MASK    (0x7 << 21)  // 21~23
#define VOUT_PMM_PIXDATA_R4_MASK    (0x7 << 24)  // 24~26
#define VOUT_PMM_PIXDATA_R5_MASK    (0x7 << 27)  // 27~29
#define VOUT_PMM_PIXDATA_R6_MASK    (0x7 << 0)   // 0~2
#define VOUT_PMM_PIXDATA_R7_MASK    (0x7 << 3)   // 3~5
#define VOUT_PMM_PIXDATA_R8_MASK    (0x7 << 6)   // 6~8
#define VOUT_PMM_PIXDATA_R9_MASK    (0x7 << 9)   // 9~11
#define VOUT_PMM_PIXDATA_R0_VAL     (0x3 << 12)
#define VOUT_PMM_PIXDATA_R1_VAL     (0x3 << 15)
#define VOUT_PMM_PIXDATA_R2_VAL     (0x3 << 18)
#define VOUT_PMM_PIXDATA_R3_VAL     (0x3 << 21)
#define VOUT_PMM_PIXDATA_R4_VAL     (0x3 << 24)
#define VOUT_PMM_PIXDATA_R5_VAL     (0x3 << 27)
#define VOUT_PMM_PIXDATA_R6_VAL     (0x3 << 0)
#define VOUT_PMM_PIXDATA_R7_VAL     (0x3 << 3)
#define VOUT_PMM_PIXDATA_R8_VAL     (0x3 << 6)
#define VOUT_PMM_PIXDATA_R9_VAL     (0x3 << 9)
//VOUT pixel data green channel
#define VOUT_PMM_PIXDATA_G0_REG     (VOUT_PMM_REG1)
#define VOUT_PMM_PIXDATA_G1_REG     (VOUT_PMM_REG1)
#define VOUT_PMM_PIXDATA_G2_REG     (VOUT_PMM_REG1)
#define VOUT_PMM_PIXDATA_G3_REG     (VOUT_PMM_REG1)
#define VOUT_PMM_PIXDATA_G4_REG     (VOUT_PMM_REG1)
#define VOUT_PMM_PIXDATA_G5_REG     (VOUT_PMM_REG1)
#define VOUT_PMM_PIXDATA_G6_REG     (VOUT_PMM_REG2)
#define VOUT_PMM_PIXDATA_G7_REG     (VOUT_PMM_REG2)
#define VOUT_PMM_PIXDATA_G8_REG     (VOUT_PMM_REG2)
#define VOUT_PMM_PIXDATA_G9_REG     (VOUT_PMM_REG2)
#define VOUT_PMM_PIXDATA_G0_MASK    (0x7 << 12)  // 12~14
#define VOUT_PMM_PIXDATA_G1_MASK    (0x7 << 15)  // 15~17
#define VOUT_PMM_PIXDATA_G2_MASK    (0x7 << 18)  // 18~20
#define VOUT_PMM_PIXDATA_G3_MASK    (0x7 << 21)  // 21~23
#define VOUT_PMM_PIXDATA_G4_MASK    (0x7 << 24)  // 24~26
#define VOUT_PMM_PIXDATA_G5_MASK    (0x7 << 27)  // 27~29
#define VOUT_PMM_PIXDATA_G6_MASK    (0x7 << 0)   // 0~2
#define VOUT_PMM_PIXDATA_G7_MASK    (0x7 << 3)   // 3~5
#define VOUT_PMM_PIXDATA_G8_MASK    (0x7 << 6)   // 6~8
#define VOUT_PMM_PIXDATA_G9_MASK    (0x7 << 9)   // 9~11
#define VOUT_PMM_PIXDATA_G0_VAL     (0x3 << 12)
#define VOUT_PMM_PIXDATA_G1_VAL     (0x3 << 15)
#define VOUT_PMM_PIXDATA_G2_VAL     (0x3 << 18)
#define VOUT_PMM_PIXDATA_G3_VAL     (0x3 << 21)
#define VOUT_PMM_PIXDATA_G4_VAL     (0x3 << 24)
#define VOUT_PMM_PIXDATA_G5_VAL     (0x3 << 27)
#define VOUT_PMM_PIXDATA_G6_VAL     (0x3 << 0)
#define VOUT_PMM_PIXDATA_G7_VAL     (0x3 << 3)
#define VOUT_PMM_PIXDATA_G8_VAL     (0x3 << 6)
#define VOUT_PMM_PIXDATA_G9_VAL     (0x3 << 9)
//VOUT pixel data blue channel
#define VOUT_PMM_PIXDATA_B0_REG      (VOUT_PMM_REG2)
#define VOUT_PMM_PIXDATA_B1_REG      (VOUT_PMM_REG2)
#define VOUT_PMM_PIXDATA_B2_REG      (VOUT_PMM_REG2)
#define VOUT_PMM_PIXDATA_B3_REG      (VOUT_PMM_REG2)
#define VOUT_PMM_PIXDATA_B4_REG      (VOUT_PMM_REG2)
#define VOUT_PMM_PIXDATA_B5_REG      (VOUT_PMM_REG2)
#define VOUT_PMM_PIXDATA_B6_REG      (VOUT_PMM_REG3)
#define VOUT_PMM_PIXDATA_B7_REG      (VOUT_PMM_REG3)
#define VOUT_PMM_PIXDATA_B8_REG      (VOUT_PMM_REG3)
#define VOUT_PMM_PIXDATA_B9_REG      (VOUT_PMM_REG3)
#define VOUT_PMM_PIXDATA_B0_MASK     (0x7 << 12)  // 12~14
#define VOUT_PMM_PIXDATA_B1_MASK     (0x7 << 15)  // 15~17
#define VOUT_PMM_PIXDATA_B2_MASK     (0x7 << 18)  // 18~20
#define VOUT_PMM_PIXDATA_B3_MASK     (0x7 << 21)  // 21~23
#define VOUT_PMM_PIXDATA_B4_MASK     (0x7 << 24)  // 24~26
#define VOUT_PMM_PIXDATA_B5_MASK     (0x7 << 27)  // 27~29
#define VOUT_PMM_PIXDATA_B6_MASK     (0x7 << 0)   // 0~2
#define VOUT_PMM_PIXDATA_B7_MASK     (0x7 << 3)   // 3~5
#define VOUT_PMM_PIXDATA_B8_MASK     (0x7 << 6)   // 6~8
#define VOUT_PMM_PIXDATA_B9_MASK     (0x7 << 9)   // 9~11
#define VOUT_PMM_PIXDATA_B0_VAL      (0x3 << 12)
#define VOUT_PMM_PIXDATA_B1_VAL      (0x3 << 15)
#define VOUT_PMM_PIXDATA_B2_VAL      (0x3 << 18)
#define VOUT_PMM_PIXDATA_B3_VAL      (0x3 << 21)
#define VOUT_PMM_PIXDATA_B4_VAL      (0x3 << 24)
#define VOUT_PMM_PIXDATA_B5_VAL      (0x3 << 27)
#define VOUT_PMM_PIXDATA_B6_VAL      (0x3 << 0)
#define VOUT_PMM_PIXDATA_B7_VAL      (0x3 << 3)
#define VOUT_PMM_PIXDATA_B8_VAL      (0x3 << 6)
#define VOUT_PMM_PIXDATA_B9_VAL      (0x3 << 9)
#else   /*WK IO*/
#define VOUT_PMM_REG0                (0x34)
#define VOUT_PMM_REG1                (0x38)
#define VOUT_PMM_REG2                (0x3C)
#define VOUT_PMM_REG3                (0x40)
#define VOUT_PMM_REG4                (0x44)
#define VOUT_PMM_PIXCLK_REG          (VOUT_PMM_REG0)
#define VOUT_PMM_PIXCLK_VAL          (0x1 << 27)
#define VOUT_PMM_PIXCLK_MASK         (0x7 << 27)  // 27~29
#define VOUT_PMM_DATAEN_REG          (VOUT_PMM_REG1)
#define VOUT_PMM_DATAEN_VAL          (0x1 << 0)
#define VOUT_PMM_DATAEN_MASK         (0x7 << 0)   // 0~2
#define VOUT_PMM_HSYNC_REG           (VOUT_PMM_REG1)
#define VOUT_PMM_HSYNC_VAL           (0x1 << 3)
#define VOUT_PMM_HSYNC_MASK          (0x7 << 3)   // 3~5
#define VOUT_PMM_VSYNC_REG           (VOUT_PMM_REG1)
#define VOUT_PMM_VSYNC_VAL           (0x1 << 6)
#define VOUT_PMM_VSYNC_MASK          (0x7 << 6)  // 6~8
//VOUT pixel data red channel
#define VOUT_PMM_PIXDATA_R0_REG      (VOUT_PMM_REG1)
#define VOUT_PMM_PIXDATA_R1_REG      (VOUT_PMM_REG1)
#define VOUT_PMM_PIXDATA_R2_REG      (VOUT_PMM_REG1)
#define VOUT_PMM_PIXDATA_R3_REG      (VOUT_PMM_REG1)
#define VOUT_PMM_PIXDATA_R4_REG      (VOUT_PMM_REG1)
#define VOUT_PMM_PIXDATA_R5_REG      (VOUT_PMM_REG1)
#define VOUT_PMM_PIXDATA_R6_REG      (VOUT_PMM_REG1)
#define VOUT_PMM_PIXDATA_R7_REG      (VOUT_PMM_REG2)
#define VOUT_PMM_PIXDATA_R8_REG      (VOUT_PMM_REG2)
#define VOUT_PMM_PIXDATA_R9_REG      (VOUT_PMM_REG2)
#define VOUT_PMM_PIXDATA_R0_MASK     (0x7 << 9)   // 9~11
#define VOUT_PMM_PIXDATA_R1_MASK     (0x7 << 12)  // 12~14
#define VOUT_PMM_PIXDATA_R2_MASK     (0x7 << 15)  // 15~17
#define VOUT_PMM_PIXDATA_R3_MASK     (0x7 << 18)  // 18~20
#define VOUT_PMM_PIXDATA_R4_MASK     (0x7 << 21)  // 21~23
#define VOUT_PMM_PIXDATA_R5_MASK     (0x7 << 24)  // 24~26
#define VOUT_PMM_PIXDATA_R6_MASK     (0x7 << 27)  // 27~29
#define VOUT_PMM_PIXDATA_R7_MASK     (0x7 << 0)   // 0~2
#define VOUT_PMM_PIXDATA_R8_MASK     (0x7 << 3)   // 3~5
#define VOUT_PMM_PIXDATA_R9_MASK     (0x7 << 6)   // 6~8
#define VOUT_PMM_PIXDATA_R0_VAL      (0x1 << 9)
#define VOUT_PMM_PIXDATA_R1_VAL      (0x1 << 12)
#define VOUT_PMM_PIXDATA_R2_VAL      (0x1 << 15)
#define VOUT_PMM_PIXDATA_R3_VAL      (0x1 << 18)
#define VOUT_PMM_PIXDATA_R4_VAL      (0x1 << 21)
#define VOUT_PMM_PIXDATA_R5_VAL      (0x1 << 24)
#define VOUT_PMM_PIXDATA_R6_VAL      (0x1 << 27)
#define VOUT_PMM_PIXDATA_R7_VAL      (0x1 << 0)
#define VOUT_PMM_PIXDATA_R8_VAL      (0x1 << 3)
#define VOUT_PMM_PIXDATA_R9_VAL      (0x1 << 6)
//VOUT pixel data green channel
#define VOUT_PMM_PIXDATA_G0_REG      (VOUT_PMM_REG2)
#define VOUT_PMM_PIXDATA_G1_REG      (VOUT_PMM_REG2)
#define VOUT_PMM_PIXDATA_G2_REG      (VOUT_PMM_REG2)
#define VOUT_PMM_PIXDATA_G3_REG      (VOUT_PMM_REG2)
#define VOUT_PMM_PIXDATA_G4_REG      (VOUT_PMM_REG2)
#define VOUT_PMM_PIXDATA_G5_REG      (VOUT_PMM_REG2)
#define VOUT_PMM_PIXDATA_G6_REG      (VOUT_PMM_REG2)
#define VOUT_PMM_PIXDATA_G7_REG      (VOUT_PMM_REG3)
#define VOUT_PMM_PIXDATA_G8_REG      (VOUT_PMM_REG3)
#define VOUT_PMM_PIXDATA_G9_REG      (VOUT_PMM_REG3)
#define VOUT_PMM_PIXDATA_G0_MASK     (0x7 << 9)   // 9~11
#define VOUT_PMM_PIXDATA_G1_MASK     (0x7 << 12)  // 12~14
#define VOUT_PMM_PIXDATA_G2_MASK     (0x7 << 15)  // 15~17
#define VOUT_PMM_PIXDATA_G3_MASK     (0x7 << 18)  // 18~20
#define VOUT_PMM_PIXDATA_G4_MASK     (0x7 << 21)  // 21~23
#define VOUT_PMM_PIXDATA_G5_MASK     (0x7 << 24)  // 24~26
#define VOUT_PMM_PIXDATA_G6_MASK     (0x7 << 27)  // 27~29
#define VOUT_PMM_PIXDATA_G7_MASK     (0x7 << 0)   // 0~2
#define VOUT_PMM_PIXDATA_G8_MASK     (0x7 << 3)   // 3~5
#define VOUT_PMM_PIXDATA_G9_MASK     (0x7 << 6)   // 6~8
#define VOUT_PMM_PIXDATA_G0_VAL      (0x1 << 9)
#define VOUT_PMM_PIXDATA_G1_VAL      (0x1 << 12)
#define VOUT_PMM_PIXDATA_G2_VAL      (0x1 << 15)
#define VOUT_PMM_PIXDATA_G3_VAL      (0x1 << 18)
#define VOUT_PMM_PIXDATA_G4_VAL      (0x1 << 21)
#define VOUT_PMM_PIXDATA_G5_VAL      (0x1 << 24)
#define VOUT_PMM_PIXDATA_G6_VAL      (0x1 << 27)
#define VOUT_PMM_PIXDATA_G7_VAL      (0x1 << 0)
#define VOUT_PMM_PIXDATA_G8_VAL      (0x1 << 3)
#define VOUT_PMM_PIXDATA_G9_VAL      (0x1 << 6)
//VOUT pixel data blue channel
#define VOUT_PMM_PIXDATA_B0_REG      (VOUT_PMM_REG3)
#define VOUT_PMM_PIXDATA_B1_REG      (VOUT_PMM_REG3)
#define VOUT_PMM_PIXDATA_B2_REG      (VOUT_PMM_REG3)
#define VOUT_PMM_PIXDATA_B3_REG      (VOUT_PMM_REG3)
#define VOUT_PMM_PIXDATA_B4_REG      (VOUT_PMM_REG3)
#define VOUT_PMM_PIXDATA_B5_REG      (VOUT_PMM_REG3)
#define VOUT_PMM_PIXDATA_B6_REG      (VOUT_PMM_REG3)
#define VOUT_PMM_PIXDATA_B7_REG      (VOUT_PMM_REG4)
#define VOUT_PMM_PIXDATA_B8_REG      (VOUT_PMM_REG4)
#define VOUT_PMM_PIXDATA_B9_REG      (VOUT_PMM_REG4)
#define VOUT_PMM_PIXDATA_B0_MASK     (0x7 << 9)   // 9~11
#define VOUT_PMM_PIXDATA_B1_MASK     (0x7 << 12)  // 12~14
#define VOUT_PMM_PIXDATA_B2_MASK     (0x7 << 15)  // 15~17
#define VOUT_PMM_PIXDATA_B3_MASK     (0x7 << 18)  // 18~20
#define VOUT_PMM_PIXDATA_B4_MASK     (0x7 << 21)  // 21~23
#define VOUT_PMM_PIXDATA_B5_MASK     (0x7 << 24)  // 24~26
#define VOUT_PMM_PIXDATA_B6_MASK     (0x7 << 27)  // 27~29
#define VOUT_PMM_PIXDATA_B7_MASK     (0x7 << 0)   // 0~2
#define VOUT_PMM_PIXDATA_B8_MASK     (0x7 << 3)   // 3~5
#define VOUT_PMM_PIXDATA_B9_MASK     (0x7 << 6)   // 6~8
#define VOUT_PMM_PIXDATA_B0_VAL      (0x1 << 9)
#define VOUT_PMM_PIXDATA_B1_VAL      (0x1 << 12)
#define VOUT_PMM_PIXDATA_B2_VAL      (0x1 << 15)
#define VOUT_PMM_PIXDATA_B3_VAL      (0x1 << 18)
#define VOUT_PMM_PIXDATA_B4_VAL      (0x1 << 21)
#define VOUT_PMM_PIXDATA_B5_VAL      (0x1 << 24)
#define VOUT_PMM_PIXDATA_B6_VAL      (0x1 << 27)
#define VOUT_PMM_PIXDATA_B7_VAL      (0x1 << 0)
#define VOUT_PMM_PIXDATA_B8_VAL      (0x1 << 3)
#define VOUT_PMM_PIXDATA_B9_VAL      (0x1 << 6)

#define VOUT_PMM_PREADY_REG          (VOUT_PMM_REG4)
#define VOUT_PMM_TEEXT_REG           (VOUT_PMM_REG4)
#define VOUT_PMM_TETRIG_REG          (VOUT_PMM_REG4)
#define VOUT_PMM_PREADY_MASK         (0x7 << 9)      // 9~11
#define VOUT_PMM_TEEXT_MASK          (0x7 << 12)     // 12~14
#define VOUT_PMM_TETRIG_MASK         (0x7 << 15)     // 15~17
#define VOUT_PMM_PREADY_VAL          (0x1 << 9)
#define VOUT_PMM_TEEXT_VAL           (0x1 << 12)
#define VOUT_PMM_TETRIG_VAL          (0x1 << 15)

#endif

static inline u32 vout_conn_read(void *base, u32 reg){
	u32 val;
	val = readl(base + reg);
	return val;
}

#define vout_conn_read_reg(base, reg) ({ \
	vout_conn_read(base, reg);	\
})

static inline void vout_conn_write(void *base, u32 reg, u32 val){
	writel(val, (base + reg));
}

#define vout_conn_write_reg(base, reg, val) ({ \
	vout_conn_write(base, reg, val); \
})

void bst_select_dpu_output_to_vout(struct bst_dpu_connection *conn);

#endif
