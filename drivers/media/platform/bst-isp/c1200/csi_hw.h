/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2025 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_CSI_HW_H__
#define __BST_CSI_HW_H__

#include <linux/types.h>

struct csi_device;

/* Hardware specification */
#define MAX_VC_PER_CSI		   (4)
#define CSI_MAX_DPHY_LANES	   (4)
#define CSI_MAX_DPHY_SPEED	   (4500)
#define CSI_MAX_CPHY_LANES	   (3)
#define CSI_MAX_CPHY_SPEED	   (3500)
#define CSI_MAX_EQ		   (7)
#define CSI_MAX_MODULES		   (3)

/* Top register definitions */
#define R_TOP_PHY_CLK		   (0x00)
#define R_TOP_PHY_LANE0		   (0x04)
#define R_TOP_PHY_LANE1		   (0x08)
#define R_TOP_PHY_LANE2		   (0x0C)
#define R_TOP_PHY_LANE3		   (0x10)
#define R_TOP_CTRL		   (0x14)
#define R_TOP_IDI_MONITOR_EN	   (0x20)
#define R_TOP_IDI_TIMEOUT_CLR	   (0x24)
#define R_TOP_IDI_TIMEOUT_CFG_TIME (0x28)
#define R_TOP_IDI_TIMEOUT	   (0x2C)
#define R_TOP_SYS_RD		   (0x30)
#define R_TOP_IDI_OUT2		   (0x3C)
#define R_TOP_IPTEST0		   (0x40)
#define R_TOP_INT_CONFIG_CLR	   (0x90)
#define R_TOP_INT_FUNC_MUX_SEL	   (0x94)
#define R_TOP_INT_FUNC_EN	   (0x98)
#define R_TOP_INT_FUNC_CLR	   (0xA0)
#define R_TOP_INT_ERR_MUX_SEL	   (0xA4)
#define R_TOP_INT_ERR_EN	   (0xA8)
#define R_TOP_INT_ERR_CLR	   (0xB0)
#define R_TOP_INT_DIAG_MUX_SEL	   (0xB4)
#define R_TOP_INT_DIAG_EN	   (0xB8)
#define R_TOP_INT_DIAG_CLR	   (0xC0)
#define R_TOP_INT_FUNC_STATE	   (0xC4)
#define R_TOP_INT_ERR_STATE	   (0xC8)
#define R_TOP_INT_DIAG_STATE	   (0xCC)

/* Top register settings */
#define TOP_FUNC_EN_BITS	   (BIT(28) | BIT(0))
#define TOP_DIAG_EN_BITS	   (0x7FFF)

/* Runtime parameters, etc */
#define CSI_RATELIMIT_INTERVAL	  (CSI_MAX_MODULES * HZ)
#define CSI_RATELIMIT_BURST	  (CSI_MAX_MODULES)
#define NS_PER_SECOND		  (1000 * 1000 * 1000)
#define NS_PER_USEC		  (1000)
#define DEFAULT_EQ		  (4)
#define DEFAULT_RECOVER_THRESHOLD (40)
#define DEFAULT_RECOVER_WINDOW	  (2 * (NS_PER_SECOND)) /* by ns */
#define IDI_TIMEOUT_DEFAULT	  ((u32)-1)
/* Additional timeout by ns beyond frame rate */
#define IDI_TIMEOUT_EXTRA	  (5 * 1000 * 1000)
#define IDI_COUNT_UNIT		  (10) /* Hardware use 10ns as unit */

void csi_top_func_irq_clear(struct csi_device *csi);
void csi_top_func_irq_enable(struct csi_device *csi);
void csi_top_func_irq_disable(struct csi_device *csi);
void csi_top_diag_irq_clear(struct csi_device *csi);
void csi_top_diag_irq_enable(struct csi_device *csi);
void csi_top_diag_irq_disable(struct csi_device *csi);
u32 csi_top_get(struct csi_device *csi, u32 reg);
int csi_top_set(struct csi_device *csi, u32 reg, u32 val);
int csi_top_update(struct csi_device *csi, u32 reg, u32 val, u32 mask);
bool csi_hw_has_inited(struct csi_device *csi);
int csi_hw_init(struct csi_device *csi);
void csi_hw_exit(struct csi_device *csi);
void csi_hw_reset(struct csi_device *csi);

#endif /* __BST_CSI_HW_H__ */
