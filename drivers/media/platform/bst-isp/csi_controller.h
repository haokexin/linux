/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_CSI_CONTROLLER_H__
#define __BST_CSI_CONTROLLER_H__

#include <linux/types.h>

#include "csi_hw.h"

struct csi_device;

#define VC_EN_MASK			 (BIT(MAX_VC_PER_CSI) - 1)
/* Controller register definitions */
#define R_CTRL_VERSION			 (0x000)
#define R_CTRL_N_LANES			 (0x004)
#define R_CTRL_CSI2_RESETN		 (0x008)
#define R_CTRL_ST_MAIN			 (0x00C)
#define R_CTRL_PHY_CFG			 (0x018)
#define R_CTRL_PHY_MODE			 (0x01C)
#define R_CTRL_ST_AP_MAIN		 (0x02C)
#define R_CTRL_PHY_SHUTDOWNZ		 (0x040)
#define R_CTRL_DPHY_RSTZ		 (0x044)
#define R_CTRL_PHY_RX			 (0x048)
#define R_CTRL_PHY_CAL			 (0x0CC)
#define R_CTRL_ST_PHY_FATAL		 (0x0E0)
#define R_CTRL_MSK_PHY_FATAL		 (0x0E4)
#define R_CTRL_ST_PKT_FATAL		 (0x0F0)
#define R_CTRL_MSK_PKT_FATAL		 (0x0F4)
#define R_CTRL_ST_PHY			 (0x110)
#define R_CTRL_MSK_PHY			 (0x114)
#define R_CTRL_ST_LINE			 (0x130)
#define R_CTRL_MSK_LINE			 (0x134)
#define R_CTRL_ST_AP_GENERIC		 (0x180)
#define R_CTRL_MSK_AP_GENERIC		 (0x184)
#define R_CTRL_ST_LOGGER_ERR		 (0x1D0) /* HOST_AP == 1 */
#define R_CTRL_MSK_LOGGER_ERR		 (0x1D4)
#define R_CTRL_ST_BNDRY_FRAME_FATAL	 (0x280)
#define R_CTRL_MSK_BNDRY_FRAME_FATAL	 (0x284)
#define R_CTRL_ST_SEQ_FRAME_FATAL	 (0x290)
#define R_CTRL_MSK_SEQ_FRAME_FATAL	 (0x294)
#define R_CTRL_ST_CRC_FRAME_FATAL	 (0x2A0)
#define R_CTRL_MSK_CRC_FRAME_FATAL	 (0x2A4)
#define R_CTRL_ST_PLD_CRC_FATAL		 (0x2B0)
#define R_CTRL_MSK_PLD_CRC_FATAL	 (0x2B4)
#define R_CTRL_ST_DATA_ID		 (0x2C0)
#define R_CTRL_MSK_DATA_ID		 (0x2C4)
#define R_CTRL_ST_ECC_CORRECTED		 (0x2D0)
#define R_CTRL_MSK_ECC_CORRECTED	 (0x2D4)
#define R_CTRL_NSYNC			 (0x340)
#define R_CTRL_ST_FAP_PHY_FATAL		 (0x360)
#define R_CTRL_MSK_FAP_PHY_FATAL	 (0x364)
#define R_CTRL_ST_FAP_PKT_FATAL		 (0x370)
#define R_CTRL_MSK_FAP_PKT_FATAL	 (0x374)
#define R_CTRL_ST_FAP_PHY		 (0x390)
#define R_CTRL_MSK_FAP_PHY		 (0x394)
#define R_CTRL_ST_FAP_LINE		 (0x3B0)
#define R_CTRL_MSK_FAP_LINE		 (0x3B4)
#define R_CTRL_ST_FAP_BNDRY_FRAME_FATAL	 (0x420)
#define R_CTRL_MSK_FAP_BNDRY_FRAME_FATAL (0x424)
#define R_CTRL_ST_FAP_SEQ_FRAME_FATAL	 (0x430)
#define R_CTRL_MSK_FAP_SEQ_FRAME_FATAL	 (0x434)
#define R_CTRL_ST_FAP_CRC_FRAME_FATAL	 (0x440)
#define R_CTRL_MSK_FAP_CRC_FRAME_FATAL	 (0x444)
#define R_CTRL_ST_FAP_PLD_CRC_FATAL	 (0x450)
#define R_CTRL_MSK_FAP_PLD_CRC_FATAL	 (0x454)
#define R_CTRL_ST_FAP_DATA_ID		 (0x460)
#define R_CTRL_MSK_FAP_DATA_ID		 (0x464)
#define R_CTRL_ST_FAP_ECC_CORRECTED	 (0x470)
#define R_CTRL_MSK_FAP_ECC_CORRECTED	 (0x474)

#define CSI_CTRL_ERR_ON(csi, reg)                                             \
	({                                                                    \
		u32 val = csi_ctrl_get((csi), (reg));                         \
		if (val)                                                      \
			dev_err((csi)->dev, "Err: 0x%08X -> 0x%08X\n", (reg), \
				val);                                         \
	})

void csi_ctrl_func_irq_clear(struct csi_device *csi);
void csi_ctrl_func_irq_enable(struct csi_device *csi);
void csi_ctrl_func_irq_disable(struct csi_device *csi);
void csi_ctrl_diag_irq_clear(struct csi_device *csi);
void csi_ctrl_diag_irq_enable(struct csi_device *csi);
void csi_ctrl_diag_irq_disable(struct csi_device *csi);
u32 csi_ctrl_get(struct csi_device *csi, u32 reg);
int csi_ctrl_set(struct csi_device *csi, u32 reg, u32 val);
int csi_ctrl_update(struct csi_device *csi, u32 reg, u32 val, u32 mask);

#endif /* __BST_CSI_CONTROLLER_H__ */
