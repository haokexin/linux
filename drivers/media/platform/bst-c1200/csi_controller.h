/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_CSI_CONTROLLER_H__
#define __BST_CSI_CONTROLLER_H__

#define REG_VERSION	    0x0000
#define REG_N_LANES	    0x0004
#define REG_CSI_RESETN	    0x0008
#define REG_INT_ST_MAIN	    0x000C
#define REG_INT_ST_AP_MAIN  0x002C
#define REG_PHY_SHUTDOWNZ   0x0040
#define REG_DPHY_RSTZ	    0x0044
#define REG_PHY_RX	    0x0048
#define REG_STOPSTATE	    0x004C
#define REG_PHY_TEST_CTRL0  0x0050
#define REG_PHY_TEST_CTRL1  0x0054
#define REG_PHY2_TEST_CTRL0 0x0058
#define REG_PHY2_TEST_CTRL1 0x005C
#define REG_PHY_CAL	    0x00CC

#define REG_INT_ST_PHY_FATAL	      0x00E0
#define REG_INT_MSK_PHY_FATAL	      0x00E4
#define REG_INT_ST_PKT_FATAL	      0x00F0
#define REG_INT_MSK_PKT_FATAL	      0x00F4
#define REG_INT_ST_PHY		      0x0110
#define REG_INT_MSK_PHY		      0x0114
#define REG_INT_ST_LINE		      0x0130
#define REG_INT_MSK_LINE	      0x0134
#define REG_INT_ST_AP_GENERIC	      0x0180
#define REG_INT_MSK_AP_GENERIC	      0x0184
#define REG_INT_ST_BNDRY_FRAME_FATAL  0x0280
#define REG_INT_MSK_BNDRY_FRAME_FATAL 0x0284
#define REG_INT_ST_SEQ_FRAME_FATAL    0x0290
#define REG_INT_MSK_SEQ_FRAME_FATAL   0x0294
#define REG_INT_ST_CRC_FRAME_FATAL    0x02A0
#define REG_INT_MSK_CRC_FRAME_FATAL   0x02A4
#define REG_INT_ST_PLD_CRC_FATAL      0x02B0
#define REG_INT_MSK_PLD_CRC_FATAL     0x02B4
#define REG_INT_ST_DATA_ID	      0x02C0
#define REG_INT_MSK_DATA_ID	      0x02C4
#define REG_INT_ST_ECC_CORRECT	      0x02D0
#define REG_INT_MSK_ECC_CORRECT	      0x02D4

#define REG_INT_ST_FAP_PHY_FATAL	  0x0360
#define REG_INT_MSK_FAP_PHY_FATAL	  0x0364
#define REG_INT_ST_FAP_PKT_FATAL	  0x0370
#define REG_INT_MSK_FAP_PKT_FATAL	  0x0374
#define REG_INT_ST_FAP_PHY		  0x0390
#define REG_INT_MSK_FAP_PHY		  0x0394
#define REG_INT_ST_FAP_LINE		  0x03B0
#define REG_INT_MSK_FAP_LINE		  0x03B4
#define REG_INT_ST_FAP_BNDRY_FRAME_FATAL  0x0420
#define REG_INT_MSK_FAP_BNDRY_FRAME_FATAL 0x0424
#define REG_INT_ST_FAP_SEQ_FRAME_FATAL	  0x0430
#define REG_INT_MSK_FAP_SEQ_FRAME_FATAL	  0x0434
#define REG_INT_ST_FAP_CRC_FRAME_FATAL	  0x0440
#define REG_INT_MSK_FAP_CRC_FRAME_FATAL	  0x0444
#define REG_INT_ST_FAP_PLD_CRC_FATAL	  0x0450
#define REG_INT_MSK_FAP_PLD_CRC_FATAL	  0x0454
#define REG_INT_ST_FAP_DATA_ID		  0x0460
#define REG_INT_MSK_FAP_DATA_ID		  0x0464
#define REG_INT_ST_FAP_ECC_CORRECT	  0x0470
#define REG_INT_MSK_FAP_ECC_CORRECT	  0x0474

extern void controller_enable_function_irq(struct bst_csi_device *csi_dev);
extern void controller_enable_fmeda_irq(struct bst_csi_device *csi_dev);

#endif // __BST_CSI_CONTROLLER_H__
