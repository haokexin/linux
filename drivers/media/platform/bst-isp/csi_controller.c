// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/delay.h>
#include <linux/io.h>

#ifdef CONFIG_BST_HEALTH_MONITOR
#include <bst/bst_common_api.h>
#endif

#include "csi_controller.h"

#include "csi_rx.h"
#include "csi_safety.h"

void csi_ctrl_func_irq_clear(struct csi_device *csi)
{
	CSI_CTRL_ERR_ON(csi, R_CTRL_ST_MAIN);
	CSI_CTRL_ERR_ON(csi, R_CTRL_ST_PHY_FATAL);
	CSI_CTRL_ERR_ON(csi, R_CTRL_ST_PKT_FATAL);
	CSI_CTRL_ERR_ON(csi, R_CTRL_ST_PHY);
	CSI_CTRL_ERR_ON(csi, R_CTRL_ST_LINE);
	CSI_CTRL_ERR_ON(csi, R_CTRL_ST_BNDRY_FRAME_FATAL);
	CSI_CTRL_ERR_ON(csi, R_CTRL_ST_SEQ_FRAME_FATAL);
	CSI_CTRL_ERR_ON(csi, R_CTRL_ST_CRC_FRAME_FATAL);
	CSI_CTRL_ERR_ON(csi, R_CTRL_ST_PLD_CRC_FATAL);
	CSI_CTRL_ERR_ON(csi, R_CTRL_ST_DATA_ID);
	CSI_CTRL_ERR_ON(csi, R_CTRL_ST_ECC_CORRECTED);
}

void csi_ctrl_func_irq_enable(struct csi_device *csi)
{
	csi_ctrl_update(csi, R_CTRL_MSK_PHY_FATAL, 0x10F, 0x1FF);
	csi_ctrl_update(csi, R_CTRL_MSK_PKT_FATAL, 0x3, 0x3);
	csi_ctrl_update(csi, R_CTRL_MSK_PHY, 0x0F000F, 0xFF00FF);
	csi_ctrl_update(csi, R_CTRL_MSK_LINE, 0xFF00FF, 0xFF00FF);
	csi_ctrl_set(csi, R_CTRL_MSK_BNDRY_FRAME_FATAL, VC_EN_MASK);
	/* Disable for compatibility with Max96726 if needed */
	// csi_ctrl_set(csi, R_CTRL_MSK_SEQ_FRAME_FATAL, VC_EN_MASK);
	csi_ctrl_set(csi, R_CTRL_MSK_CRC_FRAME_FATAL, VC_EN_MASK);
	csi_ctrl_set(csi, R_CTRL_MSK_PLD_CRC_FATAL, VC_EN_MASK);
	csi_ctrl_set(csi, R_CTRL_MSK_DATA_ID, VC_EN_MASK);
	csi_ctrl_set(csi, R_CTRL_MSK_ECC_CORRECTED, VC_EN_MASK);
}

void csi_ctrl_func_irq_disable(struct csi_device *csi)
{
	csi_ctrl_update(csi, R_CTRL_MSK_PHY_FATAL, 0, 0x1FF);
	csi_ctrl_update(csi, R_CTRL_MSK_PKT_FATAL, 0, 0x3);
	csi_ctrl_update(csi, R_CTRL_MSK_PHY, 0, 0xFF00FF);
	csi_ctrl_update(csi, R_CTRL_MSK_LINE, 0, 0xFF00FF);
	csi_ctrl_set(csi, R_CTRL_MSK_BNDRY_FRAME_FATAL, 0);
	csi_ctrl_set(csi, R_CTRL_MSK_SEQ_FRAME_FATAL, 0);
	csi_ctrl_set(csi, R_CTRL_MSK_CRC_FRAME_FATAL, 0);
	csi_ctrl_set(csi, R_CTRL_MSK_PLD_CRC_FATAL, 0);
	csi_ctrl_set(csi, R_CTRL_MSK_DATA_ID, 0);
	csi_ctrl_set(csi, R_CTRL_MSK_ECC_CORRECTED, 0);
}

void csi_ctrl_diag_irq_clear(struct csi_device *csi)
{
	csi_ctrl_get(csi, R_CTRL_ST_AP_MAIN);
	csi_ctrl_get(csi, R_CTRL_ST_AP_GENERIC);
	csi_ctrl_get(csi, R_CTRL_ST_LOGGER_ERR);
	csi_ctrl_get(csi, R_CTRL_ST_FAP_PHY_FATAL);
	csi_ctrl_get(csi, R_CTRL_ST_FAP_PKT_FATAL);
	csi_ctrl_get(csi, R_CTRL_ST_FAP_PHY);
	csi_ctrl_get(csi, R_CTRL_ST_FAP_LINE);
	csi_ctrl_get(csi, R_CTRL_ST_FAP_BNDRY_FRAME_FATAL);
	csi_ctrl_get(csi, R_CTRL_ST_FAP_SEQ_FRAME_FATAL);
	csi_ctrl_get(csi, R_CTRL_ST_FAP_CRC_FRAME_FATAL);
	csi_ctrl_get(csi, R_CTRL_ST_FAP_PLD_CRC_FATAL);
	csi_ctrl_get(csi, R_CTRL_ST_FAP_DATA_ID);
	csi_ctrl_get(csi, R_CTRL_ST_FAP_ECC_CORRECTED);
}

void csi_ctrl_diag_irq_enable(struct csi_device *csi)
{
	u32 val_ap;

	val_ap = 0;
	if (csi->psm.frame_line_counter) {
		csi_ctrl_update(csi, R_CTRL_MSK_FAP_LINE, 0xFF, 0xFF);
		csi_ctrl_set(csi, R_CTRL_MSK_FAP_BNDRY_FRAME_FATAL, VC_EN_MASK);
	}

	if (csi->psm.internal_reg_parity)
		val_ap |= BIT(0) | BIT(4) | BIT(6) | BIT(7) | BIT(8) | BIT(10) |
			  BIT(12) | BIT(24);

	if (csi->psm.data_path_crc)
		csi_ctrl_set(csi, R_CTRL_MSK_FAP_PLD_CRC_FATAL, VC_EN_MASK);

	if (csi->psm.dphy_header_ecc || csi->psm.cphy_header_crc)
		csi_ctrl_update(csi, R_CTRL_MSK_FAP_PKT_FATAL, 0x1, 0x1);

	if (csi->psm.config_reg_parity)
		val_ap |= BIT(2);

	if (csi->psm.module_reduancy)
		val_ap |= BIT(9) | BIT(11) | BIT(13) | BIT(25);

	csi_ctrl_update(csi, R_CTRL_MSK_AP_GENERIC, val_ap, 0x3FFFFFF);
}

void csi_ctrl_diag_irq_disable(struct csi_device *csi)
{
	csi_ctrl_update(csi, R_CTRL_MSK_AP_GENERIC, 0, 0x03FFFFFF);
	csi_ctrl_update(csi, R_CTRL_MSK_FAP_PHY_FATAL, 0, 0x701FF);
	csi_ctrl_update(csi, R_CTRL_MSK_FAP_PKT_FATAL, 0, 0x3);
	csi_ctrl_update(csi, R_CTRL_MSK_FAP_PHY, 0, 0xFF00FF);
	csi_ctrl_update(csi, R_CTRL_MSK_FAP_LINE, 0, 0xFF00FF);
	csi_ctrl_set(csi, R_CTRL_MSK_FAP_BNDRY_FRAME_FATAL, 0);
	csi_ctrl_set(csi, R_CTRL_MSK_FAP_SEQ_FRAME_FATAL, 0);
	csi_ctrl_set(csi, R_CTRL_MSK_FAP_CRC_FRAME_FATAL, 0);
	csi_ctrl_set(csi, R_CTRL_MSK_FAP_PLD_CRC_FATAL, 0);
	csi_ctrl_set(csi, R_CTRL_MSK_FAP_DATA_ID, 0);
	csi_ctrl_set(csi, R_CTRL_MSK_FAP_ECC_CORRECTED, 0);
}

/* Caller must implement mutual exclusion protection for csi */
u32 csi_ctrl_get(struct csi_device *csi, u32 reg)
{
	u32 val;

	val = readl_relaxed(csi->ctrl_base + reg);
	dev_dbg(csi->dev, "CCGET: 0x%08X -> 0x%08X\n", reg, val);

	return val;
}

/* Caller must implement mutual exclusion protection for csi */
int csi_ctrl_set(struct csi_device *csi, u32 reg, u32 val)
{
	u32 rval;
	int i;

	dev_dbg(csi->dev, "CCSET: 0x%08X -> 0x%08X\n", reg, val);
	i = 0;
	do {
		writel_relaxed(val, csi->ctrl_base + reg);
		rval = readl_relaxed(csi->ctrl_base + reg);
		if (rval == val)
			return 0;
	} while (++i <= csi->host_access_retries);

#ifdef CONFIG_BST_HEALTH_MONITOR
	if (csi->psm.host_access_confirm)
		send_dtc_to_safety_svc(MKDTC(csi->id, PSM_ID_HOST_REG_ACCESS));
#endif
	dev_err(csi->dev, "CCSET: 0x%08X -> 0x%08X vs 0x%08X, FAILED\n", reg,
		rval, val);

	return -EIO;
}

/* Caller must implement mutual exclusion protection for csi */
int csi_ctrl_update(struct csi_device *csi, u32 reg, u32 val, u32 mask)
{
	u32 rval;
	u32 wval;
	int i;

	rval = readl_relaxed(csi->ctrl_base + reg);
	wval = (rval & ~mask) | (val & mask);
	dev_dbg(csi->dev,
		"CCUP: 0x%08X -> 0x%08X & 0x%08X, rval: 0x%08X, wval: 0x%08X\n",
		reg, val, mask, rval, wval);
	i = 0;
	do {
		writel_relaxed(wval, csi->ctrl_base + reg);
		rval = readl_relaxed(csi->ctrl_base + reg);
		if (rval == wval)
			return 0;
	} while (++i <= csi->host_access_retries);

#ifdef CONFIG_BST_HEALTH_MONITOR
	if (csi->psm.host_access_confirm)
		send_dtc_to_safety_svc(MKDTC(csi->id, PSM_ID_HOST_REG_ACCESS));
#endif
	dev_err(csi->dev,
		"CCUP: failed: 0x%08X -> 0x%08X & 0x%08X, rval: 0x%08X, wval: 0x%08X\n",
		reg, val, mask, rval, wval);

	return -EIO;
}
