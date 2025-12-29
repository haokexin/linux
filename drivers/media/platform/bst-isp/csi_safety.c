// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/delay.h>

#ifdef CONFIG_BST_HEALTH_MONITOR
#include <bst/bst_common_api.h>
#endif

#include "csi_safety.h"

#include "csi_controller.h"
#include "csi_hw.h"
#include "csi_rx.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

/* Caller must implement mutual exclusion protection for csi */
static void check_phy_cal(struct csi_device *csi)
{
	int i;
	u32 val;

	i = 0;
	do {
		val = csi_ctrl_get(csi, R_CTRL_PHY_CAL);
		if ((val & 0x1) == 0x1)
			return;
		usleep_range(1000, 2000);
	} while (++i <= csi->host_access_retries);
	dev_err(csi->dev,
		"Caribrated unsuccessfully for %d times, val: 0x%08X\n", i,
		val);
}

#pragma GCC diagnostic pop

/* Caller must implement mutual exclusion protection for csi */
static void set_idi_timeout(struct csi_device *csi, u32 time_ns)
{
	u32 val;

	val = csi_top_get(csi, R_TOP_IDI_TIMEOUT_CFG_TIME);
	if (time_ns / IDI_COUNT_UNIT > val)
		csi_top_set(csi, R_TOP_IDI_TIMEOUT_CFG_TIME,
			    time_ns / IDI_COUNT_UNIT);
}

/* Caller must implement mutual exclusion protection for csi */
static void enable_idi_monitor(struct csi_device *csi, int vc)
{
	u32 val;

	csi_top_update(csi, R_TOP_INT_FUNC_CLR, 0x10000000, 0x10000000);
	csi_top_update(csi, R_TOP_INT_FUNC_CLR, 0x00000000, 0x10000000);
	csi_top_update(csi, R_TOP_INT_DIAG_CLR, 0x00004000, 0x4000);
	csi_top_update(csi, R_TOP_INT_DIAG_CLR, 0x00000000, 0x4000);
	csi_top_set(csi, R_TOP_IDI_TIMEOUT_CLR, 0xFFFFFFFF);
	csi_top_set(csi, R_TOP_IDI_TIMEOUT_CLR, 0);

	val = csi_top_get(csi, R_TOP_IDI_MONITOR_EN);
	dev_dbg(csi->dev, "IDIMONEN: vc: %d, current cfg: 0x%08X\n", vc, val);
	csi_top_update(csi, R_TOP_IDI_MONITOR_EN, BIT(vc), BIT(vc));
}

/* Caller must implement mutual exclusion protection for csi */
static void disable_idi_monitor(struct csi_device *csi, int vc)
{
	u32 val;

	val = csi_top_get(csi, R_TOP_IDI_MONITOR_EN);
	dev_dbg(csi->dev, "IDIMONDIS: vc: %d, current cfg: 0x%08X\n", vc, val);
	csi_top_update(csi, R_TOP_IDI_MONITOR_EN, 0, BIT(vc));
}

#ifdef CONFIG_BST_HEALTH_MONITOR
void csi_safety_get_psm(struct csi_device *csi)
{
	u8 block_id;
	u32 psm[PSM_BLOCK_CFG_SIZE];
	int rv;

	memset(psm, 0, sizeof(psm));
	rv = get_psmid_from_safety_lib((PSM_BLOCK_ID_CSI_BASE + csi->id),
				       &block_id, psm);
	if (rv) {
		dev_err(csi->dev, "Failed to get PSM\n");

		return;
	}

	/* CSI RX only has limited safety mechanism */
	csi->psm.val = psm[0];
	dev_dbg(csi->dev, "PSM: 0x%08X, 0x%08X, 0x%08X, 0x%08X\n", psm[0],
		psm[1], psm[2], psm[3]);
}
#endif

void csi_safety_stream(struct csi_device *csi, int enable)
{
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	int rv;
#endif
	int vc;
	int en;
	struct device *dev;
	struct camera_dev *cam_dev;
	u32 timeout;
	int i;

	dev = csi->dev;
	vc = STREAM_DEC_VC(enable);
	en = STREAM_DEC_EN(enable);

	/* Enable safety on first stream on */
	if (en && csi->used_vcs == 1) {
		if (csi->func_irq_enable) {
			csi_top_func_irq_clear(csi);
			csi_ctrl_func_irq_clear(csi);
			csi_top_func_irq_enable(csi);
			csi_ctrl_func_irq_enable(csi);
		}
		/* Enable diag IRQ always, since it's handled by safety subsystem */
		csi_top_diag_irq_enable(csi);
		csi_ctrl_diag_irq_enable(csi);
	} else if (!en && csi->used_vcs == 0) {
		if (csi->func_irq_enable) {
			csi_top_func_irq_disable(csi);
			csi_ctrl_func_irq_disable(csi);
		}
		csi_top_diag_irq_disable(csi);
		csi_ctrl_diag_irq_disable(csi);
	}

	if (!csi->psm.link_timeout_monitor)
		return;

	if (!en) {
		dev_dbg(dev, "CSS: Disable IDI monitor for VC %u\n", vc);
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
		rv = get_sem_lock_with_timeout(csi->hwlock,
					       CSI_HW_LOCK_TIMEOUT);
		if (rv != 0 && rv != csi->sem_master) {
			dev_warn(
				dev,
				"CSS IDIDIS: Sem %u of bank %u is hold by master %u\n",
				csi->sem_id, csi->sem_bank, rv);
			return;
		}
#endif
		disable_idi_monitor(csi, vc);
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
		release_sem_lock(csi->hwlock);
#endif
		return;
	}

	cam_dev = NULL;
	for (i = 0; i < ARRAY_SIZE(csi->channels); ++i) {
		dev_dbg(dev, "CSS: Channel: %02d, vc: %02d, cam: 0x%llX\n", i,
			csi->channels[i].vc, (u64)csi->channels[i].cam_dev);
		if (csi->channels[i].vc == vc) {
			cam_dev = csi->channels[i].cam_dev;
			break;
		}
	}

	if (cam_dev != NULL && cam_dev->sensor_fps > 0) {
		timeout = (NS_PER_SECOND / cam_dev->sensor_fps +
			   IDI_TIMEOUT_EXTRA);
		usleep_range(timeout / NS_PER_USEC, timeout / NS_PER_USEC + 1);
	} else {
		timeout = IDI_TIMEOUT_DEFAULT;
		dev_warn(dev, "CSS: Invalid fps for vc: %d\n", vc);
	}
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	rv = get_sem_lock_with_timeout(csi->hwlock, CSI_HW_LOCK_TIMEOUT);
	if (rv != 0 && rv != csi->sem_master) {
		dev_warn(dev,
			 "CSS IDIEN: Sem %u of bank %u is hold by master %u\n",
			 csi->sem_id, csi->sem_bank, rv);
		return;
	}
#endif
	set_idi_timeout(csi, timeout);
	enable_idi_monitor(csi, vc);

#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	release_sem_lock(csi->hwlock);
#endif
}
