/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_CSI_SAFETY_H__
#define __BST_CSI_SAFETY_H__

#include <linux/types.h>

struct csi_device;

/* Retries to write and read back register to compare */
#ifdef CONFIG_BST_HEALTH_MONITOR
#define REG_ACCESS_RETRIES     (3)
#define PSM_BLOCK_ID_CSI_BASE  (0x3B)
#define PSM_BLOCK_CFG_SIZE     (4)
#define PSM_ID_PHY_REG_ACCESS  (0x03)
#define PSM_ID_HOST_REG_ACCESS (0x04)
#define MKDTC(csi_id, psm_id)                                              \
	((0xF6 << 16) | (((csi_id + PSM_BLOCK_ID_CSI_BASE) & 0xFF) << 8) | \
	 ((psm_id) & 0xFF))
#endif

union psm {
	u32 val;

	struct {
		u32 info_crc		   : 1; /* Splitted to sub items */
		u32 link_timeout_monitor   : 1;
		u32 frame_line_counter	   : 1;
		u32 phy_access_confirm	   : 1;
		u32 host_access_confirm	   : 1;
		u32 interrupt_test	   : 1; /* Implement by Safety SS */
		u32 interrupt_monitor	   : 1; /* Implement by Safety SS */
		u32 internal_reg_parity	   : 1;
		u32 data_path_crc	   : 1;
		u32 dphy_header_ecc	   : 1;
		u32 cphy_header_crc	   : 1;
		u32 config_reg_parity	   : 1;
		u32 module_reduancy	   : 1;
		u32 triple_module_reduancy : 1; /* Unsupported for IDI */
		u32 mem_data_crc	   : 1; /* Unsupported for IDI */
		u32 idi_bus_parity	   : 1; /* Unsupported for IDI SRC */
		u32 rsv			   : 16;
	};
};

#ifdef CONFIG_BST_HEALTH_MONITOR
void csi_safety_get_psm(struct csi_device *csi);
#endif
void csi_safety_stream(struct csi_device *csi, int enable);

#endif /* __BST_CSI_SAFETY_H__ */
