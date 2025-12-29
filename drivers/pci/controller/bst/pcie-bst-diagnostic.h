/* SPDX-License-Identifier: GPL-2.0 */
/*
 * BST PCIe diagnostic
 *
 * Copyright (C) 2024 Black Sesame Technologies, Inc.
 *
 * Author: Gordon.Ge <gordon.geg@bst.ai>
 */
#ifdef CONFIG_PCIE_BST_DIAGNOSTIC
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/delay.h>

#define PHY_ADDR_SPACE_LIMIT_UPPER       0x8000
#define PCIE_CONFIGURE_REG_SPACE_SIZE    0x100 // pcie configure header space + pcie standard capability structure
#define DEFAULT_MON_PERIODIC             100  // ms

#define PCIE_DIAG_SHUTDOWN_STATE              0x0
#define PCIE_DIAG_RUNNING_STATE               0x1
#define PCIE_DIAG_SUSPEND_STATE               0x2


/* BLOCK */
#define PSM_CPU_PCIE_BLOCK                0x7E
#define PSM_CPU_PCIE_BLOCK_OFF            0x8
#define PSM_CPU_PCIE_MASK                 0xF60000
/* PSM_ID */
#define PSM_ID_RASDES_COUNTER             0x13
#define PSM_ID_CONFIGURE_REG_READBACK     0x14
#define PSM_ID_LTSSM                      0x16
#define PSM_ID_AXI_SUPERVISOR             0x17
#define PSM_ID_DBI_REG_RW_ACCESS          0x1A
#define PSM_ID_RASDP_ERR_MODE_MONITOR     0x20
#define PSM_ID_PHY_REG_CHECK              0x26
#define PSM_ID_EXT_SRAM_ACCESS            0x27
#define PSM_ID_TX2RX_LOOPBACK             0x28

#define SLV_RASDP_ERR_MODE_OFF            25
#define MSTR_RASDP_ERR_MODE_OFF           24
// #define DBI_RASDP_ERR_MODE_OFF            24
#define PSM_ID_RASDES_COUNTER_DTC                 PSM_CPU_PCIE_MASK | (PSM_CPU_PCIE_BLOCK << PSM_CPU_PCIE_BLOCK_OFF) | PSM_ID_RASDES_COUNTER
#define PSM_ID_CONFIGURE_REG_READBACK_DTC         PSM_CPU_PCIE_MASK | (PSM_CPU_PCIE_BLOCK << PSM_CPU_PCIE_BLOCK_OFF) | PSM_ID_CONFIGURE_REG_READBACK
#define PSM_ID_LTSSM_DTC                          PSM_CPU_PCIE_MASK | (PSM_CPU_PCIE_BLOCK << PSM_CPU_PCIE_BLOCK_OFF) | PSM_ID_LTSSM
#define PSM_ID_AXI_SUPERVISOR_DTC                 PSM_CPU_PCIE_MASK | (PSM_CPU_PCIE_BLOCK << PSM_CPU_PCIE_BLOCK_OFF) | PSM_ID_AXI_SUPERVISOR
#define PSM_ID_DBI_REG_RW_ACCESS_DTC              PSM_CPU_PCIE_MASK | (PSM_CPU_PCIE_BLOCK << PSM_CPU_PCIE_BLOCK_OFF) | PSM_ID_DBI_REG_RW_ACCESS
#define PSM_ID_RASDP_ERR_DTC                      PSM_CPU_PCIE_MASK | (PSM_CPU_PCIE_BLOCK << PSM_CPU_PCIE_BLOCK_OFF) | PSM_ID_RASDP_ERR_MODE_MONITOR

#define PSM_ID_PHY_REG_CHECK_DTC              PSM_CPU_PCIE_MASK | (PSM_CPU_PCIE_BLOCK << PSM_CPU_PCIE_BLOCK_OFF) | PSM_ID_PHY_REG_CHECK
#define PSM_ID_EXT_SRAM_ACCESS_DTC              PSM_CPU_PCIE_MASK | (PSM_CPU_PCIE_BLOCK << PSM_CPU_PCIE_BLOCK_OFF) | PSM_ID_EXT_SRAM_ACCESS
#define PSM_ID_TX2RX_LOOPBACK_DTC              PSM_CPU_PCIE_MASK | (PSM_CPU_PCIE_BLOCK << PSM_CPU_PCIE_BLOCK_OFF) | PSM_ID_TX2RX_LOOPBACK

ssize_t diagnostic_show(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t diagnostic_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
int pcie_bst_diag_init(void* data);
#endif
