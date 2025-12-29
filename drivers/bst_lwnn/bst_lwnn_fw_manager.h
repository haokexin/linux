// SPDX-License-Identifier: GPL-2.0+
/*
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

/*!
 * bst_lwnn: Linux device driver for Black Sesame Technologies Computer Vision IP
 * @author: AI Tools Team, BST Ltd.
 *
 * @file    bst_lwnn_fw_manager.h
 * @brief   This file is the header file of the firmware manager part of the
 *          bst_lwnn driver. It contains related constants and structure
 *          definitions and function declarations.
 */

#ifndef BST_LWNN_FW_MANAGER_H
#define BST_LWNN_FW_MANAGER_H

#define BST_LWNN_HANDSHAKE_TIMEOUT 100
#define BST_LWNN_HANDSHAKE_SLEEP_INTERVAL 10
#define BST_LWNN_HANDSHAKE_RETRY_NUM \
	(BST_LWNN_HANDSHAKE_TIMEOUT / BST_LWNN_HANDSHAKE_SLEEP_INTERVAL)

#define BST_LWNN_REG_WIDTH 4

#define BST_LWNN_FIRMWARE_DUMP_SIZE 64
#define BST_LWNN_FIRMWARE_DUMP_LINE_SIZE 8

#define LB_CV_REG_R_CV_SYS_CTRL_OFFSET 0x0
#define BST_LWNN_SOFT_RESET_BIT 28
#define BST_LWNN_SOFT_DEBUG_RESET_BIT 8
#define BST_LWNN_DSP0_CLK_EN 24
#define BST_LWNN_RUNSTALL_BIT 20
#define LB_CV_REG_R_CV_SYS_CTRL_DEFAULT 0x0f000000

#define LB_CV_REG_R_CV_DSP_ALT_RESET_VEC_OFFSET 0x4

#define LB_CV_REG_R_CV_DSP_CLUSTER_INTR_EN_REG 0x28
#define LB_CV_REG_R_CV_DSP_CV_PARITY_CTRL_REG0 0x50

struct bst_lwnn_res_bypass {
	phys_addr_t paddr;
	dma_addr_t iova;
	resource_size_t size;
};

struct bst_lwnn_dsp_fw_ctl {
	// flags used for cleanup
	uint8_t init;
	uint8_t boot;
	// firmware fileanme
	char *name;
	char *name_msgbox;
	// memory reserved for firmware codes
	void __iomem *fwmem_base;
	resource_size_t fwmem_size;
	phys_addr_t fwmem_phys_addr;
	dma_addr_t fwmem_iova;
	dsp_ptr rt_init_addr;
	// memory assigned to firmware
	struct bst_lwnn_memblock *assigned_mem;
	// ipc source ARM core
	uint32_t ipc_src_core;
};

struct bst_lwnn_fw_manager {
	// the cv registers
	void __iomem *lb_cv_reg_base;
	dsp_ptr ipc_register_addr;
	struct bst_lwnn_dsp_fw_ctl dsps[BST_LWNN_MAX_DSP_NUM];
	struct bst_lwnn_res_bypass res_bypass[2];
	struct _bst_lwnn_ver_info ver_info;
};

int bst_lwnn_fw_manager_init(struct bst_lwnn *pbst_lwnn);
int bst_lwnn_fw_rt_setup(struct bst_lwnn *pbst_lwnn);
void bst_lwnn_fw_manager_cleanup(struct bst_lwnn *pbst_lwnn);
void bst_lwnn_fw_rt_cleanup(struct bst_lwnn *pbst_lwnn);
void bst_lwnn_fw_manager_exit(struct bst_lwnn *pbst_lwnn);
void bst_lwnn_fw_rt_exit(struct bst_lwnn *pbst_lwnn);

#endif
