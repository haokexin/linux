/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

/*
 * bst_cv: Linux device driver for Black Sesame Technologies Computer Vision IP
 * author: AI Tools Team, BST Ltd.
 *
 * @file    bst_cv_fw_manager.h
 * @brief   This file is the header file of the firmware manager part of the
 *          bst_cv driver. It contains related constants and structure
 *          definitions and function declarations.
 */

#ifndef BST_CV_FW_MANAGER_H
#define BST_CV_FW_MANAGER_H

#define BST_CV_HANDSHAKE_BUF_SIZE                   0x400
#define BST_CV_HANDSHAKE_TIMEOUT                    100
#define BST_CV_HANDSHAKE_SLEEP_INTERVAL             10
#define BST_CV_HANDSHAKE_RETRY_NUM                  (BST_CV_HANDSHAKE_TIMEOUT / BST_CV_HANDSHAKE_SLEEP_INTERVAL)

#define BST_CV_REG_WIDTH                            4

#define BST_CV_FIRMWARE_DUMP_SIZE                   64
#define BST_CV_FIRMWARE_DUMP_LINE_SIZE              8

#define LB_CV_REG_R_CV_SYS_CTRL_OFFSET              0x0
#define BST_CV_SOFT_RESET_BIT                       28
#define BST_CV_CLK_EN_BIT                           24
#define BST_CV_RUNSTALL_BIT                         20
#define LB_CV_REG_R_CV_SYS_CTRL_DEFAULT             0x0f000000

#define LB_CV_REG_R_CV_DSP_ALT_RESET_VEC_OFFSET     0x4

#define LB_CV_REG_R_CV_DSP_CLUSTER_INTR_EN_REG 		0x28
#define LB_CV_REG_R_CV_DSP_CV_PARITY_CTRL_REG0 		0x50

#define CACHE_LINE_SIZE                             64

#define BST_CV_SUPPORT_FIRMWARE_NUM                 (2)
#define BST_CV_FIRMWARE_NAME_MAX_SIZE               (512)
#define BST_CV_DSP_RAM_BASE                         (0x5c000000)
#define BST_CV_DSP_RAM_BASE_OFFSET                  (0x100000)
#define BST_CV_DSP_RAM_BASE_INTERVAL                (0x100000)

struct bst_cv_rt_setup_info {
	dsp_ptr assigned_mem;
	uint32_t assigned_mem_size;
};

struct bst_cv_dsp_fw_ctl {
	uint8_t init;
	uint8_t boot;
	char *name;
	//memory reserved for firmware codes
	void __iomem *fwmem_base;
	resource_size_t fwmem_size;
	phys_addr_t fwmem_phys_addr;
	dsp_ptr rt_init_addr;
	//handshake base
	void *sync_virt_base;
	//ipc source ARM core
	uint32_t ipc_src_core;
};

struct bst_cv_fw_manager {
	//the cv registers
	void __iomem *lb_cv_reg_base;
	dsp_ptr ipc_register_addr;
	//memory assigned to firmware handshake
	struct bst_cv_memblock *assigned_mem;
	struct bst_cv_dsp_fw_ctl dsps[BST_CV_DSP_NUM];
};

int bst_cv_fw_manager_init(struct bst_cv *pbst_cv);
int bst_cv_fw_rt_setup(struct bst_cv *pbst_cv);
void bst_cv_fw_manager_cleanup(struct bst_cv *pbst_cv);
void bst_cv_fw_rt_cleanup(struct bst_cv *pbst_cv);
void bst_cv_fw_manager_exit(struct bst_cv *pbst_cv);
void bst_cv_fw_rt_exit(struct bst_cv *pbst_cv);

#endif
