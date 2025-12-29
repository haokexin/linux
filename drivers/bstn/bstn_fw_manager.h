// SPDX-License-Identifier: GPL-2.0+
/*
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

/*
 * BSTN: Linux device driver for Black Sesame Technologies Neural Network IP
 * @author: AI Tools Team, BST Ltd.
 *
 * @file    bstn_fw_manager.h
 * @brief   This file is the header file of the firmware manager part of the
 *          BSTN driver. It contains related constants and structure definitions
 *          and function declarations.
 */

#ifndef BSTN_FIRMWARE_H
#define BSTN_FIRMWARE_H

#include "bstn_hwsem.h"

#define BSTN_FW_BOOT_UP_TIME_MS 100
#define BSTN_FIRMWARE_DUMP_SIZE 64
#define BSTN_FIRMWARE_DUMP_LINE_SIZE 8

// BSTN Hardware Register offset & bit
#define BSTN_SYS_CTRL 0x0
typedef union {
	uint32_t all;
	struct {
		uint32_t rsvd_0 : 10; // bit[0:9]
		uint32_t soft_rst_wdt_n : 1; // bit10
		uint32_t wdt_clk_en : 1; // bit11
		uint32_t safety_apb_parity_chk_dec_en : 1; // bit12
		uint32_t safety_apb_parity_chk_enc_en : 1; // bit13
		uint32_t soft_rst_msgbox_n : 1; // bit14
		uint32_t wdt_speed_up : 1; // bit15
		uint32_t rsvd_1 : 1; // bit16
		uint32_t net_parity_chk_dec_en : 1; // bit17
		uint32_t dsp_parity_chk_dec_en : 1; // bit18
		uint32_t soft_rst_dsp_debug_n : 1; // bit19
		uint32_t net_debug_bus_sel : 2; // bit[21:20]
		uint32_t net_parity_chk_en : 1; // bit22
		uint32_t dsp_parity_chk_en : 1; // bit23
		uint32_t dsp_axi_ecc_en : 1; // bit24
		uint32_t msgbox_parity_chk_dec_en : 1; // bit25
		uint32_t msgbox_parity_chk_enc_en : 1; // bit26
		uint32_t dsp_runstall : 1; // bit27
		uint32_t core_clk_en : 1; // bit28
		uint32_t dsp_clk_en : 1; // bit29
		uint32_t soft_rst_core_n : 1; // bit30
		uint32_t soft_rst_dsp_n : 1; // bit31
	} b;
} bstn_sys_ctrl_t;

#define BSTN_INTEN_CTRL 0x24
typedef union {
	uint32_t all;
	struct {
		uint32_t net0_core2dsp_error_int_en : 1; // bit0
		uint32_t net0_core2dsp_int_en : 1; // bit1
		uint32_t msgbox_0_int_en : 1; // bit2
		uint32_t msgbox_1_int_en : 1; // bit3
		uint32_t msgbox_2_int_en : 1; // bit4
		uint32_t msgbox_3_int_en : 1; // bit5
		uint32_t wdt_err_dsp_int_en : 1; // bit6
	} b;
} bstn_inten_ctrl_t;

#define BSTN_CORE0_CTRL 0x60
typedef union {
	uint32_t all;
	struct {
		uint32_t core0_greg_clk_en : 1;
		uint32_t core0_gemm_clk_en : 1;
		uint32_t core0_edp_clk_en : 1;
		uint32_t core0_hctl_clk_en : 1;
		uint32_t core0_dctl_clk_en : 1; // bit4
		uint32_t core0_dbuf_clk_en : 1;
		uint32_t core0_slice3_clk_en : 1;
		uint32_t core0_slice2_clk_en : 1;
		uint32_t core0_slice1_clk_en : 1; // bit8
		uint32_t core0_slice0_clk_en : 1;
		uint32_t core0_conv_clk_en : 1;
		uint32_t core0_btmem_clk_en : 1;
		uint32_t core0_ahb_clk_en : 1; // bit12
		uint32_t core0_clk_en : 1;
		uint32_t rsvd_14 : 2; // bit[15:14]
		uint32_t soft_rst_core0_greg_n : 1; // bit16
		uint32_t soft_rst_core0_gemm_n : 1;
		uint32_t soft_rst_core0_edp_n : 1;
		uint32_t soft_rst_core0_hctl_n : 1;
		uint32_t soft_rst_core0_dctl_n : 1; // bit20
		uint32_t soft_rst_core0_dbuf_n : 1;
		uint32_t soft_rst_core0_slice3_n : 1;
		uint32_t soft_rst_core0_slice2_n : 1;
		uint32_t soft_rst_core0_slice1_n : 1; // bit24
		uint32_t soft_rst_core0_slice0_n : 1;
		uint32_t soft_rst_core0_conv_n : 1;
		uint32_t soft_rst_core0_btmem_n : 1;
		uint32_t soft_rst_core0_ahb_n : 1; // bit28
		uint32_t soft_rst_core0_n : 1;
		uint32_t rsvd_30 : 2;
	} b;
} bstn_core0_ctrl_t;

#define DDRC0_CTRL 0x38001000
#define DDRC1_CTRL 0x3C001000
#define OCPARCFG0 0x330
#define OC_PARITY_EN BIT0

// BSTN_SYS_CTRL_STATUS register bit
#define BSTN_DSP_SOFT_RESET_BIT 31 // active-low
#define BSTN_NET_SOFT_RESET_BIT 30 // active-low
#define BSTN_DSP_CLK_EN_BIT 29
#define BSTN_CORE_CLK_EN_BIT 28
#define BSTN_DSP_RUNSTALL_BIT 27

#define BSTN_SYS_STATUS_OFFSET 0x4
#define BSTN_DSP_ALT_RESET_VEC_OFFSET 0x8

#define BSTN_LOADING_OFFSET 0x58

#pragma pack(push) /* push current alignment to stack */
#pragma pack(4) /* set alignment to 4 byte boundary */

struct bstn_fw_msginfo {
	uint32_t a78_irq_recved;
	uint32_t a78_irq_send;
	uint32_t other_irq_recved;
	uint32_t other_irq_send;
	uint32_t msgbx_recved;
	uint32_t msgbx_send;
};

// the initialization message body for runtime firmware
struct bstn_rt_setup_info {
	dsp_ptr assigned_mem;
	uint32_t assigned_mem_size;
	dsp_ptr rsp_addr;
	dsp_ptr msginfo_addr;
};
// the initialization response body for runtime firmware
struct bstn_rt_setup_rsp {
	uint32_t release_date;
	uint8_t ver_major;
	uint8_t ver_minor;
	uint8_t ver_patch;
};

#pragma pack(pop) /* restore original alignment from stack */

/* use mailbox reg0 to save boot flag */
#define BSTN_FW_BOOT_FLAG_OFFSET    (0x84)
/* regs default value 0x0 */
#define BSTN_FW_STATUS_INIT         (0x0)
/* 0xB00BA55A as flag, avoid other modify this regs don't know */
#define BSTN_FW_STATUS_BOOT_DONE    (0xB00BA55A)

struct bstn_fw_manager {
	char *name;
	char *name_msgbox;
	//the net registers
	void __iomem *net_sreg_base;
	//memory reserved for firmware codes
	void __iomem *fwmem_base;
	resource_size_t fwmem_size;
	phys_addr_t fwmem_phys_addr;
	dma_addr_t fwmem_iova;
	struct bstn_memblock *assigned_mem; //memory assigned to firmware
	unsigned char fw_setup;
	unsigned char ver_major;
	unsigned char ver_minor;
	unsigned char ver_patch;
	unsigned char release_date;
	unsigned char release_month;
	unsigned short release_year;

	struct bstn_hwsem *hwsem;
	bool main_os;

	bool fw_boot_done;
};

int bstn_fw_manager_init(struct bstn_device *pbstn);
void bstn_fw_manager_exit(struct bstn_device *pbstn);

int bstn_fw_manager_map(struct bstn_device *pbstn);
void bstn_fw_manager_unmap(struct bstn_device *pbstn);

void bstn_fw_rt_exit(struct bstn_device *pbstn);
int bstn_firmware_load(struct bstn_device *pbstn);
void bstn_firmware_stall(struct bstn_device *pbstn);
void bstn_firmware_unstall(struct bstn_device *pbstn);
void bstn_firmware_boot(struct bstn_device *pbstn);

void bstn_fw_set_boot_flag(struct bstn_device *pbstn);
bool bstn_fw_is_boot(struct bstn_device *pbstn);

#endif
