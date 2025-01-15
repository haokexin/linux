/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

/*!
 * wdt:     watchdog device driver for Black Sesame Technologies
 * @author: AI Tools Team, BST Ltd.
 *
 * @file    wdt.c
 * @brief   This file is the header file of the firmware manager part of the
 *          watchdog driver. It contains related constants and structure
 *          definitions and function declarations.
 */
#include "bst_cv_wdt.h"

static struct _wdt_ctl wdt_ctl = {
	.ctl_reg[0] = NULL,
	.ctl_reg[1] = NULL,
	.rst_reg = NULL,
	.glb_reg = NULL,
	.init_flag = 0,
	.config_flag = 0,
};

/*!
 * @brief       watchdog init
 * @return      0 - success, error code - failure
 */
int wdt_cv_init(void)
{
	uint32_t i;
	uint32_t phys_addr;

	wdt_ctl.init_flag = 0;
	wdt_ctl.config_flag = 0;

	for (i = 0; i < WDT_NUM; i++) {
		phys_addr = WDT_REG_BASE_ADDR(i + WDT_ID_MIN);
		WDT_TRACE_PRINT("phys_addr=0x%x", phys_addr);
		wdt_ctl.ctl_reg[i] = wdt_reg_remap(phys_addr, WDT_REG_SIZE);
		WDT_TRACE_PRINT("reg_addr=0x%p", wdt_ctl.ctl_reg[i]);
		if (wdt_ctl.ctl_reg[i] == NULL) {
			WDT_ERROR_PRINT("unable to map wdt regs 0x%x - 0x%x",
					phys_addr, phys_addr + WDT_REG_SIZE);
			return -WDT_EFAULT;
		}
	}

	phys_addr = LSP_1_CRM_BASE_ADDR + LSP_1_CRM_GLB_CTRL_OFFSET;
	WDT_TRACE_PRINT("phys_addr=0x%x", phys_addr);
	wdt_ctl.glb_reg = wdt_reg_remap(phys_addr, LSP_1_CRM_REG_SIZE);
	WDT_TRACE_PRINT("reg_addr=0x%p", wdt_ctl.glb_reg);
	if (wdt_ctl.glb_reg == NULL) {
		WDT_ERROR_PRINT("unable to map safe sys ctrl regs 0x%x",
				phys_addr);
		return -WDT_EFAULT;
	}

	phys_addr =
	    SEC_SAFE_SYS_CTRL_BASE_ADDR + SEC_SAFE_SYS_CTRL_REST_SEL_OFFSET;
	WDT_TRACE_PRINT("phys_addr=0x%x", phys_addr);
	wdt_ctl.rst_reg = wdt_reg_remap(phys_addr, SEC_SAFE_SYS_CTRL_REG_SIZE);
	WDT_TRACE_PRINT("reg_addr=0x%p", wdt_ctl.rst_reg);
	if (wdt_ctl.rst_reg == NULL) {
		WDT_ERROR_PRINT("unable to map safe sys ctrl regs 0x%x",
				phys_addr);
		return -WDT_EFAULT;
	}

	WDT_TRACE_PRINT("wdt init");

	wdt_ctl.init_flag = 1;
	return 0;
}

/*!
 * @brief       wdt release
 */
void wdt_cv_release(void)
{
	uint32_t i;

	if (wdt_ctl.init_flag != 1)
		return;

	for (i = 0; i < WDT_NUM; i++) {
		if (wdt_ctl.ctl_reg[i] != NULL) {
			wdt_reg_unmap(wdt_ctl.ctl_reg[i], WDT_REG_SIZE);
			wdt_ctl.ctl_reg[i] = NULL;
		}
	}

	if (wdt_ctl.rst_reg != NULL) {
		wdt_reg_unmap(wdt_ctl.rst_reg, SEC_SAFE_SYS_CTRL_REG_SIZE);
		wdt_ctl.rst_reg = NULL;
	}

	if (wdt_ctl.glb_reg != NULL) {
		wdt_reg_unmap(wdt_ctl.glb_reg, LSP_1_CRM_REG_SIZE);
		wdt_ctl.glb_reg = NULL;
	}

	wdt_ctl.init_flag = 0;
	wdt_ctl.config_flag = 0;
}

/*!
 * @brief       config watchdog
 * @param[in]   wdt_id - watchdog id (2-3)
 * @param[in]   wdt_tm - (0-0xf) 0xf:20sec, 0xe:10sec, 0xd:5sec, 0xc:2.5sec, ...
 * @return      0 - success, error code - failure
 */
int wdt_cv_config(uint32_t wdt_id, uint32_t wdt_tm)
{
	uint32_t reg;
	wdt_reg_t *reg_addr;

	if (wdt_ctl.config_flag == 1)
		return 0;

	if ((wdt_id > WDT_ID_MAX)
	    || (wdt_id < WDT_ID_MIN)
	    || (wdt_ctl.init_flag != 1)
	    || (wdt_tm > WDT_PING_TIME_MAX)) {
		return -WDT_EFAULT;
	}

	reg_addr = wdt_ctl.ctl_reg[wdt_id - WDT_ID_MIN] + WDT_CR_OFFSET;
	reg = wdt_read_reg(reg_addr);
	WDT_TRACE_PRINT("reg_addr=0x%p,reg=0x%x", reg_addr, reg);
	reg &= ~(WDT_CR_RPL_MASK << WDT_CR_RPL_BIT_OFFSET);
	reg |=
	    ((WDT_CR_RPL_DEFAULT & WDT_CR_RPL_MASK) << WDT_CR_RPL_BIT_OFFSET);
#if WDT_MODE_RESET
	reg &= (~(WDT_CR_RMOD_MASK << WDT_CR_RMOD_BIT_OFFSET));
#else
	reg |= (WDT_CR_RMOD_MASK << WDT_CR_RMOD_BIT_OFFSET);
#endif
	wdt_write_reg(reg_addr, reg);
	WDT_TRACE_PRINT("addr=0x%x,reg=0x%x",
			WDT_REG_BASE_ADDR(wdt_id) + WDT_CR_OFFSET, reg);

	reg_addr = wdt_ctl.ctl_reg[wdt_id - WDT_ID_MIN] + WDT_TORR_OFFSET;
	reg = wdt_read_reg(reg_addr);
	WDT_TRACE_PRINT("reg_addr=0x%p,reg=0x%x", reg_addr, reg);
	reg &= ~(WDT_TORR_TOP_MASK << WDT_TORR_TOP_BIT_OFFSET);
	reg |= ((wdt_tm & WDT_TORR_TOP_MASK) << WDT_TORR_TOP_BIT_OFFSET);
	wdt_write_reg(reg_addr, reg);
	WDT_TRACE_PRINT("addr=0x%x,reg=0x%x",
			WDT_REG_BASE_ADDR(wdt_id) + WDT_TORR_OFFSET, reg);

	reg_addr = wdt_ctl.rst_reg;
	reg = wdt_read_reg(reg_addr);
	WDT_TRACE_PRINT("reg_addr=%p,reg=0x%x", reg_addr, reg);
#if WDT_MODE_RESET
	if ((reg & (1 << SEC_SAFE_SYS_CTRL_REST_SEL_WDT_BIT_OFFSET(wdt_id))) !=
	    0) {
		reg &=
		    (~(1 << SEC_SAFE_SYS_CTRL_REST_SEL_WDT_BIT_OFFSET(wdt_id)));
		wdt_write_reg(reg_addr, reg);
		WDT_TRACE_PRINT("addr=0x%x,reg=0x%x",
				SEC_SAFE_SYS_CTRL_BASE_ADDR +
				SEC_SAFE_SYS_CTRL_REST_SEL_OFFSET, reg);
	}
#else
	if ((reg & (1 << SEC_SAFE_SYS_CTRL_REST_SEL_WDT_BIT_OFFSET(wdt_id))) ==
	    0) {
		reg |= (1 << SEC_SAFE_SYS_CTRL_REST_SEL_WDT_BIT_OFFSET(wdt_id));
		wdt_write_reg(reg_addr, reg);
		WDT_TRACE_PRINT("addr=0x%x,reg=0x%x",
				SEC_SAFE_SYS_CTRL_BASE_ADDR +
				SEC_SAFE_SYS_CTRL_REST_SEL_OFFSET, reg);
	}
#endif

	wdt_ctl.config_flag = 1;

	WDT_STAGE_PRINT("wdt %d config (wdt_tm=0x%x)", wdt_id, wdt_tm);
	return 0;
}

/*!
 * @brief       start watchdog
 * @param[in]   wdt_id - watchdog id (2-3)
 * @return      0 - success, error code - failure
 */
int wdt_cv_start(int wdt_id)
{
	uint32_t reg;
	wdt_reg_t *reg_addr;

	if ((wdt_id > WDT_ID_MAX)
	    || (wdt_id < WDT_ID_MIN)
	    || (wdt_ctl.init_flag != 1)) {
		return -WDT_EFAULT;
	}
	// if wdt_tm == 0, block wdt start
	reg_addr = wdt_ctl.ctl_reg[wdt_id - WDT_ID_MIN] + WDT_TORR_OFFSET;
	reg = wdt_read_reg(reg_addr);
	if ((reg & (WDT_TORR_TOP_MASK << WDT_TORR_TOP_BIT_OFFSET)) == 0) {
		WDT_STAGE_PRINT("wdt %d tm = 0, wdt did not start.", wdt_id);
		return 0;
	}

	reg_addr = wdt_ctl.ctl_reg[wdt_id - WDT_ID_MIN] + WDT_CRR_OFFSET;
	wdt_write_reg(reg_addr, WDT_FEED_VAL);
	WDT_TRACE_PRINT("addr=0x%x,reg=0x%x",
			WDT_REG_BASE_ADDR(wdt_id) + WDT_CRR_OFFSET,
			WDT_FEED_VAL);

	reg_addr = wdt_ctl.glb_reg;
	reg = wdt_read_reg(reg_addr);
	WDT_TRACE_PRINT("reg_addr=0x%p,reg=0x%x", reg_addr, reg);
	reg &= (~(1 << LSP_1_CRM_GLB_CTRL_PAUSE_ENABLE_BIT_OFFSET(wdt_id)));
	reg |= (1 << LSP_1_CRM_GLB_CTRL_GATE_ENABLE_BIT_OFFSET(wdt_id));
	wdt_write_reg(reg_addr, reg);
	WDT_TRACE_PRINT("addr=0x%x,reg=0x%x",
			LSP_1_CRM_BASE_ADDR + LSP_1_CRM_GLB_CTRL_OFFSET, reg);

	reg_addr = wdt_ctl.ctl_reg[wdt_id - WDT_ID_MIN] + WDT_CR_OFFSET;
	reg = wdt_read_reg(reg_addr);
	WDT_TRACE_PRINT("reg_addr=0x%p,reg=0x%x", reg_addr, reg);
	reg |= (WDT_CR_ENABLE_MASK << WDT_CR_ENABLE_BIT_OFFSET);
	wdt_write_reg(reg_addr, reg);
	WDT_TRACE_PRINT("addr=0x%x,reg=0x%x",
			WDT_REG_BASE_ADDR(wdt_id) + WDT_CR_OFFSET, reg);

	reg_addr = wdt_ctl.ctl_reg[wdt_id - WDT_ID_MIN] + WDT_CRR_OFFSET;
	wdt_write_reg(reg_addr, WDT_FEED_VAL);
	WDT_TRACE_PRINT("addr=0x%x,reg=0x%x",
			WDT_REG_BASE_ADDR(wdt_id) + WDT_CRR_OFFSET,
			WDT_FEED_VAL);

	WDT_STAGE_PRINT("wdt %d start", wdt_id);
	return 0;
}

/*!
 * @brief       stop watchdog
 * @param[in]   wdt_id - watchdog id (2-3)
 * @return      0 - success, error code - failure
 */
int wdt_cv_stop(int wdt_id)
{
	uint32_t reg;
	wdt_reg_t *reg_addr;

	if ((wdt_id > WDT_ID_MAX)
	    || (wdt_id < WDT_ID_MIN)
	    || (wdt_ctl.init_flag != 1)) {
		return -WDT_EFAULT;
	}

	reg_addr = wdt_ctl.glb_reg;
	reg = wdt_read_reg(reg_addr);
	reg |= (1 << LSP_1_CRM_GLB_CTRL_PAUSE_ENABLE_BIT_OFFSET(wdt_id));
	reg &= (~(1 << LSP_1_CRM_GLB_CTRL_GATE_ENABLE_BIT_OFFSET(wdt_id)));
	wdt_write_reg(reg_addr, reg);
	WDT_TRACE_PRINT("addr=0x%x,reg=0x%x",
			LSP_1_CRM_BASE_ADDR + LSP_1_CRM_GLB_CTRL_OFFSET, reg);

	reg_addr = wdt_ctl.ctl_reg[wdt_id - WDT_ID_MIN] + WDT_CR_OFFSET;
	reg = wdt_read_reg(reg_addr);
	reg &= (~(WDT_CR_ENABLE_MASK << WDT_CR_ENABLE_BIT_OFFSET));
	wdt_write_reg(reg_addr, reg);
	WDT_TRACE_PRINT("addr=0x%x,reg=0x%x",
			WDT_REG_BASE_ADDR(wdt_id) + WDT_CR_OFFSET, reg);

	WDT_STAGE_PRINT("wdt %d stop", wdt_id);
	return 0;
}

/*!
 * @func    _bst_cv_wdt_feed
 * @brief   feed watchdog
 * @params  wdt_id - watchdog id (2-3)
 * @return  0 - success
 *          error code - failure
 */
int wdt_cv_feed(int wdt_id)
{
	uint32_t reg;
	wdt_reg_t *reg_addr;

	if ((wdt_id > WDT_ID_MAX)
	    || (wdt_id < WDT_ID_MIN)
	    || (wdt_ctl.init_flag != 1)) {
		return -WDT_EFAULT;
	}
	// if wdt_tm == 0, no need feed
	reg_addr = wdt_ctl.ctl_reg[wdt_id - WDT_ID_MIN] + WDT_TORR_OFFSET;
	reg = wdt_read_reg(reg_addr);
	if ((reg & (WDT_TORR_TOP_MASK << WDT_TORR_TOP_BIT_OFFSET)) == 0)
		return 0;

	reg_addr = wdt_ctl.ctl_reg[wdt_id - WDT_ID_MIN] + WDT_CRR_OFFSET;
	wdt_write_reg(reg_addr, WDT_FEED_VAL);
	WDT_TRACE_PRINT("addr=0x%x,reg=0x%x",
			WDT_REG_BASE_ADDR(wdt_id) + WDT_CRR_OFFSET,
			wdt_read_reg(wdt_ctl.ctl_reg[wdt_id - WDT_ID_MIN] +
				     WDT_CRR_OFFSET));

	return 0;
}
