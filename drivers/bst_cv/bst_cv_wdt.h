/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

/* wdt:     watchdog device driver for Black Sesame Technologies
 * @author: AI Tools Team, BST Ltd.
 *
 * @file    wdt.h
 * @brief   This file is the header file of the firmware manager part of the
 *          watchdog driver. It contains related constants and structure
 *          definitions and function declarations.
 */
#ifndef _BST_CV_WDT_H_
#define _BST_CV_WDT_H_

#if defined(__QNX__)

#include <sys/iofunc.h>
#include <sys/mman.h>
#include <stdint.h>
#include <stdio.h>

#elif defined(__linux__)

#include <linux/printk.h>
#include <linux/io.h>

#else

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <xtensa/config/core-isa.h>
#if XCHAL_HAVE_EXTERN_REGS
#include <xtensa/tie/xt_externalregisters.h>
#endif

#endif

#define WDT_MODE_RESET                                  (0)	//1:reset mode, 0:int mode
#define WDT_BST_CV_DSP_ID                               (2)
#define WDT_PING_TIME_DEFAULT                           (0x0a)

#define LSP_1_CRM_BASE_ADDR                             (0x20021000)
#define LSP_1_CRM_REG_SIZE                              (0x4)
#define LSP_1_CRM_GLB_CTRL_OFFSET                       (0x4)
#define LSP_1_CRM_GLB_CTRL_PAUSE_ENABLE_BIT_OFFSET(n)   (15 + (((n)-2) * 3))
#define LSP_1_CRM_GLB_CTRL_GATE_ENABLE_BIT_OFFSET(n)    (8 + (n) - 2)

#define SEC_SAFE_SYS_CTRL_BASE_ADDR                     (0x70035000)
#define SEC_SAFE_SYS_CTRL_REG_SIZE                      (0x4)
#define SEC_SAFE_SYS_CTRL_REST_SEL_OFFSET               (0xc)
#define SEC_SAFE_SYS_CTRL_REST_SEL_WDT_BIT_OFFSET(n)    (n)

#define WDT_NUM                                         (2)
#define WDT_ID_MIN                                      (2)
#define WDT_ID_MAX                                      (3)

#define WDT_REG_BASE_ADDR(n)                            (0x2001a000 + (0x1000 * (n)))
#define WDT_REG_SIZE                                    (0x10)

#define WDT_PING_TIME_OFFSET                            (0x0)
#define WDT_PING_TIME_MASK                              (0xf)
#define WDT_PING_TIME_MIN                               (0x0)
#define WDT_PING_TIME_MAX                               (0xf)

#define WDT_CR_OFFSET                                   (0x0)
#define WDT_CR_RPL_MASK                                 (0x7)
#define WDT_CR_RPL_BIT_OFFSET                           (0x2)
#define WDT_CR_RPL_DEFAULT                              (0x4)
#define WDT_CR_RMOD_MASK                                (0x1)
#define WDT_CR_RMOD_BIT_OFFSET                          (0x1)
#define WDT_CR_ENABLE_MASK                              (0x1)
#define WDT_CR_ENABLE_BIT_OFFSET                        (0x0)

#define WDT_TORR_OFFSET                                 (0x4)
#define WDT_TORR_TOP_MASK                               (0xf)
#define WDT_TORR_TOP_BIT_OFFSET                         (0x0)

#define WDT_CRR_OFFSET                                  (0xc)
#define WDT_FEED_VAL                                    (0x76)

#define WDT_DEBUG_PRINT                                 (2)
#define WDT_LOG_PRINT                                   (1)
#define WDT_NO_PRINT                                    (0)

#ifdef DEBUG
__attribute__ ((unused))
static int wdt_print_level = WDT_DEBUG_PRINT;
#else
__attribute__ ((unused))
static int wdt_print_level = WDT_LOG_PRINT;
#endif

//////////////////////// for qnx arm ////////////////////////
#if defined(__QNX__)

typedef void wdt_reg_t;

#define WDT_DRIVER_NAME                     ("cv_wdt(qnx)")
#define WDT_EFAULT                          (EFAULT)
#define wdt_read_reg(addr)                  (*((volatile uint32_t *)(addr)))
#define wdt_write_reg(addr, val)            (*((volatile uint32_t *)(addr)) = ((uint32_t) (val)))
#define wdt_reg_remap(addr, size)           mmap_device_memory(NULL, size, \
	PROT_NOCACHE|PROT_READ|PROT_WRITE, 0, addr)
#define wdt_reg_unmap(addr, size)           munmap_device_memory(addr, size)

#define WDT_TRACE_PRINT(format, ...)      do { \
	if (wdt_print_level >= WDT_DEBUG_PRINT) \
		fprintf(stdout, "[%s]: %s %d: " format "\n", \
		WDT_DRIVER_NAME, __func__, __LINE__, ##__VA_ARGS__); \
} while (0)

#define WDT_STAGE_PRINT(format, ...)      do { \
	if (wdt_print_level >= WDT_LOG_PRINT) \
		fprintf(stdout, "[%s]: %s %d: " format "\n", \
		WDT_DRIVER_NAME, __func__, __LINE__, ##__VA_ARGS__); \
} while (0)

#define WDT_ERROR_PRINT(format, ...)         fprintf(stderr, "%s %d: " \
		format "\n", __func__, __LINE__, ##__VA_ARGS__)

// //////////////////////// for linux arm ////////////////////////
#elif defined(__linux__)

typedef void __iomem wdt_reg_t;

#define WDT_DRIVER_NAME                     ("cv_wdt(linux)")
#define WDT_EFAULT                          (EFAULT)
#define wdt_read_reg(addr)                  (readl_relaxed(addr))
#define wdt_write_reg(addr, val)            (writel_relaxed(val, addr))
#define wdt_reg_remap(addr, size)           ioremap(addr, size)
#define wdt_reg_unmap(addr, size)           iounmap(addr)

#define WDT_TRACE_PRINT(format, ...)      do { \
	if (wdt_print_level >= WDT_DEBUG_PRINT) \
		printk(KERN_DEBUG "[%s]: %s %d: " format "\n", \
		WDT_DRIVER_NAME, __func__, __LINE__, ##__VA_ARGS__); \
} while (0)

#define WDT_STAGE_PRINT(format, ...)      do { \
	if (wdt_print_level >= WDT_LOG_PRINT) \
		printk(KERN_INFO "[%s]: %s %d: " format "\n", \
		WDT_DRIVER_NAME, __func__, __LINE__, ##__VA_ARGS__); \
} while (0)

#define WDT_ERROR_PRINT(format, ...)         printk(KERN_ERR "%s %d: " \
		format "\n", __func__, __LINE__, ##__VA_ARGS__)

//////////////////////// for dsp ////////////////////////
#else

typedef void wdt_reg_t;

#define WDT_DRIVER_NAME                     ("cv_wdt")
#define WDT_EFAULT                          (1)
#define wdt_read_reg(addr)                  (*((volatile uint32_t *)(addr)))
#define wdt_write_reg(addr, val)            (*((volatile uint32_t *)(addr)) = ((uint32_t) (val)))
#define wdt_reg_remap(addr, size)           ((wdt_reg_t *)((addr)))
#define wdt_reg_unmap(addr, size)           do {} while (0)

#define WDT_TRACE_PRINT(format, ...)      do { \
	if (wdt_print_level >= WDT_DEBUG_PRINT) \
		dprintf("[%s]: %s %d: " format "\n", \
			WDT_DRIVER_NAME, __func__, __LINE__, ##__VA_ARGS__); \
} while (0)

#define WDT_STAGE_PRINT(format, ...)      do { \
	if (wdt_print_level >= WDT_LOG_PRINT) \
		dprintf("[%s]: %s %d: " format "\n", \
			WDT_DRIVER_NAME, __func__, __LINE__, ##__VA_ARGS__); \
} while (0)

#define WDT_ERROR_PRINT(format, ...)         dprintf("%s %d: " format "\n", \
			__func__, __LINE__, ##__VA_ARGS__)

#endif

struct _wdt_ctl {
	wdt_reg_t *ctl_reg[WDT_NUM];
	wdt_reg_t *rst_reg;
	wdt_reg_t *glb_reg;
	unsigned char init_flag;
	unsigned char config_flag;
};

int wdt_cv_init(void);
int wdt_cv_config(uint32_t wdt_id, uint32_t wdt_tm);
int wdt_cv_start(int wdt_id);
int wdt_cv_stop(int wdt_id);
int wdt_cv_feed(int wdt_id);
void wdt_cv_release(void);

#endif
