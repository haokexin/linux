/*
 * Copyright (C) 2010 Wind River Systems
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define XFER_CONFIG			(0x0)
#define RECV_CONFIG			(0x4)
#define XFER_STATUS			(0x8)
#define RECV_STATUS			(0xc)
#define INTR_ENABLE			(0x10)
#define INTR_CLEAR			(0X14)
#define INTR_STATUS			(0x18)
#define CLK_CONFIG			(0x1c)
#define START_SETUP_HOLD_CONFIG		(0x20)
#define STOP_SETUP_HOLD_CONFIG		(0x24)
#define DATA_SETUP_HOLD_CONFIG		(0x28)
#define BYPASS_MODE			(0x2c)
#define SLAVE_ADDRESS			(0x30)
#define XFER_DATA0			(0x34)
#define XFER_DATA1			(0x38)
#define RECV_DATA0			(0x3c)
#define RECV_DATA1			(0x40)

#define I2C_TIMER_LOAD			(0x0)
#define I2C_TIMER_CONTROL		(0x8)

#define CFG_CLK_CONFIG_SCL_LOW		(10 * 40)   /* Yx 1.2 us / 30 ns  */
#define CFG_CLK_CONFIG_SCL_LOW_RD	(256)       /* Yx 0.75 us / 30 ns */
#define CFG_CLK_CONFIG_SCL_HIGH		(10 * 20)   /* Yx 0.6 us / 30 ns  */
#define CFG_START_SETUP_PERIOD		20          /* 0.6 us / 30 ns     */
#define CFG_START_HOLD_PERIOD		20          /* 0.6 us / 30 ns     */
#define CFG_STOP_SETUP_PERIOD		20          /* 0.6 us / 30 ns     */
#define CFG_STOP_HOLD_PERIOD		0           /* 0.0 us / 30 ns     */
#define CFG_DATA_SETUP_PERIOD		2           /* 60 ns / 30 ns      */
#define CFG_DATA_HOLD_PERIOD		0           /* 0.0 us / 30 ns     */


struct xfer_config {
#ifdef __BIG_ENDIAN
	unsigned reserved2:21;
	unsigned zero_all:1;
	unsigned command:1;
	unsigned master_mode_active:1;
	unsigned ten_bit_addr:1;
	unsigned reserved1:1;
	unsigned endianness:1;
	unsigned nbytes:4;
	unsigned ready:1;
#else
	unsigned ready:1;
	unsigned nbytes:4;
	unsigned endianness:1;
	unsigned reserved1:1;
	unsigned ten_bit_addr:1;
	unsigned master_mode_active:1;
	unsigned command:1;
	unsigned zero_all:1;
	unsigned reserved2:21;
#endif
};

struct recv_config {
#ifdef __BIG_ENDIAN
	unsigned reserved:26;
	unsigned endianness:1;
	unsigned nbytes:4;
	unsigned ready:1;
#else
	unsigned ready:1;
	unsigned nbytes:4;
	unsigned endianness:1;
	unsigned reserved:26;
#endif
};

struct xfer_status {
#ifdef __BIG_ENDIAN
	unsigned reserved:30;
	unsigned error:1;
	unsigned done:1;
#else
	unsigned done:1;
	unsigned error:1;
	unsigned reserved:30;
#endif
};

struct recv_status {
#ifdef __BIG_ENDIAN
	unsigned reserved:26;
	unsigned nbytes_done:4;
	unsigned error:1;
	unsigned done:1;
#else
	unsigned done:1;
	unsigned error:1;
	unsigned nbytes_done:4;
	unsigned reserved:26;
#endif
};

struct intr_enable {
#ifdef __BIG_ENDIAN
	unsigned reserved:28;
	unsigned recv_error:1;
	unsigned recv_done:1;
	unsigned xfer_error:1;
	unsigned xfer_done:1;
#else
	unsigned xfer_done:1;
	unsigned xfer_error:1;
	unsigned recv_done:1;
	unsigned recv_error:1;
	unsigned reserved:28;
#endif
};

struct clk_config {
#ifdef __BIG_ENDIAN
	unsigned reserved2:6;
	unsigned pclk_high:10;
	unsigned reserved1:6;
	unsigned pclk_low:10;
#else
	unsigned pclk_low:10;
	unsigned reserved1:6;
	unsigned pclk_high:10;
	unsigned reserved2:6;
#endif
};

struct hold_setup_clk {
#ifdef __BIG_ENDIAN
	unsigned reserved2:6;
	unsigned pclk_hold:10;
	unsigned reserved1:6;
	unsigned pclk_setup:10;
#else
	unsigned pclk_setup:10;
	unsigned reserved1:6;
	unsigned pclk_hold:10;
	unsigned reserved2:6;
#endif
};
