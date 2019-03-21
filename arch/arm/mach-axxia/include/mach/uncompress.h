/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (C) 2021 INTEL Corporation
 */

#define AMBA_UART_DR(base)	(*(unsigned char *)((base) + 0x00))
#define AMBA_UART_LCRH(base)	(*(unsigned char *)((base) + 0x2c))
#define AMBA_UART_CR(base)	(*(unsigned char *)((base) + 0x30))
#define AMBA_UART_FR(base)	(*(unsigned char *)((base) + 0x18))

#if defined(CONFIG_DEBUG_VEXPRESS_CA9X4_UART)
#define get_uart_base()	(0x10000000 + 0x00009000)
#elif defined(CONFIG_DEBUG_VEXPRESS_RS1_UART)
#define get_uart_base()	(0x1c000000 + 0x00090000)
#else
#define get_uart_base() (0UL)
#endif

/*
 * This does not append a newline
 */
static inline void putc(int c)
{
	unsigned long base = get_uart_base();

	if (!base)
		return;

	while (AMBA_UART_FR(base) & (1 << 5))
		barrier();

	AMBA_UART_DR(base) = c;
}

static inline void flush(void)
{
	unsigned long base = get_uart_base();

	if (!base)
		return;

	while (AMBA_UART_FR(base) & (1 << 3))
		barrier();
}

/*
 * nothing to do
 */
#define arch_decomp_setup()
#define arch_decomp_wdog()
