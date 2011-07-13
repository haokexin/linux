/*
 * 16550 serial console support.
 *
 * Original copied from <file:arch/ppc/boot/common/ns16550.c>
 * (which had no copyright)
 * Modifications: 2006 (c) MontaVista Software, Inc.
 *
 * Modified by: Mark A. Greer <mgreer@mvista.com>
 */
#include <stdarg.h>
#include <stddef.h>
#include "types.h"
#include "string.h"
#include "stdio.h"
#include "io.h"
#include "ops.h"

#define UART_DLL	0	/* Out: Divisor Latch Low */
#define UART_DLM	1	/* Out: Divisor Latch High */
#define UART_FCR	2	/* Out: FIFO Control Register */
#define UART_LCR	3	/* Out: Line Control Register */
#define UART_MCR	4	/* Out: Modem Control Register */
#define UART_LSR	5	/* In:  Line Status Register */
#define UART_LSR_THRE	0x20	/* Transmit-hold-register empty */
#define UART_LSR_DR	0x01	/* Receiver data ready */
#define UART_MSR	6	/* In:  Modem Status Register */
#define UART_SCR	7	/* I/O: Scratch Register */

static unsigned char *reg_base;
static u32 reg_shift;

static int ns16550_open(void)
{
	out_8(reg_base + (UART_FCR << reg_shift), 0x06);
	return 0;
}

static void ns16550_putc(unsigned char c)
{
	while ((in_8(reg_base + (UART_LSR << reg_shift)) & UART_LSR_THRE) == 0);
	out_8(reg_base, c);
}

static unsigned char ns16550_getc(void)
{
	while ((in_8(reg_base + (UART_LSR << reg_shift)) & UART_LSR_DR) == 0);
	return in_8(reg_base);
}

static u8 ns16550_tstc(void)
{
	return ((in_8(reg_base + (UART_LSR << reg_shift)) & UART_LSR_DR) != 0);
}

#if defined(CONFIG_WRHV_P4080DS)

#define TLBWE_CODE          0x7C0007A4
#define MAS0_TLBSEL1_ENTRY  0x10000000
#define MAS1_VALID_4K_TSIZE (0x80000000|((1 << 8) & 0x00000F00))
#define MAS2_IG             0x0000000a
#define MAS2_SXWR           0x00000015

/*
 * Here we have to create a TLB0 entry to map uart since the
 * Hypervisor dose not map this while pass the guest OS on E500mc.
 */
static void wrhv_uart_tlb0_create(void)
{
	/* Write MAS0/1/2/3 to create tlb0 to map uart. And note
	 * r3 should be privileged instruction code as the HY expect. */
	__asm__ __volatile__(
						 "mtspr  0x270,%0\n"
						 "mtspr  0x271,%1\n"
						 "mtspr  0x272,%2\n"
						 "mtspr  0x273,%3\n"
						 "lis    3,%4@h\n"
						 "ori    3,3,%4@l\n"
						 "tlbwe\n"
						 ::"r" (MAS0_TLBSEL1_ENTRY), "r" (MAS1_VALID_4K_TSIZE),
						  "r" ((unsigned int)reg_base|MAS2_IG),
						  "r" ((unsigned int)reg_base|MAS2_SXWR),
						  "i" (TLBWE_CODE)
						 );
}

#endif

int ns16550_console_init(void *devp, struct serial_console_data *scdp)
{
	int n;
	u32 reg_offset;

	if (dt_get_virtual_reg(devp, (void **)&reg_base, 1) < 1)
		return -1;

	n = getprop(devp, "reg-offset", &reg_offset, sizeof(reg_offset));
	if (n == sizeof(reg_offset))
		reg_base += reg_offset;

	n = getprop(devp, "reg-shift", &reg_shift, sizeof(reg_shift));
	if (n != sizeof(reg_shift))
		reg_shift = 0;

#if defined(CONFIG_WRHV_P4080DS)
	wrhv_uart_tlb0_create();
#endif

	scdp->open = ns16550_open;
	scdp->putc = ns16550_putc;
	scdp->getc = ns16550_getc;
	scdp->tstc = ns16550_tstc;
	scdp->close = NULL;

	return 0;
}
