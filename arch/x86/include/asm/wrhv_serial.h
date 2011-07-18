#ifndef _ASM_X86_WRHV_SERIAL_H
#define _ASM_X86_WRHV_SERIAL_H

#include <asm/serial.h>

#undef SERIAL_PORT_DFNS
#define SERIAL_PORT_DFNS			\
	/* UART CLK   PORT IRQ     FLAGS        */			\
	{ 0, BASE_BAUD, 0x3F8, 4,  STD_COM_FLAGS },	/* ttyS0 */	\
	{ 0, BASE_BAUD, 0x2F8, 3,  STD_COM_FLAGS },	/* ttyS1 */	\
	{ 0, BASE_BAUD, 0x220, 30, STD_COM_FLAGS },	/* ttyS2 */	\
	{ 0, BASE_BAUD, 0x238, 31, STD_COM_FLAGS },	/* ttyS3 */

#endif /* _ASM_X86_WRHV_SERIAL_H */
