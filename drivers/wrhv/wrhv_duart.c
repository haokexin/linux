/*
 *  Copyright (C) 2011 Wind River Systems, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/wrhv.h>
#include <vbi/vbi.h>
#include <vbi/errors.h>
#include <vbi/pdc.h>

extern struct vb_config *wr_config;

/* interface to wrhv duart actual device driver */
#define WRHV_DUART_RX_SIZE	(16)
#define WRHV_DUART_TX_SIZE	(1024)
static char rxBuf[WRHV_DUART_RX_SIZE];
static char txBuf[WRHV_DUART_TX_SIZE];
char wrhv_duart_name[256];
vbi_pdc_handle duart_pdc;
int is_wrhv_duart_inited;

struct intr_device_channel_buffer idc = {
	.rxBuf = (void *)rxBuf,
	.rxBufLen = WRHV_DUART_RX_SIZE,
	.rxBufWrPtr = (size_t)0,
	.rxBufRdPtr = (size_t)0,
	.txBuf = (void *)txBuf,
	.txBufLen = WRHV_DUART_TX_SIZE,
	.txBufWrPtr = (size_t)0,
	.txBufRdPtr = (size_t)0
};

void __attribute__((weak)) wrhv_get_serial_devices(void)  { return; }
__attribute__((weak)) char *wrhv_get_serial_dev_name(int index)
{
	return NULL;
}

void wrhv_duart_putc(char c)
{
	if (((idc.txBufWrPtr + 1) % WRHV_DUART_TX_SIZE) == idc.txBufRdPtr) {
		/* tx ring buffer full */
		return ;
	}

	if (c == '\n') {
		wrhv_duart_putc('\r');
	}

	txBuf[idc.txBufWrPtr] = c;
	idc.txBufWrPtr = (idc.txBufWrPtr + 1) % WRHV_DUART_TX_SIZE;

	vbi_pdc_op(duart_pdc, PDC_REQUEST_WRITE,
		0, (void *)idc.txBuf, (size_t)idc.txBufLen, 0);

}

void wrhv_duart_puts(char *str)
{
	while (*str != 0) {
		wrhv_duart_putc(*str);
		str++;
	}
}

int wrhv_duart_tstc(void)
{
	if (idc.rxBufWrPtr == idc.rxBufRdPtr) {
		/* rx ring buffer empty */
		return 0;
	}
	return 1;
}

int wrhv_duart_getc(void)
{
	int value;

	if (!wrhv_duart_tstc()) {
		return 0;
	}
	value = rxBuf[idc.rxBufRdPtr++];
	idc.rxBufRdPtr = idc.rxBufRdPtr % WRHV_DUART_RX_SIZE;
	return value;
}

void wrhv_duart_init(void)
{
	const char *opt;
	char *pstring;
	unsigned char bootline[VB_MAX_BOOTLINE_LENGTH];

	/* this functions can be called very early, even before
	 * the platform code. The command line options needed
	 * to be directly from VBI_BOOTLINE
	 */
	strncpy(bootline, wr_config->bootLine, VB_MAX_BOOTLINE_LENGTH - 1);
	bootline[VB_MAX_BOOTLINE_LENGTH - 1] = 0;

	/* get the duart name, the duart name is
	 * specified in the hypervisor xml
	 */
	memset(wrhv_duart_name, 0, sizeof(wrhv_duart_name));
	pstring = wrhv_duart_name;
	opt = strstr(bootline, "duart=");
	if (opt) {
		opt += 6;
		while (*opt && (*opt != ' ')) {
			*pstring = *opt;
			pstring++;
			opt++;
		}
	} else {
		opt = strstr(bootline, "console=hvc");
		if (opt) {
			char *p;

			opt += 11;
			if (*opt < '0' || *opt > '9')
				return;
			wrhv_get_serial_devices();
			p = wrhv_get_serial_dev_name(*opt - '0');
			strcpy(pstring, p);
		} else
			return;
	}

	/* only init the duart once */
	if (is_wrhv_duart_inited) {
		return;
	}
	is_wrhv_duart_inited = 1;


	/* init channel */
	if (vbi_pdc_init(wrhv_duart_name, &duart_pdc)) {
		printk("vbi_pdc_init failed\n");
		return;
	}

	/* init device */
	if (vbi_pdc_op(duart_pdc, PDC_REQUEST_INIT,
		0, (void *) &idc, 0, 0)) {
		printk("vbi_pdc_op: PDC_REQUEST_INIT failed\n");
		return;
	}
}
