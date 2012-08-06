/*
 * arch/arm/plat-spear/udc.c
 *
 * Copyright (C) 2010 ST Microelectronics
 * Rajeev Kumar<rajeev-dlh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/usb/ch9.h>
#include <plat/udc.h>

#define EP(idx, nam, size, maxpkt, address)	\
	[idx] = {				\
		.name		= nam,		\
		.fifo_size	= size,		\
		.maxpacket	= maxpkt,	\
		.addr		= address,	\
	}

static struct udc_ep_data udc_ep[] __initdata = {
	EP(0, "ep0-ctrl", 64/4 , 64 , 0),
	EP(1, "ep1in"   , 512/4, 512, USB_DIR_IN  | 1),
	EP(2, "ep2out"  , 512/4, 512, USB_DIR_OUT | 2),
	EP(3, "ep3in"   , 512/4, 512, USB_DIR_IN  | 3),
	EP(4, "ep4out"  , 512/4, 512, USB_DIR_OUT | 4),
	EP(5, "ep5in"   , 512/4, 512, USB_DIR_IN  | 5),
	EP(6, "ep6out"  , 512/4, 512, USB_DIR_OUT | 6),
	EP(7, "ep7in"   , 512/4, 512, USB_DIR_IN  | 7),
	EP(8, "ep8out"  , 512/4, 512, USB_DIR_OUT | 8),
	EP(9, "ep9in"   , 512/4, 512, USB_DIR_IN  | 9),
	EP(10, "ep10out"   , 512/4, 512, USB_DIR_OUT | 10),
	EP(11, "ep11in"   , 512/4, 512, USB_DIR_IN  | 11),
	EP(12, "ep12out"   , 512/4, 512, USB_DIR_OUT | 12),
	EP(13, "ep13in"   , 512/4, 512, USB_DIR_IN  | 13),
	EP(14, "ep14out"   , 512/4, 512, USB_DIR_OUT | 14),
	EP(15, "ep15in"   , 512/4, 512, USB_DIR_IN  | 15),
};

struct udc_plat_data {
	int num_ep;
	struct udc_ep_data ep[UDC_MAX_ENDPOINT];
};

static __initdata struct udc_plat_data pdata;

void __init set_udc_plat_data(struct platform_device *pdev)
{
	pdata.num_ep = 16;
	memcpy(pdata.ep, udc_ep, sizeof(udc_ep));;

	if (platform_device_add_data(pdev, (struct udc_platform_data *)&pdata,
				sizeof(pdata)))
		pr_err("usb device: Error setting plat data");
}
