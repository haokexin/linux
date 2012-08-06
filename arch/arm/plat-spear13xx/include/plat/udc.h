/*
 * arch/arm/plat-spear/include/plat/udc.h
 *
 * Copyright (C) 2010 ST Microelectronics
 * Rajeev Kumar<rajeev-dlh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
*/

#ifndef __PLAT_UDC_H
#define __PLAT_UDC_H

#include <linux/platform_device.h>

#define UDC_MAX_ENDPOINT	16

/*
 * Platform data definitions for synopsys gadget driver.
 */
struct udc_ep_data {
	char	*name;
	int	fifo_size;
	int	maxpacket;
	u8	addr;
	u8	attrib;
};

struct udc_platform_data {
	int num_ep;
	struct udc_ep_data ep[0];
};

void set_udc_plat_data(struct platform_device *pdev);
#endif
