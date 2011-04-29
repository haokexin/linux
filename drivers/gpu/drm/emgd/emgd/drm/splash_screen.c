/* -*- pse-c -*-
 *-----------------------------------------------------------------------------
 * Filename: splash_screen.c
 * $Revision: 1.4 $
 *-----------------------------------------------------------------------------
 * Copyright (c) 2002-2010, Intel Corporation.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 *-----------------------------------------------------------------------------
 * Description:
 *  This is the Intel Embedded Graphics EFI Driver Splash Screen implementation
 *  file. This code shows a splash screen with a customizable icon.
 *-----------------------------------------------------------------------------
 */
#define MODULE_NAME hal.oal

#include <drm/drmP.h>
#include <drm/drm.h>
#include "splash_screen.h"
#include "bitmap_data.h"
/**
 * Function to display a splash screen to the user. The splash screen must be
 * accessible to the kernel mode driver as it has to be displayed immediately
 * after setting the mode (if requested by the user through config options).
 *
 * @param ss_data (IN) a non null pointer to splash screen information like
 * width, height etc.
 *
 * @return 0 on Success
 * @return <0 on Error
 */
void display_splash_screen(
	igd_framebuffer_info_t *fb_info,
	unsigned char *fb,
	emgd_drm_splash_screen_t *ss_data)
{
	unsigned char *fb_addr, *icon_temp;
	unsigned long *fb_addr_long, icon_long;
	unsigned long bitmap_pitch;
	short x, y;
	unsigned long init_x_shift, init_y_shift;
	unsigned long row, col, fb_index;
	unsigned long bytecount, temp;

	x = (short) ss_data->x;
	y = (short) ss_data->y;

	if(x < 0) {
		init_x_shift = (fb_info->width + x) * 4;
		init_y_shift = (fb_info->height + y) * fb_info->screen_pitch;
	} else {
		init_x_shift = x * 4;
		init_y_shift = y * fb_info->screen_pitch;
	}

	fb_addr = fb + init_y_shift;
	bytecount = (unsigned long) bitmap_data[0];
	bitmap_pitch = ss_data->width * bytecount;

	for(row = 0; row < ss_data->height; row++) {
		fb_addr_long =
			(unsigned long *) &fb_addr[fb_info->screen_pitch * row +
			init_x_shift];
		/*
		 * We are adding 2 bytes here, because the first byte is the bytecount
		 * and the second byte is the palette count
		 */
		icon_temp = &bitmap_data[2 + (row * bitmap_pitch)];
		fb_index = 0;

		for(col = 0; col < ss_data->width; col++) {

			icon_long = *((unsigned long *) &icon_temp[col*bytecount]);
			switch(bytecount) {
				case 1:
					/* 8 bit */
					temp = (icon_long & 0xFF);
					icon_long = ((temp & 0xE0)<<16) | ((temp & 0x1C)<<11) |
						((temp & 0x3)<<6);
					break;
				case 2:
					/* 16 bit */
					temp = (icon_long & 0xFFFF);
					icon_long = CONV_16_TO_32_BIT(temp);
					break;
			}
			/*
			 * For 24 bit we don't really have to do anything as it is already in
			 * RGB 888 format
			 */
			fb_addr_long[fb_index++] = icon_long & 0x00FFFFFF;
		}
	}
}
