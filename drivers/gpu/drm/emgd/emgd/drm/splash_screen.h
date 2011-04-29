/* -*- pse-c -*-
 *-----------------------------------------------------------------------------
 * Filename: splash_screen.h
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
 *  This is the Intel Embedded Graphics EFI Driver Splash Screen header file.
 *  This file contains data structures pertinent to showing a splash screen
 *  with a customizable icon.
 *-----------------------------------------------------------------------------
 */

#ifndef _SPLASH_SCREEN_H
#define _SPLASH_SCREEN_H

#include <user_config.h>

#define CONV_16_TO_32_BIT(a) (((a & 0xF800)<<8) | ((a & 0x7E0)<<5) |\
						(a & 0x1F)<<3)

typedef struct _bitmap_header {
	/* What is the widht and height of the bitmap */
	unsigned short width;
	unsigned short height;
	/* If Negative, from bottom right, how much to go left by */
	/* If Positive, from top left, how much to go right by */
	short x_coord;
	/* If Negative, from bottom right, how much to go up by */
	/* If Positive, from top left, how much to go down by */
	short y_coord;
} bitmap_header;

void display_splash_screen(
	igd_framebuffer_info_t *fb_info,
	unsigned char *fb,
	emgd_drm_splash_screen_t *ss_data);

#endif
