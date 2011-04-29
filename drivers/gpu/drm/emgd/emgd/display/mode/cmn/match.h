/* -*- pse-c -*-
 *-----------------------------------------------------------------------------
 * Filename: match.h
 * $Revision: 1.5 $
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
 *  This file contains the mode matching algorithms
 *-----------------------------------------------------------------------------
 */

#include <mode.h>

int validate_fb(
	pigd_framebuffer_info_t fb_info,
	igd_display_plane_t *plane);

int validate_cursor(
	igd_cursor_info_t *cursor_info,
	igd_display_context_t *display);

int match_mode (
	igd_display_context_t *display,
	igd_timing_info_t *timing_table,
	igd_framebuffer_info_t *fb_info,
	igd_display_info_t *pt_info,
	igd_timing_info_t **timing);


/*----------------------------------------------------------------------------
 * File Revision History
 * $Id: match.h,v 1.5 2011/03/02 22:47:05 astead Exp $
 * $Source: /nfs/fm/proj/eia/cvsroot/koheo/linux/egd_drm/emgd/display/mode/cmn/match.h,v $
 *----------------------------------------------------------------------------
 */
