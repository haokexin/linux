/* -*- pse-c -*-
 *-----------------------------------------------------------------------------
 * Filename: match.c
 * $Revision: 1.10 $
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
 *
 *-----------------------------------------------------------------------------
 */

#define MODULE_NAME hal.mode


#define CURSOR_DEFAULT_WIDTH	64
#define CURSOR_DEFAULT_HEIGHT	64

#include <context.h>
#include <igd_init.h>
#include <io.h>
#include <memory.h>
#include <edid.h>
#include <pi.h>

#include <igd_mode.h>
#include <igd_errno.h>

#include <mode.h>
#include <config.h>

#include "match.h"

/*!
 * @addtogroup display_group
 * @{
 */

/* Local variables */
#ifndef CONFIG_MICRO
igd_cursor_info_t default_cursor = {
	CURSOR_DEFAULT_WIDTH,
	CURSOR_DEFAULT_HEIGHT,
	CONFIG_DEFAULT_PF,
	0, 0, 0, 0, 0, 0,
	0, 0, {0, 0, 0, 0}, IGD_CURSOR_ON, 0, 0, 0, 0
};

/*!
 *
 * @param cursor_info
 * @param display
 *
 * @return -IGD_INVAL on failure
 * @return 0 on success
 */
int validate_cursor(igd_cursor_info_t *cursor_info,
	igd_display_context_t *display)
{
	unsigned long *list_pfs;
	igd_display_pipe_t *pipe = (igd_display_pipe_t *)(display->pipe);

	EMGD_TRACE_ENTER;
	if (pipe) {
		if (pipe->cursor) {
			list_pfs = pipe->cursor->pixel_formats;

			while (*list_pfs) {
				if (cursor_info->pixel_format == *list_pfs) {
					return 0;
				}
				list_pfs++;
			}
		}
	}

	EMGD_TRACE_EXIT;
	return -IGD_INVAL;
}
#endif

/*!
 *
 * @param timing
 * @param pt_info
 *
 * @return void
 */
static void fill_pt(
	igd_timing_info_t *timing,
	pigd_display_info_t pt_info)
{
	unsigned long flags;

	EMGD_DEBUG("fill_pt Entry");

	/* preserve existing pt_info flags */
	flags = pt_info->flags;

	/* Simply memcpy the structures and fix up the flags */
	OS_MEMCPY(pt_info, timing, sizeof(igd_timing_info_t));

	pt_info->flags |= flags;

	/* pt_info doesn't require a IGD_MODE_VESA flag, so clear IGD_MODE_VESA
	 * Setting this flag creates issues in match mode. */
	pt_info->flags &= ~IGD_MODE_VESA;
	return;
}

#ifndef CONFIG_NEW_MATCH


extern igd_timing_info_t vga_timing_table[];
extern igd_timing_info_t crt_timing_table[];


#define MATCH_MOD(x)  ((x>0)?x:-x)
#define MATCH_EXACT    0x01
#define MATCH_NATIVE   0x02
#define MATCH_CENTER   0x10
#define MATCH_FOR_VGA  0x20

/*!
 *
 * @param display
 * @param timing_table
 * @param pt_info
 * @param type
 *
 * @return NULL on failure
 * @return timing on success
 */
static igd_timing_info_t *match_resolution(
		igd_display_context_t *display,
		igd_timing_info_t *timing_table,
		igd_display_info_t *pt_info,
		int type)
{
	igd_timing_info_t *timing;
	igd_timing_info_t *match;
	igd_timing_info_t *native_match = NULL;
	igd_display_port_t *port;

	EMGD_DEBUG("Enter match_resolution");

	EMGD_DEBUG("Width=%d, height=%d, refresh=%d mode_number=0x%x",
		pt_info->width, pt_info->height, pt_info->refresh,
		pt_info->mode_number);

	timing = timing_table;
	match = NULL;
	port = PORT_OWNER(display);

	/*
	 * Note on Native matching.
	 * The Ideal thing is for a fp_native_dtd to already be marked as such.
	 * If there is no native timing indicated then we must choose what is
	 * most likely correct.
	 * If the mode is not VGA then we should choose any DTD that closely
	 * matches the mode being set. Failing that we should choose any timing
	 * that closely matches the mode.
	 * If the mode is VGA then we should take the current mode as it is
	 * more likely correct.
	 */
	if(type == MATCH_NATIVE) {
		if(port->fp_native_dtd) {
			EMGD_DEBUG("Returning quick with a native match");

			EMGD_DEBUG("NATIVE Width=%d, height=%d, refresh=%d mode_num=0x%x",
				port->fp_native_dtd->width, port->fp_native_dtd->height,
				port->fp_native_dtd->refresh, port->fp_native_dtd->mode_number);

			return port->fp_native_dtd;
		}
		if((pt_info->flags & IGD_MODE_VESA) &&
			(pt_info->mode_number <= 0x13)) {
			if(PIPE(display)->timing) {
				native_match = PIPE(display)->timing;
			}
		}
	}

	while (timing->width != IGD_TIMING_TABLE_END) {
		if(!(timing->mode_info_flags & IGD_MODE_SUPPORTED)) {
			timing++;
			continue;
		}

		if(type == MATCH_NATIVE) {
			if(timing->mode_info_flags & IGD_MODE_DTD_FP_NATIVE) {
				port->fp_native_dtd = timing;
				return timing;
			}

			if(port->fp_info) {
				/*
				 * We may have only fp_width and fp_height which is really
				 * not enough information to be useful. If we find a
				 * matching width and height we'll keep the first one while
				 * still hoping to find an actual native mode later.
				 */
				if(!match &&
					(port->fp_info->fp_width ==
						(unsigned long)timing->width) &&
					(port->fp_info->fp_height ==
						(unsigned long)timing->height)) {
					match = timing;
				}
			} else {
				/*
				 * Keep a match because in the event that we never find a
				 * native DTD then we will just take the exact match.
				 */
				if(!match &&
					(timing->width == pt_info->width) &&
					(timing->height == pt_info->height) &&
					(timing->refresh == pt_info->refresh)) {
					match = timing;
				}
			}

			/*
			 * If it is a DTD then keep it only if it is better than any
			 * found before.
			 */
			if(timing->mode_info_flags & IGD_MODE_DTD_USER) {
				if(native_match) {
					if(MATCH_MOD((int)(pt_info->width*pt_info->height) -
							(native_match->width * native_match->height)) >
						MATCH_MOD((int)(pt_info->width*pt_info->height) -
							(timing->width*timing->height))) {
						native_match = timing;
					}
				} else {
					native_match = timing;
				}
			}
		} else if (type == MATCH_EXACT) {
			/*
			 * Looking for an exact match. For VGA/VESA it must match
			 * mode number. Otherwise it must match width, height, refresh
			 * etc.
			 */
			if(pt_info->flags & IGD_MODE_VESA) {
				/* ((timing->mode_info_flags & IGD_MODE_VESA)) */
				if((pt_info->mode_number == timing->mode_number) &&
					(!pt_info->refresh ||
						(pt_info->refresh == timing->refresh))) {
					match = timing;
					break;
				}
			} else {
				/* If exact match found, then break the loop */
				if((timing->width == pt_info->width) &&
					(timing->height == pt_info->height) &&
					(timing->refresh == pt_info->refresh) &&
					(
						(timing->mode_info_flags &
							(IGD_SCAN_INTERLACE|IGD_PIXEL_DOUBLE|
								IGD_LINE_DOUBLE)) ==
						(pt_info->flags &
							(IGD_SCAN_INTERLACE|IGD_PIXEL_DOUBLE|
								IGD_LINE_DOUBLE)))) {
					match = timing;

					/* If exact match found, then break the loop */
					if ((timing->mode_info_flags & PD_MODE_DTD_USER) ||
						(timing->mode_info_flags & PD_MODE_DTD)) {
						break;
					}
				}
			}
		}


		/* Center needs only to be bigger. Aspect ratio doesn't matter. */
		/*
		 * Note: The timings have to be big enough to fit the pt_info
		 * including any pixel double flags. VGA modes will sometimes be
		 * pixel doubled and need to be centered in a pipe that is double
		 * in size.
		 *
		 * Note2: 720x400 VGA modes can be centered in 640x480 with a
		 * special hardware config that drops every 9th pixel. Only do
		 * this when requested.
		 */
		else if(type & MATCH_CENTER) {
			unsigned short eff_width = pt_info->width;
			unsigned short eff_height = pt_info->height;

			if(type & MATCH_FOR_VGA) {
				/*
				 * 720x400 is a magic mode that means all VGA modes are supported
				 * always use that mode for centering if found.
				 */
				if((timing->width == 720) && (timing->height == 400)) {
					EMGD_DEBUG("Returning with a magic VGA mode");
					return timing;
				}
				if(pt_info->flags & IGD_PIXEL_DOUBLE) {
					eff_width *= 2;
				}
				if(pt_info->flags & IGD_LINE_DOUBLE) {
					eff_height *= 2;
				}
				if((eff_width == 720) &&
					(port->port_features & IGD_VGA_COMPRESS)) {
					eff_width = 640;
				}
			}

			if((timing->width >= eff_width) &&
				(timing->height >= eff_height) &&
				(timing->mode_info_flags & IGD_SCAN_INTERLACE) ==
				(pt_info->flags & IGD_SCAN_INTERLACE)) {
				if(match) {
					/* Check for tighter fit */
					if((match->width > timing->width) ||
						(match->height > timing->height)) {
						match = timing;
					}
					/* Try to match refreshrate as well */
					if((match->width == timing->width) &&
					   (match->height == timing->height) &&
					   (pt_info->refresh == timing->refresh)){
						match = timing;
					}
				} else {
					match = timing;
				}
			}
		}
		timing++;
	}

	if(native_match) {
		EMGD_DEBUG("Returning with a native match");
		EMGD_DEBUG("Width=%d, height=%d, refresh=%d mode_number=0x%x",
			native_match->width, native_match->height, native_match->refresh,
			native_match->mode_number);
		return native_match;
	}
	if (!match) {
		EMGD_DEBUG("Returning with NO match");
		return NULL;
	}

	EMGD_DEBUG("Returning with a match");
	EMGD_DEBUG("Width=%d, height=%d, refresh=%d mode_number=0x%x",
		match->width, match->height, match->refresh, match->mode_number);
	return match;
} /* end match_resolution */

static igd_timing_info_t scaled_timing[IGD_MAX_PIPES];

/*!
 * Match the fb and pt structures to a Mode Structure from the table.
 * When a mode is found update the input structures to reflect the
 * values found.
 *
 * Notes:
 *  Match mode has several options for what it can do. Foremost it should
 * attempt to find a mode matching the requested one from the timing table
 * provided. If the mode requested is not in the list this means one of
 * two things.
 *   1) The IAL is calling without checking modes. It is just passing down
 *  something that a user asked for. This is ok but we need to be safe so
 *  we return the next smaller mode with the same aspect ratio.
 *
 *   2) The IAL is requesting a very common "required" mode even though the
 *  port doesn't support it. In this case it should be in the static common
 *  modes table and can be centered in the next larger timings in the
 *  mode table.
 *
 * If the Frambuffer is smaller than the timings requested a fake set of
 * centered timings is returned to program the pipe.
 *
 * In the case of VGA modes. If the mode is in the mode table everything is
 * fine and we just return that. If it is not in the table we find the next
 * larger suitable mode and prepare to center in that mode. Using the static
 * timings from the VGA table as the VGA mode. We do not need to generate
 * a fake set of timings because VGA will center itself automatically in
 * hardware.
 *
 * In the case of LVDS both centering and scaling can happen. If the mode
 * is in the list it will be scaled to the Native Timings. If the mode
 * is not in the list (common or VGA) it will be centered in the next larger
 * supported mode and then scaled to the native timings.
 *
 * Centering is always indicated by returning the timings that should be
 * programmed to the pipe. The timings will then have their extension pointer
 * set to point to the centered timings. For centering with scaling the
 * first extension pointer will contain the scalable timings and the
 * second will contain the centering timings. The static "scaled_timings"
 * data structure will be used when the scaled timings need to be
 * created on the fly due to a Framebuffer that is smaller than the
 * timings.
 *
 * @param display
 * @param timing_table
 * @param fb_info
 * @param pt_info
 * @param timing
 *
 * @return -IGD_ERROR_INVAL on failure
 * @return 0 on success
 */
int match_mode (
	igd_display_context_t *display,
	igd_timing_info_t *timing_table,
	igd_framebuffer_info_t *fb_info,
	igd_display_info_t *pt_info,
	igd_timing_info_t **timing)
{
	igd_timing_info_t *exact_timing = NULL;
	igd_timing_info_t *pipe_timing = NULL;
	igd_timing_info_t *user_timing = NULL;
	igd_timing_info_t *native_timing = NULL;
	igd_timing_info_t *vga_timing = NULL;
	igd_timing_info_t *vesa_timing = NULL;
	short cntr_dff_w = 0;
	short cntr_dff_h = 0;
	unsigned long upscale = 0;

	EMGD_DEBUG("Enter Match Mode");

	if(!pt_info) {
		EMGD_ERROR("NULL Port info detected, returning");
		return -IGD_ERROR_INVAL;
	}

	/* Check for default case */
	if (!(pt_info->flags & IGD_MODE_VESA) &&
		(pt_info->width == 0) && (pt_info->height == 0)) {
		EMGD_DEBUG("Display Info width, height are zero, using default case");
		pt_info->width = CONFIG_DEFAULT_WIDTH;
		pt_info->height = CONFIG_DEFAULT_HEIGHT;
	}

	EMGD_DEBUG("Checking for exact mode match");
	exact_timing = match_resolution(display, timing_table, pt_info,
		MATCH_EXACT);
	/*
	 * At this point we have one of these cases:
	 *  1) Found an exact match, VGA, VESA or other.
	 *    -> Go check for FB centering and finish up.
	 *  2) Found nothing
	 *    -> Check for VGA/VESA mode to center.
	 *    -> Check common modes.
	 */
	if(exact_timing) {
		pipe_timing = exact_timing;
		user_timing = exact_timing;
		pipe_timing->extn_ptr = NULL;
	} else {
		/* No match found? Is it VGA? */
		if( (pt_info->flags & IGD_MODE_VESA) &&
			(pt_info->mode_number < 0x1D)    ){
			EMGD_DEBUG("Checking for exact match in VGA table");
			/* this only happens if it was a VGA mode number */
			pt_info->refresh = 0;
			vga_timing = match_resolution(display, vga_timing_table,
				pt_info, MATCH_EXACT);

			if(!vga_timing) {
				return -IGD_ERROR_INVAL;
			}

			vga_timing->extn_ptr = NULL;
			/* We got something sane that needs to be centered */
			user_timing = vga_timing;
			fill_pt(vga_timing,pt_info);

			/* continue at the bottom where we have
			 * pipe_timing = NULL, so we will look
			 * for centered timings for pt_info and
			 * use cmn_vga_timings to tell match_resolution
			 * to take into account special VGA mode
			 * centering regulations
			 */
		}
	}

	/* Find UPSCALING attr value*/
	pi_pd_find_attr_and_value(PORT_OWNER(display),
			PD_ATTR_ID_PANEL_FIT,
			0,/*no PD_FLAG for UPSCALING */
			NULL, /* dont need the attr ptr*/
			&upscale);
	/* this PI func will not modify value of upscale if attr does not exist */

	if(!pipe_timing){
		/* At this point, one of 2 things has happenned:
		 *      - we have a mode request that we could not match exactly.
		 *        and it WASNT a VESA_MODE number request.
		 *      - we have a request based on VESA_MODE number (maybe from
		 *        VBIOS IAL) and we could not get a exact match from the
		 *        port_timing_table, but we did get a match from the vga-
		 *        timing_table.
		 * In this case, there is one thing to do - MATCH_CENTER. Match
		 * resolution will handle it this way:
		 *      - if its VESA MODE number based, we only need to get
		 *        the best (tightest) match if its VGA OR DONT match
		 *        if its one of those magic timings
		 *      - Else, we need to get the best (tightest) match, AND
		 *        we need to center requested timings in that tightest fitting
		 *        timing. But wait! This could mean if the requested pt_info
		 *        is bigger than anything in the port timing table, we have
		 *        no choice but to fail.
		 */
		unsigned char match_type = MATCH_CENTER;

		EMGD_DEBUG("Checking for a safe centered match");
		if(vga_timing) {
			match_type |= MATCH_FOR_VGA;
		} else if(pt_info->flags & IGD_MODE_VESA) {
			/* if a vesa mode number was requested...
			 * and we are centering that mode, we
			 * need to get the common mode fb size
			 * in case we need it later for VBIOS
			 * which doesnt populate the FBInfo
			 */
			vesa_timing = match_resolution(display, crt_timing_table,
				pt_info, MATCH_EXACT);
		}

		if (upscale && vga_timing) {
			/* If port supports upscaling and match is called for VGA,
			 * then center vga mode resolution directly in the native mode
			 * instead of centering VGA in another resolution */
			pipe_timing = vga_timing;
		} else {
			pipe_timing = match_resolution(display, timing_table, pt_info,
				match_type);
			/* This can happen if there is a spurious pt_info from IAL */
			if (!pipe_timing) {
				return -IGD_ERROR_INVAL;
			}
			pipe_timing->extn_ptr = vga_timing;
			/* for the case of non VGA mode call,
			 * at this point, vga_timing is NULL
			 */
		}

		if(!vga_timing) {
			user_timing = pipe_timing;
		}
	}

	/*
	 * At this point pipe_timing is what we are going to program the
	 * pipe to roughly speaking. If there is a common timing then we
	 * want it centered in the pipe_timing.
	 *
	 * If the framebuffer is smaller than the timings then we need to
	 * generate a centered set of timings by copying the pipe timings
	 * and shifting them a bit.
	 *
	 * If fb width and height are zero just assume that we want it to
	 * match the timings and make up a pixel format. This is mostly because
	 * VGA/VESA modes will just be set by number. We don't know their size
	 * until we look up the number.
	 */
	if(fb_info) {
		/*
		 * fb_info is sometimes NULL when just testing something.
		 */
		if(!fb_info->pixel_format) {
			/* Ugly VGA modes, it doesn't matter */
			fb_info->pixel_format = IGD_PF_ARGB8_INDEXED;
		}
		if(!fb_info->width) {
			if(vga_timing) {
				fb_info->width = vga_timing->width;
				fb_info->height = vga_timing->height;
			} else {
				if(!vesa_timing){
					vesa_timing = pipe_timing;
					/* in case vesa_timing is false set it to
					 * pipe_timing so we dont need to check for
					 * validity later, when increasing fb size for
					 * VBIOS in clone mode (see 18 lines below)
					 */
				}
				fb_info->width = vesa_timing->width;
				fb_info->height = vesa_timing->height;
			}
		}

		/*
		 * VGA common timings are centered in pipe timings by hardware.
		 * Otherwise we need to adjust the timings when centering is
		 * needed.
		 */
		if (!vga_timing) {
			/*
			 * For VBIOS clone modes the FB should be the biggest mode
			 * if this is the second match we may need to update the fb
			 * data structure.
			 */
			if(fb_info->flags & IGD_VBIOS_FB) {
				if ((fb_info->width < vesa_timing->width) ||
					(fb_info->height < vesa_timing->height)) {
					fb_info->width = vesa_timing->width;
					fb_info->height = vesa_timing->height;
				}
			}


			/* Do centering if fb is smaller than timing except on TV */
			if ((fb_info->width < pipe_timing->width) ||
				(fb_info->height < pipe_timing->height)) {
				unsigned short temp_width = pipe_timing->width;
				unsigned short temp_height = pipe_timing->height;
				/* Normally, we should NOT be in here. All IALs only
				 * are supposed to request for timings that ARE surely
				 * supported by the HAL,... i.e. query the list of
				 * supported timings by the port first!
				 *
				 * The exception would be if the IAL is purposely
				 * asking for CENTERING!!! (pt_info's that were not
				 * part of the supported mode list). This could indicate an
				 * error or an explicit request for VESA centering!.
				 */

				/* let's use these 2 variables as flags... and do the
				 * actual "centering" of the timings later since we do
				 * also need to acomodate native timings as well
				 */
				/* NOTE: we could never be in here in fb_info was NULL */
				cntr_dff_w = (pipe_timing->width - fb_info->width) / 2;
				cntr_dff_h = (pipe_timing->height - fb_info->height) / 2;

				/* Dont forget to use a different storage sice we dont
				 * want to change the original (and to be used later)
				 * ports mode list timings
				 */
				OS_MEMCPY(&scaled_timing[(PIPE(display)->pipe_num)],
					pipe_timing,
					sizeof(igd_timing_info_t));

				pipe_timing = &scaled_timing[(PIPE(display)->pipe_num)];

				if(PORT_OWNER(display)->pd_type != PD_DISPLAY_TVOUT ) {
					/* TV display don't like changed pipe actives,
					 * Updating syncs work for TV centering */
					if (fb_info->width < temp_width) {
						pipe_timing->width = (unsigned short)fb_info->width;
						pipe_timing->hblank_start -= cntr_dff_w;
						pipe_timing->hblank_end -= cntr_dff_w;
					}

					if (fb_info->height < temp_height) {
						pipe_timing->height = (unsigned short)fb_info->height;
						pipe_timing->vblank_start -= cntr_dff_h;
						pipe_timing->vblank_end -= cntr_dff_h;
					}
				}

				if (fb_info->width < temp_width) {
					pipe_timing->hsync_start -= cntr_dff_w;
					pipe_timing->hsync_end -= cntr_dff_w;
				}

				if (fb_info->height < temp_height) {
					pipe_timing->vsync_start -= cntr_dff_h;
					pipe_timing->vsync_end -= cntr_dff_h;
				}
			}
		}
	}

	if(upscale) {
		/* Get the native timings */
		EMGD_DEBUG("Checking for Native LVDS match for scaling");
		native_timing = match_resolution(display, timing_table, pt_info,
			MATCH_NATIVE);
		if(native_timing && (native_timing != pipe_timing)) {
			native_timing->extn_ptr = pipe_timing;
			pipe_timing = native_timing;
		}
	}

	/*
	 * Match mode returns as follows:
	 * In case of VGA setmode:
	 * 1) We will end up with either:
	 *   magic->vga   ---   For displays supports native VGA
	 *      or
	 *   native->vga  ---   Upscaling displays
	 *      or
	 *   pipe->vga    ---   For other displays
	 *
	 * 2) In case of regular setmode:
	 *   pipe         ---   For regular displays
	 *      or
	 *   native->vesa ---   Upscaling displays
	 *
	 *   Note: 1) Here "pipe" can be munged if centering is required.
	 *         2) "vesa" is the requested mode, native is the native timing
	 *            of the display.
	 */

	/*
	 * Update Input Structures with values found
	 * Note: This might not be what is going to be programmed. It is what
	 * the user thinks they set. Scaling or centering could have altered
	 * that.
	 */
	fill_pt(user_timing, pt_info);
	*timing = pipe_timing;
	EMGD_DEBUG("Return");

	return 0;
}
#else

#define MATCH_NUMBER    0x001
#define MATCH_REFRESH   0x002
#define MATCH_WIDTH     0x004
#define MATCH_HEIGHT    0x008
#define MATCH_FLAGS     0x010
#define MATCH_GE_WIDTH  0x020
#define MATCH_GE_HEIGHT 0x040
#define MATCH_FOR_VGA   0x100

static int modes_match(igd_display_info_t *x,
	igd_timing_info_t *y,
	unsigned int flags)
{
	if((flags & MATCH_NUMBER) &&
		(x->mode_number != y->mode_number)) {
		return 0;
	}
	if((flags & MATCH_REFRESH) &&
		(x->refresh && (x->refresh != y->refresh))) {
		return 0;
	}
	if((flags & MATCH_WIDTH) &&
		(x->width != y->width)) {
		return 0;
	}
	if((flags & MATCH_HEIGHT) &&
		(x->height != y->height)) {
		return 0;
	}
	if((flags & MATCH_GE_WIDTH) &&
		(x->width > y->width)) {
		return 0;
	}
	if((flags & MATCH_GE_HEIGHT) &&
		(x->height > y->height)) {
		return 0;
	}
	if((flags & MATCH_FLAGS) &&
		(x->flags & (IGD_SCAN_INTERLACE|IGD_PIXEL_DOUBLE|IGD_LINE_DOUBLE)) !=
		(y->mode_info_flags &
			(IGD_SCAN_INTERLACE|IGD_PIXEL_DOUBLE|IGD_LINE_DOUBLE))) {
		return 0;
	}

	return 1;
}

typedef struct _vga_wh {
	short width;
	short height;
} vga_wh_t;

static vga_wh_t vga_size[] = {
	{640, 400}, {640, 400}, {640, 400}, {640, 400},
	{320, 400}, {320, 400}, {640, 400}, {720, 350},
	{640, 400}, {640, 400}, {640, 350}, {640, 350},
	{640, 350}, {640, 350}, {640, 350}, {640, 350},
	{640, 400}, {720, 400}, {720, 400}, {640, 480},
	{640, 480}, {640, 400}
};

/*
 * For VGA modes we do not actually have a mode table. Just use a
 * temporary slot (one per-pipe) and put the VGA mode number in it.
 */
static igd_timing_info_t vga_timing[] = {
	{
		720, 400, 70,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0x00,
		IGD_MODE_VESA | IGD_MODE_SUPPORTED,
		0, 0, NULL, NULL
	},
	{
		720, 400, 70,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0x00,
		IGD_MODE_VESA | IGD_MODE_SUPPORTED,
		0, 0, NULL, NULL
	}
};

/*
 * The magic mode, when found in a port's mode list, means that the port
 * can do native VGA output. A mode with a wxh of 720x480 and refresh of
 * 70 is the magic mode.
 */
static igd_display_info_t magic_timing = {
	720, 400, 70,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0x00,
	IGD_MODE_VESA | PD_MODE_SUPPORTED,
	0, 0, NULL, NULL
};

/*!
 * Match Resolution searches the provided timing table for a mode that
 * matches the requested parameters using the provided criteria.
 *
 * @param display The display on which the mode will be displayed.
 * @param timing_table The list of modes to search through.
 * @param pt_info The timing set used as reference for the match
 * @param criteria The criteria that must match (bitfield). Options are
 *  MATCH_NUMBER: Match the VGA/VESA mode number
 *  MATCH_REFRESH: Match the refresh rate
 *  MATCH_WIDTH: Match the width
 *  MATCH_HEIGHT: Match the height
 *  MATCH_FLAGS: Match relavent interlace, pixel double, etc flags.
 *  MATCH_GE_WIDTH: Match width greater or equal to requested (used with
 *    best_match = 1)
 *  MATCH_GE_HEIGHT: Match height greater or equal to requested (used with
 *    best_match = 1)
 *  MATCH_FOR_VGA: Match is used for a VGA container which may need to take
 *    into accoutn 9th pixel dropping in the hardware.
 * @param best_match When set to 0 the first mode found matching the criteria
 * 	will be returned. Null will be returned if no match is found.
 * 	When set to 1 the best matching mode is found after searching all modes.
 * 	if no mode is found matching the criteria, the last mode in the table
 * 	is used (should be the largest mode as mode tables are sorted)
 *
 * @return igd_timing_info_t
 */
static igd_timing_info_t *match_resolution(
		igd_display_context_t *display,
		igd_timing_info_t *timing_table,
		igd_display_info_t *pt_info,
		unsigned int criteria,
		int best_match)
{
	igd_timing_info_t *timing;
	igd_timing_info_t *match;
	igd_timing_info_t *last_mode = NULL;
	igd_display_port_t *port;
	unsigned short save_width;

	EMGD_DEBUG("Enter match_resolution");

	EMGD_DEBUG("Width=%d, height=%d, refresh=%d mode_number=0x%x",
		pt_info->width, pt_info->height, pt_info->refresh,
		pt_info->mode_number);

	timing = timing_table;
	match = NULL;
	port = PORT_OWNER(display);
	save_width = pt_info->width;

	/*
	 * Hardware has a mode where it can drop every 9th pixel to fit
	 * a 720 width VGA mode into a 640 width container. Modify the
	 * reference mode to reflect 640 width and put it back at the end.
	 */
	if(criteria & MATCH_FOR_VGA) {
		if((pt_info->width == 720) &&
			(port->port_features & IGD_VGA_COMPRESS)) {
			pt_info->width = 640;
		}
	}

	while (timing->width != IGD_TIMING_TABLE_END) {
		if(!MODE_IS_SUPPORTED(timing)) {
			timing++;
			continue;
		}
		if(modes_match(pt_info, timing, criteria)) {
			/*
			 * If we are looking for the first match we are done.
			 */
			if(!best_match) {
				match = timing;
				break;
			}
			/*
			 * Looking for best match. Compare the new match to the last one
			 * to see if it is better.
			 */
			if(match) {
				/* Check for tighter fit */
				if((timing->width < match->width) ||
					(timing->height < match->height)) {
					match = timing;
				}
			} else {
				match = timing;
			}
		}
		last_mode = timing;
		timing++;
	}

	pt_info->width = save_width;
	if(!match && best_match) {
		/*
		 * Take the last one in the table for container modes. This insures
		 * that we never fail a container mode.
		 */
		match = last_mode;
	}

	if (match) {
		EMGD_DEBUG("Returning with a match");
		EMGD_DEBUG("Width=%d, height=%d, refresh=%d mode_number=0x%x",
			match->width, match->height, match->refresh, match->mode_number);
	} else {
		EMGD_DEBUG("Returning with NO match");
	}

	return match;
} /* end match_resolution */


void update_fb_size(igd_timing_info_t *match,
	igd_framebuffer_info_t *fb_info)
{

	/*
	 * fb_info is sometimes NULL when just testing something.
	 */
	if(fb_info) {
		/*
		 * If The FB data comes in as zero we populate it with the matched
		 * size. Also if the FB is coming from vBIOS we increase the size
		 * to fit the match.
		 */
		if((fb_info->width == 0) || (fb_info->flags & IGD_VBIOS_FB)) {

			if((fb_info->width < match->width) ||
				(fb_info->height < match->height)) {
				fb_info->width = match->width;
				fb_info->height = match->height;
			}
		}
	}
}

/*!
 * Push a Match on to the stack which is represented by a link list.
 * Only push the mode if it is different than the head of the list.
 *
 * @param match
 * @param timing_stack
 *
 * @return void
 */
void push_match(igd_timing_info_t *match,
	igd_timing_info_t **timing_stack)
{

	if(match == *timing_stack) {
		return;
	}
	match->extn_ptr = *timing_stack;
	*timing_stack = match;
	return;
}


static igd_timing_info_t scaled_timing[IGD_MAX_PIPES];

/*!
 * This function determines if the input timings need to be modified
 * to accomodate the provided framebuffer. If so it converts the timing set
 * to a modified set that centers the provided framebuffer image.
 * The provided timings are not modified and the returned set
 * may be used until the next mode set. The returned timings do
 * not need to be freed.
 *
 * @param display A pointer to the display used for these timings.
 * 	This parameter is used to determine which pipe the timings
 * 	will be used with. There is one set of static timings for
 * 	each pipe.
 * @param timing The input timings which are used as reference to
 * 	generate the new timings. Only the width/height and blank/syncs
 * 	will be altered. The pixel clock and total sizes will not be changed.
 * @param fb_info The framebuffer information to center within the
 * 	provided timings. (May be NULL)
 *
 * @return Modified timing set
 */
igd_timing_info_t *get_centered_timings(igd_display_context_t *display,
	igd_timing_info_t *timing,
	igd_framebuffer_info_t *fb_info)
{
	short cntr_dff_w = 0;
	short cntr_dff_h = 0;

	if(!fb_info) {
		return timing;
	}

	/*
	 * If we end up making a munged centered timings we need to use
	 * a copy and not the originals.
	 *
	 */
	OS_MEMCPY(&scaled_timing[(PIPE(display)->pipe_num)],
		timing, sizeof(igd_timing_info_t));

	if(PORT_OWNER(display)->pd_type != PD_DISPLAY_TVOUT) {
		/*
		 * TV display don't like changed pipe actives,
		 * Updating syncs work for TV centering
		 */
		if (fb_info->width < timing->width) {
			timing = &scaled_timing[(PIPE(display)->pipe_num)];
			cntr_dff_w = (timing->width - fb_info->width) / 2;
			timing->width = (unsigned short)fb_info->width;
			timing->hblank_start -= cntr_dff_w;
			timing->hblank_end -= cntr_dff_w;
		}
		if (fb_info->height < timing->height) {
			timing = &scaled_timing[(PIPE(display)->pipe_num)];
			cntr_dff_h = (timing->height - fb_info->height) / 2;
			timing->height = (unsigned short)fb_info->height;
			timing->vblank_start -= cntr_dff_h;
			timing->vblank_end -= cntr_dff_h;
		}
	}

	timing->hsync_start -= cntr_dff_w;
	timing->hsync_end -= cntr_dff_w;
	timing->vsync_start -= cntr_dff_h;
	timing->vsync_end -= cntr_dff_h;

	return timing;
}


/*!
 * Match the fb and pt structures to a Mode Structure from the table.
 * When a mode is found update the input structures to reflect the
 * values found.
 *
 *  The match mode function will compile all the information needed by
 * the HAL to put the hardware into a mode. This may be a regular
 * full-screen mode, a centered mode, a panned mode, a scaled mode.
 * Below is a table of the mode that will be used and the cases in which
 * it will occurr.
 *
 * Native
 *  The requested mode is matched in the mode table in terms of width
 *  height, refresh, and interlace flags. There is no fp_native_dtd or
 *  the matching mode IS the fp_native_dtd.
 * Centered
 *  1) The requested mode is matched in the mode table but the framebuffer
 *     is a smaller size and needs to be centered.
 *  2) The requested mode is NOT matched in the mode table and the
 *     framebuffer matches the requested mode. A container mode is used
 *     with the original smaller framebuffer.
 * Scaled
 *  The requested mode is matched in the mode table and a fp_native_dtd
 *  is present for the port. The matched mode does not match the native
 *  and is therefore scaled to fit.
 * Panned/Cropped
 *  The requested mode is matched in the mode table but the framebuffer
 *  is larger (in at least one demension) that the mode. The mode is
 *  set with a cropped FB that may then be panned by the IAL.
 * Render Scaled
 *  The requested mode may or may not be matched in the mode table. The
 *  framebuffer data is empty. The HAL will match the mode or find a
 *  container or find the largest mode and set the framebuffer to the
 *  same size. The IAL will scale the actual framebuffer to the returned
 *  framebuffer.
 * Native VGA
 *  A VGA mode is requested (by number) and the magic VGA mode is matched
 *  in the mode table. FB contents are output only.
 * Centered VGA
 *  A VGA mode is requested (by number) and the magic VGA mode is NOT
 *  matched. A container mode is found or the largest mode is used. The
 *  VGA is hardware centered in the container mode. FB contents are
 *  output only.
 * Scaled VGA
 *  A VGA mode is requested (by number) and the magic VGA mode is NOT
 *  matched. A fp_native_dtd is found. The VGA image will be scaled to
 *  the native timings.
 *
 *
 * The timings returned may be part of a linked list of timings which
 * are used by the HAL to perform centering and scaling operations. One
 * of the following will result.
 *
 * Requested Mode
 *   The requested mode matched and is returned with no linked modes.
 * Munged Requested Mode
 *   The requested mode was larger than the requested framebuffer. The
 *   timings were then modified to center the framebuffer.
 * Native Mode -> Requested Mode
 *   The display is fixed resolution and any mode must be scaled to that
 *   resolution by the PD. The native mode is returned and linked to the
 *   requested mode which will be scaled to fit the native mode. The
 *   pt_info will be populated with the requested mode giving the caller
 *   the impression that the mode was set natively.
 * Native Mode -> Munged Requested Mode
 *   The display is fixed resolution and any mode must be scaled to that
 *   resolution by the PD. In addition the requested framebuffer is smaller
 *   then the requested mode. The native mode is returned and linked to the
 *   requested mode which has been modifed to center the framebuffer.
 *   The result will be the requested framebuffer scaled to fit the native
 *   mode. The pt_info will be populated with the requested mode giving the
 *   caller the impression that the mode was set natively.
 * VGA Mode -> Magic VGA Mode
 *   The requested mode was a legacy VGA mode and the display supports
 *   native VGA output. The 720x400 "magic" mode is returned to indicate
 *   to the HAL that native VGA should be used. The magic mode is linked
 *   to the requested VGA mode.
 * Container Mode -> VGA Mode
 *   The requested mode was a legacy VGA mode but the display does not
 *   support native VGA output. A larger container mode was found and
 *   returned. The hardware will center the VGA mode within the container
 *   for output. The pt_info will contain the VGA mode giving the caller
 *   the impression that the VGA mode was set natively.
 * Native Mode -> VGA Mode
 *   The requested mode was a legacy VGA mode but the and the PD is
 *   set for scaling to a fixed resolution display. The Native mode is
 *   returned linked to the VGA mode. The result is the VGA mode scaled to
 *   fit the native screen.
 *
 *   @note: There is a tradeoff to this behavior. If a VGA mode is set and
 *   scaled to full-screen there is no guarentee that ALL VGA modes are
 *   possible in that configuration. For instance, in DOS if a mode 3 is
 *   set (720x400) and scaled full-screen, an application cannot directly
 *   program the VGA registers (as many games do) to get an acceptable
 *   output at 640x480. This tradeoff is acknoledged and accepted.
 *
 *
 * Assumtions:
 *  1) fp_native_dtd is set in all instances where the output is a fixed
 *   resolution. (and only those instances)
 *  2) PDs timing table contains all modes that can be output or all
 *   modes that can be scaled to the native output.
 *  3) Scaling PDs can accept a VGA mode to be scaled to the native.
 *
 * @param display
 * @param timing_table
 * @param fb_info
 * @param pt_info
 * @param timing
 *
 * @return 0
 */
int match_mode (
	igd_display_context_t *display,
	igd_timing_info_t *timing_table,
	igd_framebuffer_info_t *fb_info,
	igd_display_info_t *pt_info,
	igd_timing_info_t **timing)
{
	igd_timing_info_t *match = NULL;
	igd_timing_info_t *timing_stack = NULL;

	EMGD_DEBUG("Enter Match Mode");

	EMGD_ASSERT(pt_info, "Null PT Info", -IGD_ERROR_INVAL);
	EMGD_ASSERT((pt_info->flags & IGD_MODE_VESA) ||
		(pt_info->width && pt_info->height),
		"Width and Height are Zero", -IGD_ERROR_INVAL);

	if(MODE_IS_VGA(pt_info)) {
		/*
		 * The requested mode is a legacy VGA mode
		 */
		EMGD_DEBUG("Matching a VGA mode");

		/*
		 * All VGA modes are possible. Just use a temporary timing
		 * block and put the VGA mode number in it. We don't actually
		 * use any timing information for VGA modes.
		 */
		match = &vga_timing[(PIPE(display)->pipe_num)];
		match->mode_number = pt_info->mode_number;
		match->width = vga_size[pt_info->mode_number].width;
		match->height = vga_size[pt_info->mode_number].height;

		push_match(match, &timing_stack);
		update_fb_size(match, fb_info);

		/* The caller should think we set the VGA mode directly */
		fill_pt(match, pt_info);

		match = match_resolution(display, timing_table, &magic_timing,
			MATCH_WIDTH|MATCH_HEIGHT, 0);
		if(match) {
			/*
			 * Magic VGA mode was found. This indicates the display can
			 * do full native VGA timings. We will send the mode out
			 * natively.
			 */
			EMGD_DEBUG("Native VGA output");
			push_match(match, &timing_stack);

			*timing = timing_stack;
			return 0;
		}

		/*
		 * This display cannot do native VGA. We need to center or
		 * scale the VGA to fit.
		 */
		if(PORT_OWNER(display)->fp_native_dtd) {
			/*
			 * Native mode in the port indicates the PD can scale.
			 * Push the native mode on the list to get VGA scaled
			 * to native.
			 */
			EMGD_DEBUG("VGA Scaled to Native");
			push_match(PORT_OWNER(display)->fp_native_dtd, &timing_stack);
		} else {
			/*
			 * No native mode means the PD is not scaling. Output
			 * VGA centered in a safe container mode.
			 */
			EMGD_DEBUG("VGA centered in Container");
			match = match_resolution(display, timing_table, pt_info,
				MATCH_FOR_VGA|MATCH_GE_WIDTH|MATCH_GE_HEIGHT|MATCH_FLAGS, 1);
			EMGD_ASSERT(match, "Match Container Failed", -IGD_ERROR_INVAL);

			push_match(match, &timing_stack);
		}

		*timing = timing_stack;
		return 0;

	}

	/*
	 * Regular NON-VGA mode
	 */
	EMGD_DEBUG("Matching a regular NON-VGA mode");

	match = match_resolution(display, timing_table, pt_info,
		(pt_info->flags & IGD_MODE_VESA)?
		MATCH_NUMBER|MATCH_REFRESH:
		MATCH_WIDTH|MATCH_HEIGHT|MATCH_REFRESH|MATCH_FLAGS, 0);
	if(match) {
		/*
		 * Exact match for requested mode was found in the mode table.
		 * This mode will be output directly.
		 */
		EMGD_DEBUG("Input Mode Matched exactly");
	} else {
		/*
		 * Matching mode was not found. This happens for several
		 * reasons:
		 *
		 * 1) We are setting the clone mode but the IAL does not
		 *   know the clone width/height. We are just getting the
		 *   best guess. In this case fb info is populated.
		 * 2) IAL is doing Render Scaling but doesn't know what mode
		 *   to scale to. We are getting the best fit. In this case
		 *   fb info is all zeros.
		 * 3) IAL is just setting random modes and wants something
		 *   close. Fb info may or may not be populated.
		 */
		EMGD_DEBUG("Input Mode NOT matched, container used");
		match = match_resolution(display, timing_table, pt_info,
			MATCH_GE_WIDTH|MATCH_GE_HEIGHT|MATCH_FLAGS, 1);
		EMGD_ASSERT(match, "Match Container Failed", -IGD_ERROR_INVAL);
	}
	update_fb_size(match, fb_info);
	match = get_centered_timings(display, match, fb_info);
	push_match(match, &timing_stack);

	fill_pt(match, pt_info);

	if(PORT_OWNER(display)->fp_native_dtd) {
		/*
		 * Native DTD indicates that the PD is scaling. Hook the
		 * match mode up to the native to get scaling.
		 */
		EMGD_DEBUG("Input Mode Scaled to Native");
		push_match(PORT_OWNER(display)->fp_native_dtd, &timing_stack);
	}


	*timing = timing_stack;
	return 0;
}


#endif


/*----------------------------------------------------------------------------
 * File Revision History
 * $Id: match.c,v 1.10 2011/03/02 22:47:05 astead Exp $
 * $Source: /nfs/fm/proj/eia/cvsroot/koheo/linux/egd_drm/emgd/display/mode/cmn/match.c,v $
 *----------------------------------------------------------------------------
 */
