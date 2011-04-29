/* -*- pse-c -*-
 *-----------------------------------------------------------------------------
 * Filename: user_config.c
 * $Revision: 1.19 $
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
 *  A file that contains the initial display configuration information of the
 *  EMGD kernel module.  A user can edit this file in order to affect the way
 *  that the kernel initially configures the displays.  This file is compiled
 *  into the EMGD kernel module.
 *-----------------------------------------------------------------------------
 */
#include "user_config.h"


/*
 * One array of attribute pairs may exist for each configured port.  See the
 * "include/igd_pd.h" file for attributes.
 */
static igd_param_attr_t attrs_config1_port2[] = {
	{PD_ATTR_ID_TVFORMAT, PD_TV_STD_NTSC_M}
};

static igd_param_attr_t attrs_config1_port4[] = {
	{0x46, 100},	/* PWM Intensity */
	{0x47, 20300},	/* Inverter Frequency */
	{0x1b, 0},		/* Boolean - channel type dual/single channel*/
	{0x1a, 18},		/* Panel depth 24/18 */
	{0x3c, 1},		/* Fixed timing */
};



/*
 * One array of igd_display_info_t structures should exist for each port that
 * needs to provide a DTD list.  Each igd_display_info_t contains the DTD
 * information for a given resolution/refresh-rate.  This is especially needed
 * for analog/VGA ports.
 */
static igd_display_info_t dtd_config1_port4_dtdlist[] = {
	{
		1024,		/* Width */
		768,		/* Height */
		60,			/* Refresh Rate */
		65000,		/* Dot Clock (in KHz) */
		1343,		/* Horizontal Total (horizontal synch end) */
		1023,		/* Horizontal Blank Start (h_active-1) */
		1343,		/* Horizontal Blank End (start + h_blank) */
		1047,		/* Horizontal Sync Start (h_active+h_synch-1) */
		1183,		/* Horizontal Sync End (start + h_syncp) */
		805,		/* Vertical Total (Vertical synch end) */
		767,		/* Vertical Blank Start (v_active-1) */
		805,		/* Vertical Blank End (start + v_blank) */
		770,		/* Vertical Sync Start (v_active+v_synch-1) */
		776,		/* Vertical Sync End (start + v_synchp) */
		0,			/* Mode Number */
		0x20000,	/* Flags */
		0,			/* X Offset */
		0,			/* Y Offset */
		NULL,		/* pd extension pointer */
		0,0		/* mode extension pointer */
	},

#if 0
	{
		1366,		/* Width */
		768,		/* Height */
		60,			/* Refresh Rate */
		72300,		/* Dot Clock (in KHz) */
		1525,		/* Horizontal Total (horizontal synch end) */
		1365,		/* Horizontal Blank Start (h_active-1) */
		1525,		/* Horizontal Blank End (start + h_blank) */
		1413,		/* Horizontal Sync Start (h_active+h_synch-1) */
		1445,		/* Horizontal Sync End (start + h_syncp) */
		789,		/* Vertical Total (Vertical synch end) */
		767,		/* Vertical Blank Start (v_active-1) */
		789,		/* Vertical Blank End (start + v_blank) */
		770,		/* Vertical Sync Start (v_active+v_synch-1) */
		775,		/* Vertical Sync End (start + v_synchp) */
		0,			/* Mode Number */
		0x20000,	/* Flags */
		0,			/* X Offset */
		0,			/* Y Offset */
		NULL,		/* pd extension pointer */
		0,0		/* mode extension pointer */
	},
	{
		1280,		/* Width */
		800,		/* Height */
		60,			/* Refresh Rate */
		68940,		/* Dot Clock (in KHz) */
		1407,		/* Horizontal Total (horizontal synch end) */
		1279,		/* Horizontal Blank Start (h_active-1) */
		1407,		/* Horizontal Blank End (start + h_blank) */
		1295,		/* Horizontal Sync Start (h_active+h_synch-1) */
		1343,		/* Horizontal Sync End (start + h_syncp) */
		815,		/* Vertical Total (Vertical synch end) */
		799,		/* Vertical Blank Start (v_active-1) */
		815,		/* Vertical Blank End (start + v_blank) */
		800,		/* Vertical Sync Start (v_active+v_synch-1) */
		803,		/* Vertical Sync End (start + v_synchp) */
		0,			/* Mode Number */
		0x20000,	/* Flags */
		0,			/* X Offset */
		0,			/* Y Offset */
		NULL,		/* pd extension pointer */
		NULL,		/* mode extension pointer */
	},
#endif
};

static emgd_drm_splash_screen_t splash_screen_data = {
	0,			/* bg_color */
	0,			/* x */
	0,			/* y */
	0,			/* width */
	0,			/* height */
};

static emgd_drm_splash_video_t splash_video_data = {
	0,			/* offset */
	0,			/* pixel_format */
	0,			/* src_width */
	0,			/* src_height */
	0,			/* src_pitch */
	0,			/* dst_x */
	0,			/* dst_y */
	0,			/* dst_width */
	0,			/* dst_height */
};

/*
 * The igd_param_t structure contains many configuration values used by the
 * EMGD kernel module.
 */
igd_param_t config_params_config1 = {
	1*256*1024,	/* Page request */
	0,			/* Max frame buffer size (0 = no limit) */
	1,			/* Preserve registers (should be 1, so VT switches work and so
				 * that the console will be restored after X server exits).
				 */
	0x6,		/* Display flags (bitfield, where:
				 * - 0x2 = DISPLAY_MULTI_DVO
				 * - 0x4 = Detect the display(s)
				 * - 0x8 = DISPLAY_FB_BLEND_OVL
				 */
	{ 4, 2, 0, 0, 0 },	/* Display port order (corresponds to the "portorder"
						 * module parameter, which is a comma-separate list)
						 */
	{			/* Display Params: */
	{				/* Port: */
		4,				/* Display port number (0 if not configured) */
		0x180,			/* Parameters present (bitfield, where:
						 * - 0x001 = DDC GPIO
						 * - 0x002 = DDC SPEED
						 * - 0x004 = DDC DAB
						 * - 0x008 = I2C GPIO
						 * - 0x010 = I2C SPEED
						 * - 0x020 = DAB
						 * - 0x040 = FP INFO
						 * - 0x080 = DTD LIST
						 * - 0x100 = ATTR LIST
						 */
		0x1,			/* EDID flag */
		0x3,			/* Flags when EDID is available (bitfield, where:
						 * - 0x1 = Use built-in standard timings
						 * - 0x2 = Use EDID block and filter modes
						 * - 0x4 = Use user-provided DTDs
						 */
		0x5,			/* Flags when EDID is not available (bitfield, where:
						 * - 0x1 = Use built-in standard timings
						 * - 0x2 = Use EDID block and filter modes
						 * - 0x4 = Use user-provided DTDs
						 */
		0,				/* DDC GPIO pins */
		0,				/* DDC speed */
		0,				/* DDC DAB */
		0,				/* I2C GPIO pins */
		0,				/* I2C speed */
		0,				/* I2C DAB */
		{				/* Flat Panel Info: */
			0,				/* Flat Panel width */
			0,				/* Flat Panel height */
			0,				/* Flat Panel power method */
			0,				/* VDD active & DVO clock/data active */
			0,				/* DVO clock/data active & backlight enable */
			0,				/* backlight disable & DVO clock/data inactive */
			0,				/* DVO clock/data inactive & VDD inactive */
			0				/* VDD inactive & VDD active */
		},
		{				/* DTD Info */
			sizeof(dtd_config1_port4_dtdlist)/sizeof(igd_display_info_t),
			/* number */
			dtd_config1_port4_dtdlist/* DTD name */
		},
		{				/* Attribute Info */
			sizeof(attrs_config1_port4)/sizeof(igd_param_attr_t), /* number */
			attrs_config1_port4	/* Attr name */
		}
	},
	{				/* Port: */
		2,				/* Display port number (0 if not configured) */
		0x140,			/* Parameters present (see above) */
		0x1,			/* EDID flag */
		0x3,			/* Flags when EDID is available (see above) */
		0x1,			/* Flags when EDID is not available (see above) */
		0,				/* DDC GPIO pins */
		0,				/* DDC speed */
		0,				/* DDC DAB */
		0,				/* I2C GPIO pins */
		0,				/* I2C speed */
		0,				/* I2C DAB */
		{				/* Flat Panel Info: */
			0,				/* Flat Panel width */
			0,				/* Flat Panel height */
			1,				/* Flat Panel power method */
			60,				/* VDD active & DVO clock/data active */
			200,			/* DVO clock/data active & backlight enable */
			200,			/* backlight disable & DVO clock/data inactive */
			50,				/* DVO clock/data inactive & VDD inactive */
			400				/* VDD inactive & VDD active */
		},
		{				/* DTD Info */
			0,				/* number */
			NULL,			/* DTD name */
		},
		{				/* Attribute Info */
			sizeof(attrs_config1_port2)/sizeof(igd_param_attr_t), /* number */
			attrs_config1_port2	/* Attr name */
		}
	},
	{				/* Port: */
		0,				/* Display port number (0 if not configured) */
		0,
		0, 0, 0, 0, 0, 0, 0, 0, 0,
		{ 0, 0, 0, 0, 0, 0, 0, 0 },
		{ 0, NULL },
		{ 0, NULL }
	},
	{				/* Port: */
		0,				/* Display port number (0 if not configured) */
		0,
		0, 0, 0, 0, 0, 0, 0, 0, 0,
		{ 0, 0, 0, 0, 0, 0, 0, 0 },
		{ 0, NULL },
		{ 0, NULL }
	},
	{				/* Port: */
		0,				/* Display port number (0 if not configured) */
		0,
		0, 0, 0, 0, 0, 0, 0, 0, 0,
		{ 0, 0, 0, 0, 0, 0, 0, 0 },
		{ 0, NULL },
		{ 0, NULL }
	},
	},
	0,			/* 24-bit RGB color that framebuffer is cleared to */
	0,			/* Quickboot (1 = enabled) */
	0,			/* Quickboot seamless (1 = enabled) */
	0,			/* Quickboot video input (1 = enabled) */
	0			/* Polling (1 = override interrupt support and use polling) */
};


igd_param_t *config_params = {&config_params_config1};


/*
 * The emgd_drm_config_t structure is the main configuration structure
 * for the EMGD kernel module.
 */
emgd_drm_config_t config_drm = {
	0,	/* Whether to initialize the display at EMGD module startup time
		 * (corresponds to the "init" module parameter)
		 */
	1,	/* The display configuration to use if initializing the display
		 * (corresponds to the "init" module parameter), where:
		 * - 1 = Single port/display
		 * - 2 = Cloned port/display (e.g. LVDS + CRT with different timings)
		 * - 4 = Twin ports/displays (e.g. LVDS + CRT with same timings)
		 *       Note: Twin is NOT CURRENTLY SUPPORTED
		 * - 8 = Extended displays (e.g. LVDS + CRT displaying different images)
		 */
	1024,/* Display width to use if initializing the display
		 * (corresponds to the "width" module parameter)
		 */
	768,/* Display height to use if initializing the display
		 * (corresponds to the "height" module parameter)
		 */
	60,	/* Display refresh rate to use if initializing the display
		 * (corresponds to the "refresh" module parameter)
		 */
	0,	/* ovl_brightness */
	0,	/* ovl_contrast */
	0,	/* ovl_saturation */
	0,	/* ovl_gamma_red */
	0,	/* ovl_gamma_green */
	0,	/* ovl_gamma_blue */
	&splash_screen_data,
	&splash_video_data,
	&config_params	/* driver parameters from above */
};
