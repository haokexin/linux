/*
* arch/arm/plat-spear/clcd.c
*
* Copyright (C) 2009 ST Microelectronics
* Shiraz Hashim<shiraz.hashim@st.com>
* Ashish Priyadarshi<ashish.priyadarshi@st.com>
*
* This file is licensed under the terms of the GNU General Public
* License version 2. This program is licensed "as is" without any
* warranty of any kind, whether express or implied.
*/

#include <linux/dma-mapping.h>
#include <linux/amba/bus.h>
#include <linux/amba/clcd.h>

/* This is enough for the size of 800x480 (1.5 MB) */
#define FRAMESIZE 0x00180000

#ifdef CONFIG_FB_ARMCLCD_SHARP_LQ043T1DG01
static struct clcd_panel sharp_LQ043T1DG01_in = {
	.mode = {
		.name = "Sharp LQ043T1DG01",
		.refresh = 0,
		.xres = 480,
		.yres = 272,
		.pixclock = 48000,
		.left_margin = 2,
		.right_margin = 2,
		.upper_margin = 2,
		.lower_margin = 2,
		.hsync_len = 41,
		.vsync_len = 11,
		.sync = 0,/* FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT */
		.vmode = FB_VMODE_NONINTERLACED,
	},
	.width = -1,
	.height = -1,
	.tim2 = TIM2_IOE | TIM2_CLKSEL | 3,
	.cntl = CNTL_LCDTFT | CNTL_BGR,
	.bpp = 32,
};
#endif

#ifdef CONFIG_FB_ARMCLCD_SAMSUNG_LMS700
static struct clcd_panel samsung_LMS700_in = {
	.mode = {
		.name = "Samsung LMS700",
		.refresh = 0,
		.xres = 800,
		.yres = 480,
		.pixclock = 48000,
		.left_margin = 16,
		.right_margin = 8,
		.upper_margin = 6,
		.lower_margin = 5,
		.hsync_len = 3,
		.vsync_len = 2,
		.sync = 0,
		.vmode = FB_VMODE_NONINTERLACED,
	},
	.width = -1,
	.height = -1,
	.tim2 = TIM2_CLKSEL,
	.cntl = CNTL_LCDTFT | CNTL_BGR,
	.bpp = 32,
};
#endif

#ifdef CONFIG_FB_ARMCLCD_ET057010_640x480
static struct clcd_panel et057010_640x480_in = {
	.mode = {
		.name = "ET057010 640x480",
		.refresh = 0,
		.xres = 640,
		.yres = 480,
		.pixclock = 48000,
		.left_margin = 144,
		.right_margin = 16,
		.upper_margin = 33,
		.lower_margin = 10,
		.hsync_len = 30,
		.vsync_len = 3,
		.sync = 0,
		.vmode = FB_VMODE_NONINTERLACED,
	},
	.width = -1,
	.height = -1,
	.tim2 = TIM2_CLKSEL | TIM2_IPC,	/* This display requires 24MHz */
	.cntl = CNTL_LCDTFT | CNTL_BGR,
	.bpp = 32,
};
#endif

static int clcd_setup(struct clcd_fb *fb)
{
	dma_addr_t dma;

	/* Detect which LCD panel is connected */
#ifdef CONFIG_FB_ARMCLCD_SHARP_LQ043T1DG01
	fb->panel = &sharp_LQ043T1DG01_in;
#endif
#ifdef CONFIG_FB_ARMCLCD_SAMSUNG_LMS700
	fb->panel = &samsung_LMS700_in;
#endif
#ifdef CONFIG_FB_ARMCLCD_ET057010_640x480
	fb->panel = &et057010_640x480_in;
#endif
	fb->fb.screen_base = dma_alloc_writecombine(&fb->dev->dev, FRAMESIZE,
			&dma, GFP_KERNEL);
	if (!fb->fb.screen_base) {
		printk(KERN_ERR "CLCD: unable to map framebuffer\n");
		return -ENOMEM;
	}
	fb->fb.fix.smem_start = dma;
	fb->fb.fix.smem_len = FRAMESIZE;

	return 0;
}

static int clcd_check(struct clcd_fb *fb, struct fb_var_screeninfo *var)
{
	var->xres_virtual = var->xres = (var->xres + 15) & ~15;
	var->yres_virtual = var->yres = (var->yres + 1) & ~1;

#define CHECK(e, l, h) (var->e < l || var->e > h)
	if (CHECK(right_margin, (1), 256) ||	/* back porch */
			CHECK(left_margin, (1), 256) ||	/* front porch */
			CHECK(hsync_len, (1), 256) ||
			var->xres > 4096 ||
			var->lower_margin > 255 ||	/* back porch */
			var->upper_margin > 255 ||	/* front porch */
			var->vsync_len > 32 ||
			var->yres > 1024)
		return -EINVAL;
#undef CHECK

	/* single panel mode: PCD = max(PCD, 1) */
	/* dual panel mode: PCD = max(PCD, 5) */

	/*
	 * You can't change the grayscale setting, and
	 * we can only do non-interlaced video.
	 */
	if (var->grayscale != fb->fb.var.grayscale ||
			(var->vmode & FB_VMODE_MASK) != FB_VMODE_NONINTERLACED)
		return -EINVAL;

#define CHECK(e) (var->e != fb->fb.var.e)
	if (fb->panel->fixedtimings &&
			(CHECK(xres)		||
			 CHECK(yres)		||
			 CHECK(bits_per_pixel)	||
			 CHECK(pixclock)	||
			 CHECK(left_margin)	||
			 CHECK(right_margin)	||
			 CHECK(upper_margin)	||
			 CHECK(lower_margin)	||
			 CHECK(hsync_len)	||
			 CHECK(vsync_len)	||
			 CHECK(sync)))
		return -EINVAL;
#undef CHECK

	var->nonstd = 0;
	var->accel_flags = 0;

	return 0;
}

static void clcd_remove(struct clcd_fb *fb)
{
	dma_free_writecombine(&fb->dev->dev, fb->fb.fix.smem_len,
			fb->fb.screen_base, fb->fb.fix.smem_start);
}

static int clcd_mmap(struct clcd_fb *fb, struct vm_area_struct *vma)
{
	return dma_mmap_writecombine(&fb->dev->dev, vma,
			fb->fb.screen_base,
			fb->fb.fix.smem_start,
			fb->fb.fix.smem_len);
}

struct clcd_board clcd_plat_data = {
	.name = "spear-clcd",
	.check = clcd_check,
	.decode = clcdfb_decode,
	.setup = clcd_setup,
	.mmap = clcd_mmap,
	.remove = clcd_remove,
};
