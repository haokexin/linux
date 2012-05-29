/*
 * linux/drivers/video/db9000fb.c
 * -- Digital Blocks DB9000 LCD Controller Frame Buffer Device
 * Copyright (C) 2010 Digital Blocks, Inc.
 *
 * Based on pxafb.c and amba-clcd.c which is:
 * Copyright (C) 1999 Eric A. Thomas.
 * Copyright (C) 2004 Jean-Frederic Clere.
 * Copyright (C) 2004 Ian Campbell.
 * Copyright (C) 2004 Jeff Lackey.
 * Based on sa1100fb.c Copyright (C) 1999 Eric A. Thomas
 * which in turn is
 * Based on acornfb.c Copyright (C) Russell King.
 * Copyright (C) 2001 ARM Limited, by David A Rusling
 * Updated to 2.5, Deep Blue Solutions Ltd.

 * 2010-05-01: Guy Winter <gwinter@digitalblocks.com>
 * - ported pxafb and some amba-clcd code to DB9000
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 *
 *	Digital Blocks DB9000 LCD Controller Frame Buffer Driver
 *
 *
 */

#include <linux/backlight.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <video/db9000fb.h>

#include <asm/irq.h>
#include <mach/bitfield.h>
#include <mach/generic.h>
#include <mach/hardware.h>

#ifdef CONFIG_FB_DB9000_DRM
#include <ump/ump_kernel_interface.h>
#include <ump/ump_kernel_interface_ref_drv.h>
#endif /* CONFIG_FB_DB9000_DRM */

/* Complain if VAR is out of range. */
#define DEBUG_VAR 1
#define DRIVER_NAME "clcd-db9000"
#define DEF_BRIGHTNESS 0x80

static char *mode_option __devinitdata;

/* Bits which should not be set in machine configuration structures */
#define CR1_INVALID_CONFIG_MASK	(~(DB9000_CR1_ENB | DB9000_CR1_LPE |\
			DB9000_CR1_BPP(7) | DB9000_CR1_RGB | DB9000_CR1_EPO |\
			DB9000_CR1_EBO | DB9000_CR1_DEP | DB9000_CR1_PCP |\
			DB9000_CR1_HSP | DB9000_CR1_VSP | DB9000_CR1_OPS(7) |\
			DB9000_CR1_PSS | DB9000_CR1_FDW(3) | DB9000_CR1_LPS |\
			DB9000_CR1_FBP | DB9000_CR1_DEE | DB9000_CR1_DFR |\
			DB9000_CR1_DFB | DB9000_CR1_DFE))

#define DB9000_IMR_MASK_ALL (DB9000_ISR_OFUM | DB9000_ISR_OFOM |\
			DB9000_ISR_IFUM | DB9000_ISR_IFOM | DB9000_ISR_FERM |\
			DB9000_ISR_MBEM | DB9000_ISR_VCTM | DB9000_ISR_BAUM |\
			DB9000_ISR_LDDM | DB9000_ISR_ABLM | DB9000_ISR_ARIM |\
			DB9000_ISR_ARSM | DB9000_ISR_FBEM | DB9000_ISR_FNCM |\
			DB9000_ISR_FLCM)


/* Mali DRM Integration */
#ifdef CONFIG_FB_DB9000_DRM
#include <linux/uaccess.h>
 
static ump_dd_handle ump_wrapped_buffer;
static ump_dd_physical_block ump_memory_description;
static int got_ump_handle = 0;
 
/* Change this ioctl according to your specific UMP integration with LCD kernel driver */
#define GET_UMP_SECURE_ID _IOWR('m', 310, unsigned int) //TODO check
#endif /* CONFIG_FB_DB9000_DRM */


extern uint32_t __attribute__((weak)) __div64_32(uint64_t *n, uint32_t base);
static inline void db9000fb_backlight_power(struct db9000fb_info *fbi, int on);
static inline void db9000fb_lcd_power(struct db9000fb_info *fbi, int on);
static int db9000fb_activate_var(struct fb_var_screeninfo *var,
		struct db9000fb_info *fbi);
static void set_ctrlr_state(struct db9000fb_info *fbi, u_int state);

static inline unsigned long
lcd_readl(struct db9000fb_info *fbi, unsigned int off)
{
	return readl(fbi->mmio_base + off);
}

static inline void
lcd_writel(struct db9000fb_info *fbi, unsigned int off, unsigned long val)
{
	writel(val, fbi->mmio_base + off);
}

static inline void
db9000fb_schedule_work(struct db9000fb_info *fbi, u_int state)
{
	unsigned long flags;

	local_irq_save(flags);
	/*
	 * We need to handle two requests being made at the same time.
	 * There are two important cases:
	 * 1. When we are changing VT (C_REENABLE) while unblanking (C_ENABLE)
	 *	We must perform the unblanking, which will do our REENABLE for
	 *	us.
	 * 2. When we are blanking, but immediately unblank before we have
	 *	blanked. We do the "REENABLE" thing here as well, just to be
	 *	sure.
	 */
	if (fbi->task_state == C_ENABLE && state == C_REENABLE)
		state = (u_int) -1;

	if (fbi->task_state == C_DISABLE && state == C_ENABLE)
		state = C_REENABLE;

	if (state != (u_int)-1) {
		fbi->task_state = state;
		schedule_work(&fbi->task);
	}

	local_irq_restore(flags);
}

static inline u_int convert_bitfield(u_int val, struct fb_bitfield *bf)
{
	unsigned int mask = (1 << bf->length) - 1;

	return (val >> (16 - bf->length) & mask) << bf->offset;
}

static int
db9000fb_setpalettereg(u_int regno, u_int red, u_int green, u_int blue, u_int
		trans, struct fb_info *info)
{
	struct db9000fb_info *fbi = to_db9000fb(info);
	u_int val;
	u16 *pal;

	if (regno >= fbi->palette_size)
		return 1;

	if (fbi->fb.var.grayscale) {
		fbi->palette_cpu[regno] = ((blue >> 8) & 0x00ff);
		return 0;
	}

	pal = (u16 *)fbi->palette_cpu;

	switch (fbi->reg_cr1 & (DB9000_CR1_RGB | DB9000_CR1_OPS(1))) {
		/* RGB == 0 && OPS[0] == 1 */
		/* RGB, 5:5:5 format */
	case DB9000_CR1_OPS(1):
		val = ((red >> 1) & 0x7c00);
		val |= ((green >> 6) & 0x03e0);
		val |= ((blue >> 11) & 0x001f);
		pal[regno] = val;
		break;
		/* RGB == 1 && OPS[0] == 1	*/
		/* BGR, 5:5:5 format		*/
	case (DB9000_CR1_RGB | DB9000_CR1_OPS(1)):
		val = ((blue >> 1) & 0x7c00);
		val |= ((green >> 6) & 0x03e0);
		val |= ((red >> 11) & 0x001f);
		pal[regno] = val;
		break;
		/* RGB == 0 && OPS[0] == 0 */
		/* RGB, 5:6:5 format */
	case 0:
		val = ((red >> 0) & 0xf800);
		val |= ((green >> 5) & 0x07e0);
		val |= ((blue >> 11) & 0x001f);
		pal[regno] = val;
		break;
		/* RGB == 1 && OPS[0] == 0	*/
		/* BGR, 5:6:5 format		*/
	case DB9000_CR1_RGB:
		val = ((blue >> 0) & 0xf800);
		val |= ((green >> 6) & 0x07e0);
		val |= ((red >> 11) & 0x001f);
		pal[regno] = val;
		break;
	}

	return 0;
}

static int
db9000fb_setcolreg(u_int regno, u_int red, u_int green, u_int blue, u_int trans,
		struct fb_info *info)
{
	struct db9000fb_info *fbi = to_db9000fb(info);
	unsigned int val;
	int ret = 1;

	/*
	 * If inverse mode was selected, invert all the colours rather than the
	 * register number. The register number is what you poke into the
	 * framebuffer to produce the colour you requested.
	 */
	if (fbi->cmap_inverse) {
		red	= 0xffff - red;
		green	= 0xffff - green;
		blue	= 0xffff - blue;
	}

	/*
	 * If greyscale is true, then we convert the RGB value to greyscale no
	 * matter what visual we are using.
	 */
	if (fbi->fb.var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
				7471 * blue) >> 16;

	switch (fbi->fb.fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/*
		 * 16-bit True Colour. We encode the RGB value according to the
		 * RGB bitfield information.
		 */
		if (regno < 16) {
			u32 *pal = fbi->fb.pseudo_palette;

			val = convert_bitfield(red, &fbi->fb.var.red);
			val |= convert_bitfield(green, &fbi->fb.var.green);
			val |= convert_bitfield(blue, &fbi->fb.var.blue);

			pal[regno] = val;
			ret = 0;
		}
		break;

	case FB_VISUAL_STATIC_PSEUDOCOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		ret = db9000fb_setpalettereg(regno, red, green, blue, trans,
				info);
		break;
	}

	return ret;
}

static int db9000fb_setcmap(struct fb_cmap *cmap, struct fb_info *info)
{
	int count, index, r;
	u16 *red, *green, *blue, *transp;
	u16 trans = 0xffff;

	red	= cmap->red;
	green	= cmap->green;
	blue	= cmap->blue;
	transp	= cmap->transp;
	index	= cmap->start;

	for (count = 0; count < cmap->len; count++) {
		if (transp)
			trans = *transp++;
		r = db9000fb_setcolreg(index++, *red++, *green++, *blue++,
				trans, info);
		if (r != 0)
			return r;
	}

	return 0;
}

/*
 * db9000fb_bpp_to_cr1():
 *	Convert a bits per pixel value to the correct bit pattern for CR1
 */
static int db9000fb_bpp_to_cr1(struct fb_var_screeninfo *var)
{
	int ret = 0;

	switch (var->bits_per_pixel) {
	case 1:
		ret = DB9000_CR1_BPP(DB9000_CR1_BPP_1bpp);
		break;
	case 2:
		ret = DB9000_CR1_BPP(DB9000_CR1_BPP_2bpp);
		break;
	case 4:
		ret = DB9000_CR1_BPP(DB9000_CR1_BPP_4bpp);
		break;
	case 8:
		ret = DB9000_CR1_BPP(DB9000_CR1_BPP_8bpp);
		break;
	case 16:
		ret = DB9000_CR1_BPP(DB9000_CR1_BPP_16bpp);
		break;
	case 18:
		ret = DB9000_CR1_BPP(DB9000_CR1_BPP_18bpp);
		break;
	case 24:
	default:
		ret = DB9000_CR1_BPP(DB9000_CR1_BPP_24bpp);
	}

	return ret;
}

#ifdef CONFIG_CPU_FREQ
/*
 * db9000fb_display_dma_period()
 *	Calculate the minimum period (in picoseconds) between two DMA requests
 *	for the LCD controller. If we hit this, it means we're doing nothing but
 *	LCD DMA.
 */
static unsigned int db9000fb_display_dma_period(struct fb_var_screeninfo *var)
{
	/*
	 * Period = pixclock * bits_per_byte * bytes_per_transfer /
	 * memory_bits_per_pixel;
	 */
	return var->pixclock * 8 * 16 / var->bits_per_pixel;
}
#endif

/*
 * db9000fb_check_var():
 *	Get the video params out of 'var'. If a value doesn't fit, round it up,
 *	if it's too big, return -EINVAL.
 *
 * Round up in the following order: bits_per_pixel, xres, yres, xres_virtual,
 * yres_virtual, xoffset, yoffset, grayscale, bitfields, horizontal timing,
 * vertical timing.
 */
static int
db9000fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	/*
	 * Setup the RGB parameters for this display.
	 *
	 * The pixel packing format is described on page ?? of the DB9000 TRM.
	 */
	if (var->hsync_len < 0 || var->hsync_len > 255)
		return -EINVAL;
	if (var->left_margin < 0 || var->left_margin > 1023)
		return -EINVAL;
	if (var->right_margin < 0 || var->right_margin > 1023)
		return -EINVAL;
	if (var->yres < 0 || var->yres > 4096)
		return -EINVAL;
	if (var->xres < 16 || var->xres > 4096 || (var->xres % 16))
		return -EINVAL;
	if (var->vsync_len < 0 || var->vsync_len > 255)
		return -EINVAL;
	if (var->upper_margin < 0 || var->upper_margin > 1023)
		return -EINVAL;
	if (var->lower_margin < 0 || var->lower_margin > 1023)
		return -EINVAL;
	if (var->pixclock <= 0)
		return -EINVAL;

	switch (var->bits_per_pixel) {
	case 1:
	case 2:
	case 4:
	case 8:
		var->red.offset = var->green.offset = 0;
		var->blue.offset = var->transp.offset = 0;
		var->red.length		= var->bits_per_pixel;
		var->green.length	= var->bits_per_pixel;
		var->blue.length	= var->bits_per_pixel;
		var->transp.length	= 0;
		break;
	case 16:
		var->red.offset		= 11;
		var->red.length		= 5;
		var->green.offset	= 5;
		var->green.length	= 6;
		var->blue.offset	= 0;
		var->blue.length	= 5;
		var->transp.offset = var->transp.length = 0;
		break;
	case 18: /* RGB666 */
		var->transp.offset = var->transp.length = 0;
		var->red.offset		= 12;
		var->red.length		= 6;
		var->green.offset	= 6;
		var->green.length	= 6;
		var->blue.offset	= 0;
		var->blue.length	= 6;
		break;
	case 24: /* RGB888 */
		var->transp.offset = var->transp.length = 0;
		var->red.offset		= 16;
		var->red.length		= 8;
		var->green.offset	= 8;
		var->green.length	= 8;
		var->blue.offset	= 0;
		var->blue.length	= 8;
		break;
	case 32: /* RGB888 */
		var->transp.offset	= 24;
		var->transp.length	= 0;
		var->red.offset		= 16;
		var->red.length		= 8;
		var->green.offset	= 8;
		var->green.length	= 8;
		var->blue.offset	= 0;
		var->blue.length	= 8;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static inline void db9000fb_set_truecolor(u_int is_true_color)
{
	/* do your machine-specific setup if needed */
}

#ifdef CONFIG_BACKLIGHT_DB9000_LCD
static int db9000_bl_update_status(struct backlight_device *bl)
{
	struct db9000fb_info *fbi = bl_get_data(bl);
	int power = fbi->bl_power;
	int brightness = bl->props.brightness;

	if (bl->props.fb_blank != fbi->bl_power)
		power =	bl->props.fb_blank;
	else if (bl->props.power != fbi->bl_power)
		power =	bl->props.power;

	if (power == FB_BLANK_UNBLANK && brightness < 0)
		brightness = lcd_readl(fbi, DB9000_PWMDCR);
	else if (power != FB_BLANK_UNBLANK)
		brightness = 0;

	lcd_writel(fbi, DB9000_PWMDCR, brightness);
	bl->props.fb_blank = bl->props.power = fbi->bl_power = power;

	return 0;
}

static int db9000_bl_get_brightness(struct backlight_device *bl)
{
	struct db9000fb_info *fbi = bl_get_data(bl);

	return lcd_readl(fbi, DB9000_PWMDCR);
}

static const struct backlight_ops db9000_lcd_bl_ops = {
	.update_status = db9000_bl_update_status,
	.get_brightness	= db9000_bl_get_brightness,
};

static void init_backlight(struct db9000fb_info *fbi)
{
	struct backlight_device *bl;
	struct backlight_properties props = {0, };

	if (fbi->backlight)
		return;

	fbi->bl_power = FB_BLANK_UNBLANK;
	props.max_brightness = 0xff;
	props.type = BACKLIGHT_RAW;
	bl = backlight_device_register("backlight", &fbi->pdev->dev, fbi,
			&db9000_lcd_bl_ops, &props);

	if (IS_ERR(bl)) {
		dev_err(&fbi->pdev->dev, "error %ld on backlight register\n",
				PTR_ERR(bl));
		return;
	}

	fbi->backlight = bl;

	bl->props.power = FB_BLANK_UNBLANK;
	bl->props.fb_blank = FB_BLANK_UNBLANK;
	bl->props.brightness = db9000_bl_get_brightness(bl);
}

static void exit_backlight(struct db9000fb_info *fbi)
{
	if (fbi->backlight)
		backlight_device_unregister(fbi->backlight);
}
#endif

/*
 * db9000fb_set_par():
 *	Set the user defined part of the display for the specified console
 */
static int db9000fb_set_par(struct fb_info *info)
{
	struct db9000fb_info *fbi = to_db9000fb(info);

	fbi->fb.var.xres_virtual = fbi->fb.var.xres;
	fbi->fb.var.yres_virtual = fbi->fb.var.yres *
		NUM_OF_FRAMEBUFFERS;

	if (fbi->fb.var.bits_per_pixel >= 16)
		fbi->fb.fix.visual = FB_VISUAL_TRUECOLOR;
	else if (!fbi->cmap_static)
		fbi->fb.fix.visual = FB_VISUAL_PSEUDOCOLOR;
	else
		fbi->fb.fix.visual = FB_VISUAL_STATIC_PSEUDOCOLOR;

	fbi->fb.fix.line_length = fbi->fb.var.xres_virtual *
					fbi->fb.var.bits_per_pixel / 8;
	if (fbi->fb.var.bits_per_pixel >= 16) {
		fbi->palette_size = 0;
		fbi->palette_mode = PAL_NONE;
	} else {
		fbi->palette_size = 1 << fbi->fb.var.bits_per_pixel;
	}

	fbi->palette_cpu = (u16 *)&fbi->palette[0];

	/* Set (any) board control register to handle new color depth */
	db9000fb_set_truecolor(fbi->fb.fix.visual == FB_VISUAL_TRUECOLOR);

	if (fbi->fb.var.bits_per_pixel >= 16) {
		if (fbi->fb.cmap.len)
			fb_dealloc_cmap(&fbi->fb.cmap);
	} else {
		fb_alloc_cmap(&fbi->fb.cmap, 1<<fbi->fb.var.bits_per_pixel, 0);
	}

	db9000fb_activate_var(&fbi->fb.var, fbi);

	return 0;
}

/*
 * db9000fb_blank():
 *	Blank the display by setting all palette values to zero. Note, the 16
 *	bpp mode does not really use the palette, so this will not blank the
 *	display in all modes.
 */
static int db9000fb_blank(int blank, struct fb_info *info)
{
	struct db9000fb_info *fbi = to_db9000fb(info);
	int i;

	switch (blank) {
	case FB_BLANK_POWERDOWN:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
		if (fbi->fb.fix.visual == FB_VISUAL_PSEUDOCOLOR ||
		fbi->fb.fix.visual == FB_VISUAL_STATIC_PSEUDOCOLOR) {
			for (i = 0; i < fbi->palette_size; i++)
				db9000fb_setpalettereg(i, 0, 0, 0, 0, info);
		}

		db9000fb_schedule_work(fbi, C_DISABLE);
	/* TODO: if (db9000fb_blank_helper) db9000fb_blank_helper(blank); */
		break;

	case FB_BLANK_UNBLANK:
	/* TODO: if (db9000fb_blank_helper) db9000fb_blank_helper(blank); */
		if (fbi->fb.fix.visual == FB_VISUAL_PSEUDOCOLOR ||
			fbi->fb.fix.visual == FB_VISUAL_STATIC_PSEUDOCOLOR)
			fb_set_cmap(&fbi->fb.cmap, info);
		db9000fb_schedule_work(fbi, C_ENABLE);
	}
	return 0;
}

static int db9000fb_open(struct fb_info *info, int user)
{
	struct db9000fb_info *fbi = to_db9000fb(info);

	/* allow only one user at a time */
	if (atomic_inc_and_test(&fbi->usage))
		return -EBUSY;

	/* Enable Controller */
	set_ctrlr_state(fbi, C_ENABLE);

	return 0;
}

static int db9000fb_release(struct fb_info *info, int user)
{
	struct db9000fb_info *fbi = to_db9000fb(info);

	set_ctrlr_state(fbi, C_DISABLE);
	atomic_dec(&fbi->usage);

	return 0;
}

/* Pan the display if device supports it. */
static int db9000fb_pan_display(struct fb_var_screeninfo *var,
	struct fb_info *info)
{
	struct db9000fb_info *fbi = to_db9000fb(info);
	unsigned next_frame_address;
	u_int y_bottom = var->yoffset;

	if (!(var->vmode & FB_VMODE_YWRAP))
		y_bottom += var->yres;

	BUG_ON(y_bottom > var->yres_virtual);

	next_frame_address = info->fix.smem_start +
		(var->yoffset * info->fix.line_length);

	/* There are some probing calls with no buffer switch */
	if (fbi->reg_dbar != next_frame_address) {
		lcd_writel(fbi, DB9000_DBAR, next_frame_address);
		lcd_writel(fbi, DB9000_DEAR, next_frame_address +
			(fbi->video_mem_size / NUM_OF_FRAMEBUFFERS));
		lcd_writel(fbi, DB9000_MRR,
			DB9000_MRR_DEAR_MRR(next_frame_address +
			(fbi->video_mem_size / NUM_OF_FRAMEBUFFERS)) |
			DB9000_MRR_MRR(DB9000_MRR_OUTST_4));
		/*
		 * Force waiting till the current buffer is completely drawn by
		 * video controller
		 */
		wait_for_completion(&fbi->vsync_notifier);
	}

	return 0;
}

static int db9000fb_ioctl(struct fb_info *fb, unsigned int cmd,
		unsigned long arg)
{
	int ret = 0;
#ifdef CONFIG_FB_DB9000_DRM
	u32 __user *psecureid = (u32 __user *) arg;
	ump_secure_id secure_id;
#endif /* CONFIG_FB_DB9000_DRM */

	switch (cmd) {
#ifdef CONFIG_FB_DB9000_DRM
	/* handle additional 'ioctl' for secure ID */
	case GET_UMP_SECURE_ID:
		if (!got_ump_handle) {
			ump_wrapped_buffer =
				ump_dd_handle_create_from_phys_blocks(
					&ump_memory_description, 1);
			got_ump_handle = 1;
		}
		secure_id = ump_dd_secure_id_get(ump_wrapped_buffer);
		ret = put_user((unsigned int)secure_id, psecureid);
		break;
#endif /* CONFIG_FB_DB9000_DRM */
	default:
		break;
	}
	return ret;
}

static struct fb_ops db9000fb_ops = {
	.owner		= THIS_MODULE,
	.fb_open	= db9000fb_open,
	.fb_release	= db9000fb_release,
	.fb_check_var	= db9000fb_check_var,
	.fb_set_par	= db9000fb_set_par,
	.fb_setcolreg	= db9000fb_setcolreg,
	.fb_pan_display = db9000fb_pan_display,
	.fb_setcmap	= db9000fb_setcmap,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_blank	= db9000fb_blank,
	.fb_ioctl	= db9000fb_ioctl,
};

/*
 * Calculate the PCTR value from the clock rate (in picoseconds).
 *
 *				 Bus_Clock
 *	pixclock	=	-------------
 *				 (PCD + 2)
 *
 *				 Bus_Clock
 *	PCD		=	------------- - 2
 *				 pixclock
 *
 * Where:
 *	Bus_Clock = Bus Clock
 *	PCD = PCTR[7:0]
 *
 * pixclock here is in picoseconds.
 *
 */
static inline int get_pcd(struct db9000fb_info *fbi,
				unsigned int pixclock)
{
	uint32_t pcd, pcd_32;

	pcd = (uint32_t)(clk_get_rate(fbi->bus_clk)/1000000);
	pcd_32 = (pcd * pixclock)/1000000;

	if (pcd_32 < 2)
		return -EINVAL;

	pcd_32 = pcd_32 - 2;
	return pcd_32;
}

/*
 * Some touchscreens need hsync information from the video driver to function
 * correctly. We export it here. Note that 'hsync_time' and the value returned
 * from db9000fb_get_hsync_time() is the *reciprocal* of the hsync period in
 * seconds.
 */
static inline void set_hsync_time(struct db9000fb_info *fbi, unsigned int pcd)
{
	unsigned long htime;

	if ((pcd == 0) || (fbi->fb.var.hsync_len == 0)) {
		fbi->hsync_time = 0;
		return;
	}

	htime = clk_get_rate(fbi->clk) / (pcd * fbi->fb.var.hsync_len);

	fbi->hsync_time = htime;
}

unsigned long db9000fb_get_hsync_time(struct device *dev)
{
	struct db9000fb_info *fbi = dev_get_drvdata(dev);

	/* If display is blanked/suspended, hsync isn't active */
	if (!fbi || (fbi->state != C_ENABLE))
		return 0;

	return fbi->hsync_time;
}
EXPORT_SYMBOL(db9000fb_get_hsync_time);

static int setup_frame_dma(struct db9000fb_info *fbi, int dma, int pal,
		unsigned int offset, size_t size)
{

	if (dma < 0 || dma >= DMA_MAX)
		return -EINVAL;
	if ((pal == PAL_NONE) || (pal == PAL_STATIC))
		fbi->reg_cr1 &= ~DB9000_CR1_PSS;
	else if (pal == PAL_IN_FB)
		fbi->reg_cr1 |= DB9000_CR1_PSS;

	return 0;
}

static void setup_parallel_timing(struct db9000fb_info *fbi,
				struct fb_var_screeninfo *var)
{
	u32 clk_rate;
	int pcd;

	clk_rate = PICOS2KHZ(var->pixclock) * 1000;
	pr_debug("Clock value is %d", clk_rate);

	fbi->reg_pctr &= ~0x7FF;

	pcd = get_pcd(fbi, var->pixclock);
	if (pcd >= 0) {
		/* first try bus clk src */
		fbi->reg_pctr |= DB9000_PCTR_PCD(pcd);
		set_hsync_time(fbi, pcd);
		clk_set_parent(fbi->clk, fbi->bus_clk);

	} else {
		fbi->reg_pctr |= DB9000_PCTR_PCI;
		/* else try pixel clk src */
		clk_set_rate(fbi->pixel_clk, clk_rate);
		clk_set_parent(fbi->clk, fbi->pixel_clk);
	}

	fbi->reg_htr =
	/* horizontal sync width */
		DB9000_HTR_HSW(var->hsync_len) |
	/* horizontal back porch */
		DB9000_HTR_HBP(var->left_margin) |
	/* Pixels per line */
		DB9000_HTR_PPL((var->xres)/16) |
	/* Horizontal Front Porch */
		DB9000_HTR_HFP(var->right_margin);

	/* Vertical and Horizontal Timing Extension write */
	if (fbi->db9000_rev >= DB9000_REVISION_1_14) {
		fbi->reg_hvter =
			DB9000_HVTER_HFPE(var->right_margin) |
			DB9000_HVTER_HBPE(var->left_margin) |
			DB9000_HVTER_VFPE(var->lower_margin) |
			DB9000_HVTER_VBPE(var->upper_margin);
	}

	fbi->reg_vtr1 =
		DB9000_VTR1_VBP(var->upper_margin) |
		DB9000_VTR1_VFP(var->lower_margin) |
		DB9000_VTR1_VSW((var->vsync_len));

	fbi->reg_vtr2 = DB9000_VTR2_LPP(var->yres);

	fbi->reg_cr1 &= ~(DB9000_CR1_HSP | DB9000_CR1_VSP);
	fbi->reg_cr1 |=
		(var->sync & FB_SYNC_HOR_HIGH_ACT) ? 0 : DB9000_CR1_HSP;
	fbi->reg_cr1 |=
		(var->sync & FB_SYNC_VERT_HIGH_ACT) ? 0 : DB9000_CR1_VSP;
	fbi->reg_dear = lcd_readl(fbi, DB9000_DBAR) + (var->xres * var->yres *
			var->bits_per_pixel / 8);
}

/*
 * db9000fb_activate_var():
 *	Configures LCD Controller based on entries in var parameter. Settings
 *	are only written to the controller if changes were made.
 */
static int db9000fb_activate_var(struct fb_var_screeninfo *var,
				struct db9000fb_info *fbi)
{
	size_t nbytes;

#if DEBUG_VAR
	if (var->xres < 16 || var->xres > 4096 || (var->xres % 16))
		pr_err("%s: invalid xres %d\n", fbi->fb.fix.id, var->xres);

	switch (var->bits_per_pixel) {
	case 1:
	case 2:
	case 4:
	case 8:
	case 16:
	case 18:
	case 24:
	case 32:
		break;
	default:
		pr_err("%s: invalid bit depth %d\n", fbi->fb.fix.id,
				var->bits_per_pixel);
		break;
	}
	if (var->hsync_len < 0 || var->hsync_len > 255)
		pr_err("%s: invalid hsync_len %d\n", fbi->fb.fix.id,
				var->hsync_len);
	if (var->left_margin < 0 || var->left_margin > 1023)
		pr_err("%s: invalid left_margin %d\n",
			fbi->fb.fix.id, var->left_margin);
	if (var->right_margin < 0 || var->right_margin > 1023)
		pr_err("%s: invalid right_margin %d\n", fbi->fb.fix.id,
				var->right_margin);
	if (var->yres < 0 || var->yres > 4096)
		pr_err("%s: invalid yres %d\n", fbi->fb.fix.id, var->yres);
	if (var->vsync_len < 0 || var->vsync_len > 255)
		pr_err("%s: invalid vsync_len %d\n", fbi->fb.fix.id,
				var->vsync_len);
	if (var->upper_margin < 0 || var->upper_margin > 1023)
		pr_err("%s: invalid upper_margin %d\n", fbi->fb.fix.id,
				var->upper_margin);
	if (var->lower_margin < 0 || var->lower_margin > 1023)
		pr_err("%s: invalid lower_margin %d\n", fbi->fb.fix.id,
				var->lower_margin);
	if (var->pixclock <= 0)
		pr_err("%s: invalid pixel clock %d\n", fbi->fb.fix.id,
				var->pixclock);
	if (var->xres_virtual < var->xres)
		return -EINVAL;
	if (var->yres_virtual < var->yres * NUM_OF_FRAMEBUFFERS)
		return -EINVAL;
#endif
	/* Update shadow copy atomically */
	setup_parallel_timing(fbi, var);

	fbi->reg_imr = DB9000_IMR_MASK_ALL;
	fbi->reg_cr1 &= ~DB9000_CR1_BPP(7);
	fbi->reg_cr1 |= db9000fb_bpp_to_cr1(var);

	nbytes = var->yres * fbi->fb.fix.line_length;

	if ((var->bits_per_pixel >= 16))
		setup_frame_dma(fbi, DMA_BASE, PAL_NONE, 0, nbytes);
	else if (fbi->reg_cr1 & DB9000_CR1_PSS)
		setup_frame_dma(fbi, DMA_BASE, PAL_IN_FB, 0, nbytes);
	else
		setup_frame_dma(fbi, DMA_BASE, PAL_STATIC, 0, nbytes);

	/*
	 * Only update the registers if the controller is enabled and something
	 * has changed.
	 * DBAR is not checked here, it is adviced to be updated on BAU event
	 */
	if ((lcd_readl(fbi, DB9000_CR1) != fbi->reg_cr1) ||
		(lcd_readl(fbi, DB9000_HTR) != fbi->reg_htr) ||
		(lcd_readl(fbi, DB9000_VTR1) != fbi->reg_vtr1) ||
		(lcd_readl(fbi, DB9000_VTR2) != fbi->reg_vtr2) ||
		(lcd_readl(fbi, DB9000_HVTER) != fbi->reg_hvter) ||
		(lcd_readl(fbi, DB9000_PCTR) != fbi->reg_pctr) ||
		(lcd_readl(fbi, DB9000_DBAR) != fbi->reg_dbar) ||
		(lcd_readl(fbi, DB9000_DEAR) != fbi->reg_dear))
		db9000fb_schedule_work(fbi, C_REENABLE);

	return 0;
}

/*
 * NOTE! The following functions are purely helpers for set_ctrlr_state. Do not
 * call them directly; set_ctrlr_state does the correct serialisation to ensure
 * that things happen in the right way 100% of time time.
 * -- rmk
 */
static inline void db9000fb_backlight_power(struct db9000fb_info *fbi, int on)
{
	fbi->reg_pwmfr &= ~DB9000_PWMFR_PWM_FCE;

	if (on) {
		fbi->reg_pwmfr |= DB9000_PWMFR_PWM_FCE;
		lcd_writel(fbi, DB9000_PWMDCR, fbi->reg_pwmdcr);
	} else {
		fbi->reg_pwmdcr = lcd_readl(fbi, DB9000_PWMDCR);
		lcd_writel(fbi, DB9000_PWMDCR, 0x0);
	}

	lcd_writel(fbi, DB9000_PWMFR, fbi->reg_pwmfr);
}

static inline void db9000fb_lcd_power(struct db9000fb_info *fbi, int on)
{
	fbi->reg_cr1 &= ~DB9000_CR1_LPE;
	if (on)
		fbi->reg_cr1 |= DB9000_CR1_LPE;
	lcd_writel(fbi, DB9000_CR1, fbi->reg_cr1);
}

static void db9000fb_setup_gpio(struct db9000fb_info *fbi, bool on)
{
	/* gpio or clcd pad selection */
	if (fbi->setup_gpio)
		fbi->setup_gpio(on);
}

static void db9000fb_enable_controller(struct db9000fb_info *fbi)
{
	int i;
	u32 val;
	unsigned int isr;
	unsigned int imr;

	pr_debug("db9000fb: Enabling LCD controller\n");
	pr_debug("reg_cr1: 0x%08x\n", (unsigned int) fbi->reg_cr1);
	pr_debug("reg_htr : 0x%08x\n", (unsigned int) fbi->reg_htr);
	pr_debug("reg_vtr1: 0x%08x\n", (unsigned int) fbi->reg_vtr1);
	pr_debug("reg_vtr2: 0x%08x\n", (unsigned int) fbi->reg_vtr2);
	pr_debug("reg_pctr: 0x%08x\n", (unsigned int) fbi->reg_pctr);

	/* enable LCD controller clock */
	if (!fbi->clk_enabled) {
		fbi->clk_enabled =true;
		clk_enable(fbi->clk);
	}

	/* Write into the palette memory */
	if (fbi->palette_size > 0) {
		for (i = 0; i < (fbi->palette_size/2) ; ++i) {
			val = fbi->palette[i];
			lcd_writel(fbi, (DB9000_PALT + i*4), val);
		}
	}

	lcd_writel(fbi, DB9000_HTR, fbi->reg_htr);
	lcd_writel(fbi, DB9000_VTR1, fbi->reg_vtr1);
	lcd_writel(fbi, DB9000_VTR2, fbi->reg_vtr2);
	lcd_writel(fbi, DB9000_HVTER, fbi->reg_hvter);
	lcd_writel(fbi, DB9000_PCTR, fbi->reg_pctr | DB9000_PCTR_PCR);

	fbi->reg_dbar = fbi->fb.fix.smem_start;
	fbi->reg_dear = fbi->reg_dbar + (fbi->video_mem_size /
			NUM_OF_FRAMEBUFFERS);

	lcd_writel(fbi, DB9000_DBAR, fbi->reg_dbar);
	lcd_writel(fbi, DB9000_DEAR, fbi->reg_dear);

	/* configure MRR to 4 outstanding requests */
	lcd_writel(fbi, DB9000_MRR,
		DB9000_MRR_DEAR_MRR(fbi->reg_dear) |
		DB9000_MRR_MRR(DB9000_MRR_OUTST_4));

	/* enable BAU event for IRQ */
	isr = lcd_readl(fbi, DB9000_ISR);
	imr = lcd_readl(fbi, DB9000_IMR);
	lcd_writel(fbi, DB9000_ISR, isr | DB9000_ISR_BAU);
	lcd_writel(fbi, DB9000_IMR, imr | DB9000_ISR_BAU);

	lcd_writel(fbi, DB9000_CR1,
		fbi->reg_cr1 | DB9000_CR1_ENB | DB9000_CR1_DEE);
}

static void db9000fb_disable_controller(struct db9000fb_info *fbi)
{
	uint32_t cr1;
	cr1 = lcd_readl(fbi, DB9000_CR1) & ~DB9000_CR1_ENB;
	lcd_writel(fbi, DB9000_CR1, cr1);
	msleep(100);

	if (fbi->clk_enabled) {
		fbi->clk_enabled = false;
		clk_disable(fbi->clk);
	}
}

/* db9000fb_handle_irq: Handle 'LCD DONE' interrupts. */
static irqreturn_t db9000fb_handle_irq(int irq, void *dev_id)
{
	struct db9000fb_info *fbi = dev_id;
	unsigned int isr = lcd_readl(fbi, DB9000_ISR);
	unsigned int imr;
	u32 dbar;

	if (isr & DB9000_ISR_BAU) {
		/*DMA Base Address Register Update to DCAR*/
		dbar = lcd_readl(fbi, DB9000_DBAR);
		if (dbar != fbi->reg_dbar) {
			fbi->reg_dbar = dbar;
			fbi->reg_dear = dbar + (fbi->video_mem_size /
				NUM_OF_FRAMEBUFFERS);
			complete(&fbi->vsync_notifier);
		}
	}

	if (isr & DB9000_ISR_LDD) {
		imr = lcd_readl(fbi, DB9000_IMR);
		lcd_writel(fbi, DB9000_IMR, imr | DB9000_ISR_LDDM);
		complete(&fbi->disable_done);
	}
	lcd_writel(fbi, DB9000_ISR, isr);
	return IRQ_HANDLED;
}

/*
 * This function must be called from task context only, since it will sleep when
 * disabling the LCD controller, or if we get two contending processes trying to
 * alter state.
 */
static void set_ctrlr_state(struct db9000fb_info *fbi, u_int state)
{
	u_int old_state;
	mutex_lock(&fbi->ctrlr_lock);
	old_state = fbi->state;

	/* Hack around fbcon initialisation. */
	if (old_state == C_STARTUP && state == C_REENABLE)
		state = C_STARTUP;

	switch (state) {
	case C_DISABLE_CLKCHANGE:
		/*
		 * Disable controller for clock change. If the controller is
		 * already disabled, then do nothing.
		 */
		if (old_state != C_DISABLE && old_state != C_DISABLE_PM) {
			db9000fb_backlight_power(fbi, 0);
			db9000fb_lcd_power(fbi, 0);
			db9000fb_setup_gpio(fbi, false);
			fbi->state = state;
			db9000fb_disable_controller(fbi);
		}
		break;

	case C_DISABLE_PM:
	case C_DISABLE:
		/* Disable controller */
		if (old_state != C_DISABLE) {
			fbi->state = state;
			db9000fb_backlight_power(fbi, 0);
			db9000fb_lcd_power(fbi, 0);
			db9000fb_setup_gpio(fbi, false);
			if (old_state != C_DISABLE_CLKCHANGE)
				db9000fb_disable_controller(fbi);
		}
		break;

	case C_ENABLE_CLKCHANGE:
		/*
		 * Enable the controller after clock change. Only do this if we
		 * were disabled for the clock change.
		 */
		if (old_state == C_DISABLE_CLKCHANGE) {
			fbi->state = C_ENABLE;
			db9000fb_setup_gpio(fbi, true);
			db9000fb_lcd_power(fbi, 1);
			db9000fb_enable_controller(fbi);
			/* TODO __db9000fb_lcd_power(fbi, 1); */
		}
		break;

	case C_REENABLE:
		/*
		 * Re-enable the controller only if it was already enabled.
		 * This is so we reprogram the control registers.
		 */
		if (old_state == C_ENABLE) {
			db9000fb_backlight_power(fbi, 0);
			db9000fb_lcd_power(fbi, 0);
			db9000fb_setup_gpio(fbi, false);
			db9000fb_disable_controller(fbi);
			msleep(100);
			db9000fb_setup_gpio(fbi, true);
			db9000fb_lcd_power(fbi, 1);
			db9000fb_enable_controller(fbi);
			db9000fb_backlight_power(fbi, 1);
		}
		break;

	case C_ENABLE_PM:
		/*
		 * Re-enable the controller after PM. This is not perfect -
		 * think about the case where we were doing a clock change, and
		 * we suspended half-way through.
		 */
		if (old_state != C_DISABLE_PM)
			break;
		/* fall through */

	case C_ENABLE:
		/*
		 * Power up the LCD screen, enable controller, and turn on the
		 * backlight.
		 */
		if (old_state != C_ENABLE) {

			fbi->state = C_ENABLE;
			db9000fb_setup_gpio(fbi, true);
			db9000fb_lcd_power(fbi, 1);
			db9000fb_enable_controller(fbi);
			db9000fb_backlight_power(fbi, 1);
		}
		break;

	case C_STARTUP:
		fbi->state = C_STARTUP;
		db9000fb_setup_gpio(fbi, true);
		db9000fb_lcd_power(fbi, 1);
		db9000fb_enable_controller(fbi);
		db9000fb_backlight_power(fbi, 1);
		msleep(100);
		db9000fb_backlight_power(fbi, 0);
		db9000fb_lcd_power(fbi, 0);
		db9000fb_setup_gpio(fbi, false);
		db9000fb_disable_controller(fbi);
		break;
	}
	mutex_unlock(&fbi->ctrlr_lock);
}

/*
 * Our LCD controller task (which is called when we blank or unblank) via
 * keventd.
 */
static void db9000fb_task(struct work_struct *work)
{
	struct db9000fb_info *fbi =
		container_of(work, struct db9000fb_info, task);
	u_int state = xchg(&fbi->task_state, -1);

	set_ctrlr_state(fbi, state);
}

#ifdef CONFIG_CPU_FREQ
/*
 * CPU clock speed change handler. We need to adjust the LCD timing parameters
 * when the CPU clock is adjusted by the power management subsystem.
 *
 * TODO: Determine why f->new != 10*get_lclk_frequency_10khz()
 */
static int
db9000fb_freq_transition(
	struct notifier_block *nb, unsigned long val, void *data)
{
	struct db9000fb_info *fbi = TO_INF(nb, freq_transition);
	struct fb_var_screeninfo *var = &fbi->fb.var;
	/* TODO struct cpufreq_freqs *f = data; */

	switch (val) {
	case CPUFREQ_PRECHANGE:
		if (!fbi->ignore_cpufreq_notification)
			set_ctrlr_state(fbi, C_DISABLE_CLKCHANGE);
		break;

	case CPUFREQ_POSTCHANGE:
		if (!fbi->ignore_cpufreq_notification) {
			setup_parallel_timing(fbi, var);
			set_ctrlr_state(fbi, C_ENABLE_CLKCHANGE);
		}
		break;
	}
	return 0;
}

static int
db9000fb_freq_policy(struct notifier_block *nb, unsigned long val, void *data)
{
	struct db9000fb_info *fbi = TO_INF(nb, freq_policy);
	struct fb_var_screeninfo *var = &fbi->fb.var;
	struct cpufreq_policy *policy = data;

	switch (val) {
	case CPUFREQ_ADJUST:
	case CPUFREQ_INCOMPATIBLE:
		pr_debug("min dma period: %d ps, "
			"new clock %d kHz\n", db9000fb_display_dma_period(var),
			policy->max);
		/* TODO: fill in min/max values */
		break;
	}
	return 0;
}
#endif

#ifdef CONFIG_PM
/*
 * Power management hooks. Note that we won't be called from IRQ context, unlike
 * the blank functions above, so we may sleep.
 */
static int db9000fb_suspend(struct device *dev)
{
	struct db9000fb_info *fbi = dev_get_drvdata(dev);

	set_ctrlr_state(fbi, C_DISABLE_PM);
	return 0;
}

static int db9000fb_resume(struct device *dev)
{
	struct db9000fb_info *fbi = dev_get_drvdata(dev);

	set_ctrlr_state(fbi, C_ENABLE_PM);
	return 0;
}

static int db9000fb_thaw(struct device *dev)
{
	struct db9000fb_info *fbi = dev_get_drvdata(dev);

	if (!fbi->clk_enabled) {
		fbi->clk_enabled = true;
		clk_enable(fbi->clk);
	}

	fbi->state = fbi->old_state;

	return 0;
}

static int db9000fb_freeze(struct device *dev)
{
	struct db9000fb_info *fbi = dev_get_drvdata(dev);

	if (fbi->clk_enabled) {
		fbi->clk_enabled = false;
		clk_disable(fbi->clk);
	}

	fbi->old_state = fbi->state;
	fbi->state = C_DISABLE_PM;

	return 0;
}

static const struct dev_pm_ops db9000fb_pm_ops = {
	.suspend	= db9000fb_suspend,
	.resume		= db9000fb_resume,
	.freeze		= db9000fb_freeze,
	.thaw		= db9000fb_thaw,
	.poweroff	= db9000fb_suspend,
	.restore	= db9000fb_resume,
};
#endif

static void db9000fb_decode_mach_info(struct db9000fb_info *fbi,
				struct db9000fb_mach_info *inf)
{
	fbi->cmap_inverse	= inf->cmap_inverse;
	fbi->cmap_static	= inf->cmap_static;
	fbi->reg_cr1		= inf->ctrl_info->cr1;
	fbi->reg_pctr		= inf->ctrl_info->pctr;
	fbi->reg_dear		= inf->ctrl_info->dear;
#ifdef CONFIG_BACKLIGHT_DB9000_LCD
	fbi->reg_pwmfr		= inf->ctrl_info->pwmfr;
#endif
}

static struct db9000fb_info * __devinit db9000fb_init_fbinfo(struct device *dev)
{
	struct db9000fb_info *fbi;
	struct db9000fb_mach_info *inf = dev_get_platdata(dev);

	/* Alloc the db9000fb_info with the embedded pseudo_palette */
	fbi = kzalloc(sizeof(struct db9000fb_info), GFP_KERNEL);
	if (!fbi) {
		dev_err(dev, "%s: kmalloc allocation failed\n", __func__);
		return NULL;
	}

	fbi->clk = clk_get(dev, NULL);
	if (IS_ERR(fbi->clk)) {
		kzfree(fbi);
		return NULL;
	}

	fbi->pixel_clk = inf->pixel_clk;
	fbi->bus_clk = inf->bus_clk;

	if (!fbi->pixel_clk || !fbi->bus_clk)
		return NULL;

	strcpy(fbi->fb.fix.id, DB9000FB_NAME);

	fbi->dev = dev;
	fbi->fb.fix.type	= FB_TYPE_PACKED_PIXELS;
	fbi->fb.fix.type_aux	= 0;
	fbi->fb.fix.xpanstep	= 0;
	fbi->fb.fix.ypanstep	= 1;
	fbi->fb.fix.ywrapstep	= 1;
	fbi->fb.fix.accel	= FB_ACCEL_NONE;

	fbi->fb.var.nonstd	= 0;
	fbi->fb.var.activate	= FB_ACTIVATE_NOW;
	fbi->fb.var.height	= fbi->fb.var.yres;
	fbi->fb.var.width	= fbi->fb.var.xres;
	fbi->fb.var.accel_flags	= 0;
	fbi->fb.var.vmode	= FB_VMODE_NONINTERLACED;
	fbi->fb.pseudo_palette	= fbi->cmap;
	fbi->fb.fbops		= &db9000fb_ops;
	fbi->fb.flags		= FBINFO_DEFAULT;
	fbi->fb.node		= -1;

	fbi->palette_mode = PAL_STATIC;

	fbi->state		= C_STARTUP;
	fbi->task_state		= (u_char)-1;
	fbi->frame_base		= inf->frame_buf_base;
	fbi->video_mem_size	= inf->mem_size;
	db9000fb_decode_mach_info(fbi, inf);

	atomic_set(&fbi->usage, 0);
	init_waitqueue_head(&fbi->ctrlr_wait);
	INIT_WORK(&fbi->task, db9000fb_task);
	mutex_init(&fbi->ctrlr_lock);
	init_completion(&fbi->disable_done);
	init_completion(&fbi->vsync_notifier);

	return fbi;
}

#ifdef CONFIG_FB_DB9000_PARAMETERS
static int __devinit parse_opt_mode(struct device *dev, const char *this_opt)
{
	struct db9000fb_mach_info *inf = dev_get_platdata(dev);

	const char *name = this_opt+5;
	unsigned int namelen = strlen(name);
	int res_specified = 0, bpp_specified = 0;
	unsigned int xres = 0, yres = 0, bpp = 0;
	int yres_specified = 0;
	int i;
	for (i = namelen-1; i >= 0; i--) {
		switch (name[i]) {
		case '-':
			namelen = i;
			if (!bpp_specified && !yres_specified) {
				bpp = strict_strtoul(&name[i+1], 0, 0);
				bpp_specified = 1;
			} else
				goto done;
			break;
		case 'x':
			if (!yres_specified) {
				yres = strict_strtoul(&name[i+1], 0, 0);
				yres_specified = 1;
			} else
				goto done;
			break;
		case '0' ... '9':
			break;
		default:
			goto done;
		}
	}
	if (i < 0 && yres_specified) {
		xres = strict_strtoul(name, 0, 0);
		res_specified = 1;
	}
done:
	if (res_specified) {
		dev_info(dev, "overriding resolution: %dx%d\n", xres, yres);
		inf->modes[0].xres = xres; inf->modes[0].yres = yres;
	}
	if (bpp_specified)
		switch (bpp) {
		case 1:
		case 2:
		case 4:
		case 8:
		case 16:
		case 18:
		case 24:
		case 32:

			inf->ctrl_info->bpp = bpp;
			dev_info(dev, "overriding bit depth: %d\n", bpp);
			break;
		default:
			dev_err(dev, "Depth %d is not valid\n", bpp);
			return -EINVAL;
		}
	return 0;
}

static int __devinit parse_opt(struct device *dev, char *this_opt)
{
	struct db9000fb_mach_info *inf = dev_get_platdata(dev);
	struct fb_videomode *mode = inf->modes;
	struct db9000fb_info *info = dev_get_drvdata(dev);
	char s[81];

	s[0] = '\0';

	if (!strncmp(this_opt, "vmem:", 5)) {
		inf->mem_size = memparse(this_opt + 5, NULL);
	} else if (!strncmp(this_opt, "mode:", 5)) {
		return parse_opt_mode(dev, this_opt);
	} else if (!strncmp(this_opt, "pixclock:", 9)) {
		mode->pixclock = strict_strtoul(this_opt+9, 0, 0);
		sprintf(s, "pixclock: %u\n", mode->pixclock);
	} else if (!strncmp(this_opt, "left:", 5)) {
		mode->left_margin = strict_strtoul(this_opt+5, 0, 0);
		sprintf(s, "left: %u\n", mode->left_margin);
	} else if (!strncmp(this_opt, "right:", 6)) {
		mode->right_margin = strict_strtoul(this_opt+6, 0, 0);
		sprintf(s, "right: %u\n", mode->right_margin);
	} else if (!strncmp(this_opt, "upper:", 6)) {
		mode->upper_margin = strict_strtoul(this_opt+6, 0, 0);
		sprintf(s, "upper: %u\n", mode->upper_margin);
	} else if (!strncmp(this_opt, "lower:", 6)) {
		mode->lower_margin = strict_strtoul(this_opt+6, 0, 0);
		sprintf(s, "lower: %u\n", mode->lower_margin);
	} else if (!strncmp(this_opt, "hsynclen:", 9)) {
		mode->hsync_len = strict_strtoul(this_opt+9, 0, 0);
		sprintf(s, "hsynclen: %u\n", mode->hsync_len);
	} else if (!strncmp(this_opt, "vsynclen:", 9)) {
		mode->vsync_len = strict_strtoul(this_opt+9, 0, 0);
		sprintf(s, "vsynclen: %u\n", mode->vsync_len);
	} else if (!strncmp(this_opt, "hsync:", 6)) {
		if (strict_strtoul(this_opt+6, 0, 0) == 0) {
			sprintf(s, "hsync: Active Low\n");
			mode->sync &= ~FB_SYNC_HOR_HIGH_ACT;
		} else {
			sprintf(s, "hsync: Active High\n");
			mode->sync |= FB_SYNC_HOR_HIGH_ACT;
		}
	} else if (!strncmp(this_opt, "vsync:", 6)) {
		if (strict_strtoul(this_opt+6, 0, 0) == 0) {
			sprintf(s, "vsync: Active Low\n");
			mode->sync &= ~FB_SYNC_VERT_HIGH_ACT;
		} else {
			sprintf(s, "vsync: Active High\n");
			mode->sync |= FB_SYNC_VERT_HIGH_ACT;
		}
	} else if (!strncmp(this_opt, "pixclockpol:", 12)) {
		if (strict_strtoul(this_opt+12, 0, 0) == 0) {
			sprintf(s, "pixel clock polarity: falling edge\n");
			inf->ctrl_info->cr1 = (info->reg_cr1 &
					~DB9000_CR1_PCP) | DB9000_CR1_PixFlEdg;
		} else {
			sprintf(s, "pixel clock polarity: rising edge\n");
			inf->ctrl_info->cr1 = (info->reg_cr1 &
					~DB9000_CR1_PCP) | DB9000_CR1_PixRsEdg;
		}
	} else if (!strncmp(this_opt, "fbp:", 4)) {
		if (strict_strtoul(this_opt+4, 0, 0) == 0) {
			sprintf(s, "frame buffer 24 bit packing: No packing, \
					24 bits per 32-bit word\n");
			inf->ctrl_info->cr1 = (info->reg_cr1 & ~DB9000_CR1_FBP);
		} else {
			sprintf(s, "frame buffer 24 bit packing: pack 4 24 bit \
					pixel in 3 32-bit memory words\n");
			inf->ctrl_info->cr1 = (info->reg_cr1 | DB9000_CR1_FBP);
		}
	} else if (!strncmp(this_opt, "dep:", 4)) {
		if (strict_strtoul(this_opt+4, 0, 0) == 0) {
			sprintf(s, "data enable polarity: active low\n");
			inf->ctrl_info->cr1 = (info->reg_cr1 & ~DB9000_CR1_DEP);
		} else {
			sprintf(s, "data enable polarity: active high\n");
			inf->ctrl_info->cr1 = (info->reg_cr1 | DB9000_CR1_DEP);
		}
	} else if (!strncmp(this_opt, "dee:", 4)) {
		if (strict_strtoul(this_opt+4, 0, 0) == 0) {
			sprintf(s, "DMA end address enable: disabled\n");
			inf->ctrl_info->cr1 = (info->reg_cr1 & ~DB9000_CR1_DEE);
		} else {
			sprintf(s, "DMA end address enable: enabled\n");
			inf->ctrl_info->cr1 = (info->reg_cr1 | DB9000_CR1_DEE);
		}
	} else if (!strncmp(this_opt, "dear:", 5)) {
		inf->ctrl_info->dear = strict_strtoul(this_opt+5, 0, 0);
		sprintf(s, "DMA end address offset: %d\n",
			strict_strtoul(this_opt+4, 0, 0));
	} else if (!strncmp(this_opt, "lps:", 4)) {
		if (strict_strtoul(this_opt+4, 0, 0) == 0) {
			sprintf(s, "Selected Single port output mode to LCD\n");
			inf->ctrl_info->cr1 = (info->reg_cr1 & ~DB9000_CR1_LPS);
		} else {
			sprintf(s, "Selected Dual Output port mode to LCD\n");
			inf->ctrl_info->cr1 = (info->reg_cr1 | DB9000_CR1_LPS);
		}
	} else if (!strncmp(this_opt, "fdw:", 4)) {
		inf->ctrl_info->cr1 = (info->reg_cr1 & ~DB9000_CR1_FDW(3));
		inf->ctrl_info->cr1 |=
			DB9000_CR1_FDW(strict_strtoul(this_opt+4, 0, 0));
		sprintf(s, "Fifo DMA Words setting: %d\n",
				strict_strtoul(this_opt+4, 0, 0));
	} else if (!strncmp(this_opt, "pss:", 4)) {
		if (strict_strtoul(this_opt+4, 0, 0) == 0) {
			sprintf(s, "Palette load source from internal \
					registers\n");
			inf->ctrl_info->cr1 = (info->reg_cr1 & ~DB9000_CR1_PSS);
		} else {
			sprintf(s, "Palette load source from frame buffer\n");
			inf->ctrl_info->cr1 = (info->reg_cr1 | DB9000_CR1_PSS);
		}
	} else if (!strncmp(this_opt, "ops:", 4)) {
		inf->ctrl_info->cr1 = (info->reg_cr1 & ~DB9000_CR1_OPS(7));
		inf->ctrl_info->cr1 |=
			(DB9000_CR1_OPS(strict_strtoul(this_opt+4, 0, 0))
				& DB9000_CR1_OPS(3));
		sprintf(s, "Output pixel select: %d\n",
				strict_strtoul(this_opt+4, 0, 0));
	} else if (!strncmp(this_opt, "rgb:", 4)) {
		if (strict_strtoul(this_opt+4, 0, 0) == 0) {
			sprintf(s, "RGB mode set to: RGB\n");
			inf->ctrl_info->cr1 = (info->reg_cr1 & ~DB9000_CR1_RGB);
		} else {
			sprintf(s, "RGB mode set to BGR\n");
			inf->ctrl_info->cr1 = (info->reg_cr1 | DB9000_CR1_RGB);
		}
	} else if (!strncmp(this_opt, "epo:", 4)) {
		if (strict_strtoul(this_opt+4, 0, 0) == 0) {
			sprintf(s, "Pixel endianness set to little\n");
			inf->ctrl_info->cr1 = (info->reg_cr1 & ~DB9000_CR1_EPO);
		} else {
			sprintf(s, "Pixel endianness set to big\n");
			inf->ctrl_info->cr1 = (info->reg_cr1 | DB9000_CR1_EPO);
		}
	} else if (!strncmp(this_opt, "ebo:", 4)) {
		if (strict_strtoul(this_opt+4, 0, 0) == 0) {
			sprintf(s, "Frame buffer byte endianness set to \
					little\n");
			inf->ctrl_info->cr1 = (info->reg_cr1 & ~DB9000_CR1_EBO);
		} else {
			sprintf(s, "Frame buffer byte endianness set to big\n");
			inf->ctrl_info->cr1 = (info->reg_cr1 | DB9000_CR1_EBO);
		}
	} else if (!strncmp(this_opt, "pci:", 4)) {
		if (strict_strtoul(this_opt+4, 0, 0) == 0) {
			sprintf(s, "Pixel Clock Input Select: Master bus \
					clock or Pixel clock divider\n");
			inf->ctrl_info->pctr =
				(info->reg_pctr & ~DB9000_PCTR_PCI);
		} else {
			sprintf(s, "Pixel Clock Input Select: PCLK_IN\n");
			inf->ctrl_info->pctr =
				(info->reg_pctr | DB9000_PCTR_PCI);
		}
	} else if (!strncmp(this_opt, "pcb:", 4)) {
		if (strict_strtoul(this_opt+4, 0, 0) == 0) {
			sprintf(s, "Pixel Clock comes from pixel clock \
					divider\n");
			inf->ctrl_info->pctr =
				(info->reg_pctr & ~DB9000_PCTR_PCB);
		} else {
			sprintf(s, "Pixel clock comes from bus clock\n");
			inf->ctrl_info->pctr =
				(info->reg_pctr & ~DB9000_PCTR_PCB);
		}
	} else if (!strncmp(this_opt, "pcd:", 4)) {
		inf->ctrl_info->pctr = (info->reg_pctr & ~DB9000_PCTR_PCD(255));
		inf->ctrl_info->pctr |=
			DB9000_PCTR_PCD(strict_strtoul(this_opt+4, 0, 0));
		sprintf(s, "Pixel clock divider: %d\n",
				strict_strtoul(this_opt+4, 0, 0));
	} else {
		dev_err(dev, "unknown option: %s\n", this_opt);
		return -EINVAL;
	}
	if (s[0] != '\0')
		dev_info(dev, "override %s", s);
	return 0;
}

static int __devinit db9000fb_parse_options(struct device *dev, char *options)
{
	char *this_opt;
	int ret;

	if (!options || !*options)
		return 0;

	dev_dbg(dev, "options are \"%s\"\n", options ? options : "null");

	/* could be made table driven or similar?... */
	while ((this_opt = strsep(&options, ",")) != NULL) {
		ret = parse_opt(dev, this_opt);
		if (ret)
			return ret;
	}
	return 0;
}

static char g_options[256] __devinitdata = "";

#ifndef MODULE
static int __init db9000fb_setup_options(void)
{
	char *options = NULL;

	if (fb_get_options("db9000fb", &options))
		return -ENODEV;

	if (options)
		strlcpy(g_options, options, sizeof(g_options));

	return 0;
}
#else
#define db9000fb_setup_options()		(0)

module_param_string(options, g_options, sizeof(g_options), 0);
MODULE_PARM_DESC(options, "LCD parameters (see Documentation/fb/db9000fb.txt)");
#endif

#else
#define db9000fb_parse_options(...)	(0)
#define db9000fb_setup_options()		(0)
#endif

#ifdef DEBUG_VAR
/*
 * Check for various illegal bit-combinations. Currently only a warning is
 * given.
 */
static void __devinit db9000fb_check_options(struct device *dev,
		struct db9000fb_mach_info *inf)
{
	if (inf->ctrl_info->cr1 & CR1_INVALID_CONFIG_MASK)
		dev_warn(dev, "machine CR1 setting contains illegal bits: \
				%08x\n", inf->ctrl_info->cr1 &
				CR1_INVALID_CONFIG_MASK);

	if (inf->ctrl_info->cr1 & DB9000_CR1_OPS(4))
		dev_warn(dev, "CR1 OPS[2] bit set illegally: %08x\n",
				inf->ctrl_info->cr1);

	if ((inf->ctrl_info->cr1 & DB9000_CR1_FDW(3)) == DB9000_CR1_FDW(3))
		dev_warn(dev, "CR1 Fifo DMA Words field illegal value: %08x\n",
				inf->ctrl_info->cr1 & DB9000_CR1_FDW(3));

	switch (inf->ctrl_info->bpp) {
	case 1:
	case 2:
	case 4:
	case 8:
	case 16:
	case 18:
	case 24:
	case 32:
		break;
	default:
		dev_warn(dev, "BPP setting illegal: %d\n", inf->ctrl_info->bpp);
	}

	if (inf->modes->xres < 16 || inf->modes->xres > 4096 || (inf->modes->yres % 16))
		dev_warn(dev, "Horizontal resolution out of range: %d\n",
				inf->modes->xres);

	if ((inf->modes->yres > 4096) || (inf->modes->yres < 0))
		dev_warn(dev, "Vertical resolution out of range: %d\n",
				inf->modes->yres);

	if (inf->modes->left_margin > 1023)
		dev_warn(dev, "machine Horizontal Back Port setting out of \
				range: %d\n", inf->modes->left_margin);

	if (inf->modes->right_margin > 1023)
		dev_warn(dev, "machine Horizontal Front Port setting out of \
				range: %d\n", inf->modes->left_margin);
}
#endif

static int __devinit db9000fb_probe(struct platform_device *pdev)
{
	struct db9000fb_info *fbi = NULL;
	struct db9000fb_mach_info *inf = dev_get_platdata(&pdev->dev);
	struct resource *r;
	char __iomem *addr;
	int irq;
	int ret = -EBUSY;

	if (!inf)
		return ret;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r)
		return -EINVAL;

	if (!devm_request_mem_region(&pdev->dev, r->start, resource_size(r),
				DRIVER_NAME)) {
		dev_warn(&pdev->dev, "failed to get memory region resource\n");
		return -ENOENT;
	}

	ret = db9000fb_parse_options(&pdev->dev, g_options);
	if (ret < 0)
		return ret;

#ifdef DEBUG_VAR
	db9000fb_check_options(&pdev->dev, inf);
#endif

	fbi = db9000fb_init_fbinfo(&pdev->dev);
	if (!fbi) {
		/* only reason for db9000fb_init_fbinfo to fail is kmalloc */
		dev_err(&pdev->dev, "Failed to init framebuffer device\n");
		return -ENOMEM;
	}

	fbi->mmio_base = ioremap(r->start, resource_size(r));
	if (!fbi->mmio_base) {
		dev_err(&pdev->dev, "failed to map I/O memory\n");
		ret = -ENOMEM;
		goto err_free_fbi;
	}

	if (!mode_option)
		mode_option = inf->def_mode;

	ret = fb_find_mode(&fbi->fb.var, &fbi->fb, mode_option,
			inf->modes, inf->num_modes, NULL,
			inf->ctrl_info->bpp);

	if (ret > 2)
		ret = fb_find_mode(&fbi->fb.var, &fbi->fb, mode_option,
				NULL, 0, NULL, inf->ctrl_info->bpp);

	if (!ret)
		pr_err("unsupprted resolution format");

	dev_info(&pdev->dev, "got a %dx%dx%d LCD\n", fbi->fb.var.xres,
			fbi->fb.var.yres, inf->ctrl_info->bpp);

	if ((inf->ctrl_info->bpp == 24) &&
			((inf->ctrl_info->cr1 & DB9000_CR1_FBP) == 0))
		inf->ctrl_info->bpp = 32;

	if (inf->ctrl_info->bpp < 16)
		fbi->video_mem_size += PALETTE_SIZE;

	/* Initialize video memory */
	addr = ioremap(fbi->frame_base, fbi->video_mem_size);
	if (!addr) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		ret = -ENOMEM;
		goto err_free_mmio_base;
	}

	fbi->fb.screen_base	= addr;
	fbi->fb.fix.smem_start	= fbi->frame_base;
	fbi->fb.fix.smem_len	= fbi->video_mem_size;
	fbi->fb.var.height	= fbi->fb.var.yres;
	fbi->fb.var.width	= fbi->fb.var.xres;
	fbi->setup_gpio		= inf->clcd_mux_selection;

	/* Clear the screen */
	memset((char *)fbi->fb.screen_base, 0, fbi->fb.fix.smem_len);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no IRQ defined\n");
		ret = irq;
		goto err_free_framebuffer_addr;
	}

	ret = request_irq(irq, db9000fb_handle_irq, IRQF_DISABLED, "LCD", fbi);
	if (ret) {
		dev_err(&pdev->dev, "request_irq failed: %d\n", ret);
		goto err_free_framebuffer_addr;
	}

#ifdef CONFIG_BACKLIGHT_DB9000_LCD
	fbi->reg_pwmdcr = DEF_BRIGHTNESS;
	fbi->pdev = pdev;
	init_backlight(fbi);
#endif

	ret = db9000fb_check_var(&fbi->fb.var, &fbi->fb);
	if (ret) {
		dev_err(&pdev->dev, "failed to get suitable mode\n");
		goto err_free_irq;
	}

	if (fbi->palette_mode == PAL_IN_FB)
		fbi->video_mem_size += (fbi->palette_size * 2);

	platform_set_drvdata(pdev, fbi);

	ret = register_framebuffer(&fbi->fb);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Failed to register framebuffer device:%d\n", ret);
		goto err_clear_plat_data;
	}

#ifdef CONFIG_CPU_FREQ
	fbi->freq_transition.notifier_call = db9000fb_freq_transition;
	fbi->freq_policy.notifier_call = db9000fb_freq_policy;
	cpufreq_register_notifier(&fbi->freq_transition,
			CPUFREQ_TRANSITION_NOTIFIER);
	cpufreq_register_notifier(&fbi->freq_policy,
			CPUFREQ_POLICY_NOTIFIER);
#endif

	/* Read the core version register and print it out */
	fbi->db9000_rev = lcd_readl(fbi, DB9000_CIR);
	dev_info(&pdev->dev, "%s: Core ID reg: 0x%08X\n",
			__func__, fbi->db9000_rev);

	ret = db9000fb_set_par(&fbi->fb);
	if (ret) {
		dev_err(&pdev->dev, "Failed to set parameters\n");
		goto err_clear_plat_data;
	}

#ifdef CONFIG_FB_DB9000_DRM
	ump_memory_description.addr = fbi->fb.fix.smem_start;
	ump_memory_description.size = ((fbi->fb.fix.smem_len / 4096) + 1)* 4096;
#endif /* CONFIG_FB_DB9000_DRM */

	fbi->ignore_cpufreq_notification = inf->ignore_cpufreq_notification;

	return 0;

err_clear_plat_data:
	platform_set_drvdata(pdev, NULL);
	if (fbi->fb.cmap.len)
		fb_dealloc_cmap(&fbi->fb.cmap);
err_free_irq:
	free_irq(irq, fbi);
err_free_framebuffer_addr:
#ifdef CONFIG_BACKLIGHT_DB9000_LCD
	exit_backlight(fbi);
#endif
	iounmap(addr);
err_free_mmio_base:
	iounmap(fbi->mmio_base);
err_free_fbi:
	kfree(fbi);
	clk_put(fbi->clk);
	release_mem_region(r->start, resource_size(r));

	return ret;
}

static int __devexit db9000fb_remove(struct platform_device *pdev)
{
	struct db9000fb_info *fbi = platform_get_drvdata(pdev);

	struct resource *r;
	int irq;
	struct fb_info *info;

	if (!fbi)
		return 0;

	info = &fbi->fb;

	unregister_framebuffer(info);
	db9000fb_setup_gpio(fbi, false);
	db9000fb_disable_controller(fbi);
	complete_and_exit(&fbi->vsync_notifier, 0);

	if (fbi->fb.cmap.len)
		fb_dealloc_cmap(&fbi->fb.cmap);

	irq = platform_get_irq(pdev, 0);
	free_irq(irq, fbi);
#ifdef CONFIG_BACKLIGHT_DB9000_LCD
	exit_backlight(fbi);
#endif
	iounmap(fbi->fb.screen_base);
	iounmap(fbi->mmio_base);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(r->start, resource_size(r));

	clk_put(fbi->clk);
	kfree(fbi);

	return 0;
}

static struct platform_driver db9000fb_driver = {
	.probe		= db9000fb_probe,
	.remove		= db9000fb_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= DRIVER_NAME,
#ifdef CONFIG_PM
		.pm = &db9000fb_pm_ops,
#endif
	},
};

#ifndef MODULE
static int __init db9000fb_setup(char *options)
{
	char *this_opt;

	/* Parse user speficied options (`video=db9000:') */
	if (!options || !*options)
		return 0;

	while ((this_opt = strsep(&options, ",")) != NULL) {
		if (!*this_opt)
			continue;
		else
			mode_option = this_opt;
	}
	return 0;
}
#endif

static int __init db9000fb_init(void)
{
	/* For kernel boot options (in 'video=pm3fb:<options>' format) */
#ifndef MODULE
	char *option = NULL;

	if (fb_get_options("db9000", &option))
		return -ENODEV;
	db9000fb_setup(option);
#endif
	if (db9000fb_setup_options())
		return -EINVAL;

	return platform_driver_register(&db9000fb_driver);
}
module_init(db9000fb_init);

static void __exit db9000fb_exit(void)
{
	platform_driver_unregister(&db9000fb_driver);
}
module_exit(db9000fb_exit);

MODULE_DESCRIPTION("loadable framebuffer driver for Digital Blocks DB9000");
MODULE_LICENSE("GPL");
