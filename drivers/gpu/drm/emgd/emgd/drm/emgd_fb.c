/* -*- pse-c -*-
 *-----------------------------------------------------------------------------
 * Filename: emgd_fb.c
 * $Revision: 1.8 $
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
 *  Framebuffer / kenrel mode setting functions.
 *-----------------------------------------------------------------------------
 */
#include "drm_emgd_private.h"
#include "emgd_drv.h"
#include "emgd_drm.h"
#include "memory.h"
/* #include <drmP.h> */
/* #include <drm.h> */
/* #include <drm_crtc.h> */
#include <drm_crtc_helper.h>
#include <linux/version.h>


/*
 * Move this to emgd_drv.h?
 */
typedef struct _emgd_framebuffer {
	struct drm_framebuffer base;
	unsigned long size;
	unsigned long offset;
} emgd_framebuffer_t;

typedef struct _emgdfb_par {
	struct drm_device *dev;
	emgd_framebuffer_t *emgd_fb;
} emgdfb_par_t;


typedef struct _emgd_crtc {
	struct drm_crtc base;
	emgd_framebuffer_t *fbdev_fb;
	struct drm_mode_set mode_set;
	struct drm_display_mode saved_mode;
	struct drm_display_mode saved_adjusted_mode;
	unsigned char lut_r[256];
	unsigned char lut_g[256];
	unsigned char lut_b[256];
	unsigned char lut_a[256];
} emgd_crtc_t;

#if  (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
#define RETURN_PROBE_TYPE int
#define RETURN_PROBE return 0
#define PROBE_FUNC fb_changed
#else
#define RETURN_PROBE_TYPE void
#define RETURN_PROBE
#define PROBE_FUNC output_poll_changed
#endif


static struct drm_framebuffer *emgd_fb_create(struct drm_device *dev,
		struct drm_file *filp, struct drm_mode_fb_cmd *r);

RETURN_PROBE_TYPE  emgd_fb_probe(struct drm_device *dev);
static void emgd_fb_destroy (struct drm_framebuffer *fb);
static int emgd_fb_create_handle(struct drm_framebuffer *fb,
		struct drm_file *file_priv, unsigned int *handle);
static int emgd_fb_check_var(struct fb_var_screeninfo *var,
		struct fb_info *info);
static int emgd_fb_set_par(struct fb_info *info);
static int emgd_fb_setcolreg(unsigned int regno,
		unsigned int red, unsigned int green, unsigned int blue,
		unsigned int transp, struct fb_info *info);
static int emgd_fb_pan_display(struct fb_var_screeninfo *var,
		struct fb_info *info);
static int emgd_fb_blank(int blank, struct fb_info *info);

static const struct drm_mode_config_funcs emgd_mode_funcs = {
	.fb_create = emgd_fb_create,
	.PROBE_FUNC = emgd_fb_probe,
};

static const struct drm_framebuffer_funcs emgd_fb_funcs = {
	.destroy = emgd_fb_destroy,
	.create_handle = emgd_fb_create_handle,
};

static const struct fb_ops emgd_fb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = emgd_fb_check_var,
	.fb_set_par = emgd_fb_set_par,
	.fb_setcolreg = emgd_fb_setcolreg,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_mmap = NULL,
	.fb_pan_display = emgd_fb_pan_display,
	.fb_blank = emgd_fb_blank,
};


static void emgd_crtc_dpms(struct drm_crtc *crtc, int mode);
static bool emgd_crtc_mode_fixup(struct drm_crtc *crtc,
		struct drm_display_mode *mode, struct drm_display_mode *adjusted_mode);
static int emgd_crtc_mode_set(struct drm_crtc *crtc,
		struct drm_display_mode *mode, struct drm_display_mode *adjusted_mode,
		int x, int y, struct drm_framebuffer *old_fb);
static int emgd_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
		struct drm_framebuffer *old_fb);
static void emgd_crtc_prepare(struct drm_crtc *crtc);
static void emgd_crtc_commit(struct drm_crtc *crtc);

static void emgd_crtc_save(struct drm_crtc *crtc);
static void emgd_crtc_restore(struct drm_crtc *crtc);
static int emgd_crtc_cursor_set(struct drm_crtc *crtc,
		struct drm_file *file_priv, uint32_t handle,
		uint32_t width, uint32_t height);
static int emgd_crtc_cursor_move(struct drm_crtc *crtc, int x, int y);
static void emgd_crtc_gamma_set(struct drm_crtc *crtc,
		unsigned short *red, unsigned short *green, unsigned short *blue,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
		uint32_t start,
#endif
		uint32_t size);
static void emgd_crtc_destroy(struct drm_crtc *crtc);



static const struct drm_crtc_helper_funcs emgd_helper_funcs = {
	.dpms = emgd_crtc_dpms,
	.mode_fixup = emgd_crtc_mode_fixup,
	.mode_set = emgd_crtc_mode_set,
	.mode_set_base = emgd_crtc_mode_set_base,
	.prepare = emgd_crtc_prepare,
	.commit = emgd_crtc_commit,
};

static const struct drm_crtc_funcs emgd_crtc_funcs = {
	.save = emgd_crtc_save,
	.restore = emgd_crtc_restore,
	.cursor_set = emgd_crtc_cursor_set,
	.cursor_move = emgd_crtc_cursor_move,
	.gamma_set = emgd_crtc_gamma_set,
	.set_config = drm_crtc_helper_set_config,
	.destroy = emgd_crtc_destroy,
};





static struct drm_mode_set panic_mode;

/*
 * emgd_fb_restore
 *
 * Restore the kernel's fbcon mode.
 */
void emgd_fb_restore(void)
{
	int ret;

	/*
	 * FIXME: Need to have the real crtc saved so it can be restored.
	 */
	if ((ret = drm_crtc_helper_set_config(&panic_mode)) != 0) {
		printk(KERN_ERR "Failed to restore crtc configuration: %d\n", ret);
	}
}


/*
 * Called if something fails while trying to set up framebuffer based
 * console.
 */
static int emgd_fb_panic(struct notifier_block *n,
		unsigned long res,
		void *panic_str)
{
	printk(KERN_ALERT "Panic occurred, switch back to text console.");

	emgd_fb_restore();

	return 0;
}


static struct notifier_block paniced = {
	.notifier_call = emgd_fb_panic,
};



/*
 * emgd_ms_init
 *
 * This is the main initialization entry point.  Called during driver load
 * and does basic setup.
 */
void emgd_ms_init(struct drm_device *dev)
{
	emgd_crtc_t *emgd_crtc;
	unsigned short *r, *g, *b;
	int i;

	drm_mode_config_init(dev);  /* drm helper function */

	dev->mode_config.min_width = 0;
	dev->mode_config.max_width = 2048;
	dev->mode_config.min_height = 0;
	dev->mode_config.max_height = 2048;
	dev->mode_config.funcs = (void *)&emgd_mode_funcs;
	dev->mode_config.fb_base = pci_resource_start(dev->pdev, 2);

	emgd_crtc = kzalloc(sizeof(emgd_crtc_t) +
			(INTELFB_CONN_LIMIT * sizeof(struct drm_connector *)),
			GFP_KERNEL);

	if (emgd_crtc == NULL) {
		printk(KERN_ERR "emgd_ms_init: Failed to allocate CRTC structure.\n");
		return;
	}

	drm_crtc_init(dev, &emgd_crtc->base, &emgd_crtc_funcs);
	drm_mode_crtc_set_gamma_size(&emgd_crtc->base, 256);

	/* Set initial gamma values */
	r = emgd_crtc->base.gamma_store;
	g = emgd_crtc->base.gamma_store + 256;
	b = emgd_crtc->base.gamma_store + 512;
	for (i = 0; i < 256; i++) {
		emgd_crtc->lut_r[i] = i;
		emgd_crtc->lut_g[i] = i;
		emgd_crtc->lut_b[i] = i;
		emgd_crtc->lut_a[i] = 0;
		r[i] = i << 8;
		g[i] = i << 8;
		b[i] = i << 8;
	}

	drm_crtc_helper_add(&emgd_crtc->base, &emgd_helper_funcs);

	/* TODO: Create connector list */


	/* TODO: Is there more this needs to do? */
	/*
	 * i915 driver does the following:
	 *       Q? what is base?
	 *   drm_crtc_init(dev, base, crtc_funcs);
	 *   drm_mode_crtc_set_gamma_size(base, 256);
	 *   drm_crtc_helper_add(base, helper_funcs);
	 *   Builds dev->mode_config.connector_list
	 *
	 *   crtc_funcs:
	 *		cursor_set  - probabl a simple wrapper around alter_cursor
	 *		cursor_move - probabl a simple wrapper around alter_cursor
	 *		gamma_set   - set palette ?
	 *		set_config  - drm_crtc_helper_set_config
	 *		destroy     - clean up
	 *
	 *   drm_crtc_helper_funcs
	 *		dpms         - power management
	 *		mode_fixup   - ???
	 *		mode_set     - wrapper around alter_displays ?
	 *		mode_set_base- noop (handled in alter_displays) ?
	 *		prepare      - DPMS off
	 *		commit       - DPMS on
	 *
	 * This is a lot of the code that impliments KMS.
	 */

}


/*
 * emgd_fb_probe
 *
 * Handle changes to the framebuffer.
 */
RETURN_PROBE_TYPE emgd_fb_probe(struct drm_device *drm)
{

	/* FIXME: Need to set panic_mode to the a valid mode */

	/* Register a notifier to switch back to kernel console on panic */
	atomic_notifier_chain_register(&panic_notifier_list, &paniced);

	RETURN_PROBE;
}
EXPORT_SYMBOL(emgd_fb_probe);


/*
 * emgd_fb_remove
 *
 * clean up everything we've done to create the framebuffer instance.
 */
int emgd_fb_remove(struct drm_device *dev, struct drm_framebuffer *fb)
{
	struct fb_info *info;
	drm_emgd_private *dev_priv = dev->dev_private;

	info = dev_priv->fbdev;

	if (info) {
		unregister_framebuffer(info);
		framebuffer_release(info);
	}

	/* Remove the notifier that checks for failed framebuffers */
	atomic_notifier_chain_unregister(&panic_notifier_list, &paniced);
	memset(&panic_mode, 0, sizeof(struct drm_mode_set));

	return 0;
}
EXPORT_SYMBOL(emgd_fb_remove);


/*
 * emgd_fb_create
 */
static struct drm_framebuffer *emgd_fb_create(struct drm_device *dev,
		struct drm_file *filp,
		struct drm_mode_fb_cmd *r)
{
	int ret;
	struct fb_info *info;
	emgdfb_par_t *par;
	emgd_framebuffer_t *emgd_fb;
	drm_emgd_private *dev_priv;

	emgd_fb = kzalloc(sizeof(emgd_framebuffer_t), GFP_KERNEL);

	/*
	 * TODO: What is drm_mode_fb_cmd?  UMG gets some kind of memory
	 *       block based on r->handle (psb_get_meminfo_by_handle())
	 */

	/* Create a framebuffer instance */
	ret = drm_framebuffer_init(dev, &emgd_fb->base, &emgd_fb_funcs);
	if (ret) {
		printk(KERN_ERR "Failed to create framebuffer instance.\n");
		return NULL;
	}

	drm_helper_mode_fill_fb_struct(&emgd_fb->base, r);

	info = framebuffer_alloc(sizeof(emgdfb_par_t), &dev->pdev->dev);
	if (info == NULL) {
		printk(KERN_ERR "Failed to allocate framebuffer info.\n");
		return NULL;
	}

	par = info->par;

	/* Fill in fb info structure */
	strcpy(info->fix.id, "emgdfb");
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_TRUECOLOR;
	info->fix.type_aux = 0;
	info->fix.xpanstep = 1;
	info->fix.ypanstep = 1;
	info->fix.ywrapstep = 0;
	info->fix.accel = FB_ACCEL_I830;
	info->fix.line_length = emgd_fb->base.pitch;
	info->fix.smem_start = dev->mode_config.fb_base;
	info->fix.smem_len = emgd_fb->size;
	info->fix.mmio_start = pci_resource_start(dev->pdev, 0);
	info->fix.mmio_len = pci_resource_len(dev->pdev, 0);

	info->flags = FBINFO_DEFAULT;
	info->fbops = (struct fb_ops*)&emgd_fb_ops;
	info->screen_base = 0;  /* FIXME: This is kernel memory address */
	info->screen_size = emgd_fb->size;
#if  (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
	info->pseudo_palette = emgd_fb->base.pseudo_palette;
#endif
	info->var.xres_virtual = emgd_fb->base.width;
	info->var.yres_virtual = emgd_fb->base.height;
	info->var.bits_per_pixel = emgd_fb->base.bits_per_pixel;
	info->var.xoffset = 0;
	info->var.yoffset = 0;
	info->var.activate = FB_ACTIVATE_NOW;
	info->var.width = -1;
	info->var.height = -1;
	info->var.xres = r->width;
	info->var.yres = r->height;

	info->pixmap.size = 64 * 1024;
	info->pixmap.buf_align = 8;
	info->pixmap.access_align = 32;
	info->pixmap.flags = FB_PIXMAP_SYSTEM;
	info->pixmap.scan_align = 1;

	switch (emgd_fb->base.depth) {
		case 8:
			info->var.red.offset = 0;
			info->var.green.offset = 0;
			info->var.blue.offset = 0;
			info->var.red.length = 8;
			info->var.green.length = 8;
			info->var.blue.length = 8;
			info->var.transp.offset = 0;
			info->var.transp.length = 0;
			break;
		case 15:
			info->var.red.offset = 10;
			info->var.green.offset = 5;
			info->var.blue.offset = 0;
			info->var.red.length = 5;
			info->var.green.length = 5;
			info->var.blue.length = 5;
			info->var.transp.offset = 15;
			info->var.transp.length = 1;
			break;
		case 16:
			info->var.red.offset = 11;
			info->var.green.offset = 5;
			info->var.blue.offset = 0;
			info->var.red.length = 5;
			info->var.green.length = 6;
			info->var.blue.length = 5;
			info->var.transp.offset = 0;
			info->var.transp.length = 0;
			break;
		case 24:
			info->var.red.offset = 16;
			info->var.green.offset = 8;
			info->var.blue.offset = 0;
			info->var.red.length = 8;
			info->var.green.length = 8;
			info->var.blue.length = 8;
			info->var.transp.offset = 0;
			info->var.transp.length = 0;
			break;
		case 32:
			info->var.red.offset = 16;
			info->var.green.offset = 8;
			info->var.blue.offset = 0;
			info->var.red.length = 8;
			info->var.green.length = 8;
			info->var.blue.length = 8;
			info->var.transp.offset = 24;
			info->var.transp.length = 8;
			break;
		default:
			break;
	}

	register_framebuffer(info);

	dev_priv = dev->dev_private;
	dev_priv->fbdev = info;

	par->emgd_fb = emgd_fb;
	par->dev = dev;

	return &(emgd_fb->base);
}



/*
 * emgd_fb_destroy
 *
 * clean up and remove a framebuffer instance.
 */
static void emgd_fb_destroy (struct drm_framebuffer *fb)
{
	struct drm_device *dev = fb->dev;
	drm_emgd_private *dev_priv = dev->dev_private;

	/* TODO: Unmap any pages mapped to the GTT */

	if (dev_priv->fbdev) {
		emgd_fb_remove(dev, fb);
	}

	drm_framebuffer_cleanup(fb);

	kfree(fb);
}


/*
 * emgd_fb_create_handle
 *
 * TODO: Figure out what this is suppose to do. The intel driver creates
 * a handle to a GEM object.  The UMG driver does nothing.
 */
static int emgd_fb_create_handle(struct drm_framebuffer *fb,
		struct drm_file *file_priv,
		unsigned int *handle)
{
	(void)file_priv;
	*handle = 0;

	return 0;
}


static int emgd_fb_check_var(struct fb_var_screeninfo *var,
		struct fb_info *info)
{
	printk(KERN_ALERT "STUBED emgd_fb_check_var\n");
	return 0;
}

static int emgd_fb_set_par(struct fb_info *info)
{
	printk(KERN_ALERT "STUBED emgd_fb_set_par\n");
	return 0;
}

static int emgd_fb_setcolreg(unsigned int regno,
		unsigned int red, unsigned int green, unsigned int blue,
		unsigned int transp, struct fb_info *info)
{
	printk(KERN_ALERT "STUBED emgd_fb_setcolreg\n");
	return 0;
}

static int emgd_fb_pan_display(struct fb_var_screeninfo *var,
		struct fb_info *info)
{
	printk(KERN_ALERT "STUBED emgd_fb_pan_display\n");
	return 0;
}

static int emgd_fb_blank(int blank, struct fb_info *info)
{
	switch(blank) {
	case FB_BLANK_UNBLANK:
		printk(KERN_ALERT "Should call turn on display\n");
		break;
	case FB_BLANK_NORMAL:
		printk(KERN_ALERT "Should call standby\n");
		break;
	case FB_BLANK_HSYNC_SUSPEND:
		printk(KERN_ALERT "Should call standby\n");
		break;
	case FB_BLANK_VSYNC_SUSPEND:
		printk(KERN_ALERT "Should call suspend\n");
		break;
	case FB_BLANK_POWERDOWN:
		printk(KERN_ALERT "Should call DRM_MODE_DPMS_OFF\n");
		break;
	}

	return 0;
}



/****************************************************************************
 * CRTC functions
 ****************************************************************************/

static void emgd_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	printk(KERN_ALERT "emgd_crtc_dpms: STUB\n");
}

static bool emgd_crtc_mode_fixup(struct drm_crtc *crtc,
		struct drm_display_mode *mode, struct drm_display_mode *adjusted_mode)
{
	printk(KERN_ALERT "emgd_crtc_mode_fixup: STUB\n");
	return 0;
}

static int emgd_crtc_mode_set(struct drm_crtc *crtc,
		struct drm_display_mode *mode, struct drm_display_mode *adjusted_mode,
		int x, int y, struct drm_framebuffer *old_fb)
{
	printk(KERN_ALERT "emgd_crtc_mode_set: STUB\n");
	return 0;
}

static int emgd_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
		struct drm_framebuffer *old_fb)
{
	printk(KERN_ALERT "emgd_pipe_set_base: STUB\n");
	return 0;
}

static void emgd_crtc_prepare(struct drm_crtc *crtc)
{
	printk(KERN_ALERT "emgd_crtc_prepare: STUB\n");
}

static void emgd_crtc_commit(struct drm_crtc *crtc)
{
	printk(KERN_ALERT "emgd_crtc_commit: STUB\n");
}


static void emgd_crtc_save(struct drm_crtc *crtc)
{
	printk(KERN_ALERT "emgd_crtc_save: STUB\n");
}

static void emgd_crtc_restore(struct drm_crtc *crtc)
{
	printk(KERN_ALERT "emgd_crtc_restore: STUB\n");
}

static int emgd_crtc_cursor_set(struct drm_crtc *crtc,
		struct drm_file *file_priv, uint32_t handle,
		uint32_t width, uint32_t height)
{
	printk(KERN_ALERT "emgd_crtc_cursor_set: STUB\n");
	return 0;
}

static int emgd_crtc_cursor_move(struct drm_crtc *crtc, int x, int y)
{
	printk(KERN_ALERT "emgd_crtc_cursor_move: STUB\n");
	return 0;
}

static void emgd_crtc_gamma_set(struct drm_crtc *crtc,
		unsigned short *red, unsigned short *green, unsigned short *blue,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
		uint32_t start,
#endif
		uint32_t size)
{
	printk(KERN_ALERT "emgd_crtc_gamma_set: STUB\n");
}

static void emgd_crtc_destroy(struct drm_crtc *crtc)
{
	printk(KERN_ALERT "emgd_crtc_destroy: STUB\n");
}
