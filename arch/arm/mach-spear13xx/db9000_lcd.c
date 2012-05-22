/*
* arch/arm/mach-spear13xx/db9000_lcd.c
* Copyright (C) 2010 Digital Blocks
*
* Based on arch/arm/mach-pxa/devices.c
*
* This file is licensed under the terms of the GNU General Public License
* version 2. This program is licensed "as is" without any warranty of any kind,
* whether express or implied.
*/

#include <linux/amba/bus.h>
#include <linux/clk.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <video/db9000fb.h>

#include <asm/setup.h>
#include <plat/clock.h>
#include <mach/generic.h>
#include <mach/hardware.h>

static struct fb_videomode def_modelist[] = {
	{
		"480x272-32@0", 0, 480, 272, 111000, 2, 2, 2, 2, 40, 10, 0,
		FB_VMODE_NONINTERLACED
	}, {
		"800x480-32@0", 0, 800, 480, 33333, 40, 40, 29, 13, 48, 3, 0,
		FB_VMODE_NONINTERLACED
	}, {
		"1024x768-32@60", 60, 1024, 768, 15384, 160, 24, 29, 3, 136, 6,
		0, FB_VMODE_NONINTERLACED
	}, {
		"1280x720-32@60", 60, 1280, 720, 13468, 220, 110, 20, 5, 40, 5,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_NONINTERLACED
	}, {
		"1920x540-32@60", 60, 1920, 540, 13468, 148, 88, 15, 2, 44, 5,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_NONINTERLACED
	}, {
		"1920x1080-32@60", 60, 1920, 1080, 6734, 148, 88, 36, 4, 44, 5,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_NONINTERLACED
	},
};

static struct db9000fb_ctrl_info ctrl_info = {
	.bpp = 32,
	.cr1 = DB9000_CR1_EBO | DB9000_CR1_DEP | DB9000_CR1_HSP |
		DB9000_CR1_VSP | DB9000_CR1_OPS(1) | DB9000_CR1_FDW(2),
	.pwmfr = DB9000_PWMFR_PWM_FCD(24),
	.pctr = DB9000_PCTR_PCI,
	.dear = 0,
};

static struct db9000fb_mach_info clcd_plat_info = {
	.modes		= def_modelist,
	.num_modes	= ARRAY_SIZE(def_modelist),
	.ctrl_info	= &ctrl_info,
	.lcd_conn	= LCD_PCLK_EDGE_FALL,
	.mem_size	= 0,
	.cmap_static	= 0,
	.cmap_inverse	= 0,
};

static void clcd_set_plat_data(struct platform_device *pdev,
		struct db9000fb_mach_info *data)
{
	struct clk *clcd_pclk, *fb_clk, *ah_clk;
	int ret;

	pdev->dev.platform_data = data;

	if (cpu_is_spear1340())
		data->clcd_mux_selection = &config_clcd_gpio_pads;

	ret = clk_set_parent_sys(NULL, "clcd_synth_clk", NULL, "vco1div4_clk");
	if (ret) {
		pr_err("%s:failed to set vco1div4 as parent of clcd_synth\n",
				__func__);
		return ;
	}

	ret = clk_set_parent_sys("clcd_pixel_clk", NULL, NULL,
			"clcd_synth_clk");
	if (ret) {
		pr_err("%s:failed to set clcd_synth as parent of pixel clk\n",
				__func__);
		return ;
	}

	clcd_pclk = clk_get_sys("clcd_pixel_clk", NULL);
	if (IS_ERR(clcd_pclk)) {
		pr_err("%s:clcd-pixel clock get fail\n", __func__);
		return;
	}

	ah_clk = clk_get(NULL, "ahb_clk");
	if (IS_ERR(ah_clk)) {
		pr_err("%s:enabling ahb_clk fail\n", __func__);
		goto free_clcd_pclk;
	}

	fb_clk = clk_get_sys("clcd-db9000", NULL);
	if (IS_ERR(fb_clk)) {
		pr_err("%s:enabling fb_clk fail\n", __func__);
		goto free_ah_clk;
	}

	/*
	 * set pixel clk on bus clk by default, would be changed by
	 * driver as per the need
	 */
	ret = clk_set_parent(fb_clk, clcd_pclk);
	if (ret) {
		pr_err("%s:failed to set pixel clk as parent of clcd\n",
				__func__);
		goto free_fb_clk;
	}

	data->pixel_clk = clcd_pclk;
	data->bus_clk = ah_clk;

	/* SPEAr1340 uses different pll for cpu and clcd */
	if (cpu_is_spear1340())
		data->ignore_cpufreq_notification = 1;
	else
		data->ignore_cpufreq_notification = 0;

free_fb_clk:
	clk_put(fb_clk);
free_ah_clk:
	clk_put(ah_clk);
free_clcd_pclk:
	clk_put(clcd_pclk);
}

static unsigned long frame_buf_base;
static unsigned long frame_buf_size;

void spear13xx_panel_fixup(struct meminfo *mi)
{
	frame_buf_size = (NUM_OF_FRAMEBUFFERS * PANEL_MAX_XRES *
			PANEL_MAX_YRES * PANEL_MAX_BPP / 8);
	frame_buf_base = reserve_mem(mi, ALIGN(frame_buf_size, SZ_1M));
	if (frame_buf_base == ~0)
		pr_err("Unable to allocate fb buffer\n");
}

void spear13xx_panel_init(struct platform_device *pdev)
{
	struct db9000fb_mach_info *mach_info;
	mach_info = &clcd_plat_info;

	if (!mach_info) {
		pr_err("Invalid panel requested:\n");
		return;
	}
	if (machine_is_spear1340_evb() || machine_is_spear900_evb())
		mach_info->def_mode = "1024x768-32@60";
	else
		mach_info->def_mode = "480x272-32@0";

	mach_info->frame_buf_base = frame_buf_base;
	mach_info->mem_size = frame_buf_size;
	clcd_set_plat_data(&spear13xx_db9000_clcd_device, mach_info);
}
