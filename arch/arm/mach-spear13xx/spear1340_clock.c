/*
 * arch/arm/mach-spear13xx/spear1340_clock.c
 *
 * SPEAr13xx machines clock framework source file
 *
 * Copyright (C) 2011 ST Microelectronics
 * Viresh Kumar <viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <plat/clock.h>
#include <mach/hardware.h>
#include <mach/spear1340_misc_regs.h>

/* root clks */
/* 24 MHz oscillator clock */
static struct clk osc1_24m_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.rate = 24000000,
};

/* 32 KHz oscillator clock */
static struct clk osc2_32k_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.rate = 32000,
};

/* 25 MHz MIPHY oscillator clock */
static struct clk osc3_25m_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.rate = 25000000,
};

/* clock derived from 32 KHz osc clk */
/* rtc clock */
static struct clk rtc_clk = {
	.en_reg = VA_SPEAR1340_PERIP1_CLK_ENB,
	.en_reg_bit = SPEAR1340_RTC_CLK_ENB,
	.pclk = &osc2_32k_clk,
	.recalc = &follow_parent,
};

/* clock derived from osc1 or osc3 */
/* vco[1-3] parents */
static struct pclk_info vco_pclk_info[] = {
	{
		.pclk = &osc1_24m_clk,
		.pclk_val = SPEAR1340_OSC_24M_VAL,
	}, {
		.pclk = &osc3_25m_clk,
		.pclk_val = SPEAR1340_OSC_25M_VAL,
	},
};

/* vco[1-3] parent select structure */
static struct pclk_sel vco_pclk_sel = {
	.pclk_info = vco_pclk_info,
	.pclk_count = ARRAY_SIZE(vco_pclk_info),
	.pclk_sel_reg = VA_SPEAR1340_PLL_CFG,
	.pclk_sel_mask = SPEAR1340_PLL_CLK_MASK,
};

/* vco masks structure */
static struct vco_clk_masks vco_masks = {
	.mode_mask = SPEAR1340_PLL_MODE_MASK,
	.mode_shift = SPEAR1340_PLL_MODE_SHIFT,
	.norm_fdbk_m_mask = SPEAR1340_PLL_NORM_FDBK_M_MASK,
	.norm_fdbk_m_shift = SPEAR1340_PLL_NORM_FDBK_M_SHIFT,
	.dith_fdbk_m_mask = SPEAR1340_PLL_DITH_FDBK_M_MASK,
	.dith_fdbk_m_shift = SPEAR1340_PLL_DITH_FDBK_M_SHIFT,
	.div_p_mask = SPEAR1340_PLL_DIV_P_MASK,
	.div_p_shift = SPEAR1340_PLL_DIV_P_SHIFT,
	.div_n_mask = SPEAR1340_PLL_DIV_N_MASK,
	.div_n_shift = SPEAR1340_PLL_DIV_N_SHIFT,
	.pll_lock_mask = SPEAR1340_PLL_LOCK_MASK,
	.pll_lock_shift = SPEAR1340_PLL_LOCK_SHIFT,
};
/* vco1 configuration structure */
static struct vco_clk_config vco1_config = {
	.mode_reg = VA_SPEAR1340_PLL1_CTR,
	.cfg_reg = VA_SPEAR1340_PLL1_FRQ,
	.masks = &vco_masks,
};

/* vco rate configuration table, in ascending order of rates */
static struct vco_rate_tbl vco_rtbl[] = {
	/* PCLK 24MHz */
	{.mode = 0, .m = 0x83, .n = 0x04, .p = 0x5}, /* vco 1572, pll 49.125 MHz */
	{.mode = 0, .m = 0x7D, .n = 0x06, .p = 0x3}, /* vco 1000, pll 125 MHz */
	{.mode = 0, .m = 0x64, .n = 0x06, .p = 0x1}, /* vco 800, pll 400 MHz */
	{.mode = 0, .m = 0x7D, .n = 0x06, .p = 0x1}, /* vco 1000, pll 500 MHz */
	{.mode = 0, .m = 0xA6, .n = 0x06, .p = 0x1}, /* vco 1328, pll 664 MHz */
	{.mode = 0, .m = 0xC8, .n = 0x06, .p = 0x1}, /* vco 1600, pll 800 MHz */
	{.mode = 0, .m = 0x7D, .n = 0x06, .p = 0x0}, /* vco 1, pll 1 GHz */
	{.mode = 0, .m = 0x96, .n = 0x06, .p = 0x0}, /* vco 1200, pll 1200 MHz */
};

/* vco1 clock */
static struct clk vco1_clk = {
	.flags = ENABLED_ON_INIT | SYSTEM_CLK,
	.pclk_sel = &vco_pclk_sel,
	.pclk_sel_shift = SPEAR1340_PLL1_CLK_SHIFT,
	.en_reg = VA_SPEAR1340_PLL1_CTR,
	.en_reg_bit = SPEAR1340_PLL_ENABLE,
	.calc_rate = &vco_calc_rate,
	.recalc = &vco_clk_recalc,
	.set_rate = &vco_clk_set_rate,
	.rate_config = {vco_rtbl, ARRAY_SIZE(vco_rtbl), 6},
	.private_data = &vco1_config,
};

/* thermal clock */
static struct clk thermal_clk = {
	.en_reg = VA_SPEAR1340_PERIP2_CLK_ENB,
	.en_reg_bit = SPEAR1340_THSENS_CLK_ENB,
	.pclk = &osc1_24m_clk,
	.div_factor = 128,
	.recalc = &follow_parent,
};

/* clock derived from vco1 clock */
/* pll1 clock */
static struct clk pll1_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.pclk = &vco1_clk,
	.recalc = &pll_clk_recalc,
};

/* vco1div2 clock */
static struct clk vco1div2_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.pclk = &vco1_clk,
	.div_factor = 2,
	.recalc = &follow_parent,
};

/*
 * Synthesizer Clock derived from vcodiv2. This clock is one of the
 * possible clocks to feed cpu directly.
 * We can program this synthesizer to make cpu run on different clock
 * frequencies.
 * Following table provides configuration values to let cpu run on 200,
 * 250, 332, 400 or 500 MHz considering different possibilites of input
 * (vco1div2) clock.
 *
 * --------------------------------------------------------------------
 * vco1div2(Mhz)	fout(Mhz)	cpuclk = fout/2		div
 * --------------------------------------------------------------------
 * 400			200		100			0x04000
 * 400			250		125			0x03333
 * 400			332		166			0x0268D
 * 400			400		200			0x02000
 * --------------------------------------------------------------------
 * 500			200		100			0x05000
 * 500			250		125			0x04000
 * 500			332		166			0x03031
 * 500			400		200			0x02800
 * 500			500		250			0x02000
 * --------------------------------------------------------------------
 * 664			200		100			0x06a38
 * 664			250		125			0x054FD
 * 664			332		166			0x04000
 * 664			400		200			0x0351E
 * 664			500		250			0x02A7E
 * --------------------------------------------------------------------
 * 800			200		100			0x08000
 * 800			250		125			0x06666
 * 800			332		166			0x04D18
 * 800			400		200			0x04000
 * 800			500		250			0x03333
 * --------------------------------------------------------------------
 * sys rate configuration table is in descending order of divisor.
 */
static struct frac_synth_rate_tbl sys_synth_rtbl[] = {
	{.div = 0x08000},
	{.div = 0x06a38},
	{.div = 0x06666},
	{.div = 0x054FD},
	{.div = 0x05000},
	{.div = 0x04D18},
	{.div = 0x04000},
	{.div = 0x0351E},
	{.div = 0x03333},
	{.div = 0x03031},
	{.div = 0x02A7E},
	{.div = 0x02800},
	{.div = 0x0268D},
	{.div = 0x02000},
};

/* common fractional synthesizer masks */
static struct frac_synth_masks frac_synth_masks = {
	.div_factor_mask = SPEAR1340_FRAC_SYNT_DIV_FACTOR_MASK,
	.div_factor_shift = SPEAR1340_FRAC_SYNT_DIV_FACTOR_SHIFT,
};

/* system synthesizer clock definitions */
static struct frac_synth_clk_config sys_synth_config = {
	.synth_reg = VA_SPEAR1340_SYS_CLK_SYNT,
	.masks = &frac_synth_masks,
};

/* sys synth clock */
static struct clk sys_synth_clk = {
	.flags = SYSTEM_CLK,
	.en_reg = VA_SPEAR1340_SYS_CLK_SYNT,
	.en_reg_bit = SPEAR1340_FRAC_SYNT_ENB,
	.pclk = &vco1div2_clk,
	.calc_rate = &frac_synth_calc_rate,
	.recalc = &frac_synth_clk_recalc,
	.set_rate = &frac_synth_clk_set_rate,
	.rate_config = {sys_synth_rtbl, ARRAY_SIZE(sys_synth_rtbl), 8},
	.private_data = &sys_synth_config,
};

/*
 * All below entries generate 166 MHz for
 * different values of vco1div2
 */
static struct frac_synth_rate_tbl amba_synth_rtbl[] = {
	{.div = 0x06062}, /* for vco1div2 = 500 MHz */
	{.div = 0x04D1B}, /* for vco1div2 = 400 MHz */
	{.div = 0x04000}, /* for vco1div2 = 332 MHz */
	{.div = 0x03031}, /* for vco1div2 = 250 MHz */
	{.div = 0x0268D}, /* for vco1div2 = 200 MHz */
};

/* amba synth clock definitions */
static struct frac_synth_clk_config amba_synth_config = {
	.synth_reg = VA_SPEAR1340_AMBA_CLK_SYNT,
	.masks = &frac_synth_masks,
};

/* sys synth clock */
static struct clk amba_synth_clk = {
	.flags = SYSTEM_CLK,
	.en_reg = VA_SPEAR1340_AMBA_CLK_SYNT,
	.en_reg_bit = SPEAR1340_FRAC_SYNT_ENB,
	.pclk = &vco1div2_clk,
	.calc_rate = &frac_synth_calc_rate,
	.recalc = &frac_synth_clk_recalc,
	.set_rate = &frac_synth_clk_set_rate,
	.rate_config = {amba_synth_rtbl, ARRAY_SIZE(amba_synth_rtbl), 0},
	.private_data = &amba_synth_config,
};

/* vco1div4 clock */
static struct clk vco1div4_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.pclk = &vco1_clk,
	.div_factor = 4,
	.recalc = &follow_parent,
};

/* vco2 configuration structure */
static struct vco_clk_config vco2_config = {
	.mode_reg = VA_SPEAR1340_PLL2_CTR,
	.cfg_reg = VA_SPEAR1340_PLL2_FRQ,
	.masks = &vco_masks,
};

/* vco2 clock */
static struct clk vco2_clk = {
	.flags = ENABLED_ON_INIT | SYSTEM_CLK,
	.pclk_sel = &vco_pclk_sel,
	.pclk_sel_shift = SPEAR1340_PLL2_CLK_SHIFT,
	.en_reg = VA_SPEAR1340_PLL2_CTR,
	.en_reg_bit = SPEAR1340_PLL_ENABLE,
	.calc_rate = &vco_calc_rate,
	.recalc = &vco_clk_recalc,
	.set_rate = &vco_clk_set_rate,
	.rate_config = {vco_rtbl, ARRAY_SIZE(vco_rtbl), 6},
	.private_data = &vco2_config,
};

/* clock derived from vco2 clock */
/* pll2 clock */
static struct clk pll2_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.pclk = &vco2_clk,
	.recalc = &pll_clk_recalc,
};

/* vco2div2 clock */
static struct clk vco2div2_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.pclk = &vco2_clk,
	.div_factor = 2,
	.recalc = &follow_parent,
};

/* vco3 configuration structure */
static struct vco_clk_config vco3_config = {
	.mode_reg = VA_SPEAR1340_PLL3_CTR,
	.cfg_reg = VA_SPEAR1340_PLL3_FRQ,
	.masks = &vco_masks,
};

/* vco3 clock */
static struct clk vco3_clk = {
	.flags = ENABLED_ON_INIT | SYSTEM_CLK,
	.pclk_sel = &vco_pclk_sel,
	.pclk_sel_shift = SPEAR1340_PLL3_CLK_SHIFT,
	.en_reg = VA_SPEAR1340_PLL3_CTR,
	.en_reg_bit = SPEAR1340_PLL_ENABLE,
	.calc_rate = &vco_calc_rate,
	.recalc = &vco_clk_recalc,
	.set_rate = &vco_clk_set_rate,
	.rate_config = {vco_rtbl, ARRAY_SIZE(vco_rtbl), 0},
	.private_data = &vco3_config,
};

/* clock derived from vco3 clock */
/* pll3 clock */
static struct clk pll3_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.pclk = &vco3_clk,
	.recalc = &pll_clk_recalc,
};

/* vco3div2 clock */
static struct clk vco3div2_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.pclk = &vco3_clk,
	.div_factor = 2,
	.recalc = &follow_parent,
};

/* vco4 (DDR) configuration structure */
static struct vco_clk_config vco4_config = {
	.mode_reg = VA_SPEAR1340_PLL4_CTR,
	.cfg_reg = VA_SPEAR1340_PLL4_FRQ,
	.masks = &vco_masks,
};

/* vco4 rate configuration table, in ascending order of rates */
static struct vco_rate_tbl vco4_rtbl[] = {
	{.mode = 0, .m = 0x7D, .n = 0x06, .p = 0x2}, /* vco 1000, pll 250 MHz */
	{.mode = 0, .m = 0xA6, .n = 0x06, .p = 0x2}, /* vco 1328, pll 332 MHz */
	{.mode = 0, .m = 0xC8, .n = 0x06, .p = 0x2}, /* vco 1600, pll 400 MHz */
	{.mode = 0, .m = 0x7D, .n = 0x06, .p = 0x0}, /* vco 1, pll 1 GHz */
};

/* vco4 (DDR) clock */
static struct clk vco4_clk = {
	.flags = ENABLED_ON_INIT | SYSTEM_CLK,
	.en_reg = VA_SPEAR1340_PLL4_CTR,
	.en_reg_bit = SPEAR1340_PLL_ENABLE,
	.pclk = &osc1_24m_clk,
	.calc_rate = &vco_calc_rate,
	.recalc = &vco_clk_recalc,
	.set_rate = &vco_clk_set_rate,
	.rate_config = {vco4_rtbl, ARRAY_SIZE(vco4_rtbl), 3},
	.private_data = &vco4_config,
};

/* clock derived from vco4 clock */
/* pll4 clock */
static struct clk pll4_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.pclk = &vco4_clk,
	.recalc = &pll_clk_recalc,
};

/* pll5 USB 48 MHz clock */
static struct clk pll5_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.pclk = &osc1_24m_clk,
	.rate = 48000000,
};

/* pll6 (MIPHY) clock */
static struct clk pll6_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.pclk = &osc3_25m_clk,
	.rate = 25000000,
};

/* clocks derived from pll1 clk */
/* ddr clock */
static struct ddr_rate_tbl ddr_rate_tbl = {
	.minrate = 332000000,
	.maxrate = 500000000,
};

static struct clk ddr_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.pclk = &pll4_clk,
	.recalc = &follow_parent,
	.private_data = &ddr_rate_tbl,
};

static struct pclk_info sys_pclk_info[] = {
	{
		.pclk = &pll1_clk,
		.pclk_val = SPEAR1340_SCLK_SRC_PLL1,
	}, {
		.pclk = &sys_synth_clk,
		.pclk_val = SPEAR1340_SCLK_SRC_SYNT,
	}, {
		.pclk = &pll2_clk,
		.pclk_val = SPEAR1340_SCLK_SRC_PLL2,
	}, {
		.pclk = &pll3_clk,
		.pclk_val = SPEAR1340_SCLK_SRC_PLL3,
	},
};

static struct pclk_sel sys_pclk_sel = {
	.pclk_info = sys_pclk_info,
	.pclk_count = ARRAY_SIZE(sys_pclk_info),
	.pclk_sel_reg = VA_SPEAR1340_SYS_CLK_CTRL,
	.pclk_sel_mask = SPEAR1340_SCLK_SRC_SEL_MASK,
};

/* sys clock */
static struct clk sys_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.pclk_sel = &sys_pclk_sel,
	.pclk_sel_shift = SPEAR1340_SCLK_SRC_SEL_SHIFT,
	.recalc = &follow_parent,
};

/* cpu clock */
static struct clk cpu_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.pclk = &sys_clk,
	.div_factor = 2,
	.recalc = &follow_parent,
};

/* cpu clock div3*/
static struct clk cpu_clk_div3 = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.pclk = &cpu_clk,
	.div_factor = 3,
	.recalc = &follow_parent,
};

/* ahb clock */
static struct pclk_info ahb_pclk_info[] = {
	{
		.pclk = &cpu_clk_div3,
		.pclk_val = SPEAR1340_HCLK_SRC_CPU,
	}, {
		.pclk = &amba_synth_clk,
		.pclk_val = SPEAR1340_HCLK_SRC_SYNT,
	},
};

static struct pclk_sel ahb_pclk_sel = {
	.pclk_info = ahb_pclk_info,
	.pclk_count = ARRAY_SIZE(ahb_pclk_info),
	.pclk_sel_reg = VA_SPEAR1340_SYS_CLK_CTRL,
	.pclk_sel_mask = SPEAR1340_HCLK_SRC_SEL_MASK,
};

static struct clk ahb_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.pclk_sel = &ahb_pclk_sel,
	.pclk_sel_shift = SPEAR1340_HCLK_SRC_SEL_SHIFT,
	.recalc = &follow_parent,
};

/* apb clock */
static struct clk apb_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.pclk = &ahb_clk,
	.div_factor = 2,
	.recalc = &follow_parent,
};

/* clocks derived from osc1, ahb or apb */
/* gpt[0-3] parents */
static struct pclk_info gpt_pclk_info[] = {
	{
		.pclk = &osc1_24m_clk,
		.pclk_val = SPEAR1340_GPT_OSC24_VAL,
	}, {
		.pclk = &apb_clk,
		.pclk_val = SPEAR1340_GPT_APB_VAL,
	},
};

/* gpt[0-3] parent select structure */
static struct pclk_sel gpt_pclk_sel = {
	.pclk_info = gpt_pclk_info,
	.pclk_count = ARRAY_SIZE(gpt_pclk_info),
	.pclk_sel_reg = VA_SPEAR1340_PERIP_CLK_CFG,
	.pclk_sel_mask = SPEAR1340_GPT_CLK_MASK,
};

/* gpt0 timer clock */
static struct clk gpt0_clk = {
	.en_reg = VA_SPEAR1340_PERIP1_CLK_ENB,
	.en_reg_bit = SPEAR1340_GPT0_CLK_ENB,
	.pclk_sel = &gpt_pclk_sel,
	.pclk_sel_shift = SPEAR1340_GPT0_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* gpt1 timer clock */
static struct clk gpt1_clk = {
	.en_reg = VA_SPEAR1340_PERIP1_CLK_ENB,
	.en_reg_bit = SPEAR1340_GPT1_CLK_ENB,
	.pclk_sel = &gpt_pclk_sel,
	.pclk_sel_shift = SPEAR1340_GPT1_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* gpt2 timer clock */
static struct clk gpt2_clk = {
	.en_reg = VA_SPEAR1340_PERIP2_CLK_ENB,
	.en_reg_bit = SPEAR1340_GPT2_CLK_ENB,
	.pclk_sel = &gpt_pclk_sel,
	.pclk_sel_shift = SPEAR1340_GPT2_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* gpt3 timer clock */
static struct clk gpt3_clk = {
	.en_reg = VA_SPEAR1340_PERIP2_CLK_ENB,
	.en_reg_bit = SPEAR1340_GPT3_CLK_ENB,
	.pclk_sel = &gpt_pclk_sel,
	.pclk_sel_shift = SPEAR1340_GPT3_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* watch dog timer clock */
static struct clk wdt_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &cpu_clk,
	.div_factor = 2,
	.recalc = &follow_parent,
};

/* auxiliary synthesizers masks */
static struct aux_clk_masks aux_masks = {
	.eq_sel_mask = SPEAR1340_AUX_EQ_SEL_MASK,
	.eq_sel_shift = SPEAR1340_AUX_EQ_SEL_SHIFT,
	.eq1_mask = SPEAR1340_AUX_EQ1_SEL,
	.eq2_mask = SPEAR1340_AUX_EQ2_SEL,
	.xscale_sel_mask = SPEAR1340_AUX_XSCALE_MASK,
	.xscale_sel_shift = SPEAR1340_AUX_XSCALE_SHIFT,
	.yscale_sel_mask = SPEAR1340_AUX_YSCALE_MASK,
	.yscale_sel_shift = SPEAR1340_AUX_YSCALE_SHIFT,
};

/* aux rate configuration table, in ascending order of rates */
static struct aux_rate_tbl aux_rtbl[] = {
	/* For VCO1div2 = 500 MHz */
	{.xscale = 10, .yscale = 204, .eq = 0}, /* 12.29 MHz */
	{.xscale = 4, .yscale = 21, .eq = 0}, /* 48 MHz */
	{.xscale = 2, .yscale = 6, .eq = 0}, /* 83 MHz */
	{.xscale = 2, .yscale = 4, .eq = 0}, /* 125 MHz */
	{.xscale = 1, .yscale = 3, .eq = 1}, /* 166 MHz */
	{.xscale = 1, .yscale = 2, .eq = 1}, /* 250 MHz */
};

/* clocks derived multiple parents (pll1, pll5, synthesizers or others) */
/* uart0 configurations */
static struct aux_clk_config uart0_synth_config = {
	.synth_reg = VA_SPEAR1340_UART0_CLK_SYNT,
	.masks = &aux_masks,
};

/* uart1 configurations */
static struct aux_clk_config uart1_synth_config = {
	.synth_reg = VA_SPEAR1340_UART1_CLK_SYNT,
	.masks = &aux_masks,
};

/* uart0 synth clock */
static struct clk uart0_synth_clk = {
	.en_reg = VA_SPEAR1340_UART0_CLK_SYNT,
	.en_reg_bit = SPEAR1340_AUX_SYNT_ENB,
	.pclk = &vco1div2_clk,
	.calc_rate = &aux_calc_rate,
	.recalc = &aux_clk_recalc,
	.set_rate = &aux_clk_set_rate,
	.rate_config = {aux_rtbl, ARRAY_SIZE(aux_rtbl), 2},
	.private_data = &uart0_synth_config,
};

/* uart1 synth clock */
static struct clk uart1_synth_clk = {
	.en_reg = VA_SPEAR1340_UART1_CLK_SYNT,
	.en_reg_bit = SPEAR1340_AUX_SYNT_ENB,
	.pclk = &vco1div2_clk,
	.calc_rate = &aux_calc_rate,
	.recalc = &aux_clk_recalc,
	.set_rate = &aux_clk_set_rate,
	.rate_config = {aux_rtbl, ARRAY_SIZE(aux_rtbl), 2},
	.private_data = &uart1_synth_config,
};

/* uart0 parents */
static struct pclk_info uart0_pclk_info[] = {
	{
		.pclk = &pll5_clk,
		.pclk_val = SPEAR1340_UART_CLK_PLL5_VAL,
	}, {
		.pclk = &osc1_24m_clk,
		.pclk_val = SPEAR1340_UART_CLK_OSC24_VAL,
	}, {
		.pclk = &uart0_synth_clk,
		.pclk_val = SPEAR1340_UART_CLK_SYNT_VAL,
	},
};

/* uart1 parents */
static struct pclk_info uart1_pclk_info[] = {
	{
		.pclk = &pll5_clk,
		.pclk_val = SPEAR1340_UART_CLK_PLL5_VAL,
	}, {
		.pclk = &osc1_24m_clk,
		.pclk_val = SPEAR1340_UART_CLK_OSC24_VAL,
	}, {
		.pclk = &uart1_synth_clk,
		.pclk_val = SPEAR1340_UART_CLK_SYNT_VAL,
	},
};

/* uart0 parent select structure */
static struct pclk_sel uart0_pclk_sel = {
	.pclk_info = uart0_pclk_info,
	.pclk_count = ARRAY_SIZE(uart0_pclk_info),
	.pclk_sel_reg = VA_SPEAR1340_PERIP_CLK_CFG,
	.pclk_sel_mask = SPEAR1340_UART_CLK_MASK,
};

/* uart1 parent select structure */
static struct pclk_sel uart1_pclk_sel = {
	.pclk_info = uart1_pclk_info,
	.pclk_count = ARRAY_SIZE(uart1_pclk_info),
	.pclk_sel_reg = VA_SPEAR1340_PERIP_CLK_CFG,
	.pclk_sel_mask = SPEAR1340_UART_CLK_MASK,
};

/* uart0 clock */
static struct clk uart0_clk = {
	.en_reg = VA_SPEAR1340_PERIP1_CLK_ENB,
	.en_reg_bit = SPEAR1340_UART0_CLK_ENB,
	.pclk_sel = &uart0_pclk_sel,
	.pclk_sel_shift = SPEAR1340_UART0_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* uart1 clock */
static struct clk uart1_clk = {
	.en_reg = VA_SPEAR1340_PERIP1_CLK_ENB,
	.en_reg_bit = SPEAR1340_UART1_CLK_ENB,
	.pclk_sel = &uart1_pclk_sel,
	.pclk_sel_shift = SPEAR1340_UART1_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* sdhci configurations */
static struct aux_clk_config sdhci_synth_config = {
	.synth_reg = VA_SPEAR1340_SDHCI_CLK_SYNT,
	.masks = &aux_masks,
};

/* sdhci synth clock */
static struct clk sdhci_synth_clk = {
	.en_reg = VA_SPEAR1340_SDHCI_CLK_SYNT,
	.en_reg_bit = SPEAR1340_AUX_SYNT_ENB,
	.pclk = &vco1div2_clk,
	.calc_rate = &aux_calc_rate,
	.recalc = &aux_clk_recalc,
	.set_rate = &aux_clk_set_rate,
	.rate_config = {aux_rtbl, ARRAY_SIZE(aux_rtbl), 1},
	.private_data = &sdhci_synth_config,
};

/* sdhci clock */
static struct clk sdhci_clk = {
	.en_reg = VA_SPEAR1340_PERIP1_CLK_ENB,
	.en_reg_bit = SPEAR1340_SDHCI_CLK_ENB,
	.pclk = &sdhci_synth_clk,
	.recalc = &follow_parent,
};

/* cfxd configurations */
static struct aux_clk_config cfxd_synth_config = {
	.synth_reg = VA_SPEAR1340_CFXD_CLK_SYNT,
	.masks = &aux_masks,
};

/* cfxd synth clock */
static struct clk cfxd_synth_clk = {
	.en_reg = VA_SPEAR1340_CFXD_CLK_SYNT,
	.en_reg_bit = SPEAR1340_AUX_SYNT_ENB,
	.pclk = &vco1div2_clk,
	.calc_rate = &aux_calc_rate,
	.recalc = &aux_clk_recalc,
	.set_rate = &aux_clk_set_rate,
	.rate_config = {aux_rtbl, ARRAY_SIZE(aux_rtbl), 4},
	.private_data = &cfxd_synth_config,
};

/* cfxd clock */
static struct clk cfxd_clk = {
	.en_reg = VA_SPEAR1340_PERIP1_CLK_ENB,
	.en_reg_bit = SPEAR1340_CFXD_CLK_ENB,
	.pclk = &cfxd_synth_clk,
	.recalc = &follow_parent,
};

/* C3 clk configurations */
static struct aux_clk_config c3_synth_config = {
	.synth_reg = VA_SPEAR1340_C3_CLK_SYNT,
	.masks = &aux_masks,
};

/* c3 synth clock */
static struct clk c3_synth_clk = {
	.en_reg = VA_SPEAR1340_C3_CLK_SYNT,
	.en_reg_bit = SPEAR1340_AUX_SYNT_ENB,
	.pclk = &vco1div2_clk,
	.calc_rate = &aux_calc_rate,
	.recalc = &aux_clk_recalc,
	.set_rate = &aux_clk_set_rate,
	.rate_config = {aux_rtbl, ARRAY_SIZE(aux_rtbl), 2},
	.private_data = &c3_synth_config,
};

/* c3 parents */
static struct pclk_info c3_pclk_info[] = {
	{
		.pclk = &pll5_clk,
		.pclk_val = SPEAR1340_AUX_CLK_PLL5_VAL,
	}, {
		.pclk = &c3_synth_clk,
		.pclk_val = SPEAR1340_AUX_CLK_SYNT_VAL,
	},
};

/* c3 parent select structure */
static struct pclk_sel c3_pclk_sel = {
	.pclk_info = c3_pclk_info,
	.pclk_count = ARRAY_SIZE(c3_pclk_info),
	.pclk_sel_reg = VA_SPEAR1340_PERIP_CLK_CFG,
	.pclk_sel_mask = SPEAR1340_C3_CLK_MASK,
};

/* c3 clock */
static struct clk c3_clk = {
	.en_reg = VA_SPEAR1340_PERIP1_CLK_ENB,
	.en_reg_bit = SPEAR1340_C3_CLK_ENB,
	.pclk_sel = &c3_pclk_sel,
	.pclk_sel_shift = SPEAR1340_C3_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* gmac phy clk configurations */
static struct aux_clk_config gmac_phy_synth_config = {
	.synth_reg = VA_SPEAR1340_GMAC_CLK_SYNT,
	.masks = &aux_masks,
};

/* gmii external pad clock for phy operation */
static struct clk gmii_125m_pad = {
	.flags = ALWAYS_ENABLED,
	.rate = 125000000,
};

/* gmac phy set of input clks*/
static struct pclk_info gmac_phy_input_pclk_info[] = {
	{
		.pclk = &gmii_125m_pad,
		.pclk_val = SPEAR1340_GMAC_PHY_125M_PAD_VAL,
	}, {
		.pclk = &pll2_clk,
		.pclk_val = SPEAR1340_GMAC_PHY_PLL2_VAL,
	}, {
		.pclk = &osc3_25m_clk,
		.pclk_val = SPEAR1340_GMAC_PHY_OSC3_VAL,
	},
};

static struct pclk_sel gmac_phy_input_pclk_sel = {
	.pclk_info = gmac_phy_input_pclk_info,
	.pclk_count = ARRAY_SIZE(gmac_phy_input_pclk_info),
	.pclk_sel_reg = VA_SPEAR1340_GMAC_CLK_CFG,
	.pclk_sel_mask = SPEAR1340_GMAC_PHY_INPUT_CLK_MASK,
};

static struct clk gmac_phy_input_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk_sel = &gmac_phy_input_pclk_sel,
	.pclk_sel_shift = SPEAR1340_GMAC_PHY_INPUT_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* gmac rate configuration table, in ascending order of rates */
static struct aux_rate_tbl gmac_rtbl[] = {
	/* For gmac phy input clk */
	{.xscale = 2, .yscale = 6, .eq = 0}, /* divided by 6 */
	{.xscale = 2, .yscale = 4, .eq = 0}, /* divided by 4 */
	{.xscale = 1, .yscale = 3, .eq = 1}, /* divided by 3 */
	{.xscale = 1, .yscale = 2, .eq = 1}, /* divided by 2 */
};

static struct clk gmac_phy_synth_clk = {
	.en_reg = VA_SPEAR1340_GMAC_CLK_SYNT,
	.en_reg_bit = SPEAR1340_AUX_SYNT_ENB,
	.pclk = &gmac_phy_input_clk,
	.calc_rate = &aux_calc_rate,
	.recalc = &aux_clk_recalc,
	.set_rate = &aux_clk_set_rate,
	.rate_config = {gmac_rtbl, ARRAY_SIZE(gmac_rtbl), 0},
	.private_data = &gmac_phy_synth_config,
};

/* gmac phy parents */
static struct pclk_info gmac_phy_pclk_info[] = {
	{
		.pclk = &gmac_phy_input_clk,
		.pclk_val = SPEAR1340_GMAC_PHY_INPUT_ENB_VAL,
	}, {
		.pclk = &gmac_phy_synth_clk,
		.pclk_val = SPEAR1340_GMAC_PHY_SYNT_ENB_VAL,
	}
};

/* gmac phy parent select structure */
static struct pclk_sel gmac_phy_pclk_sel = {
	.pclk_info = gmac_phy_pclk_info,
	.pclk_count = ARRAY_SIZE(gmac_phy_pclk_info),
	.pclk_sel_reg = VA_SPEAR1340_GMAC_CLK_CFG,
	.pclk_sel_mask = SPEAR1340_GMAC_PHY_CLK_MASK,
};

/* gmac phy clock */
static struct clk gmac_phy0_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk_sel = &gmac_phy_pclk_sel,
	.pclk_sel_shift = SPEAR1340_GMAC_PHY_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* clcd fractional synthesizers definition */
static struct frac_synth_clk_config clcd_synth_config = {
	.synth_reg = VA_SPEAR1340_CLCD_CLK_SYNT,
	.masks = &frac_synth_masks,
};

/* clcd fractional synthesizer parents */
static struct pclk_info clcd_synth_pclk_info[] = {
	{
		.pclk = &vco1div4_clk,
		.pclk_val = SPEAR1340_CLCD_SYNT_VCO1_DIV4_VAL,
	}, {
		.pclk = &pll2_clk,
		.pclk_val = SPEAR1340_CLCD_SYNT_PLL2_VAL,
	},
};

/* clcd fractional synthesizer parent select structure */
static struct pclk_sel clcd_synth_pclk_sel = {
	.pclk_info = clcd_synth_pclk_info,
	.pclk_count = ARRAY_SIZE(clcd_synth_pclk_info),
	.pclk_sel_reg = VA_SPEAR1340_PLL_CFG,
	.pclk_sel_mask = SPEAR1340_CLCD_SYNT_CLK_MASK,
};

/* clcd rate configuration table, in ascending order of rates */
static struct frac_synth_rate_tbl clcd_rtbl[] = {
	{.div = 0x14000}, /* 25 Mhz , for vc01div4 = 250 MHz*/
	{.div = 0x1284B}, /* 27 Mhz , for vc01div4 = 250 MHz*/
	{.div = 0x0D8D3}, /* 58 Mhz , for vco1div4 = 393 MHz */
	{.div = 0x0B72C}, /* 58 Mhz , for vco1div4 = 332 MHz */
	{.div = 0x089EE}, /* 58 Mhz , for vc01div4 = 250 MHz*/
	{.div = 0x07BA0}, /* 65 Mhz , for vc01div4 = 250 MHz*/
	{.div = 0x06f1C}, /* 72 Mhz , for vc01div4 = 250 MHz*/
	{.div = 0x06E58}, /* 58 Mhz , for vco1div4 = 200 MHz */
	{.div = 0x06c1B}, /* 74 Mhz , for vc01div4 = 250 MHz*/
	{.div = 0x04A12}, /* 108 Mhz , for vc01div4 = 250 MHz*/
	{.div = 0x0378E}, /* 144 Mhz , for vc01div4 = 250 MHz*/
	{.div = 0x0360D}, /* 148 Mhz , for vc01div4 = 250 MHz*/
	{.div = 0x035E0}, /* 148.5 MHz, for vc01div4 = 250 MHz*/
};

/* clcd fractional synthesizer clock */
static struct clk clcd_synth_clk = {
	.en_reg = VA_SPEAR1340_CLCD_CLK_SYNT,
	.en_reg_bit = SPEAR1340_FRAC_SYNT_ENB,
	.pclk_sel = &clcd_synth_pclk_sel,
	.pclk_sel_shift = SPEAR1340_CLCD_SYNT_CLK_SHIFT,
	.calc_rate = &frac_synth_calc_rate,
	.recalc = &frac_synth_clk_recalc,
	.set_rate = &frac_synth_clk_set_rate,
	.rate_config = {clcd_rtbl, ARRAY_SIZE(clcd_rtbl), 2},
	.private_data = &clcd_synth_config,
};

/*
 * clcd clock parents
 * This would select to use pll5_clk or clcd_synth_clk as a parent clock
 * of clcd (called as pixel clock in CLCD documentation).
 * For selection of any of these two parent, check the combination of
 * 3-2 bit of clcdclk_sel misc register
 */
static struct pclk_info clcd_pixel_pclk_info[] = {
	{
		.pclk = &pll5_clk,
		.pclk_val = SPEAR1340_AUX_CLK_PLL5_VAL,
	}, {
		.pclk = &clcd_synth_clk,
		.pclk_val = SPEAR1340_AUX_CLK_SYNT_VAL,
	},
};

/* clcd parent select structure */
static struct pclk_sel clcd_pixel_pclk_sel = {
	.pclk_info = clcd_pixel_pclk_info,
	.pclk_count = ARRAY_SIZE(clcd_pixel_pclk_info),
	.pclk_sel_reg = VA_SPEAR1340_PERIP_CLK_CFG,
	.pclk_sel_mask = SPEAR1340_CLCD_CLK_MASK,
};

static struct clk clcd_pixel_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk_sel = &clcd_pixel_pclk_sel,
	.pclk_sel_shift = SPEAR1340_CLCD_CLK_SHIFT,
	.recalc = &follow_parent,
};

/*
 * clcd clock
 * There are 2 options for clcd clock,
 * - derived from AHB (bus clk)
 * - dervied from Pixel Clock Input (pixel clk)
 * The selection bit for these clock is in device itself, hence treating
 * them as virtual clocks without selection register.
 * Controller driver on itself needs to program proper selection
 */

static struct pclk_info clcd_pclk_info[] = {
	{
		.pclk = &clcd_pixel_clk,
	}, {
		.pclk = &ahb_clk,
	},
};

static struct pclk_sel clcd_pclk_sel = {
	.pclk_info = clcd_pclk_info,
	.pclk_count = ARRAY_SIZE(clcd_pclk_info),
	.pclk_sel_reg = 0, /* no select register */
};

/* clcd clock */
static struct clk clcd_clk = {
	.en_reg = VA_SPEAR1340_PERIP1_CLK_ENB,
	.en_reg_bit = SPEAR1340_CLCD_CLK_ENB,
	.pclk_sel = &clcd_pclk_sel,
	.recalc = &follow_parent,
};

/* i2s related clocks */
/* i2s source clock parents */
static struct clk i2s_src_pad_clk = {
	.flags = ALWAYS_ENABLED,
	.rate = 12288000,
};

static struct pclk_info i2s_src_pclk_info[] = {
	{
		.pclk = &vco1div2_clk,
		.pclk_val = SPEAR1340_I2S_SRC_VCO1DIV2_VAL,
	}, {
		.pclk = &pll2_clk,
		.pclk_val = SPEAR1340_I2S_SRC_PLL2_VAL,
	}, {
		.pclk = &pll3_clk,
		.pclk_val = SPEAR1340_I2S_SRC_PLL3_VAL,
	}, {
		.pclk = &i2s_src_pad_clk,
		.pclk_val = SPEAR1340_I2S_SRC_PL_CLK1_VAL,
	},
};

static struct pclk_sel i2s_src_pclk_sel = {
	.pclk_info = i2s_src_pclk_info,
	.pclk_count = ARRAY_SIZE(i2s_src_pclk_info),
	.pclk_sel_reg = VA_SPEAR1340_I2S_CLK_CFG,
	.pclk_sel_mask = SPEAR1340_I2S_SRC_CLK_MASK,
};

/* i2s src clock */
static struct clk i2s_src_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk_sel = &i2s_src_pclk_sel,
	.pclk_sel_shift = SPEAR1340_I2S_SRC_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* i2s prescaler1 masks */
static struct aux_clk_masks i2s_prs1_aux_masks = {
	.eq_sel_mask = SPEAR1340_AUX_EQ_SEL_MASK,
	.eq_sel_shift = SPEAR1340_I2S_PRS1_EQ_SEL_SHIFT,
	.eq1_mask = SPEAR1340_AUX_EQ1_SEL,
	.eq2_mask = SPEAR1340_AUX_EQ2_SEL,
	.xscale_sel_mask = SPEAR1340_I2S_PRS1_CLK_X_MASK,
	.xscale_sel_shift = SPEAR1340_I2S_PRS1_CLK_X_SHIFT,
	.yscale_sel_mask = SPEAR1340_I2S_PRS1_CLK_Y_MASK,
	.yscale_sel_shift = SPEAR1340_I2S_PRS1_CLK_Y_SHIFT,
};

/* i2s prs1 clk configurations */
static struct aux_clk_config i2s_prs1_config = {
	.synth_reg = VA_SPEAR1340_I2S_CLK_CFG,
	.masks = &i2s_prs1_aux_masks,
};

/* i2s prs1 aux rate configuration table, in ascending order of rates */
static struct aux_rate_tbl i2s_prs1_aux_rtbl[] = {
	/* For parent clk = 49.152 MHz */
	{.xscale = 1, .yscale = 12, .eq = 0}, /* 2.048 MHz, smp freq = 8Khz */
	{.xscale = 11, .yscale = 96, .eq = 0}, /* 2.816 MHz, smp freq = 11Khz */
	{.xscale = 1, .yscale = 6, .eq = 0}, /* 4.096 MHz, smp freq = 16Khz */
	{.xscale = 11, .yscale = 48, .eq = 0}, /* 5.632 MHz, smp freq = 22Khz */

	/*
	 * with parent clk = 49.152, freq gen is 8.192 MHz, smp freq = 32Khz
	 * with parent clk = 12.288, freq gen is 2.048 MHz, smp freq = 8Khz
	 */
	{.xscale = 1, .yscale = 3, .eq = 0},

	/* For parent clk = 49.152 MHz */
	{.xscale = 17, .yscale = 37, .eq = 0}, /* 11.289 MHz, smp freq = 44Khz*/
	{.xscale = 1, .yscale = 2, .eq = 0}, /* 12.288 MHz, smp freq = 48Khz*/
};

/* i2s prs1 clock */
static struct clk i2s_prs1_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &i2s_src_clk,
	.calc_rate = &aux_calc_rate,
	.recalc = &aux_clk_recalc,
	.set_rate = &aux_clk_set_rate,
	.rate_config = {i2s_prs1_aux_rtbl, ARRAY_SIZE(i2s_prs1_aux_rtbl), 6},
	.private_data = &i2s_prs1_config,
};

/* i2s ref clk configuration */
static struct pclk_info i2s_ref_pclk_info[] = {
	{
		.pclk = &i2s_src_clk,
		.pclk_val = SPEAR1340_I2S_REF_SRC_VAL,
	}, {
		.pclk = &i2s_prs1_clk,
		.pclk_val = SPEAR1340_I2S_REF_PRS1_VAL,
	},
};

/* i2s ref clock parent select structure */
static struct pclk_sel i2s_ref_pclk_sel = {
	.pclk_info = i2s_ref_pclk_info,
	.pclk_count = ARRAY_SIZE(i2s_ref_pclk_info),
	.pclk_sel_reg = VA_SPEAR1340_I2S_CLK_CFG,
	.pclk_sel_mask = SPEAR1340_I2S_REF_SEL_MASK,
};

/* i2s ref clock */
static struct clk i2s_ref_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk_sel = &i2s_ref_pclk_sel,
	.pclk_sel_shift = SPEAR1340_I2S_REF_SHIFT,
	.recalc = &follow_parent,
};

/* i2s ref pad clock: for codec */
static struct clk i2s_ref_pad_clk = {
	.en_reg = VA_SPEAR1340_PERIP2_CLK_ENB,
	.en_reg_bit = SPEAR1340_I2S_REF_PAD_CLK_ENB,
	.pclk = &i2s_ref_clk,
	.recalc = &follow_parent,
};

/* i2s sclk aux rate configuration table, in ascending order of rates */
static struct aux_rate_tbl i2s_sclk_aux_rtbl[] = {
	/* For sclk = ref_clk * x/2/y */
	{.xscale = 1, .yscale = 4, .eq = 0},
	{.xscale = 1, .yscale = 2, .eq = 0},
};

/* i2s sclk (bit clock) syynthesizers masks */
static struct aux_clk_masks i2s_sclk_aux_masks = {
	.eq_sel_mask = SPEAR1340_AUX_EQ_SEL_MASK,
	.eq_sel_shift = SPEAR1340_I2S_SCLK_EQ_SEL_SHIFT,
	.eq1_mask = SPEAR1340_AUX_EQ1_SEL,
	.eq2_mask = SPEAR1340_AUX_EQ2_SEL,
	.xscale_sel_mask = SPEAR1340_I2S_SCLK_X_MASK,
	.xscale_sel_shift = SPEAR1340_I2S_SCLK_X_SHIFT,
	.yscale_sel_mask = SPEAR1340_I2S_SCLK_Y_MASK,
	.yscale_sel_shift = SPEAR1340_I2S_SCLK_Y_SHIFT,
};

/* i2s sclk synth configurations */
static struct aux_clk_config i2s_sclk_synth_config = {
	.synth_reg = VA_SPEAR1340_I2S_CLK_CFG,
	.masks = &i2s_sclk_aux_masks,
};

/* i2s sclk (bit clock) */
static struct clk i2s_sclk_clk = {
	.en_reg = VA_SPEAR1340_I2S_CLK_CFG,
	.en_reg_bit = SPEAR1340_I2S_SCLK_SYNTH_ENB,
	.pclk = &i2s_ref_clk,
	.calc_rate = &aux_calc_rate,
	.recalc = &aux_clk_recalc,
	.set_rate = &aux_clk_set_rate,
	.rate_config = {i2s_sclk_aux_rtbl, ARRAY_SIZE(i2s_sclk_aux_rtbl), 0},
	.private_data = &i2s_sclk_synth_config,
};

/* clock derived from ahb clk */

/* i2c0 clock */
static struct clk i2c0_clk = {
	.en_reg = VA_SPEAR1340_PERIP1_CLK_ENB,
	.en_reg_bit = SPEAR1340_I2C0_CLK_ENB,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

/* i2c1 clock */
static struct clk i2c1_clk = {
	.en_reg = VA_SPEAR1340_PERIP3_CLK_ENB,
	.en_reg_bit = SPEAR1340_I2C1_CLK_ENB,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

/* dma clock */
static struct clk dma_pclk = {
	.en_reg = VA_SPEAR1340_PERIP1_CLK_ENB,
	.en_reg_bit = SPEAR1340_DMA_CLK_ENB,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

static struct clk dma0_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &dma_pclk,
	.recalc = &follow_parent,
};

static struct clk dma1_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &dma_pclk,
	.recalc = &follow_parent,
};

/* gmac clock */
static struct clk gmac_clk = {
	.en_reg = VA_SPEAR1340_PERIP1_CLK_ENB,
	.en_reg_bit = SPEAR1340_GMAC_CLK_ENB,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

/* fsmc clock */
static struct clk fsmc_clk = {
	.en_reg = VA_SPEAR1340_PERIP1_CLK_ENB,
	.en_reg_bit = SPEAR1340_FSMC_CLK_ENB,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

/* fsmc-nand clock */
static struct clk fsmc_nand_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &fsmc_clk,
	.recalc = &follow_parent,
};

/* fsmc-nor clock */
static struct clk fsmc_nor_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &fsmc_clk,
	.recalc = &follow_parent,
};

/* smi clock */
static struct clk smi_clk = {
	.en_reg = VA_SPEAR1340_PERIP1_CLK_ENB,
	.en_reg_bit = SPEAR1340_SMI_CLK_ENB,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

/* uhc0 clock */
static struct clk uhci0_clk = {
	.en_reg = VA_SPEAR1340_PERIP1_CLK_ENB,
	.en_reg_bit = SPEAR1340_UHC0_CLK_ENB,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

/* uhc1 clock */
static struct clk uhci1_clk = {
	.en_reg = VA_SPEAR1340_PERIP1_CLK_ENB,
	.en_reg_bit = SPEAR1340_UHC1_CLK_ENB,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

/* uoc clock */
static struct clk uoc_clk = {
	.en_reg = VA_SPEAR1340_PERIP1_CLK_ENB,
	.en_reg_bit = SPEAR1340_UOC_CLK_ENB,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

/* pcie-sata parent clock */
static struct clk pcie_sata_pclk = {
	.en_reg = VA_SPEAR1340_PERIP1_CLK_ENB,
	.en_reg_bit = SPEAR1340_PCIE_SATA_CLK_ENB,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

/* pcie clock */
static struct clk pcie_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &pcie_sata_pclk,
	.recalc = &follow_parent,
};

/* sata clock */
static struct clk sata_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &pcie_sata_pclk,
	.recalc = &follow_parent,
};

/* sysram clocks */
static struct clk sysram0_clk = {
	.en_reg = VA_SPEAR1340_PERIP1_CLK_ENB,
	.en_reg_bit = SPEAR1340_SYSRAM0_CLK_ENB,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

static struct clk sysram1_clk = {
	.en_reg = VA_SPEAR1340_PERIP1_CLK_ENB,
	.en_reg_bit = SPEAR1340_SYSRAM1_CLK_ENB,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

/* adc configurations */
static struct aux_clk_config adc_synth_config = {
	.synth_reg = VA_SPEAR1340_ADC_CLK_SYNT,
	.masks = &aux_masks,
};

/* adc rate configuration table, in ascending order of rates */
/* possible adc range is 2.5 MHz to 20 MHz. */
static struct aux_rate_tbl adc_rtbl[] = {
	/* For ahb = 166.67 MHz */
	{.xscale = 1, .yscale = 31, .eq = 0}, /* 2.68 MHz */
	{.xscale = 2, .yscale = 21, .eq = 0}, /* 7.94 MHz */
	{.xscale = 4, .yscale = 21, .eq = 0}, /* 15.87 MHz */
	{.xscale = 10, .yscale = 42, .eq = 0}, /* 19.84 MHz */
};

/* adc synth clock */
static struct clk adc_synth_clk = {
	.en_reg = VA_SPEAR1340_ADC_CLK_SYNT,
	.en_reg_bit = SPEAR1340_AUX_SYNT_ENB,
	.pclk = &ahb_clk,
	.calc_rate = &aux_calc_rate,
	.recalc = &aux_clk_recalc,
	.set_rate = &aux_clk_set_rate,
	.rate_config = {adc_rtbl, ARRAY_SIZE(adc_rtbl), 0},
	.private_data = &adc_synth_config,
};

/* adc clock */
static struct clk adc_clk = {
	.en_reg = VA_SPEAR1340_PERIP1_CLK_ENB,
	.en_reg_bit = SPEAR1340_ADC_CLK_ENB,
	.pclk = &adc_synth_clk,
	.recalc = &follow_parent,
};

/* clock derived from apb clk */
/* ssp clock */
static struct clk ssp_clk = {
	.en_reg = VA_SPEAR1340_PERIP1_CLK_ENB,
	.en_reg_bit = SPEAR1340_SSP_CLK_ENB,
	.pclk = &apb_clk,
	.recalc = &follow_parent,
};

/* gpio clock */
static struct clk gpio0_clk = {
	.en_reg = VA_SPEAR1340_PERIP1_CLK_ENB,
	.en_reg_bit = SPEAR1340_GPIO0_CLK_ENB,
	.pclk = &apb_clk,
	.recalc = &follow_parent,
};

/* gpio clock */
static struct clk gpio1_clk = {
	.en_reg = VA_SPEAR1340_PERIP1_CLK_ENB,
	.en_reg_bit = SPEAR1340_GPIO1_CLK_ENB,
	.pclk = &apb_clk,
	.recalc = &follow_parent,
};

/* i2s play clock */
static struct clk i2s_play_clk = {
	.en_reg = VA_SPEAR1340_PERIP1_CLK_ENB,
	.en_reg_bit = SPEAR1340_I2S_PLAY_CLK_ENB,
	.pclk = &apb_clk,
	.recalc = &follow_parent,
};

/* i2s rec clock */
static struct clk i2s_rec_clk = {
	.en_reg = VA_SPEAR1340_PERIP1_CLK_ENB,
	.en_reg_bit = SPEAR1340_I2S_REC_CLK_ENB,
	.pclk = &apb_clk,
	.recalc = &follow_parent,
};

/* keyboard clock */
static struct clk kbd_clk = {
	.en_reg = VA_SPEAR1340_PERIP2_CLK_ENB,
	.en_reg_bit = SPEAR1340_KBD_CLK_ENB,
	.pclk = &apb_clk,
	.recalc = &follow_parent,
};

/* general fractional synthesizers definitions */
static struct frac_synth_clk_config gen_synth0_config = {
	.synth_reg = VA_SPEAR1340_GEN_CLK_SYNT0,
	.masks = &frac_synth_masks,
};

static struct frac_synth_clk_config gen_synth1_config = {
	.synth_reg = VA_SPEAR1340_GEN_CLK_SYNT1,
	.masks = &frac_synth_masks,
};

static struct frac_synth_clk_config gen_synth2_config = {
	.synth_reg = VA_SPEAR1340_GEN_CLK_SYNT2,
	.masks = &frac_synth_masks,
};

static struct frac_synth_clk_config gen_synth3_config = {
	.synth_reg = VA_SPEAR1340_GEN_CLK_SYNT3,
	.masks = &frac_synth_masks,
};

/* GEN Fractional Synthesizer parents */
static struct pclk_info gen_synth0_1_pclk_info[] = {
	{
		.pclk = &vco1div4_clk,
		.pclk_val = SPEAR1340_GEN_SYNT0_1_VCO1_DIV4_VAL,
	}, {
		.pclk = &vco3div2_clk,
		.pclk_val = SPEAR1340_GEN_SYNT0_1_VCO3_DIV2_VAL,
	}, {
		.pclk = &pll3_clk,
		.pclk_val = SPEAR1340_GEN_SYNT0_1_PLL3_VAL,
	},
};

/* GEN Fractional Synthesizer-0 and 1 parent select structure */
static struct pclk_sel gen_synth0_1_pclk_sel = {
	.pclk_info = gen_synth0_1_pclk_info,
	.pclk_count = ARRAY_SIZE(gen_synth0_1_pclk_info),
	.pclk_sel_reg = VA_SPEAR1340_PLL_CFG,
	.pclk_sel_mask = SPEAR1340_GEN_SYNT0_1_CLK_MASK,
};

static struct pclk_info gen_synth2_3_pclk_info[] = {
	{
		.pclk = &vco1div4_clk,
		.pclk_val = SPEAR1340_GEN_SYNT2_3_VCO1_DIV4_VAL,
	}, {
		.pclk = &vco2div2_clk,
		.pclk_val = SPEAR1340_GEN_SYNT2_3_VCO2_DIV2_VAL,
	}, {
		.pclk = &pll2_clk,
		.pclk_val = SPEAR1340_GEN_SYNT2_3_PLL2_VAL,
	},
};

/* GEN Fractional Synthesizer-2 and 3 parent select structure */
static struct pclk_sel gen_synth2_3_pclk_sel = {
	.pclk_info = gen_synth2_3_pclk_info,
	.pclk_count = ARRAY_SIZE(gen_synth2_3_pclk_info),
	.pclk_sel_reg = VA_SPEAR1340_PLL_CFG,
	.pclk_sel_mask = SPEAR1340_GEN_SYNT2_3_CLK_MASK,
};

/* GEN rate configuration table, in ascending order of rates */
static struct frac_synth_rate_tbl gen_rtbl[] = {
	/* For vco1div4 = 250 MHz */
	{.div = 0x1624E}, /* 22.5792 MHz */
	{.div = 0x14585}, /* 24.576 MHz */
	{.div = 0x14000}, /* 25 MHz */
	{.div = 0x0B127}, /* 45.1584 MHz */
	{.div = 0x0A000}, /* 50 MHz */
	{.div = 0x061A8}, /* 81.92 MHz */
	{.div = 0x05000}, /* 100 MHz */
	{.div = 0x02800}, /* 200 MHz */
	{.div = 0x02620}, /* 210 MHz */
	{.div = 0x02460}, /* 220 MHz */
	{.div = 0x022C0}, /* 230 MHz */
	{.div = 0x02160}, /* 240 MHz */
	{.div = 0x02000}, /* 250 MHz */
};

/* GEN Fractional Synthesizer-0 Clock */
static struct clk gen_synth0_clk = {
	.en_reg = VA_SPEAR1340_GEN_CLK_SYNT0,
	.en_reg_bit = SPEAR1340_FRAC_SYNT_ENB,
	.pclk_sel = &gen_synth0_1_pclk_sel,
	.pclk_sel_shift = SPEAR1340_GEN_SYNT0_1_CLK_SHIFT,
	.calc_rate = &frac_synth_calc_rate,
	.recalc = &frac_synth_clk_recalc,
	.set_rate = &frac_synth_clk_set_rate,
	.rate_config = {gen_rtbl, ARRAY_SIZE(gen_rtbl), 4},
	.private_data = &gen_synth0_config,
};

/* GEN Fractional Synthesizer1 Clock */
static struct clk gen_synth1_clk = {
	.en_reg = VA_SPEAR1340_GEN_CLK_SYNT1,
	.en_reg_bit = SPEAR1340_FRAC_SYNT_ENB,
	.pclk_sel = &gen_synth0_1_pclk_sel,
	.pclk_sel_shift = SPEAR1340_GEN_SYNT0_1_CLK_SHIFT,
	.calc_rate = &frac_synth_calc_rate,
	.recalc = &frac_synth_clk_recalc,
	.set_rate = &frac_synth_clk_set_rate,
	.rate_config = {gen_rtbl, ARRAY_SIZE(gen_rtbl), 4},
	.private_data = &gen_synth1_config,
};

/* GEN Fractional Synthesizer2 Clock */
static struct clk gen_synth2_clk = {
	.en_reg = VA_SPEAR1340_GEN_CLK_SYNT2,
	.en_reg_bit = SPEAR1340_FRAC_SYNT_ENB,
	.pclk_sel = &gen_synth2_3_pclk_sel,
	.pclk_sel_shift = SPEAR1340_GEN_SYNT2_3_CLK_SHIFT,
	.calc_rate = &frac_synth_calc_rate,
	.recalc = &frac_synth_clk_recalc,
	.set_rate = &frac_synth_clk_set_rate,
	.rate_config = {gen_rtbl, ARRAY_SIZE(gen_rtbl), 4},
	.private_data = &gen_synth2_config,
};

/* GEN Fractional Synthesizer3 Clock */
static struct clk gen_synth3_clk = {
	.en_reg = VA_SPEAR1340_GEN_CLK_SYNT3,
	.en_reg_bit = SPEAR1340_FRAC_SYNT_ENB,
	.pclk_sel = &gen_synth2_3_pclk_sel,
	.pclk_sel_shift = SPEAR1340_GEN_SYNT2_3_CLK_SHIFT,
	.calc_rate = &frac_synth_calc_rate,
	.recalc = &frac_synth_clk_recalc,
	.set_rate = &frac_synth_clk_set_rate,
	.rate_config = {gen_rtbl, ARRAY_SIZE(gen_rtbl), 4},
	.private_data = &gen_synth3_config,
};

/* mali clock */
static struct clk mali_clk = {
	.en_reg = VA_SPEAR1340_PERIP3_CLK_ENB,
	.en_reg_bit = SPEAR1340_MALI_CLK_ENB,
	.pclk = &gen_synth3_clk,
	.recalc = &follow_parent,
};

/* cec0 clock */
static struct clk cec0_clk = {
	.en_reg = VA_SPEAR1340_PERIP3_CLK_ENB,
	.en_reg_bit = SPEAR1340_CEC0_CLK_ENB,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

/* cec1 clock */
static struct clk cec1_clk = {
	.en_reg = VA_SPEAR1340_PERIP3_CLK_ENB,
	.en_reg_bit = SPEAR1340_CEC1_CLK_ENB,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

/* SPDIF Out clk */
static struct pclk_info spdif_out_pclk_info[] = {
	{
		.pclk = &gen_synth2_clk,
		.pclk_val = SPEAR1340_SPDIF_OUT_GSYNT2_VAL,
	}, {
		.pclk = &i2s_src_pad_clk,
		.pclk_val = SPEAR1340_SPDIF_OUT_I2S_PAD_VAL,
	},
};

static struct pclk_sel spdif_out_pclk_sel = {
	.pclk_info = spdif_out_pclk_info,
	.pclk_count = ARRAY_SIZE(spdif_out_pclk_info),
	.pclk_sel_reg = VA_SPEAR1340_PERIP_CLK_CFG,
	.pclk_sel_mask = SPEAR1340_SPDIF_CLK_MASK,
};

static struct clk spdif_out_clk = {
	.en_reg = VA_SPEAR1340_PERIP3_CLK_ENB,
	.en_reg_bit = SPEAR1340_SPDIF_OUT_CLK_ENB,
	.pclk_sel = &spdif_out_pclk_sel,
	.pclk_sel_shift = SPEAR1340_SPDIF_OUT_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* SPDIF In clk */
static struct pclk_info spdif_in_pclk_info[] = {
	{
		.pclk = &pll2_clk,
		.pclk_val = SPEAR1340_SPDIF_IN_PLL2_VAL,
	}, {
		.pclk = &gen_synth3_clk,
		.pclk_val = SPEAR1340_SPDIF_IN_GSYNT3_VAL,
	},
};

static struct pclk_sel spdif_in_pclk_sel = {
	.pclk_info = spdif_in_pclk_info,
	.pclk_count = ARRAY_SIZE(spdif_in_pclk_info),
	.pclk_sel_reg = VA_SPEAR1340_PERIP_CLK_CFG,
	.pclk_sel_mask = SPEAR1340_SPDIF_CLK_MASK,
};

static struct clk spdif_in_clk = {
	.en_reg = VA_SPEAR1340_PERIP3_CLK_ENB,
	.en_reg_bit = SPEAR1340_SPDIF_IN_CLK_ENB,
	.pclk_sel = &spdif_in_pclk_sel,
	.pclk_sel_shift = SPEAR1340_SPDIF_IN_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* ACP clock */
static struct clk acp_clk = {
	.en_reg = VA_SPEAR1340_PERIP2_CLK_ENB,
	.en_reg_bit = SPEAR1340_ACP_CLK_ENB,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

/* PLGPIO clock */
static struct clk plgpio_clk = {
	.en_reg = VA_SPEAR1340_PERIP3_CLK_ENB,
	.en_reg_bit = SPEAR1340_PLGPIO_CLK_ENB,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

/* VIDEO_DEC clock */
static struct clk video_dec_clk = {
	.en_reg = VA_SPEAR1340_PERIP3_CLK_ENB,
	.en_reg_bit = SPEAR1340_VIDEO_DEC_CLK_ENB,
	.pclk = &gen_synth0_clk,
	.recalc = &follow_parent,
};

/* VIDEO_ENC clock */
static struct clk video_enc_clk = {
	.en_reg = VA_SPEAR1340_PERIP3_CLK_ENB,
	.en_reg_bit = SPEAR1340_VIDEO_ENC_CLK_ENB,
	.pclk = &gen_synth1_clk,
	.recalc = &follow_parent,
};

/* VIDEO Input Port clock */
static struct clk video_input_port_clk = {
	.en_reg = VA_SPEAR1340_PERIP3_CLK_ENB,
	.en_reg_bit = SPEAR1340_VIDEO_IN_CLK_ENB,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

/* CAM0 clock */
static struct clk cam0_clk = {
	.en_reg = VA_SPEAR1340_PERIP3_CLK_ENB,
	.en_reg_bit = SPEAR1340_CAM0_CLK_ENB,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

/* CAM1 clock */
static struct clk cam1_clk = {
	.en_reg = VA_SPEAR1340_PERIP3_CLK_ENB,
	.en_reg_bit = SPEAR1340_CAM1_CLK_ENB,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

/* CAM2 clock */
static struct clk cam2_clk = {
	.en_reg = VA_SPEAR1340_PERIP3_CLK_ENB,
	.en_reg_bit = SPEAR1340_CAM2_CLK_ENB,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

/* CAM3 clock */
static struct clk cam3_clk = {
	.en_reg = VA_SPEAR1340_PERIP3_CLK_ENB,
	.en_reg_bit = SPEAR1340_CAM3_CLK_ENB,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

/* pwm clock */
static struct clk pwm_clk = {
	.en_reg = VA_SPEAR1340_PERIP3_CLK_ENB,
	.en_reg_bit = SPEAR1340_PWM_CLK_ENB,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

static struct clk dummy_apb_pclk;

/* array of all spear 13xx clock lookups */
static struct clk_lookup spear1340_clk_lookups[] = {
	{ .con_id = "apb_pclk",			.clk = &dummy_apb_pclk},
	/* root clks */
	{.con_id = "osc1_24m_clk",		.clk = &osc1_24m_clk},
	{.con_id = "osc2_32k_clk",		.clk = &osc2_32k_clk},
	{.con_id = "osc3_25m_clk",		.clk = &osc3_25m_clk},

	/* clock derived from 32 KHz osc clk */
	{.dev_id = "rtc-spear",			.clk = &rtc_clk},

	/* clock derived from 24/25 MHz osc1/osc3 clk */
	{.con_id = "vco1_clk",			.clk = &vco1_clk},
	{.con_id = "vco2_clk",			.clk = &vco2_clk},
	{.con_id = "vco3_clk",			.clk = &vco3_clk},
	{.con_id = "vco4_clk",			.clk = &vco4_clk},
	{.con_id = "pll5_clk",			.clk = &pll5_clk},
	{.con_id = "pll6_clk",			.clk = &pll6_clk},

	/* clock derived from vco1-5 clk */
	{.con_id = "pll1_clk",			.clk = &pll1_clk},
	{.con_id = "pll2_clk",			.clk = &pll2_clk},
	{.con_id = "pll3_clk",			.clk = &pll3_clk},
	{.con_id = "pll4_clk",			.clk = &pll4_clk},
	{.con_id = "vco1div2_clk",		.clk = &vco1div2_clk},
	{.con_id = "vco1div4_clk",		.clk = &vco1div4_clk},
	{.con_id = "vco2div2_clk",		.clk = &vco2div2_clk},
	{.con_id = "vco3div2_clk",		.clk = &vco3div2_clk},

	/* System synth clk */
	{.con_id = "sys_synth_clk",		.clk = &sys_synth_clk},
	{.con_id = "amba_synth_clk",		.clk = &amba_synth_clk},

	/* clock derived from pll1 clk */
	{.con_id = "ddr_clk",			.clk = &ddr_clk},

	/* sys clk */
	{.con_id = "sys_clk",			.clk = &sys_clk},

	/* clock derived from sys clk */
	{.con_id = "cpu_clk",			.clk = &cpu_clk},

	/* clock derived from cpu clock */
	{.con_id = "cpu_clk_div3",		.clk = &cpu_clk_div3},
	{.con_id = "ahb_clk",			.clk = &ahb_clk},
	{.con_id = "apb_clk",			.clk = &apb_clk},

	/* synthesizers/prescaled clocks */
	{.con_id = "c3_synth_clk",		.clk = &c3_synth_clk},
	{.con_id = "gmii_125m_pad_clk",		.clk = &gmii_125m_pad},
	{.con_id = "clcd_synth_clk",		.clk = &clcd_synth_clk},
	{.con_id = "uart0_synth_clk",		.clk = &uart0_synth_clk},
	{.con_id = "uart1_synth_clk",		.clk = &uart1_synth_clk},
	{.con_id = "sdhci_synth_clk",		.clk = &sdhci_synth_clk},
	{.con_id = "cfxd_synth_clk",		.clk = &cfxd_synth_clk},
	{.con_id = "gmac_phy_input_clk",	.clk = &gmac_phy_input_clk},
	{.con_id = "gmac_phy_synth_clk",	.clk = &gmac_phy_synth_clk},
	{.con_id = "stmmacphy.0",		.clk = &gmac_phy0_clk},
	{.dev_id = "gen_synth0_clk",		.clk = &gen_synth0_clk},
	{.dev_id = "gen_synth1_clk",		.clk = &gen_synth1_clk},
	{.dev_id = "gen_synth2_clk",		.clk = &gen_synth2_clk},
	{.dev_id = "gen_synth3_clk",		.clk = &gen_synth3_clk},

	/* i2s refout and sclk clks */
	{.con_id = "i2s_src_pad_clk",		.clk = &i2s_src_pad_clk},
	{.con_id = "i2s_src_clk",		.clk = &i2s_src_clk},
	{.con_id = "i2s_prs1_clk",		.clk = &i2s_prs1_clk},
	{.con_id = "i2s_ref_clk",		.clk = &i2s_ref_clk},
	{.con_id = "i2s_ref_pad_clk",		.clk = &i2s_ref_pad_clk},
	{.con_id = "i2s_sclk_clk",		.clk = &i2s_sclk_clk},

	/* cec clks */
	{.dev_id = "spear_cec.0",		.clk = &cec0_clk},
	{.dev_id = "spear_cec.1",		.clk = &cec1_clk},

	/* clocks having multiple parent source from above clocks */
	{.dev_id = "clcd_pixel_clk",		.clk = &clcd_pixel_clk},
	{.dev_id = "clcd-db9000",		.clk = &clcd_clk},
	{.dev_id = "gpt0",			.clk = &gpt0_clk},
	{.dev_id = "gpt1",			.clk = &gpt1_clk},
	{.dev_id = "gpt2",			.clk = &gpt2_clk},
	{.dev_id = "gpt3",			.clk = &gpt3_clk},
	{.dev_id = "uart",			.clk = &uart0_clk},
	{.dev_id = "uart1",			.clk = &uart1_clk},
	{.dev_id = "sdhci",			.clk = &sdhci_clk},
	{.dev_id = "arasan_cf",			.clk = &cfxd_clk},
	{.dev_id = "arasan_xd",			.clk = &cfxd_clk},
	{.dev_id = "mali",			.clk = &mali_clk},
	{.dev_id = "spdif-out",			.clk = &spdif_out_clk},
	{.dev_id = "spdif-in",			.clk = &spdif_in_clk},
	{.dev_id = "video_dec",			.clk = &video_dec_clk},
	{.dev_id = "video_enc",			.clk = &video_enc_clk},

	/* clock derived from ahb clk */
	{.dev_id = "smi",			.clk = &smi_clk},
	{.con_id = "usbh.0_clk",		.clk = &uhci0_clk},
	{.con_id = "usbh.1_clk",		.clk = &uhci1_clk},
	{.dev_id = "uoc",			.clk = &uoc_clk},
	{.dev_id = "i2c_designware.0",		.clk = &i2c0_clk},
	{.dev_id = "i2c_designware.1",		.clk = &i2c1_clk},
	{.con_id = "dmac_pclk",			.clk = &dma_pclk},
	{.dev_id = "dw_dmac.0",			.clk = &dma0_clk},
	{.dev_id = "dw_dmac.1",			.clk = &dma1_clk},
	{.dev_id = "stmmaceth.0",		.clk = &gmac_clk},
	{.dev_id = "c3",			.clk = &c3_clk},
	{.con_id = "pcie_sata_pclk",		.clk = &pcie_sata_pclk},
	{.dev_id = "dw_pcie.0",			.clk = &pcie_clk},
	{.dev_id = "ahci",			.clk = &sata_clk},
	{.con_id = "fsmc",			.clk = &fsmc_clk},
	{.dev_id = "fsmc-nand",                 .clk = &fsmc_nand_clk},
	{.dev_id = "fsmc-nor",                  .clk = &fsmc_nor_clk},
	{.dev_id = "sysram0",			.clk = &sysram0_clk},
	{.dev_id = "sysram1",			.clk = &sysram1_clk},
	{.dev_id = "acp_clk",			.clk = &acp_clk},
	{.dev_id = "spear_camif.0",		.clk = &cam0_clk},
	{.dev_id = "spear_camif.1",		.clk = &cam1_clk},
	{.dev_id = "spear_camif.2",		.clk = &cam2_clk},
	{.dev_id = "spear_camif.3",		.clk = &cam3_clk},
	{.dev_id = "plgpio",			.clk = &plgpio_clk},
	{.dev_id = "pwm",			.clk = &pwm_clk},
	{.dev_id = "spear_vip",			.clk = &video_input_port_clk},
	{.con_id = "adc_synth_clk",		.clk = &adc_synth_clk},
	{.dev_id = "adc",			.clk = &adc_clk},

	/* clock derived from apb clk */
	{.dev_id = "designware-i2s.0",		.clk = &i2s_play_clk},
	{.dev_id = "designware-i2s.1",		.clk = &i2s_rec_clk},
	{.dev_id = "ssp-pl022",			.clk = &ssp_clk},
	{.dev_id = "gpio0",			.clk = &gpio0_clk},
	{.dev_id = "gpio1",			.clk = &gpio1_clk},
	{.dev_id = "keyboard",			.clk = &kbd_clk},
	{.dev_id = "cortexa9-wdt",		.clk = &wdt_clk},
	{.dev_id = "spear_thermal",		.clk = &thermal_clk},
};

/* machine clk init */
void __init spear1340_clk_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(spear1340_clk_lookups); i++)
		clk_register(&spear1340_clk_lookups[i]);

	clk_init(&ddr_clk);
}
