/*
 * arch/arm/mach-spear13xx/clock.c
 *
 * SPEAr13xx machines clock framework source file
 *
 * Copyright (C) 2010 ST Microelectronics
 * shiraz hashim<shiraz.hashim@st.com>
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
#include <mach/misc_regs.h>

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
	.pclk = &osc2_32k_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = RTC_CLK_ENB,
	.recalc = &follow_parent,
};

/* clock derived from osc1 or osc3 */
/* vco[1-3] parents */
static struct pclk_info vco_pclk_info[] = {
	{
		.pclk = &osc1_24m_clk,
		.pclk_val = OSC_24M_VAL,
	}, {
		.pclk = &osc3_25m_clk,
		.pclk_val = OSC_25M_VAL,
	},
};

/* vco[1-3] parent select structure */
static struct pclk_sel vco_pclk_sel = {
	.pclk_info = vco_pclk_info,
	.pclk_count = ARRAY_SIZE(vco_pclk_info),
	.pclk_sel_reg = VA_PLL_CFG,
	.pclk_sel_mask = PLL_CLK_MASK,
};

/* vco masks structure */
static struct vco_clk_masks vco_masks = {
	.mode_mask = PLL_MODE_MASK,
	.mode_shift = PLL_MODE_SHIFT,
	.norm_fdbk_m_mask = PLL_NORM_FDBK_M_MASK,
	.norm_fdbk_m_shift = PLL_NORM_FDBK_M_SHIFT,
	.dith_fdbk_m_mask = PLL_DITH_FDBK_M_MASK,
	.dith_fdbk_m_shift = PLL_DITH_FDBK_M_SHIFT,
	.div_p_mask = PLL_DIV_P_MASK,
	.div_p_shift = PLL_DIV_P_SHIFT,
	.div_n_mask = PLL_DIV_N_MASK,
	.div_n_shift = PLL_DIV_N_SHIFT,
	.pll_lock_mask = PLL_LOCK_MASK,
	.pll_lock_shift = PLL_LOCK_SHIFT,
};
/* vco1 configuration structure */
static struct vco_clk_config vco1_config = {
	.mode_reg = VA_PLL1_CTR,
	.cfg_reg = VA_PLL1_FRQ,
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
};

/* vco1 clock */
static struct clk vco1_clk = {
	.flags = ENABLED_ON_INIT | SYSTEM_CLK,
	.pclk_sel = &vco_pclk_sel,
	.pclk_sel_shift = PLL1_CLK_SHIFT,
	.en_reg = VA_PLL1_CTR,
	.en_reg_bit = PLL_ENABLE,
	.calc_rate = &vco_calc_rate,
	.recalc = &vco_clk_recalc,
	.set_rate = &vco_clk_set_rate,
	.rate_config = {vco_rtbl, ARRAY_SIZE(vco_rtbl), 6},
	.private_data = &vco1_config,
};

/* thermal clock */
static struct clk thermal_clk = {
	.en_reg = VA_PERIP2_CLK_ENB,
	.en_reg_bit = THSENS_CLK_ENB,
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

/* vco1div4 clock */
static struct clk vco1div4_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.pclk = &vco1_clk,
	.div_factor = 4,
	.recalc = &follow_parent,
};

/* vco2 configuration structure */
static struct vco_clk_config vco2_config = {
	.mode_reg = VA_PLL2_CTR,
	.cfg_reg = VA_PLL2_FRQ,
	.masks = &vco_masks,
};

/* vco2 clock */
static struct clk vco2_clk = {
	.flags = SYSTEM_CLK,
	.pclk_sel = &vco_pclk_sel,
	.pclk_sel_shift = PLL2_CLK_SHIFT,
	.en_reg = VA_PLL2_CTR,
	.en_reg_bit = PLL_ENABLE,
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
	.mode_reg = VA_PLL3_CTR,
	.cfg_reg = VA_PLL3_FRQ,
	.masks = &vco_masks,
};

/* vco3 clock */
static struct clk vco3_clk = {
	.flags = SYSTEM_CLK,
	.pclk_sel = &vco_pclk_sel,
	.pclk_sel_shift = PLL3_CLK_SHIFT,
	.en_reg = VA_PLL3_CTR,
	.en_reg_bit = PLL_ENABLE,
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
	.mode_reg = VA_PLL4_CTR,
	.cfg_reg = VA_PLL4_FRQ,
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
	.pclk = &osc1_24m_clk,
	.en_reg = VA_PLL4_CTR,
	.en_reg_bit = PLL_ENABLE,
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

static struct pclk_info ddr_pclk_info[] = {
	{
		.pclk = &pll1_clk,
		.pclk_val = MCTR_CLK_PLL1_VAL,
	}, {
		.pclk = &pll4_clk,
		.pclk_val = MCTR_CLK_PLL4_VAL,
	},
};

/* ddr parent select structure */
static struct pclk_sel ddr_pclk_sel = {
	.pclk_info = ddr_pclk_info,
	.pclk_count = ARRAY_SIZE(ddr_pclk_info),
	.pclk_sel_reg = VA_PERIP_CLK_CFG,
	.pclk_sel_mask = MCTR_CLK_MASK,
};

static struct clk ddr_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.recalc = &follow_parent,
	.pclk_sel = &ddr_pclk_sel,
	.pclk_sel_shift = MCTR_CLK_SHIFT,
	.private_data = &ddr_rate_tbl,
};

/* cpu clock */
static struct clk cpu_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.pclk = &pll1_clk,
	.div_factor = 2,
	.recalc = &follow_parent,
};

/* ahb clock */
static struct clk ahb_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.pclk = &pll1_clk,
	.div_factor = 6,
	.recalc = &follow_parent,
};

/* apb clock */
static struct clk apb_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.pclk = &pll1_clk,
	.div_factor = 12,
	.recalc = &follow_parent,
};

/* clocks derived from osc1, ahb or apb */
/* gpt[0-3] parents */
static struct pclk_info gpt_pclk_info[] = {
	{
		.pclk = &osc1_24m_clk,
		.pclk_val = GPT_OSC24_VAL,
	}, {
		.pclk = &apb_clk,
		.pclk_val = GPT_APB_VAL,
	},
};

/* gpt[0-3] parent select structure */
static struct pclk_sel gpt_pclk_sel = {
	.pclk_info = gpt_pclk_info,
	.pclk_count = ARRAY_SIZE(gpt_pclk_info),
	.pclk_sel_reg = VA_PERIP_CLK_CFG,
	.pclk_sel_mask = GPT_CLK_MASK,
};

/* gpt0 timer clock */
static struct clk gpt0_clk = {
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = GPT0_CLK_ENB,
	.pclk_sel = &gpt_pclk_sel,
	.pclk_sel_shift = GPT0_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* gpt1 timer clock */
static struct clk gpt1_clk = {
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = GPT1_CLK_ENB,
	.pclk_sel = &gpt_pclk_sel,
	.pclk_sel_shift = GPT1_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* gpt2 timer clock */
static struct clk gpt2_clk = {
	.en_reg = VA_PERIP2_CLK_ENB,
	.en_reg_bit = GPT2_CLK_ENB,
	.pclk_sel = &gpt_pclk_sel,
	.pclk_sel_shift = GPT2_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* gpt3 timer clock */
static struct clk gpt3_clk = {
	.en_reg = VA_PERIP2_CLK_ENB,
	.en_reg_bit = GPT3_CLK_ENB,
	.pclk_sel = &gpt_pclk_sel,
	.pclk_sel_shift = GPT3_CLK_SHIFT,
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
	.eq_sel_mask = AUX_EQ_SEL_MASK,
	.eq_sel_shift = AUX_EQ_SEL_SHIFT,
	.eq1_mask = AUX_EQ1_SEL,
	.eq2_mask = AUX_EQ2_SEL,
	.xscale_sel_mask = AUX_XSCALE_MASK,
	.xscale_sel_shift = AUX_XSCALE_SHIFT,
	.yscale_sel_mask = AUX_YSCALE_MASK,
	.yscale_sel_shift = AUX_YSCALE_SHIFT,
};

/* clocks derived multiple parents (pll1, pll5, synthesizers or others) */
/* uart configurations */
static struct aux_clk_config uart_synth_config = {
	.synth_reg = VA_UART_CLK_SYNT,
	.masks = &aux_masks,
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

/* uart synth clock */
static struct clk uart_synth_clk = {
	.en_reg = VA_UART_CLK_SYNT,
	.en_reg_bit = AUX_SYNT_ENB,
	.pclk = &vco1div2_clk,
	.calc_rate = &aux_calc_rate,
	.recalc = &aux_clk_recalc,
	.set_rate = &aux_clk_set_rate,
	.rate_config = {aux_rtbl, ARRAY_SIZE(aux_rtbl), 2},
	.private_data = &uart_synth_config,
};

/* uart parents */
static struct pclk_info uart_pclk_info[] = {
	{
		.pclk = &pll5_clk,
		.pclk_val = AUX_CLK_PLL5_VAL,
	}, {
		.pclk = &uart_synth_clk,
		.pclk_val = AUX_CLK_SYNT_VAL,
	},
};

/* uart parent select structure */
static struct pclk_sel uart_pclk_sel = {
	.pclk_info = uart_pclk_info,
	.pclk_count = ARRAY_SIZE(uart_pclk_info),
	.pclk_sel_reg = VA_PERIP_CLK_CFG,
	.pclk_sel_mask = UART_CLK_MASK,
};

/* uart clock */
static struct clk uart_clk = {
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = UART_CLK_ENB,
	.pclk_sel = &uart_pclk_sel,
	.pclk_sel_shift = UART_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* sdhci configurations */
static struct aux_clk_config sdhci_synth_config = {
	.synth_reg = VA_SDHCI_CLK_SYNT,
	.masks = &aux_masks,
};

/* sdhci synth clock */
static struct clk sdhci_synth_clk = {
	.en_reg = VA_SDHCI_CLK_SYNT,
	.en_reg_bit = AUX_SYNT_ENB,
	.pclk = &vco1div2_clk,
	.calc_rate = &aux_calc_rate,
	.recalc = &aux_clk_recalc,
	.set_rate = &aux_clk_set_rate,
	.rate_config = {aux_rtbl, ARRAY_SIZE(aux_rtbl), 1},
	.private_data = &sdhci_synth_config,
};

/* sdhci clock */
static struct clk sdhci_clk = {
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = SDHCI_CLK_ENB,
	.pclk = &sdhci_synth_clk,
	.recalc = &follow_parent,
};

/* cfxd configurations */
static struct aux_clk_config cfxd_synth_config = {
	.synth_reg = VA_CFXD_CLK_SYNT,
	.masks = &aux_masks,
};

/* cfxd synth clock */
static struct clk cfxd_synth_clk = {
	.en_reg = VA_CFXD_CLK_SYNT,
	.en_reg_bit = AUX_SYNT_ENB,
	.pclk = &vco1div2_clk,
	.calc_rate = &aux_calc_rate,
	.recalc = &aux_clk_recalc,
	.set_rate = &aux_clk_set_rate,
	.rate_config = {aux_rtbl, ARRAY_SIZE(aux_rtbl), 4},
	.private_data = &cfxd_synth_config,
};

/* cfxd clock */
static struct clk cfxd_clk = {
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = CFXD_CLK_ENB,
	.pclk = &cfxd_synth_clk,
	.recalc = &follow_parent,
};

/* C3 clk configurations */
static struct aux_clk_config c3_synth_config = {
	.synth_reg = VA_C3_CLK_SYNT,
	.masks = &aux_masks,
};

/* c3 synth clock */
static struct clk c3_synth_clk = {
	.en_reg = VA_C3_CLK_SYNT,
	.en_reg_bit = AUX_SYNT_ENB,
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
		.pclk_val = AUX_CLK_PLL5_VAL,
	}, {
		.pclk = &c3_synth_clk,
		.pclk_val = AUX_CLK_SYNT_VAL,
	},
};

/* c3 parent select structure */
static struct pclk_sel c3_pclk_sel = {
	.pclk_info = c3_pclk_info,
	.pclk_count = ARRAY_SIZE(c3_pclk_info),
	.pclk_sel_reg = VA_PERIP_CLK_CFG,
	.pclk_sel_mask = C3_CLK_MASK,
};

/* c3 clock */
static struct clk c3_clk = {
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = C3_CLK_ENB,
	.pclk_sel = &c3_pclk_sel,
	.pclk_sel_shift = C3_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* gmac phy clk configurations */
static struct aux_clk_config gmac_phy_synth_config = {
	.synth_reg = VA_GMAC_CLK_SYNT,
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
		.pclk_val = GMAC_PHY_125M_PAD_VAL,
	}, {
		.pclk = &pll2_clk,
		.pclk_val = GMAC_PHY_PLL2_VAL,
	}, {
		.pclk = &osc3_25m_clk,
		.pclk_val = GMAC_PHY_OSC3_VAL,
	},
};

static struct pclk_sel gmac_phy_input_pclk_sel = {
	.pclk_info = gmac_phy_input_pclk_info,
	.pclk_count = ARRAY_SIZE(gmac_phy_input_pclk_info),
	.pclk_sel_reg = VA_GMAC_CLK_CFG,
	.pclk_sel_mask = GMAC_PHY_INPUT_CLK_MASK,
};

static struct clk gmac_phy_input_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk_sel = &gmac_phy_input_pclk_sel,
	.pclk_sel_shift = GMAC_PHY_INPUT_CLK_SHIFT,
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
	.en_reg = VA_GMAC_CLK_SYNT,
	.en_reg_bit = AUX_SYNT_ENB,
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
		.pclk_val = GMAC_PHY_INPUT_ENB_VAL,
	}, {
		.pclk = &gmac_phy_synth_clk,
		.pclk_val = GMAC_PHY_SYNT_ENB_VAL,
	}
};

/* gmac phy parent select structure */
static struct pclk_sel gmac_phy_pclk_sel = {
	.pclk_info = gmac_phy_pclk_info,
	.pclk_count = ARRAY_SIZE(gmac_phy_pclk_info),
	.pclk_sel_reg = VA_GMAC_CLK_CFG,
	.pclk_sel_mask = GMAC_PHY_CLK_MASK,
};

/* gmac phy clock */
static struct clk gmac_phy0_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk_sel = &gmac_phy_pclk_sel,
	.pclk_sel_shift = GMAC_PHY_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* clcd fractional synthesizers masks */
static struct frac_synth_masks clcd_masks = {
	.div_factor_mask = FRAC_SYNT_DIV_FACTOR_MASK,
	.div_factor_shift = FRAC_SYNT_DIV_FACTOR_SHIFT,
};

static struct frac_synth_clk_config clcd_synth_config = {
	.synth_reg = VA_CLCD_CLK_SYNT,
	.masks = &clcd_masks,
};

/* clcd fractional synthesizer parents */
static struct pclk_info clcd_synth_pclk_info[] = {
	{
		.pclk = &vco1div4_clk,
		.pclk_val = CLCD_SYNT_VCO1_DIV4_VAL,
	}, {
		.pclk = &pll2_clk,
		.pclk_val = CLCD_SYNT_PLL2_VAL,
	},
};

/* clcd fractional synthesizer parent select structure */
static struct pclk_sel clcd_synth_pclk_sel = {
	.pclk_info = clcd_synth_pclk_info,
	.pclk_count = ARRAY_SIZE(clcd_synth_pclk_info),
	.pclk_sel_reg = VA_PLL_CFG,
	.pclk_sel_mask = CLCD_SYNT_CLK_MASK,
};

/* clcd rate configuration table, in ascending order of rates */
static struct frac_synth_rate_tbl clcd_rtbl[] = {
	{.div = 0x14000}, /* 25 Mhz , for vc01div4 = 250 MHz*/
	{.div = 0x1284B}, /* 27 Mhz , for vc01div4 = 250 MHz*/
	{.div = 0x0D8D3}, /* 58 Mhz , for vco1div4 = 393 MHz */
	{.div = 0x0B72C}, /* 58 Mhz , for vco1div4 = 332 MHz */
	{.div = 0x089EE}, /* 58 Mhz , for vc01div4 = 250 MHz*/
	{.div = 0x06f1C}, /* 72 Mhz , for vc01div4 = 250 MHz*/
	{.div = 0x06E58}, /* 58 Mhz , for vco1div4 = 200 MHz */
	{.div = 0x06c1B}, /* 74 Mhz , for vc01div4 = 250 MHz*/
	{.div = 0x04A12}, /* 108 Mhz , for vc01div4 = 250 MHz*/
	{.div = 0x0378E}, /* 144 Mhz , for vc01div4 = 250 MHz*/
	/*
	 * TODO : 1080p should work on 148 Mhz. But we see lots of
	 * flickering at 148 Mhz.So, commenting this entry till we
	 * resolve this issue
	 */
	/* {.div = 0x0360D}, */ /* 148 Mhz , for vc01div4 = 250 MHz*/
};

/* clcd fractional synthesizer clock */
static struct clk clcd_synth_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk_sel = &clcd_synth_pclk_sel,
	.pclk_sel_shift = CLCD_SYNT_CLK_SHIFT,
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
		.pclk_val = AUX_CLK_PLL5_VAL,
	}, {
		.pclk = &clcd_synth_clk,
		.pclk_val = AUX_CLK_SYNT_VAL,
	},
};

/* clcd parent select structure */
static struct pclk_sel clcd_pixel_pclk_sel = {
	.pclk_info = clcd_pixel_pclk_info,
	.pclk_count = ARRAY_SIZE(clcd_pixel_pclk_info),
	.pclk_sel_reg = VA_PERIP_CLK_CFG,
	.pclk_sel_mask = CLCD_CLK_MASK,
};

static struct clk clcd_pixel_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk_sel = &clcd_pixel_pclk_sel,
	.pclk_sel_shift = CLCD_CLK_SHIFT,
	.recalc = &follow_parent,
};

/*
 * clcd clock
 * There are 2 options for clcd clock,
 *  - derived from AHB (bus clk)
 *  - dervied from Pixel Clock Input (pixel clk)
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
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = CLCD_CLK_ENB,
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
		.pclk_val = I2S_SRC_VCODIV2_VAL,
	}, {
		.pclk = &pll3_clk,
		.pclk_val = I2S_SRC_PLL3_VAL,
	}, {
		.pclk = &i2s_src_pad_clk,
		.pclk_val = I2S_SRC_PL_CLK1_VAL,
	},
};

static struct pclk_sel i2s_src_pclk_sel = {
	.pclk_info = i2s_src_pclk_info,
	.pclk_count = ARRAY_SIZE(i2s_src_pclk_info),
	.pclk_sel_reg = VA_I2S_CLK_CFG,
	.pclk_sel_mask = I2S_SRC_CLK_MASK,
};

/* i2s src clock */
static struct clk i2s_src_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk_sel = &i2s_src_pclk_sel,
	.pclk_sel_shift = I2S_SRC_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* i2s prescaler1 masks */
static struct aux_clk_masks i2s_prs1_aux_masks = {
	.eq_sel_mask = AUX_EQ_SEL_MASK,
	.eq_sel_shift = I2S_PRS1_EQ_SEL_SHIFT,
	.eq1_mask = AUX_EQ1_SEL,
	.eq2_mask = AUX_EQ2_SEL,
	.xscale_sel_mask = I2S_PRS1_CLK_X_MASK,
	.xscale_sel_shift = I2S_PRS1_CLK_X_SHIFT,
	.yscale_sel_mask = I2S_PRS1_CLK_Y_MASK,
	.yscale_sel_shift = I2S_PRS1_CLK_Y_SHIFT,
};

/* i2s prs1 clk configurations */
static struct aux_clk_config i2s_prs1_config = {
	.synth_reg = VA_I2S_CLK_CFG,
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
		.pclk_val = I2S_REF_SRC_VAL,
	}, {
		.pclk = &i2s_prs1_clk,
		.pclk_val = I2S_REF_PRS1_VAL,
	},
};

/* i2s ref clock parent select structure */
static struct pclk_sel i2s_ref_pclk_sel = {
	.pclk_info = i2s_ref_pclk_info,
	.pclk_count = ARRAY_SIZE(i2s_ref_pclk_info),
	.pclk_sel_reg = VA_I2S_CLK_CFG,
	.pclk_sel_mask = I2S_REF_SEL_MASK,
};

/* i2s ref clock */
static struct clk i2s_ref_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk_sel = &i2s_ref_pclk_sel,
	.pclk_sel_shift = I2S_REF_SHIFT,
	.recalc = &follow_parent,
};

/* i2s ref pad clock: for codec */
static struct clk i2s_ref_pad_clk = {
	.en_reg = VA_PERIP2_CLK_ENB,
	.en_reg_bit = I2S_REF_PAD_CLK_ENB,
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
	.eq_sel_mask = AUX_EQ_SEL_MASK,
	.eq_sel_shift = I2S_SCLK_EQ_SEL_SHIFT,
	.eq1_mask = AUX_EQ1_SEL,
	.eq2_mask = AUX_EQ2_SEL,
	.xscale_sel_mask = I2S_SCLK_X_MASK,
	.xscale_sel_shift = I2S_SCLK_X_SHIFT,
	.yscale_sel_mask = I2S_SCLK_Y_MASK,
	.yscale_sel_shift = I2S_SCLK_Y_SHIFT,
};

/* i2s sclk synth configurations */
static struct aux_clk_config i2s_sclk_synth_config = {
	.synth_reg = VA_I2S_CLK_CFG,
	.masks = &i2s_sclk_aux_masks,
};

/* i2s sclk (bit clock) */
static struct clk i2s_sclk_clk = {
	.en_reg = VA_I2S_CLK_CFG,
	.en_reg_bit = I2S_SCLK_SYNTH_ENB,
	.pclk = &i2s_ref_clk,
	.calc_rate = &aux_calc_rate,
	.recalc = &aux_clk_recalc,
	.set_rate = &aux_clk_set_rate,
	.rate_config = {i2s_sclk_aux_rtbl, ARRAY_SIZE(i2s_sclk_aux_rtbl), 0},
	.private_data = &i2s_sclk_synth_config,
};

/* clock derived from ahb clk */

/* i2c clock */
static struct clk i2c_clk = {
	.pclk = &ahb_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = I2C_CLK_ENB,
	.recalc = &follow_parent,
};

/* dma clock */
static struct clk dma_pclk = {
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = DMA_CLK_ENB,
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

/* jpeg clock */
static struct clk jpeg_clk = {
	.pclk = &ahb_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = JPEG_CLK_ENB,
	.recalc = &follow_parent,
};

/* gmac clock :Fixed Part*/
static struct clk gmac0_clk = {
	.pclk = &ahb_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = GMAC_CLK_ENB,
	.recalc = &follow_parent,
};

/* fsmc clock */
static struct clk fsmc_clk = {
	.pclk = &ahb_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = FSMC_CLK_ENB,
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
	.pclk = &ahb_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = SMI_CLK_ENB,
	.recalc = &follow_parent,
};

/* uhc0 clock */
static struct clk uhci0_clk = {
	.pclk = &ahb_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = UHC0_CLK_ENB,
	.recalc = &follow_parent,
};

/* uhc1 clock */
static struct clk uhci1_clk = {
	.pclk = &ahb_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = UHC1_CLK_ENB,
	.recalc = &follow_parent,
};

/* usbd clock */
static struct clk usbd_clk = {
	.pclk = &ahb_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = USBD_CLK_ENB,
	.recalc = &follow_parent,
};

/* pci clocks */
static struct clk pcie0_clk = {
	.pclk = &ahb_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = PCIE0_CLK_ENB,
	.recalc = &follow_parent,
};

static struct clk pcie1_clk = {
	.pclk = &ahb_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = PCIE1_CLK_ENB,
	.recalc = &follow_parent,
};

static struct clk pcie2_clk = {
	.pclk = &ahb_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = PCIE2_CLK_ENB,
	.recalc = &follow_parent,
};

/* sysram clocks */
static struct clk sysram0_clk = {
	.pclk = &ahb_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = SYSRAM0_CLK_ENB,
	.recalc = &follow_parent,
};

static struct clk sysram1_clk = {
	.pclk = &ahb_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = SYSRAM1_CLK_ENB,
	.recalc = &follow_parent,
};

/* clock derived from apb clk */
/* adc clock */
static struct clk adc_clk = {
	.pclk = &apb_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = ADC_CLK_ENB,
	.recalc = &follow_parent,
};

/* ssp clock */
static struct clk ssp_clk = {
	.pclk = &apb_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = SSP_CLK_ENB,
	.recalc = &follow_parent,
};

/* gpio clock */
static struct clk gpio0_clk = {
	.pclk = &apb_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = GPIO0_CLK_ENB,
	.recalc = &follow_parent,
};

/* gpio clock */
static struct clk gpio1_clk = {
	.pclk = &apb_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = GPIO1_CLK_ENB,
	.recalc = &follow_parent,
};

/* i2s0 clock */
static struct clk i2s0_clk = {
	.pclk = &apb_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = I2S0_CLK_ENB,
	.recalc = &follow_parent,
};

/* i2s1 clock */
static struct clk i2s1_clk = {
	.pclk = &apb_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = I2S1_CLK_ENB,
	.recalc = &follow_parent,
};

/* keyboard clock */
static struct clk kbd_clk = {
	.pclk = &apb_clk,
	.en_reg = VA_PERIP2_CLK_ENB,
	.en_reg_bit = KBD_CLK_ENB,
	.recalc = &follow_parent,
};

/* RAS CLOCKS */

/* RAS fractional synthesizers masks */
static struct frac_synth_masks ras_synth_masks = {
	.div_factor_mask = FRAC_SYNT_DIV_FACTOR_MASK,
	.div_factor_shift = FRAC_SYNT_DIV_FACTOR_SHIFT,
};

static struct frac_synth_clk_config ras_synth0_config = {
	.synth_reg = VA_RAS_CLK_SYNT0,
	.masks = &ras_synth_masks,
};

static struct frac_synth_clk_config ras_synth1_config = {
	.synth_reg = VA_RAS_CLK_SYNT1,
	.masks = &ras_synth_masks,
};

static struct frac_synth_clk_config ras_synth2_config = {
	.synth_reg = VA_RAS_CLK_SYNT2,
	.masks = &ras_synth_masks,
};

static struct frac_synth_clk_config ras_synth3_config = {
	.synth_reg = VA_RAS_CLK_SYNT3,
	.masks = &ras_synth_masks,
};

/* RAS Fractional Synthesizer parents */
static struct pclk_info ras_synth0_1_pclk_info[] = {
	{
		.pclk = &vco1div4_clk,
		.pclk_val = RAS_SYNT0_1_VCO1_DIV4_VAL,
	}, {
		.pclk = &vco3div2_clk,
		.pclk_val = RAS_SYNT0_1_VCO3_DIV2_VAL,
	}, {
		.pclk = &pll3_clk,
		.pclk_val = RAS_SYNT0_1_PLL3_VAL,
	},
};

/* RAS Fractional Synthesizer-0 and 1 parent select structure */
static struct pclk_sel ras_synth0_1_pclk_sel = {
	.pclk_info = ras_synth0_1_pclk_info,
	.pclk_count = ARRAY_SIZE(ras_synth0_1_pclk_info),
	.pclk_sel_reg = VA_PLL_CFG,
	.pclk_sel_mask = RAS_SYNT0_1_CLK_MASK,
};

static struct pclk_info ras_synth2_3_pclk_info[] = {
	{
		.pclk = &vco1div4_clk,
		.pclk_val = RAS_SYNT2_3_VCO1_DIV4_VAL,
	}, {
		.pclk = &vco2div2_clk,
		.pclk_val = RAS_SYNT2_3_VCO2_DIV2_VAL,
	}, {
		.pclk = &pll2_clk,
		.pclk_val = RAS_SYNT2_3_PLL2_VAL,
	},
};

/* RAS Fractional Synthesizer-2 and 3 parent select structure */
static struct pclk_sel ras_synth2_3_pclk_sel = {
	.pclk_info = ras_synth2_3_pclk_info,
	.pclk_count = ARRAY_SIZE(ras_synth2_3_pclk_info),
	.pclk_sel_reg = VA_PLL_CFG,
	.pclk_sel_mask = RAS_SYNT2_3_CLK_MASK,
};

/* RAS rate configuration table, in ascending order of rates */
static struct frac_synth_rate_tbl ras_rtbl[] = {
	/* For vco1div4 = 250 MHz */
	{.div = 0x14000}, /* 25 MHz */
	{.div = 0x0A000}, /* 50 MHz */
	{.div = 0x05000}, /* 100 MHz */
	{.div = 0x02000}, /* 250 MHz */
};

/* RAS Fractional Synthesizer-0 Clock */
static struct clk ras_synth0_clk = {
	.en_reg = VA_RAS_CLK_ENB,
	.en_reg_bit = SYNT0_CLK_ENB,
	.pclk_sel = &ras_synth0_1_pclk_sel,
	.pclk_sel_shift = RAS_SYNT0_1_CLK_SHIFT,
	.calc_rate = &frac_synth_calc_rate,
	.recalc = &frac_synth_clk_recalc,
	.set_rate = &frac_synth_clk_set_rate,
	.rate_config = {ras_rtbl, ARRAY_SIZE(ras_rtbl), 1},
	.private_data = &ras_synth0_config,
};

/* RAS Fractional Synthesizer1 Clock */
static struct clk ras_synth1_clk = {
	.en_reg = VA_RAS_CLK_ENB,
	.en_reg_bit = SYNT1_CLK_ENB,
	.pclk_sel = &ras_synth0_1_pclk_sel,
	.pclk_sel_shift = RAS_SYNT0_1_CLK_SHIFT,
	.calc_rate = &frac_synth_calc_rate,
	.recalc = &frac_synth_clk_recalc,
	.set_rate = &frac_synth_clk_set_rate,
	.rate_config = {ras_rtbl, ARRAY_SIZE(ras_rtbl), 1},
	.private_data = &ras_synth1_config,
};

/* RAS Fractional Synthesizer2 Clock */
static struct clk ras_synth2_clk = {
	.en_reg = VA_RAS_CLK_ENB,
	.en_reg_bit = SYNT2_CLK_ENB,
	.pclk_sel = &ras_synth2_3_pclk_sel,
	.pclk_sel_shift = RAS_SYNT2_3_CLK_SHIFT,
	.calc_rate = &frac_synth_calc_rate,
	.recalc = &frac_synth_clk_recalc,
	.set_rate = &frac_synth_clk_set_rate,
	.rate_config = {ras_rtbl, ARRAY_SIZE(ras_rtbl), 1},
	.private_data = &ras_synth2_config,
};

/* RAS Fractional Synthesizer3 Clock */
static struct clk ras_synth3_clk = {
	.en_reg = VA_RAS_CLK_ENB,
	.en_reg_bit = SYNT3_CLK_ENB,
	.pclk_sel = &ras_synth2_3_pclk_sel,
	.pclk_sel_shift = RAS_SYNT2_3_CLK_SHIFT,
	.calc_rate = &frac_synth_calc_rate,
	.recalc = &frac_synth_clk_recalc,
	.set_rate = &frac_synth_clk_set_rate,
	.rate_config = {ras_rtbl, ARRAY_SIZE(ras_rtbl), 1},
	.private_data = &ras_synth3_config,
};

/* pll2 generated clock */
static struct clk ras_pll2_clk = {
	.pclk = &pll2_clk,
	.en_reg = VA_RAS_CLK_ENB,
	.en_reg_bit = PLL2_CLK_ENB,
	.recalc = &follow_parent,
};

/* pll3 generated clock */
static struct clk ras_pll3_clk = {
	.pclk = &pll3_clk,
	.en_reg = VA_RAS_CLK_ENB,
	.en_reg_bit = PLL3_CLK_ENB,
	.recalc = &follow_parent,
};

/* 125MHz clock generated on Tx pad */
static struct clk ras_tx125_clk = {
	.pclk = &gmii_125m_pad,
	.en_reg = VA_RAS_CLK_ENB,
	.en_reg_bit = C125M_PAD_CLK_ENB,
	.recalc = &follow_parent,
};

/* 30 MHz clock generated by USB PHy Pll */
static struct clk ras_30Mhz_clk = {
	.rate = 30000000,
	.en_reg = VA_RAS_CLK_ENB,
	.en_reg_bit = C30M_CLK_ENB,
};

/* 48 MHz clock generated by USB PHy Pll */
static struct clk ras_48Mhz_clk = {
	.pclk = &pll5_clk,
	.en_reg = VA_RAS_CLK_ENB,
	.en_reg_bit = C48M_CLK_ENB,
	.recalc = &follow_parent,
};

/* osc3 generated clock */
static struct clk ras_osc3_clk = {
	.pclk = &ahb_clk,
	.en_reg = VA_RAS_CLK_ENB,
	.en_reg_bit = OSC3_CLK_ENB,
	.recalc = &follow_parent,
};

/* osc2 generated clock */
static struct clk ras_osc2_clk = {
	.pclk = &osc2_32k_clk,
	.en_reg = VA_RAS_CLK_ENB,
	.en_reg_bit = OSC2_CLK_ENB,
	.recalc = &follow_parent,
};

/* osc1 generated clock */
static struct clk ras_osc1_clk = {
	.pclk = &osc1_24m_clk,
	.en_reg = VA_RAS_CLK_ENB,
	.en_reg_bit = OSC1_CLK_ENB,
	.recalc = &follow_parent,
};

/* apb generated clock */
static struct clk ras_pclk_clk = {
	.pclk = &apb_clk,
	.en_reg = VA_RAS_CLK_ENB,
	.en_reg_bit = PCLK_CLK_ENB,
	.recalc = &follow_parent,
};

/* ahb generated clock */
static struct clk ras_aclk_clk = {
	.pclk = &ahb_clk,
	.en_reg = VA_RAS_CLK_ENB,
	.en_reg_bit = ACLK_CLK_ENB,
	.recalc = &follow_parent,
};

/* External pad 50 MHz clock for phy operation */
static struct clk ras_tx50_clk = {
	.flags = ALWAYS_ENABLED,
	.rate = 50000000,
};

/* spear1300 machine specific clock structures */
#ifdef CONFIG_CPU_SPEAR1300

#endif

/* spear1310_reva machine specific clock structures */
#ifdef CONFIG_CPU_SPEAR1310_REVA
/* can0 clock */
static struct clk can0_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &apb_clk,
	.recalc = &follow_parent,
};

/* can1 clock */
static struct clk can1_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &apb_clk,
	.recalc = &follow_parent,
};

/* gmac clocks :RAS part*/
static struct clk gmac_ras1_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &ras_aclk_clk,
	.recalc = &follow_parent,
};

static struct clk gmac_ras2_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &ras_aclk_clk,
	.recalc = &follow_parent,
};

static struct clk gmac_ras3_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &ras_aclk_clk,
	.recalc = &follow_parent,
};

static struct clk gmac_ras4_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &ras_aclk_clk,
	.recalc = &follow_parent,
};

/* phy clock parent select */
static struct pclk_info rmii_phy_pclk_info[] = {
	{
		.pclk = &ras_tx50_clk,
		.pclk_val = SPEAR1310_REVA_RAS_TX50M_VAL,
	}, {
		.pclk = &ras_pll2_clk,
		.pclk_val = SPEAR1310_REVA_RAS_PLL2_VAL,
	}, {
		.pclk = &ras_synth0_clk,
		.pclk_val = SPEAR1310_REVA_RAS_SYNTH0_VAL,
	},
};

static struct pclk_info smii_rgmii_phy_pclk_info[] = {
	{
		.pclk = &ras_tx125_clk,
		.pclk_val = SPEAR1310_REVA_RAS_TX125M_PAD_VAL,
	}, {
		.pclk = &ras_pll2_clk,
		.pclk_val = SPEAR1310_REVA_RAS_PLL2_VAL,
	}, {
		.pclk = &ras_synth0_clk,
		.pclk_val = SPEAR1310_REVA_RAS_SYNTH0_VAL,
	},
};

/* RMII interface is driven by 50 MHz clock source */
static struct pclk_sel phy_rmii_pclk_sel = {
	.pclk_info = rmii_phy_pclk_info,
	.pclk_count = ARRAY_SIZE(rmii_phy_pclk_info),
	.pclk_sel_reg = IOMEM(IO_ADDRESS(SPEAR1310_REVA_RAS_CTRL_REG1)),
	.pclk_sel_mask = SPEAR1310_REVA_PHY_CLK_MASK,
};

/* SMII and RGMII are both driven by 125 MHz clock source */
static struct pclk_sel phy_smii_rgmii_pclk_sel = {
	.pclk_info = smii_rgmii_phy_pclk_info,
	.pclk_count = ARRAY_SIZE(smii_rgmii_phy_pclk_info),
	.pclk_sel_reg = IOMEM(IO_ADDRESS(SPEAR1310_REVA_RAS_CTRL_REG1)),
	.pclk_sel_mask = SPEAR1310_REVA_PHY_CLK_MASK,
};

/* Phy 1 Clock */
static struct clk gmac_phy1_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk_sel = &phy_smii_rgmii_pclk_sel,
	.pclk_sel_shift = SPEAR1310_REVA_SMII_RGMII_PHY_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* Phy 2 Clock */
static struct clk gmac_phy2_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk_sel = &phy_smii_rgmii_pclk_sel,
	.pclk_sel_shift = SPEAR1310_REVA_SMII_RGMII_PHY_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* Phy 3 Clock */
static struct clk gmac_phy3_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk_sel = &phy_rmii_pclk_sel,
	.pclk_sel_shift = SPEAR1310_REVA_RMII_PHY_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* Phy 4 Clock */
static struct clk gmac_phy4_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk_sel = &phy_smii_rgmii_pclk_sel,
	.pclk_sel_shift = SPEAR1310_REVA_SMII_RGMII_PHY_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* uart1 clock */
static struct clk uart1_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &ras_pclk_clk,
	.recalc = &follow_parent,
};

/* uart2 clock */
static struct clk uart2_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &ras_pclk_clk,
	.recalc = &follow_parent,
};

/* uart3 clock */
static struct clk uart3_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &ras_pclk_clk,
	.recalc = &follow_parent,
};

/* uart4 clock */
static struct clk uart4_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &ras_pclk_clk,
	.recalc = &follow_parent,
};

/* uart5 clock */
static struct clk uart5_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &ras_pclk_clk,
	.recalc = &follow_parent,
};

/* i2c1 clock */
static struct clk i2c1_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &ras_pclk_clk,
	.recalc = &follow_parent,
};

/* tdm_hdlc clock */
static struct pclk_info tdm_pclk_info[] = {
	{
		.pclk = &pll3_clk,
		.pclk_val = TDM_CLK_PLL3,
	}, {
		.pclk = &ras_synth1_clk,
		.pclk_val = TDM_CLK_RAS_SYNT1,
	},
};

static struct pclk_sel tdm_pclk_sel = {
	.pclk_info = tdm_pclk_info,
	.pclk_count = ARRAY_SIZE(tdm_pclk_info),
	.pclk_sel_reg =
		(unsigned int *)(IO_ADDRESS(SPEAR1310_REVA_RAS_CTRL_REG0)),
	.pclk_sel_mask = TDM_CLK_MASK,
};

static struct clk tdm_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk_sel = &tdm_pclk_sel,
	.pclk_sel_shift = TDM_CLK_SHIFT,
	.recalc = &follow_parent,
};

#endif

static struct clk dummy_apb_pclk;

/* array of all spear 13xx clock lookups */
static struct clk_lookup spear_clk_lookups[] = {
	{ .con_id = "apb_pclk",		.clk = &dummy_apb_pclk},
	/* root clks */
	{.con_id = "osc1_24m_clk",	.clk = &osc1_24m_clk},
	{.con_id = "osc2_32k_clk",	.clk = &osc2_32k_clk},
	{.con_id = "osc3_25m_clk",	.clk = &osc3_25m_clk},

	/* clock derived from 32 KHz osc clk */
	{.dev_id = "rtc-spear",		.clk = &rtc_clk},

	/* clock derived from 24/25 MHz osc1/osc3 clk */
	{.con_id = "vco1_clk",		.clk = &vco1_clk},
	{.con_id = "vco2_clk",		.clk = &vco2_clk},
	{.con_id = "vco3_clk",		.clk = &vco3_clk},
	{.con_id = "vco4_clk",		.clk = &vco4_clk},
	{.con_id = "pll5_clk",		.clk = &pll5_clk},
	{.con_id = "pll6_clk",		.clk = &pll6_clk},

	/* clock derived from vco1-5 clk */
	{.con_id = "pll1_clk",		.clk = &pll1_clk},
	{.con_id = "pll2_clk",		.clk = &pll2_clk},
	{.con_id = "pll3_clk",		.clk = &pll3_clk},
	{.con_id = "pll4_clk",		.clk = &pll4_clk},
	{.con_id = "vco1div2_clk",	.clk = &vco1div2_clk},
	{.con_id = "vco1div4_clk",	.clk = &vco1div4_clk},
	{.con_id = "vco2div2_clk",	.clk = &vco2div2_clk},
	{.con_id = "vco3div2_clk",	.clk = &vco3div2_clk},

	/* clock derived from pll1 clk */
	{.con_id = "ddr_clk",		.clk = &ddr_clk},
	{.con_id = "cpu_clk",		.clk = &cpu_clk},
	{.con_id = "ahb_clk",		.clk = &ahb_clk},
	{.con_id = "apb_clk",		.clk = &apb_clk},

	/* synthesizers/prescaled clocks */
	{.con_id = "c3_synth_clk",		.clk = &c3_synth_clk},
	{.con_id = "gmii_125m_pad_clk",		.clk = &gmii_125m_pad},
	{.con_id = "clcd_synth_clk",		.clk = &clcd_synth_clk},
	{.con_id = "uart_synth_clk",		.clk = &uart_synth_clk},
	{.con_id = "sdhci_synth_clk",		.clk = &sdhci_synth_clk},
	{.con_id = "cfxd_synth_clk",		.clk = &cfxd_synth_clk},
	{.con_id = "gmac_phy_input_clk",	.clk = &gmac_phy_input_clk},
	{.con_id = "gmac_phy_synth_clk",	.clk = &gmac_phy_synth_clk},
	{.con_id = "stmmacphy.0",		.clk = &gmac_phy0_clk},

	/* RAS clocks */
	{.con_id = "ras_synth0_clk",		.clk = &ras_synth0_clk},
	{.con_id = "ras_synth1_clk",		.clk = &ras_synth1_clk},
	{.con_id = "ras_synth2_clk",		.clk = &ras_synth2_clk},
	{.con_id = "ras_synth3_clk",		.clk = &ras_synth3_clk},
	{.con_id = "ras_pll3_clk",		.clk = &ras_pll3_clk},
	{.con_id = "ras_pll2_clk",		.clk = &ras_pll2_clk},
	{.con_id = "ras_tx125_clk",		.clk = &ras_tx125_clk},
	{.con_id = "ras_30Mhz_clk",		.clk = &ras_30Mhz_clk},
	{.con_id = "ras_48Mhz_clk",		.clk = &ras_48Mhz_clk},
	{.con_id = "ras_osc3_clk",		.clk = &ras_osc3_clk},
	{.con_id = "ras_osc2_clk",		.clk = &ras_osc2_clk},
	{.con_id = "ras_osc1_clk",		.clk = &ras_osc1_clk},
	{.con_id = "ras_pclk_clk",		.clk = &ras_pclk_clk},
	{.con_id = "ras_aclk_clk",		.clk = &ras_aclk_clk},
	{.con_id = "ras_tx50_clk",		.clk = &ras_tx50_clk},
	/* i2s refout and sclk clks */
	{.con_id = "i2s_src_pad_clk",		.clk = &i2s_src_pad_clk},
	{.con_id = "i2s_src_clk",		.clk = &i2s_src_clk},
	{.con_id = "i2s_prs1_clk",		.clk = &i2s_prs1_clk},
	{.con_id = "i2s_ref_clk",		.clk = &i2s_ref_clk},
	{.con_id = "i2s_ref_pad_clk",		.clk = &i2s_ref_pad_clk},
	{.con_id = "i2s_sclk_clk",		.clk = &i2s_sclk_clk},

	/* clocks having multiple parent source from above clocks */
	{.dev_id = "clcd_pixel_clk",	.clk = &clcd_pixel_clk},
	{.dev_id = "clcd-db9000",	.clk = &clcd_clk},
	{.dev_id = "gpt0",		.clk = &gpt0_clk},
	{.dev_id = "gpt1",		.clk = &gpt1_clk},
	{.dev_id = "gpt2",		.clk = &gpt2_clk},
	{.dev_id = "gpt3",		.clk = &gpt3_clk},
	{.dev_id = "uart",		.clk = &uart_clk},

	/* clock derived from ahb clk */
	{.dev_id = "smi",		.clk = &smi_clk},
	{.con_id = "usbh.0_clk",	.clk = &uhci0_clk},
	{.con_id = "usbh.1_clk",	.clk = &uhci1_clk},
	{.dev_id = "designware_udc",	.clk = &usbd_clk},
	{.dev_id = "i2c_designware.0",	.clk = &i2c_clk},
	{.con_id = "dmac_pclk",		.clk = &dma_pclk},
	{.dev_id = "dw_dmac.0",		.clk = &dma0_clk},
	{.dev_id = "dw_dmac.1",		.clk = &dma1_clk},
	{.dev_id = "jpeg-designware",	.clk = &jpeg_clk},
	{.dev_id = "stmmaceth.0",	.clk = &gmac0_clk},
	{.dev_id = "c3",		.clk = &c3_clk},
	{.dev_id = "dw_pcie.0",		.clk = &pcie0_clk},
	{.dev_id = "dw_pcie.1",		.clk = &pcie1_clk},
	{.dev_id = "dw_pcie.2",		.clk = &pcie2_clk},
	{.dev_id = "sdhci",		.clk = &sdhci_clk},
	{.con_id = "fsmc",		.clk = &fsmc_clk},
	{.dev_id = "fsmc-nand",		.clk = &fsmc_nand_clk},
	{.dev_id = "fsmc-nor",		.clk = &fsmc_nor_clk},
	{.dev_id = "sysram0",		.clk = &sysram0_clk},
	{.dev_id = "sysram1",		.clk = &sysram1_clk},
	{.dev_id = "arasan_cf",		.clk = &cfxd_clk},
	{.dev_id = "arasan_xd",		.clk = &cfxd_clk},

	/* clock derived from apb clk */
	{.dev_id = "adc",		.clk = &adc_clk},
	{.dev_id = "designware-i2s.0",	.clk = &i2s0_clk},
	{.dev_id = "designware-i2s.1",	.clk = &i2s1_clk},
	{.dev_id = "ssp-pl022",		.clk = &ssp_clk},
	{.dev_id = "gpio0",		.clk = &gpio0_clk},
	{.dev_id = "gpio1",		.clk = &gpio1_clk},
	{.dev_id = "keyboard",		.clk = &kbd_clk},
	{.dev_id = "cortexa9-wdt",	.clk = &wdt_clk},
	{.dev_id = "spear_thermal",	.clk = &thermal_clk},
};

/* array of all spear 1300 clock lookups */
static struct clk_lookup spear1300_clk_lookups[] = {
#ifdef CONFIG_CPU_SPEAR1300
#endif
};

/* array of all spear 1310 clock lookups */
static struct clk_lookup spear1310_reva_clk_lookups[] = {
#ifdef CONFIG_CPU_SPEAR1310_REVA
	{.dev_id = "c_can_platform.0",	.clk = &can0_clk},
	{.dev_id = "c_can_platform.1",	.clk = &can1_clk},
	{.dev_id = "stmmaceth.1",	.clk = &gmac_ras1_clk},
	{.dev_id = "stmmaceth.2",	.clk = &gmac_ras2_clk},
	{.dev_id = "stmmaceth.3",	.clk = &gmac_ras3_clk},
	{.dev_id = "stmmaceth.4",	.clk = &gmac_ras4_clk},
	{.con_id = "stmmacphy.1",	.clk = &gmac_phy1_clk},
	{.con_id = "stmmacphy.2",	.clk = &gmac_phy2_clk},
	{.con_id = "stmmacphy.3",	.clk = &gmac_phy3_clk},
	{.con_id = "stmmacphy.4",	.clk = &gmac_phy4_clk},
	{.dev_id = "uart1",		.clk = &uart1_clk},
	{.dev_id = "uart2",		.clk = &uart2_clk},
	{.dev_id = "uart3",		.clk = &uart3_clk},
	{.dev_id = "uart4",		.clk = &uart4_clk},
	{.dev_id = "uart5",		.clk = &uart5_clk},
	{.dev_id = "i2c_designware.1",	.clk = &i2c1_clk},
	{.con_id = "tdm_hdlc",		.clk = &tdm_clk},
#endif
};

/* array of all spear 900 clock lookups */
static struct clk_lookup spear900_clk_lookups[] = {
#ifdef CONFIG_CPU_SPEAR900
#endif
};

/* machine clk init */
void __init spear13xx_clk_init(void)
{
	int i, cnt;
	struct clk_lookup *lookups;

	if (cpu_is_spear1300()) {
		cnt = ARRAY_SIZE(spear1300_clk_lookups);
		lookups = spear1300_clk_lookups;
	} else if (cpu_is_spear1310_reva()) {
		cnt = ARRAY_SIZE(spear1310_reva_clk_lookups);
		lookups = spear1310_reva_clk_lookups;
	} else {
		cnt = ARRAY_SIZE(spear900_clk_lookups);
		lookups = spear900_clk_lookups;
	}

	for (i = 0; i < ARRAY_SIZE(spear_clk_lookups); i++)
		clk_register(&spear_clk_lookups[i]);

	for (i = 0; i < cnt; i++)
		clk_register(&lookups[i]);

	clk_init(&ddr_clk);
}
