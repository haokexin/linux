// SPDX-License-Identifier: GPL-2.0
/*
 * System Control and Management Interface (SCMI) Clock Protocol
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 * Copyright (C) 2018-2020 ARM Ltd.
 */
#include <linux/module.h>
#include <linux/sort.h>
#include "common.h"
#include "clock.h"

#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/pm_runtime.h>
#include <linux/sched.h>
#include <linux/debugfs.h>



struct C1200_clock_info g_clk_info[] = {
     [FIX_CLK_25M_OSC] = {
		.name = CLK_25M_OSC,
		.type = CLOCK_TREE_FIX,
		.fix = {
			.rate = 25000000,
		}
	},
	[FIX_CLK_24M_OSC] = {
		.name = CLK_24M_OSC,
		.type = CLOCK_TREE_FIX,
		.fix = {
			.rate = 24000000,
		}
	},
	[FIX_CLK_26M_OSC] = {
		.name = CLK_26M_OSC,
		.type = CLOCK_TREE_FIX,
		.fix = {
			.rate = 26000000,
		}
	},
	[FIX_LB_SOC_LSP_AUDIO0_WCLK] = {
		.name = LB_SOC_LSP_AUDIO0_WCLK,
		.type = CLOCK_TREE_FIX,
		.fix = {
			.rate = 884700000,
		}
	},
	[FIX_LB_SOC_LSP_AUDIO1_WCLK] = {
		.name = LB_SOC_LSP_AUDIO1_WCLK,
		.type = CLOCK_TREE_FIX,
		.fix = {
			.rate = 812800000,
		}
	},
	[FIX_LB_SOC_LSP_I2S_S0_SCK] = {
		.name = LB_SOC_LSP_I2S_S0_SCK,
		.type = CLOCK_TREE_FIX,
		.fix = {
			.rate = 36864000,
		}
	},
	[FIX_LB_SOC_LSP_I3C0_SCL] = {
		.name = LB_SOC_LSP_I3C0_SCL,
		.type = CLOCK_TREE_FIX,
		.fix = {
			.rate = 12500000,
		}
	},
	[FIX_LB_SOC_LSP_I3C1_SCL] = {
		.name = LB_SOC_LSP_I3C1_SCL,
		.type = CLOCK_TREE_FIX,
		.fix = {
			.rate = 12500000,
		}
	},
	[FIX_LB_SOC_LSP_FLEXRAY_HCLK] = {
		.name = LB_SOC_LSP_FLEXRAY_HCLK,
		.type = CLOCK_TREE_FIX,
		.fix = {
			.rate = 150000000,
		}
	},
	[FIX_LB_SOC_LSP_PCM_TDM0_SCLK_IN] = {
		.name = LB_SOC_LSP_PCM_TDM0_SCLK_IN,
		.type = CLOCK_TREE_FIX,
		.fix = {
			.rate = 6144000,
		}
	},
	[FIX_LB_SOC_LSP_PCM_TDM1_SCLK_IN] = {
		.name = LB_SOC_LSP_PCM_TDM1_SCLK_IN,
		.type = CLOCK_TREE_FIX,
		.fix = {
			.rate = 6144000,
		}
	},
	[FIX_LB_SW_PTP_SCLK] = {
		.name = LB_SW_XGMAC_PTP_CLK_NAME,
		.type = CLOCK_TREE_FIX,
		.fix = {
			.rate = 125000000,
		}
	},
	[FIX_LB_SW_WORK_SCLK] = {
		.name = LB_SW_XGMAC_WCLK_NAME,
		.type = CLOCK_TREE_FIX,
		.fix = {
			.rate = 312500000,
		}
	},
	[PLL_CLK_CPU] = {
		.name = PLL_CPU,
		.parent = CLK_25M_OSC,
		.type = CLOCK_TREE_PLL,
	},
	[PLL_CLK_GPU] = {
		.name = PLL_GPU,
		.parent = CLK_25M_OSC,
		.type = CLOCK_TREE_PLL
	},
	[PLL_CLK_CPU_DSU] = {
		.name = PLL_CPU_DSU,
		.parent = CLK_25M_OSC,
		.type = CLOCK_TREE_PLL,
	},
	[PLL_CLK_CMN] = {
		.name = PLL_CMN,
		.parent = CLK_25M_OSC,
		.type = CLOCK_TREE_PLL,
	},
	[PLL_CLK_SYSBUS0] = {
		.name = PLL_SYSBUS0,
		.parent = CLK_25M_OSC,
		.type = CLOCK_TREE_PLL,
	},
	[PLL_CLK_SYSBUS1] = {
		.name = PLL_SYSBUS1,
		.parent = CLK_25M_OSC,
		.type = CLOCK_TREE_PLL,
	},
	[PLL_CLK_DISPLAY0] = {
		.name = PLL_DISPLAY0,
		.parent = CLK_25M_OSC,
		.type = CLOCK_TREE_PLL,
	},
	[PLL_CLK_DISPLAY1] = {
		.name = PLL_DISPLAY1,
		.parent = CLK_25M_OSC,
		.type = CLOCK_TREE_PLL,
	},
	[PLL_CLK_DISPLAY2] = {
		.name = PLL_DISPLAY2,
		.parent = CLK_25M_OSC,
		.type = CLOCK_TREE_PLL,
	},
	[PLL_CLK_DISPLAY3] = {
		.name = PLL_DISPLAY3,
		.parent = CLK_25M_OSC,
		.type = CLOCK_TREE_PLL,
	},
	[PLL_CLK_NET] = {
		.name = PLL_NET,
		.parent = CLK_25M_OSC,
		.type = CLOCK_TREE_PLL,
	},
	[PLL_CLK_UFS] = {
		.name = PLL_UFS,
		.parent = CLK_25M_OSC,
		.type = CLOCK_TREE_PLL,
	},
	[FACTOR_FIX_CPU_DSU] = {
		.name = FACTOR_CPU_DSU_NAME,
		.parent = PLL_CPU_DSU,
		.type = CLOCK_TREE_FCT,
		.factor = {
			.mult = 1,
			.div = 2,
		}
	},
	[FACTOR_FIX_CMN_550] = {
		.name = FACTOR_CMN_550,
		.parent = MUX_CMN,
		.type = CLOCK_TREE_FCT,
		.factor = {
			.mult = 1,
			.div = 2,
		}
	},
	[FACTOR_FIX_CMN_275] = {
		.name = FACTOR_CMN_275,
		.parent = MUX_CMN,
		.type = CLOCK_TREE_FCT,
		.factor = {
			.mult = 1,
			.div = 4,
		}
	},
	[FACTOR_FIX_SYSBUS0_1000] = {
		.name = FACTOR_SYSBUS0_1000,
		.parent = MUX_SYSBUS0,
		.type = CLOCK_TREE_FCT,
		.factor = {
			.mult = 1,
			.div = 2,
		}
	},
	[FACTOR_FIX_SYSBUS0_666] = {
		.name = FACTOR_SYSBUS0_666,
		.parent = MUX_SYSBUS0,
		.type = CLOCK_TREE_FCT,
		.factor = {
			.mult = 1,
			.div = 3,
		}
	},
	[FACTOR_FIX_SYSBUS0_500] = {
		.name = FACTOR_SYSBUS0_500,
		.parent = MUX_SYSBUS0,
		.type = CLOCK_TREE_FCT,
		.factor = {
			.mult = 1,
			.div = 4,
		}
	},
	[FACTOR_FIX_SYSBUS0_250] = {
		.name = FACTOR_SYSBUS0_250,
		.parent = MUX_SYSBUS0,
		.type = CLOCK_TREE_FCT,
		.factor = {
			.mult = 1,
			.div = 8,
		}
	},
	[FACTOR_FIX_SYSBUS0_200] = {
		.name = FACTOR_SYSBUS0_200,
		.parent = MUX_SYSBUS0,
		.type = CLOCK_TREE_FCT,
		.factor = {
			.mult = 1,
			.div = 10,
		}
	},
	[FACTOR_FIX_SYSBUS0_125] = {
		.name = FACTOR_SYSBUS0_125,
		.parent = MUX_SYSBUS0,
		.type = CLOCK_TREE_FCT,
		.factor = {
			.mult = 1,
			.div = 16,
		}
	},
	[FACTOR_FIX_SYSBUS0_100] = {
		.name = FACTOR_SYSBUS0_100,
		.parent = MUX_SYSBUS0,
		.type = CLOCK_TREE_FCT,
		.factor = {
			.mult = 1,
			.div = 20,
		}
	},
	[FACTOR_FIX_SYSBUS0_50] = {
		.name = FACTOR_SYSBUS0_50,
		.parent = MUX_SYSBUS0,
		.type = CLOCK_TREE_FCT,
		.factor = {
			.mult = 1,
			.div = 40,
		}
	},
	[FACTOR_FIX_SYSBUS1_1200] = {
		.name = FACTOR_SYSBUS1_1200,
		.parent = MUX_SYSBUS1,
		.type = CLOCK_TREE_FCT,
		.factor = {
			.mult = 1,
			.div = 2,
		}
	},
	[FACTOR_FIX_SYSBUS1_800] = {
		.name = FACTOR_SYSBUS1_800,
		.parent = MUX_SYSBUS1,
		.type = CLOCK_TREE_FCT,
		.factor = {
			.mult = 1,
			.div = 3,
		}
	},
	[FACTOR_FIX_SYSBUS1_600] = {
		.name = FACTOR_SYSBUS1_600,
		.parent = MUX_SYSBUS1,
		.type = CLOCK_TREE_FCT,
		.factor = {
			.mult = 1,
			.div = 4,
		}
	},
	[FACTOR_FIX_SYSBUS1_400] = {
		.name = FACTOR_SYSBUS1_400,
		.parent = MUX_SYSBUS1,
		.type = CLOCK_TREE_FCT,
		.factor = {
			.mult = 1,
			.div = 6,
		}
	},
	[FACTOR_FIX_SYSBUS1_300] = {
		.name = FACTOR_SYSBUS1_300,
		.parent = MUX_SYSBUS1,
		.type = CLOCK_TREE_FCT,
		.factor = {
			.mult = 1,
			.div = 8,
		}
	},
	[FACTOR_FIX_SYSBUS1_150] = {
		.name = FACTOR_SYSBUS1_150,
		.parent = MUX_SYSBUS1,
		.type = CLOCK_TREE_FCT,
		.factor = {
			.mult = 1,
			.div = 16,
		}
	},
	[FACTOR_FIX_SYSBUS1_75] = {
		.name = FACTOR_SYSBUS1_75,
		.parent = MUX_SYSBUS1,
		.type = CLOCK_TREE_FCT,
		.factor = {
			.mult = 1,
			.div = 32,
		}
	},
	[FACTOR_FIX_SYSBUS1_25] = {
		.name = FACTOR_SYSBUS1_25,
		.parent = MUX_SYSBUS1,
		.type = CLOCK_TREE_FCT,
		.factor = {
			.mult = 1,
			.div = 96,
		}
	},
	[FACTOR_LSP0_CLK_DIV_2] = {
		.name = LSP0_WCLK_CLK_DIV_2,
		.parent = LB_SOC_LSP0_WCLK_EN,
		.type = CLOCK_TREE_FCT,
		.factor = {
			.mult = 1,
			.div = 2,
		}
	},
	[FACTOR_LSP0_CLK_DIV_4] = {
		.name = LSP0_WCLK_CLK_DIV_4,
		.parent = LB_SOC_LSP0_WCLK_EN,
		.type = CLOCK_TREE_FCT,
		.factor = {
			.mult = 1,
			.div = 4,
		}
	},
	[FACTOR_LSP1_CLK_DIV_2] = {
		.name = LSP1_WCLK_CLK_DIV_2,
		.parent = LB_SOC_LSP1_WCLK_EN,
		.type = CLOCK_TREE_FCT,
		.factor = {
			.mult = 1,
			.div = 2,
		}
	},
	[FACTOR_LSP1_CLK_DIV_4] = {
		.name = LSP1_WCLK_CLK_DIV_4,
		.parent = LB_SOC_LSP1_WCLK_EN,
		.type = CLOCK_TREE_FCT,
		.factor = {
			.mult = 1,
			.div = 4,
		}
	},
	[MUX_CPU_SEL] = {
		.name = MUX_CPU,
		.mux_parent = cpu_sel_parents,
		.mux_parent_count = ARRAY_SIZE(cpu_sel_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_GPU_SEL] = {
		.name = MUX_GPU,
		.mux_parent = gpu_sel_parents,
		.mux_parent_count = ARRAY_SIZE(gpu_sel_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_CPU_DSU_SEL] = {
		.name = MUX_CPU_DSU,
		.mux_parent = cpu_dsu_sel_parents,
		.mux_parent_count = ARRAY_SIZE(cpu_dsu_sel_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_CMN_SEL] = {
		.name = MUX_CMN,
		.mux_parent = cmn_sel_parents,
		.mux_parent_count = ARRAY_SIZE(cmn_sel_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SYSBUS0_SEL] = {
		.name = MUX_SYSBUS0,
		.mux_parent = sysbus0_sel_parents,
		.mux_parent_count = ARRAY_SIZE(sysbus0_sel_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SYSBUS1_SEL] = {
		.name = MUX_SYSBUS1,
		.mux_parent = sysbus1_sel_parents,
		.mux_parent_count = ARRAY_SIZE(sysbus1_sel_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_DISPLAY0_SEL] = {
		.name = MUX_DISPLAY0,
		.mux_parent = display0_sel_parents,
		.mux_parent_count = ARRAY_SIZE(display0_sel_parents),
		.type = CLOCK_TREE_MUX,
    },
	[MUX_DISPLAY1_SEL] = {
		.name = MUX_DISPLAY1,
		.mux_parent = display1_sel_parents,
		.mux_parent_count = ARRAY_SIZE(display1_sel_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_DISPLAY2_SEL] = {
		.name = MUX_DISPLAY2,
		.mux_parent = display2_sel_parents,
		.mux_parent_count = ARRAY_SIZE(display2_sel_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_DISPLAY3_SEL] = {
		.name = MUX_DISPLAY3,
		.mux_parent = display3_sel_parents,
		.mux_parent_count = ARRAY_SIZE(display3_sel_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_NET_SEL] = {
		.name = MUX_NET,
		.mux_parent = net_sel_parents,
		.mux_parent_count = ARRAY_SIZE(net_sel_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_UFS_SEL] = {
		.name = MUX_UFS,
		.mux_parent = ufs_sel_parents,
		.mux_parent_count = ARRAY_SIZE(ufs_sel_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_DISPLAY0_SEL0_SEL] = {
		.name = MUX_DISPLAY0_SEL0,
		.mux_parent = display0_mux_parents,
		.mux_parent_count = ARRAY_SIZE(display0_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_DISPLAY1_SEL0_SEL] = {
		.name = MUX_DISPLAY1_SEL0,
		.mux_parent = display1_mux_parents,
		.mux_parent_count = ARRAY_SIZE(display1_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_DISPLAY2_SEL0_SEL] = {
		.name = MUX_DISPLAY2_SEL0,
		.mux_parent = display2_mux_parents,
		.mux_parent_count = ARRAY_SIZE(display2_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_DISPLAY3_SEL0_SEL] = {
		.name = MUX_DISPLAY3_SEL0,
		.mux_parent = display3_mux_parents,
		.mux_parent_count = ARRAY_SIZE(display3_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_DISPLAY4_SEL0_SEL] = {
		.name = MUX_DISPLAY4_SEL0,
		.mux_parent = display4_mux_parents,
		.mux_parent_count = ARRAY_SIZE(display4_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_HIFI_DSP_ACLK_SEL] = {
		.name = MUX_HIFI_DSP_ACLK,
		.mux_parent = hifi_dsp_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(hifi_dsp_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_CV_CORE_SEL] = {
		.name = MUX_CV_CORE_CLK,
		.mux_parent = cv_core_clk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(cv_core_clk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SAFETYNOC_TO_SOCNOC_M_ACLK_SEL] = {
		.name = MUX_SAFETYNOC_TO_SOCNIC_M_ACLK,
		.mux_parent = safetynoc_to_socnic_m_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(safetynoc_to_socnic_m_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_XGAMC_AXI_ACLK_CLK_SEL] = {
		.name = MUX_XGMAC_AXI_ACLK,
		.mux_parent = xgmac_axi_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(xgmac_axi_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SYSNOC_TO_SOCNIC_AHB_SEL] = {
		.name = MUX_SYSNOC_TO_SOCNIC_AHB_CFG_HCLK,
		.mux_parent = sysnoc_to_socnic_ahb_cfg_hclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(sysnoc_to_socnic_ahb_cfg_hclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	}, 
	[MUX_G78AE_SP_WCLK_SEL] = {
		.name = MUX_G78AE_SP_WCLK,
		.mux_parent = g78ae_sp_wclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(g78ae_sp_wclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_HIFI_DSP_XNNE_WCLK_SEL] = {
		.name = MUX_HIFI_DSP_XNNE_WCLK,
		.mux_parent = hifi_dsp_xnne_wclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(hifi_dsp_xnne_wclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_CV_AXIM1_SEL] = {
		.name = MUX_CV_AXIM1_CLK,
		.mux_parent = cv_axim1_clk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(cv_axim1_clk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_CORENOC_TO_SOCNIC_SEL] = {
		.name = MUX_CORENOC_TO_SOCNIC_M_ACLK,
		.mux_parent = corenoc_to_socnic_m_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(corenoc_to_socnic_m_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SGMAC_APB_PCLK_SEL] = {
		.name = MUX_XGMAC_APB_PCLK,
		.mux_parent = xgmac_apb_pclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(xgmac_apb_pclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SOC_SEC_HCLK_SEL] = {
		.name = MUX_SOC_SEC_HCLK,
		.mux_parent = soc_sec_hclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(soc_sec_hclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_G78AE_WCLK_SEL] = {
		.name = MUX_G78AE_WCLK_CLK,
		.mux_parent = g78ae_wclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(g78ae_wclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_HIFI_DSP_WCLK_CLK_SEL] = {
		.name = MUX_HIFI_DSP_WCLK,
		.mux_parent = hifi_dsp_wclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(hifi_dsp_wclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_CV_AXIM0_SEL] = {
		.name = MUX_CV_AXIM0_CLK,
		.mux_parent = cv_axim0_clk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(cv_axim0_clk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SECNOC_TO_SOCNIC_SEL] = {
		.name = MUX_SECNOC_TO_SOCNIC_M_ACLK,
		.mux_parent = secnoc_to_socnic_m_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(secnoc_to_socnic_m_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_PROBE_AXI_ACLK_SEL] = {
		.name = MUX_PROBE_AXI_ACLK,
		.mux_parent = probe_axi_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(probe_axi_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SOC_SEC_ACLK_SEL] = {
		.name = MUX_SOC_SEC_ACLK,
		.mux_parent = soc_sec_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(soc_sec_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_G78AE_ACE_ACLK_SEL] = {
		.name = MUX_G78AE_ACE_ACLK,
		.mux_parent = g78ae_ace_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(g78ae_ace_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_LVDS1_HSPEED_SEL] = {
		.name = MUX_CLK_LVDS1_HSPEED,
		.mux_parent = clk_lvds1_hspeed_mux_parents,
		.mux_parent_count = ARRAY_SIZE(clk_lvds1_hspeed_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_CV_AXIS_SEL] = {
		.name = MUX_CV_AXIS_CLK,
		.mux_parent = cv_axis_clk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(cv_axis_clk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SECNOC_TO_RTNOC_SEL] = {
		.name = MUX_SECNOC_TO_RTNOC_ACLK,
		.mux_parent = secnoc_to_rtnoc_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(secnoc_to_rtnoc_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_ATB_TPIU_TRACE_SEL] = {
		.name = MUX_ATB_TPIU_TRACE_CLK,
		.mux_parent = atb_tpiu_trace_clk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(atb_tpiu_trace_clk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_EDP_ACLK_SEL] = {
		.name = MUX_EDP_ACLK,
		.mux_parent = edp_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(edp_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_G78AE_AXIS_ACLK_SEL] = {
		.name = MUX_G78AE_AXIS_ACLK,
		.mux_parent = g78ae_axis_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(g78ae_axis_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_LVDS0_HSPEED_SEL] = {
		.name = MUX_CLK_LVDS0_HSPEED,
		.mux_parent = clk_lvds0_hspeed_mux_parents,
		.mux_parent_count = ARRAY_SIZE(clk_lvds0_hspeed_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_ISP_SCLK_SEL] = {
		.name = MUX_ISP_SCLK,
		.mux_parent = isp_sclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(isp_sclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_RTNOC_TO_SOCNIC_SEL] = {
		.name = MUX_RTNOC_TO_SOCNIC_M_ACLK,
		.mux_parent = rtnoc_to_socnic_m_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(rtnoc_to_socnic_m_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_ATB_AXI_ACLK_SEL] = {
		.name = MUX_ATB_AXI_ACLK,
		.mux_parent = atb_axi_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(atb_axi_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_DBSOCNOC_TO_CMN_SEL] = {
		.name = MUX_DBSOCNOC_TO_CMN_ACLK,
		.mux_parent = dbsocnoc_to_cmn_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(dbsocnoc_to_cmn_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_G78AE_TCU_WCLK_SEL] = {
		.name = MUX_G78AE_TCU_WCLK,
		.mux_parent = g78ae_tcu_wclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(g78ae_tcu_wclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_CPU_1188_SEL] = {
		.name = MUX_CLK_CPU_1188,
		.mux_parent = clk_cpu_1188_mux_parents,
		.mux_parent_count = ARRAY_SIZE(clk_cpu_1188_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SWNOC_TO_SYSNOC_ACLK_SEL] = {
		.name = MUX_SWNOC_TO_SYSNOC_ACLK,
		.mux_parent = swnoc_to_sysnoc_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(swnoc_to_sysnoc_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_DBNOC_TO_DBSOCNOC_SEL] = {
		.name = MUX_DBNOC_TO_DBSOCNOC_M_ACLK,
		.mux_parent = dbnoc_to_dbsocnoc_m_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(dbnoc_to_dbsocnoc_m_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SMMU_TCU_CODEC_SEL] = {
		.name = MUX_SMMU_TCU_CODEC_ACLK,
		.mux_parent = smmu_tcu_codec_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(smmu_tcu_codec_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_DBSOCNOC_TO_SYSNOC_SEL] = {
		.name = MUX_DBSOCNOC_TO_SYSNOC_ACLK,
		.mux_parent = dbsocnoc_to_sysnoc_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(dbsocnoc_to_sysnoc_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SYS_NOC_800M_WCLK_SEL] = {
		.name = MUX_SYS_NOC_800M_WCLK,
		.mux_parent = sys_noc_800m_wclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(sys_noc_800m_wclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SAFENOC_TO_SWNOC_SEL] = {
		.name = MUX_SAFENOC_TO_SWNOC_ACLK,
		.mux_parent = safenoc_to_swnoc_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(safenoc_to_swnoc_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SOCNIC_S_TO_DBNOC_SEL] = {
		.name = MUX_SOCNIC_S_TO_DBNOC_ACLK,
		.mux_parent = socnic_s_to_dbnoc_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(socnic_s_to_dbnoc_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SMMU_TCU_COREIP_ACLK_SEL] = {
		.name = MUX_SMMU_TCU_COREIP_ACLK,
		.mux_parent = smmu_tcu_coreip_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(smmu_tcu_coreip_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SYSNOC_TO_DBSOCNOC_SEL] = {
		.name = MUX_SYSNOC_TO_DBSOCNOC_ACLK,
		.mux_parent = sysnoc_to_dbsocnoc_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(sysnoc_to_dbsocnoc_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},

	[MUX_SYS_NOC_400M_WCLK_SEL] = {
		.name = MUX_SYS_NOC_400M_WCLK,
		.mux_parent = sys_noc_400m_wclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(sys_noc_400m_wclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SOC_LSP1_WCLK_SEL] = {
		.name = MUX_SOC_LSP1_WCLK,
		.mux_parent = soc_lsp1_wclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(soc_lsp1_wclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SDNIC_TO_SYSNOC_ACLK_SEL] = {
		.name = MUX_SDNIC_TO_SYSNOC_ACLK,
		.mux_parent = sdnic_to_sysnoc_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(sdnic_to_sysnoc_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_CPU_PERIP_CLK_SEL] = {
		.name = MUX_CPU_PERIP_CLK,
		.mux_parent = cpu_perip_clk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(cpu_perip_clk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_DBSOCNOC_WCLK_SEL] = {
		.name = MUX_DBSOCNOC_WCLK,
		.mux_parent = dbsocnoc_wclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(dbsocnoc_wclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_MEDIA_NOC_WCLK_SEL] = {
		.name = MUX_MEDIA_NOC_WCLK,
		.mux_parent = media_noc_wclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(media_noc_wclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SOC_LSP0_WCLK_SEL] = {
		.name = MUX_SOC_LSP0_WCLK,
		.mux_parent = soc_lsp0_wclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(soc_lsp0_wclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_USBNIC_TO_SYSNOC_ACLK_SEL] = {
		.name = MUX_USBNIC_TO_SYSNOC_ACLK,
		.mux_parent = usbnic_to_sysnoc_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(usbnic_to_sysnoc_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_RTNOC_SWNOC_ACLK_SEL] = {
		.name = MUX_RTNOC_SWNOC_ACLK,
		.mux_parent = rtnoc_swnoc_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(rtnoc_swnoc_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_MSGBX_SWITCH1_TO_SAFE_SEL] = {
		.name = MUX_MSGBX_SWITCH1_TO_SAFE_HCLK,
		.mux_parent = msgbx_switch1_to_safe_hclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(msgbx_switch1_to_safe_hclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_CORE_NOC_WCLK_SEL] = {
		.name = MUX_CORE_NOC_WCLK,
		.mux_parent = core_noc_wclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(core_noc_wclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_UFS_ACLK_CLK_SEL] = {
		.name = MUX_UFS_ACLK,
		.mux_parent = ufs_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(ufs_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_LPDDR5_1_S4_ACLK_SEL] = {
		.name = MUX_LPDDR5_1_S4_ACLK,
		.mux_parent = lpddr5_1_s4_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(lpddr5_1_s4_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
    },
	[MUX_RTNOC_SAFENOC_ACLK_SEL] = {
		.name = MUX_RTNOC_SAFENOC_ACLK,
		.mux_parent = rtnoc_safenoc_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(rtnoc_safenoc_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	
	},
	[MUX_MSGBOX_SWITCH1_TO_SECURE_HCLK_SEL] = {
		.name = MUX_MSGBX_SWITCH1_TO_SECURE_HCLK,
		.mux_parent = msgbx_switch1_to_secure_hclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(msgbx_switch1_to_secure_hclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_DB_NOC_WCLK_CLK_SEL] = {
		.name = MUX_DB_NOC_WCLK,
		.mux_parent = db_noc_wclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(db_noc_wclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SDEMMC1_HCLK_SEL] = {
		.name = MUX_SDEMMC1_HCLK,
		.mux_parent = sdemmc1_hclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(sdemmc1_hclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_LDDR5_1_S3_ACLK_SEL] = {
		.name = MUX_LPDDR5_1_S3_ACLK,
		.mux_parent = lpddr5_1_s3_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(lpddr5_1_s3_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SYSNOC_TO_DBNOC_ACLK_SEL] = {
		.name = MUX_SYSNOC_TO_DBNOC_ACLK,
		.mux_parent = sysnoc_to_dbnoc_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(sysnoc_to_dbnoc_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_MSGBX_SWITCH1_TO_REALTIME_HCLK_SEL] = {
		.name = MUX_MSGBX_SWITCH1_TO_REALTIME_HCLK,
		.mux_parent = msgbx_switch1_to_realtime_hclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(msgbx_switch1_to_realtime_hclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_CMN_WCLK_SEL] = {
		.name = MUX_CMN_WCLK_MUX,
		.mux_parent = cmn_wclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(cmn_wclk_mux_parents),
		.type = CLOCK_TREE_MUX,
    },
	[MUX_CS_DMA_HCLK_SEL] = {
		.name = MUX_CS_DMA_HCLK,
		.mux_parent = cs_dma_hclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(cs_dma_hclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SDEMMC1_W_BCLK_SEL] = {
		.name = MUX_SDEMMC1_W_BCLK,
		.mux_parent = sdemmc1_w_bclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(sdemmc1_w_bclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_LPPDR5_1_S2_ACLK_SEL] = {
		.name = MUX_LPDDR5_1_S2_ACLK,
		.mux_parent = lpddr5_1_s2_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(lpddr5_1_s2_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SOC_SRAM_ACLK_CLK_SEL] = {
		.name = MUX_SOC_SRAM_ACLK,
		.mux_parent = soc_sram_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(soc_sram_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_MSGBX_SWITCH0_TO_SWITCH1_WCLK_SEL] = {
		.name = MUX_MSGBX_SWITCH0_TO_SWITCH1_WCLK,
		.mux_parent = msgbx_switch0_to_switch1_wclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(msgbx_switch0_to_switch1_wclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_CMN_TO_SYSNOC_ACLK_SEL] = {
		.name = MUX_CMN_TO_SYSNOC_ACLK,
		.mux_parent = cmn_to_sysnoc_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(cmn_to_sysnoc_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_200_SYSBUS_APB_SEL] = {
		.name = MUX_CLK_200_SYSBUS_APB,
		.mux_parent = clk_200_sysbus_apb_mux_parents,
		.mux_parent_count = ARRAY_SIZE(clk_200_sysbus_apb_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_MEDIA_DMA_HCLK_SEL] = {
		.name = MUX_MEDIA_DMA_HCLK,
		.mux_parent = media_dma_hclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(media_dma_hclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SDEMMC0_HCLK_SEL] = {
		.name = MUX_SDEMMC0_HCLK,
		.mux_parent = sdemmc0_hclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(sdemmc0_hclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_LPDDR5_1_S1_ACLK_SEL] = {
		.name = MUX_LPDDR5_1_S1_ACLK,
		.mux_parent = lpddr5_1_s1_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(lpddr5_1_s1_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SYSNOC_TO_CMN_ACLK_SEL] = {
		.name = MUX_SYSNOC_TO_CMN_ACLK,
		.mux_parent = sysnoc_to_cmn_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(sysnoc_to_cmn_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_MSGBX_SWITCH1_TO_MEDIA_SEL] = {
		.name = MUX_MSGBX_SWITCH1_TO_MEDIA_HCLK,
		.mux_parent = msgbx_switch1_to_media_hclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(msgbx_switch1_to_media_hclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_100_SYSBUS_APB_SEL] = {
		.name = MUX_CLK_100_SYSBUS_APB,
		.mux_parent = clk_100_sysbus_apb_mux_parents,
		.mux_parent_count = ARRAY_SIZE(clk_100_sysbus_apb_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SOC_DMA_HCLK_SEL] = {
		.name = MUX_SOC_DMA_HCLK,
		.mux_parent = soc_dma_hclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(soc_dma_hclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SDEMMC0_W_BCLK_SEL] = {
		.name = MUX_SDEMMC0_W_BCLK,
		.mux_parent = sdemmc0_w_bclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(sdemmc0_w_bclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_LPDDR5_1_S0_ACLK_SEL] = {
		.name = MUX_LPDDR5_1_S0_ACLK,
		.mux_parent = lpddr5_1_s0_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(lpddr5_1_s0_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_CORENOC_TO_CMN_ACLK_SEL] = {
		.name = MUX_CORENOC_TO_CMN_ACLK,
		.mux_parent = corenoc_to_cmn_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(corenoc_to_cmn_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_MSGBX_SWITCH1_TO_SW_HCLK_SEL] = {
		.name = MUX_MSGBX_SWITCH1_TO_SW_HCLK,
		.mux_parent = msgbx_switch1_to_sw_hclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(msgbx_switch1_to_sw_hclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_NOC_ATB_CLK_SEL] = {
		.name = MUX_NOC_ATB_CLK,
		.mux_parent = noc_atb_clk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(noc_atb_clk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_CPU0_MP4_DSU_CHI_SEL] = {
		.name = MUX_CPU0_MP4_DSU_CHI_CLK,
		.mux_parent = cpu0_mp4_dsu_chi_clk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(cpu0_mp4_dsu_chi_clk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_DB_DMA_HCLK_SEL] = {
		.name = MUX_DB_DMA_HCLK,
		.mux_parent = db_dma_hclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(db_dma_hclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_USB_1_ACLK_SEL] = {
		.name = MUX_USB_1_ACLK,
		.mux_parent = usb_1_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(usb_1_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_LPDDR5_0_S4_ACLK_SEL] = {
		.name = MUX_LPDDR5_0_S4_ACLKK,
		.mux_parent = lpddr5_0_s4_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(lpddr5_0_s4_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_MEDIANOC_TO_CMN_ACLK_SEL] = {
		.name = MUX_MEDIANOC_TO_CMN_ACLK,
		.mux_parent = medianoc_to_cmn_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(medianoc_to_cmn_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_MSGBX_SWITCH1_WCLK_SEL] = {
		.name = MUX_MSGBX_SWITCH1_WCLK,
		.mux_parent = msgbx_switch1_wclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(msgbx_switch1_wclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SOC_DMA_CORE_SEL] = {
		.name = MUX_SOC_DMA_CORE,
		.mux_parent = soc_dma_core_clk_parents,
		.mux_parent_count = ARRAY_SIZE(soc_dma_core_clk_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_CPU0_MP4_CORE_SEL] = {
		.name = MUX_CPU0_MP4_CORE_CLK,
		.mux_parent = cpu0_mp4_core_clk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(cpu0_mp4_core_clk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SOC_DMA_ACLK_CLK_SEL] = {
		.name = MUX_CS_DMA_ACLK,
		.mux_parent = cs_dma_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(cs_dma_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_USB_0_ACLK_CLK_SEL] = {
		.name = MUX_USB_0_ACLK,
		.mux_parent = usb_0_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(usb_0_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_LPDDR5_0_S3_ACLK_CLK_SEL] = {
		.name = MUX_LPDDR5_0_S3_ACLK,
		.mux_parent = lpddr5_0_s3_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(lpddr5_0_s3_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SOCNIC_S_TO_MEDIANOC_ACLK_CLK_SEL] = {
		.name = MUX_SOCNIC_S_TO_MEDIANOC_ACLK,
		.mux_parent = socnic_s_to_medianoc_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(socnic_s_to_medianoc_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_MSGBX_SWITCH0_TO_SOCDMA_HCLK_CLK_SEL] = {
		.name = MUX_MSGBX_SWITCH0_TO_SOCDMA_HCLK,
		.mux_parent = msgbx_switch0_to_socdma_hclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(msgbx_switch0_to_socdma_hclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_FLEXRAYNIC_TO_SYSNOC_HCLK_CLK_SEL] = {
		.name = MUX_FLEXRAYNIC_TO_SYSNOC_HCLK,
		.mux_parent = flexraynic_to_sysnoc_hclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(flexraynic_to_sysnoc_hclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_CPU1_MP4_DSU_CHI_CLK_SEL] = {
		.name = MUX_CPU1_MP4_DSU_CHI_CLK,
		.mux_parent = cpu1_mp4_dsu_chi_clk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(cpu1_mp4_dsu_chi_clk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_MEDIA_DMA_ACLK_SEL] = {
		.name = MUX_MEDIA_DMA_ACLK,
		.mux_parent =media_dma_aclk_mux_parents ,
		.mux_parent_count = ARRAY_SIZE(media_dma_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_PCIE_ACLK_SEL] = {
		.name = MUX_PCIE_ACLK,
		.mux_parent = pcie_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(pcie_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_LPDDR5_0_S2_ACLK_SEL] = {
		.name = MUX_LPDDR5_0_S2_ACLK,
		.mux_parent = lpddr5_0_s2_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(lpddr5_0_s2_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SOCNIC_S_TO_CORENOC_ACLK_SEL] = {
		.name = MUX_SOCNIC_S_TO_CORENOC_ACLK,
		.mux_parent = socnic_s_to_corenoc_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(socnic_s_to_corenoc_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_MSGBX_SWITCH0_TO_NET_HCLK_SEL] = {
		.name = MUX_MSGBX_SWITCH0_TO_NET_HCLK,
		.mux_parent = msgbx_switch0_to_net_hclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(msgbx_switch0_to_net_hclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_DSI_CSITX_IPI_SEL] = {
		.name = MUX_DSI_CSITX_IPI,
		.mux_parent = dsi_csitx_ipi_clk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(dsi_csitx_ipi_clk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_CPU1_MP4_CORE_CLK_SEL] = {
		.name = MUX_CPU1_MP4_CORE_CLK,
		.mux_parent = cpu1_mp4_core_clk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(cpu1_mp4_core_clk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SOC_DMA_ACLK_SEL] = {
		.name = MUX_SOC_DMA_ACLK,
		.mux_parent = soc_dma_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(soc_dma_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_PCIE_DBI_ACLK_SEL] = {
		.name = MUX_PCIE_DBI_ACLK,
		.mux_parent = pcie_dbi_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(pcie_dbi_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_LPDDR5_0_S1_ACLK_SEL] = {
		.name = MUX_LPDDR5_0_S1_ACLK,
		.mux_parent = lpddr5_0_s1_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(lpddr5_0_s1_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_SOCNIC_S_TO_SAFENOC_ACLK_SEL] = {
		.name = MUX_SOCNIC_S_TO_SAFENOC_ACLK,
		.mux_parent = socnic_s_to_safenoc_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(socnic_s_to_safenoc_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_MSGBX_SWITCH0_TO_CPU_HCLK_SEL] = {
		.name = MUX_MSGBX_SWITCH0_TO_CPU_HCLK,
		.mux_parent = msgbx_switch0_to_cpu_hclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(msgbx_switch0_to_cpu_hclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	
	[MUX_DSI1_CLKEXT_SEL] = {
		.name = MUX_DSI1_CLKEXT,
		.mux_parent = dsi1_clkext_mux_parents,
		.mux_parent_count = ARRAY_SIZE(dsi1_clkext_mux_parents),
		.type = CLOCK_TREE_MUX,
	},

	[MUX_CPU2_MP_ACLK_SEL] = {
		.name =MUX_CPU_MP2_MP_ACLK ,
		.mux_parent =cpu_mp2_mp_aclk_mux_parents ,
		.mux_parent_count = ARRAY_SIZE(cpu_mp2_mp_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
		[MUX_DB_DMA_ACLK_SEL] = {
		.name =MUX_DB_DMA_ACLK ,
		.mux_parent = db_dma_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(db_dma_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
		[MUX_DISPLAY2_ACLK_SEL] = {
		.name =MUX_DISPLAY2_ACLK ,
		.mux_parent = display2_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(display2_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
		[MUX_LPDDR5_0_S0_ACLK_SEL] = {
		.name = MUX_LPDDR5_0_S0_ACLK,
		.mux_parent =lpddr5_0_s0_aclk_mux_parents ,
		.mux_parent_count = ARRAY_SIZE(lpddr5_0_s0_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
		[MUX_SOCNIC_S_TO_SWNOC_ACLK_SEL] = {
		.name =MUX_SOCNIC_S_TO_SWNOC_ACLK ,
		.mux_parent = socnic_s_to_swnoc_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(socnic_s_to_swnoc_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
		[MUX_MSGBX_SWITCH0_TO_DB_HCLK_SEL] = {
		.name = MUX_MSGBX_SWITCH0_TO_DB_HCLK,
		.mux_parent = msgbx_switch0_to_db_hclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(msgbx_switch0_to_db_hclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
		[MUX_DSI0_CLKEXT_SEL] = {
		.name =MUX_DSI0_CLKEXT ,
		.mux_parent = dsi0_clkext_mux_parents,
		.mux_parent_count = ARRAY_SIZE(dsi0_clkext_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
		[MUX_MP2_MASTER_ACLK_SEL] = {
		.name = MUX_CPU_MP2_MASTER_ACLK,
		.mux_parent = cpu_mp2_master_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(cpu_mp2_master_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
		[MUX_CODEC1_WCLK_SEL] = {
		.name = MUX_CODEC1_WCLK,
		.mux_parent = codec1_wclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(codec1_wclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
		[MUX_DISPLAY1_ACLK_SEL] = {
		.name = MUX_DISPLAY1_ACLK,
		.mux_parent = display1_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(display1_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
		[MUX_DSP_CLK_SEL] = {
		.name =MUX_DSP_CLK ,
		.mux_parent = dsp_clk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(dsp_clk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
		[MUX_SYSNOC_TO_SOCNIC_S_ACLK_SEL] = {
		.name = MUX_SYSNOC_TO_SOCNIC_S_ACLK,
		.mux_parent = sysnoc_to_socnic_s_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(sysnoc_to_socnic_s_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
		[MUX_MSGBX_SWITCH0_TO_ISPCV_HCLK_SEL] = {
		.name =MUX_MSGBX_SWITCH0_TO_ISPCV_HCLK ,
		.mux_parent = msgbx_switch0_to_ispcv_hclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(msgbx_switch0_to_ispcv_hclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_MEDIANOC_TO_SYSNOC_ACLK_SEL] = {
		.name = MUX_MEDIANOC_TO_SYSNOC_ACLK,
		.mux_parent = medianoc_to_sysnoc_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(medianoc_to_sysnoc_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
		[MUX_CPU_MP2_DSU_SEL] = {
		.name = MUX_CPU_MP2_DSU_CLK,
		.mux_parent = cpu_mp2_dsu_clk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(cpu_mp2_dsu_clk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
		[MUX_CODEC0_WCLK_SEL] = {
		.name =MUX_CODEC0_WCLK ,
		.mux_parent = codec0_wclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(codec0_wclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
		[MUX_DISPLAY0_ACLK_SEL] = {
		.name = MUX_DISPLAY0_ACLK,
		.mux_parent = display0_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(display0_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
		[MUX_NET_WCLK_SEL] = {
		.name = MUX_NET_WCLK,
		.mux_parent = net_wclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(net_wclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
		[MUX_SOCNIC_M_TO_SYSNOC_ACLK_SEL] = {
		.name = MUX_SOCNIC_M_TO_SYSNOC_ACLK,
		.mux_parent = socnic_m_to_sysnoc_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(socnic_m_to_sysnoc_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
		[MUX_MSGBX_SWITCH0_WCLK_SEL] = {
		.name = MUX_MSGBX_SWITCH0_WCLK,
		.mux_parent = msgbx_switch0_wclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(msgbx_switch0_wclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
		[MUX_SYSNOC_TO_GPUNIC_ACLK_SEL] = {
		.name = MUX_SYSNOC_TO_GPUNIC_ACLK,
		.mux_parent = sysnoc_to_gpunic_aclk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(sysnoc_to_gpunic_aclk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_MP2_CORE_SEL] = {
		.name = MUX_CPU_MP2_CORE_CLK,
		.mux_parent = cpu_mp2_core_clk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(cpu_mp2_core_clk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_BD_EXTERNAL_MUX_CLK0_SEL] = {
		.name = MUX_BD_EXTERNAL_MUX_CLK0,
		.mux_parent = bd_external_mux_clk0_mux_parents,
		.mux_parent_count = ARRAY_SIZE(bd_external_mux_clk0_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_BD_EXTERNAL_MUX_CLK1_SEL] = {
		.name = MUX_BD_EXTERNAL_MUX_CLK1,
		.mux_parent = bd_external_mux_clk1_mux_parents,
		.mux_parent_count = ARRAY_SIZE(bd_external_mux_clk1_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_BD_EXTERNAL_MUX_CLK2_SEL] = {
		.name = MUX_BD_EXTERNAL_MUX_CLK2,
		.mux_parent = bd_external_mux_clk2_mux_parents,
		.mux_parent_count = ARRAY_SIZE(bd_external_mux_clk2_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_BD_EXTERNAL_MUX_CLK3_SEL] = {
		.name = MUX_BD_EXTERNAL_MUX_CLK3,
		.mux_parent = bd_external_mux_clk3_mux_parents,
		.mux_parent_count = ARRAY_SIZE(bd_external_mux_clk3_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_BD_EXTERNAL_MUX_CLK4_SEL] = {
		.name = MUX_BD_EXTERNAL_MUX_CLK4,
		.mux_parent = bd_external_mux_clk4_mux_parents,
		.mux_parent_count = ARRAY_SIZE(bd_external_mux_clk4_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_BD_EXTERNAL_MUX_CLK5_SEL] = {
		.name = MUX_BD_EXTERNAL_MUX_CLK5,
		.mux_parent = bd_external_mux_clk5_mux_parents,
		.mux_parent_count = ARRAY_SIZE(bd_external_mux_clk5_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_BD_EXTERNAL_MUX_CLK6_SEL] = {
		.name = MUX_BD_EXTERNAL_MUX_CLK6,
		.mux_parent = bd_external_mux_clk6_mux_parents,
		.mux_parent_count = ARRAY_SIZE(bd_external_mux_clk6_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_BD_EXTERNAL_MUX_CLK7_SEL] = {
		.name = MUX_BD_EXTERNAL_MUX_CLK7,
		.mux_parent = bd_external_mux_clk7_mux_parents,
		.mux_parent_count = ARRAY_SIZE(bd_external_mux_clk7_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_DSI_CFG_REF_CLK_SEL] = {
		.name =MUX_DSI_CFG_REF_CLK ,
		.mux_parent = dsi_cfg_ref_clk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(dsi_cfg_ref_clk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_UFS_REF_ALT_CLK_26M_SEL] = {
		.name = MUX_UFS_REF_ALT_CLK_26M,
		.mux_parent = ufs_ref_alt_clk_26m_clk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(ufs_ref_alt_clk_26m_clk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_USB_U31_PHY_REF_CLK_SEL] = {
		.name = MUX_USB_U31_PHY_REF_CLK,
		.mux_parent = usb_u31_phy_ref_clk_clk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(usb_u31_phy_ref_clk_clk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_USB_U20_PHY_REF_CLK_SEL] = {
		.name = MUX_USB_U20_PHY_REF_CLK,
		.mux_parent = usb_u20_phy_ref_clk_mux_parents,
		.mux_parent_count = ARRAY_SIZE(usb_u20_phy_ref_clk_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_GTC_WCLK_SEL] = {
		.name = MUX_GTC_WCLK,
		.mux_parent = gtc_mux_parents,
		.mux_parent_count = ARRAY_SIZE(gtc_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_LSP0_AUDIO_WCLK_4_NUM] = {
		.name = MUX_LSP0_AUDIO_WCLK_4,
		.mux_parent = lsp0_audio_wclk_4_mux_parents,
		.mux_parent_count = ARRAY_SIZE(lsp0_audio_wclk_4_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_LSP0_AUDIO_WCLK_5_NUM] = {
		.name = MUX_LSP0_AUDIO_WCLK_5,
		.mux_parent = lsp0_audio_wclk_5_mux_parents,
		.mux_parent_count = ARRAY_SIZE(lsp0_audio_wclk_5_mux_parents),
		.type = CLOCK_TREE_MUX,
	},

	[MUX_LSP0_AUDIO_WCLK_0_NUM] = {
		.name = MUX_LSP0_AUDIO_WCLK_0,
		.mux_parent = lsp0_audio_wclk_0_mux_parents,
		.mux_parent_count = ARRAY_SIZE(lsp0_audio_wclk_0_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_LSP0_AUDIO_WCLK_1_NUM] = {
		.name = MUX_LSP0_AUDIO_WCLK_1,
		.mux_parent = lsp0_audio_wclk_1_mux_parents,
		.mux_parent_count = ARRAY_SIZE(lsp0_audio_wclk_1_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_LSP0_AUDIO_WCLK_2_NUM] = {
		.name = MUX_LSP0_AUDIO_WCLK_2,
		.mux_parent = lsp0_audio_wclk_2_mux_parents,
		.mux_parent_count = ARRAY_SIZE(lsp0_audio_wclk_2_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_LSP0_AUDIO_WCLK_3_NUM] = {
		.name = MUX_LSP0_AUDIO_WCLK_3,
		.mux_parent = lsp0_audio_wclk_3_mux_parents,
		.mux_parent_count = ARRAY_SIZE(lsp0_audio_wclk_3_mux_parents),
		.type = CLOCK_TREE_MUX,
	},

	[MUX_LSP0_LSP_WCLK_MUX_NUM] = {
		.name = MUX_LSP0_LSP_WCLK_MUX,
		.mux_parent = lsp0_wclk_mux_mux_parents,
		.mux_parent_count = ARRAY_SIZE(lsp0_wclk_mux_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_LSP1_LSP_WCLK_MUX_NUM] = {
		.name = MUX_LSP1_LSP_WCLK_MUX,
		.mux_parent = lsp1_wclk_mux_mux_parents,
		.mux_parent_count = ARRAY_SIZE(lsp1_wclk_mux_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_LSP0_UART_WCLK_MUX_NUM] = {
		.name = MUX_LSP0_UART_WCLK_MUX,
		.mux_parent = lsp0_uart_wclk_mux_mux_parents,
		.mux_parent_count = ARRAY_SIZE(lsp0_uart_wclk_mux_mux_parents),
		.type = CLOCK_TREE_MUX,
	},

	[MUX_LSP1_AUDIO_WCLK_4_NUM] = {
		.name = MUX_LSP1_AUDIO_WCLK_4,
		.mux_parent = lsp1_audio_wclk_4_mux_parents,
		.mux_parent_count = ARRAY_SIZE(lsp1_audio_wclk_4_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_LSP1_AUDIO_WCLK_5_NUM] = {
		.name = MUX_LSP1_AUDIO_WCLK_5,
		.mux_parent = lsp1_audio_wclk_5_mux_parents,
		.mux_parent_count = ARRAY_SIZE(lsp1_audio_wclk_5_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_LSP1_AUDIO_WCLK_0_NUM] = {
		.name = MUX_LSP1_AUDIO_WCLK_0,
		.mux_parent = lsp1_audio_wclk_0_mux_parents,
		.mux_parent_count = ARRAY_SIZE(lsp1_audio_wclk_0_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_LSP1_AUDIO_WCLK_1_NUM] = {
		.name = MUX_LSP1_AUDIO_WCLK_1,
		.mux_parent = lsp1_audio_wclk_1_mux_parents,
		.mux_parent_count = ARRAY_SIZE(lsp1_audio_wclk_1_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_LSP1_AUDIO_WCLK_2_NUM] = {
		.name = MUX_LSP1_AUDIO_WCLK_2,
		.mux_parent = lsp1_audio_wclk_2_mux_parents,
		.mux_parent_count = ARRAY_SIZE(lsp1_audio_wclk_2_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_LSP1_AUDIO_WCLK_3_NUM] = {
		.name = MUX_LSP1_AUDIO_WCLK_3,
		.mux_parent = lsp1_audio_wclk_3_mux_parents,
		.mux_parent_count = ARRAY_SIZE(lsp1_audio_wclk_3_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[MUX_LSP1_UART_WCLK_MUX_NUM] = {
		.name = MUX_LSP1_UART_WCLK_MUX,
		.mux_parent = lsp1_uart_wclk_mux_mux_parents,
		.mux_parent_count = ARRAY_SIZE(lsp1_uart_wclk_mux_mux_parents),
		.type = CLOCK_TREE_MUX,
	},
	[DIVIDOR_TOP_USB_U20_PHY_REF_CLK] = {
		.name = DIVIDOR_TOP_USB_U20_PHY_REF_CLK_NAME,
		.parent = FACTOR_SYSBUS1_1200,
		.type = CLOCK_TREE_DIV,
	},
	[DIVIDOR_SOC_LSP0_FLEXRAY_HCLK] = {
		.name = DIVIDOR_SOC_LSP0_FLEXRAY_HCLK_NAME,
		.parent = FACTOR_SYSBUS1_1200,
		.type = CLOCK_TREE_DIV,
	},
	[DIVIDOR_SOC_LSP1_FLEXRAY_HCLK] = {
		.name = DIVIDOR_SOC_LSP1_FLEXRAY_HCLK_NAME,
		.parent = FACTOR_SYSBUS1_1200,
		.type = CLOCK_TREE_DIV,
	},
	[DIVIDOR_DISPLAY0_CH0] = {
		.name = DISPLAY0_CH0,
		.parent = MUX_DISPLAY0_SEL0,
		.type = CLOCK_TREE_DIV,
	},
	[DIVIDOR_DISPLAY0_CH1] = {
		.name = DISPLAY0_CH1,
		.parent = MUX_DISPLAY1_SEL0,
		.type = CLOCK_TREE_DIV,
	},
	[DIVIDOR_DISPLAY1_CH0] = {
		.name = DISPLAY1_CH0,
		.parent = MUX_DISPLAY2_SEL0,
		.type = CLOCK_TREE_DIV,
	},
	[DIVIDOR_DISPLAY1_CH1] = {
		.name = DISPLAY1_CH1,
		.parent = MUX_DISPLAY3_SEL0,
		.type = CLOCK_TREE_DIV,
	},
	[DIVIDOR_DISPLAY2_CH0] = {
		.name = DISPLAY2_CH0,
		.parent = MUX_DISPLAY4_SEL0,
		.type = CLOCK_TREE_DIV,
	},
	[DIVIDOR_FIX_UFS_26] = {
		.name = UFS_26,
		.parent = MUX_UFS,
		.type = CLOCK_TREE_DIV,
	},
	[DIVIDOR_MUX_BD_EXTERNAL_MUX_CLK0] = {
		.name = DIVIDOR_BD_EXTERNAL_CLK0,
		.parent = MUX_BD_EXTERNAL_MUX_CLK0,
		.type = CLOCK_TREE_DIV,
	},
	[DIVIDOR_MUX_BD_EXTERNAL_MUX_CLK1] = {
		.name = DIVIDOR_BD_EXTERNAL_CLK1,
		.parent = MUX_BD_EXTERNAL_MUX_CLK1,
		.type = CLOCK_TREE_DIV,
	},
	[DIVIDOR_MUX_BD_EXTERNAL_MUX_CLK2] = {
		.name = DIVIDOR_BD_EXTERNAL_CLK2,
		.parent = MUX_BD_EXTERNAL_MUX_CLK2,
		.type = CLOCK_TREE_DIV,
	},
	[DIVIDOR_MUX_BD_EXTERNAL_MUX_CLK3] = {
		.name = DIVIDOR_BD_EXTERNAL_CLK3,
		.parent = MUX_BD_EXTERNAL_MUX_CLK3,
		.type = CLOCK_TREE_DIV,
	},
	[DIVIDOR_MUX_BD_EXTERNAL_MUX_CLK4] = {
		.name = DIVIDOR_BD_EXTERNAL_CLK4,
		.parent = MUX_BD_EXTERNAL_MUX_CLK4,
		.type = CLOCK_TREE_DIV,
	},
	[DIVIDOR_MUX_BD_EXTERNAL_MUX_CLK5] = {
		.name = DIVIDOR_BD_EXTERNAL_CLK5,
		.parent = MUX_BD_EXTERNAL_MUX_CLK5,
		.type = CLOCK_TREE_DIV,
	},
	[DIVIDOR_MUX_BD_EXTERNAL_MUX_CLK6] = {
		.name = DIVIDOR_BD_EXTERNAL_CLK6,
		.parent = MUX_BD_EXTERNAL_MUX_CLK6,
		.type = CLOCK_TREE_DIV,
	},
	[DIVIDOR_MUX_BD_EXTERNAL_MUX_CLK7] = {
		.name = DIVIDOR_BD_EXTERNAL_CLK7,
		.parent = MUX_BD_EXTERNAL_MUX_CLK7,
		.type = CLOCK_TREE_DIV,
	},
	[DIVIDOR_GTC_WCLK] = {
		.name = DIVIDOR_GTC_DIV_WCLK,
		.parent = MUX_GTC_WCLK,
		.type = CLOCK_TREE_DIV,
	},
    [DIVIDOR_LSP0_GPIO_DBCLK_DIV_WCLK] = {
		.name = DIVIDOR_LSP0_STA_DIV_GPIO_DBCLK_DIV,
		.parent = LB_SOC_LSP0_WCLK_EN,
		.type = CLOCK_TREE_DIV,
	},
    [DIVIDOR_LSP1_GPIO_DBCLK_DIV_WCLK] = {
		.name = DIVIDOR_LSP1_STA_DIV_GPIO_DBCLK_DIV,
		.parent = LB_SOC_LSP1_WCLK_EN,
		.type = CLOCK_TREE_DIV,
	},
	[DIVIDOR_LSP0_I2S_S0_MCLK_OUT_NUM] = {
		.name = DIVIDOR_LSP0_I2S_S0_MCLK_OUT,
		.parent = MUX_LSP0_AUDIO_WCLK_4,
		.type = CLOCK_TREE_DIV,
	},
	 [DIVIDOR_LSP0_I2S_M0_WCLK_DIV_NUM] = {
		.name = DIVIDOR_LSP0_I2S_M0_WCLK_DIV,
		.parent = MUX_LSP0_AUDIO_WCLK_4,
		.type = CLOCK_TREE_DIV,
	},
	 [DIVIDOR_LSP0_I2S_S1_MCLK_OUT_NUM] = {
		.name = DIVIDOR_LSP0_I2S_S1_MCLK_OUT,
		.parent = MUX_LSP0_AUDIO_WCLK_5,
		.type = CLOCK_TREE_DIV,
	},
	 [DIVIDOR_LSP0_I2S_M1_WCLK_DIV_NUM] = {
		.name = DIVIDOR_LSP0_I2S_M1_WCLK_DIV,
		.parent = MUX_LSP0_AUDIO_WCLK_5,
		.type = CLOCK_TREE_DIV,
	},
	[DIVIDOR_LSP0_TDM0_WCLK_DIV_NUM] = {
		.name = DIVIDOR_LSP0_TDM0_WCLK_DIV,
		.parent = MUX_LSP0_AUDIO_WCLK_0,
		.type = CLOCK_TREE_DIV,
	},
	 [DIVIDOR_LSP0_TDM1_WCLK_DIV_NUM] = {
		.name = DIVIDOR_LSP0_TDM1_WCLK_DIV,
		.parent = MUX_LSP0_AUDIO_WCLK_1,
		.type = CLOCK_TREE_DIV,
	},
	 [DIVIDOR_LSP0_SPDIF0_WCLK_DIV_NUM] = {
		.name = DIVIDOR_LSP0_SPDIF0_WCLK_DIV,
		.parent = MUX_LSP0_AUDIO_WCLK_2,
		.type = CLOCK_TREE_DIV,
	},
	 [DIVIDOR_LSP0_SPDIF1_WCLK_DIV_NUM] = {
		.name = DIVIDOR_LSP0_SPDIF1_WCLK_DIV,
		.parent = MUX_LSP0_AUDIO_WCLK_3,
		.type = CLOCK_TREE_DIV,
	},
	[DIVIDOR_LSP1_I2S_S0_MCLK_OUT_NUM] = {
		.name = DIVIDOR_LSP1_I2S_S0_MCLK_OUT,
		.parent = MUX_LSP1_AUDIO_WCLK_4,
		.type = CLOCK_TREE_DIV,
	},
	 [DIVIDOR_LSP1_I2S_M0_WCLK_DIV_NUM] = {
		.name = DIVIDOR_LSP1_I2S_M0_WCLK_DIV,
		.parent = MUX_LSP1_AUDIO_WCLK_4,
		.type = CLOCK_TREE_DIV,
	},
	 [DIVIDOR_LSP1_I2S_S1_MCLK_OUT_NUM] = {
		.name = DIVIDOR_LSP1_I2S_S1_MCLK_OUT,
		.parent = MUX_LSP1_AUDIO_WCLK_5,
		.type = CLOCK_TREE_DIV,
	},
	 [DIVIDOR_LSP1_I2S_M1_WCLK_DIV_NUM] = {
		.name = DIVIDOR_LSP1_I2S_M1_WCLK_DIV,
		.parent = MUX_LSP1_AUDIO_WCLK_5,
		.type = CLOCK_TREE_DIV,
	},
	[DIVIDOR_LSP1_TDM0_WCLK_DIV_NUM] = {
		.name = DIVIDOR_LSP1_TDM0_WCLK_DIV,
		.parent = MUX_LSP1_AUDIO_WCLK_0,
		.type = CLOCK_TREE_DIV,
	},
	 [DIVIDOR_LSP1_TDM1_WCLK_DIV_NUM] = {
		.name = DIVIDOR_LSP1_TDM1_WCLK_DIV,
		.parent = MUX_LSP1_AUDIO_WCLK_1,
		.type = CLOCK_TREE_DIV,
	},
	 [DIVIDOR_LSP1_SPDIF0_WCLK_DIV_NUM] = {
		.name = DIVIDOR_LSP1_SPDIF0_WCLK_DIV,
		.parent = MUX_LSP1_AUDIO_WCLK_2,
		.type = CLOCK_TREE_DIV,
	},
	 [DIVIDOR_LSP1_SPDIF1_WCLK_DIV_NUM] = {
		.name = DIVIDOR_LSP1_SPDIF1_WCLK_DIV,
		.parent = MUX_LSP1_AUDIO_WCLK_3,
		.type = CLOCK_TREE_DIV,
	},
	[DIVIDOR_LSP0_STA_DIV_I3C_WCLK_DIV_NUM] = {
		.name = DIVIDOR_LSP0_STA_DIV_I3C_WCLK_DIV,
		.parent = LB_SOC_LSP0_WCLK_EN,
		.type = CLOCK_TREE_DIV,
	},
	 [DIVIDOR_LSP1_STA_DIV_I3C_WCLK_DIV_NUM] = {
		.name = DIVIDOR_LSP1_STA_DIV_I3C_WCLK_DIV,
		.parent = LB_SOC_LSP1_WCLK_EN,
		.type = CLOCK_TREE_DIV,
	},
	[GATE_LB_CPU1_MP4_CORE_CLK_EN] = {
        .name = LB_CPU1_MP4_CORE_CLK_EN,
        .parent = MUX_CPU1_MP4_CORE_CLK,
        .type = CLOCK_TREE_GAT,
	},
	[GATE_LB_CPU1_MP4_DSU_CLK_EN] = {
			.name = LB_CPU1_MP4_DSU_CLK_EN,
			.parent = MUX_CPU1_MP4_DSU_CHI_CLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_CPU_MP2_CORE_CLK_EN] = {
			.name = LB_CPU_MP2_CORE_CLK_EN,
			.parent = MUX_CPU_MP2_CORE_CLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_CPU_MP2_MP_ACLK_EN] = {
			.name = LB_CPU_MP2_MP_ACLK_EN,
			.parent = MUX_CPU_MP2_MP_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_CPU_MP2_DSU_CLK_EN] = {
			.name = LB_CPU_MP2_DSU_CLK_EN,
			.parent = MUX_CPU_MP2_DSU_CLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_CPU_MP2_CS_PCLK_EN] = {
			.name = LB_CPU_MP2_CS_PCLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_LVDS0_PCLK_EN] = {
			.name = LB_LVDS0_PCLK_EN,
			.parent = MUX_CLK_200_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_DBNOC_APB_BR_200_PCLK_EN] = {
			.name = DBNOC_APB_BR_200_PCLK_EN,
			.parent = MUX_CLK_200_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_CPU_MP2_DB_NOC_PCLK_EN] = {
			.name = LB_CPU_MP2_DB_NOC_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_CPU_MP2_MASTER_ACLK_EN] = {
			.name = LB_CPU_MP2_MASTER_ACLK_EN,
			.parent = MUX_CPU_MP2_MASTER_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_CMN_GWCLK_EN] = {
			.name = CMN_GWCLK_EN,
			.parent = MUX_CMN_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_GPU_G78AE_SP_S_ACLK_EN] = {
			.name = LB_GPU_G78AE_SP_S_ACLK_EN,
			.parent = MUX_G78AE_SP_WCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_GPU_G78AE_SP_PCLK_EN] = {
			.name = LB_GPU_G78AE_SP_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_GPU_G78AE_SP_M_ACLK_EN] = {
			.name = LB_GPU_G78AE_SP_M_ACLK_EN,
			.parent = MUX_G78AE_SP_WCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_CODEC0_PCLK_EN] = {
			.name = LB_CODEC0_PCLK_EN,
			.parent = MUX_CLK_200_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_CODEC1_PCLK_EN] = {
			.name = LB_CODEC1_PCLK_EN,
			.parent = MUX_CLK_200_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_GPU_G78AE_WCLK_EN] = {
			.name = LB_GPU_G78AE_WCLK_EN,
			.parent = MUX_G78AE_WCLK_CLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_GPU_G78AE_ACE_M0_ACLK_EN] = {
			.name = LB_GPU_G78AE_ACE_M0_ACLK_EN,
			.parent = MUX_G78AE_ACE_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_GPU_G78AE_ACE_M1_ACLK_EN] = {
			.name = LB_GPU_G78AE_ACE_M1_ACLK_EN,
			.parent = MUX_G78AE_ACE_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_GPU_G78AE_ACE_M2_ACLK_EN] = {
			.name = LB_GPU_G78AE_ACE_M2_ACLK_EN,
			.parent = MUX_G78AE_ACE_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_GPU_G78AE_AXI_S_ACLK_EN] = {
			.name = LB_GPU_G78AE_AXI_S_ACLK_EN,
			.parent = MUX_G78AE_AXIS_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_GPU_G78AE_TCU_WCLK_EN] = {
			.name = LB_GPU_G78AE_TCU_WCLK_EN,
			.parent = MUX_G78AE_TCU_WCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_CODEC0_WCLK_EN] = {
			.name = LB_CODEC0_WCLK_EN,
			.parent = MUX_CODEC0_WCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_CODEC1_WCLK_EN] = {
			.name = LB_CODEC1_WCLK_EN,
			.parent = MUX_CODEC1_WCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_EDP_PCLK_EN] = {
			.name = LB_EDP_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_HIFI_DSP_CFG_PCLK_EN] = {
			.name = LB_HIFI_DSP_CFG_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_HIFI_DSP_CS_PBCLK_EN] = {
			.name = LB_HIFI_DSP_CS_PBCLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_HIFI_DSP_WCLK_EN] = {
			.name = LB_HIFI_DSP_WCLK_EN,
			.parent = MUX_HIFI_DSP_WCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_HIFI_DSP_XNNE_WCLK_EN] = {
			.name = LB_HIFI_DSP_XNNE_WCLK_EN,
			.parent = MUX_HIFI_DSP_XNNE_WCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_HIFI_DSP_S_ACLK_EN] = {
			.name = LB_HIFI_DSP_S_ACLK_EN,
			.parent = MUX_HIFI_DSP_ACLK               ,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_DB_DMA_ACLK_EN] = {
			.name = LB_DB_DMA_ACLK_EN,
			.parent = MUX_DB_DMA_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_DB_DMA_HCLK_EN] = {
			.name = LB_DB_DMA_HCLK_EN,
			.parent = MUX_DB_DMA_HCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_SOC_DMA_ACLK_EN] = {
			.name = LB_SOC_DMA_ACLK_EN,
			.parent = MUX_SOC_DMA_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_SOC_DMA_HCLK_EN] = {
			.name = LB_SOC_DMA_HCLK_EN,
			.parent = MUX_SOC_DMA_HCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_MEDIA_DMA_ACLK_EN] = {
			.name = LB_MEDIA_DMA_ACLK_EN,
			.parent = MUX_MEDIA_DMA_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_MEDIA_DMA_HCLK_EN] = {
			.name = LB_MEDIA_DMA_HCLK_EN,
			.parent = MUX_MEDIA_DMA_HCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_CS_DMA_ACLK_EN] = {
			.name = LB_CS_DMA_ACLK_EN,
			.parent = MUX_CS_DMA_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_CS_DMA_HCLK_EN] = {
			.name = LB_CS_DMA_HCLK_EN,
			.parent = MUX_CS_DMA_HCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_DISPLAY0_CH0_CLK_EN] = {
			.name = LB_DISPLAY0_CH0_CLK_EN,
			.parent = DISPLAY0_CH0,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_DISPLAY0_CH1_CLK_EN] = {
			.name = LB_DISPLAY0_CH1_CLK_EN,
			.parent = DISPLAY0_CH1,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_DISPLAY1_CH0_CLK_EN] = {
			.name = LB_DISPLAY1_CH0_CLK_EN,
			.parent = DISPLAY1_CH0,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_DISPLAY1_CH1_CLK_EN] = {
			.name = LB_DISPLAY1_CH1_CLK_EN,
			.parent = DISPLAY1_CH1,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_DISPLAY2_CH0_CLK_EN] = {
			.name = LB_DISPLAY2_CH0_CLK_EN,
			.parent = DISPLAY2_CH0,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_LVDS_CH0_HSPEED_CLK_EN] = {
			.name = LB_LVDS_CH0_HSPEED_CLK_EN,
			.parent = MUX_CLK_LVDS0_HSPEED,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_LVDS_CH1_HSPEED_CLK_EN] = {
			.name = LB_LVDS_CH1_HSPEED_CLK_EN,
			.parent = MUX_CLK_LVDS1_HSPEED,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_DISPLAY0_ACLK_EN] = {
			.name = LB_DISPLAY0_ACLK_EN,
			.parent = MUX_DISPLAY0_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_DISPLAY1_ACLK_EN] = {
			.name = LB_DISPLAY1_ACLK_EN,
			.parent = MUX_DISPLAY1_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_DISPLAY2_ACLK_EN] = {
			.name = LB_DISPLAY2_ACLK_EN,
			.parent = MUX_DISPLAY2_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_DISPLAY0_PCLK_EN] = {
			.name = LB_DISPLAY0_PCLK_EN,
			.parent = MUX_CLK_200_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_DISPLAY1_PCLK_EN] = {
			.name = LB_DISPLAY1_PCLK_EN,
			.parent = MUX_CLK_200_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_DISPLAY2_PCLK_EN] = {
			.name = LB_DISPLAY2_PCLK_EN,
			.parent = MUX_CLK_200_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_PCIE_IST_ATSPEED_500M_CLK_EN] = {
			.name = LB_PCIE_IST_ATSPEED_500M_CLK_EN,
			.parent = FACTOR_SYSBUS0_500,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_PCIE_OSC_CLK_EN] = {
			.name = LB_PCIE_OSC_CLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_PCIE_APB_PCLK_EN] = {
			.name = LB_PCIE_APB_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_PCIE_DBI_ACLK_EN] = {
			.name = LB_PCIE_DBI_ACLK_EN,
			.parent = MUX_PCIE_DBI_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_PCIE_SLV_ACLK_EN] = {
			.name = LB_PCIE_SLV_ACLK_EN,
			.parent = MUX_PCIE_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_PCIE_X2_MSTR_ACLK_EN] = {
			.name = LB_PCIE_X2_MSTR_ACLK_EN,
			.parent = MUX_PCIE_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_PCIE_X4_MSTR_ACLK_EN] = {
			.name = LB_PCIE_X4_MSTR_ACLK_EN,
			.parent = MUX_PCIE_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_USB0_IST_ATSPEED_666M_CLK_EN] = {
			.name = LB_USB0_IST_ATSPEED_666M_CLK_EN,
			.parent = FACTOR_SYSBUS0_666,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_USB0_IST_ATSPEED_75M_CLK_EN] = {
			.name = LB_USB0_IST_ATSPEED_75M_CLK_EN,
			.parent = FACTOR_SYSBUS0_666,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_USB_0_REF_ALT_CLK_EN] = {
			.name = LB_USB_0_REF_ALT_CLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_USB_0_AXI_ACLK_EN] = {
			.name = LB_USB_0_AXI_ACLK_EN,
			.parent = MUX_USB_0_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_USB_0_APB_PCLK_EN] = {
			.name = LB_USB_0_APB_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_USB1_IST_ATSPEED_666M_CLK_EN] = {
			.name = LB_USB1_IST_ATSPEED_666M_CLK_EN,
			.parent = FACTOR_SYSBUS0_666,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_USB1_IST_ATSPEED_75M_CLK_EN] = {
			.name = LB_USB1_IST_ATSPEED_75M_CLK_EN,
			.parent = FACTOR_SYSBUS0_666,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_USB_1_REF_ALT_CLK_EN] = {
			.name = LB_USB_1_REF_ALT_CLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_USB_1_AXI_ACLK_EN] = {
			.name = LB_USB_1_AXI_ACLK_EN,
			.parent = MUX_USB_1_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_USB_1_APB_PCLK_EN] = {
			.name = LB_USB_1_APB_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_SDEMMC0_W_BCLK_EN] = {
			.name = LB_SDEMMC0_W_BCLK_EN,
			.parent = MUX_SDEMMC0_W_BCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_SDEMMC0_M_HCLK_EN] = {
			.name = LB_SDEMMC0_M_HCLK_EN,
			.parent = MUX_SDEMMC0_HCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_SDEMMC0_S_PCLK_EN] = {
			.name = LB_SDEMMC0_S_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_SDEMMC1_S_PCLK_EN] = {
			.name = LB_SDEMMC1_S_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_SDEMMC0_S_HCLK_EN] = {
			.name = LB_SDEMMC0_S_HCLK_EN,
			.parent = MUX_SDEMMC0_HCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_SDEMMC1_W_BCLK_EN] = {
			.name = LB_SDEMMC1_W_BCLK_EN,
			.parent = MUX_SDEMMC1_W_BCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_SDEMMC1_M_HCLK_EN] = {
			.name = LB_SDEMMC1_M_HCLK_EN,
			.parent = MUX_SDEMMC1_HCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_SDEMMC1_S_HCLK_EN] = {
			.name = LB_SDEMMC1_S_HCLK_EN,
			.parent = MUX_SDEMMC1_HCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_UFS_AXI_ACLK_EN] = {
			.name = LB_UFS_AXI_ACLK_EN,
			.parent = MUX_UFS_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_SOC_LSP0_UART_WCLK_EN] = {
			.name = LB_SOC_LSP0_UART_WCLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_SOC_LSP1_UART_WCLK_EN] = {
			.name = LB_SOC_LSP1_UART_WCLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_SOC_LSP0_WCLK_EN] = {
			.name = LB_SOC_LSP0_WCLK_EN,
			.parent = MUX_SOC_LSP0_WCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_SOC_LSP1_WCLK_EN] = {
			.name = LB_SOC_LSP1_WCLK_EN,
			.parent = MUX_SOC_LSP1_WCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SAFENOC_TO_SWNOC_GACLK_EN] = {
			.name = SAFENOC_TO_SWNOC_GACLK_EN,
			.parent = MUX_SAFENOC_TO_SWNOC_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SWNOC_TO_SYSNOC_GACLK_EN] = {
			.name = SWNOC_TO_SYSNOC_GACLK_EN,
			.parent = MUX_SWNOC_TO_SYSNOC_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_MIPI0_APB_CFG_PCLK_EN] = {
			.name = LB_MIPI0_APB_CFG_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_MIPI1_APB_CFG_PCLK_EN] = {
			.name = LB_MIPI1_APB_CFG_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_MIPI2_APB_CFG_PCLK_EN] = {
			.name = LB_MIPI2_APB_CFG_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_MIPI0_PHY_CFG_CLK_EN] = {
			.name = LB_MIPI0_PHY_CFG_CLK_EN,
			.parent = UFS_26,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_MIPI1_PHY_CFG_CLK_EN] = {
			.name = LB_MIPI1_PHY_CFG_CLK_EN,
			.parent = UFS_26,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_MIPI2_PHY_CFG_CLK_EN] = {
			.name = LB_MIPI2_PHY_CFG_CLK_EN,
			.parent = UFS_26,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_ISP_SCLK_EN] = {
			.name = LB_ISP_SCLK_EN,
			.parent = MUX_ISP_SCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_CV_AXIS_CLK_EN] = {
			.name = LB_CV_AXIS_CLK_EN,
			.parent = MUX_CV_AXIS_CLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_CV_AXIM0_CLK_EN] = {
			.name = LB_CV_AXIM0_CLK_EN,
			.parent = MUX_CV_AXIM0_CLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_CV_AXIM1_CLK_EN] = {
			.name = LB_CV_AXIM1_CLK_EN,
			.parent = MUX_CV_AXIM1_CLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_CV_CORE_CLK_EN] = {
			.name = LB_CV_CORE_CLK_EN,
			.parent = MUX_CV_CORE_CLK                  ,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_CV_DSP0_PBCLK_EN] = {
			.name = LB_CV_DSP0_PBCLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_CV_DSP1_PBCLK_EN] = {
			.name = LB_CV_DSP1_PBCLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_CV_DSP2_PBCLK_EN] = {
			.name = LB_CV_DSP2_PBCLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_CV_DSP3_PBCLK_EN] = {
			.name = LB_CV_DSP3_PBCLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_NET_CLK_EN] = {
			.name = LB_NET_CLK_EN,
			.parent = MUX_NET_WCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_NET_DSP_PBCLK_EN] = {
			.name = LB_NET_DSP_PBCLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_NET_DSP_CLK_EN] = {
			.name = LB_NET_DSP_CLK_EN,
			.parent = MUX_DSP_CLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_LPDDR5_0_S0_ACLK_EN] = {
			.name = LB_LPDDR5_0_S0_ACLK_EN,
			.parent = MUX_LPDDR5_0_S0_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_LPDDR5_0_S1_ACLK_EN] = {
			.name = LB_LPDDR5_0_S1_ACLK_EN,
			.parent = MUX_LPDDR5_0_S1_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_LPDDR5_0_S2_ACLK_EN] = {
			.name = LB_LPDDR5_0_S2_ACLK_EN,
			.parent = MUX_LPDDR5_0_S2_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_LPDDR5_0_S3_ACLK_EN] = {
			.name = LB_LPDDR5_0_S3_ACLK_EN,
			.parent = MUX_LPDDR5_0_S3_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_LPDDR5_0_S4_ACLK_EN] = {
			.name = LB_LPDDR5_0_S4_ACLK_EN,
			.parent = MUX_LPDDR5_0_S4_ACLKK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_LPDDR5_1_S0_ACLK_EN] = {
			.name = LB_LPDDR5_1_S0_ACLK_EN,
			.parent = MUX_LPDDR5_1_S0_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_LPDDR5_1_S1_ACLK_EN] = {
			.name = LB_LPDDR5_1_S1_ACLK_EN,
			.parent = MUX_LPDDR5_1_S1_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_LPDDR5_1_S2_ACLK_EN] = {
			.name = LB_LPDDR5_1_S2_ACLK_EN,
			.parent = MUX_LPDDR5_1_S2_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_LPDDR5_1_S3_ACLK_EN] = {
			.name = LB_LPDDR5_1_S3_ACLK_EN,
			.parent = MUX_LPDDR5_1_S3_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_LPDDR5_1_S4_ACLK_EN] = {
			.name = LB_LPDDR5_1_S4_ACLK_EN,
			.parent = MUX_LPDDR5_1_S4_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_LPDDR5_0_PCLK_EN] = {
			.name = LB_LPDDR5_0_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_LPDDR5_1_PCLK_EN] = {
			.name = LB_LPDDR5_1_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_CORE_NOC_GWCLK_EN] = {
			.name = CORE_NOC_GWCLK_EN,
			.parent = MUX_MEDIA_NOC_WCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_DB_NOC_GWCLK_EN] = {
			.name = DB_NOC_GWCLK_EN,
			.parent = MUX_DB_NOC_WCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_MEDIA_NOC_GWCLK_EN] = {
			.name = MEDIA_NOC_GWCLK_EN,
			.parent = MUX_MEDIA_NOC_WCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SYS_NOC_800M_GWCLK_EN] = {
			.name = SYS_NOC_800M_GWCLK_EN,
			.parent = MUX_SYS_NOC_800M_WCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SYS_NOC_400M_GWCLK_EN] = {
			.name = SYS_NOC_400M_GWCLK_EN,
			.parent = MUX_SYS_NOC_400M_WCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SYSNOC_APB0_200_PCLK_EN] = {
			.name = SYSNOC_APB0_200_PCLK_EN,
			.parent = MUX_CLK_200_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SYSNOC_APB1_200_PCLK_EN] = {
			.name = SYSNOC_APB1_200_PCLK_EN,
			.parent = MUX_CLK_200_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_MATRIX_TOP_CRM_APB_PCLK_EN] = {
			.name = LB_MATRIX_TOP_CRM_APB_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_MATRIX_PMM_REG_APB_PCLK_EN] = {
			.name = LB_MATRIX_PMM_REG_APB_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_MATRIX_SYS_CTRL_APB_PCLK_EN] = {
			.name = LB_MATRIX_SYS_CTRL_APB_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_MATRIX_IPC_APB_PCLK_EN] = {
			.name = LB_MATRIX_IPC_APB_PCLK_EN,
			.parent = MUX_CLK_200_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_COREIP_SUBSYSTEM_CSR_APB_PCLK_EN] = {
			.name = COREIP_SUBSYSTEM_CSR_APB_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_DB_SUBSYSTEM_CSR_APB_PCLK_EN] = {
			.name = DB_SUBSYSTEM_CSR_APB_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_MEDIA_SUBSYSTEM_CSR_APB_PCLK_EN] = {
			.name = MEDIA_SUBSYSTEM_CSR_APB_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_CPU_SUBSYSTEM_CSR_APB_PCLK_EN] = {
			.name = CPU_SUBSYSTEM_CSR_APB_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_MEDIANOC_APB_200_PCLK_EN] = {
			.name = MEDIANOC_APB_200_PCLK_EN,
			.parent = MUX_CLK_200_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_USBNIC_TO_SYSNOC_GACLK_EN] = {
			.name = USBNIC_TO_SYSNOC_GACLK_EN,
			.parent = MUX_USBNIC_TO_SYSNOC_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SDNIC_TO_SYSNOC_GACLK_EN] = {
			.name = SDNIC_TO_SYSNOC_GACLK_EN,
			.parent = MUX_SDNIC_TO_SYSNOC_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SOCNIC_S_TO_DBNOC_GACLK_EN] = {
			.name = SOCNIC_S_TO_DBNOC_GACLK_EN,
			.parent = MUX_SOCNIC_S_TO_DBNOC_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_DBNOC_TO_DBSOCNOC_M_GACLK_EN] = {
			.name = DBNOC_TO_DBSOCNOC_M_GACLK_EN,
			.parent = MUX_DBNOC_TO_DBSOCNOC_M_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_RTNOC_TO_SOCNIC_M_GACLK_EN] = {
			.name = RTNOC_TO_SOCNIC_M_GACLK_EN,
			.parent = MUX_RTNOC_TO_SOCNIC_M_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SECNOC_TO_RTNOC_GACLK_EN] = {
			.name = SECNOC_TO_RTNOC_GACLK_EN,
			.parent = MUX_SECNOC_TO_RTNOC_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SECNOC_TO_SOCNIC_M_GACLK_EN] = {
			.name = SECNOC_TO_SOCNIC_M_GACLK_EN,
			.parent = MUX_SECNOC_TO_SOCNIC_M_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_CORENOC_TO_SOCNIC_M_GACLK_EN] = {
			.name = CORENOC_TO_SOCNIC_M_GACLK_EN,
			.parent = MUX_CORENOC_TO_SOCNIC_M_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SAFETYNOC_TO_SOCNIC_M_GACLK_EN] = {
			.name = SAFETYNOC_TO_SOCNIC_M_GACLK_EN,
			.parent = MUX_SAFETYNOC_TO_SOCNIC_M_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SOCNIC_M_TO_SYSNOC_GACLK_EN] = {
			.name = SOCNIC_M_TO_SYSNOC_GACLK_EN,
			.parent = MUX_SOCNIC_M_TO_SYSNOC_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SYSNOC_TO_SOCNIC_S_GACLK_EN] = {
			.name = SYSNOC_TO_SOCNIC_S_GACLK_EN,
			.parent = MUX_SYSNOC_TO_SOCNIC_S_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SOCNIC_S_TO_SWNOC_GACLK_EN] = {
			.name = SOCNIC_S_TO_SWNOC_GACLK_EN,
			.parent = MUX_SOCNIC_S_TO_SWNOC_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SOCNIC_S_TO_SAFENOC_GACLK_EN] = {
			.name = SOCNIC_S_TO_SAFENOC_GACLK_EN,
			.parent = MUX_SOCNIC_S_TO_SAFENOC_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SOCNIC_S_TO_CORENOC_GACLK_EN] = {
			.name = SOCNIC_S_TO_CORENOC_GACLK_EN,
			.parent = MUX_SOCNIC_S_TO_CORENOC_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SOCNIC_S_TO_MEDIANOC_GACLK_EN] = {
			.name = SOCNIC_S_TO_MEDIANOC_GACLK_EN,
			.parent = MUX_SOCNIC_S_TO_MEDIANOC_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_PCIENOC_TO_CMN_GACLK_EN] = {
			.name = PCIENOC_TO_CMN_GACLK_EN,
			.parent = MUX_PCIE_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_MEDIANOC_TO_CMN_GACLK_EN] = {
			.name = MEDIANOC_TO_CMN_GACLK_EN,
			.parent = MUX_MEDIANOC_TO_CMN_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_CORENOC_TO_CMN_GACLK_EN] = {
			.name = CORENOC_TO_CMN_GACLK_EN,
			.parent = MUX_CORENOC_TO_CMN_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SYSNOC_TO_CMN_GACLK_EN] = {
			.name = SYSNOC_TO_CMN_GACLK_EN,
			.parent = MUX_SYSNOC_TO_CMN_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_CMN_TO_SYSNOC_GACLK_EN] = {
			.name = CMN_TO_SYSNOC_GACLK_EN,
			.parent = MUX_CMN_TO_SYSNOC_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SOC_SRAM_GACLK_EN] = {
			.name = SOC_SRAM_GACLK_EN,
			.parent = MUX_SOC_SRAM_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SYSNOC_TO_DBNOC_GACLK_EN] = {
			.name = SYSNOC_TO_DBNOC_GACLK_EN,
			.parent = MUX_SYSNOC_TO_DBNOC_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_RTNOC_TO_SAFENOC_GACLK_EN] = {
			.name = RTNOC_TO_SAFENOC_GACLK_EN,
			.parent = MUX_RTNOC_SAFENOC_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SAFENOC_TO_RTNOC_GACLK_EN] = {
			.name = SAFENOC_TO_RTNOC_GACLK_EN,
			.parent = MUX_RTNOC_SAFENOC_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_RTNOC_TO_SWNOC_GACLK_EN] = {
			.name = RTNOC_TO_SWNOC_GACLK_EN,
			.parent = MUX_RTNOC_SWNOC_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SWNOC_TO_RTNOC_GACLK_EN] = {
			.name = SWNOC_TO_RTNOC_GACLK_EN,
			.parent = MUX_RTNOC_SWNOC_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_EDP_SUSPEND_CLK_EN] = {
			.name = LB_EDP_SUSPEND_CLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_EDP_CLK_800M_EN] = {
			.name = LB_EDP_CLK_800M_EN,
			.parent = FACTOR_SYSBUS1_800,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_CPU_PERIP_SYSNOC_APB_PCLK_EN] = {
			.name = LB_CPU_PERIP_SYSNOC_APB_PCLK_EN,
			.parent = MUX_CLK_200_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_CPU_PERIP_CLK_EN] = {
			.name = LB_CPU_PERIP_CLK_EN,
			.parent = MUX_CPU_PERIP_CLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SMMU_TCU_COREIP_GACLK_EN] = {
			.name = SMMU_TCU_COREIP_GACLK_EN,
			.parent = MUX_SMMU_TCU_COREIP_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SMMU_TCU_CODEC_GACLK_EN] = {
			.name = SMMU_TCU_CODEC_GACLK_EN,
			.parent = MUX_SMMU_TCU_CODEC_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_PCIE_NOC_600M_CLK_EN] = {
			.name = LB_PCIE_NOC_600M_CLK_EN,
			.parent = MUX_PCIE_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_CPU_PERIP_CSTOPCLK_EN] = {
			.name = LB_CPU_PERIP_CSTOPCLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_ISP_CV_MIPI_PVT_SYS_CLK_EN] = {
			.name = LB_ISP_CV_MIPI_PVT_SYS_CLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_NET_PVT_SYS_CLK_EN] = {
			.name = LB_NET_PVT_SYS_CLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_CPU0_MP4_PVT_SYS_CLK_EN] = {
			.name = LB_CPU0_MP4_PVT_SYS_CLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_CPU1_MP4_PVT_SYS_CLK_EN] = {
			.name = LB_CPU1_MP4_PVT_SYS_CLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_GPU_G78AE_PVT_SYS_CLK_EN] = {
			.name = LB_GPU_G78AE_PVT_SYS_CLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_VDD_SOC_PVT_SYS_CLK_EN] = {
			.name = LB_VDD_SOC_PVT_SYS_CLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_VDD_SAFETY_PVT_SYS_CLK_EN] = {
			.name = LB_VDD_SAFETY_PVT_SYS_CLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_SW_CSTOP_CLK_EN] = {
			.name = LB_SW_CSTOP_CLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SYSNOC_ATBCLK_EN] = {
			.name = SYSNOC_ATBCLK_EN,
			.parent = MUX_NOC_ATB_CLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_SW_SUBSYSTEM_IST_PCLK_EN] = {
			.name = LB_SW_SUBSYSTEM_IST_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_DSI_PCLK_EN] = {
			.name = LB_DSI_PCLK_EN,
			.parent = MUX_CLK_200_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_DSI_CFG_REF_CLK_EN] = {
			.name = LB_DSI_CFG_REF_CLK_EN,
			.parent = MUX_DSI_CFG_REF_CLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_CMN_CSR_PCLK_EN] = {
			.name = CMN_CSR_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_SOC_LSP0_APB_S0_PCLK_EN] = {
			.name = LB_SOC_LSP0_APB_S0_PCLK_EN,
			.parent = MUX_CLK_200_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_SOC_LSP1_APB_S0_PCLK_EN] = {
			.name = LB_SOC_LSP1_APB_S0_PCLK_EN,
			.parent = MUX_CLK_200_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_LPDDR5_0_MAIN_CLK_EN] = {
			.name = LB_LPDDR5_0_MAIN_CLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_LPDDR5_1_MAIN_CLK_EN] = {
			.name = LB_LPDDR5_1_MAIN_CLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SOC_PROBE_APB_PCLK_EN] = {
			.name = SOC_PROBE_APB_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SOC_PROBE_AXI_GACLK_EN] = {
			.name = SOC_PROBE_AXI_GACLK_EN,
			.parent = MUX_PROBE_AXI_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_DDR0_PROBE_AXI_GACLK_EN] = {
			.name = DDR0_PROBE_AXI_GACLK_EN,
			.parent = MUX_PROBE_AXI_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_DDR1_PROBE_AXI_GACLK_EN] = {
			.name = DDR1_PROBE_AXI_GACLK_EN,
			.parent = MUX_PROBE_AXI_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_MEDIA_PROBE_APB_PCLK_EN] = {
			.name = MEDIA_PROBE_APB_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_MEDIA_PROBE_AXI_GACLK_EN] = {
			.name = MEDIA_PROBE_AXI_GACLK_EN,
			.parent = MUX_PROBE_AXI_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_COREIP_PROBE_APB_PCLK_EN] = {
			.name = COREIP_PROBE_APB_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_COREIP_PROBE_AXI_GACLK_EN] = {
			.name = COREIP_PROBE_AXI_GACLK_EN,
			.parent = MUX_PROBE_AXI_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_UFS_APB_PCLK_EN] = {
			.name = LB_UFS_APB_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_UFS_REF_ALT_CLK_EN] = {
			.name = LB_UFS_REF_ALT_CLK_EN,
			.parent = MUX_UFS_REF_ALT_CLK_26M,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_TOP_IST_PCLK_EN] = {
			.name = TOP_IST_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SOC_IST_CLK_EN] = {
			.name = SOC_IST_CLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SAFETY_COREIP_PCLK_EN] = {
			.name = SAFETY_COREIP_PCLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_STANDBY_APB_PCLK_EN] = {
			.name = LB_STANDBY_APB_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SOC_SAFE_CSR_PCLK_EN] = {
			.name = SOC_SAFE_CSR_PCLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_NET_SAFETY_PCLK_EN] = {
			.name = LB_NET_SAFETY_PCLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_ISP_SAFE_PCLK_EN] = {
			.name = LB_ISP_SAFE_PCLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_CV_SAFE_PCLK_EN] = {
			.name = LB_CV_SAFE_PCLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_UFS_IST_ATSPEED_600M_CLK_EN] = {
			.name = LB_UFS_IST_ATSPEED_600M_CLK_EN,
			.parent = FACTOR_SYSBUS1_600,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SAFETY_TO_SOC_SYS_CSTOP_CLK_EN] = {
			.name = SAFETY_TO_SOC_SYS_CSTOP_CLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SOC_SYS_TO_DSP_CSTOP_CLK_EN] = {
			.name = SOC_SYS_TO_DSP_CSTOP_CLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_RT_CSTOP_CLK_EN] = {
			.name = LB_RT_CSTOP_CLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_ATB_CSTOP_CLK_EN] = {
			.name = ATB_CSTOP_CLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_ATB_APB_GPCLK_EN] = {
			.name = ATB_APB_GPCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_ATB_AXI_GACLK_EN] = {
			.name = ATB_AXI_GACLK_EN,
			.parent = MUX_ATB_AXI_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_NET_IST_ATSPEED_25M_CLK_EN] = {
			.name = LB_NET_IST_ATSPEED_25M_CLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_DSP0_IST_ATSPEED_25M_CLK_EN] = {
			.name = DSP0_IST_ATSPEED_25M_CLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_DSP1_IST_ATSPEED_25M_CLK_EN] = {
			.name = DSP1_IST_ATSPEED_25M_CLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_DSP2_IST_ATSPEED_25M_CLK_EN] = {
			.name = DSP2_IST_ATSPEED_25M_CLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_DSP3_IST_ATSPEED_25M_CLK_EN] = {
			.name = DSP3_IST_ATSPEED_25M_CLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_HIFI_DSP_IST_ATSPEED_25M_CLK_EN] = {
			.name = LB_HIFI_DSP_IST_ATSPEED_25M_CLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_XGMAC_AXIM_ACLK_EN] = {
			.name = LB_XGMAC_AXIM_ACLK_EN,
			.parent = MUX_XGMAC_AXI_ACLK            ,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_XGMAC_APB_S_PCLK_EN] = {
			.name = LB_XGMAC_APB_S_PCLK_EN,
			.parent = MUX_XGMAC_APB_PCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_SOC_LSP0_IST_ATSPEED_25M_CLK_EN] = {
			.name = LB_SOC_LSP0_IST_ATSPEED_25M_CLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_SOC_LSP1_IST_ATSPEED_25M_CLK_EN] = {
			.name = LB_SOC_LSP1_IST_ATSPEED_25M_CLK_EN,
			.parent = CLK_25M_OSC,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_MSGBX_SWITCH0_GWCLK_EN] = {
			.name = MSGBX_SWITCH0_GWCLK_EN,
			.parent = MUX_MSGBX_SWITCH0_WCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_MSGBX_SWITCH0_TO_ISPCV_GHCLK_EN] = {
			.name = MSGBX_SWITCH0_TO_ISPCV_GHCLK_EN,
			.parent = MUX_MSGBX_SWITCH0_TO_ISPCV_HCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_MSGBX_SWITCH0_TO_DB_GHCLK_EN] = {
			.name = MSGBX_SWITCH0_TO_DB_GHCLK_EN,
			.parent = MUX_MSGBX_SWITCH0_TO_DB_HCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_MSGBX_SWITCH0_TO_CPU_GHCLK_EN] = {
			.name = MSGBX_SWITCH0_TO_CPU_GHCLK_EN,
			.parent = MUX_MSGBX_SWITCH0_TO_CPU_HCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_MSGBX_SWITCH0_TO_NET_GHCLK_EN] = {
			.name = MSGBX_SWITCH0_TO_NET_GHCLK_EN,
			.parent = MUX_MSGBX_SWITCH0_TO_NET_HCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_MSGBX_SWITCH0_TO_SOCDMA_GHCLK_EN] = {
			.name = MSGBX_SWITCH0_TO_SOCDMA_GHCLK_EN,
			.parent = MUX_MSGBX_SWITCH0_TO_SOCDMA_HCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_MSGBX_SWITCH1_GWCLK_EN] = {
			.name = MSGBX_SWITCH1_GWCLK_EN,
			.parent = MUX_MSGBX_SWITCH1_WCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_MSGBX_SWITCH1_TO_SW_GHCLK_EN] = {
			.name = MSGBX_SWITCH1_TO_SW_GHCLK_EN,
			.parent = MUX_MSGBX_SWITCH1_TO_SW_HCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_MSGBX_SWITCH1_TO_MEDIA_GHCLK_EN] = {
			.name = MSGBX_SWITCH1_TO_MEDIA_GHCLK_EN,
			.parent = MUX_MSGBX_SWITCH1_TO_MEDIA_HCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_MSGBX_SWITCH0_TO_SWITCH1_GWCLK_EN] = {
			.name = MSGBX_SWITCH0_TO_SWITCH1_GWCLK_EN,
			.parent = MUX_MSGBX_SWITCH0_TO_SWITCH1_WCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_MSGBX_SWITCH1_TO_REALTIME_GHCLK_EN] = {
			.name = MSGBX_SWITCH1_TO_REALTIME_GHCLK_EN,
			.parent = MUX_MSGBX_SWITCH1_TO_REALTIME_HCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_MSGBX_SWITCH1_TO_SECURE_GHCLK_EN] = {
			.name = MSGBX_SWITCH1_TO_SECURE_GHCLK_EN,
			.parent = MUX_MSGBX_SWITCH1_TO_SECURE_HCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_MSGBX_SWITCH1_TO_SAFE_GHCLK_EN] = {
			.name = MSGBX_SWITCH1_TO_SAFE_GHCLK_EN,
			.parent = MUX_MSGBX_SWITCH1_TO_SAFE_HCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_ATB_TPIU_TRACE_GCLK_EN] = {
			.name = ATB_TPIU_TRACE_GCLK_EN,
			.parent = MUX_ATB_TPIU_TRACE_CLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_DBSOCNOC_GWCLK_EN] = {
			.name = DBSOCNOC_GWCLK_EN,
			.parent = MUX_DBSOCNOC_WCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SYSNOC_TO_DBSOCNOC_GACLK_EN] = {
			.name = SYSNOC_TO_DBSOCNOC_GACLK_EN,
			.parent = MUX_SYSNOC_TO_DBSOCNOC_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_DBSOCNOC_TO_CMN_GACLK_EN] = {
			.name = DBSOCNOC_TO_CMN_GACLK_EN,
			.parent = MUX_DBSOCNOC_TO_CMN_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_DBSOCNOC_TO_SYSNOC_GACLK_EN] = {
			.name = DBSOCNOC_TO_SYSNOC_GACLK_EN,
			.parent = MUX_DBSOCNOC_TO_SYSNOC_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_EDP_ACLK_EN] = {
			.name = LB_EDP_ACLK_EN,
			.parent = MUX_EDP_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_SOC_SEC_ACLK_EN] = {
			.name = LB_SOC_SEC_ACLK_EN,
			.parent = MUX_SOC_SEC_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_SOC_SEC_HCLK_EN] = {
			.name = LB_SOC_SEC_HCLK_EN,
			.parent = MUX_SOC_SEC_HCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SYSNOC_TO_SOCNIC_AHB_CFG0_GHCLK_EN] = {
			.name = SYSNOC_TO_SOCNIC_AHB_CFG0_GHCLK_EN,
			.parent = MUX_SYSNOC_TO_SOCNIC_AHB_CFG_HCLK  ,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SYSNOC_TO_SOCNIC_AHB_CFG1_GHCLK_EN] = {
			.name = SYSNOC_TO_SOCNIC_AHB_CFG1_GHCLK_EN,
			.parent = MUX_SYSNOC_TO_SOCNIC_AHB_CFG_HCLK  ,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_SYSNOC_TO_GPUNIC_GACLK_EN] = {
			.name = SYSNOC_TO_GPUNIC_GACLK_EN,
			.parent = MUX_SYSNOC_TO_GPUNIC_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_BD_EXTERNAL_GCLK0_EN] = {
			.name = BD_EXTERNAL_GCLK0_EN,
			.parent = DIVIDOR_BD_EXTERNAL_CLK0,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_BD_EXTERNAL_GCLK1_EN] = {
			.name = BD_EXTERNAL_GCLK1_EN,
			.parent = DIVIDOR_BD_EXTERNAL_CLK1,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_BD_EXTERNAL_GCLK2_EN] = {
			.name = BD_EXTERNAL_GCLK2_EN,
			.parent = DIVIDOR_BD_EXTERNAL_CLK2,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_BD_EXTERNAL_GCLK3_EN] = {
			.name = BD_EXTERNAL_GCLK3_EN,
			.parent = DIVIDOR_BD_EXTERNAL_CLK3,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_BD_EXTERNAL_GCLK4_EN] = {
			.name = BD_EXTERNAL_GCLK4_EN,
			.parent = DIVIDOR_BD_EXTERNAL_CLK4,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_BD_EXTERNAL_GCLK5_EN] = {
			.name = BD_EXTERNAL_GCLK5_EN,
			.parent = DIVIDOR_BD_EXTERNAL_CLK5,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_BD_EXTERNAL_GCLK6_EN] = {
			.name = BD_EXTERNAL_GCLK6_EN,
			.parent = DIVIDOR_BD_EXTERNAL_CLK6,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_BD_EXTERNAL_GCLK7_EN] = {
			.name = BD_EXTERNAL_GCLK7_EN,
			.parent = DIVIDOR_BD_EXTERNAL_CLK7,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_ISP_IST_ATSPEED_TEST_CLK_EN] = {
			.name = LB_ISP_IST_ATSPEED_TEST_CLK_EN,
			.parent = MUX_CLK_200_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_EDP_REF_ALT_CLK_EN] = {
			.name = LB_EDP_REF_ALT_CLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_HIFI_DSP_M_ACLK_EN] = {
			.name = LB_HIFI_DSP_M_ACLK_EN,
			.parent = MUX_HIFI_DSP_ACLK               ,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_REALTIME_SUBSYSTEM_IST_PCLK_EN] = {
			.name = LB_REALTIME_SUBSYSTEM_IST_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_MEDIANOC_TO_SYSNOC_GACLK_EN] = {
			.name = MEDIANOC_TO_SYSNOC_GACLK_EN,
			.parent = MUX_MEDIANOC_TO_SYSNOC_ACLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_TOP_GTC_PCLK_EN] = {
			.name = TOP_GTC_PCLK_EN,
			.parent = DIVIDOR_GTC_DIV_WCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_DSI0_CLKEXT_EN] = {
			.name = LB_DSI0_CLKEXT_EN,
			.parent = MUX_DSI0_CLKEXT,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_DSI1_CLKEXT_EN] = {
			.name = LB_DSI1_CLKEXT_EN,
			.parent = MUX_DSI1_CLKEXT,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_DSI_CSITX_IPI_CLK_EN] = {
			.name = LB_DSI_CSITX_IPI_CLK_EN,
			.parent = MUX_DSI_CSITX_IPI,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_FLEXRAYNIC_TO_SYSNOC_GHCLK_EN] = {
			.name = FLEXRAYNIC_TO_SYSNOC_GHCLK_EN,
			.parent = MUX_FLEXRAYNIC_TO_SYSNOC_HCLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_SOC_DMA_CORE_CLK_EN] = {
			.name = LB_SOC_DMA_CORE_CLK_EN,
			.parent = MUX_SOC_DMA_CORE,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_MEDIANOC_ATBCLK_EN] = {
			.name = MEDIANOC_ATBCLK_EN,
			.parent = MUX_NOC_ATB_CLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_DBNOC_ATBCLK_EN] = {
			.name = DBNOC_ATBCLK_EN,
			.parent = MUX_NOC_ATB_CLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_GPUNOC_ATBCLK_EN] = {
			.name = GPUNOC_ATBCLK_EN,
			.parent = MUX_NOC_ATB_CLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_PCIENOC_ATBCLK_EN] = {
			.name = PCIENOC_ATBCLK_EN,
			.parent = MUX_NOC_ATB_CLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_CORENOC_ATBCLK_EN] = {
			.name = CORENOC_ATBCLK_EN,
			.parent = MUX_NOC_ATB_CLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_TOPISTCTRL_TO_TOPCRM_PCLK_EN] = {
			.name = TOPISTCTRL_TO_TOPCRM_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_USB_0_U20_PHY_REF_CLK_EN] = {
			.name = LB_USB_0_U20_PHY_REF_CLK_EN,
			.parent = MUX_USB_U20_PHY_REF_CLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_USB_1_U20_PHY_REF_CLK_EN] = {
			.name = LB_USB_1_U20_PHY_REF_CLK_EN,
			.parent = MUX_USB_U20_PHY_REF_CLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_USB_0_U31_PHY_REF_CLK_EN] = {
			.name = LB_USB_0_U31_PHY_REF_CLK_EN,
			.parent = MUX_USB_U31_PHY_REF_CLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_USB_1_U31_PHY_REF_CLK_EN] = {
			.name = LB_USB_1_U31_PHY_REF_CLK_EN,
			.parent = MUX_USB_U31_PHY_REF_CLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_SOC_LSP0_FLEXRAY_HCLK_EN] = {
			.name = LB_SOC_LSP0_FLEXRAY_HCLK_EN,
			.parent = DIVIDOR_SOC_LSP0_FLEXRAY_HCLK_NAME,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LB_SOC_LSP1_FLEXRAY_HCLK_EN] = {
			.name = LB_SOC_LSP1_FLEXRAY_HCLK_EN,
			.parent = DIVIDOR_SOC_LSP1_FLEXRAY_HCLK_NAME,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_CORENOC_APB_BR_PCLK_EN] = {
			.name = CORENOC_APB_BR_PCLK_EN,
			.parent = MUX_CLK_100_SYSBUS_APB,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_I2C0_SMBUS_WCLK_EN] = {
			.name = GATE_LSP0_I2C0_SMBUS_WCLK,
			.parent = MUX_LSP0_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_I2C0_SMBUS_PCLK_EN] = {
			.name = GATE_LSP0_I2C0_SMBUS_PCLK,
			.parent = LB_SOC_LSP0_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_I2C0_SMBUS_WCLK_EN] = {
			.name = GATE_LSP1_I2C0_SMBUS_WCLK,
			.parent = MUX_LSP1_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_I2C0_SMBUS_PCLK_EN] = {
			.name = GATE_LSP1_I2C0_SMBUS_PCLK,
			.parent = LB_SOC_LSP1_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_I2C1_SMBUS_WCLK_EN] = {
			.name = GATE_LSP0_I2C1_SMBUS_WCLK,
			.parent = MUX_LSP0_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_I2C1_SMBUS_PCLK_EN] = {
			.name = GATE_LSP0_I2C1_SMBUS_PCLK,
			.parent = LB_SOC_LSP0_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_I2C1_SMBUS_WCLK_EN] = {
			.name = GATE_LSP1_I2C1_SMBUS_WCLK,
			.parent = MUX_LSP1_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_I2C1_SMBUS_PCLK_EN] = {
			.name = GATE_LSP1_I2C1_SMBUS_PCLK,
			.parent = LB_SOC_LSP1_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_I2C0_WCLK_EN] = {
			.name = GATE_LSP0_I2C0_WCLK,
			.parent = MUX_LSP0_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_I2C0_PCLK_EN] = {
			.name = GATE_LSP0_I2C0_PCLK,
			.parent = LB_SOC_LSP0_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_I2C0_WCLK_EN] = {
			.name = GATE_LSP1_I2C0_WCLK,
			.parent = MUX_LSP1_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_I2C0_PCLK_EN] = {
			.name = GATE_LSP1_I2C0_PCLK,
			.parent = LB_SOC_LSP1_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_I2C1_WCLK_EN] = {
			.name = GATE_LSP0_I2C1_WCLK,
			.parent = MUX_LSP0_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_I2C1_PCLK_EN] = {
			.name = GATE_LSP0_I2C1_PCLK,
			.parent = LB_SOC_LSP0_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_I2C1_WCLK_EN] = {
			.name = GATE_LSP1_I2C1_WCLK,
			.parent = MUX_LSP1_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_I2C1_PCLK_EN] = {
			.name = GATE_LSP1_I2C1_PCLK,
			.parent = LB_SOC_LSP1_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_I2C2_WCLK_EN] = {
			.name = GATE_LSP0_I2C2_WCLK,
			.parent = MUX_LSP0_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_I2C2_PCLK_EN] = {
			.name = GATE_LSP0_I2C2_PCLK,
			.parent = LB_SOC_LSP0_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_I2C2_WCLK_EN] = {
			.name = GATE_LSP1_I2C2_WCLK,
			.parent = MUX_LSP1_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_I2C2_PCLK_EN] = {
			.name = GATE_LSP1_I2C2_PCLK,
			.parent = LB_SOC_LSP1_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_I2C3_WCLK_EN] = {
			.name = GATE_LSP0_I2C3_WCLK,
			.parent = MUX_LSP0_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_I2C3_PCLK_EN] = {
			.name = GATE_LSP0_I2C3_PCLK,
			.parent = LB_SOC_LSP0_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_I2C3_WCLK_EN] = {
			.name = GATE_LSP1_I2C3_WCLK,
			.parent = MUX_LSP1_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_I2C3_PCLK_EN] = {
			.name = GATE_LSP1_I2C3_PCLK,
			.parent = LB_SOC_LSP1_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_UART0_WCLK_EN] = {
			.name = GATE_LSP0_UART0_WCLK,
			.parent = MUX_LSP0_UART_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_UART0_PCLK_EN] = {
			.name = GATE_LSP0_UART0_PCLK,
			.parent = LB_SOC_LSP0_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_UART0_WCLK_EN] = {
			.name = GATE_LSP1_UART0_WCLK,
			.parent = MUX_LSP1_UART_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_UART0_PCLK_EN] = {
			.name = GATE_LSP1_UART0_PCLK,
			.parent = LB_SOC_LSP1_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_UART1_WCLK_EN] = {
			.name = GATE_LSP0_UART1_WCLK,
			.parent = MUX_LSP0_UART_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_UART1_PCLK_EN] = {
			.name = GATE_LSP0_UART1_PCLK,
			.parent = LB_SOC_LSP0_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_UART1_WCLK_EN] = {
			.name = GATE_LSP1_UART1_WCLK,
			.parent = MUX_LSP1_UART_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_UART1_PCLK_EN] = {
			.name = GATE_LSP1_UART1_PCLK,
			.parent = LB_SOC_LSP1_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},

	[GATE_LSP0_SSI_M_WCLK_EN] = {
			.name = GATE_LSP0_SSI_M_WCLK,
			.parent = MUX_LSP0_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_SSI_M_PCLK_EN] = {
			.name = GATE_LSP0_SSI_M_PCLK,
			.parent = LB_SOC_LSP0_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_SSI_M_WCLK_EN] = {
			.name = GATE_LSP1_SSI_M_WCLK,
			.parent = MUX_LSP1_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_SSI_M_PCLK_EN] = {
			.name = GATE_LSP1_SSI_M_PCLK,
			.parent = LB_SOC_LSP1_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_SSI_S_WCLK_EN] = {
			.name = GATE_LSP0_SSI_S_WCLK,
			.parent = MUX_LSP0_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_SSI_S_PCLK_EN] = {
			.name = GATE_LSP0_SSI_S_PCLK,
			.parent = LB_SOC_LSP0_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_SSI_S_WCLK_EN] = {
			.name = GATE_LSP1_SSI_S_WCLK,
			.parent = MUX_LSP1_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_SSI_S_PCLK_EN] = {
			.name = GATE_LSP1_SSI_S_PCLK,
			.parent = LB_SOC_LSP1_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	
	[GATE_LSP0_GPIO0_DBCLK_EN] = {
			.name = GATE_LSP0_GPIO0_DBCLK,
			.parent = DIVIDOR_LSP0_STA_DIV_GPIO_DBCLK_DIV,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_GPIO0_PCLK_EN] = {
			.name = GATE_LSP0_GPIO0_PCLK,
			.parent = LB_SOC_LSP0_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_GPIO1_DBCLK_EN] = {
			.name = GATE_LSP0_GPIO1_DBCLK,
			.parent = DIVIDOR_LSP0_STA_DIV_GPIO_DBCLK_DIV,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_GPIO1_PCLK_EN] = {
			.name = GATE_LSP0_GPIO1_PCLK,
			.parent = LB_SOC_LSP0_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_GPIO2_DBCLK_EN] = {
			.name = GATE_LSP0_GPIO2_DBCLK,
			.parent = DIVIDOR_LSP0_STA_DIV_GPIO_DBCLK_DIV,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_GPIO2_PCLK_EN] = {
			.name = GATE_LSP0_GPIO2_PCLK,
			.parent = LB_SOC_LSP0_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_GPIO3_DBCLK_EN] = {
			.name = GATE_LSP0_GPIO3_DBCLK,
			.parent = DIVIDOR_LSP0_STA_DIV_GPIO_DBCLK_DIV,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_GPIO3_PCLK_EN] = {
			.name = GATE_LSP0_GPIO3_PCLK,
			.parent = LB_SOC_LSP0_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_GPIO0_DBCLK_EN] = {
			.name = GATE_LSP1_GPIO0_DBCLK,
			.parent = DIVIDOR_LSP1_STA_DIV_GPIO_DBCLK_DIV,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_GPIO0_PCLK_EN] = {
			.name = GATE_LSP1_GPIO0_PCLK,
			.parent = LB_SOC_LSP1_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_GPIO1_DBCLK_EN] = {
			.name = GATE_LSP1_GPIO1_DBCLK,
			.parent = DIVIDOR_LSP1_STA_DIV_GPIO_DBCLK_DIV,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_GPIO1_PCLK_EN] = {
			.name = GATE_LSP1_GPIO1_PCLK,
			.parent = LB_SOC_LSP1_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_GPIO2_DBCLK_EN] = {
			.name = GATE_LSP1_GPIO2_DBCLK,
			.parent = DIVIDOR_LSP1_STA_DIV_GPIO_DBCLK_DIV,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_GPIO2_PCLK_EN] = {
			.name = GATE_LSP1_GPIO2_PCLK,
			.parent = LB_SOC_LSP1_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_GPIO3_DBCLK_EN] = {
			.name = GATE_LSP1_GPIO3_DBCLK,
			.parent = DIVIDOR_LSP1_STA_DIV_GPIO_DBCLK_DIV,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_GPIO3_PCLK_EN] = {
			.name = GATE_LSP1_GPIO3_PCLK,
			.parent = LB_SOC_LSP1_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_WDT0_WCLK_EN] = {
			.name = GATE_LSP0_WDT0_WCLK,
			.parent = MUX_LSP0_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_WDT0_PCLK_EN] = {
			.name = GATE_LSP0_WDT0_PCLK,
			.parent = LB_SOC_LSP0_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_WDT1_WCLK_EN] = {
			.name = GATE_LSP0_WDT1_WCLK,
			.parent = MUX_LSP0_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_WDT1_PCLK_EN] = {
			.name = GATE_LSP0_WDT1_PCLK,
			.parent = LB_SOC_LSP0_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_WDT0_WCLK_EN] = {
			.name = GATE_LSP1_WDT0_WCLK,
			.parent = MUX_LSP1_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_WDT0_PCLK_EN] = {
			.name = GATE_LSP1_WDT0_PCLK,
			.parent = LB_SOC_LSP1_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_WDT1_WCLK_EN] = {
			.name = GATE_LSP1_WDT1_WCLK,
			.parent = MUX_LSP1_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_WDT1_PCLK_EN] = {
			.name = GATE_LSP1_WDT1_PCLK,
			.parent = LB_SOC_LSP1_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_TIMER0_WCLK_EN] = {
			.name = GATE_LSP0_TIMER0_WCLK,
			.parent = MUX_LSP0_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_TIMER1_WCLK_EN] = {
			.name = GATE_LSP0_TIMER1_WCLK,
			.parent = MUX_LSP0_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_TIMER2_WCLK_EN] = {
			.name = GATE_LSP0_TIMER2_WCLK,
			.parent = MUX_LSP0_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_TIMER3_WCLK_EN] = {
			.name = GATE_LSP0_TIMER3_WCLK,
			.parent = MUX_LSP0_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_TIMER4_WCLK_EN] = {
			.name = GATE_LSP0_TIMER4_WCLK,
			.parent = MUX_LSP0_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_TIMER5_WCLK_EN] = {
			.name = GATE_LSP0_TIMER5_WCLK,
			.parent = MUX_LSP0_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_TIMER6_WCLK_EN] = {
			.name = GATE_LSP0_TIMER6_WCLK,
			.parent = MUX_LSP0_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_TIMER7_WCLK_EN] = {
			.name = GATE_LSP0_TIMER7_WCLK,
			.parent = MUX_LSP0_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_TIMER_PCLK_EN] = {
			.name = GATE_LSP0_TIMER_PCLK,
			.parent = LB_SOC_LSP0_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_TIMER0_WCLK_EN] = {
			.name = GATE_LSP1_TIMER0_WCLK,
			.parent = MUX_LSP1_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_TIMER1_WCLK_EN] = {
			.name = GATE_LSP1_TIMER1_WCLK,
			.parent = MUX_LSP1_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_TIMER2_WCLK_EN] = {
			.name = GATE_LSP1_TIMER2_WCLK,
			.parent = MUX_LSP1_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_TIMER3_WCLK_EN] = {
			.name = GATE_LSP1_TIMER3_WCLK,
			.parent = MUX_LSP1_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_TIMER4_WCLK_EN] = {
			.name = GATE_LSP1_TIMER4_WCLK,
			.parent = MUX_LSP1_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_TIMER5_WCLK_EN] = {
			.name = GATE_LSP1_TIMER5_WCLK,
			.parent = MUX_LSP1_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_TIMER6_WCLK_EN] = {
			.name = GATE_LSP1_TIMER6_WCLK,
			.parent = MUX_LSP1_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_TIMER7_WCLK_EN] = {
			.name = GATE_LSP1_TIMER7_WCLK,
			.parent = MUX_LSP1_LSP_WCLK_MUX,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_TIMER_PCLK_EN] = {
			.name = GATE_LSP1_TIMER_PCLK,
			.parent = LB_SOC_LSP1_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_I2S0_WCLK_EN] = {
			.name = GATE_LSP0_I2S0_WCLK,
			.parent = DIVIDOR_LSP0_I2S_M0_WCLK_DIV,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_I2S0_PCLK_EN] = {
			.name = GATE_LSP0_I2S0_PCLK,
			.parent = LB_SOC_LSP0_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_I2S1_WCLK_EN] = {
			.name = GATE_LSP0_I2S1_WCLK,
			.parent = DIVIDOR_LSP0_I2S_M1_WCLK_DIV,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_I2S1_PCLK_EN] = {
			.name = GATE_LSP0_I2S1_PCLK,
			.parent = LB_SOC_LSP0_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_I2S0_WCLK_EN] = {
			.name = GATE_LSP1_I2S0_WCLK,
			.parent = DIVIDOR_LSP1_I2S_M0_WCLK_DIV,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_I2S0_PCLK_EN] = {
			.name = GATE_LSP1_I2S0_PCLK,
			.parent = LB_SOC_LSP1_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_I2S1_WCLK_EN] = {
			.name = GATE_LSP1_I2S1_WCLK,
			.parent = DIVIDOR_LSP1_I2S_M1_WCLK_DIV,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_I2S1_PCLK_EN] = {
			.name = GATE_LSP1_I2S1_PCLK,
			.parent = LB_SOC_LSP1_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},

	[GATE_LSP0_I3C0_WCLK_EN] = {
			.name = GATE_LSP0_I3C0_WCLK,
			.parent = DIVIDOR_LSP0_STA_DIV_I3C_WCLK_DIV,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_I3C0_PCLK_EN] = {
			.name = GATE_LSP0_I3C0_PCLK,
			.parent = LB_SOC_LSP0_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_I3C1_WCLK_EN] = {
			.name = GATE_LSP0_I3C1_WCLK,
			.parent = DIVIDOR_LSP0_STA_DIV_I3C_WCLK_DIV,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_I3C1_PCLK_EN] = {
			.name = GATE_LSP0_I3C1_PCLK,
			.parent = LB_SOC_LSP0_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_I3C0_WCLK_EN] = {
			.name = GATE_LSP1_I3C0_WCLK,
			.parent = DIVIDOR_LSP1_STA_DIV_I3C_WCLK_DIV,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_I3C0_PCLK_EN] = {
			.name = GATE_LSP1_I3C0_PCLK,
			.parent = LB_SOC_LSP1_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_I3C1_WCLK_EN] = {
			.name = GATE_LSP1_I3C1_WCLK,
			.parent = DIVIDOR_LSP1_STA_DIV_I3C_WCLK_DIV,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_I3C1_PCLK_EN] = {
			.name = GATE_LSP1_I3C1_PCLK,
			.parent = LB_SOC_LSP1_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_TDM0_WCLK_EN] = {
			.name = GATE_LSP0_TDM0_WCLK,
			.parent = DIVIDOR_LSP0_TDM0_WCLK_DIV,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_TDM0_PCLK_EN] = {
			.name = GATE_LSP0_TDM0_PCLK,
			.parent = LB_SOC_LSP0_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_TDM1_WCLK_EN] = {
			.name = GATE_LSP0_TDM1_WCLK,
			.parent = DIVIDOR_LSP0_TDM1_WCLK_DIV,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_TDM1_PCLK_EN] = {
			.name = GATE_LSP0_TDM1_PCLK,
			.parent = LB_SOC_LSP0_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_TDM0_WCLK_EN] = {
			.name = GATE_LSP1_TDM0_WCLK,
			.parent = DIVIDOR_LSP1_TDM0_WCLK_DIV,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_TDM0_PCLK_EN] = {
			.name = GATE_LSP1_TDM0_PCLK,
			.parent = LB_SOC_LSP1_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_TDM1_WCLK_EN] = {
			.name = GATE_LSP1_TDM1_WCLK,
			.parent = DIVIDOR_LSP1_TDM1_WCLK_DIV,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_TDM1_PCLK_EN] = {
			.name = GATE_LSP1_TDM1_PCLK,
			.parent = LB_SOC_LSP1_APB_S0_PCLK_EN,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_SPDIF0_WCLK_EN] = {
			.name = GATE_LSP0_SPDIF0_WCLK,
			.parent = DIVIDOR_LSP0_SPDIF0_WCLK_DIV,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP0_SPDIF1_WCLK_EN] = {
			.name = GATE_LSP0_SPDIF1_WCLK,
			.parent = DIVIDOR_LSP0_SPDIF1_WCLK_DIV,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_SPDIF0_WCLK_EN] = {
			.name = GATE_LSP1_SPDIF0_WCLK,
			.parent = DIVIDOR_LSP1_SPDIF0_WCLK_DIV,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_LSP1_SPDIF1_WCLK_EN] = {
		.name = GATE_LSP1_SPDIF1_WCLK,
		.parent = DIVIDOR_LSP1_SPDIF1_WCLK_DIV,
		.type = CLOCK_TREE_GAT,
	},
	[GATE_XGMAC_PTP_CLK_EN] = {
			.name = GATE_XGMAC_PTP_CLK_EN_NAME,
			.parent = LB_SW_XGMAC_PTP_CLK_NAME,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_XGMAC_WCLK_EN] = {
		.name = GATE_XGMAC_WCLK_EN_NAME,
		.parent = LB_SW_XGMAC_WCLK_NAME,
		.type = CLOCK_TREE_GAT,
	},
	[GATE_CPU0_MP4_DSU_CHI_EN] = {
			.name = GATE_CPU0_MP4_DSU_CHI_NAME,
			.parent = MUX_CPU0_MP4_DSU_CHI_CLK,
			.type = CLOCK_TREE_GAT,
	},
	[GATE_CPU0_MP4_CORE_EN] = {
		.name = GATE_CPU0_MP4_CORE_NAME,
		.parent = MUX_CPU0_MP4_CORE_CLK,
		.type = CLOCK_TREE_GAT,
	}
};



enum scmi_clock_protocol_cmd {
	CLOCK_ATTRIBUTES = 0x3,
	CLOCK_DESCRIBE_RATES = 0x4,
	CLOCK_RATE_SET = 0x5,
	CLOCK_RATE_GET = 0x6,
	CLOCK_CONFIG_SET = 0x7,
	CLOCK_CONFIG_GET = 0x8,
	CLOCK_PARENT_SET = 0x9,
	CLOCK_DIVIDER_SET = 0xA,
};

struct scmi_msg_resp_clock_protocol_attributes {
	__le16 num_clocks;
	u8 max_async_req;
	u8 reserved;
};

struct scmi_msg_resp_clock_attributes {
	__le32 attributes;
	__le32 parent_index[9];
	__le32 parent_num;
	__le32 clk_type;

#define	CLOCK_ENABLE	BIT(0)

#define	CLOCK_DISABLE	BIT(1)
	    u8 name[48];
};


struct scmi_clock_set_config {
	__le32 id;
	__le32 attributes;
	__le32 sid;
};



struct scmi_clock_set_divider {
	__le32 id;
	__le32 divider;
};



struct scmi_clock_get_config {
	__le32 id;
	__le32 sid;
};


struct scmi_msg_clock_describe_rates {
	__le32 id;
	__le32 rate_index;
};


struct scmi_clock_set_parent_s {
	__le32 id;
	__le32 index;
};


struct scmi_msg_resp_clock_describe_rates {
	__le32 num_rates_flags;
#define NUM_RETURNED(x)		((x) & 0xfff)
#define RATE_DISCRETE(x)	!((x) & BIT(12))
#define NUM_REMAINING(x)	((x) >> 16)
	struct {
		__le32 value_low;
		__le32 value_high;
	} rate[3];
#define RATE_TO_U64(X)		\
({				\
	typeof(X) x = (X);	\
	le32_to_cpu((x).value_low) | (u64)le32_to_cpu((x).value_high) << 32; \
})
};

struct scmi_clock_set_rate {
	__le32 flags;
#define CLOCK_SET_ASYNC		BIT(0)
#define CLOCK_SET_IGNORE_RESP	BIT(1)
#define CLOCK_SET_ROUND_UP	BIT(2)
#define CLOCK_SET_ROUND_AUTO	BIT(3)
	__le32 id;
	__le32 value_low;
	__le32 value_high;
};

struct clock_info {
	u32 version;
	int num_clocks;
	int max_async_req;
	atomic_t cur_async_req;
	struct scmi_clock_info *clk;
};



//DEFINE_MUTEX(scmi_mutex);



static int
scmi_clock_protocol_attributes_get(const struct scmi_protocol_handle *ph,
				   struct clock_info *ci)
{
	int ret,count = 5;
	struct scmi_xfer *t;
	struct scmi_msg_resp_clock_protocol_attributes *attr;

	ret = ph->xops->xfer_get_init(ph, PROTOCOL_ATTRIBUTES,
				      0, sizeof(*attr), &t);
	if (ret)
		return ret;

	attr = t->rx.buf;

	while(count--){
		ret = ph->xops->do_xfer(ph, t);
		if (!ret) {
			ci->num_clocks = le16_to_cpu(attr->num_clocks);
			ci->max_async_req = attr->max_async_req;
			break;
		}else{
			//printk("%s %d timeout:count:%d",__func__,__LINE__,count);
			continue;
		}
	}

	if(count == 0){
		dev_err(ph->dev, "%s ivoke error",__func__);
	}


	ph->xops->xfer_put(ph, t);
	return ret;
}

static int scmi_clock_attributes_get(const struct scmi_protocol_handle *ph,
				     u32 clk_id, struct scmi_msg_resp_clock_attributes * mattr)
{
	int ret,count=5;
	struct scmi_xfer *t;
	struct scmi_msg_resp_clock_attributes *attr;

	ret = ph->xops->xfer_get_init(ph, CLOCK_ATTRIBUTES,
				      sizeof(clk_id), sizeof(*attr), &t);
	if (ret)
		return ret;

	put_unaligned_le32(clk_id, t->tx.buf);
	attr = t->rx.buf;

	//mutex_lock(&scmi_mutex);
	while(count--){
		ret = ph->xops->do_xfer(ph, t);
		//mutex_unlock(&scmi_mutex);
		if (!ret){
			memcpy(mattr, attr,sizeof(struct scmi_msg_resp_clock_attributes));
			break;
		}
		else{
			continue;
		}
	}

	if(count == 0){
		mattr->attributes = 0;
		memset(mattr,0,sizeof(struct scmi_msg_resp_clock_attributes));
		dev_err(ph->dev, "%s ivoke error",__func__);
	}


	ph->xops->xfer_put(ph, t);
	return ret;
}

// static int rate_cmp_func(const void *_r1, const void *_r2)
// {
// 	const u64 *r1 = _r1, *r2 = _r2;

// 	if (*r1 < *r2)
// 		return -1;
// 	else if (*r1 == *r2)
// 		return 0;
// 	else
// 		return 1;
// }




static int
scmi_clock_describe_rates_get(const struct scmi_protocol_handle *ph,struct scmi_clock_info *clk)
{
	int ret,clk_id = 0,count = 5;
	bool rate_discrete = false;
	u32 tot_rate_cnt = 0;

	struct scmi_xfer *t;
	struct scmi_msg_clock_describe_rates *clk_desc;
	struct scmi_msg_resp_clock_describe_rates *rlist;


	ret = ph->xops->xfer_get_init(ph, CLOCK_DESCRIBE_RATES,
				      sizeof(*clk_desc), 0, &t);
	if (ret)
		return ret;

	clk_desc = t->tx.buf;
	rlist = t->rx.buf;

	clk_desc->id = cpu_to_le32(clk_id);
	/* Set the number of rates to be skipped/already read */
	clk_desc->rate_index = cpu_to_le32(tot_rate_cnt);


	while(count--){
		ret = ph->xops->do_xfer(ph, t);
		if(ret){
			continue;
		}
		else
			break;
	}

	if(count == 0){
		dev_err(ph->dev, "%s ivoke error",__func__);
		goto err;
	}

	clk->range.min_rate = RATE_TO_U64(rlist->rate[0]);
	clk->range.max_rate = RATE_TO_U64(rlist->rate[1]);
	clk->range.step_size = RATE_TO_U64(rlist->rate[2]);
	clk->rate_discrete = rate_discrete;

err:
	ph->xops->xfer_put(ph, t);
	return ret;
}





static int
scmi_clock_rate_get(const struct scmi_protocol_handle *ph,
		    u32 clk_id, u64 *value)
{
	int ret,count = 5;
	struct scmi_xfer *t;


	ret = ph->xops->xfer_get_init(ph, CLOCK_RATE_GET,
				      sizeof(__le32), sizeof(u64), &t);
	if (ret)
		return ret;

	put_unaligned_le32(clk_id, t->tx.buf);

	while(count--){
		ret = ph->xops->do_xfer(ph, t);
		
		if (!ret){
			*value = get_unaligned_le64(t->rx.buf);
			break;
		}
		else {
			continue;
		}
	}

	if(count == 0){
		dev_err(ph->dev, "%s ivoke error",__func__);
	}

	
	ph->xops->xfer_put(ph, t);
	return ret;
}

static int scmi_clock_rate_set(const struct scmi_protocol_handle *ph,
			       u32 clk_id, u64 rate)
{
	int ret;
	u32 flags = 0;
	struct scmi_xfer *t;
	struct scmi_clock_set_rate *cfg;
	struct clock_info *ci = ph->get_priv(ph);

	ret = ph->xops->xfer_get_init(ph, CLOCK_RATE_SET, sizeof(*cfg), 0, &t);
	if (ret)
		return ret;

	if (ci->max_async_req &&
	    atomic_inc_return(&ci->cur_async_req) < ci->max_async_req)
		flags |= CLOCK_SET_ASYNC;

	cfg = t->tx.buf;
	cfg->flags = cpu_to_le32(flags);
	cfg->id = cpu_to_le32(clk_id);
	cfg->value_low = cpu_to_le32(rate & 0xffffffff);
	cfg->value_high = cpu_to_le32(rate >> 32);


	ret = ph->xops->do_xfer(ph, t);

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int
scmi_clock_config_set(const struct scmi_protocol_handle *ph, u32 clk_id,
		      u32 config)
{
	int ret,count = 2;
	struct scmi_xfer *t;
	struct scmi_clock_set_config *cfg;

	#ifdef CONFIG_BST_C1200_IVI
	uint32_t sid = 0;
	#endif

	
	#ifdef CONFIG_BST_C1200_ADAS
	uint32_t sid = 1;
	#endif

	

	#ifdef CONFIG_BST_C1200_DB
	uint32_t sid = 2;
	#endif



	ret = ph->xops->xfer_get_init(ph, CLOCK_CONFIG_SET,
				      sizeof(*cfg), 0, &t);
	if (ret)
		return ret;

	cfg = t->tx.buf;
	cfg->id = cpu_to_le32(clk_id);
	cfg->attributes = cpu_to_le32(config);
	cfg->sid = cpu_to_le32(sid);

	while(count--){
		ret = ph->xops->do_xfer(ph, t);
		if(!ret){
			break;
		}else{
			continue;
		}
	}

	if(count == 0){
		dev_err(ph->dev, "%s ivoke error",__func__);
	}


	ph->xops->xfer_put(ph, t);

	return ret;
}







static int
scmi_clock_config_get(const struct scmi_protocol_handle *ph, u32 clk_id)
{
	int ret,count=5;
	int is_enable = 0;
	unsigned char * p;

	struct scmi_xfer *t;
	struct scmi_clock_get_config *cfg;


	#ifdef CONFIG_BST_C1200_IVI
	uint32_t sid = 0;
	#endif

	
	#ifdef CONFIG_BST_C1200_ADAS
	uint32_t sid = 1;
	#endif

	

	#ifdef CONFIG_BST_C1200_DB
	uint32_t sid = 2;
	#endif


	ret = ph->xops->xfer_get_init(ph, CLOCK_CONFIG_GET,
				      sizeof(*cfg), 0, &t);
	if (ret)
		return ret;

	cfg = t->tx.buf;
	cfg->id = cpu_to_le32(clk_id);
	cfg->sid = cpu_to_le32(sid);

	while(count--){
		ret = ph->xops->do_xfer(ph, t);
		if (!ret){
			p = (char *)t->rx.buf;
			is_enable = p[0];
			break;
		}else{
			continue;
		}
	}

	if(count == 0){
		dev_err(ph->dev, "%s ivoke error",__func__);
	}



	ph->xops->xfer_put(ph, t);
	return is_enable;
}





static int scmi_clock_enable(const struct scmi_protocol_handle *ph, u32 clk_id)
{
	//printk("clk name:%s enable",g_clk_info[clk_id].name);
	scmi_clock_config_set(ph, clk_id, 1);
	return 0;
}



static int scmi_clock_isenable(const struct scmi_protocol_handle *ph, u32 clk_id)
{
	return scmi_clock_config_get(ph, clk_id);
}



static int scmi_clock_disable(const struct scmi_protocol_handle *ph, u32 clk_id)
{
	scmi_clock_config_set(ph, clk_id, 0);
	return 0;
}

static int scmi_clock_count_get(const struct scmi_protocol_handle *ph)
{
	struct clock_info *ci = ph->get_priv(ph);

	return ci->num_clocks;
}

static const struct scmi_clock_info *
scmi_clock_info_get(const struct scmi_protocol_handle *ph, u32 clk_id)
{
	struct clock_info *ci = ph->get_priv(ph);
	struct scmi_clock_info *clk = ci->clk + clk_id;

	if (!clk)
		return NULL;

	return clk;
}


static int scmi_clock_get_parent(const struct scmi_protocol_handle *ph, u32 clk_id)
{
	struct clock_info *ci = ph->get_priv(ph);
	struct scmi_clock_info *clk = ci->clk + clk_id;

	if (!clk)
		return 0;

	return clk->mux_index;
}



static int scmi_clock_set_parent(const struct scmi_protocol_handle *ph, u32 clk_id,u8 index)
{
	int ret,count = 5;
	struct scmi_xfer *t;
	struct scmi_clock_set_parent_s *cfg;



	ret = ph->xops->xfer_get_init(ph, CLOCK_PARENT_SET,
				      sizeof(*cfg), 0, &t);
	if (ret)
		return ret;

	cfg = t->tx.buf;
	cfg->id = cpu_to_le32(clk_id);
	cfg->index = cpu_to_le32(index);

	while(count--){
		ret = ph->xops->do_xfer(ph, t);
		if(!ret){
			break;
		}else{
			continue;
		}
	}

	if(count == 0){
		dev_err(ph->dev, "%s ivoke error",__func__);
	}


	ph->xops->xfer_put(ph, t);


	return 0;
}


static int scmi_has_child(const struct scmi_protocol_handle *ph, u32 clk_id){
	int i = 0,j = 0;
	int isFound = 0;

	for(i=0;i<CLK_MAX;i++){

		switch(g_clk_info[i].type){

			case CLOCK_TREE_PLL:{
				if(strcmp(g_clk_info[clk_id].name,g_clk_info[i].parent) == 0){
					isFound = 1;
				}
				break;
			}
			case CLOCK_TREE_MUX:{
				for(j = 0;j < g_clk_info[i].mux_parent_count;  j++){

					if(strcmp(g_clk_info[clk_id].name,g_clk_info[i].mux_parent[j]) == 0){
						isFound = 1;
						break;
					}
				}
				break;
			}
			case CLOCK_TREE_FIX:{
				break;
			}
			case CLOCK_TREE_GAT:{
				if(strcmp(g_clk_info[clk_id].name,g_clk_info[i].parent) == 0){
					isFound = 1;
				}
				break;
			}
			case CLOCK_TREE_FCT:{
				if(strcmp(g_clk_info[clk_id].name,g_clk_info[i].parent) == 0){
					isFound = 1;
				}
				break;
			}
			case CLOCK_TREE_DIV:{
				if(strcmp(g_clk_info[clk_id].name,g_clk_info[i].parent) == 0){
					isFound = 1;
				}
				break;
			}
		}

		if(isFound == 1){
			break;
		}
	}


	return isFound;
}







#define ULONG_MAX	(~0UL)
#define my_min(x,y) (x > y ? y : x)

static void best_approximation(
	u64 given_numerator, u64 given_denominator,
	u64 max_numerator, u64 max_denominator,
	u64 *best_numerator, u64 *best_denominator)
{
	u64 n, d, n0, d0, n1, d1, n2, d2;
	n = given_numerator;
	d = given_denominator;
	n0 = d1 = 0;
	n1 = d0 = 1;

	for (;;) {
		u64 dp, a;

		if (d == 0)
			break;

		dp = d;
		a = n / d;
		d = n % d;
		n = dp;

		n2 = n0 + a * n1;
		d2 = d0 + a * d1;

		if ((n2 > max_numerator) || (d2 > max_denominator)) {
			u64 t = ULONG_MAX;

			if (d1)
				t = (max_denominator - d0) / d1;
			if (n1)
				t = my_min(t, (max_numerator - n0) / n1);

			if (!d1 || 2u * t > a || (2u * t == a && d0 * dp > d1 * d)) {
				n1 = n0 + t * n1;
				d1 = d0 + t * d1;
			}
			break;
		}
		n0 = n1;
		n1 = n2;
		d0 = d1;
		d1 = d2;
	}
	*best_numerator = n1;
	*best_denominator = d1;
}

static unsigned int get_display_div(u64 parent_rate,u64 rate){
	unsigned int val;
	u64 zhengshu = 0;
	u64  denominator = 0;
	u64  numerator =0;
	u64 remainder =0 ;

	best_approximation(parent_rate,rate,0xFFF,0xFFF,&numerator,&denominator);

	zhengshu =  numerator/denominator;
	remainder = numerator%denominator;

	val = (zhengshu&0xff) << 24 | (remainder&0xfff)<<12 | (denominator&0xfff);

	return val;
 }



int scmi_dividor_set(const struct scmi_protocol_handle *ph, u32 clk_id,unsigned long rate){

	struct scmi_xfer *t;
	struct scmi_clock_set_divider *cfg;
	int ret,i;
	unsigned int val;
	u64 prate;


	if(g_clk_info[clk_id].type != CLOCK_TREE_DIV){
		pr_err("scmi type:%d %d error !!!\n",clk_id,g_clk_info[clk_id].type);
		return -1;
	}	

	for(i=0;i<CLK_MAX;i++) {
		if(strcmp(g_clk_info[clk_id].parent,g_clk_info[i].name) == 0){
			break;
		}
	}

	scmi_clock_rate_get(ph,i,&prate);


	val = get_display_div(prate,rate);

	ret = ph->xops->xfer_get_init(ph, CLOCK_DIVIDER_SET,
						sizeof(*cfg), 0, &t);
	if (ret)
		return ret;

	cfg = t->tx.buf;
	cfg->id = cpu_to_le32(clk_id);
	cfg->divider = cpu_to_le32(val);


	ret = ph->xops->do_xfer(ph, t);
	ph->xops->xfer_put(ph, t);
	return ret;
}


static const struct scmi_clk_proto_ops clk_proto_ops = {
	.count_get = scmi_clock_count_get,
	.info_get = scmi_clock_info_get,
	.rate_get = scmi_clock_rate_get,
	.rate_set = scmi_clock_rate_set,
	.enable = scmi_clock_enable,
	.is_enable = scmi_clock_isenable,
	.disable = scmi_clock_disable,
	.get_parent= scmi_clock_get_parent,
	.set_parent = scmi_clock_set_parent,
	.has_child = scmi_has_child,
	.dividor_set = scmi_dividor_set,
};

int get_pid_by_name(const char * name){
	int i = 0;

	for(i=0;i<CLK_MAX;i++){
		if(strcmp(g_clk_info[i].name,name) == 0){
			break;
		}
	}

	return i;
}

static int get_clock_check_pid_valid(int clock_id,struct scmi_msg_resp_clock_attributes *clk){
	int i = 0;
	int pid = 0;

	if(g_clk_info[clock_id].type != clk->clk_type){

		pr_err("scmi mux_parent type:%d %d %d GATE_LSP1_TIMER_PCLK_EN:%d\n",clock_id,g_clk_info[clock_id].type,clk->clk_type,GATE_LSP1_TIMER_PCLK_EN);
		return -1;
	}	

	switch(clk->clk_type){

		case CLOCK_TREE_PLL:{
			pid = get_pid_by_name(g_clk_info[clock_id].parent);
			if(pid == CLK_MAX || pid != clk->parent_index[0]){
				return -1;
			}
			break;
		}
		case CLOCK_TREE_FIX:{
		
			break;
		}
		case CLOCK_TREE_MUX:{
			for(i=0;i<g_clk_info[clock_id].mux_parent_count;i++){

				pid = get_pid_by_name(g_clk_info[clock_id].mux_parent[i]);

				if(pid == CLK_MAX || pid != clk->parent_index[i]){

					pr_err("scmi mux_parent %s pid:%d %d\n",g_clk_info[clock_id].mux_parent[i],pid,clk->parent_index[i]);

					return -1;
				}
			}

			break;
		}
		case CLOCK_TREE_GAT:{
			pid = get_pid_by_name(g_clk_info[clock_id].parent);
			if(pid == CLK_MAX || pid != clk->parent_index[0]){
				printk("scmi pid:%d\n",pid);
				printk("scmi clk->parent_index[0]:%d\n",clk->parent_index[0]);
				return -1;
			}
			break;
		}
		case CLOCK_TREE_FCT:{
			pid = get_pid_by_name(g_clk_info[clock_id].parent);
			if(pid == CLK_MAX || pid != clk->parent_index[0]){
				return -1;
			}
			break;
		}
		case CLOCK_TREE_DIV:{
			pid = get_pid_by_name(g_clk_info[clock_id].parent);
			if(pid == CLK_MAX || pid != clk->parent_index[0]){
				return -1;
			}
			break;
		}
	}

	return 0;
}



//#define CLOCK_SCMI_STRESS_TEST

#ifdef CLOCK_SCMI_STRESS_TEST

typedef struct private_data_{
	const struct scmi_protocol_handle *ph;
}private_data_s;



static int multi_cpu_scmi_task(void *data)
{
	
	private_data_s * ptr = data;
	const struct scmi_protocol_handle *ph = ptr->ph;

	struct clock_info *cinfo;
	struct scmi_msg_resp_clock_attributes *mattr;
	int clkid,ret;
	int count=0;

	
	cinfo = devm_kzalloc(ph->dev, sizeof(*cinfo), GFP_KERNEL);
	if (!cinfo)
		return -ENOMEM;

	mattr = devm_kzalloc(ph->dev, sizeof(*mattr), GFP_KERNEL);
	if (!mattr)
		return -ENOMEM;

	scmi_clock_protocol_attributes_get(ph, cinfo);

	if(cinfo->num_clocks != CLK_MAX){
		dev_info(ph->dev, "Safety Clk Num:%d  Linux Clk Num:%d failed\n",cinfo->num_clocks , CLK_MAX);
		return -EBADF;
	}


	cinfo->clk = devm_kcalloc(ph->dev, cinfo->num_clocks,
				  sizeof(*cinfo->clk), GFP_KERNEL);
	if (!cinfo->clk)
		return -ENOMEM;


	while(1){

		for (clkid = 0; clkid < cinfo->num_clocks; clkid++){
			struct scmi_clock_info *clk = (struct scmi_clock_info *)(cinfo->clk + clkid);
			strlcpy(clk->name,g_clk_info[clkid].name,SCMI_MAX_STR_SIZE);
		

			ret = scmi_clock_attributes_get(ph,clkid,mattr);
			if(get_clock_check_pid_valid(clkid,mattr) == -1){
				dev_info(ph->dev, "Clock Linux and Safety is different:%s\n",clk->name);
				return -EBADF;
			}
		}


		dev_info(ph->dev, "scmi init count:%d\n",count++);

		usleep_range(1000000, 1000000);
	}


	return 0;
}
#endif

static int scmi_clock_protocol_init(const struct scmi_protocol_handle *ph)
{
	u32 version;
	int clkid, ret,i;
	struct clock_info *cinfo;
	
	struct scmi_msg_resp_clock_attributes *mattr;
	
	ph->xops->version_get(ph, &version);
	

	dev_info(ph->dev, "Clock Version %d.%d\n",
		PROTOCOL_REV_MAJOR(version), PROTOCOL_REV_MINOR(version));



	cinfo = devm_kzalloc(ph->dev, sizeof(*cinfo), GFP_KERNEL);
	if (!cinfo)
		return -ENOMEM;

	
	mattr = devm_kzalloc(ph->dev, sizeof(*mattr), GFP_KERNEL);
	if (!mattr)
		return -ENOMEM;

	scmi_clock_protocol_attributes_get(ph, cinfo);

	if(cinfo->num_clocks != CLK_MAX){
		dev_info(ph->dev, "Safety Clk Num:%d  Linux Clk Num:%d failed\n",cinfo->num_clocks , CLK_MAX);
		return -EBADF;
	}


	cinfo->clk = devm_kcalloc(ph->dev, cinfo->num_clocks,
				  sizeof(*cinfo->clk), GFP_KERNEL);
	if (!cinfo->clk)
		return -ENOMEM;


	

	for (clkid = 0; clkid < cinfo->num_clocks; clkid++) {



		struct scmi_clock_info *clk = (struct scmi_clock_info *)(cinfo->clk + clkid);
		strlcpy(clk->name,g_clk_info[clkid].name,SCMI_MAX_STR_SIZE);


		ret = scmi_clock_attributes_get(ph,clkid,mattr);
		if(get_clock_check_pid_valid(clkid,mattr) == -1){
			dev_info(ph->dev, "Clock Linux and Safety is different:%s\n",clk->name);
			return -EBADF;
		}

		clk->type = (enum clock_type)g_clk_info[clkid].type;


		switch(clk->type){

			case CLOCK_TREE_PLL:{
					clk->parent_num =1;
					clk->parent_index[0] = g_clk_info[clkid].parent;
					break;
			}
			case CLOCK_TREE_FIX:{
					clk->parent_num = 0;
					clk->fix_rate = g_clk_info[clkid].fix.rate;
					break;
			}
			case CLOCK_TREE_MUX:{

				clk->parent_num = g_clk_info[clkid].mux_parent_count;

				for(i=0;i<clk->parent_num;i++){

						clk->parent_index[i] = g_clk_info[clkid].mux_parent[i];
				}
				clk->mux_index = mattr->parent_index[i++];
				break;
			}
			case CLOCK_TREE_GAT:{
					clk->parent_num = 1;
					clk->parent_index[0] = g_clk_info[clkid].parent;
					break;
			}
			case CLOCK_TREE_FCT:{
					clk->parent_num = 1;
					clk->parent_index[0] =  g_clk_info[clkid].parent;

					clk->fix_factor_mult = g_clk_info[clkid].factor.mult;
					clk->fix_factor_div = g_clk_info[clkid].factor.div;
					break;
			}
			case CLOCK_TREE_DIV:{
					clk->parent_num = 1;
					clk->parent_index[0] = g_clk_info[clkid].parent;
					break;
			}
		}

		scmi_clock_describe_rates_get(ph,clk);
	}

	devm_kfree(ph->dev, mattr);
	cinfo->version = version;

#ifdef CLOCK_SCMI_STRESS_TEST
	{
		private_data_s * ptr;
			
		ptr = devm_kzalloc(ph->dev, sizeof(*ptr), GFP_KERNEL);
		if (!ptr)
			return -ENOMEM;
		ptr->ph = ph;
		kthread_run(multi_cpu_scmi_task,ptr, "scmi");
	}
#endif

	return ph->set_priv(ph, cinfo);
}

static const struct scmi_protocol scmi_clock = {
	.id = SCMI_PROTOCOL_CLOCK,
	.owner = THIS_MODULE,
	.instance_init = &scmi_clock_protocol_init,
	.ops = &clk_proto_ops,
	.events = NULL,
};

DEFINE_SCMI_PROTOCOL_REGISTER_UNREGISTER(clock, scmi_clock)
MODULE_AUTHOR("BST Ltd.");
