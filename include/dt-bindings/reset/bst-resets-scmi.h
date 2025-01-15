/* SPDX-License-Identifier: GPL-2.0 */
/* reset driver for BST C1200
 * This file contains proprietary information that is the sole intellectual
 * property of Black Sesame Technologies, Inc. and its affiliates.
 * No portions of this material may be reproduced in any
 * form without the written permission of:
 * Black Sesame Technologies, Inc. and its affiliates
 * 2255 Martin Ave. Suite D
 * Santa Clara, CA 95050
 * Copyright @2016: all right reserved.
 */

#ifndef _DT_BINDINGS_BST_C1200_RESETS_H_
#define _DT_BINDINGS_BST_C1200_RESETS_H_

  
//top block
#define RST_CMN_FMU_SW              0
#define RST_XGMAC_SW                1
#define RST_CLK_MONITOR_SW          2
#define RST_CPU0_MP4_SW             3
#define RST_CPU1_MP4_SW             4
#define RST_CPU_PERIP_SW            5          
#define RST_EDP_SW                  6
#define RST_HIFI_DSP_SW             7
#define RST_MEDIA_DMA_SW            8
#define RST_MIPI_DSI_CSITX_SW       9

#define RST_MATRIX_SW               10
#define RST_SOC_DMA_SW              11
#define RST_PCIE_SW                 12
#define RST_UFS_SW                  13
#define RST_PLL_SW                  14
#define RST_CPU_MP2_SW              15
#define RST_GPU_G78AE_SP_SW         16
#define RST_DB_DMA_SW               17
#define RST_ISP_SW                  18
#define RST_MIPI0_CSIRX_SW          19
#define RST_MIPI1_CSIRX_SW          20
#define RST_MIPI2_CSIRX_SW          21
#define RST_CV_SW                   22
#define RST_NET_SW                  23
#define RST_CS_DMA_SW               24
#define RST_GPU_G78AE_SW            25
#define RST_DISPLAY0_SW             26
#define RST_DISPLAY1_SW             27
#define RST_DISPLAY2_SW             28
#define RST_CODEC0_SW               29
#define RST_CODEC1_SW               30
#define RST_LVDS_0_SW               31
#define RST_JIAYU_SW                32
#define RST_USB3_0_SW               33
#define RST_USB3_1_SW               34
#define RST_SDEMMC0_SW              35
#define RST_SDEMMC1_SW              36
#define RST_SOC_LSP0_SW             37
#define RST_SOC_LSP1_SW             38
#define RST_LPDDR5_0_SW             39
#define RST_LPDDR5_1_SW             40

//lsp0

#define RST_LSP0_I2C0_SMBUS_WCLK_SW  41
#define RST_LSP0_I2C1_SMBUS_WCLK_SW  42
#define RST_LSP0_I2C0_WCLK_SW        43
#define RST_LSP0_I2C1_WCLK_SW        44
#define RST_LSP0_I2C2_WCLK_SW        45
#define RST_LSP0_I2C3_WCLK_SW        46
#define RST_LSP0_SSI_M_WCLK_SW       47
#define RST_LSP0_UART0_WCLK_SW       48
#define RST_LSP0_UART1_WCLK_SW       49
#define RST_LSP0_WDT0_WCLK_SW        50
#define RST_LSP0_WDT1_WCLK_SW        51
#define RST_LSP0_GPIO0_DBCLK_SW      52
#define RST_LSP0_GPIO1_DBCLK_SW      53
#define RST_LSP0_GPIO2_DBCLK_SW      54
#define RST_LSP0_GPIO3_DBCLK_SW      55
#define RST_LSP0_I3C0_WCLK_SW        56
#define RST_LSP0_I3C1_WCLK_SW        57
#define RST_LSP0_XRAY_WCLK_SW        58
#define RST_LSP0_I2S_M0_WCLK_SW      59
#define RST_LSP0_I2S_M1_WCLK_SW      60
#define RST_LSP0_TIMER0_WCLK_SW      61
#define RST_LSP0_TIMER1_WCLK_SW      62
#define RST_LSP0_TIMER2_WCLK_SW      63
#define RST_LSP0_TIMER3_WCLK_SW      64
#define RST_LSP0_TIMER4_WCLK_SW      65
#define RST_LSP0_TIMER5_WCLK_SW      66
#define RST_LSP0_TIMER6_WCLK_SW      67
#define RST_LSP0_TIMER7_WCLK_SW      68
#define RST_LSP0_SSI_S_WCLK_SW       69
#define RST_LSP0_SPDIF0_WCLK_SW      70
#define RST_LSP0_SPDIF1_WCLK_SW      71


//lsp1
#define RST_LSP1_I2C0_SMBUS_WCLK_SW  72
#define RST_LSP1_I2C1_SMBUS_WCLK_SW  73
#define RST_LSP1_I2C0_WCLK_SW        74
#define RST_LSP1_I2C1_WCLK_SW        75
#define RST_LSP1_I2C2_WCLK_SW        76
#define RST_LSP1_I2C3_WCLK_SW        77
#define RST_LSP1_SSI_M_WCLK_SW       78
#define RST_LSP1_UART0_WCLK_SW       79
#define RST_LSP1_UART1_WCLK_SW       80
#define RST_LSP1_WDT0_WCLK_SW        81
#define RST_LSP1_WDT1_WCLK_SW        82
#define RST_LSP1_GPIO0_DBCLK_SW      83
#define RST_LSP1_GPIO1_DBCLK_SW      84
#define RST_LSP1_GPIO2_DBCLK_SW      85
#define RST_LSP1_GPIO3_DBCLK_SW      86
#define RST_LSP1_I3C0_WCLK_SW        87
#define RST_LSP1_I3C1_WCLK_SW        88
#define RST_LSP1_XRAY_WCLK_SW        89
#define RST_LSP1_I2S_M0_WCLK_SW      90
#define RST_LSP1_I2S_M1_WCLK_SW      91
#define RST_LSP1_TIMER0_WCLK_SW      92
#define RST_LSP1_TIMER1_WCLK_SW      93
#define RST_LSP1_TIMER2_WCLK_SW      94
#define RST_LSP1_TIMER3_WCLK_SW      95
#define RST_LSP1_TIMER4_WCLK_SW      96
#define RST_LSP1_TIMER5_WCLK_SW      97
#define RST_LSP1_TIMER6_WCLK_SW      98
#define RST_LSP1_TIMER7_WCLK_SW      99
#define RST_LSP1_SSI_S_WCLK_SW       100
#define RST_LSP1_SPDIF0_WCLK_SW      101
#define RST_LSP1_SPDIF1_WCLK_SW      102

#define RST_LSP0_TIMER_PCLK_SW      103
#define RST_LSP1_TIMER_PCLK_SW      104



#define RST_COUNT                     (RST_LSP1_TIMER_PCLK_SW+1)

#endif /* _DT_BINDINGS_BST_C1200_RESETS_H_ */