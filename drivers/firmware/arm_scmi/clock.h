#ifndef __C1200_SCMI_H__
#define __C1200_SCMI_H__


#define FIX_CLK_25M_OSC                                       (0)   
#define FIX_CLK_24M_OSC                                       (1)   
#define FIX_CLK_26M_OSC                                       (2) 
#define FIX_LB_SOC_LSP_AUDIO0_WCLK                            (3)
#define FIX_LB_SOC_LSP_AUDIO1_WCLK                            (4)
#define FIX_LB_SOC_LSP_I2S_S0_SCK                             (5)
#define FIX_LB_SOC_LSP_I3C0_SCL                               (6)
#define FIX_LB_SOC_LSP_I3C1_SCL                               (7)
#define FIX_LB_SOC_LSP_FLEXRAY_HCLK                           (8)
#define FIX_LB_SOC_LSP_PCM_TDM0_SCLK_IN                       (9)
#define FIX_LB_SOC_LSP_PCM_TDM1_SCLK_IN                       (10)
#define FIX_LB_SW_PTP_SCLK                                    (11)
#define FIX_LB_SW_WORK_SCLK                                   (12)
#define FIX_COUNT                                             (13)

#define PLL_CLK_CPU                                           (FIX_COUNT + 0)
#define PLL_CLK_GPU                                           (FIX_COUNT + 1)
#define PLL_CLK_CPU_DSU                                       (FIX_COUNT + 2)
#define PLL_CLK_CMN                                           (FIX_COUNT + 3)
#define PLL_CLK_SYSBUS0                                       (FIX_COUNT + 4)
#define PLL_CLK_SYSBUS1                               	      (FIX_COUNT + 5)
#define PLL_CLK_DISPLAY0                                      (FIX_COUNT + 6)
#define PLL_CLK_DISPLAY1                                      (FIX_COUNT + 7)
#define PLL_CLK_DISPLAY2                                      (FIX_COUNT + 8)
#define PLL_CLK_DISPLAY3                                      (FIX_COUNT + 9)
#define PLL_CLK_NET                                 	      (FIX_COUNT + 10)
#define PLL_CLK_UFS                                           (FIX_COUNT + 11)
#define PLL_COUNT                                             (12)



#define FACTOR_FIX_CPU_DSU                                     (FIX_COUNT+PLL_COUNT + 0) 
#define FACTOR_FIX_CMN_550                                     (FIX_COUNT+PLL_COUNT + 1) 
#define FACTOR_FIX_CMN_275                                     (FIX_COUNT+PLL_COUNT + 2) 
#define FACTOR_FIX_SYSBUS0_1000                                (FIX_COUNT+PLL_COUNT + 3) 
#define FACTOR_FIX_SYSBUS0_666                                 (FIX_COUNT+PLL_COUNT + 4) 
#define FACTOR_FIX_SYSBUS0_500                                 (FIX_COUNT+PLL_COUNT + 5) 
#define FACTOR_FIX_SYSBUS0_250                                 (FIX_COUNT+PLL_COUNT + 6) 
#define FACTOR_FIX_SYSBUS0_200                                 (FIX_COUNT+PLL_COUNT + 7) 
#define FACTOR_FIX_SYSBUS0_125                                 (FIX_COUNT+PLL_COUNT + 8) 
#define FACTOR_FIX_SYSBUS0_100                                 (FIX_COUNT+PLL_COUNT + 9) 
#define FACTOR_FIX_SYSBUS0_50                                  (FIX_COUNT+PLL_COUNT + 10)
#define FACTOR_FIX_SYSBUS1_1200                                (FIX_COUNT+PLL_COUNT + 11)
#define FACTOR_FIX_SYSBUS1_800                                 (FIX_COUNT+PLL_COUNT + 12)
#define FACTOR_FIX_SYSBUS1_600                                 (FIX_COUNT+PLL_COUNT + 13)
#define FACTOR_FIX_SYSBUS1_400                                 (FIX_COUNT+PLL_COUNT + 14)
#define FACTOR_FIX_SYSBUS1_300                                 (FIX_COUNT+PLL_COUNT + 15)
#define FACTOR_FIX_SYSBUS1_150                                 (FIX_COUNT+PLL_COUNT + 16)
#define FACTOR_FIX_SYSBUS1_75                                  (FIX_COUNT+PLL_COUNT + 17)
#define FACTOR_FIX_SYSBUS1_25                                  (FIX_COUNT+PLL_COUNT + 18) 
#define FACTOR_LSP0_CLK_DIV_2                                  (FIX_COUNT+PLL_COUNT + 19)    
#define FACTOR_LSP0_CLK_DIV_4                                  (FIX_COUNT+PLL_COUNT + 20)    
#define FACTOR_LSP1_CLK_DIV_2                                  (FIX_COUNT+PLL_COUNT + 21) 
#define FACTOR_LSP1_CLK_DIV_4                                  (FIX_COUNT+PLL_COUNT + 22)       
#define FACTOER_COUNT                                           (23)


#define MUX_CPU_SEL                                                (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 0)  
#define MUX_GPU_SEL                                                (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 1)  
#define MUX_CPU_DSU_SEL                                            (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 2)  
#define MUX_CMN_SEL                                                (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 3)  
#define MUX_SYSBUS0_SEL                                            (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 4)  
#define MUX_SYSBUS1_SEL                                            (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 5)  
#define MUX_DISPLAY0_SEL                                           (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 6)  
#define MUX_DISPLAY1_SEL                                           (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 7)  
#define MUX_DISPLAY2_SEL                                           (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 8)  
#define MUX_DISPLAY3_SEL                                           (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 9)  
#define MUX_NET_SEL                                                (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 10) 
#define MUX_UFS_SEL                                                (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 11) 
#define MUX_DISPLAY0_SEL0_SEL                                      (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 12) 
#define MUX_DISPLAY1_SEL0_SEL                                      (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 13) 
#define MUX_DISPLAY2_SEL0_SEL                                      (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 14) 
#define MUX_DISPLAY3_SEL0_SEL                                      (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 15) 
#define MUX_DISPLAY4_SEL0_SEL                                      (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 16) 
#define MUX_HIFI_DSP_ACLK_SEL                                      (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 17)
#define MUX_CV_CORE_SEL                                            (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 18)
#define MUX_SAFETYNOC_TO_SOCNOC_M_ACLK_SEL                         (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 19)
#define MUX_XGAMC_AXI_ACLK_CLK_SEL                                 (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 20) 
#define MUX_SYSNOC_TO_SOCNIC_AHB_SEL                               (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 21) 
#define MUX_G78AE_SP_WCLK_SEL                                      (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 22) 
#define MUX_HIFI_DSP_XNNE_WCLK_SEL                                 (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 23) 
#define MUX_CV_AXIM1_SEL                                           (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 24) 
#define MUX_CORENOC_TO_SOCNIC_SEL                                  (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 25) 
#define MUX_SGMAC_APB_PCLK_SEL                                     (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 26) 
#define MUX_SOC_SEC_HCLK_SEL                                       (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 27) 
#define MUX_G78AE_WCLK_SEL                                         (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 28) 
#define MUX_HIFI_DSP_WCLK_CLK_SEL                                  (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 29) 
#define MUX_CV_AXIM0_SEL                                           (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 30) 
#define MUX_SECNOC_TO_SOCNIC_SEL                                   (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 31) 
#define MUX_PROBE_AXI_ACLK_SEL                                     (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 32) 
#define MUX_SOC_SEC_ACLK_SEL                                       (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 33) 
#define MUX_G78AE_ACE_ACLK_SEL                                     (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 34) 
#define MUX_LVDS1_HSPEED_SEL                                       (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 35) 
#define MUX_CV_AXIS_SEL                                            (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 36) 
#define MUX_SECNOC_TO_RTNOC_SEL                                    (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 37) 
#define MUX_ATB_TPIU_TRACE_SEL                                     (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 38) 
#define MUX_EDP_ACLK_SEL                                           (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 39) 
#define MUX_G78AE_AXIS_ACLK_SEL                                    (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 40) 
#define MUX_LVDS0_HSPEED_SEL                                       (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 41) 
#define MUX_ISP_SCLK_SEL                                           (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 42) 
#define MUX_RTNOC_TO_SOCNIC_SEL                                    (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 43) 
#define MUX_ATB_AXI_ACLK_SEL                                       (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 44) 
#define MUX_DBSOCNOC_TO_CMN_SEL                                    (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 45)
#define MUX_G78AE_TCU_WCLK_SEL                                     (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 46) 
#define MUX_CPU_1188_SEL                                           (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 47) 
#define MUX_SWNOC_TO_SYSNOC_ACLK_SEL                               (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 48) 
#define MUX_DBNOC_TO_DBSOCNOC_SEL                                  (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 49) 
#define MUX_SMMU_TCU_CODEC_SEL                                     (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 50) 
#define MUX_DBSOCNOC_TO_SYSNOC_SEL                                 (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 51) 
#define MUX_SYS_NOC_800M_WCLK_SEL                                  (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 52) 
#define MUX_SAFENOC_TO_SWNOC_SEL                                   (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 53) 
#define MUX_SOCNIC_S_TO_DBNOC_SEL                                  (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 54) 
#define MUX_SMMU_TCU_COREIP_ACLK_SEL                               (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 55) 
#define MUX_SYSNOC_TO_DBSOCNOC_SEL                                 (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 56) 
#define MUX_SYS_NOC_400M_WCLK_SEL                                  (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 57) 
#define MUX_SOC_LSP1_WCLK_SEL                                      (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 58) 
#define MUX_SDNIC_TO_SYSNOC_ACLK_SEL                               (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 59) 
#define MUX_CPU_PERIP_CLK_SEL                                      (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 60) 
#define MUX_DBSOCNOC_WCLK_SEL                                      (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 61) 
#define MUX_MEDIA_NOC_WCLK_SEL                                     (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 62) 
#define MUX_SOC_LSP0_WCLK_SEL                                      (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 63) 
#define MUX_USBNIC_TO_SYSNOC_ACLK_SEL                              (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 64) 
#define MUX_RTNOC_SWNOC_ACLK_SEL                                   (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 65) 
#define MUX_MSGBX_SWITCH1_TO_SAFE_SEL                              (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 66) 
#define MUX_CORE_NOC_WCLK_SEL                                      (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 67) 
#define MUX_UFS_ACLK_CLK_SEL                                       (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 68) 
#define MUX_LPDDR5_1_S4_ACLK_SEL                                   (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 69) 
#define MUX_RTNOC_SAFENOC_ACLK_SEL                                 (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 70) 
#define MUX_MSGBOX_SWITCH1_TO_SECURE_HCLK_SEL                      (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 71) 
#define MUX_DB_NOC_WCLK_CLK_SEL                                    (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 72) 
#define MUX_SDEMMC1_HCLK_SEL                                       (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 73) 
#define MUX_LDDR5_1_S3_ACLK_SEL                                    (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 74) 
#define MUX_SYSNOC_TO_DBNOC_ACLK_SEL                               (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 75) 
#define MUX_MSGBX_SWITCH1_TO_REALTIME_HCLK_SEL                     (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 76) 
#define MUX_CMN_WCLK_SEL                                           (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 77) 
#define MUX_CS_DMA_HCLK_SEL                                        (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 78) 
#define MUX_SDEMMC1_W_BCLK_SEL                                     (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 79) 
#define MUX_LPPDR5_1_S2_ACLK_SEL                                   (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 80) 
#define MUX_SOC_SRAM_ACLK_CLK_SEL                                  (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 81) 
#define MUX_MSGBX_SWITCH0_TO_SWITCH1_WCLK_SEL                      (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 82) 
#define MUX_CMN_TO_SYSNOC_ACLK_SEL                                 (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 83) 
#define MUX_200_SYSBUS_APB_SEL                                     (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 84) 
#define MUX_MEDIA_DMA_HCLK_SEL                                     (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 85) 
#define MUX_SDEMMC0_HCLK_SEL                                       (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 86) 
#define MUX_LPDDR5_1_S1_ACLK_SEL                                   (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 87) 
#define MUX_SYSNOC_TO_CMN_ACLK_SEL                                 (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 88) 
#define MUX_MSGBX_SWITCH1_TO_MEDIA_SEL                             (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 89) 
#define MUX_100_SYSBUS_APB_SEL                                     (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 90) 
#define MUX_SOC_DMA_HCLK_SEL                                       (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 91) 
#define MUX_SDEMMC0_W_BCLK_SEL                                     (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 92) 
#define MUX_LPDDR5_1_S0_ACLK_SEL                                   (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 93) 
#define MUX_CORENOC_TO_CMN_ACLK_SEL                                (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 94) 
#define MUX_MSGBX_SWITCH1_TO_SW_HCLK_SEL                           (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 95) 
#define MUX_NOC_ATB_CLK_SEL                                        (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 96) 
#define MUX_CPU0_MP4_DSU_CHI_SEL                                   (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 97) 
#define MUX_DB_DMA_HCLK_SEL                                        (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 98) 
#define MUX_USB_1_ACLK_SEL                                         (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 99) 
#define MUX_LPDDR5_0_S4_ACLK_SEL                                   (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 100)
#define MUX_MEDIANOC_TO_CMN_ACLK_SEL                               (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 101)
#define MUX_MSGBX_SWITCH1_WCLK_SEL                                 (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 102)
#define MUX_SOC_DMA_CORE_SEL                                       (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 103)
#define MUX_CPU0_MP4_CORE_SEL                                      (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 104)
#define MUX_SOC_DMA_ACLK_CLK_SEL                                   (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 105)
#define MUX_USB_0_ACLK_CLK_SEL                                     (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 106)
#define MUX_LPDDR5_0_S3_ACLK_CLK_SEL                               (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 107)
#define MUX_SOCNIC_S_TO_MEDIANOC_ACLK_CLK_SEL                      (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 108)
#define MUX_MSGBX_SWITCH0_TO_SOCDMA_HCLK_CLK_SEL                   (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 109)
#define MUX_FLEXRAYNIC_TO_SYSNOC_HCLK_CLK_SEL                      (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 110)
#define MUX_CPU1_MP4_DSU_CHI_CLK_SEL                               (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 111)
#define MUX_MEDIA_DMA_ACLK_SEL                                     (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 112)
#define MUX_PCIE_ACLK_SEL                                          (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 113)
#define MUX_LPDDR5_0_S2_ACLK_SEL                                   (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 114)
#define MUX_SOCNIC_S_TO_CORENOC_ACLK_SEL                           (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 115)
#define MUX_MSGBX_SWITCH0_TO_NET_HCLK_SEL                          (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 116)
#define MUX_DSI_CSITX_IPI_SEL                                      (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 117)
#define MUX_CPU1_MP4_CORE_CLK_SEL                                  (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 118)
#define MUX_SOC_DMA_ACLK_SEL                                       (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 119)
#define MUX_PCIE_DBI_ACLK_SEL                                      (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 120)
#define MUX_LPDDR5_0_S1_ACLK_SEL                                   (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 121)
#define MUX_SOCNIC_S_TO_SAFENOC_ACLK_SEL                           (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 122)
#define MUX_MSGBX_SWITCH0_TO_CPU_HCLK_SEL                          (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 123)
#define MUX_DSI1_CLKEXT_SEL                                        (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 124)
#define MUX_CPU2_MP_ACLK_SEL                                       (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 125)
#define MUX_DB_DMA_ACLK_SEL                                        (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 126)
#define MUX_DISPLAY2_ACLK_SEL                                      (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 127)
#define MUX_LPDDR5_0_S0_ACLK_SEL                                   (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 128)
#define MUX_SOCNIC_S_TO_SWNOC_ACLK_SEL                             (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 129)
#define MUX_MSGBX_SWITCH0_TO_DB_HCLK_SEL                           (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 130)
#define MUX_DSI0_CLKEXT_SEL                                        (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 131)
#define MUX_MP2_MASTER_ACLK_SEL                                    (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 132)
#define MUX_CODEC1_WCLK_SEL                                        (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 133)
#define MUX_DISPLAY1_ACLK_SEL                                      (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 134)
#define MUX_DSP_CLK_SEL                                            (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 135)
#define MUX_SYSNOC_TO_SOCNIC_S_ACLK_SEL                            (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 136)
#define MUX_MSGBX_SWITCH0_TO_ISPCV_HCLK_SEL                        (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 137)
#define MUX_MEDIANOC_TO_SYSNOC_ACLK_SEL                            (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 138)
#define MUX_CPU_MP2_DSU_SEL                                        (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 139)
#define MUX_CODEC0_WCLK_SEL                                        (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 140)
#define MUX_DISPLAY0_ACLK_SEL                                      (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 141)
#define MUX_NET_WCLK_SEL                                           (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 142)
#define MUX_SOCNIC_M_TO_SYSNOC_ACLK_SEL                            (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 143)
#define MUX_MSGBX_SWITCH0_WCLK_SEL                                 (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 144)
#define MUX_SYSNOC_TO_GPUNIC_ACLK_SEL                              (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 145)
#define MUX_MP2_CORE_SEL                                           (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 146)
#define MUX_BD_EXTERNAL_MUX_CLK0_SEL                               (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 147)
#define MUX_BD_EXTERNAL_MUX_CLK1_SEL                               (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 148)
#define MUX_BD_EXTERNAL_MUX_CLK2_SEL                               (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 149)
#define MUX_BD_EXTERNAL_MUX_CLK3_SEL                               (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 150)
#define MUX_BD_EXTERNAL_MUX_CLK4_SEL                               (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 151)
#define MUX_BD_EXTERNAL_MUX_CLK5_SEL                               (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 152)
#define MUX_BD_EXTERNAL_MUX_CLK6_SEL                               (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 153)
#define MUX_BD_EXTERNAL_MUX_CLK7_SEL                               (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 154)
#define MUX_GTC_WCLK_SEL                                           (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 155)
#define MUX_DSI_CFG_REF_CLK_SEL                                    (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 156)
#define MUX_UFS_REF_ALT_CLK_26M_SEL                                (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 157)
#define MUX_USB_U31_PHY_REF_CLK_SEL                                (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 158)
#define MUX_USB_U20_PHY_REF_CLK_SEL                                (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 159)
#define MUX_LSP0_AUDIO_WCLK_4_NUM                                  (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 160)
#define MUX_LSP0_AUDIO_WCLK_5_NUM                                  (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 161)
#define MUX_LSP0_AUDIO_WCLK_0_NUM                                  (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 162)
#define MUX_LSP0_AUDIO_WCLK_1_NUM                                  (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 163)
#define MUX_LSP0_AUDIO_WCLK_2_NUM                                  (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 164)
#define MUX_LSP0_AUDIO_WCLK_3_NUM                                  (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 165)
#define MUX_LSP0_LSP_WCLK_MUX_NUM                                  (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 166)
#define MUX_LSP1_LSP_WCLK_MUX_NUM                                  (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 167)
#define MUX_LSP0_UART_WCLK_MUX_NUM                                 (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 168)
#define MUX_LSP1_AUDIO_WCLK_4_NUM                                  (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 169)
#define MUX_LSP1_AUDIO_WCLK_5_NUM                                  (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 170)
#define MUX_LSP1_AUDIO_WCLK_0_NUM                                  (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 171)
#define MUX_LSP1_AUDIO_WCLK_1_NUM                                  (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 172)
#define MUX_LSP1_AUDIO_WCLK_2_NUM                                  (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 173)
#define MUX_LSP1_AUDIO_WCLK_3_NUM                                  (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 174)
#define MUX_LSP1_UART_WCLK_MUX_NUM                                 (FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 175)
#define MUX_CNT                                     (176)




#define DIVIDOR_TOP_USB_U20_PHY_REF_CLK                    (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 0)    
#define DIVIDOR_SOC_LSP0_FLEXRAY_HCLK                      (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 1)    
#define DIVIDOR_SOC_LSP1_FLEXRAY_HCLK                      (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 2)    
#define DIVIDOR_DISPLAY0_CH0                               (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 3)    
#define DIVIDOR_DISPLAY0_CH1                               (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 4)    
#define DIVIDOR_DISPLAY1_CH0                               (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 5)  //176 + 23 +12 + 13 + 5  
#define DIVIDOR_DISPLAY1_CH1                               (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 6)    
#define DIVIDOR_DISPLAY2_CH0                               (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 7)    
#define DIVIDOR_FIX_UFS_26                                 (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 8) 
#define DIVIDOR_MUX_BD_EXTERNAL_MUX_CLK0                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 9) 
#define DIVIDOR_MUX_BD_EXTERNAL_MUX_CLK1                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 10) 
#define DIVIDOR_MUX_BD_EXTERNAL_MUX_CLK2                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 11) 
#define DIVIDOR_MUX_BD_EXTERNAL_MUX_CLK3                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 12) 
#define DIVIDOR_MUX_BD_EXTERNAL_MUX_CLK4                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 13) 
#define DIVIDOR_MUX_BD_EXTERNAL_MUX_CLK5                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 14) 
#define DIVIDOR_MUX_BD_EXTERNAL_MUX_CLK6                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 15) 
#define DIVIDOR_MUX_BD_EXTERNAL_MUX_CLK7                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 16) 
#define DIVIDOR_GTC_WCLK                                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 17)
#define DIVIDOR_LSP0_GPIO_DBCLK_DIV_WCLK                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 18)
#define DIVIDOR_LSP1_GPIO_DBCLK_DIV_WCLK                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 19)
#define DIVIDOR_LSP0_I2S_S0_MCLK_OUT_NUM                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 20)
#define DIVIDOR_LSP0_I2S_M0_WCLK_DIV_NUM                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 21)
#define DIVIDOR_LSP0_I2S_S1_MCLK_OUT_NUM                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 22)
#define DIVIDOR_LSP0_I2S_M1_WCLK_DIV_NUM                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 23)
#define DIVIDOR_LSP0_TDM0_WCLK_DIV_NUM     	               (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 24)
#define DIVIDOR_LSP0_TDM1_WCLK_DIV_NUM                     (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 25)
#define DIVIDOR_LSP0_SPDIF0_WCLK_DIV_NUM                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 26)
#define DIVIDOR_LSP0_SPDIF1_WCLK_DIV_NUM                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 27)
#define DIVIDOR_LSP1_I2S_S0_MCLK_OUT_NUM                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 28)
#define DIVIDOR_LSP1_I2S_M0_WCLK_DIV_NUM                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 29)
#define DIVIDOR_LSP1_I2S_S1_MCLK_OUT_NUM                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 30)
#define DIVIDOR_LSP1_I2S_M1_WCLK_DIV_NUM                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 31)
#define DIVIDOR_LSP1_TDM0_WCLK_DIV_NUM     	               (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 32)
#define DIVIDOR_LSP1_TDM1_WCLK_DIV_NUM                     (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 33)
#define DIVIDOR_LSP1_SPDIF0_WCLK_DIV_NUM                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 34)
#define DIVIDOR_LSP1_SPDIF1_WCLK_DIV_NUM                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 35)
#define DIVIDOR_LSP0_STA_DIV_I3C_WCLK_DIV_NUM              (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 36)
#define DIVIDOR_LSP1_STA_DIV_I3C_WCLK_DIV_NUM              (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT + 37)
#define DIVIDOR_CNT                                     (38)



#define    GATE_LB_CPU1_MP4_CORE_CLK_EN                    (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 0)
#define    GATE_LB_CPU1_MP4_DSU_CLK_EN                     (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 1)
#define    GATE_LB_CPU_MP2_CORE_CLK_EN                     (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 2)
#define    GATE_LB_CPU_MP2_MP_ACLK_EN                      (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 3)
#define    GATE_LB_CPU_MP2_DSU_CLK_EN                      (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 4)
#define    GATE_LB_CPU_MP2_CS_PCLK_EN                      (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 5)
#define    GATE_LB_LVDS0_PCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 6)
#define    GATE_DBNOC_APB_BR_200_PCLK_EN                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 7)
#define    GATE_LB_CPU_MP2_DB_NOC_PCLK_EN                  (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 8)
#define    GATE_LB_CPU_MP2_MASTER_ACLK_EN                  (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 9)
#define    GATE_CMN_GWCLK_EN                               (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 10)
#define    GATE_LB_GPU_G78AE_SP_S_ACLK_EN                  (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 11)
#define    GATE_LB_GPU_G78AE_SP_PCLK_EN                    (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 12)
#define    GATE_LB_GPU_G78AE_SP_M_ACLK_EN                  (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 13)
#define    GATE_LB_CODEC0_PCLK_EN                          (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 14)
#define    GATE_LB_CODEC1_PCLK_EN                          (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 15)
#define    GATE_LB_GPU_G78AE_WCLK_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 16)
#define    GATE_LB_GPU_G78AE_ACE_M0_ACLK_EN                (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 17)
#define    GATE_LB_GPU_G78AE_ACE_M1_ACLK_EN                (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 18)
#define    GATE_LB_GPU_G78AE_ACE_M2_ACLK_EN                (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 19)
#define    GATE_LB_GPU_G78AE_AXI_S_ACLK_EN                 (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 20)
#define    GATE_LB_GPU_G78AE_TCU_WCLK_EN                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 21)
#define    GATE_LB_CODEC0_WCLK_EN                          (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 22)
#define    GATE_LB_CODEC1_WCLK_EN                          (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 23)
#define    GATE_LB_EDP_PCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 24)
#define    GATE_LB_HIFI_DSP_CFG_PCLK_EN                    (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 25)
#define    GATE_LB_HIFI_DSP_CS_PBCLK_EN                    (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 26)
#define    GATE_LB_HIFI_DSP_WCLK_EN                        (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 27)
#define    GATE_LB_HIFI_DSP_XNNE_WCLK_EN                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 28)
#define    GATE_LB_HIFI_DSP_S_ACLK_EN                      (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 29)
#define    GATE_LB_DB_DMA_ACLK_EN                          (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 30)
#define    GATE_LB_DB_DMA_HCLK_EN                          (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 31)
#define    GATE_LB_SOC_DMA_ACLK_EN                         (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 32)
#define    GATE_LB_SOC_DMA_HCLK_EN                         (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 33)
#define    GATE_LB_MEDIA_DMA_ACLK_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 34)
#define    GATE_LB_MEDIA_DMA_HCLK_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 35)
#define    GATE_LB_CS_DMA_ACLK_EN                          (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 36)
#define    GATE_LB_CS_DMA_HCLK_EN                          (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 37)
#define    GATE_LB_DISPLAY0_CH0_CLK_EN                     (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 38)
#define    GATE_LB_DISPLAY0_CH1_CLK_EN                     (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 39)
#define    GATE_LB_DISPLAY1_CH0_CLK_EN                     (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 40)
#define    GATE_LB_DISPLAY1_CH1_CLK_EN                     (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 41)
#define    GATE_LB_DISPLAY2_CH0_CLK_EN                     (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 42)
#define    GATE_LB_LVDS_CH0_HSPEED_CLK_EN                  (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 43)
#define    GATE_LB_LVDS_CH1_HSPEED_CLK_EN                  (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 44)
#define    GATE_LB_DISPLAY0_ACLK_EN                        (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 45)
#define    GATE_LB_DISPLAY1_ACLK_EN                        (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 46)
#define    GATE_LB_DISPLAY2_ACLK_EN                        (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 47)
#define    GATE_LB_DISPLAY0_PCLK_EN                        (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 48)
#define    GATE_LB_DISPLAY1_PCLK_EN                        (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 49)
#define    GATE_LB_DISPLAY2_PCLK_EN                        (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 50)
#define    GATE_LB_PCIE_IST_ATSPEED_500M_CLK_EN            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 51)
#define    GATE_LB_PCIE_OSC_CLK_EN                         (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 52)
#define    GATE_LB_PCIE_APB_PCLK_EN                        (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 53)
#define    GATE_LB_PCIE_DBI_ACLK_EN                        (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 54)
#define    GATE_LB_PCIE_SLV_ACLK_EN                        (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 55)
#define    GATE_LB_PCIE_X2_MSTR_ACLK_EN                    (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 56)
#define    GATE_LB_PCIE_X4_MSTR_ACLK_EN                    (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 57)
#define    GATE_LB_USB0_IST_ATSPEED_666M_CLK_EN            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 58)
#define    GATE_LB_USB0_IST_ATSPEED_75M_CLK_EN             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 59)
#define    GATE_LB_USB_0_REF_ALT_CLK_EN                    (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 60)
#define    GATE_LB_USB_0_AXI_ACLK_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 61)
#define    GATE_LB_USB_0_APB_PCLK_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 62)
#define    GATE_LB_USB1_IST_ATSPEED_666M_CLK_EN            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 63)
#define    GATE_LB_USB1_IST_ATSPEED_75M_CLK_EN             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 64)
#define    GATE_LB_USB_1_REF_ALT_CLK_EN                    (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 65)
#define    GATE_LB_USB_1_AXI_ACLK_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 66)
#define    GATE_LB_USB_1_APB_PCLK_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 67)
#define    GATE_LB_SDEMMC0_W_BCLK_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 68)
#define    GATE_LB_SDEMMC0_M_HCLK_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 69)
#define    GATE_LB_SDEMMC0_S_PCLK_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 70)
#define    GATE_LB_SDEMMC1_S_PCLK_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 71)
#define    GATE_LB_SDEMMC0_S_HCLK_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 72)
#define    GATE_LB_SDEMMC1_W_BCLK_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 73)
#define    GATE_LB_SDEMMC1_M_HCLK_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 74)
#define    GATE_LB_SDEMMC1_S_HCLK_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 75)
#define    GATE_LB_UFS_AXI_ACLK_EN                         (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 76)
#define    GATE_LB_SOC_LSP0_UART_WCLK_EN                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 77)
#define    GATE_LB_SOC_LSP1_UART_WCLK_EN                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 78)
#define    GATE_LB_SOC_LSP0_WCLK_EN                        (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 79)
#define    GATE_LB_SOC_LSP1_WCLK_EN                        (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 80)
#define    GATE_SAFENOC_TO_SWNOC_GACLK_EN                  (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 81)
#define    GATE_SWNOC_TO_SYSNOC_GACLK_EN                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 82)
#define    GATE_LB_MIPI0_APB_CFG_PCLK_EN                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 83)
#define    GATE_LB_MIPI1_APB_CFG_PCLK_EN                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 84)
#define    GATE_LB_MIPI2_APB_CFG_PCLK_EN                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 85)
#define    GATE_LB_MIPI0_PHY_CFG_CLK_EN                    (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 86)
#define    GATE_LB_MIPI1_PHY_CFG_CLK_EN                    (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 87)
#define    GATE_LB_MIPI2_PHY_CFG_CLK_EN                    (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 88)
#define    GATE_LB_ISP_SCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 89)
#define    GATE_LB_CV_AXIS_CLK_EN                          (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 90)
#define    GATE_LB_CV_AXIM0_CLK_EN                         (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 91)
#define    GATE_LB_CV_AXIM1_CLK_EN                         (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 92)
#define    GATE_LB_CV_CORE_CLK_EN                          (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 93)
#define    GATE_LB_CV_DSP0_PBCLK_EN                        (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 94)
#define    GATE_LB_CV_DSP1_PBCLK_EN                        (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 95)
#define    GATE_LB_CV_DSP2_PBCLK_EN                        (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 96)
#define    GATE_LB_CV_DSP3_PBCLK_EN                        (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 97)
#define    GATE_LB_NET_CLK_EN                              (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 98)
#define    GATE_LB_NET_DSP_PBCLK_EN                        (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 99)
#define    GATE_LB_NET_DSP_CLK_EN                          (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 100)
#define    GATE_LB_LPDDR5_0_S0_ACLK_EN                     (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 101)
#define    GATE_LB_LPDDR5_0_S1_ACLK_EN                     (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 102)
#define    GATE_LB_LPDDR5_0_S2_ACLK_EN                     (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 103)
#define    GATE_LB_LPDDR5_0_S3_ACLK_EN                     (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 104)
#define    GATE_LB_LPDDR5_0_S4_ACLK_EN                     (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 105)
#define    GATE_LB_LPDDR5_1_S0_ACLK_EN                     (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 106)
#define    GATE_LB_LPDDR5_1_S1_ACLK_EN                     (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 107)
#define    GATE_LB_LPDDR5_1_S2_ACLK_EN                     (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 108)
#define    GATE_LB_LPDDR5_1_S3_ACLK_EN                     (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 109)
#define    GATE_LB_LPDDR5_1_S4_ACLK_EN                     (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 110)
#define    GATE_LB_LPDDR5_0_PCLK_EN                        (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 111)
#define    GATE_LB_LPDDR5_1_PCLK_EN                        (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 112)
#define    GATE_CORE_NOC_GWCLK_EN                          (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 113)
#define    GATE_DB_NOC_GWCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 114)
#define    GATE_MEDIA_NOC_GWCLK_EN                         (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 115)
#define    GATE_SYS_NOC_800M_GWCLK_EN                      (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 116)
#define    GATE_SYS_NOC_400M_GWCLK_EN                      (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 117)
#define    GATE_SYSNOC_APB0_200_PCLK_EN                    (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 118)
#define    GATE_SYSNOC_APB1_200_PCLK_EN                    (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 119)
#define    GATE_LB_MATRIX_TOP_CRM_APB_PCLK_EN              (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 120)
#define    GATE_LB_MATRIX_PMM_REG_APB_PCLK_EN              (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 121)
#define    GATE_LB_MATRIX_SYS_CTRL_APB_PCLK_EN             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 122)
#define    GATE_LB_MATRIX_IPC_APB_PCLK_EN                  (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 123)
#define    GATE_COREIP_SUBSYSTEM_CSR_APB_PCLK_EN           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 124)
#define    GATE_DB_SUBSYSTEM_CSR_APB_PCLK_EN               (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 125)
#define    GATE_MEDIA_SUBSYSTEM_CSR_APB_PCLK_EN            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 126)
#define    GATE_CPU_SUBSYSTEM_CSR_APB_PCLK_EN              (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 127)
#define    GATE_MEDIANOC_APB_200_PCLK_EN                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 128)
#define    GATE_USBNIC_TO_SYSNOC_GACLK_EN                  (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 129)
#define    GATE_SDNIC_TO_SYSNOC_GACLK_EN                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 130)
#define    GATE_SOCNIC_S_TO_DBNOC_GACLK_EN                 (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 131)
#define    GATE_DBNOC_TO_DBSOCNOC_M_GACLK_EN               (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 132)
#define    GATE_RTNOC_TO_SOCNIC_M_GACLK_EN                 (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 133)
#define    GATE_SECNOC_TO_RTNOC_GACLK_EN                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 134)
#define    GATE_SECNOC_TO_SOCNIC_M_GACLK_EN                (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 135)
#define    GATE_CORENOC_TO_SOCNIC_M_GACLK_EN               (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 136)
#define    GATE_SAFETYNOC_TO_SOCNIC_M_GACLK_EN             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 137)
#define    GATE_SOCNIC_M_TO_SYSNOC_GACLK_EN                (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 138)
#define    GATE_SYSNOC_TO_SOCNIC_S_GACLK_EN                (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 139)
#define    GATE_SOCNIC_S_TO_SWNOC_GACLK_EN                 (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 140)
#define    GATE_SOCNIC_S_TO_SAFENOC_GACLK_EN               (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 141)
#define    GATE_SOCNIC_S_TO_CORENOC_GACLK_EN               (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 142)
#define    GATE_SOCNIC_S_TO_MEDIANOC_GACLK_EN              (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 143)
#define    GATE_PCIENOC_TO_CMN_GACLK_EN                    (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 144)
#define    GATE_MEDIANOC_TO_CMN_GACLK_EN                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 145)
#define    GATE_CORENOC_TO_CMN_GACLK_EN                    (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 146)
#define    GATE_SYSNOC_TO_CMN_GACLK_EN                     (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 147)
#define    GATE_CMN_TO_SYSNOC_GACLK_EN                     (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 148)
#define    GATE_SOC_SRAM_GACLK_EN                          (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 149)
#define    GATE_SYSNOC_TO_DBNOC_GACLK_EN                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 150)
#define    GATE_RTNOC_TO_SAFENOC_GACLK_EN                  (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 151)
#define    GATE_SAFENOC_TO_RTNOC_GACLK_EN                  (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 152)
#define    GATE_RTNOC_TO_SWNOC_GACLK_EN                    (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 153)
#define    GATE_SWNOC_TO_RTNOC_GACLK_EN                    (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 154)
#define    GATE_LB_EDP_SUSPEND_CLK_EN                      (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 155)
#define    GATE_LB_EDP_CLK_800M_EN                         (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 156)
#define    GATE_LB_CPU_PERIP_SYSNOC_APB_PCLK_EN            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 157)
#define    GATE_LB_CPU_PERIP_CLK_EN                        (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 158)
#define    GATE_SMMU_TCU_COREIP_GACLK_EN                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 159)
#define    GATE_SMMU_TCU_CODEC_GACLK_EN                    (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 160)
#define    GATE_LB_PCIE_NOC_600M_CLK_EN                    (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 161)
#define    GATE_LB_CPU_PERIP_CSTOPCLK_EN                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 162)
#define    GATE_LB_ISP_CV_MIPI_PVT_SYS_CLK_EN              (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 163)
#define    GATE_LB_NET_PVT_SYS_CLK_EN                      (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 164)
#define    GATE_LB_CPU0_MP4_PVT_SYS_CLK_EN                 (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 165)
#define    GATE_LB_CPU1_MP4_PVT_SYS_CLK_EN                 (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 166)
#define    GATE_LB_GPU_G78AE_PVT_SYS_CLK_EN                (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 167)
#define    GATE_LB_VDD_SOC_PVT_SYS_CLK_EN                  (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 168)
#define    GATE_LB_VDD_SAFETY_PVT_SYS_CLK_EN               (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 169)
#define    GATE_LB_SW_CSTOP_CLK_EN                         (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 170)
#define    GATE_SYSNOC_ATBCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 171)
#define    GATE_LB_SW_SUBSYSTEM_IST_PCLK_EN                (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 172)
#define    GATE_LB_DSI_PCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 173)
#define    GATE_LB_DSI_CFG_REF_CLK_EN                      (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 174)
#define    GATE_CMN_CSR_PCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 175)
#define    GATE_LB_SOC_LSP0_APB_S0_PCLK_EN                 (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 176)
#define    GATE_LB_SOC_LSP1_APB_S0_PCLK_EN                 (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 177)
#define    GATE_LB_LPDDR5_0_MAIN_CLK_EN                    (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 178)
#define    GATE_LB_LPDDR5_1_MAIN_CLK_EN                    (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 179)
#define    GATE_SOC_PROBE_APB_PCLK_EN                      (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 180)
#define    GATE_SOC_PROBE_AXI_GACLK_EN                     (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 181)
#define    GATE_DDR0_PROBE_AXI_GACLK_EN                    (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 182)
#define    GATE_DDR1_PROBE_AXI_GACLK_EN                    (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 183)
#define    GATE_MEDIA_PROBE_APB_PCLK_EN                    (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 184)
#define    GATE_MEDIA_PROBE_AXI_GACLK_EN                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 185)
#define    GATE_COREIP_PROBE_APB_PCLK_EN                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 186)
#define    GATE_COREIP_PROBE_AXI_GACLK_EN                  (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 187)
#define    GATE_LB_UFS_APB_PCLK_EN                         (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 188)
#define    GATE_LB_UFS_REF_ALT_CLK_EN                      (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 189)
#define    GATE_TOP_IST_PCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 190)
#define    GATE_SOC_IST_CLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 191)
#define    GATE_SAFETY_COREIP_PCLK_EN                      (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 192)
#define    GATE_LB_STANDBY_APB_PCLK_EN                     (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 193)
#define    GATE_SOC_SAFE_CSR_PCLK_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 194)
#define    GATE_LB_NET_SAFETY_PCLK_EN                      (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 195)
#define    GATE_LB_ISP_SAFE_PCLK_EN                        (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 196)
#define    GATE_LB_CV_SAFE_PCLK_EN                         (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 197)
#define    GATE_LB_UFS_IST_ATSPEED_600M_CLK_EN             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 198)
#define    GATE_SAFETY_TO_SOC_SYS_CSTOP_CLK_EN             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 199)
#define    GATE_SOC_SYS_TO_DSP_CSTOP_CLK_EN                (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 200)
#define    GATE_LB_RT_CSTOP_CLK_EN                         (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 201)
#define    GATE_ATB_CSTOP_CLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 202)
#define    GATE_ATB_APB_GPCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 203)
#define    GATE_ATB_AXI_GACLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 204)
#define    GATE_LB_NET_IST_ATSPEED_25M_CLK_EN              (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 205)
#define    GATE_DSP0_IST_ATSPEED_25M_CLK_EN                (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 206)
#define    GATE_DSP1_IST_ATSPEED_25M_CLK_EN                (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 207)
#define    GATE_DSP2_IST_ATSPEED_25M_CLK_EN                (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 208)
#define    GATE_DSP3_IST_ATSPEED_25M_CLK_EN                (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 209)
#define    GATE_LB_HIFI_DSP_IST_ATSPEED_25M_CLK_EN         (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 210)
#define    GATE_LB_XGMAC_AXIM_ACLK_EN                      (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 211)
#define    GATE_LB_XGMAC_APB_S_PCLK_EN                     (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 212)
#define    GATE_LB_SOC_LSP0_IST_ATSPEED_25M_CLK_EN         (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 213)
#define    GATE_LB_SOC_LSP1_IST_ATSPEED_25M_CLK_EN         (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 214)
#define    GATE_MSGBX_SWITCH0_GWCLK_EN                     (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 215)
#define    GATE_MSGBX_SWITCH0_TO_ISPCV_GHCLK_EN            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 216)
#define    GATE_MSGBX_SWITCH0_TO_DB_GHCLK_EN               (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 217)
#define    GATE_MSGBX_SWITCH0_TO_CPU_GHCLK_EN              (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 218)
#define    GATE_MSGBX_SWITCH0_TO_NET_GHCLK_EN              (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 219)
#define    GATE_MSGBX_SWITCH0_TO_SOCDMA_GHCLK_EN           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 220)
#define    GATE_MSGBX_SWITCH1_GWCLK_EN                     (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 221)
#define    GATE_MSGBX_SWITCH1_TO_SW_GHCLK_EN               (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 222)
#define    GATE_MSGBX_SWITCH1_TO_MEDIA_GHCLK_EN            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 223)
#define    GATE_MSGBX_SWITCH0_TO_SWITCH1_GWCLK_EN          (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 224)
#define    GATE_MSGBX_SWITCH1_TO_REALTIME_GHCLK_EN         (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 225)
#define    GATE_MSGBX_SWITCH1_TO_SECURE_GHCLK_EN           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 226)
#define    GATE_MSGBX_SWITCH1_TO_SAFE_GHCLK_EN             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 227)
#define    GATE_ATB_TPIU_TRACE_GCLK_EN                     (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 228)
#define    GATE_DBSOCNOC_GWCLK_EN                          (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 229)
#define    GATE_SYSNOC_TO_DBSOCNOC_GACLK_EN                (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 230)
#define    GATE_DBSOCNOC_TO_CMN_GACLK_EN                   (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 231)
#define    GATE_DBSOCNOC_TO_SYSNOC_GACLK_EN                (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 232)
#define    GATE_LB_EDP_ACLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 233)
#define    GATE_LB_SOC_SEC_ACLK_EN                         (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 234)
#define    GATE_LB_SOC_SEC_HCLK_EN                         (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 235)
#define    GATE_SYSNOC_TO_SOCNIC_AHB_CFG0_GHCLK_EN         (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 236)
#define    GATE_SYSNOC_TO_SOCNIC_AHB_CFG1_GHCLK_EN         (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 237)
#define    GATE_SYSNOC_TO_GPUNIC_GACLK_EN                  (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 238)
#define    GATE_BD_EXTERNAL_GCLK0_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 239)
#define    GATE_BD_EXTERNAL_GCLK1_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 240)
#define    GATE_BD_EXTERNAL_GCLK2_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 241)
#define    GATE_BD_EXTERNAL_GCLK3_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 242)
#define    GATE_BD_EXTERNAL_GCLK4_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 243)
#define    GATE_BD_EXTERNAL_GCLK5_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 244)
#define    GATE_BD_EXTERNAL_GCLK6_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 245)
#define    GATE_BD_EXTERNAL_GCLK7_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 246)
#define    GATE_LB_ISP_IST_ATSPEED_TEST_CLK_EN             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 247)
#define    GATE_LB_EDP_REF_ALT_CLK_EN                      (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 248)
#define    GATE_LB_HIFI_DSP_M_ACLK_EN                      (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 249)
#define    GATE_LB_REALTIME_SUBSYSTEM_IST_PCLK_EN          (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 250)
#define    GATE_MEDIANOC_TO_SYSNOC_GACLK_EN                (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 251)
#define    GATE_TOP_GTC_PCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 252)
#define    GATE_LB_DSI0_CLKEXT_EN                          (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 253)
#define    GATE_LB_DSI1_CLKEXT_EN                          (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 254)
#define    GATE_LB_DSI_CSITX_IPI_CLK_EN                    (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 255)
#define    GATE_FLEXRAYNIC_TO_SYSNOC_GHCLK_EN              (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 256)
#define    GATE_LB_SOC_DMA_CORE_CLK_EN                     (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 257)
#define    GATE_MEDIANOC_ATBCLK_EN                         (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 258)
#define    GATE_DBNOC_ATBCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 259)
#define    GATE_GPUNOC_ATBCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 260)
#define    GATE_PCIENOC_ATBCLK_EN                          (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 261)
#define    GATE_CORENOC_ATBCLK_EN                          (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 262)
#define    GATE_TOPISTCTRL_TO_TOPCRM_PCLK_EN               (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 263)
#define    GATE_LB_USB_0_U20_PHY_REF_CLK_EN                (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 264)
#define    GATE_LB_USB_1_U20_PHY_REF_CLK_EN                (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 265)
#define    GATE_LB_USB_0_U31_PHY_REF_CLK_EN                (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 266)
#define    GATE_LB_USB_1_U31_PHY_REF_CLK_EN                (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 267)
#define    GATE_LB_SOC_LSP0_FLEXRAY_HCLK_EN                (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 268)
#define    GATE_LB_SOC_LSP1_FLEXRAY_HCLK_EN                (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 269)
#define    GATE_CORENOC_APB_BR_PCLK_EN                     (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 270)
#define GATE_LSP0_I2C0_SMBUS_WCLK_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 271) 
#define GATE_LSP0_I2C0_SMBUS_PCLK_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 272)
#define GATE_LSP1_I2C0_SMBUS_WCLK_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 273)
#define GATE_LSP1_I2C0_SMBUS_PCLK_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 274)
#define GATE_LSP0_I2C1_SMBUS_WCLK_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 275)
#define GATE_LSP0_I2C1_SMBUS_PCLK_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 276)
#define GATE_LSP1_I2C1_SMBUS_WCLK_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 277)
#define GATE_LSP1_I2C1_SMBUS_PCLK_EN                       (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 278)
#define GATE_LSP0_I2C0_WCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 279)
#define GATE_LSP0_I2C0_PCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 280)
#define GATE_LSP1_I2C0_WCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 281)
#define GATE_LSP1_I2C0_PCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 282)
#define GATE_LSP0_I2C1_WCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 283)
#define GATE_LSP0_I2C1_PCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 284)
#define GATE_LSP1_I2C1_WCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 285)
#define GATE_LSP1_I2C1_PCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 286)
#define GATE_LSP0_I2C2_WCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 287)
#define GATE_LSP0_I2C2_PCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 288)
#define GATE_LSP1_I2C2_WCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 289)
#define GATE_LSP1_I2C2_PCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 290)
#define GATE_LSP0_I2C3_WCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 291)
#define GATE_LSP0_I2C3_PCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 292)
#define GATE_LSP1_I2C3_WCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 293)
#define GATE_LSP1_I2C3_PCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 294)
#define GATE_LSP0_UART0_WCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 295)
#define GATE_LSP0_UART0_PCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 296)
#define GATE_LSP1_UART0_WCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 297)
#define GATE_LSP1_UART0_PCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 298)
#define GATE_LSP0_UART1_WCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 299)
#define GATE_LSP0_UART1_PCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 300)
#define GATE_LSP1_UART1_WCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 301)
#define GATE_LSP1_UART1_PCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 302)
#define GATE_LSP0_SSI_M_WCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 303)
#define GATE_LSP0_SSI_M_PCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 304)
#define GATE_LSP1_SSI_M_WCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 305)
#define GATE_LSP1_SSI_M_PCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 306)
#define GATE_LSP0_SSI_S_WCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 307)
#define GATE_LSP0_SSI_S_PCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 308)
#define GATE_LSP1_SSI_S_WCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 309)
#define GATE_LSP1_SSI_S_PCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 310)
#define GATE_LSP0_GPIO0_DBCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 311)
#define GATE_LSP0_GPIO0_PCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 312)
#define GATE_LSP0_GPIO1_DBCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 313)
#define GATE_LSP0_GPIO1_PCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 314)
#define GATE_LSP0_GPIO2_DBCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 315)
#define GATE_LSP0_GPIO2_PCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 316)
#define GATE_LSP0_GPIO3_DBCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 317)
#define GATE_LSP0_GPIO3_PCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 318)
#define GATE_LSP1_GPIO0_DBCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 319)
#define GATE_LSP1_GPIO0_PCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 320)
#define GATE_LSP1_GPIO1_DBCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 321)
#define GATE_LSP1_GPIO1_PCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 322)
#define GATE_LSP1_GPIO2_DBCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 323)
#define GATE_LSP1_GPIO2_PCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 324)
#define GATE_LSP1_GPIO3_DBCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 325)
#define GATE_LSP1_GPIO3_PCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 326)
#define GATE_LSP0_WDT0_WCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 327)
#define GATE_LSP0_WDT0_PCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 328)
#define GATE_LSP0_WDT1_WCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 329)
#define GATE_LSP0_WDT1_PCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 330)
#define GATE_LSP1_WDT0_WCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 331)
#define GATE_LSP1_WDT0_PCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 332)
#define GATE_LSP1_WDT1_WCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 333)
#define GATE_LSP1_WDT1_PCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 334)
#define GATE_LSP0_TIMER0_WCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 335)
#define GATE_LSP0_TIMER1_WCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 336)
#define GATE_LSP0_TIMER2_WCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 337)
#define GATE_LSP0_TIMER3_WCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 338)
#define GATE_LSP0_TIMER4_WCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 339)
#define GATE_LSP0_TIMER5_WCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 340)
#define GATE_LSP0_TIMER6_WCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 341)
#define GATE_LSP0_TIMER7_WCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 342)
#define GATE_LSP0_TIMER_PCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 343)
#define GATE_LSP1_TIMER0_WCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 344)
#define GATE_LSP1_TIMER1_WCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 345)
#define GATE_LSP1_TIMER2_WCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 346)
#define GATE_LSP1_TIMER3_WCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 347)
#define GATE_LSP1_TIMER4_WCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 348)
#define GATE_LSP1_TIMER5_WCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 349)
#define GATE_LSP1_TIMER6_WCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 350)
#define GATE_LSP1_TIMER7_WCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 351)
#define GATE_LSP1_TIMER_PCLK_EN                            (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 352)
#define GATE_LSP0_I2S0_WCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 353)
#define GATE_LSP0_I2S0_PCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 354)
#define GATE_LSP0_I2S1_WCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 355)
#define GATE_LSP0_I2S1_PCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 356)
#define GATE_LSP1_I2S0_WCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 357)
#define GATE_LSP1_I2S0_PCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 358)
#define GATE_LSP1_I2S1_WCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 359)
#define GATE_LSP1_I2S1_PCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 360)
#define GATE_LSP0_I3C0_WCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 361)
#define GATE_LSP0_I3C0_PCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 362)
#define GATE_LSP0_I3C1_WCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 363)
#define GATE_LSP0_I3C1_PCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 364)
#define GATE_LSP1_I3C0_WCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 365)
#define GATE_LSP1_I3C0_PCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 366)
#define GATE_LSP1_I3C1_WCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 367)
#define GATE_LSP1_I3C1_PCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 368)
#define GATE_LSP0_TDM0_WCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 369)
#define GATE_LSP0_TDM0_PCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 370)
#define GATE_LSP0_TDM1_WCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 371)
#define GATE_LSP0_TDM1_PCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 372)
#define GATE_LSP1_TDM0_WCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 373)
#define GATE_LSP1_TDM0_PCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 374)
#define GATE_LSP1_TDM1_WCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 375)
#define GATE_LSP1_TDM1_PCLK_EN                             (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 376)
#define GATE_LSP0_SPDIF0_WCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 377)
#define GATE_LSP0_SPDIF1_WCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 378)
#define GATE_LSP1_SPDIF0_WCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 379)
#define GATE_LSP1_SPDIF1_WCLK_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 380)
#define GATE_XGMAC_PTP_CLK_EN                              (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 381)
#define GATE_XGMAC_WCLK_EN                                 (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 382)
#define GATE_CPU0_MP4_DSU_CHI_EN                           (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 383)
#define GATE_CPU0_MP4_CORE_EN                              (MUX_CNT+FACTOER_COUNT+FIX_COUNT+PLL_COUNT+DIVIDOR_CNT + 384)
#define GATE_CNT                                           385

#define CLK_MAX                                            (MUX_CNT+FACTOER_COUNT + FIX_COUNT + PLL_COUNT + DIVIDOR_CNT + GATE_CNT)






/* clock name macros */
#define CLK_25M_OSC                                  "25m_clk" //osc
#define CLK_24M_OSC                                "24m_clk" //osc
#define CLK_26M_OSC                                "26m_clk" //osc


#define PLL_CPU                                 "pll_cpu" //pll cpu
#define PLL_GPU                                 "pll_gpu" //pll cpu
#define PLL_CPU_DSU                             "pll_cpu_dsu" //pll cpu
#define PLL_CMN                                 "pll_cmn" //pll cpu
#define PLL_SYSBUS0                             "pll_sysbus0" //pll cpu
#define PLL_SYSBUS1                             "pll_sysbus1" //pll cpu
#define PLL_DISPLAY0                            "pll_display0" //pll cpu
#define PLL_DISPLAY1                            "pll_display1" //pll cpu
#define PLL_DISPLAY2                            "pll_display2" //pll cpu
#define PLL_DISPLAY3                            "pll_display3" //pll cpu
#define PLL_NET                                 "pll_net" //pll cpu
#define PLL_UFS                                 "pll_ufs" //pll cpu
#define PLL_PCIE                                "pll_pcie" //pll cpu


#define FACTOR_CPU_DSU_NAME                     "factor_cpu_dsu_2" //pll cpu
#define FACTOR_CMN_550                          "cmn_550" //pll cpu
#define FACTOR_CMN_275                          "cmn_275" //pll cpu
#define FACTOR_SYSBUS0_1000                     "sysbus0_1000" //pll cpu
#define FACTOR_SYSBUS0_666                             "clk_666_sysbus" //pll cpu
#define FACTOR_SYSBUS0_500                             "clk_500_sysbus" //pll cpu
#define FACTOR_SYSBUS0_250                             "sysbus0_250" //pll cpu
#define FACTOR_SYSBUS0_200                            "sysbus0_200" //pll cpu
#define FACTOR_SYSBUS0_125                            "sysbus0_125" //pll cpu
#define FACTOR_SYSBUS0_100                            "sysbus0_100" //pll cpu
#define FACTOR_SYSBUS0_50                            "sysbus0_50" //pll cpu
#define FACTOR_SYSBUS1_1200                     "sysbus1_1200" //pll cpu
#define FACTOR_SYSBUS1_800                                 "clk_800_sysbus" //pll cpu
#define FACTOR_SYSBUS1_600                             "clk_600_sysbus" //pll cpu
#define FACTOR_SYSBUS1_400                             "sysbus1_400" //pll cpu
#define FACTOR_SYSBUS1_300                            "sysbus1_300" //pll cpu
#define FACTOR_SYSBUS1_150                            "sysbus1_150" //pll cpu
#define FACTOR_SYSBUS1_75                            "sysbus1_75" //pll cpu
#define FACTOR_SYSBUS1_25                            "sysbus1_25" //pll cpu




#define MUX_CPU                                 "sel_cpu" //pll cpu
#define MUX_GPU                                 "sel_gpu" //pll cpu
#define MUX_CPU_DSU                             "sel_cpu_dsu" //pll cpu
#define MUX_CMN                                 "sel_cmn" //pll cpu
#define MUX_SYSBUS0                             "sel_sysbus0" //pll cpu
#define MUX_SYSBUS1                             "sel_sysbus1" //pll cpu
#define MUX_DISPLAY0                            "sel_display0" //pll cpu
#define MUX_DISPLAY1                            "sel_display1" //pll cpu
#define MUX_DISPLAY2                            "sel_display2" //pll cpu
#define MUX_DISPLAY3                            "sel_display3" //pll cpu
#define MUX_NET                                 "sel_net" //pll cpu
#define MUX_UFS                                 "sel_ufs" //pll cpu
#define MUX_PCIE                                "sel_pcie_25m" //pll cpu

#define MUX_DISPLAY0_SEL0                       "pll_display_mux0_1188_clk_sel"
#define MUX_DISPLAY1_SEL0                       "pll_display_mux1_1188_clk_sel"
#define MUX_DISPLAY2_SEL0                       "pll_display_mux2_1188_clk_sel"
#define MUX_DISPLAY3_SEL0                       "pll_display_mux3_1188_clk_sel"
#define MUX_DISPLAY4_SEL0                       "pll_display_mux4_1188_clk_sel"


#define MUX_HIFI_DSP_ACLK                      "hifi_dsp_aclk"
#define MUX_CV_CORE_CLK                        "cv_core_clk"
#define MUX_SAFETYNOC_TO_SOCNIC_M_ACLK                        "safetynoc_to_socnic_m_aclk"
#define MUX_XGMAC_AXI_ACLK                       "xgmac_axi_aclk"
#define MUX_SYSNOC_TO_SOCNIC_AHB_CFG_HCLK                        "sysnoc_to_socnic_ahb_cfg_hclk"



#define MUX_G78AE_SP_WCLK                      "g78ae_sp_wclk"
#define MUX_HIFI_DSP_XNNE_WCLK                       "hifi_dsp_xnne_wclk"
#define MUX_CV_AXIM1_CLK                        "cv_axim1_clk"
#define MUX_CORENOC_TO_SOCNIC_M_ACLK                       "corenoc_to_socnic_m_aclk"
#define MUX_XGMAC_APB_PCLK                        "xgmac_apb_pclk"
#define MUX_SOC_SEC_HCLK                        "soc_sec_hclk"


#define MUX_G78AE_WCLK_CLK                      "g78ae_wclk"
#define MUX_HIFI_DSP_WCLK                       "hifi_dsp_wclk"
#define MUX_CV_AXIM0_CLK                        "cv_axim0_clk"
#define MUX_SECNOC_TO_SOCNIC_M_ACLK                       "secnoc_to_socnic_m_aclk"
#define MUX_PROBE_AXI_ACLK                        "probe_axi_aclk"
#define MUX_SOC_SEC_ACLK                        "soc_sec_aclk"




#define MUX_G78AE_ACE_ACLK                      "g78ae_ace_aclk"
#define MUX_CLK_LVDS1_HSPEED                       "clk_lvds1_hspeed_1188"
#define MUX_CV_AXIS_CLK                        "cv_axis_clk"
#define MUX_SECNOC_TO_RTNOC_ACLK                      "secnoc_to_rtnoc_aclk"
#define MUX_ATB_TPIU_TRACE_CLK                        "atb_tpiu_trace_clk"
#define MUX_EDP_ACLK                        "edp_aclk"


#define MUX_G78AE_AXIS_ACLK                     "g78ae_axis_aclk"
#define MUX_CLK_LVDS0_HSPEED                       "clk_lvds0_hspeed_1188"
#define MUX_ISP_SCLK                        "isp_sclk"
#define MUX_RTNOC_TO_SOCNIC_M_ACLK                      "rtnoc_to_socnic_m_aclk"
#define MUX_ATB_AXI_ACLK                        "atb_axi_aclk"
#define MUX_DBSOCNOC_TO_CMN_ACLK                        "dbsocnoc_to_cmn_aclk"


#define MUX_G78AE_TCU_WCLK                     "g78ae_tcu_wclk"
#define MUX_CLK_CPU_1188                       "clk_cpu_1188"
#define MUX_SWNOC_TO_SYSNOC_ACLK                        "swnoc_to_sysnoc_aclk"
#define MUX_DBNOC_TO_DBSOCNOC_M_ACLK                      "dbnoc_to_dbsocnoc_m_aclk"
#define MUX_SMMU_TCU_CODEC_ACLK                        "smmu_tcu_codec_aclk"
#define MUX_DBSOCNOC_TO_SYSNOC_ACLK                        "dbsocnoc_to_sysnoc_aclk"




#define MUX_SYS_NOC_800M_WCLK                     "sys_noc_800m_wclk"
#define MUX_SAFENOC_TO_SWNOC_ACLK                        "safenoc_to_swnoc_aclk"
#define MUX_SOCNIC_S_TO_DBNOC_ACLK                      "socnic_s_to_dbnoc_aclk"
#define MUX_SMMU_TCU_COREIP_ACLK                        "smmu_tcu_coreip_aclk"
#define MUX_SYSNOC_TO_DBSOCNOC_ACLK                        "sysnoc_to_dbsocnoc_aclk"

#define MUX_SYS_NOC_400M_WCLK                     "sys_noc_400m_wclk"
#define MUX_SOC_LSP1_WCLK                        "soc_lsp1_wclk"
#define MUX_SDNIC_TO_SYSNOC_ACLK                      "sdnic_to_sysnoc_aclk"
#define MUX_CPU_PERIP_CLK                        "cpu_perip_clk"
#define MUX_DBSOCNOC_WCLK                        "dbsocnoc_wclk"



#define MUX_MEDIA_NOC_WCLK                     "media_noc_wclk"
#define MUX_SOC_LSP0_WCLK                        "soc_lsp0_wclk"
#define MUX_USBNIC_TO_SYSNOC_ACLK                      "usbnic_to_sysnoc_aclk"
#define MUX_RTNOC_SWNOC_ACLK                        "rtnoc_swnoc_aclk"
#define MUX_MSGBX_SWITCH1_TO_SAFE_HCLK                        "msgbx_switch1_to_safe_hclk"

#define MUX_CORE_NOC_WCLK                    "core_noc_wclk"
#define MUX_UFS_ACLK                        "ufs_aclk"
#define MUX_LPDDR5_1_S4_ACLK                      "lpddr5_1_s4_aclk"
#define MUX_RTNOC_SAFENOC_ACLK                        "rtnoc_safenoc_aclk"
#define MUX_MSGBX_SWITCH1_TO_SECURE_HCLK                        "msgbx_switch1_to_secure_hclk"



#define MUX_DB_NOC_WCLK                     "db_noc_wclk"
#define MUX_SDEMMC1_HCLK                        "sdemmc1_hclk"
#define MUX_LPDDR5_1_S3_ACLK                      "lpddr5_1_s3_aclk"
#define MUX_SYSNOC_TO_DBNOC_ACLK                        "sysnoc_to_dbnoc_aclk"
#define MUX_MSGBX_SWITCH1_TO_REALTIME_HCLK                        "msgbx_switch1_to_realtime_hclk"

#define MUX_CMN_WCLK_MUX                    "cmn_wclk"
#define MUX_CS_DMA_HCLK                    "cs_dma_hclk"
#define MUX_SDEMMC1_W_BCLK                        "sdemmc1_w_bclk"
#define MUX_LPDDR5_1_S2_ACLK                     "lpddr5_1_s2_aclk"
#define MUX_SOC_SRAM_ACLK                        "soc_sram_aclk"
#define MUX_MSGBX_SWITCH0_TO_SWITCH1_WCLK                        "msgbx_switch0_to_switch1_wclk"
#define MUX_CMN_TO_SYSNOC_ACLK                       "cmn_to_sysnoc_aclk"




#define MUX_CLK_200_SYSBUS_APB                    "clk_200_sysbus_apb"
#define MUX_MEDIA_DMA_HCLK                    "media_dma_hclk"
#define MUX_SDEMMC0_HCLK                        "sdemmc0_hclk"
#define MUX_LPDDR5_1_S1_ACLK                     "lpddr5_1_s1_aclk"
#define MUX_SYSNOC_TO_CMN_ACLK                        "sysnoc_to_cmn_aclk"
#define MUX_MSGBX_SWITCH1_TO_MEDIA_HCLK                       "msgbx_switch1_to_media_hclk"


#define MUX_CLK_100_SYSBUS_APB                    "clk_100_sysbus_apb"
#define MUX_SOC_DMA_HCLK                   "soc_dma_hclk"
#define MUX_SDEMMC0_W_BCLK                        "sdemmc0_w_bclk"
#define MUX_LPDDR5_1_S0_ACLK                     "lpddr5_1_s0_aclk"
#define MUX_CORENOC_TO_CMN_ACLK                       "corenoc_to_cmn_aclk"
#define MUX_MSGBX_SWITCH1_TO_SW_HCLK                       "msgbx_switch1_to_sw_hclk"
#define MUX_NOC_ATB_CLK                      "noc_atb_clk"






#define MUX_CPU0_MP4_DSU_CHI_CLK                    "cpu0_mp4_dsu_chi_clk"
#define MUX_DB_DMA_HCLK                   "db_dma_hclk"
#define MUX_USB_1_ACLK                        "usb_1_aclk"
#define MUX_LPDDR5_0_S4_ACLKK                     "lpddr5_0_s4_aclk"
#define MUX_MEDIANOC_TO_CMN_ACLK                        "medianoc_to_cmn_aclk"
#define MUX_MSGBX_SWITCH1_WCLK                       "msgbx_switch1_wclk"
#define MUX_SOC_DMA_CORE                       "soc_dma_core_clk"


#define MUX_CPU0_MP4_CORE_CLK                    "cpu0_mp4_core_clk"
#define MUX_CS_DMA_ACLK                   "cs_dma_aclk"
#define MUX_USB_0_ACLK                        "usb_0_aclk"
#define MUX_LPDDR5_0_S3_ACLK                     "lpddr5_0_s3_aclk"
#define MUX_SOCNIC_S_TO_MEDIANOC_ACLK                       "socnic_s_to_medianoc_aclk"
#define MUX_MSGBX_SWITCH0_TO_SOCDMA_HCLK                       "msgbx_switch0_to_socdma_hclk"
#define MUX_FLEXRAYNIC_TO_SYSNOC_HCLK                      "flexraynic_to_sysnoc_hclk"





#define MUX_CPU1_MP4_DSU_CHI_CLK                    "cpu1_mp4_dsu_chi_clk"
#define MUX_MEDIA_DMA_ACLK                   "media_dma_aclk"
#define MUX_PCIE_ACLK                        "pcie_aclk"
#define MUX_LPDDR5_0_S2_ACLK                    "lpddr5_0_s2_aclk"
#define MUX_SOCNIC_S_TO_CORENOC_ACLK                        "socnic_s_to_corenoc_aclk"
#define MUX_MSGBX_SWITCH0_TO_NET_HCLK                       "msgbx_switch0_to_net_hclk"
#define MUX_DSI_CSITX_IPI                       "dsi_csitx_ipi_clk"


#define MUX_CPU1_MP4_CORE_CLK                   "cpu1_mp4_core_clk"
#define MUX_SOC_DMA_ACLK                   "soc_dma_aclk"
#define MUX_PCIE_DBI_ACLK                        "pcie_dbi_aclk"
#define MUX_LPDDR5_0_S1_ACLK                     "lpddr5_0_s1_aclk"
#define MUX_SOCNIC_S_TO_SAFENOC_ACLK                       "socnic_s_to_safenoc_aclk"
#define MUX_MSGBX_SWITCH0_TO_CPU_HCLK                       "msgbx_switch0_to_cpu_hclk"
#define MUX_DSI1_CLKEXT                      "dsi1_clkext"







#define MUX_CPU_MP2_MP_ACLK                   "cpu_mp2_mp_aclk"
#define MUX_DB_DMA_ACLK                   "db_dma_aclk"
#define MUX_DISPLAY2_ACLK                        "display2_aclk"
#define MUX_LPDDR5_0_S0_ACLK                    "lpddr5_0_s0_aclk"
#define MUX_SOCNIC_S_TO_SWNOC_ACLK                        "socnic_s_to_swnoc_aclk"
#define MUX_MSGBX_SWITCH0_TO_DB_HCLK                       "msgbx_switch0_to_db_hclk"
#define MUX_DSI0_CLKEXT                       "dsi0_clkext"


#define MUX_CPU_MP2_MASTER_ACLK                   "cpu_mp2_master_aclk"
#define MUX_CODEC1_WCLK                  "codec1_wclk"
#define MUX_DISPLAY1_ACLK                        "display1_aclk"
#define MUX_DSP_CLK                     "dsp_clk"
#define MUX_SYSNOC_TO_SOCNIC_S_ACLK                       "sysnoc_to_socnic_s_aclk"
#define MUX_MSGBX_SWITCH0_TO_ISPCV_HCLK                       "msgbx_switch0_to_ispcv_hclk"
#define MUX_MEDIANOC_TO_SYSNOC_ACLK                      "medianoc_to_sysnoc_aclk"


#define MUX_CPU_MP2_DSU_CLK                    "cpu_mp2_dsu_clk"
#define MUX_CODEC0_WCLK                   "codec0_wclk"
#define MUX_DISPLAY0_ACLK                        "display0_aclk"
#define MUX_NET_WCLK                    "net_wclk"
#define MUX_SOCNIC_M_TO_SYSNOC_ACLK                        "socnic_m_to_sysnoc_aclk"
#define MUX_MSGBX_SWITCH0_WCLK                       "msgbx_switch0_wclk"
#define MUX_SYSNOC_TO_GPUNIC_ACLK                       "sysnoc_to_gpunic_aclk"


#define MUX_CPU_MP2_CORE_CLK                   "cpu_mp2_core_clk"

#define MUX_BD_EXTERNAL_MUX_CLK0                "bd_external_mux_clk0" 
#define MUX_BD_EXTERNAL_MUX_CLK1                "bd_external_mux_clk1" 
#define MUX_BD_EXTERNAL_MUX_CLK2                "bd_external_mux_clk2" 
#define MUX_BD_EXTERNAL_MUX_CLK3                "bd_external_mux_clk3" 
#define MUX_BD_EXTERNAL_MUX_CLK4                "bd_external_mux_clk4" 
#define MUX_BD_EXTERNAL_MUX_CLK5                "bd_external_mux_clk5" 
#define MUX_BD_EXTERNAL_MUX_CLK6                "bd_external_mux_clk6" 
#define MUX_BD_EXTERNAL_MUX_CLK7                "bd_external_mux_clk7" 

#define MUX_GTC_WCLK                            "gtc_wclk"

#define MUX_DSI_CFG_REF_CLK         "dsi_cfg_ref_clk"
#define MUX_UFS_REF_ALT_CLK_26M        "ufs_ref_alt_clk_26m"
#define MUX_USB_U31_PHY_REF_CLK         "usb_u31_phy_ref_clk"
#define MUX_USB_U20_PHY_REF_CLK         "usb_u20_phy_ref_clk" 





#define DIVIDOR_BD_EXTERNAL_CLK0                  "bd_external_clk0"
#define DIVIDOR_BD_EXTERNAL_CLK1                  "bd_external_clk1"
#define DIVIDOR_BD_EXTERNAL_CLK2                  "bd_external_clk2"
#define DIVIDOR_BD_EXTERNAL_CLK3                  "bd_external_clk3"
#define DIVIDOR_BD_EXTERNAL_CLK4                  "bd_external_clk4"
#define DIVIDOR_BD_EXTERNAL_CLK5                  "bd_external_clk5"
#define DIVIDOR_BD_EXTERNAL_CLK6                  "bd_external_clk6"
#define DIVIDOR_BD_EXTERNAL_CLK7                  "bd_external_clk7"



#define  DIVIDOR_GTC_DIV_WCLK                     "gtc_div_wclk"

#define  DIVIDOR_TOP_USB_U20_PHY_REF_CLK_NAME                   "top_usb_u20_phy_ref_clk"
#define  DIVIDOR_SOC_LSP0_FLEXRAY_HCLK_NAME                     "soc_lsp0_flexray_hclk"
#define  DIVIDOR_SOC_LSP1_FLEXRAY_HCLK_NAME                     "soc_lsp1_flexray_hclk"



#define LB_CPU1_MP4_CORE_CLK_EN                         "lb_cpu1_mp4_core_clk_en"
#define LB_CPU1_MP4_DSU_CLK_EN                          "lb_cpu1_mp4_dsu_clk_en"
#define LB_CPU_MP2_CORE_CLK_EN                          "lb_cpu_mp2_core_clk_en"
#define LB_CPU_MP2_MP_ACLK_EN                           "lb_cpu_mp2_mp_aclk_en"
#define LB_CPU_MP2_DSU_CLK_EN                           "lb_cpu_mp2_dsu_clk_en"
#define LB_CPU_MP2_CS_PCLK_EN                           "lb_cpu_mp2_cs_pclk_en"
#define LB_LVDS0_PCLK_EN                                "lb_lvds0_pclk_en"
#define DBNOC_APB_BR_200_PCLK_EN                        "dbnoc_apb_br_200_pclk_en"
#define LB_CPU_MP2_DB_NOC_PCLK_EN                       "lb_cpu_mp2_db_noc_pclk_en"
#define LB_CPU_MP2_MASTER_ACLK_EN                       "lb_cpu_mp2_master_aclk_en"
#define CMN_GWCLK_EN                                    "cmn_gwclk_en"
#define LB_GPU_G78AE_SP_S_ACLK_EN                       "lb_gpu_g78ae_sp_s_aclk_en"
#define LB_GPU_G78AE_SP_PCLK_EN                         "lb_gpu_g78ae_sp_pclk_en"
#define LB_GPU_G78AE_SP_M_ACLK_EN                       "lb_gpu_g78ae_sp_m_aclk_en"
#define LB_CODEC0_PCLK_EN                               "lb_codec0_pclk_en"
#define LB_CODEC1_PCLK_EN                               "lb_codec1_pclk_en"
#define LB_GPU_G78AE_WCLK_EN                            "lb_gpu_g78ae_wclk_en"
#define LB_GPU_G78AE_ACE_M0_ACLK_EN                     "lb_gpu_g78ae_ace_m0_aclk_en"
#define LB_GPU_G78AE_ACE_M1_ACLK_EN                     "lb_gpu_g78ae_ace_m1_aclk_en"
#define LB_GPU_G78AE_ACE_M2_ACLK_EN                     "lb_gpu_g78ae_ace_m2_aclk_en"
#define LB_GPU_G78AE_AXI_S_ACLK_EN                      "lb_gpu_g78ae_axi_s_aclk_en"
#define LB_GPU_G78AE_TCU_WCLK_EN                        "lb_gpu_g78ae_tcu_wclk_en"
#define LB_CODEC0_WCLK_EN                               "lb_codec0_wclk_en"
#define LB_CODEC1_WCLK_EN                               "lb_codec1_wclk_en"
#define LB_EDP_PCLK_EN                                  "lb_edp_pclk_en"
#define LB_HIFI_DSP_CFG_PCLK_EN                         "lb_hifi_dsp_cfg_pclk_en"
#define LB_HIFI_DSP_CS_PBCLK_EN                         "lb_hifi_dsp_cs_pbclk_en"
#define LB_HIFI_DSP_WCLK_EN                             "lb_hifi_dsp_wclk_en"
#define LB_HIFI_DSP_XNNE_WCLK_EN                        "lb_hifi_dsp_xnne_wclk_en"
#define LB_HIFI_DSP_S_ACLK_EN                           "lb_hifi_dsp_s_aclk_en"
#define LB_DB_DMA_ACLK_EN                               "lb_db_dma_aclk_en"
#define LB_DB_DMA_HCLK_EN                               "lb_db_dma_hclk_en"
#define LB_SOC_DMA_ACLK_EN                              "lb_soc_dma_aclk_en"
#define LB_SOC_DMA_HCLK_EN                              "lb_soc_dma_hclk_en"
#define LB_MEDIA_DMA_ACLK_EN                            "lb_media_dma_aclk_en"
#define LB_MEDIA_DMA_HCLK_EN                            "lb_media_dma_hclk_en"
#define LB_CS_DMA_ACLK_EN                               "lb_cs_dma_aclk_en"
#define LB_CS_DMA_HCLK_EN                               "lb_cs_dma_hclk_en"
#define LB_DISPLAY0_CH0_CLK_EN                          "lb_display0_ch0_clk_en"
#define LB_DISPLAY0_CH1_CLK_EN                          "lb_display0_ch1_clk_en"
#define LB_DISPLAY1_CH0_CLK_EN                          "lb_display1_ch0_clk_en"
#define LB_DISPLAY1_CH1_CLK_EN                          "lb_display1_ch1_clk_en"
#define LB_DISPLAY2_CH0_CLK_EN                          "lb_display2_ch0_clk_en"
#define LB_LVDS_CH0_HSPEED_CLK_EN                       "lb_lvds_ch0_hspeed_clk_en"
#define LB_LVDS_CH1_HSPEED_CLK_EN                       "lb_lvds_ch1_hspeed_clk_en"
#define LB_DISPLAY0_ACLK_EN                             "lb_display0_aclk_en"
#define LB_DISPLAY1_ACLK_EN                             "lb_display1_aclk_en"
#define LB_DISPLAY2_ACLK_EN                             "lb_display2_aclk_en"
#define LB_DISPLAY0_PCLK_EN                             "lb_display0_pclk_en"
#define LB_DISPLAY1_PCLK_EN                             "lb_display1_pclk_en"
#define LB_DISPLAY2_PCLK_EN                             "lb_display2_pclk_en"
#define LB_PCIE_IST_ATSPEED_500M_CLK_EN                 "lb_pcie_ist_atspeed_500m_clk_en"
#define LB_PCIE_OSC_CLK_EN                              "lb_pcie_osc_clk_en"
#define LB_PCIE_APB_PCLK_EN                             "lb_pcie_apb_pclk_en"
#define LB_PCIE_DBI_ACLK_EN                             "lb_pcie_dbi_aclk_en"
#define LB_PCIE_SLV_ACLK_EN                             "lb_pcie_slv_aclk_en"
#define LB_PCIE_X2_MSTR_ACLK_EN                         "lb_pcie_x2_mstr_aclk_en"
#define LB_PCIE_X4_MSTR_ACLK_EN                         "lb_pcie_x4_mstr_aclk_en"
#define LB_USB0_IST_ATSPEED_666M_CLK_EN                 "lb_usb0_ist_atspeed_666m_clk_en"
#define LB_USB0_IST_ATSPEED_75M_CLK_EN                  "lb_usb0_ist_atspeed_75m_clk_en"
#define LB_USB_0_REF_ALT_CLK_EN                         "lb_usb_0_ref_alt_clk_en"
#define LB_USB_0_AXI_ACLK_EN                            "lb_usb_0_axi_aclk_en"
#define LB_USB_0_APB_PCLK_EN                            "lb_usb_0_apb_pclk_en"
#define LB_USB1_IST_ATSPEED_666M_CLK_EN                 "lb_usb1_ist_atspeed_666m_clk_en"
#define LB_USB1_IST_ATSPEED_75M_CLK_EN                  "lb_usb1_ist_atspeed_75m_clk_en"
#define LB_USB_1_REF_ALT_CLK_EN                         "lb_usb_1_ref_alt_clk_en"
#define LB_USB_1_AXI_ACLK_EN                            "lb_usb_1_axi_aclk_en"
#define LB_USB_1_APB_PCLK_EN                            "lb_usb_1_apb_pclk_en"
#define LB_SDEMMC0_W_BCLK_EN                            "lb_sdemmc0_w_bclk_en"
#define LB_SDEMMC0_M_HCLK_EN                            "lb_sdemmc0_m_hclk_en"
#define LB_SDEMMC0_S_PCLK_EN                            "lb_sdemmc0_s_pclk_en"
#define LB_SDEMMC1_S_PCLK_EN                            "lb_sdemmc1_s_pclk_en"
#define LB_SDEMMC0_S_HCLK_EN                            "lb_sdemmc0_s_hclk_en"
#define LB_SDEMMC1_W_BCLK_EN                            "lb_sdemmc1_w_bclk_en"
#define LB_SDEMMC1_M_HCLK_EN                            "lb_sdemmc1_m_hclk_en"
#define LB_SDEMMC1_S_HCLK_EN                            "lb_sdemmc1_s_hclk_en"
#define LB_UFS_AXI_ACLK_EN                              "lb_ufs_axi_aclk_en"
#define LB_SOC_LSP0_UART_WCLK_EN                        "lb_soc_lsp0_uart_wclk_en"
#define LB_SOC_LSP1_UART_WCLK_EN                        "lb_soc_lsp1_uart_wclk_en"
#define LB_SOC_LSP0_WCLK_EN                             "lb_soc_lsp0_wclk_en"
#define LB_SOC_LSP1_WCLK_EN                             "lb_soc_lsp1_wclk_en"
#define SAFENOC_TO_SWNOC_GACLK_EN                       "safenoc_to_swnoc_gaclk_en"
#define SWNOC_TO_SYSNOC_GACLK_EN                        "swnoc_to_sysnoc_gaclk_en"
#define LB_MIPI0_APB_CFG_PCLK_EN                        "lb_mipi0_apb_cfg_pclk_en"
#define LB_MIPI1_APB_CFG_PCLK_EN                        "lb_mipi1_apb_cfg_pclk_en"
#define LB_MIPI2_APB_CFG_PCLK_EN                        "lb_mipi2_apb_cfg_pclk_en"
#define LB_MIPI0_PHY_CFG_CLK_EN                         "lb_mipi0_phy_cfg_clk_en"
#define LB_MIPI1_PHY_CFG_CLK_EN                         "lb_mipi1_phy_cfg_clk_en"
#define LB_MIPI2_PHY_CFG_CLK_EN                         "lb_mipi2_phy_cfg_clk_en"
#define LB_ISP_SCLK_EN                                  "lb_isp_sclk_en"
#define LB_CV_AXIS_CLK_EN                               "lb_cv_axis_clk_en"
#define LB_CV_AXIM0_CLK_EN                              "lb_cv_axim0_clk_en"
#define LB_CV_AXIM1_CLK_EN                              "lb_cv_axim1_clk_en"
#define LB_CV_CORE_CLK_EN                               "lb_cv_core_clk_en"
#define LB_CV_DSP0_PBCLK_EN                             "lb_cv_dsp0_pbclk_en"
#define LB_CV_DSP1_PBCLK_EN                             "lb_cv_dsp1_pbclk_en"
#define LB_CV_DSP2_PBCLK_EN                             "lb_cv_dsp2_pbclk_en"
#define LB_CV_DSP3_PBCLK_EN                             "lb_cv_dsp3_pbclk_en"
#define LB_NET_CLK_EN                                   "lb_net_clk_en"
#define LB_NET_DSP_PBCLK_EN                             "lb_net_dsp_pbclk_en"
#define LB_NET_DSP_CLK_EN                               "lb_net_dsp_clk_en"
#define LB_LPDDR5_0_S0_ACLK_EN                          "lb_lpddr5_0_s0_aclk_en"
#define LB_LPDDR5_0_S1_ACLK_EN                          "lb_lpddr5_0_s1_aclk_en"
#define LB_LPDDR5_0_S2_ACLK_EN                          "lb_lpddr5_0_s2_aclk_en"
#define LB_LPDDR5_0_S3_ACLK_EN                          "lb_lpddr5_0_s3_aclk_en"
#define LB_LPDDR5_0_S4_ACLK_EN                          "lb_lpddr5_0_s4_aclk_en"
#define LB_LPDDR5_1_S0_ACLK_EN                          "lb_lpddr5_1_s0_aclk_en"
#define LB_LPDDR5_1_S1_ACLK_EN                          "lb_lpddr5_1_s1_aclk_en"
#define LB_LPDDR5_1_S2_ACLK_EN                          "lb_lpddr5_1_s2_aclk_en"
#define LB_LPDDR5_1_S3_ACLK_EN                          "lb_lpddr5_1_s3_aclk_en"
#define LB_LPDDR5_1_S4_ACLK_EN                          "lb_lpddr5_1_s4_aclk_en"
#define LB_LPDDR5_0_PCLK_EN                             "lb_lpddr5_0_pclk_en"
#define LB_LPDDR5_1_PCLK_EN                             "lb_lpddr5_1_pclk_en"
#define CORE_NOC_GWCLK_EN                               "core_noc_gwclk_en"
#define DB_NOC_GWCLK_EN                                 "db_noc_gwclk_en"
#define MEDIA_NOC_GWCLK_EN                              "media_noc_gwclk_en"
#define SYS_NOC_800M_GWCLK_EN                           "sys_noc_800m_gwclk_en"
#define SYS_NOC_400M_GWCLK_EN                           "sys_noc_400m_gwclk_en"
#define SYSNOC_APB0_200_PCLK_EN                         "sysnoc_apb0_200_pclk_en"
#define SYSNOC_APB1_200_PCLK_EN                         "sysnoc_apb1_200_pclk_en"
#define LB_MATRIX_TOP_CRM_APB_PCLK_EN                   "lb_matrix_top_crm_apb_pclk_en"
#define LB_MATRIX_PMM_REG_APB_PCLK_EN                   "lb_matrix_pmm_reg_apb_pclk_en"
#define LB_MATRIX_SYS_CTRL_APB_PCLK_EN                  "lb_matrix_sys_ctrl_apb_pclk_en"
#define LB_MATRIX_IPC_APB_PCLK_EN                       "lb_matrix_ipc_apb_pclk_en"
#define COREIP_SUBSYSTEM_CSR_APB_PCLK_EN                "coreip_subsystem_csr_apb_pclk_en"
#define DB_SUBSYSTEM_CSR_APB_PCLK_EN                    "db_subsystem_csr_apb_pclk_en"
#define MEDIA_SUBSYSTEM_CSR_APB_PCLK_EN                 "media_subsystem_csr_apb_pclk_en"
#define CPU_SUBSYSTEM_CSR_APB_PCLK_EN                   "cpu_subsystem_csr_apb_pclk_en"
#define MEDIANOC_APB_200_PCLK_EN                        "medianoc_apb_200_pclk_en"
#define USBNIC_TO_SYSNOC_GACLK_EN                       "usbnic_to_sysnoc_gaclk_en"
#define SDNIC_TO_SYSNOC_GACLK_EN                        "sdnic_to_sysnoc_gaclk_en"
#define SOCNIC_S_TO_DBNOC_GACLK_EN                      "socnic_s_to_dbnoc_gaclk_en"
#define DBNOC_TO_DBSOCNOC_M_GACLK_EN                    "dbnoc_to_dbsocnoc_m_gaclk_en"
#define RTNOC_TO_SOCNIC_M_GACLK_EN                      "rtnoc_to_socnic_m_gaclk_en"
#define SECNOC_TO_RTNOC_GACLK_EN                        "secnoc_to_rtnoc_gaclk_en"
#define SECNOC_TO_SOCNIC_M_GACLK_EN                     "secnoc_to_socnic_m_gaclk_en"
#define CORENOC_TO_SOCNIC_M_GACLK_EN                    "corenoc_to_socnic_m_gaclk_en"
#define SAFETYNOC_TO_SOCNIC_M_GACLK_EN                  "safetynoc_to_socnic_m_gaclk_en"
#define SOCNIC_M_TO_SYSNOC_GACLK_EN                     "socnic_m_to_sysnoc_gaclk_en"
#define SYSNOC_TO_SOCNIC_S_GACLK_EN                     "sysnoc_to_socnic_s_gaclk_en"
#define SOCNIC_S_TO_SWNOC_GACLK_EN                      "socnic_s_to_swnoc_gaclk_en"
#define SOCNIC_S_TO_SAFENOC_GACLK_EN                    "socnic_s_to_safenoc_gaclk_en"
#define SOCNIC_S_TO_CORENOC_GACLK_EN                    "socnic_s_to_corenoc_gaclk_en"
#define SOCNIC_S_TO_MEDIANOC_GACLK_EN                   "socnic_s_to_medianoc_gaclk_en"
#define PCIENOC_TO_CMN_GACLK_EN                         "pcienoc_to_cmn_gaclk_en"
#define MEDIANOC_TO_CMN_GACLK_EN                        "medianoc_to_cmn_gaclk_en"
#define CORENOC_TO_CMN_GACLK_EN                         "corenoc_to_cmn_gaclk_en"
#define SYSNOC_TO_CMN_GACLK_EN                          "sysnoc_to_cmn_gaclk_en"
#define CMN_TO_SYSNOC_GACLK_EN                          "cmn_to_sysnoc_gaclk_en"
#define SOC_SRAM_GACLK_EN                               "soc_sram_gaclk_en"
#define SYSNOC_TO_DBNOC_GACLK_EN                        "sysnoc_to_dbnoc_gaclk_en"
#define RTNOC_TO_SAFENOC_GACLK_EN                       "rtnoc_to_safenoc_gaclk_en"
#define SAFENOC_TO_RTNOC_GACLK_EN                       "safenoc_to_rtnoc_gaclk_en"
#define RTNOC_TO_SWNOC_GACLK_EN                         "rtnoc_to_swnoc_gaclk_en"
#define SWNOC_TO_RTNOC_GACLK_EN                         "swnoc_to_rtnoc_gaclk_en"
#define LB_EDP_SUSPEND_CLK_EN                           "lb_edp_suspend_clk_en"
#define LB_EDP_CLK_800M_EN                              "lb_edp_clk_800m_en"
#define LB_CPU_PERIP_SYSNOC_APB_PCLK_EN                 "lb_cpu_perip_sysnoc_apb_pclk_en"
#define LB_CPU_PERIP_CLK_EN                             "lb_cpu_perip_clk_en"
#define SMMU_TCU_COREIP_GACLK_EN                        "smmu_tcu_coreip_gaclk_en"
#define SMMU_TCU_CODEC_GACLK_EN                         "smmu_tcu_codec_gaclk_en"
#define LB_PCIE_NOC_600M_CLK_EN                         "lb_pcie_noc_600m_clk_en"
#define LB_CPU_PERIP_CSTOPCLK_EN                        "lb_cpu_perip_cstopclk_en"
#define LB_ISP_CV_MIPI_PVT_SYS_CLK_EN                   "lb_isp_cv_mipi_pvt_sys_clk_en"
#define LB_NET_PVT_SYS_CLK_EN                           "lb_net_pvt_sys_clk_en"
#define LB_CPU0_MP4_PVT_SYS_CLK_EN                      "lb_cpu0_mp4_pvt_sys_clk_en"
#define LB_CPU1_MP4_PVT_SYS_CLK_EN                      "lb_cpu1_mp4_pvt_sys_clk_en"
#define LB_GPU_G78AE_PVT_SYS_CLK_EN                     "lb_gpu_g78ae_pvt_sys_clk_en"
#define LB_VDD_SOC_PVT_SYS_CLK_EN                       "lb_vdd_soc_pvt_sys_clk_en"
#define LB_VDD_SAFETY_PVT_SYS_CLK_EN                    "lb_vdd_safety_pvt_sys_clk_en"
#define LB_SW_CSTOP_CLK_EN                              "lb_sw_cstop_clk_en"
#define SYSNOC_ATBCLK_EN                                "sysnoc_atbclk_en"
#define LB_SW_SUBSYSTEM_IST_PCLK_EN                     "lb_sw_subsystem_ist_pclk_en"
#define LB_DSI_PCLK_EN                                  "lb_dsi_pclk_en"
#define LB_DSI_CFG_REF_CLK_EN                           "lb_dsi_cfg_ref_clk_en"
#define CMN_CSR_PCLK_EN                                 "cmn_csr_pclk_en"
#define LB_SOC_LSP0_APB_S0_PCLK_EN                      "lb_soc_lsp0_apb_s0_pclk_en"
#define LB_SOC_LSP1_APB_S0_PCLK_EN                      "lb_soc_lsp1_apb_s0_pclk_en"
#define LB_LPDDR5_0_MAIN_CLK_EN                         "lb_lpddr5_0_main_clk_en"
#define LB_LPDDR5_1_MAIN_CLK_EN                         "lb_lpddr5_1_main_clk_en"
#define SOC_PROBE_APB_PCLK_EN                           "soc_probe_apb_pclk_en"
#define SOC_PROBE_AXI_GACLK_EN                          "soc_probe_axi_gaclk_en"
#define DDR0_PROBE_AXI_GACLK_EN                         "ddr0_probe_axi_gaclk_en"
#define DDR1_PROBE_AXI_GACLK_EN                         "ddr1_probe_axi_gaclk_en"
#define MEDIA_PROBE_APB_PCLK_EN                         "media_probe_apb_pclk_en"
#define MEDIA_PROBE_AXI_GACLK_EN                        "media_probe_axi_gaclk_en"
#define COREIP_PROBE_APB_PCLK_EN                        "coreip_probe_apb_pclk_en"
#define COREIP_PROBE_AXI_GACLK_EN                       "coreip_probe_axi_gaclk_en"
#define LB_UFS_APB_PCLK_EN                              "lb_ufs_apb_pclk_en"
#define LB_UFS_REF_ALT_CLK_EN                           "lb_ufs_ref_alt_clk_en"
#define TOP_IST_PCLK_EN                                 "top_ist_pclk_en"
#define SOC_IST_CLK_EN                                  "soc_ist_clk_en"
#define SAFETY_COREIP_PCLK_EN                           "safety_coreip_pclk_en"
#define LB_STANDBY_APB_PCLK_EN                          "lb_standby_apb_pclk_en"
#define SOC_SAFE_CSR_PCLK_EN                            "soc_safe_csr_pclk_en"
#define LB_NET_SAFETY_PCLK_EN                           "lb_net_safety_pclk_en"
#define LB_ISP_SAFE_PCLK_EN                             "lb_isp_safe_pclk_en"
#define LB_CV_SAFE_PCLK_EN                              "lb_cv_safe_pclk_en"
#define LB_UFS_IST_ATSPEED_600M_CLK_EN                  "lb_ufs_ist_atspeed_600m_clk_en"
#define SAFETY_TO_SOC_SYS_CSTOP_CLK_EN                  "safety_to_soc_sys_cstop_clk_en"
#define SOC_SYS_TO_DSP_CSTOP_CLK_EN                     "soc_sys_to_dsp_cstop_clk_en"
#define LB_RT_CSTOP_CLK_EN                              "lb_rt_cstop_clk_en"
#define ATB_CSTOP_CLK_EN                                "atb_cstop_clk_en"
#define ATB_APB_GPCLK_EN                                "atb_apb_gpclk_en"
#define ATB_AXI_GACLK_EN                                "atb_axi_gaclk_en"
#define LB_NET_IST_ATSPEED_25M_CLK_EN                   "lb_net_ist_atspeed_25m_clk_en"
#define DSP0_IST_ATSPEED_25M_CLK_EN                     "dsp0_ist_atspeed_25m_clk_en"
#define DSP1_IST_ATSPEED_25M_CLK_EN                     "dsp1_ist_atspeed_25m_clk_en"
#define DSP2_IST_ATSPEED_25M_CLK_EN                     "dsp2_ist_atspeed_25m_clk_en"
#define DSP3_IST_ATSPEED_25M_CLK_EN                     "dsp3_ist_atspeed_25m_clk_en"
#define LB_HIFI_DSP_IST_ATSPEED_25M_CLK_EN              "lb_hifi_dsp_ist_atspeed_25m_clk_en"
#define LB_XGMAC_AXIM_ACLK_EN                           "lb_xgmac_axim_aclk_en"
#define LB_XGMAC_APB_S_PCLK_EN                          "lb_xgmac_apb_s_pclk_en"
#define LB_SOC_LSP0_IST_ATSPEED_25M_CLK_EN              "lb_soc_lsp0_ist_atspeed_25m_clk_en"
#define LB_SOC_LSP1_IST_ATSPEED_25M_CLK_EN              "lb_soc_lsp1_ist_atspeed_25m_clk_en"
#define MSGBX_SWITCH0_GWCLK_EN                          "msgbx_switch0_gwclk_en"
#define MSGBX_SWITCH0_TO_ISPCV_GHCLK_EN                 "msgbx_switch0_to_ispcv_ghclk_en"
#define MSGBX_SWITCH0_TO_DB_GHCLK_EN                    "msgbx_switch0_to_db_ghclk_en"
#define MSGBX_SWITCH0_TO_CPU_GHCLK_EN                   "msgbx_switch0_to_cpu_ghclk_en"
#define MSGBX_SWITCH0_TO_NET_GHCLK_EN                   "msgbx_switch0_to_net_ghclk_en"
#define MSGBX_SWITCH0_TO_SOCDMA_GHCLK_EN                "msgbx_switch0_to_socdma_ghclk_en"
#define MSGBX_SWITCH1_GWCLK_EN                          "msgbx_switch1_gwclk_en"
#define MSGBX_SWITCH1_TO_SW_GHCLK_EN                    "msgbx_switch1_to_sw_ghclk_en"
#define MSGBX_SWITCH1_TO_MEDIA_GHCLK_EN                 "msgbx_switch1_to_media_ghclk_en"
#define MSGBX_SWITCH0_TO_SWITCH1_GWCLK_EN               "msgbx_switch0_to_switch1_gwclk_en"
#define MSGBX_SWITCH1_TO_REALTIME_GHCLK_EN              "msgbx_switch1_to_realtime_ghclk_en"
#define MSGBX_SWITCH1_TO_SECURE_GHCLK_EN                "msgbx_switch1_to_secure_ghclk_en"
#define MSGBX_SWITCH1_TO_SAFE_GHCLK_EN                  "msgbx_switch1_to_safe_ghclk_en"
#define ATB_TPIU_TRACE_GCLK_EN                          "atb_tpiu_trace_gclk_en"
#define DBSOCNOC_GWCLK_EN                               "dbsocnoc_gwclk_en"
#define SYSNOC_TO_DBSOCNOC_GACLK_EN                     "sysnoc_to_dbsocnoc_gaclk_en"
#define DBSOCNOC_TO_CMN_GACLK_EN                        "dbsocnoc_to_cmn_gaclk_en"
#define DBSOCNOC_TO_SYSNOC_GACLK_EN                     "dbsocnoc_to_sysnoc_gaclk_en"
#define LB_EDP_ACLK_EN                                  "lb_edp_aclk_en"
#define LB_SOC_SEC_ACLK_EN                              "lb_soc_sec_aclk_en"
#define LB_SOC_SEC_HCLK_EN                              "lb_soc_sec_hclk_en"
#define SYSNOC_TO_SOCNIC_AHB_CFG0_GHCLK_EN              "sysnoc_to_socnic_ahb_cfg0_ghclk_en"
#define SYSNOC_TO_SOCNIC_AHB_CFG1_GHCLK_EN              "sysnoc_to_socnic_ahb_cfg1_ghclk_en"
#define SYSNOC_TO_GPUNIC_GACLK_EN                       "sysnoc_to_gpunic_gaclk_en"
#define BD_EXTERNAL_GCLK0_EN                            "bd_external_gclk0_en"
#define BD_EXTERNAL_GCLK1_EN                            "bd_external_gclk1_en"
#define BD_EXTERNAL_GCLK2_EN                            "bd_external_gclk2_en"
#define BD_EXTERNAL_GCLK3_EN                            "bd_external_gclk3_en"
#define BD_EXTERNAL_GCLK4_EN                            "bd_external_gclk4_en"
#define BD_EXTERNAL_GCLK5_EN                            "bd_external_gclk5_en"
#define BD_EXTERNAL_GCLK6_EN                            "bd_external_gclk6_en"
#define BD_EXTERNAL_GCLK7_EN                            "bd_external_gclk7_en"
#define LB_ISP_IST_ATSPEED_TEST_CLK_EN                  "lb_isp_ist_atspeed_test_clk_en"
#define LB_EDP_REF_ALT_CLK_EN                           "lb_edp_ref_alt_clk_en"
#define LB_HIFI_DSP_M_ACLK_EN                           "lb_hifi_dsp_m_aclk_en"
#define LB_REALTIME_SUBSYSTEM_IST_PCLK_EN               "lb_realtime_subsystem_ist_pclk_en"
#define MEDIANOC_TO_SYSNOC_GACLK_EN                     "medianoc_to_sysnoc_gaclk_en"
#define TOP_GTC_PCLK_EN                                 "top_gtc_pclk_en"
#define LB_DSI0_CLKEXT_EN                               "lb_dsi0_clkext_en"
#define LB_DSI1_CLKEXT_EN                               "lb_dsi1_clkext_en"
#define LB_DSI_CSITX_IPI_CLK_EN                         "lb_dsi_csitx_ipi_clk_en"
#define FLEXRAYNIC_TO_SYSNOC_GHCLK_EN                   "flexraynic_to_sysnoc_ghclk_en"
#define LB_SOC_DMA_CORE_CLK_EN                          "lb_soc_dma_core_clk_en"
#define MEDIANOC_ATBCLK_EN                              "medianoc_atbclk_en"
#define DBNOC_ATBCLK_EN                                 "dbnoc_atbclk_en"
#define GPUNOC_ATBCLK_EN                                "gpunoc_atbclk_en"
#define PCIENOC_ATBCLK_EN                               "pcienoc_atbclk_en"
#define CORENOC_ATBCLK_EN                               "corenoc_atbclk_en"
#define TOPISTCTRL_TO_TOPCRM_PCLK_EN                    "topistctrl_to_topcrm_pclk_en"
#define LB_USB_0_U20_PHY_REF_CLK_EN                     "lb_usb_0_u20_phy_ref_clk_en"
#define LB_USB_1_U20_PHY_REF_CLK_EN                     "lb_usb_1_u20_phy_ref_clk_en"
#define LB_USB_0_U31_PHY_REF_CLK_EN                     "lb_usb_0_u31_phy_ref_clk_en"
#define LB_USB_1_U31_PHY_REF_CLK_EN                     "lb_usb_1_u31_phy_ref_clk_en"
#define LB_SOC_LSP0_FLEXRAY_HCLK_EN                     "lb_soc_lsp0_flexray_hclk_en"
#define LB_SOC_LSP1_FLEXRAY_HCLK_EN                     "lb_soc_lsp1_flexray_hclk_en"
#define CORENOC_APB_BR_PCLK_EN                          "corenoc_apb_br_pclk_en"







#define LB_SOC_LSP_AUDIO0_WCLK     "lb_soc_lsp_audio0_wclk"
#define LB_SOC_LSP_AUDIO1_WCLK     "lb_soc_lsp_audio1_wclk"
#define LB_SOC_LSP_I2S_S0_SCK     "lb_soc_lsp_i2s_s0_sck"
#define LB_SOC_LSP_I3C0_SCL     "lb_soc_lsp_i3c0_scl"
#define LB_SOC_LSP_I3C1_SCL     "lb_soc_lsp_i3c1_scl"
#define LB_SOC_LSP_FLEXRAY_HCLK     "lb_soc_lsp_flexray_hclk"
#define LB_SOC_LSP1_IST_ATSPEED_25M_CLK     "lb_soc_lsp1_ist_atspeed_25m_clk"
#define LB_SOC_LSP_PCM_TDM0_SCLK_IN     "lb_soc_lsp_pcm_tdm0_sclk_in"
#define LB_SOC_LSP_PCM_TDM1_SCLK_IN     "lb_soc_lsp_pcm_tdm1_sclk_in"
#define LSP0_WCLK_CLK_DIV_2     "lsp0_wclk_clk_div_2"
#define LSP0_WCLK_CLK_DIV_4     "lsp0_wclk_clk_div_4"
#define LSP1_WCLK_CLK_DIV_2     "lsp1_wclk_clk_div_2"
#define LSP1_WCLK_CLK_DIV_4     "lsp1_wclk_clk_div_4"

#define DIVIDOR_LSP0_STA_DIV_GPIO_DBCLK_DIV     "lsp0_sta_div_gpio_dbclk_div"
#define DIVIDOR_LSP1_STA_DIV_GPIO_DBCLK_DIV     "lsp1_sta_div_gpio_dbclk_div"
#define DIVIDOR_LSP0_STA_DIV_I3C_WCLK_DIV     "lsp0_sta_div_i3c_wclk_div"
#define DIVIDOR_LSP1_STA_DIV_I3C_WCLK_DIV     "lsp1_sta_div_i3c_wclk_div"

#define DIVIDOR_LSP0_RAY_CLK_INT_DIV_SINGLE_EDGE     "ray_clk_int_div_single_edge"
#define DIVIDOR_LSP0_I2S_S0_MCLK_OUT     "lsp0_i2s_s0_mclk_out"
#define DIVIDOR_LSP0_I2S_M0_WCLK_DIV     "lsp0_i2s_m0_wclk_div"
#define DIVIDOR_LSP0_I2S_S1_MCLK_OUT     "lsp0_i2s_s1_mclk_out"
#define DIVIDOR_LSP0_I2S_M1_WCLK_DIV     "lsp0_i2s_m1_wclk_div"
#define DIVIDOR_LSP0_TDM0_WCLK_DIV     	 "lsp0_tdm0_wclk_div"
#define DIVIDOR_LSP0_TDM1_WCLK_DIV       "lsp0_tdm1_wclk_div"
#define DIVIDOR_LSP0_SPDIF0_WCLK_DIV     "lsp0_spdif0_wclk_div"
#define DIVIDOR_LSP0_SPDIF1_WCLK_DIV     "lsp0_spdif1_wclk_div"

#define DIVIDOR_LSP1_RAY_CLK_INT_DIV_SINGLE_EDGE     "ray_clk_int_div_single_edge"
#define DIVIDOR_LSP1_I2S_S0_MCLK_OUT     "lsp1_i2s_s0_mclk_out"
#define DIVIDOR_LSP1_I2S_M0_WCLK_DIV     "lsp1_i2s_m0_wclk_div"
#define DIVIDOR_LSP1_I2S_S1_MCLK_OUT     "lsp1_i2s_s1_mclk_out"
#define DIVIDOR_LSP1_I2S_M1_WCLK_DIV     "lsp1_i2s_m1_wclk_div"
#define DIVIDOR_LSP1_TDM0_WCLK_DIV       "lsp1_tdm0_wclk_div"
#define DIVIDOR_LSP1_TDM1_WCLK_DIV       "lsp1_tdm1_wclk_div"

#define DIVIDOR_LSP1_SPDIF0_WCLK_DIV     "lsp1_spdif0_wclk_div"
#define DIVIDOR_LSP1_SPDIF1_WCLK_DIV     "lsp1_spdif1_wclk_div"



// #define MUX_LSP0_I3C0_IST_CLK_MUX2                 "i3c0_ist_clk_mux2"
// #define MUX_LSP0_I3C1_IST_CLK_MUX2                 "i3c1_ist_clk_mux2"
// #define MUX_LSP0_TDM0_IST_CLK_MUX2                 "tdm0_ist_clk_mux2"
// #define MUX_LSP0_TDM1_IST_CLK_MUX2                 "tdm1_ist_clk_mux2"
#define MUX_LSP0_AUDIO_WCLK_4                      "lsp0_audio_wclk_4"
#define MUX_LSP0_AUDIO_WCLK_5                      "lsp0_audio_wclk_5"
#define MUX_LSP0_AUDIO_WCLK_0                      "lsp0_audio_wclk_0"
#define MUX_LSP0_AUDIO_WCLK_1                      "lsp0_audio_wclk_1"
#define MUX_LSP0_AUDIO_WCLK_2                      "lsp0_audio_wclk_2"
#define MUX_LSP0_AUDIO_WCLK_3                      "lsp0_audio_wclk_3"
#define MUX_LSP0_UART_WCLK_MUX                     "lsp0_uart_wclk_mux"

// #define MUX_LSP1_I3C0_IST_CLK_MUX2                 "i3c0_ist_clk_mux2"
// #define MUX_LSP1_I3C1_IST_CLK_MUX2                 "i3c1_ist_clk_mux2"
// #define MUX_LSP1_TDM0_IST_CLK_MUX2                 "tdm0_ist_clk_mux2"
// #define MUX_LSP1_TDM1_IST_CLK_MUX2                 "tdm1_ist_clk_mux2"
#define MUX_LSP1_AUDIO_WCLK_4                      "lsp1_audio_wclk_4"
#define MUX_LSP1_AUDIO_WCLK_5                      "lsp1_audio_wclk_5"
#define MUX_LSP1_AUDIO_WCLK_0                      "lsp1_audio_wclk_0"
#define MUX_LSP1_AUDIO_WCLK_1                      "lsp1_audio_wclk_1"
#define MUX_LSP1_AUDIO_WCLK_2                      "lsp1_audio_wclk_2"
#define MUX_LSP1_AUDIO_WCLK_3                      "lsp1_audio_wclk_3"
#define MUX_LSP1_UART_WCLK_MUX                     "lsp1_uart_wclk_mux"

#define MUX_LSP0_LSP_WCLK_MUX                      "lsp0_wclk_mux"
#define MUX_LSP1_LSP_WCLK_MUX                      "lsp1_wclk_mux"




#define GATE_LSP0_I2C0_SMBUS_WCLK                 "lsp0_i2c0_smbus_wclk"
#define GATE_LSP0_I2C0_SMBUS_PCLK                 "lsp0_i2c0_smbus_pclk"
#define GATE_LSP1_I2C0_SMBUS_WCLK                 "lsp1_i2c0_smbus_wclk"
#define GATE_LSP1_I2C0_SMBUS_PCLK                 "lsp1_i2c0_smbus_pclk"

#define GATE_LSP0_I2C1_SMBUS_WCLK                 "lsp0_i2c1_smbus_wclk"
#define GATE_LSP0_I2C1_SMBUS_PCLK                 "lsp0_i2c1_smbus_pclk"
#define GATE_LSP1_I2C1_SMBUS_WCLK                 "lsp1_i2c1_smbus_wclk"
#define GATE_LSP1_I2C1_SMBUS_PCLK                 "lsp1_i2c1_smbus_pclk"

#define GATE_LSP0_I2C0_WCLK                 "lsp0_i2c0_wclk"
#define GATE_LSP0_I2C0_PCLK                 "lsp0_i2c0_pclk"
#define GATE_LSP1_I2C0_WCLK                 "lsp1_i2c0_wclk"
#define GATE_LSP1_I2C0_PCLK                 "lsp1_i2c0_pclk"

#define GATE_LSP0_I2C1_WCLK                 "lsp0_i2c1_wclk"
#define GATE_LSP0_I2C1_PCLK                 "lsp0_i2c1_pclk"
#define GATE_LSP1_I2C1_WCLK                 "lsp1_i2c1_wclk"
#define GATE_LSP1_I2C1_PCLK                 "lsp1_i2c1_pclk"

#define GATE_LSP0_I2C2_WCLK                 "lsp0_i2c2_wclk"
#define GATE_LSP0_I2C2_PCLK                 "lsp0_i2c2_pclk"
#define GATE_LSP1_I2C2_WCLK                 "lsp1_i2c2_wclk"
#define GATE_LSP1_I2C2_PCLK                 "lsp1_i2c2_pclk"


#define GATE_LSP0_I2C3_WCLK                 "lsp0_i2c3_wclk"
#define GATE_LSP0_I2C3_PCLK                 "lsp0_i2c3_pclk"
#define GATE_LSP1_I2C3_WCLK                 "lsp1_i2c3_wclk"
#define GATE_LSP1_I2C3_PCLK                 "lsp1_i2c3_pclk"


#define GATE_LSP0_UART0_WCLK                 "lsp0_uart0_wclk"
#define GATE_LSP0_UART0_PCLK                 "lsp0_uart0_pclk"
#define GATE_LSP1_UART0_WCLK                 "lsp1_uart0_wclk"
#define GATE_LSP1_UART0_PCLK                 "lsp1_uart0_pclk"

#define GATE_LSP0_UART1_WCLK                 "lsp0_uart1_wclk"
#define GATE_LSP0_UART1_PCLK                 "lsp0_uart1_pclk"
#define GATE_LSP1_UART1_WCLK                 "lsp1_uart1_wclk"
#define GATE_LSP1_UART1_PCLK                 "lsp1_uart1_pclk"

#define GATE_LSP0_SSI_M_WCLK                 "lsp0_ssi_m_wclk"
#define GATE_LSP0_SSI_M_PCLK                 "lsp0_ssi_m_pclk"
#define GATE_LSP1_SSI_M_WCLK                 "lsp1_ssi_m_wclk"
#define GATE_LSP1_SSI_M_PCLK                 "lsp1_ssi_m_pclk"

#define GATE_LSP0_SSI_S_WCLK                 "lsp0_ssi_s_wclk"
#define GATE_LSP0_SSI_S_PCLK                 "lsp0_ssi_s_pclk"
#define GATE_LSP1_SSI_S_WCLK                 "lsp1_ssi_s_wclk"
#define GATE_LSP1_SSI_S_PCLK                 "lsp1_ssi_s_pclk"


#define GATE_LSP0_GPIO0_DBCLK                "lsp0_gpio0_dbclk"
#define GATE_LSP0_GPIO0_PCLK                 "lsp0_gpio0_pclk"
#define GATE_LSP0_GPIO1_DBCLK                "lsp0_gpio1_dbclk"
#define GATE_LSP0_GPIO1_PCLK                 "lsp0_gpio1_pclk"
#define GATE_LSP0_GPIO2_DBCLK                "lsp0_gpio2_dbclk"
#define GATE_LSP0_GPIO2_PCLK                 "lsp0_gpio2_pclk"
#define GATE_LSP0_GPIO3_DBCLK                "lsp0_gpio3_dbclk"
#define GATE_LSP0_GPIO3_PCLK                 "lsp0_gpio3_pclk"

#define GATE_LSP1_GPIO0_DBCLK                "lsp1_gpio0_dbclk"
#define GATE_LSP1_GPIO0_PCLK                 "lsp1_gpio0_pclk"
#define GATE_LSP1_GPIO1_DBCLK                "lsp1_gpio1_dbclk"
#define GATE_LSP1_GPIO1_PCLK                 "lsp1_gpio1_pclk"
#define GATE_LSP1_GPIO2_DBCLK                "lsp1_gpio2_dbclk"
#define GATE_LSP1_GPIO2_PCLK                 "lsp1_gpio2_pclk"
#define GATE_LSP1_GPIO3_DBCLK                "lsp1_gpio3_dbclk"
#define GATE_LSP1_GPIO3_PCLK                 "lsp1_gpio3_pclk"


#define GATE_LSP0_WDT0_WCLK                 "lsp0_wdt0_wclk"
#define GATE_LSP0_WDT0_PCLK                 "lsp0_wdt0_pclk"
#define GATE_LSP0_WDT1_WCLK                 "lsp0_wdt1_wclk"
#define GATE_LSP0_WDT1_PCLK                 "lsp0_wdt1_pclk"


#define GATE_LSP1_WDT0_WCLK                 "lsp1_wdt0_wclk"
#define GATE_LSP1_WDT0_PCLK                 "lsp1_wdt0_pclk"
#define GATE_LSP1_WDT1_WCLK                 "lsp1_wdt1_wclk"
#define GATE_LSP1_WDT1_PCLK                 "lsp1_wdt1_pclk"



#define GATE_LSP0_TIMER0_WCLK                 "lsp0_timer0_wclk"
#define GATE_LSP0_TIMER1_WCLK                 "lsp0_timer1_wclk"
#define GATE_LSP0_TIMER2_WCLK                 "lsp0_timer2_wclk"
#define GATE_LSP0_TIMER3_WCLK                 "lsp0_timer3_wclk"
#define GATE_LSP0_TIMER4_WCLK                 "lsp0_timer4_wclk"
#define GATE_LSP0_TIMER5_WCLK                 "lsp0_timer5_wclk"
#define GATE_LSP0_TIMER6_WCLK                 "lsp0_timer6_wclk"
#define GATE_LSP0_TIMER7_WCLK                 "lsp0_timer7_wclk"
#define GATE_LSP0_TIMER_PCLK                  "lsp0_timer_pclk"


#define GATE_LSP1_TIMER0_WCLK                 "lsp1_timer0_wclk"
#define GATE_LSP1_TIMER1_WCLK                 "lsp1_timer1_wclk"
#define GATE_LSP1_TIMER2_WCLK                 "lsp1_timer2_wclk"
#define GATE_LSP1_TIMER3_WCLK                 "lsp1_timer3_wclk"
#define GATE_LSP1_TIMER4_WCLK                 "lsp1_timer4_wclk"
#define GATE_LSP1_TIMER5_WCLK                 "lsp1_timer5_wclk"
#define GATE_LSP1_TIMER6_WCLK                 "lsp1_timer6_wclk"
#define GATE_LSP1_TIMER7_WCLK                 "lsp1_timer7_wclk"
#define GATE_LSP1_TIMER_PCLK                  "lsp1_timer_pclk"



#define GATE_LSP0_I2S0_WCLK                 "lsp0_i2s0_wclk"
#define GATE_LSP0_I2S0_PCLK                 "lsp0_i2s0_pclk"
#define GATE_LSP0_I2S1_WCLK                 "lsp0_i2s1_wclk"
#define GATE_LSP0_I2S1_PCLK                 "lsp0_i2s1_pclk"

#define GATE_LSP1_I2S0_WCLK                 "lsp1_i2s0_wclk"
#define GATE_LSP1_I2S0_PCLK                 "lsp1_i2s0_pclk"
#define GATE_LSP1_I2S1_WCLK                 "lsp1_i2s1_wclk"
#define GATE_LSP1_I2S1_PCLK                 "lsp1_i2s1_pclk"


#define GATE_LSP0_I3C0_WCLK                 "lsp0_i3c0_wclk"
#define GATE_LSP0_I3C0_PCLK                 "lsp0_i3c0_pclk"
#define GATE_LSP0_I3C1_WCLK                 "lsp0_i3c1_wclk"
#define GATE_LSP0_I3C1_PCLK                 "lsp0_i3c1_pclk"



#define GATE_LSP1_I3C0_WCLK                 "lsp1_i3c0_wclk"
#define GATE_LSP1_I3C0_PCLK                 "lsp1_i3c0_pclk"
#define GATE_LSP1_I3C1_WCLK                 "lsp1_i3c1_wclk"
#define GATE_LSP1_I3C1_PCLK                 "lsp1_i3c1_pclk"



#define GATE_LSP0_TDM0_WCLK                 "lsp0_tdm0_wclk"
#define GATE_LSP0_TDM0_PCLK                 "lsp0_tdm0_pclk"
#define GATE_LSP0_TDM1_WCLK                 "lsp0_tdm1_wclk"
#define GATE_LSP0_TDM1_PCLK                 "lsp0_tdm1_pclk"

#define GATE_LSP1_TDM0_WCLK                 "lsp1_tdm0_wclk"
#define GATE_LSP1_TDM0_PCLK                 "lsp1_tdm0_pclk"
#define GATE_LSP1_TDM1_WCLK                 "lsp1_tdm1_wclk"
#define GATE_LSP1_TDM1_PCLK                 "lsp1_tdm1_pclk"

#define GATE_LSP0_SPDIF0_WCLK                 "lsp0_spdif0_wclk"
#define GATE_LSP0_SPDIF1_WCLK                 "lsp0_spdif1_wclk"
#define GATE_LSP1_SPDIF0_WCLK                 "lsp1_spdif0_wclk"
#define GATE_LSP1_SPDIF1_WCLK                 "lsp1_spdif1_wclk"


#define GATE_LSP0_PWM_WCLK                     "lsp0_pwm_wclk"
#define GATE_LSP1_PWM_WCLK                     "lsp1_pwm_wclk"


#define U20_PHY                         "u20_ref_phy_clk"
#define LSP0_FLEXRAY_HCLK                "lsp0_flexray_hclk"
#define LSP1_FLEXRAY_HCLK                "lsp1_flexray_hclk"
#define DISPLAY0_CH0                        "clk_display0_ch0_594"
#define DISPLAY0_CH1                        "clk_display0_ch1_594"
#define DISPLAY1_CH0                        "clk_display1_ch0_594"
#define DISPLAY1_CH1                        "clk_display1_ch1_594"
#define DISPLAY2_CH0                        "clk_display2_ch0_594"
#define UFS_26                            "ref_alt_clk_26m" 


#define LB_SW_XGMAC_WCLK_NAME    "lb_sw_xgmac_wclk"
#define LB_SW_XGMAC_PTP_CLK_NAME     "lb_sw_xgmac_ptp_clk"

#define GATE_XGMAC_PTP_CLK_EN_NAME                 "lb_sw_xgmac_wclk_en"
#define GATE_XGMAC_WCLK_EN_NAME                 "lb_sw_xgmac_ptp_clk_en"

#define GATE_CPU0_MP4_DSU_CHI_NAME                 "lb_cpu0_mp4_dsu_clk_en"
#define GATE_CPU0_MP4_CORE_NAME                 "lb_cpu0_mp4_core_clk_en"




static const char *const cpu_sel_parents[] = { CLK_25M_OSC, PLL_CPU};
static const char *const gpu_sel_parents[] = { CLK_25M_OSC, PLL_GPU};
static const char *const cpu_dsu_sel_parents[] = { CLK_25M_OSC, FACTOR_CPU_DSU_NAME};
static const char *const cmn_sel_parents[] = { CLK_25M_OSC, PLL_CMN};
static const char *const display0_sel_parents[] = { CLK_25M_OSC, PLL_DISPLAY0 };
static const char *const display1_sel_parents[] = { CLK_25M_OSC, PLL_DISPLAY1 };
static const char *const display2_sel_parents[] = { CLK_25M_OSC, PLL_DISPLAY2 };
static const char *const display3_sel_parents[] = { CLK_25M_OSC, PLL_DISPLAY3 };
static const char *const sysbus0_sel_parents[] = { CLK_25M_OSC, PLL_SYSBUS0};
static const char *const sysbus1_sel_parents[] = { CLK_25M_OSC, PLL_SYSBUS1};
static const char *const net_sel_parents[] = { CLK_25M_OSC, PLL_NET};
static const char *const ufs_sel_parents[] = { CLK_25M_OSC, PLL_UFS};

static const char *const display0_mux_parents[] = { MUX_DISPLAY0, MUX_DISPLAY1,MUX_DISPLAY2,MUX_DISPLAY3};
static const char *const display1_mux_parents[] = { MUX_DISPLAY0, MUX_DISPLAY1,MUX_DISPLAY2,MUX_DISPLAY3};
static const char *const display2_mux_parents[] = { MUX_DISPLAY0, MUX_DISPLAY1,MUX_DISPLAY2,MUX_DISPLAY3};
static const char *const display3_mux_parents[] = { MUX_DISPLAY0, MUX_DISPLAY1,MUX_DISPLAY2,MUX_DISPLAY3};
static const char *const display4_mux_parents[] = { MUX_DISPLAY0, MUX_DISPLAY1,MUX_DISPLAY2,MUX_DISPLAY3};



static const char *const hifi_dsp_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const cv_core_clk_mux_parents[] = { FACTOR_SYSBUS1_1200, FACTOR_SYSBUS0_1000,FACTOR_SYSBUS0_500,FACTOR_SYSBUS0_200};
static const char *const safetynoc_to_socnic_m_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};
static const char *const xgmac_axi_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};
static const char *const sysnoc_to_socnic_ahb_cfg_hclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100,FACTOR_SYSBUS0_50};


static const char *const g78ae_sp_wclk_mux_parents[] = { CLK_25M_OSC, MUX_GPU,FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_400};
static const char *const hifi_dsp_xnne_wclk_mux_parents[] = { FACTOR_SYSBUS0_1000, FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const cv_axim1_clk_mux_parents[] = { FACTOR_SYSBUS1_800, FACTOR_SYSBUS0_1000,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const corenoc_to_socnic_m_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};
static const char *const xgmac_apb_pclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const soc_sec_hclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100,FACTOR_SYSBUS0_50};




static const char *const g78ae_wclk_mux_parents[] = { CLK_25M_OSC, MUX_GPU,FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400};
static const char *const hifi_dsp_wclk_mux_parents[] = { FACTOR_SYSBUS0_1000, FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const cv_axim0_clk_mux_parents[] = { FACTOR_SYSBUS1_800, FACTOR_SYSBUS0_1000,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const secnoc_to_socnic_m_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};
static const char *const probe_axi_aclk_mux_parents[] = { FACTOR_SYSBUS1_600, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};
static const char *const soc_sec_aclk_mux_parents[] = { FACTOR_SYSBUS1_600, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};



static const char *const g78ae_ace_aclk_mux_parents[] = { CLK_25M_OSC, MUX_GPU,FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400};
static const char *const clk_lvds1_hspeed_mux_parents[] = { MUX_DISPLAY0, MUX_DISPLAY1,MUX_DISPLAY2,MUX_DISPLAY3};
static const char *const cv_axis_clk_mux_parents[] = { FACTOR_SYSBUS0_1000, MUX_GPU,FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400};
static const char *const secnoc_to_rtnoc_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};
static const char *const atb_tpiu_trace_clk_mux_parents[] = { FACTOR_SYSBUS1_150, CLK_25M_OSC,FACTOR_SYSBUS0_100,FACTOR_SYSBUS0_50};
static const char *const edp_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};


static const char *const g78ae_axis_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS1_600,FACTOR_SYSBUS0_200};
static const char *const clk_lvds0_hspeed_mux_parents[] = { MUX_DISPLAY0, MUX_DISPLAY1,MUX_DISPLAY2,MUX_DISPLAY3};
static const char *const isp_sclk_mux_parents[] = { FACTOR_SYSBUS1_800, FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const rtnoc_to_socnic_m_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};
static const char *const atb_axi_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const dbsocnoc_to_cmn_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};


static const char *const g78ae_tcu_wclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS1_600,FACTOR_SYSBUS0_200};
static const char *const clk_cpu_1188_mux_parents[] = { MUX_DISPLAY0, MUX_DISPLAY1,MUX_DISPLAY2,MUX_DISPLAY3};
static const char *const swnoc_to_sysnoc_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const dbnoc_to_dbsocnoc_m_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const smmu_tcu_codec_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};
static const char *const dbsocnoc_to_sysnoc_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};

static const char *const sys_noc_800m_wclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const safenoc_to_swnoc_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};
static const char *const socnic_s_to_dbnoc_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};
static const char *const smmu_tcu_coreip_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};
static const char *const sysnoc_to_dbsocnoc_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};

static const char *const sys_noc_400m_wclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};
static const char *const soc_lsp1_wclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};
static const char *const sdnic_to_sysnoc_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};
static const char *const cpu_perip_clk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_600,FACTOR_SYSBUS0_500,FACTOR_SYSBUS1_400};
static const char *const dbsocnoc_wclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};



static const char *const media_noc_wclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS0_1000,FACTOR_SYSBUS0_500,FACTOR_SYSBUS0_250};
static const char *const soc_lsp0_wclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};
static const char *const usbnic_to_sysnoc_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};
static const char *const rtnoc_swnoc_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};
static const char *const msgbx_switch1_to_safe_hclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};

static const char *const core_noc_wclk_mux_parents[] = { FACTOR_SYSBUS0_1000, MUX_NET,FACTOR_SYSBUS1_600,FACTOR_SYSBUS0_200};
static const char *const ufs_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100,FACTOR_SYSBUS0_50};
static const char *const lpddr5_1_s4_aclk_mux_parents[] = { FACTOR_SYSBUS0_1000, FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400};
static const char *const rtnoc_safenoc_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};
static const char *const msgbx_switch1_to_secure_hclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};



static const char *const db_noc_wclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const sdemmc1_hclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100,FACTOR_SYSBUS0_50};
static const char *const lpddr5_1_s3_aclk_mux_parents[] = { FACTOR_SYSBUS0_1000, FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400};
static const char *const sysnoc_to_dbnoc_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};
static const char *const msgbx_switch1_to_realtime_hclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};

static const char *const cmn_wclk_mux_parents[] = { MUX_UFS, MUX_CMN,FACTOR_SYSBUS0_1000,FACTOR_SYSBUS1_600};
static const char *const cs_dma_hclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100,FACTOR_SYSBUS0_50};
static const char *const sdemmc1_w_bclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_800,FACTOR_SYSBUS0_666,FACTOR_SYSBUS1_600};
static const char *const lpddr5_1_s2_aclk_mux_parents[] = { FACTOR_SYSBUS0_1000, FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400};
static const char *const soc_sram_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};
static const char *const msgbx_switch0_to_switch1_wclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const cmn_to_sysnoc_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};



static const char *const clk_200_sysbus_apb_mux_parents[] =  { CLK_25M_OSC, FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100,FACTOR_SYSBUS0_50};
static const char *const media_dma_hclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS0_125,FACTOR_SYSBUS0_100,FACTOR_SYSBUS0_50};
static const char *const sdemmc0_hclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100,FACTOR_SYSBUS0_50};
static const char *const lpddr5_1_s1_aclk_mux_parents[] ={ FACTOR_SYSBUS0_1000, FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400};
static const char *const sysnoc_to_cmn_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const msgbx_switch1_to_media_hclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};

static const char *const clk_100_sysbus_apb_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS0_100,FACTOR_SYSBUS1_75,FACTOR_SYSBUS0_50};
static const char *const soc_dma_hclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100,FACTOR_SYSBUS0_50};
static const char *const sdemmc0_w_bclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_800,FACTOR_SYSBUS0_666,FACTOR_SYSBUS1_600};
static const char *const lpddr5_1_s0_aclk_mux_parents[] = { FACTOR_SYSBUS0_1000, FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400};
static const char *const corenoc_to_cmn_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const msgbx_switch1_to_sw_hclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const noc_atb_clk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100,FACTOR_SYSBUS0_50};






static const char *const cpu0_mp4_dsu_chi_clk_mux_parents[] =  { CLK_25M_OSC, MUX_UFS,MUX_CPU_DSU,FACTOR_SYSBUS0_500};
static const char *const db_dma_hclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100,FACTOR_SYSBUS0_50};
static const char *const usb_1_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};
static const char *const lpddr5_0_s4_aclk_mux_parents[] ={ FACTOR_SYSBUS0_1000, FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400};
static const char *const medianoc_to_cmn_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const msgbx_switch1_wclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const soc_dma_core_clk_parents[] =  { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};



static const char *const cpu0_mp4_core_clk_mux_parents[] = { CLK_25M_OSC, MUX_CPU,FACTOR_SYSBUS1_1200,FACTOR_SYSBUS0_500};
static const char *const cs_dma_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const usb_0_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};
static const char *const lpddr5_0_s3_aclk_mux_parents[] = { FACTOR_SYSBUS0_1000, FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400};
static const char *const socnic_s_to_medianoc_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};
static const char *const msgbx_switch0_to_socdma_hclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const flexraynic_to_sysnoc_hclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100,FACTOR_SYSBUS0_50};





static const char *const cpu1_mp4_dsu_chi_clk_mux_parents[] =  { CLK_25M_OSC, MUX_UFS,MUX_CPU_DSU,FACTOR_SYSBUS0_500};
static const char *const media_dma_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const pcie_aclk_mux_parents[] =  { CLK_25M_OSC, FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const lpddr5_0_s2_aclk_mux_parents[] ={ FACTOR_SYSBUS0_1000, FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400};
static const char *const socnic_s_to_corenoc_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};
static const char *const msgbx_switch0_to_net_hclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const dsi_csitx_ipi_clk_mux_parents[] =  { CLK_25M_OSC, FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};



static const char *const cpu1_mp4_core_clk_mux_parents[] = { CLK_25M_OSC, MUX_CPU,FACTOR_SYSBUS1_1200,FACTOR_SYSBUS0_500};
static const char *const soc_dma_aclk_mux_parents[] =  { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};
static const char *const pcie_dbi_aclk_mux_parents[] =  { CLK_25M_OSC, FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100,FACTOR_SYSBUS0_50};
static const char *const lpddr5_0_s1_aclk_mux_parents[] = { FACTOR_SYSBUS0_1000, FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400};
static const char *const socnic_s_to_safenoc_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};
static const char *const msgbx_switch0_to_cpu_hclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const dsi1_clkext_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};




static const char *const cpu_mp2_mp_aclk_mux_parents[] =  { CLK_25M_OSC, FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const db_dma_aclk_mux_parents[] =  { CLK_25M_OSC, FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const display2_aclk_mux_parents[] = { FACTOR_SYSBUS0_1000, FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400};
static const char *const lpddr5_0_s0_aclk_mux_parents[] = { FACTOR_SYSBUS0_1000, FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400};
static const char *const socnic_s_to_swnoc_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};
static const char *const msgbx_switch0_to_db_hclk_mux_parents[] =  { CLK_25M_OSC, FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const dsi0_clkext_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};



static const char *const cpu_mp2_master_aclk_mux_parents[] =  { FACTOR_SYSBUS1_1200, FACTOR_SYSBUS0_1000,FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400};
static const char *const codec1_wclk_mux_parents[] =  { CLK_25M_OSC, FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const display1_aclk_mux_parents[] = { FACTOR_SYSBUS0_1000, FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400};
static const char *const dsp_clk_mux_parents[] = { FACTOR_SYSBUS1_800,FACTOR_SYSBUS0_1000,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const sysnoc_to_socnic_s_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};
static const char *const msgbx_switch0_to_ispcv_hclk_mux_parents[] =  { CLK_25M_OSC, FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const medianoc_to_sysnoc_aclk_mux_parents[] ={ CLK_25M_OSC, FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100,FACTOR_SYSBUS0_50};


static const char *const cpu_mp2_dsu_clk_mux_parents[] =  { FACTOR_SYSBUS1_1200, FACTOR_SYSBUS0_1000,FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400};
static const char *const codec0_wclk_mux_parents[] =  { CLK_25M_OSC, FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const display0_aclk_mux_parents[] = { FACTOR_SYSBUS0_1000, FACTOR_SYSBUS1_800,FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400};
static const char *const net_wclk_mux_parents[] = { FACTOR_SYSBUS0_1000,MUX_NET,FACTOR_SYSBUS1_600,FACTOR_SYSBUS0_200};
static const char *const socnic_m_to_sysnoc_aclk_mux_parents[] = { CLK_25M_OSC, FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const msgbx_switch0_wclk_mux_parents[] =  { CLK_25M_OSC, FACTOR_SYSBUS1_600,FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200};
static const char *const sysnoc_to_gpunic_aclk_mux_parents[] ={ CLK_25M_OSC, FACTOR_SYSBUS1_400,FACTOR_SYSBUS0_200,FACTOR_SYSBUS0_100};

static const char *const cpu_mp2_core_clk_mux_parents[] ={ CLK_25M_OSC, FACTOR_SYSBUS0_1000,FACTOR_SYSBUS1_800,FACTOR_SYSBUS0_500};


static const char *const bd_external_mux_clk0_mux_parents[] ={ CLK_25M_OSC, FACTOR_SYSBUS1_1200,UFS_26,MUX_DISPLAY0,MUX_DISPLAY1,MUX_DISPLAY2,MUX_DISPLAY3,CLK_25M_OSC};
static const char *const bd_external_mux_clk1_mux_parents[] ={ CLK_25M_OSC, FACTOR_SYSBUS1_1200,UFS_26,MUX_DISPLAY0,MUX_DISPLAY1,MUX_DISPLAY2,MUX_DISPLAY3,CLK_25M_OSC};
static const char *const bd_external_mux_clk2_mux_parents[] ={ CLK_25M_OSC, FACTOR_SYSBUS1_1200,UFS_26,MUX_DISPLAY0,MUX_DISPLAY1,MUX_DISPLAY2,MUX_DISPLAY3,CLK_25M_OSC};
static const char *const bd_external_mux_clk3_mux_parents[] ={ CLK_25M_OSC, FACTOR_SYSBUS1_1200,UFS_26,MUX_DISPLAY0,MUX_DISPLAY1,MUX_DISPLAY2,MUX_DISPLAY3,CLK_25M_OSC};
static const char *const bd_external_mux_clk4_mux_parents[] ={ CLK_25M_OSC, FACTOR_SYSBUS1_1200,UFS_26,MUX_DISPLAY0,MUX_DISPLAY1,MUX_DISPLAY2,MUX_DISPLAY3,CLK_25M_OSC};
static const char *const bd_external_mux_clk5_mux_parents[] ={ CLK_25M_OSC, FACTOR_SYSBUS1_1200,UFS_26,MUX_DISPLAY0,MUX_DISPLAY1,MUX_DISPLAY2,MUX_DISPLAY3,CLK_25M_OSC};
static const char *const bd_external_mux_clk6_mux_parents[] ={ CLK_25M_OSC, FACTOR_SYSBUS1_1200,UFS_26,MUX_DISPLAY0,MUX_DISPLAY1,MUX_DISPLAY2,MUX_DISPLAY3,CLK_25M_OSC};
static const char *const bd_external_mux_clk7_mux_parents[] ={ CLK_25M_OSC, FACTOR_SYSBUS1_1200,UFS_26,MUX_DISPLAY0,MUX_DISPLAY1,MUX_DISPLAY2,MUX_DISPLAY3,CLK_25M_OSC};
static const char *const gtc_mux_parents[] ={ CLK_25M_OSC, FACTOR_SYSBUS1_1200,CLK_25M_OSC,CLK_25M_OSC,CLK_25M_OSC,CLK_25M_OSC,CLK_25M_OSC,FACTOR_SYSBUS0_200};


static const char *const dsi_cfg_ref_clk_mux_parents[] ={ CLK_25M_OSC, FACTOR_SYSBUS1_25};
static const char *const ufs_ref_alt_clk_26m_clk_mux_parents[] ={ UFS_26, CLK_26M_OSC};
static const char *const usb_u31_phy_ref_clk_clk_mux_parents[] ={ CLK_25M_OSC, FACTOR_SYSBUS1_25};
static const char *const usb_u20_phy_ref_clk_mux_parents[] ={ DIVIDOR_TOP_USB_U20_PHY_REF_CLK_NAME, CLK_24M_OSC};





static const char *const lsp0_audio_wclk_4_mux_parents[] ={ LB_SOC_LSP_AUDIO0_WCLK, LB_SOC_LSP_AUDIO1_WCLK};
static const char *const lsp0_audio_wclk_5_mux_parents[] ={ LB_SOC_LSP_AUDIO0_WCLK, LB_SOC_LSP_AUDIO1_WCLK};
static const char *const lsp0_audio_wclk_0_mux_parents[] ={ LB_SOC_LSP_AUDIO0_WCLK, LB_SOC_LSP_AUDIO1_WCLK};
static const char *const lsp0_audio_wclk_1_mux_parents[] ={ LB_SOC_LSP_AUDIO0_WCLK, LB_SOC_LSP_AUDIO1_WCLK};
static const char *const lsp0_audio_wclk_2_mux_parents[] ={ LB_SOC_LSP_AUDIO0_WCLK, LB_SOC_LSP_AUDIO1_WCLK};
static const char *const lsp0_audio_wclk_3_mux_parents[] ={ LB_SOC_LSP_AUDIO0_WCLK, LB_SOC_LSP_AUDIO1_WCLK};
static const char *const lsp0_uart_wclk_mux_mux_parents[] ={ LB_SOC_LSP0_UART_WCLK_EN, MUX_LSP0_LSP_WCLK_MUX};
static const char *const lsp0_wclk_mux_mux_parents[] ={ LSP0_WCLK_CLK_DIV_2, LSP0_WCLK_CLK_DIV_4};



static const char *const lsp1_audio_wclk_4_mux_parents[] ={ LB_SOC_LSP_AUDIO0_WCLK, LB_SOC_LSP_AUDIO1_WCLK};
static const char *const lsp1_audio_wclk_5_mux_parents[] ={ LB_SOC_LSP_AUDIO0_WCLK, LB_SOC_LSP_AUDIO1_WCLK};
static const char *const lsp1_audio_wclk_0_mux_parents[] ={ LB_SOC_LSP_AUDIO0_WCLK, LB_SOC_LSP_AUDIO1_WCLK};
static const char *const lsp1_audio_wclk_1_mux_parents[] ={ LB_SOC_LSP_AUDIO0_WCLK, LB_SOC_LSP_AUDIO1_WCLK};
static const char *const lsp1_audio_wclk_2_mux_parents[] ={ LB_SOC_LSP_AUDIO0_WCLK, LB_SOC_LSP_AUDIO1_WCLK};
static const char *const lsp1_audio_wclk_3_mux_parents[] ={ LB_SOC_LSP_AUDIO0_WCLK, LB_SOC_LSP_AUDIO1_WCLK};
static const char *const lsp1_uart_wclk_mux_mux_parents[] ={ LB_SOC_LSP1_UART_WCLK_EN, MUX_LSP1_LSP_WCLK_MUX};
static const char *const lsp1_wclk_mux_mux_parents[] ={ LSP1_WCLK_CLK_DIV_2, LSP1_WCLK_CLK_DIV_4};

//#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0])) 


// enum clock_type {
// 	CLOCK_TREE_PLL = 0,
// 	CLOCK_TREE_MUX = 1,
// 	CLOCK_TREE_FIX = 2,
// 	CLOCK_TREE_GATE = 3,
// 	CLOCK_TREE_FACTOR = 4,
// 	CLOCK_TREE_DIVIDER = 5,
// };


struct clock_fix {
	unsigned long long rate;
};


struct clock_factor {
	unsigned int mult;
	unsigned int div;
};




struct C1200_clock_info {
	char name[48];
	const char *const * mux_parent;
	unsigned char mux_parent_count;
	const char * parent;
	enum clock_type type;
	struct clock_fix fix;
	struct clock_factor factor;
};



#endif