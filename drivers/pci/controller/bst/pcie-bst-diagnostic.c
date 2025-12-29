/* SPDX-License-Identifier: GPL-2.0 */
/*
 * BST PCIe diagnostic
 *
 * Copyright (C) 2024 Black Sesame Technologies, Inc.
 *
 * Author: Gordon.Ge <gordon.geg@bst.ai>
 */
#ifdef CONFIG_PCIE_BST_DIAGNOSTIC
#include <linux/kthread.h>

#include "pcie-bst-diagnostic.h"
#include "pcie-bst.h"
#include "pcie-bst-phy.h"

// import health monitory interface to send/get psm id
extern int get_psmid_from_safety_lib(uint8_t block_id_in ,uint8_t *block_id_out, uint32_t *psm_id_out);
extern int send_dtc_to_safety_svc(u32 dtc);

static void bst_pcie_writel_dbi(struct dw_pcie *pci, u32 reg, u32 val)
{
	bst_pcie_write_dbi(pci, pci->dbi_base, reg, 0x4, val);
}

static u32 bst_pcie_readl_dbi(struct dw_pcie *pci, u32 reg)
{
	return bst_pcie_read_dbi(pci, pci->dbi_base, reg, 0x4);
}

static inline void event_counter_enable_all(struct bst_pcie *pcie, u32 opt)
{
	u32 val;
	val = bst_pcie_readl_dbi(pcie->pci, pcie->bst_pcie_diag->ras_des_cap + PCIE_RAS_DES_EVENT_COUNTER_CONTROL);
	val |= opt << EVENT_COUNTER_ENABLE_SHIFT;
	bst_pcie_writel_dbi(pcie->pci, pcie->bst_pcie_diag->ras_des_cap + PCIE_RAS_DES_EVENT_COUNTER_CONTROL, val);
}

static inline u32 event_counter_prog(struct bst_pcie *pcie, u32 lane,  u32 event, u32 group)
{
	u32 val;
	val = bst_pcie_readl_dbi(pcie->pci, pcie->bst_pcie_diag->ras_des_cap + PCIE_RAS_DES_EVENT_COUNTER_CONTROL);
	val &= ~(EVENT_COUNTER_LANE_SEL_MASK << EVENT_COUNTER_LANE_SEL_SHIFT);
	val &= ~(EVENT_COUNTER_GROUP_SEL_MASK << EVENT_COUNTER_GROUP_SEL_SHIFT);
	val &= ~(EVENT_COUNTER_EVENT_SEL_MASK << EVENT_COUNTER_EVENT_SEL_SHIFT);
	// lane select lane0 test
	val |= lane << EVENT_COUNTER_LANE_SEL_SHIFT;
	val |= group << EVENT_COUNTER_GROUP_SEL_SHIFT;
	val |= event << EVENT_COUNTER_EVENT_SEL_SHIFT;
	// val |= EVENT_COUNTER_PER_EVENT_ON << EVENT_COUNTER_ENABLE_SHIFT;
	bst_pcie_writel_dbi(pcie->pci, pcie->bst_pcie_diag->ras_des_cap + PCIE_RAS_DES_EVENT_COUNTER_CONTROL, val);
	val = bst_pcie_readl_dbi(pcie->pci, pcie->bst_pcie_diag->ras_des_cap + PCIE_RAS_DES_EVENT_COUNTER_DATA);

	return val;
}

/**
* bst_diag_rasdes_counter_monitor.
* @data:	struct bst_pcie
* PSM_CPU_PCIE_15
* PCIE Periodic Read of RASDES Counters
*/
static int bst_diag_rasdes_counter_monitor(void *data)
{
	struct bst_pcie* pcie = (struct bst_pcie*)data;
	u32 val = 0;
	u32 index = 0;
	u32 lane_index = 0;
	u32 lane = 2; // default is lane 2
	lane = pcie->ctrl_id ? 2 : 4;
	event_counter_enable_all(pcie, EVENT_COUNTER_ENABLE_ALL_ON);
	while(1)
	{
		// /* - GROUP0 EVENT - */
		// #define EVENT_COUNTER_EVENT_EBUF_OVERFLOW       		0x00
		// #define EVENT_COUNTER_EVENT_EBUF_UNDER_FUN       		0x01
		// #define EVENT_COUNTER_EVENT_DECODE_ERR      			0x02
		// #define EVENT_COUNTER_EVENT_RUNNING_DISPARITY_ERR   	0x03
		// #define EVENT_COUNTER_EVENT_SKP_OS_PARITY_ERR       	0x04
		// #define EVENT_COUNTER_EVENT_SYNC_HEADER_ERROR      		0x05
		// #define EVENT_COUNTER_EVENT_RX_VALID_DE_ASSERTION   	0x06
		// #define EVENT_COUNTER_EVENT_CTL_SKP_OS_PARITY_ERR   	0x07
		// #define EVENT_COUNTER_EVENT_1ST_RETIMER_PARITY_ERR  	0x08
		// #define EVENT_COUNTER_EVENT_2ND_RETIMER_PARITY_ERR      0x09
		// #define EVENT_COUNTER_EVENT_MARGIN_CRC_PARITY_ERR       0x0A
		for(index  = 0; index < 11; index++)
		{
			// group 0 per-lane 
			for(lane_index = 0; lane_index < lane; lane_index++)
			{
				val = event_counter_prog(pcie, lane_index, index, EVENT_COUNTER_GROUP_0);
				if(val > 0)
				{
					pr_err("RASDES Counter Group:[%d] event:[%d] lane:[%d] val:%d  DTC:[0x%x]\n", EVENT_COUNTER_GROUP_0, index, lane_index, val, PSM_ID_RASDES_COUNTER_DTC);
					// send dtc to 
					send_dtc_to_safety_svc(PSM_ID_RASDES_COUNTER_DTC);
				}
			}
		}
		#if 0 // only detect group 0 event
		// /* - GROUP1 EVENT - */
		// #define EVENT_COUNTER_EVENT_RESERVED                    0x00
		// #define EVENT_COUNTER_EVENT_RESERVED                    0x01
		// #define EVENT_COUNTER_EVENT_RESERVED                    0x02
		// #define EVENT_COUNTER_EVENT_RESERVED                    0x03
		// #define EVENT_COUNTER_EVENT_RESERVED                    0x04
		// #define EVENT_COUNTER_EVENT_DETECT_EI_INFER             0x05
		// #define EVENT_COUNTER_EVENT_RECEIVER_ERR                0x06
		// #define EVENT_COUNTER_EVENT_RX_RECOVERY_REQ             0x07
		// #define EVENT_COUNTER_EVENT_N_FTS_TIMEOUT               0x08
		// #define EVENT_COUNTER_EVENT_FRAMEING_ERR                0x09
		// #define EVENT_COUNTER_EVENT_DESKEW_ERR                  0x0a
		for(index  = 5; index < 11; index++)
		{
			// common-lane
			lane_index = 0;
			// for(lane_index = 0; lane_index < lane; lane_index++)
			{
				val = event_counter_prog(pcie, lane_index, index, EVENT_COUNTER_GROUP_1);
				if(val > 0)
				{
					// send dtc
					pr_err("RASDES Group:[%d] event:[%d] lane:[%d] val:%d DTC:[0x%x]\n", EVENT_COUNTER_GROUP_1, index, lane_index, val, PSM_ID_RASDES_COUNTER_DTC);
					// send dtc to 
					send_dtc_to_safety_svc(PSM_ID_RASDES_COUNTER_DTC);
				}
			}
		}

		// /* - GROUP2 EVENT - */
		// #define EVENT_COUNTER_EVENT_BAD_TLP                     0x00
		// #define EVENT_COUNTER_EVENT_LCRC_ERR                    0x01
		// #define EVENT_COUNTER_EVENT_BAD_DLLP                    0x02
		// #define EVENT_COUNTER_EVENT_REPLAT_NUM_ROLLOVER         0x03
		// #define EVENT_COUNTER_EVENT_REPLAY_TIMEOUT              0x04
		// #define EVENT_COUNTER_EVENT_RX_NAK_DLLP                 0x05
		// #define EVENT_COUNTER_EVENT_TX_NAK_DLLP                 0x06
		// #define EVENT_COUNTER_EVENT_RETRY_TLP                   0x07
		for(index  = 0; index < 8; index++)
		{
			// common-lane
			lane_index = 0;
			// for(lane_index = 0; lane_index < lane; lane_index++)
			{
				val = event_counter_prog(pcie, lane_index, index, EVENT_COUNTER_GROUP_2);
				if(val > 0)
				{
					// send dtc
					pr_err("RASDES Counter Group:[%d] event:[%d] lane:[%d] val:%d DTC:[0x%x]\n", EVENT_COUNTER_GROUP_2, index, lane_index, val, PSM_ID_RASDES_COUNTER_DTC);
					// send dtc to 
					send_dtc_to_safety_svc(PSM_ID_RASDES_COUNTER_DTC);
				}				
			}
		}

		/* - GROUP3 EVENT - */
		// #define EVENT_COUNTER_EVENT_FC_TIMEOUT                  0x00
		// #define EVENT_COUNTER_EVENT_POISONED_TLP                0x01
		// #define EVENT_COUNTER_EVENT_ECRC_ERR                    0x02
		// #define EVENT_COUNTER_EVENT_UNSUPPORTED_REQ             0x03
		// #define EVENT_COUNTER_EVENT_COMPLETER_ABORT             0x04
		// #define EVENT_COUNTER_EVENT_COMPLETETION_TIMEOUT        0x05
		for(index  = 0; index < 5; index++)
		{
			// common-lane
			lane_index = 0;
			// for(lane_index = 0; lane_index < lane; lane_index++)
			{
				val = event_counter_prog(pcie, lane_index, index, EVENT_COUNTER_GROUP_3);
				if(val > 0)
				{
					// send dtc
					pr_err("RASDES Counter Group:[%d] event:[%d] lane:[%d] val:%d DTC:[0x%x]\n", EVENT_COUNTER_GROUP_3, index, lane_index, val, PSM_ID_RASDES_COUNTER_DTC);
					// send dtc to 
					send_dtc_to_safety_svc(PSM_ID_RASDES_COUNTER_DTC);
				}				
			}
		}

		/* - GROUP4 EVENT - */
		// #define EVENT_COUNTER_EVENT_EBUF_SKP_ADD                0x00
		// #define EVENT_COUNTER_EVENT_EBUG_SKP_DEL                0x01
		for(index  = 0; index < 2; index++)
		{
			// i --> event ID
			for(lane_index = 0; lane_index < lane; lane_index++)
			{
				val = event_counter_prog(pcie, lane_index, index, EVENT_COUNTER_GROUP_4);
				if(val > 0)
				{
					pr_err("RASDES Counter Group:[%d] event:[%d] lane:[%d] val:%d DTC:[0x%x]\n", EVENT_COUNTER_GROUP_4, index, lane_index, val, PSM_ID_RASDES_COUNTER_DTC);
					// send dtc to 
					send_dtc_to_safety_svc(PSM_ID_RASDES_COUNTER_DTC);
				}			
			}
		}

		/* - GROUP5 EVENT - */
		// #define EVENT_COUNTER_EVENT_L0_TO_RECOVERY_ENTRY		0x0
		// #define EVENT_COUNTER_EVENT_L1_TO_RECOVERY_ENTRY	    0x1
		// #define EVENT_COUNTER_EVENT_Tx_L0S						0x2
		// #define EVENT_COUNTER_EVENT_Rx_L0S						0x3
		// #define EVENT_COUNTER_EVENT_ASPM_L1_REJECT              0x04
		// #define EVENT_COUNTER_EVENT_L1							0x5
		// #define EVENT_COUNTER_EVENT_L1_CPM  					0x06
		// #define EVENT_COUNTER_EVENT_L1_1						0x7
		// #define EVENT_COUNTER_EVENT_L1_2						0x8
		// #define EVENT_COUNTER_EVENT_L1_SHORT_DURATION   		0x09
		// #define EVENT_COUNTER_EVENT_L1_2_ABORT          		0x0A
		// #define EVENT_COUNTER_EVENT_L2_ENTRY           			0x0B
		// #define EVENT_COUNTER_EVENT_SPEED_CHANGE        		0x0C
		// #define EVENT_COUNTER_EVENT_LINK_WIDTH_CHANGE   		0x0D
		for(index  = 0; index < 15; index++)
		{
			// i --> event ID
			for(lane_index = 0; lane_index < lane; lane_index++)
			{
				val = event_counter_prog(pcie, lane_index, index, EVENT_COUNTER_GROUP_5);
				if(val > 0)
				{
					pr_err("RASDES Counter Group:[%d] event:[%d] lane:[%d] val:%d DTC:[0x%x]\n", EVENT_COUNTER_GROUP_5, index, lane_index, val, PSM_ID_RASDES_COUNTER_DTC);
					// send dtc to 
					send_dtc_to_safety_svc(PSM_ID_RASDES_COUNTER_DTC);
				}
			}
		}

		/* - GROUP6 EVENT - */
		// #define EVENT_COUNTER_EVENT_TX_ACK_DLLP                 0x00
		// #define EVENT_COUNTER_EVENT_TX_UPDATE_FC_DLLP           0x01
		// #define EVENT_COUNTER_EVENT_RX_ACK_DLLP                 0x02
		// #define EVENT_COUNTER_EVENT_RX_UPDATE_FC_DLLP           0x03
		// #define EVENT_COUNTER_EVENT_RX_NULLI_TLP                0x04
		// #define EVENT_COUNTER_EVENT_TX_NULLI_TLP        		   0x05
		// #define EVENT_COUNTER_EVENT_RX_DUP_TLP       		   0x06
		for(index  = 0; index < 7; index++)
		{
			// i --> event ID
			for(lane_index = 0; lane_index < lane; lane_index++)
			{
				val = event_counter_prog(pcie, lane_index, index, EVENT_COUNTER_GROUP_6);
				if(val > 0)
				{
					pr_err("RASDES Counter Group:[%d] event:[%d] lane:[%d] val:%d DTC:[0x%x]\n", EVENT_COUNTER_GROUP_6, index, lane_index, val, PSM_ID_RASDES_COUNTER_DTC);
					// send dtc to 
					send_dtc_to_safety_svc(PSM_ID_RASDES_COUNTER_DTC);
				}			
			}
		}

		/* - GROUP7 EVENT - */
		// #define EVENT_COUNTER_EVENT_TX_MEM_WRITE                0x00
		// #define EVENT_COUNTER_EVENT_TX_MEM_READ                 0x01
		// #define EVENT_COUNTER_EVENT_TX_CONFIG_WRITE             0x02
		// #define EVENT_COUNTER_EVENT_TX_CONFIG_READ              0x03
		// #define EVENT_COUNTER_EVENT_TX_IO_WRITE                 0x04
		// #define EVENT_COUNTER_EVENT_TX_IO_READ        		    0x05
		// #define EVENT_COUNTER_EVENT_TX_COMPLE_WITHOUT_DATA      0x06
		// #define EVENT_COUNTER_EVENT_TX_COMPLE_WITH_DATA      	0x07
		// #define EVENT_COUNTER_EVENT_TX_MSG_TLP     				0x08
		// #define EVENT_COUNTER_EVENT_TX_ATOMIC    				0x09
		// #define EVENT_COUNTER_EVENT_TX_TLP_WITH_PREFIX    		0x0A
		// #define EVENT_COUNTER_EVENT_RX_MEM_WRITE                0x0B
		// #define EVENT_COUNTER_EVENT_RX_MEM_READ                 0x0C
		// #define EVENT_COUNTER_EVENT_RX_CONFIG_WRITE             0x0D
		// #define EVENT_COUNTER_EVENT_RX_CONFIG_READ              0x0E
		// #define EVENT_COUNTER_EVENT_RX_IO_WRITE                 0x0F
		// #define EVENT_COUNTER_EVENT_RX_IO_READ        		    0x10
		// #define EVENT_COUNTER_EVENT_RX_COMPLE_WITHOUT_DATA      0x11
		// #define EVENT_COUNTER_EVENT_RX_COMPLE_WITH_DATA      	0x12
		// #define EVENT_COUNTER_EVENT_RX_MSG_TLP     				0x13
		// #define EVENT_COUNTER_EVENT_RX_ATOMIC    				0x14
		// #define EVENT_COUNTER_EVENT_RX_TLP_WITH_PREFIX    		0x15
		// #define EVENT_COUNTER_EVENT_TX_CCIX_TLP    				0x16
		// #define EVENT_COUNTER_EVENT_RX_CCIX_TLP    		        0x15
		for(index  = 0; index < 0x16; index++)
		{
			// i --> event ID
			for(lane_index = 0; lane_index < lane; lane_index++)
			{
				val = event_counter_prog(pcie, lane_index, index, EVENT_COUNTER_GROUP_7);
				if(val > 0)
				{
					pr_err("RASDES Counter Group:[%d] event:[%d] lane:[%d] val:%d DTC:[0x%x]\n", EVENT_COUNTER_GROUP_7, index, lane_index, val, PSM_ID_RASDES_COUNTER_DTC);
					// send dtc to 
					send_dtc_to_safety_svc(PSM_ID_RASDES_COUNTER_DTC);
				}	
			}
		}
#endif
		/* Clear all counters */
		bst_pcie_writel_dbi(pcie->pci, 
				pcie->bst_pcie_diag->ras_des_cap +
				PCIE_RAS_DES_EVENT_COUNTER_CONTROL,
				EVENT_COUNTER_ALL_CLEAR);

		msleep(pcie->bst_pcie_diag->resdes_monitor_periodic);
	}

	return 0;
}

int smlh_ltssm_state(struct bst_pcie* bst_pcie)
{
	struct pcie_phy *phy = bst_pcie->phy;
	u32 dev_info;
	u32 state = 0;
	dev_info = (bst_pcie->chip_type << 8) | bst_pcie->ctrl_id;
	switch (dev_info)
	{
	case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL0):  /*C1200 ctrl0 */
		pcie_phy_cfg(phy, X4_MISC_FUNC_CTRL1, 1, BIT(1)); // config get_current_state
		state = pcie_phy_read(phy, X4_MISC_COM_STATUS) & 0x3f;
		break;
	case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL1): /*C1200 ctrl1 */
		pcie_phy_cfg(phy, X2_MISC_FUNC_CTRL1, 1, BIT(1)); // config get_current_state
		state = pcie_phy_read(phy, X2_MISC_COM_STATUS) & 0x3f;
		break;
	default:
		pr_err("state: ctrl_id error\n");
		return 0;
	}

	return state;
}

/**
* pcie_smlh_ltssm_state_monitor.
* @data:	struct bst_pcie
* PSM_CPU_PCIE_15
* PCIE Periodic Read of RASDES Counters
*/
int pcie_smlh_ltssm_state_monitor(void *data)
{
	struct bst_pcie* bst_pcie = (struct bst_pcie*)data;
	u32 state = 0;


	u32 old_state = smlh_ltssm_state(bst_pcie);
	if(old_state != PORT_LOGIC_LTSSM_STATE_L0)
	{
		pr_info("LTSSM state:[%x], Exit ltssm monitor\n", state);
		return 0;
	}

    while(bst_pcie->bst_pcie_diag->diag_status)
    {
		state = smlh_ltssm_state(bst_pcie);
		if(state != PORT_LOGIC_LTSSM_STATE_L0)
		{
			if(state > 0x23) // more than max state
			{
				pr_err("Limit LTSSM  State:[%x]\n", state);
				send_dtc_to_safety_svc(PSM_ID_LTSSM_DTC);
			}
			else if(state <= 0x0C && old_state == PORT_LOGIC_LTSSM_STATE_L0
						&& bst_pcie->bst_pcie_diag->diag_status == PCIE_DIAG_RUNNING_STATE) // L0 can not switch to detect/poll/configure
			{ // If the application detects an incorrect LTSSM state transition (e.g. L0 to Polling) it should push to a safe state
				// temp code send a DTC  is not in L0
				// pr_err("incorrect LTSSM state transition :[%x]\n", state);
				send_dtc_to_safety_svc(PSM_ID_LTSSM_DTC);
			}
			else
			{
				pr_debug("Current LTSSM :[%x]\n", state);
			}
		}
		old_state = state;
        msleep(bst_pcie->bst_pcie_diag->smlh_ltssm_state_monitor_periodic);
    }
	pr_info("diag_status:[%d] Exit ltssm monitor\n", bst_pcie->bst_pcie_diag->diag_status);
    return 0;
}

/**
* axi_interface_monitor.
* @data:	struct bst_pcie
* PSM_CPU_PCIE_19
* PCIE AXI Fabric Supervisor
*/
int axi_interface_monitor(struct bst_pcie* bst_pcie)
{
	struct pcie_phy *phy = bst_pcie->phy;
	unsigned long val;
	u32 dev_info;
	u32 status;
	pr_info("axi_interface_monitor %d\n", bst_pcie->ctrl_id);
	dev_info = (bst_pcie->chip_type << 8) | bst_pcie->ctrl_id;
	switch (dev_info) {
	case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL0): /*C1200 ctrl0 */
		val = pcie_phy_read(phy, X4_AXI_SIDEBAND_CTRL0);
		break;
	case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL1): /*C1200 ctrl1 */
		val = pcie_phy_read(phy, X2_AXI_SIDEBAND_CTRL0);
		break;
	default:
		return -EINVAL;
	}

	if(bst_pcie->bst_pcie_diag->axi_monitor_psm)
	{
		status = (val >> AXI_MASTR_READ_STATUS_SECTION_SHIFT) & AXI_STATUS_DATA_MASK;
		if(status != AXI_SC)
		{
			pr_err("AXI Master Read SLVERR/DECERR [%d]\n", status);
			send_dtc_to_safety_svc(PSM_ID_AXI_SUPERVISOR_DTC);
		}
		status = (val >> AXI_MASTR_WRITE_STATUS_SECTION_SHIFT) & AXI_STATUS_DATA_MASK;
		if(status != AXI_SC)
		{
			pr_err("AXI Master Write SLVERR/DECERR [%d]\n", status);
			send_dtc_to_safety_svc(PSM_ID_AXI_SUPERVISOR_DTC);
		}
	}

	return 0;
}

/**
* dbi_access_monitor_monitor.
* @data:	struct bst_pcie
* PSM_CPU_PCIE_22
* PCIE DBI Register Read / Write Access
*/
int dbi_access_monitor_monitor(void* data)
{
	struct bst_pcie* bst_pcie = (struct bst_pcie*)data;
    while(1)
    {
		if(bst_pcie->bst_pcie_diag->dbi_access_failed_count > 0)
		{
			pr_err("exist dbi access fail [%d]\n", bst_pcie->bst_pcie_diag->dbi_access_failed_count);
			send_dtc_to_safety_svc(PSM_ID_DBI_REG_RW_ACCESS_DTC);
			// clear
			bst_pcie->bst_pcie_diag->dbi_access_failed_count = 0;
		}

		msleep(bst_pcie->bst_pcie_diag->dbi_access_monitor_periodic);
	}
	return 0;
}

// This diagnostic provides a mechanism to detect erroneous indications of RASDP error mode 
// (mstr_rasdp_err_mode and slv_rasdp_err_mode). 
// The diagnostic consists of monitoring the signals indicating 
// DM is in the data path protection mode. The test runs as part of the ISR and follows these steps:
// • Procedure:
// 1. Upon assertion of mstr_rasdp_err_mode or slv_rasdp_err_mode, read all RASDP uncorrectable error counters -- RASDP_UNCORR_COUNTER_CTRL_OFF/RASDP_UNCORR_COUNT_REPORT_OFF
// 2. If all uncorrectable counters are at 0 this is an indication of a false RASDP error indication
// 3. If ISR is not triggered for a certain period, read the RASDP uncorrectable counters. If any counter value is
// greater than 0 this indicates a missing RASDP error mode indication
/**
* pcie_rasdp_error_mode_monitor.
* @data:	struct bst_pcie
* PSM_CPU_PCIE_28
* PCIE RASDP Error Mode Monitoring
*/
int pcie_rasdp_error_mode_monitor(void *data)
{
	struct bst_pcie* bst_pcie = (struct bst_pcie*)data;
	struct pcie_phy *phy = bst_pcie->phy;
	u32 status = 0;
	int i = 0;
	u8 sel = 0;
	u32 dev_info;

	dev_info = (bst_pcie->chip_type << 8) | bst_pcie->ctrl_id;
	while(1)
	{
		switch (dev_info) {
		case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL0): /*C1200 ctrl0 */
			status = pcie_phy_read(phy, X4_MISC_INFO_10);
			break;
		case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL1): /*C1200 ctrl1 */
			status = pcie_phy_read(phy, X2_MISC_INFO_10);
			break;
		default:
			return -EINVAL;
		}

		// pr_info("bst_pcie->ctrl_id [%d] status_slv: %x statue_mstr:%x\n", 
		// 									bst_pcie->ctrl_id,
		// 									status & (1 << SLV_RASDP_ERR_MODE_OFF),
		// 									status & (1 << MSTR_RASDP_ERR_MODE_OFF));

		if(status & (1 << SLV_RASDP_ERR_MODE_OFF) || status & (1 << MSTR_RASDP_ERR_MODE_OFF))
		{
			for(sel = 0; sel < 14; sel++) // selection region
			{
				// cycle RASDP_UNCORR_COUNTER_CTRL_OFF:UNCORR_COUNTER_SELECTION field 0 ~ 255
				for(i = 0; i < 256; i++)
				{
					status = bst_pcie_readl_dbi(bst_pcie->pci, bst_pcie->bst_pcie_diag->ras_dp_cap + RASDP_UNCORR_COUNTER_CTRL_OFF);

					status &= ~(RASDP_UNCORR_COUNTER_SELECTIN_MASK << RASDP_UNCORR_COUNTER_SELECTION_SHIFT); // bit24 ~ 31 清0
					status &= ~(RASDP_UNCORR_COUNTER_REG_SELECTIN_MASK << RASDP_UNCORR_COUNTER_REG_SELECTION_SHIFT); // bit20 ~ 23 清0
					status |= i << RASDP_UNCORR_COUNTER_SELECTION_SHIFT;
					status |= sel << RASDP_UNCORR_COUNTER_REG_SELECTION_SHIFT;
					status |= 1 << 4; //enable
					pr_info("RASDP_UNCORR_COUNTER_CTRL_OFF 0x%x 0x%x\n", bst_pcie->bst_pcie_diag->ras_dp_cap + RASDP_UNCORR_COUNTER_CTRL_OFF, status);
					bst_pcie_writel_dbi(bst_pcie->pci, bst_pcie->bst_pcie_diag->ras_dp_cap + RASDP_UNCORR_COUNTER_CTRL_OFF, status);
					status = bst_pcie_readl_dbi(bst_pcie->pci, bst_pcie->bst_pcie_diag->ras_dp_cap + RASDP_UNCORR_COUNT_REPORT_OFF);
					if(status > 0)
					{
						// pr_err("UNCORR_COUNTER_SELECTION ID:[%d] region:[0x%x] counter:[%d]\n", i, sel, status);
						// send dtc
						send_dtc_to_safety_svc(PSM_ID_RASDP_ERR_DTC);
					}
				}
				status = bst_pcie_readl_dbi(bst_pcie->pci, bst_pcie->bst_pcie_diag->ras_dp_cap + RASDP_UNCORR_COUNTER_CTRL_OFF);
				status |= RASDP_UNCORR_CLEAR_COUNTERS_MASK;
				bst_pcie_writel_dbi(bst_pcie->pci, bst_pcie->bst_pcie_diag->ras_dp_cap + RASDP_UNCORR_COUNTER_CTRL_OFF, status);
			}
		}

        msleep(bst_pcie->bst_pcie_diag->rasdp_error_mode_monitor_periodic);
	}

    return 0;
}

/**
* pcie_cr_check_safety_monitor.
* @data:	struct bst_pcie
* PSM_CPU_PCIE_PHY_001
* PCIE PHY Register check through CR Parallel Interface
*/
int pcie_cr_check_safety_monitor(void *data)
{
	struct bst_pcie* bst_pcie = (struct bst_pcie*)data;
	struct pcie_phy *phy = bst_pcie->phy;
    u16 value = 0;

    // to confirm
    // 4 lanes or 2 lanes
    u16 lanes = bst_pcie->ctrl_id ? 2 : 4;

    while(1)
    {
        for(int lane = 0; lane < lanes; lane++)
        {
            // 1. LANEN_DIG_TX_LBERT_CTL 0x1N32
            // 2. LANEN_DIG_RX_LBERT_CTL 0x1n47
            // 3. LANEN_DIG_RX_LBERT_ERR 0x1N48
            u16 lanen_dig_tx_lbert_ctl = (1 << 12 ) | (lane << 8) | 0x32;
            u16 lanen_dig_rx_lbert_ctl = (1 << 12 ) | (lane << 8) | 0x47;
            u16 lanen_dig_rx_lbert_err = (1 << 12 ) | (lane << 8) | 0x48;

			c1200_read_phy_cr(phy, bst_pcie->ctrl_id , lanen_dig_tx_lbert_ctl, &value);
			//pr_info("ctrl:[%d] lanen_dig_tx_lbert_ctl:%x cr: %x\n", bst_pcie->ctrl_id, lanen_dig_tx_lbert_ctl, value);
			c1200_read_phy_cr(phy, bst_pcie->ctrl_id , lanen_dig_rx_lbert_ctl, &value);
			//pr_info("ctrl:[%d] lanen_dig_rx_lbert_ctl:%x cr: %x\n", bst_pcie->ctrl_id, lanen_dig_rx_lbert_ctl, value);
			c1200_read_phy_cr(phy, bst_pcie->ctrl_id , lanen_dig_rx_lbert_err, &value);
			//pr_info("ctrl:[%d] lanen_dig_rx_lbert_err:%x cr: %x\n", bst_pcie->ctrl_id, lanen_dig_rx_lbert_err, value);
			//pr_info("\n");
			if(value > 0)
			{
				pr_err("ctrl:[%d] lanen_dig_rx_lbert_err:%x cr: %x\n", bst_pcie->ctrl_id, lanen_dig_rx_lbert_err, value);
				send_dtc_to_safety_svc(PSM_ID_PHY_REG_CHECK_DTC);
			}
        }

        msleep(bst_pcie->bst_pcie_diag->cr_check_safety_monitor_periodic);
    }

    return 0;
}

/**
* ext_sram_access_entry.
* @data:	struct bst_pcie
* PSM_CPU_PCIE_PHY_002
* PCIE PHY External SRAM Interface Access
*/
int ext_sram_access_entry(void *data)
{
	struct bst_pcie* bst_pcie = (struct bst_pcie*)data;
	struct pcie_phy *phy = bst_pcie->phy;
	u32 dev_info;
	u16 val;
	// u32 result;
	u16 tmp = 0xde;
	dev_info = (bst_pcie->chip_type << 8) | bst_pcie->ctrl_id;

	// read sram status
	c1200_read_phy_cr(phy, bst_pcie->ctrl_id, PHY_STATUS0, &val);
	if(!val)
	{
		pr_err("sram init fail\n");
		return -1;
	}

    while(1)
    {
		switch (dev_info) {
		case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL0): /*C1200 ctrl0 */
			pcie_phy_cfg(phy, X4_MISC_FUNC_CTRL0, 1, BIT(4)); // enable phy1_sram_bypass
			pcie_phy_cfg(phy, X4_MISC_FUNC_CTRL0, 1, BIT(2)); // csr_phy0_sram_ext_ld_done
			break;
		case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL1): /*C1200 ctrl1 */
			pcie_phy_cfg(phy, X2_MISC_FUNC_CTRL0, 1, BIT(5)); // enable phy0_sram_bypass
			pcie_phy_cfg(phy, X2_MISC_FUNC_CTRL0, 1, BIT(3)); // csr_phy1_sram_ext_ld_done
			break;
		default:
			return -EINVAL;
		}

		// Test range 0xf000 ~ 0xFFF0
		c1200_write_phy_cr(phy, bst_pcie->ctrl_id, bst_pcie->bst_pcie_diag->ext_sram_access_addr, tmp);
		c1200_read_phy_cr(phy, bst_pcie->ctrl_id, bst_pcie->bst_pcie_diag->ext_sram_access_addr, &val);

		switch (dev_info) {
			case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL0): /*C1200 ctrl0 */
				pcie_phy_cfg(phy, X4_MISC_FUNC_CTRL0, 0, BIT(4)); // disable phy0_sram_bypass
				pcie_phy_cfg(phy, X4_MISC_FUNC_CTRL0, 0, BIT(2)); // csr_phy0_sram_ext_ld_done
				break;
			case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL1): /*C1200 ctrl1 */
				pcie_phy_cfg(phy, X2_MISC_FUNC_CTRL0, 0, BIT(5));  // disable phy1_sram_bypass
				pcie_phy_cfg(phy, X2_MISC_FUNC_CTRL0, 0, BIT(3)); // csr_phy1_sram_ext_ld_done
				break;
			default:
				return -EINVAL;
		}

		if(val != tmp)
		{
			pr_info("external sram r/w mismatch val:0x%x tmp:0x%x\n", val, tmp);
			send_dtc_to_safety_svc(PSM_ID_EXT_SRAM_ACCESS_DTC);
		}

        msleep(bst_pcie->bst_pcie_diag->ext_sram_access_periodic);
    }

    return 0;
}

/**
* pcie_tx2rx_loopback_monitor.
* @data:	struct bst_pcie
* PSM_CPU_PCIE_PHY_003
* PCIE PHY Internal TX to RX Loopback
*/
int pcie_tx2rx_loopback_monitor(void* data)
{
	struct bst_pcie* bst_pcie = (struct bst_pcie*)data;
	struct pcie_phy *phy = bst_pcie->phy;
	u16 status = 0;
	u32 dev_info;
	dev_info = (bst_pcie->chip_type << 8) | bst_pcie->ctrl_id;

	while(1)
	{
		// enable tx2rx
		switch (dev_info)
		{
		case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL0):  /*C1200 ctrl0 */
			pcie_phy_cfg(phy, X4_MISC_FUNC_CTRL5, 1, BIT(6)); // config get_current_state
			pcie_phy_cfg(phy, X4_MISC_FUNC_CTRL5, 1, BIT(14));
			pcie_phy_cfg(phy, X4_MISC_FUNC_CTRL5, 1, BIT(22)); // config get_current_state
			pcie_phy_cfg(phy, X4_MISC_FUNC_CTRL5, 1, BIT(30));
			break;
		case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL1): /*C1200 ctrl1 */
			pcie_phy_cfg(phy, X2_MISC_FUNC_CTRL5, 1, BIT(6)); // config get_current_state
			pcie_phy_cfg(phy, X2_MISC_FUNC_CTRL5, 1, BIT(14));
			//pcie_phy_cfg(phy, X2_MISC_FUNC_CTRL5, 1, BIT(22)); // config get_current_state
			//pcie_phy_cfg(phy, X2_MISC_FUNC_CTRL5, 1, BIT(30));
			break;
		default:
			pr_err("state: ctrl_id error\n");
			return 0;
		}

		msleep(bst_pcie->bst_pcie_diag->tx2rx_monitor_periodic);

		// designer say:读phy的0x1048和0x1148，每个读两次，第二次为0就说明没有错误
		c1200_read_phy_cr(phy, bst_pcie->ctrl_id, 0x1048, &status);
		//pr_info("%s status:%d\n", __FUNCTION__, status);
		c1200_read_phy_cr(phy, bst_pcie->ctrl_id, 0x1048, &status);
		//pr_info("%s status:%d\n", __FUNCTION__, status);
		c1200_read_phy_cr(phy, bst_pcie->ctrl_id, 0x1148, &status);
		//pr_info("%s status:%d\n", __FUNCTION__, status);
		c1200_read_phy_cr(phy, bst_pcie->ctrl_id, 0x1148, &status);
		//pr_info("%s status:%d\n", __FUNCTION__, status);

		if(status) //监控状态寄存器
		{
			pr_err("tx2rx loopback fail\n");
			send_dtc_to_safety_svc(PSM_ID_TX2RX_LOOPBACK_DTC);
		}
	
		// disable tx2rx
		// enable tx2rx
		switch (dev_info)
		{
		case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL0):  /*C1200 ctrl0 */
			pcie_phy_cfg(phy, X4_MISC_FUNC_CTRL5, 0, BIT(6)); // config get_current_state
			pcie_phy_cfg(phy, X4_MISC_FUNC_CTRL5, 0, BIT(14));
			pcie_phy_cfg(phy, X4_MISC_FUNC_CTRL5, 0, BIT(22)); // config get_current_state
			pcie_phy_cfg(phy, X4_MISC_FUNC_CTRL5, 0, BIT(30));
			break;
		case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL1): /*C1200 ctrl1 */
			pcie_phy_cfg(phy, X2_MISC_FUNC_CTRL5, 0, BIT(6)); // config get_current_state
			pcie_phy_cfg(phy, X2_MISC_FUNC_CTRL5, 0, BIT(14));
			//pcie_phy_cfg(phy, X2_MISC_FUNC_CTRL5, 1, BIT(22)); // config get_current_state
			//pcie_phy_cfg(phy, X2_MISC_FUNC_CTRL5, 1, BIT(30));
			break;
		default:
			pr_err("state: ctrl_id error\n");
			return 0;
		}
		msleep(bst_pcie->bst_pcie_diag->tx2rx_monitor_periodic);
	}

	return 0;
}

// find bst vendor capability ID
static u16 bst_pcie_find_vendor_ext_cap(struct dw_pcie *pci, u8 cap, u8 vendor_id)
{
	u32 header;
	u32 vnd_header;
	int ttl;
	int pos = PCI_CFG_SPACE_SIZE;

	/* minimum 8 bytes per capability */
	ttl = (PCI_CFG_SPACE_EXP_SIZE - PCI_CFG_SPACE_SIZE) / 8;


	header = dw_pcie_readl_dbi(pci, pos);
	/*
	* If we have no capabilities, this is indicated by cap ID,
	* cap version and next pointer all being 0.
	*/
	if (header == 0)
		return 0;

	while (ttl-- > 0)
	{
		if (PCI_EXT_CAP_ID(header) == cap)
		{
			// find vendor-specific header
			vnd_header = dw_pcie_readl_dbi(pci, pos + 0x4);
			if((vnd_header & 0x0000ffff) == vendor_id)
				return pos;
		}

		pos = PCI_EXT_CAP_NEXT(header);
		if (pos < PCI_CFG_SPACE_SIZE)
			break;

		header = dw_pcie_readl_dbi(pci, pos);
	}

	return 0;
}

int pcie_bst_diag_init(void* data)
{
    struct bst_pcie* bst_pcie = (struct bst_pcie*)data;
	uint8_t block_id_out;
	uint32_t psm_id_out[4] = {0};
	u32 psm_id_out_mask = 0x1;
	// bst_pcie_diagnostic p_bst_pcie_diag = NULL;

	// 获取pcie block下面的所以psm state
	// 3次fail，enable全部psm
	int ret = get_psmid_from_safety_lib(PSM_CPU_PCIE_BLOCK, &block_id_out, (uint32_t*)&psm_id_out);
	if(ret < 0)
	{
		pr_err("get PSM_CPU_PCIE_BLOCK psm status err\n");
	}

	bst_pcie->bst_pcie_diag = devm_kzalloc(bst_pcie->pci->dev, sizeof(struct bst_pcie_diagnostic_t), GFP_KERNEL);
	if (!bst_pcie->bst_pcie_diag)
    {
        pr_err("alloc p_bst_pcie_diag failed\n");
		return -ENOMEM;
    }
   // bst_pcie->bst_pcie_diag->diag_status = PCIE_DIAG_RUNNING_STATE; // diagnostic is running
	/*
	* DHP-47 
	* 
	* PSM_CPU_PCIE_15
	* PCIE Periodic Read of RASDES Counters
	*/
	if((psm_id_out_mask << (PSM_ID_RASDES_COUNTER % 32)) & psm_id_out[PSM_ID_RASDES_COUNTER / 32])
	{
		bst_pcie->bst_pcie_diag->resdes_monitor_psm = 1; // psm enable
		bst_pcie->bst_pcie_diag->resdes_monitor_periodic = DEFAULT_MON_PERIODIC; // default 2s

		bst_pcie->bst_pcie_diag->ras_des_cap = bst_pcie_find_vendor_ext_cap(bst_pcie->pci, PCI_EXT_CAP_ID_VNDR, 2);
		pr_err("ras_des_cap:0x%x\n", bst_pcie->bst_pcie_diag->ras_des_cap);
		bst_pcie->bst_pcie_diag->resdes_counter_monitor_tsk = kthread_run(bst_diag_rasdes_counter_monitor, bst_pcie, "rasdes-c-mon");
		if (IS_ERR(bst_pcie->bst_pcie_diag->resdes_counter_monitor_tsk)) {
			dev_err(bst_pcie->pci->dev, 
										"rasdes-counter-monitor failed, rv: %ld\n", 
										PTR_ERR(bst_pcie->bst_pcie_diag->resdes_counter_monitor_tsk));
			return -1;
		}
	}

    /*
	* DHP-47 
	* 
	* PSM_CPU_PCIE_16
	* PCIE Configuration Register Readback
	*/
	if((psm_id_out_mask << (PSM_ID_CONFIGURE_REG_READBACK % 32)) & psm_id_out[PSM_ID_CONFIGURE_REG_READBACK / 32])
	{
		bst_pcie->bst_pcie_diag->reg_rdback_monitor_psm = 1;
		// Trigger on demand
		// default is disable, user can configure
		bst_pcie->bst_pcie_diag->reg_rdback_monitor_enable = 0;
	}


    /*
	* DHP-47 
	* 
	* PSM_CPU_PCIE_18
	* PCIE Monitor PCIe Link State (LTSSM)
	*/

	if((psm_id_out_mask << (PSM_ID_LTSSM % 32)) & psm_id_out[PSM_ID_LTSSM / 32]) 
	{
		bst_pcie->bst_pcie_diag->smlh_ltssm_state_monitor_psm = 1;
		bst_pcie->bst_pcie_diag->smlh_ltssm_state_monitor_periodic = DEFAULT_MON_PERIODIC; //default is set 5s
		bst_pcie->bst_pcie_diag->smlh_ltssm_state_task = kthread_run(pcie_smlh_ltssm_state_monitor,
															bst_pcie,
															"ltssm_mon");;
		if (IS_ERR(bst_pcie->bst_pcie_diag->smlh_ltssm_state_task )) {
			dev_err(bst_pcie->pci->dev, 
										"smlh_ltssm_state_monitor failed, rv: %ld\n", 
										PTR_ERR(bst_pcie->bst_pcie_diag->smlh_ltssm_state_task));
			return -1;
		}
	}

    /*
	* DHP-47 
	* 
	* PSM_CPU_PCIE_19
	* PCIE AXI Fabric Supervisor
	*/
	if((psm_id_out_mask << (PSM_ID_AXI_SUPERVISOR % 32)) & psm_id_out[PSM_ID_AXI_SUPERVISOR / 32])
	{
		bst_pcie->bst_pcie_diag->axi_monitor_psm = 1;
	}

    /*
	* DHP-47 
	* 
	* PSM_CPU_PCIE_22
	* PCIE DBI Register Read / Write Access
	*/
	if((psm_id_out_mask << (PSM_ID_DBI_REG_RW_ACCESS % 32)) & psm_id_out[PSM_ID_DBI_REG_RW_ACCESS / 32])
	{
		bst_pcie->bst_pcie_diag->dbi_access_monitor_psm = 1;
		bst_pcie->bst_pcie_diag->dbi_access_monitor_periodic = DEFAULT_MON_PERIODIC; //default is set 5s
		bst_pcie->bst_pcie_diag->dbi_access_monitor_task = kthread_run(dbi_access_monitor_monitor,
															bst_pcie,
															"dbi_access_monitor_monitor");;
		if (IS_ERR(bst_pcie->bst_pcie_diag->dbi_access_monitor_task )) {
			dev_err(bst_pcie->pci->dev, 
										"dbi_access_monitor failed, rv: %ld\n", 
										PTR_ERR(bst_pcie->bst_pcie_diag->dbi_access_monitor_task));
			return -1;
		}
	}

    /*
	* DHP-47 
	* 
	* PSM_CPU_PCIE_28
	* PCIE RASDP Error Mode Monitoring
	*/
	if((psm_id_out_mask << (PSM_ID_RASDP_ERR_MODE_MONITOR % 32)) & psm_id_out[PSM_ID_RASDP_ERR_MODE_MONITOR / 32])
	{
		bst_pcie->bst_pcie_diag->rasdp_error_mode_monitor_psm = 1;
		bst_pcie->bst_pcie_diag->rasdp_error_mode_monitor_periodic = DEFAULT_MON_PERIODIC; //default is set 5s

		bst_pcie->bst_pcie_diag->ras_dp_cap = bst_pcie_find_vendor_ext_cap(bst_pcie->pci, PCI_EXT_CAP_ID_VNDR, 1);
		pr_err("ras_dp_cap:0x%x\n", bst_pcie->bst_pcie_diag->ras_dp_cap);
		bst_pcie->bst_pcie_diag->rasdp_error_mode_monitor_task = kthread_run(pcie_rasdp_error_mode_monitor,
															bst_pcie,
															"smlh_ltssm_state_monitor");;
		if (IS_ERR(bst_pcie->bst_pcie_diag->rasdp_error_mode_monitor_task )) {
			dev_err(bst_pcie->pci->dev, 
										"smlh_ltssm_state_monitor failed, rv: %ld\n", 
										PTR_ERR(bst_pcie->bst_pcie_diag->rasdp_error_mode_monitor_task));
			return -1;
		}
	}


	/*
	* DHP-47 
	* 
	* PSM_CPU_PCIE_PHY_001
	* PCIE PHY Register check through CR Parallel Interface
	*/
	if((psm_id_out_mask << (PSM_ID_PHY_REG_CHECK % 32)) & psm_id_out[PSM_ID_PHY_REG_CHECK / 32])
	{
		bst_pcie->bst_pcie_diag->cr_check_safety_monitor_psm = 1;
		bst_pcie->bst_pcie_diag->cr_check_safety_monitor_enable = 1;
		bst_pcie->bst_pcie_diag->cr_check_safety_monitor_periodic = DEFAULT_MON_PERIODIC; //default is set 5s
		bst_pcie->bst_pcie_diag->cr_check_safety_monitor_task = kthread_run(pcie_cr_check_safety_monitor,
																bst_pcie,
																"pcie_cr_check_safety_monitor");;
		if (IS_ERR(bst_pcie->bst_pcie_diag->cr_check_safety_monitor_task )) {
			dev_err(bst_pcie->pci->dev, 
										"pcie_cr_check_safety_monitor failed, rv: %ld\n", 
										PTR_ERR(bst_pcie->bst_pcie_diag->cr_check_safety_monitor_task));
			return -1;
		}
	}

    /*
	* DHP-47 
	* 
	* PSM_CPU_PCIE_PHY_002
	* PCIE PHY External SRAM Interface Access
	*/
	if((psm_id_out_mask << (PSM_ID_EXT_SRAM_ACCESS % 32)) & psm_id_out[PSM_ID_EXT_SRAM_ACCESS / 32])
	{
		bst_pcie->bst_pcie_diag->ext_sram_access_psm = 1;
		bst_pcie->bst_pcie_diag->ext_sram_access_periodic = DEFAULT_MON_PERIODIC; //default is set 5s
		bst_pcie->bst_pcie_diag->ext_sram_access_addr = 0xFFF0;
		bst_pcie->bst_pcie_diag->ext_sram_access_task = kthread_run(ext_sram_access_entry,
																bst_pcie,
																"pcie_cr_check_safety_monitor");;
		if (IS_ERR(bst_pcie->bst_pcie_diag->ext_sram_access_task )) {
			dev_err(bst_pcie->pci->dev, 
									  "ext_sram_access_entry failed, rv: %ld\n",
									  PTR_ERR(bst_pcie->bst_pcie_diag->ext_sram_access_task));
			return -1;
		}
	}
	/*
	* DHP-47 
	* 
	* PSM_CPU_PCIE_PHY_003
	* PCIE PHY Internal TX to RX Loopback
	*/
	if((psm_id_out_mask << (PSM_ID_TX2RX_LOOPBACK % 32)) & psm_id_out[PSM_ID_TX2RX_LOOPBACK / 32])
	{
		bst_pcie->bst_pcie_diag->tx2rx_monitor_psm = 1; //default is set 5s
		bst_pcie->bst_pcie_diag->tx2rx_monitor_periodic = DEFAULT_MON_PERIODIC; //default is set 5s
		bst_pcie->bst_pcie_diag->tx2rx_loopback_monitor_task = kthread_run(pcie_tx2rx_loopback_monitor,
															bst_pcie,
															"tx2rx_monitor");;
		if (IS_ERR(bst_pcie->bst_pcie_diag->tx2rx_loopback_monitor_task )) {
			dev_err(bst_pcie->pci->dev, 
										"pcie_tx2rx_loopback_monitor failed, rv: %ld\n", 
										PTR_ERR(bst_pcie->bst_pcie_diag->tx2rx_loopback_monitor_task));
			return -1;
		}
	}

	bst_pcie->pcie_diagnostic_init_done = 1;
	bst_pcie->bst_pcie_diag->diag_status = PCIE_DIAG_RUNNING_STATE;
	pr_info("bst pcie_diagnostic_init_done\n");
    return 0;
}


ssize_t diagnostic_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bst_pcie *bst_pcie = dev->driver_data;
	uint8_t block_id_out;
	uint32_t psm_id_out[4] = {0};
	int ret = 0;
    pr_info("\n <====== PCIe Diagnostic information show ====> \n");
	ret = get_psmid_from_safety_lib(PSM_CPU_PCIE_BLOCK, &block_id_out, (uint32_t*)&psm_id_out);
	if(ret < 0)
	{
		pr_err("get PSM_CPU_PCIE_BLOCK psm status err\n");
	}
	pr_info("BLOCK ID [0x%x] PSM STATUS:\n", block_id_out);
	pr_info("H-Bit=> 0x%x-0x%x-0x%x-0x%x <= L-Bit\n", psm_id_out[3],
													  psm_id_out[2],
													  psm_id_out[1],
													  psm_id_out[0]);
	pr_info("resdes_monitor_psm:[%d]\n", bst_pcie->bst_pcie_diag->resdes_monitor_psm);
	pr_info("resdes_monitor_periodic:[%d]\n", bst_pcie->bst_pcie_diag->resdes_monitor_periodic);

	pr_info("reg_rdback_monitor_psm:[%d]\n", bst_pcie->bst_pcie_diag->reg_rdback_monitor_psm);
	pr_info("reg_rdback_monitor_enable:[%d]\n", bst_pcie->bst_pcie_diag->reg_rdback_monitor_enable);

	pr_info("smlh_ltssm_state_monitor_psm:[%d]\n", bst_pcie->bst_pcie_diag->smlh_ltssm_state_monitor_psm);
	pr_info("smlh_ltssm_state_monitor_periodic:[%d]\n", bst_pcie->bst_pcie_diag->smlh_ltssm_state_monitor_periodic);

	pr_info("axi_monitor_psm:[%d]\n", bst_pcie->bst_pcie_diag->axi_monitor_psm);

	pr_info("dbi_access_monitor_psm:[%d]\n", bst_pcie->bst_pcie_diag->dbi_access_monitor_psm);
	pr_info("dbi_access_failed_count:[%d]\n", bst_pcie->bst_pcie_diag->dbi_access_failed_count);
	pr_info("dbi_access_monitor_periodic:[%d]\n", bst_pcie->bst_pcie_diag->dbi_access_monitor_periodic);

	pr_info("rasdp_error_mode_monitor_psm:[%d]\n", bst_pcie->bst_pcie_diag->cr_check_safety_monitor_psm);
	pr_info("rasdp_error_mode_monitor_periodic:[%d]\n", bst_pcie->bst_pcie_diag->rasdp_error_mode_monitor_periodic);

	pr_info("cr_check_safety_monitor_psm:[%d]\n", bst_pcie->bst_pcie_diag->rasdp_error_mode_monitor_psm);
	pr_info("rasdp_error_mode_monitor_periodic:[%d]\n", bst_pcie->bst_pcie_diag->cr_check_safety_monitor_periodic);

	pr_info("ext_sram_access_psm:[%d]\n", bst_pcie->bst_pcie_diag->ext_sram_access_psm); 
	pr_info("ext_sram_access_addr:[%x]\n", bst_pcie->bst_pcie_diag->ext_sram_access_addr);
	pr_info("ext_sram_access_periodic:[%d]\n", bst_pcie->bst_pcie_diag->ext_sram_access_periodic);

	pr_info("tx2rx_monitor_psm:[%d]\n", bst_pcie->bst_pcie_diag->tx2rx_monitor_psm);
	pr_info("tx2rx_monitor_periodic:[%d]\n", bst_pcie->bst_pcie_diag->tx2rx_monitor_periodic);

    pr_info("<====== PCIe Diagnostic information show ====> \n");
	// pcie_cr_check_safety_monitor(bst_pcie);
	// ext_sram_access_entry(bst_pcie);
	// pcie_tx2rx_loopback_monitor(bst_pcie);

    return 0;
}

ssize_t diagnostic_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct bst_pcie *bst_pcie = dev->driver_data;
    long val;
	char *token, *cur, *tmp;
	char *delimiter = " ";

	tmp = kmalloc(strlen(buf) + 1, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	strscpy(tmp, buf, strlen(buf) + 1);
	cur = tmp;

	while ((token = strsep(&cur, delimiter)) != NULL) 
	{
		if(!strncmp(token, "trigger", strlen(token))) 
		{
			token = strsep(&cur, delimiter);
			if(!strncmp(token, "rdbk", strlen(token)))
			{
				token = strsep(&cur, delimiter);
				if(!kstrtol(token, 10, &val))
				{
					bst_pcie->bst_pcie_diag->reg_rdback_monitor_enable = val;
				}
			}
			else if(!strncmp(token, "axi", strlen(token)))
			{
				token = strsep(&cur, delimiter);
				if(!kstrtol(token, 10, &val))
				{
					if(val == 1)
						axi_interface_monitor(bst_pcie);
				}
			}		
		}
		else if(!strncmp(token, "set", strlen(token))) 
		{
			token = strsep(&cur, delimiter);
			if(!strncmp(token, "rasdes", strlen(token)))
			{
				token = strsep(&cur, delimiter);
				if(!kstrtol(token, 10, &val))
				{
					bst_pcie->bst_pcie_diag->resdes_monitor_periodic = val;
				}
			}
			else if(!strncmp(token, "ltssm_p", strlen(token)))
			{
				token = strsep(&cur, delimiter);
				if(!kstrtol(token, 10, &val))
				{
					bst_pcie->bst_pcie_diag->smlh_ltssm_state_monitor_periodic = val;
				}
			}
			else if(!strncmp(token, "dbi_access", strlen(token)))
			{
				token = strsep(&cur, delimiter);
				if(!kstrtol(token, 10, &val))
				{
					bst_pcie->bst_pcie_diag->dbi_access_monitor_periodic = val;
				}
			}
			else if(!strncmp(token, "rasdp", strlen(token)))
			{
				token = strsep(&cur, delimiter);
				if(!kstrtol(token, 10, &val))
				{
					bst_pcie->bst_pcie_diag->rasdp_error_mode_monitor_periodic = val;
				}
			}
			else if(!strncmp(token, "cr_check", strlen(token)))
			{
				token = strsep(&cur, delimiter);
				if(!kstrtol(token, 10, &val))
				{
					bst_pcie->bst_pcie_diag->cr_check_safety_monitor_periodic = val;
				}
			}
			else if(!strncmp(token, "sram_p", strlen(token)))
			{
				token = strsep(&cur, delimiter);
				if(!kstrtol(token, 10, &val))
				{
					bst_pcie->bst_pcie_diag->ext_sram_access_periodic = val;
				}
			}	
			else if(!strncmp(token, "tx2rx", strlen(token)))
			{
				token = strsep(&cur, delimiter);
				if(!kstrtol(token, 10, &val))
				{
					bst_pcie->bst_pcie_diag->tx2rx_monitor_periodic = val;
				}
			}		
			else if(!strncmp(token, "sram_addr", strlen(token)))
			{
				token = strsep(&cur, delimiter);
				if(!kstrtol(token, 16, &val))
				{
					bst_pcie->bst_pcie_diag->ext_sram_access_addr = val;
				}
			}	
		}
		else
		{
			// dump help information
			pr_info("\n Wow!! refer to the operation below !\n");
			pr_info("diagnostic set format:\n");
			pr_info("trigger type[rdbk/axi] status[1:enable 0:disable]\n");
			pr_info("set type[rasdes/ltssm_p/dbi_access/rasdp/cr_check/sram_p/tx2rx] periodic\n");
			pr_info("set type[sram_addr] addr\n");
			pr_info("Thanks\n\n");
			break;
		}
	}

    return count;
}

#endif
