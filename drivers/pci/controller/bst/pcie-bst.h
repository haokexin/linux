/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Synopsys DesignWare PCIe host controller driver
 *
 * Copyright (C) 2013 Samsung Electronics Co., Ltd.
 *		https://www.samsung.com
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 * Author: Jingoo Han <jg1.han@samsung.com>
 *         Xuran Yang <xuran.yang@bst.ai>
 */

#ifndef _PCIE_DESIGNWARE_H
#define _PCIE_DESIGNWARE_H

#include <linux/bitfield.h>
#include <linux/dma-mapping.h>
#include <linux/dma/edma.h>
#include <linux/irq.h>
#include <linux/msi.h>
#include <linux/pci.h>

#include <linux/pci-epc.h>
#include <linux/pci-epf.h>

#include <linux/reset.h>
#include <linux/reset-controller.h>

/* DWC PCIe IP-core versions (native support since v4.70a) */
#define DW_PCIE_VER_365A		0x3336352a
#define DW_PCIE_VER_460A		0x3436302a
#define DW_PCIE_VER_470A		0x3437302a
#define DW_PCIE_VER_480A		0x3438302a
#define DW_PCIE_VER_490A		0x3439302a
#define DW_PCIE_VER_520A		0x3532302a
#define DW_PCIE_VER_540A		0x3534302a

#define __dw_pcie_ver_cmp(_pci, _ver, _op) \
	((_pci)->version _op DW_PCIE_VER_ ## _ver)

#define dw_pcie_ver_is(_pci, _ver) __dw_pcie_ver_cmp(_pci, _ver, ==)

#define dw_pcie_ver_is_ge(_pci, _ver) __dw_pcie_ver_cmp(_pci, _ver, >=)

#define dw_pcie_ver_type_is(_pci, _ver, _type) \
	(__dw_pcie_ver_cmp(_pci, _ver, ==) && \
	 __dw_pcie_ver_cmp(_pci, TYPE_ ## _type, ==))

#define dw_pcie_ver_type_is_ge(_pci, _ver, _type) \
	(__dw_pcie_ver_cmp(_pci, _ver, ==) && \
	 __dw_pcie_ver_cmp(_pci, TYPE_ ## _type, >=))

/* Parameters for the waiting for link up routine */
#define LINK_WAIT_MAX_RETRIES		10
#define LINK_WAIT_USLEEP_MIN		90000
#define LINK_WAIT_USLEEP_MAX		100000

/* Parameters for the waiting for iATU enabled routine */
#define LINK_WAIT_MAX_IATU_RETRIES	5
#define LINK_WAIT_IATU			9

/* Synopsys-specific PCIe configuration registers */
#define PCIE_PORT_AFR			0x70C
#define PORT_AFR_N_FTS_MASK		GENMASK(15, 8)
#define PORT_AFR_N_FTS(n)		FIELD_PREP(PORT_AFR_N_FTS_MASK, n)
#define PORT_AFR_CC_N_FTS_MASK		GENMASK(23, 16)
#define PORT_AFR_CC_N_FTS(n)		FIELD_PREP(PORT_AFR_CC_N_FTS_MASK, n)
#define PORT_AFR_ENTER_ASPM		BIT(30)
#define PORT_AFR_L0S_ENTRANCE_LAT_SHIFT	24
#define PORT_AFR_L0S_ENTRANCE_LAT_MASK	GENMASK(26, 24)
#define PORT_AFR_L1_ENTRANCE_LAT_SHIFT	27
#define PORT_AFR_L1_ENTRANCE_LAT_MASK	GENMASK(29, 27)

#define PCIE_PORT_LINK_CONTROL		0x710
#define PORT_LINK_DLL_LINK_EN		BIT(5)
#define PORT_LINK_FAST_LINK_MODE	BIT(7)
#define PORT_LINK_MODE_MASK		GENMASK(21, 16)
#define PORT_LINK_MODE(n)		FIELD_PREP(PORT_LINK_MODE_MASK, n)
#define PORT_LINK_MODE_1_LANES		PORT_LINK_MODE(0x1)
#define PORT_LINK_MODE_2_LANES		PORT_LINK_MODE(0x3)
#define PORT_LINK_MODE_4_LANES		PORT_LINK_MODE(0x7)
#define PORT_LINK_MODE_8_LANES		PORT_LINK_MODE(0xf)

#define PCIE_PORT_DEBUG0		0x728
#define PORT_LOGIC_LTSSM_STATE_MASK	0x1f
#define PORT_LOGIC_LTSSM_STATE_L0	0x11
#define PORT_LOGIC_LTSSM_STATE_L0S	0x12
#define PORT_LOGIC_LTSSM_STATE_L1	0x14
#define PORT_LOGIC_LTSSM_STATE_L2	0x15
#define PCIE_PORT_DEBUG1		0x72C
#define PCIE_PORT_DEBUG1_LINK_UP		BIT(4)
#define PCIE_PORT_DEBUG1_LINK_IN_TRAINING	BIT(29)

#define PCIE_LINK_WIDTH_SPEED_CONTROL	0x80C
#define PORT_LOGIC_N_FTS_MASK		GENMASK(7, 0)
#define PORT_LOGIC_SPEED_CHANGE		BIT(17)
#define PORT_LOGIC_LINK_WIDTH_MASK	GENMASK(12, 8)
#define PORT_LOGIC_LINK_WIDTH(n)	FIELD_PREP(PORT_LOGIC_LINK_WIDTH_MASK, n)
#define PORT_LOGIC_LINK_WIDTH_1_LANES	PORT_LOGIC_LINK_WIDTH(0x1)
#define PORT_LOGIC_LINK_WIDTH_2_LANES	PORT_LOGIC_LINK_WIDTH(0x2)
#define PORT_LOGIC_LINK_WIDTH_4_LANES	PORT_LOGIC_LINK_WIDTH(0x4)
#define PORT_LOGIC_LINK_WIDTH_8_LANES	PORT_LOGIC_LINK_WIDTH(0x8)

#define PCIE_MSI_ADDR_LO		0x820
#define PCIE_MSI_ADDR_HI		0x824
#define PCIE_MSI_INTR0_ENABLE		0x828
#define PCIE_MSI_INTR0_MASK		0x82C
#define PCIE_MSI_INTR0_STATUS		0x830

#define GEN3_RELATED_OFF			0x890
#define GEN3_RELATED_OFF_GEN3_ZRXDC_NONCOMPL	BIT(0)
#define GEN3_RELATED_OFF_RXEQ_RGRDLESS_RXTS	BIT(13)
#define GEN3_RELATED_OFF_GEN3_EQ_DISABLE	BIT(16)
#define GEN3_RELATED_OFF_RATE_SHADOW_SEL_SHIFT	24
#define GEN3_RELATED_OFF_RATE_SHADOW_SEL_MASK	GENMASK(25, 24)

#define PCIE_PORT_MULTI_LANE_CTRL	0x8C0
#define PORT_MLTI_UPCFG_SUPPORT		BIT(7)

#define PCIE_VERSION_NUMBER		0x8F8
#define PCIE_VERSION_TYPE		0x8FC

/*
 * iATU inbound and outbound windows CSRs. Before the IP-core v4.80a each
 * iATU region CSRs had been indirectly accessible by means of the dedicated
 * viewport selector. The iATU/eDMA CSRs space was re-designed in DWC PCIe
 * v4.80a in a way so the viewport was unrolled into the directly accessible
 * iATU/eDMA CSRs space.
 */
#define PCIE_ATU_VIEWPORT		0x900
#define PCIE_ATU_REGION_DIR_IB		BIT(31)
#define PCIE_ATU_REGION_DIR_OB		0
#define PCIE_ATU_VIEWPORT_BASE		0x904
#define PCIE_ATU_UNROLL_BASE(dir, index) \
	(((index) << 9) | ((dir == PCIE_ATU_REGION_DIR_IB) ? BIT(8) : 0))
#define PCIE_ATU_VIEWPORT_SIZE		0x2C
#define PCIE_ATU_REGION_CTRL1		0x000
#define PCIE_ATU_INCREASE_REGION_SIZE	BIT(13)
#define PCIE_ATU_TYPE_MEM		0x0
#define PCIE_ATU_TYPE_IO		0x2
#define PCIE_ATU_TYPE_CFG0		0x4
#define PCIE_ATU_TYPE_CFG1		0x5
#define PCIE_ATU_TD			BIT(8)
#define PCIE_ATU_FUNC_NUM(pf)           ((pf) << 20)
#define PCIE_ATU_REGION_CTRL2		0x004
#define PCIE_ATU_ENABLE			BIT(31)
#define PCIE_ATU_BAR_MODE_ENABLE	BIT(30)
#define PCIE_ATU_VFBAR_MATCH_EN		BIT(26)
#define PCIE_ATU_VF_MATCH_EN		BIT(20)
#define PCIE_ATU_FUNC_NUM_MATCH_EN      BIT(19)
#define PCIE_ATU_LOWER_BASE		0x008
#define PCIE_ATU_UPPER_BASE		0x00C
#define PCIE_ATU_LIMIT			0x010
#define PCIE_ATU_LOWER_TARGET		0x014
#define PCIE_ATU_BUS(x)			FIELD_PREP(GENMASK(31, 24), x)
#define PCIE_ATU_DEV(x)			FIELD_PREP(GENMASK(23, 19), x)
#define PCIE_ATU_FUNC(x)		FIELD_PREP(GENMASK(18, 16), x)
#define PCIE_ATU_UPPER_TARGET		0x018
#define PCIE_ATU_REGION_CTRL3		0x01C
#define PCIE_ATU_VFUNC_NUM(vf)		(vf)
#define PCIE_ATU_UPPER_LIMIT		0x020
#define PCIE_ATU_VF_ACTIVE		BIT(31)

#define PCIE_MISC_CONTROL_1_OFF		0x8BC
#define PCIE_DBI_RO_WR_EN		BIT(0)

#define PCIE_MSIX_DOORBELL		0x948
#define PCIE_MSIX_DOORBELL_PF_SHIFT	24
#define PCIE_MSIX_DOORBELL_VF_SHIFT	16
#define PCIE_MSIX_DOORBELL_VF_ACTIVE	BIT(15)

/*
 * eDMA CSRs. DW PCIe IP-core v4.70a and older had the eDMA registers accessible
 * over the Port Logic registers space. Afterwards the unrolled mapping was
 * introduced so eDMA and iATU could be accessed via a dedicated registers
 * space.
 */
#define PCIE_DMA_VIEWPORT_BASE		0x970
#define PCIE_DMA_UNROLL_BASE		0x80000
#define PCIE_DMA_CTRL			0x008
#define PCIE_DMA_NUM_WR_CHAN		GENMASK(3, 0)
#define PCIE_DMA_NUM_RD_CHAN		GENMASK(19, 16)

#define PCIE_PL_CHK_REG_CONTROL_STATUS			0xB20
#define PCIE_PL_CHK_REG_CHK_REG_START			BIT(0)
#define PCIE_PL_CHK_REG_CHK_REG_CONTINUOUS		BIT(1)
#define PCIE_PL_CHK_REG_CHK_REG_COMPARISON_ERROR	BIT(16)
#define PCIE_PL_CHK_REG_CHK_REG_LOGIC_ERROR		BIT(17)
#define PCIE_PL_CHK_REG_CHK_REG_COMPLETE		BIT(18)

#define PCIE_PL_CHK_REG_ERR_ADDR			0xB28

/*
 * iATU Unroll-specific register definitions
 * From 4.80 core version the address translation will be made by unroll
 */
#define PCIE_ATU_UNR_REGION_CTRL1	0x00
#define PCIE_ATU_UNR_REGION_CTRL2	0x04
#define PCIE_ATU_UNR_LOWER_BASE		0x08
#define PCIE_ATU_UNR_UPPER_BASE		0x0C
#define PCIE_ATU_UNR_LOWER_LIMIT	0x10
#define PCIE_ATU_UNR_LOWER_TARGET	0x14
#define PCIE_ATU_UNR_UPPER_TARGET	0x18
#define PCIE_ATU_UNR_UPPER_LIMIT	0x20

/*
 * RAS-DES register definitions
 */
#define PCIE_RAS_DES_EVENT_COUNTER_CONTROL				0x8
#define EVENT_COUNTER_ALL_CLEAR							0x3
#define EVENT_COUNTER_PER_EVENT_OFF						0x1
#define EVENT_COUNTER_PER_EVENT_ON						0x3
#define EVENT_COUNTER_ENABLE_ALL_OFF					0x7
#define EVENT_COUNTER_ENABLE_ALL_ON						0x7
#define EVENT_COUNTER_ENABLE_SHIFT						2
#define EVENT_COUNTER_GROUP_SEL_MASK					GENMASK(3, 0)
#define EVENT_COUNTER_LANE_SEL_MASK					    GENMASK(3, 0)
#define EVENT_COUNTER_EVENT_SEL_MASK					GENMASK(7, 0)
#define EVENT_COUNTER_LANE_SEL_SHIFT                    8
#define EVENT_COUNTER_EVENT_SEL_SHIFT					16
#define EVENT_COUNTER_GROUP_SEL_SHIFT					24
#define PCIE_RAS_DES_EVENT_COUNTER_DATA					0xc

#define EVENT_COUNTER_GROUP_0		    				0x00
/* - GROUP0 EVENT - */
#define EVENT_COUNTER_EVENT_EBUF_OVERFLOW       		0x00
#define EVENT_COUNTER_EVENT_EBUF_UNDER_FUN       		0x01
#define EVENT_COUNTER_EVENT_DECODE_ERR      			0x02
#define EVENT_COUNTER_EVENT_RUNNING_DISPARITY_ERR   	0x03
#define EVENT_COUNTER_EVENT_SKP_OS_PARITY_ERR       	0x04
#define EVENT_COUNTER_EVENT_SYNC_HEADER_ERROR      		0x05
#define EVENT_COUNTER_EVENT_RX_VALID_DE_ASSERTION   	0x06
#define EVENT_COUNTER_EVENT_CTL_SKP_OS_PARITY_ERR   	0x07
#define EVENT_COUNTER_EVENT_1ST_RETIMER_PARITY_ERR  	0x08
#define EVENT_COUNTER_EVENT_2ND_RETIMER_PARITY_ERR      0x09
#define EVENT_COUNTER_EVENT_MARGIN_CRC_PARITY_ERR       0x0A

#define EVENT_COUNTER_GROUP_1		    				0x1
/* - GROUP1 EVENT - */
#define EVENT_COUNTER_EVENT_DETECT_EI_INFER             0x05
#define EVENT_COUNTER_EVENT_RECEIVER_ERR                0x06
#define EVENT_COUNTER_EVENT_RX_RECOVERY_REQ             0x07
#define EVENT_COUNTER_EVENT_N_FTS_TIMEOUT               0x08
#define EVENT_COUNTER_EVENT_FRAMEING_ERR                0x09
#define EVENT_COUNTER_EVENT_DESKEW_ERR                  0x0a

#define EVENT_COUNTER_GROUP_2		    				0x2
/* - GROUP2 EVENT - */
#define EVENT_COUNTER_EVENT_BAD_TLP                     0x00
#define EVENT_COUNTER_EVENT_LCRC_ERR                    0x01
#define EVENT_COUNTER_EVENT_BAD_DLLP                    0x02
#define EVENT_COUNTER_EVENT_REPLAT_NUM_ROLLOVER         0x03
#define EVENT_COUNTER_EVENT_REPLAY_TIMEOUT              0x04
#define EVENT_COUNTER_EVENT_RX_NAK_DLLP                 0x05
#define EVENT_COUNTER_EVENT_TX_NAK_DLLP                 0x06
#define EVENT_COUNTER_EVENT_RETRY_TLP                   0x07

#define EVENT_COUNTER_GROUP_3		    				0x3
/* - GROUP3 EVENT - */
#define EVENT_COUNTER_EVENT_FC_TIMEOUT                  0x00
#define EVENT_COUNTER_EVENT_POISONED_TLP                0x01
#define EVENT_COUNTER_EVENT_ECRC_ERR                    0x02
#define EVENT_COUNTER_EVENT_UNSUPPORTED_REQ             0x03
#define EVENT_COUNTER_EVENT_COMPLETER_ABORT             0x04
#define EVENT_COUNTER_EVENT_COMPLETETION_TIMEOUT        0x05

#define EVENT_COUNTER_GROUP_4		    				0x4
/* - GROUP4 EVENT - */
#define EVENT_COUNTER_EVENT_EBUF_SKP_ADD                0x00
#define EVENT_COUNTER_EVENT_EBUG_SKP_DEL                0x01


#define EVENT_COUNTER_GROUP_5		   					0x5
/* - GROUP5 EVENT - */
#define EVENT_COUNTER_EVENT_L0_TO_RECOVERY_ENTRY		0x0
#define EVENT_COUNTER_EVENT_L1_TO_RECOVERY_ENTRY	    0x1
#define EVENT_COUNTER_EVENT_Tx_L0S						0x2
#define EVENT_COUNTER_EVENT_Rx_L0S						0x3
#define EVENT_COUNTER_EVENT_ASPM_L1_REJECT              0x04
#define EVENT_COUNTER_EVENT_L1							0x5
#define EVENT_COUNTER_EVENT_L1_CPM  					0x06
#define EVENT_COUNTER_EVENT_L1_1						0x7
#define EVENT_COUNTER_EVENT_L1_2						0x8
#define EVENT_COUNTER_EVENT_L1_SHORT_DURATION   		0x09
#define EVENT_COUNTER_EVENT_L1_2_ABORT          		0x0A
#define EVENT_COUNTER_EVENT_L2_ENTRY           			0x0B
#define EVENT_COUNTER_EVENT_SPEED_CHANGE        		0x0C
#define EVENT_COUNTER_EVENT_LINK_WIDTH_CHANGE   		0x0D
// #define EVENT_COUNTER_EVENT_                    0x0E  //RESERVED

#define EVENT_COUNTER_GROUP_6		    				0x6
/* - GROUP6 EVENT - */
#define EVENT_COUNTER_EVENT_TX_ACK_DLLP                 0x00
#define EVENT_COUNTER_EVENT_TX_UPDATE_FC_DLLP           0x01
#define EVENT_COUNTER_EVENT_RX_ACK_DLLP                 0x02
#define EVENT_COUNTER_EVENT_RX_UPDATE_FC_DLLP           0x03
#define EVENT_COUNTER_EVENT_RX_NULLI_TLP                0x04
#define EVENT_COUNTER_EVENT_TX_NULLI_TLP        		0x05
#define EVENT_COUNTER_EVENT_RX_DUP_TLP       		    0x06

#define EVENT_COUNTER_GROUP_7		    				0x7
/* - GROUP7 EVENT - */
#define EVENT_COUNTER_EVENT_TX_MEM_WRITE                0x00
#define EVENT_COUNTER_EVENT_TX_MEM_READ                 0x01
#define EVENT_COUNTER_EVENT_TX_CONFIG_WRITE             0x02
#define EVENT_COUNTER_EVENT_TX_CONFIG_READ              0x03
#define EVENT_COUNTER_EVENT_TX_IO_WRITE                 0x04
#define EVENT_COUNTER_EVENT_TX_IO_READ        		    0x05
#define EVENT_COUNTER_EVENT_TX_COMPLE_WITHOUT_DATA      0x06
#define EVENT_COUNTER_EVENT_TX_COMPLE_WITH_DATA      	0x07
#define EVENT_COUNTER_EVENT_TX_MSG_TLP     				0x08
#define EVENT_COUNTER_EVENT_TX_ATOMIC    				0x09
#define EVENT_COUNTER_EVENT_TX_TLP_WITH_PREFIX    		0x0A
#define EVENT_COUNTER_EVENT_RX_MEM_WRITE                0x0B
#define EVENT_COUNTER_EVENT_RX_MEM_READ                 0x0C
#define EVENT_COUNTER_EVENT_RX_CONFIG_WRITE             0x0D
#define EVENT_COUNTER_EVENT_RX_CONFIG_READ              0x0E
#define EVENT_COUNTER_EVENT_RX_IO_WRITE                 0x0F
#define EVENT_COUNTER_EVENT_RX_IO_READ        		    0x10
#define EVENT_COUNTER_EVENT_RX_COMPLE_WITHOUT_DATA      0x11
#define EVENT_COUNTER_EVENT_RX_COMPLE_WITH_DATA      	0x12
#define EVENT_COUNTER_EVENT_RX_MSG_TLP     				0x13
#define EVENT_COUNTER_EVENT_RX_ATOMIC    				0x14
#define EVENT_COUNTER_EVENT_RX_TLP_WITH_PREFIX    		0x15
#define EVENT_COUNTER_EVENT_TX_CCIX_TLP    				0x16
#define EVENT_COUNTER_EVENT_RX_CCIX_TLP    		        0x15

/*
 * RAS-DP register definitions
 */
#define RASDP_UNCORR_COUNTER_SELECTIN_MASK				GENMASK(7, 0)
#define RASDP_UNCORR_COUNTER_REG_SELECTIN_MASK			GENMASK(3, 0)
#define RASDP_UNCORR_COUNTER_SELECTION_SHIFT            24
#define RASDP_UNCORR_COUNTER_REG_SELECTION_SHIFT        20
#define RASDP_UNCORR_COUNTER_CTRL_OFF                   0x14
#define RASDP_UNCORR_COUNT_REPORT_OFF                   0x18
#define RASDP_UNCORR_CLEAR_COUNTERS_MASK                0x1

/*
*
* AXI interface Supervisor Status
* 
*/
#define AXI_SC 											0x00 		// Successful Completion
#define AXI_UR 											0x01		// Unsupported Request
#define AXI_RRS 										0x02 		// Request Retry Status
#define AXI_CA 											0x03        // Completer Abort
#define AXI_MASTR_READ_STATUS_SECTION_SHIFT             0x0
#define AXI_MASTR_WRITE_STATUS_SECTION_SHIFT            0x10
#define AXI_STATUS_DATA_MASK                            0x3

/*
 * The default address offset between dbi_base and atu_base. Root controller
 * drivers are not required to initialize atu_base if the offset matches this
 * default; the driver core automatically derives atu_base from dbi_base using
 * this offset, if atu_base not set.
 */
#define DEFAULT_DBI_ATU_OFFSET (0x3 << 20)
#define DEFAULT_DBI_DMA_OFFSET PCIE_DMA_UNROLL_BASE

#define MAX_MSI_IRQS			256
#define MAX_MSI_IRQS_PER_CTRL		32
#define MAX_MSI_CTRLS			(MAX_MSI_IRQS / MAX_MSI_IRQS_PER_CTRL)
#define MSI_REG_CTRL_BLOCK_SIZE		12
#define MSI_DEF_NUM_VECTORS		32

/* Maximum number of inbound/outbound iATUs */
#define MAX_IATU_IN			256
#define MAX_IATU_OUT			256

/* Default eDMA LLP memory size */
#define DMA_LLP_MEM_SIZE		PAGE_SIZE

/* BST Target0 Address Definition */
#define BST_TRGT0_BAR			0
#define BST_TRGT0_DOORBELL_BASE		0xE00
#define BST_TRGT0_DOORBELL_OFF(func)	(0x10 * (func))
#define BST_TRGT0_HDMA_BASE		0x4000

struct dw_pcie;
struct dw_pcie_rp;
struct dw_pcie_ep;

enum dw_pcie_device_mode {
	DW_PCIE_UNKNOWN_TYPE,
	DW_PCIE_EP_TYPE,
	DW_PCIE_LEG_EP_TYPE,
	DW_PCIE_RC_TYPE,
};

struct dw_pcie_host_ops {
	int (*host_init)(struct dw_pcie_rp *pp);
	void (*host_deinit)(struct dw_pcie_rp *pp);
	int (*msi_host_init)(struct dw_pcie_rp *pp);
};

struct dw_pcie_rp {
	bool			has_msi_ctrl:1;
	bool			cfg0_io_shared:1;
	u64			cfg0_base;
	void __iomem		*va_cfg0_base;
	u32			cfg0_size;
	resource_size_t		io_base;
	phys_addr_t		io_bus_addr;
	u32			io_size;
	int			irq;
	const struct dw_pcie_host_ops *ops;
	int			msi_irq[MAX_MSI_CTRLS];
	struct irq_domain	*irq_domain;
	struct irq_domain	*msi_domain;
	dma_addr_t		msi_data;
	struct irq_chip		*msi_irq_chip;
	u32			num_vectors;
	u32			irq_mask[MAX_MSI_CTRLS];
	struct pci_host_bridge  *bridge;
	raw_spinlock_t		lock;
	DECLARE_BITMAP(msi_irq_in_use, MAX_MSI_IRQS);
};

struct dw_pcie_ep_ops {
	void	(*ep_init)(struct dw_pcie_ep *ep);
	int	(*raise_irq)(struct dw_pcie_ep *ep, u8 func_no, u8 vfunc_no,
			     enum pci_epc_irq_type type, u16 interrupt_num);
	const struct pci_epc_features* (*get_features)(struct dw_pcie_ep *ep,
					u8 func_no, u8 vfunc_no);
	/*
	 * Provide a method to implement the different func config space
	 * access for different platform, if different func have different
	 * offset, return the offset of func. if use write a register way
	 * return a 0, and implement code in callback function of platform
	 * driver.
	 */
	unsigned int (*func_conf_select)(struct dw_pcie_ep *ep, u8 func_no, u8 vfunc_no);
};

struct dw_pcie_ep_func {
	struct list_head	list;
	u8			func_no;
	u8			vfunc_no;
	bool			contain_vf;
	u8			msi_cap;	/* MSI capability offset */
	u8			msix_cap;	/* MSI-X capability offset */
	u8			bar_to_atu[PCI_STD_NUM_BARS];
	struct pci_epf_bar	*epf_bar[PCI_STD_NUM_BARS];
};

struct dw_pcie_ep {
	struct pci_epc		*epc;
	struct list_head	func_list;
	const struct dw_pcie_ep_ops *ops;
	phys_addr_t		phys_base;
	size_t			addr_size;
	size_t			page_size;
	phys_addr_t		*outbound_addr;
	unsigned long		*ib_window_map;
	unsigned long		*ob_window_map;
	void __iomem		*msi_mem;
	phys_addr_t		msi_mem_phys;
};

struct dw_pcie_ops {
	u64	(*cpu_addr_fixup)(struct dw_pcie *pcie, u64 cpu_addr);
	u32	(*read_dbi)(struct dw_pcie *pcie, void __iomem *base, u32 reg,
			    size_t size);
	void	(*write_dbi)(struct dw_pcie *pcie, void __iomem *base, u32 reg,
			     size_t size, u32 val);
	void    (*write_dbi2)(struct dw_pcie *pcie, void __iomem *base, u32 reg,
			      size_t size, u32 val);
	int	(*link_up)(struct dw_pcie *pcie);
	int	(*start_link)(struct dw_pcie *pcie);
	void	(*stop_link)(struct dw_pcie *pcie);
};

struct pcie_cfg_window {
	int		ob_win;
	u32		busdev;
	int		type;
	void __iomem	*va_cfg0_base;
	u64		cfg0_base;
};

struct dw_pcie {
	struct device		*dev;
	void __iomem		*dbi_base;
	void __iomem		*dbi_base2;
	void __iomem		*atu_base;
	size_t			atu_size;
	u32			num_ib_windows;
	u32			num_ob_windows;
	u32			region_align;
	u64			region_limit;
	struct dw_pcie_rp	pp;
	struct dw_pcie_ep	ep;
	const struct dw_pcie_ops *ops;
	u32			version;
	u32			type;
	int			num_lanes;
	int			link_gen;
	u8			n_fts[2];
	struct dw_edma_chip	edma;
	bool			iatu_unroll_enabled: 1;

	#ifdef CONFIG_ARCH_BSTC1200
	struct pcie_cfg_window	cfg_win[8];
	#endif
};

#ifdef CONFIG_PCIE_BST_DIAGNOSTIC
typedef struct bst_pcie_diagnostic_t {
	u32 ras_des_cap;
	u32 ras_dp_cap;
	u32 resdes_monitor_psm;
	u32 resdes_monitor_periodic;
    struct task_struct* resdes_counter_monitor_tsk;
	u32 reg_rdback_monitor_psm;
	u32 reg_rdback_monitor_enable;
	u32 smlh_ltssm_state_monitor_psm;
	u32 smlh_ltssm_state_monitor_periodic;
	struct task_struct* smlh_ltssm_state_task;
	u32 axi_monitor_psm;
	u32 dbi_access_monitor_psm;
	u32 dbi_access_failed_count;
	u32 dbi_access_monitor_periodic;
	struct task_struct* dbi_access_monitor_task;
	u32 rasdp_error_mode_monitor_psm;
	u32 rasdp_error_mode_monitor_periodic;
	struct task_struct* rasdp_error_mode_monitor_task;
	u32 cr_check_safety_monitor_psm;
	u32 cr_check_safety_monitor_periodic;
	u32 cr_check_safety_monitor_enable;
	struct task_struct* cr_check_safety_monitor_task;
	u32 ext_sram_access_psm;
	u32 ext_sram_access_periodic;
	u16 ext_sram_access_addr;
	struct task_struct* ext_sram_access_task;
	u32 tx2rx_monitor_psm;
	u32 tx2rx_monitor_periodic;
	struct task_struct* tx2rx_loopback_monitor_task;
	u8 diag_status;
}bst_pcie_diag, *bst_pcie_diagnostic;
#endif

struct bst_pcie {
	struct dw_pcie			*pci;
	enum dw_pcie_device_mode	mode;

	struct pcie_phy			*phy;
	u32				chip_type;
	u32				ctrl_id;
	bool				is_pre_init;
	int				reset_gpio;
	bool				gpio_active_high;
	int				legacy_parent_irq;
	struct irq_domain		*legacy_irq_domain;
	raw_spinlock_t			legacy_irq_lock;

	spinlock_t			share_dbi_lock;

	int				db_irq;
	u32				db_irq_num;
	unsigned long			*db_irq_win;
	struct list_head		db_irq_list;
	spinlock_t			db_lock;
#ifdef CONFIG_PCIE_BST_DIAGNOSTIC
	bst_pcie_diagnostic bst_pcie_diag;
	bool pcie_diagnostic_init_done;
#endif

#ifdef CONFIG_PCIE_BST_RESET
	struct reset_controller_dev rcdev;
#endif
};

#define to_dw_pcie_from_pp(port) container_of((port), struct dw_pcie, pp)

#define to_dw_pcie_from_ep(endpoint)   \
		container_of((endpoint), struct dw_pcie, ep)

void dw_pcie_version_detect(struct dw_pcie *pci);

u8 dw_pcie_find_capability(struct dw_pcie *pci, u8 cap);
u16 dw_pcie_find_ext_capability(struct dw_pcie *pci, u8 cap);

int dw_pcie_read(void __iomem *addr, int size, u32 *val);
int dw_pcie_write(void __iomem *addr, int size, u32 val);

u32 dw_pcie_read_dbi(struct dw_pcie *pci, u32 reg, size_t size);
void dw_pcie_write_dbi(struct dw_pcie *pci, u32 reg, size_t size, u32 val);
void dw_pcie_write_dbi2(struct dw_pcie *pci, u32 reg, size_t size, u32 val);
int dw_pcie_link_up(struct dw_pcie *pci);
void dw_pcie_upconfig_setup(struct dw_pcie *pci);
int dw_pcie_wait_for_link(struct dw_pcie *pci);
int dw_pcie_prog_outbound_atu(struct dw_pcie *pci, int index, int type,
			      u64 cpu_addr, u64 pci_addr, u64 size);
int dw_pcie_prog_ep_outbound_atu(struct dw_pcie *pci, u8 func_no, u8 vfunc_no, int index,
				 int type, u64 cpu_addr, u64 pci_addr, u64 size);
int dw_pcie_prog_inbound_atu(struct dw_pcie *pci, u8 func_no, u8 vfunc_no,
				int index, int type, u64 cpu_addr, u8 bar);
void dw_pcie_disable_atu(struct dw_pcie *pci, u32 dir, int index);
void dw_pcie_setup(struct dw_pcie *pci);
void dw_pcie_iatu_detect(struct dw_pcie *pci);
int dw_pcie_edma_detect(struct dw_pcie *pci);
void dw_pcie_edma_remove(struct dw_pcie *pci);
u32 bst_pcie_read_dbi(struct dw_pcie *pci, void __iomem *base,
		      u32 reg, size_t size);
void bst_pcie_write_dbi(struct dw_pcie *pci, void __iomem *base,
			u32 reg, size_t size, u32 val);
void bst_pcie_write_dbi2(struct dw_pcie *pci, void __iomem *base,
			 u32 reg, size_t size, u32 val);

static inline void dw_pcie_writel_dbi(struct dw_pcie *pci, u32 reg, u32 val)
{
	dw_pcie_write_dbi(pci, reg, 0x4, val);
}

static inline u32 dw_pcie_readl_dbi(struct dw_pcie *pci, u32 reg)
{
	return dw_pcie_read_dbi(pci, reg, 0x4);
}

static inline void dw_pcie_writew_dbi(struct dw_pcie *pci, u32 reg, u16 val)
{
	dw_pcie_write_dbi(pci, reg, 0x2, val);
}

static inline u16 dw_pcie_readw_dbi(struct dw_pcie *pci, u32 reg)
{
	return dw_pcie_read_dbi(pci, reg, 0x2);
}

static inline void dw_pcie_writeb_dbi(struct dw_pcie *pci, u32 reg, u8 val)
{
	dw_pcie_write_dbi(pci, reg, 0x1, val);
}

static inline u8 dw_pcie_readb_dbi(struct dw_pcie *pci, u32 reg)
{
	return dw_pcie_read_dbi(pci, reg, 0x1);
}

static inline void dw_pcie_writel_dbi2(struct dw_pcie *pci, u32 reg, u32 val)
{
	dw_pcie_write_dbi2(pci, reg, 0x4, val);
}

static inline void dw_pcie_dbi_ro_wr_en(struct dw_pcie *pci)
{
	u32 reg;
	u32 val;

	reg = PCIE_MISC_CONTROL_1_OFF;
	val = dw_pcie_readl_dbi(pci, reg);
	val |= PCIE_DBI_RO_WR_EN;
	dw_pcie_writel_dbi(pci, reg, val);
}

static inline void dw_pcie_dbi_ro_wr_dis(struct dw_pcie *pci)
{
	u32 reg;
	u32 val;

	reg = PCIE_MISC_CONTROL_1_OFF;
	val = dw_pcie_readl_dbi(pci, reg);
	val &= ~PCIE_DBI_RO_WR_EN;
	dw_pcie_writel_dbi(pci, reg, val);
}

static inline int dw_pcie_start_link(struct dw_pcie *pci)
{
	if (pci->ops && pci->ops->start_link)
		return pci->ops->start_link(pci);

	return 0;
}

static inline void dw_pcie_stop_link(struct dw_pcie *pci)
{
	if (pci->ops && pci->ops->stop_link)
		pci->ops->stop_link(pci);
}

#ifdef CONFIG_PCIE_BST_HOST
irqreturn_t dw_handle_msi_irq(struct dw_pcie_rp *pp);
int dw_pcie_setup_rc(struct dw_pcie_rp *pp);
int dw_pcie_host_init(struct dw_pcie_rp *pp);
void dw_pcie_host_deinit(struct dw_pcie_rp *pp);
int dw_pcie_allocate_domains(struct dw_pcie_rp *pp);
void __iomem *dw_pcie_own_conf_map_bus(struct pci_bus *bus, unsigned int devfn,
				       int where);
#else
static inline irqreturn_t dw_handle_msi_irq(struct dw_pcie_rp *pp)
{
	return IRQ_NONE;
}

static inline int dw_pcie_setup_rc(struct dw_pcie_rp *pp)
{
	return 0;
}

static inline int dw_pcie_host_init(struct dw_pcie_rp *pp)
{
	return 0;
}

static inline void dw_pcie_host_deinit(struct dw_pcie_rp *pp)
{
}

static inline int dw_pcie_allocate_domains(struct dw_pcie_rp *pp)
{
	return 0;
}
static inline void __iomem *dw_pcie_own_conf_map_bus(struct pci_bus *bus,
						     unsigned int devfn,
						     int where)
{
	return NULL;
}
#endif

#ifdef CONFIG_PCIE_BST_EP
void dw_pcie_ep_linkup(struct dw_pcie_ep *ep);
int dw_pcie_ep_init(struct dw_pcie_ep *ep);
int dw_pcie_ep_init_complete(struct dw_pcie_ep *ep);
void dw_pcie_ep_init_notify(struct dw_pcie_ep *ep);
void dw_pcie_ep_exit(struct dw_pcie_ep *ep);
int dw_pcie_ep_raise_legacy_irq(struct dw_pcie_ep *ep, u8 func_no);
int dw_pcie_ep_raise_msi_irq(struct dw_pcie_ep *ep, u8 func_no, u8 vfunc_no,
			     u8 interrupt_num);
int dw_pcie_ep_raise_msix_irq(struct dw_pcie_ep *ep, u8 func_no, u8 vfunc_no,
			     u16 interrupt_num);
int dw_pcie_ep_raise_msix_irq_doorbell(struct dw_pcie_ep *ep, u8 func_no, u8 vfunc_no,
				       u16 interrupt_num);
void dw_pcie_ep_reset_bar(struct dw_pcie *pci, enum pci_barno bar);
struct dw_pcie_ep_func *
dw_pcie_ep_get_func_from_ep(struct dw_pcie_ep *ep, u8 func_no, u8 vfunc_no);
int bst_pcie_ep_db_irq_alloc(struct pci_epc *epc, u8 func_no, u8 vfunc_no);
int bst_pcie_ep_db_irq_request(struct pci_epc *epc, u8 func_no, u8 vfunc_no, u32 irq_no,
			       int (*handler)(int irq, void *arg), void *arg);
void bst_pcie_ep_db_irq_free(struct pci_epc *epc, u8 func_no, u8 vfunc_no, u32 irq_no);
int bst_pcie_ep_db_info_get(struct pci_epc *epc, u8 func_no, u8 vfunc_no, u32 irq_no,
			    u32 *bar_no, u32 *offset, u32 *msg);
#else
static inline void dw_pcie_ep_linkup(struct dw_pcie_ep *ep)
{
}

static inline int dw_pcie_ep_init(struct dw_pcie_ep *ep)
{
	return 0;
}

static inline int dw_pcie_ep_init_complete(struct dw_pcie_ep *ep)
{
	return 0;
}

static inline void dw_pcie_ep_init_notify(struct dw_pcie_ep *ep)
{
}

static inline void dw_pcie_ep_exit(struct dw_pcie_ep *ep)
{
}

static inline int dw_pcie_ep_raise_legacy_irq(struct dw_pcie_ep *ep, u8 func_no)
{
	return 0;
}

static inline int dw_pcie_ep_raise_msi_irq(struct dw_pcie_ep *ep, u8 func_no,
					   u8 interrupt_num)
{
	return 0;
}

static inline int dw_pcie_ep_raise_msix_irq(struct dw_pcie_ep *ep, u8 func_no,
					   u16 interrupt_num)
{
	return 0;
}

static inline int dw_pcie_ep_raise_msix_irq_doorbell(struct dw_pcie_ep *ep,
						     u8 func_no,
						     u16 interrupt_num)
{
	return 0;
}

static inline void dw_pcie_ep_reset_bar(struct dw_pcie *pci, enum pci_barno bar)
{
}
static inline struct dw_pcie_ep_func *
dw_pcie_ep_get_func_from_ep(struct dw_pcie_ep *ep, u8 func_no, u8 vfunc_no)
{
	return NULL;
}

int bst_pcie_ep_db_irq_alloc(struct pci_epc *epc, u8 func_no, u8 vfunc_no)
{
	return -EINVAL;
}

int bst_pcie_ep_db_irq_request(struct pci_epc *epc, u8 func_no, u8 vfunc_no, u32 irq_no,
			       int (*handler)(int irq, void *arg), void *arg)
{
	return -EINVAL;
}

void bst_pcie_ep_db_irq_free(struct pci_epc *epc, u8 func_no, u8 vfunc_no, u32 irq_no)
{
}

int bst_pcie_ep_db_info_get(struct pci_epc *epc, u8 func_no, u8 vfunc_no, u32 irq_no,
			    u32 *bar_no, u32 *offset, u32 *msg)
{
	return -EINVAL;
}
#endif
#endif /* _PCIE_DESIGNWARE_H */
