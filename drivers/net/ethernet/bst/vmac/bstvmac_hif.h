// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021-2024 Black Sesame Technologies. All Rights Reserved.
 * Copyright (C) 2018 Synopsys, Inc. and/or its affiliates.
 */

#ifndef __BSTVMAC_HIF_H__
#define __BSTVMAC_HIF_H__

/* HIF MAC Registers */
#define HIF_VERSION								0x00
#define HIF_TX_POLL_CTRL						0x04
#define HIF_RX_POLL_CTRL						0x08
#define HIF_MISC								0x0c
#define HIF_TIMEOUT_REG							0x10
#define HIF_SOFT_RESET							0x14
#define HIF_DOORBELL_ENABLE_REG1				0x18
#define HIF_DOORBELL_ENABLE_REG2				0x1c
#define HIF_SINGLE_BIT_ECC_ERR0_EN				0x20
#define HIF_SINGLE_BIT_ECC_ERR0_STATUS			0x24
#define HIF_MULTI_BIT_ECC_ERR0_EN				0x28
#define HIF_MULTI_BIT_ECC_ERR0_STATUS			0x2c
#define HIF_MULTI_OR_ADDR_BIT_ECC_ERR0_EN		0x30
#define HIF_MULTI_OR_ADDR_BIT_ECC_ERR0_STATUS	0x34
#define HIF_ADDR_BIT_ECC_ERR0_EN				0x38
#define HIF_ADDR_BIT_ECC_ERR0_STATUS			0x3c
#define HIF_INT_SRC								0x40
#define HIF_INT_SRC_REG2						0x44
#define HIF_INT_SRC_REG3						0x48
#define HIF_WDT_INT_REG1						0x4c
#define HIF_WDT_INT_REG2						0x50
#define HIF_RX_PACKET_DROP_EN					0x5c
#define HIF_RX_PACKET_DROP_CNT					0x60
#define HIF_RX_PACKET_DROP_THRESHOLD			0x64
#define HIF_ERR_INT_SRC							0x68
#define HIF_ERR_INT_EN							0x6c
#define HIF_TX_FIFO_ERR_INT_SRC					0x70
#define HIF_TX_FIFO_ERR_INT_EN					0x74
#define HIF_RX_FIFO_ERR_INT_SRC					0x78
#define HIF_RX_FIFO_ERR_INT_EN					0x7c
#define HIF_TX_STATE							0x80
#define HIF_TX_ACTV								0x84
#define HIF_TX_CURR_CH_NO						0x88
#define HIF_DXR_TX_FIFO_CNT						0x8c
#define HIF_TX_CTRL_WORD_FIFO_CNT1				0x90
#define HIF_TX_CTRL_WORD_FIFO_CNT2				0x94
#define HIF_TX_BVALID_FIFO_CNT					0x98
#define HIF_TX_PKT_CNT1							0x9c
#define HIF_TX_PKT_CNT2							0xa0
#define HIF_RX_STATE							0xa4
#define HIF_RX_ACTV								0xa8
#define HIF_RX_CURR_CH_NO						0xac
#define HIF_DXR_RX_FIFO_CNT						0xb0
#define HIF_RX_CTRL_WORD_FIFO_CNT				0xb4
#define HIF_RX_BVALID_FIFO_CNT					0xb8
#define HIF_RX_PKT_CNT1							0xbc
#define HIF_RX_PKT_CNT2							0xc0
#define HIF_DMA_BASE							0xc4
#define HIF_DMA_BURST_SIZE						0xc8
#define HIF_RX_QUEUE_MAP_CH_NO					0xcc
#define HIF_LTC_PKT_CTRL						0xd0
#define HIF_RX_QUEUE_MAP_CH_NO_REG2				0xd4
#define HIF_RX_QUEUE_MAP_CH_NO_REG3				0xd8
#define HIF_RX_QUEUE_MAP_CH_NO_REG4				0xdc
#define HIF_SINGLE_BIT_ECC_ERR1_EN				0xe0
#define HIF_SINGLE_BIT_ECC_ERR1_STATUS			0xe4
#define HIF_MULTI_BIT_ECC_ERR1_EN				0xe8
#define HIF_MULTI_BIT_ECC_ERR1_STATUS			0xec
#define HIF_MULTI_OR_ADDR_BIT_ECC_ERR1_EN		0xf0
#define HIF_MULTI_OR_ADDR_BIT_ECC_ERR1_STATUS	0xf4
#define HIF_ADDR_BIT_ECC_ERR1_EN				0xf8
#define HIF_ADDR_BIT_ECC_ERR1_STATUS			0xfc

/* HIF Channel MAC Registers */
#define HIF_CTRL_CH(x)							(0x00 + (0x1000 * x))
#define HIF_RX_BDP_WR_LOW_ADDR_CH(x)			(0x04 + (0x1000 * x))
#define HIF_RX_BDP_WR_HIGH_ADDR_CH(x)			(0x08 + (0x1000 * x))
#define HIF_RX_BDP_RD_LOW_ADDR_CH(x)			(0x0c + (0x1000 * x))
#define HIF_RX_BDP_RD_HIGH_ADDR_CH(x)			(0x10 + (0x1000 * x))
#define HIF_TX_BDP_WR_LOW_ADDR_CH(x)			(0x14 + (0x1000 * x))
#define HIF_TX_BDP_WR_HIGH_ADDR_CH(x)			(0x18 + (0x1000 * x))
#define HIF_TX_BDP_RD_LOW_ADDR_CH(x)			(0x1c + (0x1000 * x))
#define HIF_TX_BDP_RD_HIGH_ADDR_CH(x)			(0x20 + (0x1000 * x))
#define HIF_RX_WRBK_BD_CH_BUFFER_SIZE(x)		(0x24 + (0x1000 * x))
#define HIF_RX_CH_START(x)						(0x28 + (0x1000 * x))
#define HIF_TX_WRBK_BD_CH_BUFFER_SIZE(x)		(0x2c + (0x1000 * x))
#define HIF_TX_CH_START(x)						(0x30 + (0x1000 * x))
#define HIF_DOORBELL_ADDR_LSB_CH(x)				(0x34 + (0x1000 * x))
#define HIF_DOORBELL_ADDR_MSB_CH(x)				(0x38 + (0x1000 * x))
#define HIF_RX_PACKET_DROP_CNT_CH(x)			(0x3c + (0x1000 * x))
#define HIF_CH_INT_SRC(x)						(0x60 + (0x1000 * x))
#define HIF_CH_INT_EN(x)						(0x64 + (0x1000 * x))
#define HIF_TX_RD_CURR_BD_LOW_ADDR_CH(x)		(0x80 + (0x1000 * x))
#define HIF_TX_RD_CURR_BD_HIGH_ADDR_CH(x)		(0x84 + (0x1000 * x))
#define HIF_TX_WR_CURR_BD_LOW_ADDR_CH(x)		(0x88 + (0x1000 * x))
#define HIF_TX_WR_CURR_BD_HIGH_ADDR_CH(x)		(0x8c + (0x1000 * x))
#define HIF_BDP_CH_TX_FIFO_CNT(x)				(0x90 + (0x1000 * x))
#define HIF_TX_DMA_STATUS_0_CH(x)				(0x94 + (0x1000 * x))
#define HIF_TX_STATUS_0_CH(x)					(0x98 + (0x1000 * x))
#define HIF_TX_STATUS_1_CH(x)					(0x9c + (0x1000 * x))
#define HIF_TX_PKT_CNT0_CH(x)					(0xa0 + (0x1000 * x))
#define HIF_TX_PKT_CNT1_CH(x)					(0xa4 + (0x1000 * x))
#define HIF_TX_PKT_CNT2_CH(x)					(0xa8 + (0x1000 * x))
#define HIF_RX_RD_CURR_BD_LOW_ADDR_CH(x)		(0xc0 + (0x1000 * x))
#define HIF_RX_RD_CURR_BD_HIGH_ADDR_CH(x)		(0xc4 + (0x1000 * x))
#define HIF_RX_WR_CURR_BD_LOW_ADDR_CH(x)		(0xc8 + (0x1000 * x))
#define HIF_RX_WR_CURR_BD_HIGH_ADDR_CH(x)		(0xcc + (0x1000 * x))
#define HIF_BDP_CH_RX_FIFO_CNT(x)				(0xd0 + (0x1000 * x))
#define HIF_RX_DMA_STATUS_0_CH(x)				(0xd4 + (0x1000 * x))
#define HIF_RX_STATUS_0_CH(x)					(0xd8 + (0x1000 * x))
#define HIF_RX_PKT_CNT0_CH(x)					(0xdc + (0x1000 * x))
#define HIF_RX_PKT_CNT1_CH(x)					(0xe0 + (0x1000 * x))
#define HIF_LTC_MAX_PKT_CH(x)					(0xe4 + (0x1000 * x))
#define HIF_ABS_INT_TIMER_CH(x)					(0xe8 + (0x1000 * x))
#define HIF_ABS_FRAME_COUNT_CH(x)				(0xec + (0x1000 * x))
#define HIF_INT_COAL_EN_CH(x)					(0xf0 + (0x1000 * x))

/* MAC Register */
#define CSR_SEQ_NUM_CHECK_EN					BIT(0)
#define CSR_BDPRD_AXI_WRITE_DONE				BIT(1)
#define CSR_BDPWR_AXI_WRITE_DONE				BIT(2)
#define CSR_RXDXR_AXI_WRITE_DONE				BIT(3)
#define CSR_TXDXR_AXI_WRITE_DONE				BIT(4)
#define CSR_AXI_WRITE_DONE						GENMASK(4, 1)
#define CSR_HIF_TIMEOUT_EN						BIT(5)
#define CSR_BD_START_SEQ_NUM					GENMASK(31, 16)

/* DMA Register */
#define HIF_CH_INT_STS        					BIT(0)
#define BDP_CSR_RX_CBD_CH_INT_STS   			BIT(1)
#define BDP_CSR_RX_PKT_CH_INT_STS  				BIT(2)
#define BDP_CSR_TX_CBD_CH_INT_STS   			BIT(3)
#define BDP_CSR_TX_PKT_CH_INT_STS  				BIT(4)

#define HIF_CH_INT_ENABLE        				BIT(0)
#define BDP_CSR_RX_CBD_CH_INT_EN   				BIT(1)
#define BDP_CSR_RX_PKT_CH_INT_EN  				BIT(2)
#define BDP_CSR_TX_CBD_CH_INT_EN   				BIT(3)
#define BDP_CSR_TX_PKT_CH_INT_EN  				BIT(4)

#define CSR_RX_BDP_CH_START_OUT					BIT(0)
#define CSR_TX_BDP_CH_START_OUT					BIT(0)
#define CSR_TX_DMA_EN_CH_OUT					BIT(0)
#define CSR_TX_BDP_POLL_CNTR_EN_CH_OUT			BIT(1)
#define CSR_RX_DMA_EN_CH_OUT					BIT(16)
#define CSR_RX_BDP_POLL_CNTR_EN_CH_OUT			BIT(17)

/* Descriptors */
#define VMAC_BD_DES0_DESC_EN	BIT(31)
#define VMAC_BD_DES0_DIR		BIT(20)
#define VMAC_BD_DES0_LAST_BD	BIT(19)
#define VMAC_BD_DES0_LIFM		BIT(18)
#define VMAC_BD_DES0_CBD_INT_EN	BIT(17)
#define VMAC_BD_DES0_PKT_INT_EN BIT(16)
#define VMAC_BD_DES0_SEQNUM		GENMASK(15, 0)
#define VMAC_BD_DES0_CTRL		GENMASK(20, 16)
#define VMAC_BD_DES1_NEXTPTR	GENMASK(31, 24)
#define VMAC_BD_DES1_HADDR		GENMASK(23, 16)
#define VMAC_BD_DES1_BUFLEN		GENMASK(15, 0)
#define VMAC_WRBD_DES0_LIFM		BIT(6)
#define VMAC_WRBD_DES0_CTRL     GENMASK(9, 4)
#define VMAC_WRBD_DES1_SEQNUM	GENMASK(31, 16)
#define VMAC_WRBD_DES1_BUFLEN	GENMASK(15, 0)
#define VMAC_WRBD_DES1_SEQNUM_SHIFT		16

#endif /* __BSTVMAC_HIF_H__ */
