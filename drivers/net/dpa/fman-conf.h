/* Copyright 2008-2011 Freescale Semiconductor, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     - Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * \section	Introduction
 *
 * This document details the low-level API that is provided for configuration of the Frame Manager
 * (FMan) block in the Freescale DPA architecture (P4080). These API is used to set up
 * classification, parsing, QoS and other FMan features.
 *
 * Both the FMan LLD/API and this document are work in progress. The implementation is incremental
 * and centered around, on one side, the functionality implemented in the FMan model/simulator
 * and, on the other side, the functionality used in the applications using this driver/API.
 *
 * As far as BG compliance goes, this driver/API is supposedly based on version/revision .79.
 * However, where a discrepancy between the BGs and the model/simulator was identified the decision
 * was made to be follow the model/simulator for the sake of maximizing the working feature set.
 *
 * The API is designed to have a consistent look-and-feel with the B/QMan drivers/API. Beyond
 * that, the code tries to follow the Linux coding style and conventions.
 *
 * \section	Virtualization
 *
 * Certain components/modules of the FMan can be virtualized easier then others. The \ref BMI,
 * \ref QMI, \ref Parser, \ref MAC and \ref MDIO can be virtualized easier, in the sense of being
 * able to assign them to a hypervisor partition and be able to control them without the need to
 * access global resources that might affect the stability and functionality of other hypervisor
 * partitions. This virtualization is achieved by having registers mapped in a 4 KB page that can
 * be mapped in the MMU to a single partition.
 *
 * Other blocks in the FMan such as the KeyGen, policer, etc. can be used by multiple partitions
 * but their cofiguration needs to be programmed/done by a single partition (control-plane
 * partition) on behalf of the other partitions (data-plane partitions).
 *
 * For more information on virtualization and partitioning please refer to the Freescale hypervisor
 * documentation.
 */

#ifndef __FMAN_CONF_H
#define __FMAN_CONF_H

/** \defgroup	Port	Port
 *  \defgroup	BMI	BMI - BMan interface
 *  \defgroup	QMI	QMI - QMan interface
 *  \defgroup	Parser	Parser
 *  \defgroup	KeyGen	KeyGen - Key generator
 *  \defgroup	DMA	DMA
 *  \defgroup	FPM	FPM - FMan processing manager
 *  \defgroup	Policer	Policer
 *  \defgroup	MAC	MAC
 *  \defgroup	MDIO	MDIO
 */

/* Register level abstraction */

/** \addtogroup	Port
 *  \details	A port contains the registers for the \ref BMI, \ref QMI and \ref Parser
 *  @{
 */

/** \addtogroup	BMI
 *  @{
 */

/** \internal
 *  \brief	BMI commmon port register layout
 *
 *  This is a FMan LLD private data structure and should _not_ be used anywhere including in the
 *  FMan LLD.
 *
 *  It is the method of choice for describing the FMan registers offsets and it exists only for that
 *  purpose.
 *
 */
struct fm_bmi_common_mmap_s {
	uint32_t	FMBM_INIT;	/* Initialization */
	uint32_t	FMBM_CFG[3];	/* Configuration */
	uint8_t		reserved0[0x10];
	uint32_t	FMBM_IVER;	/* Interrupt EVent */
	uint32_t	FMBM_IER;	/* Interrupt Enable */
	uint32_t	FMBM_IFR;	/* Interrupt Force */
	uint8_t		reserved1[0x14];
	uint32_t	FMBM_ARB[8];	/* Arbitration */
	uint32_t	FMBM_BCMA;	/* Buffers Command Mutual Exclusive Access */
	uint32_t	FMBM_EBC;	/* External Buffers Command */
	uint32_t	FMBM_EBCR;	/* External Buffers Command Result */
	uint32_t	FMBM_EPRH;	/* External PointeR High */
	uint32_t	FMBM_EPRL;	/* External PointeR Low */
	uint8_t		reserved2[0x18];
	uint32_t	FMBM_DTC[3];	/* Debug Trap Counter */
	uint8_t		reserved3[0x4];
	uint32_t	FMBM_DCV[12];	/* Debug Compare Value */
	uint32_t	FMBM_DCM[12];	/* Debug Compare Mask */
	uint32_t	FMBM_GDE;	/* Global Debug Enable */
	uint32_t	FMBM_PP[63];	/* Port Parameters */
	uint8_t		reserved4[0x4];
	uint32_t	FMBM_IPS;	/* Internal Probe Select */
	uint32_t	FMBM_PFS[63];	/* Port FIFO Size */
	uint32_t	FMBM_IPD;	/* Internal Probe Data */
	uint32_t	FMBM_PPID[63];	/* Port Partition ID */
} __aligned(0x400) __packed;

#define FM_BMI_COMMON_INIT_STR	0x80000000		/* Start */

#define FM_BMI_COMMON_CFG0_FBPS(n)	((n) << 16)	/* Free Buffer Pool Size */
#define FM_BMI_COMMON_CFG0_FMBM_OF(n)	(n)		/* Free Buffer Pool OFfset */
#define FM_BMI_COMMON_CFG1_TNTSKS(n)	((n) << 16)	/* Total Number of TaSKS */
#define FM_BMI_COMMON_CFG1_TDMA(n)	(n)		/* Total DMA */

/** \internal
 *  \brief		BMI offline parsing/host command port register layout
 *  \copydetails	fm_bmi_common_mmap_s
 */
struct fm_bmi_oh_mmap_s {
	uint8_t		reserved0[0x400];
} __aligned(0x400) __packed;

/** \internal
 *  \brief		BMI Rx port register layout
 *  \copydetails	fm_bmi_common_mmap_s
 */
struct fm_bmi_rx_mmap_s {
	uint32_t	FMBM_RCFG;	/* Configuration */
	uint32_t	FMBM_RST;	/* Status */
	uint32_t	FMBM_RDA;	/* DMA Attributes */
	uint32_t	FMBM_RFP;	/* FIFO Parameters */
	uint32_t	FMBM_RFED;	/* Frame End Data */
	uint32_t	FMBM_RICP;	/* Internal Context Parameters */
	uint32_t	FMBM_RIM;	/* Internal Margins */
	uint32_t	FMBM_REBM;	/* External Buffer Margins */
	uint32_t	FMBM_RFNE;	/* Frame Next Engine */
	uint32_t	FMBM_RFCA;	/* Frame Command Attributes */
	uint32_t	FMBM_RFPNE;	/* Frame Parser Next Engine */
	uint32_t	FMBM_RPSO;	/* Parse Start Offset */
	uint32_t	FMBM_RPP;	/* Policer Profile */
	uint32_t	FMBM_RCCB;	/* Coarse Classification Base */
	uint8_t		reserved0[0x8];
	uint32_t	FMBM_RPRAI[8];	/* Parse Results Array Initialization */
	uint32_t	FMBM_RFQID;	/* Frame Queue ID */
	uint32_t	FMBM_REFQID;	/* Error Frame Queue ID */
	uint32_t	FMBM_RFSDM;	/* Frame Status Discard Mask */
	uint32_t	FMBM_RFSEM;	/* Frame Status Error Mask */
	uint32_t	FMBM_RFENE;	/* Frame Enqueue Next Engine */
	uint8_t		reserved1[0x8c];
	uint32_t	FMBM_EBMPI[8];	/* Buffer Manager Pool Information */
	uint32_t	FMBM_ACNT[8];	/* Allocate CouNTer */
	uint8_t		reserved2[0x20];
	uint32_t	FMBM_CGM[8];	/* Congestion Group Map */
	uint32_t	FMBM_MPD;	/* Pool Depletion */
	uint8_t		reserved3[0x7c];
	uint32_t	FMBM_RSTC;	/* STatistics Counter */
	uint32_t	FMBM_RFRC;	/* FRame Counter */
	uint32_t	FMBM_RBFC;	/* Bad Frames Counter */
	uint32_t	FMBM_RLFC;	/* Large Frames Counter */
	uint32_t	FMBM_RFFC;	/* Filter Frames Counter */
	uint32_t	FMBM_RFDC;	/* Frames Discard Counter */
	uint32_t	FMBM_RFLDEC;	/* Frames List DMA Error Counter */
	uint32_t	FMBM_RODC;	/* Out of buffers Discard Counter */
	uint32_t	FMBM_RBDC;	/* Buffers Deallocate Counter */
	uint8_t		reserved4[0x5c];
	uint32_t	FMBM_RPC;	/* Performance Counters */
	uint32_t	FMBM_RPCP;	/* Performance Count Parameters */
	uint32_t	FMBM_RCCN;	/* Cycle CouNter */
	uint32_t	FMBM_RTUC;	/* Task Utilization Counter */
	uint32_t	FMBM_RRQUC;	/* Receive Queue Utilization Counter */
	uint32_t	FMBM_RDUC;	/* DMA Utilization Counter */
	uint32_t	FMBM_RFUC;	/* FIFO Utilization Counter */
	uint32_t	FMBM_RPAC;	/* Pause Activation Counter */
	uint8_t		reserved5[0x60];
	uint32_t	FMBM_RDCFG[3];	/* Debug ConFiGuration */
	uint8_t		reserved6[0x74];
	uint32_t	FMBM_RIBA;	/* Internal Buffer Allocation */
	uint32_t	FMBM_RIBD;	/* Internal Buffer Deallocation */
	uint32_t	FMBM_RIBC;	/* Internal Buffer Chain */
	uint32_t	FMBM_RIBLF;	/* Internal Buffer Link Follow */
	uint8_t		reserved7[0x70];
} __aligned(0x400) __packed;

#define FM_BMI_RX_RCFG_EN		0x80000000	/* ENable */
#define FM_BMI_RX_RCFG_FDOVR		0x02000000	/* Frame Discard OVerRide */
#define FM_BMI_RX_RCFG_IM		0x01000000	/* Independent Mode */

#define FM_BMI_RX_RFED_CSI(n)		((n) << 24)	/* CheckSum Ignore */
#define FM_BMI_RX_RFED_CFED(n)		((n) << 16)	/* Chop Frame's End Data */

#define FM_BMI_RX_RICP_ICEOF(n)		((n) << 16)	/* Internat Context External OFfset */
#define FM_BMI_RX_RICP_ICIOF(n)		((n) << 8)	/* Internat Context Internal OFfset */
#define FM_BMI_RX_RICP_ICSZ(n)		(n)		/* Internat Context SiZe */

#define FM_BMI_RX_RIM_FOF(n)		((n) << 24)	/* Frame OFfset */

#define FM_BMI_RX_REBM_BSM(n)		((n) << 16)	/* Buffer Start Margin */
#define FM_BMI_RX_REBM_BEM(n)		(n)		/* Buffer End Margin */

#define FM_BMI_RX_RFNE_NIA(n)		(n)		/* Next Invoked Action */

#define FM_BMI_RX_RFCA_OR		0x80000000	/* ORder definition */
#define FM_BMI_RX_RFCA_COLOR_GREEN	0x00000000	/* Default COLOR - green */
#define FM_BMI_RX_RFCA_COLOR_YELLOW	0x040000000	/* Default COLOR - yellow */
#define FM_BMI_RX_RFCA_COLOR_RED	0x08000000	/* Default COLOR - red */
#define FM_BMI_RX_RFCA_COLOR_OVERRIDE	0x0C000000	/* Default COLOR - override */
#define FM_BMI_RX_RFCA_SYNC(n)		((n) << 24)	/* SYNChronization attributes */
#define FM_BMI_RX_RFCA_MR(n)		((n) << 16)	/* Mode attRibutes */

#define FM_BMI_RX_RFPNE_HPNIA(n)	(n)		/* Hardware Parser Next Invoked Action */

#define FM_BMI_RX_RPSO_PSO(n)		(n)		/* Parsing Start Offset */

#define FM_BMI_RX_RPP_PNUM(n)		(n)		/* Policer profile */

#define FM_BMI_RX_RCCB_CBASE(n)		(n)		/* Coarse classification BASE */

#define FM_BMI_RX_RPRAI_PRA(n)		(n)		/* Parse Results Array */

#define FM_BMI_RX_RFQID_DFQID(n)	(n)		/* Default Frame Queue ID */

#define FM_BMI_RX_REFQID_EFQID(n)	(n)		/* Error Frame Queue ID */

#define FM_BMI_RX_RFSDM_FSDM(n)		(n)		/* Frame Status Discard Mask */

#define FM_BMI_RX_RFSEM_FSEM(n)		(n)		/* Frame Status Error Mask */

#define FM_BMI_RX_RFENE_NIA(n)		(n)		/* Next Invoked Action */

#define FM_BMI_RX_EBMPI_VAL		0x80000000	/* Valid */
#define FM_BMI_RX_EBMPI_ADCE		0x40000000	/* Allocate/Deallocate Coutners Enable */
#define FM_BMI_RX_EBMPI_BPID(n)		((n) << 16)	/* Buffer Pool ID */
#define FM_BMI_RX_EBMPI_PBS(n)		(n)		/* Pool Buffer Size */

/** \internal
 *  \brief		BMI Tx port register layout
 *  \copydetails	fm_bmi_common_mmap_s
 */
struct fm_bmi_tx_mmap_s {
	uint32_t	FMBM_TCFG;	/* Configuration */
	uint32_t	FMBM_TST;	/* STatus */
	uint32_t	FMBM_TDA;	/* DMA Attributes */
	uint32_t	FMBM_TFP;	/* FIFO Parameters */
	uint32_t	FMBM_TFED;	/* Frame End Data */
	uint32_t	FMBM_TICP;	/* Internal Context Paramenters */
	uint32_t	FMBM_TFNE;	/* Frame Next Engine */
	uint32_t	FMBM_TFCA;	/* Framme Command Attributes */
	uint32_t	FMBM_TCFQID;	/* Confirmation Queue ID */
	uint32_t	FMBM_TEFQID;	/* Error Frame Queue ID */
	uint32_t	FMBM_TFENE;	/* Frame Enqueue Next Engine */
	uint32_t	FMBM_TRLMTS;	/* Rate LiMiTer Scale */
	uint32_t	FMBM_TRLMT;	/* Rate LiMiTer */
	uint8_t		reserved0[0x1cc];
	uint32_t	FMBM_TSTC;	/* STatistics Counter */
	uint32_t	FMBM_TFRC;	/* FRame Counter */
	uint32_t	FMBM_TFDC;	/* Frames Discard Counter */
	uint32_t	FMBM_TFLEDC;	/* Framees Length Error Discard Counter */
	uint32_t	FMBM_TFUFDC;	/* Frames Unsupported Format Discard Counter */
	uint32_t	FMBM_TBDC;	/* Buffer Deallocate Counter */
	uint8_t		reserved1[0x68];
	uint32_t	FMBM_TPC;	/* Performance Counters */
	uint32_t	FMBM_TPCP;	/* Performance Count Parameters */
	uint32_t	FMBM_TCCN;	/* Cycle Counter */
	uint32_t	FMBM_TTUC;	/* Tasks Utilization Counter */
	uint32_t	FMBM_TTCQUC;	/* Transmit Confirm Queue Utilization Counter */
	uint32_t	FMBM_TDUC;	/* DMA Utilization Counter */
	uint32_t	FMBM_TFUC;	/* FIFO Utilization Counter */
	uint8_t		reserved2[0x64];
	uint32_t	FMBM_TDCFG[3];	/* Debug ConFiGuration */
	uint8_t		reserved3[0x74];
	uint32_t	FMBM_TIBA;	/* Internal Buffer Allocation */
	uint32_t	FMBM_TIBD;	/* Internal Buffer Deallocation */
	uint32_t	FMBM_TIBC;	/* Internal Buffer Chain */
	uint32_t	FMBM_TIBLF;	/* Internal Buffer Link Follow */
	uint8_t		reserved4[0x70];
} __aligned(0x400) __packed;

#define FM_BMI_TX_TCFG_EN		0x80000000	/* ENable  */
#define FM_BMI_TX_TCFG_IM		0x01000000	/* Independent Mode */

#define FM_BMI_TX_TFP_MFL(n)		((n) << 16)	/* Minimum Fill Level */
#define FM_BMI_TX_TFP_DPDE(n)		((n) << 12)	/* Dequeue Pipeline Depth */
#define FM_BMI_TX_TFP_FLCL(n)		(n)		/* FIFO Low Comfort Level */

#define FM_BMI_TX_TICP_ICEOF(n)		((n) << 16)	/* Internat Context External OFfset */
#define FM_BMI_TX_TICP_ICIOF(n)		((n) << 8)	/* Internat Context Internal OFfset */
#define FM_BMI_TX_TICP_ICSZ(n)		(n)		/* Internat Context SiZe */

#define FM_BMI_TX_TFNE_NIA(n)		(n)		/* Next Invoked Action */

#define FM_BMI_TX_TFCA_OR		0x80000000	/* ORder definition */
#define FM_BMI_TX_TFCA_COLOR_GREEN	0x00000000	/* Default COLOR - green */
#define FM_BMI_TX_TFCA_COLOR_YELLOW	0x040000000	/* Default COLOR - yellow */
#define FM_BMI_TX_TFCA_COLOR_RED	0x08000000	/* Default COLOR - red */
#define FM_BMI_TX_TFCA_COLOR_OVERRIDE	0x0C000000	/* Default COLOR - override */
#define FM_BMI_TX_TFCA_SYNC(n)		((n) << 24)	/* SYNChronization attributes */
#define FM_BMI_TX_TFCA_MR(n)		((n) << 16)	/* Mode attRibutes */

#define FM_BMI_TX_TCFQID_CFQID(n)	(n)		/* Confirmation Frame Queue ID */

#define FM_BMI_TX_TFENE_NIA(n)		(n)		/* Next Invoked Action */

/** \internal
 *  \brief		The BMI part of a port
 *  \copydetails	fm_bmi_common_mmap_s
 */
union fm_port_bmi_mmap_u {
	struct fm_bmi_common_mmap_s	common;
	struct fm_bmi_oh_mmap_s		oh;
	struct fm_bmi_rx_mmap_s		rx;
	struct fm_bmi_tx_mmap_s		tx;
} __aligned(0x400) __packed;

/** BMI common register offset
 *
 *  \param[in]	reg	The name of a register as it appears in the BG but without the \c "FMBM_"
 *			prefix
 *
 *  Use this macro to get the offset of a BMI common register. This offset is suitable for using
 *  with the fm_port_in() and fm_port_out() functions.
 *
 *  \par	Example:
 *
 *  Here is a sequence of code that initializes the internal (MURAM) free buffer pool by writing
 *  into the FMBM_INIT register:
 *
 *  \code
 *		...
 *	struct fman_port *common_port;
 *		...
 *	common_port = fm_port_create(0xfe480000);
 *		...
 *	fm_port_out(common_port, FM_BMI_COMMON(INIT), FM_BMI_COMMON_INIT_STR);
 *		...
 *	fm_port_destroy(common_port);
 *		...
 *  \endcode
 *
 *  \hideinitializer
 */
#define FM_BMI_COMMON(reg)	\
	(offsetof(struct fm_port_mmap_s, bmi) + offsetof(struct fm_bmi_common_mmap_s, FMBM_##reg))

/** BMI offline parsing/host command register offset
 *
 *  \param[in]	reg	The name of a register as it appears in the BG but without the \c "FMBM_"
 *			prefix
 *
 *  Use this macro to get the offset of a BMI offline parsing/host command register. This offset is
 *  suitable for using with the fm_port_in() and fm_port_out() functions.
 *
 *  \par	Example:
 *
 *  Here is a sequence of code that enables a O/H port for TX by writing into the FMBM_OCFG
 *  register:
 *
 *  \code
 *		...
 *	struct fman_port *oh_port;
 *		...
 *	oh_port = fm_port_create(0xfe481000);
 *		...
 *	fm_port_out(oh_port, FM_BMI_OH(OCFG), FM_BMI_OH_OCFG_EN);
 *		...
 *	fm_port_destroy(oh_port);
 *		...
 *  \endcode
 *
 *  \hideinitializer
 */
#define FM_BMI_OH(reg)		\
	(offsetof(struct fm_port_mmap_s, bmi) + offsetof(struct fm_bmi_oh_mmap_s, FMBM_##reg))

/** BMI RX register offset
 *
 *  \param[in]	reg	The name of a register as it appears in the BG but without the \c "FMBM_"
 *			prefix
 *
 *  Use this macro to get the offset of a BMI RX register. This offset is suitable for using with
 *  the fm_port_in() and fm_port_out() functions.
 *
 *  \par	Example:
 *
 *  Here is a sequence of code that reads the status of a RX port by reading from the FMBM_RST
 *  register:
 *
 *  \code
 *		...
 *	struct fman_port *rx_port;
 *		...
 *	rx_port = fm_port_create(0xfe488000);
 *		...
 *	while (fm_port_in(rx_port, FM_BMI_RX(RST)) & FM_BMI_RX_RST_BSY)
 *		cpu_relax();
 *	/ * Port not busy * /
 *		...
 *	fm_port_destroy(rx_port);
 *		...
 *  \endcode
 *
 *  \hideinitializer
 */
#define FM_BMI_RX(reg)		\
	(offsetof(struct fm_port_mmap_s, bmi) + offsetof(struct fm_bmi_rx_mmap_s, FMBM_##reg))

/** BMI TX register offset
 *
 *  \param[in]	reg	The name of a register as it appears in the BG but without the \c "FMBM_"
 *			prefix
 *
 *  Use this macro to get the offset of a BMI TX register. This offset is suitable for using with
 *  the fm_port_in() and fm_port_out() functions.
 *
 *  \par	Example:
 *
 *  Here is a sequence of code that enables TX by writing into the FMBM_TCFG register:
 *
 *  \code
 *		...
 *	struct fman_port *tx_port;
 *		...
 *	tx_port = fm_port_create(0xfe4a8000);
 *		...
 *	fm_port_out(tx_port, FM_BMI_TX(TCFG), FM_BMI_TX_RCFG_EN);
 *		...
 *	fm_port_destroy(tx_port);
 *		...
 *  \endcode
 *
 *  \hideinitializer
 */
#define FM_BMI_TX(reg)		\
	(offsetof(struct fm_port_mmap_s, bmi) + offsetof(struct fm_bmi_tx_mmap_s, FMBM_##reg))

/** @} */

/** \addtogroup	QMI
 *  @{
 */

/** \internal
 *  \brief		QMI common port register layout
 *  \copydetails	fm_bmi_common_mmap_s
 */
struct fm_qmi_common_mmap_s {
	uint32_t	FMQM_GC;	/* Global Configuration */
	uint8_t		reserved0[4];
	uint32_t	FMQM_EIE;	/* Error Interrupt Event */
	uint32_t	FMQM_EIEN;	/* Error Interrupt Enable */
	uint32_t	FMQM_EIF;	/* Error Interrupt Force */
	uint8_t		reserved1[0x3ec];
} __aligned(0x400) __packed;

#define FM_QMI_COMMON_GC_EN		0x80000000	/* ENable */
#define FM_QMI_COMMON_GC_ACC		0x40000000	/* All Counters Clear */
#define FM_QMI_COMMON_GC_STEN		0x10000000	/* global STatistics Enable */
#define FM_QMI_COMMON_GC_SR		0x01000000	/* Soft Reset */

#define FM_QMI_IRQ_DEE			0x80000000	/* Double-bit ECC Error */

/** \internal
 *  \brief		QMI offline parsing/host command and Tx port register layout
 *  \copydetails	fm_bmi_common_mmap_s
 */
struct fm_qmi_oh_tx_mmap_s {
	uint8_t		reserved0[0x30];
	uint32_t	FMQM_PDC;	/* Dequeue Configuration */
	uint8_t		reserved1[0x3cc];
} __aligned(0x400) __packed;

#define FM_QMI_OH_PDC_PRI		0x80000000	/* Priority */
#define FM_QMI_OH_PDC_OPT(n)		((n) << 28)	/* dequeue OPTions */
#define FM_QMI_OH_PDC_PF		0x02000000	/* PreFetch */
#define FM_QMI_OH_PDC_FRM		0x01000000	/* FRaMe count */
/* Sub-Portal */
#define FM_QMI_OH_PDC_SP_MASK		0x00f00000
#define FM_QMI_OH_PDC_SP(n)		((n) << 20 & FM_QMI_OH_PDC_SP_MASK)
#define FM_QMI_OH_PDC_WQ(n)		((n) << 16)	/* Work Queue */
#define FM_QMI_OH_PDC_BC(n)		(n)		/* Byte Count */

#define FM_QMI_TX_PDC_PRI		FM_QMI_OH_PDC_PRI
#define FM_QMI_TX_PDC_OPT		FM_QMI_OH_PDC_OPT
#define FM_QMI_TX_PDC_PF		FM_QMI_OH_PDC_PF
#define FM_QMI_TX_PDC_FRM		FM_QMI_OH_PDC_FRM
#define FM_QMI_TX_PDC_SP		FM_QMI_OH_PDC_SP
#define FM_QMI_TX_PDC_WQ		FM_QMI_OH_PDC_WQ
#define FM_QMI_TX_PDC_BC		FM_QMI_OH_PDC_BC

/** \internal
 *  \brief		QMI Rx port register layout
 *  \copydetails	fm_bmi_common_mmap_s
 */
struct fm_qmi_rx_mmap_s {
	uint8_t		reserved0[0x400];
} __aligned(0x400) __packed;

/** \internal
 *  \brief		The QMI part of a port
 *  \copydetails	fm_bmi_common_mmap_s
 */
union fm_port_qmi_mmap_u {
	struct fm_qmi_common_mmap_s	common;
	struct fm_qmi_oh_tx_mmap_s	oh;
	struct fm_qmi_rx_mmap_s		rx;
	struct fm_qmi_oh_tx_mmap_s	tx;
} __aligned(0x400) __packed;

/** QMI common register offset
 *
 *  \param[in]	reg	The name of a register as it appears in the BG but without the \c "FMQM_"
 *			prefix
 *
 *  Use this macro to get the offset of a QMI common register. This offset is suitable for using
 *  with the fm_port_in() and fm_port_out() functions.
 *
 *  \par	Example:
 *
 *  Here is a sequence of code that enables QMI enqueues and dequeues by writing into the FMQM_GC
 *  register:
 *
 *  \code
 *		...
 *	struct fman_port *common_port;
 *		...
 *	common_port = fm_port_create(0xfe480000);
 *		...
 *	fm_port_out(common_port, FM_QMI_COMMON(GC),
 *			FM_QMI_COMMON_GC_ENQ_EN | FM_QMI_COMMON_GC_DEQ_EN);
 *		...
 *	fm_port_destroy(common_port);
 *		...
 *  \endcode
 *
 *  \hideinitializer
 */
#define FM_QMI_COMMON(reg)	\
	(offsetof(struct fm_port_mmap_s, qmi) + offsetof(struct fm_qmi_common_mmap_s, FMQM_##reg))

/** QMI offline parsing/host command register offset
 *
 *  \param[in]	reg	The name of a register as it appears in the BG but without the \c "FMQM_"
 *			prefix
 *
 *  Use this macro to get the offset of a QMI offline parsing/host command register. This offset is
 *  suitable for using with the fm_port_in() and fm_port_out() functions.
 *
 *  \par	Example:
 *
 *  Here is a sequence of code that enables a O/H port by writing into the FMQM_PC register:
 *
 *  \code
 *		...
 *	struct fman_port *oh_port;
 *		...
 *	oh_port = fm_port_create(0xfe481000);
 *		...
 *	fm_port_out(oh_port, FM_QMI_OH(PC), FM_QMI_OH_PC_EN);
 *		...
 *	fm_port_destroy(oh_port);
 *		...
 *  \endcode
 *
 *  \hideinitializer
 */
#define FM_QMI_OH(reg)		\
	(offsetof(struct fm_port_mmap_s, qmi) + offsetof(struct fm_qmi_oh_tx_mmap_s, FMQM_##reg))

/** QMI RX register offset
 *
 *  \param[in]	reg	The name of a register as it appears in the BG but without the \c "FMQM_"
 *			prefix
 *
 *  Use this macro to get the offset of a QMI RX register. This offset is suitable for using with
 *  the fm_port_in() and fm_port_out() functions.
 *
 *  \par	Example:
 *
 *  Here is a sequence of code that reads the enqueue status of a RX port by reading from the
 *  FMQM_PS register:
 *
 *  \code
 *		...
 *	struct fman_port *rx_port;
 *		...
 *	rx_port = fm_port_create(0xfe488000);
 *		...
 *	while (fm_port_in(rx_port, FM_QMI_RX(PS)) & FM_QMI_RX_PS_PBSY_ET)
 *		cpu_relax();
 *	/ * Port not busy with enqueues * /
 *		...
 *	fm_port_destroy(rx_port);
 *		...
 *  \endcode
 *
 *  \hideinitializer
 */
#define FM_QMI_RX(reg)		\
	(offsetof(struct fm_port_mmap_s, qmi) + offsetof(struct fm_qmi_rx_mmap_s, FMQM_##reg))

/** QMI TX register offset
 *
 *  \param[in]	reg	The name of a register as it appears in the BG but without the \c "FMQM_"
 *			prefix
 *
 *  Use this macro to get the offset of a QMI TX register. This offset is suitable for using with
 *  the fm_port_in() and fm_port_out() functions.
 *
 *  \par	Example:
 *
 *  Here is a sequence of code that enables a TX port by writing into the FMQM_PC register:
 *
 *  \code
 *		...
 *	struct fman_port *tx_port;
 *		...
 *	tx_port = fm_port_create(0xfe4a8000);
 *		...
 *	fm_port_out(tx_port, FM_QMI_TX(PC), FM_QMI_TX_PC_EN);
 *		...
 *	fm_port_destroy(tx_port);
 *		...
 *  \endcode
 *
 *  \hideinitializer
 */
#define FM_QMI_TX(reg)		\
	(offsetof(struct fm_port_mmap_s, qmi) + offsetof(struct fm_qmi_oh_tx_mmap_s, FMQM_##reg))

/** @} */

/** \addtogroup	Parser
 *  @{
 */

/** \internal
 *  \brief		Parser common/global register layout
 *  \copydetails	fm_bmi_common_mmap_s
 */
struct fm_parser_common_mmap_s {
	uint8_t		FMPR_PMDA[0x800];	/* Parse Memory Direct Access */
	uint32_t	FMPR_SXPAW[0x10];	/* Soft eXamination Parameter Array */
	uint32_t	FMPR_RPCLIM;		/* Parsing Cycle LImit */
	uint32_t	FMPR_RPIMAC;		/* Internal Memory Access Control */
	uint32_t	FMPR_PMEEC;		/* Parse Memory EEC Error Capture */
	uint8_t		reserved0[0x14];
	uint32_t	FMPR_PEVR;		/* Event */
	uint32_t	FMPR_PEVER;		/* Event Enable */
	uint8_t		reserved1[4];
	uint32_t	FMPR_PERR;		/* Error */
	uint32_t	FMPR_PERER;		/* Error Enable */
	uint8_t		reserved2[0x78c];
} __aligned(0x1000) __packed;

#define FM_PARSER_COMMON_RPCLIM_RHPCLIM(n)	(n)		/* maximum Parse Cycle LIMit */

#define FM_PARSER_COMMON_RPIMAC_PSTAT		0x00000100	/* Parser STATus */
#define FM_PARSER_COMMON_RPIMAC_PEN		0x00000001	/* Parser ENable */

#define FM_PARSER_COMMON_PMEEC_CAP		0x80000000	/* CAPtured error indication */
#define FM_PARSER_COMMON_PMEEC_CET		0x40000000	/* Captured Error Type */
#define FM_PARSER_COMMON_PMEEC_SERCNT(n)	((n) << 16)	/* Soft ERror CouNTer */
#define FM_PARSER_COMMON_PMEEC_MEMADDR(n)	(n)		/* ECC error MEMory ADDRess */

#define FM_PARSER_IRQ_SPI(n)			(1 << (16 + (n)))	/* Stopped Port is not Idle */
#define FM_PARSER_IRQ_SCM			0x00004000	/* Single-bit ECC is at Max */

#define FM_PARSER_EIRQ_IA(n)			(1 << (16 + (n)))	/* Illegal Access */
#define FM_PARSER_EIRQ_IAG			FM_PARSER_EIRQ_IA(-1)	/* Illegal Access Global*/
#define FM_PARSER_EIRQ_ECCE			0x00004000		/* ECC multiple-bit Error */

/** \internal
 *  \brief		The parser part of a port
 *  \copydetails	fm_bmi_common_mmap_s
 */
struct fm_port_parser_mmap_s {
	uint8_t		FMPR_PMDA[0x80];	/* Parse Memory Direct Access */
	uint8_t		reserved0[0x378];
	uint32_t	FMPR_PCAC;		/* Configuration Access Control */
	uint32_t	FMPR_PCTPID;		/* Configured TPID */
} __aligned(0x400) __packed;

#define FM_PARSER_PCAC_PSTAT	0x00000100	/* Port STATus */
#define FM_PARSER_PCAC_PSTOP	0x00000001	/* Port STOP */

/** Parser common/global register offset
 *
 *  \param[in]	reg	The name of a register as it appears in the BG but without the \c "FMPR_"
 *			prefix
 *
 *  Use this macro to get the offset of a parser common/global register. This offset is suitable
 *  for using with the fm_in() and fm_out() functions.
 *
 *  \par	Example:
 *
 *  Here is a sequence of code that disables the parser by writing into the FMPR_RPIMAC register:
 *
 *  \code
 *		...
 *	struct fman *fm;
 *		...
 *	fm = fm_create(0xfe400000);
 *		...
 *	fm_out(fm, FM_PARSER_COMMON(RPIMAC), 0);
 *		...
 *	fm_destroy(fm);
 *		...
 *  \endcode
 *
 *  \hideinitializer
 */
#define FM_PARSER_COMMON(reg)	\
	(offsetof(struct fm_mmap_s, parser) + offsetof(struct fm_parser_common_mmap_s, FMPR_##reg))

/** Parser register offset
 *
 *  \param[in]	reg	The name of a register as it appears in the BG but without the \c "FMPR_"
 *			prefix
 *
 *  Use this macro to get the offset of a parser register. This offset is suitable for using with
 *  the fm_port_in() and fm_port_out() functions.
 *
 *  \par	Example:
 *
 *  Here is a sequence of code that stops a parser port by writing and reading the FMPR_PCAC
 *  register:
 *
 *  \code
 *		...
 *	struct fman_port *port;
 *		...
 *	port = fm_port_create(0xfe488000);
 *		...
 *	fm_port_out(port, FM_PARSER(PCAC), FM_PARSER_PCAC_PSTOP);
 *	while (fm_port_in(port, FM_PARSER(PCAC)) & FM_PARSER_PCAP_PSTAT)
 *		cpu_relax();
 *	/ * The port is stopped * /
 *		...
 *	fm_port_destroy(port);
 *		...
 *  \endcode
 *
 *  \hideinitializer
 */
#define FM_PARSER(reg)		\
	(offsetof(struct fm_port_mmap_s, parser) + offsetof(struct fm_port_parser_mmap_s, FMPR_##reg))

/** @} */

/* Port */

/** \internal
 *  \brief		Aggregated port register layout
 *  \copydetails	fm_bmi_common_mmap_s
 */
struct fm_port_mmap_s {
	union fm_port_bmi_mmap_u	bmi;
	union fm_port_qmi_mmap_u	qmi;
	struct fm_port_parser_mmap_s	parser;
	uint8_t				reserved0[0x400];
} __aligned(0x1000) __packed;

/** @} */

/* Policer */

/** \addtogroup	Policer
 *  @{
 */

/** \internal
 *  \brief		Policer register layout
 *  \copydetails	fm_bmi_common_mmap_s
 */
struct fm_policer_mmap_s {
	uint8_t		reserved0[0x8c];
	uint32_t	FMPL_PAR;		/* Profile Action */
	uint32_t	FMPL_PE_MODE;		/* Profile Entry Mode */
	uint8_t		reserved1[0xf6c];
} __aligned(0x1000) __packed;

/** Policer register offset
 *
 *  \param[in]	reg	The name of a register as it appears in the BG but without the \c "FMPL_"
 *			prefix
 *
 *  Use this macro to get the offset of a policer register. This offset is suitable for using with
 *  the fm_in() and fm_out() functions.
 *
 *  \hideinitializer
 */
#define FM_POLICER(reg)	\
	(offsetof(struct fm_mmap_s, policer) + offsetof(struct fm_policer_mmap_s, FMPL_##reg))

/** @} */

/* KeyGen */

/** \addtogroup KeyGen
 *  @{
 */

/** \internal
 *  \brief		KeyGen scheme register layout
 *  \copydetails	fm_bmi_common_mmap_s
 */
struct fm_keygen_scheme_entry_mmap_s {
	uint32_t	FMKG_SE_MODE;	/* Scheme Entry Mode */
	uint32_t	FMKG_SE_EKFC;	/* Scheme Entry Extract Known Fields Command */
	uint32_t	FMKG_SE_EKDV;	/* Scheme Entry Extract Known Default Value */
	uint32_t	FMKG_SE_BMCH;	/* Scheme Entry Bit Mask Command High */
	uint32_t	FMKG_SE_BMCL;	/* Scheme Entry Bit Mask Command Low */
	uint32_t	FMKG_SE_FQB;	/* Scheme Entry Frame Queue Base */
	uint32_t	FMKG_SE_HC;	/* Scheme Entry Hash Command */
	uint32_t	FMKG_SE_PPC;	/* Scheme Entry Policer Profile Command */
	uint32_t	FMKG_SE_GEC[8];	/* Scheme Entry Generic Extract Command */
	uint32_t	FMKG_SE_SPC;	/* Scheme Entry Statistics Packet Counter */
	uint32_t	FMKG_SE_DV[2];	/* Scheme Entry Default Value */
	uint32_t	FMKG_SE_DC;	/* Scheme Entry Debug Criteria */
	uint32_t	FMKG_SE_MV;	/* Scheme Entry Match Vector */
} __aligned(0x100) __packed;

/** \internal
 *  \brief		KeyGen register layout
 *  \copydetails	fm_bmi_common_mmap_s
 *  \todo		Identified a suitable overlaying scheme for port partitions, classification
 *			plans and classification schemes
 */
struct fm_keygen_mmap_s {
	uint32_t	FMKG_GCR;	/* General Configuration */
	uint32_t	FMKG_ER;	/* Event */
	uint32_t	FMKG_EMR;	/* Event Mask */
	uint32_t	FMKG_EER;	/* Error Event */
	uint32_t	FMKG_EEMR;	/* Error Event Mask */
	uint32_t	FMKG_SER;	/* Scheme Event */
	uint32_t	FMKG_SEMR;	/* Scheme Event Mask */
	uint32_t	FMKG_SEER;	/* Scheme Error Event */
	uint32_t	FMKG_SEEMR;	/* Scheme Error Event Mask */
	uint32_t	FMKG_GSR;	/* Global Status */
	uint32_t	FMKG_TPC;	/* Total Packet Counter */
	uint32_t	FMKG_SERC;	/* Soft Error Capture */
	uint8_t		reserved0[0x10];
	uint32_t	FMKG_FDOR;	/* Frame Data Offset */
	uint32_t	FMKG_GDVR[2];	/* Global Default Value */
	uint8_t		reserved1[0xb4];
	uint32_t	FMKG_SE_MODE;	/* Scheme Entry Mode */
	uint32_t	FMKG_SE_EKFC;	/* Scheme Entry Extract Known Fields Command */
	uint32_t	FMKG_SE_EKDV;	/* Scheme Entry Extract Known Default Value */
	uint32_t	FMKG_SE_BMCH;	/* Scheme Entry Bit Mask Command High */
	uint32_t	FMKG_SE_BMCL;	/* Scheme Entry Bit Mask Command Low */
	uint32_t	FMKG_SE_FQB;	/* Scheme Entry Frame Queue Base */
	uint32_t	FMKG_SE_HC;	/* Scheme Entry Hash Command */
	uint32_t	FMKG_SE_PPC;	/* Scheme Entry Policer Profile Command */
	uint32_t	FMKG_SE_GEC[8];	/* Scheme Entry Generic Extract Command */
	uint32_t	FMKG_SE_SPC;	/* Scheme Entry Statistics Packet Counter */
	uint32_t	FMKG_SE_DV[2];	/* Scheme Entry Default Value */
	uint32_t	FMKG_SE_DC;	/* Scheme Entry Debug Criteria */
	uint32_t	FMKG_SE_MV;	/* Scheme Entry Match Vector */
	uint8_t		reserved2[0xa8];
	uint32_t	FMKG_AR;	/* Action */
	uint8_t		reserved3[0xe00];
} __aligned(0x1000) __packed;

#define FM_KEYGEN_GCR_EN		0x80000000		/* ENable */
#define FM_KEYGEN_GCR_DEFNIA(n)		(n)			/* DEFault Next Invoked Action */

#define FM_KEYGEN_EIRQ_DECC		0x80000000		/* Double-bit ECC error */
#define FM_KEYGEN_EIRQ_IEE		0x40000000		/* Initialization Entry Error */
#define FM_KEYGEN_EIRQ_KSO		0x20000000		/* KeyGen Size Overflow */

#define FM_KEYGEN_SIRQ_SC(n)		(0x80000000 >> (n))	/* SCheme interrupt */

#define FM_KEYGEN_SEIRQ_SC(n)		(0x80000000 >> (n))	/* SCheme error interrupt */

#define FM_KEYGEN_GSR_BSY		0x80000000		/* Busy */

#define FM_KEYGEN_TPC_TPCNT(n)		(n)			/* Total Packet CouNTer */

#define FM_KEYGEN_SERC_CAP		0x80000000		/* CAPtured error indication */
#define FM_KEYGEN_SERC_CET		0x40000000		/* Captured Error Type */
#define FM_KEYGEN_SERC_SERCNT(n)	((n) << 16)		/* Soft ERror CouNTer */
#define FM_KEYGEN_SERC_ET		0x00008000		/* Entry Type */
#define FM_KEYGEN_SERC_SOFS(n)		((n) << 8)		/* Scheme OFfSet capture */
#define FM_KEYGEN_SERC_NUM(n)		(n)			/* Scheme/Classification plan NUMber capture */

#define FM_KEYGEN_FDOR_OFFSET(n)	(n)			/* OFFSET to the end of the parsing point */

#define FM_KEYGEN_GDVR_DV(n)		(n)			/* Default Value */

#define FM_KEYGEN_AR_GO			0x80000000		/* Activate the atomic scheme entry access */
#define FM_KEYGEN_AR_RW			0x40000000		/* Read/Write access type */
#define FM_KEYGEN_AR_PORTID(n)		((n) << 24)		/* Port ID */
/* SELect entry */
#define FM_KEYGEN_AR_SEL_MASK		0x00c00000
#define FM_KEYGEN_AR_SEL(n)		((n) << 22 & FM_KEYGEN_AR_SEL_MASK)
/* Scheme/Classification NUMber */
#define FM_KEYGEN_AR_NUM_MASK		0x001f0000
#define FM_KEYGEN_AR_NUM(n)		((n) << 16 & FM_KEYGEN_AR_NUM_MASK)
#define FM_KEYGEN_AR_WSEL(n)		(n)			/* Word SELect */

#define FM_KEYGEN_SE_MODE_SI		0x80000000		/* Scheme Initialization bit */
#define FM_KEYGEN_SE_MODE_IM		0x40000000		/* Index Mode */
#define FM_KEYGEN_SE_MODE_PL		0x01000000		/* PoLicer type */
#define FM_KEYGEN_SE_MODE_NIA(n)	(n)			/* scheme Next Instruction Address */
#define FM_KEYGEN_EKFC_PORTID		0x80000000		/* PORT ID */
#define FM_KEYGEN_EKFC_MACDST		0x40000000		/* MAC DeSTination address */
#define FM_KEYGEN_EKFC_MACSRC		0x20000000		/* MAC SouRCe address */
#define FM_KEYGEN_EKFC_TC1		0x10000000		/* TCI from the first VLAN header */
#define FM_KEYGEN_EKFC_TC2		0x08000000		/* TCI from the second VLAN header */
#define FM_KEYGEN_EKFC_ETYPE		0x04000000		/* Ethernet TYPE */
#define FM_KEYGEN_EKFC_PPPSID		0x02000000		/* PPPoE Session ID */
#define FM_KEYGEN_EKFC_PPPPID		0x01000000		/* PPPoE Protocol ID */
#define FM_KEYGEN_EKFC_MPLS1		0x00800000		/* MPLS from first label */
#define FM_KEYGEN_EKFC_MPLS2		0x00400000		/* MPLS from seconf label */
#define FM_KEYGEN_EKFC_MPLS3		0x00200000		/* MPLS from third label */
#define FM_KEYGEN_EKFC_IPSRC		0x00100000		/* IP SouRCe address */
#define FM_KEYGEN_EKFC_IPDST		0x00080000		/* IP DeSTination Address */
#define FM_KEYGEN_EKFC_PTYPE		0x00040000		/* IP Protocol TYPE */
#define FM_KEYGEN_EKFC_IPTOS		0x00020000		/* IP Type Of Service */
#define FM_KEYGEN_EKFC_TIPSRC		0x00008000		/* Tunneled IP SouRCe address */
#define FM_KEYGEN_EKFC_TIPDST		0x00004000		/* Tunneled IP DeSTination address */
#define FM_KEYGEN_EKFC_TIPTYPE		0x00002000		/* Tunneled IP Protocol TYPE */
#define FM_KEYGEN_EKFC_TIPTOS		0x00001000		/* Tunneled IP Type Of Service */
#define FM_KEYGEN_EKFC_IPSECSPI		0x00000400		/* IPSEC SPI */
#define FM_KEYGEN_EKFC_IPSECNH		0x00000200		/* IPSEC Next Header */
#define FM_KEYGEN_EKFC_L4PSRC		0x00000004		/* L4 Protocol SouRCe port */
#define FM_KEYGEN_EKFC_L4PDST		0x00000002		/* L4 Protocol DeSTination port */
#define FM_KEYGEN_EKFC_TFLG		0x00000001		/* TCP FLaGs */

#define FM_KEYGEN_EKDV_MACDV(n)		((n) << 30)		/*  */
#define FM_KEYGEN_EKDV_TCDV(n)		((n) << 28)		/*  */
#define FM_KEYGEN_EKDV_ETYPEDV(n)	((n) << 26)		/*  */
#define FM_KEYGEN_EKDV_PPPSIDDV(n)	((n) << 24)		/*  */
#define FM_KEYGEN_EKDV_PPPPIDDV(n)	((n) << 22)		/*  */
#define FM_KEYGEN_EKDV_MPLSDV(n)	((n) << 20)		/*  */
#define FM_KEYGEN_EKDV_IPADV(n)		((n) << 18)		/*  */
#define FM_KEYGEN_EKDV_PTYPEDV(n)	((n) << 16)		/*  */
#define FM_KEYGEN_EKDV_IPTOSDV(n)	((n) << 14)		/*  */
#define FM_KEYGEN_EKDV_IP6FLDV(n)	((n) << 12)		/*  */
#define FM_KEYGEN_EKDV_IPSECSPIDV(n)	((n) << 10)		/*  */
#define FM_KEYGEN_EKDV_L4PDV(n)		((n) << 8)		/*  */
#define FM_KEYGEN_EKDV_TFLAGDV(n)	((n) << 6)		/*  */

#define FM_KEYGEN_BMCH_MCS0(n)		((n) << 26)		/* Mask Command Select 0 */
#define FM_KEYGEN_BMCH_MCS1(n)		((n) << 20)		/* Mask Command Select 1 */
#define FM_KEYGEN_BMCH_MO0(n)		((n) << 16)		/* Mask Offset0 */
#define FM_KEYGEN_BMCH_MCS2(n)		((n) << 10)		/* Mask Command Select 2 */
#define FM_KEYGEN_BMCH_MCS3(n)		((n) << 4)		/* Mast Command Select 3 */
#define FM_KEYGEN_BMCH_MO1(n)		(n)			/* Mask Offst 1 */

#define FM_KEYGEN_BMCL_BM0(n)		((n) << 24)		/* Bit Mask 0 */
#define FM_KEYGEN_BMCL_BM1(n)		((n) << 16)		/* Bit Mask 1 */
#define FM_KEYGEN_BMCL_BM2(n)		((n) << 8)		/* Bit Mask 2 */
#define FM_KEYGEN_BMCL_BM3(n)		(n)			/* Bit Mask 3 */

#define FM_KEYGEN_FQB_MO2(n)		((n) << 28)		/* Mask Offset 2 */
#define FM_KEYGEN_FQB_MO3(n)		((n) << 24)		/* Mask Offset 3*/
#define FM_KEYGEN_FQB_FQBASE(n)		(n)			/* Frame Queue Base */

/* Hash SHIFT right */
#define FM_KEYGEN_HC_HSHIFT_MASK	0x3f000000
#define FM_KEYGEN_HC_HSHIFT(n)		((n) << 24 & FM_KEYGEN_HC_HSHIFT)
/* Hash MASK */
#define FM_KEYGEN_HC_HMASK_MASK		0x00ffffff
#define FM_KEYGEN_HC_HMASK(n)		((n) & FM_KEYGEN_HC_HMASK_MASK)

#define FM_KEYGEN_PPC_PPS(n)		((n) << 12)		/* Policer Profile Shift */
#define FM_KEYGEN_PPC_PPMASK(n)		((n) << 16)		/* Policer Profile MASK */
#define FM_KEYGEN_PPC_PPBASE(n)		(n)			/* Policer Profile BASE */

#define FM_KEYGEN_GEC_V			0x80000000		/* Valid */
#define FM_KEYGEN_GEC_DV(n)		((n) << 29)		/* Default Value */
#define FM_KEYGEN_GEC_SIZE(n)		((n) << 24)		/* SIZE */
#define FM_KEYGEN_GEC_MASK(n)		((n) << 16)		/* MASK */
#define FM_KEYGEN_GEC_TYPE		0x00008000		/* command TYPE */
#define FM_KEYGEN_GEC_HT(n)		((n) << 8)		/* Header Type */
#define FM_KEYGEN_GEC_EO(n)		(n)			/* Extract Offset */

#define FM_KEYGEN_SPC_PC(n)		(n)			/* Packet Counter */

#define FM_KEYGEN_DV_DV(n)		(n)			/* Default Value */

#define FM_KEYGEN_DC_DE			0x80000000		/* Debug Enable */

#define FM_KEYGEN_MV_MV(n)		(n)			/* Match Vector */

/** KeyGen register offset
 *
 *  \param[in]	reg	The name of a register as it appears in the BG but without the \c "FMKG_"
 *			prefix
 *
 *  Use this macro to get the offset of a KeyGen register. This offset is suitable for using with
 *  the fm_keygen_in() and fm_keygen_out() functions.
 *
 *  \par	Example:
 *
 *  Here is a sequence of code that enables the KeyGen by writing into the FMKG_GCR register:
 *
 *  \code
 *		...
 *	struct fman_keygen *keygen;
 *		...
 *	keygen = fm_keygen_create(0xfe4c1000);
 *		...
 *	fm_keygen_out(keygen, FM_KEYGEN(GCR), FM_KEYGEN_GCR_EN);
 *		...
 *	fm_keygen_destroy(keygen);
 *		...
 *  \endcode
 *
 *  \hideinitializer
 */
#define FM_KEYGEN(reg)	\
	(offsetof(struct fm_mmap_s, keygen) + offsetof(struct fm_keygen_mmap_s, FMKG_##reg))

/** @} */

/* DMA */

/** \addtogroup	DMA
 *  @{
 */

/** \internal
 *  \brief		DMA register layout
 *  \copydetails	fm_bmi_common_mmap_s
 */
struct fm_dma_mmap_s {
	uint8_t		reserved0[0x60];
	uint32_t	FMDM_PLR[32];	/* PID-LIODN */
	uint8_t		reserved1[0xf20];
} __aligned(0x1000) __packed;

#define FM_DMA(reg) (offsetof(struct fm_mmap_s, dma) + offsetof(struct fm_dma_mmap_s, FMDM_##reg))

/** @} */

/* FPM */

/** \addtogroup	FPM
 *  @{
 */

/** \internal
 *  \brief		FPM register layout
 *  \copydetails	fm_bmi_common_mmap_s
 */
struct fm_fpm_mmap_s {
	uint32_t	FMFP_TNC;	/* TNUM control */
	uint32_t	FMFP_PRC;	/* PoRt_id Control */
	uint8_t		reserved0[0x18];
	uint32_t	FMFP_REV[8];	/* RISC EVent */
	uint32_t	FMFP_REE[8];	/* RISC Event Enable */
	uint8_t		reserved1[0xfa0];
} __aligned(0x1000) __packed;

#define FM_FPM(reg) (offsetof(struct fm_mmap_s, fpm) + offsetof(struct fm_fpm_mmap_s, FMFP_##reg))

/** @} */

/* dTSEC */

/** \addtogroup MAC
 * @{
 */

/** \internal
 *  \brief		dTSEC register layout
 *  \copydetails	fm_bmi_common_mmap_s
 */
struct fm_dtsec_mmap_s {
	uint8_t		reserved0[0x100];
	uint32_t	MACCFG[2];	/* MAC ConFiGuration */
	uint8_t		reserved1[0xef8];
} __aligned(0x1000) __packed;

/** dTSEC register offset
 *
 *  \param[in]	reg	The name of a register as it appears in the BG
 *
 *  Use this macro to get the offset of a dTSEC register. This offset is suitable for using with
 *  the fm_mac_in() and fm_mac_out() functions.
 *
 *  \par	Example:
 *
 *  Here is a sequence of code that initializes a dTSEC to append a CRC to all frames by writing
 *  into the MACCFG2 register:
 *
 *  \code
 *		...
 *	struct fman_mac *mac;
 *		...
 *	mac = fm_mac_create(0xfe4e0000);
 *		...
 *	fm_mac_out(mac, FM_DTSEC(MACCFG[1]), FM_DTSEC_MACCFG2_CRC_EN);
 *		...
 *	fm_mac_destroy(mac);
 *		...
 *  \endcode
 *
 *  \hideinitializer
 */
#define FM_DTSEC(reg) offsetof(struct fm_dtsec_mmap_s, reg)

/** @} */

/** \addtogroup MDIO
 *  @{
 */

/** \internal
 *  \brief		MDIO register layout
 *  \copydetails	fm_bmi_common_mmap_s
 */
struct fm_mdio_mmap_s {
	uint8_t	reserved[0x1000];
}  __aligned(0x1000) __packed;

/** @} */

/** \internal
 *  \brief		Aggregated dTSEC and MDIO register layout
 *  \copydetails	fm_bmi_common_mmap_s
 */
struct fm_dtsec_mdio_mmap_s {
	struct fm_dtsec_mmap_s	dtsec;
	struct fm_mdio_mmap_s	mdio;
}  __aligned(0x2000) __packed;

/* FMan */

/** \internal
 *  \brief		Aggregated FMan register layout
 *  \copydetails	fm_bmi_common_mmap_s
 */
struct fm_mmap_s {
	uint8_t				MURAM[0x80000];
	struct fm_port_mmap_s		ports[0x40];
	struct fm_policer_mmap_s	policer;
	struct fm_keygen_mmap_s		keygen;
	struct fm_dma_mmap_s		dma;
	struct fm_fpm_mmap_s		fpm;
	uint8_t				reserved1[0x3000];
	struct fm_parser_common_mmap_s	parser;
	uint8_t				reserved2[0x18000];
	struct fm_dtsec_mdio_mmap_s	dtsec_mdio[4];
	uint8_t				reserved3[0x18000];
} __aligned(0x100000) __packed;

/** Parse results memory layout */
struct fman_parse_results {
	uint8_t		lpid;
	uint8_t		shimr;
	uint16_t	l2r;
	uint16_t	l3r;
	uint8_t		l4r;
	uint8_t		cplan;
	uint16_t	nxthdr;
	uint16_t	cksum;
	uint32_t	lcv;
	uint8_t		shim_off[3];
	uint8_t		eth_off;
	uint8_t		llc_snap_off;
	uint8_t		vlan_off;
	uint8_t		etype_off;
	uint8_t		pppoe_off;
	uint8_t		mpls_off;
	uint8_t		ip_off;
	uint8_t		gre_off;
	uint8_t		l4_off;
	uint8_t		nxthdr_off;
} __packed;

/* Low-level abstractions */

/** \struct	fman
 *  \brief	Handle to a FMan object/device
 *  \details	Used with the fm_in() and fm_out() register access functions
 */
struct fman;

/** Get a handle to a FMan device/object
 *
 *  \note	For an usage example please see FM_PARSER_COMMON()
 */
static inline struct fman __iomem *fm_create(void *regs)
{
	return (struct fman *)regs;
}

/** Read a 32-bit register from the FMan CCSR memory space
 *
 *  \note	Use this function as a last resort. It's preferable to use more specific functions
 *		such as fm_port_in(), etc.
 */
static inline uint32_t fm_in(const struct fman *fm,	/**< FMan handle */
			     off_t reg_off)		/**< Register offset */
{
	return in_be32((void *)((uintptr_t)fm + reg_off));
}

/** Write a 32-bit register into the FMan CCSR memory space
 *
 *  \note		Use this function as a last resort. It's preferable to use more specific
 *			functions such as fm_port_out(), etc.
 *
 *  \copydetails	fm_create
 */
static inline void fm_out(struct fman *fm,	/**< FMan handle */
			  off_t reg_off,	/**< Register offset */
			  uint32_t val)		/**< Register value */
{
	out_be32((void *)((uintptr_t)fm + reg_off), val);
}

/** Release a handle to a FMan device
 *
 *  \copydetails	fm_create
 */
static inline void fm_destroy(struct fman *regs __maybe_unused)
{
}

/** \addtogroup	Port
 *  @{
 */

/** Hardware port IDs */
enum fm_port {
	fm_port_common,
	fm_port_oh0,
	fm_port_oh1,
	fm_port_oh2,
	fm_port_oh3,
	fm_port_oh4,
	fm_port_oh5,
	fm_port_oh6,
	fm_port_rx0,
	fm_port_rx1,
	fm_port_rx2,
	fm_port_rx3,
	fm_port_rx4 = 0x10,
	fm_port_tx0 = 0x28,
	fm_port_tx1,
	fm_port_tx2,
	fm_port_tx3,
	fm_port_tx4 = 0x30
} __packed;

/** \struct	fman_port
 *  \brief	Port handle
 *  \details	Used with the fm_port_in() and fm_port_out() register access functions
 */
struct fman_port;

/** Get a handle to a port
 *
 *  \note	For an usage example please see one of FM_BMI_COMMON(), FM_BMI_OH(), FM_BMI_RX(),
 *		FM_BMI_TX(), FM_QMI_COMMON(), FM_QMI_OH(), FM_QMI_RX(), FM_QMI_TX() or/and
 *		FM_PARSER().
 */
static inline struct fman_port __iomem *fm_port_create(void *regs)
{
	return (struct fman_port *)regs;
}

/** Read a 32-bit register from a port CCSR memory space
 *
 *  \note	For an usage example please see one of FM_BMI_RX(), FM_QMI_RX() or/and FM_PARSER().
 */
static inline uint32_t fm_port_in(const struct fman_port *fm_port,	/**< Port handle */
				  off_t reg_off)			/**< Register offset */
{
	return in_be32((void *)((uintptr_t)fm_port + reg_off));
}

/** Write a 32-bit register to a port CCSR memory space
 *
 *  \note	For an usage example please see one of FM_BMI_COMMON(), FM_BMI_OH(), FM_BMI_TX(),
 *		FM_QMI_COMMON(), FM_QMI_OH(), FM_QMI_TX() or/and FM_PARSER().
 */
static inline void fm_port_out(struct fman_port *fm_port,	/**< Port handle */
			       off_t reg_off,			/**< Register offset */
			       uint32_t val)			/**< Register value */
{
	out_be32((void *)((uintptr_t)fm_port + reg_off), val);
}

/** Release a handle to a port
 *
 *  \copydetails	fm_port_create
 */
static inline void fm_port_destroy(struct fman_port *regs __maybe_unused)
{
}

/** @} */

/** \addtogroup	KeyGen
 *  @{
 */

/** KeyGen handle
 */
struct fman_keygen {
	struct fm_keygen_mmap_s	*mmap;
	bool			 busy;
};

/** Get a handle to KeyGen
 *
 *  \note	For a usage example please see FM_KEYGEN() or fm_keygen_entry_start().
 *
 *  \todo	No memory allocation
 */
static inline struct fman_keygen __iomem *fm_keygen_create(void *regs)
{
	struct fman_keygen	*fm_keygen;

	fm_keygen = kzalloc(sizeof(*fm_keygen), GFP_KERNEL);
	if (likely(fm_keygen != NULL))
		fm_keygen->mmap = (typeof(fm_keygen->mmap))regs;

	return fm_keygen;
}

static inline struct fman *keygen_to_fman(struct fman_keygen *fm_keygen)
{
	return (struct fman *)container_of(fm_keygen->mmap, struct fm_mmap_s, keygen);
}

/** Read a 32-bit register from the KeyGen CCSR memory space */
static inline uint32_t fm_keygen_in(struct fman_keygen *fm_keygen, off_t reg_off)
{
	return fm_in(keygen_to_fman(fm_keygen), reg_off);
}

/** Write a 32-bit register to the KeyGen CCSR memory space
 *
 *  \copydetails	fm_keygen_create
 */
static inline void fm_keygen_out(struct fman_keygen *fm_keygen, off_t reg_off, uint32_t val)
{
	fm_out(keygen_to_fman(fm_keygen), reg_off, val);
}

/** Release a handle to KeyGen
 *
 *  \copydetails	fm_keygen_create
 *
 *  \todo		No memory allocation
 */
static inline void fm_keygen_destroy(struct fman_keygen *fm_keygen)
{
	kfree(fm_keygen);
}

/** @} */

/** \addtogroup	MAC
 *  @{
 */

/** \struct	fman_mac
 *  \brief	MAC handle
 *  \details	Used with the fm_mac_in() and fm_mac_out() register access functions
 *  \note	Currently supports only the dTSEC(s).
 */
struct fman_mac;

/** Get a handle to a MAC
 *
 *  \note	For an usage example see FM_DTSEC()
 */
static inline struct fman_mac __iomem *fm_mac_create(void *regs)
{
	return (struct fman_mac *)regs;
}

/** Read a 32-bit register from a MAC CCSR memory space */
static inline uint32_t fm_mac_in(const struct fman_mac *fm_mac,	/**< MAC handle */
				 off_t reg_off)			/**< Register offset */
{
	return in_be32((void *)((uintptr_t)fm_mac + reg_off));
}

/** Write a 32-bit register to a MAC CCSR memory space
 *
 *  \copydetails	fm_mac_create
 */
static inline void fm_mac_out(struct fman_mac *fm_mac,	/**< MAC handle */
			      off_t reg_off,		/**< Register offset */
			      uint32_t val)		/**< Register value */
{
	out_be32((void *)((uintptr_t)fm_mac + reg_off), val);
}

/** Release a handle to a MAC
 *
 *  \copydetails	fm_mac_create
 */
static inline void fm_mac_destroy(struct fman_mac *regs __maybe_unused)
{
}

/** @} */

#define FM_FD_STAT_DME		0x01000000	/* DMA Error - QMI */
#define FM_FD_STAT_FHE		0x00080000	/* Physical Error - BMI */
#define FM_FD_STAT_FSE		0x00040000	/* Frame Size Error - BMI */
#define FM_FD_STAT_DIS		0x00020000	/* Discarded frame - BMI */
#define FM_FD_STAT_EOF		0x00008000	/* Extract Out of Frame - KEYGEN */
#define FM_FD_STAT_NSS		0x00004000	/* No Scheme Selected - KEYGEN */
#define FM_FD_STAT_FCL		0x00000c00	/* Frame Color - Policer */
#define FM_FD_STAT_IPP		0x00000200	/* Illegal Policer Profile - Policer */
#define FM_FD_STAT_PTE		0x00000080	/* Parser Timeout Exceeded - Parser */
#define FM_FD_STAT_ISP		0x00000040	/* Invalid Soft Parser Instruction - Parser */
#define FM_FD_STAT_PHE		0x00000020	/* Parsing Header Error - Parser */
#define FM_FD_STAT_ERRORS	(FM_FD_STAT_DME | FM_FD_STAT_FHE | FM_FD_STAT_FSE |	\
				 FM_FD_STAT_DIS | FM_FD_STAT_EOF | FM_FD_STAT_NSS |	\
				 FM_FD_STAT_IPP | FM_FD_STAT_PTE | FM_FD_STAT_ISP |	\
				 FM_FD_STAT_PHE)

#define FM_FD_CMD_FCO	0x80000000	/* Frame queue Context Override */
#define FM_FD_CMD_RPD	0x40000000	/* Read Prepended Data */
#define FM_FD_CMD_UDP	0x20000000	/* Update Prepended Data */
#define FM_FD_CMD_BMF	0x10000000	/* Buffer Must not be Freed */
#define FM_FD_CMD_DTC	0x08000000	/* Do TCP Checksum */
#define FM_FD_CMD_DME	0x01000000	/* DMA Error */
#define FM_FD_CMD_CFQ	0x00ffffff	/* Confirmation Frame Queue */

/* High-level abstractions */

/* KeyGen */

/** \addtogroup	KeyGen
 *  @{
 */

/** KeyGen entry types
 *
 *  The Keygen uses the same registers area to overlay three different functionalities. Access to
 *  register sets should be guarded with calls to fm_keygen_entry_start() and fm_keygen_commit().
 */
enum fman_keygen_entry_type {
	fman_keygen_scheme,	/**< Classification scheme */
	fman_keygen_plan,	/**< Classification plan */
	fman_keygen_port	/**< Port partition */
} __packed;

/** KeyGen entry handle */
struct fman_keygen_entry {
	struct fman_keygen		*fm_keygen;
	uint32_t			 ar;
};

/** Start reading/writing a new KeyGen entry
 *
 *  Once a KeyGen entry handle is aquired, use the fm_keygen_in() and fm_keygen_out() to access the
 *  entry registers and activate the entry using fm_keygen_commit(). In order to abandon an entry
 *  before activating it and start a new kind of entry use again fm_keygen_entry_start().
 *
 *  \par	Example:
 *
 *  Here is a partial sequence of code that ilustrates how to program/write a classification scheme:
 *
 *  \code
 *		...
 *	struct fman_keygen		*keygen;
 *	struct fman_keygen_entry	*keygen_entry;
 *		...
 *	keygen = fm_keygen_create(0xfe4c1000);
 *		...
 *	keygen_entry = fm_keygen_entry_start(keygen, false, fman_keygen_scheme, 0);
 *		...
 *	/ * Hash based on the IP source and destination addresses and TOS bits * /
 *	fm_keygen_out(keygen, FM_KEYGEN(SE_EKFC), FM_KEYGEN_EKFC_IPSRC |
 *						  FM_KEYGEN_EKFC_IPDST |
 *						  FM_KEYGEN_EKFC_IPTOS);
 *		...
 *	fm_keygen_commit(keygen_entry);
 *		...
 *	fm_keygen_destroy(keygen);
 *		...
 *  \endcode
 *
 *  \todo	No memory allocation
 */
static inline struct fman_keygen_entry *
fm_keygen_entry_start(struct fman_keygen		*fm_keygen,	/**< KeyGen handle */
		      bool				 read,		/**< Read or write  */
		      enum fman_keygen_entry_type	 type,		/**< Entry type */
		      uint8_t				 index)		/**< Entry index */
{
	struct fman_keygen_entry	*fm_keygen_entry;

	if (fm_keygen->busy == true)
		return NULL;

	while(fm_keygen_in(fm_keygen, FM_KEYGEN(AR)) & FM_KEYGEN_AR_GO)
		cpu_relax();

	fm_keygen_entry = kzalloc(sizeof(*fm_keygen_entry), GFP_KERNEL);
	if (likely(fm_keygen_entry != NULL)) {
		fm_keygen_entry->fm_keygen = fm_keygen;

		fm_keygen_entry->ar = FM_KEYGEN_AR_GO		|
				      FM_KEYGEN_AR_SEL(type)	|
				      FM_KEYGEN_AR_NUM(index);
		if (read == true) {
			fm_keygen_entry->ar |= FM_KEYGEN_AR_RW;
			fm_keygen_out(fm_keygen, FM_KEYGEN(AR), fm_keygen_entry->ar);
			while (fm_keygen_in(fm_keygen, FM_KEYGEN(AR)) & FM_KEYGEN_AR_GO)
				cpu_relax();
		}

		fm_keygen->busy = true;
	}

	return fm_keygen_entry;
}

/** Activate the current KeyGen entry
 *
 *  \note	For an usage example please see fm_keygen_entry_start()
 *
 *  \todo	No memory allocation
 */
static inline void fm_keygen_commit(struct fman_keygen_entry *fm_keygen_entry)
{
	BUG_ON(fm_keygen_entry->fm_keygen->busy == false);

	if ((fm_keygen_entry->ar & FM_KEYGEN_AR_RW) == 0)	/* Write access type */
		fm_keygen_out(fm_keygen_entry->fm_keygen, FM_KEYGEN(AR), fm_keygen_entry->ar);

	fm_keygen_entry->fm_keygen->busy = false;
	kfree(fm_keygen_entry);
}

/** @} */

enum fm_isr_reg {
	fm_isr_status,
	fm_isr_enable
} __packed;

/** FMan engine code
 *
 *  To be used by/with fm_nia() for building the NIA codes for the \ref BMI, \ref KeyGen, etc.
 */
enum fm_engine {
	fm_risc,		/**< RISC */
	fm_parser = 0x11,	/**< \ref Parser \hideinitializer */
	fm_keygen,		/**< \ref KeyGen*/
	fm_policer,		/**< \ref Policer */
	fm_bmi,			/**< \ref BMI */
	fm_qmi_enqueue,		/**< \ref QMI */
	fm_qmi_dequeue		/**< \ref QMI */
} __packed;

#define FM_NIA_OPR_SHIFT	23
#define FM_NIA_OPR_MASK		0x00800000
#define FM_NIA_ENG_SHIFT	18
#define FM_NIA_ENG_MASK		0x007c0000
#define FM_NIA_AC_SHIFT		0
#define FM_NIA_AC_MASK		0x0003ffff

/** Build NIA codes */
static inline uint32_t fm_nia(bool opr,			/**< Use order preservation */
			      enum fm_engine eng,	/**< Next engine */
			      uint32_t ac)		/**< Action code */
{
	return (opr	<< FM_NIA_OPR_SHIFT	& FM_NIA_OPR_MASK)	|
	       (eng	<< FM_NIA_ENG_SHIFT	& FM_NIA_ENG_MASK)	|
	       (ac	<< FM_NIA_AC_SHIFT	& FM_NIA_AC_MASK);
}

#endif	/* __FMAN_CONF_H */
