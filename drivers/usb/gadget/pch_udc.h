/*
 * Copyright (C) 2010 OKI SEMICONDUCTOR Co., LTD.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307, USA.
 */

#ifndef _PCH_UDC_H_
#define _PCH_UDC_H_

/* Address offset of Registers */
#define UDC_EPIN_REGS_ADDR		0x000
#define UDC_EPOUT_REGS_ADDR		0x200
#define UDC_EP_REG_OFS			0x20	/* Offset to next EP */
#define UDC_DEVCFG_ADDR			0x400
#define PCH_UDC_CSR_BUSY_ADDR		0x4f0
#define PCH_UDC_SRST_ADDR		0x4fc
#define UDC_CSR_ADDR			0x500

/* Bit position in UDC CSR Busy status Register */
#define PCH_UDC_CSR_BUSY		1
/* Bit position in UDC Soft reset Register */
#define PCH_UDC_PSRST			1
#define PCH_UDC_SRST			0

/* DMA */
#define DMA_DIR_RX		1	/* DMA for data receive */
#define DMA_DIR_TX		2	/* DMA for data transmit */
#define UDC_DMA_MAXPACKET	65536	/* maximum packet size for DMA */

#define EP_IS_IN(ep)	(((u32)(ep)) < (pch_udc_base + UDC_EPOUT_REGS_ADDR))
#define EP_NUM(ep)	((((u32)(ep) - (pch_udc_base +\
				UDC_EPIN_REGS_ADDR)) / UDC_EP_REG_OFS) & 0xf)

#define PCH_UDC_EP_NUM		32	/* Total number of EPs (16 IN,16 OUT) */
#define PCH_UDC_USED_EP_NUM	4	/* EP number of EP's really used */

/* Control ep index */
#define UDC_EP0IN_IDX		0
#define UDC_EP0OUT_IDX		1
/* number of EP */
#define PCH_UDC_EP0		0
#define PCH_UDC_EP1		1
#define PCH_UDC_EP2		2
#define PCH_UDC_EP3		3

/* Rx fifo address and size = 2k */
#define UDC_RXFIFO_ADDR		0x800	/* Address offset of Rx FIFO */
#define UDC_RXFIFO_SIZE		0x800	/* Rx FIFO size */

/* Tx fifo address and size = 4k */
#define UDC_TXFIFO_ADDR		0x1000	/* Address offset of Tx FIFO */
#define UDC_TXFIFO_SIZE		0x1000	/* Tx FIFO size */


/**
 * pch_udc_csrs - Structure to Endpoint configuration registers
 */
struct pch_udc_csrs {
	u32 ne[PCH_UDC_USED_EP_NUM * 2];
/* Starting bit position */
#define UDC_CSR_NE_NUM_OFS			0
#define UDC_CSR_NE_DIR_OFS			4
#define UDC_CSR_NE_TYPE_OFS			5
#define UDC_CSR_NE_CFG_OFS			7
#define UDC_CSR_NE_INTF_OFS			11
#define UDC_CSR_NE_ALT_OFS			15
#define UDC_CSR_NE_MAX_PKT_OFS			19
/* Mask patern */
#define UDC_CSR_NE_NUM_MASK			0x0000000f
#define UDC_CSR_NE_DIR_MASK			0x00000010
#define UDC_CSR_NE_TYPE_MASK			0x00000060
#define UDC_CSR_NE_CFG_MASK			0x00000780
#define UDC_CSR_NE_INTF_MASK			0x00007800
#define UDC_CSR_NE_ALT_MASK			0x00078000
#define UDC_CSR_NE_MAX_PKT_MASK			0x3ff80000
};

/**
 * pch_udc_regs - Structure holding values of configuration registers
 *
 * @devcfg	Device configuration register
 * @devctl	Device control register
 * @devsts	Device status register
 * @devirqsts	Device irq status register
 * @devirqmsk	Device irq mask register
 * @epirqsts	Endpoint irq status register
 * @epirqmsk	Endpoint irq mask register
 * @devlpm	LPM control / status registe
 */
struct pch_udc_regs {
	u32 devcfg;
/* Bit position */
#define UDC_DEVCFG_SET_DESC	18
#define UDC_DEVCFG_CSR_PRG	17
#define UDC_DEVCFG_HALT_STATUS	16
#define UDC_DEVCFG_STATUS1	8
#define UDC_DEVCFG_STATUS	7
#define UDC_DEVCFG_DIR		6
#define UDC_DEVCFG_PI		5
#define UDC_DEVCFG_SS		4
#define UDC_DEVCFG_SP		3
#define UDC_DEVCFG_RWKP		2
#define UDC_DEVCFG_SPD_OFS	0
/* Mask patern */
#define UDC_DEVCFG_SPD_MASK	0x00000003
/* SPD Valee */
#define UDC_DEVCFG_SPD_HS	0x0
#define UDC_DEVCFG_SPD_FS	0x1
#define UDC_DEVCFG_SPD_LS	0x2

	u32 devctl;
/* Bit position */
#define UDC_DEVCTL_THLEN_OFS	24
#define UDC_DEVCTL_BRLEN_OFS	16
#define UDC_DEVCTL_CSR_DONE	13
#define UDC_DEVCTL_DEVNAK	12
#define UDC_DEVCTL_SD		10
#define UDC_DEVCTL_MODE		9
#define UDC_DEVCTL_BREN		8
#define UDC_DEVCTL_THE		7
#define UDC_DEVCTL_BF		6
#define UDC_DEVCTL_BE		5
#define UDC_DEVCTL_DU		4
#define UDC_DEVCTL_TDE		3
#define UDC_DEVCTL_RDE		2
#define UDC_DEVCTL_RES		0
/* Mask patern */
#define UDC_DEVCTL_THLEN_MASK	0xff000000
#define UDC_DEVCTL_BRLEN_MASK	0x00ff0000
/* Length Valee */
#define PCH_UDC_BRLEN		0x0F	/* Burst length */
#define PCH_UDC_THLEN		0x1F	/* Threshold length */

	u32 devsts;
/* Bit position */
#define UDC_DEVSTS_TS_OFS		18
#define UDC_DEVSTS_RWKPST		17
#define UDC_DEVSTS_PHY_ERROR		16
#define UDC_DEVSTS_RXFIFO_EMPTY		15
#define UDC_DEVSTS_ENUM_SPEED_OFS	13
#define UDC_DEVSTS_SUSP			12
#define UDC_DEVSTS_ALT_OFS		8
#define UDC_DEVSTS_INTF_OFS		4
#define UDC_DEVSTS_CFG_OFS		0
/* Mask patern */
#define UDC_DEVSTS_TS_MASK		0xfffc0000
#define UDC_DEVSTS_ENUM_SPEED_MASK	0x00006000
#define UDC_DEVSTS_ALT_MASK		0x00000f00
#define UDC_DEVSTS_INTF_MASK		0x000000f0
#define UDC_DEVSTS_CFG_MASK		0x0000000f
/* value for maximum speed for SPEED field */
#define UDC_DEVSTS_ENUM_SPEED_FULL	1
#define UDC_DEVSTS_ENUM_SPEED_HIGH	0
#define UDC_DEVSTS_ENUM_SPEED_LOW	2
#define UDC_DEVSTS_ENUM_SPEED_FULLX	3

	u32 devirqsts;
/* Bit position */
#define UDC_DEVINT_RWKP			7
#define UDC_DEVINT_ENUM			6
#define UDC_DEVINT_SOF			5
#define UDC_DEVINT_US			4
#define UDC_DEVINT_UR			3
#define UDC_DEVINT_ES			2
#define UDC_DEVINT_SI			1
#define UDC_DEVINT_SC			0

	u32 devirqmsk;
/* Mask patern */
#define UDC_DEVINT_MSK			0x7f

	u32 epirqsts;
/* Bit position */
#define UDC_EPINT_IN_OFS		0
#define UDC_EPINT_OUT_OFS		16
#define UDC_EPINT_IN_EP0		0
#define UDC_EPINT_IN_EP1		1
#define UDC_EPINT_IN_EP2		2
#define UDC_EPINT_IN_EP3		3
#define UDC_EPINT_OUT_EP0		16
#define UDC_EPINT_OUT_EP1		17
#define UDC_EPINT_OUT_EP2		18
#define UDC_EPINT_OUT_EP3		19
/* Mask patern */
#define UDC_EPINT_OUT_MASK		0xffff0000
#define UDC_EPINT_IN_MASK		0x0000ffff
#define UDC_EPINT_EP0_ENABLE_MSK	0x000e000e

	u32 epirqmsk;
/* Bit position */
#define UDC_EPINT_OUT_MSK_OFS		16
#define UDC_EPINT_IN_MSK_OFS		0
/* Mask patern */
#define UDC_EPINT_MSK_DISABLE_ALL	0xffffffff

	u32 devlpm;
};

/**
 * pch_udc_ep_regs -  Structure holding values of ep configuration registers
 * @epctl		Endpoint control register
 * @epsts		Endpoint status register
 * @bufin_framenum	buffer size in / frame number out
 * @bufout_maxpkt	buffer size out / maxpkt in
 * @subptr		setup buffer pointer
 * @desptr		Data descriptor pointer
 * @confirm		Write/Read confirmation  for slave mode only
 */
struct pch_udc_ep_regs {
	u32 epctl;
/* Bit position */
#define UDC_EPCTL_MRXFLUSH		12
#define UDC_EPCTL_RRDY			9
#define UDC_EPCTL_CNAK			8
#define UDC_EPCTL_SNAK			7
#define UDC_EPCTL_NAK			6
#define UDC_EPCTL_ET_OFS		4
#define UDC_EPCTL_P			3
#define UDC_EPCTL_SN			2
#define UDC_EPCTL_F			1
#define UDC_EPCTL_S			0
/* Mask patern */
#define UDC_EPCTL_ET_MASK		0x00000030
/* Value for ET field */
#define UDC_EPCTL_ET_CONTROL		0
#define UDC_EPCTL_ET_ISO		1
#define UDC_EPCTL_ET_BULK		2
#define UDC_EPCTL_ET_INTERRUPT		3

	u32 epsts;
/* Bit position */
#define UDC_EPSTS_XFERDONE		27
#define UDC_EPSTS_RSS			26
#define UDC_EPSTS_RCS			25
#define UDC_EPSTS_TXEMPTY		24
#define UDC_EPSTS_ISOINDONE		23
#define UDC_EPSTS_RX_PKT_SIZE_OFS	11
#define UDC_EPSTS_TDC			10
#define UDC_EPSTS_HE			9
#define UDC_EPSTS_MRXFIFO_EMP		8
#define UDC_EPSTS_BNA			7
#define UDC_EPSTS_IN			6
#define UDC_EPSTS_OUT_OFS		4
#define UDC_EPSTS_OUT_SETUP		2
#define UDC_EPSTS_OUT_DATA		1
/* Mask patern */
#define UDC_EPSTS_RX_PKT_SIZE_MASK	0x007ff800
#define UDC_EPSTS_OUT_MASK		0x00000030
/* Value for OUT field */
#define UDC_EPSTS_OUT_DATA_CLEAR	0x10
#define UDC_EPSTS_OUT_SETUP_CLEAR	0x20
#define UDC_EPSTS_OUT_CLEAR		0x30

	u32 bufin_framenum;
/* Bit position */
#define UDC_EPIN_BUFF_SIZE_OFS		0
#define UDC_EPOUT_FRAME_NUMBER_OFS	0
/* Mask patern */
#define UDC_EPIN_BUFF_SIZE_MASK		0x0000ffff
#define UDC_EPOUT_FRAME_NUMBER_MASK	0x0000ffff
/* EPIN0 Buffer Size */
#define UDC_EP0IN_BUFF_SIZE		64
#define UDC_FS_EPIN0_BUFF_SIZE		32
#define UDC_EPIN_BUFF_SIZE_MULT		2
#define UDC_EPIN_BUFF_SIZE		512
#define UDC_EPIN_SMALLINT_BUFF_SIZE	32
#define UDC_FS_EPIN_BUFF_SIZE		32

	u32 bufout_maxpkt;
/* Starting bit position */
#define UDC_EPOUT_BUFF_SIZE_OFS		16
#define UDC_EP_MAX_PKT_SIZE_OFS		0
/* Mask patern */
#define UDC_EPOUT_BUFF_SIZE_MASK	0xffff0000
#define UDC_EP_MAX_PKT_SIZE_MASK	0x0000ffff
/* Value of EP OUT Buffer Size */
#define UDC_EP0OUT_BUFF_SIZE		64
#define UDC_EPOUT_BUFF_SIZE		512
#define UDC_FS_EPOUT_BUFF_SIZE		32
/* Value of EP0 maximum packet size */
#define UDC_EP0IN_MAX_PKT_SIZE		64
#define UDC_EP0OUT_MAX_PKT_SIZE		64
#define UDC_BULK_MAX_PKT_SIZE		512
#define UDC_FS_EP0IN_MAX_PKT_SIZE	64
#define UDC_FS_EP0OUT_MAX_PKT_SIZE	64

	u32 subptr;
	u32 desptr;
	u32 confirm;
};

#define DMA_ADDR_INVALID	(~(dma_addr_t)0)

/**
 * pch_udc_data_dma_desc - Structure to hold DMA descriptor information
 *				for data
 * @status		Status quadlet
 * @reserved		Reserved
 * @dataptr		Buffer descriptor
 * @next		Next descriptor
*/
struct pch_udc_data_dma_desc {
	u32 status;
	u32 reserved;
	u32 dataptr;
	u32 next;
};

/**
 * pch_udc_stp_dma_desc - Structure to hold DMA descriptor information
 *				for control data
 * @status	Status
 * @reserved	Reserved
 * @data12	First setup word
 * @data34	Second setup word

*/
struct pch_udc_stp_dma_desc {
	u32 status;
	u32 reserved;
	u32 data12;
	u32 data34;
};

/* DMA status definitions */
/* Buffer status */
#define PCH_UDC_BUFF_STS	0xC0000000
#define PCH_UDC_BS_HST_RDY	0x00000000
#define PCH_UDC_BS_DMA_BSY	0x40000000
#define PCH_UDC_BS_DMA_DONE	0x80000000
#define PCH_UDC_BS_HST_BSY	0xC0000000
/*  Rx/Tx Status */
#define PCH_UDC_RXTX_STS	0x30000000
#define PCH_UDC_RTS_SUCC	0x00000000
#define PCH_UDC_RTS_DESERR	0x10000000
#define PCH_UDC_RTS_BUFERR	0x30000000
/* Last Descriptor Indication */
#define PCH_UDC_DMA_LAST	0x08000000
/* Number of Rx/Tx Bytes Mask */
#define PCH_UDC_RXTX_BYTES	0x0000ffff

/**
 * pch_udc_cfg_data - Structure to hold current configuration
 *			and interface information
 * @cur_cfg	current configuration in use
 * @cur_intf	current interface in use
 * @cur_alt	current alt interface in use
 */
struct pch_udc_cfg_data {
	u16 cur_cfg;
	u16 cur_intf;
	u16 cur_alt;
};


/**
 * pch_udc_request - Structure holding a PCH USB device request
 * @req			embedded ep request
 * @td_data_phys	phys. address
 * @td_data		first dma desc. of chain
 * @td_data_last	last dma desc. of chain
 * @queue		associated queue
 * @dma_going		DMA in progress for request
 * @dma_mapped		DMA memory mapped for request
 * @dma_done		DMA completed for request
 * @chain_len		chain length
 */
struct pch_udc_request /* request packet */
{
	struct usb_request		req;
	dma_addr_t			td_data_phys;
	struct pch_udc_data_dma_desc	*td_data;
	struct pch_udc_data_dma_desc	*td_data_last;
	struct list_head		queue;
	unsigned			dma_going:1,
					dma_mapped:1,
					dma_done:1;
	unsigned			chain_len;
};

/**
 * pch_udc_ep - Structure holding a PCH USB device Endpoint information
 * @ep			embedded ep request
 * @td_stp_phys		for setup request
 * @td_data_phys	for data request
 * @td_stp		for setup request
 * @td_data		for data request
 * @dev			reference to device struct
 * @regs
 * @dma			dma enabled or not
 * @desc		for this ep
 * @queue		queue for requests
 * @num			endpoint number
 * @in			endpoint is IN
 * @halted		endpoint halted?
 * @epsts		Endpoint status
 */
struct pch_udc_ep {
	struct usb_ep			ep;
	dma_addr_t			td_stp_phys;
	dma_addr_t			td_data_phys;
	struct pch_udc_stp_dma_desc	*td_stp;
	struct pch_udc_data_dma_desc	*td_data;
	struct pch_udc_dev		*dev;
	struct pch_udc_ep_regs __iomem	*regs;
	u32 __iomem			*dma;
	const struct usb_endpoint_descriptor	*desc;
	struct list_head		queue;
	unsigned			num:5;
	unsigned			in:1;
	unsigned			halted;
	unsigned long			epsts;
};


/**
 * pch_udc_dev - Structure holding complete information of the PCH USB device
 *
 * @gadget		gadget driver data
 * @driver;		reference to gadget driver bound
 * @pdev;		reference to the PCI device
 * @ep[PCH_UDC_EP_NUM];	array of endpoints
 * @lock;		protects all state
 * @active:1,		enabled the PCI device
 * @stall:1,		stall requested
 * @prot_stall:1,	protcol stall requested
 * @irq_registered:1,	irq registered with system
 * @mem_region:1,	device memory mapped
 * @registered:1,	driver regsitered with system
 * @suspended:1,	driver in suspended state
 * @connected:1,	gadget driver associated
 * @set_cfg_not_acked:1,	pending acknowledgement 4 setup
 * @waiting_zlp_ack:1;	pending acknowledgement 4 ZLP
 * @csr;		address of config & status
 * @regs;		address of device registers
 * @*ep_regs;		address of endpoint registers
 * @data_requests;	DMA pool for data requests
 * @stp_requests;	DMA pool for setup requests
 * @phys_addr;		of device memory
 * @virt_addr;		for mapped device memory
 * @irq;		IRQ line for the device
 * @cfg_data;		current cfg, intf, and alt in use
 */

struct pch_udc_dev {
	struct usb_gadget		gadget;
	struct usb_gadget_driver	*driver;
	struct pci_dev			*pdev;
	/* all endpoints */
	struct pch_udc_ep		ep[PCH_UDC_EP_NUM];
	spinlock_t			lock;
	unsigned	active:1,
			stall:1,
			prot_stall:1,
			irq_registered:1,
			mem_region:1,
			registered:1,
			suspended:1,
			connected:1,
			set_cfg_not_acked:1,
			waiting_zlp_ack:1;
	struct pch_udc_csrs __iomem	*csr;
	struct pch_udc_regs __iomem	*regs;
	struct pch_udc_ep_regs __iomem	*ep_regs;
	struct pci_pool			*data_requests;
	struct pci_pool			*stp_requests;
	unsigned long			phys_addr;
	void __iomem			*virt_addr;
	unsigned			irq;
	struct pch_udc_cfg_data		cfg_data;
};

/**
 * pch_udc_ep - Structure holding setup request data
 *
 * @data[2];		8 bytes of setup data
 * @request;		setup request for gadget driver
 */
union pch_udc_setup_data {
	u32		data[2];
	struct usb_ctrlrequest	request;
};

#endif	/* _PCH_UDC_H_ */
