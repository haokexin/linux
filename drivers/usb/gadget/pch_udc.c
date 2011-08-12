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

#include <linux/types.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/dmapool.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/irq.h>

#include <asm/byteorder.h>
#include <asm/system.h>
#include <asm/unaligned.h>

/* gadget stack */
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include "pch_udc.h"


/* macro to set a specified bit(mask) at the specified address */
#define PCH_UDC_BIT_SET(reg, bitmask) \
		 iowrite32(((ioread32((reg)) | (bitmask))), (reg))
/* macro to clear a specified bit(mask) at the specified address */
#define PCH_UDC_BIT_CLR(reg, bitMask) \
		iowrite32((ioread32((reg)) & (~(bitMask))), (reg))

#define MAX_LOOP				200
#define PCH_UDC_PCI_BAR			1
#define MODULE_NAME				"pch_udc"
#define PCI_VENDOR_ID_INTEL			0x8086
#define PCI_DEVICE_ID_INTEL_PCH1_UDC	0x8808

#ifdef DEBUG
#define PCH_UDC_DEBUG(fmt, args...)\
	printk(KERN_DEBUG MODULE_NAME ": " fmt, ##args)
#else
#define PCH_UDC_DEBUG(fmt, args...)	do { } while (0)
#endif

const char	ep0_string[] = "ep0in";
static DEFINE_SPINLOCK(udc_stall_spinlock);	/* stall spin lock */
static u32 pch_udc_base;
static union pch_udc_setup_data setup_data;	/* received setup data */
static unsigned long ep0out_buf[64];
static dma_addr_t dma_addr;
struct pch_udc_dev *pch_udc;		/* pointer to device object */
int speed_fs;

module_param_named(speed_fs, speed_fs, bool, S_IRUGO);
MODULE_PARM_DESC(speed_fs, "true for Full speed operation");
MODULE_DESCRIPTION("OKISEMI PCH UDC - USB Device Controller");
MODULE_LICENSE("GPL");

/**
 * pch_udc_write_csr - Write the command and status registers.
 * @val:	value to be written to CSR register
 * @addr:	address of CSR register
 */
inline void pch_udc_write_csr(unsigned long val, unsigned long addr)
{
	int count = MAX_LOOP;

	/* Wait till idle */
	while ((count > 0) &&\
	(ioread32((u32 *)(PCH_UDC_CSR_BUSY_ADDR + pch_udc_base)) &
	PCH_UDC_CSR_BUSY))
		count--;

	if (count < 0)
		PCH_UDC_DEBUG("%s: wait error; count = %x", __func__, count);

	iowrite32(val, (u32 *)addr);
	/* Wait till idle */
	count = MAX_LOOP;
	while ((count > 0) &&
	(ioread32((u32 *)(PCH_UDC_CSR_BUSY_ADDR + pch_udc_base)) &
	PCH_UDC_CSR_BUSY))
		count--;

	if (count < 0)
		PCH_UDC_DEBUG("%s: wait error; count = %x", __func__, count);

}

/**
 * pch_udc_read_csr - Read the command and status registers.
 * @addr:	address of CSR register
 * Returns
 *	content of CSR register
 */
inline u32 pch_udc_read_csr(unsigned long addr)
{
	int count = MAX_LOOP;

	/* Wait till idle */
	while ((count > 0) &&
	 (ioread32((u32 *)(PCH_UDC_CSR_BUSY_ADDR + pch_udc_base)) &
	 PCH_UDC_CSR_BUSY))
		count--;

	if (count < 0)
		PCH_UDC_DEBUG("%s: wait error; count = %x", __func__, count);
	/* Dummy read */
	ioread32((u32 *)addr);
	count = MAX_LOOP;
	/* Wait till idle */
	while ((count > 0) &&
	 (ioread32((u32 *)(PCH_UDC_CSR_BUSY_ADDR + pch_udc_base)) &
	 PCH_UDC_CSR_BUSY))
		count--;
	/* actual read */
	if (count < 0)
		PCH_UDC_DEBUG("%s: wait error; count = %x", __func__, count);

	return ioread32((u32 *)addr);
}

/**
 * pch_udc_rmt_wakeup - Initiate for remote wakeup
 * @dev:	Reference to pch_udc_regs structure
 */
inline void pch_udc_rmt_wakeup(struct pch_udc_regs *dev)
{
	PCH_UDC_BIT_SET(&dev->devctl, 1 << UDC_DEVCTL_RES);
	mdelay(1);
	PCH_UDC_BIT_CLR(&dev->devctl, 1 << UDC_DEVCTL_RES);
}

/**
 * pch_udc_get_frame - Get the current frame from device status register
 * @dev:	Reference to pch_udc_regs structure
 * Retern	current frame
 */
inline int pch_udc_get_frame(struct pch_udc_regs *dev)
{
	u32 frame;

	frame = ioread32(&dev->devsts);
	return (frame & UDC_DEVSTS_TS_MASK) >> UDC_DEVSTS_TS_OFS;
}

/**
 * pch_udc_clear_selfpowered - Clear the self power control
 * @dev:	Reference to pch_udc_regs structure
 */
inline void pch_udc_clear_selfpowered(struct pch_udc_regs __iomem *dev)
{
	PCH_UDC_BIT_CLR(&dev->devcfg, 1 << UDC_DEVCFG_SP);
}

/**
 * pch_udc_set_selfpowered - Set the self power control
 * @dev:	Reference to pch_udc_regs structure
 */
inline void pch_udc_set_selfpowered(struct pch_udc_regs __iomem *dev)
{
	PCH_UDC_BIT_SET(&dev->devcfg, 1 << UDC_DEVCFG_SP);
}

/**
 * pch_udc_set_disconnect - Set the disconnect status.
 * @dev:	Reference to pch_udc_regs structure
 */
inline void pch_udc_set_disconnect(struct pch_udc_regs __iomem *dev)
{
	PCH_UDC_BIT_SET(&dev->devctl, 1 << UDC_DEVCTL_SD);
}

/**
 * pch_udc_get_speed - Return the speed status
 * @dev:	Reference to pch_udc_regs structure
 * Retern	The speed(LOW=1, FULL=2, HIGH=3)
 */
inline int pch_udc_get_speed(struct pch_udc_regs __iomem *dev)
{
	u32 val;

	val = ioread32(&dev->devsts);
	return (val & UDC_DEVSTS_ENUM_SPEED_MASK) >> UDC_DEVSTS_ENUM_SPEED_OFS;
}

/**
 * pch_udc_clear_disconnect - Clear the disconnect status.
 * @dev:	Reference to pch_udc_regs structure
 */
void pch_udc_clear_disconnect(struct pch_udc_regs __iomem *dev)
{
	/* Clear the disconnect */
	PCH_UDC_BIT_SET(&dev->devctl, 1 << UDC_DEVCTL_RES);
	PCH_UDC_BIT_CLR(&dev->devctl, 1 << UDC_DEVCTL_SD);
	mdelay(1);
	/* Resume USB signalling */
	PCH_UDC_BIT_CLR(&dev->devctl, 1 << UDC_DEVCTL_RES);
}

/**
 * pch_udc_vbus_session - set or clearr the disconnect status.
 * @dev:	Reference to pch_udc_regs structure
 * @is_active:	Parameter specifying the action
 *		- is_active = 0   indicating VBUS power is ending
 *		- is_active != 0  indicating VBUS power is starting
 */
inline void pch_udc_vbus_session(struct pch_udc_regs __iomem *dev,
					int is_active)
{
	if (is_active == 0)
		pch_udc_set_disconnect(dev);
	else
		pch_udc_clear_disconnect(dev);
}

/**
 * pch_udc_ep_set_stall - Set the stall of endpoint
 * @ep:		Reference to structure of type pch_udc_ep_regs
 */
void pch_udc_ep_set_stall(struct pch_udc_ep_regs __iomem *ep)
{
	if (EP_IS_IN(ep)) {
		PCH_UDC_BIT_SET(&ep->epctl, 1 << UDC_EPCTL_F);
		PCH_UDC_BIT_SET(&ep->epctl, 1 << UDC_EPCTL_S);
	} else {
		PCH_UDC_BIT_SET(&ep->epctl, 1 << UDC_EPCTL_S);
	}
}

/**
 * pch_udc_ep_clear_stall - Clear the stall of endpoint
 * @ep:		Reference to structure of type pch_udc_ep_regs
 */
inline void pch_udc_ep_clear_stall(struct pch_udc_ep_regs __iomem *ep)
{
	/* Clear the stall */
	PCH_UDC_BIT_CLR(&ep->epctl, 1 << UDC_EPCTL_S);
	/* clear NAK by writing CNAK */
	PCH_UDC_BIT_SET(&ep->epctl, 1 << UDC_EPCTL_CNAK);
}

/**
 * pch_udc_ep_set_trfr_type - Set the transfer type of endpoint
 * @ep:		Reference to structure of type pch_udc_ep_regs
 * @type:	Type of endpoint
 */
inline void pch_udc_ep_set_trfr_type(struct pch_udc_ep_regs __iomem *ep,
					u8 type)
{
	iowrite32(((type << UDC_EPCTL_ET_OFS) & UDC_EPCTL_ET_MASK) ,
							 &ep->epctl);
}

/**
 * pch_udc_ep_set_bufsz - Set the maximum packet size for the endpoint
 * @ep:		Reference to structure of type pch_udc_ep_regs
 * @buf_size:	The buffer size
 */
void pch_udc_ep_set_bufsz(struct pch_udc_ep_regs __iomem *ep,
						 u32 buf_size, u32 ep_in)
{
	u32 data;
	if (ep_in) {
		data = ioread32(&ep->bufin_framenum);
		data = (data & 0xffff0000) | (buf_size & 0xffff);
		iowrite32(data, &ep->bufin_framenum);
	} else {
		data = ioread32(&ep->bufout_maxpkt);
		data = (buf_size << 16) | (data & 0xffff);
		iowrite32(data, &ep->bufout_maxpkt);
	}
}

/**
 * pch_udc_ep_set_maxpkt - Set the Max packet size for the endpoint
 * @ep:		Reference to structure of type pch_udc_ep_regs
 * @pkt_size:	The packet size
 */
void pch_udc_ep_set_maxpkt(struct pch_udc_ep_regs __iomem *ep, u32 pkt_size)
{
	u32 data;
	data = ioread32(&ep->bufout_maxpkt);
	data = (data & 0xffff0000) | (pkt_size & 0xffff);
	iowrite32(data, &ep->bufout_maxpkt);
}

/**
 * pch_udc_ep_set_subptr - Set the Setup buffer pointer for the endpoint
 * @ep:		Reference to structure of type pch_udc_ep_regs
 * @addr:	Address of the register
 */
inline void pch_udc_ep_set_subptr(struct pch_udc_ep_regs __iomem *ep,
					u32 addr)
{
	iowrite32(addr, &ep->subptr);
}

/**
 * pch_udc_ep_set_ddptr - Set the Data descriptor pointer for the endpoint
 * @ep:		Reference to structure of type pch_udc_ep_regs
 * @addr:	Address of the register
 */
inline void pch_udc_ep_set_ddptr(struct pch_udc_ep_regs __iomem *ep, u32 addr)
{
	iowrite32(addr, &ep->desptr);
}

/**
 * pch_udc_ep_set_pd - Set the poll demand bit for the endpoint
 * @ep:		Reference to structure of type pch_udc_ep_regs
 */
inline void pch_udc_ep_set_pd(struct pch_udc_ep_regs __iomem *ep)
{
	PCH_UDC_BIT_SET(&ep->epctl, 1 << UDC_EPCTL_P);
}

/**
 * pch_udc_ep_set_rrdy - Set the receive ready bit for the endpoint
 * @ep:		Reference to structure of type pch_udc_ep_regs
 */
inline void pch_udc_ep_set_rrdy(struct pch_udc_ep_regs __iomem *ep)
{
	PCH_UDC_BIT_SET(&ep->epctl, 1 << UDC_EPCTL_RRDY);
}

/**
 * pch_udc_ep_clear_rrdy - Clear the receive ready bit for the endpoint
 * @ep:		Reference to structure of type pch_udc_ep_regs
 */
inline void pch_udc_ep_clear_rrdy(struct pch_udc_ep_regs __iomem *ep)
{
	PCH_UDC_BIT_CLR(&ep->epctl, 1 << UDC_EPCTL_RRDY);
}

/**
 * pch_udc_set_dma - Set the 'TDE' or RDE bit of device control
 *			register depending on the direction specified
 * @dev:	Reference to structure of type pch_udc_regs
 * @dir:	whether Tx or Rx
 *			- dir = DMA_DIR_RX Receive
 *			- dir = DMA_DIR_TX Transmit
 */
inline void pch_udc_set_dma(struct pch_udc_regs __iomem *dev, int dir)
{
	if (dir == DMA_DIR_RX)
		PCH_UDC_BIT_SET(&dev->devctl, 1 << UDC_DEVCTL_RDE);
	else if (dir == DMA_DIR_TX)
		PCH_UDC_BIT_SET(&dev->devctl, (1 << UDC_DEVCTL_TDE));

}

/**
 * pch_udc_clear_dma - Clear the 'TDE' or RDE bit of device control
 *				 register depending on the direction specified
 * @dev:	Reference to structure of type pch_udc_regs
 * @dir:	Whether Tx or Rx
 *		- dir = DMA_DIR_RX Receive
 *		- dir = DMA_DIR_TX Transmit
 */
inline void pch_udc_clear_dma(struct pch_udc_regs __iomem *dev, int dir)
{
	if (dir == DMA_DIR_RX)
		PCH_UDC_BIT_CLR(&dev->devctl, 1 << UDC_DEVCTL_RDE);
	else if (dir == DMA_DIR_TX)
		PCH_UDC_BIT_CLR(&dev->devctl, 1 << UDC_DEVCTL_TDE);

}

/**
 * pch_udc_set_csr_done - Set the device control register
 *				CSR done field (bit 13)
 * @dev:	reference to structure of type pch_udc_regs
 */
inline void pch_udc_set_csr_done(struct pch_udc_regs __iomem *dev)
{
	PCH_UDC_BIT_SET(&dev->devctl, 1 << UDC_DEVCTL_CSR_DONE);
}

/**
 * pch_udc_set_burst_length - The main tasks done by this method are:
 *			- Set the device control register burst length field
 *			- Enable the bust mode
 * @dev:	Reference to structure of type pch_udc_regs
 * @len:	Burst length
 */
void pch_udc_set_burst_length(struct pch_udc_regs __iomem *dev, u8 len)
{
	PCH_UDC_BIT_CLR(&dev->devctl, (0xff << UDC_DEVCTL_BRLEN_OFS));
	/* set Burst length  and enable burst mode*/
	PCH_UDC_BIT_SET(&dev->devctl, (len << UDC_DEVCTL_BRLEN_OFS) |
		(1 << UDC_DEVCTL_BREN));
}

/**
 * pch_udc_set_threshold_length - Set the device control register threshold
 *				length field
 *				- Enable the threshold mode
 * @dev:	Reference to structure of type pch_udc_regs
 * @len:	Burst length
 */
inline void pch_udc_set_threshold_length(struct pch_udc_regs __iomem *dev,
									u8 len)
{
	PCH_UDC_BIT_CLR(&dev->devctl, (0xff << UDC_DEVCTL_THLEN_OFS));
	/* set Burst Threshold length and enable threshold mode*/
	PCH_UDC_BIT_SET(&dev->devctl, (len << UDC_DEVCTL_THLEN_OFS) |
		(1 << UDC_DEVCTL_THE));
}

/**
 * pch_udc_disable_interrupts - Disables the specified interrupts
 * @dev:	Reference to structure of type pch_udc_regs
 * @mask:	Mask to disable interrupts
 */
inline void pch_udc_disable_interrupts(struct pch_udc_regs __iomem *dev,
								u32 mask)
{
	PCH_UDC_BIT_SET(&dev->devirqmsk, mask);
}

/**
 * pch_udc_enable_interrupts - Enable the specified interrupts
 * @dev:	Reference to structure of type pch_udc_regs
 * @mask:	Mask to enable interrupts
 */
inline void pch_udc_enable_interrupts(struct pch_udc_regs __iomem *dev,
								u32 mask)
{
	PCH_UDC_BIT_CLR(&dev->devirqmsk, mask);
}

/**
 * pch_udc_disable_ep_interrupts - Disable endpoint interrupts
 * @dev:	Reference to structure of type pch_udc_regs
 * @mask:	Mask to disable interrupts
 */
inline void pch_udc_disable_ep_interrupts(struct pch_udc_regs __iomem *dev,
								u32 mask)
{
	PCH_UDC_BIT_SET(&dev->epirqmsk, mask);
}

/**
 * pch_udc_enable_ep_interrupts - Enable endpoint interrupts
 * @dev:	Reference to structure of type pch_udc_regs
 * @mask:	Mask to enable interrupts
 */
inline void pch_udc_enable_ep_interrupts(struct pch_udc_regs __iomem *dev,
								u32 mask)
{
	PCH_UDC_BIT_CLR(&dev->epirqmsk, mask);
}

/**
 * pch_udc_read_device_interrupts - Read the device interrupts
 * @dev:	Reference to structure of type pch_udc_regs
 * Retern	The device interrupts
 */
inline u32 pch_udc_read_device_interrupts(struct pch_udc_regs __iomem *dev)
{
	return ioread32(&dev->devirqsts);
}

/**
 * pch_udc_write_device_interrupts - Write device interrupts
 * @dev:	Reference to structure of type pch_udc_regs
 * @val:	The value to be written to interrupt register
 */
inline void pch_udc_write_device_interrupts(struct pch_udc_regs __iomem *dev,
								 u32 val)
{
	iowrite32(val, &dev->devirqsts);
}

/**
 * pch_udc_read_ep_interrupts - Read the endpoint interrupts
 * @dev:	Reference to structure of type pch_udc_regs
 * Retern	The endpoint interrupt
 */
inline u32 pch_udc_read_ep_interrupts(struct pch_udc_regs __iomem *dev)
{
	return ioread32(&dev->epirqsts);
}

/**
 * pch_udc_write_ep_interrupts - Clear endpoint interupts
 * @dev:	Reference to structure of type pch_udc_regs
 * @val:	The value to be written to interrupt register
 */
inline void pch_udc_write_ep_interrupts(struct pch_udc_regs __iomem *dev,
								 u32 val)
{
	iowrite32(val, &dev->epirqsts);
}

/**
 * pch_udc_read_device_status - Read the device status
 * @dev:	Reference to structure of type pch_udc_regs
 * Retern	The device status
 */
inline u32 pch_udc_read_device_status(struct pch_udc_regs __iomem *dev)
{
	return ioread32(&dev->devsts);
}

/**
 * pch_udc_read_ep_control - Read the endpoint control
 * @ep:		Reference to structure of type pch_udc_ep_regs
 * Retern	The endpoint control register value
 */
inline u32 pch_udc_read_ep_control(struct pch_udc_ep_regs __iomem *ep)
{
	return ioread32(&ep->epctl);
}

/**
 * pch_udc_clear_ep_control - Clear the endpoint control register
 * @ep:		Reference to structure of type pch_udc_ep_regs
 * Retern	The endpoint control register value
 */
inline void pch_udc_clear_ep_control(struct pch_udc_ep_regs __iomem *ep)
{
	return iowrite32(0, &ep->epctl);
}

/**
 * pch_udc_read_ep_status - Read the endpoint status
 * @ep:		Reference to structure of type pch_udc_ep_regs
 * Retern	The endpoint status
 */
inline u32 pch_udc_read_ep_status(struct pch_udc_ep_regs __iomem *ep)
{
	return ioread32(&ep->epsts);
}

/**
 * pch_udc_clear_ep_status - Clear the endpoint status
 * @ep:		Reference to structure of type pch_udc_ep_regs
 * @stat:	Endpoint status
 */
inline void pch_udc_clear_ep_status(struct pch_udc_ep_regs __iomem *ep,
								 u32 stat)
{
	return iowrite32(stat, &ep->epsts);
}

/**
 * pch_udc_ep_set_nak - Set the bit 7 (SNAK field)
 *				of the endpoint control register
 * @ep:		Reference to structure of type pch_udc_ep_regs
 */
inline void pch_udc_ep_set_nak(struct pch_udc_ep_regs __iomem *ep)
{
	PCH_UDC_BIT_SET(&ep->epctl, 1 << UDC_EPCTL_SNAK);
}

/**
 * pch_udc_ep_clear_nak - Set the bit 8 (CNAK field)
 *				of the endpoint control register
 * @ep:		reference to structure of type pch_udc_ep_regs
 */
void pch_udc_ep_clear_nak(struct pch_udc_ep_regs __iomem *ep)
{
	unsigned int loopcnt = 0;

	if (ioread32(&ep->epctl) & (1 << UDC_EPCTL_NAK)) {
		if (!(EP_IS_IN(ep))) {
			while ((pch_udc_read_ep_status(ep) &
					 (1 << UDC_EPSTS_MRXFIFO_EMP)) == 0) {
				if (loopcnt++ > 100000) {
					PCH_UDC_DEBUG("%s: RxFIFO not Empty "
							"loop count = %d",
							__func__, loopcnt);
					break;
				}
				udelay(100);
			}
		}
		while (ioread32(&ep->epctl) & (1 << UDC_EPCTL_NAK)) {
			PCH_UDC_BIT_SET(&ep->epctl, 1 << UDC_EPCTL_CNAK);
			udelay(5);
			if (loopcnt++ >= 25) {
				PCH_UDC_DEBUG("%s: Clear NAK not set for"
						"ep%d%s: counter=%d",
						__func__, EP_NUM(ep),
						(EP_IS_IN(ep) ? "in" : "out"),
						loopcnt);
				break;
			}
		}
	}
}

/**
 * pch_udc_ep_fifo_flush - Flush the endpoint fifo
 * @ep:	reference to structure of type pch_udc_ep_regs
 * @dir:	direction of endpoint
 *		- dir = 0 endpoint is OUT
 *		- dir != 0 endpoint is IN
 */
void pch_udc_ep_fifo_flush(struct pch_udc_ep_regs __iomem *ep, int dir)
{
	unsigned int loopcnt = 0;

	PCH_UDC_DEBUG("%s: ep%d%s", __func__, EP_NUM(ep),
						 (EP_IS_IN(ep) ? "in" : "out"));
	if (dir) {	/* IN ep */
		PCH_UDC_BIT_SET(&ep->epctl, 1 << UDC_EPCTL_F);
	} else {
		if ((pch_udc_read_ep_status(ep) &
					 (1 << UDC_EPSTS_MRXFIFO_EMP)) == 0) {
			PCH_UDC_BIT_SET(&ep->epctl, 1 << UDC_EPCTL_MRXFLUSH);
			/* Wait for RxFIFO Empty */
			while ((pch_udc_read_ep_status(ep) &
					 (1 << UDC_EPSTS_MRXFIFO_EMP)) == 0) {
				if (loopcnt++ > 1000000) {
					PCH_UDC_DEBUG("RxFIFO not Empty loop"
							" count = %d", loopcnt);
					break;
				}
				udelay(100);
			}
			PCH_UDC_BIT_CLR(&ep->epctl, 1 << UDC_EPCTL_MRXFLUSH);
		}
	}
}

/**
 * pch_udc_ep_enable - This api enables endpoint
 * @regs:	Reference to structure pch_udc_ep_regs
 * @desc:	endpoint descriptor
 */
void pch_udc_ep_enable(struct pch_udc_ep_regs __iomem *regs,
				 struct pch_udc_cfg_data *cfg,
				const struct usb_endpoint_descriptor *desc)
{
	u32 ep_num = EP_NUM(regs);
	u32 ep_in = EP_IS_IN(regs);
	u32 val = 0;
	u32 buff_size = 0;

	PCH_UDC_DEBUG("%s: ep%x%s  bmAttributes = %d"
			" wMaxPacketSize = %d", __func__,
			 ep_num, (ep_in ? "in" : "out"), desc->bmAttributes,
			 desc->wMaxPacketSize);
	/* set traffic type */
	pch_udc_ep_set_trfr_type(regs, desc->bmAttributes);
	/* Set buff size */
	if (ep_in)
		buff_size = UDC_EPIN_BUFF_SIZE;
	else
		buff_size = UDC_EPOUT_BUFF_SIZE;

	pch_udc_ep_set_bufsz(regs, buff_size, ep_in);
	/* Set max packet size */
	pch_udc_ep_set_maxpkt(regs, le16_to_cpu(desc->wMaxPacketSize));
	/* Set NAK */
	pch_udc_ep_set_nak(regs);
	/* Flush fifo */
	pch_udc_ep_fifo_flush(regs, ep_in);
	/* Configure the endpoint */
	val = ep_num << UDC_CSR_NE_NUM_OFS | ep_in << UDC_CSR_NE_DIR_OFS |
			((desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) <<
			 UDC_CSR_NE_TYPE_OFS) |
			(cfg->cur_cfg << UDC_CSR_NE_CFG_OFS) |
			(cfg->cur_intf << UDC_CSR_NE_INTF_OFS) |
			(cfg->cur_alt << UDC_CSR_NE_ALT_OFS) |
			le16_to_cpu(desc->wMaxPacketSize) <<
			 UDC_CSR_NE_MAX_PKT_OFS;

	if (ep_in)
		pch_udc_write_csr(val, (u32) (pch_udc_base + UDC_CSR_ADDR +
							 (ep_num * 2) * 4));
	else
		pch_udc_write_csr(val, (u32) (pch_udc_base + UDC_CSR_ADDR +
							 (ep_num * 2 + 1) * 4));

	PCH_UDC_DEBUG("%s: Endpoint register = 0x%08x", __func__, val);
}


/**
 * pch_udc_ep_disable - This api disables endpoint
 * @regs:	Reference to structure pch_udc_ep_regs
 */
void pch_udc_ep_disable(struct pch_udc_ep_regs __iomem *regs)
{
	PCH_UDC_DEBUG("%s: enter", __func__);
	if (EP_IS_IN(regs)) {
		/* flush the fifo */
		iowrite32(1 << UDC_EPCTL_F, &regs->epctl);
		/* set NAK */
		iowrite32(1 << UDC_EPCTL_SNAK, &regs->epctl);

		PCH_UDC_BIT_SET(&regs->epsts, 1 << UDC_EPSTS_IN);
	} else {
		/* set NAK */
		iowrite32(1 << UDC_EPCTL_SNAK, &regs->epctl);
	}
	/* reset desc pointer */
	iowrite32(0, &regs->desptr);
}


/**
 * pch_udc_init - This API initializes usb device controller
 * @dev:	Rreference to pch_udc_regs structure
 */
void pch_udc_init(struct pch_udc_regs *dev)
{
	u32 reset_reg;

	PCH_UDC_DEBUG("%s: enter", __func__);
	if (NULL == dev) {
		printk(KERN_ERR MODULE_NAME ": "  "%s: Invalid address",
			__func__);
		return;
	}
	/* Set the UDC_Global variable */
	pch_udc_base = (u32) dev - UDC_DEVCFG_ADDR;
	/* Soft Reset and Reset PHY */
	reset_reg = (pch_udc_base + PCH_UDC_SRST_ADDR);
	iowrite32((1 << PCH_UDC_SRST), (u32 *)(reset_reg));
	iowrite32((1 << PCH_UDC_SRST) | (1 << PCH_UDC_PSRST),
							 (u32 *)(reset_reg));
	mdelay(1);
	iowrite32((1 << PCH_UDC_SRST), (u32 *)(reset_reg));
	iowrite32(0x00, (u32 *)(reset_reg));
	mdelay(1);

	/* mask and clear all device interrupts */
	PCH_UDC_BIT_SET(&dev->devirqmsk, UDC_DEVINT_MSK);
	PCH_UDC_BIT_SET(&dev->devirqsts, UDC_DEVINT_MSK);

	/* mask and clear all ep interrupts */
	PCH_UDC_BIT_SET(&dev->epirqmsk, UDC_EPINT_MSK_DISABLE_ALL);
	PCH_UDC_BIT_SET(&dev->epirqsts, UDC_EPINT_MSK_DISABLE_ALL);

	/* enable dynamic CSR programmingi, self powered and device speed */
	if (speed_fs) {
		PCH_UDC_BIT_SET(&dev->devcfg, (1 << UDC_DEVCFG_CSR_PRG) |
			(1 << UDC_DEVCFG_SP) | /* set self powered */
			UDC_DEVCFG_SPD_FS); /* program speed - full speed */
	} else { /* defaul high speed */
		PCH_UDC_BIT_SET(&dev->devcfg, (1 << UDC_DEVCFG_CSR_PRG) |
			(1 << UDC_DEVCFG_SP) | /* set self powered */
			UDC_DEVCFG_SPD_HS); /* program speed - high speed */
	}
#ifdef DMA_PPB_WITH_DESC_UPDATE
	PCH_UDC_BIT_SET(&dev->devctl,
			 (PCH_UDC_THLEN << UDC_DEVCTL_THLEN_OFS) |
			(PCH_UDC_BRLEN << UDC_DEVCTL_BRLEN_OFS) |
			(1 << UDC_DEVCTL_MODE) | (1 << UDC_DEVCTL_BREN) |
			(1 << UDC_DEVCTL_DU) |
			(1 << UDC_DEVCTL_THE));
#else
	PCH_UDC_BIT_SET(&dev->devctl,
			 (PCH_UDC_THLEN << UDC_DEVCTL_THLEN_OFS) |
			(PCH_UDC_BRLEN << UDC_DEVCTL_BRLEN_OFS) |
			(1 << UDC_DEVCTL_MODE) | (1 << UDC_DEVCTL_BREN) |
			(1 << UDC_DEVCTL_THE));
#endif
}

/**
 * pch_udc_exit - This API exit usb device controller
 * @dev:	Reference to pch_udc_regs structure
 */
void pch_udc_exit(struct pch_udc_regs *dev)
{
	/* mask all device interrupts */
	PCH_UDC_BIT_SET(&dev->devirqmsk, UDC_DEVINT_MSK);
	/* mask all ep interrupts */
	PCH_UDC_BIT_SET(&dev->epirqmsk, UDC_EPINT_MSK_DISABLE_ALL);
	/* put device in disconnected state */
	pch_udc_set_disconnect(dev);
}

/**
 * pch_udc_pcd_get_frame - This API is invoked to get the current frame number
 * @gadget:	Reference to the gadget driver
 * Returns
 *	0:		Success
 *	-EINVAL:	If the gadget passed is NULL
 */
static int pch_udc_pcd_get_frame(struct usb_gadget *gadget)
{
	struct pch_udc_dev		*dev;

	PCH_UDC_DEBUG("%s: enter", __func__);
	if (gadget == NULL) {
		PCH_UDC_DEBUG("%s: exit -EINVAL", __func__);
		return -EINVAL;
	}
	dev = container_of(gadget, struct pch_udc_dev, gadget);
	return pch_udc_get_frame(dev->regs);
}

/**
 * pch_udc_pcd_wakeup - This API is invoked to initiate a remote wakeup
 * @gadget:	Reference to the gadget driver
 * Returns
 *	0:		Success
 *	-EINVAL:	If the gadget passed is NULL
 */
static int pch_udc_pcd_wakeup(struct usb_gadget *gadget)
{
	struct pch_udc_dev		*dev;
	unsigned long			flags;

	PCH_UDC_DEBUG("%s: enter", __func__);
	if (gadget == NULL) {
		PCH_UDC_DEBUG("%s: exit -EINVAL", __func__);
		return -EINVAL;
	}

	dev = container_of(gadget, struct pch_udc_dev, gadget);

	PCH_UDC_DEBUG("%s: initiate remote wakeup", __func__);
	spin_lock_irqsave(&dev->lock, flags);
	pch_udc_rmt_wakeup(dev->regs);
	spin_unlock_irqrestore(&dev->lock, flags);
	return 0;
}

/**
 * pch_udc_pcd_selfpowered - This API is invoked to specify whether the device
 *				is self powered or not
 * @gadget:	Reference to the gadget driver
 * @value:	Specifies self powered or not
 * Returns
 *	0:		Success
 *	-EINVAL:	If the gadget passed is NULL
 */
static int pch_udc_pcd_selfpowered(struct usb_gadget *gadget, int value)
{
	struct pch_udc_dev		*dev;

	PCH_UDC_DEBUG("%s: enter", __func__);
	if (gadget == NULL) {
		PCH_UDC_DEBUG("%s: exit -EINVAL", __func__);
		return -EINVAL;
	}
	dev = container_of(gadget, struct pch_udc_dev, gadget);
	if (value == 0)
		pch_udc_clear_selfpowered(dev->regs);
	else
		pch_udc_set_selfpowered(dev->regs);
	return 0;
}

/**
 * pch_udc_pcd_pullup - This API is invoked to make the device
 *				visible/invisible to the host
 * @gadget:	Reference to the gadget driver
 * @is_on:	Specifies whether the pull up is made active or inactive
 * Returns
 *	0:		Success
 *	-EINVAL:	If the gadget passed is NULL
 */
static int pch_udc_pcd_pullup(struct usb_gadget *gadget, int is_on)
{
	struct pch_udc_dev		*dev;

	PCH_UDC_DEBUG("%s: enter", __func__);
	if (gadget == NULL) {
		PCH_UDC_DEBUG("%s: exit -EINVAL", __func__);
		return -EINVAL;
	}
	dev = container_of(gadget, struct pch_udc_dev, gadget);
	if (is_on == 0)
		pch_udc_set_disconnect(dev->regs);
	else
		pch_udc_clear_disconnect(dev->regs);
	return 0;
}

/**
 * pch_udc_pcd_vbus_session - This API is used by a driver for an external
 *				transceiver (or GPIO) that
 *				detects a VBUS power session starting/ending
 * @gadget:	Reference to the gadget driver
 * @is_active:	specifies whether the session is starting or ending
 * Returns
 *	0:		Success
 *	-EINVAL:	If the gadget passed is NULL
 */
static int pch_udc_pcd_vbus_session(struct usb_gadget *gadget, int is_active)
{
	struct pch_udc_dev	*dev;

	PCH_UDC_DEBUG("%s: enter", __func__);
	if (gadget == NULL) {
		PCH_UDC_DEBUG("%s: exit -EINVAL", __func__);
		return -EINVAL;
	}
	dev = container_of(gadget, struct pch_udc_dev, gadget);
	pch_udc_vbus_session(dev->regs, is_active);
	return 0;
}

/**
 * pch_udc_pcd_vbus_draw - This API is used by gadget drivers during
 *				SET_CONFIGURATION calls to
 *				specify how much power the device can consume
 * @gadget:	Reference to the gadget driver
 * @mA:		specifies the current limit in 2mA unit
 * Returns
 *	-EINVAL:	If the gadget passed is NULL
 *	-EOPNOTSUPP:
 */
static int pch_udc_pcd_vbus_draw(struct usb_gadget *gadget, unsigned int mA)
{
	PCH_UDC_DEBUG("%s: enter", __func__);
	if ((gadget == NULL) || (mA > 250)) { /* Max is 250 in 2mA unit */
		PCH_UDC_DEBUG("%s: exit -EINVAL", __func__);
		return -EINVAL;
	}
	/* Could not find any regs where we can set the limit	*/
	return -EOPNOTSUPP;
}

const struct usb_gadget_ops pch_udc_ops = {
	.get_frame = pch_udc_pcd_get_frame,
	.wakeup = pch_udc_pcd_wakeup,
	.set_selfpowered = pch_udc_pcd_selfpowered,
	.pullup = pch_udc_pcd_pullup,
	.vbus_session = pch_udc_pcd_vbus_session,
	.vbus_draw = pch_udc_pcd_vbus_draw,
};

/**
 * complete_req - This API is invoked from the driver when processing
 *			of a request is complete
 * @ep:		Reference to the endpoint structure
 * @req:	Reference to the request structure
 * @status:	Indicates the success/failure of completion
 */
static void complete_req(struct pch_udc_ep *ep, struct pch_udc_request *req,
								 int status)
{
	struct pch_udc_dev	*dev;
	unsigned halted = ep->halted;

	PCH_UDC_DEBUG("%s: enter", __func__);
	list_del_init(&req->queue);

	/* set new status if pending */
	if (req->req.status == -EINPROGRESS)
		req->req.status = status;
	else
		status = req->req.status;

	dev = ep->dev;
	if (req->dma_mapped) {
		if (ep->in) {
			pci_unmap_single(dev->pdev, req->req.dma,
					 req->req.length, PCI_DMA_TODEVICE);
		} else {
			pci_unmap_single(dev->pdev, req->req.dma,
					 req->req.length, PCI_DMA_FROMDEVICE);
		}
		req->dma_mapped = 0;
		req->req.dma = DMA_ADDR_INVALID;
	}
	ep->halted = 1;

	PCH_UDC_DEBUG("%s: %s req %p status %d len %u pch-req 0x%08x"
			"req->queue 0x%08x", __func__,
			ep->ep.name, &req->req, status, req->req.length,
			(u32)req, (u32)(&(req->queue)));
	spin_unlock(&dev->lock);
	if (!ep->in)
		pch_udc_ep_clear_rrdy(ep->regs);

	req->req.complete(&ep->ep, &req->req);

	spin_lock(&dev->lock);
	ep->halted = halted;
}

/**
 * empty_req_queue - This API empties the request queue of an endpoint
 * @ep:		Reference to the endpoint structure
 */
void empty_req_queue(struct pch_udc_ep *ep)
{
	struct pch_udc_request	*req;

	PCH_UDC_DEBUG("%s: enter", __func__);
	ep->halted = 1;
	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct pch_udc_request, queue);
		PCH_UDC_DEBUG("%s: complete_req  ep%d%s", __func__, ep->num,
						 (ep->in ? "in" : "out"));
		complete_req(ep, req, -ESHUTDOWN);
	}
}

/**
 * pch_udc_free_dma_chain - This function frees the DMA chain created
 *				for the request
 * @dev		Reference to the driver structure
 * @req		Reference to the request to be freed
 * Return	0: Success
 */
static int pch_udc_free_dma_chain(struct pch_udc_dev *dev,
						 struct pch_udc_request *req)
{
	struct pch_udc_data_dma_desc	*td;
	struct pch_udc_data_dma_desc	*td_last;
	u32 i;

	PCH_UDC_DEBUG("%s: enter", __func__);
	/* do not free first desc., will be done by free for request */
	td_last = req->td_data;
	td = phys_to_virt(td_last->next);

	for (i = 1; i < req->chain_len; i++) {
		pci_pool_free(dev->data_requests, td,
				(dma_addr_t) td_last->next);
		td_last = td;
		td = phys_to_virt(td_last->next);
	}
	return 0;
}

/**
 * pch_udc_create_dma_chain - This function creates or reinitializes
 *				a DMA chain
 * @ep:		Reference to the endpoint structure
 * @req:	Reference to the request
 * @buf_len:	The buffer length
 * @gfp_flags:	Flags to be used while mapping the data buffer
 * Return
 *	0:		success,
 *	-ENOMEM:	pci_pool_alloc invocation fails
 */
static int pch_udc_create_dma_chain(struct pch_udc_ep *ep,
			struct pch_udc_request *req,
			 unsigned long buf_len, gfp_t gfp_flags)
{
	unsigned long bytes = req->req.length;
	unsigned int i;
	dma_addr_t dma_addr;
	struct pch_udc_data_dma_desc	*td = NULL;
	struct pch_udc_data_dma_desc	*last = NULL;
	unsigned long txbytes;
	unsigned len;

	PCH_UDC_DEBUG("%s: enter  bytes = %ld buf_len = %ld",
				__func__, bytes, buf_len);
	/* unset L bit in first desc for OUT */
	if (!ep->in)
		req->td_data->status = PCH_UDC_BS_HST_BSY;

	/* alloc only new desc's if not already available */
	len = req->req.length / buf_len;
	if (req->req.length % buf_len)
		len++;

	/* shorter chain already allocated before */
	if (req->chain_len > 1)
		pch_udc_free_dma_chain(ep->dev, req);

	req->chain_len = len;

	td = req->td_data;
	/* gen. required number of descriptors and buffers */
	for (i = buf_len; i < bytes; i += buf_len) {
		dma_addr = DMA_ADDR_INVALID;
		/* create or determine next desc. */
		td = pci_pool_alloc(ep->dev->data_requests, gfp_flags,
								 &dma_addr);
		if (td == NULL)
			return -ENOMEM;

		td->status = 0;
		td->dataptr = req->req.dma + i; /* assign buffer */

		if ((bytes - i) >= buf_len) {
			txbytes = buf_len;
		} else { /* short packet */
			txbytes = bytes - i;
		}
		/* link td and assign tx bytes */
		if (i == buf_len) {
			req->td_data->next = dma_addr;
			/* set the count bytes */
			if (ep->in) {
				req->td_data->status = PCH_UDC_BS_HST_BSY |
								 buf_len;
				/* second desc */
				td->status = PCH_UDC_BS_HST_BSY | txbytes;
			} else {
				td->status = PCH_UDC_BS_HST_BSY;
			}
		} else {
			last->next = dma_addr;
			if (ep->in)
				td->status = PCH_UDC_BS_HST_BSY | txbytes;
			else
				td->status = PCH_UDC_BS_HST_BSY;

		}
		last = td;
	}
	/* set last bit */
	if (td) {
		td->status |= PCH_UDC_DMA_LAST;
		/* last desc. points to itself */
		req->td_data_last = td;
		td->next = req->td_data_phys;
	}
	return 0;
}

/**
 * prepare_dma - This function creates and initializes the DMA chain
 *			for the request
 * @ep:		Reference to the endpoint structure
 * @req:	Reference to the request
 * @gfp:	Flag to be used while mapping the data buffer
 * Returns
 *	0:		Success
 *	Other 0:	linux error number on failure
 */
static int prepare_dma(struct pch_udc_ep *ep, struct pch_udc_request *req,
								 gfp_t gfp)
{
	int	retval = 0;

	PCH_UDC_DEBUG("%s: enter  req->req.dma=0x%08x", __func__, req->req.dma);
	/* set buffer pointer */
	req->td_data->dataptr = req->req.dma;
	/* set last bit */
	req->td_data->status |= PCH_UDC_DMA_LAST;

	/* Allocate and create a DMA chain */
	retval = pch_udc_create_dma_chain(ep, req, ep->ep.maxpacket, gfp);
	if (retval != 0) {
		if (retval == -ENOMEM)
			printk(KERN_ERR MODULE_NAME ": "
				"%s: Out of DMA memory", __func__);
		return retval;
	}
	if (ep->in) {
		if (req->req.length <= ep->ep.maxpacket) {
			/* write tx bytes */
			req->td_data->status = PCH_UDC_DMA_LAST |
					 PCH_UDC_BS_HST_BSY | req->req.length;
		}
	}

	if (ep->in) {
		/* if bytes < max packet then tx bytes must
		 *be written in packet per buffer mode
		 */
		if ((req->req.length < ep->ep.maxpacket) || (ep->num == 0)) {
			/* write the count */
			req->td_data->status = (req->td_data->status &
							 ~PCH_UDC_RXTX_BYTES) |
								req->req.length;
		}
		/* set HOST BUSY */
		req->td_data->status = (req->td_data->status &
							 ~PCH_UDC_BUFF_STS) |
							PCH_UDC_BS_HST_BSY;
	}
	return retval;
}

/**
 * process_zlp - This function process zero length packets
 *			from the gadget driver
 * @ep:		Reference to the endpoint structure
 * @req:	Reference to the request
 */
static void process_zlp(struct pch_udc_ep *ep, struct pch_udc_request *req)
{
	struct pch_udc_dev	*dev = ep->dev;

	PCH_UDC_DEBUG("%s: enter  ep%d%s",
				__func__, ep->num, (ep->in ? "in" : "out"));
	/* IN zlp's are handled by hardware */
	complete_req(ep, req, 0);

	/* if set_config or set_intf is waiting for ack by zlp
	 *then set CSR_DONE
	 */
	if (dev->set_cfg_not_acked) {
		PCH_UDC_DEBUG("%s: process_zlp: csr done", __func__);
		pch_udc_set_csr_done(dev->regs);
		dev->set_cfg_not_acked = 0;
	}
	/* setup command is ACK'ed now by zlp */
	if (!dev->stall) {
		if (dev->waiting_zlp_ack) {
			/* clear NAK by writing CNAK in EP0_IN */
			pch_udc_ep_clear_nak(dev->ep[UDC_EP0IN_IDX].regs);
			dev->waiting_zlp_ack = 0;
		}
	}
}

/**
 * pch_udc_start_rxrequest - This function starts the receive requirement.
 * @ep:		Reference to the endpoint structure
 * @req:	Reference to the request structure
 */
static void pch_udc_start_rxrequest(struct pch_udc_ep *ep,
						 struct pch_udc_request *req)
{
	struct pch_udc_data_dma_desc *td_data;

	PCH_UDC_DEBUG("%s: enter", __func__);
	pch_udc_clear_dma(ep->dev->regs, DMA_DIR_RX);
	td_data = req->td_data;
	ep->td_data = req->td_data;
	/* Set the status bits for all descriptors */
	while (1) {
		td_data->status = (td_data->status & ~PCH_UDC_BUFF_STS) |
							 PCH_UDC_BS_HST_RDY;
		if ((td_data->status & PCH_UDC_DMA_LAST) ==  PCH_UDC_DMA_LAST)
			break;

		td_data = (struct pch_udc_data_dma_desc *) \
						phys_to_virt(td_data->next);
	}
	/* Write the descriptor pointer */
	pch_udc_ep_set_ddptr(ep->regs, req->td_data_phys);
	req->dma_going = 1;
	pch_udc_enable_ep_interrupts(ep->dev->regs, 1 <<
						 (ep->num + UDC_EPINT_OUT_EP0));
	pch_udc_set_dma(ep->dev->regs, DMA_DIR_RX);
	pch_udc_ep_clear_nak(ep->regs);
	pch_udc_ep_set_rrdy(ep->regs);
}

/**
 * pch_udc_pcd_ep_enable - This API enables the endpoint. It is called
 *				from gadget driver
 * @usbep:	Reference to the USB endpoint structure
 * @desc:	Reference to the USB endpoint descriptor structure
 * Returns
 *	0:		Success
 *	-EINVAL:
 *	-ESHUTDOWN:
 */
static int pch_udc_pcd_ep_enable(struct usb_ep *usbep,
			 const struct usb_endpoint_descriptor *desc)
{
	struct pch_udc_ep	*ep;
	struct pch_udc_dev	*dev;
	unsigned long		iflags;

	PCH_UDC_DEBUG("%s: enter", __func__);
	if ((usbep == NULL) || (usbep->name == ep0_string) || (desc == NULL) ||
		(desc->bDescriptorType != USB_DT_ENDPOINT) ||
						 (desc->wMaxPacketSize == 0)) {
		return -EINVAL;
	}

	ep = container_of(usbep, struct pch_udc_ep, ep);
	dev = ep->dev;

	PCH_UDC_DEBUG("%s: ep %d", __func__, ep->num);
	if ((dev->driver == NULL) || (dev->gadget.speed == USB_SPEED_UNKNOWN))
		return -ESHUTDOWN;


	spin_lock_irqsave(&dev->lock, iflags);
	ep->desc = desc;
	ep->halted = 0;
	pch_udc_ep_enable(ep->regs, &ep->dev->cfg_data, desc);
	ep->ep.maxpacket = le16_to_cpu(desc->wMaxPacketSize);
	pch_udc_enable_ep_interrupts(ep->dev->regs,
			1 << (ep->in ? ep->num : ep->num + UDC_EPINT_OUT_EP0));

	PCH_UDC_DEBUG("%s: %s enabled", __func__, usbep->name);

	spin_unlock_irqrestore(&dev->lock, iflags);
	return 0;
}

/**
 * pch_udc_pcd_ep_disable - This API disables endpoint and is called
 *				from gadget driver
 * @usbep	Reference to the USB endpoint structure
 * Returns
 *	0:		Success
 *	-EINVAL:
 */
static int pch_udc_pcd_ep_disable(struct usb_ep *usbep)
{
	struct pch_udc_ep	*ep = NULL;
	unsigned long	iflags;

	PCH_UDC_DEBUG("%s: enter", __func__);
	if (usbep == NULL)
		return -EINVAL;

	ep = container_of(usbep, struct pch_udc_ep, ep);
	if ((usbep->name == ep0_string) || (ep->desc == NULL))
		return -EINVAL;

	PCH_UDC_DEBUG("%s: ep%d%s", __func__, ep->num,
						 (ep->in ? "in" : "out"));
	spin_lock_irqsave(&ep->dev->lock, iflags);
	empty_req_queue(ep);
	ep->halted = 1;
	pch_udc_ep_disable(ep->regs);

	/* disable interrupt */
	pch_udc_disable_ep_interrupts(ep->dev->regs,
			1 << (ep->in ? ep->num : ep->num + UDC_EPINT_OUT_EP0));
	ep->desc = NULL;
	INIT_LIST_HEAD(&ep->queue);
	spin_unlock_irqrestore(&ep->dev->lock, iflags);
	return 0;
}

/**
 * pch_udc_alloc_request - This function allocates request structure.
 *				It iscalled by gadget driver
 * @usbep:	Reference to the USB endpoint structure
 * @gfp:	Flag to be used while allocating memory
 * Returns
 *	NULL:			Failure
 *	Allocated address:	Success
 */
static struct usb_request *pch_udc_alloc_request(struct usb_ep *usbep,
								 gfp_t gfp)
{
	struct pch_udc_request	*req;
	struct pch_udc_ep		*ep;

	PCH_UDC_DEBUG("%s: enter", __func__);
	if (usbep == NULL)
		return NULL;

	ep = container_of(usbep, struct pch_udc_ep, ep);
	PCH_UDC_DEBUG("%s: ep %s", __func__, usbep->name);
	req = kzalloc(sizeof(struct pch_udc_request), gfp);
	if (req == NULL) {
		PCH_UDC_DEBUG("%s: no memory for request", __func__);
		return NULL;
	}
	memset(req, 0, sizeof(struct pch_udc_request));
	req->req.dma = DMA_ADDR_INVALID;
	INIT_LIST_HEAD(&req->queue);

	if (ep->dma != NULL) {
		struct pch_udc_data_dma_desc	*dma_desc;

		/* ep0 in requests are allocated from data pool here */
		dma_desc = pci_pool_alloc(ep->dev->data_requests, gfp,
							 &req->td_data_phys);
		if (NULL == dma_desc) {
			kfree(req);
			return NULL;
		}

		PCH_UDC_DEBUG("%s: req = 0x%p dma_desc = 0x%p, "
			"td_phys = 0x%08lx", __func__,
			req, dma_desc, (unsigned long)req->td_data_phys);

		/* prevent from using desc. - set HOST BUSY */
		dma_desc->status |= PCH_UDC_BS_HST_BSY;
		dma_desc->dataptr = __constant_cpu_to_le32(DMA_ADDR_INVALID);
		req->td_data = dma_desc;
		req->td_data_last = dma_desc;
		req->chain_len = 1;
	}
	return &req->req;
}

/**
 * pch_udc_free_request - This function frees request structure.
 *				It is called by gadget driver
 * @usbep:	Reference to the USB endpoint structure
 * @usbreq:	Reference to the USB request
 */
static void pch_udc_free_request(struct usb_ep *usbep,
						 struct usb_request *usbreq)
{
	struct pch_udc_ep	*ep;
	struct pch_udc_request	*req;

	PCH_UDC_DEBUG("%s: enter", __func__);
	if ((usbep == NULL) || (usbreq == NULL))
		return;

	ep = container_of(usbep, struct pch_udc_ep, ep);
	req = container_of(usbreq, struct pch_udc_request, req);
	PCH_UDC_DEBUG("%s: %s  req = 0x%p", __func__, usbep->name, req);

	if (!list_empty(&req->queue))
		printk(KERN_ERR MODULE_NAME ": "
			"%s: %s  req = 0x%p  queue not empty",
			__func__, usbep->name, req);

	if (req->td_data != NULL) {
		if (req->chain_len > 1)
			pch_udc_free_dma_chain(ep->dev, req);
		else
			pci_pool_free(ep->dev->data_requests, req->td_data,
							 req->td_data_phys);

	}
	kfree(req);
}

/**
 * pch_udc_pcd_queue - This function queues a request packet. It is called
 *			by gadget driver
 * @usbep:	Reference to the USB endpoint structure
 * @usbreq:	Reference to the USB request
 * @gfp:	Flag to be used while mapping the data buffer
 * Returns
 *	0:			Success
 *	linux error number:	Failure
 */
static int pch_udc_pcd_queue(struct usb_ep *usbep, struct usb_request *usbreq,
								 gfp_t gfp)
{
	int retval = 0;
	struct pch_udc_ep	*ep;
	struct pch_udc_dev	*dev;
	struct pch_udc_request	*req;
	unsigned long	iflags;

	PCH_UDC_DEBUG("%s: enter", __func__);
	if ((usbep == NULL) || (usbreq == NULL) || (usbreq->complete == NULL) ||
						 (usbreq->buf == NULL)) {
		PCH_UDC_DEBUG("%s: Invalid end point OR request", __func__);
		return -EINVAL;
	}

	ep = container_of(usbep, struct pch_udc_ep, ep);
	if ((ep->desc == NULL) && (ep->num != 0)) {
		PCH_UDC_DEBUG("%s: Trying to queue before before enabling "
				"the end point %d", __func__, ep->num);
		/* Don't let non-control ep queue before enable */
		return -EINVAL;
	}
	req = container_of(usbreq, struct pch_udc_request, req);
	PCH_UDC_DEBUG("%s: ep%d%s  req = 0x%08x", __func__, ep->num,
					 (ep->in ? "in" : "out"), (u32)req);
	if (!list_empty(&req->queue)) {
		PCH_UDC_DEBUG("%s: list_empty error: req->queue = 0x%08x"
				 " pch-req = 0x%08x",
				__func__, (u32)(&(req->queue)), (u32)req);
		return -EINVAL;
	}
	dev = ep->dev;
	if ((dev->driver == NULL) || (dev->gadget.speed == USB_SPEED_UNKNOWN)) {
		PCH_UDC_DEBUG("%s: Gadget not bound/invalid dev->driver = 0x%p "
				" speed = 0x%x",
				__func__, dev->driver, dev->gadget.speed);
		return -ESHUTDOWN;
	}
	spin_lock_irqsave(&ep->dev->lock, iflags);
	/* map the buffer for dma */
	if ((usbreq->length != 0) &&
		((usbreq->dma == DMA_ADDR_INVALID) || (usbreq->dma == 0))) {
		if (ep->in) {
			usbreq->dma = pci_map_single(dev->pdev, usbreq->buf,
					usbreq->length, PCI_DMA_TODEVICE);
		} else {
			usbreq->dma = pci_map_single(dev->pdev, usbreq->buf,
					usbreq->length, PCI_DMA_FROMDEVICE);
		}
		req->dma_mapped = 1;
	}

	if (usbreq->length > 0) { /* setup the descriptors */
		retval = prepare_dma(ep, req, gfp);
		if (retval != 0) {
			/* Need to unmap before returning? ...
							 req->dma_mapped = 1; */
			spin_unlock_irqrestore(&dev->lock, iflags);
			return retval;
		}
	}

	usbreq->actual = 0;
	usbreq->status = -EINPROGRESS;
	req->dma_done = 0;

	if (list_empty(&ep->queue) && !ep->halted) {
		/* no pending transfer, so start this req */
		if ((usbreq->length == 0)) {
			process_zlp(ep, req);
			spin_unlock_irqrestore(&dev->lock, iflags);
			return 0;
		}
		if (!ep->in) {
			pch_udc_start_rxrequest(ep, req);
		} else {
			/*
			* For IN trfr the descriptors will be programmed and
			* P bit will be set when
			* we get an IN token
			*/

			while (pch_udc_read_ep_control(ep->regs) &
							 (1 << UDC_EPCTL_S))
				udelay(100);

			pch_udc_ep_clear_nak(ep->regs);
			pch_udc_enable_ep_interrupts(ep->dev->regs,
							 (1 << ep->num));
			/* enable DMA */
			pch_udc_set_dma(dev->regs, DMA_DIR_TX);
		}
	}
	PCH_UDC_DEBUG("%s: desc[stat:0x%08x  dptr:0x%08x next:0x%08x]",
			__func__, req->td_data->status, req->td_data->dataptr,
							 req->td_data->next);
	/* Now add this request to the ep's pending requests */
	if (req != NULL)
		list_add_tail(&req->queue, &ep->queue);

	spin_unlock_irqrestore(&dev->lock, iflags);
	return retval;
}

/**
 * pch_udc_pcd_dequeue - This function de-queues a request packet.
 *				It is called by gadget driver
 * @usbep:	Reference to the USB endpoint structure
 * @usbreq:	Reference to the USB request
 * Returns
 *	0:			Success
 *	linux error number:	Failure
 */
static int pch_udc_pcd_dequeue(struct usb_ep *usbep,
				struct usb_request *usbreq)
{
	struct pch_udc_ep		*ep;
	struct pch_udc_request	*req;
	unsigned long			flags;

	PCH_UDC_DEBUG("%s: enter", __func__);
	ep = container_of(usbep, struct pch_udc_ep, ep);
	if ((usbep == NULL) || (usbreq == NULL) ||
			((ep->desc == NULL) && (ep->num != 0))) {
		return -EINVAL;
	}
	PCH_UDC_DEBUG("%s: enter  ep%d%s", __func__, ep->num,
						 (ep->in ? "in" : "out"));
	req = container_of(usbreq, struct pch_udc_request, req);
	spin_lock_irqsave(&ep->dev->lock, flags);
	/* make sure it's still queued on this endpoint */
	list_for_each_entry(req, &ep->queue, queue) {
		if (&req->req == usbreq)
			break;

	}

	if (&req->req != usbreq) {
		spin_unlock_irqrestore(&ep->dev->lock, flags);
		return -EINVAL;
	}
	pch_udc_ep_set_nak(ep->regs);
	if (!list_empty(&req->queue))
		complete_req(ep, req, -ECONNRESET);

	spin_unlock_irqrestore(&ep->dev->lock, flags);
	return 0;
}

/**
 * pch_udc_pcd_set_halt - This function Sets or clear the endpoint halt feature
 * @usbep:	Reference to the USB endpoint structure
 * @halt:	Specifies whether to set or clear the feature
 * Returns
 *	0:			Success
 *	linux error number:	Failure
 */
static int pch_udc_pcd_set_halt(struct usb_ep *usbep, int halt)
{
	struct pch_udc_ep	*ep;
	unsigned long iflags;

	if (usbep == NULL)
		return -EINVAL;


	PCH_UDC_DEBUG("%s: %s: halt=%d", __func__, usbep->name, halt);
	ep = container_of(usbep, struct pch_udc_ep, ep);
	if ((ep->desc == NULL) && (ep->num == 0)) {
		PCH_UDC_DEBUG("%s: ep->desc = 0x%x: ep->num = 0x%x",
				__func__, (u32)(ep->desc), ep->num);
		return -EINVAL;
	}
	if ((ep->dev->driver == NULL) || (ep->dev->gadget.speed\
						 == USB_SPEED_UNKNOWN)) {
		PCH_UDC_DEBUG("%s: ep->dev->driver = 0x%x:"
				" ep->dev->gadget.speed = 0x%x",
				__func__, (u32)(ep->dev->driver),
				 ep->dev->gadget.speed);
		return -ESHUTDOWN;
	}

	spin_lock_irqsave(&udc_stall_spinlock, iflags);

	if (!list_empty(&ep->queue)) {
		PCH_UDC_DEBUG("%s: list not empty", __func__);
		spin_unlock_irqrestore(&udc_stall_spinlock, iflags);
		return -EAGAIN;
	}
	/* halt or clear halt */
	if (halt == 0) {
			pch_udc_ep_clear_stall(ep->regs);
	} else {
		if (ep->num == PCH_UDC_EP0)
			ep->dev->stall = 1;

		pch_udc_ep_set_stall(ep->regs);
		pch_udc_enable_ep_interrupts(ep->dev->regs,
			1 << (ep->in ? ep->num : ep->num + UDC_EPINT_OUT_EP0));
	}
	spin_unlock_irqrestore(&udc_stall_spinlock, iflags);
	return 0;
}

/**
 * pch_udc_pcd_set_wedge - This function Sets or clear the endpoint
 *				halt feature
 * @usbep:	Reference to the USB endpoint structure
 * @halt:	Specifies whether to set or clear the feature
 * Returns
 *	0:			Success
 *	linux error number:	Failure
 */
static int pch_udc_pcd_set_wedge(struct usb_ep *usbep)
{
	struct pch_udc_ep	*ep;
	unsigned long iflags;

	if (usbep == NULL)
		return -EINVAL;

	PCH_UDC_DEBUG("%s: %s:", __func__, usbep->name);
	ep = container_of(usbep, struct pch_udc_ep, ep);
	if ((ep->desc == NULL) && (ep->num == 0)) {
		PCH_UDC_DEBUG("%s: ep->desc = 0x%x: ep->num = 0x%x",
				__func__, (u32)(ep->desc), ep->num);
		return -EINVAL;
	}
	if ((ep->dev->driver == NULL) || (ep->dev->gadget.speed ==\
							 USB_SPEED_UNKNOWN)) {
		PCH_UDC_DEBUG("%s: ep->dev->driver = 0x%x:"
				" ep->dev->gadget.speed = 0x%x",
				__func__, (u32)(ep->dev->driver),
				 ep->dev->gadget.speed);
		return -ESHUTDOWN;
	}

	spin_lock_irqsave(&udc_stall_spinlock, iflags);

	if (!list_empty(&ep->queue)) {
		PCH_UDC_DEBUG("%s: list not empty", __func__);
		spin_unlock_irqrestore(&udc_stall_spinlock, iflags);
		return -EAGAIN;
	}
	/* halt */
	if (ep->num == PCH_UDC_EP0)
		ep->dev->stall = 1;

	pch_udc_ep_set_stall(ep->regs);
	pch_udc_enable_ep_interrupts(ep->dev->regs,
		1 << (ep->in ? ep->num : ep->num + UDC_EPINT_OUT_EP0));

	ep->dev->prot_stall = 1;
	spin_unlock_irqrestore(&udc_stall_spinlock, iflags);
	return 0;
}

/**
 * pch_udc_pcd_fifo_flush - This function Flush the FIFO of specified endpoint
 * @usbep:	Reference to the USB endpoint structure
 */
static void pch_udc_pcd_fifo_flush(struct usb_ep *usbep)
{
	struct pch_udc_ep	*ep;

	if (usbep == NULL)
		return;

	PCH_UDC_DEBUG("%s: %s", __func__, usbep->name);
	ep = container_of(usbep, struct pch_udc_ep, ep);
	if ((ep->desc == NULL) && (ep->num != 0))
		return;
	pch_udc_ep_fifo_flush(ep->regs, ep->in);
}

static const struct usb_ep_ops pch_udc_ep_ops = {
	.enable		= pch_udc_pcd_ep_enable,
	.disable	= pch_udc_pcd_ep_disable,
	.alloc_request	= pch_udc_alloc_request,
	.free_request	= pch_udc_free_request,
	.queue		= pch_udc_pcd_queue,
	.dequeue	= pch_udc_pcd_dequeue,
	.set_halt	= pch_udc_pcd_set_halt,
	.set_wedge	= pch_udc_pcd_set_wedge,
	.fifo_status	= NULL,
	.fifo_flush	= pch_udc_pcd_fifo_flush,
};

/**
 * pch_udc_init_setup_buff - This function initializes the SETUP buffer
 * @td_stp:	Reference to the SETP buffer structure
 */
static void pch_udc_init_setup_buff(struct pch_udc_stp_dma_desc *td_stp)
{
	static u32	pky_marker;

	PCH_UDC_DEBUG("%s: enter", __func__);
	if (td_stp == NULL) {
		PCH_UDC_DEBUG("%s: SETUP BUFF == NULL", __func__);
		return;
	}
	td_stp->reserved = ++pky_marker;
	td_stp->data12 = 0xFFFFFFFF;
	td_stp->data34 = 0xFFFFFFFF;
	td_stp->status = PCH_UDC_BS_HST_RDY;
}

/**
 * pch_udc_start_next_txrequest - This function starts
 *					the next transmission requirement
 * @ep:	Reference to the endpoint structure
 */
static void pch_udc_start_next_txrequest(struct pch_udc_ep *ep)
{
	struct pch_udc_request *req;

	PCH_UDC_DEBUG("%s: enter", __func__);
	if (pch_udc_read_ep_control(ep->regs) & (1 << UDC_EPCTL_P))
		return;

	if (!list_empty(&ep->queue)) {
		/* next request */
		req = list_entry(ep->queue.next, struct pch_udc_request, queue);
		if (req && !req->dma_going) {
			PCH_UDC_DEBUG("%s: Set request: req=%p req->td_data=%p",
					__func__, req, req->td_data);
			if (req->td_data) {
				struct pch_udc_data_dma_desc *td_data;

				while (pch_udc_read_ep_control(ep->regs) &
							 (1 << UDC_EPCTL_S))
					udelay(100);

				req->dma_going = 1;
				/* Clear the descriptor pointer */
				pch_udc_ep_set_ddptr(ep->regs, 0);

				td_data = req->td_data;
				while (1) {
					td_data->status = (td_data->status &
						 ~PCH_UDC_BUFF_STS) |
						 PCH_UDC_BS_HST_RDY;
					if ((td_data->status &
						 PCH_UDC_DMA_LAST) ==
						 PCH_UDC_DMA_LAST)
						break;

					td_data =
					(struct pch_udc_data_dma_desc *)\
					phys_to_virt(td_data->next);
				}
				/* Write the descriptor pointer */
				pch_udc_ep_set_ddptr(ep->regs,
							 req->td_data_phys);
				pch_udc_set_dma(ep->dev->regs, DMA_DIR_TX);
				/* Set the poll demand bit */
				pch_udc_ep_set_pd(ep->regs);
				pch_udc_enable_ep_interrupts(ep->dev->regs,
						1 << (ep->in ? ep->num :\
						 ep->num + UDC_EPINT_OUT_EP0));
				pch_udc_ep_clear_nak(ep->regs);
			}
		}
	}
}

/**
 * pch_udc_complete_transfer - This function completes a transfer
 * @ep:		Reference to the endpoint structure
 */
static void pch_udc_complete_transfer(struct pch_udc_ep	*ep)
{
	struct pch_udc_request *req;

	PCH_UDC_DEBUG("%s: enter", __func__);
	if (!list_empty(&ep->queue)) {
		PCH_UDC_DEBUG("%s: list_entry", __func__);
		req = list_entry(ep->queue.next, struct pch_udc_request, queue);
		if (req && ((req->td_data_last->status & PCH_UDC_BUFF_STS) ==
							 PCH_UDC_BS_DMA_DONE)) {
#ifdef DMA_PPB_WITH_DESC_UPDATE
			struct pch_udc_data_dma_desc *td_data = req->td_data;
			for (i = 0; i < req->chain_len; i++) {
				if ((td_data->status & PCH_UDC_RXTX_STS) !=
							 PCH_UDC_RTS_SUCC) {
					printk(KERN_ERR MODULE_NAME ": "
						"Invalid RXTX status"
						" (0x%08x) epstatus=0x%08x\n",
						(td_data->status &
							 PCH_UDC_RXTX_STS),
						 (int)(ep->epsts));
					return;
				}
				td_data = (struct pch_udc_data_dma_desc *)
						 phys_to_virt(td_data->next);
			}
#else
			if ((req->td_data_last->status & PCH_UDC_RXTX_STS) !=
							 PCH_UDC_RTS_SUCC) {
				printk(KERN_ERR MODULE_NAME ": "
					"Invalid RXTX status (0x%08x)"
					" epstatus=0x%08x\n",
					(req->td_data_last->status &
						 PCH_UDC_RXTX_STS),
					 (int)(ep->epsts));
				return;
			}
#endif
			req->req.actual = req->req.length;
			req->td_data_last->status = PCH_UDC_BS_HST_BSY |
							 PCH_UDC_DMA_LAST;
			req->td_data->status = PCH_UDC_BS_HST_BSY |
							 PCH_UDC_DMA_LAST;
			/* complete req */
			complete_req(ep, req, 0);
			req->dma_going = 0;
			if (!list_empty(&ep->queue)) {
				while (pch_udc_read_ep_control(ep->regs) &
							 (1 << UDC_EPCTL_S))
					udelay(100);

				pch_udc_ep_clear_nak(ep->regs);
				pch_udc_enable_ep_interrupts(ep->dev->regs,
					1 << (ep->in ? ep->num : ep->num +
							 UDC_EPINT_OUT_EP0));
			} else {
				pch_udc_disable_ep_interrupts(ep->dev->regs,
					1 << (ep->in ? ep->num : ep->num +
							 UDC_EPINT_OUT_EP0));
			}
		}
	}
}

/**
 * pch_udc_complete_receiver - This function completes a receiver
 * @ep:		Reference to the endpoint structure
 */
static void pch_udc_complete_receiver(struct pch_udc_ep	*ep)
{
	struct pch_udc_request *req;
	unsigned int count;

	PCH_UDC_DEBUG("%s: enter", __func__);
	if (!list_empty(&ep->queue)) {
		/* next request */
		req = list_entry(ep->queue.next, struct pch_udc_request, queue);
		if (req && (req->td_data_last->status & PCH_UDC_BUFF_STS) ==
							PCH_UDC_BS_DMA_DONE) {
			PCH_UDC_DEBUG("%s: ep%d%s  DMA Done", __func__,
					 ep->num, (ep->in ? "in" : "out"));
			/* Disable DMA */
			pch_udc_clear_dma(ep->dev->regs, DMA_DIR_RX);
#ifdef DMA_PPB_WITH_DESC_UPDATE
{
			/* Get Rx bytes */
			struct pch_udc_data_dma_desc *td_data = req->td_data;
			for (i = 0, count = 0; i < req->chain_len; i++) {
				if ((td_data->status & PCH_UDC_RXTX_STS) !=
							 PCH_UDC_RTS_SUCC) {
					printk(KERN_ERR MODULE_NAME ": "
						"Invalid RXTX status"
						" (0x%08x) epstatus=0x%08x\n",
						(td_data->status &
							 PCH_UDC_RXTX_STS),
						 (int)(ep->epsts));
					return;
				}
				count += td_data->status & PCH_UDC_RXTX_BYTES;
				td_data = (struct pch_udc_data_dma_desc *)\
						 phys_to_virt(td_data->next);
			}
}
#else
			if ((req->td_data_last->status & PCH_UDC_RXTX_STS) !=
							 PCH_UDC_RTS_SUCC) {
				printk(KERN_ERR MODULE_NAME ": "
					"Invalid RXTX status (0x%08x)"
					" epstatus=0x%08x\n",
					(req->td_data_last->status &
						 PCH_UDC_RXTX_STS),
					 (int)(ep->epsts));
				return;
			}
			count = req->td_data_last->status & PCH_UDC_RXTX_BYTES;
#endif
			if ((count == 0) && (req->req.length ==
						 UDC_DMA_MAXPACKET)) {
				/* on 64k packets the RXBYTES field is zero */
				count = UDC_DMA_MAXPACKET;
			}

			/* Set the descriptor status */
			req->td_data->status |= PCH_UDC_DMA_LAST;
			req->td_data_last->status |= PCH_UDC_BS_HST_BSY;

			req->dma_going = 0;
			/* complete request */
			req->req.actual = count;
			complete_req(ep, req, 0);

			/* If there is a new/failed requests try that now */
			if (!list_empty(&ep->queue)) {
				req = list_entry(ep->queue.next,
					 struct pch_udc_request, queue);
				pch_udc_start_rxrequest(ep, req);
			}
		}
#ifdef DMA_PPB_WITH_DESC_UPDATE
		else {
			PCH_UDC_DEBUG("%s: ep%d%s  DMA not Done",
				__func__, ep->num, (ep->in ? "in" : "out"));
			pch_udc_ep_set_rrdy(ep->regs);
		}
#endif
	}
}

/**
 * pch_udc_svc_data_in - This function process endpoint interrupts
 *				for IN endpoints
 * @dev:	Reference to the device structure
 * @ep_num:	Endpoint that generated the interrupt
 */
static void pch_udc_svc_data_in(struct pch_udc_dev *dev, int ep_num)
{
	u32	epsts;
	struct pch_udc_ep	*ep;
	ep = &dev->ep[2*ep_num];
	epsts = ep->epsts;
	ep->epsts = 0;

	PCH_UDC_DEBUG("%s: enter  ep%d%s status = 0x%08x", __func__, ep->num,
					 (ep->in ? "in" : "out"), epsts);

	if ((epsts & ((1 << UDC_EPSTS_IN) | (1 << UDC_EPSTS_BNA)  |
						 (1 << UDC_EPSTS_HE) |
			 (1 << UDC_EPSTS_TDC) | (1 << UDC_EPSTS_RCS) |
						 (1 << UDC_EPSTS_TXEMPTY) |
			 (1 << UDC_EPSTS_RSS) |
			 (1 << UDC_EPSTS_XFERDONE))) == 0) {
		PCH_UDC_DEBUG("%s: Non interrupt request ep%din status %x",
				__func__, ep->num, epsts);
		return;
	}
	if ((epsts & (1 << UDC_EPSTS_BNA))) { /* Just log it */
		PCH_UDC_DEBUG("%s: BNA on ep%din occured", __func__, ep->num);
		return;
	}
	if (epsts & (1 << UDC_EPSTS_HE)) {
		PCH_UDC_DEBUG("%s: Host Error on ep%din occured",
				__func__, ep->num);
		return;
	}
	if (epsts & (1 << UDC_EPSTS_RSS)) {
		PCH_UDC_DEBUG("%s: RSS", __func__);
		pch_udc_ep_set_stall(ep->regs);
		pch_udc_enable_ep_interrupts(ep->dev->regs,
			1 << (ep->in ? ep->num : ep->num + UDC_EPINT_OUT_EP0));
	}
	if (epsts & (1 << UDC_EPSTS_RCS)) {
		PCH_UDC_DEBUG("%s: RCS  prot_stall = %d",
				__func__, dev->prot_stall);
		if (dev->prot_stall == 0) {
			pch_udc_ep_clear_stall(ep->regs);
		} else {
			pch_udc_ep_set_stall(ep->regs);
			pch_udc_enable_ep_interrupts(ep->dev->regs,
				1 << (ep->in ? ep->num : ep->num +
							 UDC_EPINT_OUT_EP0));
		}
	}
	if (epsts & (1 << UDC_EPSTS_TDC)) { /* DMA completed */
		pch_udc_complete_transfer(ep);
	}
	/* On IN interrupt, provide data if we have any */
	if ((epsts & (1 << UDC_EPSTS_IN)) &&
		 ((epsts & (1 << UDC_EPSTS_RSS)) == 0) &&
		 ((epsts & (1 << UDC_EPSTS_TDC)) == 0) &&
		 ((epsts & (1 << UDC_EPSTS_TXEMPTY)) == 0)) {
		pch_udc_start_next_txrequest(ep);
	}
	PCH_UDC_DEBUG("%s: ep ctrl = 0x%x", __func__,
					 ioread32(&ep->regs->epctl));
}

/**
 * pch_udc_svc_data_out - Handles interrupts from OUT endpoint
 * @dev:	Reference to the device structure
 * @ep_num:	Endpoint that generated the interrupt
 */
static void pch_udc_svc_data_out(struct pch_udc_dev *dev, int ep_num)
{
	u32			epsts;
	struct pch_udc_ep		*ep;
	struct pch_udc_request		*req = NULL;

	ep = &dev->ep[2*ep_num + 1];
	epsts = ep->epsts;
	ep->epsts = 0;

	PCH_UDC_DEBUG("%s: enter  ep%d%s status = 0x%08x", __func__, ep->num,
					 (ep->in ? "in" : "out"), epsts);
	if (epsts & (1 << UDC_EPSTS_BNA)) { /* Just log it; only in DMA mode */
		if (!list_empty(&ep->queue)) {
			/* next request */
			req = list_entry(ep->queue.next, struct pch_udc_request,
									 queue);
			PCH_UDC_DEBUG("%s: BNA on ep%dout occured",
					__func__, ep->num);
			if ((req->td_data_last->status & PCH_UDC_BUFF_STS) !=
							 PCH_UDC_BS_DMA_DONE) {
				if (req->dma_going == 0)
					pch_udc_start_rxrequest(ep, req);

				return;
			}
		}
	}
	if (epsts & (1 << UDC_EPSTS_HE)) {  /* Host error - Just log it */
		PCH_UDC_DEBUG("%s: Host Error on ep%dout occured",
				__func__, ep->num);
		return;
	}
	if (epsts & (1 << UDC_EPSTS_RSS)) {
		PCH_UDC_DEBUG("%s: RSS", __func__);
		pch_udc_ep_set_stall(ep->regs);
		pch_udc_enable_ep_interrupts(ep->dev->regs,
			1 << (ep->in ? ep->num : ep->num + UDC_EPINT_OUT_EP0));
	}
	if (epsts & (1 << UDC_EPSTS_RCS)) {
		PCH_UDC_DEBUG("%s: RCS  prot_stall = %d", __func__,
							 dev->prot_stall);
		if (dev->prot_stall == 0) {
			pch_udc_ep_clear_stall(ep->regs);
		} else {
			pch_udc_ep_set_stall(ep->regs);
			pch_udc_enable_ep_interrupts(ep->dev->regs,
				1 << (ep->in ? ep->num : ep->num +
							 UDC_EPINT_OUT_EP0));
		}
	}
	if (((epsts & UDC_EPSTS_OUT_MASK) >> UDC_EPSTS_OUT_OFS) ==
						 UDC_EPSTS_OUT_DATA) {
		if (ep->dev->prot_stall == 1) {
			pch_udc_ep_set_stall(ep->regs);
			pch_udc_enable_ep_interrupts(ep->dev->regs,
				1 << (ep->in ? ep->num : ep->num +
							 UDC_EPINT_OUT_EP0));
		} else {
			pch_udc_complete_receiver(ep);
		}
	}

	if (list_empty(&ep->queue)) {
		/* enable DMA */
		pch_udc_set_dma(dev->regs, DMA_DIR_RX);
	}
}

/**
 * pch_udc_svc_control_in - Handle Control IN endpoint interrupts
 * @dev:	Reference to the device structure
 */
static void pch_udc_svc_control_in(struct pch_udc_dev *dev)
{
	u32	epsts;
	struct pch_udc_ep	*ep;

	ep = &dev->ep[UDC_EP0IN_IDX];
	epsts = ep->epsts;
	ep->epsts = 0;

	PCH_UDC_DEBUG("%s: enter  status 0x%x", __func__, epsts);
	if ((epsts & ((1 << UDC_EPSTS_IN) | (1 << UDC_EPSTS_BNA)  |
						 (1 << UDC_EPSTS_HE) |
			 (1 << UDC_EPSTS_TDC) | (1 << UDC_EPSTS_RCS) |
						 (1 << UDC_EPSTS_TXEMPTY) |
			 (1 << UDC_EPSTS_XFERDONE))) == 0) {
		PCH_UDC_DEBUG("%s: Non interrupt request ep%din status %x",
					__func__, ep->num, epsts);
		return;
	}
	if ((epsts & (1 << UDC_EPSTS_BNA))) { /* Just log it */
		PCH_UDC_DEBUG("%s: BNA on ep%din occured", __func__, ep->num);
		return;
	}
	if (epsts & (1 << UDC_EPSTS_HE)) {
		PCH_UDC_DEBUG("%s: Host Error on ep%din occured",
					__func__, ep->num);
		return;
	}
	if (epsts & (1 << UDC_EPSTS_TXEMPTY)) { /* Tx empty */
		PCH_UDC_DEBUG("%s: TXEMPTY", __func__);
	}
	if ((epsts & (1 << UDC_EPSTS_TDC)) && (!dev->stall)) {
		/* DMA completed */
		PCH_UDC_DEBUG("%s: TDC on ep%din", __func__, ep->num);
		pch_udc_complete_transfer(ep);
	}
	/* On IN interrupt, provide data if we have any */
	if ((epsts & (1 << UDC_EPSTS_IN)) &&
		 ((epsts & (1 << UDC_EPSTS_TDC)) == 0) &&
		 ((epsts & (1 << UDC_EPSTS_TXEMPTY)) == 0))	{
		pch_udc_start_next_txrequest(ep);
	}
}

/**
 * pch_udc_svc_control_out - Routine that handle Control
 *					OUT endpoint interrupts
 * @dev:	Reference to the device structure
 */
static void pch_udc_svc_control_out(struct pch_udc_dev *dev)
{
	u32	stat;
	int setup_supported;
	struct pch_udc_ep	*ep;

	PCH_UDC_DEBUG("%s: enter", __func__);
	ep = &dev->ep[UDC_EP0OUT_IDX];
	stat = ep->epsts;
	ep->epsts = 0;

	if (stat & (1 << UDC_EPSTS_BNA)) {
		PCH_UDC_DEBUG("%s: EP0: BNA", __func__);
		/* When we get a request, we will populate the descriptors. */
		/* Anything else to do? */
	}
	/* If setup data */
	if (((stat & UDC_EPSTS_OUT_MASK) >> UDC_EPSTS_OUT_OFS) ==
							 UDC_EPSTS_OUT_SETUP) {
		dev->stall = 0;
		dev->ep[UDC_EP0IN_IDX].halted = 0;
		dev->ep[UDC_EP0OUT_IDX].halted = 0;
		/* In data not ready */
		pch_udc_ep_set_nak(dev->ep[UDC_EP0IN_IDX].regs);
		setup_data.data[0] = ep->td_stp->data12;
		setup_data.data[1] = ep->td_stp->data34;
		PCH_UDC_DEBUG("%s: EP0 setup data12: 0x%x data34:0x%x",
			__func__, ep->td_stp->data12, ep->td_stp->data34);
		pch_udc_init_setup_buff(ep->td_stp);
		pch_udc_clear_dma(dev->regs, DMA_DIR_TX);
		pch_udc_ep_fifo_flush(dev->ep[UDC_EP0IN_IDX].regs,
						 dev->ep[UDC_EP0IN_IDX].in);
		if ((setup_data.request.bRequestType & USB_DIR_IN) != 0) {
			dev->gadget.ep0 = &dev->ep[UDC_EP0IN_IDX].ep;
		} else { /*	OUT */
			dev->gadget.ep0 = &ep->ep;
		}
		PCH_UDC_DEBUG("%s: EP0 setup data: 0x%x 0x%x", __func__,
				setup_data.data[0], setup_data.data[1]);
		spin_unlock(&dev->lock);
		/* Mass storage Reset */
		if ((setup_data.data[0] == 0x0000ff21) && (setup_data.data[1] ==
								 0x00000000)) {
			dev->prot_stall = 0;
			PCH_UDC_DEBUG("%s: Mass storage reset  prot_stall = %d",
					__func__, dev->prot_stall);
		}
		/* call gadget with setup data received */
		setup_supported = dev->driver->setup(&dev->gadget,
							 &setup_data.request);
		spin_lock(&dev->lock);

		/* ep0 in returns data on IN phase */
		if (setup_supported >= 0 && setup_supported <
						 UDC_EP0IN_MAX_PKT_SIZE) {
			pch_udc_ep_clear_nak(dev->ep[UDC_EP0IN_IDX].regs);
			/* Gadget would have queued a request when
							 we called the setup */
			pch_udc_set_dma(dev->regs, DMA_DIR_RX);
			pch_udc_ep_clear_nak(ep->regs);
		} else if (setup_supported < 0) {
			/* if unsupported request, then stall */
			PCH_UDC_DEBUG("EP0 setup unsupported: ep0_set_stall");
			pch_udc_ep_set_stall(dev->ep[UDC_EP0IN_IDX].regs);
			pch_udc_enable_ep_interrupts(ep->dev->regs,
				1 << (ep->in ? ep->num : ep->num +
							 UDC_EPINT_OUT_EP0));
			dev->stall = 0;
			pch_udc_set_dma(dev->regs, DMA_DIR_RX);
		} else {
			dev->waiting_zlp_ack = 1;
		}
	} else if ((((stat & UDC_EPSTS_OUT_MASK) >> UDC_EPSTS_OUT_OFS) ==
				 UDC_EPSTS_OUT_DATA) && (dev->stall == 0)) {
		if (list_empty(&ep->queue)) {
			printk(KERN_ERR MODULE_NAME ": "  "%s: ZLP", __func__);
			/* If no requests, reactivate */
			ep->td_data->status =
					(ep->td_data->status &
							 ~PCH_UDC_BUFF_STS) |
							PCH_UDC_BS_HST_RDY;
			/* Enable RDE */
			pch_udc_set_dma(dev->regs, DMA_DIR_RX);
		} else {
			/* control write */
			pch_udc_svc_data_out(dev, UDC_EP0OUT_IDX);
			/* re-program desc. pointer for possible ZLPs */
			pch_udc_ep_set_ddptr(ep->regs,
					ep->td_data_phys);
			/* Enable RDE */
			pch_udc_set_dma(dev->regs, DMA_DIR_RX);
		}
	}
	pch_udc_ep_set_rrdy(ep->regs);
}


/**
 * pch_udc_postsvc_epinters - This function enables end point interrupts
 *				and clears NAK status
 * @dev:	Reference to the device structure
 * @ep_num:	End point number
 */
static void pch_udc_postsvc_epinters(struct pch_udc_dev *dev, int ep_num)
{
	struct pch_udc_ep	*ep;
	struct pch_udc_request *req;
	ep = &dev->ep[2*ep_num];

	if (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct pch_udc_request, queue);
		pch_udc_enable_ep_interrupts(ep->dev->regs,
			1 << (ep->in ? ep->num : ep->num + UDC_EPINT_OUT_EP0));
		pch_udc_ep_clear_nak(ep->regs);
	}
}

/**
 * pch_udc_read_all_epstatus - This function read all endpoint status
 * @dev:	Reference to the device structure
 * @ep_intr:	Status of endpoint interrupt
 */
static void pch_udc_read_all_epstatus(struct pch_udc_dev *dev, u32 ep_intr)
{
	int i;
	struct pch_udc_ep	*ep;

	for (i = 0; i < PCH_UDC_USED_EP_NUM; i++) {
		/* IN */
		if (ep_intr & (0x1 << i)) {
			ep = &dev->ep[2*i];
			ep->epsts = pch_udc_read_ep_status(ep->regs);
			pch_udc_clear_ep_status(ep->regs, ep->epsts);
		}
		/* OUT */
		if (ep_intr & (0x10000 << i)) {
			ep = &dev->ep[2*i+1];
			ep->epsts = pch_udc_read_ep_status(ep->regs);
			pch_udc_clear_ep_status(ep->regs, ep->epsts);
		}
	}
	return;
}

/**
 * pch_udc_activate_control_ep - This function enables the control endpoints
 *					for traffic after a reset
 * @dev:	Reference to the device structure
 */
void pch_udc_activate_control_ep(struct pch_udc_dev *dev)
{
	struct pch_udc_ep	*ep;
	u32 val;

	PCH_UDC_DEBUG("%s: enter", __func__);
	/* Setup IN endpoint */
	ep = &dev->ep[UDC_EP0IN_IDX];

	/* Flush TX fifo */
	pch_udc_clear_ep_control(ep->regs);
	pch_udc_ep_fifo_flush(ep->regs, ep->in);

	/* Set buffer size (tx fifo entries) of EP0_IN  */
	pch_udc_ep_set_bufsz(ep->regs, UDC_EP0IN_BUFF_SIZE, ep->in);

	/* Set max packet size of EP0_IN */
	pch_udc_ep_set_maxpkt(ep->regs, UDC_EP0IN_MAX_PKT_SIZE);

	/* Initialize the IN EP Descriptor */
	ep->td_data      = NULL;
	ep->td_stp       = NULL;
	ep->td_data_phys = 0;
	ep->td_stp_phys  = 0;

	/* Setup OUT endpoint */
	ep = &dev->ep[UDC_EP0OUT_IDX];

	/* Flush RX fifo */
	pch_udc_clear_ep_control(ep->regs);
	pch_udc_ep_fifo_flush(ep->regs, ep->in);

	/* Set buffer size (rx fifo entries) of EP0_OUT  */
	pch_udc_ep_set_bufsz(ep->regs, UDC_EP0OUT_BUFF_SIZE, ep->in);

	/* Set max packet size of EP0_OUT */
	pch_udc_ep_set_maxpkt(ep->regs, UDC_EP0OUT_MAX_PKT_SIZE);

	/* Set max packet size of EP0 OUT UDC CSR */
	val = UDC_EP0OUT_MAX_PKT_SIZE << UDC_CSR_NE_MAX_PKT_OFS;
	pch_udc_write_csr(val, (u32) (&dev->csr->ne[UDC_EP0OUT_IDX]));

	/* Initialize the SETUP buffer */
	pch_udc_init_setup_buff(ep->td_stp);

	/* Write dma desc address */
	pch_udc_ep_set_subptr(ep->regs, ep->td_stp_phys);

	/* Write Setup desc address */
	pch_udc_ep_set_ddptr(ep->regs, ep->td_data_phys);

	/* Initialize dma descriptor */
	ep->td_data->status  = PCH_UDC_DMA_LAST;
	ep->td_data->dataptr = dma_addr;
	ep->td_data->next    = ep->td_data_phys;

	/* Clear NAK */
	pch_udc_ep_clear_nak(ep->regs);
}


/**
 * pch_udc_svc_ur_interrupt - This function handles a USB reset interrupt
 * @dev:	Reference to driver structure
 */
void pch_udc_svc_ur_interrupt(struct pch_udc_dev *dev)
{
	struct pch_udc_ep	*ep;
	int i;

	PCH_UDC_DEBUG("%s: enter", __func__);
	pch_udc_clear_dma(dev->regs, DMA_DIR_TX);
	pch_udc_clear_dma(dev->regs, DMA_DIR_RX);
	/* Mask all endpoint interrupts */
	pch_udc_disable_ep_interrupts(dev->regs, UDC_EPINT_MSK_DISABLE_ALL);
	/* clear all endpoint interrupts */
	pch_udc_write_ep_interrupts(dev->regs, UDC_EPINT_MSK_DISABLE_ALL);

	for (i = 0; i < PCH_UDC_EP_NUM; i++) {
		ep = &dev->ep[i];
		pch_udc_clear_ep_status(ep->regs, 0x1F0006F0);
		pch_udc_clear_ep_control(ep->regs);
		pch_udc_ep_set_ddptr(ep->regs, 0);
		pch_udc_write_csr(0x00, (u32) (&dev->csr->ne[i]));
	}

	dev->stall = 0;
	dev->prot_stall = 0;
	dev->waiting_zlp_ack = 0;
	dev->set_cfg_not_acked = 0;

	/* disable ep to empty req queue. Skip the control EP's */
	for (i = 0; i < (PCH_UDC_USED_EP_NUM*2); i++) {
		ep = &dev->ep[i];
		/* Set NAK */
		pch_udc_ep_set_nak(ep->regs);
		/* Flush fifo */
		pch_udc_ep_fifo_flush(ep->regs , ep->in);
		/* Complete request queue */
		empty_req_queue(ep);
	}
	if (dev->driver && dev->driver->disconnect)
		dev->driver->disconnect(&dev->gadget);
}

/**
 * pch_udc_svc_enum_interrupt - This function handles a USB speed enumeration
 *				done interrupt
 * @dev:	Reference to driver structure
 */
void
pch_udc_svc_enum_interrupt(struct pch_udc_dev *dev)
{
	u32 dev_stat, dev_speed;
	u32 speed = USB_SPEED_FULL;

	PCH_UDC_DEBUG("%s: enter", __func__);
	dev_stat = pch_udc_read_device_status(dev->regs);
	dev_speed = (dev_stat & UDC_DEVSTS_ENUM_SPEED_MASK) >>
						 UDC_DEVSTS_ENUM_SPEED_OFS;

	PCH_UDC_DEBUG("%s: dev_speed = 0x%08x", __func__, dev_speed);

	if (dev_speed == UDC_DEVSTS_ENUM_SPEED_HIGH) {
		PCH_UDC_DEBUG("HighSpeed");
		speed = USB_SPEED_HIGH;
	} else if (dev_speed == UDC_DEVSTS_ENUM_SPEED_FULL) {
		PCH_UDC_DEBUG("FullSpeed");
		speed = USB_SPEED_FULL;
	} else if (dev_speed == UDC_DEVSTS_ENUM_SPEED_LOW) {
		PCH_UDC_DEBUG("LowSpeed?");
		speed = USB_SPEED_LOW;
	} else {
		PCH_UDC_DEBUG("FullSpeed?");
	}
	dev->gadget.speed = speed;

	pch_udc_activate_control_ep(dev);

	/* enable ep0 interrupts */
	pch_udc_enable_ep_interrupts(dev->regs, 1 << UDC_EPINT_IN_EP0 |
							1 << UDC_EPINT_OUT_EP0);
	/* enable DMA */
	pch_udc_set_dma(dev->regs, DMA_DIR_TX);
	pch_udc_set_dma(dev->regs, DMA_DIR_RX);
	pch_udc_ep_set_rrdy(dev->ep[UDC_EP0OUT_IDX].regs);


	PCH_UDC_DEBUG("%s: EP mask set to %x", __func__,
				 ioread32(&dev->regs->epirqmsk));
}

/**
 * pch_udc_svc_intf_interrupt - This function handles a set interface interrupt
 * @dev:	Reference to driver structure
 */
void
pch_udc_svc_intf_interrupt(struct pch_udc_dev *dev)
{
	u32 reg, dev_stat = 0;
	int i, ret;

	PCH_UDC_DEBUG("%s: enter", __func__);
	dev_stat = pch_udc_read_device_status(dev->regs);
	dev->cfg_data.cur_intf = (dev_stat & UDC_DEVSTS_INTF_MASK) >>
							 UDC_DEVSTS_INTF_OFS;
	dev->cfg_data.cur_alt = (dev_stat & UDC_DEVSTS_ALT_MASK) >>
							 UDC_DEVSTS_ALT_OFS;
	PCH_UDC_DEBUG("DVSTATUS=%08x, cfg=%d, intf=%d, alt=%d", dev_stat,
			(dev_stat & UDC_CSR_NE_CFG_MASK) >> UDC_CSR_NE_CFG_OFS,
			dev->cfg_data.cur_intf, dev->cfg_data.cur_alt);

	dev->set_cfg_not_acked = 1;

	/* Construct the usb request for gadget driver and inform it */
	memset(&setup_data, 0 , sizeof setup_data);
	setup_data.request.bRequest = USB_REQ_SET_INTERFACE;
	setup_data.request.bRequestType = USB_RECIP_INTERFACE;
	setup_data.request.wValue = cpu_to_le16(dev->cfg_data.cur_alt);
	setup_data.request.wIndex = cpu_to_le16(dev->cfg_data.cur_intf);

	/* programm the Endpoint Cfg registers */
	for (i = 0; i < PCH_UDC_USED_EP_NUM * 2; i++) {
		if (i == 1) { /* Only one end point cfg register */
			reg = pch_udc_read_csr((u32) (&dev->csr->ne[i]));
			reg = (reg & ~UDC_CSR_NE_INTF_MASK) |
			 (dev->cfg_data.cur_intf << UDC_CSR_NE_INTF_OFS);
			reg = (reg & ~UDC_CSR_NE_ALT_MASK) |
			 (dev->cfg_data.cur_alt << UDC_CSR_NE_ALT_OFS);
			pch_udc_write_csr(reg, (u32) (&dev->csr->ne[i]));
		}
		/* clear stall bits */
		pch_udc_ep_clear_stall(dev->ep[i].regs);
		dev->ep[i].halted = 0;
	}
	dev->stall = 0;
	spin_unlock(&dev->lock);
	ret = dev->driver->setup(&dev->gadget, &setup_data.request);
	spin_lock(&dev->lock);
}

/**
 * pch_udc_svc_cfg_interrupt - This function handles a set configuration
 *				interrupt
 * @dev:	Reference to driver structure
 */
void
pch_udc_svc_cfg_interrupt(struct pch_udc_dev *dev)
{
	int i, ret;
	u32 reg, dev_stat = 0;

	PCH_UDC_DEBUG("%s: enter", __func__);
	dev_stat = pch_udc_read_device_status(dev->regs);
	PCH_UDC_DEBUG("DVSTATUS=%08x, cfg=%d, intf=%d, alt=%d", dev_stat,
		(dev_stat & UDC_DEVSTS_CFG_MASK) >> UDC_DEVSTS_CFG_OFS,
		(dev_stat & UDC_DEVSTS_INTF_MASK) >> UDC_DEVSTS_INTF_OFS,
		(dev_stat & UDC_DEVSTS_ALT_MASK) >> UDC_DEVSTS_ALT_OFS);

	dev->set_cfg_not_acked = 1;

	dev->cfg_data.cur_cfg = (dev_stat & UDC_DEVSTS_CFG_MASK) >>
							 UDC_DEVSTS_CFG_OFS;
	/* make usb request for gadget driver */
	memset(&setup_data, 0 , sizeof setup_data);
	setup_data.request.bRequest = USB_REQ_SET_CONFIGURATION;
	setup_data.request.wValue = cpu_to_le16(dev->cfg_data.cur_cfg);

	/* program the NE registers */
	for (i = 0; i < PCH_UDC_USED_EP_NUM * 2; i++) {
		if (i == 1) {
			reg = pch_udc_read_csr((u32) (&dev->csr->ne[i]));
			reg = (reg & ~UDC_CSR_NE_CFG_MASK) |
				 (dev->cfg_data.cur_cfg << UDC_CSR_NE_CFG_OFS);
			pch_udc_write_csr(reg, (u32) (&dev->csr->ne[i]));
		}
		/* clear stall bits */
		pch_udc_ep_clear_stall(dev->ep[i].regs);
		dev->ep[i].halted = 0;
	}
	dev->stall = 0;

	/* call gadget zero with setup data received */
	spin_unlock(&dev->lock);
	ret = dev->driver->setup(&dev->gadget, &setup_data.request);
	spin_lock(&dev->lock);
}

/**
 * pch_udc_dev_isr - This function services device interrupts
 *			by invoking appropriate routines.
 * @dev:	Reference to the device structure
 * @dev_intr:	The Device interrupt status.
 */
void pch_udc_dev_isr(struct pch_udc_dev *dev, u32 dev_intr)
{
	/* USB Reset Interrupt */
	if (dev_intr & (1 << UDC_DEVINT_UR))
		pch_udc_svc_ur_interrupt(dev);

	/* Enumeration Done Interrupt */
	if (dev_intr & (1 << UDC_DEVINT_ENUM))
		pch_udc_svc_enum_interrupt(dev);

	/* Set Interface Interrupt */
	if (dev_intr & (1 << UDC_DEVINT_SI))
		pch_udc_svc_intf_interrupt(dev);

	/* Set Config Interrupt */
	if (dev_intr & (1 << UDC_DEVINT_SC))
		pch_udc_svc_cfg_interrupt(dev);

	/* USB Suspend interrupt */
	if (dev_intr & (1 << UDC_DEVINT_US))
		PCH_UDC_DEBUG("USB_SUSPEND");

	/* Clear the SOF interrupt, if enabled */
	if (dev_intr & (1 << UDC_DEVINT_SOF))
		PCH_UDC_DEBUG("SOF");

	/* ES interrupt, IDLE > 3ms on the USB */
	if (dev_intr & (1 << UDC_DEVINT_ES))
		PCH_UDC_DEBUG("ES");

	/* RWKP interrupt */
	if (dev_intr & (1 << UDC_DEVINT_RWKP))
		PCH_UDC_DEBUG("RWKP");

}

/**
 * pch_udc_isr - This function handles interrupts from the PCH USB Device
 * @irq:	Interrupt request number
 * @dev:	Reference to the device structure
 */
irqreturn_t pch_udc_isr(int irq, void *pdev)
{
	struct pch_udc_dev *dev;
	u32 dev_intr, ep_intr;
	int i;

	PCH_UDC_DEBUG("%s: enter", __func__);
	dev = (struct pch_udc_dev *) pdev;
	dev_intr = pch_udc_read_device_interrupts(dev->regs);
	ep_intr = pch_udc_read_ep_interrupts(dev->regs);

	if (dev_intr != 0) {
		/* Clear device interrupts */
		pch_udc_write_device_interrupts(dev->regs, dev_intr);
	}
	if (ep_intr != 0) {
		/* Clear ep interrupts */
		pch_udc_write_ep_interrupts(dev->regs, ep_intr);
	}
	if ((dev_intr == 0) && (ep_intr == 0)) {
		PCH_UDC_DEBUG("%s: exit IRQ_NONE", __func__);
		return IRQ_NONE;
	}
	spin_lock(&dev->lock);

	if (dev_intr != 0) {
		PCH_UDC_DEBUG("%s: device intr 0x%x", __func__, dev_intr);
		pch_udc_dev_isr(dev, dev_intr);
	}

	if (ep_intr != 0) {
		PCH_UDC_DEBUG("%s: ep intr 0x%x", __func__, ep_intr);
		pch_udc_read_all_epstatus(dev, ep_intr);

		/* Process Control In interrupts, if present */
		if (ep_intr & (1 << UDC_EPINT_IN_EP0)) {
			pch_udc_svc_control_in(dev);
			pch_udc_postsvc_epinters(dev, 0);
		}
		/* Process Control Out interrupts, if present */
		if (ep_intr & (1 << UDC_EPINT_OUT_EP0))
			pch_udc_svc_control_out(dev);

		/* Process data in end point interrupts */
		for (i = 1; i < PCH_UDC_USED_EP_NUM; i++) {
			if (ep_intr & (1 <<  i)) {
				pch_udc_svc_data_in(dev, i);
				pch_udc_postsvc_epinters(dev, i);
			}
		}
		/* Process data out end point interrupts */
		for (i = UDC_EPINT_OUT_EP1; i < (UDC_EPINT_OUT_EP0 +
						 PCH_UDC_USED_EP_NUM); i++) {
			if (ep_intr & (1 <<  i))
				pch_udc_svc_data_out(dev, i -
							 UDC_EPINT_OUT_EP0);
		}
	}
	spin_unlock(&dev->lock);
	return IRQ_HANDLED;
}

/**
 * pch_udc_setup_ep0 - This function enables control endpoint for traffic
 * @dev:	Reference to the device structure
 */
static void pch_udc_setup_ep0(struct pch_udc_dev *dev)
{
	PCH_UDC_DEBUG("%s: enter", __func__);
	/* enable ep0 interrupts */
	pch_udc_enable_ep_interrupts(dev->regs, 1 << UDC_EPINT_IN_EP0 |
						 1 << UDC_EPINT_OUT_EP0);

	/* enable device interrupts */
	pch_udc_enable_interrupts(dev->regs, (1 << UDC_DEVINT_UR) |
			(1 << UDC_DEVINT_US) | (1 << UDC_DEVINT_ES) |
			(1 << UDC_DEVINT_ENUM) | (1 << UDC_DEVINT_SI) |
						 (1 << UDC_DEVINT_SC));
	PCH_UDC_DEBUG("Dev intr mask set to %x  Ep intr mask set to %x",
				ioread32(&dev->regs->devirqmsk),
				ioread32(&dev->regs->epirqmsk));
}

/**
 * gadget_release - Free the gadget driver private data
 * @pdev	reference to struct pci_dev
 */
static void gadget_release(struct device *pdev)
{
	struct pch_udc_dev *dev = dev_get_drvdata(pdev);
	kfree(dev);
}

/**
 * pch_udc_pcd_reinit - This API initializes the endpoint structures
 * @dev:	Reference to the driver structure
 */
static void pch_udc_pcd_reinit(struct pch_udc_dev *dev)
{
	static const char *ep_string[] = {
		ep0_string, "ep0out", "ep1in", "ep1out",
		"ep2in", "ep2out", "ep3in", "ep3out",
		"ep4in", "ep4out", "ep5in", "ep5out",
		"ep6in", "ep6out", "ep7in", "ep7out",
		"ep8in", "ep8out", "ep9in", "ep9out",
		"ep10in", "ep10out", "ep11in", "ep11out",
		"ep12in", "ep12out", "ep13in", "ep13out",
		"ep14in", "ep14out", "ep15in", "ep15out",
	};
	int i;

	PCH_UDC_DEBUG("%s: enter", __func__);
	dev->gadget.speed = USB_SPEED_UNKNOWN;
	INIT_LIST_HEAD(&dev->gadget.ep_list);

	/* Initialize the endpoints structures */
	for (i = 0; i < PCH_UDC_EP_NUM; i++) {
		struct pch_udc_ep *ep = &dev->ep[i];
		memset(ep, 0, sizeof(*ep));

		ep->desc = NULL;
		ep->dev = dev;
		ep->halted = 1;
		ep->num = i / 2;
		ep->in = ((i & 1) == 0) ? 1 : 0;

		ep->ep.name = ep_string[i];
		ep->ep.ops = &pch_udc_ep_ops;
		if (ep->in)
			ep->regs = (struct pch_udc_ep_regs *)\
				 ((int)dev->ep_regs + ep->num * UDC_EP_REG_OFS);
		else
			ep->regs = (struct pch_udc_ep_regs *)\
				 ((int)dev->ep_regs + \
				(UDC_EPINT_OUT_EP0 + ep->num) * UDC_EP_REG_OFS);

		ep->dma = &ep->regs->epctl;
		/* need to set ep->ep.maxpacket and set Default Configuration?*/
		ep->ep.maxpacket = UDC_BULK_MAX_PKT_SIZE;
		list_add_tail(&ep->ep.ep_list, &dev->gadget.ep_list);
		INIT_LIST_HEAD(&ep->queue);
	}
	dev->ep[UDC_EP0IN_IDX].ep.maxpacket = UDC_EP0IN_MAX_PKT_SIZE;
	dev->ep[UDC_EP0OUT_IDX].ep.maxpacket = UDC_EP0OUT_MAX_PKT_SIZE;

	dma_addr = pci_map_single(dev->pdev, ep0out_buf,
						 256, PCI_DMA_FROMDEVICE);

	/* remove ep0 in and out from the list.  They have own pointer */
	list_del_init(&dev->ep[UDC_EP0IN_IDX].ep.ep_list);
	list_del_init(&dev->ep[UDC_EP0OUT_IDX].ep.ep_list);

	dev->gadget.ep0 = &dev->ep[UDC_EP0IN_IDX].ep;
	INIT_LIST_HEAD(&dev->gadget.ep0->ep_list);
}

/**
 * pch_udc_pcd_init - This API initializes the driver structure
 * @dev:	Reference to the driver structure
 * Return	0: Success
 */
int pch_udc_pcd_init(struct pch_udc_dev *dev)
{
	PCH_UDC_DEBUG("%s: enter", __func__);
	/* udc csr registers base */
	dev->csr = dev->virt_addr + UDC_CSR_ADDR;
	/* dev registers base */
	dev->regs = dev->virt_addr + UDC_DEVCFG_ADDR;
	/* ep registers base */
	dev->ep_regs = dev->virt_addr + UDC_EPIN_REGS_ADDR;

	/* init registers, interrupts, ... */
	pch_udc_init(dev->regs);

	pch_udc_pcd_reinit(dev);
	return 0;
}

/**
 * init_dma_pools - create dma pools during initialization
 * @pdev:	reference to struct pci_dev
 */
static int init_dma_pools(struct pch_udc_dev *dev)
{
	struct pch_udc_stp_dma_desc	*td_stp;
	struct pch_udc_data_dma_desc	*td_data;

	PCH_UDC_DEBUG("%s: enter", __func__);
	/* DMA setup */
	dev->data_requests = pci_pool_create("data_requests", dev->pdev,
		sizeof(struct pch_udc_data_dma_desc), 0, 0);
	if (dev->data_requests == NULL) {
		printk(KERN_ERR MODULE_NAME ": %s: can't get request"
						" data pool", __func__);
		return -ENOMEM;
	}

	/* dma desc for setup data */
	dev->stp_requests = pci_pool_create("setup requests", dev->pdev,
		sizeof(struct pch_udc_stp_dma_desc), 0, 0);
	if (dev->stp_requests == NULL) {
		printk(KERN_ERR MODULE_NAME ": %s: can't get setup"
						" request pool", __func__);
		return -ENOMEM;
	}
	/* setup */
	td_stp = pci_pool_alloc(dev->stp_requests, GFP_KERNEL,
				&dev->ep[UDC_EP0OUT_IDX].td_stp_phys);
	if (td_stp == NULL) {
		printk(KERN_ERR MODULE_NAME ": %s: can't allocate setup"
						" dma descriptor", __func__);
		return -ENOMEM;
	}
	dev->ep[UDC_EP0OUT_IDX].td_stp = td_stp;

	/* data: 0 packets !? */
	td_data = pci_pool_alloc(dev->data_requests, GFP_KERNEL,
				&dev->ep[UDC_EP0OUT_IDX].td_data_phys);
	if (td_data == NULL) {
		printk(KERN_ERR MODULE_NAME ": %s: can't allocate data dma"
						" descriptor", __func__);
		return -ENOMEM;
	}
	dev->ep[UDC_EP0OUT_IDX].td_data = td_data;
	dev->ep[UDC_EP0IN_IDX].td_stp = NULL;
	dev->ep[UDC_EP0IN_IDX].td_stp_phys = 0;
	dev->ep[UDC_EP0IN_IDX].td_data = NULL;
	dev->ep[UDC_EP0IN_IDX].td_data_phys = 0;
	return 0;
}

int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	struct pch_udc_dev	*dev = pch_udc;
	int			retval;

	PCH_UDC_DEBUG("%s: enter", __func__);
	if ((driver == NULL) || (driver->speed == USB_SPEED_UNKNOWN) ||
		(driver->bind == NULL) || (driver->setup == NULL) ||
		(driver->unbind == NULL) || (driver->disconnect == NULL)) {
		printk(KERN_ERR MODULE_NAME ": %s: invalid driver parameter",
								__func__);
		return -EINVAL;
	}

	if (dev == NULL)
		return -ENODEV;

	if (dev->driver != NULL) {
		printk(KERN_ERR MODULE_NAME ": %s: already bound", __func__);
		return -EBUSY;
	}
	driver->driver.bus = NULL;
	dev->driver = driver;
	dev->gadget.dev.driver = &driver->driver;

	/* Invoke the bind routine of the gadget driver */
	retval = driver->bind(&dev->gadget);

	if (retval != 0) {
		printk(KERN_ERR MODULE_NAME ": %s: binding to %s returning %d",
				__func__, driver->driver.name, retval);
		dev->driver = NULL;
		dev->gadget.dev.driver = NULL;
		return retval;
	}
	/* get ready for ep0 traffic */
	pch_udc_setup_ep0(dev);

	/* clear SD */
	pch_udc_clear_disconnect(dev->regs);

	dev->connected = 1;
	return 0;
}
EXPORT_SYMBOL(usb_gadget_register_driver);

int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct pch_udc_dev	*dev = pch_udc;

	PCH_UDC_DEBUG("%s: enter", __func__);
	if (dev == NULL)
		return -ENODEV;

	if ((driver == NULL) || (driver != dev->driver)) {
		printk(KERN_ERR MODULE_NAME ": %s: invalid driver parameter",
								__func__);
		return -EINVAL;
	}

	pch_udc_disable_interrupts(dev->regs, UDC_DEVINT_MSK);

	/* Assures that there are no pending requests with this driver */
	driver->disconnect(&dev->gadget);
	driver->unbind(&dev->gadget);
	dev->gadget.dev.driver = NULL;
	dev->driver = NULL;
	dev->connected = 0;

	/* set SD */
	pch_udc_set_disconnect(dev->regs);
	return 0;
}
EXPORT_SYMBOL(usb_gadget_unregister_driver);

void pch_udc_shutdown(struct pci_dev *pdev)
{
	struct pch_udc_dev *dev = pci_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "%s: enter", __func__);
	pch_udc_disable_interrupts(dev->regs, UDC_DEVINT_MSK);
	pch_udc_disable_ep_interrupts(dev->regs, UDC_EPINT_MSK_DISABLE_ALL);

	/* disable the pullup so the host will think we're gone */
	pch_udc_set_disconnect(dev->regs);
}

void pch_udc_remove(struct pci_dev *pdev)
{
	struct pch_udc_dev	*dev = pci_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "%s: enter", __func__);
	/* gadget driver must not be registered */
	if (dev->driver != NULL)
		dev_err(&pdev->dev, "%s: gadget driver still bound!!!",
								__func__);
	/* dma pool cleanup */
	if (dev->data_requests != NULL)
		pci_pool_destroy(dev->data_requests);

	if (dev->stp_requests != NULL) {
		/* cleanup DMA desc's for ep0in */
		if (dev->ep[UDC_EP0OUT_IDX].td_stp != NULL) {
			pci_pool_free(dev->stp_requests,
				dev->ep[UDC_EP0OUT_IDX].td_stp,
				dev->ep[UDC_EP0OUT_IDX].td_stp_phys);
		}
		if (dev->ep[UDC_EP0OUT_IDX].td_data != NULL) {
			pci_pool_free(dev->stp_requests,
				dev->ep[UDC_EP0OUT_IDX].td_data,
				dev->ep[UDC_EP0OUT_IDX].td_data_phys);
		}
		pci_pool_destroy(dev->stp_requests);
	}

	pch_udc_exit(dev->regs);

	if (dev->irq_registered)
		free_irq(pdev->irq, dev);

	if (dev->virt_addr != NULL)
		iounmap(dev->virt_addr);

	if (dev->mem_region)
		release_mem_region(dev->phys_addr, pci_resource_len(pdev,
							 PCH_UDC_PCI_BAR));
	if (dev->active)
		pci_disable_device(pdev);

	if (dev->registered)
		device_unregister(&dev->gadget.dev);
	else
		kfree(dev);

	pci_set_drvdata(pdev, NULL);
}

#ifdef CONFIG_PM
int pch_udc_suspend(struct pci_dev *pdev, pm_message_t state)
{
	struct pch_udc_dev *dev = pci_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "%s: enter", __func__);
	pch_udc_disable_interrupts(dev->regs, UDC_DEVINT_MSK);
	pch_udc_disable_ep_interrupts(dev->regs, UDC_EPINT_MSK_DISABLE_ALL);

	pci_disable_device(pdev);
	pci_enable_wake(pdev, PCI_D3hot, 0);

	if (pci_save_state(pdev) != 0) {
		dev_err(&pdev->dev, "%s: could not save PCI config state",
								__func__);
		return -ENOMEM;
	}

	if (pci_set_power_state(pdev, pci_choose_state(pdev, state)) == -EIO)
		dev_dbg(&pdev->dev, "%s: does not support PM cpabilities",
								__func__);

	return 0;
}

int pch_udc_resume(struct pci_dev *pdev)
{
	int ret;

	dev_dbg(&pdev->dev, "%s: enter", __func__);
	ret = pci_set_power_state(pdev, PCI_D0);
	if (ret != 0)
		dev_dbg(&pdev->dev,
			"%s: does not support PM cpabilities", __func__);

	ret = pci_restore_state(pdev);
	if (ret != 0) {
		dev_err(&pdev->dev,
			"%s: pci_restore_state failed", __func__);
		return ret;
	}

	ret = pci_enable_device(pdev);

	if (ret != 0) {
		dev_err(&pdev->dev, "%s: pci_enable_device failed", __func__);
		return ret;
	}
	pci_enable_wake(pdev, PCI_D3hot, 0);

	return 0;
}
#else
#define pch_udc_suspend	NULL
#define pch_udc_resume	NULL
#endif /* CONFIG_PM */

int pch_udc_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	unsigned long		resource;
	unsigned long		len;
	int					retval = 0;
	struct pch_udc_dev	*dev;

	dev_dbg(&pdev->dev, "%s: enter", __func__);
	/* one udc only */
	if (pch_udc != NULL) {
		dev_err(&pdev->dev, "%s: already probed", __func__);
		return -EBUSY;
	}

	/* init */
	dev = kzalloc(sizeof(struct pch_udc_dev), GFP_KERNEL);
	if (dev == NULL) {
		dev_err(&pdev->dev, "%s: no memory for device structure",
								__func__);
		return -ENOMEM;
	}
	memset(dev, 0, sizeof(struct pch_udc_dev));
	/* pci setup */
	if (pci_enable_device(pdev) < 0) {
		kfree(dev);
		dev_err(&pdev->dev, "%s: pci_enable_device failed", __func__);
		return -ENODEV;
	}
	dev->active = 1;
	pci_set_drvdata(pdev, dev);

	/* PCI resource allocation */
	resource = pci_resource_start(pdev, 1);
	len = pci_resource_len(pdev, 1);
	dev_dbg(&pdev->dev, "%s: resource %lx, len %ld",
			__func__, resource, len);

	if (request_mem_region(resource, len, MODULE_NAME) == NULL) {
		dev_err(&pdev->dev, "%s: pci device used already", __func__);
		retval = -EBUSY;
		goto finished;
	}
	dev->phys_addr = resource;
	dev->mem_region = 1;

	dev->virt_addr = ioremap_nocache(resource, len);
	if (dev->virt_addr == NULL) {
		dev_err(&pdev->dev, "%s: device memory cannot be mapped",
								__func__);
		retval = -ENOMEM;
		goto finished;
	}
	dev_dbg(&pdev->dev, "%s: device memory mapped at %x", __func__,
						 (int)dev->virt_addr);

	if (pdev->irq == 0) {
		dev_err(&pdev->dev, "%s: irq not set", __func__);
		retval = -ENODEV;
		goto finished;
	}

	pch_udc = dev;

	/* initialize the hardware */
	if (pch_udc_pcd_init(dev) != 0)
		goto finished;

	if (request_irq(pdev->irq, pch_udc_isr, IRQF_SHARED, MODULE_NAME, dev)
			!= 0) {
		dev_err(&pdev->dev, "%s: request_irq(%d) fail", __func__,
								 pdev->irq);
		retval = -ENODEV;
		goto finished;
	}
	dev->irq = pdev->irq;
	dev->irq_registered = 1;

	pci_set_master(pdev);
	pci_try_set_mwi(pdev);

	/* device struct setup */
	spin_lock_init(&dev->lock);
	dev->pdev = pdev;
	dev->gadget.ops = &pch_udc_ops;

	retval = init_dma_pools(dev);
	if (retval != 0)
		goto finished;

	dev_set_name(&dev->gadget.dev, "gadget");
	dev->gadget.dev.parent = &pdev->dev;
	dev->gadget.dev.dma_mask = pdev->dev.dma_mask;
	dev->gadget.dev.release = gadget_release;
	dev->gadget.name = MODULE_NAME;
	dev->gadget.is_dualspeed = 1;

	retval = device_register(&dev->gadget.dev);
	if (retval != 0)
		goto finished;
	dev->registered = 1;

	/* Put the device in disconnected state till a driver is bound */
	pch_udc_set_disconnect(dev->regs);
	return 0;

finished:
	pch_udc_remove(pdev);
	return retval;
}

static const struct pci_device_id pch_udc_pcidev_id[] = {
	{
		PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_PCH1_UDC),
		.class = (PCI_CLASS_SERIAL_USB << 8) | 0xfe,
		.class_mask = 0xffffffff,
	},
	{ 0 },
};

MODULE_DEVICE_TABLE(pci, pch_udc_pcidev_id);


static struct pci_driver pch_udc_driver = {
	.name =	MODULE_NAME,
	.id_table =	pch_udc_pcidev_id,
	.probe =	pch_udc_probe,
	.remove =	pch_udc_remove,
	.suspend =	pch_udc_suspend,
	.resume =	pch_udc_resume,
	.shutdown =	pch_udc_shutdown,
};

static int __init pch_udc_pci_init(void)
{
	return pci_register_driver(&pch_udc_driver);
}
module_init(pch_udc_pci_init);

static void __exit pch_udc_pci_exit(void)
{
	pci_unregister_driver(&pch_udc_driver);
}
module_exit(pch_udc_pci_exit);
