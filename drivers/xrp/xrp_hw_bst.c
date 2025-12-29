/*
 * xrp_hw_bst: Simple xtensa/arm low-level XRP driver
 *
 * Copyright (c) 2017 Cadence Design Systems, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Alternatively you can use and distribute this file under the terms of
 * the GNU General Public License version 2 or later.
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <asm/cacheflush.h>
#include "xrp_kernel_defs.h"
#include "xrp_hw.h"
#include "xrp_hw_bst_dsp_interface.h"

#define DRIVER_NAME "xrp-hw-bst"
#define TOP_CRM_ADDR (0x30002000)
#define HIFI5_DSP_S_BASE_ADDR (0X24A00000)
#define HIFI5_NN_S_BASE_ADDR (0X24B00000)
#define HIFI5_DSP_CSR_BASE_ADDR (0X24D00000)
#define HIFI5_NN_S_UBUF_BASE_ADDR (0X27000000)

#define HIFI_DSP_GLBCTRL 				0x0


#define	HIFI_CFG_RESET 					( 1 << 0 )
#define	HIFI_CFG_NIC_M_RESET 			( 1 << 1 )
#define	HIFI_CFG_NIC_S_RESET 			( 1 << 2 )
#define	HIFI_CFG_DEBUG_RESET 			( 1 << 3 )
#define	HIFI_CFG_XNNE_RESET 			( 1 << 4 )
#define	HIFI_CFG_RUNTSTALL 				( 1 << 5 )
#define	HIFI_CFG_PRID 					( 0xFFFF << 6 )
#define	HIFI_CFG_STATVECPRSEL			( 1 << 22 )
#define	HIFI_CFG_OCD_HALTONRESET			( 1 << 23 )
#define	HIFI_CFG_CLK_PHASE_STORE		( 1 << 24 )
#define HIFI_RESET_ALL (HIFI_CFG_RESET|HIFI_CFG_NIC_M_RESET|HIFI_CFG_NIC_S_RESET|HIFI_CFG_DEBUG_RESET|HIFI_CFG_XNNE_RESET)
#define HIFI_DSP_GLBSTATE		 		0x4

#define HIFI_STATE_ERRORCORRECTED		( 1 << 8 )
#define HIFI_STATE_DEBUGMODE			( 1 << 7 )
#define HIFI_STATE_XOCDMODE				( 1 << 6 )
#define HIFI_STATE_PWAITDMODE			( 1 << 5 )
#define HIFI_STATE_IRAM_LOADSTORE		( 1 << 4 )
#define HIFI_STATE_PREFETCH_RAM_CORRECTED		( 1 << 3 )
#define HIFI_STATE_PREFETCH_RAM_UNCORRECTED		( 1 << 2 )
#define HIFI_STATE_DOUBLE_EXCEPTION		( 1 << 1 )
#define HIFI_STATE_PFATALERROR			( 1 << 0 )
#define HIFI_INTR_GLB_STATE_MASK (HIFI_STATE_ERRORCORRECTED |  HIFI_STATE_DEBUGMODE | HIFI_STATE_XOCDMODE | 	\
								 	HIFI_STATE_PWAITDMODE | HIFI_STATE_IRAM_LOADSTORE | HIFI_STATE_PREFETCH_RAM_CORRECTED | 	\
								 	HIFI_STATE_PREFETCH_RAM_UNCORRECTED | HIFI_STATE_DOUBLE_EXCEPTION | HIFI_STATE_PFATALERROR )

#define HIFI_DSP_ERRSTATE0  			0x8
#define HIFI_DSP_ERRSTATE1	 			0xc
#define HIFI_DSP_ERRSTATE2		 		0x10
#define HIFI_ALTRESET_VEC		 		0x14
#define HIFI_PARITY_EN		 			0x18
#define HIFI_PARITY_ERR_INJ		 		0x1c
#define HIFI_PARITY_INTR_CLR	 		0x20
#define HIFI_PARITY_INTR	 			0x24
#define HIFI_PARITY_INT_MASK	 		0x28
#define HIFI_PDBUG_INFO_0		 		0x2C
#define HIFI_PDBUG_INFO_1		 		0x30
#define HIFI_WWDT_STATUS		 		0x34
#define HIFI_WWDT_CTRL			 		0x38
#define HIFI_PDEBUG_EN			 		0x3C
#define HIFI_PDEBUG_DATA		 		0x40
#define HIFI_PDEBUG_INST		 		0x44
#define HIFI_PDEBUG_PC			 		0x48
#define HIFI_PDEBUG_LS0_ADDR	 		0x4C
#define HIFI_PDEBUG_LS0_DATA	 		0x50
#define HIFI_PDEBUG_LS0_STAT	 		0x54
#define HIFI_PDEBUG_LS1_ADDR	 		0x58
#define HIFI_PDEBUG_LS1_DATA	 		0x5C
#define HIFI_PDEBUG_LS1_STAT	 		0x60
enum xrp_irq_mode {
	XRP_IRQ_NONE,
	XRP_IRQ_LEVEL,
	XRP_IRQ_EDGE,
	XRP_IRQ_EDGE_SW,
	XRP_IRQ_MAX,
};
#define BST_DEBUG_REG_EN 1
struct xrp_hw_bst {
	struct xvp *xrp;
	phys_addr_t regs_phys;
	void __iomem *regs;

	/* how IRQ is used to notify the device of incoming data */
	enum xrp_irq_mode device_irq_mode;
	/*
	 * offset of device IRQ register in MMIO region (device side)
	 * bit number
	 * device IRQ#
	 */
	u32 device_irq[3];
	/* offset of devuce IRQ register in MMIO region (host side) */
	u32 device_irq_host_offset;
	/* how IRQ is used to notify the host of incoming data */
	enum xrp_irq_mode host_irq_mode;
	/*
	 * offset of IRQ register (device side)
	 * bit number
	 */
	u32 host_irq[2];
};

static inline void reg_write32(struct xrp_hw_bst *hw, unsigned addr, u32 v)
{
	if (hw->regs) {
		__raw_writel(v, hw->regs + addr);
	}
		pr_err("%s: regs_phys(0x%llx) regs(0x%llx) = 0x%x\n",
		 __func__, hw->regs_phys+addr,(uint64_t)(hw->regs + addr), __raw_readl(hw->regs + addr));
		
}

static inline u32 reg_read32(struct xrp_hw_bst *hw, unsigned addr)
{
	if (hw->regs)
		return __raw_readl(hw->regs + addr);
	else
		return 0;
}

#if 0
static void *get_hw_sync_data(void *hw_arg, size_t *sz)
{
	static const u32 irq_mode[] = {
		[XRP_IRQ_NONE] = XRP_DSP_SYNC_IRQ_MODE_NONE,
		[XRP_IRQ_LEVEL] = XRP_DSP_SYNC_IRQ_MODE_LEVEL,
		[XRP_IRQ_EDGE] = XRP_DSP_SYNC_IRQ_MODE_EDGE,
		[XRP_IRQ_EDGE_SW] = XRP_DSP_SYNC_IRQ_MODE_EDGE,
	};
	struct xrp_hw_bst *hw = hw_arg;
	struct xrp_hw_bst_sync_data *hw_sync_data =
		kmalloc(sizeof(*hw_sync_data), GFP_KERNEL);

	if (!hw_sync_data)
		return NULL;

	*hw_sync_data = (struct xrp_hw_bst_sync_data){
		.host_mmio_addr = hw->regs_phys,		
	};
	*sz = sizeof(*hw_sync_data);
	return hw_sync_data;
}
#else
static void *get_hw_sync_data(void *hw_arg, size_t *sz)
{
	static const u32 irq_mode[] = {
		[XRP_IRQ_NONE] = XRP_DSP_SYNC_IRQ_MODE_NONE,
		[XRP_IRQ_LEVEL] = XRP_DSP_SYNC_IRQ_MODE_LEVEL,
		[XRP_IRQ_EDGE] = XRP_DSP_SYNC_IRQ_MODE_EDGE,
		[XRP_IRQ_EDGE_SW] = XRP_DSP_SYNC_IRQ_MODE_EDGE,
	};
	struct xrp_hw_bst *hw = hw_arg;
	struct xrp_hw_simple_sync_data *hw_sync_data =
		kmalloc(sizeof(*hw_sync_data), GFP_KERNEL);

	pr_debug("%s: irq_mode \n", __func__);

	if (!hw_sync_data)
		return NULL;

	*hw_sync_data = (struct xrp_hw_simple_sync_data){
		.device_mmio_base = hw->regs_phys,
		.host_irq_mode = hw->host_irq_mode,
		.host_irq_offset = hw->host_irq[0],
		.host_irq_bit = hw->host_irq[1],
		.device_irq_mode = irq_mode[hw->device_irq_mode],
		.device_irq_offset = hw->device_irq[0],
		.device_irq_bit = hw->device_irq[1],
		.device_irq = hw->device_irq[2],
	};
	*sz = sizeof(*hw_sync_data);
	return hw_sync_data;
}
#endif

static void reset(void *hw_arg)
{
	reg_write32(hw_arg, HIFI_DSP_GLBCTRL, reg_read32(hw_arg,HIFI_DSP_GLBCTRL)&(~HIFI_RESET_ALL));
	udelay(1);
	reg_write32(hw_arg, HIFI_DSP_GLBCTRL, reg_read32(hw_arg,HIFI_DSP_GLBCTRL)|(HIFI_RESET_ALL));
}

static void xrp_hw_bst_halt(void *hw_arg)
{
	//reg_write32(hw_arg, HIFI_DSP_GLBCTRL, reg_read32(hw_arg,HIFI_DSP_GLBCTRL)|(HIFI_CFG_RUNTSTALL|HIFI_CFG_STATVECPRSEL));
	reg_write32(hw_arg, HIFI_DSP_GLBCTRL, reg_read32(hw_arg,HIFI_DSP_GLBCTRL)|(HIFI_CFG_RUNTSTALL));
}

#if 0
static void release(void *hw_arg)
{
	#if (BST_DEBUG_REG_EN ==1)
	reg_write32(hw_arg, HIFI_PDEBUG_EN, reg_read32(hw_arg,HIFI_PDEBUG_EN)|(0x1));
	#endif
	reg_write32(hw_arg, HIFI_DSP_GLBCTRL, reg_read32(hw_arg,HIFI_DSP_GLBCTRL)&(~HIFI_CFG_RUNTSTALL));
}
#endif

static void interrupt_mask(void *hw_arg,u32 mask_bit,u32 flag)
{
	if(flag)
	{
		reg_write32(hw_arg, HIFI_PARITY_INT_MASK, reg_read32(hw_arg,HIFI_PARITY_INT_MASK)|(mask_bit));
	}
	else
	{
		reg_write32(hw_arg, HIFI_PARITY_INT_MASK, reg_read32(hw_arg,HIFI_PARITY_INT_MASK)&(mask_bit));
	}
}

#if 0
static void send_message_box_irq(void *hw_arg)
{
	pr_debug("%s: \n", __func__);
	return;
}


static void send_irq(void *hw_arg)
{
	struct xrp_hw_bst *hw = hw_arg;
	send_message_box_irq(hw_arg);
}
#else
static void send_irq(void *hw_arg)
{
	struct xrp_hw_bst *hw = hw_arg;
	
#ifdef CONFIG_HIFI_XRP_LOG_EN
	pr_debug("%s: \n", __func__);
	pr_debug("%s: device_irq_mode= %d\n", __func__, hw->device_irq_mode);
#endif

	switch (hw->device_irq_mode) {
	case XRP_IRQ_EDGE_SW:
		reg_write32(hw, hw->device_irq_host_offset,
			    BIT(hw->device_irq[1]));
		while ((reg_read32(hw, hw->device_irq_host_offset) &
			BIT(hw->device_irq[1])))
			mb();
		break;
	case XRP_IRQ_EDGE:
		reg_write32(hw, hw->device_irq_host_offset, 0);
		wmb();
		reg_write32(hw, hw->device_irq_host_offset,
			    BIT(hw->device_irq[1]));
		break;
		/* fallthrough */
	case XRP_IRQ_LEVEL:
		wmb();
		reg_write32(hw, hw->device_irq_host_offset,
			    BIT(hw->device_irq[1]));
		break;
	default:
		break;
	}
}
#endif

static void ack_irq(void *hw_arg)
{
	struct xrp_hw_bst *hw = hw_arg;

	if (hw->host_irq_mode == XRP_IRQ_LEVEL)
		reg_write32(hw, hw->host_irq[0], 0);
}

static irqreturn_t irq_handler(int irq, void *dev_id)
{
	struct xrp_hw_bst *hw = dev_id;
	irqreturn_t ret = xrp_irq_handler(irq, hw->xrp);

	if (ret == IRQ_HANDLED)
		ack_irq(hw);

	return ret;
}
static void memcpy_tohw(void __iomem *dst, const void *src, phys_addr_t pa_dest, size_t sz)
{
	sz = ALIGN(sz, 4);
	
	if(pa_dest>=0x24a00000&&pa_dest<0x24a02000){		
		if ((((phys_addr_t)dst | (phys_addr_t)src) & 0x3) == 0) {
			for ( ; sz > 0; sz -= 4) {
				*(volatile u32 *)dst = *(u32 *)src;
				dst += 4;
				src += 4;
			}
		}	
		mb();	
	}
	else{
		memcpy_toio(dst,src,sz);
	}	
	
}
static void memset_hw(void __iomem *dst, int c, phys_addr_t pa_dest, size_t sz)
{		
	sz = ALIGN(sz, 4);
	
	if(pa_dest>=0x24a00000&&pa_dest<0x24a02000){		
		if (((phys_addr_t)dst & 0x3) == 0) {
			while (sz) {
                sz-=4;
                writel(c, dst);
                dst+=4;
        	}
		}				
	}
	else{
		memset_io(dst,c,sz);
	}	
	
}
#if defined(__XTENSA__)
static bool cacheable(void *hw_arg, unsigned long pfn, unsigned long n_pages)
{
	return true;
}

static void dma_sync_for_device(void *hw_arg,
				void *vaddr, phys_addr_t paddr,
				unsigned long sz, unsigned flags)
{
	pr_info("%s, __XTENSA__ \n", __func__);
	switch (flags) {
	case XRP_FLAG_READ:
		__flush_dcache_range((unsigned long)vaddr, sz);
		break;

	case XRP_FLAG_READ_WRITE:
		__flush_dcache_range((unsigned long)vaddr, sz);
		__invalidate_dcache_range((unsigned long)vaddr, sz);
		break;

	case XRP_FLAG_WRITE:
		__invalidate_dcache_range((unsigned long)vaddr, sz);
		break;
	}
}

static void dma_sync_for_cpu(void *hw_arg,
			     void *vaddr, phys_addr_t paddr,
			     unsigned long sz, unsigned flags)
{
	pr_info("%s, __XTENSA__ \n", __func__);
	switch (flags) {
	case XRP_FLAG_READ_WRITE:
	case XRP_FLAG_WRITE:
		__invalidate_dcache_range((unsigned long)vaddr, sz);
		break;
	}
}

#elif defined(__arm__)
static bool cacheable(void *hw_arg, unsigned long pfn, unsigned long n_pages)
{
	return true;
}

static void dma_sync_for_device(void *hw_arg,
				void *vaddr, phys_addr_t paddr,
				unsigned long sz, unsigned flags)
{
	pr_info("%s, __arm__ \n", __func__);
	switch (flags) {
	case XRP_FLAG_READ:
		__cpuc_flush_dcache_area(vaddr, sz);
		outer_clean_range(paddr, paddr + sz);
		break;

	case XRP_FLAG_WRITE:
		__cpuc_flush_dcache_area(vaddr, sz);
		outer_inv_range(paddr, paddr + sz);
		break;

	case XRP_FLAG_READ_WRITE:
		__cpuc_flush_dcache_area(vaddr, sz);
		outer_flush_range(paddr, paddr + sz);
		break;
	}
}

static void dma_sync_for_cpu(void *hw_arg,
			     void *vaddr, phys_addr_t paddr,
			     unsigned long sz, unsigned flags)
{
	pr_info("%s, __arm__ \n", __func__);
	switch (flags) {
	case XRP_FLAG_WRITE:
	case XRP_FLAG_READ_WRITE:
		__cpuc_flush_dcache_area(vaddr, sz);
		outer_inv_range(paddr, paddr + sz);
		break;
	}
}
#endif

static const struct xrp_hw_ops hw_ops = {
	.halt = xrp_hw_bst_halt,
//	.release = release,
	.reset = reset,

	.get_hw_sync_data = get_hw_sync_data,

	.send_irq = send_irq,
	.memcpy_tohw = memcpy_tohw,
	.memset_hw = memset_hw,

#if defined(__XTENSA__) || defined(__arm__)
	.cacheable = cacheable,
	.dma_sync_for_device = dma_sync_for_device,
	.dma_sync_for_cpu = dma_sync_for_cpu,
#endif
};

static long init_hw(struct platform_device *pdev, struct xrp_hw_bst *hw,
		    int mem_idx, enum xrp_init_flags *init_flags)
{
	struct resource *mem;
	int irq;
	long ret;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, mem_idx);
	if (!mem) {
		ret = -ENODEV;
		goto err;
	}
	hw->regs_phys = mem->start;
	hw->regs = devm_ioremap_resource(&pdev->dev, mem);
	pr_err("%s: regs = %pap/%p  end =%pap size =0x%llx\n", __func__, &mem->start, hw->regs, &mem->end,resource_size(mem));
#if 1
	ret = device_property_read_u32_array(&pdev->dev,
					     "device-irq",
					     hw->device_irq,
					     ARRAY_SIZE(hw->device_irq));
	if (ret == 0) {
		u32 device_irq_host_offset;

		ret = device_property_read_u32(&pdev->dev,
					       "device-irq-host-offset",
					       &device_irq_host_offset);
		if (ret == 0) {
			hw->device_irq_host_offset = device_irq_host_offset;
		} else {
			hw->device_irq_host_offset = hw->device_irq[0];
			ret = 0;
		}
	}
	if (ret == 0) {
		u32 device_irq_mode;

		ret = device_property_read_u32(&pdev->dev,
					       "device-irq-mode",
					       &device_irq_mode);
		if (device_irq_mode < XRP_IRQ_MAX)
			hw->device_irq_mode = device_irq_mode;
		else
			ret = -ENOENT;
	}
	if (ret == 0) {
		dev_err(&pdev->dev,
			"%s: device IRQ MMIO host offset = 0x%08x, offset = 0x%08x, bit = %d, device IRQ = %d, IRQ mode = %d",
			__func__, hw->device_irq_host_offset,
			hw->device_irq[0], hw->device_irq[1],
			hw->device_irq[2], hw->device_irq_mode);
	} else {
		dev_err(&pdev->dev,
			 "using polling mode on the device side\n");
	}

	ret = device_property_read_u32_array(&pdev->dev, "host-irq",
					     hw->host_irq,
					     ARRAY_SIZE(hw->host_irq));
	if (ret == 0) {
		u32 host_irq_mode;

		ret = device_property_read_u32(&pdev->dev,
					       "host-irq-mode",
					       &host_irq_mode);
		if (host_irq_mode < XRP_IRQ_MAX)
			hw->host_irq_mode = host_irq_mode;
		else
			ret = -ENOENT;
	}

	if (ret == 0 && hw->host_irq_mode != XRP_IRQ_NONE)
		irq = platform_get_irq(pdev, 0);
	else
		irq = -1;
	// interrupt_mask(hw,0xFFFF,1);
	// if (irq >= 0) {
	// 	dev_err(&pdev->dev, "%s: host IRQ = %d, ",
	// 		__func__, irq);
	// 	ret = devm_request_irq(&pdev->dev, irq, irq_handler,
	// 			       IRQF_SHARED, pdev->name, hw);
	// 	if (ret < 0) {
	// 		dev_err(&pdev->dev, "request_irq %d failed\n", irq);
	// 		goto err;
	// 	}
	// 	*init_flags |= XRP_INIT_USE_HOST_IRQ;
	// } else { 
	// 	dev_err(&pdev->dev, "using polling mode on the host side\n");
	// }
	// interrupt_mask(hw,0xFFE0,0);

	interrupt_mask(hw,0xFFFF,1);
	if (irq >= 0) {
		dev_err(&pdev->dev, "%s: host IRQ = %d, ",
			__func__, irq);
		ret = devm_request_irq(&pdev->dev, irq, irq_handler,
				       IRQF_SHARED, pdev->name, hw);
		if (ret < 0) {
			dev_err(&pdev->dev, "request_irq %d failed\n", irq);
			goto err;
		}
		*init_flags |= XRP_INIT_USE_HOST_IRQ;
	} else { 
		dev_err(&pdev->dev, "using polling mode on the host side\n");
	}
	interrupt_mask(hw,0xFFE0,0);
#endif
	ret = 0;
err:
	return ret;
}



static long init_v1(struct platform_device *pdev, struct xrp_hw_bst *hw)
{
	long ret;
	enum xrp_init_flags init_flags = 0;

	ret = init_hw(pdev, hw, 1, &init_flags);
	if (ret < 0)
		return ret;

	return xrp_init_v1(pdev, init_flags, &hw_ops, hw);
}

static long init_cma(struct platform_device *pdev, struct xrp_hw_bst *hw)
{
	long ret;
	enum xrp_init_flags init_flags = 0;

	ret = init_hw(pdev, hw, 0, &init_flags);
	if (ret < 0)
		return ret;

	return xrp_init_cma(pdev, init_flags, &hw_ops, hw);
}

#ifdef CONFIG_OF
static const struct of_device_id xrp_hw_bst_match[] = {
	{
		.compatible = "bst,xrp-hw-simple,v1",
		.data = init_v1,
	}, {
		.compatible = "bst,xrp-hw-simple,cma",
		.data = init_cma,
	}, {},
};
MODULE_DEVICE_TABLE(of, xrp_hw_bst_match);
#endif

static int xrp_hw_bst_probe(struct platform_device *pdev)
{
	struct xrp_hw_bst *hw =
		devm_kzalloc(&pdev->dev, sizeof(*hw), GFP_KERNEL);
	const struct of_device_id *match;
	long (*init)(struct platform_device *pdev, struct xrp_hw_bst *hw);
	long ret;

	if (!hw)
		return -ENOMEM;

	match = of_match_device(of_match_ptr(xrp_hw_bst_match),
				&pdev->dev);
	if (!match)
		return -ENODEV;

	init = match->data;
	ret = init(pdev, hw);
	if (IS_ERR_VALUE(ret)) {
		xrp_deinit(pdev);
		return ret;
	} else {
		hw->xrp = ERR_PTR(ret);
		return 0;
	}

}

static int xrp_hw_bst_remove(struct platform_device *pdev)
{
	return xrp_deinit(pdev);
}

static const struct dev_pm_ops xrp_hw_bst_pm_ops = {
	SET_RUNTIME_PM_OPS(xrp_runtime_suspend,
			   xrp_runtime_resume, NULL)
};

static struct platform_driver xrp_hw_bst_driver = {
	.probe   = xrp_hw_bst_probe,
	.remove  = xrp_hw_bst_remove,
	.driver  = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(xrp_hw_bst_match),
		.pm = &xrp_hw_bst_pm_ops,
	},
};

module_platform_driver(xrp_hw_bst_driver);

MODULE_AUTHOR("Max Filippov");
MODULE_DESCRIPTION("XRP: low level device driver for Xtensa Remote Processing");
MODULE_LICENSE("Dual MIT/GPL");
