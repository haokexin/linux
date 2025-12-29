/*
 * XRP: Linux device driver for Xtensa Remote Processing
 *
 * Copyright (c) 2015 - 2017 Cadence Design Systems, Inc.
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

#include <linux/version.h>
#include <linux/atomic.h>
#include <linux/acpi.h>
#include <linux/completion.h>
#include <linux/delay.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 16, 0)
#include <linux/dma-mapping.h>
#else
#include <linux/dma-direct.h>
#endif
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/hashtable.h>
#include <linux/highmem.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mman.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
#include <linux/of_reserved_mem.h>
#endif
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/sort.h>
#include <linux/kthread.h>
#include <asm/mman.h>
#include <asm/uaccess.h>
#include "xrp_cma_alloc.h"
#include "xrp_compat.h"
#include "xrp_firmware.h"
#include "xrp_hw.h"
#include "xrp_internal.h"
#include "xrp_kernel_defs.h"
#include "xrp_kernel_dsp_interface.h"
#include "xrp_private_alloc.h"
#ifdef ipc_en
#include "audio_msgbox_client.h"
#endif
#include "test_data_user.h"
#define DRIVER_NAME "xrp"
#define XRP_DEFAULT_TIMEOUT 10

#ifndef __io_virt
#define __io_virt(a) ((void __force *)(a))
#endif

static audio_msgbox_client_data_t soc_data = {0};

struct xrp_shared_allocation {
	unsigned long flags;
	unsigned long vaddr;
	unsigned long size;
	unsigned long mm;
	struct xrp_allocation *allocation;
	struct hlist_node node;
};

struct xrp_alien_mapping {
	unsigned long vaddr;
	unsigned long size;
	phys_addr_t paddr;
	struct xrp_allocation *allocation;
	struct xrp_shared_allocation *shared_allocation;
	enum {
		ALIEN_GUP,
		ALIEN_PFN_MAP,
		ALIEN_COPY,
	} type;
};

struct xrp_mapping {
	enum {
		XRP_MAPPING_NONE,
		XRP_MAPPING_NATIVE,
		XRP_MAPPING_ALIEN,
		XRP_MAPPING_KERNEL = 0x4,
	} type;
	union {
		struct {
			struct xrp_allocation *xrp_allocation;
			unsigned long vaddr;
		} native;
		struct xrp_alien_mapping alien_mapping;
	};
};

struct xvp_file {
	struct xvp *xvp;
	spinlock_t busy_list_lock;
	struct xrp_allocation *busy_list;
};

struct xrp_known_file {
	void *filp;
	struct hlist_node node;
};

#define HIFI_STATE_PADDR 0xa0000000
#define HIFI_STATE_READY 0x11111111
//#define HIFI_XRP_HEAD_PADDR 0xa0000100
#define HIFI_XRP_HEAD_PADDR 0x5170100
#define HIFI_XRP_HEAD_START_0 0x55555555
#define HIFI_XRP_HEAD_START_1 0xAAAAAAAA
// #define HIFI_XRP_MSG_PADDR 0xa0000200
// #define HIFI_XRP_NS_PADDR 0xa0000250
#define HIFI_XRP_MSG_PADDR 0x5170200
#define HIFI_XRP_NS_PADDR 0x5170250
// #define HIFI_XRP_PCM_MSG_PADDR 0xa0000300
// #define HIFI_XRP_IN_DATA_PADDR 0xa0001000
// #define HIFI_XRP_REF_DATA_PADDR 0xa0002000
// #define HIFI_XRP_OUT_DATA_PADDR 0xa0003000
#define HIFI_XRP_PCM_MSG_PADDR 0x5170300
#define HIFI_XRP_IN_DATA_PADDR 0x5171000
#define HIFI_XRP_REF_DATA_PADDR 0x5172000
#define HIFI_XRP_OUT_DATA_PADDR 0x5173000

enum xrp_opcode{
	XRP_OPCODE_CMD_NONE = 0,
	XRP_OPCODE_CMD_CTRL = 1,
	XRP_OPCODE_CMD_PCM_DATA =2,
	XRP_OPCODE_CMD_GET_INFO = 3,
	XRP_OPCODE_CMD_FLUSH_DATA = 4,
	XRP_OPCODE_CMD_ALLOC_DRAM = 5,
	XRP_OPCODE_CMD_USER_DEFINE1 = 6,
	XRP_OPCODE_CMD_INVALID = 0xFF,
};

struct xrp_dsp_data
{
	__u32 data_addr;
	__u32 data_size;
};
struct pcm_cmd {
	struct xrp_dsp_data in_data;
	struct xrp_dsp_data ref_data;
	struct xrp_dsp_data out_data;
};
typedef struct
{
	    uint32_t data_addr;
	    uint32_t size;
}shm_arry_t;
typedef struct
{
	uint8_t opcode;
	shm_arry_t user_data;
	shm_arry_t i_name_space_id;
}hifi_a78_shm_t;
static int firmware_command_timeout = XRP_DEFAULT_TIMEOUT;
module_param(firmware_command_timeout, int, 0644);
MODULE_PARM_DESC(firmware_command_timeout, "Firmware command timeout in seconds.");

static int firmware_reboot = 1;
module_param(firmware_reboot, int, 0644);
MODULE_PARM_DESC(firmware_reboot, "Reboot firmware on command timeout.");

enum {
	LOOPBACK_NORMAL,	/* normal work mode */
	LOOPBACK_NOIO,		/* don't communicate with FW, but still load it and control DSP */
	LOOPBACK_NOMMIO,	/* don't comminicate with FW or use DSP MMIO, but still load the FW */
	LOOPBACK_NOFIRMWARE,	/* don't communicate with FW or use DSP MMIO, don't load the FW */
	LOOPBACK_BSTONLY,	/* don't communicate with FW , don't load the FW    use DSP MMIO   BST only*/
};
static int loopback = LOOPBACK_BSTONLY;
module_param(loopback, int, 0644);
MODULE_PARM_DESC(loopback, "Don't use actual DSP, perform everything locally.");

static DEFINE_HASHTABLE(xrp_known_files, 10);
static DEFINE_SPINLOCK(xrp_known_files_lock);

static DEFINE_IDA(xvp_nodeid);

static DEFINE_MUTEX(xrp_shared_allocations_lock);
static DEFINE_HASHTABLE(xrp_shared_allocations, 10);

static int xrp_boot_firmware(struct xvp *xvp);

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 12, 0)
#define of_reserved_mem_device_init(dev) 0
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 13, 0)
#define devm_kmalloc devm_kzalloc
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 15, 0)
#define devm_kstrdup xrp_devm_kstrdup
static char *xrp_devm_kstrdup(struct device *dev, const char *s, gfp_t gfp)
{
       size_t size;
       char *buf;

       if (!s)
               return NULL;

       size = strlen(s) + 1;
       buf = devm_kmalloc(dev, size, gfp);
       if (buf)
               memcpy(buf, s, size);
       return buf;
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 8, 0)
#define mmap_read_lock(mm) (down_read(&(mm)->mmap_sem))
#define mmap_read_unlock(mm) (up_read(&(mm)->mmap_sem))
#endif

#if 0
static bool xrp_cacheable(struct xvp *xvp, unsigned long pfn,
			  unsigned long n_pages)
{
	if (xvp->hw_ops->cacheable) {
		pr_debug("%s 1 \n", __func__);
		return xvp->hw_ops->cacheable(xvp->hw_arg, pfn, n_pages);
	} else {
		unsigned long i;
		pr_debug("%s 2 \n", __func__);
		for (i = 0; i < n_pages; ++i)
			if (!pfn_valid(pfn + i))
				return false;
		return true;
	}
}
#else
static bool xrp_cacheable(struct xvp *xvp, unsigned long pfn,
			  unsigned long n_pages)
{
	if (xvp->hw_ops->cacheable) {
		return xvp->hw_ops->cacheable(xvp->hw_arg, pfn, n_pages);
	} else {
		unsigned long i;
		for (i = 0; i < n_pages; ++i)
			if (!pfn_valid(pfn + i))
				return false;
		return false;
	}
}

#endif


static int xrp_dma_direction(unsigned flags)
{
	static const enum dma_data_direction xrp_dma_direction[] = {
		[0] = DMA_NONE,
		[XRP_FLAG_READ] = DMA_TO_DEVICE,
		[XRP_FLAG_WRITE] = DMA_FROM_DEVICE,
		[XRP_FLAG_READ_WRITE] = DMA_BIDIRECTIONAL,
	};
	return xrp_dma_direction[flags & XRP_FLAG_READ_WRITE];
}



static void xrp_default_dma_sync_for_device(struct xvp *xvp,
					    phys_addr_t phys,
					    unsigned long size,
					    unsigned long flags)
{
	int direct = xrp_dma_direction(flags);
	pr_info("%s, flags:%ld, direct:%d \n", __func__, flags, direct);
	dma_sync_single_for_device(xvp->dev, phys_to_dma(xvp->dev, phys), size,
				   xrp_dma_direction(flags));
}

static void xrp_dma_sync_for_device(struct xvp *xvp,
				    unsigned long virt,
				    phys_addr_t phys,
				    unsigned long size,
				    unsigned long flags)
{
	if (xvp->hw_ops->dma_sync_for_device){
		pr_debug("%s: xvp->hw_ops->dma_sync_for_device\n",__func__);
		xvp->hw_ops->dma_sync_for_device(xvp->hw_arg,
						 (void *)virt, phys, size,
						 flags);
	}else{
		pr_debug("%s: xrp_default_dma_sync_for_device \n",__func__);
		xrp_default_dma_sync_for_device(xvp, phys, size, flags);
	}

}

static void xrp_default_dma_sync_for_cpu(struct xvp *xvp,
					 phys_addr_t phys,
					 unsigned long size,
					 unsigned long flags)
{
	dma_sync_single_for_cpu(xvp->dev, phys_to_dma(xvp->dev, phys), size,
				xrp_dma_direction(flags));
}

static void xrp_dma_sync_for_cpu(struct xvp *xvp,
				 unsigned long virt,
				 phys_addr_t phys,
				 unsigned long size,
				 unsigned long flags)
{
	if (xvp->hw_ops->dma_sync_for_cpu)
		xvp->hw_ops->dma_sync_for_cpu(xvp->hw_arg,
					      (void *)virt, phys, size,
					      flags);
	else
		xrp_default_dma_sync_for_cpu(xvp, phys, size, flags);
}

static void *xrp_alloc_host(struct xvp *xvp, size_t sz)
{
	if (xvp->hw_ops->alloc_host)
		return xvp->hw_ops->alloc_host(xvp->hw_arg, sz);
	else
		return kmalloc(sz, GFP_KERNEL);
}

static void xrp_free_host(struct xvp *xvp, void *p)
{
	if (xvp->hw_ops->free_host)
		xvp->hw_ops->free_host(xvp->hw_arg, p);
	else
		kfree(p);
}

static inline void xrp_copy_from_alloc(struct xvp *xvp,
				       void *p, unsigned long sz,
				       phys_addr_t paddr)
{
	if (!xvp->direct_mapping)
		xvp->hw_ops->copy_from_alloc(xvp->hw_arg, p, sz, paddr);
}

static inline void xrp_comm_copy_from_alloc(struct xvp *xvp,
					    volatile void __iomem *p,
					    unsigned long sz)
{
	xrp_copy_from_alloc(xvp, (void __force *)p, sz,
			    xvp->comm_phys + (p - xvp->comm));
}

static inline void xrp_copy_to_alloc(struct xvp *xvp,
				     const void *p, unsigned long sz,
				     phys_addr_t paddr)
{
	if (!xvp->direct_mapping)
		xvp->hw_ops->copy_to_alloc(xvp->hw_arg, p, sz, paddr);
}

static inline void xrp_comm_copy_to_alloc(struct xvp *xvp,
					  const volatile void __iomem *p,
					  unsigned long sz)
{
	xrp_copy_to_alloc(xvp, (const void __force *)p, sz,
			  xvp->comm_phys + (p - xvp->comm));
}

static inline void xrp_comm_write32(volatile void __iomem *addr, u32 v)
{
	__raw_writel(v, addr);
}

static inline u32 xrp_comm_read32(volatile void __iomem *addr)
{
	return __raw_readl(addr);
}

static inline void xrp_comm_copy_write32(struct xvp *xvp,
					 volatile void __iomem *addr, u32 v)
{
	xrp_comm_write32(addr, v);
	xrp_comm_copy_to_alloc(xvp, addr, sizeof(u32));
}

static inline u32 xrp_comm_copy_read32(struct xvp *xvp,
				       volatile void __iomem *addr)
{
	xrp_comm_copy_from_alloc(xvp, addr, sizeof(u32));
	return xrp_comm_read32(addr);
}

static inline void __iomem *xrp_comm_put_tlv(void __iomem **addr,
					     uint32_t type,
					     uint32_t length)
{
	struct xrp_dsp_tlv __iomem *tlv = *addr;

	xrp_comm_write32(&tlv->type, type);
	xrp_comm_write32(&tlv->length, length);
	*addr = tlv->value + ((length + 3) / 4);
	return tlv->value;
}

static inline void __iomem *xrp_comm_get_tlv(void __iomem **addr,
					     uint32_t *type,
					     uint32_t *length)
{
	struct xrp_dsp_tlv __iomem *tlv = *addr;

	*type = xrp_comm_read32(&tlv->type);
	*length = xrp_comm_read32(&tlv->length);
	*addr = tlv->value + ((*length + 3) / 4);
	return tlv->value;
}

static inline void xrp_comm_write(volatile void __iomem *addr, const void *p,
				  size_t sz)
{
	size_t sz32 = sz & ~3;
	u32 v;

	while (sz32) {
		memcpy(&v, p, sizeof(v));
		__raw_writel(v, addr);
		p += 4;
		addr += 4;
		sz32 -= 4;
	}
	sz &= 3;
	if (sz) {
		v = 0;
		memcpy(&v, p, sz);
		__raw_writel(v, addr);
	}
}

static inline void xrp_comm_read(volatile void __iomem *addr, void *p,
				  size_t sz)
{
	size_t sz32 = sz & ~3;
	u32 v;

	while (sz32) {
		v = __raw_readl(addr);
		memcpy(p, &v, sizeof(v));
		p += 4;
		addr += 4;
		sz32 -= 4;
	}
	sz &= 3;
	if (sz) {
		v = __raw_readl(addr);
		memcpy(p, &v, sz);
	}
}

static inline void xrp_send_device_irq(struct xvp *xvp)
{
	if (xvp->hw_ops->send_irq)
		xvp->hw_ops->send_irq(xvp->hw_arg);
}

static inline bool xrp_panic_check(struct xvp *xvp)
{
	if (xvp->hw_ops->panic_check)
		return xvp->hw_ops->panic_check(xvp->hw_arg);
	else
		return false;
}

static void xrp_add_known_file(struct file *filp)
{
	struct xrp_known_file *p = kmalloc(sizeof(*p), GFP_KERNEL);

	if (!p)
		return;

	p->filp = filp;
	spin_lock(&xrp_known_files_lock);
	hash_add(xrp_known_files, &p->node, (unsigned long)filp);
	spin_unlock(&xrp_known_files_lock);
}

static void xrp_remove_known_file(struct file *filp)
{
	struct xrp_known_file *p;
	struct xrp_known_file *pf = NULL;

	spin_lock(&xrp_known_files_lock);
	hash_for_each_possible(xrp_known_files, p, node, (unsigned long)filp) {
		if (p->filp == filp) {
			hash_del(&p->node);
			pf = p;
			break;
		}
	}
	spin_unlock(&xrp_known_files_lock);
	if (pf)
		kfree(pf);
}

static bool xrp_is_known_file(struct file *filp)
{
	bool ret = false;
	struct xrp_known_file *p;

	spin_lock(&xrp_known_files_lock);
	hash_for_each_possible(xrp_known_files, p, node, (unsigned long)filp) {
		if (p->filp == filp) {
			ret = true;
			break;
		}
	}
	spin_unlock(&xrp_known_files_lock);
	return ret;
}

static void xrp_lock_shared_allocations(void)
{
	mutex_lock(&xrp_shared_allocations_lock);
}

static void xrp_unlock_shared_allocations(void)
{
	mutex_unlock(&xrp_shared_allocations_lock);
}

static struct xrp_shared_allocation *
xrp_get_shared_allocation(unsigned long flags,
			  unsigned long vaddr,
			  unsigned long size)
{
	struct xrp_shared_allocation *p;
	unsigned long mm = (unsigned long)(current->mm);

	hash_for_each_possible(xrp_shared_allocations, p, node, vaddr ^ size ^ mm) {
		if (p->vaddr == vaddr &&
		    p->size == size &&
		    p->mm == mm) {
			return p;
		}
	}
	return NULL;
}

static struct xrp_shared_allocation *
xrp_add_shared_allocation(unsigned long flags,
			  unsigned long vaddr,
			  unsigned long size,
			  struct xrp_allocation *allocation)
{
	struct xrp_shared_allocation *p = kmalloc(sizeof(*p), GFP_KERNEL);
	unsigned long mm = (unsigned long)(current->mm);

	if (!p)
		return NULL;

	p->flags = flags;
	p->vaddr = vaddr;
	p->size = size;
	p->mm = mm;
	p->allocation = allocation;

	hash_add(xrp_shared_allocations, &p->node, vaddr ^ size ^ mm);
	return p;
}

static void xrp_remove_shared_allocation(struct xrp_shared_allocation *p)
{
	WARN_ON(!p);
	if (p) {
		hash_del(&p->node);
		kfree(p);
	}
}

static void xrp_sync_v2(struct xvp *xvp,
			void *hw_sync_data, size_t sz)
{
												struct xrp_dsp_sync_v2 __iomem *shared_sync = xvp->comm;
												void __iomem *addr = shared_sync->hw_sync_data;

												xrp_comm_write(xrp_comm_put_tlv(&addr,
																XRP_DSP_SYNC_TYPE_HW_SPEC_DATA, sz),
														hw_sync_data, sz);
												if (xvp->n_queues > 1) {
													struct xrp_dsp_sync_v2 __iomem *queue_sync;
													unsigned i;

													xrp_comm_write(xrp_comm_put_tlv(&addr,
																	XRP_DSP_SYNC_TYPE_HW_QUEUES,
																	xvp->n_queues * sizeof(u32)),
															xvp->queue_priority,
															xvp->n_queues * sizeof(u32));
													for (i = 1; i < xvp->n_queues; ++i) {
														queue_sync = xvp->queue[i].comm;
														xrp_comm_write32(&queue_sync->sync,
																XRP_DSP_SYNC_IDLE);
													}
												}
												xrp_comm_put_tlv(&addr, XRP_DSP_SYNC_TYPE_LAST, 0);
}

static int xrp_sync_complete_v2(struct xvp *xvp, size_t sz)
{
	struct xrp_dsp_sync_v2 __iomem *shared_sync = xvp->comm;
	void __iomem *addr = shared_sync->hw_sync_data;
	u32 type, len;

	xrp_comm_get_tlv(&addr, &type, &len);
	if (len != sz) {
		dev_err(xvp->dev,
			"HW spec data size modified by the DSP\n");
		return -EINVAL;
	}
	if (!(type & XRP_DSP_SYNC_TYPE_ACCEPT))
		dev_err(xvp->dev,
			 "HW spec data not recognized by the DSP\n");

	if (xvp->n_queues > 1) {
		void __iomem *p = xrp_comm_get_tlv(&addr, &type, &len);

		if (len != xvp->n_queues * sizeof(u32)) {
			dev_err(xvp->dev,
				"Queue priority size modified by the DSP\n");
			return -EINVAL;
		}
		if (type & XRP_DSP_SYNC_TYPE_ACCEPT) {
			xrp_comm_read(p, xvp->queue_priority,
				      xvp->n_queues * sizeof(u32));
		} else {
			dev_info(xvp->dev,
				 "Queue priority data not recognized by the DSP\n");
			xvp->n_queues = 1;
		}
	}
	return 0;
}
static u32 bst_xrp_read_phys_bst(u32 phys_addr)
{
	u32 phys_addr_page = phys_addr & 0xFFFFE000;
	u32 phys_offset = phys_addr & 0x00001FFF;
	u32 map_size = phys_offset + sizeof(u32);
	u32 ret = 0xDEADBEEF;
	void *mem_mapped = ioremap(phys_addr_page, map_size);

	if (mem_mapped != NULL) {
		ret = (u32)ioread32(((u8 *)mem_mapped) + phys_offset);
		iounmap(mem_mapped);
	}

	return ret;
}

static void bst_xrp_write_phys_bst(u32 phys_addr, u32 value)
{
	u32 phys_addr_page = phys_addr & 0xFFFFE000;
	u32 phys_offset = phys_addr & 0x00001FFF;
	u32 map_size = phys_offset + sizeof(u32);
	void *mem_mapped = ioremap(phys_addr_page, map_size);

	if (mem_mapped != NULL) {
		iowrite32(value, ((u8 *)mem_mapped) + phys_offset);
		iounmap(mem_mapped);
	}
}

#if 0

static void bst_xrp_phys_cpy(u32 phys_addr, void* p,u32 size)
{
	u32 phys_addr_page = phys_addr & 0xFFFFE000;
	u32 phys_offset = phys_addr & 0x00001FFF;
	u32 map_size = phys_offset + size;
	void *mem_mapped = ioremap(phys_addr_page, map_size );

	if (mem_mapped != NULL) {
		memcpy_toio(mem_mapped+ phys_offset, p, ALIGN(size, 4));
		//memcpy(mem_mapped,p,size);
		iounmap(mem_mapped);
	}
	else
	{
		pr_err("0x%x map failed \r\n",phys_addr);
	}
}
#endif

#if 0
static void bst_xrp_phys_get(u32 phys_addr, void* p,u32 size)
{
	u32 phys_addr_page = phys_addr & 0xFFFFE000;
	u32 phys_offset = phys_addr & 0x00001FFF;
	u32 map_size = phys_offset + size;
	void *mem_mapped = ioremap(phys_addr_page, map_size );

	if (mem_mapped != NULL) {
		memcpy_fromio(p,mem_mapped+ phys_offset, ALIGN(size, 4));
		//memcpy(mem_mapped,p,size);
		iounmap(mem_mapped);
	}
	else
	{
		pr_err("0x%x map failed \r\n",phys_addr);
	}
}


static int xrp_sharememory_alg_test(struct xvp *xvp)
{
	size_t sz;
	void *hw_sync_data;

	struct pcm_cmd *pcm = NULL;
//	uint8_t __iomem *shared_sync = (uint8_t __iomem *)xvp->comm;
//	phys_addr_t comm_phys = xvp->comm_phys; 
//	u32 phys_offset = comm_phys & 0x00001FFF;
	hifi_a78_shm_t *xrp_msg;
	int ret;
	pr_err("%s: %d in \r\n", __func__,__LINE__);
	hw_sync_data = xvp->hw_ops->get_hw_sync_data(xvp->hw_arg, &sz);
	if (!hw_sync_data) {
		pr_err("%s: %d hw_sync_data failed  \r\n", __func__,__LINE__);
		ret = -ENOMEM;
		return ret;
	}
	xrp_msg = kmalloc(sizeof(hifi_a78_shm_t),GFP_KERNEL);
	if(!xrp_msg)
	{
		pr_err("%s: %d kmalloc failed  \r\n", __func__,__LINE__);
		kfree(xrp_msg);
		xrp_msg = NULL;
		ret = -ENOMEM;
		return ret;
	}

	while(1) //hifi ready
	{
		if(bst_xrp_read_phys_bst(HIFI_STATE_PADDR) == HIFI_STATE_READY)
			break;
	}
	bst_xrp_write_phys_bst(HIFI_XRP_HEAD_PADDR,HIFI_XRP_HEAD_START_0);//head 
	xrp_msg->opcode =XRP_OPCODE_CMD_PCM_DATA;
	xrp_msg->i_name_space_id.data_addr = HIFI_XRP_PCM_MSG_PADDR;
	xrp_msg->i_name_space_id.size = 16;
	pcm = kmalloc(sizeof(struct pcm_cmd),GFP_KERNEL);
	if(!pcm)
	{
		pr_err("%s: %d kmalloc failed  \r\n", __func__,__LINE__);
		kfree(pcm);
		pcm = NULL;
		ret = -ENOMEM;
		return ret;
	}
	pcm->in_data.data_addr = HIFI_XRP_IN_DATA_PADDR;
	pcm->in_data.data_size = sizeof(mic_in);
	pcm->ref_data.data_addr = HIFI_XRP_REF_DATA_PADDR;
	pcm->ref_data.data_size = sizeof(spk_in);
	pcm->out_data.data_addr = HIFI_XRP_OUT_DATA_PADDR;
	pcm->out_data.data_size = sizeof(out_aec_ns);
	bst_xrp_phys_cpy(HIFI_XRP_PCM_MSG_PADDR,(void*)pcm,sizeof(struct pcm_cmd));
	xrp_msg->user_data.data_addr = HIFI_XRP_PCM_MSG_PADDR;
	xrp_msg->user_data.size = sizeof(struct pcm_cmd)/sizeof(uint8_t);
	bst_xrp_phys_cpy(HIFI_XRP_IN_DATA_PADDR,(void*)mic_in,sizeof(mic_in));

	bst_xrp_phys_cpy(HIFI_XRP_REF_DATA_PADDR,(void*)spk_in,sizeof(spk_in));
	bst_xrp_phys_cpy(HIFI_XRP_MSG_PADDR,(void*)xrp_msg,sizeof(hifi_a78_shm_t));
	bst_xrp_write_phys_bst(HIFI_XRP_HEAD_PADDR+4,HIFI_XRP_HEAD_START_1);//alg start 
	pr_err("%s: %d while in \r\n", __func__,__LINE__);
	while(1) //hifi alg finished 
	{
		if(bst_xrp_read_phys_bst(HIFI_XRP_HEAD_PADDR) == 0xFFFFFFFF)
			break;
	}
	bst_xrp_phys_get(HIFI_XRP_MSG_PADDR,(void*)xrp_msg,sizeof(hifi_a78_shm_t));

	//xrp_comm_write((volatile void __iomem *)(shared_sync+0x100), hw_sync_data, sz);
	pr_err("%s: %d out \r\n", __func__,__LINE__);
	kfree(xrp_msg);
	kfree(pcm);
	pcm = NULL;
	xrp_msg = NULL;
	return 0;
}
#endif

static int xrp_synchronize(struct xvp *xvp)
{
										size_t sz;
										void *hw_sync_data;
										unsigned long deadline = jiffies + firmware_command_timeout * HZ;
										struct xrp_dsp_sync_v1 __iomem *shared_sync = xvp->comm;
										int ret;
										u32 v, v1;
										dev_err(xvp->dev, "xrp_synchronize\n");
										hw_sync_data = xvp->hw_ops->get_hw_sync_data(xvp->hw_arg, &sz);
										if (!hw_sync_data) {
											ret = -ENOMEM;
											goto err;
										}
										ret = -ENODEV;
										pr_info("%s, origin sync: 0x%x\n", __func__, shared_sync->sync);
										xrp_comm_copy_write32(xvp, &shared_sync->sync, XRP_DSP_SYNC_START);

										mb();
										do {
											v = xrp_comm_copy_read32(xvp, &shared_sync->sync);
											if (v != XRP_DSP_SYNC_START)
												break;
											if (xrp_panic_check(xvp))
												goto err;
											schedule();
										} while (time_before(jiffies, deadline));

										pr_info("%s, v: 0x%x\n", __func__, v);

										switch (v) {
										case XRP_DSP_SYNC_DSP_READY_V1:
											if (xvp->n_queues > 1) {
												dev_info(xvp->dev,
													"Queue priority data not recognized by the DSP\n");
												xvp->n_queues = 1;
											}
											xrp_comm_write(&shared_sync->hw_sync_data, hw_sync_data, sz);
											break;
										case XRP_DSP_SYNC_DSP_READY_V2:
											xrp_sync_v2(xvp, hw_sync_data, sz);
											break;
										case XRP_DSP_SYNC_START:
											dev_err(xvp->dev, "DSP is not ready for synchronization\n");
											goto err;
										default:
											dev_err(xvp->dev,
												"DSP response to XRP_DSP_SYNC_START is not recognized\n");
											goto err;
										}

										xrp_comm_copy_to_alloc(xvp, shared_sync, PAGE_SIZE);
										mb();
										xrp_comm_copy_write32(xvp, &shared_sync->sync,
													XRP_DSP_SYNC_HOST_TO_DSP);

										do {
											mb();
											v1 = xrp_comm_copy_read32(xvp, &shared_sync->sync);
											if (v1 == XRP_DSP_SYNC_DSP_TO_HOST) {
												dev_err(xvp->dev, "XRP_DSP_SYNC_DSP_TO_HOST\n");
												break;
											}

											if (xrp_panic_check(xvp))
												goto err;
											schedule();
										} while (time_before(jiffies, deadline));

										if (v1 != XRP_DSP_SYNC_DSP_TO_HOST) {
											dev_err(xvp->dev,
												"DSP haven't confirmed initialization data reception\n");
											goto err;
										}

										xrp_comm_copy_from_alloc(xvp, shared_sync, PAGE_SIZE);

										if (v == XRP_DSP_SYNC_DSP_READY_V2) {
											ret = xrp_sync_complete_v2(xvp, sz);
											if (ret < 0)
												goto err;
										}

										xrp_send_device_irq(xvp);

										if (xvp->host_irq_mode) {
											int res = wait_for_completion_timeout(&xvp->queue[0].completion,
																firmware_command_timeout * HZ);
											dev_err(xvp->dev, "after wait_for_completion_timeout\n");

											ret = -ENODEV;
											if (xrp_panic_check(xvp))
												goto err;
											if (res == 0) {
												dev_err(xvp->dev,
													"host IRQ mode is requested, but DSP couldn't deliver IRQ during synchronization\n");
												goto err;
											}
										}
										ret = 0;
err:
	kfree(hw_sync_data);
	xrp_comm_copy_write32(xvp, &shared_sync->sync, XRP_DSP_SYNC_IDLE);
	return ret;
}

static bool xrp_cmd_complete(struct xvp *xvp, struct xrp_comm *xrp_comm)
{
	struct xrp_dsp_cmd __iomem *cmd = xrp_comm->comm;
	u32 flags;

	xrp_comm_copy_from_alloc(xvp, cmd, sizeof(*cmd));
	flags = xrp_comm_read32(&cmd->flags);

	rmb();
	return (flags & (XRP_DSP_CMD_FLAG_REQUEST_VALID |
			 XRP_DSP_CMD_FLAG_RESPONSE_VALID)) ==
		(XRP_DSP_CMD_FLAG_REQUEST_VALID |
		 XRP_DSP_CMD_FLAG_RESPONSE_VALID);
}

irqreturn_t xrp_irq_handler(int irq, struct xvp *xvp)
{
	unsigned i, n = 0;

	dev_err(xvp->dev, "%s\n", __func__);
	if (!xvp->comm)
		return IRQ_NONE;

	for (i = 0; i < xvp->n_queues; ++i) {
		if (xrp_cmd_complete(xvp, xvp->queue + i)) {
			dev_dbg(xvp->dev, "  completing queue %d\n", i);
			complete(&xvp->queue[i].completion);
			++n;
		}
	}
	return n ? IRQ_HANDLED : IRQ_NONE;
}
EXPORT_SYMBOL(xrp_irq_handler);

static inline void xvp_file_lock(struct xvp_file *xvp_file)
{
	spin_lock(&xvp_file->busy_list_lock);
}

static inline void xvp_file_unlock(struct xvp_file *xvp_file)
{
	spin_unlock(&xvp_file->busy_list_lock);
}

static void xrp_allocation_queue(struct xvp_file *xvp_file,
				 struct xrp_allocation *xrp_allocation)
{
	xvp_file_lock(xvp_file);

	xrp_allocation->next = xvp_file->busy_list;
	xvp_file->busy_list = xrp_allocation;

	xvp_file_unlock(xvp_file);
}

static struct xrp_allocation *xrp_allocation_dequeue(struct xvp_file *xvp_file,
						     phys_addr_t paddr, u32 size)
{
	struct xrp_allocation **pcur;
	struct xrp_allocation *cur;

	xvp_file_lock(xvp_file);

	for (pcur = &xvp_file->busy_list; (cur = *pcur); pcur = &((*pcur)->next)) {
#ifdef CONFIG_HIFI_XRP_LOG_EN
		pr_debug("%s: %pap %d / %pap x %d\n", __func__, &paddr, size, &cur->start, cur->size);
#endif
		if (paddr >= cur->start && paddr + size - cur->start <= cur->size) {
			*pcur = cur->next;
			break;
		}
	}

	xvp_file_unlock(xvp_file);
	return cur;
}
static long xrp_ioctl_alloc_sync_cmd(struct file *filp,
			    struct xrp_sync_cmd __user *p,uint32_t pio_flag)
{
	#if 0
	#ifdef ipc_en
	test_xrp_msg_t *xrp_msg;
	struct xrp_sync_cmd tmp_cmd;
	xrp_msg =kzalloc(sizeof(test_xrp_msg_t),GFP_KERNEL);
	if(!xrp_msg)
	return -ENOMEM;
	tmp_cmd.cmd_addr = (long)xrp_msg;
	tmp_cmd.cmd_size = sizeof(test_xrp_msg_t);
	pr_debug("%s: cmd_addr: 0x%x \n", __func__, tmp_cmd.cmd_addr);
	if (copy_to_user(p, &tmp_cmd, sizeof(*p))) {

		return -EFAULT;
	}
	#endif
	#endif

	return 0;
}
#ifdef ipc_en
#if 0
static void xrp_complex_method_reply(const uint8_t response, const test_Array_Uint8_t resp_data, const audio_ipc_ErrorEnum_t err, void *ext)
{
   	struct xvp *xvp = (struct xvp *)ext;
	uint32_t i =0;
 	printf("Receive complex_method reply.\n");
	switch (response)
	{
		case 0x01:
		for (i = 0; i < xvp->n_queues; ++i) {
			dev_dbg(xvp->dev, "  completing queue %d\n", i);
			complete(&xvp->queue[i].completion);
		}
		break;

		default:
			break;
	}
}
#endif
#endif

static long xrp_ioctl_sync_cmd(struct file *filp,
			    struct xrp_sync_cmd __user *p,uint32_t pio_flag)
{
	struct xvp_file *xvp_file = filp->private_data;
	struct xvp *xvp = xvp_file->xvp;

#ifdef ipc_en

	audio_msgbox_client_t *xrp_ipc =xvp->xrp_ipc;
	struct xrp_sync_cmd tmp_cmd;
//	test_xrp_msg_t *xrp_msg;
	audio_ipc_ErrorEnum_t err = AUDIO_IPC_NO_ERROR;
	char *message = NULL;
	des_buf_t *ext_buf =kzalloc(sizeof(des_buf_t),GFP_KERNEL);

	pr_debug("%s: \n", __func__);

	if (copy_from_user(&tmp_cmd, p, sizeof(*p))) {
		kfree(ext_buf);
		return -EFAULT;
	}

	pr_debug("%s: input_str: %s \n", __func__, tmp_cmd.input_str);

	if(!pio_flag)
	{
		int32_t ret;
		pr_debug("%s: call hello_sync \n", __func__);
		ret =xrp_ipc->audio_ipc_client.hello_sync(tmp_cmd.input_str, &message, &err, 0, ext_buf);

		if (ret < 0)
		{
			kfree(ext_buf);
			pr_debug("%s: send hello failed \n", __func__);
			return -EIO;
		}
		if (message != NULL) {
			pr_debug("%s: message: %s \n", __func__, message);
			pr_debug("%s: err: %d \n", __func__, err);
			strlcpy(tmp_cmd.out_str, message, sizeof(tmp_cmd.out_str));

			if (copy_to_user(p, &tmp_cmd, sizeof(*p))) {
				kfree(ext_buf);
				return -EFAULT;
			}
		}

//		xrp_synchronize(xvp);

	}
	else
	{
		bst_xrp_write_phys_bst(0xa0000100,0x55555555);
//		bst_xrp_phys_cpy(0xA0000200,(void*)xrp_msg,sizeof(test_xrp_msg_t));
	}

	kfree(ext_buf);
#endif
	return 0;
}

static long xrp_ioctl_nsid_get(struct file *filp,
			    struct xrp_nsid __user *p,uint32_t pio_flag)
{
#ifdef ipc_en
	#if 0
	test_hifi_dsp_client *xrp_ipc =xvp->xrp_ipc;

	test_xrp_msg_t *xrp_msg;
	struct xrp_nsid tmp_cmd;
	uint8_t name_id[16] ={0};
	uint32_t nsid_count;
	if (copy_from_user(&tmp_cmd, p, sizeof(*p))) {
		return -EFAULT;
	}
	if(tmp_cmd.nsid_size<16)
	{
		return -EFAULT;
	}
	nsid_count = tmp_cmd.nsid_size/16;
	tmp_cmd.nsid_addr = name_id;
	tmp_cmd.nsid_size =16;
	while(nsid_count --)
	{
		xrp_msg = (test_xrp_msg_t *)&tmp_cmd;
		//get ns id from hifi firmware
		if(!pio_flag)//ipc mode 
		{

			int32_t ret =xrp_ipc->hifi_a78_msg_sync(*xrp_msg,xrp_complex_method_reply,NULL);
			if (ret < 0)
			{
				printf("send method complex_method failed. ret is %u\n", ret);
				return -EIO;
			}

		}
		else// share memory mode
		{
			bst_xrp_write_phys_bst(0xa0000100,0x55555555);
			bst_xrp_phys_cpy(0xA0000200,(void*)&tmp_cmd,sizeof(test_xrp_msg_t));
		}
		if(copy_to_user(p,&tmp_cmd,sizeof(tmp_cmd)))
		{
			return -EIO;
		}
	}
	#endif
#endif
	return 0;
}

static long xrp_ioctl_def_alg_set(struct file *filp,
			    struct xrp_alg_param __user *p,uint32_t pio_flag)
{

#ifdef ipc_en
	#if 0
	test_hifi_dsp_client *xrp_ipc =xvp->xrp_ipc;

	test_xrp_msg_t *xrp_msg;
	struct xrp_alg_param tmp_para;    
    uint8_t in4_data[16] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff,0x00};    

	if (copy_from_user(&tmp_para, p, sizeof(*p))) {
		return -EFAULT;
	}
	xrp_ipc->def_xrp_msg->i_name_space_id.data = in4_data;
	xrp_ipc->def_xrp_msg->i_name_space_id.size = 16;
	xrp_ipc->def_xrp_msg->opcode = 0x10;
	xrp_ipc->def_xrp_msg->user_data.data =(uint8_t *) &tmp_para;
	xrp_ipc->def_xrp_msg->user_data.size = 15;
	if(!pio_flag)
	{

		int32_t ret =xrp_ipc->hifi_a78_msg_sync(*(xrp_ipc->def_xrp_msg),xrp_complex_method_reply,xvp);
		if (ret < 0)
		{
			printf("send method complex_method failed. ret is %u\n", ret);
			return -EIO;
		}

	}
	else
	{
		xrp_msg= xrp_ipc->def_xrp_msg;
		bst_xrp_write_phys_bst(0xa0000100,0x55555555);
		bst_xrp_phys_cpy(0xA0000200,(void*)xrp_msg,sizeof(test_xrp_msg_t));
	}
	#endif
#endif
	return 0;
}
#if 0
static long xrp_ioctl_def_alg_dataset(struct file *filp,
			    struct xrp_data_param  __user *p,uint32_t pio_flag)
{
	struct xvp_file *xvp_file = filp->private_data;
	struct xvp *xvp = xvp_file->xvp;
	#ifdef ipc_en
	test_hifi_dsp_client *xrp_ipc =xvp->xrp_ipc;

	test_xrp_msg_t *xrp_msg;
	struct xrp_data_param tmp_para;    
    uint8_t in4_data[16] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff,0x00};    

	if (copy_from_user(&tmp_para, p, sizeof( tmp_para))) {
		return -EFAULT;
	}

	if(!pio_flag)
	{

		xrp_ipc->def_xrp_msg->i_name_space_id.data = in4_data;
		xrp_ipc->def_xrp_msg->i_name_space_id.size = 16;
		xrp_ipc->def_xrp_msg->opcode = 0x10;
		xrp_ipc->def_xrp_msg->user_data.data =(uint32_t*) &tmp_para;
		xrp_ipc->def_xrp_msg->user_data.size = 15;
		int32_t ret =xrp_ipc->hifi_a78_msg_sync(*(xrp_ipc->def_xrp_msg),xrp_complex_method_reply,xvp);
		if (ret < 0)
		{
			printf("send method complex_method failed. ret is %u\n", ret);
			return -EIO;
		}

	}
	else
	{
		xrp_msg= xrp_ipc->def_xrp_msg;
		bst_xrp_write_phys_bst(0xa0000100,0x55555555);
		bst_xrp_phys_cpy(0xA0000200,(void*)xrp_msg,sizeof(test_xrp_msg_t));
	}
	#endif
	return 0;
}
#endif
static long xrp_ioctl_alg_result_get(struct file *filp,
			    struct xrp_output __user *p,uint32_t pio_flag)
{
	struct xrp_output tmp_para; 
	#ifdef ipc_en
	#if 0
	unsigned long deadline =jiffies + firmware_command_timeout * HZ;
	struct xvp_file *xvp_file = filp->private_data;
	struct xvp *xvp = xvp_file->xvp;

	test_hifi_dsp_client *xrp_ipc =xvp->xrp_ipc;

	test_xrp_msg_t *xrp_msg;
	   
    uint8_t in4_data[16] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff,0x00};    
	uint32_t in5_data[4] = {0x00};  
	if (copy_from_user(&tmp_para, p, sizeof(*p))) {
		return -EFAULT;
	}
	memcpy(in5_data,&tmp_para,sizeof(tmp_para));
	xrp_ipc->def_xrp_msg->i_name_space_id.data = in4_data;
	xrp_ipc->def_xrp_msg->i_name_space_id.size = 16;
	xrp_ipc->def_xrp_msg->opcode = 0x20;
	xrp_ipc->def_xrp_msg->user_data.data = (uint8_t *)in5_data;
	xrp_ipc->def_xrp_msg->user_data.size = 4; 
	if(!pio_flag){

		int32_t ret =xrp_ipc->hifi_a78_msg_sync(*(xrp_ipc->def_xrp_msg),xrp_complex_method_reply,NULL);
		if (ret < 0)
		{
			printf("send method complex_method failed. ret is %u\n", ret);
			return -EIO;
		}
		int res = wait_for_completion_timeout(&xvp->queue[0].completion,
								firmware_command_timeout * HZ);
		if(res) 
			return -EBUSY;

	}
	else
	{
		xrp_msg= xrp_ipc->def_xrp_msg;
		bst_xrp_write_phys_bst(0xa0000100,0x55555555);
		bst_xrp_phys_cpy(0xA0000200,(void*)xrp_msg,sizeof(test_xrp_msg_t));

		do {
		if (bst_xrp_read_phys_bst(0xa0000010) == 0xFFFFFFFF)
			goto copy;
		if (xrp_panic_check(xvp))
			return -EBUSY;
		schedule();
		} while (time_before(jiffies, deadline));

		return -EBUSY;
	}
	#endif
	#endif
//copy:
	tmp_para.output_addr = bst_xrp_read_phys_bst(0xa0000010);
	tmp_para.outsize = bst_xrp_read_phys_bst(0xa0000014);
	pr_err("output_addr = 0x%x outsize =%d \r\n",tmp_para.output_addr,tmp_para.outsize);
	if (copy_to_user(p, &tmp_para, sizeof(*p))) {

		return -EFAULT;
	}
	return 0;
}


static long xrp_ioctl_alloc(struct file *filp,
			    struct xrp_ioctl_alloc __user *p)
{
	struct xvp_file *xvp_file = filp->private_data;
	struct xrp_allocation *xrp_allocation;
	unsigned long vaddr;
	struct xrp_ioctl_alloc xrp_ioctl_alloc;
	long err;

#ifdef CONFIG_HIFI_XRP_LOG_EN
	pr_debug("%s: %p\n", __func__, p);
#endif

	if (copy_from_user(&xrp_ioctl_alloc, p, sizeof(*p)))
		return -EFAULT;

#ifdef CONFIG_HIFI_XRP_LOG_EN
	pr_debug("%s: size = %d, align = %x\n", __func__,
		 xrp_ioctl_alloc.size, xrp_ioctl_alloc.align);
#endif

	if (xvp_file->xvp->direct_mapping) {
		err = xrp_allocate(xvp_file->xvp->pool,
				   xrp_ioctl_alloc.size,
				   xrp_ioctl_alloc.align,
				   &xrp_allocation);
		if (err)
			return err;

		xrp_allocation_queue(xvp_file, xrp_allocation);

#ifdef CONFIG_HIFI_XRP_LOG_EN
		pr_debug("%s: before vm_mmap \n", __func__);
#endif

		vaddr = vm_mmap(filp, 0, xrp_allocation->size,
				PROT_READ | PROT_WRITE, MAP_SHARED,
				xrp_allocation_offset(xrp_allocation));
#ifdef CONFIG_HIFI_XRP_LOG_EN
		pr_debug("%s: vaddr = 0x%lx \n", __func__,vaddr);
#endif
	} else {
		vaddr = vm_mmap(NULL, 0, xrp_ioctl_alloc.size,
				PROT_READ | PROT_WRITE,
				MAP_SHARED | MAP_ANONYMOUS, 0);
	}

	if (IS_ERR((void *)vaddr)) {
		return PTR_ERR((void *)vaddr);
	}

	xrp_ioctl_alloc.addr = vaddr;

	if (copy_to_user(p, &xrp_ioctl_alloc, sizeof(*p))) {
		vm_munmap(vaddr, xrp_ioctl_alloc.size);
		return -EFAULT;
	}
	return 0;
}

static void xrp_put_pages(phys_addr_t phys, unsigned long n_pages)
{
	struct page *page;
	unsigned long i;

	page = pfn_to_page(__phys_to_pfn(phys));
	for (i = 0; i < n_pages; ++i)
		put_page(page + i);
}

static void xrp_alien_mapping_destroy(struct xrp_alien_mapping *alien_mapping)
{
	switch (alien_mapping->type) {
	case ALIEN_GUP:
		xrp_put_pages(alien_mapping->paddr,
			      PFN_UP(alien_mapping->vaddr +
				     alien_mapping->size) -
			      PFN_DOWN(alien_mapping->vaddr));
		break;
	case ALIEN_COPY:
		xrp_lock_shared_allocations();
		if (xrp_allocation_put(alien_mapping->allocation))
			xrp_remove_shared_allocation(alien_mapping->shared_allocation);
		xrp_unlock_shared_allocations();
		break;
	default:
		break;
	}
}

static long xvp_pfn_virt_to_phys(struct xvp_file *xvp_file,
				 struct vm_area_struct *vma,
				 unsigned long vaddr, unsigned long size,
				 phys_addr_t *paddr,
				 struct xrp_alien_mapping *mapping)
{
	int ret;
	unsigned long i;
	unsigned long nr_pages = PFN_UP(vaddr + size) - PFN_DOWN(vaddr);
	unsigned long pfn;
	const struct xrp_address_map_entry *address_map;

	ret = follow_pfn(vma, vaddr, &pfn);
	if (ret)
		return ret;

	*paddr = __pfn_to_phys(pfn) + (vaddr & ~PAGE_MASK);
	address_map = xrp_get_address_mapping(&xvp_file->xvp->address_map,
					      *paddr);
	if (!address_map) {
		pr_debug("%s: untranslatable addr: %pap\n", __func__, paddr);
		return -EINVAL;
	}

	for (i = 1; i < nr_pages; ++i) {
		unsigned long next_pfn;
		phys_addr_t next_phys;

		ret = follow_pfn(vma, vaddr + (i << PAGE_SHIFT), &next_pfn);
		if (ret)
			return ret;
		if (next_pfn != pfn + 1) {
			pr_debug("%s: non-contiguous physical memory\n",
				 __func__);
			return -EINVAL;
		}
		next_phys = __pfn_to_phys(next_pfn);
		if (xrp_compare_address(next_phys, address_map)) {
			pr_debug("%s: untranslatable addr: %pap\n",
				 __func__, &next_phys);
			return -EINVAL;
		}
		pfn = next_pfn;
	}
	*mapping = (struct xrp_alien_mapping){
		.vaddr = vaddr,
		.size = size,
		.paddr = *paddr,
		.type = ALIEN_PFN_MAP,
	};
	pr_debug("%s: success, paddr: %pap\n", __func__, paddr);
	return 0;
}

static long xvp_gup_virt_to_phys(struct xvp_file *xvp_file,
				 unsigned long vaddr, unsigned long size,
				 phys_addr_t *paddr,
				 struct xrp_alien_mapping *mapping)
{
	int ret;
	int i;
	int nr_pages;
	struct page **page;
	const struct xrp_address_map_entry *address_map;

	if (PFN_UP(vaddr + size) - PFN_DOWN(vaddr) > INT_MAX)
		return -EINVAL;

	nr_pages = PFN_UP(vaddr + size) - PFN_DOWN(vaddr);
	page = kmalloc(nr_pages * sizeof(void *), GFP_KERNEL);
	if (!page)
		return -ENOMEM;

	ret = get_user_pages_fast(vaddr, nr_pages, 1, page);
	if (ret < 0)
		goto out;

	if (ret < nr_pages) {
		pr_debug("%s: asked for %d pages, but got only %d\n",
			 __func__, nr_pages, ret);
		nr_pages = ret;
		ret = -EINVAL;
		goto out_put;
	}

	address_map = xrp_get_address_mapping(&xvp_file->xvp->address_map,
					      page_to_phys(page[0]));
	if (!address_map) {
		phys_addr_t addr = page_to_phys(page[0]);
		pr_debug("%s: untranslatable addr: %pap\n",
			 __func__, &addr);
		ret = -EINVAL;
		goto out_put;
	}

	for (i = 1; i < nr_pages; ++i) {
		phys_addr_t addr;

		if (page[i] != page[i - 1] + 1) {
			pr_debug("%s: non-contiguous physical memory\n",
				 __func__);
			ret = -EINVAL;
			goto out_put;
		}
		addr = page_to_phys(page[i]);
		if (xrp_compare_address(addr, address_map)) {
			pr_debug("%s: untranslatable addr: %pap\n",
				 __func__, &addr);
			ret = -EINVAL;
			goto out_put;
		}
	}

	*paddr = __pfn_to_phys(page_to_pfn(page[0])) + (vaddr & ~PAGE_MASK);
	*mapping = (struct xrp_alien_mapping){
		.vaddr = vaddr,
		.size = size,
		.paddr = *paddr,
		.type = ALIEN_GUP,
	};
	ret = 0;
	pr_debug("%s: success, paddr: %pap\n", __func__, paddr);

out_put:
	if (ret < 0)
		for (i = 0; i < nr_pages; ++i)
			put_page(page[i]);
out:
	kfree(page);
	return ret;
}

static long copy_from_x(void *dst, const void *src, size_t sz, bool user)
{
	if (user) {
		return copy_from_user(dst, (void __user *)src, sz);
	} else {
		char *src_byte = (char *)src;
		char *dst_byte = (char *)dst;
//		memcpy(dst, src, sz);
		for(int i=0; i<sz; i++) {
			dst_byte[i] = src_byte[i];
		}
		return 0;
	}
}

static long copy_to_x(void *dst, const void *src, size_t sz, bool user)
{
	if (user) {
		return copy_to_user((void __user *)dst, src, sz);
	} else {
		char *src_byte = (char *)src;
		char *dst_byte = (char *)dst;
//		memcpy(dst, src, sz);
		for(int i=0; i<sz; i++) {
			dst_byte[i] = src_byte[i];
		}
		return 0;
	}
}

static long _xrp_copy_user_phys(struct xvp *xvp,
				unsigned long vaddr, unsigned long size,
				phys_addr_t paddr, unsigned long flags,
				bool to_phys, bool user)
{
	void __iomem *p = ioremap(paddr, size);
	unsigned long rc;

#ifdef CONFIG_HIFI_XRP_LOG_EN
	pr_debug("%s: ioremap \n",__func__);
#endif

	if (!p) {
		dev_err(xvp->dev,
			"couldn't ioremap %pap x 0x%08x\n",
			&paddr, (u32)size);
		return -EINVAL;
	}
	if (to_phys)
		rc = copy_from_x(__io_virt(p),
				 (void *)vaddr, size, user);
	else
		rc = copy_to_x((void *)vaddr,
			       __io_virt(p), size, user);
	iounmap(p);
	if (rc)
		return -EFAULT;
	return 0;
}

static long xrp_copy_user_to_phys(struct xvp *xvp,
				  unsigned long vaddr, unsigned long size,
				  phys_addr_t paddr, unsigned long flags,
				  bool user)
{
	if (!xvp->direct_mapping) {
		if (!user)
			return xvp->hw_ops->copy_to_alloc(xvp->hw_arg,
							  (const void *)vaddr,
							  size, paddr);
		else
			return xvp->hw_ops->copy_to_alloc_user(xvp->hw_arg,
							       vaddr, size,
							       paddr);
	} else {
		return _xrp_copy_user_phys(xvp, vaddr, size, paddr, flags,
					   true, user);
	}
}

static long xrp_copy_user_from_phys(struct xvp *xvp,
				    unsigned long vaddr, unsigned long size,
				    phys_addr_t paddr, unsigned long flags,
				    bool user)
{
	if (!xvp->direct_mapping) {
		if (!user)
			return xvp->hw_ops->copy_from_alloc(xvp->hw_arg,
							    (void *)vaddr,
							    size, paddr);
		else
			return xvp->hw_ops->copy_from_alloc_user(xvp->hw_arg,
								 vaddr, size,
								 paddr);
	} else {
		return _xrp_copy_user_phys(xvp, vaddr, size, paddr, flags,
					   false, user);
	}
}

static long xvp_copy_virt_to_phys(struct xvp_file *xvp_file,
				  unsigned long flags,
				  unsigned long vaddr, unsigned long size,
				  phys_addr_t *paddr,
				  struct xrp_alien_mapping *mapping,
				  bool user)
{
											phys_addr_t phys;
											unsigned long align = clamp(vaddr & -vaddr, 16ul, PAGE_SIZE);
											unsigned long offset = vaddr & (align - 1);
											struct xrp_allocation *allocation;
											struct xrp_shared_allocation *shared_allocation;
											long rc;

											xrp_lock_shared_allocations();
											shared_allocation = xrp_get_shared_allocation(flags, vaddr, size);
											if (shared_allocation) {
												pr_debug("%s: sharing bounce buffer for va: 0x%08lx x 0x%08lx",
													__func__, vaddr, size);
												allocation = shared_allocation->allocation;
												xrp_allocation_get(allocation);
												phys = (allocation->start & -align) | offset;
												if (phys < allocation->start)
													phys += align;
											} else {
												rc = xrp_allocate(xvp_file->xvp->pool,
														size + align, align, &allocation);
												if (rc < 0) {
													xrp_unlock_shared_allocations();
													return rc;
												}
												phys = (allocation->start & -align) | offset;
												if (phys < allocation->start)
													phys += align;

												if (flags & XRP_FLAG_READ) {
													if (xrp_copy_user_to_phys(xvp_file->xvp,
																vaddr, size, phys,
																flags, user)) {
														xrp_allocation_put(allocation);
														xrp_unlock_shared_allocations();
														return -EFAULT;
													}
												}
												shared_allocation = xrp_add_shared_allocation(flags, vaddr,
																		size, allocation);
												if (!shared_allocation) {
													xrp_allocation_put(allocation);
													xrp_unlock_shared_allocations();
													return -ENOMEM;
												}
											}

											xrp_unlock_shared_allocations();

											*paddr = phys;
											*mapping = (struct xrp_alien_mapping){
												.vaddr = vaddr,
												.size = size,
												.paddr = *paddr,
												.allocation = allocation,
												.shared_allocation = shared_allocation,
												.type = ALIEN_COPY,
											};
											#ifdef CONFIG_HIFI_XRP_LOG_EN
											pr_debug("%s: copying to pa: %pap\n", __func__, paddr);
											#endif

	return 0;
}

static unsigned xvp_get_region_vma_count(unsigned long virt,
					 unsigned long size,
					 struct vm_area_struct *vma)
{
	unsigned i;
	struct mm_struct *mm = current->mm;

	if (virt + size < virt)
		return 0;
	if (vma->vm_start > virt)
		return 0;
	if (vma->vm_start <= virt &&
	    virt + size <= vma->vm_end)
		return 1;
	for (i = 2; ; ++i) {
		struct vm_area_struct *next_vma = find_vma(mm, vma->vm_end);

		if (!next_vma)
			return 0;
		if (next_vma->vm_start != vma->vm_end)
			return 0;
		vma = next_vma;
		if (virt + size <= vma->vm_end)
			return i;
	}
	return 0;
}

static long xrp_share_kernel(struct file *filp,
			     unsigned long virt, unsigned long size,
			     unsigned long flags, phys_addr_t *paddr,
			     struct xrp_mapping *mapping)
{
	struct xvp_file *xvp_file = filp->private_data;
	struct xvp *xvp = xvp_file->xvp;
	phys_addr_t phys = __pa(virt);
	long err = 0;

#ifdef CONFIG_HIFI_XRP_LOG_EN
	pr_debug("%s: sharing kernel-only buffer: %pap\n", __func__, &phys);
	pr_debug("%s: sharing kernel-only buffer: 0x%llx\n", __func__, phys);
#endif
	if (!xvp->direct_mapping ||
	    xrp_translate_to_dsp(&xvp->address_map, phys) ==
	    XRP_NO_TRANSLATION) {
#ifdef CONFIG_HIFI_XRP_LOG_EN
		pr_debug("%s: untranslatable addr, making shadow copy\n",
			 __func__);
#endif
		err = xvp_copy_virt_to_phys(xvp_file, flags,
					    virt, size, paddr,
					    &mapping->alien_mapping, false);
		mapping->type = XRP_MAPPING_ALIEN | XRP_MAPPING_KERNEL;
	} else {
		mapping->type = XRP_MAPPING_KERNEL;
		*paddr = phys;

		xrp_default_dma_sync_for_device(xvp, phys, size, flags);
	}
#ifdef CONFIG_HIFI_XRP_LOG_EN
	pr_debug("%s: mapping = %p, mapping->type = %d\n",
		 __func__, mapping, mapping->type);
#endif
	return err;
}

static bool vma_needs_cache_ops(struct vm_area_struct *vma)
{
	pgprot_t prot = vma->vm_page_prot;

	return pgprot_val(prot) != pgprot_val(pgprot_noncached(prot)) &&
		pgprot_val(prot) != pgprot_val(pgprot_writecombine(prot));
}

/* Share blocks of memory, from host to IVP or back.
 *
 * When sharing to IVP return physical addresses in paddr.
 * Areas allocated from the driver can always be shared in both directions.
 * Contiguous 3rd party allocations need to be shared to IVP before they can
 * be shared back.
 */

static long __xrp_share_block(struct file *filp,
			      unsigned long virt, unsigned long size,
			      unsigned long flags, phys_addr_t *paddr,
			      struct xrp_mapping *mapping)
{
	phys_addr_t phys = ~0ul;
	struct xvp_file *xvp_file = filp->private_data;
	struct xvp *xvp = xvp_file->xvp;
	struct mm_struct *mm = current->mm;
	struct vm_area_struct *vma;
	bool do_cache = true;
	long rc = -EINVAL;

	if (!xvp->direct_mapping)
		goto no_direct_mapping;

	vma = find_vma(mm, virt);
	if (!vma) {
		pr_debug("%s: no vma for vaddr/size = 0x%08lx/0x%08lx\n",
			 __func__, virt, size);
		return -EINVAL;
	}
	/*
	 * Region requested for sharing should be within single VMA.
	 * That's true for the majority of cases, but sometimes (e.g.
	 * sharing buffer in the beginning of .bss which shares a
	 * file-mapped page with .data, followed by anonymous page)
	 * region will cross multiple VMAs. Support it in the simplest
	 * way possible: start with get_user_pages and use shadow copy
	 * if that fails.
	 */
	switch (xvp_get_region_vma_count(virt, size, vma)) {
	case 0:
		pr_debug("%s: bad vma for vaddr/size = 0x%08lx/0x%08lx\n",
			 __func__, virt, size);
		pr_debug("%s: vma->vm_start = 0x%08lx, vma->vm_end = 0x%08lx\n",
			 __func__, vma->vm_start, vma->vm_end);
		return -EINVAL;
	case 1:
		break;
	default:
		pr_debug("%s: multiple vmas cover vaddr/size = 0x%08lx/0x%08lx\n",
			 __func__, virt, size);
		vma = NULL;
		break;
	}
	/*
	 * And it need to be allocated from the same file descriptor, or
	 * at least from a file descriptor managed by the XRP.
	 */
	if (vma &&
	    (vma->vm_file == filp || xrp_is_known_file(vma->vm_file))) {
		struct xvp_file *vm_file = vma->vm_file->private_data;
		struct xrp_allocation *xrp_allocation = vma->vm_private_data;

		phys = vm_file->xvp->pmem + (vma->vm_pgoff << PAGE_SHIFT) +
			virt - vma->vm_start;
		#ifdef CONFIG_HIFI_XRP_LOG_EN
		pr_debug("%s: XRP allocation at 0x%08lx, phy_addr: 0x%llx\n",
			 __func__, virt, phys);
		#endif
		/*
		 * If it was allocated from a different XRP file it may belong
		 * to a different device and not be directly accessible.
		 * Check if it is.
		 */
		if (vma->vm_file != filp) {

			const struct xrp_address_map_entry *address_map;
			pr_debug("%s: vma->vm_file != filp \n",__func__);
			address_map=xrp_get_address_mapping(&xvp->address_map,
							phys);


			if (!address_map ||
			    xrp_compare_address(phys + size - 1, address_map))
				pr_debug("%s: untranslatable addr: %pap\n",
					 __func__, &phys);
			else
				rc = 0;

		} else {
			rc = 0;
		}

		if (rc == 0) {
			mapping->type = XRP_MAPPING_NATIVE;
			mapping->native.xrp_allocation = xrp_allocation;
			mapping->native.vaddr = virt;
			xrp_allocation_get(xrp_allocation);
			do_cache = vma_needs_cache_ops(vma);
			#ifdef CONFIG_HIFI_XRP_LOG_EN
			pr_debug("%s: do_cache: %d\n",__func__, do_cache);
			#endif
		}
	}
	if (rc < 0) {
		struct xrp_alien_mapping *alien_mapping;
		unsigned long n_pages;
		alien_mapping = &mapping->alien_mapping;
		n_pages = PFN_UP(virt + size) - PFN_DOWN(virt);
		pr_debug("%s: rc < 0 \n",__func__);

		/* Otherwise this is alien allocation. */
		pr_debug("%s: non-XVP allocation at 0x%08lx\n",
			 __func__, virt);

		/*
		 * A range can only be mapped directly if it is either
		 * uncached or HW-specific cache operations can handle it.
		 */
		if (vma && vma->vm_flags & (VM_IO | VM_PFNMAP)) {
			rc = xvp_pfn_virt_to_phys(xvp_file, vma,
						  virt, size,
						  &phys,
						  alien_mapping);
			if (rc == 0 && vma_needs_cache_ops(vma) &&
			    !xrp_cacheable(xvp, PFN_DOWN(phys), n_pages)) {
				pr_debug("%s: needs unsupported cache mgmt\n",
					 __func__);
				rc = -EINVAL;
			}
		} else {
			mmap_read_unlock(mm);
			rc = xvp_gup_virt_to_phys(xvp_file, virt,
						  size, &phys,
						  alien_mapping);
			if (rc == 0 &&
			    (!vma || vma_needs_cache_ops(vma)) &&
			    !xrp_cacheable(xvp, PFN_DOWN(phys), n_pages)) {
				pr_debug("%s: needs unsupported cache mgmt\n",
					 __func__);
				xrp_put_pages(phys, n_pages);
				rc = -EINVAL;
			}
			mmap_read_lock(mm);
		}
		if (rc == 0 && vma && !vma_needs_cache_ops(vma))
			do_cache = false;

		/*
		 * If we couldn't share try to make a shadow copy.
		 */
		if (rc < 0) {
no_direct_mapping:
			alien_mapping = &mapping->alien_mapping;
			rc = xvp_copy_virt_to_phys(xvp_file, flags,
						   virt, size, &phys,
						   alien_mapping, true);
			do_cache = false;
		}

		/* We couldn't share it. Fail the request. */
		if (rc < 0) {
			pr_debug("%s: couldn't map virt to phys\n",
				 __func__);
			return -EINVAL;
		}

		phys = alien_mapping->paddr +
			virt - alien_mapping->vaddr;

		mapping->type = XRP_MAPPING_ALIEN;
	}

	*paddr = phys;
	#ifdef CONFIG_HIFI_XRP_LOG_EN
	pr_debug("%s: mapping = %p, mapping->type = %d\n",
		 __func__, mapping, mapping->type);
	#endif

	if (do_cache) {

		pr_debug("%s: do_cache = %d \n", __func__, do_cache);
		xrp_dma_sync_for_device(xvp,
					virt, phys, size,
					flags);

		#if 0
		u8 *ori = (u8 *)virt;
		for (u32 k = 0; k < 257; k++){
			pr_debug("%d ", ori[k]);
		}
		pr_debug("\n ");
		 

		pr_debug("%s: 0.5 \n", __func__);
		void __iomem *pcm_config = ioremap(0x81900000, 0x1000);
//		void __iomem *pcm_config = ioremap(0x20003014, 0x1000);
		void __iomem *sh_config = ioremap(phys, 0x1000);
		u32 *ori = (u32 *)pcm_config;
		u32 *cp  = (u32 *)sh_config;
		pr_debug("%s: 1 \n", __func__);
		for (u32 k = 0; k < (0xf0/4); k++){
			pr_debug("0x%08x ",*(ori+k));
		}
		pr_debug("%s: 1 \n", __func__);
		xrp_comm_read(pcm_config, (void *)sh_config,0xf0);
// 		copy_to_user((void __user *)(unsigned long)virt,pcm_config,0xf0);
		pr_debug("%s: 1.5 \n", __func__);
		*cp = 0xff;
		pr_debug("%s: 2 \n", __func__);


		for (u32 j = 0; j < (0xf0/4); j++){
			pr_debug("0x%04x-0x%04x \n",*(ori+j), *(cp+j));
		}
		pr_debug("%s: 3 \n", __func__);

		#endif

	}

	return 0;
}

static long xrp_writeback_alien_mapping(struct xvp_file *xvp_file,
					struct xrp_alien_mapping *alien_mapping,
					unsigned long flags, bool user)
{
	struct page *page;
	size_t nr_pages;
	size_t i;
	long ret = 0;

	switch (alien_mapping->type) {
	case ALIEN_GUP:
		xrp_dma_sync_for_cpu(xvp_file->xvp,
				     alien_mapping->vaddr,
				     alien_mapping->paddr,
				     alien_mapping->size,
				     flags);
		pr_debug("%s: dirtying alien GUP @va = %p, pa = %pap\n",
			 __func__, (void __user *)alien_mapping->vaddr,
			 &alien_mapping->paddr);
		page = pfn_to_page(__phys_to_pfn(alien_mapping->paddr));
		nr_pages = PFN_UP(alien_mapping->vaddr + alien_mapping->size) -
			PFN_DOWN(alien_mapping->vaddr);
		for (i = 0; i < nr_pages; ++i)
			SetPageDirty(page + i);
		break;

	case ALIEN_COPY:
		#ifdef CONFIG_HIFI_XRP_LOG_EN
		pr_debug("%s: synchronizing alien copy @pa = %pap back to %p\n",
			 __func__, &alien_mapping->paddr,
			 (void __user *)alien_mapping->vaddr);
		#endif
		if (xrp_copy_user_from_phys(xvp_file->xvp,
					    alien_mapping->vaddr,
					    alien_mapping->size,
					    alien_mapping->paddr,
					    flags, user))
			ret = -EINVAL;
		break;

	default:
		break;
	}
	return ret;
}

/*
 *
 */
static long __xrp_unshare_block(struct file *filp, struct xrp_mapping *mapping,
				unsigned long flags)
{
	long ret = 0;

	switch (mapping->type & ~XRP_MAPPING_KERNEL) {
	case XRP_MAPPING_NATIVE:
		if (flags & XRP_FLAG_WRITE) {
//			struct xvp_file *xvp_file = filp->private_data;
			#ifdef CONFIG_HIFI_XRP_LOG_EN
			pr_debug("%s: xrp_dma_sync_for_cpu\n",__func__);
			#endif
#if 0
			xrp_dma_sync_for_cpu(xvp_file->xvp,
					     mapping->native.vaddr,
					     mapping->native.xrp_allocation->start,
					     mapping->native.xrp_allocation->size,
					     flags);
#endif

		}
		xrp_allocation_put(mapping->native.xrp_allocation);
		break;

	case XRP_MAPPING_ALIEN:
		if (flags & XRP_FLAG_WRITE)
			ret = xrp_writeback_alien_mapping(filp->private_data,
							  &mapping->alien_mapping,
							  flags,
							  !(mapping->type & XRP_MAPPING_KERNEL));

		xrp_alien_mapping_destroy(&mapping->alien_mapping);
		break;

	case XRP_MAPPING_KERNEL:
		break;

	default:
		break;
	}

	mapping->type = XRP_MAPPING_NONE;

	return ret;
}

static long xrp_ioctl_free(struct file *filp,
			   struct xrp_ioctl_alloc __user *p)
{
	struct xvp_file *xvp_file = filp->private_data;
	struct mm_struct *mm = current->mm;
	struct xrp_ioctl_alloc xrp_ioctl_alloc;
	struct vm_area_struct *vma;
	unsigned long start;
	size_t size;

	#ifdef CONFIG_HIFI_XRP_LOG_EN
	pr_debug("%s: %p\n", __func__, p);
	#endif

	if (copy_from_user(&xrp_ioctl_alloc, p, sizeof(*p)))
		return -EFAULT;

	start = xrp_ioctl_alloc.addr;

	#ifdef CONFIG_HIFI_XRP_LOG_EN
	pr_debug("%s: virt_addr = 0x%08lx\n", __func__, start);
	#endif

	if (xvp_file->xvp->direct_mapping) {
		mmap_read_lock(mm);
		vma = find_vma(mm, start);

		if (vma && vma->vm_file == filp &&
		    vma->vm_start <= start && start < vma->vm_end) {
			start = vma->vm_start;
			size = vma->vm_end - vma->vm_start;
			mmap_read_unlock(mm);
			#ifdef CONFIG_HIFI_XRP_LOG_EN
			pr_debug("%s: 0x%lx x %zu\n", __func__, start, size);
			#endif
			return vm_munmap(start, size);
		}
		pr_debug("%s: no vma/bad vma for vaddr = 0x%08lx\n", __func__, start);
		mmap_read_unlock(mm);

		return -EINVAL;
	} else {
		size = xrp_ioctl_alloc.size;
		return vm_munmap(start, size);
	}
}

static long xvp_complete_cmd_irq(struct xvp *xvp, struct xrp_comm *comm,
				 bool (*cmd_complete)(struct xvp *xvp,
						      struct xrp_comm *p))
{
	long timeout = firmware_command_timeout * HZ;

	if (cmd_complete(xvp, comm))
		return 0;
	if (xrp_panic_check(xvp))
		return -EBUSY;
	do {
		timeout = wait_for_completion_interruptible_timeout(&comm->completion,
								    timeout);
		if (cmd_complete(xvp, comm))
			return 0;
		if (xrp_panic_check(xvp))
			return -EBUSY;
	} while (timeout > 0);

	if (timeout == 0)
		return -EBUSY;
	return timeout;
}

static long xvp_complete_cmd_poll(struct xvp *xvp, struct xrp_comm *comm,
				  bool (*cmd_complete)(struct xvp *xvp,
						       struct xrp_comm *p))
{
	unsigned long deadline = jiffies + firmware_command_timeout * HZ;

	do {
		if (cmd_complete(xvp, comm))
			return 0;
		if (xrp_panic_check(xvp))
			return -EBUSY;
		schedule();
	} while (time_before(jiffies, deadline));

	return -EBUSY;
}

struct xrp_request {
	struct xrp_ioctl_queue ioctl_queue;
	size_t n_buffers;
	struct xrp_mapping *buffer_mapping;
	struct xrp_dsp_buffer *dsp_buffer;
	phys_addr_t in_data_phys;
	phys_addr_t out_data_phys;
	phys_addr_t dsp_buffer_phys;
	union {
		struct xrp_mapping in_data_mapping;
		u8 in_data[XRP_DSP_CMD_INLINE_DATA_SIZE];
	};
	union {
		struct xrp_mapping out_data_mapping;
		u8 out_data[XRP_DSP_CMD_INLINE_DATA_SIZE];
	};
	union {
		struct xrp_mapping dsp_buffer_mapping;
		struct xrp_dsp_buffer buffer_data[XRP_DSP_CMD_INLINE_BUFFER_COUNT];
	};
	u8 nsid[XRP_DSP_CMD_NAMESPACE_ID_SIZE];
};

static void xrp_unmap_request_nowb(struct file *filp, struct xrp_request *rq)
{
	struct xvp_file *xvp_file = filp->private_data;
	struct xvp *xvp = xvp_file->xvp;
	size_t n_buffers = rq->n_buffers;
	size_t i;

	if (rq->ioctl_queue.in_data_size > XRP_DSP_CMD_INLINE_DATA_SIZE)
		__xrp_unshare_block(filp, &rq->in_data_mapping, 0);
	if (rq->ioctl_queue.out_data_size > XRP_DSP_CMD_INLINE_DATA_SIZE)
		__xrp_unshare_block(filp, &rq->out_data_mapping, 0);
	for (i = 0; i < n_buffers; ++i)
		__xrp_unshare_block(filp, rq->buffer_mapping + i, 0);
	if (n_buffers > XRP_DSP_CMD_INLINE_BUFFER_COUNT)
		__xrp_unshare_block(filp, &rq->dsp_buffer_mapping, 0);

	if (n_buffers) {
		kfree(rq->buffer_mapping);
		if (n_buffers > XRP_DSP_CMD_INLINE_BUFFER_COUNT) {
			xrp_free_host(xvp, rq->dsp_buffer);
		}
	}
}

static long xrp_unmap_request(struct file *filp, struct xrp_request *rq)
{
	struct xvp_file *xvp_file = filp->private_data;
	struct xvp *xvp = xvp_file->xvp;
	size_t n_buffers = rq->n_buffers;
	size_t i;
	long ret = 0;
	long rc;

	if (rq->ioctl_queue.in_data_size > XRP_DSP_CMD_INLINE_DATA_SIZE) {
		#ifdef CONFIG_HIFI_XRP_LOG_EN
		pr_debug("%s: in_data, __xrp_unshare_block\n",__func__);
		#endif
		__xrp_unshare_block(filp, &rq->in_data_mapping, XRP_FLAG_READ);
		#ifdef CONFIG_HIFI_XRP_LOG_EN
		pr_debug("%s: in_data, __xrp_unshare_block done \n",__func__);
		#endif
	}
	if (rq->ioctl_queue.out_data_size > XRP_DSP_CMD_INLINE_DATA_SIZE) {
		#ifdef CONFIG_HIFI_XRP_LOG_EN
		pr_debug("%s: out_data, __xrp_unshare_block\n",__func__);
		#endif
		rc = __xrp_unshare_block(filp, &rq->out_data_mapping,
					 XRP_FLAG_WRITE);
		#ifdef CONFIG_HIFI_XRP_LOG_EN
		pr_debug("%s: out_data, __xrp_unshare_block done \n",__func__);
		#endif

		if (rc < 0) {
			pr_debug("%s: out_data could not be unshared\n",
				 __func__);
			ret = rc;
		}
	} else {
		if (copy_to_user((void __user *)(unsigned long)rq->ioctl_queue.out_data_addr,
				 rq->out_data,
				 rq->ioctl_queue.out_data_size)) {
			pr_debug("%s: out_data could not be copied\n",
				 __func__);
			ret = -EFAULT;
		}
	}

	if (n_buffers > XRP_DSP_CMD_INLINE_BUFFER_COUNT)
		__xrp_unshare_block(filp, &rq->dsp_buffer_mapping,
				    XRP_FLAG_READ_WRITE);

	for (i = 0; i < n_buffers; ++i) {
		#ifdef CONFIG_HIFI_XRP_LOG_EN
		pr_debug("%s: dsp_buffer, __xrp_unshare_block\n",__func__);
		#endif
		rc = __xrp_unshare_block(filp, rq->buffer_mapping + i,
					 rq->dsp_buffer[i].flags);
		#ifdef CONFIG_HIFI_XRP_LOG_EN
		pr_debug("%s: dsp_buffer, __xrp_unshare_block done \n",__func__);
		#endif
		if (rc < 0) {
			pr_debug("%s: buffer %zd could not be unshared\n",
				 __func__, i);
			ret = rc;
		}
	}

	if (n_buffers) {
		kfree(rq->buffer_mapping);
		if (n_buffers > XRP_DSP_CMD_INLINE_BUFFER_COUNT) {
			xrp_free_host(xvp, rq->dsp_buffer);
		}
		rq->n_buffers = 0;
	}

	return ret;
}

static long xrp_map_request(struct file *filp, struct xrp_request *rq,
			    struct mm_struct *mm)
{
	struct xvp_file *xvp_file = filp->private_data;
	struct xvp *xvp = xvp_file->xvp;
	struct xrp_ioctl_buffer __user *buffer;
	size_t n_buffers = rq->ioctl_queue.buffer_size /
		sizeof(struct xrp_ioctl_buffer);

	size_t i;
	long ret = 0;

	if ((rq->ioctl_queue.flags & XRP_QUEUE_FLAG_NSID) &&
	    copy_from_user(rq->nsid,
			   (void __user *)(unsigned long)rq->ioctl_queue.nsid_addr,
			   sizeof(rq->nsid))) {
		pr_debug("%s: nsid could not be copied\n ", __func__);
		return -EINVAL;
	}
	rq->n_buffers = n_buffers;
	if (n_buffers) {
		rq->buffer_mapping =
			kzalloc(n_buffers * sizeof(*rq->buffer_mapping),
				GFP_KERNEL);
		if (n_buffers > XRP_DSP_CMD_INLINE_BUFFER_COUNT) {
			rq->dsp_buffer =
				xrp_alloc_host(xvp,
					       n_buffers * sizeof(*rq->dsp_buffer));
			if (!rq->dsp_buffer) {
				kfree(rq->buffer_mapping);
				return -ENOMEM;
			}
		} else {
			rq->dsp_buffer = rq->buffer_data;
		}
	}

	mmap_read_lock(mm);

	if (rq->ioctl_queue.in_data_size > XRP_DSP_CMD_INLINE_DATA_SIZE) {
		#ifdef CONFIG_HIFI_XRP_LOG_EN
		pr_debug("%s: in_data __xrp_share_block\n", __func__);
		#endif
		ret = __xrp_share_block(filp, rq->ioctl_queue.in_data_addr,
					rq->ioctl_queue.in_data_size,
					XRP_FLAG_READ, &rq->in_data_phys,
					&rq->in_data_mapping);
		if(ret < 0) {
			pr_debug("%s: in_data could not be shared\n",
				 __func__);
			goto share_err;
		}
	} else {
		if (copy_from_user(rq->in_data,
				   (void __user *)(unsigned long)rq->ioctl_queue.in_data_addr,
				   rq->ioctl_queue.in_data_size)) {
			pr_debug("%s: in_data could not be copied\n",
				 __func__);
			ret = -EFAULT;
			goto share_err;
		}
	}

	if (rq->ioctl_queue.out_data_size > XRP_DSP_CMD_INLINE_DATA_SIZE) {
		#ifdef CONFIG_HIFI_XRP_LOG_EN
		pr_debug("%s: out_data __xrp_share_block\n", __func__);
		#endif
		ret = __xrp_share_block(filp, rq->ioctl_queue.out_data_addr,
					rq->ioctl_queue.out_data_size,
					XRP_FLAG_WRITE, &rq->out_data_phys,
					&rq->out_data_mapping);
		if (ret < 0) {
			pr_debug("%s: out_data could not be shared\n",
				 __func__);
			goto share_err;
		}
	}

	buffer = (void __user *)(unsigned long)rq->ioctl_queue.buffer_addr;

	for (i = 0; i < n_buffers; ++i) {
		struct xrp_ioctl_buffer ioctl_buffer;
		phys_addr_t buffer_phys = ~0ul;

		if (copy_from_user(&ioctl_buffer, buffer + i,
				   sizeof(ioctl_buffer))) {
			ret = -EFAULT;
			goto share_err;
		}
		if (ioctl_buffer.flags & XRP_FLAG_READ_WRITE) {
			#ifdef CONFIG_HIFI_XRP_LOG_EN
			pr_debug("%s: ioctl_buffer __xrp_share_block\n", __func__);
			#endif
			ret = __xrp_share_block(filp, ioctl_buffer.addr,
						ioctl_buffer.size,
						ioctl_buffer.flags,
						&buffer_phys,
						rq->buffer_mapping + i);
			if (ret < 0) {
				pr_debug("%s: buffer %zd could not be shared\n",
					 __func__, i);
				goto share_err;
			}
		}

		rq->dsp_buffer[i] = (struct xrp_dsp_buffer){
			.flags = ioctl_buffer.flags,
			.size = ioctl_buffer.size,
			.addr = xrp_translate_to_dsp(&xvp->address_map,
						     buffer_phys),
		};
		#ifdef CONFIG_HIFI_XRP_LOG_EN
		pr_debug("%s: rq->dsp_buffer[i].addr = 0x%x , rq->dsp_buffer[i].size = 0x%x \n", __func__,  rq->dsp_buffer[i].addr, rq->dsp_buffer[i].size);
		#endif
	}

	if (n_buffers > XRP_DSP_CMD_INLINE_BUFFER_COUNT) {
		#ifdef CONFIG_HIFI_XRP_LOG_EN
		pr_debug("%s: dsp_buffer xrp_share_kernel\n", __func__);
		#endif
		ret = xrp_share_kernel(filp, (unsigned long)rq->dsp_buffer,
				       n_buffers * sizeof(*rq->dsp_buffer),
				       XRP_FLAG_READ_WRITE, &rq->dsp_buffer_phys,
				       &rq->dsp_buffer_mapping);
		#ifdef CONFIG_HIFI_XRP_LOG_EN
		pr_debug("%s: rq->dsp_buffer_phys : 0x%llx \n", __func__, rq->dsp_buffer_phys);
		#endif
		if(ret < 0) {
			pr_debug("%s: buffer descriptors could not be shared\n",
				 __func__);
			goto share_err;
		}
	}
share_err:
	mmap_read_unlock(mm);
	if (ret < 0)
		xrp_unmap_request_nowb(filp, rq);
	return ret;
}

static void xrp_fill_hw_request(struct xvp *xvp,
				struct xrp_dsp_cmd __iomem *cmd,
				struct xrp_request *rq,
				const struct xrp_address_map *map)
{
	struct xrp_dsp_cmd dsp_cmd;

	xrp_comm_write32(&cmd->in_data_size, rq->ioctl_queue.in_data_size);
	xrp_comm_write32(&cmd->out_data_size, rq->ioctl_queue.out_data_size);
	xrp_comm_write32(&cmd->buffer_size,
			 rq->n_buffers * sizeof(struct xrp_dsp_buffer));

#ifdef CONFIG_HIFI_XRP_LOG_EN
	pr_debug("%s: in_data_size= 0x%x, out_data_size= 0x%x, buffer_size= 0x%x, n_buffers= 0x%lx \n", __func__, cmd->in_data_size, cmd->out_data_size, cmd->buffer_size,  rq->n_buffers);
#endif

	if (rq->ioctl_queue.in_data_size > XRP_DSP_CMD_INLINE_DATA_SIZE)
		xrp_comm_write32(&cmd->in_data_addr,
				 xrp_translate_to_dsp(map, rq->in_data_phys));
	else
		xrp_comm_write(&cmd->in_data, rq->in_data,
			       rq->ioctl_queue.in_data_size);
#ifdef CONFIG_HIFI_XRP_LOG_EN
	pr_debug("%s: in_data_addr= 0x%x \n", __func__, cmd->in_data_addr);
#endif

	if (rq->ioctl_queue.out_data_size > XRP_DSP_CMD_INLINE_DATA_SIZE)
		xrp_comm_write32(&cmd->out_data_addr,
				 xrp_translate_to_dsp(map, rq->out_data_phys));

#ifdef CONFIG_HIFI_XRP_LOG_EN
	pr_debug("%s: out_data_addr= 0x%x \n", __func__, cmd->out_data_addr);
#endif

	if (rq->n_buffers > XRP_DSP_CMD_INLINE_BUFFER_COUNT)
		xrp_comm_write32(&cmd->buffer_addr,
				 xrp_translate_to_dsp(map, rq->dsp_buffer_phys));
	else
		xrp_comm_write(&cmd->buffer_data, rq->dsp_buffer,
			       rq->n_buffers * sizeof(struct xrp_dsp_buffer));
#ifdef CONFIG_HIFI_XRP_LOG_EN
	pr_debug("%s: buffer_addr= 0x%x \n", __func__, cmd->buffer_addr);
#endif

	if (rq->ioctl_queue.flags & XRP_QUEUE_FLAG_NSID)
		xrp_comm_write(&cmd->nsid, rq->nsid, sizeof(rq->nsid));


	xrp_comm_copy_to_alloc(xvp, cmd, sizeof(*cmd));
	wmb();
	/* update flags */
	xrp_comm_copy_write32(xvp, &cmd->flags,
			      (rq->ioctl_queue.flags & ~XRP_DSP_CMD_FLAG_RESPONSE_VALID) |
			      XRP_DSP_CMD_FLAG_REQUEST_VALID);

	xrp_comm_read(cmd, &dsp_cmd, sizeof(dsp_cmd));

#ifdef CONFIG_HIFI_XRP_LOG_EN
	pr_debug("%s: cmd for DSP: %p: %*ph\n",__func__, cmd,(int)sizeof(dsp_cmd), &dsp_cmd);
#endif

}

static long xrp_complete_hw_request(struct xrp_dsp_cmd __iomem *cmd,
				    struct xrp_request *rq)
{
	u32 flags = xrp_comm_read32(&cmd->flags);

	if (rq->ioctl_queue.out_data_size <= XRP_DSP_CMD_INLINE_DATA_SIZE)
		xrp_comm_read(&cmd->out_data, rq->out_data,
			      rq->ioctl_queue.out_data_size);
	if (rq->n_buffers <= XRP_DSP_CMD_INLINE_BUFFER_COUNT)
		xrp_comm_read(&cmd->buffer_data, rq->dsp_buffer,
			      rq->n_buffers * sizeof(struct xrp_dsp_buffer));
	xrp_comm_write32(&cmd->flags, 0);

	return (flags & XRP_DSP_CMD_FLAG_RESPONSE_DELIVERY_FAIL) ? -ENXIO : 0;
}

static long xrp_ioctl_submit_sync(struct file *filp,
				  struct xrp_ioctl_queue __user *p)
{
	struct xvp_file *xvp_file = filp->private_data;
	struct xvp *xvp = xvp_file->xvp;
	struct xrp_comm *queue = xvp->queue;
	struct xrp_request *rq;
	long ret = 0;
	bool went_off = false;

	rq = xrp_alloc_host(xvp, sizeof(struct xrp_request));
	if (!rq)
		return -ENOMEM;

	if (copy_from_user(&rq->ioctl_queue, p, sizeof(*p))) {
		ret =-EFAULT;
		goto err;
	}

	if (rq->ioctl_queue.flags & ~XRP_QUEUE_VALID_FLAGS) {
		dev_dbg(xvp->dev, "%s: invalid flags 0x%08x\n",
			__func__, rq->ioctl_queue.flags);
		ret = -EINVAL;
		goto err;
	}

	if (xvp->n_queues > 1) {
		unsigned n = (rq->ioctl_queue.flags & XRP_QUEUE_FLAG_PRIO) >>
			XRP_QUEUE_FLAG_PRIO_SHIFT;

		if (n >= xvp->n_queues)
			n = xvp->n_queues - 1;
		queue = xvp->queue_ordered[n];
		dev_dbg(xvp->dev, "%s: priority: %d -> %d\n",
			__func__, n, queue->priority);
	}

	ret = xrp_map_request(filp, rq, current->mm);
	if (ret < 0)
		goto err;

#ifdef CONFIG_HIFI_XRP_LOG_EN
	pr_debug("%s: loopback= %d\n", __func__, loopback);
#endif
//	loopback = 0;
//	if (loopback < LOOPBACK_NOIO) {
	if (loopback == LOOPBACK_BSTONLY) {

		int reboot_cycle;
retry:
		mutex_lock(&queue->lock);
		reboot_cycle = atomic_read(&xvp->reboot_cycle);
		if (reboot_cycle != atomic_read(&xvp->reboot_cycle_complete)) {
			mutex_unlock(&queue->lock);
			goto retry;
		}

#ifdef CONFIG_HIFI_XRP_LOG_EN
		pr_debug("%s: xvp->off= %d\n", __func__, xvp->off);
#endif

		if (xvp->off) {
			ret = -ENODEV;
		} else {

			audio_ipc_XrpDspCmd_t in_cmd = {0};
			audio_ipc_XrpDspCmd_t out_cmd = {0};
			audio_ipc_ErrorEnum_t err = AUDIO_IPC_NO_ERROR;
			des_buf_t *ext_buf;
			struct xrp_dsp_cmd *dsp_cmd;
			uint8_t in_data[1] = {1};
			uint8_t out_data[1] = {1};
			uint8_t buffer_data[1] = {1};
			uint8_t buffer_alignment[1] = {1};
			uint8_t nsid[1] = {1};

			xrp_fill_hw_request(xvp, queue->comm, rq,
					    &xvp->address_map);

//			pr_debug("%s: device_irq_mode= %d\n", __func__, hw->device_irq_mode);
			xrp_send_device_irq(xvp);

			//send sharm mem by mesgbox

			ext_buf =kzalloc(sizeof(des_buf_t),GFP_KERNEL);
			dsp_cmd = (struct xrp_dsp_cmd *)queue->comm;
			in_cmd.flags = dsp_cmd->flags;
			in_cmd.in_data_size = dsp_cmd->in_data_size;
			in_cmd.out_data_size = dsp_cmd->out_data_size;
			in_cmd.buffer_size = dsp_cmd->buffer_size;
			in_cmd.in_data_addr = dsp_cmd->in_data_addr;
			in_cmd.out_data_addr = dsp_cmd->out_data_addr;
			in_cmd.buffer_addr = dsp_cmd->buffer_addr;

//			uint8_t in_data[1] = {1};
			in_cmd.in_data.data = in_data;
			in_cmd.in_data.size = 1;

//			uint8_t out_data[1] = {1};
			in_cmd.out_data.data = out_data;
			in_cmd.out_data.size = 1;

//			uint8_t buffer_data[1] = {1};
			in_cmd.buffer_data.data = buffer_data;
			in_cmd.buffer_data.size = 1;

//			uint8_t buffer_alignment[1] = {1};
			in_cmd.buffer_alignment.data = buffer_alignment;
			in_cmd.buffer_alignment.size = 1;

//			uint8_t nsid[1] = {1};
			in_cmd.nsid.data = nsid;
			in_cmd.nsid.size = 1;

#ifdef CONFIG_HIFI_XRP_LOG_EN
			pr_debug("%s: call xrp_shmem_addr_method_sync \n", __func__);
#endif
			ret =xvp->xrp_ipc->audio_ipc_client.xrp_shmem_addr_method_sync(in_cmd, &out_cmd, &err, 0, ext_buf);

			if (ret < 0)
			{
				pr_debug("%s: send xrp_shmem_addr_method_sync failed \n", __func__);
			}
#ifdef CONFIG_HIFI_XRP_LOG_EN
   		 	pr_debug("%s, out_cmd -- flags: 0x%x, in_data_size: 0x%x, out_data_size: 0x%x, buffer_size: 0x%x, in_data_addr: 0x%x, out_data_addr: 0x%x, buffer_addr: 0x%x \n", __func__,
        	out_cmd.flags, out_cmd.in_data_size, out_cmd.out_data_size, out_cmd.buffer_size, out_cmd.in_data_addr, out_cmd.out_data_addr,out_cmd.buffer_addr);

			pr_debug("%s: host_irq_mode= %d\n", __func__, xvp->host_irq_mode);
#endif

			if (xvp->host_irq_mode) {
				pr_debug("%s: wait for irq\n", __func__);
				ret = xvp_complete_cmd_irq(xvp, queue,
							   xrp_cmd_complete);
			} else {
				#ifdef CONFIG_HIFI_XRP_LOG_EN
				pr_debug("%s: wait for poll\n", __func__);
				#endif
				ret = xvp_complete_cmd_poll(xvp, queue,
							    xrp_cmd_complete);
			}

			if (xrp_panic_check(xvp) != 0) {
				mutex_unlock(&queue->lock);
				xrp_unmap_request_nowb(filp, rq);
				ret = -EBUSY;
				goto err;
			}

			/* copy back inline data */
			if (ret == 0) {
				#ifdef CONFIG_HIFI_XRP_LOG_EN
				pr_debug("%s: xrp_complete_hw_request\n", __func__);
				#endif
				ret = xrp_complete_hw_request(queue->comm, rq);
			} else if (ret == -EBUSY && firmware_reboot &&
				   atomic_inc_return(&xvp->reboot_cycle) ==
				   reboot_cycle + 1) {
				int rc;
				unsigned i;
				pr_debug("%s: restarting firmware\n", __func__);
				dev_dbg(xvp->dev,
					"%s: restarting firmware...\n",
					 __func__);
				for (i = 0; i < xvp->n_queues; ++i)
					if (xvp->queue + i != queue)
						mutex_lock(&xvp->queue[i].lock);
				rc = xrp_boot_firmware(xvp);
				atomic_set(&xvp->reboot_cycle_complete,
					   atomic_read(&xvp->reboot_cycle));
				for (i = 0; i < xvp->n_queues; ++i)
					if (xvp->queue + i != queue)
						mutex_unlock(&xvp->queue[i].lock);
				if (rc < 0) {
					ret = rc;
					went_off = xvp->off;
				}
			}
		}
		mutex_unlock(&queue->lock);
	}

//	ret = 0;

	if (ret == 0) {
		#ifdef CONFIG_HIFI_XRP_LOG_EN
		pr_debug("%s: xrp_unmap_request\n", __func__);
		#endif
		ret = xrp_unmap_request(filp, rq);
	}
	else if (!went_off) {
		pr_debug("%s: xrp_unmap_request_nowb\n", __func__);
		xrp_unmap_request_nowb(filp, rq);
	}

	/*
	 * Otherwise (if the DSP went off) all mapped buffers are leaked here.
	 * There seems to be no way to recover them as we don't know what's
	 * going on with the DSP; the DSP may still be reading and writing
	 * this memory.
	 */
err:
	xrp_free_host(xvp, rq);

	return ret;
}

static long xvp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long retval = 0;

#ifdef CONFIG_HIFI_XRP_LOG_EN
	pr_debug("%s: %x\n", __func__, cmd);
#endif

	switch(cmd){
	case XRP_IOCTL_ALLOC:
		retval = xrp_ioctl_alloc(filp,
					 (struct xrp_ioctl_alloc __user *)arg);
		break;

	case XRP_IOCTL_FREE:
		retval = xrp_ioctl_free(filp,
					(struct xrp_ioctl_alloc __user *)arg);
		break;

	case XRP_IOCTL_QUEUE:
	case XRP_IOCTL_QUEUE_NS:
		retval = xrp_ioctl_submit_sync(filp,
					       (struct xrp_ioctl_queue __user *)arg);
		break;

	case XRP_IOCTL_ALLOC_SYNC_CMD:
		retval = xrp_ioctl_alloc_sync_cmd(filp,
					       (struct xrp_sync_cmd __user *)arg,0);
		break;
	case XRP_IOCTL_SYNC_CMD:
		retval = xrp_ioctl_sync_cmd(filp,
					       (struct xrp_sync_cmd __user *)arg,0);
		break;
	case XRP_IOCTL_REGISTER_CB:

		break;
	case XRP_IOCTL_DEF_ALG_SET:
	case XRP_IOCTL_USERDEF_ALG_SET:
		retval = xrp_ioctl_def_alg_set(filp,
					       (struct xrp_alg_param __user *)arg,0);
		break;
	case XRP_IOCTL_ALG_RESULT_GET:
	case XRP_IOCTL_ALG_FLUSH:
		retval = xrp_ioctl_alg_result_get(filp,
					       (struct xrp_output __user *)arg,0);
		break;
	case XRP_IOCTL_ALLOC_NSID:
		retval = xrp_ioctl_nsid_get(filp,
					       (struct xrp_nsid __user *)arg,0);
		break;
	case XRP_IOCTL_SYNC_CMD_PIO:
		retval = xrp_ioctl_sync_cmd(filp,
					       (struct xrp_sync_cmd __user *)arg,1);
		break;
	case XRP_IOCTL_DEF_ALG_SET_PIO:
		retval = xrp_ioctl_def_alg_set(filp,
					    (struct xrp_alg_param __user *)arg,1);
		break;
	case XRP_IOCTL_ALG_RESULT_GET_PIO:
		retval = xrp_ioctl_alg_result_get(filp,
					       (struct xrp_output __user *)arg,1);
		break;
	case XRP_IOCTL_ALG_FLUSH_PIO:
		retval = xrp_ioctl_alg_result_get(filp,
					       (struct xrp_output __user *)arg,1);
		break;
	default:
		retval = -EINVAL;
		break;
	}
	return retval;
}

static void xvp_vm_open(struct vm_area_struct *vma)
{
	#ifdef CONFIG_HIFI_XRP_LOG_EN
	struct xrp_allocation *xrp_allocation = vma->vm_private_data;
	pr_debug("%s\n", __func__);
	pr_debug("%s: start:0x%llx, size: 0x%x\n", __func__, xrp_allocation->start, xrp_allocation->size);
	#endif
	xrp_allocation_get(vma->vm_private_data);
}

static void xvp_vm_close(struct vm_area_struct *vma)
{
	#ifdef CONFIG_HIFI_XRP_LOG_EN
	struct xrp_allocation *xrp_allocation = vma->vm_private_data;
	pr_debug("%s\n", __func__);
	pr_debug("%s: start:0x%llx, size: 0x%x\n", __func__, xrp_allocation->start, xrp_allocation->size);
	#endif
	xrp_allocation_put(vma->vm_private_data);
}

static const struct vm_operations_struct xvp_vm_ops = {
	.open = xvp_vm_open,
	.close = xvp_vm_close,
};

static int xvp_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int err;
	struct xvp_file *xvp_file = filp->private_data;
	unsigned long pfn = vma->vm_pgoff + PFN_DOWN(xvp_file->xvp->pmem);
	struct xrp_allocation *xrp_allocation;

#ifdef CONFIG_HIFI_XRP_LOG_EN
	pr_debug("%s\n", __func__);
	pr_debug("%s: vm_pgoff : 0x%lx \n", __func__, (vma->vm_pgoff) << PAGE_SHIFT);
	pr_debug("%s: vm_start : 0x%lx \n", __func__, vma->vm_start);
#endif
	xrp_allocation = xrp_allocation_dequeue(filp->private_data,
						pfn << PAGE_SHIFT,
						vma->vm_end - vma->vm_start);
	if (xrp_allocation) {
		struct xvp *xvp = xvp_file->xvp;
		pgprot_t prot = vma->vm_page_prot;
		if (!xrp_cacheable(xvp, pfn,
				   PFN_DOWN(vma->vm_end - vma->vm_start))) {
			#ifdef CONFIG_HIFI_XRP_LOG_EN
			pr_debug("%s pgprot_writecombine \n", __func__);
			#endif
			prot = pgprot_writecombine(prot);
			vma->vm_page_prot = prot;
		}

		err = remap_pfn_range(vma, vma->vm_start, pfn,
				      vma->vm_end - vma->vm_start,
				      prot);
		#ifdef CONFIG_HIFI_XRP_LOG_EN
		pr_debug("%s err:%d \n", __func__, err);
		#endif
		vma->vm_private_data = xrp_allocation;
		vma->vm_ops = &xvp_vm_ops;
	} else {
		err = -EINVAL;
	}

	return err;
}

static int xvp_open(struct inode *inode, struct file *filp)
{
	struct xvp *xvp = container_of(filp->private_data,
				       struct xvp, miscdev);
	struct xvp_file *xvp_file;
	int rc;

#ifdef CONFIG_HIFI_XRP_LOG_EN
	pr_debug("%s\n", __func__);
#endif

	rc = pm_runtime_get_sync(xvp->dev);
	if (rc < 0)
		return rc;

	xvp_file = devm_kzalloc(xvp->dev, sizeof(*xvp_file), GFP_KERNEL);
	if (!xvp_file) {
		pr_debug("%s 2\n", __func__);
		pm_runtime_put_sync(xvp->dev);
		return -ENOMEM;
	}

	xvp_file->xvp = xvp;
	spin_lock_init(&xvp_file->busy_list_lock);
	filp->private_data = xvp_file;
	xrp_add_known_file(filp);
	return 0;
}

static int xvp_close(struct inode *inode, struct file *filp)
{
	struct xvp_file *xvp_file = filp->private_data;

#ifdef CONFIG_HIFI_XRP_LOG_EN
	pr_debug("%s\n", __func__);
#endif

	xrp_remove_known_file(filp);
	pm_runtime_put_sync(xvp_file->xvp->dev);
	devm_kfree(xvp_file->xvp->dev, xvp_file);
	return 0;
}

static inline int xvp_enable_dsp(struct xvp *xvp)
{
	if (loopback < LOOPBACK_NOMMIO &&
	    xvp->hw_ops->enable)
		return xvp->hw_ops->enable(xvp->hw_arg);
	else if(loopback == LOOPBACK_BSTONLY &&xvp->hw_ops->enable)
	{
		return xvp->hw_ops->enable(xvp->hw_arg);
	}
	else
		return 0;
}

static inline void xvp_disable_dsp(struct xvp *xvp)
{
	if (loopback < LOOPBACK_NOMMIO &&xvp->hw_ops->disable)
	{
		xvp->hw_ops->disable(xvp->hw_arg);
	}
	else if(loopback == LOOPBACK_BSTONLY &&xvp->hw_ops->disable)
	{
		xvp->hw_ops->disable(xvp->hw_arg);
	}
}

static inline void xrp_reset_dsp(struct xvp *xvp)
{
	if (loopback < LOOPBACK_NOMMIO &&xvp->hw_ops->reset)
	{
		xvp->hw_ops->reset(xvp->hw_arg);
	}
	else if(loopback == LOOPBACK_BSTONLY &&xvp->hw_ops->reset)
	{
		xvp->hw_ops->reset(xvp->hw_arg);
	}
}

static inline void xrp_halt_dsp(struct xvp *xvp)
{
	if (loopback < LOOPBACK_NOMMIO && xvp->hw_ops->halt)
	{
		xvp->hw_ops->halt(xvp->hw_arg);
	}
	else if(loopback == LOOPBACK_BSTONLY &&xvp->hw_ops->halt)
	{
		xvp->hw_ops->halt(xvp->hw_arg);
	}
}

static inline void xrp_release_dsp(struct xvp *xvp)
{
	if (loopback < LOOPBACK_NOMMIO && xvp->hw_ops->release)
	{
		xvp->hw_ops->release(xvp->hw_arg);
	}
	else if(loopback == LOOPBACK_BSTONLY &&xvp->hw_ops->release)
	{
		pr_debug("%s: do hw_ops->release\n", __func__);
		xvp->hw_ops->release(xvp->hw_arg);
	}
}

//static struct task_struct *g_test_tid, *g_client_tid, *g_server_tid;

static int xrp_boot_firmware(struct xvp *xvp)
{
	int ret;
	struct xrp_dsp_sync_v1 __iomem *shared_sync = xvp->comm;
	pr_debug("%s: \n", __func__);
	if (loopback < LOOPBACK_BSTONLY){
		xrp_halt_dsp(xvp);
		xrp_reset_dsp(xvp);
	}


	if (xvp->firmware_name) {
		if (loopback < LOOPBACK_NOFIRMWARE) {
			ret = xrp_request_firmware(xvp);
			if (ret < 0){
				dev_err(xvp->dev, "xrp_request_firmware failed\n");
				return ret;
			}
		}

		if (loopback < LOOPBACK_NOIO) {
			xrp_comm_copy_write32(xvp, &shared_sync->sync,
					      XRP_DSP_SYNC_IDLE);
			mb();
		}
	}
	xrp_release_dsp(xvp);

	pr_debug("%s: loopback= %d\n", __func__, loopback);

//	if (loopback < LOOPBACK_NOIO) {
	if (loopback == LOOPBACK_BSTONLY) {

		pr_debug("%s: go to xrp_synchronize\n", __func__);
		ret = xrp_synchronize(xvp);
		if (ret < 0) {
			xrp_halt_dsp(xvp);
			dev_err(xvp->dev,
				"%s: couldn't synchronize with the DSP core\n",
				__func__);
			dev_err(xvp->dev,
				"XRP device will not use the DSP until the driver is rebound to this device\n");
			xvp->off = true;
			return ret;
		}
	}
{
	#ifdef ipc_en
	#if 0
	extern struct task_struct *start_client_test(void);
	extern struct task_struct *start_server_test(void);
	g_server_tid = start_server_test();
	g_client_tid = start_client_test();
	if (IS_ERR(g_server_tid)) {
		pr_err("create server tast faill!");
	} else {
		cpumask_t cpumask;
		cpumask_clear(&cpumask);
		cpumask_set_cpu(0, &cpumask);
		if (sched_setaffinity(g_server_tid->pid, &cpumask) != 0) {
			pr_err("set server tast to cpu0 fail!");
		}
	}
	wake_up_process(g_server_tid);
	msleep(2000);
	wake_up_process(g_client_tid);
	#endif
	#endif
}

	return 0;
}

static const struct file_operations xvp_fops = {
	.owner  = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = xvp_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = xvp_ioctl,
#endif
	.mmap = xvp_mmap,
	.open = xvp_open,
	.release = xvp_close,
};

int xrp_runtime_suspend(struct device *dev)
{
#if 0
	pr_debug("%s: \n", __func__);

	struct xvp *xvp = dev_get_drvdata(dev);
	xrp_halt_dsp(xvp);
	xvp_disable_dsp(xvp);

	pr_debug("%s: out\n", __func__);
#endif
	return 0;
}
EXPORT_SYMBOL(xrp_runtime_suspend);

int xrp_runtime_resume(struct device *dev)
{
//	struct xvp *xvp = dev_get_drvdata(dev);
//	unsigned i;
	int ret = 0;

#if 0
	pr_debug("%s: \n", __func__);

	for (i = 0; i < xvp->n_queues; ++i)
		mutex_lock(&xvp->queue[i].lock);

	if (xvp->off){
		dev_err(xvp->dev, "off\n");
		goto out;
	}
	ret = xvp_enable_dsp(xvp);
	if (ret < 0) {
		dev_err(xvp->dev, "couldn't enable DSP\n");
		goto out;
	}

	ret = xrp_boot_firmware(xvp);
	if (ret < 0)
		xvp_disable_dsp(xvp);

out:
	for (i = 0; i < xvp->n_queues; ++i)
		mutex_unlock(&xvp->queue[i].lock);

	pr_debug("%s: out\n", __func__);
#endif
	return ret;
}
EXPORT_SYMBOL(xrp_runtime_resume);
//static uint32_t s_index = 0;

// static void on_hello_reply(const char *message, const audio_ipc_ErrorEnum_t err, void *ext)
// {
//     if (message)
//         printf("Receive hello reply : %s.\n", message);

//     ++s_index;
// }

// static inline void call_hello(test_hifi_dsp_client* client)
// {
//     if (!client)
//         return;
//     int32_t ret = client->hello("Client", on_hello_reply, NULL);
//     if (ret < 0)
//         printf("send method hello failed. ret is %u\n", ret);
// }
#ifdef ipc_en
#if 0
static void on_complex_method_reply(const uint8_t response, const test_Array_Uint8_t resp_data, const audio_ipc_ErrorEnum_t err, void *ext)
{
    printf("Receive complex_method reply.\n");
#if 0
    printf("out1: %d, out2: %s, out3.size: %d, out4.size: %d, out5.m1: %d, out5.m2: %d, out5.m3.size: %d, out5.m4: %s, out5.m5.size: %d, out6.m3: %d, err: %d\n",
        out1, out2, out3.size, out4.size, out5.m1, out5.m2, out5.m3.size, out5.m4, out5.m5.size, out6.m3, err);
    printf("out3 data: %d, %d, %d, %d, %d, %d, %d, %d\n", out3.data[0], out3.data[1], out3.data[2], out3.data[3], out3.data[4], out3.data[5], out3.data[6], out3.data[7]);
    printf("out4 data: %d, %d, %d, %d\n", out4.data[0], out4.data[1], out4.data[2], out4.data[3]);
    printf("out5.m3 data: %d, %d, %d, %d, %d\n", out5.m3.data[0], out5.m3.data[1], out5.m3.data[2], out5.m3.data[3], out5.m3.data[4]);
    printf("out5.m5 data: %d, %d, %d, %d, %d, %d\n", out5.m5.data[0], out5.m5.data[1], out5.m5.data[2], out5.m5.data[3], out5.m5.data[4], out5.m5.data[5]);
#endif
    ++s_index;
}

static inline void call_a78_msg_sync(test_hifi_dsp_client* client)
{
	test_xrp_msg_t fs_msg;
    if (!client)
        return;

    uint8_t opcode = 100;
    
    test_Array_Uint8_t user_data = {0};
    uint8_t in3_data[16] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
    user_data.data = in3_data;
    user_data.size = 16;
    test_Array_Uint8_t i_name_space_id = {0};
    uint8_t in4_data[16] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff,0x00};
    i_name_space_id.data = in4_data;
    i_name_space_id.size = 16;
	fs_msg.i_name_space_id.data =in4_data;
	fs_msg.i_name_space_id.size =16;
	fs_msg.user_data.data=(uint32_t)in3_data;
	fs_msg.user_data.size=4;
	fs_msg.opcode =0x02;

    int32_t ret = client->hifi_a78_msg_sync(  fs_msg, on_complex_method_reply, NULL);
    if (ret < 0)
        printf("send method complex_method failed. ret is %u\n", ret);
}
#endif
#endif
// static void on_heartbeat_triggered(test_xrp_result_t status, void *ext)
// {
//     printf("Receive heartbeat broadcast. status is %d.\n", status.status);
//     call_hello((test_hifi_dsp_client *)ext);
//     call_a78_msg_sync((test_hifi_dsp_client *)ext);
// }

// static void on_heartbeat_sub_reply(uint32_t err, void *ext)
// {
//     if (err == 0)
//         printf("Subscribe heartbeat success.\n");
//     else
//         printf("Subscribe heartbeat fail. ret is %u.\n", err);
// }

// static void on_heartbeat_unsub_reply(uint32_t err, void *ext)
// {
//     if (err == 0)
//         printf("Unsubscribe heartbeat success.\n");
//     else
//         printf("Unsubscribe heartbeat fail. ret is %u.\n", err);
// }

// int audio_ipc_client_main_loop(void *arg)
// {
//     test_hifi_dsp_client *client = test_hifi_dsp_client_init();
//     if (!client)
//     {
//         printf("init client fail.\n");
//         return -1;
//     }

//     // get version
//     ipc_inf_version version = client->version();
//     printf("Interface version: major %d, minor %d.\n", version.major, version.minor);

//     // subscribe broadcast.
//     int32_t ret = client->heartbeat_sub(on_heartbeat_triggered, (void *)client, on_heartbeat_sub_reply, NULL);
//     if (ret < 0)
//     {
//         printf("send subscribe message fail.\n");
//         return -2;
//     }

//     // call method.
//     call_hello(client);
//     call_a78_msg_sync(client);

//     uint32_t total = 4u;

//     printf("\n [%s] enter main loop\n", __FUNCTION__);
//     while (1)
//     {
//         ret = client->receive_message();
//         if (ret < 0)
//             continue;
//         ret = client->dispatch_message();
//         if (ret < 0)
//             continue;

//         if (s_index > total)
//             break;

//         msleep(10);
//     }

//     ret = test_hifi_dsp_client_destroy();
//     if (ret < 0)
//     {
//         printf("destory client fail.\n");
//         return -3;
//     }

//     return 0;
// }

static int xrp_init_regs_v0(struct platform_device *pdev, struct xvp *xvp)
{
	struct resource *mem;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!mem)
		return -ENODEV;

	xvp->comm_phys = mem->start;
	if (xvp->direct_mapping)
		xvp->comm = devm_ioremap_resource(&pdev->dev, mem);
	else
		xvp->comm = xrp_alloc_host(xvp, resource_size(mem));

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!mem)
		return -ENODEV;

	xvp->pmem = mem->start;
	xvp->shared_size = resource_size(mem);
	return xrp_init_private_pool(&xvp->pool, xvp->pmem,
				     xvp->shared_size);
}

static int xrp_init_regs_v1(struct platform_device *pdev, struct xvp *xvp)
{
	struct resource *mem;
	struct resource r;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem)
		return -ENODEV;

	if (resource_size(mem) < 2 * PAGE_SIZE) {
		dev_err(xvp->dev,
			"%s: shared memory size is too small\n",
			__func__);
		return -ENOMEM;
	}

	xvp->comm_phys = mem->start;
	xvp->pmem = mem->start + PAGE_SIZE;
	xvp->shared_size = resource_size(mem) - PAGE_SIZE;
	dev_err(xvp->dev,"%s: shared memory size is 0x%llx , comm_phys =0x%llx , pmem =0x%llx\n",__func__,resource_size(mem),xvp->comm_phys, xvp->pmem);

	r = *mem;
	r.end = r.start + PAGE_SIZE;
	if (xvp->direct_mapping) {
		pr_info("%s: devm_ioremap_resource_wc",__func__);
//		xvp->comm = ioremap_wc(r.start, PAGE_SIZE);
		xvp->comm = devm_ioremap_resource_wc(&pdev->dev, &r);
	}
	else {
		xvp->comm = xrp_alloc_host(xvp, PAGE_SIZE);
	}

	//kthread_create(audio_ipc_client_main_loop, NULL, "ipc-client-msgbox-%u", smp_processor_id());
	//xvp->xrp_ipc = test_hifi_dsp_client_init();

	return xrp_init_private_pool(&xvp->pool, xvp->pmem,
				     xvp->shared_size);
}

static int xrp_init_regs_cma(struct platform_device *pdev, struct xvp *xvp)
{
	dma_addr_t comm_phys;

	if (of_reserved_mem_device_init(xvp->dev) < 0)
		return -ENODEV;

	xvp->comm = dma_alloc_attrs(xvp->dev, PAGE_SIZE, &comm_phys,
				    GFP_KERNEL, 0);
	if (!xvp->comm)
		return -ENOMEM;
	//xvp->xrp_ipc = test_hifi_dsp_client_init();
	xvp->comm_phys = dma_to_phys(xvp->dev, comm_phys);
	return xrp_init_cma_pool(&xvp->pool, xvp->dev);
}

static int compare_queue_priority(const void *a, const void *b)
{
	const void * const *ppa = a;
	const void * const *ppb = b;
	const struct xrp_comm *pa = *ppa, *pb = *ppb;

	if (pa->priority == pb->priority)
		return 0;
	else
		return pa->priority < pb->priority ? -1 : 1;
}

static long xrp_init_common(struct platform_device *pdev,
			    enum xrp_init_flags init_flags,
			    const struct xrp_hw_ops *hw_ops, void *hw_arg,
			    int (*xrp_init_regs)(struct platform_device *pdev,
						 struct xvp *xvp))
{
	long ret;
	char nodename[sizeof("xvp") + 3 * sizeof(int)];
	struct xvp *xvp;
	int nodeid;
	unsigned i;
	ipc_inf_version_t version;

	xvp = devm_kzalloc(&pdev->dev, sizeof(*xvp), GFP_KERNEL);
	if (!xvp) {
		ret = -ENOMEM;
		goto err;
	}

	xvp->dev = &pdev->dev;
	xvp->hw_ops = hw_ops;
	xvp->hw_arg = hw_arg;
	if (init_flags & XRP_INIT_USE_HOST_IRQ)
		xvp->host_irq_mode = true;
	if (!(init_flags & XRP_INIT_NO_DIRECT_MAPPING)) {
		xvp->direct_mapping = true;
	} else {
		if (WARN_ON(hw_ops->copy_to_alloc == NULL ||
			    hw_ops->copy_from_alloc == NULL ||
			    hw_ops->copy_to_alloc_user == NULL ||
			    hw_ops->copy_from_alloc_user == NULL)) {
			ret = -EINVAL;
			goto err;
		}
	}
	platform_set_drvdata(pdev, xvp);

	ret = xrp_init_regs(pdev, xvp);
	#if 0
	if (ret < 0)
	{
		pr_err("%s: check 111\n", __func__);
		goto err;
	}

	#else
	if (ret < 0)
		goto err;
	#endif

	pr_err("%s: comm = %pap/%p \n", __func__, &xvp->comm_phys, xvp->comm);
	pr_err("%s: xvp->pmem = %pap\n", __func__, &xvp->pmem);

	ret = xrp_init_address_map(xvp->dev, &xvp->address_map);
	#if 0
	if (ret < 0)
	{
		pr_err("%s: check 2222\n", __func__);
		goto err_free_pool;
	}

	#else
	if (ret < 0)
		goto err_free_pool;
	#endif

	ret = device_property_read_u32_array(xvp->dev, "queue-priority",
					     NULL, 0);
	if (ret > 0) {
		xvp->n_queues = ret;
		xvp->queue_priority = devm_kmalloc(&pdev->dev,
						   ret * sizeof(u32),
						   GFP_KERNEL);
		if (xvp->queue_priority == NULL)
			goto err_free_pool;
		ret = device_property_read_u32_array(xvp->dev,
						     "queue-priority",
						     xvp->queue_priority,
						     xvp->n_queues);
		if (ret < 0)
			goto err_free_pool;
		dev_err(xvp->dev,
			"multiqueue (%d) configuration, queue priorities:\n",
			xvp->n_queues);
		for (i = 0; i < xvp->n_queues; ++i)
			dev_err(xvp->dev, "  %d\n", xvp->queue_priority[i]);
	} else {
		xvp->n_queues = 1;
	}
	xvp->queue = devm_kmalloc(&pdev->dev,
				  xvp->n_queues * sizeof(*xvp->queue),
				  GFP_KERNEL);
	xvp->queue_ordered = devm_kmalloc(&pdev->dev,
					  xvp->n_queues * sizeof(*xvp->queue_ordered),
					  GFP_KERNEL);
	if (xvp->queue == NULL ||
	    xvp->queue_ordered == NULL)
		goto err_free_pool;

	for (i = 0; i < xvp->n_queues; ++i) {
		mutex_init(&xvp->queue[i].lock);
		xvp->queue[i].comm = xvp->comm + XRP_DSP_CMD_STRIDE * i;
		init_completion(&xvp->queue[i].completion);
		if (xvp->queue_priority)
			xvp->queue[i].priority = xvp->queue_priority[i];
		xvp->queue_ordered[i] = xvp->queue + i;
	}
	sort(xvp->queue_ordered, xvp->n_queues, sizeof(*xvp->queue_ordered),
	     compare_queue_priority, NULL);
	if (xvp->n_queues > 1) {
		dev_err(xvp->dev, "SW -> HW queue priority mapping:\n");
		for (i = 0; i < xvp->n_queues; ++i) {
			dev_err(xvp->dev, "  %d -> %d\n",
				i, xvp->queue_ordered[i]->priority);
		}
	}

	ret = device_property_read_string(xvp->dev, "firmware-name",
					  &xvp->firmware_name);
	if (ret == -EINVAL || ret == -ENODATA) {
		dev_err(xvp->dev,
			"no firmware-name property, not loading firmware");
	} else if (ret < 0) {
		dev_err(xvp->dev, "invalid firmware name (%ld)", ret);
		goto err_free_map;
	}

	//init xrp msgbox client
	xvp->xrp_ipc = audio_msgbox_client_init(&soc_data);
	if(xvp->xrp_ipc == NULL){
		return -ENOMEM;
	}
	xvp->xrp_ipc->start();
	version = xvp->xrp_ipc->audio_ipc_client.version();
	pr_debug("%s:  major %d, minor %d.\n",  __func__, version.major, version.minor);


	pm_runtime_enable(xvp->dev);
#if 0
//	pm_runtime_enable(xvp->dev);
//	pr_debug("%s: pm_runtime_enable 1 \n", __func__);
//	if (!pm_runtime_enabled(xvp->dev)) {
		pr_debug("%s: go to xrp_runtime_resume \n", __func__);
		ret = xrp_runtime_resume(xvp->dev);
		if (ret)
			goto err_pm_disable;
//	}
#endif

	nodeid = ida_simple_get(&xvp_nodeid, 0, 0, GFP_KERNEL);
	if (nodeid < 0) {
		ret = nodeid;
		goto err_pm_disable;
	}
	xvp->nodeid = nodeid;
	sprintf(nodename, "xvp%u", nodeid);

	xvp->miscdev = (struct miscdevice){
		.minor = MISC_DYNAMIC_MINOR,
		.name = devm_kstrdup(&pdev->dev, nodename, GFP_KERNEL),
		.nodename = devm_kstrdup(&pdev->dev, nodename, GFP_KERNEL),
		.fops = &xvp_fops,
	};

	ret = misc_register(&xvp->miscdev);
	if (ret < 0)
		goto err_free_id;
	return PTR_ERR(xvp);
err_free_id:
	ida_simple_remove(&xvp_nodeid, nodeid);
err_pm_disable:
	pm_runtime_disable(xvp->dev);
err_free_map:
	xrp_free_address_map(&xvp->address_map);
err_free_pool:
	//test_hifi_dsp_client_destroy();
	#ifdef ipc_en
	xvp->xrp_ipc = NULL;
	#endif
	xrp_free_pool(xvp->pool);
	if (!xvp->direct_mapping)
		xrp_free_host(xvp, xvp->comm);
	else if (xvp->comm_phys && !xvp->pmem)
		dma_free_attrs(xvp->dev, PAGE_SIZE, xvp->comm,
			       phys_to_dma(xvp->dev, xvp->comm_phys), 0);
err:
	dev_err(&pdev->dev, "%s: ret = %ld\n", __func__, ret);
	return ret;
}

typedef long xrp_init_function(struct platform_device *pdev,
			       enum xrp_init_flags flags,
			       const struct xrp_hw_ops *hw_ops, void *hw_arg);

xrp_init_function xrp_init;
long xrp_init(struct platform_device *pdev, enum xrp_init_flags flags,
	      const struct xrp_hw_ops *hw_ops, void *hw_arg)
{
	return xrp_init_common(pdev, flags, hw_ops, hw_arg, xrp_init_regs_v0);
}
EXPORT_SYMBOL(xrp_init);

xrp_init_function xrp_init_v1;
long xrp_init_v1(struct platform_device *pdev, enum xrp_init_flags flags,
		 const struct xrp_hw_ops *hw_ops, void *hw_arg)
{
	return xrp_init_common(pdev, flags, hw_ops, hw_arg, xrp_init_regs_v1);
}
EXPORT_SYMBOL(xrp_init_v1);

xrp_init_function xrp_init_cma;
long xrp_init_cma(struct platform_device *pdev, enum xrp_init_flags flags,
		  const struct xrp_hw_ops *hw_ops, void *hw_arg)
{
	return xrp_init_common(pdev, flags, hw_ops, hw_arg, xrp_init_regs_cma);
}
EXPORT_SYMBOL(xrp_init_cma);

int xrp_deinit(struct platform_device *pdev)
{
	struct xvp *xvp = platform_get_drvdata(pdev);

	if (!xvp)
		return 0;
	pm_runtime_disable(xvp->dev);
	if (!pm_runtime_status_suspended(xvp->dev))
		xrp_runtime_suspend(xvp->dev);

	misc_deregister(&xvp->miscdev);
	xrp_release_firmware(xvp);
	xrp_free_pool(xvp->pool);
	if (!xvp->direct_mapping)
		xrp_free_host(xvp, xvp->comm);
	else if (xvp->comm_phys && !xvp->pmem)
		dma_free_attrs(xvp->dev, PAGE_SIZE, xvp->comm,
			       phys_to_dma(xvp->dev, xvp->comm_phys), 0);
	xrp_free_address_map(&xvp->address_map);
	ida_simple_remove(&xvp_nodeid, xvp->nodeid);
	return 0;
}
EXPORT_SYMBOL(xrp_deinit);

int xrp_deinit_hw(struct platform_device *pdev, void **hw_arg)
{
	if (hw_arg) {
		struct xvp *xvp = platform_get_drvdata(pdev);
		*hw_arg = xvp->hw_arg;
	}
	return xrp_deinit(pdev);
}
EXPORT_SYMBOL(xrp_deinit_hw);

static void *get_hw_sync_data(void *hw_arg, size_t *sz)
{
	void *p = kzalloc(64, GFP_KERNEL);
	pr_debug("%s: kzalloc \n", __func__);

	*sz = 64;
	return p;
}

static const struct xrp_hw_ops hw_ops = {
	.get_hw_sync_data = get_hw_sync_data,
};

#ifdef CONFIG_OF
static const struct of_device_id xrp_of_match[] = {
	{
		.compatible = "cdns,xrp",
		.data = xrp_init,
	}, {
		.compatible = "cdns,xrp,v1",
		.data = xrp_init_v1,
	}, {
		.compatible = "cdns,xrp,cma",
		.data = xrp_init_cma,
	}, {},
};
MODULE_DEVICE_TABLE(of, xrp_of_match);
#endif

#ifdef CONFIG_ACPI
xrp_init_function xrp_acpi_init_v0;
long xrp_acpi_init_v0(struct platform_device *pdev,
		      enum xrp_init_flags flags,
		      const struct xrp_hw_ops *hw_ops, void *hw_arg)
{
	long ret = xrp_init(pdev, flags, hw_ops, hw_arg);

	if (!IS_ERR_VALUE(ret)) {
		struct xvp *xvp = ERR_PTR(ret);
		struct xrp_address_map_entry entry[] = {
			{
				.src_addr = xvp->comm_phys,
				.dst_addr = (u32)xvp->comm_phys,
				.size = PAGE_SIZE,
			}, {
				.src_addr = xvp->pmem,
				.dst_addr = (u32)xvp->pmem,
				.size = xvp->shared_size,
			},
		};

		/*
		 * On ACPI system DSP can currently only access
		 * its communication area and shared memory.
		 */
		ret = xrp_set_address_map(&xvp->address_map,
					  ARRAY_SIZE(entry),
					  entry);
		if (ret) {
			dev_err(xvp->dev,
				"%s: couldn't set up mapping for shared memory\n",
				__func__);
		}
	}
	return ret;
}
EXPORT_SYMBOL(xrp_acpi_init_v0);

static xrp_init_function xrp_acpi_init_v1;
static long xrp_acpi_init_v1(struct platform_device *pdev,
			     enum xrp_init_flags flags,
			     const struct xrp_hw_ops *hw_ops, void *hw_arg)
{
	long ret = xrp_init_v1(pdev, flags, hw_ops, hw_arg);

	if (!IS_ERR_VALUE(ret)) {
		struct xvp *xvp = ERR_PTR(ret);
		struct xrp_address_map_entry entry = {
			.src_addr = xvp->comm_phys,
			.dst_addr = (u32)xvp->comm_phys,
			.size = (u32)xvp->shared_size + PAGE_SIZE,
		};

		/*
		 * On ACPI system DSP can currently only access
		 * its own shared memory.
		 */
		ret = xrp_set_address_map(&xvp->address_map,
					  1, &entry);
		if (ret) {
			dev_err(xvp->dev,
				"%s: couldn't set up mapping for shared memory\n",
				__func__);
		}
	}
	return ret;
}

static const struct acpi_device_id xrp_acpi_match[] = {
	{ "CXRP0000", (unsigned long)xrp_acpi_init_v0, },
	{ "CXRP0001", (unsigned long)xrp_acpi_init_v1, },
	{ },
};
MODULE_DEVICE_TABLE(acpi, xrp_acpi_match);
#endif

static int xrp_probe(struct platform_device *pdev)
{
	long ret = -EINVAL;

#ifdef CONFIG_OF
	{
		const struct of_device_id *match;

		match = of_match_device(xrp_of_match, &pdev->dev);
		if (match) {
			xrp_init_function *init = match->data;

			ret = init(pdev, 0, &hw_ops, NULL);
			return IS_ERR_VALUE(ret) ? ret : 0;
		} else {
			pr_debug("%s: no OF device match found\n", __func__);
		}
	}
#endif
#ifdef CONFIG_ACPI
	{
		const struct acpi_device_id *match;

		match = acpi_match_device(xrp_acpi_match, &pdev->dev);
		if (match) {
			xrp_init_function *init = (void *)match->driver_data;

			ret = init(pdev, 0, &hw_ops, NULL);
			return IS_ERR_VALUE(ret) ? ret : 0;
		} else {
			pr_debug("%s: no ACPI device match found\n", __func__);
		}
	}
#endif
	return ret;
}

static int xrp_remove(struct platform_device *pdev)
{
	return xrp_deinit(pdev);
}

static const struct dev_pm_ops xrp_pm_ops = {
	SET_RUNTIME_PM_OPS(xrp_runtime_suspend,
			   xrp_runtime_resume, NULL)
};

static struct platform_driver xrp_driver = {
	.probe   = xrp_probe,
	.remove  = xrp_remove,
	.driver  = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(xrp_of_match),
		.acpi_match_table = ACPI_PTR(xrp_acpi_match),
		.pm = &xrp_pm_ops,
	},
};

module_platform_driver(xrp_driver);

MODULE_AUTHOR("Takayuki Sugawara");
MODULE_AUTHOR("Max Filippov");
MODULE_DESCRIPTION("XRP: Linux device driver for Xtensa Remote Processing");
MODULE_LICENSE("Dual MIT/GPL");
