/*
 * syscall_vbi.c - vbi interface system calls.
 *
 * Copyright (c) 2010 Wind River Systems, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 */

#include <linux/gfp.h>
#include <linux/slab.h>
#include <linux/types.h>

#ifdef CONFIG_WRHV
#include <vbi/vbi.h>
#include <asm/wrhv.h>
#endif

#include <asm/uaccess.h>
#include <linux/vmalloc.h>
#include <asm/unistd.h>

#include <linux/capability.h>
#include <linux/sched.h>

#define OK              0
#define ERROR           -1

#ifdef CONFIG_WRHV

asmlinkage long sys_vbi_activate_vb(uint32_t vb, uint32_t addr)
{
	int retval;
	u32 vb_cfg;
	VBI_HREG_SET_CMPLX_QUALIFIED rctl;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	/*
	 * The policy should be 'do not operate self VB'
	 */
	if (vb == VBI_BOARD_ID_GET()) {
		printk(KERN_ERR "%s: can not operate self VB.\n", __func__);
		return -EPERM;
	}

	/*
	 * Note, here suspend all cores means if dest VB is configured
	 * as multi CPUs VB, all vcpus of this VB will be suspended.
	 */
	retval = vbi_vb_suspend(vb, VBI_VB_CORES_ALL);
	if (retval) {
		printk(KERN_ERR "%s: vb_suspend VB%d failed.\n", __func__, vb);
		return retval;
	}

	retval = vbi_vb_find_board_config(vb, 0, &vb_cfg);
	if (retval) {
		printk(KERN_ERR "%s: Get VB%d config error.\n", __func__, vb);
		return retval;
	}

	retval = vbi_vb_read_reg(&rctl, vb, 0);
	if (retval) {
		printk(KERN_ERR "%s: read_reg VB%d failed.\n", __func__, vb);
		return retval;
	}

#ifdef CONFIG_X86
	/*
	 * Since the memory of the target vb has been changed(writen
	 * alternative image). Disable interrupts could make sure
	 * old image won't get chance to run before loaded image
	 * gain control
	 */
	rctl.vbiRegSet.hreg32.eflags &= ~X86_EFLAGS_IF;
	rctl.vbiRegSet.hreg32.eip = addr;
	rctl.vbiRegSet.hreg32.eax = vb_cfg;
#endif
#ifdef CONFIG_PPC
	rctl.vbiRegSet.hreg32.pc = addr;
	wrhv_setup_msr_for_ap(&rctl);
	/* r3, 1st argument pointer to config page */
	rctl.vbiRegSet.hreg32.gpr[3] = vb_cfg;
#endif
#ifdef CONFIG_ARM
	rctl.vbiRegSet.hreg32.pc = (unsigned long *)addr;
	rctl.vbiRegSet.hreg32.r[0] = (uint32_t)vb_cfg;
	rctl.vbiRegSet.hreg32.cpsr = PSR_F_BIT | PSR_I_BIT | SVC_MODE;
#endif

	retval = vbi_vb_write_reg(&rctl, vb, 0);
	if (retval) {
		printk(KERN_ERR "%s: write_reg VB%d failed.\n", __func__, vb);
		return retval;
	}

	/*
	 * Here just core0 of the VB gets re-activated.
	 */
	retval = vbi_vb_resume(vb, 0);
	if (retval)
		printk(KERN_ERR "%s: vb_resume VB%d failed.\n", __func__, vb);

	return retval;
}

asmlinkage long sys_vbi_mem(uint32_t vb, void *dest, void *src,
				  uint32_t size, uint32_t flags)
{
	struct vbi_mem_ctl memCtl;
	void *vbi_mem_buf;
	uint32_t len;
	long rv = OK;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	/*
	 * The policy should be 'do not operate self VB'
	 */
	if (vb == VBI_BOARD_ID_GET()) {
		printk(KERN_ERR "%s: can not operate self VB.\n", __func__);
		return -EPERM;
	}

	/* Must be one page in size and page aligned */
	vbi_mem_buf = (void *)__get_free_page(GFP_KERNEL);
	if (!vbi_mem_buf)
		return -ENOMEM;

	if (flags & VBI_MEM_WRITE) {
		while (size > 0) {
			len = size < PAGE_SIZE ? size : PAGE_SIZE;

			if ((copy_from_user(vbi_mem_buf, src, len) > 0)) {
				rv = -EACCES;
				goto err;
			}

			memCtl.pBuffer = dest;
			memCtl.pAddress = vbi_mem_buf;
			memCtl.size_in = len;
			memCtl.size_out = 0;
			memCtl.flags = VBI_ICACHE_INV|VBI_DCACHE_FLUSH;
			if ((vbi_vb_write_mem(&memCtl, vb) != OK)) {
				rv = -EACCES;
				goto err;
			}

			size -= len;
			src += len;
			dest += len;
		}
	} else if (flags & VBI_MEM_READ) {
		memCtl.pBuffer = src;
		memCtl.pAddress = vbi_mem_buf;
		memCtl.size_in = size;
		memCtl.size_out = 0;
		memCtl.flags = flags;
		if ((vbi_vb_read_mem(&memCtl, vb) != OK))
			rv = -EACCES;
		else {
			if ((len = copy_to_user(vbi_mem_buf, dest, size)) > 0)
				rv = -EACCES;
		}
	} else {
		rv = -EINVAL;
	}

err:
	free_page((unsigned long)vbi_mem_buf);
	return rv;
}

asmlinkage long sys_vbi_control(uint32_t vb, uint32_t command, uint32_t flags)
{
	long ret = 0;
	int  cores;
	uint32_t reset_opts;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	/*
	 * The policy should be 'do not operate self VB'
	 */
	if (vb == VBI_BOARD_ID_GET()) {
		printk(KERN_ERR "%s: can not reset self.\n", __func__);
		return -EPERM;
	}

	reset_opts = VBI_VBMGMT_RESET_DOWNLOAD;

	cores = flags & 0xff;
	if (cores == 0xff) {
		cores = VBI_VB_CORES_ALL;
		reset_opts |= VBI_VBMGMT_RESET_AND_START_CORE0;
	}

	switch(command) {
	case SYS_VBI_VB_SUSPEND:
		ret = vbi_vb_suspend(vb, cores);
		if (ret)
			printk(KERN_ERR "%s: vb_suspend VB%d failed.\n", __func__, vb);
		break;
	case SYS_VBI_VB_RESUME:
		ret = vbi_vb_resume(vb, cores);
		if (ret)
			printk(KERN_ERR "%s: vb_resume VB%d failed.\n", __func__, vb);
		break;
	case SYS_VBI_VB_RESTART:
		/*
		 * vbi_vb_reset won't success in case 'non-boot core'
		 * issue the vbi call, so we have to make sure 'current'
		 * is running on core0
		 */
		set_cpus_allowed_ptr(current, cpumask_of(0));
		ret = vbi_vb_reset(vb, cores, reset_opts);
		if (ret)
			printk(KERN_ERR "%s: vb_reset VB%d failed.\n", __func__, vb);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

#else /* native/stub variants CONFIG_WRHV */

asmlinkage long sys_vbi_activate_vb(uint32_t vb, uint32_t addr)
{
	return -ENOSYS;
}

asmlinkage long sys_vbi_mem(uint32_t vb, void *dest, void *src,
				  uint32_t size, uint32_t flags)
{
	return -ENOSYS;
}

asmlinkage long sys_vbi_control(uint32_t vb, uint32_t command, uint32_t flags)
{
	return -ENOSYS;
}
#endif /* CONFIG_WRHV */
