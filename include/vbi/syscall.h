/*
 * syscall.h - hypervisor system calls
 *
 * Copyright (c) 2007-2009 Wind River Systems, Inc.
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

#ifndef _VBI_SYSCALL_H
#define _VBI_SYSCALL_H

#include <linux/linkage.h>
#include <vbi/types.h>
#include <vbi/arch.h>
#include <vbi/interface.h>
#include <vbi/syscalls.h>
#include <vbi/stats.h>

#ifndef	_ASMLANGUAGE

/* Forward declaration */
struct vmmuConfig;
struct vmmu64_config;

/* information about incoming message */

struct vbi_msg_info
{
	int32_t  id;		/* context id of sender */
	uint32_t type;		/* message type (msg/event) */
	uint64_t slen;		/* length of sent buffer */
	uint64_t rlen;		/* length of received buffer */
	uint16_t status;	/* message status information */
	uint16_t error;		/* extended error status */
	uint64_t timestamp;	/* time message was sent */
	uint32_t nmsg;		/* number of queued messages remaining */
}  __attribute__((packed));

/* message status bits as reported in the "status" field */

#define	VBI_MSG_STATUS_OK		0
#define	VBI_MSG_STATUS_TRUNCATED	1
#define	VBI_MSG_STATUS_COPY_ERROR	2

/* extended error codes reported in "error" field */

#define	VBI_MSG_ERROR_INTLEVEL	1 /* operation not allowed at interrupt level */
#define	VBI_MSG_ERROR_BAD_ID    2 /* bad context id specified */
#define	VBI_MSG_ERROR_ABORTED   3 /* operation aborted */
#define	VBI_MSG_ERROR_TIMEOUT   4 /* operation timed out */
#define	VBI_MSG_ERROR_XCALL     5 /* cross call to remote cpu failed */

/* message types as reported in the "type" field */

#define	VBI_MSG_TYPE_REGULAR	1 /* regular message */
#define	VBI_MSG_TYPE_EVENT	2 /* event message */
#define	VBI_MSG_TYPE_REPLY	3 /* reply message */


/* modifiers for message processing */

struct vbi_msg_ctl
{
	uint32_t flags;		/* operation flags */
	uint32_t ordering;	/* order to receive messages */
	uint64_t timeout;	/* receive timeout */
} __attribute__((packed));

/* message control flags */

#define VBI_MSG_CTL_FLAG_RETRY 1

/* memory read/write control structure */
struct vbi_mem_ctl
{
	VB_ALIGN_FIELD_64(void *pBuffer, pad1); /* address of target context */
	VB_ALIGN_FIELD_64(void *pAddress, pad2);/* address of calling context */
	uint64_t size_in;	    /* number of bytes to be read	    */
	uint64_t size_out;	    /* number of bytes successfully read    */
	uint32_t flags;	    /* data/instruction flush option	    */
};

#define VBI_DCACHE_FLUSH	0x0001
#define VBI_ICACHE_INV		0x0002

/* system call prototypes for use within a context */

extern asmlinkage int vbi_hy_ioctl(unsigned ioctl, void *arg1, void *arg2,
				void *arg3, void *arg4);
extern asmlinkage int vbi_vtlb_op(unsigned int op, unsigned long arg1,
				unsigned long arg2, unsigned long arg3);

/* name server options and timeouts */

#define VBI_NS_NOVERSION	0xffffffff /* no specific service version */
#define VBI_NS_ANYVERSION	0xffffffff /* match any service version   */
#define VBI_NS_OPTION_NONE	0x0000     /* no options                  */
#define VBI_NS_OPTION_LOOKUP_WAIT 0x0001   /* wait for name registration  */
#define VBI_NS_NO_TIMEOUT	0x0000     /* wait forever */

/*
 * Modified APIs for VBI 2.0
 */

/* message receive */
extern int32_t vbi_receive(void *rmsg, uint32_t rlen, struct vbi_msg_info *info,
				struct vbi_msg_ctl *ctl);

extern asmlinkage int32_t vbi_panic(const char *msg);
extern int32_t vbi_flush_dcache(void *saddr, size_t size);
extern int32_t vbi_flush_icache(void *saddr, size_t size);
extern int32_t vbi_config_vmmu(struct vmmuConfig * config);
#ifdef CONFIG_MIPS
extern int32_t vbi_create_vmmu(struct vmmu64_config * config);
extern int32_t vbi_delete_vmmu(struct vmmu64_config * config);
#else
extern int32_t vbi_create_vmmu(struct vmmuConfig * config);
extern int32_t vbi_delete_vmmu(struct vmmuConfig * config);
#endif
extern int32_t vbi_enable_vmmu(uint32_t  vmmu_num);
extern int32_t vbi_disable_vmmu(uint32_t vmmu_num);

extern int32_t vbi_ns_register(char  *name, uint32_t  revision);
extern int32_t vbi_ns_unregister(char *name, uint32_t  revision);
extern int32_t vbi_ns_lookup_old(char *name, uint32_t  rev,
				VBI_NS_HANDLE *pHandle);
extern int32_t vbi_ns_lookup(char *name, uint32_t  rev, VBI_NS_HANDLE *pHandle,
				uint32_t timeout, uint32_t options);

extern asmlinkage void vbi_vcore_irq_unlock(void);
extern asmlinkage int32_t vbi_vcore_irq_lock(void);
extern int32_t vbi_update_text_cache(void *saddr, size_t size);
extern int32_t vbi_set_exc_base(void *excTblBase);

/* virtual board management API's */
extern asmlinkage int32_t vbi_vb_reset(uint32_t id, int32_t core, uint32_t options);

/* standard hypervisor and safety hypervisor stub functions */
#if !defined(CONFIG_WRHV_SAFETY_PROFILE)
extern int32_t vbi_flush_tlb(uint32_t asid, void *addr, size_t len);
extern asmlinkage int vbi_ctx_ctl(unsigned operation, unsigned arg1,
				unsigned arg2);
extern asmlinkage int32_t vbi_kputs(const char *s);
extern asmlinkage int32_t vbi_kputc(int c);
extern asmlinkage int32_t  vbi_set_mem_attr(void *vaddr, size_t len,
						int32_t attr);
extern asmlinkage int32_t  vbi_get_mem_attr(void *vaddr, int32_t * attr);
extern asmlinkage int32_t vbi_vb_suspend(uint32_t id, int32_t core);
extern asmlinkage int32_t vbi_vb_restart(uint32_t id, int32_t core);
extern asmlinkage int32_t vbi_vb_resume(uint32_t id, int32_t core);
#else
/* following functions available to debug version of safety profile hypervisor */
extern asmlinkage int32_t safety_debug_vbi_kputs(const char *s);
extern asmlinkage int32_t safety_debug_vbi_kputc(int c);
extern asmlinkage void safety_debug_vbi_shell_start_debug(uint32_t  flags);

/* schedule transition api */
extern int32_t vbi_sched_transition(char *name, uint32_t transition_type,
				uint32_t core_id);

/* system call prototypes for use within a context */
static inline int vbi_ctx_ctl(unsigned operation, unsigned arg1,
				unsigned arg2)
{
	/* GOS often call this api to go into idle, do not
	* print log message to reduce large amount of output
	*/
	VBISTAT_VERBOSE(vbi_ctx_ctl);
	return -1;
}
static inline int32_t vbi_kputs(const char *s)
{
	/* non-debug safety profile hypervisor does not support
	 * this function.
	 */
	if (safety_hyp_version == SAFETY_HYP_VER_STD) {
		VBISTAT_VERBOSE(vbi_kputs);
		return -1;
	}
	return safety_debug_vbi_kputs(s);
}

static inline int32_t vbi_kputc(int c)
{
	/* non-debug safety profile hypervisor does not support
	 * this function.
	 */
	if (safety_hyp_version == SAFETY_HYP_VER_STD) {
		VBISTAT_VERBOSE(vbi_kputc);
		return -1;
	}
	return safety_debug_vbi_kputc(c);
}
/* Prior to vbi 2.0 these api were vbi_set_mmu_attr/Get */
static inline int32_t vbi_set_mem_attr(void *vaddr, size_t len, int32_t attr)
{
	VBISTAT_VERBOSE(vbi_set_mem_attr);
	return -1;
}
static inline int32_t vbi_get_mem_attr(void *vaddr, int32_t *attr)
{
	VBISTAT_VERBOSE(vbi_get_mem_attr);
	return -1;
}
static inline int32_t vbi_vb_suspend(uint32_t id, int32_t core)
{
	VBISTAT_VERBOSE(vbi_vb_suspend);
	return -1;
}
/* virtual board management API's */
static inline int32_t vbi_vb_restart(uint32_t id, int32_t core)
{
	VBISTAT_VERBOSE(vbi_vb_restart);
	return -1;
}
static inline int32_t vbi_vb_resume(uint32_t id, int32_t core)
{
	VBISTAT_VERBOSE(vbi_vb_resume);
	return -1;
}

#endif

/*
 * These two VBI should only be used in core + SMP mode.
 */
#if !defined(CONFIG_SMP) && defined(CONFIG_WRHV_COREVBI_ONLY)
static inline int32_t vbi_vb_read_reg(VBI_HREG_SET_CMPLX_QUALIFIED *regSet,
				       uint32_t targetBoard, int32_t core)
{
	VBISTAT_VERBOSE(vbi_vb_read_reg);
	return -1;
}
static inline int32_t vbi_vb_write_reg(VBI_HREG_SET_CMPLX_QUALIFIED *regSet,
					uint32_t targetBoard, int32_t core)
{
	VBISTAT_VERBOSE(vbi_vb_write_reg);
	return -1;
}
#else
/* read/write remote vb's registers */
extern asmlinkage int32_t vbi_vb_read_reg(VBI_HREG_SET_CMPLX_QUALIFIED *regSet,
				       uint32_t targetBoard, int32_t core);
extern asmlinkage int32_t vbi_vb_write_reg(VBI_HREG_SET_CMPLX_QUALIFIED *regSet,
					uint32_t targetBoard, int32_t core);
#endif

/* Optional VBI */
#if defined(CONFIG_WRHV_COREVBI_ONLY)
/* message reply */
static inline int32_t vbi_reply(int32_t id, void *smsg, size_t slen,
		struct vbi_msg_ctl *ctl)
{
	VBISTAT_VERBOSE(vbi_reply);
	return -1;
}
/* Message send */
static inline int32_t vbi_send(int32_t id, void *smsg, size_t slen,
		void *rmsg, size_t rlen, struct vbi_msg_info *info,
		struct vbi_msg_ctl *ctl)
{
	VBISTAT_VERBOSE(vbi_send);
	return -1;
}
static inline int32_t vbi_vb_read_mem(struct vbi_mem_ctl *memCtl,
					uint32_t targetBoard)
{
	VBISTAT_VERBOSE(vbi_vb_read_mem);
	return -1;
}
static inline int32_t vbi_vb_write_mem(struct vbi_mem_ctl *memCtl,
					uint32_t targetBoard)
{
	VBISTAT_VERBOSE(vbi_vb_write_mem);
	return -1;
}
static inline void vbi_shell_start_debug(uint32_t  flags)
{
	/* safety profile hypervisor does not support this function */
	if (safety_hyp_version == SAFETY_HYP_VER_DEBUG) {
#ifdef CONFIG_WRHV_SAFETY_PROFILE
		safety_debug_vbi_shell_start_debug(flags);
#endif
		return;
	}

	VBISTAT_VERBOSE(vbi_shell_start_debug);
	return;
}
static inline int32_t vbi_vb_mgmt(uint32_t cmd, uint32_t boardId,
				int32_t *outError, uint32_t flags, void * ctl)
{
	VBISTAT_VERBOSE(vbi_vb_mgmt);
	return -1;
}
static inline int vbi_io_apic_ioctl(unsigned ioctl, unsigned arg1,
				unsigned arg2)
{
	VBISTAT_VERBOSE(vbi_io_apic_ioctl);
	return -1;
}

static inline int32_t vbi_get_max_asid_vmmu(void)
{
	VBISTAT_VERBOSE(vbi_get_max_asid_vmmu);
	return -1;
}
static inline int32_t vbi_tlb_flush_vmmu(struct vmmuConfig *config,
				void *addr, size_t len)
{
	VBISTAT_VERBOSE(vbi_tlb_flush_vmmu);
	return -1;
}
static inline int32_t vbi_tlb_load_vmmu(struct vmmuConfig *config, void *addr,
					unsigned int len)
{
	VBISTAT_VERBOSE(vbi_tlb_load_vmmu);
	return -1;
}
#else
/* message reply */
extern asmlinkage int32_t vbi_reply(int32_t id, void *smsg, size_t slen,
				struct vbi_msg_ctl *ctl);
/* Message send */
extern asmlinkage int32_t vbi_send(int32_t id, void *smsg, size_t slen,
			    void *rmsg, size_t rlen, struct vbi_msg_info *info,
			    struct vbi_msg_ctl *ctl);
/* read remote vb's memory */
extern asmlinkage int32_t vbi_vb_read_mem(struct vbi_mem_ctl *memCtl, uint32_t targetBoard);
extern asmlinkage int32_t vbi_vb_write_mem(struct vbi_mem_ctl *memCtl, uint32_t targetBoard);
extern asmlinkage void vbi_shell_start_debug(uint32_t  flags);
extern asmlinkage int32_t vbi_vb_mgmt(uint32_t cmd, uint32_t boardId,
				int32_t *outError, uint32_t flags, void * ctl);
extern asmlinkage int vbi_io_apic_ioctl(unsigned ioctl, unsigned arg1,
				unsigned arg2);
extern int32_t vbi_get_max_asid_vmmu(void);
#ifdef CONFIG_MIPS
extern int32_t vbi_tlb_flush_vmmu(struct vmmu64_config *config,
					 void *addr, size_t len);
#else
extern int32_t vbi_tlb_flush_vmmu(struct vmmuConfig *config, void *addr,
					size_t len);
#endif
#endif

#endif	/* _ASMLANGUAGE */
#endif  /* _VBI_SYSCALL_H */
