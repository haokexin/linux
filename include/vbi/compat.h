/*
 * compat.h - virtual board compatibility support definitions
 *
 * Copyright (c) 2009 Wind River Systems, Inc.
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

/*
 * This introduces some of the non-linux like things into scope for
 * those VBI users who have an expectation that they will be present.
 * None of the default linux VBI code needs to include this file.
 * Only those source files which expect default/legacy VBI API
 * content need to include this file.
 */


#ifndef _VBI_COMPAT_H
#define _VBI_COMPAT_H

/*
 * These were particularly confusing, since there are similarly named
 * macros used to map the bits within these structs.  Eeech.
 */
#define VB_CONTROL	struct vb_control
#define VB_CONFIG	struct vb_config
#define VB_STATUS	struct vb_status

#define VB_INT_INFO		struct vb_int_info
#define VB_MEM_INFO		struct vb_mem_info
#define VB_SM_INFO		struct vb_sm_info
#define VB_DEV_INFO		struct vb_dev_info
#define VB_DEV_INT_INFO		struct vb_dev_int_info
#define VB_DEV_REGSET_INFO	struct vb_dev_regset_info

#define VBI_MSG_INFO		struct vbi_msg_info
#define VBI_MSG_CTL		struct vbi_msg_ctl
#define VBI_MEM_CTL		struct vbi_mem_ctl

#define VBI_CTX_STATS		struct vbi_ctx_stats
#define VBI_VTLB_OP		struct vbi_vtlb_op
#define VBI_VTLB_CONTROL	struct vbi_vtlb_control
#define VBI_VTLB_CR3_CACHE	struct vbi_vtlb_cr3_cache

#define VBI_CLK_HOOK_INFO	struct vbi_clk_hook
#define VBI_CLK_HOOK_INFO_PTR	struct vbi_clk_hook *

#define VB_ARCH_CONTROL_REGS	struct vb_arch_ctrl_regs
#define VB_ARCH_STATUS_REGS	struct vb_arch_stat_regs

#define VIOAPIC_ENTRY		struct vioapic_entry
#define VIOAPIC_ID		union vioapic_id
#define VIOAPIC_VERSION		union vioapic_version
#define VIOAPIC_REDIR_LOW	union vioapic_redir_low
#define VIOAPIC_REDIR_HIGH	union vioapic_redir_high

#define VBI_MSG_HDR		struct vbi_msg_hdr
#define MSG_PDC			struct msg_pdc
#define MSG_PDC_REPLY		struct msg_pdc_reply

#define VB_TIMESTAMP		uint64_t
#define vbiStatus_t		int32_t
#define vbiIrq_t		int32_t
#define vbiVb_t			uint32_t
#define vbiCore_t		int32_t
#define vbiCtx_t		int32_t
#define vbiPhysAddr_t		uint64_t
#define vbiGuestPhysAddr_t	void *
#define vbiIntState_t		int32_t
#define vbiMemAttr_t		int32_t
#define vbiVector_t		int32_t
#define vbiCoreSet_t		uint32_t
#define vbiRegSet_t		uint32_t
#define virtAddr_t		uint64_t

/*
 * struct fields and similar
 */
#define vbControlRegs		vb_control_regs
#define vbStatusRegs		vb_status_regs
#define vbStatus		vb_status
#define vbControl		vb_control

#define vioapicLow		vioapic_low
#define vioapicHigh		vioapic_high

#define wrhvControl		wr_control
#define wrhvConfig		wr_config
#define wrhvStatus		wr_status
#define wrhvVbStatus		wr_vb_status
#define wrhvVbControl		wr_vb_control
#define wrhvVbConfig		wr_vb_config

#define memoryAliasSize		mem_alias_size
#define memoryAliasAddress	mem_alias_addr
#define physicalMemorySize	phys_mem_size

#define numInt			num_int
#define numSm			num_sm
#define numMem			num_mem
#define sizeIn			size_in
#define sizeOut			size_out

#define resetPC			reset_pc
#define bootCount		boot_count
#define boardName		board_name
#define boardType		board_type

#define tickTimerFrequency	tick_freq
#define tickCount		tick_count
#define timeStampFrequency	stamp_freq

#define intDisable		irq_disable
#define newIntDisable		next_irq_disable
#define oldIntDisable		prev_irq_disable
#define intLevelDisable		level_irq_disable
#define intName			irq_name
#define intNumber		irq_num
#define intDirection		irq_dir
#define intPending		irq_pend
#define intPendingType		irq_pend_type

#define vbiPciDevice_t		struct vbi_pci_device
#define vbSimpleInformation_t	struct vb_simple_information
#define vbInformation_t		struct vb_information

/*
 * function calls, etc.
 */
#define vbiVbSuspend		vbi_vb_suspend
#define vbiVbResume		vbi_vb_resume
#define vbiVbRestart		vbi_vb_restart
#define vbiVbReset		vbi_vb_reset
#define vbiVbRemote		vbi_vb_remote

#ifdef CONFIG_WRHV_CERT
#define vbiSchedControlOp	vbi_sched_control_op
#define vbiPortOp		vbi_port_op
#endif

#define vbiVbRegisterRead	vbi_vb_read_reg
#define vbiVbRegisterWrite	vbi_vb_write_reg
#define vbiVbMemoryRead		vbi_vb_read_mem
#define vbiVbMemoryWrite	vbi_vb_write_mem
#define vbiVbBoardConfigFind	vbi_vb_find_board_config

#define vbiVcoreIntRed_op	vbi_vcore_irq_redirect
#define vbiDebugShellStart	vbi_shell_start_debug
#define vbiIoapicOp		vbi_io_apic_op
#define vbiIoapicIoctl		vbi_io_apic_ioctl
#define vbiHyIoctl		vbi_hy_ioctl
#define vbiCtxctl		vbi_ctx_ctl
#define vbiVbMgmt		vbi_vb_mgmt

#define vbiSend			vbi_send
#define vbiReceive		vbi_receive
#define vbiReply		vbi_reply
#define vbiPanic		vbi_panic
#define vbiPs			vbi_ps
#define vbiKputs		vbi_kputs
#define vbiKputc		vbi_kputc

#define vbiIcacheFlush		vbi_flush_icache
#define vbiDcacheFlush		vbi_flush_dcache
#define vbiCacheTextUpdate	vbi_update_text_cache
#define vbiTlbFlush		vbi_flush_tlb
#define vbiVtlbOp		vbi_vtlb_op

#define vbiDirectInterruptEOI	vbi_direct_IRQ_EOI
#define vbiIntVCoreUnlock	vbi_vcore_irq_unlock
#define vbiIntVCoreLock		vbi_vcore_irq_lock
#define vbiIntVCoreStateGet	vbi_vcore_irq_state
#define vbiExcBaseSet		vbi_set_exc_base
#define vbiExcOffsetsSet	vbi_set_exc_offset
#define vbiExcOffsetsGet	vbi_get_exc_offset
#define vbiMemAttrSet		vbi_set_mem_attr
#define vbiMemAttrGet		vbi_get_mem_attr

#define vbiVmmuConfig		vbi_config_vmmu
#define vbiVmmuEnable		vbi_enable_vmmu
#define vbiVmmuDisable		vbi_disable_vmmu
#define vbiVmmuTlbLoad		vbi_tlb_load_vmmu
#define vbiVmmuTlbFlush		vbi_tlb_flush_vmmu

#define vbiNsOp			vbi_ns_op
#define vbiNsLookup		vbi_ns_lookup
#define vbiNsRegister		vbi_ns_register
#define vbiNsUnregister		vbi_ns_unregister

#define vbiConfigShow		vbi_show_cfg
#define vbiControlShow		vbi_show_ctrl
#define vbiCtrlRegsDisplay	vbi_disp_ctrl_regs
#define vbiInterruptsShow	vbi_show_irq
#define vbiMemoryShow		vbi_show_mem
#define vbiSharedMemoryShow	vbi_show_shmem
#define vbiStatusShow		vbi_show_stat
#define vbiStsRegsDisplay	vbi_disp_status_regs

#define vbiVersionMajor		vbi_version_major
#define vbiVersionMinor		vbi_version_minor
#define vbiVersionMaint		vbi_version_maint
#define vbiCreationDate		vbi_creation_date
#define vbiRuntimeVersion	vbi_runtime_version
#define vbiRuntimeName		vbi_runtime_name
#define vbiVersion		vbi_version

#define vbiExcStub		vbi_exc_stub
#define vbiGuestDmaAddrGet	vbi_get_guest_dma_addr
#define vbiGuestPhysToPhysAddr	vbi_guest_phys_to_phys
#define vbiIdle			vbi_idle
#define vbiInit			vbi_init
#define vbiMmuAttrSet		vbi_set_mmu_attr

#define vbiCorePrvMemFind	vbi_find_core_prv_mem
#define vbiIntVecFind		vbi_find_irq
#define vbiMemRegionFind	vbi_find_mem
#define vbiShmemRegionFind	vbi_find_shmem

#define vbiVioapicAddrGet	vbi_get_vioapic_addr
#define vbiVioapicDisplay	vbi_disp_vioapic
#define vbiVioapicIntAck	vbi_ack_vioapic_irq
#define vbiVioapicIntMask	vbi_mask_vioapic_irq
#define vbiVioapicIntRedirect	vbi_redir_vioapic_irq
#define vbiVioapicIntPending	vbi_get_pending_vioapic_irq
#define vbiVioapicIntSend	vbi_send_vioapic_irq
#define vbiVioapicIntUnmask	vbi_unmask_vioapic_irq
#define vbiVioapicVcoreIntSend	vbi_send_vcore_vioapic_irq
#define vbiVioapicVectorGet	vbi_get_vioapic_vec
#define vbiVioapicVectorSet	vbi_set_vioapic_vec

#define vbiDeviceCount		vbi_dev_count
#define vbiDeviceGet		vbi_get_dev
#define vbiDeviceInterruptGet	vbi_get_dev_interrupt
#define vbiDeviceRegisterSetGet	vbi_get_dev_registers

#define vbiIntControllerDone	vbi_irq_controller_done
#define vbiIntEnable		vbi_irq_enable

#define vbiVmmuMaxAsid		vbi_get_max_asid_vmmu

#define vbiCtxLoad		vbi_ctx_load

#define vbiVbCreate		vbi_vb_create
#define vbiVbDelete		vbi_vb_delete
#define vbiBoardSimpleConfigGet	vbi_board_simple_config_get
#define vbiBoardConfigGet	vbi_board_config_get
#define vbiVbMove		vbi_vb_move
#define vbiVbPrioritySet	vbi_vb_priority_set

#define intrDeviceChannelBuffer	intr_device_channel_buffer
#define vbiPdcInit		vbi_pdc_init               

#endif  /* _VBI_COMPAT_H */
