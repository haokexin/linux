#ifndef STATS_H
#define STATS_H

#ifndef _ASMLANGUAGE

#if defined(CONFIG_WRHV_COREVBI_ONLY)
/* header file required by the printk, which is used in VBISTAT_VERBOSE */
#include <linux/kernel.h>
#endif

#define VBI_API_STATS_INIT(x)	\
	[wrhv_##x] = {.count = 0, .name = #x}

#define VBI_API_STATS_DEF(x)	\
	wrhv_##x

enum wrhv_vbi {
	/* functions implemented in vbi.h */
	VBI_API_STATS_DEF(vbi_send_vcore_vioapic_irq),
	VBI_API_STATS_DEF(vbi_redir_vioapic_irq),
	VBI_API_STATS_DEF(vbi_show_shmem),
	VBI_API_STATS_DEF(vbi_show_stat),
	VBI_API_STATS_DEF(vbi_show_ctrl),
	VBI_API_STATS_DEF(vbi_show_cfg),
	VBI_API_STATS_DEF(vbi_show_mem),
	/* functions implemented in syscall.h */
	VBI_API_STATS_DEF(vbi_set_mem_attr),
	VBI_API_STATS_DEF(vbi_get_mem_attr),
	VBI_API_STATS_DEF(vbi_kputs),
	VBI_API_STATS_DEF(vbi_kputc),
	VBI_API_STATS_DEF(vbi_vb_restart),
	VBI_API_STATS_DEF(vbi_vb_resume),
	VBI_API_STATS_DEF(vbi_vb_suspend),
	VBI_API_STATS_DEF(vbi_ctx_ctl),
	VBI_API_STATS_DEF(core_vbi_end),
	/* optional vbi */
	VBI_API_STATS_DEF(vbi_vb_remote),
	VBI_API_STATS_DEF(vbi_ns_lookup),
	VBI_API_STATS_DEF(vbi_send),
	VBI_API_STATS_DEF(vbi_receive),
	VBI_API_STATS_DEF(vbi_reply),
	VBI_API_STATS_DEF(vbi_vb_write_reg),
	VBI_API_STATS_DEF(vbi_vb_read_reg),
	VBI_API_STATS_DEF(vbi_vb_write_mem),
	VBI_API_STATS_DEF(vbi_vb_read_mem),
	VBI_API_STATS_DEF(vbi_get_exc_offset),
	VBI_API_STATS_DEF(vbi_set_exc_offset),
	VBI_API_STATS_DEF(vbi_vtlb_op),
	VBI_API_STATS_DEF(vbi_vb_find_board_config),
	VBI_API_STATS_DEF(vbi_disp_vioapic),
	VBI_API_STATS_DEF(vbi_disp_status_regs),
	VBI_API_STATS_DEF(vbi_disp_ctrl_regs),
	VBI_API_STATS_DEF(vbi_show_config_page_map),
	VBI_API_STATS_DEF(vbi_shell_start_debug),
	VBI_API_STATS_DEF(vbi_ns_register),
	VBI_API_STATS_DEF(vbi_ns_unregister),
	VBI_API_STATS_DEF(vbi_vb_mgmt),
	VBI_API_STATS_DEF(vbi_di_eoi),
	VBI_API_STATS_DEF(vbi_send_vioapic_irq),
	VBI_API_STATS_DEF(vbi_dev_count),
	VBI_API_STATS_DEF(vbi_get_dev),
	VBI_API_STATS_DEF(vbi_get_dev_interrupt),
	VBI_API_STATS_DEF(vbi_get_dev_registers),
	VBI_API_STATS_DEF(vbi_get_dev_device_tree_source),
	VBI_API_STATS_DEF(vbi_vb_move),
	VBI_API_STATS_DEF(vbi_vb_priority_set),
	VBI_API_STATS_DEF(vbi_io_apic_op),
	VBI_API_STATS_DEF(vbi_ns_op),
	VBI_API_STATS_DEF(vbi_find_shmem),
	VBI_API_STATS_DEF(vbi_find_mem),
	VBI_API_STATS_DEF(vbi_vb_find_ram_size),
	VBI_API_STATS_DEF(vbi_io_apic_ioctl),
	VBI_API_STATS_DEF(vbi_get_max_asid_vmmu),
	VBI_API_STATS_DEF(vbi_get_vioapic_addr),
	VBI_API_STATS_DEF(vbi_tlb_load_vmmu),
	VBI_API_STATS_DEF(vbi_tlb_flush_vmmu),
	VBI_API_STATS_DEF(vbi_vb_create),
	VBI_API_STATS_DEF(vbi_vb_delete),
	VBI_API_STATS_DEF(vbi_board_simple_config_get),
	VBI_API_STATS_DEF(vbi_board_config_get),
	VBI_API_STATS_DEF(vbi_set_vb_priority),
	VBI_API_STATS_DEF(vbi_end),
};

struct vbi_api_stats {
	uint32_t count;
	char *name;
};

/* declared in linux/kernel/vbi/wrhv.c */
extern int vbistat_verbose;
extern struct vbi_api_stats vbistat[];
extern int safety_hyp_version;

#define SAFETY_HYP_VER_NONE	(0)
#define SAFETY_HYP_VER_STD	(1)
#define SAFETY_HYP_VER_DEBUG	(2)

#define VBISTAT_VERBOSE(vbi) \
do {									\
	int i = wrhv_##vbi;						\
	struct vbi_api_stats *p;					\
									\
	if (i >= wrhv_vbi_end) {					\
		pr_err("Wrong argument %s for VBISTAT_VERBOSE\n", #vbi);\
		break;							\
	}								\
	p  = &vbistat[i];						\
	/* limit output to maximum of 100 times */			\
	if ((vbistat_verbose) && (p->count < 100))			\
		printk(KERN_INFO "Unsupported vbi call %s\n", p->name);	\
	p->count++;							\
} while (0)

static inline int is_safety_hyp(void){
	return safety_hyp_version != SAFETY_HYP_VER_NONE;
}

#ifdef CONFIG_WRHV_COREVBI_ONLY
#define is_corevbi_only()	(1)
#else
#define is_corevbi_only()	(0)
#endif

#endif

#endif
