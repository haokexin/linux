/*
 * procfs.c - Wind River hypervisor procfs entries
 *
 * Copyright (c) 2011 Wind River Systems, Inc.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/wrhv.h>

#include <vbi/vbi.h>

static char hypinfo_name[] = "hypinfo";
static const char *wind_name = WINDRIVER_NAME;

struct proc_dir_entry *wrhv_procfs_root;
EXPORT_SYMBOL(wrhv_procfs_root);

/*
 * Implement a seq_file to display hypinfo for all VBs
 */
static int show_hypinfo(struct seq_file *m, void *v)
{
	struct vb_config *config = v;

	seq_printf(m, "board          : %u\n", config->boardID);
	seq_printf(m, "name           : %s\n", config->board_name);
	seq_printf(m, "type           : %u\n", config->board_type);
	seq_printf(m, "cores          : %u\n", config->numCores);
	seq_printf(m, "phys mem size  : 0x%X\n", config->phys_mem_size);
	seq_printf(m, "bootline       : %s\n", config->bootLine);
	seq_printf(m, "num devices    : %u\n", config->numDevices);
	seq_printf(m, "num interrupts : %u\n", config->num_ints);
	seq_printf(m, "boot count     : %u\n", config->boot_count);
	seq_printf(m, "core id        : %u\n", config->coreId);
	seq_printf(m, "num shared mem : %u\n", config->num_sm);
	seq_printf(m, "\n");

	return 0;
}

static void *hyp_start(struct seq_file *m, loff_t *pos)
{
	int vb = (*pos) + 1;
	struct vb_config *config = kmalloc(sizeof(struct vb_config),
					   GFP_KERNEL);
	if (!config)
		return NULL;
	if (wrhv_get_vb_config(vb, config) < 0)
		return NULL;

	return config;
}

static void *hyp_next(struct seq_file *m, void *v, loff_t *pos)
{
	int vb;
	struct vb_config *config = v;

	++(*pos);
	vb = (*pos) + 1;

	if (wrhv_get_vb_config(vb, config) < 0)
		return NULL;

	return config;
}

static void hyp_stop(struct seq_file *m, void *v)
{
	kfree(v);
}

static const struct seq_operations hypinfo_op = {
	.start  = hyp_start,
	.next   = hyp_next,
	.stop   = hyp_stop,
	.show   = show_hypinfo,
};

static int hypinfo_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &hypinfo_op);
}

static const struct file_operations procfs_hypinfo_operations = {
	.open           = hypinfo_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = seq_release,
};

/*
 * Safety Profile Hypervisor VBI statistic counts
 */
static char vbistat_name[] = "vbistat";
int vbistat_verbose;

/* variable to identify difference safety profile hypervisor version */
int safety_hyp_version = SAFETY_HYP_VER_NONE;

struct vbi_api_stats vbistat[] = {
	/* functions implemented in vbi.h */
	VBI_API_STATS_INIT(vbi_send_vcore_vioapic_irq),
	VBI_API_STATS_INIT(vbi_redir_vioapic_irq),
	VBI_API_STATS_INIT(vbi_show_shmem),
	VBI_API_STATS_INIT(vbi_show_stat),
	VBI_API_STATS_INIT(vbi_show_ctrl),
	VBI_API_STATS_INIT(vbi_show_cfg),
	VBI_API_STATS_INIT(vbi_show_mem),

	/* functions implemented in syscall.h */
	VBI_API_STATS_INIT(vbi_set_mem_attr),
	VBI_API_STATS_INIT(vbi_get_mem_attr),
	VBI_API_STATS_INIT(vbi_kputs),
	VBI_API_STATS_INIT(vbi_kputc),
	VBI_API_STATS_INIT(vbi_vb_restart),
	VBI_API_STATS_INIT(vbi_vb_resume),
	VBI_API_STATS_INIT(vbi_vb_suspend),
	VBI_API_STATS_INIT(vbi_ctx_ctl),
	VBI_API_STATS_INIT(core_vbi_end),

	/* optionsl vbi */
	VBI_API_STATS_INIT(vbi_vb_remote),
	VBI_API_STATS_INIT(vbi_ns_lookup),
	VBI_API_STATS_INIT(vbi_send),
	VBI_API_STATS_INIT(vbi_receive),
	VBI_API_STATS_INIT(vbi_reply),
	VBI_API_STATS_INIT(vbi_vb_write_reg),
	VBI_API_STATS_INIT(vbi_vb_read_reg),
	VBI_API_STATS_INIT(vbi_vb_write_mem),
	VBI_API_STATS_INIT(vbi_vb_read_mem),
	VBI_API_STATS_INIT(vbi_get_exc_offset),
	VBI_API_STATS_INIT(vbi_set_exc_offset),
	VBI_API_STATS_INIT(vbi_vtlb_op),
	VBI_API_STATS_INIT(vbi_vb_find_board_config),
	VBI_API_STATS_INIT(vbi_disp_vioapic),
	VBI_API_STATS_INIT(vbi_disp_status_regs),
	VBI_API_STATS_INIT(vbi_disp_ctrl_regs),
	VBI_API_STATS_INIT(vbi_show_config_page_map),
	VBI_API_STATS_INIT(vbi_shell_start_debug),
	VBI_API_STATS_INIT(vbi_ns_register),
	VBI_API_STATS_INIT(vbi_ns_unregister),
	VBI_API_STATS_INIT(vbi_vb_mgmt),
	VBI_API_STATS_INIT(vbi_di_eoi),
	VBI_API_STATS_INIT(vbi_send_vioapic_irq),
	VBI_API_STATS_INIT(vbi_dev_count),
	VBI_API_STATS_INIT(vbi_get_dev),
	VBI_API_STATS_INIT(vbi_get_dev_interrupt),
	VBI_API_STATS_INIT(vbi_get_dev_registers),
	VBI_API_STATS_INIT(vbi_get_dev_device_tree_source),
	VBI_API_STATS_INIT(vbi_vb_move),
	VBI_API_STATS_INIT(vbi_vb_priority_set),
	VBI_API_STATS_INIT(vbi_io_apic_op),
	VBI_API_STATS_INIT(vbi_ns_op),
	VBI_API_STATS_INIT(vbi_find_shmem),
	VBI_API_STATS_INIT(vbi_find_mem),
	VBI_API_STATS_INIT(vbi_vb_find_ram_size),
	VBI_API_STATS_INIT(vbi_io_apic_ioctl),
	VBI_API_STATS_INIT(vbi_get_max_asid_vmmu),
	VBI_API_STATS_INIT(vbi_get_vioapic_addr),
	VBI_API_STATS_INIT(vbi_tlb_load_vmmu),
	VBI_API_STATS_INIT(vbi_tlb_flush_vmmu),
	VBI_API_STATS_INIT(vbi_vb_create),
	VBI_API_STATS_INIT(vbi_vb_delete),
	VBI_API_STATS_INIT(vbi_board_simple_config_get),
	VBI_API_STATS_INIT(vbi_board_config_get),
	VBI_API_STATS_INIT(vbi_set_vb_priority),
	VBI_API_STATS_INIT(vbi_end),
};

static int vbistat_proc_show(struct seq_file *m, void *v)
{
	int i;

	switch (safety_hyp_version) {

	case SAFETY_HYP_VER_STD:
		seq_printf(m, "Safety Profile ");
		break;

	case SAFETY_HYP_VER_DEBUG:
		seq_printf(m, "Safety Profile Debug Version ");
		break;
	}
	seq_printf(m, "Hypervisor API Call Counts:\n");

	for (i = 0; i < wrhv_core_vbi_end; i++)
		seq_printf(m, "%s %d\n", vbistat[i].name, vbistat[i].count);

	/* optional vbi */
	seq_printf(m, "\n");
	seq_printf(m, "Optional VBI Call Counts:\n");
	seq_printf(m, "Core VBI only is ");
	if (is_corevbi_only())
		seq_printf(m, "enabled\n");
	else
		seq_printf(m, "not enabled\n");

	for (; i < wrhv_vbi_end; i++)
		seq_printf(m, "%s %d\n", vbistat[i].name, vbistat[i].count);
	seq_printf(m, "\n");

	return 0;
}
static int vbistat_open(struct inode *inode, struct file *file)
{
	return single_open(file, vbistat_proc_show, NULL);
}

/* reset stats counter, code template is from linux/fs/proc/base.c */
static ssize_t vbistat_write(struct file *file, const char __user *buf,
				size_t count, loff_t *offs)
{
	char c;
	int i;

	if (copy_from_user(&c, buf, 1))
		return -EFAULT;

	switch (c) {

	case '0':
		/* clear statistic counter */
		for (i = 0; i < wrhv_vbi_end; i++)
			vbistat[i].count = 0;
		break;

	case 'v':
		/* printk message whenever an unsupported VBI is called */
		vbistat_verbose = 1;
		break;

	case 'q':
		/* quiet printk message whenever an unsupported VBI is called */
		vbistat_verbose = 0;
		break;

	default:
		/* help message */
		printk(KERN_INFO "\n");
		printk(KERN_INFO "echo 0 > /proc/%s/%s to clear counters\n",
				wind_name,  vbistat_name);
		printk(KERN_INFO "echo v > /proc/%s/%s to print warning message\n",
				wind_name, vbistat_name);
		printk(KERN_INFO "echo q > /proc/%s/%s to quiet warning message\n",
				wind_name, vbistat_name);
		break;

	}

	return count;
}

static const struct file_operations procfs_vbistat_operations = {
	.open    = vbistat_open,
	.read    = seq_read,
	.write   = vbistat_write,
	.llseek  = seq_lseek,
	.release = seq_release,
};

void wrhv_init_procfs(void)
{
	struct proc_dir_entry *hypinfo_procfs = NULL;
	struct proc_dir_entry *vbistat_procfs = NULL;

	wrhv_procfs_root = proc_mkdir(wind_name, NULL);
	if (!wrhv_procfs_root) {
		printk(KERN_CRIT "ERROR. unable to create /proc/%s", wind_name);
		return;
	}

	hypinfo_procfs = proc_create(hypinfo_name, 0,
				   wrhv_procfs_root,
				   &procfs_hypinfo_operations);
	if (!hypinfo_procfs) {
		printk(KERN_WARNING "Cannot create %s/%s\n",
		       wind_name,
		       hypinfo_name);
	}

	/* Safety Profile VBI statistic counts */
	vbistat_procfs = proc_create(vbistat_name, 0,
				wrhv_procfs_root,
				&procfs_vbistat_operations);

	if (!vbistat_procfs) {
		printk(KERN_WARNING "Cannot create %s/%s\n",
		       wind_name,
		       vbistat_name);
	}

}
