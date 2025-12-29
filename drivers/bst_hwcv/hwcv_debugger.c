// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#define pr_fmt(fmt) "hwcv_debugger: " fmt

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/syscalls.h>
#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "hwcv_core.h"
#include "hwcv_debugger.h"
#include "hwcv_dma_buf.h"
#include "hwcv_mm.h"

#define HWCV_DEBUGGER_ROOT_NAME "bst_hwcv"

#define UNIT_ROW_LENGTH 15
#define SEPARATOR_LINE "---------------"

static int hwcv_version_show(struct seq_file *m, void *data)
{
	seq_printf(m, "v%s\n", DRIVER_VERSION);

	return 0;
}

static int hwcv_process_show(struct seq_file *m, void *data)
{
	int id;
	struct hwcv_session *session;
	struct hwcv_session_manager *session_manager;

	session_manager = hwcv_drvdata->session_manager;

	mutex_lock(&session_manager->lock);

	seq_printf(m, "%-8s %-8s %-50s\n", "id", "pid", "cmd");
	seq_puts(m, "--------------------------------------------------");
	seq_puts(m, "--------------------------------------------------\n");

	idr_for_each_entry(&session_manager->ctx_id_idr, session, id)
		seq_printf(m, "%-8d %-8d %-50s\n", id, session->tgid,
			   session->pname);

	mutex_unlock(&session_manager->lock);

	return 0;
}

static int hwcv_memory_show(struct seq_file *m, void *data)
{
	int id;
	struct hwcv_mm *mm;
	struct hwcv_buf *buf;

	mm = hwcv_drvdata->mm;

	mutex_lock(&mm->lock);

	seq_printf(m, "%-8s %-8s %-8s %-8s %-8s %-15s %-15s %-10s %-10s\n",
		   "type", "handle", "fd", "pid", "users", "phy_addr",
		   "dma_addr", "bytesused", "length");
	seq_puts(m, "--------------------------------------------------");
	seq_puts(m, "--------------------------------------------------\n");

	idr_for_each_entry(&mm->memory_idr, buf, id) {
		seq_printf(
			m,
			"%-8d %-8d %-8d %-8d %-8d 0x%-13llx 0x%-13llx %-10d %-10d\n",
			buf->type, buf->id, buf->fd, buf->session->tgid,
			mm->ops->num_users(buf->mem_priv), buf->phys_addr,
			buf->dma_addr, buf->bytesused, buf->length);
	}

	mutex_unlock(&mm->lock);

	return 0;
}

void print_separator_line(struct seq_file *m)
{
	seq_printf(m, "+-%-*s-+-%-*s-+-%-*s-+-%-*s-+-%-*s-+-%-*s-+-%-*s-+\n",
		   UNIT_ROW_LENGTH, SEPARATOR_LINE, UNIT_ROW_LENGTH,
		   SEPARATOR_LINE, UNIT_ROW_LENGTH, SEPARATOR_LINE,
		   UNIT_ROW_LENGTH, SEPARATOR_LINE, UNIT_ROW_LENGTH,
		   SEPARATOR_LINE, UNIT_ROW_LENGTH, SEPARATOR_LINE,
		   UNIT_ROW_LENGTH, SEPARATOR_LINE);
}

void hwcv_dump_ktime(struct seq_file *m, struct hwcv_ktime *kt,
		     const char *row_name)
{
	seq_printf(
		m,
		"| %-*s | %-*llu | %-*llu | %-*llu | %-*llu | %-*llu | %-*llu |\n",
		UNIT_ROW_LENGTH, row_name, UNIT_ROW_LENGTH, kt->cur,
		UNIT_ROW_LENGTH, kt->max, UNIT_ROW_LENGTH, kt->min,
		UNIT_ROW_LENGTH, kt->average, UNIT_ROW_LENGTH, kt->sum,
		UNIT_ROW_LENGTH, kt->count);
}

void hwcv_dump_stat(struct seq_file *m, struct hwcv_stat *s,
		    const char *module_name)
{
	print_separator_line(m);

	seq_printf(m, "| %-*s | %-*s | %-*s | %-*s | %-*s | %-*s | %-*s |\n",
		   UNIT_ROW_LENGTH, module_name, UNIT_ROW_LENGTH, "cur",
		   UNIT_ROW_LENGTH, "max", UNIT_ROW_LENGTH, "min",
		   UNIT_ROW_LENGTH, "average", UNIT_ROW_LENGTH, "sum",
		   UNIT_ROW_LENGTH, "count");

	print_separator_line(m);

	hwcv_dump_ktime(m, &s->get_lock, "Get Lock");

	hwcv_dump_ktime(m, &s->config_reg, "Config Reg");

	hwcv_dump_ktime(m, &s->complete_frame, "Complete Frame");

	hwcv_dump_ktime(m, &s->notify_upper, "Notify Upper");

	hwcv_dump_ktime(m, &s->hw_cycle, "HW Cycle");

	print_separator_line(m);

	seq_puts(m, "\n");
}

static int hwcv_stat_show(struct seq_file *m, void *data)
{
	int i;
	struct hwcv_core *core;
	char gwarp_name[20];

	core = hwcv_drvdata->core;

	mutex_lock(&core->scaler_lock);

	hwcv_dump_stat(m, &core->scaler_stat, "SCALER(us)");

	mutex_unlock(&core->scaler_lock);

	for (i = 0; i < HWCV_MAX_GWARP_NUM; i++) {
		mutex_lock(&core->gwarp_lock[i]);

		snprintf(gwarp_name, sizeof(gwarp_name), "GWARP%d(us)", i);
		hwcv_dump_stat(m, &core->gwarp_stat[i], gwarp_name);

		mutex_unlock(&core->gwarp_lock[i]);
	}

	return 0;
}

static int hwcv_sys_reg_show(struct seq_file *m, void *data)
{
	struct hwcv_core *core;

	core = hwcv_drvdata->core;

	core->ops->debug_sys(core, m);

	return 0;
}

static int hwcv_scaler_reg_show(struct seq_file *m, void *data)
{
	struct hwcv_core *core;

	core = hwcv_drvdata->core;

	core->ops->debug_scaler(core, m);

	return 0;
}

static int hwcv_gwarp_reg_show(struct seq_file *m, void *data)
{
	struct hwcv_core *core;

	core = hwcv_drvdata->core;

	core->ops->debug_gwarp(core, m);

	return 0;
}

static struct hwcv_debugger_list hwcv_debugger_root_list[] = {
	{ "version", hwcv_version_show, NULL, NULL },
	{ "process", hwcv_process_show, NULL, NULL },
	{ "memory", hwcv_memory_show, NULL, NULL },
	{ "sys_reg", hwcv_sys_reg_show, NULL, NULL },
	{ "scaler_reg", hwcv_scaler_reg_show, NULL, NULL },
	{ "gwarp_reg", hwcv_gwarp_reg_show, NULL, NULL },
	{ "stat", hwcv_stat_show, NULL, NULL },
};

static ssize_t hwcv_debugger_write(struct file *file, const char __user *ubuf,
				   size_t len, loff_t *offp)
{
	struct seq_file *priv = file->private_data;
	struct hwcv_debugger_node *node = priv->private;

	if (node->info_ent->write)
		return node->info_ent->write(file, ubuf, len, offp);
	else
		return len;
}

#ifdef CONFIG_BST_HWCV_DEBUG_FS
static int hwcv_debugfs_open(struct inode *inode, struct file *file)
{
	struct hwcv_debugger_node *node = inode->i_private;

	return single_open(file, node->info_ent->show, node);
}

static const struct file_operations hwcv_debugfs_fops = {
	.owner = THIS_MODULE,
	.open = hwcv_debugfs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = hwcv_debugger_write,
};

static int hwcv_debugfs_remove_files(struct hwcv_debugger *debugger)
{
	struct hwcv_debugger_node *pos, *q;
	struct list_head *entry_list;

	mutex_lock(&debugger->debugfs_lock);

	/* Delete debugfs entry list */
	entry_list = &debugger->debugfs_entry_list;
	list_for_each_entry_safe(pos, q, entry_list, list) {
		if (pos->dent == NULL)
			continue;
		list_del(&pos->list);
		kfree(pos);
		pos = NULL;
	}

	/* Delete all debugfs node in this directory */
	debugfs_remove_recursive(debugger->debugfs_dir);
	debugger->debugfs_dir = NULL;

	mutex_unlock(&debugger->debugfs_lock);

	return 0;
}

static int hwcv_debugfs_create_files(const struct hwcv_debugger_list *files,
				     int count, struct dentry *root,
				     struct hwcv_debugger *debugger)
{
	int i;
	struct dentry *ent;
	struct hwcv_debugger_node *tmp;

	for (i = 0; i < count; i++) {
		tmp = kmalloc(sizeof(struct hwcv_debugger_node), GFP_KERNEL);
		if (tmp == NULL)
			goto MALLOC_FAIL;

		tmp->info_ent = &files[i];
		tmp->debugger = debugger;

		ent = debugfs_create_file(files[i].name, S_IFREG | 0444, root,
					  tmp, &hwcv_debugfs_fops);
		if (!ent) {
			pr_err("Cannot create /sys/kernel/debug/%pd/%s\n", root,
			       files[i].name);
			goto CREATE_FAIL;
		}

		tmp->dent = ent;

		mutex_lock(&debugger->debugfs_lock);
		list_add_tail(&tmp->list, &debugger->debugfs_entry_list);
		mutex_unlock(&debugger->debugfs_lock);
	}

	return 0;

CREATE_FAIL:
	kfree(tmp);
MALLOC_FAIL:
	hwcv_debugfs_remove_files(debugger);

	return -1;
}

int hwcv_debugfs_remove(void)
{
	struct hwcv_debugger *debugger;

	debugger = hwcv_drvdata->debugger;

	hwcv_debugfs_remove_files(debugger);

	return 0;
}

int hwcv_debugfs_init(void)
{
	int ret;
	struct hwcv_debugger *debugger;

	debugger = hwcv_drvdata->debugger;

	debugger->debugfs_dir =
		debugfs_create_dir(HWCV_DEBUGGER_ROOT_NAME, NULL);
	if (IS_ERR_OR_NULL(debugger->debugfs_dir)) {
		pr_err("failed on mkdir /sys/kernel/debug/%s\n",
		       HWCV_DEBUGGER_ROOT_NAME);
		debugger->debugfs_dir = NULL;
		return -EIO;
	}

	ret = hwcv_debugfs_create_files(hwcv_debugger_root_list,
					ARRAY_SIZE(hwcv_debugger_root_list),
					debugger->debugfs_dir, debugger);
	if (ret) {
		pr_err("Could not install hwcv_debugger_root_list debugfs\n");
		goto CREATE_FAIL;
	}

	return 0;

CREATE_FAIL:
	hwcv_debugfs_remove();

	return ret;
}
#endif /* #ifdef CONFIG_BST_HWCV_DEBUG_FS */

#ifdef CONFIG_BST_HWCV_PROC_FS
static int hwcv_procfs_open(struct inode *inode, struct file *file)
{
	struct hwcv_debugger_node *node = inode->i_private;

	return single_open(file, node->info_ent->show, node);
}

static const struct proc_ops hwcv_procfs_fops = {
	.proc_open = hwcv_procfs_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
	.proc_write = hwcv_debugger_write,
};

static int hwcv_procfs_remove_files(struct hwcv_debugger *debugger)
{
	struct hwcv_debugger_node *pos, *q;
	struct list_head *entry_list;

	mutex_lock(&debugger->procfs_lock);

	/* Delete procfs entry list */
	entry_list = &debugger->procfs_entry_list;
	list_for_each_entry_safe(pos, q, entry_list, list) {
		if (pos->pent == NULL)
			continue;
		list_del(&pos->list);
		kfree(pos);
		pos = NULL;
	}

	/* Delete all procfs node in this directory */
	proc_remove(debugger->procfs_dir);
	debugger->procfs_dir = NULL;

	mutex_unlock(&debugger->procfs_lock);

	return 0;
}

static int hwcv_procfs_create_files(const struct hwcv_debugger_list *files,
				    int count, struct proc_dir_entry *root,
				    struct hwcv_debugger *debugger)
{
	int i;
	struct proc_dir_entry *ent;
	struct hwcv_debugger_node *tmp;

	for (i = 0; i < count; i++) {
		tmp = kmalloc(sizeof(struct hwcv_debugger_node), GFP_KERNEL);
		if (tmp == NULL)
			goto MALLOC_FAIL;

		tmp->info_ent = &files[i];
		tmp->debugger = debugger;

		ent = proc_create_data(files[i].name, S_IFREG | 0444, root,
				       &hwcv_procfs_fops, tmp);
		if (!ent) {
			pr_err("Cannot create /proc/%s/%s\n",
			       HWCV_DEBUGGER_ROOT_NAME, files[i].name);
			goto CREATE_FAIL;
		}

		tmp->pent = ent;

		mutex_lock(&debugger->procfs_lock);
		list_add_tail(&tmp->list, &debugger->procfs_entry_list);
		mutex_unlock(&debugger->procfs_lock);
	}

	return 0;

CREATE_FAIL:
	kfree(tmp);
MALLOC_FAIL:
	hwcv_procfs_remove_files(debugger);
	return -1;
}

int hwcv_procfs_remove(void)
{
	struct hwcv_debugger *debugger;

	debugger = hwcv_drvdata->debugger;

	hwcv_procfs_remove_files(debugger);

	return 0;
}

int hwcv_procfs_init(void)
{
	int ret;
	struct hwcv_debugger *debugger;

	debugger = hwcv_drvdata->debugger;

	debugger->procfs_dir = proc_mkdir(HWCV_DEBUGGER_ROOT_NAME, NULL);
	if (IS_ERR_OR_NULL(debugger->procfs_dir)) {
		pr_err("failed on mkdir /proc/%s\n", HWCV_DEBUGGER_ROOT_NAME);
		debugger->procfs_dir = NULL;
		return -EIO;
	}

	ret = hwcv_procfs_create_files(hwcv_debugger_root_list,
				       ARRAY_SIZE(hwcv_debugger_root_list),
				       debugger->procfs_dir, debugger);
	if (ret) {
		pr_err("Could not install hwcv_debugger_root_list procfs\n");
		goto CREATE_FAIL;
	}

	return 0;

CREATE_FAIL:
	hwcv_procfs_remove();

	return ret;
}
#endif /* #ifdef CONFIG_BST_HWCV_PROC_FS */
