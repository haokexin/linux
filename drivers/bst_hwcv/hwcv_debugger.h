/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_HWCV_DEBUGGER_H__
#define __BST_HWCV_DEBUGGER_H__

#include "hwcv_core.h"

/*
 * struct hwcv_debugger - HWCV debugger information
 *
 * This structure represents a debugger to be created by the hwcv driver
 * or core.
 */
struct hwcv_debugger {
#ifdef CONFIG_BST_HWCV_DEBUG_FS
	/* Directory of debugfs file */
	struct dentry *debugfs_dir;
	struct list_head debugfs_entry_list;
	struct mutex debugfs_lock;
#endif

#ifdef CONFIG_BST_HWCV_PROC_FS
	/* Directory of procfs file */
	struct proc_dir_entry *procfs_dir;
	struct list_head procfs_entry_list;
	struct mutex procfs_lock;
#endif
};

/*
 * struct hwcv_debugger_list - debugfs/procfs info list entry
 *
 * This structure represents a debugfs/procfs file to be created by the hwcv
 * driver or core.
 */
struct hwcv_debugger_list {
	/* File name */
	const char *name;
	/*
	 * Show callback. &seq_file->private will be set to the &struct
	 * hwcv_debugger_node corresponding to the instance of this info
	 * on a given &struct hwcv_debugger.
	 */
	int (*show)(struct seq_file *seq, void *data);
	/*
	 * Write callback. &seq_file->private will be set to the &struct
	 * hwcv_debugger_node corresponding to the instance of this info
	 * on a given &struct hwcv_debugger.
	 */
	ssize_t (*write)(struct file *file, const char __user *ubuf, size_t len,
			 loff_t *offp);
	/* Procfs/Debugfs private data. */
	void *data;
};

/*
 * struct hwcv_debugger_node - Nodes for debugfs/procfs
 *
 * This structure represents each instance of procfs/debugfs created from the
 * template.
 */
struct hwcv_debugger_node {
	struct hwcv_debugger *debugger;

	/* template for this node. */
	const struct hwcv_debugger_list *info_ent;

	/* Each Procfs/Debugfs file. */
#ifdef CONFIG_BST_HWCV_DEBUG_FS
	struct dentry *dent;
#endif

#ifdef CONFIG_BST_HWCV_PROC_FS
	struct proc_dir_entry *pent;
#endif

	struct list_head list;
};

#ifdef CONFIG_BST_HWCV_DEBUG_FS
int hwcv_debugfs_init(void);
int hwcv_debugfs_remove(void);
#else
static inline int hwcv_debugfs_remove(void)
{
	return 0;
}

static inline int hwcv_debugfs_init(void)
{
	return 0;
}
#endif /* #ifdef CONFIG_BST_HWCV_DEBUG_FS */

#ifdef CONFIG_BST_HWCV_PROC_FS
int hwcv_procfs_remove(void);
int hwcv_procfs_init(void);
#else
static inline int hwcv_procfs_remove(void)
{
	return 0;
}

static inline int hwcv_procfs_init(void)
{
	return 0;
}
#endif /* #ifdef CONFIG_BST_HWCV_PROC_FS */

#endif /* #ifndef _HWCV_DEBUGGER_H_ */
