/*
 * BRIEF DESCRIPTION
 *
 * Definitions for the PRAMFS filesystem.
 *
 * Copyright 2009-2010 Marco Stornelli <marco.stornelli@gmail.com>
 * Copyright 2003 Sony Corporation
 * Copyright 2003 Matsushita Electric Industrial Co., Ltd.
 * 2003-2004 (c) MontaVista Software, Inc. , Steve Longerbeam
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#ifndef __PRAM_H
#define __PRAM_H

#include <linux/buffer_head.h>
#include <linux/pram_fs.h>
#include <linux/pram_fs_sb.h>
#include <linux/crc16.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include "wprotect.h"

/*
 * Debug code
 */
#ifdef pr_fmt
#undef pr_fmt
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#endif

#define pram_dbg(s, args...)		pr_debug(s, ## args)
#define pram_err(sb, s, args...)	pram_error_mng(sb, s, ## args)
#define pram_warn(s, args...)		pr_warning(s, ## args)
#define pram_info(s, args...)		pr_info(s, ## args)

/* Function Prototypes */
extern void pram_error_mng(struct super_block * sb, const char * fmt, ...);
extern int pram_get_and_update_block(struct inode *inode, sector_t iblock,
				     struct buffer_head *bh, int create);

static inline int pram_readpage(struct file *file, struct page *page)
{
	return block_read_full_page(page, pram_get_and_update_block);
}

/* file.c */
extern ssize_t pram_direct_IO(int rw, struct kiocb *iocb,
			  const struct iovec *iov,
			  loff_t offset, unsigned long nr_segs);
extern int pram_mmap(struct file *file, struct vm_area_struct *vma);

#define pram_set_bit			ext2_set_bit
#define pram_clear_bit			ext2_clear_bit
#define pram_find_next_zero_bit		ext2_find_next_zero_bit

#define clear_opt(o, opt)	(o &= ~PRAM_MOUNT_##opt)
#define set_opt(o, opt)		(o |= PRAM_MOUNT_##opt)
#define test_opt(sb, opt)	(((struct pram_sb_info *)sb->s_fs_info)->s_mount_opt & \
				 PRAM_MOUNT_##opt)

/* balloc.c */
extern void pram_init_bitmap(struct super_block *sb);
extern void pram_free_block(struct super_block *sb, unsigned long blocknr);
extern int pram_new_block(struct super_block *sb, unsigned long *blocknr, int zero);
extern unsigned long pram_count_free_blocks(struct super_block *sb);

/* dir.c */
extern int pram_add_link(struct dentry *dentry, struct inode *inode);
extern int pram_remove_link(struct inode *inode);

/* namei.c */
extern struct dentry *pram_get_parent(struct dentry *child);

/* inode.c */
extern int pram_alloc_blocks(struct inode *inode, int file_blocknr, int num);
extern u64 pram_find_data_block(struct inode *inode,
					 unsigned long file_blocknr);

extern struct inode *pram_iget(struct super_block *sb, unsigned long ino);
extern void pram_put_inode(struct inode *inode);
extern void pram_evict_inode(struct inode *inode);
extern struct inode *pram_new_inode(struct inode *dir, int mode);
extern int pram_update_inode(struct inode *inode);
extern int pram_write_inode(struct inode *inode, struct writeback_control *wbc);
extern void pram_dirty_inode(struct inode *inode);
extern int pram_notify_change(struct dentry *dentry, struct iattr *attr);


/* super.c */
#ifdef CONFIG_PRAMFS_TEST
extern struct pram_super_block *get_pram_super(void);
#endif
extern struct super_block *pram_read_super(struct super_block *sb,
					      void *data,
					      int silent);
extern int pram_statfs(struct dentry *d, struct kstatfs *buf);
extern int pram_remount(struct super_block *sb, int *flags, char *data);

/* symlink.c */
extern int pram_block_symlink(struct inode *inode,
			       const char *symname, int len);

/* Inline functions start here */

static inline int pram_calc_checksum(u8 *data, int n)
{
	u16 crc = 0;
	crc = crc16(~0, (__u8 *)data + sizeof(__be16), n - sizeof(__be16));
	if (*((__be16 *)data) == cpu_to_be16(crc))
		return 0;
	else
		return 1;
}

/* If this is part of a read-modify-write of the super block,
   pram_memunlock_super() before calling! */
static inline struct pram_super_block *
pram_get_super(struct super_block *sb)
{
	struct pram_sb_info *sbi = (struct pram_sb_info *)sb->s_fs_info;
	return (struct pram_super_block *)sbi->virt_addr;
}

static inline struct pram_super_block *
pram_get_redund_super(struct super_block *sb)
{
	struct pram_sb_info *sbi = (struct pram_sb_info *)sb->s_fs_info;
	return (struct pram_super_block *)(sbi->virt_addr + PRAM_SB_SIZE);
}

static inline void *
pram_get_bitmap(struct super_block *sb)
{
	struct pram_super_block *ps = pram_get_super(sb);
	return (void *)ps + be64_to_cpu(ps->s_bitmap_start);
}

/* If this is part of a read-modify-write of the inode metadata,
   pram_memunlock_inode() before calling! */
static inline struct pram_inode *
pram_get_inode(struct super_block *sb, u64 ino)
{
	struct pram_super_block *ps = pram_get_super(sb);
	return ino ? (struct pram_inode *)((void *)ps + ino) : NULL;
}

static inline ino_t
pram_get_inodenr(struct super_block *sb, struct pram_inode *pi)
{
	struct pram_super_block *ps = pram_get_super(sb);
	return (ino_t)((unsigned long)pi - (unsigned long)ps);
}

static inline u64
pram_get_block_off(struct super_block *sb, unsigned long blocknr)
{
	struct pram_super_block *ps = pram_get_super(sb);
	return (u64)(be64_to_cpu(ps->s_bitmap_start) +
			     (blocknr << sb->s_blocksize_bits));
}

static inline unsigned long
pram_get_blocknr(struct super_block *sb, u64 block)
{
	struct pram_super_block *ps = pram_get_super(sb);
	return (block - be64_to_cpu(ps->s_bitmap_start)) >> sb->s_blocksize_bits;
}

/* If this is part of a read-modify-write of the block,
   pram_memunlock_block() before calling! */
static inline void *
pram_get_block(struct super_block *sb, u64 block)
{
	struct pram_super_block *ps = pram_get_super(sb);
	return block ? ((void *)ps + block) : NULL;
}

struct pram_inode_vfs {
#ifdef CONFIG_PRAMFS_XATTR
	/*
	 * Extended attributes can be read independently of the main file
	 * data. Taking i_mutex even when reading would cause contention
	 * between readers of EAs and writers of regular file data, so
	 * instead we synchronize on xattr_sem when reading or changing
	 * EAs.
	 */
	struct rw_semaphore xattr_sem;
#endif
	/*
	 * truncate_mutex is for serialising the truncate path against
	 * get/update block.
	 */
	struct mutex truncate_mutex;
	struct mutex i_meta_mutex;
	struct inode vfs_inode;
};

static inline struct pram_inode_vfs *PRAM_I(struct inode *inode)
{
	return container_of(inode, struct pram_inode_vfs, vfs_inode);
}

/*
 * Inodes and files operations
 */

/* dir.c */
extern struct file_operations pram_dir_operations;

/* file.c */
extern struct inode_operations pram_file_inode_operations;
extern struct file_operations pram_file_operations;
extern struct file_operations pram_xip_file_operations;

/* inode.c */
extern struct address_space_operations pram_aops;
extern struct address_space_operations pram_aops_xip;

/* namei.c */
extern struct inode_operations pram_dir_inode_operations;

/* symlink.c */
extern struct inode_operations pram_symlink_inode_operations;

extern struct backing_dev_info pram_backing_dev_info;

#endif	/* __PRAM_H */
