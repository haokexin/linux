/*
 * Advanced XIP File System for Linux - AXFS
 *   Readonly, compressed, and XIP filesystem for Linux systems big and small
 *
 * Copyright(c) 2008 Numonyx
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * Authors:
 *  Jared Hulbert <jaredeh@gmail.com>
 *
 * Project url: http://axfs.sourceforge.net
 *
 * axfs_bdev.c -
 *   Allows axfs to use block devices or has dummy functions if block
 *   device support is compiled out of the kernel.
 *
 */

#include <linux/axfs.h>
#include <linux/mount.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,10)
#else
#define CONFIG_BLOCK
#endif
#ifdef CONFIG_BLOCK
#include <linux/buffer_head.h>
#include <linux/namei.h>

int axfs_fill_super(struct super_block *sb, void *data, int silent);

int axfs_get_sb_bdev(struct file_system_type *fs_type, int flags,
		     const char *dev_name, struct axfs_super *sbi,
		     struct vfsmount *mnt, int *err)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,17)
	*err = get_sb_bdev(fs_type, flags, dev_name, sbi, axfs_fill_super, mnt);

	if (*err)
		return FALSE;
#else
	mnt->mnt_sb =
	    get_sb_bdev(fs_type, flags, dev_name, (void *)sbi, axfs_fill_super);
	if (IS_ERR(mnt->mnt_sb)) {
		*err = PTR_ERR(mnt->mnt_sb);
		return FALSE;
	}
#endif
	return TRUE;
}

void axfs_kill_block_super(struct super_block *sb)
{
	kill_block_super(sb);
}

/******************************************************************************
 *
 * axfs_copy_block_data
 *
 * Description: Helper function to read data from block device
 *
 * Parameters:
 *    (IN) sb - pointer to super block structure.
 *
 *    (IN) dst_addr - pointer to buffer into which data is to be read.
 *
 *    (IN) boffset - offset within block device
 *
 *    (IN) len - length of data to be read
 *
 * Returns:
 *     0 or error number
 *
 *****************************************************************************/
int axfs_copy_block(struct super_block *sb, void *dst_addr, u64 fsoffset,
		    u64 len)
{
	struct axfs_super *sbi = AXFS_SB(sb);
	u64 boffset = AXFS_FSOFFSET_2_DEVOFFSET(sbi, fsoffset);
	u64 blocks;
	u64 blksize = sb->s_blocksize;
	unsigned long dst;
	unsigned long src;
	sector_t block;
	size_t bytes;
	struct buffer_head *bh;
	u64 copied = 0;

	if (len == 0)
		return 0;

	blocks = len / blksize;
	if ((len % blksize) > 0)
		blocks += 1;

	while (copied < len) {
		/* Explicit casting for ARM linker errors. */
		block = (sector_t) boffset + (sector_t) copied;
		block /= (sector_t) blksize;
		bh = sb_bread(sb, block);
		src = (unsigned long)bh->b_data;
		dst = (unsigned long)dst_addr;
		if (copied == 0) {
			/* Explicit casting for ARM linker errors. */
			bytes = (size_t) blksize;
			bytes -= (size_t) boffset % (size_t) blksize;
			if (bytes > len)
				bytes = len;
			/* Explicit casting for ARM linker errors. */
			src += (unsigned long)boffset % (unsigned long)blksize;
		} else {
			dst += copied;
			if ((len - copied) < blksize) {
				bytes = len - copied;
			} else {
				bytes = blksize;
			}
		}
		memcpy((void *)dst, (void *)src, bytes);
		copied += bytes;
		brelse(bh);
	}
	return 0;
}

int axfs_is_dev_bdev(char *path)
{
	struct nameidata nd;
	int ret = FALSE;

	if (!path)
		return FALSE;

	if (path_lookup(path, LOOKUP_FOLLOW, &nd))
		return FALSE;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,24)
	if (S_ISBLK(nd.path.dentry->d_inode->i_mode))
#else
	if (S_ISBLK(nd.dentry->d_inode->i_mode))
#endif
		ret = TRUE;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,24)
	path_put(&nd.path);
#else
	path_release(&nd);
#endif
	return ret;
}

#else

int axfs_get_sb_bdev(struct file_system_type *fs_type, int flags,
		     const char *dev_name, struct axfs_super *sbi,
		     struct vfsmount *mnt, int *err)
{
	return FALSE;
}

void axfs_kill_block_super(struct super_block *sb)
{
}

int axfs_copy_block(struct super_block *sb, void *dst_addr, u64 fsoffset,
		    u64 len)
{
	return -EINVAL;
}

int axfs_is_dev_bdev(char *path)
{
	return FALSE;
}

#endif /* CONFIG_BLOCK */
