/*
 * BRIEF DESCRIPTION
 *
 * File operations for files.
 *
 * Copyright 2009-2010 Marco Stornelli <marco.stornelli@gmail.com>
 * Copyright 2003 Sony Corporation
 * Copyright 2003 Matsushita Electric Industrial Co., Ltd.
 * 2003-2004 (c) MontaVista Software, Inc. , Steve Longerbeam
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uio.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include "pram.h"
#include "acl.h"
#include "xip.h"
#include "xattr.h"

static int pram_open_file(struct inode *inode, struct file *filp)
{
	filp->f_flags |= O_DIRECT;
	return generic_file_open(inode, filp);
}

ssize_t pram_direct_IO(int rw, struct kiocb *iocb,
		   const struct iovec *iov,
		   loff_t offset, unsigned long nr_segs)
{
	struct file *file = iocb->ki_filp;
	struct inode *inode = file->f_mapping->host;
	struct super_block *sb = inode->i_sb;
	int progress = 0, hole = 0;
	ssize_t retval = 0;
	void *tmp = NULL;
	unsigned long blocknr, blockoff;
	int num_blocks, blocksize_mask, blocksize, blocksize_bits;
	char __user *buf = iov->iov_base;
	size_t length = iov_length(iov, nr_segs);

	if (length < 0)
		return -EINVAL;
	if ((rw == READ) && (offset + length > inode->i_size))
		length = inode->i_size - offset;
	if (!length)
		goto out;

	blocksize_bits = inode->i_sb->s_blocksize_bits;
	blocksize = 1 << blocksize_bits;
	blocksize_mask = blocksize - 1;

	/* find starting block number to access */
	blocknr = offset >> blocksize_bits;
	/* find starting offset within starting block */
	blockoff = offset & blocksize_mask;
	/* find number of blocks to access */
	num_blocks = (blockoff + length + blocksize_mask) >> blocksize_bits;

	if (rw == WRITE) {
		/* prepare a temporary buffer to hold a user data block
		   for writing. */
		tmp = kmalloc(blocksize, GFP_KERNEL);
		if (!tmp)
			return -ENOMEM;
		/* now allocate the data blocks we'll need */
		retval = pram_alloc_blocks(inode, blocknr, num_blocks);
		if (retval)
			goto fail1;
	}

	while (length) {
		int count;
		u8 *bp = NULL;
		u64 block = pram_find_data_block(inode, blocknr++);
		if (unlikely(!block && rw == READ)) {
			/* We are falling in a hole */
			hole = 1;
		} else {
			bp = (u8 *)pram_get_block(sb, block);
			if (!bp)
				goto fail2;
		}

		count = blockoff + length > blocksize ?
			blocksize - blockoff : length;

		if (rw == READ) {
			if (unlikely(hole)) {
				retval = clear_user(buf, count);
				if (retval) {
					retval = -EFAULT;
					goto fail1;
				}
			} else {
				retval = copy_to_user(buf, &bp[blockoff], count);
				if (retval) {
					retval = -EFAULT;
					goto fail1;
				}
			}
		} else {
			retval = copy_from_user(tmp, buf, count);
			if (retval) {
				retval = -EFAULT;
				goto fail1;
			}

			pram_memunlock_block(inode->i_sb, bp);
			memcpy(&bp[blockoff], tmp, count);
			pram_memlock_block(inode->i_sb, bp);
		}

		progress += count;
		buf += count;
		length -= count;
		blockoff = 0;
		hole = 0;
	}

fail2:
	retval = progress;
fail1:
	kfree(tmp);
out:
	return retval;
}

int pram_mmap(struct file *file, struct vm_area_struct *vma)
{
	/* Only private mappings */
	if (vma->vm_flags & VM_SHARED)
		return -EINVAL;
	return generic_file_mmap(file, vma);
}

static int pram_check_flags(int flags)
{
	if (!(flags & O_DIRECT))
		return -EINVAL;

	return 0;
}

struct file_operations pram_file_operations = {
	.llseek		= generic_file_llseek,
	.read		= do_sync_read,
	.write		= do_sync_write,
	.aio_read	= generic_file_aio_read,
	.aio_write	= generic_file_aio_write,
	.mmap		= pram_mmap,
	.open		= pram_open_file,
	.fsync		= noop_fsync,
	.check_flags	= pram_check_flags,
};

#ifdef CONFIG_PRAMFS_XIP
struct file_operations pram_xip_file_operations = {
	.llseek		= generic_file_llseek,
	.read		= xip_file_read,
	.write		= xip_file_write,
	.mmap		= xip_file_mmap,
	.open		= generic_file_open,
	.fsync		= noop_fsync,
};
#endif

struct inode_operations pram_file_inode_operations = {
#ifdef CONFIG_PRAMFS_XATTR
	.setxattr	= generic_setxattr,
	.getxattr	= generic_getxattr,
	.listxattr	= pram_listxattr,
	.removexattr	= generic_removexattr,
#endif
	.setattr	= pram_notify_change,
	.check_acl	= pram_check_acl,
};
