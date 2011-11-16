/*
 * BRIEF DESCRIPTION
 *
 * Inode methods (allocate/free/read/write).
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
#include <linux/smp_lock.h>
#include <linux/sched.h>
#include <linux/highuid.h>
#include <linux/quotaops.h>
#include <linux/module.h>
#include <linux/mpage.h>
#include <linux/backing-dev.h>
#include <linux/falloc.h>
#include "pram.h"
#include "xattr.h"
#include "xip.h"
#include "acl.h"

struct backing_dev_info pram_backing_dev_info __read_mostly = {
	.ra_pages       = 0,    /* No readahead */
	.capabilities	= BDI_CAP_NO_ACCT_AND_WRITEBACK,
};

/*
 * allocate a data block for inode and return it's absolute blocknr.
 * Zeroes out the block if zero set. Increments inode->i_blocks.
 */
static int pram_new_data_block(struct inode *inode, unsigned long *blocknr,
			       int zero)
{
	int errval = pram_new_block(inode->i_sb, blocknr, zero);

	if (!errval) {
		struct pram_inode *pi = pram_get_inode(inode->i_sb,
							inode->i_ino);
		inode->i_blocks++;
		pram_memunlock_inode(inode->i_sb, pi);
		pi->i_blocks = cpu_to_be32(inode->i_blocks);
		pram_memlock_inode(inode->i_sb, pi);
	}

	return errval;
}

/*
 * find the offset to the block represented by the given inode's file
 * relative block number.
 */
u64 pram_find_data_block(struct inode *inode, unsigned long file_blocknr)
{
	struct super_block *sb = inode->i_sb;
	struct pram_inode *pi;
	u64 *row; /* ptr to row block */
	u64 *col; /* ptr to column blocks */
	u64 bp = 0;
	unsigned int i_row, i_col;
	unsigned int N = sb->s_blocksize >> 3; /* num block ptrs per block */
	unsigned int Nbits = sb->s_blocksize_bits - 3;

	pi = pram_get_inode(sb, inode->i_ino);

	i_row = file_blocknr >> Nbits;
	i_col  = file_blocknr & (N-1);

	row = pram_get_block(sb, be64_to_cpu(pi->i_type.reg.row_block));
	if (row) {
		col = pram_get_block(sb, be64_to_cpu(row[i_row]));
		if (col)
			bp = be64_to_cpu(col[i_col]);
	}

	return bp;
}

/*
 * Free data blocks from inode in the range start <=> end
 */
static void __pram_truncate_blocks(struct inode *inode, loff_t start,
				   loff_t end)
{
	struct super_block *sb = inode->i_sb;
	struct pram_inode *pi = pram_get_inode(sb, inode->i_ino);
	int N = sb->s_blocksize >> 3; /* num block ptrs per block */
	int Nbits = sb->s_blocksize_bits - 3;
	int first_row_index, last_row_index, i, j;
	unsigned long blocknr, first_blocknr, last_blocknr;
	unsigned int freed = 0;
	u64 *row; /* ptr to row block */
	u64 *col; /* ptr to column blocks */

	if (start > end || !inode->i_blocks || !pi->i_type.reg.row_block)
		return;

	mutex_lock(&PRAM_I(inode)->truncate_mutex);

	first_blocknr = (start + sb->s_blocksize - 1) >> sb->s_blocksize_bits;

	if ((be32_to_cpu(pi->i_flags) & PRAM_EOFBLOCKS_FL) && start == 0)
		last_blocknr = (1UL << (2*sb->s_blocksize_bits - 6)) - 1;
	else
		last_blocknr = (end + sb->s_blocksize - 1) >>
							   sb->s_blocksize_bits;
	first_row_index = first_blocknr >> Nbits;
	last_row_index  = last_blocknr >> Nbits;

	row = pram_get_block(sb, be64_to_cpu(pi->i_type.reg.row_block));

	for (i = first_row_index; i <= last_row_index; i++) {
		int first_col_index = (i == first_row_index) ?
			first_blocknr & (N-1) : 0;
		int last_col_index = (i == last_row_index) ?
			last_blocknr & (N-1) : N-1;

		if (unlikely(!row[i]))
			continue;

		col = pram_get_block(sb, be64_to_cpu(row[i]));

		for (j = first_col_index; j <= last_col_index; j++) {

			if (unlikely(!col[j]))
				continue;

			blocknr = pram_get_blocknr(sb, be64_to_cpu(col[j]));
			pram_free_block(sb, blocknr);
			freed++;
			pram_memunlock_block(sb, col);
			col[j] = 0;
			pram_memlock_block(sb, col);
		}

		if (first_col_index == 0) {
			blocknr = pram_get_blocknr(sb, be64_to_cpu(row[i]));
			pram_free_block(sb, blocknr);
			pram_memunlock_block(sb, row);
			row[i] = 0;
			pram_memlock_block(sb, row);
		}
	}

	inode->i_blocks -= freed;

	if (start == 0) {
		unsigned long flags;
		blocknr = pram_get_blocknr(sb, be64_to_cpu(pi->i_type.reg.row_block));
		pram_free_block(sb, blocknr);
		pram_memunlock_inode(sb, pi);
		pi->i_type.reg.row_block = 0;
		flags = be32_to_cpu(pi->i_flags);
		flags &= ~PRAM_EOFBLOCKS_FL;
		pi->i_flags = cpu_to_be32(flags);
		pram_memlock_inode(sb, pi);
	}
	pram_memunlock_inode(sb, pi);
	pi->i_blocks = cpu_to_be32(inode->i_blocks);
	pram_memlock_inode(sb, pi);

	mutex_unlock(&PRAM_I(inode)->truncate_mutex);
}

static void pram_truncate_blocks(struct inode *inode, loff_t start, loff_t end)
{
	if (!(S_ISREG(inode->i_mode) || S_ISDIR(inode->i_mode) ||
	      S_ISLNK(inode->i_mode)))
		return;
	if (IS_APPEND(inode) || IS_IMMUTABLE(inode))
		return;

	__pram_truncate_blocks(inode, start, end);
	inode->i_mtime = inode->i_ctime = CURRENT_TIME_SEC;
	pram_update_inode(inode);
}

/*
 * Allocate num data blocks for inode, starting at given file-relative
 * block number. All blocks except the last are zeroed out.
 */
int pram_alloc_blocks(struct inode *inode, int file_blocknr, int num)
{
	struct super_block *sb = inode->i_sb;
	struct pram_inode *pi = pram_get_inode(sb, inode->i_ino);
	int N = sb->s_blocksize >> 3; /* num block ptrs per block */
	int Nbits = sb->s_blocksize_bits - 3;
	int first_file_blocknr;
	int last_file_blocknr;
	int first_row_index, last_row_index;
	int i, j, errval;
	unsigned long blocknr;
	u64 *row;
	u64 *col;

	if (!pi->i_type.reg.row_block) {
		/* alloc the 2nd order array block */
		errval = pram_new_block(sb, &blocknr, 1);
		if (errval) {
			pram_dbg("failed to alloc 2nd order array block\n");
			goto fail;
		}
		pram_memunlock_inode(sb, pi);
		pi->i_type.reg.row_block = cpu_to_be64(pram_get_block_off(sb,
								      blocknr));
		pram_memlock_inode(sb, pi);
	}

	row = pram_get_block(sb, be64_to_cpu(pi->i_type.reg.row_block));

	first_file_blocknr = file_blocknr;
	last_file_blocknr = file_blocknr + num - 1;

	first_row_index = first_file_blocknr >> Nbits;
	last_row_index  = last_file_blocknr >> Nbits;

	for (i = first_row_index; i <= last_row_index; i++) {
		int first_col_index, last_col_index;

		/*
		 * we are starting a new row, so make sure
		 * there is a block allocated for the row.
		 */
		if (!row[i]) {
			/* allocate the row block */
			errval = pram_new_block(sb, &blocknr, 1);
			if (errval) {
				pram_dbg("failed to alloc row block\n");
				goto fail;
			}
			pram_memunlock_block(sb, row);
			row[i] = cpu_to_be64(pram_get_block_off(sb, blocknr));
			pram_memlock_block(sb, row);
		}
		col = pram_get_block(sb, be64_to_cpu(row[i]));

		first_col_index = (i == first_row_index) ?
			first_file_blocknr & (N-1) : 0;

		last_col_index = (i == last_row_index) ?
			last_file_blocknr & (N-1) : N-1;

		for (j = first_col_index; j <= last_col_index; j++) {
			if (!col[j]) {
				errval = pram_new_data_block(inode, &blocknr, 1);
				if (errval) {
					pram_dbg("failed to alloc data block\n");
					goto fail;
				}
				pram_memunlock_block(sb, col);
				col[j] = cpu_to_be64(pram_get_block_off(sb,
								      blocknr));
				pram_memlock_block(sb, col);
			}
		}
	}

	errval = 0;
 fail:
	return errval;
}

static int pram_read_inode(struct inode *inode, struct pram_inode *pi)
{
	int ret = -EIO;

	mutex_lock(&PRAM_I(inode)->i_meta_mutex);

	if (pram_calc_checksum((u8 *)pi, PRAM_INODE_SIZE)) {
		pram_err(inode->i_sb, "checksum error in inode %08x\n",
			  (u32)inode->i_ino);
		goto bad_inode;
	}

	inode->i_mode = be16_to_cpu(pi->i_mode);
	inode->i_uid = be32_to_cpu(pi->i_uid);
	inode->i_gid = be32_to_cpu(pi->i_gid);
	inode->i_nlink = be16_to_cpu(pi->i_links_count);
	inode->i_size = be32_to_cpu(pi->i_size);
	inode->i_atime.tv_sec = be32_to_cpu(pi->i_atime);
	inode->i_ctime.tv_sec = be32_to_cpu(pi->i_ctime);
	inode->i_mtime.tv_sec = be32_to_cpu(pi->i_mtime);
	inode->i_atime.tv_nsec = inode->i_mtime.tv_nsec =
		inode->i_ctime.tv_nsec = 0;
	inode->i_generation = be32_to_cpu(pi->i_generation);
	pram_set_inode_flags(inode, pi);

	/* check if the inode is active. */
	if (inode->i_nlink == 0 && (inode->i_mode == 0 || be32_to_cpu(pi->i_dtime))) {
		/* this inode is deleted */
		pram_dbg("read inode: inode %lu not active", inode->i_ino);
		ret = -EINVAL;
		goto bad_inode;
	}

	inode->i_blocks = be32_to_cpu(pi->i_blocks);
	inode->i_ino = pram_get_inodenr(inode->i_sb, pi);
	inode->i_mapping->a_ops = &pram_aops;
	inode->i_mapping->backing_dev_info = &pram_backing_dev_info;

	insert_inode_hash(inode);
	switch (inode->i_mode & S_IFMT) {
	case S_IFREG:
		if (pram_use_xip(inode->i_sb)) {
			inode->i_mapping->a_ops = &pram_aops_xip;
			inode->i_fop = &pram_xip_file_operations;
		} else {
			inode->i_op = &pram_file_inode_operations;
			inode->i_fop = &pram_file_operations;
		}
		break;
	case S_IFDIR:
		inode->i_op = &pram_dir_inode_operations;
		inode->i_fop = &pram_dir_operations;
		break;
	case S_IFLNK:
		inode->i_op = &pram_symlink_inode_operations;
		break;
	default:
		inode->i_size = 0;
		init_special_inode(inode, inode->i_mode,
				   be32_to_cpu(pi->i_type.dev.rdev));
		break;
	}

	mutex_unlock(&PRAM_I(inode)->i_meta_mutex);
	return 0;

 bad_inode:
	make_bad_inode(inode);
	mutex_unlock(&PRAM_I(inode)->i_meta_mutex);
	return ret;
}

int pram_update_inode(struct inode *inode)
{
	struct pram_inode *pi;
	int retval = 0;

	pi = pram_get_inode(inode->i_sb, inode->i_ino);
	if (!pi)
		return -EACCES;

	mutex_lock(&PRAM_I(inode)->i_meta_mutex);

	pram_memunlock_inode(inode->i_sb, pi);
	pi->i_mode = cpu_to_be16(inode->i_mode);
	pi->i_uid = cpu_to_be32(inode->i_uid);
	pi->i_gid = cpu_to_be32(inode->i_gid);
	pi->i_links_count = cpu_to_be16(inode->i_nlink);
	pi->i_size = cpu_to_be32(inode->i_size);
	pi->i_blocks = cpu_to_be32(inode->i_blocks);
	pi->i_atime = cpu_to_be32(inode->i_atime.tv_sec);
	pi->i_ctime = cpu_to_be32(inode->i_ctime.tv_sec);
	pi->i_mtime = cpu_to_be32(inode->i_mtime.tv_sec);
	pi->i_generation = cpu_to_be32(inode->i_generation);
	pram_get_inode_flags(inode, pi);

	if (S_ISCHR(inode->i_mode) || S_ISBLK(inode->i_mode))
		pi->i_type.dev.rdev = cpu_to_be32(inode->i_rdev);
	
	pram_memlock_inode(inode->i_sb, pi);

	mutex_unlock(&PRAM_I(inode)->i_meta_mutex);
	return retval;
}

/*
 * NOTE! When we get the inode, we're the only people
 * that have access to it, and as such there are no
 * race conditions we have to worry about. The inode
 * is not on the hash-lists, and it cannot be reached
 * through the filesystem because the directory entry
 * has been deleted earlier.
 */
static void pram_free_inode(struct inode *inode)
{
	struct super_block *sb = inode->i_sb;
	struct pram_super_block *ps;
	struct pram_inode *pi;
	unsigned long inode_nr;

	pram_xattr_delete_inode(inode);

	lock_super(sb);

	inode_nr = (inode->i_ino - PRAM_ROOT_INO) >> PRAM_INODE_BITS;

	pi = pram_get_inode(sb, inode->i_ino);
	pram_memunlock_inode(sb, pi);
	pi->i_dtime = cpu_to_be32(get_seconds());
	pi->i_type.reg.row_block = 0;
	pi->i_xattr = 0;
	pram_memlock_inode(sb, pi);

	/* increment s_free_inodes_count */
	ps = pram_get_super(sb);
	pram_memunlock_super(sb, ps);
	if (inode_nr < be32_to_cpu(ps->s_free_inode_hint))
		ps->s_free_inode_hint = cpu_to_be32(inode_nr);
	be32_add_cpu(&ps->s_free_inodes_count, 1);
	if (be32_to_cpu(ps->s_free_inodes_count) == be32_to_cpu(ps->s_inodes_count) - 1) {
		/* filesystem is empty */
		pram_dbg("fs is empty!\n");
		ps->s_free_inode_hint = cpu_to_be32(1);
	}
	pram_memlock_super(sb, ps);

	unlock_super(sb);
}

struct inode *pram_iget(struct super_block *sb, unsigned long ino)
{
	struct inode *inode;
	struct pram_inode *pi;
	int err;

	inode = iget_locked(sb, ino);
	if (unlikely(!inode))
		return ERR_PTR(-ENOMEM);
	if (!(inode->i_state & I_NEW))
		return inode;

	pi = pram_get_inode(sb, ino);
	if (!pi) {
		err = -EACCES;
		goto fail;
	}
	err = pram_read_inode(inode, pi);
	if (unlikely(err))
		goto fail;

	unlock_new_inode(inode);
	return inode;
fail:
	iget_failed(inode);
	return ERR_PTR(err);
}

void pram_clear_inode(struct inode *inode)
{
	int want_delete = 0;

	if (!inode->i_nlink && !is_bad_inode(inode))
		want_delete = 1;

	truncate_inode_pages(&inode->i_data, 0);

	if (want_delete) {
		/* unlink from chain in the inode's directory */
		pram_remove_link(inode);
		if (inode->i_blocks)
			pram_truncate_blocks(inode, 0, inode->i_size);
		inode->i_size = 0;
	}

	invalidate_inode_buffers(inode);
	end_writeback(inode);

	if (want_delete)
		pram_free_inode(inode);
}


struct inode *pram_new_inode(struct inode *dir, int mode)
{
	struct super_block *sb;
	struct pram_sb_info *sbi;
	struct pram_super_block *ps;
	struct inode *inode;
	struct pram_inode *pi = NULL;
	struct pram_inode *diri = NULL;
	int i, errval;
	ino_t ino = 0;

	sb = dir->i_sb;
	sbi = (struct pram_sb_info *)sb->s_fs_info;
	inode = new_inode(sb);
	if (!inode)
		return ERR_PTR(-ENOMEM);

	lock_super(sb);
	ps = pram_get_super(sb);

	if (ps->s_free_inodes_count) {
		/* find the oldest unused pram inode */
		for (i = be32_to_cpu(ps->s_free_inode_hint); i < be32_to_cpu(ps->s_inodes_count); i++) {
			ino = PRAM_ROOT_INO + (i << PRAM_INODE_BITS);
			pi = pram_get_inode(sb, ino);
			/* check if the inode is active. */
			if (be16_to_cpu(pi->i_links_count) == 0 &&
			   (be16_to_cpu(pi->i_mode) == 0 ||
			   be32_to_cpu(pi->i_dtime))) {
				/* this inode is deleted */
				break;
			}
		}

		if (unlikely(i >= be32_to_cpu(ps->s_inodes_count))) {
			pram_err(sb, "s_free_inodes_count!=0 but none free!?\n");
			errval = -ENOSPC;
			goto fail1;
		}

		pram_dbg("allocating inode %lu\n", ino);
	} else {
		pram_dbg("no space left to create new inode!\n");
		errval = -ENOSPC;
		goto fail1;
	}

	diri = pram_get_inode(sb, dir->i_ino);
	if (!diri) {
		errval = -EACCES;
		goto fail1;
	}

	/* chosen inode is in ino */
	inode->i_ino = ino;
	inode_init_owner(inode, dir, mode);
	inode->i_blocks = inode->i_size = 0;
	inode->i_mtime = inode->i_atime = inode->i_ctime = CURRENT_TIME;

	inode->i_generation = atomic_add_return(1, &sbi->next_generation);

	pram_memunlock_inode(sb, pi);
	pi->i_d.d_next = 0;
	pi->i_d.d_prev = 0;
	pi->i_flags = diri->i_flags;
	pram_memlock_inode(sb, pi);

	if (insert_inode_locked(inode) < 0) {
		errval = -EINVAL;
		goto fail2;
	}
	pram_write_inode(inode, 0);

	errval = pram_init_acl(inode, dir);
	if (errval)
		goto fail2;

	errval = pram_init_security(inode, dir);
	if (errval)
		goto fail2;

	pram_memunlock_super(sb, ps);
	be32_add_cpu(&ps->s_free_inodes_count, -1);
	if (i < be32_to_cpu(ps->s_inodes_count)-1)
		ps->s_free_inode_hint = cpu_to_be32(i+1);
	else
		ps->s_free_inode_hint = 0;
	pram_memlock_super(sb, ps);

	unlock_super(sb);

	return inode;
fail2:
	unlock_super(sb);
	unlock_new_inode(inode);
	iput(inode);
	return ERR_PTR(errval);
fail1:
	unlock_super(sb);
	make_bad_inode(inode);
	iput(inode);
	return ERR_PTR(errval);
}

int pram_write_inode(struct inode *inode, struct writeback_control *wbc)
{
	return pram_update_inode(inode);
}

/*
 * dirty_inode() is called from __mark_inode_dirty()
 */
void pram_dirty_inode(struct inode *inode)
{
	pram_update_inode(inode);
}

/* pram_get_and_update_block()
 *
 * Look for a block. If not found it can create a new one if create is
 * different from zero.
 *
 * It returns zero if plain lookup failed or blocks mapped or allocated
 * (plain lookup failed is not an error, e.g. for holes). Minor than zero
 * otherwise.
 */
int pram_get_and_update_block(struct inode *inode, sector_t iblock,
				     struct buffer_head *bh, int create)
{
	struct super_block *sb = inode->i_sb;
	unsigned int blocksize = 1 << inode->i_blkbits;
	int err = 0;
	u64 block;
	void *bp;

	mutex_lock(&PRAM_I(inode)->truncate_mutex);

	block = pram_find_data_block(inode, iblock);

	if (!block) {
		if (!create)
			goto out;

		err = pram_alloc_blocks(inode, iblock, 1);
		if (err)
			goto out;
		block = pram_find_data_block(inode, iblock);
		if (!block) {
			err = -EIO;
			goto out;
		}
		set_buffer_new(bh);
	}

	bh->b_blocknr = block;
	set_buffer_mapped(bh);

	/* now update the buffer synchronously */
	bp = pram_get_block(sb, block);
	if (buffer_new(bh)) {
		pram_memunlock_block(sb, bp);
		memset(bp, 0, blocksize);
		pram_memlock_block(sb, bp);
		memset(bh->b_data, 0, blocksize);
	} else {
		memcpy(bh->b_data, bp, blocksize);
	}

	set_buffer_uptodate(bh);

 out:
	mutex_unlock(&PRAM_I(inode)->truncate_mutex);
	return err;
}

/*
 * Called to zeros out a single block. It's used in the "resize"
 * to avoid to keep data in case the file grow up again.
 */
static int pram_clear_block(struct inode *inode, loff_t newsize)
{
	pgoff_t index = newsize >> PAGE_CACHE_SHIFT;
	unsigned long offset = newsize & (PAGE_CACHE_SIZE - 1);
	unsigned long blocksize, length;
	sector_t iblock;
	u64 blockoff;
	char *bp;
	int ret = 0;

	blocksize = 1 << inode->i_blkbits;
	length = offset & (blocksize - 1);

	/* Block boundary ? */
	if (!length)
		goto out;

	length = blocksize - length;
	iblock = (sector_t)index << (PAGE_CACHE_SHIFT - inode->i_blkbits);

	mutex_lock(&PRAM_I(inode)->truncate_mutex);
	blockoff = pram_find_data_block(inode, iblock);

	/* Hole ? */
	if (!blockoff)
		goto out_unlock;

	bp = pram_get_block(inode->i_sb, blockoff);
	if (!bp) {
		ret = -EACCES;
		goto out_unlock;
	}
	pram_memunlock_block(inode->i_sb, bp);
	memset(bp + offset, 0, length);
	pram_memlock_block(inode->i_sb, bp);

out_unlock:
	mutex_unlock(&PRAM_I(inode)->truncate_mutex);
out:
	return ret;
}

static int pram_setsize(struct inode *inode, loff_t newsize)
{
	int ret = 0;
	loff_t oldsize;

	if (!(S_ISREG(inode->i_mode) || S_ISDIR(inode->i_mode) ||
	    S_ISLNK(inode->i_mode)))
		return -EINVAL;
	if (IS_APPEND(inode) || IS_IMMUTABLE(inode))
		return -EPERM;

	if (mapping_is_xip(inode->i_mapping))
		ret = xip_truncate_page(inode->i_mapping, newsize);
	else
		ret = pram_clear_block(inode, newsize);
	if (ret)
		return ret;

	oldsize = inode->i_size;
	i_size_write(inode, newsize);
	__pram_truncate_blocks(inode, newsize, oldsize);
	inode->i_mtime = inode->i_ctime = CURRENT_TIME_SEC;
	pram_update_inode(inode);

	return ret;
}

int pram_notify_change(struct dentry *dentry, struct iattr *attr)
{
	struct inode *inode = dentry->d_inode;
	int error;

	error = inode_change_ok(inode, attr);
	if (error)
		return error;

	if (attr->ia_valid & ATTR_SIZE && attr->ia_size != inode->i_size) {
		error = pram_setsize(inode, attr->ia_size);
		if (error)
			return error;
	}
	inode_setattr(inode, attr);
	if (attr->ia_valid & ATTR_MODE)
		error = pram_acl_chmod(inode);
	error = pram_update_inode(inode);

	return error;
}

long pram_fallocate(struct inode *inode, int mode, loff_t offset, loff_t len)
{
	long ret = 0;
	unsigned long blocknr, blockoff, flags_old;
	int num_blocks, blocksize_mask;
	struct pram_inode *pi;
	loff_t new_size;

	/* preallocation to directories is currently not supported */
	if (S_ISDIR(inode->i_mode))
		return -ENODEV;

	mutex_lock(&inode->i_mutex);
	mutex_lock(&PRAM_I(inode)->truncate_mutex);

	if (IS_IMMUTABLE(inode) || IS_APPEND(inode)) {
		ret = -EPERM;
		goto out;
	}

	new_size = len + offset;
	if (!(mode & FALLOC_FL_KEEP_SIZE) && new_size > inode->i_size) {
		ret = inode_newsize_ok(inode, new_size);
		if (ret)
			goto out;
	}

	blocksize_mask = (1 << inode->i_sb->s_blocksize_bits) - 1;
	offset += inode->i_size;
	blocknr = offset >> inode->i_sb->s_blocksize_bits;
	blockoff = offset & blocksize_mask;
	num_blocks = (blockoff + len + blocksize_mask) >>
						inode->i_sb->s_blocksize_bits;
	ret = pram_alloc_blocks(inode, blocknr, num_blocks);
	if (ret)
		goto out;

	if (mode & FALLOC_FL_KEEP_SIZE) {
		pi = pram_get_inode(inode->i_sb, inode->i_ino);
		if (!pi) {
			ret = -EACCES;
			goto out;
		}
		pram_memunlock_inode(inode->i_sb, pi);
		flags_old = be32_to_cpu(pi->i_flags);
		flags_old |= PRAM_EOFBLOCKS_FL;
		pi->i_flags = cpu_to_be32(flags_old);
		pram_memlock_inode(inode->i_sb, pi);

	}

	inode->i_mtime = inode->i_ctime = CURRENT_TIME_SEC;
	if (!(mode & FALLOC_FL_KEEP_SIZE) && new_size > inode->i_size)
		inode->i_size = new_size;
	ret = pram_update_inode(inode);
 out:
	mutex_unlock(&PRAM_I(inode)->truncate_mutex);
	mutex_unlock(&inode->i_mutex);
	return ret;
}

void pram_set_inode_flags(struct inode *inode, struct pram_inode *pi)
{
	unsigned int flags = be32_to_cpu(pi->i_flags);

	inode->i_flags &= ~(S_SYNC|S_APPEND|S_IMMUTABLE|S_NOATIME|S_DIRSYNC);
	if (flags & FS_SYNC_FL)
		inode->i_flags |= S_SYNC;
	if (flags & FS_APPEND_FL)
		inode->i_flags |= S_APPEND;
	if (flags & FS_IMMUTABLE_FL)
		inode->i_flags |= S_IMMUTABLE;
	if (flags & FS_NOATIME_FL)
		inode->i_flags |= S_NOATIME;
	if (flags & FS_DIRSYNC_FL)
		inode->i_flags |= S_DIRSYNC;
}

void pram_get_inode_flags(struct inode *inode, struct pram_inode *pi)
{
	unsigned int flags = inode->i_flags;
	unsigned int pram_flags = be32_to_cpu(pi->i_flags);

	pram_flags &= ~(FS_SYNC_FL|FS_APPEND_FL|FS_IMMUTABLE_FL|
			FS_NOATIME_FL|FS_DIRSYNC_FL);
	if (flags & S_SYNC)
		pram_flags |= FS_SYNC_FL;
	if (flags & S_APPEND)
		pram_flags |= FS_APPEND_FL;
	if (flags & S_IMMUTABLE)
		pram_flags |= FS_IMMUTABLE_FL;
	if (flags & S_NOATIME)
		pram_flags |= FS_NOATIME_FL;
	if (flags & S_DIRSYNC)
		pram_flags |= FS_DIRSYNC_FL;

	pi->i_flags = cpu_to_be32(pram_flags);
}

struct address_space_operations pram_aops = {
	.readpage	= pram_readpage,
	.direct_IO	= pram_direct_IO,
};

struct address_space_operations pram_aops_xip = {
	.get_xip_mem	= pram_get_xip_mem,
};
