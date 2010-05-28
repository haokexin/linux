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
 *  Eric Anderson
 *  Jared Hulbert <jaredeh@gmail.com>
 *  Sujaya Srinivasan
 *  Justin Treon
 *
 * More info and current contacts at http://axfs.sourceforge.net
 *
 * axfs_super.c -
 *   Contains the core code used to mount the fs.
 *
 */

#include <linux/axfs.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>
#include <linux/parser.h>
#include <linux/statfs.h>
#include <linux/module.h>
#include <linux/mount.h>
#include <linux/mtd/mtd.h>

/******************** Function Declarations ****************************/
static struct super_operations axfs_sops;
static struct axfs_super *axfs_get_sbi(void);
static void axfs_put_sbi(struct axfs_super *);
/***************** functions in other axfs files ***************************/
int axfs_get_sb_bdev(struct file_system_type *, int, const char *,
		     struct axfs_super *, struct vfsmount *, int *);
void axfs_kill_block_super(struct super_block *);
int axfs_copy_block(struct super_block *, void *, u64, u64);
int axfs_is_dev_bdev(char *);
#ifndef NO_PHYSMEM
void axfs_map_physmem(struct axfs_super *, unsigned long);
void axfs_unmap_physmem(struct super_block *);
#endif
int axfs_map_mtd(struct super_block *);
void axfs_unmap_mtd(struct super_block *);
int axfs_copy_mtd(struct super_block *, void *, u64, u64);
int axfs_get_sb_mtd(struct file_system_type *, int, const char *,
		    struct axfs_super *, struct vfsmount *, int *);
int axfs_is_dev_mtd(char *, int *);
void axfs_kill_mtd_super(struct super_block *);
struct inode *axfs_create_vfs_inode(struct super_block *, int);
int axfs_get_uml_address(char *, unsigned long *, unsigned long *);
int axfs_init_profiling(struct axfs_super *);
int axfs_shutdown_profiling(struct axfs_super *);
void axfs_profiling_add(struct axfs_super *, unsigned long, unsigned int);
struct inode *axfs_create_vfs_inode(struct super_block *, int);
/******************************************************************************/

static void axfs_free_region(struct axfs_super *sbi,
			     struct axfs_region_desc *region)
{
	if (!region)
		return;

	if (AXFS_IS_REGION_XIP(sbi, region))
		return;

	if (region->virt_addr)
		vfree(region->virt_addr);
}

static void axfs_put_sbi(struct axfs_super *sbi)
{
	if (!sbi)
		return;

	axfs_shutdown_profiling(sbi);

	axfs_free_region(sbi, &sbi->strings);
	axfs_free_region(sbi, &sbi->xip);
	axfs_free_region(sbi, &sbi->compressed);
	axfs_free_region(sbi, &sbi->byte_aligned);
	axfs_free_region(sbi, &sbi->node_type);
	axfs_free_region(sbi, &sbi->node_index);
	axfs_free_region(sbi, &sbi->cnode_offset);
	axfs_free_region(sbi, &sbi->cnode_index);
	axfs_free_region(sbi, &sbi->banode_offset);
	axfs_free_region(sbi, &sbi->cblock_offset);
	axfs_free_region(sbi, &sbi->inode_file_size);
	axfs_free_region(sbi, &sbi->inode_name_offset);
	axfs_free_region(sbi, &sbi->inode_num_entries);
	axfs_free_region(sbi, &sbi->inode_mode_index);
	axfs_free_region(sbi, &sbi->inode_array_index);
	axfs_free_region(sbi, &sbi->modes);
	axfs_free_region(sbi, &sbi->uids);
	axfs_free_region(sbi, &sbi->gids);

	if (sbi->second_dev)
		kfree(sbi->second_dev);

	if (sbi->cblock_buffer[0])
		vfree(sbi->cblock_buffer[0]);
	if (sbi->cblock_buffer[1])
		vfree(sbi->cblock_buffer[1]);

	kfree(sbi);
}

static void axfs_put_super(struct super_block *sb)
{
	axfs_unmap_mtd(sb);
#ifndef NO_PHYSMEM
	axfs_unmap_physmem(sb);
#endif
	axfs_put_sbi(AXFS_SB(sb));
	sb->s_fs_info = NULL;
}

static int axfs_copy_mem(struct super_block *sb, void *buf, u64 fsoffset,
			 u64 len)
{
	struct axfs_super *sbi = AXFS_SB(sb);
	unsigned long addr;

	addr = sbi->virt_start_addr + (unsigned long)fsoffset;
	memcpy(buf, (void *)addr, (size_t) len);
	return 0;
}

static int axfs_copy_metadata(struct super_block *sb, void *buf, u64 fsoffset,
			      u64 len)
{
	struct axfs_super *sbi = AXFS_SB(sb);
	u64 end = fsoffset + len;
	u64 a = sbi->mmap_size - fsoffset;
	u64 b = end - sbi->mmap_size;
	void *bb = (void *)((unsigned long)buf + (unsigned long)a);
	int err;

	/* Catches case where sbi is not yet fully initialized. */
	if ((sbi->magic == 0) && (sbi->virt_start_addr != 0))
		return axfs_copy_mem(sb, buf, fsoffset, len);

	if (fsoffset < sbi->mmap_size) {
		if (end > sbi->mmap_size) {
			err = axfs_copy_metadata(sb, buf, fsoffset, a);
			if (err)
				return err;
			err = axfs_copy_metadata(sb, bb, sbi->mmap_size, b);
		} else {
			if (AXFS_IS_OFFSET_MMAPABLE(sbi, fsoffset)) {
				err = axfs_copy_mem(sb, buf, fsoffset, len);
			} else if (AXFS_HAS_MTD(sb)) {
				err = axfs_copy_mtd(sb, buf, fsoffset, len);
			} else if (AXFS_HAS_BDEV(sb)) {
				err = axfs_copy_block(sb, buf, fsoffset, len);
			} else {
				err = -EINVAL;
			}
		}
	} else {
		if (AXFS_NODEV(sb)) {
			err = axfs_copy_mem(sb, buf, fsoffset, len);
		} else if (AXFS_HAS_BDEV(sb)) {
			err = axfs_copy_block(sb, buf, fsoffset, len);
		} else if (AXFS_HAS_MTD(sb)) {
			err = axfs_copy_mtd(sb, buf, fsoffset, len);
		} else {
			err = -EINVAL;
		}
	}
	return err;
}

static int axfs_fill_region_data(struct super_block *sb,
				 struct axfs_region_desc *region, int force)
{
	struct axfs_super *sbi = AXFS_SB(sb);
	unsigned long addr;
	void *buff = NULL;
	void *vaddr;
	int err = -ENOMEM;
	u64 size = region->size;
	u64 fsoffset = region->fsoffset;
	u64 end = fsoffset + size;
	u64 c_size = region->compressed_size;

	if (size == 0)
		return 0;

	if (AXFS_IS_REGION_INCORE(region))
		goto incore;

	if (AXFS_IS_REGION_COMPRESSED(region))
		goto incore;

	if (AXFS_IS_REGION_XIP(sbi, region)) {
		if ((end > sbi->mmap_size) && (force))
			goto incore;
		addr = sbi->virt_start_addr;
		addr += (unsigned long)fsoffset;
		region->virt_addr = (void *)addr;
		return 0;
	}

	if (force)
		goto incore;

	region->virt_addr = NULL;
	return 0;

incore:
	region->virt_addr = vmalloc(size);
	if (!region->virt_addr)
		goto out;
	vaddr = region->virt_addr;

	if (AXFS_IS_REGION_COMPRESSED(region)) {
		buff = vmalloc(c_size);
		if (!buff)
			goto out;
		axfs_copy_metadata(sb, buff, fsoffset, c_size);
		err = axfs_uncompress_block(vaddr, size, buff, c_size);
		if (!err)
			goto out;
		vfree(buff);
	} else {
		axfs_copy_metadata(sb, vaddr, fsoffset, size);
	}

	return 0;

out:
	if (buff)
		vfree(buff);
	if (region->virt_addr)
		vfree(region->virt_addr);
	return err;
}

static int axfs_fill_region_data_ptrs(struct super_block *sb)
{
	int err;
	struct axfs_super *sbi = AXFS_SB(sb);

	err = axfs_fill_region_data(sb, &sbi->strings, TRUE);
	if (err)
		goto out;

	err = axfs_fill_region_data(sb, &sbi->xip, TRUE);
	if (err)
		goto out;

	err = axfs_fill_region_data(sb, &sbi->compressed, FALSE);
	if (err)
		goto out;

	err = axfs_fill_region_data(sb, &sbi->byte_aligned, FALSE);
	if (err)
		goto out;

	err = axfs_fill_region_data(sb, &sbi->node_type, TRUE);
	if (err)
		goto out;
	err = axfs_fill_region_data(sb, &sbi->node_index, TRUE);
	if (err)
		goto out;
	err = axfs_fill_region_data(sb, &sbi->cnode_offset, TRUE);
	if (err)
		goto out;
	err = axfs_fill_region_data(sb, &sbi->cnode_index, TRUE);
	if (err)
		goto out;
	err = axfs_fill_region_data(sb, &sbi->banode_offset, TRUE);
	if (err)
		goto out;
	err = axfs_fill_region_data(sb, &sbi->cblock_offset, TRUE);
	if (err)
		goto out;
	err = axfs_fill_region_data(sb, &sbi->inode_file_size, TRUE);
	if (err)
		goto out;
	err = axfs_fill_region_data(sb, &sbi->inode_name_offset, TRUE);
	if (err)
		goto out;
	err = axfs_fill_region_data(sb, &sbi->inode_num_entries, TRUE);
	if (err)
		goto out;
	err = axfs_fill_region_data(sb, &sbi->inode_mode_index, TRUE);
	if (err)
		goto out;
	err = axfs_fill_region_data(sb, &sbi->inode_array_index, TRUE);
	if (err)
		goto out;
	err = axfs_fill_region_data(sb, &sbi->modes, TRUE);
	if (err)
		goto out;
	err = axfs_fill_region_data(sb, &sbi->uids, TRUE);
	if (err)
		goto out;
	err = axfs_fill_region_data(sb, &sbi->gids, TRUE);
	if (err)
		goto out;

out:
	return err;
}

static int axfs_init_cblock_buffers(struct axfs_super *sbi)
{
	sbi->current_cnode_index = -1;
	sbi->cblock_buffer[0] = vmalloc(sbi->cblock_size);
	sbi->cblock_buffer[1] = vmalloc(sbi->cblock_size);
	if ((!sbi->cblock_buffer[0]) || (!sbi->cblock_buffer[1]))
		return -ENOMEM;

	return 0;
}

static int axfs_fixup_devices(struct super_block *sb)
{
	struct axfs_super *sbi = AXFS_SB(sb);
	int err = 0;

#ifndef NO_PHYSMEM
	if (AXFS_IS_PHYSMEM(sbi)) {
		axfs_map_physmem(sbi, sbi->mmap_size);
	} else if (AXFS_HAS_MTD(sb)) {
#else
	if (AXFS_HAS_MTD(sb)) {
#endif
		err = axfs_map_mtd(sb);
	} else if (AXFS_IS_IOMEM(sbi)) {
		sbi->phys_start_addr = 0;
	}
	return err;
}

static void axfs_fill_region_desc(struct super_block *sb,
				  struct axfs_region_desc_onmedia *out,
				  u64 offset_be, struct axfs_region_desc *in)
{
	u64 offset = be64_to_cpu(offset_be);

	axfs_copy_metadata(sb, (void *)out, offset, sizeof(*out));

	in->fsoffset = be64_to_cpu(out->fsoffset);
	in->size = be64_to_cpu(out->size);
	in->compressed_size = be64_to_cpu(out->compressed_size);
	in->max_index = be64_to_cpu(out->max_index);
	in->table_byte_depth = out->table_byte_depth;
	in->incore = out->incore;
}

static int axfs_fill_region_descriptors(struct super_block *sb,
					struct axfs_super_onmedia *sbo)
{
	struct axfs_super *sbi = AXFS_SB(sb);
	struct axfs_region_desc_onmedia *out;

	out = kmalloc(sizeof(*out), GFP_KERNEL);
	if (!out)
		return -ENOMEM;
	memset(out, 0, sizeof(*out));

	axfs_fill_region_desc(sb, out, sbo->strings, &sbi->strings);
	axfs_fill_region_desc(sb, out, sbo->xip, &sbi->xip);
	axfs_fill_region_desc(sb, out, sbo->compressed, &sbi->compressed);
	axfs_fill_region_desc(sb, out, sbo->byte_aligned, &sbi->byte_aligned);
	axfs_fill_region_desc(sb, out, sbo->node_type, &sbi->node_type);
	axfs_fill_region_desc(sb, out, sbo->node_index, &sbi->node_index);
	axfs_fill_region_desc(sb, out, sbo->cnode_offset, &sbi->cnode_offset);
	axfs_fill_region_desc(sb, out, sbo->cnode_index, &sbi->cnode_index);
	axfs_fill_region_desc(sb, out, sbo->banode_offset, &sbi->banode_offset);
	axfs_fill_region_desc(sb, out, sbo->cblock_offset, &sbi->cblock_offset);
	axfs_fill_region_desc(sb, out, sbo->inode_file_size,
			      &sbi->inode_file_size);
	axfs_fill_region_desc(sb, out, sbo->inode_name_offset,
			      &sbi->inode_name_offset);
	axfs_fill_region_desc(sb, out, sbo->inode_num_entries,
			      &sbi->inode_num_entries);
	axfs_fill_region_desc(sb, out, sbo->inode_mode_index,
			      &sbi->inode_mode_index);
	axfs_fill_region_desc(sb, out, sbo->inode_array_index,
			      &sbi->inode_array_index);
	axfs_fill_region_desc(sb, out, sbo->modes, &sbi->modes);
	axfs_fill_region_desc(sb, out, sbo->uids, &sbi->uids);
	axfs_fill_region_desc(sb, out, sbo->gids, &sbi->gids);

	kfree(out);

	return 0;
}

int axfs_set_compression_type(struct axfs_super *sbi)
{
	if (sbi->compression_type != ZLIB)
		return -EINVAL;

	return 0;
}

static int axfs_get_onmedia_super(struct super_block *sb)
{
	int err;
	struct axfs_super *sbi = AXFS_SB(sb);
	struct axfs_super_onmedia *sbo;

	sbo = kmalloc(sizeof(*sbo), GFP_KERNEL);
	if (!sbo)
		return -ENOMEM;

#ifndef NO_PHYSMEM
	axfs_map_physmem(sbi, sizeof(*sbo));
#endif
	axfs_copy_metadata(sb, (void *)sbo, 0, sizeof(*sbo));

	/* Do sanity checks on the superblock */
	if (be32_to_cpu(sbo->magic) != AXFS_MAGIC) {
		printk(KERN_ERR "axfs: wrong magic\n");
		err = -EINVAL;
		goto out;
	}

	/* verify the signiture is correct */
	if (strncmp(sbo->signature, AXFS_SIGNATURE, sizeof(AXFS_SIGNATURE))) {
		printk(KERN_ERR "axfs: wrong axfs signature,"
		       " read %s, expected %s\n",
		       sbo->signature, AXFS_SIGNATURE);
		err = -EINVAL;
		goto out;
	}

	sbi->magic = be32_to_cpu(sbo->magic);
	sbi->version_major = sbo->version_major;
	sbi->version_minor = sbo->version_minor;
	sbi->version_sub = sbo->version_sub;
	sbi->files = be64_to_cpu(sbo->files);
	sbi->size = be64_to_cpu(sbo->size);
	sbi->blocks = be64_to_cpu(sbo->blocks);
	sbi->mmap_size = be64_to_cpu(sbo->mmap_size);
	sbi->cblock_size = be32_to_cpu(sbo->cblock_size);
	sbi->timestamp.tv_sec = be64_to_cpu(sbo->timestamp);
	sbi->timestamp.tv_nsec = 0;
	sbi->compression_type = sbo->compression_type;

	err = axfs_set_compression_type(sbi);
	if (err)
		goto out;

	/* If no block or MTD device, adjust mmapable to cover all image */
	if (AXFS_NODEV(sb))
		sbi->mmap_size = sbi->size;

	err = axfs_fill_region_descriptors(sb, sbo);
	if (err)
		goto out;

	err = 0;
out:
	kfree(sbo);
#ifndef NO_PHYSMEM
	axfs_unmap_physmem(sb);
#endif
	return err;
}

/* Verify that the size of the block segment of a split filesystem
   is less than or equal to that of the device containing it.
   Validation of the size of an mmap segment vs. the device containing
   it is handled by the point() function in axfs_map_mtd.
*/
int axfs_verify_device_sizes(struct super_block *sb)
{
	struct axfs_super *sbi = AXFS_SB(sb);
	struct mtd_info *mtd0 = AXFS_MTD(sb);
	struct mtd_info *mtd1 = AXFS_MTD1(sb);
	int sndsize = sbi->size - sbi->mmap_size;

	/* Whole FS on one device */
	if (mtd0 && !mtd1 && (mtd0->size < sbi->size)) {
		printk(KERN_ERR "axfs: ERROR: Filesystem extends beyond end of"
		       "MTD! Filesystem cannot be safely mounted!\n");
		printk(KERN_ERR "mtd name: %s, mtd size: 0x%x,"
		       " fs size: 0x%llx\n", mtd0->name, mtd0->size, sbi->size);
		return -EINVAL;
	}

	/* Split filesystem using physaddr */
	if (sndsize && !mtd0 && mtd1 && (mtd1->size < sndsize)) {
		printk(KERN_ERR "axfs: ERROR: The specified second_dev device "
		       "is smaller than the store and download segment!\n");
		printk(KERN_ERR "mtd name: %s, mtd size: 0x%x, "
		       "snd size: 0x%x\n", mtd1->name, mtd1->size, sndsize);
		return -EINVAL;
	}

	/* Split filesystem using two devices */
	if (sndsize && mtd0 && mtd1) {
		if (mtd0->size < sbi->mmap_size) {
			printk(KERN_ERR "axfs: ERROR: Mmap segment extends"
			       " beyond end of MTD!");
			printk(KERN_ERR "mtd name: %s, mtd size: 0x%x, mmap "
			       "size: 0x%llx",
			       mtd0->name, mtd0->size, sbi->mmap_size);
			return -EINVAL;
		}
		if (mtd1->size < sndsize) {
			printk(KERN_ERR "axfs: ERROR: The specified second_dev "
			       "device is smaller than the store and download "
			       "segment!\n");
			printk(KERN_ERR "mtd name: %s, mtd size: 0x%x, "
			       "snd size: 0x%x\n",
			       mtd1->name, mtd1->size, sndsize);
			return -EINVAL;
		}
	}

	return 0;
}

/* Read the last four bytes of the volume and make sure the AXFS magic is
   present. */
int axfs_verify_eofs_magic(struct super_block *sb)
{
	struct axfs_super *sbi = AXFS_SB(sb);
	u32 buf = 0;
	int err;
	u32 fsoffset = sbi->size - sizeof(u32);
	int len = sizeof(u32);

	err = axfs_copy_metadata(sb, &buf, fsoffset, len);

	if (err)
		return -EINVAL;

	if (be32_to_cpu(buf) != AXFS_MAGIC) {
		printk(KERN_ERR "READ: 0x%x\n", be32_to_cpu(buf));
		printk(KERN_ERR "ERROR: Filesystem is incomplete and cannot be "
		       "mounted!\n");
		return -EINVAL;
	}

	return 0;
}

static int axfs_do_fill_super(struct super_block *sb)
{
	struct axfs_super *sbi = AXFS_SB(sb);
	int err;

	err = axfs_get_onmedia_super(sb);
	if (err)
		goto out;

	err = axfs_fixup_devices(sb);
	if (err)
		goto out;

	err = axfs_verify_device_sizes(sb);
	if (err)
		goto out;

	err = axfs_verify_eofs_magic(sb);
	if (err)
		goto out;

	err = axfs_fill_region_data_ptrs(sb);
	if (err)
		goto out;

	/* Check that the root inode is in a sane state */
	if (!S_ISDIR(AXFS_GET_MODE(sbi, 0))) {
		printk(KERN_ERR "axfs: root is not a directory\n");
		err = -EINVAL;
		goto out;
	}

	if (AXFS_GET_INODE_NUM_ENTRIES(sbi, 0) == 0) {
		printk(KERN_INFO "axfs: empty filesystem");
		err = -EINVAL;
		goto out;
	}

	err = axfs_init_cblock_buffers(sbi);
	if (err)
		goto out;

	init_rwsem(&sbi->lock);

	return 0;

out:
	axfs_put_super(sb);
	return err;
}

int axfs_fill_super(struct super_block *sb, void *data, int silent)
{
	struct axfs_super *sbi_in = (struct axfs_super *)data;
	struct axfs_super *sbi;
	struct inode *root;
	int err;

	sbi = axfs_get_sbi();
	sb->s_fs_info = (void *)sbi;
	memcpy(sbi, sbi_in, sizeof(*sbi));

	/* fully populate the incore superblock structures */
	err = axfs_do_fill_super(sb);
	if (err)
		goto out;

	sb->s_flags |= MS_RDONLY;

	/* Setup the VFS super block now */
	sb->s_op = &axfs_sops;
	root = axfs_create_vfs_inode(sb, 0);
	if (!root) {
		err = -EINVAL;
		goto out;
	}

	sb->s_root = d_alloc_root(root);
	if (!sb->s_root) {
		iput(root);
		err = -EINVAL;
		goto out;
	}

	err = axfs_init_profiling(sbi);
	if (err)
		goto out;

	return 0;

out:
	axfs_put_super(sb);
	return err;
}

static int axfs_get_sb_address(struct file_system_type *fs_type, int flags,
			       struct axfs_super *sbi, struct vfsmount *mnt,
			       int *err)
{
	int mtdnr;
	char *sd = sbi->second_dev;

	if (sbi->phys_start_addr == 0)
		return FALSE;

	if (sbi->phys_start_addr & (PAGE_SIZE - 1)) {
		printk(KERN_ERR
		       "axfs: address 0x%lx for axfs image isn't aligned"
		       " to a page boundary\n", sbi->phys_start_addr);
		*err = -EINVAL;
		return TRUE;
	}

	if (axfs_is_dev_mtd(sd, &mtdnr)) {
		return axfs_get_sb_mtd(fs_type, flags, sd, sbi, mnt, err);
	} else if (axfs_is_dev_bdev(sd)) {
		return axfs_get_sb_bdev(fs_type, flags, sd, sbi, mnt, err);
	} else {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,10)
		*err = get_sb_nodev(fs_type, flags, sbi, axfs_fill_super, mnt);
#else
		mnt->mnt_sb =
		    get_sb_nodev(fs_type, flags, (void *)sbi, axfs_fill_super);
#endif
	}

	return TRUE;
}

/* helpers for parse_axfs_options */
enum {
	OPTION_ERR,
	OPTION_SECOND_DEV,
	OPTION_PHYSICAL_ADDRESS_LOWER_X,
	OPTION_PHYSICAL_ADDRESS_UPPER_X,
	OPTION_IOMEM
};

/* helpers for parse_axfs_options */
static match_table_t tokens = {
	{OPTION_SECOND_DEV, "second_dev=%s"},
	{OPTION_PHYSICAL_ADDRESS_LOWER_X, "physaddr=0x%s"},
	{OPTION_PHYSICAL_ADDRESS_UPPER_X, "physaddr=0X%s"},
	{OPTION_IOMEM, "iomem=%s"},
	{OPTION_ERR, NULL}
};

static int axfs_check_options(char *options, struct axfs_super *sbi)
{
	unsigned long address = 0;
	char *iomem = NULL;
	unsigned long length = 0;
	char *p;
	int err = -EINVAL;
	substring_t args[MAX_OPT_ARGS];

	if (!options)
		return 0;

	if (!*options)
		return 0;

	while ((p = strsep(&options, ",")) != NULL) {
		int token;
		if (!*p)
			continue;

		token = match_token(p, tokens, args);
		switch (token) {
		case OPTION_SECOND_DEV:
			sbi->second_dev = match_strdup(&args[0]);
			if (!(sbi->second_dev)) {
				err = -ENOMEM;
				goto out;
			}
			if (!*(sbi->second_dev))
				goto bad_value;
			break;
		case OPTION_IOMEM:
			iomem = match_strdup(&args[0]);
			if (!(iomem)) {
				err = -ENOMEM;
				goto out;
			}
			if (!*iomem)
				goto bad_value;
			break;
		case OPTION_PHYSICAL_ADDRESS_LOWER_X:
		case OPTION_PHYSICAL_ADDRESS_UPPER_X:
			if (match_hex(&args[0], (int *)&address))
				goto out;
			if (!address)
				goto bad_value;
			break;
		default:
			printk(KERN_ERR
			       "axfs: unrecognized mount option \"%s\" "
			       "or missing value\n", p);
			goto out;
		}
	}

	if (iomem) {
		if (address)
			goto out;
		err = axfs_get_uml_address(iomem, &address, &length);
		kfree(iomem);
		sbi->iomem_size = length;
		sbi->virt_start_addr = address;
	}

	sbi->phys_start_addr = address;
	return 0;

bad_value:
	printk(KERN_ERR
	       "axfs: unrecognized mount option \"%s\" "
	       "or missing value\n", p);
out:
	if (iomem)
		kfree(iomem);
	return err;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,17)
int axfs_get_sb(struct file_system_type *fs_type, int flags,
		const char *dev_name, void *data, struct vfsmount *mnt)
#else
struct super_block *axfs_get_sb(struct file_system_type *fs_type, int flags,
				const char *dev_name, void *data)
#endif
{
	struct axfs_super *sbi;
	int err;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,17)
#else
	struct super_block *sb;
	struct vfsmount *mnt;
	mnt = kmalloc(sizeof(*mnt), GFP_KERNEL);
	memset(mnt, 0, sizeof(*mnt));
#endif

	sbi = axfs_get_sbi();
	if (IS_ERR(sbi))
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,17)
		return PTR_ERR(sbi);
#else
		return (struct super_block *)sbi;
#endif

	err = axfs_check_options((char *)data, sbi);
	if (err)
		goto out;

	/* First we check if we are mounting directly to memory */
	if (axfs_get_sb_address(fs_type, flags, sbi, mnt, &err))
		goto out;

	/* Next we assume there's a MTD device */
	if (axfs_get_sb_mtd(fs_type, flags, dev_name, sbi, mnt, &err))
		goto out;

	/* Now we assume it's a block device */
	if (sbi->second_dev) {
		printk(KERN_ERR
		       "axfs: can't specify secondary block device %s because"
		       " %s is assumed to be a block device\n", sbi->second_dev,
		       dev_name);
		err = -EINVAL;
		goto out;
	}

	if (axfs_get_sb_bdev(fs_type, flags, dev_name, sbi, mnt, &err))
		goto out;

	err = -EINVAL;

out:
	axfs_put_sbi(sbi);
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,17)
	return err;
#else
	if (err)
		return ERR_PTR(err);

	sb = mnt->mnt_sb;
	kfree(mnt);
	return sb;
#endif
}

static struct axfs_super *axfs_get_sbi(void)
{
	struct axfs_super *sbi;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,10)
	sbi = kzalloc(sizeof(*sbi), GFP_KERNEL);
#else
	sbi = kmalloc(sizeof(*sbi), GFP_KERNEL);
	memset(sbi, 0, sizeof(*sbi));
#endif
	if (sbi)
		return sbi;

	axfs_put_sbi(sbi);
	return ERR_PTR(-ENOMEM);
}

static void axfs_kill_super(struct super_block *sb)
{
	if (AXFS_NODEV(sb))
		return kill_anon_super(sb);

	if (AXFS_HAS_MTD(sb))
		axfs_kill_mtd_super(sb);

	if (AXFS_HAS_BDEV(sb))
		axfs_kill_block_super(sb);
}

static int axfs_remount(struct super_block *sb, int *flags, char *data)
{
	*flags |= MS_RDONLY;
	return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,17)
static int axfs_statfs(struct dentry *dentry, struct kstatfs *buf)
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)
static int axfs_statfs(struct super_block *sb, struct kstatfs *buf)
#else
static int axfs_statfs(struct super_block *sb, struct statfs *buf)
#endif
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,17)
	struct axfs_super *sbi = AXFS_SB(dentry->d_sb);
#else
	struct axfs_super *sbi = AXFS_SB(sb);
#endif

	buf->f_type = AXFS_MAGIC;
	buf->f_bsize = AXFS_PAGE_SIZE;
	buf->f_blocks = sbi->blocks;
	buf->f_bfree = 0;
	buf->f_bavail = 0;
	buf->f_files = sbi->files;
	buf->f_ffree = 0;
	buf->f_namelen = AXFS_MAXPATHLEN;
	return 0;
}

static struct super_operations axfs_sops = {
	.put_super = axfs_put_super,
	.remount_fs = axfs_remount,
	.statfs = axfs_statfs,
};

static struct file_system_type axfs_fs_type = {
	.owner = THIS_MODULE,
	.name = "axfs",
	.get_sb = axfs_get_sb,
	.kill_sb = axfs_kill_super,
};

static int __init init_axfs_fs(void)
{
	axfs_uncompress_init();
	return register_filesystem(&axfs_fs_type);
}

static void __exit exit_axfs_fs(void)
{
	axfs_uncompress_exit();
	unregister_filesystem(&axfs_fs_type);
}

module_init(init_axfs_fs);
module_exit(exit_axfs_fs);
MODULE_LICENSE("GPL");
