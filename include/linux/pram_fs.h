/*
 * FILE NAME include/linux/pram_fs.h
 *
 * BRIEF DESCRIPTION
 *
 * Definitions for the PRAMFS filesystem.
 *
 * Copyright 2009-2011 Marco Stornelli <marco.stornelli@gmail.com>
 * Copyright 2003 Sony Corporation
 * Copyright 2003 Matsushita Electric Industrial Co., Ltd.
 * 2003-2004 (c) MontaVista Software, Inc. , Steve Longerbeam
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#ifndef _LINUX_PRAM_FS_H
#define _LINUX_PRAM_FS_H

#include <linux/types.h>
#include <linux/magic.h>

/*
 * The PRAM filesystem constants/structures
 */

/*
 * Mount flags
 */
#define PRAM_MOUNT_PROTECT		0x000001  /* Use memory protection */
#define PRAM_MOUNT_XATTR_USER		0x000002  /* Extended user attributes */
#define PRAM_MOUNT_POSIX_ACL		0x000004  /* POSIX ACL */
#define PRAM_MOUNT_XIP			0x000008  /* Execute in place */
#define PRAM_MOUNT_ERRORS_CONT		0x000010  /* Continue on errors */
#define PRAM_MOUNT_ERRORS_RO		0x000020  /* Remount fs ro on errors */
#define PRAM_MOUNT_ERRORS_PANIC		0x000040  /* Panic on errors */

/*
 * Maximal count of links to a file
 */
#define PRAM_LINK_MAX		32000

#define PRAM_MIN_BLOCK_SIZE 512
#define PRAM_MAX_BLOCK_SIZE 4096
#define PRAM_DEF_BLOCK_SIZE 2048

#define PRAM_INODE_SIZE 128 /* must be power of two */
#define PRAM_INODE_BITS   7

/*
 * Structure of a directory entry in PRAMFS.
 * Offsets are to the inode that holds the referenced dentry.
 */
struct pram_dentry {
	__be64	d_next;     /* next dentry in this directory */
	__be64	d_prev;     /* previous dentry in this directory */
	__be64	d_parent;   /* parent directory */
	char	d_name[0];
};


/*
 * Structure of an inode in PRAMFS
 */
struct pram_inode {
	__be16	i_sum;          /* checksum of this inode */
	__be32	i_uid;		/* Owner Uid */
	__be32	i_gid;		/* Group Id */
	__be16	i_mode;		/* File mode */
	__be16	i_links_count;	/* Links count */
	__be32	i_blocks;	/* Blocks count */
	__be32	i_size;		/* Size of data in bytes */
	__be32	i_atime;	/* Access time */
	__be32	i_ctime;	/* Creation time */
	__be32	i_mtime;	/* Modification time */
	__be32	i_dtime;	/* Deletion Time */
	__be64	i_xattr;	/* Extended attribute block */
	__be32	i_generation;	/* File version (for NFS) */
	__be32	i_flags;	/* Inode flags */

	union {
		struct {
			/*
			 * ptr to row block of 2D block pointer array,
			 * file block #'s 0 to (blocksize/8)^2 - 1.
			 */
			__be64 row_block;
		} reg;   /* regular file or symlink inode */
		struct {
			__be64 head; /* first entry in this directory */
			__be64 tail; /* last entry in this directory */
		} dir;
		struct {
			__be32 rdev; /* major/minor # */
		} dev;   /* device inode */
	} i_type;

	struct pram_dentry i_d;
};

#define PRAM_NAME_LEN \
	(PRAM_INODE_SIZE - offsetof(struct pram_inode, i_d.d_name) - 1)


#define PRAM_SB_SIZE 128 /* must be power of two */

/*
 * Structure of the super block in PRAMFS
 */
struct pram_super_block {
	__be16	s_sum;          /* checksum of this sb, including padding */
	__be64	s_size;         /* total size of fs in bytes */
	__be32	s_blocksize;    /* blocksize in bytes */
	__be32	s_inodes_count;	/* total inodes count (used or free) */
	__be32	s_free_inodes_count;/* free inodes count */
	__be32	s_free_inode_hint;  /* start hint for locating free inodes */
	__be32	s_blocks_count;	/* total data blocks count (used or free) */
	__be32	s_free_blocks_count;/* free data blocks count */
	__be32	s_free_blocknr_hint;/* free data blocks count */
	__be64	s_bitmap_start; /* data block in-use bitmap location */
	__be32	s_bitmap_blocks;/* size of bitmap in number of blocks */
	__be32	s_mtime;	/* Mount time */
	__be32	s_wtime;	/* Write time */
	__be16	s_magic;	/* Magic signature */
	char	s_volume_name[16]; /* volume name */
};

/* The root inode follows immediately after the redundant super block */
#define PRAM_ROOT_INO (PRAM_SB_SIZE*2)

#endif	/* _LINUX_PRAM_FS_H */
