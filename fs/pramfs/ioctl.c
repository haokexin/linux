/*
 * BRIEF DESCRIPTION
 *
 * Ioctl operations.
 *
 * Copyright 2010 Marco Stornelli <marco.stornelli@gmail.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/capability.h>
#include <linux/time.h>
#include <linux/sched.h>
#include <linux/compat.h>
#include <linux/mount.h>
#include "pram.h"

long pram_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct inode *inode = filp->f_dentry->d_inode;
	struct pram_inode *pi;
	unsigned int flags;
	int ret;

	pi = pram_get_inode(inode->i_sb, inode->i_ino);
	if (!pi)
		return -EACCES;

	switch (cmd) {
	case FS_IOC_GETFLAGS:
		flags = be32_to_cpu(pi->i_flags) & FS_FL_USER_VISIBLE;
		return put_user(flags, (int __user *) arg);
	case FS_IOC_SETFLAGS: {
		unsigned int oldflags;

		ret = mnt_want_write(filp->f_path.mnt);
		if (ret)
			return ret;

		if (!is_owner_or_cap(inode)) {
			ret = -EACCES;
			goto flags_out;
		}

		if (get_user(flags, (int __user *) arg)) {
			ret = -EFAULT;
			goto flags_out;
		}

		mutex_lock(&inode->i_mutex);
		oldflags = be32_to_cpu(pi->i_flags);

		if ((flags ^ oldflags) & (FS_APPEND_FL | FS_IMMUTABLE_FL)) {
			if (!capable(CAP_LINUX_IMMUTABLE)) {
				mutex_unlock(&inode->i_mutex);
				ret = -EPERM;
				goto flags_out;
			}
		}

		if (!S_ISDIR(inode->i_mode))
			flags &= ~FS_DIRSYNC_FL;

		flags = flags & FS_FL_USER_MODIFIABLE;
		flags |= oldflags & ~FS_FL_USER_MODIFIABLE;
		pram_memunlock_inode(inode->i_sb, pi);
		pi->i_flags = cpu_to_be32(flags);
		inode->i_ctime = CURRENT_TIME_SEC;
		pi->i_ctime = cpu_to_be32(inode->i_ctime.tv_sec);
		pram_set_inode_flags(inode, pi);
		pram_memlock_inode(inode->i_sb, pi);
		mutex_unlock(&inode->i_mutex);
flags_out:
		mnt_drop_write(filp->f_path.mnt);
		return ret;
	}
	case FS_IOC_GETVERSION:
		return put_user(inode->i_generation, (int __user *) arg);
	case FS_IOC_SETVERSION:
		if (!is_owner_or_cap(inode))
			return -EPERM;
		ret = mnt_want_write(filp->f_path.mnt);
		if (ret)
			return ret;
		if (get_user(inode->i_generation, (int __user *) arg)) {
			ret = -EFAULT;
		} else {
			inode->i_ctime = CURRENT_TIME_SEC;
			pram_update_inode(inode);
		}
		mnt_drop_write(filp->f_path.mnt);
		return ret;
	default:
		return -ENOTTY;
	}
}
