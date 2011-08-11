#include <linux/linkage.h>
#include <linux/errno.h>

asmlinkage long sys_revokeat(int dfd, const char __user *filename)
{
	return -ENOSYS;
}

