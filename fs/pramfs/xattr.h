/*
 * BRIEF DESCRIPTION
 *
 * Extended attributes for the pram filesystem.
 *
 * Copyright 2010-2011 Marco Stornelli <marco.stornelli@gmail.com>
 *
 * based on fs/ext2/xattr.h with the following copyright:
 *
 *(C) 2001 Andreas Gruenbacher, <a.gruenbacher@computer.org>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/init.h>
#include <linux/xattr.h>

/* Magic value in attribute blocks */
#define PRAM_XATTR_MAGIC		0x6d617270

/* Maximum number of references to one attribute block */
#define PRAM_XATTR_REFCOUNT_MAX		1024

/* Name indexes */
#define PRAM_XATTR_INDEX_USER			1
#define PRAM_XATTR_INDEX_POSIX_ACL_ACCESS	2
#define PRAM_XATTR_INDEX_POSIX_ACL_DEFAULT	3
#define PRAM_XATTR_INDEX_TRUSTED		4
#define PRAM_XATTR_INDEX_SECURITY	        5

struct pram_xattr_header {
	__be32	h_magic;	/* magic number for identification */
	__be32	h_refcount;	/* reference count */
	__be32	h_hash;		/* hash value of all attributes */
	__u32	h_reserved[4];	/* zero right now */
};

struct pram_xattr_entry {
	__u8	e_name_len;	/* length of name */
	__u8	e_name_index;	/* attribute name index */
	__be16	e_value_offs;	/* offset in disk block of value */
	__be32	e_value_block;	/* disk block attribute is stored on (n/i) */
	__be32	e_value_size;	/* size of attribute value */
	__be32	e_hash;		/* hash value of name and value */
	char	e_name[0];	/* attribute name */
};

#define PRAM_XATTR_PAD_BITS		2
#define PRAM_XATTR_PAD		(1<<PRAM_XATTR_PAD_BITS)
#define PRAM_XATTR_ROUND		(PRAM_XATTR_PAD-1)
#define PRAM_XATTR_LEN(name_len) \
	(((name_len) + PRAM_XATTR_ROUND + \
	sizeof(struct pram_xattr_entry)) & ~PRAM_XATTR_ROUND)
#define PRAM_XATTR_NEXT(entry) \
	((struct pram_xattr_entry *)( \
	  (char *)(entry) + PRAM_XATTR_LEN((entry)->e_name_len)))
#define PRAM_XATTR_SIZE(size) \
	(((size) + PRAM_XATTR_ROUND) & ~PRAM_XATTR_ROUND)

#ifdef CONFIG_PRAMFS_XATTR

extern const struct xattr_handler pram_xattr_user_handler;
extern const struct xattr_handler pram_xattr_trusted_handler;
extern const struct xattr_handler pram_xattr_acl_access_handler;
extern const struct xattr_handler pram_xattr_acl_default_handler;
extern const struct xattr_handler pram_xattr_security_handler;

extern ssize_t pram_listxattr(struct dentry *, char *, size_t);

extern int pram_xattr_get(struct inode *, int, const char *, void *, size_t);
extern int pram_xattr_set(struct inode *, int, const char *, const void *,
			  size_t, int);

extern void pram_xattr_delete_inode(struct inode *);
extern void pram_xattr_put_super(struct super_block *);

extern int init_pram_xattr(void) __init;
extern void exit_pram_xattr(void);

extern const struct xattr_handler *pram_xattr_handlers[];

# else  /* CONFIG_PRAMFS_XATTR */

static inline int
pram_xattr_get(struct inode *inode, int name_index,
	       const char *name, void *buffer, size_t size)
{
	return -EOPNOTSUPP;
}

static inline int
pram_xattr_set(struct inode *inode, int name_index, const char *name,
	       const void *value, size_t size, int flags)
{
	return -EOPNOTSUPP;
}

static inline void
pram_xattr_delete_inode(struct inode *inode)
{
}

static inline void
pram_xattr_put_super(struct super_block *sb)
{
}

static inline int
init_pram_xattr(void)
{
	return 0;
}

static inline void
exit_pram_xattr(void)
{
}

#define pram_xattr_handlers NULL

# endif  /* CONFIG_PRAMFS_XATTR */

#ifdef CONFIG_PRAMFS_SECURITY
extern int pram_init_security(struct inode *inode, struct inode *dir,
				const struct qstr *qstr);
#else
static inline int pram_init_security(struct inode *inode, struct inode *dir,
					const struct qstr *qstr)
{
	return 0;
}
#endif
