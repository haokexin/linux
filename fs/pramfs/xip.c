/*
 * BRIEF DESCRIPTION
 *
 * XIP operations.
 *
 * Copyright 2009-2010 Marco Stornelli <marco.stornelli@gmail.com>
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/genhd.h>
#include <linux/buffer_head.h>
#include "pram.h"
#include "xip.h"

static int pram_find_and_alloc_blocks(struct inode *inode, sector_t iblock,
				     sector_t *data_block, int create)
{
	int err = -EIO;
	u64 block;

	mutex_lock(&PRAM_I(inode)->truncate_mutex);

	block = pram_find_data_block(inode, iblock);

	if (!block) {
		if (!create) {
			err = -ENODATA;
			goto err;
		}

		err = pram_alloc_blocks(inode, iblock, 1);
		if (err)
			goto err;

		block = pram_find_data_block(inode, iblock);
		if (!block) {
			err = -ENODATA;
			goto err;
		}
	}

	*data_block = block;
	err = 0;

 err:
	mutex_unlock(&PRAM_I(inode)->truncate_mutex);
	return err;
}

static inline int __pram_get_block(struct inode *inode, pgoff_t pgoff, int create,
		   sector_t *result)
{
	int rc = 0;

	rc = pram_find_and_alloc_blocks(inode, (sector_t)pgoff, result, create);

	if (rc == -ENODATA)
		BUG_ON(create);

	return rc;
}

int pram_get_xip_mem(struct address_space *mapping, pgoff_t pgoff, int create,
				void **kmem, unsigned long *pfn)
{
	int rc;
	sector_t block;

	/* first, retrieve the block */
	rc = __pram_get_block(mapping->host, pgoff, create, &block);
	if (rc)
		goto exit;

	*kmem = pram_get_block(mapping->host->i_sb, block);
	*pfn = page_to_pfn(virt_to_page((unsigned long)*kmem));

exit:
	return rc;
}
