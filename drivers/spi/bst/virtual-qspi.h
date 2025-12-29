/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __VIRTUAL_BST_H__
#define __VIRTUAL_BST_H__

#define MAX_BLOCK_SIZE 0x10000
#define ERASE_BLOCK_SIZE 0x1000
struct virtual_qspi_ {
	struct mtd_info *mtd_info;
	unsigned int safety_addr;
	SocClient_t *client;
	dma_addr_t partition_dma_addr;
	void *map_addr;
};

struct nor_ipcmsg_ {
	u32 addr;
	u32 maxsize;
	u32 offset;
	u32 size;
	u32 bus_num;
	u32 flag; //0：read,1: write 2:erase
};

#endif /* __NULL_BLK_H */
