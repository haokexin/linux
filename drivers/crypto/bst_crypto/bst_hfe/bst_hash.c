// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2014 Imagination Technologies
 * Copyright (C) 2024 Black Sesame Technologies. Inc.
 * Authors:  Will Thomas, James Hartley
 *
 *	Interface structure taken from omap-sham driver
 */

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/errno.h>
#include <crypto/internal/hash.h>
#include <crypto/md5.h>
#include <crypto/sha1.h>
#include <crypto/sha2.h>
#include <crypto/sha3.h>
#include <crypto/sm3.h>
#include <linux/workqueue.h>
#include "bst_hash.h"

#define HFE_ADDR(offset) (ctx->base + offset)

struct bst_hash_dev;

struct bst_hash_tfm_ctx {
	/* for hmac*/
	// uint8_t key[HASH_BLOCK_MAX_WORD_LEN];
	uint8_t *key;
	uint32_t key_len;
};

struct bst_hash_dev {
	struct list_head list;
	struct device *dev;
	void __iomem *io_base;
	spinlock_t lock;
	int irq;
	struct workqueue_struct *workqueue;
	struct work_struct work;
};

static unsigned int refcnt;
static DEFINE_MUTEX(refcnt_lock);

struct bst_hash_list {
	struct list_head dev_list;
	spinlock_t lock; /* protect dev_list */
};

static struct bst_hash_list hash_list = {
	.dev_list = LIST_HEAD_INIT(hash_list.dev_list),
	.lock = __SPIN_LOCK_UNLOCKED(hash_list.lock),
};

#ifndef HASH_CPU_BIG_ENDIAN
uint32_t const SM3_IV[8] = {
	0x6F168073,
	0xB9B21449,
	0xD7422417,
	0x00068ADA,
	0xBC306FA9,
	0xAA383116,
	0x4DEE8DE3,
	0x4E0EFBB0,
};
uint32_t const MD5_IV[4] = {
	0x67452301,
	0xefcdab89,
	0x98badcfe,
	0x10325476,
};
uint32_t const SHA256_IV[8] = {
	0x67E6096A,
	0x85AE67BB,
	0x72F36E3C,
	0x3AF54FA5,
	0x7F520E51,
	0x8C68059B,
	0xABD9831F,
	0x19CDE05B,
};
uint32_t const SHA384_IV[16] = {
	0x5D9DBBCB,
	0xD89E05C1,
	0x2A299A62,
	0x07D57C36,
	0x5A015991,
	0x17DD7030,
	0xD8EC2F15,
	0x39590EF7,
	0x67263367,
	0x310BC0FF,
	0x874AB48E,
	0x11155868,
	0x0D2E0CDB,
	0xA78FF964,
	0x1D48B547,
	0xA44FFABE,
};
uint32_t const SHA512_IV[16] = {
	0x67E6096A,
	0x08C9BCF3,
	0x85AE67BB,
	0x3BA7CA84,
	0x72F36E3C,
	0x2BF894FE,
	0x3AF54FA5,
	0xF1361D5F,
	0x7F520E51,
	0xD182E6AD,
	0x8C68059B,
	0x1F6C3E2B,
	0xABD9831F,
	0x6BBD41FB,
	0x19CDE05B,
	0x79217E13,
};
uint32_t const SHA1_IV[5] = {
	0x01234567,
	0x89ABCDEF,
	0xFEDCBA98,
	0x76543210,
	0xF0E1D2C3,
};
uint32_t const SHA224_IV[8] = {
	0xD89E05C1,
	0x07D57C36,
	0x17DD7030,
	0x39590EF7,
	0x310BC0FF,
	0x11155868,
	0xA78FF964,
	0xA44FFABE,
};
uint32_t const SHA512_224_IV[16] = {
	0xC8373D8C,
	0xA24D5419,
	0x6699E173,
	0xD6D4DC89,
	0xAEB7FA1D,
	0x829CFF32,
	0x14D59D67,
	0xCF9F2F58,
	0x692B6D0F,
	0xA84DD47B,
	0x736FE377,
	0x4289C404,
	0xA8859D3F,
	0xC8361D6A,
	0xADE61211,
	0xA192D691,
};
uint32_t const SHA512_256_IV[16] = {
	0x94213122,
	0x2CF72BFC,
	0xA35F559F,
	0xC2644CC8,
	0x6BB89323,
	0x51B1536F,
	0x19773896,
	0xBDEA4059,
	0xE23E2896,
	0xE3FF8EA8,
	0x251E5EBE,
	0x92398653,
	0xFC99012B,
	0xAAB8852C,
	0xDC2DB70E,
	0xA22CC581,
};

#else
uint32_t const SM3_IV[8] = {
	0x7380166f,
	0x4914b2b9,
	0x172442d7,
	0xda8a0600,
	0xa96f30bc,
	0x163138aa,
	0xe38dee4d,
	0xb0fb0e4e,
};
uint32_t const MD5_IV[4] = {
	0x01234567,
	0x89ABCDEF,
	0xFEDCBA98,
	0x76543210,
};
uint32_t const SHA256_IV[8] = {
	0x6a09e667,
	0xbb67ae85,
	0x3c6ef372,
	0xa54ff53a,
	0x510e527f,
	0x9b05688c,
	0x1f83d9ab,
	0x5be0cd19,
};
uint32_t const SHA384_IV[16] = {
	0xcbbb9d5d,
	0xc1059ed8,
	0x629a292a,
	0x367cd507,
	0x9159015a,
	0x3070dd17,
	0x152fecd8,
	0xf70e5939,
	0x67332667,
	0xffc00b31,
	0x8eb44a87,
	0x68581511,
	0xdb0c2e0d,
	0x64f98fa7,
	0x47b5481d,
	0xbefa4fa4,
};
uint32_t const SHA512_IV[16] = {
	0x6a09e667,
	0xf3bcc908,
	0xbb67ae85,
	0x84caa73b,
	0x3c6ef372,
	0xfe94f82b,
	0xa54ff53a,
	0x5f1d36f1,
	0x510e527f,
	0xade682d1,
	0x9b05688c,
	0x2b3e6c1f,
	0x1f83d9ab,
	0xfb41bd6b,
	0x5be0cd19,
	0x137e2179,
};
uint32_t const SHA1_IV[5] = {
	0x67452301,
	0xefcdab89,
	0x98badcfe,
	0x10325476,
	0xc3d2e1f0,
};
uint32_t const SHA224_IV[8] = {
	0xc1059ed8,
	0x367cd507,
	0x3070dd17,
	0xf70e5939,
	0xffc00b31,
	0x68581511,
	0x64f98fa7,
	0xbefa4fa4,
};
uint32_t const SHA512_224_IV[16] = {
	0x8C3D37C8,
	0x19544DA2,
	0x73E19966,
	0x89DCD4D6,
	0x1DFAB7AE,
	0x32FF9C82,
	0x679DD514,
	0x582F9FCF,
	0x0F6D2B69,
	0x7BD44DA8,
	0x77E36F73,
	0x04C48942,
	0x3F9D85A8,
	0x6A1D36C8,
	0x1112E6AD,
	0x91D692A1,
};
uint32_t const SHA512_256_IV[16] = {
	0x22312194,
	0xFC2BF72C,
	0x9F555FA3,
	0xC84C64C2,
	0x2393B86B,
	0x6F53B151,
	0x96387719,
	0x5940EABD,
	0x96283EE2,
	0xA88EFFE3,
	0xBE5E1E25,
	0x53863992,
	0x2B0199FC,
	0x2C85B8AA,
	0x0EB72DDC,
	0x81C52CA2,
};

#endif

static int bst_hash_irq_handler(struct bst_hash_dev *dev)
{
	u32 stat, flag;

	stat = readl_relaxed(dev->io_base + (HFE_MDIN_CR));

	//printk("%s:%d 0x%x", __func__, __LINE__, stat);
	if (stat & 1)
		flag = 1;
	else if (stat & (1 << 16))
		flag = 16;
	else
		flag = 0;

	return 0;
}

static irqreturn_t bst_hash_irq(int irq, void *dev_id)
{
	struct bst_hash_dev *hdev = dev_id;
	u32 stat, enabled;

	enabled = readl_relaxed(hdev->io_base + HFE_IMCR);
	stat = readl_relaxed(hdev->io_base + HFE_MISR);
	dev_dbg(hdev->dev, "enabled=%#x stat=%#x\n", enabled, stat);
	if (!enabled || !stat)
		return IRQ_NONE;

	bst_hash_irq_handler(hdev);
	writel_relaxed(readl_relaxed(hdev->io_base + HFE_RISR) & (~1),
				   hdev->io_base + HFE_RISR);

	return IRQ_HANDLED;
}

static void hfe_get_version(void __iomem *io_base, u32 *major, u32 *minor)
{
	*major = (readl_relaxed(io_base + HFE_VERSION) & 0xf0) >> 4;
	*minor = readl_relaxed(io_base + HFE_VERSION) & 0x0f;
}

static void hash_enable_interrupt(struct bst_hash_ctx *ctx)
{
	uint32_t flag = (uint32_t)1;

	writel_relaxed(readl_relaxed(HFE_ADDR(HFE_IMCR)) | flag,
				   HFE_ADDR(HFE_IMCR));
}

static void __attribute__((unused)) hash_disable_interrupt(struct bst_hash_ctx *ctx)
{
	uint32_t mask = ~((uint32_t)1);

	writel_relaxed(readl_relaxed(HFE_ADDR(HFE_IMCR)) & mask,
				   HFE_ADDR(HFE_IMCR));
}

static struct bst_hash_dev *bst_hash_get_dev(void)
{
	struct bst_hash_dev *hash_dev;

	spin_lock_bh(&hash_list.lock);
	hash_dev = list_first_entry(&hash_list.dev_list,
								struct bst_hash_dev, list);
	if (hash_dev)
		list_move_tail(&hash_dev->list, &hash_list.dev_list);
	spin_unlock_bh(&hash_list.lock);

	return hash_dev;
}

static uint32_t hash_get_block_word_len(enum BST_HASH_ALG hash_alg)
{
	uint32_t block_words;

	switch (hash_alg) {
	case HASH_SM3:
	case HASH_MD5:
	case HASH_SHA1:
	case HASH_SHA256:
	case HASH_SHA224:
		block_words = 16;
		break;
	case HASH_SHA384:
	case HASH_SHA512:
	case HASH_SHA512_224:
	case HASH_SHA512_256:
		block_words = 32;
		break;
	default:
		break;
	}

	return block_words;
}

static uint32_t hash_get_iterator_word_len(enum BST_HASH_ALG hash_alg)
{
	uint32_t iterator_words;

	switch (hash_alg) {
	case HASH_MD5:
		iterator_words = 4;
		break;
	case HASH_SHA1:
		iterator_words = 5;
		break;
	case HASH_SM3:
	case HASH_SHA256:
	case HASH_SHA224:
		iterator_words = 8;
		break;
	case HASH_SHA384:
	case HASH_SHA512:
	case HASH_SHA512_224:
	case HASH_SHA512_256:
		iterator_words = 16;
		break;
	default:
		break;
	}

	return iterator_words;
}

static uint32_t hash_total_byte_len_add_uint32(uint32_t *a, uint32_t a_words,
											   uint32_t b)
{
	uint32_t i;

	for (i = 0; i < a_words; i++) {
		a[i] += b;
		if (a[i] < b)
			b = 1;
		else
			break;
	}

	if (i == a_words)
		return 1;
	else if (a[a_words - 1] & 0xE0000000)
		return 1;
	else
		return 0;
}

static uint32_t hash_get_digest_word_len(enum BST_HASH_ALG hash_alg)
{
	uint32_t digest_words;

	switch (hash_alg) {
	case HASH_MD5:
		digest_words = 4;
		break;
	case HASH_SHA1:
		digest_words = 5;
		break;
	case HASH_SHA224:
	case HASH_SHA512_224:
		digest_words = 7;
		break;
	case HASH_SM3:
	case HASH_SHA256:
	case HASH_SHA512_256:
		digest_words = 8;
		break;
	case HASH_SHA384:
		digest_words = 12;
		break;
	case HASH_SHA512:
		digest_words = 16;
		break;
	//case HASH_SHA3_224:
	//	digest_words = 7;
	//	break;
	//case HASH_SHA3_256:
	//	digest_words = 8;
	//	break;
	//case HASH_SHA3_384:
	//	digest_words = 12;
	//	break;
	//case HASH_SHA3_512:
	//	digest_words = 16;
	//	break;
	default:
		break;
	}

	return digest_words;
}

static void hash_set_msg_len(struct bst_hash_ctx *ctx, uint32_t bytelen)
{
	uint32_t flag = 0;
	size_t i;

	writel_relaxed(bytelen << 3, HFE_ADDR(HFE_MSG_LEN));
	writel_relaxed(bytelen >> (32 - 3), HFE_ADDR(HFE_MSG_LEN + 4));
	writel_relaxed(flag, HFE_ADDR(HFE_MSG_LEN + 8));
	writel_relaxed(flag, HFE_ADDR(HFE_MSG_LEN + 12));
	for (i = 0; i < 4; i++)
		writel_relaxed(flag, HFE_ADDR(HFE_MSG_CNT + 4 * i));
}

static void hash_set_iterator(struct bst_hash_ctx *ctx, uint32_t *iterator,
							  uint32_t hash_iterator_words)
{
	uint32_t i;

	if (iterator) {
		for (i = 0; i < hash_iterator_words; i++)
			writel_relaxed(iterator[i], HFE_ADDR(HFE_IN + 4 * i));
	} else {
		for (i = 0; i < hash_iterator_words; i++)
			writel_relaxed(0, HFE_ADDR(HFE_IN + 4 * i));
	}
}

static uint32_t *hash_get_iv(enum BST_HASH_ALG hash_alg)
{
	uint32_t *iv = NULL;

	switch (hash_alg) {
	case HASH_SM3:
		iv = (uint32_t *)SM3_IV;
		break;

	case HASH_MD5:
		iv = (uint32_t *)MD5_IV;
		break;

	case HASH_SHA256:
		iv = (uint32_t *)SHA256_IV;
		break;

	case HASH_SHA384:
		iv = (uint32_t *)SHA384_IV;
		break;
	case HASH_SHA1:
		iv = (uint32_t *)SHA1_IV;
		break;
	case HASH_SHA512:
		iv = (uint32_t *)SHA512_IV;
		break;
	case HASH_SHA224:
		iv = (uint32_t *)SHA224_IV;
		break;
	case HASH_SHA512_224:
		iv = (uint32_t *)SHA512_224_IV;
		break;
	case HASH_SHA512_256:
		iv = (uint32_t *)SHA512_256_IV;
		break;
	// here iv = NULL means SHA3 IV is zero of 1600 bits
	default:
		iv = NULL;
	}

	return iv;
}

static void hash_set_iv(struct bst_hash_ctx *ctx)
{
	hash_set_iterator(ctx, hash_get_iv(ctx->hash_alg), ctx->iterator_word_len);
}

static void hash_start(struct bst_hash_ctx *ctx)
{
	uint32_t clear_flag = 0;
	uint32_t start_flag = 1;

	writel_relaxed(clear_flag, HFE_ADDR(HFE_RISR));
	writel_relaxed(readl_relaxed(HFE_ADDR(HFE_CTRL)) | start_flag,
				   HFE_ADDR(HFE_CTRL));
}

static void hash_start_calculate(struct bst_hash_ctx *ctx)
{
	if (ctx->first_update_flag) {
		if (ctx->hfe_mode == HASH_MODE)
			hash_set_iv(ctx);
		ctx->first_update_flag = 0;
	}
	hash_start(ctx);
}

static void hash_input_msg(struct bst_hash_ctx *ctx, const uint8_t *msg,
						   uint32_t msg_words)
{
	uint32_t tmp;

	if (((uint64_t)msg) & 3) {
		while (msg_words--) {
			memcpy(&tmp, msg, 4);
			writel_relaxed(tmp, HFE_ADDR(HFE_MDIN));
			msg += 4;
		}
	} else {
		while (msg_words--) {
			writel_relaxed(*((uint32_t *)msg), HFE_ADDR(HFE_MDIN));
			msg += 4;
		}
	}
}

static void __attribute__((unused)) hfe_input_msg(void __iomem *io_base, const uint8_t *msg,
						  uint32_t msg_words)
{
	uint32_t tmp;

	if (((uint64_t)msg) & 3) {
		while (msg_words--) {
			memcpy(&tmp, msg, 4);
			writel_relaxed(tmp, io_base + (HFE_MDIN));
			msg += 4;
		}
	} else {
		while (msg_words--) {
			writel_relaxed(*((uint32_t *)msg), io_base + (HFE_MDIN));
			msg += 4;
		}
	}
}

static void hash_wait_till_done(struct bst_hash_ctx *ctx)
{
	uint32_t flag = 1;

	while ((readl_relaxed(HFE_ADDR(HFE_CTRL)) & flag))
		;
}

static void hash_calc_blocks(struct bst_hash_ctx *ctx, const uint8_t *msg,
							 uint32_t block_count)
{
	uint32_t block_word_len = (ctx->block_byte_len) >> 2;

	hash_set_msg_len(ctx, ctx->block_byte_len * block_count);
	hash_start_calculate(ctx);
	while (block_count--) {
		hash_input_msg(ctx, (uint8_t *)msg, block_word_len);
		msg += ctx->block_byte_len;
	}

	hash_wait_till_done(ctx);
}

static int32_t check_hash_alg(enum BST_HASH_ALG hash_alg)
{
	int32_t ret;

	switch (hash_alg) {
	case HASH_SM3:
	case HASH_MD5:
	case HASH_SHA256:
	case HASH_SHA384:
	case HASH_SHA512:
	case HASH_SHA1:
	case HASH_SHA224:
	case HASH_SHA512_224:
	case HASH_SHA512_256:
		// case HASH_SHA3_224:
		// case HASH_SHA3_256:
		// case HASH_SHA3_384:
		// case HASH_SHA3_512:
		ret = HASH_SUCCESS;
		break;
	default:
		ret = HASH_INPUT_INVALID;
		break;
	}

	return ret;
}

static void hash_set_cpu_mode(struct bst_hash_ctx *ctx)
{
	uint32_t mask = ~(((uint32_t)1) << HASH_DMA_OFFSET);

	writel_relaxed(readl_relaxed(HFE_ADDR(HFE_CFG)) & mask, HFE_ADDR(HFE_CFG));
}

static void hash_set_hash_mode(struct bst_hash_ctx *ctx)
{
	uint32_t mask = ~(((uint32_t)1) << HASH_HMAC_OFFSET);

	writel_relaxed(readl_relaxed(HFE_ADDR(HFE_CFG)) & mask, HFE_ADDR(HFE_CFG));
}

static void hash_set_endian_uint32(struct bst_hash_ctx *ctx, uint32_t endian)
{
	uint32_t mask;
	uint32_t flag;

	mask = ~(((uint32_t)3) << HASH_REVERSE_BYTE_ORDER_IN_WORD_OFFSET);
	flag = (((uint32_t)2) << HASH_REVERSE_BYTE_ORDER_IN_WORD_OFFSET);
	if (endian)
		writel_relaxed(readl_relaxed(HFE_ADDR(HFE_CFG)) & mask,
					   HFE_ADDR(HFE_CFG));
	else
		writel_relaxed(readl_relaxed(HFE_ADDR(HFE_CFG)) | flag,
					   HFE_ADDR(HFE_CFG));
}

static void hash_clear_msg_len(struct bst_hash_ctx *ctx)
{
	uint32_t flag = 0;
	size_t i;

	for (i = 0; i < 4; i++) {
		writel_relaxed(flag, HFE_ADDR(HFE_MSG_LEN + 4 * i));
		writel_relaxed(flag, HFE_ADDR(HFE_MSG_CNT + 4 * i));
	}
}

static void hash_disable_cpu_interruption(struct bst_hash_ctx *ctx)
{
	uint32_t mask = ~1;

	writel_relaxed(readl_relaxed(HFE_ADDR(HFE_IMCR)) & mask,
				   HFE_ADDR(HFE_IMCR));
}

static void hash_set_last_block(struct bst_hash_ctx *ctx, uint32_t tag)
{
	uint32_t mask = ~(((uint32_t)1) << HASH_LAST_BLOCK_OFFSET);
	uint32_t flag = (((uint32_t)1) << HASH_LAST_BLOCK_OFFSET);

	if (tag)
		writel_relaxed(readl_relaxed(HFE_ADDR(HFE_MDIN_CR)) | flag,
					   HFE_ADDR(HFE_MDIN_CR));
	else
		writel_relaxed(readl_relaxed(HFE_ADDR(HFE_MDIN_CR)) & mask,
					   HFE_ADDR(HFE_MDIN_CR));
}

static void hash_set_alg(struct bst_hash_ctx *ctx, enum BST_HASH_ALG hash_alg)
{
	uint32_t mask = (~0x0000000F);

	writel_relaxed(readl_relaxed(HFE_ADDR(HFE_CFG)) & mask, HFE_ADDR(HFE_CFG));
	writel_relaxed(readl_relaxed(HFE_ADDR(HFE_CFG)) | hash_alg,
				   HFE_ADDR(HFE_CFG));
}

static void hash_update_config(struct bst_hash_ctx *ctx)
{
	uint32_t mask = ~(((uint32_t)1) << HASH_UPDATE_CONFIG_OFFSET);
	uint32_t flag = ((uint32_t)1) << HASH_UPDATE_CONFIG_OFFSET;
	uint32_t flag_1 = 1;

	writel_relaxed(readl_relaxed(HFE_ADDR(HFE_CFG)) | flag, HFE_ADDR(HFE_CFG));
	writel_relaxed(readl_relaxed(HFE_ADDR(HFE_CTRL)) | flag_1,
				   HFE_ADDR(HFE_CTRL));
	hash_wait_till_done(ctx);
	writel_relaxed(readl_relaxed(HFE_ADDR(HFE_CFG)) & mask, HFE_ADDR(HFE_CFG));
}

static void hash_total_bytelen_2_bitlen(uint32_t *a, uint32_t a_words)
{
	int32_t i;

	for (i = a_words - 1; i > 0; i--) {
		a[i] <<= 3;
		a[i] |= a[i - 1] >> (32 - 3);
	}
	a[i] <<= 3;
}

static void hash_set_msg_total_bit_len(struct bst_hash_ctx *ctx,
									   uint32_t *msg_total_bits, uint32_t block_byte_len)
{
	uint32_t mask_1 = 0xFFFFFE00;
	uint32_t mask_2 = 0xFFFFFC00;
	uint32_t words = HASH_BLOCK_MAX_WORD_LEN / 8;

	while (words--) {
		writel_relaxed(msg_total_bits[words],
					   HFE_ADDR(HFE_MSG_LEN + 4 * words));
		writel_relaxed(msg_total_bits[words],
					   HFE_ADDR(HFE_MSG_CNT + 4 * words));
	}

	if (block_byte_len == 64) {
		writel_relaxed(readl_relaxed(HFE_ADDR(HFE_MSG_CNT)) & mask_1,
					   HFE_ADDR(HFE_MSG_CNT));
	} else {
		writel_relaxed(readl_relaxed(HFE_ADDR(HFE_MSG_CNT)) & mask_2,
					   HFE_ADDR(HFE_MSG_CNT));
	}
}

static void hash_calc_rand_len_msg(struct bst_hash_ctx *ctx,
								   const uint8_t *msg, uint32_t msg_bytes)
{
	hash_set_last_block(ctx, 1);
	hash_start_calculate(ctx);
	hash_input_msg(ctx, (uint8_t *)msg, (msg_bytes + 3) / 4);
	hash_wait_till_done(ctx);
}

static void hash_get_iterator(struct bst_hash_ctx *ctx, uint8_t *iterator,
							  uint32_t hash_iterator_words)
{
	uint32_t temp;
	uint32_t i;

	if (((uint64_t)iterator) & 3) {
		for (i = 0; i < hash_iterator_words; i++) {
			temp = readl_relaxed(HFE_ADDR(HFE_OUT + i * 4));
			memcpy(iterator + (i << 2), &temp, 4);
		}
	} else {
		for (i = 0; i < hash_iterator_words; i++)
			((uint32_t *)iterator)[i] = readl_relaxed(HFE_ADDR(HFE_OUT + i * 4));
	}
}

static void hash_set_hmac_mode(struct bst_hash_ctx *ctx)
{
	uint32_t flag = (((uint32_t)1) << HASH_HMAC_OFFSET);

	writel_relaxed(readl_relaxed(HFE_ADDR(HFE_CFG)) | flag, HFE_ADDR(HFE_CFG));
}

static void hash_set_hmac_key_mode(struct bst_hash_ctx *ctx)
{
	uint32_t flag = 1;

	writel_relaxed(readl_relaxed(HFE_ADDR(HFE_MDIN_CR)) | flag,
				   HFE_ADDR(HFE_MDIN_CR));
}

static void hash_set_hmac_key_cnt(struct bst_hash_ctx *ctx, uint32_t bitlen)
{
	writel_relaxed(bitlen, HFE_ADDR(HFE_KEY_CNT));
}

static void hash_set_hmac_key_len(struct bst_hash_ctx *ctx, uint32_t bitlen)
{
	writel_relaxed(bitlen, HFE_ADDR(HFE_KEY_LEN));
}

static void hash_hmac_key_opr_one_block(struct bst_hash_ctx *ctx)
{
	uint32_t i;
	uint32_t block_words_len = ctx->block_byte_len >> 2;

	hash_set_hmac_key_len(ctx, ctx->block_byte_len << 3);
	hash_set_hmac_key_cnt(ctx, 0);
	hash_set_last_block(ctx, 1);
	hash_start(ctx);
	for (i = 0; i < block_words_len; i++)
		writel_relaxed(ctx->key[i], HFE_ADDR(HFE_MDIN));

	// hash_wait_till_done(ctx);
}

static void hash_input_msg_u8(struct bst_hash_ctx *ctx, const uint8_t *msg,
							  uint32_t msg_bytes)
{
	uint32_t tmp1, tmp2;

	hash_input_msg(ctx, msg, msg_bytes >> 2);
	tmp1 = msg_bytes & 0x00000003;

	if (tmp1 != 0) {
		tmp2 = 0;
		memcpy((uint8_t *)&tmp2, msg + (msg_bytes & 0xFFFFFFFC), tmp1);
		hash_input_msg(ctx, (uint8_t *)&tmp2, 1);
	}
}

static void hash_hmac_key_opr_longer_than_one_block(struct bst_hash_ctx *ctx,
													const uint8_t *key, uint32_t key_bytes)
{
	hash_set_hmac_key_len(ctx, key_bytes << 3);
	hash_set_hmac_key_cnt(ctx, 0);
	hash_set_last_block(ctx, 1);
	hash_start(ctx);
	hash_input_msg_u8(ctx, key, key_bytes);
	hash_wait_till_done(ctx);
}

static void hash_clear_hmac_key_mode(struct bst_hash_ctx *ctx)
{
	uint32_t mask = ~1;

	writel_relaxed(readl_relaxed(HFE_ADDR(HFE_MDIN_CR)) & mask,
				   HFE_ADDR(HFE_MDIN_CR));
}

static void hash_hmac_set_key(struct bst_hash_ctx *ctx, uint8_t *in_key)
{
	hash_set_iv(ctx);
	if (ctx->key_len <= ctx->block_byte_len) {
		ctx->key_len_flag = 1;
		memcpy((uint8_t *)(ctx->key), in_key, ctx->key_len);
		memset(((uint8_t *)(ctx->key)) + ctx->key_len, 0,
			   ctx->block_byte_len - ctx->key_len);
		hash_hmac_key_opr_one_block(ctx);
	} else {
		ctx->key_len_flag = 2;
		hash_hmac_key_opr_longer_than_one_block(ctx, (const uint8_t *)in_key,
												ctx->key_len);
		hash_get_iterator(ctx, ((uint8_t *)(ctx->key)),
						  ctx->digest_byte_len >> 2);
		memset(((uint8_t *)(ctx->key)) + ctx->digest_byte_len, 0,
			   ctx->block_byte_len - ctx->digest_byte_len);
	}
}

static void __attribute__((unused)) hash_hmac_set_key1(struct bst_hash_ctx *ctx)
{
	hash_set_iv(ctx);
	if (ctx->key_len <= ctx->block_byte_len) {
		ctx->key_len_flag = 1;
		memset(((uint8_t *)(ctx->key)) + ctx->key_len, 0,
			   ctx->block_byte_len - ctx->key_len);
		hash_hmac_key_opr_one_block(ctx);
	} else {
		ctx->key_len_flag = 2;
		hash_hmac_key_opr_longer_than_one_block(ctx, (const uint8_t *)ctx->key,
												ctx->key_len);
		hash_get_iterator(ctx, ((uint8_t *)(ctx->key)),
						  ctx->digest_byte_len >> 2);
		memset(((uint8_t *)(ctx->key)) + ctx->digest_byte_len, 0,
			   ctx->block_byte_len - ctx->digest_byte_len);
	}
}

void hash_hmac_disable_secure_port(struct bst_hash_ctx *ctx)
{
	uint32_t mask = ~(((uint32_t)1) << HASH_HMAC_SECURE_PORT_OFFSET);

	writel_relaxed(readl_relaxed(HFE_ADDR(HFE_CFG)) & mask, HFE_ADDR(HFE_CFG));
}

void hash_disable_dma_interruption(struct bst_hash_ctx *ctx)
{
	uint32_t mask = ~(((uint32_t)1) << 1);

	writel_relaxed(readl_relaxed(HFE_ADDR(HFE_IMCR)) & mask,
				   HFE_ADDR(HFE_IMCR));
}
static uint32_t hash_init(struct bst_hash_ctx *ctx)
{
	if (ctx == NULL)
		return HASH_BUFFER_NULL;
	else if (check_hash_alg(ctx->hash_alg) != HASH_SUCCESS)
		return HASH_INPUT_INVALID;

	hash_set_cpu_mode(ctx);
	hash_set_hash_mode(ctx);
	hash_clear_msg_len(ctx);
	hash_disable_cpu_interruption(ctx);
	hash_set_endian_uint32(ctx, 0);
	hash_set_alg(ctx, ctx->hash_alg);
	hash_update_config(ctx);
	ctx->block_byte_len = hash_get_block_word_len(ctx->hash_alg) << 2;
	ctx->iterator_word_len = hash_get_iterator_word_len(ctx->hash_alg);
	ctx->digest_byte_len = hash_get_digest_word_len(ctx->hash_alg) << 2;
	ctx->status.busy = 0;
	ctx->first_update_flag = 1;
	ctx->finish_flag = 0;

	hash_enable_interrupt(ctx);
	hash_set_last_block(ctx, 0);

	return HASH_SUCCESS;
}

static uint32_t hash_update(struct bst_hash_ctx *ctx, const uint8_t *msg,
							uint32_t msg_bytes)
{
	uint32_t count;
	uint8_t left, fill;

	if (ctx == NULL)
		return HASH_BUFFER_NULL;
	else if ((msg == NULL) || (msg_bytes == 0))
		return HASH_SUCCESS;
	ctx->status.busy = 1;
	left = ctx->total[0] % (ctx->block_byte_len);
	fill = (ctx->block_byte_len) - left;

	if (hash_total_byte_len_add_uint32(ctx->total,
									   ctx->block_byte_len / 32, msg_bytes))
		return HASH_LEN_OVERFLOW;

	if (left) {
		if (msg_bytes >= fill) {
			memcpy(ctx->hash_buffer + left, (uint8_t *)msg, fill);
			hash_calc_blocks(ctx, ctx->hash_buffer, 1);
			msg_bytes -= fill;
			msg += fill;
		} else {
			memcpy(ctx->hash_buffer + left, (uint8_t *)msg, msg_bytes);
			goto update_end;
		}
	}

	count = msg_bytes / (ctx->block_byte_len);
	if (count)
		hash_calc_blocks(ctx, msg, count);

	msg_bytes = msg_bytes % (ctx->block_byte_len);
	if (msg_bytes) {
		msg += (ctx->block_byte_len) * count;
		memcpy(ctx->hash_buffer, (uint8_t *)msg, msg_bytes);
	}
update_end:
	ctx->status.busy = 0;

	return HASH_SUCCESS;
}

static uint32_t hash_final(struct bst_hash_ctx *ctx, uint8_t *digest)
{
	uint8_t tmp;

	if ((ctx == NULL) || (digest == NULL))
		return HASH_BUFFER_NULL;

	ctx->finish_flag = 1;
	tmp = ctx->total[0] % (ctx->block_byte_len);

	hash_total_bytelen_2_bitlen(ctx->total, (ctx->block_byte_len) / 32);
	hash_set_msg_total_bit_len(ctx, ctx->total, ctx->block_byte_len);

	hash_calc_rand_len_msg(ctx, ctx->hash_buffer, tmp);
	hash_get_iterator(ctx, digest, (ctx->digest_byte_len) >> 2);

	memset(ctx, 0, sizeof(struct bst_hash_ctx));
	
	return HASH_SUCCESS;
}

static int bst_sm3_init(struct shash_desc *desc)
{
	struct bst_hash_ctx *ctx = shash_desc_ctx(desc);

	memset(ctx, 0, sizeof(struct bst_hash_ctx));
	ctx->base = bst_hash_get_dev()->io_base;
	ctx->hfe_mode = HASH_MODE;
	ctx->hash_alg = HASH_SM3;
	
	return hash_init(ctx);
}

static int bst_md5_init(struct shash_desc *desc)
{
	struct bst_hash_ctx *ctx = shash_desc_ctx(desc);

	memset(ctx, 0, sizeof(struct bst_hash_ctx));
	ctx->base = bst_hash_get_dev()->io_base;
	ctx->hfe_mode = HASH_MODE;
	ctx->hash_alg = HASH_MD5;

	return hash_init(ctx);
}

static int bst_sha256_init(struct shash_desc *desc)
{
	struct bst_hash_ctx *ctx = shash_desc_ctx(desc);

	memset(ctx, 0, sizeof(struct bst_hash_ctx));
	ctx->base = bst_hash_get_dev()->io_base;
	ctx->hfe_mode = HASH_MODE;
	ctx->hash_alg = HASH_SHA256;

	return hash_init(ctx);
}

static int bst_sha1_init(struct shash_desc *desc)
{
	struct bst_hash_ctx *ctx = shash_desc_ctx(desc);

	memset(ctx, 0, sizeof(struct bst_hash_ctx));
	ctx->base = bst_hash_get_dev()->io_base;
	ctx->hfe_mode = HASH_MODE;
	ctx->hash_alg = HASH_SHA1;

	return hash_init(ctx);
}

static int bst_sha224_init(struct shash_desc *desc)
{
	struct bst_hash_ctx *ctx = shash_desc_ctx(desc);

	memset(ctx, 0, sizeof(struct bst_hash_ctx));
	ctx->base = bst_hash_get_dev()->io_base;
	ctx->hfe_mode = HASH_MODE;
	ctx->hash_alg = HASH_SHA224;

	return hash_init(ctx);
}

static int bst_sha512_init(struct shash_desc *desc)
{
	struct bst_hash_ctx *ctx = shash_desc_ctx(desc);

	memset(ctx, 0, sizeof(struct bst_hash_ctx));
	ctx->base = bst_hash_get_dev()->io_base;
	ctx->hfe_mode = HASH_MODE;
	ctx->hash_alg = HASH_SHA512;

	return hash_init(ctx);
}

static int bst_sha512_224_init(struct shash_desc *desc)
{
	struct bst_hash_ctx *ctx = shash_desc_ctx(desc);

	memset(ctx, 0, sizeof(struct bst_hash_ctx));
	ctx->base = bst_hash_get_dev()->io_base;
	ctx->hfe_mode = HASH_MODE;
	ctx->hash_alg = HASH_SHA512_224;

	return hash_init(ctx);
}

static int bst_sha512_256_init(struct shash_desc *desc)
{
	struct bst_hash_ctx *ctx = shash_desc_ctx(desc);

	memset(ctx, 0, sizeof(struct bst_hash_ctx));
	ctx->base = bst_hash_get_dev()->io_base;
	ctx->hfe_mode = HASH_MODE;
	ctx->hash_alg = HASH_SHA512_256;

	return hash_init(ctx);
}

static int bst_hash_update(struct shash_desc *desc, const u8 *data,
						   unsigned int len)
{
	struct bst_hash_ctx *ctx = shash_desc_ctx(desc);

	return hash_update(ctx, data, len);
}

static int bst_hash_final(struct shash_desc *desc, u8 *out)
{
	struct bst_hash_ctx *ctx = shash_desc_ctx(desc);

	return hash_final(ctx, out);
}

static int bst_hash_export(struct shash_desc *desc, void *out)
{
	struct bst_hash_ctx *ctx = shash_desc_ctx(desc);

	memcpy(out, ctx, sizeof(struct bst_hash_ctx));

	return 0;
}

static int bst_hash_import(struct shash_desc *desc, const void *in)
{
	struct bst_hash_ctx *ctx = shash_desc_ctx(desc);

	memcpy(ctx, in, sizeof(struct bst_hash_ctx));

	return 0;
}

static int bst_hmac_init(struct shash_desc *desc, enum BST_HASH_ALG alg)
{
	struct bst_hash_ctx *ctx = shash_desc_ctx(desc);
	struct bst_hash_tfm_ctx *mctx = crypto_shash_ctx(desc->tfm);

	memset(ctx, 0, sizeof(struct bst_hash_ctx));
	ctx->base = bst_hash_get_dev()->io_base;
	ctx->hfe_mode = HMAC_MODE;
	ctx->hash_alg = alg;
	ctx->key_len = mctx->key_len;

	hash_set_cpu_mode(ctx);
	hash_hmac_disable_secure_port(ctx);
	hash_set_hmac_mode(ctx);
	hash_set_hmac_key_mode(ctx);
	hash_disable_dma_interruption(ctx);

	hash_set_endian_uint32(ctx, 0);
	hash_set_alg(ctx, ctx->hash_alg);
	hash_update_config(ctx);
	ctx->block_byte_len = hash_get_block_word_len(ctx->hash_alg) << 2;
	ctx->iterator_word_len = hash_get_iterator_word_len(ctx->hash_alg);
	ctx->digest_byte_len = hash_get_digest_word_len(ctx->hash_alg) << 2;
	ctx->status.busy = 0;
	ctx->first_update_flag = 1;
	ctx->finish_flag = 0;
	hash_hmac_set_key(ctx, mctx->key);
	hash_clear_hmac_key_mode(ctx);
	hash_enable_interrupt(ctx);
	hash_set_last_block(ctx, 0);

	kfree(mctx->key);
	return HASH_SUCCESS;
}

static int bst_hmac_sha256_init(struct shash_desc *desc)
{
	return bst_hmac_init(desc, HASH_SHA256);
}

static int bst_hmac_sha224_init(struct shash_desc *desc)
{
	return bst_hmac_init(desc, HASH_SHA224);
}

static int bst_hmac_sha512_init(struct shash_desc *desc)
{
	return bst_hmac_init(desc, HASH_SHA512);
}

static int bst_hmac_sha512_224_init(struct shash_desc *desc)
{
	return bst_hmac_init(desc, HASH_SHA512_224);
}

static int bst_hmac_sha512_256_init(struct shash_desc *desc)
{
	return bst_hmac_init(desc, HASH_SHA512_256);
}

static int bst_hmac_sha1_init(struct shash_desc *desc)
{
	return bst_hmac_init(desc, HASH_SHA1);
}

static int bst_hmac_md5_init(struct shash_desc *desc)
{
	return bst_hmac_init(desc, HASH_MD5);
}

static int bst_hmac_sm3_init(struct shash_desc *desc)
{
	return bst_hmac_init(desc, HASH_SM3);
}

static int bst_hmac_setkey(struct crypto_shash *tfm, const u8 *key,
						   unsigned int keylen)
{
	struct bst_hash_tfm_ctx *mctx = crypto_shash_ctx(tfm);
	// printk("%s:%d\n", __func__, __LINE__);
	mctx->key = kmalloc(keylen, GFP_KERNEL);
	if (mctx->key == NULL)
		return -ENOMEM;

	memcpy(mctx->key, key, keylen);
	mctx->key_len = keylen;
	
	return 0;
}
void bst_hfe_work_func(struct work_struct *work)
{
	// printk("%s()\n", __func__);

	// mdelay(1000);
	// queue_work(workqueue_test, &work_test);
}
static struct shash_alg bst_algs[] = {
	{.digestsize = SM3_DIGEST_SIZE,
	 .init = bst_sm3_init,
	 .update = bst_hash_update,
	 .final = bst_hash_final,
	 .export = bst_hash_export,
	 .import = bst_hash_import,
	 .descsize = sizeof(struct bst_hash_ctx),
	 .statesize = sizeof(struct bst_hash_ctx),
	 .base = {
		 .cra_name = "bst_sm3",
		 .cra_driver_name = "sm3-bst",
		 .cra_blocksize = SM3_BLOCK_SIZE,
		 .cra_module = THIS_MODULE,
		}
	 },
	{.digestsize = MD5_DIGEST_SIZE, .init = bst_md5_init, .update = bst_hash_update, .final = bst_hash_final, .export = bst_hash_export, .import = bst_hash_import, .descsize = sizeof(struct bst_hash_ctx), .statesize = sizeof(struct bst_hash_ctx), .base = {
																																																														   .cra_name = "bst_md5",
																																																														   .cra_driver_name = "md5-bst",
																																																														   .cra_blocksize = MD5_HMAC_BLOCK_SIZE,
																																																														   .cra_module = THIS_MODULE,
																																																													   }},
	{.digestsize = SHA256_DIGEST_SIZE, .init = bst_sha256_init, .update = bst_hash_update, .final = bst_hash_final, .export = bst_hash_export, .import = bst_hash_import, .descsize = sizeof(struct bst_hash_ctx), .statesize = sizeof(struct bst_hash_ctx), .base = {
																																																																 .cra_name = "bst_sha256",
																																																																 .cra_driver_name = "sha256-bst",
																																																																 .cra_blocksize = SHA256_BLOCK_SIZE,
																																																																 .cra_module = THIS_MODULE,
																																																															 }},
	{.digestsize = SHA1_DIGEST_SIZE, .init = bst_sha1_init, .update = bst_hash_update, .final = bst_hash_final, .export = bst_hash_export, .import = bst_hash_import, .descsize = sizeof(struct bst_hash_ctx), .statesize = sizeof(struct bst_hash_ctx), .base = {
																																																															 .cra_name = "bst_sha1",
																																																															 .cra_driver_name = "sha1-bst",
																																																															 .cra_blocksize = SHA1_BLOCK_SIZE,
																																																															 .cra_module = THIS_MODULE,
																																																														 }},
	{.digestsize = SHA224_DIGEST_SIZE, .init = bst_sha224_init, .update = bst_hash_update, .final = bst_hash_final, .export = bst_hash_export, .import = bst_hash_import, .descsize = sizeof(struct bst_hash_ctx), .statesize = sizeof(struct bst_hash_ctx), .base = {
																																																																 .cra_name = "bst_sha224",
																																																																 .cra_driver_name = "sha224-bst",
																																																																 .cra_blocksize = SHA224_BLOCK_SIZE,
																																																																 .cra_module = THIS_MODULE,
																																																															 }},
	{.digestsize = SHA512_DIGEST_SIZE, .init = bst_sha512_init, .update = bst_hash_update, .final = bst_hash_final, .export = bst_hash_export, .import = bst_hash_import, .descsize = sizeof(struct bst_hash_ctx), .statesize = sizeof(struct bst_hash_ctx), .base = {
																																																																 .cra_name = "bst_sha512",
																																																																 .cra_driver_name = "sha512-bst",
																																																																 .cra_blocksize = SHA512_BLOCK_SIZE,
																																																																 .cra_module = THIS_MODULE,
																																																															 }},
	{.digestsize = SHA512_DIGEST_SIZE, .init = bst_sha512_224_init, .update = bst_hash_update, .final = bst_hash_final, .export = bst_hash_export, .import = bst_hash_import, .descsize = sizeof(struct bst_hash_ctx), .statesize = sizeof(struct bst_hash_ctx), .base = {
																																																																	 .cra_name = "bst_sha512_224",
																																																																	 .cra_driver_name = "sha512-bst",
																																																																	 .cra_blocksize = SHA512_BLOCK_SIZE,
																																																																	 .cra_module = THIS_MODULE,
																																																																 }},
	{.digestsize = SHA512_DIGEST_SIZE, .init = bst_sha512_256_init, .update = bst_hash_update, .final = bst_hash_final, .export = bst_hash_export, .import = bst_hash_import, .descsize = sizeof(struct bst_hash_ctx), .statesize = sizeof(struct bst_hash_ctx), .base = {
																																																																	 .cra_name = "bst_sha512_256",
																																																																	 .cra_driver_name = "sha512-bst",
																																																																	 .cra_blocksize = SHA512_BLOCK_SIZE,
																																																																	 .cra_module = THIS_MODULE,
																																																																 }},
	{.digestsize = SHA256_DIGEST_SIZE, .init = bst_hmac_sha256_init, .update = bst_hash_update, .final = bst_hash_final, .export = bst_hash_export, .import = bst_hash_import, .setkey = bst_hmac_setkey, .descsize = sizeof(struct bst_hash_ctx), .statesize = sizeof(struct bst_hash_ctx), .base = {
																																																																								 .cra_name = "bst_hmac_sha256",
																																																																								 .cra_driver_name = "hmac-sha256-bst",
																																																																								 .cra_ctxsize = sizeof(struct bst_hash_tfm_ctx),
																																																																								 .cra_blocksize = SHA256_BLOCK_SIZE,
																																																																								 .cra_module = THIS_MODULE,
																																																																							 }},
	{.digestsize = SHA224_DIGEST_SIZE, .init = bst_hmac_sha224_init, .update = bst_hash_update, .final = bst_hash_final, .export = bst_hash_export, .import = bst_hash_import, .setkey = bst_hmac_setkey, .descsize = sizeof(struct bst_hash_ctx), .statesize = sizeof(struct bst_hash_ctx), .base = {
																																																																								 .cra_name = "bst_hmac_sha224",
																																																																								 .cra_driver_name = "hmac-sha224-bst",
																																																																								 .cra_ctxsize = sizeof(struct bst_hash_tfm_ctx),
																																																																								 .cra_blocksize = SHA224_BLOCK_SIZE,
																																																																								 .cra_module = THIS_MODULE,
																																																																							 }},
	{.digestsize = SHA512_DIGEST_SIZE, .init = bst_hmac_sha512_init, .update = bst_hash_update, .final = bst_hash_final, .export = bst_hash_export, .import = bst_hash_import, .setkey = bst_hmac_setkey, .descsize = sizeof(struct bst_hash_ctx), .statesize = sizeof(struct bst_hash_ctx), .base = {
																																																																								 .cra_name = "bst_hmac_sha512",
																																																																								 .cra_driver_name = "hmac-sha512-bst",
																																																																								 .cra_ctxsize = sizeof(struct bst_hash_tfm_ctx),
																																																																								 .cra_blocksize = SHA512_BLOCK_SIZE,
																																																																								 .cra_module = THIS_MODULE,
																																																																							 }},
	{.digestsize = SHA512_DIGEST_SIZE, .init = bst_hmac_sha512_224_init, .update = bst_hash_update, .final = bst_hash_final, .export = bst_hash_export, .import = bst_hash_import, .setkey = bst_hmac_setkey, .descsize = sizeof(struct bst_hash_ctx), .statesize = sizeof(struct bst_hash_ctx), .base = {
																																																																									 .cra_name = "bst_hmac_sha512_224",
																																																																									 .cra_driver_name = "hmac-sha512-224-bst",
																																																																									 .cra_ctxsize = sizeof(struct bst_hash_tfm_ctx),
																																																																									 .cra_blocksize = SHA512_BLOCK_SIZE,
																																																																									 .cra_module = THIS_MODULE,
																																																																								 }},
	{.digestsize = SHA512_DIGEST_SIZE, .init = bst_hmac_sha512_256_init, .update = bst_hash_update, .final = bst_hash_final, .export = bst_hash_export, .import = bst_hash_import, .setkey = bst_hmac_setkey, .descsize = sizeof(struct bst_hash_ctx), .statesize = sizeof(struct bst_hash_ctx), .base = {
																																																																									 .cra_name = "bst_hmac_sha512_256",
																																																																									 .cra_driver_name = "hmac-sha512-256-bst",
																																																																									 .cra_ctxsize = sizeof(struct bst_hash_tfm_ctx),
																																																																									 .cra_blocksize = SHA512_BLOCK_SIZE,
																																																																									 .cra_module = THIS_MODULE,
																																																																								 }},
	{.digestsize = SHA1_DIGEST_SIZE, .init = bst_hmac_sha1_init, .update = bst_hash_update, .final = bst_hash_final, .export = bst_hash_export, .import = bst_hash_import, .setkey = bst_hmac_setkey, .descsize = sizeof(struct bst_hash_ctx), .statesize = sizeof(struct bst_hash_ctx), .base = {
																																																																							 .cra_name = "bst_hmac_sha1",
																																																																							 .cra_driver_name = "hmac-sha1-bst",
																																																																							 .cra_ctxsize = sizeof(struct bst_hash_tfm_ctx),
																																																																							 .cra_blocksize = SHA1_BLOCK_SIZE,
																																																																							 .cra_module = THIS_MODULE,
																																																																						 }},
	{.digestsize = MD5_DIGEST_SIZE, .init = bst_hmac_md5_init, .update = bst_hash_update, .final = bst_hash_final, .export = bst_hash_export, .import = bst_hash_import, .setkey = bst_hmac_setkey, .descsize = sizeof(struct bst_hash_ctx), .statesize = sizeof(struct bst_hash_ctx), .base = {
																																																																						   .cra_name = "bst_hmac_md5",
																																																																						   .cra_driver_name = "hmac-md5-bst",
																																																																						   .cra_ctxsize = sizeof(struct bst_hash_tfm_ctx),
																																																																						   .cra_blocksize = MD5_HMAC_BLOCK_SIZE,
																																																																						   .cra_module = THIS_MODULE,
																																																																					   }},
	{.digestsize = SM3_DIGEST_SIZE, .init = bst_hmac_sm3_init, .update = bst_hash_update, .final = bst_hash_final, .export = bst_hash_export, .import = bst_hash_import, .setkey = bst_hmac_setkey, .descsize = sizeof(struct bst_hash_ctx), .statesize = sizeof(struct bst_hash_ctx), .base = {
																																																																						   .cra_name = "bst_hmac_sm3",
																																																																						   .cra_driver_name = "hamc-sm3-bst",
																																																																						   .cra_ctxsize = sizeof(struct bst_hash_tfm_ctx),
																																																																						   .cra_blocksize = SM3_BLOCK_SIZE,
																																																																						   .cra_module = THIS_MODULE,
																																																																					   }}};

static const struct of_device_id bst_hash_match[] = {
	{.compatible = "bst,c1200-hfe"},
	{}};
MODULE_DEVICE_TABLE(of, bst_hash_match);

static int bst_hash_probe(struct platform_device *pdev)
{
	struct bst_hash_dev *hdev;
	struct device *dev = &pdev->dev;
	int err, ret;
	u32 v_major, v_minor;
	// u32 irq_remap[2];

	hdev = devm_kzalloc(dev, sizeof(*hdev), GFP_KERNEL);
	if (hdev == NULL)
		return -ENOMEM;

	hdev->dev = dev;
	spin_lock_init(&hdev->lock);
	platform_set_drvdata(pdev, hdev);
	spin_lock(&hash_list.lock);
	list_add(&hdev->list, &hash_list.dev_list);
	spin_unlock(&hash_list.lock);

	/* Register bank */
	hdev->io_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(hdev->io_base)) {
		err = PTR_ERR(hdev->io_base);
		dev_err(dev, "can't ioremap, returned %d\n", err);
		goto res_err;
	}

	hdev->irq = platform_get_irq(pdev, 0);
	if (hdev->irq < 0) {
		if (hdev->irq != -EPROBE_DEFER)
			dev_err(dev, "cannot get irq\n");
		return hdev->irq;
	}

	ret = devm_request_irq(hdev->dev, hdev->irq, bst_hash_irq, IRQF_SHARED,
						   dev_name(hdev->dev), hdev);
	if (ret) {
		dev_err(hdev->dev, "failure requesting irq %i: %d\n",
				hdev->irq, ret);
		return ret;
	}

	mutex_lock(&refcnt_lock);
	if (!refcnt) {
		ret = crypto_register_shashes(bst_algs, ARRAY_SIZE(bst_algs));
		if (ret) {
			mutex_unlock(&refcnt_lock);
			dev_err(dev, "Failed to register\n");
			return ret;
		}
	}

	refcnt++;
	mutex_unlock(&refcnt_lock);

	// INIT_WORK(&hdev->work, bst_hfe_work_func);
	// queue_work(system_wq, &hdev->work);
	hfe_get_version(hdev->io_base, &v_major, &v_minor);
	dev_info(dev, "Hardware version: v%d.%d\n", v_major, v_minor);

	return 0;
res_err:
	return err;
}

static int bst_hash_remove(struct platform_device *pdev)
{
	struct bst_hash_dev *hdev;

	hdev = platform_get_drvdata(pdev);
	spin_lock(&hash_list.lock);
	list_del(&hash_list.dev_list);
	spin_unlock(&hash_list.lock);
	devm_free_irq(hdev->dev, hdev->irq, hdev);
	mutex_lock(&refcnt_lock);
	if (!--refcnt)
		crypto_unregister_shashes(bst_algs, ARRAY_SIZE(bst_algs));

	mutex_unlock(&refcnt_lock);

	return 0;
}

static struct platform_driver bst_hash_driver = {
	.probe = bst_hash_probe,
	.remove = bst_hash_remove,
	.driver = {
		.name = "bst_hfe",
		.of_match_table = bst_hash_match,
	},
};

module_platform_driver(bst_hash_driver);

// static int __init bst_hash_driver_init(void)
// {
// 	printk("%s: %d\n", __func__, __LINE__);
// 	return platform_driver_register(&bst_hash_driver);
// }

// static void __exit bst_hash_driver_exit(void)
// {
// 	printk("%s: %d", __func__, __LINE__);
// 	return platform_driver_unregister(&bst_hash_driver);
// }

// module_init(bst_hash_driver_init);
// module_exit(bst_hash_driver_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("BST Hash Function Engine driver");
