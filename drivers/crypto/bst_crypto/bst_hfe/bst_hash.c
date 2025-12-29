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
#include <linux/of_reserved_mem.h>
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
#include <linux/mutex.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include "bst_hash.h"
#include "../common/bst_sa_common.h"

static DEFINE_MUTEX(op_mutex);
static ktime_t last_op_lock_time;
#define MAX_OP_LOCK_MS 5000
#define OPUPDATETIME {UpdateOPTime();}
#define OPTRYLOCK { CheckOPTimeout(); \
					if (!mutex_trylock(&op_mutex)) { \
						bst_dbg(1, "hfe opmutex is busy, cannot acquire.\n"); \
						return -EBUSY; \
					}else{\
						OPUPDATETIME \
					}}

#define OPUNLOCK {if (mutex_is_locked(&op_mutex)) {\
					mutex_unlock(&op_mutex);\
				}}

#define HFE_ADDR(offset) (global_hash->io_base + offset)
#define CMA_ADDR_OFFSET  (0x800000000 - 0x80000000)

#define USE_REQUEST_CTX 0
#define INIT_USE_REQUEST_CTX 0
#define GETKEY_USE_REQUEST_CTX 0

#if 0
#define write_reg  writel
#define read_reg   readl
#else
#define write_reg  writel_relaxed
#define read_reg   readl_relaxed
#endif


struct bst_hash_dev;

// static uint8_t *hmac_key = NULL;
// static uint32_t hmac_key_len;
struct bst_hash_tfm_ctx {
	struct mutex key_lock;
	uint8_t *keySrc;
	uint32_t keySrc_len;
	uint32_t key_len_flag;
	void __iomem *base;
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

static struct bst_hash_dev* global_hash = NULL;
static unsigned int refcnt = 0;
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

static void __attribute__((unused)) CheckOPTimeout(void){
	ktime_t now = ktime_get();
	s64 delta_ms = ktime_to_ms(ktime_sub(now, last_op_lock_time));

	if (delta_ms > MAX_OP_LOCK_MS) {
		last_op_lock_time = now;
		if (mutex_is_locked(&op_mutex)) {
			mutex_unlock(&op_mutex);
		}
	}
}

static void __attribute__((unused)) UpdateOPTime(void){
	last_op_lock_time = ktime_get();
}



static void hash_enable_cpu_interruption(void)
{
	uint32_t flag = (uint32_t)1;

	write_reg(read_reg(HFE_ADDR(HFE_IMCR)) | flag,
				   HFE_ADDR(HFE_IMCR));
}

static void hash_disable_interruption(void)
{
	uint32_t mask = ~((uint32_t)3);

	write_reg(read_reg(HFE_ADDR(HFE_IMCR)) & mask,
				   HFE_ADDR(HFE_IMCR));
}

static void hash_enable_dma_interruption(void)
{
	uint32_t flag = ((uint32_t)1) << 1;

	write_reg(read_reg(HFE_ADDR(HFE_IMCR)) | flag, HFE_ADDR(HFE_IMCR));
}

static int bst_hash_irq_handler(struct bst_hash_dev *dev)
{
	// u32 stat, flag;

	// stat = read_reg(dev->io_base + (HFE_MDIN_CR));

	// //bst_dbg(2, "%s:%d 0x%x", __func__, __LINE__, stat);
	// if (stat & 1)
	// 	flag = 1;
	// else if (stat & (1 << 16))
	// 	flag = 16;
	// else
	// 	flag = 0;
	// pr_info("bst_hfe_irq_handler");
	hash_disable_interruption();
	return 0;
}

static irqreturn_t bst_hash_irq(int irq, void *dev_id)
{
	struct bst_hash_dev *hdev = dev_id;
	u32 stat, enabled;

	enabled = read_reg(hdev->io_base + HFE_IMCR);
	stat = read_reg(hdev->io_base + HFE_MISR);
	dev_dbg(hdev->dev, "enabled=%#x stat=%#x\n", enabled, stat);
	if (!enabled || !stat)
		return IRQ_NONE;

	bst_hash_irq_handler(hdev);
	write_reg(read_reg(hdev->io_base + HFE_RISR) & (~1),
				   hdev->io_base + HFE_RISR);

	return IRQ_HANDLED;
}

static void hfe_get_version(void __iomem *io_base, u32 *major, u32 *minor)
{
	*major = (read_reg(io_base + HFE_VERSION) & 0xf0) >> 4;
	*minor = read_reg(io_base + HFE_VERSION) & 0x0f;
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

static uint8_t hash_get_block_word_len(enum BST_HASH_ALG hash_alg)
{
	uint8_t block_words;

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

static uint8_t hash_get_iterator_word_len(enum BST_HASH_ALG hash_alg)
{
	uint8_t iterator_words;

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

static uint8_t hash_get_digest_word_len(enum BST_HASH_ALG hash_alg)
{
	uint8_t digest_words;

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

static void hash_set_msg_len(uint32_t bytelen)
{
	uint32_t flag = 0;
	size_t i;

	write_reg(bytelen << 3, HFE_ADDR(HFE_MSG_LEN));
	write_reg(bytelen >> (32 - 3), HFE_ADDR(HFE_MSG_LEN + 4));
	write_reg(flag, HFE_ADDR(HFE_MSG_LEN + 8));
	write_reg(flag, HFE_ADDR(HFE_MSG_LEN + 12));
	for (i = 0; i < 4; i++)
		write_reg(flag, HFE_ADDR(HFE_MSG_CNT + 4 * i));
}

static void hash_set_iterator(uint32_t *iterator,
							  uint32_t hash_iterator_words)
{
	uint32_t i;

	if (iterator) {
		for (i = 0; i < hash_iterator_words; i++)
			write_reg(iterator[i], HFE_ADDR(HFE_IN + 4 * i));
	} else {
		for (i = 0; i < hash_iterator_words; i++)
			write_reg(0, HFE_ADDR(HFE_IN + 4 * i));
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

static void hash_start(void)
{
	uint32_t clear_flag = 0;
	uint32_t start_flag = 1;
	wmb();
	writel(clear_flag, HFE_ADDR(HFE_RISR));
	writel(read_reg(HFE_ADDR(HFE_CTRL)) | start_flag,
				   HFE_ADDR(HFE_CTRL));
}

static void hash_start_calculate(struct bst_hash_ctx *ctx)
{
	if (ctx->first_update_flag) {
		if (ctx->hfe_mode == HASH_MODE)
			hash_set_iterator(hash_get_iv(ctx->hash_alg), ctx->iterator_word_len);
		ctx->first_update_flag = 0;
	}
	hash_start();
}

static void hash_input_msg(const uint8_t *msg,
						   uint32_t msg_words)
{
	uint32_t tmp;

	if (((uint64_t)msg) & 3) {
		while (msg_words--) {
			memcpy(&tmp, msg, 4);
			write_reg(tmp, HFE_ADDR(HFE_MDIN));
			msg += 4;
		}
	} else {
		while (msg_words--) {
			write_reg(*((uint32_t *)msg), HFE_ADDR(HFE_MDIN));
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
			write_reg(tmp, io_base + (HFE_MDIN));
			msg += 4;
		}
	} else {
		while (msg_words--) {
			write_reg(*((uint32_t *)msg), io_base + (HFE_MDIN));
			msg += 4;
		}
	}
}

static void hash_wait_till_done(void)
{
	uint32_t flag = 1;

	while ((read_reg(HFE_ADDR(HFE_CTRL)) & flag))
		;
}

static void hash_dma_wait_till_done(HASH_CALLBACK callback)
{
	uint32_t flag = 1;

	while ((read_reg(HFE_ADDR(HFE_CTRL)) & flag))
	{
		if (callback)
			callback();
	}
}

static void hash_calc_blocks(struct bst_hash_ctx *ctx, const uint8_t *msg,
							 uint32_t block_count)
{
	uint32_t block_word_len = (ctx->block_byte_len) >> 2;

	hash_set_msg_len(ctx->block_byte_len * block_count);
	hash_start_calculate(ctx);
	while (block_count--) {
		hash_input_msg((uint8_t *)msg, block_word_len);
		msg += ctx->block_byte_len;
	}

	hash_wait_till_done();
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

static void hash_set_cpu_mode(void)
{
	uint32_t mask = ~(((uint32_t)1) << HASH_DMA_OFFSET);

	write_reg(read_reg(HFE_ADDR(HFE_CFG)) & mask, HFE_ADDR(HFE_CFG));
}

static void hash_set_dma_mode(void)
{
	uint32_t flag = ((uint32_t)1) << HASH_DMA_OFFSET;

	write_reg(read_reg(HFE_ADDR(HFE_CFG)) | flag, HFE_ADDR(HFE_CFG));
}

static void hash_set_hash_mode(void)
{
	uint32_t mask = ~(((uint32_t)1) << HASH_HMAC_OFFSET);

	write_reg(read_reg(HFE_ADDR(HFE_CFG)) & mask, HFE_ADDR(HFE_CFG));
}

static void hash_set_endian_uint32(uint32_t endian)
{
	uint32_t mask;
	uint32_t flag;

	mask = ~(((uint32_t)3) << HASH_REVERSE_BYTE_ORDER_IN_WORD_OFFSET);
	flag = (((uint32_t)2) << HASH_REVERSE_BYTE_ORDER_IN_WORD_OFFSET);
	if (endian)
		write_reg(read_reg(HFE_ADDR(HFE_CFG)) & mask,
					   HFE_ADDR(HFE_CFG));
	else
		write_reg(read_reg(HFE_ADDR(HFE_CFG)) | flag,
					   HFE_ADDR(HFE_CFG));
}

static void hash_clear_msg_len(void)
{
	uint32_t flag = 0;
	size_t i;

	for (i = 0; i < 4; i++) {
		write_reg(flag, HFE_ADDR(HFE_MSG_LEN + 4 * i));
		write_reg(flag, HFE_ADDR(HFE_MSG_CNT + 4 * i));
	}
}

static void hash_set_dma_output_len(uint32_t bytes)
{
	write_reg(bytes, HFE_ADDR(HFE_DMA_WLEN));
}

static void hash_clear_dma_sa_da(void)
{
	uint32_t flag = 0;

	write_reg(flag, HFE_ADDR(HFE_DMA_L_SADDR));
	write_reg(flag, HFE_ADDR(HFE_DMA_H_SADDR));

	write_reg(flag, HFE_ADDR(HFE_DMA_L_DADDR));
	write_reg(flag, HFE_ADDR(HFE_DMA_H_DADDR));
}

static void hash_set_last_block(uint32_t tag)
{
	uint32_t mask = ~(((uint32_t)1) << HASH_LAST_BLOCK_OFFSET);
	uint32_t flag = (((uint32_t)1) << HASH_LAST_BLOCK_OFFSET);

	if (tag)
		write_reg(read_reg(HFE_ADDR(HFE_MDIN_CR)) | flag,
					   HFE_ADDR(HFE_MDIN_CR));
	else
		write_reg(read_reg(HFE_ADDR(HFE_MDIN_CR)) & mask,
					   HFE_ADDR(HFE_MDIN_CR));
}

static void hash_set_alg(enum BST_HASH_ALG hash_alg)
{
	uint32_t mask = (~0x0000000F);

	write_reg(read_reg(HFE_ADDR(HFE_CFG)) & mask, HFE_ADDR(HFE_CFG));
	write_reg(read_reg(HFE_ADDR(HFE_CFG)) | hash_alg,
				   HFE_ADDR(HFE_CFG));
}

static void hash_update_config(void)
{
	uint32_t mask = ~(((uint32_t)1) << HASH_UPDATE_CONFIG_OFFSET);
	uint32_t flag = ((uint32_t)1) << HASH_UPDATE_CONFIG_OFFSET;
	uint32_t flag_1 = 1;

	write_reg(read_reg(HFE_ADDR(HFE_CFG)) | flag, HFE_ADDR(HFE_CFG));
	write_reg(read_reg(HFE_ADDR(HFE_CTRL)) | flag_1,
				   HFE_ADDR(HFE_CTRL));
	hash_wait_till_done();
	write_reg(read_reg(HFE_ADDR(HFE_CFG)) & mask, HFE_ADDR(HFE_CFG));
}

static void hash_clear_risp(void)
{
	uint32_t mask = ~((uint32_t)3);

	write_reg(read_reg(HFE_ADDR(HFE_RISR)) & mask, HFE_ADDR(HFE_RISR));
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

static void hash_set_msg_total_bit_len(uint32_t *msg_total_bits, uint32_t block_byte_len)
{
	uint32_t mask_1 = 0xFFFFFE00;
	uint32_t mask_2 = 0xFFFFFC00;
	uint32_t words = HASH_BLOCK_MAX_WORD_LEN / 8;

	while (words--) {
		write_reg(msg_total_bits[words],
					   HFE_ADDR(HFE_MSG_LEN + 4 * words));
		write_reg(msg_total_bits[words],
					   HFE_ADDR(HFE_MSG_CNT + 4 * words));
	}

	if (block_byte_len == 64) {
		write_reg(read_reg(HFE_ADDR(HFE_MSG_CNT)) & mask_1,
					   HFE_ADDR(HFE_MSG_CNT));
	} else {
		write_reg(read_reg(HFE_ADDR(HFE_MSG_CNT)) & mask_2,
					   HFE_ADDR(HFE_MSG_CNT));
	}
}

static void hash_calc_rand_len_msg(struct bst_hash_ctx *ctx,
								   const uint8_t *msg, uint32_t msg_bytes)
{
	hash_set_last_block(1);
	hash_start_calculate(ctx);
	hash_input_msg((uint8_t *)msg, (msg_bytes + 3) / 4);
	hash_wait_till_done();
}

static void hash_get_iterator(uint8_t *iterator, uint32_t hash_iterator_words)
{
	uint32_t temp;
	uint32_t i;

	if (((uint64_t)iterator) & 3) {
		for (i = 0; i < hash_iterator_words; i++) {
			temp = read_reg(HFE_ADDR(HFE_OUT + i * 4));
			memcpy(iterator + (i << 2), &temp, 4);
		}
	} else {
		for (i = 0; i < hash_iterator_words; i++)
			((uint32_t *)iterator)[i] = read_reg(HFE_ADDR(HFE_OUT + i * 4));
	}
}
static void hash_dma_operate(const uint64_t in_addr, const uint64_t out_addr, uint32_t bytelen, HASH_CALLBACK callback)
{
	write_reg((uint32_t)(in_addr & 0xFFFFFFFF), HFE_ADDR(HFE_DMA_L_SADDR));
	if (sizeof(uint32_t *) != 4) {
		write_reg((uint32_t)(in_addr >> 32), HFE_ADDR(HFE_DMA_H_SADDR));
	} else {
		write_reg((uint32_t)0, HFE_ADDR(HFE_DMA_H_SADDR));
	}
	//dst addr
	if (out_addr) {
		write_reg((uint32_t)(out_addr & 0xFFFFFFFF), HFE_ADDR(HFE_DMA_L_DADDR));
		if (sizeof(uint32_t *) != 4) {
			write_reg((uint32_t)(out_addr >> 32), HFE_ADDR(HFE_DMA_H_DADDR));
		} else {
			write_reg((uint32_t)0, HFE_ADDR(HFE_DMA_H_DADDR));
		}
	}

	write_reg(bytelen, HFE_ADDR(HFE_DMA_RLEN));

	hash_start();

	hash_dma_wait_till_done(callback);
}
static void hash_set_hmac_mode(void)
{
	uint32_t flag = (((uint32_t)1) << HASH_HMAC_OFFSET);

	write_reg(read_reg(HFE_ADDR(HFE_CFG)) | flag, HFE_ADDR(HFE_CFG));
}

static void hash_set_hmac_key_mode(void)
{
	uint32_t flag = 1;

	write_reg(read_reg(HFE_ADDR(HFE_MDIN_CR)) | flag,
				   HFE_ADDR(HFE_MDIN_CR));
}

static void hash_set_hmac_key_cnt(uint32_t bitlen)
{
	write_reg(bitlen, HFE_ADDR(HFE_KEY_CNT));
}

static void hash_set_hmac_key_len(uint32_t bitlen)
{
	write_reg(bitlen, HFE_ADDR(HFE_KEY_LEN));
}

static void hash_hmac_key_opr_one_block(uint32_t block_byte_len, uint32_t *ctx_key)
{
	uint32_t i;
	uint32_t block_words_len = block_byte_len >> 2;

	hash_set_hmac_key_len(block_byte_len << 3);
	hash_set_hmac_key_cnt(0);
	hash_set_last_block(1);
	hash_start();
	for (i = 0; i < block_words_len; i++)
		write_reg(ctx_key[i], HFE_ADDR(HFE_MDIN));

	// hash_wait_till_done();
}

static void hash_input_msg_u8(const uint8_t *msg, uint32_t msg_bytes)
{
	uint32_t tmp1, tmp2;

	hash_input_msg(msg, msg_bytes >> 2);
	tmp1 = msg_bytes & 0x00000003;

	if (tmp1 != 0) {
		tmp2 = 0;
		memcpy((uint8_t *)&tmp2, msg + (msg_bytes & 0xFFFFFFFC), tmp1);
		hash_input_msg((uint8_t *)&tmp2, 1);
	}
}

static void hash_hmac_key_opr_longer_than_one_block(uint32_t *ctx_key, const uint8_t *key, uint32_t key_bytes)
{
	hash_set_hmac_key_len(key_bytes << 3);
	hash_set_hmac_key_cnt(0);
	hash_set_last_block(1);
	hash_start();
	hash_input_msg_u8(key, key_bytes);
	hash_wait_till_done();
}

static void hash_clear_hmac_key_mode(void)
{
	uint32_t mask = ~1;

	write_reg(read_reg(HFE_ADDR(HFE_MDIN_CR)) & mask,
				   HFE_ADDR(HFE_MDIN_CR));
}

static void hash_hmac_set_key(struct bst_hash_ctx *ctx, uint8_t *in_key)
{
	printHex("usekey",in_key, ctx->key_len);
	hash_set_iterator(hash_get_iv(ctx->hash_alg), ctx->iterator_word_len);
	if (ctx->key_len <= ctx->block_byte_len) {
		ctx->key_len_flag = 1;
		memcpy((uint8_t *)(ctx->key), in_key, ctx->key_len);
		memset(((uint8_t *)(ctx->key)) + ctx->key_len, 0,
			   ctx->block_byte_len - ctx->key_len);
		hash_hmac_key_opr_one_block(ctx->block_byte_len, (uint32_t *)(ctx->key));
	} else {
		ctx->key_len_flag = 2;
		hash_hmac_key_opr_longer_than_one_block((uint32_t *)(ctx->key), (const uint8_t *)in_key,
												ctx->key_len);
		hash_get_iterator((uint8_t *)(ctx->key), ctx->digest_byte_len >> 2);
		memset(((uint8_t *)(ctx->key)) + ctx->digest_byte_len, 0,
			   ctx->block_byte_len - ctx->digest_byte_len);
	}
}

static void hash_hmac_dma_set_key(struct bst_hash_dma_ctx *ctx, uint8_t *in_key)
{
	printHex("usekey",in_key, ctx->key_len);
	hash_set_iterator(hash_get_iv(ctx->hash_alg), ctx->iterator_word_len);
	if (ctx->key_len <= ctx->block_byte_len) {
		ctx->key_len_flag = 1;
		memcpy((uint8_t *)(ctx->key), in_key, ctx->key_len);
		memset(((uint8_t *)(ctx->key)) + ctx->key_len, 0,
			   ctx->block_byte_len - ctx->key_len);
		hash_hmac_key_opr_one_block(ctx->block_byte_len, (uint32_t *)(ctx->key));
	} else {
		ctx->key_len_flag = 2;
		hash_hmac_key_opr_longer_than_one_block((uint32_t *)(ctx->key), (const uint8_t *)in_key,
												ctx->key_len);
		hash_get_iterator((uint8_t *)(ctx->key), ctx->digest_byte_len >> 2);
		memset(((uint8_t *)(ctx->key)) + ctx->digest_byte_len, 0,
			   ctx->block_byte_len - ctx->digest_byte_len);
	}
}


static void hash_hmac_disable_secure_port(void)
{
	uint32_t mask = ~(((uint32_t)1) << HASH_HMAC_SECURE_PORT_OFFSET);

	write_reg(read_reg(HFE_ADDR(HFE_CFG)) & mask, HFE_ADDR(HFE_CFG));
}

static void hash_dma_callback(void)
{

}

static uint32_t hash_init(struct bst_hash_ctx *ctx)
{
	if (ctx == NULL)
		return HASH_BUFFER_NULL;
	else if (check_hash_alg(ctx->hash_alg) != HASH_SUCCESS)
		return HASH_INPUT_INVALID;

	hash_set_cpu_mode();
	hash_set_hash_mode();
	hash_clear_msg_len();
	hash_disable_interruption();
	hash_set_endian_uint32(0);
	hash_set_alg(ctx->hash_alg);
	hash_update_config();
	ctx->block_byte_len = hash_get_block_word_len(ctx->hash_alg) << 2;
	ctx->iterator_word_len = hash_get_iterator_word_len(ctx->hash_alg);
	ctx->digest_byte_len = hash_get_digest_word_len(ctx->hash_alg) << 2;
	ctx->status.busy = 0;
	ctx->first_update_flag = 1;
	ctx->finish_flag = 0;

	hash_enable_cpu_interruption();
	hash_set_last_block(0);
	ctx->inited = 1;

	return HASH_SUCCESS;
}

static uint32_t hash_dma_init(struct bst_hash_dma_ctx *ctx, HASH_CALLBACK callback)
{
	if (NULL == ctx || NULL == callback)
		return HASH_BUFFER_NULL;
	else if (check_hash_alg(ctx->hash_alg) != HASH_SUCCESS)
		return HASH_INPUT_INVALID;

	hash_set_dma_mode();
	hash_set_hash_mode();
	hash_clear_msg_len();
	hash_disable_interruption();
	hash_set_endian_uint32(0);
	hash_set_alg(ctx->hash_alg);
	hash_update_config();
	hash_clear_risp();
	hash_set_dma_output_len(0);
	hash_clear_dma_sa_da();
	ctx->digest_byte_len = hash_get_digest_word_len(ctx->hash_alg) << 2;
	ctx->iterator_word_len = hash_get_iterator_word_len(ctx->hash_alg);
	ctx->block_word_len = hash_get_block_word_len(ctx->hash_alg);
	ctx->callback = callback;
	uint32_clear(ctx->total, sizeof(ctx->total)/4);

	//set IV
	hash_set_iterator(hash_get_iv(ctx->hash_alg), ctx->iterator_word_len);

	hash_enable_dma_interruption();
	hash_set_last_block(0);

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
	hash_set_msg_total_bit_len(ctx->total, ctx->block_byte_len);

	hash_calc_rand_len_msg(ctx, ctx->hash_buffer, tmp);
	hash_get_iterator(digest, (ctx->digest_byte_len) >> 2);

	#if USE_REQUEST_CTX == 1
	memset(ctx, 0, sizeof(struct bst_hash_ctx));
	#else
	memset(ctx, 0, offsetof(struct  bst_hash_ctx, key_lock));
	#endif
	
	return HASH_SUCCESS;
}

static uint32_t hash_dma_update_blocks(struct bst_hash_dma_ctx *ctx, const uint32_t *msg,
							uint32_t msg_words)
{
	if (ctx == NULL)
		return HASH_BUFFER_NULL;
	else if((msg == NULL) || (msg_words == 0))
		return HASH_SUCCESS;
	else if(msg_words % ctx->block_word_len)
		return HASH_INPUT_INVALID;

	if(hash_total_byte_len_add_uint32(ctx->total, ctx->block_word_len/8, msg_words * 4))
		return HASH_LEN_OVERFLOW;
	hash_set_msg_len(msg_words * 4);

	hash_dma_operate((uint64_t)(ctx->dma_addr.phys_in - CMA_ADDR_OFFSET), 0, msg_words * 4, ctx->callback);

	return HASH_SUCCESS;
}

static uint32_t hash_dma_final(struct bst_hash_dma_ctx *ctx)
{
	if (ctx == NULL)
		return HASH_BUFFER_NULL;
	if (ctx->remainder_msg == NULL)
		ctx->remainder_bytes = 0;
	if (ctx->remainder_bytes >= (ctx->block_word_len << 2))
		return HASH_INPUT_INVALID;

	hash_set_last_block(1);
	hash_set_dma_output_len(hash_get_digest_word_len(ctx->hash_alg) << 2);

	if (hash_total_byte_len_add_uint32(ctx->total, ctx->block_word_len/8, ctx->remainder_bytes))
		return HASH_LEN_OVERFLOW;

	hash_total_bytelen_2_bitlen(ctx->total, (ctx->block_word_len / 8));
	hash_set_msg_total_bit_len(ctx->total, (ctx->block_word_len * 4));

	hash_dma_operate((uint64_t)(ctx->dma_addr.phys_in - CMA_ADDR_OFFSET + 0x4 * ctx->block_words), (uint64_t)(ctx->dma_addr.phys_out - CMA_ADDR_OFFSET),
					ctx->remainder_bytes, ctx->callback);

	return HASH_SUCCESS;
}

static int bst_sm3_init(struct shash_desc *desc)
{
	struct bst_hash_ctx *ctx = shash_desc_ctx(desc);

	#if USE_REQUEST_CTX == 1
	memset(ctx, 0, sizeof(struct bst_hash_ctx));
	#else
	memset(ctx, 0, offsetof(struct  bst_hash_ctx, key_lock));
	#endif
	ctx->base = bst_hash_get_dev()->io_base;
	ctx->hfe_mode = HASH_MODE;
	ctx->hash_alg = HASH_SM3;
	
	return hash_init(ctx);
}

// static int bst_sm3_dma_init(struct shash_desc *desc)
// {
// 	struct bst_hash_dma_ctx *ctx = shash_desc_ctx(desc);

// 	#if USE_REQUEST_CTX == 1
// 	memset(ctx, 0, sizeof(struct bst_hash_dma_ctx));
// 	#else
// 	memset(ctx, 0, offsetof(struct  bst_hash_dma_ctx, key_lock));
// 	#endif
// 	ctx->base = bst_hash_get_dev()->io_base;
// 	ctx->hfe_mode = HASH_MODE;
// 	ctx->hash_alg = HASH_SM3;

// 	return hash_dma_init(ctx, hash_dma_callback);
// }
static int bst_asm3_dma_init(struct ahash_request *req)
{
	struct bst_hash_dma_ctx *ctx = crypto_tfm_ctx(req->base.tfm);

	#if USE_REQUEST_CTX == 1
	memset(ctx, 0, sizeof(struct bst_hash_dma_ctx));
	#else
	memset(ctx, 0, offsetof(struct  bst_hash_dma_ctx, key_lock));
	#endif
	ctx->base = bst_hash_get_dev()->io_base;
	ctx->hfe_mode = HASH_MODE;
	ctx->hash_alg = HASH_SM3;

	return hash_dma_init(ctx, hash_dma_callback);
}

static int bst_md5_init(struct shash_desc *desc)
{
	struct bst_hash_ctx *ctx = shash_desc_ctx(desc);

	#if USE_REQUEST_CTX == 1
	memset(ctx, 0, sizeof(struct bst_hash_ctx));
	#else
	memset(ctx, 0, offsetof(struct  bst_hash_ctx, key_lock));
	#endif
	ctx->base = bst_hash_get_dev()->io_base;
	ctx->hfe_mode = HASH_MODE;
	ctx->hash_alg = HASH_MD5;

	return hash_init(ctx);
}

// static int bst_md5_dma_init(struct shash_desc *desc)
// {
// 	struct bst_hash_dma_ctx *ctx = shash_desc_ctx(desc);

// 	#if USE_REQUEST_CTX == 1
// 	memset(ctx, 0, sizeof(struct bst_hash_dma_ctx));
// 	#else
// 	memset(ctx, 0, offsetof(struct  bst_hash_dma_ctx, key_lock));
// 	#endif
// 	ctx->base = bst_hash_get_dev()->io_base;
// 	ctx->hfe_mode = HASH_MODE;
// 	ctx->hash_alg = HASH_MD5;

// 	return hash_dma_init(ctx, hash_dma_callback);
// }
static int bst_amd5_dma_init(struct ahash_request *req)
{
	struct bst_hash_dma_ctx *ctx = crypto_tfm_ctx(req->base.tfm);

	#if USE_REQUEST_CTX == 1
	memset(ctx, 0, sizeof(struct bst_hash_dma_ctx));
	#else
	memset(ctx, 0, offsetof(struct  bst_hash_dma_ctx, key_lock));
	#endif
	ctx->base = bst_hash_get_dev()->io_base;
	ctx->hfe_mode = HASH_MODE;
	ctx->hash_alg = HASH_MD5;

	return hash_dma_init(ctx, hash_dma_callback);
}

static int bst_sha256_init(struct shash_desc *desc)
{
	struct bst_hash_ctx *ctx = shash_desc_ctx(desc);

	#if USE_REQUEST_CTX == 1
	memset(ctx, 0, sizeof(struct bst_hash_ctx));
	#else
	memset(ctx, 0, offsetof(struct  bst_hash_ctx, key_lock));
	#endif
	ctx->base = bst_hash_get_dev()->io_base;
	ctx->hfe_mode = HASH_MODE;
	ctx->hash_alg = HASH_SHA256;

	return hash_init(ctx);
}

static int bst_asha256_dma_init(struct ahash_request *req)
{
	struct bst_hash_dma_ctx *ctx = crypto_tfm_ctx(req->base.tfm);

	#if USE_REQUEST_CTX == 1
	memset(ctx, 0, sizeof(struct bst_hash_dma_ctx));
	#else
	memset(ctx, 0, offsetof(struct  bst_hash_dma_ctx, key_lock));
	#endif
	ctx->base = bst_hash_get_dev()->io_base;
	ctx->hfe_mode = HASH_MODE;
	ctx->hash_alg = HASH_SHA256;

	return hash_dma_init(ctx, hash_dma_callback);
}

static int bst_asha512_dma_init(struct ahash_request *req)
{
	struct bst_hash_dma_ctx *ctx = crypto_tfm_ctx(req->base.tfm);

	#if USE_REQUEST_CTX == 1
	memset(ctx, 0, sizeof(struct bst_hash_dma_ctx));
	#else
	memset(ctx, 0, offsetof(struct  bst_hash_dma_ctx, key_lock));
	#endif
	ctx->base = bst_hash_get_dev()->io_base;
	ctx->hfe_mode = HASH_MODE;
	ctx->hash_alg = HASH_SHA512;

	return hash_dma_init(ctx, hash_dma_callback);
}

static int bst_sha1_init(struct shash_desc *desc)
{
	struct bst_hash_ctx *ctx = shash_desc_ctx(desc);

	#if USE_REQUEST_CTX == 1
	memset(ctx, 0, sizeof(struct bst_hash_ctx));
	#else
	memset(ctx, 0, offsetof(struct  bst_hash_ctx, key_lock));
	#endif
	ctx->base = bst_hash_get_dev()->io_base;
	ctx->hfe_mode = HASH_MODE;
	ctx->hash_alg = HASH_SHA1;

	return hash_init(ctx);
}

// static int bst_sha1_dma_init(struct shash_desc *desc)
// {
// 	struct bst_hash_dma_ctx *ctx = shash_desc_ctx(desc);

// 	#if USE_REQUEST_CTX == 1
// 	memset(ctx, 0, sizeof(struct bst_hash_dma_ctx));
// 	#else
// 	memset(ctx, 0, offsetof(struct  bst_hash_dma_ctx, key_lock));
// 	#endif
// 	ctx->base = bst_hash_get_dev()->io_base;
// 	ctx->hfe_mode = HASH_MODE;
// 	ctx->hash_alg = HASH_SHA1;

// 	return hash_dma_init(ctx, hash_dma_callback);
// }
static int bst_asha1_dma_init(struct ahash_request *req)
{
	struct bst_hash_dma_ctx *ctx = crypto_tfm_ctx(req->base.tfm);

	#if USE_REQUEST_CTX == 1
	memset(ctx, 0, sizeof(struct bst_hash_dma_ctx));
	#else
	memset(ctx, 0, offsetof(struct  bst_hash_dma_ctx, key_lock));
	#endif
	ctx->base = bst_hash_get_dev()->io_base;
	ctx->hfe_mode = HASH_MODE;
	ctx->hash_alg = HASH_SHA1;

	return hash_dma_init(ctx, hash_dma_callback);
}

static int bst_sha224_init(struct shash_desc *desc)
{
	struct bst_hash_ctx *ctx = shash_desc_ctx(desc);

	#if USE_REQUEST_CTX == 1
	memset(ctx, 0, sizeof(struct bst_hash_ctx));
	#else
	memset(ctx, 0, offsetof(struct  bst_hash_ctx, key_lock));
	#endif
	ctx->base = bst_hash_get_dev()->io_base;
	ctx->hfe_mode = HASH_MODE;
	ctx->hash_alg = HASH_SHA224;

	return hash_init(ctx);
}

// static int bst_sha224_dma_init(struct shash_desc *desc)
// {
// 	struct bst_hash_dma_ctx *ctx = shash_desc_ctx(desc);

// 	#if USE_REQUEST_CTX == 1
// 	memset(ctx, 0, sizeof(struct bst_hash_dma_ctx));
// 	#else
// 	memset(ctx, 0, offsetof(struct  bst_hash_dma_ctx, key_lock));
// 	#endif
// 	ctx->base = bst_hash_get_dev()->io_base;
// 	ctx->hfe_mode = HASH_MODE;
// 	ctx->hash_alg = HASH_SHA224;

// 	return hash_dma_init(ctx, hash_dma_callback);
// }
static int bst_asha224_dma_init(struct ahash_request *req)
{
	struct bst_hash_dma_ctx *ctx = crypto_tfm_ctx(req->base.tfm);

	#if USE_REQUEST_CTX == 1
	memset(ctx, 0, sizeof(struct bst_hash_dma_ctx));
	#else
	memset(ctx, 0, offsetof(struct  bst_hash_dma_ctx, key_lock));
	#endif
	ctx->base = bst_hash_get_dev()->io_base;
	ctx->hfe_mode = HASH_MODE;
	ctx->hash_alg = HASH_SHA224;

	return hash_dma_init(ctx, hash_dma_callback);
}

static int bst_sha512_init(struct shash_desc *desc)
{
	struct bst_hash_ctx *ctx = shash_desc_ctx(desc);

	#if USE_REQUEST_CTX == 1
	memset(ctx, 0, sizeof(struct bst_hash_ctx));
	#else
	memset(ctx, 0, offsetof(struct  bst_hash_ctx, key_lock));
	#endif
	ctx->base = bst_hash_get_dev()->io_base;
	ctx->hfe_mode = HASH_MODE;
	ctx->hash_alg = HASH_SHA512;

	return hash_init(ctx);
}

// static int bst_sha512_dma_init(struct shash_desc *desc)
// {
// 	struct bst_hash_dma_ctx *ctx = shash_desc_ctx(desc);

// 	#if USE_REQUEST_CTX == 1
// 	memset(ctx, 0, sizeof(struct bst_hash_dma_ctx));
// 	#else
// 	memset(ctx, 0, offsetof(struct  bst_hash_dma_ctx, key_lock));
// 	#endif
// 	ctx->base = bst_hash_get_dev()->io_base;
// 	ctx->hfe_mode = HASH_MODE;
// 	ctx->hash_alg = HASH_SHA512;

// 	return hash_dma_init(ctx, hash_dma_callback);
// }

static int bst_sha512_224_init(struct shash_desc *desc)
{
	struct bst_hash_ctx *ctx = shash_desc_ctx(desc);

	#if USE_REQUEST_CTX == 1
	memset(ctx, 0, sizeof(struct bst_hash_ctx));
	#else
	memset(ctx, 0, offsetof(struct  bst_hash_ctx, key_lock));
	#endif
	ctx->base = bst_hash_get_dev()->io_base;
	ctx->hfe_mode = HASH_MODE;
	ctx->hash_alg = HASH_SHA512_224;

	return hash_init(ctx);
}

// static int bst_sha512_224_dma_init(struct shash_desc *desc)
// {
// 	struct bst_hash_dma_ctx *ctx = shash_desc_ctx(desc);

// 	#if USE_REQUEST_CTX == 1
// 	memset(ctx, 0, sizeof(struct bst_hash_dma_ctx));
// 	#else
// 	memset(ctx, 0, offsetof(struct  bst_hash_dma_ctx, key_lock));
// 	#endif
// 	ctx->base = bst_hash_get_dev()->io_base;
// 	ctx->hfe_mode = HASH_MODE;
// 	ctx->hash_alg = HASH_SHA512_224;

// 	return hash_dma_init(ctx, hash_dma_callback);
// }

static int bst_sha512_256_init(struct shash_desc *desc)
{
	struct bst_hash_ctx *ctx = shash_desc_ctx(desc);

	#if USE_REQUEST_CTX == 1
	memset(ctx, 0, sizeof(struct bst_hash_ctx));
	#else
	memset(ctx, 0, offsetof(struct  bst_hash_ctx, key_lock));
	#endif
	ctx->base = bst_hash_get_dev()->io_base;
	ctx->hfe_mode = HASH_MODE;
	ctx->hash_alg = HASH_SHA512_256;

	return hash_init(ctx);
}

// static int bst_sha512_256_dma_init(struct shash_desc *desc)
// {
// 	struct bst_hash_dma_ctx *ctx = shash_desc_ctx(desc);

// 	#if USE_REQUEST_CTX == 1
// 	memset(ctx, 0, sizeof(struct bst_hash_dma_ctx));
// 	#else
// 	memset(ctx, 0, offsetof(struct  bst_hash_dma_ctx, key_lock));
// 	#endif
// 	ctx->base = bst_hash_get_dev()->io_base;
// 	ctx->hfe_mode = HASH_MODE;
// 	ctx->hash_alg = HASH_SHA512_256;

// 	return hash_dma_init(ctx, hash_dma_callback);
// }

static int bst_hash_update(struct shash_desc *desc, const uint8_t *data,
						   unsigned int len)
{
	struct bst_hash_ctx *ctx = shash_desc_ctx(desc);

	return hash_update(ctx, data, len);
}

static int bst_hash_final(struct shash_desc *desc, uint8_t *out)
{
	struct bst_hash_ctx *ctx = shash_desc_ctx(desc);

	return hash_final(ctx, out);
}

static int bst_hash_finup(struct shash_desc *desc, const uint8_t *data,
	unsigned int length, uint8_t *out)
{
	return bst_hash_update(desc, data, length) ?: bst_hash_final(desc, out);
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

static void dma_readl(uint8_t *data, uint8_t *addr, unsigned int len8)
{
	unsigned int i;
	for (i = 0; i < len8; i++) {
		data[i] = read_reg(addr + i);
	}
}

static void dma_writel(uint8_t *addr, const uint8_t *data, unsigned int len8)
{
	unsigned int i;
	for (i = 0; i < len8; i++) {
		write_reg(data[i], addr + i);
	}
}

static int bst_ahash_dma_update(struct ahash_request *req)
{
	int ret, err;
	struct bst_hash_dma_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	uint8_t *data = (uint8_t *)kmalloc(req->nbytes, GFP_KERNEL);
	if (data == NULL)
		return HASH_BUFFER_NULL;
	err = sg_pcopy_to_buffer(req->src, sg_nents_for_len(req->src, req->nbytes), data, req->nbytes, 0);
	if (err != req->nbytes) {
		pr_info("input data copy error: %d", err);
		ret =  HASH_BUFFER_NULL;
		goto end;
	}

	ctx->msg_bytes = req->nbytes;
	ctx->remainder_bytes = ctx->msg_bytes % (ctx->block_word_len << 2);
	ctx->block_words = (ctx->msg_bytes - ctx->remainder_bytes) / 4;
	ctx->remainder_msg = (uint32_t *)data + ctx->block_words;

	if(ctx->dma_addr.virt_in != NULL){
		dmam_free_coherent(global_hash->dev, ctx->dma_addr.alloc_size[0], ctx->dma_addr.virt_in, ctx->dma_addr.phys_in);
		ctx->dma_addr.virt_in = NULL;
	}
	ctx->dma_addr.alloc_size[0] = max((uint32_t)(ctx->msg_bytes), (uint32_t)(2 * PAGE_SIZE));
	ctx->dma_addr.virt_in = dmam_alloc_coherent(global_hash->dev, ctx->dma_addr.alloc_size[0], &ctx->dma_addr.phys_in, GFP_ATOMIC);
	if (ctx->dma_addr.virt_in == NULL) {
		pr_info("input dma alloc failed, allocate_szie: %d", ctx->dma_addr.alloc_size[0]);
		ret = HASH_BUFFER_NULL;
		goto end;
	}
	// printk("virt in addr: %p, phys in addr: 0x%llx", ctx->dma_addr.virt_in, ctx->dma_addr.phys_in);

	dma_writel(ctx->dma_addr.virt_in, data, ctx->msg_bytes);
	ret = hash_dma_update_blocks(ctx, (const uint32_t *)data, ctx->block_words);
	if (ret != HASH_SUCCESS)
		pr_info("hash_dma_update_blocks failed");

	bst_dbg(2, "ahash dma Update:ctx:%p ctx->msg_bytes=%u, block_words=%u, remainder_bytes=%u\n",ctx, ctx->msg_bytes, ctx->block_words, ctx->remainder_bytes);
	printHex("ahash dma update in", data, ctx->msg_bytes);

end:

	bst_kfree(data);
	return ret;
}

static int bst_ahash_dma_final(struct ahash_request *req)
{
	int ret;
	struct bst_hash_dma_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	uint8_t *out = (uint8_t *)req->result;
	if (out == NULL) {
		pr_info("output buffer is NULL");
		ret = HASH_BUFFER_NULL;
		goto free_input_dma;
	}

	ctx->dma_addr.alloc_size[1] = max((uint32_t)(ctx->digest_byte_len), (uint32_t)(2 * PAGE_SIZE));
	ctx->dma_addr.virt_out = dmam_alloc_coherent(global_hash->dev, ctx->dma_addr.alloc_size[1], &ctx->dma_addr.phys_out, GFP_ATOMIC);
	if (ctx->dma_addr.virt_out == NULL) {
		pr_info("input dma alloc failed, allocate_szie: %d", ctx->dma_addr.alloc_size[1]);
		ret = HASH_BUFFER_NULL;
		goto free_input_dma;
	}
	// printk("virt out addr: %p, phys out addr: 0x%llx", ctx->dma_addr.virt_out, ctx->dma_addr.phys_out);
	
	ret = hash_dma_final(ctx);
	if (ret == HASH_SUCCESS)
		dma_readl(out, ctx->dma_addr.virt_out, ctx->digest_byte_len);

	dmam_free_coherent(global_hash->dev, ctx->dma_addr.alloc_size[1], ctx->dma_addr.virt_out, ctx->dma_addr.phys_out);
	ctx->dma_addr.virt_out = NULL;
free_input_dma:
	if(ctx->dma_addr.virt_in != NULL){
		dmam_free_coherent(global_hash->dev, ctx->dma_addr.alloc_size[0], ctx->dma_addr.virt_in, ctx->dma_addr.phys_in);
		ctx->dma_addr.virt_in = NULL;
	}


	bst_dbg(2, "dma final:ctx:%p \n",ctx);
	printHex("ahash dma final out", out, ctx->digest_byte_len);

	#if USE_REQUEST_CTX == 1
	memset(ctx, 0, sizeof(struct bst_hash_dma_ctx));
	#else
	memset(ctx, 0, offsetof(struct  bst_hash_dma_ctx, key_lock));
	#endif

	return ret;
}

/*optional  finalize hashing operation after an update */
static int bst_ahash_dma_finup(struct ahash_request *req)
{
	return bst_ahash_dma_update(req) ?: bst_ahash_dma_final(req);
}


// static int bst_hash_dma_export(struct shash_desc *desc, void *out)
// {
// 	struct bst_hash_dma_ctx *ctx = shash_desc_ctx(desc);
// 	memcpy(out, ctx, sizeof(struct bst_hash_dma_ctx));

// 	return 0;
// }

// static int bst_hash_dma_import(struct shash_desc *desc, const void *in)
// {
// 	struct bst_hash_dma_ctx *ctx = shash_desc_ctx(desc);
// 	memcpy(ctx, in, sizeof(struct bst_hash_dma_ctx));

// 	return 0;
// }

static int bst_ahash_dma_export(struct ahash_request *req, void *out)
{
	struct bst_hash_dma_ctx *ctx = crypto_tfm_ctx(req->base.tfm);

	memcpy(out, ctx, sizeof(struct bst_hash_dma_ctx));

	return 0;
}

static int bst_ahash_dma_import(struct ahash_request *req, const void *in)
{
	struct bst_hash_dma_ctx *ctx = crypto_tfm_ctx(req->base.tfm);

	memcpy(ctx, in, sizeof(struct bst_hash_dma_ctx));

	return 0;
}

static int bst_hmac_init(struct shash_desc *desc, enum BST_HASH_ALG alg)
{
#if GETKEY_USE_REQUEST_CTX == 1
	struct bst_hash_ctx *tctx = shash_desc_ctx(desc);
#else
    struct bst_hash_ctx *tctx = crypto_shash_ctx(desc->tfm);
#endif
	struct bst_hash_ctx *ctx = shash_desc_ctx(desc);
	// struct bst_hash_tfm_ctx *mctx = crypto_shash_ctx(desc->tfm);

	#if USE_REQUEST_CTX == 1
	memset(ctx, 0, sizeof(struct bst_hash_ctx));
	#else
	memset(ctx, 0, offsetof(struct  bst_hash_ctx, key_lock));
	#endif
	ctx->base = bst_hash_get_dev()->io_base;
	ctx->hfe_mode = HMAC_MODE;
	ctx->hash_alg = alg;
	ctx->key_len = tctx->keySrc_len;
	bst_dbg(2,"use key get tfm ctx:%p, key len:%d,\n",tctx,tctx->keySrc_len);

	hash_set_cpu_mode();
	hash_hmac_disable_secure_port();
	hash_set_hmac_mode();
	hash_set_hmac_key_mode();
	hash_disable_interruption();

	hash_set_endian_uint32(0);
	hash_set_alg(ctx->hash_alg);
	hash_update_config();
	ctx->block_byte_len = hash_get_block_word_len(ctx->hash_alg) << 2;
	ctx->iterator_word_len = hash_get_iterator_word_len(ctx->hash_alg);
	ctx->digest_byte_len = hash_get_digest_word_len(ctx->hash_alg) << 2;
	ctx->status.busy = 0;
	ctx->first_update_flag = 1;
	ctx->finish_flag = 0;
	hash_hmac_set_key(ctx, tctx->keySrc);
	hash_clear_hmac_key_mode();
	hash_enable_cpu_interruption();
	hash_set_last_block(0);

	// kfree(hmac_key);
	// hmac_key_len = 0;
	return HASH_SUCCESS;
}

static int bst_ahmac_dma_init(struct ahash_request *req, enum BST_HASH_ALG alg)
{
#if GETKEY_USE_REQUEST_CTX == 1
	struct bst_hash_dma_ctx *tctx = crypto_tfm_ctx(req->base.tfm);
#else
    struct bst_hash_dma_ctx *tctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
#endif
	struct bst_hash_dma_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	// struct bst_hash_tfm_ctx *mctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req)); 

	#if USE_REQUEST_CTX == 1
	memset(ctx, 0, sizeof(struct bst_hash_dma_ctx));
	#else
	memset(ctx, 0, offsetof(struct  bst_hash_dma_ctx, key_lock));
	#endif
	ctx->base = bst_hash_get_dev()->io_base;
	ctx->hfe_mode = HMAC_MODE;
	ctx->hash_alg = alg;
	ctx->key_len = tctx->keySrc_len;
	bst_dbg(2,"use key get tfm ctx:%p, key len:%d,\n",tctx,tctx->keySrc_len);

	hash_set_cpu_mode();
	hash_hmac_disable_secure_port();
	hash_set_hmac_mode();
	hash_set_hmac_key_mode();
	hash_disable_interruption();
	hash_set_endian_uint32(0);
	hash_set_alg(ctx->hash_alg);
	hash_update_config();
	ctx->block_byte_len = hash_get_block_word_len(ctx->hash_alg) << 2;
	ctx->iterator_word_len = hash_get_iterator_word_len(ctx->hash_alg);
	ctx->digest_byte_len = hash_get_digest_word_len(ctx->hash_alg) << 2;
	ctx->status.busy = 0;
	ctx->first_update_flag = 1;
	ctx->finish_flag = 0;
	hash_hmac_dma_set_key(ctx, tctx->keySrc);
	hash_clear_hmac_key_mode();
	hash_enable_cpu_interruption();

	hash_set_dma_mode();
	hash_set_last_block(0);
	hash_set_dma_output_len(0);
	ctx->block_word_len = hash_get_block_word_len(ctx->hash_alg);
	ctx->callback = hash_dma_callback;
	uint32_clear(ctx->total, sizeof(ctx->total)/4);
	
	// kfree(hmac_key);
	// hmac_key_len = 0;
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

static int bst_ahmac_sha256_dma_init(struct ahash_request *req)
{
	return bst_ahmac_dma_init(req, HASH_SHA256);
}

static int bst_ahmac_sha224_dma_init(struct ahash_request *req)
{
	return bst_ahmac_dma_init(req, HASH_SHA224);
}

static int bst_ahmac_sha512_dma_init(struct ahash_request *req)
{
	return bst_ahmac_dma_init(req, HASH_SHA512);
}

// static int bst_hmac_sha512_224_dma_init(struct shash_desc *desc)
// {
// 	return bst_hmac_dma_init(desc, HASH_SHA512_224);
// }

// static int bst_hmac_sha512_256_dma_init(struct shash_desc *desc)
// {
// 	return bst_hmac_dma_init(desc, HASH_SHA512_256);
// }

static int bst_ahmac_sha1_dma_init(struct ahash_request *req)
{
	return bst_ahmac_dma_init(req, HASH_SHA1);
}

static int bst_ahmac_md5_dma_init(struct ahash_request *req)
{
	return bst_ahmac_dma_init(req, HASH_MD5);
}

static int bst_ahmac_sm3_dma_init(struct ahash_request *req)
{
	return bst_ahmac_dma_init(req, HASH_SM3);
}

static int bst_hmac_setkey(struct crypto_shash *tfm, const uint8_t *key,
						   unsigned int keylen)
{
	//can not get request ctx from struct crypto_shash *tfm
	struct bst_hash_ctx *mctx = crypto_shash_ctx(tfm);
	mutex_lock(&mctx->key_lock);
	if (mctx->keySrc == NULL) {
		mctx->keySrc = kmalloc(sizeof(uint32_t) * HASH_BLOCK_MAX_WORD_LEN, GFP_KERNEL);
	}
	// mctx->key = kmalloc(keylen, GFP_KERNEL);
	// if (mctx->key == NULL)
	// 	return -ENOMEM;

	memcpy(mctx->keySrc, key, keylen);
	mctx->keySrc_len = keylen;
	mutex_unlock(&mctx->key_lock);
	// hmac_key = kmalloc(keylen, GFP_KERNEL);
	// if (hmac_key == NULL)
	// 	return -ENOMEM;

	// memcpy(hmac_key, key, keylen);
	// hmac_key_len = keylen;
	bst_dbg(2, "hmac setkey tfm:%p\n",mctx);
	printHex("key",mctx->keySrc, mctx->keySrc_len);
	
	return 0;
}
static int bst_ahmac_setkey(struct crypto_ahash *tfm, const u8 *key,
		      				unsigned int keylen)
{
	//can not get request ctx from struct crypto_ahash *tfm
	struct bst_hash_dma_ctx *mctx = crypto_ahash_ctx(tfm);
	mutex_lock(&mctx->key_lock);
	if (mctx->keySrc == NULL) {
		mctx->keySrc = kmalloc(sizeof(uint32_t) * HASH_BLOCK_MAX_WORD_LEN, GFP_KERNEL);
	}
	// int i;
	// mctx->key = kmalloc(keylen, GFP_KERNEL);
	// if (mctx->key == NULL)
	// 	return -ENOMEM;

	// pr_info("%s crypto_ahash: %p, bst_hash_tfm_ctx: %p", __func__, tfm, mctx);
	memcpy(mctx->keySrc, key, keylen);
	mctx->keySrc_len = keylen;
	mutex_unlock(&mctx->key_lock);
	// pr_info("mctx->key addr: %p", mctx->key);
	// for (i=0;i<mctx->key_len/4;i++)
	// 	pr_info("key[%d]: 0x%08x", i, *((uint32_t *)mctx->key+i));
	// hmac_key = kmalloc(keylen, GFP_KERNEL);
	// if (hmac_key == NULL)
	// 	return -ENOMEM;

	// memcpy(hmac_key, key, keylen);
	// hmac_key_len = keylen;
	bst_dbg(2, "ahmac setkey mctx:%p\n",mctx);
	printHex("key",mctx->keySrc, mctx->keySrc_len);
	return 0;
}
void bst_hfe_work_func(struct work_struct *work)
{
	// bst_dbg(2, "%s()\n", __func__);

	// mdelay(1000);
	// queue_work(workqueue_test, &work_test);
}

static int bst_amd5_dma_digest(struct ahash_request *req)
{
	return bst_amd5_dma_init(req) ?: bst_ahash_dma_finup(req);
}

static int bst_asm3_dma_digest(struct ahash_request *req)
{
	return bst_asm3_dma_init(req) ?: bst_ahash_dma_finup(req);
}

static int bst_sha256_digest(struct shash_desc *desc, const uint8_t *data,unsigned int len, uint8_t *out)
{
	return bst_sha256_init(desc) ?: bst_hash_finup(desc, data, len, out);
}

static int bst_sm3_digest(struct shash_desc *desc, const uint8_t *data,unsigned int len, uint8_t *out)
{
	return bst_sm3_init(desc) ?: bst_hash_finup(desc, data, len, out);
}

static int bst_md5_digest(struct shash_desc *desc, const uint8_t *data,unsigned int len, uint8_t *out)
{
	return bst_md5_init(desc) ?: bst_hash_finup(desc, data, len, out);
}

static int bst_asha256_dma_digest(struct ahash_request *req)
{
    return bst_asha256_dma_init(req) ?: bst_ahash_dma_finup(req);
}

static int bst_asha512_dma_digest(struct ahash_request *req)
{
    return bst_asha512_dma_init(req) ?: bst_ahash_dma_finup(req);
}

static int bst_sha1_digest(struct shash_desc *desc, const uint8_t *data,unsigned int len, uint8_t *out)
{
    return bst_sha1_init(desc) ?: bst_hash_finup(desc, data, len, out);
}


static int bst_asha1_dma_digest(struct ahash_request *req)
{
    return bst_asha1_dma_init(req) ?: bst_ahash_dma_finup(req);
}

static int bst_sha224_digest(struct shash_desc *desc, const uint8_t *data,unsigned int len, uint8_t *out)
{
    return bst_sha224_init(desc) ?: bst_hash_finup(desc, data, len, out);
}


static int bst_asha224_dma_digest(struct ahash_request *req)
{
    return bst_asha224_dma_init(req) ?: bst_ahash_dma_finup(req);
}

static int bst_sha512_digest(struct shash_desc *desc, const uint8_t *data,unsigned int len, uint8_t *out)
{
    return bst_sha512_init(desc) ?: bst_hash_finup(desc, data, len, out);
}


static int bst_sha512_224_digest(struct shash_desc *desc, const uint8_t *data,unsigned int len, uint8_t *out)
{
    return bst_sha512_224_init(desc) ?: bst_hash_finup(desc, data, len, out);
}

static int bst_sha512_256_digest(struct shash_desc *desc, const uint8_t *data,unsigned int len, uint8_t *out)
{
    return bst_sha512_256_init(desc) ?: bst_hash_finup(desc, data, len, out);
}


static int bst_hmac_sha256_digest(struct shash_desc *desc, const uint8_t *data,unsigned int len, uint8_t *out)
{
    return bst_hmac_sha256_init(desc) ?: bst_hash_finup(desc, data, len, out);
}

static int bst_hmac_sha224_digest(struct shash_desc *desc, const uint8_t *data,unsigned int len, uint8_t *out)
{
    return bst_hmac_sha224_init(desc) ?: bst_hash_finup(desc, data, len, out);
}

static int bst_hmac_sha512_digest(struct shash_desc *desc, const uint8_t *data,unsigned int len, uint8_t *out)
{
    return bst_hmac_sha512_init(desc) ?: bst_hash_finup(desc, data, len, out);
}

static int bst_hmac_sha512_224_digest(struct shash_desc *desc, const uint8_t *data,unsigned int len, uint8_t *out)
{
    return bst_hmac_sha512_224_init(desc) ?: bst_hash_finup(desc, data, len, out);
}

static int bst_hmac_sha512_256_digest(struct shash_desc *desc, const uint8_t *data,unsigned int len, uint8_t *out)
{
    return bst_hmac_sha512_256_init(desc) ?: bst_hash_finup(desc, data, len, out);
}

static int bst_hmac_sha1_digest(struct shash_desc *desc, const uint8_t *data,unsigned int len, uint8_t *out)
{
    return bst_hmac_sha1_init(desc) ?: bst_hash_finup(desc, data, len, out);
}

static int bst_hmac_md5_digest(struct shash_desc *desc, const uint8_t *data,unsigned int len, uint8_t *out)
{
    return bst_hmac_md5_init(desc) ?: bst_hash_finup(desc, data, len, out);
}

static int bst_hmac_sm3_digest(struct shash_desc *desc, const uint8_t *data,unsigned int len, uint8_t *out)
{
    return bst_hmac_sm3_init(desc) ?: bst_hash_finup(desc, data, len, out);
}

static int bst_ahmac_sha256_dma_digest(struct ahash_request *req)
{
    return bst_ahmac_sha256_dma_init(req) ?: bst_ahash_dma_finup(req);
}

static int bst_ahmac_sha224_dma_digest(struct ahash_request *req)
{
    return bst_ahmac_sha224_dma_init(req) ?: bst_ahash_dma_finup(req);
}

static int bst_ahmac_sha512_dma_digest(struct ahash_request *req)
{
    return bst_ahmac_sha512_dma_init(req) ?: bst_ahash_dma_finup(req);
}

static int bst_ahmac_sha1_dma_digest(struct ahash_request *req)
{
    return bst_ahmac_sha1_dma_init(req) ?: bst_ahash_dma_finup(req);
}

static int bst_ahmac_md5_dma_digest(struct ahash_request *req)
{
    return bst_ahmac_md5_dma_init(req) ?: bst_ahash_dma_finup(req);
}

static int bst_ahmac_sm3_dma_digest(struct ahash_request *req)
{
    return bst_ahmac_sm3_dma_init(req) ?: bst_ahash_dma_finup(req);
}

// static int bst_ahash_init_tfm(struct crypto_tfm *tfm)
// {
//     // struct bst_hash_dma_ctx *ctx = crypto_tfm_ctx(tfm);
    
//     // memset(ctx, 0, sizeof(struct bst_hash_dma_ctx));
    
//     // mutex_init(&ctx->key_lock);
//     // mutex_lock(&ctx->key_lock);
    
//     // ctx->keySrc = kmalloc(sizeof(uint32_t) * HASH_BLOCK_MAX_WORD_LEN, GFP_KERNEL);
//     // if (ctx->keySrc == NULL) {
//     //     mutex_unlock(&ctx->key_lock);
//     //     pr_err("Failed to allocate key buffer\n");
//     //     return -ENOMEM;
//     // }
    
//     // ctx->keySrc_len = 0;
//     // ctx->key_len_flag = 0; 
    
//     // mutex_unlock(&ctx->key_lock);
    
//     // bst_dbg(2, "bst_ahash_init_tfm: ctx=%p\n", ctx);
//     return 0;
// }

static void bst_ahash_exit_tfm(struct crypto_tfm *tfm)
{
    struct bst_hash_dma_ctx *ctx = crypto_tfm_ctx(tfm);
    mutex_lock(&ctx->key_lock);

    bst_kfree(ctx->keySrc);
    ctx->keySrc_len = 0;
    mutex_unlock(&ctx->key_lock);
    bst_dbg(2, "bst_ahash_exit_tfm: ctx=%p\n", ctx);
}

// static int bst_shash_init_tfm(struct crypto_tfm *tfm)
// {
//     // struct bst_hash_ctx *ctx = crypto_tfm_ctx(tfm);
    
//     // memset(ctx, 0, sizeof(struct bst_hash_ctx));
    
//     // mutex_init(&ctx->key_lock);
//     // mutex_lock(&ctx->key_lock);
    
//     // ctx->keySrc = kmalloc(sizeof(uint32_t) * HASH_BLOCK_MAX_WORD_LEN, GFP_KERNEL);
//     // if (ctx->keySrc == NULL) {
//     //     mutex_unlock(&ctx->key_lock);
//     //     pr_err("Failed to allocate key buffer\n");
//     //     return -ENOMEM;
//     // }
    
//     // ctx->keySrc_len = 0;
//     // ctx->key_len_flag = 0;  
    
//     // mutex_unlock(&ctx->key_lock);
    
//     // bst_dbg(2, "bst_shash_init_tfm : ctx=%p\n", ctx);
//     return 0;
// }

static void bst_shash_exit_tfm(struct crypto_tfm *tfm)
{
    struct bst_hash_ctx *ctx = crypto_tfm_ctx(tfm);
    mutex_lock(&ctx->key_lock);
    bst_kfree(ctx->keySrc);
    ctx->keySrc_len = 0;
    mutex_unlock(&ctx->key_lock);
    bst_dbg(2, "bst_shash_exit_tfm : ctx=%p\n", ctx);
}

static struct ahash_alg bst_ahash_algs[] = {
	{.init = bst_asha256_dma_init,
	.update = bst_ahash_dma_update,
	.final = bst_ahash_dma_final,
	.finup = bst_ahash_dma_finup,
	.digest = bst_asha256_dma_digest,
	.export = bst_ahash_dma_export,
	.import = bst_ahash_dma_import, 
	.halg = {
		.digestsize = SHA256_DIGEST_SIZE,
		.statesize = sizeof(struct bst_hash_dma_ctx),
		.base = {.cra_name = "sha256",
				 .cra_priority = 400,
				 .cra_driver_name = "sha256-bst",
				 .cra_ctxsize = sizeof(struct bst_hash_dma_ctx),
				 .cra_exit = bst_ahash_exit_tfm,
				 .cra_blocksize = SHA256_BLOCK_SIZE,
				 .cra_module = THIS_MODULE, },
		},
	},
	{.init = bst_asha512_dma_init,
	.update = bst_ahash_dma_update,
	.final = bst_ahash_dma_final,
	.finup = bst_ahash_dma_finup,
	.digest = bst_asha512_dma_digest,
	.export = bst_ahash_dma_export,
	.import = bst_ahash_dma_import, 
	.halg = {
		.digestsize = SHA512_DIGEST_SIZE,
		.statesize = sizeof(struct bst_hash_dma_ctx),
		.base = {.cra_name = "sha512",
				 .cra_priority = 400,
				 .cra_driver_name = "sha512-bst",
				 .cra_ctxsize = sizeof(struct bst_hash_dma_ctx),
				 .cra_exit = bst_ahash_exit_tfm,
				 .cra_blocksize = SHA512_BLOCK_SIZE,
				 .cra_module = THIS_MODULE, },
		},
	},
	{.init = bst_asha1_dma_init,
	.update = bst_ahash_dma_update,
	.final = bst_ahash_dma_final,
	.finup = bst_ahash_dma_finup,
	.digest = bst_asha1_dma_digest,
	.export = bst_ahash_dma_export,
	.import = bst_ahash_dma_import, 
	.halg = {
		.digestsize = SHA1_DIGEST_SIZE,
		.statesize = sizeof(struct bst_hash_dma_ctx),
		.base = {.cra_name = "sha1",
				 .cra_priority = 400,
				 .cra_driver_name = "sha1-bst",
				 .cra_ctxsize = sizeof(struct bst_hash_dma_ctx),
				 .cra_exit = bst_ahash_exit_tfm,
				 .cra_blocksize = SHA1_BLOCK_SIZE,
				 .cra_module = THIS_MODULE, },
		},
	},
	{.init = bst_amd5_dma_init,
	.update = bst_ahash_dma_update,
	.final = bst_ahash_dma_final,
	.finup = bst_ahash_dma_finup,
	.digest = bst_amd5_dma_digest,
	.export = bst_ahash_dma_export,
	.import = bst_ahash_dma_import, 
	.halg = {
		.digestsize = MD5_DIGEST_SIZE,
		.statesize = sizeof(struct bst_hash_dma_ctx),
		.base = {.cra_name = "md5",
				 .cra_priority = 400,
				 .cra_driver_name = "md5-bst",
				 .cra_ctxsize = sizeof(struct bst_hash_dma_ctx),
				 .cra_exit = bst_ahash_exit_tfm,
				 .cra_blocksize = MD5_HMAC_BLOCK_SIZE,
				 .cra_module = THIS_MODULE, },
		},
	},
	{.init = bst_asm3_dma_init,
	.update = bst_ahash_dma_update,
	.final = bst_ahash_dma_final,
	.finup = bst_ahash_dma_finup,
	.digest = bst_asm3_dma_digest,
	.export = bst_ahash_dma_export,
	.import = bst_ahash_dma_import, 
	.halg = {
		.digestsize = SM3_DIGEST_SIZE,
		.statesize = sizeof(struct bst_hash_dma_ctx),
		.base = {.cra_name = "sm3",
				 .cra_priority = 400,
				 .cra_driver_name = "sm3-bst",
				 .cra_ctxsize = sizeof(struct bst_hash_dma_ctx),
				 .cra_exit = bst_ahash_exit_tfm,
				 .cra_blocksize = SM3_BLOCK_SIZE,
				 .cra_module = THIS_MODULE, },
		},
	},
	{.init = bst_asha224_dma_init,
	.update = bst_ahash_dma_update,
	.final = bst_ahash_dma_final,
	.finup = bst_ahash_dma_finup,
	.digest = bst_asha224_dma_digest,
	.export = bst_ahash_dma_export,
	.import = bst_ahash_dma_import, 
	.halg = {
		.digestsize = SHA224_DIGEST_SIZE,
		.statesize = sizeof(struct bst_hash_dma_ctx),
		.base = {.cra_name = "sha224",
				 .cra_priority = 400,
				 .cra_driver_name = "sha224-bst",
				 .cra_ctxsize = sizeof(struct bst_hash_dma_ctx),
				 .cra_exit = bst_ahash_exit_tfm,
				 .cra_blocksize = SHA224_BLOCK_SIZE,
				 .cra_module = THIS_MODULE, },
		},
	},
	/* hmac */
	{.init = bst_ahmac_sha256_dma_init,
	.update = bst_ahash_dma_update,
	.final = bst_ahash_dma_final,
	.finup = bst_ahash_dma_finup,
	.digest = bst_ahmac_sha256_dma_digest,
	.setkey = bst_ahmac_setkey, 
	.export = bst_ahash_dma_export,
	.import = bst_ahash_dma_import, 
	.halg = {
		.digestsize = SHA256_DIGEST_SIZE,
		.statesize = sizeof(struct bst_hash_dma_ctx),
		.base = {.cra_name = "hmac(sha256)",
				 .cra_priority = 400,
				 .cra_driver_name = "hmac-sha256-bst",
				 .cra_ctxsize = sizeof(struct bst_hash_dma_ctx),
				 .cra_exit = bst_ahash_exit_tfm,
				 .cra_blocksize = SHA256_BLOCK_SIZE,
				 .cra_module = THIS_MODULE, },
		},
	},
	{.init = bst_ahmac_sha224_dma_init,
	.update = bst_ahash_dma_update,
	.final = bst_ahash_dma_final,
	.finup = bst_ahash_dma_finup,
	.digest = bst_ahmac_sha224_dma_digest,
	.setkey = bst_ahmac_setkey, 
	.export = bst_ahash_dma_export,
	.import = bst_ahash_dma_import, 
	.halg = {
		.digestsize = SHA224_DIGEST_SIZE,
		.statesize = sizeof(struct bst_hash_dma_ctx),
		.base = {.cra_name = "hmac(sha224)",
				 .cra_priority = 400,
				 .cra_driver_name = "hmac-sha224-bst",
				 .cra_ctxsize = sizeof(struct bst_hash_dma_ctx),
				 .cra_exit = bst_ahash_exit_tfm,
				 .cra_blocksize = SHA224_BLOCK_SIZE,
				 .cra_module = THIS_MODULE, },
		},
	},
	{.init = bst_ahmac_sha512_dma_init,
	.update = bst_ahash_dma_update,
	.final = bst_ahash_dma_final,
	.finup = bst_ahash_dma_finup,
	.digest = bst_ahmac_sha512_dma_digest,
	.setkey = bst_ahmac_setkey, 
	.export = bst_ahash_dma_export,
	.import = bst_ahash_dma_import, 
	.halg = {
		.digestsize = SHA512_DIGEST_SIZE,
		.statesize = sizeof(struct bst_hash_dma_ctx),
		.base = {.cra_name = "hmac(sha512)",
				 .cra_priority = 400,
				 .cra_driver_name = "hmac-sha512-bst",
				 .cra_ctxsize = sizeof(struct bst_hash_dma_ctx),
				 .cra_exit = bst_ahash_exit_tfm,
				 .cra_blocksize = SHA512_BLOCK_SIZE,
				 .cra_module = THIS_MODULE, },
		},
	},
	{.init = bst_ahmac_sha1_dma_init,
	.update = bst_ahash_dma_update,
	.final = bst_ahash_dma_final,
	.finup = bst_ahash_dma_finup,
	.digest = bst_ahmac_sha1_dma_digest,
	.setkey = bst_ahmac_setkey, 
	.export = bst_ahash_dma_export,
	.import = bst_ahash_dma_import, 
	.halg = {
		.digestsize = SHA1_DIGEST_SIZE,
		.statesize = sizeof(struct bst_hash_dma_ctx),
		.base = {.cra_name = "hmac(sha1)",
				 .cra_priority = 400,
				 .cra_driver_name = "hmac-sha1-bst",
				 .cra_ctxsize = sizeof(struct bst_hash_dma_ctx),
				 .cra_exit = bst_ahash_exit_tfm,
				 .cra_blocksize = SHA1_BLOCK_SIZE,
				 .cra_module = THIS_MODULE, },
		},
	},
	{.init = bst_ahmac_md5_dma_init,
	.update = bst_ahash_dma_update,
	.final = bst_ahash_dma_final,
	.finup = bst_ahash_dma_finup,
	.digest = bst_ahmac_md5_dma_digest,
	.setkey = bst_ahmac_setkey, 
	.export = bst_ahash_dma_export,
	.import = bst_ahash_dma_import, 
	.halg = {
		.digestsize = MD5_DIGEST_SIZE,
		.statesize = sizeof(struct bst_hash_dma_ctx),
		.base = {.cra_name = "hmac(md5)",
				 .cra_priority = 400,
				 .cra_driver_name = "hmac-md5-bst",
				 .cra_ctxsize = sizeof(struct bst_hash_dma_ctx),
				 .cra_exit = bst_ahash_exit_tfm,
				 .cra_blocksize = MD5_HMAC_BLOCK_SIZE,
				 .cra_module = THIS_MODULE, },
		},
	},
	{.init = bst_ahmac_sm3_dma_init,
	.update = bst_ahash_dma_update,
	.final = bst_ahash_dma_final,
	.finup = bst_ahash_dma_finup,
	.digest = bst_ahmac_sm3_dma_digest,
	.setkey = bst_ahmac_setkey, 
	.export = bst_ahash_dma_export,
	.import = bst_ahash_dma_import, 
	.halg = {
		.digestsize = SM3_DIGEST_SIZE,
		.statesize = sizeof(struct bst_hash_dma_ctx),
		.base = {.cra_name = "hmac(sm3)",
				 .cra_priority = 400,
				 .cra_driver_name = "hmac-sm3-bst",
				 .cra_ctxsize = sizeof(struct bst_hash_dma_ctx),
				 .cra_exit = bst_ahash_exit_tfm,
				 .cra_blocksize = SM3_BLOCK_SIZE,
				 .cra_module = THIS_MODULE, },
		},
	},

	{.init = bst_asha256_dma_init,
	.update = bst_ahash_dma_update,
	.final = bst_ahash_dma_final,
	.finup = bst_ahash_dma_finup,
	.digest = bst_asha256_dma_digest,
	.export = bst_ahash_dma_export,
	.import = bst_ahash_dma_import, 
	.halg = {
		.digestsize = SHA256_DIGEST_SIZE,
		.statesize = sizeof(struct bst_hash_dma_ctx),
		.base = {.cra_name = "bst_sha256_dma",
				 .cra_driver_name = "sha256-bst",
				 .cra_ctxsize = sizeof(struct bst_hash_dma_ctx),
				 .cra_exit = bst_ahash_exit_tfm,
				 .cra_blocksize = SHA256_BLOCK_SIZE,
				 .cra_module = THIS_MODULE, },
		},
	},
	{.init = bst_asha512_dma_init,
	.update = bst_ahash_dma_update,
	.final = bst_ahash_dma_final,
	.finup = bst_ahash_dma_finup,
	.digest = bst_asha512_dma_digest,
	.export = bst_ahash_dma_export,
	.import = bst_ahash_dma_import, 
	.halg = {
		.digestsize = SHA512_DIGEST_SIZE,
		.statesize = sizeof(struct bst_hash_dma_ctx),
		.base = {.cra_name = "bst_sha512_dma",
				 .cra_driver_name = "sha512-bst",
				 .cra_ctxsize = sizeof(struct bst_hash_dma_ctx),
				 .cra_exit = bst_ahash_exit_tfm,
				 .cra_blocksize = SHA512_BLOCK_SIZE,
				 .cra_module = THIS_MODULE, },
		},
	},
	{.init = bst_asha1_dma_init,
	.update = bst_ahash_dma_update,
	.final = bst_ahash_dma_final,
	.finup = bst_ahash_dma_finup,
	.digest = bst_asha1_dma_digest,
	.export = bst_ahash_dma_export,
	.import = bst_ahash_dma_import, 
	.halg = {
		.digestsize = SHA1_DIGEST_SIZE,
		.statesize = sizeof(struct bst_hash_dma_ctx),
		.base = {.cra_name = "bst_sha1_dma",
				 .cra_driver_name = "sha1-bst",
				 .cra_ctxsize = sizeof(struct bst_hash_dma_ctx),
				 .cra_exit = bst_ahash_exit_tfm,
				 .cra_blocksize = SHA1_BLOCK_SIZE,
				 .cra_module = THIS_MODULE, },
		},
	},
	{.init = bst_amd5_dma_init,
	.update = bst_ahash_dma_update,
	.final = bst_ahash_dma_final,
	.finup = bst_ahash_dma_finup,
	.digest = bst_amd5_dma_digest,
	.export = bst_ahash_dma_export,
	.import = bst_ahash_dma_import, 
	.halg = {
		.digestsize = MD5_DIGEST_SIZE,
		.statesize = sizeof(struct bst_hash_dma_ctx),
		.base = {.cra_name = "bst_md5_dma",
				 .cra_driver_name = "md5-bst",
				 .cra_ctxsize = sizeof(struct bst_hash_dma_ctx),
				 .cra_exit = bst_ahash_exit_tfm,
				 .cra_blocksize = MD5_HMAC_BLOCK_SIZE,
				 .cra_module = THIS_MODULE, },
		},
	},
	{.init = bst_asm3_dma_init,
	.update = bst_ahash_dma_update,
	.final = bst_ahash_dma_final,
	.finup = bst_ahash_dma_finup,
	.digest = bst_asm3_dma_digest,
	.export = bst_ahash_dma_export,
	.import = bst_ahash_dma_import, 
	.halg = {
		.digestsize = SM3_DIGEST_SIZE,
		.statesize = sizeof(struct bst_hash_dma_ctx),
		.base = {.cra_name = "bst_sm3_dma",
				 .cra_driver_name = "sm3-bst",
				 .cra_ctxsize = sizeof(struct bst_hash_dma_ctx),
				 .cra_exit = bst_ahash_exit_tfm,
				 .cra_blocksize = SM3_BLOCK_SIZE,
				 .cra_module = THIS_MODULE, },
		},
	},
	{.init = bst_asha224_dma_init,
	.update = bst_ahash_dma_update,
	.final = bst_ahash_dma_final,
	.finup = bst_ahash_dma_finup,
	.digest = bst_asha224_dma_digest,
	.export = bst_ahash_dma_export,
	.import = bst_ahash_dma_import, 
	.halg = {
		.digestsize = SHA224_DIGEST_SIZE,
		.statesize = sizeof(struct bst_hash_dma_ctx),
		.base = {.cra_name = "bst_sha224_dma",
				 .cra_driver_name = "sha224-bst",
				 .cra_ctxsize = sizeof(struct bst_hash_dma_ctx),
				 .cra_exit = bst_ahash_exit_tfm,
				 .cra_blocksize = SHA224_BLOCK_SIZE,
				 .cra_module = THIS_MODULE, },
		},
	},
	/* hmac */
	{.init = bst_ahmac_sha256_dma_init,
	.update = bst_ahash_dma_update,
	.final = bst_ahash_dma_final,
	.finup = bst_ahash_dma_finup,
	.digest = bst_ahmac_sha256_dma_digest,
	.setkey = bst_ahmac_setkey, 
	.export = bst_ahash_dma_export,
	.import = bst_ahash_dma_import, 
	.halg = {
		.digestsize = SHA256_DIGEST_SIZE,
		.statesize = sizeof(struct bst_hash_dma_ctx),
		.base = {.cra_name = "bst_hmac_sha256_dma",
				 .cra_driver_name = "hmac-sha256-bst",
				 .cra_ctxsize = sizeof(struct bst_hash_dma_ctx),
				 .cra_exit = bst_ahash_exit_tfm,
				 .cra_blocksize = SHA256_BLOCK_SIZE,
				 .cra_module = THIS_MODULE, },
		},
	},
	{.init = bst_ahmac_sha224_dma_init,
	.update = bst_ahash_dma_update,
	.final = bst_ahash_dma_final,
	.finup = bst_ahash_dma_finup,
	.digest = bst_ahmac_sha224_dma_digest,
	.setkey = bst_ahmac_setkey, 
	.export = bst_ahash_dma_export,
	.import = bst_ahash_dma_import, 
	.halg = {
		.digestsize = SHA224_DIGEST_SIZE,
		.statesize = sizeof(struct bst_hash_dma_ctx),
		.base = {.cra_name = "bst_hmac_sha224_dma",
				 .cra_driver_name = "hmac-sha224-bst",
				 .cra_ctxsize = sizeof(struct bst_hash_dma_ctx),
				 .cra_exit = bst_ahash_exit_tfm,
				 .cra_blocksize = SHA224_BLOCK_SIZE,
				 .cra_module = THIS_MODULE, },
		},
	},
	{.init = bst_ahmac_sha512_dma_init,
	.update = bst_ahash_dma_update,
	.final = bst_ahash_dma_final,
	.finup = bst_ahash_dma_finup,
	.digest = bst_ahmac_sha512_dma_digest,
	.setkey = bst_ahmac_setkey, 
	.export = bst_ahash_dma_export,
	.import = bst_ahash_dma_import, 
	.halg = {
		.digestsize = SHA512_DIGEST_SIZE,
		.statesize = sizeof(struct bst_hash_dma_ctx),
		.base = {.cra_name = "bst_hmac_sha512_dma",
				 .cra_driver_name = "hmac-sha512-bst",
				 .cra_ctxsize = sizeof(struct bst_hash_dma_ctx),
				 .cra_exit = bst_ahash_exit_tfm,
				 .cra_blocksize = SHA512_BLOCK_SIZE,
				 .cra_module = THIS_MODULE, },
		},
	},
	{.init = bst_ahmac_sha1_dma_init,
	.update = bst_ahash_dma_update,
	.final = bst_ahash_dma_final,
	.finup = bst_ahash_dma_finup,
	.digest = bst_ahmac_sha1_dma_digest,
	.setkey = bst_ahmac_setkey, 
	.export = bst_ahash_dma_export,
	.import = bst_ahash_dma_import, 
	.halg = {
		.digestsize = SHA1_DIGEST_SIZE,
		.statesize = sizeof(struct bst_hash_dma_ctx),
		.base = {.cra_name = "bst_hmac_sha1_dma",
				 .cra_driver_name = "hmac-sha1-bst",
				 .cra_ctxsize = sizeof(struct bst_hash_dma_ctx),
				 .cra_exit = bst_ahash_exit_tfm,
				 .cra_blocksize = SHA1_BLOCK_SIZE,
				 .cra_module = THIS_MODULE, },
		},
	},
	{.init = bst_ahmac_md5_dma_init,
	.update = bst_ahash_dma_update,
	.final = bst_ahash_dma_final,
	.finup = bst_ahash_dma_finup,
	.digest = bst_ahmac_md5_dma_digest,
	.setkey = bst_ahmac_setkey, 
	.export = bst_ahash_dma_export,
	.import = bst_ahash_dma_import, 
	.halg = {
		.digestsize = MD5_DIGEST_SIZE,
		.statesize = sizeof(struct bst_hash_dma_ctx),
		.base = {.cra_name = "bst_hmac_md5_dma",
				 .cra_driver_name = "hmac-md5-bst",
				 .cra_ctxsize = sizeof(struct bst_hash_dma_ctx),
				 .cra_exit = bst_ahash_exit_tfm,
				 .cra_blocksize = MD5_HMAC_BLOCK_SIZE,
				 .cra_module = THIS_MODULE, },
		},
	},
	{.init = bst_ahmac_sm3_dma_init,
	.update = bst_ahash_dma_update,
	.final = bst_ahash_dma_final,
	.finup = bst_ahash_dma_finup,
	.digest = bst_ahmac_sm3_dma_digest,
	.setkey = bst_ahmac_setkey, 
	.export = bst_ahash_dma_export,
	.import = bst_ahash_dma_import, 
	.halg = {
		.digestsize = SM3_DIGEST_SIZE,
		.statesize = sizeof(struct bst_hash_dma_ctx),
		.base = {.cra_name = "bst_hmac_sm3_dma",
				 .cra_driver_name = "hmac-sm3-bst",
				 .cra_ctxsize = sizeof(struct bst_hash_dma_ctx),
				 .cra_exit = bst_ahash_exit_tfm,
				 .cra_blocksize = SM3_BLOCK_SIZE,
				 .cra_module = THIS_MODULE, },
		},
	},
};

static struct shash_alg bst_algs[] = {
	{.digestsize = SM3_DIGEST_SIZE,
	 .init = bst_sm3_init,
	 .update = bst_hash_update,
	 .final = bst_hash_final,
	 .finup = bst_hash_finup,
	 .digest = bst_sm3_digest,
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
	{.digestsize = MD5_DIGEST_SIZE,
	 .init = bst_md5_init,
	 .update = bst_hash_update,
	 .final = bst_hash_final,
	 .finup = bst_hash_finup,
	 .digest = bst_md5_digest,
	 .export = bst_hash_export,
	 .import = bst_hash_import,
	 .descsize = sizeof(struct bst_hash_ctx),
	 .statesize = sizeof(struct bst_hash_ctx),
	 .base = {
		.cra_name = "bst_md5",
		.cra_driver_name = "md5-bst",
		//.cra_flags = CRYPTO_ALG_KERN_DRIVER_ONLY,
		.cra_blocksize = MD5_HMAC_BLOCK_SIZE,
		//.cra_ctxsize = sizeof(struct bst_hash_ctx),
		.cra_module = THIS_MODULE,
	 }},
	{.digestsize = SHA256_DIGEST_SIZE,
	 .init = bst_sha256_init,
	 .update = bst_hash_update,
	 .final = bst_hash_final,
	 .finup = bst_hash_finup,
	 .digest = bst_sha256_digest,
	 .export = bst_hash_export,
	 .import = bst_hash_import,
	 .descsize = sizeof(struct bst_hash_ctx),
	 .statesize = sizeof(struct bst_hash_ctx),
	 .base = {
		.cra_name = "bst_sha256",
		.cra_driver_name = "sha256-bst",
		//.cra_flags = CRYPTO_ALG_KERN_DRIVER_ONLY,
		.cra_blocksize = SHA256_BLOCK_SIZE,
		//.cra_ctxsize = sizeof(struct bst_hash_ctx),
		.cra_module = THIS_MODULE,
	 }},
	{	 .digestsize = SHA1_DIGEST_SIZE,
	 .init = bst_sha1_init,
	 .update = bst_hash_update,
	 .final = bst_hash_final,
	 .finup = bst_hash_finup,
	 .digest = bst_sha1_digest,
	 .export = bst_hash_export,
	 .import = bst_hash_import,
	 .descsize = sizeof(struct bst_hash_ctx),
	 .statesize = sizeof(struct bst_hash_ctx),
	 .base = {
		.cra_name = "bst_sha1",
		.cra_driver_name = "sha1-bst",
		//.cra_flags = CRYPTO_ALG_KERN_DRIVER_ONLY,
		.cra_blocksize = SHA1_BLOCK_SIZE,
		//.cra_ctxsize = sizeof(struct bst_hash_ctx),
		.cra_module = THIS_MODULE,
	 }},
	{	 .digestsize = SHA224_DIGEST_SIZE,
	 .init = bst_sha224_init,
	 .update = bst_hash_update,
	 .final = bst_hash_final,
	 .finup = bst_hash_finup,
	 .digest = bst_sha224_digest,
	 .export = bst_hash_export,
	 .import = bst_hash_import,
	 .descsize = sizeof(struct bst_hash_ctx),
	 .statesize = sizeof(struct bst_hash_ctx),
	 .base = {
		.cra_name = "bst_sha224",
		.cra_driver_name = "sha224-bst",
		//.cra_flags = CRYPTO_ALG_KERN_DRIVER_ONLY,
		.cra_blocksize = SHA224_BLOCK_SIZE,
		//.cra_ctxsize = sizeof(struct bst_hash_ctx),
		.cra_module = THIS_MODULE,
	 }},
	{	 .digestsize = SHA512_DIGEST_SIZE,
	 .init = bst_sha512_init,
	 .update = bst_hash_update,
	 .final = bst_hash_final,
	 .finup = bst_hash_finup,
	 .digest = bst_sha512_digest,
	 .export = bst_hash_export,
	 .import = bst_hash_import,
	 .descsize = sizeof(struct bst_hash_ctx),
	 .statesize = sizeof(struct bst_hash_ctx),
	 .base = {
		.cra_name = "bst_sha512",
		.cra_driver_name = "sha512-bst",
		//.cra_flags = CRYPTO_ALG_KERN_DRIVER_ONLY,
		.cra_blocksize = SHA512_BLOCK_SIZE,
		//.cra_ctxsize = sizeof(struct bst_hash_ctx),
		.cra_module = THIS_MODULE,
	 }},
	{	 .digestsize = SHA512_DIGEST_SIZE,
	 .init = bst_sha512_224_init,
	 .update = bst_hash_update,
	 .final = bst_hash_final,
	 .finup = bst_hash_finup,
	 .digest = bst_sha512_224_digest,
	 .export = bst_hash_export,
	 .import = bst_hash_import,
	 .descsize = sizeof(struct bst_hash_ctx),
	 .statesize = sizeof(struct bst_hash_ctx),
	 .base = {
		.cra_name = "bst_sha512_224",
		.cra_driver_name = "sha512-bst",
		//.cra_flags = CRYPTO_ALG_KERN_DRIVER_ONLY,
		.cra_blocksize = SHA512_BLOCK_SIZE,
		//.cra_ctxsize = sizeof(struct bst_hash_ctx),
		.cra_module = THIS_MODULE,
	 }},
	{	 .digestsize = SHA512_DIGEST_SIZE,
	 .init = bst_sha512_256_init,
	 .update = bst_hash_update,
	 .final = bst_hash_final,
	 .finup = bst_hash_finup,
	 .digest = bst_sha512_256_digest,
	 .export = bst_hash_export,
	 .import = bst_hash_import,
	 .descsize = sizeof(struct bst_hash_ctx),
	 .statesize = sizeof(struct bst_hash_ctx),
	 .base = {
		.cra_name = "bst_sha512_256",
		.cra_driver_name = "sha512-bst",
		//.cra_flags = CRYPTO_ALG_KERN_DRIVER_ONLY,
		.cra_blocksize = SHA512_BLOCK_SIZE,
		//.cra_ctxsize = sizeof(struct bst_hash_ctx),
		.cra_module = THIS_MODULE,
	 }},
	{.digestsize = SHA256_DIGEST_SIZE,
	 .init = bst_hmac_sha256_init,
	 .update = bst_hash_update,
	 .final = bst_hash_final,
	 .finup = bst_hash_finup,
	 .digest = bst_hmac_sha256_digest,
	 .export = bst_hash_export,
	 .import = bst_hash_import,
	 .setkey = bst_hmac_setkey,
	 .descsize = sizeof(struct bst_hash_ctx),
	 .statesize = sizeof(struct bst_hash_ctx),
	 .base = {
		.cra_name = "bst_hmac_sha256",
		.cra_driver_name = "hmac-sha256-bst",
		//.cra_ctxsize = sizeof(struct bst_hash_ctx),
		//.cra_flags = CRYPTO_ALG_KERN_DRIVER_ONLY,
		//.cra_init = bst_shash_init_tfm,
        .cra_exit = bst_shash_exit_tfm,
		.cra_blocksize = SHA256_BLOCK_SIZE,
		.cra_module = THIS_MODULE,
	 }},
	{.digestsize = SHA224_DIGEST_SIZE,
	 .init = bst_hmac_sha224_init,
	 .update = bst_hash_update,
	 .final = bst_hash_final,
	 .finup = bst_hash_finup,
	 .digest = bst_hmac_sha224_digest,
	 .export = bst_hash_export,
	 .import = bst_hash_import,
	 .setkey = bst_hmac_setkey,
	 .descsize = sizeof(struct bst_hash_ctx),
	 .statesize = sizeof(struct bst_hash_ctx),
	 .base = {
		.cra_name = "bst_hmac_sha224",
		.cra_driver_name = "hmac-sha224-bst",
		//.cra_ctxsize = sizeof(struct bst_hash_ctx),
		//.cra_flags = CRYPTO_ALG_KERN_DRIVER_ONLY,
		//.cra_init = bst_shash_init_tfm,
        .cra_exit = bst_shash_exit_tfm,
		.cra_blocksize = SHA224_BLOCK_SIZE,
		.cra_module = THIS_MODULE,
	 }},
	{.digestsize = SHA512_DIGEST_SIZE,
	 .init = bst_hmac_sha512_init,
	 .update = bst_hash_update,
	 .final = bst_hash_final,
	 .finup = bst_hash_finup,
	 .digest = bst_hmac_sha512_digest,
	 .export = bst_hash_export,
	 .import = bst_hash_import,
	 .setkey = bst_hmac_setkey,
	 .descsize = sizeof(struct bst_hash_ctx),
	 .statesize = sizeof(struct bst_hash_ctx),
	 .base = {
		.cra_name = "bst_hmac_sha512",
		.cra_driver_name = "hmac-sha512-bst",
		//.cra_ctxsize = sizeof(struct bst_hash_ctx),
		//.cra_flags = CRYPTO_ALG_KERN_DRIVER_ONLY,
		//.cra_init = bst_shash_init_tfm,
        .cra_exit = bst_shash_exit_tfm,
		.cra_blocksize = SHA512_BLOCK_SIZE,
		.cra_module = THIS_MODULE,
	 }},
	{.digestsize = SHA512_DIGEST_SIZE,
	 .init = bst_hmac_sha512_224_init,
	 .update = bst_hash_update,
	 .final = bst_hash_final,
	 .finup = bst_hash_finup,
	 .digest = bst_hmac_sha512_224_digest,
	 .export = bst_hash_export,
	 .import = bst_hash_import,
	 .setkey = bst_hmac_setkey,
	 .descsize = sizeof(struct bst_hash_ctx),
	 .statesize = sizeof(struct bst_hash_ctx),
	 .base = {
		.cra_name = "bst_hmac_sha512_224",
		.cra_driver_name = "hmac-sha512-224-bst",
		//.cra_ctxsize = sizeof(struct bst_hash_ctx),
		//.cra_flags = CRYPTO_ALG_KERN_DRIVER_ONLY,
		//.cra_init = bst_shash_init_tfm,
        .cra_exit = bst_shash_exit_tfm,
		.cra_blocksize = SHA512_BLOCK_SIZE,
		.cra_module = THIS_MODULE,
	 }},
	{.digestsize = SHA512_DIGEST_SIZE,
	 .init = bst_hmac_sha512_256_init,
	 .update = bst_hash_update,
	 .final = bst_hash_final,
	 .finup = bst_hash_finup,
	 .digest = bst_hmac_sha512_256_digest,
	 .export = bst_hash_export,
	 .import = bst_hash_import,
	 .setkey = bst_hmac_setkey,
	 .descsize = sizeof(struct bst_hash_ctx),
	 .statesize = sizeof(struct bst_hash_ctx),
	 .base = {
		.cra_name = "bst_hmac_sha512_256",
		.cra_driver_name = "hmac-sha512-256-bst",
		//.cra_ctxsize = sizeof(struct bst_hash_ctx),
		//.cra_flags = CRYPTO_ALG_KERN_DRIVER_ONLY,
		//.cra_init = bst_shash_init_tfm,
        .cra_exit = bst_shash_exit_tfm,
		.cra_blocksize = SHA512_BLOCK_SIZE,
		.cra_module = THIS_MODULE,
	 }},
	{.digestsize = SHA1_DIGEST_SIZE,
	 .init = bst_hmac_sha1_init,
	 .update = bst_hash_update,
	 .final = bst_hash_final,
	 .finup = bst_hash_finup,
	 .digest = bst_hmac_sha1_digest,
	 .export = bst_hash_export,
	 .import = bst_hash_import,
	 .setkey = bst_hmac_setkey,
	 .descsize = sizeof(struct bst_hash_ctx),
	 .statesize = sizeof(struct bst_hash_ctx),
	 .base = {
		.cra_name = "bst_hmac_sha1",
		.cra_driver_name = "hmac-sha1-bst",
		//.cra_ctxsize = sizeof(struct bst_hash_ctx),
		//.cra_flags = CRYPTO_ALG_KERN_DRIVER_ONLY,
		//.cra_init = bst_shash_init_tfm,
        .cra_exit = bst_shash_exit_tfm,
		.cra_blocksize = SHA1_BLOCK_SIZE,
		.cra_module = THIS_MODULE,
	 }},
	{.digestsize = MD5_DIGEST_SIZE,
	 .init = bst_hmac_md5_init,
	 .update = bst_hash_update,
	 .final = bst_hash_final,
	 .finup = bst_hash_finup,
	 .digest = bst_hmac_md5_digest,
	 .export = bst_hash_export,
	 .import = bst_hash_import,
	 .setkey = bst_hmac_setkey,
	 .descsize = sizeof(struct bst_hash_ctx),
	 .statesize = sizeof(struct bst_hash_ctx),
	 .base = {
		.cra_name = "bst_hmac_md5",
		.cra_driver_name = "hmac-md5-bst",
		//.cra_ctxsize = sizeof(struct bst_hash_ctx),
		//.cra_flags = CRYPTO_ALG_KERN_DRIVER_ONLY,
		//.cra_init = bst_shash_init_tfm,
        .cra_exit = bst_shash_exit_tfm,
		.cra_blocksize = MD5_HMAC_BLOCK_SIZE,
		.cra_module = THIS_MODULE,
	 }},
	{.digestsize = SM3_DIGEST_SIZE,
	 .init = bst_hmac_sm3_init,
	 .update = bst_hash_update,
	 .final = bst_hash_final,
	 .finup = bst_hash_finup,
	 .digest = bst_hmac_sm3_digest,
	 .export = bst_hash_export,
	 .import = bst_hash_import,
	 .setkey = bst_hmac_setkey,
	 .descsize = sizeof(struct bst_hash_ctx),
	 .statesize = sizeof(struct bst_hash_ctx),
	 .base = {
		.cra_name = "bst_hmac_sm3",
		.cra_driver_name = "hamc-sm3-bst",
		//.cra_ctxsize = sizeof(struct bst_hash_ctx),
		//.cra_flags = CRYPTO_ALG_KERN_DRIVER_ONLY,
		//.cra_init = bst_shash_init_tfm,
        .cra_exit = bst_shash_exit_tfm,
		.cra_blocksize = SM3_BLOCK_SIZE,
		.cra_module = THIS_MODULE,
	 }},
};

static const struct of_device_id bst_hash_match[] = {
	{.compatible = "bst,c1200-hfe"},
	{}};
MODULE_DEVICE_TABLE(of, bst_hash_match);

int bst_register_all_hfe_algs(void){
	int i, ret;
	for (i = 0; i < ARRAY_SIZE(bst_algs); i++) {
		ret = crypto_register_shash(&bst_algs[i]);
		if (ret) {
			pr_err("hfe: Failed to register shash algo [%s] driver [%s], err=%d\n",
				bst_algs[i].base.cra_name,
				bst_algs[i].base.cra_driver_name,
				ret);
			return ret;
		}
	}
	
	for (i = 0; i < ARRAY_SIZE(bst_ahash_algs); i++) {
		ret = crypto_register_ahash(&bst_ahash_algs[i]);
		if (ret) {
			pr_err("hfe: Failed to register ahash algo [%s] driver [%s], err=%d\n",
				bst_ahash_algs[i].halg.base.cra_name,
				bst_ahash_algs[i].halg.base.cra_driver_name,
				ret);
			return ret;
		}
	}

	return 0;
}

static int bst_hash_probe(struct platform_device *pdev)
{
	struct bst_hash_dev *hdev;
	struct device *dev = &pdev->dev;
	int err, ret;
	uint32_t v_major, v_minor;
	// u32 irq_remap[2];

	of_reserved_mem_device_init(&pdev->dev);
	dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(40));
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
	global_hash = hdev;

	if (bst_sec_sa_hfe_enable) {
		mutex_lock(&refcnt_lock);
		if (refcnt++ == 0) {
			ret = bst_register_all_hfe_algs();
			if (ret) {
				//mutex_unlock(&refcnt_lock);
				dev_err(dev, "Failed to register hfe algs\n");
				//return ret;
			}else{
				dev_info(&pdev->dev, "BST hfe algorithms registered\n");
			}
		}
		mutex_unlock(&refcnt_lock);
		//dev_info(&pdev->dev, "BST hash algorithms registered\n");
	} else {
		dev_info(&pdev->dev, "BST hash driver loaded but algorithms disabled (bst_sec_sa_hfe_enable=0)\n");
	}

	// INIT_WORK(&hdev->work, bst_hfe_work_func);
	// queue_work(system_wq, &hdev->work);
	hfe_get_version(hdev->io_base, &v_major, &v_minor);
	dev_info(dev, "Hardware version: v%d.%d\n", v_major, v_minor);

	return 0;
res_err:
	return err;
}

void sa_enbale_change_hfe(void){
	int ret;
	mutex_lock(&refcnt_lock);
	if (bst_sec_sa_hfe_enable && refcnt == 0) {
		ret = bst_register_all_hfe_algs();
		if (ret) {
			pr_err( "Failed to register hfe algs\n");
		}else{
			pr_info( "BST hfe algorithms registered\n");
		}
		refcnt = 1;
	} else if (!bst_sec_sa_hfe_enable && refcnt) {
		crypto_unregister_shashes(bst_algs, ARRAY_SIZE(bst_algs));
		crypto_unregister_ahashes(bst_ahash_algs, ARRAY_SIZE(bst_ahash_algs));
		refcnt = 0;
		pr_info( "crypto_unregister_shashes and crypto_unregister_ahashes\n");
	}
	mutex_unlock(&refcnt_lock);
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
	if (!--refcnt) {
		crypto_unregister_shashes(bst_algs, ARRAY_SIZE(bst_algs));
		crypto_unregister_ahashes(bst_ahash_algs, ARRAY_SIZE(bst_ahash_algs));
	}

	mutex_unlock(&refcnt_lock);
	global_hash = NULL;
	return 0;
}

static struct platform_driver bst_hash_driver = {
	.probe = bst_hash_probe,
	.remove = bst_hash_remove,
	.driver = {
		.name = "bst-hfe",
		.of_match_table = bst_hash_match,
	},
};

module_platform_driver(bst_hash_driver);

// static int __init bst_hash_driver_init(void)
// {
// 	bst_dbg(2, "%s: %d\n", __func__, __LINE__);
// 	return platform_driver_register(&bst_hash_driver);
// }

// static void __exit bst_hash_driver_exit(void)
// {
// 	bst_dbg(2, "%s: %d", __func__, __LINE__);
// 	return platform_driver_unregister(&bst_hash_driver);
// }

// module_init(bst_hash_driver_init);
// module_exit(bst_hash_driver_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("BST Hash Function Engine driver");
