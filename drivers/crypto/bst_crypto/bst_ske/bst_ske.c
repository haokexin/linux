/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (C) 2024 Black Sesame Technologies. Inc.
 */

#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/stmp_device.h>
#include <linux/clk.h>
#include <crypto/gcm.h>
#include <crypto/aes.h>
#include <crypto/des.h>
#include <crypto/internal/skcipher.h>
#include <crypto/scatterwalk.h>
#include <crypto/sm4.h>
#include <crypto/gcm.h>
#include <crypto/xts.h>
#include <crypto/internal/aead.h>
#include <crypto/internal/skcipher.h>
#include <linux/scatterlist.h>
#include "bst_ske.h"

#define SKE_MAX_CHANS 4
#define SKE_BUF_SZ PAGE_SIZE
#define SKE_ALIGNMENT 64
#define SKE_ADDR(offset) (global_ske->base + offset)

struct ske_dma_desc {
	uint32_t next_cmd_addr;
	uint32_t control0;
	uint32_t control1;
	uint32_t source;
	uint32_t destination;
	uint32_t size;
	uint32_t payload;
	uint32_t status;
};

struct ske_coherent_block {
	uint8_t aes_in_buf[SKE_BUF_SZ];
	uint8_t aes_out_buf[SKE_BUF_SZ];
	uint8_t aes_key[2 * AES_KEYSIZE_128];

	struct ske_dma_desc desc[SKE_MAX_CHANS];
};

struct ske {
	struct device *dev;
	void __iomem *base;
	struct ske_coherent_block *coh;
	struct completion completion[SKE_MAX_CHANS];
	spinlock_t lock[SKE_MAX_CHANS];
	struct task_struct *thread[SKE_MAX_CHANS];
	struct crypto_queue queue[SKE_MAX_CHANS];
	int irq;
};

struct ske_ctx {
	/* Common context */
	uint32_t fill;

	/* SHA Hash-specific context */
	struct mutex mutex;
	// uint32_t				alg;
	unsigned int hot : 1;

	/* Crypto-specific context */
	struct crypto_skcipher *fallback;
	unsigned int key_len;
	uint8_t key[AES_MAX_KEY_SIZE * 2];
	uint8_t block_bytes;
	uint8_t block_words;
	enum ske_alg alg;
	enum ske_mode mode;
	enum ske_crypto crypto;
	uint8_t buf[16];
	uint32_t c_bytes;
	uint32_t current_bytes;
	uint32_t aad_bytes;
	uint32_t mac_bytes;
	uint8_t b1_aad_start_offset;
	uint8_t b1_aad_end_offset;
	uint8_t M;
	uint8_t L;
};

static struct ske *global_ske;
// static void print_buf_u32(uint32_t buf[], uint32_t word_len)
// {
	// uint32_t i;

	// for (i = 0; i < word_len; i++)
		// printk("%08x", buf[i]);

	// printk("\r\n");
// }

static void reverse_byte_array(uint8_t *in, uint8_t *out, uint32_t byteLen)
{
	uint32_t idx, round = byteLen >> 1;
	uint8_t tmp;

	for (idx = 0; idx < round; idx++) {
		tmp = in[idx];
		out[idx] = in[byteLen - 1 - idx];
		out[byteLen - 1 - idx] = tmp;
	}

	if ((byteLen & 0x1) && (in != out))
		out[round] = in[round];
}

// static void ske_enable_interrupt(void)
// {
// 	uint32_t flag = (uint32_t)1;

// 	writel_relaxed(readl_relaxed(SKE_ADDR(SKE_IMCR)) | flag,
// 				   SKE_ADDR(SKE_IMCR));
// }

// static void ske_disable_interrupt(void)
// {
// 	uint32_t mask = ~((uint32_t)1);

// 	writel_relaxed(readl_relaxed(SKE_ADDR(SKE_IMCR)) & mask,
// 				   SKE_ADDR(SKE_IMCR));
// }

uint8_t ske_hp_get_block_byte_len(enum ske_alg ske_alg)
{
	uint8_t byteLen;

	switch (ske_alg) {
	case SKE_ALG_DES:
	case SKE_ALG_TDES_128:
	case SKE_ALG_TDES_192:
	case SKE_ALG_TDES_EEE_128:
	case SKE_ALG_TDES_EEE_192:
		byteLen = 8;
		break;
	case SKE_ALG_AES_128:
	case SKE_ALG_AES_192:
	case SKE_ALG_AES_256:
	case SKE_ALG_SM4:
		byteLen = 16;
		break;
	default:
		byteLen = 16;
	}

	return byteLen;
}

uint8_t ske_hp_get_key_byte_len(enum ske_alg ske_alg)
{
	uint8_t byte_len;

	switch (ske_alg) {
	case SKE_ALG_DES:
		byte_len = 8;
		break;
	case SKE_ALG_TDES_128:
	case SKE_ALG_TDES_EEE_128:
	case SKE_ALG_AES_128:
	case SKE_ALG_SM4:
		byte_len = 16;
		break;
	case SKE_ALG_TDES_192:
	case SKE_ALG_TDES_EEE_192:
	case SKE_ALG_AES_192:
		byte_len = 24;
		break;
	case SKE_ALG_AES_256:
		byte_len = 32;
		break;
	default:
		byte_len = 16;
	}

	return byte_len;
}

void ske_hp_set_cpu_mode(void)
{
	uint32_t mask = ~(((uint32_t)1) << SKE_HP_DMA_OFFSET);

	writel_relaxed(readl_relaxed(SKE_ADDR(SKE_CFG)) & mask, SKE_ADDR(SKE_CFG));
}

void ske_hp_set_endian_uint32(uint32_t endian)
{
	uint32_t mask;
	uint32_t flag;

	mask = ~(((uint32_t)3) << SKE_HP_REVERSE_BYTE_ORDER_IN_WORD_OFFSET);
	flag = (((uint32_t)2) << SKE_HP_REVERSE_BYTE_ORDER_IN_WORD_OFFSET);
	writel_relaxed(readl_relaxed(SKE_ADDR(SKE_CFG)) & mask, SKE_ADDR(SKE_CFG));
	if (!endian)
		writel_relaxed(readl_relaxed(SKE_ADDR(SKE_CFG)) | flag,
					   SKE_ADDR(SKE_CFG));
}

void ske_hp_set_alg(enum ske_alg ske_alg)
{
	uint32_t mask = ~(0x000000FFU);
	uint32_t cfg;

	switch (ske_alg) {
	case SKE_ALG_DES:
		cfg = 3;
		break;
	case SKE_ALG_TDES_128:
	case SKE_ALG_TDES_192:
		cfg = 4;
		break;
	case SKE_ALG_TDES_EEE_128:
	case SKE_ALG_TDES_EEE_192:
		cfg = 5;
		break;
	case SKE_ALG_AES_128:
		cfg = (1 << 6) | (1 << 4) | (1);
		break;
	case SKE_ALG_AES_192:
		cfg = (2 << 6) | (2 << 4) | (1);
		break;
	case SKE_ALG_AES_256:
		cfg = (3 << 6) | (3 << 4) | (1);
		break;
	case SKE_ALG_SM4:
		cfg = 2;
		break;
	default:
		cfg = 2; // default alg SM4
	}

	writel_relaxed(readl_relaxed(SKE_ADDR(SKE_CFG)) & mask, SKE_ADDR(SKE_CFG));
	writel_relaxed(readl_relaxed(SKE_ADDR(SKE_CFG)) | cfg, SKE_ADDR(SKE_CFG));
}

void ske_hp_set_mode(enum ske_mode mode)
{
	uint32_t mask = ~(0x0000000FU << SKE_HP_MODE_OFFSET);
	uint32_t cfg = (((uint32_t)mode) << SKE_HP_MODE_OFFSET);

	writel_relaxed(readl_relaxed(SKE_ADDR(SKE_CFG)) & mask, SKE_ADDR(SKE_CFG));
	writel_relaxed(readl_relaxed(SKE_ADDR(SKE_CFG)) | cfg, SKE_ADDR(SKE_CFG));
}

void ske_hp_set_crypto(enum ske_crypto crypto)
{
	uint32_t mask = ~(((uint32_t)1) << SKE_HP_CRYPTO_OFFSET);
	uint32_t cfg = (((uint32_t)crypto) << SKE_HP_CRYPTO_OFFSET);

	writel_relaxed(readl_relaxed(SKE_ADDR(SKE_CFG)) & mask, SKE_ADDR(SKE_CFG));
	writel_relaxed(readl_relaxed(SKE_ADDR(SKE_CFG)) | cfg, SKE_ADDR(SKE_CFG));
}

void ske_hp_set_last_block(uint32_t is_last_block)
{
	uint32_t flag = (((uint32_t)1) << SKE_HP_LAST_DATA_OFFSET);
	uint32_t mask = ~(((uint32_t)1) << SKE_HP_LAST_DATA_OFFSET);

	if (is_last_block)
		writel_relaxed(readl_relaxed(SKE_ADDR(SKE_DIN_CR)) | flag,
					   SKE_ADDR(SKE_DIN_CR));
	else
		writel_relaxed(readl_relaxed(SKE_ADDR(SKE_DIN_CR)) & mask,
					   SKE_ADDR(SKE_DIN_CR));
}

void ske_hp_set_iv_uint32(uint32_t *iv, uint32_t block_words)
{
	int32_t i;

	for (i = block_words; i > 0; i--)
		writel_relaxed(iv[block_words - i], SKE_ADDR(SKE_IV + (i - 1) * 4));
}

void ske_hp_set_iv(uint8_t *iv, uint32_t block_bytes)
{
	uint32_t tmp[4];

	if (((uint64_t)iv) & 3) {
		memcpy(tmp, iv, block_bytes);
		ske_hp_set_iv_uint32(tmp, block_bytes / 4);
	} else
		ske_hp_set_iv_uint32((uint32_t *)iv, block_bytes / 4);
}

void ske_hp_disable_secure_port(void)
{
	uint32_t mask = ~1;

	writel_relaxed(readl_relaxed(SKE_ADDR(SKE_SP)) & mask, SKE_ADDR(SKE_SP));
}

void ske_hp_set_key_uint32(uint32_t *key, uint32_t idx, uint32_t key_words)
{
	uint32_t *key_reg;
	int32_t i;

	if (idx == 1)
		key_reg = SKE_ADDR(SKE_KEY1);
	else
		key_reg = SKE_ADDR(SKE_KEY2);
	for (i = key_words; i > 0; i--)
		writel_relaxed(key[key_words - i], key_reg + i - 1);
}

void ske_hp_set_key(enum ske_alg alg, uint8_t *key, uint16_t key_bytes,
					uint16_t key_idx)
{
	uint32_t tmp[8];

	memcpy(tmp, key, key_bytes);
	//for 3DES-2key, set key3=key1
	switch (alg) {
	case SKE_ALG_TDES_128:
	case SKE_ALG_TDES_EEE_128:
		memcpy(tmp+4, key, 8);
		key_bytes += 8;
		break;
	default:
		break;
	}
	ske_hp_set_key_uint32(tmp, key_idx, key_bytes / 4);
}

void ske_hp_start(void)
{
	uint32_t clear_flag = 0;
	uint32_t start_flag = 1;

	writel_relaxed(clear_flag, SKE_ADDR(SKE_RISR));
	writel_relaxed(readl_relaxed(SKE_ADDR(SKE_CTRL)) | start_flag,
				   SKE_ADDR(SKE_CTRL));
}

void ske_hp_set_c_len_uint32(uint32_t c_bytes)
{
	writel_relaxed(((c_bytes) << 3) & 0xFFFFFFFF, SKE_ADDR(SKE_C_LEN_L));
	writel_relaxed(c_bytes >> (32 - 3), SKE_ADDR(SKE_C_LEN_H));
}

void ske_hp_set_aad_len_uint32(uint32_t aad_bytes)
{
	writel_relaxed(((aad_bytes) << 3) & 0xFFFFFFFF, SKE_ADDR(SKE_A_LEN_L));
	writel_relaxed(aad_bytes >> (32 - 3), SKE_ADDR(SKE_A_LEN_H));
}

uint32_t ske_hp_wait_till_done(enum ske_wait_mode wait_mode)
{
	uint32_t finish_flag;
	uint32_t alarm_flag = 1;
	uint32_t *reg_status = SKE_ADDR(SKE_SR);

	switch (wait_mode) {
	case WAIT_TILL_EXPAND_KEY_DONE:
		finish_flag = 1;
		break;
	case WAIT_TILL_COULD_INPUT:
		finish_flag = (1 << 16);
		break;
	case WAIT_TILL_OUTPUT_READY:
		finish_flag = (1 << 17);
		break;
	case WAIT_TILL_CALC_DONE:
		finish_flag = 7;
		reg_status = SKE_ADDR(SKE_RISR);
		break;
	default:
		return SKE_ATTACK_ALARM;
	}

	while (!(readl_relaxed(reg_status) & finish_flag)) {
		if (readl_relaxed(SKE_ADDR(SKE_ALARM)) & alarm_flag)
			return SKE_ATTACK_ALARM;
	}

	return SKE_SUCCESS;
}

uint32_t ske_hp_expand_key(void)
{
	uint32_t mask = ~(((uint32_t)1) << SKE_HP_UP_CFG_OFFSET);
	uint32_t flag = (((uint32_t)1) << SKE_HP_UP_CFG_OFFSET);
	uint32_t ret;

	writel_relaxed(readl_relaxed(SKE_ADDR(SKE_CFG)) | flag, SKE_ADDR(SKE_CFG));
	ske_hp_start();
	ret = ske_hp_wait_till_done(WAIT_TILL_EXPAND_KEY_DONE);
	if (ret != SKE_SUCCESS)
		return ret;

	writel_relaxed(readl_relaxed(SKE_ADDR(SKE_CFG)) & mask, SKE_ADDR(SKE_CFG));

	return SKE_SUCCESS;
}

uint32_t ske_hp_init(struct ske_ctx *ctx, uint8_t *iv)
{
	uint32_t key_bytes;

	if (ctx->mode == SKE_MODE_ECB)
		iv = NULL;
	else if (iv == NULL)
		return SKE_BUFFER_NULL;

	ctx->block_bytes = ske_hp_get_block_byte_len(ctx->alg);
	ctx->block_words = ctx->block_bytes / 4;
	if (ctx->mode == SKE_MODE_XTS)
		ske_hp_set_c_len_uint32(ctx->c_bytes);

	if (ctx->mode == SKE_MODE_GCM || ctx->mode == SKE_MODE_CCM) {
		ske_hp_set_c_len_uint32(ctx->c_bytes);
		ske_hp_set_aad_len_uint32(ctx->aad_bytes);
	}

	ske_hp_set_cpu_mode();
	ske_hp_set_endian_uint32(0);
	ske_hp_set_alg(ctx->alg);
	ske_hp_set_mode(ctx->mode);
	ske_hp_set_crypto(ctx->crypto);
	ske_hp_set_last_block(0);
	// ske_enable_interrupt();
	if (iv != NULL)
		ske_hp_set_iv(iv, ctx->block_bytes);

	ske_hp_disable_secure_port();
	key_bytes = ske_hp_get_key_byte_len(ctx->alg);
	ske_hp_set_key(ctx->alg, ctx->key, key_bytes, 1);
	if (ctx->mode == SKE_MODE_XTS)
		ske_hp_set_key(ctx->alg, ctx->key + key_bytes, key_bytes, 2);

	return ske_hp_expand_key();
}

void ske_hp_simple_set_input_block(uint32_t *in, uint32_t block_words)
{
	int32_t i;

	for (i = block_words; i > 0; i--)
		writel_relaxed(in[block_words - i], SKE_ADDR(SKE_DIN + (i - 1) * 4));
}

void ske_hp_simple_get_output_block(uint32_t *out, uint32_t block_words)
{
	uint32_t flag = 0x02;
	int32_t i;

	writel_relaxed(readl_relaxed(SKE_ADDR(SKE_CTRL)) | flag,
				   SKE_ADDR(SKE_CTRL));
	for (i = block_words; i > 0; i--)
		out[block_words - i] = readl_relaxed(SKE_ADDR(SKE_DOUT) + (i - 1) * 4);
}

uint32_t ske_hp_update_blocks_internal(struct ske_ctx *ctx, const uint8_t *in,
									   uint8_t *out, uint32_t bytes)
{
	uint32_t flag_0 = 0x00;
	uint32_t flag_1 = 0x01;
	uint32_t flag_2 = 0x02;
	uint8_t *out_bak = out;
	uint32_t in_word_align, out_word_align;
	uint32_t tmp_in[4];
	uint32_t i, round = bytes / ctx->block_bytes;
	uint32_t block_bytes = ctx->block_bytes;
	uint32_t ret;

	if ((uintptr_t)in& 3)
		in_word_align = 0;
	else
		in_word_align = 1;

	if ((uintptr_t)out & 3)
		out_word_align = 0;
	else
		out_word_align = 1;

	if (in_word_align && out_word_align) {
		if (block_bytes == 16) {
			for (i = 0; i < round; i++) {
				writel_relaxed(((uint32_t *)in)[0], SKE_ADDR(SKE_DIN + 3 * 4));
				writel_relaxed(((uint32_t *)in)[1], SKE_ADDR(SKE_DIN + 2 * 4));
				writel_relaxed(((uint32_t *)in)[2], SKE_ADDR(SKE_DIN + 1 * 4));
				writel_relaxed(((uint32_t *)in)[3], SKE_ADDR(SKE_DIN + 0 * 4));
				writel_relaxed(flag_0, SKE_ADDR(SKE_RISR));
				writel_relaxed(flag_1, SKE_ADDR(SKE_CTRL));
				ret = ske_hp_wait_till_done(WAIT_TILL_OUTPUT_READY);
				if (ret != SKE_SUCCESS) {
					memset(out_bak, 0, bytes);
					return ret;
				}

				writel_relaxed(flag_2, SKE_ADDR(SKE_CTRL));
				((uint32_t *)out)[0] = readl_relaxed(SKE_ADDR(SKE_DOUT + 3 * 4));
				((uint32_t *)out)[1] = readl_relaxed(SKE_ADDR(SKE_DOUT + 2 * 4));
				((uint32_t *)out)[2] = readl_relaxed(SKE_ADDR(SKE_DOUT + 1 * 4));
				((uint32_t *)out)[3] = readl_relaxed(SKE_ADDR(SKE_DOUT + 0 * 4));
				in += block_bytes;
				out += block_bytes;
			}
		} else {
			for (i = 0; i < round; i++) {
				writel_relaxed(((uint32_t *)in)[0], SKE_ADDR(SKE_DIN + 1 * 4));
				writel_relaxed(((uint32_t *)in)[1], SKE_ADDR(SKE_DIN + 0 * 4));
				writel_relaxed(flag_0, SKE_ADDR(SKE_RISR));
				writel_relaxed(flag_1, SKE_ADDR(SKE_CTRL));
				ret = ske_hp_wait_till_done(WAIT_TILL_OUTPUT_READY);
				if (ret != SKE_SUCCESS) {
					memset(out_bak, 0, bytes);
					return ret;
				}

				writel_relaxed(flag_2, SKE_ADDR(SKE_CTRL));
				((uint32_t *)out)[0] = readl_relaxed(SKE_ADDR(SKE_DOUT + 1 * 4));
				((uint32_t *)out)[1] = readl_relaxed(SKE_ADDR(SKE_DOUT + 0 * 4));
				in += block_bytes;
				out += block_bytes;
			}
		}
	} else {
		for (i = 0; i < round; i++) {
			if (in_word_align)
				ske_hp_simple_set_input_block((uint32_t *)in,
											  ctx->block_words);
			else {
				memcpy(tmp_in, in, block_bytes);
				ske_hp_simple_set_input_block((uint32_t *)tmp_in,
											  ctx->block_words);
			}

			ske_hp_start();
			ret = ske_hp_wait_till_done(WAIT_TILL_OUTPUT_READY);
			if (ret != SKE_SUCCESS) {
				memset(out_bak, 0, bytes);
				return ret;
			}

			if (out_word_align)
				ske_hp_simple_get_output_block((uint32_t *)out,
											   ctx->block_words);
			else {
				ske_hp_simple_get_output_block((uint32_t *)tmp_in,
											   ctx->block_words);
				memcpy(out, tmp_in, block_bytes);
			}

			in += block_bytes;
			out += block_bytes;
		}
	}

	return SKE_SUCCESS;
}

uint32_t ske_hp_update_blocks(struct ske_ctx *ctx, const uint8_t *in,
							  uint8_t *out, uint32_t bytes)
{
	if ((in == NULL) || (out == NULL))
		return SKE_BUFFER_NULL;
	else if (bytes & (ctx->block_bytes - 1))
		return SKE_INPUT_INVALID;
	else if (bytes == 0)
		return SKE_SUCCESS;

	return ske_hp_update_blocks_internal(ctx, in, out, bytes);
}
uint32_t ske_hp_update_including_last_2_blocks(struct ske_ctx *ctx,
											   uint8_t *in, uint8_t *out, uint32_t bytes)
{
	uint32_t blocks_bytes;
	uint32_t buf[4];
	uint32_t ret;

	if (ctx == NULL || in == NULL || out == NULL)
		return SKE_BUFFER_NULL;
	else if (bytes <= ctx->block_bytes || !(bytes & 0x0F))
		return SKE_INPUT_INVALID;
	else if (ctx->current_bytes & (ctx->block_bytes - 1) ||
			 ctx->current_bytes + bytes != ctx->c_bytes)
		return SKE_INPUT_INVALID;

	blocks_bytes = (bytes & (~0x0F)) - ctx->block_bytes;
	if (blocks_bytes) {
		ret = ske_hp_update_blocks(ctx, in, out, blocks_bytes);
		if (ret != SKE_SUCCESS)
			return ret;

		in += blocks_bytes;
		out += blocks_bytes;
		bytes -= blocks_bytes;
	}

	memcpy(buf, in, 16);
	ske_hp_simple_set_input_block((uint32_t *)buf, ctx->block_words);
	ske_hp_start();

	ske_hp_set_last_block(1);
	memcpy(buf, in + 16, bytes - 16);
	memset(((uint8_t *)(buf)) + bytes - 16, 0, 32 - bytes);
	ske_hp_simple_set_input_block((uint32_t *)buf, ctx->block_words);
	ske_hp_start();
	ret = ske_hp_wait_till_done(WAIT_TILL_OUTPUT_READY);
	if (ret != SKE_SUCCESS)
		return ret;

	ske_hp_simple_get_output_block((uint32_t *)buf, ctx->block_words);
	memcpy(out, buf, 16);

	ske_hp_simple_get_output_block((uint32_t *)buf, ctx->block_words);
	memcpy(out + 16, buf, bytes - 16);

	return SKE_SUCCESS;
}

uint32_t ske_hp_update_blocks_no_output(struct ske_ctx *ctx, uint8_t *in, uint32_t bytes)
{
	uint32_t flag_2 = 0x02;
	uint32_t in_word_align, is_ccm_gcm_mode = 0;
	uint32_t tmp_in[4];
	uint32_t i;
	uint32_t ret;

	if ((uintptr_t)in & 3)
		in_word_align = 0;
	else
		in_word_align = 1;

	switch (ctx->mode) {
	case SKE_MODE_GCM:
	case SKE_MODE_CCM:
		is_ccm_gcm_mode = 1;
		break;
	default: // CMAC or CBC-MAC mode
		is_ccm_gcm_mode = 0;
	}

	// input one block ---> calculating ---> output one block
	for (i = 0; i < bytes; i += ctx->block_bytes) {
		if (in_word_align)
			ske_hp_simple_set_input_block((uint32_t *)in, ctx->block_words);
		else {
			memcpy(tmp_in, in, ctx->block_bytes);
			ske_hp_simple_set_input_block((uint32_t *)tmp_in, ctx->block_words);
		}

		ske_hp_start();

		if (is_ccm_gcm_mode) {
			ret = ske_hp_wait_till_done(WAIT_TILL_COULD_INPUT);
			if (ret != SKE_SUCCESS)
				return ret;
		} else {
			ret = ske_hp_wait_till_done(WAIT_TILL_CALC_DONE);
			if (ret == SKE_SUCCESS)
				writel_relaxed(readl_relaxed(SKE_ADDR(SKE_CTRL)) | flag_2, SKE_ADDR(SKE_CTRL));
			else
				return ret;
		}
		in += ctx->block_bytes;
	}

	return SKE_SUCCESS;
}

uint32_t ske_hp_gcm_aad(struct ske_ctx *ctx, uint8_t *aad)
{

	uint32_t blocks_bytes, remainder_bytes;
	uint32_t ret = SKE_ERROR;

	if (ctx == NULL || (aad == NULL && ctx->aad_bytes != 0))
		return SKE_BUFFER_NULL;
	else if (ctx->aad_bytes == 0)
		return SKE_SUCCESS;
	// printk("%s:%d", __func__, __LINE__);
	blocks_bytes = (ctx->aad_bytes) & (~0x0F);
	remainder_bytes = (ctx->aad_bytes) & 0x0F;

	if (remainder_bytes == 0) {
		blocks_bytes -= 16;
		remainder_bytes = 16;
	}

	ret = ske_hp_update_blocks_no_output(ctx, aad, blocks_bytes);
	if (ret != SKE_SUCCESS)
		return ret;
	// printk("%s:%d", __func__, __LINE__);
	// the last block
	memcpy(ctx->buf, aad + blocks_bytes, remainder_bytes);
	memset(ctx->buf + remainder_bytes, 0, sizeof(ctx->buf) - remainder_bytes);
	// printk("%s:%d", __func__, __LINE__);
	ske_hp_set_last_block(1);
	ret = ske_hp_update_blocks_no_output(ctx, ctx->buf, 16);
	ske_hp_set_last_block(0);
	// printk("%s:%d", __func__, __LINE__);
	return ret;
}

uint32_t ske_hp_gcm_update_blocks(struct ske_ctx *ctx, uint8_t *in, uint8_t *out, uint32_t bytes)
{
	uint32_t blocks_bytes, remainder_bytes;
	uint32_t total_bytes;
	uint32_t ret = SKE_ERROR;

	if (ctx == NULL || in == NULL || out == NULL)
		return SKE_BUFFER_NULL;
	else if (bytes == 0)
		return SKE_SUCCESS;

	total_bytes = ctx->current_bytes + bytes;
	if (total_bytes < bytes || total_bytes > ctx->c_bytes)
		return SKE_INPUT_INVALID;
	else if (total_bytes == ctx->c_bytes) {
		blocks_bytes = (bytes) & (~0x0F);
		remainder_bytes = (bytes)&0x0F;
		if (remainder_bytes == 0) {
			blocks_bytes -= 16;
			remainder_bytes = 16;
		}

		ret = ske_hp_update_blocks_internal(ctx, in, out, blocks_bytes);
		if (ret != SKE_SUCCESS)
			goto update_end;

		// the last block
		memcpy(ctx->buf, in + blocks_bytes, remainder_bytes);
		memset(ctx->buf + remainder_bytes, 0, sizeof(ctx->buf) - remainder_bytes);

		ske_hp_set_last_block(1);
		ret = ske_hp_update_blocks_internal(ctx, ctx->buf, ctx->buf, 16);
		ske_hp_set_last_block(0);

		if (ret != SKE_SUCCESS)
			goto update_end;

		memcpy(out + blocks_bytes, ctx->buf, remainder_bytes);
	} else {
		if (bytes & (16 - 1)) {
			ret = SKE_INPUT_INVALID;
			goto update_end;
		} else {
			ret = ske_hp_update_blocks_internal(ctx, in, out, bytes);
			if (ret != SKE_SUCCESS)
				goto update_end;
		}
	}

	ret = SKE_SUCCESS;
	ctx->current_bytes = total_bytes;

update_end:

	return ret;
}

uint32_t ske_hp_gcm_final(struct ske_ctx *ctx, uint8_t *mac)
{
	uint32_t ret;

	if (NULL == ctx || NULL == mac)
		return SKE_BUFFER_NULL;

	// printk("%s:%d", __func__, __LINE__);
	ret = ske_hp_wait_till_done(WAIT_TILL_OUTPUT_READY);
	if (ret != SKE_SUCCESS)
		return ret;

	// printk("%s:%d", __func__, __LINE__);
	ske_hp_simple_get_output_block((uint32_t *)ctx->buf, ctx->block_words);
	if (ctx->crypto == SKE_CRYPTO_ENCRYPT) {
		// printk("%s:%d", __func__, __LINE__);
		memcpy(mac, ctx->buf, ctx->mac_bytes);
		ret = SKE_SUCCESS;
	} else
		ret = memcmp(mac, ctx->buf, ctx->mac_bytes);
	// printk("%s:%d", __func__, __LINE__);
	return ret;
}

void ske_get_version(void __iomem *io_base, u32 *major, u32 *minor)
{
	*major = (readl_relaxed(io_base + SKE_VERSION) & 0xf0) >> 4;
	*minor = readl_relaxed(io_base + SKE_VERSION) & 0x0f;
}

int bst_ske_setkey(struct crypto_skcipher *tfm, const u8 *key,
				   unsigned int keylen)
{
	struct ske_ctx *actx = crypto_skcipher_ctx(tfm);

	if (keylen != DES_KEY_SIZE     &&
		keylen != AES_KEYSIZE_128 &&
		keylen != AES_KEYSIZE_192 &&
		keylen != AES_KEYSIZE_256 &&
		keylen != AES_KEYSIZE_128 * 2 &&
		keylen != AES_KEYSIZE_192 * 2 &&
		keylen != AES_KEYSIZE_256 * 2)
		return -EINVAL;

	actx->key_len = keylen;
	memcpy(actx->key, key, keylen);

	return 0;
}

static uint32_t ske_crypt(struct ske_ctx *ctx, const void *src,
						  void *dst, u32 len, u8 *iv)
{
	uint32_t ret;

	ret = ske_hp_init(ctx, iv);
	if (ret != SKE_SUCCESS)
		return ret;

	return ske_hp_update_including_last_2_blocks(ctx, (uint8_t *)src, dst, len);
}

int bst_ske_crypt(struct skcipher_request *req)
{
	struct crypto_async_request *arq = &req->base;
	struct ske_ctx *ctx = crypto_tfm_ctx(arq->tfm);
	struct skcipher_walk walk;
	unsigned int nbytes, block_size;
	int err;
	int loop = 0;

	walk.flags = 0;
	err = skcipher_walk_virt(&walk, req, false);
	block_size = (ctx->alg == SKE_ALG_DES)? DES_BLOCK_SIZE : AES_BLOCK_SIZE;
	while ((nbytes = walk.nbytes) != 0) {
		ske_crypt(ctx, walk.src.virt.addr, walk.dst.virt.addr,
				  round_down(nbytes, block_size), walk.iv);
		err = skcipher_walk_done(&walk, nbytes % block_size);
		if (loop++ > 10000) {
			pr_info("%s failed, ske_crypt loop too long\n", __func__);
			break;
		}
	}

	return err;
}

int bst_ske_crypt1(struct skcipher_request *req)
{
	struct crypto_async_request *arq = &req->base;
	struct ske_ctx *ctx = crypto_tfm_ctx(arq->tfm);

	ctx->c_bytes = req->cryptlen;
	return ske_crypt(ctx, sg_virt(req->src), sg_virt(req->dst),
					 req->cryptlen, req->iv);
}

int bst_ske_gcm_crypt(struct aead_request *req)
{
	struct crypto_async_request *arq = &(req->base);
	struct ske_ctx *ctx = crypto_tfm_ctx(arq->tfm);
	uint8_t *in_buf = sg_virt(req->src);
	uint8_t *out_buf = sg_virt(req->dst);
	uint8_t *mac_buf;
	uint32_t ret;

	// printk("%s:%d cryptlen=%d, assoclen=%d\n", __func__, __LINE__, req->cryptlen, req->assoclen);
	// printk("%s:%d in:", __func__, __LINE__);
	// print_buf_u32((uint32_t *)in_buf, req->assoclen/4 + req->cryptlen/4 - ctx->mac_bytes/4);
	// printk("%s:%d key:", __func__, __LINE__);
	// print_buf_u32((uint32_t *)ctx->key, ctx->key_len/4);
	// printk("%s:%d iv:", __func__, __LINE__);
	// print_buf_u32((uint32_t *)req->iv, 12/4);
	// printk("%s:%d add:", __func__, __LINE__);
	// print_buf_u32((uint32_t *)in_buf, req->assoclen/4);
	/*
	 * encrypt
	 *   req->src: aad||plaintext
	 *   req->dst: aad||ciphertext||tag
	 * decrypt
	 *   req->src: aad||ciphertext||tag
	 *   req->dst: aad||plaintext, return 0 or -EBADMSG
	 * aad, plaintext and ciphertext may be empty.
	 */
	ctx->aad_bytes = req->assoclen;
	ctx->mac_bytes = crypto_aead_authsize(crypto_aead_reqtfm(req));

	if (ctx->crypto == SKE_CRYPTO_ENCRYPT) {
		ctx->c_bytes = req->cryptlen;
		mac_buf = out_buf + ctx->c_bytes + ctx->aad_bytes;
	} else {
		ctx->c_bytes = req->cryptlen - ctx->mac_bytes;
		mac_buf = in_buf + ctx->c_bytes + ctx->aad_bytes;
	}

	ret = ske_hp_init(ctx, req->iv);
	if (ret != SKE_SUCCESS)
		return ret;
	// printk("%s:%d", __func__, __LINE__);
	ret = ske_hp_gcm_aad(ctx, in_buf);
	if (ret != SKE_SUCCESS)
		return ret;

	// printk("%s:%d", __func__, __LINE__);
	ret = ske_hp_gcm_update_blocks(ctx, (in_buf + ctx->aad_bytes), (out_buf + ctx->aad_bytes), ctx->c_bytes);
	if (ret != SKE_SUCCESS)
		return ret;

	// printk("%s:%d pt_buf:", __func__, __LINE__);
	// print_buf_u32((uint32_t *)pt_buf, (req->cryptlen + 3)/4);
	// printk("%s:%d mac_buf:", __func__, __LINE__);
	// print_buf_u32((uint32_t *)mac_buf, 4);

	return ske_hp_gcm_final(ctx, mac_buf);
}

void ske_hp_ccm_get_B0(uint8_t *nonce, uint8_t M, uint8_t L, uint32_t aad_bytes, uint32_t c_bytes, uint8_t out[16])
{
	uint8_t tmp[4];

	// B0 flag
	out[0] = 0;
	out[0] |= (M - 2) / 2;
	out[0] <<= 3;
	out[0] |= L - 1;

	if (aad_bytes)
		out[0] |= 0x40; // with aad flag
	// B0 nonce
	if (nonce != out + 1) {
		memcpy(out + 1, nonce, 15 - L);
		memset(out + 1 + 15 - L, 0, L);
	}
	// B0 message byte length
#ifdef SKE_HP_CPU_BIG_ENDIAN
	memcpy(tmp, &c_bytes, 4);
#else
	reverse_byte_array((uint8_t *)(&c_bytes), tmp, 4);
#endif

	if (L <= 4)
		memcpy(out + 16 - L, tmp + 4 - L, L);
	else
		memcpy(out + 16 - 4, tmp, 4);
}

void ske_hp_ccm_pre_B1(struct ske_ctx *ctx)
{
	uint8_t tmp[4];
	uint32_t left_bytes;

#ifdef SKE_HP_CPU_BIG_ENDIAN
	memcpy(tmp, &ctx->aad_bytes, 4);
#else
	reverse_byte_array((uint8_t *)(&ctx->aad_bytes), tmp, 4);
#endif

	if (ctx->aad_bytes < ((1 << 16) - (1 << 8))) {
		memcpy(ctx->buf, tmp + 2, 2);
		ctx->current_bytes = 2;
		left_bytes = 16 - 2;
	} else {
		ctx->buf[0] = 0xFF;
		ctx->buf[1] = 0xFE;
		memcpy(ctx->buf + 2, tmp, 4);
		ctx->current_bytes = 6;
		left_bytes = 16 - 6;
	}

	ctx->b1_aad_start_offset = ctx->current_bytes;
	if (ctx->aad_bytes < left_bytes) {
		ctx->b1_aad_end_offset = ctx->b1_aad_start_offset + ctx->aad_bytes;
		memset(ctx->buf + ctx->b1_aad_end_offset, 0, 16 - ctx->b1_aad_end_offset);
	} else
		ctx->b1_aad_end_offset = 16;
}

void ske_hp_ccm_get_B1(uint8_t *aad, uint32_t aad_bytes, uint32_t *aad_offset, uint8_t out[16])
{
	uint8_t tmp[4];
	uint32_t current_bytes, left_bytes;

#ifdef SKE_HP_CPU_BIG_ENDIAN
	memcpy(tmp, &aad_bytes, 4);
#else
	reverse_byte_array((uint8_t *)(&aad_bytes), tmp, 4);
#endif

	if (aad_bytes < ((1 << 16) - (1 << 8))) {
		memcpy(out, tmp + 2, 2);
		current_bytes = 2;
		left_bytes = 16 - 2;
	} else {
		out[0] = 0xFF;
		out[1] = 0xFE;
		memcpy(out + 2, tmp, 4);
		current_bytes = 6;
		left_bytes = 16 - 6;
	}

	if (aad_bytes < left_bytes) {
		memcpy(out + current_bytes, aad, aad_bytes);
		memset(out + current_bytes + aad_bytes, 0, left_bytes - aad_bytes);
		*aad_offset = aad_bytes;
	} else {
		memcpy(out + current_bytes, aad, left_bytes);
		*aad_offset = left_bytes;
	}
}

uint32_t ske_hp_ccm_update_aad(struct ske_ctx *ctx, uint8_t *aad, uint32_t bytes)
{
	uint32_t blocks_bytes, remainder_bytes;
	uint32_t total_bytes, idx, capacity_bytes;
	uint32_t ret;

	if (ctx == NULL || (aad == NULL && ctx->aad_bytes != 0))
		return SKE_BUFFER_NULL;
	else if ((ctx->aad_bytes == 0) || (aad == NULL) || (bytes == 0))
		return SKE_SUCCESS;

	// now bytes is not 0

	if (ctx->current_bytes < ctx->b1_aad_end_offset) {
		remainder_bytes = ctx->b1_aad_end_offset - ctx->current_bytes;
		if (bytes >= remainder_bytes) {
			memcpy(ctx->buf + ctx->current_bytes, aad, remainder_bytes);
			aad += remainder_bytes;
			bytes -= remainder_bytes;
			ctx->current_bytes += remainder_bytes;

			if (ctx->current_bytes == ctx->aad_bytes + ctx->b1_aad_start_offset)
				ske_hp_set_last_block(1); // last block;

			ret = ske_hp_update_blocks_no_output(ctx, ctx->buf, ctx->block_bytes);
			if (ret != SKE_SUCCESS)
				return ret;

			if (ctx->current_bytes == ctx->aad_bytes + ctx->b1_aad_start_offset) {
				ske_hp_set_last_block(0); // not last block;
				ctx->current_bytes = 0;
				return SKE_SUCCESS; // aad input finished
			}
		} else {
			memcpy(ctx->buf + ctx->current_bytes, aad, bytes);
			ctx->current_bytes += bytes;
			return SKE_SUCCESS;
		}
	}

	// now bytes is not 0
	if (bytes == 0)
		return SKE_SUCCESS;

	/******** input B2,B3... ********/
	idx = ctx->current_bytes & 0x0F;
	capacity_bytes = 16 - idx;

	total_bytes = ctx->current_bytes + bytes;
	if (total_bytes < bytes || total_bytes > (ctx->b1_aad_start_offset + ctx->aad_bytes))
		return SKE_INPUT_INVALID;
	else if (total_bytes == (ctx->b1_aad_start_offset + ctx->aad_bytes)) {
		if (idx) {
			if (bytes > capacity_bytes) {
				memcpy(ctx->buf + idx, aad, capacity_bytes);
				ret = ske_hp_update_blocks_no_output(ctx, ctx->buf, 16);
				if (ret != SKE_SUCCESS)
					return ret;

				aad += capacity_bytes;
				bytes -= capacity_bytes;
			} else {
				// the last block
				memcpy(ctx->buf + idx, aad, bytes);
				memset(ctx->buf + idx + bytes, 0, sizeof(ctx->buf) - (idx + bytes));
				goto ccm_update_aad_last_block;
			}
		}

		blocks_bytes = (bytes) & (~0x0F); // assume that ctx->ske_ccm_ctx->block_bytes is 16
		remainder_bytes = (bytes)&0x0F;
		if (remainder_bytes == 0) {
			blocks_bytes -= 16;
			remainder_bytes = 16;
		}

		ret = ske_hp_update_blocks_no_output(ctx, aad, blocks_bytes);
		if (ret != SKE_SUCCESS)
			return ret;

		memcpy(ctx->buf, aad + blocks_bytes, remainder_bytes);
		if (remainder_bytes < 16) {
			memset(ctx->buf + remainder_bytes, 0, ctx->block_bytes - remainder_bytes);
		}

ccm_update_aad_last_block:
		ske_hp_set_last_block(1); // last block
		ret = ske_hp_update_blocks_no_output(ctx, ctx->buf, ctx->block_bytes);
		ske_hp_set_last_block(0); // not last block
		if (ret != SKE_SUCCESS)
			return ret;

		ctx->current_bytes = 0;
	} else {
		ctx->current_bytes = total_bytes;

		if (idx) {
			if (bytes >= capacity_bytes) {
				memcpy(ctx->buf + idx, aad, capacity_bytes);
				ret = ske_hp_update_blocks_no_output(ctx, ctx->buf, 16);
				if (ret != SKE_SUCCESS)
					return ret;

				aad += capacity_bytes;
				bytes -= capacity_bytes;
			} else {
				memcpy(ctx->buf + idx, aad, bytes);
				ret = SKE_SUCCESS;
				goto ccm_update_aad_end;
			}
		}

		blocks_bytes = (bytes) & (~0x0F);
		remainder_bytes = (bytes)&0x0F;

		ret = ske_hp_update_blocks_no_output(ctx, aad, blocks_bytes);
		if (ret != SKE_SUCCESS)
			return ret;

		if (remainder_bytes)
			memcpy(ctx->buf, aad + blocks_bytes, remainder_bytes);
	}

ccm_update_aad_end:

	return SKE_SUCCESS;
}

uint32_t ske_hp_ccm_aad(struct ske_ctx *ctx, uint8_t *aad)
{
#ifndef SKE_HP_CCM_CPU_UPDATE_AAD_BY_STEP
	uint32_t blocks_bytes, remainder_bytes;
	uint32_t aad_bytes, aad_offset;
	uint32_t ret;

	if (ctx == NULL || (aad == NULL && ctx->aad_bytes != 0))
		return SKE_BUFFER_NULL;
	else if ((aad == NULL) || (ctx->aad_bytes == 0))
		return SKE_SUCCESS;

	// now aad is not NULL, and ctx->aad_bytes is not 0

	// input B1,B2...
	aad_bytes = ctx->aad_bytes;

	/******** get and input B1 ********/
	ske_hp_ccm_get_B1(aad, aad_bytes, &aad_offset, ctx->buf);

	aad_bytes -= aad_offset;
	aad += aad_offset;
	if (aad_bytes == 0)
		ske_hp_set_last_block(1); // last block;

	ret = ske_hp_update_blocks_no_output(ctx, ctx->buf, ctx->block_bytes);
	if (ret != SKE_SUCCESS)
		return ret;

	if (aad_bytes == 0) {
		ske_hp_set_last_block(0); // not last block
		ctx->current_bytes = 0;
		return SKE_SUCCESS;
	}

	/******** input B2,B3... ********/
	blocks_bytes = (aad_bytes) & (~0x0F); // assume that ctx->ske_ccm_ctx->block_bytes is 16
	remainder_bytes = (aad_bytes)&0x0F;
	if (remainder_bytes == 0) {
		blocks_bytes -= 16;
		remainder_bytes = 16;
	}

	ret = ske_hp_update_blocks_no_output(ctx, aad, blocks_bytes);
	if (ret != SKE_SUCCESS)
		return ret;

	memcpy(ctx->buf, aad + blocks_bytes, remainder_bytes);
	memset(ctx->buf + remainder_bytes, 0, ctx->block_bytes - remainder_bytes);
	ske_hp_set_last_block(1); // last block
	ret = ske_hp_update_blocks_no_output(ctx, ctx->buf, ctx->block_bytes);
	ske_hp_set_last_block(0); // not last block
	if (ret != SKE_SUCCESS)
		return ret;

	ctx->current_bytes = 0;

	return SKE_SUCCESS;
#else
	if (ctx == NULL)
		return SKE_BUFFER_NULL;
	else
		return ske_hp_ccm_update_aad(ctx, aad, ctx->aad_bytes);
#endif
}

uint32_t ske_hp_ccm_update_blocks(struct ske_ctx *ctx, uint8_t *in, uint8_t *out, uint32_t bytes)
{
	uint32_t blocks_bytes, remainder_bytes;
	uint32_t total_bytes;
	uint32_t ret;

	if (ctx == NULL || in == NULL || out == NULL)
		return SKE_BUFFER_NULL;
	else if (bytes == 0)
		return SKE_SUCCESS;

	// now bytes is not 0
	total_bytes = ctx->current_bytes + bytes;
	if (total_bytes < bytes || total_bytes > ctx->c_bytes)
		return SKE_INPUT_INVALID;
	else if (total_bytes == ctx->c_bytes) {
		blocks_bytes = bytes & (~0x0F);
		remainder_bytes = bytes & 0x0F;
		if (remainder_bytes == 0) {
			blocks_bytes -= 16;
			remainder_bytes = 16;
		}

		ret = ske_hp_update_blocks_internal(ctx, in, out, blocks_bytes);
		if (ret != SKE_SUCCESS)
			goto ccm_update_blocks_end;

		// the last block
		memcpy(ctx->buf, in + blocks_bytes, remainder_bytes);
		memset(ctx->buf + remainder_bytes, 0, sizeof(ctx->buf) - remainder_bytes);

		ske_hp_set_last_block(1);
		ret = ske_hp_update_blocks_internal(ctx, ctx->buf, ctx->buf, 16);
		ske_hp_set_last_block(0);
		if (ret != SKE_SUCCESS)
			goto ccm_update_blocks_end;

		memcpy(out + blocks_bytes, ctx->buf, remainder_bytes);
	} else {
		if (bytes & (16 - 1)) {
			ret = SKE_INPUT_INVALID;
			goto ccm_update_blocks_end;
		} else {
			ret = ske_hp_update_blocks_internal(ctx, in, out, bytes);
			if (ret != SKE_SUCCESS)
				goto ccm_update_blocks_end;
		}
	}

	ret = SKE_SUCCESS;
	ctx->current_bytes = total_bytes;

ccm_update_blocks_end:

	return ret;
}

uint32_t ske_hp_ccm_final(struct ske_ctx *ctx, uint8_t *mac)
{
	uint32_t ret;

	if (ctx == NULL || mac  == NULL)
		return SKE_BUFFER_NULL;

	// get mac
	ret = ske_hp_wait_till_done(WAIT_TILL_OUTPUT_READY);
	if (ret != SKE_SUCCESS)
		return ret;

	ske_hp_simple_get_output_block((uint32_t *)ctx->buf, ctx->block_words);

	if (ctx->crypto == SKE_CRYPTO_ENCRYPT) {
		memcpy(mac, ctx->buf, ctx->M);
		ret = SKE_SUCCESS;
	} else
		ret = memcmp(mac, ctx->buf, ctx->M);

	memset(ctx, 0, sizeof(struct ske_ctx));

	return ret;
}

/* taken from crypto/ccm.c */
static inline int crypto_ccm_check_iv(const u8 *iv)
{
	/* 2 <= L <= 8, so 1 <= L' <= 7. */
	if (iv[0] < 1 || iv[0] > 7)
		return -EINVAL;

	return 0;
}

uint32_t ske_hp_ccm_pre_init(struct ske_ctx *ctx, uint8_t *nonce)
{
	uint32_t tmp, len;

	if (NULL == ctx || NULL == nonce)
		return SKE_BUFFER_NULL;

	// check c_bytes
	tmp = ctx->c_bytes;
	len = 0;
	while (tmp) {
		len++;
		tmp >>= 8;
	}

	if (len > ctx->L)
		return SKE_INPUT_INVALID;

	// A0
	ctx->buf[0] = (ctx->L) - 1;
	memcpy(ctx->buf + 1, nonce, 15 - (ctx->L));
	memset(ctx->buf + 16 - ctx->L, 0, ctx->L);

	// ske_hp_set_gcm_aad_len_uint32(aad_bytes);   //for CPU and DMA mode, this action is different.

	// ske_hp_set_c_len_uint32(c_bytes);

	ctx->current_bytes = 0;

	return SKE_SUCCESS;
}

int bst_ske_ccm_crypt(struct aead_request *req)
{
	struct crypto_async_request *arq = &(req->base);
	struct ske_ctx *ctx = crypto_tfm_ctx(arq->tfm);
	uint8_t *in_buf = sg_virt(req->src);
	uint8_t *out_buf = sg_virt(req->dst);
	uint8_t *mac_buf;
	int ret;

	// printk("%s:%d cryptlen=%d, assoclen=%d\n", __func__, __LINE__, req->cryptlen, req->assoclen);
	// printk("%s:%d add:", __func__, __LINE__);
	// print_buf_u32((uint32_t *)in_buf, req->assoclen/4 + req->cryptlen/4 +  ctx->mac_bytes/4);
	// printk("%s:%d key:", __func__, __LINE__);
	// print_buf_u32((uint32_t *)ctx->key, ctx->key_len/4);
	// printk("%s:%d iv:", __func__, __LINE__);
	// print_buf_u32((uint32_t *)req->iv, 16/4);
	// printk("%s:%d add:", __func__, __LINE__);
	// print_buf_u32((uint32_t *)add_buf, req->assoclen/4);

	ret = crypto_ccm_check_iv(req->iv);
	if (ret)
		return ret;
	ctx->aad_bytes = req->assoclen;
	ctx->L = req->iv[0] + 1;
	ctx->mac_bytes = crypto_aead_authsize(crypto_aead_reqtfm(req));
	if (ctx->crypto == SKE_CRYPTO_ENCRYPT) {
		ctx->c_bytes = req->cryptlen;
		mac_buf = out_buf + ctx->c_bytes + ctx->aad_bytes;
	} else {
		ctx->c_bytes = req->cryptlen - ctx->mac_bytes;
		mac_buf = in_buf + ctx->c_bytes + ctx->aad_bytes;
	}
	// ctx->L = 2;
	// printk("%s:%d L=%d\n", __func__, __LINE__, ctx->L);
	// printk("%s:%d mac_buf:", __func__, __LINE__);
	// print_buf_u32((uint32_t *)mac_buf, 4);
	ctx->M = crypto_aead_authsize(crypto_aead_reqtfm(req));

	ret = ske_hp_ccm_pre_init(ctx, (req->iv + 1));
	if (ret != SKE_SUCCESS)
		return ret;

	// printk("%s:%d buf a0:", __func__, __LINE__);
	// print_buf_u32((uint32_t *)ctx->buf, 16/4);
	ret = ske_hp_init(ctx, ctx->buf);
	if (ret != SKE_SUCCESS)
		return ret;

	// get and input B0
	ske_hp_ccm_get_B0((req->iv + 1), ctx->M, ctx->L, ctx->aad_bytes, ctx->c_bytes, ctx->buf);
	if (ctx->aad_bytes == 0)
		ske_hp_set_last_block(1); // last block;

	// printk("%s:%d buf:", __func__, __LINE__);
	// print_buf_u32((uint32_t *)ctx->buf, 16/4);
	ret = ske_hp_update_blocks_no_output(ctx, ctx->buf, ctx->block_bytes);
	if (ret != SKE_SUCCESS)
		return ret;

	if (ctx->aad_bytes == 0)
		ske_hp_set_last_block(0); // not last block

	// prepare B1
	if (ctx->aad_bytes != 0)
		ske_hp_ccm_pre_B1(ctx);

	// printk("%s:%d", __func__, __LINE__);
	ret = ske_hp_ccm_aad(ctx, in_buf);
	if (ret != SKE_SUCCESS)
		return ret;

	// printk("%s:%d", __func__, __LINE__);
	ret = ske_hp_ccm_update_blocks(ctx, (in_buf + ctx->aad_bytes), (out_buf + ctx->aad_bytes), ctx->c_bytes);
	if (ret != SKE_SUCCESS)
		return ret;

	// printk("%s:%d out_buf:", __func__, __LINE__);
	// print_buf_u32((uint32_t *)(out_buf + ctx->aad_bytes), (req->cryptlen + 3)/4);
	// printk("%s:%d mac_buf2:", __func__, __LINE__);
	// print_buf_u32((uint32_t *)mac_buf, 4);
	// printk("%s:%d", __func__, __LINE__);
	return ske_hp_ccm_final(ctx, mac_buf);
}

int bst_ske_encrypt(struct skcipher_request *req)
{
	struct crypto_async_request *arq = &req->base;
	struct ske_ctx *ctx = crypto_tfm_ctx(arq->tfm);

	ctx->crypto = SKE_CRYPTO_ENCRYPT;
	return bst_ske_crypt(req);
}

int bst_ske_decrypt(struct skcipher_request *req)
{
	struct crypto_async_request *arq = &req->base;
	struct ske_ctx *ctx = crypto_tfm_ctx(arq->tfm);

	ctx->crypto = SKE_CRYPTO_DECRYPT;

	return bst_ske_crypt(req);
}

int bst_ske_init(struct crypto_skcipher *tfm, enum ske_alg alg, enum ske_mode mode)
{
	struct ske_ctx *ctx = crypto_skcipher_ctx(tfm);

	ctx->alg = alg;
	ctx->mode = mode;

	return 0;
}

int bst_ske_gcm_encrypt(struct aead_request *req)
{
	struct crypto_async_request *arq = &req->base;
	struct ske_ctx *ctx = crypto_tfm_ctx(arq->tfm);

	ctx->crypto = SKE_CRYPTO_ENCRYPT;

	return bst_ske_gcm_crypt(req);
}

int bst_ske_gcm_decrypt(struct aead_request *req)
{
	struct crypto_async_request *arq = &req->base;
	struct ske_ctx *ctx = crypto_tfm_ctx(arq->tfm);

	ctx->crypto = SKE_CRYPTO_DECRYPT;

	return bst_ske_gcm_crypt(req);
}

int bst_ske_aead_init(struct crypto_aead *tfm, enum ske_alg alg, enum ske_mode mode)
{
	struct ske_ctx *ctx = crypto_aead_ctx(tfm);

	ctx->alg = alg;
	ctx->mode = mode;

	return 0;
}

static int bst_ske_gcm_setauthsize(struct crypto_aead *tfm,
								   unsigned int authsize)
{
	switch (authsize) {
	case 4:
	case 8:
	case 12:
	case 13:
	case 14:
	case 15:
	case 16:
		break;
	default:
		return -EINVAL;
	}
	// printk("%s:%d authsize=%d", __func__, __LINE__, authsize);
	return 0;
}

int bst_ske_gcm_setkey(struct crypto_aead *tfm, const u8 *key,
					   unsigned int keylen)
{
	struct ske_ctx *actx = crypto_aead_ctx(tfm);
	// printk("%s:%d keylen=%d", __func__, __LINE__, keylen);
	if (keylen != AES_KEYSIZE_128 &&
		keylen != AES_KEYSIZE_192 &&
		keylen != AES_KEYSIZE_256 &&
		keylen != AES_KEYSIZE_128 * 2 &&
		keylen != AES_KEYSIZE_192 * 2 &&
		keylen != AES_KEYSIZE_256 * 2)
		return -EINVAL;
	// printk("%s:%d", __func__, __LINE__);
	actx->key_len = keylen;
	memcpy(actx->key, key, keylen);

	return 0;
}

int bst_ske_ccm_encrypt(struct aead_request *req)
{
	struct crypto_async_request *arq = &req->base;
	struct ske_ctx *ctx = crypto_tfm_ctx(arq->tfm);

	ctx->crypto = SKE_CRYPTO_ENCRYPT;

	return bst_ske_ccm_crypt(req);
}

int bst_ske_ccm_decrypt(struct aead_request *req)
{
	struct crypto_async_request *arq = &req->base;
	struct ske_ctx *ctx = crypto_tfm_ctx(arq->tfm);

	ctx->crypto = SKE_CRYPTO_DECRYPT;

	return bst_ske_ccm_crypt(req);
}

static int bst_ske_ccm_setauthsize(struct crypto_aead *tfm,
								   unsigned int authsize)
{
	// struct ske_ctx *actx = crypto_aead_ctx(tfm);

	switch (authsize) {
	case 4:
	case 6:
	case 8:
	case 10:
	case 12:
	case 14:
	case 16:
		break;
	default:
		return -EINVAL;
	}
	// printk("%s:%d authsize=%d", __func__, __LINE__, authsize);
	return 0;
}

int bst_ske_ccm_setkey(struct crypto_aead *tfm, const u8 *key,
					   unsigned int keylen)
{
	struct ske_ctx *actx = crypto_aead_ctx(tfm);
	// printk("%s:%d keylen=%d", __func__, __LINE__, keylen);
	if (keylen != AES_KEYSIZE_128 &&
		keylen != AES_KEYSIZE_192 &&
		keylen != AES_KEYSIZE_256 &&
		keylen != AES_KEYSIZE_128 * 2 &&
		keylen != AES_KEYSIZE_192 * 2 &&
		keylen != AES_KEYSIZE_256 * 2)
		return -EINVAL;
	// printk("%s:%d", __func__, __LINE__);
	actx->key_len = keylen;
	memcpy(actx->key, key, keylen);

	return 0;
}
int bst_aes_128_gcm_init(struct crypto_aead *tfm)
{
	// printk("%s:%d", __func__, __LINE__);
	return bst_ske_aead_init(tfm, SKE_ALG_AES_128, SKE_MODE_GCM);
}
int bst_aes_128_ccm_init(struct crypto_aead *tfm)
{
	// printk("%s:%d", __func__, __LINE__);
	return bst_ske_aead_init(tfm, SKE_ALG_AES_128, SKE_MODE_CCM);
}
static struct aead_alg bst_algs_aead[] = {
	{
		.base = {
			.cra_name = "bst_gcm_aes_128",
			.cra_driver_name = "bst-gcm-aes",
			.cra_priority = 300,
			.cra_blocksize = 1,
			.cra_ctxsize = sizeof(struct ske_ctx),
			.cra_alignmask = 0xf,
			.cra_module = THIS_MODULE,
		},
		.init = bst_aes_128_gcm_init,
		.ivsize = GCM_AES_IV_SIZE,
		.maxauthsize = AES_BLOCK_SIZE,
		.setkey = bst_ske_gcm_setkey,
		.setauthsize = bst_ske_gcm_setauthsize,
		.encrypt = bst_ske_gcm_encrypt,
		.decrypt = bst_ske_gcm_decrypt,
	},
	{
		.base = {
			.cra_name = "bst_ccm_aes_128",
			.cra_driver_name = "bst-ccm-aes",
			.cra_priority = 300,
			.cra_blocksize = 1,
			.cra_ctxsize = sizeof(struct ske_ctx),
			.cra_alignmask = 0xf,
			.cra_module = THIS_MODULE,
		},
		.init = bst_aes_128_ccm_init,
		.ivsize = AES_BLOCK_SIZE,
		.maxauthsize = AES_BLOCK_SIZE,
		.setkey = bst_ske_ccm_setkey,
		.setauthsize = bst_ske_ccm_setauthsize,
		.encrypt = bst_ske_ccm_encrypt,
		.decrypt = bst_ske_ccm_decrypt,
	},
};
/* AES 256*/
int bst_ske_aes_ecb_256_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_AES_256, SKE_MODE_ECB);
}

int bst_ske_aes_cbc_256_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_AES_256, SKE_MODE_CBC);
}

int bst_ske_aes_cfb_256_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_AES_256, SKE_MODE_CFB);
}

int bst_ske_aes_ofb_256_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_AES_256, SKE_MODE_OFB);
}

int bst_ske_aes_ctr_256_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_AES_256, SKE_MODE_CTR);
}

int bst_ske_aes_xts_256_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_AES_256, SKE_MODE_XTS);
}

/* AES 192*/
int bst_ske_aes_ecb_192_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_AES_192, SKE_MODE_ECB);
}
int bst_ske_aes_xts_192_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_AES_192, SKE_MODE_XTS);
}
int bst_ske_aes_cbc_192_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_AES_192, SKE_MODE_CBC);
}
int bst_ske_aes_cfb_192_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_AES_192, SKE_MODE_CFB);
}
int bst_ske_aes_ofb_192_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_AES_192, SKE_MODE_OFB);
}
int bst_ske_aes_ctr_192_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_AES_192, SKE_MODE_CTR);
}

/* AES 128*/
int bst_ske_aes_ecb_128_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_AES_128, SKE_MODE_ECB);
}
int bst_ske_aes_xts_128_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_AES_128, SKE_MODE_XTS);
}
int bst_ske_aes_cbc_128_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_AES_128, SKE_MODE_CBC);
}
int bst_ske_aes_cfb_128_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_AES_128, SKE_MODE_CFB);
}
int bst_ske_aes_ofb_128_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_AES_128, SKE_MODE_OFB);
}
int bst_ske_aes_ctr_128_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_AES_128, SKE_MODE_CTR);
}

/* SM4*/
int bst_ske_sm4_ecb_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_SM4, SKE_MODE_ECB);
}
int bst_ske_sm4_xts_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_SM4, SKE_MODE_XTS);
}
int bst_ske_sm4_cbc_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_SM4, SKE_MODE_CBC);
}
int bst_ske_sm4_cfb_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_SM4, SKE_MODE_CFB);
}
int bst_ske_sm4_ofb_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_SM4, SKE_MODE_OFB);
}
int bst_ske_sm4_ctr_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_SM4, SKE_MODE_CTR);
}
int bst_ske_des_ecb_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_DES, SKE_MODE_ECB);
}
int bst_ske_des_cbc_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_DES, SKE_MODE_CBC);
}
int bst_ske_des_cfb_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_DES, SKE_MODE_CFB);
}
int bst_ske_des_ofb_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_DES, SKE_MODE_OFB);
}
int bst_ske_des_ctr_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_DES, SKE_MODE_CTR);
}

static struct skcipher_alg bst_ske_algs[] = {
	/* AES 256*/
	{
		.base.cra_name = "bst_ecb_aes_256",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_ecb_256_init,
		.exit = NULL,
	},
	{
		.base.cra_name = "bst_xts_aes_256",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE * 2,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_xts_256_init,
		.exit = NULL,
	},
	{
		.base.cra_name = "bst_cbc_aes_256",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_cbc_256_init,
		.exit = NULL,
	},
	{
		.base.cra_name = "bst_cfb_aes_256",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_cfb_256_init,
		.exit = NULL,
	},
	{
		.base.cra_name = "bst_ofb_aes_256",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_ofb_256_init,
		.exit = NULL,
	},
	{
		.base.cra_name = "bst_ctr_aes_256",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_ctr_256_init,
		.exit = NULL,
	},
	/* AES 192*/
	{
		.base.cra_name = "bst_ecb_aes_192",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_ecb_192_init,
		.exit = NULL,
	},
	{
		.base.cra_name = "bst_xts_aes_192",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE * 2,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_xts_192_init,
		.exit = NULL,
	},
	{
		.base.cra_name = "bst_cbc_aes_192",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_cbc_192_init,
		.exit = NULL,
	},
	{
		.base.cra_name = "bst_cfb_aes_192",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_cfb_192_init,
		.exit = NULL,
	},
	{
		.base.cra_name = "bst_ofb_aes_192",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_ofb_192_init,
		.exit = NULL,
	},
	{
		.base.cra_name = "bst_ctr_aes_192",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_ctr_192_init,
		.exit = NULL,
	},
	/* AES 128*/
	{
		.base.cra_name = "bst_ecb_aes_128",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_ecb_128_init,
		.exit = NULL,
	},
	{
		.base.cra_name = "bst_xts_aes_128",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE * 2,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_xts_128_init,
		.exit = NULL,
	},
	{
		.base.cra_name = "bst_cbc_aes_128",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_cbc_128_init,
		.exit = NULL,
	},
	{
		.base.cra_name = "bst_cfb_aes_128",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_cfb_128_init,
		.exit = NULL,
	},
	{
		.base.cra_name = "bst_ofb_aes_128",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_ofb_128_init,
		.exit = NULL,
	},
	{
		.base.cra_name = "bst_ctr_aes_128",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_ctr_128_init,
		.exit = NULL,
	},
	/*SM4*/
	{
		.base.cra_name = "bst_cbc_sm4",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = SM4_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = SM4_KEY_SIZE,
		.max_keysize = SM4_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = SM4_BLOCK_SIZE,
		.init = bst_ske_sm4_cbc_init,
		.exit = NULL,
	},
	{
		.base.cra_name = "bst_ecb_sm4",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = SM4_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = SM4_KEY_SIZE,
		.max_keysize = SM4_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = SM4_BLOCK_SIZE,
		.init = bst_ske_sm4_ecb_init,
		.exit = NULL,
	},
	{
		.base.cra_name = "bst_ctr_sm4",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = SM4_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = SM4_KEY_SIZE,
		.max_keysize = SM4_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = SM4_BLOCK_SIZE,
		.init = bst_ske_sm4_ctr_init,
		.exit = NULL,
	},
	{
		.base.cra_name = "bst_cfb_sm4",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = SM4_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = SM4_KEY_SIZE,
		.max_keysize = SM4_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = SM4_BLOCK_SIZE,
		.init = bst_ske_sm4_cfb_init,
		.exit = NULL,
	},
	{
		.base.cra_name = "bst_ofb_sm4",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = SM4_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = SM4_KEY_SIZE,
		.max_keysize = SM4_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = SM4_BLOCK_SIZE,
		.init = bst_ske_sm4_ofb_init,
		.exit = NULL,
	},
	{
		.base.cra_name = "bst_xts_sm4",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = SM4_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = SM4_KEY_SIZE,
		.max_keysize = SM4_KEY_SIZE * 2,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = SM4_BLOCK_SIZE,
		.init = bst_ske_sm4_xts_init,
		.exit = NULL,
	},
	/*DES*/
	{
		.base.cra_name = "bst_ecb_des",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = DES_BLOCK_SIZE*2,
		.init = bst_ske_des_ecb_init,
		.exit = NULL,
	},
	{
		.base.cra_name = "bst_cbc_des",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = DES_BLOCK_SIZE*2,
		.init = bst_ske_des_cbc_init,
		.exit = NULL,
	},
	{
		.base.cra_name = "bst_cfb_des",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = DES_BLOCK_SIZE*2,
		.init = bst_ske_des_cfb_init,
		.exit = NULL,
	},
	{
		.base.cra_name = "bst_ofb_des",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = DES_BLOCK_SIZE*2,
		.init = bst_ske_des_ofb_init,
		.exit = NULL,
	},
	{
		.base.cra_name = "bst_ctr_des",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = DES_BLOCK_SIZE*2,
		.init = bst_ske_des_ctr_init,
		.exit = NULL,
	}
};

static int bst_ske_irq_handler(struct ske *dev)
{
	return 0;
}

static irqreturn_t bst_ske_irq(int irq, void *dev_id)
{
	struct ske *hdev = dev_id;
	u32 stat, enabled;

	enabled = readl_relaxed(hdev->base + SKE_IMCR);
	stat = readl_relaxed(hdev->base + SKE_MISR);

	dev_dbg(hdev->dev, "enabled=%#x stat=%#x\n", enabled, stat);
	if (!enabled || !stat)
		return IRQ_NONE;

	bst_ske_irq_handler(hdev);
	writel_relaxed(readl_relaxed(hdev->base + SKE_RISR) & (~1),
				   hdev->base + SKE_RISR);

	return IRQ_HANDLED;
}

static int bst_ske_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ske *ske = NULL;
	int i, ret;
	u32 v_major, v_minor;

	ske = devm_kzalloc(dev, sizeof(*ske), GFP_KERNEL);
	if (!ske)
		return -ENOMEM;

	ske->dev = dev;
	ske->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ske->base))
		return PTR_ERR(ske->base);

	ske->irq = platform_get_irq(pdev, 0);
	if (ske->irq < 0) {
		if (ske->irq != -EPROBE_DEFER)
			dev_err(dev, "cannot get irq\n");
		return ske->irq;
	}

	ret = devm_request_irq(ske->dev, ske->irq, bst_ske_irq, IRQF_SHARED,
						   dev_name(ske->dev), ske);
	if (ret) {
		dev_err(ske->dev, "failure requesting irq %i: %d\n",
				ske->irq, ret);
		return ret;
	}
	/* Allocate coherent helper block. */
	ske->coh = devm_kzalloc(dev, sizeof(*ske->coh) + SKE_ALIGNMENT,
							GFP_KERNEL);
	if (!ske->coh)
		return -ENOMEM;

	/* Re-align the structure so it fits the DCP constraints. */
	ske->coh = PTR_ALIGN(ske->coh, SKE_ALIGNMENT);
	global_ske = ske;
	platform_set_drvdata(pdev, ske);
	for (i = 0; i < SKE_MAX_CHANS; i++) {
		spin_lock_init(&ske->lock[i]);
		init_completion(&ske->completion[i]);
		crypto_init_queue(&ske->queue[i], 50);
	}

	ret = crypto_register_skciphers(bst_ske_algs,
									ARRAY_SIZE(bst_ske_algs));
	if (ret) {
		/* Failed to register algorithm. */
		dev_err(dev, "Failed to register ske crypto.\n");
		goto err_unregister_skciphers;
	}

	ret = crypto_register_aeads(bst_algs_aead, ARRAY_SIZE(bst_algs_aead));
	if (ret) {
		dev_err(dev, "Failed to register aeads\n");
		goto err_unregister_aeads;
	}
	ske_get_version(ske->base, &v_major, &v_minor);
	dev_info(dev, "Hardware version: v%d.%d\n", v_major, v_minor);

	return 0;
err_unregister_aeads:
	crypto_unregister_aeads(bst_algs_aead, ARRAY_SIZE(bst_algs_aead));
err_unregister_skciphers:
	crypto_unregister_skciphers(bst_ske_algs, ARRAY_SIZE(bst_ske_algs));

	return ret;
}

static int bst_ske_remove(struct platform_device *pdev)
{
	struct ske *ske = platform_get_drvdata(pdev);

	crypto_unregister_skciphers(bst_ske_algs, ARRAY_SIZE(bst_ske_algs));
	kthread_stop(ske->thread[0]);
	platform_set_drvdata(pdev, NULL);
	global_ske = NULL;

	return 0;
}

static const struct of_device_id bst_ske_match[] = {
	{.compatible = "bst,c1200-ske"},
	{}};
MODULE_DEVICE_TABLE(of, bst_hash_match);

static struct platform_driver bst_ske_driver = {
	.probe = bst_ske_probe,
	.remove = bst_ske_remove,
	.driver = {
		.name = "bst-ske",
		.of_match_table = of_match_ptr(bst_ske_match),
	}};
module_platform_driver(bst_ske_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("BST Symmetric Key Engine driver");
