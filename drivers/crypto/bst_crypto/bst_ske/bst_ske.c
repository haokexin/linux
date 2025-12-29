// SPDX-License-Identifier: GPL-2.0
#include <linux/dma-mapping.h>
#include <linux/of_reserved_mem.h>
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
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/string.h>
#include "bst_ske.h"
#include "../common/bst_sa_common.h"


static unsigned int refcnt = 0;
static DEFINE_MUTEX(refcnt_lock);
#define BST_IV_UPDATE 1
static DEFINE_MUTEX(op_mutex);
static ktime_t last_op_lock_time;
#define MAX_OP_LOCK_MS 50
#define OPUPDATETIME {UpdateOPTime();}
#define OPTRYLOCK { CheckOPTimeout(); \
					if (!mutex_trylock(&op_mutex)) { \
						bst_dbg(1, "ske opmutex is busy, cannot acquire.\n"); \
						return -EBUSY; \
					}else{\
						OPUPDATETIME \
					}}

#define OPUNLOCK {if (mutex_is_locked(&op_mutex)) {\
					mutex_unlock(&op_mutex);\
				}}



#define SKE_MAX_CHANS 4
#define SKE_BUF_SZ PAGE_SIZE
#define SKE_ALIGNMENT 64
#define SKE_ADDR(offset) (global_ske->base + offset)
#define CMA_ADDR_OFFSET  (0x800000000 - 0x80000000)

#if 0
#define write_reg  writel
#define read_reg   readl
#else
#define write_reg  writel_relaxed
#define read_reg   readl_relaxed
#endif

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
	uint8_t ivInited;
	uint8_t iv[16];
	uint8_t block_bytes;
	uint8_t block_words;
	uint8_t left_bytes;
	enum ske_alg alg;
	enum ske_mode mode;
	enum ske_crypto crypto;
	enum ske_mac mac;
	uint8_t buf[16];
	uint32_t c_bytes;
	uint32_t current_bytes;
	uint32_t aad_bytes;
	uint32_t mac_bytes;
	uint8_t b1_aad_start_offset;
	uint8_t b1_aad_end_offset;
	uint8_t M;
	uint8_t L;
	struct dma_alloc_addr dma_addr;
	uint8_t hasMutex;
};

static struct ske *global_ske = NULL;
static void __attribute__((unused)) print_buf_u32(uint32_t buf[], uint32_t word_len)
{
	uint32_t i;

	for (i = 0; i < word_len; i++)
		bst_dbg(1, "%08x", buf[i]);

	bst_dbg(1, "\r\n");
}

static void __attribute__((unused)) CheckOPTimeout(void){
	ktime_t now = ktime_get();
	s64 delta_ms = ktime_to_ms(ktime_sub(now, last_op_lock_time));
	
	bst_dbg(1, "CheckOPTimeout %lld", delta_ms);
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


static void bst_ske_exit(struct crypto_skcipher *tfm)
{
    struct ske_ctx *ctx = crypto_skcipher_ctx(tfm);

    if (ctx->hasMutex != 0){
        ctx->hasMutex = 0;
	}

	//OPUNLOCK
}


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

static void ske_enable_interruption(enum ske_hp_mode irq_mode)
{
	uint32_t flag = (uint32_t)1;

	if (SKE_HP_CPU_MODE == irq_mode) {
		write_reg(read_reg(SKE_ADDR(SKE_IMCR)) | flag,
				   SKE_ADDR(SKE_IMCR));
	} else if (SKE_HP_DMA_MODE == irq_mode) {
		write_reg(read_reg(SKE_ADDR(SKE_IMCR)) | (flag << 1),
				   SKE_ADDR(SKE_IMCR));
	}
}

static void ske_disable_interruption(void)
{
	uint32_t mask = ~((uint32_t)3);

	write_reg(read_reg(SKE_ADDR(SKE_IMCR)) & mask,
					SKE_ADDR(SKE_IMCR));
}

uint8_t __attribute__((unused)) ske_hp_get_block_byte_len(enum ske_alg ske_alg)
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

void ske_hp_set_cpu_mode(enum ske_hp_mode hp_mode)
{
	if (SKE_HP_CPU_MODE == hp_mode) {
		write_reg(read_reg(SKE_ADDR(SKE_CFG)) & (~((uint32_t)1 << SKE_HP_DMA_OFFSET)), SKE_ADDR(SKE_CFG));
	} else if (SKE_HP_DMA_MODE == hp_mode) {
		write_reg(read_reg(SKE_ADDR(SKE_CFG)) | ((uint32_t)1 << SKE_HP_DMA_OFFSET), SKE_ADDR(SKE_CFG));
		/* disable ske_hp DMA linked list */
		write_reg(read_reg(SKE_ADDR(SKE_CFG)) & (~((uint32_t)1 << SKE_HP_DMA_LL_OFFSET)), SKE_ADDR(SKE_CFG));
	}
}

void ske_hp_set_endian_uint32(uint32_t endian)
{
	uint32_t mask;
	uint32_t flag;

	mask = ~(((uint32_t)3) << SKE_HP_REVERSE_BYTE_ORDER_IN_WORD_OFFSET);
	flag = (((uint32_t)2) << SKE_HP_REVERSE_BYTE_ORDER_IN_WORD_OFFSET);
	write_reg(read_reg(SKE_ADDR(SKE_CFG)) & mask, SKE_ADDR(SKE_CFG));
	if (!endian)
		write_reg(read_reg(SKE_ADDR(SKE_CFG)) | flag,
					   SKE_ADDR(SKE_CFG));
}

unsigned int get_alg_blksize(enum ske_alg alg)
{
	switch (alg) {
		case SKE_ALG_AES_128:
		case SKE_ALG_AES_192:
		case SKE_ALG_AES_256:
			return AES_BLOCK_SIZE;
		case SKE_ALG_SM4:
			return SM4_BLOCK_SIZE;
		case SKE_ALG_DES:
			return DES_BLOCK_SIZE;
		case SKE_ALG_TDES_128:
		case SKE_ALG_TDES_192:
			return DES3_EDE_BLOCK_SIZE;
		default:
			return AES_BLOCK_SIZE;
	}
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

	write_reg(read_reg(SKE_ADDR(SKE_CFG)) & mask, SKE_ADDR(SKE_CFG));
	write_reg(read_reg(SKE_ADDR(SKE_CFG)) | cfg, SKE_ADDR(SKE_CFG));
}

void ske_hp_set_mode(enum ske_mode mode)
{
	uint32_t mask = ~(0x0000000FU << SKE_HP_MODE_OFFSET);
	uint32_t cfg = (((uint32_t)mode) << SKE_HP_MODE_OFFSET);

	write_reg(read_reg(SKE_ADDR(SKE_CFG)) & mask, SKE_ADDR(SKE_CFG));
	write_reg(read_reg(SKE_ADDR(SKE_CFG)) | cfg, SKE_ADDR(SKE_CFG));
}

void ske_hp_set_crypto(enum ske_crypto crypto)
{
	uint32_t mask = ~(((uint32_t)1) << SKE_HP_CRYPTO_OFFSET);
	uint32_t cfg = (((uint32_t)crypto) << SKE_HP_CRYPTO_OFFSET);

	write_reg(read_reg(SKE_ADDR(SKE_CFG)) & mask, SKE_ADDR(SKE_CFG));
	write_reg(read_reg(SKE_ADDR(SKE_CFG)) | cfg, SKE_ADDR(SKE_CFG));
}

void ske_hp_set_last_block(uint32_t is_last_block)
{
	uint32_t flag = (((uint32_t)1) << SKE_HP_LAST_DATA_OFFSET);
	uint32_t mask = ~(((uint32_t)1) << SKE_HP_LAST_DATA_OFFSET);

	if (is_last_block)
		write_reg(read_reg(SKE_ADDR(SKE_DIN_CR)) | flag,
					   SKE_ADDR(SKE_DIN_CR));
	else
		write_reg(read_reg(SKE_ADDR(SKE_DIN_CR)) & mask,
					   SKE_ADDR(SKE_DIN_CR));
}

void ske_hp_set_last_block_len(uint32_t bytes)
{
	uint32_t mask = ~0x000000FFU;

	write_reg(read_reg(SKE_ADDR(SKE_DIN_CR)) & mask, SKE_ADDR(SKE_DIN_CR));
	write_reg(read_reg(SKE_ADDR(SKE_DIN_CR)) | (bytes << 3), SKE_ADDR(SKE_DIN_CR));
}

void ske_hp_set_iv_uint32(uint32_t *iv, uint32_t block_words)
{
	int32_t i;

	for (i = block_words; i > 0; i--)
		write_reg(iv[block_words - i], SKE_ADDR(SKE_IV + (i - 1) * 4));
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

// 读出硬件 SKE_IV 寄存器到 uint32_t 数组
void ske_hp_get_iv_uint32(uint32_t *iv, uint32_t block_words)
{
    int32_t i;

    for (i = block_words; i > 0; i--) {
        iv[block_words - i] = read_reg(SKE_ADDR(SKE_IV + (i - 1) * 4));
    }
}

// 处理字节对齐的读取接口
void ske_hp_get_iv(uint8_t *out, uint32_t block_bytes)
{
    uint32_t tmp[4];
	uint8_t iv[16];
    ske_hp_get_iv_uint32(tmp, block_bytes / 4);

    if (((uint64_t)iv) & 3) {
        memcpy(iv, tmp, block_bytes);
    } else {
        *(uint32_t *)iv = tmp[0];
        if (block_bytes > 4) ((uint32_t *)iv)[1] = tmp[1];
        if (block_bytes > 8) ((uint32_t *)iv)[2] = tmp[2];
        if (block_bytes > 12) ((uint32_t *)iv)[3] = tmp[3];
    }
	printHex("read iv from reg", iv, block_bytes);
	memcpy(out, iv, block_bytes);
}



#if BST_IV_UPDATE == 1




static inline void ske_xor_block(u8 *dst, const u8 *a, const u8 *b, size_t n)
{
    size_t i;
    for (i = 0; i < n; i++)
        dst[i] = a[i] ^ b[i];
}

/*
 * Add big-endian integer n to IV buffer
 * iv_out and iv_in can point to the same buffer
 */
static inline void ske_add_big_endian_iv(u8 *iv_out, const u8 *iv_in,
                                         u64 n, size_t block_bytes)
{
    u64 carry = n;
    int i;

    // if (iv_out != iv_in)
    //     memcpy(iv_out, iv_in, block_bytes);

    for (i = block_bytes - 1; i >= 0 && carry; i--) {
        u64 sum = (u64)iv_out[i] + (carry & 0xffULL);
        iv_out[i] = (u8)sum;
        carry = (carry >> 8) + (sum >> 8);
    }
}

/*
 * CTR模式IV自增函数，适配多数国产安全引擎
 * 默认假设counter在IV的最后4字节，小端字节序
 */
static inline void ske_ctr_inc_iv(u8 *iv, size_t iv_bytes)
{
    int i;
    for (i = iv_bytes - 1; i >= (int)(iv_bytes - 4); i--) {
        if (++iv[i] != 0)
            break;
    }
}


enum ske_ctr_mode {
    CTR_COUNTER_AT_END_LE,  /* little endian, counter in last 4 bytes */
    CTR_COUNTER_AT_END_BE,  /* big endian, counter in last 4 bytes */
    CTR_COUNTER_FULL_BE,    /* full 128-bit big endian counter */
};

static inline void ske_add_ctr_iv(u8 *iv_out, const u8 *iv_in,
                                  size_t block_bytes, size_t blocks,
                                  enum ske_ctr_mode layout)
{
    int i, j;
    memcpy(iv_out, iv_in, block_bytes);

    for (i = 0; i < blocks; i++) {
        switch (layout) {
        case CTR_COUNTER_AT_END_LE:
            for (j = block_bytes - 1; j >= (int)(block_bytes - 4); j--)
                if (++iv_out[j] != 0)
                    break;
            break;
        case CTR_COUNTER_AT_END_BE:
            for (j = block_bytes - 1; j >= (int)(block_bytes - 4); j--)
                if (++iv_out[j] != 0)
                    break;
            break;
        case CTR_COUNTER_FULL_BE:
            for (j = block_bytes - 1; j >= 0; j--)
                if (++iv_out[j] != 0)
                    break;
            break;
        }
    }
}


/*
 * Compute next IV after an AES/SM4 operation.
 * mode     : cipher mode (CBC/CFB/OFB/CTR)
 * encrypt  : true if encryption, false if decryption
 * cur_iv   : IV used in this operation
 * iv_out   : buffer (block_bytes) to store next IV
 * in_buf   : input buffer (plaintext for encrypt, ciphertext for decrypt)
 * in_len   : length of in_buf
 * out_buf  : output buffer (ciphertext for encrypt, plaintext for decrypt)
 * out_len  : length of out_buf
 * block_bytes : block size (16 for AES/SM4, 8 for DES)
 *
 * Returns 0 on success.
 */
static inline int ske_compute_next_iv(enum ske_mode mode, bool encrypt,
                                      u8 *iv_out,size_t block_bytes,
                                      const u8 *in_buf, size_t in_len,
                                      const u8 *out_buf, size_t out_len)
{
	
	u8 cur_iv[16];
    if (!iv_out || !block_bytes)
        return -EINVAL;
    memcpy(cur_iv, iv_out, block_bytes);

    switch (mode) {
    case SKE_MODE_CBC:
    case SKE_MODE_CFB: {
        const u8 *cipher_src = encrypt ? out_buf : in_buf;
        size_t cipher_len = encrypt ? out_len : in_len;

        if (!cipher_src || cipher_len < block_bytes)
            return 0;

        memcpy(iv_out,
               cipher_src + ((cipher_len / block_bytes) * block_bytes - block_bytes),
               block_bytes);
        return 0;
    }

    case SKE_MODE_OFB: {
        size_t proc = min(in_len, out_len);
        if (!in_buf || !out_buf || proc < block_bytes)
            return 0;

        ske_xor_block(iv_out,
                      out_buf + ((proc / block_bytes) * block_bytes - block_bytes),
                      in_buf + ((proc / block_bytes) * block_bytes - block_bytes),
                      block_bytes);
        return 0;
    }

    case SKE_MODE_CTR: {
#if 1
        size_t proc = (out_len > in_len) ? out_len : in_len;
        size_t blocks = proc / block_bytes;
        if (!blocks)
            return 0;

        ske_add_big_endian_iv(iv_out, cur_iv, (u64)blocks, block_bytes);
        return 0;
#else
		size_t blocks = (out_len + block_bytes - 1) / block_bytes;
		size_t i;

		if (!blocks)
			return 0;

		memcpy(iv_out, cur_iv, block_bytes);
		for (i = 0; i < blocks; i++)
			ske_ctr_inc_iv(iv_out, block_bytes);
		return 0;
#endif
    }

    default:
        return -EINVAL;
    }

    return 0;
}

#endif /* BST_IV_UPDATE */



void ske_hp_disable_secure_port(void)
{
	uint32_t mask = ~1;

	write_reg(read_reg(SKE_ADDR(SKE_SP)) & mask, SKE_ADDR(SKE_SP));
}

void clear_block_tail(uint32_t in[4], uint32_t bytes)
{
	uint32_t i;

	i = bytes / 4;
	bytes &= 3;

	if(bytes)
	{
		in[i] &= 0xFFFFFFFF<<(32 - bytes * 8);
		i++;
	}
	while(i < 4)
	{
		in[i++]=0;
	}
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
		write_reg(key[key_words - i], key_reg + i - 1);
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
	wmb();
	writel(clear_flag, SKE_ADDR(SKE_RISR));
	writel(read_reg(SKE_ADDR(SKE_CTRL)) | start_flag,
				   SKE_ADDR(SKE_CTRL));
}

void ske_hp_set_c_len_uint32(uint32_t c_bytes)
{
	write_reg(((c_bytes) << 3) & 0xFFFFFFFF, SKE_ADDR(SKE_C_LEN_L));
	write_reg(c_bytes >> (32 - 3), SKE_ADDR(SKE_C_LEN_H));
}

void ske_hp_set_aad_len_uint32(uint32_t aad_bytes)
{
	write_reg(((aad_bytes) << 3) & 0xFFFFFFFF, SKE_ADDR(SKE_A_LEN_L));
	write_reg(aad_bytes >> (32 - 3), SKE_ADDR(SKE_A_LEN_H));
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

	while (!(read_reg(reg_status) & finish_flag)) {
		if (read_reg(SKE_ADDR(SKE_ALARM)) & alarm_flag)
			return SKE_ATTACK_ALARM;
	}

	return SKE_SUCCESS;
}

uint32_t ske_hp_dma_calc_wait_till_done(SKE_CALLBACK callback)
{
	volatile uint32_t finish_flag = 7;
	volatile uint32_t alarm_flag = 1;

	while(!(read_reg(SKE_ADDR(SKE_RISR)) & finish_flag)) {
		if(read_reg(SKE_ADDR(SKE_ALARM)) & alarm_flag) {
			return SKE_ATTACK_ALARM;
		} else if (callback) {
			callback();
		}
	}

	return SKE_SUCCESS;
}

uint32_t ske_hp_expand_key(void)
{
	uint32_t mask = ~(((uint32_t)1) << SKE_HP_UP_CFG_OFFSET);
	uint32_t flag = (((uint32_t)1) << SKE_HP_UP_CFG_OFFSET);
	uint32_t ret;

	write_reg(read_reg(SKE_ADDR(SKE_CFG)) | flag, SKE_ADDR(SKE_CFG));
	ske_hp_start();
	ret = ske_hp_wait_till_done(WAIT_TILL_EXPAND_KEY_DONE);
	if (ret != SKE_SUCCESS)
		return ret;

	write_reg(read_reg(SKE_ADDR(SKE_CFG)) & mask, SKE_ADDR(SKE_CFG));

	return SKE_SUCCESS;
}

void ske_dma_callback(void)
{

}

static uint32_t __attribute__((unused)) ske_hp_init_clen_or_key(struct ske_ctx *ctx, enum ske_hp_mode hp_mode)
{
	uint32_t key_bytes;

	if (ctx->mode == SKE_MODE_XTS) {
		ske_hp_set_c_len_uint32(ctx->c_bytes);
		ctx->current_bytes = 0;
	} else if (ctx->mode == SKE_MODE_GCM || ctx->mode == SKE_MODE_CCM) {
		ske_hp_set_c_len_uint32(ctx->c_bytes);
		ske_hp_set_aad_len_uint32(ctx->aad_bytes);
	}else{
		return SKE_SUCCESS;
	}

	key_bytes = ske_hp_get_key_byte_len(ctx->alg);

	if (ctx->mode == SKE_MODE_XTS){
		ske_hp_set_key(ctx->alg, ctx->key + key_bytes, key_bytes, 2);
	}

	return ske_hp_expand_key();;
}

uint32_t ske_hp_init(struct ske_ctx *ctx, uint8_t *iv, enum ske_hp_mode hp_mode)
{
	uint32_t key_bytes;
#if BST_IV_UPDATE == 0
	if(ctx->ivInited == 1){
		return ske_hp_init_clen_or_key(ctx, hp_mode);
	}
#endif
	if (ctx->mode == SKE_MODE_ECB)
		iv = NULL;
	else if (iv == NULL)
		return SKE_BUFFER_NULL;

	switch(ctx->alg){
		case SKE_ALG_AES:
			if(ctx->key_len == 16){
				ctx->alg = SKE_ALG_AES_128;
			}else if(ctx->key_len == 24){
				ctx->alg = SKE_ALG_AES_192;
			}else if(ctx->key_len == 32){
				ctx->alg = SKE_ALG_AES_256;
			}
			break;
		case SKE_ALG_TDES:
			if(ctx->key_len == 16){
				ctx->alg = SKE_ALG_TDES_128;
			}else if(ctx->key_len == 24){
				ctx->alg = SKE_ALG_TDES_192;
			}
			break;
		default:
			break;
	}
	bst_dbg(1, "init ctx:%p %s alg:%d mode:%d\n", ctx, ctx->crypto == SKE_CRYPTO_ENCRYPT ? "crypt enc alg" : "crypt dec alg",ctx->alg,ctx->mode);

	ctx->block_bytes = get_alg_blksize(ctx->alg);
	ctx->block_words = ctx->block_bytes / 4;
	if (ctx->mode == SKE_MODE_XTS) {
		ske_hp_set_c_len_uint32(ctx->c_bytes);
		ctx->current_bytes = 0;
	} else if (ctx->mode == SKE_MODE_GCM || ctx->mode == SKE_MODE_CCM) {
		ske_hp_set_c_len_uint32(ctx->c_bytes);
		ske_hp_set_aad_len_uint32(ctx->aad_bytes);
	} else {
		ske_hp_set_c_len_uint32(0);
	}

	ske_hp_set_cpu_mode(hp_mode);
	ske_hp_set_endian_uint32(0);
	ske_hp_set_alg(ctx->alg);
	ske_hp_set_mode(ctx->mode);
	ske_hp_set_crypto(ctx->crypto);
	ske_hp_set_last_block(0);
	ske_enable_interruption(hp_mode);
	if(ctx->ivInited == 1){
		ske_hp_set_iv(ctx->iv, ctx->block_bytes);
		printHex(ctx->crypto == SKE_CRYPTO_ENCRYPT ? "crypt 1 enc iv" : "crypt dec iv",ctx->iv, ctx->block_bytes);
	}else if (iv != NULL){
		ske_hp_set_iv(iv, ctx->block_bytes);
		memcpy(ctx->iv, iv, ctx->block_bytes);
		printHex(ctx->crypto == SKE_CRYPTO_ENCRYPT ? "crypt 0 enc iv" : "crypt dec iv",ctx->iv, ctx->block_bytes);
	}

	ske_hp_disable_secure_port();
	key_bytes = ske_hp_get_key_byte_len(ctx->alg);
	ske_hp_set_key(ctx->alg, ctx->key, key_bytes, 1);
	if (ctx->mode == SKE_MODE_XTS)
		ske_hp_set_key(ctx->alg, ctx->key + key_bytes, key_bytes, 2);
	if(ctx->ivInited == 0){
		ctx->ivInited = 1;
	}
	return ske_hp_expand_key();
}

uint32_t ske_hp_cmac_init(struct ske_ctx *ctx)
{
	uint32_t iv[4] = {0};
	ctx->left_bytes = 0;
	return ske_hp_init(ctx, (uint8_t *)iv, SKE_HP_CPU_MODE);
}

uint32_t ske_hp_cbc_mac_init(struct ske_ctx *ctx)
{
	uint32_t iv[4] = {0};
	ctx->left_bytes = 0;
	return ske_hp_init(ctx, (uint8_t *)iv, SKE_HP_CPU_MODE);
}

uint32_t ske_hp_dma_cmac_init(struct ske_ctx *ctx)
{
	uint32_t iv[4] = {0};
	return ske_hp_init(ctx, (uint8_t *)iv, SKE_HP_DMA_MODE);
}

uint32_t ske_hp_dma_cbc_mac_init(struct ske_ctx *ctx)
{
	uint32_t iv[4] = {0};
	return ske_hp_init(ctx, (uint8_t *)iv, SKE_HP_DMA_MODE);
}

void ske_hp_simple_set_input_block(uint32_t *in, uint32_t block_words)
{
	int32_t i;

	for (i = block_words; i > 0; i--)
		write_reg(in[block_words - i], SKE_ADDR(SKE_DIN + (i - 1) * 4));
}

void ske_hp_simple_get_output_block(uint32_t *out, uint32_t block_words)
{
	uint32_t flag = 0x02;
	int32_t i;

	write_reg(read_reg(SKE_ADDR(SKE_CTRL)) | flag,
				   SKE_ADDR(SKE_CTRL));
	for (i = block_words; i > 0; i--)
		out[block_words - i] = read_reg(SKE_ADDR(SKE_DOUT) + (i - 1) * 4);
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
				write_reg(((uint32_t *)in)[0], SKE_ADDR(SKE_DIN + 3 * 4));
				write_reg(((uint32_t *)in)[1], SKE_ADDR(SKE_DIN + 2 * 4));
				write_reg(((uint32_t *)in)[2], SKE_ADDR(SKE_DIN + 1 * 4));
				write_reg(((uint32_t *)in)[3], SKE_ADDR(SKE_DIN + 0 * 4));
				write_reg(flag_0, SKE_ADDR(SKE_RISR));
				write_reg(flag_1, SKE_ADDR(SKE_CTRL));
				ret = ske_hp_wait_till_done(WAIT_TILL_OUTPUT_READY);
				if (ret != SKE_SUCCESS) {
					memset(out_bak, 0, bytes);
					return ret;
				}

				write_reg(flag_2, SKE_ADDR(SKE_CTRL));
				((uint32_t *)out)[0] = read_reg(SKE_ADDR(SKE_DOUT + 3 * 4));
				((uint32_t *)out)[1] = read_reg(SKE_ADDR(SKE_DOUT + 2 * 4));
				((uint32_t *)out)[2] = read_reg(SKE_ADDR(SKE_DOUT + 1 * 4));
				((uint32_t *)out)[3] = read_reg(SKE_ADDR(SKE_DOUT + 0 * 4));
				in += block_bytes;
				out += block_bytes;
			}
		} else {
			for (i = 0; i < round; i++) {
				write_reg(((uint32_t *)in)[0], SKE_ADDR(SKE_DIN + 1 * 4));
				write_reg(((uint32_t *)in)[1], SKE_ADDR(SKE_DIN + 0 * 4));
				write_reg(flag_0, SKE_ADDR(SKE_RISR));
				write_reg(flag_1, SKE_ADDR(SKE_CTRL));
				ret = ske_hp_wait_till_done(WAIT_TILL_OUTPUT_READY);
				if (ret != SKE_SUCCESS) {
					memset(out_bak, 0, bytes);
					return ret;
				}

				write_reg(flag_2, SKE_ADDR(SKE_CTRL));
				((uint32_t *)out)[0] = read_reg(SKE_ADDR(SKE_DOUT + 1 * 4));
				((uint32_t *)out)[1] = read_reg(SKE_ADDR(SKE_DOUT + 0 * 4));
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

void dma_readl(uint32_t *data, uint32_t *addr, unsigned int len32)
{
	unsigned int i;
	for (i = 0; i < len32; i++) {
		data[i] = read_reg(addr + i);
	}
}

void dma_writel(uint32_t *addr, const uint32_t *data, unsigned int len32)
{
	unsigned int i;
	for (i = 0; i < len32; i++) {
		write_reg(data[i], addr + i);
	}
}

uint32_t dma_cmp(uint32_t *data, uint32_t *addr, unsigned int len32)
{
	unsigned int i;
	for (i = 0; i < len32; i++) {
		if (data[i] != read_reg(addr + i)) {
			return 1;
		}
	}
	return 0;
}

uint32_t ske_hp_dma_operate(const uint64_t in_addr, const uint64_t out_addr, uint32_t in_words, uint32_t out_words,
										SKE_CALLBACK callback)
{
	volatile uint32_t flag_0 = 0;

	if(0 == in_addr)
		return SKE_BUFFER_NULL;
	else if (0 == in_words)
		return SKE_SUCCESS;

	//src & dst addr low 32bits
	write_reg((uint32_t)(in_addr & 0xFFFFFFFF), SKE_ADDR(SKE_DMA_L_SADDR));
	write_reg((uint32_t)(out_addr & 0xFFFFFFFF), SKE_ADDR(SKE_DMA_L_DADDR));

	//src & dst addr high 32bits
	if(4 == (sizeof(uint32_t *))) {
		//in this case, if using (((uint64_t)in)>>32), you may get 0xFFFFFFFF, not 0 you expected!
		write_reg((uint32_t)flag_0, SKE_ADDR(SKE_DMA_H_SADDR));
		write_reg((uint32_t)flag_0, SKE_ADDR(SKE_DMA_H_DADDR));
	} else {
		write_reg((uint32_t)(in_addr >> 32), SKE_ADDR(SKE_DMA_H_SADDR));
		write_reg((uint32_t)(out_addr >> 32), SKE_ADDR(SKE_DMA_H_DADDR));
	}

	//data bit length
	write_reg(in_words << 5, SKE_ADDR(SKE_DMA_RLEN));
	write_reg(out_words << 5, SKE_ADDR(SKE_DMA_WLEN));

	ske_hp_start();

	return ske_hp_dma_calc_wait_till_done(callback);
}

uint32_t ske_hp_dma_update_blocks(struct ske_ctx *ctx, const uint32_t *in,
							  uint32_t *out, uint32_t words, SKE_CALLBACK callback)
{
	uint32_t ret;
	ret = SKE_ERROR;
	if ((in == NULL) || (out == NULL))
		return SKE_BUFFER_NULL;
	else if(0 == words)
		return SKE_SUCCESS;
	else if(words & (ctx->block_words - 1))
		return SKE_INPUT_INVALID;

	dma_writel((uint32_t *)ctx->dma_addr.virt_in, in, words);
	ret = ske_hp_dma_operate((ctx->dma_addr.phys_in - CMA_ADDR_OFFSET), (ctx->dma_addr.phys_out - CMA_ADDR_OFFSET),
							words, words, ske_dma_callback);
	if (ret == SKE_SUCCESS)
		dma_readl(out, (uint32_t *)ctx->dma_addr.virt_out, words);
	else
		pr_info("ske_hp_dma_operate failed, ret: %d", ret);

	return ret;
}

uint32_t ske_hp_xts_update_blocks(struct ske_ctx *ctx, const uint8_t *in,
							  uint8_t *out, uint32_t bytes)
{
	uint32_t ret;
	uint32_t blocks_bytes, remainder_bytes;

	if(bytes & 0x0F) {
		blocks_bytes = bytes - AES_BLOCK_SIZE - (bytes & 0x0F);
		remainder_bytes = AES_BLOCK_SIZE + (bytes & 0x0F);
	} else {
		blocks_bytes = bytes;
		remainder_bytes = 0;
	}
	
	if ((in == NULL) || (out == NULL)) {
		return SKE_BUFFER_NULL;
	} else if (blocks_bytes & (ctx->block_bytes - 1)) {
		return SKE_INPUT_INVALID;
	} else if (ctx->current_bytes & (ctx->block_bytes - 1)) {
		return SKE_INPUT_INVALID;
	}

	if(ctx->c_bytes & 0x0F) {
		if(ctx->c_bytes - 16 - (ctx->c_bytes & 0x0F) < ctx->current_bytes + blocks_bytes) {
			return SKE_INPUT_INVALID;
		}
	} else {
		if(ctx->c_bytes < ctx->current_bytes + blocks_bytes) {
			return SKE_INPUT_INVALID;	
		}
	}
	ret = ske_hp_update_blocks_internal(ctx, in, out, blocks_bytes);
	if (ret != SKE_SUCCESS)
		return ret;
	ctx->current_bytes += blocks_bytes;

	if (remainder_bytes) {
		ret = ske_hp_update_including_last_2_blocks(ctx, (uint8_t *)(in + blocks_bytes), (uint8_t *)(out + blocks_bytes), remainder_bytes);
		if(SKE_SUCCESS != ret)
			return ret;
	}
	return SKE_SUCCESS;
}

uint32_t ske_hp_dma_xts_update_blocks(struct ske_ctx *ctx, const uint32_t *in,
							  uint32_t *out, SKE_CALLBACK callback)
{
	uint32_t ret;
	uint32_t words;
	uint32_t rest_bytes;
	if(NULL == ctx || NULL == in || NULL == out)
	{
		return SKE_BUFFER_NULL;
	}

	rest_bytes = ctx->c_bytes & 0x0F;
	if(rest_bytes) {
		words = (ctx->c_bytes + 15) / 16 * 4;
		if (ctx->c_bytes <= ctx->block_bytes)
			return SKE_INPUT_INVALID;
		else if (ctx->current_bytes & (ctx->block_bytes - 1) || ctx->current_bytes + ctx->c_bytes != ctx->c_bytes)
			return SKE_INPUT_INVALID;
		ske_hp_set_last_block(1);
		dma_writel((uint32_t *)ctx->dma_addr.virt_in, in, words);
		ret = ske_hp_dma_operate((ctx->dma_addr.phys_in - CMA_ADDR_OFFSET), (ctx->dma_addr.phys_out - CMA_ADDR_OFFSET),
								words, words, callback);
		if (ret == SKE_SUCCESS)
			dma_readl(out, (uint32_t *)ctx->dma_addr.virt_out, words);
		else
			pr_info("ske_hp_dma_operate failed, ret: %d", ret);
		clear_block_tail((uint32_t *)((uint8_t *)out + ctx->c_bytes - rest_bytes), rest_bytes);
	}
	else {
		words = ctx->c_bytes / 4;
		if (0 == words)
			return SKE_SUCCESS;
		if (words & (ctx->block_words - 1))
			return SKE_INPUT_INVALID;
		if (ctx->current_bytes & (ctx->block_bytes - 1))
			return SKE_INPUT_INVALID;
		dma_writel((uint32_t *)ctx->dma_addr.virt_in, in, words);
		ret = ske_hp_dma_operate((ctx->dma_addr.phys_in - CMA_ADDR_OFFSET), (ctx->dma_addr.phys_out - CMA_ADDR_OFFSET),
								words, words, callback);
		if (SKE_SUCCESS == ret) {
			dma_readl(out, (uint32_t *)ctx->dma_addr.virt_out, words);
			ctx->current_bytes += (words<<2);
		} else {
			pr_info("ske_hp_dma_operate failed, ret: %d", ret);
		}
	}

	return ret;
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
				write_reg(read_reg(SKE_ADDR(SKE_CTRL)) | flag_2, SKE_ADDR(SKE_CTRL));
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

	blocks_bytes = (ctx->aad_bytes) & (~0x0F);
	remainder_bytes = (ctx->aad_bytes) & 0x0F;

	if (remainder_bytes == 0) {
		blocks_bytes -= 16;
		remainder_bytes = 16;
	}

	ret = ske_hp_update_blocks_no_output(ctx, aad, blocks_bytes);
	if (ret != SKE_SUCCESS)
		return ret;
	// the last block
	memcpy(ctx->buf, aad + blocks_bytes, remainder_bytes);
	memset(ctx->buf + remainder_bytes, 0, sizeof(ctx->buf) - remainder_bytes);
	ske_hp_set_last_block(1);
	ret = ske_hp_update_blocks_no_output(ctx, ctx->buf, 16);
	ske_hp_set_last_block(0);
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

	ret = ske_hp_wait_till_done(WAIT_TILL_OUTPUT_READY);
	if (ret != SKE_SUCCESS)
		return ret;

	ske_hp_simple_get_output_block((uint32_t *)ctx->buf, ctx->block_words);
	if (ctx->crypto == SKE_CRYPTO_ENCRYPT) {
		memcpy(mac, ctx->buf, ctx->mac_bytes);
		ret = SKE_SUCCESS;
	} else
		ret = memcmp(mac, ctx->buf, ctx->mac_bytes);
	return ret;
}

uint32_t ske_hp_cmac_update(struct ske_ctx *ctx, uint8_t *msg, uint32_t msg_bytes)
{
	uint32_t blocks_bytes;
	uint32_t ret;
	uint8_t fill_bytes, remainder;

	if(NULL == ctx)
		return SKE_BUFFER_NULL;
	else if(NULL == msg || 0 == msg_bytes)
		return SKE_SUCCESS;

	//if one block left, process it
	if(ctx->block_bytes == ctx->left_bytes) {
		ret = ske_hp_update_blocks_no_output(ctx, ctx->buf, ctx->block_bytes);
		if(SKE_SUCCESS != ret)
			return ret;
		else
			ctx->left_bytes = 0;
	}

	//padding
	if(ctx->left_bytes) {
		fill_bytes = ctx->block_bytes - ctx->left_bytes;
		if(msg_bytes <= fill_bytes) {
			memcpy(ctx->buf + ctx->left_bytes, msg, msg_bytes);
			ctx->left_bytes += msg_bytes;
			return SKE_SUCCESS;
		} else {
			memcpy(ctx->buf + ctx->left_bytes, msg, fill_bytes);
			ret = ske_hp_update_blocks_no_output(ctx, ctx->buf, ctx->block_bytes);
			if(SKE_SUCCESS != ret) {
				return ret;
			} else {
				ctx->left_bytes = 0;
				msg += fill_bytes;
				msg_bytes -= fill_bytes;
			}
		}
	}

	//process some blocks
	blocks_bytes = (msg_bytes / ctx->block_bytes) * ctx->block_bytes;
	remainder = msg_bytes % ctx->block_bytes;

	//process remainder
	if(remainder) {
		ret = ske_hp_update_blocks_no_output(ctx, msg, blocks_bytes);
		if(SKE_SUCCESS != ret) {
			return ret;
		} else {
			memcpy(ctx->buf, msg + blocks_bytes, remainder);
			ctx->left_bytes = remainder;
		}
	} else {
		blocks_bytes -= ctx->block_bytes;
		ret = ske_hp_update_blocks_no_output(ctx, msg, blocks_bytes);
		if(SKE_SUCCESS != ret) {
			return ret;
		} else {
			memcpy(ctx->buf, msg + blocks_bytes, ctx->block_bytes);
			ctx->left_bytes = ctx->block_bytes;
		}
	}
	return ret;
}

uint32_t ske_hp_cbc_mac_update(struct ske_ctx *ctx, uint8_t *msg, uint32_t msg_bytes)
{
	uint32_t blocks_bytes;
	uint32_t ret;
	uint8_t fill_bytes, remainder;

	if(ctx->left_bytes) {
		fill_bytes = ctx->block_bytes - ctx->left_bytes;
		if(msg_bytes < fill_bytes) {
			memcpy(ctx->buf + ctx->left_bytes, msg, msg_bytes);
			ctx->left_bytes += msg_bytes;
			return SKE_SUCCESS;
		} else {
			memcpy(ctx->buf + ctx->left_bytes, msg, fill_bytes);
			ret = ske_hp_update_blocks_no_output(ctx, ctx->buf, ctx->block_bytes);
			if(SKE_SUCCESS != ret)
			{
				return ret;
			} else {
				ctx->left_bytes = 0;
				msg += fill_bytes;
				msg_bytes -= fill_bytes;
			}
		}
	}

	//update blocks
	blocks_bytes = (msg_bytes / ctx->block_bytes) * ctx->block_bytes;
	ret = ske_hp_update_blocks_no_output(ctx, msg, blocks_bytes);
	if(SKE_SUCCESS != ret)
		return ret;

	//hold the remainder
	remainder = msg_bytes % ctx->block_bytes;
	if(remainder) {
		memcpy(ctx->buf, msg + blocks_bytes, remainder);
		ctx->left_bytes = remainder;
	}

	return ret;
}

uint32_t ske_hp_cmac_final(struct ske_ctx *ctx, uint8_t *mac)
{
	uint32_t tmp[4];
	uint32_t ret;

	if((NULL == ctx) || (NULL == mac))
		return SKE_BUFFER_NULL;

	ske_hp_set_last_block(1);
	ske_hp_set_last_block_len(ctx->left_bytes);

	if(ctx->block_bytes == ctx->left_bytes) {
		ret = ske_hp_update_blocks_internal(ctx, ctx->buf, (uint8_t *)tmp, ctx->block_bytes);
		if(SKE_SUCCESS != ret)
			return ret;
	} else {
		ctx->buf[ctx->left_bytes] = 0x80;
		memset(ctx->buf + ctx->left_bytes + 1, 0, ctx->block_bytes - 1 - ctx->left_bytes);
		ret = ske_hp_update_blocks_internal(ctx, ctx->buf, (uint8_t *)tmp, ctx->block_bytes);
		if(SKE_SUCCESS != ret)
			return ret;
	}

	if(SKE_GENERATE_MAC == ctx->mac) {
		memcpy(mac, tmp, ctx->mac_bytes);
		ret = SKE_SUCCESS;
	} else if (SKE_VERIFY_MAC == ctx->mac) {
		ret = memcmp(mac, tmp, ctx->mac_bytes);
		pr_info("bst_cmac_%s verify ret:%d", ske_alg_info[ctx->alg], ret);
	}

	return ret;
}

uint32_t ske_hp_cbc_mac_final(struct ske_ctx *ctx, uint8_t *mac)
{
	uint32_t tmp[4];
	uint32_t ret;

	if(0 == ctx->left_bytes) {
		ske_hp_simple_get_output_block(tmp, ctx->block_words);
	} else {
		memset(ctx->buf + ctx->left_bytes, 0, ctx->block_bytes - ctx->left_bytes);
		ret = ske_hp_update_blocks_internal(ctx, ctx->buf, (uint8_t *)tmp, ctx->block_bytes);
		if(SKE_SUCCESS != ret)
			return ret;
	}

	if(SKE_GENERATE_MAC == ctx->mac) {
		memcpy(mac, tmp, ctx->mac_bytes);
		ret = SKE_SUCCESS;
	} else {
		ret = memcmp(mac, tmp, ctx->mac_bytes);
		pr_info("bst_cbc_mac_%s verify ret:%d", ske_alg_info[ctx->alg], ret);
	}

	return ret;
}

uint32_t ske_hp_dma_cmac_update_including_last_block(struct ske_ctx *ctx, uint32_t *msg, uint32_t msg_bytes, uint32_t *mac)
{
	uint32_t ret;
	uint32_t msg_words, block_words = 0;
	uint32_t block_bytes, remainder_bytes;

	if((NULL == ctx) || (NULL == msg) || (NULL == mac))
		return SKE_BUFFER_NULL;

	ske_hp_set_last_block(1);

	//get last block length and pad
	//padded by software, not hardware, do not delete this padding action
	if(0 == msg_bytes) {
		msg[0] = 0x80;
		msg[1] = 0;
		msg[2] = 0;
		msg[3] = 0;

		msg_bytes = 1;
		remainder_bytes = 1;
	} else if(msg_bytes & (ctx->block_bytes - 1)) {
		/* if the actual message length msg_bytes is not a multiple of block length, please make sure the last block
		   is padded with 0 already */
		memset(ctx->buf, 0, ctx->block_bytes);
		block_bytes = msg_bytes / ctx->block_bytes * ctx->block_bytes;
		block_words = block_bytes / 4;
		remainder_bytes = msg_bytes & (ctx->block_bytes - 1);
		memcpy(ctx->buf, (uint8_t *)msg + block_bytes, remainder_bytes);
		ctx->buf[remainder_bytes] |= 0x80;
	} else {
		remainder_bytes = ctx->block_bytes;
	}

	//set the last block message length
	ske_hp_set_last_block_len(remainder_bytes);

	msg_words = block_words + (ctx->block_words); // last padding
	if(ctx->dma_addr.virt_in != NULL){
		dmam_free_coherent(global_ske->dev, ctx->dma_addr.alloc_size, ctx->dma_addr.virt_in, ctx->dma_addr.phys_in);
		ctx->dma_addr.virt_in = NULL;
	}
	ctx->dma_addr.alloc_size = max((uint32_t)((msg_words * 4 + ctx->block_bytes)), (uint32_t)(2 * PAGE_SIZE));
	ctx->dma_addr.virt_in = dmam_alloc_coherent(global_ske->dev, ctx->dma_addr.alloc_size, &ctx->dma_addr.phys_in, GFP_KERNEL);
	if (ctx->dma_addr.virt_in == NULL) {
		pr_info("msg_bytes=%u, block_bytes=%u, block_words=%u, msg_words=%u, alloc_size=%u\n",
			msg_bytes, ctx->block_bytes, block_words, msg_words, ctx->dma_addr.alloc_size);
		pr_info("input dma alloc failed, allocate_size: %d", ctx->dma_addr.alloc_size);
		return SKE_BUFFER_NULL;
	}
	ctx->dma_addr.virt_out = ctx->dma_addr.virt_in + (msg_words * 4);
	ctx->dma_addr.phys_out = ctx->dma_addr.phys_in + (msg_words * 4);
	// pr_info("msg_bytes=%u, block_bytes=%u, block_words=%u, msg_words=%u, alloc_size=%u\n",
    //     msg_bytes, ctx->block_bytes, block_words, msg_words, ctx->dma_addr.alloc_size);

	if ((msg_words * 4 + ctx->block_bytes) > ctx->dma_addr.alloc_size) {
		pr_err("ERROR: DMA buffer too small. msg_words=%u, block_bytes=%u, alloc_size=%u\n",
				msg_words, ctx->block_bytes, ctx->dma_addr.alloc_size);
		dmam_free_coherent(global_ske->dev, ctx->dma_addr.alloc_size, ctx->dma_addr.virt_in, ctx->dma_addr.phys_in);
		return -ENOMEM;
	}

	dma_writel((uint32_t *)ctx->dma_addr.virt_in, msg, block_words);
	dma_writel((uint32_t *)ctx->dma_addr.virt_in + block_words, (uint32_t *)ctx->buf, msg_words - block_words);

	ret = ske_hp_dma_operate((ctx->dma_addr.phys_in - CMA_ADDR_OFFSET), (ctx->dma_addr.phys_out - CMA_ADDR_OFFSET),
							msg_words, ctx->block_words, ske_dma_callback);
	if (ret == SKE_SUCCESS) {
		if (SKE_GENERATE_MAC == ctx->mac)
			dma_readl(mac, (uint32_t *)ctx->dma_addr.virt_out, ctx->block_words);
		else if (SKE_VERIFY_MAC == ctx->mac && dma_cmp(mac, (uint32_t *)ctx->dma_addr.virt_out, ctx->block_words)) {
			ret = SKE_ERROR;
			pr_info("bst_dma_cmac_%s verify ret:%d", ske_alg_info[ctx->alg], ret);
		}	
	} else {
		bst_dbg(1, "ske_hp_dma_operate failed, ret: %d", ret);
	}
	
	dmam_free_coherent(global_ske->dev, ctx->dma_addr.alloc_size, ctx->dma_addr.virt_in, ctx->dma_addr.phys_in);
	ctx->dma_addr.virt_in = NULL;
	ctx->dma_addr.virt_out = NULL;
	return ret;
}


uint32_t ske_hp_dma_cbc_mac_update_including_last_block(struct ske_ctx *ctx, uint32_t *msg, uint32_t msg_bytes, uint32_t *mac)
{
	uint32_t ret;
	uint32_t msg_words, block_words;
	uint32_t block_bytes, remainder_bytes;

	if((NULL == ctx) || (NULL == msg) || (NULL == mac)) {
		return SKE_BUFFER_NULL;
	} else if(0 == msg_bytes) {
		bst_dbg(1, "ske_hp_dma_cbc_mac_update_including_last_block 0 == msg_bytes\n");
		return SKE_INPUT_INVALID;
	}

	block_bytes = (msg_bytes / ctx->block_bytes) * ctx->block_bytes;
	block_words = block_bytes / 4;

	if (msg_bytes & (ctx->block_bytes - 1)) {
		//padding
		memset(ctx->buf, 0, ctx->block_bytes);
		remainder_bytes = msg_bytes & (ctx->block_bytes - 1);
		memcpy(ctx->buf, (uint8_t *)msg + block_bytes, remainder_bytes);

		//padding padding
		msg_words = block_words + ctx->block_words;
	} else {
		//align
		msg_words = block_words;
	}
	//msg_words *= ctx->block_words;

	ske_hp_set_last_block(1);
	if(ctx->dma_addr.virt_in != NULL){
		dmam_free_coherent(global_ske->dev, ctx->dma_addr.alloc_size, ctx->dma_addr.virt_in, ctx->dma_addr.phys_in);
		ctx->dma_addr.virt_in = NULL;
	}
	ctx->dma_addr.alloc_size = max((uint32_t)((msg_words * 4 + ctx->block_bytes)), (uint32_t)(2 * PAGE_SIZE));
	if ((msg_words * 4 + ctx->block_bytes) > ctx->dma_addr.alloc_size) {
		pr_err("ERROR: DMA buffer too small. msg_words=%u, block_bytes=%u, alloc_size=%u\n",
				msg_words, ctx->block_bytes, ctx->dma_addr.alloc_size);
		dmam_free_coherent(global_ske->dev, ctx->dma_addr.alloc_size, ctx->dma_addr.virt_in, ctx->dma_addr.phys_in);
		return -ENOMEM;
	}
	ctx->dma_addr.virt_in = dmam_alloc_coherent(global_ske->dev, ctx->dma_addr.alloc_size, &ctx->dma_addr.phys_in, GFP_KERNEL);
	if (ctx->dma_addr.virt_in == NULL) {
		bst_dbg(1, "ctx->block_bytes=%u, ctx->block_words=%u, msg_bytes=%u, block_bytes=%u, block_words=%u, msg_words=%u, alloc_size=%u\n",
			ctx->block_bytes, ctx->block_words, msg_bytes, block_bytes, block_words, msg_words, ctx->dma_addr.alloc_size);
		bst_dbg(1, "input dma alloc failed, allocate_size: %d", ctx->dma_addr.alloc_size);
		return SKE_BUFFER_NULL;
	}
	ctx->dma_addr.virt_out = ctx->dma_addr.virt_in + (msg_words * 4);
	ctx->dma_addr.phys_out = ctx->dma_addr.phys_in + (msg_words * 4);
	// bst_dbg(1, "msg_bytes=%u, block_bytes=%u, block_words=%u, msg_words=%u, alloc_size=%u\n",
    //     msg_bytes, ctx->block_bytes, block_words, msg_words, ctx->dma_addr.alloc_size);

	dma_writel((uint32_t *)ctx->dma_addr.virt_in, msg, block_words);
	dma_writel((uint32_t *)ctx->dma_addr.virt_in + block_words, (uint32_t *)ctx->buf, msg_words - block_words);

	ret = ske_hp_dma_operate((ctx->dma_addr.phys_in - CMA_ADDR_OFFSET), (ctx->dma_addr.phys_out - CMA_ADDR_OFFSET),
							msg_words, ctx->block_words, ske_dma_callback);
	if (ret == SKE_SUCCESS) {
		if (SKE_GENERATE_MAC == ctx->mac)
			dma_readl(mac, (uint32_t *)ctx->dma_addr.virt_out, ctx->block_words);
		else if (SKE_VERIFY_MAC == ctx->mac && dma_cmp(mac, (uint32_t *)ctx->dma_addr.virt_out, ctx->block_words)) {
			ret = SKE_ERROR;
			bst_dbg(1, "bst_dma_cbc_mac_%s verify ret:%d", ske_alg_info[ctx->alg], ret);
		}	
	} else {
		bst_dbg(1, "ske_hp_dma_operate failed, ret: %d", ret);
	}
	dmam_free_coherent(global_ske->dev, ctx->dma_addr.alloc_size, ctx->dma_addr.virt_in, ctx->dma_addr.phys_in);
	ctx->dma_addr.virt_in = NULL;
	ctx->dma_addr.virt_out = NULL;
	return ret;
}

void ske_get_version(void __iomem *io_base, uint32_t *major, uint32_t *minor)
{
	*major = (read_reg(io_base + SKE_VERSION) & 0xf0) >> 4;
	*minor = read_reg(io_base + SKE_VERSION) & 0x0f;
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
	bst_dbg(1, "ske setkey ctx:%p\n",actx);
	printHex("ske key",key, keylen);
	return 0;
}

static uint32_t ske_crypt(struct ske_ctx *ctx, const void *src,
						  void *dst, uint32_t len, u8 *iv)
{
	uint32_t ret;
	ret = ske_hp_init(ctx, iv, SKE_HP_CPU_MODE);
	if (ret != SKE_SUCCESS)
		return ret;
	ret = ske_hp_update_including_last_2_blocks(ctx, (uint8_t *)src, dst, len);
	bst_dbg(1, "len:%d", len);
	printHex("src",(uint8_t *)src, 10);
	printHex("dst",(uint8_t *)dst, 10);
	bst_dbg(1, "ret:%d", ret);
	return ret;
}
// static uint32_t ske_cmac_test(struct ske_ctx *ctx)
// {
// 	uint8_t mac[16];
// 	uint32_t ret;
// 	// uint8_t std_sm4_cmac_key[16]  =  {0xE0,0x70,0x99,0xF1,0xBF,0xAF,0xFD,0x7F,0x24,0x0C,0xD7,0x90,0xCA,0x4F,0xE1,0x34,};
// 	uint8_t message[10];
// 	// memcpy(ctx->key, std_sm4_cmac_key, 16);
// 	memcpy(message, "abc", 3);
// 	ctx->key_len = 16;
// 	ret = ske_hp_cmac_init(ctx);
// 	if (ret != SKE_SUCCESS)
// 		return ret;
// 	ret = ske_hp_cmac_update(ctx, (uint8_t *)message, 3);
// 	if (ret != SKE_SUCCESS)
// 		return ret;
// 	ret = ske_hp_cmac_final(ctx, (uint8_t *)mac);
// 	if (ret != SKE_SUCCESS)
// 		return ret;
// 	bst_dbg(1, "sm4_cmac test mac: %x %x %x %x %x %x %x %x %x %x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], mac[6], mac[7], mac[8], mac[9]);
// 	return SKE_SUCCESS;
// }
static uint32_t ske_mac_cmac(struct ske_ctx *ctx, void *msg,
						void *mac, uint32_t len)
{
	uint32_t ret;
	ret = ske_hp_cmac_init(ctx);
	if (ret != SKE_SUCCESS)
		return ret;
	ret = ske_hp_cmac_update(ctx, (uint8_t *)msg, len);
	if (ret != SKE_SUCCESS)
		return ret;
	ret = ske_hp_cmac_final(ctx, (uint8_t *)mac);	
	if (ret != SKE_SUCCESS)
		return ret;
	return ret;
}

static uint32_t ske_mac_cbc_mac(struct ske_ctx *ctx,void *msg,
						void *mac, uint32_t len)
{
	uint32_t ret;
	
	ret = ske_hp_cbc_mac_init(ctx);
	if (ret != SKE_SUCCESS)
		return ret;
	ret = ske_hp_cbc_mac_update(ctx, (uint8_t *)msg, len);
	if (ret != SKE_SUCCESS)
		return ret;
	return ske_hp_cbc_mac_final(ctx, (uint8_t *)mac);
}

static uint32_t ske_mac_dma_cmac(struct ske_ctx *ctx, void *msg,
						void *mac, uint32_t len)
{
	uint32_t ret;
	ret = ske_hp_dma_cmac_init(ctx);
	if (ret != SKE_SUCCESS){
		bst_dbg(1, "ske_hp_dma_cmac_init fail ret = %d\n",ret);
		return ret;
	}
	ret = ske_hp_dma_cmac_update_including_last_block(ctx, (uint32_t *)msg, len, (uint32_t *)mac);
	return ret;
}

static uint32_t ske_mac_dma_cbc_mac(struct ske_ctx *ctx,void *msg,
						void *mac, uint32_t len)
{
	uint32_t ret;
	
	ret = ske_hp_dma_cbc_mac_init(ctx);
	if (ret != SKE_SUCCESS){
		bst_dbg(1, "ske_hp_dma_cbc_mac_init fail ret = %d\n",ret);
		return ret;
	}
	ret = ske_hp_dma_cbc_mac_update_including_last_block(ctx, (uint32_t *)msg, len, (uint32_t *)mac);
	return ret;
}

int bst_ske_crypt(struct skcipher_request *req)
{
	struct crypto_async_request *arq = &req->base;
	struct ske_ctx *ctx = crypto_tfm_ctx(arq->tfm);
	uint8_t *input_data = NULL;
	uint8_t *output_data = NULL;
	unsigned int nbytes;
	int err;
	
	bst_dbg(1, "in bst_ske_crypt ctx:%p\n" ,ctx);

	nbytes = req->cryptlen;
	ctx->c_bytes = nbytes;

	if(ctx->hasMutex == 0){
		OPTRYLOCK
		ctx->hasMutex = 1;
	}else{
		OPUPDATETIME
	}

	err = ske_hp_init(ctx, req->iv, SKE_HP_CPU_MODE);
	if (SKE_SUCCESS != err){
		err = -EINVAL;
		goto free;
	}

	input_data = (uint8_t *)kmalloc(nbytes, GFP_KERNEL);
	if (input_data == NULL){
		err = -ENOMEM;
		goto free;
	}
	output_data = (uint8_t *)kmalloc(nbytes, GFP_KERNEL);
	if (output_data == NULL) {
		err = -ENOMEM;
		goto free;
	}

	err = sg_pcopy_to_buffer(req->src, sg_nents_for_len(req->src, nbytes), input_data, nbytes, 0);
	if (err != nbytes) {
		err = -EINVAL;
		goto free;
	}

	if (ctx->mode == SKE_MODE_XTS)
		err = ske_hp_xts_update_blocks(ctx, input_data, output_data, nbytes);
	else
		err = ske_hp_update_blocks(ctx, input_data, output_data, round_down(nbytes, ctx->block_bytes));
	if (SKE_SUCCESS != err){
		goto free;
	}

	err = sg_pcopy_from_buffer(req->dst, sg_nents(req->dst), output_data, nbytes, 0);
	if (err != nbytes){
		err = -EINVAL;
		goto free;
	}

#if BST_IV_UPDATE == 1
	//ske_hp_get_iv(ctx->iv, ctx->block_bytes);
    err = ske_compute_next_iv(ctx->mode, ctx->crypto == SKE_CRYPTO_ENCRYPT,
                        ctx->iv, ctx->block_bytes,
                        input_data, nbytes, output_data, nbytes);
    // if (err != 0){
	// 	bst_dbg(1, "ske_compute_next_iv fail ret:%d\n" ,err);
	// }else{
	// 	printHex("new iv",ctx->iv,ctx->block_bytes);
	// }
#endif
	err = 0;

free:
	bst_kfree(input_data);
	bst_kfree(output_data);
	ctx->hasMutex = 0;
	OPUNLOCK
	return err;
}


int bst_ske_dma_crypt(struct skcipher_request *req)
{
	struct crypto_async_request *arq = &req->base;
	struct ske_ctx *ctx = crypto_tfm_ctx(arq->tfm);
	uint8_t *input_data = NULL;
	uint8_t *output_data = NULL;
	unsigned int nbytes;
	int err;
	nbytes = req->cryptlen;
	ctx->c_bytes = nbytes;
	bst_dbg(1, "in bst_ske_dma_crypt ctx:%p\n" ,ctx);

	if(ctx->hasMutex == 0){
		OPTRYLOCK
		ctx->hasMutex = 1;
	}else{
		OPUPDATETIME
	}

	err = ske_hp_init(ctx, req->iv, SKE_HP_DMA_MODE);
	if (SKE_SUCCESS != err) {
		bst_dbg(1, "dma crypt ske_hp_init fail err = %d, return ctx:%p\n",err ,ctx);
		err = -EINVAL;
		goto free_data;
	}

	input_data = (uint8_t *)kmalloc(nbytes, GFP_KERNEL);
	if (input_data == NULL){
		bst_dbg(1, "dma crypt malloc input_data fail nbytes = %d return ctx:%p\n",nbytes ,ctx);
		err = -ENOMEM;
		goto free_data;
	}
	output_data = (uint8_t *)kmalloc(nbytes, GFP_KERNEL);
	if (output_data == NULL) {
		bst_dbg(1, "dma crypt malloc output_data fail nbytes = %d return ctx:%p\n",nbytes ,ctx);
		err = -ENOMEM;
		goto free_data;
	}
	err = sg_pcopy_to_buffer(req->src, sg_nents_for_len(req->src, nbytes), input_data, nbytes, 0);
	if (err != nbytes) {
		bst_dbg(1, "dma crypt sg_pcopy_to_buffer fail %d != nbytes:%d return ctx:%p\n",err,nbytes ,ctx);
		err = -EINVAL;
		goto free_data;
	}
	bst_dbg(1, "crypt ctx:%p\n",ctx);
	printHex(ctx->crypto == SKE_CRYPTO_ENCRYPT ? "bst_ske_dma_crypt enc data in" : "bst_ske_dma_crypt dec data in",input_data, nbytes);

	/* allocate twice of ctx->c_bytes, and use the first half for input, and the second half for output */
	/* to allocate cma memory, size must be at least 2 * PAGE_SIZE */
	ctx->dma_addr.alloc_size = max((uint32_t)(2 * ((ctx->c_bytes + 15) / 16 * 16)), (uint32_t)(2 * PAGE_SIZE));
	ctx->dma_addr.virt_in = dmam_alloc_coherent(global_ske->dev, ctx->dma_addr.alloc_size, &ctx->dma_addr.phys_in, GFP_KERNEL);
	if (ctx->dma_addr.virt_in == NULL) {
		bst_dbg(1,"inpt dma crypt dmam_alloc_coherent failed, allocate_size: %d, ctx:%p", ctx->dma_addr.alloc_size, ctx);
		err = SKE_BUFFER_NULL;
		goto free_data;
	}
	ctx->dma_addr.virt_out = ctx->dma_addr.virt_in + (ctx->c_bytes + 15) / 16 * 16;
	ctx->dma_addr.phys_out = ctx->dma_addr.phys_in + (ctx->c_bytes + 15) / 16 * 16;

	if (ctx->mode == SKE_MODE_XTS) 
		err = ske_hp_dma_xts_update_blocks(ctx, (uint32_t *)input_data, (uint32_t *)output_data, ske_dma_callback);
	else
		err = ske_hp_dma_update_blocks(ctx, (uint32_t *)input_data, (uint32_t *)output_data, round_down(nbytes, ctx->block_bytes) / 4, ske_dma_callback);

	if (SKE_SUCCESS != err){
		bst_dbg(1,"input dma crypt update fail err:%d, allocate_size: %d ctx:%p\n",err , ctx->dma_addr.alloc_size,ctx);
		goto free_dma;
	}

	err = sg_pcopy_from_buffer(req->dst, sg_nents(req->dst), output_data, nbytes, 0);
	if (err != nbytes){
		bst_dbg(1, "dma crypt sg_pcopy_from_buffer fail %d != nbytes:%d return ctx:%p\n",err,nbytes ,ctx);
		err = -EINVAL;
		goto free_dma;
	}
	
#if BST_IV_UPDATE == 1
	//ske_hp_get_iv(ctx->iv, ctx->block_bytes);
	err = ske_compute_next_iv(ctx->mode, ctx->crypto == SKE_CRYPTO_ENCRYPT,
                        ctx->iv, ctx->block_bytes,
                        input_data, nbytes, output_data, nbytes);
    // if (err != 0){
	// 	bst_dbg(1, "ske_compute_next_iv fail ret:%d\n" ,err);
	// }else{
	// 	printHex("new iv",ctx->iv,ctx->block_bytes);
	// }
#endif
	err = 0;

free_dma:
	dmam_free_coherent(global_ske->dev, ctx->dma_addr.alloc_size, ctx->dma_addr.virt_in, ctx->dma_addr.phys_in);
	ctx->dma_addr.virt_in = NULL;
	ctx->dma_addr.virt_out = NULL;
free_data:
	printHex(ctx->crypto == SKE_CRYPTO_ENCRYPT ? "bst_ske_dma_crypt enc data out" : "bst_ske_dma_crypt dec data out",output_data, nbytes);
	bst_kfree(input_data);
	bst_kfree(output_data);
	ctx->hasMutex = 0;
	OPUNLOCK
	return err;
}

int bst_ske_mac(struct skcipher_request *req)
{
	int ret, err;
	uint32_t (*mac_func)(struct ske_ctx *, void *, void *, uint32_t);
	struct crypto_async_request *arq = &req->base;
	struct ske_ctx *ctx = crypto_tfm_ctx(arq->tfm);
	uint8_t *in = (uint8_t *)NULL;
	uint8_t *out = (uint8_t *)NULL;
	// uint8_t *in = sg_virt(req->src);
	// uint8_t *out = sg_virt(req->dst);
	uint8_t *msg_buf = (uint8_t *)NULL;
	uint8_t *mac_buf = (uint8_t *)NULL;
	uint32_t msg_bytes = 0;

	bst_dbg(1, "in bst_ske_mac ctx:%p\n" ,ctx);
	if(ctx->hasMutex == 0){
		OPTRYLOCK
		ctx->hasMutex = 1;
	}else{
		OPUPDATETIME
	}

	mac_func =  (ctx->mode == SKE_MODE_CMAC) ? ske_mac_cmac : ske_mac_cbc_mac;
	ctx->mac_bytes = get_alg_blksize(ctx->alg);  /* mac_bytes = block_size of each algorithm */
	
	in = kmalloc(req->cryptlen, GFP_KERNEL);
	if (in == NULL){
		ret = -ENOMEM;
		goto free;
	}
	out = kmalloc(ctx->mac_bytes, GFP_KERNEL);
	if (out == NULL) {
		ret = -ENOMEM;
		goto free;
	}
	err = sg_pcopy_to_buffer(req->src, sg_nents_for_len(req->src, req->cryptlen), in, req->cryptlen, 0);
	if (err != req->cryptlen) {
		pr_info("input data copy error: %d", err);
		ret = SKE_BUFFER_NULL;
		goto free;
	}

	mac_buf = out;
	msg_buf = in;
	msg_bytes = req->cryptlen;


	ret = mac_func(ctx, (uint8_t *)msg_buf, (uint8_t *)mac_buf, msg_bytes);

	err = sg_pcopy_from_buffer(req->dst, sg_nents(req->dst), out, ctx->mac_bytes, 0);

	if (err != ctx->mac_bytes) {
		pr_info("output data copy error: %d", err);
		ret = SKE_BUFFER_NULL;
	}
free:
	bst_kfree(in);
	bst_kfree(out);
	ctx->hasMutex = 0;
	OPUNLOCK
	return ret;
}

int bst_ske_dma_mac(struct skcipher_request *req)
{
	int ret, err;
	uint32_t (*mac_func)(struct ske_ctx *, void *, void *, uint32_t);
	struct crypto_async_request *arq = &req->base;
	struct ske_ctx *ctx = crypto_tfm_ctx(arq->tfm);
	uint8_t *in = (uint8_t *)NULL;
	uint8_t *out = (uint8_t *)NULL;
	// uint8_t *in = sg_virt(req->src);
	// uint8_t *out = sg_virt(req->dst);
	uint8_t *msg_buf = (uint8_t *)NULL;
	uint8_t *mac_buf = (uint8_t *)NULL;
	uint32_t msg_bytes = 0;// mac_bytes = 0;

	bst_dbg(1, "in bst_ske_dma_mac ctx:%p\n" ,ctx);
	if(ctx->hasMutex == 0){
		OPTRYLOCK
		ctx->hasMutex = 1;
	}else{
		OPUPDATETIME
	}

	mac_func =  (ctx->mode == SKE_MODE_CMAC) ? ske_mac_dma_cmac : ske_mac_dma_cbc_mac;
	ctx->mac_bytes = get_alg_blksize(ctx->alg);  /* mac_bytes = block_size of each algorithm */
	
	in = kmalloc(req->cryptlen, GFP_KERNEL);
	if (in == NULL){
		ret = -ENOMEM;
		goto free;
	}
	out = kmalloc(ctx->mac_bytes, GFP_KERNEL);
	if (out == NULL){
		ret = -ENOMEM;
		goto free;
	}
	err = sg_pcopy_to_buffer(req->src, sg_nents_for_len(req->src, req->cryptlen), in, req->cryptlen, 0);
	if (err != req->cryptlen) {
		pr_info("input data copy error: %d", err);
		ret = SKE_BUFFER_NULL;
		goto free;
	}

	mac_buf = out;
	msg_buf = in;
	msg_bytes = req->cryptlen;

	ret = mac_func(ctx, (uint32_t *)msg_buf, (uint32_t *)mac_buf, msg_bytes);
	if(ret != 0){
		bst_dbg(1, "mac_func fail ret = %d\n",ret);
	}
	err = sg_pcopy_from_buffer(req->dst, sg_nents(req->dst), out, ctx->mac_bytes, 0);

	if (err != ctx->mac_bytes) {
		pr_info("output data copy error: %d", err);
		ret = SKE_BUFFER_NULL;
	}
free:
	bst_kfree(in);
	bst_kfree(out);
	ctx->hasMutex = 0;
	OPUNLOCK
	return ret;
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

	// bst_dbg(1, "%s:%d cryptlen=%d, assoclen=%d\n", __func__, __LINE__, req->cryptlen, req->assoclen);
	// print_buf_u32((uint32_t *)in_buf, req->assoclen/4 + req->cryptlen/4 - ctx->mac_bytes/4);
	// print_buf_u32((uint32_t *)ctx->key, ctx->key_len/4);
	// print_buf_u32((uint32_t *)req->iv, 12/4);
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

	ret = ske_hp_init(ctx, req->iv, SKE_HP_CPU_MODE);
	if (ret != SKE_SUCCESS)
		return ret;

	ret = ske_hp_gcm_aad(ctx, in_buf);
	if (ret != SKE_SUCCESS)
		return ret;

	ret = ske_hp_gcm_update_blocks(ctx, (in_buf + ctx->aad_bytes), (out_buf + ctx->aad_bytes), ctx->c_bytes);
	if (ret != SKE_SUCCESS)
		return ret;

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

	// bst_dbg(1, "%s:%d cryptlen=%d, assoclen=%d\n", __func__, __LINE__, req->cryptlen, req->assoclen);
	// print_buf_u32((uint32_t *)in_buf, req->assoclen/4 + req->cryptlen/4 +  ctx->mac_bytes/4);
	// print_buf_u32((uint32_t *)ctx->key, ctx->key_len/4);
	// print_buf_u32((uint32_t *)req->iv, 16/4);
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
	// bst_dbg(1, "%s:%d L=%d\n", __func__, __LINE__, ctx->L);
	// bst_dbg(1, "%s:%d mac_buf:", __func__, __LINE__);
	// print_buf_u32((uint32_t *)mac_buf, 4);
	ctx->M = crypto_aead_authsize(crypto_aead_reqtfm(req));

	ret = ske_hp_ccm_pre_init(ctx, (req->iv + 1));
	if (ret != SKE_SUCCESS)
		return ret;

	// print_buf_u32((uint32_t *)ctx->buf, 16/4);
	ret = ske_hp_init(ctx, ctx->buf, SKE_HP_CPU_MODE);
	if (ret != SKE_SUCCESS)
		return ret;

	// get and input B0
	ske_hp_ccm_get_B0((req->iv + 1), ctx->M, ctx->L, ctx->aad_bytes, ctx->c_bytes, ctx->buf);
	if (ctx->aad_bytes == 0)
		ske_hp_set_last_block(1); // last block;

	// print_buf_u32((uint32_t *)ctx->buf, 16/4);
	ret = ske_hp_update_blocks_no_output(ctx, ctx->buf, ctx->block_bytes);
	if (ret != SKE_SUCCESS)
		return ret;

	if (ctx->aad_bytes == 0)
		ske_hp_set_last_block(0); // not last block

	// prepare B1
	if (ctx->aad_bytes != 0)
		ske_hp_ccm_pre_B1(ctx);

	ret = ske_hp_ccm_aad(ctx, in_buf);
	if (ret != SKE_SUCCESS)
		return ret;

	ret = ske_hp_ccm_update_blocks(ctx, (in_buf + ctx->aad_bytes), (out_buf + ctx->aad_bytes), ctx->c_bytes);
	if (ret != SKE_SUCCESS)
		return ret;

	// print_buf_u32((uint32_t *)(out_buf + ctx->aad_bytes), (req->cryptlen + 3)/4);
	// print_buf_u32((uint32_t *)mac_buf, 4);
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

int bst_ske_dma_encrypt(struct skcipher_request *req)
{
	struct crypto_async_request *arq = &req->base;
	struct ske_ctx *ctx = crypto_tfm_ctx(arq->tfm);

	ctx->crypto = SKE_CRYPTO_ENCRYPT;
	return bst_ske_dma_crypt(req);
}

int bst_ske_dma_decrypt(struct skcipher_request *req)
{
	struct crypto_async_request *arq = &req->base;
	struct ske_ctx *ctx = crypto_tfm_ctx(arq->tfm);

	ctx->crypto = SKE_CRYPTO_DECRYPT;
	return bst_ske_dma_crypt(req);
}

int bst_ske_genmac(struct skcipher_request *req)
{
	struct crypto_async_request *arq = &req->base;
	struct ske_ctx *ctx = crypto_tfm_ctx(arq->tfm);

	ctx->crypto = SKE_CRYPTO_ENCRYPT;
	ctx->mac = SKE_GENERATE_MAC;

	return bst_ske_mac(req);
}

int bst_ske_vermac(struct skcipher_request *req)
{
	struct crypto_async_request *arq = &req->base;
	struct ske_ctx *ctx = crypto_tfm_ctx(arq->tfm);

	ctx->crypto = SKE_CRYPTO_ENCRYPT;
	ctx->mac = SKE_VERIFY_MAC;

	return bst_ske_mac(req);
}

int bst_ske_dma_genmac(struct skcipher_request *req)
{
	struct crypto_async_request *arq = &req->base;
	struct ske_ctx *ctx = crypto_tfm_ctx(arq->tfm);

	ctx->crypto = SKE_CRYPTO_ENCRYPT;
	ctx->mac = SKE_GENERATE_MAC;

	return bst_ske_dma_mac(req);
}

int bst_ske_dma_vermac(struct skcipher_request *req)
{
	struct crypto_async_request *arq = &req->base;
	struct ske_ctx *ctx = crypto_tfm_ctx(arq->tfm);

	ctx->crypto = SKE_CRYPTO_ENCRYPT;
	ctx->mac = SKE_VERIFY_MAC;

	return bst_ske_dma_mac(req);
}

int bst_ske_init(struct crypto_skcipher *tfm, enum ske_alg alg, enum ske_mode mode)
{
	struct ske_ctx *ctx = crypto_skcipher_ctx(tfm);

	ctx->alg = alg;
	ctx->mode = mode;
	ctx->ivInited = 0;
	ctx->hasMutex = 0;
	//CheckOPTimeout();
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
	// bst_dbg(1, "%s:%d authsize=%d", __func__, __LINE__, authsize);
	return 0;
}

int bst_ske_gcm_setkey(struct crypto_aead *tfm, const u8 *key,
					   unsigned int keylen)
{
	struct ske_ctx *actx = crypto_aead_ctx(tfm);
	// bst_dbg(1, "%s:%d keylen=%d", __func__, __LINE__, keylen);
	if (keylen != AES_KEYSIZE_128 &&
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
	// bst_dbg(1, "%s:%d authsize=%d", __func__, __LINE__, authsize);
	return 0;
}

int bst_ske_ccm_setkey(struct crypto_aead *tfm, const u8 *key,
					   unsigned int keylen)
{
	struct ske_ctx *actx = crypto_aead_ctx(tfm);
	// bst_dbg(1, "%s:%d keylen=%d", __func__, __LINE__, keylen);
	if (keylen != AES_KEYSIZE_128 &&
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
int bst_aes_128_gcm_init(struct crypto_aead *tfm)
{
	return bst_ske_aead_init(tfm, SKE_ALG_AES_128, SKE_MODE_GCM);
}
int bst_aes_128_ccm_init(struct crypto_aead *tfm)
{
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
int bst_ske_aes_cbc_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_AES, SKE_MODE_CBC);
}
int bst_ske_aes_ecb_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_AES, SKE_MODE_ECB);
}
int bst_ske_aes_cfb_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_AES, SKE_MODE_CFB);
}
int bst_ske_aes_ofb_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_AES, SKE_MODE_OFB);
}
int bst_ske_aes_xts_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_AES, SKE_MODE_XTS);
}
int bst_ske_aes_ctr_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_AES, SKE_MODE_CTR);
}
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

int bst_ske_aes_cbc_mac_256_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_AES_256, SKE_MODE_CBC_MAC);
}

int bst_ske_aes_cmac_256_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_AES_256, SKE_MODE_CMAC);
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

int bst_ske_aes_cbc_mac_192_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_AES_192, SKE_MODE_CBC_MAC);
}

int bst_ske_aes_cmac_192_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_AES_192, SKE_MODE_CMAC);
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

int bst_ske_aes_cbc_mac_128_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_AES_128, SKE_MODE_CBC_MAC);
}

int bst_ske_aes_cmac_128_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_AES_128, SKE_MODE_CMAC);
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

int bst_ske_sm4_cbc_mac_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_SM4, SKE_MODE_CBC_MAC);
}

int bst_ske_sm4_cmac_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_SM4, SKE_MODE_CMAC);
}

/* DES*/
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

int bst_ske_des_cbc_mac_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_DES, SKE_MODE_CBC_MAC);
}

int bst_ske_des_cmac_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_DES, SKE_MODE_CMAC);
}
/* TDES_128*/
int bst_ske_tdes_ecb_128_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_TDES_128, SKE_MODE_ECB);
}
int bst_ske_tdes_cbc_128_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_TDES_128, SKE_MODE_CBC);
}
int bst_ske_tdes_cfb_128_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_TDES_128, SKE_MODE_CFB);
}
int bst_ske_tdes_ofb_128_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_TDES_128, SKE_MODE_OFB);
}
int bst_ske_tdes_ctr_128_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_TDES_128, SKE_MODE_CTR);
}

int bst_ske_tdes_cbc_mac_128_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_TDES_128, SKE_MODE_CBC_MAC);
}

int bst_ske_tdes_cmac_128_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_TDES_128, SKE_MODE_CMAC);
}

/* TDES_192*/
int bst_ske_tdes_ecb_192_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_TDES_192, SKE_MODE_ECB);
}
int bst_ske_tdes_cbc_192_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_TDES_192, SKE_MODE_CBC);
}
int bst_ske_tdes_cfb_192_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_TDES_192, SKE_MODE_CFB);
}
int bst_ske_tdes_ofb_192_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_TDES_192, SKE_MODE_OFB);
}
int bst_ske_tdes_ctr_192_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_TDES_192, SKE_MODE_CTR);
}

int bst_ske_tdes_cbc_mac_192_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_TDES_192, SKE_MODE_CBC_MAC);
}

int bst_ske_tdes_cmac_192_init(struct crypto_skcipher *tfm)
{
	return bst_ske_init(tfm, SKE_ALG_TDES_192, SKE_MODE_CMAC);
}

static struct skcipher_alg bst_ske_algs[] = {
	{
		.base.cra_name = "cbc(aes)",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_cbc_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "ecb(aes)",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_ecb_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "cfb(aes)",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_cfb_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "ofb(aes)",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_ofb_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "xts(aes)",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_xts_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "ctr(aes)",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_ctr_init,
		.exit = bst_ske_exit,
	},
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
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_ecb_aes_256_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_ecb_256_init,
		.exit = bst_ske_exit,
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
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_xts_aes_256_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE * 2,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_xts_256_init,
		.exit = bst_ske_exit,
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
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cbc_aes_256_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_cbc_256_init,
		.exit = bst_ske_exit,
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
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cfb_aes_256_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_cfb_256_init,
		.exit = bst_ske_exit,
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
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_ofb_aes_256_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_ofb_256_init,
		.exit = bst_ske_exit,
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
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_ctr_aes_256_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_ctr_256_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cbc_mac_aes_256",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.macgen = bst_ske_genmac,
		.macver = bst_ske_vermac,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_cbc_mac_256_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cbc_mac_aes_256_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.macgen = bst_ske_dma_genmac,
		.macver = bst_ske_dma_vermac,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_cbc_mac_256_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cmac_aes_256",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.macgen = bst_ske_genmac,
		.macver = bst_ske_vermac,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_cmac_256_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cmac_aes_256_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.macgen = bst_ske_dma_genmac,
		.macver = bst_ske_dma_vermac,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_cmac_256_init,
		.exit = bst_ske_exit,
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
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_ecb_aes_192_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_ecb_192_init,
		.exit = bst_ske_exit,
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
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_xts_aes_192_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE * 2,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_xts_192_init,
		.exit = bst_ske_exit,
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
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cbc_aes_192_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_cbc_192_init,
		.exit = bst_ske_exit,
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
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cfb_aes_192_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_cfb_192_init,
		.exit = bst_ske_exit,
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
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_ofb_aes_192_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_ofb_192_init,
		.exit = bst_ske_exit,
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
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_ctr_aes_192_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_ctr_192_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cbc_mac_aes_192",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.macgen = bst_ske_genmac,
		.macver = bst_ske_vermac,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_cbc_mac_192_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cbc_mac_aes_192_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.macgen = bst_ske_dma_genmac,
		.macver = bst_ske_dma_vermac,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_cbc_mac_192_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cmac_aes_192",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.macgen = bst_ske_genmac,
		.macver = bst_ske_vermac,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_cmac_192_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cmac_aes_192_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.macgen = bst_ske_dma_genmac,
		.macver = bst_ske_dma_vermac,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_cmac_192_init,
		.exit = bst_ske_exit,
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
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_ecb_aes_128_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_ecb_128_init,
		.exit = bst_ske_exit,
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
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_xts_aes_128_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE * 2,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_xts_128_init,
		.exit = bst_ske_exit,
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
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cbc_aes_128_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_cbc_128_init,
		.exit = bst_ske_exit,
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
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cfb_aes_128_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_cfb_128_init,
		.exit = bst_ske_exit,
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
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_ofb_aes_128_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_ofb_128_init,
		.exit = bst_ske_exit,
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
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_ctr_aes_128_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_ctr_128_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cbc_mac_aes_128",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.macgen = bst_ske_genmac,
		.macver = bst_ske_vermac,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_cbc_mac_128_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cbc_mac_aes_128_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.macgen = bst_ske_dma_genmac,
		.macver = bst_ske_dma_vermac,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_cbc_mac_128_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cmac_aes_128",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.macgen = bst_ske_genmac,
		.macver = bst_ske_vermac,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_cmac_128_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cmac_aes_128_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.macgen = bst_ske_dma_genmac,
		.macver = bst_ske_dma_vermac,
		.ivsize = AES_BLOCK_SIZE,
		.init = bst_ske_aes_cmac_128_init,
		.exit = bst_ske_exit,
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
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cbc_sm4_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = SM4_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = SM4_KEY_SIZE,
		.max_keysize = SM4_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = SM4_BLOCK_SIZE,
		.init = bst_ske_sm4_cbc_init,
		.exit = bst_ske_exit,
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
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_ecb_sm4_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = SM4_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = SM4_KEY_SIZE,
		.max_keysize = SM4_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = SM4_BLOCK_SIZE,
		.init = bst_ske_sm4_ecb_init,
		.exit = bst_ske_exit,
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
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_ctr_sm4_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = SM4_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = SM4_KEY_SIZE,
		.max_keysize = SM4_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = SM4_BLOCK_SIZE,
		.init = bst_ske_sm4_ctr_init,
		.exit = bst_ske_exit,
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
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cfb_sm4_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = SM4_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = SM4_KEY_SIZE,
		.max_keysize = SM4_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = SM4_BLOCK_SIZE,
		.init = bst_ske_sm4_cfb_init,
		.exit = bst_ske_exit,
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
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_ofb_sm4_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = SM4_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = SM4_KEY_SIZE,
		.max_keysize = SM4_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = SM4_BLOCK_SIZE,
		.init = bst_ske_sm4_ofb_init,
		.exit = bst_ske_exit,
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
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_xts_sm4_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = SM4_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = SM4_KEY_SIZE,
		.max_keysize = SM4_KEY_SIZE * 2,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = SM4_BLOCK_SIZE,
		.init = bst_ske_sm4_xts_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cbc_mac_sm4",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = SM4_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = SM4_KEY_SIZE,
		.max_keysize = SM4_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.macgen = bst_ske_genmac,
		.macver = bst_ske_vermac,
		.ivsize = SM4_BLOCK_SIZE,
		.init = bst_ske_sm4_cbc_mac_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cbc_mac_sm4_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = SM4_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = SM4_KEY_SIZE,
		.max_keysize = SM4_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.macgen = bst_ske_dma_genmac,
		.macver = bst_ske_dma_vermac,
		.ivsize = SM4_BLOCK_SIZE,
		.init = bst_ske_sm4_cbc_mac_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cmac_sm4",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = SM4_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = SM4_KEY_SIZE,
		.max_keysize = SM4_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.macgen = bst_ske_genmac,
		.macver = bst_ske_vermac,
		.ivsize = SM4_BLOCK_SIZE,
		.init = bst_ske_sm4_cmac_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cmac_sm4_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = SM4_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = SM4_KEY_SIZE,
		.max_keysize = SM4_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.macgen = bst_ske_dma_genmac,
		.macver = bst_ske_dma_vermac,
		.ivsize = SM4_BLOCK_SIZE,
		.init = bst_ske_sm4_cmac_init,
		.exit = bst_ske_exit,
	},
	/* DES*/
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
		.ivsize = DES_BLOCK_SIZE,
		.init = bst_ske_des_ecb_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_ecb_des_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = DES_BLOCK_SIZE,
		.init = bst_ske_des_ecb_init,
		.exit = bst_ske_exit,
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
		.ivsize = DES_BLOCK_SIZE,
		.init = bst_ske_des_cbc_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cbc_des_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = DES_BLOCK_SIZE,
		.init = bst_ske_des_cbc_init,
		.exit = bst_ske_exit,
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
		.ivsize = DES_BLOCK_SIZE,
		.init = bst_ske_des_cfb_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cfb_des_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = DES_BLOCK_SIZE,
		.init = bst_ske_des_cfb_init,
		.exit = bst_ske_exit,
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
		.ivsize = DES_BLOCK_SIZE,
		.init = bst_ske_des_ofb_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_ofb_des_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = DES_BLOCK_SIZE,
		.init = bst_ske_des_ofb_init,
		.exit = bst_ske_exit,
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
		.ivsize = DES_BLOCK_SIZE,
		.init = bst_ske_des_ctr_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_ctr_des_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = DES_BLOCK_SIZE,
		.init = bst_ske_des_ctr_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cbc_mac_des",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.macgen = bst_ske_genmac,
		.macver = bst_ske_vermac,
		.ivsize = DES_BLOCK_SIZE,
		.init = bst_ske_des_cbc_mac_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cbc_mac_des_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.macgen = bst_ske_dma_genmac,
		.macver = bst_ske_dma_vermac,
		.ivsize = DES_BLOCK_SIZE,
		.init = bst_ske_des_cbc_mac_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cmac_des",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.macgen = bst_ske_genmac,
		.macver = bst_ske_vermac,
		.ivsize = DES_BLOCK_SIZE,
		.init = bst_ske_des_cmac_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cmac_des_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.macgen = bst_ske_dma_genmac,
		.macver = bst_ske_dma_vermac,
		.ivsize = DES_BLOCK_SIZE,
		.init = bst_ske_des_cmac_init,
		.exit = bst_ske_exit,
	},
	/* 3DES_128*/
	{
		.base.cra_name = "bst_ecb_tdes_128",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES3_EDE_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES3_EDE_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = DES3_EDE_BLOCK_SIZE,
		.init = bst_ske_tdes_ecb_128_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_ecb_tdes_128_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES3_EDE_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES3_EDE_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = DES3_EDE_BLOCK_SIZE,
		.init = bst_ske_tdes_ecb_128_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cbc_tdes_128",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES3_EDE_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES3_EDE_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = DES3_EDE_BLOCK_SIZE,
		.init = bst_ske_tdes_cbc_128_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cbc_tdes_128_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES3_EDE_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES3_EDE_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = DES3_EDE_BLOCK_SIZE,
		.init = bst_ske_tdes_cbc_128_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cfb_tdes_128",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES3_EDE_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES3_EDE_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = DES3_EDE_BLOCK_SIZE,
		.init = bst_ske_tdes_cfb_128_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cfb_tdes_128_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES3_EDE_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES3_EDE_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = DES3_EDE_BLOCK_SIZE,
		.init = bst_ske_tdes_cfb_128_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_ofb_tdes_128",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES3_EDE_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES3_EDE_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = DES3_EDE_BLOCK_SIZE,
		.init = bst_ske_tdes_ofb_128_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_ofb_tdes_128_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES3_EDE_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES3_EDE_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = DES3_EDE_BLOCK_SIZE,
		.init = bst_ske_tdes_ofb_128_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_ctr_tdes_128",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES3_EDE_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES3_EDE_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = DES3_EDE_BLOCK_SIZE,
		.init = bst_ske_tdes_ctr_128_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_ctr_tdes_128_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES3_EDE_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES3_EDE_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = DES3_EDE_BLOCK_SIZE,
		.init = bst_ske_tdes_ctr_128_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cbc_mac_tdes_128",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES3_EDE_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES3_EDE_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.macgen = bst_ske_genmac,
		.macver = bst_ske_vermac,
		.ivsize = DES3_EDE_BLOCK_SIZE,
		.init = bst_ske_tdes_cbc_mac_128_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cbc_mac_tdes_128_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES3_EDE_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES3_EDE_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.macgen = bst_ske_dma_genmac,
		.macver = bst_ske_dma_vermac,
		.ivsize = DES3_EDE_BLOCK_SIZE,
		.init = bst_ske_tdes_cbc_mac_128_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cmac_tdes_128",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES3_EDE_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.macgen = bst_ske_genmac,
		.macver = bst_ske_vermac,
		.ivsize = DES_BLOCK_SIZE,
		.init = bst_ske_tdes_cmac_128_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cmac_tdes_128_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES3_EDE_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.macgen = bst_ske_dma_genmac,
		.macver = bst_ske_dma_vermac,
		.ivsize = DES_BLOCK_SIZE,
		.init = bst_ske_tdes_cmac_128_init,
		.exit = bst_ske_exit,
	},
	/* 3DES_192*/
	{
		.base.cra_name = "bst_ecb_tdes_192",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES3_EDE_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES3_EDE_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = DES3_EDE_BLOCK_SIZE,
		.init = bst_ske_tdes_ecb_192_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_ecb_tdes_192_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES3_EDE_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES3_EDE_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = DES3_EDE_BLOCK_SIZE,
		.init = bst_ske_tdes_ecb_192_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cbc_tdes_192",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES3_EDE_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES3_EDE_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = DES3_EDE_BLOCK_SIZE,
		.init = bst_ske_tdes_cbc_192_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cbc_tdes_192_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES3_EDE_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES3_EDE_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = DES3_EDE_BLOCK_SIZE,
		.init = bst_ske_tdes_cbc_192_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cfb_tdes_192",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES3_EDE_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES3_EDE_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = DES3_EDE_BLOCK_SIZE,
		.init = bst_ske_tdes_cfb_192_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cfb_tdes_192_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES3_EDE_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES3_EDE_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = DES3_EDE_BLOCK_SIZE,
		.init = bst_ske_tdes_cfb_192_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_ofb_tdes_192",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES3_EDE_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES3_EDE_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = DES3_EDE_BLOCK_SIZE,
		.init = bst_ske_tdes_ofb_192_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_ofb_tdes_192_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES3_EDE_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES3_EDE_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = DES3_EDE_BLOCK_SIZE,
		.init = bst_ske_tdes_ofb_192_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_ctr_tdes_192",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES3_EDE_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES3_EDE_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_encrypt,
		.decrypt = bst_ske_decrypt,
		.ivsize = DES3_EDE_BLOCK_SIZE,
		.init = bst_ske_tdes_ctr_192_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_ctr_tdes_192_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES3_EDE_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES3_EDE_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.encrypt = bst_ske_dma_encrypt,
		.decrypt = bst_ske_dma_decrypt,
		.ivsize = DES3_EDE_BLOCK_SIZE,
		.init = bst_ske_tdes_ctr_192_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cbc_mac_tdes_192",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES3_EDE_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES3_EDE_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.macgen = bst_ske_genmac,
		.macver = bst_ske_vermac,
		.ivsize = DES3_EDE_BLOCK_SIZE,
		.init = bst_ske_tdes_cbc_mac_192_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cbc_mac_tdes_192_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES3_EDE_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES3_EDE_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.macgen = bst_ske_dma_genmac,
		.macver = bst_ske_dma_vermac,
		.ivsize = DES3_EDE_BLOCK_SIZE,
		.init = bst_ske_tdes_cbc_mac_192_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cmac_tdes_192",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES3_EDE_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.macgen = bst_ske_genmac,
		.macver = bst_ske_vermac,
		.ivsize = DES_BLOCK_SIZE,
		.init = bst_ske_tdes_cmac_192_init,
		.exit = bst_ske_exit,
	},
	{
		.base.cra_name = "bst_cmac_tdes_192_dma",
		.base.cra_driver_name = "bst_ske",
		.base.cra_priority = 400,
		.base.cra_alignmask = 15,
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ske_ctx),
		.base.cra_module = THIS_MODULE,

		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES3_EDE_KEY_SIZE,
		.setkey = bst_ske_setkey,
		.macgen = bst_ske_dma_genmac,
		.macver = bst_ske_dma_vermac,
		.ivsize = DES_BLOCK_SIZE,
		.init = bst_ske_tdes_cmac_192_init,
		.exit = bst_ske_exit,
	},
};

static int bst_ske_irq_handler(struct ske *dev)
{
	// pr_info("bst_ske_irq_handler");
	ske_disable_interruption();
	return 0;
}

static irqreturn_t bst_ske_irq(int irq, void *dev_id)
{
	struct ske *hdev = dev_id;
	uint32_t stat, enabled;

	enabled = read_reg(hdev->base + SKE_IMCR);
	stat = read_reg(hdev->base + SKE_MISR);

	dev_dbg(hdev->dev, "enabled=%#x stat=%#x\n", enabled, stat);
	if (!enabled || !stat)
		return IRQ_NONE;

	bst_ske_irq_handler(hdev);
	write_reg(read_reg(hdev->base + SKE_RISR) & (~1),
				   hdev->base + SKE_RISR);

	return IRQ_HANDLED;
}

static int bst_ske_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ske *ske = NULL;
	int i, ret;
	uint32_t v_major, v_minor;

	of_reserved_mem_device_init(&pdev->dev);
	dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(40));
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

	if (bst_sec_sa_ske_enable) {
		mutex_lock(&refcnt_lock);
		if (refcnt++ == 0) {
			ret = crypto_register_skciphers(bst_ske_algs,ARRAY_SIZE(bst_ske_algs));
			if (ret) {
				//mutex_unlock(&refcnt_lock);
				dev_err(dev, "Failed to register ske\n");
				//return ret;
			}else{
				dev_info(&pdev->dev, "BST ske algorithms registered\n");
			}
			ret = crypto_register_aeads(bst_algs_aead, ARRAY_SIZE(bst_algs_aead));
			if (ret) {
				//mutex_unlock(&refcnt_lock);
				dev_err(dev, "Failed to register aead\n");
				crypto_unregister_skciphers(bst_ske_algs, ARRAY_SIZE(bst_ske_algs));
				refcnt--;
				mutex_unlock(&refcnt_lock);
				return ret;
			}else{
				dev_info(&pdev->dev, "BST aead algorithms registered\n");
			}
		}
		mutex_unlock(&refcnt_lock);
	} else {
		dev_info(&pdev->dev, "BST hash driver loaded but algorithms disabled (bst_sec_sa_ske_enable=0)\n");
	}

	ske_get_version(ske->base, &v_major, &v_minor);
	dev_info(dev, "Hardware version: v%d.%d\n", v_major, v_minor);

	return 0;
}

static int bst_ske_remove(struct platform_device *pdev)
{
	struct ske *ske = platform_get_drvdata(pdev);
	mutex_lock(&refcnt_lock);
	if (!--refcnt) {
		crypto_unregister_skciphers(bst_ske_algs, ARRAY_SIZE(bst_ske_algs));
		crypto_unregister_aeads(bst_algs_aead, ARRAY_SIZE(bst_algs_aead));
	}
	mutex_unlock(&refcnt_lock);
	kthread_stop(ske->thread[0]);
	platform_set_drvdata(pdev, NULL);
	global_ske = NULL;

	return 0;
}

void sa_enbale_change_ske(void){
	mutex_lock(&refcnt_lock);
	if (bst_sec_sa_ske_enable && refcnt == 0) {
		crypto_register_skciphers(bst_ske_algs,ARRAY_SIZE(bst_ske_algs));
		crypto_register_aeads(bst_algs_aead, ARRAY_SIZE(bst_algs_aead));
		refcnt = 1;
	} else if (!bst_sec_sa_ske_enable && refcnt) {
		crypto_unregister_skciphers(bst_ske_algs, ARRAY_SIZE(bst_ske_algs));
		crypto_unregister_aeads(bst_algs_aead, ARRAY_SIZE(bst_algs_aead));
		refcnt = 0;
	}
	mutex_unlock(&refcnt_lock);
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
