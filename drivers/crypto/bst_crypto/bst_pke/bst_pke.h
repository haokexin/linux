/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (C) 2024 Black Sesame Technologies. Inc.
 */

#ifndef __BST_PKE_H__
#define __BST_PKE_H__

#include <linux/types.h>
#include <linux/mpi.h>
#include "eccp_curve.h"

#define PKE_CTRL 0x00		// 32 控制寄存器 W1S 0x0
#define PKE_CFG 0x04		// 32 配置寄存器 RW 0x0002_0100
#define PKE_MC_PTR 0x08		// 32 过程入口寄存器 RW 0x0
#define PKE_RISR 0x0C		// 32 中断源寄存器 W0C 0x0
#define PKE_IMCR 0x10		// 32 中断使能寄存器 RW 0x0
#define PKE_MISR 0x14		// 32 中断输出寄存器 RO 0x0
#define PKE_RT_CODE 0x24	// 32 返回日志寄存器 RO 0x0
#define PKE_RAND_SEED 0x40	// 32 随机数种子寄存器 RW 0x628a_92e1
#define PKE_RC_EN 0x60		// 32 存储加密使能寄存器 RW 0x0000_0001
#define PKE_RC_KEY 0x64		// 32 存储加密密钥寄存器 RW 0x0
#define PKE_RC_D_NONCE 0x68 // 32 存储加密数据临时数寄存器 RW 0x0
#define PKE_RC_A_NONCE 0x6C // 32 存储加密地址临时数寄存器 RW 0x0
#define PKE_VERSION 0xFC	// 32 版本寄存器 RO 0xXX1C_0010
#define PKE_MEM_A 0x400		// – 0xDFC 32 运算操作数 RAM0 RW 0x0
#define PKE_MEM_B 0x1000	// –0x19FC 32 运算操作数 RAM1 RW 0x0
#define PKE_A(a, step) ((uint32_t *)(0x0400 + (a) * (step) + global_pke->base))
#define PKE_B(a, step) ((uint32_t *)(0x1000 + (a) * (step) + global_pke->base))
/*********** PKE register action offset ************/
#define PKE_START_CALC (1)

/***************** PKE microcode ******************/
#define MICROCODE_PDBL (0x04)
#define MICROCODE_PADD (0x08)
#define MICROCODE_PVER (0x0C)
#define MICROCODE_PMUL (0x10)
#define MICROCODE_MODEXP (0x14)
#define MICROCODE_MODMUL (0x18)
#define MICROCODE_MODINV (0x1C)
#define MICROCODE_MODADD (0x20)
#define MICROCODE_MODSUB (0x24)
#define MICROCODE_MGMR_PRE_H (0x28)
#define MICROCODE_INTMUL (0x2C)
#define MICROCODE_Ed25519_PMUL (0x30)
#define MICROCODE_Ed25519_PADD (0x34)
#define MICROCODE_C25519_PMUL (0x38)
#define MICROCODE_MODRES (0x3C)
#define MICROCODE_INTADD (0x40)
#define MICROCODE_INTSUB (0x44)
#define MICROCODE_PMULF (0x48)
#define MICROCODE_MGMR_PRE_H_MM (0x4C)
#define MICROCODE_MGMR_PRE_N0 (0x50)

#define GET_MAX_LEN(a, b) (((a) > (b)) ? (a) : (b))
#define GET_MIN_LEN(a, b) (((a) > (b)) ? (b) : (a))
#define GET_WORD_LEN(bit_len) (((bit_len) + 31) / 32)
#define GET_BYTE_LEN(bit_len) (((bit_len) + 7) / 8)

/*********** some PKE algorithm operand length ************/
#define OPERAND_MAX_BIT_LEN (4096)
#define OPERAND_MAX_WORD_LEN (GET_WORD_LEN(OPERAND_MAX_BIT_LEN))

#define ECCP_MAX_BIT_LEN (521) // ECC521
#define ECCP_MAX_BYTE_LEN (GET_BYTE_LEN(ECCP_MAX_BIT_LEN))
#define ECCP_MAX_WORD_LEN (GET_WORD_LEN(ECCP_MAX_BIT_LEN))

#define C25519_BYTE_LEN (256 / 8)
#define C25519_WORD_LEN (256 / 32)

#define Ed25519_BYTE_LEN C25519_BYTE_LEN
#define Ed25519_WORD_LEN C25519_WORD_LEN

#define MAX_RSA_WORD_LEN OPERAND_MAX_WORD_LEN
#define MAX_RSA_BIT_LEN (MAX_RSA_WORD_LEN << 5)
#define MIN_RSA_BIT_LEN (512)

#define PKE_ECDSA_MAX_SIG_SIZE (2 * 1024 / 8)
#define PKE_ECDSA_MAX_DIGITS (1024 / 64)
#define PKE_ECDSA_DIGEST_SIZE 128

#define POINT_NOT_COMPRESSED (0x04)

#define SM2_HIGH_SPEED

#define KFreeMem(p)       if (p) {kfree(p); p = NULL;}
#define MFreeMem(p)       if (p) {mpi_free(p); p = NULL;}
/******************* PKE return code ********************/
enum PKE_RET_CODE {
	PKE_SUCCESS = 0,
	PKE_STOP,
	PKE_NO_MODINV,
	PKE_NOT_ON_CURVE,
	PKE_INVALID_MC,
	PKE_ZERO_ALL,		 // for ECCP input check
	PKE_INTEGER_TOO_BIG, // for ECCP input check
	PKE_INVALID_INPUT,
	PKE_ERROR,
};

enum RSA_RET_CODE {
	RSA_SUCCESS = PKE_SUCCESS,
	RSA_BUFFER_NULL = PKE_SUCCESS + 0x30,
	RSA_INPUT_TOO_LONG,
	RSA_INPUT_INVALID,
};

enum PKE_STATUS {
	PKE_NOT_AVAILABLE = 0,
	PKE_IS_AVAILABLE,

};
struct bst_rsa_mpi_key {
	MPI n;
	MPI e;
	MPI d;
};
// some sm2 length
#define SM2_BIT_LEN (256)
#define SM2_BYTE_LEN (32)
#define SM2_WORD_LEN (8)
#define SM3_DIGEST_BYTE_LEN SM2_BYTE_LEN
#define SM2_MAX_ID_BYTE_LEN (1 << 13)

// ECDSA return code
enum ECDSA_RET_CODE {
	ECDSA_SUCCESS = PKE_SUCCESS,
	ECDSA_POINTOR_NULL = PKE_SUCCESS + 0x50,
	ECDSA_INVALID_INPUT,
	ECDSA_ZERO_ALL,
	ECDSA_INTEGER_TOO_BIG,
	ECDSA_VERIFY_FAILED,
};

struct bst_mpi_ec_ctx {
	struct mpi_ec_ctx ec;
	MPI p_h;
	MPI n_h;
	MPI n_1;
	MPI g_x_h;
	MPI g_y_h;
	uint8_t *key;
	uint32_t key_len;
};

enum ec_type {
	/* EC */
	ec_brainpoolp160r1 = 0x4d, /* 1.3.36.3.3.2.8.1.1.1 */
	ec_secp192r1,			   /* 1.2.840.10045.3.1.1 */
	ec_secp224r1,			   /* 1.3.132.0.33 */
	ec_secp256r1 = 0x50,	   /* 1.2.840.10045.3.1.7 */
	ec_secp384r1,			   /* 1.3.132.0.34 */
	ec_brainpoolp512r1,		   /* 1.3.36.3.3.2.8.1.1.13 */
	ec_secp521r1,			   /* 1.3.132.0.35 */
};

struct bst_ecc_point {
	u64 *x;
	u64 *y;
	u8 ndigits;
};

#define PKE_ECC_POINT_INIT(x, y, ndigits) \
	((struct bst_ecc_point) { x, y, ndigits })

struct bst_ecdsa_ctx {
	const struct pke_ec_curve *pke_ec;
	unsigned int digest_len; /* parameter (bytes) */
	const char *digest;		 /* digest name from oid */
	unsigned int key_len;	 /* @key length (bytes) */
	const char *key;		 /* raw public key */
	struct bst_ecc_point pub_key;
	u64 _pubp[2][PKE_ECDSA_MAX_DIGITS]; /* point storage for @pub_key */
	enum ec_type ec_type;
};

// SM2 error code
enum SM2_RET_CODE {
	SM2_SUCCESS = 0,
	SM2_BUFFER_NULL = PKE_SUCCESS + 0x40,
	SM2_NOT_ON_CURVE,
	SM2_EXCHANGE_ROLE_INVALID,
	SM2_INPUT_INVALID,
	SM2_ZERO_ALL,
	SM2_INTEGER_TOO_BIG,
	SM2_VERIFY_FAILED,
	SM2_DECRYPT_VERIFY_FAILED
};

// TRNG return code
enum TRNG_RET_CODE {
	TRNG_SUCCESS = 0,
	TRNG_BUFFER_NULL,
	TRNG_INVALID_INPUT,
	TRNG_INVALID_CONFIG,
	TRNG_HT_ERROR,
	TRNG_ERROR
};

void print_buf_u32(uint32_t buf[], uint32_t word_len);
uint32_t get_valid_words(uint32_t *a, uint32_t max_words);
uint32_t uint32_big_num_check_zero(uint32_t a[], uint32_t a_word_len);
int32_t uint32_big_num_cmp(uint32_t *a, uint32_t a_word_len, uint32_t *b,
						   uint32_t b_word_len);
void uint32_clear(uint32_t *a, uint32_t wordLen);
void reverse_byte_array(uint8_t *in, uint8_t *out, uint32_t byte_len);
void pke_set_operand_uint32_value(uint32_t *a, uint32_t a_word_len,
								  uint32_t b);
uint32_t pke_pre_calc_mont(const uint32_t *modulus, uint32_t bit_len,
						   uint32_t *H);
uint32_t pke_mod_exp(const uint32_t *modulus, const uint32_t *exponent,
					 const uint32_t *base, uint32_t *out, uint32_t mod_word_len,
					 uint32_t exp_word_len);
uint32_t pke_mod_add(const uint32_t *modulus, const uint32_t *a,
					 const uint32_t *b, uint32_t *out, uint32_t word_len);
uint32_t eccp_point_mul_shamir_safe(const struct pke_ec_curve *curve, uint32_t *k1,
									uint32_t *P1x, uint32_t *P1y, uint32_t *k2, uint32_t *P2x,
									uint32_t *P2y, uint32_t *Qx, uint32_t *Qy);

uint32_t bst_rsa_mod_exp(MPI res, MPI base, MPI exp, MPI mod);
int bst_rsa_enc(struct akcipher_request *req);
int bst_rsa_dec(struct akcipher_request *req);
int bst_rsa_set_priv_key(struct crypto_akcipher *tfm, const void *key,
						 unsigned int keylen);
int bst_rsa_set_pub_key(struct crypto_akcipher *tfm, const void *key,
						unsigned int keylen);
unsigned int bst_rsa_max_size(struct crypto_akcipher *tfm);
int bst_rsa_init_tfm(struct crypto_akcipher *tfm);
void bst_rsa_exit_tfm(struct crypto_akcipher *tfm);

uint32_t pke_sm2_sign(const struct pke_ec_curve *curve, uint8_t E[32],
					  uint8_t rand_k[32], uint8_t pri_key[32], uint8_t signature[64]);
uint32_t pke_sm2_verify(const struct pke_ec_curve *sm2_curve, uint8_t E[32],
						uint8_t pub_key[65], uint8_t signature[64]);
int bst_sm2_verify(struct akcipher_request *req);
int bst_sm2_sign(struct akcipher_request *req);
int bst_sm2_set_pub_key(struct crypto_akcipher *tfm,
						const void *key, unsigned int keylen);
int bst_sm2_set_pri_key(struct crypto_akcipher *tfm,
						const void *key, unsigned int keylen);
unsigned int bst_sm2_max_size(struct crypto_akcipher *tfm);
int bst_sm2_init_tfm(struct crypto_akcipher *tfm);
void bst_sm2_exit_tfm(struct crypto_akcipher *tfm);

uint32_t pke_ecdsa_verify(const struct pke_ec_curve *curve, uint8_t *E,
						  uint32_t e_byte_len, uint8_t *pub_key_x, uint8_t *pub_key_y, uint8_t *signature);
uint32_t pke_ecdsa_sign(const struct pke_ec_curve *curve, uint8_t *E,
						  uint32_t e_byte_len, uint8_t *rand_k, uint8_t *priv_key, uint8_t *signature);
int bst_ecdsa_verify(struct akcipher_request *req);
int bst_ecdsa_sign(struct akcipher_request *req);
int bst_ecdsa_set_pub_key(struct crypto_akcipher *tfm, const void *key, unsigned int keylen);
int bst_ecdsa_set_priv_key(struct crypto_akcipher *tfm, const void *key, unsigned int keylen);
unsigned int bst_ecdsa_max_size(struct crypto_akcipher *tfm);
int bst_ecdsa_init_tfm(struct crypto_akcipher *tfm);
void bst_ecdsa_exit_tfm(struct crypto_akcipher *tfm);
extern void pke_enable_interrupt(void);
extern void pke_disable_interrupt(void);
#endif
