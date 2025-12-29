// SPDX-License-Identifier: GPL-2.0
/*
 * Elliptic Curve (Russian) Digital Signature Algorithm for Cryptographic API
 *
 * Copyright (c) 2019 Vitaly Chikunov <vt@altlinux.org>
 * Copyright (C) 2024 Black Sesame Technologies. Inc.
 *
 * References:
 * GOST 34.10-2018, GOST R 34.10-2012, RFC 7091, ISO/IEC 14888-3:2018.
 *
 * Historical references:
 * GOST R 34.10-2001, RFC 4357, ISO/IEC 14888-3:2006/Amd 1:2010.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#include <linux/module.h>
#include <linux/crypto.h>
#include <crypto/streebog.h>
#include <crypto/internal/akcipher.h>
#include <crypto/akcipher.h>
#include <linux/oid_registry.h>
#include <linux/scatterlist.h>
#include <asm/unaligned.h>
#include "bst_pke.h"
#include "eccp_curve.h"
#include "../common/bst_sa_common.h"

static int sg_copy_part_from_buf(struct scatterlist *dest, u8 *src,
			   unsigned int len, unsigned int skip)
{
	size_t copied;
	unsigned int nents = sg_nents(dest);

	copied = sg_pcopy_from_buffer(dest, nents, src, len, skip);
	if (copied != len)
		return -1;

	return 0;
}
/*
static void ecdsa_print_key(uint8_t *in, uint8_t len, char *str)
{
	uint8_t i;
	if (in == NULL) {
		bst_dbg(1, "%s print input is NULL", str);
		return;
	}
	bst_dbg(1, "%s print:", str);
	for (i = 0; i < len; i++) {
		bst_dbg(1, "0x%02x", in[i]);
	}
}
*/
int bst_ecdsa_verify(struct akcipher_request *req)
{
	struct crypto_akcipher *tfm = crypto_akcipher_reqtfm(req);
	struct bst_ecdsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	unsigned char *signature = NULL;
	unsigned char *digest = NULL;
	// unsigned char signature[PKE_ECDSA_MAX_SIG_SIZE];
	// unsigned char digest[PKE_ECDSA_DIGEST_SIZE];
	unsigned int sign_len = ctx->key_len;
	char result[1];
	uint32_t ret;

	/*
	 * Digest value, digest algorithm, and curve (modulus) should have the
	 * same length (256 or 512 bits), public key and signature should be
	 * twice bigger.
	 */
	if (!ctx->pke_ec ||
	    !ctx->digest ||
	    !req->src ||
	    !ctx->pub_key.x
		)
		return -EBADMSG;

	signature = kmalloc(sign_len, GFP_KERNEL);
	digest = kmalloc(ctx->digest_len, GFP_KERNEL);
	if (!signature || !digest) {
		KFreeMem(signature);
		KFreeMem(digest);
		return -ENOMEM;
	}
	/*
	 * req->src: signature + digest
	 * req->src_len: signature length + digest length
	 * ctx->key_len: public key length( = signature length)
	 * ctx->digest_len: digest length
	 */
	sg_copy_to_buffer(req->src, sg_nents_for_len(req->src, sign_len), signature, sign_len);
	sg_pcopy_to_buffer(req->src, sg_nents_for_len(req->src, req->src_len), digest, ctx->digest_len, sign_len);

	if (pke_ecdsa_verify(ctx->pke_ec, digest, ctx->digest_len, (uint8_t *)ctx->pub_key.x,
		(uint8_t *)ctx->pub_key.y, signature) == ECDSA_SUCCESS) {	
		ret = 0;
		result[0] = 0;
	} else {
		ret = 1;
		result[0] = 1;
	}
	sg_copy_part_from_buf(req->dst, result, sizeof(result), 0);

	bst_kfree(signature);
	bst_kfree(digest);

	return ret;
}

int bst_ecdsa_sign(struct akcipher_request *req)
{
	struct crypto_akcipher *tfm = crypto_akcipher_reqtfm(req);
	struct bst_ecdsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	unsigned int sign_len = ctx->key_len*2;
	unsigned char *signature = NULL;
	unsigned char *digest = NULL;
	// unsigned char signature[PKE_ECDSA_MAX_SIG_SIZE];
	// unsigned char digest[PKE_ECDSA_DIGEST_SIZE];
	int rc = -EINVAL;
	uint32_t ret;

	/*
	 * Digest value, digest algorithm, as well as the private key and curve (modulus) should have the
	 * same length (256 or 512 bits), and signature should be twice bigger.
	 */
	if (!ctx->pke_ec ||
	    !ctx->digest ||
	    !req->src ||
	    !ctx->key
		)
		return -EBADMSG;

	signature = kmalloc(sign_len, GFP_KERNEL);
	digest = kmalloc(ctx->digest_len, GFP_KERNEL);
	if (!signature || !digest) {
		KFreeMem(signature);
		KFreeMem(digest);
		return -ENOMEM;
	}

	sg_pcopy_to_buffer(req->src, sg_nents_for_len(req->src, req->src_len), digest, ctx->digest_len, 0);

	rc = EKEYREJECTED;
	ret = pke_ecdsa_sign(ctx->pke_ec, digest, ctx->digest_len, NULL, (uint8_t *)ctx->key, signature);

	if (ret != ECDSA_SUCCESS)
		goto leave;
	rc = 0;
	sg_copy_part_from_buf(req->dst, signature, sign_len, 0);

leave:
	bst_kfree(signature);
	bst_kfree(digest);

	return rc;
}

void bst_vli_from_le64(u64 *dest, const void *src, unsigned int ndigits)
{
	int i;
	const u64 *from = src;

	for (i = 0; i < ndigits; i++)
		dest[i] = get_unaligned_le64(&from[i]);
}
EXPORT_SYMBOL(bst_vli_from_le64);

/* Parse BER encoded subjectPublicKey. 
 * Identifier(uin8) | Length(uin8) | Contents(uint8[]) | Algo(uin32)
*/
int bst_ecdsa_set_pub_key(struct crypto_akcipher *tfm, const void *key,
			      unsigned int keylen)
{
	struct bst_ecdsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	//unsigned int ndigits;
	uint32_t algo;
	uint8_t *params;

	params = (uint8_t *)key;
	if (keylen < (6 + params[1]))
		return -ENOPKG;

	algo = params[2 + params[1]];
	if (algo == ec_brainpoolp160r1) {
		ctx->ec_type = ec_brainpoolp160r1;
		ctx->digest	= "brainpoolp160r1";
		ctx->digest_len	= 160 / 8;
		ctx->pke_ec = &pke_brainpoolp160r1[0];
	} else if (algo == ec_secp192r1) {
		ctx->ec_type = ec_secp192r1;
		ctx->digest	= "secp192r1";
		ctx->digest_len	= 192 / 8;
		ctx->pke_ec = &pke_secp192r1[0];
	} else if (algo == ec_secp224r1) {
		ctx->ec_type = ec_secp224r1;
		ctx->digest	= "secp224r1";
		ctx->digest_len	= 224 / 8;
		ctx->pke_ec = &pke_secp224r1[0];
	} else if (algo == ec_secp256r1) {
		ctx->ec_type = ec_secp256r1;
		ctx->digest	= "secp256r1";
		ctx->digest_len	= 256 / 8;
		ctx->pke_ec = &pke_secp256r1[0];
	} else if (algo == ec_secp384r1) {
		ctx->ec_type = ec_secp384r1;
		ctx->digest	= "secp384r1";
		ctx->digest_len	= 384 / 8;
		ctx->pke_ec = &pke_secp384r1[0];
	} else if (algo == ec_brainpoolp512r1) {
		ctx->ec_type = ec_brainpoolp512r1;
		ctx->digest	= "brainpoolp512r1";
		ctx->digest_len	= 512 / 8;
		ctx->pke_ec = &pke_brainpoolp512r1[0];
	} else if (algo == ec_secp521r1) {
		ctx->ec_type = ec_secp521r1;
		ctx->digest	= "ec_secp521r1";
		ctx->digest_len	= 528 / 8;
		ctx->pke_ec = &pke_secp521r1[0];
	} else
		return -ENOPKG;
		
	ctx->key_len = params[1];
	bst_kfree(ctx->key);
	ctx->key = kmalloc(ctx->key_len, GFP_KERNEL);
	if (ctx->key == NULL)
		return -ENOMEM;
	memcpy((uint8_t *)ctx->key, (uint8_t *)(&params[2]), ctx->key_len);
	/*
	 * Sizes of algo (set in digest_len) and curve should match
	 * each other.
	 */
	if (!ctx->pke_ec ||
	    (ctx->pke_ec->n_bit_len + 7) / 8 != ctx->digest_len)
		return -ENOPKG;

	/*
	 * PubKey is two 256- or 512-bit coordinates which should match
	 * curve size.
	 */
	if (ctx->key_len / 2 != (ctx->pke_ec->n_bit_len + 7) / 8)
		return -ENOPKG;

	ctx->pub_key = PKE_ECC_POINT_INIT(ctx->_pubp[0], ctx->_pubp[1], ctx->key_len/2);
	memcpy(ctx->pub_key.x, ctx->key, ctx->key_len/2);
	memcpy(ctx->pub_key.y, ctx->key + ctx->key_len/2, ctx->key_len/2);

	return 0;
}

int bst_ecdsa_set_priv_key(struct crypto_akcipher *tfm, const void *key,
			      unsigned int keylen)
{
	struct bst_ecdsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	uint32_t algo;
	uint8_t *params = (uint8_t *)key;

	if (keylen < (6 + params[1]))
		return -ENOPKG;

	algo = (params[2 + params[1]]) | (params[3 + params[1]] << 8) | (params[4 + params[1]] << 16) | (params[5 + params[1]] << 24);
	if (algo == ec_brainpoolp160r1) {
		ctx->ec_type = ec_brainpoolp160r1;
		ctx->digest	= "brainpoolp160r1";
		ctx->digest_len	= 160 / 8;
		ctx->pke_ec = &pke_brainpoolp160r1[0];
	} else if (algo == ec_secp192r1) {
		ctx->ec_type = ec_secp192r1;
		ctx->digest	= "secp192r1";
		ctx->digest_len	= 192 / 8;
		ctx->pke_ec = &pke_secp192r1[0];
	} else if (algo == ec_secp224r1) {
		ctx->ec_type = ec_secp224r1;
		ctx->digest	= "secp224r1";
		ctx->digest_len	= 224 / 8;
		ctx->pke_ec = &pke_secp224r1[0];
	} else if (algo == ec_secp256r1) {
		ctx->ec_type = ec_secp256r1;
		ctx->digest	= "secp256r1";
		ctx->digest_len	= 256 / 8;
		ctx->pke_ec = &pke_secp256r1[0];
	} else if (algo == ec_secp384r1) {
		ctx->ec_type = ec_secp384r1;
		ctx->digest	= "secp384r1";
		ctx->digest_len	= 384 / 8;
		ctx->pke_ec = &pke_secp384r1[0];
	} else if (algo == ec_brainpoolp512r1) {
		ctx->ec_type = ec_brainpoolp512r1;
		ctx->digest	= "brainpoolp512r1";
		ctx->digest_len	= 512 / 8;
		ctx->pke_ec = &pke_brainpoolp512r1[0];
	} else if (algo == ec_secp521r1) {
		ctx->ec_type = ec_secp521r1;
		ctx->digest	= "ec_secp521r1";
		ctx->digest_len	= 528 / 8;
		ctx->pke_ec = &pke_secp521r1[0];
	} else
		return -ENOPKG;
	
	ctx->key_len = params[1];
	bst_kfree(ctx->key);
	ctx->key = kmalloc(ctx->key_len, GFP_KERNEL);
	if (ctx->key == NULL)
		return -ENOMEM;
	memcpy((uint8_t *)ctx->key, (uint8_t *)(&params[2]), ctx->key_len);

	/*
	 * Sizes of algo (set in digest_len) and curve should match
	 * each other.
	 */
	if (!ctx->pke_ec ||
	    (ctx->pke_ec->n_bit_len + 7) / 8 != ctx->digest_len)
		return -ENOPKG;

	/*
	 * PrivKey is 256- or 512-bit coordinates which should match
	 * curve size.
	 */
	if (ctx->key_len != (ctx->pke_ec->n_bit_len + 7) / 8)
		return -ENOPKG;
	
	return 0;
}

unsigned int bst_ecdsa_max_size(struct crypto_akcipher *tfm)
{
	struct bst_ecdsa_ctx *ctx = akcipher_tfm_ctx(tfm);

	/*
	 * Verify doesn't need any output, so it's just informational
	 * for keyctl to determine the key bit size.
	 */
	return (ctx->pke_ec->n_bit_len + 7) / 8 * 2;
	// return ctx->pub_key.ndigits * sizeof(u64);
}

void bst_ecdsa_exit_tfm(struct crypto_akcipher *tfm)
{
}

int bst_ecdsa_init_tfm(struct crypto_akcipher *tfm)
{
	pke_enable_interrupt();
	return 0;
}

MODULE_DESCRIPTION("BST ECDSA algorithm");
MODULE_ALIAS_CRYPTO("bst ecdsa");
