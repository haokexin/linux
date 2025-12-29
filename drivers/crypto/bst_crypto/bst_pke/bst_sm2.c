/* SPDX-License-Identifier: GPL-2.0-or-later
 * SM2 asymmetric public-key algorithm
 * as specified by OSCCA GM/T 0003.1-2012 -- 0003.5-2012 SM2 and
 * described at https://tools.ietf.org/html/draft-shen-sm2-ecdsa-02
 *
 * Copyright (c) 2020, Alibaba Group.
 * Copyright (C) 2024 Black Sesame Technologies. Inc.
 * Authors: Tianjia Zhang <tianjia.zhang@linux.alibaba.com>
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
#include <linux/mpi.h>
#include <crypto/algapi.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/mpi.h>
#include <crypto/internal/akcipher.h>
#include <crypto/akcipher.h>
#include <crypto/hash.h>
#include <crypto/sm3_base.h>
#include <crypto/rng.h>
#include <crypto/sm2.h>
#include "bst_pke.h"
#include "../common/bst_sa_common.h"

#define MPI_NBYTES(m)   ((mpi_get_nbits(m) + 7) / 8)
extern void sm2_print(uint8_t *array, uint8_t len, uint32_t line);
extern void reverse_byte_array(uint8_t *in, uint8_t *out, uint32_t byte_len);
struct bst_ecc_domain_parms {
	const char *desc;           /* Description of the curve.  */
	unsigned int nbits;         /* Number of bits.  */
	unsigned int fips:1; /* True if this is a FIPS140-2 approved curve */

	/* The model describing this curve.  This is mainly used to select
	 * the group equation.
	 */
	enum gcry_mpi_ec_models model;

	/* The actual ECC dialect used.  This is used for curve specific
	 * optimizations and to select encodings etc.
	 */
	enum ecc_dialects dialect;

	const char *p;              /* The prime defining the field.  */
	const char *p_h;
	const char *a, *b;          /* The coefficients.  For Twisted Edwards
				     * Curves b is used for d.  For Montgomery
				     * Curves (a,b) has ((A-2)/4,B^-1).
				     */
	const char *n;              /* The order of the base point.  */
	const char *n_h;
	const char *n_1;
	const char *g_x, *g_y;      /* Base point.  */
	const char *g_x_h, *g_y_h;
	unsigned int h;             /* Cofactor.  */
};

static const struct bst_ecc_domain_parms bst_sm2_ecp = {
	.desc = "sm2p256v1",
	.nbits = 256,
	.fips = 0,
	.model = MPI_EC_WEIERSTRASS,
	.dialect = ECC_DIALECT_STANDARD,
	.p   = "0xfffffffeffffffffffffffffffffffffffffffff00000000ffffffffffffffff",
	.a   = "0xfffffffeffffffffffffffffffffffffffffffff00000000fffffffffffffffc",
	.b   = "0x28e9fa9e9d9f5e344d5a9e4bcf6509a7f39789f515ab8f92ddbcbd414d940e93",
	.n   = "0xfffffffeffffffffffffffffffffffff7203df6b21c6052b53bbf40939d54123",
	.g_x = "0x32c4ae2c1f1981195f9904466a39c9948fe30bbff2660be1715a4589334c74c7",
	.g_y = "0xbc3736a2f4f6779c59bdcee36b692153d0a9877cc62a474002df32e52139f0a0",
	.h = 1,
	.p_h = "0x0000000400000002000000010000000100000002FFFFFFFF0000000200000003",
	.n_h = "0x1EB5E412A22B3D3B620FC84C3AFFE0D43464504ADE6FA2FA901192AF7C114F20",
	.n_1 = "0xfffffffeffffffffffffffffffffffff7203df6b21c6052b53bbf40939d54122",
	.g_x_h = "0xB692E5B574D55DA93DB7B24888C21F3A2B2308F6484E1B38EAE3D9A9D13A42ED",
	.g_y_h = "0xA175051B0F3FB6135A924F85544926F9DB61AC1773438E6DD186469DE295E5AB"
};

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

static int bst_sm2_ec_ctx_init(struct bst_mpi_ec_ctx *bst_ec)
{
	const struct bst_ecc_domain_parms *ecp = &bst_sm2_ecp;
	MPI p, a, b;
	MPI x, y;

	int rc = -EINVAL;

	p = mpi_scanval(ecp->p);
	a = mpi_scanval(ecp->a);
	b = mpi_scanval(ecp->b);
	if (!p || !a || !b)
		goto free_p;

	x = mpi_scanval(ecp->g_x);
	y = mpi_scanval(ecp->g_y);

	if (!x || !y)
		goto free;

	rc = -ENOMEM;

	bst_ec->ec.Q = mpi_point_new(0);
	if (!bst_ec->ec.Q)
		goto free;

	/* mpi_ec_setup_elliptic_curve */
	bst_ec->ec.G = mpi_point_new(0);
	if (!bst_ec->ec.G) {
		mpi_point_release(bst_ec->ec.Q);
		goto free;
	}

	mpi_set(bst_ec->ec.G->x, x);
	mpi_set(bst_ec->ec.G->y, y);
	mpi_set_ui(bst_ec->ec.G->z, 1);

	rc = -EINVAL;
	bst_ec->ec.n = mpi_scanval(ecp->n);
	if (!bst_ec->ec.n) {
		mpi_point_release(bst_ec->ec.Q);
		mpi_point_release(bst_ec->ec.G);
		goto free;
	}

	bst_ec->ec.h = ecp->h;
	bst_ec->ec.name = ecp->desc;
	mpi_ec_init(&bst_ec->ec, ecp->model, ecp->dialect, 0, p, a, b);

	bst_ec->p_h = mpi_scanval(ecp->p_h);
	bst_ec->n_h = mpi_scanval(ecp->n_h);
	bst_ec->n_1 = mpi_scanval(ecp->n_1);
	bst_ec->g_x_h = mpi_scanval(ecp->g_x_h);
	bst_ec->g_y_h = mpi_scanval(ecp->g_y_h);

	rc = 0;

free:
	mpi_free(x);
	mpi_free(y);
free_p:
	mpi_free(p);
	mpi_free(a);
	mpi_free(b);

	return rc;
}

static void bst_sm2_ec_ctx_deinit(struct bst_mpi_ec_ctx *bst_ec)
{
	mpi_ec_deinit(&bst_ec->ec);

	mpi_free(bst_ec->p_h);
	mpi_free(bst_ec->n_h);
	mpi_free(bst_ec->n_1);
	mpi_free(bst_ec->g_x_h);
	mpi_free(bst_ec->g_y_h);
	memset(bst_ec, 0, sizeof(*bst_ec));
}

/* RESULT must have been initialized and is set on success to the
 * point given by VALUE.
 */
static int bst_sm2_ecc_os2ec(MPI_POINT result, MPI value)
{
	int rc;
	size_t n;
	unsigned char *buf;
	MPI x, y;

	n = MPI_NBYTES(value);
	buf = kmalloc(n, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	rc = mpi_print(GCRYMPI_FMT_USG, buf, n, &n, value);
	if (rc)
		goto err_freebuf;

	rc = -EINVAL;
	if (n < 1 || ((n - 1) % 2))
		goto err_freebuf;
	/* No support for point compression */
	if (*buf != 0x4)
		goto err_freebuf;

	rc = -ENOMEM;
	n = (n - 1) / 2;
	x = mpi_read_raw_data(buf + 1, n);
	if (!x)
		goto err_freebuf;
	y = mpi_read_raw_data(buf + 1 + n, n);
	if (!y)
		goto err_freex;

	mpi_normalize(x);
	mpi_normalize(y);
	mpi_set(result->x, x);
	mpi_set(result->y, y);
	mpi_set_ui(result->z, 1);

	rc = 0;

	mpi_free(y);
err_freex:
	mpi_free(x);
err_freebuf:
	bst_kfree(buf);
	return rc;
}

struct bst_sm2_signature_ctx {
	MPI sig_r;
	MPI sig_s;
};

int bst_sm2_get_signature_r(void *context, size_t hdrlen, unsigned char tag,
				const void *value, size_t vlen)
{
	struct bst_sm2_signature_ctx *sig = context;

	if (!value || !vlen)
		return -EINVAL;

	sig->sig_r = mpi_read_raw_data(value, vlen);
	if (!sig->sig_r)
		return -ENOMEM;

	return 0;
}

int bst_sm2_get_signature_s(void *context, size_t hdrlen, unsigned char tag,
				const void *value, size_t vlen)
{
	struct bst_sm2_signature_ctx *sig = context;

	if (!value || !vlen)
		return -EINVAL;

	sig->sig_s = mpi_read_raw_data(value, vlen);
	if (!sig->sig_s)
		return -ENOMEM;

	return 0;
}

static int bst_sm2_z_digest_update(struct shash_desc *desc,
			MPI m, unsigned int pbytes)
{
	static const unsigned char zero[32];
	unsigned char *in;
	unsigned int inlen;

	in = mpi_get_buffer(m, &inlen, NULL);
	if (!in)
		return -EINVAL;

	if (inlen < pbytes) {
		/* padding with zero */
		crypto_sm3_update(desc, zero, pbytes - inlen);
		crypto_sm3_update(desc, in, inlen);
	} else if (inlen > pbytes) {
		/* skip the starting zero */
		crypto_sm3_update(desc, in + inlen - pbytes, pbytes);
	} else {
		crypto_sm3_update(desc, in, inlen);
	}

	bst_kfree(in);
	return 0;
}

static int bst_sm2_z_digest_update_point(struct shash_desc *desc,
		MPI_POINT point, struct mpi_ec_ctx *ec, unsigned int pbytes)
{
	MPI x, y;
	int ret = -EINVAL;

	x = mpi_new(0);
	y = mpi_new(0);

	if (!mpi_ec_get_affine(x, y, point, ec) &&
		!bst_sm2_z_digest_update(desc, x, pbytes) &&
		!bst_sm2_z_digest_update(desc, y, pbytes))
		ret = 0;

	mpi_free(x);
	mpi_free(y);
	return ret;
}

int bst_sm2_compute_z_digest(struct crypto_akcipher *tfm,
			const unsigned char *id, size_t id_len,
			unsigned char dgst[SM3_DIGEST_SIZE])
{
	struct bst_mpi_ec_ctx *bst_ec = akcipher_tfm_ctx(tfm);
	struct mpi_ec_ctx *ec = &bst_ec->ec;
	uint16_t bits_len;
	unsigned char entl[2];
	SHASH_DESC_ON_STACK(desc, NULL);
	unsigned int pbytes;

	if (id_len > (USHRT_MAX / 8) || !ec->Q)
		return -EINVAL;

	bits_len = (uint16_t)(id_len * 8);
	entl[0] = bits_len >> 8;
	entl[1] = bits_len & 0xff;

	pbytes = MPI_NBYTES(ec->p);

	/* ZA = H256(ENTLA | IDA | a | b | xG | yG | xA | yA) */
	sm3_base_init(desc);
	crypto_sm3_update(desc, entl, 2);
	crypto_sm3_update(desc, id, id_len);

	if (bst_sm2_z_digest_update(desc, ec->a, pbytes) ||
		bst_sm2_z_digest_update(desc, ec->b, pbytes) ||
		bst_sm2_z_digest_update_point(desc, ec->G, ec, pbytes) ||
		bst_sm2_z_digest_update_point(desc, ec->Q, ec, pbytes))
		return -EINVAL;

	crypto_sm3_final(desc, dgst);
	return 0;
}
EXPORT_SYMBOL(bst_sm2_compute_z_digest);

int bst_sm2_verify(struct akcipher_request *req)
{
	struct crypto_akcipher *tfm = crypto_akcipher_reqtfm(req);
	struct bst_mpi_ec_ctx *bst_ec = akcipher_tfm_ctx(tfm);
	struct mpi_ec_ctx *ec = &(bst_ec->ec);
	unsigned char *buffer = NULL;
	char result[1];
	int ret;
	struct pke_ec_curve pke_ec;

	if (unlikely(!ec->Q))
		return -EINVAL;

	buffer = kmalloc(req->src_len, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	sg_pcopy_to_buffer(req->src,
		sg_nents_for_len(req->src, req->src_len),
		buffer, req->src_len, 0);

	pke_ec.n_bit_len = bst_ec->ec.nbits;
	pke_ec.p_bit_len = bst_ec->ec.nbits;
	pke_ec.eccp_a = (uint32_t *)ec->a->d;
	pke_ec.eccp_b = (uint32_t *)ec->b->d;
	pke_ec.eccp_Gx = (uint32_t *)ec->G->x->d;
	pke_ec.eccp_Gy = (uint32_t *)ec->G->y->d;
	pke_ec.eccp_n = (uint32_t *)ec->n->d;
	pke_ec.eccp_p = (uint32_t *)ec->p->d;
	pke_ec.eccp_n_h = (uint32_t *)bst_ec->n_h->d;
	pke_ec.eccp_p_h = (uint32_t *)bst_ec->p_h->d;
	pke_ec.eccp_half_Gx = (uint32_t *)bst_ec->g_x_h->d;
	pke_ec.eccp_half_Gy = (uint32_t *)bst_ec->g_y_h->d;
	ret = -ENOMEM;

	if (pke_sm2_verify(&pke_ec, (uint8_t *)(buffer + SM2_BYTE_LEN * 2),
		bst_ec->key, (uint8_t *)buffer) == SM2_SUCCESS) {
		ret = 0;
		result[0] = 0;
	}
	else {
		ret = 1;
		result[0] = 1;
	}
	bst_kfree(buffer);

	sg_copy_part_from_buf(req->dst, result, sizeof(result), 0);
	// sg_copy_from_buffer(req->dst, sg_nents_for_len(req->dst, req->dst_len), result, sizeof(result));

	return ret;
}

int bst_sm2_set_pub_key(struct crypto_akcipher *tfm,
			const void *key, unsigned int keylen)
{
	struct bst_mpi_ec_ctx *bst_ec = akcipher_tfm_ctx(tfm);
	MPI a;
	int rc;

	/* include the uncompressed flag '0x04' */
	a = mpi_read_raw_data(key, keylen);
	if (!a)
		return -ENOMEM;

	mpi_normalize(a);
	rc = bst_sm2_ecc_os2ec(bst_ec->ec.Q, a);
	mpi_free(a);
	bst_kfree(bst_ec->key);
	bst_ec->key = kmalloc(keylen, GFP_KERNEL);
	if (bst_ec->key == NULL)
		return -ENOMEM;
	
	memcpy(bst_ec->key, key, keylen);
	return rc;
}


static int _bst_sm2_sign(struct bst_mpi_ec_ctx *bst_ec, MPI hash, uint8_t *signature)
{
	int rc = -EINVAL;
	uint32_t ret;
	struct mpi_ec_ctx *ec = &(bst_ec->ec);
	struct pke_ec_curve pke_ec;

	pke_ec.n_bit_len = bst_ec->ec.nbits;
	pke_ec.p_bit_len = bst_ec->ec.nbits;
	pke_ec.eccp_a = (uint32_t *)ec->a->d;
	pke_ec.eccp_b = (uint32_t *)ec->b->d;
	pke_ec.eccp_Gx = (uint32_t *)ec->G->x->d;
	pke_ec.eccp_Gy = (uint32_t *)ec->G->y->d;
	pke_ec.eccp_n = (uint32_t *)ec->n->d;
	pke_ec.eccp_p = (uint32_t *)ec->p->d;
	pke_ec.eccp_n_h = (uint32_t *)bst_ec->n_h->d;
	pke_ec.eccp_p_h = (uint32_t *)bst_ec->p_h->d;
	pke_ec.eccp_half_Gx = (uint32_t *)bst_ec->g_x_h->d;
	pke_ec.eccp_half_Gy = (uint32_t *)bst_ec->g_y_h->d;
	pke_ec.eccp_n_1 = (uint32_t *)bst_ec->n_1->d;

	rc = EKEYREJECTED;
	ret = pke_sm2_sign(&pke_ec, (uint8_t *)hash->d, NULL, (uint8_t *)ec->d->d, signature);
	if (ret != SM2_SUCCESS)
		goto leave;

	rc = 0;
leave:

	return rc;
}


int bst_sm2_sign(struct akcipher_request *req)
{
	struct crypto_akcipher *tfm = crypto_akcipher_reqtfm(req);
	struct bst_mpi_ec_ctx *bst_ec = akcipher_tfm_ctx(tfm);
	struct mpi_ec_ctx *ec = &(bst_ec->ec);
	unsigned char *buffer;
	unsigned char *reverse_hash = NULL;
	uint8_t *signature;
	MPI hash;
	int ret;
	// bst_dbg(2, "%s:%d\n", __func__, __LINE__);
	if (unlikely(!ec->Q))
		return -EINVAL;
	// bst_dbg(2, "%s:%d 0x%x, 0x%x\n", __func__, __LINE__, req->src_len, req->dst_len);
	buffer = kmalloc(req->src_len, GFP_KERNEL);
	signature = kmalloc(SM2_BYTE_LEN*2, GFP_KERNEL);
	if (!buffer || !signature) {
		KFreeMem(buffer);
		KFreeMem(signature);
		return -ENOMEM;
	}
	// bst_dbg(2, "%s:%d\n", __func__, __LINE__);
	sg_pcopy_to_buffer(req->src,
		sg_nents_for_len(req->src, req->src_len),
		buffer, req->src_len, 0);

	// bst_dbg(2, "%s:%d\n", __func__, __LINE__);
	ret = -ENOMEM;
	//hash = mpi_read_raw_data(buffer + req->src_len, req->dst_len);
	reverse_hash = kmalloc(32, GFP_KERNEL);
	if (!reverse_hash)
		goto error;
	reverse_byte_array(buffer + req->src_len - 32, reverse_hash, 32);
	hash = mpi_read_raw_data(reverse_hash, 32);
	if (!hash)
		goto error;
	// bst_dbg(2, "%s:%d\n", __func__, __LINE__);
	ret = _bst_sm2_sign(bst_ec, hash, signature);
	// bst_dbg(2, "%s:%d ret = %d\n", __func__, __LINE__, ret);

	sg_copy_part_from_buf(req->dst, signature, SM2_BYTE_LEN*2, 0);

	mpi_free(hash);
error:
	bst_kfree(buffer);
	bst_kfree(signature);
	bst_kfree(reverse_hash);
	return ret;
}

int bst_sm2_set_pri_key(struct crypto_akcipher *tfm,
			const void *key, unsigned int keylen)
{
	struct bst_mpi_ec_ctx *bst_ec = akcipher_tfm_ctx(tfm);
	MPI a;
	int rc;
	uint8_t *reverse_key = NULL;

	// sm2_print((uint8_t *)key, 10, __LINE__);
	reverse_key = kmalloc(keylen, GFP_KERNEL);
	if (!reverse_key)
		return -ENOMEM;
	reverse_byte_array((uint8_t *)key, reverse_key, keylen);
	/* include the uncompressed flag '0x04' */
	a = mpi_read_raw_data(reverse_key, keylen);
	if (!a) {
		bst_kfree(reverse_key);
		return -ENOMEM;
	}
	mpi_normalize(a);
	bst_ec->ec.d = a;
	rc = 0;
	// sm2_print((uint8_t *)bst_ec->ec.d->d, 10, __LINE__);
	bst_kfree(reverse_key);
	return rc;
}

unsigned int bst_sm2_max_size(struct crypto_akcipher *tfm)
{
	struct bst_mpi_ec_ctx *ctx = akcipher_tfm_ctx(tfm);
	
	return mpi_get_size(ctx->n_h) * 2;
}

int bst_sm2_init_tfm(struct crypto_akcipher *tfm)
{
	struct bst_mpi_ec_ctx *bst_ec = akcipher_tfm_ctx(tfm);

	bst_ec->key = NULL;
	bst_ec->key_len = 0;
	pke_enable_interrupt();
	return bst_sm2_ec_ctx_init(bst_ec);
}

void bst_sm2_exit_tfm(struct crypto_akcipher *tfm)
{
	struct bst_mpi_ec_ctx *bst_ec = akcipher_tfm_ctx(tfm);

	bst_kfree(bst_ec->key);
	bst_ec->key_len = 0;
	bst_sm2_ec_ctx_deinit(bst_ec);
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("BST SM2 driver");
