/* SPDX-License-Identifier: GPL-2.0-or-later
 * RSA asymmetric public-key algorithm [RFC3447]
 *
 * Copyright (c) 2015, Intel Corporation
 * Copyright (C) 2024 Black Sesame Technologies. Inc.
 * Authors: Tadeusz Struk <tadeusz.struk@intel.com>
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
#include <crypto/internal/rsa.h>
#include <crypto/internal/akcipher.h>
#include <crypto/akcipher.h>
#include <crypto/algapi.h>
#include <linux/types.h>
#include <linux/slab.h>
#include "bst_pke.h"
#include "../common/bst_sa_common.h"

//RSA return code
static void bst_rsa_free_mpi_key(struct bst_rsa_mpi_key *key)
{
	mpi_free(key->d);
	mpi_free(key->e);
	mpi_free(key->n);
	key->d = NULL;
	key->e = NULL;
	key->n = NULL;
}

/*
 * RSAEP function [RFC3447 sec 5.1.1]
 * c = m^e mod n;
 */
static int pke_rsa_enc(const struct bst_rsa_mpi_key *key, MPI c, MPI m)
{
	/* (1) Validate 0 <= m < n */
	if (mpi_cmp_ui(m, 0) < 0 || mpi_cmp(m, key->n) >= 0)
		return -EINVAL;
	return bst_rsa_mod_exp(c, m, key->e, key->n);
}

/*
 * RSADP function [RFC3447 sec 5.1.2]
 * m = c^d mod n;
 */
static int pke_rsa_dec(const struct bst_rsa_mpi_key *key, MPI m, MPI c)
{
	/* (1) Validate 0 <= c < n */
	if (mpi_cmp_ui(c, 0) < 0 || mpi_cmp(c, key->n) >= 0)
		return -EINVAL;

	return bst_rsa_mod_exp(m, c, key->d, key->n);
}

static inline struct bst_rsa_mpi_key *bst_rsa_get_key(
					struct crypto_akcipher *tfm)
{
	return akcipher_tfm_ctx(tfm);
}

int bst_rsa_enc(struct akcipher_request *req)
{
	struct crypto_akcipher *tfm = crypto_akcipher_reqtfm(req);
	const struct bst_rsa_mpi_key *pkey = bst_rsa_get_key(tfm);
	MPI m, c = mpi_alloc(0);
	int ret = 0;
	int sign;

	if (!c)
		return -ENOMEM;

	if (unlikely(!pkey->n || !pkey->e)) {
		ret = -EINVAL;
		goto err_free_c;
	}

	ret = -ENOMEM;
	m = mpi_read_raw_from_sgl(req->src, req->src_len);
	if (!m)
		goto err_free_c;
	ret = pke_rsa_enc(pkey, c, m);
	if (ret)
		goto err_free_m;

	ret = mpi_write_to_sgl(c, req->dst, req->dst_len, &sign);
	if (ret)
		goto err_free_m;

	if (sign < 0)
		ret = -EBADMSG;
err_free_m:
	mpi_free(m);
err_free_c:
	mpi_free(c);
	return ret;
}

int bst_rsa_dec(struct akcipher_request *req)
{
	struct crypto_akcipher *tfm = crypto_akcipher_reqtfm(req);
	const struct bst_rsa_mpi_key *pkey = bst_rsa_get_key(tfm);
	MPI c, m = mpi_alloc(0);
	int ret = 0;
	int sign;

	if (!m)
		return -ENOMEM;

	if (unlikely(!pkey->n || !pkey->d)) {
		ret = -EINVAL;
		goto err_free_m;
	}

	ret = -ENOMEM;
	c = mpi_read_raw_from_sgl(req->src, req->src_len);
	if (!c)
		goto err_free_m;

	ret = pke_rsa_dec(pkey, m, c);
	if (ret)
		goto err_free_c;

	ret = mpi_write_to_sgl(m, req->dst, req->dst_len, &sign);
	if (ret)
		goto err_free_c;

	if (sign < 0)
		ret = -EBADMSG;
err_free_c:
	mpi_free(c);
err_free_m:
	mpi_free(m);
	return ret;
}

static int bst_rsa_check_key_length(unsigned int len)
{
	switch (len) {
	case 1024:
	case 2048:
	case 4096:
		return 0;
	}

	return -EINVAL;
}

int bst_rsa_set_pub_key(struct crypto_akcipher *tfm, const void *key,
			   unsigned int keylen)
{
	struct bst_rsa_mpi_key *mpi_key = akcipher_tfm_ctx(tfm);
	struct rsa_key raw_key = {0};
	int ret;

	/* Free the old MPI key if any */
	bst_rsa_free_mpi_key(mpi_key);

	ret = rsa_parse_pub_key(&raw_key, key, keylen);
	if (ret)
		return ret;

	mpi_key->e = mpi_read_raw_data(raw_key.e, raw_key.e_sz);
	if (!mpi_key->e)
		goto err;

	mpi_key->n = mpi_read_raw_data(raw_key.n, raw_key.n_sz);
	if (!mpi_key->n)
		goto err;

	if (bst_rsa_check_key_length(mpi_get_size(mpi_key->n) << 3)) {
		bst_rsa_free_mpi_key(mpi_key);
		return -EINVAL;
	}

	return 0;

err:
	bst_rsa_free_mpi_key(mpi_key);
	return -ENOMEM;
}

int bst_rsa_set_priv_key(struct crypto_akcipher *tfm, const void *key,
			    unsigned int keylen)
{
	struct bst_rsa_mpi_key *mpi_key = akcipher_tfm_ctx(tfm);
	struct rsa_key raw_key = {0};
	int ret;

	/* Free the old MPI key if any */
	bst_rsa_free_mpi_key(mpi_key);

	ret = rsa_parse_priv_key(&raw_key, key, keylen);
	if (ret)
		return ret;

	mpi_key->d = mpi_read_raw_data(raw_key.d, raw_key.d_sz);
	if (!mpi_key->d)
		goto err;

	mpi_key->e = mpi_read_raw_data(raw_key.e, raw_key.e_sz);
	if (!mpi_key->e)
		goto err;

	mpi_key->n = mpi_read_raw_data(raw_key.n, raw_key.n_sz);
	if (!mpi_key->n)
		goto err;

	if (bst_rsa_check_key_length(mpi_get_size(mpi_key->n) << 3)) {
		bst_rsa_free_mpi_key(mpi_key);
		return -EINVAL;
	}

	return 0;

err:
	bst_rsa_free_mpi_key(mpi_key);
	return -ENOMEM;
}

unsigned int bst_rsa_max_size(struct crypto_akcipher *tfm)
{
	struct bst_rsa_mpi_key *pkey = akcipher_tfm_ctx(tfm);

	return mpi_get_size(pkey->n);
}

int bst_rsa_init_tfm(struct crypto_akcipher *tfm)
{
	struct bst_rsa_mpi_key *pkey = akcipher_tfm_ctx(tfm);

	bst_rsa_free_mpi_key(pkey);
	pke_enable_interrupt();
	
	return 0;
}

void bst_rsa_exit_tfm(struct crypto_akcipher *tfm)
{
	struct bst_rsa_mpi_key *pkey = akcipher_tfm_ctx(tfm);

	bst_rsa_free_mpi_key(pkey);
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("BST RSA driver");
