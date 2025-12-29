/*
 * Driver for /dev/crypto device (aka CryptoDev)
 *
 * Copyright (c) 2004 Michal Ludvig <mludvig@logix.net.nz>, SuSE Labs
 * Copyright (c) 2009-2013 Nikos Mavrogiannopoulos <nmav@gnutls.org>
 *
 * This file is part of linux cryptodev.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/*
 * Device /dev/crypto provides an interface for
 * accessing kernel CryptoAPI algorithms (ciphers,
 * hashes) from userspace programs.
 *
 * /dev/crypto interface was originally introduced in
 * OpenBSD and this module attempts to keep the API.
 *
 */
#include <crypto/hash.h>
#include <linux/crypto.h>
#include <linux/mm.h>
#include <linux/highmem.h>
#include <linux/ioctl.h>
#include <linux/random.h>
#include <linux/syscalls.h>
#include <linux/pagemap.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include "cryptodev.h"
#include <crypto/scatterwalk.h>
#include <linux/scatterlist.h>
#include "cryptodev_int.h"
#include "zc.h"
#include "cryptlib.h"
#include <linux/version.h>

/* This file contains the traditional operations of encryption
 * and hashing of /dev/crypto.
 */

static int
hash_n_crypt(struct csession *ses_ptr, struct crypt_op *cop,
		struct scatterlist *src_sg, struct scatterlist *dst_sg,
		uint32_t len)
{
	int ret;

	/* Always hash before encryption and after decryption. Maybe
	 * we should introduce a flag to switch... TBD later on.
	 */
	switch (cop->op)
	{
		case COP_ENCRYPT:
			if (ses_ptr->hdata.init != 0) {
				ret = cryptodev_hash_update(&ses_ptr->hdata,
								src_sg, len);
				if (unlikely(ret))
					goto out_err;
			}
			if (ses_ptr->cdata.init != 0) {
				ret = cryptodev_cipher_encrypt(&ses_ptr->cdata,
								src_sg, dst_sg, len);

				if (unlikely(ret))
					goto out_err;
			}
			break;

		case COP_DECRYPT:
			if (ses_ptr->cdata.init != 0) {
				ret = cryptodev_cipher_decrypt(&ses_ptr->cdata,
								src_sg, dst_sg, len);

				if (unlikely(ret))
					goto out_err;
			}

			if (ses_ptr->hdata.init != 0) {
				ret = cryptodev_hash_update(&ses_ptr->hdata,
									dst_sg, len);
				if (unlikely(ret))
					goto out_err;
			}
			break;
		
		case COP_SIGN:
			if (ses_ptr->cdata.init != 0) {
				ret = cryptodev_cipher_sign(&ses_ptr->cdata,
								src_sg, dst_sg, len);
				if (unlikely(ret)) {
					derr(0, "cryptodev_cipher_sign: %d", ret);
					return ret;
				}
			}
			break;

		case COP_VERIFY:
			if (ses_ptr->cdata.init != 0) {
				ret = cryptodev_cipher_verify(&ses_ptr->cdata,
								src_sg, dst_sg, len);
				if (unlikely(ret)) {
					derr(0, "cryptodev_cipher_verify: %d", ret);
					return ret;
				}
			}
			break;

		case COP_MACGEN:
			if (ses_ptr->cdata.init != 0) {
				ret = cryptodev_cipher_macgen(&ses_ptr->cdata,
								src_sg, dst_sg, len);

				if (unlikely(ret))
					goto out_err;
			}
			break;
		
		case COP_MACVER:
			if (ses_ptr->cdata.init != 0) {
				ret = cryptodev_cipher_macver(&ses_ptr->cdata,
								src_sg, dst_sg, len);

				if (unlikely(ret))
					goto out_err;
			}
			break;

		default:
			ret = -EPROTONOSUPPORT;
			goto out_err;
	}
	
	return 0;


out_err:
	derr(0, "CryptoAPI failure: %d", ret);
	return ret;
}

/* This is the main crypto function - feed it with plaintext
   and get a ciphertext (or vice versa :-) */
static int
__crypto_run_std(struct csession *ses_ptr, struct crypt_op *cop)
{
	char *data;
	char __user *src, *dst;
	struct scatterlist sg;
	size_t nbytes, bufsize;
	int ret = 0;

	//derr(1, "__crypto_run_std");

	nbytes = cop->len;
	data = (char *)__get_free_page(GFP_KERNEL);

	if (unlikely(!data)) {
		derr(1, "Error getting free page.");
		return -ENOMEM;
	}

	bufsize = PAGE_SIZE < nbytes ? PAGE_SIZE : nbytes;

	src = cop->src;
	dst = cop->dst;

	while (nbytes > 0) {
		size_t current_len = nbytes > bufsize ? bufsize : nbytes;

		if (unlikely(copy_from_user(data, src, current_len))) {
		        derr(1, "Error copying %zu bytes from user address %p.", current_len, src);
			ret = -EFAULT;
			break;
		}

		sg_init_one(&sg, data, current_len);

		ret = hash_n_crypt(ses_ptr, cop, &sg, &sg, current_len);

		if (unlikely(ret)) {
		        derr(1, "hash_n_crypt failed.");
			break;
		}

		if (ses_ptr->cdata.init != 0) {
			if (unlikely(copy_to_user(dst, data, current_len))) {
			        derr(1, "could not copy to user.");
				ret = -EFAULT;
				break;
			}
		}

		dst += current_len;
		nbytes -= current_len;
		src += current_len;
	}

	free_page((unsigned long)data);
	return ret;
}



/* This is the main crypto function - zero-copy edition */
static int
__crypto_run_zc(struct csession *ses_ptr, struct kernel_crypt_op *kcop)
{
	struct scatterlist *src_sg, *dst_sg;
	struct crypt_op *cop = &kcop->cop;
	int ret = 0;
	//derr(1, "__crypto_run_zc");

	ret = get_userbuf(ses_ptr, cop->src, cop->len, cop->dst, 2*cop->len,
	                  kcop->task, kcop->mm, &src_sg, &dst_sg);
	if (unlikely(ret)) {
		derr(1, "Error getting user pages. Falling back to non zero copy.");
		return __crypto_run_std(ses_ptr, cop);
	}

	ret = hash_n_crypt(ses_ptr, cop, src_sg, dst_sg, cop->len);

	release_user_pages(ses_ptr);
	return ret;
}

int crypto_run(struct fcrypt *fcr, struct kernel_crypt_op *kcop)
{
	struct csession *ses_ptr;
	struct crypt_op *cop = &kcop->cop;
	int ret;

	if (unlikely(cop->op != COP_ENCRYPT && cop->op != COP_DECRYPT && cop->op != COP_SIGN && cop->op != COP_VERIFY 
		&& cop->op != COP_MACGEN && cop->op != COP_MACVER)) {
		ddebug(1, "invalid operation op=%u", cop->op);
		return -EINVAL;
	}

	/* this also enters ses_ptr->sem */
	ses_ptr = crypto_get_session_by_sid(fcr, cop->ses);
	if (unlikely(!ses_ptr)) {
		derr(1, "invalid session ID=0x%08X", cop->ses);
		return -EINVAL;
	}

	if (ses_ptr->hdata.init != 0 && (cop->flags == 0 || cop->flags & COP_FLAG_RESET)) {
		ret = cryptodev_hash_reset(&ses_ptr->hdata);
		if (unlikely(ret)) {
			derr(1, "error in cryptodev_hash_reset()");
			goto out_unlock;
		}
	}

	if (ses_ptr->cdata.init != 0 && ses_ptr->cdata.akcipher == 0) {
		int blocksize = ses_ptr->cdata.blocksize;

		if (unlikely(cop->len % blocksize)) {
			derr(1, "data size (%u) isn't a multiple of block size (%u)",
				cop->len, blocksize);
			ret = -EINVAL;
			goto out_unlock;
		}

		cryptodev_cipher_set_iv(&ses_ptr->cdata, kcop->iv,
				min(ses_ptr->cdata.ivsize, kcop->ivlen));
	}

	if (likely(cop->len)) {
		if (!(cop->flags & COP_FLAG_NO_ZC)) {
			if (unlikely(ses_ptr->alignmask && !IS_ALIGNED((unsigned long)cop->src, ses_ptr->alignmask + 1))) {
				dwarning(2, "source address %p is not %d byte aligned - disabling zero copy",
						cop->src, ses_ptr->alignmask + 1);
				cop->flags |= COP_FLAG_NO_ZC;
			}

			if (unlikely(ses_ptr->alignmask && !IS_ALIGNED((unsigned long)cop->dst, ses_ptr->alignmask + 1))) {
				dwarning(2, "destination address %p is not %d byte aligned - disabling zero copy",
						cop->dst, ses_ptr->alignmask + 1);
				cop->flags |= COP_FLAG_NO_ZC;
			}
		}

		if (cop->flags & COP_FLAG_NO_ZC)
			ret = __crypto_run_std(ses_ptr, &kcop->cop);
		else
			ret = __crypto_run_zc(ses_ptr, kcop);
		if (unlikely(ret))
			goto out_unlock;
	}

	if (ses_ptr->cdata.init != 0 && ses_ptr->cdata.akcipher == 0) {
		cryptodev_cipher_get_iv(&ses_ptr->cdata, kcop->iv,
				min(ses_ptr->cdata.ivsize, kcop->ivlen));
	}

	if (ses_ptr->hdata.init != 0 &&
		((cop->flags & COP_FLAG_FINAL) ||
		   (!(cop->flags & COP_FLAG_UPDATE) || cop->len == 0))) {

		ret = cryptodev_hash_final(&ses_ptr->hdata, kcop->hash_output);
		if (unlikely(ret)) {
			derr(0, "CryptoAPI failure: %d", ret);
			goto out_unlock;
		}
		kcop->digestsize = ses_ptr->hdata.digestsize;
	}

out_unlock:
	crypto_put_session(ses_ptr);
	return ret;
}
