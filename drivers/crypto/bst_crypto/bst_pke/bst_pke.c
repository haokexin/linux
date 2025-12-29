// SPDX-License-Identifier: GPL-2.0
/*
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
#include <linux/mpi.h>
#include <crypto/internal/rsa.h>
#include <crypto/internal/akcipher.h>
#include <crypto/akcipher.h>
#include <crypto/algapi.h>
#include "bst_pke.h"
#include "../common/bst_sa_common.h"

#define PKE_ADDR(offset) (global_pke->base + offset)

static unsigned int refcnt = 0;
static DEFINE_MUTEX(refcnt_lock);

struct pke {
	struct device *dev;
	void __iomem *base;
	int irq;
	int pke_status;
};

static uint32_t g_step;
static struct pke *global_pke;

void pke_get_version(void __iomem *io_base, uint32_t *major, uint32_t *minor)
{
	*major = (readl_relaxed(io_base + PKE_VERSION) & 0xf0) >> 4;
	*minor = readl_relaxed(io_base + PKE_VERSION) & 0x0f;
}

void uint32_clear(uint32_t *a, uint32_t word_len)
{
	uint32_t i = word_len;

	while (i) {
		writel_relaxed(0, a + (word_len - i));
		i--;
	}
}

uint32_t get_valid_bits(const uint32_t *a, uint32_t word_len)
{
	uint32_t i = 0;
	uint32_t j = 0;

	if (word_len == 0)
		return 0;

	for (i = word_len; i > 0; i--)
		if (a[i - 1])
			break;

	if (i == 0)
		return 0;

	for (j = 32; j > 0; j--)
		if (a[i - 1] & (((uint32_t)0x1) << (j - 1)))
			break;

	return ((i - 1) << 5) + j;
}

uint32_t get_valid_words(uint32_t *a, uint32_t max_words)
{
	uint32_t i;

	for (i = max_words; i > 0; i--)
		if (readl_relaxed(a + (i - 1)))
			return i;

	return 0;
}

void pke_set_operand_uint32_value(uint32_t *a, uint32_t a_word_len, uint32_t b)
{
	uint32_t i = a_word_len;

	while (i > 1) {
		i--;
		writel_relaxed(0, a + i);
	}

	writel_relaxed(b, a);
}

// void print_buf_u32(uint32_t buf[], uint32_t word_len)
// {
	// uint32_t i;

	// for (i = 0; i < word_len; i++)
	// {
		// bst_dbg(1, "%08x", buf[i]);
	// }

	// bst_dbg(1, "\r\n");
// }

uint32_t big_div2n(uint32_t a[], int32_t aWordLen, uint32_t n)
{
	int32_t i;
	uint32_t j;

	aWordLen = get_valid_words(a, aWordLen);

	if(0 == n)
	{
		return aWordLen;
	}
	else if(!aWordLen)
	{
		return 0;
	}
	else
	{;}

	//now a is not zero(aWordLen is not zero), and n is not zero either.

	if(n<32)
	{
		for(i=0; i<aWordLen-1; i++)
		{
			a[i] >>= n;
			a[i] |= (a[i+1]<<(32-n));
		}
		a[i] >>= n;

		if(!a[i])
		{
			return i;
		}
		else
		{
			return aWordLen;
		}
	}
	else
	{;}

	j=n>>5; //j=n/32;
	n&=31;  //n=n%32;

	if(j<aWordLen)
	{
		if(n)   //n is in [1, 31]
		{
			for(i=0; i<aWordLen-(int32_t)j-1; i++)
			{
				a[i] = a[i+j]>>n;
				a[i] |= (a[i+j+1]<<(32-n));
			}
			a[i] = a[i+j]>>n;
			uint32_clear(a+aWordLen-j, j);

			if(!a[i])
			{
				return i;
			}
			else
			{
				return aWordLen-j;
			}
		}
		else    //n is 0
		{
			for(i=0; i<aWordLen-(int32_t)j; i++)
			{
				a[i] = a[i+j];
			}
			uint32_clear(a+aWordLen-j, j);

			return aWordLen-j;
		}
	}
	else
	{
		uint32_clear(a, aWordLen);
		return 0;
	}
}

int32_t uint32_big_num_cmp(uint32_t *a, uint32_t a_word_len, uint32_t *b,
						   uint32_t b_word_len)
{
	int32_t i;

	a_word_len = get_valid_words(a, a_word_len);
	b_word_len = get_valid_words(b, b_word_len);
	if (a_word_len > b_word_len)
		return 1;
	else if (a_word_len < b_word_len)
		return -1;

	for (i = (a_word_len - 1); i >= 0; i--) {
		if (a[i] > b[i])
			return 1;
		else if (a[i] < b[i])
			return -1;
	}

	return 0;
}

uint32_t uint32_big_num_check_zero(uint32_t *a, uint32_t a_word_len)
{
	uint32_t i;

	for (i = 0; i < a_word_len; i++)
		if (a[i])
			return 0;

	return 1;
}

void reverse_byte_array(uint8_t *in, uint8_t *out, uint32_t byte_len)
{
	uint32_t idx, round = byte_len >> 1;
	uint8_t tmp;

	for (idx = 0; idx < round; idx++) {
		tmp = in[idx];
		out[idx] = in[byte_len - 1 - idx];
		out[byte_len - 1 - idx] = tmp;
	}

	if ((byte_len & 0x1) && (in != out))
		out[round] = in[round];
}

void pke_set_operand_width(uint32_t bit_len)
{
	uint32_t mask = ~(0x07FFFF);
	uint32_t cfg = 0, len;

	len = (bit_len + 255) / 256;
	if (len == 1) {
		cfg = 2;
		g_step = 0x20;
	} else if (len == 2) {
		cfg = 3;
		g_step = 0x40;
	} else if (len <= 4) {
		cfg = 4;
		g_step = 0x80;
	} else if (len <= 8) {
		cfg = 5;
		g_step = 0x100;
	} else if (len <= 16) {
		cfg = 6;
		g_step = 0x200;
	}

	cfg = (cfg << 16) | (len << 8);
	writel_relaxed(readl_relaxed(PKE_ADDR(PKE_CFG)) & mask, PKE_ADDR(PKE_CFG));
	writel_relaxed(readl_relaxed(PKE_ADDR(PKE_CFG)) | cfg, PKE_ADDR(PKE_CFG));
}

void pke_load_operand(uint32_t *base_addr, uint32_t *data, uint32_t word_len)
{
	uint32_t i;

	if (base_addr != data)
		for (i = 0; i < word_len; i++)
			writel_relaxed(data[i], base_addr + i);
}

void pke_set_microcode(uint32_t addr)
{
	writel_relaxed(addr, PKE_ADDR(PKE_MC_PTR));
}

void pke_clear_interrupt(void)
{
	uint32_t mask = ~((uint32_t)1);

	writel_relaxed(readl_relaxed(PKE_ADDR(PKE_RISR)) & mask,
				   PKE_ADDR(PKE_RISR));
}

void pke_enable_interrupt(void)
{
	uint32_t flag = (uint32_t)1;

	writel_relaxed(readl_relaxed(PKE_ADDR(PKE_IMCR)) | flag,
				   PKE_ADDR(PKE_IMCR));
}

void pke_disable_interrupt(void)
{
	uint32_t mask = ~((uint32_t)1);

	writel_relaxed(readl_relaxed(PKE_ADDR(PKE_IMCR)) & mask,
				   PKE_ADDR(PKE_IMCR));
}

void pke_start(void)
{
	uint32_t flag = PKE_START_CALC;

	writel_relaxed(readl_relaxed(PKE_ADDR(PKE_CTRL)) | flag,
				   PKE_ADDR(PKE_CTRL));
}

void pke_wait_till_done(void)
{
	uint32_t flag = 1;

	while (!(readl_relaxed(PKE_ADDR(PKE_RISR)) & flag))
		;
}

uint32_t pke_check_rt_code(void)
{
	uint32_t mask = 0x07u;

	return (uint8_t)(readl_relaxed(PKE_ADDR(PKE_RT_CODE)) & mask);
}

void pke_read_operand(uint32_t *base_addr, uint32_t *data, uint32_t word_len)
{
	uint32_t i;

	if (base_addr != data)
		for (i = 0; i < word_len; i++)
			data[i] = readl_relaxed(base_addr + i);
}

uint32_t pke_set_micro_code_start_wait_return_code(uint32_t micro_code)
{
	pke_set_microcode(micro_code);
	pke_clear_interrupt();
	pke_start();
	pke_wait_till_done();

	return pke_check_rt_code();
}

uint32_t pke_pre_calc_mont_n0(void)
{
	return pke_set_micro_code_start_wait_return_code(MICROCODE_MGMR_PRE_N0);
}

uint32_t pke_pre_calc_mont(const uint32_t *modulus, uint32_t bit_len,
						   uint32_t *H)
{
	uint32_t word_len = GET_WORD_LEN(bit_len);
	uint32_t ret;

	pke_set_operand_width(bit_len);
	pke_load_operand((PKE_A(0, g_step)), (uint32_t *)modulus, word_len);
	if ((g_step / 4) > word_len) {
		uint32_clear((PKE_A(0, g_step)) + word_len, (g_step / 4) - word_len);
		uint32_clear((PKE_B(0, g_step)) + word_len, (g_step / 4) - word_len);
	}

	ret = pke_pre_calc_mont_n0();
	if (ret != PKE_SUCCESS)
		return ret;

	if (256 == bit_len || 512 == bit_len || 1024 == bit_len || 2048 == bit_len || 4096 == bit_len) {
		ret = pke_set_micro_code_start_wait_return_code(
			MICROCODE_MGMR_PRE_H_MM);
	} else
		ret = pke_set_micro_code_start_wait_return_code(MICROCODE_MGMR_PRE_H);

	if (ret != PKE_SUCCESS)
		return ret;
	else if (NULL != H)
		pke_read_operand((PKE_B(0, g_step)), H, word_len);

	return PKE_SUCCESS;
}

void uint32_copy(uint32_t *dst, uint32_t *src, uint32_t word_len)
{
	uint32_t i;

	if (dst != src)
		for (i = 0; i < word_len; i++)
			dst[i] = src[i];
}

void uint32_big_num_add_one(uint32_t *a, uint32_t word_len)
{
	uint32_t i, carry;

	carry = 1;
	for (i = 0; i < word_len; i++) {
		a[i] += carry;
		if (a[i] < carry)
			carry = 1;
		else
			break;
	}
}

uint32_t big_div_2n(uint32_t a[], int32_t a_word_len, uint32_t n)
{
	int32_t i;
	uint32_t j;

	a_word_len = get_valid_words(a, a_word_len);
	if (n == 0)
		return a_word_len;
	else if (!a_word_len)
		return 0;

	if (n < 32) {
		for (i = 0; i < a_word_len - 1; i++) {
			a[i] >>= n;
			a[i] |= (a[i + 1] << (32 - n));
		}

		a[i] >>= n;
		if (!a[i])
			return i;
		else
			return a_word_len;
	}

	j = n >> 5;
	n &= 31;
	if (j < a_word_len) {
		if (n) {
			for (i = 0; i < a_word_len - (int32_t)j - 1; i++) {
				a[i] = a[i + j] >> n;
				a[i] |= (a[i + j + 1] << (32 - n));
			}

			a[i] = a[i + j] >> n;
			uint32_clear(a + a_word_len - j, j);
			if (!a[i])
				return i;
			else
				return a_word_len - j;
		} else {
			for (i = 0; i < a_word_len - (int32_t)j; i++)
				a[i] = a[i + j];

			uint32_clear(a + a_word_len - j, j);

			return a_word_len - j;
		}
	} else {
		uint32_clear(a, a_word_len);

		return 0;
	}
}

uint32_t pke_mod_exp(const uint32_t *modulus, const uint32_t *exponent,
					 const uint32_t *base, uint32_t *out, uint32_t mod_word_len,
					 uint32_t exp_word_len)
{
	uint32_t ret;

	pke_set_operand_width(mod_word_len << 5);
	pke_load_operand((PKE_A(2, g_step)), (uint32_t *)exponent, exp_word_len);
	if ((g_step / 4) > exp_word_len)
		uint32_clear((PKE_A(2, g_step)) + exp_word_len,
					 (g_step / 4) - exp_word_len);

	pke_load_operand((PKE_A(0, g_step)), (uint32_t *)modulus, mod_word_len);
	pke_load_operand((PKE_B(1, g_step)), (uint32_t *)base, mod_word_len);
	if ((g_step / 4) > mod_word_len) {
		uint32_clear((PKE_A(0, g_step)) + mod_word_len,
					 (g_step / 4) - mod_word_len);
		uint32_clear((PKE_B(1, g_step)) + mod_word_len,
					 (g_step / 4) - mod_word_len);
	}

	ret = pke_set_micro_code_start_wait_return_code(MICROCODE_MODEXP);
	if (ret == PKE_SUCCESS)
		pke_read_operand((PKE_A(1, g_step)), out, mod_word_len);

	return ret;
}

uint32_t pke_modadd_modsub_internal(const uint32_t *modulus, const uint32_t *a,
									const uint32_t *b, uint32_t *out, uint32_t word_len, uint32_t micro_code)
{
	uint32_t ret;

	pke_set_operand_width(word_len << 5);
	pke_load_operand((PKE_A(0, g_step)), (uint32_t *)modulus, word_len);
	pke_load_operand((PKE_A(1, g_step)), (uint32_t *)a, word_len);
	pke_load_operand((PKE_B(1, g_step)), (uint32_t *)b, word_len);
	if ((g_step / 4) > word_len) {
		uint32_clear((PKE_A(0, g_step)) + word_len, (g_step / 4) - word_len);
		uint32_clear((PKE_A(1, g_step)) + word_len, (g_step / 4) - word_len);
		uint32_clear((PKE_B(1, g_step)) + word_len, (g_step / 4) - word_len);
	}

	ret = pke_set_micro_code_start_wait_return_code(micro_code);
	if (ret == PKE_SUCCESS)
		pke_read_operand((PKE_A(1, g_step)), out, word_len);

	return ret;
}

uint32_t pke_load_modulus_and_pre_monts(uint32_t *modulus, uint32_t *modulus_h,
										uint32_t bit_len)
{
	uint32_t word_len = GET_WORD_LEN(bit_len);

	pke_set_operand_width(bit_len);
	pke_load_operand((PKE_A(0, g_step)), modulus, word_len);
	pke_load_operand((PKE_B(0, g_step)), modulus_h, word_len);
	if ((g_step / 4) > word_len) {
		uint32_clear((PKE_A(0, g_step)) + word_len, (g_step / 4) - word_len);
		uint32_clear((PKE_B(0, g_step)) + word_len, (g_step / 4) - word_len);
	}

	return pke_pre_calc_mont_n0();
}

uint32_t pke_set_modulus_and_pre_monts(uint32_t *modulus, uint32_t *modulus_h,
									   uint32_t bit_len)
{
	if (modulus_h == NULL)
		return pke_pre_calc_mont(modulus, bit_len, NULL);
	else
		return pke_load_modulus_and_pre_monts(modulus, modulus_h, bit_len);
}

uint32_t eccp_point_mul_shamir(const struct pke_ec_curve *curve,
							   uint32_t *k1, uint32_t *P1x, uint32_t *P1y,
							   uint32_t *k2, uint32_t *P2x, uint32_t *P2y,
							   uint32_t *Qx, uint32_t *Qy)
{
	uint32_t word_len = GET_WORD_LEN(curve->n_bit_len);
	uint32_t ret;

	ret = pke_set_modulus_and_pre_monts(curve->eccp_p,
										curve->eccp_p_h, curve->p_bit_len);
	if (ret != PKE_SUCCESS)
		return ret;

	pke_load_operand((PKE_B(1, g_step)), P1x, word_len);
	pke_load_operand((PKE_B(2, g_step)), P1y, word_len);
	pke_set_operand_uint32_value((PKE_A(3, g_step)), g_step / 4, 1);

	pke_load_operand((PKE_B(5, g_step)), P2x, word_len);
	pke_load_operand((PKE_B(6, g_step)), P2y, word_len);

	pke_load_operand((PKE_B(4, g_step)), curve->eccp_a, word_len);
	pke_load_operand((PKE_A(4, g_step)), k1, word_len);
	pke_load_operand((PKE_A(5, g_step)), k2, word_len);

	if ((g_step / 4) > word_len) {
		uint32_clear((PKE_B(1, g_step)) + word_len, (g_step / 4) - word_len);
		uint32_clear((PKE_B(2, g_step)) + word_len, (g_step / 4) - word_len);
		uint32_clear((PKE_B(5, g_step)) + word_len, (g_step / 4) - word_len);
		uint32_clear((PKE_B(6, g_step)) + word_len, (g_step / 4) - word_len);
		uint32_clear((PKE_B(4, g_step)) + word_len, (g_step / 4) - word_len);
		uint32_clear((PKE_A(4, g_step)) + word_len, (g_step / 4) - word_len);
		uint32_clear((PKE_A(5, g_step)) + word_len, (g_step / 4) - word_len);
	}

	ret = pke_set_micro_code_start_wait_return_code(MICROCODE_PMULF);
	if (ret != PKE_SUCCESS)
		return ret;

	pke_read_operand((PKE_A(1, g_step)), Qx, word_len);
	if (Qy != NULL)
		pke_read_operand((PKE_A(2, g_step)), Qy, word_len);

	return PKE_SUCCESS;
}

uint32_t pke_mod_add(const uint32_t *modulus, const uint32_t *a,
					 const uint32_t *b, uint32_t *out, uint32_t word_len)
{
	return pke_modadd_modsub_internal(modulus, a, b, out, word_len,
									  MICROCODE_MODADD);
}

uint32_t pke_mod_inv(const uint32_t *modulus, const uint32_t *a,
					 uint32_t *a_inv, uint32_t mod_word_len, uint32_t a_word_len)
{
	uint32_t ret;

	pke_set_operand_width(mod_word_len << 5);
	pke_load_operand((PKE_A(0, g_step)), (uint32_t *)modulus, mod_word_len);
	if ((g_step / 4) > mod_word_len)
		uint32_clear((PKE_A(0, g_step)) + mod_word_len,
					 (g_step / 4 - mod_word_len));

	pke_load_operand((PKE_B(1, g_step)), (uint32_t *)a, a_word_len);
	if ((g_step / 4) > a_word_len)
		uint32_clear((PKE_B(1, g_step)) + a_word_len, (g_step / 4) - a_word_len);

	ret = pke_set_micro_code_start_wait_return_code(MICROCODE_MODINV);
	if (ret == PKE_SUCCESS)
		pke_read_operand((PKE_A(1, g_step)), a_inv, mod_word_len);

	return ret;
}

uint32_t uint32_integer_check(uint32_t *k, uint32_t *n, uint32_t word_len,
							  uint32_t ret_zero, uint32_t ret_big, uint32_t ret_success)
{
	if (uint32_big_num_check_zero(k, word_len))
		return ret_zero;
	else if (uint32_big_num_cmp(k, word_len, n, word_len) >= 0)
		return ret_big;

	return ret_success;
}
uint32_t pke_add_sub_internal(const uint32_t *a, const uint32_t *b,
							  uint32_t *out, uint32_t word_len, uint32_t micro_code)
{
	uint32_t ret;

	pke_set_operand_width(word_len << 5);
	pke_load_operand((uint32_t *)(PKE_A(1, g_step)), (uint32_t *)a, word_len);
	pke_load_operand((uint32_t *)(PKE_B(1, g_step)), (uint32_t *)b, word_len);
	if ((g_step / 4) > word_len) {
		uint32_clear((uint32_t *)(PKE_A(1, g_step)) + word_len,
					 (g_step / 4) - word_len);
		uint32_clear((uint32_t *)(PKE_B(1, g_step)) + word_len,
					 (g_step / 4) - word_len);
	}

	ret = pke_set_micro_code_start_wait_return_code(micro_code);
	if (ret == PKE_SUCCESS) {
		pke_read_operand((uint32_t *)(PKE_A(1, g_step)), out, word_len);
		return PKE_SUCCESS;
	} else
		return ret;

}

uint32_t pke_sub(const uint32_t *a, const uint32_t *b, uint32_t *out,
				 uint32_t word_len)
{
	return pke_add_sub_internal(a, b, out, word_len, MICROCODE_INTSUB);
}

uint32_t pke_mod_mul_internal(const uint32_t *a, const uint32_t *b,
							  uint32_t *out, uint32_t word_len)
{
	uint32_t ret;

	pke_set_operand_width(word_len << 5);
	pke_load_operand((uint32_t *)(PKE_A(1, g_step)), (uint32_t *)a, word_len);
	pke_load_operand((uint32_t *)(PKE_B(1, g_step)), (uint32_t *)b, word_len);
	if ((g_step / 4) > word_len) {
		uint32_clear((uint32_t *)(PKE_A(1, g_step)) + word_len,
					 (g_step / 4) - word_len);
		uint32_clear((uint32_t *)(PKE_B(1, g_step)) + word_len,
					 (g_step / 4) - word_len);
	}

	ret = pke_set_micro_code_start_wait_return_code(MICROCODE_MODMUL);
	if (ret == PKE_SUCCESS)
		pke_read_operand((uint32_t *)(PKE_A(1, g_step)), out, word_len);

	return ret;
}

uint32_t pke_mod_sub(const uint32_t *modulus, const uint32_t *a,
					 const uint32_t *b, uint32_t *out, uint32_t word_len)
{
	return pke_modadd_modsub_internal(modulus, a, b, out, word_len,
									  MICROCODE_MODSUB);
}

uint32_t pke_mod(uint32_t *a, uint32_t a_word_len, uint32_t *b, uint32_t *b_h,
				 uint32_t b_word_len, uint32_t *c)
{
	int32_t flag;
	uint32_t bit_len, tmp_len;
	uint32_t *t1, *t2;
	uint32_t ret;

	flag = uint32_big_num_cmp(a, a_word_len, b, b_word_len);
	if (flag < 0) {
		a_word_len = get_valid_words(a, a_word_len);
		uint32_copy(c, a, a_word_len);
		uint32_clear(c + a_word_len, b_word_len - a_word_len);

		return PKE_SUCCESS;
	} else if (flag == 0) {
		uint32_clear(c, b_word_len);

		return PKE_SUCCESS;
	}

	bit_len = get_valid_bits(b, b_word_len);
	pke_set_operand_width(bit_len);

	t1 = (uint32_t *)(PKE_A(1, g_step));
	t2 = (uint32_t *)(PKE_B(2, g_step));

	bit_len &= 0x1F;

	// get t2 = a high part mod b
	if (bit_len) {
		tmp_len = a_word_len - b_word_len + 1;
		uint32_copy(t2, a + b_word_len - 1, tmp_len);
		big_div_2n(t2, tmp_len, bit_len);
		if (tmp_len < b_word_len)
			uint32_clear(t2 + tmp_len, b_word_len - tmp_len);
		else if (uint32_big_num_cmp(t2, b_word_len, b, b_word_len) >= 0) {
			ret = pke_sub(t2, b, t2, b_word_len);
			if (ret != PKE_SUCCESS)
				return ret;
		}
	} else {
		tmp_len = a_word_len - b_word_len;
		if (uint32_big_num_cmp(a + b_word_len, tmp_len, b, b_word_len) >= 0) {
			ret = pke_sub(a + b_word_len, b, t2, b_word_len);
			if (ret != PKE_SUCCESS)
				return ret;
		} else {
			uint32_copy(t2, a + b_word_len, tmp_len);
			uint32_clear(t2 + tmp_len, b_word_len - tmp_len);
		}
	}

	// set the pre-calculated mont parameters
	ret = pke_set_modulus_and_pre_monts(b, b_h, get_valid_bits(b, b_word_len));
	if (ret != PKE_SUCCESS)
		return ret;
	// get t1 = 1000...000 mod b
	uint32_clear(t1, b_word_len);
	if (bit_len)
		t1[b_word_len - 1] = 1 << (bit_len);

	ret = pke_sub(t1, b, t1, b_word_len);
	if (ret != PKE_SUCCESS)
		return ret;

	// get t2 = a_high * 1000..000 mod b
	ret = pke_mod_mul_internal(t1, t2, t2, b_word_len);
	if (ret != PKE_SUCCESS)
		return ret;

	// get t1 = a low part mod b
	if (bit_len) {
		uint32_copy(t1, a, b_word_len);
		t1[b_word_len - 1] &= ((1 << (bit_len)) - 1);
		if (uint32_big_num_cmp(t1, b_word_len, b, b_word_len) >= 0) {
			ret = pke_sub(t1, b, t1, b_word_len);
			if (ret != PKE_SUCCESS)
				return ret;
		}
	} else {
		if (uint32_big_num_cmp(a, b_word_len, b, b_word_len) >= 0) {
			ret = pke_sub(a, b, t1, b_word_len);
			if (ret != PKE_SUCCESS)
				return ret;
		} else
			t1 = a;
	}

	return pke_mod_add(b, t1, t2, c, b_word_len);
}

uint32_t eccp_point_add(const struct pke_ec_curve *curve, uint32_t *P1x,
						uint32_t *P1y, uint32_t *P2x, uint32_t *P2y, uint32_t *Qx, uint32_t *Qy)
{
	uint32_t word_len = GET_WORD_LEN(curve->n_bit_len);
	uint32_t ret;

	ret = pke_set_modulus_and_pre_monts(curve->eccp_p, curve->eccp_p_h,
										curve->n_bit_len);
	if (ret != PKE_SUCCESS)
		return ret;

	pke_load_operand((PKE_A(1, g_step)), P1x, word_len);
	pke_load_operand((PKE_A(2, g_step)), P1y, word_len);
	pke_set_operand_uint32_value((PKE_B(3, g_step)), g_step / 4, 1);

	pke_load_operand((PKE_B(1, g_step)), P2x, word_len);
	pke_load_operand((PKE_B(2, g_step)), P2y, word_len);
	pke_set_operand_uint32_value((PKE_A(3, g_step)), g_step / 4, 1);

	if ((g_step / 4) > word_len) {
		uint32_clear((PKE_A(1, g_step)) + word_len, (g_step / 4) - word_len);
		uint32_clear((PKE_A(2, g_step)) + word_len, (g_step / 4) - word_len);
		uint32_clear((PKE_B(1, g_step)) + word_len, (g_step / 4) - word_len);
		uint32_clear((PKE_B(2, g_step)) + word_len, (g_step / 4) - word_len);
	}

	ret = pke_set_micro_code_start_wait_return_code(MICROCODE_PADD);
	if (ret != PKE_SUCCESS)
		return ret;

	pke_read_operand((PKE_A(1, g_step)), Qx, word_len);
	if (Qy != NULL)
		pke_read_operand((PKE_A(2, g_step)), Qy, word_len);

	return PKE_SUCCESS;
}

uint32_t eccp_point_mul(const struct pke_ec_curve *curve, uint32_t *k, uint32_t *Px,
						uint32_t *Py, uint32_t *Qx, uint32_t *Qy)
{
	uint32_t word_len = GET_WORD_LEN(curve->n_bit_len);
	uint32_t ret;

	ret = pke_set_modulus_and_pre_monts(curve->eccp_p, curve->eccp_p_h,
										curve->n_bit_len);
	if (ret != PKE_SUCCESS)
		return ret;

	pke_load_operand((PKE_B(1, g_step)), Px, word_len);
	pke_load_operand((PKE_B(2, g_step)), Py, word_len);
	pke_set_operand_uint32_value((PKE_A(3, g_step)), g_step / 4, 1);
	pke_load_operand((PKE_B(4, g_step)), curve->eccp_a, word_len);
	pke_load_operand((PKE_A(4, g_step)), k, word_len);

	if ((g_step / 4) > word_len) {
		uint32_clear((PKE_B(1, g_step)) + word_len, (g_step / 4) - word_len);
		uint32_clear((PKE_B(2, g_step)) + word_len, (g_step / 4) - word_len);
		uint32_clear((PKE_B(4, g_step)) + word_len, (g_step / 4) - word_len);
		uint32_clear((PKE_A(4, g_step)) + word_len, (g_step / 4) - word_len);
	}

	ret = pke_set_micro_code_start_wait_return_code(MICROCODE_PMUL);
	if (ret != PKE_SUCCESS)
		return ret;

	pke_read_operand((PKE_A(1, g_step)), Qx, word_len);
	if (Qy != NULL)
		pke_read_operand((PKE_A(2, g_step)), Qy, word_len);

	return PKE_SUCCESS;
}

uint32_t eccp_point_verify(const struct pke_ec_curve *curve, uint32_t *Px,
						   uint32_t *Py)
{
	uint32_t word_len = GET_WORD_LEN(curve->p_bit_len);
	uint32_t ret;

	ret = pke_set_modulus_and_pre_monts(curve->eccp_p, curve->eccp_p_h,
										curve->p_bit_len);
	if (ret != PKE_SUCCESS)
		return ret;

	pke_load_operand((PKE_A(1, g_step)), Px, word_len);
	pke_load_operand((PKE_A(2, g_step)), Py, word_len);
	pke_load_operand((PKE_B(4, g_step)), curve->eccp_a, word_len);
	pke_load_operand((PKE_A(4, g_step)), curve->eccp_b, word_len);

	if ((g_step / 4) > word_len) {
		uint32_clear((PKE_A(1, g_step)) + word_len, (g_step / 4) - word_len);
		uint32_clear((PKE_A(2, g_step)) + word_len, (g_step / 4) - word_len);
		uint32_clear((PKE_B(4, g_step)) + word_len, (g_step / 4) - word_len);
		uint32_clear((PKE_A(4, g_step)) + word_len, (g_step / 4) - word_len);
	}

	ret = pke_set_micro_code_start_wait_return_code(MICROCODE_PVER);
	if (ret != PKE_SUCCESS)
		return ret;
	else
		return PKE_SUCCESS;
}

uint32_t eccp_point_mul_shamir_safe(const struct pke_ec_curve *curve, uint32_t *k1,
									uint32_t *P1x, uint32_t *P1y, uint32_t *k2, uint32_t *P2x,
									uint32_t *P2y, uint32_t *Qx, uint32_t *Qy)
{
	uint32_t x[ECCP_MAX_WORD_LEN], y[ECCP_MAX_WORD_LEN];
	uint32_t ret = PKE_NO_MODINV;

	ret = eccp_point_mul_shamir(curve, k1, P1x, P1y, k2, P2x, P2y, Qx, Qy);
	if (ret == PKE_NO_MODINV) {
		ret = eccp_point_mul(curve, k1, P1x, P1y, x, y);
		if (ret != PKE_SUCCESS)
			return ret;

		ret = eccp_point_mul(curve, k2, P2x, P2y, (PKE_A(1, g_step)),
							 (PKE_A(2, g_step)));
		if (ret != PKE_SUCCESS)
			return ret;

		ret = eccp_point_add(curve, (PKE_A(1, g_step)), (PKE_A(2, g_step)),
							 x, y, Qx, Qy);
		if (ret != PKE_SUCCESS)
			return ret;
	}

	return ret;
}

uint32_t eccp_point_mul_base(const struct pke_ec_curve *curve, uint32_t *k,
							 uint32_t *Qx, uint32_t *Qy)
{
	uint32_t n_word_len = GET_WORD_LEN(curve->n_bit_len);
	uint32_t *k1, *k2;
	uint32_t tmp_bit_len, tmp_word_len;
	uint32_t ret;

	pke_set_operand_width(curve->p_bit_len);
	k1 = (uint32_t *)(PKE_A(4, g_step));
	k2 = (uint32_t *)(PKE_A(5, g_step));

	// k2: low half part
	tmp_bit_len = (curve->n_bit_len) / 2;
	tmp_word_len = GET_WORD_LEN(tmp_bit_len);
	uint32_copy(k2, k, tmp_word_len);
	uint32_clear(k2 + tmp_word_len, n_word_len - tmp_word_len);
	tmp_bit_len = tmp_bit_len & 0x1F;
	if (tmp_bit_len)
		k2[tmp_word_len - 1] &= (1 << tmp_bit_len) - 1;
	// k1: high half part
	if (tmp_bit_len) {
		uint32_copy(k1, k + tmp_word_len - 1, n_word_len - tmp_word_len + 1);
		uint32_clear(k1 + n_word_len - tmp_word_len + 1, tmp_word_len - 1);
		big_div_2n(k1, n_word_len - tmp_word_len + 1, tmp_bit_len);
	} else {
		uint32_copy(k1, k + tmp_word_len, n_word_len - tmp_word_len);
		uint32_clear(k1 + n_word_len - tmp_word_len, tmp_word_len);
	}
	tmp_bit_len = curve->n_bit_len - (curve->n_bit_len) / 2;
	tmp_word_len = GET_WORD_LEN(tmp_bit_len);
	tmp_bit_len = tmp_bit_len & 0x1F;
	if (tmp_bit_len)
		k1[tmp_word_len - 1] &= (1 << tmp_bit_len) - 1;

	ret = eccp_point_mul_shamir(curve,
								k1, curve->eccp_half_Gx, curve->eccp_half_Gy,
								k2, curve->eccp_Gx, curve->eccp_Gy,
								Qx, Qy);
	if (ret == PKE_NO_MODINV)
		ret = eccp_point_mul(curve, k, curve->eccp_Gx, curve->eccp_Gy, Qx, Qy);

	return ret;
}

//(MPI res, MPI base, MPI exp, MPI mod)
uint32_t bst_rsa_mod_exp(MPI res, MPI base, MPI exp, MPI mod)
{
	int32_t flag;
	uint32_t ret;
	uint32_t ret_rsa_success = RSA_SUCCESS;
	uint32_t e_bit_len = exp->nbits;
	uint32_t n_bit_len = mod->nbits;
	uint32_t base_bit_len = base->nbits;
	uint32_t e_word_len = GET_WORD_LEN(e_bit_len);
	uint32_t n_word_len = GET_WORD_LEN(n_bit_len);
	uint32_t base_word_len = GET_WORD_LEN(base_bit_len);

	MPI a = mpi_alloc(GET_BYTE_LEN(n_bit_len));
	MPI e = mpi_alloc(GET_BYTE_LEN(n_bit_len));
	MPI n = mpi_alloc(GET_BYTE_LEN(n_bit_len));

	if (global_pke->pke_status)
		global_pke->pke_status = PKE_NOT_AVAILABLE;
	else {
		ret = PKE_ERROR;
		goto free_room;
	}
	if (NULL == a || NULL == e || NULL == n || NULL == res) {
		ret = RSA_BUFFER_NULL;
		goto free_room;
	}
	else if ((n_bit_len > MAX_RSA_BIT_LEN) || (e_bit_len > n_bit_len)) {
		ret = RSA_INPUT_TOO_LONG;
		goto free_room;
	}
	else if ((n_bit_len == 0) || (!(mod->d[0] & 1))) {
		ret = RSA_INPUT_INVALID;
		goto free_room;
	}

	memset(a->d, 0, GET_BYTE_LEN(n_bit_len));
	memset(e->d, 0, GET_BYTE_LEN(n_bit_len));
	memset(n->d, 0, GET_BYTE_LEN(n_bit_len));
	memcpy(a->d, base->d, GET_BYTE_LEN(base_bit_len));
	memcpy(e->d, exp->d, GET_BYTE_LEN(e_bit_len));
	memcpy(n->d, mod->d, GET_BYTE_LEN(n_bit_len));

	mpi_resize(res, n_word_len);

	// a should be in [0,n]
	flag = uint32_big_num_cmp((uint32_t *)a->d, n_word_len, (uint32_t *)n->d, n_word_len);
	if (flag > 0) {
		ret = RSA_INPUT_INVALID;
		goto free_room;
	}

	// if a is 0 or n
	if ((flag == 0) || (uint32_big_num_check_zero((uint32_t *)a->d, base_word_len) == 1)) {
		if (uint32_big_num_check_zero((uint32_t *)e->d, e_word_len)) {
			ret = RSA_INPUT_INVALID;
			goto free_room;
		} else {
			uint32_clear((uint32_t *)res->d, n_word_len);
			ret = ret_rsa_success;
			goto free_room;
		}
	} else if (uint32_big_num_check_zero((uint32_t *)e->d, e_word_len)) {
		pke_set_operand_uint32_value((uint32_t *)res->d, n_word_len, 1);
		ret = ret_rsa_success;
		goto free_room;
	}

	ret = pke_pre_calc_mont((uint32_t *)n->d, n_bit_len, NULL);
	if (ret != PKE_SUCCESS)
		goto free_room;

	res->nlimbs = GET_BYTE_LEN(n_bit_len) / 8;
	ret = pke_mod_exp((uint32_t *)n->d, (uint32_t *)e->d, (uint32_t *)a->d,
					  (uint32_t *)res->d, n_word_len, e_word_len);
free_room:
	if (!global_pke->pke_status)
		global_pke->pke_status = PKE_IS_AVAILABLE;
	MFreeMem(n);
	MFreeMem(e);
	MFreeMem(a);
	return ret;
}

void sm2_print(uint8_t *array, uint8_t len, uint32_t line)
{
	int i;
	if (array == NULL) {
		bst_dbg(1, "sm2_print array NULL: %d", line);
		return;
	}
	bst_dbg(1, "sm2_print: %d", line);
	for (i = 0; i < len; i++) {
		bst_dbg(1, "0x%02x", array[i]);
	}
}

uint32_t sm2_sign_with_k(const struct pke_ec_curve *sm2_curve, uint32_t e[8],
						 uint32_t k[8], uint32_t dA[8], uint32_t r[8], uint32_t s[8])
{
	uint32_t tmp1[SM2_WORD_LEN], tmp2[SM2_WORD_LEN];
	uint32_t ret;
	uint32_t ret_sm2_success = SM2_SUCCESS;

	if (e == NULL || k == NULL || dA == NULL || r == NULL || s == NULL)
		return SM2_BUFFER_NULL;

	// make sure k in [1, n-1]
	ret = uint32_integer_check(k, sm2_curve->eccp_n, SM2_WORD_LEN,
							   SM2_ZERO_ALL, SM2_INTEGER_TOO_BIG, ret_sm2_success);
	if (ret != ret_sm2_success)
		return ret;

#ifdef SM2_HIGH_SPEED
	ret = eccp_point_mul_base(sm2_curve, k, tmp1, NULL);
#else
	ret = eccp_point_mul(sm2_curve, k, sm2_curve->eccp_Gx, sm2_curve->eccp_Gy,
						 tmp1, NULL);
#endif
	if (ret != PKE_SUCCESS)
		return ret;

	// tmp1 = x1 mod n
	if (uint32_big_num_cmp(tmp1, SM2_WORD_LEN, sm2_curve->eccp_n,
						   SM2_WORD_LEN) >= 0) {
		ret = pke_sub(tmp1, sm2_curve->eccp_n, tmp1, SM2_WORD_LEN);
		if (ret != PKE_SUCCESS)
			return ret;
	}

	// r = e + x1 mod n
	ret = pke_mod_add((uint32_t *)sm2_curve->eccp_n, e, tmp1, r, SM2_WORD_LEN);
	if (ret != PKE_SUCCESS)
		return ret;

	// make sure r is not zero
	if (uint32_big_num_check_zero(r, SM2_WORD_LEN))
		return SM2_ZERO_ALL;

	// tmp1 = r + k mod n
	ret = pke_mod_add((uint32_t *)sm2_curve->eccp_n, r, k, tmp1, SM2_WORD_LEN);
	if (ret != PKE_SUCCESS)
		return ret;
	else if (uint32_big_num_check_zero(tmp1, SM2_WORD_LEN))
		return SM2_ZERO_ALL;

	ret = pke_load_modulus_and_pre_monts((uint32_t *)sm2_curve->eccp_n,
										 (uint32_t *)sm2_curve->eccp_n_h, SM2_BIT_LEN);
	if (ret != PKE_SUCCESS)
		return ret;

	// tmp1 =  r*dA mod n
	ret = pke_mod_mul_internal(r, dA, tmp1, SM2_WORD_LEN);
	if (ret != PKE_SUCCESS)
		return ret;

	// tmp1 =  (k - r*dA) mod n
	ret = pke_mod_sub((uint32_t *)sm2_curve->eccp_n, k, tmp1, tmp1,
					  SM2_WORD_LEN);
	if (ret != PKE_SUCCESS)
		return ret;

	// tmp2 = (1+dA)^(-1) mod n
	uint32_copy(tmp2, dA, SM2_WORD_LEN);
	uint32_big_num_add_one(tmp2, SM2_WORD_LEN);
	ret = pke_mod_inv(sm2_curve->eccp_n, tmp2, tmp2, SM2_WORD_LEN,
					  SM2_WORD_LEN);
	if (ret != PKE_SUCCESS)
		return ret;

	// s = ((1+dA)^(-1))*(k - r*dA) mod n
	ret = pke_mod_mul_internal(tmp1, tmp2, s, SM2_WORD_LEN);
	if (ret != PKE_SUCCESS)
		return ret;

	// make sure s is not zero
	if (uint32_big_num_check_zero(s, SM2_WORD_LEN))
		return SM2_ZERO_ALL;
	else
		return SM2_SUCCESS;
}

uint32_t pke_sm2_verify(const struct pke_ec_curve *sm2_curve, uint8_t E[32],
						uint8_t pub_key[65], uint8_t signature[64])
{
	uint32_t e[SM2_WORD_LEN], r[SM2_WORD_LEN], s[SM2_WORD_LEN], tmp[SM2_WORD_LEN * 4];
	uint32_t *t = e;
	uint32_t ret = 0;
	uint32_t ret_sm2_success = SM2_SUCCESS;

	if (pub_key == NULL || signature == NULL)//E == NULL ||
		return SM2_BUFFER_NULL;
	else if (pub_key[0] != POINT_NOT_COMPRESSED)
		return SM2_INPUT_INVALID;
	// get PA and check PA
	reverse_byte_array(pub_key + 1, (uint8_t *)(tmp + 2 * SM2_WORD_LEN),
					   SM2_BYTE_LEN);
	reverse_byte_array(pub_key + 1 + SM2_BYTE_LEN,
					   (uint8_t *)(tmp + 3 * SM2_WORD_LEN), SM2_BYTE_LEN);
	ret = eccp_point_verify(sm2_curve, (uint32_t *)(tmp + 2 * SM2_WORD_LEN),
							(uint32_t *)(tmp + 3 * SM2_WORD_LEN));
	if (ret != PKE_SUCCESS)
		return SM2_NOT_ON_CURVE;

	// make sure r in [1, n-1]
	reverse_byte_array(signature, (uint8_t *)r, SM2_BYTE_LEN);
	ret = uint32_integer_check(r, sm2_curve->eccp_n, SM2_WORD_LEN,
							   SM2_ZERO_ALL, SM2_INTEGER_TOO_BIG, ret_sm2_success);
	if (ret != ret_sm2_success)
		goto END;

	// make sure s in [1, n-1]
	reverse_byte_array(signature + SM2_BYTE_LEN, (uint8_t *)s,
					   SM2_BYTE_LEN);
	ret = uint32_integer_check(s, sm2_curve->eccp_n, SM2_WORD_LEN,
							   SM2_ZERO_ALL, SM2_INTEGER_TOO_BIG, ret_sm2_success);
	if (ret != ret_sm2_success)
		goto END;

	// t = (r+s) mod n
	ret = pke_mod_add(sm2_curve->eccp_n, r, s, t, SM2_WORD_LEN);
	if (ret != PKE_SUCCESS)
		goto END;

	// if t is 0, refuse the signature
	if (uint32_big_num_check_zero(t, SM2_WORD_LEN)) {
		ret = SM2_ZERO_ALL;
		goto END;
	}

#ifdef SM2_HIGH_SPEED
	ret = eccp_point_mul_shamir_safe(sm2_curve,
									 s, sm2_curve->eccp_Gx, sm2_curve->eccp_Gy,
									 t, tmp + 2 * SM2_WORD_LEN, tmp + 3 * SM2_WORD_LEN,
									 tmp, NULL);
#else
	//[s]G
	ret = eccp_point_mul(sm2_curve, s, sm2_curve->eccp_Gx,
						 sm2_curve->eccp_Gy, tmp, tmp + SM2_WORD_LEN);
	if (ret != PKE_SUCCESS)
		goto END;

	//[t]PA
	ret = eccp_point_mul(sm2_curve, t, tmp + 2 * SM2_WORD_LEN,
						 tmp + 3 * SM2_WORD_LEN, tmp + 2 * SM2_WORD_LEN, tmp + 3 * SM2_WORD_LEN);
	if (ret != PKE_SUCCESS)
		goto END;

	//[s]G + [t]PA
	ret = eccp_point_add(sm2_curve, tmp, tmp + SM2_WORD_LEN,
						 tmp + 2 * SM2_WORD_LEN, tmp + 3 * SM2_WORD_LEN, tmp, NULL);
#endif
	if (ret != PKE_SUCCESS)
		goto END;

	// e = e mod n
	reverse_byte_array(E, (uint8_t *)e, SM2_BYTE_LEN);
	if (uint32_big_num_cmp(e, SM2_WORD_LEN, sm2_curve->eccp_n,
						   SM2_WORD_LEN) >= 0) {
		ret = pke_sub(e, sm2_curve->eccp_n, e, SM2_WORD_LEN);
		if (ret != PKE_SUCCESS)
			goto END;
	}

	// tmp = x1 mod n
	if (uint32_big_num_cmp(tmp, SM2_WORD_LEN, sm2_curve->eccp_n,
						   SM2_WORD_LEN) >= 0) {
		ret = pke_sub(tmp, sm2_curve->eccp_n, tmp, SM2_WORD_LEN);
		if (ret != PKE_SUCCESS)
			goto END;
	}

	// tmp = e + x1 mod n
	ret = pke_mod_add(sm2_curve->eccp_n, e, tmp, tmp, SM2_WORD_LEN);
	if (ret != PKE_SUCCESS)
		goto END;

	// cmp
	if (uint32_big_num_cmp(tmp, SM2_WORD_LEN, r, SM2_WORD_LEN)) {
		ret = SM2_VERIFY_FAILED;
		goto END;
	}

	// success
	ret = SM2_SUCCESS;

END:

	return ret;
}

uint32_t pke_sm2_sign(const struct pke_ec_curve *curve, uint8_t E[32],
					  uint8_t rand_k[32], uint8_t pri_key[32], uint8_t signature[64])
{
	uint32_t e[SM2_WORD_LEN], k[SM2_WORD_LEN], dA[SM2_WORD_LEN];
	uint32_t r[SM2_WORD_LEN], s[SM2_WORD_LEN];
	uint32_t ret;
	uint32_t ret_sm2_success = SM2_SUCCESS;

	if (NULL == E || NULL == pri_key || NULL == signature)
		return SM2_BUFFER_NULL;
	// e = e mod n
	reverse_byte_array(E, (uint8_t *)e, SM2_BYTE_LEN);
	if (uint32_big_num_cmp(e, SM2_WORD_LEN, curve->eccp_n,
						   SM2_WORD_LEN) >= 0) {
		ret = pke_sub(e, curve->eccp_n, e, SM2_WORD_LEN);
		if (ret != PKE_SUCCESS)
			return ret;
	}
	// make sure pri_key in [1, n-2]
	reverse_byte_array(pri_key, (uint8_t *)dA, SM2_BYTE_LEN);
	ret = uint32_integer_check(dA, (uint32_t *)curve->eccp_n_1, SM2_WORD_LEN,
							   SM2_ZERO_ALL, SM2_INTEGER_TOO_BIG, ret_sm2_success);
	if (ret != ret_sm2_success)
		return ret;

	if (rand_k)
		reverse_byte_array(rand_k, (uint8_t *)k, SM2_BYTE_LEN);
	else {
SM2_SIGN_LOOP:
		get_random_bytes((uint8_t *)k, SM2_BYTE_LEN);
	}
	ret = sm2_sign_with_k(curve, e, k, dA, r, s);
	if((SM2_ZERO_ALL == ret || SM2_INTEGER_TOO_BIG == ret) && (NULL == rand_k))
		goto SM2_SIGN_LOOP;

	if (ret == ret_sm2_success) {
		reverse_byte_array((uint8_t *)r, signature, SM2_BYTE_LEN);
		reverse_byte_array((uint8_t *)s, signature + SM2_BYTE_LEN,
						   SM2_BYTE_LEN);

		return ret_sm2_success;
	} else
		return ret;
}

uint32_t pke_ecdsa_verify(const struct pke_ec_curve *curve, uint8_t *E,
						  uint32_t e_byte_len, uint8_t *pub_key_x, uint8_t *pub_key_y,
						  uint8_t *signature)
{
	uint32_t tmp_len;
	uint32_t n_byte_len;
	uint32_t n_word_len;
	uint32_t p_byte_len;
	uint32_t p_word_len;
	uint32_t max_word_len;
	uint32_t e[ECCP_MAX_WORD_LEN], r[ECCP_MAX_WORD_LEN], s[ECCP_MAX_WORD_LEN];
	uint32_t tmp[ECCP_MAX_WORD_LEN], x[ECCP_MAX_WORD_LEN];
	uint32_t ret;
	uint32_t ret_ecdsa_success = ECDSA_SUCCESS;

	if (NULL == curve || NULL == pub_key_x || NULL == pub_key_y || NULL == signature)
		return ECDSA_POINTOR_NULL;
	else if (curve->p_bit_len > ECCP_MAX_BIT_LEN)
		return ECDSA_INVALID_INPUT;
	// e could be zero
	if (NULL == E)
		e_byte_len = 0;
	n_byte_len = GET_BYTE_LEN(curve->n_bit_len);
	n_word_len = GET_WORD_LEN(curve->n_bit_len);
	p_byte_len = GET_BYTE_LEN(curve->p_bit_len);
	p_word_len = GET_WORD_LEN(curve->p_bit_len);
	max_word_len = GET_MAX_LEN(n_word_len, p_word_len);

	// make sure r in [1, n-1]
	memset(((uint8_t *)r) + n_byte_len, 0, (n_word_len << 2) - n_byte_len);
	reverse_byte_array(signature, (uint8_t *)r, n_byte_len);
	ret = uint32_integer_check(r, curve->eccp_n, n_word_len,
							   ECDSA_ZERO_ALL, ECDSA_INTEGER_TOO_BIG, ret_ecdsa_success);
	if (ret != ret_ecdsa_success)
		return ret;
	// make sure s in [1, n-1]
	memset(((uint8_t *)s) + n_byte_len, 0, (n_word_len << 2) - n_byte_len);
	reverse_byte_array(signature + n_byte_len, (uint8_t *)s, n_byte_len);
	ret = uint32_integer_check(s, curve->eccp_n, n_word_len,
							   ECDSA_ZERO_ALL, ECDSA_INTEGER_TOO_BIG, ret_ecdsa_success);
	if (ret != ret_ecdsa_success)
		return ret;

	// tmp = s^(-1) mod n
	ret = pke_mod_inv(curve->eccp_n, s, tmp, n_word_len, n_word_len);
	if (ret != PKE_SUCCESS)
		return ret;

	// get integer e from hash value e(according to SEC1-V2 2009)
	uint32_clear(e, n_word_len);
	if (curve->n_bit_len >= (e_byte_len << 3)) {
		if (E) {
			reverse_byte_array((uint8_t *)E, (uint8_t *)e, e_byte_len);
		}
	} else {
		if (E) {
			memcpy(e, E, n_byte_len);
			reverse_byte_array((uint8_t *)E, (uint8_t *)e, n_byte_len);
		}
		tmp_len = (curve->n_bit_len) & 7;
		if (tmp_len)
			big_div_2n(e, n_word_len, 8 - tmp_len);
	}

	// get e = e mod n, i.e., make sure e in [0, n-1]
	if (uint32_big_num_cmp(e, n_word_len, curve->eccp_n, n_word_len) >= 0) {
		ret = pke_sub(e, curve->eccp_n, e, n_word_len);
		if (ret != PKE_SUCCESS)
			return ret;
	}

	ret = pke_set_modulus_and_pre_monts(curve->eccp_n, curve->eccp_n_h,
										curve->n_bit_len);
	if (ret != PKE_SUCCESS)
		return ret;

	// x =  e*(s^(-1)) mod n
	ret = pke_mod_mul_internal(e, tmp, x, n_word_len);
	if (ret != PKE_SUCCESS)
		return ret;

	// tmp =  r*(s^(-1)) mod n
	ret = pke_mod_mul_internal(r, tmp, tmp, n_word_len);
	if (ret != PKE_SUCCESS)
		return ret;

	// check public key
	memset(e, 0, (max_word_len << 2) - p_byte_len);
	memset(s, 0, (max_word_len << 2) - p_byte_len);
	reverse_byte_array(pub_key_x, (uint8_t *)e, p_byte_len);
	reverse_byte_array(pub_key_y, (uint8_t *)s, p_byte_len);
	ret = eccp_point_verify(curve, e, s);
	if (ret != PKE_SUCCESS)
		return ret;

	if (curve->eccp_half_Gx && curve->eccp_half_Gy)
		ret = eccp_point_mul_shamir(curve, tmp, e, s, x,
									curve->eccp_Gx, curve->eccp_Gy, e, s);
	else
		ret = ~(PKE_SUCCESS);

	if (ret != PKE_SUCCESS) {
		ret = eccp_point_mul(curve, tmp, e, s, e, s);
		if (ret != PKE_SUCCESS)
			return ret;

		if (!uint32_big_num_check_zero(x, n_word_len)) {
			ret = eccp_point_mul(curve, x, curve->eccp_Gx, curve->eccp_Gy, x,
								 tmp);
			if (ret != PKE_SUCCESS)
				return ret;

			ret = eccp_point_add(curve, e, s, x, tmp, e, s);
			if (ret != PKE_SUCCESS)
				return ret;
		}
	}

	// x = x1 mod n
	ret = pke_mod(e, p_word_len, curve->eccp_n, curve->eccp_n_h, n_word_len,
				  tmp);
	if (ret != PKE_SUCCESS)
		return ret;

	if (uint32_big_num_cmp(tmp, n_word_len, r, n_word_len))
		return ECDSA_VERIFY_FAILED;
	else
		return ECDSA_SUCCESS;
}

uint32_t ecdsa_sign_uint32(const struct pke_ec_curve *curve, uint32_t *e, uint32_t *k, uint32_t *dA, uint32_t *r, uint32_t *s)
{
	uint32_t n_word_len;
	uint32_t p_word_len;
	uint32_t tmp1[ECCP_MAX_WORD_LEN];
	uint32_t ret;
	uint32_t ret_ecdsa_success = ECDSA_SUCCESS;

	if(NULL == curve || NULL == e || NULL == k || NULL == dA || NULL == r || NULL == s)
		return ECDSA_POINTOR_NULL;
	else if(curve->p_bit_len > ECCP_MAX_BIT_LEN)
		return ECDSA_INVALID_INPUT;

	n_word_len = GET_WORD_LEN(curve->n_bit_len);
	p_word_len = GET_WORD_LEN(curve->p_bit_len);

	//make sure k in [1, n-1]
	ret = uint32_integer_check(k, curve->eccp_n, n_word_len, ECDSA_ZERO_ALL, ECDSA_INTEGER_TOO_BIG,
		ret_ecdsa_success);
	if(ret_ecdsa_success != ret)
		return ret;

	//get x1
	if(curve->eccp_half_Gx && curve->eccp_half_Gy)
		ret = eccp_point_mul_base(curve, k, tmp1, NULL);
	else
		ret = eccp_point_mul(curve, k, curve->eccp_Gx, curve->eccp_Gy, tmp1, NULL);  //y coordinate is not needed
	if(PKE_SUCCESS != ret)
		return ret;

	//r = x1 mod n
	ret = pke_mod(tmp1, p_word_len, curve->eccp_n, curve->eccp_n_h, n_word_len, r);
	if(PKE_SUCCESS != ret)
		return ret;
	else if(uint32_big_num_check_zero(r, n_word_len))//make sure r is not zero
		return ECDSA_ZERO_ALL;

	ret = pke_set_modulus_and_pre_monts(curve->eccp_n, curve->eccp_n_h, curve->n_bit_len);
	if(PKE_SUCCESS != ret)
		return ret;

	//tmp1 =  r*dA mod n
	ret = pke_mod_mul_internal(r, dA, tmp1, n_word_len);
	if(PKE_SUCCESS != ret)
		return ret;

	//tmp1 = e + r*dA mod n
	ret = pke_mod_add(curve->eccp_n, e, tmp1, tmp1, n_word_len);
	if(PKE_SUCCESS != ret)
		return ret;

	//s = k^(-1) mod n
	ret = pke_mod_inv(curve->eccp_n, k, s, n_word_len, n_word_len);
	if(PKE_SUCCESS != ret)
		return ret;

	//s = (k^(-1))*(e + r*dA) mod n
	ret = pke_mod_mul_internal(s, tmp1, s, n_word_len);
	if(PKE_SUCCESS != ret)
		return ret;

	//make sure s is not zero
	if(uint32_big_num_check_zero(s, n_word_len))
		return ECDSA_ZERO_ALL;
	else
		return ECDSA_SUCCESS;
}

uint32_t pke_ecdsa_sign(const struct pke_ec_curve *curve, uint8_t *E,
						  uint32_t e_byte_len, uint8_t *rand_k, uint8_t *priv_key, uint8_t *signature)
{
	uint32_t tmpLen;
	uint32_t n_byte_len;
	uint32_t n_word_len;
	uint32_t e[ECCP_MAX_WORD_LEN], k[ECCP_MAX_WORD_LEN], dA[ECCP_MAX_WORD_LEN];
	uint32_t r[ECCP_MAX_WORD_LEN], s[ECCP_MAX_WORD_LEN];
	uint32_t ret;
	uint32_t ret_ecdsa_success = ECDSA_SUCCESS;

	if(NULL == curve || NULL == priv_key || NULL == signature)
		return ECDSA_POINTOR_NULL;
	else if(curve->p_bit_len > ECCP_MAX_BIT_LEN)
		return ECDSA_INVALID_INPUT;

	//E could be zero
	if(NULL == E)
		e_byte_len = 0;

	n_byte_len = GET_BYTE_LEN(curve->n_bit_len);
	n_word_len = GET_WORD_LEN(curve->n_bit_len);

	//get integer e from hash value E(according to SEC1-V2 2009)
	uint32_clear(e, n_word_len);
	if(curve->n_bit_len >= (e_byte_len<<3)) {     //in this case, make E as e directly
		if (E) {
			reverse_byte_array((uint8_t *)E, (uint8_t *)e, e_byte_len);
		}
	} else {                                     //in this case, make left n_bit_len bits of E as e
		if (E) {
			reverse_byte_array((uint8_t *)E, (uint8_t *)e, n_byte_len);
		}
		tmpLen = (curve->n_bit_len)&7;
		if(tmpLen) {
			big_div2n(e, n_word_len, 8-tmpLen);
		}
	}

	//get e = e mod n, i.e., make sure e in [0, n-1]
	if(uint32_big_num_cmp(e, n_word_len, curve->eccp_n, n_word_len) >= 0) {
		ret = pke_sub(e, curve->eccp_n, e, n_word_len);
		if(PKE_SUCCESS != ret)
			return ret;
	}

	//make sure priv_key in [1, n-1]
	memset(((uint8_t *)dA)+n_byte_len, 0, (n_word_len<<2)-n_byte_len);
	reverse_byte_array((uint8_t *)priv_key, (uint8_t *)dA, n_byte_len);
	ret = uint32_integer_check(dA, curve->eccp_n, n_word_len, ECDSA_ZERO_ALL, ECDSA_INTEGER_TOO_BIG,
		ret_ecdsa_success);
	if(ret_ecdsa_success != ret)
		return ret;

	//get k
	memset(((uint8_t *)k)+n_byte_len, 0, (n_word_len<<2)-n_byte_len);
	if(rand_k) {
		reverse_byte_array(rand_k, (uint8_t *)k, n_byte_len);
	} else {
ECDSA_SIGN_LOOP:
		get_random_bytes((uint8_t *)k, n_byte_len);	
	}
	ret = ecdsa_sign_uint32(curve, e, k, dA, r, s);
	if((ECDSA_ZERO_ALL == ret || ECDSA_INTEGER_TOO_BIG == ret) && (NULL == rand_k))
		goto ECDSA_SIGN_LOOP;
		
	if(ret_ecdsa_success != ret)
	{
		return ret;
	}
	else
	{
		reverse_byte_array((uint8_t *)r, signature, n_byte_len);
		reverse_byte_array((uint8_t *)s, signature+n_byte_len, n_byte_len);

		return ret_ecdsa_success;
	}
}

static struct akcipher_alg bst_rsa = {
	.encrypt = bst_rsa_enc,
	.decrypt = bst_rsa_dec,
	.set_priv_key = bst_rsa_set_priv_key,
	.set_pub_key = bst_rsa_set_pub_key,
	.max_size = bst_rsa_max_size,
	.init = bst_rsa_init_tfm,
	.exit = bst_rsa_exit_tfm,
	.base = {
		.cra_name = "bst_rsa",
		.cra_driver_name = "bst-rsa",
		.cra_priority = 100,
		.cra_module = THIS_MODULE,
		.cra_ctxsize = sizeof(struct bst_rsa_mpi_key),
	},

};

static struct akcipher_alg bst_sm2 = {
	.verify = bst_sm2_verify,
	.sign = bst_sm2_sign,
	.set_pub_key = bst_sm2_set_pub_key,
	.set_priv_key = bst_sm2_set_pri_key,
	.max_size = bst_sm2_max_size,
	.init = bst_sm2_init_tfm,
	.exit = bst_sm2_exit_tfm,
	.base = {
		.cra_name = "bst_sm2",
		.cra_driver_name = "bst-sm2",
		.cra_priority = 100,
		.cra_module = THIS_MODULE,
		.cra_ctxsize = sizeof(struct bst_mpi_ec_ctx),
	},
};

static struct akcipher_alg bst_ecdsa = {
	.verify = bst_ecdsa_verify,
	.sign = bst_ecdsa_sign,
	.set_pub_key = bst_ecdsa_set_pub_key,
	.set_priv_key = bst_ecdsa_set_priv_key,
	.max_size = bst_ecdsa_max_size,
	.init = bst_ecdsa_init_tfm,
	.exit = bst_ecdsa_exit_tfm,
	.base = {
		.cra_name = "bst_ecdsa",
		.cra_driver_name = "bst-ecdsa",
		.cra_priority = 100,
		.cra_module = THIS_MODULE,
		.cra_ctxsize = sizeof(struct bst_ecdsa_ctx),
	},
};

static int bst_pke_irq_handler(struct pke *dev)
{
	// pr_info("bst_pke_irq_handler");
	pke_disable_interrupt();
	return 0;
}

static irqreturn_t bst_pke_irq(int irq, void *dev_id)
{
	struct pke *hdev = dev_id;
	u32 stat, enabled;

	enabled = readl_relaxed(hdev->base + PKE_IMCR);
	stat = readl_relaxed(hdev->base + PKE_MISR);

	dev_dbg(hdev->dev, "enabled=%#x stat=%#x\n", enabled, stat);
	if (!enabled || !stat)
		return IRQ_NONE;

	bst_pke_irq_handler(hdev);
	// writel_relaxed(readl_relaxed(hdev->base + PKE_RISR) & (~1),
	// 			   hdev->base + PKE_RISR);

	return IRQ_HANDLED;
}

static int bst_pke_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pke *pke = NULL;
	int ret;
	uint32_t v_major, v_minor;

	pke = devm_kzalloc(dev, sizeof(*pke), GFP_KERNEL);
	if (!pke)
		return -ENOMEM;

	pke->dev = dev;
	pke->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pke->base))
		return PTR_ERR(pke->base);

	pke->irq = platform_get_irq(pdev, 0);
	if (pke->irq < 0) {
		if (pke->irq != -EPROBE_DEFER)
			dev_err(dev, "cannot get irq\n");
		return pke->irq;
	}

	ret = devm_request_irq(pke->dev, pke->irq, bst_pke_irq, IRQF_SHARED,
						   dev_name(pke->dev), pke);
	if (ret) {
		dev_err(pke->dev, "failure requesting irq %i: %d\n",
				pke->irq, ret);
		return ret;
	}

	global_pke = pke;
	global_pke->pke_status = PKE_IS_AVAILABLE;
	platform_set_drvdata(pdev, pke);

	if (bst_sec_sa_pke_enable) {
		mutex_lock(&refcnt_lock);
		if (refcnt++ == 0) {
			ret = crypto_register_akcipher(&bst_rsa);
			if (ret) {
				dev_err(dev, "Failed to register rsa\n");
			}else{
				dev_info(&pdev->dev, "BST rsa algorithms registered\n");
			}
			ret = crypto_register_akcipher(&bst_sm2);
			if (ret) {
				dev_err(dev, "Failed to register sm2\n");
				crypto_unregister_akcipher(&bst_rsa);
				refcnt--;
				mutex_unlock(&refcnt_lock);
				return ret;
			}else{
				dev_info(&pdev->dev, "BST sm2 algorithms registered\n");
			}
			ret = crypto_register_akcipher(&bst_ecdsa);
			if (ret) {
				dev_err(dev, "Failed to register ecdsa\n");
				crypto_unregister_akcipher(&bst_rsa);
				crypto_unregister_akcipher(&bst_sm2);
				refcnt--;
				mutex_unlock(&refcnt_lock);
				return ret;
			}else{
				dev_info(&pdev->dev, "BST ecdsa algorithms registered\n");
			}
		}
		mutex_unlock(&refcnt_lock);
	} else {
		dev_info(&pdev->dev, "BST hash driver loaded but algorithms disabled (bst_sec_sa_pke_enable=0)\n");
	}

	pke_get_version(pke->base, &v_major, &v_minor);
	dev_info(dev, "Hardware version: v%d.%d\n", v_major, v_minor);

	return 0;
}

static int bst_pke_remove(struct platform_device *pdev)
{
	// struct pke *pke = platform_get_drvdata(pdev);
	mutex_lock(&refcnt_lock);
	if (!--refcnt) {
		crypto_unregister_akcipher(&bst_rsa);
		crypto_unregister_akcipher(&bst_sm2);
		crypto_unregister_akcipher(&bst_ecdsa);
	}
	mutex_unlock(&refcnt_lock);
	platform_set_drvdata(pdev, NULL);

	global_pke = NULL;

	return 0;
}

void sa_enbale_change_pke(void){
	mutex_lock(&refcnt_lock);
	if (bst_sec_sa_pke_enable && refcnt == 0) {
		crypto_register_akcipher(&bst_rsa);
		crypto_register_akcipher(&bst_sm2);
		crypto_register_akcipher(&bst_ecdsa);
		refcnt = 1;
	} else if (!bst_sec_sa_pke_enable && refcnt) {
		crypto_unregister_akcipher(&bst_rsa);
		crypto_unregister_akcipher(&bst_sm2);
		crypto_unregister_akcipher(&bst_ecdsa);
		refcnt = 0;
	}
	mutex_unlock(&refcnt_lock);
}

static const struct of_device_id bst_pke_match[] = {
	{.compatible = "bst,c1200-pke"},
	{}};
MODULE_DEVICE_TABLE(of, bst_hash_match);

static struct platform_driver bst_pke_driver = {
	.probe = bst_pke_probe,
	.remove = bst_pke_remove,
	.driver = {
		.name = "bst-pke",
		.of_match_table = of_match_ptr(bst_pke_match),
	}};
module_platform_driver(bst_pke_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("BST Symmetric Key Engine driver");
