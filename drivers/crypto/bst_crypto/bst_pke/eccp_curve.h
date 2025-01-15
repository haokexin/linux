/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (C) 2024 Black Sesame Technologies. Inc.
 */

#ifndef ECCP_CURVE_H
#define ECCP_CURVE_H

#include <linux/types.h>
// sample ecc curve
#define PKE_HP
#define SUPPORT_BRAINPOOLP160R1
#define SUPPORT_SECP192R1
#define SUPPORT_SECP224R1
#define SUPPORT_SECP256R1
#define SUPPORT_SECP384R1
#define SUPPORT_BRAINPOOLP512R1
#define SUPPORT_SECP521R1

// eccp curve struct
#ifdef PKE_HP

struct pke_ec_curve {
	uint32_t p_bit_len; // bit length of prime p
	uint32_t n_bit_len; // bit length of order n
	uint32_t *eccp_p;
	uint32_t *eccp_p_h;
	uint32_t *eccp_a;
	uint32_t *eccp_b;
	uint32_t *eccp_Gx;
	uint32_t *eccp_Gy;
	uint32_t *eccp_n;
	uint32_t *eccp_n_h;
	uint32_t *eccp_half_Gx;
	uint32_t *eccp_half_Gy;
	uint32_t *eccp_n_1;
};
#else

#endif

#ifdef SUPPORT_BRAINPOOLP160R1
extern const struct pke_ec_curve pke_brainpoolp160r1[1];
#endif

#ifdef SUPPORT_SECP192R1
extern const struct pke_ec_curve pke_secp192r1[1];
#endif

#ifdef SUPPORT_SECP224R1
extern const struct pke_ec_curve pke_secp224r1[1];
#endif

#ifdef SUPPORT_SECP256R1
extern const struct pke_ec_curve pke_secp256r1[1];
#endif

#ifdef SUPPORT_SECP384R1
extern const struct pke_ec_curve pke_secp384r1[1];
#endif

#ifdef SUPPORT_BRAINPOOLP512R1
extern const struct pke_ec_curve pke_brainpoolp512r1[1];
#endif

#ifdef SUPPORT_SECP521R1
extern const struct pke_ec_curve pke_secp521r1[1];
#endif

#endif
