// SPDX-License-Identifier: (GPL-2.0 OR MIT)

/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include "ipc_common.h"

/**
 * 64 bit and less is supported
 * sample usage:  find_bit_zero(0xafffffff, 64, 0)
 */
int32_t find_bit_zero(uint64_t number, int32_t bit_num, unsigned int bit_offset)
{
	uint64_t max = 0;
	uint64_t half_bit_num = bit_num / 2;
	uint64_t half_bit_max = (1ull << half_bit_num) - 1;
	uint64_t high = 0;
	uint64_t low = 0;
	int32_t ret = 0;
	if(bit_num >= 64)
		max = ULLONG_MAX;
	else
		max = (1ull << bit_num) - 1;
	if (number == 0)
		return bit_offset;

	if (number == max || (max == 0 && number + 1 == max))
		return -1;

	high = number >> half_bit_num;
	low = number & half_bit_max;

	ret = find_bit_zero(low, half_bit_num, bit_offset);
	if (ret >= 0)
		return ret;

	ret = find_bit_zero(high, half_bit_num, bit_offset + half_bit_num);
	if (ret >= 0)
		return ret;

	return ret;
}

int32_t find_1_bit(uint64_t number, int32_t bit_num, unsigned int bit_offset)
{
	uint64_t max = 0;
	uint64_t half_bit_num = bit_num / 2;
	uint64_t half_bit_max = (1ull << half_bit_num) - 1;
	uint64_t high = 0;
	uint64_t low = 0;
	int32_t ret = 0;
	if(bit_num >= 64)
		max = ULLONG_MAX;
	else
		max = (1ull << bit_num) - 1;
	if (number == 0)
		return -1;

	if (number == max || (max == 0 && number + 1 == max))
		return bit_offset;

	high = number >> half_bit_num;
	low = number & half_bit_max;

	ret = find_1_bit(low, half_bit_num, bit_offset);
	if (ret >= 0)
		return ret;

	ret = find_1_bit(high, half_bit_num, bit_offset + half_bit_num);
	if (ret >= 0)
		return ret;

	return ret;
}

