// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#include "dptx_drv.h"
#include "dptx_utils.h"

 /* Find first (least significant) bit set
  * @param[in] data word to search
  * @return bit position or 32 if none is set
  */
uint32_t first_bit_set(uint32_t data)
{
	uint32_t n = 0;

	if (data != 0) {
		for (n = 0; (data & 1) == 0; n++)
			data >>= 1;
	}
	return n;
}

/**
 * Get bit field
 * @param[in] data raw data
 * @param[in] mask bit field mask
 * @return bit field value
 */
uint32_t get(uint32_t data, uint32_t mask)
{
	return ((data & mask) >> first_bit_set(mask));
}

/**
 * Set bit field
 * @param[in] data raw data
 * @param[in] mask bit field mask
 * @param[in] value new value
 * @return new raw data
 */
uint32_t set(uint32_t data, uint32_t mask, uint32_t value)
{
	return (((value << first_bit_set(mask)) & mask) | (data & ~mask));
}

/**
 * Get bit field
 * @param[in] data raw data
 * @param[in] mask bit field mask
 * @return bit field value
 */
uint8_t get8(uint8_t data, uint8_t mask)
{
	return ((data & mask) >> first_bit_set(mask));
}

/**
 * Set bit field
 * @param[in] data raw data
 * @param[in] mask bit field mask
 * @param[in] value new value
 * @return new raw data
 */
uint8_t set8(uint8_t data, uint8_t mask, uint8_t value)
{
	return (((value << first_bit_set(mask)) & mask) | (data & ~mask));
}

/**
 * Get bit field
 * @param[in] data raw data
 * @param[in] mask bit field mask
 * @return bit field value
 */
uint16_t get16(uint16_t data, uint16_t mask)
{
	return ((data & mask) >> first_bit_set(mask));
}

/**
 * Set bit field
 * @param[in] data raw data
 * @param[in] mask bit field mask
 * @param[in] value new value
 * @return new raw data
 */
uint16_t set16(uint16_t data, uint16_t mask, uint16_t value)
{
	return (((value << first_bit_set(mask)) & mask) | (data & ~mask));
}

int bus_write(struct dptx *dptx, u32 idx, u32 offset, u32 data)
{
	if (offset & 0x3)
		return -EIO;

	writel(data, (void *)(dptx->base[idx] + offset));

	return 0;
}

int bus_read(struct dptx *dptx, u32 idx, u32 offset)
{
	return readl((void *)(dptx->base[idx] + offset));
}

int phyif_write(struct dptx *dptx, u32 offset, u32 data)
{
	return bus_write(dptx, PHYIF, offset, data);
}

void phyif_write_mask(struct dptx *dptx, u32 addr, u32 mask, u32 data)
{
	u32 temp;

	temp = set(phyif_read(dptx, addr), mask, data);
	phyif_write(dptx, addr, temp);
}

int phyif_read(struct dptx *dptx, u32 offset)
{
	return bus_read(dptx, PHYIF, offset);
}

u32 phyif_read_mask(struct dptx *dptx, u32 addr, u32 mask)
{
	return get(phyif_read(dptx, addr), mask);
}

u8 double_is_equal(uint32_t a, uint32_t b)
{
	int aux_a, aux_b;

	aux_a = (int)(a*1000);
	aux_b = (int)(b*1000);

	if ((aux_a - aux_b < 5) && (aux_a - aux_b > -5))
		return TRUE;
	else
		return FALSE;
}

int print_buf(u8 *buf, int len)
{
	int i;
	#define PRINT_BUF_SIZE 1024
	char str[PRINT_BUF_SIZE];
	int written = 0;

	written += snprintf(&str[written], PRINT_BUF_SIZE - written, "Buffer:");

	for (i = 0; i < len; i++) {
		if (!(i % 16)) {
			written += snprintf(&str[written],
					    PRINT_BUF_SIZE - written,
					    "\n%04x:", i);

			if (written >= PRINT_BUF_SIZE)
				break;
		}

		written += snprintf(&str[written],
				    PRINT_BUF_SIZE - written,
				    " %02x", buf[i]);

		if (written >= PRINT_BUF_SIZE)
			break;
	}

	return 0;
}
