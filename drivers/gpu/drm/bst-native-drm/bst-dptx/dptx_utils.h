// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#ifndef __DPTX_UTILS_H__
#define __DPTX_UTILS_H__

u8 double_is_equal(uint32_t a, uint32_t b);
uint32_t first_bit_set(uint32_t data);
uint32_t get(uint32_t data, uint32_t mask);
uint32_t set(uint32_t data, uint32_t mask, uint32_t value);
uint8_t get8(uint8_t data, uint8_t mask);
uint8_t set8(uint8_t data, uint8_t mask, uint8_t value);
uint16_t get16(uint16_t data, uint16_t mask);
uint16_t set16(uint16_t data, uint16_t mask, uint16_t value);

int bus_write(struct dptx *dptx, u32 idx, u32 offset, u32 data);
int bus_read(struct dptx *dptx, u32 idx, u32 offset);

int phyif_write(struct dptx *dptx, u32 offset, u32 data);
void phyif_write_mask(struct dptx *dptx, u32 addr, u32 mask, u32 data);
int phyif_read(struct dptx *dptx, u32 offset);
u32 phyif_read_mask(struct dptx *dptx, u32 addr, u32 mask);

int print_buf(u8 *buf, int len);
#endif