// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#ifndef _BST_VIRT_DRM_CONNECTOR_H_
#define _BST_VIRT_DRM_CONNECTOR_H_

#define BST_EDID_LENGTH 128

static noinline void bst_conn_edid_bit_change(uint8_t *byte, uint8_t shift, uint8_t width,
		    const uint32_t in)
{
	*byte = ((in & ((((uint16_t)1) << width) - 1)) << shift) |
		(*byte & ~GENMASK(shift + width - 1, shift));
}


static __maybe_unused noinline void bst_conn_edid_byte_gen(uint8_t *bhi, uint8_t ohi, uint8_t nhi, uint8_t *blo,
		  uint8_t olo, uint8_t nlo, const uint32_t dtd_data)
{
	bst_conn_edid_bit_change(bhi, ohi, nhi, (dtd_data >> nlo));
	bst_conn_edid_bit_change(blo, olo, nlo, dtd_data & GENMASK(nlo - 1, 0));
}

static __maybe_unused noinline int bst_edid_block_checksum(const u8 *raw_edid)
{
	int i;
	u8 csum = 0, crc = 0;

	for (i = 0; i < BST_EDID_LENGTH - 1; i++)
		csum += raw_edid[i];

	crc = 0x100 - csum;

	return crc;
}

void panel_edid_bit_change(uint8_t *byte, uint8_t shift, uint8_t width,
		    const uint32_t in);
int panel_edid_block_checksum(const u8 *raw_edid);
int panel_timing_to_edid(u8 *edid, struct drm_connector *connector);
int bst_virt_drm_get_edid_block(void *data, u8 *buf, unsigned int block,
				  size_t len);
int bst_virt_drm_connector_get_edid(struct bst_virt_connector *vconn);
#endif /*_BST_VIRT_DRM_CONNECTOR_H_*/
