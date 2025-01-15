// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */

#ifndef _BST_DISP_CONN_H_
#define _BST_DISP_CONN_H_

#include <linux/of.h>
#include <linux/device.h>


#define BST_EDID_LENGTH 128

struct bst_dpu_port {
	unsigned int dpu_id;
	unsigned int pipeline_id;
	unsigned int link_id;
};

struct bst_dpu_connection {
	struct device *host;
	struct bst_dpu_port port;
};

static noinline void bst_conn_edid_bit_change(uint8_t *byte, uint8_t shift, uint8_t width,
		    const uint32_t in)
{
	*byte = ((in & GENMASK(shift - 1, 0)) << shift) |
		(*byte & ~GENMASK(shift + width - 1, shift));
}


static noinline __maybe_unused void bst_conn_edid_byte_gen(uint8_t *bhi, uint8_t ohi, uint8_t nhi, uint8_t *blo,
		  uint8_t olo, uint8_t nlo, const uint32_t dtd_data)
{
	bst_conn_edid_bit_change(bhi, ohi, nhi, (dtd_data >> nlo));
	bst_conn_edid_bit_change(blo, olo, nlo, dtd_data & GENMASK(nlo - 1, 0));
}

static noinline __maybe_unused int bst_edid_block_checksum(const u8 *raw_edid)
{
	int i;
	u8 csum = 0, crc = 0;

	for (i = 0; i < BST_EDID_LENGTH - 1; i++)
		csum += raw_edid[i];

	crc = 0x100 - csum;

	return crc;
}


int bst_get_remote_dpu_connection(struct device_node *endpoint,
		struct bst_dpu_connection *conn);
int bst_get_remote_dpu_connection_by_port(struct device *dev,
        int port, struct bst_dpu_connection *conn);

void bst_select_dpu_output_to_edp(struct bst_dpu_connection *conn);
void bst_select_dpu_output_to_lvds(struct bst_dpu_connection *conn, unsigned int lvds_n);
void bst_select_dpu_output_to_dsi(struct bst_dpu_connection *conn, unsigned int dsi_n);

#endif