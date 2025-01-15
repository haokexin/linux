// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
 
#ifndef _BST_PRODUCT_H_
#define _BST_PRODUCT_H_

#define BSTDC_CORE_ID(__product, __major, __minor, __status) \
	((((__product) & 0xFFFF) << 16) | (((__major) & 0xF) << 12) | \
	(((__minor) & 0xF) << 8) | ((__status) & 0xFF))

#define BSTDC_CORE_ID_PRODUCT_ID(__core_id) ((__u32)(__core_id) >> 16)
#define BSTDC_CORE_ID_MAJOR(__core_id)      (((__u32)(__core_id) >> 12) & 0xF)
#define BSTDC_CORE_ID_MINOR(__core_id)      (((__u32)(__core_id) >> 8) & 0xF)
#define BSTDC_CORE_ID_STATUS(__core_id)     (((__u32)(__core_id)) & 0xFF)

#define BSTDC_C1200_PRODUCT_ID	0x0071

union bst_config_id {
	struct {
		__u32	max_line_sz:16,
			n_pipelines:2,
			n_scalers:2,
			n_layers:3,
			n_richs:3,
			reserved_bits:6;
	};
	__u32 value;
};

#endif /* _BST_PRODUCT_H_ */
