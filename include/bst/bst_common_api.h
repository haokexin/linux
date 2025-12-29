/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _BST_COMMON_API_H
#define _BST_COMMON_API_H

int send_dtc_to_safety_svc(u32 dtc);
int get_psmid_from_safety_lib(uint8_t block_id_in ,uint8_t *block_id_out, uint32_t *psm_id_out);

#endif

