/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2024 Black Sesame Technologies, Inc.
 *
 * Author: Xuran Yang <xuran.yang@bst.ai>
 */

#ifndef _BST_PCIE_H
#define _BST_PCIE_H

#include <linux/pci-epc.h>

#ifdef CONFIG_PCIE_BST_EP
int bst_pcie_ep_db_irq_alloc(struct pci_epc *epc, u8 func_no, u8 vfunc_no);
int bst_pcie_ep_db_irq_request(struct pci_epc *epc, u8 func_no, u8 vfunc_no, u32 irq_no,
			       int (*handler)(int irq, void *arg), void *arg);
void bst_pcie_ep_db_irq_free(struct pci_epc *epc, u8 func_no, u8 vfunc_no, u32 irq_no);
int bst_pcie_ep_db_info_get(struct pci_epc *epc, u8 func_no, u8 vfunc_no, u32 irq_no,
			    u32 *bar_no, u32 *offset, u32 *msg);
#else
int bst_pcie_ep_db_irq_alloc(struct pci_epc *epc, u8 func_no, u8 vfunc_no)
{
	return -EINVAL;
}

int bst_pcie_ep_db_irq_request(struct pci_epc *epc, u8 func_no, u8 vfunc_no, u32 irq_no,
			       int (*handler)(int irq, void *arg), void *arg)
{
	return -EINVAL;
}

void bst_pcie_ep_db_irq_free(struct pci_epc *epc, u8 func_no, u8 vfunc_no, u32 irq_no)
{
}

int bst_pcie_ep_db_info_get(struct pci_epc *epc, u8 func_no, u8 vfunc_no, u32 irq_no,
			    u32 *bar_no, u32 *offset, u32 *msg)
{
	return -EINVAL;
}
#endif

#endif /* _BST_PCIE_H */
