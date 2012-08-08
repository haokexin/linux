#ifndef _ASM_UPROBES_H
#define _ASM_UPROBES_H
/*
 * User-space Probes (UProbes) for powerpc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * Copyright (C) IBM Corporation, 2007-2012
 *
 * Adapted from the x86 port by Ananth N Mavinakayanahalli <ananth@in.ibm.com>
 */

#include <linux/notifier.h>

typedef unsigned int uprobe_opcode_t;

#define MAX_UINSN_BYTES			   4
#define UPROBE_XOL_SLOT_BYTES	   (MAX_UINSN_BYTES)

#define UPROBE_SWBP_INSN		 0x7fe00008
#define UPROBE_SWBP_INSN_SIZE	   4 /* swbp insn size in bytes */

struct arch_uprobe {
	u8      insn[MAX_UINSN_BYTES];
};

struct arch_uprobe_task {
};

extern int  arch_uprobe_analyze_insn(struct arch_uprobe *aup, struct mm_struct *mm, unsigned long addr);
extern int  arch_uprobe_pre_xol(struct arch_uprobe *aup, struct pt_regs *regs);
extern int  arch_uprobe_post_xol(struct arch_uprobe *aup, struct pt_regs *regs);
extern bool arch_uprobe_xol_was_trapped(struct task_struct *tsk);
extern int  arch_uprobe_exception_notify(struct notifier_block *self, unsigned long val, void *data);
extern void arch_uprobe_abort_xol(struct arch_uprobe *aup, struct pt_regs *regs);
#endif /* _ASM_UPROBES_H */
