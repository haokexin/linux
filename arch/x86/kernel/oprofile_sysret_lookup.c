/**
 * @file oprofile_sysret_lookup.c
 *
 * @remark Copyright 2009 OProfile authors
 * @remark Read the file COPYING
 *
 * @author Dave Lerner
 */

#include <linux/module.h>
#include "oprofile_sysret_lookup.h"

static struct oprofile_sysret_lookup osl;

struct oprofile_sysret_lookup *init_oprofile_sysret(void)
{
	osl.system_call_done = system_call_done;

#if defined(CONFIG_X86_64)
	osl.tracesys_done = tracesys_done;
#elif defined(CONFIG_X86_32)
	osl.ia32_sysenter_done = ia32_sysenter_done;
#endif
#if defined(CONFIG_IA32_EMULATION)
	osl.ia32_syscall_done = ia32_syscall_done;
	osl.ia32_cstar_done = ia32_cstar_done;
	osl.ia32_sysenter_done = ia32_sysenter_done;
#endif
	return &osl;
}
EXPORT_SYMBOL(init_oprofile_sysret);
