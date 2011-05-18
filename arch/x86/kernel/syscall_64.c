/* System call table for x86-64. */

#include <linux/linkage.h>
#include <linux/module.h>
#include <linux/sys.h>
#include <linux/cache.h>
#include <linux/marker.h>
#include <linux/kallsyms.h>
#include <asm/asm-offsets.h>

#define __SYSCALL_COMMON(nr, sym, compat) __SYSCALL_64(nr, sym, compat)

#ifdef CONFIG_X86_X32_ABI
# define __SYSCALL_X32(nr, sym, compat) __SYSCALL_64(nr, sym, compat)
#else
# define __SYSCALL_X32(nr, sym, compat) /* nothing */
#endif

#define __SYSCALL_64(nr, sym, compat) extern asmlinkage void sym(void) ;
#include <asm/syscalls_64.h>
#undef __SYSCALL_64

#define __SYSCALL_64(nr, sym, compat) [nr] = sym,

typedef void (*sys_call_ptr_t)(void);

extern void sys_ni_syscall(void);

const sys_call_ptr_t sys_call_table[__NR_syscall_max+1] = {
	/*
	 * Smells like a compiler bug -- it doesn't work
	 * when the & below is removed.
	 */
	[0 ... __NR_syscall_max] = &sys_ni_syscall,
#include <asm/syscalls_64.h>
};

void ltt_dump_sys_call_table(void *call_data)
{
	int i;
	char namebuf[KSYM_NAME_LEN];

	for (i = 0; i < __NR_syscall_max + 1; i++) {
		sprint_symbol(namebuf, (unsigned long)sys_call_table[i]);
		__trace_mark(0, syscall_state, sys_call_table,
			call_data,
			"id %d address %p symbol %s",
			i, (void*)sys_call_table[i], namebuf);
	}
}
EXPORT_SYMBOL_GPL(ltt_dump_sys_call_table);
