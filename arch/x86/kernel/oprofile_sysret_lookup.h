/**
 * @file oprofile_sysret_lookup.h
 *
 * @remark Copyright 2009 OProfile authors
 * @remark Read the file COPYING
 *
 * @author Dave Lerner <dave.lerner@windriver.com>
 */

#ifndef OPROFILE_SYSRET_LOOKUP_H
#define OPROFILE_SYSRET_LOOKUP_H

/* These are defined in scall_xx.S files and the addresses are
 * tested against candidate addresses to determine if
 * a system call frame has been found.
 */
extern void system_call_done(void);
extern void tracesys_done(void);
extern void ia32_syscall_done(void);
extern void ia32_sysenter_done(void);
extern void ia32_cstar_done(void);



struct oprofile_sysret_lookup{
	void (*system_call_done)(void);
	void (*tracesys_done)(void);
	void (*ia32_syscall_done)(void);
	void (*ia32_sysenter_done)(void);
	void (*ia32_cstar_done)(void);
};


/* initialized during first backtrace */
extern struct oprofile_sysret_lookup *init_oprofile_sysret(void);

#endif /* OPROFILE_SYSRET_LOOKUP_H */
