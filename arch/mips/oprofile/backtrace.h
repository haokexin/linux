/*
 * Mips specific backtracing code for oprofile
 *
 * Copyright (c) 2008 Windriver Systems, Inc.
 *
 * Author: David Lerner <david.lerner@windriver.com>
 *
 * Based on i386 oprofile backtrace code by John Levon, David Smith
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef BACKTRACE_H
#define BACKTRACE_H

/*
 * The mips backtrace crawl - this may take a while since
 * frame pointers are of no value, and the code traces and follows
 * instructions to find function epilogs or prologs.
 */
extern void mips_backtrace(struct pt_regs *const regs, unsigned int depth);

#endif /* BACKTRACE_H */
