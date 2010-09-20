#ifndef _LINUX_TRACE__CLOCK_H
#define _LINUX_TRACE__CLOCK_H

/*
 * Trace clock
 *
 * Chooses between an architecture specific clock or an atomic logical clock.
 *
 * Copyright (C) 2007,2008 Mathieu Desnoyers (mathieu.desnoyers@polymtl.ca)
 */

#ifdef CONFIG_HAVE_TRACE_CLOCK
#include <asm/trace-clock.h>
#else
#include <asm-generic/trace-clock.h>
#endif /* CONFIG_HAVE_TRACE_CLOCK */

extern u64 notrace trace_clock_cyc2ns(u64 cycles);

static inline u64 notrace trace_clock_read64_ns(void)
{
	return trace_clock_cyc2ns(trace_clock_read64());
}

#endif /* _LINUX_TRACE__CLOCK_H */
