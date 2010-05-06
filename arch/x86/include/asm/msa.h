/************************************************************************
 * msa.h
 *
 * Provide an architecture-specific clock for x86.
 ***********************************************************************/

#ifndef _ASM_X86_MSA_H
# define _ASM_X86_MSA_H

# if defined(CONFIG_MICROSTATE_ACCT_TSC_CLOCKSOURCE)
/*
 * Use the processor's time-stamp counter as a timesource
 */
#  include <linux/compiler.h>
#  include <asm/msr.h>
#  include <asm/div64.h>

#  define MSA_NOW(now)  rdtscll(now)

/*
 * Note that interrupts are configured before cpu_khz is set, so we have
 * to have the check here.
 */
#  define MSA_TO_NSEC(clk) ({\
	msa_time_t _x = 0;			\
	if (likely(cpu_khz)) {			\
		_x = ((clk) * 1000000ULL);	\
		do_div(_x, cpu_khz);		\
	}					\
	_x; })

# elif defined(CONFIG_MICROSTATE_ACCT_SCHED_CLOCK_CLOCKSOURCE)
#  include <asm-generic/msa.h>
# else
#  error "No clocksource defined for Microstate Accounting"
# endif

#endif /* _ASM_X86_MSA_H */
