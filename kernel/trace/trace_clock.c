/*
 * tracing clocks
 *
 *  Copyright (C) 2009 Red Hat, Inc., Ingo Molnar <mingo@redhat.com>
 *
 * Implements 3 trace clock variants, with differing scalability/precision
 * tradeoffs:
 *
 *  -   local: CPU-local trace clock
 *  -  medium: scalable global clock with some jitter
 *  -  global: globally monotonic, serialized clock
 *
 * Tracer plugins will chose a default from these clocks.
 */
#include <linux/spinlock.h>
#include <linux/irqflags.h>
#include <linux/hardirq.h>
#include <linux/module.h>
#include <linux/percpu.h>
#include <linux/sched.h>
#include <linux/ktime.h>
#include <linux/trace_clock.h>
#include <linux/trace-clock.h>
#include <linux/delay.h>

#include "trace.h"

trace_clock_t ftrace_trace_clock __read_mostly = sched_clock;

/*
 * trace_clock_local(): the simplest and least coherent tracing clock.
 *
 * Useful for tracing that does not cross to other CPUs nor
 * does it go through idle events.
 */
u64 notrace trace_clock_local(void)
{
	u64 clock;
	int resched;

	/*
	 * sched_clock() is an architecture implemented, fast, scalable,
	 * lockless clock. It is not guaranteed to be coherent across
	 * CPUs, nor across CPU idle events.
	 */
	resched = ftrace_preempt_disable();
	clock = ftrace_trace_clock();
	ftrace_preempt_enable(resched);

	return clock;
}

/*
 * trace_clock(): 'inbetween' trace clock. Not completely serialized,
 * but not completely incorrect when crossing CPUs either.
 *
 * This is based on cpu_clock(), which will allow at most ~1 jiffy of
 * jitter between CPUs. So it's a pretty scalable clock, but there
 * can be offsets in the trace data.
 */
u64 notrace trace_clock(void)
{
	return cpu_clock(raw_smp_processor_id());
}


/*
 * trace_clock_global(): special globally coherent trace clock
 *
 * It has higher overhead than the other trace clocks but is still
 * an order of magnitude faster than GTOD derived hardware clocks.
 *
 * Used by plugins that need globally coherent timestamps.
 */

/* keep prev_time and lock in the same cacheline. */
static struct {
	u64 prev_time;
	arch_spinlock_t lock;
} trace_clock_struct ____cacheline_aligned_in_smp =
	{
		.lock = (arch_spinlock_t)__ARCH_SPIN_LOCK_UNLOCKED,
	};

u64 notrace trace_clock_global(void)
{
	unsigned long flags;
	int this_cpu;
	u64 now;

	raw_local_irq_save(flags);

	this_cpu = raw_smp_processor_id();
	now = cpu_clock(this_cpu);
	/*
	 * If in an NMI context then dont risk lockups and return the
	 * cpu_clock() time:
	 */
	if (unlikely(in_nmi()))
		goto out;

	arch_spin_lock(&trace_clock_struct.lock);

	/*
	 * TODO: if this happens often then maybe we should reset
	 * my_scd->clock to prev_time+1, to make sure
	 * we start ticking with the local clock from now on?
	 */
	if ((s64)(now - trace_clock_struct.prev_time) < 0)
		now = trace_clock_struct.prev_time + 1;

	trace_clock_struct.prev_time = now;

	arch_spin_unlock(&trace_clock_struct.lock);

 out:
	raw_local_irq_restore(flags);

	return now;
}

/*
 * If the trace clock can distinguish 1us' delta, we consider
 * it has high resolution
 */

#define DELAY_US 1

static bool trace_clock_get_hres(trace_clock_t func)
{
	unsigned long long flags;
	u64 t1, t2, delta;

	/* func() return nanosecs */
	t1 = func();
	raw_local_irq_save(flags);
	udelay(DELAY_US);
	raw_local_irq_restore(flags);
	t2 = func();

	delta = t2 - t1;

	if (delta > 0)
		return 1;

	return 0;
}

static bool trace_clock_registered __read_mostly;
static bool sched_clock_has_hres __read_mostly;
static bool trace_clock_read64_ns_has_hres __read_mostly;

void notrace register_trace_clock(void)
{
	if (trace_clock_registered)
		return;
	/*
	 * If there is a high resolution sched_clock(), no need to
	 * register another one.
	 */
	if (sched_clock_has_hres)
		return;

	if (trace_clock_read64_ns_has_hres) {
		get_trace_clock();
		ftrace_trace_clock = trace_clock_read64_ns;
		trace_clock_registered = 1;
	}
}

void notrace unregister_trace_clock(void)
{
	if (trace_clock_registered) {
		trace_clock_registered = 0;
		ftrace_trace_clock = sched_clock;
		put_trace_clock();
	}
}

static int __init check_res_of_trace_clock(void)
{
	int has_hres;

	pr_info("%s: sched_clock() ", __func__);
	if (trace_clock_get_hres(sched_clock)) {
		pr_cont("high resolution\n");
		sched_clock_has_hres = 1;
	} else {
		pr_cont("low resolution\n");

		get_trace_clock();
		pr_info("%s: trace_clock_read64_ns() ", __func__);
		if (trace_clock_get_hres(trace_clock_read64_ns)) {
			pr_cont("has high resolution\n");
			trace_clock_read64_ns_has_hres = 1;
		} else
			pr_cont("has low resolution\n");
		put_trace_clock();
	}

	has_hres = sched_clock_has_hres | trace_clock_read64_ns_has_hres;
	WARN(!has_hres, "No available high resolution trace clock\n");

	return 0;
}

/*
 * Note: check_res_of_trace_clock() must be called to initialize the
 * trace_clock_read64_ns_has_hres before the register of irqsoff,
 * preemptoff, preemptirqsoff and wakeup tracers, otherwise, the
 * selftest of them will fail.
 *
 * The low-level clocks must be initialized before us, in
 * early_initcall(), otherwise, the kernel may hang after decompressing
 * the kernel.
 */

arch_initcall(check_res_of_trace_clock);
