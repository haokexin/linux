#ifndef _ASM_X86_STACKTRACE_H
#define _ASM_X86_STACKTRACE_H

extern int kstack_depth_to_print;

struct thread_info;
struct stacktrace_ops;

typedef unsigned long (*walk_stack_t)(struct thread_info *tinfo,
				      unsigned long *stack,
				      unsigned long bp,
				      const struct stacktrace_ops *ops,
				      void *data,
				      unsigned long *end,
				      int *graph,
				      int with_sp);

extern unsigned long
print_context_stack(struct thread_info *tinfo,
		    unsigned long *stack, unsigned long bp,
		    const struct stacktrace_ops *ops, void *data,
		    unsigned long *end, int *graph,
		    int with_sp);

extern unsigned long
print_context_stack_bp(struct thread_info *tinfo,
		       unsigned long *stack, unsigned long bp,
		       const struct stacktrace_ops *ops, void *data,
		       unsigned long *end, int *graph,
		       int with_sp);

/* Generic stack tracer with callbacks */

struct stacktrace_ops {
	void (*warning)(void *data, char *msg);
	/* msg must contain %s for the symbol */
	void (*warning_symbol)(void *data, char *msg, unsigned long symbol);
	void (*address)(void *data, unsigned long address, int reliable);
	/* returns return address and sp for this address  */
	void (*address_with_sp)(void *data, unsigned long address,
		unsigned long sp);
	/* On negative return stop dumping */
	int (*stack)(void *data, char *name);
	walk_stack_t	walk_stack;
};

void dump_trace(struct task_struct *tsk, struct pt_regs *regs,
		unsigned long *stack, unsigned long bp,
		const struct stacktrace_ops *ops, void *data);

/* extends the callbacks to include address_with_sp() */
void dump_trace_extended(struct task_struct *tsk, struct pt_regs *regs,
		unsigned long *stack, unsigned long bp,
		const struct stacktrace_ops *ops, void *data,
		unsigned int with_stack);

#endif /* _ASM_X86_STACKTRACE_H */
