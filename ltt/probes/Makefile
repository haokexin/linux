# LTTng tracing probes

ifdef CONFIG_FTRACE
CFLAGS_REMOVE_kernel-trace.o = -pg
CFLAGS_REMOVE_mm-trace.o = -pg
endif

obj-$(CONFIG_LTT_TRACEPROBES)	+= kernel-trace.o mm-trace.o
