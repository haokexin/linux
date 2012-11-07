/*Usage:
 * Enable build as a module
 * On the target, do the following:
 * $ mount -t debugfs nodev /sys/kernel/debug
 * $ cd /sys/kernel/debug/tracing
 * $ echo 1 > /sys/kernel/debug/tracing/events/printk/enable
 * $ modprobe trace_printk_test
 * $ echo 0 > /sys/kernel/debug/tracing/events/printk/enable
 * $ cat /sys/kernel/debug/tracing/trace
 * Verify the dump log.
 * You should see "XXX printk trace" msgs*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/console.h>

MODULE_AUTHOR("Paul Barrette");
MODULE_DESCRIPTION("Simple kmod for testing printk tracing");
MODULE_LICENSE("GPL");

static void dumb_print_handler (unsigned int l)
{
	printk("XXX printk trace (%d) %s\n", l, __func__);
}
static int __init init_trace_printk_test(void)
{
	/* enable tracing now */
	tracing_on();

	/* mark the trace so that you know where to start looking */
	trace_printk(KERN_INFO "START-trace_printk\n");

	/* call a function that printks */
	dumb_print_handler(1);

	/* dump a few more printks */
	printk(KERN_INFO "XXX printk trace this msg\n");
	dumb_print_handler(2);

	/* mark the trace so that you know where to stop looking */
	trace_printk(KERN_INFO "STOP-trace_printk\n");

	/* disable tracking */
	tracing_off();
	return 0;
}

static void __exit exit_trace_printk_test(void)
{
}

module_init(init_trace_printk_test);
module_exit(exit_trace_printk_test);
