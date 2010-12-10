#
# Makefile for the vbi goodies.
#


# Most everything is conditional on the Wind River Hypervisor being configured.
# Except the user ABI, which is always present, and defined in syscall_vbi

obj-$(CONFIG_WRHV) += shmem.o wrhv.o interrupt.o paddr.o idle.o io_apic.o lib.o \
                      version.o msg.o ns.o show.o pdc.o device.o procfs.o

obj-$(CONFIG_WRHV_CERT) += sched.o port.o

obj-y += syscall_vbi.o
