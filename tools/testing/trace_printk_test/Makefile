# Note: this Makefile is designed to work with the wrlinux "make kds"
# shell.
obj-m := trace_printk_test.o

all:
	make -C $(KBUILD_OUTPUT) M=$(PWD)
clean:
	make -C $(KBUILD_OUTPUT) M=$(PWD) clean
