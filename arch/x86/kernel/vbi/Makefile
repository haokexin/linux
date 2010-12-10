#
# Makefile for the x86 vbi.
#

obj-y		= wrhv.o wrhv_initrd.o syscalls.o

# wrhv embeds an initrd in the vmlinux
$(obj)/wrhv_initrd.o: FORCE
	@ $(AS) -$(BITS) $(srctree)/$(src)/wrhv_initrd.S -o $@
ifneq (X$(INITRD), X)
	@ $(OBJCOPY) --add-section .initrd=$(INITRD) $@
	@ $(OBJCOPY) --set-section-flags .initrd=alloc,load $@
endif
