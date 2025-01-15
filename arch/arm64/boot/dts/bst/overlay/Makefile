dtb-$(CONFIG_ARCH_BSTC1200)	+= bst-overlay.dtbo
subdir-y			+= isp/c1200-cdcu

# Define default overlay dtbo
$(obj)/bst-overlay.dtbo:
ifeq ($(CONFIG_C1200_MASS),y)
	ln -sf isp/c1200-cdcu/2hkx8b-4hkx3c-4hkx3c.dtbo $@
else
ifeq ($(CONFIG_C1200_SLT),y)
	ln -sf isp/c1200-cdcu/1jhx8b_1hkx3c-1jhx8b-1jhx8b.dtbo $@
else
	ln -sf isp/c1200-cdcu/2hkx8b-4hkx3c-4hkx3f.dtbo $@
endif
endif