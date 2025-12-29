dtb-$(CONFIG_ARCH_BSTC1200)	+= bst-overlay.dtbo
subdir-y			+= isp/c1200-cdcu

# Define default overlay dtbo
$(obj)/bst-overlay.dtbo:
ifeq ($(CONFIG_C1200_MASS),y)
	ln -sf isp/c1200-cdcu/legacy-2hkx8b-4hkx3c-4hkx3c-max96726d.dtbo $@
else ifeq ($(CONFIG_BST_C1200_ADAS),y)
	ln -sf isp/c1200-cdcu/1hkx8b_1hkx3c-4hkx3c-4hkx3f.dtbo $@
else ifeq ($(CONFIG_VIDEO_BST_ISP_MULTI_OS),y)
    ifeq ($(CONFIG_BST_C1200_DB),y)
	ln -sf isp/c1200-cdcu/1hkx8b_1hkx3c-4hkx3c-4hkx3f.dtbo $@
    else ifeq ($(CONFIG_BST_C1200_IVI),y)
	ln -sf isp/c1200-cdcu/1hkx8b_1hkx3c-4hkx3c-4hkx3f.dtbo $@
    else
	ln -sf isp/c1200-cdcu/disabled.dtbo $@
    endif
else
	ln -sf isp/c1200-cdcu/disabled.dtbo $@
endif
