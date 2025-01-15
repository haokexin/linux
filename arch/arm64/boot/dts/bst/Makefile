ifeq ($(CONFIG_SECOND_KERNEL), )

# Enables support for device-tree overlays
DTC_FLAGS := -@

dtb-$(CONFIG_ARCH_BSTC1200) += \
		bstc1200-adas.dtb bstc1200-db.dtb bstc1200-ivi.dtb bstc1200-cdcu-db.dtb bstc1200-cdcu-adas.dtb c1200-evb-overlay.dtb c1200-cdcu-overlay.dtb \
		bstc1200-evm-adas-slt.dtb  bstc1200-evm-db-slt.dtb \
		bstc1200-cdcu-adas-mass.dtb  bstc1200-cdcu-db-mass.dtb \
		bstc1200-recovery.dtb

subdir-y	+= overlay
else
dtb-$(CONFIG_ARCH_BSTC1200) += \
		bstc1200-adas-kdump.dtb bstc1200-db-kdump.dtb bstc1200-ivi-kdump.dtb
endif

clean-files	:= *.dtb
