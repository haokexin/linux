ifeq ($(CONFIG_SECOND_KERNEL), )

# Enables support for device-tree overlays
DTC_FLAGS := -@
DTC_INCLUDE += $(objtree)/include

dtb-$(CONFIG_ARCH_BSTC1200) += \
		bstc1200-ivi.dtb bstc1200-cdcu-db.dtb \
		bstc1236-cdcu1.0-adas_8c4g.dtb bstc1236-cdcu2.0-adas_8c4g.dtb \
		bstc1296-cdcu1.0-adas_4c2g.dtb \
		bstc1296-cdcu2.0-adas_4c2g.dtb \
		bstc1296-cdcu1.0-adas_8c8g.dtb \
		bstc1296-cdcu2.0-adas_8c8g.dtb \
		bstc1296-evm1.0-adas_4c2g.dtb \
		c1296-cdcu1.0-ivi-4c6g-overlay.dtb c1296-cdcu1.0-ivi-8c8g-overlay.dtb c1296-cdcu2.0-ivi-4c6g-overlay.dtb c1296-cdcu2.0-ivi-8c8g-overlay.dtb c1296-evm1.0-ivi-4c6g-overlay.dtb \
		bstc1200-recovery.dtb bstc1200-recovery-emmc.dtb bstc1200-recovery-ufs.dtb

subdir-y	+= overlay
else
dtb-$(CONFIG_ARCH_BSTC1200) += \
		bstc1296-adas-kdump.dtb bstc1200-db-kdump.dtb bstc1200-ivi-kdump.dtb \
		bstc1236-adas-kdump-emmc.dtb bstc1236-adas-kdump-ufs.dtb
endif

clean-files	:= *.dtb
