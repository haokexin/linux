#
# Makefile for SPEAr13XX machine series
#

# common files
obj-y					+= spear13xx.o fsmc-nor.o
obj-$(CONFIG_SMP)			+= platsmp.o headsmp.o
obj-$(CONFIG_HOTPLUG_CPU)		+= hotplug.o
obj-$(CONFIG_DW_PCIE)			+= dw_pcie.o
obj-$(CONFIG_SPEAR_PCIE_REV341)		+= spear_pcie_rev_341.o
obj-$(CONFIG_SPEAR_PCIE_REV370)		+= spear_pcie_rev_370.o
obj-$(CONFIG_PM)			+= pm.o wakeup.o sleep_1340.o sleep_13xx.o
obj-y					+= db9000_lcd.o

# spear1310_reva specific files
obj-$(CONFIG_CPU_SPEAR1310_REVA)	+= clock.o spear1310_reva.o

# spear1310_reva boards files
obj-$(CONFIG_MACH_SPEAR1310_REVA_EVB)	+= spear1310_reva_evb.o

# spear1310 specific files
obj-$(CONFIG_CPU_SPEAR1310)		+= spear1310_clock.o spear1310.o

# spear1310 boards files
obj-$(CONFIG_MACH_SPEAR1310_EVB)	+= spear1310_evb.o
obj-$(CONFIG_MACH_R1801E)		+= r1801e.o

# spear1340 specific files
obj-$(CONFIG_CPU_SPEAR1340)		+= spear1340_clock.o spear1340.o

# spear1340 boards files
obj-$(CONFIG_MACH_SPEAR1340_EVB)	+= spear1340_evb.o
obj-$(CONFIG_SPEAR1340_PLUG_BOARDS)	+= spear1340_plug_boards.o
