#
# Makefile config for the Freescale NetcommSW
#
NET_DPA     = $(srctree)/drivers/net/ethernet/freescale
DRV_DPA     = $(srctree)/drivers/net/ethernet/freescale/dpa
NCSW        = $(srctree)/drivers/net/ethernet/freescale/dpa/NetCommSw

ifeq ("$(CONFIG_FMAN_P3040_P4080_P5020)", "y")
EXTRA_CFLAGS +=-include $(NCSW)/p3040_4080_5020_dflags.h
endif
ifeq ("$(CONFIG_FMAN_P1023)", "y")
EXTRA_CFLAGS +=-include $(NCSW)/p1023_dflags.h
endif
ifdef CONFIG_FMAN_T4240
EXTRA_CFLAGS +=-include $(NCSW)/t4240_dflags.h
endif

EXTRA_CFLAGS += -I$(DRV_DPA)/
EXTRA_CFLAGS += -I$(NCSW)/inc
EXTRA_CFLAGS += -I$(NCSW)/inc/cores
EXTRA_CFLAGS += -I$(NCSW)/inc/etc
EXTRA_CFLAGS += -I$(NCSW)/inc/Peripherals
EXTRA_CFLAGS += -I$(NCSW)/inc/flib

ifeq ("$(CONFIG_FMAN_P3040_P4080_P5020)", "y")
EXTRA_CFLAGS += -I$(NCSW)/inc/integrations/P3040_P4080_P5020
endif
ifeq ("$(CONFIG_FMAN_P1023)", "y")
EXTRA_CFLAGS += -I$(NCSW)/inc/integrations/P1023
endif
ifdef CONFIG_FMAN_T4240
EXTRA_CFLAGS += -I$(NCSW)/inc/integrations/T4240
endif

EXTRA_CFLAGS += -I$(NCSW)/src/inc
EXTRA_CFLAGS += -I$(NCSW)/src/inc/system
EXTRA_CFLAGS += -I$(NCSW)/src/inc/wrapper
EXTRA_CFLAGS += -I$(NCSW)/src/inc/xx
EXTRA_CFLAGS += -I$(srctree)/include/linux/fmd
EXTRA_CFLAGS += -I$(srctree)/include/linux/fmd/Peripherals
EXTRA_CFLAGS += -I$(srctree)/include/linux/fmd/integrations
