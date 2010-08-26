ifeq ($(CONFIG_PHUB_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif

obj-$(CONFIG_PCH_PHUB) += pch_phub_drv.o
#to set CAN clock to 50Mhz
EXTRA_CFLAGS+=-DIOH_CAN_PCLK_50MHZ

pch_phub_drv-objs := pch_phub.o pch_phub_pci.o pch_phub_hal.o
