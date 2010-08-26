ifeq ($(CONFIG_PCH_GBE_DEBUG_CORE),y)
EXTRA_CFLAGS += -DDEBUG
endif

obj-$(CONFIG_PCH_GBE) += pch_gbe.o
pch_gbe-objs := pch_gbe_mac.o pch_gbe_phy.o pch_gbe_nvm.o pch_gbe_ethtool.o \
	pch_gbe_plat.o pch_gbe_param.o pch_gbe_api.o pch_gbe_main.o
