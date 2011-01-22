# Bman
obj-$(CONFIG_FSL_BMAN_CONFIG)	+= bman_config.o
obj-$(CONFIG_FSL_BMAN_PORTAL)	+= bman_driver.o bman_high.o
obj-$(CONFIG_FSL_BMAN_TEST)	+= bman_tester.o
obj-$(CONFIG_FSL_BMAN_DEBUGFS)  += bman_debugfs_interface.o
bman_tester-y			 = bman_test.o
bman_tester-$(CONFIG_FSL_BMAN_TEST_HIGH) += bman_test_high.o
bman_tester-$(CONFIG_FSL_BMAN_TEST_THRESH) += bman_test_thresh.o
bman_debugfs_interface-y	 = bman_debugfs.o

# Qman
obj-$(CONFIG_FSL_QMAN)		+= qman_utility.o
obj-$(CONFIG_FSL_QMAN_CONFIG)	+= qman_config.o
obj-$(CONFIG_FSL_QMAN_PORTAL)	+= qman_driver.o qman_high.o
obj-$(CONFIG_FSL_QMAN_FQALLOCATOR) += qman_fqalloc.o
obj-$(CONFIG_FSL_QMAN_TEST)	+= qman_tester.o
qman_tester-y			 = qman_test.o qman_test_hotpotato.o \
					qman_test_high.o
qman_tester-$(CONFIG_FSL_QMAN_TEST_ERRATA) += qman_test_errata.o
obj-$(CONFIG_FSL_QMAN_DEBUGFS) += qman_debugfs_interface.o
qman_debugfs_interface-y	= qman_debugfs.o

# Common
ifeq ($(CONFIG_FSL_BMAN_PORTAL),y)
obj-y				+= dpa_uio.o
else
ifeq ($(CONFIG_FSL_QMAN_PORTAL),y)
obj-y				+= dpa_uio.o
endif
endif
