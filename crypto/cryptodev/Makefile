# Since version 1.6 the asynchronous mode has been
# disabled by default. To re-enable it uncomment the
# corresponding CFLAG.
#
CRYPTODEV_CFLAGS ?= #-DENABLE_ASYNC
KBUILD_CFLAGS += $(CRYPTODEV_CFLAGS)

obj-$(CONFIG_CRYPTODEV) += cryptodev.o
cryptodev-objs += ioctl.o main.o cryptlib.o authenc.o zc.o util.o