CC              = $(CROSS_COMPILE)gcc
BUILD_OUTPUT	:= $(PWD)
PREFIX		:= /usr
DESTDIR		:=

turbostat : turbostat.c

%: %.c
	@mkdir -p $(BUILD_OUTPUT)
	$(CC) $(CFLAGS) $< -o $(BUILD_OUTPUT)/$@

clean :
	@rm -f $(BUILD_OUTPUT)/turbostat

install :
	install $(BUILD_OUTPUT)/turbostat $(DESTDIR)$(PREFIX)/bin/turbostat
	install turbostat.8 $(DESTDIR)$(PREFIX)/share/man/man8
