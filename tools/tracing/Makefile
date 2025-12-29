# SPDX-License-Identifier: GPL-2.0
include ../scripts/Makefile.include

all: all_tools

all_tools: libtracefs_install
	$(call descend,latency)
	$(call descend,rtla)

clean: latency_clean rtla_clean libtracefs_clean libtraceevent_clean

install: latency_install rtla_install

libtraceevent:
	@echo "---------start libtraceevent-----------"
	$(call descend,libtraceevent)

libtraceevent_install: libtraceevent
	@echo "---------start libtraceevent_install-----------"
	$(call descend,libtraceevent,install)

libtraceevent_clean:
	$(call descend,libtraceevent,clean)

libtracefs: libtraceevent_install
	@echo "---------start libtracefs-----------"
	$(call descend,libtracefs)

libtracefs_install: libtracefs
	@echo "---------start libtracefs_install-----------"
	$(call descend,libtracefs,install)

libtracefs_clean:
	$(call descend,libtracefs,clean)

latency:
	$(call descend,latency)

latency_install:
	$(call descend,latency,install)

latency_clean:
	$(call descend,latency,clean)

rtla:
	$(call descend,rtla)

rtla_install:
	$(call descend,rtla,install)

rtla_clean:
	$(call descend,rtla,clean)

.PHONY: all install clean latency latency_install latency_clean \
	rtla rtla_install rtla_clean libtracefs_clean libtraceevent_clean
