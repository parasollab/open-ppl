# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  export STAPL = $(shell echo "$(PWD)" | sed 's,/test/containers,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

.PHONY:compile test clean
default: compile

SUBDIRS:= sequential array graph unordered_map consistency \
          multiarray map heap unordered_set \
          composition vector list graph/generators profile/run

SUBDIRS_SERIAL:=composition graph graph/generators

#
## test related rules
#

TESTDIRS:=$(addsuffix .test, $(SUBDIRS))

.PHONY: test runtests $(TESTDIRS)

test: compile compile_serial
	$(MAKE) -l 0.0 runtests

$(TESTDIRS): %.test: compile
	$(MAKE) -C $* test

runtests: $(TESTDIRS)

#
## compile rules
#

COMPILEDIRS:=$(addsuffix .compile, $(SUBDIRS))
COMPILEDIRS_SERIAL:=$(addsuffix .compile_serial, $(SUBDIRS_SERIAL))

.PHONY: compile compile_serial $(COMPILEDIRS) $(COMPILEDIRS_SERIAL)

compile: $(COMPILEDIRS)
	$(MAKE) -l 0.0 compile_serial

$(COMPILEDIRS): %.compile:
	$(MAKE) -C $* compile

#Special target for memory-heavy compilations, so they are serialized
compile_serial:
	$(MAKE) -l 0.0 $(COMPILEDIRS_SERIAL)

$(COMPILEDIRS_SERIAL): %.compile_serial:
	$(MAKE) -C $* compile_serial

#
## clean related rules
#
SUBCLEAN:= $(addsuffix .clean,$(SUBDIRS))
.PHONY: $(SUBCLEAN)

clean: $(SUBCLEAN)

$(SUBCLEAN): %.clean:
	$(MAKE) -C $* clean
