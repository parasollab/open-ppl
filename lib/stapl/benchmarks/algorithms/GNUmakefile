# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
   export STAPL = $(shell echo "$(PWD)" | sed 's,/benchmarks/algorithms,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

SUBDIRS=stl parallel_stl stapl

default: compile

all: compile

#
## test related rules
#

TESTDIRS:=$(addsuffix .test, $(SUBDIRS))

.PHONY: test runtests $(TESTDIRS)

test: all
	$(MAKE) -l 0.0 runtests

$(TESTDIRS): %.test: compile
	$(MAKE) -C $* test

runtests: $(TESTDIRS)

#
## compile rules
#

COMPILEDIRS:=$(addsuffix .compile, $(SUBDIRS))
.PHONY: compile

compile: $(COMPILEDIRS)

$(COMPILEDIRS): %.compile:
	$(MAKE) -C $* compile

#
## clean related rules
#
SUBCLEAN:= $(addsuffix .clean,$(SUBDIRS))
.PHONY: $(SUBCLEAN)

clean: $(SUBCLEAN)

$(SUBCLEAN): %.clean:
	$(MAKE) -C $* clean
