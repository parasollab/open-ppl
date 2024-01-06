# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  export STAPL = $(shell echo "$(PWD)" | sed 's,/benchmarks/lonestar/delaunay,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

SUBDIRS=triangulation

COMPILEDIRS := $(addsuffix .compile, $(SUBDIRS))
TESTDIRS    := $(addsuffix .test, $(SUBDIRS))
CLEANDIRS   := $(addsuffix .clean, $(SUBDIRS))

.PHONY: all compile test clean $(COMPILEDIRS) $(TESTDIRS) $(CLEANDIRS)

default: all

all: compile

# Rules for compiling
compile: $(COMPILEDIRS)
$(COMPILEDIRS): %.compile:
	$(MAKE) -C $* compile

# Rules for testing
test: $(TESTDIRS)
$(TESTDIRS): %.test:
	$(MAKE) -C $* test

# Rules for cleaning
clean: $(CLEANDIRS)
$(CLEANDIRS): %.clean:
	$(MAKE) -C $* clean
