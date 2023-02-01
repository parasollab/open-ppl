# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

##############################################################################
# See GNUmakefile.STAPL_README
###############################################################################

ifndef STAPL
  export STAPL=$(PWD)
endif

include GNUmakefile.STAPLdefaults

.PHONY: base compile test clean
default: base

all: base test

BASEDIRS:= src

#
# base builds precompiled version of stapl distro
#
base:
	$(MAKE) -C src

SUBDIRS:= examples \
          docs/tutorial_guide \
          test \
          benchmarks

SUBDIRS_SERIAL:=test

TESTDIRS:=$(addsuffix .test, $(SUBDIRS))

.PHONY: $(SUBDIRS) $(TESTDIRS)

#
# Compile, then run, all tests
#

.PHONY: test runtests

test: export RUNNING_VALIDATION=1
test: base compile
	$(MAKE) -l 0.0 runtests

runtests: $(TESTDIRS)

#Force sequential test execution
$(TESTDIRS): %.test: base compile
	$(MAKE) -C $* test

#
# Compile all tests, but don't run
#

COMPILEDIRS:=$(addsuffix .compile, $(SUBDIRS))
COMPILEDIRS_SERIAL:=$(addsuffix .compile_serial, $(SUBDIRS_SERIAL))

.PHONY: compile compile_serial $(COMPILEDIRS) $(COMPILEDIRS_SERIAL)

compile: base $(COMPILEDIRS)
	$(MAKE) -l 0.0 compile_serial

$(COMPILEDIRS): base
$(COMPILEDIRS): %.compile:
	$(MAKE) -C $* compile

#Special target for memory-heavy compilations, so they are serialized
compile_serial: $(COMPILEDIRS_SERIAL)

$(COMPILEDIRS_SERIAL): base
$(COMPILEDIRS_SERIAL): %.compile_serial:
	$(MAKE) -C $* compile_serial

#
# clean related rules
#
CLEANDIRS = $(BASEDIRS) $(SUBDIRS)
SUBCLEAN  = $(addsuffix .clean,$(CLEANDIRS))
.PHONY: clean $(SUBCLEAN)

clean: $(SUBCLEAN)
	@rm -f $(THISBUILD)  # forget current platform/rts/stl

mostlyclean: $(SUBCLEAN)

$(SUBCLEAN): %.clean:
	$(MAKE) -C $* clean


showconf:
	@echo "platform        = $(platform)"
	@echo "stl             = $(stl)"
	@echo "CC              = $(CC)"
	@echo "ld              = $(LD)"
	@echo "ar              = $(AR)"
	@echo "USER_CXXFLAGS   = $(USER_CXXFLAGS)"
	@echo "STAPL_CXXFLAGS  = $(STAPL_CXXFLAGS)"
	@echo "AUTOFLAGS       = $(AUTOFLAGS)"
	@echo "CXXFLAGS        = $(CXXFLAGS)"
	@echo "USER_LIB        = $(USER_LIB)"
	@echo "STAPL_LIBRARIES = $(STAPL_LIBRARIES)"
	@echo "AUTOLIB         = $(AUTOLIB)"
	@echo "LIB             = $(LIB)"
	@echo "LIB_EPILOGUE    = $(LIB_EPILOGUE)"
	@echo "staplrun        = $(staplrun)"
	@echo "ALL_FLAGS       = $(STAPL_CXXFLAGS) $(CXXFLAGS)"
	@echo "ALL_LIBS        = $(STAPL_LIBRARIES) $(LIB) $(LIB_EPILOGUE)"
