# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  export STAPL = $(shell echo "$(PWD)" | sed 's,/test/skeletons,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

SUBDIRS=sequential paragraph_ports
TESTDIRS:=$(addsuffix .test, $(SUBDIRS))
COMPILEDIRS:=$(addsuffix .compile, $(SUBDIRS))
SUBCLEAN:= $(addsuffix .clean,$(SUBDIRS))

#CXXFLAGS += -DSKELETONS_FULL_NODE_NAME
#CXXFLAGS += -DSHOW_RESULTS
#CXXFLAGS += -DGRAPHVIZ_OUTPUT
CXXFLAGS += -DVALIDATE_RESULTS
#CXXFLAGS += -ftime-report

LIB+=-lboost_program_options

OBJS:=$(patsubst %.cc,%,$(wildcard *.cc))

.PHONY:compile test clean
default: compile

compile: $(OBJS) $(COMPILEDIRS)

test_inline_cycle_check: test_inline_cycle_check.cc
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} -o $@.exe $< ${STAPL_LIBRARIES} ${LIB} ${LIB_EPILOGUE}2> $@ ; true

$(TESTDIRS): %.test: compile
	$(MAKE) -C $* test

$(COMPILEDIRS): %.compile:
	$(MAKE) -C $* compile

test: compile $(TESTDIRS)
	./testme.sh "$(call staplrun,1)"
	./testme.sh "$(call staplrun,2)"
	./testme.sh "$(call staplrun,3)"
	./testme.sh "$(call staplrun,4)"

clean: $(SUBCLEAN)
	rm -rf *.o *.i *.err *~ *.TVD.* *.o core* a.out ii_files rii_files results.* *.dot $(OBJS)

$(SUBCLEAN): %.clean:
	$(MAKE) -C $* clean
