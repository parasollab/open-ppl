# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/test/runtime/gang,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

OBJS=$(shell ls *.cc | sed 's/.cc//g')

.PHONY:compile test clean
default: compile

compile: CXXFLAGS+=-D_TEST_QUIET
compile: $(OBJS)

test: compile
	./testme.sh "$(call staplrun,1)"
	./testme.sh "$(call staplrun,4)"

clean:
	rm -rf *.o core* a.out ii_files rii_files $(OBJS)

test-debug:
test-debug: $(OBJS)
	./testme.sh "$(call staplrun,1)"
	./testme.sh "$(call staplrun,4)"
	./testme.sh "$(call staplrun,5)"
	./testme.sh "$(call staplrun,8)"
