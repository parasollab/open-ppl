# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/test/utility,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

TESTS:=$(shell ls *.cc | sed 's/.cc//g')

default: all

all: $(TESTS)

compile: $(TESTS)

test: all
	./testme.sh "$(call staplrun,2)"

clean:
	rm -rf $(TESTS) *.o *.i *.err tiny_factors*.zout sequence.new new.txt bin.txt
