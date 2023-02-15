# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/test/containers/graph/generators,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

#Grab all .cc test files
OBJS:=$(shell ls *.cc | sed 's/\.cc//g')

.PHONY:compile compile_serial test clean
default: compile_serial

compile:

compile_serial:
	$(MAKE) -l 0.0 $(OBJS)

test: compile_serial
	./testme.sh "$(call staplrun,4)"

clean:
	rm -rf *.o *.i *.err *~ *.TVD.* *.o core* a.out ii_files rii_files $(OBJS) *.out
