# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/benchmarks/sweep,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

OBJS=$(shell ls *.cc | sed 's/.cc//g')

default: compile

test: all run

all: compile

compile: $(OBJS)

run: all
	$(call staplrun,1) ./sweep 4 3
	$(call staplrun,4) ./sweep 4 3

clean:
	rm -rf *.o core* a.out $(OBJS)

