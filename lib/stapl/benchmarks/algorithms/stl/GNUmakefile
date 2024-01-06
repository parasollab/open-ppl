# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
   export STAPL = $(shell echo "$(PWD)" | sed 's,/benchmarks/algorithms/stl,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

default: compile

all: compile

test: all
	$(MAKE) -l 0.0 runtests

runtests:
	./numeric 262144
	./non_mutating 262144
	./mutating 262144
	./sort 262144

compile: numeric non_mutating mutating sort

%: %.cc
	$(CC) $< -O3 -std=c++11 -o $@

clean:
	rm -f numeric non_mutating mutating sort

