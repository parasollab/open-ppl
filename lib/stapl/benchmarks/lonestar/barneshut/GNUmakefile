# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/benchmarks/lonestar/barneshut,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

default: compile

test: all run

all: compile

compile: barnes_hut

run: all
	$(call staplrun,1) ./barnes_hut 128 2 1 5
	$(call staplrun,4) ./barnes_hut 128 2 1 5

clean:
	rm -rf *.o core* a.out barnes_hut

veryclean:
	rm -rf *.o core* a.out barnes_hut *v3breakpoints*
