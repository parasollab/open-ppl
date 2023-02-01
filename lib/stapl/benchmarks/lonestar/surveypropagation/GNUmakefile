# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/benchmarks/lonestar/surveypropagation,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

default: compile

test: all run

all: compile

compile:  surveypropagation

run: all
	$(call staplrun,1) ./surveypropagation 30000 10000 3 1429
	$(call staplrun,2) ./surveypropagation 30000 10000 3 1429
	$(call staplrun,4) ./surveypropagation 30000 10000 3 1429

clean:
	rm -rf *.o core* a.out surveypropagation

veryclean:
	rm -rf *.o core* a.out surveypropagation *v3breakpoints*
