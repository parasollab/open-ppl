# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/benchmarks/lonestar/max_cardinality,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

default: compile

test: all

all: compile

compile: maximal_bipartite_matching

clean:
	rm -rf *.o core* a.out maximal_bipartite_matching

veryclean:
	rm -rf *.o core* a.out maximal_bipartite_matching *v3breakpoints*
