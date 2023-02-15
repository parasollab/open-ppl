# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/benchmarks/lonestar/agglomerativeclustering,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

default: compile

test: all run

all: compile

compile: agglomerativeclustering

run: all
	$(call staplrun,2) ./agglomerativeclustering 100 1
	$(call staplrun,4) ./agglomerativeclustering 128 1

clean:
	rm -rf *.o core* a.out agglomerativeclustering

veryclean:
	rm -rf *.o core* a.out agglomerativeclustering *v3breakpoints*
