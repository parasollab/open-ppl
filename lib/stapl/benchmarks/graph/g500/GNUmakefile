# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/benchmarks/graph/g500,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

OBJS=$(shell ls *.cc | sed 's/.cc//g')
LIB+=-L./generator -lgraph_generator_mpi

default: compile

test: all run

all: compile

compile: $(OBJS)

run: all
	$(call staplrun,1) ./g500_bench
	$(call staplrun,4) ./g500_bench

clean:
	rm -rf *.o core* a.out $(OBJS)
	cd generator; make clean
