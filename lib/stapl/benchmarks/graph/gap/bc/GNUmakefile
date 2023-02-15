# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/benchmarks/graph/gap/bc,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

LIB+=-L$(STAPL)/benchmarks/graph/g500/generator -lgraph_generator_mpi -lboost_program_options

default: compile

test: all run

all: compile

compile: bc

bc: bc.cc
	cd $(STAPL)/benchmarks/graph/g500/generator && $(MAKE) libgraph_generator_mpi.a
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} -o $@ $< ${STAPL_LIBRARIES} ${LIB} ${LIB_EPILOGUE}

run: all
	$(call staplrun,4) ./bc --paradigm hier --num_sources 10 --trials 10 --type er --n 100 --prob 0.05

clean:
	rm -rf *.o core* a.out bc
