# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
   export STAPL = $(shell echo "$(PWD)" | sed 's,/benchmarks/algorithms/stapl,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

default: compile

all: compile

test: all
	$(MAKE) -l 0.0 runtests

runtests:
	$(call staplrun,1) ./basic
	$(call staplrun,1) ./numeric 262144
	$(call staplrun,1) ./non_mutating 262144
	$(call staplrun,1) ./sort 262144
	$(call staplrun,1) ./sort_define_dag 262144
	$(call staplrun,4) ./numeric 262144
	$(call staplrun,4) ./non_mutating 262144
	$(call staplrun,4) ./sort 262144
	$(call staplrun,4) ./sort_define_dag 262144

compile: numeric non_mutating mutating sort basic sort_define_dag

sort_define_dag: CXXFLAGS+=-DUSE_NEW_NOTATION
sort_define_dag: sort.cc
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} -o $@ $< ${STAPL_LIBRARIES} ${LIB} ${LIB_EPILOGUE}

clean:
	rm -f basic numeric non_mutating mutating sort sort_define_dag *TVD*

