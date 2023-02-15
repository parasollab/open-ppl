# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/benchmarks/containers/multiarray,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

OBJS=multiarray_test
LIB+=-l$(shell ls $(Boost_LIBRARY_DIRS) | grep -m 1 -e boost_program_options | sed 's/lib//g' | sed 's/\.so//g' | sed 's/\.a//g')

ifdef KOKKOS_ROOT
  LIB+=-L$(KOKKOS_ROOT)/lib -lkokkos
endif

default: compile

test: compile
	$(MAKE) -l 0.0 run

all: test

CXXFLAGS+=-I$(STAPL)/benchmarks/kripke/reference/src -I$(STAPL)/test -Wno-switch

ifdef KOKKOS_ROOT
  CXXFLAGS+=-I$(KOKKOS_ROOT)/include -DKOKKOS_DEFINED
endif

compile: $(OBJS)

run:
	$(call staplrun,1) ./multiarray_test DGZ 12 12 12 96 64 5 12 8
	$(call staplrun,1) ./multiarray_test DZG 12 12 12 96 64 5 12 8
	$(call staplrun,1) ./multiarray_test GDZ 12 12 12 96 64 5 12 8
	$(call staplrun,1) ./multiarray_test GZD 12 12 12 96 64 5 12 8
	$(call staplrun,1) ./multiarray_test ZDG 12 12 12 96 64 5 12 8
	$(call staplrun,1) ./multiarray_test ZGD 12 12 12 96 64 5 12 8

clean:
	rm -rf *.o core* a.out ii_files rii_files $(OBJS)
