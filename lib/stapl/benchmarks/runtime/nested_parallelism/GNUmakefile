# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/benchmarks/runtime/nested_parallelism,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

OBJS=bench_execute_levels
LIB+=-l$(shell ls $(Boost_LIBRARY_DIRS) | grep -m 1 -e boost_program_options | sed 's/lib//g' | sed 's/\.so//g' | sed 's/\.a//g')

ifdef STAPL_OPENMP_AVAILABLE
  OBJS+=bench_execute
endif

default: compile

test: all

all: compile

compile: $(OBJS)

clean:
	rm -rf *.o core* a.out ii_files rii_files $(OBJS)
