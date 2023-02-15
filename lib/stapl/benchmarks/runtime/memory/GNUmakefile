# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/benchmarks/runtime/memory,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

OBJS=$(shell ls *.cc | sed 's/.cc//g')

default: compile

test: all

all: compile

compile: $(OBJS)

clean:
	rm -rf *.o core* a.out ii_files rii_files $(OBJS)

memory_footprint_native: memory_footprint_native.cc
	${CC} ${CXXFLAGS} -o $@ $< ${LIB}
