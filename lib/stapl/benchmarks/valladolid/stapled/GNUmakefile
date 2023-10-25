# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/benchmarks/valladolid/stapled,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

OBJS=$(shell ls *.cc | sed 's/.cc//g')

default: compile_serial

test: all

all: compile_serial

compile_serial: CXXFLAGS+=-DSTAPL_PER_VIEW_LOCALIZATION
compile_serial:
	$(MAKE) -l 0.0 $(OBJS)

clean:
	rm -rf *.o core* a.out ii_files rii_files $(OBJS)
