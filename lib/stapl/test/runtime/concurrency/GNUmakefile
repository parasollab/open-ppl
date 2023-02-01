# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/test/runtime/concurrency,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

OBJS=$(shell ls *.cc | sed 's/.cc//g')
LIB+=-l$(shell ls $(Boost_LIBRARY_DIRS) | grep -e boost_unit_test_framework | head -n 1 | sed 's/lib//g' | sed 's/\.so//g' | sed 's/\.a//g')

.PHONY:compile test clean
default: compile

compile: $(OBJS)

test: compile
	./testme.sh

%: %.cc
	$(CC) $(STAPL_CXXFLAGS) $(CXXFLAGS) -o $@ $< $(LIB) $(LIB_EPILOGUE)

clean:
	rm -rf *.o core* a.out ii_files rii_files $(OBJS)
