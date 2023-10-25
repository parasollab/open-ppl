# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/test/containers/array,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

#Grab all .cc test files
OBJS:=$(shell ls *.cc | sed 's/.cc//g')

.PHONY:compile test clean
default: compile

const_iterator_write: const_iterator_write.cc
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} -o $@.exe $< ${STAPL_LIBRARIES} ${LIB} ${LIB_EPILOGUE}2> $@ ; echo 0 1> /dev/null 2>&1

compile: $(OBJS)

test: compile
	./testme.sh "$(call staplrun,4)"

#Remove all generated files
clean:
	rm -rf *.o *.i *.err *~ *.TVD.* *.o core* a.out ii_files rii_files $(OBJS)
