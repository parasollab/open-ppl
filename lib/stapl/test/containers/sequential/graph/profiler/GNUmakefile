# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.


ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/test/containers/sequential/graph/profiler,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

OBJS:=profiler_graph

# not compiling: profiler_graph_algos

.PHONY:compile test clean

default: compile

compile: $(OBJS)

#Special case compilation
%: %.cc
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} -o $@ $< ${STAPL_LIBRARIES_ONLY_RTS} ${LIB} ${LIB_EPILOGUE}

test: compile

clean:
	rm -rf *.o *.i *.err *~ *.TVD.* *.o core* a.out ii_files rii_files $(OBJS)
