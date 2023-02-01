# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
 STAPL = $(shell echo "$(PWD)" | sed 's,/test/containers/sequential/graph,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

OBJS:=test_graph test_graph_algo test_scc test_astar test_iter_adaptors

# not compiling: test_bfs test_bgl_compare test_dijkstra test_graph_algo2
#                test_graph_view test_mst

.PHONY:compile test clean
default: compile

compile: $(OBJS)

#Special case compilation
%: %.cc
	${CC} ${STAPL_CXXFLAGS} -I../ ${CXXFLAGS} -o $@ $< ${LIB} ${LIB_EPILOGUE}

test: compile
	./test_graph 1234
	./test_graph_algo 400
	./test_scc 1236
	./test_astar 70
	./test_iter_adaptors

clean:
	rm -rf *.o *.i *.err *~ *.TVD.* *.o core* a.out ii_files rii_files $(OBJS) mp.graph vertices_edges.graph
