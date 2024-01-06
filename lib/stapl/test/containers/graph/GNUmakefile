# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/test/containers/graph,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

#Grab all .cc test files except conditional ones
COND_OBJS:=multilevel_partitioner test_arbitrary_mesh_decomposition_2D test_arbitrary_mesh_decomposition_3D
COND_OBJS_BIS:=$(shell echo $(COND_OBJS) | sed 's/ /|/g')
OBJS:=$(shell ls *.cc | grep -v test_algo_perf | sed 's/\.cc//g' | perl -pe "s/$(COND_OBJS_BIS)//g")
PARALLEL_OBJS:=bfs_kla_perf breadth_first_search connected_components  csr \
	cut_conductance graph_sharder_test graph_test \
	k_core_dynamic mssp page_rank sssp  \
	test_implicit_regular_mesh  topological_sort \
	view_based_dynamic_graph

PSCC_OBJS:=pscc test_pscc_kla time_pscc time_pscc2 time_pscc_kla

ifdef METIS_ROOT
  OBJS+=multilevel_partitioner
  CXXFLAGS+=-I${METIS_ROOT}/include
  LIB+=-L${METIS_ROOT}/lib -lmetis
endif

ifdef HDF5_ROOT
ifdef SILO_ROOT
  OBJS+=test_arbitrary_mesh_decomposition_2D test_arbitrary_mesh_decomposition_3D
  CXXFLAGS+=-I${HDF5_ROOT}/include -I${SILO_ROOT}/include
  LIB+=-L${HDF5_ROOT}/lib -L${SILO_ROOT}/lib -lsiloh5 -lhdf5 -lz
endif
endif

.PHONY:compile compile_serial test clean compile_pscc clean_pscc
default: compile compile_serial

compile: $(PARALLEL_OBJS)

compile_serial:
	$(MAKE) -l 0.0 $(OBJS) test_algo_perf

compile_pscc: $(PSCC_OBJS)

clean_pscc:
	rm -rf $(PSCC_OBJS)

time_pscc_kla: LIB+=-L$(STAPL)/benchmarks/graph/g500/generator -lgraph_generator_mpi
time_pscc_kla:
	cd $(STAPL)/benchmarks/graph/g500/generator && $(MAKE) libgraph_generator_mpi.a
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} -o $@ $@.cc ${STAPL_LIBRARIES} ${LIB} ${LIB_EPILOGUE}

test_algo_perf: test_algo_perf_part2.o test_algo_perf_part3.o test_algo_perf.o
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} -o $@ $@.o test_algo_perf_part2.o test_algo_perf_part3.o ${STAPL_LIBRARIES} ${LIB} ${LIB_EPILOGUE}

test: compile compile_serial
	./testme.sh "$(call staplrun,4)"

clean:
	rm -rf *.o *.i *.err *~ *.TVD.* *.o core* a.out ii_files rii_files graph.out $(OBJS) $(COND_OBJS) $(PARALLEL_OBJS) test_algo_perf
