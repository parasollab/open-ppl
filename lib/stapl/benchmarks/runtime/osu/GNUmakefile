# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/benchmarks/runtime/osu,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

OBJS=async_rmi_acc_latency async_rmi_bw async_rmi_latency async_rmi_multi_lat \
     sync_rmi_acc_latency sync_rmi_bw sync_rmi_cas_latency \
       sync_rmi_fop_latency sync_rmi_latency \
     collective one_sided_collective sync \
     serialization_latency
LIB+=-l$(shell ls $(Boost_LIBRARY_DIRS) | grep -e boost_program_options | head -n 1 | sed 's/lib//g' | sed 's/\.so//g' | sed 's/\.a//g')

ifdef BOOST_MPI_LIBRARY_DIR
  BOOST_MPI_LIB=-L$(BOOST_MPI_LIBRARY_DIR) -l$(shell ls $(BOOST_MPI_LIBRARY_DIR) | grep -e boost_mpi | head -n 1 | sed 's/lib//g' | sed 's/\.so//g' | sed 's/\.a//g')
  OBJS+=boost_mpi_latency
endif

default: compile

test: all

all: compile

compile: $(OBJS)

clean:
	rm -rf *.o core* a.out ii_files rii_files $(OBJS)

boost_%: boost_%.cc
	${CC} ${CXXFLAGS} -DNDEBUG -o $@ $< ${BOOST_MPI_LIB} ${LIB} ${LIB_EPILOGUE}
