# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/benchmarks/data_mining/k_means_cluster,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

OBJS:=km_nr.exe km_seq.exe km_stl.exe km_stapl.exe

ifdef MPI_HOME
  OBJS += km_mpi.exe km_mpi_block_cyclic.exe
endif

# cannot compile: km_stapl_block_cyclic.exe

default: all

test: all

all: compile

compile: $(OBJS)


km_seq.o: km_seq.cc
	${CC} -I${STAPL} ${CXXFLAGS} km_seq.cc -c $<

km_stl.o: km_stl.cc
	${CC} -I${STAPL} ${CXXFLAGS} km_stl.cc -c $<

km_mpi.o: km_mpi.cc
	${CC} -I${STAPL} ${CXXFLAGS} km_mpi.cc -c $<

km_mpi_block_cyclic.o: km_mpi_block_cyclic.cc
	${CC} -I${STAPL} ${CXXFLAGS} km_mpi_block_cyclic.cc -c $<

km_stapl.o: km_stapl.cc
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} km_stapl.cc -c $<

km_stapl_block_cyclic.o: km_stapl_block_cyclic.cc
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} km_stapl_block_cyclic.cc -c $<


km_seq.exe: km_seq.o
	${CC} ${CXXFLAGS} km_seq.o -o km_seq.exe ${LIB} ${LIB_EPILOGUE}

km_stl.exe: km_stl.o
	${CC} ${CXXFLAGS} km_stl.o -o km_stl.exe ${LIB} ${LIB_EPILOGUE}

km_nr.exe: km_nr.o
	${CC} ${CXXFLAGS} km_nr.o -o km_nr.exe ${LIB} ${LIB_EPILOGUE}

km_mpi.exe: km_mpi.o
	${CC} ${CXXFLAGS} km_mpi.o -o km_mpi.exe ${LIB} ${LIB_EPILOGUE}

km_mpi_block_cyclic.exe: km_mpi_block_cyclic.o
	${CC} ${CXXFLAGS} km_mpi_block_cyclic.o -o km_mpi_block_cyclic.exe ${LIB} ${LIB_EPILOGUE}

km_stapl.exe: km_stapl.o
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} km_stapl.o -o km_stapl.exe ${STAPL_LIBRARIES} ${LIB} ${LIB_EPILOGUE}

km_stapl_block_cyclic.exe: km_stapl_block_cyclic.o
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} km_stapl_block_cyclic.o -o km_stapl_block_cyclic.exe ${STAPL_LIBRARIES} ${LIB} ${LIB_EPILOGUE}


clean:
	rm -f *.o *.exe

