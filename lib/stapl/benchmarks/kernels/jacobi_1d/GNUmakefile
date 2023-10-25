# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/benchmarks/kernels/jacobi_1d,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

OBJS=jacobi_1d_armi jacobi_1d_armi_zero_copy

ifdef MPI_HOME
  OBJS+=jacobi_1d_mpi_omp_naive jacobi_1d_mpi_omp
endif

default: compile

test: all

all: compile

compile: ${OBJS}

clean:
	rm -rf *.o core* a.out ${OBJS}

veryclean:
	rm -rf *.o core* a.out ${OBJS} *v3breakpoints*
