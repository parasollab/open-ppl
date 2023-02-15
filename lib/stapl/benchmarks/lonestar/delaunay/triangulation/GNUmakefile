# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/benchmarks/lonestar/delaunay/triangulation,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults


HEADERS = $(wildcard *.hpp ../*.hpp)

default: compile

test: all run

all: compile

compile: delaunay_triangulation

delaunay_triangulation: delaunay_triangulation.cc ${HEADERS}

run: all
	$(call staplrun,1) ./delaunay_triangulation --num_points 50000
	$(call staplrun,4) ./delaunay_triangulation --num_points 50000

clean:
	rm -rf *.o core* a.out delaunay_triangulation

veryclean:
	rm -rf *.o core* a.out delaunay_triangulation *v3breakpoints*
