# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/test/skeletons.*,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

STAPL_INCLUDE=-I$(STAPL)

CXXFLAGS += -DVALIDATE_RESULTS
CXXFLAGS += -Wno-missing-braces

OBJS:=$(patsubst %.cc,%,$(wildcard *.cc)) stl_stencil_2d_2p

stl_stencil: stl_stencil.cc
	${CC} ${STAPL_INCLUDE} ${CXXFLAGS} -o $@ $< 
stl_stencil_2d_2p: CXXFLAGS+=-D N_MACRO=2 -D P_MACRO=2
stl_stencil_2d_2p: stl_stencil.cc
	${CC} ${STAPL_INCLUDE} ${CXXFLAGS} -o $@ $< 

.PHONY:compile test clean
default: compile

compile: $(OBJS)

test: compile
	./testme.sh

clean:
	rm -rf *.o *.i *.err *~ *.TVD.* *.o core* a.out ii_files rii_files results.* *.dot $(OBJS)
