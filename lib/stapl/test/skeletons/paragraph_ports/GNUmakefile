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

ALLOBJS :=$(patsubst %.cc,%,$(wildcard *.cc))
OBJS_2D := $(patsubst %.cc,%_2D,$(wildcard *hom_dim.cc))
OBJS_3D := $(patsubst %.cc,%_3D,$(wildcard *hom_dim.cc))
HOMOBJS := $(patsubst %.cc,%,$(wildcard *hom_dim.cc))
OTHERS  = $(filter-out $(HOMOBJS),$(ALLOBJS))
OBJS    := $(OTHERS) $(OBJS_2D) $(OBJS_3D)

%_2D: %.cc
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} -DCUR_DIM=2 -o $@ $< ${STAPL_LIBRARIES} ${LIB} ${LIB_EPILOGUE}

%_3D: %.cc
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} -DCUR_DIM=3 -o $@ $< ${STAPL_LIBRARIES} ${LIB} ${LIB_EPILOGUE}

.PHONY:compile test clean
default: compile

compile: $(OBJS)

test: compile
	./testme.sh "$(call staplrun,1)"
	./testme.sh "$(call staplrun,2)"
	./testme.sh "$(call staplrun,3)"
	./testme.sh "$(call staplrun,4)"

clean:
	rm -rf *.o *.i *.err *~ *.TVD.* *.o core* a.out ii_files rii_files results.* *.dot $(OBJS)
