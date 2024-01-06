# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

include GNUmakefile.include

#include $(STAPL)/GNUmakefile.STAPLdefaults
LFLAGS	+= -DMASS # Use all positive sources
#LFLAGS	+= -DIneJ # Different target and source points
#LFLAGS	+= -DDIRECT # Compare with direct summation
#LFLAGS	+= -DWRITE_TIME # Write timings to file
LFLAGS 	+= -DSTRONG #STRONG SCALE

default: compile

test: all
	./testme.sh "$(call staplrun,1)"
	./testme.sh "$(call staplrun,2)"
	./testme.sh "$(call staplrun,4)"

all: compile

compile: fmm fmm_define_dag

fmm: fmm.cc
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} -o $@ $< ${STAPL_LIBRARIES} ${LIB} ${LIB_EPILOGUE} ${LFLAGS}

fmm_define_dag: CXXFLAGS+=-DDO_ZIP_FUSION
fmm_define_dag: fmm_define_dag.cc
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} -o $@ $< ${STAPL_LIBRARIES} ${LIB} ${LIB_EPILOGUE} ${LFLAGS}

clean:
	find . -name "*.o" -o -name "*.out*" -o -name "*.mod" -o -name "fmm" -o -name "fmm_define_dag" | xargs rm -rf

cleandat:
	find . -name "*.dat" -o -name "*.pdb" | xargs rm -f

cleanlib:
	find . -name "*.a" -o -name "*.so" | xargs rm -f

cleanall:
	make clean
	make cleandat
	make cleanlib
