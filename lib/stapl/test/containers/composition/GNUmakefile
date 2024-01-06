# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/test/containers/composition,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

LIB+=-l$(shell ls $(Boost_LIBRARY_DIRS) | grep -e boost_random | head -n 1 | sed 's/lib//g' | sed 's/\.so//g' | sed 's/\.a//g')
OBJS_SERIAL:=array graph_multilevel_composition min_row \
             mixed_multilevel_composition \
             multilevel_composition_3 multilevel_composition_4 \
             vector_multilevel_composition \
             array_redistribution multiarray_redistribution \
             vector_redistribution

.PHONY:compile compile_serial test clean
default: compile_serial

compile:

compile_serial:
	$(MAKE) -l 0.0 $(OBJS_SERIAL)


# Special Cases for composition tests

array: array.cc
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} -DNUM_COMP_LEVELS=3 -o $@ $< ${STAPL_LIBRARIES} ${LIB} ${LIB_EPILOGUE}

%_3: %.cc
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} -DNUM_COMP_LEVELS=3 -o $@ $< ${STAPL_LIBRARIES} ${LIB} ${LIB_EPILOGUE}

%_4: %.cc
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} -DNUM_COMP_LEVELS=4 -o $@ $< ${STAPL_LIBRARIES} ${LIB} ${LIB_EPILOGUE}

test: compile_serial
	./testme.sh "$(call staplrun,4)"

clean:
	rm -rf *.o *.i *.err *~ *.TVD.* *.o core* a.out ii_files rii_files $(OBJS_SERIAL)
