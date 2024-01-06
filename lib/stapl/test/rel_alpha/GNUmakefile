# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/test/rel_alpha,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

OBJS_SERIAL:= helloworld datapar1 datapar1v datapar3 dataparn dataparset \
	multiview_ma multiview redistrib redistrib_ma \
	nestpar1 nestpar2 nestpar3_homogeneous nestpar3_heterogeneous \
	nestpar4 nestpar6 nestpar7 nestpar_set nestpar_graph_bfs perf_nestpar_set \
	nestpar_3g_map nestpar_3g_graph

# TODO: return nestpar_graph to OBJS_SERIAL when memory usage of graph test
# compilation has been reduced.  See gForge Bug 1313.

# The warnings for unused variables and return values are to be reenabled as the
# tests are completed.
CXXFLAGS += tiny_data.cc fixed_data.cc util.cc -Wno-unused -Wno-return-type
LIB += -lboost_program_options

.PHONY:compile compile_serial test clean
default: compile_serial

compile:

compile_serial:
	$(MAKE) -l 0.0 $(OBJS_SERIAL)

test: compile compile_serial
	echo "rel_alpha tests are compile-only at this point."

nestpar_3g_map:
	/usr/bin/time ${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} -DFILL_MAP \
		-o nestpar_3g_map nestpar3g.cc ${STAPL_LIBRARIES} \
		${LIB} ${LIB_EPILOGUE}

nestpar_3g_graph:
	/usr/bin/time ${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} -DFILL_GRAPH \
		-o nestpar_3g_graph nestpar3g.cc ${STAPL_LIBRARIES} \
		${LIB} ${LIB_EPILOGUE}

clean:
	rm -rf *.o *.i *.err *~ *.TVD.* *.o core* a.out ii_files rii_files \
		$(OBJS_SERIAL) nestpar3
