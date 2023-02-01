# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/test/containers/profile/run,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

#OBJS:= profiler_hash_map profiler_p_array_algo profiler_p_vector_algo \
#		profiler_p_list_algo profiler_p_list profiler_p_container_composition \
#		profiler_assoc profiler_p_multiarray_array

OBJS:= test_view_composition_builder \
	profiler_array profiler_vector profiler_graph \
	profiler_map profiler_map_stl profiler_unordered_map \
	profiler_unordered_map_stl

.PHONY: compile test clean
default: compile

compile: $(OBJS)

test: compile
	./testme.sh "$(call staplrun,4)"
	./testme.sh "$(call staplrun,1)"

clean:
	rm -rf *.o *.i *.err *~ *.TVD.* *.o core* a.out ii_files rii_files profile_log
	rm -rf $(OBJS)
