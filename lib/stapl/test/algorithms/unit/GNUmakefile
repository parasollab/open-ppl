# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/test/algorithms/unit,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

OBJS:= adjacent_difference find adjacent_find binary_search \
       dynamic_programming find_end find_first_of find_first_of_2 generate \
       lexicographic_compare merge min_element mismatch partition_copy \
       random_shuffle rotate_copy search search_n \
       test_is_partitioned test_map_reduce test_n_partition \
       test_partition_point test_stable_sort minmax test_column_sort

# not compiling: test_euler_tour

.PHONY:compile test clean
default: compile

compile: $(OBJS)

test: compile
	./testme.sh "$(call staplrun,1)"
	./testme.sh "$(call staplrun,2)"
	./testme.sh "$(call staplrun,3)"
	./testme.sh "$(call staplrun,4)"

clean:
	rm -rf *.o *.i *.err *~ *.TVD.* *.o core* a.out ii_files rii_files results.* $(OBJS)
