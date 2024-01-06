# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/test/views,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

OBJS:= test_counting test_domset1D test_domain_view \
       test_multiarray_view_over_array_view test_map_proxy test_pair_proxy \
       test_overlap_view test_partitioned_domain test_partitioned_view \
       test_periodic_boundary_view test_repeat_view test_stl_view \
       test_strided_view test_vector_proxy test_zip_view \
       test_extended_view test_lazy_insert_view \
       segmented_view_metadata test_set_proxy \
       test_index_view test_proxy_conversion test_segmented test_as_reference \
       test_transform_view

ifdef TBBROOT
  OBJS+=test_tbb_range_adaptor
  CXXFLAGS+=-I${TBBROOT}/include -fno-inline
  LIB+=-L${TBBROOT}/lib -ltbb -ltbbmalloc
endif

.PHONY:compile test clean
default: compile

compile: $(OBJS)

test: compile
	$(call staplrun,1) ./test_stl_view
	./testme.sh "$(call staplrun,1)"
	./testme.sh "$(call staplrun,4)"

clean:
	rm -rf *.o *.i *.err *~ *.TVD.* *.o core* a.out ii_files rii_files $(OBJS)
