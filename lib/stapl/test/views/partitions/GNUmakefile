# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/test/views/partitions,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

OBJS = partition_splitter_test splitter_test

# not compiling: block_partition

.PHONY:compile test clean
default: compile

compile: $(OBJS)

test: compile
	./testme.sh "$(call staplrun,1)"
	./testme.sh "$(call staplrun,4)"

clean:
	-rm -f ${OBJS}
