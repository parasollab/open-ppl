# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/test/algorithms,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

OBJS_SERIAL:=test_non_mutating test_numeric test_mutating test_sorting

LIB += -lboost_program_options

.PHONY:compile compile_serial test clean
default: compile_serial

compile:

compile_serial: CXXFLAGS+=-DSTAPL_PER_VIEW_LOCALIZATION
compile_serial:
	$(MAKE) -l 0.0 $(OBJS_SERIAL)

test: compile compile_serial
	./testme.sh "$(call staplrun,1)"
	./testme.sh "$(call staplrun,2)"
	./testme.sh "$(call staplrun,3)"
	./testme.sh "$(call staplrun,4)"

clean:
	rm -rf *.o *.i *.err *~ *.TVD.* *.o core* a.out ii_files rii_files $(OBJS_SERIAL)
