# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/benchmarks/kripke/stapled,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

OBJS:=driver.o Grid.o BaseGrid.o Cell.o GroupDirSet.o
KRIPKE_OBJS:=Kripke/Sweep_Solver.o Kripke/User_Data.o Kripke/Directions.o \
             Kripke/Timing.o Kripke/Kernel.o Kripke/Kernel/Kernel_3d_ZDG.o \
             Kripke/Kernel/Kernel_3d_DGZ.o Kripke/Kernel/Kernel_3d_GDZ.o \
						 tools/testKernels.o

default: kripke

compile: $(OBJS)
	$(MAKE) -C tools
	$(MAKE) -C Kripke

CXXFLAGS += -I$(PWD) -Wno-format-security -Wno-switch
kripke: compile
	${CC} ${CXXFLAGS} ${OBJS} ${KRIPKE_OBJS} -o kripke \
		${STAPL_LIBRARIES} ${LIB} ${LIB_EPILOGUE}

clean:
	$(MAKE) -C Kripke clean
	rm -f *.o kripke

