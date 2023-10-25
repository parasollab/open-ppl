# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/benchmarks/uts,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

COMMON_SRCS = uts.c
STATS_SRC = stats.c

# ------------------------------------- #
# Set Random Number Generator sources:
# ------------------------------------- #

# Set the default RNG
ifndef RNG
  RNG=BRG
endif

ifeq ($(RNG), Devine)
  RNG_SRC = rng/devine_sha1.c
  RNG_INCL= rng/devine_sha1.h
  RNG_DEF = -DDEVINE_RNG
endif

ifeq ($(RNG), BRG)
  RNG_SRC = rng/brg_sha1.c
  RNG_INCL= rng/brg_sha1.h
  RNG_DEF = -DBRG_RNG
endif

ifeq ($(RNG), ALFG)
  RNG_SRC = rng/alfg.c
  RNG_INCL= rng/alfg.h
  RNG_DEF = -DUTS_ALFG
endif

default: compile

test: all run

all: compile

CXXFLAGS += -Wno-deprecated
compile: stapl_workstealing

run: all
	@./testme.sh "$(call staplrun,1)"
	@./testme.sh "$(call staplrun,4)"

clean:
	rm -rf *.o core* stapl_workstealing

veryclean:
	rm -rf *.o core* stapl_workstealing *v3breakpoints*

stapl_workstealing: stapl_workstealing.cc uts_stapl.cc ${RNG_SRC} ${COMMON_SRCS} ${STATS_SRC}
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} ${RNG_DEF} -o $@ $+ ${STAPL_LIBRARIES} ${LIB} ${LIB_EPILOGUE}
