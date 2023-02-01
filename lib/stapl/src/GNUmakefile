# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  export STAPL = $(shell echo "$(PWD)" | sed 's,/src,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

default: build

build: compile
	mkdir -p $(STAPL)/lib
	$(AR) $(STAPL)/lib/libstapl_rt.a runtime/*.o
	$(AR) $(STAPL)/lib/libstapl.a paragraph/*.o runtime/*.o containers/*.o views/*.o skeletons/*.o containers/graph/algorithms/*.o

#
#compile rules
#
SUBDIRS:= paragraph runtime containers views skeletons containers/graph/algorithms
.PHONY: compile $(SUBDIRS)

compile: $(SUBDIRS)

$(SUBDIRS):
	$(MAKE) -C $@

#
#clean rules
#
SUBCLEAN:= $(addsuffix .clean,$(SUBDIRS))
.PHONY: clean $(SUBCLEAN)

clean: $(SUBCLEAN)
	rm -f $(STAPL)/lib/*

$(SUBCLEAN): %.clean:
	$(MAKE) -C $* clean
