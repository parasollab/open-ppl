# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  export STAPL = $(shell echo "$(PWD)" | sed 's,/test,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

.PHONY:compile test clean
default: compile

SUBDIRS:= runtime/type_traits runtime/concurrency runtime/utility \
          runtime/serialization runtime/system runtime runtime/gang \
          runtime/executor runtime/interoperability \
          containers/sequential containers/array containers/graph \
          containers/unordered_map containers/consistency \
          containers/multiarray containers/map \
          containers/set containers/heap containers/composition \
          containers/vector containers/list containers/graph/generators \
          containers/unordered_set containers/profile/run \
          views views/partitions views/type_traits views/metadata \
          views/metadata/projection views/metadata/extraction \
          views/slice views/slice/test_composed_slices_full_paragraph \
          algorithms algorithms/unit algorithms/sequential \
          skeletons rel_alpha \
          paragraph \
          profiler utility utility/tuple

SUBDIRS_SERIAL:= containers/composition containers/graph \
                 containers/graph/generators \
                 algorithms \
                 rel_alpha

# not compiling: views/associated

#
## test related rules
#

TESTDIRS:=$(addsuffix .test, $(SUBDIRS))

.PHONY: test runtests $(TESTDIRS)

test: compile compile_serial
	$(MAKE) -l 0.0 runtests

$(TESTDIRS): %.test: compile
	$(MAKE) -C $* test

runtests: $(TESTDIRS)

#
## compile rules
#

COMPILEDIRS:=$(addsuffix .compile, $(SUBDIRS))
COMPILEDIRS_SERIAL:=$(addsuffix .compile_serial, $(SUBDIRS_SERIAL))

.PHONY: compile compile_serial $(COMPILEDIRS) $(COMPILEDIRS_SERIAL)

compile: $(COMPILEDIRS)
	$(MAKE) -l 0.0 compile_serial

$(COMPILEDIRS): %.compile:
	$(MAKE) -C $* compile

#Special target for memory-heavy compilations, so they are serialized
compile_serial:
	$(MAKE) -l 0.0 $(COMPILEDIRS_SERIAL)

$(COMPILEDIRS_SERIAL): %.compile_serial:
	$(MAKE) -C $* compile_serial

#
## clean related rules
#
SUBCLEAN:= $(addsuffix .clean,$(SUBDIRS))
.PHONY: $(SUBCLEAN)

clean: $(SUBCLEAN)

$(SUBCLEAN): %.clean:
	$(MAKE) -C $* clean
