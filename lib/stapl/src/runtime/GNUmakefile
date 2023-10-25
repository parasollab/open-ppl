# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.


ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/src/runtime,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

default: compile

compile: abort.o collective.o communicator.o concurrency.o context.o \
 executor.o exit_code.o external.o gang.o gang_description.o \
 gang_md_registry.o heap_tracker.o instrumentation.o location.o main.o \
 message.o new.o p_object_registry.o primitives.o rmi_handle.o runqueue.o \
 serialization.o synchronization.o system.o

ifneq ($(platform), clang)
ifneq ($(platform), LINUX_icc)
runqueue.o: CXXFLAGS+=-Wno-return-local-addr
runqueue.o: runqueue.cc
endif
endif

.PHONY: clean
clean:
	$(RM) *.o
