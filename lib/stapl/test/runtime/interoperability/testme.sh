#!/bin/sh

# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

run_command=$1
TEST_OUTPUT_PREFIX='(STAPL RTS interoperability)'

eval $run_command ./test_external_call_mpi
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_external_call_mpi - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_external_call_mpi - [passed]"
fi
