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
TEST_OUTPUT_PREFIX='(STAPL RTS executor)'

eval $run_command ./test_no_executor
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_no_executor - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_no_executor - [passed]"
fi

eval $run_command ./test_defer_executor
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_defer_executor - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_defer_executor - [passed]"
fi

eval $run_command ./test_dummy_task
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_dummy_task - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_dummy_task - [passed]"
fi

eval $run_command ./test_multiple_executors
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_multiple_executors - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_multiple_executors - [passed]"
fi

eval $run_command ./test_set_executor_scheduler
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_set_executor_scheduler - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_set_executor_scheduler - [passed]"
fi

eval $run_command ./test_generic_terminator
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_generic_terminator - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_generic_terminator - [passed]"
fi

