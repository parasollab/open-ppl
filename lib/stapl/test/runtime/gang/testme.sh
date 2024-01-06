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
TEST_OUTPUT_PREFIX='(STAPL RTS gang)'

eval $run_command ./test_gang
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_gang - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_gang - [passed]"
fi

eval $run_command ./test_gang_switch
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_gang_switch - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_gang_switch - [passed]"
fi

eval $run_command ./test_gang_comm
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_gang_comm - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_gang_comm - [passed]"
fi

eval $run_command ./test_nested_gangs
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_nested_gangs - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_nested_gangs - [passed]"
fi

eval $run_command ./test_intragang
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_intragang - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_intragang - [passed]"
fi

eval $run_command ./test_intergang
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_intergang - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_intergang - [passed]"
fi

eval $run_command ./test_lss
if test $? != 0
then
  echo $TEST_OUTPUT_PREFIX" test_lss - [FAILED]"
else
  echo $TEST_OUTPUT_PREFIX" test_lss - [passed]"
fi
