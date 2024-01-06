#!/bin/sh

# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

TEST_OPTIONS='--report_level=no --log_level=nothing'
TEST_OUTPUT_PREFIX='(STAPL RTS system)'

eval ./test_demangle $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_demangle - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_demangle - [passed]"
fi

eval ./test_getpid $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_getpid - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_getpid - [passed]"
fi

eval ./test_memory $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_memory - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_memory - [passed]"
fi

