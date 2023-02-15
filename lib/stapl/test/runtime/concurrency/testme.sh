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
TEST_OUTPUT_PREFIX='(STAPL RTS concurrency)'

eval ./test_fuzzy_barrier $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_fuzzy_barrier - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_fuzzy_barrier - [passed]"
fi

eval ./test_reduction $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_reduction - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_reduction - [passed]"
fi

eval ./test_thread_local_storage $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_thread_local_storage - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_thread_local_storage - [passed]"
fi

eval ./test_queue $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_queue - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_queue - [passed]"
fi

eval ./test_intrusive_mpsc_queue $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_intrusive_mpsc_queue - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_intrusive_mpsc_queue - [passed]"
fi

eval ./test_task_queue $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_task_queue - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_task_queue - [passed]"
fi
