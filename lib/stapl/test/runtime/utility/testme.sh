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
TEST_OUTPUT_PREFIX='(STAPL RTS utility)'

eval ./test_algorithms $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_algorithms - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_algorithms - [passed]"
fi

eval ./test_comparable_proxy $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_comparable_proxy - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_comparable_proxy - [passed]"
fi

eval ./test_option $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_option - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_option - [passed]"
fi

eval ./test_pool $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_pool - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_pool - [passed]"
fi

eval ./test_timer $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_timer - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_timer - [passed]"
fi

eval ./test_tree $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_tree - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_tree - [passed]"
fi

eval ./test_block_registry $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_block_registry - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_block_registry - [passed]"
fi

