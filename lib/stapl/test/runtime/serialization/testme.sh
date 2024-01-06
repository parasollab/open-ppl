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
TEST_OUTPUT_PREFIX='(STAPL RTS serialization)'

eval ./test_typer_traits $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_typer_traits - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_typer_traits - [passed]"
fi

eval ./test_boost_serialization $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_boost_serialization - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_boost_serialization - [passed]"
fi

eval ./test_arg_storage $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_arg_storage - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_arg_storage - [passed]"
fi

eval ./test_stl_containers $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_stl_containers - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_stl_containers - [passed]"
fi

eval ./test_boost_containers $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_boost_containers - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_boost_containers - [passed]"
fi

eval ./test_string $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_string - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_string - [passed]"
fi

eval ./test_function $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_function - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_function - [passed]"
fi

eval ./test_smart_ptr $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_smart_ptr - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_smart_ptr - [passed]"
fi
