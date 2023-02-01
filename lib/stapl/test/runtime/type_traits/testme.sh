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
TEST_OUTPUT_PREFIX='(STAPL RTS type traits)'

eval ./test_aligned_storage $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_aligned_storage - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_aligned_storage - [passed]"
fi

eval ./test_has_define_type $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_has_define_type - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_has_define_type - [passed]"
fi

eval ./test_is_basic $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_is_basic - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_is_basic - [passed]"
fi

eval ./test_is_non_commutative $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_is_non_commutative - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_is_non_commutative - [passed]"
fi

eval ./test_is_p_object $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_is_p_object - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_is_p_object - [passed]"
fi

eval ./test_is_reference_wrapper $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_is_reference_wrapper - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_is_reference_wrapper - [passed]"
fi

eval ./test_is_shared_ptr $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_is_shared_ptr - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_is_shared_ptr - [passed]"
fi

eval ./test_lazy_storage $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_lazy_storage - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_lazy_storage - [passed]"
fi

eval ./test_supports_stapl_packing $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_supports_stapl_packing - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_supports_stapl_packing - [passed]"
fi

eval ./test_type_id $TEST_OPTIONS
if test $? != 0
then
    echo $TEST_OUTPUT_PREFIX" test_type_id - [FAILED]"
else
    echo $TEST_OUTPUT_PREFIX" test_type_id - [passed]"
fi
