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

eval $run_command ./vector 1000
if test $? != 0
then
    echo "ERROR:: while testing vector"
fi

grep error: const_iterator_write > /dev/null 2>&1
if test $? != 0
then
    echo "ERROR:: vector/const_iterator_write.cc compiled without error"
else
    echo "Testing vector::const_iterator write : PASSED"
fi

eval $run_command ./algorithms 1000
if test $? != 0
then
    echo "ERROR:: while testing algorithms"
fi

eval $run_command ./vector_viewbased_dist
if test $? != 0
then
    echo "ERROR:: while testing viewbased distributions of vector"
fi

eval $run_command ./vector_push_back_test
if test $? != 0
then
    echo "ERROR:: while testing vector_push_back_test"
fi

eval $run_command ./vector_redistribution
if test $? != 0
then
    echo "ERROR:: while testing redistribution of vector"
fi

eval $run_command ./vector_test
if test $? != 0
then
    echo "ERROR:: while testing const correctness in vector_test"
fi

