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

eval $run_command ./unordered_map 100 0
if test $? != 0
then
    echo "ERROR:: while testing unordered_map case 0"
fi

eval $run_command ./unordered_map 100 1
if test $? != 0
then
    echo "ERROR:: while testing unordered_map case 1"
fi

eval $run_command ./unordered_map 100 2
if test $? != 0
then
    echo "ERROR:: while testing unordered_map case 2"
fi

eval $run_command ./unordered_map 100 3
if test $? != 0
then
    echo "ERROR:: while testing unordered_map case 3"
fi

eval $run_command ./unordered_map_algos 100
if test $? != 0
then
    echo "ERROR:: while testing unordered_map_algos"
fi

eval $run_command ./unordered_map_test
if test $? != 0
then
    echo "ERROR:: while testing const correctness in unordered_map_test"
fi

eval $run_command ./unordered_map_viewbased_dist
if test $? != 0
then
    echo "ERROR:: while testing unordered_map_viewbased_dist"
fi
