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

eval $run_command ./set 1000
if test $? != 0
then
    echo "ERROR:: while testing set"
fi

eval $run_command ./nested_set 1000
if test $? != 0
then
    echo "ERROR:: while testing nested_set"
fi

eval $run_command ./concurrent_insert 100
if test $? != 0
then
    echo "ERROR:: while testing concurrent_insert"
fi

eval $run_command ./set_enumerable 100
if test $? != 0
then
    echo "ERROR:: while testing set_enumerable"
fi

eval $run_command ./set_algos 100
if test $? != 0
then
    echo "ERROR:: while testing set_algos"
fi

eval $run_command ./set_test
if test $? != 0
then
    echo "ERROR:: while testing const correctness in set_test"
fi

eval $run_command ./set_redistribution
if test $? != 0
then
    echo "ERROR:: while testing redistribution of set"
fi
