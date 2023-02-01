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

eval $run_command ./map 1000
if test $? != 0
then
    echo "ERROR:: while testing map"
fi

eval $run_command ./concurrent_insert 100
if test $? != 0
then
    echo "ERROR:: while testing concurrent_insert"
fi

eval $run_command ./map_continuous 100
if test $? != 0
then
    echo "ERROR:: while testing map_continuous"
fi

eval $run_command ./map_enumerable 100
if test $? != 0
then
    echo "ERROR:: while testing map_enumerable"
fi

eval $run_command ./map_algos 100
if test $? != 0
then
    echo "ERROR:: while testing map_algos"
fi

eval $run_command ./map_viewbased_dist
if test $? != 0
then
    echo "ERROR:: while testing viewbased distributions of map"
fi

eval $run_command ./map_update_dist
if test $? != 0
then
    echo "ERROR:: while testing update of viewbased distributions of map"
fi

eval $run_command ./map_test
if test $? != 0
then
    echo "ERROR:: while testing const correctness in map_test"
fi

eval $run_command ./map_redistribution
if test $? != 0
then
    echo "ERROR:: while testing redistribution of map"
fi
