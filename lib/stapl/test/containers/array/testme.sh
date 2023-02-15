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

eval $run_command ./array 1000
if test $? != 0
then
    echo "ERROR:: while testing array"
fi

grep error: const_iterator_write > /dev/null 2>&1
if test $? != 0
then
    echo "ERROR:: const_iterator_write.cc compiled without error"
else
    echo "Testing const_iterator write : PASSED"
fi

eval $run_command ./static_array 1000
if test $? != 0
then
    echo "ERROR:: while testing static array"
fi

eval $run_command ./algorithms 1000
if test $? != 0
then
    echo "ERROR:: while testing algorithms"
fi

eval $run_command ./array_of_vectors 100
if test $? != 0
then
    echo "ERROR:: while testing algorithms"
fi

eval $run_command ./array_viewbased_dist
if test $? != 0
then
    echo "ERROR:: while testing viewbased distributions of array"
fi

eval $run_command ./row_dist_arrays 100 100
if test $? != 0
then
    echo "ERROR:: while testing array distributions with explicit location sets"
fi

eval $run_command ./array_redistribution
if test $? != 0
then
    echo "ERROR:: while testing redistribution of array"
fi

eval $run_command ./array_test
if test $? != 0
then
    echo "ERROR:: while testing const correctness in array_test"
fi

#eval $run_command ./composed 1000 100
#if test $? != 0
#then
#    echo "ERROR:: while testing composed"
#fi

#eval $run_command ./migrate 1000 100
#if test $? != 0
#then
#    echo "ERROR:: while testing migrate"
#fi
