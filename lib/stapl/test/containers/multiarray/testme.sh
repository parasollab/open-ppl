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
echo $run_command

eval $run_command ./multiarray 16 16 16
if test $? != 0
then
    echo "ERROR:: while testing test_pmultiarray"
fi

eval $run_command ./algorithm 16 16 16
if test $? != 0
then
    echo "ERROR:: while testing multiarray algorithm"
fi

eval $run_command ./multiarray_nd 8
if test $? != 0
then
    echo "ERROR:: while testing multiarray_nd"
fi

eval $run_command ./multiarray_nested
if test $? != 0
then
    echo "ERROR:: while testing multiarray_nested"
fi

eval $run_command ./multiarray_viewbased_dist
if test $? != 0
then
    echo "ERROR:: while testing multiarray_viewbased_dist"
fi

eval $run_command ./multiarray_test
if test $? != 0
then
    echo "ERROR:: while testing const correctness in multiarray_test"
fi

eval $run_command ./algorithm_multiview 8 8 8
if test $? != 0
then
    echo "ERROR:: while testing multiarray view alignment"
fi

eval $run_command ./stencil 32
if test $? != 0
then
    echo "ERROR:: while testing stencil"
fi

