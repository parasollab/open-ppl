#!/bin/bash

# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

run_command=$1

eval $run_command ./test1 3 4 5 6 7
if test $? != 0
then
    echo "ERROR:: while testing composed slices (sliced distribution)"\
"+ skeletons (1-1-1-2)"
fi

eval $run_command ./test2 3 4 5 6 7
if test $? != 0
then
    echo "ERROR:: while testing composed slices (balanced distribution)"\
"+ skeletons (1-1-1-2)"
fi

eval $run_command ./test3 3 4 5 6 7
if test $? != 0
then
    echo "ERROR:: while testing composed slices (sliced distribution)"\
"+ skeletons (1-1-2-1)"
fi

eval $run_command ./test4 3 4 5 6 7
if test $? != 0
then
    echo "ERROR:: while testing composed slices (balanced distribution)"\
"+ skeletons (1-1-2-1)"
fi

eval $run_command ./test5 3 4 5 6 7
if test $? != 0
then
    echo "ERROR:: while testing composed slices (sliced distribution)"\
"+ skeletons (1-2-1-1)"
fi

eval $run_command ./test6 3 4 5 6 7
if test $? != 0
then
    echo "ERROR:: while testing composed slices (balanced distribution)"\
"+ skeletons (1-2-1-1)"
fi

eval $run_command ./test7 3 4 5 6 7
if test $? != 0
then
    echo "ERROR:: while testing composed slices (sliced distribution)"\
"+ skeletons (2-1-1-1)"
fi

eval $run_command ./test8 3 4 5 6 7
if test $? != 0
then
    echo "ERROR:: while testing composed slices (balanced distribution)"\
"+ skeletons (2-1-1-1)"
fi

