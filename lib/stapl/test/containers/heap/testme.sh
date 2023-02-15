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

eval $run_command ./seq_heap 10000
if test $? != 0
then
    echo "ERROR:: while testing seq_heap"
fi

eval $run_command ./bc_heap 10000
if test $? != 0
then
    echo "ERROR:: while testing bc_heap"
fi

eval $run_command ./cm_heap 10000
if test $? != 0
then
    echo "ERROR:: while testing cm_heap"
fi

eval $run_command ./dis_heap 10000
if test $? != 0
then
    echo "ERROR:: while testing dis_heap"
fi

eval $run_command ./heap 10000
if test $? != 0
then
    echo "ERROR:: while testing heap"
fi

eval $run_command ./heap_view 10000
if test $? != 0
then
    echo "ERROR:: while testing view_heap"
fi

eval $run_command ./pop_top 10000 2
if test $? != 0
then
    echo "ERROR:: while testing pop_top"
fi

