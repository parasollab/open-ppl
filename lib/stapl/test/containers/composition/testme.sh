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

eval $run_command ./min_row 1001 1001
if test $? != 0
then
    echo "ERROR:: while testing min_row"
fi

eval $run_command ./multilevel_composition_3 20
if test $? != 0
then
    echo "ERROR:: while testing multilevel_composition_3"
fi

eval $run_command ./multilevel_composition_4 20
if test $? != 0
then
    echo "ERROR:: while testing multilevel_composition_4"
fi

eval $run_command ./graph_multilevel_composition 20
if test $? != 0
then
    echo "ERROR:: while testing graph_multilevel_composition"
fi

eval $run_command ./vector_multilevel_composition 20
if test $? != 0
then
    echo "ERROR:: while testing vector_multilevel_composition"
fi

eval $run_command ./mixed_multilevel_composition 20
if test $? != 0
then
    echo "ERROR:: while testing mixed_multilevel_composition"
fi

eval $run_command ./array
if test $? != 0
then
    echo "ERROR:: while testing composed array sizing"
fi

eval $run_command ./array_redistribution 8 8 8
if test $? != 0
then
    echo "ERROR:: while testing composed array redistribution"
fi

eval $run_command ./vector_redistribution 8 8 8
if test $? != 0
then
    echo "ERROR:: while testing composed vector redistribtion"
fi

eval $run_command ./multiarray_redistribution 8 8
if test $? != 0
then
    echo "ERROR:: while testing composed multiarray redistribution"
fi
