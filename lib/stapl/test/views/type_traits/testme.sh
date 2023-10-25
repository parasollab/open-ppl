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

eval $run_command ./test_is_view
if test $? != 0
then
    echo "ERROR:: while testing test_is_view"
fi

eval $run_command ./test_dimension_traits
if test $? != 0
then
    echo "ERROR:: while testing test_dimension_traits"
fi
