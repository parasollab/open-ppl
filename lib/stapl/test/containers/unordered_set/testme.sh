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

eval $run_command ./unordered_set 100
if test $? != 0
then
    echo "ERROR:: while testing unordered_set"
    exit
fi

eval $run_command ./unordered_set_algos 100
if test $? != 0
then
    echo "ERROR:: while testing unordered_set_algos"
    exit
fi
