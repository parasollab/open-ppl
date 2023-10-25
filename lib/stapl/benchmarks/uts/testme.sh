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

. ./sample_trees.sh
echo "$run_command ./stapl_workstealing $T1"
eval $run_command ./stapl_workstealing $T1

if test $? != 0
then
    echo "ERROR:: while testing UTS benchmark"
fi
