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

reg="[1,2,4]"

eval $run_command ./is_skeletons S
if test $? != 0
then
    echo "ERROR:: while testing NAS IS (Skeleton-based)"
fi

eval $run_command ./ep s1
if test $? != 0
then
    echo "ERROR:: while testing NAS EP (Skeleton-based)"
fi

eval $run_command ./ft S
if test $? != 0
then
    echo "ERROR:: while testing NAS FT (Skeleton-based)"
fi
