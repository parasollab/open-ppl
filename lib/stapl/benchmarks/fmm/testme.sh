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

if [[ $run_command =~ $reg ]]
then
  eval $run_command ./fmm
  if test $? != 0
  then
      echo "ERROR:: while testing FMM (Skeleton-based)"
  fi
fi

eval $run_command ./fmm
if test $? != 0
then
    echo "ERROR:: while testing FMM (Skeleton-based)"
fi

eval $run_command ./fmm_define_dag
if test $? != 0
then
    echo "ERROR:: while testing FMM (define_dag and zip fusion based)"
fi


