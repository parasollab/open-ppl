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

eval $run_command ./base_container_range
if test $? != 0
then
    echo "ERROR:: while testing base_container_range"
fi

eval $run_command ./default_coarsener 117
if test $? != 0
then
    echo "ERROR:: while testing default_coarsener"
fi

eval $run_command ./overpartitioned_multiarray 8 8 8
if test $? != 0
then
    echo "ERROR:: while testing overpartitioned_multiarray"
fi

for i in 16 100 117 45
do
  eval $run_command ./matrix_views $i
  if test $? != 0
  then
      echo "ERROR:: while testing matrix_views"
  fi

  eval $run_command ./strided_view $i
  if test $? != 0
  then
      echo "ERROR:: while testing default_coarsener"
  fi
done
