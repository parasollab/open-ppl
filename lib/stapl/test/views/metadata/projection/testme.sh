#!/bin/bash

# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

trap 'prev=$cur; cur=$BASH_COMMAND' DEBUG

function test_fail()
{
  if test $? != 0
  then
      echo "ERROR:: while testing $prev"
  fi
}

run_command=$1

eval $run_command ./geometry && test_fail
eval $run_command ./grid_graph && test_fail
eval $run_command ./projected_container 192 && test_fail

for n in 45 117 128
do
  eval $run_command ./nd_strided $n 2 && test_fail
  eval $run_command ./invertible $n && test_fail
done
