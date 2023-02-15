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

reg="[1]"

# Sequential tests
if [[ $run_command =~ $reg ]]
then
  eval $run_command ./test_multiarray_slice 8 8
  if test $? != 0
  then
      echo "ERROR:: while testing test_multiarray_slice"
  fi
fi

eval $run_command ./test_slices_view 24 8 4
if test $? != 0
then
    echo "ERROR:: while testing slices view"
fi

eval $run_command ./test_composed_slices_algo 5 7
if test $? != 0
then
    echo "ERROR:: while testing composed slices algo"
fi

eval $run_command ./test_sliced_extended_view 16
if test $? != 0
then
    echo "ERROR:: while testing sliced extended view"
fi

eval $run_command ./test_composed_slices_view 4
if test $? != 0
then
    echo "ERROR:: while testing composed slices view"
fi

eval $run_command ./test_deep_slicing 8 8
if test $? != 0
then
    echo "ERROR:: while testing deep slicing"
fi

eval $run_command ./test_sliced_sliced 277
if test $? != 0
then
    echo "ERROR:: while testing deep slicing"
fi

eval $run_command ./test_linearizer
if test $? != 0
then
    echo "ERROR:: while testing linearizer"
fi
