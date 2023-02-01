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
PASSED_TEST=0

echo "-------- Testing `pwd` -------------"

for binary in "test_non_mutating" "test_mutating" "test_numeric" "test_sorting";
do
  echo "================================================"
  echo "Running ./$binary Using $run_command"
  echo "================================================"
  eval $run_command ./$binary
  if [ $? -ne 0 ]; then
    echo "ERROR:: $@ - $* exited abnormally"
  else
    echo "PASS:: $@ - $* exited normally"
  fi
  for cpart1 in "-cpart1 balanced" "-cpart1 block 25" "-cpart1 block 32";
  do
    for cpart2 in "-cpart2 block 32";
    do
      ARGS="$cpart1 $cpart2"
      echo "================================================"
      echo "Running ./$binary $ARGS Using $run_command Processors"
      echo "================================================"
      eval $run_command ./$binary $ARGS
      if [ $? -ne 0 ]; then
        echo "ERROR:: $@ - $* exited abnormally"
      else
        echo "PASS:: $@ - $* exited normally"
      fi
    done
  done
done
