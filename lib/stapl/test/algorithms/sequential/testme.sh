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
TOTAL_TESTS=9
PASSED_TEST=0

echo "-------- Testing `pwd` -------------"

for binary in "test_n_partition"; do
  echo "================================================"
  echo "Running ./$binary Using $run_command"
  echo "================================================"
  eval $run_command ./$binary
  if [ $? -ne 0 ]; then
    echo "ERROR:: $@ - $* exited abnormally"
  else
    echo "PASS:: $@ - $* exited normally"
    PASSED_TEST=`expr $PASSED_TEST + 1`;
  fi
  for d in "-d 1000" "-d 1"; do
    for s in "-s 999" "-s 256" "-s 1000" "-s 1"; do
      ARGS="$d $s"
      echo "================================================"
      echo "Running ./$binary $ARGS Using $run_command Processors"
      echo "================================================"
      eval $run_command ./$binary $ARGS
      if [ $? -ne 0 ]; then
        echo "ERROR:: $@ - $* exited abnormally"
      else
        echo "PASS:: $@ - $* exited normally"
        PASSED_TEST=`expr $PASSED_TEST + 1`;
      fi
    done
  done
done

echo "----------------------------------------------------------------------"
echo "Tests executed: $PASSED_TEST/$TOTAL_TESTS"

