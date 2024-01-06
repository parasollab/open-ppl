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
TOTAL_TESTS=37
PASSED_TEST=0

echo "-------- Testing `pwd` -------------"

eval $run_command ./search 1000 10
if test $? -eq 0
then
    PASSED_TEST=`expr $PASSED_TEST + 1`;
else
    echo "ERROR:: while testing search"
fi

eval $run_command ./search_n
if test $? -eq 0
then
    PASSED_TEST=`expr $PASSED_TEST + 1`;
else
    echo "ERROR:: while testing search_n"
fi

eval $run_command ./find 1234
if test $? -eq 0
then
    PASSED_TEST=`expr $PASSED_TEST + 1`;
else
    echo "ERROR:: while testing find"
fi

eval $run_command ./adjacent_difference 1234
if test $? -eq 0
then
    PASSED_TEST=`expr $PASSED_TEST + 1`;
else
    echo "ERROR:: while testing adjacent_difference"
fi

eval $run_command ./adjacent_find 1234
if test $? -eq 0
then
    PASSED_TEST=`expr $PASSED_TEST + 1`;
else
    echo "ERROR:: while testing adjacent_find"
fi

eval $run_command ./find_first_of 1234
if test $? -eq 0
then
    PASSED_TEST=`expr $PASSED_TEST + 1`;
else
    echo "ERROR:: while testing find_first_of"
fi

eval $run_command ./find_first_of_2 1000 10
if test $? -eq 0
then
    PASSED_TEST=`expr $PASSED_TEST + 1`;
else
    echo "ERROR:: while testing find_first_of_2"
fi

eval $run_command ./test_partition_point
if test $? -eq 0
then
    PASSED_TEST=`expr $PASSED_TEST + 1`;
else
    echo "ERROR:: while testing partition_point"
fi

eval $run_command ./test_is_partitioned
if test $? -eq 0
then
    PASSED_TEST=`expr $PASSED_TEST + 1`;
else
    echo "ERROR:: while testing is_partitioned"
fi

eval $run_command ./lexicographic_compare 1000 2000 1
if test $? -eq 0
then
    PASSED_TEST=`expr $PASSED_TEST + 1`;
else
    echo "ERROR:: while testing lexicographic_compare"
fi

eval $run_command ./test_map_reduce test_map_reduce.input
if test $? -eq 0
then
    PASSED_TEST=`expr $PASSED_TEST + 1`;
else
    echo "ERROR:: while testing test_map_reduce"
fi

eval $run_command ./generate 1234 1
if test $? -eq 0
then
    PASSED_TEST=`expr $PASSED_TEST + 1`;
else
    echo "ERROR:: while testing generate"
fi

eval $run_command ./mismatch 1234
if test $? -eq 0
then
    PASSED_TEST=`expr $PASSED_TEST + 1`;
else
    echo "ERROR:: while testing mismatch"
fi

eval $run_command ./random_shuffle 1234
if test $? -eq 0
then
    PASSED_TEST=`expr $PASSED_TEST + 1`;
else
    echo "ERROR:: while testing random_shuffle"
fi

eval $run_command ./binary_search 1234
if test $? -eq 0
then
    PASSED_TEST=`expr $PASSED_TEST + 1`;
else
    echo "ERROR:: while testing binary_search"
fi

eval $run_command ./find_end
if test $? -eq 0
then
    PASSED_TEST=`expr $PASSED_TEST + 1`;
else
    echo "ERROR:: while testing find_end"
fi


eval $run_command ./rotate_copy
if test $? -eq 0
then
    PASSED_TEST=`expr $PASSED_TEST + 1`;
else
    echo "ERROR:: while testing rotate_copy"
fi

eval $run_command ./min_element
if test $? -eq 0
then
    PASSED_TEST=`expr $PASSED_TEST + 1`;
else
    echo "ERROR:: while testing min_element"
fi

eval $run_command ./partition_copy
if test $? -eq 0
then
    PASSED_TEST=`expr $PASSED_TEST + 1`;
else
    echo "ERROR:: while testing partition_copy"
fi

# merge two disjoint arrays
eval $run_command ./merge 10000 0
if test $? -eq 0
then
    PASSED_TEST=`expr $PASSED_TEST + 1`;
else
    echo "ERROR:: while testing merge (disjoint)"
fi

# merge two interleaved arrays
eval $run_command ./merge 10000 1
if test $? -eq 0
then
    PASSED_TEST=`expr $PASSED_TEST + 1`;
else
    echo "ERROR:: while testing merge (interleaved)"
fi

# merge two offset arrays
eval $run_command ./merge 10000 2
if test $? -eq 0
then
    PASSED_TEST=`expr $PASSED_TEST + 1`;
else
    echo "ERROR:: while testing merge (offset)"
fi

eval $run_command ./minmax 114
if test $? -eq 0
then
    PASSED_TEST=`expr $PASSED_TEST + 1`;
else
    echo "ERROR:: while testing minmax"
fi

eval $run_command ./dynamic_programming
if test $? -eq 0
then
    PASSED_TEST=`expr $PASSED_TEST + 1`;
else
    echo "ERROR:: while testing dynamic_programming"
fi

eval $run_command ./test_stable_sort 1024 16
if test $? -eq 0
then
    PASSED_TEST=`expr $PASSED_TEST + 1`;
else
    echo "ERROR:: while testing stable_sort."
fi

eval $run_command ./test_column_sort 1024
if test $? -eq 0
then
    PASSED_TEST=`expr $PASSED_TEST + 1`;
else
    echo "ERROR:: while testing column_sort."
fi

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
  for d in "-d 1000" "-d 10"; do
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

