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
echo $run_command

eval $run_command ./test_do_once 100
if test $? != 0
then
    echo "ERROR:: while testing test_do_once"
fi

eval $run_command ./test_stream
if test $? != 0
then
    echo "ERROR:: while testing test_stream"
fi

eval $run_command ./expand_and_copy
if test $? != 0
then
    echo "ERROR:: while testing test_expand_and_copy"
fi

eval $run_command ./test_unravel
if test $? != 0
then
    echo "ERROR:: while testing test_unravel"
fi

eval $run_command ./for_each_range
if test $? != 0
then
    echo "ERROR:: while testing for_each_range"
fi

eval $run_command ./perf_for_each_range 5000
if test $? != 0
then
    echo "ERROR:: while testing perf_for_each_range"
fi

eval diff tiny_factors.zout ../rel_alpha/tiny_factors.zin
if test $? != 0
then
    echo "ERROR:: output of test_stream is incorrect."
fi

eval diff tiny_factors_getline.zout ../rel_alpha/tiny_factors.zin
if test $? != 0
then
    echo "ERROR:: output of test_stream is incorrect."
fi

eval xxd -p sequence.new > new.txt
eval xxd -p sequence.bin > bin.txt
eval diff bin.txt new.txt
if test $? != 0
then
    echo "ERROR:: output of test_stream write is incorrect."
fi

eval $run_command ./test_type_printer
if test $? != 0
then
    echo "ERROR:: while testing test_type_printer"
fi

eval $run_command ./test_random
if test $? != 0
then
    echo "ERROR:: while testing test_random"
fi
