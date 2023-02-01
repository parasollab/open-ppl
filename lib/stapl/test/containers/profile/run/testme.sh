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

eval $run_command ./test_view_composition_builder
if test $? != 0
then
    echo "ERROR:: while testing the view composition builder."
fi

##NOTE: Uncomment these lines as and when ported, Delete this note when all have
#  been ported.

eval $run_command ./profiler_array 0 10000 --ninvokes 4 --maxiterations 4
if test $? != 0
then
    echo "ERROR:: while testing profiler_array 0"
fi

eval $run_command ./profiler_array 1 10000 --ninvokes 4 --maxiterations 4
if test $? != 0
then
    echo "ERROR:: while testing profiler_array 1"
fi

eval $run_command ./profiler_array 2 10000 --ninvokes 4 --maxiterations 4
if test $? != 0
then
    echo "ERROR:: while testing profiler_array 2"
fi

eval $run_command ./profiler_array 3 10000 --ninvokes 4 --maxiterations 4
if test $? != 0
then
    echo "ERROR:: while testing profiler_array 3"
fi

eval $run_command ./profiler_array 4 10000 --ninvokes 4 --maxiterations 4
if test $? != 0
then
    echo "ERROR:: while testing profiler_array 4"
fi

eval $run_command ./profiler_array 5 10000 --ninvokes 4 --maxiterations 4
if test $? != 0
then
    echo "ERROR:: while testing profiler_array 5"
fi

eval $run_command ./profiler_array 6 10000 --ninvokes 4 --maxiterations 4
if test $? != 0
then
    echo "ERROR:: while testing profiler_array 6"
fi

eval $run_command ./profiler_array 7 10000 --ninvokes 4 --maxiterations 4
if test $? != 0
then
    echo "ERROR:: while testing profiler_array 7"
fi

eval $run_command ./profiler_vector 0 10000 --ninvokes 4 --maxiterations 4
if test $? != 0
then
    echo "ERROR:: while testing profiler_vector 0"
fi

eval $run_command ./profiler_vector 1 10000 --ninvokes 4 --maxiterations 4
if test $? != 0
then
    echo "ERROR:: while testing profiler_vector 1"
fi

eval $run_command ./profiler_vector 2 10000 --ninvokes 4 --maxiterations 4
if test $? != 0
then
    echo "ERROR:: while testing profiler_vector 2"
fi

eval $run_command ./profiler_vector 3 10000 --ninvokes 4 --maxiterations 4
if test $? != 0
then
    echo "ERROR:: while testing profiler_vector 3"
fi

eval $run_command ./profiler_graph 0 1000 --ninvokes 4 --maxiterations 4
if test $? != 0
then
    echo "ERROR:: while testing profiler_graph 0"
fi

eval $run_command ./profiler_graph 1 1000 --ninvokes 4 --maxiterations 4
if test $? != 0
then
    echo "ERROR:: while testing profiler_graph 1"
fi

#eval $run_command ./profiler_assoc 0 1000 --ninvokes 4 --maxiterations 4
#eval $run_command ./profiler_assoc 1 1000 --ninvokes 4 --maxiterations 4
#eval $run_command ./profiler_assoc 2 1000 --ninvokes 4 --maxiterations 4
#eval $run_command ./profiler_assoc 3 1000 --ninvokes 4 --maxiterations 4
#eval $run_command ./profiler_assoc 4 1000 --ninvokes 4 --maxiterations 4
#eval $run_command ./profiler_assoc 5 1000 --ninvokes 4 --maxiterations 4
