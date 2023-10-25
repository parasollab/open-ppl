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

eval $run_command ./mesh 53 171
if test $? != 0
then
    echo "ERROR:: while testing mesh"
fi

eval $run_command ./star 8129 1
if test $? != 0
then
    echo "ERROR:: while testing star"
fi

eval $run_command ./grid 53 171
if test $? != 0
then
    echo "ERROR:: while testing grid"
fi

eval $run_command ./torus 53 171
if test $? != 0
then
    echo "ERROR:: while testing torus"
fi

eval $run_command ./torus_3D 53 17 19
if test $? != 0
then
    echo "ERROR:: while testing torus_3D"
fi

eval $run_command ./lollipop 53 171
if test $? != 0
then
    echo "ERROR:: while testing lollipop"
fi

eval $run_command ./list 10057
if test $? != 0
then
    echo "ERROR:: while testing list"
fi

eval $run_command ./complete 105
if test $? != 0
then
    echo "ERROR:: while testing complete"
fi

eval $run_command ./disjointed_complete 16 64
if test $? != 0
then
    echo "ERROR:: while testing disjointed_complete"
fi

eval $run_command ./cycles 43 41
if test $? != 0
then
    echo "ERROR:: while testing cycles"
fi

eval $run_command ./watts_strogatz 1024 8 0.35
if test $? != 0
then
    echo "ERROR:: while testing watts_strogatz"
fi

eval $run_command ./random_neighborhood 1297 37 171
if test $? != 0
then
    echo "ERROR:: while testing random_neighborhood"
fi

eval $run_command ./erdos_renyi 123 .33
if test $? != 0
then
    echo "ERROR:: while testing erdos_renyi"
fi

eval $run_command ./binary_tree
if test $? != 0
then
    echo "ERROR:: while testing ./binary_tree"
fi

eval $run_command ./binary_tree_network
if test $? != 0
then
    echo "ERROR:: while testing ./binary_tree_network"
fi

eval $run_command ./transform_edge_list
if test $? != 0
then
    echo "ERROR:: while testing ./transform_edge_list"
fi

eval $run_command ./barabasi_albert 179 11
if test $? != 0
then
    echo "ERROR:: while testing ./barabasi_albert"
fi

eval $run_command ./random_dag 64 4
if test $? != 0
then
    echo "ERROR:: while testing ./random_dag"
fi

eval $run_command ./sparse_mesh 64 64 0.4
if test $? != 0
then
    echo "ERROR:: while testing ./sparse_mesh"
fi
