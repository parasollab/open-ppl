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

nested="[4]"

eval $run_command ./pg_p2p_2level_hom_dim_2D 2 2
if test $? != 0
then
    echo "ERROR:: while testing pg_p2p_dataflow_2level_2D"
fi

eval $run_command ./pg_p2p_3level_hom_dim_2D 2 2 2
if test $? != 0
then
    echo "ERROR:: while testing pg_p2p_dataflow_3level_2D"
fi

eval $run_command ./pg_p2p_4level_hom_dim_2D 2 2 2 2
if test $? != 0
then
    echo "ERROR:: while testing pg_p2p_dataflow_4level_2D"
fi

eval $run_command ./pg_p2p_2level_hom_dim_3D 2 2
if test $? != 0
then
    echo "ERROR:: while testing pg_p2p_dataflow_2level_3D"
fi

eval $run_command ./pg_p2p_3level_hom_dim_3D 2 2 2
if test $? != 0
then
    echo "ERROR:: while testing pg_p2p_dataflow_3level_3D"
fi

eval $run_command ./pg_p2p_4level_hom_dim_3D 2 2 2 2
if test $? != 0
then
    echo "ERROR:: while testing pg_p2p_dataflow_4level_3D"
fi

if [[ $run_command =~ $nested ]]
then
  eval $run_command ./pg_p2p_nlevel_hom_dim_2D 16 1 1 2,1 1,2
  if test $? != 0
  then
      echo "ERROR:: while testing pg_p2p_dataflow_nlevel_2D"
  fi
fi

if [[ $run_command =~ $nested ]]
then
  eval $run_command ./pg_p2p_nlevel_hom_dim_3D 16 1 1 2,1,1 1,1,2
  if test $? != 0
  then
      echo "ERROR:: while testing pg_p2p_dataflow_nlevel_3D"
  fi
fi

eval $run_command ./pg_p2p_2level_het_dim 2 2
if test $? != 0
then
    echo "ERROR:: while testing pg_p2p_2level_het_dim"
fi

eval $run_command ./pg_p2p_3level_het_dim 2 2 2
if test $? != 0
then
    echo "ERROR:: while testing pg_p2p_3level_het_dim"
fi

eval $run_command ./pg_p2p_4level_het_dim 2 2 2 2
if test $? != 0
then
    echo "ERROR:: while testing pg_p2p_4level_het_dim"
fi

eval $run_command ./pg_p2p_dataflow_coarsened 4 10
if test $? != 0
then
    echo "ERROR:: while testing pg_p2p_dataflow_coarsened"
fi

