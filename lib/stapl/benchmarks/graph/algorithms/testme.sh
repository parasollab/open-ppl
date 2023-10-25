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

eval $run_command ./abfs --trials 10 --type er --n 100 --prob 0.05 \
  --tau 0.3 --paradigm kla --k 4
if test $? != 0
then
  echo "ERROR:: while testing Approsimate BFS"
fi

for paradigm in "lsync" "async" "kla --k 4" "hier" "hubs --hub_degree 10";
do
  eval $run_command ./pd --trials 10 --type er --n 100 --prob 0.05 \
    --paradigm $paradigm
  if test $? != 0
  then
    echo "ERROR:: while testing Pseudo Diameter $paradigm"
  fi
done

for paradigm in "lsync" "hier" "hubs --hub_degree 10";
do
  eval $run_command ./br --trials 10 --type er --n 100 --prob 0.05 \
    --damping 0.3 --iterations 20 --blacklist_prob 0.1 --paradigm $paradigm
  if test $? != 0
  then
    echo "ERROR:: while testing Bad Rank $paradigm"
  fi
done

for paradigm in "lsync" "async" "kla --k 4" "hier" "hubs --hub_degree 10";
do
  eval $run_command ./cl --trials 10 --type er --n 100 --prob 0.05 \
    --paradigm $paradigm
  if test $? != 0
  then
    echo "ERROR:: while testing Closeness Centrality $paradigm"
  fi
done

eval $run_command ./cu --trials 10 --type er --n 1000 --prob 0.05 \
  --id 0 --paradigm lsync --sets 10
if test $? != 0
then
  echo "ERROR:: while testing Cut Conductance"
fi

for paradigm in "lsync" "async" "kla --k 4"
do
  eval $run_command ./lp --trials 10 --type er --n 100 --prob 0.05 \
    --paradigm $paradigm
  if test $? != 0
  then
    echo "ERROR:: while testing Link Prediction $paradigm"
  fi
done

for paradigm in "lsync" "async" "kla --k 4"
do
  eval $run_command ./rw --trials 10 --type er --n 1000 --prob 0.05 \
    --paradigm $paradigm --path_length 5
  if test $? != 0
  then
    echo "ERROR:: while testing Random Walk $paradigm"
  fi
done

for paradigm in "lsync" "async" "kla --k 4" "hier" "hubs --hub_degree 10";
do
  eval $run_command ./cd --trials 10 --type er --n 100 --prob 0.05 \
    --paradigm $paradigm --iterations 20
  if test $? != 0
  then
    echo "ERROR:: while testing Community Detection $paradigm"
  fi
done

for paradigm in "lsync" "async" "kla --k 4" "hier" "hubs --hub_degree 10";
do
  eval $run_command ./kc --trials 10 --type er --n 100 --prob 0.05 \
  --paradigm $paradigm --core_sz 4
  if test $? != 0
  then
    echo "ERROR:: while testing k_core $paradigm"
  fi
done
