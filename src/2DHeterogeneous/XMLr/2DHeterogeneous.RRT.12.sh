#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N 2DHeterogeneous.RRT.12 -V
cd ./2DHeterogeneous/
../pmpl -f 2DHeterogeneous.RRT.12.xml >& 2DHeterogeneous.RRT.12.log
