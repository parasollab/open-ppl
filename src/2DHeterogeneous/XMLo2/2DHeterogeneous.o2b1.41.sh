#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N 2DHeterogeneous.o2b1.41 -V
cd ./2DHeterogeneous/
../pmpl -f 2DHeterogeneous.o2b1.41.xml >& 2DHeterogeneous.o2b1.41.log
