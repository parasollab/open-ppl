#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N 2DHeterogeneous.o2b4.47 -V
cd ./2DHeterogeneous/
../pmpl -f 2DHeterogeneous.o2b4.47.xml >& 2DHeterogeneous.o2b4.47.log
