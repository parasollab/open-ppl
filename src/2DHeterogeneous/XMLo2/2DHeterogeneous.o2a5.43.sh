#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N 2DHeterogeneous.o2a5.43 -V
cd ./2DHeterogeneous/
../pmpl -f 2DHeterogeneous.o2a5.43.xml >& 2DHeterogeneous.o2a5.43.log
