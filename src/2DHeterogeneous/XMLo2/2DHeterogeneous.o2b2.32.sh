#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N 2DHeterogeneous.o2b2.32 -V
cd ./2DHeterogeneous/
../pmpl -f 2DHeterogeneous.o2b2.32.xml >& 2DHeterogeneous.o2b2.32.log
