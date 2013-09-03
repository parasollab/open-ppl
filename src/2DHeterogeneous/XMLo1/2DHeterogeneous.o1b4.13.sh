#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N 2DHeterogeneous.o1b4.13 -V
cd ./2DHeterogeneous/
../pmpl -f 2DHeterogeneous.o1b4.13.xml >& 2DHeterogeneous.o1b4.13.log
