#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N 2DHeterogeneous.t1b3.32 -V
cd ./2DHeterogeneous/
../pmpl -f 2DHeterogeneous.t1b3.32.xml >& 2DHeterogeneous.t1b3.32.log
