#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N 2DHeterogeneous.t1a2.2 -V
cd ./2DHeterogeneous/
../pmpl -f 2DHeterogeneous.t1a2.2.xml >& 2DHeterogeneous.t1a2.2.log
