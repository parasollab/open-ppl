#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N 2DHeterogeneous.t1a3.44 -V
cd ./2DHeterogeneous/
../pmpl -f 2DHeterogeneous.t1a3.44.xml >& 2DHeterogeneous.t1a3.44.log
