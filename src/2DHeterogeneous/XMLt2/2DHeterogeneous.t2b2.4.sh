#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N 2DHeterogeneous.t2b2.4 -V
cd ./2DHeterogeneous/
../pmpl -f 2DHeterogeneous.t2b2.4.xml >& 2DHeterogeneous.t2b2.4.log
