#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N 2DHeterogeneous.SRT.14 -V
cd ./2DHeterogeneous/
../pmpl -f 2DHeterogeneous.SRT.14.xml >& 2DHeterogeneous.SRT.14.log
