#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N 2DHeterogeneous.u2a6.34 -V
cd ./2DHeterogeneous/
../pmpl -f 2DHeterogeneous.u2a6.34.xml >& 2DHeterogeneous.u2a6.34.log
