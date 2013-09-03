#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N 2DHeterogeneous.u2b2.8 -V
cd ./2DHeterogeneous/
../pmpl -f 2DHeterogeneous.u2b2.8.xml >& 2DHeterogeneous.u2b2.8.log
