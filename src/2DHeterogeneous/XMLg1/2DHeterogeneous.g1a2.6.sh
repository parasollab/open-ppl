#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N 2DHeterogeneous.g1a2.6 -V
cd ./2DHeterogeneous/
../pmpl -f 2DHeterogeneous.g1a2.6.xml >& 2DHeterogeneous.g1a2.6.log
