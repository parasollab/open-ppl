#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N 2DHeterogeneous.Gauss.10 -V
cd ./2DHeterogeneous/
../pmpl -f 2DHeterogeneous.Gauss.10.xml >& 2DHeterogeneous.Gauss.10.log
