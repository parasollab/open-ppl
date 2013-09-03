#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N 2DHeterogeneous.Toggle.21 -V
cd ./2DHeterogeneous/
../pmpl -f 2DHeterogeneous.Toggle.21.xml >& 2DHeterogeneous.Toggle.21.log
