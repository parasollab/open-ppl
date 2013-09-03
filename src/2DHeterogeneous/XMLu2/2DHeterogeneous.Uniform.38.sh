#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N 2DHeterogeneous.Uniform.38 -V
cd ./2DHeterogeneous/
../pmpl -f 2DHeterogeneous.Uniform.38.xml >& 2DHeterogeneous.Uniform.38.log
