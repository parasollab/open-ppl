#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N 2DHeterogeneous.g2b1.46 -V
cd ./2DHeterogeneous/
../pmpl -f 2DHeterogeneous.g2b1.46.xml >& 2DHeterogeneous.g2b1.46.log
