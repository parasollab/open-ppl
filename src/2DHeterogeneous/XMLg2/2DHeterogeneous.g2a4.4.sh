#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N 2DHeterogeneous.g2a4.4 -V
cd ./2DHeterogeneous/
../pmpl -f 2DHeterogeneous.g2a4.4.xml >& 2DHeterogeneous.g2a4.4.log
