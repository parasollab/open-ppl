#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N 2DHeterogeneous.u1b2.7 -V
cd ./2DHeterogeneous/
../pmpl -f 2DHeterogeneous.u1b2.7.xml >& 2DHeterogeneous.u1b2.7.log
