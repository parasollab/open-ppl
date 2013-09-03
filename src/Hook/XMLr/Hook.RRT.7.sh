#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.RRT.7 -V
cd ./Hook/
../pmpl -f Hook.RRT.7.xml >& Hook.RRT.7.log
