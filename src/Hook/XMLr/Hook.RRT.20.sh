#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.RRT.20 -V
cd ./Hook/
../pmpl -f Hook.RRT.20.xml >& Hook.RRT.20.log
