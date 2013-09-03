#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.RRT.4 -V
cd ./Hook/
../pmpl -f Hook.RRT.4.xml >& Hook.RRT.4.log
