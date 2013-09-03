#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.RRT.14 -V
cd ./Hook/
../pmpl -f Hook.RRT.14.xml >& Hook.RRT.14.log
