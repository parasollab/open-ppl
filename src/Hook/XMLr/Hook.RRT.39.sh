#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.RRT.39 -V
cd ./Hook/
../pmpl -f Hook.RRT.39.xml >& Hook.RRT.39.log
