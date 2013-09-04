#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.SRT.0 -V
cd ./Hook/
../pmpl -f Hook.SRT.0.xml >& Hook.SRT.0.log
