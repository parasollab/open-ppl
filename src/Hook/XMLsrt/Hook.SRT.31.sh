#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.SRT.31 -V
cd ./Hook/
../pmpl -f Hook.SRT.31.xml >& Hook.SRT.31.log
