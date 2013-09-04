#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.SRT.19 -V
cd ./Hook/
../pmpl -f Hook.SRT.19.xml >& Hook.SRT.19.log
