#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.SRT.20 -V
cd ./Hook/
../pmpl -f Hook.SRT.20.xml >& Hook.SRT.20.log
